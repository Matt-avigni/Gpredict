#include <math.h>
#include <string.h>

#include <gio/gio.h>
#include <glib.h>

#include "rigctld_client.h"
#include "rotctld_client.h"
#include "radio-conf.h"

#define MOCK_CONNECT_RETRIES 20
#define MOCK_CONNECT_DELAY_US 100000

static gchar *find_python(void)
{
    gchar *python = g_find_program_in_path("python3");
    if (python != NULL)
        return python;
    return g_find_program_in_path("python");
}

static guint16 pick_free_port(void)
{
    GSocketListener *listener = g_socket_listener_new();
    GError *error = NULL;
    guint16 port = 0;

    port = g_socket_listener_add_any_inet_port(listener, NULL, &error);
    if (port == 0)
    {
        g_printerr("failed to pick free port: %s\n",
                   error ? error->message : "unknown");
        g_clear_error(&error);
    }

    g_object_unref(listener);
    return port;
}

static GSubprocess *spawn_mock(const gchar *python,
                               const gchar *script,
                               guint16 port)
{
    GSubprocess *proc = NULL;
    GError *error = NULL;
    gchar port_str[16];

    g_snprintf(port_str, sizeof(port_str), "%u", port);

    proc = g_subprocess_new(G_SUBPROCESS_FLAGS_STDOUT_SILENCE |
                                G_SUBPROCESS_FLAGS_STDERR_SILENCE,
                            &error,
                            python,
                            script,
                            "--host", "127.0.0.1",
                            "--port", port_str,
                            "--once",
                            NULL);
    if (proc == NULL)
    {
        g_printerr("failed to spawn %s: %s\n",
                   script, error ? error->message : "unknown");
        g_clear_error(&error);
    }

    return proc;
}

static gboolean connect_rigctld_with_retry(RigctldClient *client,
                                           const gchar *host,
                                           guint16 port)
{
    for (gint i = 0; i < MOCK_CONNECT_RETRIES; i++)
    {
        gchar *error = NULL;
        if (rigctld_client_connect(client, host, port, 200, &error))
        {
            g_free(error);
            return TRUE;
        }
        g_free(error);
        g_usleep(MOCK_CONNECT_DELAY_US);
    }
    return FALSE;
}

static gboolean connect_rotctld_with_retry(RotctldClient *client,
                                           const gchar *host,
                                           guint16 port)
{
    for (gint i = 0; i < MOCK_CONNECT_RETRIES; i++)
    {
        gchar *error = NULL;
        if (rotctld_client_connect(client, host, port, 200, &error))
        {
            g_free(error);
            return TRUE;
        }
        g_free(error);
        g_usleep(MOCK_CONNECT_DELAY_US);
    }
    return FALSE;
}

int main(void)
{
    gchar *python = NULL;
    gchar *rig_script = NULL;
    gchar *rot_script = NULL;
    guint16 rig_port = 0;
    guint16 rot_port = 0;
    GSubprocess *rig_proc = NULL;
    GSubprocess *rot_proc = NULL;
    RigctldClient *rig = NULL;
    RotctldClient *rot = NULL;
    radio_conf_t conf;
    const RigCaps *caps = NULL;
    const RotCaps *rcaps = NULL;
    gdouble freq = 0.0;
    gdouble az = 0.0;
    gdouble el = 0.0;
    GError *error = NULL;
    gboolean ok = TRUE;

    python = find_python();
    if (python == NULL)
    {
        g_printerr("python not found; skipping mock tests\n");
        return 77;
    }

    rig_script = g_build_filename("..", "scripts", "mock_rigctld.py", NULL);
    rot_script = g_build_filename("..", "scripts", "mock_rotctld.py", NULL);

    if (!g_file_test(rig_script, G_FILE_TEST_EXISTS) ||
        !g_file_test(rot_script, G_FILE_TEST_EXISTS))
    {
        g_printerr("mock scripts not found; skipping mock tests\n");
        g_free(python);
        g_free(rig_script);
        g_free(rot_script);
        return 77;
    }

    rig_port = pick_free_port();
    rot_port = pick_free_port();
    for (gint attempt = 0; attempt < 5 && rot_port == rig_port; attempt++)
        rot_port = pick_free_port();
    if (rig_port == 0 || rot_port == 0)
    {
        ok = FALSE;
        goto cleanup;
    }
    if (rot_port == rig_port)
    {
        g_printerr("failed to select distinct mock ports\n");
        ok = FALSE;
        goto cleanup;
    }

    rig_proc = spawn_mock(python, rig_script, rig_port);
    rot_proc = spawn_mock(python, rot_script, rot_port);
    if (rig_proc == NULL || rot_proc == NULL)
    {
        ok = FALSE;
        goto cleanup;
    }

    rig = rigctld_client_new("mock-rig");
    rot = rotctld_client_new("mock-rot");
    if (rig == NULL || rot == NULL)
    {
        ok = FALSE;
        goto cleanup;
    }

    if (!connect_rigctld_with_retry(rig, "127.0.0.1", rig_port))
    {
        g_printerr("failed to connect to rigctld mock\n");
        ok = FALSE;
        goto cleanup;
    }
    memset(&conf, 0, sizeof(conf));
    conf.rigctld_model = 3081;
    conf.radio_mode = RADIO_MODE_FULL_DUPLEX_MAIN_SUB;
    if (!rigctld_client_probe(rig, &conf, 500))
    {
        g_printerr("rigctld probe failed\n");
        ok = FALSE;
        goto cleanup;
    }
    {
        HamlibResponseInfo info = { 0 };
        gchar reply[64] = { 0 };
        gint count = 0;

        if (!rigctld_client_request_raw(rig, "\\get_conn_count\n",
                                        reply, sizeof(reply), &info))
        {
            g_printerr("rigctld conn count query failed\n");
            ok = FALSE;
            goto cleanup;
        }

        count = (gint)g_ascii_strtoll(reply, NULL, 10);
        if (count > 1)
        {
            g_printerr("rigctld unexpected connection count: %d\n", count);
            ok = FALSE;
            goto cleanup;
        }
    }
    caps = rigctld_client_get_caps(rig);
    if (caps == NULL || !caps->has_get_freq || !caps->has_set_freq)
    {
        g_printerr("rigctld caps missing expected frequency support\n");
        ok = FALSE;
        goto cleanup;
    }
    if ((caps->quirks & RIG_QUIRK_FORCE_MAIN_SUB) == 0)
    {
        g_printerr("rigctld quirk missing (expected IC-9700 signature)\n");
        ok = FALSE;
        goto cleanup;
    }

    if (!rigctld_client_get_freq(rig, VFO_MAIN, &freq))
    {
        g_printerr("rigctld get freq failed\n");
        ok = FALSE;
        goto cleanup;
    }
    if (fabs(freq - 145800000.0) >= 1.0)
    {
        g_printerr("rigctld initial freq mismatch: %.0f\n", freq);
        ok = FALSE;
        goto cleanup;
    }
    if (!rigctld_client_set_freq(rig, VFO_MAIN, 145900000.0))
    {
        g_printerr("rigctld set freq failed\n");
        ok = FALSE;
        goto cleanup;
    }
    if (!rigctld_client_get_freq(rig, VFO_MAIN, &freq))
    {
        g_printerr("rigctld get freq failed (post-set)\n");
        ok = FALSE;
        goto cleanup;
    }
    if (fabs(freq - 145900000.0) >= 1.0)
    {
        g_printerr("rigctld post-set freq mismatch: %.0f\n", freq);
        ok = FALSE;
        goto cleanup;
    }

    {
        HamlibResponseInfo info = { 0 };
        gchar reply[128] = { 0 };
        gint count = 0;

        if (!rigctld_client_request_raw(rig, "\\reset_set_freq_count\n",
                                        reply, sizeof(reply), &info))
        {
            g_printerr("rigctld reset count failed\n");
            ok = FALSE;
            goto cleanup;
        }

        for (gint i = 0; i < 10; i++)
        {
            if (!rigctld_client_set_freq(rig, VFO_MAIN,
                                         145900000.0 + (gdouble)(i * 10)))
            {
                g_printerr("rigctld set freq failed (counting)\n");
                ok = FALSE;
                goto cleanup;
            }
            g_usleep(50000);
        }

        if (!rigctld_client_request_raw(rig, "\\get_set_freq_count\n",
                                        reply, sizeof(reply), &info))
        {
            g_printerr("rigctld count query failed\n");
            ok = FALSE;
            goto cleanup;
        }

        count = (gint)g_ascii_strtoll(reply, NULL, 10);
        if (count < 2 || count > 10)
        {
            g_printerr("rigctld set freq count out of range: %d\n", count);
            ok = FALSE;
            goto cleanup;
        }
    }

    if (!connect_rotctld_with_retry(rot, "127.0.0.1", rot_port))
    {
        g_printerr("failed to connect to rotctld mock\n");
        ok = FALSE;
        goto cleanup;
    }
    if (!rotctld_client_probe(rot, 500))
    {
        g_printerr("rotctld probe failed\n");
        ok = FALSE;
        goto cleanup;
    }
    rcaps = rotctld_client_get_caps(rot);
    if (rcaps == NULL || !rcaps->has_get_pos || !rcaps->has_set_pos)
    {
        g_printerr("rotctld caps missing expected position support\n");
        ok = FALSE;
        goto cleanup;
    }

    if (!rotctld_client_get_pos(rot, &az, &el))
    {
        g_printerr("rotctld get pos failed\n");
        ok = FALSE;
        goto cleanup;
    }
    if (!rotctld_client_set_pos(rot, 15.0, 5.0))
    {
        g_printerr("rotctld set pos failed\n");
        ok = FALSE;
        goto cleanup;
    }
    {
        gchar dump_state[4096];
        gdouble hs_az = 0.0;
        gdouble hs_el = 0.0;

        if (!rotctld_client_handshake(rot, 500, &hs_az, &hs_el,
                                      dump_state, sizeof(dump_state),
                                      NULL, 0))
        {
            g_printerr("rotctld handshake failed\n");
            ok = FALSE;
            goto cleanup;
        }

        for (gint i = 0; i < 3; i++)
        {
            if (!rotctld_client_get_pos(rot, &az, &el))
            {
                g_printerr("rotctld get pos failed (repeat)\n");
                ok = FALSE;
                goto cleanup;
            }
        }

        if (!rotctld_client_handshake(rot, 500, &hs_az, &hs_el,
                                      dump_state, sizeof(dump_state),
                                      NULL, 0))
        {
            g_printerr("rotctld handshake failed (repeat)\n");
            ok = FALSE;
            goto cleanup;
        }
    }
    {
        HamlibResponseInfo info = { 0 };
        gchar reply[64] = { 0 };
        gint count = 0;

        if (!rotctld_client_request_raw(rot, "\\get_conn_count\n",
                                        reply, sizeof(reply), &info))
        {
            g_printerr("rotctld conn count query failed\n");
            ok = FALSE;
            goto cleanup;
        }

        count = (gint)g_ascii_strtoll(reply, NULL, 10);
        if (count > 1)
        {
            g_printerr("rotctld unexpected connection count: %d\n", count);
            ok = FALSE;
            goto cleanup;
        }
    }
    if (!rotctld_client_get_pos(rot, &az, &el))
    {
        g_printerr("rotctld get pos failed (post-set)\n");
        ok = FALSE;
        goto cleanup;
    }
    if (fabs(az - 15.0) >= 1e-3 || fabs(el - 5.0) >= 1e-3)
    {
        g_printerr("rotctld post-set pos mismatch: az=%.1f el=%.1f\n", az, el);
        ok = FALSE;
        goto cleanup;
    }

cleanup:
    rigctld_client_close(rig);
    rotctld_client_close(rot);
    rigctld_client_free(&rig);
    rotctld_client_free(&rot);

    if (rig_proc != NULL && ok &&
        !g_subprocess_wait_check(rig_proc, NULL, &error))
    {
        g_printerr("rigctld mock exit error: %s\n",
                   error ? error->message : "unknown");
        ok = FALSE;
        g_clear_error(&error);
    }
    if (rot_proc != NULL && ok &&
        !g_subprocess_wait_check(rot_proc, NULL, &error))
    {
        g_printerr("rotctld mock exit error: %s\n",
                   error ? error->message : "unknown");
        ok = FALSE;
        g_clear_error(&error);
    }

    if (rig_proc != NULL && !ok)
    {
        g_subprocess_force_exit(rig_proc);
        (void)g_subprocess_wait(rig_proc, NULL, NULL);
    }
    if (rot_proc != NULL && !ok)
    {
        g_subprocess_force_exit(rot_proc);
        (void)g_subprocess_wait(rot_proc, NULL, NULL);
    }

    g_clear_object(&rig_proc);
    g_clear_object(&rot_proc);
    g_free(python);
    g_free(rig_script);
    g_free(rot_script);

    return ok ? 0 : 1;
}
