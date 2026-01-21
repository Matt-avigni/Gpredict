#include <errno.h>
#include <math.h>
#include <stdlib.h>
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

static GSubprocess *spawn_rigctld_mock(const gchar *python,
                                       const gchar *script,
                                       guint16 port,
                                       gboolean no_vfo_opt)
{
    GSubprocess *proc = NULL;
    GError *error = NULL;
    GPtrArray *argv = NULL;
    gchar port_str[16];

    g_snprintf(port_str, sizeof(port_str), "%u", port);

    argv = g_ptr_array_new_with_free_func(g_free);
    g_ptr_array_add(argv, g_strdup(python));
    g_ptr_array_add(argv, g_strdup(script));
    g_ptr_array_add(argv, g_strdup("--host"));
    g_ptr_array_add(argv, g_strdup("127.0.0.1"));
    g_ptr_array_add(argv, g_strdup("--port"));
    g_ptr_array_add(argv, g_strdup(port_str));
    g_ptr_array_add(argv, g_strdup("--once"));
    if (no_vfo_opt)
        g_ptr_array_add(argv, g_strdup("--no-vfo-opt"));
    g_ptr_array_add(argv, NULL);

    proc = g_subprocess_newv((const gchar *const *)argv->pdata,
                             G_SUBPROCESS_FLAGS_STDOUT_SILENCE |
                                 G_SUBPROCESS_FLAGS_STDERR_SILENCE,
                             &error);
    if (proc == NULL)
    {
        g_printerr("failed to spawn %s: %s\n",
                   script, error ? error->message : "unknown");
        g_clear_error(&error);
    }

    g_ptr_array_free(argv, TRUE);
    return proc;
}

static GSubprocess *spawn_rotctld_mock(const gchar *python,
                                       const gchar *script,
                                       guint16 port,
                                       gboolean fail_get_pos,
                                       gboolean fail_set_pos,
                                       gboolean drop_set_pos)
{
    GSubprocess *proc = NULL;
    GError *error = NULL;
    GPtrArray *argv = NULL;
    gchar port_str[16];

    g_snprintf(port_str, sizeof(port_str), "%u", port);

    argv = g_ptr_array_new_with_free_func(g_free);
    g_ptr_array_add(argv, g_strdup(python));
    g_ptr_array_add(argv, g_strdup(script));
    g_ptr_array_add(argv, g_strdup("--host"));
    g_ptr_array_add(argv, g_strdup("127.0.0.1"));
    g_ptr_array_add(argv, g_strdup("--port"));
    g_ptr_array_add(argv, g_strdup(port_str));
    g_ptr_array_add(argv, g_strdup("--once"));
    if (fail_get_pos)
        g_ptr_array_add(argv, g_strdup("--fail-get-pos"));
    if (fail_set_pos)
        g_ptr_array_add(argv, g_strdup("--fail-set-pos"));
    if (drop_set_pos)
        g_ptr_array_add(argv, g_strdup("--drop-set-pos"));
    g_ptr_array_add(argv, NULL);

    proc = g_subprocess_newv((const gchar *const *)argv->pdata,
                             G_SUBPROCESS_FLAGS_STDOUT_SILENCE |
                                 G_SUBPROCESS_FLAGS_STDERR_SILENCE,
                             &error);
    if (proc == NULL)
    {
        g_printerr("failed to spawn %s: %s\n",
                   script, error ? error->message : "unknown");
        g_clear_error(&error);
    }

    g_ptr_array_free(argv, TRUE);
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
    guint16 rig_port_select = 0;
    guint16 rot_port = 0;
    guint16 rot_fail_port = 0;
    GSubprocess *rig_proc = NULL;
    GSubprocess *rig_select_proc = NULL;
    GSubprocess *rot_proc = NULL;
    GSubprocess *rot_fail_proc = NULL;
    RigctldClient *rig = NULL;
    RigctldClient *rig_select = NULL;
    RotctldClient *rot = NULL;
    RotctldClient *rot_fail = NULL;
    radio_conf_t conf;
    const RigCaps *caps = NULL;
    const RotCaps *rcaps = NULL;
    gint64 freq = 0;
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
    rig_port_select = pick_free_port();
    for (gint attempt = 0; attempt < 5 && rot_port == rig_port; attempt++)
        rot_port = pick_free_port();
    for (gint attempt = 0;
         attempt < 5 &&
         (rig_port_select == 0 ||
          rig_port_select == rig_port ||
          rig_port_select == rot_port);
         attempt++)
        rig_port_select = pick_free_port();
    if (rig_port == 0 || rot_port == 0 || rig_port_select == 0)
    {
        ok = FALSE;
        goto cleanup;
    }
    if (rot_port == rig_port || rig_port_select == rig_port ||
        rig_port_select == rot_port)
    {
        g_printerr("failed to select distinct mock ports\n");
        ok = FALSE;
        goto cleanup;
    }

    rig_proc = spawn_rigctld_mock(python, rig_script, rig_port, FALSE);
    rig_select_proc = spawn_rigctld_mock(python, rig_script, rig_port_select, TRUE);
    rot_proc = spawn_rotctld_mock(python, rot_script, rot_port,
                                  FALSE, FALSE, FALSE);
    if (rig_proc == NULL || rig_select_proc == NULL || rot_proc == NULL)
    {
        ok = FALSE;
        goto cleanup;
    }

    rig = rigctld_client_new("mock-rig");
    rig_select = rigctld_client_new("mock-rig-select");
    rot = rotctld_client_new("mock-rot");
    if (rig == NULL || rig_select == NULL || rot == NULL)
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
    if (!connect_rigctld_with_retry(rig_select, "127.0.0.1", rig_port_select))
    {
        g_printerr("failed to connect to rigctld mock (no-vfo-opt)\n");
        ok = FALSE;
        goto cleanup;
    }
    if (!rigctld_client_probe(rig_select, &conf, 500))
    {
        g_printerr("rigctld probe failed (no-vfo-opt)\n");
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
    if (llabs(freq - 145800000) >= 1)
    {
        g_printerr("rigctld initial freq mismatch: %" G_GINT64_FORMAT "\n", freq);
        ok = FALSE;
        goto cleanup;
    }
    if (!rigctld_client_set_freq(rig, VFO_MAIN, 145900000))
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
    if (llabs(freq - 145900000) >= 1)
    {
        g_printerr("rigctld post-set freq mismatch: %" G_GINT64_FORMAT "\n", freq);
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
                                         145900000 + (gint64)(i * 10)))
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
    {
        HamlibResponseInfo info = { 0 };
        gchar reply[64] = { 0 };
        gint64 raw_freq = 0;

        if (!rigctld_client_request_raw(rig, "f",
                                        reply, sizeof(reply), &info))
        {
            g_printerr("rigctld raw request failed (newline)\n");
            ok = FALSE;
            goto cleanup;
        }

        raw_freq = g_ascii_strtoll(reply, NULL, 10);
        if (raw_freq <= 0)
        {
            g_printerr("rigctld raw request returned invalid freq: %" G_GINT64_FORMAT "\n",
                       raw_freq);
            ok = FALSE;
            goto cleanup;
        }
    }
    {
        HamlibResponseInfo info = { 0 };
        gchar reply[512] = { 0 };
        gchar **lines = NULL;
        gint idx_v_sub = -1;
        gint idx_f_set = -1;
        gint idx_v_main = -1;
        gint idx_f_get = -1;

        if (!rigctld_client_request_raw(rig_select, "\\reset_cmd_log",
                                        reply, sizeof(reply), &info))
        {
            g_printerr("rigctld cmd log reset failed\n");
            ok = FALSE;
            goto cleanup;
        }

        if (!rigctld_client_set_freq(rig_select, VFO_SUB, 145910000))
        {
            g_printerr("rigctld set freq failed (select-vfo)\n");
            ok = FALSE;
            goto cleanup;
        }

        if (!rigctld_client_get_freq(rig_select, VFO_MAIN, &freq))
        {
            g_printerr("rigctld get freq failed (select-vfo)\n");
            ok = FALSE;
            goto cleanup;
        }

        if (!rigctld_client_request_raw(rig_select, "\\get_cmd_log",
                                        reply, sizeof(reply), &info))
        {
            g_printerr("rigctld cmd log query failed\n");
            ok = FALSE;
            goto cleanup;
        }

        lines = g_strsplit(reply, "\n", -1);
        for (gint i = 0; lines[i] != NULL; i++)
        {
            const gchar *line = lines[i];

            if (line[0] == '\0' || g_str_has_prefix(line, "RPRT"))
                continue;

            if (g_str_has_prefix(line, "V "))
            {
                if (idx_v_sub == -1)
                    idx_v_sub = i;
                else if (idx_v_main == -1)
                    idx_v_main = i;
                continue;
            }

            if (line[0] == 'F' && idx_f_set == -1)
            {
                idx_f_set = i;
                continue;
            }
            if (line[0] == 'f' && idx_f_get == -1)
            {
                idx_f_get = i;
                continue;
            }
        }

        if (idx_v_sub < 0 || idx_f_set < idx_v_sub ||
            idx_v_main < 0 || idx_f_get < idx_v_main)
        {
            g_printerr("rigctld VFO selection sequence mismatch\n");
            ok = FALSE;
            g_strfreev(lines);
            goto cleanup;
        }
        if (g_strrstr(lines[idx_v_sub], "Sub") == NULL ||
            g_strrstr(lines[idx_v_main], "Main") == NULL)
        {
            g_printerr("rigctld VFO token mismatch: sub=%s main=%s\n",
                       lines[idx_v_sub], lines[idx_v_main]);
            ok = FALSE;
            g_strfreev(lines);
            goto cleanup;
        }

        g_strfreev(lines);
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
        HamlibResponseInfo info = { 0 };
        gchar reply[64] = { 0 };

        if (!rotctld_client_request_raw(rot, "P 15.0 5.0",
                                        reply, sizeof(reply), &info))
        {
            g_printerr("rotctld raw set pos failed (newline)\n");
            ok = FALSE;
            goto cleanup;
        }
        if (!info.saw_rprt || info.rprt_code != 0)
        {
            g_printerr("rotctld raw set pos unexpected rprt=%d saw=%d\n",
                       info.rprt_code, info.saw_rprt ? 1 : 0);
            ok = FALSE;
            goto cleanup;
        }
    }
    {
        gchar dump_state[4096];
        gdouble hs_az = 0.0;
        gdouble hs_el = 0.0;

        if (!rotctld_client_handshake(rot, 500, &hs_az, &hs_el,
                                      dump_state, sizeof(dump_state),
                                      NULL, 0, NULL))
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
                                      NULL, 0, NULL))
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

    rot_fail_port = pick_free_port();
    for (gint attempt = 0;
         attempt < 5 &&
         (rot_fail_port == 0 ||
          rot_fail_port == rig_port ||
          rot_fail_port == rot_port);
         attempt++)
        rot_fail_port = pick_free_port();
    if (rot_fail_port == 0 ||
        rot_fail_port == rig_port ||
        rot_fail_port == rot_port)
    {
        g_printerr("failed to select mock port for rotctld failure test\n");
        ok = FALSE;
        goto cleanup;
    }

    rot_fail_proc = spawn_rotctld_mock(python, rot_script, rot_fail_port,
                                       TRUE, TRUE, FALSE);
    if (rot_fail_proc == NULL)
    {
        ok = FALSE;
        goto cleanup;
    }

    rot_fail = rotctld_client_new("mock-rot-fail");
    if (rot_fail == NULL)
    {
        ok = FALSE;
        goto cleanup;
    }

    if (!connect_rotctld_with_retry(rot_fail, "127.0.0.1", rot_fail_port))
    {
        g_printerr("failed to connect to rotctld mock (failure mode)\n");
        ok = FALSE;
        goto cleanup;
    }
    {
        HamlibResponseInfo info = { 0 };
        rotctld_pos_result_t pos_res =
            rotctld_client_get_pos_ex(rot_fail, &az, &el, &info, NULL, 0);

        if (pos_res != ROTCTLD_POS_RPRT_ERR || info.rprt_code != -6)
        {
            g_printerr("rotctld get pos failure mode mismatch: res=%d rprt=%d\n",
                       pos_res, info.rprt_code);
            ok = FALSE;
            goto cleanup;
        }
    }
    {
        HamlibResponseInfo info = { 0 };
        gchar reply[64] = { 0 };

        if (!rotctld_client_request_raw(rot_fail, "P 10.0 20.0",
                                        reply, sizeof(reply), &info))
        {
            g_printerr("rotctld set pos failed in failure mode\n");
            ok = FALSE;
            goto cleanup;
        }
        if (!info.saw_rprt || info.rprt_code != -6)
        {
            g_printerr("rotctld set pos failure mode unexpected rprt=%d saw=%d\n",
                       info.rprt_code, info.saw_rprt ? 1 : 0);
            ok = FALSE;
            goto cleanup;
        }
    }
    {
        HamlibResponseInfo info = { 0 };
        gchar reply[64] = { 0 };
        gint count = 0;

        if (!rotctld_client_request_raw(rot_fail, "\\get_conn_count",
                                        reply, sizeof(reply), &info))
        {
            g_printerr("rotctld conn count query failed (failure mode)\n");
            ok = FALSE;
            goto cleanup;
        }

        count = (gint)g_ascii_strtoll(reply, NULL, 10);
        if (count > 1)
        {
            g_printerr("rotctld unexpected connection count (failure mode): %d\n",
                       count);
            ok = FALSE;
            goto cleanup;
        }
    }

cleanup:
    rigctld_client_close(rig);
    rigctld_client_close(rig_select);
    rotctld_client_close(rot);
    rotctld_client_close(rot_fail);
    rigctld_client_free(&rig);
    rigctld_client_free(&rig_select);
    rotctld_client_free(&rot);
    rotctld_client_free(&rot_fail);

    if (rig_proc != NULL && ok &&
        !g_subprocess_wait_check(rig_proc, NULL, &error))
    {
        g_printerr("rigctld mock exit error: %s\n",
                   error ? error->message : "unknown");
        ok = FALSE;
        g_clear_error(&error);
    }
    if (rig_select_proc != NULL && ok &&
        !g_subprocess_wait_check(rig_select_proc, NULL, &error))
    {
        g_printerr("rigctld mock (no-vfo-opt) exit error: %s\n",
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
    if (rot_fail_proc != NULL && ok &&
        !g_subprocess_wait_check(rot_fail_proc, NULL, &error))
    {
        g_printerr("rotctld mock failure exit error: %s\n",
                   error ? error->message : "unknown");
        ok = FALSE;
        g_clear_error(&error);
    }

    if (rig_proc != NULL && !ok)
    {
        g_subprocess_force_exit(rig_proc);
        (void)g_subprocess_wait(rig_proc, NULL, NULL);
    }
    if (rig_select_proc != NULL && !ok)
    {
        g_subprocess_force_exit(rig_select_proc);
        (void)g_subprocess_wait(rig_select_proc, NULL, NULL);
    }
    if (rot_proc != NULL && !ok)
    {
        g_subprocess_force_exit(rot_proc);
        (void)g_subprocess_wait(rot_proc, NULL, NULL);
    }
    if (rot_fail_proc != NULL && !ok)
    {
        g_subprocess_force_exit(rot_fail_proc);
        (void)g_subprocess_wait(rot_fail_proc, NULL, NULL);
    }

    g_clear_object(&rig_proc);
    g_clear_object(&rig_select_proc);
    g_clear_object(&rot_proc);
    g_free(python);
    g_free(rig_script);
    g_free(rot_script);

    return ok ? 0 : 1;
}
