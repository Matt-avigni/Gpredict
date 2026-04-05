/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#ifndef G_OS_WIN32
#include <sys/socket.h>
#include <unistd.h>
#endif

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

static gboolean command_matches_any_token(const gchar *line,
                                          const gchar * const *tokens)
{
    if (line == NULL || tokens == NULL)
        return FALSE;

    for (gint i = 0; tokens[i] != NULL; i++)
    {
        if (g_strrstr(line, tokens[i]) != NULL)
            return TRUE;
    }

    return FALSE;
}

static GSubprocess *spawn_rigctld_mock(const gchar *python,
                                       const gchar *script,
                                       guint16 port,
                                       gboolean no_vfo_opt,
                                       gboolean reject_main_sub_tokenized_retune,
                                       gboolean reject_sub_select,
                                       gboolean fail_get_freq,
                                       gboolean no_get_vfo_advertise,
                                       gboolean no_vfo_opt_advertise,
                                       gboolean fail_plain_get_freq,
                                       gboolean fail_get_vfo,
                                       gboolean require_vfo_before_set,
                                       gint fail_vfo_select_count)
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
    if (reject_main_sub_tokenized_retune)
        g_ptr_array_add(argv, g_strdup("--reject-main-sub-tokenized-retune"));
    if (reject_sub_select)
        g_ptr_array_add(argv, g_strdup("--reject-sub-select"));
    if (fail_get_freq)
        g_ptr_array_add(argv, g_strdup("--fail-get-freq"));
    if (no_get_vfo_advertise)
        g_ptr_array_add(argv, g_strdup("--no-get-vfo-advertise"));
    if (no_vfo_opt_advertise)
        g_ptr_array_add(argv, g_strdup("--no-vfo-opt-advertise"));
    if (fail_plain_get_freq)
        g_ptr_array_add(argv, g_strdup("--fail-plain-get-freq"));
    if (fail_get_vfo)
        g_ptr_array_add(argv, g_strdup("--fail-get-vfo"));
    if (require_vfo_before_set)
        g_ptr_array_add(argv, g_strdup("--require-vfo-before-set"));
    if (fail_vfo_select_count > 0)
    {
        g_ptr_array_add(argv, g_strdup("--fail-vfo-select-count"));
        g_ptr_array_add(argv,
                        g_strdup_printf("%d", fail_vfo_select_count));
    }
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
                                       gboolean drop_set_pos,
                                       gboolean split_replies,
                                       gint disconnect_after,
                                       gboolean once)
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
    if (once)
        g_ptr_array_add(argv, g_strdup("--once"));
    if (fail_get_pos)
        g_ptr_array_add(argv, g_strdup("--fail-get-pos"));
    if (fail_set_pos)
        g_ptr_array_add(argv, g_strdup("--fail-set-pos"));
    if (drop_set_pos)
        g_ptr_array_add(argv, g_strdup("--drop-set-pos"));
    if (split_replies)
        g_ptr_array_add(argv, g_strdup("--split-replies"));
    if (disconnect_after > 0)
    {
        g_ptr_array_add(argv, g_strdup("--disconnect-after"));
        g_ptr_array_add(argv, g_strdup_printf("%d", disconnect_after));
    }
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

typedef struct {
    RotctldClient *client;
    gboolean ok;
    gint loops;
} RotThreadCtx;

typedef struct {
    GPtrArray *lines;
} RigLogCapture;

static void rig_log_capture_cb(RigctldClient *client,
                               const gchar *prefix,
                               const gchar *line,
                               gpointer user_data)
{
    RigLogCapture *capture = user_data;

    (void)client;

    if (capture == NULL || capture->lines == NULL ||
        prefix == NULL || line == NULL)
        return;

    g_ptr_array_add(capture->lines,
                    g_strdup_printf("%s %s", prefix, line));
}

static gboolean rig_log_capture_contains(const RigLogCapture *capture,
                                         const gchar *needle)
{
    if (capture == NULL || capture->lines == NULL || needle == NULL)
        return FALSE;

    for (guint i = 0; i < capture->lines->len; i++)
    {
        const gchar *line = g_ptr_array_index(capture->lines, i);

        if (line != NULL && g_strrstr(line, needle) != NULL)
            return TRUE;
    }

    return FALSE;
}

static void rig_log_capture_clear(RigLogCapture *capture)
{
    if (capture == NULL || capture->lines == NULL)
        return;

    g_ptr_array_set_size(capture->lines, 0);
}

static void rig_log_capture_dump(const RigLogCapture *capture,
                                 const gchar *label)
{
    if (capture == NULL || capture->lines == NULL)
        return;

    g_printerr("%s (%u lines):\n",
               label ? label : "rig log capture",
               capture->lines->len);
    for (guint i = 0; i < capture->lines->len; i++)
    {
        const gchar *line = g_ptr_array_index(capture->lines, i);

        g_printerr("  %s\n", line ? line : "(null)");
    }
}

#ifndef G_OS_WIN32
typedef struct {
    gint fd;
    gint first_freq_term_delay_ms;
    gint vfo_select_delay_ms;
    gchar current_vfo[32];
} RigProbeLateReplyCtx;

static gboolean socket_test_write_all(gint fd, const gchar *text)
{
    gsize offset = 0;
    gsize len = 0;

    if (fd < 0 || text == NULL)
        return FALSE;

    len = strlen(text);
    while (offset < len)
    {
        gssize written = write(fd, text + offset, len - offset);

        if (written < 0)
        {
            if (errno == EINTR)
                continue;
            return FALSE;
        }
        if (written == 0)
            return FALSE;

        offset += (gsize) written;
    }

    return TRUE;
}

static gboolean socket_test_read_line(gint fd, gchar *out, gsize out_len)
{
    gsize used = 0;

    if (fd < 0 || out == NULL || out_len == 0)
        return FALSE;

    out[0] = '\0';
    while (used + 1 < out_len)
    {
        gchar ch = '\0';
        gssize size = read(fd, &ch, 1);

        if (size < 0)
        {
            if (errno == EINTR)
                continue;
            return FALSE;
        }
        if (size == 0)
            return used > 0;
        if (ch == '\r')
            continue;
        if (ch == '\n')
            break;

        out[used++] = ch;
    }

    out[used] = '\0';
    return TRUE;
}

static gpointer rig_probe_late_reply_server(gpointer data)
{
    RigProbeLateReplyCtx *ctx = data;
    gchar line[128];
    gboolean first_freq = TRUE;

    if (ctx == NULL || ctx->fd < 0)
        return NULL;

    g_strlcpy(ctx->current_vfo, "VFOA", sizeof(ctx->current_vfo));
    while (socket_test_read_line(ctx->fd, line, sizeof(line)))
    {
        if (g_strcmp0(line, "\\dump_state") == 0)
        {
            if (!socket_test_write_all(ctx->fd,
                                       "1\n"
                                       "3081\n"
                                       "Hamlib Mock rigctld\n"
                                       "has_get_vfo: 1\n"
                                       "has_set_vfo: 1\n"
                                       "has_set_vfo_opt: 0\n"
                                       "vfo list: VFOA VFOB Main Sub currVFO\n"))
                break;
            continue;
        }

        if (g_strcmp0(line, "f") == 0)
        {
            if (!socket_test_write_all(ctx->fd, "145800000\n"))
                break;
            if (first_freq)
            {
                g_usleep((gulong) ctx->first_freq_term_delay_ms * 1000);
                first_freq = FALSE;
            }
            if (!socket_test_write_all(ctx->fd, "RPRT 0\n"))
                break;
            continue;
        }

        if (g_strcmp0(line, "v") == 0)
        {
            gchar reply[64];

            g_snprintf(reply, sizeof(reply), "%s\nRPRT 0\n", ctx->current_vfo);
            if (!socket_test_write_all(ctx->fd, reply))
                break;
            continue;
        }

        if (g_str_has_prefix(line, "V "))
        {
            const gchar *token = line + 2;

            g_strlcpy(ctx->current_vfo, token, sizeof(ctx->current_vfo));
            g_usleep((gulong) ctx->vfo_select_delay_ms * 1000);
            if (!socket_test_write_all(ctx->fd, "RPRT 0\n"))
                break;
            continue;
        }

        if (g_str_has_prefix(line, "F "))
        {
            if (!socket_test_write_all(ctx->fd, "RPRT 0\n"))
                break;
            continue;
        }

        if (g_strcmp0(line, "q") == 0)
            break;

        if (!socket_test_write_all(ctx->fd, "RPRT 0\n"))
            break;
    }

    close(ctx->fd);
    ctx->fd = -1;
    return NULL;
}

static gboolean rigctld_probe_late_reply_test(void)
{
    gint fds[2] = { -1, -1 };
    RigProbeLateReplyCtx ctx = { .fd = -1,
                                 .first_freq_term_delay_ms = 120,
                                 .vfo_select_delay_ms = 150 };
    GThread *server_thread = NULL;
    RigctldClient *client = NULL;
    RigCaps *caps = NULL;
    gchar *attach_error = NULL;
    radio_conf_t conf;
    gboolean ok = FALSE;

    if (socketpair(AF_UNIX, SOCK_STREAM, 0, fds) != 0)
    {
        g_printerr("socketpair failed for rigctld late-reply probe test: %s\n",
                   g_strerror(errno));
        goto cleanup;
    }

    client = rigctld_client_new("mock-rig-late-reply");
    if (client == NULL)
    {
        g_printerr("rigctld client allocation failed\n");
        goto cleanup;
    }

    if (!rigctld_client_attach_fd(client, fds[0], &attach_error))
    {
        g_printerr("rigctld attach fd failed: %s\n",
                   attach_error ? attach_error : "unknown");
        goto cleanup;
    }
    fds[0] = -1;

    ctx.fd = fds[1];
    fds[1] = -1;
    server_thread = g_thread_new("rig-probe-late-reply",
                                 rig_probe_late_reply_server, &ctx);

    memset(&conf, 0, sizeof(conf));
    conf.rigctld_model = 3081;
    conf.radio_mode = RADIO_MODE_FULL_DUPLEX_MAIN_SUB;
    if (!rigctld_client_probe(client, &conf, 500))
    {
        g_printerr("rigctld probe failed when first freq reply terminator arrived late\n");
        goto cleanup;
    }
    caps = rigctld_client_get_caps_snapshot(client);
    if (caps == NULL || caps->strategy != RIG_STRATEGY_SELECT_VFO)
    {
        g_printerr("rigctld late-reply probe should settle on SELECT_VFO strategy\n");
        goto cleanup;
    }
    if (!caps->has_get_vfo ||
        caps->vfo_token_main == NULL ||
        caps->vfo_token_sub == NULL)
    {
        g_printerr("rigctld late-reply probe did not retain explicit Main/Sub token mapping\n");
        goto cleanup;
    }

    ok = TRUE;

cleanup:
    rigctld_client_caps_snapshot_free(caps);
    rigctld_client_close(client);
    rigctld_client_free(&client);
    if (fds[0] >= 0)
        close(fds[0]);
    if (fds[1] >= 0)
        close(fds[1]);
    if (server_thread != NULL)
        g_thread_join(server_thread);
    if (ctx.fd >= 0)
        close(ctx.fd);
    g_free(attach_error);

    return ok;
}
#endif

static gpointer rot_thread_getpos(gpointer data)
{
    RotThreadCtx *ctx = data;
    gdouble az = 0.0;
    gdouble el = 0.0;

    if (ctx == NULL || ctx->client == NULL)
        return NULL;

    gint loops = (ctx->loops > 0) ? ctx->loops : 1;

    for (gint i = 0; i < loops; i++)
    {
        if (!rotctld_client_get_pos(ctx->client, &az, &el))
        {
            ctx->ok = FALSE;
            break;
        }
    }
    return NULL;
}

static gpointer rot_thread_setpos(gpointer data)
{
    RotThreadCtx *ctx = data;

    if (ctx == NULL || ctx->client == NULL)
        return NULL;

    gint loops = (ctx->loops > 0) ? ctx->loops : 1;

    for (gint i = 0; i < loops; i++)
    {
        gdouble az = (gdouble)(i % 10) * 3.0;
        gdouble el = (gdouble)(i % 5) * 2.0;
        if (!rotctld_client_set_pos(ctx->client, az, el))
        {
            ctx->ok = FALSE;
            break;
        }
    }
    return NULL;
}

int main(void)
{
    gchar *python = NULL;
    gchar *rig_script = NULL;
    gchar *rot_script = NULL;
    guint16 rig_port = 0;
    guint16 rig_port_select = 0;
    guint16 rig_port_reject = 0;
    guint16 rig_port_missing_sub = 0;
    guint16 rig_port_select_only = 0;
    guint16 rig_port_vfo_opt_hidden = 0;
    guint16 rot_port = 0;
    guint16 rot_fail_port = 0;
    guint16 rot_split_port = 0;
    guint16 rot_drop_port = 0;
    GSubprocess *rig_proc = NULL;
    GSubprocess *rig_select_proc = NULL;
    GSubprocess *rig_reject_proc = NULL;
    GSubprocess *rig_missing_sub_proc = NULL;
    GSubprocess *rig_select_only_proc = NULL;
    GSubprocess *rig_vfo_opt_hidden_proc = NULL;
    GSubprocess *rot_proc = NULL;
    GSubprocess *rot_fail_proc = NULL;
    GSubprocess *rot_split_proc = NULL;
    GSubprocess *rot_drop_proc = NULL;
    RigctldClient *rig = NULL;
    RigctldClient *rig_select = NULL;
    RigctldClient *rig_reject = NULL;
    RigctldClient *rig_missing_sub = NULL;
    RigctldClient *rig_select_only = NULL;
    RigctldClient *rig_vfo_opt_hidden = NULL;
    RotctldClient *rot = NULL;
    RotctldClient *rot_fail = NULL;
    RotctldClient *rot_split = NULL;
    RotctldClient *rot_drop = NULL;
    radio_conf_t conf;
    RigCaps *caps = NULL;
    RigCaps *reject_caps = NULL;
    RigCaps *select_only_caps = NULL;
    RigCaps *vfo_opt_hidden_caps = NULL;
    RotCaps rcaps_snapshot = { 0 };
    const RotCaps *rcaps = NULL;
    gint64 freq = 0;
    gdouble az = 0.0;
    gdouble el = 0.0;
    GError *error = NULL;
    gboolean ok = TRUE;
    RigLogCapture rig_probe_log = { 0 };
    RigLogCapture rig_select_log = { 0 };

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
    rig_port_reject = pick_free_port();
    rig_port_missing_sub = pick_free_port();
    rig_port_select_only = pick_free_port();
    rig_port_vfo_opt_hidden = pick_free_port();
    for (gint attempt = 0; attempt < 5 && rot_port == rig_port; attempt++)
        rot_port = pick_free_port();
    for (gint attempt = 0;
         attempt < 5 &&
         (rig_port_select == 0 ||
          rig_port_select == rig_port ||
          rig_port_select == rot_port);
         attempt++)
        rig_port_select = pick_free_port();
    for (gint attempt = 0;
         attempt < 5 &&
         (rig_port_reject == 0 ||
          rig_port_reject == rig_port ||
          rig_port_reject == rot_port ||
          rig_port_reject == rig_port_select);
         attempt++)
        rig_port_reject = pick_free_port();
    for (gint attempt = 0;
         attempt < 5 &&
         (rig_port_missing_sub == 0 ||
          rig_port_missing_sub == rig_port ||
          rig_port_missing_sub == rot_port ||
          rig_port_missing_sub == rig_port_select ||
          rig_port_missing_sub == rig_port_reject);
         attempt++)
        rig_port_missing_sub = pick_free_port();
    for (gint attempt = 0;
         attempt < 5 &&
         (rig_port_select_only == 0 ||
          rig_port_select_only == rig_port ||
          rig_port_select_only == rot_port ||
          rig_port_select_only == rig_port_select ||
          rig_port_select_only == rig_port_reject ||
          rig_port_select_only == rig_port_missing_sub);
         attempt++)
        rig_port_select_only = pick_free_port();
    for (gint attempt = 0;
         attempt < 5 &&
         (rig_port_vfo_opt_hidden == 0 ||
          rig_port_vfo_opt_hidden == rig_port ||
          rig_port_vfo_opt_hidden == rot_port ||
          rig_port_vfo_opt_hidden == rig_port_select ||
          rig_port_vfo_opt_hidden == rig_port_reject ||
          rig_port_vfo_opt_hidden == rig_port_missing_sub ||
          rig_port_vfo_opt_hidden == rig_port_select_only);
         attempt++)
        rig_port_vfo_opt_hidden = pick_free_port();
    if (rig_port == 0 || rot_port == 0 || rig_port_select == 0 ||
        rig_port_reject == 0 || rig_port_missing_sub == 0 ||
        rig_port_select_only == 0 || rig_port_vfo_opt_hidden == 0)
    {
        ok = FALSE;
        goto cleanup;
    }
    if (rot_port == rig_port || rig_port_select == rig_port ||
        rig_port_select == rot_port || rig_port_reject == rig_port ||
        rig_port_reject == rot_port || rig_port_reject == rig_port_select ||
        rig_port_missing_sub == rig_port ||
        rig_port_missing_sub == rot_port ||
        rig_port_missing_sub == rig_port_select ||
        rig_port_missing_sub == rig_port_reject ||
        rig_port_select_only == rig_port ||
        rig_port_select_only == rot_port ||
        rig_port_select_only == rig_port_select ||
        rig_port_select_only == rig_port_reject ||
        rig_port_select_only == rig_port_missing_sub ||
        rig_port_vfo_opt_hidden == rig_port ||
        rig_port_vfo_opt_hidden == rot_port ||
        rig_port_vfo_opt_hidden == rig_port_select ||
        rig_port_vfo_opt_hidden == rig_port_reject ||
        rig_port_vfo_opt_hidden == rig_port_missing_sub ||
        rig_port_vfo_opt_hidden == rig_port_select_only)
    {
        g_printerr("failed to select distinct mock ports\n");
        ok = FALSE;
        goto cleanup;
    }

    rig_proc = spawn_rigctld_mock(python, rig_script, rig_port, FALSE, FALSE,
                                  FALSE, FALSE, FALSE, FALSE, FALSE, FALSE,
                                  FALSE, 0);
    rig_select_proc = spawn_rigctld_mock(python, rig_script, rig_port_select,
                                         TRUE, FALSE, FALSE, FALSE, FALSE,
                                         FALSE, FALSE, FALSE, TRUE, 2);
    rig_reject_proc = spawn_rigctld_mock(python, rig_script, rig_port_reject,
                                         FALSE, TRUE, FALSE, FALSE, FALSE,
                                         FALSE, FALSE, FALSE, FALSE, 0);
    rig_missing_sub_proc = spawn_rigctld_mock(python, rig_script,
                                              rig_port_missing_sub,
                                              TRUE, FALSE, TRUE, FALSE, FALSE,
                                              FALSE, FALSE, FALSE, FALSE, 0);
    rig_select_only_proc = spawn_rigctld_mock(python, rig_script,
                                              rig_port_select_only,
                                              TRUE, FALSE, FALSE, TRUE, TRUE,
                                              FALSE, FALSE, FALSE, FALSE, 0);
    rig_vfo_opt_hidden_proc = spawn_rigctld_mock(python, rig_script,
                                                 rig_port_vfo_opt_hidden,
                                                 FALSE, FALSE, FALSE, FALSE,
                                                 TRUE, TRUE, TRUE, TRUE,
                                                 FALSE, 0);
    rot_proc = spawn_rotctld_mock(python, rot_script, rot_port,
                                  FALSE, FALSE, FALSE, FALSE, 0, TRUE);
    if (rig_proc == NULL || rig_select_proc == NULL ||
        rig_reject_proc == NULL || rig_missing_sub_proc == NULL ||
        rig_select_only_proc == NULL || rig_vfo_opt_hidden_proc == NULL ||
        rot_proc == NULL)
    {
        ok = FALSE;
        goto cleanup;
    }

    rig_probe_log.lines = g_ptr_array_new_with_free_func(g_free);
    rig_select_log.lines = g_ptr_array_new_with_free_func(g_free);
    rigctld_client_set_log_level(RIG_LOG_VERBOSE);

    rig = rigctld_client_new("mock-rig");
    rig_select = rigctld_client_new("mock-rig-select");
    rig_reject = rigctld_client_new("mock-rig-reject");
    rig_missing_sub = rigctld_client_new("mock-rig-missing-sub");
    rig_select_only = rigctld_client_new("mock-rig-select-only");
    rig_vfo_opt_hidden = rigctld_client_new("mock-rig-vfo-opt-hidden");
    rot = rotctld_client_new("mock-rot");
    if (rig == NULL || rig_select == NULL || rig_reject == NULL ||
        rig_missing_sub == NULL || rig_select_only == NULL ||
        rig_vfo_opt_hidden == NULL || rot == NULL)
    {
        ok = FALSE;
        goto cleanup;
    }

    rigctld_client_set_log_callback(rig, rig_log_capture_cb, &rig_probe_log);
    rigctld_client_set_log_callback(rig_select, rig_log_capture_cb,
                                    &rig_select_log);

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
    if (!rig_log_capture_contains(&rig_probe_log,
                                  "gpredict:tx [mock-rig] \\dump_state") ||
        !rig_log_capture_contains(&rig_probe_log,
                                  "gpredict:tx [mock-rig] V ") ||
        !rig_log_capture_contains(&rig_probe_log,
                                  "gpredict:rx [mock-rig] RPRT 0"))
    {
        g_printerr("rigctld probe logging missing expected tx/rx lines\n");
        rig_log_capture_dump(&rig_probe_log, "probe log");
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
    if (!connect_rigctld_with_retry(rig_reject, "127.0.0.1", rig_port_reject))
    {
        g_printerr("failed to connect to rigctld mock (tokenized Main/Sub retune reject)\n");
        ok = FALSE;
        goto cleanup;
    }
    if (!rigctld_client_probe(rig_reject, &conf, 500))
    {
        g_printerr("rigctld probe failed (tokenized Main/Sub retune reject)\n");
        ok = FALSE;
        goto cleanup;
    }
    if (!connect_rigctld_with_retry(rig_missing_sub, "127.0.0.1",
                                    rig_port_missing_sub))
    {
        g_printerr("failed to connect to rigctld mock (missing Sub control)\n");
        ok = FALSE;
        goto cleanup;
    }
    if (rigctld_client_probe(rig_missing_sub, &conf, 500))
    {
        rigctld_client_state_t state = RIGCTLD_CLIENT_STOPPED;
        gchar reason[128] = { 0 };
        RigCaps *missing_sub_caps =
            rigctld_client_get_caps_snapshot(rig_missing_sub);

        rigctld_client_get_status(rig_missing_sub, &state,
                                  reason, sizeof(reason));
        g_printerr("rigctld probe should fail when shared Main/Sub lacks explicit Sub control"
                   " (state=%d strategy=%d reason=%s)\n",
                   (gint) state,
                   missing_sub_caps ? (gint) missing_sub_caps->strategy : -1,
                   reason[0] != '\0' ? reason : "(none)");
        rigctld_client_caps_snapshot_free(missing_sub_caps);
        ok = FALSE;
        goto cleanup;
    }
    if (!connect_rigctld_with_retry(rig_select_only, "127.0.0.1",
                                    rig_port_select_only))
    {
        g_printerr("failed to connect to rigctld mock (select-only)\n");
        ok = FALSE;
        goto cleanup;
    }
    if (!rigctld_client_probe(rig_select_only, &conf, 500))
    {
        g_printerr("rigctld probe should accept Main/Sub when current-VFO readback validates selection\n");
        ok = FALSE;
        goto cleanup;
    }
    select_only_caps = rigctld_client_get_caps_snapshot(rig_select_only);
    if (select_only_caps == NULL ||
        select_only_caps->strategy != RIG_STRATEGY_SELECT_VFO)
    {
        g_printerr("rigctld select-only probe should settle on SELECT_VFO strategy\n");
        ok = FALSE;
        goto cleanup;
    }
#ifndef G_OS_WIN32
    if (!rigctld_probe_late_reply_test())
    {
        ok = FALSE;
        goto cleanup;
    }
#endif
    if (!select_only_caps->has_get_vfo)
    {
        g_printerr("rigctld select-only probe should validate VFO selection via current-VFO readback\n");
        ok = FALSE;
        goto cleanup;
    }
    if (select_only_caps->vfo_working == NULL ||
        !g_hash_table_contains(select_only_caps->vfo_working, "VFOA") ||
        !g_hash_table_contains(select_only_caps->vfo_working, "VFOB"))
    {
        g_printerr("rigctld select-only probe did not cache expected validated VFO tokens\n");
        ok = FALSE;
        goto cleanup;
    }
    if (!connect_rigctld_with_retry(rig_vfo_opt_hidden, "127.0.0.1",
                                    rig_port_vfo_opt_hidden))
    {
        g_printerr("failed to connect to rigctld mock (hidden vfo-opt)\n");
        ok = FALSE;
        goto cleanup;
    }
    if (!rigctld_client_probe(rig_vfo_opt_hidden, &conf, 500))
    {
        g_printerr("rigctld probe should discover tokenized Main/Sub VFO args when set_vfo_opt works but is not advertised\n");
        ok = FALSE;
        goto cleanup;
    }
    vfo_opt_hidden_caps = rigctld_client_get_caps_snapshot(rig_vfo_opt_hidden);
    if (vfo_opt_hidden_caps == NULL ||
        vfo_opt_hidden_caps->strategy != RIG_STRATEGY_VFO_OPT_ARGS ||
        !vfo_opt_hidden_caps->has_get_freq ||
        !vfo_opt_hidden_caps->has_set_freq)
    {
        g_printerr("rigctld hidden vfo-opt probe should settle on VFO_OPT_ARGS with frequency support\n");
        ok = FALSE;
        goto cleanup;
    }
    {
        HamlibResponseInfo info = { 0 };
        gchar reply[512] = { 0 };
        gchar **lines = NULL;
        static const gchar *main_tokens[] =
            { "Main", "MainA", "VFO_MAIN", "VFOA", NULL };
        static const gchar *sub_tokens[] =
            { "Sub", "SubA", "VFO_SUB", "VFOB", NULL };
        gint idx_f_get = -1;
        gint idx_f_set = -1;

        if (!rigctld_client_request_raw(rig_vfo_opt_hidden, "\\reset_cmd_log",
                                        reply, sizeof(reply), &info))
        {
            g_printerr("rigctld cmd log reset failed (hidden vfo-opt)\n");
            ok = FALSE;
            goto cleanup;
        }

        if (!rigctld_client_get_freq(rig_vfo_opt_hidden, VFO_MAIN, &freq) ||
            !rigctld_client_set_freq(rig_vfo_opt_hidden, VFO_SUB, 145920000))
        {
            g_printerr("rigctld hidden vfo-opt runtime commands failed\n");
            ok = FALSE;
            goto cleanup;
        }

        if (!rigctld_client_request_raw(rig_vfo_opt_hidden, "\\get_cmd_log",
                                        reply, sizeof(reply), &info))
        {
            g_printerr("rigctld cmd log query failed (hidden vfo-opt)\n");
            ok = FALSE;
            goto cleanup;
        }

        lines = g_strsplit(reply, "\n", -1);
        for (gint i = 0; lines[i] != NULL; i++)
        {
            const gchar *line = lines[i];

            if (line[0] == '\0' || g_str_has_prefix(line, "RPRT"))
                continue;

            if (g_str_has_prefix(line, "f ") && idx_f_get == -1)
            {
                idx_f_get = i;
                continue;
            }
            if (g_str_has_prefix(line, "F ") && idx_f_set == -1)
            {
                idx_f_set = i;
                continue;
            }
        }

        if (idx_f_get < 0 || idx_f_set < 0 ||
            !command_matches_any_token(lines[idx_f_get], main_tokens) ||
            !command_matches_any_token(lines[idx_f_set], sub_tokens))
        {
            g_printerr("rigctld hidden vfo-opt runtime token mapping mismatch\n");
            ok = FALSE;
            g_strfreev(lines);
            goto cleanup;
        }

        g_strfreev(lines);
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
    caps = rigctld_client_get_caps_snapshot(rig);
    if (caps == NULL || !caps->has_get_freq || !caps->has_set_freq)
    {
        g_printerr("rigctld caps missing expected frequency support\n");
        ok = FALSE;
        goto cleanup;
    }
    if (caps->strategy != RIG_STRATEGY_SELECT_VFO)
    {
        g_printerr("rigctld FULL_DUPLEX_MAIN_SUB should prefer SELECT_VFO over tokenized Main/Sub args\n");
        ok = FALSE;
        goto cleanup;
    }
    if ((caps->quirks & RIG_QUIRK_FORCE_MAIN_SUB) == 0)
    {
        g_printerr("rigctld quirk missing (expected IC-9700 signature)\n");
        ok = FALSE;
        goto cleanup;
    }
    if (caps->vfo_working == NULL ||
        !g_hash_table_contains(caps->vfo_working, "VFOA") ||
        !g_hash_table_contains(caps->vfo_working, "VFOB"))
    {
        g_printerr("rigctld probe did not cache expected VFO tokens\n");
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
        static const gchar *sub_tokens[] =
            { "Sub", "SubA", "VFO_SUB", "VFOB", NULL };
        static const gchar *main_tokens[] =
            { "Main", "MainA", "VFO_MAIN", "VFOA", NULL };
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

        rig_log_capture_clear(&rig_select_log);

        if (!rigctld_client_set_freq(rig_select, VFO_SUB, 145910000))
        {
            g_printerr("rigctld set freq failed (select-vfo)\n");
            ok = FALSE;
            goto cleanup;
        }
        if (!rig_log_capture_contains(&rig_select_log,
                                      "gpredict:tx [mock-rig-select] V ") ||
            !rig_log_capture_contains(&rig_select_log,
                                      "gpredict:tx [mock-rig-select] F 145910000") ||
            !rig_log_capture_contains(&rig_select_log,
                                      "gpredict:rx [mock-rig-select] RPRT 0"))
        {
            g_printerr("rigctld internal select/set logging missing expected tx/rx lines\n");
            rig_log_capture_dump(&rig_select_log, "select/set log");
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
        if (!command_matches_any_token(lines[idx_v_sub], sub_tokens) ||
            !command_matches_any_token(lines[idx_v_main], main_tokens))
        {
            g_printerr("rigctld VFO token mismatch: sub=%s main=%s\n",
                       lines[idx_v_sub], lines[idx_v_main]);
            ok = FALSE;
            g_strfreev(lines);
            goto cleanup;
        }

        g_strfreev(lines);
    }
    {
        HamlibResponseInfo info = { 0 };
        gchar reply[512] = { 0 };
        gchar **lines = NULL;
        static const gchar *sub_tokens[] =
            { "Sub", "SubA", "VFO_SUB", "VFOB", NULL };
        gint idx_v_first = -1;
        gint idx_f_first = -1;
        gint idx_v_second = -1;
        gint idx_f_second = -1;

        if (!rigctld_client_request_raw(rig_select, "\\reset_cmd_log",
                                        reply, sizeof(reply), &info))
        {
            g_printerr("rigctld cmd log reset failed (repeat select-vfo)\n");
            ok = FALSE;
            goto cleanup;
        }

        if (!rigctld_client_set_freq(rig_select, VFO_SUB, 145930000) ||
            !rigctld_client_set_freq(rig_select, VFO_SUB, 145930010))
        {
            g_printerr("rigctld repeated set freq failed (select-vfo)\n");
            ok = FALSE;
            goto cleanup;
        }

        if (!rigctld_client_request_raw(rig_select, "\\get_cmd_log",
                                        reply, sizeof(reply), &info))
        {
            g_printerr("rigctld cmd log query failed (repeat select-vfo)\n");
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
                if (idx_v_first == -1)
                    idx_v_first = i;
                else if (idx_v_second == -1)
                    idx_v_second = i;
                continue;
            }

            if (g_str_has_prefix(line, "F ") && idx_f_first == -1)
            {
                idx_f_first = i;
                continue;
            }
            if (g_str_has_prefix(line, "F ") && idx_f_second == -1)
            {
                idx_f_second = i;
                continue;
            }
        }

        if (idx_v_first < 0 || idx_f_first < idx_v_first ||
            idx_v_second < 0 || idx_f_second < idx_v_second ||
            idx_v_second < idx_f_first)
        {
            g_printerr("rigctld repeated SELECT_VFO sequence mismatch\n");
            ok = FALSE;
            g_strfreev(lines);
            goto cleanup;
        }
        if (!command_matches_any_token(lines[idx_v_first], sub_tokens) ||
            !command_matches_any_token(lines[idx_v_second], sub_tokens))
        {
            g_printerr("rigctld repeated SELECT_VFO token mismatch: first=%s second=%s\n",
                       lines[idx_v_first], lines[idx_v_second]);
            ok = FALSE;
            g_strfreev(lines);
            goto cleanup;
        }

        g_strfreev(lines);
    }
    {
        HamlibResponseInfo info = { 0 };
        gchar reply[512] = { 0 };
        gchar **lines = NULL;
        static const gchar *sub_tokens[] =
            { "Sub", "SubA", "VFO_SUB", "VFOB", NULL };
        static const gchar *main_tokens[] =
            { "Main", "MainA", "VFO_MAIN", "VFOA", NULL };
        gboolean saw_bad_tokenized_set = FALSE;
        gint idx_v_sub = -1;
        gint idx_f_set = -1;
        gint idx_v_main = -1;
        gint idx_f_get = -1;

        rigctld_client_caps_snapshot_free(reject_caps);
        reject_caps = rigctld_client_get_caps_snapshot(rig_reject);
        if (reject_caps == NULL ||
            reject_caps->strategy != RIG_STRATEGY_SELECT_VFO)
        {
            g_printerr("rigctld probe should have downgraded to SELECT_VFO when Main/Sub tokenized retune is rejected\n");
            ok = FALSE;
            goto cleanup;
        }

        if (!rigctld_client_request_raw(rig_reject, "\\reset_cmd_log",
                                        reply, sizeof(reply), &info))
        {
            g_printerr("rigctld cmd log reset failed (tokenized Main/Sub retune reject)\n");
            ok = FALSE;
            goto cleanup;
        }

        if (!rigctld_client_set_freq(rig_reject, VFO_SUB, 145920000))
        {
            g_printerr("rigctld set freq failed after SELECT_VFO downgrade\n");
            ok = FALSE;
            goto cleanup;
        }

        if (!rigctld_client_get_freq(rig_reject, VFO_MAIN, &freq))
        {
            g_printerr("rigctld get freq failed after SELECT_VFO downgrade\n");
            ok = FALSE;
            goto cleanup;
        }

        if (!rigctld_client_request_raw(rig_reject, "\\get_cmd_log",
                                        reply, sizeof(reply), &info))
        {
            g_printerr("rigctld cmd log query failed (tokenized Main/Sub retune reject)\n");
            ok = FALSE;
            goto cleanup;
        }

        lines = g_strsplit(reply, "\n", -1);
        for (gint i = 0; lines[i] != NULL; i++)
        {
            const gchar *line = lines[i];

            if (line[0] == '\0' || g_str_has_prefix(line, "RPRT"))
                continue;

            if (g_strcmp0(line, "F Sub 145920000") == 0 ||
                g_strcmp0(line, "F Main 145920000") == 0 ||
                g_strcmp0(line, "f Main") == 0 ||
                g_strcmp0(line, "f Sub") == 0)
            {
                saw_bad_tokenized_set = TRUE;
            }

            if (g_str_has_prefix(line, "V "))
            {
                if (idx_v_sub == -1)
                    idx_v_sub = i;
                else if (idx_v_main == -1)
                    idx_v_main = i;
                continue;
            }

            if (g_str_has_prefix(line, "F ") && idx_f_set == -1)
            {
                idx_f_set = i;
                continue;
            }
            if (g_strcmp0(line, "f") == 0 && idx_f_get == -1)
            {
                idx_f_get = i;
                continue;
            }
        }

        if (saw_bad_tokenized_set)
        {
            g_printerr("rigctld should not issue tokenized Main/Sub freq ops after downgrade\n");
            ok = FALSE;
            g_strfreev(lines);
            goto cleanup;
        }
        if (idx_v_sub < 0 || idx_f_set < idx_v_sub ||
            idx_v_main < 0 || idx_f_get < idx_v_main)
        {
            g_printerr("rigctld downgraded SELECT_VFO sequence mismatch\n");
            ok = FALSE;
            g_strfreev(lines);
            goto cleanup;
        }
        if (!command_matches_any_token(lines[idx_v_sub], sub_tokens) ||
            !command_matches_any_token(lines[idx_v_main], main_tokens))
        {
            g_printerr("rigctld downgraded SELECT_VFO token mismatch: sub=%s main=%s\n",
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
    {
        memset(&rcaps_snapshot, 0, sizeof(rcaps_snapshot));
        rcaps = rotctld_client_get_caps_snapshot(rot, &rcaps_snapshot)
                    ? &rcaps_snapshot
                    : NULL;
    }
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
        RotThreadCtx ctx_get = { rot, TRUE, 25 };
        RotThreadCtx ctx_set = { rot, TRUE, 25 };
        GThread *t_get = NULL;
        GThread *t_set = NULL;

        t_get = g_thread_new("rot-getpos", rot_thread_getpos, &ctx_get);
        t_set = g_thread_new("rot-setpos", rot_thread_setpos, &ctx_set);
        g_thread_join(t_get);
        g_thread_join(t_set);

        if (!ctx_get.ok || !ctx_set.ok)
        {
            g_printerr("rotctld concurrent request test failed\n");
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
    if (!rotctld_client_set_pos(rot, 15.0, 5.0))
    {
        g_printerr("rotctld set pos failed (restore)\n");
        ok = FALSE;
        goto cleanup;
    }
    if (!rotctld_client_get_pos(rot, &az, &el))
    {
        g_printerr("rotctld get pos failed (post-restore)\n");
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
    rot_split_port = pick_free_port();
    for (gint attempt = 0;
         attempt < 5 &&
         (rot_split_port == 0 ||
          rot_split_port == rig_port ||
          rot_split_port == rot_port ||
          rot_split_port == rot_fail_port);
         attempt++)
        rot_split_port = pick_free_port();
    rot_drop_port = pick_free_port();
    for (gint attempt = 0;
         attempt < 5 &&
         (rot_drop_port == 0 ||
          rot_drop_port == rig_port ||
          rot_drop_port == rot_port ||
          rot_drop_port == rot_fail_port ||
          rot_drop_port == rot_split_port);
         attempt++)
        rot_drop_port = pick_free_port();
    if (rot_fail_port == 0 ||
        rot_fail_port == rig_port ||
        rot_fail_port == rot_port)
    {
        g_printerr("failed to select mock port for rotctld failure test\n");
        ok = FALSE;
        goto cleanup;
    }
    if (rot_split_port == 0 || rot_drop_port == 0)
    {
        g_printerr("failed to select mock port for rotctld split/drop test\n");
        ok = FALSE;
        goto cleanup;
    }

    rot_fail_proc = spawn_rotctld_mock(python, rot_script, rot_fail_port,
                                       TRUE, TRUE, FALSE, FALSE, 0, TRUE);
    if (rot_fail_proc == NULL)
    {
        ok = FALSE;
        goto cleanup;
    }

    rot_split_proc = spawn_rotctld_mock(python, rot_script, rot_split_port,
                                        FALSE, FALSE, FALSE, TRUE, 0, TRUE);
    rot_drop_proc = spawn_rotctld_mock(python, rot_script, rot_drop_port,
                                       FALSE, FALSE, FALSE, FALSE, 2, TRUE);
    if (rot_split_proc == NULL || rot_drop_proc == NULL)
    {
        ok = FALSE;
        goto cleanup;
    }

    rot_fail = rotctld_client_new("mock-rot-fail");
    rot_split = rotctld_client_new("mock-rot-split");
    rot_drop = rotctld_client_new("mock-rot-drop");
    if (rot_fail == NULL)
    {
        ok = FALSE;
        goto cleanup;
    }
    if (rot_split == NULL || rot_drop == NULL)
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
        gdouble hs_az = 0.0;
        gdouble hs_el = 0.0;
        gchar dump_state[1024];
        gchar pos_reply[128];
        gboolean pos_ok = FALSE;

        dump_state[0] = '\0';
        pos_reply[0] = '\0';
        if (!rotctld_client_handshake(rot_fail,
                                      500,
                                      &hs_az,
                                      &hs_el,
                                      dump_state,
                                      sizeof(dump_state),
                                      pos_reply,
                                      sizeof(pos_reply),
                                      &pos_ok))
        {
            g_printerr("rotctld handshake failed in failure mode\n");
            ok = FALSE;
            goto cleanup;
        }
        if (pos_ok)
        {
            g_printerr("rotctld handshake unexpected pos_ok in failure mode\n");
            ok = FALSE;
            goto cleanup;
        }
        if (rotctld_client_get_state(rot_fail) == ROTCTLD_CLIENT_READY)
        {
            g_printerr("rotctld handshake failure should not set READY state\n");
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

    if (!connect_rotctld_with_retry(rot_split, "127.0.0.1", rot_split_port))
    {
        g_printerr("failed to connect to rotctld mock (split replies)\n");
        ok = FALSE;
        goto cleanup;
    }
    for (gint i = 0; i < 3; i++)
    {
        if (!rotctld_client_get_pos(rot_split, &az, &el))
        {
            g_printerr("rotctld get pos failed with split replies\n");
            ok = FALSE;
            goto cleanup;
        }
    }
    if (!rotctld_client_set_pos(rot_split, 25.0, 10.0))
    {
        g_printerr("rotctld set pos failed with split replies\n");
        ok = FALSE;
        goto cleanup;
    }
    {
        RotThreadCtx ctx_get = { rot_split, TRUE, 200 };
        RotThreadCtx ctx_set = { rot_split, TRUE, 200 };
        GThread *t_get = NULL;
        GThread *t_set = NULL;

        t_get = g_thread_new("rot-split-getpos", rot_thread_getpos, &ctx_get);
        t_set = g_thread_new("rot-split-setpos", rot_thread_setpos, &ctx_set);
        g_thread_join(t_get);
        g_thread_join(t_set);

        if (!ctx_get.ok || !ctx_set.ok)
        {
            g_printerr("rotctld split-reply concurrent request test failed\n");
            ok = FALSE;
            goto cleanup;
        }
    }

    if (!connect_rotctld_with_retry(rot_drop, "127.0.0.1", rot_drop_port))
    {
        g_printerr("failed to connect to rotctld mock (disconnect)\n");
        ok = FALSE;
        goto cleanup;
    }
    if (!rotctld_client_get_pos(rot_drop, &az, &el))
    {
        g_printerr("rotctld get pos failed before disconnect\n");
        ok = FALSE;
        goto cleanup;
    }
    (void)rotctld_client_set_pos(rot_drop, 12.0, 7.0);
    if (rotctld_client_get_pos(rot_drop, &az, &el))
    {
        g_printerr("rotctld disconnect test expected failure after drop\n");
        ok = FALSE;
        goto cleanup;
    }
    rotctld_client_close(rot_drop);
    if (rot_drop_proc != NULL)
    {
        if (!g_subprocess_wait_check(rot_drop_proc, NULL, &error))
        {
            g_printerr("rotctld mock disconnect exit error: %s\n",
                       error ? error->message : "unknown");
            ok = FALSE;
            g_clear_error(&error);
            goto cleanup;
        }
        g_clear_object(&rot_drop_proc);
    }
    rot_drop_proc = spawn_rotctld_mock(python, rot_script, rot_drop_port,
                                       FALSE, FALSE, FALSE, FALSE, 0, TRUE);
    if (rot_drop_proc == NULL)
    {
        ok = FALSE;
        goto cleanup;
    }
    if (!connect_rotctld_with_retry(rot_drop, "127.0.0.1", rot_drop_port))
    {
        g_printerr("failed to reconnect to rotctld mock after drop\n");
        ok = FALSE;
        goto cleanup;
    }
    if (!rotctld_client_get_pos(rot_drop, &az, &el))
    {
        g_printerr("rotctld get pos failed after reconnect\n");
        ok = FALSE;
        goto cleanup;
    }

cleanup:
    rigctld_client_caps_snapshot_free(caps);
    rigctld_client_caps_snapshot_free(reject_caps);
    rigctld_client_caps_snapshot_free(select_only_caps);
    rigctld_client_caps_snapshot_free(vfo_opt_hidden_caps);
    rigctld_client_close(rig);
    rigctld_client_close(rig_select);
    rigctld_client_close(rig_reject);
    rigctld_client_close(rig_missing_sub);
    rigctld_client_close(rig_select_only);
    rigctld_client_close(rig_vfo_opt_hidden);
    rotctld_client_close(rot);
    rotctld_client_close(rot_fail);
    rotctld_client_close(rot_split);
    rotctld_client_close(rot_drop);
    rigctld_client_free(&rig);
    rigctld_client_free(&rig_select);
    rigctld_client_free(&rig_reject);
    rigctld_client_free(&rig_missing_sub);
    rigctld_client_free(&rig_select_only);
    rigctld_client_free(&rig_vfo_opt_hidden);
    rotctld_client_free(&rot);
    rotctld_client_free(&rot_fail);
    rotctld_client_free(&rot_split);
    rotctld_client_free(&rot_drop);

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
    if (rig_reject_proc != NULL && ok &&
        !g_subprocess_wait_check(rig_reject_proc, NULL, &error))
    {
        g_printerr("rigctld mock (tokenized Main/Sub reject) exit error: %s\n",
                   error ? error->message : "unknown");
        ok = FALSE;
        g_clear_error(&error);
    }
    if (rig_missing_sub_proc != NULL && ok &&
        !g_subprocess_wait_check(rig_missing_sub_proc, NULL, &error))
    {
        g_printerr("rigctld mock (missing Sub control) exit error: %s\n",
                   error ? error->message : "unknown");
        ok = FALSE;
        g_clear_error(&error);
    }
    if (rig_select_only_proc != NULL && ok &&
        !g_subprocess_wait_check(rig_select_only_proc, NULL, &error))
    {
        g_printerr("rigctld mock (select-only) exit error: %s\n",
                   error ? error->message : "unknown");
        ok = FALSE;
        g_clear_error(&error);
    }
    if (rig_vfo_opt_hidden_proc != NULL && ok &&
        !g_subprocess_wait_check(rig_vfo_opt_hidden_proc, NULL, &error))
    {
        g_printerr("rigctld mock (hidden vfo-opt) exit error: %s\n",
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
    if (rot_split_proc != NULL && ok &&
        !g_subprocess_wait_check(rot_split_proc, NULL, &error))
    {
        g_printerr("rotctld mock split exit error: %s\n",
                   error ? error->message : "unknown");
        ok = FALSE;
        g_clear_error(&error);
    }
    if (rot_drop_proc != NULL && ok &&
        !g_subprocess_wait_check(rot_drop_proc, NULL, &error))
    {
        g_printerr("rotctld mock disconnect exit error: %s\n",
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
    if (rig_reject_proc != NULL && !ok)
    {
        g_subprocess_force_exit(rig_reject_proc);
        (void)g_subprocess_wait(rig_reject_proc, NULL, NULL);
    }
    if (rig_missing_sub_proc != NULL && !ok)
    {
        g_subprocess_force_exit(rig_missing_sub_proc);
        (void)g_subprocess_wait(rig_missing_sub_proc, NULL, NULL);
    }
    if (rig_select_only_proc != NULL && !ok)
    {
        g_subprocess_force_exit(rig_select_only_proc);
        (void)g_subprocess_wait(rig_select_only_proc, NULL, NULL);
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
    if (rot_split_proc != NULL && !ok)
    {
        g_subprocess_force_exit(rot_split_proc);
        (void)g_subprocess_wait(rot_split_proc, NULL, NULL);
    }
    if (rot_drop_proc != NULL && !ok)
    {
        g_subprocess_force_exit(rot_drop_proc);
        (void)g_subprocess_wait(rot_drop_proc, NULL, NULL);
    }

    g_clear_object(&rig_proc);
    g_clear_object(&rig_select_proc);
    g_clear_object(&rig_reject_proc);
    g_clear_object(&rig_missing_sub_proc);
    g_clear_object(&rig_select_only_proc);
    g_clear_object(&rot_proc);
    g_clear_object(&rot_fail_proc);
    g_clear_object(&rot_split_proc);
    g_clear_object(&rot_drop_proc);
    if (rig_probe_log.lines != NULL)
        g_ptr_array_free(rig_probe_log.lines, TRUE);
    if (rig_select_log.lines != NULL)
        g_ptr_array_free(rig_select_log.lines, TRUE);
    g_free(python);
    g_free(rig_script);
    g_free(rot_script);

    return ok ? 0 : 1;
}
