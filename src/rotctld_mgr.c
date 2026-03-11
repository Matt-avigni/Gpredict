/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include "rotctld_mgr.h"
#include "sat-log.h"

#include <gio/gio.h>
#include <glib/gi18n.h>
#include <string.h>
#include <limits.h>

#ifdef G_OS_UNIX
#include <unistd.h>
#endif

#ifdef G_OS_UNIX
#include <signal.h>
#endif

#ifndef G_SUBPROCESS_FLAGS_STDIN_DEV_NULL
#ifdef G_SUBPROCESS_FLAGS_STDIN_INHERIT
#define G_SUBPROCESS_FLAGS_STDIN_DEV_NULL G_SUBPROCESS_FLAGS_STDIN_INHERIT
#else
#define G_SUBPROCESS_FLAGS_STDIN_DEV_NULL 0
#endif
#endif

#define ROTCTLD_LOG_MAX_LINES 50

static gchar *rotctld_mgr_argv_to_shell_string(const gchar * const *argv)
{
    GString *buf = NULL;

    if (argv == NULL || argv[0] == NULL)
        return NULL;

    buf = g_string_new(NULL);
    for (gint i = 0; argv[i] != NULL; i++)
    {
        gchar *quoted = g_shell_quote(argv[i]);
        if (i > 0)
            g_string_append_c(buf, ' ');
        g_string_append(buf, quoted);
        g_free(quoted);
    }

    return g_string_free(buf, FALSE);
}

struct _RotctldMgr {
    GSubprocess *proc;
    gchar       *identifier;
    GThread     *stdout_thread;
    GThread     *stderr_thread;
    GThread     *exit_thread;
    GMutex       log_lock;
    GQueue      *stdout_lines;
    GQueue      *stderr_lines;
    RotctldMgrLogFunc log_cb;
    gpointer     log_cb_data;
    gboolean     proc_exited;
    gboolean     exit_ready;
    gboolean     exit_reported;
    gint         exit_status;
    gint         exit_signal;
};

typedef struct {
    RotctldMgr  *mgr;
    GDataInputStream *stream;
    const gchar *prefix;
    GQueue      *queue;
} RotctldLogReader;

typedef struct {
    RotctldMgr  *mgr;
    GMainLoop   *loop;
} RotctldMgrWaitCtx;

static gchar *rotctld_mgr_preferred_hamlib_libdir(void)
{
#ifdef __APPLE__
    const gchar *home = g_get_home_dir();
    gchar *candidate = NULL;

    if (home == NULL || *home == '\0')
        return NULL;

    candidate = g_build_filename(home, "hamlib-local", "lib", NULL);
    if (g_file_test(candidate, G_FILE_TEST_IS_DIR))
        return candidate;

    g_free(candidate);
    return NULL;
#else
    return NULL;
#endif
}

static gchar *rotctld_mgr_resolve_rotctld_path(gchar **source_out,
                                               gchar **error_out)
{
    const gchar *env_path = g_getenv("ROTCTLD_BIN");

    if (source_out)
        *source_out = NULL;

    if (env_path && *env_path)
    {
        if (!g_path_is_absolute(env_path))
        {
            if (error_out)
                *error_out = g_strdup_printf(
                    "ROTCTLD_BIN must be an absolute path: %s", env_path);
            return NULL;
        }
        if (!g_file_test(env_path, G_FILE_TEST_IS_EXECUTABLE))
        {
            if (error_out)
                *error_out = g_strdup_printf(
                    "ROTCTLD_BIN path is not executable: %s", env_path);
            return NULL;
        }
        if (source_out)
            *source_out = g_strdup("ROTCTLD_BIN");
        return g_strdup(env_path);
    }

    {
        gchar *path = g_find_program_in_path("rotctld");
        if (path == NULL)
        {
            if (error_out)
                *error_out = g_strdup("rotctld not found in PATH.");
            return NULL;
        }
        if (source_out)
            *source_out = g_strdup("PATH");
        return path;
    }
}

static void rotctld_mgr_log_version(const gchar *path)
{
    gchar *stdout_data = NULL;
    gchar *stderr_data = NULL;
    gint status = 0;
    GError *error = NULL;
    gchar *argv[] = { (gchar *) path, "-V", NULL };
    const gchar *output = NULL;
    gchar *line = NULL;

    if (path == NULL || *path == '\0')
        return;

    if (!g_spawn_sync(NULL, argv, NULL, 0, NULL, NULL,
                      &stdout_data, &stderr_data, &status, &error))
    {
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rotctld_mgr: rotctld -V failed: %s",
                    error ? error->message : "unknown error");
        g_clear_error(&error);
        g_free(stdout_data);
        g_free(stderr_data);
        return;
    }

    if (stdout_data && *stdout_data)
        output = stdout_data;
    else if (stderr_data && *stderr_data)
        output = stderr_data;

    if (output && *output)
    {
        const gchar *newline = strchr(output, '\n');
        if (newline)
            line = g_strndup(output, (gsize)(newline - output));
        else
            line = g_strdup(output);
    }

    if (line && *line)
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "rotctld_mgr: rotctld -V: %s", line);
    else if (status != 0)
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rotctld_mgr: rotctld -V exited with status %d", status);

    g_free(line);
    g_free(stdout_data);
    g_free(stderr_data);
}

static gchar *prepend_path_env_if_missing(const gchar *existing,
                                          const gchar *path)
{
    gchar **parts = NULL;
    gchar *updated = NULL;

    if (path == NULL || *path == '\0')
        return NULL;

    if (existing == NULL || *existing == '\0')
        return g_strdup(path);

    parts = g_strsplit(existing, ":", -1);
    for (gint i = 0; parts != NULL && parts[i] != NULL; i++)
    {
        if (g_strcmp0(parts[i], path) == 0)
        {
            g_strfreev(parts);
            return NULL;
        }
    }
    g_strfreev(parts);

    updated = g_strdup_printf("%s:%s", path, existing);
    return updated;
}

gboolean rotctld_mgr_host_is_local(const gchar *host)
{
    if (host == NULL || *host == '\0')
        return FALSE;

    if (g_ascii_strcasecmp(host, "localhost") == 0)
        return TRUE;
    if (g_strcmp0(host, "127.0.0.1") == 0)
        return TRUE;
    if (g_strcmp0(host, "::1") == 0)
        return TRUE;

    return FALSE;
}

static const gchar *rotctld_mgr_bind_host(const gchar *host)
{
    if (host == NULL || *host == '\0')
        return "127.0.0.1";

    if (rotctld_mgr_host_is_local(host))
        return "127.0.0.1";

    return host;
}

static gboolean rotctld_mgr_dump_state_has_id(const gchar *text)
{
    gchar *lower = NULL;
    gboolean ok = FALSE;

    if (text == NULL || *text == '\0')
        return FALSE;

    lower = g_ascii_strdown(text, -1);
    if (lower == NULL)
        return FALSE;

    ok = (g_strrstr(lower, "rotator") != NULL) ||
         (g_strrstr(lower, "rot_model") != NULL) ||
         (g_strrstr(lower, "azimuth") != NULL) ||
         (g_strrstr(lower, "elevation") != NULL);

    g_free(lower);
    return ok;
}

static gboolean rotctld_mgr_probe_dump_state(GSocket *sock, gint timeout_ms)
{
    const gchar *cmd = "\\dump_state\n";
    gchar buffer[1024];
    GError *error = NULL;
    gssize size;
    gint64 timeout_us;

    if (sock == NULL)
        return FALSE;

    if (timeout_ms <= 0)
        timeout_ms = 100;

    timeout_us = (gint64) timeout_ms * 1000;
    g_socket_set_blocking(sock, FALSE);

    size = g_socket_send(sock, cmd, strlen(cmd), NULL, &error);
    if (size < 0)
    {
        if (g_error_matches(error, G_IO_ERROR, G_IO_ERROR_WOULD_BLOCK))
        {
            g_clear_error(&error);
            if (!g_socket_condition_timed_wait(sock, G_IO_OUT, timeout_us,
                                               NULL, &error))
            {
                g_clear_error(&error);
                return FALSE;
            }
            size = g_socket_send(sock, cmd, strlen(cmd), NULL, &error);
        }
    }
    if (size < 0)
    {
        g_clear_error(&error);
        return FALSE;
    }

    if (!g_socket_condition_timed_wait(sock, G_IO_IN, timeout_us, NULL, &error))
    {
        g_clear_error(&error);
        return FALSE;
    }

    size = g_socket_receive(sock, buffer, sizeof(buffer) - 1, NULL, &error);
    if (size <= 0)
    {
        g_clear_error(&error);
        return FALSE;
    }

    buffer[size] = '\0';
    return rotctld_mgr_dump_state_has_id(buffer);
}

static gboolean rotctld_mgr_port_probe(const gchar *host, gint port,
                                       gint timeout_ms,
                                       gboolean require_dump_state,
                                       gboolean *unresponsive)
{
    gboolean        ok = FALSE;
    gboolean        resolved = FALSE;
    GResolver      *resolver = NULL;
    GList          *addrs = NULL;
    GError         *error = NULL;
    gint64          timeout_us;
    gint            probe_timeout_ms;

    if (unresponsive != NULL)
        *unresponsive = FALSE;

    if (host == NULL || *host == '\0' || port <= 0)
        return FALSE;

    if (rotctld_mgr_host_is_local(host))
    {
        GInetAddress *ipv4 = g_inet_address_new_from_string("127.0.0.1");
        GInetAddress *ipv6 = NULL;

        if (g_ascii_strcasecmp(host, "127.0.0.1") != 0)
        {
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rotctld_mgr: host %s treated as local; probing 127.0.0.1 first",
                        host);
            ipv6 = g_inet_address_new_from_string("::1");
        }

        if (ipv4)
            addrs = g_list_append(addrs, ipv4);
        if (ipv6)
            addrs = g_list_append(addrs, ipv6);
    }
    else
    {
        resolver = g_resolver_get_default();
        addrs = g_resolver_lookup_by_name(resolver, host, NULL, &error);
        if (addrs == NULL)
        {
            g_clear_error(&error);
            g_object_unref(resolver);
            return FALSE;
        }
        resolved = TRUE;
    }

    if (addrs == NULL)
    {
        if (resolver)
            g_object_unref(resolver);
        return FALSE;
    }

    timeout_us = (gint64) timeout_ms * 1000;
    probe_timeout_ms = timeout_ms;
    if (probe_timeout_ms < 100)
        probe_timeout_ms = 100;
    else if (probe_timeout_ms > 1000)
        probe_timeout_ms = 1000;

    for (GList *iter = addrs; iter != NULL; iter = iter->next)
    {
        GInetAddress   *addr = G_INET_ADDRESS(iter->data);
        GSocket        *sock = NULL;
        GSocketAddress *sockaddr = NULL;
        gboolean        connected = FALSE;
        gchar          *addr_text = NULL;

        addr_text = g_inet_address_to_string(addr);
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rotctld_mgr: probing %s:%d via %s",
                    host, port,
                    addr_text ? addr_text : "(unknown)");
        g_free(addr_text);

        sock = g_socket_new(g_inet_address_get_family(addr),
                            G_SOCKET_TYPE_STREAM,
                            G_SOCKET_PROTOCOL_TCP, &error);
        if (sock == NULL)
        {
            g_clear_error(&error);
            continue;
        }

        g_socket_set_blocking(sock, FALSE);
        sockaddr = g_inet_socket_address_new(addr, port);

        if (g_socket_connect(sock, sockaddr, NULL, &error))
        {
            connected = TRUE;
        }
        else if (g_error_matches(error, G_IO_ERROR, G_IO_ERROR_PENDING))
        {
            g_clear_error(&error);
            if (g_socket_condition_timed_wait(sock, G_IO_OUT, timeout_us, NULL,
                                              &error))
            {
                if (g_socket_check_connect_result(sock, &error))
                    connected = TRUE;
                else
                    g_clear_error(&error);
            }
            else
            {
                g_clear_error(&error);
            }
        }
        else
        {
            g_clear_error(&error);
        }

        if (connected)
        {
            if (!require_dump_state)
            {
                ok = TRUE;
            }
            else if (rotctld_mgr_probe_dump_state(sock, probe_timeout_ms))
            {
                ok = TRUE;
            }
            else if (unresponsive != NULL)
            {
                *unresponsive = TRUE;
            }
        }

        g_object_unref(sockaddr);
        g_object_unref(sock);

        if (ok)
            break;
    }

    if (resolved)
    {
        g_resolver_free_addresses(addrs);
        g_object_unref(resolver);
    }
    else
    {
        g_list_free_full(addrs, g_object_unref);
    }

    return ok;
}

gboolean rotctld_mgr_wait_for_port(const gchar *host, gint port,
                                   gint timeout_ms)
{
    gint interval_ms = 100;
    gboolean unresponsive = FALSE;
    gint64 start_us = 0;
    gint64 deadline_us = 0;

    if (timeout_ms <= 0)
        return FALSE;

    start_us = g_get_monotonic_time();
    deadline_us = start_us + ((gint64) timeout_ms * 1000);

    while (g_get_monotonic_time() < deadline_us)
    {
        gint64 loop_start_us = g_get_monotonic_time();
        gint64 remaining_us = deadline_us - loop_start_us;
        gint slice = interval_ms;

        if (remaining_us <= 0)
            break;

        if (slice > (remaining_us / 1000))
            slice = (gint) (remaining_us / 1000);
        if (slice <= 0)
            slice = 1;

        if (rotctld_mgr_port_probe(host, port, slice, TRUE, &unresponsive))
            return TRUE;

        {
            gint64 loop_end_us = g_get_monotonic_time();
            gint64 elapsed_us = loop_end_us - loop_start_us;
            gint64 sleep_us = ((gint64) interval_ms * 1000) - elapsed_us;
            gint64 remaining_us = deadline_us - loop_end_us;

            if (sleep_us > remaining_us)
                sleep_us = remaining_us;

            if (sleep_us > 0)
                g_usleep(sleep_us);
        }
    }

    if (unresponsive)
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    "rotctld_mgr: port open but not responding to \\dump_state");
    return FALSE;
}

gboolean rotctld_mgr_wait_for_listen(const gchar *host, gint port,
                                     gint timeout_ms)
{
    gint interval_ms = 100;
    gint64 start_us = 0;
    gint64 deadline_us = 0;

    if (timeout_ms <= 0)
        return FALSE;

    start_us = g_get_monotonic_time();
    deadline_us = start_us + ((gint64) timeout_ms * 1000);

    while (g_get_monotonic_time() < deadline_us)
    {
        gint64 loop_start_us = g_get_monotonic_time();
        gint64 remaining_us = deadline_us - loop_start_us;
        gint slice = interval_ms;

        if (remaining_us <= 0)
            break;

        if (slice > (remaining_us / 1000))
            slice = (gint) (remaining_us / 1000);
        if (slice <= 0)
            slice = 1;

        if (rotctld_mgr_port_probe(host, port, slice, FALSE, NULL))
            return TRUE;

        {
            gint64 loop_end_us = g_get_monotonic_time();
            gint64 elapsed_us = loop_end_us - loop_start_us;
            gint64 sleep_us = ((gint64) interval_ms * 1000) - elapsed_us;
            gint64 remaining_us = deadline_us - loop_end_us;

            if (sleep_us > remaining_us)
                sleep_us = remaining_us;

            if (sleep_us > 0)
                g_usleep(sleep_us);
        }
    }

    return FALSE;
}

static void rotctld_mgr_store_line(GQueue *queue, const gchar *line)
{
    if (queue == NULL || line == NULL || *line == '\0')
        return;

    g_queue_push_tail(queue, g_strdup(line));
    while (queue->length > ROTCTLD_LOG_MAX_LINES)
        g_free(g_queue_pop_head(queue));
}

static void rotctld_mgr_emit_log(RotctldMgr *mgr, const gchar *prefix,
                                 const gchar *line)
{
    RotctldMgrLogFunc cb = NULL;
    gpointer data = NULL;

    if (mgr == NULL || prefix == NULL || line == NULL)
        return;

    g_mutex_lock(&mgr->log_lock);
    cb = mgr->log_cb;
    data = mgr->log_cb_data;
    g_mutex_unlock(&mgr->log_lock);

    if (cb)
        cb(mgr, prefix, line, data);
}

static void rotctld_mgr_emit_exit_if_ready(RotctldMgr *mgr)
{
    RotctldMgrLogFunc cb = NULL;
    gpointer data = NULL;
    gboolean ready = FALSE;
    gboolean reported = FALSE;
    gint status = -1;
    gint sig = 0;

    if (mgr == NULL)
        return;

    g_mutex_lock(&mgr->log_lock);
    ready = mgr->exit_ready;
    reported = mgr->exit_reported;
    cb = mgr->log_cb;
    data = mgr->log_cb_data;
    status = mgr->exit_status;
    sig = mgr->exit_signal;
    if (ready && !reported && cb != NULL)
        mgr->exit_reported = TRUE;
    g_mutex_unlock(&mgr->log_lock);

    if (ready && !reported && cb != NULL)
    {
        gchar *line = g_strdup_printf("exited status=%d signal=%d",
                                      status, sig);
        cb(mgr, "rotctld", line, data);
        g_free(line);
    }
}

static void rotctld_mgr_wait_cb(GObject *source, GAsyncResult *res,
                                gpointer data)
{
    RotctldMgrWaitCtx *ctx = data;
    RotctldMgr *mgr = ctx ? ctx->mgr : NULL;
    GSubprocess *proc = G_SUBPROCESS(source);
    GError *error = NULL;
    gint status = -1;
    gint sig = 0;

    if (mgr == NULL || proc == NULL)
    {
        if (ctx && ctx->loop)
            g_main_loop_quit(ctx->loop);
        return;
    }

    gboolean waited_ok = g_subprocess_wait_finish(proc, res, &error);
    if (!waited_ok)
    {
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rotctld_mgr: wait failed: %s",
                    error ? error->message : "unknown error");
        g_clear_error(&error);
    }

    if (waited_ok && g_subprocess_get_if_exited(proc))
        status = g_subprocess_get_exit_status(proc);
#if defined(G_OS_UNIX) && GLIB_CHECK_VERSION(2, 40, 0)
    if (waited_ok && g_subprocess_get_if_signaled(proc))
        sig = g_subprocess_get_term_sig(proc);
#endif

    g_mutex_lock(&mgr->log_lock);
    mgr->proc_exited = TRUE;
    mgr->exit_ready = TRUE;
    mgr->exit_status = status;
    mgr->exit_signal = sig;
    g_mutex_unlock(&mgr->log_lock);

    rotctld_mgr_emit_exit_if_ready(mgr);

    if (ctx->loop)
        g_main_loop_quit(ctx->loop);
}

static gpointer rotctld_mgr_wait_thread(gpointer data)
{
    RotctldMgr *mgr = data;
    GMainContext *context = NULL;
    GMainLoop *loop = NULL;
    RotctldMgrWaitCtx ctx;

    if (mgr == NULL || mgr->proc == NULL)
        return NULL;

    context = g_main_context_new();
    loop = g_main_loop_new(context, FALSE);
    ctx.mgr = mgr;
    ctx.loop = loop;

    g_main_context_push_thread_default(context);
    g_subprocess_wait_async(mgr->proc, NULL, rotctld_mgr_wait_cb, &ctx);
    g_main_loop_run(loop);
    g_main_context_pop_thread_default(context);

    g_main_loop_unref(loop);
    g_main_context_unref(context);
    return NULL;
}

static gpointer rotctld_mgr_read_stream(gpointer data)
{
    RotctldLogReader *reader = data;
    GError *error = NULL;
    gchar *line = NULL;
    gsize length = 0;

    if (reader == NULL || reader->mgr == NULL)
        return NULL;

    while ((line = g_data_input_stream_read_line(reader->stream, &length,
                                                 NULL, &error)) != NULL)
    {
        if (length > 0)
        {
            g_mutex_lock(&reader->mgr->log_lock);
            rotctld_mgr_store_line(reader->queue, line);
            g_mutex_unlock(&reader->mgr->log_lock);
            rotctld_mgr_emit_log(reader->mgr, reader->prefix, line);
        }
        g_free(line);
    }

    if (error != NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rotctld_mgr: log read failed: %s",
                    error->message);
        g_clear_error(&error);
    }

    g_object_unref(reader->stream);
    g_free(reader);
    return NULL;
}

static gchar *rotctld_mgr_join_lines(GQueue *queue)
{
    GString *buf = NULL;

    if (queue == NULL || g_queue_is_empty(queue))
        return NULL;

    buf = g_string_new(NULL);
    for (GList *iter = queue->head; iter != NULL; iter = iter->next)
    {
        g_string_append(buf, iter->data);
        if (iter->next)
            g_string_append_c(buf, '\n');
    }

    return g_string_free(buf, FALSE);
}

static RotctldMgr *rotctld_mgr_spawn_internal(GPtrArray *argv,
                                              const gchar *path,
                                              gchar **error_out)
{
    RotctldMgr *mgr = NULL;
    GSubprocessLauncher *launcher = NULL;
    GSubprocess *proc = NULL;
    GError *error = NULL;
    gchar *libdir = NULL;

    launcher = g_subprocess_launcher_new(G_SUBPROCESS_FLAGS_STDIN_DEV_NULL |
                                         G_SUBPROCESS_FLAGS_STDOUT_PIPE |
                                         G_SUBPROCESS_FLAGS_STDERR_PIPE);
    libdir = rotctld_mgr_preferred_hamlib_libdir();
    {
        const gchar *path_existing = g_getenv("PATH");
        const gchar *dyld_existing = g_getenv("DYLD_LIBRARY_PATH");
        const gchar *fallback_existing = g_getenv("DYLD_FALLBACK_LIBRARY_PATH");
        const gchar *dyld_final = dyld_existing;
        const gchar *fallback_final = fallback_existing;
        gchar *dyld_updated = NULL;
        gchar *fallback_updated = NULL;

        if (path_existing && *path_existing)
            g_subprocess_launcher_setenv(launcher, "PATH", path_existing, TRUE);

        if (libdir != NULL)
        {
            dyld_updated = prepend_path_env_if_missing(dyld_existing, libdir);
            if (dyld_updated != NULL)
            {
                g_subprocess_launcher_setenv(launcher, "DYLD_LIBRARY_PATH",
                                             dyld_updated, TRUE);
                dyld_final = dyld_updated;
            }

            fallback_updated =
                prepend_path_env_if_missing(fallback_existing, libdir);
            if (fallback_updated != NULL)
            {
                g_subprocess_launcher_setenv(launcher,
                                             "DYLD_FALLBACK_LIBRARY_PATH",
                                             fallback_updated, TRUE);
                fallback_final = fallback_updated;
            }
        }

        {
            gchar *cmdline =
                rotctld_mgr_argv_to_shell_string(
                    (const gchar * const *) argv->pdata);
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        "rotctld_mgr: spawn detail: path=%s argv=%s "
                        "PATH=%s DYLD_LIBRARY_PATH=%s "
                        "DYLD_FALLBACK_LIBRARY_PATH=%s",
                        path ? path : "(null)",
                        cmdline ? cmdline : "(null)",
                        (path_existing && *path_existing) ? path_existing
                                                          : "(unset)",
                        (dyld_final && *dyld_final) ? dyld_final : "(unset)",
                        (fallback_final && *fallback_final) ? fallback_final
                                                           : "(unset)");
            g_free(cmdline);
        }

        g_free(dyld_updated);
        g_free(fallback_updated);
    }

    proc = g_subprocess_launcher_spawnv(launcher,
                                        (const gchar * const *) argv->pdata,
                                        &error);
    g_object_unref(launcher);

    if (proc == NULL)
    {
        gchar *cmdline =
            rotctld_mgr_argv_to_shell_string(
                (const gchar * const *) argv->pdata);
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    "rotctld_mgr: spawn failed: %s (argv=%s)",
                    error ? error->message : "unknown error",
                    cmdline ? cmdline : "(null)");
        g_free(cmdline);
        if (error_out)
            *error_out = g_strdup(error ? error->message
                                        : "Failed to start rotctld.");
        g_clear_error(&error);
        g_free(libdir);
        return NULL;
    }

    sat_log_log(SAT_LOG_LEVEL_INFO,
                "rotctld_mgr: spawn success pid=%s path=%s",
                g_subprocess_get_identifier(proc),
                path ? path : "(null)");

    mgr = g_new0(RotctldMgr, 1);
    mgr->proc = proc;
    mgr->identifier = g_strdup(g_subprocess_get_identifier(proc));
    g_mutex_init(&mgr->log_lock);
    mgr->stdout_lines = g_queue_new();
    mgr->stderr_lines = g_queue_new();
    mgr->proc_exited = FALSE;
    mgr->exit_ready = FALSE;
    mgr->exit_reported = FALSE;
    mgr->exit_status = -1;
    mgr->exit_signal = 0;

    if (g_subprocess_get_stdout_pipe(proc))
    {
        RotctldLogReader *reader = g_new0(RotctldLogReader, 1);

        reader->mgr = mgr;
        reader->prefix = "rotctld:out";
        reader->queue = mgr->stdout_lines;
        reader->stream = g_data_input_stream_new(
            g_subprocess_get_stdout_pipe(proc));
        g_data_input_stream_set_newline_type(reader->stream,
                                             G_DATA_STREAM_NEWLINE_TYPE_ANY);
        mgr->stdout_thread =
            g_thread_new("rotctld-log-out", rotctld_mgr_read_stream, reader);
    }

    if (g_subprocess_get_stderr_pipe(proc))
    {
        RotctldLogReader *reader = g_new0(RotctldLogReader, 1);

        reader->mgr = mgr;
        reader->prefix = "rotctld:err";
        reader->queue = mgr->stderr_lines;
        reader->stream = g_data_input_stream_new(
            g_subprocess_get_stderr_pipe(proc));
        g_data_input_stream_set_newline_type(reader->stream,
                                             G_DATA_STREAM_NEWLINE_TYPE_ANY);
        mgr->stderr_thread =
            g_thread_new("rotctld-log-err", rotctld_mgr_read_stream, reader);
    }

    mgr->exit_thread =
        g_thread_new("rotctld-exit", rotctld_mgr_wait_thread, mgr);

    g_free(libdir);
    return mgr;
}

RotctldMgr *rotctld_mgr_spawn_argv(gchar **argv, gchar **error_out)
{
    GPtrArray *argv_copy = NULL;
    gchar *path = NULL;
    gchar *source = NULL;

    if (argv == NULL || argv[0] == NULL || argv[0][0] == '\0')
    {
        if (error_out)
            *error_out = g_strdup("Missing rotctld command.");
        return NULL;
    }

    path = rotctld_mgr_resolve_rotctld_path(&source, error_out);
    if (path == NULL)
        return NULL;

    sat_log_log(SAT_LOG_LEVEL_INFO,
                "rotctld_mgr: resolved rotctld path=%s source=%s",
                path, source ? source : "(unknown)");
    rotctld_mgr_log_version(path);

    argv_copy = g_ptr_array_new_with_free_func(g_free);
    g_ptr_array_add(argv_copy, g_strdup(path));
    for (gint i = 1; argv[i] != NULL; i++)
        g_ptr_array_add(argv_copy, g_strdup(argv[i]));
    g_ptr_array_add(argv_copy, NULL);

    {
        gchar *cmdline =
            rotctld_mgr_argv_to_shell_string(
                (const gchar * const *) argv_copy->pdata);
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("rotctld_mgr: spawn argv: %s"),
                    cmdline ? cmdline : "(null)");
        g_free(cmdline);
    }

    RotctldMgr *mgr = rotctld_mgr_spawn_internal(argv_copy, path, error_out);
    g_ptr_array_free(argv_copy, TRUE);
    g_free(path);
    g_free(source);
    return mgr;
}

RotctldMgr *rotctld_mgr_spawn_timeout(const gchar *host, gint port, gint model,
                                      const gchar *device, gint baud,
                                      gboolean verbose, gint timeout_ms,
                                      gchar **error_out)
{
    GPtrArray *argv = NULL;
    gchar *path = NULL;
    gchar *source = NULL;

    if (timeout_ms <= 0)
        timeout_ms = 1200;

    if (model <= 0)
    {
        if (error_out)
            *error_out = g_strdup("Missing rotctld model number.");
        return NULL;
    }

    if (device == NULL || *device == '\0')
    {
        if (error_out)
            *error_out = g_strdup("Missing rotctld device.");
        return NULL;
    }

    if (port <= 0)
    {
        if (error_out)
            *error_out = g_strdup("Missing rotctld port.");
        return NULL;
    }

    path = rotctld_mgr_resolve_rotctld_path(&source, error_out);
    if (path == NULL)
        return NULL;

    sat_log_log(SAT_LOG_LEVEL_INFO,
                "rotctld_mgr: resolved rotctld path=%s source=%s",
                path, source ? source : "(unknown)");
    rotctld_mgr_log_version(path);

    argv = g_ptr_array_new_with_free_func(g_free);
    g_ptr_array_add(argv, g_strdup(path));
    g_ptr_array_add(argv, g_strdup("-m"));
    g_ptr_array_add(argv, g_strdup_printf("%d", model));
    g_ptr_array_add(argv, g_strdup("-r"));
    g_ptr_array_add(argv, g_strdup(device));
    if (baud > 0)
    {
        g_ptr_array_add(argv, g_strdup("-s"));
        g_ptr_array_add(argv, g_strdup_printf("%d", baud));
    }
    g_ptr_array_add(argv, g_strdup("-C"));
    g_ptr_array_add(argv, g_strdup_printf("timeout=%d", timeout_ms));
    {
        const gchar *bind_host = rotctld_mgr_bind_host(host);
        if (host && *host && g_strcmp0(host, bind_host) != 0)
        {
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rotctld_mgr: host %s treated as local; binding to %s",
                        host, bind_host);
        }
        g_ptr_array_add(argv, g_strdup("-T"));
        g_ptr_array_add(argv, g_strdup(bind_host));
    }
    g_ptr_array_add(argv, g_strdup("-t"));
    g_ptr_array_add(argv, g_strdup_printf("%d", port));
    g_ptr_array_add(argv, g_strdup(verbose ? "-vvvv" : "-v"));
    g_ptr_array_add(argv, NULL);

    {
        gchar *cmdline =
            rotctld_mgr_argv_to_shell_string(
                (const gchar * const *) argv->pdata);
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("rotctld_mgr: spawn argv: %s"),
                    cmdline ? cmdline : "(null)");
        g_free(cmdline);
    }

    RotctldMgr *mgr = rotctld_mgr_spawn_internal(argv, path, error_out);
    g_ptr_array_free(argv, TRUE);
    g_free(path);
    g_free(source);
    return mgr;
}

RotctldMgr *rotctld_mgr_spawn(const gchar *host, gint port, gint model,
                              const gchar *device, gint baud,
                              gboolean verbose, gchar **error_out)
{
    return rotctld_mgr_spawn_timeout(host, port, model, device, baud,
                                     verbose, 1200, error_out);
}

gboolean rotctld_mgr_is_running(const RotctldMgr *mgr)
{
    if (mgr == NULL || mgr->proc == NULL)
        return FALSE;

    return !mgr->proc_exited;
}

const gchar *rotctld_mgr_get_identifier(const RotctldMgr *mgr)
{
    if (mgr == NULL)
        return NULL;

    if (mgr->identifier && *mgr->identifier)
        return mgr->identifier;

    if (mgr->proc == NULL)
        return NULL;

    return g_subprocess_get_identifier(mgr->proc);
}

gchar *rotctld_mgr_get_log_tail(RotctldMgr *mgr)
{
    gchar *copy = NULL;

    if (mgr == NULL)
        return NULL;

    g_mutex_lock(&mgr->log_lock);
    if (mgr->stderr_lines && !g_queue_is_empty(mgr->stderr_lines))
        copy = rotctld_mgr_join_lines(mgr->stderr_lines);
    else if (mgr->stdout_lines && !g_queue_is_empty(mgr->stdout_lines))
        copy = rotctld_mgr_join_lines(mgr->stdout_lines);
    g_mutex_unlock(&mgr->log_lock);

    return copy;
}

gboolean rotctld_mgr_get_exit_info(RotctldMgr *mgr,
                                   gint *status_out,
                                   gint *signal_out)
{
    gboolean ready = FALSE;
    gint status = -1;
    gint sig = 0;

    if (mgr == NULL || mgr->proc == NULL)
        return FALSE;

    g_mutex_lock(&mgr->log_lock);
    ready = mgr->exit_ready;
    if (ready)
    {
        status = mgr->exit_status;
        sig = mgr->exit_signal;
    }
    g_mutex_unlock(&mgr->log_lock);

    if (ready)
    {
        if (status_out)
            *status_out = status;
        if (signal_out)
            *signal_out = sig;
        rotctld_mgr_emit_exit_if_ready(mgr);
    }

    return ready;
}

void rotctld_mgr_set_log_callback(RotctldMgr *mgr,
                                  RotctldMgrLogFunc cb,
                                  gpointer user_data)
{
    if (mgr == NULL)
        return;

    g_mutex_lock(&mgr->log_lock);
    mgr->log_cb = cb;
    mgr->log_cb_data = user_data;
    g_mutex_unlock(&mgr->log_lock);

    rotctld_mgr_emit_exit_if_ready(mgr);
}

static void rotctld_mgr_wait_exit(RotctldMgr *mgr, gint timeout_ms)
{
    gint waited_ms = 0;

    if (mgr == NULL)
        return;

    while (waited_ms < timeout_ms)
    {
        if (mgr->proc_exited)
            return;

        g_usleep(100 * 1000);
        waited_ms += 100;
    }
}

void rotctld_mgr_terminate(RotctldMgr **mgr_ptr)
{
    RotctldMgr *mgr;

    if (mgr_ptr == NULL || *mgr_ptr == NULL)
        return;

    mgr = *mgr_ptr;

    if (mgr->proc != NULL)
    {
        if (!mgr->proc_exited)
        {
#if defined(G_OS_UNIX) && GLIB_CHECK_VERSION(2, 40, 0)
            g_subprocess_send_signal(mgr->proc, SIGTERM);
            rotctld_mgr_wait_exit(mgr, 1500);
#endif
            if (!mgr->proc_exited)
                g_subprocess_force_exit(mgr->proc);
        }

        g_subprocess_wait(mgr->proc, NULL, NULL);
    }

    if (mgr->stdout_thread)
        g_thread_join(mgr->stdout_thread);
    if (mgr->stderr_thread)
        g_thread_join(mgr->stderr_thread);
    if (mgr->exit_thread)
        g_thread_join(mgr->exit_thread);

    g_clear_object(&mgr->proc);
    g_free(mgr->identifier);
    mgr->identifier = NULL;

    g_mutex_clear(&mgr->log_lock);
    if (mgr->stdout_lines)
    {
        g_queue_free_full(mgr->stdout_lines, g_free);
        mgr->stdout_lines = NULL;
    }
    if (mgr->stderr_lines)
    {
        g_queue_free_full(mgr->stderr_lines, g_free);
        mgr->stderr_lines = NULL;
    }

    g_free(mgr);
    *mgr_ptr = NULL;
}
