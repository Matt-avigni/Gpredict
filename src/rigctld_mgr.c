#include "rigctld_mgr.h"
#include "sat-log.h"

#include <gio/gio.h>
#include <glib/gi18n.h>
#include <string.h>
#include <limits.h>

#ifdef G_OS_UNIX
#include <unistd.h>
#endif

#ifdef __APPLE__
#include <mach-o/dyld.h>
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

#define RIGCTLD_LOG_MAX_LEN 4096

struct _RigctldMgr {
    GSubprocess *proc;
    GThread     *stdout_thread;
    GThread     *stderr_thread;
    GThread     *exit_thread;
    GMutex       log_lock;
    GString     *log;
    RigctldMgrLogFunc log_cb;
    gpointer     log_cb_data;
    gboolean     exit_ready;
    gboolean     exit_reported;
    gint         exit_status;
    gint         exit_signal;
};

typedef struct {
    RigctldMgr  *mgr;
    GDataInputStream *stream;
    const gchar *prefix;
} RigctldLogReader;

static gchar *rigctld_mgr_find_bundled_rigctld(void)
{
    gchar *exe_path = NULL;
    gchar *dir = NULL;
    gchar *candidate = NULL;

#ifdef __APPLE__
    {
        uint32_t size = PATH_MAX;
        char buf[PATH_MAX];

        if (_NSGetExecutablePath(buf, &size) == 0)
            exe_path = g_strdup(buf);
        else
        {
            char *dyn = g_malloc(size);
            if (_NSGetExecutablePath(dyn, &size) == 0)
                exe_path = g_strdup(dyn);
            g_free(dyn);
        }
    }
#elif defined(G_OS_UNIX)
    {
        char buf[PATH_MAX];
        ssize_t len = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
        if (len > 0)
        {
            buf[len] = '\0';
            exe_path = g_strdup(buf);
        }
    }
#endif

    if (exe_path == NULL)
        exe_path = g_find_program_in_path(g_get_prgname());

    if (exe_path == NULL)
        return NULL;

    dir = g_path_get_dirname(exe_path);
#ifdef G_OS_WIN32
    candidate = g_build_filename(dir, "rigctld.exe", NULL);
#else
    candidate = g_build_filename(dir, "rigctld", NULL);
#endif

    if (!g_file_test(candidate, G_FILE_TEST_IS_EXECUTABLE))
    {
        g_free(candidate);
        candidate = NULL;
    }

    g_free(dir);
    g_free(exe_path);
    return candidate;
}

static void rigctld_mgr_emit_log(RigctldMgr *mgr, const gchar *prefix,
                                 const gchar *line)
{
    RigctldMgrLogFunc cb = NULL;
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

static void rigctld_mgr_emit_exit_if_ready(RigctldMgr *mgr)
{
    RigctldMgrLogFunc cb = NULL;
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
        cb(mgr, "rigctld", line, data);
        g_free(line);
    }
}

static gpointer rigctld_mgr_wait_thread(gpointer data)
{
    RigctldMgr *mgr = data;
    gint status = -1;
    gint sig = 0;

    if (mgr == NULL || mgr->proc == NULL)
        return NULL;

    g_subprocess_wait(mgr->proc, NULL, NULL);

    if (g_subprocess_get_if_exited(mgr->proc))
        status = g_subprocess_get_exit_status(mgr->proc);

#if GLIB_CHECK_VERSION(2, 40, 0)
    if (g_subprocess_get_if_signaled(mgr->proc))
        sig = g_subprocess_get_term_sig(mgr->proc);
#endif

    g_mutex_lock(&mgr->log_lock);
    mgr->exit_ready = TRUE;
    mgr->exit_status = status;
    mgr->exit_signal = sig;
    g_mutex_unlock(&mgr->log_lock);

    rigctld_mgr_emit_exit_if_ready(mgr);
    return NULL;
}
static void rigctld_mgr_log_append(RigctldMgr *mgr, const gchar *data,
                                   gsize len)
{
    gsize          excess;

    if (mgr == NULL || data == NULL || len == 0)
        return;

    g_mutex_lock(&mgr->log_lock);
    if (mgr->log == NULL)
    {
        g_mutex_unlock(&mgr->log_lock);
        return;
    }

    g_string_append_len(mgr->log, data, len);
    if (mgr->log->len > RIGCTLD_LOG_MAX_LEN)
    {
        excess = mgr->log->len - RIGCTLD_LOG_MAX_LEN;
        g_string_erase(mgr->log, 0, excess);
    }
    g_mutex_unlock(&mgr->log_lock);
}

static gpointer rigctld_mgr_read_stream(gpointer data)
{
    RigctldLogReader *reader = data;
    GError           *error = NULL;
    gchar            *line = NULL;
    gsize             length = 0;

    if (reader == NULL)
        return NULL;

    while ((line = g_data_input_stream_read_line(reader->stream, &length,
                                                 NULL, &error)) != NULL)
    {
        if (length > 0)
        {
            rigctld_mgr_log_append(reader->mgr, line, length);
            rigctld_mgr_log_append(reader->mgr, "\n", 1);
            rigctld_mgr_emit_log(reader->mgr, reader->prefix, line);
        }
        else
        {
            rigctld_mgr_log_append(reader->mgr, "\n", 1);
            rigctld_mgr_emit_log(reader->mgr, reader->prefix, "");
        }
        g_free(line);
    }

    if (error != NULL)
    {
        gchar *msg = g_strdup_printf("rigctld log read failed: %s",
                                     error->message);
        rigctld_mgr_log_append(reader->mgr, msg, strlen(msg));
        rigctld_mgr_log_append(reader->mgr, "\n", 1);
        rigctld_mgr_emit_log(reader->mgr, "rigctld:err", msg);
        g_free(msg);
        g_clear_error(&error);
    }

    g_object_unref(reader->stream);
    g_free(reader);
    return NULL;
}

gchar *rigctld_mgr_normalize_host(const gchar *host)
{
    if (host == NULL || *host == '\0')
        return NULL;

    if (g_ascii_strcasecmp(host, "localhost") == 0 ||
        g_strcmp0(host, "::1") == 0)
        return g_strdup("127.0.0.1");

    return g_strdup(host);
}

gboolean rigctld_mgr_host_is_local(const gchar *host)
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

gboolean rigctld_mgr_port_is_open(const gchar *host, gint port,
                                  gint timeout_ms)
{
    gboolean        ok = FALSE;
    GResolver      *resolver = NULL;
    GList          *addrs = NULL;
    GError         *error = NULL;
    gint64          timeout_us;

    if (host == NULL || *host == '\0' || port <= 0)
        return FALSE;

    resolver = g_resolver_get_default();
    addrs = g_resolver_lookup_by_name(resolver, host, NULL, &error);
    if (addrs == NULL)
    {
        g_clear_error(&error);
        g_object_unref(resolver);
        return FALSE;
    }

    timeout_us = (gint64) timeout_ms * 1000;
    for (GList *iter = addrs; iter != NULL; iter = iter->next)
    {
        GInetAddress   *addr = G_INET_ADDRESS(iter->data);
        GSocket        *sock = NULL;
        GSocketAddress *sockaddr = NULL;

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
            ok = TRUE;
        }
        else if (g_error_matches(error, G_IO_ERROR, G_IO_ERROR_PENDING))
        {
            g_clear_error(&error);
            if (g_socket_condition_timed_wait(sock, G_IO_OUT, timeout_us, NULL,
                                              &error))
            {
                if (g_socket_check_connect_result(sock, &error))
                    ok = TRUE;
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

        g_object_unref(sockaddr);
        g_object_unref(sock);

        if (ok)
            break;
    }

    g_resolver_free_addresses(addrs);
    g_object_unref(resolver);

    return ok;
}

gboolean rigctld_mgr_wait_for_port(const gchar *host, gint port,
                                   gint timeout_ms)
{
    gint    waited_ms = 0;
    gint    interval_ms = 50;

    while (waited_ms < timeout_ms)
    {
        gint slice = interval_ms;

        if (slice > (timeout_ms - waited_ms))
            slice = timeout_ms - waited_ms;

        if (rigctld_mgr_port_is_open(host, port, slice))
            return TRUE;

        waited_ms += slice;
        if (interval_ms < 500)
            interval_ms = MIN(interval_ms * 2, 500);
    }

    return FALSE;
}

RigctldMgr *rigctld_mgr_spawn(const radio_conf_t *conf,
                              const gchar *bind_host,
                              gchar **error_out)
{
    RigctldMgr    *mgr = NULL;
    GSubprocess   *proc = NULL;
    GSubprocessLauncher *launcher = NULL;
    GPtrArray     *argv = NULL;
    GError        *error = NULL;
    gchar         *path = NULL;

    if (conf == NULL)
    {
        if (error_out)
            *error_out = g_strdup("Missing radio configuration.");
        return NULL;
    }

    {
        gint model_id = conf->rigctld_model;

        if (model_id <= 0)
            model_id = radio_model_to_hamlib_model(conf->radio_model);

        if (model_id <= 0)
        {
            if (error_out)
                *error_out = g_strdup("Missing rigctld model number.");
            return NULL;
        }
    }

    if (conf->rigctld_device == NULL || *conf->rigctld_device == '\0')
    {
        if (error_out)
            *error_out = g_strdup("Missing rigctld device/address.");
        return NULL;
    }

    if (conf->port <= 0)
    {
        if (error_out)
            *error_out = g_strdup("Missing rigctld port.");
        return NULL;
    }

    if (conf->rigctld_path && *conf->rigctld_path)
        path = g_strdup(conf->rigctld_path);
    else
    {
        path = rigctld_mgr_find_bundled_rigctld();
        if (path == NULL)
            path = g_find_program_in_path("rigctld");
    }

    if (path == NULL)
    {
        if (error_out)
            *error_out = g_strdup("rigctld not found (bundle or PATH).");
        return NULL;
    }

    argv = g_ptr_array_new_with_free_func(g_free);
    g_ptr_array_add(argv, g_strdup(path));
    if (bind_host && *bind_host)
    {
        g_ptr_array_add(argv, g_strdup("-b"));
        g_ptr_array_add(argv, g_strdup(bind_host));
    }
    {
        gint model_id = conf->rigctld_model;

        if (model_id <= 0)
            model_id = radio_model_to_hamlib_model(conf->radio_model);

        g_ptr_array_add(argv, g_strdup("-m"));
        g_ptr_array_add(argv, g_strdup_printf("%d", model_id));
    }
    g_ptr_array_add(argv, g_strdup("-r"));
    g_ptr_array_add(argv, g_strdup(conf->rigctld_device));
    if (conf->rigctld_conn == RIGCTLD_CONN_SERIAL &&
        conf->rigctld_baud > 0)
    {
        g_ptr_array_add(argv, g_strdup("-s"));
        g_ptr_array_add(argv, g_strdup_printf("%d", conf->rigctld_baud));
    }
    g_ptr_array_add(argv, g_strdup("-t"));
    g_ptr_array_add(argv, g_strdup_printf("%d", conf->port));
    if (conf->rigctld_civaddr && *conf->rigctld_civaddr)
    {
        g_ptr_array_add(argv, g_strdup("-C"));
        g_ptr_array_add(argv,
                        g_strdup_printf("civaddr=%s", conf->rigctld_civaddr));
    }
    if (conf->rigctld_auto_power_on)
    {
        g_ptr_array_add(argv, g_strdup("-C"));
        g_ptr_array_add(argv, g_strdup("auto_power_on=1"));
    }
    g_ptr_array_add(argv, g_strdup("-vvvv"));
    if (conf->rigctld_extra_args && *conf->rigctld_extra_args)
    {
        gchar **extra_argv = NULL;
        gint    extra_argc = 0;

        if (!g_shell_parse_argv(conf->rigctld_extra_args, &extra_argc,
                                &extra_argv, &error))
        {
            if (error_out)
                *error_out = g_strdup(error->message);
            g_clear_error(&error);
            g_strfreev(extra_argv);
            g_ptr_array_free(argv, TRUE);
            g_free(path);
            return NULL;
        }

        for (gint i = 0; i < extra_argc; i++)
            g_ptr_array_add(argv, g_strdup(extra_argv[i]));
        g_strfreev(extra_argv);
    }

    g_ptr_array_add(argv, NULL);

    {
        gchar *cmdline = g_strjoinv(" ", (gchar **) argv->pdata);
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("rigctld spawn argv: %s"),
                    cmdline ? cmdline : "(null)");
        g_free(cmdline);
    }

    launcher = g_subprocess_launcher_new(G_SUBPROCESS_FLAGS_STDIN_DEV_NULL |
                                         G_SUBPROCESS_FLAGS_STDOUT_PIPE |
                                         G_SUBPROCESS_FLAGS_STDERR_PIPE);
    proc = g_subprocess_launcher_spawnv(launcher,
                                        (const gchar * const *) argv->pdata,
                                        &error);
    g_object_unref(launcher);
    if (proc == NULL)
    {
        if (error_out)
            *error_out = g_strdup(error->message);
        g_clear_error(&error);
        g_ptr_array_free(argv, TRUE);
        g_free(path);
        return NULL;
    }

    mgr = g_new0(RigctldMgr, 1);
    mgr->proc = proc;
    g_mutex_init(&mgr->log_lock);
    mgr->log = g_string_new(NULL);
    mgr->exit_ready = FALSE;
    mgr->exit_reported = FALSE;
    mgr->exit_status = -1;
    mgr->exit_signal = 0;

    if (g_subprocess_get_stdout_pipe(proc))
    {
        RigctldLogReader *reader = g_new0(RigctldLogReader, 1);

        reader->mgr = mgr;
        reader->prefix = "rigctld:out";
        reader->stream = g_data_input_stream_new(
            g_subprocess_get_stdout_pipe(proc));
        g_data_input_stream_set_newline_type(reader->stream,
                                             G_DATA_STREAM_NEWLINE_TYPE_ANY);
        mgr->stdout_thread =
            g_thread_new("rigctld-log-out", rigctld_mgr_read_stream, reader);
    }

    if (g_subprocess_get_stderr_pipe(proc))
    {
        RigctldLogReader *reader = g_new0(RigctldLogReader, 1);

        reader->mgr = mgr;
        reader->prefix = "rigctld:err";
        reader->stream = g_data_input_stream_new(
            g_subprocess_get_stderr_pipe(proc));
        g_data_input_stream_set_newline_type(reader->stream,
                                             G_DATA_STREAM_NEWLINE_TYPE_ANY);
        mgr->stderr_thread =
            g_thread_new("rigctld-log-err", rigctld_mgr_read_stream, reader);
    }

    mgr->exit_thread =
        g_thread_new("rigctld-exit", rigctld_mgr_wait_thread, mgr);

    g_ptr_array_free(argv, TRUE);
    g_free(path);
    return mgr;
}

gboolean rigctld_mgr_is_running(const RigctldMgr *mgr)
{
    if (mgr == NULL || mgr->proc == NULL)
        return FALSE;

    return !g_subprocess_get_if_exited(mgr->proc);
}

const gchar *rigctld_mgr_get_identifier(const RigctldMgr *mgr)
{
    if (mgr == NULL || mgr->proc == NULL)
        return NULL;

    return g_subprocess_get_identifier(mgr->proc);
}

gchar *rigctld_mgr_get_log_tail(RigctldMgr *mgr)
{
    gchar *copy = NULL;

    if (mgr == NULL)
        return NULL;

    g_mutex_lock(&mgr->log_lock);
    if (mgr->log && mgr->log->len > 0)
        copy = g_strdup(mgr->log->str);
    g_mutex_unlock(&mgr->log_lock);

    return copy;
}

void rigctld_mgr_set_log_callback(RigctldMgr *mgr,
                                  RigctldMgrLogFunc cb,
                                  gpointer user_data)
{
    if (mgr == NULL)
        return;

    g_mutex_lock(&mgr->log_lock);
    mgr->log_cb = cb;
    mgr->log_cb_data = user_data;
    g_mutex_unlock(&mgr->log_lock);

    rigctld_mgr_emit_exit_if_ready(mgr);
}

static void rigctld_mgr_wait_exit(GSubprocess *proc, gint timeout_ms)
{
    gint waited_ms = 0;

    while (waited_ms < timeout_ms)
    {
        if (g_subprocess_get_if_exited(proc))
            return;

        g_usleep(100 * 1000);
        waited_ms += 100;
    }
}

void rigctld_mgr_terminate(RigctldMgr **mgr_ptr)
{
    RigctldMgr *mgr;

    if (mgr_ptr == NULL || *mgr_ptr == NULL)
        return;

    mgr = *mgr_ptr;

    if (mgr->proc != NULL)
    {
        if (!g_subprocess_get_if_exited(mgr->proc))
        {
#if defined(G_OS_UNIX) && GLIB_CHECK_VERSION(2, 40, 0)
            g_subprocess_send_signal(mgr->proc, SIGTERM);
            rigctld_mgr_wait_exit(mgr->proc, 1500);
#endif
            if (!g_subprocess_get_if_exited(mgr->proc))
                g_subprocess_force_exit(mgr->proc);
        }

        g_subprocess_wait(mgr->proc, NULL, NULL);
    }

    if (mgr->stdout_thread)
    {
        g_thread_join(mgr->stdout_thread);
        mgr->stdout_thread = NULL;
    }

    if (mgr->stderr_thread)
    {
        g_thread_join(mgr->stderr_thread);
        mgr->stderr_thread = NULL;
    }

    if (mgr->exit_thread)
    {
        g_thread_join(mgr->exit_thread);
        mgr->exit_thread = NULL;
    }

    g_clear_object(&mgr->proc);
    g_mutex_clear(&mgr->log_lock);
    if (mgr->log)
        g_string_free(mgr->log, TRUE);
    g_free(mgr);
    *mgr_ptr = NULL;
}
