#include "rigctld_mgr.h"

#include <gio/gio.h>

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
    GMutex       log_lock;
    GString     *log;
};

typedef struct {
    RigctldMgr  *mgr;
    GInputStream *stream;
} RigctldLogReader;

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
    gchar             buffer[256];
    gssize            nread;

    if (reader == NULL)
        return NULL;

    while ((nread = g_input_stream_read(reader->stream, buffer,
                                        sizeof(buffer), NULL, NULL)) > 0)
    {
        rigctld_mgr_log_append(reader->mgr, buffer, (gsize) nread);
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

    if (conf->rigctld_model <= 0)
    {
        if (error_out)
            *error_out = g_strdup("Missing rigctld model number.");
        return NULL;
    }

    if (conf->rigctld_device == NULL || *conf->rigctld_device == '\0')
    {
        if (error_out)
            *error_out = g_strdup("Missing rigctld device path.");
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
        path = g_find_program_in_path("rigctld");

    if (path == NULL)
    {
        if (error_out)
            *error_out = g_strdup("rigctld not found in PATH.");
        return NULL;
    }

    argv = g_ptr_array_new_with_free_func(g_free);
    g_ptr_array_add(argv, g_strdup(path));
    if (bind_host && *bind_host)
    {
        g_ptr_array_add(argv, g_strdup("-b"));
        g_ptr_array_add(argv, g_strdup(bind_host));
    }
    g_ptr_array_add(argv, g_strdup("-m"));
    g_ptr_array_add(argv, g_strdup_printf("%d", conf->rigctld_model));
    g_ptr_array_add(argv, g_strdup("-r"));
    g_ptr_array_add(argv, g_strdup(conf->rigctld_device));
    if (conf->rigctld_baud > 0)
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

    if (g_subprocess_get_stdout_pipe(proc))
    {
        RigctldLogReader *reader = g_new0(RigctldLogReader, 1);

        reader->mgr = mgr;
        reader->stream = g_object_ref(g_subprocess_get_stdout_pipe(proc));
        mgr->stdout_thread =
            g_thread_new("rigctld-log-out", rigctld_mgr_read_stream, reader);
    }

    if (g_subprocess_get_stderr_pipe(proc))
    {
        RigctldLogReader *reader = g_new0(RigctldLogReader, 1);

        reader->mgr = mgr;
        reader->stream = g_object_ref(g_subprocess_get_stderr_pipe(proc));
        mgr->stderr_thread =
            g_thread_new("rigctld-log-err", rigctld_mgr_read_stream, reader);
    }

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

    g_clear_object(&mgr->proc);
    g_mutex_clear(&mgr->log_lock);
    if (mgr->log)
        g_string_free(mgr->log, TRUE);
    g_free(mgr);
    *mgr_ptr = NULL;
}
