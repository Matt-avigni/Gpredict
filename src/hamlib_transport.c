#include "hamlib_transport.h"

#include <errno.h>
#include <string.h>
#include <sys/types.h>

#include <gio/gio.h>

#ifndef WIN32
#include <sys/socket.h>
#else
#include <winsock2.h>
#endif

#define HAMLIB_RXBUF_CHUNK 512
#define HAMLIB_LINE_BUF 512

struct _HamlibTransport {
    GSocket *socket;
    gint     fd;
    GString *rxbuf;
    GMutex   io_lock;
    GCond    io_cond;
    gboolean busy;
    gint     last_err;
    gint64   last_rtt_us;
};

static gboolean hamlib_rxbuf_find_line(const GString *buf, gsize *line_len)
{
    const gchar *pos = NULL;

    if (buf == NULL || buf->len == 0)
        return FALSE;

    pos = memchr(buf->str, '\n', buf->len);
    if (pos == NULL)
        return FALSE;

    if (line_len)
        *line_len = (gsize)(pos - buf->str) + 1;

    return TRUE;
}

static gssize hamlib_rxbuf_take_line(GString *buf, gchar *out, gsize out_len)
{
    gsize line_len = 0;
    gsize copy_len = 0;

    if (!hamlib_rxbuf_find_line(buf, &line_len))
        return 0;

    if (out_len == 0)
        return -1;

    copy_len = line_len;
    if (copy_len >= out_len)
        copy_len = out_len - 1;

    if (copy_len > 0)
        memcpy(out, buf->str, copy_len);
    out[copy_len] = '\0';

    g_string_erase(buf, 0, line_len);

    return (gssize)copy_len;
}

static gint hamlib_poll_readable(gint fd, gint timeout_ms, gint *err_out)
{
    GPollFD pfd;
    gint rc;

    pfd.fd = fd;
    pfd.events = G_IO_IN;
    pfd.revents = 0;

    rc = g_poll(&pfd, 1, timeout_ms);
    if (rc == 0)
        return 0;

    if (rc < 0)
    {
        if (err_out)
            *err_out = errno;
        return -1;
    }

    if (pfd.revents & (G_IO_ERR | G_IO_HUP | G_IO_NVAL))
    {
        if (err_out)
            *err_out = EIO;
        return -1;
    }

    return 1;
}

static gboolean hamlib_line_is_done(const gchar *line)
{
    const gchar *scan = line;

    if (scan == NULL)
        return FALSE;

    while (*scan != '\0' && g_ascii_isspace(*scan))
        scan++;

    if (!g_ascii_strcasecmp(scan, "done\n") ||
        !g_ascii_strcasecmp(scan, "done\r\n") ||
        !g_ascii_strcasecmp(scan, "done"))
        return TRUE;

    return FALSE;
}

static gboolean hamlib_line_parse_rprt(const gchar *line, gint *code_out)
{
    const gchar *scan = line;
    gchar *endptr = NULL;
    glong code = 0;

    if (scan == NULL)
        return FALSE;

    while (*scan != '\0' && g_ascii_isspace(*scan))
        scan++;

    if (!g_str_has_prefix(scan, "RPRT"))
        return FALSE;

    scan += 4;
    while (*scan != '\0' && g_ascii_isspace(*scan))
        scan++;

    if (*scan == '\0')
    {
        if (code_out)
            *code_out = 0;
        return TRUE;
    }

    code = g_ascii_strtoll(scan, &endptr, 10);
    if (endptr == scan)
        code = 0;

    if (code_out)
        *code_out = (gint)code;

    return TRUE;
}

static gboolean hamlib_line_is_prompt(const gchar *line)
{
    const gchar *scan = line;

    if (scan == NULL)
        return FALSE;

    while (*scan != '\0' && g_ascii_isspace(*scan))
        scan++;

    if (g_str_has_prefix(scan, "rigctld>") ||
        g_str_has_prefix(scan, "rotctld>"))
        return TRUE;

    return FALSE;
}

static gboolean hamlib_line_is_blank(const gchar *line)
{
    const gchar *scan = line;

    if (scan == NULL)
        return TRUE;

    while (*scan != '\0')
    {
        if (!g_ascii_isspace(*scan))
            return FALSE;
        scan++;
    }

    return TRUE;
}

static gsize hamlib_append_out(gchar *out, gsize out_len, gsize used,
                               const gchar *data, gsize data_len)
{
    gsize copy_len = 0;

    if (out == NULL || out_len == 0)
        return used;

    if (used >= out_len - 1)
    {
        out[out_len - 1] = '\0';
        return used;
    }

    copy_len = data_len;
    if (copy_len > (out_len - 1 - used))
        copy_len = out_len - 1 - used;

    if (copy_len > 0)
        memcpy(out + used, data, copy_len);

    used += copy_len;
    out[used] = '\0';
    return used;
}

static gsize hamlib_append_out_line(gchar *out, gsize out_len, gsize used,
                                    const gchar *line, gsize line_len)
{
    gsize trimmed = line_len;

    while (trimmed > 0 &&
           (line[trimmed - 1] == '\n' || line[trimmed - 1] == '\r'))
        trimmed--;

    if (trimmed > 0)
        used = hamlib_append_out(out, out_len, used, line, trimmed);

    if (used < out_len - 1)
    {
        out[used++] = '\n';
        out[used] = '\0';
    }

    return used;
}

static gssize hamlib_read_reply_line(gint fd,
                                     GString *buf,
                                     gchar *out,
                                     gsize out_len,
                                     gint timeout_ms,
                                     gint *err_out)
{
    gint64 deadline_us;

    if (err_out)
        *err_out = 0;

    if (buf == NULL || out == NULL || out_len == 0)
    {
        if (err_out)
            *err_out = EINVAL;
        return -1;
    }

    if (timeout_ms < 0)
        timeout_ms = 0;

    deadline_us = g_get_monotonic_time() +
        ((gint64)timeout_ms * 1000);

    for (;;)
    {
        gssize copied = hamlib_rxbuf_take_line(buf, out, out_len);
        if (copied > 0)
            return copied;
        if (copied < 0)
        {
            if (err_out)
                *err_out = EINVAL;
            return -1;
        }

        {
            gint64 now_us = g_get_monotonic_time();
            gint64 remaining_us = deadline_us - now_us;
            gint remaining_ms;
            gint poll_rc;
            gchar chunk[HAMLIB_RXBUF_CHUNK];
            gssize size;

            if (remaining_us <= 0)
            {
                if (err_out)
                    *err_out = EAGAIN;
                return -1;
            }

            remaining_ms = (gint)(remaining_us / 1000);
            if (remaining_ms <= 0)
                remaining_ms = 1;

            poll_rc = hamlib_poll_readable(fd, remaining_ms, err_out);
            if (poll_rc <= 0)
            {
                if (poll_rc == 0 && err_out)
                    *err_out = EAGAIN;
                return -1;
            }

            size = recv(fd, chunk, sizeof(chunk), 0);
            if (size == 0)
                return 0;
            if (size < 0)
            {
                if (err_out)
                    *err_out = errno;
                return -1;
            }

            g_string_append_len(buf, chunk, (gsize)size);
        }
    }
}

static gssize hamlib_read_dump_state(gint fd,
                                     GString *buf,
                                     gchar *out,
                                     gsize out_len,
                                     gint base_timeout_ms,
                                     gint idle_timeout_ms,
                                     gint *err_out)
{
    gint64 deadline_us;
    gint err = 0;
    gboolean saw_rprt = FALSE;
    gboolean saw_done = FALSE;
    gboolean done = FALSE;
    gsize used = 0;
    gchar line[HAMLIB_LINE_BUF];

    if (err_out)
        *err_out = 0;

    if (buf == NULL || out == NULL || out_len == 0)
    {
        if (err_out)
            *err_out = EINVAL;
        return -1;
    }

    if (base_timeout_ms <= 0)
        base_timeout_ms = 1000;
    if (idle_timeout_ms <= 0)
        idle_timeout_ms = 50;

    deadline_us = g_get_monotonic_time() +
        ((gint64)base_timeout_ms * 1000);

    while (!done)
    {
        gint timeout_ms;
        gint64 now_us;
        gint64 remaining_us;
        gssize size;

        if (!saw_rprt)
        {
            now_us = g_get_monotonic_time();
            remaining_us = deadline_us - now_us;
            if (remaining_us <= 0)
            {
                if (err_out)
                    *err_out = EAGAIN;
                return -1;
            }
            timeout_ms = (gint)(remaining_us / 1000);
            if (timeout_ms <= 0)
                timeout_ms = 1;
        }
        else
        {
            timeout_ms = idle_timeout_ms;
        }

        size = hamlib_read_reply_line(fd, buf, line, sizeof(line),
                                      timeout_ms, &err);
        if (size <= 0)
        {
            if (saw_rprt)
                break;
            if (err_out)
                *err_out = err;
            return -1;
        }

        if (hamlib_line_is_prompt(line))
            continue;

        if (hamlib_line_is_done(line))
        {
            saw_done = TRUE;
            used = hamlib_append_out_line(out, out_len, used,
                                          line, (gsize)size);
            done = TRUE;
            break;
        }

        if (hamlib_line_parse_rprt(line, NULL))
        {
            saw_rprt = TRUE;
            continue;
        }

        if (saw_rprt)
        {
            if (hamlib_line_is_blank(line))
            {
                done = TRUE;
                break;
            }
        }

        used = hamlib_append_out_line(out, out_len, used,
                                      line, (gsize)size);
    }

    if (!saw_rprt && !saw_done)
    {
        if (err_out)
            *err_out = EAGAIN;
        return -1;
    }

    if (buf)
        g_string_set_size(buf, 0);

    return (gssize)used;
}

static gssize hamlib_read_response(gint fd,
                                   GString *buf,
                                   hamlib_read_mode_t mode,
                                   hamlib_term_t term,
                                   gchar *out,
                                   gsize out_len,
                                   gint base_timeout_ms,
                                   gint idle_timeout_ms,
                                   HamlibResponseInfo *info)
{
    gchar line[HAMLIB_LINE_BUF];
    gssize size;
    gsize used = 0;
    gint err = 0;
    gboolean saw_term = FALSE;
    gboolean multi = FALSE;
    gboolean saw_rprt = FALSE;
    gboolean saw_done = FALSE;

    if (info)
    {
        info->saw_rprt = FALSE;
        info->rprt_code = 0;
        info->saw_done = FALSE;
        info->used_multiline = FALSE;
        info->err = 0;
    }

    if (mode == HAMLIB_READ_MULTILINE_IDLE)
    {
        gssize dump_size = hamlib_read_dump_state(fd, buf, out, out_len,
                                                  base_timeout_ms,
                                                  idle_timeout_ms, &err);
        if (info)
            info->err = err;
        return dump_size;
    }

    for (;;)
    {
        size = hamlib_read_reply_line(fd, buf, line, sizeof(line),
                                      base_timeout_ms, &err);
        if (size <= 0)
        {
            if (info)
                info->err = err;
            return size;
        }

        if (hamlib_line_is_prompt(line))
            continue;

        if (hamlib_line_parse_rprt(line, info ? &info->rprt_code : NULL))
        {
            saw_rprt = TRUE;
            saw_term = TRUE;
            break;
        }

        if (term == HAMLIB_TERM_RPRT_OR_DONE && hamlib_line_is_done(line))
        {
            saw_done = TRUE;
            saw_term = TRUE;
            break;
        }

        used = hamlib_append_out_line(out, out_len, used, line, (gsize)size);
        break;
    }

    if (mode == HAMLIB_READ_MULTILINE_RPRT)
    {
        multi = TRUE;
    }
    else if (buf != NULL && buf->len > 0)
    {
        multi = TRUE;
    }
    else if (!saw_term && idle_timeout_ms > 0)
    {
        gint poll_rc = hamlib_poll_readable(fd, idle_timeout_ms, &err);
        if (poll_rc < 0)
        {
            if (info)
                info->err = err;
            return -1;
        }
        if (poll_rc > 0)
            multi = TRUE;
    }

    if (!multi)
    {
        if (info)
        {
            info->saw_rprt = saw_rprt;
            info->saw_done = saw_done;
        }
        return (gssize)used;
    }

    if (info)
        info->used_multiline = TRUE;

    {
        gint follow_timeout_ms =
            (mode == HAMLIB_READ_MULTILINE_RPRT) ? base_timeout_ms : idle_timeout_ms;

        while (!saw_term)
        {
            size = hamlib_read_reply_line(fd, buf, line, sizeof(line),
                                          follow_timeout_ms, &err);
            if (size <= 0)
            {
                if (size < 0 && err != EAGAIN && err != EWOULDBLOCK)
                {
                    if (info)
                        info->err = err;
                    return -1;
                }
                break;
            }

            if (hamlib_line_is_prompt(line))
                continue;

            if (hamlib_line_parse_rprt(line, info ? &info->rprt_code : NULL))
            {
                saw_rprt = TRUE;
                saw_term = TRUE;
                break;
            }

            if (term == HAMLIB_TERM_RPRT_OR_DONE && hamlib_line_is_done(line))
            {
                saw_done = TRUE;
                saw_term = TRUE;
                break;
            }

            used = hamlib_append_out_line(out, out_len, used,
                                          line, (gsize)size);
        }
    }

    if (info)
    {
        info->saw_rprt = saw_rprt;
        info->saw_done = saw_done;
    }

    return (gssize)used;
}

static gboolean hamlib_send_all(gint fd, const gchar *data, gsize len)
{
    gsize sent = 0;

    while (sent < len)
    {
        gssize rc = send(fd, data + sent, len - sent, 0);
        if (rc <= 0)
            return FALSE;
        sent += (gsize)rc;
    }

    return TRUE;
}

HamlibTransport *hamlib_transport_new(void)
{
    HamlibTransport *transport = g_new0(HamlibTransport, 1);

    transport->fd = -1;
    g_mutex_init(&transport->io_lock);
    g_cond_init(&transport->io_cond);
    return transport;
}

void hamlib_transport_free(HamlibTransport **transport)
{
    if (transport == NULL || *transport == NULL)
        return;

    hamlib_transport_close(*transport);
    if ((*transport)->rxbuf)
        g_string_free((*transport)->rxbuf, TRUE);
    g_mutex_clear(&(*transport)->io_lock);
    g_cond_clear(&(*transport)->io_cond);
    g_free(*transport);
    *transport = NULL;
}

gboolean hamlib_transport_connect(HamlibTransport *transport,
                                  const gchar *host,
                                  gint port,
                                  gint timeout_ms,
                                  gchar **error_out)
{
    GSocketClient *client = NULL;
    GSocketConnection *conn = NULL;
    GError *error = NULL;
    guint timeout_sec = 0;

    if (error_out)
        *error_out = NULL;

    if (transport == NULL || host == NULL || *host == '\0' || port <= 0)
    {
        if (error_out)
            *error_out = g_strdup("invalid host/port");
        return FALSE;
    }

    hamlib_transport_close(transport);

    client = g_socket_client_new();
    if (timeout_ms > 0)
    {
        timeout_sec = (guint)((timeout_ms + 999) / 1000);
        if (timeout_sec == 0)
            timeout_sec = 1;
        g_socket_client_set_timeout(client, timeout_sec);
    }

    conn = g_socket_client_connect_to_host(client, host,
                                           (guint16)port,
                                           NULL, &error);
    g_object_unref(client);

    if (conn == NULL)
    {
        if (error_out)
            *error_out = g_strdup(error ? error->message : "connect failed");
        g_clear_error(&error);
        return FALSE;
    }

    transport->socket = g_socket_connection_get_socket(conn);
    g_object_ref(transport->socket);
    transport->fd = g_socket_get_fd(transport->socket);
    g_object_unref(conn);

    if (transport->rxbuf == NULL)
        transport->rxbuf = g_string_sized_new(128);
    else
        g_string_set_size(transport->rxbuf, 0);

    transport->last_err = 0;
    transport->last_rtt_us = 0;
    return TRUE;
}

gboolean hamlib_transport_attach_fd(HamlibTransport *transport,
                                    gint fd,
                                    gchar **error_out)
{
    GSocket *socket = NULL;
    GError *error = NULL;

    if (error_out)
        *error_out = NULL;

    if (transport == NULL || fd < 0)
    {
        if (error_out)
            *error_out = g_strdup("invalid socket");
        return FALSE;
    }

    hamlib_transport_close(transport);

    socket = g_socket_new_from_fd(fd, &error);
    if (socket == NULL)
    {
        if (error_out)
            *error_out = g_strdup(error ? error->message : "attach failed");
        g_clear_error(&error);
        return FALSE;
    }

    transport->socket = socket;
    transport->fd = g_socket_get_fd(socket);

    if (transport->rxbuf == NULL)
        transport->rxbuf = g_string_sized_new(128);
    else
        g_string_set_size(transport->rxbuf, 0);

    transport->last_err = 0;
    transport->last_rtt_us = 0;
    return TRUE;
}

void hamlib_transport_close(HamlibTransport *transport)
{
    if (transport == NULL)
        return;

    if (transport->socket)
    {
        g_socket_close(transport->socket, NULL);
        g_clear_object(&transport->socket);
    }
    transport->fd = -1;
    if (transport->rxbuf)
        g_string_set_size(transport->rxbuf, 0);
}

gboolean hamlib_transport_is_ready(const HamlibTransport *transport)
{
    return (transport != NULL && transport->socket != NULL && transport->fd >= 0);
}

gboolean hamlib_transport_request(HamlibTransport *transport,
                                  const gchar *cmd,
                                  hamlib_read_mode_t mode,
                                  hamlib_term_t term,
                                  gint base_timeout_ms,
                                  gint idle_timeout_ms,
                                  gint retries,
                                  gint retry_delay_ms,
                                  gchar *out,
                                  gsize out_len,
                                  HamlibResponseInfo *info)
{
    gint attempt = 0;
    gboolean ok = FALSE;

    if (info)
        memset(info, 0, sizeof(*info));

    if (transport == NULL || cmd == NULL || !hamlib_transport_is_ready(transport))
        return FALSE;

    g_mutex_lock(&transport->io_lock);
    while (transport->busy)
        g_cond_wait(&transport->io_cond, &transport->io_lock);
    transport->busy = TRUE;
    g_mutex_unlock(&transport->io_lock);

    if (out && out_len > 0)
        out[0] = '\0';

    for (attempt = 0; attempt <= retries; attempt++)
    {
        gint err = 0;
        HamlibResponseInfo local = { 0 };
        gint64 start_us;

        if (transport->rxbuf)
            g_string_set_size(transport->rxbuf, 0);

        start_us = g_get_monotonic_time();
        if (!hamlib_send_all(transport->fd, cmd, strlen(cmd)))
        {
            transport->last_err = errno;
            local.err = errno;
        }
        else
        {
            gssize size = hamlib_read_response(transport->fd,
                                               transport->rxbuf,
                                               mode, term,
                                               out, out_len,
                                               base_timeout_ms,
                                               idle_timeout_ms,
                                               &local);
            if (size >= 0)
            {
                ok = TRUE;
                transport->last_err = 0;
            }
        }

        local.rtt_us = g_get_monotonic_time() - start_us;
        transport->last_rtt_us = local.rtt_us;

        if (info)
            *info = local;

        if (ok)
            break;

        err = local.err;
        if (attempt < retries &&
            (err == EAGAIN || err == EWOULDBLOCK || err == ETIMEDOUT))
        {
            if (retry_delay_ms > 0)
                g_usleep((gulong)retry_delay_ms * 1000);
            continue;
        }
        break;
    }

    g_mutex_lock(&transport->io_lock);
    transport->busy = FALSE;
    g_cond_signal(&transport->io_cond);
    g_mutex_unlock(&transport->io_lock);

    return ok;
}

gssize hamlib_transport_drain(HamlibTransport *transport,
                              gint idle_timeout_ms,
                              gint *err_out)
{
    gssize total = 0;
    gint err = 0;

    if (err_out)
        *err_out = 0;

    if (transport == NULL || !hamlib_transport_is_ready(transport))
        return -1;

    if (idle_timeout_ms <= 0)
        idle_timeout_ms = 50;

    if (transport->rxbuf)
        g_string_set_size(transport->rxbuf, 0);

    for (;;)
    {
        gint poll_rc;
        gchar chunk[HAMLIB_RXBUF_CHUNK];
        gssize size;

        poll_rc = hamlib_poll_readable(transport->fd, idle_timeout_ms, &err);
        if (poll_rc == 0)
            break;
        if (poll_rc < 0)
        {
            if (err_out)
                *err_out = err;
            return -1;
        }

        size = recv(transport->fd, chunk, sizeof(chunk), 0);
        if (size <= 0)
            break;

        total += size;
    }

    return total;
}

gssize hamlib_transport_clear_rxbuf(HamlibTransport *transport)
{
    gssize total = 0;

    if (transport == NULL || !hamlib_transport_is_ready(transport))
        return -1;

    if (transport->rxbuf)
        g_string_set_size(transport->rxbuf, 0);

    for (;;)
    {
        gint err = 0;
        gint poll_rc;
        gchar chunk[HAMLIB_RXBUF_CHUNK];
        gssize size;

        poll_rc = hamlib_poll_readable(transport->fd, 0, &err);
        if (poll_rc <= 0)
            break;

        size = recv(transport->fd, chunk, sizeof(chunk), 0);
        if (size <= 0)
        {
            if (size < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
                break;
            break;
        }

        total += size;
    }

    return total;
}

gint64 hamlib_transport_last_rtt_us(const HamlibTransport *transport)
{
    if (transport == NULL)
        return 0;
    return transport->last_rtt_us;
}
