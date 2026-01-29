#include "rigctld-io.h"

#include <errno.h>
#include <string.h>
#include <sys/types.h>

#include "net_compat.h"

#define RIGCTLD_RXBUF_CHUNK 512
#define RIGCTLD_DUMP_STATE_MIN_LINES 2
#define RIGCTLD_LINE_BUF 512

static GMutex rigctld_io_lock;
static gsize rigctld_io_lock_ready = 0;

static void rigctld_io_lock_init(void)
{
    if (g_once_init_enter(&rigctld_io_lock_ready))
    {
        g_mutex_init(&rigctld_io_lock);
        g_once_init_leave(&rigctld_io_lock_ready, 1);
    }
}

void rigctld_io_lock_acquire(void)
{
    rigctld_io_lock_init();
    g_mutex_lock(&rigctld_io_lock);
}

void rigctld_io_lock_release(void)
{
    g_mutex_unlock(&rigctld_io_lock);
}

static gint rigctld_count_newlines(const gchar *data, gsize len)
{
    gint count = 0;

    if (data == NULL || len == 0)
        return 0;

    for (gsize i = 0; i < len; i++)
    {
        if (data[i] == '\n')
            count++;
    }

    return count;
}

static gboolean rigctld_rxbuf_find_line(const GString *buf, gsize *line_len)
{
    const gchar *pos = NULL;

    if (buf == NULL || buf->len == 0)
        return FALSE;

    pos = memchr(buf->str, '\n', buf->len);
    if (pos == NULL)
        return FALSE;

    if (line_len)
        *line_len = (gsize) (pos - buf->str) + 1;

    return TRUE;
}

static gssize rigctld_rxbuf_take_line(GString *buf, gchar *out, gsize out_len)
{
    gsize line_len = 0;
    gsize copy_len = 0;

    if (!rigctld_rxbuf_find_line(buf, &line_len))
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

    return (gssize) copy_len;
}

static gint rigctld_poll_readable(int fd, gint timeout_ms, gint *err_out)
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

    if (pfd.revents & (G_IO_ERR | G_IO_NVAL))
    {
        if (err_out)
            *err_out = EIO;
        return -1;
    }

    return 1;
}

static gboolean rigctld_line_is_rprt(const gchar *line)
{
    const gchar *scan = line;

    if (scan == NULL)
        return FALSE;

    while (*scan != '\0' && g_ascii_isspace(*scan))
        scan++;

    return g_str_has_prefix(scan, "RPRT ");
}

static gsize rigctld_append_out(gchar *out,
                                gsize out_len,
                                gsize used,
                                const gchar *data,
                                gsize data_len)
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

GString *rigctld_rxbuf_new(gsize initial_size)
{
    if (initial_size == 0)
        initial_size = 128;

    return g_string_sized_new(initial_size);
}

void rigctld_rxbuf_free(GString **buf)
{
    if (buf == NULL || *buf == NULL)
        return;

    g_string_free(*buf, TRUE);
    *buf = NULL;
}

void rigctld_rxbuf_clear(GString *buf)
{
    if (buf == NULL)
        return;

    g_string_set_size(buf, 0);
}

gssize rigctld_read_reply_line(int fd,
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
        ((gint64) timeout_ms * 1000);

    for (;;)
    {
        gssize copied = rigctld_rxbuf_take_line(buf, out, out_len);
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
            gchar chunk[RIGCTLD_RXBUF_CHUNK];
            gssize size;

            if (remaining_us <= 0)
            {
                if (err_out)
                    *err_out = EAGAIN;
                return -1;
            }

            remaining_ms = (gint) (remaining_us / 1000);
            if (remaining_ms <= 0)
                remaining_ms = 1;

            poll_rc = rigctld_poll_readable(fd, remaining_ms, err_out);
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

            g_string_append_len(buf, chunk, (gsize) size);
        }
    }
}

gssize rigctld_read_response(int fd,
                             GString *buf,
                             rigctld_read_mode_t mode,
                             gchar *out,
                             gsize out_len,
                             gint base_timeout_ms,
                             gint idle_timeout_ms,
                             gboolean *saw_rprt,
                             gboolean *used_multiline,
                             gint *err_out)
{
    gchar line[RIGCTLD_LINE_BUF];
    gssize size;
    gsize used = 0;
    gint err = 0;
    gboolean saw_term = FALSE;
    gboolean multi = FALSE;

    if (saw_rprt)
        *saw_rprt = FALSE;
    if (used_multiline)
        *used_multiline = FALSE;
    if (err_out)
        *err_out = 0;

    if (mode == RIGCTLD_READ_MULTILINE_IDLE)
        return rigctld_read_dump_state(fd, buf, out, out_len,
                                       base_timeout_ms, idle_timeout_ms,
                                       err_out);

    size = rigctld_read_reply_line(fd, buf, line, sizeof(line),
                                   base_timeout_ms, &err);
    if (size <= 0)
    {
        if (err_out)
            *err_out = err;
        return size;
    }

    used = rigctld_append_out(out, out_len, used, line, (gsize) size);
    if (rigctld_line_is_rprt(line))
        saw_term = TRUE;

    if (mode == RIGCTLD_READ_MULTILINE_RPRT)
    {
        multi = TRUE;
    }
    else if (buf != NULL && buf->len > 0)
    {
        multi = TRUE;
    }
    else if (!saw_term && idle_timeout_ms > 0)
    {
        gint poll_rc = rigctld_poll_readable(fd, idle_timeout_ms, &err);

        if (poll_rc < 0)
        {
            if (err_out)
                *err_out = err;
            return -1;
        }

        if (poll_rc > 0)
            multi = TRUE;
    }

    if (!multi)
    {
        if (saw_rprt)
            *saw_rprt = saw_term;
        return (gssize) used;
    }

    if (used_multiline)
        *used_multiline = TRUE;

    {
        gint follow_timeout_ms =
            (mode == RIGCTLD_READ_MULTILINE_RPRT) ?
            base_timeout_ms : idle_timeout_ms;

        while (!saw_term)
        {
            size = rigctld_read_reply_line(fd, buf, line, sizeof(line),
                                           follow_timeout_ms, &err);
            if (size <= 0)
            {
                if (size < 0 && err != EAGAIN && err != EWOULDBLOCK)
                {
                    if (err_out)
                        *err_out = err;
                    return -1;
                }
                break;
            }

            used = rigctld_append_out(out, out_len, used, line, (gsize) size);
            if (rigctld_line_is_rprt(line))
                saw_term = TRUE;
        }
    }

    if (saw_rprt)
        *saw_rprt = saw_term;

    return (gssize) used;
}

gssize rigctld_read_dump_state(int fd,
                               GString *buf,
                               gchar *out,
                               gsize out_len,
                               gint base_timeout_ms,
                               gint idle_timeout_ms,
                               gint *err_out)
{
    gint64 deadline_us;
    gboolean got_data = FALSE;
    gint lines = 0;

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
        ((gint64) base_timeout_ms * 1000);

    if (buf->len > 0)
    {
        got_data = TRUE;
        lines = rigctld_count_newlines(buf->str, buf->len);
    }

    for (;;)
    {
        gint timeout_ms;
        gint poll_rc;
        gchar chunk[RIGCTLD_RXBUF_CHUNK];
        gssize size;

        if (got_data && lines >= RIGCTLD_DUMP_STATE_MIN_LINES)
        {
            timeout_ms = idle_timeout_ms;
        }
        else
        {
            gint64 now_us = g_get_monotonic_time();
            gint64 remaining_us = deadline_us - now_us;

            if (remaining_us <= 0)
            {
                if (err_out)
                    *err_out = EAGAIN;
                return -1;
            }

            timeout_ms = (gint) (remaining_us / 1000);
            if (timeout_ms <= 0)
                timeout_ms = 1;
        }

        poll_rc = rigctld_poll_readable(fd, timeout_ms, err_out);
        if (poll_rc == 0)
        {
            if (got_data && lines >= RIGCTLD_DUMP_STATE_MIN_LINES)
                break;
            if (err_out)
                *err_out = EAGAIN;
            return -1;
        }
        if (poll_rc < 0)
            return -1;

        size = recv(fd, chunk, sizeof(chunk), 0);
        if (size == 0)
        {
            if (got_data)
                break;
            return 0;
        }
        if (size < 0)
        {
            if (err_out)
                *err_out = errno;
            return -1;
        }

        g_string_append_len(buf, chunk, (gsize) size);
        got_data = TRUE;
        lines += rigctld_count_newlines(chunk, (gsize) size);
    }

    {
        gsize copy_len = buf->len;

        if (copy_len >= out_len)
            copy_len = out_len - 1;
        if (copy_len > 0)
            memcpy(out, buf->str, copy_len);
        out[copy_len] = '\0';
        rigctld_rxbuf_clear(buf);
        return (gssize) copy_len;
    }
}

gssize rigctld_drain_idle(int fd,
                          GString *buf,
                          gint idle_timeout_ms,
                          gint *err_out)
{
    gssize total = 0;
    gint err = 0;

    if (err_out)
        *err_out = 0;

    if (idle_timeout_ms <= 0)
        idle_timeout_ms = 50;

    if (buf != NULL)
        rigctld_rxbuf_clear(buf);

    for (;;)
    {
        gint poll_rc;
        gchar chunk[RIGCTLD_RXBUF_CHUNK];
        gssize size;

        poll_rc = rigctld_poll_readable(fd, idle_timeout_ms, &err);
        if (poll_rc == 0)
            break;
        if (poll_rc < 0)
        {
            if (err_out)
                *err_out = err;
            return -1;
        }

        size = recv(fd, chunk, sizeof(chunk), 0);
        if (size <= 0)
            break;

        total += size;
    }

    return total;
}
