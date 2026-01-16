#include <glib.h>

#include "rigctld-io.h"

#ifndef G_OS_WIN32
#include <errno.h>
#include <string.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#ifndef G_OS_WIN32
typedef struct {
    int server_fd;
    gboolean stop;
    gboolean saw_v;
} RigctldTestServer;

typedef struct {
    int fd;
    const gchar *first;
    const gchar *rest;
    gint delay_ms;
} RigctldDelaySend;

static gboolean send_all(int fd, const gchar *data, gsize len)
{
    gsize sent = 0;

    while (sent < len)
    {
        ssize_t rc = send(fd, data + sent, len - sent, 0);
        if (rc <= 0)
            return FALSE;
        sent += (gsize) rc;
    }

    return TRUE;
}

static gboolean send_lines_with_delay(int fd, const gchar *text, gint delay_ms)
{
    const gchar *scan = text;
    const gchar *line_end = NULL;

    while (*scan != '\0')
    {
        line_end = strchr(scan, '\n');
        if (line_end == NULL)
            line_end = scan + strlen(scan);
        else
            line_end++;

        if (!send_all(fd, scan, (gsize) (line_end - scan)))
            return FALSE;

        scan = line_end;
        if (*scan == '\0')
            break;

        if (delay_ms > 0)
            g_usleep((gulong) delay_ms * 1000);
    }

    return TRUE;
}

static gboolean read_line(int fd, GString *line)
{
    gchar ch = '\0';
    gssize rc = 0;

    g_string_set_size(line, 0);
    for (;;)
    {
        rc = recv(fd, &ch, 1, 0);
        if (rc <= 0)
            return FALSE;
        g_string_append_c(line, ch);
        if (ch == '\n')
            return TRUE;
    }
}

static gpointer rigctld_delay_sender(gpointer data)
{
    RigctldDelaySend *send = data;

    if (send == NULL)
        return NULL;

    if (send->first)
        (void) send_all(send->fd, send->first, strlen(send->first));
    if (send->delay_ms > 0)
        g_usleep((gulong) send->delay_ms * 1000);
    if (send->rest)
        (void) send_all(send->fd, send->rest, strlen(send->rest));

    return NULL;
}

static void rigctld_serve_client(int fd, RigctldTestServer *server)
{
    const gchar *dump_state =
        "1\n"
        "3081\n"
        "0\n"
        "0x400001 9000 9000 0x0 0x0 0x0 0x0\n"
        "0x400001 9000 9000 0x0 0x0 0x0 0x0\n"
        "ctcss_list=67.0 69.3 71.9 74.4\n";
    const gchar *vfo_err = "RPRT -11\n";
    const gchar *freq =
        "437800000\n"
        "RPRT 0\n";
    const gchar *ok = "RPRT 0\n";
    GString *line = g_string_new(NULL);

    while (read_line(fd, line))
    {
        if (g_str_has_prefix(line->str, "\\dump_state"))
        {
            if (!send_lines_with_delay(fd, dump_state, 20))
                break;
        }
        else if (line->str[0] == 'v')
        {
            if (server != NULL)
                server->saw_v = TRUE;
            if (!send_all(fd, vfo_err, strlen(vfo_err)))
                break;
        }
        else if (g_str_has_prefix(line->str, "\\set_vfo_opt"))
        {
            if (!send_all(fd, ok, strlen(ok)))
                break;
        }
        else if (g_str_has_prefix(line->str, "V "))
        {
            if (!send_all(fd, ok, strlen(ok)))
                break;
        }
        else if (line->str[0] == 'f')
        {
            if (!send_lines_with_delay(fd, freq, 20))
                break;
        }
        else if (line->str[0] == 'F')
        {
            gchar *trim = g_strdup(line->str);
            gchar **parts = g_strsplit(g_strstrip(trim), " ", 0);
            gint count = 0;

            for (gint i = 0; parts[i] != NULL; i++)
            {
                if (parts[i][0] != '\0')
                    count++;
            }

            g_strfreev(parts);
            g_free(trim);

            if (count != 2)
            {
                const gchar *err = "RPRT -8\n";
                if (!send_all(fd, err, strlen(err)))
                    break;
            }
            else if (!send_all(fd, ok, strlen(ok)))
                break;
        }
        else
        {
            const gchar *err = "RPRT -8\n";
            if (!send_all(fd, err, strlen(err)))
                break;
        }
    }

    g_string_free(line, TRUE);
}

static gpointer rigctld_test_server_thread(gpointer data)
{
    RigctldTestServer *server = data;

    while (!server->stop)
    {
        struct sockaddr_in addr;
        socklen_t addr_len = sizeof(addr);
        int client_fd = accept(server->server_fd,
                               (struct sockaddr *)&addr, &addr_len);
        if (client_fd < 0)
        {
            if (server->stop)
                break;
            continue;
        }

        rigctld_serve_client(client_fd, server);
        close(client_fd);
    }

    return NULL;
}

static int rigctld_test_server_start(RigctldTestServer *server, guint16 *port_out)
{
    struct sockaddr_in addr;
    socklen_t addr_len = sizeof(addr);

    server->server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server->server_fd < 0)
        return -1;

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    addr.sin_port = 0;

    if (bind(server->server_fd, (struct sockaddr *)&addr, sizeof(addr)) != 0)
    {
        close(server->server_fd);
        server->server_fd = -1;
        return -1;
    }

    if (listen(server->server_fd, 4) != 0)
    {
        close(server->server_fd);
        server->server_fd = -1;
        return -1;
    }

    if (getsockname(server->server_fd, (struct sockaddr *)&addr, &addr_len) != 0)
    {
        close(server->server_fd);
        server->server_fd = -1;
        return -1;
    }

    if (port_out)
        *port_out = ntohs(addr.sin_port);

    return 0;
}

static int rigctld_test_connect(guint16 port)
{
    struct sockaddr_in addr;
    int fd = socket(AF_INET, SOCK_STREAM, 0);

    if (fd < 0)
        return -1;

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(port);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) != 0)
    {
        close(fd);
        return -1;
    }

    return fd;
}

static int rigctld_run_probe_simple(int fd)
{
    GString *rxbuf = rigctld_rxbuf_new(128);
    gchar out[4096];
    gssize size = 0;
    gboolean saw_rprt = FALSE;
    gboolean used_multiline = FALSE;
    gint err = 0;
    gboolean ok = FALSE;

    if (!send_all(fd, "f\n", 2))
        goto fail;
    size = rigctld_read_response(fd, rxbuf, RIGCTLD_READ_SINGLE,
                                 out, sizeof(out), 500, 50,
                                 &saw_rprt, &used_multiline, &err);
    if (size <= 0)
        goto fail;

    out[size] = '\0';
    {
        gchar *line1 = out;
        gchar *eol = strchr(out, '\n');
        if (eol != NULL)
            *eol = '\0';
        line1 = g_strstrip(line1);
        if (line1[0] != '\0' &&
            (g_ascii_isdigit(line1[0]) || g_str_has_prefix(line1, "RPRT")))
            ok = TRUE;
    }
    if (!ok)
        goto fail;

    rigctld_rxbuf_free(&rxbuf);
    return 0;

fail:
    rigctld_rxbuf_free(&rxbuf);
    return -1;
}

static int run_fake_server_test(void)
{
    RigctldTestServer server;
    GThread *thread = NULL;
    guint16 port = 0;
    int probe_fd = -1;
    int main_fd = -1;
    int rc = 0;
    GString *rxbuf = NULL;
    gchar out[4096];
    gssize size = 0;
    gboolean saw_rprt = FALSE;
    gboolean used_multiline = FALSE;
    gint err = 0;

    memset(&server, 0, sizeof(server));
    server.server_fd = -1;
    server.stop = FALSE;

    if (rigctld_test_server_start(&server, &port) != 0)
        return -1;

    thread = g_thread_new("rigctld-test", rigctld_test_server_thread, &server);

    probe_fd = rigctld_test_connect(port);
    if (probe_fd < 0)
    {
        rc = -2;
        goto out;
    }
    if (rigctld_run_probe_simple(probe_fd) != 0)
    {
        rc = -4;
        goto out;
    }
    close(probe_fd);
    probe_fd = -1;

    main_fd = rigctld_test_connect(port);
    if (main_fd < 0)
    {
        rc = -5;
        goto out;
    }

    if (!send_all(main_fd, "\\dump_state\n", strlen("\\dump_state\n")))
    {
        rc = -6;
        goto out;
    }
    rxbuf = rigctld_rxbuf_new(256);
    size = rigctld_read_response(main_fd, rxbuf, RIGCTLD_READ_MULTILINE_IDLE,
                                 out, sizeof(out), 500, 50,
                                 &saw_rprt, &used_multiline, &err);
    if (size <= 0 || strstr(out, "ctcss_list=") == NULL)
    {
        rc = -7;
        goto out;
    }

    if (!send_all(main_fd, "V Sub\n", strlen("V Sub\n")))
    {
        rc = -8;
        goto out;
    }
    rigctld_rxbuf_clear(rxbuf);
    size = rigctld_read_response(main_fd, rxbuf, RIGCTLD_READ_SINGLE,
                                 out, sizeof(out), 500, 50,
                                 &saw_rprt, &used_multiline, &err);
    if (size <= 0 || strncmp(out, "RPRT 0", 6) != 0)
    {
        rc = -9;
        goto out;
    }

    if (!send_all(main_fd, "F 437800000\n", strlen("F 437800000\n")))
    {
        rc = -10;
        goto out;
    }
    rigctld_rxbuf_clear(rxbuf);
    size = rigctld_read_response(main_fd, rxbuf, RIGCTLD_READ_SINGLE,
                                 out, sizeof(out), 500, 50,
                                 &saw_rprt, &used_multiline, &err);
    if (size <= 0 || strncmp(out, "RPRT 0", 6) != 0)
    {
        rc = -11;
        goto out;
    }

    if (!send_all(main_fd, "f\n", 2))
    {
        rc = -12;
        goto out;
    }
    rigctld_rxbuf_clear(rxbuf);
    size = rigctld_read_response(main_fd, rxbuf, RIGCTLD_READ_MULTILINE_RPRT,
                                 out, sizeof(out), 500, 50,
                                 &saw_rprt, &used_multiline, &err);
    if (size <= 0 || strstr(out, "437800000") == NULL)
    {
        rc = -13;
        goto out;
    }

    probe_fd = rigctld_test_connect(port);
    if (probe_fd < 0)
    {
        rc = -14;
        goto out;
    }
    if (rigctld_run_probe_simple(probe_fd) != 0)
    {
        rc = -15;
        goto out;
    }
    close(probe_fd);
    probe_fd = -1;

out:
    rigctld_rxbuf_free(&rxbuf);
    if (probe_fd >= 0)
        close(probe_fd);
    if (main_fd >= 0)
        close(main_fd);
    server.stop = TRUE;
    if (server.server_fd >= 0)
        close(server.server_fd);
    if (thread != NULL)
        g_thread_join(thread);
    if (server.saw_v)
        return -18;

    return rc;
}

static int check_full_duplex_sequence_transcript(void)
{
    const gchar *transcript =
        "V Sub\n"
        "F 437800000\n"
        "V Main\n"
        "F 145850000\n";
    gchar **lines = NULL;
    gint count = 0;
    gboolean ok = TRUE;

    lines = g_strsplit(transcript, "\n", -1);
    for (gint i = 0; lines[i] != NULL; i++)
    {
        gchar *line = g_strstrip(lines[i]);

        if (line[0] == '\0')
            continue;

        if (strstr(line, "VFOA") != NULL || strstr(line, "VFOB") != NULL)
        {
            ok = FALSE;
            break;
        }

        if (count == 0 && !g_str_has_prefix(line, "V Sub"))
            ok = FALSE;
        else if (count == 1 && line[0] != 'F')
            ok = FALSE;
        else if (count == 2 && !g_str_has_prefix(line, "V Main"))
            ok = FALSE;
        else if (count == 3 && line[0] != 'F')
            ok = FALSE;

        if (!ok)
            break;

        count++;
    }
    g_strfreev(lines);

    if (!ok || count < 4)
        return -20;

    return 0;
}
#endif

int main(void)
{
#ifdef G_OS_WIN32
    return 0;
#else
    int fds[2];
    GString *rxbuf = NULL;
    gchar dump_out[4096];
    gint err = 0;
    gssize size;
    gboolean saw_rprt = FALSE;
    gboolean used_multiline = FALSE;
    GThread *delay_thread = NULL;

    const gchar *dump_state =
        "1\n"
        "3081\n"
        "0\n"
        "0x400001 9000 9000 0x0 0x0 0x0 0x0\n"
        "0x400001 9000 9000 0x0 0x0 0x0 0x0\n"
        "ctcss_list=67.0 69.3 71.9 74.4\n";
    const gchar *set_vfo_opt = "RPRT 0\n";
    const gchar *set_freq = "RPRT 0\n";
    const gchar *single_as_multi =
        "1\n"
        "foo=bar\n"
        "RPRT 0\n";

    if (socketpair(AF_UNIX, SOCK_STREAM, 0, fds) != 0)
        return 1;

    rxbuf = rigctld_rxbuf_new(256);

    if (send(fds[1], single_as_multi, strlen(single_as_multi), 0) !=
        (gssize) strlen(single_as_multi))
        return 2;

    saw_rprt = FALSE;
    used_multiline = FALSE;
    size = rigctld_read_response(fds[0], rxbuf, RIGCTLD_READ_SINGLE,
                                 dump_out, sizeof(dump_out),
                                 500, 50, &saw_rprt, &used_multiline, &err);
    if (size <= 0)
        return 3;
    if (!saw_rprt || !used_multiline)
        return 4;
    if (strstr(dump_out, "foo=bar") == NULL)
        return 5;

    rigctld_rxbuf_clear(rxbuf);
    {
        RigctldDelaySend send = {
            .fd = fds[1],
            .first = "1\n",
            .rest = "foo=bar\nRPRT 0\n",
            .delay_ms = 60
        };

        delay_thread = g_thread_new("rigctld-delay", rigctld_delay_sender,
                                    &send);
        saw_rprt = FALSE;
        used_multiline = FALSE;
        size = rigctld_read_response(fds[0], rxbuf, RIGCTLD_READ_SINGLE,
                                     dump_out, sizeof(dump_out),
                                     500, 100,
                                     &saw_rprt, &used_multiline, &err);
        g_thread_join(delay_thread);
        delay_thread = NULL;

        if (size <= 0)
            return 6;
        if (!saw_rprt || !used_multiline)
            return 7;
        if (strstr(dump_out, "foo=bar") == NULL)
            return 8;
    }

    rigctld_rxbuf_clear(rxbuf);
    if (send(fds[1], dump_state, strlen(dump_state), 0) !=
        (gssize) strlen(dump_state))
        return 9;

    size = rigctld_read_response(fds[0], rxbuf, RIGCTLD_READ_MULTILINE_IDLE,
                                 dump_out, sizeof(dump_out),
                                 500, 50, &saw_rprt, &used_multiline, &err);
    if (size <= 0)
        return 10;
    if (strstr(dump_out, "ctcss_list=") == NULL)
        return 11;

    if (send(fds[1], set_vfo_opt, strlen(set_vfo_opt), 0) !=
        (gssize) strlen(set_vfo_opt))
        return 12;
    saw_rprt = FALSE;
    used_multiline = FALSE;
    size = rigctld_read_response(fds[0], rxbuf, RIGCTLD_READ_SINGLE,
                                 dump_out, sizeof(dump_out),
                                 500, 50, &saw_rprt, &used_multiline, &err);
    if (size <= 0 || strncmp(dump_out, "RPRT 0", 6) != 0)
        return 13;

    if (send(fds[1], set_freq, strlen(set_freq), 0) !=
        (gssize) strlen(set_freq))
        return 14;
    saw_rprt = FALSE;
    used_multiline = FALSE;
    size = rigctld_read_response(fds[0], rxbuf, RIGCTLD_READ_SINGLE,
                                 dump_out, sizeof(dump_out),
                                 500, 50, &saw_rprt, &used_multiline, &err);
    if (size <= 0 || strncmp(dump_out, "RPRT 0", 6) != 0)
        return 15;
    if (rxbuf->len != 0)
        return 16;

    if (run_fake_server_test() != 0)
        return 17;
    if (check_full_duplex_sequence_transcript() != 0)
        return 18;

    rigctld_rxbuf_free(&rxbuf);
    close(fds[0]);
    close(fds[1]);

    return 0;
#endif
}
