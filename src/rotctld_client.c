#include "rotctld_client.h"

#include <errno.h>
#include <stdarg.h>
#include <string.h>

#include "rotctld-parse.h"
#include "sat-log.h"

#define ROTCTLD_PROBE_RETRY_DELAY_MS 150
#define ROTCTLD_LOG_THROTTLE_US 2000000

struct _RotctldClient {
    HamlibTransport        *transport;
    rotctld_client_state_t  state;
    gchar                  *state_reason;
    gchar                  *label;
    gint64                  last_probe_us;
    gint64                  last_failure_log_us;
    RotCaps                 caps;
};

static void rotctld_client_set_state(RotctldClient *client,
                                     rotctld_client_state_t state,
                                     const gchar *fmt,
                                     ...) G_GNUC_PRINTF(3, 4);

static void rotctld_client_caps_clear(RotCaps *caps)
{
    if (caps == NULL)
        return;

    g_free(caps->signature);
    caps->signature = NULL;
    caps->model_id = 0;
    caps->has_get_pos = FALSE;
    caps->has_set_pos = FALSE;
    caps->has_stop = FALSE;
    caps->has_park = FALSE;
    caps->limits_valid = FALSE;
    caps->az_min = 0.0;
    caps->az_max = 360.0;
    caps->el_min = 0.0;
    caps->el_max = 180.0;
    caps->quirks = 0;
}

static void rotctld_client_caps_init(RotCaps *caps)
{
    if (caps == NULL)
        return;

    memset(caps, 0, sizeof(*caps));
    caps->az_min = 0.0;
    caps->az_max = 360.0;
    caps->el_min = 0.0;
    caps->el_max = 180.0;
}

static gboolean rotctld_client_parse_first_two_numbers(const gchar *line,
                                                       gdouble *a,
                                                       gdouble *b)
{
    const gchar *p = line;
    gchar *endptr = NULL;
    gdouble first = 0.0;
    gdouble second = 0.0;
    gboolean have_first = FALSE;
    gboolean have_second = FALSE;

    if (line == NULL || a == NULL || b == NULL)
        return FALSE;

    while (*p != '\0')
    {
        if (g_ascii_isdigit(*p) || *p == '-' || *p == '+')
        {
            gdouble val = g_ascii_strtod(p, &endptr);
            if (endptr != p)
            {
                if (!have_first)
                {
                    first = val;
                    have_first = TRUE;
                }
                else
                {
                    second = val;
                    have_second = TRUE;
                    break;
                }
                p = endptr;
                continue;
            }
        }
        p++;
    }

    if (!have_first || !have_second)
        return FALSE;

    *a = first;
    *b = second;
    return TRUE;
}

static gboolean rotctld_client_parse_first_number(const gchar *line,
                                                  gdouble *out)
{
    const gchar *p = line;
    gchar *endptr = NULL;
    gdouble value = 0.0;

    if (line == NULL || out == NULL)
        return FALSE;

    while (*p != '\0')
    {
        if (g_ascii_isdigit(*p) || *p == '-' || *p == '+')
        {
            value = g_ascii_strtod(p, &endptr);
            if (endptr != p)
            {
                *out = value;
                return TRUE;
            }
        }
        p++;
    }

    return FALSE;
}

static void rotctld_client_parse_limits(RotCaps *caps, const gchar *text)
{
    gchar **lines = NULL;
    gboolean have_az_min = FALSE;
    gboolean have_az_max = FALSE;
    gboolean have_el_min = FALSE;
    gboolean have_el_max = FALSE;

    if (caps == NULL || text == NULL || *text == '\0')
        return;

    lines = g_strsplit(text, "\n", -1);
    for (gint i = 0; lines[i] != NULL; i++)
    {
        gchar *line = g_strstrip(lines[i]);
        gchar *lower = NULL;
        gdouble a = 0.0;
        gdouble b = 0.0;

        if (line[0] == '\0')
            continue;

        lower = g_ascii_strdown(line, -1);

        if (!have_az_min && (g_strrstr(lower, "min_az") ||
                             g_strrstr(lower, "az_min") ||
                             g_strrstr(lower, "az min")))
        {
            if (rotctld_client_parse_first_number(line, &a))
            {
                caps->az_min = a;
                have_az_min = TRUE;
            }
        }
        if (!have_az_max && (g_strrstr(lower, "max_az") ||
                             g_strrstr(lower, "az_max") ||
                             g_strrstr(lower, "az max")))
        {
            if (rotctld_client_parse_first_number(line, &a))
            {
                caps->az_max = a;
                have_az_max = TRUE;
            }
        }
        if (!have_el_min && (g_strrstr(lower, "min_el") ||
                             g_strrstr(lower, "el_min") ||
                             g_strrstr(lower, "el min")))
        {
            if (rotctld_client_parse_first_number(line, &a))
            {
                caps->el_min = a;
                have_el_min = TRUE;
            }
        }
        if (!have_el_max && (g_strrstr(lower, "max_el") ||
                             g_strrstr(lower, "el_max") ||
                             g_strrstr(lower, "el max")))
        {
            if (rotctld_client_parse_first_number(line, &a))
            {
                caps->el_max = a;
                have_el_max = TRUE;
            }
        }

        if ((g_strrstr(lower, "azimuth") && g_strrstr(lower, "min") &&
             g_strrstr(lower, "max")) &&
            rotctld_client_parse_first_two_numbers(line, &a, &b))
        {
            caps->az_min = a;
            caps->az_max = b;
            have_az_min = TRUE;
            have_az_max = TRUE;
        }
        if ((g_strrstr(lower, "elevation") && g_strrstr(lower, "min") &&
             g_strrstr(lower, "max")) &&
            rotctld_client_parse_first_two_numbers(line, &a, &b))
        {
            caps->el_min = a;
            caps->el_max = b;
            have_el_min = TRUE;
            have_el_max = TRUE;
        }

        g_free(lower);
    }
    g_strfreev(lines);

    if (have_az_min && have_az_max && have_el_min && have_el_max)
        caps->limits_valid = TRUE;
}

static gboolean rotctld_client_parse_pos(const gchar *text,
                                         gdouble *az_out,
                                         gdouble *el_out)
{
    const gchar *p = text;
    gchar *endptr = NULL;
    gdouble first = 0.0;
    gdouble second = 0.0;
    gboolean have_first = FALSE;
    gboolean have_second = FALSE;

    if (az_out)
        *az_out = 0.0;
    if (el_out)
        *el_out = 0.0;

    if (text == NULL || *text == '\0')
        return FALSE;

    while (*p != '\0')
    {
        if (g_ascii_isdigit(*p) || *p == '-' || *p == '+')
        {
            gdouble val = g_ascii_strtod(p, &endptr);
            if (endptr != p)
            {
                if (!have_first)
                {
                    first = val;
                    have_first = TRUE;
                }
                else
                {
                    second = val;
                    have_second = TRUE;
                    break;
                }
                p = endptr;
                continue;
            }
        }
        p++;
    }

    if (!have_first || !have_second)
        return FALSE;

    if (az_out)
        *az_out = first;
    if (el_out)
        *el_out = second;
    return TRUE;
}

static gboolean rotctld_dump_state_first_line_is_one(const gchar *text)
{
    gchar **lines = NULL;
    gboolean ok = FALSE;

    if (text == NULL || *text == '\0')
        return FALSE;

    lines = g_strsplit(text, "\n", -1);
    for (gint i = 0; lines[i] != NULL; i++)
    {
        gchar *line = g_strstrip(lines[i]);

        if (line[0] == '\0')
            continue;

        ok = (strcmp(line, "1") == 0);
        break;
    }
    g_strfreev(lines);

    return ok;
}

static gboolean rotctld_dump_state_has_done(const gchar *text)
{
    gchar **lines = NULL;
    gboolean ok = FALSE;

    if (text == NULL || *text == '\0')
        return FALSE;

    lines = g_strsplit(text, "\n", -1);
    for (gint i = 0; lines[i] != NULL; i++)
    {
        gchar *line = g_strstrip(lines[i]);

        if (line[0] == '\0')
            continue;

        if (g_ascii_strcasecmp(line, "done") == 0)
        {
            ok = TRUE;
            break;
        }
    }
    g_strfreev(lines);

    return ok;
}

static void rotctld_client_log_failure(RotctldClient *client,
                                       const gchar *label,
                                       const gchar *cmd,
                                       const gchar *reply)
{
    gsize bytes = reply ? strlen(reply) : 0;
    gchar *snippet = NULL;
    gint64 now_us = g_get_monotonic_time();

    if (client != NULL &&
        client->last_failure_log_us > 0 &&
        (now_us - client->last_failure_log_us) < ROTCTLD_LOG_THROTTLE_US)
        return;

    if (client != NULL)
        client->last_failure_log_us = now_us;

    if (reply && *reply)
    {
        if (bytes > 240)
            snippet = g_strdup_printf("%.*s...", 240, reply);
        else
            snippet = g_strdup(reply);
    }
    else
    {
        snippet = g_strdup("(none)");
    }

    sat_log_log(SAT_LOG_LEVEL_ERROR,
                "%s: cmd='%s' bytes=%" G_GSIZE_FORMAT " reply=%s",
                label,
                cmd ? cmd : "(null)",
                bytes,
                snippet ? snippet : "(null)");

    g_free(snippet);
}

static void rotctld_client_set_state(RotctldClient *client,
                                     rotctld_client_state_t state,
                                     const gchar *fmt,
                                     ...)
{
    va_list args;

    if (client == NULL)
        return;

    client->state = state;
    g_free(client->state_reason);
    client->state_reason = NULL;

    if (fmt != NULL)
    {
        va_start(args, fmt);
        client->state_reason = g_strdup_vprintf(fmt, args);
        va_end(args);
    }
}

RotctldClient *rotctld_client_new(const gchar *label)
{
    RotctldClient *client = g_new0(RotctldClient, 1);

    client->transport = hamlib_transport_new();
    client->state = ROTCTLD_CLIENT_STOPPED;
    client->label = g_strdup(label ? label : "rot");
    rotctld_client_caps_init(&client->caps);
    client->last_failure_log_us = 0;

    return client;
}

void rotctld_client_free(RotctldClient **client)
{
    if (client == NULL || *client == NULL)
        return;

    rotctld_client_close(*client);
    rotctld_client_caps_clear(&(*client)->caps);
    g_free((*client)->state_reason);
    g_free((*client)->label);
    g_free(*client);
    *client = NULL;
}

void rotctld_client_reset(RotctldClient *client)
{
    if (client == NULL)
        return;

    rotctld_client_caps_clear(&client->caps);
    rotctld_client_set_state(client, ROTCTLD_CLIENT_STOPPED, "reset");
    client->last_probe_us = 0;
    client->last_failure_log_us = 0;
}

gboolean rotctld_client_connect(RotctldClient *client,
                                const gchar *host,
                                gint port,
                                gint timeout_ms,
                                gchar **error_out)
{
    gboolean ok = FALSE;

    if (client == NULL)
        return FALSE;

    rotctld_client_caps_clear(&client->caps);
    client->last_probe_us = 0;

    rotctld_client_set_state(client, ROTCTLD_CLIENT_CONNECTING, "connect");
    ok = hamlib_transport_connect(client->transport, host, port,
                                  timeout_ms, error_out);
    if (!ok)
    {
        rotctld_client_set_state(client, ROTCTLD_CLIENT_DEGRADED,
                                 "connect failed");
        return FALSE;
    }

    rotctld_client_set_state(client, ROTCTLD_CLIENT_CONNECTING, "connected");
    return TRUE;
}

void rotctld_client_close(RotctldClient *client)
{
    if (client == NULL)
        return;

    hamlib_transport_close(client->transport);
    rotctld_client_set_state(client, ROTCTLD_CLIENT_STOPPED, "closed");
}

rotctld_client_state_t rotctld_client_get_state(const RotctldClient *client)
{
    if (client == NULL)
        return ROTCTLD_CLIENT_STOPPED;
    return client->state;
}

const gchar *rotctld_client_get_state_reason(const RotctldClient *client)
{
    if (client == NULL)
        return NULL;
    return client->state_reason;
}

const RotCaps *rotctld_client_get_caps(const RotctldClient *client)
{
    if (client == NULL)
        return NULL;
    return &client->caps;
}

HamlibTransport *rotctld_client_get_transport(RotctldClient *client)
{
    if (client == NULL)
        return NULL;
    return client->transport;
}

gboolean rotctld_client_probe(RotctldClient *client,
                              gint timeout_ms)
{
    gchar dump_state[4096];
    HamlibResponseInfo info = { 0 };
    gdouble az = 0.0;
    gdouble el = 0.0;
    gint64 now_us = g_get_monotonic_time();

    if (client == NULL || client->transport == NULL)
        return FALSE;

    if (client->last_probe_us > 0 &&
        (now_us - client->last_probe_us) < 500000)
    {
        return (client->state == ROTCTLD_CLIENT_READY);
    }

    client->last_probe_us = now_us;
    rotctld_client_set_state(client, ROTCTLD_CLIENT_PROBING, "probe start");

    if (!hamlib_transport_request(client->transport,
                                  "\\dump_state\n",
                                  HAMLIB_READ_MULTILINE_IDLE,
                                  HAMLIB_TERM_RPRT_OR_DONE,
                                  timeout_ms,
                                  50,
                                  1, ROTCTLD_PROBE_RETRY_DELAY_MS,
                                  dump_state, sizeof(dump_state),
                                  &info))
    {
        rotctld_client_set_state(client, ROTCTLD_CLIENT_DEGRADED,
                                 "dump_state failed");
        return FALSE;
    }

    rotctld_client_caps_clear(&client->caps);
    rotctld_parse_model(dump_state, &client->caps.model_id);
    client->caps.signature = g_strdup(dump_state);
    rotctld_client_parse_limits(&client->caps, dump_state);

    for (gint attempt = 0; attempt < 3; attempt++)
    {
        if (rotctld_client_get_pos(client, &az, &el))
        {
            client->caps.has_get_pos = TRUE;
            break;
        }

        (void)hamlib_transport_drain(client->transport, 50, NULL);
        g_usleep((gulong)ROTCTLD_PROBE_RETRY_DELAY_MS * 1000);
    }

    if (client->caps.has_get_pos)
    {
        if (rotctld_client_set_pos(client, az, el))
            client->caps.has_set_pos = TRUE;
    }

    rotctld_client_set_state(client, ROTCTLD_CLIENT_READY, "probe ok");
    return client->caps.has_get_pos;
}

gboolean rotctld_client_handshake(RotctldClient *client,
                                  gint timeout_ms,
                                  gdouble *az_out,
                                  gdouble *el_out,
                                  gchar *dump_state_out,
                                  gsize dump_state_len)
{
    gchar dump_buf[4096];
    gchar reply[256];
    HamlibResponseInfo info = { 0 };
    gdouble az = 0.0;
    gdouble el = 0.0;
    gboolean ok = FALSE;

    if (az_out)
        *az_out = 0.0;
    if (el_out)
        *el_out = 0.0;
    if (dump_state_out && dump_state_len > 0)
        dump_state_out[0] = '\0';

    if (client == NULL || client->transport == NULL ||
        !hamlib_transport_is_ready(client->transport))
        return FALSE;

    rotctld_client_set_state(client, ROTCTLD_CLIENT_PROBING, "handshake");

    (void)hamlib_transport_clear_rxbuf(client->transport);

    if (dump_state_out == NULL || dump_state_len == 0)
    {
        dump_state_out = dump_buf;
        dump_state_len = sizeof(dump_buf);
        dump_state_out[0] = '\0';
    }

    ok = hamlib_transport_request(client->transport,
                                  "\\dump_state\n",
                                  HAMLIB_READ_MULTILINE_IDLE,
                                  HAMLIB_TERM_RPRT_OR_DONE,
                                  timeout_ms,
                                  50,
                                  1, ROTCTLD_PROBE_RETRY_DELAY_MS,
                                  dump_state_out, dump_state_len,
                                  &info);
    if (!ok)
    {
        rotctld_client_log_failure(client,
                                   "handshake dump_state failed",
                                   "\\dump_state",
                                   dump_state_out);
        rotctld_client_set_state(client, ROTCTLD_CLIENT_DEGRADED,
                                 "handshake dump_state failed");
        (void)hamlib_transport_clear_rxbuf(client->transport);
        hamlib_transport_close(client->transport);
        return FALSE;
    }

    if (!rotctld_dump_state_first_line_is_one(dump_state_out))
    {
        rotctld_client_log_failure(client,
                                   "handshake dump_state invalid (expected line1=1)",
                                   "\\dump_state",
                                   dump_state_out);
        rotctld_client_set_state(client, ROTCTLD_CLIENT_DEGRADED,
                                 "handshake dump_state invalid");
        (void)hamlib_transport_clear_rxbuf(client->transport);
        hamlib_transport_close(client->transport);
        return FALSE;
    }
    if (!rotctld_dump_state_has_done(dump_state_out))
    {
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rotctld handshake: dump_state missing done; accepting");
    }

    memset(&info, 0, sizeof(info));
    ok = hamlib_transport_request(client->transport,
                                  "p\n",
                                  HAMLIB_READ_MULTILINE_RPRT,
                                  HAMLIB_TERM_RPRT_OR_DONE,
                                  timeout_ms,
                                  50,
                                  1, ROTCTLD_PROBE_RETRY_DELAY_MS,
                                  reply, sizeof(reply),
                                  &info);
    if (!ok)
    {
        rotctld_client_log_failure(client,
                                   "handshake get_pos failed",
                                   "p",
                                   reply);
        rotctld_client_set_state(client, ROTCTLD_CLIENT_DEGRADED,
                                 "handshake get_pos failed");
        (void)hamlib_transport_clear_rxbuf(client->transport);
        hamlib_transport_close(client->transport);
        return FALSE;
    }

    if (info.saw_rprt && info.rprt_code != 0)
    {
        rotctld_client_log_failure(client,
                                   "handshake get_pos rejected",
                                   "p",
                                   reply);
        rotctld_client_set_state(client, ROTCTLD_CLIENT_DEGRADED,
                                 "handshake get_pos rejected");
        (void)hamlib_transport_clear_rxbuf(client->transport);
        hamlib_transport_close(client->transport);
        return FALSE;
    }

    if (!rotctld_client_parse_pos(reply, &az, &el))
    {
        rotctld_client_log_failure(client,
                                   "handshake get_pos parse failed (expected az/el)",
                                   "p",
                                   reply);
        rotctld_client_set_state(client, ROTCTLD_CLIENT_DEGRADED,
                                 "handshake get_pos parse failed");
        (void)hamlib_transport_clear_rxbuf(client->transport);
        hamlib_transport_close(client->transport);
        return FALSE;
    }

    if (az_out)
        *az_out = az;
    if (el_out)
        *el_out = el;

    sat_log_log(SAT_LOG_LEVEL_INFO,
                "rotor handshake OK az=%.2f el=%.2f RTT=%.1fms",
                az, el,
                hamlib_transport_last_rtt_us(client->transport) / 1000.0);

    rotctld_client_set_state(client, ROTCTLD_CLIENT_READY, "handshake ok");
    return TRUE;
}

gssize rotctld_client_clear_rxbuf(RotctldClient *client)
{
    if (client == NULL || client->transport == NULL)
        return -1;

    return hamlib_transport_clear_rxbuf(client->transport);
}

gboolean rotctld_client_get_pos(RotctldClient *client,
                                gdouble *az_out,
                                gdouble *el_out)
{
    gchar reply[256];
    HamlibResponseInfo info = { 0 };
    gboolean ok = FALSE;

    if (az_out)
        *az_out = 0.0;
    if (el_out)
        *el_out = 0.0;

    if (client == NULL)
        return FALSE;

    ok = hamlib_transport_request(client->transport,
                                  "p\n",
                                  HAMLIB_READ_MULTILINE_RPRT,
                                  HAMLIB_TERM_RPRT_OR_DONE,
                                  500,
                                  50,
                                  0, 0,
                                  reply, sizeof(reply),
                                  &info);
    if (!ok)
        return FALSE;

    if (info.saw_rprt && info.rprt_code != 0)
        return FALSE;

    return rotctld_client_parse_pos(reply, az_out, el_out);
}

gboolean rotctld_client_set_pos(RotctldClient *client,
                                gdouble az,
                                gdouble el)
{
    gchar cmd[96];
    gchar reply[128];
    HamlibResponseInfo info = { 0 };
    gboolean ok = FALSE;

    if (client == NULL)
        return FALSE;

    g_snprintf(cmd, sizeof(cmd), "P %.2f %.2f\x0a", az, el);
    ok = hamlib_transport_request(client->transport,
                                  cmd,
                                  HAMLIB_READ_MULTILINE_RPRT,
                                  HAMLIB_TERM_RPRT_OR_DONE,
                                  500,
                                  50,
                                  0, 0,
                                  reply, sizeof(reply),
                                  &info);
    if (!ok)
        return FALSE;

    if (info.saw_rprt && info.rprt_code != 0)
        return FALSE;

    return TRUE;
}

gboolean rotctld_client_stop(RotctldClient *client)
{
    gchar reply[128];
    HamlibResponseInfo info = { 0 };

    if (client == NULL)
        return FALSE;

    if (!hamlib_transport_request(client->transport,
                                  "S\n",
                                  HAMLIB_READ_MULTILINE_RPRT,
                                  HAMLIB_TERM_RPRT_OR_DONE,
                                  500,
                                  50,
                                  0, 0,
                                  reply, sizeof(reply),
                                  &info))
        return FALSE;

    if (info.saw_rprt && info.rprt_code != 0)
        return FALSE;

    return TRUE;
}

gboolean rotctld_client_request_raw(RotctldClient *client,
                                    const gchar *cmd,
                                    gchar *out,
                                    gsize out_len,
                                    HamlibResponseInfo *info)
{
    hamlib_read_mode_t mode = HAMLIB_READ_MULTILINE_RPRT;
    HamlibResponseInfo local = { 0 };
    gboolean ok = FALSE;

    if (info)
        memset(info, 0, sizeof(*info));

    if (client == NULL || cmd == NULL)
        return FALSE;

    if (g_str_has_prefix(cmd, "\\dump_state"))
        mode = HAMLIB_READ_MULTILINE_IDLE;

    ok = hamlib_transport_request(client->transport,
                                  cmd,
                                  mode,
                                  HAMLIB_TERM_RPRT_OR_DONE,
                                  1000,
                                  50,
                                  0, 0,
                                  out, out_len,
                                  &local);
    if (info)
        *info = local;
    if (!ok)
        return FALSE;

    if (local.saw_rprt && local.rprt_code != 0)
        return FALSE;

    return TRUE;
}

gint64 rotctld_client_last_rtt_us(const RotctldClient *client)
{
    if (client == NULL || client->transport == NULL)
        return 0;
    return hamlib_transport_last_rtt_us(client->transport);
}
