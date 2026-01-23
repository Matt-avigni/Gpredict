#include "rotctld_client.h"

#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <string.h>

#include "rotctld-parse.h"
#include "sat-log.h"

#define ROTCTLD_PROBE_RETRY_DELAY_MS 150
#define ROTCTLD_LOG_THROTTLE_US 2000000
#define ROTCTLD_GETPOS_TIMEOUT_MS 1500
#define ROTCTLD_GETPOS_RETRIES 2
#define ROTCTLD_GETPOS_RETRY_DELAY_MS 50
#define ROTCTLD_SETPOS_TIMEOUT_MS 1500
#define ROTCTLD_SETPOS_RETRIES 1
#define ROTCTLD_SETPOS_RETRY_DELAY_MS 100

struct _RotctldClient {
    HamlibTransport        *transport;
    rotctld_client_state_t  state;
    gchar                  *state_reason;
    gchar                  *label;
    gint64                  last_probe_us;
    gint64                  last_failure_log_us;
    gint64                  last_parse_log_us;
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
    caps->south_zero = FALSE;
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
    caps->south_zero = FALSE;
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

static gboolean rotctld_client_line_parse_rprt(const gchar *line,
                                               gint *code_out)
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
        return FALSE;

    if (code_out)
        *code_out = (gint) code;
    return TRUE;
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

        if (g_strrstr(lower, "south_zero") ||
            g_strrstr(lower, "south zero"))
        {
            if (rotctld_client_parse_first_number(line, &a))
                caps->south_zero = (fabs(a) > 0.5);
        }

        g_free(lower);
    }
    g_strfreev(lines);

    if (have_az_min && have_az_max && have_el_min && have_el_max)
        caps->limits_valid = TRUE;
}

static void rotctld_client_update_caps_from_dump(RotctldClient *client,
                                                 const gchar *dump_state)
{
    if (client == NULL || dump_state == NULL)
        return;

    rotctld_client_caps_clear(&client->caps);
    rotctld_parse_model(dump_state, &client->caps.model_id);
    client->caps.signature = g_strdup(dump_state);
    rotctld_client_parse_limits(&client->caps, dump_state);
}

typedef struct {
    gboolean valid;
    gdouble  az_min;
    gdouble  az_max;
    gdouble  el_min;
    gdouble  el_max;
} rot_limits_t;

static rot_limits_t rotctld_client_limits_from_caps(const RotCaps *caps)
{
    rot_limits_t lim;

    lim.valid = FALSE;
    lim.az_min = 0.0;
    lim.az_max = 360.0;
    lim.el_min = 0.0;
    lim.el_max = 180.0;

    if (caps != NULL && caps->limits_valid)
    {
        lim.valid = TRUE;
        lim.az_min = caps->az_min;
        lim.az_max = caps->az_max;
        lim.el_min = caps->el_min;
        lim.el_max = caps->el_max;
    }

    return lim;
}

static gboolean rotctld_client_is_gs232b_model(gint model_id)
{
    return model_id == 603;
}

static gboolean normalize_gs232b_pos(gdouble *az, gdouble *el,
                                     const rot_limits_t *lim)
{
    gdouble az_norm = 0.0;
    gdouble el_norm = 0.0;
    gdouble el_min = 0.0;
    gdouble el_max = 180.0;

    if (az == NULL || el == NULL)
        return FALSE;

    if (!isfinite(*az) || !isfinite(*el))
        return FALSE;

    az_norm = *az;
    el_norm = *el;

    if (fabs(az_norm - 360.0) < 1e-6)
        az_norm = 0.0;
    az_norm = fmod(az_norm, 360.0);
    if (az_norm < 0.0)
        az_norm += 360.0;
    if (az_norm >= 360.0)
        az_norm = 0.0;

    if (lim != NULL && lim->valid)
    {
        el_min = MAX(el_min, lim->el_min);
        el_max = MIN(el_max, lim->el_max);
        if (el_min > el_max)
        {
            el_min = 0.0;
            el_max = 180.0;
        }
    }

    el_norm = CLAMP(el_norm, el_min, el_max);

    *az = az_norm;
    *el = el_norm;
    return TRUE;
}

static gboolean rotctld_client_parse_pos(const gchar *text,
                                         gdouble *az_out,
                                         gdouble *el_out,
                                         gboolean *saw_rprt_out,
                                         gint *rprt_code_out)
{
    gboolean have_az = FALSE;
    gboolean have_el = FALSE;
    gboolean saw_rprt = FALSE;
    gint rprt_code = 0;

    if (az_out)
        *az_out = 0.0;
    if (el_out)
        *el_out = 0.0;
    if (saw_rprt_out)
        *saw_rprt_out = FALSE;
    if (rprt_code_out)
        *rprt_code_out = 0;

    if (text == NULL || *text == '\0')
        return FALSE;

    gchar **lines = g_strsplit(text, "\n", -1);
    for (gint i = 0; lines[i] != NULL; i++)
    {
        gchar *line = g_strstrip(lines[i]);
        gdouble a = 0.0;
        gdouble b = 0.0;

        if (line[0] == '\0')
            continue;

        if (rotctld_client_line_parse_rprt(line, &rprt_code))
        {
            saw_rprt = TRUE;
            continue;
        }

        if (g_ascii_strcasecmp(line, "done") == 0)
        {
            continue;
        }

        if (!have_az && !have_el &&
            rotctld_client_parse_first_two_numbers(line, &a, &b))
        {
            if (az_out)
                *az_out = a;
            if (el_out)
                *el_out = b;
            have_az = TRUE;
            have_el = TRUE;
            continue;
        }

        if (!have_az && rotctld_client_parse_first_number(line, &a))
        {
            if (az_out)
                *az_out = a;
            have_az = TRUE;
            continue;
        }

        if (have_az && !have_el && rotctld_client_parse_first_number(line, &b))
        {
            if (el_out)
                *el_out = b;
            have_el = TRUE;
            continue;
        }
    }
    g_strfreev(lines);

    if (saw_rprt_out)
        *saw_rprt_out = saw_rprt;
    if (rprt_code_out)
        *rprt_code_out = saw_rprt ? rprt_code : 0;

    return have_az && have_el;
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

        ok = g_str_has_prefix(line, "1");
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

static void rotctld_client_log_parse_debug(RotctldClient *client,
                                           const gchar *label,
                                           const gchar *reply)
{
    gint64 now_us = g_get_monotonic_time();

    if (client != NULL &&
        client->last_parse_log_us > 0 &&
        (now_us - client->last_parse_log_us) < ROTCTLD_LOG_THROTTLE_US)
        return;

    if (client != NULL)
        client->last_parse_log_us = now_us;

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "%s: raw reply=%s",
                label ? label : "parse failed",
                (reply && *reply) ? reply : "(none)");
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
    client->last_parse_log_us = 0;

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
    client->last_parse_log_us = 0;
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

    if (client->caps.signature != NULL &&
        client->caps.has_get_pos &&
        client->state == ROTCTLD_CLIENT_READY &&
        hamlib_transport_is_ready(client->transport) &&
        client->last_probe_us > 0 &&
        (now_us - client->last_probe_us) < 500000)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "rotctld probe skipped: READY (caps cached)");
        return TRUE;
    }

    if (client->last_probe_us > 0 &&
        (now_us - client->last_probe_us) < 500000)
    {
        return (client->state == ROTCTLD_CLIENT_READY);
    }

    client->last_probe_us = now_us;

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
        sat_log_log(SAT_LOG_LEVEL_WARN,
                    "rotctld probe: dump_state failed");
        return FALSE;
    }

    rotctld_client_update_caps_from_dump(client, dump_state);
    (void)hamlib_transport_clear_rxbuf(client->transport);
    (void)hamlib_transport_drain(client->transport, 50, NULL);
    sat_log_log(SAT_LOG_LEVEL_INFO,
                "rotctld dump_state caps: model=%d az=%.2f..%.2f el=%.2f..%.2f south_zero=%d limits=%d",
                client->caps.model_id,
                client->caps.az_min, client->caps.az_max,
                client->caps.el_min, client->caps.el_max,
                client->caps.south_zero ? 1 : 0,
                client->caps.limits_valid ? 1 : 0);

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
    else
    {
        rotctld_client_caps_clear(&client->caps);
        sat_log_log(SAT_LOG_LEVEL_WARN,
                    "rotctld probe: get_position failed");
        return FALSE;
    }

    return TRUE;
}

gboolean rotctld_client_handshake(RotctldClient *client,
                                  gint timeout_ms,
                                  gdouble *az_out,
                                  gdouble *el_out,
                                  gchar *dump_state_out,
                                  gsize dump_state_len,
                                  gchar *pos_reply_out,
                                  gsize pos_reply_len,
                                  gboolean *pos_ok_out)
{
    gchar dump_buf[4096];
    gchar reply[256];
    HamlibResponseInfo info = { 0 };
    gdouble az = 0.0;
    gdouble el = 0.0;
    gboolean pos_ok = FALSE;
    gboolean ok = FALSE;
    gint model_id = 0;
    RotCaps dump_caps;

    if (pos_ok_out)
        *pos_ok_out = FALSE;
    if (az_out)
        *az_out = 0.0;
    if (el_out)
        *el_out = 0.0;
    if (dump_state_out && dump_state_len > 0)
        dump_state_out[0] = '\0';
    if (pos_reply_out && pos_reply_len > 0)
        pos_reply_out[0] = '\0';

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

    sat_log_log(SAT_LOG_LEVEL_INFO, "handshake: dump_state ok");

    if (!rotctld_dump_state_first_line_is_one(dump_state_out))
    {
        sat_log_log(SAT_LOG_LEVEL_WARN,
                    "rotctld handshake: dump_state first line not '1'; accepting");
    }
    if (!rotctld_dump_state_has_done(dump_state_out))
    {
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rotctld handshake: dump_state missing done; accepting");
    }

    rotctld_client_caps_init(&dump_caps);
    if (rotctld_parse_model(dump_state_out, &model_id))
        dump_caps.model_id = model_id;
    rotctld_client_parse_limits(&dump_caps, dump_state_out);
    if (client != NULL && model_id != 0)
        client->caps.model_id = model_id;
    rotctld_client_update_caps_from_dump(client, dump_state_out);
    client->last_probe_us = g_get_monotonic_time();
    (void)hamlib_transport_clear_rxbuf(client->transport);
    (void)hamlib_transport_drain(client->transport, 50, NULL);

    memset(&info, 0, sizeof(info));
    reply[0] = '\0';
    {
        rotctld_pos_result_t pos_res = rotctld_client_get_pos_ex_timeout(client,
                                                                         &az, &el,
                                                                         &info,
                                                                         reply,
                                                                         sizeof(reply),
                                                                         timeout_ms,
                                                                         ROTCTLD_GETPOS_RETRIES);
        if (pos_reply_out && pos_reply_len > 0)
            g_strlcpy(pos_reply_out, reply, pos_reply_len);

        if (pos_res == ROTCTLD_POS_OK)
        {
            pos_ok = TRUE;
        }
        else if (pos_res == ROTCTLD_POS_RPRT_ERR &&
                 info.saw_rprt && info.rprt_code == -6)
        {
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        "handshake: get_position failed (-6), continuing without position");
        }
        else
        {
            const gchar *reason = NULL;

            if (pos_res == ROTCTLD_POS_TIMEOUT)
                reason = "timeout";
            else if (pos_res == ROTCTLD_POS_RPRT_ERR)
                reason = "rprt";
            else if (pos_res == ROTCTLD_POS_PARSE_FAIL)
                reason = "parse";
            else
                reason = "io";

            if (g_strcmp0(reason, "rprt") == 0)
            {
                rotctld_client_log_failure(client,
                                           "handshake get_pos rejected",
                                           "p",
                                           reply);
            }
            else if (g_strcmp0(reason, "parse") == 0)
            {
                rotctld_client_log_failure(client,
                                           "handshake get_pos parse failed (expected az/el)",
                                           "p",
                                           reply);
            }
            else
            {
                rotctld_client_log_failure(client,
                                           "handshake get_pos failed",
                                           "p",
                                           reply);
            }

            sat_log_log(SAT_LOG_LEVEL_WARN,
                        "handshake get_pos failed; continuing without position");
        }
    }

    if (pos_ok_out)
        *pos_ok_out = pos_ok;

    if (az_out)
        *az_out = az;
    if (el_out)
        *el_out = el;
    if (pos_ok)
        client->caps.has_get_pos = TRUE;

    sat_log_log(SAT_LOG_LEVEL_INFO,
                "rotor handshake OK az=%.2f el=%.2f RTT=%.1fms",
                az, el,
                hamlib_transport_last_rtt_us(client->transport) / 1000.0);

    if (pos_ok)
        rotctld_client_set_state(client, ROTCTLD_CLIENT_READY, "handshake ok");
    else
        rotctld_client_set_state(client, ROTCTLD_CLIENT_CONNECTING,
                                 "handshake ok (waiting position)");
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
    return rotctld_client_get_pos_ex(client, az_out, el_out, NULL, NULL, 0) ==
        ROTCTLD_POS_OK;
}

rotctld_pos_result_t rotctld_client_get_pos_ex(RotctldClient *client,
                                               gdouble *az_out,
                                               gdouble *el_out,
                                               HamlibResponseInfo *info,
                                               gchar *reply_out,
                                               gsize reply_len)
{
    return rotctld_client_get_pos_ex_timeout(client,
                                             az_out,
                                             el_out,
                                             info,
                                             reply_out,
                                             reply_len,
                                             ROTCTLD_GETPOS_TIMEOUT_MS,
                                             ROTCTLD_GETPOS_RETRIES);
}

rotctld_pos_result_t rotctld_client_get_pos_ex_timeout(RotctldClient *client,
                                                       gdouble *az_out,
                                                       gdouble *el_out,
                                                       HamlibResponseInfo *info,
                                                       gchar *reply_out,
                                                       gsize reply_len,
                                                       gint timeout_ms,
                                                       gint retries)
{
    gchar reply_local[256];
    gchar *reply = reply_local;
    gsize reply_cap = sizeof(reply_local);
    HamlibResponseInfo local = { 0 };
    gboolean ok = FALSE;
    gint retry_delay_ms = ROTCTLD_GETPOS_RETRY_DELAY_MS;

    if (az_out)
        *az_out = 0.0;
    if (el_out)
        *el_out = 0.0;
    if (info)
        memset(info, 0, sizeof(*info));

    if (reply_out && reply_len > 0)
    {
        reply = reply_out;
        reply_cap = reply_len;
    }

    if (reply && reply_cap > 0)
        reply[0] = '\0';

    if (client == NULL || client->transport == NULL ||
        !hamlib_transport_is_ready(client->transport))
        return ROTCTLD_POS_IO_ERR;

    if (timeout_ms <= 0)
        timeout_ms = ROTCTLD_GETPOS_TIMEOUT_MS;
    if (retries < 0)
        retries = ROTCTLD_GETPOS_RETRIES;

    for (gint attempt = 0; attempt <= retries; attempt++)
    {
        gboolean saw_rprt = FALSE;
        gint rprt_code = 0;
        gboolean parsed = FALSE;

        memset(&local, 0, sizeof(local));
        if (reply && reply_cap > 0)
            reply[0] = '\0';

        ok = hamlib_transport_request(client->transport,
                                      "p\n",
                                      HAMLIB_READ_MULTILINE_RPRT,
                                      HAMLIB_TERM_RPRT_OR_DONE,
                                      timeout_ms,
                                      50,
                                      0, 0,
                                      reply, reply_cap,
                                      &local);

        if (!ok)
        {
            gboolean is_timeout =
                (local.err == EAGAIN ||
                 local.err == EWOULDBLOCK ||
                 local.err == ETIMEDOUT);

            if (attempt < retries && is_timeout)
            {
                (void)hamlib_transport_clear_rxbuf(client->transport);
                (void)hamlib_transport_drain(client->transport, 50, NULL);
                g_usleep((gulong)retry_delay_ms * 1000);
                retry_delay_ms = MIN(retry_delay_ms * 2, 500);
                continue;
            }
            if (info)
                *info = local;
            return is_timeout ? ROTCTLD_POS_TIMEOUT : ROTCTLD_POS_IO_ERR;
        }

        parsed = rotctld_client_parse_pos(reply, az_out, el_out,
                                          &saw_rprt, &rprt_code);
        if (local.saw_rprt)
        {
            saw_rprt = TRUE;
            rprt_code = local.rprt_code;
        }
        if (saw_rprt && !local.saw_rprt)
        {
            local.saw_rprt = TRUE;
            local.rprt_code = rprt_code;
        }

        if (saw_rprt && rprt_code != 0)
        {
            if (info)
                *info = local;
            return ROTCTLD_POS_RPRT_ERR;
        }

        if (parsed)
        {
            if (rotctld_client_is_gs232b_model(client->caps.model_id))
            {
                rot_limits_t limits = rotctld_client_limits_from_caps(&client->caps);
                if (!normalize_gs232b_pos(az_out, el_out, &limits))
                    return ROTCTLD_POS_PARSE_FAIL;
            }
            if (info)
                *info = local;
            return ROTCTLD_POS_OK;
        }

        rotctld_client_log_parse_debug(client,
                                       "get_position parse failed",
                                       reply);

        if (attempt < retries)
        {
            (void)hamlib_transport_clear_rxbuf(client->transport);
            (void)hamlib_transport_drain(client->transport, 50, NULL);
            g_usleep((gulong)retry_delay_ms * 1000);
            retry_delay_ms = MIN(retry_delay_ms * 2, 500);
            continue;
        }

        rotctld_client_log_failure(client,
                                   "get_position parse failed (expected az/el)",
                                   "p",
                                   reply);
        if (info)
            *info = local;
        return ROTCTLD_POS_PARSE_FAIL;
    }

    return ROTCTLD_POS_IO_ERR;
}

gboolean rotctld_client_set_pos_ex(RotctldClient *client,
                                   gdouble az,
                                   gdouble el,
                                   gint *rprt_code_out,
                                   HamlibResponseInfo *info_out,
                                   gchar *reply_out,
                                   gsize reply_len)
{
    gchar cmd[96];
    gchar reply_local[128];
    gchar *reply = reply_local;
    gsize reply_cap = sizeof(reply_local);
    HamlibResponseInfo local = { 0 };
    gboolean ok = FALSE;
    gint retry_delay_ms = ROTCTLD_SETPOS_RETRY_DELAY_MS;

    if (rprt_code_out)
        *rprt_code_out = 0;
    if (info_out)
        memset(info_out, 0, sizeof(*info_out));

    if (reply_out && reply_len > 0)
    {
        reply = reply_out;
        reply_cap = reply_len;
    }

    if (reply && reply_cap > 0)
        reply[0] = '\0';

    if (client == NULL || client->transport == NULL ||
        !hamlib_transport_is_ready(client->transport))
        return FALSE;

    g_snprintf(cmd, sizeof(cmd), "P %.2f %.2f\n", az, el);

    for (gint attempt = 0; attempt <= ROTCTLD_SETPOS_RETRIES; attempt++)
    {
        memset(&local, 0, sizeof(local));
        if (reply && reply_cap > 0)
            reply[0] = '\0';

        ok = hamlib_transport_request(client->transport,
                                      cmd,
                                      HAMLIB_READ_SINGLE,
                                      HAMLIB_TERM_RPRT,
                                      ROTCTLD_SETPOS_TIMEOUT_MS,
                                      50,
                                      0, 0,
                                      reply, reply_cap,
                                      &local);
        if (!ok)
        {
            if (attempt < ROTCTLD_SETPOS_RETRIES &&
                (local.err == EAGAIN ||
                 local.err == EWOULDBLOCK ||
                 local.err == ETIMEDOUT))
            {
                (void)hamlib_transport_clear_rxbuf(client->transport);
                (void)hamlib_transport_drain(client->transport, 50, NULL);
                g_usleep((gulong)retry_delay_ms * 1000);
                retry_delay_ms = MIN(retry_delay_ms * 2, 500);
                continue;
            }
            if (info_out)
                *info_out = local;
            return FALSE;
        }

        if (info_out)
            *info_out = local;

        if (local.used_multiline)
        {
            rotctld_client_log_failure(client,
                                       "set_position protocol error (unexpected multiline reply)",
                                       "P",
                                       reply);
            (void)hamlib_transport_clear_rxbuf(client->transport);
            (void)hamlib_transport_drain(client->transport, 50, NULL);
            return FALSE;
        }

        if (!local.saw_rprt)
        {
            rotctld_client_log_failure(client,
                                       "set_position protocol error (expected RPRT)",
                                       "P",
                                       reply);
            (void)hamlib_transport_clear_rxbuf(client->transport);
            (void)hamlib_transport_drain(client->transport, 50, NULL);
            return FALSE;
        }

        if (rprt_code_out)
            *rprt_code_out = local.rprt_code;

        if (local.rprt_code != 0)
            return FALSE;

        return TRUE;
    }

    return FALSE;
}

gboolean rotctld_client_set_pos(RotctldClient *client,
                                gdouble az,
                                gdouble el)
{
    return rotctld_client_set_pos_ex(client, az, el, NULL, NULL, NULL, 0);
}

gboolean rotctld_client_stop(RotctldClient *client)
{
    gchar reply[128];
    HamlibResponseInfo info = { 0 };

    if (client == NULL)
        return FALSE;

    if (!rotctld_client_request_raw(client, "S",
                                    reply, sizeof(reply), &info))
        return FALSE;

    return !(info.saw_rprt && info.rprt_code != 0);
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
    const gchar *send_cmd = NULL;
    gchar *tmp_cmd = NULL;
    gsize cmd_len = 0;

    if (info)
        memset(info, 0, sizeof(*info));

    if (client == NULL || cmd == NULL)
        return FALSE;

    cmd_len = strlen(cmd);
    if (cmd_len > 0 && cmd[cmd_len - 1] != '\n')
    {
        tmp_cmd = g_strdup_printf("%s\n", cmd);
        send_cmd = tmp_cmd;
    }
    else
    {
        send_cmd = cmd;
    }

    if (g_str_has_prefix(send_cmd, "\\dump_state"))
        mode = HAMLIB_READ_MULTILINE_IDLE;

    ok = hamlib_transport_request(client->transport,
                                  send_cmd,
                                  mode,
                                  HAMLIB_TERM_RPRT_OR_DONE,
                                  1000,
                                  50,
                                  0, 0,
                                  out, out_len,
                                  &local);
    g_free(tmp_cmd);
    if (info)
        *info = local;
    if (!ok)
        return FALSE;

    return TRUE;
}

gint64 rotctld_client_last_rtt_us(const RotctldClient *client)
{
    if (client == NULL || client->transport == NULL)
        return 0;
    return hamlib_transport_last_rtt_us(client->transport);
}
