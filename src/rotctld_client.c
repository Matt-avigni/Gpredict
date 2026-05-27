/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

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
#define ROTCTLD_SETPOS_NO_FEEDBACK_TIMEOUT_MS 50
#define ROTCTLD_SETPOS_RETRIES 1
#define ROTCTLD_SETPOS_RETRY_DELAY_MS 100
#define ROTCTLD_SETPOS_LOG_THROTTLE_US 1000000
#define ROTCTLD_PARSE_DUMP_LIMIT 256

struct _RotctldClient {
    HamlibTransport        *transport;
    GMutex                  meta_mutex;
    rotctld_client_state_t  state;
    gchar                  *state_reason;
    gchar                  *label;
    gint64                  last_probe_us;
    gint64                  last_failure_log_us;
    gint64                  last_parse_log_us;
    gchar                  *last_parse_class;
    gint64                  last_setpos_empty_log_us;
    gint64                  last_setpos_warn_log_us;
    gint64                  invalid_backoff_until_us;
    guint                   invalid_backoff_ms;
    gboolean                recovery_triggered;
    RotCaps                 caps;
};

static const gchar *rotctld_client_state_name(rotctld_client_state_t state)
{
    switch (state)
    {
    case ROTCTLD_CLIENT_STOPPED:
        return "STOPPED";
    case ROTCTLD_CLIENT_CONNECTING:
        return "CONNECTING";
    case ROTCTLD_CLIENT_PROBING:
        return "PROBING";
    case ROTCTLD_CLIENT_READY:
        return "READY";
    case ROTCTLD_CLIENT_DEGRADED:
        return "DEGRADED";
    default:
        return "UNKNOWN";
    }
}

static void rotctld_client_emit_wire_lines(RotctldClient *client,
                                           sat_log_level_t level,
                                           const gchar *prefix,
                                           const gchar *text)
{
    gchar **lines = NULL;

    if (client == NULL || prefix == NULL || text == NULL || *text == '\0')
        return;

    lines = g_strsplit(text, "\n", -1);
    for (gint i = 0; lines[i] != NULL; i++)
    {
        gchar *trimmed = g_strdup(lines[i]);

        g_strstrip(trimmed);
        if (*trimmed != '\0')
            sat_log_forensic(level,
                             "%s: [%s] %s",
                             prefix,
                             client->label ? client->label : "rot",
                             trimmed);
        g_free(trimmed);
    }

    g_strfreev(lines);
}

static GMutex *rotctld_client_meta_mutex(const RotctldClient *client)
{
    return (GMutex *)&client->meta_mutex;
}

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

static void rotctld_client_caps_copy_snapshot(const RotCaps *src,
                                              RotCaps *dst)
{
    if (dst == NULL)
        return;

    rotctld_client_caps_init(dst);
    if (src == NULL)
        return;

    dst->signature = g_strdup(src->signature);
    dst->model_id = src->model_id;
    dst->has_get_pos = src->has_get_pos;
    dst->has_set_pos = src->has_set_pos;
    dst->has_stop = src->has_stop;
    dst->has_park = src->has_park;
    dst->limits_valid = src->limits_valid;
    dst->az_min = src->az_min;
    dst->az_max = src->az_max;
    dst->el_min = src->el_min;
    dst->el_max = src->el_max;
    dst->south_zero = src->south_zero;
    dst->quirks = src->quirks;
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
        if (g_ascii_isdigit(*p) || *p == '-' || *p == '+' || *p == '.')
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
        if (g_ascii_isdigit(*p) || *p == '-' || *p == '+' || *p == '.')
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

static gchar **rotctld_client_split_lines(const gchar *text)
{
    if (text == NULL)
        return NULL;

    return g_strsplit_set(text, "\r\n", -1);
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

    lines = rotctld_client_split_lines(text);
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

    g_mutex_lock(rotctld_client_meta_mutex(client));
    rotctld_client_caps_clear(&client->caps);
    rotctld_parse_model(dump_state, &client->caps.model_id);
    client->caps.signature = g_strdup(dump_state);
    rotctld_client_parse_limits(&client->caps, dump_state);
    g_mutex_unlock(rotctld_client_meta_mutex(client));
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

static void rotctld_client_get_limits_snapshot(const RotctldClient *client,
                                               gint *model_id_out,
                                               rot_limits_t *limits_out)
{
    if (model_id_out)
        *model_id_out = 0;
    if (limits_out)
        *limits_out = rotctld_client_limits_from_caps(NULL);

    if (client == NULL)
        return;

    g_mutex_lock(rotctld_client_meta_mutex(client));
    if (model_id_out)
        *model_id_out = client->caps.model_id;
    if (limits_out)
        *limits_out = rotctld_client_limits_from_caps(&client->caps);
    g_mutex_unlock(rotctld_client_meta_mutex(client));
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

    gchar **lines = rotctld_client_split_lines(text);
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

static gboolean rotctld_client_reply_is_empty(const gchar *reply)
{
    if (reply == NULL)
        return TRUE;

    for (const gchar *p = reply; *p != '\0'; p++)
    {
        if (!g_ascii_isspace(*p))
            return FALSE;
    }

    return TRUE;
}

static gboolean rotctld_client_reply_has_digit(const gchar *reply)
{
    if (reply == NULL)
        return FALSE;

    for (const gchar *p = reply; *p != '\0'; p++)
    {
        if (g_ascii_isdigit(*p))
            return TRUE;
    }

    return FALSE;
}

static gboolean rotctld_client_reply_is_prompt(const gchar *reply)
{
    gchar *tmp = NULL;
    gboolean ok = FALSE;

    if (reply == NULL || *reply == '\0')
        return FALSE;

    tmp = g_strdup(reply);
    tmp = g_strstrip(tmp);

    if (g_strcmp0(tmp, "?") == 0 ||
        g_strcmp0(tmp, "?>") == 0 ||
        g_strcmp0(tmp, ">?") == 0)
        ok = TRUE;
    else if (g_str_has_prefix(tmp, "?>") || g_str_has_prefix(tmp, "?"))
        ok = TRUE;

    g_free(tmp);
    return ok;
}

static gboolean rotctld_client_reply_is_ok(const gchar *reply)
{
    gchar *tmp = NULL;
    gboolean ok = FALSE;

    if (reply == NULL || *reply == '\0')
        return FALSE;

    tmp = g_strdup(reply);
    tmp = g_strstrip(tmp);

    if (g_ascii_strcasecmp(tmp, "OK") == 0 ||
        g_ascii_strcasecmp(tmp, "0") == 0)
        ok = TRUE;

    g_free(tmp);
    return ok;
}

static gboolean rotctld_client_err_is_disconnect(gint err)
{
    return (err == EPIPE ||
            err == ECONNRESET ||
            err == ECONNABORTED ||
            err == ENOTCONN ||
            err == ETIMEDOUT);
}

static guint rotctld_client_count_lines(const gchar *text)
{
    guint lines = 0;
    const gchar *scan = text;

    if (text == NULL || *text == '\0')
        return 0;

    while (*scan != '\0')
    {
        if (*scan == '\n')
            lines++;
        scan++;
    }

    if (scan != text && *(scan - 1) != '\n')
        lines++;

    return lines;
}

static const gchar *rotctld_client_read_mode_name(hamlib_read_mode_t mode)
{
    switch (mode)
    {
        case HAMLIB_READ_SINGLE:
            return "single";
        case HAMLIB_READ_MULTILINE_RPRT:
            return "multiline_rprt";
        case HAMLIB_READ_MULTILINE_IDLE:
            return "multiline_idle";
        default:
            return "unknown";
    }
}

static const gchar *rotctld_client_term_name(hamlib_term_t term)
{
    switch (term)
    {
        case HAMLIB_TERM_RPRT:
            return "rprt";
        case HAMLIB_TERM_RPRT_OR_DONE:
            return "rprt_or_done";
        default:
            return "unknown";
    }
}

static gboolean rotctld_client_scan_rprt(const gchar *reply, gint *code_out)
{
    gchar **lines = NULL;
    gboolean found = FALSE;

    if (reply == NULL || *reply == '\0')
        return FALSE;

    lines = rotctld_client_split_lines(reply);
    for (gint i = 0; lines[i] != NULL; i++)
    {
        gint code = 0;
        if (rotctld_client_line_parse_rprt(lines[i], &code))
        {
            if (code_out)
                *code_out = code;
            found = TRUE;
            break;
        }
    }
    g_strfreev(lines);
    return found;
}

static gboolean rotctld_client_exchange(RotctldClient *client,
                                        const gchar *cmd,
                                        hamlib_read_mode_t mode,
                                        hamlib_term_t term,
                                        gint timeout_ms,
                                        gint retries,
                                        gint retry_delay_ms,
                                        gchar *out,
                                        gsize out_len,
                                        HamlibResponseInfo *info)
{
    HamlibResponseInfo local = { 0 };
    gboolean ok = FALSE;
    const gchar *send_cmd = NULL;
    gchar *tmp_cmd = NULL;
    gsize cmd_len = 0;

    if (info)
        memset(info, 0, sizeof(*info));

    if (client == NULL || cmd == NULL || client->transport == NULL)
        return FALSE;

    if (!hamlib_transport_is_ready(client->transport))
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

    rotctld_client_emit_wire_lines(client, SAT_LOG_LEVEL_INFO,
                                   "gpredict:tx", send_cmd);
    ok = hamlib_transport_request(client->transport,
                                  send_cmd,
                                  mode,
                                  term,
                                  timeout_ms,
                                  50,
                                  retries,
                                  retry_delay_ms,
                                  out,
                                  out_len,
                                  &local);
    g_free(tmp_cmd);

    if (info)
        *info = local;

    if (!ok)
    {
        if (rotctld_client_err_is_disconnect(local.err))
        {
            hamlib_transport_close(client->transport);
            rotctld_client_set_state(client, ROTCTLD_CLIENT_DEGRADED,
                                     "transport lost");
        }

        sat_log_log(SAT_LOG_LEVEL_WARN,
                    "rotctld exchange failed cmd=%s err=%d",
                    cmd, local.err);
        sat_log_forensic(SAT_LOG_LEVEL_ERROR,
                         "rotctld exchange failed [%s] cmd=%s err=%d",
                         client->label ? client->label : "rot",
                         cmd, local.err);
        return FALSE;
    }

    if (out && out_len > 0 && !local.saw_rprt)
    {
        gint code = 0;
        if (rotctld_client_scan_rprt(out, &code))
        {
            local.saw_rprt = TRUE;
            local.rprt_code = code;
            if (info)
                *info = local;
        }
    }

    if (out && out_len > 0)
    {
        rotctld_client_emit_wire_lines(client, SAT_LOG_LEVEL_INFO,
                                       "gpredict:rx", out);
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rotctld exchange ok cmd=%s lines=%u rprt=%d code=%d",
                    cmd,
                    rotctld_client_count_lines(out),
                    local.saw_rprt ? 1 : 0,
                    local.rprt_code);
    }

    return TRUE;
}

static gboolean rotctld_dump_state_first_line_is_one(const gchar *text)
{
    gchar **lines = NULL;
    gboolean ok = FALSE;

    if (text == NULL || *text == '\0')
        return FALSE;

    lines = rotctld_client_split_lines(text);
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

    lines = rotctld_client_split_lines(text);
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
    sat_log_forensic(SAT_LOG_LEVEL_ERROR,
                     "%s [%s]: cmd='%s' bytes=%" G_GSIZE_FORMAT " reply=%s",
                     label ? label : "rotctld failure",
                     (client && client->label) ? client->label : "rot",
                     cmd ? cmd : "(null)",
                     bytes,
                     snippet ? snippet : "(null)");

    g_free(snippet);
}

static void rotctld_client_build_reply_debug(const gchar *reply,
                                             gsize *len_out,
                                             gchar **printable_out,
                                             gchar **hex_out)
{
    gsize len = 0;
    gsize limit = 0;
    GString *printable = NULL;
    GString *hex = NULL;

    if (len_out)
        *len_out = 0;
    if (printable_out)
        *printable_out = NULL;
    if (hex_out)
        *hex_out = NULL;

    if (reply == NULL || *reply == '\0')
    {
        if (printable_out)
            *printable_out = g_strdup("(none)");
        if (hex_out)
            *hex_out = g_strdup("(none)");
        return;
    }

    len = strlen(reply);
    limit = MIN(len, (gsize)ROTCTLD_PARSE_DUMP_LIMIT);
    printable = g_string_sized_new(limit * 2 + 8);
    hex = g_string_sized_new(limit * 3 + 8);

    for (gsize i = 0; i < limit; i++)
    {
        unsigned char c = (unsigned char)reply[i];

        if (c == '\n')
            g_string_append(printable, "\\n");
        else if (c == '\r')
            g_string_append(printable, "\\r");
        else if (c == '\t')
            g_string_append(printable, "\\t");
        else if (g_ascii_isprint(c))
            g_string_append_c(printable, (gchar)c);
        else
            g_string_append_c(printable, '.');

        g_string_append_printf(hex, "%02X", c);
        if (i + 1 < limit)
            g_string_append_c(hex, ' ');
    }

    if (len > limit)
        g_string_append(printable, "...");

    if (len_out)
        *len_out = len;
    if (printable_out)
        *printable_out = g_string_free(printable, FALSE);
    else
        g_string_free(printable, TRUE);
    if (hex_out)
        *hex_out = g_string_free(hex, FALSE);
    else
        g_string_free(hex, TRUE);
}

static gboolean rotctld_client_rate_limit(gint64 *last_us,
                                          gint64 throttle_us)
{
    gint64 now_us = g_get_monotonic_time();

    if (last_us != NULL &&
        *last_us > 0 &&
        (now_us - *last_us) < throttle_us)
        return FALSE;

    if (last_us != NULL)
        *last_us = now_us;

    return TRUE;
}

static void rotctld_client_log_parse_debug(RotctldClient *client,
                                           const gchar *label,
                                           const gchar *cmd,
                                           const gchar *reply,
                                           const gchar *class_id)
{
    gint64 now_us = g_get_monotonic_time();
    gsize raw_len = 0;
    gchar *printable = NULL;
    gchar *hex = NULL;

    if (client != NULL &&
        client->last_parse_log_us > 0 &&
        (now_us - client->last_parse_log_us) < ROTCTLD_LOG_THROTTLE_US &&
        g_strcmp0(client->last_parse_class, class_id) == 0)
        return;

    if (client != NULL)
    {
        client->last_parse_log_us = now_us;
        g_free(client->last_parse_class);
        client->last_parse_class = g_strdup(class_id);
    }

    rotctld_client_build_reply_debug(reply, &raw_len, &printable, &hex);

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "%s: cmd=%s len=%" G_GSIZE_FORMAT " class=%s view=%s hex=%s",
                label ? label : "parse failed",
                cmd ? cmd : "(null)",
                raw_len,
                class_id ? class_id : "(none)",
                printable ? printable : "(none)",
                hex ? hex : "(none)");

    g_free(printable);
    g_free(hex);
}

static void rotctld_client_note_invalid_reply(RotctldClient *client,
                                              const gchar *label,
                                              const gchar *cmd,
                                              const gchar *reply,
                                              const gchar *class_id)
{
    gint64 now_us = g_get_monotonic_time();

    if (client == NULL)
        return;

    g_mutex_lock(rotctld_client_meta_mutex(client));
    client->recovery_triggered = TRUE;
    g_mutex_unlock(rotctld_client_meta_mutex(client));

    if (client->invalid_backoff_ms == 0)
        client->invalid_backoff_ms = 100;

    client->invalid_backoff_until_us =
        now_us + ((gint64)client->invalid_backoff_ms * 1000);
    client->invalid_backoff_ms =
        MIN(client->invalid_backoff_ms * 2, 500u);

    rotctld_client_log_parse_debug(client, label, cmd, reply, class_id);
}

static void rotctld_client_note_valid_reply(RotctldClient *client)
{
    if (client == NULL)
        return;

    client->invalid_backoff_until_us = 0;
    client->invalid_backoff_ms = 0;
}

static void rotctld_client_wait_invalid_backoff(RotctldClient *client,
                                                gint max_wait_ms)
{
    gint64 now_us = 0;
    gint64 remaining_us = 0;

    /* Backoff after invalid replies to avoid spamming retries/resync. */
    if (client == NULL)
        return;

    if (client->invalid_backoff_until_us <= 0)
        return;

    now_us = g_get_monotonic_time();
    if (now_us >= client->invalid_backoff_until_us)
        return;

    remaining_us = client->invalid_backoff_until_us - now_us;
    if (max_wait_ms > 0)
        remaining_us = MIN(remaining_us, (gint64)max_wait_ms * 1000);

    if (remaining_us > 0)
        g_usleep((gulong)remaining_us);
}

static void rotctld_client_set_state(RotctldClient *client,
                                     rotctld_client_state_t state,
                                     const gchar *fmt,
                                     ...)
{
    va_list args;
    gchar *reason = NULL;
    gchar *label = NULL;

    if (client == NULL)
        return;

    if (fmt != NULL)
    {
        va_start(args, fmt);
        reason = g_strdup_vprintf(fmt, args);
        va_end(args);
    }

    g_mutex_lock(rotctld_client_meta_mutex(client));
    label = g_strdup(client->label ? client->label : "rot");
    client->state = state;
    g_free(client->state_reason);
    client->state_reason = reason;
    g_mutex_unlock(rotctld_client_meta_mutex(client));

    sat_log_forensic(SAT_LOG_LEVEL_INFO,
                     "rotctld client (%s) state=%s reason=%s",
                     label ? label : "rot",
                     rotctld_client_state_name(state),
                     reason ? reason : "(none)");
    g_free(label);
}

RotctldClient *rotctld_client_new(const gchar *label)
{
    RotctldClient *client = g_new0(RotctldClient, 1);

    client->transport = hamlib_transport_new();
    g_mutex_init(&client->meta_mutex);
    client->state = ROTCTLD_CLIENT_STOPPED;
    client->label = g_strdup(label ? label : "rot");
    rotctld_client_caps_init(&client->caps);
    client->last_failure_log_us = 0;
    client->last_parse_log_us = 0;
    client->last_parse_class = NULL;
    client->last_setpos_empty_log_us = 0;
    client->last_setpos_warn_log_us = 0;
    client->invalid_backoff_until_us = 0;
    client->invalid_backoff_ms = 0;
    client->recovery_triggered = FALSE;

    return client;
}

void rotctld_client_free(RotctldClient **client)
{
    if (client == NULL || *client == NULL)
        return;

    rotctld_client_close(*client);
    g_mutex_lock(rotctld_client_meta_mutex(*client));
    rotctld_client_caps_clear(&(*client)->caps);
    g_free((*client)->state_reason);
    g_mutex_unlock(rotctld_client_meta_mutex(*client));
    g_free((*client)->label);
    g_free((*client)->last_parse_class);
    g_mutex_clear(&(*client)->meta_mutex);
    g_free(*client);
    *client = NULL;
}

void rotctld_client_reset(RotctldClient *client)
{
    if (client == NULL)
        return;

    g_mutex_lock(rotctld_client_meta_mutex(client));
    rotctld_client_caps_clear(&client->caps);
    client->recovery_triggered = FALSE;
    g_mutex_unlock(rotctld_client_meta_mutex(client));
    rotctld_client_set_state(client, ROTCTLD_CLIENT_STOPPED, "reset");
    client->last_probe_us = 0;
    client->last_failure_log_us = 0;
    client->last_parse_log_us = 0;
    g_free(client->last_parse_class);
    client->last_parse_class = NULL;
    client->last_setpos_empty_log_us = 0;
    client->last_setpos_warn_log_us = 0;
    client->invalid_backoff_until_us = 0;
    client->invalid_backoff_ms = 0;
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

    g_mutex_lock(rotctld_client_meta_mutex(client));
    rotctld_client_caps_clear(&client->caps);
    client->recovery_triggered = FALSE;
    g_mutex_unlock(rotctld_client_meta_mutex(client));
    client->last_probe_us = 0;
    client->invalid_backoff_until_us = 0;
    client->invalid_backoff_ms = 0;
    g_free(client->last_parse_class);
    client->last_parse_class = NULL;

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
    rotctld_client_state_t state = ROTCTLD_CLIENT_STOPPED;

    if (client == NULL)
        return ROTCTLD_CLIENT_STOPPED;

    g_mutex_lock(rotctld_client_meta_mutex(client));
    state = client->state;
    g_mutex_unlock(rotctld_client_meta_mutex(client));
    return state;
}

void rotctld_client_get_status(const RotctldClient *client,
                               rotctld_client_state_t *state_out,
                               gchar *reason_out,
                               gsize reason_len)
{
    if (state_out)
        *state_out = ROTCTLD_CLIENT_STOPPED;
    if (reason_out && reason_len > 0)
        reason_out[0] = '\0';

    if (client == NULL)
        return;

    g_mutex_lock(rotctld_client_meta_mutex(client));
    if (state_out)
        *state_out = client->state;
    if (reason_out && reason_len > 0)
        g_strlcpy(reason_out,
                  client->state_reason ? client->state_reason : "",
                  reason_len);
    g_mutex_unlock(rotctld_client_meta_mutex(client));
}

gboolean rotctld_client_get_caps_snapshot(const RotctldClient *client,
                                          RotCaps *caps_out)
{
    if (caps_out == NULL)
        return FALSE;

    rotctld_client_caps_init(caps_out);
    if (client == NULL)
        return FALSE;

    g_mutex_lock(rotctld_client_meta_mutex(client));
    rotctld_client_caps_copy_snapshot(&client->caps, caps_out);
    g_mutex_unlock(rotctld_client_meta_mutex(client));
    return TRUE;
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
    RotCaps caps_snapshot = { 0 };
    gboolean ready_cached = FALSE;
    gboolean has_get_pos = FALSE;

    if (client == NULL || client->transport == NULL)
        return FALSE;

    ready_cached = (rotctld_client_get_state(client) == ROTCTLD_CLIENT_READY);
    if (rotctld_client_get_caps_snapshot(client, &caps_snapshot))
        has_get_pos = caps_snapshot.has_get_pos;

    if (has_get_pos &&
        ready_cached &&
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
        return ready_cached;
    }

    client->last_probe_us = now_us;

    if (!rotctld_client_exchange(client,
                                 "\\dump_state\n",
                                 HAMLIB_READ_MULTILINE_IDLE,
                                 HAMLIB_TERM_RPRT_OR_DONE,
                                 timeout_ms,
                                 1,
                                 ROTCTLD_PROBE_RETRY_DELAY_MS,
                                 dump_state, sizeof(dump_state),
                                 &info))
    {
        sat_log_log(SAT_LOG_LEVEL_WARN,
                    "rotctld probe: dump_state failed");
        return FALSE;
    }

    rotctld_client_update_caps_from_dump(client, dump_state);
    rotctld_client_get_caps_snapshot(client, &caps_snapshot);
    (void)hamlib_transport_clear_rxbuf(client->transport);
    (void)hamlib_transport_drain(client->transport, 50, NULL);
    sat_log_log(SAT_LOG_LEVEL_INFO,
                "rotctld dump_state caps: model=%d az=%.2f..%.2f el=%.2f..%.2f south_zero=%d limits=%d",
                caps_snapshot.model_id,
                caps_snapshot.az_min, caps_snapshot.az_max,
                caps_snapshot.el_min, caps_snapshot.el_max,
                caps_snapshot.south_zero ? 1 : 0,
                caps_snapshot.limits_valid ? 1 : 0);

    for (gint attempt = 0; attempt < 3; attempt++)
    {
        if (rotctld_client_get_pos(client, &az, &el))
        {
            g_mutex_lock(rotctld_client_meta_mutex(client));
            client->caps.has_get_pos = TRUE;
            g_mutex_unlock(rotctld_client_meta_mutex(client));
            has_get_pos = TRUE;
            break;
        }

        (void)hamlib_transport_drain(client->transport, 50, NULL);
        g_usleep((gulong)ROTCTLD_PROBE_RETRY_DELAY_MS * 1000);
    }

    if (has_get_pos)
    {
        if (rotctld_client_set_pos(client, az, el))
        {
            g_mutex_lock(rotctld_client_meta_mutex(client));
            client->caps.has_set_pos = TRUE;
            g_mutex_unlock(rotctld_client_meta_mutex(client));
        }
    }
    else
    {
        g_mutex_lock(rotctld_client_meta_mutex(client));
        rotctld_client_caps_clear(&client->caps);
        g_mutex_unlock(rotctld_client_meta_mutex(client));
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

    ok = rotctld_client_exchange(client,
                                 "\\dump_state\n",
                                 HAMLIB_READ_MULTILINE_IDLE,
                                 HAMLIB_TERM_RPRT_OR_DONE,
                                 timeout_ms,
                                 1,
                                 ROTCTLD_PROBE_RETRY_DELAY_MS,
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
    {
        g_mutex_lock(rotctld_client_meta_mutex(client));
        client->caps.has_get_pos = TRUE;
        g_mutex_unlock(rotctld_client_meta_mutex(client));
    }

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
    gint max_retries = retries;
    gboolean prompt_retry_used = FALSE;

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
    max_retries = retries;

    rotctld_client_wait_invalid_backoff(client, retry_delay_ms);

    for (gint attempt = 0; attempt <= max_retries; attempt++)
    {
        gboolean saw_rprt = FALSE;
        gint rprt_code = 0;
        gboolean parsed = FALSE;
        gboolean invalid = FALSE;
        gboolean prompt_reply = FALSE;
        const gchar *class_id = "parse";

        memset(&local, 0, sizeof(local));
        if (reply && reply_cap > 0)
            reply[0] = '\0';

        ok = rotctld_client_exchange(client,
                                     "p\n",
                                     HAMLIB_READ_MULTILINE_RPRT,
                                     HAMLIB_TERM_RPRT_OR_DONE,
                                     timeout_ms,
                                     0,
                                     0,
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
            rotctld_client_note_valid_reply(client);
            return ROTCTLD_POS_RPRT_ERR;
        }

        if (parsed)
        {
            gint model_id = 0;
            rot_limits_t limits;

            rotctld_client_get_limits_snapshot(client, &model_id, &limits);
            if (rotctld_client_is_gs232b_model(model_id))
            {
                if (!normalize_gs232b_pos(az_out, el_out, &limits))
                    return ROTCTLD_POS_PARSE_FAIL;
            }
            if (info)
                *info = local;
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rotctld get_pos parsed az=%.2f el=%.2f rprt=%d code=%d",
                        az_out ? *az_out : 0.0,
                        el_out ? *el_out : 0.0,
                        local.saw_rprt ? 1 : 0,
                        local.rprt_code);
            rotctld_client_note_valid_reply(client);
            return ROTCTLD_POS_OK;
        }

        if (rotctld_client_reply_is_empty(reply))
        {
            invalid = TRUE;
            class_id = "empty";
        }
        else if (rotctld_client_reply_is_prompt(reply))
        {
            invalid = TRUE;
            class_id = "prompt";
            prompt_reply = TRUE;
        }
        else if (!rotctld_client_reply_has_digit(reply) &&
                 g_strrstr(reply, "RPRT") == NULL)
        {
            invalid = TRUE;
            class_id = "non_numeric";
        }

        if (invalid)
            rotctld_client_note_invalid_reply(client,
                                              "get_position invalid reply",
                                              "p",
                                              reply,
                                              class_id);
        else
            rotctld_client_log_parse_debug(client,
                                           "get_position parse failed",
                                           "p",
                                           reply,
                                           class_id);

        if (prompt_reply && !prompt_retry_used && attempt < max_retries)
        {
            prompt_retry_used = TRUE;
            max_retries = MIN(max_retries, attempt + 1);
        }

        if (attempt < max_retries)
        {
            (void)hamlib_transport_clear_rxbuf(client->transport);
            (void)hamlib_transport_drain(client->transport, 50, NULL);
            if (retry_delay_ms > 0)
            {
                if (invalid)
                    rotctld_client_wait_invalid_backoff(client,
                                                        prompt_reply ? 0
                                                                     : retry_delay_ms);
                else
                    g_usleep((gulong)retry_delay_ms * 1000);
            }
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

rotctld_pos_result_t rotctld_client_get_position_timed(RotctldClient *client,
                                                       gint timeout_ms,
                                                       gint retries,
                                                       gint retry_delay_ms,
                                                       gdouble *az_out,
                                                       gdouble *el_out,
                                                       gint *rprt_out)
{
    HamlibResponseInfo info = { 0 };
    gchar reply[256];
    gint attempts = retries;
    gboolean prompt_retry_used = FALSE;

    if (az_out)
        *az_out = 0.0;
    if (el_out)
        *el_out = 0.0;
    if (rprt_out)
        *rprt_out = 0;

    if (client == NULL || client->transport == NULL ||
        !hamlib_transport_is_ready(client->transport))
        return ROTCTLD_POS_IO_ERR;

    if (timeout_ms <= 0)
        timeout_ms = ROTCTLD_GETPOS_TIMEOUT_MS;
    if (attempts <= 0)
        attempts = 1;
    if (retry_delay_ms < 0)
        retry_delay_ms = 0;

    for (gint attempt = 0; attempt < attempts; attempt++)
    {
        gboolean saw_rprt = FALSE;
        gint rprt_code = 0;
        gboolean parsed = FALSE;
        gboolean ok = FALSE;
        gboolean invalid = FALSE;
        gboolean prompt_reply = FALSE;
        const gchar *class_id = "parse";

        memset(&info, 0, sizeof(info));
        reply[0] = '\0';

        ok = rotctld_client_exchange(client,
                                     "p\n",
                                     HAMLIB_READ_MULTILINE_RPRT,
                                     HAMLIB_TERM_RPRT_OR_DONE,
                                     timeout_ms,
                                     0,
                                     0,
                                     reply, sizeof(reply),
                                     &info);
        if (!ok)
        {
            gboolean is_timeout =
                (info.err == EAGAIN ||
                 info.err == EWOULDBLOCK ||
                 info.err == ETIMEDOUT);

            if (attempt + 1 < attempts && is_timeout)
            {
                (void)hamlib_transport_clear_rxbuf(client->transport);
                (void)hamlib_transport_drain(client->transport, 50, NULL);
                if (retry_delay_ms > 0)
                    g_usleep((gulong) retry_delay_ms * 1000);
                continue;
            }
            return is_timeout ? ROTCTLD_POS_TIMEOUT : ROTCTLD_POS_IO_ERR;
        }

        parsed = rotctld_client_parse_pos(reply, az_out, el_out,
                                          &saw_rprt, &rprt_code);
        if (info.saw_rprt)
        {
            saw_rprt = TRUE;
            rprt_code = info.rprt_code;
        }

        if (saw_rprt && rprt_code != 0)
        {
            if (rprt_out)
                *rprt_out = rprt_code;

            if (rprt_code == -5)
            {
                if (attempt + 1 < attempts)
                {
                    (void)hamlib_transport_clear_rxbuf(client->transport);
                    (void)hamlib_transport_drain(client->transport, 50, NULL);
                    if (retry_delay_ms > 0)
                        g_usleep((gulong) retry_delay_ms * 1000);
                    continue;
                }
                return ROTCTLD_POS_TIMEOUT;
            }
            rotctld_client_note_valid_reply(client);
            return ROTCTLD_POS_RPRT_ERR;
        }

        if (parsed)
        {
            gint model_id = 0;
            rot_limits_t limits;

            rotctld_client_get_limits_snapshot(client, &model_id, &limits);
            if (rotctld_client_is_gs232b_model(model_id))
            {
                if (!normalize_gs232b_pos(az_out, el_out, &limits))
                    return ROTCTLD_POS_PARSE_FAIL;
            }
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rotctld get_pos parsed az=%.2f el=%.2f rprt=%d code=%d",
                        az_out ? *az_out : 0.0,
                        el_out ? *el_out : 0.0,
                        info.saw_rprt ? 1 : 0,
                        info.rprt_code);
            rotctld_client_note_valid_reply(client);
            return ROTCTLD_POS_OK;
        }

        if (rotctld_client_reply_is_empty(reply))
        {
            invalid = TRUE;
            class_id = "empty";
        }
        else if (rotctld_client_reply_is_prompt(reply))
        {
            invalid = TRUE;
            class_id = "prompt";
            prompt_reply = TRUE;
        }
        else if (!rotctld_client_reply_has_digit(reply) &&
                 g_strrstr(reply, "RPRT") == NULL)
        {
            invalid = TRUE;
            class_id = "non_numeric";
        }

        if (invalid)
            rotctld_client_note_invalid_reply(client,
                                              "get_position invalid reply",
                                              "p",
                                              reply,
                                              class_id);
        else
            rotctld_client_log_parse_debug(client,
                                           "get_position parse failed",
                                           "p",
                                           reply,
                                           class_id);

        if (prompt_reply && !prompt_retry_used && attempt + 1 < attempts)
        {
            prompt_retry_used = TRUE;
            attempts = MIN(attempts, attempt + 2);
        }

        if (attempt + 1 < attempts)
        {
            (void)hamlib_transport_clear_rxbuf(client->transport);
            (void)hamlib_transport_drain(client->transport, 50, NULL);
            if (retry_delay_ms > 0)
            {
                if (invalid)
                    rotctld_client_wait_invalid_backoff(client,
                                                        prompt_reply ? 0
                                                                     : retry_delay_ms);
                else
                    g_usleep((gulong) retry_delay_ms * 1000);
            }
            continue;
        }

        rotctld_client_log_failure(client,
                                   "get_position parse failed (expected az/el)",
                                   "p",
                                   reply);
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
    gchar azbuf[G_ASCII_DTOSTR_BUF_SIZE];
    gchar elbuf[G_ASCII_DTOSTR_BUF_SIZE];
    gchar reply_local[128];
    gchar *reply = reply_local;
    gsize reply_cap = sizeof(reply_local);
    HamlibResponseInfo local = { 0 };
    gboolean ok = FALSE;
    gint retry_delay_ms = ROTCTLD_SETPOS_RETRY_DELAY_MS;
    hamlib_read_mode_t mode = HAMLIB_READ_SINGLE;
    hamlib_term_t term = HAMLIB_TERM_RPRT;
    gchar cmd_log[96];

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

    rotctld_client_wait_invalid_backoff(client, retry_delay_ms);

    g_ascii_formatd(azbuf, sizeof(azbuf), "%.2f", az);
    g_ascii_formatd(elbuf, sizeof(elbuf), "%.2f", el);
    g_warn_if_fail(strchr(azbuf, ',') == NULL && strchr(elbuf, ',') == NULL);
    g_snprintf(cmd, sizeof(cmd), "P %s %s\n", azbuf, elbuf);
    g_snprintf(cmd_log, sizeof(cmd_log), "P %s %s", azbuf, elbuf);

    for (gint attempt = 0; attempt <= ROTCTLD_SETPOS_RETRIES; attempt++)
    {
        gboolean invalid = FALSE;
        gint parsed_rprt = 0;
        const gchar *class_id = "setpos";

        memset(&local, 0, sizeof(local));
        if (reply && reply_cap > 0)
            reply[0] = '\0';

        ok = rotctld_client_exchange(client,
                                     cmd,
                                     mode,
                                     term,
                                     ROTCTLD_SETPOS_TIMEOUT_MS,
                                     0,
                                     0,
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

        if (local.saw_rprt)
        {
            if (rprt_code_out)
                *rprt_code_out = local.rprt_code;

            rotctld_client_note_valid_reply(client);
            if (local.rprt_code != 0)
                return FALSE;
            if (rotctld_client_reply_is_empty(reply) &&
                client != NULL &&
                rotctld_client_rate_limit(&client->last_setpos_empty_log_us,
                                          ROTCTLD_SETPOS_LOG_THROTTLE_US))
            {
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            "set_position empty reply: cmd=%s mode=%s term=%s bytes=%" G_GSIZE_FORMAT,
                            cmd_log,
                            rotctld_client_read_mode_name(mode),
                            rotctld_client_term_name(term),
                            local.bytes);
            }
            return TRUE;
        }

        if (rotctld_client_reply_is_empty(reply))
        {
            if (client != NULL &&
                rotctld_client_rate_limit(&client->last_setpos_warn_log_us,
                                          ROTCTLD_SETPOS_LOG_THROTTLE_US))
            {
                sat_log_log(SAT_LOG_LEVEL_WARN,
                            "set_position empty reply without RPRT: cmd=%s mode=%s term=%s bytes=%" G_GSIZE_FORMAT,
                            cmd_log,
                            rotctld_client_read_mode_name(mode),
                            rotctld_client_term_name(term),
                            local.bytes);
            }
            invalid = TRUE;
            class_id = "empty";
        }
        else if (rotctld_client_reply_is_prompt(reply))
        {
            invalid = TRUE;
            class_id = "prompt";
        }
        else if (rotctld_client_reply_is_ok(reply))
        {
            if (rprt_code_out)
                *rprt_code_out = 0;
            rotctld_client_note_valid_reply(client);
            return TRUE;
        }
        else if (rotctld_client_line_parse_rprt(reply, &parsed_rprt))
        {
            if (rprt_code_out)
                *rprt_code_out = parsed_rprt;
            rotctld_client_note_valid_reply(client);
            if (parsed_rprt != 0)
                return FALSE;
            return TRUE;
        }
        else
        {
            invalid = TRUE;
            class_id = "non_numeric";
        }

        if (invalid)
            rotctld_client_note_invalid_reply(client,
                                              "set_position invalid reply",
                                              "P",
                                              reply,
                                              class_id);

        if (attempt < ROTCTLD_SETPOS_RETRIES)
        {
            (void)hamlib_transport_clear_rxbuf(client->transport);
            (void)hamlib_transport_drain(client->transport, 50, NULL);
            if (retry_delay_ms > 0)
            {
                if (invalid)
                    rotctld_client_wait_invalid_backoff(client, retry_delay_ms);
                else
                    g_usleep((gulong)retry_delay_ms * 1000);
            }
            retry_delay_ms = MIN(retry_delay_ms * 2, 500);
            continue;
        }

        rotctld_client_log_failure(client,
                                   "set_position protocol error (expected RPRT/OK)",
                                   "P",
                                   reply);
        return FALSE;
    }

    return FALSE;
}

gboolean rotctld_client_set_position_checked(RotctldClient *client,
                                             gdouble az,
                                             gdouble el,
                                             gint *rprt_code_out,
                                             HamlibResponseInfo *info_out,
                                             gchar *reply_out,
                                             gsize reply_len)
{
    return rotctld_client_set_pos_ex(client, az, el,
                                     rprt_code_out,
                                     info_out,
                                     reply_out,
                                     reply_len);
}

gboolean rotctld_client_set_position_no_feedback(RotctldClient *client,
                                                 gdouble az,
                                                 gdouble el,
                                                 gint *rprt_code_out,
                                                 HamlibResponseInfo *info_out,
                                                 gchar *reply_out,
                                                 gsize reply_len)
{
    gchar cmd[96];
    gchar azbuf[G_ASCII_DTOSTR_BUF_SIZE];
    gchar elbuf[G_ASCII_DTOSTR_BUF_SIZE];
    gchar reply_local[128];
    gchar *reply = reply_local;
    gsize reply_cap = sizeof(reply_local);
    HamlibResponseInfo local = { 0 };
    gboolean ok = FALSE;
    gint parsed_rprt = 0;

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

    g_ascii_formatd(azbuf, sizeof(azbuf), "%.2f", az);
    g_ascii_formatd(elbuf, sizeof(elbuf), "%.2f", el);
    g_warn_if_fail(strchr(azbuf, ',') == NULL && strchr(elbuf, ',') == NULL);
    g_snprintf(cmd, sizeof(cmd), "P %s %s\n", azbuf, elbuf);

    rotctld_client_emit_wire_lines(client, SAT_LOG_LEVEL_INFO,
                                   "gpredict:tx", cmd);
    ok = hamlib_transport_request(client->transport,
                                  cmd,
                                  HAMLIB_READ_SINGLE,
                                  HAMLIB_TERM_RPRT,
                                  ROTCTLD_SETPOS_NO_FEEDBACK_TIMEOUT_MS,
                                  10,
                                  0,
                                  0,
                                  reply,
                                  reply_cap,
                                  &local);

    if (info_out)
        *info_out = local;

    if (ok)
    {
        if (reply && reply_cap > 0)
            rotctld_client_emit_wire_lines(client, SAT_LOG_LEVEL_INFO,
                                           "gpredict:rx", reply);

        if (local.saw_rprt)
        {
            if (rprt_code_out)
                *rprt_code_out = local.rprt_code;
            rotctld_client_note_valid_reply(client);
            return local.rprt_code == 0;
        }

        if (rotctld_client_reply_is_ok(reply))
        {
            rotctld_client_note_valid_reply(client);
            return TRUE;
        }

        if (rotctld_client_line_parse_rprt(reply, &parsed_rprt))
        {
            local.saw_rprt = TRUE;
            local.rprt_code = parsed_rprt;
            if (info_out)
                *info_out = local;
            if (rprt_code_out)
                *rprt_code_out = parsed_rprt;
            rotctld_client_note_valid_reply(client);
            return parsed_rprt == 0;
        }

        return TRUE;
    }

    if (local.err == EAGAIN ||
        local.err == EWOULDBLOCK ||
        local.err == ETIMEDOUT)
    {
        return TRUE;
    }

    if (rotctld_client_err_is_disconnect(local.err))
    {
        hamlib_transport_close(client->transport);
        rotctld_client_set_state(client, ROTCTLD_CLIENT_DEGRADED,
                                 "transport lost");
    }

    return FALSE;
}

gboolean rotctld_client_set_pos(RotctldClient *client,
                                gdouble az,
                                gdouble el)
{
    return rotctld_client_set_position_checked(client, az, el,
                                               NULL, NULL, NULL, 0);
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
    gboolean ok = FALSE;

    if (info)
        memset(info, 0, sizeof(*info));

    if (client == NULL || cmd == NULL)
        return FALSE;

    if (g_str_has_prefix(cmd, "\\dump_state"))
        mode = HAMLIB_READ_MULTILINE_IDLE;

    ok = rotctld_client_exchange(client,
                                 cmd,
                                 mode,
                                 HAMLIB_TERM_RPRT_OR_DONE,
                                 1000,
                                 0,
                                 0,
                                 out, out_len,
                                 info);
    return ok;
}

gboolean rotctld_client_request_raw_timeout(RotctldClient *client,
                                            const gchar *cmd,
                                            gint timeout_ms,
                                            gchar *out,
                                            gsize out_len,
                                            HamlibResponseInfo *info)
{
    hamlib_read_mode_t mode = HAMLIB_READ_MULTILINE_RPRT;
    gboolean ok = FALSE;

    if (info)
        memset(info, 0, sizeof(*info));

    if (client == NULL || cmd == NULL)
        return FALSE;

    if (timeout_ms <= 0)
        timeout_ms = 1000;

    if (g_str_has_prefix(cmd, "\\dump_state"))
        mode = HAMLIB_READ_MULTILINE_IDLE;

    ok = rotctld_client_exchange(client,
                                 cmd,
                                 mode,
                                 HAMLIB_TERM_RPRT_OR_DONE,
                                 timeout_ms,
                                 0,
                                 0,
                                 out, out_len,
                                 info);
    return ok;
}

gint64 rotctld_client_last_rtt_us(const RotctldClient *client)
{
    if (client == NULL || client->transport == NULL)
        return 0;
    return hamlib_transport_last_rtt_us(client->transport);
}

gboolean rotctld_client_recovery_triggered(const RotctldClient *client)
{
    gboolean recovery = FALSE;

    if (client == NULL)
        return FALSE;

    g_mutex_lock(rotctld_client_meta_mutex(client));
    recovery = client->recovery_triggered;
    g_mutex_unlock(rotctld_client_meta_mutex(client));
    return recovery;
}
