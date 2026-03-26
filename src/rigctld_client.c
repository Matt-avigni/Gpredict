/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include "rigctld_client.h"

#include <errno.h>
#include <stdarg.h>
#include <string.h>

#include "rotctld-parse.h"
#include "sat-log.h"

#define RIGCTLD_PROBE_RETRY_DELAY_MS 50
#define RIGCTLD_VFO_TOKEN_MAX 64

static gint rig_log_level = RIG_LOG_QUIET;

struct _RigctldClient {
    HamlibTransport       *transport;
    GRecMutex              meta_lock;
    rigctld_client_state_t state;
    gchar                 *state_reason;
    gchar                 *label;
    gint64                 last_probe_us;
    vfo_t                  last_selected_vfo;
    RigCaps                caps;
};

static GRecMutex *rigctld_client_meta_lock(const RigctldClient *client)
{
    return (GRecMutex *)&client->meta_lock;
}

typedef struct {
    const gchar *match;
    guint        quirks;
} RigQuirkEntry;

static const RigQuirkEntry rig_quirks[] = {
    { "ic-9700", RIG_QUIRK_FORCE_MAIN_SUB },
    { "3081", RIG_QUIRK_FORCE_MAIN_SUB },
    { NULL, 0 }
};

void rigctld_client_set_log_level(rig_log_level_t level)
{
    if (level < RIG_LOG_QUIET)
        level = RIG_LOG_QUIET;
    if (level > RIG_LOG_TRACE)
        level = RIG_LOG_TRACE;

    g_atomic_int_set(&rig_log_level, (gint)level);
}

rig_log_level_t rigctld_client_get_log_level(void)
{
    return (rig_log_level_t)g_atomic_int_get(&rig_log_level);
}

static void rigctld_client_set_state(RigctldClient *client,
                                     rigctld_client_state_t state,
                                     const gchar *fmt,
                                     ...) G_GNUC_PRINTF(3, 4);

static void rigctld_client_caps_clear(RigCaps *caps)
{
    if (caps == NULL)
        return;

    g_free(caps->signature);
    g_free(caps->backend_version);
    g_free(caps->default_vfo_token);
    g_free(caps->vfo_token_main);
    g_free(caps->vfo_token_sub);
    caps->signature = NULL;
    caps->backend_version = NULL;
    caps->default_vfo_token = NULL;
    caps->vfo_token_main = NULL;
    caps->vfo_token_sub = NULL;

    if (caps->vfo_candidates)
        g_ptr_array_set_size(caps->vfo_candidates, 0);
    if (caps->vfo_working)
        g_hash_table_remove_all(caps->vfo_working);

    caps->rig_model = 0;
    caps->has_get_freq = FALSE;
    caps->has_set_freq = FALSE;
    caps->has_get_vfo = FALSE;
    caps->has_set_vfo = FALSE;
    caps->has_set_vfo_opt = FALSE;
    caps->vfo_opt_enabled = FALSE;
    caps->vfo_opt_unsafe = FALSE;
    caps->prefer_main_sub_tokens = FALSE;
    caps->strategy = RIG_STRATEGY_PLAIN_FREQ;
    caps->quirks = RIG_QUIRK_NONE;
}

static void rigctld_client_caps_init(RigCaps *caps)
{
    if (caps == NULL)
        return;

    memset(caps, 0, sizeof(*caps));
    caps->vfo_candidates = g_ptr_array_new_with_free_func(g_free);
    caps->vfo_working = g_hash_table_new_full(g_str_hash, g_str_equal,
                                              g_free, NULL);
    caps->strategy = RIG_STRATEGY_PLAIN_FREQ;
}

static void rigctld_client_caps_release(RigCaps *caps)
{
    if (caps == NULL)
        return;

    rigctld_client_caps_clear(caps);
    if (caps->vfo_candidates)
        g_ptr_array_free(caps->vfo_candidates, TRUE);
    if (caps->vfo_working)
        g_hash_table_destroy(caps->vfo_working);
    caps->vfo_candidates = NULL;
    caps->vfo_working = NULL;
}

static void rigctld_client_caps_copy_snapshot(const RigCaps *src,
                                              RigCaps *dst)
{
    if (dst == NULL)
        return;

    rigctld_client_caps_init(dst);
    if (src == NULL)
        return;

    dst->signature = g_strdup(src->signature);
    dst->backend_version = g_strdup(src->backend_version);
    dst->rig_model = src->rig_model;
    dst->has_get_freq = src->has_get_freq;
    dst->has_set_freq = src->has_set_freq;
    dst->has_get_vfo = src->has_get_vfo;
    dst->has_set_vfo = src->has_set_vfo;
    dst->has_set_vfo_opt = src->has_set_vfo_opt;
    dst->vfo_opt_enabled = src->vfo_opt_enabled;
    dst->vfo_opt_unsafe = src->vfo_opt_unsafe;
    dst->prefer_main_sub_tokens = src->prefer_main_sub_tokens;
    dst->strategy = src->strategy;
    dst->default_vfo_token = g_strdup(src->default_vfo_token);
    dst->vfo_token_main = g_strdup(src->vfo_token_main);
    dst->vfo_token_sub = g_strdup(src->vfo_token_sub);
    dst->quirks = src->quirks;

    if (src->vfo_candidates != NULL)
    {
        for (guint i = 0; i < src->vfo_candidates->len; i++)
        {
            const gchar *token = g_ptr_array_index(src->vfo_candidates, i);

            if (token != NULL)
                g_ptr_array_add(dst->vfo_candidates, g_strdup(token));
        }
    }

    if (src->vfo_working != NULL)
    {
        GHashTableIter iter;
        gpointer key = NULL;
        gpointer value = NULL;

        g_hash_table_iter_init(&iter, src->vfo_working);
        while (g_hash_table_iter_next(&iter, &key, &value))
        {
            const gchar *token = key;

            if (token != NULL)
                g_hash_table_replace(dst->vfo_working, g_strdup(token), value);
        }
    }
}

static void rigctld_client_apply_quirks(RigCaps *caps)
{
    gchar *sig = NULL;

    if (caps == NULL || caps->signature == NULL)
        return;

    sig = g_ascii_strdown(caps->signature, -1);
    for (gint i = 0; rig_quirks[i].match != NULL; i++)
    {
        if (g_strrstr(sig, rig_quirks[i].match) != NULL)
            caps->quirks |= rig_quirks[i].quirks;
    }
    g_free(sig);
}

static void rigctld_client_extract_first_lines(const gchar *text,
                                               gchar **line1_out,
                                               gchar **line2_out)
{
    gchar *line1 = NULL;
    gchar *line2 = NULL;

    if (line1_out)
        *line1_out = NULL;
    if (line2_out)
        *line2_out = NULL;

    if (text == NULL || *text == '\0')
        return;

    gchar **lines = g_strsplit(text, "\n", -1);
    for (gint i = 0; lines[i] != NULL; i++)
    {
        gchar *line = g_strstrip(lines[i]);

        if (line[0] == '\0')
            continue;

        if (line1 == NULL)
        {
            line1 = g_strdup(line);
            continue;
        }
        if (line2 == NULL)
        {
            line2 = g_strdup(line);
            break;
        }
    }
    g_strfreev(lines);

    if (line1_out)
        *line1_out = line1;
    else
        g_free(line1);

    if (line2_out)
        *line2_out = line2;
    else
        g_free(line2);
}

static gboolean rigctld_dump_state_line1_ok(const gchar *text)
{
    gchar *line1 = NULL;
    gboolean ok = FALSE;

    if (text == NULL || *text == '\0')
        return FALSE;

    rigctld_client_extract_first_lines(text, &line1, NULL);
    if (line1 != NULL && g_str_has_prefix(line1, "1"))
        ok = TRUE;
    g_free(line1);

    return ok;
}

static gchar *rigctld_client_build_signature(const gchar *text,
                                             gint rig_model,
                                             const gchar *backend_version)
{
    gchar *line1 = NULL;
    gchar *line2 = NULL;
    gchar *signature = NULL;

    rigctld_client_extract_first_lines(text, &line1, &line2);
    signature = g_strdup_printf("model=%d line1=%s line2=%s backend=%s",
                                rig_model,
                                line1 ? line1 : "(none)",
                                line2 ? line2 : "(none)",
                                backend_version ? backend_version : "(none)");
    g_free(line1);
    g_free(line2);
    return signature;
}

static gboolean rigctld_client_parse_frequency(const gchar *text,
                                               gint64 *freq_out)
{
    const gchar *p = text;
    gchar *endptr = NULL;
    gint64 value = 0;

    if (freq_out)
        *freq_out = 0;

    if (text == NULL || *text == '\0')
        return FALSE;

    while (*p != '\0')
    {
        if (g_ascii_isdigit(*p) || *p == '-' || *p == '+')
        {
            value = g_ascii_strtoll(p, &endptr, 10);
            if (endptr != p)
            {
                if (freq_out)
                    *freq_out = value;
                return TRUE;
            }
        }
        p++;
    }

    return FALSE;
}

static gint rigctld_client_expected_model(const radio_conf_t *conf)
{
    gint model = 0;

    if (conf == NULL)
        return 0;

    model = conf->rigctld_model;
    if (model <= 0)
        model = radio_model_to_hamlib_model(conf->radio_model);

    return model;
}

static void rigctld_client_add_vfo_candidate(RigCaps *caps,
                                             const gchar *token)
{
    gchar *trimmed = NULL;

    if (caps == NULL || token == NULL || *token == '\0')
        return;

    trimmed = g_strdup(token);
    g_strstrip(trimmed);
    if (*trimmed == '\0')
    {
        g_free(trimmed);
        return;
    }

    for (guint i = 0; i < caps->vfo_candidates->len; i++)
    {
        const gchar *existing = g_ptr_array_index(caps->vfo_candidates, i);
        if (g_strcmp0(existing, trimmed) == 0)
        {
            g_free(trimmed);
            return;
        }
    }

    g_ptr_array_add(caps->vfo_candidates, trimmed);
}

static void rigctld_client_parse_vfo_list(RigCaps *caps, const gchar *line)
{
    const gchar *sep = NULL;
    gchar *list = NULL;
    gchar **parts = NULL;

    if (caps == NULL || line == NULL)
        return;

    sep = strchr(line, '=');
    if (sep == NULL)
        sep = strchr(line, ':');
    if (sep == NULL || *(sep + 1) == '\0')
        return;

    list = g_strdup(sep + 1);
    g_strstrip(list);
    parts = g_strsplit_set(list, " \t,", -1);
    for (gint i = 0; parts[i] != NULL; i++)
    {
        if (parts[i][0] == '\0')
            continue;
        rigctld_client_add_vfo_candidate(caps, parts[i]);
    }

    g_strfreev(parts);
    g_free(list);
}

static void rigctld_client_parse_dump_state(RigCaps *caps, const gchar *text)
{
    gchar **lines = NULL;

    if (caps == NULL || text == NULL || *text == '\0')
        return;

    (void) parse_dump_state_model_id(text, &caps->rig_model);

    lines = g_strsplit(text, "\n", -1);
    for (gint i = 0; lines[i] != NULL; i++)
    {
        gchar *line = g_strstrip(lines[i]);
        gchar *lower = NULL;

        if (line[0] == '\0')
            continue;

        lower = g_ascii_strdown(line, -1);
        if (g_str_has_prefix(lower, "has_get_vfo"))
            caps->has_get_vfo = g_strrstr(lower, "1") != NULL;
        if (g_str_has_prefix(lower, "has_set_vfo"))
            caps->has_set_vfo = g_strrstr(lower, "1") != NULL;
        if (g_str_has_prefix(lower, "has_set_vfo_opt"))
            caps->has_set_vfo_opt = g_strrstr(lower, "1") != NULL;

        if (g_strrstr(lower, "vfo list") != NULL ||
            g_strrstr(lower, "vfo_list") != NULL)
            rigctld_client_parse_vfo_list(caps, line);

        if (caps->backend_version == NULL &&
            (g_strrstr(lower, "hamlib") != NULL ||
             g_strrstr(lower, "rigctld") != NULL ||
             g_strrstr(lower, "backend") != NULL))
        {
            caps->backend_version = g_strdup(line);
        }

        g_free(lower);
    }
    g_strfreev(lines);
}

static gboolean rigctld_client_try_get_freq(RigctldClient *client,
                                            const gchar *cmd,
                                            gint64 *freq_out,
                                            gchar *reply,
                                            gsize reply_len,
                                            gint timeout_ms)
{
    HamlibResponseInfo info = { 0 };
    gboolean ok = FALSE;

    if (freq_out)
        *freq_out = 0;
    if (reply && reply_len > 0)
        reply[0] = '\0';

    ok = hamlib_transport_request(client->transport,
                                  cmd,
                                  HAMLIB_READ_MULTILINE_RPRT,
                                  HAMLIB_TERM_RPRT,
                                  timeout_ms,
                                  50,
                                  0, 0,
                                  reply, reply_len,
                                  &info);
    if (!ok)
        return FALSE;

    if (info.saw_rprt && info.rprt_code != 0)
        return FALSE;

    return rigctld_client_parse_frequency(reply, freq_out);
}

static gboolean rigctld_client_try_get_freq_retry(RigctldClient *client,
                                                  const gchar *cmd,
                                                  gint64 *freq_out,
                                                  gchar *reply,
                                                  gsize reply_len,
                                                  gint timeout_ms,
                                                  gint retries)
{
    gint attempt = 0;

    for (attempt = 0; attempt <= retries; attempt++)
    {
        if (rigctld_client_try_get_freq(client, cmd, freq_out,
                                        reply, reply_len, timeout_ms))
            return TRUE;

        (void)hamlib_transport_drain(client->transport, 50, NULL);
        if (attempt < retries)
            g_usleep((gulong)RIGCTLD_PROBE_RETRY_DELAY_MS * 1000);
    }

    return FALSE;
}

static gboolean rigctld_client_try_set_ok(RigctldClient *client,
                                          const gchar *cmd,
                                          gchar *reply,
                                          gsize reply_len,
                                          gint timeout_ms)
{
    HamlibResponseInfo info = { 0 };
    gboolean ok = FALSE;

    if (reply && reply_len > 0)
        reply[0] = '\0';

    ok = hamlib_transport_request(client->transport,
                                  cmd,
                                  HAMLIB_READ_MULTILINE_RPRT,
                                  HAMLIB_TERM_RPRT,
                                  timeout_ms,
                                  50,
                                  0, 0,
                                  reply, reply_len,
                                  &info);
    if (!ok)
        return FALSE;

    if (info.saw_rprt && info.rprt_code != 0)
        return FALSE;

    return TRUE;
}

static gboolean rigctld_client_try_select_vfo(RigctldClient *client,
                                              const gchar *token,
                                              gint timeout_ms)
{
    gchar cmd[RIGCTLD_VFO_TOKEN_MAX];
    gchar reply[128];

    if (token == NULL || *token == '\0')
        return FALSE;

    g_snprintf(cmd, sizeof(cmd), "V %s\x0a", token);
    return rigctld_client_try_set_ok(client, cmd, reply, sizeof(reply),
                                     timeout_ms);
}

static const gchar *rigctld_client_vfo_token(RigctldClient *client,
                                             vfo_t vfo)
{
    static const gchar *main_candidates_default[] =
        { "VFOA", "Main", "MainA", "VFO_MAIN", NULL };
    static const gchar *sub_candidates_default[] =
        { "VFOB", "Sub", "SubA", "VFO_SUB", NULL };
    static const gchar *main_candidates_prefer[] =
        { "Main", "MainA", "VFO_MAIN", "VFOA", NULL };
    static const gchar *sub_candidates_prefer[] =
        { "Sub", "SubA", "VFO_SUB", "VFOB", NULL };
    static const gchar *main_candidates_strict[] =
        { "Main", "MainA", "VFO_MAIN", NULL };
    static const gchar *sub_candidates_strict[] =
        { "Sub", "SubA", "VFO_SUB", NULL };
    const gchar *fallback = (vfo == VFO_SUB) ? "Sub" : "Main";
    const gchar * const *candidates = NULL;
    RigCaps *caps = NULL;

    if (client == NULL)
        return fallback;

    caps = &client->caps;

    if (vfo != VFO_MAIN && vfo != VFO_SUB)
        return fallback;

    if (vfo == VFO_MAIN && caps->vfo_token_main)
        return caps->vfo_token_main;
    if (vfo == VFO_SUB && caps->vfo_token_sub)
        return caps->vfo_token_sub;

    if (caps->quirks & RIG_QUIRK_FORCE_MAIN_SUB)
        candidates = (vfo == VFO_MAIN) ? main_candidates_strict
                                       : sub_candidates_strict;
    else if (caps->prefer_main_sub_tokens)
        candidates = (vfo == VFO_MAIN) ? main_candidates_prefer
                                       : sub_candidates_prefer;
    else
        candidates = (vfo == VFO_MAIN) ? main_candidates_default
                                       : sub_candidates_default;

    for (gint i = 0; candidates[i] != NULL; i++)
    {
        if (g_hash_table_contains(caps->vfo_working, candidates[i]))
        {
            if (vfo == VFO_MAIN)
                caps->vfo_token_main = g_strdup(candidates[i]);
            else
                caps->vfo_token_sub = g_strdup(candidates[i]);
            return candidates[i];
        }
    }

    return fallback;
}

static void rigctld_client_set_state(RigctldClient *client,
                                     rigctld_client_state_t state,
                                     const gchar *fmt,
                                     ...)
{
    va_list args;
    gchar *reason = NULL;

    if (client == NULL)
        return;

    if (fmt != NULL)
    {
        va_start(args, fmt);
        reason = g_strdup_vprintf(fmt, args);
        va_end(args);
    }

    g_rec_mutex_lock(rigctld_client_meta_lock(client));
    client->state = state;
    g_free(client->state_reason);
    client->state_reason = reason;
    g_rec_mutex_unlock(rigctld_client_meta_lock(client));
}

RigctldClient *rigctld_client_new(const gchar *label)
{
    RigctldClient *client = g_new0(RigctldClient, 1);

    client->transport = hamlib_transport_new();
    g_rec_mutex_init(&client->meta_lock);
    client->state = RIGCTLD_CLIENT_STOPPED;
    client->label = g_strdup(label ? label : "rig");
    client->last_selected_vfo = VFO_NONE;
    rigctld_client_caps_init(&client->caps);

    return client;
}

void rigctld_client_free(RigctldClient **client)
{
    if (client == NULL || *client == NULL)
        return;

    rigctld_client_close(*client);
    g_rec_mutex_lock(rigctld_client_meta_lock(*client));
    rigctld_client_caps_release(&(*client)->caps);
    g_free((*client)->state_reason);
    g_rec_mutex_unlock(rigctld_client_meta_lock(*client));
    g_free((*client)->label);
    g_rec_mutex_clear(&(*client)->meta_lock);
    g_free(*client);
    *client = NULL;
}

void rigctld_client_reset(RigctldClient *client)
{
    if (client == NULL)
        return;

    g_rec_mutex_lock(rigctld_client_meta_lock(client));
    rigctld_client_caps_clear(&client->caps);
    client->last_probe_us = 0;
    client->last_selected_vfo = VFO_NONE;
    rigctld_client_set_state(client, RIGCTLD_CLIENT_STOPPED, "reset");
    g_rec_mutex_unlock(rigctld_client_meta_lock(client));
}

gboolean rigctld_client_connect(RigctldClient *client,
                                const gchar *host,
                                gint port,
                                gint timeout_ms,
                                gchar **error_out)
{
    gboolean ok = FALSE;

    if (client == NULL)
        return FALSE;

    g_rec_mutex_lock(rigctld_client_meta_lock(client));
    rigctld_client_caps_clear(&client->caps);
    client->last_probe_us = 0;
    client->last_selected_vfo = VFO_NONE;

    rigctld_client_set_state(client, RIGCTLD_CLIENT_CONNECTING, "connect");
    ok = hamlib_transport_connect(client->transport, host, port,
                                  timeout_ms, error_out);
    if (!ok)
    {
        rigctld_client_set_state(client, RIGCTLD_CLIENT_DEGRADED,
                                 "connect failed");
        g_rec_mutex_unlock(rigctld_client_meta_lock(client));
        return FALSE;
    }

    rigctld_client_set_state(client, RIGCTLD_CLIENT_CONNECTING,
                             "connected");
    g_rec_mutex_unlock(rigctld_client_meta_lock(client));
    return TRUE;
}

gboolean rigctld_client_attach_fd(RigctldClient *client,
                                  gint fd,
                                  gchar **error_out)
{
    gboolean ok = FALSE;

    if (client == NULL)
        return FALSE;

    g_rec_mutex_lock(rigctld_client_meta_lock(client));
    rigctld_client_caps_clear(&client->caps);
    client->last_probe_us = 0;
    client->last_selected_vfo = VFO_NONE;

    rigctld_client_set_state(client, RIGCTLD_CLIENT_CONNECTING, "attach");
    ok = hamlib_transport_attach_fd(client->transport, fd, error_out);
    if (!ok)
    {
        rigctld_client_set_state(client, RIGCTLD_CLIENT_DEGRADED,
                                 "attach failed");
        g_rec_mutex_unlock(rigctld_client_meta_lock(client));
        return FALSE;
    }

    rigctld_client_set_state(client, RIGCTLD_CLIENT_CONNECTING, "attached");
    g_rec_mutex_unlock(rigctld_client_meta_lock(client));
    return TRUE;
}

void rigctld_client_close(RigctldClient *client)
{
    if (client == NULL)
        return;

    g_rec_mutex_lock(rigctld_client_meta_lock(client));
    hamlib_transport_close(client->transport);
    rigctld_client_set_state(client, RIGCTLD_CLIENT_STOPPED, "closed");
    client->last_selected_vfo = VFO_NONE;
    g_rec_mutex_unlock(rigctld_client_meta_lock(client));
}

rigctld_client_state_t rigctld_client_get_state(const RigctldClient *client)
{
    rigctld_client_state_t state = RIGCTLD_CLIENT_STOPPED;

    if (client == NULL)
        return RIGCTLD_CLIENT_STOPPED;

    g_rec_mutex_lock(rigctld_client_meta_lock(client));
    state = client->state;
    g_rec_mutex_unlock(rigctld_client_meta_lock(client));
    return state;
}

void rigctld_client_get_status(const RigctldClient *client,
                               rigctld_client_state_t *state_out,
                               gchar *reason_out,
                               gsize reason_len)
{
    if (state_out)
        *state_out = RIGCTLD_CLIENT_STOPPED;
    if (reason_out && reason_len > 0)
        reason_out[0] = '\0';

    if (client == NULL)
        return;

    g_rec_mutex_lock(rigctld_client_meta_lock(client));
    if (state_out)
        *state_out = client->state;
    if (reason_out && reason_len > 0)
        g_strlcpy(reason_out,
                  client->state_reason ? client->state_reason : "",
                  reason_len);
    g_rec_mutex_unlock(rigctld_client_meta_lock(client));
}

RigCaps *rigctld_client_get_caps_snapshot(const RigctldClient *client)
{
    RigCaps *caps = NULL;

    if (client == NULL)
        return NULL;

    caps = g_new0(RigCaps, 1);
    if (caps == NULL)
        return NULL;

    g_rec_mutex_lock(rigctld_client_meta_lock(client));
    rigctld_client_caps_copy_snapshot(&client->caps, caps);
    g_rec_mutex_unlock(rigctld_client_meta_lock(client));
    return caps;
}

void rigctld_client_caps_snapshot_free(RigCaps *caps)
{
    if (caps == NULL)
        return;

    rigctld_client_caps_release(caps);
    g_free(caps);
}

HamlibTransport *rigctld_client_get_transport(RigctldClient *client)
{
    if (client == NULL)
        return NULL;
    return client->transport;
}

gboolean rigctld_client_probe(RigctldClient *client,
                              const radio_conf_t *conf,
                              gint timeout_ms)
{
    gchar dump_state[4096];
    gchar reply[256];
    gint64 freq = 0;
    gboolean dump_ok = FALSE;
    gboolean freq_ok = FALSE;
    gboolean vfo_select_ok = FALSE;
    gboolean vfo_opt_args_ok = FALSE;
    gboolean vfo_opt_set = FALSE;
    HamlibResponseInfo info = { 0 };
    gint expected_model = rigctld_client_expected_model(conf);
    gint64 now_us = g_get_monotonic_time();

    if (client == NULL || client->transport == NULL)
        return FALSE;

    g_rec_mutex_lock(rigctld_client_meta_lock(client));
    if (client->last_probe_us > 0 &&
        (now_us - client->last_probe_us) < 500000)
    {
        gboolean ready = (client->state == RIGCTLD_CLIENT_READY);

        g_rec_mutex_unlock(rigctld_client_meta_lock(client));
        return ready;
    }

    client->last_probe_us = now_us;
    rigctld_client_set_state(client, RIGCTLD_CLIENT_PROBING, "probe start");

    rigctld_client_caps_clear(&client->caps);
    if (!hamlib_transport_request(client->transport,
                                  "\\dump_state\n",
                                  HAMLIB_READ_MULTILINE_IDLE,
                                  HAMLIB_TERM_RPRT_OR_DONE,
                                  timeout_ms,
                                  50,
                                  1, RIGCTLD_PROBE_RETRY_DELAY_MS,
                                  dump_state, sizeof(dump_state),
                                  &info))
    {
        if (rigctld_client_get_log_level() >= RIG_LOG_VERBOSE)
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rigctld probe dump_state failed err=%d rprt=%d done=%d",
                        info.err, info.saw_rprt ? 1 : 0, info.saw_done ? 1 : 0);
    }
    else
    {
        dump_ok = rigctld_dump_state_line1_ok(dump_state);
        if (!dump_ok)
        {
            if (rigctld_client_get_log_level() >= RIG_LOG_VERBOSE)
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            "rigctld probe dump_state missing line1=1");
        }

        rigctld_client_parse_dump_state(&client->caps, dump_state);

        client->caps.signature =
            rigctld_client_build_signature(dump_state,
                                           client->caps.rig_model,
                                           client->caps.backend_version);
        rigctld_client_apply_quirks(&client->caps);

        if (expected_model > 0 && client->caps.rig_model > 0 &&
            client->caps.rig_model != expected_model)
        {
            rigctld_client_set_state(client, RIGCTLD_CLIENT_DEGRADED,
                                     "model mismatch expected=%d got=%d",
                                     expected_model, client->caps.rig_model);
            g_rec_mutex_unlock(rigctld_client_meta_lock(client));
            return FALSE;
        }
    }

    client->caps.prefer_main_sub_tokens =
        (conf != NULL && conf->radio_mode == RADIO_MODE_FULL_DUPLEX_MAIN_SUB);

    if (client->caps.vfo_candidates->len == 0)
    {
        rigctld_client_add_vfo_candidate(&client->caps, "VFOA");
        rigctld_client_add_vfo_candidate(&client->caps, "VFOB");
        rigctld_client_add_vfo_candidate(&client->caps, "Main");
        rigctld_client_add_vfo_candidate(&client->caps, "MainA");
        rigctld_client_add_vfo_candidate(&client->caps, "Sub");
        rigctld_client_add_vfo_candidate(&client->caps, "SubA");
        rigctld_client_add_vfo_candidate(&client->caps, "currVFO");
    }

    freq_ok = rigctld_client_try_get_freq_retry(client, "f\n",
                                                &freq, reply, sizeof(reply),
                                                timeout_ms, 1);
    client->caps.has_get_freq = freq_ok;

    for (guint i = 0; i < client->caps.vfo_candidates->len; i++)
    {
        const gchar *token =
            g_ptr_array_index(client->caps.vfo_candidates, i);

        if (!rigctld_client_try_select_vfo(client, token, timeout_ms))
            continue;

        if (rigctld_client_try_get_freq_retry(client, "f\n",
                                              &freq, reply, sizeof(reply),
                                              timeout_ms, 1))
        {
            vfo_select_ok = TRUE;
            g_hash_table_replace(client->caps.vfo_working,
                                 g_strdup(token),
                                 GINT_TO_POINTER(1));
            if (client->caps.default_vfo_token == NULL)
                client->caps.default_vfo_token = g_strdup(token);
        }
    }

    if (client->caps.has_set_vfo_opt || (conf != NULL && conf->vfo_opt))
    {
        vfo_opt_set = rigctld_client_try_set_ok(client,
                                                "\\set_vfo_opt 1\x0a",
                                                reply, sizeof(reply),
                                                timeout_ms);
        if (vfo_opt_set)
        {
            client->caps.vfo_opt_enabled = TRUE;
            if (!rigctld_client_try_get_freq_retry(client, "f\n",
                                                   &freq, reply, sizeof(reply),
                                                   timeout_ms, 1))
            {
                client->caps.vfo_opt_unsafe = TRUE;
                rigctld_client_try_set_ok(client, "\\set_vfo_opt 0\x0a",
                                          reply, sizeof(reply),
                                          timeout_ms);
                client->caps.vfo_opt_enabled = FALSE;
            }
            else
            {
                for (guint i = 0; i < client->caps.vfo_candidates->len; i++)
                {
                    const gchar *token =
                        g_ptr_array_index(client->caps.vfo_candidates, i);
                    gchar cmd[96];

                    g_snprintf(cmd, sizeof(cmd), "f %s\x0a", token);
                    if (!rigctld_client_try_get_freq_retry(client, cmd,
                                                           &freq, reply,
                                                           sizeof(reply),
                                                           timeout_ms, 1))
                        continue;

                    g_free(client->caps.default_vfo_token);
                    client->caps.default_vfo_token = g_strdup(token);
                    g_snprintf(cmd, sizeof(cmd), "F %s %" G_GINT64_FORMAT "\x0a",
                               token, freq);
                    if (rigctld_client_try_set_ok(client, cmd,
                                                  reply, sizeof(reply),
                                                  timeout_ms))
                    {
                        vfo_opt_args_ok = TRUE;
                        break;
                    }
                }
            }
        }
    }

    if (conf != NULL &&
        conf->radio_mode == RADIO_MODE_FULL_DUPLEX_MAIN_SUB &&
        vfo_select_ok)
    {
        /* Shared Main/Sub rigs should stay on the simpler V + F path when
           both sides already prove selectable. The failing logs are on the
           tokenized F <vfo> <freq> path, so do not promote that strategy. */
        vfo_opt_args_ok = FALSE;
    }

    if (vfo_opt_args_ok)
    {
        client->caps.strategy = RIG_STRATEGY_VFO_OPT_ARGS;
        if (client->caps.default_vfo_token == NULL)
            client->caps.default_vfo_token = g_strdup("currVFO");
    }
    else if (vfo_select_ok)
    {
        client->caps.strategy = RIG_STRATEGY_SELECT_VFO;
        if (client->caps.vfo_opt_enabled)
        {
            rigctld_client_try_set_ok(client, "\\set_vfo_opt 0\x0a",
                                      reply, sizeof(reply), timeout_ms);
            client->caps.vfo_opt_enabled = FALSE;
        }
    }
    else if (freq_ok)
    {
        client->caps.strategy = RIG_STRATEGY_PLAIN_FREQ;
        if (client->caps.vfo_opt_enabled)
        {
            rigctld_client_try_set_ok(client, "\\set_vfo_opt 0\x0a",
                                      reply, sizeof(reply), timeout_ms);
            client->caps.vfo_opt_enabled = FALSE;
        }
    }
    else if (dump_ok)
    {
        client->caps.strategy = RIG_STRATEGY_PLAIN_FREQ;
    }
    else
    {
        rigctld_client_set_state(client, RIGCTLD_CLIENT_DEGRADED,
                                 "no usable control strategy");
        g_rec_mutex_unlock(rigctld_client_meta_lock(client));
        return FALSE;
    }

    if (freq_ok)
    {
        gchar cmd[96];
        g_snprintf(cmd, sizeof(cmd), "F %" G_GINT64_FORMAT "\x0a", freq);
        client->caps.has_set_freq =
            rigctld_client_try_set_ok(client, cmd,
                                      reply, sizeof(reply), timeout_ms);
    }

    if (!dump_ok && !freq_ok && !vfo_select_ok && !vfo_opt_args_ok)
    {
        rigctld_client_set_state(client, RIGCTLD_CLIENT_DEGRADED,
                                 "probe failed");
        g_rec_mutex_unlock(rigctld_client_meta_lock(client));
        return FALSE;
    }

    rigctld_client_set_state(client, RIGCTLD_CLIENT_READY,
                             "strategy=%d", client->caps.strategy);
    g_rec_mutex_unlock(rigctld_client_meta_lock(client));
    return TRUE;
}

gboolean rigctld_client_get_freq(RigctldClient *client,
                                 vfo_t vfo,
                                 gint64 *freq_out)
{
    gchar cmd[96];
    gchar reply[256];
    RigCaps *caps = NULL;

    if (freq_out)
        *freq_out = 0;

    if (client == NULL)
        return FALSE;

    g_rec_mutex_lock(rigctld_client_meta_lock(client));
    caps = &client->caps;

    if (caps->strategy == RIG_STRATEGY_VFO_OPT_ARGS)
    {
        const gchar *token = NULL;
        token = rigctld_client_vfo_token(client, vfo);
        g_snprintf(cmd, sizeof(cmd), "f %s\x0a", token);
        {
            gboolean ok = rigctld_client_try_get_freq(client, cmd,
                                                      freq_out, reply,
                                                      sizeof(reply), 500);
            g_rec_mutex_unlock(rigctld_client_meta_lock(client));
            return ok;
        }
    }

    if (caps->strategy == RIG_STRATEGY_SELECT_VFO)
    {
        if (!rigctld_client_ensure_vfo(client, vfo))
        {
            g_rec_mutex_unlock(rigctld_client_meta_lock(client));
            return FALSE;
        }
    }

    g_snprintf(cmd, sizeof(cmd), "f\x0a");
    {
        gboolean ok = rigctld_client_try_get_freq(client, cmd,
                                                  freq_out, reply,
                                                  sizeof(reply), 500);
        g_rec_mutex_unlock(rigctld_client_meta_lock(client));
        return ok;
    }
}

gboolean rigctld_client_ensure_vfo(RigctldClient *client,
                                   vfo_t vfo)
{
    RigCaps *caps = NULL;
    const gchar *token = NULL;
    gboolean ok = FALSE;

    if (client == NULL)
        return FALSE;

    g_rec_mutex_lock(rigctld_client_meta_lock(client));
    caps = &client->caps;
    if (caps->strategy != RIG_STRATEGY_SELECT_VFO)
    {
        g_rec_mutex_unlock(rigctld_client_meta_lock(client));
        return TRUE;
    }

    if (vfo != VFO_MAIN && vfo != VFO_SUB)
    {
        g_rec_mutex_unlock(rigctld_client_meta_lock(client));
        return TRUE;
    }

    if (client->last_selected_vfo == vfo)
    {
        g_rec_mutex_unlock(rigctld_client_meta_lock(client));
        return TRUE;
    }

    token = rigctld_client_vfo_token(client, vfo);
    ok = rigctld_client_try_select_vfo(client, token, 500);
    if (ok)
        client->last_selected_vfo = vfo;

    g_rec_mutex_unlock(rigctld_client_meta_lock(client));
    return ok;
}

gboolean rigctld_client_set_freq(RigctldClient *client,
                                 vfo_t vfo,
                                 gint64 freq_hz)
{
    gchar cmd[96];
    gchar reply[128];
    const gchar *token = NULL;
    RigCaps *caps = NULL;
    gboolean ok = FALSE;

    if (client == NULL)
        return FALSE;

    g_rec_mutex_lock(rigctld_client_meta_lock(client));
    caps = &client->caps;

    if (caps->strategy == RIG_STRATEGY_VFO_OPT_ARGS)
    {
        token = rigctld_client_vfo_token(client, vfo);
        g_snprintf(cmd, sizeof(cmd), "F %s %" G_GINT64_FORMAT "\x0a",
                   token, freq_hz);
        {
            gboolean ok = rigctld_client_try_set_ok(client, cmd,
                                                    reply, sizeof(reply), 500);
            g_rec_mutex_unlock(rigctld_client_meta_lock(client));
            return ok;
        }
    }

    if (caps->strategy == RIG_STRATEGY_SELECT_VFO)
    {
        gboolean can_select = (vfo == VFO_MAIN || vfo == VFO_SUB);

        if (can_select && !rigctld_client_ensure_vfo(client, vfo))
        {
            g_rec_mutex_unlock(rigctld_client_meta_lock(client));
            return FALSE;
        }

        g_snprintf(cmd, sizeof(cmd), "F %" G_GINT64_FORMAT "\x0a", freq_hz);
        ok = rigctld_client_try_set_ok(client, cmd,
                                       reply, sizeof(reply), 500);
        if (ok || !can_select)
        {
            g_rec_mutex_unlock(rigctld_client_meta_lock(client));
            return ok;
        }

        token = rigctld_client_vfo_token(client, vfo);
        if (!rigctld_client_try_select_vfo(client, token, 500))
        {
            g_rec_mutex_unlock(rigctld_client_meta_lock(client));
            return FALSE;
        }
        client->last_selected_vfo = vfo;
        {
            gboolean retry_ok = rigctld_client_try_set_ok(client, cmd,
                                                          reply,
                                                          sizeof(reply), 500);
            g_rec_mutex_unlock(rigctld_client_meta_lock(client));
            return retry_ok;
        }
    }

    g_snprintf(cmd, sizeof(cmd), "F %" G_GINT64_FORMAT "\x0a", freq_hz);
    {
        gboolean ok = rigctld_client_try_set_ok(client, cmd,
                                                reply, sizeof(reply), 500);
        g_rec_mutex_unlock(rigctld_client_meta_lock(client));
        return ok;
    }
}

gboolean rigctld_client_set_vfo(RigctldClient *client,
                                const gchar *token)
{
    gchar cmd[96];
    gchar reply[128];

    if (client == NULL || token == NULL || *token == '\0')
        return FALSE;

    g_snprintf(cmd, sizeof(cmd), "V %s\x0a", token);
    return rigctld_client_try_set_ok(client, cmd,
                                     reply, sizeof(reply), 500);
}

gboolean rigctld_client_set_vfo_opt(RigctldClient *client,
                                    gboolean enable)
{
    gchar reply[128];

    if (client == NULL)
        return FALSE;

    g_rec_mutex_lock(rigctld_client_meta_lock(client));
    if (enable)
    {
        if (rigctld_client_try_set_ok(client, "\\set_vfo_opt 1\x0a",
                                      reply, sizeof(reply), 500))
        {
            client->caps.vfo_opt_enabled = TRUE;
            g_rec_mutex_unlock(rigctld_client_meta_lock(client));
            return TRUE;
        }
    }
    else
    {
        if (rigctld_client_try_set_ok(client, "\\set_vfo_opt 0\x0a",
                                      reply, sizeof(reply), 500))
        {
            client->caps.vfo_opt_enabled = FALSE;
            g_rec_mutex_unlock(rigctld_client_meta_lock(client));
            return TRUE;
        }
    }

    g_rec_mutex_unlock(rigctld_client_meta_lock(client));
    return FALSE;
}

gboolean rigctld_client_request_raw(RigctldClient *client,
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
    gint request_timeout_ms = 1000;

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
    else if (g_str_has_prefix(send_cmd, "F ") ||
             g_str_has_prefix(send_cmd, "I "))
        request_timeout_ms = 3000;

    ok = hamlib_transport_request(client->transport,
                                  send_cmd,
                                  mode,
                                  HAMLIB_TERM_RPRT,
                                  request_timeout_ms,
                                  50,
                                  0, 0,
                                  out, out_len,
                                  &local);
    g_free(tmp_cmd);
    if (info)
        *info = local;
    if (!ok)
        return FALSE;

    if (out && out_len > 0 && local.saw_rprt)
    {
        gchar rprt[32];
        g_snprintf(rprt, sizeof(rprt), "RPRT %d\n", local.rprt_code);
        g_strlcat(out, rprt, out_len);
    }

    return TRUE;
}

gint64 rigctld_client_last_rtt_us(const RigctldClient *client)
{
    if (client == NULL || client->transport == NULL)
        return 0;
    return hamlib_transport_last_rtt_us(client->transport);
}
