/*
  Gpredict: Real-time satellite tracking and orbit prediction program

  Copyright (C)  2001-2019  Alexandru Csete, OZ9AEC
  Copyright (C)       2017  Patrick Dohmen, DL4PD
  Copyright (C)       2018  Mario Haustein, DM5AHA

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, visit http://www.fsf.org/
*/
/*
 * RIG control window.
 *
 * The master radio control UI is implemented as a Gtk+ Widget in order
 * to allow multiple instances. The widget is created from the module
 * popup menu and each module can have several radio control windows
 * attached to it. Note, however, that current implementation only
 * allows one control window per module.
 *
 * TODO Duplex TRX
 * TODO Transponder passband display somewhere, below Sat freq?
 *
 */
#ifdef HAVE_CONFIG_H
#include <build-config.h>
#endif

#include <gdk/gdkkeysyms.h>
#include <glib.h>
#include <glib/gi18n.h>
#include <glib/gstdio.h>
#include <gtk/gtk.h>
#include <gio/gio.h>
#include <math.h>
#include <errno.h>
#include <sys/time.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#ifdef G_OS_WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>
#ifndef socklen_t
typedef int socklen_t;
#endif
#include <windows.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <unistd.h>
#endif
#ifndef WIN32
#include <fcntl.h>
#endif

#include "net_compat.h"

#include "compat.h"
#include "gp-term-view.h"
#include "gpredict-utils.h"
#include "gtk-freq-knob.h"
#include "gtk-rig-ctrl.h"
#include "rig-mode-dispatch.h"
#include "predict-tools.h"
#include "radio-conf.h"
#include "rigctld-io.h"
#include "rigctld_client.h"
#include "rigctld_mgr.h"
#include "rotctld-parse.h"
#include "serial-ports.h"
#include "sat-log.h"
#include "sat-cfg.h"
#include "sat-pref-rig-editor.h"
#include "status_indicator.h"
#include "trsp-conf.h"
#include "ui-popup-quarantine.h"
#include "ui-status.h"

#ifndef G_SUBPROCESS_FLAGS_STDIN_DEV_NULL
#ifdef G_SUBPROCESS_FLAGS_STDIN_INHERIT
#define G_SUBPROCESS_FLAGS_STDIN_DEV_NULL G_SUBPROCESS_FLAGS_STDIN_INHERIT
#else
#define G_SUBPROCESS_FLAGS_STDIN_DEV_NULL 0
#endif
#endif

#ifndef AI_ADDRCONFIG
#define AI_ADDRCONFIG 0
#endif

#ifdef G_OS_WIN32
static gboolean winsock_ensure_init(void)
{
    static gsize init_state = 0;

    if (g_once_init_enter(&init_state))
    {
        WSADATA wsa;
        int rc = WSAStartup(MAKEWORD(2, 2), &wsa);
        g_once_init_leave(&init_state, (rc == 0) ? 1 : 2);
    }

    return init_state == 1;
}
#endif


#define AZEL_FMTSTR "%7.2f\302\260"
#define MAX_ERROR_COUNT 5
#define WR_DEL 5000             /* delay in usec to wait between write and read commands */
#define RIGCTLD_SOCKET_TIMEOUT_MS 3000
#define RIGCTLD_DUMP_STATE_IDLE_MS 100
#define RIGCTLD_FOLLOW_IDLE_MS 50
#define RIGCTLD_AUTODETECT_MAX_CANDIDATES 8
#define RIGCTLD_AUTODETECT_TOTAL_MS 10000
#define RIGCTLD_AUTODETECT_TOTAL_MS_PER_CANDIDATE 12000
#define RIGCTLD_AUTODETECT_TOTAL_MS_MAX 60000
#define RIGCTLD_AUTODETECT_WAIT_MS 4000
#define RIGCTLD_AUTODETECT_PROBE_MS 1500
#define RIGCTLD_PROBE_SHORT_MS 300
#define RIGCTLD_STARTUP_TIMEOUT_MS 2000
#define RIGCTLD_STARTUP_POLL_MS 25
#define RIGCTLD_AUTOSTART_TIMEOUT_MS 2000
#define RIGCTLD_AUTOSTART_POLL_MS 20
#define RIGCTLD_HEALTH_TIMEOUT_MS 200
#define RIGCTLD_HEALTH_RETRIES 3
#define RIGCTLD_HEALTH_RETRY_DELAY_MS 50
#define RIGCTRL_FREQ_PLACEHOLDER "--- Hz"
#define RIGCTRL_FREQ_PLACEHOLDER_DIGIT "<span size='xx-large'>-</span>"
#define RIGCTLD_AUTOSTART_MAX_RESTARTS 2
#define RIGCTLD_AUTOSTART_RETRY_DELAY_MS 150
#define RIGCTLD_MODEL_IC9700 3081
#define RIGCTLD_MODEL_IC905 3090
#define RIGCTLD_IC905_FALLBACK_TIMEOUT_MS 12000
#define RIGCTRL_RECONNECT_BACKOFF_MIN_MS 5000
#define RIGCTRL_RECONNECT_BACKOFF_MAX_MS 10000
#define RIGCTRL_RECONNECT_MAX_ATTEMPTS 2
#define RIGCTRL_RESPONSE_OPEN_CONFIG 1001
#define RIGCTRL_RESPONSE_DISABLE_AUTOSTART 1002
#define RIGCTRL_RESPONSE_SHOW_LOG 1003
#define RIGCTRL_TRSP_POPUP_MAX_HEIGHT 360
#define RIGCTRL_TRSP_POPUP_MAX_FACTOR 0.45
#define RIGCTRL_TRSP_POPUP_SEARCH_THRESHOLD 20
#define RIGCTRL_TRSP_POPUP_SCROLL_CHECK_THRESHOLD 50
#define RIGCTRL_TRSP_POPUP_DEBOUNCE_US (250 * G_TIME_SPAN_MILLISECOND)
#define RIGCTRL_DOPPLER_MIN_STEP_HZ_DEFAULT 1.0
#define RIGCTRL_DOPPLER_MAX_INTERVAL_MS 1500
#define RIGCTRL_DOPPLER_LOG_INTERVAL_US 2000000
#define RIGCTRL_PROBE_LOG_INTERVAL_US 5000000
#define RIGCTRL_VERIFY_LOG_INTERVAL_US 5000000
#define RIGCTRL_TARGET_MAX_AZ_DEG 720.0
#define RIGCTRL_TARGET_MAX_EL_DEG 180.0
#define RIGCTRL_TARGET_MAX_RANGE_KM 1000000.0
#define RIGCTRL_TARGET_MAX_RANGE_RATE_KM_S 1000.0

#ifdef RIGCTRL_TRSP_POPUP_DEBUG
#define RIGCTRL_TRSP_POPUP_LOG(...) \
    sat_log_log(SAT_LOG_LEVEL_DEBUG, __VA_ARGS__)
#else
#define RIGCTRL_TRSP_POPUP_LOG(...) do { } while (0)
#endif

static GHashTable *rigctld_device_cache = NULL;
static GHashTable *rig_freq_cache = NULL;

typedef enum {
    RIGCTLD_PROBE_OK = 0,
    RIGCTLD_PROBE_NOT_READY,
    RIGCTLD_PROBE_MISMATCH
} rigctld_probe_result_t;

typedef enum {
    RIG_SESSION_STOPPED = 0,
    RIG_SESSION_STARTING_RIGCTLD,
    RIG_SESSION_CONNECTING,
    RIG_SESSION_PROBING,
    RIG_SESSION_CONFIGURING,
    RIG_SESSION_READY,
    RIG_SESSION_DEGRADED,
    RIG_SESSION_RECONNECTING
} rig_session_state_t;

typedef enum {
    VFO_ROLE_DOWNLINK = 0,
    VFO_ROLE_UPLINK
} vfo_role_t;

typedef enum {
    RIG_BASE_SRC_NONE = 0,
    RIG_BASE_SRC_PRESET,
    RIG_BASE_SRC_MANUAL
} rig_base_source_t;

typedef struct _RigSession {
    rig_session_state_t state;
    rig_strategy_t strategy;
    gchar *state_reason;
    gchar *backend_version;
    gchar *signature;
    gint rig_model;
    guint quirks;
    gboolean has_get_vfo;
    gboolean has_set_vfo;
    gboolean has_set_vfo_opt;
    gboolean vfo_opt_enabled;
    gboolean vfo_opt_unsafe;
    gchar *default_vfo_token;
    GPtrArray *vfo_candidates;
    GHashTable *vfo_working;
    gboolean strategy_logged;
    vfo_t last_selected_vfo;
    gboolean last_selected_vfo_valid;
    gint64 user_base_freq_hz[2]; /* base_rx_hz/base_tx_hz */
    gboolean user_base_valid[2];
    rig_base_source_t base_src[2];
    gint64 calc_base_hz[2];
    gint64 calc_doppler_hz[2];
    gint64 calc_final_hz[2];
    gboolean calc_valid[2];
    gboolean doppler_enabled[2];
    gint64 last_sent_hz[2]; /* last_sent_rx_hz/last_sent_tx_hz */
    gint64 last_sent_us[2];
    gint64 last_force_send_us[2];
    gint64 last_target_hz[2]; /* last_target_rx_hz/last_target_tx_hz */
    gchar *label;
} RigSession;

static void rig_term_log(GtkRigCtrl *ctrl, const gchar *prefix,
                         const gchar *fmt, ...) G_GNUC_PRINTF(3, 4);
static const gchar *rigctld_probe_result_name(rigctld_probe_result_t result)
{
    switch (result)
    {
    case RIGCTLD_PROBE_OK:
        return "OK";
    case RIGCTLD_PROBE_NOT_READY:
        return "NOT_READY";
    case RIGCTLD_PROBE_MISMATCH:
        return "MISMATCH";
    default:
        return "UNKNOWN";
    }
}

static RigSession *rig_session_new(const gchar *label)
{
    RigSession *session = g_new0(RigSession, 1);

    session->state = RIG_SESSION_STOPPED;
    session->strategy = RIG_STRATEGY_PLAIN_FREQ;
    session->rig_model = 0;
    session->quirks = RIG_QUIRK_NONE;
    session->has_get_vfo = FALSE;
    session->has_set_vfo = FALSE;
    session->has_set_vfo_opt = FALSE;
    session->vfo_opt_enabled = FALSE;
    session->vfo_opt_unsafe = FALSE;
    session->vfo_candidates = g_ptr_array_new_with_free_func(g_free);
    session->vfo_working = g_hash_table_new_full(g_str_hash, g_str_equal,
                                                 g_free, NULL);
    session->strategy_logged = FALSE;
    session->last_selected_vfo = VFO_NONE;
    session->last_selected_vfo_valid = FALSE;
    session->user_base_freq_hz[0] = 0;
    session->user_base_freq_hz[1] = 0;
    session->user_base_valid[0] = FALSE;
    session->user_base_valid[1] = FALSE;
    session->base_src[0] = RIG_BASE_SRC_NONE;
    session->base_src[1] = RIG_BASE_SRC_NONE;
    session->calc_base_hz[0] = 0;
    session->calc_base_hz[1] = 0;
    session->calc_doppler_hz[0] = 0;
    session->calc_doppler_hz[1] = 0;
    session->calc_final_hz[0] = 0;
    session->calc_final_hz[1] = 0;
    session->calc_valid[0] = FALSE;
    session->calc_valid[1] = FALSE;
    session->doppler_enabled[0] = FALSE;
    session->doppler_enabled[1] = FALSE;
    session->last_sent_hz[0] = 0;
    session->last_sent_hz[1] = 0;
    session->last_sent_us[0] = 0;
    session->last_sent_us[1] = 0;
    session->last_force_send_us[0] = 0;
    session->last_force_send_us[1] = 0;
    session->last_target_hz[0] = 0;
    session->last_target_hz[1] = 0;
    session->label = g_strdup(label ? label : "rig");

    return session;
}

static void rig_session_reset(RigSession *session)
{
    if (session == NULL)
        return;

    session->state = RIG_SESSION_STOPPED;
    session->strategy = RIG_STRATEGY_PLAIN_FREQ;
    g_free(session->state_reason);
    session->state_reason = NULL;
    g_free(session->backend_version);
    session->backend_version = NULL;
    g_free(session->signature);
    session->signature = NULL;
    session->rig_model = 0;
    session->quirks = RIG_QUIRK_NONE;
    session->has_get_vfo = FALSE;
    session->has_set_vfo = FALSE;
    session->has_set_vfo_opt = FALSE;
    session->vfo_opt_enabled = FALSE;
    session->vfo_opt_unsafe = FALSE;
    g_free(session->default_vfo_token);
    session->default_vfo_token = NULL;
    if (session->vfo_candidates)
        g_ptr_array_set_size(session->vfo_candidates, 0);
    if (session->vfo_working)
        g_hash_table_remove_all(session->vfo_working);
    session->strategy_logged = FALSE;
    session->last_selected_vfo = VFO_NONE;
    session->last_selected_vfo_valid = FALSE;
    session->user_base_freq_hz[0] = 0;
    session->user_base_freq_hz[1] = 0;
    session->user_base_valid[0] = FALSE;
    session->user_base_valid[1] = FALSE;
    session->base_src[0] = RIG_BASE_SRC_NONE;
    session->base_src[1] = RIG_BASE_SRC_NONE;
    session->calc_base_hz[0] = 0;
    session->calc_base_hz[1] = 0;
    session->calc_doppler_hz[0] = 0;
    session->calc_doppler_hz[1] = 0;
    session->calc_final_hz[0] = 0;
    session->calc_final_hz[1] = 0;
    session->calc_valid[0] = FALSE;
    session->calc_valid[1] = FALSE;
    session->doppler_enabled[0] = FALSE;
    session->doppler_enabled[1] = FALSE;
    session->last_sent_hz[0] = 0;
    session->last_sent_hz[1] = 0;
    session->last_sent_us[0] = 0;
    session->last_sent_us[1] = 0;
    session->last_force_send_us[0] = 0;
    session->last_force_send_us[1] = 0;
    session->last_target_hz[0] = 0;
    session->last_target_hz[1] = 0;
}

static void rig_session_free(RigSession **session_ptr)
{
    RigSession *session;

    if (session_ptr == NULL || *session_ptr == NULL)
        return;

    session = *session_ptr;
    g_free(session->state_reason);
    g_free(session->backend_version);
    g_free(session->signature);
    g_free(session->default_vfo_token);
    g_free(session->label);
    if (session->vfo_candidates)
        g_ptr_array_free(session->vfo_candidates, TRUE);
    if (session->vfo_working)
        g_hash_table_destroy(session->vfo_working);

    g_free(session);
    *session_ptr = NULL;
}

static const gchar *rig_session_state_name(rig_session_state_t state)
{
    switch (state)
    {
    case RIG_SESSION_STOPPED:
        return "STOPPED";
    case RIG_SESSION_STARTING_RIGCTLD:
        return "STARTING_RIGCTLD";
    case RIG_SESSION_CONNECTING:
        return "CONNECTING";
    case RIG_SESSION_PROBING:
        return "PROBING";
    case RIG_SESSION_CONFIGURING:
        return "CONFIGURING";
    case RIG_SESSION_READY:
        return "READY";
    case RIG_SESSION_DEGRADED:
        return "DEGRADED";
    case RIG_SESSION_RECONNECTING:
        return "RECONNECTING";
    default:
        return "UNKNOWN";
    }
}

static const gchar *rig_strategy_name(rig_strategy_t strategy)
{
    switch (strategy)
    {
    case RIG_STRATEGY_PLAIN_FREQ:
        return "PLAIN_FREQ";
    case RIG_STRATEGY_SELECT_VFO:
        return "SELECT_VFO_BEFORE_OP";
    case RIG_STRATEGY_VFO_OPT_ARGS:
        return "VFO_OPT_WITH_ARGS";
    default:
        return "UNKNOWN";
    }
}

static gboolean rig_strategy_supports_explicit_vfo(rig_strategy_t strategy)
{
    return (strategy == RIG_STRATEGY_SELECT_VFO ||
            strategy == RIG_STRATEGY_VFO_OPT_ARGS);
}

static void rigctrl_clear_ui_hard_error(GtkRigCtrl *ctrl);
static void rigctrl_queue_ui_status_refresh(GtkRigCtrl *ctrl,
                                            const gchar *reason);

static void rig_session_set_state(GtkRigCtrl *ctrl, RigSession *session,
                                  rig_session_state_t state,
                                  const gchar *reason_fmt, ...)
{
    gchar *reason = NULL;
    va_list args;

    if (session == NULL)
        return;

    if (reason_fmt != NULL)
    {
        va_start(args, reason_fmt);
        reason = g_strdup_vprintf(reason_fmt, args);
        va_end(args);
    }

    if (session->state != state || reason != NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "rig session (%s) state=%s reason=%s",
                    session->label ? session->label : "rig",
                    rig_session_state_name(state),
                    reason ? reason : "(none)");
        if (ctrl != NULL)
        {
            rig_term_log(ctrl, "gpredict",
                         "rig session (%s) state=%s reason=%s",
                         session->label ? session->label : "rig",
                         rig_session_state_name(state),
                         reason ? reason : "(none)");
        }
    }

    session->state = state;
    g_free(session->state_reason);
    session->state_reason = reason;

    if (state == RIG_SESSION_READY)
        rigctrl_clear_ui_hard_error(ctrl);
    if (ctrl != NULL)
        rigctrl_queue_ui_status_refresh(ctrl,
                                        reason ? reason : "session state");
}

static void rig_session_apply_caps(RigSession *session, const RigCaps *caps)
{
    if (session == NULL || caps == NULL)
        return;

    session->strategy = caps->strategy;
    g_free(session->backend_version);
    session->backend_version = g_strdup(caps->backend_version);
    g_free(session->signature);
    session->signature = g_strdup(caps->signature);
    session->rig_model = caps->rig_model;
    session->quirks = caps->quirks;
    session->has_get_vfo = caps->has_get_vfo;
    session->has_set_vfo = caps->has_set_vfo;
    session->has_set_vfo_opt = caps->has_set_vfo_opt;
    session->vfo_opt_enabled = caps->vfo_opt_enabled;
    session->vfo_opt_unsafe = caps->vfo_opt_unsafe;

    g_free(session->default_vfo_token);
    session->default_vfo_token = g_strdup(caps->default_vfo_token);

    if (session->vfo_candidates)
        g_ptr_array_set_size(session->vfo_candidates, 0);
    if (caps->vfo_candidates)
    {
        for (guint i = 0; i < caps->vfo_candidates->len; i++)
        {
            const gchar *token = g_ptr_array_index(caps->vfo_candidates, i);
            if (token != NULL && *token != '\0')
                g_ptr_array_add(session->vfo_candidates, g_strdup(token));
        }
    }

    if (session->vfo_working)
        g_hash_table_remove_all(session->vfo_working);
    if (caps->vfo_working)
    {
        GHashTableIter iter;
        gpointer key = NULL;
        gpointer value = NULL;

        g_hash_table_iter_init(&iter, caps->vfo_working);
        while (g_hash_table_iter_next(&iter, &key, &value))
        {
            const gchar *token = key;
            if (token != NULL)
                g_hash_table_replace(session->vfo_working,
                                     g_strdup(token),
                                     value);
        }
    }
}

static void rig_session_copy_caps(RigSession *dst, const RigSession *src)
{
    if (dst == NULL || src == NULL)
        return;

    dst->strategy = src->strategy;
    dst->rig_model = src->rig_model;
    dst->quirks = src->quirks;
    dst->has_get_vfo = src->has_get_vfo;
    dst->has_set_vfo = src->has_set_vfo;
    dst->has_set_vfo_opt = src->has_set_vfo_opt;
    dst->vfo_opt_enabled = src->vfo_opt_enabled;
    dst->vfo_opt_unsafe = src->vfo_opt_unsafe;

    g_free(dst->backend_version);
    dst->backend_version = g_strdup(src->backend_version);
    g_free(dst->signature);
    dst->signature = g_strdup(src->signature);
    g_free(dst->default_vfo_token);
    dst->default_vfo_token = g_strdup(src->default_vfo_token);

    if (dst->vfo_candidates)
        g_ptr_array_set_size(dst->vfo_candidates, 0);
    if (src->vfo_candidates)
    {
        for (guint i = 0; i < src->vfo_candidates->len; i++)
        {
            const gchar *token = g_ptr_array_index(src->vfo_candidates, i);
            if (token != NULL && *token != '\0')
                g_ptr_array_add(dst->vfo_candidates, g_strdup(token));
        }
    }

    if (dst->vfo_working)
        g_hash_table_remove_all(dst->vfo_working);
    if (src->vfo_working)
    {
        GHashTableIter iter;
        gpointer key = NULL;
        gpointer value = NULL;

        g_hash_table_iter_init(&iter, src->vfo_working);
        while (g_hash_table_iter_next(&iter, &key, &value))
        {
            const gchar *token = key;
            if (token != NULL)
                g_hash_table_replace(dst->vfo_working,
                                     g_strdup(token),
                                     value);
        }
    }
}

/* radio control functions */
static void     exec_rx_cycle(GtkRigCtrl * ctrl);
static void     exec_tx_cycle(GtkRigCtrl * ctrl);
static void     exec_trx_cycle(GtkRigCtrl * ctrl);
static void     exec_toggle_cycle(GtkRigCtrl * ctrl);
static void     exec_toggle_tx_cycle(GtkRigCtrl * ctrl);
static void     exec_full_duplex_main_sub_cycle(GtkRigCtrl * ctrl,
                                                gboolean force_send);
static void     exec_duplex_cycle(GtkRigCtrl * ctrl);
static void     exec_duplex_tx_cycle(GtkRigCtrl * ctrl);
static void     exec_dual_rig_cycle(GtkRigCtrl * ctrl);
static gboolean check_aos_los(GtkRigCtrl * ctrl);
static gboolean set_freq_simplex(GtkRigCtrl * ctrl, gint sock, gint64 freq);
static gboolean get_freq_simplex(GtkRigCtrl * ctrl, gint sock, gint64 * freq);
static gboolean get_freq_simplex_strict(GtkRigCtrl * ctrl, gint sock,
                                        gint64 * freq);
static gboolean set_freq_toggle(GtkRigCtrl * ctrl, gint sock, gint64 freq);
static gboolean set_toggle(GtkRigCtrl * ctrl, gint sock);
static gboolean unset_toggle(GtkRigCtrl * ctrl, gint sock);
static gboolean get_freq_toggle(GtkRigCtrl * ctrl, gint sock, gint64 * freq);
static gboolean get_freq_toggle_strict(GtkRigCtrl * ctrl, gint sock,
                                       gint64 * freq);
static gboolean set_freq_simplex_vfo(GtkRigCtrl *ctrl, gint sock,
                                     gint64 freq, vfo_t vfo);
static gboolean get_freq_simplex_vfo(GtkRigCtrl *ctrl, gint sock,
                                     gint64 *freq, vfo_t vfo);
static gboolean get_freq_simplex_vfo_strict(GtkRigCtrl *ctrl, gint sock,
                                            gint64 *freq, vfo_t vfo);
static gboolean set_freq_toggle_vfo(GtkRigCtrl *ctrl, gint sock,
                                    gint64 freq, vfo_t vfo);
static gboolean get_freq_toggle_vfo(GtkRigCtrl *ctrl, gint sock,
                                    gint64 *freq, vfo_t vfo);
static gboolean get_freq_toggle_vfo_strict(GtkRigCtrl *ctrl, gint sock,
                                           gint64 *freq, vfo_t vfo);
static gboolean get_ptt(GtkRigCtrl * ctrl, gint sock);
static gboolean set_ptt(GtkRigCtrl * ctrl, gint sock, gboolean ptt);
static gboolean set_rit(GtkRigCtrl * ctrl, gint sock, gdouble hz);
static gboolean set_xit(GtkRigCtrl * ctrl, gint sock, gdouble hz);
static void     apply_rit_xit_offsets(GtkRigCtrl * ctrl, gdouble rit,
                                      gdouble xit);
static void     update_rit_xit_offsets(GtkRigCtrl * ctrl);
static gint64   rigctrl_round_hz(gdouble hz);
static gboolean is_full_duplex_main_sub_configured(const radio_conf_t *conf);
static vfo_t    rigctrl_target_vfo_for_role(const radio_conf_t *conf,
                                            vfo_role_t role);
static vfo_t    rigctrl_vfo_for_side(GtkRigCtrl *ctrl, gboolean downlink);
static const gchar *rigctrl_vfo_label(GtkRigCtrl *ctrl,
                                      gboolean downlink,
                                      vfo_t vfo);
static gboolean rigctrl_set_freq_for_role(GtkRigCtrl *ctrl,
                                          gint sock,
                                          gboolean downlink,
                                          gboolean toggle_mode,
                                          gint64 freq_hz,
                                          vfo_t *vfo_out);
static gboolean rigctrl_get_freq_for_role(GtkRigCtrl *ctrl,
                                          gint sock,
                                          gboolean downlink,
                                          gboolean toggle_mode,
                                          gboolean strict,
                                          gint64 *freq_out,
                                          vfo_t *vfo_out);
static const radio_conf_t *rigctrl_conf_for_role(const GtkRigCtrl *ctrl,
                                                 gboolean downlink);
static RigSession *rigctrl_session_for_role(GtkRigCtrl *ctrl, gboolean downlink);
static gboolean rigctrl_prepare_shared_tx_session(GtkRigCtrl *ctrl);
static RigSession *rig_session_for_socket(GtkRigCtrl *ctrl, gint sock);
static RigctldClient *rigctld_client_for_socket(GtkRigCtrl *ctrl, gint sock);
static rig_strategy_t rig_session_strategy(GtkRigCtrl *ctrl, gint sock);
static RigSession *rig_session_for_socket_vfo(GtkRigCtrl *ctrl, gint sock,
                                              vfo_t vfo);
static rig_strategy_t rig_session_strategy_for_vfo(GtkRigCtrl *ctrl,
                                                   gint sock,
                                                   vfo_t vfo);
static const gchar *rig_session_default_vfo_token(RigSession *session);
static gboolean rig_session_vfo_candidate_exists(const RigSession *session,
                                                 const gchar *token);
static gboolean G_GNUC_UNUSED rig_session_send_command(GtkRigCtrl *ctrl,
                                                       gint sock,
                                                       const gchar *cmd,
                                                       gchar *buffout,
                                                       gint sizeout);
static gboolean rig_session_probe_and_configure(GtkRigCtrl *ctrl,
                                                RigSession *session,
                                                gint sock,
                                                const radio_conf_t *conf);
static gboolean open_rigctld_socket_with_autostart(GtkRigCtrl *ctrl,
                                                   radio_conf_t *conf,
                                                   gint *sock,
                                                   gboolean secondary,
                                                   const gchar *role,
                                                   gboolean *error_reported);
static gboolean rigctld_autodetect_device(GtkRigCtrl *ctrl,
                                          radio_conf_t *conf,
                                          const gchar *role,
                                          gboolean *error_reported);
static gboolean ensure_rigctld_running(GtkRigCtrl *ctrl,
                                       radio_conf_t *conf,
                                       RigctldMgr **mgr,
                                       gboolean secondary,
                                       const gchar *role,
                                       gchar **connect_host,
                                       gboolean *error_reported,
                                       gboolean ic905_force_fallback,
                                       gboolean force_local_recovery);
static gboolean open_rigctld_socket_host(const gchar *host, gint port,
                                         gint *sock,
                                         gint *err_out,
                                         gint *so_err_out,
                                         gboolean log_fail);
static void     schedule_rig_conn_error(GtkRigCtrl *ctrl, radio_conf_t *conf,
                                        const gchar *role);
static void     schedule_rig_autostart_error(GtkRigCtrl *ctrl,
                                             const radio_conf_t *conf,
                                             const gchar *role,
                                             const gchar *detail);
static void     schedule_rig_autodetect_error(GtkRigCtrl *ctrl,
                                              const radio_conf_t *conf,
                                              const gchar *detail);
static void     schedule_rig_missing_model_dialog(GtkRigCtrl *ctrl,
                                                  const radio_conf_t *conf);
static void     schedule_rig_disengage(GtkRigCtrl *ctrl);
static void     rig_engaged_cb(GtkToggleButton * button, gpointer data);
static gboolean radio_apply_ui_settings(GtkRigCtrl *ctrl, gboolean strict);
static gboolean rigctrl_cycle_focus_out_cb(GtkWidget *widget,
                                           GdkEventFocus *event,
                                           gpointer data);
static void     rigctrl_cycle_activate_cb(GtkEntry *entry, gpointer data);
static void     rig_logs_toggle_cb(GtkToggleButton *button, gpointer data);
static void     rig_verbose_toggle_cb(GtkToggleButton *button, gpointer data);
static void     rig_term_log_tx(GtkRigCtrl *ctrl, const gchar *cmd);
static void     rig_term_log_rx(GtkRigCtrl *ctrl, const gchar *reply);
static void     rig_term_log_raw(GtkRigCtrl *ctrl, const gchar *prefix,
                                 const gchar *line, gboolean force);
static void     rig_term_log_verbose(GtkRigCtrl *ctrl,
                                     const gchar *prefix,
                                     const gchar *fmt, ...) G_GNUC_PRINTF(3, 4);
static void     rig_term_log_err_rprt(GtkRigCtrl *ctrl, const gchar *cmd,
                                      const gchar *reply, gint code);
static gboolean rigctrl_log_at_least(const GtkRigCtrl *ctrl,
                                     rig_log_level_t level);
static void     rigctrl_ui_begin_update(GtkRigCtrl *ctrl, const gchar *reason);
static void     rigctrl_ui_end_update(GtkRigCtrl *ctrl, const gchar *reason);
static void     rigctrl_capture_log_closed_height(GtkRigCtrl *ctrl);
static void     rigctrl_restore_log_height(GtkRigCtrl *ctrl);
static void     rigctrl_schedule_trsp_refresh(GtkRigCtrl *ctrl);
static void     rigctrl_set_freq_knob_value(GtkRigCtrl *ctrl,
                                            gboolean uplink,
                                            gdouble value);
static void     rigctrl_set_log_level(GtkRigCtrl *ctrl,
                                      rig_log_level_t level);
static void     rigctrl_sync_log_toggles(GtkRigCtrl *ctrl);
static void     rigctrl_set_user_base_freq(GtkRigCtrl *ctrl,
                                           gboolean downlink,
                                           gint64 hz,
                                           rig_base_source_t src,
                                           gboolean mark_manual);
static void     rigctrl_seed_user_base_from_ui(GtkRigCtrl *ctrl,
                                               const gchar *reason);
static void     rigctrl_reset_send_tracking(GtkRigCtrl *ctrl,
                                            gboolean downlink);
static gint64   rigctrl_get_user_base_freq(GtkRigCtrl *ctrl,
                                           gboolean downlink);
static gint64   rigctrl_knob_to_hz(GtkWidget *knob);
static gint64   rigctrl_get_cached_user_base(const GtkRigCtrl *ctrl,
                                             gboolean downlink);
static void     rigctrl_set_cached_user_base(GtkRigCtrl *ctrl,
                                             gboolean downlink,
                                             gint64 hz);
static gint64   rigctrl_get_cached_doppler(const GtkRigCtrl *ctrl,
                                           gboolean downlink);
static void     rigctrl_set_cached_doppler(GtkRigCtrl *ctrl,
                                           gboolean downlink,
                                           gint64 hz);
static void     rigctrl_reset_doppler_smoothing(GtkRigCtrl *ctrl);
static void     rigctrl_update_doppler(GtkRigCtrl *ctrl);
static void     rigctrl_apply_trsp_preset(GtkRigCtrl *ctrl,
                                          gboolean mark_manual);
static gboolean rigctrl_should_send_freq(GtkRigCtrl *ctrl,
                                         gboolean downlink,
                                         gint64 target_freq_hz);
static void     rigctrl_clear_cached_freq(gint sock,
                                          const gchar *tag,
                                          gint vfo_key);
static void     rigctrl_log_role_op(GtkRigCtrl *ctrl,
                                    gboolean downlink,
                                    vfo_t vfo,
                                    const gchar *op,
                                    const gchar *cmd,
                                    gint64 freq_hz,
                                    gint64 reply_hz,
                                    gboolean have_reply);
static gboolean rigctrl_role_band_limits(GtkRigCtrl *ctrl,
                                         gboolean downlink,
                                         gint64 *min_out,
                                         gint64 *max_out);
static gboolean rigctrl_read_freq_wrong_vfo(GtkRigCtrl *ctrl,
                                            gboolean downlink,
                                            gint64 freq_hz,
                                            gint64 *exp_min_out,
                                            gint64 *exp_max_out,
                                            gint64 *other_min_out,
                                            gint64 *other_max_out);
static void     rigctrl_log_send(GtkRigCtrl *ctrl,
                                 gboolean downlink,
                                 const gchar *vfo_label,
                                 gint64 base_hz,
                                 gint64 doppler_hz,
                                 gint64 target_hz);
static void     rigctrl_log_calc(GtkRigCtrl *ctrl,
                                 gboolean downlink,
                                 gint64 base_hz,
                                 gint64 doppler_hz,
                                 gint64 final_hz,
                                 gboolean doppler_enabled);
static gboolean rigctrl_sat_freq_within_limits(const GtkRigCtrl *ctrl,
                                               gboolean downlink,
                                               gint64 sat_freq);
static void     rig_error_dialog_response(GtkDialog *dialog, gint response_id,
                                          gpointer data);
static void     rigctrl_show_log(GtkRigCtrl *ctrl);
static void     rigctrl_schedule_status(GtkRigCtrl *ctrl,
                                        const gchar *text,
                                        gboolean is_error,
                                        RigUiCommandOutcome outcome);
static gboolean rigctrl_on_main_thread(const GtkRigCtrl *ctrl);
static void     rig_show_warning_dialog(GtkRigCtrl *ctrl,
                                        const gchar *primary,
                                        const gchar *secondary);
static void     rigctrl_warn_shared_uplink_mode(GtkRigCtrl *ctrl,
                                                const radio_conf_t *conf);
static void     rigctrl_register_combo_quarantine_cb(GtkWidget *widget,
                                                     gpointer data);
static void     rigctrl_register_combo_quarantine(GtkComboBox *combo);
static void     rigctrl_queue_ui_status_refresh(GtkRigCtrl *ctrl,
                                                const gchar *reason);
static const gchar *rigctrl_conn_state_name(rigctrl_conn_state_t state);
static void     rigctrl_set_conn_state(GtkRigCtrl *ctrl,
                                       gboolean secondary,
                                       rigctrl_conn_state_t state,
                                       const gchar *reason);
static void     rigctrl_cancel_reconnect(GtkRigCtrl *ctrl, gboolean secondary);
static void     rigctrl_cancel_open_task(GtkRigCtrl *ctrl);
static void     rigctrl_request_open(GtkRigCtrl *ctrl, const gchar *reason);
static gboolean G_GNUC_UNUSED rigctrl_open_idle(gpointer data);
static void     rigctrl_start_open_task(GtkRigCtrl *ctrl);
static void     rigctrl_open_task(GTask *task, gpointer source_object,
                                  gpointer task_data, GCancellable *cancellable);
static void     rigctrl_open_task_done(GObject *source, GAsyncResult *res,
                                       gpointer user_data);
static gboolean rigctrl_collect_thread(GtkRigCtrl *ctrl,
                                       gboolean allow_block,
                                       const gchar *context);
static void     rigctrl_request_close(GtkRigCtrl *ctrl);
static gboolean rigctrl_close_idle(gpointer data);
static void     rigctrl_close_internal(GtkRigCtrl *ctrl);
static void     rigctrl_close_socket_internal(GtkRigCtrl *ctrl,
                                              gboolean secondary);
static gboolean rigctrl_close_socket_idle(gpointer data);
static gboolean rigctrl_configure_trsp_popup_idle(gpointer data);
static void     rigctrl_trsp_combo_realize(GtkWidget *widget, gpointer data);
static void     rigctrl_trsp_popup_show(GtkWidget *widget, gpointer data);
static void     rigctrl_trsp_popup_hide(GtkWidget *widget, gpointer data);
static void     sat_selected_cb(GtkComboBox *satsel, gpointer data);
static gint     rigctrl_trsp_popup_get_max_height(GtkWidget *anchor);
static gint     rigctrl_trsp_tree_row_count(GtkWidget *tree);
static GtkWidget *rigctrl_trsp_find_child(GtkWidget *widget,
                                          GType child_type);
static GtkWidget *rigctrl_trsp_find_scrolled(GtkWidget *widget);
static void     rigctrl_trsp_popup_set_ts(GtkWidget *widget, const gchar *key);
static guint    rigctrl_trsp_popup_bump_seq(GtkWidget *widget);
static guint    rigctrl_trsp_popup_get_seq(GtkWidget *widget);
static gint     rigctld_parse_identifier_pid(const gchar *identifier);
static void     rigctld_set_spawn_state(GtkRigCtrl *ctrl,
                                        gboolean secondary,
                                        RigctldMgr *mgr);
static void     rigctld_clear_spawn_state(GtkRigCtrl *ctrl,
                                          gboolean secondary);
static gboolean rigctld_spawned_by_us(const GtkRigCtrl *ctrl,
                                      gboolean secondary);
static void     rigctld_terminate_spawned(GtkRigCtrl *ctrl,
                                          gboolean secondary,
                                          RigctldMgr **mgr_ptr);
static gboolean rig_parse_rprt_code(const gchar *reply, gint *code_out);
static const gchar *rig_rprt_error_string(gint code);
static void     rigctld_log_cb(RigctldMgr *mgr, const gchar *prefix,
                               const gchar *line, gpointer user_data);
static void     rigctld_persist_device(GtkRigCtrl *ctrl, radio_conf_t *conf,
                                       const gchar *reason);
static gboolean G_GNUC_UNUSED rigctld_try_autodetect_restart(GtkRigCtrl *ctrl,
                                                             radio_conf_t *conf,
                                                             RigctldMgr **mgr,
                                                             gboolean secondary,
                                                             const gchar *role,
                                                             gboolean *reported,
                                                             gint *restart_attempts,
                                                             const gchar *reason);
static gboolean is_full_duplex_main_sub_configured(const radio_conf_t *conf);
static gboolean is_full_duplex_main_sub_active(const GtkRigCtrl *ctrl);
static const gchar *vfo_name(vfo_t vfo);
static const gchar *rigctrl_role_label(gboolean downlink);
static const gchar *rig_base_source_name(rig_base_source_t src);
static void     rigctrl_log_config(GtkRigCtrl *ctrl,
                                   const radio_conf_t *conf,
                                   const gchar *role);
static gboolean rigctrl_validate_mode(GtkRigCtrl *ctrl,
                                      const radio_conf_t *conf,
                                      const gchar *role);
static void     rigctrl_reset_reconnect(GtkRigCtrl *ctrl, gboolean secondary);
static void     rigctrl_reset_link_lost_latch(GtkRigCtrl *ctrl);
static void     rigctrl_schedule_reconnect(GtkRigCtrl *ctrl, gboolean secondary,
                                           const gchar *role);
static void     rigctrl_latch_link_lost(GtkRigCtrl *ctrl,
                                        gboolean secondary,
                                        const gchar *role,
                                        guint attempts);
static void     rigctrl_reset_error_gates(GtkRigCtrl *ctrl);
static void     rigctrl_fail_engage(GtkRigCtrl *ctrl, const gchar *reason);
static void     rigctrl_force_toplevel_resize(GtkRigCtrl *ctrl);
static gboolean rigctrl_resize_idle(gpointer data);
static void     rigctrl_schedule_resize(GtkRigCtrl *ctrl);
static void     rigctrl_handle_socket_error(GtkRigCtrl *ctrl, gint sock,
                                            const gchar *context);
static gboolean rigctrl_should_show_dialog(GHashTable **table_ptr,
                                           const gchar *rig_id);
static gboolean rigctrl_autostart_error_allowed(GtkRigCtrl *ctrl,
                                                const radio_conf_t *conf);
static gboolean rigctrl_missing_model_dialog_allowed(GtkRigCtrl *ctrl,
                                                     const radio_conf_t *conf);
static void     rigctrl_apply_log_level_from_conf(GtkRigCtrl *ctrl,
                                                  const radio_conf_t *conf);
static void     rigctrl_set_editing(GtkRigCtrl *ctrl,
                                    const gchar *rig_id,
                                    gboolean editing);
static gboolean rigctrl_editing_for_role(GtkRigCtrl *ctrl, gboolean secondary);
static radio_conf_t *rigctrl_load_conf(const gchar *rig_id);
static void     rigctrl_apply_conf_update(radio_conf_t *dst,
                                          const radio_conf_t *src);
static void     schedule_rig_backend_error(GtkRigCtrl *ctrl,
                                           radio_conf_t *conf,
                                           const gchar *role);
static void     rigctrl_update_conf_from_disk(GtkRigCtrl *ctrl,
                                              const gchar *rig_id,
                                              const radio_conf_t *updated);
static void     rigctrl_open_radio_config(GtkRigCtrl *ctrl,
                                          const gchar *rig_id);
static void     rigctrl_disable_autostart(GtkRigCtrl *ctrl,
                                          const gchar *rig_id);
static gchar   *rigctrl_combo_get_active_id(GtkComboBox *box,
                                            gboolean allow_none);
static void     rigctrl_combo_set_active_blocked(GtkComboBox *box, gint index,
                                                 GCallback cb, gpointer data);
static void     rigctrl_set_selection_id(GtkRigCtrl *ctrl,
                                         const gchar *label,
                                         gchar **stored_id,
                                         gchar *new_id,
                                         gboolean rebuilt);
static void     rigctrl_rebuild_device_selectors(GtkRigCtrl *ctrl,
                                                 gboolean rebuilt);
static void     rig_term_log(GtkRigCtrl *ctrl, const gchar *prefix,
                             const gchar *fmt, ...);

/*  add thread for hamlib communication */
gpointer        rigctl_run(gpointer data);
static gboolean rigctrl_open_internal(GtkRigCtrl * data);
static void     rigctrl_close(GtkRigCtrl * data);
static void     setconfig(gpointer data);
static void     remove_timer(GtkRigCtrl * data);

static void     start_timer(GtkRigCtrl * data);

void gtk_rig_ctrl_request_close(GtkRigCtrl *ctrl)
{
    if (!IS_GTK_RIG_CTRL(ctrl) || ctrl->destroying)
        return;

    if (ctrl->LockBut != NULL &&
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ctrl->LockBut)))
    {
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->LockBut), FALSE);
    }
    else
    {
        ctrl->engaged = FALSE;
        ctrl->engage_pending = FALSE;
        rigctrl_cancel_open_task(ctrl);
        if (!ctrl->rigctl_thread_done && ctrl->rigctlq != NULL)
            setconfig(ctrl);
    }

    rigctrl_request_close(ctrl);
}

static gboolean rigctrl_collect_thread(GtkRigCtrl *ctrl,
                                       gboolean allow_block,
                                       const gchar *context)
{
    GThread *thread = NULL;

    if (!IS_GTK_RIG_CTRL(ctrl))
        return TRUE;

    thread = ctrl->rigctl_thread;
    if (thread == NULL)
        return TRUE;

#if GLIB_CHECK_VERSION(2, 32, 0)
    if (!allow_block)
    {
        gboolean thread_done = FALSE;

        g_mutex_lock(&ctrl->widgetsync);
        thread_done = ctrl->rigctl_thread_done;
        g_mutex_unlock(&ctrl->widgetsync);

        if (!thread_done)
            return FALSE;

        g_thread_unref(thread);
    }
    else
#endif
    {
        g_thread_join(thread);
    }

    ctrl->rigctl_thread = NULL;
    if (context != NULL && *context != '\0')
        rig_term_log(ctrl, "gpredict", "rigctl thread collected (%s)", context);
    return TRUE;
}

gboolean gtk_rig_ctrl_can_destroy(GtkRigCtrl *ctrl)
{
    if (!IS_GTK_RIG_CTRL(ctrl))
        return TRUE;

    if (ctrl->open_task != NULL || ctrl->open_cancellable != NULL)
        return FALSE;
    if (ctrl->close_pending_id != 0)
        return FALSE;
    if (ctrl->opening || ctrl->opening2)
        return FALSE;
    if (!rigctrl_collect_thread(ctrl, FALSE, "close-poll"))
        return FALSE;

    return TRUE;
}

static void
rig_show_message_dialog(GtkRigCtrl *ctrl,
                        GtkMessageType type,
                        const gchar *primary,
                        const gchar *secondary)
{
    GtkWidget *toplevel;
    GtkWindow *parent = NULL;
    GtkWidget *dialog;

    if (ctrl == NULL)
        return;
    if (ctrl->destroying)
        return;

    toplevel = gtk_widget_get_toplevel(GTK_WIDGET(ctrl));
    if (GTK_IS_WINDOW(toplevel))
        parent = GTK_WINDOW(toplevel);

    dialog =
        gtk_message_dialog_new(parent,
                               GTK_DIALOG_DESTROY_WITH_PARENT,
                               type,
                               GTK_BUTTONS_CLOSE,
                               "%s",
                               primary ? primary : _("Radio error"));

    if (secondary != NULL && *secondary != '\0')
    {
        gtk_message_dialog_format_secondary_text(GTK_MESSAGE_DIALOG(dialog),
                                                 "%s",
                                                 secondary);
    }

    g_signal_connect_swapped(dialog, "response",
                             G_CALLBACK(gtk_widget_destroy), dialog);
    gtk_widget_show(dialog);
}

/* Show a simple error dialog related to radio control */
static void
rig_show_error_dialog(GtkRigCtrl *ctrl,
                      const gchar *primary,
                      const gchar *secondary)
{
    rig_show_message_dialog(ctrl, GTK_MESSAGE_ERROR, primary, secondary);
}

static void
rig_show_warning_dialog(GtkRigCtrl *ctrl,
                        const gchar *primary,
                        const gchar *secondary)
{
    rig_show_message_dialog(ctrl, GTK_MESSAGE_WARNING, primary, secondary);
}

static void rigctrl_warn_shared_uplink_mode(GtkRigCtrl *ctrl,
                                            const radio_conf_t *conf)
{
    const gchar *name;
    gchar *secondary = NULL;

    if (ctrl == NULL)
        return;

    name = (conf != NULL && conf->name != NULL && *conf->name != '\0') ?
        conf->name : _("selected radio");

    if (conf != NULL && conf->radio_mode == RADIO_MODE_SIMPLEX)
    {
        secondary = g_strdup_printf(_("The uplink selector cannot reuse \"%s\" "
                                      "while its Radio mode is Simplex. "
                                      "Set the radio configuration to "
                                      "Full-duplex MAIN/SUB and try again."),
                                    name);
        rig_show_warning_dialog(ctrl,
                                _("Radio is in simplex mode"),
                                secondary);
    }
    else
    {
        secondary = g_strdup_printf(_("The uplink selector cannot reuse \"%s\" "
                                      "unless its Radio mode is set to "
                                      "Full-duplex MAIN/SUB."),
                                    name);
        rig_show_warning_dialog(ctrl,
                                _("Shared uplink requires Full-duplex MAIN/SUB"),
                                secondary);
    }

    g_free(secondary);
}

static void rigctrl_register_combo_quarantine_cb(GtkWidget *widget,
                                                 gpointer data)
{
    GtkWidget *toplevel;

    (void)data;

    if (widget == NULL || !GTK_IS_COMBO_BOX(widget))
        return;

    toplevel = gtk_widget_get_toplevel(widget);
    if (!GTK_IS_WINDOW(toplevel))
        return;

    gp_ui_quarantine_register_combo(toplevel, GTK_COMBO_BOX(widget));
}

static void rigctrl_register_combo_quarantine(GtkComboBox *combo)
{
    GtkWidget *widget;

    if (combo == NULL)
        return;

    widget = GTK_WIDGET(combo);
    rigctrl_register_combo_quarantine_cb(widget, NULL);
    g_signal_connect(widget, "map",
                     G_CALLBACK(rigctrl_register_combo_quarantine_cb), NULL);
}

static void rig_error_dialog_response(GtkDialog *dialog, gint response_id,
                                      gpointer data)
{
    GtkRigCtrl *ctrl = GTK_RIG_CTRL(data);

    if (response_id == RIGCTRL_RESPONSE_SHOW_LOG)
        rigctrl_show_log(ctrl);

    gtk_widget_destroy(GTK_WIDGET(dialog));
}

static void rig_show_error_dialog_with_details(GtkRigCtrl *ctrl,
                                               const gchar *primary,
                                               const gchar *summary,
                                               const gchar *details)
{
    GtkWidget *toplevel;
    GtkWindow *parent = NULL;
    GtkWidget *dialog;
    if (ctrl == NULL)
        return;

    toplevel = gtk_widget_get_toplevel(GTK_WIDGET(ctrl));
    if (GTK_IS_WINDOW(toplevel))
        parent = GTK_WINDOW(toplevel);

    dialog =
        gtk_message_dialog_new(parent,
                               GTK_DIALOG_DESTROY_WITH_PARENT,
                               GTK_MESSAGE_ERROR,
                               GTK_BUTTONS_NONE,
                               "%s",
                               summary ? summary :
                                   (primary ? primary : _("Radio error")));
    if (primary != NULL && *primary != '\0')
        gtk_window_set_title(GTK_WINDOW(dialog), primary);

    if (details != NULL && *details != '\0')
    {
        rig_term_log(ctrl, "gpredict:err", "%s", details);
        sat_log_log(SAT_LOG_LEVEL_ERROR, "%s", details);
    }

    gtk_dialog_add_button(GTK_DIALOG(dialog),
                          _("Show log"),
                          RIGCTRL_RESPONSE_SHOW_LOG);
    gtk_dialog_add_button(GTK_DIALOG(dialog),
                          _("Close"),
                          GTK_RESPONSE_CLOSE);
    g_signal_connect(dialog, "response",
                     G_CALLBACK(rig_error_dialog_response), ctrl);
    gtk_widget_show(dialog);
}

static void free_radio_conf(radio_conf_t *conf)
{
    if (conf == NULL)
        return;

    g_free(conf->name);
    g_free(conf->host);
    g_free(conf->rigctld_path);
    g_free(conf->rigctld_device);
    g_free(conf->rigctld_civaddr);
    g_free(conf->rigctld_extra_args);
    g_free(conf->rigctld_autodetect_match);
    g_free(conf);
}

static gboolean rigctrl_should_show_dialog(GHashTable **table_ptr,
                                           const gchar *rig_id)
{
    /* Track per-radio dialogs so retry loops don't spam popups. */
    if (rig_id == NULL || *rig_id == '\0')
        return TRUE;

    if (*table_ptr == NULL)
        *table_ptr = g_hash_table_new_full(g_str_hash, g_str_equal, g_free, NULL);

    if (g_hash_table_lookup(*table_ptr, rig_id) != NULL)
        return FALSE;

    g_hash_table_insert(*table_ptr, g_strdup(rig_id), GINT_TO_POINTER(1));
    return TRUE;
}

static gboolean rigctrl_autostart_error_allowed(GtkRigCtrl *ctrl,
                                                const radio_conf_t *conf)
{
    if (ctrl == NULL)
        return FALSE;

    return rigctrl_should_show_dialog(&ctrl->autostart_error_reported,
                                      conf ? conf->name : NULL);
}

static gboolean rigctrl_missing_model_dialog_allowed(GtkRigCtrl *ctrl,
                                                     const radio_conf_t *conf)
{
    if (ctrl == NULL)
        return FALSE;

    return rigctrl_should_show_dialog(&ctrl->missing_model_reported,
                                      conf ? conf->name : NULL);
}

static void rigctrl_set_editing(GtkRigCtrl *ctrl,
                                const gchar *rig_id,
                                gboolean editing)
{
    if (ctrl == NULL)
        return;

    if (!editing)
    {
        ctrl->edit_primary = FALSE;
        ctrl->edit_secondary = FALSE;
        return;
    }

    if (rig_id == NULL || *rig_id == '\0')
        return;

    if (ctrl->conf && g_strcmp0(ctrl->conf->name, rig_id) == 0)
        ctrl->edit_primary = editing;

    if (ctrl->conf2 && g_strcmp0(ctrl->conf2->name, rig_id) == 0)
        ctrl->edit_secondary = editing;
}

static gboolean rigctrl_editing_for_role(GtkRigCtrl *ctrl, gboolean secondary)
{
    if (ctrl == NULL)
        return FALSE;

    return secondary ? ctrl->edit_secondary : ctrl->edit_primary;
}

static radio_conf_t *rigctrl_load_conf(const gchar *rig_id)
{
    radio_conf_t *conf;

    if (rig_id == NULL || *rig_id == '\0')
        return NULL;

    conf = g_try_new0(radio_conf_t, 1);
    if (conf == NULL)
        return NULL;

    conf->name = g_strdup(rig_id);
    if (!radio_conf_read(conf))
    {
        free_radio_conf(conf);
        return NULL;
    }

    return conf;
}

static void rigctrl_apply_conf_update(radio_conf_t *dst,
                                      const radio_conf_t *src)
{
    if (dst == NULL || src == NULL)
        return;

    if (g_strcmp0(dst->name, src->name) != 0)
    {
        g_free(dst->name);
        dst->name = g_strdup(src->name);
    }

    g_free(dst->host);
    dst->host = g_strdup(src->host);
    dst->port = src->port;
    dst->cycle = src->cycle;
    dst->lo = src->lo;
    dst->loup = src->loup;
    dst->type = src->type;
    dst->radio_model = src->radio_model;
    dst->radio_mode = src->radio_mode;
    dst->ptt = src->ptt;
    dst->uplink_vfo = src->uplink_vfo;
    dst->downlink_vfo = src->downlink_vfo;
    dst->signal_aos = src->signal_aos;
    dst->signal_los = src->signal_los;
    dst->supports_rit_xit = src->supports_rit_xit;
    dst->supports_full_duplex = src->supports_full_duplex;
    dst->supports_dual_vfo_sat = src->supports_dual_vfo_sat;
    dst->rigctld_autostart = src->rigctld_autostart;
    dst->rigctld_auto_power_on = src->rigctld_auto_power_on;
    g_free(dst->rigctld_path);
    dst->rigctld_path = g_strdup(src->rigctld_path);
    dst->rigctld_model = src->rigctld_model;
    dst->rigctld_conn = src->rigctld_conn;
    g_free(dst->rigctld_device);
    dst->rigctld_device = g_strdup(src->rigctld_device);
    dst->rigctld_baud = src->rigctld_baud;
    g_free(dst->rigctld_civaddr);
    dst->rigctld_civaddr = g_strdup(src->rigctld_civaddr);
    g_free(dst->rigctld_extra_args);
    dst->rigctld_extra_args = g_strdup(src->rigctld_extra_args);
    g_free(dst->rigctld_autodetect_match);
    dst->rigctld_autodetect_match = g_strdup(src->rigctld_autodetect_match);
    dst->rig_log_level = src->rig_log_level;
}

static void rigctrl_update_conf_from_disk(GtkRigCtrl *ctrl,
                                          const gchar *rig_id,
                                          const radio_conf_t *updated)
{
    gboolean renamed = FALSE;

    if (ctrl == NULL || rig_id == NULL || updated == NULL)
        return;

    if (ctrl->conf && g_strcmp0(ctrl->conf->name, rig_id) == 0)
    {
        rigctrl_apply_conf_update(ctrl->conf, updated);
        if (g_strcmp0(rig_id, ctrl->conf->name) != 0)
            renamed = TRUE;
    }

    if (ctrl->conf2 && g_strcmp0(ctrl->conf2->name, rig_id) == 0)
    {
        rigctrl_apply_conf_update(ctrl->conf2, updated);
        if (g_strcmp0(rig_id, ctrl->conf2->name) != 0)
            renamed = TRUE;
    }

    if (renamed && updated->name != NULL)
    {
        if (ctrl->primary_rig_id &&
            g_strcmp0(ctrl->primary_rig_id, rig_id) == 0)
        {
            g_free(ctrl->primary_rig_id);
            ctrl->primary_rig_id = g_strdup(updated->name);
        }
        if (ctrl->secondary_rig_id &&
            g_strcmp0(ctrl->secondary_rig_id, rig_id) == 0)
        {
            g_free(ctrl->secondary_rig_id);
            ctrl->secondary_rig_id = g_strdup(updated->name);
        }
        rigctrl_rebuild_device_selectors(ctrl, TRUE);
    }
}

typedef struct {
    GtkRigCtrl *ctrl;
    gchar      *rig_id;
} RigOpenConfInfo;

static void rigctrl_open_radio_config_done(radio_conf_t *conf,
                                           gboolean applied,
                                           gpointer user_data)
{
    RigOpenConfInfo *info = user_data;

    if (info == NULL)
        return;

    if (info->ctrl)
        rigctrl_set_editing(info->ctrl, info->rig_id, FALSE);

    if (applied && conf != NULL)
    {
        radio_conf_save(conf);
        rigctrl_update_conf_from_disk(info->ctrl, info->rig_id, conf);
    }

    free_radio_conf(conf);
    g_free(info->rig_id);
    g_free(info);
}

static void rigctrl_open_radio_config(GtkRigCtrl *ctrl, const gchar *rig_id)
{
    radio_conf_t *conf;
    RigOpenConfInfo *info;

    if (ctrl == NULL || rig_id == NULL || *rig_id == '\0')
        return;

    conf = rigctrl_load_conf(rig_id);
    if (conf == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: Failed to load radio configuration %s"),
                    __func__, rig_id);
        rig_term_log(ctrl, "gpredict:err",
                     "failed to load radio config %s", rig_id);
        rig_show_error_dialog(ctrl,
                              _("Unable to open radio configuration"),
                              _("Radio configuration could not be loaded."));
        return;
    }

    info = g_new0(RigOpenConfInfo, 1);
    info->ctrl = ctrl;
    info->rig_id = g_strdup(rig_id);

    rigctrl_set_editing(ctrl, rig_id, TRUE);
    rig_term_log(ctrl, "gpredict", "edit radio config %s", rig_id);
    sat_pref_rig_editor_run(conf, rigctrl_open_radio_config_done, info);
}

static void rigctrl_disable_autostart(GtkRigCtrl *ctrl, const gchar *rig_id)
{
    radio_conf_t *conf;

    if (ctrl == NULL || rig_id == NULL || *rig_id == '\0')
        return;

    conf = rigctrl_load_conf(rig_id);
    if (conf == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: Failed to load radio configuration %s"),
                    __func__, rig_id);
        rig_term_log(ctrl, "gpredict:err",
                     "failed to load radio config %s", rig_id);
        rig_show_error_dialog(ctrl,
                              _("Unable to update radio configuration"),
                              _("Radio configuration could not be loaded."));
        return;
    }

    conf->rigctld_autostart = FALSE;
    radio_conf_save(conf);
    rigctrl_update_conf_from_disk(ctrl, rig_id, conf);

    sat_log_log(SAT_LOG_LEVEL_INFO,
                _("%s: Disabled rigctld autostart for %s"),
                __func__, rig_id);
    rig_term_log(ctrl, "gpredict",
                 "disabled rigctld auto-start for %s", rig_id);

    free_radio_conf(conf);
}

static gchar *rig_term_format_timestamp(void)
{
    GDateTime *now = g_date_time_new_now_local();
    gchar *base = NULL;
    gchar *stamp = NULL;
    gint ms = 0;

    if (now == NULL)
        return g_strdup("00:00:00.000");

    base = g_date_time_format(now, "%H:%M:%S");
    ms = g_date_time_get_microsecond(now) / 1000;
    stamp = g_strdup_printf("%s.%03d", base ? base : "00:00:00", ms);
    g_free(base);
    g_date_time_unref(now);
    return stamp;
}

typedef struct {
    GtkRigCtrl *ctrl;
    gchar      *line;
} RigTermLogInfo;

static gboolean rig_term_log_idle(gpointer data)
{
    RigTermLogInfo *info = data;

    if (info == NULL)
        return G_SOURCE_REMOVE;

    if (info->ctrl != NULL && info->ctrl->term_view != NULL && info->line != NULL)
        gp_term_view_log(info->ctrl->term_view, "%s", info->line);

    if (info->ctrl != NULL)
        g_object_unref(info->ctrl);
    g_free(info->line);
    g_free(info);

    return G_SOURCE_REMOVE;
}

static void rig_term_log(GtkRigCtrl *ctrl, const gchar *prefix,
                         const gchar *fmt, ...)
{
    va_list ap;
    gchar *msg = NULL;
    gchar *stamp = NULL;

    if (ctrl == NULL || ctrl->term_view == NULL || prefix == NULL ||
        fmt == NULL)
        return;

    va_start(ap, fmt);
    msg = g_strdup_vprintf(fmt, ap);
    va_end(ap);

    if (msg == NULL)
        return;

    stamp = rig_term_format_timestamp();

    if (!rigctrl_on_main_thread(ctrl))
    {
        RigTermLogInfo *info = g_new0(RigTermLogInfo, 1);

        info->ctrl = g_object_ref(ctrl);
        info->line = g_strdup_printf("%s [%s] %s", stamp, prefix, msg);
        g_idle_add(rig_term_log_idle, info);
        g_free(stamp);
        g_free(msg);
        return;
    }

    gp_term_view_log(ctrl->term_view, "%s [%s] %s", stamp, prefix, msg);
    g_free(stamp);
    g_free(msg);
}

static gboolean rigctrl_log_at_least(const GtkRigCtrl *ctrl,
                                     rig_log_level_t level)
{
    if (ctrl == NULL)
        return FALSE;

    return ctrl->log_level >= level;
}

static void rigctrl_ui_begin_update(GtkRigCtrl *ctrl, const gchar *reason)
{
    if (ctrl == NULL)
        return;

    ctrl->ui_updating = TRUE;
    if (rigctrl_log_at_least(ctrl, RIG_LOG_VERBOSE))
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rigctrl ui_begin_update %s",
                    reason ? reason : "(none)");
}

static void rigctrl_ui_end_update(GtkRigCtrl *ctrl, const gchar *reason)
{
    if (ctrl == NULL)
        return;

    ctrl->ui_updating = FALSE;
    if (rigctrl_log_at_least(ctrl, RIG_LOG_VERBOSE))
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rigctrl ui_end_update %s",
                    reason ? reason : "(none)");
}

typedef struct {
    GtkRigCtrl *ctrl;
    gboolean    uplink;
    gdouble     value;
} RigctrlKnobUpdate;

static gboolean rigctrl_knob_update_idle(gpointer data)
{
    RigctrlKnobUpdate *update = data;
    GtkRigCtrl *ctrl;
    GtkWidget *knob;

    if (update == NULL)
        return G_SOURCE_REMOVE;

    ctrl = update->ctrl;
    knob = NULL;
    if (ctrl != NULL && !ctrl->destroying)
        knob = update->uplink ? ctrl->RigFreqUp : ctrl->RigFreqDown;

    if (knob != NULL)
        gtk_freq_knob_set_value(GTK_FREQ_KNOB(knob), update->value);

    if (ctrl != NULL)
        g_object_unref(ctrl);
    g_free(update);
    return G_SOURCE_REMOVE;
}

static void rigctrl_set_freq_knob_value(GtkRigCtrl *ctrl,
                                        gboolean uplink,
                                        gdouble value)
{
    GtkWidget *knob = NULL;

    if (ctrl == NULL)
        return;

    knob = uplink ? ctrl->RigFreqUp : ctrl->RigFreqDown;
    if (knob == NULL)
        return;

    if (rigctrl_on_main_thread(ctrl))
    {
        gtk_freq_knob_set_value(GTK_FREQ_KNOB(knob), value);
        return;
    }

    if (rigctrl_log_at_least(ctrl, RIG_LOG_VERBOSE))
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rigctrl marshal freq knob %s value=%.0f",
                    uplink ? "uplink" : "downlink",
                    value);

    {
        RigctrlKnobUpdate *update = g_new0(RigctrlKnobUpdate, 1);

        update->ctrl = g_object_ref(ctrl);
        update->uplink = uplink;
        update->value = value;
        g_main_context_invoke(NULL, rigctrl_knob_update_idle, update);
    }
}

static void rig_term_log_raw(GtkRigCtrl *ctrl, const gchar *prefix,
                             const gchar *line, gboolean force)
{
    gchar *trim;

    if (ctrl == NULL || line == NULL || prefix == NULL)
        return;

    if (!force && !rigctrl_log_at_least(ctrl, RIG_LOG_VERBOSE))
        return;

    trim = g_strdup(line);
    g_strchomp(trim);
    g_strstrip(trim);
    if (*trim != '\0')
        rig_term_log(ctrl, prefix, "%s", trim);
    g_free(trim);
}

static void rig_term_log_tx(GtkRigCtrl *ctrl, const gchar *cmd)
{
    if (ctrl == NULL || cmd == NULL)
        return;

    rig_term_log_raw(ctrl, "gpredict:tx", cmd, FALSE);
}

static void rig_term_log_rx(GtkRigCtrl *ctrl, const gchar *reply)
{
    if (ctrl == NULL || reply == NULL)
        return;

    rig_term_log_raw(ctrl, "gpredict:rx", reply, FALSE);
}

static void rig_term_log_verbose(GtkRigCtrl *ctrl,
                                 const gchar *prefix,
                                 const gchar *fmt, ...)
{
    va_list ap;
    gchar *msg = NULL;

    if (ctrl == NULL || fmt == NULL || prefix == NULL)
        return;

    if (!rigctrl_log_at_least(ctrl, RIG_LOG_VERBOSE))
        return;

    va_start(ap, fmt);
    msg = g_strdup_vprintf(fmt, ap);
    va_end(ap);

    if (msg == NULL)
        return;

    rig_term_log(ctrl, prefix, "%s", msg);
    g_free(msg);
}

static const gchar *rig_rprt_error_string(gint code)
{
    switch (code)
    {
    case -1:
        return "Invalid parameter";
    case -2:
        return "Configuration error";
    case -3:
        return "Out of memory";
    case -4:
        return "Function not implemented";
    case -5:
        return "Timeout";
    case -6:
        return "I/O error";
    case -7:
        return "Internal error";
    case -8:
        return "Protocol error";
    case -9:
        return "Command rejected";
    case -10:
        return "Truncated response";
    case -11:
        return "Unavailable";
    default:
        return "Hamlib error";
    }
}

static gboolean G_GNUC_UNUSED rig_parse_rprt_code(const gchar *reply, gint *code_out)
{
    const gchar *start = reply;
    gchar *endp = NULL;
    glong code;

    if (reply == NULL)
        return FALSE;

    if (!g_str_has_prefix(reply, "RPRT"))
        return FALSE;

    start += 4;
    while (*start == ' ')
        start++;

    code = g_ascii_strtoll(start, &endp, 10);
    if (endp == start)
        return FALSE;

    if (code_out)
        *code_out = (gint) code;
    return TRUE;
}

static gboolean rig_parse_rprt_code_any(const gchar *reply, gint *code_out)
{
    const gchar *line = reply;

    if (reply == NULL)
        return FALSE;

    while (*line != '\0')
    {
        const gchar *eol = strchr(line, '\n');
        const gchar *scan = line;
        const gchar *line_end = eol ? eol : line + strlen(line);

        while (scan < line_end && g_ascii_isspace(*scan))
            scan++;

        if ((line_end - scan) >= 4 && g_str_has_prefix(scan, "RPRT"))
        {
            const gchar *start = scan + 4;
            gchar *endp = NULL;
            glong code;

            while (*start == ' ')
                start++;

            code = g_ascii_strtoll(start, &endp, 10);
            if (endp == start)
                return FALSE;

            if (code_out)
                *code_out = (gint) code;
            return TRUE;
        }

        if (eol == NULL)
            break;
        line = eol + 1;
    }

    return FALSE;
}

static gboolean G_GNUC_UNUSED rigctld_reply_indicates_stale_device(const gchar *reply)
{
    gint code = 0;

    if (reply == NULL)
        return FALSE;

    if (rig_parse_rprt_code_any(reply, &code) && code == -6)
        return TRUE;

    if (g_strrstr(reply, "No such file or directory") != NULL)
        return TRUE;
    if (g_strrstr(reply, "no such file or directory") != NULL)
        return TRUE;

    return FALSE;
}

static gboolean rigctld_log_tail_indicates_stale_device(const gchar *log_tail)
{
    gchar *lower = NULL;
    gboolean match = FALSE;

    if (log_tail == NULL || *log_tail == '\0')
        return FALSE;

    lower = g_ascii_strdown(log_tail, -1);
    if (g_strrstr(lower, "no such file or directory") != NULL ||
        g_strrstr(lower, "status=-6") != NULL ||
        g_strrstr(lower, "port_open") != NULL ||
        g_strrstr(lower, "rig_open") != NULL ||
        g_strrstr(lower, "file not found") != NULL ||
        g_strrstr(lower, "cannot find the file") != NULL)
    {
        match = TRUE;
    }

    g_free(lower);
    return match;
}

static void rig_term_log_err_rprt(GtkRigCtrl *ctrl, const gchar *cmd,
                                  const gchar *reply, gint code)
{
    gchar *trim_cmd;
    gchar *trim_reply;

    if (ctrl == NULL || cmd == NULL)
        return;

    trim_cmd = g_strdup(cmd);
    g_strchomp(trim_cmd);
    g_strstrip(trim_cmd);
    if (*trim_cmd == '\0')
    {
        g_free(trim_cmd);
        return;
    }

    trim_reply = g_strdup(reply ? reply : "");
    g_strchomp(trim_reply);
    g_strstrip(trim_reply);
    rig_term_log(ctrl, "gpredict:err", "cmd=%s reply=%s (%s %d)",
                 trim_cmd,
                 *trim_reply ? trim_reply : "(empty)",
                 rig_rprt_error_string(code), code);
    g_free(trim_cmd);
    g_free(trim_reply);
}

static void rigctld_log_cb(RigctldMgr *mgr, const gchar *prefix,
                           const gchar *line, gpointer user_data)
{
    GtkRigCtrl *ctrl = GTK_RIG_CTRL(user_data);

    (void)mgr;

    if (ctrl == NULL || line == NULL || prefix == NULL)
        return;

    if (!rigctrl_log_at_least(ctrl, RIG_LOG_TRACE))
        return;

    rig_term_log(ctrl, prefix, "%s", line);
}

static void rig_logs_toggle_cb(GtkToggleButton *button, gpointer data)
{
    GtkRigCtrl *ctrl = GTK_RIG_CTRL(data);
    gboolean visible;

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    if (ctrl == NULL || ctrl->term_view == NULL)
        return;

    visible = gtk_toggle_button_get_active(button);
    if (visible)
        rigctrl_capture_log_closed_height(ctrl);

    gp_term_view_set_visible(ctrl->term_view, visible);

    if (visible)
        rigctrl_schedule_resize(ctrl);
    else
        rigctrl_restore_log_height(ctrl);
}

static void rigctrl_set_log_level(GtkRigCtrl *ctrl, rig_log_level_t level)
{
    if (ctrl == NULL)
        return;

    if (level < RIG_LOG_QUIET)
        level = RIG_LOG_QUIET;
    if (level > RIG_LOG_TRACE)
        level = RIG_LOG_TRACE;

    if (ctrl->log_level == level)
        return;

    ctrl->log_level = level;
    rigctld_client_set_log_level(level);
    if (ctrl->conf)
        ctrl->conf->rig_log_level = level;
    if (ctrl->conf2)
        ctrl->conf2->rig_log_level = level;
}

static void rigctrl_sync_log_toggles(GtkRigCtrl *ctrl)
{
    gboolean verbose = FALSE;

    if (ctrl == NULL)
        return;

    verbose = rigctrl_log_at_least(ctrl, RIG_LOG_VERBOSE);

    if (ctrl->log_verbose_toggle != NULL)
    {
        g_signal_handlers_block_by_func(ctrl->log_verbose_toggle,
                                        (gpointer)rig_verbose_toggle_cb,
                                        ctrl);
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->log_verbose_toggle),
                                     verbose);
        g_signal_handlers_unblock_by_func(ctrl->log_verbose_toggle,
                                          (gpointer)rig_verbose_toggle_cb,
                                          ctrl);
    }
}

static void rig_verbose_toggle_cb(GtkToggleButton *button, gpointer data)
{
    GtkRigCtrl *ctrl = GTK_RIG_CTRL(data);
    gboolean verbose;

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    verbose = gtk_toggle_button_get_active(button);
    rigctrl_set_log_level(ctrl,
                          verbose ? RIG_LOG_VERBOSE : RIG_LOG_QUIET);
}

static void rigctrl_apply_log_level_from_conf(GtkRigCtrl *ctrl,
                                              const radio_conf_t *conf)
{
    rig_log_level_t level = RIG_LOG_QUIET;

    if (ctrl == NULL)
        return;

    if (conf != NULL)
        level = conf->rig_log_level;

    if (level == RIG_LOG_TRACE)
        level = RIG_LOG_VERBOSE;

    rigctrl_set_log_level(ctrl, level);
    rigctrl_sync_log_toggles(ctrl);
}

static void rigctrl_force_toplevel_resize(GtkRigCtrl *ctrl)
{
    GtkWidget *toplevel;
    GtkRequisition min_req;
    gint cur_w = 0;
    gint cur_h = 0;
    gint new_w = 0;
    gint new_h = 0;

    if (ctrl == NULL)
        return;

    toplevel = gtk_widget_get_toplevel(GTK_WIDGET(ctrl));
    if (!GTK_IS_WINDOW(toplevel))
        return;

    gtk_window_get_size(GTK_WINDOW(toplevel), &cur_w, &cur_h);
    gtk_widget_get_preferred_size(GTK_WIDGET(ctrl), &min_req, NULL);

    new_w = cur_w;
    new_h = cur_h;

    if (new_w < min_req.width)
        new_w = min_req.width;
    if (new_h < min_req.height)
        new_h = min_req.height;

    if (new_w != cur_w || new_h != cur_h)
        gtk_window_resize(GTK_WINDOW(toplevel), new_w, new_h);

    gtk_widget_queue_resize(GTK_WIDGET(ctrl));
}

static void rigctrl_capture_log_closed_height(GtkRigCtrl *ctrl)
{
    GtkWidget *toplevel;
    gint cur_w = 0;
    gint cur_h = 0;

    if (ctrl == NULL)
        return;

    toplevel = gtk_widget_get_toplevel(GTK_WIDGET(ctrl));
    if (!GTK_IS_WINDOW(toplevel))
        return;

    gtk_window_get_size(GTK_WINDOW(toplevel), &cur_w, &cur_h);
    ctrl->log_closed_height = cur_h;
}

static void rigctrl_restore_log_height(GtkRigCtrl *ctrl)
{
    GtkWidget *toplevel;
    GtkRequisition min_req;
    GtkRequisition nat_req;
    gint cur_w = 0;
    gint cur_h = 0;
    gint new_h = 0;
    gint new_w = 0;

    if (ctrl == NULL)
        return;

    if (ctrl->log_closed_height <= 0)
        return;

    toplevel = gtk_widget_get_toplevel(GTK_WIDGET(ctrl));
    if (!GTK_IS_WINDOW(toplevel))
        return;

    gtk_window_get_size(GTK_WINDOW(toplevel), &cur_w, &cur_h);
    gtk_widget_get_preferred_size(GTK_WIDGET(ctrl), &min_req, &nat_req);

    new_w = cur_w;
    new_h = ctrl->log_closed_height;

    if (new_w < min_req.width)
        new_w = min_req.width;
    if (new_h < min_req.height)
        new_h = min_req.height;

    if (new_w != cur_w || new_h != cur_h)
        gtk_window_resize(GTK_WINDOW(toplevel), new_w, new_h);

    gtk_widget_queue_resize(GTK_WIDGET(ctrl));
}

static gboolean rigctrl_resize_idle(gpointer data)
{
    GtkRigCtrl *ctrl = GTK_RIG_CTRL(data);

    if (ctrl == NULL)
        return G_SOURCE_REMOVE;

    ctrl->resize_idle_id = 0;
    rigctrl_force_toplevel_resize(ctrl);
    return G_SOURCE_REMOVE;
}

static void rigctrl_schedule_resize(GtkRigCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    if (ctrl->resize_idle_id != 0)
        return;

    ctrl->resize_idle_id =
        g_idle_add_full(G_PRIORITY_LOW, rigctrl_resize_idle, ctrl, NULL);
}

static const gchar *rigctrl_id_for_log(const gchar *id)
{
    return id ? id : "(none)";
}

static void rigctrl_log_selection_change(const gchar *label,
                                         const gchar *prev_id,
                                         const gchar *new_id,
                                         gboolean rebuilt)
{
    if (g_strcmp0(prev_id, new_id) == 0)
        return;

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "RIGCTRL: %s selection %s -> %s (rebuild=%s)",
                label ? label : "rig",
                rigctrl_id_for_log(prev_id),
                rigctrl_id_for_log(new_id),
                rebuilt ? "yes" : "no");
}

static gchar *rigctrl_combo_get_active_id(GtkComboBox *box,
                                          gboolean allow_none)
{
    gint active;

    if (box == NULL)
        return NULL;

    active = gtk_combo_box_get_active(box);
    if (active < 0)
        return NULL;

    if (allow_none && active == 0)
        return NULL;

    return gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(box));
}

static gboolean rigctrl_combo_popup_shown(GtkComboBox *box)
{
    return gp_ui_combo_popup_shown(box);
}


typedef struct
{
    GtkComboBox *box;
    gint index;
    GCallback cb;
    gpointer data;
    GtkRigCtrl *ctrl;
} RigCtrlComboUpdate;

static gboolean rigctrl_combo_set_active_idle(gpointer data)
{
    RigCtrlComboUpdate *update = data;
    gboolean was_updating = FALSE;

    if (update == NULL || update->box == NULL)
    {
        g_free(update);
        return G_SOURCE_REMOVE;
    }

    if (rigctrl_combo_popup_shown(update->box))
        return G_SOURCE_CONTINUE;

    if (update->ctrl != NULL)
    {
        if (update->ctrl->destroying)
        {
            g_object_unref(update->box);
            g_object_unref(update->ctrl);
            g_free(update);
            return G_SOURCE_REMOVE;
        }

        was_updating = update->ctrl->ui_updating;
        rigctrl_ui_begin_update(update->ctrl, "combo_deferred");
    }

    g_signal_handlers_block_by_func(update->box, (gpointer)update->cb, update->data);
    gtk_combo_box_set_active(update->box, update->index);
    g_signal_handlers_unblock_by_func(update->box, (gpointer)update->cb, update->data);

    if (update->ctrl != NULL && !was_updating)
        rigctrl_ui_end_update(update->ctrl, "combo_deferred");

    g_object_unref(update->box);
    if (update->ctrl != NULL)
        g_object_unref(update->ctrl);
    g_free(update);
    return G_SOURCE_REMOVE;
}

static void rigctrl_combo_set_active_blocked(GtkComboBox *box, gint index,
                                             GCallback cb, gpointer data)
{
    GtkRigCtrl *ctrl = IS_GTK_RIG_CTRL(data) ? GTK_RIG_CTRL(data) : NULL;

    if (box == NULL)
        return;

    if (ctrl == NULL || !ctrl->ui_updating)
    {
        if (rigctrl_combo_popup_shown(box))
        {
            RigCtrlComboUpdate *update = g_new0(RigCtrlComboUpdate, 1);
            update->box = g_object_ref(box);
            update->index = index;
            update->cb = cb;
            update->data = data;
            if (ctrl != NULL)
                update->ctrl = g_object_ref(ctrl);
            g_idle_add_full(G_PRIORITY_DEFAULT_IDLE,
                            rigctrl_combo_set_active_idle,
                            update,
                            NULL);
            return;
        }
    }

    g_signal_handlers_block_by_func(box, (gpointer)cb, data);
    gtk_combo_box_set_active(box, index);
    g_signal_handlers_unblock_by_func(box, (gpointer)cb, data);
}

static void rigctrl_set_selection_id(GtkRigCtrl *ctrl,
                                     const gchar *label,
                                     gchar **stored_id,
                                     gchar *new_id,
                                     gboolean rebuilt)
{
    const gchar *prev_id;

    (void)ctrl;

    if (stored_id == NULL)
    {
        g_free(new_id);
        return;
    }

    prev_id = *stored_id;
    rigctrl_log_selection_change(label, prev_id, new_id, rebuilt);
    g_free(*stored_id);
    *stored_id = new_id;
}

typedef struct {
    GtkRigCtrl *ctrl;
    gchar      *text;
    gboolean    is_error;
    RigUiCommandOutcome outcome;
} RigStatusInfo;

typedef struct {
    GtkRigCtrl *ctrl;
    gboolean secondary;
} RigctrlCloseSocketInfo;

typedef struct {
    GtkRigCtrl *ctrl;
    gchar      *reason;
} RigUiStatusRefreshInfo;

static gboolean rig_session_state_is_engaging(const RigSession *session)
{
    if (session == NULL)
        return FALSE;

    switch (session->state)
    {
    case RIG_SESSION_STARTING_RIGCTLD:
    case RIG_SESSION_CONNECTING:
    case RIG_SESSION_PROBING:
    case RIG_SESSION_CONFIGURING:
    case RIG_SESSION_RECONNECTING:
        return TRUE;
    default:
        return FALSE;
    }
}

static gboolean rig_session_state_is_degraded(const RigSession *session)
{
    if (session == NULL)
        return FALSE;

    return (session->state == RIG_SESSION_DEGRADED);
}

static void rigctrl_set_status_detail(GtkRigCtrl *ctrl, const gchar *detail)
{
    if (ctrl == NULL)
        return;

    if (detail == NULL || *detail == '\0')
    {
        ctrl->ui_status_detail[0] = '\0';
        return;
    }

    g_strlcpy(ctrl->ui_status_detail, detail, sizeof(ctrl->ui_status_detail));
}

static void rigctrl_set_ui_hard_error(GtkRigCtrl *ctrl, const gchar *reason)
{
    if (ctrl == NULL)
        return;

    ctrl->ui_hard_error = TRUE;
    if (reason == NULL || *reason == '\0')
        ctrl->ui_hard_error_reason[0] = '\0';
    else
        g_strlcpy(ctrl->ui_hard_error_reason, reason,
                  sizeof(ctrl->ui_hard_error_reason));
}

static void rigctrl_clear_ui_hard_error(GtkRigCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    ctrl->ui_hard_error = FALSE;
    ctrl->ui_hard_error_reason[0] = '\0';
}

static void rigctrl_refresh_ui_status(GtkRigCtrl *ctrl, const gchar *reason)
{
    RigUiCommandWindowStats stats = { 0 };
    RigStateSnapshot snap = { 0 };
    RadioUiStatus new_status;
    UiSeverity new_severity;
    UiSeverity prev_severity;
    StatusIndicatorPulseMode pulse_mode;
    gboolean needs_secondary = FALSE;
    gboolean primary_disconnected;
    gboolean secondary_disconnected = FALSE;
    const gchar *detail = NULL;
    gint64 now_us = g_get_monotonic_time();
    const gchar *why = (reason != NULL) ? reason : "update";
    GtkWidget *status_label = NULL;
    GtkWidget *status_indicator = NULL;

    if (ctrl == NULL)
        return;

    rig_ui_command_window_get_stats(&ctrl->ui_cmd_window,
                                    now_us,
                                    RIG_UI_ACTIVE_WINDOW_MS,
                                    &stats);

    needs_secondary = (ctrl->conf2 != NULL);
    primary_disconnected = (ctrl->conn_state == RIGCTRL_CONN_DISCONNECTED ||
                            ctrl->conn_state == RIGCTRL_CONN_DISCONNECTING);
    if (needs_secondary)
    {
        secondary_disconnected =
            (ctrl->conn_state2 == RIGCTRL_CONN_DISCONNECTED ||
             ctrl->conn_state2 == RIGCTRL_CONN_DISCONNECTING);
    }

    snap.control_active = (ctrl->engaged ||
                           ctrl->engage_pending ||
                           ctrl->ui_hard_error ||
                           ctrl->link_lost_latched ||
                           ctrl->link_lost_latched2);
    snap.engaging = (ctrl->engage_pending ||
                     ctrl->opening ||
                     ctrl->opening2 ||
                     ctrl->conn_state == RIGCTRL_CONN_CONNECTING ||
                     (needs_secondary &&
                      ctrl->conn_state2 == RIGCTRL_CONN_CONNECTING) ||
                     rig_session_state_is_engaging(ctrl->rig_session) ||
                     rig_session_state_is_engaging(ctrl->rig_session2));
    snap.hard_error = ctrl->ui_hard_error;
    snap.link_lost = (snap.control_active &&
                      !snap.engaging &&
                      !snap.hard_error &&
                      (ctrl->link_lost_latched ||
                       ctrl->link_lost_latched2 ||
                       primary_disconnected ||
                       secondary_disconnected));
    snap.degraded = (snap.control_active &&
                     (ctrl->verify_degraded_down ||
                      ctrl->verify_degraded_up ||
                      rig_session_state_is_degraded(ctrl->rig_session) ||
                      rig_session_state_is_degraded(ctrl->rig_session2)));
    snap.active_flow = stats.active;
    snap.consecutive_link_failures = stats.consecutive_link_failures;
    snap.link_fail_count_in_window = stats.link_fail_count;
    snap.total_fail_count_in_window = stats.total_fail_count;
    snap.ok_count_in_window = stats.ok_count;

    new_status = radio_compute_ui_status(&snap);

    if (new_status != ctrl->ui_status)
    {
        if (rigctrl_log_at_least(ctrl, RIG_LOG_VERBOSE))
        {
            rig_term_log(ctrl, "gpredict",
                         "rig ui status: %s -> %s (reason=%s, window: ok=%u reject=%u linkfail=%u consec_linkfail=%u active=%d)",
                         radio_ui_status_to_string(ctrl->ui_status),
                         radio_ui_status_to_string(new_status),
                         why,
                         stats.ok_count,
                         stats.reject_count,
                         stats.link_fail_count,
                         stats.consecutive_link_failures,
                         stats.active ? 1 : 0);
        }
        ctrl->ui_status = new_status;
    }

    status_label = ctrl->status_label;
    if (status_label != NULL)
    {
        gtk_label_set_text(GTK_LABEL(status_label),
                           radio_ui_status_to_string(ctrl->ui_status));

        status_indicator = ctrl->status_indicator_widget;
        if (status_indicator != NULL)
        {
            prev_severity = status_indicator_get_severity(status_indicator);
            new_severity = radio_ui_status_to_severity(ctrl->ui_status);
            pulse_mode = radio_ui_status_to_pulse_mode(ctrl->ui_status);
            if (rigctrl_log_at_least(ctrl, RIG_LOG_VERBOSE) &&
                prev_severity != new_severity)
            {
                rig_term_log(ctrl, "gpredict",
                             "status indicator: severity %s -> %s",
                             ui_severity_to_string(prev_severity),
                             ui_severity_to_string(new_severity));
            }
            status_indicator_set_severity(status_indicator, new_severity);
            status_indicator_set_pulse_mode(status_indicator, pulse_mode);
        }

        if (ctrl->ui_hard_error && ctrl->ui_hard_error_reason[0] != '\0')
            detail = ctrl->ui_hard_error_reason;
        else if (ctrl->ui_status_detail[0] != '\0')
            detail = ctrl->ui_status_detail;

        gtk_widget_set_tooltip_text(status_label, detail);
    }
}

static gboolean rigctrl_refresh_ui_status_idle(gpointer data)
{
    RigUiStatusRefreshInfo *info = data;

    if (info == NULL)
        return G_SOURCE_REMOVE;

    rigctrl_refresh_ui_status(info->ctrl, info->reason);
    if (info->ctrl != NULL)
        g_object_unref(info->ctrl);
    g_free(info->reason);
    g_free(info);
    return G_SOURCE_REMOVE;
}

static void rigctrl_queue_ui_status_refresh(GtkRigCtrl *ctrl,
                                            const gchar *reason)
{
    RigUiStatusRefreshInfo *info;

    if (ctrl == NULL)
        return;

    if (rigctrl_on_main_thread(ctrl))
    {
        rigctrl_refresh_ui_status(ctrl, reason);
        return;
    }

    info = g_new0(RigUiStatusRefreshInfo, 1);
    info->ctrl = g_object_ref(ctrl);
    info->reason = g_strdup(reason);
    g_idle_add(rigctrl_refresh_ui_status_idle, info);
}

static gboolean rig_status_idle(gpointer data)
{
    RigStatusInfo *info = data;
    GtkRigCtrl *ctrl;
    gint64 now_us;

    if (info == NULL)
        return G_SOURCE_REMOVE;

    ctrl = info->ctrl;
    if (ctrl != NULL)
    {
        ctrl->cmd_error = info->is_error;
        if (info->is_error)
            rigctrl_set_status_detail(ctrl, info->text);
        else
            rigctrl_set_status_detail(ctrl, NULL);
        now_us = g_get_monotonic_time();
        rig_ui_command_window_record(&ctrl->ui_cmd_window, info->outcome, now_us);
        rigctrl_refresh_ui_status(ctrl, "command outcome");
    }

    g_free(info->text);
    g_free(info);
    return G_SOURCE_REMOVE;
}

static void rigctrl_schedule_status(GtkRigCtrl *ctrl,
                                    const gchar *text,
                                    gboolean is_error,
                                    RigUiCommandOutcome outcome)
{
    RigStatusInfo *info;

    if (ctrl == NULL)
        return;

    info = g_new0(RigStatusInfo, 1);
    info->ctrl = ctrl;
    info->text = g_strdup(text);
    info->is_error = is_error;
    info->outcome = outcome;
    g_idle_add(rig_status_idle, info);
}

static gboolean rigctrl_on_main_thread(const GtkRigCtrl *ctrl)
{
    if (ctrl == NULL || ctrl->main_thread == NULL)
        return FALSE;

    return (g_thread_self() == ctrl->main_thread);
}

static gboolean rigctrl_freq_display_connected(const GtkRigCtrl *ctrl,
                                               gboolean uplink)
{
    if (ctrl == NULL || !ctrl->engaged)
        return FALSE;

    if (uplink)
    {
        if (ctrl->conf2 != NULL)
            return (ctrl->conn_state2 == RIGCTRL_CONN_CONNECTED);
        return (ctrl->conn_state == RIGCTRL_CONN_CONNECTED);
    }

    return (ctrl->conn_state == RIGCTRL_CONN_CONNECTED);
}

static void rigctrl_set_knob_placeholder(GtkFreqKnob *knob, gboolean placeholder)
{
    guint i;

    if (knob == NULL)
        return;

    if (!placeholder)
    {
        gtk_freq_knob_set_value(knob, gtk_freq_knob_get_value(knob));
        return;
    }

    for (i = 0; i < G_N_ELEMENTS(knob->digits); i++)
        gtk_label_set_markup(GTK_LABEL(knob->digits[i]),
                             RIGCTRL_FREQ_PLACEHOLDER_DIGIT);
}

static void rigctrl_update_freq_display(GtkRigCtrl *ctrl)
{
    gboolean down_ok;
    gboolean up_ok;

    if (ctrl == NULL)
        return;

    down_ok = rigctrl_freq_display_connected(ctrl, FALSE);
    up_ok = rigctrl_freq_display_connected(ctrl, TRUE);

    /* Doppler is computed from preset frequency + satellite; independent of rig connectivity. */
    if (ctrl->RigFreqDown)
        rigctrl_set_knob_placeholder(GTK_FREQ_KNOB(ctrl->RigFreqDown), !down_ok);
    if (ctrl->RigFreqUp)
        rigctrl_set_knob_placeholder(GTK_FREQ_KNOB(ctrl->RigFreqUp), !up_ok);
}

static gboolean rigctrl_update_freq_display_idle(gpointer data)
{
    GtkRigCtrl *ctrl = GTK_RIG_CTRL(data);

    if (ctrl != NULL && !ctrl->destroying)
        rigctrl_update_freq_display(ctrl);
    if (ctrl != NULL)
        g_object_unref(ctrl);
    return G_SOURCE_REMOVE;
}

static const gchar *rigctrl_conn_state_name(rigctrl_conn_state_t state)
{
    switch (state)
    {
    case RIGCTRL_CONN_DISCONNECTED:
        return "DISCONNECTED";
    case RIGCTRL_CONN_CONNECTING:
        return "CONNECTING";
    case RIGCTRL_CONN_CONNECTED:
        return "CONNECTED";
    case RIGCTRL_CONN_DISCONNECTING:
        return "DISCONNECTING";
    default:
        return "UNKNOWN";
    }
}

static void rigctrl_set_conn_state(GtkRigCtrl *ctrl,
                                   gboolean secondary,
                                   rigctrl_conn_state_t state,
                                   const gchar *reason)
{
    rigctrl_conn_state_t *state_ptr;
    const gchar *role;
    rigctrl_conn_state_t prev_state;

    if (ctrl == NULL)
        return;

    state_ptr = secondary ? &ctrl->conn_state2 : &ctrl->conn_state;
    prev_state = *state_ptr;
    if (prev_state == state)
        return;

    *state_ptr = state;
    role = secondary ? "uplink" : "receiver";

    sat_log_log(SAT_LOG_LEVEL_INFO,
                "rigctrl %s state %s -> %s (%s)",
                role,
                rigctrl_conn_state_name(prev_state),
                rigctrl_conn_state_name(state),
                reason ? reason : "no reason");
    rig_term_log(ctrl, "gpredict",
                 "rig %s state %s -> %s (%s)",
                 role,
                 rigctrl_conn_state_name(prev_state),
                 rigctrl_conn_state_name(state),
                 reason ? reason : "no reason");

    if (rigctrl_on_main_thread(ctrl))
        rigctrl_update_freq_display(ctrl);
    else
    {
        guint source_id;

        g_object_ref(ctrl);
        source_id = g_idle_add(rigctrl_update_freq_display_idle, ctrl);
        if (source_id == 0)
            g_object_unref(ctrl);
    }

    rigctrl_queue_ui_status_refresh(ctrl, reason);
}

static void G_GNUC_UNUSED rig_show_conn_error(GtkRigCtrl *ctrl,
                                radio_conf_t *conf,
                                const gchar *role)
{
    const gchar    *host = (conf && conf->host) ? conf->host : "(null)";
    gint            port = conf ? conf->port : 0;
    const gchar    *label = (role != NULL) ? role : _("rig");
    gchar          *details;
    const gchar    *summary =
        _("rigctld did not respond. See Rig Log for details.");

    if (ctrl == NULL)
        return;

    details = g_strdup_printf(_("Rig: %s\nHost: %s\nPort: %d"),
                              label, host, port);
    rig_show_error_dialog_with_details(ctrl,
                                       _("Unable to connect to radio"),
                                       summary, details);
    g_free(details);
}

static void rigctrl_cancel_reconnect(GtkRigCtrl *ctrl, gboolean secondary)
{
    guint *source_id;

    if (ctrl == NULL)
        return;

    source_id = secondary ? &ctrl->reconnect_source_id2
                          : &ctrl->reconnect_source_id;

    if (*source_id != 0)
    {
        g_source_remove(*source_id);
        *source_id = 0;
    }
}

typedef struct {
    GtkRigCtrl *ctrl;
    gchar      *primary;
    gchar      *secondary;
} RigLinkLostInfo;

static gboolean rig_link_lost_error_idle(gpointer data)
{
    RigLinkLostInfo *info = data;

    if (info == NULL)
        return G_SOURCE_REMOVE;

    if (info->ctrl != NULL && !info->ctrl->destroying)
    {
        rig_show_error_dialog(info->ctrl,
                              info->primary ? info->primary : _("Radio link lost"),
                              info->secondary ? info->secondary : _("See log for details."));
    }

    g_free(info->primary);
    g_free(info->secondary);
    g_free(info);
    return G_SOURCE_REMOVE;
}

static void rigctrl_schedule_link_lost_popup(GtkRigCtrl *ctrl,
                                             const gchar *role,
                                             guint attempts)
{
    RigLinkLostInfo *info;
    const gchar *label = (role != NULL) ? role : _("radio");

    if (ctrl == NULL)
        return;

    info = g_new0(RigLinkLostInfo, 1);
    info->ctrl = ctrl;
    info->primary = g_strdup(_("Radio link lost"));
    info->secondary = g_strdup_printf(
        _("The %s link failed after %u reconnect attempt(s). "
          "Check power, cable, and rigctld, then press Engage to retry."),
        label, attempts);
    g_idle_add(rig_link_lost_error_idle, info);
}

static void rigctrl_reset_reconnect(GtkRigCtrl *ctrl, gboolean secondary)
{
    if (ctrl == NULL)
        return;

    if (secondary)
    {
        rigctrl_cancel_reconnect(ctrl, TRUE);
        ctrl->reconnect_backoff_ms2 = 0;
        ctrl->reconnect_next_us2 = 0;
        ctrl->reconnect_attempts2 = 0;
        ctrl->tx_conn_error_reported = FALSE;
    }
    else
    {
        rigctrl_cancel_reconnect(ctrl, FALSE);
        ctrl->reconnect_backoff_ms = 0;
        ctrl->reconnect_next_us = 0;
        ctrl->reconnect_attempts = 0;
        ctrl->rx_conn_error_reported = FALSE;
    }
}

static void rigctrl_reset_link_lost_latch(GtkRigCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    ctrl->link_lost_latched = FALSE;
    ctrl->link_lost_latched2 = FALSE;
}

static void rigctrl_latch_link_lost(GtkRigCtrl *ctrl,
                                    gboolean secondary,
                                    const gchar *role,
                                    guint attempts)
{
    gboolean *latched_ptr;
    RigSession *session;
    gchar detail[sizeof(ctrl->ui_status_detail)] = { 0 };
    const gchar *label = (role != NULL) ? role : _("radio");
    gboolean shared_secondary = FALSE;

    if (ctrl == NULL || ctrl->destroying)
        return;

    latched_ptr = secondary ? &ctrl->link_lost_latched2 : &ctrl->link_lost_latched;
    if (*latched_ptr)
        return;
    *latched_ptr = TRUE;

    session = secondary ? ctrl->rig_session2 : ctrl->rig_session;
    if (session != NULL)
    {
        rig_session_set_state(ctrl, session, RIG_SESSION_DEGRADED,
                              "reconnect failed after %u attempt(s)", attempts);
    }

    g_snprintf(detail, sizeof(detail),
               _("%s link lost after %u reconnect attempt(s)"),
               label, attempts);
    rigctrl_set_status_detail(ctrl, detail);

    rig_term_log(ctrl, "gpredict:err", "%s", detail);
    sat_log_log(SAT_LOG_LEVEL_ERROR, "%s", detail);

    rigctrl_cancel_reconnect(ctrl, secondary);
    rigctrl_cancel_open_task(ctrl);
    if (secondary)
        ctrl->opening2 = FALSE;
    else
        ctrl->opening = FALSE;

    if (!secondary && ctrl->conf2 == NULL &&
        is_full_duplex_main_sub_configured(ctrl->conf))
    {
        shared_secondary = TRUE;
        ctrl->link_lost_latched2 = TRUE;
        if (ctrl->rig_session2 != NULL)
        {
            rig_session_set_state(ctrl, ctrl->rig_session2, RIG_SESSION_DEGRADED,
                                  "shared link lost");
        }
    }

    rigctrl_set_conn_state(ctrl, secondary, RIGCTRL_CONN_DISCONNECTED,
                           "retry limit reached");
    if (shared_secondary)
    {
        rigctrl_set_conn_state(ctrl, TRUE, RIGCTRL_CONN_DISCONNECTED,
                               "shared retry limit reached");
    }

    rigctrl_schedule_link_lost_popup(ctrl, label, attempts);
    rigctrl_queue_ui_status_refresh(ctrl, "link lost latched");

    if (ctrl->engaged || ctrl->engage_pending)
        schedule_rig_disengage(ctrl);
}

static void rigctrl_reset_error_gates(GtkRigCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    /* Reset retry/error gating only when the user engages again. */
    rigctrl_reset_reconnect(ctrl, FALSE);
    rigctrl_reset_reconnect(ctrl, TRUE);
    rigctrl_reset_link_lost_latch(ctrl);

    if (ctrl->autostart_error_reported != NULL)
        g_hash_table_remove_all(ctrl->autostart_error_reported);
    if (ctrl->missing_model_reported != NULL)
        g_hash_table_remove_all(ctrl->missing_model_reported);

    ctrl->last_verify_log_down_us = 0;
    ctrl->last_verify_log_up_us = 0;
    ctrl->verify_degraded_down = FALSE;
    ctrl->verify_degraded_up = FALSE;
}

typedef struct {
    GtkRigCtrl *ctrl;
    gboolean    secondary;
    gchar      *role;
} RigctrlReconnectInfo;

static gboolean rigctrl_reconnect_cb(gpointer data)
{
    RigctrlReconnectInfo *info = data;
    GtkRigCtrl *ctrl = info ? info->ctrl : NULL;
    gboolean secondary = info ? info->secondary : FALSE;
    const gchar *role = info && info->role ?
        info->role : (secondary ? "uplink" : "receiver");

    if (ctrl != NULL)
    {
        if (secondary)
            ctrl->reconnect_source_id2 = 0;
        else
            ctrl->reconnect_source_id = 0;

        if (!ctrl->destroying && ctrl->engaged)
        {
            if ((secondary && ctrl->link_lost_latched2) ||
                (!secondary && ctrl->link_lost_latched))
                goto done;

            rigctrl_request_open(ctrl, "reconnect timer");

            if (secondary)
            {
                if (ctrl->conf2 != NULL && ctrl->sock2 < 0 &&
                    !ctrl->opening2)
                    rigctrl_schedule_reconnect(ctrl, TRUE, role);
            }
            else
            {
                if (ctrl->sock < 0 && !ctrl->opening)
                    rigctrl_schedule_reconnect(ctrl, FALSE, role);
            }
        }
    }

done:
    if (ctrl != NULL)
        g_object_unref(ctrl);
    if (info != NULL)
    {
        g_free(info->role);
        g_free(info);
    }

    return G_SOURCE_REMOVE;
}

static void rigctrl_schedule_reconnect(GtkRigCtrl *ctrl, gboolean secondary,
                                       const gchar *role)
{
    gint    backoff;
    gint64  now_us;
    gint   *backoff_ptr;
    gint64 *next_ptr;
    guint  *source_ptr;
    guint  *attempts_ptr;
    guint   attempt = 0;

    if (ctrl == NULL)
        return;

    if (ctrl->destroying || !ctrl->engaged)
        return;

    /* Pause retry attempts while the config editor is open. */
    if (rigctrl_editing_for_role(ctrl, secondary))
        return;

    if (!secondary && ctrl->conf == NULL)
        return;
    if (secondary && ctrl->conf2 == NULL)
        return;
    if ((secondary && ctrl->link_lost_latched2) ||
        (!secondary && ctrl->link_lost_latched))
        return;

    backoff_ptr = secondary ? &ctrl->reconnect_backoff_ms2
                            : &ctrl->reconnect_backoff_ms;
    next_ptr = secondary ? &ctrl->reconnect_next_us2
                         : &ctrl->reconnect_next_us;
    source_ptr = secondary ? &ctrl->reconnect_source_id2
                           : &ctrl->reconnect_source_id;
    attempts_ptr = secondary ? &ctrl->reconnect_attempts2
                             : &ctrl->reconnect_attempts;

    if (*source_ptr != 0)
        return;

    if (*attempts_ptr >= RIGCTRL_RECONNECT_MAX_ATTEMPTS)
    {
        rigctrl_latch_link_lost(ctrl, secondary, role, *attempts_ptr);
        return;
    }

    (*attempts_ptr)++;
    attempt = *attempts_ptr;

    if (*backoff_ptr <= 0)
        backoff = RIGCTRL_RECONNECT_BACKOFF_MIN_MS;
    else
        backoff = MIN(*backoff_ptr * 2, RIGCTRL_RECONNECT_BACKOFF_MAX_MS);

    *backoff_ptr = backoff;
    now_us = g_get_monotonic_time();
    *next_ptr = now_us + ((gint64) backoff * 1000);

    sat_log_log(SAT_LOG_LEVEL_INFO,
                _("%s: scheduling %s reconnect attempt %u/%u in %d ms"),
                __func__, role ? role : _("rig"),
                attempt, (guint)RIGCTRL_RECONNECT_MAX_ATTEMPTS, backoff);
    rig_term_log(ctrl, "gpredict",
                 "reconnect %s attempt %u/%u in %d ms",
                 role ? role : "rig",
                 attempt, (guint)RIGCTRL_RECONNECT_MAX_ATTEMPTS, backoff);

    {
        RigSession *session = secondary ? ctrl->rig_session2 : ctrl->rig_session;

        rig_session_set_state(ctrl, session, RIG_SESSION_RECONNECTING,
                              "backoff %d ms attempt %u/%u",
                              backoff,
                              attempt,
                              (guint)RIGCTRL_RECONNECT_MAX_ATTEMPTS);
    }

    {
        RigctrlReconnectInfo *info = g_new0(RigctrlReconnectInfo, 1);

        info->ctrl = g_object_ref(ctrl);
        info->secondary = secondary;
        info->role = g_strdup(role);

        *source_ptr = g_timeout_add(backoff, rigctrl_reconnect_cb, info);
        if (*source_ptr == 0)
        {
            g_object_unref(info->ctrl);
            g_free(info->role);
            g_free(info);
        }
    }
}

static GtkBoxClass *parent_class = NULL;

static void gtk_rig_ctrl_destroy(GtkWidget * widget)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(widget);

    ctrl->destroying = TRUE;
    rigctrl_cancel_open_task(ctrl);
    g_clear_object(&ctrl->open_task);
    g_clear_object(&ctrl->open_cancellable);
    rigctrl_cancel_reconnect(ctrl, FALSE);
    rigctrl_cancel_reconnect(ctrl, TRUE);
    if (ctrl->close_pending_id != 0)
    {
        g_source_remove(ctrl->close_pending_id);
        ctrl->close_pending_id = 0;
    }
    remove_timer(ctrl);
    if (ctrl->pending_ui_refresh_id != 0)
    {
        g_source_remove(ctrl->pending_ui_refresh_id);
        ctrl->pending_ui_refresh_id = 0;
    }

    if (ctrl->rigctl_thread != NULL)
    {
        ctrl->engaged = 0;
        if (ctrl->rigctlq != NULL)
            setconfig(ctrl);
        (void)rigctrl_collect_thread(ctrl, TRUE, "destroy");
    }

    rigctrl_close_internal(ctrl);

    rigctld_terminate_spawned(ctrl, TRUE, &ctrl->rigctld_mgr2);
    rigctld_terminate_spawned(ctrl, FALSE, &ctrl->rigctld_mgr);

    if (ctrl->term_view != NULL)
    {
        gp_term_view_free(ctrl->term_view);
        ctrl->term_view = NULL;
    }
    rigctld_rxbuf_free(&ctrl->rigctld_rxbuf);
    rigctld_rxbuf_free(&ctrl->rigctld_rxbuf2);
    rigctld_client_free(&ctrl->rig_client);
    rigctld_client_free(&ctrl->rig_client2);
    rig_session_free(&ctrl->rig_session);
    rig_session_free(&ctrl->rig_session2);
    g_free(ctrl->rigctld_vfo_main_token);
    g_free(ctrl->rigctld_vfo_sub_token);
    g_free(ctrl->rigctld_vfo_main_token2);
    g_free(ctrl->rigctld_vfo_sub_token2);
    ctrl->rigctld_vfo_main_token = NULL;
    ctrl->rigctld_vfo_sub_token = NULL;
    ctrl->rigctld_vfo_main_token2 = NULL;
    ctrl->rigctld_vfo_sub_token2 = NULL;
    ctrl->rigctld_vfo_map_logged = FALSE;
    ctrl->rigctld_vfo_map_logged2 = FALSE;
    ctrl->log_toggle = NULL;
    ctrl->log_verbose_toggle = NULL;
    ctrl->log_level = RIG_LOG_QUIET;
    if (ctrl->resize_idle_id != 0)
    {
        g_source_remove(ctrl->resize_idle_id);
        ctrl->resize_idle_id = 0;
    }
    if (ctrl->autostart_error_reported != NULL)
    {
        g_hash_table_destroy(ctrl->autostart_error_reported);
        ctrl->autostart_error_reported = NULL;
    }
    if (ctrl->missing_model_reported != NULL)
    {
        g_hash_table_destroy(ctrl->missing_model_reported);
        ctrl->missing_model_reported = NULL;
    }
    g_free(ctrl->primary_rig_id);
    g_free(ctrl->secondary_rig_id);
    ctrl->primary_rig_id = NULL;
    ctrl->secondary_rig_id = NULL;

    if (ctrl->conf != NULL)
    {
        radio_conf_save(ctrl->conf);
        free_radio_conf(ctrl->conf);
        ctrl->conf = NULL;
    }
    if (ctrl->conf2 != NULL)
    {
        free_radio_conf(ctrl->conf2);
        ctrl->conf2 = NULL;
    }

    if (ctrl->trsplist != NULL)
    {
        free_transponders(ctrl->trsplist);
        ctrl->trsplist = NULL;
    }

    (*GTK_WIDGET_CLASS(parent_class)->destroy) (widget);
}

static void gtk_rig_ctrl_class_init(GtkRigCtrlClass * class,
				    gpointer class_data)
{
    GtkWidgetClass *widget_class;

    (void)class_data;

    widget_class = (GtkWidgetClass *) class;
    parent_class = g_type_class_peek_parent(class);
    widget_class->destroy = gtk_rig_ctrl_destroy;
}

static void gtk_rig_ctrl_init(GtkRigCtrl * ctrl,
			      gpointer g_class)
{
    (void)g_class;

    ctrl->sats = NULL;
    ctrl->target = NULL;
    ctrl->pass = NULL;
    ctrl->qth = NULL;
    ctrl->conf = NULL;
    ctrl->conf2 = NULL;
    ctrl->trsp = NULL;
    ctrl->trsplist = NULL;
    ctrl->trsplock = FALSE;
    ctrl->tracking = FALSE;
    ctrl->rx_track_enabled = FALSE;
    ctrl->tx_track_enabled = FALSE;
    ctrl->menu_rx_hz = 0;
    ctrl->menu_tx_hz = 0;
    ctrl->target_radio_rx_hz = 0;
    ctrl->target_radio_tx_hz = 0;
    ctrl->prev_ele = 0.0;
    ctrl->sock = -1;
    ctrl->sock2 = -1;
    ctrl->conn_state = RIGCTRL_CONN_DISCONNECTED;
    ctrl->conn_state2 = RIGCTRL_CONN_DISCONNECTED;
    ctrl->opening = FALSE;
    ctrl->opening2 = FALSE;
    ctrl->open_task = NULL;
    ctrl->open_cancellable = NULL;
    ctrl->reconnect_source_id = 0;
    ctrl->reconnect_source_id2 = 0;
    ctrl->close_pending_id = 0;
    ctrl->pending_close_sock = -1;
    ctrl->pending_close_sock2 = -1;
    ctrl->destroying = FALSE;
    ctrl->main_thread = g_thread_self();
    ctrl->rigctl_thread_done = TRUE;
    ctrl->reconnect_backoff_ms = 0;
    ctrl->reconnect_backoff_ms2 = 0;
    ctrl->reconnect_next_us = 0;
    ctrl->reconnect_next_us2 = 0;
    ctrl->reconnect_attempts = 0;
    ctrl->reconnect_attempts2 = 0;
    ctrl->link_lost_latched = FALSE;
    ctrl->link_lost_latched2 = FALSE;
    ctrl->rx_conn_error_reported = FALSE;
    ctrl->tx_conn_error_reported = FALSE;
    ctrl->edit_primary = FALSE;
    ctrl->edit_secondary = FALSE;
    ctrl->autostart_error_reported = NULL;
    ctrl->missing_model_reported = NULL;
    ctrl->rigctld_mgr = NULL;
    ctrl->rigctld_mgr2 = NULL;
    ctrl->rigctld_spawned = FALSE;
    ctrl->rigctld_spawn_pid = 0;
    ctrl->rigctld_spawned2 = FALSE;
    ctrl->rigctld_spawn_pid2 = 0;
    ctrl->rigctld_rxbuf = NULL;
    ctrl->rigctld_rxbuf2 = NULL;
    ctrl->rig_client = rigctld_client_new(_("receiver"));
    ctrl->rig_client2 = rigctld_client_new(_("uplink"));
    ctrl->rig_session = rig_session_new(_("receiver"));
    ctrl->rig_session2 = rig_session_new(_("uplink"));
    ctrl->rigctld_vfo_main_token = NULL;
    ctrl->rigctld_vfo_sub_token = NULL;
    ctrl->rigctld_vfo_main_token2 = NULL;
    ctrl->rigctld_vfo_sub_token2 = NULL;
    ctrl->rigctld_vfo_map_logged = FALSE;
    ctrl->rigctld_vfo_map_logged2 = FALSE;
    ctrl->term_view = gp_term_view_new(_("Follow tail"), TRUE, FALSE);
    ctrl->log_toggle = NULL;
    ctrl->log_verbose_toggle = NULL;
    ctrl->log_closed_height = 0;
    ctrl->log_level = RIG_LOG_QUIET;
    rigctld_client_set_log_level(ctrl->log_level);
    ctrl->ui_updating = FALSE;
    ctrl->pending_ui_refresh_id = 0;
    ctrl->resize_idle_id = 0;
    ctrl->primary_rig_id = NULL;
    ctrl->secondary_rig_id = NULL;
    ctrl->status_label = NULL;
    ctrl->status_indicator_widget = NULL;
    ctrl->cmd_error = FALSE;
    ctrl->ui_status = RADIO_UI_STATUS_DISENGAGED;
    ctrl->ui_hard_error = FALSE;
    ctrl->ui_hard_error_reason[0] = '\0';
    ctrl->ui_status_detail[0] = '\0';
    rig_ui_command_window_init(&ctrl->ui_cmd_window);
    g_mutex_init(&(ctrl->busy));
    g_mutex_init(&ctrl->freq_cache_lock);
    ctrl->engaged = FALSE;
    ctrl->engage_pending = FALSE;
    ctrl->delay = 1000;
    ctrl->timerid = 0;
    ctrl->errcnt = 0;
    ctrl->lastrxptt = FALSE;
    ctrl->lasttxptt = TRUE;
    ctrl->lastrxf = 0;
    ctrl->lasttxf = 0;
    memset(&ctrl->payload_profile, 0, sizeof(ctrl->payload_profile));
    ctrl->payload_profile_valid = FALSE;
    ctrl->payload_profile_logged = FALSE;
    ctrl->doppler_down_ema = 0.0;
    ctrl->doppler_up_ema = 0.0;
    ctrl->doppler_ema_valid = FALSE;
    ctrl->last_doppler_calc_us = 0;
    ctrl->last_send_down_us = 0;
    ctrl->last_send_up_us = 0;
    ctrl->last_sent_down_hz = 0;
    ctrl->last_sent_up_hz = 0;
    ctrl->last_target_down_hz = 0;
    ctrl->last_target_up_hz = 0;
    ctrl->target_radio_rx_hz = 0;
    ctrl->target_radio_tx_hz = 0;
    ctrl->last_doppler_log_us = 0;
    ctrl->last_probe_log_us = 0;
    ctrl->doppler_suppress_down = 0;
    ctrl->doppler_suppress_up = 0;
    ctrl->user_base_down_hz = 0;
    ctrl->user_base_up_hz = 0;
    ctrl->doppler_down_hz = 0;
    ctrl->doppler_up_hz = 0;
    ctrl->rig_target_down_hz = 0;
    ctrl->rig_target_up_hz = 0;
    ctrl->rig_actual_down_hz = 0;
    ctrl->rig_actual_up_hz = 0;
    ctrl->last_valid_target_down_hz = 0;
    ctrl->last_valid_target_up_hz = 0;
    ctrl->last_send_log_down_us = 0;
    ctrl->last_send_log_up_us = 0;
    ctrl->last_calc_log_down_us = 0;
    ctrl->last_calc_log_up_us = 0;
    ctrl->last_invalid_log_us = 0;
    ctrl->last_verify_log_down_us = 0;
    ctrl->last_verify_log_up_us = 0;
    ctrl->verify_degraded_down = FALSE;
    ctrl->verify_degraded_up = FALSE;
    ctrl->pending_manual_down = FALSE;
    ctrl->pending_manual_up = FALSE;
    ctrl->pending_preset_down = FALSE;
    ctrl->pending_preset_up = FALSE;
    ctrl->user_edit_down = FALSE;
    ctrl->user_edit_up = FALSE;
    ctrl->suppress_user_base = FALSE;
    ctrl->xit_supported = TRUE;
    ctrl->xit_supported2 = TRUE;
    ctrl->last_rit_valid = FALSE;
    ctrl->last_xit_valid = FALSE;
    ctrl->last_xit_valid2 = FALSE;
    ctrl->last_rit_offset = 0;
    ctrl->last_xit_offset = 0;
    ctrl->last_xit_offset2 = 0;
    ctrl->last_toggle_tx = -1;
}

GType gtk_rig_ctrl_get_type(void)
{
    static GType    gtk_rig_ctrl_type = 0;

    if (!gtk_rig_ctrl_type)
    {

        static const GTypeInfo gtk_rig_ctrl_info = {
            sizeof(GtkRigCtrlClass),
            NULL,               /* base_init */
            NULL,               /* base_finalize */
            (GClassInitFunc) gtk_rig_ctrl_class_init,
            NULL,               /* class_finalize */
            NULL,               /* class_data */
            sizeof(GtkRigCtrl),
            2,                  /* n_preallocs */
            (GInstanceInitFunc) gtk_rig_ctrl_init,
            NULL
        };

        gtk_rig_ctrl_type = g_type_register_static(GTK_TYPE_BOX,
                                                   "GtkRigCtrl",
                                                   &gtk_rig_ctrl_info, 0);
    }

    return gtk_rig_ctrl_type;
}


static void update_count_down(GtkRigCtrl * ctrl, gdouble t)
{
    gchar          *buff;

    if (ctrl == NULL || ctrl->SatCnt == NULL || ctrl->target == NULL ||
        !isfinite(t) ||
        !isfinite(ctrl->target->az) ||
        !isfinite(ctrl->target->el) ||
        !isfinite(ctrl->target->aos) ||
        !isfinite(ctrl->target->los) ||
        !isfinite(ctrl->target->range) ||
        !isfinite(ctrl->target->range_rate))
    {
        gtk_label_set_markup(GTK_LABEL(ctrl->SatCnt),
                             "<span size='xx-large'><b>--</b></span>");
        return;
    }

    buff = predict_format_aoslos_countdown(ctrl->target, t, TRUE, TRUE);
    if (buff == NULL)
        buff = g_strdup("<span size='xx-large'><b>--</b></span>");

    gtk_label_set_markup(GTK_LABEL(ctrl->SatCnt), buff);
    g_free(buff);
}

static gboolean rigctrl_target_display_valid(const sat_t *sat)
{
    if (sat == NULL)
        return FALSE;

    if (!isfinite(sat->az) || !isfinite(sat->el) ||
        !isfinite(sat->aos) || !isfinite(sat->los) ||
        !isfinite(sat->range) || !isfinite(sat->range_rate))
    {
        return FALSE;
    }

    if (fabs(sat->az) > RIGCTRL_TARGET_MAX_AZ_DEG ||
        fabs(sat->el) > RIGCTRL_TARGET_MAX_EL_DEG)
    {
        return FALSE;
    }

    if (sat->range < 0.0 || sat->range > RIGCTRL_TARGET_MAX_RANGE_KM)
        return FALSE;

    if (fabs(sat->range_rate) > RIGCTRL_TARGET_MAX_RANGE_RATE_KM_S)
        return FALSE;

    return TRUE;
}

static void rigctrl_set_user_base_freq(GtkRigCtrl *ctrl,
                                       gboolean downlink,
                                       gint64 hz,
                                       rig_base_source_t src,
                                       gboolean mark_manual)
{
    if (ctrl == NULL)
        return;

    if (hz <= 0)
        return;

    rigctrl_set_cached_user_base(ctrl, downlink, hz);
    if (downlink)
        ctrl->menu_rx_hz = hz;
    else
        ctrl->menu_tx_hz = hz;

    {
        RigSession *session = NULL;
        gint idx = downlink ? 0 : 1;

        session = rigctrl_session_for_role(ctrl, downlink);

        if (session != NULL)
        {
            session->user_base_freq_hz[idx] = hz;
            session->user_base_valid[idx] = TRUE;
            if (src != RIG_BASE_SRC_NONE)
                session->base_src[idx] = src;
            session->last_sent_hz[idx] = 0;
            session->last_sent_us[idx] = 0;
            session->last_force_send_us[idx] = 0;
        }
    }

    ctrl->suppress_user_base = TRUE;
    if (downlink)
        gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->SatFreqDown), (gdouble)hz);
    else
        gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->SatFreqUp), (gdouble)hz);
    ctrl->suppress_user_base = FALSE;

    if (src == RIG_BASE_SRC_MANUAL || src == RIG_BASE_SRC_PRESET)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "[rig] base_update role=%s src=%s hz=%" G_GINT64_FORMAT,
                    rigctrl_role_label(downlink),
                    rig_base_source_name(src),
                    hz);
    }

    if (mark_manual)
    {
        if (downlink)
            ctrl->user_edit_down = TRUE;
        else
            ctrl->user_edit_up = TRUE;

        rig_term_log_verbose(ctrl, "gpredict:rx",
                             "manual base update side=%s base=%" G_GINT64_FORMAT,
                             downlink ? "RX" : "TX", hz);
    }
}

static void rigctrl_seed_user_base_from_ui(GtkRigCtrl *ctrl,
                                           const gchar *reason)
{
    gint64 down = 0;
    gint64 up = 0;
    gboolean seeded = FALSE;

    if (ctrl == NULL)
        return;

    if (!rigctrl_on_main_thread(ctrl))
        return;

    if (rigctrl_get_cached_user_base(ctrl, TRUE) <= 0 && ctrl->SatFreqDown)
    {
        down = rigctrl_knob_to_hz(ctrl->SatFreqDown);
        if (down > 0)
        {
            rigctrl_set_user_base_freq(ctrl, TRUE, down, RIG_BASE_SRC_NONE, FALSE);
            seeded = TRUE;
        }
    }

    if (rigctrl_get_cached_user_base(ctrl, FALSE) <= 0 && ctrl->SatFreqUp)
    {
        up = rigctrl_knob_to_hz(ctrl->SatFreqUp);
        if (up > 0)
        {
            rigctrl_set_user_base_freq(ctrl, FALSE, up, RIG_BASE_SRC_NONE, FALSE);
            seeded = TRUE;
        }
    }

    if (seeded)
    {
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rig update: seeded base from UI down=%" G_GINT64_FORMAT
                    " up=%" G_GINT64_FORMAT " reason=%s",
                    rigctrl_get_cached_user_base(ctrl, TRUE),
                    rigctrl_get_cached_user_base(ctrl, FALSE),
                    reason ? reason : "unknown");
    }
}

static void rigctrl_reset_send_tracking(GtkRigCtrl *ctrl,
                                        gboolean downlink)
{
    RigSession *session = NULL;
    gint idx = downlink ? 0 : 1;

    if (ctrl == NULL)
        return;

    session = rigctrl_session_for_role(ctrl, downlink);
    if (session != NULL)
    {
        session->last_sent_hz[idx] = 0;
        session->last_sent_us[idx] = 0;
        session->last_force_send_us[idx] = 0;
        session->last_target_hz[idx] = 0;
    }

    if (downlink)
    {
        ctrl->last_sent_down_hz = 0;
        ctrl->last_send_down_us = 0;
        ctrl->last_target_down_hz = 0;
        ctrl->last_valid_target_down_hz = 0;
        ctrl->target_radio_rx_hz = 0;
    }
    else
    {
        ctrl->last_sent_up_hz = 0;
        ctrl->last_send_up_us = 0;
        ctrl->last_target_up_hz = 0;
        ctrl->last_valid_target_up_hz = 0;
        ctrl->target_radio_tx_hz = 0;
    }
}

static gint64 rigctrl_get_user_base_freq(GtkRigCtrl *ctrl,
                                         gboolean downlink)
{
    RigSession *session = NULL;
    gint idx = downlink ? 0 : 1;
    gint64 value = rigctrl_get_cached_user_base(ctrl, downlink);

    session = rigctrl_session_for_role(ctrl, downlink);
    if (value > 0)
        return value;

    if (session != NULL && session->user_base_valid[idx])
        value = session->user_base_freq_hz[idx];

    return value;
}

static gint64 rigctrl_get_cached_user_base(const GtkRigCtrl *ctrl,
                                           gboolean downlink)
{
    if (ctrl == NULL)
        return 0;

    gint64 value = 0;
    GMutex *lock = (GMutex *)&ctrl->freq_cache_lock;

    g_mutex_lock(lock);
    value = downlink ? ctrl->user_base_down_hz : ctrl->user_base_up_hz;
    g_mutex_unlock(lock);

    return value;
}

static void rigctrl_set_cached_user_base(GtkRigCtrl *ctrl,
                                         gboolean downlink,
                                         gint64 hz)
{
    if (ctrl == NULL)
        return;

    g_mutex_lock(&ctrl->freq_cache_lock);
    if (downlink)
        ctrl->user_base_down_hz = hz;
    else
        ctrl->user_base_up_hz = hz;
    g_mutex_unlock(&ctrl->freq_cache_lock);
}

static gint64 rigctrl_get_cached_doppler(const GtkRigCtrl *ctrl,
                                         gboolean downlink)
{
    if (ctrl == NULL)
        return 0;

    gint64 value = 0;
    GMutex *lock = (GMutex *)&ctrl->freq_cache_lock;

    g_mutex_lock(lock);
    value = downlink ? ctrl->dd : ctrl->du;
    g_mutex_unlock(lock);

    return value;
}

static void rigctrl_set_cached_doppler(GtkRigCtrl *ctrl,
                                       gboolean downlink,
                                       gint64 hz)
{
    if (ctrl == NULL)
        return;

    g_mutex_lock(&ctrl->freq_cache_lock);
    if (downlink)
        ctrl->dd = hz;
    else
        ctrl->du = hz;
    g_mutex_unlock(&ctrl->freq_cache_lock);
}

static void rigctrl_reset_doppler_smoothing(GtkRigCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    if (ctrl->rig_session != NULL)
    {
        ctrl->rig_session->last_sent_hz[0] = 0;
        ctrl->rig_session->last_sent_hz[1] = 0;
        ctrl->rig_session->last_sent_us[0] = 0;
        ctrl->rig_session->last_sent_us[1] = 0;
        ctrl->rig_session->last_force_send_us[0] = 0;
        ctrl->rig_session->last_force_send_us[1] = 0;
        ctrl->rig_session->last_target_hz[0] = 0;
        ctrl->rig_session->last_target_hz[1] = 0;
    }
    if (ctrl->rig_session2 != NULL)
    {
        ctrl->rig_session2->last_sent_hz[0] = 0;
        ctrl->rig_session2->last_sent_hz[1] = 0;
        ctrl->rig_session2->last_sent_us[0] = 0;
        ctrl->rig_session2->last_sent_us[1] = 0;
        ctrl->rig_session2->last_force_send_us[0] = 0;
        ctrl->rig_session2->last_force_send_us[1] = 0;
        ctrl->rig_session2->last_target_hz[0] = 0;
        ctrl->rig_session2->last_target_hz[1] = 0;
    }

    ctrl->doppler_ema_valid = FALSE;
    ctrl->doppler_down_ema = 0.0;
    ctrl->doppler_up_ema = 0.0;
    ctrl->last_doppler_calc_us = 0;
    ctrl->last_send_down_us = 0;
    ctrl->last_send_up_us = 0;
    rigctrl_set_cached_doppler(ctrl, TRUE, 0);
    rigctrl_set_cached_doppler(ctrl, FALSE, 0);
    ctrl->doppler_down_hz = 0;
    ctrl->doppler_up_hz = 0;
    ctrl->last_sent_down_hz = 0;
    ctrl->last_sent_up_hz = 0;
    ctrl->last_target_down_hz = 0;
    ctrl->last_target_up_hz = 0;
    ctrl->last_doppler_log_us = 0;
    ctrl->doppler_suppress_down = 0;
    ctrl->doppler_suppress_up = 0;
    ctrl->last_verify_log_down_us = 0;
    ctrl->last_verify_log_up_us = 0;
    ctrl->verify_degraded_down = FALSE;
    ctrl->verify_degraded_up = FALSE;
}

static void rigctrl_update_doppler(GtkRigCtrl *ctrl)
{
    const gdouble c_mps = 299792458.0;
    gdouble range_rate_mps = 0.0;
    gdouble ratio = 0.0;
    gint64 menu_rx_hz = 0;
    gint64 menu_tx_hz = 0;
    gdouble doppler_rx = 0.0;
    gdouble doppler_tx = 0.0;
    gint64 dd = 0;
    gint64 du = 0;

    if (ctrl == NULL || !rigctrl_target_display_valid(ctrl->target))
    {
        if (ctrl != NULL)
        {
            rigctrl_set_cached_doppler(ctrl, TRUE, 0);
            rigctrl_set_cached_doppler(ctrl, FALSE, 0);
            ctrl->doppler_down_hz = 0;
            ctrl->doppler_up_hz = 0;
        }
        return;
    }

    menu_rx_hz = (ctrl->menu_rx_hz > 0) ?
                     ctrl->menu_rx_hz :
                     rigctrl_get_user_base_freq(ctrl, TRUE);
    menu_tx_hz = (ctrl->menu_tx_hz > 0) ?
                     ctrl->menu_tx_hz :
                     rigctrl_get_user_base_freq(ctrl, FALSE);
    if (ctrl->menu_rx_hz <= 0 && menu_rx_hz > 0)
        ctrl->menu_rx_hz = menu_rx_hz;
    if (ctrl->menu_tx_hz <= 0 && menu_tx_hz > 0)
        ctrl->menu_tx_hz = menu_tx_hz;

    range_rate_mps = ctrl->target->range_rate * 1000.0;
    ratio = range_rate_mps / c_mps;

    /* Range rate is positive when moving away from the observer. */
    /* RX uses minus (arrival), TX uses plus (pre-compensation). */
    if (menu_rx_hz > 0)
        doppler_rx = -ratio * (gdouble)menu_rx_hz;
    if (menu_tx_hz > 0)
        doppler_tx = ratio * (gdouble)menu_tx_hz;

    dd = rigctrl_round_hz(doppler_rx);
    du = rigctrl_round_hz(doppler_tx);

    rigctrl_set_cached_doppler(ctrl, TRUE, dd);
    rigctrl_set_cached_doppler(ctrl, FALSE, du);
    ctrl->doppler_down_hz = dd;
    ctrl->doppler_up_hz = du;
    ctrl->last_doppler_calc_us = g_get_monotonic_time();
}

typedef enum {
    RIGCTRL_SUPPRESS_NONE = 0,
    RIGCTRL_SUPPRESS_STATE,
    RIGCTRL_SUPPRESS_RATE,
    RIGCTRL_SUPPRESS_STEP
} rigctrl_suppress_t;

static gboolean rigctrl_log_throttled(GtkRigCtrl *ctrl,
                                      gint64 *last_us,
                                      gint64 interval_us)
{
    gint64 now_us = 0;

    if (ctrl == NULL || last_us == NULL)
        return FALSE;

    now_us = g_get_monotonic_time();
    if (*last_us > 0 && (now_us - *last_us) < interval_us)
        return FALSE;

    *last_us = now_us;
    return TRUE;
}

static gint64 rigctrl_round_hz(gdouble hz)
{
    if (hz >= (gdouble)G_MAXINT64)
        return G_MAXINT64;
    if (hz <= (gdouble)G_MININT64)
        return G_MININT64;
    return (gint64) llround(hz);
}

static gboolean rigctrl_reject_main_sub_without_vfo_strategy(
    GtkRigCtrl *ctrl, RigSession *session, const radio_conf_t *conf,
    const gchar *reason)
{
    const gchar *strategy = NULL;

    if (!is_full_duplex_main_sub_configured(conf))
        return FALSE;

    if (session != NULL &&
        rig_strategy_supports_explicit_vfo(session->strategy))
        return FALSE;

    if (session != NULL)
        rig_session_set_state(ctrl, session, RIG_SESSION_DEGRADED,
                              "%s",
                              reason ? reason
                                     : "Main/Sub requires rigctld VFO support");

    strategy = session ? rig_strategy_name(session->strategy) : "UNKNOWN";
    sat_log_log(SAT_LOG_LEVEL_WARN,
                "FULL-DUPLEX MAIN/SUB: refusing strategy %s (%s)",
                strategy,
                reason ? reason : "rigctld VFO support required");

    if (rigctrl_log_throttled(ctrl, &ctrl->last_probe_log_us,
                              RIGCTRL_PROBE_LOG_INTERVAL_US))
    {
        rig_term_log(ctrl, "gpredict:err",
                     "Main/Sub requires rigctld VFO support; strategy=%s (try rigctld --vfo)",
                     strategy);
    }

    return TRUE;
}

static gboolean rigctrl_parse_freq_text(const gchar *text, gint64 *hz_out)
{
    GString *normalized = NULL;
    gchar *numeric = NULL;
    gchar *suffix = NULL;
    gchar *endptr = NULL;
    gdouble value = 0.0;
    gdouble scale = 1.0;
    gboolean ok = FALSE;
    gint dot_count = 0;
    gint comma_count = 0;

    if (hz_out)
        *hz_out = 0;

    if (text == NULL)
        return FALSE;

    normalized = g_string_sized_new(strlen(text));
    for (const gchar *p = text; *p != '\0'; p++)
    {
        if (g_ascii_isspace(*p) || *p == '_')
            continue;
        g_string_append_c(normalized, *p);
    }

    if (normalized->len == 0)
    {
        g_string_free(normalized, TRUE);
        return FALSE;
    }

    for (gsize i = 0; i < normalized->len; i++)
    {
        if (normalized->str[i] == '.')
            dot_count++;
        else if (normalized->str[i] == ',')
            comma_count++;
    }

    if ((dot_count > 0 && comma_count > 0) || dot_count > 1 || comma_count > 1)
    {
        g_string_free(normalized, TRUE);
        return FALSE;
    }

    if (comma_count == 1 && dot_count == 0)
    {
        for (gsize i = 0; i < normalized->len; i++)
        {
            if (normalized->str[i] == ',')
                normalized->str[i] = '.';
        }
    }

    {
        gsize len = normalized->len;
        gsize suffix_len = 0;

        while (len > 0 && g_ascii_isalpha(normalized->str[len - 1]))
        {
            suffix_len++;
            len--;
        }

        if (suffix_len > 0)
        {
            suffix = g_ascii_strdown(normalized->str + len, -1);
            numeric = g_strndup(normalized->str, len);
        }
        else
        {
            numeric = g_strdup(normalized->str);
        }
    }

    g_string_free(normalized, TRUE);
    g_strstrip(numeric);
    if (*numeric == '\0')
        goto done;

    value = g_ascii_strtod(numeric, &endptr);
    if (endptr != NULL)
    {
        while (g_ascii_isspace(*endptr))
            endptr++;
    }
    if (endptr == numeric || (endptr != NULL && *endptr != '\0'))
        goto done;

    if (suffix != NULL)
    {
        if (g_strcmp0(suffix, "ghz") == 0)
            scale = 1.0e9;
        else if (g_strcmp0(suffix, "mhz") == 0)
            scale = 1.0e6;
        else if (g_strcmp0(suffix, "khz") == 0)
            scale = 1.0e3;
        else if (g_strcmp0(suffix, "hz") == 0)
            scale = 1.0;
        else
            goto done;
    }
    else if (dot_count == 1 || comma_count == 1)
    {
        scale = 1.0e6;
    }

    if (value <= 0.0)
        goto done;

    if (value > (gdouble)G_MAXINT64 / scale)
    {
        if (hz_out)
            *hz_out = G_MAXINT64;
        goto done;
    }

    if (hz_out)
        *hz_out = rigctrl_round_hz(value * scale);
    ok = TRUE;

done:
    g_free(numeric);
    g_free(suffix);
    return ok;
}

static G_GNUC_UNUSED gint64 parse_ui_freq_to_hz(const gchar *text,
                                                gboolean *ok_out)
{
    gint64 hz = 0;
    gboolean ok = rigctrl_parse_freq_text(text, &hz);

    if (ok_out)
        *ok_out = ok;

    return ok ? hz : 0;
}

static gint64 rigctrl_knob_to_hz(GtkWidget *knob)
{
    if (knob == NULL)
        return 0;
    return rigctrl_round_hz(gtk_freq_knob_get_value(GTK_FREQ_KNOB(knob)));
}

static void hz_to_rigctld_string(gint64 hz, gchar *buf, gsize len)
{
    if (buf == NULL || len == 0)
        return;
    g_snprintf(buf, len, "%" G_GINT64_FORMAT, hz);
}

static gboolean rigctrl_freq_valid_for_send(GtkRigCtrl *ctrl,
                                            gboolean downlink,
                                            gint64 hz)
{
    const gint64 min_hz = 10000;
    const gint64 max_hz = 10000000000LL;
    vfo_t vfo = rigctrl_vfo_for_side(ctrl, downlink);
    const gchar *vfo_label = rigctrl_vfo_label(ctrl, downlink, vfo);

    if (hz >= min_hz && hz <= max_hz)
        return TRUE;

    if (rigctrl_log_throttled(ctrl, &ctrl->last_invalid_log_us,
                              G_USEC_PER_SEC))
    {
        sat_log_log(SAT_LOG_LEVEL_WARN,
                    "rig update: side=%s vfo=%s invalid target_hz=%" G_GINT64_FORMAT,
                    downlink ? "RX" : "TX",
                    vfo_label ? vfo_label : "Unknown",
                    hz);
    }

    return FALSE;
}

static gboolean rigctrl_should_send_freq(GtkRigCtrl *ctrl,
                                         gboolean downlink,
                                         gint64 target_freq_hz)
{
    RigSession *session = NULL;
    gint64 now_us = 0;
    gint64 max_interval_us = 0;
    gint64 last_sent = 0;
    gint64 deadband = 0;
    gint64 last_send_us = 0;
    gint64 last_sent_us = 0;
    gboolean force_pending = FALSE;
    gint idx = downlink ? 0 : 1;
    rigctrl_suppress_t suppress = RIGCTRL_SUPPRESS_NONE;

    if (ctrl == NULL)
        return FALSE;

    if (!rigctrl_freq_valid_for_send(ctrl, downlink, target_freq_hz))
        return FALSE;

    now_us = g_get_monotonic_time();
    max_interval_us = (gint64)RIGCTRL_DOPPLER_MAX_INTERVAL_MS * 1000;

    session = rigctrl_session_for_role(ctrl, downlink);
    if (session != NULL)
        force_pending = (session->last_force_send_us[idx] == 0);
    if (downlink)
    {
        last_sent = ctrl->last_sent_down_hz;
        last_send_us = ctrl->last_send_down_us;
    }
    else
    {
        last_sent = ctrl->last_sent_up_hz;
        last_send_us = ctrl->last_send_up_us;
    }

    if (session != NULL)
    {
        last_sent = session->last_sent_hz[idx];
        last_sent_us = session->last_sent_us[idx];
    }
    else
    {
        last_sent_us = last_send_us;
    }

    deadband = (gint64)RIGCTRL_DOPPLER_MIN_STEP_HZ_DEFAULT;
    if (!force_pending &&
        last_sent > 0 &&
        llabs(target_freq_hz - last_sent) < deadband)
    {
        if (max_interval_us <= 0 ||
            (last_sent_us > 0 &&
             (now_us - last_sent_us) < max_interval_us))
            suppress = RIGCTRL_SUPPRESS_STEP;
    }

    if (suppress == RIGCTRL_SUPPRESS_STEP)
    {
        if (downlink)
            ctrl->doppler_suppress_down = suppress;
        else
            ctrl->doppler_suppress_up = suppress;
        return FALSE;
    }

    if (downlink)
        ctrl->doppler_suppress_down = RIGCTRL_SUPPRESS_NONE;
    else
        ctrl->doppler_suppress_up = RIGCTRL_SUPPRESS_NONE;
    return TRUE;
}

static void rigctrl_log_role_op(GtkRigCtrl *ctrl,
                                gboolean downlink,
                                vfo_t vfo,
                                const gchar *op,
                                const gchar *cmd,
                                gint64 freq_hz,
                                gint64 reply_hz,
                                gboolean have_reply)
{
    if (ctrl == NULL || op == NULL || cmd == NULL)
        return;
    if (!rigctrl_log_at_least(ctrl, RIG_LOG_TRACE))
        return;

    if (have_reply)
    {
        if (freq_hz > 0)
        {
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rig op role=%s vfo=%s op=%s cmd=%s freq=%" G_GINT64_FORMAT
                        " reply=%" G_GINT64_FORMAT,
                        rigctrl_role_label(downlink),
                        vfo_name(vfo),
                        op,
                        cmd,
                        freq_hz,
                        reply_hz);
        }
        else
        {
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rig op role=%s vfo=%s op=%s cmd=%s reply=%" G_GINT64_FORMAT,
                        rigctrl_role_label(downlink),
                        vfo_name(vfo),
                        op,
                        cmd,
                        reply_hz);
        }
    }
    else
    {
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rig op role=%s vfo=%s op=%s cmd=%s freq=%" G_GINT64_FORMAT,
                    rigctrl_role_label(downlink),
                    vfo_name(vfo),
                    op,
                    cmd,
                    freq_hz);
    }
}

static void rigctrl_log_send(GtkRigCtrl *ctrl,
                             gboolean downlink,
                             const gchar *vfo_label,
                             gint64 base_hz,
                             gint64 doppler_hz,
                             gint64 target_hz)
{
    gint64 *last_log_us = NULL;
    RigSession *session = NULL;
    gboolean force_pending = FALSE;
    gint idx = downlink ? 0 : 1;

    if (ctrl == NULL || vfo_label == NULL)
        return;

    last_log_us = downlink ? &ctrl->last_send_log_down_us
                           : &ctrl->last_send_log_up_us;
    if (!rigctrl_log_throttled(ctrl, last_log_us, G_USEC_PER_SEC))
        return;

    sat_log_log(SAT_LOG_LEVEL_INFO,
                "[rig] send role=%s vfo=%s hz=%" G_GINT64_FORMAT,
                rigctrl_role_label(downlink),
                vfo_label,
                target_hz);
    rig_term_log(ctrl, "gpredict",
                 "send role=%s vfo=%s hz=%" G_GINT64_FORMAT,
                 rigctrl_role_label(downlink),
                 vfo_label,
                 target_hz);

    session = rigctrl_session_for_role(ctrl, downlink);
    if (session != NULL)
        force_pending = (session->last_force_send_us[idx] == 0);

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "rig send side=%s vfo=%s base_hz=%" G_GINT64_FORMAT
                " doppler_hz=%" G_GINT64_FORMAT " target_hz=%" G_GINT64_FORMAT
                " force=%d",
                downlink ? "RX" : "TX",
                vfo_label,
                base_hz,
                doppler_hz,
                target_hz,
                force_pending ? 1 : 0);
}

static void rigctrl_log_calc(GtkRigCtrl *ctrl,
                             gboolean downlink,
                             gint64 base_hz,
                             gint64 doppler_hz,
                             gint64 final_hz,
                             gboolean doppler_enabled)
{
    gint64 *last_log_us = NULL;

    if (ctrl == NULL)
        return;
    if (!rigctrl_log_at_least(ctrl, RIG_LOG_VERBOSE))
        return;

    last_log_us = downlink ? &ctrl->last_calc_log_down_us
                           : &ctrl->last_calc_log_up_us;

    if (!rigctrl_log_throttled(ctrl, last_log_us, G_USEC_PER_SEC))
        return;

    sat_log_log(SAT_LOG_LEVEL_INFO,
                "[rig] calc role=%s menu=%" G_GINT64_FORMAT
                " doppler=%" G_GINT64_FORMAT
                " target=%" G_GINT64_FORMAT " track=%d",
                rigctrl_role_label(downlink),
                base_hz,
                doppler_hz,
                final_hz,
                doppler_enabled ? 1 : 0);
}

static gint64 rigctrl_verify_tolerance_hz(const GtkRigCtrl *ctrl)
{
    gint64 tol = (gint64)RIGCTRL_DOPPLER_MIN_STEP_HZ_DEFAULT;

    (void)ctrl;

    if (tol < 1)
        tol = 1;

    return tol;
}

static gboolean rigctrl_role_band_limits(GtkRigCtrl *ctrl,
                                         gboolean downlink,
                                         gint64 *min_out,
                                         gint64 *max_out)
{
    if (min_out)
        *min_out = 0;
    if (max_out)
        *max_out = 0;

    (void)ctrl;
    (void)downlink;

    return FALSE;
}

static gboolean rigctrl_read_freq_wrong_vfo(GtkRigCtrl *ctrl,
                                            gboolean downlink,
                                            gint64 freq_hz,
                                            gint64 *exp_min_out,
                                            gint64 *exp_max_out,
                                            gint64 *other_min_out,
                                            gint64 *other_max_out)
{
    gint64 exp_min = 0;
    gint64 exp_max = 0;
    gint64 other_min = 0;
    gint64 other_max = 0;
    gboolean have_expected = FALSE;
    gboolean have_other = FALSE;

    if (exp_min_out)
        *exp_min_out = 0;
    if (exp_max_out)
        *exp_max_out = 0;
    if (other_min_out)
        *other_min_out = 0;
    if (other_max_out)
        *other_max_out = 0;

    if (ctrl == NULL || freq_hz <= 0)
        return FALSE;

    have_expected = rigctrl_role_band_limits(ctrl, downlink,
                                             &exp_min, &exp_max);
    have_other = rigctrl_role_band_limits(ctrl, !downlink,
                                          &other_min, &other_max);

    if (exp_min_out)
        *exp_min_out = exp_min;
    if (exp_max_out)
        *exp_max_out = exp_max;
    if (other_min_out)
        *other_min_out = other_min;
    if (other_max_out)
        *other_max_out = other_max;

    if (have_expected)
    {
        if (freq_hz >= exp_min && freq_hz <= exp_max)
            return FALSE;
        if (have_other && freq_hz >= other_min && freq_hz <= other_max)
            return TRUE;
    }

    {
        gint64 expected = downlink ? ctrl->rig_target_down_hz
                                   : ctrl->rig_target_up_hz;
        gint64 other = downlink ? ctrl->rig_target_up_hz
                                : ctrl->rig_target_down_hz;
        gint64 tol = rigctrl_verify_tolerance_hz(ctrl);

        if (expected > 0 && other > 0)
        {
            gint64 d_expected = llabs(freq_hz - expected);
            gint64 d_other = llabs(freq_hz - other);

            if (d_other + tol < d_expected)
                return TRUE;
        }
    }

    return FALSE;
}

static void rigctrl_verify_mark_ok(GtkRigCtrl *ctrl, gboolean downlink)
{
    if (ctrl == NULL)
        return;

    if (downlink)
        ctrl->verify_degraded_down = FALSE;
    else
        ctrl->verify_degraded_up = FALSE;
}

static void rigctrl_verify_log_mismatch(GtkRigCtrl *ctrl,
                                        gboolean downlink,
                                        vfo_t vfo,
                                        gint64 target_hz,
                                        gint64 readback_hz,
                                        gint rprt_code)
{
    gint64 *last_log_us = NULL;
    gboolean *degraded = NULL;

    if (ctrl == NULL)
        return;

    last_log_us = downlink ? &ctrl->last_verify_log_down_us
                           : &ctrl->last_verify_log_up_us;
    degraded = downlink ? &ctrl->verify_degraded_down
                        : &ctrl->verify_degraded_up;

    if (*degraded &&
        !rigctrl_log_throttled(ctrl, last_log_us, RIGCTRL_VERIFY_LOG_INTERVAL_US))
        return;

    *degraded = TRUE;

    sat_log_log(SAT_LOG_LEVEL_ERROR,
                "set_freq verify failed side=%s vfo=%s target=%" G_GINT64_FORMAT
                " readback=%" G_GINT64_FORMAT " rprt=%d",
                downlink ? "RX" : "TX",
                vfo_name(vfo),
                target_hz,
                readback_hz,
                rprt_code);
}

static gboolean rigctrl_verify_match(GtkRigCtrl *ctrl,
                                     gboolean downlink,
                                     vfo_t vfo,
                                     gint64 target_hz,
                                     gint64 readback_hz,
                                     gint rprt_code)
{
    gint64 tol = rigctrl_verify_tolerance_hz(ctrl);

    if (llabs(readback_hz - target_hz) <= tol)
    {
        rigctrl_verify_mark_ok(ctrl, downlink);
        return TRUE;
    }

    rigctrl_verify_log_mismatch(ctrl, downlink, vfo, target_hz,
                                readback_hz, rprt_code);
    return FALSE;
}

static void rigctrl_log_doppler_tick(GtkRigCtrl *ctrl)
{
    gint64 rx_base = 0;
    gint64 tx_base = 0;
    gint64 rx_doppler = 0;
    gint64 tx_doppler = 0;
    gint64 now_us = 0;

    if (ctrl == NULL)
        return;

    now_us = g_get_monotonic_time();
    if (ctrl->last_doppler_log_us > 0 &&
        (now_us - ctrl->last_doppler_log_us) < RIGCTRL_DOPPLER_LOG_INTERVAL_US)
        return;

    ctrl->last_doppler_log_us = now_us;

    rx_base = (ctrl->menu_rx_hz > 0) ?
                  ctrl->menu_rx_hz :
                  rigctrl_get_user_base_freq(ctrl, TRUE);
    tx_base = (ctrl->menu_tx_hz > 0) ?
                  ctrl->menu_tx_hz :
                  rigctrl_get_user_base_freq(ctrl, FALSE);
    rx_doppler = rigctrl_get_cached_doppler(ctrl, TRUE);
    tx_doppler = rigctrl_get_cached_doppler(ctrl, FALSE);

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "doppler tick: rx_menu=%" G_GINT64_FORMAT " rx_doppler=%" G_GINT64_FORMAT
                " rx_target=%" G_GINT64_FORMAT " rx_track=%d"
                " tx_menu=%" G_GINT64_FORMAT " tx_doppler=%" G_GINT64_FORMAT
                " tx_target=%" G_GINT64_FORMAT " tx_track=%d",
                rx_base,
                rx_doppler,
                ctrl->target_radio_rx_hz,
                ctrl->rx_track_enabled ? 1 : 0,
                tx_base,
                tx_doppler,
                ctrl->target_radio_tx_hz,
                ctrl->tx_track_enabled ? 1 : 0);
}

static void rigctrl_update_last_sent(GtkRigCtrl *ctrl, gboolean downlink,
                                     gint64 freq_hz)
{
    RigSession *session = NULL;
    gint idx = downlink ? 0 : 1;
    gint64 now_us = 0;

    if (ctrl == NULL)
        return;

    session = rigctrl_session_for_role(ctrl, downlink);
    now_us = g_get_monotonic_time();

    if (session != NULL)
    {
        session->last_sent_hz[idx] = freq_hz;
        session->last_sent_us[idx] = now_us;
        session->last_force_send_us[idx] = now_us;
    }

    if (downlink)
        ctrl->last_sent_down_hz = freq_hz;
    else
        ctrl->last_sent_up_hz = freq_hz;
}

static void rigctrl_update_last_target(GtkRigCtrl *ctrl, gboolean downlink,
                                       gint64 target_hz)
{
    RigSession *session = NULL;
    gint idx = downlink ? 0 : 1;

    if (ctrl == NULL)
        return;

    session = rigctrl_session_for_role(ctrl, downlink);
    if (session != NULL)
        session->last_target_hz[idx] = target_hz;

    if (downlink)
    {
        ctrl->rig_target_down_hz = target_hz;
        ctrl->target_radio_rx_hz = target_hz;
    }
    else
    {
        ctrl->rig_target_up_hz = target_hz;
        ctrl->target_radio_tx_hz = target_hz;
    }
}

static void rigctrl_doppler_tick(GtkRigCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    if (!ctrl->engaged)
        return;

    if (ctrl->conn_state != RIGCTRL_CONN_CONNECTED)
        return;

    rigctrl_update_doppler(ctrl);
}

static gboolean rigctrl_sat_freq_within_limits(const GtkRigCtrl *ctrl,
                                               gboolean downlink,
                                               gint64 sat_freq)
{
    const gint64 min_hz = 10000;
    const gint64 max_hz = 10000000000LL;

    (void)ctrl;
    (void)downlink;

    return (sat_freq >= min_hz && sat_freq <= max_hz);
}

static gint64 rigctrl_adjust_target(GtkRigCtrl *ctrl,
                                    gboolean downlink,
                                    gint64 base_sat,
                                    gint64 doppler,
                                    gint64 target_rig,
                                    gboolean *target_ok_out)
{
    gint64 sat_target = base_sat + doppler;
    gboolean ok = rigctrl_sat_freq_within_limits(ctrl, downlink, sat_target);
    gint64 last_valid = downlink ? ctrl->last_valid_target_down_hz
                                 : ctrl->last_valid_target_up_hz;

    if (!ok)
    {
        sat_log_log(SAT_LOG_LEVEL_WARN,
                    "rig update: side=%s target out of range sat=%" G_GINT64_FORMAT,
                    downlink ? "RX" : "TX", sat_target);
        if (target_ok_out)
            *target_ok_out = FALSE;
        if (last_valid > 0)
            return last_valid;
        return target_rig;
    }

    if (target_ok_out)
        *target_ok_out = TRUE;
    if (downlink)
        ctrl->last_valid_target_down_hz = target_rig;
    else
        ctrl->last_valid_target_up_hz = target_rig;
    return target_rig;
}

static gint64 rigctrl_compute_target_rx(GtkRigCtrl *ctrl,
                                        gdouble lo,
                                        gboolean use_rit_xit,
                                        gint64 *menu_hz_out,
                                        gint64 *doppler_out,
                                        gboolean *target_ok_out)
{
    gdouble base_rig_f = 0.0;
    gint64 menu_hz = 0;
    gint64 base_rig = 0;
    gint64 doppler = 0;
    gint64 target_rig = 0;
    gboolean doppler_enabled = FALSE;
    RigSession *session = NULL;
    gint idx = 0;

    if (ctrl == NULL)
    {
        if (target_ok_out)
            *target_ok_out = FALSE;
        return 0;
    }

    menu_hz = (ctrl->menu_rx_hz > 0) ?
                  ctrl->menu_rx_hz :
                  rigctrl_get_user_base_freq(ctrl, TRUE);
    if (ctrl->menu_rx_hz <= 0 && menu_hz > 0)
        ctrl->menu_rx_hz = menu_hz;
    if (menu_hz <= 0)
    {
        if (target_ok_out)
            *target_ok_out = FALSE;
        if (rigctrl_log_throttled(ctrl, &ctrl->last_invalid_log_us,
                                  G_USEC_PER_SEC))
        {
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rig update: side=RX missing menu frequency");
        }
        session = rigctrl_session_for_role(ctrl, TRUE);
        if (session != NULL)
        {
            session->calc_base_hz[idx] = 0;
            session->calc_doppler_hz[idx] = 0;
            session->calc_final_hz[idx] = 0;
            session->calc_valid[idx] = FALSE;
            session->doppler_enabled[idx] = FALSE;
        }
        return 0;
    }

    doppler_enabled = ctrl->rx_track_enabled && !use_rit_xit;
    if (doppler_enabled)
        doppler = rigctrl_get_cached_doppler(ctrl, TRUE);

    base_rig_f = (gdouble)menu_hz - lo;
    base_rig = rigctrl_round_hz(base_rig_f);
    target_rig = base_rig + doppler;

    if (menu_hz_out)
        *menu_hz_out = menu_hz;
    if (doppler_out)
        *doppler_out = doppler;

    target_rig = rigctrl_adjust_target(ctrl, TRUE, menu_hz,
                                       doppler, target_rig, target_ok_out);
    if (target_rig > 0)
        (void) rigctrl_freq_valid_for_send(ctrl, TRUE, target_rig);

    session = rigctrl_session_for_role(ctrl, TRUE);
    if (session != NULL)
    {
        session->calc_base_hz[idx] = base_rig;
        session->calc_doppler_hz[idx] = doppler;
        session->calc_final_hz[idx] = target_rig;
        session->calc_valid[idx] = (target_ok_out != NULL) ? *target_ok_out : TRUE;
        session->doppler_enabled[idx] = doppler_enabled;
    }

    rigctrl_log_calc(ctrl, TRUE, menu_hz, doppler, target_rig, doppler_enabled);

    return target_rig;
}

static gint64 rigctrl_compute_target_tx(GtkRigCtrl *ctrl,
                                        gdouble lo,
                                        gboolean use_rit_xit,
                                        gint64 *menu_hz_out,
                                        gint64 *doppler_out,
                                        gboolean *target_ok_out)
{
    gdouble base_rig_f = 0.0;
    gint64 menu_hz = 0;
    gint64 base_rig = 0;
    gint64 doppler = 0;
    gint64 target_rig = 0;
    gboolean doppler_enabled = FALSE;
    RigSession *session = NULL;
    gint idx = 1;

    if (ctrl == NULL)
    {
        if (target_ok_out)
            *target_ok_out = FALSE;
        return 0;
    }

    menu_hz = (ctrl->menu_tx_hz > 0) ?
                  ctrl->menu_tx_hz :
                  rigctrl_get_user_base_freq(ctrl, FALSE);
    if (ctrl->menu_tx_hz <= 0 && menu_hz > 0)
        ctrl->menu_tx_hz = menu_hz;
    if (menu_hz <= 0)
    {
        if (target_ok_out)
            *target_ok_out = FALSE;
        if (rigctrl_log_throttled(ctrl, &ctrl->last_invalid_log_us,
                                  G_USEC_PER_SEC))
        {
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rig update: side=TX missing menu frequency");
        }
        session = rigctrl_session_for_role(ctrl, FALSE);
        if (session != NULL)
        {
            session->calc_base_hz[idx] = 0;
            session->calc_doppler_hz[idx] = 0;
            session->calc_final_hz[idx] = 0;
            session->calc_valid[idx] = FALSE;
            session->doppler_enabled[idx] = FALSE;
        }
        return 0;
    }

    doppler_enabled = ctrl->tx_track_enabled && !use_rit_xit;
    if (doppler_enabled)
        doppler = rigctrl_get_cached_doppler(ctrl, FALSE);

    base_rig_f = (gdouble)menu_hz - lo;
    base_rig = rigctrl_round_hz(base_rig_f);
    target_rig = base_rig + doppler;

    if (menu_hz_out)
        *menu_hz_out = menu_hz;
    if (doppler_out)
        *doppler_out = doppler;

    target_rig = rigctrl_adjust_target(ctrl, FALSE, menu_hz,
                                       doppler, target_rig, target_ok_out);
    if (target_rig > 0)
        (void) rigctrl_freq_valid_for_send(ctrl, FALSE, target_rig);

    session = rigctrl_session_for_role(ctrl, FALSE);
    if (session != NULL)
    {
        session->calc_base_hz[idx] = base_rig;
        session->calc_doppler_hz[idx] = doppler;
        session->calc_final_hz[idx] = target_rig;
        session->calc_valid[idx] = (target_ok_out != NULL) ? *target_ok_out : TRUE;
        session->doppler_enabled[idx] = doppler_enabled;
    }

    rigctrl_log_calc(ctrl, FALSE, menu_hz, doppler, target_rig, doppler_enabled);

    return target_rig;
}

static gint64 rigctrl_compute_target(GtkRigCtrl *ctrl,
                                     gboolean downlink,
                                     gdouble lo,
                                     gboolean use_rit_xit,
                                     gint64 *base_sat_out,
                                     gint64 *doppler_out,
                                     gboolean *target_ok_out)
{
    if (downlink)
        return rigctrl_compute_target_rx(ctrl, lo, use_rit_xit,
                                         base_sat_out, doppler_out,
                                         target_ok_out);

    return rigctrl_compute_target_tx(ctrl, lo, use_rit_xit,
                                     base_sat_out, doppler_out,
                                     target_ok_out);
}

static void rigctrl_show_log(GtkRigCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    rigctrl_capture_log_closed_height(ctrl);

    if (ctrl->log_toggle != NULL)
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->log_toggle), TRUE);
    else if (ctrl->term_view != NULL)
        gp_term_view_set_visible(ctrl->term_view, TRUE);

    rigctrl_schedule_resize(ctrl);
}

static void rigctrl_trsp_popup_set_ts(GtkWidget *widget, const gchar *key)
{
    gint64 *ts = NULL;

    if (widget == NULL || key == NULL)
        return;

    ts = g_object_get_data(G_OBJECT(widget), key);
    if (ts == NULL)
    {
        ts = g_new0(gint64, 1);
        g_object_set_data_full(G_OBJECT(widget), key, ts, g_free);
    }
    *ts = g_get_monotonic_time();
}

static guint rigctrl_trsp_popup_bump_seq(GtkWidget *widget)
{
    guint seq = 0;

    if (widget == NULL)
        return 0;

    seq = GPOINTER_TO_UINT(g_object_get_data(G_OBJECT(widget),
                                             "rigctrl-popup-open-seq"));
    seq++;
    g_object_set_data(G_OBJECT(widget), "rigctrl-popup-open-seq",
                      GUINT_TO_POINTER(seq));
    return seq;
}

static guint rigctrl_trsp_popup_get_seq(GtkWidget *widget)
{
    if (widget == NULL)
        return 0;

    return GPOINTER_TO_UINT(g_object_get_data(G_OBJECT(widget),
                                              "rigctrl-popup-open-seq"));
}

static gboolean rigctrl_configure_trsp_popup_idle(gpointer data)
{
    GtkComboBox *combo = GTK_COMBO_BOX(data);
    AtkObject *popup_acc;
    GtkWidget *popup_widget = NULL;
    GtkWidget *scrolled = NULL;
    gint attempt = 0;
    gint anchor_width = 0;
    gint anchor_min = 0;
    gint anchor_nat = 0;

    if (combo == NULL)
        return G_SOURCE_REMOVE;

    attempt = GPOINTER_TO_INT(g_object_get_data(G_OBJECT(combo),
                                                "rigctrl-popup-attempt"));
    popup_acc = gtk_combo_box_get_popup_accessible(combo);
    if (popup_acc != NULL)
        popup_widget = gtk_accessible_get_widget(GTK_ACCESSIBLE(popup_acc));

    if (popup_widget == NULL)
    {
        if (attempt < 4)
        {
            g_object_set_data(G_OBJECT(combo), "rigctrl-popup-attempt",
                              GINT_TO_POINTER(attempt + 1));
            g_timeout_add(25, rigctrl_configure_trsp_popup_idle,
                          g_object_ref(combo));
        }

        g_object_unref(combo);
        return G_SOURCE_REMOVE;
    }

    g_object_set_data(G_OBJECT(combo), "rigctrl-popup-attempt",
                      GINT_TO_POINTER(0));

    if (popup_widget != NULL)
    {
        GtkWidget *tree = NULL;
        gint max_height = rigctrl_trsp_popup_get_max_height(GTK_WIDGET(combo));
        gint list_min = 0;
        gint list_nat = 0;
        gint rows = 0;
#ifndef __APPLE__
        gboolean needs_scroll = FALSE;
#endif
        gint popup_width = -1;
        gint popup_height = -1;
        guint open_seq = 0;
        guint configured_seq = 0;

        gtk_widget_set_hexpand(popup_widget, FALSE);
        gtk_widget_set_vexpand(popup_widget, FALSE);

        open_seq = rigctrl_trsp_popup_get_seq(GTK_WIDGET(combo));
        configured_seq = GPOINTER_TO_UINT(
            g_object_get_data(G_OBJECT(combo), "rigctrl-popup-configured-seq"));
        if (open_seq > 0 && configured_seq == open_seq)
        {
            g_object_unref(combo);
            return G_SOURCE_REMOVE;
        }
        g_object_set_data(G_OBJECT(combo), "rigctrl-popup-configured-seq",
                          GUINT_TO_POINTER(open_seq));

        scrolled = rigctrl_trsp_find_scrolled(popup_widget);

        tree = rigctrl_trsp_find_child(popup_widget, GTK_TYPE_TREE_VIEW);
        if (GTK_IS_TREE_VIEW(tree))
        {
            gtk_widget_get_preferred_height(tree, &list_min, &list_nat);
            if (list_nat <= 0 && list_min > 0)
                list_nat = list_min;
            rows = rigctrl_trsp_tree_row_count(tree);
#ifndef __APPLE__
            if (list_nat > 0)
                needs_scroll = (list_nat > max_height);
            else
                needs_scroll = (rows > RIGCTRL_TRSP_POPUP_SEARCH_THRESHOLD);
#endif
        }
        if (GTK_IS_SCROLLED_WINDOW(scrolled))
        {
            anchor_width = gtk_widget_get_allocated_width(GTK_WIDGET(combo));
            if (anchor_width <= 1)
            {
                gtk_widget_get_preferred_width(GTK_WIDGET(combo),
                                               &anchor_min, &anchor_nat);
                anchor_width = (anchor_nat > 0) ? anchor_nat : anchor_min;
            }
            if (anchor_width > 0)
                popup_width = anchor_width;

            gtk_widget_set_hexpand(scrolled, FALSE);
            gtk_widget_set_vexpand(scrolled, FALSE);
            gtk_widget_add_events(scrolled, GDK_SCROLL_MASK);
            gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scrolled),
                                           GTK_POLICY_NEVER,
                                           GTK_POLICY_AUTOMATIC);
#if GTK_CHECK_VERSION(3, 16, 0)
            gtk_scrolled_window_set_overlay_scrolling(GTK_SCROLLED_WINDOW(scrolled),
                                                      FALSE);
#endif
#ifdef __APPLE__
            if (rows > 0)
            {
                gint row_height = 24;
                gint visible_rows = MIN(rows, 12);

                if (list_nat > 0)
                    row_height = list_nat / MAX(rows, 1);
                else if (list_min > 0)
                    row_height = list_min / MAX(rows, 1);

                if (row_height < 18 || row_height > 64)
                    row_height = 24;

                popup_height = row_height * visible_rows + 2;
                if (popup_height > max_height)
                    popup_height = max_height;
            }
            if (popup_height > 0)
                gtk_widget_set_size_request(scrolled, -1, popup_height);
            else
                gtk_widget_set_size_request(scrolled, -1, max_height);
#else
            gtk_scrolled_window_set_propagate_natural_height(
                GTK_SCROLLED_WINDOW(scrolled), !needs_scroll);
#if GTK_CHECK_VERSION(3, 22, 0)
            gtk_scrolled_window_set_max_content_height(
                GTK_SCROLLED_WINDOW(scrolled), needs_scroll ? max_height : -1);
            if (popup_width > 0)
                gtk_scrolled_window_set_min_content_width(
                    GTK_SCROLLED_WINDOW(scrolled), popup_width);
#endif
            gtk_widget_set_size_request(scrolled, -1, -1);
#endif
        }
#ifdef __APPLE__
        if (popup_width > 0 || popup_height > 0)
            gtk_widget_set_size_request(popup_widget,
                                        popup_width > 0 ? popup_width : -1,
                                        popup_height > 0 ? popup_height : -1);
#endif

        if (GTK_IS_TREE_VIEW(tree))
        {
            gtk_widget_add_events(tree, GDK_SCROLL_MASK);
            gtk_tree_view_set_headers_visible(GTK_TREE_VIEW(tree), FALSE);
            gtk_tree_view_set_fixed_height_mode(GTK_TREE_VIEW(tree), FALSE);
            gtk_tree_view_set_search_column(GTK_TREE_VIEW(tree), 0);
            gtk_tree_view_set_activate_on_single_click(GTK_TREE_VIEW(tree), FALSE);
            gtk_tree_view_set_enable_search(GTK_TREE_VIEW(tree),
                                            rows > RIGCTRL_TRSP_POPUP_SEARCH_THRESHOLD);
        }

        if (rows >= RIGCTRL_TRSP_POPUP_SCROLL_CHECK_THRESHOLD)
        {
            g_warn_if_fail(GTK_IS_SCROLLED_WINDOW(scrolled));
            if (GTK_IS_SCROLLED_WINDOW(scrolled))
            {
                gint scrolled_min = 0;
                gint scrolled_nat = 0;

                gtk_widget_get_preferred_height(scrolled, &scrolled_min, &scrolled_nat);
                if (scrolled_nat <= 0 && scrolled_min > 0)
                    scrolled_nat = scrolled_min;
                if (scrolled_nat > max_height)
                    g_warn_if_fail(scrolled_nat <= max_height);
            }
        }

        if (g_object_get_data(G_OBJECT(popup_widget), "rigctrl-popup-hooked") == NULL)
        {
            g_object_set_data(G_OBJECT(popup_widget), "rigctrl-popup-hooked",
                              GINT_TO_POINTER(1));
            g_signal_connect(popup_widget, "map",
                             G_CALLBACK(rigctrl_trsp_popup_show), combo);
            g_signal_connect(popup_widget, "hide",
                             G_CALLBACK(rigctrl_trsp_popup_hide), combo);
        }
    }

    g_object_unref(combo);
    return G_SOURCE_REMOVE;
}

static void rigctrl_trsp_combo_realize(GtkWidget *widget, gpointer data)
{
    (void)data;

    if (widget == NULL)
        return;

    g_idle_add(rigctrl_configure_trsp_popup_idle, g_object_ref(widget));
}

static void rigctrl_trsp_popup_show(GtkWidget *widget, gpointer data)
{
    GtkWidget *scrolled = NULL;
    GtkWidget *tree = NULL;
    GtkAdjustment *vadj;
    GtkTreePath *path;
    GtkWidget *combo = GTK_IS_WIDGET(data) ? GTK_WIDGET(data) : NULL;
    gint popup_min = 0, popup_nat = 0;
    gint scrolled_min = 0, scrolled_nat = 0;
    gint list_min = 0, list_nat = 0;

    (void)data;

    if (widget == NULL)
        return;

    if (combo != NULL)
    {
        g_object_set_data(G_OBJECT(combo), "rigctrl-popup-opened",
                          GINT_TO_POINTER(1));
        rigctrl_trsp_popup_set_ts(combo, "rigctrl-popup-last-show");
        rigctrl_trsp_popup_bump_seq(combo);
    }
    if (combo != NULL)
        g_idle_add(rigctrl_configure_trsp_popup_idle, g_object_ref(combo));

    scrolled = rigctrl_trsp_find_scrolled(widget);

    if (!GTK_IS_SCROLLED_WINDOW(scrolled))
        return;

    vadj = gtk_scrolled_window_get_vadjustment(GTK_SCROLLED_WINDOW(scrolled));
    if (vadj == NULL)
        return;

    gtk_adjustment_set_value(vadj, gtk_adjustment_get_lower(vadj));

    tree = rigctrl_trsp_find_child(widget, GTK_TYPE_TREE_VIEW);
    if (GTK_IS_TREE_VIEW(tree))
    {
        path = gtk_tree_path_new_first();
        gtk_tree_view_scroll_to_cell(GTK_TREE_VIEW(tree), path, NULL,
                                     TRUE, 0.0, 0.0);
        gtk_tree_path_free(path);
        gtk_widget_queue_resize(tree);
        gtk_widget_queue_draw(tree);
    }

    gtk_widget_get_preferred_height(widget, &popup_min, &popup_nat);
    gtk_widget_get_preferred_height(scrolled, &scrolled_min, &scrolled_nat);
    if (tree != NULL)
        gtk_widget_get_preferred_height(tree, &list_min, &list_nat);

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                _("%s: trsp popup show alloc=%dx%d scroll_alloc=%dx%d "
                  "pref popup=%d/%d scroll=%d/%d list=%d/%d"),
                __func__,
                gtk_widget_get_allocated_width(widget),
                gtk_widget_get_allocated_height(widget),
                gtk_widget_get_allocated_width(scrolled),
                gtk_widget_get_allocated_height(scrolled),
                popup_min, popup_nat,
                scrolled_min, scrolled_nat,
                list_min, list_nat);
    RIGCTRL_TRSP_POPUP_LOG("%s: popup shown", __func__);
}

static void rigctrl_trsp_popup_hide(GtkWidget *widget, gpointer data)
{
    GtkWidget *combo = GTK_IS_WIDGET(data) ? GTK_WIDGET(data) : NULL;
    GtkWidget *scrolled = NULL;

    if (widget == NULL)
        return;

    if (combo != NULL)
    {
        g_object_set_data(G_OBJECT(combo), "rigctrl-popup-opened", NULL);
        rigctrl_trsp_popup_set_ts(combo, "rigctrl-popup-last-hide");
    }

    scrolled = rigctrl_trsp_find_scrolled(widget);

    if (GTK_IS_SCROLLED_WINDOW(scrolled))
    {
        gtk_widget_set_size_request(scrolled, -1, -1);
#if GTK_CHECK_VERSION(3, 22, 0)
        gtk_scrolled_window_set_max_content_height(
            GTK_SCROLLED_WINDOW(scrolled), -1);
        gtk_scrolled_window_set_min_content_width(
            GTK_SCROLLED_WINDOW(scrolled), -1);
#endif
    }

    gtk_widget_set_size_request(widget, -1, -1);

    sat_log_log(SAT_LOG_LEVEL_DEBUG, _("%s: trsp popup hide"), __func__);
    RIGCTRL_TRSP_POPUP_LOG("%s: popup hidden", __func__);
}

static gint rigctrl_trsp_popup_get_max_height(GtkWidget *anchor)
{
    gint max_height = RIGCTRL_TRSP_POPUP_MAX_HEIGHT;
    GdkRectangle workarea;
    gboolean have_workarea = FALSE;

#if GTK_CHECK_VERSION(3, 22, 0)
    GdkDisplay *display = gdk_display_get_default();
    GdkMonitor *monitor = NULL;
    GdkWindow *gdk_window = NULL;

    if (anchor != NULL)
        gdk_window = gtk_widget_get_window(anchor);
    if (display != NULL)
    {
        if (gdk_window != NULL)
            monitor = gdk_display_get_monitor_at_window(display, gdk_window);
        if (monitor == NULL)
            monitor = gdk_display_get_primary_monitor(display);
    }

    if (monitor != NULL)
    {
        gdk_monitor_get_workarea(monitor, &workarea);
        have_workarea = (workarea.height > 0);
    }
#else
    if (anchor != NULL)
    {
        GdkScreen *screen = gtk_widget_get_screen(anchor);
        if (screen != NULL)
        {
            GdkWindow *gdk_window = gtk_widget_get_window(anchor);
            gint monitor = 0;

            if (gdk_window != NULL)
                monitor = gdk_screen_get_monitor_at_window(screen, gdk_window);
            gdk_screen_get_monitor_workarea(screen, monitor, &workarea);
            have_workarea = (workarea.height > 0);
        }
    }
#endif

    if (have_workarea)
    {
        gint scaled = (gint)(workarea.height * RIGCTRL_TRSP_POPUP_MAX_FACTOR);
        if (scaled > 0)
            max_height = MIN(max_height, scaled);
    }

    return max_height;
}

static gint rigctrl_trsp_tree_row_count(GtkWidget *tree)
{
    GtkTreeModel *model;

    if (!GTK_IS_TREE_VIEW(tree))
        return 0;

    model = gtk_tree_view_get_model(GTK_TREE_VIEW(tree));
    if (model == NULL)
        return 0;

    return gtk_tree_model_iter_n_children(model, NULL);
}

static GtkWidget *rigctrl_trsp_find_child(GtkWidget *widget, GType child_type)
{
    GList *children = NULL;
    GList *entry = NULL;
    GtkWidget *child = NULL;

    if (widget == NULL)
        return NULL;

    if (G_TYPE_CHECK_INSTANCE_TYPE(widget, child_type))
        return widget;

    if (!GTK_IS_CONTAINER(widget))
        return NULL;

    children = gtk_container_get_children(GTK_CONTAINER(widget));
    for (entry = children; entry != NULL; entry = entry->next)
    {
        child = rigctrl_trsp_find_child(GTK_WIDGET(entry->data), child_type);
        if (child != NULL)
            break;
    }

    g_list_free(children);
    return child;
}

static GtkWidget *rigctrl_trsp_find_scrolled(GtkWidget *widget)
{
    GtkWidget *scrolled = widget;

    while (scrolled != NULL && !GTK_IS_SCROLLED_WINDOW(scrolled))
        scrolled = gtk_widget_get_parent(scrolled);

    if (!GTK_IS_SCROLLED_WINDOW(scrolled))
        scrolled = rigctrl_trsp_find_child(widget, GTK_TYPE_SCROLLED_WINDOW);

    return scrolled;
}

static gint rigctld_parse_identifier_pid(const gchar *identifier)
{
    gchar *endp = NULL;
    glong pid = 0;

    if (identifier == NULL || *identifier == '\0')
        return 0;

    pid = g_ascii_strtoll(identifier, &endp, 10);
    if (endp == identifier || pid <= 0 || pid > G_MAXINT)
        return 0;

    return (gint) pid;
}

static void rigctld_set_spawn_state(GtkRigCtrl *ctrl,
                                    gboolean secondary,
                                    RigctldMgr *mgr)
{
    gint pid = 0;
    const gchar *identifier = NULL;

    if (ctrl == NULL)
        return;

    identifier = rigctld_mgr_get_identifier(mgr);
    pid = rigctld_parse_identifier_pid(identifier);

    if (secondary)
    {
        ctrl->rigctld_spawned2 = (mgr != NULL);
        ctrl->rigctld_spawn_pid2 = pid;
    }
    else
    {
        ctrl->rigctld_spawned = (mgr != NULL);
        ctrl->rigctld_spawn_pid = pid;
    }
}

static void rigctld_clear_spawn_state(GtkRigCtrl *ctrl, gboolean secondary)
{
    if (ctrl == NULL)
        return;

    if (secondary)
    {
        ctrl->rigctld_spawned2 = FALSE;
        ctrl->rigctld_spawn_pid2 = 0;
    }
    else
    {
        ctrl->rigctld_spawned = FALSE;
        ctrl->rigctld_spawn_pid = 0;
    }
}

static gboolean rigctld_spawned_by_us(const GtkRigCtrl *ctrl,
                                      gboolean secondary)
{
    if (ctrl == NULL)
        return FALSE;

    return secondary ? ctrl->rigctld_spawned2 : ctrl->rigctld_spawned;
}

static void rigctld_terminate_spawned(GtkRigCtrl *ctrl,
                                      gboolean secondary,
                                      RigctldMgr **mgr_ptr)
{
    if (mgr_ptr == NULL || *mgr_ptr == NULL)
    {
        rigctld_clear_spawn_state(ctrl, secondary);
        return;
    }

    if (!rigctld_spawned_by_us(ctrl, secondary))
        return;

    rigctld_mgr_terminate(mgr_ptr);
    rigctld_clear_spawn_state(ctrl, secondary);
}

/*
 * Update rig control state.
 *
 * This function is called by the parent, i.e. GtkSatModule, indicating that
 * the satellite data has been updated. The function updates the internal state
 * of the controller and the rigator.
 */
void gtk_rig_ctrl_update(GtkRigCtrl * ctrl, gdouble t)
{
    gchar          *buff;

    g_mutex_lock(&ctrl->rig_ctrl_updatelock);

    if (rigctrl_target_display_valid(ctrl->target) && isfinite(t))
    {
        buff = g_strdup_printf(AZEL_FMTSTR, ctrl->target->az);
        gtk_label_set_text(GTK_LABEL(ctrl->SatAz), buff);
        g_free(buff);
        buff = g_strdup_printf(AZEL_FMTSTR, ctrl->target->el);
        gtk_label_set_text(GTK_LABEL(ctrl->SatEl), buff);
        g_free(buff);

        update_count_down(ctrl, t);

        if (sat_cfg_get_bool(SAT_CFG_BOOL_USE_IMPERIAL))
        {
            buff = g_strdup_printf("%.0f mi", KM_TO_MI(ctrl->target->range));
        }
        else
        {
            buff = g_strdup_printf("%.0f km", ctrl->target->range);
        }
        gtk_label_set_text(GTK_LABEL(ctrl->SatRng), buff);
        g_free(buff);

        if (sat_cfg_get_bool(SAT_CFG_BOOL_USE_IMPERIAL))
        {
            buff = g_strdup_printf("%.3f mi/s",
                                   KM_TO_MI(ctrl->target->range_rate));
        }
        else
        {
            buff = g_strdup_printf("%.3f km/s", ctrl->target->range_rate);
        }
        gtk_label_set_text(GTK_LABEL(ctrl->SatRngRate), buff);
        g_free(buff);

        rigctrl_update_doppler(ctrl);
        buff = g_strdup_printf("%" G_GINT64_FORMAT " Hz", ctrl->doppler_down_hz);
        gtk_label_set_text(GTK_LABEL(ctrl->SatDopDown), buff);
        g_free(buff);
        buff = g_strdup_printf("%" G_GINT64_FORMAT " Hz", ctrl->doppler_up_hz);
        gtk_label_set_text(GTK_LABEL(ctrl->SatDopUp), buff);
        g_free(buff);

        /* update next pass if necessary */
        if (ctrl->pass != NULL)
        {
            if (ctrl->target->aos > ctrl->pass->aos)
            {
                free_pass(ctrl->pass);
                ctrl->pass = get_next_pass(ctrl->target, ctrl->qth, 3.0);
            }
        }
        else
        {
            /* we don't have any current pass; store the current one */
            ctrl->pass = get_next_pass(ctrl->target, ctrl->qth, 3.0);
        }
    }
    else
    {
        if (ctrl->target != NULL &&
            rigctrl_log_throttled(ctrl, &ctrl->last_invalid_log_us,
                                  RIGCTRL_DOPPLER_LOG_INTERVAL_US))
        {
            sat_log_log(SAT_LOG_LEVEL_WARN,
                        "rig target telemetry invalid: az=%.3f el=%.3f range=%.3f rate=%.3f",
                        ctrl->target->az,
                        ctrl->target->el,
                        ctrl->target->range,
                        ctrl->target->range_rate);
        }

        gtk_label_set_text(GTK_LABEL(ctrl->SatAz), " --- ");
        gtk_label_set_text(GTK_LABEL(ctrl->SatEl), " --- ");
        if (sat_cfg_get_bool(SAT_CFG_BOOL_USE_IMPERIAL))
        {
            gtk_label_set_text(GTK_LABEL(ctrl->SatRng), "--- mi");
            gtk_label_set_text(GTK_LABEL(ctrl->SatRngRate), "--- mi/s");
        }
        else
        {
            gtk_label_set_text(GTK_LABEL(ctrl->SatRng), "--- km");
            gtk_label_set_text(GTK_LABEL(ctrl->SatRngRate), "--- km/s");
        }
        update_count_down(ctrl, t);
        rigctrl_update_doppler(ctrl);
        buff = g_strdup_printf("%" G_GINT64_FORMAT " Hz", ctrl->doppler_down_hz);
        gtk_label_set_text(GTK_LABEL(ctrl->SatDopDown), buff);
        g_free(buff);
        buff = g_strdup_printf("%" G_GINT64_FORMAT " Hz", ctrl->doppler_up_hz);
        gtk_label_set_text(GTK_LABEL(ctrl->SatDopUp), buff);
        g_free(buff);
    }

    rigctrl_update_freq_display(ctrl);
    rigctrl_refresh_ui_status(ctrl, "module update");

    g_mutex_unlock(&ctrl->rig_ctrl_updatelock);
}



void gtk_rig_ctrl_select_sat(GtkRigCtrl * ctrl, gint catnum)
{
    sat_t          *sat;
    int             i, n;

    /* find index in satellite list */
    n = g_slist_length(ctrl->sats);
    for (i = 0; i < n; i++)
    {
        sat = SAT(g_slist_nth_data(ctrl->sats, i));
        if (sat)
        {
            if (sat->tle.catnr == catnum)
            {
                /* assume the index is the same in sat selector */
                rigctrl_combo_set_active_blocked(GTK_COMBO_BOX(ctrl->SatSel), i,
                                                 G_CALLBACK(sat_selected_cb),
                                                 ctrl);
                break;
            }
        }
    }
}

static void downlink_changed_cb(GtkFreqKnob * knob, gpointer data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);
    gint64          hz = 0;

    (void)knob;

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    if (ctrl->suppress_user_base)
        return;

    hz = rigctrl_knob_to_hz(ctrl->SatFreqDown);
    rigctrl_set_user_base_freq(ctrl, TRUE, hz, RIG_BASE_SRC_MANUAL, TRUE);
    rigctrl_reset_send_tracking(ctrl, TRUE);
    ctrl->pending_manual_down = TRUE;

}

static void uplink_changed_cb(GtkFreqKnob * knob, gpointer data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);
    gint64          hz = 0;

    (void)knob;

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    if (ctrl->suppress_user_base)
        return;

    hz = rigctrl_knob_to_hz(ctrl->SatFreqUp);
    rigctrl_set_user_base_freq(ctrl, FALSE, hz, RIG_BASE_SRC_MANUAL, TRUE);
    rigctrl_reset_send_tracking(ctrl, FALSE);
    ctrl->pending_manual_up = TRUE;

}

/*
 * Create freq control widgets for downlink.
 *
 * This function creates and initialises the widgets for controlling the
 * downlink frequency. It consists of a controller widget showing the
 * satellite frequency with the radio frequency below it.
 *
 */
static GtkWidget *create_downlink_widgets(GtkRigCtrl * ctrl)
{
    GtkWidget      *frame;
    GtkWidget      *vbox;
    GtkWidget      *hbox1, *hbox2;
    GtkWidget      *hbox2_wrap;
    GtkWidget      *hbox2_spacer_left;
    GtkWidget      *hbox2_spacer_right;
    GtkWidget      *freq_wrap;
    GtkWidget      *freq_spacer_left;
    GtkWidget      *freq_spacer_right;
    GtkWidget      *label;

    label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(label), _("<b> Downlink </b>"));
    frame = gtk_frame_new(NULL);
    gtk_frame_set_label_align(GTK_FRAME(frame), 0.5, 0.5);
    gtk_frame_set_label_widget(GTK_FRAME(frame), label);
    gtk_widget_set_hexpand(frame, TRUE);
    gtk_widget_set_halign(frame, GTK_ALIGN_FILL);

    vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    gtk_container_set_border_width(GTK_CONTAINER(vbox), 10);
    hbox1 = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    hbox2 = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    hbox2_wrap = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_widget_set_hexpand(hbox2_wrap, TRUE);
    hbox2_spacer_left = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    hbox2_spacer_right = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_widget_set_hexpand(hbox2_spacer_left, TRUE);
    gtk_widget_set_hexpand(hbox2_spacer_right, TRUE);
    gtk_box_pack_start(GTK_BOX(hbox2_wrap), hbox2_spacer_left, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(hbox2_wrap), hbox2, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(hbox2_wrap), hbox2_spacer_right, TRUE, TRUE, 0);

    /* satellite downlink frequency */
    ctrl->SatFreqDown = gtk_freq_knob_new(145890000.0, TRUE);
    g_signal_connect(ctrl->SatFreqDown, "freq-changed",
                     G_CALLBACK(downlink_changed_cb), ctrl);
    freq_wrap = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_widget_set_hexpand(freq_wrap, TRUE);
    freq_spacer_left = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    freq_spacer_right = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_widget_set_hexpand(freq_spacer_left, TRUE);
    gtk_widget_set_hexpand(freq_spacer_right, TRUE);
    gtk_box_pack_start(GTK_BOX(freq_wrap), freq_spacer_left, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(freq_wrap), ctrl->SatFreqDown, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(freq_wrap), freq_spacer_right, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(vbox), freq_wrap, FALSE, FALSE, 0);

    /* Downlink doppler */
    label = gtk_label_new(_("Doppler:"));
    gtk_widget_set_tooltip_text(label,
                                _("The Doppler shift according to the range "
                                  "rate and the currently selected uplink "
                                  "frequency"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_box_pack_start(GTK_BOX(hbox1), label, FALSE, FALSE, 0);
    ctrl->SatDopDown = gtk_label_new("---- Hz");
    g_object_set(ctrl->SatDopDown, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_box_pack_start(GTK_BOX(hbox1), ctrl->SatDopDown, FALSE, TRUE, 0);

    /* Downconverter LO */
    ctrl->LoDown = gtk_label_new("0 MHz");
    g_object_set(ctrl->LoDown, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_box_pack_end(GTK_BOX(hbox1), ctrl->LoDown, FALSE, FALSE, 2);
    label = gtk_label_new(_("LO:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_box_pack_end(GTK_BOX(hbox1), label, FALSE, FALSE, 0);

    /* Radio downlink frequency */
    label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(label),
                         "<span size='large'><b>Radio:</b></span>");
    g_object_set(label, "xalign", 0.5f, "yalign", 1.0f, NULL);
    gtk_box_pack_start(GTK_BOX(hbox2), label, FALSE, FALSE, 0);
    ctrl->RigFreqDown = gtk_freq_knob_new(145890000.0, FALSE);
    gtk_box_pack_start(GTK_BOX(hbox2), ctrl->RigFreqDown, FALSE, FALSE, 0);

    /* finish packing ... */
    gtk_box_pack_start(GTK_BOX(vbox), hbox1, TRUE, TRUE, 12);
    gtk_box_pack_start(GTK_BOX(vbox), hbox2_wrap, FALSE, FALSE, 6);
    gtk_container_add(GTK_CONTAINER(frame), vbox);

    return frame;
}

/*
 * Create uplink frequency display widgets.
 *
 * This function creates and initialises the widgets for displaying the
 * uplink frequency of the satellite and the radio.
 */
static GtkWidget *create_uplink_widgets(GtkRigCtrl * ctrl)
{
    GtkWidget      *frame;
    GtkWidget      *vbox;
    GtkWidget      *hbox1, *hbox2;
    GtkWidget      *hbox2_wrap;
    GtkWidget      *hbox2_spacer_left;
    GtkWidget      *hbox2_spacer_right;
    GtkWidget      *freq_wrap;
    GtkWidget      *freq_spacer_left;
    GtkWidget      *freq_spacer_right;
    GtkWidget      *label;

    label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(label), _("<b> Uplink </b>"));
    frame = gtk_frame_new(NULL);
    gtk_frame_set_label_align(GTK_FRAME(frame), 0.5, 0.5);
    gtk_frame_set_label_widget(GTK_FRAME(frame), label);
    gtk_widget_set_hexpand(frame, TRUE);
    gtk_widget_set_halign(frame, GTK_ALIGN_FILL);

    vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    gtk_container_set_border_width(GTK_CONTAINER(vbox), 10);
    hbox1 = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    hbox2 = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    hbox2_wrap = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_widget_set_hexpand(hbox2_wrap, TRUE);
    hbox2_spacer_left = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    hbox2_spacer_right = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_widget_set_hexpand(hbox2_spacer_left, TRUE);
    gtk_widget_set_hexpand(hbox2_spacer_right, TRUE);
    gtk_box_pack_start(GTK_BOX(hbox2_wrap), hbox2_spacer_left, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(hbox2_wrap), hbox2, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(hbox2_wrap), hbox2_spacer_right, TRUE, TRUE, 0);

    /* satellite uplink frequency */
    ctrl->SatFreqUp = gtk_freq_knob_new(145890000.0, TRUE);
    g_signal_connect(ctrl->SatFreqUp, "freq-changed",
                     G_CALLBACK(uplink_changed_cb), ctrl);
    freq_wrap = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_widget_set_hexpand(freq_wrap, TRUE);
    freq_spacer_left = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    freq_spacer_right = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_widget_set_hexpand(freq_spacer_left, TRUE);
    gtk_widget_set_hexpand(freq_spacer_right, TRUE);
    gtk_box_pack_start(GTK_BOX(freq_wrap), freq_spacer_left, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(freq_wrap), ctrl->SatFreqUp, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(freq_wrap), freq_spacer_right, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(vbox), freq_wrap, FALSE, FALSE, 0);

    /* Uplink doppler */
    label = gtk_label_new(_("Doppler:"));
    gtk_widget_set_tooltip_text(label,
                                _("The Doppler shift according to the range "
                                  "rate and the currently selected downlink "
                                  "frequency"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_box_pack_start(GTK_BOX(hbox1), label, FALSE, FALSE, 0);
    ctrl->SatDopUp = gtk_label_new("---- Hz");
    g_object_set(ctrl->SatDopUp, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_box_pack_start(GTK_BOX(hbox1), ctrl->SatDopUp, FALSE, TRUE, 0);

    /* Upconverter LO */
    ctrl->LoUp = gtk_label_new("0 MHz");
    g_object_set(ctrl->LoUp, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_box_pack_end(GTK_BOX(hbox1), ctrl->LoUp, FALSE, FALSE, 2);
    label = gtk_label_new(_("LO:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_box_pack_end(GTK_BOX(hbox1), label, FALSE, FALSE, 0);

    /* Radio uplink frequency */
    label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(label),
                         "<span size='large'><b>Radio:</b></span>");
    g_object_set(label, "xalign", 0.5f, "yalign", 1.0f, NULL);
    gtk_box_pack_start(GTK_BOX(hbox2), label, FALSE, FALSE, 0);
    ctrl->RigFreqUp = gtk_freq_knob_new(145890000.0, FALSE);
    gtk_box_pack_start(GTK_BOX(hbox2), ctrl->RigFreqUp, FALSE, FALSE, 0);

    gtk_box_pack_start(GTK_BOX(vbox), hbox1, TRUE, TRUE, 12);
    gtk_box_pack_start(GTK_BOX(vbox), hbox2_wrap, FALSE, FALSE, 6);
    gtk_container_add(GTK_CONTAINER(frame), vbox);

    return frame;
}
static void trsp_selected_cb(GtkComboBox * box, gpointer data);

static void load_trsp_list(GtkRigCtrl * ctrl)
{
    trsp_t         *trsp = NULL;
    GtkListStore   *store = NULL;
    GtkTreeIter     iter;
    guint           i, n;
    guint           rows = 0;
    const gchar    *empty_label = NULL;
    const gchar    *tooltip = NULL;
    gboolean        was_updating = FALSE;

    if (ctrl == NULL || ctrl->TrspSel == NULL)
        return;

    if (rigctrl_combo_popup_shown(GTK_COMBO_BOX(ctrl->TrspSel)))
    {
        rigctrl_schedule_trsp_refresh(ctrl);
        return;
    }

    if (ctrl->trsplist != NULL)
    {
        free_transponders(ctrl->trsplist);
        ctrl->trsplist = NULL;
        ctrl->trsp = NULL;
    }

    store = gtk_list_store_new(1, G_TYPE_STRING);
    tooltip = _("Select a payload preset. "
                "Its baseline RX/TX frequencies are applied.");

    /* check if there is a target satellite */
    if (ctrl->target == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s:%s: GtkSatModule has no target satellite."),
                    __FILE__, __func__);
        empty_label = _("No target selected");
        tooltip = _("Select a target satellite to load payload presets.");
        goto apply_model;
    }

    /* read transponders for new target */
    ctrl->trsplist = read_transponders(ctrl->target->tle.catnr);
    n = g_slist_length(ctrl->trsplist);
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                _("%s:%s: Satellite %d has %d transponder modes."),
                __FILE__, __func__, ctrl->target->tle.catnr, n);

    for (i = 0; i < n; i++)
    {
        trsp = (trsp_t *) g_slist_nth_data(ctrl->trsplist, i);
        if (trsp == NULL)
            continue;

        gtk_list_store_append(store, &iter);
        gtk_list_store_set(store, &iter, 0, trsp->name, -1);
        rows++;

        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    _("%s:%s: Read transponder '%s' for satellite %d"),
                    __FILE__, __func__, trsp->name, ctrl->target->tle.catnr);
    }

    if (rows > 0)
    {
        ctrl->trsp = (trsp_t *) g_slist_nth_data(ctrl->trsplist, 0);
        rigctrl_apply_trsp_preset(ctrl, FALSE);
    }
    else
    {
        empty_label = _("No transponder data (Tools -> Update Transponder Data)");
        tooltip = _("No transponder data available. "
                    "Use Tools -> Update Transponder Data.");
    }

    if (rigctrl_log_at_least(ctrl, RIG_LOG_VERBOSE))
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rigctrl trsp combo rows=%u active=%s",
                    rows,
                    rows > 0 ? "0" : "(none)");

apply_model:
    was_updating = ctrl->ui_updating;
    rigctrl_ui_begin_update(ctrl, "trsp_list");
    g_signal_handlers_block_by_func(ctrl->TrspSel,
                                    (gpointer)G_CALLBACK(trsp_selected_cb),
                                    ctrl);
    if (rows == 0 && empty_label != NULL)
    {
        gtk_list_store_append(store, &iter);
        gtk_list_store_set(store, &iter, 0, empty_label, -1);
    }
    gtk_combo_box_set_model(GTK_COMBO_BOX(ctrl->TrspSel), GTK_TREE_MODEL(store));
    if (rows > 0 || empty_label != NULL)
        gtk_combo_box_set_active(GTK_COMBO_BOX(ctrl->TrspSel), 0);
    g_signal_handlers_unblock_by_func(ctrl->TrspSel,
                                      (gpointer)G_CALLBACK(trsp_selected_cb),
                                      ctrl);
    if (!was_updating)
        rigctrl_ui_end_update(ctrl, "trsp_list");
    if (ctrl->TrspSel != NULL && tooltip != NULL)
        gtk_widget_set_tooltip_text(ctrl->TrspSel, tooltip);
    g_object_unref(store);

    if (ctrl->TrspSel != NULL)
        g_idle_add(rigctrl_configure_trsp_popup_idle,
                   g_object_ref(ctrl->TrspSel));
}

static gboolean rigctrl_trsp_refresh_idle(gpointer data)
{
    GtkRigCtrl *ctrl = GTK_RIG_CTRL(data);

    if (ctrl == NULL)
        return G_SOURCE_REMOVE;

    ctrl->pending_ui_refresh_id = 0;
    if (!ctrl->destroying)
        load_trsp_list(ctrl);

    g_object_unref(ctrl);
    return G_SOURCE_REMOVE;
}

static void rigctrl_schedule_trsp_refresh(GtkRigCtrl *ctrl)
{
    if (ctrl == NULL)
        return;
    if (ctrl->destroying)
        return;

    if (ctrl->pending_ui_refresh_id != 0)
        return;

    if (rigctrl_log_at_least(ctrl, RIG_LOG_VERBOSE))
        sat_log_log(SAT_LOG_LEVEL_DEBUG, "rigctrl schedule transponder refresh");

    ctrl->pending_ui_refresh_id =
        g_idle_add(rigctrl_trsp_refresh_idle, g_object_ref(ctrl));
    if (ctrl->pending_ui_refresh_id == 0)
        g_object_unref(ctrl);
}

static gboolean have_conf(void)
{
    GDir           *dir = NULL; /* directory handle */
    GError         *error = NULL;       /* error flag and info */
    gchar          *dirname;    /* directory name */
    const gchar    *filename;   /* file name */
    gint            i = 0;

    dirname = get_hwconf_dir();
    dir = g_dir_open(dirname, 0, &error);
    if (dir)
    {
        while ((filename = g_dir_read_name(dir)))
        {
            if (g_str_has_suffix(filename, ".rig"))
            {
                i++;
                break;
            }
        }
    }
    else
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%d: Failed to open hwconf dir (%s)"),
                    __FILE__, __LINE__, error->message);
        g_clear_error(&error);
    }

    g_free(dirname);
    g_dir_close(dir);

    return (i > 0) ? TRUE : FALSE;
}

/* Called when the user selects a new satellite. */
static void sat_selected_cb(GtkComboBox * satsel, gpointer data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);
    gint            i;

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    i = gtk_combo_box_get_active(satsel);
    if (i >= 0)
    {
        ctrl->target = SAT(g_slist_nth_data(ctrl->sats, i));

        ctrl->prev_ele = ctrl->target->el;

        /* update next pass */
        if (ctrl->pass != NULL)
            free_pass(ctrl->pass);
        ctrl->pass = get_next_pass(ctrl->target, ctrl->qth, 3.0);

        /* read transponders for new target */
        rigctrl_schedule_trsp_refresh(ctrl);
        ctrl->user_edit_down = FALSE;
        ctrl->user_edit_up = FALSE;
    }
    else
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%s: Invalid satellite selection: %d"),
                    __FILE__, __func__, i);

        /* clear pass just in case... */
        if (ctrl->pass != NULL)
        {
            free_pass(ctrl->pass);
            ctrl->pass = NULL;
        }
    }
}

static gboolean rigctrl_payload_baseline_hz(const trsp_t *trsp,
                                            gint64 *rx_hz_out,
                                            gint64 *tx_hz_out)
{
    gint64 rx_hz = 0;
    gint64 tx_hz = 0;

    if (trsp == NULL)
    {
        if (rx_hz_out)
            *rx_hz_out = 0;
        if (tx_hz_out)
            *tx_hz_out = 0;
        return FALSE;
    }

    if (trsp->downlow > 0 && trsp->downhigh > 0)
    {
        rx_hz = trsp->downlow +
            llabs(trsp->downhigh - trsp->downlow) / 2;
    }
    if (trsp->uplow > 0 && trsp->uphigh > 0)
    {
        tx_hz = trsp->uplow +
            llabs(trsp->uphigh - trsp->uplow) / 2;
    }

    if (rx_hz_out)
        *rx_hz_out = rx_hz;
    if (tx_hz_out)
        *tx_hz_out = tx_hz;

    return (rx_hz > 0 || tx_hz > 0);
}

/*
 * Apply the selected payload preset baseline frequencies.
 */
static void rigctrl_apply_trsp_preset(GtkRigCtrl *ctrl, gboolean mark_manual)
{
    gint64 rx_hz = 0;
    gint64 tx_hz = 0;

    if (!rigctrl_payload_baseline_hz(ctrl->trsp, &rx_hz, &tx_hz))
        return;

    if (rx_hz <= 0)
        rx_hz = tx_hz;
    if (rx_hz <= 0)
        return;
    if (tx_hz <= 0)
        tx_hz = rx_hz;

    rigctrl_set_user_base_freq(ctrl, TRUE, rx_hz, RIG_BASE_SRC_PRESET, mark_manual);
    rigctrl_set_user_base_freq(ctrl, FALSE, tx_hz, RIG_BASE_SRC_PRESET, mark_manual);
    rigctrl_reset_send_tracking(ctrl, TRUE);
    rigctrl_reset_send_tracking(ctrl, FALSE);

    /* invalidate RIG<->GPREDICT sync */
    ctrl->lastrxf = 0;
    ctrl->lasttxf = 0;
}

/*
 * Called when a new transponder is selected.
 * It updates ctrl->trsp with the new selection and issues a "tune" event.
 */
static void trsp_selected_cb(GtkComboBox * box, gpointer data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);
    gint            i, n;

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    i = gtk_combo_box_get_active(box);
    n = g_slist_length(ctrl->trsplist);

    if (n == 0)
    {
        ctrl->trsp = NULL;
        return;
    }

    if (i == -1)
    {
        /* clear transponder data */
        ctrl->trsp = NULL;
    }
    else if (i < n)
    {
        ctrl->trsp = (trsp_t *) g_slist_nth_data(ctrl->trsplist, i);
        ctrl->user_edit_down = FALSE;
        ctrl->user_edit_up = FALSE;
        rigctrl_apply_trsp_preset(ctrl, FALSE);
    }
    else
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: Inconsistency detected in internal transponder "
                      "data (%d,%d)"), __func__, i, n);
    }

}

/*
 * Manage lock transponder signals.
 *
 * @param button Pointer to the GtkToggleButton that received the signal.
 * @param data Pointer to the GtkRigCtrl structure.
 *
 * This function is called when the user toggles the "Lock Transponder" button.
 * When ON, the uplink and downlink are locked according to the current transponder
 * data, i.e. when user changes the downlink, the uplink will follow automatically
 * taking into account whether the transponder is inverting or not.
 */
static void rigctrl_sync_tracking_state(GtkRigCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    ctrl->tracking = ctrl->rx_track_enabled || ctrl->tx_track_enabled;
}

static void rx_track_toggle_cb(GtkToggleButton *button, gpointer data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);
    gboolean        requested;

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    requested = gtk_toggle_button_get_active(button);
    if (requested && !radio_apply_ui_settings(ctrl, TRUE))
    {
        rigctrl_ui_begin_update(ctrl, "rx_track_invalid_settings");
        gtk_toggle_button_set_active(button, FALSE);
        rigctrl_ui_end_update(ctrl, "rx_track_invalid_settings");
        return;
    }

    ctrl->rx_track_enabled = requested;
    rigctrl_sync_tracking_state(ctrl);

    sat_log_log(SAT_LOG_LEVEL_DEBUG, "SATMODE: rx tracking %s",
                ctrl->rx_track_enabled ? "on" : "off");

    if (ctrl->tracking && is_full_duplex_main_sub_configured(ctrl->conf))
    {
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "FULL-DUPLEX MAIN/SUB: downlink -> %s, uplink -> %s",
                    vfo_name(ctrl->conf->downlink_vfo),
                    vfo_name(ctrl->conf->uplink_vfo));
    }

    rigctrl_reset_send_tracking(ctrl, TRUE);

    /* invalidate sync with radio */
    ctrl->lastrxf = 0;
    ctrl->lasttxf = 0;
}

static void tx_track_toggle_cb(GtkToggleButton *button, gpointer data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);
    gboolean        requested;

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    requested = gtk_toggle_button_get_active(button);
    if (requested && !radio_apply_ui_settings(ctrl, TRUE))
    {
        rigctrl_ui_begin_update(ctrl, "tx_track_invalid_settings");
        gtk_toggle_button_set_active(button, FALSE);
        rigctrl_ui_end_update(ctrl, "tx_track_invalid_settings");
        return;
    }

    ctrl->tx_track_enabled = requested;
    rigctrl_sync_tracking_state(ctrl);

    sat_log_log(SAT_LOG_LEVEL_DEBUG, "SATMODE: tx tracking %s",
                ctrl->tx_track_enabled ? "on" : "off");

    if (ctrl->tracking && is_full_duplex_main_sub_configured(ctrl->conf))
    {
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "FULL-DUPLEX MAIN/SUB: downlink -> %s, uplink -> %s",
                    vfo_name(ctrl->conf->downlink_vfo),
                    vfo_name(ctrl->conf->uplink_vfo));
    }

    rigctrl_reset_send_tracking(ctrl, FALSE);

    /* invalidate sync with radio */
    ctrl->lastrxf = 0;
    ctrl->lasttxf = 0;
}

/* Called when the user changes the value of the cycle delay */
static void delay_changed_cb(GtkSpinButton * spin, gpointer data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    ctrl->delay = (guint) gtk_spin_button_get_value(spin);
    if (ctrl->conf)
        ctrl->conf->cycle = ctrl->delay;

    if (ctrl->engaged && ctrl->conn_state == RIGCTRL_CONN_CONNECTED)
        start_timer(ctrl);
}

static gboolean rigctrl_parse_spin_value(GtkSpinButton *spin, gdouble *value)
{
    const gchar *text;
    gchar *end = NULL;
    gdouble val;

    if (spin == NULL || value == NULL)
        return FALSE;

    text = gtk_entry_get_text(GTK_ENTRY(spin));
    if (text == NULL)
        return FALSE;

    errno = 0;
    val = g_ascii_strtod(text, &end);
    if (text == end || errno == ERANGE)
        return FALSE;

    while (g_ascii_isspace(*end))
        end++;

    if (*end != '\0')
        return FALSE;

    *value = val;
    return TRUE;
}

/* Authoritative UI apply path used before engage/track starts. */
static gboolean radio_apply_ui_settings(GtkRigCtrl *ctrl, gboolean strict)
{
    GtkSpinButton *spin;
    GtkAdjustment *adj;
    gdouble raw = 0.0;
    gdouble lower;
    gdouble upper;
    gdouble value;
    guint delay_ms;

    if (ctrl == NULL || ctrl->cycle_spin == NULL)
        return TRUE;

    spin = GTK_SPIN_BUTTON(ctrl->cycle_spin);
    adj = gtk_spin_button_get_adjustment(spin);
    lower = gtk_adjustment_get_lower(adj);
    upper = gtk_adjustment_get_upper(adj);

    if (strict)
    {
        if (!rigctrl_parse_spin_value(spin, &raw))
        {
            rig_show_error_dialog(
                ctrl,
                _("Invalid cycle delay"),
                _("Cycle delay must be a valid number."));
            return FALSE;
        }
        if (raw < lower || raw > upper)
        {
            gchar *msg = g_strdup_printf(_("Cycle delay must be between %.0f and %.0f ms."),
                                         lower, upper);
            rig_show_error_dialog(ctrl, _("Invalid cycle delay"), msg);
            g_free(msg);
            return FALSE;
        }
        gtk_spin_button_set_value(spin, raw);
    }

    gtk_spin_button_update(spin);
    value = gtk_spin_button_get_value(spin);
    delay_ms = (guint)llround(value);

    ctrl->delay = delay_ms;
    if (ctrl->conf)
        ctrl->conf->cycle = ctrl->delay;

    if (ctrl->engaged && ctrl->conn_state == RIGCTRL_CONN_CONNECTED)
        start_timer(ctrl);

    if (strict)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "Applied radio settings: cycle_ms=%u",
                    delay_ms);
        rig_term_log(ctrl, "gpredict:rx",
                     "applied radio settings: cycle_ms=%u",
                     delay_ms);
    }

    return TRUE;
}

static gboolean rigctrl_cycle_focus_out_cb(GtkWidget *widget,
                                           GdkEventFocus *event,
                                           gpointer data)
{
    GtkRigCtrl *ctrl = GTK_RIG_CTRL(data);

    (void)widget;
    (void)event;

    if (ctrl == NULL || ctrl->ui_updating)
        return FALSE;

    radio_apply_ui_settings(ctrl, FALSE);
    return FALSE;
}

static void rigctrl_cycle_activate_cb(GtkEntry *entry, gpointer data)
{
    GtkRigCtrl *ctrl = GTK_RIG_CTRL(data);

    (void)entry;

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    radio_apply_ui_settings(ctrl, FALSE);
}

static void primary_rig_selected_cb(GtkComboBox * box, gpointer data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);
    gchar          *buff;
    gchar          *selected_id;

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                _("%s:%s: Primary device selected: %d"),
                __FILE__, __func__, gtk_combo_box_get_active(box));

    selected_id = rigctrl_combo_get_active_id(box, FALSE);
    rigctrl_set_selection_id(ctrl, "primary", &ctrl->primary_rig_id,
                             selected_id, FALSE);

    if (ctrl->conf != NULL)
    {
        free_radio_conf(ctrl->conf);
    }

    ctrl->conf = g_try_new(radio_conf_t, 1);
    if (ctrl->conf == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%d: Failed to allocate memory for radio config"),
                    __FILE__, __LINE__);
        return;
    }

    ctrl->conf->name =
        g_strdup(ctrl->primary_rig_id);
    if (radio_conf_read(ctrl->conf))
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s:%s: Loaded new radio configuration %s"),
                    __FILE__, __func__, ctrl->conf->name);

        rigctrl_reset_reconnect(ctrl, FALSE);

        rigctrl_ui_begin_update(ctrl, "primary_rig_selected");
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ctrl->cycle_spin),
                                  ctrl->conf->cycle);
        rigctrl_apply_log_level_from_conf(ctrl, ctrl->conf);
        rigctrl_ui_end_update(ctrl, "primary_rig_selected");

        /* update LO widgets */
        buff = g_strdup_printf(_("%.0f MHz"), ctrl->conf->lo / 1.0e6);
        gtk_label_set_text(GTK_LABEL(ctrl->LoDown), buff);
        g_free(buff);
        /* uplink LO only if single device */
        if (ctrl->conf2 == NULL)
        {
            buff = g_strdup_printf(_("%.0f MHz"), ctrl->conf->loup / 1.0e6);
            gtk_label_set_text(GTK_LABEL(ctrl->LoUp), buff);
            g_free(buff);
        }
    }
    else
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%s: Failed to load radio configuration %s"),
                    __FILE__, __func__, ctrl->conf->name);

        free_radio_conf(ctrl->conf);
        ctrl->conf = NULL;
    }
}

static void secondary_rig_selected_cb(GtkComboBox * box, gpointer data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);
    gchar          *buff;
    gchar          *selected_id;

    if (ctrl == NULL || ctrl->ui_updating)
        return;


    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                _("%s:%s: Secondary device selected: %d"),
                __FILE__, __func__, gtk_combo_box_get_active(box));

    selected_id = rigctrl_combo_get_active_id(box, TRUE);

    if (ctrl->conf2 != NULL)
    {
        free_radio_conf(ctrl->conf2);
        ctrl->conf2 = NULL;
    }

    if (selected_id == NULL)
    {
        /* first entry is "None" */
        rigctrl_set_selection_id(ctrl, "secondary", &ctrl->secondary_rig_id,
                                 NULL, FALSE);
        rigctrl_reset_reconnect(ctrl, TRUE);

        /* reset uplink LO to what's in ctrl->conf */
        if (ctrl->conf != NULL)
        {
            buff = g_strdup_printf(_("%.0f MHz"), ctrl->conf->loup / 1.0e6);
            gtk_label_set_text(GTK_LABEL(ctrl->LoUp), buff);
            g_free(buff);
        }

        return;
    }

    /* ensure that selected secondary rig is not the same as the primary */
    if (!g_strcmp0(ctrl->primary_rig_id, selected_id))
    {
        if (is_full_duplex_main_sub_configured(ctrl->conf))
        {
            /* Allow same rig: IC-9700 uses dual VFOs in SAT mode. */
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "FULL-DUPLEX MAIN/SUB: using primary rig for uplink (dual VFO)");
            rigctrl_set_selection_id(ctrl, "secondary",
                                     &ctrl->secondary_rig_id,
                                     selected_id, FALSE);
            if (ctrl->conf != NULL)
            {
                buff = g_strdup_printf(_("%.0f MHz"), ctrl->conf->loup / 1.0e6);
                gtk_label_set_text(GTK_LABEL(ctrl->LoUp), buff);
                g_free(buff);
            }

            return;
        }

        /* selected conf is the same as the primary one */
        if (ctrl->conf != NULL)
        {
            buff = g_strdup_printf(_("%.0f MHz"), ctrl->conf->loup / 1.0e6);
            gtk_label_set_text(GTK_LABEL(ctrl->LoUp), buff);
            g_free(buff);
        }
        rigctrl_combo_set_active_blocked(GTK_COMBO_BOX(ctrl->DevSel2), 0,
                                         G_CALLBACK(secondary_rig_selected_cb),
                                         ctrl);
        rigctrl_set_selection_id(ctrl, "secondary", &ctrl->secondary_rig_id,
                                 NULL, FALSE);
        rigctrl_warn_shared_uplink_mode(ctrl, ctrl->conf);
        g_free(selected_id);

        return;
    }

    rigctrl_set_selection_id(ctrl, "secondary", &ctrl->secondary_rig_id,
                             selected_id, FALSE);

    /* else load new device */
    ctrl->conf2 = g_try_new(radio_conf_t, 1);
    if (ctrl->conf2 == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%s: Failed to allocate memory for radio config"),
                    __FILE__, __func__);
        return;
    }

    /* load new configuration */
    ctrl->conf2->name =
        g_strdup(ctrl->secondary_rig_id);
    if (radio_conf_read(ctrl->conf2))
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s:%s: Loaded new radio configuration %s"),
                    __FILE__, __func__, ctrl->conf2->name);

        rigctrl_reset_reconnect(ctrl, TRUE);

        buff = g_strdup_printf(_("%.0f MHz"), ctrl->conf2->loup / 1.0e6);
        gtk_label_set_text(GTK_LABEL(ctrl->LoUp), buff);
        g_free(buff);
    }
    else
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%s: Failed to load radio configuration %s"),
                    __FILE__, __func__, ctrl->conf->name);

        free_radio_conf(ctrl->conf2);
        ctrl->conf2 = NULL;
    }
}

static void rig_engaged_cb(GtkToggleButton * button, gpointer data)
{
    GtkRigCtrl *ctrl = GTK_RIG_CTRL(data);

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    if (!gtk_toggle_button_get_active(button))
    {
        /* Disengage: close socket / stop worker thread */
        if (ctrl->DevSel != NULL)
            gtk_widget_set_sensitive(ctrl->DevSel, TRUE);
        if (ctrl->DevSel2 != NULL)
            gtk_widget_set_sensitive(ctrl->DevSel2, TRUE);
        ctrl->engaged = FALSE;
        ctrl->engage_pending = FALSE;
        rigctrl_cancel_open_task(ctrl);
        rigctrl_reset_doppler_smoothing(ctrl);
        if (!ctrl->ui_hard_error)
        {
            rigctrl_clear_ui_hard_error(ctrl);
            if (ctrl->link_lost_latched || ctrl->link_lost_latched2)
            {
                rigctrl_set_status_detail(
                    ctrl,
                    _("Radio link lost. Check radio and press Engage to retry."));
            }
            else
            {
                rigctrl_set_status_detail(ctrl, NULL);
            }
            rig_ui_command_window_init(&ctrl->ui_cmd_window);
        }
        rig_term_log(ctrl, "gpredict", "disengage");

        /* Notify worker thread about the new configuration/state */
        if (ctrl->rigctlq != NULL)
            setconfig(ctrl);
    }
    else
    {
        /* Apply UI settings before starting any worker activity. */
        if (!radio_apply_ui_settings(ctrl, TRUE))
        {
            rigctrl_ui_begin_update(ctrl, "engage_invalid_settings");
            gtk_toggle_button_set_active(button, FALSE);
            rigctrl_ui_end_update(ctrl, "engage_invalid_settings");
            return;
        }

        /* User-initiated engage: clear error gating for a fresh attempt. */
        rigctrl_reset_error_gates(ctrl);
        rigctrl_clear_ui_hard_error(ctrl);
        rigctrl_set_status_detail(ctrl, NULL);
        rig_ui_command_window_init(&ctrl->ui_cmd_window);
        ctrl->engage_pending = TRUE;

        if (ctrl->conf == NULL)
        {
            /* We don't have a working configuration – inform the user and
             * immediately revert the toggle.
             */
            rig_show_error_dialog(
                ctrl,
                _("Unable to engage radio"),
                _("No radio configuration is selected. See log for details."));
            rig_term_log(ctrl, "gpredict:err",
                         "no radio config selected; open Interfaces -> Radios");
            rigctrl_fail_engage(ctrl, "no radio configuration selected");
            return;
        }

        if (!rigctrl_validate_mode(ctrl, ctrl->conf, _("receiver")))
        {
            rigctrl_fail_engage(ctrl, "receiver mode validation failed");
            return;
        }

        if (ctrl->conf2 != NULL &&
            !rigctrl_validate_mode(ctrl, ctrl->conf2, _("uplink")))
        {
            rigctrl_fail_engage(ctrl, "uplink mode validation failed");
            return;
        }

        /* Engage: start worker thread */
        if (ctrl->DevSel != NULL)
            gtk_widget_set_sensitive(ctrl->DevSel, FALSE);
        if (ctrl->DevSel2 != NULL)
            gtk_widget_set_sensitive(ctrl->DevSel2, FALSE);
        ctrl->engaged = TRUE;
        rig_term_log(ctrl, "gpredict", "engage");

        /* Start worker thread if not already running */
        if (ctrl->rigctl_thread == NULL)
        {
            ctrl->rigctlq = g_async_queue_new();
            ctrl->rigctl_thread_done = FALSE;
            ctrl->rigctl_thread =
                g_thread_new("rigctl_run", rigctl_run, ctrl);
        }

        rigctrl_request_open(ctrl, "engage");

        /* Push initial configuration to the worker */
        setconfig(ctrl);
    }

    /* Always clear secondary rig configuration when toggling engage
     * state; this mirrors the previous behaviour.
     */
    ctrl->conf2 = NULL;

    rigctrl_update_freq_display(ctrl);
    rigctrl_queue_ui_status_refresh(ctrl, "engage toggled");
}

static void rigctrl_combo_set_ellipsize(GtkComboBox *combo)
{
    GList *cells = NULL;
    GList *iter = NULL;

    if (combo == NULL)
        return;

    cells = gtk_cell_layout_get_cells(GTK_CELL_LAYOUT(combo));
    for (iter = cells; iter != NULL; iter = iter->next)
    {
        if (GTK_IS_CELL_RENDERER_TEXT(iter->data))
        {
            g_object_set(iter->data,
                         "ellipsize", PANGO_ELLIPSIZE_END,
                         NULL);
        }
    }
    g_list_free(cells);
}

static GtkWidget *create_target_widgets(GtkRigCtrl * ctrl)
{
    GtkWidget      *frame, *table, *label;
    GtkWidget      *rx_track, *tx_track, *track_box;
    GtkWidget      *trsp_label;
    GtkSizeGroup   *combo_group;
    gchar          *buff;
    guint           i, n;
    sat_t          *sat = NULL;

    buff = g_strdup_printf(AZEL_FMTSTR, 0.0);

    table = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(table), 5);
    gtk_grid_set_column_spacing(GTK_GRID(table), 8);
    gtk_grid_set_row_spacing(GTK_GRID(table), 8);

    label = gtk_label_new(_("Target preset:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 0, 1, 1);

    combo_group = gtk_size_group_new(GTK_SIZE_GROUP_HORIZONTAL);

    /* sat selector */
    ctrl->SatSel = gtk_combo_box_text_new();
    n = g_slist_length(ctrl->sats);
    for (i = 0; i < n; i++)
    {
        sat = SAT(g_slist_nth_data(ctrl->sats, i));
        if (sat)
            gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ctrl->SatSel),
                                           sat->nickname);
    }

    gtk_combo_box_set_active(GTK_COMBO_BOX(ctrl->SatSel), 0);
    gtk_combo_box_set_popup_fixed_width(GTK_COMBO_BOX(ctrl->SatSel), TRUE);
    rigctrl_combo_set_ellipsize(GTK_COMBO_BOX(ctrl->SatSel));
    gtk_widget_set_hexpand(ctrl->SatSel, TRUE);
    gtk_widget_set_halign(ctrl->SatSel, GTK_ALIGN_FILL);
    gtk_widget_set_tooltip_text(ctrl->SatSel, _("Select target object"));
    g_signal_connect(ctrl->SatSel, "changed", G_CALLBACK(sat_selected_cb),
                     ctrl);
    rigctrl_register_combo_quarantine(GTK_COMBO_BOX(ctrl->SatSel));
    gtk_grid_attach(GTK_GRID(table), ctrl->SatSel, 1, 0, 3, 1);
    gtk_size_group_add_widget(combo_group, ctrl->SatSel);

    /* Service/payload preset selector, apply, and lock buttons */
    trsp_label = gtk_label_new(_("Payload preset:"));
    g_object_set(trsp_label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), trsp_label, 0, 1, 1, 1);

    ctrl->TrspSel = gtk_combo_box_text_new();
    gtk_combo_box_set_popup_fixed_width(GTK_COMBO_BOX(ctrl->TrspSel), TRUE);
    gtk_widget_set_hexpand(ctrl->TrspSel, TRUE);
    gtk_widget_set_halign(ctrl->TrspSel, GTK_ALIGN_FILL);
    gtk_widget_set_tooltip_text(ctrl->TrspSel,
                                _("Select a payload preset. "
                                  "Its baseline RX/TX frequencies are applied."));
    load_trsp_list(ctrl);
    g_signal_connect(ctrl->TrspSel, "realize",
                     G_CALLBACK(rigctrl_trsp_combo_realize), NULL);
    g_signal_connect(ctrl->TrspSel, "changed", G_CALLBACK(trsp_selected_cb),
                     ctrl);
    rigctrl_register_combo_quarantine(GTK_COMBO_BOX(ctrl->TrspSel));
    gtk_grid_attach(GTK_GRID(table), ctrl->TrspSel, 1, 1, 3, 1);
    gtk_size_group_add_widget(combo_group, ctrl->TrspSel);

    /* tracking toggles */
    rx_track = gtk_toggle_button_new_with_label(_("RX Track"));
    gtk_widget_set_tooltip_text(rx_track,
                                _("Apply Doppler correction to the RX "
                                  "frequency."));
    g_signal_connect(rx_track, "toggled", G_CALLBACK(rx_track_toggle_cb), ctrl);

    tx_track = gtk_toggle_button_new_with_label(_("TX Track"));
    gtk_widget_set_tooltip_text(tx_track,
                                _("Apply Doppler correction to the TX "
                                  "frequency."));
    g_signal_connect(tx_track, "toggled", G_CALLBACK(tx_track_toggle_cb), ctrl);

    track_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    gtk_box_pack_start(GTK_BOX(track_box), rx_track, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(track_box), tx_track, TRUE, TRUE, 0);
    gtk_widget_set_hexpand(track_box, TRUE);
    gtk_widget_set_halign(track_box, GTK_ALIGN_FILL);
    gtk_grid_attach(GTK_GRID(table), track_box, 1, 2, 3, 1);

    /* Azimuth */
    label = gtk_label_new(_("Az:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 3, 1, 1);
    ctrl->SatAz = gtk_label_new(buff);
    g_object_set(ctrl->SatAz, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), ctrl->SatAz, 1, 3, 1, 1);

    /* Elevation */
    label = gtk_label_new(_("El:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 4, 1, 1);
    ctrl->SatEl = gtk_label_new(buff);
    g_object_set(ctrl->SatEl, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), ctrl->SatEl, 1, 4, 1, 1);

    /* Range */
    label = gtk_label_new(_(" Range:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 2, 3, 1, 1);
    ctrl->SatRng = gtk_label_new("0 km");
    g_object_set(ctrl->SatRng, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), ctrl->SatRng, 3, 3, 1, 1);

    gtk_widget_set_tooltip_text(label,
                                _("This is the current distance between the "
                                  "satellite and the observer."));
    gtk_widget_set_tooltip_text(ctrl->SatRng,
                                _("This is the current distance between the "
                                  "satellite and the observer."));

    /* Range rate */
    label = gtk_label_new(_(" Rate:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 2, 4, 1, 1);
    ctrl->SatRngRate = gtk_label_new("0.0 km/s");
    g_object_set(ctrl->SatRngRate, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), ctrl->SatRngRate, 3, 4, 1, 1);

    gtk_widget_set_tooltip_text(label,
                                _("The rate of change for the distance between"
                                  " the satellite and the observer."));
    gtk_widget_set_tooltip_text(ctrl->SatRngRate,
                                _("The rate of change for the distance between"
                                  " the satellite and the observer."));

    frame = gtk_frame_new(_("Target"));
    gtk_widget_set_hexpand(frame, TRUE);
    gtk_widget_set_halign(frame, GTK_ALIGN_FILL);
    gtk_container_add(GTK_CONTAINER(frame), table);
    g_object_unref(combo_group);
    g_free(buff);

    return frame;
}

static gboolean is_rig_tx_capable(const gchar * confname)
{
    radio_conf_t   *conf = NULL;
    gboolean        cantx = FALSE;

    conf = g_try_new(radio_conf_t, 1);
    if (conf == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%d: Failed to allocate memory for radio config"),
                    __FILE__, __LINE__);
        return FALSE;
    }

    /* load new configuration */
    conf->name = g_strdup(confname);
    if (radio_conf_read(conf))
    {
        cantx = (conf->type == RIG_TYPE_RX) ? FALSE : TRUE;
    }
    else
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%d: Error reading radio configuration %s"),
                    __FILE__, __LINE__, confname);

        cantx = FALSE;
    }

    free_radio_conf(conf);

    return cantx;
}

/* Sort the list of satellites in the combo box. */
static gint sat_name_compare(sat_t * a, sat_t * b)
{
    return (gpredict_strcmp(a->nickname, b->nickname));
}

/* Sort the list of rigs in the combo box */
static gint rig_name_compare(const gchar * a, const gchar * b)
{
    return (gpredict_strcmp(a, b));
}

static GSList *rigctrl_collect_rig_names(gboolean tx_only, gboolean sorted)
{
    GDir           *dir = NULL;
    GError         *error = NULL;
    gchar          *dirname;
    const gchar    *filename;
    gchar         **vbuff;
    GSList         *rigs = NULL;

    dirname = get_hwconf_dir();
    dir = g_dir_open(dirname, 0, &error);
    if (dir)
    {
        while ((filename = g_dir_read_name(dir)))
        {
            if (g_str_has_suffix(filename, ".rig"))
            {
                vbuff = g_strsplit(filename, ".rig", 0);
                if (!tx_only || is_rig_tx_capable(vbuff[0]))
                {
                    if (sorted)
                        rigs = g_slist_insert_sorted(rigs, g_strdup(vbuff[0]),
                                                     (GCompareFunc) rig_name_compare);
                    else
                        rigs = g_slist_append(rigs, g_strdup(vbuff[0]));
                }
                g_strfreev(vbuff);
            }
        }
        g_dir_close(dir);
    }
    else
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%d: Failed to open hwconf dir (%s)"),
                    __FILE__, __LINE__, error->message);
        g_clear_error(&error);
    }

    g_free(dirname);

    return rigs;
}

static void rigctrl_free_name_list(GSList *names)
{
    GSList *iter;

    for (iter = names; iter != NULL; iter = iter->next)
        g_free(iter->data);

    g_slist_free(names);
}

static gint rigctrl_combo_find_index(GtkComboBox *box, const gchar *id)
{
    GtkTreeModel *model;
    GtkTreeIter iter;
    gint index = 0;

    if (box == NULL || id == NULL)
        return -1;

    model = gtk_combo_box_get_model(box);
    if (model == NULL || !gtk_tree_model_get_iter_first(model, &iter))
        return -1;

    do
    {
        gchar *text = NULL;

        gtk_tree_model_get(model, &iter, 0, &text, -1);
        if (g_strcmp0(text, id) == 0)
        {
            g_free(text);
            return index;
        }
        g_free(text);
        index++;
    } while (gtk_tree_model_iter_next(model, &iter));

    return -1;
}

static void rigctrl_combo_restore_selection(GtkComboBox *box,
                                            const gchar *id,
                                            gboolean allow_none,
                                            GCallback cb,
                                            gpointer data)
{
    GtkTreeModel *model;
    gint index = -1;
    gint count = 0;

    if (box == NULL)
        return;

    if (allow_none && (id == NULL || *id == '\0'))
        index = 0;
    else
        index = rigctrl_combo_find_index(box, id);

    model = gtk_combo_box_get_model(box);
    if (model != NULL)
        count = gtk_tree_model_iter_n_children(model, NULL);

    if (count <= 0)
        return;

    if (index < 0)
        index = 0;

    rigctrl_combo_set_active_blocked(box, index, cb, data);
}

typedef struct
{
    GtkRigCtrl *ctrl;
    gboolean rebuilt;
} RigCtrlDeviceRefresh;

static guint rigctrl_device_refresh_id(GtkRigCtrl *ctrl)
{
    if (ctrl == NULL)
        return 0;

    return GPOINTER_TO_UINT(
        g_object_get_data(G_OBJECT(ctrl), "rigctrl-device-refresh-id"));
}

static void rigctrl_set_device_refresh_id(GtkRigCtrl *ctrl, guint id)
{
    if (ctrl == NULL)
        return;

    g_object_set_data(G_OBJECT(ctrl), "rigctrl-device-refresh-id",
                      GUINT_TO_POINTER(id));
}

static gboolean rigctrl_device_selectors_refresh_idle(gpointer data)
{
    RigCtrlDeviceRefresh *job = data;
    GtkRigCtrl *ctrl = job ? job->ctrl : NULL;

    if (ctrl == NULL)
    {
        g_free(job);
        return G_SOURCE_REMOVE;
    }

    if (ctrl->destroying)
    {
        rigctrl_set_device_refresh_id(ctrl, 0);
        g_object_unref(ctrl);
        g_free(job);
        return G_SOURCE_REMOVE;
    }

    if (rigctrl_combo_popup_shown(GTK_COMBO_BOX(ctrl->DevSel)) ||
        rigctrl_combo_popup_shown(GTK_COMBO_BOX(ctrl->DevSel2)))
        return G_SOURCE_CONTINUE;

    rigctrl_set_device_refresh_id(ctrl, 0);
    rigctrl_rebuild_device_selectors(ctrl, job->rebuilt);
    g_object_unref(ctrl);
    g_free(job);
    return G_SOURCE_REMOVE;
}

static void rigctrl_schedule_device_selectors_refresh(GtkRigCtrl *ctrl,
                                                      gboolean rebuilt)
{
    RigCtrlDeviceRefresh *job;
    guint id;

    if (ctrl == NULL || ctrl->destroying)
        return;

    if (rigctrl_device_refresh_id(ctrl) != 0)
        return;

    job = g_new0(RigCtrlDeviceRefresh, 1);
    job->ctrl = g_object_ref(ctrl);
    job->rebuilt = rebuilt;
    id = g_idle_add_full(G_PRIORITY_DEFAULT_IDLE,
                         rigctrl_device_selectors_refresh_idle,
                         job,
                         NULL);
    if (id == 0)
    {
        g_object_unref(ctrl);
        g_free(job);
        return;
    }

    rigctrl_set_device_refresh_id(ctrl, id);
}

static void rigctrl_rebuild_device_selectors(GtkRigCtrl *ctrl,
                                             gboolean rebuilt)
{
    GSList *rigs;
    GSList *tx_rigs;
    GSList *iter;
    GtkListStore *primary_model = NULL;
    GtkListStore *secondary_model = NULL;
    GtkTreeIter tree_iter;
    guint primary_rows = 0;
    guint secondary_rows = 0;
    gboolean was_updating = FALSE;

    if (ctrl == NULL || ctrl->DevSel == NULL || ctrl->DevSel2 == NULL)
        return;

    if (rigctrl_combo_popup_shown(GTK_COMBO_BOX(ctrl->DevSel)) ||
        rigctrl_combo_popup_shown(GTK_COMBO_BOX(ctrl->DevSel2)))
    {
        rigctrl_schedule_device_selectors_refresh(ctrl, rebuilt);
        return;
    }

    rigs = rigctrl_collect_rig_names(FALSE, TRUE);
    tx_rigs = rigctrl_collect_rig_names(TRUE, FALSE);

    primary_model = gtk_list_store_new(1, G_TYPE_STRING);
    for (iter = rigs; iter != NULL; iter = iter->next)
    {
        gtk_list_store_append(primary_model, &tree_iter);
        gtk_list_store_set(primary_model, &tree_iter, 0, iter->data, -1);
        primary_rows++;
    }

    secondary_model = gtk_list_store_new(1, G_TYPE_STRING);
    gtk_list_store_append(secondary_model, &tree_iter);
    gtk_list_store_set(secondary_model, &tree_iter, 0, _("None"), -1);
    secondary_rows++;
    for (iter = tx_rigs; iter != NULL; iter = iter->next)
    {
        gtk_list_store_append(secondary_model, &tree_iter);
        gtk_list_store_set(secondary_model, &tree_iter, 0, iter->data, -1);
        secondary_rows++;
    }

    if (rigctrl_log_at_least(ctrl, RIG_LOG_VERBOSE))
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rigctrl rebuild device combos primary=%u secondary=%u requested_primary=%s requested_secondary=%s",
                    primary_rows, secondary_rows,
                    rigctrl_id_for_log(ctrl->primary_rig_id),
                    rigctrl_id_for_log(ctrl->secondary_rig_id));

    was_updating = ctrl->ui_updating;
    rigctrl_ui_begin_update(ctrl, "device_selectors");
    g_signal_handlers_block_by_func(ctrl->DevSel,
                                    (gpointer)G_CALLBACK(primary_rig_selected_cb), ctrl);
    g_signal_handlers_block_by_func(ctrl->DevSel2,
                                    (gpointer)G_CALLBACK(secondary_rig_selected_cb), ctrl);

    gtk_combo_box_set_model(GTK_COMBO_BOX(ctrl->DevSel),
                            GTK_TREE_MODEL(primary_model));
    gtk_combo_box_set_model(GTK_COMBO_BOX(ctrl->DevSel2),
                            GTK_TREE_MODEL(secondary_model));

    rigctrl_combo_restore_selection(GTK_COMBO_BOX(ctrl->DevSel),
                                    ctrl->primary_rig_id, FALSE,
                                    G_CALLBACK(primary_rig_selected_cb), ctrl);
    rigctrl_combo_restore_selection(GTK_COMBO_BOX(ctrl->DevSel2),
                                    ctrl->secondary_rig_id, TRUE,
                                    G_CALLBACK(secondary_rig_selected_cb), ctrl);

    g_signal_handlers_unblock_by_func(ctrl->DevSel,
                                      (gpointer)G_CALLBACK(primary_rig_selected_cb), ctrl);
    g_signal_handlers_unblock_by_func(ctrl->DevSel2,
                                      (gpointer)G_CALLBACK(secondary_rig_selected_cb), ctrl);
    if (!was_updating)
        rigctrl_ui_end_update(ctrl, "device_selectors");

    if (rigctrl_log_at_least(ctrl, RIG_LOG_VERBOSE))
    {
        gchar *active_primary =
            rigctrl_combo_get_active_id(GTK_COMBO_BOX(ctrl->DevSel), FALSE);
        gchar *active_secondary =
            rigctrl_combo_get_active_id(GTK_COMBO_BOX(ctrl->DevSel2), TRUE);

        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rigctrl device combo active primary=%s secondary=%s",
                    rigctrl_id_for_log(active_primary),
                    rigctrl_id_for_log(active_secondary));
        g_free(active_primary);
        g_free(active_secondary);
    }

    g_object_unref(primary_model);
    g_object_unref(secondary_model);

    rigctrl_set_selection_id(ctrl, "primary", &ctrl->primary_rig_id,
                             rigctrl_combo_get_active_id(GTK_COMBO_BOX(ctrl->DevSel),
                                                         FALSE),
                             rebuilt);
    rigctrl_set_selection_id(ctrl, "secondary", &ctrl->secondary_rig_id,
                             rigctrl_combo_get_active_id(GTK_COMBO_BOX(ctrl->DevSel2),
                                                         TRUE),
                             rebuilt);

    rigctrl_free_name_list(rigs);
    rigctrl_free_name_list(tx_rigs);
}

static GtkWidget *create_conf_widgets(GtkRigCtrl * ctrl)
{
    const gint      action_panel_width = 240;
    GtkWidget      *frame, *table, *label;
    GtkWidget      *downlink_row, *uplink_row, *cycle_row;
    GtkWidget      *engage_panel, *status_panel;
    GtkWidget      *status_box, *status_led;
    GtkWidget      *logging_row;
    GtkSizeGroup   *left_label_group;

    table = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(table), 5);
    gtk_grid_set_column_spacing(GTK_GRID(table), 8);
    gtk_grid_set_row_spacing(GTK_GRID(table), 8);
    left_label_group = gtk_size_group_new(GTK_SIZE_GROUP_HORIZONTAL);

    downlink_row = gtk_grid_new();
    gtk_grid_set_column_spacing(GTK_GRID(downlink_row), 5);
    gtk_grid_set_row_spacing(GTK_GRID(downlink_row), 5);
    gtk_widget_set_hexpand(downlink_row, TRUE);

    uplink_row = gtk_grid_new();
    gtk_grid_set_column_spacing(GTK_GRID(uplink_row), 5);
    gtk_grid_set_row_spacing(GTK_GRID(uplink_row), 5);
    gtk_widget_set_hexpand(uplink_row, TRUE);

    cycle_row = gtk_grid_new();
    gtk_grid_set_column_spacing(GTK_GRID(cycle_row), 5);
    gtk_grid_set_row_spacing(GTK_GRID(cycle_row), 5);
    gtk_widget_set_hexpand(cycle_row, FALSE);
    gtk_widget_set_halign(cycle_row, GTK_ALIGN_START);

    /* Primary device */
    label = gtk_label_new(_("Downlink device:"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_widget_set_halign(label, GTK_ALIGN_START);
    gtk_size_group_add_widget(left_label_group, label);
    gtk_grid_attach(GTK_GRID(downlink_row), label, 0, 0, 1, 1);

    ctrl->DevSel = gtk_combo_box_text_new();
    g_object_add_weak_pointer(G_OBJECT(ctrl->DevSel),
                              (gpointer *)&ctrl->DevSel);
    gtk_widget_set_tooltip_text(ctrl->DevSel,
                                _("Select primary radio device."
                                  "This device will be used for downlink and "
                                  "uplink unless you select a secondary device"
                                  " for uplink"));

    /* Secondary device */
    label = gtk_label_new(_("Uplink device:"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_widget_set_halign(label, GTK_ALIGN_START);
    gtk_size_group_add_widget(left_label_group, label);
    gtk_grid_attach(GTK_GRID(uplink_row), label, 0, 0, 1, 1);

    ctrl->DevSel2 = gtk_combo_box_text_new();
    g_object_add_weak_pointer(G_OBJECT(ctrl->DevSel2),
                              (gpointer *)&ctrl->DevSel2);
    gtk_widget_set_tooltip_text(ctrl->DevSel2,
                                _("Select secondary radio device\n"
                                  "This device will be used for uplink"));

    rigctrl_rebuild_device_selectors(ctrl, FALSE);

    g_signal_connect(ctrl->DevSel, "changed",
                     G_CALLBACK(primary_rig_selected_cb), ctrl);
    rigctrl_register_combo_quarantine(GTK_COMBO_BOX(ctrl->DevSel));
    gtk_widget_set_hexpand(ctrl->DevSel, TRUE);
    gtk_widget_set_halign(ctrl->DevSel, GTK_ALIGN_FILL);
    gtk_grid_attach(GTK_GRID(downlink_row), ctrl->DevSel, 1, 0, 1, 1);
    g_signal_connect(ctrl->DevSel2, "changed",
                     G_CALLBACK(secondary_rig_selected_cb), ctrl);
    rigctrl_register_combo_quarantine(GTK_COMBO_BOX(ctrl->DevSel2));
    gtk_widget_set_hexpand(ctrl->DevSel2, TRUE);
    gtk_widget_set_halign(ctrl->DevSel2, GTK_ALIGN_FILL);
    gtk_grid_attach(GTK_GRID(uplink_row), ctrl->DevSel2, 1, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(table), downlink_row, 0, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(table), uplink_row, 0, 1, 1, 1);

    /* Logs toggle */
    ctrl->log_toggle = gtk_toggle_button_new_with_label(_("Show log"));
    gtk_widget_set_tooltip_text(ctrl->log_toggle,
                                _("Show or hide the radio control log"));
    g_signal_connect(ctrl->log_toggle, "toggled",
                     G_CALLBACK(rig_logs_toggle_cb), ctrl);
    if (ctrl->term_view != NULL)
        gp_term_view_set_visible(ctrl->term_view, FALSE);
    gtk_widget_set_halign(ctrl->log_toggle, GTK_ALIGN_START);
    gtk_widget_set_valign(ctrl->log_toggle, GTK_ALIGN_CENTER);
    gtk_grid_attach(GTK_GRID(table), ctrl->log_toggle, 2, 0, 1, 1);

    /* Verbose rig logging toggle */
    logging_row = gtk_grid_new();
    gtk_grid_set_column_spacing(GTK_GRID(logging_row), 5);
    gtk_grid_set_row_spacing(GTK_GRID(logging_row), 5);
    gtk_widget_set_halign(logging_row, GTK_ALIGN_START);

    label = gtk_label_new(_("Logging:"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_widget_set_halign(label, GTK_ALIGN_START);
    gtk_grid_attach(GTK_GRID(logging_row), label, 0, 0, 1, 1);

    ctrl->log_verbose_toggle =
        gtk_check_button_new_with_label(_("Verbose"));
    gtk_widget_set_tooltip_text(ctrl->log_verbose_toggle,
                                _("Enable detailed rig logging summaries"));
    g_signal_connect(ctrl->log_verbose_toggle, "toggled",
                     G_CALLBACK(rig_verbose_toggle_cb), ctrl);
    gtk_widget_set_halign(ctrl->log_verbose_toggle, GTK_ALIGN_START);
    gtk_widget_set_valign(ctrl->log_verbose_toggle, GTK_ALIGN_CENTER);
    gtk_grid_attach(GTK_GRID(logging_row), ctrl->log_verbose_toggle, 1, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(table), logging_row, 2, 1, 1, 1);

    rigctrl_sync_log_toggles(ctrl);

    /* Engage button */
    ctrl->LockBut = gtk_toggle_button_new_with_label(_("Engage"));
    g_object_add_weak_pointer(G_OBJECT(ctrl->LockBut),
                              (gpointer *)&ctrl->LockBut);
    gtk_widget_set_tooltip_text(ctrl->LockBut,
                                _("Engage the selected radio device"));
    g_signal_connect(ctrl->LockBut, "toggled", G_CALLBACK(rig_engaged_cb),
                     ctrl);
    gtk_widget_set_hexpand(ctrl->LockBut, TRUE);
    gtk_widget_set_halign(ctrl->LockBut, GTK_ALIGN_FILL);
    gtk_widget_set_valign(ctrl->LockBut, GTK_ALIGN_CENTER);
    engage_panel = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_widget_set_halign(engage_panel, GTK_ALIGN_CENTER);
    gtk_widget_set_size_request(engage_panel, action_panel_width, -1);
    gtk_box_pack_start(GTK_BOX(engage_panel), ctrl->LockBut, TRUE, TRUE, 0);
    gtk_grid_attach(GTK_GRID(table), engage_panel, 1, 0, 1, 1);

    /* cycle period */
    label = gtk_label_new(_("Cycle:"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_widget_set_halign(label, GTK_ALIGN_START);
    gtk_size_group_add_widget(left_label_group, label);
    gtk_grid_attach(GTK_GRID(cycle_row), label, 0, 0, 1, 1);

    ctrl->cycle_spin = gtk_spin_button_new_with_range(10, 10000, 10);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(ctrl->cycle_spin), 0);
    gtk_widget_set_tooltip_text(ctrl->cycle_spin,
                                _("This parameter controls the delay between "
                                  "commands sent to the rig."));
    g_signal_connect(ctrl->cycle_spin, "value-changed",
                     G_CALLBACK(delay_changed_cb), ctrl);
    g_signal_connect(ctrl->cycle_spin, "focus-out-event",
                     G_CALLBACK(rigctrl_cycle_focus_out_cb), ctrl);
    g_signal_connect(ctrl->cycle_spin, "activate",
                     G_CALLBACK(rigctrl_cycle_activate_cb), ctrl);
    gtk_widget_set_hexpand(ctrl->cycle_spin, FALSE);
    gtk_widget_set_halign(ctrl->cycle_spin, GTK_ALIGN_START);
    gtk_entry_set_width_chars(GTK_ENTRY(ctrl->cycle_spin), 5);
    gtk_widget_set_size_request(ctrl->cycle_spin, 125, -1);
    gtk_grid_attach(GTK_GRID(cycle_row), ctrl->cycle_spin, 1, 0, 1, 1);

    label = gtk_label_new(_("msec"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_widget_set_halign(label, GTK_ALIGN_START);
    gtk_widget_set_margin_start(label, 4);
    gtk_grid_attach(GTK_GRID(cycle_row), label, 2, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(table), cycle_row, 0, 2, 1, 1);

    /* status */
    status_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    gtk_widget_set_halign(status_box, GTK_ALIGN_CENTER);
    gtk_widget_set_margin_top(status_box, 5);
    gtk_widget_set_margin_bottom(status_box, 5);
    gtk_widget_set_margin_start(status_box, 8);
    gtk_widget_set_margin_end(status_box, 8);
    status_led = status_indicator_new();
    gtk_box_pack_start(GTK_BOX(status_box), status_led, FALSE, FALSE, 0);

    ctrl->status_label = gtk_label_new("DISENGAGED");
    g_object_set(ctrl->status_label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_label_set_ellipsize(GTK_LABEL(ctrl->status_label), PANGO_ELLIPSIZE_END);
    gtk_widget_set_halign(ctrl->status_label, GTK_ALIGN_START);
    {
        PangoAttrList *attrs = pango_attr_list_new();
        pango_attr_list_insert(attrs,
                               pango_attr_scale_new(1.30));
        pango_attr_list_insert(attrs,
                               pango_attr_weight_new(PANGO_WEIGHT_MEDIUM));
        gtk_label_set_attributes(GTK_LABEL(ctrl->status_label), attrs);
        pango_attr_list_unref(attrs);
    }
    gtk_box_pack_start(GTK_BOX(status_box), ctrl->status_label, FALSE, FALSE, 0);
    status_panel = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_widget_set_halign(status_panel, GTK_ALIGN_CENTER);
    gtk_widget_set_size_request(status_panel, action_panel_width, 42);
    gtk_box_pack_start(GTK_BOX(status_panel), status_box, TRUE, TRUE, 0);
    gtk_grid_attach(GTK_GRID(table), status_panel, 1, 1, 1, 1);
    ctrl->status_indicator_widget = status_led;
    g_object_add_weak_pointer(G_OBJECT(ctrl->status_label),
                              (gpointer *)&ctrl->status_label);
    g_object_add_weak_pointer(G_OBJECT(status_led),
                              (gpointer *)&ctrl->status_indicator_widget);
    g_object_unref(left_label_group);
    rigctrl_refresh_ui_status(ctrl, "widget init");

    frame = gtk_frame_new(_("Settings"));
    gtk_widget_set_hexpand(frame, TRUE);
    gtk_widget_set_halign(frame, GTK_ALIGN_FILL);
    gtk_container_add(GTK_CONTAINER(frame), table);

    /* load primary config */
    primary_rig_selected_cb(GTK_COMBO_BOX(ctrl->DevSel), ctrl);

    return frame;
}


/* Create count down widget */
static GtkWidget *create_count_down_widgets(GtkRigCtrl * ctrl)
{
    GtkWidget      *frame;

    /* create delta-t label */
    ctrl->SatCnt = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(ctrl->SatCnt),
                         _("<span size='large'><b>\316\224T: "
                           "00:00:00</b></span>"));
    gtk_widget_set_tooltip_text(ctrl->SatCnt,
                                _("The time remaining until the next AOS or "
                                  "LOS event"));
    g_object_set(ctrl->SatCnt, "xalign", 0.5f, "yalign", 0.5f, NULL);
    gtk_widget_set_margin_top(ctrl->SatCnt, 3);
    gtk_widget_set_margin_bottom(ctrl->SatCnt, 3);

    frame = gtk_frame_new(NULL);
    gtk_container_add(GTK_CONTAINER(frame), ctrl->SatCnt);

    return frame;
}

/* Copy satellite from hash table to singly linked list. */
static void store_sats(gpointer key, gpointer value, gpointer user_data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(user_data);
    sat_t          *sat = SAT(value);

    (void)key;

    ctrl->sats = g_slist_insert_sorted(ctrl->sats, sat,
                                       (GCompareFunc) sat_name_compare);
}

static GString *rigctld_rxbuf_for_socket(GtkRigCtrl *ctrl, gint sock,
                                         gboolean create)
{
    GString **target = NULL;

    if (ctrl == NULL)
        return NULL;

    if (sock == ctrl->sock)
        target = &ctrl->rigctld_rxbuf;
    else if (sock == ctrl->sock2)
        target = &ctrl->rigctld_rxbuf2;

    if (target == NULL)
        return NULL;

    if (*target == NULL && create)
        *target = rigctld_rxbuf_new(256);

    return *target;
}

static void rigctld_clear_rxbuf_for_socket(GtkRigCtrl *ctrl, gint sock)
{
    GString *buf = rigctld_rxbuf_for_socket(ctrl, sock, FALSE);

    if (buf != NULL)
        rigctld_rxbuf_clear(buf);
}

static gboolean _send_rigctld_command(GtkRigCtrl * ctrl, gint sock,
                                      gchar * buff, gchar * buffout,
                                      gint sizeout)
{
    HamlibResponseInfo info = { 0 };
    RigctldClient *client = rigctld_client_for_socket(ctrl, sock);
    gint rprt = 0;
    gboolean saw_rprt = FALSE;
    gboolean rprt_error = FALSE;
    gboolean ok = FALSE;

    if (client == NULL || buff == NULL)
        return FALSE;

    if (buffout && sizeout > 0)
        buffout[0] = '\0';

    rig_term_log_tx(ctrl, buff);
    if (rigctrl_log_at_least(ctrl, RIG_LOG_TRACE))
    {
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    _("%s:%s: sending %d bytes to rigctld as \"%s\""),
                    __FILE__, __func__, (gint) strlen(buff), buff);
    }

    rigctld_io_lock_acquire();
    ok = rigctld_client_request_raw(client, buff,
                                    buffout, (gsize) sizeout, &info);
    if (!ok)
    {
        gchar *trim_cmd = g_strdup(buff);
        gint err = info.err ? info.err : EIO;

        if (!rigctrl_log_at_least(ctrl, RIG_LOG_TRACE))
            rig_term_log_raw(ctrl, "gpredict:tx", buff, TRUE);

        if (trim_cmd)
        {
            g_strchomp(trim_cmd);
            g_strstrip(trim_cmd);
        }

        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: rigctld request failed"), __func__);

        if (err == EAGAIN || err == EWOULDBLOCK || err == ETIMEDOUT)
        {
            rig_term_log(ctrl, "gpredict:err",
                         "timeout waiting for rigctld reply cmd=%s",
                         trim_cmd ? trim_cmd : "(null)");
            rigctrl_schedule_status(ctrl,
                                    _("Rig control timeout (rigctld did not reply)"),
                                    TRUE,
                                    RIG_UI_CMD_LINK_FAIL);
        }
        else
        {
            rig_term_log(ctrl, "gpredict:err",
                         "rigctld request failed (%s) cmd=%s",
                         strerror(err),
                         trim_cmd ? trim_cmd : "(null)");
            rigctrl_schedule_status(ctrl,
                                    _("Command failed"),
                                    TRUE,
                                    RIG_UI_CMD_LINK_FAIL);
        }

        g_free(trim_cmd);
        rigctrl_handle_socket_error(ctrl, sock, "request");
        rigctld_io_lock_release();
        return FALSE;
    }

    if (buffout && sizeout > 0 && buffout[0] == '\0')
    {
        gchar *trim_cmd = g_strdup(buff);
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%s: Got 0 bytes from rigctld"), __FILE__, __func__);
        rig_term_log(ctrl, "gpredict:err",
                     "rigctld closed connection cmd=%s",
                     trim_cmd ? g_strchomp(trim_cmd) : "(null)");
        g_free(trim_cmd);
        rigctrl_schedule_status(ctrl,
                                _("Rigctld closed connection"),
                                TRUE,
                                RIG_UI_CMD_LINK_FAIL);
        rigctrl_handle_socket_error(ctrl, sock, "recv");
        rigctld_io_lock_release();
        return FALSE;
    }

    ctrl->wrops++;
    rig_term_log_rx(ctrl, buffout);

    if (info.saw_rprt)
    {
        saw_rprt = TRUE;
        rprt = info.rprt_code;
    }
    else if (rig_parse_rprt_code_any(buffout, &rprt))
    {
        saw_rprt = TRUE;
    }

    if (saw_rprt && rprt != 0)
    {
        gchar *trim_cmd = g_strdup(buff);
        gchar *trim_reply = g_strdup(buffout);
        char status_msg[64];

        rprt_error = TRUE;
        rig_term_log_err_rprt(ctrl, buff, buffout, rprt);
        if (!rigctrl_log_at_least(ctrl, RIG_LOG_TRACE))
        {
            rig_term_log_raw(ctrl, "gpredict:tx", buff, TRUE);
            rig_term_log_raw(ctrl, "gpredict:rx", buffout, TRUE);
        }

        if (trim_cmd)
        {
            g_strchomp(trim_cmd);
            g_strstrip(trim_cmd);
        }
        if (trim_reply)
        {
            g_strchomp(trim_reply);
            g_strstrip(trim_reply);
        }

        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("rigctld command failed: cmd=\"%s\" reply=\"%s\""),
                    (trim_cmd && *trim_cmd) ? trim_cmd : "(empty)",
                    (trim_reply && *trim_reply) ? trim_reply : "(empty)");

        g_snprintf(status_msg, sizeof(status_msg),
                   _("Command rejected (RPRT %d)"), rprt);
        rigctrl_schedule_status(ctrl,
                                status_msg,
                                TRUE,
                                RIG_UI_CMD_COMMAND_REJECT);

        g_free(trim_cmd);
        g_free(trim_reply);
    }

    if (!rprt_error)
        rigctrl_schedule_status(ctrl,
                                saw_rprt ? _("OK") : NULL,
                                FALSE,
                                RIG_UI_CMD_OK);

    rigctld_io_lock_release();
    return ok && !rprt_error;
}

static gboolean send_rigctld_command_internal(GtkRigCtrl * ctrl, gint sock,
                                              gchar * buff, gchar * buffout,
                                              gint sizeout,
                                              gboolean allow_unready)
{
    gboolean        retval;
    RigSession     *session = rig_session_for_socket(ctrl, sock);

    if (!allow_unready && session != NULL &&
        session->state != RIG_SESSION_READY)
    {
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rig session (%s) not ready; skipping cmd=%s",
                    session->label ? session->label : "rig",
                    buff ? buff : "(null)");
        rig_term_log(ctrl, "gpredict:err",
                     "rig session (%s) not ready; cmd skipped",
                     session->label ? session->label : "rig");
        return FALSE;
    }

    /* Enter critical section! */
    g_mutex_lock(&ctrl->writelock);

    retval = _send_rigctld_command(ctrl, sock, buff, buffout, sizeout);

    /* Leave critical section! */
    g_mutex_unlock(&ctrl->writelock);
    return (retval);
}

static gboolean send_rigctld_command(GtkRigCtrl * ctrl, gint sock,
                                     gchar * buff, gchar * buffout,
                                     gint sizeout)
{
    return send_rigctld_command_internal(ctrl, sock, buff, buffout,
                                         sizeout, FALSE);
}

static gboolean G_GNUC_UNUSED rig_session_send_command(GtkRigCtrl *ctrl,
                                                       gint sock,
                                                       const gchar *cmd,
                                                       gchar *buffout,
                                                       gint sizeout)
{
    gchar *tmp = NULL;
    gboolean ok;

    if (cmd == NULL)
        return FALSE;

    tmp = g_strdup(cmd);
    ok = send_rigctld_command_internal(ctrl, sock, tmp, buffout, sizeout, TRUE);
    g_free(tmp);

    return ok;
}

static inline gboolean check_set_response(gchar * buffback, gboolean retcode,
                                          const gchar * function)
{
    if (retcode == TRUE)
    {
        gint code = 0;
        gboolean has_rprt = rig_parse_rprt_code_any(buffback, &code);

        if (!has_rprt || code != 0)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s:%s: %s rigctld returned error (%s)"),
                        __FILE__, __func__, function, buffback);

            retcode = FALSE;
        }
    }

    return retcode;
}

static inline gboolean check_get_response(gchar * buffback, gboolean retcode,
                                          const gchar * function)
{
    if (retcode == TRUE)
    {
        gint code = 0;

        if (rig_parse_rprt_code_any(buffback, &code) && code != 0)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s:%s: %s rigctld returned error (%s)"),
                        __FILE__, __func__, function, buffback);

            retcode = FALSE;
        }
    }

    return retcode;
}

static const gchar *vfo_name(vfo_t vfo)
{
    switch (vfo)
    {
    case VFO_A:
        return "VFOA";
    case VFO_B:
        return "VFOB";
    case VFO_MAIN:
        return "Main";
    case VFO_SUB:
        return "Sub";
    default:
        return "Unknown";
    }
}

static const gchar *rigctld_conn_name(rigctld_conn_t conn)
{
    switch (conn)
    {
    case RIGCTLD_CONN_SERIAL:
        return "SERIAL";
    case RIGCTLD_CONN_TCP:
        return "TCP";
    default:
        return "UNKNOWN";
    }
}

static gboolean rigctld_vfo_map_refs(GtkRigCtrl *ctrl, gint sock,
                                     gchar ***main_ptr,
                                     gchar ***sub_ptr,
                                     gboolean **logged_ptr)
{
    if (ctrl == NULL || main_ptr == NULL || sub_ptr == NULL || logged_ptr == NULL)
        return FALSE;

    if (sock == ctrl->sock)
    {
        *main_ptr = &ctrl->rigctld_vfo_main_token;
        *sub_ptr = &ctrl->rigctld_vfo_sub_token;
        *logged_ptr = &ctrl->rigctld_vfo_map_logged;
        return TRUE;
    }
    if (sock == ctrl->sock2)
    {
        *main_ptr = &ctrl->rigctld_vfo_main_token2;
        *sub_ptr = &ctrl->rigctld_vfo_sub_token2;
        *logged_ptr = &ctrl->rigctld_vfo_map_logged2;
        return TRUE;
    }

    return FALSE;
}

static void rigctld_clear_vfo_map_for_socket(GtkRigCtrl *ctrl, gint sock)
{
    gchar **main_ptr = NULL;
    gchar **sub_ptr = NULL;
    gboolean *logged_ptr = NULL;

    if (!rigctld_vfo_map_refs(ctrl, sock, &main_ptr, &sub_ptr, &logged_ptr))
        return;

    g_free(*main_ptr);
    g_free(*sub_ptr);
    *main_ptr = NULL;
    *sub_ptr = NULL;
    if (logged_ptr)
        *logged_ptr = FALSE;
}

static void rigctld_reset_offset_state_for_socket(GtkRigCtrl *ctrl, gint sock)
{
    if (ctrl == NULL)
        return;

    if (sock == ctrl->sock)
    {
        ctrl->xit_supported = TRUE;
        ctrl->last_rit_valid = FALSE;
        ctrl->last_xit_valid = FALSE;
        ctrl->last_rit_offset = 0;
        ctrl->last_xit_offset = 0;
    }
    else if (sock == ctrl->sock2)
    {
        ctrl->xit_supported2 = TRUE;
        ctrl->last_xit_valid2 = FALSE;
        ctrl->last_xit_offset2 = 0;
    }
}

static gboolean rigctld_fetch_frequency_token(GtkRigCtrl *ctrl, gint sock,
                                              const gchar *token,
                                              gint64 *freq_out)
{
    gchar buffback[128];
    gboolean retcode;
    gchar **vbuff = NULL;
    gint64 freq = 0;
    gchar *cmd = NULL;

    if (freq_out)
        *freq_out = 0;

    if (ctrl == NULL)
        return FALSE;

    if (token != NULL && *token != '\0')
        cmd = g_strdup_printf("f %s\x0a", token);
    else
        cmd = g_strdup("f\x0a");

    retcode = send_rigctld_command(ctrl, sock, cmd, buffback, sizeof(buffback));
    g_free(cmd);
    retcode = check_get_response(buffback, retcode, __func__);
    if (!retcode)
        return FALSE;

    vbuff = g_strsplit(buffback, "\n", 3);
    if (vbuff[0])
    {
        gchar *endptr = NULL;
        freq = g_ascii_strtoll(vbuff[0], &endptr, 10);
        if (endptr == vbuff[0])
            freq = 0;
    }
    g_strfreev(vbuff);

    if (freq_out)
        *freq_out = freq;

    return freq > 0;
}

static gboolean rigctld_fetch_frequency(GtkRigCtrl *ctrl, gint sock,
                                        gint64 *freq_out)
{
    return rigctld_fetch_frequency_token(ctrl, sock, NULL, freq_out);
}

static gboolean rigctld_try_vfo_token(GtkRigCtrl *ctrl, gint sock,
                                      RigSession *session,
                                      const gchar *token)
{
    gchar *buff = NULL;
    gchar buffback[128];
    gboolean retcode = FALSE;
    gint64 verify = 0;

    if (ctrl == NULL || token == NULL || *token == '\0')
        return FALSE;

    if (session != NULL && session->strategy == RIG_STRATEGY_VFO_OPT_ARGS)
    {
        /* Reuse tokenized reads on rigs that already advertised stable
           VFO-option args support instead of perturbing the live VFO. */
        if (!rigctld_fetch_frequency_token(ctrl, sock, token, &verify))
            return FALSE;
    }
    else
    {
        buff = g_strdup_printf("V %s\x0a", token);
        retcode = send_rigctld_command(ctrl, sock, buff, buffback,
                                       sizeof(buffback));
        g_free(buff);
        retcode = check_set_response(buffback, retcode, __func__);
        if (!retcode)
            return FALSE;

        /* Avoid writing a probe frequency while mapping tokens. */
        if (!rigctld_fetch_frequency(ctrl, sock, &verify))
            return FALSE;
    }

    if (verify > 0 && session != NULL && session->vfo_working != NULL)
    {
        g_hash_table_replace(session->vfo_working,
                             g_strdup(token),
                             GINT_TO_POINTER(1));
    }

    return verify > 0;
}

static gboolean rigctld_prefer_main_sub_tokens(const GtkRigCtrl *ctrl)
{
    if (ctrl == NULL || ctrl->conf == NULL)
        return FALSE;

    if (is_full_duplex_main_sub_configured(ctrl->conf))
        return TRUE;

    return (ctrl->conf->uplink_vfo == VFO_MAIN &&
            ctrl->conf->downlink_vfo == VFO_SUB);
}

static gboolean rigctld_force_main_sub_tokens(const GtkRigCtrl *ctrl,
                                              const RigSession *session)
{
    if (ctrl == NULL || session == NULL)
        return FALSE;

    if (!rigctld_prefer_main_sub_tokens(ctrl))
        return FALSE;

    return (session->quirks & RIG_QUIRK_FORCE_MAIN_SUB) != 0;
}

static gboolean rigctld_should_retry_main_sub(GtkRigCtrl *ctrl,
                                              const RigSession *session,
                                              vfo_t vfo,
                                              gint rprt_code)
{
    if (ctrl == NULL || session == NULL)
        return FALSE;

    if (!rigctld_force_main_sub_tokens(ctrl, session))
        return FALSE;

    if (vfo != VFO_MAIN && vfo != VFO_SUB)
        return FALSE;

    return rprt_code != 0;
}

static gboolean rig_session_vfo_token_working(const RigSession *session,
                                              const gchar *token)
{
    GHashTableIter iter;
    gpointer key = NULL;
    gpointer value = NULL;

    if (session == NULL || token == NULL || *token == '\0' ||
        session->vfo_working == NULL)
        return FALSE;

    if (g_hash_table_contains(session->vfo_working, token))
        return TRUE;

    g_hash_table_iter_init(&iter, session->vfo_working);
    while (g_hash_table_iter_next(&iter, &key, &value))
    {
        const gchar *entry = key;

        (void)value;

        if (entry != NULL && g_ascii_strcasecmp(entry, token) == 0)
            return TRUE;
    }

    return FALSE;
}

static const gchar *rigctld_find_known_vfo_token(const RigSession *session,
                                                 const gchar * const *candidates)
{
    if (session == NULL || candidates == NULL)
        return NULL;

    for (gint i = 0; candidates[i] != NULL; i++)
    {
        if (rig_session_vfo_token_working(session, candidates[i]))
            return candidates[i];
    }

    return NULL;
}

static const gchar *rigctld_vfo_token(GtkRigCtrl *ctrl, gint sock, vfo_t vfo)
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
    const gchar *fallback = vfo_name(vfo);
    const gchar * const *candidates = NULL;
    const gchar * const *fallback_candidates = NULL;
    gchar **main_ptr = NULL;
    gchar **sub_ptr = NULL;
    gboolean *logged_ptr = NULL;
    gchar **target_ptr = NULL;
    RigSession *session = rig_session_for_socket_vfo(ctrl, sock, vfo);
    gboolean prefer_main_sub = rigctld_prefer_main_sub_tokens(ctrl);
    gboolean force_main_sub = rigctld_force_main_sub_tokens(ctrl, session);
    gboolean allow_unlisted = force_main_sub;

    if (ctrl == NULL || ctrl->conf == NULL)
        return fallback;

    if (vfo == VFO_NONE)
        return NULL;

    if (!is_full_duplex_main_sub_configured(ctrl->conf))
        return fallback;

    if (vfo != VFO_MAIN && vfo != VFO_SUB)
        return fallback;

    if (!rigctld_vfo_map_refs(ctrl, sock, &main_ptr, &sub_ptr, &logged_ptr))
        return fallback;

    if (rigctrl_reject_main_sub_without_vfo_strategy(
            ctrl, session, ctrl->conf,
            "Main/Sub requires rigctld VFO support; try rigctld --vfo"))
        return NULL;

    if (vfo == VFO_MAIN)
    {
        if (force_main_sub)
            candidates = main_candidates_strict;
        else if (prefer_main_sub)
            candidates = main_candidates_prefer;
        else
            candidates = main_candidates_default;
        if (force_main_sub)
            fallback_candidates = main_candidates_default;
        target_ptr = main_ptr;
    }
    else
    {
        if (force_main_sub)
            candidates = sub_candidates_strict;
        else if (prefer_main_sub)
            candidates = sub_candidates_prefer;
        else
            candidates = sub_candidates_default;
        if (force_main_sub)
            fallback_candidates = sub_candidates_default;
        target_ptr = sub_ptr;
    }

    if (target_ptr != NULL && *target_ptr != NULL)
    {
        if (force_main_sub &&
            (g_ascii_strcasecmp(*target_ptr, "VFOA") == 0 ||
             g_ascii_strcasecmp(*target_ptr, "VFOB") == 0))
        {
            g_free(*target_ptr);
            *target_ptr = NULL;
        }
        else
        {
            return *target_ptr;
        }
    }

    {
        gboolean mapped = FALSE;
        const gchar *known_token =
            rigctld_find_known_vfo_token(session, candidates);

        if (known_token == NULL && fallback_candidates != NULL)
            known_token = rigctld_find_known_vfo_token(session,
                                                       fallback_candidates);

        if (known_token != NULL)
        {
            *target_ptr = g_strdup(known_token);
            mapped = TRUE;
        }

        for (gint i = 0; !mapped && candidates[i] != NULL; i++)
        {
            if (!allow_unlisted &&
                session != NULL && session->vfo_candidates->len > 0 &&
                !rig_session_vfo_candidate_exists(session, candidates[i]))
                continue;

            if (rigctld_try_vfo_token(ctrl, sock, session, candidates[i]))
            {
                *target_ptr = g_strdup(candidates[i]);
                mapped = TRUE;
                break;
            }
        }

        if (!mapped && fallback_candidates != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_WARN,
                        "FULL-DUPLEX MAIN/SUB: %s tokens unavailable; falling back to VFOA/B",
                        vfo == VFO_MAIN ? "Main" : "Sub");
            for (gint i = 0; fallback_candidates[i] != NULL; i++)
            {
                if (!allow_unlisted &&
                    session != NULL && session->vfo_candidates->len > 0 &&
                    !rig_session_vfo_candidate_exists(session, fallback_candidates[i]))
                    continue;

                if (rigctld_try_vfo_token(ctrl, sock, session,
                                          fallback_candidates[i]))
                {
                    *target_ptr = g_strdup(fallback_candidates[i]);
                    break;
                }
            }
        }
    }

    if (target_ptr == NULL || *target_ptr == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    "FULL-DUPLEX MAIN/SUB: unable to map %s VFO token",
                    vfo == VFO_MAIN ? "Main" : "Sub");
        rig_term_log(ctrl, "gpredict:err",
                     "unable to map %s VFO token",
                     vfo == VFO_MAIN ? "Main" : "Sub");
        return NULL;
    }

    if (logged_ptr && !*logged_ptr && main_ptr != NULL && sub_ptr != NULL &&
        *main_ptr != NULL && *sub_ptr != NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "FULL-DUPLEX MAIN/SUB: mapped Main->%s Sub->%s",
                    *main_ptr, *sub_ptr);
        rig_term_log(ctrl, "gpredict",
                     "mapped Main->%s Sub->%s",
                     *main_ptr, *sub_ptr);
        *logged_ptr = TRUE;
    }

    return *target_ptr;
}

static gboolean rigctld_select_vfo_cached(GtkRigCtrl *ctrl, gint sock,
                                          vfo_t vfo, const gchar *token)
{
    RigSession *session = rig_session_for_socket_vfo(ctrl, sock, vfo);
    gchar vcmd[64];
    gchar buffback[128];
    gboolean retcode = FALSE;

    if (ctrl == NULL || token == NULL || *token == '\0')
        return FALSE;

    if (session != NULL &&
        session->strategy != RIG_STRATEGY_SELECT_VFO &&
        session->last_selected_vfo_valid &&
        session->last_selected_vfo == vfo)
        return TRUE;

    g_snprintf(vcmd, sizeof(vcmd), "V %s\x0a", token);
    retcode = send_rigctld_command(ctrl, sock, vcmd, buffback, 128);
    retcode = check_set_response(buffback, retcode, __func__);
    if (retcode && session != NULL)
    {
        session->last_selected_vfo = vfo;
        session->last_selected_vfo_valid = TRUE;
    }
    if (retcode && ctrl != NULL &&
        ctrl->conf2 == NULL &&
        is_full_duplex_main_sub_configured(ctrl->conf) &&
        ctrl->rig_session2 != NULL &&
        vfo != VFO_NONE)
    {
        RigSession *other = (session == ctrl->rig_session)
                                ? ctrl->rig_session2
                                : ctrl->rig_session;
        if (other != NULL)
            other->last_selected_vfo_valid = FALSE;
    }

    return retcode;
}

static gboolean rigctld_select_vfo_cached_locked(GtkRigCtrl *ctrl, gint sock,
                                                 vfo_t vfo, const gchar *token)
{
    RigSession *session = rig_session_for_socket_vfo(ctrl, sock, vfo);
    gchar vcmd[64];
    gchar buffback[128];
    gboolean retcode = FALSE;

    if (ctrl == NULL || token == NULL || *token == '\0')
        return FALSE;

    if (session != NULL &&
        session->strategy != RIG_STRATEGY_SELECT_VFO &&
        session->last_selected_vfo_valid &&
        session->last_selected_vfo == vfo)
        return TRUE;

    g_snprintf(vcmd, sizeof(vcmd), "V %s\x0a", token);
    retcode = _send_rigctld_command(ctrl, sock, vcmd, buffback, sizeof(buffback));
    retcode = check_set_response(buffback, retcode, __func__);
    if (retcode && session != NULL)
    {
        session->last_selected_vfo = vfo;
        session->last_selected_vfo_valid = TRUE;
    }
    if (retcode && ctrl != NULL &&
        ctrl->conf2 == NULL &&
        is_full_duplex_main_sub_configured(ctrl->conf) &&
        ctrl->rig_session2 != NULL &&
        vfo != VFO_NONE)
    {
        RigSession *other = (session == ctrl->rig_session)
                                ? ctrl->rig_session2
                                : ctrl->rig_session;
        if (other != NULL)
            other->last_selected_vfo_valid = FALSE;
    }

    return retcode;
}

static void rigctrl_log_config(GtkRigCtrl *ctrl,
                               const radio_conf_t *conf,
                               const gchar *role)
{
    const gchar *host;
    gint port;
    const gchar *device;
    const gchar *label = (role != NULL) ? role : _("rig");

    if (ctrl == NULL || conf == NULL)
        return;

    host = conf->host ? conf->host : "(null)";
    port = conf->port;
    device = (conf->rigctld_device && *conf->rigctld_device)
                 ? conf->rigctld_device
                 : "(none)";

    rig_term_log(ctrl, "gpredict",
                 "rig config (%s): model=%s mode=%s host=%s port=%d "
                 "autostart=%d conn=%s baud=%d device=%s "
                 "downlink_vfo=%s uplink_vfo=%s",
                 label,
                 radio_model_to_string(conf->radio_model),
                 radio_mode_to_string(conf->radio_mode),
                 host, port,
                 conf->rigctld_autostart ? 1 : 0,
                 rigctld_conn_name(conf->rigctld_conn),
                 conf->rigctld_baud,
                 device,
                 vfo_name(conf->downlink_vfo),
                 vfo_name(conf->uplink_vfo));
    sat_log_log(SAT_LOG_LEVEL_INFO,
                _("%s: rig config (%s) model=%s mode=%s host=%s port=%d "
                  "autostart=%d conn=%s baud=%d device=%s"),
                __func__, label,
                radio_model_to_string(conf->radio_model),
                radio_mode_to_string(conf->radio_mode),
                host, port,
                conf->rigctld_autostart ? 1 : 0,
                rigctld_conn_name(conf->rigctld_conn),
                conf->rigctld_baud,
                device);
}

static gboolean rigctrl_validate_mode(GtkRigCtrl *ctrl,
                                      const radio_conf_t *conf,
                                      const gchar *role)
{
    gchar *allowed = NULL;
    gchar *body = NULL;
    const gchar *label = role ? role : _("rig");

    if (ctrl == NULL || conf == NULL)
        return TRUE;

    if (radio_mode_allowed_for_model(conf->radio_model, conf->radio_mode))
        return TRUE;

    allowed = radio_mode_allowed_string(conf->radio_model);
    body = g_strdup_printf(_("Rig: %s\nModel: %s\nMode: %s\nAllowed: %s"),
                           conf->name ? conf->name : label,
                           radio_model_to_string(conf->radio_model),
                           radio_mode_to_string(conf->radio_mode),
                           allowed ? allowed : _("none"));
    rig_term_log(ctrl, "gpredict:err", "%s", body);
    rig_show_error_dialog(ctrl,
                          _("Unsupported radio mode"),
                          _("See log for details."));
    sat_log_log(SAT_LOG_LEVEL_ERROR,
                "%s: unsupported radio mode rig=%s model=%s mode=%s allowed=%s",
                __func__,
                conf->name ? conf->name : label,
                radio_model_to_string(conf->radio_model),
                radio_mode_to_string(conf->radio_mode),
                allowed ? allowed : "none");
    g_free(body);
    g_free(allowed);

    return FALSE;
}

static const gchar *vfo_role_name(vfo_role_t role)
{
    switch (role)
    {
    case VFO_ROLE_DOWNLINK:
        return "downlink";
    case VFO_ROLE_UPLINK:
        return "uplink";
    default:
        return "unknown";
    }
}

static const gchar *rigctrl_role_label(gboolean downlink)
{
    return downlink ? "DOWNLINK" : "UPLINK";
}

static const gchar *rig_base_source_name(rig_base_source_t src)
{
    switch (src)
    {
    case RIG_BASE_SRC_PRESET:
        return "PRESET";
    case RIG_BASE_SRC_MANUAL:
        return "MANUAL";
    case RIG_BASE_SRC_NONE:
    default:
        return "NONE";
    }
}

static gboolean is_full_duplex_main_sub_configured(const radio_conf_t *conf)
{
    return (conf != NULL) &&
        (conf->radio_mode == RADIO_MODE_FULL_DUPLEX_MAIN_SUB);
}

static gboolean is_full_duplex_main_sub_active(const GtkRigCtrl *ctrl)
{
    return (ctrl != NULL) &&
        ctrl->tracking &&
        is_full_duplex_main_sub_configured(ctrl->conf);
}

static const radio_conf_t *rigctrl_conf_for_role(const GtkRigCtrl *ctrl,
                                                 gboolean downlink)
{
    if (ctrl == NULL)
        return NULL;

    if (downlink)
        return ctrl->conf;

    if (ctrl->conf2 != NULL)
        return ctrl->conf2;

    return ctrl->conf;
}

static RigSession *rigctrl_session_for_role(GtkRigCtrl *ctrl, gboolean downlink)
{
    if (ctrl == NULL)
        return NULL;

    if (downlink)
        return ctrl->rig_session;

    if (ctrl->conf2 != NULL)
        return ctrl->rig_session2;

    if (is_full_duplex_main_sub_configured(ctrl->conf))
        return ctrl->rig_session2;

    return ctrl->rig_session;
}

static gboolean rigctrl_prepare_shared_tx_session(GtkRigCtrl *ctrl)
{
    RigSession *rx = NULL;
    RigSession *tx = NULL;
    RigctldClient *client = NULL;
    const RigCaps *caps = NULL;

    if (ctrl == NULL || ctrl->conf == NULL)
        return FALSE;

    if (ctrl->conf2 != NULL)
        return FALSE;

    if (!is_full_duplex_main_sub_configured(ctrl->conf))
        return FALSE;

    rx = ctrl->rig_session;
    tx = ctrl->rig_session2;
    if (rx == NULL || tx == NULL)
        return FALSE;

    if (ctrl->sock < 0)
        return FALSE;

    rig_session_reset(tx);

    client = rigctld_client_for_socket(ctrl, ctrl->sock);
    caps = client ? rigctld_client_get_caps(client) : NULL;
    if (caps != NULL)
        rig_session_apply_caps(tx, caps);
    else
        rig_session_copy_caps(tx, rx);

    rig_session_set_state(ctrl, tx, RIG_SESSION_READY, "shared rig");
    return TRUE;
}

static vfo_t rigctrl_target_vfo_for_role(const radio_conf_t *conf,
                                         vfo_role_t role)
{
    if (conf == NULL)
        return VFO_NONE;

    return (role == VFO_ROLE_DOWNLINK) ? conf->downlink_vfo : conf->uplink_vfo;
}

static vfo_t rigctrl_vfo_for_side(GtkRigCtrl *ctrl, gboolean downlink)
{
    const radio_conf_t *conf = rigctrl_conf_for_role(ctrl, downlink);

    return rigctrl_target_vfo_for_role(conf,
                                       downlink ? VFO_ROLE_DOWNLINK
                                                : VFO_ROLE_UPLINK);
}

static const gchar *rigctrl_vfo_label(GtkRigCtrl *ctrl,
                                      gboolean downlink,
                                      vfo_t vfo)
{
    const radio_conf_t *conf = rigctrl_conf_for_role(ctrl, downlink);

    if (vfo != VFO_NONE)
        return vfo_name(vfo);

    if (conf != NULL && conf->vfo_opt)
        return "currVFO";

    return "default";
}

static gboolean rigctrl_set_freq_for_role(GtkRigCtrl *ctrl,
                                          gint sock,
                                          gboolean downlink,
                                          gboolean toggle_mode,
                                          gint64 freq_hz,
                                          vfo_t *vfo_out)
{
    vfo_t vfo = rigctrl_vfo_for_side(ctrl, downlink);
    const gchar *cmd = toggle_mode ? "I" : "F";

    if (vfo_out)
        *vfo_out = vfo;

    rigctrl_log_role_op(ctrl, downlink, vfo, "set", cmd, freq_hz, 0, FALSE);

    if (vfo != VFO_NONE)
    {
        return toggle_mode ?
            set_freq_toggle_vfo(ctrl, sock, freq_hz, vfo) :
            set_freq_simplex_vfo(ctrl, sock, freq_hz, vfo);
    }

    return toggle_mode ?
        set_freq_toggle(ctrl, sock, freq_hz) :
        set_freq_simplex(ctrl, sock, freq_hz);
}

static gboolean rigctrl_get_freq_for_role(GtkRigCtrl *ctrl,
                                          gint sock,
                                          gboolean downlink,
                                          gboolean toggle_mode,
                                          gboolean strict,
                                          gint64 *freq_out,
                                          vfo_t *vfo_out)
{
    vfo_t vfo = rigctrl_vfo_for_side(ctrl, downlink);
    const gchar *cmd = toggle_mode ? "i" : "f";
    const gchar *tag = toggle_mode ? "i" : "f";
    gint cache_key = (vfo != VFO_NONE) ? (gint) vfo : (toggle_mode ? -2 : -1);
    gboolean ok = FALSE;

    if (vfo_out)
        *vfo_out = vfo;

    if (vfo != VFO_NONE)
    {
        if (toggle_mode)
        {
            ok = strict ?
                get_freq_toggle_vfo_strict(ctrl, sock, freq_out, vfo) :
                get_freq_toggle_vfo(ctrl, sock, freq_out, vfo);
        }
        else
        {
            ok = strict ?
                get_freq_simplex_vfo_strict(ctrl, sock, freq_out, vfo) :
                get_freq_simplex_vfo(ctrl, sock, freq_out, vfo);
        }
    }
    else if (toggle_mode)
    {
        ok = strict ?
            get_freq_toggle_strict(ctrl, sock, freq_out) :
            get_freq_toggle(ctrl, sock, freq_out);
    }
    else
    {
        ok = strict ?
            get_freq_simplex_strict(ctrl, sock, freq_out) :
            get_freq_simplex(ctrl, sock, freq_out);
    }

    rigctrl_log_role_op(ctrl, downlink, vfo, "get", cmd,
                        0, freq_out ? *freq_out : 0, ok);

    if (ok && freq_out != NULL && *freq_out > 0)
    {
        gint64 exp_min = 0;
        gint64 exp_max = 0;
        gint64 other_min = 0;
        gint64 other_max = 0;
        gboolean wrong_vfo =
            rigctrl_read_freq_wrong_vfo(ctrl, downlink, *freq_out,
                                        &exp_min, &exp_max,
                                        &other_min, &other_max);

        if (wrong_vfo)
        {
            if (rigctrl_log_at_least(ctrl, RIG_LOG_VERBOSE))
            {
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            "rig op role=%s vfo=%s read=%" G_GINT64_FORMAT
                            " outside expected; retrying select+read",
                            rigctrl_role_label(downlink),
                            vfo_name(vfo),
                            *freq_out);
                if (exp_min > 0 && exp_max > 0)
                {
                    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                                "rig op role=%s expected_band=%" G_GINT64_FORMAT
                                "..%" G_GINT64_FORMAT " other_band=%" G_GINT64_FORMAT
                                "..%" G_GINT64_FORMAT,
                                rigctrl_role_label(downlink),
                                exp_min, exp_max, other_min, other_max);
                }
            }

            rigctrl_clear_cached_freq(sock, tag, cache_key);

            if (vfo != VFO_NONE &&
                rig_session_strategy_for_vfo(ctrl, sock, vfo) ==
                RIG_STRATEGY_SELECT_VFO)
            {
                ok = toggle_mode
                         ? get_freq_toggle_vfo_strict(ctrl, sock, freq_out, vfo)
                         : get_freq_simplex_vfo_strict(ctrl, sock, freq_out, vfo);
            }
            else
            {
                ok = FALSE;
            }

            if (ok && freq_out != NULL && *freq_out > 0 &&
                rigctrl_read_freq_wrong_vfo(ctrl, downlink, *freq_out,
                                            &exp_min, &exp_max,
                                            &other_min, &other_max))
            {
                ok = FALSE;
            }
        }
    }

    return ok;
}

static gboolean satmode_vfo_for_role(const radio_conf_t *conf,
                                     vfo_role_t role, vfo_t *vfo)
{
    if (!is_full_duplex_main_sub_configured(conf) || vfo == NULL)
        return FALSE;

    *vfo = rigctrl_target_vfo_for_role(conf, role);
    return *vfo != VFO_NONE;
}

static gboolean G_GNUC_UNUSED select_satmode_vfo(GtkRigCtrl *ctrl, gint sock,
                                   vfo_role_t role, const gchar *action,
                                   gint64 freq, gboolean log_freq)
{
    vfo_t           vfo;

    (void)sock;

    if (!satmode_vfo_for_role(ctrl->conf, role, &vfo))
        return TRUE;

    sat_log_log(SAT_LOG_LEVEL_DEBUG, "FULL-DUPLEX MAIN/SUB: %s -> %s",
                vfo_role_name(role), vfo_name(vfo));
    if (action != NULL)
    {
        if (log_freq)
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "FULL-DUPLEX MAIN/SUB: using %s for %s %" G_GINT64_FORMAT,
                        vfo_name(vfo), action, freq);
        else
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "FULL-DUPLEX MAIN/SUB: using %s for %s",
                        vfo_name(vfo), action);
    }

    return TRUE;
}

static void rigctrl_cache_freq_key(gint sock, const gchar *tag,
                                   gint vfo_key, gchar *buf, gsize size)
{
    const gchar *tag_str = tag ? tag : "";

    g_snprintf(buf, size, "%d:%s:%d", sock, tag_str, vfo_key);
}

static void rigctrl_cache_freq(gint sock, const gchar *tag,
                               gint vfo_key, gint64 freq)
{
    gchar *key = NULL;
    gint64 *value = NULL;

    if (sock < 0 || freq <= 0)
        return;

    if (rig_freq_cache == NULL)
    {
        rig_freq_cache = g_hash_table_new_full(g_str_hash, g_str_equal,
                                               g_free, g_free);
    }

    key = g_strdup_printf("%d:%s:%d", sock, tag ? tag : "", vfo_key);
    value = g_new(gint64, 1);
    *value = freq;
    g_hash_table_replace(rig_freq_cache, key, value);
}

static void rigctrl_clear_cached_freq(gint sock,
                                      const gchar *tag,
                                      gint vfo_key)
{
    gchar key[64];

    if (rig_freq_cache == NULL || sock < 0)
        return;

    rigctrl_cache_freq_key(sock, tag, vfo_key, key, sizeof(key));
    g_hash_table_remove(rig_freq_cache, key);
}

static gboolean rigctrl_get_cached_freq(gint sock, const gchar *tag,
                                        gint vfo_key, gint64 *freq_out)
{
    gchar key[64];
    gint64 *value = NULL;

    if (rig_freq_cache == NULL || sock < 0)
        return FALSE;

    rigctrl_cache_freq_key(sock, tag, vfo_key, key, sizeof(key));
    value = g_hash_table_lookup(rig_freq_cache, key);
    if (value == NULL)
        return FALSE;

    if (freq_out)
        *freq_out = *value;

    return TRUE;
}

static gboolean set_freq_simplex_vfo(GtkRigCtrl *ctrl, gint sock,
                                     gint64 freq, vfo_t vfo)
{
    gchar           buffback[128];
    gboolean        retcode;
    const gchar    *token = NULL;
    rig_strategy_t  strategy = rig_session_strategy_for_vfo(ctrl, sock, vfo);
    RigSession     *session = rig_session_for_socket_vfo(ctrl, sock, vfo);
    gchar           freq_str[32];
    gchar          *freq_cmd = NULL;
    gint            rprt_code = 0;
    gboolean        has_rprt = FALSE;

    if (session != NULL && session->state != RIG_SESSION_READY)
        return FALSE;

    if (!ctrl->conf->vfo_opt)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    "FULL-DUPLEX MAIN/SUB: vfo_opt disabled; sending explicit VFO");
    }

    token = rigctld_vfo_token(ctrl, sock, vfo);
    if (token == NULL)
        return FALSE;

    hz_to_rigctld_string(freq, freq_str, sizeof(freq_str));

    g_mutex_lock(&ctrl->writelock);

    if (strategy == RIG_STRATEGY_SELECT_VFO)
    {
        if (!rigctld_select_vfo_cached_locked(ctrl, sock, vfo, token))
        {
            g_mutex_unlock(&ctrl->writelock);
            return FALSE;
        }
        freq_cmd = g_strdup_printf("F %s\x0a", freq_str);
    }
    else if (strategy == RIG_STRATEGY_VFO_OPT_ARGS)
    {
        freq_cmd = g_strdup_printf("F %s %s\x0a", token, freq_str);
    }
    else
    {
        sat_log_log(SAT_LOG_LEVEL_WARN,
                    "FULL-DUPLEX MAIN/SUB: strategy %s; falling back to single VFO",
                    rig_strategy_name(strategy));
        freq_cmd = g_strdup_printf("F %s\x0a", freq_str);
    }

    if (freq_cmd != NULL)
    {
        gchar *cmd_log = g_strdup(freq_cmd);
        g_strchomp(cmd_log);
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "FULL-DUPLEX MAIN/SUB: send freq cmd='%s' request=%s token=%s",
                    cmd_log, vfo_name(vfo),
                    token ? token : "(null)");
        g_free(cmd_log);
    }

    retcode = _send_rigctld_command(ctrl, sock, freq_cmd, buffback, 128);
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "FULL-DUPLEX MAIN/SUB: set %s %" G_GINT64_FORMAT " -> %s",
                vfo_name(vfo), freq, buffback);
    has_rprt = rig_parse_rprt_code_any(buffback, &rprt_code);

    if (has_rprt &&
        rigctld_should_retry_main_sub(ctrl, session, vfo, rprt_code))
    {
        sat_log_log(SAT_LOG_LEVEL_WARN,
                    "FULL-DUPLEX MAIN/SUB: set %s rejected (RPRT %d); retrying",
                    vfo_name(vfo), rprt_code);
        if (session != NULL)
            session->last_selected_vfo_valid = FALSE;
        if (!rigctld_select_vfo_cached_locked(ctrl, sock, vfo, token))
        {
            g_free(freq_cmd);
            g_mutex_unlock(&ctrl->writelock);
            return FALSE;
        }

        if (freq_cmd != NULL)
        {
            gchar *cmd_log = g_strdup(freq_cmd);
            g_strchomp(cmd_log);
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "FULL-DUPLEX MAIN/SUB: retry freq cmd='%s' request=%s token=%s",
                        cmd_log, vfo_name(vfo),
                        token ? token : "(null)");
            g_free(cmd_log);
        }
        retcode = _send_rigctld_command(ctrl, sock, freq_cmd, buffback, 128);
        g_free(freq_cmd);
        retcode = check_set_response(buffback, retcode, __func__);
        g_mutex_unlock(&ctrl->writelock);
        return retcode;
    }

    g_free(freq_cmd);
    retcode = check_set_response(buffback, retcode, __func__);
    g_mutex_unlock(&ctrl->writelock);
    return retcode;
}

static gboolean get_freq_simplex_vfo_internal(GtkRigCtrl *ctrl, gint sock,
                                              gint64 *freq, vfo_t vfo,
                                              gboolean allow_cache)
{
    gchar          *buff;
    gchar           buffback[128];
    gboolean        retcode;
    gboolean        retval = TRUE;
    gchar         **vbuff;
    const gchar    *token = NULL;
    rig_strategy_t  strategy = rig_session_strategy_for_vfo(ctrl, sock, vfo);

    if (!ctrl->conf->vfo_opt)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    "FULL-DUPLEX MAIN/SUB: vfo_opt disabled; requesting explicit VFO");
    }

    token = rigctld_vfo_token(ctrl, sock, vfo);
    if (token == NULL)
        return FALSE;

    if (strategy == RIG_STRATEGY_SELECT_VFO)
    {
        if (!rigctld_select_vfo_cached(ctrl, sock, vfo, token))
            return FALSE;
        buff = g_strdup_printf("f\x0a");
    }
    else if (strategy == RIG_STRATEGY_VFO_OPT_ARGS)
    {
        buff = g_strdup_printf("f %s\x0a", token);
    }
    else
    {
        sat_log_log(SAT_LOG_LEVEL_WARN,
                    "FULL-DUPLEX MAIN/SUB: strategy %s; falling back to single VFO",
                    rig_strategy_name(strategy));
        buff = g_strdup_printf("f\x0a");
    }

    if (buff != NULL)
    {
        gchar *cmd_log = g_strdup(buff);
        g_strchomp(cmd_log);
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "FULL-DUPLEX MAIN/SUB: send freq cmd='%s' request=%s token=%s",
                    cmd_log, vfo_name(vfo),
                    token ? token : "(null)");
        g_free(cmd_log);
    }

    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    retcode = check_get_response(buffback, retcode, __func__);
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "FULL-DUPLEX MAIN/SUB: get %s -> %s",
                vfo_name(vfo), buffback);
    if (retcode)
    {
        vbuff = g_strsplit(buffback, "\n", 3);
        if (vbuff[0])
        {
            gchar *endptr = NULL;
            *freq = g_ascii_strtoll(vbuff[0], &endptr, 10);
            if (endptr == vbuff[0])
                retval = FALSE;
        }
        else
            retval = FALSE;
        g_strfreev(vbuff);
    }
    else
    {
        retval = FALSE;
    }

    g_free(buff);
    if (retval)
    {
        rigctrl_cache_freq(sock, "f", (gint) vfo, *freq);
        return TRUE;
    }

    if (allow_cache && rigctrl_get_cached_freq(sock, "f", (gint) vfo, freq))
    {
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "FULL-DUPLEX MAIN/SUB: get %s failed; using cached %" G_GINT64_FORMAT,
                    vfo_name(vfo), *freq);
        return TRUE;
    }

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "FULL-DUPLEX MAIN/SUB: get %s failed",
                vfo_name(vfo));
    return FALSE;
}

static gboolean get_freq_simplex_vfo(GtkRigCtrl *ctrl, gint sock,
                                     gint64 *freq, vfo_t vfo)
{
    return get_freq_simplex_vfo_internal(ctrl, sock, freq, vfo, TRUE);
}

static gboolean get_freq_simplex_vfo_strict(GtkRigCtrl *ctrl, gint sock,
                                            gint64 *freq, vfo_t vfo)
{
    return get_freq_simplex_vfo_internal(ctrl, sock, freq, vfo, FALSE);
}

static gboolean set_freq_toggle_vfo(GtkRigCtrl *ctrl, gint sock,
                                    gint64 freq, vfo_t vfo)
{
    gchar          *buff;
    gchar           buffback[128];
    gboolean        retcode;
    const gchar    *token = NULL;
    rig_strategy_t  strategy = rig_session_strategy_for_vfo(ctrl, sock, vfo);
    gchar           freq_str[32];

    if (!ctrl->conf->vfo_opt)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    "FULL-DUPLEX MAIN/SUB: vfo_opt disabled; sending explicit VFO");
    }

    token = rigctld_vfo_token(ctrl, sock, vfo);
    if (token == NULL)
        return FALSE;

    hz_to_rigctld_string(freq, freq_str, sizeof(freq_str));

    if (strategy == RIG_STRATEGY_SELECT_VFO)
    {
        if (!rigctld_select_vfo_cached(ctrl, sock, vfo, token))
            return FALSE;
        buff = g_strdup_printf("I %s\x0a", freq_str);
    }
    else if (strategy == RIG_STRATEGY_VFO_OPT_ARGS)
    {
        buff = g_strdup_printf("I %s %s\x0a", token, freq_str);
    }
    else
    {
        sat_log_log(SAT_LOG_LEVEL_WARN,
                    "FULL-DUPLEX MAIN/SUB: strategy %s; falling back to single VFO",
                    rig_strategy_name(strategy));
        buff = g_strdup_printf("I %s\x0a", freq_str);
    }

    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "FULL-DUPLEX MAIN/SUB: set %s (toggle) %" G_GINT64_FORMAT " -> %s",
                vfo_name(vfo), freq, buffback);
    g_free(buff);

    return check_set_response(buffback, retcode, __func__);
}

static gboolean get_freq_toggle_vfo_internal(GtkRigCtrl *ctrl, gint sock,
                                             gint64 *freq, vfo_t vfo,
                                             gboolean allow_cache)
{
    gchar          *buff;
    gchar           buffback[128];
    gboolean        retcode;
    gboolean        retval = TRUE;
    gchar         **vbuff;
    const gchar    *token = NULL;
    rig_strategy_t  strategy = rig_session_strategy_for_vfo(ctrl, sock, vfo);

    if (freq == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%d: NULL storage."), __FILE__, __LINE__);
        return FALSE;
    }

    if (!ctrl->conf->vfo_opt)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    "FULL-DUPLEX MAIN/SUB: vfo_opt disabled; requesting explicit VFO");
    }

    token = rigctld_vfo_token(ctrl, sock, vfo);
    if (token == NULL)
        return FALSE;

    if (strategy == RIG_STRATEGY_SELECT_VFO)
    {
        if (!rigctld_select_vfo_cached(ctrl, sock, vfo, token))
            return FALSE;
        buff = g_strdup_printf("i\x0a");
    }
    else if (strategy == RIG_STRATEGY_VFO_OPT_ARGS)
    {
        buff = g_strdup_printf("i %s\x0a", token);
    }
    else
    {
        sat_log_log(SAT_LOG_LEVEL_WARN,
                    "FULL-DUPLEX MAIN/SUB: strategy %s; falling back to single VFO",
                    rig_strategy_name(strategy));
        buff = g_strdup_printf("i\x0a");
    }

    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    retcode = check_get_response(buffback, retcode, __func__);
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "FULL-DUPLEX MAIN/SUB: get %s (toggle) -> %s",
                vfo_name(vfo), buffback);
    if (retcode)
    {
        vbuff = g_strsplit(buffback, "\n", 3);
        if (vbuff[0])
        {
            gchar *endptr = NULL;
            *freq = g_ascii_strtoll(vbuff[0], &endptr, 10);
            if (endptr == vbuff[0])
                retval = FALSE;
        }
        else
            retval = FALSE;

        g_strfreev(vbuff);
    }
    else
    {
        retval = FALSE;
    }

    g_free(buff);
    if (retval)
    {
        rigctrl_cache_freq(sock, "i", (gint) vfo, *freq);
        return TRUE;
    }

    if (allow_cache && rigctrl_get_cached_freq(sock, "i", (gint) vfo, freq))
    {
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "FULL-DUPLEX MAIN/SUB: get %s (toggle) failed; using cached %" G_GINT64_FORMAT,
                    vfo_name(vfo), *freq);
        return TRUE;
    }

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "FULL-DUPLEX MAIN/SUB: get %s (toggle) failed",
                vfo_name(vfo));
    return FALSE;
}

static gboolean get_freq_toggle_vfo(GtkRigCtrl *ctrl, gint sock,
                                    gint64 *freq, vfo_t vfo)
{
    return get_freq_toggle_vfo_internal(ctrl, sock, freq, vfo, TRUE);
}

static gboolean get_freq_toggle_vfo_strict(GtkRigCtrl *ctrl, gint sock,
                                           gint64 *freq, vfo_t vfo)
{
    return get_freq_toggle_vfo_internal(ctrl, sock, freq, vfo, FALSE);
}

static int get_vfos(GtkRigCtrl * ctrl, char *rx, char *tx)
{
    // fill rx/tx with vfo name plus space if not empty
    rx = tx = "";
    switch (ctrl->conf->uplink_vfo)
    {
    case VFO_A:
        if (ctrl->conf->vfo_opt)
            {rx = "VFOB ";tx = "VFOA ";}
        break;

    case VFO_B:
        if (ctrl->conf->vfo_opt)
           {rx = "VFOA ";tx = "VFOB ";}
        break;

    case VFO_MAIN:
        if (ctrl->conf->vfo_opt)
            {rx = "Sub";tx = "Main";}
        break;

    case VFO_SUB:
        if (ctrl->conf->vfo_opt)
            {rx = "Main";tx = "Sub";}
        break;

    default:
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s called but TX VFO is %d and we don't know how to handle it."), __func__,
                    ctrl->conf->uplink_vfo);
        return 1;
    }
    sat_log_log(SAT_LOG_LEVEL_DEBUG, "rx=%x, tx=%s\n", rx, tx);
    return 0;
}

/* Setup VFOs for split operation (simplex or duplex) */
static gboolean setup_split(GtkRigCtrl * ctrl)
{
    gchar          *buff;
    gchar           buffback[256];
    gboolean        retcode;
    gchar          *rx="", *tx="";
    vfo_t           vfo_up;

    get_vfos(ctrl, rx, tx);
    vfo_up = ctrl->conf->uplink_vfo;
    satmode_vfo_for_role(ctrl->conf, VFO_ROLE_UPLINK, &vfo_up);
    switch (vfo_up)
    {
    case VFO_A:
        if (ctrl->conf->vfo_opt)
            buff = g_strdup("S VFOB 1 VFOA\x0a");
        else
            buff = g_strdup("S 1 VFOA\x0a");
        break;

    case VFO_B:
        if (ctrl->conf->vfo_opt)
            buff = g_strdup("S VFOA 1 VFOB\x0a");
        else
            buff = g_strdup("S 1 VFOB\x0a");
        break;

    case VFO_MAIN:
        if (ctrl->conf->vfo_opt)
            buff = g_strdup("S Sub 1 Main\x0a");
        else
            buff = g_strdup("S 1 Main\x0a");
        break;

    case VFO_SUB:
        if (ctrl->conf->vfo_opt)
            buff = g_strdup("S Main 1 Sub\x0a");
        else
            buff = g_strdup("S 1 Sub\x0a");
        break;

    default:
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s called but TX VFO is %d."), __func__,
                    ctrl->conf->uplink_vfo);
        return FALSE;
    }

    retcode = send_rigctld_command(ctrl, ctrl->sock, buff, buffback, 128);
    g_free(buff);

    return (check_set_response(buffback, retcode, __func__));
}

static gboolean rig_ctrl_timeout_cb(gpointer data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);

    if (ctrl == NULL || ctrl->destroying || !ctrl->engaged ||
        ctrl->conn_state != RIGCTRL_CONN_CONNECTED)
    {
        if (ctrl != NULL)
            ctrl->timerid = 0;
        return G_SOURCE_REMOVE;
    }

    if (ctrl->conf == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: Controller does not have a valid configuration"),
                    __func__);
        return FALSE;
    }

    rigctrl_doppler_tick(ctrl);

    if (g_mutex_trylock(&(ctrl->busy)) == FALSE)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR, _("%s missed the deadline"),
                    __func__);
        return TRUE;
    }

    setconfig(ctrl);
    g_mutex_unlock(&(ctrl->busy));

    return TRUE;
}

static void exec_rx_cycle(GtkRigCtrl * ctrl)
{
    gint64          readfreq = 0;
    gint64          tmpfreq = 0;
    gint64          base_sat = 0;
    gboolean        ptt = FALSE;
    gboolean        use_rit_xit =
        (ctrl->conf != NULL) ? ctrl->conf->supports_rit_xit : FALSE;
    vfo_t           vfo = VFO_NONE;
    gint64          doppler_hz = 0;
    gint64          sent_freq = 0;
    gint64          target_up = 0;
    gboolean        target_ok = TRUE;

    /* get PTT status */
    if (ctrl->engaged && ctrl->conf->ptt)
        ptt = get_ptt(ctrl, ctrl->sock);

    if (is_full_duplex_main_sub_active(ctrl))
        use_rit_xit = FALSE;

    /* Dial feedback:
       If radio device is engaged read frequency from radio and compare it to the
       last set frequency. If different, it means that user has changed frequency
       on the radio dial => update transponder knob

       Note: If ctrl->lastrxf = 0.0 the sync has been invalidated (e.g. user pressed "tune")
       and no need to execute the dial feedback.
     */
    if ((ctrl->engaged) && (ctrl->lastrxf > 0) && (ptt == FALSE))
    {
        if (!rigctrl_get_freq_for_role(ctrl, ctrl->sock, TRUE,
                                       FALSE, FALSE, &readfreq, &vfo))
        {
            /* error => use a passive value */
            readfreq = ctrl->lastrxf;
        }
        else if (llabs(readfreq - ctrl->lastrxf) >= 1)
        {
            /* user might have altered radio frequency => update rig readback */
            rigctrl_set_freq_knob_value(ctrl, FALSE,
                                    (gdouble)readfreq);
            ctrl->lastrxf = readfreq;
            ctrl->rig_actual_down_hz = readfreq;

            /* no need to forward track */
            return;
        }
    }

    /* now, forward tracking */

    tmpfreq = rigctrl_compute_target(ctrl, TRUE, ctrl->conf->lo,
                                     use_rit_xit, &base_sat,
                                     &doppler_hz, &target_ok);
    target_up = rigctrl_compute_target(ctrl, FALSE, ctrl->conf->loup,
                                       use_rit_xit, NULL, NULL, NULL);
    rigctrl_update_last_target(ctrl, TRUE, tmpfreq);
    rigctrl_update_last_target(ctrl, FALSE, target_up);

    rigctrl_set_freq_knob_value(ctrl, FALSE, (gdouble)tmpfreq);
    rigctrl_set_freq_knob_value(ctrl, TRUE, (gdouble)target_up);

    sent_freq = tmpfreq;

    /* if device is engaged, send freq command to radio */
    if ((ctrl->engaged) && (ptt == FALSE) && target_ok &&
        rigctrl_should_send_freq(ctrl, TRUE, tmpfreq))
    {
        {
            gboolean set_ok;
            gboolean read_ok;
            gint64   readback = 0;
            vfo_t   set_vfo = rigctrl_vfo_for_side(ctrl, TRUE);
            const gchar *vfo_label = rigctrl_vfo_label(ctrl, TRUE, set_vfo);

            rigctrl_log_send(ctrl, TRUE, vfo_label,
                             base_sat, doppler_hz, sent_freq);
            ctrl->last_send_down_us = g_get_monotonic_time();

            set_ok = rigctrl_set_freq_for_role(ctrl, ctrl->sock, TRUE,
                                               FALSE, tmpfreq, NULL);
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rig update: side=RX set=%s", set_ok ? "ok" : "fail");

            if (set_ok)
            {
                /* give radio a chance to set frequency */
                g_usleep(WR_DEL);

                /* The actual frequency might be different from what we have set because
                   the tuning step is larger than what we work with (e.g. FT-817 has a
                   smallest tuning step of 10 Hz). Therefore we read back the actual
                   frequency from the rig. */
                read_ok = (set_vfo != VFO_NONE) ?
                    get_freq_simplex_vfo_strict(ctrl, ctrl->sock, &readback, set_vfo) :
                    get_freq_simplex_strict(ctrl, ctrl->sock, &readback);
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            "rig update: side=RX readback=%" G_GINT64_FORMAT " ok=%d",
                            readback, read_ok ? 1 : 0);

                if (!read_ok)
                {
                    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                                "rig update: side=RX readback failed; keeping last");
                    rigctrl_update_last_sent(ctrl, TRUE, sent_freq);
                }
                else if (rigctrl_verify_match(ctrl, TRUE, set_vfo,
                                              sent_freq, readback, 0))
                {
                    ctrl->errcnt = 0;
                    ctrl->lastrxf = readback;
                    ctrl->rig_actual_down_hz = readback;
                    rigctrl_update_last_sent(ctrl, TRUE, readback);
                    rigctrl_set_freq_knob_value(ctrl, FALSE,
                                            (gdouble)readback);

                    /* This is only effective in RIG_TYPE_TRX mode.
                       Invalidate ctrl->lasttxf for two reasons.

                       1. Prevent dial feedback from changing the uplink frequency.
                       In the first TX cycle get_freq_simplex() returns the downlink
                       frequency instead of uplink. The mismatch would thus trigger
                       an uplink update as long as the VFO has not been updated.
                       2. Force updating the VFO in the first TX cycle.
                     */
                    if (ctrl->lastrxptt != ptt)
                        ctrl->lasttxf = 0;
                }
                else
                {
                    ctrl->errcnt++;
                    ctrl->lastrxf = 0;
                    rigctrl_update_last_sent(ctrl, TRUE, 0);
                }
            }
            else
            {
                ctrl->errcnt++;
            }
        }
    }

    /* Remember PTT state, to avoid misinterpreting VFO changes as dial
       feedback during TX to RX transitions.
     */
    ctrl->lastrxptt = ptt;
}

static void exec_tx_cycle(GtkRigCtrl * ctrl)
{
    gint64          readfreq = 0;
    gint64          tmpfreq = 0;
    gint64          base_sat = 0;
    gboolean        ptt = TRUE;
    gboolean        use_rit_xit =
        (ctrl->conf != NULL) ? ctrl->conf->supports_rit_xit : FALSE;
    gint64          doppler_hz = 0;
    gint64          sent_freq = 0;
    gint64          target_down = 0;
    gboolean        target_ok = TRUE;
    vfo_t           vfo = VFO_NONE;

    /* get PTT status */
    if (ctrl->engaged && ctrl->conf->ptt)
    {
        ptt = get_ptt(ctrl, ctrl->sock);
    }

    if (is_full_duplex_main_sub_active(ctrl))
        use_rit_xit = FALSE;

    /* Dial feedback:
       If radio device is engaged read frequency from radio and compare it to the
       last set frequency. If different, it means that user has changed frequency
       on the radio dial => update transponder knob

       Note: If ctrl->lasttxf = 0.0 the sync has been invalidated (e.g. user pressed "tune")
       and no need to execute the dial feedback.
     */
    if ((ctrl->engaged) && (ctrl->lasttxf > 0) && (ptt == TRUE))
    {
        if (!rigctrl_get_freq_for_role(ctrl, ctrl->sock, FALSE,
                                       FALSE, FALSE, &readfreq, &vfo))
        {
            /* error => use a passive value */
            readfreq = ctrl->lasttxf;
        }
        else if (llabs(readfreq - ctrl->lasttxf) >= 1)
        {
            /* user might have altered radio frequency => update rig readback */
            rigctrl_set_freq_knob_value(ctrl, TRUE, (gdouble)readfreq);
            ctrl->lasttxf = readfreq;
            ctrl->rig_actual_up_hz = readfreq;

            /* no need to forward track */
            return;
        }
    }

    /* now, forward tracking */

    tmpfreq = rigctrl_compute_target(ctrl, FALSE, ctrl->conf->loup,
                                     use_rit_xit, &base_sat,
                                     &doppler_hz, &target_ok);
    target_down = rigctrl_compute_target(ctrl, TRUE, ctrl->conf->lo,
                                         use_rit_xit, NULL, NULL, NULL);
    rigctrl_update_last_target(ctrl, FALSE, tmpfreq);
    rigctrl_update_last_target(ctrl, TRUE, target_down);

    rigctrl_set_freq_knob_value(ctrl, TRUE, (gdouble)tmpfreq);
    rigctrl_set_freq_knob_value(ctrl, FALSE, (gdouble)target_down);

    sent_freq = tmpfreq;

    /* if device is engaged, send freq command to radio */
    if ((ctrl->engaged) && (ptt == TRUE) && target_ok &&
        rigctrl_should_send_freq(ctrl, FALSE, tmpfreq))
    {
        {
            gboolean set_ok;
            gboolean read_ok;
            gint64   readback = 0;
            vfo_t   set_vfo = rigctrl_vfo_for_side(ctrl, FALSE);
            const gchar *vfo_label = rigctrl_vfo_label(ctrl, FALSE, set_vfo);

            rigctrl_log_send(ctrl, FALSE, vfo_label,
                             base_sat, doppler_hz, sent_freq);
            ctrl->last_send_up_us = g_get_monotonic_time();

            set_ok = rigctrl_set_freq_for_role(ctrl, ctrl->sock, FALSE,
                                               FALSE, tmpfreq, NULL);
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rig update: side=TX set=%s", set_ok ? "ok" : "fail");

            if (set_ok)
            {
                /* give radio a chance to set frequency */
                g_usleep(WR_DEL);

                /* The actual frequency migh be different from what we have set because
                   the tuning step is larger than what we work with (e.g. FT-817 has a
                   smallest tuning step of 10 Hz). Therefore we read back the actual
                   frequency from the rig. */
                read_ok = (set_vfo != VFO_NONE) ?
                    get_freq_simplex_vfo_strict(ctrl, ctrl->sock, &readback, set_vfo) :
                    get_freq_simplex_strict(ctrl, ctrl->sock, &readback);
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            "rig update: side=TX readback=%" G_GINT64_FORMAT " ok=%d",
                            readback, read_ok ? 1 : 0);

                if (!read_ok)
                {
                    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                                "rig update: side=TX readback failed; keeping last");
                    rigctrl_update_last_sent(ctrl, FALSE, sent_freq);
                }
                else if (rigctrl_verify_match(ctrl, FALSE, set_vfo,
                                              sent_freq, readback, 0))
                {
                    ctrl->errcnt = 0;
                    ctrl->lasttxf = readback;
                    ctrl->rig_actual_up_hz = readback;
                    rigctrl_update_last_sent(ctrl, FALSE, readback);
                    rigctrl_set_freq_knob_value(ctrl, TRUE,
                                            (gdouble)readback);

                    /* This is only effective in RIG_TYPE_TRX mode.
                       Invalidate ctrl->lastrxf for two reasons.

                       1. Prevent dial feedback from changing the downlink frequency.
                       In the first RX cycle get_freq_simplex() returns the uplink
                       frequency instead of downlink. The mismatch would thus
                       trigger a downlink update as long as the VFO has not been
                       updated.
                       2. Force updating the VFO in the first RX cycle.
                     */
                    if (ctrl->lasttxptt != ptt)
                        ctrl->lastrxf = 0;
                }
                else
                {
                    ctrl->errcnt++;
                    ctrl->lasttxf = 0;
                    rigctrl_update_last_sent(ctrl, FALSE, 0);
                }
            }
            else
            {
                ctrl->errcnt++;
            }
        }
    }

    /* Remember PTT state, to avoid misinterpreting VFO changes as dial
       feedback during RX to TX transitions.
     */
    ctrl->lasttxptt = ptt;
}

static void exec_trx_cycle(GtkRigCtrl * ctrl)
{
    exec_rx_cycle(ctrl);
    exec_tx_cycle(ctrl);
}

static void exec_toggle_cycle(GtkRigCtrl * ctrl)
{
    exec_rx_cycle(ctrl);

    /* TX cycle is executed only if user selected RIG_TYPE_TOGGLE_AUTO
     * In manual mode the TX freq update is performed only when TX is activated.
     * Even in auto mode, the toggling is performed only once every 10 seconds.
     */
    if (ctrl->conf->type == RIG_TYPE_TOGGLE_AUTO)
    {
	gint64          current_time;

        /* get the current time */
	current_time = g_get_real_time() / G_USEC_PER_SEC;

        if ((ctrl->last_toggle_tx == -1) ||
            ((current_time - ctrl->last_toggle_tx) >= 10))
        {
            /* it's time to update TX freq */
            exec_toggle_tx_cycle(ctrl);

            /* store current time */
            ctrl->last_toggle_tx = current_time;
        }
    }
}

/*
 * Execute TX mode cycle.
 *
 * This function executes a transmit cycle when the primary device is of
 * RIG_TYPE_TOGGLE_AUTO. This applies to radios that support split operation
 * (e.g. TX on VHF, RX on UHF) where the frequency can not be set via CAT while
 * PTT is active.
 *
 * If PTT=TRUE we are in TX mode and hence there is nothing to do since the
 * frequency is kept constant.
 *
 * If PTT=FALSE we are in RX mode and we should update the TX frequency by
 * using set_freq_toggle()
 *
 * For these kind of radios there is no dial-feedback for the TX frequency.
 */

static void exec_toggle_tx_cycle(GtkRigCtrl * ctrl)
{
    gint64          tmpfreq = 0;
    gint64          base_sat = 0;
    gint64          doppler_hz = 0;
    gboolean        ptt = TRUE;
    gboolean        use_rit_xit =
        (ctrl->conf != NULL) ? ctrl->conf->supports_rit_xit : FALSE;
    gboolean        target_ok = TRUE;

    if (ctrl->engaged && ctrl->conf->ptt)
    {
        ptt = get_ptt(ctrl, ctrl->sock);
    }

    /* if we are in TX mode do nothing */
    if (ptt == TRUE)
    {
        return;
    }

    if (is_full_duplex_main_sub_active(ctrl))
        use_rit_xit = FALSE;

    tmpfreq = rigctrl_compute_target(ctrl, FALSE, ctrl->conf->loup,
                                     use_rit_xit, &base_sat,
                                     &doppler_hz, &target_ok);
    rigctrl_update_last_target(ctrl, FALSE, tmpfreq);
    rigctrl_set_freq_knob_value(ctrl, TRUE, (gdouble)tmpfreq);

    /* if device is engaged, send freq command to radio */
    if ((ctrl->engaged) && target_ok &&
        rigctrl_should_send_freq(ctrl, FALSE, tmpfreq))
    {
        vfo_t set_vfo = rigctrl_vfo_for_side(ctrl, FALSE);
        const gchar *vfo_label = rigctrl_vfo_label(ctrl, FALSE, set_vfo);

        rigctrl_log_send(ctrl, FALSE, vfo_label,
                         base_sat, doppler_hz, tmpfreq);
        ctrl->last_send_up_us = g_get_monotonic_time();
        if (rigctrl_set_freq_for_role(ctrl, ctrl->sock, FALSE,
                                      TRUE, tmpfreq, NULL))
        {
            /* reset error counter */
            ctrl->errcnt = 0;
            rigctrl_update_last_sent(ctrl, FALSE, tmpfreq);
        }
        else
        {
            ctrl->errcnt++;
        }

        /* store the last sent frequency even if an error occurred */
        ctrl->lasttxf = tmpfreq;
    }

}

static void exec_full_duplex_main_sub_cycle(GtkRigCtrl * ctrl,
                                            gboolean force_send)
{
    rig_mode_dispatch_t plan;
    gint64          base_sat_down = 0;
    gint64          base_sat_up = 0;
    gint64          rigfreqd = 0;
    gint64          rigfrequ = 0;
    gint64          doppler_down = 0;
    gint64          doppler_up = 0;
    gint64          readback = 0;
    gboolean        set_ok;
    gboolean        read_ok;
    gboolean        down_ok = TRUE;
    gboolean        up_ok = TRUE;

    if (ctrl == NULL || ctrl->conf == NULL)
        return;

    rig_mode_dispatch(ctrl->conf->radio_mode,
                      rigctrl_target_vfo_for_role(ctrl->conf, VFO_ROLE_DOWNLINK),
                      rigctrl_target_vfo_for_role(ctrl->conf, VFO_ROLE_UPLINK),
                      &plan);
    if (!plan.send_downlink && !plan.send_uplink)
        return;

    rigfreqd = rigctrl_compute_target(ctrl, TRUE, ctrl->conf->lo, FALSE,
                                      &base_sat_down, &doppler_down,
                                      &down_ok);
    rigfrequ = rigctrl_compute_target(ctrl, FALSE, ctrl->conf->loup, FALSE,
                                      &base_sat_up, &doppler_up,
                                      &up_ok);
    rigctrl_update_last_target(ctrl, TRUE, rigfreqd);
    rigctrl_update_last_target(ctrl, FALSE, rigfrequ);

    rigctrl_set_freq_knob_value(ctrl, FALSE, (gdouble)rigfreqd);
    rigctrl_set_freq_knob_value(ctrl, TRUE, (gdouble)rigfrequ);

    if (!ctrl->engaged)
        return;

    if (plan.send_downlink)
    {
        if (plan.downlink_vfo == VFO_NONE)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        "FULL-DUPLEX MAIN/SUB: invalid downlink VFO");
            ctrl->errcnt++;
        }
        else if (!down_ok)
        {
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rig update: mode=FULL_DUPLEX_MAIN_SUB side=RX skipped (out of range)");
        }
        else if (!rigctrl_freq_valid_for_send(ctrl, TRUE, rigfreqd))
        {
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rig update: mode=FULL_DUPLEX_MAIN_SUB side=RX skipped (invalid)");
        }
        else if (!force_send && !rigctrl_should_send_freq(ctrl, TRUE, rigfreqd))
        {
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rig update: mode=FULL_DUPLEX_MAIN_SUB side=RX skipped (deadband/rate)");
        }
        else
        {
            vfo_t set_vfo = rigctrl_vfo_for_side(ctrl, TRUE);
            rigctrl_log_send(ctrl, TRUE, vfo_name(plan.downlink_vfo),
                             base_sat_down, doppler_down, rigfreqd);
            ctrl->last_send_down_us = g_get_monotonic_time();
            set_ok = rigctrl_set_freq_for_role(ctrl, ctrl->sock, TRUE,
                                               FALSE, rigfreqd, NULL);
            if (set_ok)
            {
                g_usleep(WR_DEL);
                read_ok = rigctrl_get_freq_for_role(ctrl, ctrl->sock, TRUE,
                                                    FALSE, TRUE,
                                                    &readback, NULL);
                if (!read_ok)
                {
                    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                                "rig update: mode=FULL_DUPLEX_MAIN_SUB side=RX readback failed; keeping last");
                    rigctrl_update_last_sent(ctrl, TRUE, rigfreqd);
                }
                else if (rigctrl_verify_match(ctrl, TRUE, set_vfo,
                                              rigfreqd, readback, 0))
                {
                    ctrl->errcnt = 0;
                    ctrl->lastrxf = readback;
                    ctrl->rig_actual_down_hz = readback;
                    rigctrl_update_last_sent(ctrl, TRUE, readback);
                    rigctrl_set_freq_knob_value(ctrl, FALSE,
                                            (gdouble)readback);
                }
                else
                {
                    ctrl->errcnt++;
                    ctrl->lastrxf = 0;
                    rigctrl_update_last_sent(ctrl, TRUE, 0);
                }
            }
            else
            {
                ctrl->errcnt++;
            }
        }
    }

    if (plan.send_uplink)
    {
        if (plan.uplink_vfo == VFO_NONE)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        "FULL-DUPLEX MAIN/SUB: invalid uplink VFO");
            ctrl->errcnt++;
        }
        else if (!up_ok)
        {
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rig update: mode=FULL_DUPLEX_MAIN_SUB side=TX skipped (out of range)");
        }
        else if (!rigctrl_freq_valid_for_send(ctrl, FALSE, rigfrequ))
        {
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rig update: mode=FULL_DUPLEX_MAIN_SUB side=TX skipped (invalid)");
        }
        else if (!force_send && !rigctrl_should_send_freq(ctrl, FALSE, rigfrequ))
        {
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rig update: mode=FULL_DUPLEX_MAIN_SUB side=TX skipped (deadband/rate)");
        }
        else
        {
            vfo_t set_vfo = rigctrl_vfo_for_side(ctrl, FALSE);
            rigctrl_log_send(ctrl, FALSE, vfo_name(plan.uplink_vfo),
                             base_sat_up, doppler_up, rigfrequ);
            ctrl->last_send_up_us = g_get_monotonic_time();
            set_ok = rigctrl_set_freq_for_role(ctrl, ctrl->sock, FALSE,
                                               FALSE, rigfrequ, NULL);
            if (set_ok)
            {
                g_usleep(WR_DEL);
                read_ok = rigctrl_get_freq_for_role(ctrl, ctrl->sock, FALSE,
                                                    FALSE, TRUE,
                                                    &readback, NULL);
                if (!read_ok)
                {
                    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                                "rig update: mode=FULL_DUPLEX_MAIN_SUB side=TX readback failed; keeping last");
                    rigctrl_update_last_sent(ctrl, FALSE, rigfrequ);
                }
                else if (rigctrl_verify_match(ctrl, FALSE, set_vfo,
                                              rigfrequ, readback, 0))
                {
                    ctrl->errcnt = 0;
                    ctrl->lasttxf = readback;
                    ctrl->rig_actual_up_hz = readback;
                    rigctrl_update_last_sent(ctrl, FALSE, readback);
                    rigctrl_set_freq_knob_value(ctrl, TRUE,
                                            (gdouble)readback);
                }
                else
                {
                    ctrl->errcnt++;
                    ctrl->lasttxf = 0;
                    rigctrl_update_last_sent(ctrl, FALSE, 0);
                }
            }
            else
            {
                ctrl->errcnt++;
            }
        }
    }
}

static void exec_duplex_tx_cycle(GtkRigCtrl * ctrl)
{
    gint64          readfreq = 0;
    gint64          tmpfreq = 0;
    gint64          base_sat = 0;
    gboolean        dialchanged = FALSE;
    gboolean        use_rit_xit =
        (ctrl->conf != NULL) ? ctrl->conf->supports_rit_xit : FALSE;
    gint64          doppler_hz = 0;
    gint64          sent_freq = 0;
    gint64          target_down = 0;
    gboolean        target_ok = TRUE;
    vfo_t           vfo = VFO_NONE;

    if (is_full_duplex_main_sub_active(ctrl))
        use_rit_xit = FALSE;

    /* Dial feedback:
       If radio device is engaged read frequency from radio and compare it to the
       last set frequency. If different, it means that user has changed frequency
       on the radio dial => update transponder knob

       Note: If ctrl->lasttxf = 0.0 the sync has been invalidated (e.g. user pressed "tune")
       and no need to execute the dial feedback.
     */
    if ((ctrl->engaged) && (ctrl->lasttxf > 0))
    {
        if (!rigctrl_get_freq_for_role(ctrl, ctrl->sock, FALSE,
                                       TRUE, FALSE, &readfreq, &vfo))
        {
            /* error => use a passive value */
            readfreq = ctrl->lasttxf;
        }

        if (llabs(readfreq - ctrl->lasttxf) >= 1)
        {
            dialchanged = TRUE;

            /* user might have altered radio frequency => update rig readback */
            rigctrl_set_freq_knob_value(ctrl, TRUE, (gdouble)readfreq);
            ctrl->lasttxf = readfreq;
            ctrl->rig_actual_up_hz = readfreq;

        }
    }

    /* now, forward tracking */
    if (dialchanged)
    {
        /* no need to forward track */
        return;
    }

    tmpfreq = rigctrl_compute_target(ctrl, FALSE, ctrl->conf->loup,
                                     use_rit_xit, &base_sat,
                                     &doppler_hz, &target_ok);
    target_down = rigctrl_compute_target(ctrl, TRUE, ctrl->conf->lo,
                                         use_rit_xit, NULL, NULL, NULL);
    rigctrl_update_last_target(ctrl, FALSE, tmpfreq);
    rigctrl_update_last_target(ctrl, TRUE, target_down);

    rigctrl_set_freq_knob_value(ctrl, TRUE, (gdouble)tmpfreq);
    rigctrl_set_freq_knob_value(ctrl, FALSE, (gdouble)target_down);

    sent_freq = tmpfreq;

    /* if device is engaged, send freq command to radio */
    if ((ctrl->engaged) && target_ok &&
        rigctrl_should_send_freq(ctrl, FALSE, tmpfreq))
    {
        {
            gboolean set_ok;
            gboolean read_ok;
            gint64   readback = 0;
            vfo_t   set_vfo = rigctrl_vfo_for_side(ctrl, FALSE);
            const gchar *vfo_label = rigctrl_vfo_label(ctrl, FALSE, set_vfo);

            rigctrl_log_send(ctrl, FALSE, vfo_label,
                             base_sat, doppler_hz, sent_freq);
            ctrl->last_send_up_us = g_get_monotonic_time();

            set_ok = rigctrl_set_freq_for_role(ctrl, ctrl->sock, FALSE,
                                               TRUE, tmpfreq, NULL);
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rig update: side=TX set=%s", set_ok ? "ok" : "fail");

            if (set_ok)
            {
                /* give radio a chance to set frequency */
                g_usleep(WR_DEL);

                /* The actual frequency migh be different from what we have set because
                   the tuning step is larger than what we work with (e.g. FT-817 has a
                   smallest tuning step of 10 Hz). Therefore we read back the actual
                   frequency from the rig. */
                read_ok = (set_vfo != VFO_NONE) ?
                    get_freq_toggle_vfo_strict(ctrl, ctrl->sock, &readback, set_vfo) :
                    get_freq_toggle_strict(ctrl, ctrl->sock, &readback);
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            "rig update: side=TX readback=%" G_GINT64_FORMAT " ok=%d",
                            readback, read_ok ? 1 : 0);

                if (!read_ok)
                {
                    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                                "rig update: side=TX readback failed; keeping last");
                    rigctrl_update_last_sent(ctrl, FALSE, sent_freq);
                }
                else if (rigctrl_verify_match(ctrl, FALSE, set_vfo,
                                              sent_freq, readback, 0))
                {
                    ctrl->errcnt = 0;
                    ctrl->lasttxf = readback;
                    ctrl->rig_actual_up_hz = readback;
                    rigctrl_update_last_sent(ctrl, FALSE, readback);
                    rigctrl_set_freq_knob_value(ctrl, TRUE,
                                            (gdouble)readback);
                }
                else
                {
                    ctrl->errcnt++;
                    ctrl->lasttxf = 0;
                    rigctrl_update_last_sent(ctrl, FALSE, 0);
                }
            }
            else
            {
                ctrl->errcnt++;
            }
        }
    }
}

static void exec_duplex_cycle(GtkRigCtrl * ctrl)
{
    exec_rx_cycle(ctrl);
    exec_duplex_tx_cycle(ctrl);
}

static void exec_dual_rig_cycle(GtkRigCtrl * ctrl)
{
    gint64          tmpfreq = 0;
    gint64          readfreq = 0;
    gint64          base_sat_down = 0;
    gint64          base_sat_up = 0;
    gint64          doppler_down = 0;
    gint64          doppler_up = 0;
    gboolean        down_dialchanged = FALSE;
    gboolean        up_dialchanged = FALSE;
    gboolean        rx_use_rit_xit =
        (ctrl->conf != NULL) ? ctrl->conf->supports_rit_xit : FALSE;
    gboolean        tx_use_rit_xit =
        (ctrl->conf2 != NULL) ? ctrl->conf2->supports_rit_xit : FALSE;
    gboolean        down_ok = TRUE;
    gboolean        up_ok = TRUE;
    vfo_t           down_vfo = VFO_NONE;
    vfo_t           up_vfo = VFO_NONE;

    if (ctrl->engaged && (ctrl->lastrxf > 0))
    {
        if (!rigctrl_get_freq_for_role(ctrl, ctrl->sock, TRUE,
                                       FALSE, FALSE, &readfreq, &down_vfo))
            readfreq = ctrl->lastrxf;

        if (llabs(readfreq - ctrl->lastrxf) >= 1)
        {
            down_dialchanged = TRUE;
            rigctrl_set_freq_knob_value(ctrl, FALSE,
                                    (gdouble)readfreq);
            ctrl->lastrxf = readfreq;
            ctrl->rig_actual_down_hz = readfreq;
        }
    }

    if (down_dialchanged)
    {
        if (ctrl->conf2 == NULL)
            return;

        tmpfreq = rigctrl_compute_target(ctrl, FALSE, ctrl->conf2->loup,
                                         tx_use_rit_xit, &base_sat_up,
                                         &doppler_up, &up_ok);
        rigctrl_update_last_target(ctrl, FALSE, tmpfreq);
        rigctrl_set_freq_knob_value(ctrl, TRUE, (gdouble)tmpfreq);

        if ((ctrl->engaged) && up_ok &&
            rigctrl_should_send_freq(ctrl, FALSE, tmpfreq))
        {
            vfo_t set_vfo = rigctrl_vfo_for_side(ctrl, FALSE);
            const gchar *vfo_label = rigctrl_vfo_label(ctrl, FALSE, set_vfo);
            gint64 readback = 0;
            gint64 target_freq = tmpfreq;

            rigctrl_log_send(ctrl, FALSE, vfo_label,
                             base_sat_up, doppler_up, target_freq);
            ctrl->last_send_up_us = g_get_monotonic_time();
            if (rigctrl_set_freq_for_role(ctrl, ctrl->sock2, FALSE,
                                          FALSE, target_freq, NULL))
            {
                ctrl->errcnt = 0;
                g_usleep(WR_DEL);
                if ((set_vfo != VFO_NONE) ?
                    get_freq_simplex_vfo_strict(ctrl, ctrl->sock2, &readback, set_vfo) :
                    get_freq_simplex_strict(ctrl, ctrl->sock2, &readback))
                {
                    if (rigctrl_verify_match(ctrl, FALSE, set_vfo,
                                             target_freq, readback, 0))
                    {
                        ctrl->lasttxf = readback;
                        ctrl->rig_actual_up_hz = readback;
                        rigctrl_update_last_sent(ctrl, FALSE, readback);
                        rigctrl_set_freq_knob_value(ctrl, TRUE,
                                                (gdouble)readback);
                    }
                    else
                    {
                        ctrl->errcnt++;
                        ctrl->lasttxf = 0;
                        rigctrl_update_last_sent(ctrl, FALSE, 0);
                    }
                }
                else
                {
                    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                                "rig update: side=TX readback failed; keeping last");
                    rigctrl_update_last_sent(ctrl, FALSE, target_freq);
                }
            }
            else
            {
                ctrl->errcnt++;
            }
        }
        return;
    }

    if (ctrl->conf == NULL)
        return;

    tmpfreq = rigctrl_compute_target(ctrl, TRUE, ctrl->conf->lo,
                                     rx_use_rit_xit, &base_sat_down,
                                     &doppler_down, &down_ok);
    rigctrl_update_last_target(ctrl, TRUE, tmpfreq);
    rigctrl_set_freq_knob_value(ctrl, FALSE, (gdouble)tmpfreq);

    if ((ctrl->engaged) && down_ok &&
        rigctrl_should_send_freq(ctrl, TRUE, tmpfreq))
    {
        vfo_t set_vfo = rigctrl_vfo_for_side(ctrl, TRUE);
        const gchar *vfo_label = rigctrl_vfo_label(ctrl, TRUE, set_vfo);
        gint64 readback = 0;
        gint64 target_freq = tmpfreq;

        rigctrl_log_send(ctrl, TRUE, vfo_label,
                         base_sat_down, doppler_down, target_freq);
        ctrl->last_send_down_us = g_get_monotonic_time();
        if (rigctrl_set_freq_for_role(ctrl, ctrl->sock, TRUE,
                                      FALSE, target_freq, NULL))
        {
            ctrl->errcnt = 0;
            g_usleep(WR_DEL);
            if ((set_vfo != VFO_NONE) ?
                get_freq_simplex_vfo_strict(ctrl, ctrl->sock, &readback, set_vfo) :
                get_freq_simplex_strict(ctrl, ctrl->sock, &readback))
            {
                if (rigctrl_verify_match(ctrl, TRUE, set_vfo,
                                         target_freq, readback, 0))
                {
                    ctrl->lastrxf = readback;
                    ctrl->rig_actual_down_hz = readback;
                    rigctrl_update_last_sent(ctrl, TRUE, readback);
                    rigctrl_set_freq_knob_value(ctrl, FALSE,
                                            (gdouble)readback);
                }
                else
                {
                    ctrl->errcnt++;
                    ctrl->lastrxf = 0;
                    rigctrl_update_last_sent(ctrl, TRUE, 0);
                }
            }
            else
            {
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            "rig update: side=RX readback failed; keeping last");
                rigctrl_update_last_sent(ctrl, TRUE, target_freq);
            }
        }
        else
        {
            ctrl->errcnt++;
        }
    }

    if (ctrl->conf2 == NULL)
        return;

    if (ctrl->engaged && (ctrl->lasttxf > 0))
    {
        if (!rigctrl_get_freq_for_role(ctrl, ctrl->sock2, FALSE,
                                       FALSE, FALSE, &readfreq, &up_vfo))
            readfreq = ctrl->lasttxf;

        if (llabs(readfreq - ctrl->lasttxf) >= 1)
        {
            up_dialchanged = TRUE;
            rigctrl_set_freq_knob_value(ctrl, TRUE, (gdouble)readfreq);
            ctrl->lasttxf = readfreq;
            ctrl->rig_actual_up_hz = readfreq;
        }
    }

    if (up_dialchanged)
        return;

    tmpfreq = rigctrl_compute_target(ctrl, FALSE, ctrl->conf2->loup,
                                     tx_use_rit_xit, &base_sat_up,
                                     &doppler_up, &up_ok);
    rigctrl_update_last_target(ctrl, FALSE, tmpfreq);
    rigctrl_set_freq_knob_value(ctrl, TRUE, (gdouble)tmpfreq);

    if ((ctrl->engaged) && up_ok &&
        rigctrl_should_send_freq(ctrl, FALSE, tmpfreq))
    {
        vfo_t set_vfo = rigctrl_vfo_for_side(ctrl, FALSE);
        const gchar *vfo_label = rigctrl_vfo_label(ctrl, FALSE, set_vfo);
        gint64 readback = 0;
        gint64 target_freq = tmpfreq;

        rigctrl_log_send(ctrl, FALSE, vfo_label,
                         base_sat_up, doppler_up, target_freq);
        ctrl->last_send_up_us = g_get_monotonic_time();
        if (rigctrl_set_freq_for_role(ctrl, ctrl->sock2, FALSE,
                                      FALSE, target_freq, NULL))
        {
            ctrl->errcnt = 0;
            g_usleep(WR_DEL);
            if ((set_vfo != VFO_NONE) ?
                get_freq_simplex_vfo_strict(ctrl, ctrl->sock2, &readback, set_vfo) :
                get_freq_simplex_strict(ctrl, ctrl->sock2, &readback))
            {
                if (rigctrl_verify_match(ctrl, FALSE, set_vfo,
                                         target_freq, readback, 0))
                {
                    ctrl->lasttxf = readback;
                    ctrl->rig_actual_up_hz = readback;
                    rigctrl_update_last_sent(ctrl, FALSE, readback);
                    rigctrl_set_freq_knob_value(ctrl, TRUE,
                                            (gdouble)readback);
                }
                else
                {
                    ctrl->errcnt++;
                    ctrl->lasttxf = 0;
                    rigctrl_update_last_sent(ctrl, FALSE, 0);
                }
            }
            else
            {
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            "rig update: side=TX readback failed; keeping last");
                rigctrl_update_last_sent(ctrl, FALSE, target_freq);
            }
        }
        else
        {
            ctrl->errcnt++;
        }
    }
}

static gboolean get_ptt(GtkRigCtrl * ctrl, gint sock)
{
    gchar          *buff, **vbuff;
    gchar           buffback[128];
    gboolean        retcode;
    guint64         pttstat = 0;

    if (ctrl->conf->ptt == PTT_TYPE_CAT)
    {
        /* send command get_ptt (t) */
        if (ctrl->conf->vfo_opt)
            buff = g_strdup_printf("t currVFO\x0a");
        else
            buff = g_strdup_printf("t\x0a");
    }
    else
    {
        /* send command \get_dcd */
        if (ctrl->conf->vfo_opt)
            buff = g_strdup_printf("%c currVFO\x0a", 0x8b);
        else
            buff = g_strdup_printf("%c\x0a", 0x8b);
    }

    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    if (retcode)
    {
        vbuff = g_strsplit(buffback, "\n", 3);
        if (vbuff[0])
            pttstat = g_ascii_strtoull(vbuff[0], NULL, 0);      //FIXME base = 0 ok?
        g_strfreev(vbuff);
    }

    g_free(buff);

    return (pttstat == 1) ? TRUE : FALSE;
}

static gboolean set_ptt(GtkRigCtrl * ctrl, gint sock, gboolean ptt)
{
    gchar          *buff;
    gchar           buffback[128];
    gboolean        retcode;

    /* send command */
    if (ptt == TRUE) 
    {
        if (ctrl->conf->vfo_opt)
            buff = g_strdup_printf("T currVFO 1\x0aq\x0a");
        else
            buff = g_strdup_printf("T 1\x0aq\x0a");
    }
    else
    {
        if (ctrl->conf->vfo_opt)
            buff = g_strdup_printf("T currVFO 0\x0aq\x0a");
        else
            buff = g_strdup_printf("T 0\x0aq\x0a");
    }

    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    g_free(buff);

    return (check_set_response(buffback, retcode, __func__));

}

static radio_conf_t *get_conf_for_socket(GtkRigCtrl * ctrl, gint sock)
{
    if (ctrl->conf2 != NULL && sock == ctrl->sock2)
        return ctrl->conf2;

    return ctrl->conf;
}

static RigSession *rig_session_for_socket(GtkRigCtrl *ctrl, gint sock)
{
    if (ctrl == NULL)
        return NULL;

    if (ctrl->rig_session2 != NULL && sock == ctrl->sock2)
        return ctrl->rig_session2;

    return ctrl->rig_session;
}

static RigSession *rig_session_for_socket_vfo(GtkRigCtrl *ctrl, gint sock,
                                              vfo_t vfo)
{
    RigSession *session = rig_session_for_socket(ctrl, sock);
    vfo_t down_vfo = VFO_NONE;
    vfo_t up_vfo = VFO_NONE;

    if (ctrl == NULL || session == NULL)
        return session;

    if (ctrl->conf2 != NULL || !is_full_duplex_main_sub_configured(ctrl->conf))
        return session;

    if (ctrl->rig_session2 == NULL || vfo == VFO_NONE)
        return session;

    down_vfo = rigctrl_target_vfo_for_role(ctrl->conf, VFO_ROLE_DOWNLINK);
    up_vfo = rigctrl_target_vfo_for_role(ctrl->conf, VFO_ROLE_UPLINK);

    if (vfo == down_vfo)
        return ctrl->rig_session;
    if (vfo == up_vfo)
        return ctrl->rig_session2;

    return session;
}

static RigctldClient *rigctld_client_for_socket(GtkRigCtrl *ctrl, gint sock)
{
    if (ctrl == NULL)
        return NULL;

    if (ctrl->rig_client2 != NULL && sock == ctrl->sock2)
        return ctrl->rig_client2;

    return ctrl->rig_client;
}

static rig_strategy_t rig_session_strategy(GtkRigCtrl *ctrl, gint sock)
{
    RigSession *session = rig_session_for_socket(ctrl, sock);

    if (session == NULL)
        return RIG_STRATEGY_PLAIN_FREQ;

    return session->strategy;
}

static rig_strategy_t rig_session_strategy_for_vfo(GtkRigCtrl *ctrl,
                                                   gint sock,
                                                   vfo_t vfo)
{
    RigSession *session = rig_session_for_socket_vfo(ctrl, sock, vfo);

    if (session == NULL)
        return RIG_STRATEGY_PLAIN_FREQ;

    return session->strategy;
}

static const gchar *rig_session_default_vfo_token(RigSession *session)
{
    if (session == NULL)
        return "currVFO";

    if (session->default_vfo_token != NULL)
        return session->default_vfo_token;

    return "currVFO";
}

static gboolean rigctld_xit_supported_for_socket(GtkRigCtrl *ctrl, gint sock)
{
    if (ctrl == NULL)
        return FALSE;

    if (sock == ctrl->sock2)
        return ctrl->xit_supported2;

    return ctrl->xit_supported;
}

static void rigctld_disable_xit_for_socket(GtkRigCtrl *ctrl, gint sock)
{
    if (ctrl == NULL)
        return;

    if (sock == ctrl->sock2)
    {
        if (!ctrl->xit_supported2)
            return;
        ctrl->xit_supported2 = FALSE;
    }
    else
    {
        if (!ctrl->xit_supported)
            return;
        ctrl->xit_supported = FALSE;
    }

    sat_log_log(SAT_LOG_LEVEL_INFO,
                "XIT unsupported; disabling for %s rig",
                sock == ctrl->sock2 ? "uplink" : "primary");
    rig_term_log(ctrl, "gpredict",
                 "XIT unsupported; disabling for %s rig",
                 sock == ctrl->sock2 ? "uplink" : "primary");
}

static gboolean set_rit(GtkRigCtrl * ctrl, gint sock, gdouble hz)
{
    gchar          *buff;
    gchar           buffback[128];
    gboolean        retcode;
    radio_conf_t   *conf = get_conf_for_socket(ctrl, sock);
    gint            offset = (gint) llround(hz);

    if (ctrl->last_rit_valid && offset == ctrl->last_rit_offset)
        return FALSE;

    if (conf != NULL && conf->vfo_opt)
        buff = g_strdup_printf("J currVFO %d\x0a", offset);
    else
        buff = g_strdup_printf("J %d\x0a", offset);

    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    g_free(buff);

    retcode = check_set_response(buffback, retcode, __func__);
    if (retcode == FALSE)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: Failed to set RIT offset to %d Hz"),
                    __func__, offset);
    }

    if (retcode)
    {
        ctrl->last_rit_valid = TRUE;
        ctrl->last_rit_offset = offset;
    }

    return retcode;
}

static gboolean set_xit(GtkRigCtrl * ctrl, gint sock, gdouble hz)
{
    gchar          *buff;
    gchar           buffback[128];
    gboolean        retcode;
    radio_conf_t   *conf = get_conf_for_socket(ctrl, sock);
    gint            offset = (gint) llround(hz);
    gint            rprt = 0;
    gboolean       *valid_ptr = NULL;
    gint           *last_ptr = NULL;

    if (sock == ctrl->sock2)
    {
        valid_ptr = &ctrl->last_xit_valid2;
        last_ptr = &ctrl->last_xit_offset2;
    }
    else
    {
        valid_ptr = &ctrl->last_xit_valid;
        last_ptr = &ctrl->last_xit_offset;
    }

    if (!rigctld_xit_supported_for_socket(ctrl, sock))
        return FALSE;

    if (valid_ptr != NULL && last_ptr != NULL &&
        *valid_ptr && offset == *last_ptr)
        return FALSE;

    if (conf != NULL && conf->vfo_opt)
        buff = g_strdup_printf("Z currVFO %d\x0a", offset);
    else
        buff = g_strdup_printf("Z %d\x0a", offset);

    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    g_free(buff);

    if (retcode && rig_parse_rprt_code_any(buffback, &rprt) && rprt == -11)
    {
        rigctld_disable_xit_for_socket(ctrl, sock);
        return FALSE;
    }

    retcode = check_set_response(buffback, retcode, __func__);
    if (retcode == FALSE)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: Failed to set XIT offset to %d Hz"),
                    __func__, offset);
    }

    if (retcode && valid_ptr != NULL && last_ptr != NULL)
    {
        *valid_ptr = TRUE;
        *last_ptr = offset;
    }

    return retcode;
}

static void apply_rit_xit_offsets(GtkRigCtrl * ctrl, gdouble rit,
                                  gdouble xit)
{
    gboolean        applied_rit = FALSE;
    gboolean        applied_xit = FALSE;
    gboolean        rx_support =
        (ctrl->conf != NULL) ? ctrl->conf->supports_rit_xit : FALSE;
    gboolean        tx_support = FALSE;
    gint            tx_sock = ctrl->sock;

    if (ctrl->conf2 != NULL)
    {
        tx_support = ctrl->conf2->supports_rit_xit;
        tx_sock = ctrl->sock2;
    }
    else
    {
        tx_support = rx_support;
    }

    if (is_full_duplex_main_sub_configured(ctrl->conf))
        return;

    if (rx_support && ctrl->sock >= 0)
        applied_rit = set_rit(ctrl, ctrl->sock, rit);

    if (tx_support && tx_sock >= 0)
        applied_xit = set_xit(ctrl, tx_sock, xit);

    if (applied_rit || applied_xit)
    {
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    _("RIT=%+.0f Hz XIT=%+.0f Hz"),
                    applied_rit ? rit : 0.0, applied_xit ? xit : 0.0);
    }
}

static void update_rit_xit_offsets(GtkRigCtrl * ctrl)
{
    gboolean        rx_support =
        (ctrl->conf != NULL) ? ctrl->conf->supports_rit_xit : FALSE;
    gboolean        tx_support =
        (ctrl->conf2 != NULL) ? ctrl->conf2->supports_rit_xit : rx_support;

    if (ctrl->engaged == FALSE)
        return;

    if (is_full_duplex_main_sub_active(ctrl))
        return;

    if (!rx_support && !tx_support)
        return;

    apply_rit_xit_offsets(ctrl,
                          ctrl->rx_track_enabled ?
                              rigctrl_get_cached_doppler(ctrl, TRUE) : 0.0,
                          ctrl->tx_track_enabled ?
                              rigctrl_get_cached_doppler(ctrl, FALSE) : 0.0);
}

/*
 * Check for AOS and LOS and send signal if enabled for rig.
 *
 * @param ctrl Pointer to the GtkRigCtrl handle.
 * @return TRUE if the operation was successful, FALSE if a connection error
 *         occurred.
 *
 * This function checks whether AOS or LOS just happened and sends the
 * appropriate signal to the RIG if this signalling is enabled.
 */
static gboolean check_aos_los(GtkRigCtrl * ctrl)
{
    gboolean        retcode = TRUE;
    gchar           retbuf[10];

    if (ctrl->engaged && (ctrl->rx_track_enabled || ctrl->tx_track_enabled))
    {
        if (ctrl->prev_ele < 0.0 && ctrl->target->el >= 0.0)
        {
            /* AOS has occurred */
            if (ctrl->conf->signal_aos && ctrl->sock >= 0)
            {
                retcode &= send_rigctld_command(ctrl, ctrl->sock, "AOS\n",
                                                retbuf, 10);
            }
            if (ctrl->conf2 != NULL)
            {
                if (ctrl->conf2->signal_aos && ctrl->sock2 >= 0)
                {
                    retcode &= send_rigctld_command(ctrl, ctrl->sock2, "AOS\n",
                                                    retbuf, 10);
                }
            }
        }
        else if (ctrl->prev_ele >= 0.0 && ctrl->target->el < 0.0)
        {
            /* LOS has occurred */
            if (ctrl->conf->signal_los && ctrl->sock >= 0)
            {
                retcode &= send_rigctld_command(ctrl, ctrl->sock, "LOS\n",
                                                retbuf, 10);
            }
            if (ctrl->conf2 != NULL)
            {
                if (ctrl->conf2->signal_los && ctrl->sock2 >= 0)
                {
                    retcode &= send_rigctld_command(ctrl, ctrl->sock2, "LOS\n",
                                                    retbuf, 10);
                }
            }
        }
    }

    ctrl->prev_ele = ctrl->target->el;

    return retcode;
}

/*
 * Set frequency in simplex mode
 *
 * Returns TRUE if the operation was successful, FALSE otherwise
 */
static gboolean set_freq_simplex(GtkRigCtrl * ctrl, gint sock, gint64 freq)
{
    gchar          *buff;
    gchar           buffback[128];
    gboolean        retcode;
    RigSession     *session = rig_session_for_socket(ctrl, sock);
    rig_strategy_t  strategy = rig_session_strategy(ctrl, sock);
    gchar           freq_str[32];

    if (strategy == RIG_STRATEGY_SELECT_VFO)
    {
        const gchar *token = rig_session_default_vfo_token(session);

        if (token != NULL && *token != '\0')
        {
            if (!rigctld_select_vfo_cached(ctrl, sock, VFO_NONE, token))
                return FALSE;
        }
    }

    hz_to_rigctld_string(freq, freq_str, sizeof(freq_str));

    if (strategy == RIG_STRATEGY_VFO_OPT_ARGS)
    {
        const gchar *token = rig_session_default_vfo_token(session);
        buff = g_strdup_printf("F %s %s\x0a", token, freq_str);
    }
    else
    {
        buff = g_strdup_printf("F %s\x0a", freq_str);
    }
    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    g_free(buff);

    return (check_set_response(buffback, retcode, __func__));
}


/*
 * Set frequency in toggle mode
 *
 * Returns TRUE if the operation was successful, FALSE otherwise
 */
static gboolean set_freq_toggle(GtkRigCtrl * ctrl, gint sock, gint64 freq)
{
    gchar          *buff;
    gchar           buffback[128];
    gboolean        retcode;
    RigSession     *session = rig_session_for_socket(ctrl, sock);
    rig_strategy_t  strategy = rig_session_strategy(ctrl, sock);
    gchar           freq_str[32];

    /* send command */
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                _("%s: set_freq_toggle vfo_opt=%d freq=%" G_GINT64_FORMAT),
                __func__, ctrl->conf->vfo_opt, freq);
    hz_to_rigctld_string(freq, freq_str, sizeof(freq_str));

    if (strategy == RIG_STRATEGY_SELECT_VFO)
    {
        const gchar *token = rig_session_default_vfo_token(session);

        if (token != NULL && *token != '\0')
        {
            if (!rigctld_select_vfo_cached(ctrl, sock, VFO_NONE, token))
                return FALSE;
        }
    }
    if (strategy == RIG_STRATEGY_VFO_OPT_ARGS)
    {
        const gchar *token = rig_session_default_vfo_token(session);
        buff = g_strdup_printf("I %s %s\x0a", token, freq_str);
    }
    else
    {
        buff = g_strdup_printf("I %s\x0a", freq_str);
    }

    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    g_free(buff);

    return (check_set_response(buffback, retcode, __func__));

}

/*
 * Turn on the radios toggle mode
 *
 * Returns TRUE if the operation was successful
 */
static gboolean set_toggle(GtkRigCtrl * ctrl, gint sock)
{
    gchar          *buff;
    gchar           buffback[128];
    gboolean        retcode;

    if (ctrl->conf->vfo_opt)
    buff = g_strdup_printf("S %s 1 %d\x0a", ctrl->conf->downlink_vfo==VFO_A?"VFOA":"VFOB", ctrl->conf->downlink_vfo);
    else
    buff = g_strdup_printf("S 1 %d\x0a", ctrl->conf->downlink_vfo);
    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    g_free(buff);

    return (check_set_response(buffback, retcode, __func__));
}

/*
 * Turn off the radios toggle mode
 *
 * Returns TRUE if the operation was successful
 */
static gboolean unset_toggle(GtkRigCtrl * ctrl, gint sock)
{
    gchar          *buff;
    gchar           buffback[128];
    gboolean        retcode;

    /* send command */
    if (ctrl->conf->vfo_opt)
        buff = g_strdup_printf("S VFOA 0 %d\x0a", ctrl->conf->downlink_vfo);
    else
        buff = g_strdup_printf("S 0 %d\x0a", ctrl->conf->downlink_vfo);
    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    g_free(buff);

    return (check_set_response(buffback, retcode, __func__));
}

/*
 * Get frequency
 *
 * Returns TRUE if the operation was successful, FALSE otherwise
 */
static gboolean get_freq_simplex_internal(GtkRigCtrl * ctrl, gint sock,
                                          gint64 * freq, gboolean allow_cache)
{
    gchar          *buff, **vbuff;
    gchar           buffback[128];
    gboolean        retcode;
    gboolean        retval = TRUE;
    const gchar    *label = ctrl->conf->vfo_opt ? "currVFO" : "default";
    const gint      cache_key = -1;
    RigSession     *session = rig_session_for_socket(ctrl, sock);
    rig_strategy_t  strategy = rig_session_strategy(ctrl, sock);

    if (strategy == RIG_STRATEGY_SELECT_VFO)
    {
        const gchar *token = rig_session_default_vfo_token(session);

        if (token != NULL && *token != '\0')
        {
            if (!rigctld_select_vfo_cached(ctrl, sock, VFO_NONE, token))
                goto fallback;
        }
    }

    if (strategy == RIG_STRATEGY_VFO_OPT_ARGS)
    {
        const gchar *token = rig_session_default_vfo_token(session);
        buff = g_strdup_printf("f %s\x0a", token);
    }
    else
    {
        buff = g_strdup_printf("f\x0a");
    }
    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    retcode = check_get_response(buffback, retcode, __func__);
    if (retcode)
    {
        vbuff = g_strsplit(buffback, "\n", 3);
        if (vbuff[0])
        {
            gchar *endptr = NULL;
            *freq = g_ascii_strtoll(vbuff[0], &endptr, 10);
            if (endptr == vbuff[0])
                retval = FALSE;
        }
        else
            retval = FALSE;
        g_strfreev(vbuff);
    }
    else
    {
        retval = FALSE;
    }

    g_free(buff);
    if (retval)
    {
        rigctrl_cache_freq(sock, "f", cache_key, *freq);
        return TRUE;
    }

fallback:
    if (allow_cache && rigctrl_get_cached_freq(sock, "f", cache_key, freq))
    {
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "get %s failed; using cached %" G_GINT64_FORMAT,
                    label, *freq);
        return TRUE;
    }

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "get %s failed",
                label);
    return FALSE;
}

static gboolean get_freq_simplex(GtkRigCtrl * ctrl, gint sock, gint64 * freq)
{
    return get_freq_simplex_internal(ctrl, sock, freq, TRUE);
}

static gboolean get_freq_simplex_strict(GtkRigCtrl * ctrl, gint sock,
                                        gint64 * freq)
{
    return get_freq_simplex_internal(ctrl, sock, freq, FALSE);
}

/*
 * Get frequency when the radio is working toggle
 *
 * Returns TRUE if the operation was successful, FALSE otherwise
 */
static gboolean get_freq_toggle_internal(GtkRigCtrl * ctrl, gint sock,
                                         gint64 * freq, gboolean allow_cache)
{
    gchar          *buff = NULL;
    gchar         **vbuff;
    gchar           buffback[128];
    gboolean        retcode;
    gboolean        retval = TRUE;
    const gchar    *label = ctrl->conf->vfo_opt ? "currVFO" : "default";
    const gint      cache_key = -2;
    RigSession     *session = rig_session_for_socket(ctrl, sock);
    rig_strategy_t  strategy = rig_session_strategy(ctrl, sock);

    if (freq == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%d: NULL storage."), __FILE__, __LINE__);
        return FALSE;
    }

    if (strategy == RIG_STRATEGY_SELECT_VFO)
    {
        const gchar *token = rig_session_default_vfo_token(session);

        if (token != NULL && *token != '\0')
        {
            if (!rigctld_select_vfo_cached(ctrl, sock, VFO_NONE, token))
                goto fallback;
        }
    }

    /* send command */
    if (strategy == RIG_STRATEGY_VFO_OPT_ARGS)
    {
        const gchar *token = rig_session_default_vfo_token(session);
        buff = g_strdup_printf("i %s\x0a", token);
    }
    else
    {
        buff = g_strdup_printf("i\x0a");
    }
    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    retcode = check_get_response(buffback, retcode, __func__);
    if (retcode)
    {
        vbuff = g_strsplit(buffback, "\n", 3);
        if (vbuff[0])
        {
            gchar *endptr = NULL;
            *freq = g_ascii_strtoll(vbuff[0], &endptr, 10);
            if (endptr == vbuff[0])
                retval = FALSE;
        }
        else
            retval = FALSE;

        g_strfreev(vbuff);
    }
    else
    {
        retval = FALSE;
    }

    g_free(buff);
    if (retval)
    {
        rigctrl_cache_freq(sock, "i", cache_key, *freq);
        return TRUE;
    }

fallback:
    if (allow_cache && rigctrl_get_cached_freq(sock, "i", cache_key, freq))
    {
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "get %s (toggle) failed; using cached %" G_GINT64_FORMAT,
                    label, *freq);
        return TRUE;
    }

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "get %s (toggle) failed",
                label);
    return FALSE;
}

static gboolean get_freq_toggle(GtkRigCtrl * ctrl, gint sock, gint64 * freq)
{
    return get_freq_toggle_internal(ctrl, sock, freq, TRUE);
}

static gboolean get_freq_toggle_strict(GtkRigCtrl * ctrl, gint sock,
                                       gint64 * freq)
{
    return get_freq_toggle_internal(ctrl, sock, freq, FALSE);
}

/*
 * This function is used to manage PTT events, e.g. the user presses
 * the spacebar. It is only useful for RIG_TYPE_TOGGLE_MAN and possibly for
 * RIG_TYPE_TOGGLE_AUTO.
 *
 * First, the function will try to lock the controller. If the lock is
 * acquired the function checks the current PTT status.
 * If PTT status is FALSE (off), it will set the TX frequency and set PTT to
 * TRUE (on). If PTT status is TRUE (on) it will simply set the PTT to FALSE
 * (off).
 *
 * This function assumes that the radio support set/get PTT, otherwise it makes
 * no sense to use it!
 */
static void manage_ptt_event(GtkRigCtrl * ctrl)
{
    guint           timeout = 1;
    gboolean        ptt = FALSE;

    /* wait for controller to be idle or until the timeout triggers */
    while (timeout < 5)
    {
        if (g_mutex_trylock(&(ctrl->busy)) == TRUE)
        {
            timeout = 17;       /* use an arbitrary value that is large enough */
        }
        else
        {
            /* wait for 100 msec */
            g_usleep(100000);
            timeout++;
        }
    }

    if (timeout == 17)
    {
        /* timeout did not expire, we've got the controller lock */
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    _("%s: Acquired controller lock"), __func__);

        if (ctrl->engaged == FALSE)
        {
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        _("%s: Controller not engaged; PTT event ignored "
                          "(Hint: Enable the Engage button)"), __func__);
        }
        else
        {
            ptt = get_ptt(ctrl, ctrl->sock);

            if (ptt == FALSE)
            {
                /* PTT is OFF => set TX freq then set PTT to ON */
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            _("%s: PTT is OFF => Set TX freq and PTT=ON"),
                            __func__);

                exec_toggle_tx_cycle(ctrl);
                set_ptt(ctrl, ctrl->sock, TRUE);
            }
            else
            {
                /* PTT is ON => set to OFF */
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            _("%s: PTT is ON = Set PTT=OFF"), __func__);

                set_ptt(ctrl, ctrl->sock, FALSE);
            }
        }

        g_mutex_unlock(&(ctrl->busy));
    }
    else
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: Failed to acquire controller lock; PTT event "
                      "not handled"), __func__);
    }

}


/*
 * Catch events when the user presses the SPACE key on the keyboard.
 * This is used to toggle betweer RX/TX when using FT817/857/897 in manual mode.
 */
static gboolean key_press_cb(GtkWidget * widget, GdkEventKey * pKey,
                             gpointer data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(widget);
    gboolean        event_managed = FALSE;

    (void)data;

    if (pKey->type == GDK_KEY_PRESS)
    {
        switch (pKey->keyval)
        {
            /* keyvals not in API docs. See <gdk/gdkkeysyms.h> for a complete list */
        case GDK_KEY_space:
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        _("%s: Detected SPACEBAR pressed event"), __func__);

            /* manage PTT event but only if rig is of type TOGGLE_MAN */
            if (ctrl->conf->type == RIG_TYPE_TOGGLE_MAN)
            {
                manage_ptt_event(ctrl);
                event_managed = TRUE;
            }
            break;

        default:
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        _
                        ("%s:%s: Keypress value %i not managed by this function"),
                        __FILE__, __func__, pKey->keyval);
            break;
        }
    }

    return event_managed;
}

static void rigctld_close_fd(gint fd)
{
#ifndef WIN32
    if (fd >= 0)
        close(fd);
#else
    if (fd >= 0)
        closesocket(fd);
#endif
}

static void rigctld_apply_socket_timeouts(gint fd)
{
#ifndef WIN32
    struct timeval tv;

    tv.tv_sec = RIGCTLD_SOCKET_TIMEOUT_MS / 1000;
    tv.tv_usec = (RIGCTLD_SOCKET_TIMEOUT_MS % 1000) * 1000;
    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) != 0)
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    _("%s: failed to set SO_RCVTIMEO"), __func__);
    if (setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) != 0)
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    _("%s: failed to set SO_SNDTIMEO"), __func__);
#else
    DWORD timeout = RIGCTLD_SOCKET_TIMEOUT_MS;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout,
               sizeof(timeout));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (const char *)&timeout,
               sizeof(timeout));
#endif
}

static gboolean rigctld_serial_device_exists(const gchar *device)
{
    if (device == NULL || *device == '\0')
        return FALSE;

#ifdef G_OS_WIN32
    {
        const gchar *path = device;
        gchar *alt_path = NULL;
        HANDLE handle;
        DWORD err = 0;
        gboolean exists = FALSE;

        if (!g_str_has_prefix(device, "\\\\.\\") &&
            g_ascii_strncasecmp(device, "COM", 3) == 0)
        {
            alt_path = g_strdup_printf("\\\\.\\%s", device);
            path = alt_path;
        }

        handle = CreateFileA(path,
                             0,
                             FILE_SHARE_READ | FILE_SHARE_WRITE,
                             NULL,
                             OPEN_EXISTING,
                             FILE_ATTRIBUTE_NORMAL,
                             NULL);
        if (handle != INVALID_HANDLE_VALUE)
        {
            exists = TRUE;
            CloseHandle(handle);
        }
        else
        {
            err = GetLastError();
            if (err == ERROR_ACCESS_DENIED)
                exists = TRUE;
        }

        g_free(alt_path);
        return exists;
    }
#else
    return g_file_test(device, G_FILE_TEST_EXISTS);
#endif
}

static void rigctld_apply_socket_timeouts_ms(gint fd, gint timeout_ms)
{
#ifndef WIN32
    struct timeval tv;

    if (timeout_ms <= 0)
        timeout_ms = RIGCTLD_SOCKET_TIMEOUT_MS;
    tv.tv_sec = timeout_ms / 1000;
    tv.tv_usec = (timeout_ms % 1000) * 1000;
    if (setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv)) != 0)
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    _("%s: failed to set SO_RCVTIMEO"), __func__);
    if (setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv)) != 0)
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    _("%s: failed to set SO_SNDTIMEO"), __func__);
#else
    DWORD timeout;

    if (timeout_ms <= 0)
        timeout_ms = RIGCTLD_SOCKET_TIMEOUT_MS;
    timeout = (DWORD) timeout_ms;
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (const char *)&timeout,
               sizeof(timeout));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, (const char *)&timeout,
               sizeof(timeout));
#endif
}

static const gchar *rigctld_cached_device(radio_model_t model)
{
    if (rigctld_device_cache == NULL)
        return NULL;

    return g_hash_table_lookup(rigctld_device_cache, GINT_TO_POINTER(model));
}

static void rigctld_cache_device(radio_model_t model, const gchar *device)
{
    if (device == NULL || *device == '\0')
        return;

    if (rigctld_device_cache == NULL)
    {
        rigctld_device_cache =
            g_hash_table_new_full(g_direct_hash, g_direct_equal, NULL, g_free);
    }

    g_hash_table_replace(rigctld_device_cache,
                         GINT_TO_POINTER(model),
                         g_strdup(device));
}

static gboolean rigctld_try_autodetect_restart(GtkRigCtrl *ctrl,
                                               radio_conf_t *conf,
                                               RigctldMgr **mgr,
                                               gboolean secondary,
                                               const gchar *role,
                                               gboolean *reported,
                                               gint *restart_attempts,
                                               const gchar *reason)
{
    if (restart_attempts == NULL || conf == NULL)
        return FALSE;

    if (*restart_attempts >= RIGCTLD_AUTOSTART_MAX_RESTARTS)
        return FALSE;

    if (mgr == NULL || *mgr == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_WARN,
                    _("%s: autodetect restart skipped; rigctld not managed"),
                    __func__);
        rig_term_log(ctrl, "gpredict:err",
                     "autodetect restart skipped; rigctld not managed");
        return FALSE;
    }

    (*restart_attempts)++;
    sat_log_log(SAT_LOG_LEVEL_INFO,
                _("%s: restarting rigctld with autodetect (%s) attempt=%d"),
                __func__,
                reason ? reason : "autodetect",
                *restart_attempts);
    rig_term_log(ctrl, "gpredict",
                 "restarting rigctld with autodetect (%s) attempt=%d",
                 reason ? reason : "autodetect",
                 *restart_attempts);
    rigctld_terminate_spawned(ctrl, secondary, mgr);

    if (!rigctld_autodetect_device(ctrl, conf, role, reported))
        return FALSE;

    rigctld_persist_device(ctrl, conf, reason);
    g_usleep(RIGCTLD_AUTOSTART_RETRY_DELAY_MS * 1000);
    return TRUE;
}

static void rigctld_persist_device(GtkRigCtrl *ctrl, radio_conf_t *conf,
                                   const gchar *reason)
{
    if (conf == NULL || conf->name == NULL ||
        conf->rigctld_device == NULL || *conf->rigctld_device == '\0')
        return;

    radio_conf_save(conf);
    sat_log_log(SAT_LOG_LEVEL_INFO,
                _("%s: saved autodetect device %s (%s)"),
                __func__, conf->rigctld_device,
                reason ? reason : "autodetect");
    rig_term_log(ctrl, "gpredict",
                 "saved autodetect device %s (%s)",
                 conf->rigctld_device,
                 reason ? reason : "autodetect");
}

static GSList *rigctld_prioritize_candidate(GSList *list,
                                            const gchar *candidate)
{
    GSList *found;
    gpointer data = NULL;

    if (candidate == NULL || *candidate == '\0')
        return list;

    found = g_slist_find_custom(list, candidate, (GCompareFunc)g_strcmp0);
    if (found != NULL)
    {
        data = found->data;
        list = g_slist_delete_link(list, found);
        return g_slist_prepend(list, data);
    }

    return g_slist_prepend(list, g_strdup(candidate));
}

static gboolean rigctld_candidate_matches_allowlist(const gchar *candidate,
                                                    const gchar *allowlist_lc)
{
    gchar *candidate_lc = NULL;
    gboolean match = FALSE;

    if (allowlist_lc == NULL || *allowlist_lc == '\0')
        return TRUE;

    if (candidate == NULL || *candidate == '\0')
        return FALSE;

    candidate_lc = g_ascii_strdown(candidate, -1);
    match = (g_strrstr(candidate_lc, allowlist_lc) != NULL);
    g_free(candidate_lc);

    return match;
}

static gboolean rigctld_candidate_blocked_default(const gchar *candidate)
{
    gchar *candidate_lc = NULL;
    gboolean blocked = FALSE;

    if (candidate == NULL || *candidate == '\0')
        return FALSE;

    candidate_lc = g_ascii_strdown(candidate, -1);
    blocked = (g_strrstr(candidate_lc, "slab_usbtouart") != NULL);
    g_free(candidate_lc);

    return blocked;
}

#ifdef G_OS_WIN32
static gint rigctld_candidate_windows_com_number(const gchar *candidate)
{
    const gchar *digits = NULL;

    if (!gp_serial_port_is_windows_com(candidate))
        return 0;

    if (g_ascii_strncasecmp(candidate, "\\\\.\\COM", 7) == 0)
        digits = candidate + 7;
    else if (g_ascii_strncasecmp(candidate, "COM", 3) == 0)
        digits = candidate + 3;

    if (digits == NULL || *digits == '\0')
        return 0;

    return (gint) g_ascii_strtoll(digits, NULL, 10);
}
#endif

static gint rigctld_autodetect_candidate_score(const gchar *candidate)
{
    gchar *candidate_lc = NULL;
    gint score = 0;

    if (candidate == NULL || *candidate == '\0')
        return 0;

    candidate_lc = g_ascii_strdown(candidate, -1);
#ifdef __APPLE__
    if (g_str_has_prefix(candidate_lc, "/dev/cu.") ||
        g_str_has_prefix(candidate_lc, "cu."))
        score += 40;
    if (g_str_has_prefix(candidate_lc, "/dev/tty.") ||
        g_str_has_prefix(candidate_lc, "tty."))
        score -= 10;
#endif
    if (g_strrstr(candidate_lc, "usbserial") != NULL)
        score += 30;
    if (g_strrstr(candidate_lc, "usbmodem") != NULL)
        score += 20;
    if (g_strrstr(candidate_lc, "slab") != NULL)
        score -= 20;
    if (g_strrstr(candidate_lc, "ft06hpd7") != NULL)
        score -= 15;

#ifdef G_OS_WIN32
    {
        gint com_number = rigctld_candidate_windows_com_number(candidate);

        if (com_number > 0)
        {
            /* Prefer typical USB-assigned COM ports over legacy motherboard
             * ports like COM1/COM2, which often stall rigctld on Windows.
             */
            if (com_number <= 2)
                score -= 60;
            else if (com_number <= 4)
                score -= 10;
            else
                score += MIN(com_number, 20);
        }
    }
#endif

    g_free(candidate_lc);
    return score;
}

static gint rigctld_autodetect_compare_candidates(gconstpointer a,
                                                  gconstpointer b)
{
    const gchar *cand_a = a;
    const gchar *cand_b = b;
    gint score_a = rigctld_autodetect_candidate_score(cand_a);
    gint score_b = rigctld_autodetect_candidate_score(cand_b);

    if (score_a != score_b)
        return score_b - score_a;

    return g_strcmp0(cand_a, cand_b);
}

static GSList *rigctld_autodetect_filter_candidates(GSList *candidates,
                                                    const gchar *allowlist_lc,
                                                    guint *filtered_out)
{
    GSList *filtered = NULL;
    GSList *item = NULL;
    guint skipped = 0;

    for (item = candidates; item != NULL; item = item->next)
    {
        const gchar *candidate = item->data;

        if (allowlist_lc != NULL && *allowlist_lc != '\0')
        {
            if (!rigctld_candidate_matches_allowlist(candidate, allowlist_lc))
            {
                skipped++;
                continue;
            }
        }
        else if (rigctld_candidate_blocked_default(candidate))
        {
            skipped++;
            continue;
        }

        filtered = g_slist_append(filtered, g_strdup(candidate));
    }

    gp_serial_free_candidates(candidates);
    if (filtered_out)
        *filtered_out = skipped;

    return g_slist_sort(filtered, rigctld_autodetect_compare_candidates);
}

static gint rigctld_pick_ephemeral_port(gint avoid_port)
{
    gint selected = -1;

    for (gint attempt = 0; attempt < 5; attempt++)
    {
        GSocket *sock = NULL;
        GInetAddress *addr = NULL;
        GSocketAddress *sockaddr = NULL;
        GSocketAddress *local = NULL;
        GError *error = NULL;
        gint port = -1;

        sock = g_socket_new(G_SOCKET_FAMILY_IPV4,
                            G_SOCKET_TYPE_STREAM,
                            G_SOCKET_PROTOCOL_TCP,
                            &error);
        if (sock == NULL)
        {
            g_clear_error(&error);
            continue;
        }

        addr = g_inet_address_new_from_string("127.0.0.1");
        sockaddr = g_inet_socket_address_new(addr, 0);
        g_object_unref(addr);

        if (!g_socket_bind(sock, sockaddr, FALSE, &error))
        {
            g_clear_error(&error);
            g_object_unref(sockaddr);
            g_object_unref(sock);
            continue;
        }

        local = g_socket_get_local_address(sock, &error);
        if (local != NULL && G_IS_INET_SOCKET_ADDRESS(local))
        {
            port = (gint) g_inet_socket_address_get_port(
                G_INET_SOCKET_ADDRESS(local));
        }
        g_clear_error(&error);
        g_clear_object(&local);
        g_object_unref(sockaddr);
        g_socket_close(sock, NULL);
        g_object_unref(sock);

        if (port > 0 && port != avoid_port)
        {
            selected = port;
            break;
        }
    }

    return selected;
}

static void rigctld_set_blocking(gint fd, gboolean blocking)
{
#ifndef WIN32
    gint flags = fcntl(fd, F_GETFL, 0);

    if (flags >= 0)
    {
        if (blocking)
            fcntl(fd, F_SETFL, flags & ~O_NONBLOCK);
        else
            fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    }
#else
    u_long mode = blocking ? 0 : 1;

    ioctlsocket((SOCKET)fd, FIONBIO, &mode);
#endif
}

static gboolean rigctld_connect_addrinfo_timeout(const gchar *host, gint port,
                                                 gint timeout_ms, gint *sock,
                                                 gint *err_out,
                                                 gint *so_err_out)
{
    struct addrinfo hints;
    struct addrinfo *res = NULL;
    struct addrinfo *ai;
    gchar            portstr[16];
    gboolean         connected = FALSE;
    gint             status;
    gint             last_err = 0;
    gint             last_so_err = 0;

    if (host == NULL || sock == NULL)
        return FALSE;

#ifdef G_OS_WIN32
    if (!winsock_ensure_init())
    {
        g_warning("%s: WSAStartup failed", __func__);
        return FALSE;
    }
#endif

    if (net_init() != 0)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: network init failed"), __func__);
        return FALSE;
    }

    if (err_out)
        *err_out = 0;
    if (so_err_out)
        *so_err_out = 0;

    *sock = -1;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_ADDRCONFIG;

    g_snprintf(portstr, sizeof(portstr), "%d", port);
    status = getaddrinfo(host, portstr, &hints, &res);
    if (status != 0)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: getaddrinfo failed for %s:%d (%s)"),
                    __func__, host, port, gai_strerror(status));
        return FALSE;
    }

    for (ai = res; ai != NULL; ai = ai->ai_next)
    {
        gchar addrbuf[NI_MAXHOST];
        gint fd = -1;
        gint err = 0;
        gint so_err = 0;
        gint connect_rc = -1;
        fd_set wset;
        struct timeval tv;
        gint sel = 0;
        socklen_t so_len = sizeof(so_err);

        if (getnameinfo(ai->ai_addr, ai->ai_addrlen,
                        addrbuf, sizeof(addrbuf), NULL, 0,
                        NI_NUMERICHOST) != 0)
            g_strlcpy(addrbuf, host, sizeof(addrbuf));

        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    _("%s: attempting connect to %s:%d (%s)"),
                    __func__, host, port, addrbuf);

        fd = (gint) socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0)
            continue;

        rigctld_set_blocking(fd, FALSE);

        connect_rc = connect(fd, ai->ai_addr, (socklen_t) ai->ai_addrlen);
        if (connect_rc == 0)
        {
            connected = TRUE;
        }
        else
        {
#ifdef WIN32
            err = WSAGetLastError();
            if (err == WSAEWOULDBLOCK || err == WSAEINPROGRESS ||
                err == WSAEALREADY)
#else
            err = errno;
            if (err == EINPROGRESS || err == EALREADY)
#endif
            {
                FD_ZERO(&wset);
                FD_SET(fd, &wset);
                tv.tv_sec = timeout_ms / 1000;
                tv.tv_usec = (timeout_ms % 1000) * 1000;
#ifdef WIN32
                sel = select(0, NULL, &wset, NULL, &tv);
#else
                sel = select(fd + 1, NULL, &wset, NULL, &tv);
#endif
                if (sel > 0)
                {
                    if (getsockopt(fd, SOL_SOCKET, SO_ERROR,
#ifdef WIN32
                                   (char *)&so_err, &so_len
#else
                                   &so_err, &so_len
#endif
                        ) == 0 && so_err == 0)
                    {
                        connected = TRUE;
                    }
                    else
                    {
                        err = so_err;
                    }
                }
            }
        }

        if (connected)
        {
            rigctld_set_blocking(fd, TRUE);
            *sock = fd;
            last_err = 0;
            last_so_err = 0;
            break;
        }

        if (err != 0)
            last_err = err;
        if (so_err != 0)
            last_so_err = so_err;
        rigctld_close_fd(fd);
    }

    freeaddrinfo(res);

    if (err_out)
        *err_out = last_err;
    if (so_err_out)
        *so_err_out = last_so_err;

    return connected;
}

static gboolean rigctld_connect_addrinfo(const gchar *host, gint port,
                                         gint *sock)
{
    struct addrinfo hints;
    struct addrinfo *res = NULL;
    struct addrinfo *ai;
    gchar            portstr[16];
    gboolean         connected = FALSE;
    gint             status;

    if (host == NULL || sock == NULL)
        return FALSE;

#ifdef G_OS_WIN32
    if (!winsock_ensure_init())
    {
        g_warning("%s: WSAStartup failed", __func__);
        return FALSE;
    }
#endif

    if (net_init() != 0)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: network init failed"), __func__);
        return FALSE;
    }

    *sock = -1;
    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_ADDRCONFIG;

    g_snprintf(portstr, sizeof(portstr), "%d", port);
    status = getaddrinfo(host, portstr, &hints, &res);
    if (status != 0)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: getaddrinfo failed for %s:%d (%s)"),
                    __func__, host, port, gai_strerror(status));
        return FALSE;
    }

    for (ai = res; ai != NULL; ai = ai->ai_next)
    {
        gchar addrbuf[NI_MAXHOST];

        if (getnameinfo(ai->ai_addr, ai->ai_addrlen,
                        addrbuf, sizeof(addrbuf), NULL, 0,
                        NI_NUMERICHOST) != 0)
            g_strlcpy(addrbuf, host, sizeof(addrbuf));

        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    _("%s: attempting connect to %s:%d (%s)"),
                    __func__, host, port, addrbuf);

        *sock = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (*sock < 0)
        {
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        _("%s: socket() failed for %s:%d (%s)"),
                        __func__, host, port, addrbuf);
            continue;
        }

        rigctld_apply_socket_timeouts(*sock);
        if (connect(*sock, ai->ai_addr, ai->ai_addrlen) == 0)
        {
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        _("%s: Connected to %s:%d (%s)"),
                        __func__, host, port, addrbuf);
            connected = TRUE;
            break;
        }

        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    _("%s: connect failed to %s:%d (%s): %s"),
                    __func__, host, port, addrbuf, g_strerror(errno));
        rigctld_close_fd(*sock);
        *sock = -1;
    }

    freeaddrinfo(res);
    return connected;
}

static gboolean open_rigctld_socket_host(const gchar *host, gint port,
                                         gint *sock,
                                         gint *err_out,
                                         gint *so_err_out,
                                         gboolean log_fail)
{
    const gchar    *target = host;
    gint            err = 0;
    gint            so_err = 0;

    if (host == NULL || sock == NULL)
        return FALSE;

    if (err_out)
        *err_out = 0;
    if (so_err_out)
        *so_err_out = 0;

    if (port <= 0)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: Missing rigctld host/port"), __func__);
        return FALSE;
    }

    if (g_ascii_strcasecmp(host, "localhost") == 0)
        target = "127.0.0.1";

    if (rigctld_connect_addrinfo_timeout(target, port,
                                         RIGCTLD_SOCKET_TIMEOUT_MS,
                                         sock, &err, &so_err))
    {
        rigctld_apply_socket_timeouts(*sock);
        if (err_out)
            *err_out = 0;
        if (so_err_out)
            *so_err_out = 0;
        return TRUE;
    }

    if (log_fail)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: Failed to connect to %s:%d (errno=%d so_error=%d)"),
                    __func__, target, port, err, so_err);
    }
    if (err_out)
        *err_out = err;
    if (so_err_out)
        *so_err_out = so_err;
    return FALSE;
}

static gboolean rigctld_try_connect_once(const gchar *host, gint port,
                                         gint timeout_ms, gint *last_err)
{
    struct addrinfo hints;
    struct addrinfo *res = NULL;
    struct addrinfo *ai;
    gchar            portstr[16];
    gboolean         connected = FALSE;
    gint             status;

    if (host == NULL)
        return FALSE;

#ifdef G_OS_WIN32
    if (!winsock_ensure_init())
    {
        g_warning("%s: WSAStartup failed", __func__);
        return FALSE;
    }
#endif

    if (last_err)
        *last_err = 0;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_ADDRCONFIG;

    g_snprintf(portstr, sizeof(portstr), "%d", port);
    status = getaddrinfo(host, portstr, &hints, &res);
    if (status != 0)
        return FALSE;

    for (ai = res; ai != NULL; ai = ai->ai_next)
    {
        gint fd = -1;
        gint err = 0;
        gint connect_rc = -1;
        fd_set wset;
        struct timeval tv;
        gint sel = 0;
        gint so_err = 0;
        socklen_t so_len = sizeof(so_err);

        fd = (gint) socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (fd < 0)
            continue;

        rigctld_set_blocking(fd, FALSE);

        connect_rc = connect(fd, ai->ai_addr, (socklen_t) ai->ai_addrlen);
        if (connect_rc == 0)
        {
            connected = TRUE;
        }
        else
        {
#ifdef WIN32
            err = WSAGetLastError();
            if (err == WSAEWOULDBLOCK || err == WSAEINPROGRESS ||
                err == WSAEALREADY)
#else
            err = errno;
            if (err == EINPROGRESS || err == EALREADY)
#endif
            {
                FD_ZERO(&wset);
                FD_SET(fd, &wset);
                tv.tv_sec = timeout_ms / 1000;
                tv.tv_usec = (timeout_ms % 1000) * 1000;
#ifdef WIN32
                sel = select(0, NULL, &wset, NULL, &tv);
#else
                sel = select(fd + 1, NULL, &wset, NULL, &tv);
#endif
                if (sel > 0)
                {
                    if (getsockopt(fd, SOL_SOCKET, SO_ERROR,
#ifdef WIN32
                                   (char *)&so_err, &so_len
#else
                                   &so_err, &so_len
#endif
                        ) == 0 && so_err == 0)
                    {
                        connected = TRUE;
                    }
                    else
                    {
                        err = so_err;
                    }
                }
            }
        }

#ifndef WIN32
        close(fd);
#else
        closesocket((SOCKET)fd);
#endif

        if (connected)
            break;

        if (last_err)
            *last_err = err;
    }

    freeaddrinfo(res);
    return connected;
}

static gboolean rigctld_wait_tcp_listen(GtkRigCtrl *ctrl,
                                        RigctldMgr *mgr,
                                        const gchar *host,
                                        gint port,
                                        gint timeout_ms,
                                        gint poll_ms,
                                        gint *waited_ms_out,
                                        gchar **exit_detail_out)
{
    gint64 start_us = g_get_monotonic_time();
    gint waited_ms = 0;
    gint last_err = 0;

    if (waited_ms_out)
        *waited_ms_out = 0;
    if (exit_detail_out)
        *exit_detail_out = NULL;

    while (waited_ms < timeout_ms)
    {
        gint64 attempt_start = g_get_monotonic_time();

        if (mgr && !rigctld_mgr_is_running(mgr))
        {
            gboolean exited = FALSE;
            gint status = -1;
            gint signal = 0;

            rigctld_mgr_get_exit_info(mgr, &exited, &status, &signal);
            if (exit_detail_out)
            {
                if (signal > 0)
                    *exit_detail_out =
                        g_strdup_printf("rigctld exited via signal %d", signal);
                else if (status >= 0)
                    *exit_detail_out =
                        g_strdup_printf("rigctld exit status %d", status);
                else
                    *exit_detail_out = g_strdup("rigctld exited");
            }
            if (waited_ms_out)
                *waited_ms_out = waited_ms;
            return FALSE;
        }

        if (rigctld_try_connect_once(host, port, poll_ms, &last_err))
        {
            if (waited_ms_out)
                *waited_ms_out = waited_ms;
            return TRUE;
        }

        waited_ms = (gint) ((g_get_monotonic_time() - start_us) / 1000);
        if (waited_ms >= timeout_ms)
            break;

        {
            gint attempt_ms = (gint) ((g_get_monotonic_time() - attempt_start) / 1000);
            gint sleep_ms = poll_ms - attempt_ms;

            if (sleep_ms > 0)
                g_usleep((gulong) sleep_ms * 1000);
        }
    }

    if (waited_ms_out)
        *waited_ms_out = waited_ms;
    (void)last_err;
    (void)ctrl;
    return FALSE;
}

static gchar *rigctld_read_log_tail(const gchar *path, guint max_lines)
{
    gchar *contents = NULL;
    gsize  length = 0;
    gchar **lines = NULL;
    gint total = 0;
    gint start = 0;
    GString *out = NULL;

    if (path == NULL || *path == '\0')
        return NULL;

    if (!g_file_get_contents(path, &contents, &length, NULL))
        return NULL;

    lines = g_strsplit(contents, "\n", -1);
    for (total = 0; lines[total] != NULL; total++)
        ;

    if ((gint) max_lines < total)
        start = total - (gint) max_lines;

    out = g_string_new(NULL);
    for (gint i = start; lines[i] != NULL; i++)
    {
        g_string_append(out, lines[i]);
        if (lines[i + 1] != NULL)
            g_string_append(out, "\n");
    }

    g_strfreev(lines);
    g_free(contents);
    return g_string_free(out, FALSE);
}

static gint rigctld_expected_model(const radio_conf_t *conf)
{
    gint model = 0;

    if (conf == NULL)
        return 0;

    model = conf->rigctld_model;
    if (model <= 0)
        model = radio_model_to_hamlib_model(conf->radio_model);

    return model;
}

static gboolean rigctld_is_ic905(const radio_conf_t *conf)
{
    return rigctld_expected_model(conf) == RIGCTLD_MODEL_IC905;
}

static void rigctld_extract_first_lines(const gchar *text,
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

static gboolean rig_session_vfo_candidate_exists(const RigSession *session,
                                                 const gchar *token)
{
    if (session == NULL || token == NULL)
        return FALSE;

    for (guint i = 0; i < session->vfo_candidates->len; i++)
    {
        const gchar *entry = g_ptr_array_index(session->vfo_candidates, i);
        if (entry != NULL && g_ascii_strcasecmp(entry, token) == 0)
            return TRUE;
    }

    return FALSE;
}

static gboolean rig_session_probe_and_configure(GtkRigCtrl *ctrl,
                                                RigSession *session,
                                                gint sock,
                                                const radio_conf_t *conf)
{
    RigctldClient *client = rigctld_client_for_socket(ctrl, sock);
    const RigCaps *caps = NULL;
    gint64 freq_probe = 0;
    gboolean freq_ok = FALSE;
    gboolean vfo_select_ok = FALSE;
    gboolean vfo_opt_args_ok = FALSE;

    if (ctrl == NULL || session == NULL || conf == NULL)
        return FALSE;

    if (client == NULL)
    {
        rig_session_set_state(ctrl, session, RIG_SESSION_DEGRADED,
                              "missing rigctld client");
        return FALSE;
    }

    rig_session_set_state(ctrl, session, RIG_SESSION_PROBING,
                          "probe start");

    if (!rigctld_client_probe(client, conf, 500))
    {
        const gchar *reason = rigctld_client_get_state_reason(client);
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "%s: rigctld probe failed (%s); attempting minimal fallback",
                    __func__,
                    reason ? reason : "unknown");

        if (rigctld_client_get_freq(client, VFO_MAIN, &freq_probe))
        {
            rig_session_set_state(ctrl, session, RIG_SESSION_READY,
                                  "probe failed; fallback plain freq");
            session->strategy = RIG_STRATEGY_PLAIN_FREQ;
            session->strategy_logged = FALSE;
            session->rig_model = rigctld_expected_model(conf);
            if (rigctrl_reject_main_sub_without_vfo_strategy(
                    ctrl, session, conf,
                    "probe failed; Main/Sub requires rigctld VFO support"))
                return FALSE;
            if (rigctrl_log_throttled(ctrl, &ctrl->last_probe_log_us,
                                      RIGCTRL_PROBE_LOG_INTERVAL_US))
                rig_term_log_verbose(ctrl, "gpredict",
                                     "rig session (%s) fallback READY freq=%" G_GINT64_FORMAT,
                                     session->label ? session->label : "rig",
                                     freq_probe);
            return TRUE;
        }

        rig_session_set_state(ctrl, session, RIG_SESSION_READY,
                              "probe failed; fallback default");
        session->strategy = RIG_STRATEGY_PLAIN_FREQ;
        session->strategy_logged = FALSE;
        session->rig_model = rigctld_expected_model(conf);
        if (rigctrl_reject_main_sub_without_vfo_strategy(
                ctrl, session, conf,
                "probe failed; Main/Sub requires rigctld VFO support"))
            return FALSE;
        if (rigctrl_log_throttled(ctrl, &ctrl->last_probe_log_us,
                                  RIGCTRL_PROBE_LOG_INTERVAL_US))
            rig_term_log_verbose(ctrl, "gpredict",
                                 "rig session (%s) fallback READY (no probe)",
                                 session->label ? session->label : "rig");
        return TRUE;
    }

    caps = rigctld_client_get_caps(client);
    if (caps == NULL)
    {
        rig_session_set_state(ctrl, session, RIG_SESSION_READY,
                              "caps missing; fallback plain freq");
        session->strategy = RIG_STRATEGY_PLAIN_FREQ;
        session->strategy_logged = FALSE;
        session->rig_model = rigctld_expected_model(conf);
        return TRUE;
    }

    rig_session_apply_caps(session, caps);

    if (rigctrl_reject_main_sub_without_vfo_strategy(
            ctrl, session, conf,
            "probe succeeded without a usable Main/Sub VFO strategy"))
        return FALSE;

    rig_term_log(ctrl, "gpredict",
                 "rig session (%s) dump_state model=%d backend=%s signature=%s vfo_candidates=%u",
                 session->label ? session->label : "rig",
                 session->rig_model,
                 session->backend_version ? session->backend_version : "(unknown)",
                 session->signature ? session->signature : "(unknown)",
                 session->vfo_candidates ? session->vfo_candidates->len : 0);
    if (rigctrl_log_at_least(ctrl, RIG_LOG_VERBOSE) &&
        session->vfo_candidates != NULL &&
        session->vfo_candidates->len > 0)
    {
        GString *list = g_string_new(NULL);

        for (guint i = 0; i < session->vfo_candidates->len; i++)
        {
            const gchar *entry = g_ptr_array_index(session->vfo_candidates, i);
            if (entry == NULL || *entry == '\0')
                continue;
            if (list->len > 0)
                g_string_append(list, ", ");
            g_string_append(list, entry);
        }

        if (list->len > 0)
            rig_term_log_verbose(ctrl, "gpredict",
                                 "rig session (%s) vfo_candidates=%s",
                                 session->label ? session->label : "rig",
                                 list->str);
        g_string_free(list, TRUE);
    }

    freq_ok = caps->has_get_freq;
    vfo_opt_args_ok = (session->strategy == RIG_STRATEGY_VFO_OPT_ARGS);
    vfo_select_ok = (session->strategy == RIG_STRATEGY_SELECT_VFO ||
                     session->strategy == RIG_STRATEGY_VFO_OPT_ARGS);

    if (!session->strategy_logged)
    {
        rig_term_log(ctrl, "gpredict",
                     "rig session (%s) strategy=%s freq_ok=%d vfo_select_ok=%d vfo_opt_args_ok=%d",
                     session->label ? session->label : "rig",
                     rig_strategy_name(session->strategy),
                     freq_ok ? 1 : 0,
                     vfo_select_ok ? 1 : 0,
                     vfo_opt_args_ok ? 1 : 0);
        session->strategy_logged = TRUE;
    }

    rig_session_set_state(ctrl, session, RIG_SESSION_READY,
                          "strategy=%s", rig_strategy_name(session->strategy));

    return TRUE;
}

static gboolean close_rigctld_socket(GtkRigCtrl *ctrl, gint * sock,
                                     gboolean send_quit)
{
    RigctldClient *client = NULL;
    HamlibTransport *transport = NULL;
    gchar reply[64];

    if (sock == NULL || *sock == -1)
        return TRUE;

    client = rigctld_client_for_socket(ctrl, *sock);
    transport = client ? rigctld_client_get_transport(client) : NULL;
    if (transport != NULL)
        (void)hamlib_transport_drain(transport, RIGCTLD_DUMP_STATE_IDLE_MS,
                                     NULL);

    rigctld_clear_rxbuf_for_socket(ctrl, *sock);
    rigctld_clear_vfo_map_for_socket(ctrl, *sock);
    rigctld_reset_offset_state_for_socket(ctrl, *sock);
    rig_session_reset(rig_session_for_socket(ctrl, *sock));

    if (send_quit && client != NULL)
        (void)rigctld_client_request_raw(client, "q\n",
                                         reply, sizeof(reply), NULL);

    if (client != NULL)
        rigctld_client_close(client);
    else
        rigctld_close_fd(*sock);

    *sock = -1;
    return TRUE;
}

static rigctld_probe_result_t rigctld_probe_simple(const gchar *host, gint port,
                                                   gint timeout_ms,
                                                   gint expected_model,
                                                   gint *model_out,
                                                   gchar **reply_out)
{
    gint  sock = -1;
    gchar buffer[1024];
    GString *rxbuf = NULL;
    const gchar *cmd = "\\dump_state\n";
    gint  size;
    gchar *line1 = NULL;
    gchar *line2 = NULL;
    gint model = 0;
    gboolean parsed = FALSE;
    gint err = 0;

    if (reply_out)
        *reply_out = NULL;
    if (model_out)
        *model_out = 0;

    rigctld_io_lock_acquire();

    if (!rigctld_connect_addrinfo(host, port, &sock))
    {
        rigctld_io_lock_release();
        return RIGCTLD_PROBE_NOT_READY;
    }

    rigctld_apply_socket_timeouts_ms(sock, timeout_ms);

    size = (gint) strlen(cmd);
    if (send(sock, cmd, size, 0) != size)
    {
        rigctld_close_fd(sock);
        rigctld_io_lock_release();
        return RIGCTLD_PROBE_NOT_READY;
    }

    rxbuf = rigctld_rxbuf_new(256);
    size = (gint) rigctld_read_response(sock, rxbuf,
                                        RIGCTLD_READ_MULTILINE_IDLE,
                                        buffer, sizeof(buffer),
                                        timeout_ms,
                                        RIGCTLD_DUMP_STATE_IDLE_MS,
                                        NULL, NULL, &err);
    rigctld_rxbuf_free(&rxbuf);
    if (size <= 0)
    {
        rigctld_close_fd(sock);
        rigctld_io_lock_release();
        return RIGCTLD_PROBE_NOT_READY;
    }

    buffer[size] = '\0';
    if (reply_out)
        *reply_out = g_strdup(buffer);

    rigctld_extract_first_lines(buffer, &line1, &line2);
    parsed = parse_dump_state_model_id(buffer, &model);
    if (rigctld_client_get_log_level() >= RIG_LOG_VERBOSE)
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rigctld dump_state: line1=%s line2=%s model=%d",
                    line1 ? line1 : "(none)",
                    line2 ? line2 : "(none)",
                    model);

    if (!parsed)
    {
        g_free(line1);
        g_free(line2);
        rigctld_close_fd(sock);
        rigctld_io_lock_release();
        return RIGCTLD_PROBE_NOT_READY;
    }

    if (model_out)
        *model_out = model;

    if (expected_model > 0 && model > 0 && model != expected_model)
    {
        sat_log_log(SAT_LOG_LEVEL_WARN,
                    "rigctld model mismatch expected=%d got=%d line1=%s line2=%s",
                    expected_model, model,
                    line1 ? line1 : "(none)",
                    line2 ? line2 : "(none)");
        g_free(line1);
        g_free(line2);
        rigctld_close_fd(sock);
        rigctld_io_lock_release();
        return RIGCTLD_PROBE_MISMATCH;
    }

    g_free(line1);
    g_free(line2);
    rigctld_close_fd(sock);
    rigctld_io_lock_release();

    return RIGCTLD_PROBE_OK;
}

static rigctld_probe_result_t
rigctld_wait_for_ready(const gchar *host,
                       gint port,
                       gint timeout_ms,
                       gint expected_model,
                       gint *model_out,
                       gchar **reply_out)
{
    gint64 deadline_us = g_get_monotonic_time() +
                         ((gint64) MAX(timeout_ms, 0) * 1000);
    const gint interval_ms = 200;
    rigctld_probe_result_t result = RIGCTLD_PROBE_NOT_READY;
    gchar *last_reply = NULL;

    if (reply_out)
        *reply_out = NULL;
    if (model_out)
        *model_out = 0;

    while (g_get_monotonic_time() < deadline_us)
    {
        gint64 now_us = g_get_monotonic_time();
        gint remaining_ms = (gint) MAX((deadline_us - now_us) / 1000, 1);
        gint probe_timeout_ms = MIN(RIGCTLD_PROBE_SHORT_MS, remaining_ms);

        g_free(last_reply);
        last_reply = NULL;
        result = rigctld_probe_simple(host, port,
                                      probe_timeout_ms,
                                      expected_model,
                                      model_out,
                                      &last_reply);
        if (result != RIGCTLD_PROBE_NOT_READY)
        {
            if (reply_out)
                *reply_out = last_reply;
            else
                g_free(last_reply);
            return result;
        }

        now_us = g_get_monotonic_time();
        if (now_us < deadline_us)
        {
            gint sleep_ms =
                MIN(interval_ms, (gint) MAX((deadline_us - now_us) / 1000, 0));

            if (sleep_ms > 0)
                g_usleep((gulong) sleep_ms * 1000);
        }
    }

    if (reply_out)
        *reply_out = last_reply;
    else
        g_free(last_reply);

    return result;
}

static gboolean rigctld_autodetect_device(GtkRigCtrl *ctrl,
                                          radio_conf_t *conf,
                                          const gchar *role,
                                          gboolean *error_reported)
{
    GSList *candidates = NULL;
    GSList *item = NULL;
    gint64  start_us;
    gint64  deadline_us;
    gint    tried = 0;
    gboolean success = FALSE;
    gchar  *host = NULL;
    gchar  *detail = NULL;
    gchar  *fatal_err = NULL;
    gchar  *fallback_candidate = NULL;
    gchar  *allowlist_lc = NULL;
    guint   filtered = 0;
    guint   candidate_count = 0;
    rigctld_preset_defaults_t preset;
    const gchar *cached = NULL;
    gboolean is_ic905 = rigctld_is_ic905(conf);

    if (error_reported)
        *error_reported = FALSE;

    if (conf == NULL)
        return FALSE;

    if (!radio_model_get_rigctld_defaults(conf->radio_model, &preset))
        return FALSE;

    if (conf->rigctld_device != NULL && *conf->rigctld_device != '\0')
        return TRUE;

    if (conf->rigctld_baud <= 0)
        conf->rigctld_baud = preset.baud;

    if (conf->rigctld_conn != RIGCTLD_CONN_SERIAL)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: forcing serial rigctld connection for preset auto-detect"),
                    __func__);
        conf->rigctld_conn = RIGCTLD_CONN_SERIAL;
    }

    if ((conf->rigctld_civaddr == NULL || *conf->rigctld_civaddr == '\0') &&
        preset.civaddr != NULL && *preset.civaddr != '\0' &&
        !is_ic905)
    {
        g_free(conf->rigctld_civaddr);
        conf->rigctld_civaddr = g_strdup(preset.civaddr);
    }

    host = rigctld_mgr_normalize_host(conf->host);
    if (host == NULL)
        host = g_strdup("127.0.0.1");

    if (conf->rigctld_autodetect_match &&
        *conf->rigctld_autodetect_match)
    {
        allowlist_lc = g_ascii_strdown(conf->rigctld_autodetect_match, -1);
    }

    candidates = gp_serial_list_candidates();
    candidates = rigctld_autodetect_filter_candidates(candidates,
                                                      allowlist_lc,
                                                      &filtered);
    candidate_count = g_slist_length(candidates);
    {
        gint total_budget_ms = (gint) MIN(
            (guint) RIGCTLD_AUTODETECT_TOTAL_MS_MAX,
            MAX((guint) RIGCTLD_AUTODETECT_TOTAL_MS,
                candidate_count *
                    (guint) RIGCTLD_AUTODETECT_TOTAL_MS_PER_CANDIDATE));
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: auto-detect candidates=%u filtered=%u total_budget_ms=%d allowlist=%s"),
                    __func__, candidate_count, filtered, total_budget_ms,
                    (conf->rigctld_autodetect_match &&
                     *conf->rigctld_autodetect_match) ?
                        conf->rigctld_autodetect_match : "(none)");
        rig_term_log(ctrl, "gpredict",
                     "auto-detect candidates=%u filtered=%u total_budget_ms=%d allowlist=%s",
                     candidate_count, filtered, total_budget_ms,
                     (conf->rigctld_autodetect_match &&
                      *conf->rigctld_autodetect_match) ?
                        conf->rigctld_autodetect_match : "(none)");
    }
    cached = rigctld_cached_device(conf->radio_model);
    if (cached != NULL)
    {
        if (rigctld_candidate_matches_allowlist(cached, allowlist_lc))
        {
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        _("%s: auto-detect preferring cached device %s"),
                        __func__, cached);
            candidates = rigctld_prioritize_candidate(candidates, cached);
        }
        else
        {
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        _("%s: cached device %s does not match allowlist"),
                        __func__, cached);
        }
    }

    if (candidates == NULL)
    {
        if (conf->rigctld_autodetect_match &&
            *conf->rigctld_autodetect_match)
        {
            detail = g_strdup_printf(
                _("No serial ports matched auto-detect allowlist (%s)."),
                conf->rigctld_autodetect_match);
        }
        else
        {
            detail = g_strdup(_("No serial ports found for auto-detect."));
        }
        schedule_rig_autodetect_error(ctrl, conf, detail);
        if (error_reported)
            *error_reported = TRUE;
        g_free(detail);
        g_free(allowlist_lc);
        g_free(host);
        return FALSE;
    }

    start_us = g_get_monotonic_time();
    deadline_us = start_us +
        ((gint64) MIN((guint) RIGCTLD_AUTODETECT_TOTAL_MS_MAX,
                      MAX((guint) RIGCTLD_AUTODETECT_TOTAL_MS,
                          candidate_count *
                              (guint) RIGCTLD_AUTODETECT_TOTAL_MS_PER_CANDIDATE)) *
         1000);

    gint expected_model = rigctld_expected_model(conf);

    for (item = candidates; item != NULL; item = item->next)
    {
        gchar *candidate = item->data;
        RigctldMgr *probe_mgr = NULL;
        gchar *errmsg = NULL;
        gchar *reply = NULL;
        gchar *cmdline = NULL;
        gchar *log_path = NULL;
        gchar *log_tail = NULL;
        gchar *exit_detail = NULL;
        gint temp_port = -1;
        gint detected_model = 0;
        gint waited_ms = 0;
        radio_conf_t probe_conf;

        if (tried >= RIGCTLD_AUTODETECT_MAX_CANDIDATES)
            break;

        if (g_get_monotonic_time() >= deadline_us)
            break;

        if (candidate == NULL || *candidate == '\0')
            continue;

        tried++;
        temp_port = rigctld_pick_ephemeral_port(conf->port);
        if (temp_port <= 0)
        {
            sat_log_log(SAT_LOG_LEVEL_WARN,
                        _("%s: auto-detect failed to reserve temp port for %s"),
                        __func__, candidate);
            rig_term_log(ctrl, "gpredict:err",
                         "auto-detect failed to reserve temp port for %s",
                         candidate);
            continue;
        }
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: auto-detect probing %s temp_port=%d"),
                    __func__, candidate, temp_port);
        rig_term_log(ctrl, "gpredict",
                     "auto-detect probing %s temp_port=%d",
                     candidate, temp_port);

        probe_conf = *conf;
        probe_conf.rigctld_device = candidate;
        probe_conf.port = temp_port;

        {
            GError *tmp_err = NULL;
            gint log_fd = g_file_open_tmp("gpredict-rigctld-autodetect-XXXXXX.log",
                                          &log_path, &tmp_err);
            if (log_fd >= 0)
                g_close(log_fd, NULL);
            else
            {
                sat_log_log(SAT_LOG_LEVEL_WARN,
                            _("%s: auto-detect log file create failed (%s)"),
                            __func__, tmp_err ? tmp_err->message : "unknown");
                g_clear_error(&tmp_err);
                g_free(log_path);
                log_path = NULL;
            }
        }

        probe_mgr = rigctld_mgr_spawn(&probe_conf, host, &errmsg, &cmdline,
                                      log_path);
        if (cmdline != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        _("%s: auto-detect spawn cmd: %s"),
                        __func__, cmdline);
            rig_term_log(ctrl, "gpredict",
                         "auto-detect spawn cmd: %s", cmdline);
        }
        if (probe_mgr == NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: auto-detect spawn failed for %s (%s)"),
                        __func__, candidate,
                        errmsg ? errmsg : "unknown");
            rig_term_log(ctrl, "gpredict:err",
                         "auto-detect spawn failed for %s (%s)",
                         candidate, errmsg ? errmsg : "unknown");
            if (errmsg && *errmsg)
                fatal_err = g_strdup(errmsg);
            g_free(errmsg);
            g_free(cmdline);
            if (log_path)
            {
                g_unlink(log_path);
                g_free(log_path);
            }
            break;
        }

        if (!rigctld_wait_tcp_listen(ctrl, probe_mgr, host, temp_port,
                                     RIGCTLD_STARTUP_TIMEOUT_MS,
                                     RIGCTLD_STARTUP_POLL_MS,
                                     &waited_ms, &exit_detail))
        {
            if (exit_detail != NULL)
            {
                sat_log_log(SAT_LOG_LEVEL_ERROR,
                            _("%s: auto-detect rigctld exited early for %s (%s)"),
                            __func__, candidate, exit_detail);
                rig_term_log(ctrl, "gpredict:err",
                             "auto-detect rigctld exited early for %s (%s)",
                             candidate, exit_detail);
            }
            else
            {
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            _("%s: auto-detect no listen on %s:%d after %d ms"),
                            __func__, host, temp_port, waited_ms);
                rig_term_log(ctrl, "gpredict:err",
                             "auto-detect no listen on %s:%d after %d ms",
                             host, temp_port, waited_ms);
            }
            if (log_path)
                log_tail = rigctld_read_log_tail(log_path, 30);
            if (log_tail && *log_tail)
            {
                sat_log_log(SAT_LOG_LEVEL_INFO,
                            _("%s: auto-detect rigctld log tail for %s:\n%s"),
                            __func__, candidate, log_tail);
                rig_term_log(ctrl, "gpredict:err",
                             "auto-detect rigctld log tail for %s:\n%s",
                             candidate, log_tail);
            }
            g_free(log_tail);
            g_free(exit_detail);
            rigctld_mgr_terminate(&probe_mgr);
            g_free(cmdline);
            if (log_path)
            {
                g_unlink(log_path);
                g_free(log_path);
            }
            continue;
        }

        {
            rigctld_probe_result_t probe =
                rigctld_wait_for_ready(host, temp_port,
                                       RIGCTLD_AUTODETECT_WAIT_MS,
                                       expected_model,
                                       &detected_model,
                                       &reply);
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        _("%s: auto-detect probe result=%s candidate=%s "
                          "port=%d model=%d reply=%s"),
                        __func__, rigctld_probe_result_name(probe),
                        candidate, temp_port, detected_model,
                        reply ? reply : "(none)");
            rig_term_log(ctrl, "gpredict",
                         "auto-detect probe result=%s candidate=%s port=%d model=%d",
                         rigctld_probe_result_name(probe),
                         candidate, temp_port, detected_model);
            if (probe == RIGCTLD_PROBE_MISMATCH)
            {
                sat_log_log(SAT_LOG_LEVEL_WARN,
                            _("%s: auto-detect model mismatch for %s "
                              "(expected=%d got=%d)"),
                            __func__, candidate, expected_model,
                            detected_model);
            }

            if (probe == RIGCTLD_PROBE_OK)
            {
                sat_log_log(SAT_LOG_LEVEL_INFO,
                            _("%s: auto-detect succeeded for %s "
                              "(model=%d reply: %s)"),
                            __func__, candidate,
                            detected_model,
                            reply ? reply : "(none)");
                rig_term_log(ctrl, "gpredict",
                             "auto-detect selected %s model=%d",
                             candidate, detected_model);
                g_free(conf->rigctld_device);
                conf->rigctld_device = g_strdup(candidate);
                rigctld_cache_device(conf->radio_model, candidate);
                success = TRUE;
                g_free(reply);
                rigctld_mgr_terminate(&probe_mgr);
                g_free(cmdline);
                if (log_path)
                {
                    g_unlink(log_path);
                    g_free(log_path);
                }
                break;
            }

            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        _("%s: auto-detect probe failed for %s (result=%s reply: %s)"),
                        __func__, candidate,
                        rigctld_probe_result_name(probe),
                        reply ? reply : "(none)");
            if (probe == RIGCTLD_PROBE_NOT_READY)
            {
                rig_term_log(ctrl, "gpredict:err",
                             "auto-detect probe timeout for %s on %s:%d",
                             candidate, host, temp_port);
                if (log_path)
                    log_tail = rigctld_read_log_tail(log_path, 30);
                if (log_tail && *log_tail)
                {
                    sat_log_log(SAT_LOG_LEVEL_INFO,
                                _("%s: auto-detect rigctld log tail for %s:\n%s"),
                                __func__, candidate, log_tail);
                    rig_term_log(ctrl, "gpredict:err",
                                 "auto-detect rigctld log tail for %s:\n%s",
                                 candidate, log_tail);
                }
                if (fallback_candidate == NULL &&
                    !rigctld_log_tail_indicates_stale_device(log_tail))
                {
                    fallback_candidate = g_strdup(candidate);
                    sat_log_log(SAT_LOG_LEVEL_INFO,
                                _("%s: auto-detect keeping listening fallback %s"),
                                __func__, candidate);
                    rig_term_log(ctrl, "gpredict",
                                 "auto-detect keeping listening fallback %s",
                                 candidate);
                }
                g_free(log_tail);
                log_tail = NULL;
            }
            else if (probe == RIGCTLD_PROBE_MISMATCH)
            {
                rig_term_log(ctrl, "gpredict:err",
                             "auto-detect model mismatch expected=%d got=%d for %s",
                             expected_model, detected_model, candidate);
            }
            g_free(reply);
            rigctld_mgr_terminate(&probe_mgr);
        }
        g_free(cmdline);
        if (log_path)
        {
            g_unlink(log_path);
            g_free(log_path);
        }
    }

    gp_serial_free_candidates(candidates);
    g_free(allowlist_lc);
    g_free(host);

    if (success)
    {
        g_free(fallback_candidate);
        return TRUE;
    }

    if (fallback_candidate != NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: auto-detect selected listening fallback %s"),
                    __func__, fallback_candidate);
        rig_term_log(ctrl, "gpredict",
                     "auto-detect selected listening fallback %s",
                     fallback_candidate);
        g_free(conf->rigctld_device);
        conf->rigctld_device = fallback_candidate;
        rigctld_cache_device(conf->radio_model, fallback_candidate);
        g_free(fatal_err);
        return TRUE;
    }

    sat_log_log(SAT_LOG_LEVEL_ERROR,
                _("%s: auto-detect failed after %d candidate(s)"),
                __func__, tried);
    rig_term_log(ctrl, "gpredict:err",
                 "auto-detect failed after %d candidate(s)", tried);

    if (fatal_err != NULL && *fatal_err)
        detail = g_strdup_printf(_("Auto-detect failed: %s"), fatal_err);
    else
        detail = g_strdup_printf(
            _("No compatible radio found on available serial ports (%s)."),
            role ? role : _("rig"));

    schedule_rig_autodetect_error(ctrl, conf, detail);
    if (error_reported)
        *error_reported = TRUE;
    g_free(detail);
    g_free(fatal_err);
    return FALSE;
}

static void rigctrl_handle_socket_error(GtkRigCtrl *ctrl, gint sock,
                                        const gchar *context)
{
    const gchar *role = _("rig");
    gboolean     secondary = FALSE;
    RigSession  *session = NULL;
    rigctrl_conn_state_t state;

    if (ctrl == NULL)
        return;

    if (sock == ctrl->sock)
    {
        role = _("receiver");
        secondary = FALSE;
        session = ctrl->rig_session;
    }
    else if (sock == ctrl->sock2)
    {
        role = _("uplink");
        secondary = TRUE;
        session = ctrl->rig_session2;
    }

    if (session == NULL)
        return;

    state = secondary ? ctrl->conn_state2 : ctrl->conn_state;
    if (state == RIGCTRL_CONN_DISCONNECTING ||
        state == RIGCTRL_CONN_DISCONNECTED)
        return;

    sat_log_log(SAT_LOG_LEVEL_ERROR,
                _("%s: %s socket error during %s; disconnecting"),
                __func__, role, context ? context : "command");
    rig_term_log(ctrl, "gpredict:err",
                 "%s socket error during %s",
                 role ? role : "rig",
                 context ? context : "command");

    rig_session_set_state(ctrl, session, RIG_SESSION_DEGRADED,
                          "socket error during %s",
                          context ? context : "command");

    rigctrl_set_conn_state(ctrl, secondary, RIGCTRL_CONN_DISCONNECTING,
                           "socket error");

    if (rigctrl_on_main_thread(ctrl))
    {
        rigctrl_close_socket_internal(ctrl, secondary);
    }
    else
    {
        RigctrlCloseSocketInfo *info = g_new0(RigctrlCloseSocketInfo, 1);

        info->ctrl = g_object_ref(ctrl);
        info->secondary = secondary;
        g_idle_add(rigctrl_close_socket_idle, info);
    }

    rigctrl_cancel_open_task(ctrl);
    if (secondary)
        ctrl->opening2 = FALSE;
    else
        ctrl->opening = FALSE;

    rigctrl_schedule_reconnect(ctrl, secondary, role);
}


static gboolean ensure_rigctld_running(GtkRigCtrl *ctrl,
                                       radio_conf_t *conf,
                                       RigctldMgr **mgr,
                                       gboolean secondary,
                                       const gchar *role,
                                       gchar **connect_host,
                                       gboolean *error_reported,
                                       gboolean ic905_force_fallback,
                                       gboolean force_local_recovery)
{
    gchar          *host = NULL;
    gchar          *errmsg = NULL;
    gchar          *detail = NULL;
    gboolean        reported = FALSE;
    gboolean        ok = FALSE;
    gboolean        own_host = FALSE;
    gboolean        is_ic905 = FALSE;
    gboolean        ic905_fallback = ic905_force_fallback;

    if (error_reported)
        *error_reported = FALSE;

    if (connect_host)
        *connect_host = NULL;

    if (conf == NULL)
        return FALSE;

    if (conf->port <= 0 || conf->host == NULL || *conf->host == '\0')
    {
        rigctld_preset_defaults_t preset;

        if (radio_model_get_rigctld_defaults(conf->radio_model, &preset))
        {
            if (conf->host == NULL || *conf->host == '\0')
            {
                g_free(conf->host);
                conf->host = g_strdup(preset.host);
            }
            if (conf->port <= 0)
                conf->port = preset.port;
        }
    }

    {
        rigctld_preset_defaults_t preset;
        is_ic905 = rigctld_is_ic905(conf);

        if (radio_model_get_rigctld_defaults(conf->radio_model, &preset) &&
            (conf->rigctld_conn == RIGCTLD_CONN_SERIAL ||
             conf->rigctld_device == NULL || *conf->rigctld_device == '\0'))
        {
            if (conf->rigctld_baud <= 0)
                conf->rigctld_baud = preset.baud;
            if ((conf->rigctld_civaddr == NULL ||
                 *conf->rigctld_civaddr == '\0') &&
                preset.civaddr != NULL && *preset.civaddr != '\0' &&
                !is_ic905)
            {
                g_free(conf->rigctld_civaddr);
                conf->rigctld_civaddr = g_strdup(preset.civaddr);
            }
        }
    }

    host = rigctld_mgr_normalize_host(conf->host);
    if (connect_host)
        *connect_host = host;
    else
        own_host = TRUE;

    if (host == NULL || conf->port <= 0)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: Missing rigctld host/port"), __func__);
        rig_term_log(ctrl, "gpredict:err",
                     "missing rigctld host/port");
        goto out;
    }

    {
        gint expected_model = rigctld_expected_model(conf);
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "rigctld ensure: host=%s port=%d autostart=%d "
                    "force_local_recovery=%d conn=%d model=%d baud=%d "
                    "device=%s",
                    conf->host ? conf->host : "(null)",
                    conf->port,
                    conf->rigctld_autostart ? 1 : 0,
                    force_local_recovery ? 1 : 0,
                    conf->rigctld_conn,
                    expected_model,
                    conf->rigctld_baud,
                    conf->rigctld_device ? conf->rigctld_device : "(none)");
    }

    if (conf->rigctld_autostart &&
        conf->host &&
        g_ascii_strcasecmp(conf->host, "localhost") == 0)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: auto-start enabled; using 127.0.0.1 instead of localhost"),
                    __func__);
    }

    if (!conf->rigctld_autostart && !force_local_recovery)
    {
        ok = TRUE;
        goto out;
    }

    if (!rigctld_mgr_host_is_local(conf->host))
    {
        detail = g_strdup_printf(
            _("Auto-start is only supported for 127.0.0.1.\nHost: %s"),
            conf->host ? conf->host : _("(missing)"));
        schedule_rig_autostart_error(ctrl, conf, role, detail);
        rig_term_log(ctrl, "gpredict:err",
                     "auto-start only supported for 127.0.0.1 (host=%s)",
                     conf->host ? conf->host : "(missing)");
        g_free(detail);
        reported = TRUE;
        goto out;
    }

    if (mgr && *mgr != NULL && !rigctld_mgr_is_running(*mgr))
        rigctld_terminate_spawned(ctrl, secondary, mgr);

    if (conf->rigctld_device && *conf->rigctld_device &&
        radio_model_to_hamlib_model(conf->radio_model) > 0)
    {
        rigctld_cache_device(conf->radio_model, conf->rigctld_device);
    }

    /* Connection probing is handled by the session socket to avoid extra
     * short-lived connections during engage.
     */

    if (conf->rigctld_conn == RIGCTLD_CONN_SERIAL &&
        conf->rigctld_device && *conf->rigctld_device &&
        rigctld_mgr_host_is_local(conf->host))
    {
        if (!rigctld_serial_device_exists(conf->rigctld_device))
        {
            sat_log_log(SAT_LOG_LEVEL_WARN,
                        _("%s: configured serial device missing (%s), "
                          "falling back to autodetect"),
                        __func__, conf->rigctld_device);
            rig_term_log(ctrl, "gpredict:err",
                         "configured serial device missing (%s), "
                         "falling back to autodetect",
                         conf->rigctld_device);
            g_free(conf->rigctld_device);
            conf->rigctld_device = NULL;
            if (!rigctld_autodetect_device(ctrl, conf, role, &reported))
                goto out;
            rigctld_persist_device(ctrl, conf, "missing device");
        }
    }

    if ((conf->rigctld_autostart || force_local_recovery) &&
        conf->rigctld_conn == RIGCTLD_CONN_SERIAL &&
        (conf->rigctld_device == NULL || *conf->rigctld_device == '\0') &&
        radio_model_to_hamlib_model(conf->radio_model) > 0)
    {
        if (!rigctld_autodetect_device(ctrl, conf, role, &reported))
            goto out;
        rigctld_persist_device(ctrl, conf, "autodetect");
    }

    if (conf->rigctld_model <= 0 &&
        radio_model_to_hamlib_model(conf->radio_model) <= 0)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: auto-start blocked; missing Hamlib rig model (%s)"),
                    __func__, conf->name ? conf->name : "(unknown)");
        rig_term_log(ctrl, "gpredict:err",
                     "auto-start blocked; missing Hamlib rig model");
        schedule_rig_missing_model_dialog(ctrl, conf);
        reported = TRUE;
        goto out;
    }

    if (mgr && *mgr == NULL)
    {
        const radio_conf_t *spawn_conf = conf;
        radio_conf_t spawn_override;
        gchar *fallback_args = NULL;

        if (is_ic905 && ic905_fallback)
        {
            spawn_override = *conf;
            spawn_override.rigctld_civaddr = NULL;
            if (conf->rigctld_extra_args && *conf->rigctld_extra_args)
            {
                fallback_args = g_strdup_printf(
                    "%s -C timeout=%d",
                    conf->rigctld_extra_args,
                    RIGCTLD_IC905_FALLBACK_TIMEOUT_MS);
            }
            else
            {
                fallback_args = g_strdup_printf(
                    "-C timeout=%d",
                    RIGCTLD_IC905_FALLBACK_TIMEOUT_MS);
            }
            spawn_override.rigctld_extra_args = fallback_args;
            spawn_conf = &spawn_override;

            sat_log_log(SAT_LOG_LEVEL_INFO,
                        _("%s: IC-905 fallback spawn (no civaddr, timeout=%d)"),
                        __func__, RIGCTLD_IC905_FALLBACK_TIMEOUT_MS);
            rig_term_log(ctrl, "gpredict",
                         "IC-905 fallback spawn (no civaddr, timeout=%d)",
                         RIGCTLD_IC905_FALLBACK_TIMEOUT_MS);
        }

        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: rigctld not reachable; attempting auto-start for %s:%d"),
                    __func__, host, conf->port);
        rig_term_log(ctrl, "gpredict",
                     "auto-start rigctld for %s:%d", host, conf->port);
        *mgr = rigctld_mgr_spawn(spawn_conf, host, &errmsg, NULL, NULL);
        g_free(fallback_args);
        if (*mgr == NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: auto-start failed for %s:%d (%s)"),
                        __func__, host, conf->port,
                        errmsg ? errmsg : "unknown");
            rig_term_log(ctrl, "gpredict:err",
                         "auto-start failed for %s:%d (%s)",
                         host, conf->port,
                         errmsg ? errmsg : "unknown");
            if (errmsg && *errmsg)
                detail = g_strdup_printf(
                    _("Failed to spawn rigctld for %s:%d.\n%s"),
                    host, conf->port, errmsg);
            else
                detail = g_strdup_printf(
                    _("Failed to spawn rigctld for %s:%d."),
                    host, conf->port);
            schedule_rig_autostart_error(ctrl, conf, role, detail);
            g_free(detail);
            g_free(errmsg);
            reported = TRUE;
            goto out;
        }

        rigctld_set_spawn_state(ctrl, secondary, *mgr);
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: rigctld started pid=%s"),
                    __func__,
                    rigctld_mgr_get_identifier(*mgr) ?
                        rigctld_mgr_get_identifier(*mgr) : "(unknown)");
        rig_term_log(ctrl, "gpredict",
                     "rigctld started pid=%s",
                     rigctld_mgr_get_identifier(*mgr) ?
                         rigctld_mgr_get_identifier(*mgr) : "(unknown)");
        rigctld_mgr_set_log_callback(*mgr, rigctld_log_cb, ctrl);
    }

    sat_log_log(SAT_LOG_LEVEL_INFO,
                _("%s: rigctld spawn complete; deferring probe to session"),
                __func__);
    rig_term_log(ctrl, "gpredict",
                 "rigctld spawn complete; probing on session connection");

    ok = TRUE;

out:
    if (mgr && *mgr != NULL)
        rigctld_mgr_set_log_callback(*mgr, rigctld_log_cb, ctrl);
    if (error_reported)
        *error_reported = reported;
    if (own_host)
        g_free(host);
    return ok;
}

static gboolean open_rigctld_socket_with_autostart(GtkRigCtrl *ctrl,
                                                   radio_conf_t *conf,
                                                   gint *sock,
                                                   gboolean secondary,
                                                   const gchar *role,
                                                   gboolean *error_reported)
{
    RigctldMgr **mgr =
        secondary ? &ctrl->rigctld_mgr2 : &ctrl->rigctld_mgr;
    gchar       *host = NULL;
    gchar       *detail = NULL;
    gboolean     reported = FALSE;
    gint         err = 0;
    gint         so_err = 0;
    gboolean     connected = FALSE;
    gboolean     ic905_fallback = FALSE;
    gint         autodetect_restart_attempts = 0;
    gboolean     force_local_recovery = FALSE;

    if (conf == NULL)
        return FALSE;

    if (conf->port <= 0 || conf->host == NULL || *conf->host == '\0')
    {
        rigctld_preset_defaults_t preset;

        if (radio_model_get_rigctld_defaults(conf->radio_model, &preset))
        {
            if (conf->host == NULL || *conf->host == '\0')
            {
                g_free(conf->host);
                conf->host = g_strdup(preset.host);
            }
            if (conf->port <= 0)
                conf->port = preset.port;
        }
    }

    host = rigctld_mgr_normalize_host(conf->host);
    if (host == NULL || conf->port <= 0)
        return FALSE;

    rig_term_log(ctrl, "gpredict", "connecting to %s:%d", host, conf->port);
    connected = open_rigctld_socket_host(host, conf->port, sock,
                                         &err, &so_err, TRUE);

    if (!connected)
    {
        if (!conf->rigctld_autostart)
        {
            force_local_recovery =
                (conf->rigctld_conn == RIGCTLD_CONN_SERIAL &&
                 rigctld_mgr_host_is_local(conf->host) &&
                 (radio_model_to_hamlib_model(conf->radio_model) > 0 ||
                  conf->rigctld_model > 0) &&
                 (conf->rigctld_device == NULL ||
                  *conf->rigctld_device == '\0' ||
                  !rigctld_serial_device_exists(conf->rigctld_device)));

            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: Failed to connect to %s:%d"),
                        __func__, host, conf->port);
            rig_term_log(ctrl, "gpredict:err",
                         "connect failed to %s:%d (errno=%d so_error=%d)",
                         host, conf->port, err, so_err);

            if (!force_local_recovery)
            {
                rig_term_log(ctrl, "gpredict:err",
                             "local recovery skipped: autostart=%d conn=%s device=%s",
                             conf->rigctld_autostart ? 1 : 0,
                             rigctld_conn_name(conf->rigctld_conn),
                             (conf->rigctld_device && *conf->rigctld_device)
                                 ? conf->rigctld_device
                                 : "(none)");
                g_free(host);
                if (error_reported)
                    *error_reported = reported;
                return FALSE;
            }

            rig_term_log(ctrl, "gpredict",
                         "forcing local serial recovery via autodetect (autostart=0)");
        }

retry_autostart:
        g_free(host);
        host = NULL;
        if (!ensure_rigctld_running(ctrl, conf, mgr, secondary, role, &host,
                                    &reported, ic905_fallback,
                                    force_local_recovery))
        {
            if (error_reported)
                *error_reported = reported;
            g_free(host);
            return FALSE;
        }

        if (host == NULL)
            return FALSE;

        rig_term_log(ctrl, "gpredict",
                     "connecting to %s:%d (autostart)", host, conf->port);
        {
            gint64 start_us = g_get_monotonic_time();
            gint waited_ms = 0;
            gboolean session_connected = FALSE;
            gchar *exit_detail = NULL;
            gchar *stderr_text = NULL;

            while (waited_ms < RIGCTLD_AUTOSTART_TIMEOUT_MS)
            {
                if (mgr && *mgr && !rigctld_mgr_is_running(*mgr))
                {
                    gboolean exited = FALSE;
                    gint status = -1;
                    gint signal = 0;

                    (void)rigctld_mgr_get_exit_info(*mgr, &exited,
                                                    &status, &signal);
                    if (signal > 0)
                        exit_detail =
                            g_strdup_printf("rigctld exited via signal %d", signal);
                    else if (status >= 0)
                        exit_detail =
                            g_strdup_printf("rigctld exit status %d", status);
                    else
                        exit_detail = g_strdup("rigctld exited");
                    break;
                }

                if (open_rigctld_socket_host(host, conf->port, sock,
                                             &err, &so_err, FALSE))
                {
                    session_connected = TRUE;
                    break;
                }

                g_usleep((gulong)RIGCTLD_AUTOSTART_POLL_MS * 1000);
                waited_ms = (gint)((g_get_monotonic_time() - start_us) / 1000);
            }

            if (!session_connected)
            {
                if (rigctld_is_ic905(conf) && !ic905_fallback)
                {
                    sat_log_log(SAT_LOG_LEVEL_WARN,
                                _("%s: IC-905 auto-start retry without civaddr"),
                                __func__);
                    rig_term_log(ctrl, "gpredict:err",
                                 "IC-905 auto-start retry without civaddr");
                    ic905_fallback = TRUE;
                    rigctld_terminate_spawned(ctrl, secondary, mgr);
                    g_free(exit_detail);
                    g_free(stderr_text);
                    goto retry_autostart;
                }

                if (exit_detail != NULL)
                {
                    sat_log_log(SAT_LOG_LEVEL_ERROR,
                                _("%s: auto-start failed; rigctld exited early (%s)"),
                                __func__, exit_detail);
                    rig_term_log(ctrl, "gpredict:err",
                                 "rigctld exited early (%s)",
                                 exit_detail);
                }
                else
                {
                    sat_log_log(SAT_LOG_LEVEL_ERROR,
                                _("%s: auto-start failed; rigctld not listening on %s:%d after %d ms"),
                                __func__, host, conf->port, waited_ms);
                    rig_term_log(ctrl, "gpredict:err",
                                 "rigctld not listening on %s:%d after %d ms",
                                 host, conf->port, waited_ms);
                }

                if (mgr && *mgr)
                    stderr_text = rigctld_mgr_get_log_tail(*mgr);

                if (conf->rigctld_conn == RIGCTLD_CONN_SERIAL &&
                    rigctld_mgr_host_is_local(conf->host) &&
                    (conf->rigctld_device == NULL ||
                     *conf->rigctld_device == '\0' ||
                     !rigctld_serial_device_exists(conf->rigctld_device) ||
                     rigctld_log_tail_indicates_stale_device(stderr_text)) &&
                    rigctld_try_autodetect_restart(
                        ctrl, conf, mgr, secondary, role, &reported,
                        &autodetect_restart_attempts,
                        (stderr_text && *stderr_text) ? "startup failure"
                                                      : "startup timeout"))
                {
                    g_free(exit_detail);
                    g_free(stderr_text);
                    goto retry_autostart;
                }

                if (stderr_text && *stderr_text)
                    detail = g_strdup_printf(
                        _("rigctld did not start listening on %s:%d.\n%s"),
                        host, conf->port, stderr_text);
                else
                    detail = g_strdup_printf(
                        _("rigctld did not start listening on %s:%d."),
                        host, conf->port);

                schedule_rig_autostart_error(ctrl, conf, role, detail);
                rigctld_terminate_spawned(ctrl, secondary, mgr);
                g_free(exit_detail);
                g_free(stderr_text);
                g_free(detail);
                g_free(host);
                reported = TRUE;
                if (error_reported)
                    *error_reported = reported;
                return FALSE;
            }
        }
    }

    {
        RigctldClient **client_ptr =
            secondary ? &ctrl->rig_client2 : &ctrl->rig_client;
        gchar *client_err = NULL;

        if (*client_ptr == NULL)
            *client_ptr = rigctld_client_new(role ? role : "rig");

        if (!rigctld_client_attach_fd(*client_ptr, *sock, &client_err))
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: Failed to attach rigctld socket for %s:%d: %s"),
                        __func__, host, conf->port,
                        client_err ? client_err : "(unknown)");
            rig_term_log(ctrl, "gpredict:err",
                         "attach failed for %s:%d (%s)",
                         host, conf->port,
                         client_err ? client_err : "(unknown)");
            g_free(client_err);
            rigctld_close_fd(*sock);
            *sock = -1;
            g_free(host);
            if (error_reported)
                *error_reported = reported;
            return FALSE;
        }

        g_free(client_err);
    }

    rig_term_log(ctrl, "gpredict", "connected to %s:%d", host, conf->port);
    rigctld_clear_vfo_map_for_socket(ctrl, *sock);
    rigctld_reset_offset_state_for_socket(ctrl, *sock);
    g_free(host);
    if (error_reported)
        *error_reported = reported;
    return TRUE;
}

typedef struct {
    GtkRigCtrl *ctrl;
    gchar      *host;
    gint        port;
    gchar      *role;
} RigConnErrorInfo;

static gboolean rig_conn_error_idle(gpointer data)
{
    RigConnErrorInfo *info = data;
    gchar            *details;
    const gchar      *summary =
        _("rigctld did not respond. See Rig Log for details.");

    if (info == NULL)
        return G_SOURCE_REMOVE;

    details = g_strdup_printf(_("Rig: %s\nHost: %s\nPort: %d"),
                              info->role ? info->role : _("rig"),
                              info->host ? info->host : "(null) - missing",
                              info->port);
    rig_show_error_dialog_with_details(info->ctrl,
                                       _("Unable to connect to radio"),
                                       summary, details);
    g_free(details);
    g_free(info->host);
    g_free(info->role);
    g_free(info);

    return G_SOURCE_REMOVE;
}

static void schedule_rig_conn_error(GtkRigCtrl *ctrl, radio_conf_t *conf,
                                    const gchar *role)
{
    RigConnErrorInfo *info = g_new0(RigConnErrorInfo, 1);

    info->ctrl = ctrl;
    info->host = rigctld_mgr_normalize_host(conf ? conf->host : NULL);
    info->port = conf ? conf->port : 0;
    info->role = g_strdup(role);

    g_idle_add(rig_conn_error_idle, info);
}

typedef struct {
    GtkRigCtrl *ctrl;
    gchar      *host;
    gint        port;
    gchar      *role;
} RigBackendErrorInfo;

static gboolean rig_backend_error_idle(gpointer data)
{
    RigBackendErrorInfo *info = data;
    gchar *details;
    const gchar *summary =
        _("Rig backend could not open serial port. See Rig Log for details.");

    if (info == NULL)
        return G_SOURCE_REMOVE;

    details = g_strdup_printf(_("Rig: %s\nHost: %s\nPort: %d"),
                              info->role ? info->role : _("rig"),
                              info->host ? info->host : "(null) - missing",
                              info->port);
    rig_show_error_dialog_with_details(info->ctrl,
                                       _("Unable to connect to radio"),
                                       summary, details);
    g_free(details);
    g_free(info->host);
    g_free(info->role);
    g_free(info);

    return G_SOURCE_REMOVE;
}

static void schedule_rig_backend_error(GtkRigCtrl *ctrl, radio_conf_t *conf,
                                       const gchar *role)
{
    RigBackendErrorInfo *info = g_new0(RigBackendErrorInfo, 1);

    info->ctrl = ctrl;
    info->host = rigctld_mgr_normalize_host(conf ? conf->host : NULL);
    info->port = conf ? conf->port : 0;
    info->role = g_strdup(role);

    g_idle_add(rig_backend_error_idle, info);
}

typedef struct {
    GtkRigCtrl *ctrl;
    gchar      *title;
    gchar      *body;
} RigAutostartErrorInfo;

static gboolean rig_autostart_error_idle(gpointer data)
{
    RigAutostartErrorInfo *info = data;

    if (info == NULL)
        return G_SOURCE_REMOVE;

    if (info->body && *info->body)
    {
        rig_term_log(info->ctrl, "gpredict:err", "%s", info->body);
        sat_log_log(SAT_LOG_LEVEL_ERROR, "%s", info->body);
    }
    rig_show_error_dialog(info->ctrl,
                          info->title ? info->title : _("rigctld error"),
                          _("See log for details."));
    g_free(info->title);
    g_free(info->body);
    g_free(info);

    return G_SOURCE_REMOVE;
}

static void schedule_rig_autostart_error(GtkRigCtrl *ctrl,
                                         const radio_conf_t *conf,
                                         const gchar *role,
                                         const gchar *detail)
{
    RigAutostartErrorInfo *info;
    const gchar *label = (role != NULL) ? role : _("rig");

    if (ctrl == NULL)
        return;

    if (!rigctrl_autostart_error_allowed(ctrl, conf))
        return;

    info = g_new0(RigAutostartErrorInfo, 1);
    info->ctrl = ctrl;
    info->title = g_strdup_printf(_("Unable to auto-start rigctld (%s)"), label);
    info->body = g_strdup(detail ? detail : "");

    g_idle_add(rig_autostart_error_idle, info);
}

typedef struct {
    GtkRigCtrl *ctrl;
    gchar      *body;
} RigAutodetectErrorInfo;

static gboolean rig_autodetect_error_idle(gpointer data)
{
    RigAutodetectErrorInfo *info = data;

    if (info == NULL)
        return G_SOURCE_REMOVE;

    if (info->body && *info->body)
    {
        rig_term_log(info->ctrl, "gpredict:err", "%s", info->body);
        sat_log_log(SAT_LOG_LEVEL_ERROR, "%s", info->body);
    }
    rig_show_error_dialog(info->ctrl,
                          _("Radio not found"),
                          _("See log for details."));
    g_free(info->body);
    g_free(info);

    return G_SOURCE_REMOVE;
}

static void schedule_rig_autodetect_error(GtkRigCtrl *ctrl,
                                          const radio_conf_t *conf,
                                          const gchar *detail)
{
    RigAutodetectErrorInfo *info;

    if (ctrl == NULL)
        return;

    if (!rigctrl_autostart_error_allowed(ctrl, conf))
        return;

    info = g_new0(RigAutodetectErrorInfo, 1);
    info->ctrl = ctrl;
    info->body = g_strdup(detail ? detail : "");

    g_idle_add(rig_autodetect_error_idle, info);
}

typedef struct {
    GtkRigCtrl *ctrl;
    gchar      *rig_id;
} RigMissingModelInfo;

static void rig_missing_model_response(GtkDialog *dialog,
                                       gint response,
                                       gpointer user_data)
{
    RigMissingModelInfo *info = user_data;

    if (info == NULL)
        return;

    switch (response)
    {
    case RIGCTRL_RESPONSE_OPEN_CONFIG:
        rigctrl_open_radio_config(info->ctrl, info->rig_id);
        break;
    case RIGCTRL_RESPONSE_DISABLE_AUTOSTART:
        rigctrl_disable_autostart(info->ctrl, info->rig_id);
        break;
    default:
        break;
    }

    gtk_widget_destroy(GTK_WIDGET(dialog));
    g_free(info->rig_id);
    g_free(info);
}

static gboolean rig_missing_model_idle(gpointer data)
{
    RigMissingModelInfo *info = data;
    GtkWidget *toplevel;
    GtkWindow *parent = NULL;
    GtkWidget *dialog;

    if (info == NULL)
        return G_SOURCE_REMOVE;

    if (info->ctrl != NULL)
    {
        toplevel = gtk_widget_get_toplevel(GTK_WIDGET(info->ctrl));
        if (GTK_IS_WINDOW(toplevel))
            parent = GTK_WINDOW(toplevel);
    }

    dialog = gtk_message_dialog_new(parent,
                                    GTK_DIALOG_DESTROY_WITH_PARENT,
                                    GTK_MESSAGE_ERROR,
                                    GTK_BUTTONS_NONE,
                                    "%s",
                                    _("rigctld could not be started because the Hamlib rig model is missing"));
    gtk_dialog_add_button(GTK_DIALOG(dialog),
                          _("Open Radio Config"),
                          RIGCTRL_RESPONSE_OPEN_CONFIG);
    gtk_dialog_add_button(GTK_DIALOG(dialog),
                          _("Disable Auto-start"),
                          RIGCTRL_RESPONSE_DISABLE_AUTOSTART);
    gtk_dialog_add_button(GTK_DIALOG(dialog),
                          _("Dismiss"),
                          GTK_RESPONSE_CANCEL);

    g_signal_connect(dialog, "response",
                     G_CALLBACK(rig_missing_model_response), info);
    gtk_widget_show(dialog);

    return G_SOURCE_REMOVE;
}

static void schedule_rig_missing_model_dialog(GtkRigCtrl *ctrl,
                                              const radio_conf_t *conf)
{
    RigMissingModelInfo *info;

    if (ctrl == NULL || conf == NULL)
        return;

    if (!rigctrl_missing_model_dialog_allowed(ctrl, conf))
        return;

    info = g_new0(RigMissingModelInfo, 1);
    info->ctrl = ctrl;
    info->rig_id = g_strdup(conf->name);

    g_idle_add(rig_missing_model_idle, info);
}

static gboolean rig_disengage_idle(gpointer data)
{
    GtkRigCtrl *ctrl = GTK_RIG_CTRL(data);

    if (ctrl != NULL && !ctrl->destroying)
    {
        if (ctrl->DevSel != NULL)
            gtk_widget_set_sensitive(ctrl->DevSel, TRUE);
        if (ctrl->DevSel2 != NULL)
            gtk_widget_set_sensitive(ctrl->DevSel2, TRUE);
        if (ctrl->LockBut != NULL)
        {
            g_signal_handlers_block_by_func(ctrl->LockBut,
                                            (gpointer)rig_engaged_cb, ctrl);
            gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->LockBut), FALSE);
            g_signal_handlers_unblock_by_func(ctrl->LockBut,
                                              (gpointer)rig_engaged_cb, ctrl);
            rig_engaged_cb(GTK_TOGGLE_BUTTON(ctrl->LockBut), ctrl);
        }
    }
    if (ctrl != NULL)
        g_object_unref(ctrl);

    return G_SOURCE_REMOVE;
}

static void schedule_rig_disengage(GtkRigCtrl *ctrl)
{
    guint source_id;

    if (ctrl == NULL)
        return;

    g_object_ref(ctrl);
    source_id = g_idle_add(rig_disengage_idle, ctrl);
    if (source_id == 0)
        g_object_unref(ctrl);
}

static void rigctrl_fail_engage(GtkRigCtrl *ctrl, const gchar *reason)
{
    const gchar *detail = (reason != NULL) ? reason : "rig engage failed";

    if (ctrl == NULL || !ctrl->engage_pending)
        return;

    ctrl->engage_pending = FALSE;
    ctrl->engaged = FALSE;
    rigctrl_set_ui_hard_error(ctrl, detail);
    rigctrl_set_status_detail(ctrl, detail);
    schedule_rig_disengage(ctrl);
    rigctrl_queue_ui_status_refresh(ctrl, detail);
}

static void rigctrl_cancel_open_task(GtkRigCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    if (ctrl->open_cancellable != NULL)
        g_cancellable_cancel(ctrl->open_cancellable);
}

static gboolean G_GNUC_UNUSED rigctrl_open_idle(gpointer data)
{
    GtkRigCtrl *ctrl = GTK_RIG_CTRL(data);

    rigctrl_open_internal(ctrl);
    g_object_unref(ctrl);

    return G_SOURCE_REMOVE;
}

static void rigctrl_open_task(GTask *task, gpointer source_object,
                              gpointer task_data, GCancellable *cancellable)
{
    GtkRigCtrl *ctrl = GTK_RIG_CTRL(source_object);
    gboolean ok = FALSE;

    (void)task_data;

    if (ctrl == NULL)
    {
        g_task_return_new_error(task, G_IO_ERROR, G_IO_ERROR_FAILED,
                                "rig engage failed");
        return;
    }

    if (g_cancellable_is_cancelled(cancellable))
    {
        g_task_return_new_error(task, G_IO_ERROR, G_IO_ERROR_CANCELLED,
                                "rig engage cancelled");
        return;
    }

    ok = rigctrl_open_internal(ctrl);

    if (g_cancellable_is_cancelled(cancellable))
    {
        g_task_return_new_error(task, G_IO_ERROR, G_IO_ERROR_CANCELLED,
                                "rig engage cancelled");
        return;
    }

    g_task_return_boolean(task, ok);
}

static void rigctrl_open_task_done(GObject *source, GAsyncResult *res,
                                   gpointer user_data)
{
    GtkRigCtrl *ctrl = GTK_RIG_CTRL(source);
    GError *error = NULL;
    gboolean ok = FALSE;

    (void)user_data;

    if (ctrl == NULL)
        return;

    ok = g_task_propagate_boolean(G_TASK(res), &error);

    if (ctrl->open_task != NULL)
        g_clear_object(&ctrl->open_task);
    g_clear_object(&ctrl->open_cancellable);

    if (ctrl->destroying)
    {
        g_clear_error(&error);
        return;
    }

    if (error != NULL)
    {
        if (g_error_matches(error, G_IO_ERROR, G_IO_ERROR_CANCELLED))
            rigctrl_request_close(ctrl);
        g_clear_error(&error);
        return;
    }

    if (!ok)
    {
        if (ctrl->sock < 0)
            rigctrl_schedule_reconnect(ctrl, FALSE, _("receiver"));
        if (ctrl->conf2 != NULL && ctrl->sock2 < 0)
            rigctrl_schedule_reconnect(ctrl, TRUE, _("uplink"));
        return;
    }

    rigctrl_seed_user_base_from_ui(ctrl, "engage");
}

static void rigctrl_start_open_task(GtkRigCtrl *ctrl)
{
    if (ctrl == NULL || ctrl->destroying || !ctrl->engaged)
        return;
    if (ctrl->link_lost_latched || ctrl->link_lost_latched2)
        return;

    if (ctrl->open_task != NULL)
        return;

    g_clear_object(&ctrl->open_cancellable);
    ctrl->open_cancellable = g_cancellable_new();
    ctrl->open_task = g_task_new(ctrl, ctrl->open_cancellable,
                                 rigctrl_open_task_done, NULL);
    g_task_run_in_thread(ctrl->open_task, rigctrl_open_task);
}

static void rigctrl_request_open(GtkRigCtrl *ctrl, const gchar *reason)
{
    gboolean need_rx;
    gboolean need_tx;
    gboolean schedule = FALSE;

    if (ctrl == NULL || ctrl->destroying || !ctrl->engaged)
        return;
    if (ctrl->link_lost_latched || ctrl->link_lost_latched2)
        return;

    /* Opening/state guards keep reconnect attempts from re-entering open(). */
    need_rx = (ctrl->sock < 0);
    need_tx = (ctrl->conf2 != NULL && ctrl->sock2 < 0);
    if (need_rx && ctrl->conn_state == RIGCTRL_CONN_DISCONNECTING)
        need_rx = FALSE;
    if (need_tx && ctrl->conn_state2 == RIGCTRL_CONN_DISCONNECTING)
        need_tx = FALSE;
    if (!need_rx && !need_tx)
        return;

    if (need_rx && !ctrl->opening)
    {
        ctrl->opening = TRUE;
        rigctrl_set_conn_state(ctrl, FALSE, RIGCTRL_CONN_CONNECTING,
                               reason ? reason : "open requested");
        schedule = TRUE;
    }

    if (need_tx && !ctrl->opening2)
    {
        ctrl->opening2 = TRUE;
        rigctrl_set_conn_state(ctrl, TRUE, RIGCTRL_CONN_CONNECTING,
                               reason ? reason : "open requested");
        schedule = TRUE;
    }

    if (!schedule)
        return;

    rigctrl_start_open_task(ctrl);
}

static gboolean rigctrl_close_idle(gpointer data)
{
    GtkRigCtrl *ctrl = GTK_RIG_CTRL(data);

    ctrl->close_pending_id = 0;
    rigctrl_close_internal(ctrl);
    g_object_unref(ctrl);

    return G_SOURCE_REMOVE;
}

static void rigctrl_request_close(GtkRigCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    if (rigctrl_on_main_thread(ctrl))
    {
        rigctrl_close_internal(ctrl);
        return;
    }

    if (ctrl->close_pending_id != 0)
        return;

    g_object_ref(ctrl);
    ctrl->close_pending_id = g_idle_add(rigctrl_close_idle, ctrl);
}

static void rigctrl_close(GtkRigCtrl * data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);

    if (ctrl == NULL)
        return;

    if (ctrl->destroying)
        return;

    rigctrl_request_close(ctrl);
}

static void rigctrl_close_internal(GtkRigCtrl * data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);

    if (ctrl == NULL)
        return;

    /* Stop timers before touching sockets to avoid re-entrant callbacks. */
    remove_timer(ctrl);

    rigctrl_set_conn_state(ctrl, FALSE, RIGCTRL_CONN_DISCONNECTING,
                           "close requested");
    rigctrl_set_conn_state(ctrl, TRUE, RIGCTRL_CONN_DISCONNECTING,
                           "close requested");

    ctrl->lastrxptt = FALSE;
    ctrl->lasttxptt = TRUE;
    ctrl->lasttxf = 0;
    ctrl->lastrxf = 0;

    if (ctrl->conf != NULL)
    {
        apply_rit_xit_offsets(ctrl, 0.0, 0.0);

        if ((ctrl->conf->type == RIG_TYPE_TOGGLE_AUTO) ||
            (ctrl->conf->type == RIG_TYPE_TOGGLE_MAN))
        {
            if (ctrl->sock >= 0)
                unset_toggle(ctrl, ctrl->sock);
        }
    }

    if (ctrl->conf2 != NULL)
    {
        close_rigctld_socket(ctrl, &(ctrl->sock2),
                             rigctld_spawned_by_us(ctrl, TRUE));
    }
    close_rigctld_socket(ctrl, &(ctrl->sock),
                         rigctld_spawned_by_us(ctrl, FALSE));

    if (ctrl->conf2 == NULL &&
        is_full_duplex_main_sub_configured(ctrl->conf) &&
        ctrl->rig_session2 != NULL)
    {
        rig_session_reset(ctrl->rig_session2);
    }

    rigctld_terminate_spawned(ctrl, TRUE, &ctrl->rigctld_mgr2);
    rigctld_terminate_spawned(ctrl, FALSE, &ctrl->rigctld_mgr);

    rigctrl_reset_reconnect(ctrl, FALSE);
    rigctrl_reset_reconnect(ctrl, TRUE);

    ctrl->opening = FALSE;
    ctrl->opening2 = FALSE;

    rigctrl_set_conn_state(ctrl, FALSE, RIGCTRL_CONN_DISCONNECTED, "closed");
    rigctrl_set_conn_state(ctrl, TRUE, RIGCTRL_CONN_DISCONNECTED, "closed");
}

static void rigctrl_close_socket_internal(GtkRigCtrl *ctrl, gboolean secondary)
{
    gint *sock_ptr;
    gboolean shared_tx = FALSE;

    if (ctrl == NULL)
        return;

    shared_tx = (!secondary &&
                 ctrl->conf2 == NULL &&
                 is_full_duplex_main_sub_configured(ctrl->conf));

    /* Stop timers before closing a socket to keep callbacks out. */
    remove_timer(ctrl);

    rigctrl_set_conn_state(ctrl, secondary, RIGCTRL_CONN_DISCONNECTING,
                           "socket close");
    if (shared_tx)
    {
        rigctrl_set_conn_state(ctrl, TRUE, RIGCTRL_CONN_DISCONNECTING,
                               "shared rig socket close");
    }

    sock_ptr = secondary ? &ctrl->sock2 : &ctrl->sock;
    if (sock_ptr != NULL && *sock_ptr >= 0)
        close_rigctld_socket(ctrl, sock_ptr,
                             rigctld_spawned_by_us(ctrl, secondary));

    if (secondary)
        ctrl->opening2 = FALSE;
    else
        ctrl->opening = FALSE;

    rigctrl_set_conn_state(ctrl, secondary, RIGCTRL_CONN_DISCONNECTED,
                           "socket closed");
    if (shared_tx)
    {
        if (ctrl->rig_session2 != NULL)
            rig_session_reset(ctrl->rig_session2);
        rigctrl_set_conn_state(ctrl, TRUE, RIGCTRL_CONN_DISCONNECTED,
                               "shared rig socket closed");
    }
}

static gboolean rigctrl_close_socket_idle(gpointer data)
{
    RigctrlCloseSocketInfo *info = data;

    if (info == NULL)
        return G_SOURCE_REMOVE;

    if (info->ctrl != NULL)
        rigctrl_close_socket_internal(info->ctrl, info->secondary);

    if (info->ctrl != NULL)
        g_object_unref(info->ctrl);
    g_free(info);

    return G_SOURCE_REMOVE;
}

static gboolean rigctrl_open_internal(GtkRigCtrl * data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);
    gboolean        rx_opened = FALSE;
    gboolean        tx_opened = FALSE;
    gboolean        tx_ok = TRUE;

    if (ctrl == NULL || ctrl->destroying || !ctrl->engaged)
        return FALSE;

    if (ctrl->conf == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: missing primary rig configuration"), __func__);
        rigctrl_set_ui_hard_error(ctrl, "missing primary rig configuration");
        rigctrl_set_status_detail(ctrl, "missing primary rig configuration");
        rigctrl_set_conn_state(ctrl, FALSE, RIGCTRL_CONN_DISCONNECTED,
                               "missing config");
        ctrl->opening = FALSE;
        ctrl->opening2 = FALSE;
        rigctrl_queue_ui_status_refresh(ctrl, "missing primary rig configuration");
        return FALSE;
    }

    if (ctrl->timerid)
        remove_timer(ctrl);

    if (!ctrl->opening && ctrl->sock < 0)
        ctrl->opening = TRUE;
    if (!ctrl->opening2 && ctrl->conf2 != NULL && ctrl->sock2 < 0)
        ctrl->opening2 = TRUE;

    if (ctrl->sock < 0)
    {
        rigctrl_set_conn_state(ctrl, FALSE, RIGCTRL_CONN_CONNECTING,
                               "open start");
        if (is_full_duplex_main_sub_configured(ctrl->conf) &&
            ctrl->conf2 == NULL)
        {
            rigctrl_set_conn_state(ctrl, TRUE, RIGCTRL_CONN_CONNECTING,
                                   "open start (shared rig)");
        }
        ctrl->wrops = 0;
        rigctrl_log_config(ctrl, ctrl->conf, _("receiver"));
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: opening receiver rig %s:%d"), __func__,
                    ctrl->conf ? ctrl->conf->host : "(null)",
                    ctrl->conf ? ctrl->conf->port : 0);
        g_printerr("%s: opening receiver rig %s:%d\n", __func__,
                   ctrl->conf ? ctrl->conf->host : "(null)",
                   ctrl->conf ? ctrl->conf->port : 0);

        {
            gboolean error_reported = FALSE;
            gboolean stale_device = FALSE;
            gint probe_restart_attempts = 0;
            RigSession *session = ctrl->rig_session;

open_receiver_retry:
            error_reported = FALSE;
            stale_device = FALSE;

            if (!open_rigctld_socket_with_autostart(ctrl, ctrl->conf,
                                                    &(ctrl->sock),
                                                    FALSE, _("receiver"),
                                                    &error_reported))
            {
                sat_log_log(SAT_LOG_LEVEL_ERROR,
                            _("%s: receiver rig open/probe failed"), __func__);
                close_rigctld_socket(ctrl, &(ctrl->sock),
                                     rigctld_spawned_by_us(ctrl, FALSE));
                if (stale_device && !ctrl->rx_conn_error_reported)
                {
                    schedule_rig_backend_error(ctrl, ctrl->conf, _("receiver"));
                    ctrl->rx_conn_error_reported = TRUE;
                }
                if (!stale_device &&
                    !error_reported && !ctrl->rx_conn_error_reported)
                {
                    schedule_rig_conn_error(ctrl, ctrl->conf, _("receiver"));
                    ctrl->rx_conn_error_reported = TRUE;
                }
                rigctrl_fail_engage(ctrl, "receiver open/probe failed");
                rigctrl_set_conn_state(ctrl, FALSE, RIGCTRL_CONN_DISCONNECTED,
                                       "open failed");
                ctrl->opening = FALSE;
                ctrl->opening2 = FALSE;
                return FALSE;
            }

            rig_session_reset(session);
            rig_session_set_state(ctrl, session, RIG_SESSION_CONNECTING,
                                  "connected");
            if (!rig_session_probe_and_configure(ctrl, session,
                                                 ctrl->sock, ctrl->conf))
            {
                RigctldMgr *mgr = ctrl->rigctld_mgr;
                gchar *log_tail = NULL;

                if (mgr)
                    log_tail = rigctld_mgr_get_log_tail(mgr);
                stale_device = rigctld_log_tail_indicates_stale_device(log_tail);

                sat_log_log(SAT_LOG_LEVEL_ERROR,
                            _("%s: receiver rig session probe failed"), __func__);
                if (rigctrl_log_throttled(ctrl, &ctrl->last_probe_log_us,
                                          RIGCTRL_PROBE_LOG_INTERVAL_US))
                    rig_term_log_verbose(ctrl, "gpredict",
                                         "rigctld probe failed; leaving daemon running");
                close_rigctld_socket(ctrl, &(ctrl->sock), FALSE);
                if (ctrl->rigctld_mgr != NULL &&
                    rigctld_spawned_by_us(ctrl, FALSE) &&
                    ctrl->conf->rigctld_conn == RIGCTLD_CONN_SERIAL &&
                    rigctld_mgr_host_is_local(ctrl->conf->host) &&
                    rigctld_try_autodetect_restart(
                        ctrl, ctrl->conf, &ctrl->rigctld_mgr, FALSE,
                        _("receiver"), &error_reported,
                        &probe_restart_attempts,
                        stale_device ? "probe stale device"
                                     : "probe failure"))
                {
                    g_free(log_tail);
                    goto open_receiver_retry;
                }
                g_free(log_tail);
                if (stale_device && !ctrl->rx_conn_error_reported)
                {
                    schedule_rig_backend_error(ctrl, ctrl->conf, _("receiver"));
                    ctrl->rx_conn_error_reported = TRUE;
                }
                if (!stale_device &&
                    !error_reported && !ctrl->rx_conn_error_reported)
                {
                    schedule_rig_conn_error(ctrl, ctrl->conf, _("receiver"));
                    ctrl->rx_conn_error_reported = TRUE;
                }
                rigctrl_fail_engage(ctrl, "receiver session probe failed");
                rigctrl_set_conn_state(ctrl, FALSE, RIGCTRL_CONN_DISCONNECTED,
                                       "open failed");
                ctrl->opening = FALSE;
                ctrl->opening2 = FALSE;
                return FALSE;
            }
        }

        rigctrl_reset_reconnect(ctrl, FALSE);
        rx_opened = TRUE;

        ctrl->conf->vfo_opt = (ctrl->rig_session &&
                               ctrl->rig_session->strategy == RIG_STRATEGY_VFO_OPT_ARGS);
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                _("%s:%s: VFO opt=%d"), __FILE__,
                __func__, ctrl->conf->vfo_opt);

        if (is_full_duplex_main_sub_configured(ctrl->conf) &&
            ctrl->conf2 == NULL)
        {
            if (rigctrl_prepare_shared_tx_session(ctrl))
                tx_opened = TRUE;
        }
    }

    /* set initial frequency */
    if (ctrl->conf2 != NULL && ctrl->sock2 < 0)
    {
        rigctrl_set_conn_state(ctrl, TRUE, RIGCTRL_CONN_CONNECTING,
                               "open start");
        rigctrl_log_config(ctrl, ctrl->conf2, _("uplink"));
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: opening uplink rig %s:%d"), __func__,
                    ctrl->conf2 ? ctrl->conf2->host : "(null)",
                    ctrl->conf2 ? ctrl->conf2->port : 0);
        g_printerr("%s: opening uplink rig %s:%d\n", __func__,
                   ctrl->conf2 ? ctrl->conf2->host : "(null)",
                   ctrl->conf2 ? ctrl->conf2->port : 0);

        {
            gboolean error_reported = FALSE;
            gboolean stale_device = FALSE;
            gint probe_restart_attempts = 0;
            RigSession *session = ctrl->rig_session2;

open_uplink_retry:
            error_reported = FALSE;
            stale_device = FALSE;

            tx_ok = open_rigctld_socket_with_autostart(
                ctrl, ctrl->conf2, &(ctrl->sock2), TRUE, _("uplink"),
                &error_reported);
            if (tx_ok)
            {
                rig_session_reset(session);
                rig_session_set_state(ctrl, session, RIG_SESSION_CONNECTING,
                                      "connected");
                tx_ok = rig_session_probe_and_configure(ctrl, session,
                                                        ctrl->sock2,
                                                        ctrl->conf2);
                if (!tx_ok)
                {
                    RigctldMgr *mgr = ctrl->rigctld_mgr2;
                    gchar *log_tail = NULL;

                    if (mgr)
                        log_tail = rigctld_mgr_get_log_tail(mgr);
                    stale_device = rigctld_log_tail_indicates_stale_device(log_tail);
                    close_rigctld_socket(ctrl, &(ctrl->sock2), FALSE);
                    if (ctrl->rigctld_mgr2 != NULL &&
                        rigctld_spawned_by_us(ctrl, TRUE) &&
                        ctrl->conf2->rigctld_conn == RIGCTLD_CONN_SERIAL &&
                        rigctld_mgr_host_is_local(ctrl->conf2->host) &&
                        rigctld_try_autodetect_restart(
                            ctrl, ctrl->conf2, &ctrl->rigctld_mgr2, TRUE,
                            _("uplink"), &error_reported,
                            &probe_restart_attempts,
                            stale_device ? "probe stale device"
                                         : "probe failure"))
                    {
                        g_free(log_tail);
                        goto open_uplink_retry;
                    }
                    g_free(log_tail);
                }
            }
            if (!tx_ok)
            {
                sat_log_log(SAT_LOG_LEVEL_ERROR,
                            _("%s: uplink rig open/probe failed"), __func__);
                if (rigctrl_log_throttled(ctrl, &ctrl->last_probe_log_us,
                                          RIGCTRL_PROBE_LOG_INTERVAL_US))
                    rig_term_log_verbose(ctrl, "gpredict",
                                         "rigctld probe failed; leaving daemon running");
                if (ctrl->sock2 >= 0)
                    close_rigctld_socket(ctrl, &(ctrl->sock2), FALSE);
                if (stale_device && !ctrl->tx_conn_error_reported)
                {
                    schedule_rig_backend_error(ctrl, ctrl->conf2, _("uplink"));
                    ctrl->tx_conn_error_reported = TRUE;
                }
                if (!stale_device &&
                    !error_reported && !ctrl->tx_conn_error_reported)
                {
                    schedule_rig_conn_error(ctrl, ctrl->conf2, _("uplink"));
                    ctrl->tx_conn_error_reported = TRUE;
                }
            }
        }

        if (tx_ok)
        {
            rigctrl_reset_reconnect(ctrl, TRUE);
            tx_opened = TRUE;
            ctrl->conf2->vfo_opt = (ctrl->rig_session2 &&
                                    ctrl->rig_session2->strategy == RIG_STRATEGY_VFO_OPT_ARGS);
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    _("%s:%s: VFO opt2=%d"), __FILE__,
                    __func__, ctrl->conf2->vfo_opt);
        }
    }

    if (ctrl->sock < 0)
    {
        rigctrl_fail_engage(ctrl, "uplink open/probe failed");
        rigctrl_set_conn_state(ctrl, FALSE, RIGCTRL_CONN_DISCONNECTED,
                               "open failed");
        ctrl->opening = FALSE;
        ctrl->opening2 = FALSE;
        return FALSE;
    }

    ctrl->engage_pending = FALSE;
    rigctrl_seed_user_base_from_ui(ctrl, "engage");

    if ((rx_opened || tx_opened))
    {
        /* Refresh Doppler from the current target before the first tune after
         * connect/reconnect so the initial cycle does not reuse a stale zero.
         */
        rigctrl_update_doppler(ctrl);

        if (is_full_duplex_main_sub_configured(ctrl->conf))
        {
            rigctrl_reset_send_tracking(ctrl, TRUE);
            rigctrl_reset_send_tracking(ctrl, FALSE);
        }

        if (ctrl->conf2 != NULL)
        {
            if (ctrl->sock >= 0 && ctrl->sock2 >= 0)
                exec_dual_rig_cycle(ctrl);
        }
        else
        {
            if (is_full_duplex_main_sub_configured(ctrl->conf))
            {
                exec_full_duplex_main_sub_cycle(ctrl, TRUE);
            }
            else
            {
                switch (ctrl->conf->type)
                {

                case RIG_TYPE_RX:
                    exec_rx_cycle(ctrl);
                    break;

                case RIG_TYPE_TX:
                    exec_tx_cycle(ctrl);
                    break;

                case RIG_TYPE_TRX:
                    exec_trx_cycle(ctrl);
                    break;

                case RIG_TYPE_DUPLEX:
                    /* set rig into SAT mode (hamlib needs it even if rig already in SAT) */
                    setup_split(ctrl);
                    exec_duplex_cycle(ctrl);
                    break;

                case RIG_TYPE_TOGGLE_AUTO:
                case RIG_TYPE_TOGGLE_MAN:
                    set_toggle(ctrl, ctrl->sock);
                    ctrl->last_toggle_tx = -1;
                    exec_toggle_cycle(ctrl);
                    break;

                default:
                    /* this is an error! */
                    ctrl->conf->type = RIG_TYPE_RX;
                    exec_rx_cycle(ctrl);
                    break;
                }
            }
        }

        apply_rit_xit_offsets(ctrl, 0.0, 0.0);

        if (ctrl->sock < 0 ||
            ctrl->conn_state == RIGCTRL_CONN_DISCONNECTING ||
            ctrl->conn_state == RIGCTRL_CONN_DISCONNECTED)
        {
            sat_log_log(SAT_LOG_LEVEL_WARN,
                        "%s: receiver socket lost during initial tune cycle",
                        __func__);
            ctrl->opening = FALSE;
            ctrl->opening2 = FALSE;
            return FALSE;
        }
    }

    ctrl->opening = FALSE;
    ctrl->opening2 = FALSE;

    rigctrl_set_conn_state(ctrl, FALSE, RIGCTRL_CONN_CONNECTED, "open ok");
    if (ctrl->conf2 != NULL)
    {
        if (ctrl->sock2 >= 0)
            rigctrl_set_conn_state(ctrl, TRUE, RIGCTRL_CONN_CONNECTED, "open ok");
        else
            rigctrl_set_conn_state(ctrl, TRUE, RIGCTRL_CONN_DISCONNECTED,
                                   "open failed");
    }
    else
    {
        if (is_full_duplex_main_sub_configured(ctrl->conf))
        {
            gboolean tx_ready = (ctrl->rig_session2 != NULL &&
                                 ctrl->rig_session2->state == RIG_SESSION_READY);

            if (!tx_ready)
                rig_term_log(ctrl, "gpredict:err",
                             "rig engage: uplink session not ready (shared rig)");

            rigctrl_set_conn_state(ctrl, TRUE,
                                   tx_ready ? RIGCTRL_CONN_CONNECTED
                                            : RIGCTRL_CONN_DISCONNECTED,
                                   tx_ready ? "shared rig"
                                            : "shared rig unavailable");
        }
        else
        {
            rigctrl_set_conn_state(ctrl, TRUE, RIGCTRL_CONN_DISCONNECTED,
                                   "no uplink");
        }
    }

    if (ctrl->sock >= 0 && ctrl->engaged)
        start_timer(ctrl);

    if (is_full_duplex_main_sub_configured(ctrl->conf))
    {
        vfo_t rx_vfo = rigctrl_target_vfo_for_role(ctrl->conf,
                                                   VFO_ROLE_DOWNLINK);
        vfo_t tx_vfo = rigctrl_target_vfo_for_role(ctrl->conf,
                                                   VFO_ROLE_UPLINK);
        rig_term_log(ctrl, "gpredict",
                     "rig engage: rx_session=%s tx_session=%s mapping: downlink->%s uplink->%s",
                     (ctrl->conn_state == RIGCTRL_CONN_CONNECTED) ? "OK" : "ERR",
                     (ctrl->conn_state2 == RIGCTRL_CONN_CONNECTED) ? "OK" : "ERR",
                     vfo_name(rx_vfo),
                     vfo_name(tx_vfo));
    }

    rigctrl_clear_ui_hard_error(ctrl);
    rigctrl_set_status_detail(ctrl, NULL);
    rigctrl_queue_ui_status_refresh(ctrl, "open complete");
    return TRUE;
}

/* Communication thread for hamlib rigctld */
gpointer rigctl_run(gpointer data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);
    GtkRigCtrl     *t_ctrl = GTK_RIG_CTRL(data);

    while (1)
    {
        t_ctrl = GTK_RIG_CTRL(g_async_queue_pop(ctrl->rigctlq));
        ctrl = t_ctrl;

        if (t_ctrl == NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s:%s: ERROR: NO VALID ctrl-struct"), __FILE__,
                        __func__);
            continue;
        }

        if (t_ctrl->engaged)
        {
            if (t_ctrl->conn_state != RIGCTRL_CONN_CONNECTED ||
                t_ctrl->conf == NULL)
                continue;
        }
        else
        {
            if (t_ctrl->sock != -1 || t_ctrl->sock2 != -1)
                rigctrl_close(t_ctrl);
            break;
        }

        check_aos_los(t_ctrl);

        if (t_ctrl->conf2 != NULL)
        {
            if (t_ctrl->sock2 >= 0)
            {
                exec_dual_rig_cycle(t_ctrl);
            }
            else
            {
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            _("%s: uplink rig not connected; running downlink only"),
                            __func__);
                exec_rx_cycle(t_ctrl);
            }
        }
        else
        {
            if (is_full_duplex_main_sub_configured(t_ctrl->conf))
            {
                exec_full_duplex_main_sub_cycle(t_ctrl, FALSE);
            }
            else
            {
                /* Execute controller cycle depending on primary radio type */
                switch (t_ctrl->conf->type)
                {

                case RIG_TYPE_RX:
                    exec_rx_cycle(t_ctrl);
                    break;

                case RIG_TYPE_TX:
                    exec_tx_cycle(t_ctrl);
                    break;

                case RIG_TYPE_TRX:
                    exec_trx_cycle(t_ctrl);
                    break;

                case RIG_TYPE_DUPLEX:
                    exec_duplex_cycle(t_ctrl);
                    break;

                case RIG_TYPE_TOGGLE_AUTO:
                case RIG_TYPE_TOGGLE_MAN:
                    exec_toggle_cycle(t_ctrl);
                    break;

                default:
                    /* invalid mode */
                    sat_log_log(SAT_LOG_LEVEL_ERROR,
                                _("%s:%s: Invalid radio type %d. Setting type to "
                                  "RIG_TYPE_RX"), __FILE__, __func__,
                                t_ctrl->conf->type);
                    t_ctrl->conf->type = RIG_TYPE_RX;
                }
            }
        }

        update_rit_xit_offsets(t_ctrl);
        rigctrl_log_doppler_tick(t_ctrl);

        /* perform error count checking */
        if (t_ctrl->errcnt >= MAX_ERROR_COUNT)
        {
            t_ctrl->errcnt = 0;
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s:%s: MAX_ERROR_COUNT (%d) reached. Resetting error counter."),
                        __FILE__, __func__, MAX_ERROR_COUNT);
        }

        //g_print ("       WROPS = %d\n", ctrl->wrops);
    }

    if (t_ctrl->sock >= 0 || t_ctrl->sock2 >= 0)
        rigctrl_close(t_ctrl);

    g_mutex_lock(&t_ctrl->widgetsync);
    t_ctrl->rigctl_thread_done = TRUE;
    g_cond_broadcast(&t_ctrl->widgetready);
    g_mutex_unlock(&t_ctrl->widgetsync);

    return NULL;
}

void start_timer(GtkRigCtrl * data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);

    /*  start timeout timer here ("Cycle")! */
    if (ctrl->timerid > 0)
        g_source_remove(ctrl->timerid);

    ctrl->timerid =
        gdk_threads_add_timeout(ctrl->delay, rig_ctrl_timeout_cb, ctrl);
}

void remove_timer(GtkRigCtrl * data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);

    /* stop timer */
    if (ctrl->timerid > 0)
        g_source_remove(ctrl->timerid);
    ctrl->timerid = 0;
}

void setconfig(gpointer data)
{
    /* something has changed... */
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);

    if (ctrl != NULL)
    {
        g_async_queue_push(ctrl->rigctlq, ctrl);
    }
}


GtkWidget      *gtk_rig_ctrl_new(GtkSatModule * module)
{
    GtkRigCtrl     *rigctrl;
    GtkWidget      *widget;
    GtkWidget      *table;
    GtkWidget      *outer;
    GtkWidget      *spacer_top;
    GtkWidget      *spacer_bottom;

    if (!have_conf())
        return NULL;

    widget = g_object_new(GTK_TYPE_RIG_CTRL, NULL);
    rigctrl = GTK_RIG_CTRL(widget);

    g_signal_connect(widget, "key-press-event", G_CALLBACK(key_press_cb),
                     NULL);

    g_hash_table_foreach(module->satellites, store_sats, widget);
    GTK_RIG_CTRL(widget)->target = SAT(g_slist_nth_data(rigctrl->sats, 0));

    rigctrl->qth = module->qth;

    if (rigctrl->target != NULL)
    {
        /* get next pass for target satellite */
        GTK_RIG_CTRL(widget)->pass = get_next_pass(rigctrl->target,
                                                   rigctrl->qth, 3.0);
    }

    /* create contents */
    table = gtk_grid_new();
    gtk_grid_set_row_spacing(GTK_GRID(table), 12);
    gtk_grid_set_column_spacing(GTK_GRID(table), 12);
    gtk_grid_set_column_homogeneous(GTK_GRID(table), TRUE);
    gtk_container_set_border_width(GTK_CONTAINER(table), 10);
    gtk_widget_set_hexpand(table, TRUE);
    gtk_widget_set_vexpand(table, FALSE);
    {
        GtkWidget *down = create_downlink_widgets(rigctrl);
        GtkWidget *up = create_uplink_widgets(rigctrl);
        GtkWidget *target = create_target_widgets(rigctrl);
        GtkWidget *conf = create_conf_widgets(rigctrl);
        GtkWidget *term = gp_term_view_get_widget(rigctrl->term_view);
        GtkWidget *count = create_count_down_widgets(rigctrl);

        gtk_widget_set_hexpand(down, TRUE);
        gtk_widget_set_hexpand(up, TRUE);
        gtk_widget_set_hexpand(target, TRUE);
        gtk_widget_set_hexpand(conf, TRUE);
        gtk_widget_set_hexpand(term, TRUE);
        gtk_widget_set_hexpand(count, TRUE);

        gtk_grid_attach(GTK_GRID(table), down, 0, 0, 1, 1);
        gtk_grid_attach(GTK_GRID(table), up, 1, 0, 1, 1);
        gtk_grid_attach(GTK_GRID(table), target, 0, 1, 1, 1);
        gtk_grid_attach(GTK_GRID(table), conf, 1, 1, 1, 1);
        gtk_grid_attach(GTK_GRID(table), term, 0, 2, 2, 1);
        gtk_grid_attach(GTK_GRID(table), count, 0, 3, 2, 1);
    }

    outer = gtk_grid_new();
    gtk_widget_set_hexpand(outer, TRUE);
    gtk_widget_set_vexpand(outer, TRUE);
    spacer_top = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    spacer_bottom = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    gtk_widget_set_vexpand(spacer_top, TRUE);
    gtk_widget_set_vexpand(spacer_bottom, TRUE);
    gtk_grid_attach(GTK_GRID(outer), spacer_top, 0, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(outer), table, 0, 1, 1, 1);
    gtk_grid_attach(GTK_GRID(outer), spacer_bottom, 0, 2, 1, 1);

    gtk_box_pack_start(GTK_BOX(rigctrl), outer, TRUE, TRUE, 0);

    if (module->target > 0)
        gtk_rig_ctrl_select_sat(rigctrl, module->target);

    return widget;
}
