/*
  Gpredict: Real-time satellite tracking and orbit prediction program

  Copyright (C)  2001-2017  Alexandru Csete, OZ9AEC
  Copyright (C)       2011  Charles Suprin, AA1VS

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
 * Antenna rotator control window.
 *
 * The master rotator control UI is implemented as a Gtk+ Widget in order
 * to allow multiple instances. The widget is created from the module
 * popup menu and each module can have several rotator control windows
 * attached to it. Note, however, that current implementation only
 * allows one rotor control window per module.
 * 
 */

#ifdef HAVE_CONFIG_H
#include <build-config.h>
#endif

#include <glib.h>

/* NETWORK */
#ifdef G_OS_WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>           /* htons(), etc. */
#include <ws2tcpip.h>
#include <windows.h>
#else
#include <arpa/inet.h>          /* htons() */
#include <netdb.h>              /* gethostbyname() */
#include <netinet/in.h>         /* struct sockaddr_in */
#include <sys/socket.h>         /* socket(), connect(), send() */
#include <unistd.h>
#endif

#include <errno.h>
#include <glib/gi18n.h>
#include <gio/gio.h>
#include <gtk/gtk.h>
#include <math.h>
#include <signal.h>
#include <sys/types.h>
#include <time.h>
#if defined(__APPLE__) || defined(__linux__) || defined(__FreeBSD__)
#include <locale.h>
#if defined(__APPLE__)
#include <xlocale.h>
#endif
#endif
#include <string.h>             /* strerror() */
#include <stdarg.h>
#include <stdlib.h>
#include <errno.h>

#include "compat.h"
#include "gpredict-utils.h"
#include "gtk-polar-plot.h"
#include "gtk-rot-knob.h"
#include "gtk-rot-ctrl.h"
#include "gtk-sat-module.h"
#include "gp-term-view.h"
#include "predict-tools.h"
#include "sat-log.h"
#include "rotor-conf.h"
#include "rotor_calibration.h"
#include "rotor-autocal-direct.h"
#include "rotor-cmd-map.h"
#include "rotor-trajectory-planner.h"
#include "rotor-target.h"
#include "rotor-decision.h"
#include "rotor-angle.h"
#include "rot-angle.h"
#include "rot_math.h"
#include "azel_mapping.h"
#include "tracking_policy.h"
#include "safety_window.h"
#include "status_indicator.h"
#include "ui-popup-quarantine.h"
#include "ui-status.h"
#include "rotctld_mgr.h"
#include "rotctld-parse.h"
#include "rotctld_client.h"
#include "serial-ports.h"

#ifndef G_SUBPROCESS_FLAGS_STDIN_DEV_NULL
#ifdef G_SUBPROCESS_FLAGS_STDIN_INHERIT
#define G_SUBPROCESS_FLAGS_STDIN_DEV_NULL G_SUBPROCESS_FLAGS_STDIN_INHERIT
#else
#define G_SUBPROCESS_FLAGS_STDIN_DEV_NULL 0
#endif
#endif


#define FMTSTR "%7.2f\302\260"
#define MAX_ERROR_COUNT 5
#define ROT_PLAN_SAMPLE_DT_SEC 1.0
#define ROT_POLICY_MAX_SAMPLES 200
#define ROT_PLAN_EL_WEIGHT 1.0
#define ROT_PLAN_NEAR_LIMIT_MARGIN 2.0
#define ROT_PLAN_NEAR_LIMIT_PENALTY 5.0
#define ROT_PRETRACK_LOOKAHEAD_SEC 7200.0
#define ROT_PRETRACK_REACQUIRE_SEC 2.0
#define ROT_PRETRACK_WINDOW_SEC 300.0
#define ROT_PRETRACK_AOS_OFFSET_SEC 0.5
#define ROT_PRETRACK_RECALC_US (2 * G_USEC_PER_SEC)
#define ROT_PRETRACK_SEARCH_STEP_SEC 2.0
#define ROT_PRETRACK_REFINE_SEC 0.5
#define ROT_PRETRACK_HYSTERESIS_DEG 0.5
#define ROT_PRETRACK_LIVE_FALLBACK_MARGIN_DEG 2.0
#define ROT_TRACK_LOOKAHEAD_SEC 2.0
#define ROT_CMD_MIN_AZ_DEG 0.8
#define ROT_CMD_MIN_EL_DEG 0.5
#define ROT_CMD_RESEND_MS 3000
#define ROT_CMD_DEADBAND_AZ_DEG 1.0
#define ROT_CMD_DEADBAND_EL_DEG 0.7
#define ROT_CMD_DEADBAND_PRETRACK_AZ_DEG 2.0
#define ROT_CMD_DEADBAND_PRETRACK_EL_DEG 1.0
#define ROT_CMD_COOLDOWN_MS 1500
#define ROT_CMD_MAX_HZ 1.0
#define ROT_CMD_POS_FRESH_MS 1500
#define ROT_CMD_AZ_EPS_MIN_DEG 1.5
#define ROT_CMD_MIN_PERIOD_MS 250
#define ROT_CMD_MAX_SILENCE_MS 3000
#define ROT_CMD_POS_TIMEOUT_MS 1200
#define ROT_CMD_STUCK_TIMEOUT_MS 2000
#define ROT_CMD_MOVE_GRACE_MS 500
#define ROT_CMD_DISCREPANCY_STRIKES 3
#define ROT_STALE_ENTER_MS 7000
#define ROT_STALE_EXIT_MS 2500
#define ROT_STALE_WARN_CYCLES 3
#define ROT_STALE_DEGRADED_CYCLES 4
#define ROT_STALE_DEGRADED_CONFIRM_POLLS 1
#define ROT_STALE_READY_CONFIRM_POLLS 2
#define ROT_UI_DEGRADED_CLEAR_POLLS 4
#define ROT_COMM_DEGRADED_FAIL_POLLS 4
#define ROT_COMM_LINK_LOST_EXTRA_FAIL_POLLS 4
#define ROT_COMM_LINK_LOST_FAIL_POLLS \
    (ROT_COMM_DEGRADED_FAIL_POLLS + ROT_COMM_LINK_LOST_EXTRA_FAIL_POLLS)
#define ROT_TRACK_LEAD_SEC 0.3
#define ROT_AZ_OOB_PENALTY 10000.0

#define ROT_PRESET_MAX 8
#define ROT_PRESET_ROWS 2
#define ROT_PRESET_COLS 4
#define ROT_PRESET_GROUP "rotor_presets"
#define ROT_PRESET_COUNT_KEY "count"
#define ROT_PRESET_RESPONSE_DELETE 1001
#define ROT_PRESET_BUTTON_HEIGHT 44
#define ROTCTRL_PANEL_SPACING 12
#define ROT_AZ_ENDSTOP_PENALTY 1000.0
#define ROT_BELOW_HORIZON_MARGIN_DEG 0.5
#define ROT_AUTOCAL_TIMEOUT_US (45 * G_USEC_PER_SEC)
#define ROT_AUTOCAL_STABLE_COUNT 5
#define ROT_AUTOCAL_LOG_INTERVAL_US 2000000
#define ROT_AUTOCAL_STABLE_WINDOW 10
#define ROT_AUTOCAL_STABLE_EPS_DEG 1.5
#define ROT_AUTOCAL_SEAM_TOL_DEG 2.0
#define ROT_AUTOCAL_ARRIVE_TOL_AZ_DEG 2.0
#define ROT_AUTOCAL_ARRIVE_TOL_EL_DEG 2.0
#define ROT_AUTOCAL_ARRIVE_COUNT 3
#define ROT_AUTOCAL_READY_COUNT 2
#define ROT_AUTOCAL_SETTLE_MAX_MS 5000
#define ROT_AUTOCAL_POLL_MS 200
#define ROT_AUTOCAL_MOVE_INTERVAL_MS 500
#define ROT_AUTOCAL_NO_MOTION_MS 4000
#define ROT_AUTOCAL_NO_POS_MS 4000
#define ROT_FREEZE_MIN_WAIT_MS 250
#define ROT_FREEZE_MAX_WAIT_MS 1200
#define ROT_FREEZE_POLL_SLICE_US 20000

static const gdouble k_meas_quantum_deg = 1.0;
static const gdouble k_meas_tol_deg = 1.5;
#define ROTCTLD_SOCKET_TIMEOUT_MS 1000
#define ROTCTLD_IDLE_TIMEOUT_MS 100
#define ROTCTLD_FOLLOW_IDLE_MS 50
#define ROTCTLD_HANDSHAKE_RETRY_MS 250
#define ROTCTLD_BASELINE_POLL_MS 200
#define ROTCTLD_BASELINE_TIMEOUT_MS 1200
#define ROTCTLD_BASELINE_DEADBAND_DEG 1.0
#define ROTCTLD_IO_STALE_MS 2000
#define ROTCTLD_RECONNECT_MAX_RETRIES 5
#define ROTCTLD_MAX_CONSEC_FAIL 3
#define ROTCTLD_FAILURE_LOG_INTERVAL_US 2000000
#define ROTCTLD_POS_UNKNOWN_BACKOFF_US 2000000
#define ROTCTLD_POS_RETRY_COUNT 3
#define ROTCTLD_POS_RETRY_DELAY_MS 50
#define ROTCTLD_POS_MAX_FAIL 5
#define ROTCTLD_DEFAULT_POLL_PERIOD_MS 1000
#define ROTCTLD_DEFAULT_STALE_MS 6000
#define ROTCTLD_DEFAULT_STALE_DEBOUNCE 2
#define ROTCTLD_DEFAULT_ANGLE_EPS_DEG 1.5
#define ROTCTLD_DEFAULT_ELEV_FLOOR_DEG 1.0
#define ROTCTLD_HANDSHAKE_GRACE_MS 2000
#define ROTCTLD_FIRST_POS_GRACE_MS 5000
#define ROTCTLD_POS_VALID_WINDOW_MS 4000
#define ROTCTLD_AUTODETECT_TICK_MS 50
#define ROTCTLD_AUTODETECT_TCP_READY_TIMEOUT_MS 800
#define ROTCTLD_AUTODETECT_SPAWN_SETTLE_MS 100
#define ROTCTLD_AUTODETECT_TCP_SETTLE_MS 80
#define ROTCTLD_AUTODETECT_VALIDATE_TIMEOUT_MS 300
#define ROTCTLD_AUTODETECT_VALIDATE_RETRIES 0
#define ROTCTLD_AUTODETECT_VALIDATE_RETRY_DELAY_MS 0
#define ROTCTLD_AUTODETECT_PORT_FREE_TIMEOUT_MS 300
#define ROTCTLD_AUTODETECT_COOLDOWN_MS 150
#define ROTCTLD_AUTODETECT_LASTGOOD_WINDOW_MS 1000
#define ROTCTLD_AUTODETECT_MAX_CANDIDATES 5
#define ROTCTLD_AUTODETECT_ROT_TIMEOUT_MS 3000
#define ROTCTLD_AUTODETECT_VALIDATE_MARGIN_MS 500
#define ROTCTLD_BACKEND_TIMEOUT_MS 1200
#define ROTCTLD_WIN32_ROT2PROG_TIMEOUT_MS 6000
#define ROTCTLD_WIN32_ROT2PROG_MARGIN_MS 1500
#define ROTCTRL_DEFAULT_MIN_EL -5.0
#define ROTCTRL_DEFAULT_MAX_EL 185.0
#define ROTCTLD_KEEPALIVE_US 2000000
#define ROT_SEND_MIN_INTERVAL_US ((gint64)ROT_CMD_MIN_PERIOD_MS * 1000)
#define ROT_TRACK_RESEND_US 2000000
#define ROT_TRACK_STALL_MS 1500
#define ROT_LANE_ENDSTOP_MARGIN_DEG 5.0
#define ROT_LANE_SWITCH_PENALTY_DEG 30.0
#define ROTCTRL_MAX_BACKEND_IOERR 5
#define ROTCTRL_BACKEND_IO_WINDOW_US (10 * G_USEC_PER_SEC)
#define ROTCTRL_AOSLOS_PLACEHOLDER "<span size='xx-large'><b>--</b></span>"

static void gp_safe_label_set_markup(GtkWidget *w, const gchar *markup)
{
    if (w == NULL)
        return;

    if (GTK_IS_LABEL(w))
        gtk_label_set_markup(GTK_LABEL(w), markup ? markup : "");
    else
        g_warning("Expected GtkLabel, got %s", G_OBJECT_TYPE_NAME(w));
}

typedef struct {
    GThread        *thread;
    GMutex          mutex;
    gint            socket;
    guint64         conn_id;
    guint64         thread_generation;
    RotctldClient  *client;
    gboolean        running, new_trg;
    gboolean        use_setpos;
    gboolean        apply_calib;
    gboolean        allow_send_no_pos;
    gboolean        stop_pending;
    gint            stop_requested;
    gboolean        send_quit;
    gboolean        thread_done;
    gdouble         azi_out, ele_out;
    gdouble         raw_azi_out, raw_ele_out;
    gdouble         azi_in, ele_in;
    gdouble         azi_az360;
    gdouble         azi_mech_in, ele_mech_in;
    gboolean        io_error;
    gboolean        cmd_rejected;
    gint64          reject_backoff_until_us;
    gdouble         reject_backoff_sec;
    gint64          reject_backoff_log_us;
    gboolean        limits_valid;
    gdouble         az_min, az_max;
    gdouble         el_min, el_max;
    gboolean        south_zero;
    gboolean        daemon_ok;
    GTimer         *timer;
    GString        *rxbuf;
    gint64          last_rtt_us;
    gint64          last_cmd_us;
    gdouble         last_cmd_ok_az;
    gdouble         last_cmd_ok_el;
    gint64          last_cmd_ok_us;
    gdouble         last_cmd_backend_az;
    gdouble         last_cmd_backend_el;
    gboolean        last_cmd_backend_valid;
    gdouble         desired_user_az;
    gdouble         desired_user_el;
    gdouble         desired_backend_az;
    gdouble         desired_backend_el;
    gdouble         desired_raw_az;
    gdouble         desired_raw_el;
    gboolean        desired_valid;
    gboolean        desired_allow;
    gboolean        desired_tracking;
    gboolean        force_pending;
    gint64          desired_update_us;
    gdouble         last_err_deg;
    gint64          last_err_us;
    gint64          stall_since_us;
    gdouble         last_pos_user_az;
    gdouble         last_pos_user_el;
    gint64          last_pos_sample_us;
    gdouble         last_err_to_cmd;
    gint64          last_err_time_us;
    gint64          move_grace_until_us;
    guint           discrepancy_streak;
    guint           wrong_way_count;
    gboolean        discrepancy_pending;
    gboolean        feedback_disabled_logged;
    gboolean        pos_valid;
    gboolean        pos_invalid;
    gboolean        pos_unknown;
    gboolean        pos_cmd_ok;
    gboolean        set_pos_ok;
    gint64          last_pos_us;
    gint64          last_pos_attempt_us;
    gint64          last_set_attempt_us;
    gboolean        handshake_pos_ok;
    gint64          first_pos_deadline_us;
    gint64          transport_backoff_until_us;
    gdouble         transport_backoff_sec;
    gchar           io_error_reason[64];
    gchar           last_pos_error[64];
    gint64          last_io_ok_us;
    guint           consecutive_failures;
    gint64          last_failure_log_us;
    guint           reconnect_failures;
    gboolean        reconnect_degraded;
    guint           pos_failures;
    gint64          pos_backoff_until_us;
    gdouble         pos_backoff_sec;
    gboolean        pos_degraded;
    guint           backend_ioerr_count;
    gint64          backend_ioerr_first_us;
    gint64          backend_ioerr_last_log_us;
    gboolean        backend_ioerr_disengage_pending;
} rotctld_client_t;

typedef struct RotctldProbeState RotctldProbeState;

typedef enum {
    ROTCTLD_PROBE_OK_ROTCTLD = 0,
    ROTCTLD_PROBE_NOT_READY,
    ROTCTLD_PROBE_NOT_ROTCTLD
} rotctld_probe_result_t;

typedef enum {
    ROTCTLD_ENSURE_READY = 0,
    ROTCTLD_ENSURE_PENDING,
    ROTCTLD_ENSURE_FAILED
} rotctld_ensure_result_t;

typedef enum {
    ROTCTLD_AUTODETECT_IDLE = 0,
    ROTCTLD_AUTODETECT_START_CHILD,
    ROTCTLD_AUTODETECT_SETTLE,
    ROTCTLD_AUTODETECT_WAIT_TCP_READY,
    ROTCTLD_AUTODETECT_VALIDATE_IO,
    ROTCTLD_AUTODETECT_SUCCESS,
    ROTCTLD_AUTODETECT_FAIL_NEXT,
    ROTCTLD_AUTODETECT_ABORT
} rotctld_autodetect_state_t;

typedef enum {
    ROTCTLD_AUTODETECT_STEP_CONTINUE = 0,
    ROTCTLD_AUTODETECT_STEP_SUCCESS,
    ROTCTLD_AUTODETECT_STEP_FAIL
} rotctld_autodetect_step_t;

typedef enum {
    ROT_SESSION_DISCONNECTED = 0,
    ROT_SESSION_CONNECTING,
    ROT_SESSION_ENGAGING,
    ROT_SESSION_READY,
    ROT_SESSION_DEGRADED
} rot_session_state_t;

typedef enum {
    ROT_COMM_DISENGAGED = 0,
    ROT_COMM_STANDBY,
    ROT_COMM_MOVING,
    ROT_COMM_ON_TARGET,
    ROT_COMM_DEGRADED,
    ROT_COMM_LINK_LOST
} rot_comm_status_t;

typedef enum {
    ROT_PLAN_MODE_NORMAL = 0,
    ROT_PLAN_MODE_FLIP   = 1
} rot_plan_mode_t;

typedef enum {
    ROT_TARGET_STATE_IDLE = 0,
    ROT_TARGET_STATE_HOLD,
    ROT_TARGET_STATE_PRETRACK,
    ROT_TARGET_STATE_TRACKING_NORMAL,
    ROT_TARGET_STATE_TRACKING_DEGRADED
} rot_target_state_t;

typedef enum {
    ROT_AUTOCAL_IDLE = 0,
    ROT_AUTOCAL_DRIVE_ZERO,
    ROT_AUTOCAL_SETTLE,
    ROT_AUTOCAL_READY,
    ROT_AUTOCAL_APPLYING,
    ROT_AUTOCAL_DONE,
    ROT_AUTOCAL_FAIL
} rot_autocal_state_t;

typedef struct {
    gchar *name;
    gdouble az;
    gdouble el;
} RotPreset;

typedef struct {
    gboolean        valid;
    rot_plan_mode_t mode;
    gboolean        crosses_endstop;
    rot_plan_status_t status;
    rot_plan_strategy_t strategy;
    gdouble         start_az_cmd;
    gdouble         start_el_cmd;
    gdouble         end_az_cmd;
    gdouble         end_el_cmd;
    gdouble         total_motion;
    gdouble         peak_az_rate;
    gdouble         peak_el_rate;
    gdouble         window_start;
    gdouble         window_end;
    gdouble         trackable_pct;
    gdouble         violation_mag;
    gdouble         sample_dt_sec;
    GArray         *cmd_samples; /* rot_plan_cmd_t */
    gchar          *reason;
} rot_plan_t;

typedef struct {
    gdouble         az_offset_deg;
    gdouble         el_offset_deg;
    gboolean        az_invert;
    gboolean        el_invert;
    gdouble         az_min_deg;
    gdouble         az_max_deg;
    gdouble         el_min_deg;
    gdouble         el_max_deg;
    rot_az_type_t   az_norm_mode;
    gboolean        zenith_guard_enable;
    gdouble         zenith_guard_el_deg;
    gboolean        south_zero;
} RotTransformSnapshot;

typedef struct {
    RotTransformSnapshot transform;
    guint64              version;
    gboolean             ready;
} RotTransformSnapshotState;

typedef struct {
    gint64          last_log_us;
    guint           suppressed;
} RotLogRate;

typedef struct {
    gboolean             valid;
    gdouble              az_min;
    gdouble              az_max;
    gdouble              el_min;
    gdouble              el_max;
    rot_target_wrap_mode_t wrap_mode;
    gboolean             south_zero;
} RotLimitSet;

typedef struct {
    gdouble         az_abs;
    gdouble         el_raw;
} RotTransformPre;

struct _GtkRotCtrl {
    GtkBox          box;

    GtkWidget      *AzSet, *AzRead;
    GtkWidget      *ElSet, *ElRead;
    GtkWidget      *SatSel, *AzSat, *ElSat, *SatCnt;
    GtkWidget      *aoslos_banner;
    GtkWidget      *DevSel, *LockBut, *MonitorCheckBox;
    GtkWidget      *track, *freeze, *cycle_spin, *thld_spin;
    GtkWidget      *plot;
    GtkWidget      *axis_mode_combo;
    GtkWidget      *wrap_mode_combo;
    GtkWidget      *min_az_spin;
    GtkWidget      *max_az_spin;
    GtkWidget      *min_el_spin;
    GtkWidget      *max_el_spin;
    GtkWidget      *az_endstop_spin;
    GtkWidget      *preset_grid;
    GList          *detached_plots;

    RotPreset       presets[ROT_PRESET_MAX];
    guint           preset_count;

    GSList         *sats;
    sat_t          *target;
    pass_t         *pass;
    qth_t          *qth;
    GtkSatModule   *module;
    GHashTable     *module_sats;

    guint           delay, timerid;
    gdouble         threshold, t;
    gint            errcnt;
    gboolean        cycle_perf_warned;

    gboolean        tracking, engaged, monitor, flipped;
    gboolean        engage_pending;
    guint64         engage_generation;
    rot_session_state_t session_state;
    gboolean        tracking_active;
    gdouble         tracking_session_los;
    gboolean        force_next_send;
    gboolean        hold_position_on_engage;
    gboolean        hold_position_log_emitted;
    gboolean        have_user_command;
    rot_comm_status_t comm_status;
    RotorUiStatus   ui_status;
    gboolean        ui_degraded_latched;
    guint           ui_degraded_clear_hits;
    gboolean        ui_hard_error;
    gchar           ui_hard_error_reason[128];
    gchar           ui_status_detail[160];
    guint           ok_streak;
    guint           bad_streak;
    guint           timeout_streak;
    rot_target_state_t target_state;
    gint64          target_state_since_us;
    gint64          target_valid_since_us;
    gint64          target_invalid_since_us;
    gint64          last_target_update_us;
    gint64          park_pending_since_us;
    gboolean        park_requested;
    gboolean        pretrack_enabled;
    gdouble         pretrack_lookahead_sec;
    gdouble         reacquire_hysteresis_sec;
    gboolean        pretrack_immediate;
    gdouble         pretrack_min_el;
    gdouble         pretrack_target_az;
    gdouble         pretrack_target_el;
    gdouble         pretrack_aos_time;
    gint64          pretrack_last_update_us;
    gboolean        pretrack_target_valid;
    gint64          pretrack_wait_log_us;
    gboolean        pretrack_wrap_valid;
    gdouble         pretrack_wrap_user_az;
    gdouble         pretrack_wrap_raw_az;
    gint            pretrack_wrap_k;
    gdouble         seam_az360;
    gboolean        seam_valid;
    gboolean        seam_crossing_active;
    gboolean        seam_crossing_sent;
    gboolean        seam_crossing_lane_valid;
    gint            seam_crossing_lane_k;
    gdouble         seam_crossing_target_az360;
    gint64          seam_crossing_since_us;
    gboolean        wrap_acquire_active;
    gboolean        wrap_acquire_sent;
    gdouble         wrap_acquire_target_backend;
    gdouble         wrap_acquire_target_az360;
    gint            wrap_acquire_target_k;
    gint64          wrap_acquire_since_us;
    gint64          wrap_acquire_last_log_us;
    gdouble         last_target_az360;
    gdouble         last_target_el;
    gboolean        last_target_valid;
    gdouble         pending_lane_backend;
    gboolean        pending_lane_valid;
    gint64          pending_lane_since_us;
    gint            locked_lane_k;
    gboolean        locked_lane_valid;
    gdouble         last_cmd_az360;
    gdouble         last_cmd_el;
    gboolean        last_cmd_valid;
    gdouble         last_cmd_backend_az;
    gdouble         last_cmd_backend_el;
    gboolean        last_cmd_backend_valid;
    gdouble         setpoint_user_az;
    gdouble         setpoint_user_el;
    gdouble         setpoint_backend_az;
    gdouble         setpoint_backend_el;
    gboolean        setpoint_valid;
    gdouble         committed_user_az;
    gdouble         committed_user_el;
    gdouble         committed_raw_az360;
    gdouble         committed_raw_el;
    gdouble         committed_backend_az;
    gdouble         committed_backend_el;
    gboolean        committed_valid;
    gint64          committed_since_us;
    gint64          last_keepalive_time_us;
    gint64          last_desired_update_us;
    gint64          last_send_us;
    gint            above_eps_count;
    gint64          last_hold_log_us;
    gint64          last_stale_check_log_us;
    double          az_abs_cur;
    double          az_abs_last_cmd;
    double          last_meas_span_az;
    gint64          last_cmd_time_us;
    AzSpan          span_mode;
    gboolean        span_extended;
    RotorSafety     safety;
    TrackPolicy     policy;
    RotTrackingPolicy track_policy;

    rotor_conf_t   *conf;
    rotctld_client_t client;
    rot_plan_t      trajectory_plan;
    gboolean        plan_log_pending;
    gdouble         plan_log_window_start;
    gdouble         plan_log_window_end;
    rot_plan_mode_t plan_log_mode;
    gboolean        plan_log_crosses_endstop;
    gint64          last_tracking_log_us;
    rot_plan_mode_t last_tracking_log_mode;
    gboolean        last_tracking_log_cross_endstop;
    gchar           last_tracking_log_entry_reason[16];
    guint64         diagnostics_logged_generation;

    GpTermView     *term_view;
    GtkWidget      *log_toggle;
    gint            log_closed_height;
    gboolean        ui_updating;
    guint           pending_ui_refresh_id;
    guint           resize_idle_id;
    RotctldMgr     *rotctld_mgr;
    gboolean        verbose_logging;
    gint            selected_child_pid;
    gint            selected_child_port;
    gint            last_stop_pid;
    gchar          *last_stop_reason;
    gint64          last_stop_us;
    RotctldProbeState *rotctld_probe_state;
    guint           rotctld_probe_id;

    gboolean        use_offset;
    gdouble         az_offset_deg, el_offset_deg;
    GtkWidget      *offset_check;
    GtkWidget      *az_offset_spin;
    GtkWidget      *el_offset_spin;
    gchar          *rotor_id;
    RotorCalib      calib;
    gdouble         calibration_offset_az;
    gdouble         calibration_offset_el;
    GtkWidget      *settings_window;
    GtkWidget      *chk_calib_enabled;
    GtkWidget      *calib_offsets_grid;
    GtkWidget      *calib_autocal_button;
    GtkWidget      *spin_az_off;
    GtkWidget      *spin_el_off;
    bool          (*backend_setpos)(double az, double el);
    bool          (*backend_getpos)(double *az, double *el);
    RotTransformSnapshot transform;
    GMutex          transform_snapshot_mutex;
    RotTransformSnapshotState transform_snapshot;
    guint64         transform_snapshot_version;
    gint            south_zero_cached;
    gboolean        az_hold_active;
    gdouble         az_hold_value;
    gint64          last_debug_log_us;
    gint64          last_tick_log_us;
    gboolean        axis_swap_warned;
    RotLimitSet     user_limits;
    RotLimitSet     backend_limits;
    gboolean        limits_logged;
    gboolean        wrap_mismatch_logged;
    RotLogRate      backend_clamp_rate;
    RotLogRate      send_log_rate;
    RotLogRate      cmd_skip_rate;
    RotLogRate      pos_stale_rate;
    RotLogRate      pos_warn_rate;
    RotLogRate      no_pos_gate_rate;
    guint           pos_stale_hits;
    gboolean        pos_stale_active;
    gboolean        pos_stale_hyst_active;
    guint           pos_stale_ready_hits;
    gboolean        stale_hold_active;
    gint64          stale_hold_since_us;
    gint64          stale_resume_since_us;
    gboolean        stale_recovered_pulse;

    /* Reserved flag; currently always kept FALSE (no special SEND-ONLY mode). */
    gboolean        send_only_mode;

    gboolean        out_of_range;
    gint64          last_oob_log_us;
    gdouble         last_oob_raw_az, last_oob_raw_el;
    gdouble         last_oob_mapped_az, last_oob_mapped_el;
    gint64          manual_edit_until_us;
    gint64          last_manual_sync_log_us;
    gboolean        manual_sync_pending;
    guint           calibration_retry_id;
    gboolean        calibration_pending;
    gboolean        cal_active;
    gboolean        calibration_active;
    rot_autocal_state_t cal_state;
    gboolean        cal_did_setpos;
    gboolean        cal_direct_active;
    GtkWidget      *calib_dialog;
    RotorCalib      calib_backup;
    gboolean        calib_backup_valid;
    gboolean        cal_hold_active;
    gint64          cal_hold_since_us;
    gint64          cal_start_us;
    gint64          cal_timeout_us;
    RotAutocalDirectState cal_direct;
    guint           cal_stable_count;
    guint           cal_arrive_count;
    guint           cal_ready_count;
    gint64          cal_last_log_us;
    gint64          cal_settle_start_us;
    gdouble         cal_window_az[ROT_AUTOCAL_STABLE_WINDOW];
    gdouble         cal_window_el[ROT_AUTOCAL_STABLE_WINDOW];
    guint           cal_window_count;
    guint           cal_window_idx;
    gdouble         motion_err_mag;
    gboolean        motion_err_valid;
    guint           motion_stall_count;
};

struct _GtkRotCtrlClass {
    GtkBoxClass     parent_class;
};


static GtkVBoxClass *parent_class = NULL;
static guint64 rotctld_conn_seq = 0;

typedef struct {
    GtkRotCtrl     *ctrl;
    GtkWidget      *window;
    GtkWidget      *plot;
    GtkWidget      *aoslos_label;
} RotDetachedPlot;

/* Forward declaration for error dialog helper */

static void rot_show_no_rotor_dialog(GtkRotCtrl *ctrl);
static void rot_show_conf_error(GtkRotCtrl *ctrl, const gchar *reason);
static void rot_show_plan_error(GtkRotCtrl *ctrl, const gchar *reason);
static void rot_show_message(GtkRotCtrl *ctrl,
                             GtkMessageType type,
                             const gchar *title,
                             const gchar *message);
static void rot_logs_toggle_cb(GtkToggleButton *button, gpointer data);
static void rot_verbose_cb(GtkToggleButton *button, gpointer data);
static void rotctrl_set_cal_hold(GtkRotCtrl *ctrl,
                                 gboolean active,
                                 const gchar *reason);
static gboolean rotor_apply_ui_settings(GtkRotCtrl *ctrl, gboolean strict);
static gboolean rotctrl_settings_focus_out_cb(GtkWidget *widget,
                                              GdkEventFocus *event,
                                              gpointer data);
static void     rotctrl_settings_activate_cb(GtkEntry *entry, gpointer data);
static void     rotctrl_presets_clear(GtkRotCtrl *ctrl);
static gboolean rotctrl_presets_load(GtkRotCtrl *ctrl);
static gboolean rotctrl_presets_save(GtkRotCtrl *ctrl);
static void     rotctrl_presets_refresh(GtkRotCtrl *ctrl);
static void     rotctrl_preset_create_cb(GtkButton *button, gpointer data);
static void     rotctrl_preset_edit_cb(GtkButton *button, gpointer data);
static void     rotctrl_preset_activate_cb(GtkButton *button, gpointer data);
static void     rotctrl_detach_plot_cb(GtkButton *button, gpointer data);
static void rotctrl_ui_begin_update(GtkRotCtrl *ctrl, const gchar *reason);
static void rotctrl_ui_end_update(GtkRotCtrl *ctrl, const gchar *reason);
static void rotctrl_capture_log_closed_height(GtkRotCtrl *ctrl);
static void rotctrl_restore_log_height(GtkRotCtrl *ctrl);
static void rotctrl_free_sat_list_entry(gpointer data, gpointer user_data);
static void     rotctrl_set_pass_on_plots(GtkRotCtrl *ctrl, pass_t *pass);
static void     rotctrl_set_rotor_pos_on_plots(GtkRotCtrl *ctrl,
                                               gdouble az, gdouble el);
static void     rotctrl_set_target_pos_on_plots(GtkRotCtrl *ctrl,
                                                gdouble az, gdouble el);
static void     rotctrl_set_ctrl_pos_on_plots(GtkRotCtrl *ctrl,
                                              gdouble az, gdouble el);
static void     rotctrl_queue_draw_plots(GtkRotCtrl *ctrl);
static void     rotctrl_set_ui_hard_error(GtkRotCtrl *ctrl,
                                          const gchar *reason);
static void     rotctrl_clear_ui_hard_error(GtkRotCtrl *ctrl);
static gboolean rotctrl_sync_sat_copy_from_module(GtkRotCtrl *ctrl,
                                                  sat_t *dst);
static void     rotctrl_set_ui_status_detail(GtkRotCtrl *ctrl,
                                             const gchar *detail);
static void     rotctrl_apply_ui_status(GtkRotCtrl *ctrl,
                                        const RotorStateSnapshot *snap,
                                        const gchar *reason);
static void     rotctrl_finish_tracking_pass_over(GtkRotCtrl *ctrl,
                                                  const gchar *reason);
static gboolean rotctrl_target_display_valid(const sat_t *sat);
static void     rotctrl_update_detached_labels(GtkRotCtrl *ctrl);
static void rot_session_set_state(GtkRotCtrl *ctrl,
                                  rot_session_state_t state,
                                  const gchar *reason,
                                  gboolean send_quit);
static void rotctrl_force_toplevel_resize(GtkRotCtrl *ctrl);
static gboolean rotctrl_resize_idle(gpointer data);
static void rotctrl_schedule_resize(GtkRotCtrl *ctrl);
static void rot_schedule_wrong_daemon(GtkRotCtrl *ctrl,
                                      const gchar *host, gint port);
static void rot_schedule_backend_io_disengage(GtkRotCtrl *ctrl);
static void rot_schedule_cmd_reject(GtkRotCtrl *ctrl, const gchar *reason);
static void G_GNUC_UNUSED rot_log_out_of_range(GtkRotCtrl *ctrl,
                                               gdouble raw_az, gdouble raw_el,
                                               const rot_cmd_map_t *map);
static void     rotctld_clear_rxbuf(GtkRotCtrl *ctrl);
static gboolean rot_manual_input_event(GtkWidget *widget, GdkEvent *event,
                                       gpointer data);
static gboolean rotctrl_should_sync_manual(GtkRotCtrl *ctrl);
static gboolean rotctrl_sync_manual_from_position(GtkRotCtrl *ctrl,
                                                  gdouble az_abs, gdouble el,
                                                  gdouble *setaz,
                                                  gdouble *setel);
static AzSpan   rotctrl_span_from_conf(const rotor_conf_t *conf);
static void     rotctrl_span_bounds(AzSpan span,
                                    gdouble *min_out,
                                    gdouble *max_out);
static SpanConfig rotctrl_span_from_limits(const rotor_conf_t *conf,
                                           gboolean caps_valid,
                                           gdouble caps_az_min,
                                           gdouble caps_az_max,
                                           gdouble caps_el_min,
                                           gdouble caps_el_max,
                                           gboolean *extended_out);
static gboolean G_GNUC_UNUSED rotctrl_caps_match_conf(const rotor_conf_t *conf,
                                                      gdouble caps_az_min,
                                                      gdouble caps_az_max,
                                                      gdouble caps_el_min,
                                                      gdouble caps_el_max);
static gdouble  rotctrl_normalize_az_to_limits(gdouble az,
                                               gdouble min,
                                               gdouble max);
static gdouble  rotctrl_normalize_backend_az(gdouble az,
                                             gdouble backend_min_az,
                                             gdouble backend_max_az);
static void     rotctrl_prefer_user_backend_span(const GtkRotCtrl *ctrl,
                                                 gdouble *backend_min,
                                                 gdouble *backend_max);
static gint     rotctrl_poll_period_ms(const GtkRotCtrl *ctrl);
static gint     rotctrl_getpos_timeout_ms(const GtkRotCtrl *ctrl);
static gint     rotctrl_stale_warn_ms(const GtkRotCtrl *ctrl);
static gint     rotctrl_stale_degraded_ms(const GtkRotCtrl *ctrl);
static gint     rotctrl_stale_hold_ms(const GtkRotCtrl *ctrl);
static gint     rotctrl_stale_park_ms(const GtkRotCtrl *ctrl);
static gint     rotctrl_stale_resume_ms(const GtkRotCtrl *ctrl);
static gint     rotctrl_stale_ms(const GtkRotCtrl *ctrl);
static gint     rotctrl_plot_stale_hide_ms(const GtkRotCtrl *ctrl);
static gdouble  rotctrl_angle_epsilon(const GtkRotCtrl *ctrl);
static gdouble  rotctrl_elev_floor(const GtkRotCtrl *ctrl);
static gdouble  rotctrl_apply_el_overtravel(GtkRotCtrl *ctrl,
                                            gdouble target_el);
static gdouble  rotctrl_clamp_el_for_backend(GtkRotCtrl *ctrl,
                                             gdouble target_el,
                                             gdouble backend_min_el,
                                             gdouble backend_max_el,
                                             gboolean *clamped_out);
static gboolean rot_should_send(GtkRotCtrl *ctrl,
                                gdouble meas_az360,
                                gdouble meas_el,
                                gdouble tgt_az360,
                                gdouble tgt_el,
                                gdouble eps_az,
                                gdouble eps_el,
                                gboolean periodic_due,
                                gboolean not_at_target,
                                gint64 now_us,
                                gint64 min_interval_us,
                                const gchar **reason_out);
static gboolean rotctrl_stale_suppressed(const GtkRotCtrl *ctrl);
static AzSpan   rotctrl_span_from_az_type(rot_az_type_t aztype);
static void     normalize_and_clamp_target(const SpanConfig *span,
                                           gdouble az_deg,
                                           gdouble el_deg,
                                           gdouble current_az,
                                           gboolean have_current,
                                           AzElMapResult *out);
static void     rotctrl_tracking_policy_reset_reason(GtkRotCtrl *ctrl,
                                                     const gchar *reason);
static void G_GNUC_UNUSED rotctrl_tracking_policy_choose(GtkRotCtrl *ctrl,
                                                         gdouble az_min,
                                                         gdouble az_max,
                                                         gdouble el_min,
                                                         gdouble el_max,
                                                         gboolean allow_flip);
static gboolean G_GNUC_UNUSED rotctrl_tracking_policy_select(GtkRotCtrl *ctrl,
                                                             gdouble az_pred,
                                                             gdouble el_pred,
                                                             gboolean have_measured,
                                                             gdouble measured_az,
                                                             gdouble last_cmd_az,
                                                             gdouble *az_cmd_out,
                                                             gdouble *el_cmd_out,
                                                             gboolean *empty_set_out);
static gboolean rotctrl_stop_crosses(const RotorSafety *s,
                                     double az_abs_cur,
                                     double az_abs_candidate);
static double   rotctrl_az_abs_error(double measured_abs,
                                     double desired_backend,
                                     AzSpan span_mode,
                                     gboolean span_extended);
static gboolean rotctrl_get_pos_valid(GtkRotCtrl *ctrl, gint64 *last_pos_us);
static gboolean rotctrl_pos_recent(GtkRotCtrl *ctrl,
                                   gint64 window_us,
                                   gint64 *last_pos_us);
static gboolean rotctrl_session_ready(GtkRotCtrl *ctrl,
                                      gboolean pos_recent);
static gboolean rotctrl_manual_override_active(GtkRotCtrl *ctrl);
static gboolean G_GNUC_UNUSED rotctrl_wait_for_baseline(GtkRotCtrl *ctrl);
static void     rotctrl_update_safety(GtkRotCtrl *ctrl,
                                      gboolean caps_valid,
                                      gdouble caps_az_min,
                                      gdouble caps_az_max);
static gboolean G_GNUC_UNUSED rotctrl_keepalive_enabled(void);
static const gchar *rotctrl_span_mode_name(azel_az_mode_t mode);
static void     rotctld_note_io_ok(GtkRotCtrl *ctrl);
static gboolean rotctld_note_io_failure(GtkRotCtrl *ctrl, const gchar *context);
static void     rotctld_backend_ioerr_reset(GtkRotCtrl *ctrl);
static gboolean rotctld_backend_ioerr_note(GtkRotCtrl *ctrl,
                                           const gchar *context);
static gboolean rotctld_io_recent(GtkRotCtrl *ctrl, gint64 window_us);
static gboolean rotctld_stop_requested(GtkRotCtrl *ctrl);
static void     rotctld_request_thread_stop(GtkRotCtrl *ctrl,
                                            gboolean send_quit);
static gboolean rotctld_collect_client_thread(GtkRotCtrl *ctrl,
                                              gboolean allow_block,
                                              const gchar *context);
static void     rotctld_sleep_us(GtkRotCtrl *ctrl, gint64 usec);
static void     rotctld_finish_engage(GtkRotCtrl *ctrl);
static gint     rotctld_mgr_pid(const RotctldMgr *mgr);
static const gchar *rotctld_child_owner_name(GtkRotCtrl *ctrl,
                                             const RotctldProbeState *state,
                                             gint pid);
static void     rotctld_note_stop(GtkRotCtrl *ctrl,
                                  gint pid,
                                  const gchar *reason);
static void     rotctld_log_exit(GtkRotCtrl *ctrl,
                                 RotctldProbeState *state,
                                 const gchar *context);
static gint     rotctld_autodetect_get_ms(const gchar *env_name,
                                          gint fallback_ms,
                                          gint min_ms,
                                          gint max_ms);
static void     rotctrl_error_gate_selftest(void);
static RotctldProbeState *rotctld_probe_state_ref(RotctldProbeState *state);
static void     rotctld_probe_state_unref(RotctldProbeState *state);
static const gchar *rotctld_autodetect_state_name(rotctld_autodetect_state_t state);
static void     rotctld_autodetect_transition(GtkRotCtrl *ctrl,
                                              RotctldProbeState *state,
                                              rotctld_autodetect_state_t next,
                                              const gchar *reason);
static void     rotctld_autodetect_reset_validation(RotctldProbeState *state);
static void     rotctld_autodetect_apply_success(GtkRotCtrl *ctrl,
                                                 RotctldProbeState *state);
static gboolean rotctld_autodetect_is_candidate(const gchar *candidate);
static gboolean rotctld_autodetect_is_tty(const gchar *candidate);
static gboolean rotctld_autodetect_is_cu(const gchar *candidate);
static gchar   *rotctld_autodetect_tty_equivalent(const gchar *candidate);
static gchar   *rotctld_autodetect_cu_equivalent(const gchar *candidate);
static gchar   *rotctld_autodetect_candidate_key(const gchar *candidate);
static gboolean rotctld_autodetect_prefer_cu(const rotor_conf_t *conf,
                                             const gchar *cached);
static GSList  *rotctld_autodetect_reorder_ports(GSList *list,
                                                 gboolean prefer_cu);
static gchar   *rotctld_autodetect_join_list(GSList *list);
static gint     rotctld_autodetect_candidate_score(const gchar *candidate);
static gint     rotctld_autodetect_compare_candidates(gconstpointer a,
                                                      gconstpointer b);
static GSList  *rotctld_autodetect_filter_candidates(GSList *candidates,
                                                     guint *filtered_out);
static GSList  *rotctld_autodetect_prefer_device(GSList *list,
                                                 const gchar *device);
static GSList  *rotctld_autodetect_limit_list(GSList *list, guint limit);
static GSList  *rotctld_autodetect_remove_key(GSList *list,
                                              const gchar *key);
static gboolean rotctld_autodetect_has_next_baud(const RotctldProbeState *state,
                                                 const rotor_conf_t *conf);
static gboolean rotctld_autodetect_next_baud(RotctldProbeState *state,
                                             const rotor_conf_t *conf);
static void     rotctld_autodetect_reset_baud(RotctldProbeState *state,
                                              const rotor_conf_t *conf);
static void     rotctld_autodetect_log_candidate(GtkRotCtrl *ctrl,
                                                 RotctldProbeState *state,
                                                 const gchar *device,
                                                 gint baud);
static gboolean rotctld_autodetect_stop_child(GtkRotCtrl *ctrl,
                                              RotctldProbeState *state,
                                              const gchar *reason);
static rotctld_autodetect_step_t rotctld_autodetect_step(GtkRotCtrl *ctrl,
                                                         RotctldProbeState *state,
                                                         gint64 now_us);
static gpointer rotctld_autodetect_validate_thread(gpointer data);
static gboolean rotctld_autodetect_worker_done_cb(gpointer data);
static gboolean rotctld_probe_retry_cb(gpointer data);
static guint64  rotctld_get_engage_generation(GtkRotCtrl *ctrl);
static gboolean rotctld_generation_stale(GtkRotCtrl *ctrl,
                                         guint64 generation);
static gint     rotctld_effective_port(GtkRotCtrl *ctrl);
static gdouble  norm360(gdouble a);
static gdouble  ang_delta_deg(gdouble a, gdouble b);
static gboolean rotctrl_samples_cross_endstop(const GArray *samples,
                                              const rotor_conf_t *conf,
                                              gdouble min_el);
static gdouble  rot_clamp_az_abs(gdouble az, gdouble min, gdouble max,
                                 gboolean *clamped_out);
static void     rot_transform_update(GtkRotCtrl *ctrl);
static void     rotctrl_build_target_caps(GtkRotCtrl *ctrl,
                                          rot_target_caps_t *caps);
static void     rotctrl_plan_log_mark(GtkRotCtrl *ctrl);
static void     rotctrl_plan_log_if_pending(GtkRotCtrl *ctrl);
static const gchar *rotctrl_tracking_mode_name(rot_plan_mode_t mode);
static gboolean rotctrl_find_entry_time(const rot_plan_t *plan,
                                        gdouble now_t,
                                        gdouble min_el,
                                        gdouble *entry_t_out);
static gboolean rotctrl_find_pretrack_cmd(const GtkRotCtrl *ctrl,
                                          gdouble t_start,
                                          gdouble min_el,
                                          gdouble *az_out,
                                          gdouble *el_out,
                                          gdouble *t_out);
static gboolean rotctrl_use_rotctld_caps(GtkRotCtrl *ctrl,
                                         gboolean caps_valid,
                                         gdouble caps_az_min,
                                         gdouble caps_az_max,
                                         gdouble caps_el_min,
                                         gdouble caps_el_max);
static gdouble  rotctrl_az_distance(const rot_target_caps_t *caps,
                                    gdouble a,
                                    gdouble b);
static const gchar *rotctrl_wrap_mode_name(rot_target_wrap_mode_t mode);
static const gchar *rot_target_state_name(rot_target_state_t state);
static void     rot_target_state_set(GtkRotCtrl *ctrl,
                                     rot_target_state_t state,
                                     const gchar *reason);
static gboolean G_GNUC_UNUSED rotctrl_find_first_valid_sample(GtkRotCtrl *ctrl,
                                                const rot_target_caps_t *caps,
                                                gdouble t_start,
                                                gdouble t_end,
                                                gdouble step_sec,
                                                gboolean use_flip,
                                                gdouble *out_az,
                                                gdouble *out_el,
                                                gdouble *out_t,
                                                rot_target_invalid_reason_t *reason_out);
static void     rot_transform_prepare(const RotTransformSnapshot *transform,
                                      gdouble in_az, gdouble in_el,
                                      RotTransformPre *pre);
static void     apply_inverse_transform(const RotTransformSnapshot *transform,
                                        gdouble in_az, gdouble in_el,
                                        gdouble *out_az, gdouble *out_el);
static void     rotctrl_map_mech_to_user(GtkRotCtrl *ctrl,
                                         gdouble mech_az,
                                         gdouble mech_el,
                                         gdouble *user_az,
                                         gdouble *user_el);
static void     rot_transform_snapshot_defaults(RotTransformSnapshot *transform);
static void     rot_transform_snapshot_state_init(RotTransformSnapshotState *state);
static void     rot_transform_snapshot_publish(GtkRotCtrl *ctrl, gboolean ready);
static gboolean rot_transform_snapshot_state_get(GtkRotCtrl *ctrl,
                                                 RotTransformSnapshotState *out);

static void rot_term_log(GtkRotCtrl *ctrl, const gchar *prefix,
                         const gchar *fmt, ...) G_GNUC_PRINTF(3, 4);
static void rot_term_log_verbose(GtkRotCtrl *ctrl, const gchar *prefix,
                                 const gchar *fmt, ...) G_GNUC_PRINTF(3, 4);
static void rot_term_log_tx(GtkRotCtrl *ctrl, const gchar *cmd);
static void rot_term_log_rx(GtkRotCtrl *ctrl, const gchar *cmd,
                            const gchar *reply);
static void format_rotctld_setpos(GtkRotCtrl *ctrl,
                                  gdouble az_in,
                                  gdouble el_in,
                                  gdouble *az_out,
                                  gdouble *el_out,
                                  gchar *cmd_out,
                                  gsize cmd_len);
static void rotctld_selftest(GtkRotCtrl *ctrl);
static void rotctld_log_cb(RotctldMgr *mgr,
                           const gchar *prefix,
                           const gchar *line,
                           gpointer user_data);
static gboolean rot_parse_first_number(const gchar *line, gdouble *out);
static gboolean rotctld_response_is_valid(const gchar *text,
                                          gchar **first_line_out);
static gchar *rotctld_truncate_line(const gchar *line, gsize max_len);
static gboolean rot_host_is_local(const gchar *host);
static rotctld_probe_result_t rotctld_probe_identity(const gchar *host,
                                                     gint port,
                                                     gint timeout_ms,
                                                     gchar **first_line_out,
                                                     gchar **full_text_out);

static void rotctld_process_stop(GtkRotCtrl *ctrl);
static void rotctld_process_stop_async(GtkRotCtrl *ctrl,
                                       const gchar *reason);
static gboolean rotctld_spawn_process(GtkRotCtrl *ctrl, gchar **argv);
static gchar **rotctld_build_argv_from_command(GtkRotCtrl *ctrl,
                                               const gchar *cmdline);
static gboolean rotctld_spawn_autostart(GtkRotCtrl *ctrl,
                                        const gchar *device_override,
                                        gint baud_override,
                                        gint port_override,
                                        gchar **spawn_summary_out);

/* Offset controls callbacks */

static void offset_toggle_cb(GtkToggleButton *button, gpointer data);
static void az_offset_changed_cb(GtkSpinButton *spin, gpointer data);
static void el_offset_changed_cb(GtkSpinButton *spin, gpointer data);
static void G_GNUC_UNUSED calib_enabled_toggled_cb(GtkToggleButton *button, gpointer data);
static void G_GNUC_UNUSED calib_az_offset_changed_cb(GtkSpinButton *spin, gpointer data);
static void G_GNUC_UNUSED calib_el_offset_changed_cb(GtkSpinButton *spin, gpointer data);
static void calib_autocal_clicked_cb(GtkButton *button, gpointer data);
static void calib_autocal_response_cb(GtkDialog *dialog,
                                      gint response_id,
                                      gpointer data);
static bool rotctrl_calib_ensure_id(GtkRotCtrl *ctrl);
static void rotctrl_calib_reload(GtkRotCtrl *ctrl);
static void rotctrl_calib_update_widgets(GtkRotCtrl *ctrl);
static void rotctrl_calib_update_offset_sensitivity(GtkRotCtrl *ctrl);
static gboolean G_GNUC_UNUSED rotctrl_calib_read_mech_pos(GtkRotCtrl *ctrl,
                                            gdouble *mech_az,
                                            gdouble *mech_el);
static void rotctrl_calib_apply_send(GtkRotCtrl *ctrl,
                                     gdouble world_az,
                                     gdouble world_el,
                                     gdouble *mech_az,
                                     gdouble *mech_el);
static void G_GNUC_UNUSED rotctrl_calib_apply_read(GtkRotCtrl *ctrl,
                                     gdouble mech_az,
                                     gdouble mech_el,
                                     gdouble *world_az,
                                     gdouble *world_el,
                                     gdouble *mech_az_out,
                                     gdouble *mech_el_out);
static gdouble rotctrl_calib_uncertainty_deg(GtkRotCtrl *ctrl);
static void rotctrl_autocal_start(GtkRotCtrl *ctrl);
static void rotctrl_autocal_close_dialog(GtkRotCtrl *ctrl);
static gboolean rotctrl_autocal_apply_mech(GtkRotCtrl *ctrl,
                                           gdouble mech_az,
                                           gdouble mech_el,
                                           const gchar **reason_out);
static void rotctrl_autocal_tick(GtkRotCtrl *ctrl,
                                 gboolean rotpos_valid,
                                 gdouble rotaz,
                                 gdouble rotel);
static void axis_mode_changed_cb(GtkComboBox *box, gpointer data);
static void wrap_mode_changed_cb(GtkComboBox *box, gpointer data);
static void rot_limits_changed_cb(GtkSpinButton *spin, gpointer data);
static void az_endstop_changed_cb(GtkSpinButton *spin, gpointer data);
static void sat_selected_cb(GtkComboBox *satsel, gpointer data);
static gboolean rotctrl_apply_sat_selection(GtkRotCtrl *ctrl,
                                            gint index,
                                            const gchar *source);

/* Park position helper.
 *
 * Default is AZ=0 / EL=0, but can be overridden for station-specific setups:
 *   GPREDICT_ROT_PARK_AZ  (double, degrees)
 *   GPREDICT_ROT_PARK_EL  (double, degrees)
 */
static void rot_get_park_position(GtkRotCtrl *ctrl, gdouble *park_az, gdouble *park_el)
{
    gdouble az = 0.0;
    gdouble el = 0.0;

    const gchar *env_az = g_getenv("GPREDICT_ROT_PARK_AZ");
    const gchar *env_el = g_getenv("GPREDICT_ROT_PARK_EL");

    if (env_az && *env_az) {
        gchar *endp = NULL;
        gdouble v = g_ascii_strtod(env_az, &endp);
        if (endp != env_az)
            az = v;
    }
    if (env_el && *env_el) {
        gchar *endp = NULL;
        gdouble v = g_ascii_strtod(env_el, &endp);
        if (endp != env_el)
            el = v;
    }

    /* Clamp to configured mechanical limits if available. */
    if (ctrl && ctrl->conf) {
        /* Elevation: clamp. */
        el = CLAMP(el, ctrl->conf->minel, ctrl->conf->maxel);

        /* Azimuth: keep inside allowed span. */
        if (ctrl->conf->aztype == ROT_AZ_TYPE_180) {
            while (az > 180.0) az -= 360.0;
            while (az < -180.0) az += 360.0;
            az = CLAMP(az, ctrl->conf->minaz, ctrl->conf->maxaz);
        } else if (ctrl->conf->aztype == ROT_AZ_TYPE_480) {
            az = CLAMP(az, ctrl->conf->minaz, ctrl->conf->maxaz);
        } else {
            /* Typical 0-360 style: wrap, then clamp. */
            while (az < ctrl->conf->minaz) az += 360.0;
            while (az > ctrl->conf->maxaz) az -= 360.0;
            az = CLAMP(az, ctrl->conf->minaz, ctrl->conf->maxaz);
        }
    }

    if (park_az) *park_az = az;
    if (park_el) *park_el = el;
}

static gchar *rot_term_format_timestamp(void)
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
    GtkRotCtrl *ctrl;
    gchar      *line;
} RotTermLogTask;

static gboolean rot_term_log_idle(gpointer data)
{
    RotTermLogTask *task = data;

    if (task == NULL)
        return G_SOURCE_REMOVE;

    if (task->ctrl != NULL && task->ctrl->term_view != NULL)
        gp_term_view_log(task->ctrl->term_view, "%s", task->line);

    g_clear_object(&task->ctrl);
    g_free(task->line);
    g_free(task);
    return G_SOURCE_REMOVE;
}

static void rot_term_log_post(GtkRotCtrl *ctrl, gchar *line)
{
    if (line == NULL)
        return;

    if (ctrl == NULL || ctrl->term_view == NULL)
    /* Avoid re-arming force on NORMAL<->DEGRADED oscillations. */
    {
        g_free(line);
        return;
    }

    if (g_main_context_is_owner(g_main_context_default()))
    {
        gp_term_view_log(ctrl->term_view, "%s", line);
        g_free(line);
        return;
    }

    RotTermLogTask *task = g_new0(RotTermLogTask, 1);
    task->ctrl = g_object_ref(ctrl);
    task->line = line;
    g_main_context_invoke(NULL, rot_term_log_idle, task);
}

static gboolean rot_backend_io_disengage_idle(gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);

    if (ctrl == NULL)
        return G_SOURCE_REMOVE;

    rot_term_log(ctrl, "gpredict:err",
                 "backend I/O error persists; staying engaged (DEGRADED)");

    return G_SOURCE_REMOVE;
}

static void rot_schedule_backend_io_disengage(GtkRotCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    g_idle_add(rot_backend_io_disengage_idle, ctrl);
}

static void rot_term_log(GtkRotCtrl *ctrl, const gchar *prefix,
                         const gchar *fmt, ...)
{
    va_list ap;
    gchar *msg = NULL;
    gchar *stamp = NULL;
    gchar *line = NULL;

    if (ctrl == NULL || ctrl->term_view == NULL || prefix == NULL || fmt == NULL)
        return;

    va_start(ap, fmt);
    msg = g_strdup_vprintf(fmt, ap);
    va_end(ap);

    if (msg == NULL)
        return;

    stamp = rot_term_format_timestamp();
    line = g_strdup_printf("%s [%s] %s", stamp, prefix, msg);
    rot_term_log_post(ctrl, line);
    g_free(stamp);
    g_free(msg);
}

static void rot_term_log_verbose(GtkRotCtrl *ctrl, const gchar *prefix,
                                 const gchar *fmt, ...)
{
    va_list ap;
    gchar *msg = NULL;
    gchar *stamp = NULL;
    gchar *line = NULL;

    if (ctrl == NULL || !ctrl->verbose_logging ||
        ctrl->term_view == NULL || prefix == NULL || fmt == NULL)
        return;

    va_start(ap, fmt);
    msg = g_strdup_vprintf(fmt, ap);
    va_end(ap);

    if (msg == NULL)
        return;

    stamp = rot_term_format_timestamp();
    line = g_strdup_printf("%s [%s] %s", stamp, prefix, msg);
    rot_term_log_post(ctrl, line);
    g_free(stamp);
    g_free(msg);
}

static void rotctrl_ui_begin_update(GtkRotCtrl *ctrl, const gchar *reason)
{
    if (ctrl == NULL)
        return;

    ctrl->ui_updating = TRUE;
    if (ctrl->verbose_logging)
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rotctrl ui_begin_update %s",
                    reason ? reason : "(none)");
}

static gboolean rotctrl_link_lost_popup_cb(gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);

    if (ctrl == NULL)
        return G_SOURCE_REMOVE;

    rot_show_message(ctrl,
                     GTK_MESSAGE_ERROR,
                     _("Rotor link lost"),
                     _("Lost rotor feedback. Check rotor model, serial device, baud rate, and power. Re-engage to retry."));
    return G_SOURCE_REMOVE;
}

static gboolean rotctrl_link_lost_disengage_cb(gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);

    if (ctrl == NULL)
        return G_SOURCE_REMOVE;

    sat_log_log(SAT_LOG_LEVEL_WARN,
                "rotor link lost: disengaging tracking and rotor");
    rot_term_log(ctrl, "gpredict:err",
                 "rotor link lost: disengaging tracking and rotor");

    if (ctrl->track != NULL &&
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ctrl->track)))
    {
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->track), FALSE);
    }

    if (ctrl->LockBut != NULL &&
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ctrl->LockBut)))
    {
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->LockBut), FALSE);
    }

    return G_SOURCE_REMOVE;
}

static void rotctrl_ui_end_update(GtkRotCtrl *ctrl, const gchar *reason)
{
    if (ctrl == NULL)
        return;

    ctrl->ui_updating = FALSE;
    if (ctrl->verbose_logging)
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rotctrl ui_end_update %s",
                    reason ? reason : "(none)");
}

static void rotctrl_set_pass_on_plots(GtkRotCtrl *ctrl, pass_t *pass)
{
    GList *node;

    if (ctrl == NULL)
        return;

    if (ctrl->plot != NULL)
        gtk_polar_plot_set_pass(GTK_POLAR_PLOT(ctrl->plot), pass);

    for (node = ctrl->detached_plots; node != NULL; node = node->next)
    {
        RotDetachedPlot *entry = node->data;
        if (entry != NULL && entry->plot != NULL)
            gtk_polar_plot_set_pass(GTK_POLAR_PLOT(entry->plot), pass);
    }
}

static void rotctrl_set_rotor_pos_on_plots(GtkRotCtrl *ctrl,
                                           gdouble az, gdouble el)
{
    GList *node;

    if (ctrl == NULL)
        return;

    if (ctrl->plot != NULL)
        gtk_polar_plot_set_rotor_pos(GTK_POLAR_PLOT(ctrl->plot), az, el);

    for (node = ctrl->detached_plots; node != NULL; node = node->next)
    {
        RotDetachedPlot *entry = node->data;
        if (entry != NULL && entry->plot != NULL)
            gtk_polar_plot_set_rotor_pos(GTK_POLAR_PLOT(entry->plot), az, el);
    }
}

static void rotctrl_set_target_pos_on_plots(GtkRotCtrl *ctrl,
                                            gdouble az, gdouble el)
{
    GList *node;

    if (ctrl == NULL)
        return;

    if (ctrl->plot != NULL)
        gtk_polar_plot_set_target_pos(GTK_POLAR_PLOT(ctrl->plot), az, el);

    for (node = ctrl->detached_plots; node != NULL; node = node->next)
    {
        RotDetachedPlot *entry = node->data;
        if (entry != NULL && entry->plot != NULL)
            gtk_polar_plot_set_target_pos(GTK_POLAR_PLOT(entry->plot), az, el);
    }
}

static void rotctrl_set_ctrl_pos_on_plots(GtkRotCtrl *ctrl,
                                          gdouble az, gdouble el)
{
    GList *node;

    if (ctrl == NULL)
        return;

    if (ctrl->plot != NULL)
        gtk_polar_plot_set_ctrl_pos(GTK_POLAR_PLOT(ctrl->plot), az, el);

    for (node = ctrl->detached_plots; node != NULL; node = node->next)
    {
        RotDetachedPlot *entry = node->data;
        if (entry != NULL && entry->plot != NULL)
            gtk_polar_plot_set_ctrl_pos(GTK_POLAR_PLOT(entry->plot), az, el);
    }
}

static void rotctrl_queue_draw_plots(GtkRotCtrl *ctrl)
{
    GList *node;

    if (ctrl == NULL)
        return;

    if (ctrl->plot != NULL)
        gtk_widget_queue_draw(ctrl->plot);

    for (node = ctrl->detached_plots; node != NULL; node = node->next)
    {
        RotDetachedPlot *entry = node->data;
        if (entry != NULL && entry->plot != NULL)
            gtk_widget_queue_draw(entry->plot);
    }
}

static gboolean rotctrl_target_display_valid(const sat_t *sat)
{
    if (sat == NULL)
        return FALSE;

    if (!isfinite(sat->az) || !isfinite(sat->el) ||
        !isfinite(sat->aos) || !isfinite(sat->los))
    {
        return FALSE;
    }

    if (fabs(sat->az) > 720.0 || fabs(sat->el) > 180.0)
        return FALSE;

    return TRUE;
}

static gboolean rotctrl_sync_sat_copy_from_module(GtkRotCtrl *ctrl,
                                                  sat_t *dst)
{
    sat_t *src = NULL;
    gint catnr = 0;

    if (ctrl == NULL || dst == NULL || ctrl->module_sats == NULL)
        return FALSE;

    catnr = dst->tle.catnr;
    src = SAT(g_hash_table_lookup(ctrl->module_sats, &catnr));
    if (src == NULL)
        return FALSE;

    if (g_strcmp0(dst->name, src->name) != 0)
    {
        g_free(dst->name);
        dst->name = src->name ? g_strdup(src->name) : NULL;
    }
    if (g_strcmp0(dst->nickname, src->nickname) != 0)
    {
        g_free(dst->nickname);
        dst->nickname = src->nickname ? g_strdup(src->nickname) : NULL;
    }
    if (g_strcmp0(dst->website, src->website) != 0)
    {
        g_free(dst->website);
        dst->website = src->website ? g_strdup(src->website) : NULL;
    }

    dst->tle = src->tle;
    dst->flags = src->flags;
    dst->sgps = src->sgps;
    dst->dps = src->dps;
    dst->deep_arg = src->deep_arg;
    dst->jul_epoch = src->jul_epoch;
    dst->jul_utc = src->jul_utc;
    dst->tsince = src->tsince;
    dst->pos = src->pos;
    dst->vel = src->vel;
    dst->aos = src->aos;
    dst->los = src->los;
    dst->az = src->az;
    dst->el = src->el;
    dst->range = src->range;
    dst->range_rate = src->range_rate;
    dst->ra = src->ra;
    dst->dec = src->dec;
    dst->ssplat = src->ssplat;
    dst->ssplon = src->ssplon;
    dst->alt = src->alt;
    dst->velo = src->velo;
    dst->ma = src->ma;
    dst->footprint = src->footprint;
    dst->phase = src->phase;
    dst->meanmo = src->meanmo;
    dst->orbit = src->orbit;
    dst->otype = src->otype;

    return TRUE;
}

static void rotctrl_update_detached_labels(GtkRotCtrl *ctrl)
{
    GList *node;
    const gchar *sat_name = NULL;
    gchar *buff = NULL;
    gboolean target_valid = FALSE;

    if (ctrl == NULL)
        return;

    target_valid = rotctrl_target_display_valid(ctrl->target) &&
                   isfinite(ctrl->t);

    if (ctrl->target != NULL)
        sat_name = ctrl->target->nickname;
    else
        sat_name = "--";

    for (node = ctrl->detached_plots; node != NULL; node = node->next)
    {
        RotDetachedPlot *entry = node->data;

        if (entry == NULL)
            continue;

        if (entry->plot != NULL)
            gtk_polar_plot_set_sat_name(GTK_POLAR_PLOT(entry->plot), sat_name);

        if (entry->aoslos_label != NULL)
        {
            g_free(buff);
            if (target_valid)
                buff = predict_format_aoslos_countdown(ctrl->target,
                                                       ctrl->t, TRUE, TRUE);
            else
                buff = NULL;
            if (buff == NULL)
                buff = g_strdup(ROTCTRL_AOSLOS_PLACEHOLDER);
            gp_safe_label_set_markup(entry->aoslos_label, buff);
        }
    }

    g_free(buff);
}

static gboolean rotctrl_combo_popup_shown(GtkComboBox *box)
{
    return gp_ui_combo_popup_shown(box);
}

static void rotctrl_detached_plot_destroy(GtkWidget *widget, gpointer data)
{
    RotDetachedPlot *entry = data;

    (void)widget;

    if (entry == NULL)
        return;

    if (entry->ctrl != NULL)
        entry->ctrl->detached_plots =
            g_list_remove(entry->ctrl->detached_plots, entry);

    g_free(entry);
}

static void rotctrl_detach_plot_cb(GtkButton *button, gpointer data)
{
    GtkRotCtrl     *ctrl = GTK_ROT_CTRL(data);
    GtkWidget      *window;
    GtkWidget      *frame;
    GtkWidget      *pad;
    GtkWidget      *plot;
    GtkWidget      *aoslos_frame;
    GtkWidget      *aoslos_label;
    GtkWidget      *outer;
    GtkWidget      *parent;
    const gchar    *parent_title = NULL;
    gchar          *title = NULL;
    GtkRequisition  min_req;
    GtkRequisition  nat_req;
    RotDetachedPlot *entry;

    (void)button;

    if (ctrl == NULL)
        return;

    plot = gtk_polar_plot_new(ctrl->qth, ctrl->pass);
    gtk_polar_plot_set_margin(GTK_POLAR_PLOT(plot), 32);
    gtk_widget_set_hexpand(plot, TRUE);
    gtk_widget_set_vexpand(plot, TRUE);

    aoslos_label = gtk_label_new(NULL);
    gp_safe_label_set_markup(aoslos_label, ROTCTRL_AOSLOS_PLACEHOLDER);
    gtk_widget_set_tooltip_text(aoslos_label,
                                _("The time remaining until the next AOS or "
                                  "LOS event"));
    g_object_set(aoslos_label, "xalign", 0.5f, "yalign", 0.5f, NULL);
    gtk_widget_set_margin_top(aoslos_label, 3);
    gtk_widget_set_margin_bottom(aoslos_label, 3);

    aoslos_frame = gtk_frame_new(NULL);
    gtk_container_add(GTK_CONTAINER(aoslos_frame), aoslos_label);

    frame = gtk_frame_new(NULL);
    gtk_widget_set_margin_top(frame, 8);
    gtk_widget_set_margin_bottom(frame, 8);
    gtk_widget_set_margin_start(frame, 6);
    gtk_widget_set_margin_end(frame, 6);
    pad = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    gtk_container_set_border_width(GTK_CONTAINER(pad), 12);
    gtk_widget_set_hexpand(pad, TRUE);
    gtk_widget_set_vexpand(pad, TRUE);
    gtk_container_add(GTK_CONTAINER(pad), plot);
    gtk_container_add(GTK_CONTAINER(frame), pad);

    outer = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
    gtk_widget_set_hexpand(outer, TRUE);
    gtk_widget_set_vexpand(outer, TRUE);
    gtk_box_pack_start(GTK_BOX(outer), frame, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(outer), aoslos_frame, FALSE, FALSE, 0);

    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    parent = gtk_widget_get_toplevel(GTK_WIDGET(ctrl));
    if (GTK_IS_WINDOW(parent))
    {
        parent_title = gtk_window_get_title(GTK_WINDOW(parent));
        gtk_window_set_transient_for(GTK_WINDOW(window), GTK_WINDOW(parent));
        gtk_window_set_destroy_with_parent(GTK_WINDOW(window), TRUE);
    }

    if (parent_title != NULL)
        title = g_strdup_printf(_("%s (Plot)"), parent_title);
    else
        title = g_strdup(_("Gpredict Rotator Plot"));
    gtk_window_set_title(GTK_WINDOW(window), title);
    g_free(title);

    gtk_widget_get_preferred_size(plot, &min_req, &nat_req);
    if (nat_req.width < min_req.width)
        nat_req.width = min_req.width;
    if (nat_req.height < min_req.height)
        nat_req.height = min_req.height;
    if (nat_req.width > 0 && nat_req.height > 0)
        gtk_window_set_default_size(GTK_WINDOW(window),
                                    nat_req.width, nat_req.height);

    gtk_container_add(GTK_CONTAINER(window), outer);

    entry = g_new0(RotDetachedPlot, 1);
    entry->ctrl = ctrl;
    entry->window = window;
    entry->plot = plot;
    entry->aoslos_label = aoslos_label;
    ctrl->detached_plots = g_list_append(ctrl->detached_plots, entry);
    g_signal_connect(G_OBJECT(window), "destroy",
                     G_CALLBACK(rotctrl_detached_plot_destroy), entry);

    if (ctrl->pass != NULL)
        rotctrl_set_pass_on_plots(ctrl, ctrl->pass);
    if (ctrl->target != NULL)
        rotctrl_set_target_pos_on_plots(ctrl,
                                        ctrl->target->az,
                                        ctrl->target->el);
    if (ctrl->conf != NULL)
    {
        gdouble dispaz = gtk_rot_knob_get_value(GTK_ROT_KNOB(ctrl->AzSet));
        gdouble dispel = gtk_rot_knob_get_value(GTK_ROT_KNOB(ctrl->ElSet));
        if (ctrl->conf->aztype == ROT_AZ_TYPE_180 && dispaz < 0.0)
            dispaz += 360.0;
        rotctrl_set_ctrl_pos_on_plots(ctrl, dispaz, dispel);
    }
    rotctrl_queue_draw_plots(ctrl);
    rotctrl_update_detached_labels(ctrl);

    gtk_widget_show_all(window);
}


typedef struct
{
    GtkComboBox *box;
    gint index;
    GCallback cb;
    GtkRotCtrl *ctrl;
} RotCtrlComboUpdate;

static gboolean rotctrl_combo_set_active_idle(gpointer data)
{
    RotCtrlComboUpdate *update = data;
    gboolean was_updating = FALSE;

    if (update == NULL || update->box == NULL)
    {
        g_free(update);
        return G_SOURCE_REMOVE;
    }

    if (rotctrl_combo_popup_shown(update->box))
        return G_SOURCE_CONTINUE;

    if (update->ctrl != NULL)
    {
        was_updating = update->ctrl->ui_updating;
        rotctrl_ui_begin_update(update->ctrl, "combo_deferred");
    }

    g_signal_handlers_block_by_func(update->box, (gpointer)update->cb, update->ctrl);
    gtk_combo_box_set_active(update->box, update->index);
    g_signal_handlers_unblock_by_func(update->box, (gpointer)update->cb, update->ctrl);

    if (update->ctrl != NULL && !was_updating)
        rotctrl_ui_end_update(update->ctrl, "combo_deferred");

    g_object_unref(update->box);
    if (update->ctrl != NULL)
        g_object_unref(update->ctrl);
    g_free(update);
    return G_SOURCE_REMOVE;
}

static void rotctrl_combo_set_active_safe(GtkRotCtrl *ctrl,
                                          GtkComboBox *box,
                                          gint index,
                                          GCallback cb)
{
    RotCtrlComboUpdate *update;

    if (box == NULL)
        return;

    if (ctrl == NULL || ctrl->ui_updating)
    {
        g_signal_handlers_block_by_func(box, (gpointer)cb, ctrl);
        gtk_combo_box_set_active(box, index);
        g_signal_handlers_unblock_by_func(box, (gpointer)cb, ctrl);
        return;
    }

    if (!rotctrl_combo_popup_shown(box))
    {
        rotctrl_ui_begin_update(ctrl, "combo_set_active");
        g_signal_handlers_block_by_func(box, (gpointer)cb, ctrl);
        gtk_combo_box_set_active(box, index);
        g_signal_handlers_unblock_by_func(box, (gpointer)cb, ctrl);
        rotctrl_ui_end_update(ctrl, "combo_set_active");
        return;
    }

    update = g_new0(RotCtrlComboUpdate, 1);
    update->box = g_object_ref(box);
    update->index = index;
    update->cb = cb;
    update->ctrl = g_object_ref(ctrl);
    g_idle_add_full(G_PRIORITY_DEFAULT_IDLE,
                    rotctrl_combo_set_active_idle,
                    update,
                    NULL);
}

static void rot_log_rate_limited(GtkRotCtrl *ctrl,
                                 RotLogRate *rate,
                                 gint64 interval_us,
                                 sat_log_level_t level,
                                 const gchar *prefix,
                                 const gchar *fmt, ...)
{
    va_list ap;
    gint64 now_us;
    gchar *msg = NULL;
    gchar *with_count = NULL;
    guint count = 1;

    if (fmt == NULL || rate == NULL)
        return;

    now_us = g_get_monotonic_time();
    if (rate->last_log_us > 0 &&
        (now_us - rate->last_log_us) < interval_us)
    {
        rate->suppressed++;
        return;
    }

    va_start(ap, fmt);
    msg = g_strdup_vprintf(fmt, ap);
    va_end(ap);

    if (msg == NULL)
        return;

    if (rate->suppressed > 0)
        count += rate->suppressed;

    if (count > 1)
        with_count = g_strdup_printf("%s (x%u)", msg, count);

    sat_log_log(level, "%s", with_count ? with_count : msg);
    if (prefix != NULL)
        rot_term_log(ctrl, prefix, "%s", with_count ? with_count : msg);

    rate->last_log_us = now_us;
    rate->suppressed = 0;
    g_free(with_count);
    g_free(msg);
}

static const gchar *rot_rprt_error_string(gint code)
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

static gboolean rot_parse_rprt_code(const gchar *reply, gint *code_out)
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

static gboolean rot_parse_rprt_code_any(const gchar *reply, gint *code_out)
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

static void rot_term_log_tx(GtkRotCtrl *ctrl, const gchar *cmd)
{
    gchar *trim;

    if (ctrl == NULL || cmd == NULL || !ctrl->verbose_logging)
        return;

    trim = g_strdup(cmd);
    g_strchomp(trim);
    g_strstrip(trim);

    if (g_str_has_prefix(trim, "P "))
    {
        gchar **parts = g_strsplit(trim, " ", 0);

        if (parts[1] && parts[2])
            rot_term_log(ctrl, "gpredict:tx",
                         "set_position az=%s el=%s", parts[1], parts[2]);
        else
            rot_term_log(ctrl, "gpredict:tx", "set_position %s", trim);
        g_strfreev(parts);
    }
    else if (g_ascii_strcasecmp(trim, "p") == 0)
    {
        rot_term_log(ctrl, "gpredict:tx", "get_position");
    }
    else if (g_ascii_strcasecmp(trim, "q") == 0)
    {
        rot_term_log(ctrl, "gpredict:tx", "quit");
    }
    else if (g_str_has_prefix(trim, "\\dump_state"))
    {
        rot_term_log(ctrl, "gpredict:tx", "dump_state");
    }
    else
    {
        rot_term_log(ctrl, "gpredict:tx", "%s", trim);
    }

    g_free(trim);
}

static void rot_term_log_rx(GtkRotCtrl *ctrl, const gchar *cmd,
                            const gchar *reply)
{
    gchar *trim_cmd = NULL;
    gchar *trim_reply = NULL;
    gint code = 0;

    if (ctrl == NULL || cmd == NULL || reply == NULL)
        return;

    trim_cmd = g_strdup(cmd);
    trim_reply = g_strdup(reply);
    g_strchomp(trim_cmd);
    g_strstrip(trim_cmd);
    g_strchomp(trim_reply);
    g_strstrip(trim_reply);

    if (!ctrl->verbose_logging)
    {
        if (g_str_has_prefix(trim_cmd, "P "))
        {
            if (rot_parse_rprt_code_any(trim_reply, &code) && code != 0)
                rot_term_log(ctrl, "gpredict:err",
                             "set_position failed: %s (%d)",
                             rot_rprt_error_string(code), code);
        }
        else if (g_ascii_strcasecmp(trim_cmd, "p") == 0)
        {
            if (rot_parse_rprt_code_any(trim_reply, &code) && code != 0)
                rot_term_log(ctrl, "gpredict:err",
                             "get_position failed: %s (%d)",
                             rot_rprt_error_string(code), code);
        }
        g_free(trim_cmd);
        g_free(trim_reply);
        return;
    }

    if (g_str_has_prefix(trim_cmd, "P "))
    {
        if (rot_parse_rprt_code_any(trim_reply, &code))
        {
            if (code == 0)
                rot_term_log(ctrl, "gpredict:rx", "set_position ok");
            else
                rot_term_log(ctrl, "gpredict:err",
                             "set_position failed: %s (%d)",
                             rot_rprt_error_string(code), code);
        }
        else
        {
            rot_term_log(ctrl, "gpredict:rx", "set_position reply=%s",
                         trim_reply);
        }
    }
    else if (g_ascii_strcasecmp(trim_cmd, "p") == 0)
    {
        if (rot_parse_rprt_code_any(trim_reply, &code))
        {
            if (code == 0)
                rot_term_log(ctrl, "gpredict:rx", "get_position ok");
            else
                rot_term_log(ctrl, "gpredict:err",
                             "get_position failed: %s (%d)",
                             rot_rprt_error_string(code), code);
        }
        else
        {
            gchar **lines = g_strsplit(trim_reply, "\n", 3);
            gdouble az = 0.0;
            gdouble el = 0.0;

            if (lines[0] && lines[1] &&
                rot_parse_first_number(lines[0], &az) &&
                rot_parse_first_number(lines[1], &el))
            {
                rot_term_log(ctrl, "gpredict:rx",
                             "get_position az=%.2f el=%.2f", az, el);
            }
            else
            {
                rot_term_log(ctrl, "gpredict:rx",
                             "get_position reply=%s", trim_reply);
            }
            g_strfreev(lines);
        }
    }
    else if (g_str_has_prefix(trim_cmd, "\\dump_state"))
    {
        gchar *first_line = NULL;
        gboolean valid = rotctld_response_is_valid(trim_reply, &first_line);
        if (valid)
        {
            rot_term_log(ctrl, "gpredict:rx", "dump_state ok");
        }
        else
        {
            gchar *trunc = rotctld_truncate_line(first_line, 120);
            rot_term_log(ctrl, "gpredict:rx", "dump_state reply=%s",
                         trunc ? trunc : "(none)");
            g_free(trunc);
        }
        g_free(first_line);
    }
    else
    {
        rot_term_log(ctrl, "gpredict:rx", "%s", trim_reply);
    }

    g_free(trim_cmd);
    g_free(trim_reply);
}

static void rotctrl_set_ui_status_detail(GtkRotCtrl *ctrl, const gchar *detail)
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

static void rotctrl_set_ui_hard_error(GtkRotCtrl *ctrl, const gchar *reason)
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

static void rotctrl_clear_ui_hard_error(GtkRotCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    ctrl->ui_hard_error = FALSE;
    ctrl->ui_hard_error_reason[0] = '\0';
}

static void rotctrl_apply_ui_status(GtkRotCtrl *ctrl,
                                    const RotorStateSnapshot *snap,
                                    const gchar *reason)
{
    RotorUiStatus next;
    UiSeverity next_severity;
    UiSeverity prev_severity;
    StatusIndicatorPulseMode pulse_mode;
    const gchar *detail = NULL;
    const gchar *why = (reason != NULL) ? reason : "update";
    GtkWidget *status_label;
    GtkWidget *status_indicator;

    if (ctrl == NULL || snap == NULL)
        return;

    next = rotor_compute_ui_status(snap);

    if (next != ctrl->ui_status)
    {
        if (ctrl->verbose_logging)
        {
            rot_term_log(ctrl, "gpredict:state",
                         "rotor ui status: %s -> %s (reason=%s)",
                         rotor_ui_status_to_string(ctrl->ui_status),
                         rotor_ui_status_to_string(next),
                         why);
        }
        ctrl->ui_status = next;
    }

    status_label = g_object_get_data(G_OBJECT(ctrl), "rot-status-label");
    if (status_label == NULL)
        return;

    gtk_label_set_text(GTK_LABEL(status_label),
                       rotor_ui_status_to_string(ctrl->ui_status));

    status_indicator = g_object_get_data(G_OBJECT(ctrl), "rot-status-indicator");
    if (status_indicator != NULL)
    {
        prev_severity = status_indicator_get_severity(status_indicator);
        next_severity = rotor_ui_status_to_severity(ctrl->ui_status);
        pulse_mode = rotor_ui_status_to_pulse_mode(ctrl->ui_status);
        if (ctrl->verbose_logging && prev_severity != next_severity)
        {
            rot_term_log(ctrl, "gpredict:state",
                         "status indicator: severity %s -> %s",
                         ui_severity_to_string(prev_severity),
                         ui_severity_to_string(next_severity));
        }
        status_indicator_set_severity(status_indicator, next_severity);
        status_indicator_set_pulse_mode(status_indicator, pulse_mode);
    }

    if (ctrl->ui_hard_error && ctrl->ui_hard_error_reason[0] != '\0')
        detail = ctrl->ui_hard_error_reason;
    else if (ctrl->ui_status_detail[0] != '\0')
        detail = ctrl->ui_status_detail;

    gtk_widget_set_tooltip_text(status_label, detail);
}

static const gchar *rot_session_state_name(rot_session_state_t state)
{
    switch (state)
    {
    case ROT_SESSION_DISCONNECTED:
        return "DISCONNECTED";
    case ROT_SESSION_CONNECTING:
        return "CONNECTING";
    case ROT_SESSION_ENGAGING:
        return "ENGAGING";
    case ROT_SESSION_READY:
        return "READY";
    case ROT_SESSION_DEGRADED:
        return "DEGRADED";
    default:
        return "UNKNOWN";
    }
}

static void rot_session_set_state(GtkRotCtrl *ctrl,
                                  rot_session_state_t state,
                                  const gchar *reason,
                                  gboolean send_quit)
{
    const gchar *from;
    const gchar *to;

    if (ctrl == NULL)
        return;

    if (ctrl->session_state == state)
        return;

    from = rot_session_state_name(ctrl->session_state);
    to = rot_session_state_name(state);

    sat_log_log(SAT_LOG_LEVEL_INFO,
                "rotor_state %s->%s reason=%s quit=%s",
                from, to,
                reason ? reason : "none",
                send_quit ? "yes" : "no");
    rot_term_log_verbose(ctrl, "gpredict:state",
                         "rotor_state %s->%s reason=%s quit=%s",
                         from, to,
                         reason ? reason : "none",
                         send_quit ? "yes" : "no");

    ctrl->session_state = state;
    if (state == ROT_SESSION_CONNECTING)
    {
        ctrl->hold_position_on_engage = TRUE;
        ctrl->hold_position_log_emitted = FALSE;
        ctrl->ui_degraded_latched = FALSE;
        ctrl->ui_degraded_clear_hits = 0;
        ctrl->ok_streak = 0;
        ctrl->bad_streak = 0;
        ctrl->timeout_streak = 0;
        ctrl->comm_status = ROT_COMM_STANDBY;
    }
}

static void rotctrl_update_session_state(GtkRotCtrl *ctrl,
                                         gboolean pos_recent,
                                         gboolean pos_unknown,
                                         gboolean pos_cmd_ok,
                                         gboolean io_error,
                                         gint64 last_pos_us,
                                         guint pos_failures)
{
    rotctld_client_state_t client_state = ROTCTLD_CLIENT_STOPPED;
    const gchar *reason = NULL;
    const gchar *io_reason = NULL;
    gchar io_reason_buf[64] = { 0 };
    RotTransformSnapshotState snap_state;
    gboolean reconnect_degraded = FALSE;
    gboolean pos_degraded = FALSE;
    gboolean daemon_ok = FALSE;
    gboolean pos_invalid = FALSE;
    gboolean backend_ioerr_pending = FALSE;
    gboolean handshake_pos_ok = FALSE;
    gint64 first_pos_deadline_us = 0;
    guint reconnect_failures = 0;
    gint64 last_pos_attempt_us = 0;
    gint64 now_us = 0;
    gint64 pos_age_ms = -1;
    gint64 pos_attempt_age_ms = -1;
    gboolean stale_suppressed = FALSE;
    gboolean pos_poll_pending = FALSE;
    gboolean feedback_disabled = FALSE;
    gboolean stale_enter = FALSE;
    gboolean stale_exit = FALSE;
    gboolean stale_active = FALSE;
    gint poll_ms = 0;
    gint pos_timeout_ms = 0;
    gint stale_ms = 0;
    gint warn_ms = 0;
    gint degraded_ms = 0;

    if (ctrl == NULL)
        return;

    feedback_disabled = (ctrl->conf && ctrl->conf->disable_pos_feedback_checks);

    (void)pos_unknown;
    (void)pos_cmd_ok;
    (void)pos_recent;

    if (!ctrl->engaged)
    {
        if (!ctrl->engage_pending)
            rot_session_set_state(ctrl, ROT_SESSION_DISCONNECTED, "idle", FALSE);
        ctrl->ui_degraded_latched = FALSE;
        ctrl->ui_degraded_clear_hits = 0;
        ctrl->pos_stale_hits = 0;
        ctrl->pos_stale_hyst_active = FALSE;
        ctrl->pos_stale_ready_hits = 0;
        return;
    }

    if (ctrl->client.client)
    {
        client_state = rotctld_client_get_state(ctrl->client.client);
        reason = rotctld_client_get_state_reason(ctrl->client.client);
    }

    now_us = g_get_monotonic_time();
    poll_ms = rotctrl_poll_period_ms(ctrl);
    pos_timeout_ms = rotctrl_getpos_timeout_ms(ctrl);
    g_mutex_lock(&ctrl->client.mutex);
    last_pos_attempt_us = ctrl->client.last_pos_attempt_us;
    g_mutex_unlock(&ctrl->client.mutex);

    if (last_pos_attempt_us > 0 && now_us > last_pos_attempt_us)
    {
        pos_attempt_age_ms = (now_us - last_pos_attempt_us) / 1000;
        pos_poll_pending = (pos_attempt_age_ms <= pos_timeout_ms);
    }

    if (last_pos_us > 0 && now_us > last_pos_us)
        pos_age_ms = (now_us - last_pos_us) / 1000;
    warn_ms = rotctrl_stale_warn_ms(ctrl);
    degraded_ms = rotctrl_stale_degraded_ms(ctrl);
    stale_ms = degraded_ms;
    stale_suppressed = rotctrl_stale_suppressed(ctrl);
    stale_enter = (last_pos_us <= 0) || (pos_age_ms >= degraded_ms);
    if (stale_enter &&
        pos_poll_pending &&
        last_pos_us > 0 &&
        pos_age_ms >= degraded_ms &&
        pos_age_ms < (degraded_ms + poll_ms))
    {
        stale_enter = FALSE;
    }
    stale_exit = (pos_age_ms >= 0) && (pos_age_ms <= warn_ms);
    if (stale_suppressed)
    {
        ctrl->pos_stale_hyst_active = FALSE;
        ctrl->pos_stale_hits = 0;
        ctrl->pos_stale_ready_hits = 0;
    }
    else if (!ctrl->pos_stale_hyst_active)
    {
        if (stale_enter)
        {
            if (ctrl->pos_stale_hits < G_MAXUINT)
                ctrl->pos_stale_hits++;
            if (ctrl->pos_stale_hits >= ROT_STALE_DEGRADED_CONFIRM_POLLS)
            {
                ctrl->pos_stale_hyst_active = TRUE;
                ctrl->pos_stale_ready_hits = 0;
            }
        }
        else
        {
            ctrl->pos_stale_hits = 0;
        }
    }
    else
    {
        ctrl->pos_stale_hits = ROT_STALE_DEGRADED_CONFIRM_POLLS;
        if (stale_exit)
        {
            if (ctrl->pos_stale_ready_hits < G_MAXUINT)
                ctrl->pos_stale_ready_hits++;
            if (ctrl->pos_stale_ready_hits >= ROT_STALE_READY_CONFIRM_POLLS)
            {
                ctrl->pos_stale_hyst_active = FALSE;
                ctrl->pos_stale_hits = 0;
                ctrl->pos_stale_ready_hits = 0;
            }
        }
        else
        {
            ctrl->pos_stale_ready_hits = 0;
        }
    }
    stale_active = ctrl->pos_stale_hyst_active;

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "rotor stale eval poll=%dms age=%lldms stale_ms=%d "
                "enter=%d hits=%u/%u exit=%d active=%d ready_hits=%u/%u "
                "pending=%d attempt_age=%lldms timeout=%dms state=%s",
                poll_ms,
                (long long)pos_age_ms,
                stale_ms,
                stale_enter ? 1 : 0,
                ctrl->pos_stale_hits,
                ROT_STALE_DEGRADED_CONFIRM_POLLS,
                stale_exit ? 1 : 0,
                stale_active ? 1 : 0,
                ctrl->pos_stale_ready_hits,
                ROT_STALE_READY_CONFIRM_POLLS,
                pos_poll_pending ? 1 : 0,
                (long long)pos_attempt_age_ms,
                pos_timeout_ms,
                rot_session_state_name(ctrl->session_state));

    g_mutex_lock(&ctrl->client.mutex);
    reconnect_degraded = ctrl->client.reconnect_degraded;
    reconnect_failures = ctrl->client.reconnect_failures;
    pos_degraded = ctrl->client.pos_degraded;
    daemon_ok = ctrl->client.daemon_ok;
    pos_invalid = ctrl->client.pos_invalid;
    backend_ioerr_pending = ctrl->client.backend_ioerr_disengage_pending;
    handshake_pos_ok = ctrl->client.handshake_pos_ok;
    first_pos_deadline_us = ctrl->client.first_pos_deadline_us;
    g_mutex_unlock(&ctrl->client.mutex);

    if (feedback_disabled)
    {
        pos_invalid = FALSE;
        pos_degraded = FALSE;
        handshake_pos_ok = TRUE;
        first_pos_deadline_us = 0;
    }

    if (backend_ioerr_pending)
    {
        rot_session_set_state(ctrl, ROT_SESSION_DEGRADED,
                              "backend io error", FALSE);
        return;
    }

    if (!feedback_disabled && pos_invalid)
    {
        rot_session_set_state(ctrl, ROT_SESSION_DEGRADED,
                              "pos_invalid", FALSE);
        return;
    }

    if (!feedback_disabled && pos_degraded)
    {
        rot_session_set_state(ctrl, ROT_SESSION_DEGRADED,
                              "get_position retry limit", FALSE);
        return;
    }

    if (reconnect_degraded)
    {
        rot_session_set_state(ctrl, ROT_SESSION_DEGRADED,
                              "reconnect retry limit", FALSE);
        return;
    }

    if ((client_state == ROTCTLD_CLIENT_DEGRADED || io_error) &&
        reconnect_failures > 0)
    {
        rot_session_set_state(ctrl, ROT_SESSION_ENGAGING,
                              "retrying", FALSE);
        return;
    }

    if (client_state == ROTCTLD_CLIENT_DEGRADED || io_error)
    {
        if (io_error && g_mutex_trylock(&ctrl->client.mutex))
        {
            g_strlcpy(io_reason_buf, ctrl->client.io_error_reason,
                      sizeof(io_reason_buf));
            g_mutex_unlock(&ctrl->client.mutex);
            if (io_reason_buf[0] != '\0')
                io_reason = io_reason_buf;
        }
        rot_session_set_state(ctrl, ROT_SESSION_DEGRADED,
                              reason ? reason : (io_reason ? io_reason
                                                           : (io_error ? "io error"
                                                                       : "degraded")),
                              FALSE);
        return;
    }

    if (!daemon_ok)
    {
        rot_session_set_state(ctrl, ROT_SESSION_ENGAGING,
                              "handshake pending", FALSE);
        return;
    }

    if (!feedback_disabled && !handshake_pos_ok &&
        first_pos_deadline_us > 0 &&
        g_get_monotonic_time() > first_pos_deadline_us)
    {
        rot_session_set_state(ctrl, ROT_SESSION_DEGRADED,
                              "waiting for first position", FALSE);
        return;
    }

    rot_transform_snapshot_state_init(&snap_state);
    if (!rot_transform_snapshot_state_get(ctrl, &snap_state))
    {
        rot_session_set_state(ctrl, ROT_SESSION_ENGAGING,
                              "transform not ready", FALSE);
        return;
    }

    if (!feedback_disabled && !handshake_pos_ok)
    {
        rot_session_set_state(ctrl, ROT_SESSION_ENGAGING,
                              "waiting for first position", FALSE);
        return;
    }

    /* Grace policy: allow brief get_position gaps; degrade only after hysteresis. */
    if (!stale_suppressed && stale_active)
    {
        const gchar *stale_reason = "position stale";

        if (ctrl->session_state != ROT_SESSION_DEGRADED)
        {
            rot_log_rate_limited(ctrl, &ctrl->pos_stale_rate,
                                 ROTCTLD_FAILURE_LOG_INTERVAL_US,
                                 SAT_LOG_LEVEL_WARN, "gpredict:warn",
                                 "position stale reason=%s last_good_age_ms=%lld failures=%u enter_ms=%d exit_ms=%d",
                                 stale_reason,
                                 (long long)pos_age_ms,
                                 pos_failures,
                                 degraded_ms,
                                 warn_ms);
        }
        rot_session_set_state(ctrl, ROT_SESSION_DEGRADED,
                              stale_reason, FALSE);
        return;
    }

    if (ctrl->diagnostics_logged_generation != ctrl->engage_generation)
    {
        const RotCaps *caps = ctrl->client.client
                              ? rotctld_client_get_caps(ctrl->client.client)
                              : NULL;
        const gchar *device = ctrl->conf ? ctrl->conf->device : NULL;
        gint baud = ctrl->conf ? ctrl->conf->baud : 0;
        gdouble first_az = 0.0;
        gdouble first_el = 0.0;
        gboolean have_pos = FALSE;
        gboolean recovery = ctrl->client.client
                             ? rotctld_client_recovery_triggered(ctrl->client.client)
                             : FALSE;
        rot_target_wrap_mode_t wrap_mode =
            ctrl->user_limits.valid ? ctrl->user_limits.wrap_mode
                                    : (ctrl->conf && ctrl->conf->aztype == ROT_AZ_TYPE_180
                                       ? ROT_TARGET_WRAP_180
                                       : ROT_TARGET_WRAP_360);
        gchar first_pos_buf[64] = "unknown";
        const gchar *first_pos = first_pos_buf;

        g_mutex_lock(&ctrl->client.mutex);
        first_az = ctrl->client.azi_in;
        first_el = ctrl->client.ele_in;
        have_pos = (ctrl->client.last_pos_us > 0);
        g_mutex_unlock(&ctrl->client.mutex);

        if (ctrl->conf && baud <= 0 && ctrl->conf->last_good_baud > 0)
            baud = ctrl->conf->last_good_baud;

        if (have_pos)
            g_snprintf(first_pos_buf, sizeof(first_pos_buf),
                       "%.2f/%.2f", first_az, first_el);

        gdouble threshold_deg = (ctrl->threshold > 0.0)
                                    ? ctrl->threshold
                                    : 0.10;

        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "rotor diagnostics device=%s baud=%d model=%d wrap=%s first_pos=%s recovery=%d "
                    "poll=%dms stale_ms=%d threshold_deg=%.2f",
                    device ? device : "(none)",
                    baud,
                    caps ? caps->model_id : 0,
                    rotctrl_wrap_mode_name(wrap_mode),
                    first_pos,
                    recovery ? 1 : 0,
                    rotctrl_poll_period_ms(ctrl),
                    rotctrl_stale_ms(ctrl),
                    threshold_deg);
        rot_term_log(ctrl, "gpredict:rx",
                     "rotor diagnostics device=%s baud=%d model=%d wrap=%s "
                     "first_pos=%s recovery=%d poll=%dms stale_ms=%d threshold_deg=%.2f",
                     device ? device : "(none)",
                     baud,
                     caps ? caps->model_id : 0,
                     rotctrl_wrap_mode_name(wrap_mode),
                     first_pos,
                     recovery ? 1 : 0,
                     rotctrl_poll_period_ms(ctrl),
                     rotctrl_stale_ms(ctrl),
                     threshold_deg);
        ctrl->diagnostics_logged_generation = ctrl->engage_generation;
    }

    if (ctrl->session_state == ROT_SESSION_ENGAGING)
    {
        g_mutex_lock(&ctrl->client.mutex);
        ctrl->client.last_cmd_us = 0;
        ctrl->client.last_cmd_ok_az = 0.0;
        ctrl->client.last_cmd_ok_el = 0.0;
        ctrl->client.last_cmd_ok_us = 0;
        ctrl->client.last_cmd_backend_az = 0.0;
        ctrl->client.last_cmd_backend_el = 0.0;
        ctrl->client.last_cmd_backend_valid = FALSE;
        ctrl->client.last_set_attempt_us = 0;
        ctrl->client.force_pending = FALSE;
        ctrl->client.new_trg = FALSE;
        ctrl->client.azi_out = 0.0;
        ctrl->client.ele_out = 0.0;
        ctrl->client.raw_azi_out = 0.0;
        ctrl->client.raw_ele_out = 0.0;
        g_mutex_unlock(&ctrl->client.mutex);

        ctrl->last_cmd_time_us = 0;
        ctrl->last_cmd_valid = FALSE;
        ctrl->last_cmd_backend_az = 0.0;
        ctrl->last_cmd_backend_el = 0.0;
        ctrl->last_cmd_backend_valid = FALSE;
        ctrl->last_send_us = 0;

        if (ctrl->hold_position_on_engage &&
            !ctrl->hold_position_log_emitted)
        {
            ctrl->hold_position_log_emitted = TRUE;
            rot_term_log(ctrl, "gpredict:rx",
                         "engage: holding position (no initial move)");
        }
    }

    rotctrl_clear_ui_hard_error(ctrl);
    rotctrl_set_ui_status_detail(ctrl, NULL);
    rot_session_set_state(ctrl, ROT_SESSION_READY, "ready", FALSE);
}

static void rotctld_log_cb(RotctldMgr *mgr,
                           const gchar *prefix,
                           const gchar *line,
                           gpointer user_data)
{
    GtkRotCtrl *ctrl = user_data;

    (void)mgr;

    if (ctrl == NULL || prefix == NULL || line == NULL)
        return;

    if (!ctrl->verbose_logging)
        return;

    rot_term_log(ctrl, prefix, "%s", line);
}

static void G_GNUC_UNUSED rot_log_out_of_range(GtkRotCtrl *ctrl,
                                               gdouble raw_az, gdouble raw_el,
                                               const rot_cmd_map_t *map)
{
    if (ctrl == NULL || ctrl->conf == NULL || map == NULL)
        return;

    gint64 now = g_get_monotonic_time();
    gboolean same_target =
        fabs(raw_az - ctrl->last_oob_raw_az) < 1e-3 &&
        fabs(raw_el - ctrl->last_oob_raw_el) < 1e-3 &&
        fabs(map->mapped_az - ctrl->last_oob_mapped_az) < 1e-3 &&
        fabs(map->mapped_el - ctrl->last_oob_mapped_el) < 1e-3;

    if (same_target && (now - ctrl->last_oob_log_us) < 2000000)
        return;

    ctrl->last_oob_log_us = now;
    ctrl->last_oob_raw_az = raw_az;
    ctrl->last_oob_raw_el = raw_el;
    ctrl->last_oob_mapped_az = map->mapped_az;
    ctrl->last_oob_mapped_el = map->mapped_el;

    const gchar *axis_mode =
        (ctrl->conf->axis_mode == ROT_AXIS_MODE_AZ_ONLY) ? "AZ_ONLY" : "AZ_EL";

    sat_log_log(SAT_LOG_LEVEL_WARN,
                "%s: OUT_OF_RANGE raw=(%.2f, %.2f) mapped=(%.2f, %.2f) send=(%.2f, %.2f) "
                "conf={axis=%s aztype=%d minaz=%.2f maxaz=%.2f minel=%.2f maxel=%.2f "
                "azstop=%.2f use_offset=%d az_off=%.2f el_off=%.2f invert_az=%d invert_el=%d}",
                __func__,
                raw_az, raw_el,
                map->mapped_az, map->mapped_el,
                map->send_az, map->send_el,
                axis_mode, ctrl->conf->aztype,
                ctrl->conf->minaz, ctrl->conf->maxaz,
                ctrl->conf->minel, ctrl->conf->maxel,
                ctrl->conf->azstoppos,
                ctrl->conf->use_offset ? 1 : 0,
                ctrl->conf->az_offset, ctrl->conf->el_offset,
                ctrl->conf->invert_az ? 1 : 0,
                ctrl->conf->invert_el ? 1 : 0);

    rot_term_log(ctrl, "gpredict:err",
                 "OUT_OF_RANGE raw=(%.2f, %.2f) mapped=(%.2f, %.2f) axis=%s",
                 raw_az, raw_el,
                 map->mapped_az, map->mapped_el,
                 axis_mode);
}


/* Open the rotctld transport. Returns 0 on success, -1 on failure. */
static gint rotctld_socket_open(GtkRotCtrl *ctrl, const gchar *host, gint port)
{
    gchar *error = NULL;
    gboolean ok = FALSE;

    if (ctrl == NULL || host == NULL)
        return -1;

    if (ctrl->client.client == NULL)
        ctrl->client.client = rotctld_client_new("rotctld");

    ok = rotctld_client_connect(ctrl->client.client, host, port,
                                ROTCTLD_SOCKET_TIMEOUT_MS, &error);
    if (!ok)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("Connection to rotctld server at %s:%d failed: %s"),
                    host, port, error ? error : "(unknown)");
        g_free(error);
        return -1;
    }

    g_free(error);
    ctrl->client.conn_id = ++rotctld_conn_seq;
    sat_log_log(SAT_LOG_LEVEL_INFO,
                "rotctld connection opened id=%" G_GUINT64_FORMAT " host=%s port=%d",
                ctrl->client.conn_id, host, port);
    rot_term_log_verbose(ctrl, "gpredict:rx",
                         "rotctld connection opened id=%" G_GUINT64_FORMAT,
                         ctrl->client.conn_id);
    return 0;
}

/* Close a rotcld socket. First send a q command to cleanly shut down rotctld */
static void rotctld_socket_close(GtkRotCtrl *ctrl, gint * sock)
{
    gchar           reply[64];

    if (sock == NULL || *sock == -1)
        return;

    if (ctrl != NULL)
        rot_term_log_tx(ctrl, "q");

    if (ctrl != NULL && ctrl->client.client != NULL)
        (void)rotctld_client_request_raw(ctrl->client.client, "q\n",
                                         reply, sizeof(reply), NULL);

    if (ctrl != NULL && ctrl->client.client != NULL)
        rotctld_client_close(ctrl->client.client);

    *sock = -1;

    if (ctrl != NULL && ctrl->client.conn_id != 0)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "rotctld connection closed id=%" G_GUINT64_FORMAT,
                    ctrl->client.conn_id);
        rot_term_log_verbose(ctrl, "gpredict:rx",
                             "rotctld connection closed id=%" G_GUINT64_FORMAT,
                             ctrl->client.conn_id);
        ctrl->client.conn_id = 0;
    }

    if (ctrl != NULL)
        rotctld_clear_rxbuf(ctrl);
}

/* Close a rotctld socket without sending shutdown commands. */
static void rotctld_socket_close_quiet(GtkRotCtrl *ctrl, gint *sock)
{
    if (sock == NULL)
        return;

    if (ctrl != NULL && ctrl->client.client != NULL)
        rotctld_client_close(ctrl->client.client);

    *sock = -1;

    if (ctrl != NULL && ctrl->client.conn_id != 0)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "rotctld connection closed id=%" G_GUINT64_FORMAT,
                    ctrl->client.conn_id);
        rot_term_log_verbose(ctrl, "gpredict:rx",
                             "rotctld connection closed id=%" G_GUINT64_FORMAT,
                             ctrl->client.conn_id);
        ctrl->client.conn_id = 0;
    }
}

static GString *G_GNUC_UNUSED rotctld_rxbuf_get(GtkRotCtrl *ctrl,
                                                gboolean create)
{
    if (ctrl == NULL)
        return NULL;

    if (ctrl->client.rxbuf == NULL && create)
        ctrl->client.rxbuf = g_string_sized_new(256);

    return ctrl->client.rxbuf;
}

static void rotctld_rxbuf_clear(GString *buf)
{
    if (buf == NULL)
        return;

    g_string_set_size(buf, 0);
}

static void rotctld_clear_rxbuf(GtkRotCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    if (ctrl->client.client)
        (void)rotctld_client_clear_rxbuf(ctrl->client.client);

    rotctld_rxbuf_clear(ctrl->client.rxbuf);
}

static void rotctld_drain_transport(GtkRotCtrl *ctrl, gint idle_timeout_ms)
{
    HamlibTransport *transport = NULL;

    if (ctrl == NULL || ctrl->client.client == NULL)
        return;

    transport = rotctld_client_get_transport(ctrl->client.client);
    if (transport == NULL)
        return;

    (void)hamlib_transport_drain(transport, idle_timeout_ms, NULL);
}

static gboolean rotctld_reply_is_dump_state(const gchar *reply)
{
    if (reply == NULL || *reply == '\0')
        return FALSE;

    if (g_str_has_prefix(reply, "1\n") || g_str_has_prefix(reply, "1\r\n"))
    {
        if (strstr(reply, "min_az=") || strstr(reply, "rot_type="))
            return TRUE;
    }

    if (strstr(reply, "min_az=") ||
        strstr(reply, "max_az=") ||
        strstr(reply, "rot_type=") ||
        strstr(reply, "rot_model=") ||
        strstr(reply, "south_zero="))
        return TRUE;

    return FALSE;
}

static void rotctld_note_io_ok(GtkRotCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    g_mutex_lock(&ctrl->client.mutex);
    ctrl->client.last_io_ok_us = g_get_monotonic_time();
    ctrl->client.consecutive_failures = 0;
    g_mutex_unlock(&ctrl->client.mutex);
}

static gboolean rotctld_note_io_failure(GtkRotCtrl *ctrl, const gchar *context)
{
    gint64 now_us;
    gint64 last_ok;
    guint failures;
    gboolean stale = FALSE;
    gboolean threshold = FALSE;
    gboolean log_ok = FALSE;

    if (ctrl == NULL)
        return TRUE;

    now_us = g_get_monotonic_time();
    g_mutex_lock(&ctrl->client.mutex);
    ctrl->client.consecutive_failures++;
    failures = ctrl->client.consecutive_failures;
    last_ok = ctrl->client.last_io_ok_us;
    if (ctrl->client.last_failure_log_us == 0 ||
        (now_us - ctrl->client.last_failure_log_us) > ROTCTLD_FAILURE_LOG_INTERVAL_US)
    {
        ctrl->client.last_failure_log_us = now_us;
        log_ok = TRUE;
    }
    g_mutex_unlock(&ctrl->client.mutex);

    if (last_ok > 0 &&
        (now_us - last_ok) > ((gint64)ROTCTLD_IO_STALE_MS * 1000))
        stale = TRUE;

    if (failures >= ROTCTLD_MAX_CONSEC_FAIL)
        threshold = TRUE;

    if (log_ok)
    {
        sat_log_log(SAT_LOG_LEVEL_WARN,
                    "%s: rotctld io failure (%s) failures=%u last_ok=%.1fs ago",
                    __func__,
                    context ? context : "unknown",
                    failures,
                    last_ok > 0 ? (now_us - last_ok) / 1e6 : -1.0);
    }

    return (threshold || stale);
}

static void rotctld_backend_ioerr_reset(GtkRotCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    g_mutex_lock(&ctrl->client.mutex);
    ctrl->client.backend_ioerr_count = 0;
    ctrl->client.backend_ioerr_first_us = 0;
    ctrl->client.backend_ioerr_disengage_pending = FALSE;
    g_mutex_unlock(&ctrl->client.mutex);
}

static gboolean rotctld_backend_ioerr_note(GtkRotCtrl *ctrl,
                                           const gchar *context)
{
    gint64 now_us = 0;
    guint count = 0;
    gboolean trigger = FALSE;
    gboolean log_ok = FALSE;

    if (ctrl == NULL)
        return FALSE;

    now_us = g_get_monotonic_time();

    g_mutex_lock(&ctrl->client.mutex);
    if (ctrl->client.backend_ioerr_first_us == 0 ||
        (now_us - ctrl->client.backend_ioerr_first_us) >
        ROTCTRL_BACKEND_IO_WINDOW_US)
    {
        ctrl->client.backend_ioerr_first_us = now_us;
        ctrl->client.backend_ioerr_count = 0;
    }

    ctrl->client.backend_ioerr_count++;
    count = ctrl->client.backend_ioerr_count;

    if (!ctrl->client.backend_ioerr_disengage_pending &&
        count >= ROTCTRL_MAX_BACKEND_IOERR)
    {
        ctrl->client.backend_ioerr_disengage_pending = TRUE;
        trigger = TRUE;
        if (ctrl->client.backend_ioerr_last_log_us == 0 ||
            (now_us - ctrl->client.backend_ioerr_last_log_us) >
            ROTCTLD_FAILURE_LOG_INTERVAL_US)
        {
            ctrl->client.backend_ioerr_last_log_us = now_us;
            log_ok = TRUE;
        }
    }
    g_mutex_unlock(&ctrl->client.mutex);

    if (trigger && log_ok)
    {
        sat_log_log(SAT_LOG_LEVEL_WARN,
                    "rotor backend I/O error persists; marking DEGRADED");
        rot_term_log(ctrl, "gpredict:err",
                     "rotor backend I/O error persists; marking DEGRADED");
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rotor backend error context=%s count=%u",
                    context ? context : "unknown",
                    count);
    }

    return trigger;
}

static gboolean rotctld_io_recent(GtkRotCtrl *ctrl, gint64 window_us)
{
    gint64 last_ok = 0;
    gint64 now_us = g_get_monotonic_time();

    if (ctrl == NULL)
        return FALSE;

    g_mutex_lock(&ctrl->client.mutex);
    last_ok = ctrl->client.last_io_ok_us;
    g_mutex_unlock(&ctrl->client.mutex);

    if (last_ok <= 0)
        return FALSE;

    return (now_us - last_ok) <= window_us;
}

static gboolean rotctld_stop_requested(GtkRotCtrl *ctrl)
{
    if (ctrl == NULL)
        return TRUE;
    return g_atomic_int_get(&ctrl->client.stop_requested) != 0;
}

static guint64 rotctld_get_engage_generation(GtkRotCtrl *ctrl)
{
    guint64 generation = 0;

    if (ctrl == NULL)
        return 0;

    g_mutex_lock(&ctrl->client.mutex);
    generation = ctrl->engage_generation;
    g_mutex_unlock(&ctrl->client.mutex);

    return generation;
}

static gboolean rotctld_generation_stale(GtkRotCtrl *ctrl, guint64 generation)
{
    return rotctld_get_engage_generation(ctrl) != generation;
}

static void rotctld_request_thread_stop(GtkRotCtrl *ctrl, gboolean send_quit)
{
    gboolean thread_running = FALSE;

    if (ctrl == NULL)
        return;

    g_atomic_int_set(&ctrl->client.stop_requested, 1);

    g_mutex_lock(&ctrl->client.mutex);
    thread_running = (ctrl->client.thread != NULL);
    ctrl->client.new_trg = FALSE;
    ctrl->client.allow_send_no_pos = FALSE;
    ctrl->client.apply_calib = FALSE;
    if (send_quit)
        ctrl->client.send_quit = TRUE;
    ctrl->client.running = FALSE;
    g_mutex_unlock(&ctrl->client.mutex);

    if (!thread_running && ctrl->client.client != NULL)
        rotctld_client_close(ctrl->client.client);
}

static gboolean rotctld_collect_client_thread(GtkRotCtrl *ctrl,
                                              gboolean allow_block,
                                              const gchar *context)
{
    GThread *thread = NULL;

    if (ctrl == NULL)
        return TRUE;

    thread = ctrl->client.thread;
    if (thread == NULL)
        return TRUE;

#if GLIB_CHECK_VERSION(2, 32, 0)
    if (!allow_block)
    {
        gboolean thread_done = FALSE;
        gboolean running = FALSE;

        g_mutex_lock(&ctrl->client.mutex);
        thread_done = ctrl->client.thread_done;
        running = ctrl->client.running;
        g_mutex_unlock(&ctrl->client.mutex);
        if (!thread_done || running)
            return FALSE;
        g_thread_unref(thread);
    }
    else
#endif
    {
        g_thread_join(thread);
    }

    ctrl->client.thread = NULL;
    ctrl->client.socket = -1;
    ctrl->client.thread_generation = 0;
    ctrl->client.running = FALSE;
    if (context && *context)
        rot_term_log_verbose(ctrl, "gpredict:rx",
                             "rotctld client thread joined (%s)",
                             context);
    return TRUE;
}

static void rotctld_sleep_us(GtkRotCtrl *ctrl, gint64 usec)
{
    const gint64 slice_us = 100000;
    gint64 remaining = usec;

    while (remaining > 0)
    {
        if (rotctld_stop_requested(ctrl))
            break;
        gint64 step = MIN(remaining, slice_us);
        g_usleep((gulong)step);
        remaining -= step;
    }
}

static gboolean rotctld_rxbuf_find_line(const GString *buf, gsize *line_len)
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

static gssize rotctld_rxbuf_take_line(GString *buf, gchar *out, gsize out_len)
{
    gsize line_len = 0;
    gsize copy_len = 0;

    if (!rotctld_rxbuf_find_line(buf, &line_len))
        return 0;

    if (out == NULL || out_len == 0)
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

static gint rotctld_poll_readable(int fd, gint timeout_ms, gint *err_out)
{
    GPollFD pfd;
    gint rc;

    if (err_out)
        *err_out = 0;

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

static gboolean rotctld_line_is_rprt(const gchar *line)
{
    const gchar *scan = line;

    if (scan == NULL)
        return FALSE;

    while (*scan != '\0' && g_ascii_isspace(*scan))
        scan++;

    return g_str_has_prefix(scan, "RPRT ");
}

static gboolean rotctld_line_is_done(const gchar *line)
{
    const gchar *scan = line;

    if (scan == NULL)
        return FALSE;

    while (*scan != '\0' && g_ascii_isspace(*scan))
        scan++;

    return g_ascii_strcasecmp(scan, "done") == 0;
}

static gssize rotctld_read_reply_line(int fd,
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

    deadline_us = g_get_monotonic_time() + ((gint64)timeout_ms * 1000);

    for (;;)
    {
        gssize copied = rotctld_rxbuf_take_line(buf, out, out_len);
        if (copied > 0)
            return copied;
        if (copied < 0)
        {
            if (err_out)
                *err_out = EINVAL;
            return -1;
        }

        gint64 now_us = g_get_monotonic_time();
        gint64 remaining_us = deadline_us - now_us;
        gint remaining_ms;
        gint poll_rc;
        gchar chunk[512];
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

        poll_rc = rotctld_poll_readable(fd, remaining_ms, err_out);
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

typedef enum {
    ROTCTLD_READ_SINGLE = 0,
    ROTCTLD_READ_MULTILINE,
    ROTCTLD_READ_DUMP_STATE
} rotctld_read_mode_t;

static gssize G_GNUC_UNUSED rotctld_read_response(int fd,
                                                  GString *buf,
                                                  rotctld_read_mode_t mode,
                                                  gchar *out,
                                                  gsize out_len,
                                                  gint base_timeout_ms,
                                                  gint idle_timeout_ms,
                                                  gboolean *saw_rprt,
                                                  gint *err_out)
{
    gchar line[512];
    gssize size;
    gsize used = 0;
    gboolean saw_term = FALSE;
    gboolean multi = (mode != ROTCTLD_READ_SINGLE);
    gint err = 0;
    gint lines = 0;
    gint min_lines = (mode == ROTCTLD_READ_SINGLE) ? 1 : 2;

    if (saw_rprt)
        *saw_rprt = FALSE;
    if (err_out)
        *err_out = 0;

    if (out == NULL || out_len == 0)
    {
        if (err_out)
            *err_out = EINVAL;
        return -1;
    }

    out[0] = '\0';

    size = rotctld_read_reply_line(fd, buf, line, sizeof(line),
                                   base_timeout_ms, &err);
    if (size <= 0)
    {
        if (err_out)
            *err_out = err;
        return size;
    }

    used = g_strlcat(out, line, out_len);
    lines++;
    if (rotctld_line_is_rprt(line) || rotctld_line_is_done(line))
        saw_term = TRUE;

    if (mode == ROTCTLD_READ_SINGLE && !multi)
    {
        if (buf != NULL && buf->len > 0)
            multi = TRUE;
        else if (!saw_term && idle_timeout_ms > 0)
        {
            gint poll_rc = rotctld_poll_readable(fd, idle_timeout_ms, &err);
            if (poll_rc < 0)
            {
                if (err_out)
                    *err_out = err;
                return -1;
            }
            if (poll_rc > 0)
                multi = TRUE;
        }
    }

    if (!multi)
    {
        if (saw_rprt)
            *saw_rprt = saw_term;
        return (gssize) used;
    }

    while (!saw_term)
    {
        size = rotctld_read_reply_line(fd, buf, line, sizeof(line),
                                       idle_timeout_ms, &err);
        if (size <= 0)
        {
            if (size < 0 && err != EAGAIN && err != EWOULDBLOCK)
            {
                if (err_out)
                    *err_out = err;
                return -1;
            }
            if (lines >= min_lines)
                break;
            if (err_out)
                *err_out = err;
            return -1;
        }

        used = g_strlcat(out, line, out_len);
        lines++;
        if (rotctld_line_is_rprt(line) || rotctld_line_is_done(line))
            saw_term = TRUE;
    }

    if (saw_rprt)
        *saw_rprt = saw_term;

    return (gssize) used;
}

/*
 * Send a command to rotctld and read the response.
 *
 * Inputs are the socket, a string command, and a buffer and length for
 * returning the output from rotctld.
 */
static gboolean rotctld_socket_rw(GtkRotCtrl *ctrl, gint sock,
                                  const gchar *buff, gchar *buffout,
                                  gint sizeout)
{
    HamlibResponseInfo info = { 0 };
    const gchar    *host = (ctrl && ctrl->conf && ctrl->conf->host)
                           ? ctrl->conf->host
                           : "(null)";
    gint            port = (ctrl && ctrl->conf) ? ctrl->conf->port : 0;
    gboolean        ok = FALSE;

    (void)sock;

    if (ctrl != NULL)
        rot_term_log_tx(ctrl, buff);

    if (ctrl == NULL || ctrl->client.client == NULL)
        return FALSE;

    if (buffout && sizeout > 0)
        buffout[0] = '\0';

    ok = rotctld_client_request_raw(ctrl->client.client, buff,
                                    buffout, (gsize) sizeout, &info);
    if (!ok)
    {
        if (ctrl != NULL)
        {
            gchar *cmd = g_strdup(buff ? buff : "");
            g_strchomp(cmd);
            rot_term_log(ctrl, "gpredict:err",
                         "%s failed: command '%s' to %s:%d",
                         __func__, cmd, host, port);
            g_free(cmd);
        }
        return FALSE;
    }

    if (ctrl != NULL)
        rot_term_log_rx(ctrl, buff, buffout);

    if (ctrl != NULL)
        ctrl->client.last_rtt_us = rotctld_client_last_rtt_us(ctrl->client.client);

    return TRUE;
}

static gint sat_name_compare(sat_t * a, sat_t * b)
{
    const gchar *an = "";
    const gchar *bn = "";

    if (a != NULL)
        an = (a->nickname != NULL) ? a->nickname : (a->name ? a->name : "");
    if (b != NULL)
        bn = (b->nickname != NULL) ? b->nickname : (b->name ? b->name : "");

    return gpredict_strcmp(an, bn);
}

static gint rot_name_compare(const gchar * a, const gchar * b)
{
    return (gpredict_strcmp(a, b));
}

static gboolean is_flipped_pass(pass_t * pass, rot_az_type_t type,
                                gdouble azstoppos)
{
    gdouble         max_az = 0, min_az = 0, offset = 0;
    gdouble         caz, last_az = pass->aos_az;
    guint           num, i;
    pass_detail_t  *detail;
    gboolean        retval = FALSE;

    num = g_slist_length(pass->details);
    if (type == ROT_AZ_TYPE_360 || type == ROT_AZ_TYPE_480)
    {
        min_az = 0;
        max_az = 360;
    }
    else if (type == ROT_AZ_TYPE_180)
    {
        min_az = -180;
        max_az = 180;
    }

    /* Offset by (azstoppos-min_az) to handle
     * rotators with non-default positions.
     * Note that the default positions of the rotator stops
     * (eg. -180 for ROT_AZ_TYPE_180, and 0 for 
     * ROT_AZ_TYPE_360) will create an offset of 0, which
     * seems like a pretty sane default. */
    offset = azstoppos - min_az;
    min_az += offset;
    max_az += offset;

    /* Assume that min_az and max_az are atleat 360 degrees apart
       get the azimuth that is in a settable range */
    while (last_az > max_az)
        last_az -= 360;

    while (last_az < min_az)
        last_az += 360;

    if (num > 1)
    {
        for (i = 1; i < num - 1; i++)
        {
            detail = PASS_DETAIL(g_slist_nth_data(pass->details, i));
            caz = detail->az;

            while (caz > max_az)
                caz -= 360;

            while (caz < min_az)
                caz += 360;


            if (fabs(caz - last_az) > 180)
                retval = TRUE;

            last_az = caz;
        }
    }
    caz = pass->los_az;
    while (caz > max_az)
        caz -= 360;

    while (caz < min_az)
        caz += 360;

    if (fabs(caz - last_az) > 180)
        retval = TRUE;

    return retval;
}

static void rot_plan_reset(rot_plan_t *plan)
{
    if (plan == NULL)
        return;

    if (plan->reason) {
        g_free(plan->reason);
        plan->reason = NULL;
    }

    if (plan->cmd_samples) {
        g_array_free(plan->cmd_samples, TRUE);
        plan->cmd_samples = NULL;
    }

    plan->valid = FALSE;
    plan->mode = ROT_PLAN_MODE_NORMAL;
    plan->crosses_endstop = FALSE;
    plan->status = ROT_PLAN_STATUS_FULL_TRACK;
    plan->strategy = ROT_PLAN_STRATEGY_S1;
    plan->start_az_cmd = 0.0;
    plan->start_el_cmd = 0.0;
    plan->end_az_cmd = 0.0;
    plan->end_el_cmd = 0.0;
    plan->total_motion = 0.0;
    plan->peak_az_rate = 0.0;
    plan->peak_el_rate = 0.0;
    plan->window_start = 0.0;
    plan->window_end = 0.0;
    plan->trackable_pct = 0.0;
    plan->violation_mag = 0.0;
    plan->sample_dt_sec = ROT_PLAN_SAMPLE_DT_SEC;
}

static void rotctrl_plan_log_mark(GtkRotCtrl *ctrl)
{
    if (ctrl == NULL || !ctrl->trajectory_plan.valid)
    {
        if (ctrl)
            ctrl->plan_log_pending = FALSE;
        return;
    }

    if (ctrl->plan_log_window_start == ctrl->trajectory_plan.window_start &&
        ctrl->plan_log_window_end == ctrl->trajectory_plan.window_end &&
        ctrl->plan_log_mode == ctrl->trajectory_plan.mode)
        return;

    ctrl->plan_log_window_start = ctrl->trajectory_plan.window_start;
    ctrl->plan_log_window_end = ctrl->trajectory_plan.window_end;
    ctrl->plan_log_mode = ctrl->trajectory_plan.mode;
    ctrl->plan_log_crosses_endstop = ctrl->trajectory_plan.crosses_endstop;
    ctrl->plan_log_pending = TRUE;
}

static void rotctrl_plan_log_if_pending(GtkRotCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    if (!ctrl->trajectory_plan.valid)
    {
        ctrl->plan_log_pending = FALSE;
        return;
    }

    if (!ctrl->plan_log_pending)
        return;

    {
        gdouble entry_t = 0.0;
        gboolean entry_ok = rotctrl_find_entry_time(&ctrl->trajectory_plan,
                                                    ctrl->t,
                                                    ctrl->conf->minel,
                                                    &entry_t);
        gdouble entry_sec = entry_ok ? (entry_t - ctrl->t) * secday : 0.0;
        const gchar *reason =
            ctrl->trajectory_plan.crosses_endstop
                ? "cross_endstop"
                : "no_cross";
        const gchar *entry_reason = entry_ok ? "ok" : "no_entry";
        gint64 now_us = g_get_monotonic_time();
        gboolean debounce = FALSE;

        if (entry_ok && entry_sec < 0.0)
        {
            entry_sec = 0.0;
            entry_reason = "entry_in_past";
        }

        if (ctrl->last_tracking_log_us > 0 &&
            (now_us - ctrl->last_tracking_log_us) < 1000000 &&
            ctrl->last_tracking_log_mode == ctrl->trajectory_plan.mode &&
            ctrl->last_tracking_log_cross_endstop == ctrl->trajectory_plan.crosses_endstop &&
            g_strcmp0(ctrl->last_tracking_log_entry_reason, entry_reason) == 0)
            debounce = TRUE;

        if (debounce)
        {
            ctrl->plan_log_pending = FALSE;
            return;
        }

        rot_term_log(ctrl, "gpredict",
                     "tracking start mode=%s reason=%s entry_t=%.1fs entry_reason=%s",
                     rotctrl_tracking_mode_name(ctrl->trajectory_plan.mode),
                     reason,
                     entry_sec,
                     entry_reason);
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "tracking start mode=%s reason=%s entry_t=%.1fs entry_reason=%s",
                    rotctrl_tracking_mode_name(ctrl->trajectory_plan.mode),
                    reason,
                    entry_sec,
                    entry_reason);
        ctrl->last_tracking_log_us = now_us;
        ctrl->last_tracking_log_mode = ctrl->trajectory_plan.mode;
        ctrl->last_tracking_log_cross_endstop = ctrl->trajectory_plan.crosses_endstop;
        g_strlcpy(ctrl->last_tracking_log_entry_reason, entry_reason,
                  sizeof(ctrl->last_tracking_log_entry_reason));
    }

    ctrl->plan_log_pending = FALSE;
}

static inline gboolean rot_plan_matches_pass(GtkRotCtrl *ctrl)
{
    if (ctrl == NULL || ctrl->pass == NULL || !ctrl->trajectory_plan.valid)
        return FALSE;

    return (fabs(ctrl->trajectory_plan.window_start - ctrl->pass->aos) < 1e-4 &&
            fabs(ctrl->trajectory_plan.window_end - ctrl->pass->los) < 1e-4);
}

static inline void set_flipped_pass(GtkRotCtrl * ctrl)
{
    if (rot_plan_matches_pass(ctrl)) {
        ctrl->flipped = (ctrl->trajectory_plan.mode == ROT_PLAN_MODE_FLIP);
        return;
    }

    if (ctrl->conf && ctrl->pass)
        ctrl->flipped = is_flipped_pass(ctrl->pass, ctrl->conf->aztype,
                                        ctrl->conf->azstoppos);
    else
        ctrl->flipped = FALSE;
}

static void rot_get_abs_az_limits(const rotor_conf_t *conf,
                                  gdouble *az_min, gdouble *az_max)
{
    gdouble minaz = 0.0;
    gdouble maxaz = 360.0;
    gdouble span;

    if (conf != NULL)
    {
        minaz = conf->minaz;
        maxaz = conf->maxaz;
    }

    span = maxaz - minaz;
    if (span <= 0.0)
        span += 360.0;

    if (span > 360.0 + 1e-6)
    {
        if (az_min)
            *az_min = minaz;
        if (az_max)
            *az_max = maxaz;
        return;
    }

    if (span >= 359.0)
    {
        minaz = 0.0;
        maxaz = 360.0;
    }
    else
    {
        minaz = normalize_az_0_360(minaz);
        maxaz = normalize_az_0_360(maxaz);
    }

    if (az_min)
        *az_min = minaz;
    if (az_max)
        *az_max = maxaz;
}

static gdouble rot_az_to_conf(const rotor_conf_t *conf, gdouble az_abs)
{
    return rot_cmd_az_to_conf(conf, az_abs);
}

static gboolean rot_conf_validate(const rotor_conf_t *conf, gchar **msg)
{
    if (msg)
        *msg = NULL;

    if (conf == NULL) {
        if (msg)
            *msg = g_strdup("Missing rotor configuration");
        return FALSE;
    }

    if (conf->host == NULL || conf->host[0] == '\0') {
        if (msg)
            *msg = g_strdup("Rotator host is not set");
        return FALSE;
    }

    if (conf->port <= 0 || conf->port > 65535) {
        if (msg)
            *msg = g_strdup_printf("Rotator port %d is invalid", conf->port);
        return FALSE;
    }

    return TRUE;
}

static gboolean rot_plan_get_cmd_at_time(const rot_plan_t *plan, gdouble t,
                                         gdouble *az, gdouble *el)
{
    if (plan == NULL || !plan->cmd_samples || plan->cmd_samples->len == 0)
        return FALSE;

    guint idx = 0;
    if (t <= plan->window_start) {
        idx = 0;
    } else if (t >= plan->window_end) {
        idx = plan->cmd_samples->len - 1;
    } else {
        gdouble dt = plan->sample_dt_sec > 0.0
                     ? plan->sample_dt_sec
                     : ROT_PLAN_SAMPLE_DT_SEC;
        gdouble offset = (t - plan->window_start) * secday;
        idx = (guint)floor(offset / dt);
        if (idx >= plan->cmd_samples->len)
            idx = plan->cmd_samples->len - 1;
    }

    rot_plan_cmd_t cmd = g_array_index(plan->cmd_samples, rot_plan_cmd_t, idx);
    if (az)
        *az = cmd.cmd_az;
    if (el)
        *el = cmd.cmd_el;
    return TRUE;
}

static gboolean G_GNUC_UNUSED rot_get_command(GtkRotCtrl *ctrl, gboolean plan_active,
                                              gdouble setaz, gdouble setel, gdouble t,
                                              gdouble *cmdaz, gdouble *cmdel)
{
    gboolean from_plan = FALSE;
    gdouble az = setaz;
    gdouble el = setel;

    if (plan_active && ctrl->trajectory_plan.valid) {
        if (rot_plan_get_cmd_at_time(&ctrl->trajectory_plan, t, &az, &el))
            from_plan = TRUE;
    }

    if (cmdaz)
        *cmdaz = az;
    if (cmdel)
        *cmdel = el;

    return from_plan;
}

static rot_cmd_map_status_t G_GNUC_UNUSED rotctrl_map_command(GtkRotCtrl *ctrl,
                                                              gdouble raw_az,
                                                              gdouble raw_el,
                                                              gdouble current_az,
                                                              gboolean have_current,
                                                              rot_cmd_map_t *out)
{
    RotTransformPre pre = { 0 };
    SpanConfig span;
    AzElMapResult map = { 0 };
    gboolean caps_valid = FALSE;
    gdouble caps_az_min = 0.0;
    gdouble caps_az_max = 0.0;
    gdouble caps_el_min = 0.0;
    gdouble caps_el_max = 0.0;

    if (out != NULL)
    {
        out->mapped_az = raw_az;
        out->mapped_el = raw_el;
        out->send_az = raw_az;
        out->send_el = raw_el;
    }

    if (ctrl == NULL || ctrl->conf == NULL || out == NULL)
        return ROT_CMD_MAP_INVALID_CONF;

    rot_transform_update(ctrl);

    if (ctrl->conf->minel > ctrl->conf->maxel)
        return ROT_CMD_MAP_INVALID_CONF;

    if (ctrl->conf->axis_mode == ROT_AXIS_MODE_AZ_ONLY)
        raw_el = ctrl->conf->minel;

    rot_transform_prepare(&ctrl->transform, raw_az, raw_el, &pre);

    g_mutex_lock(&ctrl->client.mutex);
    if (ctrl->client.limits_valid)
    {
        caps_valid = TRUE;
        caps_az_min = ctrl->client.az_min;
        caps_az_max = ctrl->client.az_max;
        caps_el_min = ctrl->client.el_min;
        caps_el_max = ctrl->client.el_max;
    }
    g_mutex_unlock(&ctrl->client.mutex);

    span = azel_span_from_rotor_conf(ctrl->conf,
                                     caps_az_min, caps_az_max,
                                     caps_el_min, caps_el_max,
                                     caps_valid);

    (void)azel_map(&span, pre.az_abs, pre.el_raw,
                   current_az, have_current, &map);

    out->mapped_az = az_abs_to_span(pre.az_abs,
                                    rotctrl_span_from_conf(ctrl->conf));
    out->mapped_el = pre.el_raw;
    out->send_az = map.cmd_az;
    out->send_el = map.cmd_el;

    if (map.az_clamped || map.el_clamped)
        return ROT_CMD_MAP_OUT_OF_RANGE;

    return ROT_CMD_MAP_OK;
}

static void rotctrl_apply_zenith_guard(GtkRotCtrl *ctrl,
                                       gdouble sky_el,
                                       gdouble rotaz_abs,
                                       gboolean rotpos_valid,
                                       AzSpan cmd_span_mode,
                                       gdouble *cmd_az)
{
    gdouble guard_on;
    gdouble guard_off;
    gdouble hold_value = 0.0;

    if (ctrl == NULL || cmd_az == NULL)
        return;

    if (!ctrl->transform.zenith_guard_enable)
        return;

    if (ctrl->conf != NULL &&
        ctrl->conf->axis_mode == ROT_AXIS_MODE_AZ_ONLY)
        return;

    guard_on = ctrl->transform.zenith_guard_el_deg;
    guard_off = guard_on - 2.0;

    if (!ctrl->az_hold_active && sky_el >= guard_on)
    {
        if (rotpos_valid)
        {
            if (ctrl->span_extended)
                hold_value = rotaz_abs;
            else
                hold_value = az_abs_to_span(rotaz_abs, cmd_span_mode);
        }
        else
        {
            hold_value = *cmd_az;
        }

        ctrl->az_hold_active = TRUE;
        ctrl->az_hold_value = hold_value;
        if (ctrl->verbose_logging)
        {
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "zenith guard ON sky_el=%.2f hold_az=%.2f",
                        sky_el, hold_value);
        }
    }
    else if (ctrl->az_hold_active && sky_el < guard_off)
    {
        ctrl->az_hold_active = FALSE;
        if (ctrl->verbose_logging)
        {
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "zenith guard OFF sky_el=%.2f",
                        sky_el);
        }
    }

    if (ctrl->az_hold_active)
        *cmd_az = ctrl->az_hold_value;
}

static GArray *rot_build_samples(GtkRotCtrl *ctrl, pass_t *pass,
                                 rot_plan_mode_t mode,
                                 gdouble t0, gdouble t1)
{
    (void)pass;
    GArray *samples = NULL;
    guint steps;

    steps = (guint)ceil(((t1 - t0) * secday) / ROT_PLAN_SAMPLE_DT_SEC);
    if (steps < 1)
        steps = 1;

    samples = g_array_new(FALSE, FALSE, sizeof(rot_plan_sample_t));

    for (guint i = 0; i <= steps; i++) {
        gdouble t = t0 + (i * ROT_PLAN_SAMPLE_DT_SEC) / secday;
        gdouble az_cmd;
        gdouble el_cmd;

        if (t > t1)
            t = t1;

        sat_t sat = *ctrl->target;
        predict_calc(&sat, ctrl->qth, t);

        az_cmd = sat.az;
        el_cmd = sat.el;

        if (mode == ROT_PLAN_MODE_FLIP) {
            el_cmd = 180.0 - el_cmd;
            az_cmd += 180.0;
        }

        az_cmd = normalize_az_0_360(az_cmd);

        rot_plan_sample_t sample = { t, az_cmd, el_cmd };
        g_array_append_val(samples, sample);
    }

    return samples;
}

static gboolean rot_build_tracking_plan(GtkRotCtrl *ctrl)
{
    rot_plan_result_t normal = { 0 };
    rot_plan_result_t flip = { 0 };
    gboolean flip_allowed = FALSE;
    gboolean have_normal = FALSE;
    gboolean have_flip = FALSE;
    gboolean crosses_endstop = FALSE;
    rot_plan_result_t *chosen = NULL;
    GArray *normal_samples = NULL;
    GArray *flip_samples = NULL;
    gdouble t0, t1;
    gdouble az_min, az_max;
    rot_plan_input_t in = { 0 };

    rot_plan_reset(&ctrl->trajectory_plan);

    if (ctrl == NULL || ctrl->conf == NULL || ctrl->pass == NULL ||
        ctrl->target == NULL || ctrl->qth == NULL) {
        ctrl->trajectory_plan.reason = g_strdup("Missing rotor configuration, pass or target data");
        return FALSE;
    }

    t0 = ctrl->pass->aos;
    t1 = ctrl->pass->los;
    if (ctrl->t > t0)
        t0 = ctrl->t;
    if (t1 <= t0) {
        ctrl->trajectory_plan.reason = g_strdup("Invalid pass window for trajectory planning");
        return FALSE;
    }

    rot_get_abs_az_limits(ctrl->conf, &az_min, &az_max);

    gdouble cur_az = gtk_rot_knob_get_value(GTK_ROT_KNOB(ctrl->AzSet));
    gdouble cur_el = gtk_rot_knob_get_value(GTK_ROT_KNOB(ctrl->ElSet));
    cur_az = normalize_az_0_360(cur_az);

    in.az_min = az_min;
    in.az_max = az_max;
    in.el_min = ctrl->conf->minel;
    in.el_max = ctrl->conf->maxel;
    in.az_stop = ctrl->conf->azstoppos;
    in.use_az_stop = TRUE;
    in.az_current = cur_az;
    in.el_current = cur_el;
    in.have_current = TRUE;

    normal_samples = rot_build_samples(ctrl, ctrl->pass,
                                       ROT_PLAN_MODE_NORMAL, t0, t1);
    crosses_endstop = rotctrl_samples_cross_endstop(normal_samples,
                                                    ctrl->conf,
                                                    ctrl->conf->minel);
    sat_log_log(SAT_LOG_LEVEL_INFO,
                "trajectory crosses_endstop=%d",
                crosses_endstop ? 1 : 0);
    rot_term_log(ctrl, "gpredict",
                 "trajectory crosses_endstop=%d",
                 crosses_endstop ? 1 : 0);
    in.samples = normal_samples;
    have_normal = rot_plan_build(&in, &normal);

    flip_allowed = (ctrl->conf->maxel >= 180.0);
    if (flip_allowed && crosses_endstop) {
        flip_samples = rot_build_samples(ctrl, ctrl->pass,
                                         ROT_PLAN_MODE_FLIP, t0, t1);
        in.samples = flip_samples;
        have_flip = rot_plan_build(&in, &flip);
    }

    if (crosses_endstop && have_flip)
        chosen = &flip;
    else if (have_normal)
        chosen = &normal;
    else if (have_flip)
        chosen = &flip;

    if (chosen != NULL) {
        ctrl->trajectory_plan.valid = TRUE;
        ctrl->trajectory_plan.mode =
            (chosen == &flip) ? ROT_PLAN_MODE_FLIP : ROT_PLAN_MODE_NORMAL;
        ctrl->trajectory_plan.crosses_endstop = crosses_endstop;
        ctrl->trajectory_plan.status = chosen->status;
        ctrl->trajectory_plan.strategy = chosen->strategy;
        ctrl->trajectory_plan.trackable_pct = chosen->trackable_pct;
        ctrl->trajectory_plan.violation_mag = chosen->violation_mag;
        ctrl->trajectory_plan.total_motion = chosen->total_motion;
        ctrl->trajectory_plan.window_start = ctrl->pass->aos;
        ctrl->trajectory_plan.window_end = ctrl->pass->los;
        ctrl->trajectory_plan.sample_dt_sec = ROT_PLAN_SAMPLE_DT_SEC;
        ctrl->trajectory_plan.cmd_samples = chosen->cmds;
        chosen->cmds = NULL;

        if (ctrl->trajectory_plan.cmd_samples &&
            ctrl->trajectory_plan.cmd_samples->len > 0) {
            rot_plan_cmd_t first =
                g_array_index(ctrl->trajectory_plan.cmd_samples,
                              rot_plan_cmd_t, 0);
            rot_plan_cmd_t last =
                g_array_index(ctrl->trajectory_plan.cmd_samples,
                              rot_plan_cmd_t,
                              ctrl->trajectory_plan.cmd_samples->len - 1);
            ctrl->trajectory_plan.start_az_cmd = first.cmd_az;
            ctrl->trajectory_plan.start_el_cmd = first.cmd_el;
            ctrl->trajectory_plan.end_az_cmd = last.cmd_az;
            ctrl->trajectory_plan.end_el_cmd = last.cmd_el;
        }

        ctrl->trajectory_plan.reason = chosen->reason;
        chosen->reason = NULL;
        rotctrl_plan_log_mark(ctrl);

        if (ctrl->trajectory_plan.status == ROT_PLAN_STATUS_FULL_TRACK) {
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        "%s: selected %s plan strategy=%d (full track)",
                        __func__,
                        ctrl->trajectory_plan.mode == ROT_PLAN_MODE_FLIP ? "FLIP" : "NORMAL",
                        ctrl->trajectory_plan.strategy);
        } else {
            sat_log_log(SAT_LOG_LEVEL_WARN,
                        "%s: selected %s plan strategy=%d (%s)",
                        __func__,
                        ctrl->trajectory_plan.mode == ROT_PLAN_MODE_FLIP ? "FLIP" : "NORMAL",
                        ctrl->trajectory_plan.strategy,
                        ctrl->trajectory_plan.reason ? ctrl->trajectory_plan.reason : "partial tracking");
        }
    } else {
        ctrl->trajectory_plan.valid = FALSE;
        ctrl->trajectory_plan.reason = g_strdup("Trajectory planning failed");
    }

    if (have_normal)
        rot_plan_result_clear(&normal);
    if (have_flip)
        rot_plan_result_clear(&flip);

    if (normal_samples)
        g_array_free(normal_samples, TRUE);
    if (flip_samples)
        g_array_free(flip_samples, TRUE);

    return ctrl->trajectory_plan.valid;
}

/**
 * Read rotator position from device.
 *
 * \param ctrl Pointer to the GtkRotCtrl widget.
 * \param az The current Az as read from the device
 * \param el The current El as read from the device
 * \return TRUE if the position was successfully retrieved, FALSE if an
 *         error occurred.
 */
static gboolean G_GNUC_UNUSED get_pos(GtkRotCtrl * ctrl,
                                      gdouble * az, gdouble * el)
{
    if ((az == NULL) || (el == NULL))
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%d: NULL storage."), __FILE__, __LINE__);
        return FALSE;
    }

    /* Send-only mode: reuse last commanded values instead of asking rotctld. */
    g_mutex_lock(&ctrl->client.mutex);
    *az = ctrl->client.azi_out;
    *el = ctrl->client.ele_out;
    g_mutex_unlock(&ctrl->client.mutex);

    return TRUE;
}

static void rot_format_deg_2(char *out, gsize outsz, gdouble val)
{
    gchar buf[G_ASCII_DTOSTR_BUF_SIZE];

    g_ascii_formatd(buf, sizeof(buf), "%.2f", val);
    g_strlcpy(out, buf, outsz);
}

static gdouble norm360(gdouble a)
{
    return rot_norm360(a);
}

static gdouble ang_delta_deg(gdouble a, gdouble b)
{
    return rot_ang_diff_deg(a, b);
}

static AzSpan rotctrl_span_from_az_type(rot_az_type_t aztype)
{
    switch (aztype) {
    case ROT_AZ_TYPE_180:
        return AZSPAN_PM180;
    case ROT_AZ_TYPE_480:
        return AZSPAN_480;
    case ROT_AZ_TYPE_360:
    default:
        return AZSPAN_360;
    }
}

static AzSpan rotctrl_span_from_conf(const rotor_conf_t *conf)
{
    if (conf == NULL)
        return AZSPAN_360;

    return (conf->aztype == ROT_AZ_TYPE_180) ? AZSPAN_PM180 : AZSPAN_360;
}

static void rotctrl_span_bounds(AzSpan span, gdouble *min_out, gdouble *max_out)
{
    gdouble minv = 0.0;
    gdouble maxv = 360.0;

    switch (span) {
    case AZSPAN_PM180:
        minv = -180.0;
        maxv = 180.0;
        break;
    case AZSPAN_480:
        minv = 0.0;
        maxv = 480.0;
        break;
    case AZSPAN_360:
    default:
        minv = 0.0;
        maxv = 360.0;
        break;
    }

    if (min_out)
        *min_out = minv;
    if (max_out)
        *max_out = maxv;
}

static AzSpan rotctrl_span_mode_from_span_cfg(const SpanConfig *cfg)
{
    if (cfg == NULL)
        return AZSPAN_360;

    switch (cfg->az_mode) {
    case AZ_MODE_NEG180_POS180:
        return AZSPAN_PM180;
    case AZ_MODE_EXTENDED:
    case AZ_MODE_0_360:
    default:
        return AZSPAN_360;
    }
}

static const gchar *rotctrl_wrap_mode_name(rot_target_wrap_mode_t mode)
{
    return (mode == ROT_TARGET_WRAP_180) ? "-180..180" : "0..360";
}

static gdouble rotctrl_wrap_user_az(const rotor_conf_t *conf, gdouble az)
{
    if (conf != NULL && conf->aztype == ROT_AZ_TYPE_180)
        return azel_normalize_az_neg180_pos180(az);
    return normalize_az_0_360(az);
}

static gdouble G_GNUC_UNUSED az_wrap_diff(gdouble a, gdouble b, rot_az_type_t type)
{
    gdouble span = 360.0;
    gdouble da = a;
    gdouble db = b;

    switch (type)
    {
    case ROT_AZ_TYPE_180:
        da = azel_normalize_az_neg180_pos180(a);
        db = azel_normalize_az_neg180_pos180(b);
        span = 360.0;
        break;
    case ROT_AZ_TYPE_480:
        span = 480.0;
        da = fmod(a, span);
        if (da < 0.0)
            da += span;
        db = fmod(b, span);
        if (db < 0.0)
            db += span;
        break;
    case ROT_AZ_TYPE_360:
    default:
        da = normalize_az_0_360(a);
        db = normalize_az_0_360(b);
        span = 360.0;
        break;
    }

    gdouble delta = da - db;
    gdouble half = span / 2.0;
    if (delta > half)
        delta -= span;
    else if (delta < -half)
        delta += span;
    return delta;
}

static void rotctrl_wrap_span_bounds(const rotor_conf_t *conf,
                                     gdouble *min_out,
                                     gdouble *max_out)
{
    gdouble minv = 0.0;
    gdouble maxv = 360.0;

    if (conf != NULL && conf->aztype == ROT_AZ_TYPE_180)
    {
        minv = -180.0;
        maxv = 180.0;
    }

    if (min_out)
        *min_out = minv;
    if (max_out)
        *max_out = maxv;
}

static gdouble rotctrl_round_nearest(gdouble v)
{
    return (v >= 0.0) ? floor(v + 0.5) : ceil(v - 0.5);
}

static gdouble rotctrl_unwrap_nearest(gdouble prev_cont,
                                      gdouble az_canon,
                                      gdouble span)
{
    if (span <= 0.0)
        return az_canon;

    gdouble k = rotctrl_round_nearest((prev_cont - az_canon) / span);
    return az_canon + (k * span);
}

static gboolean rotctrl_segment_crosses_endstop(gdouble prev_cont,
                                                gdouble next_cont,
                                                gdouble endstop,
                                                gdouble span)
{
    if (span <= 0.0)
        return FALSE;
    if (fabs(next_cont - prev_cont) < 1e-9)
        return FALSE;

    gdouble lo = MIN(prev_cont, next_cont);
    gdouble hi = MAX(prev_cont, next_cont);
    gdouble stop = endstop;

    while (stop < lo - span)
        stop += span;
    while (stop > hi + span)
        stop -= span;

    for (gdouble s = stop; s <= hi; s += span)
    {
        if (s > lo && s < hi)
            return TRUE;
    }

    for (gdouble s = stop; s >= lo; s -= span)
    {
        if (s > lo && s < hi)
            return TRUE;
    }

    return FALSE;
}

static gboolean rotctrl_samples_cross_endstop(const GArray *samples,
                                              const rotor_conf_t *conf,
                                              gdouble min_el)
{
    gdouble wrap_min = 0.0;
    gdouble wrap_max = 360.0;
    gdouble span = 360.0;
    gdouble endstop = 0.0;
    gdouble prev_cont = 0.0;
    gboolean have_prev = FALSE;

    if (samples == NULL || conf == NULL || samples->len == 0)
        return FALSE;

    rotctrl_wrap_span_bounds(conf, &wrap_min, &wrap_max);
    span = wrap_max - wrap_min;
    if (span <= 0.0)
        span = 360.0;

    endstop = rotctrl_wrap_user_az(conf, conf->azstoppos);

    for (guint i = 0; i < samples->len; i++)
    {
        rot_plan_sample_t sample =
            g_array_index(samples, rot_plan_sample_t, i);
        gdouble el = sample.el;
        gdouble az = rotctrl_wrap_user_az(conf, sample.az);

        if (el < (min_el - ROT_BELOW_HORIZON_MARGIN_DEG))
            continue;

        if (!have_prev)
        {
            prev_cont = az;
            have_prev = TRUE;
            continue;
        }

        gdouble az_cont = rotctrl_unwrap_nearest(prev_cont, az, span);
        gdouble delta = fabs(az_cont - prev_cont);

        if (delta > (span / 2.0 + 1e-6) ||
            rotctrl_segment_crosses_endstop(prev_cont, az_cont,
                                            endstop, span))
            return TRUE;

        prev_cont = az_cont;
    }

    return FALSE;
}

static void G_GNUC_UNUSED rotctrl_apply_inverted_el(const rotor_conf_t *conf,
                                      gdouble az_user,
                                      gdouble el_user,
                                      gdouble *az_out,
                                      gdouble *el_out)
{
    gdouble az = az_user + 180.0;
    gdouble el = 180.0 - el_user;

    az = rotctrl_wrap_user_az(conf, az);

    if (az_out)
        *az_out = az;
    if (el_out)
        *el_out = el;
}

static const gchar *rotctrl_tracking_mode_name(rot_plan_mode_t mode)
{
    return (mode == ROT_PLAN_MODE_FLIP) ? "INVERTED_EL" : "NORMAL";
}

static gboolean rotctrl_find_entry_time(const rot_plan_t *plan,
                                        gdouble t_start,
                                        gdouble min_el,
                                        gdouble *t_entry_out)
{
    if (plan == NULL || !plan->valid || plan->cmd_samples == NULL ||
        plan->cmd_samples->len == 0)
        return FALSE;

    for (guint i = 0; i < plan->cmd_samples->len; i++)
    {
        rot_plan_cmd_t cmd =
            g_array_index(plan->cmd_samples, rot_plan_cmd_t, i);

        if (cmd.t < t_start)
            continue;
        if (!cmd.trackable)
            continue;
        if (cmd.cmd_el < min_el)
            continue;

        if (t_entry_out)
            *t_entry_out = cmd.t;
        return TRUE;
    }

    return FALSE;
}

static gboolean rotctrl_find_pretrack_cmd(const GtkRotCtrl *ctrl,
                                          gdouble t_start,
                                          gdouble min_el,
                                          gdouble *az_out,
                                          gdouble *el_out,
                                          gdouble *t_out)
{
    if (ctrl == NULL)
        return FALSE;

    if (!ctrl->trajectory_plan.valid ||
        ctrl->trajectory_plan.cmd_samples == NULL ||
        ctrl->trajectory_plan.cmd_samples->len == 0)
        return FALSE;

    for (guint i = 0; i < ctrl->trajectory_plan.cmd_samples->len; i++)
    {
        rot_plan_cmd_t cmd =
            g_array_index(ctrl->trajectory_plan.cmd_samples, rot_plan_cmd_t, i);

        if (cmd.t < t_start)
            continue;
        if (!cmd.trackable)
            continue;
        if (cmd.cmd_el < min_el)
            continue;

        if (az_out)
            *az_out = cmd.cmd_az;
        if (el_out)
            *el_out = cmd.cmd_el;
        if (t_out)
            *t_out = cmd.t;
        return TRUE;
    }

    return FALSE;
}

static void rotctrl_update_user_limits(GtkRotCtrl *ctrl)
{
    RotLimitSet limits;

    memset(&limits, 0, sizeof(limits));
    limits.wrap_mode = ROT_TARGET_WRAP_360;

    if (ctrl != NULL && ctrl->conf != NULL)
    {
        limits.valid = TRUE;
        limits.az_min = ctrl->conf->minaz;
        limits.az_max = ctrl->conf->maxaz;
        limits.el_min = ctrl->conf->minel;
        limits.el_max = ctrl->conf->maxel;
        limits.wrap_mode = (ctrl->conf->aztype == ROT_AZ_TYPE_180)
                           ? ROT_TARGET_WRAP_180
                           : ROT_TARGET_WRAP_360;
    }

    if (ctrl != NULL)
        ctrl->user_limits = limits;
}

static void rotctrl_update_backend_limits(GtkRotCtrl *ctrl,
                                          gboolean valid,
                                          gdouble az_min,
                                          gdouble az_max,
                                          gdouble el_min,
                                          gdouble el_max,
                                          gboolean south_zero)
{
    RotLimitSet limits;

    if (ctrl == NULL)
        return;

    memset(&limits, 0, sizeof(limits));
    limits.wrap_mode = ROT_TARGET_WRAP_360;
    limits.valid = valid;
    if (valid)
    {
        limits.az_min = az_min;
        limits.az_max = az_max;
        limits.el_min = el_min;
        limits.el_max = el_max;
        limits.south_zero = south_zero;
    }

    ctrl->backend_limits = limits;
}

static gboolean rotctrl_wrap_mismatch(const RotLimitSet *user,
                                      const RotLimitSet *backend)
{
    const gdouble eps = 1e-6;

    if (user == NULL || backend == NULL)
        return FALSE;
    if (!user->valid || !backend->valid)
        return FALSE;

    if (user->wrap_mode == ROT_TARGET_WRAP_180)
    {
        return (backend->az_min < -180.0 - eps ||
                backend->az_max > 180.0 + eps);
    }

    return (backend->az_min < 0.0 - eps ||
            backend->az_max > 360.0 + eps);
}

static void rotctrl_note_wrap_mismatch(GtkRotCtrl *ctrl)
{
    if (ctrl == NULL || ctrl->wrap_mismatch_logged)
        return;

    if (!rotctrl_wrap_mismatch(&ctrl->user_limits, &ctrl->backend_limits))
        return;

    sat_log_log(SAT_LOG_LEVEL_DEBUG, "wrap mismatch, normalizing");
    rot_term_log_verbose(ctrl, "gpredict:rx", "wrap mismatch, normalizing");
    if (ctrl->backend_limits.valid)
    {
        /* Normalize once when user wrap differs from backend to keep az_abs consistent. */
        ctrl->az_abs_cur = rotctrl_normalize_backend_az(ctrl->az_abs_cur,
                                                        ctrl->backend_limits.az_min,
                                                        ctrl->backend_limits.az_max);
        ctrl->az_abs_last_cmd = rotctrl_normalize_backend_az(ctrl->az_abs_last_cmd,
                                                             ctrl->backend_limits.az_min,
                                                             ctrl->backend_limits.az_max);
    }
    ctrl->wrap_mismatch_logged = TRUE;
}

static void rotctrl_log_limits_on_engage(GtkRotCtrl *ctrl)
{
    RotLimitSet user;
    RotLimitSet backend;

    if (ctrl == NULL || ctrl->limits_logged)
        return;
    if (!ctrl->engaged && !ctrl->engage_pending)
        return;

    user = ctrl->user_limits;
    backend = ctrl->backend_limits;

    if (!user.valid)
        return;

    if (backend.valid)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "rot limits user az=(%.2f..%.2f) el=(%.2f..%.2f) wrap=%s "
                    "backend az=(%.2f..%.2f) el=(%.2f..%.2f) south_zero=%d",
                    user.az_min, user.az_max, user.el_min, user.el_max,
                    rotctrl_wrap_mode_name(user.wrap_mode),
                    backend.az_min, backend.az_max, backend.el_min, backend.el_max,
                    backend.south_zero ? 1 : 0);
        rot_term_log(ctrl, "gpredict:rx",
                     "rot limits user az=(%.2f..%.2f) el=(%.2f..%.2f) wrap=%s "
                     "backend az=(%.2f..%.2f) el=(%.2f..%.2f) south_zero=%d",
                     user.az_min, user.az_max, user.el_min, user.el_max,
                     rotctrl_wrap_mode_name(user.wrap_mode),
                     backend.az_min, backend.az_max, backend.el_min, backend.el_max,
                     backend.south_zero ? 1 : 0);
    }
    else
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "rot limits user az=(%.2f..%.2f) el=(%.2f..%.2f) wrap=%s "
                    "backend limits unavailable",
                    user.az_min, user.az_max, user.el_min, user.el_max,
                    rotctrl_wrap_mode_name(user.wrap_mode));
        rot_term_log(ctrl, "gpredict:rx",
                     "rot limits user az=(%.2f..%.2f) el=(%.2f..%.2f) wrap=%s "
                     "backend limits unavailable",
                     user.az_min, user.az_max, user.el_min, user.el_max,
                     rotctrl_wrap_mode_name(user.wrap_mode));
    }

    ctrl->limits_logged = TRUE;
}

static const gchar *rotctrl_span_mode_name(azel_az_mode_t mode)
{
    switch (mode) {
    case AZ_MODE_NEG180_POS180:
        return "NEG180_180";
    case AZ_MODE_EXTENDED:
        return "EXTENDED";
    case AZ_MODE_0_360:
    default:
        return "0_360";
    }
}

static SpanConfig rotctrl_span_from_limits(const rotor_conf_t *conf,
                                           gboolean caps_valid,
                                           gdouble caps_az_min,
                                           gdouble caps_az_max,
                                           gdouble caps_el_min,
                                           gdouble caps_el_max,
                                           gboolean *extended_out)
{
    SpanConfig cfg;
    gdouble az_min = 0.0;
    gdouble az_max = 360.0;
    gdouble el_min = ROTCTRL_DEFAULT_MIN_EL;
    gdouble el_max = ROTCTRL_DEFAULT_MAX_EL;
    const gdouble eps = 1e-6;
    gdouble span = 360.0;

    memset(&cfg, 0, sizeof(cfg));

    if (conf != NULL)
    {
        az_min = conf->minaz;
        az_max = conf->maxaz;
        el_min = conf->minel;
        el_max = conf->maxel;
    }

    if (caps_valid)
    {
        az_min = caps_az_min;
        az_max = caps_az_max;
        el_min = caps_el_min;
        el_max = caps_el_max;
    }

    if (el_min > el_max)
    {
        gdouble tmp = el_min;
        el_min = el_max;
        el_max = tmp;
    }

    if (conf != NULL)
    {
        if (conf->aztype == ROT_AZ_TYPE_180)
            cfg.az_mode = AZ_MODE_NEG180_POS180;
        else if (conf->aztype == ROT_AZ_TYPE_480)
            cfg.az_mode = AZ_MODE_EXTENDED;
        else
            cfg.az_mode = AZ_MODE_0_360;
    }
    else if (caps_valid)
    {
        span = az_max - az_min;
        if (span <= 0.0)
            span += 360.0;

        if (span > 360.0 + eps)
        {
            cfg.az_mode = AZ_MODE_EXTENDED;
        }
        else if (az_min < -eps && az_max <= 180.0 + eps)
        {
            cfg.az_mode = AZ_MODE_NEG180_POS180;
        }
        else if (az_min >= -eps && az_max <= 360.0 + eps)
        {
            cfg.az_mode = AZ_MODE_0_360;
        }
        else
        {
            cfg.az_mode = AZ_MODE_EXTENDED;
        }
    }
    else
    {
        cfg.az_mode = AZ_MODE_0_360;
    }

    if (!caps_valid && cfg.az_mode == AZ_MODE_0_360)
    {
        span = az_max - az_min;
        if (span <= 0.0)
            span += 360.0;
        if (span >= 359.0)
        {
            az_min = 0.0;
            az_max = 360.0;
        }
    }

    cfg.az_min = az_min;
    cfg.az_max = az_max;
    cfg.el_min = el_min;
    cfg.el_max = el_max;
    cfg.prefer_shortest_path =
        (cfg.az_mode == AZ_MODE_EXTENDED ||
         cfg.az_mode == AZ_MODE_NEG180_POS180);
    cfg.allow_wrap =
        (cfg.az_mode == AZ_MODE_EXTENDED ||
         cfg.az_mode == AZ_MODE_NEG180_POS180);
    cfg.treat_360_as_0 = (cfg.az_mode == AZ_MODE_0_360);

    if (extended_out)
        *extended_out = (cfg.az_mode == AZ_MODE_EXTENDED);

    return cfg;
}

static gboolean G_GNUC_UNUSED rotctrl_caps_match_conf(const rotor_conf_t *conf,
                                                      gdouble caps_az_min,
                                                      gdouble caps_az_max,
                                                      gdouble caps_el_min,
                                                      gdouble caps_el_max)
{
    const gdouble eps = 2.0;
    rot_az_type_t conf_type;
    azel_az_mode_t conf_mode = AZ_MODE_0_360;
    SpanConfig caps_cfg;

    if (conf == NULL)
        return FALSE;

    conf_type = conf->aztype;
    if (conf_type == ROT_AZ_TYPE_180)
        conf_mode = AZ_MODE_NEG180_POS180;
    else if (conf_type == ROT_AZ_TYPE_480)
        conf_mode = AZ_MODE_EXTENDED;

    caps_cfg = rotctrl_span_from_limits(NULL,
                                        TRUE,
                                        caps_az_min,
                                        caps_az_max,
                                        caps_el_min,
                                        caps_el_max,
                                        NULL);

    if (caps_cfg.az_mode != conf_mode)
        return FALSE;

    if (fabs(caps_az_min - conf->minaz) > eps ||
        fabs(caps_az_max - conf->maxaz) > eps)
        return FALSE;

    if (fabs(caps_el_min - conf->minel) > eps ||
        fabs(caps_el_max - conf->maxel) > eps)
        return FALSE;

    return TRUE;
}

static gdouble rotctrl_normalize_az_to_limits(gdouble az,
                                              gdouble min,
                                              gdouble max)
{
    const gdouble eps = 1e-6;
    gboolean wraps = (min > max);
    gdouble span = max - min;

    if (wraps)
        span += 360.0;
    if (fabs(span) < eps)
        return min;

    if (span > 360.0 + eps)
    {
        if (az < min)
            return min;
        if (az > max)
            return max;
        return az;
    }

    if (span >= 359.0 - eps)
    {
        while (az < min)
            az += 360.0;
        while (az > max)
            az -= 360.0;
        return az;
    }

    if (wraps)
    {
        gdouble az_norm = norm360(az);

        if (az_norm >= min || az_norm <= max)
            return az_norm;

        gdouble dmin = fabs(ang_delta_deg(az_norm, min));
        gdouble dmax = fabs(ang_delta_deg(az_norm, max));
        return (dmin <= dmax) ? min : max;
    }

    if (az < min)
        return min;
    if (az > max)
        return max;
    return az;
}

static gdouble rotctrl_clamp_user_az_interval(const GtkRotCtrl *ctrl,
                                              gdouble az)
{
    if (ctrl == NULL || ctrl->conf == NULL)
        return az;

    return rotctrl_normalize_az_to_limits(az,
                                          ctrl->conf->minaz,
                                          ctrl->conf->maxaz);
}

static void G_GNUC_UNUSED normalize_and_clamp_target(const SpanConfig *span,
                                                     gdouble az_deg,
                                                     gdouble el_deg,
                                                     gdouble current_az,
                                                     gboolean have_current,
                                                     AzElMapResult *out)
{
    if (out != NULL)
        memset(out, 0, sizeof(*out));

    if (span == NULL || out == NULL)
        return;

    (void)azel_map(span, az_deg, el_deg, current_az, have_current, out);
}

static gdouble rotctrl_map_az_to_backend_range(gdouble az_norm,
                                               gdouble az_min,
                                               gdouble az_max)
{
    gdouble minv = az_min;
    gdouble maxv = az_max;
    gdouble mid;
    gint k_min;
    gint k_max;
    gdouble best = az_norm;
    gdouble best_diff = G_MAXDOUBLE;

    if (maxv < minv)
        maxv += 360.0;

    mid = 0.5 * (minv + maxv);
    k_min = (gint)ceil((minv - az_norm) / 360.0 - 1e-9);
    k_max = (gint)floor((maxv - az_norm) / 360.0 + 1e-9);

    if (k_min > k_max)
        return az_norm;

    for (gint k = k_min; k <= k_max; k++)
    {
        gdouble cand = az_norm + (360.0 * k);
        gdouble diff = fabs(cand - mid);
        if (diff < best_diff)
        {
            best = cand;
            best_diff = diff;
        }
    }

    return best;
}

static GArray *rotctrl_build_policy_samples(GtkRotCtrl *ctrl,
                                            gdouble t0,
                                            gdouble t1,
                                            gdouble az_min,
                                            gdouble az_max)
{
    GArray *samples = NULL;
    gdouble total_sec = 0.0;
    gdouble dt_sec = ROT_PLAN_SAMPLE_DT_SEC;
    guint steps = 0;

    if (ctrl == NULL || ctrl->conf == NULL || ctrl->target == NULL ||
        ctrl->qth == NULL)
        return NULL;

    total_sec = (t1 - t0) * secday;
    if (total_sec <= 0.0)
        return NULL;

    steps = (guint)ceil(total_sec / ROT_PLAN_SAMPLE_DT_SEC);
    if (steps < 1)
        steps = 1;
    if (steps > (ROT_POLICY_MAX_SAMPLES - 1))
        steps = ROT_POLICY_MAX_SAMPLES - 1;
    dt_sec = total_sec / steps;

    samples = g_array_sized_new(FALSE, FALSE, sizeof(rot_target_sample_t),
                                steps + 1);

    for (guint i = 0; i <= steps; i++)
    {
        gdouble t = t0 + (i * dt_sec) / secday;
        sat_t sat = *ctrl->target;
        rot_target_sample_t sample = { 0 };

        if (t > t1)
            t = t1;

        predict_calc(&sat, ctrl->qth, t);
        sample.t = t;
        sample.az = rotctrl_map_az_to_backend_range(
            normalize_az_0_360(sat.az), az_min, az_max);
        sample.el = sat.el;
        g_array_append_val(samples, sample);
    }

    return samples;
}

static void rotctrl_tracking_policy_reset_reason(GtkRotCtrl *ctrl,
                                                 const gchar *reason)
{
    if (ctrl == NULL)
        return;

    rot_tracking_policy_reset(&ctrl->track_policy);

    if (reason == NULL)
        reason = "unknown";

    sat_log_log(SAT_LOG_LEVEL_INFO,
                "tracking policy reset reason=%s policy_domain=ABS_BACKEND",
                reason);
    rot_term_log_verbose(ctrl, "gpredict",
                         "tracking policy reset reason=%s policy_domain=ABS_BACKEND",
                         reason);
}

static void G_GNUC_UNUSED rotctrl_tracking_policy_choose(GtkRotCtrl *ctrl,
                                                         gdouble az_min,
                                                         gdouble az_max,
                                                         gdouble el_min,
                                                         gdouble el_max,
                                                         gboolean allow_flip)
{
    tracking_wrap_mode_t wrap_mode;
    GArray *samples = NULL;
    gdouble t0 = 0.0;
    gdouble t1 = 0.0;
    gboolean used_flip = FALSE;
    gboolean degraded = FALSE;
    const gdouble eps = 1e-3;

    if (ctrl == NULL || ctrl->conf == NULL || ctrl->pass == NULL ||
        ctrl->target == NULL || ctrl->qth == NULL)
        return;

    wrap_mode = (ctrl->conf->aztype == ROT_AZ_TYPE_180)
                ? TRACKING_WRAP_NORTH_CENTERED
                : TRACKING_WRAP_CONTINUOUS;

    if (ctrl->track_policy.chosen)
    {
        gboolean same_window =
            fabs(ctrl->track_policy.window_start - ctrl->pass->aos) < 1e-4 &&
            fabs(ctrl->track_policy.window_end - ctrl->pass->los) < 1e-4;
        gboolean same_limits =
            fabs(ctrl->track_policy.az_min - az_min) < eps &&
            fabs(ctrl->track_policy.az_max - az_max) < eps &&
            fabs(ctrl->track_policy.el_min - el_min) < eps &&
            fabs(ctrl->track_policy.el_max - el_max) < eps;
        gboolean same_wrap = (ctrl->track_policy.wrap_mode == wrap_mode);
        gboolean flip_ok = allow_flip ||
                           (ctrl->track_policy.geom_mode != TRACKING_GEOM_FLIPPED);

        if (!same_window)
            rotctrl_tracking_policy_reset_reason(ctrl, "new_pass");
        else if (same_limits && same_wrap && flip_ok)
            return;
    }

    t0 = ctrl->pass->aos;
    t1 = ctrl->pass->los;
    if (ctrl->t > t0)
        t0 = ctrl->t;
    if (t1 <= t0)
        return;

    samples = rotctrl_build_policy_samples(ctrl, t0, t1, az_min, az_max);
    if (samples == NULL || samples->len == 0)
    {
        if (samples)
            g_array_free(samples, TRUE);
        return;
    }

    rot_tracking_policy_choose(&ctrl->track_policy,
                               wrap_mode,
                               az_min,
                               az_max,
                               el_min,
                               el_max,
                               allow_flip,
                               (const rot_target_sample_t *)samples->data,
                               samples->len,
                               &used_flip,
                               &degraded);

    ctrl->track_policy.window_start = ctrl->pass->aos;
    ctrl->track_policy.window_end = ctrl->pass->los;

    rot_term_log_verbose(ctrl, "gpredict",
                         "tracking policy wrap=%s geom=%s k=%d flip_allowed=%d "
                         "policy_domain=ABS_BACKEND",
                         (wrap_mode == TRACKING_WRAP_NORTH_CENTERED)
                         ? "north-centered" : "continuous",
                         used_flip ? "flipped" : "normal",
                         ctrl->track_policy.preferred_k,
                         allow_flip ? 1 : 0);
    sat_log_log(SAT_LOG_LEVEL_INFO,
                "tracking policy wrap=%s geom=%s k=%d flip_allowed=%d "
                "policy_domain=ABS_BACKEND",
                (wrap_mode == TRACKING_WRAP_NORTH_CENTERED)
                ? "north-centered" : "continuous",
                used_flip ? "flipped" : "normal",
                ctrl->track_policy.preferred_k,
                allow_flip ? 1 : 0);

    if (degraded && !ctrl->track_policy.logged_no_branch)
    {
        rot_term_log_verbose(ctrl, "gpredict:warn",
                             "tracking policy no_feasible_branch");
        sat_log_log(SAT_LOG_LEVEL_WARN,
                    "tracking policy no_feasible_branch");
        ctrl->track_policy.logged_no_branch = TRUE;
    }

    g_array_free(samples, TRUE);
}

static gboolean G_GNUC_UNUSED rotctrl_tracking_policy_select(GtkRotCtrl *ctrl,
                                                             gdouble az_pred,
                                                             gdouble el_pred,
                                                             gboolean have_measured,
                                                             gdouble measured_az,
                                                             gdouble last_cmd_az,
                                                             gdouble *az_cmd_out,
                                                             gdouble *el_cmd_out,
                                                             gboolean *empty_set_out)
{
    if (ctrl == NULL)
        return FALSE;

    return rot_tracking_policy_select_cmd(&ctrl->track_policy,
                                          az_pred,
                                          el_pred,
                                          have_measured,
                                          measured_az,
                                          last_cmd_az,
                                          az_cmd_out,
                                          el_cmd_out,
                                          empty_set_out);
}

static gboolean G_GNUC_UNUSED rotctrl_keepalive_enabled(void)
{
    static gsize init = 0;
    static gboolean enabled = TRUE;

    if (g_once_init_enter(&init))
    {
        const gchar *env = g_getenv("GPREDICT_ROT_KEEPALIVE");

        if (env && *env &&
            (g_ascii_strcasecmp(env, "0") == 0 ||
             g_ascii_strcasecmp(env, "false") == 0 ||
             g_ascii_strcasecmp(env, "no") == 0 ||
             g_ascii_strcasecmp(env, "off") == 0))
        {
            enabled = FALSE;
        }

        g_once_init_leave(&init, 1);
    }

    return enabled;
}

static gboolean rotctrl_stop_crosses(const RotorSafety *s,
                                     double az_abs_cur,
                                     double az_abs_candidate)
{
    double band_min;
    double band_max;
    double lo;
    double hi;

    if (s == NULL || !s->stop_enabled || s->stop_margin_deg <= 0.0)
        return FALSE;

    band_min = s->stop_abs - s->stop_margin_deg;
    band_max = s->stop_abs + s->stop_margin_deg;
    lo = MIN(az_abs_cur, az_abs_candidate);
    hi = MAX(az_abs_cur, az_abs_candidate);

    return !(hi < band_min || lo > band_max);
}

static double rotctrl_az_abs_error(double measured_abs,
                                   double desired_backend,
                                   AzSpan span_mode,
                                   gboolean span_extended)
{
    double desired_abs = desired_backend;

    if (!span_extended)
        desired_abs = az_target_to_nearest_abs(measured_abs,
                                               desired_backend,
                                               span_mode);

    return fabs(desired_abs - measured_abs);
}

static gboolean rotctrl_get_pos_valid(GtkRotCtrl *ctrl, gint64 *last_pos_us)
{
    gboolean ok = FALSE;
    gint64 last = 0;

    if (ctrl == NULL)
        return FALSE;

    g_mutex_lock(&ctrl->client.mutex);
    ok = ctrl->client.pos_valid;
    last = ctrl->client.last_pos_us;
    g_mutex_unlock(&ctrl->client.mutex);

    if (last_pos_us)
        *last_pos_us = last;

    return ok;
}

static gboolean rotctrl_last_known_user_position(GtkRotCtrl *ctrl,
                                                 gdouble *az_out,
                                                 gdouble *el_out,
                                                 gint64 *sample_us_out)
{
    gboolean have = FALSE;
    gdouble az = 0.0;
    gdouble el = 0.0;
    gint64 sample_us = 0;

    if (ctrl == NULL)
        return FALSE;

    g_mutex_lock(&ctrl->client.mutex);
    if (ctrl->client.last_pos_sample_us > 0)
    {
        az = ctrl->client.last_pos_user_az;
        el = ctrl->client.last_pos_user_el;
        sample_us = ctrl->client.last_pos_sample_us;
        have = TRUE;
    }
    else if (ctrl->client.last_pos_us > 0)
    {
        az = ctrl->client.azi_in;
        el = ctrl->client.ele_in;
        sample_us = ctrl->client.last_pos_us;
        have = TRUE;
    }
    g_mutex_unlock(&ctrl->client.mutex);

    if (have)
    {
        if (az_out)
            *az_out = az;
        if (el_out)
            *el_out = el;
        if (sample_us_out)
            *sample_us_out = sample_us;
    }
    else if (sample_us_out)
    {
        *sample_us_out = 0;
    }

    return have;
}

static gboolean rotctrl_refresh_user_position(GtkRotCtrl *ctrl,
                                              gint timeout_ms,
                                              gdouble *az_out,
                                              gdouble *el_out,
                                              gboolean *fresh_out)
{
    gint64 baseline_sample_us = 0;
    gint64 deadline_us = 0;
    gdouble az = 0.0;
    gdouble el = 0.0;
    gint64 sample_us = 0;
    gboolean have = FALSE;
    gboolean fresh = FALSE;

    if (fresh_out)
        *fresh_out = FALSE;

    if (ctrl == NULL)
        return FALSE;

    have = rotctrl_last_known_user_position(ctrl,
                                            &az,
                                            &el,
                                            &baseline_sample_us);

    if (!ctrl->client.running || !ctrl->engaged)
    {
        if (have)
        {
            if (az_out)
                *az_out = az;
            if (el_out)
                *el_out = el;
        }
        return have;
    }

    g_mutex_lock(&ctrl->client.mutex);
    ctrl->client.pos_unknown = TRUE;
    ctrl->client.last_pos_attempt_us = 0;
    ctrl->client.pos_backoff_until_us = 0;
    ctrl->client.pos_backoff_sec = 0.5;
    g_mutex_unlock(&ctrl->client.mutex);

    if (timeout_ms < ROT_FREEZE_MIN_WAIT_MS)
        timeout_ms = ROT_FREEZE_MIN_WAIT_MS;
    if (timeout_ms > ROT_FREEZE_MAX_WAIT_MS)
        timeout_ms = ROT_FREEZE_MAX_WAIT_MS;

    deadline_us = g_get_monotonic_time() + ((gint64)timeout_ms * 1000);
    while (g_get_monotonic_time() < deadline_us)
    {
        have = rotctrl_last_known_user_position(ctrl, &az, &el, &sample_us);
        if (have && sample_us > baseline_sample_us)
        {
            fresh = TRUE;
            break;
        }
        g_usleep(ROT_FREEZE_POLL_SLICE_US);
    }

    if (!have)
        have = rotctrl_last_known_user_position(ctrl, &az, &el, &sample_us);

    if (have)
    {
        if (az_out)
            *az_out = az;
        if (el_out)
            *el_out = el;
    }

    if (fresh_out)
        *fresh_out = fresh;

    return have;
}

static gboolean rotctrl_pos_recent(GtkRotCtrl *ctrl,
                                   gint64 window_us,
                                   gint64 *last_pos_us)
{
    gint64 last = 0;
    gint64 now_us = g_get_monotonic_time();

    if (ctrl == NULL)
        return FALSE;

    g_mutex_lock(&ctrl->client.mutex);
    last = ctrl->client.last_pos_us;
    g_mutex_unlock(&ctrl->client.mutex);

    if (last_pos_us)
        *last_pos_us = last;

    if (last <= 0)
        return FALSE;

    return (now_us - last) <= window_us;
}

static gboolean rotctrl_session_ready(GtkRotCtrl *ctrl,
                                      gboolean pos_recent)
{
    if (ctrl == NULL)
        return FALSE;

    return ctrl->engaged &&
           (ctrl->session_state == ROT_SESSION_READY) &&
           pos_recent;
}

static gboolean rotctrl_manual_override_active(GtkRotCtrl *ctrl)
{
    if (ctrl == NULL)
        return FALSE;

    if (ctrl->cal_active)
        return TRUE;

    if (ctrl->tracking)
        return FALSE;

    return g_get_monotonic_time() < ctrl->manual_edit_until_us;
}

static gboolean G_GNUC_UNUSED rotctrl_wait_for_baseline(GtkRotCtrl *ctrl)
{
    guint delay_ms = 100;

    for (gint attempt = 0; attempt < 3; attempt++)
    {
        if (rotctrl_get_pos_valid(ctrl, NULL))
            return TRUE;
        g_usleep((gulong)delay_ms * 1000);
        delay_ms = MIN(delay_ms * 2, 500u);
    }

    return rotctrl_get_pos_valid(ctrl, NULL);
}

static void rotctrl_update_safety(GtkRotCtrl *ctrl,
                                  gboolean caps_valid,
                                  gdouble caps_az_min,
                                  gdouble caps_az_max)
{
    double span_width;

    if (ctrl == NULL)
        return;

    span_width = az_span_width(ctrl->span_mode);

    ctrl->safety.enabled = TRUE;
    if (caps_valid)
    {
        ctrl->safety.win_min_abs = caps_az_min;
        ctrl->safety.win_max_abs = caps_az_max;
    }
    else if (ctrl->conf)
    {
        ctrl->safety.win_min_abs = ctrl->conf->minaz;
        ctrl->safety.win_max_abs = ctrl->conf->maxaz;
    }
    else
    {
        ctrl->safety.win_min_abs = 0.0;
        ctrl->safety.win_max_abs = span_width;
    }

    if (ctrl->safety.win_max_abs - ctrl->safety.win_min_abs <= 0.0)
        ctrl->safety.win_max_abs = ctrl->safety.win_min_abs + span_width;

    ctrl->safety.stop_enabled = FALSE;
    ctrl->safety.stop_abs = 0.0;
    ctrl->safety.stop_margin_deg = 0.0;

    if (ctrl->conf)
    {
        double span = ctrl->safety.win_max_abs - ctrl->safety.win_min_abs;
        if (span > 0.0 && span < (span_width - 1e-3))
        {
            ctrl->safety.stop_enabled = TRUE;
            ctrl->safety.stop_abs = ctrl->conf->azstoppos;
            ctrl->safety.stop_margin_deg = ROT_PLAN_NEAR_LIMIT_MARGIN;
        }
    }
}

static gboolean rot_az_in_limits(gdouble az, gdouble min, gdouble max)
{
    if (min <= max)
        return (az >= min && az <= max);
    return (az >= min || az <= max);
}

static gdouble rotctrl_wrap_distance(const RotLimitSet *limits,
                                     gdouble a,
                                     gdouble b)
{
    if (limits && limits->wrap_mode == ROT_TARGET_WRAP_180)
        return rot_target_az_distance_wrap180(a, b);
    return rot_target_az_distance_wrap360(a, b);
}

static gboolean rotctrl_resolve_wrap_candidate(const RotLimitSet *user_limits,
                                               const RotLimitSet *backend_limits,
                                               gdouble desired_user_az,
                                               gdouble desired_user_el,
                                               gdouble desired_raw_az,
                                               gdouble ref_user_az,
                                               gboolean have_ref,
                                               rot_ui_mode_t ui_mode,
                                               gdouble ref_backend_az,
                                               gboolean have_ref_backend,
                                               gdouble *out_user_az,
                                               gdouble *out_raw_az,
                                               gint *out_k,
                                               gdouble *out_cost)
{
    static const gint k_list[] = { 0, 1, -1, 2, -2 };
    gboolean found = FALSE;
    gdouble best_user = desired_user_az;
    gdouble best_raw = desired_raw_az;
    gdouble best_cost = 0.0;
    gint best_k = 0;

    if (user_limits && user_limits->valid)
    {
        if (desired_user_el < user_limits->el_min ||
            desired_user_el > user_limits->el_max)
        {
            return FALSE;
        }
    }

    for (guint i = 0; i < G_N_ELEMENTS(k_list); i++)
    {
        gint k = k_list[i];
        gdouble cand_raw = desired_raw_az + (360.0 * k);
        gdouble cand_user = rot_az360_to_ui(cand_raw, ui_mode);
        gdouble cost = 0.0;

        if (user_limits && user_limits->valid)
        {
            if (!rot_az_in_limits(cand_user, user_limits->az_min, user_limits->az_max))
                continue;
        }
        if (backend_limits && backend_limits->valid)
        {
            if (!rot_az_in_limits(cand_raw, backend_limits->az_min, backend_limits->az_max))
                continue;
        }

        if (have_ref_backend)
            cost = fabs(cand_raw - ref_backend_az);
        else if (have_ref)
            cost = rotctrl_wrap_distance(user_limits, cand_user, ref_user_az);
        else
            cost = fabs((gdouble)k);

        if (!found || cost < (best_cost - 1e-6))
        {
            best_user = cand_user;
            best_raw = cand_raw;
            best_cost = cost;
            best_k = k;
            found = TRUE;
        }
    }

    if (!found)
        return FALSE;

    if (out_user_az)
        *out_user_az = best_user;
    if (out_raw_az)
        *out_raw_az = best_raw;
    if (out_k)
        *out_k = best_k;
    if (out_cost)
        *out_cost = best_cost;

    return TRUE;
}

static gdouble rot_clamp_az_abs(gdouble az, gdouble min, gdouble max,
                                gboolean *clamped_out)
{
    gdouble span = max - min;

    if (span > 360.0 + 1e-6)
    {
        if (az < min)
        {
            if (clamped_out)
                *clamped_out = TRUE;
            return min;
        }
        if (az > max)
        {
            if (clamped_out)
                *clamped_out = TRUE;
            return max;
        }
        if (clamped_out)
            *clamped_out = FALSE;
        return az;
    }

    gdouble az_norm = norm360(az);

    if (rot_az_in_limits(az_norm, min, max))
    {
        if (clamped_out)
            *clamped_out = FALSE;
        return az_norm;
    }

    gdouble dmin = fabs(ang_delta_deg(az_norm, min));
    gdouble dmax = fabs(ang_delta_deg(az_norm, max));
    if (clamped_out)
        *clamped_out = TRUE;
    return (dmin <= dmax) ? min : max;
}

static gdouble G_GNUC_UNUSED rotctrl_normalize_az_for_backend(GtkRotCtrl *ctrl,
                                                gdouble target_az_user,
                                                gdouble current_az_backend,
                                                gboolean have_current,
                                                gdouble backend_min_az,
                                                gdouble backend_max_az,
                                                gboolean *clamped_out)
{
    gdouble best = target_az_user;
    gdouble best_diff = G_MAXDOUBLE;
    gboolean found = FALSE;

    if (clamped_out)
        *clamped_out = FALSE;

    for (gint k = -2; k <= 2; k++)
    {
        gdouble cand = target_az_user + (360.0 * k);
        gdouble diff = 0.0;

        if (!rot_az_in_limits(cand, backend_min_az, backend_max_az))
            continue;

        diff = have_current ? fabs(shortest_az_delta(cand, current_az_backend))
                            : fabs(shortest_az_delta(cand, target_az_user));

        if (!found || diff < best_diff)
        {
            best = cand;
            best_diff = diff;
            found = TRUE;
        }
    }

    if (found)
        return rotctrl_normalize_backend_az(best,
                                            backend_min_az,
                                            backend_max_az);

    if (clamped_out)
        *clamped_out = TRUE;

    gdouble clamped = rot_clamp_az_abs(target_az_user,
                                       backend_min_az,
                                       backend_max_az,
                                       NULL);

    if (ctrl != NULL)
    {
        rot_log_rate_limited(ctrl, &ctrl->backend_clamp_rate, 2000000,
                             SAT_LOG_LEVEL_WARN, "gpredict:warn",
                             "backend az clamp: target=%.2f range=(%.2f..%.2f) -> %.2f",
                             target_az_user, backend_min_az, backend_max_az, clamped);
    }

    return rotctrl_normalize_backend_az(clamped,
                                        backend_min_az,
                                        backend_max_az);
}

static gdouble rotctrl_normalize_backend_az(gdouble az,
                                            gdouble backend_min_az,
                                            gdouble backend_max_az)
{
    const gdouble eps = 1e-6;

    if (backend_min_az >= -eps &&
        backend_max_az <= (360.0 + eps))
        return gp_norm360(az);

    return az;
}

static gboolean rotctrl_wrap_debug_enabled(void)
{
    const gchar *val = g_getenv("GP_DEBUG_ROTOR_WRAP");

    if (!val || !*val)
        return FALSE;

    return (val[0] == '1' || val[0] == 'y' || val[0] == 'Y');
}

typedef struct {
    gboolean found;
    gdouble cand;
    gdouble base_diff;
    gdouble endstop_dist;
    gint k;
    gboolean safe;
} RotLanePick;

static void rotctrl_prefer_user_backend_span(const GtkRotCtrl *ctrl,
                                             gdouble *backend_min,
                                             gdouble *backend_max)
{
    const gdouble eps = 1e-6;
    gdouble user_min = 0.0;
    gdouble user_max = 0.0;

    if (ctrl == NULL || backend_min == NULL || backend_max == NULL)
        return;

    if (!ctrl->user_limits.valid ||
        ctrl->user_limits.wrap_mode != ROT_TARGET_WRAP_360)
        return;

    /* In manual mode we want shortest-path lane selection across 0/360 when
     * backend exposes extended az limits (e.g. -180..540).
     */
    if (!ctrl->tracking)
        return;

    user_min = ctrl->user_limits.az_min;
    user_max = ctrl->user_limits.az_max;

    if (user_min > user_max)
        return;

    /* If backend exposes an extended superset (e.g. -180..450), keep lane
     * selection inside the configured user span to avoid equivalent negative
     * commands for a 0..360 rotor.
     */
    if (*backend_min <= user_min + eps &&
        *backend_max >= user_max - eps &&
        (*backend_min < user_min - eps ||
         *backend_max > user_max + eps))
    {
        *backend_min = user_min;
        *backend_max = user_max;
    }
}

static gdouble rotctrl_choose_backend_lane(GtkRotCtrl *ctrl,
                                           gdouble az360,
                                           gdouble ref_backend,
                                           gdouble min_az,
                                           gdouble max_az,
                                           gboolean locked_k_valid,
                                           gint locked_k,
                                           gdouble endstop_margin_deg,
                                           gdouble lane_switch_penalty_deg,
                                           gint *out_k,
                                           gboolean *out_used_margin,
                                           gboolean *out_switched,
                                           gboolean *out_clamped)
{
    gdouble az_norm = gp_norm360(az360);
    gdouble best = az_norm;
    gboolean debug = rotctrl_wrap_debug_enabled();
    RotLanePick best_any_safe = { 0 };
    RotLanePick best_any_all = { 0 };
    RotLanePick best_locked_safe = { 0 };
    RotLanePick best_locked_all = { 0 };
    RotLanePick best_alt_safe = { 0 };
    RotLanePick best_alt_all = { 0 };
    gboolean any_safe = FALSE;
    gboolean switched = FALSE;
    const gchar *switch_reason = NULL;

    if (out_used_margin)
        *out_used_margin = FALSE;
    if (out_switched)
        *out_switched = FALSE;
    if (out_clamped)
        *out_clamped = FALSE;

    if (min_az > max_az)
    {
        gdouble tmp = min_az;
        min_az = max_az;
        max_az = tmp;
    }

    rotctrl_prefer_user_backend_span(ctrl, &min_az, &max_az);

    if (endstop_margin_deg < 0.0)
        endstop_margin_deg = 0.0;

    {
        gint k_min = (gint)ceil((min_az - az_norm) / 360.0 - 1e-9);
        gint k_max = (gint)floor((max_az - az_norm) / 360.0 + 1e-9);
        gint cand_count = 0;

        for (gint k = k_min; k <= k_max; k++)
        {
            gdouble cand = az_norm + (360.0 * k);
            gdouble base_diff = fabs(cand - ref_backend);
            gdouble endstop_dist = MIN(cand - min_az, max_az - cand);
            gboolean safe = (endstop_dist >= endstop_margin_deg);

            if (cand < min_az - 1e-9 || cand > max_az + 1e-9)
                continue;

            cand_count++;

            if (debug)
            {
                gdouble score = base_diff;
                if (locked_k_valid && k != locked_k)
                    score += lane_switch_penalty_deg;
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            "rot wrap cand k=%d az=%.2f diff=%.2f endstop=%.2f safe=%d score=%.2f",
                            k, cand, base_diff, endstop_dist, safe ? 1 : 0, score);
            }

            if (safe)
            {
                any_safe = TRUE;
                if (!best_any_safe.found || base_diff < best_any_safe.base_diff)
                    best_any_safe = (RotLanePick){ TRUE, cand, base_diff, endstop_dist, k, TRUE };
                if (locked_k_valid && k == locked_k)
                {
                    if (!best_locked_safe.found || base_diff < best_locked_safe.base_diff)
                        best_locked_safe = (RotLanePick){ TRUE, cand, base_diff, endstop_dist, k, TRUE };
                }
                else if (locked_k_valid)
                {
                    if (!best_alt_safe.found || base_diff < best_alt_safe.base_diff)
                        best_alt_safe = (RotLanePick){ TRUE, cand, base_diff, endstop_dist, k, TRUE };
                }
            }

            if (!best_any_all.found || base_diff < best_any_all.base_diff)
                best_any_all = (RotLanePick){ TRUE, cand, base_diff, endstop_dist, k, safe };
            if (locked_k_valid && k == locked_k)
            {
                if (!best_locked_all.found || base_diff < best_locked_all.base_diff)
                    best_locked_all = (RotLanePick){ TRUE, cand, base_diff, endstop_dist, k, safe };
            }
            else if (locked_k_valid)
            {
                if (!best_alt_all.found || base_diff < best_alt_all.base_diff)
                    best_alt_all = (RotLanePick){ TRUE, cand, base_diff, endstop_dist, k, safe };
            }
        }

        if (cand_count == 0)
        {
            if (fabs(ref_backend - min_az) <= fabs(ref_backend - max_az))
            {
                best = min_az;
                if (out_k)
                    *out_k = (gint)lrint((min_az - az_norm) / 360.0);
            }
            else
            {
                best = max_az;
                if (out_k)
                    *out_k = (gint)lrint((max_az - az_norm) / 360.0);
            }

            if (out_used_margin)
                *out_used_margin = TRUE;
            if (out_clamped)
                *out_clamped = TRUE;

            return best;
        }
    }

    if (!locked_k_valid)
    {
        RotLanePick pick = any_safe ? best_any_safe : best_any_all;
        best = pick.cand;
        if (out_k)
            *out_k = pick.k;
        if (out_used_margin)
            *out_used_margin = !any_safe;
        return best;
    }

    if (any_safe)
    {
        if (best_locked_safe.found)
        {
            if (best_alt_safe.found &&
                (best_locked_safe.base_diff - best_alt_safe.base_diff) >
                    lane_switch_penalty_deg)
            {
                best = best_alt_safe.cand;
                if (out_k)
                    *out_k = best_alt_safe.k;
                switched = (best_alt_safe.k != locked_k);
                switch_reason = "improve_safe";
            }
            else
            {
                best = best_locked_safe.cand;
                if (out_k)
                    *out_k = best_locked_safe.k;
            }
        }
        else
        {
            best = best_any_safe.cand;
            if (out_k)
                *out_k = best_any_safe.k;
            switched = (best_any_safe.k != locked_k);
            switch_reason = "locked_unsafe";
        }
    }
    else
    {
        if (best_locked_all.found)
        {
            if (best_alt_all.found &&
                (best_locked_all.base_diff - best_alt_all.base_diff) >
                    lane_switch_penalty_deg)
            {
                best = best_alt_all.cand;
                if (out_k)
                    *out_k = best_alt_all.k;
                switched = (best_alt_all.k != locked_k);
                switch_reason = "improve_unsafe";
            }
            else
            {
                best = best_locked_all.cand;
                if (out_k)
                    *out_k = best_locked_all.k;
            }
        }
        else
        {
            best = best_any_all.cand;
            if (out_k)
                *out_k = best_any_all.k;
            switched = (best_any_all.k != locked_k);
            switch_reason = "locked_missing";
        }
    }

    if (out_used_margin)
        *out_used_margin = !any_safe;
    if (switched && out_switched)
        *out_switched = TRUE;
    if (switched)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "rot lane switch: k=%d->%d reason=%s ref=%.2f",
                    locked_k,
                    out_k ? *out_k : locked_k,
                    switch_reason ? switch_reason : "unknown",
                    ref_backend);
    }

    return best;
}

static gboolean rotctrl_choose_seam_crossing_lane(const GtkRotCtrl *ctrl,
                                                  gdouble target_az360,
                                                  gdouble ref_backend,
                                                  gdouble backend_min,
                                                  gdouble backend_max,
                                                  gint *lane_k_out,
                                                  gdouble *cmd_az_out)
{
    gdouble wrap_min = 0.0;
    gdouble wrap_max = 360.0;
    gdouble span = 360.0;
    gdouble endstop = 0.0;
    gboolean found = FALSE;
    gdouble best_cmd = 0.0;
    gdouble best_dist = 0.0;
    gint best_k = 0;

    if (ctrl == NULL || ctrl->conf == NULL)
        return FALSE;

    rotctrl_prefer_user_backend_span(ctrl, &backend_min, &backend_max);

    rotctrl_wrap_span_bounds(ctrl->conf, &wrap_min, &wrap_max);
    span = wrap_max - wrap_min;
    if (span <= 0.0)
        span = 360.0;

    endstop = rotctrl_wrap_user_az(ctrl->conf, ctrl->conf->azstoppos);

    {
        gint k_min = (gint)floor((backend_min - target_az360) / 360.0) - 1;
        gint k_max = (gint)ceil((backend_max - target_az360) / 360.0) + 1;

        for (gint k = k_min; k <= k_max; k++)
        {
            gdouble cand = target_az360 + (360.0 * k);

            if (cand < backend_min - 1e-6 || cand > backend_max + 1e-6)
                continue;

            if (!rotctrl_segment_crosses_endstop(ref_backend, cand, endstop, span))
                continue;

            gdouble dist = fabs(cand - ref_backend);
            if (!found || dist < best_dist)
            {
                found = TRUE;
                best_cmd = cand;
                best_dist = dist;
                best_k = k;
            }
        }
    }

    if (!found)
        return FALSE;

    if (lane_k_out)
        *lane_k_out = best_k;
    if (cmd_az_out)
        *cmd_az_out = best_cmd;
    return TRUE;
}

static gboolean rotctrl_pick_wrap_candidate(const GtkRotCtrl *ctrl,
                                            gdouble target_az360,
                                            gdouble ref_backend,
                                            gdouble backend_min,
                                            gdouble backend_max,
                                            gdouble *cand_a_out,
                                            gdouble *cand_b_out,
                                            gdouble *chosen_out,
                                            gint *chosen_k_out)
{
    gdouble az_norm = gp_norm360(target_az360);
    gint k_min = 0;
    gint k_max = -1;
    gdouble best = 0.0;
    gdouble best_diff = 0.0;
    gint best_k = 0;
    gboolean found = FALSE;
    gdouble cand_a = NAN;
    gdouble cand_b = NAN;
    gdouble diff_a = 0.0;
    gdouble diff_b = 0.0;

    rotctrl_prefer_user_backend_span(ctrl, &backend_min, &backend_max);
    k_min = (gint)floor((backend_min - az_norm) / 360.0) - 1;
    k_max = (gint)ceil((backend_max - az_norm) / 360.0) + 1;

    for (gint k = k_min; k <= k_max; k++)
    {
        gdouble cand = az_norm + (360.0 * k);

        if (cand < backend_min - 1e-6 || cand > backend_max + 1e-6)
            continue;

        gdouble diff = fabs(cand - ref_backend);
        if (!found || diff < best_diff)
        {
            best = cand;
            best_diff = diff;
            best_k = k;
            found = TRUE;
        }

        if (isnan(cand_a) || diff < diff_a)
        {
            cand_b = cand_a;
            diff_b = diff_a;
            cand_a = cand;
            diff_a = diff;
        }
        else if (isnan(cand_b) || diff < diff_b)
        {
            cand_b = cand;
            diff_b = diff;
        }
    }

    if (cand_a_out)
        *cand_a_out = cand_a;
    if (cand_b_out)
        *cand_b_out = cand_b;

    if (!found)
        return FALSE;

    if (chosen_out)
        *chosen_out = best;
    if (chosen_k_out)
        *chosen_k_out = best_k;
    return TRUE;
}

typedef struct {
    gdouble az360;
    gdouble el;
    gdouble ui_az;
    gdouble ui_el;
    gdouble cmd_az;
    gdouble cmd_el;
    gboolean az_clamped;
    gboolean el_clamped;
    gboolean lane_crosses_seam;
    gboolean lane_used_margin;
    gboolean lane_switched;
    gint    lane_k;
    gdouble lane_ref_backend;
} RotCmdPipeline;

typedef struct {
    gdouble az_sat;
    gdouble el_sat;
    gdouble az_after_southzero;
    gdouble el_after_southzero;
    gdouble az_after_offsets;
    gdouble el_after_offsets;
    gdouble az360_final;
    gdouble el_final;
} RotTargetOut;

static gboolean rotctrl_transform_debug_enabled(void)
{
    const gchar *val = g_getenv("GP_DEBUG_ROTOR_TRANSFORM");

    if (!val || !*val)
        return FALSE;

    return (val[0] == '1' || val[0] == 'y' || val[0] == 'Y');
}

static gboolean gp_rot_transform_target(GtkRotCtrl *ctrl,
                                        gdouble in_az,
                                        gdouble in_el,
                                        gdouble elev_floor,
                                        RotTargetOut *out)
{
    gdouble az = gp_norm360(in_az);
    gdouble el = in_el;
    gboolean south_zero = FALSE;
    gboolean use_offset = FALSE;
    gboolean calib_raw = FALSE;

    if (ctrl == NULL || out == NULL)
        return FALSE;

    *out = (RotTargetOut){ 0 };
    out->az_sat = az;
    out->el_sat = el;

    calib_raw = ctrl->calibration_active || ctrl->cal_hold_active;
    if (calib_raw)
    {
        az = in_az;
        el = in_el;
        out->az_after_southzero = az;
        out->el_after_southzero = el;
        out->az_after_offsets = az;
        out->el_after_offsets = el;
        out->az360_final = az;
        out->el_final = el;
        return TRUE;
    }

    south_zero = g_atomic_int_get(&ctrl->south_zero_cached) != 0;
    use_offset = ctrl->use_offset;

    if (south_zero)
        az = gp_norm360(az + 180.0);
    out->az_after_southzero = az;
    out->el_after_southzero = el;

    if (ctrl->conf && ctrl->conf->invert_az)
        az = 360.0 - az;
    if (ctrl->conf && ctrl->conf->invert_el)
        el = 180.0 - el;

    if (use_offset)
    {
        az += ctrl->az_offset_deg;
        el += ctrl->el_offset_deg;
    }
    az = gp_norm360(az);

    az = wrap360(az - ctrl->calibration_offset_az);
    el -= ctrl->calibration_offset_el;
    az = gp_norm360(az);
    out->az_after_offsets = az;
    out->el_after_offsets = el;

    if (elev_floor >= 0.0 && el < elev_floor)
        el = elev_floor;

    out->az360_final = gp_norm360(az);
    out->el_final = el;

    if (rotctrl_transform_debug_enabled())
    {
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rot xform: sat(az=%.2f el=%.2f) southzero=%.2f offs=%.2f/%.2f -> az360=%.2f el=%.2f",
                    out->az_sat,
                    out->el_sat,
                    out->az_after_southzero,
                    out->az_after_offsets,
                    out->el_after_offsets,
                    out->az360_final,
                    out->el_final);
    }

    return isfinite(out->az360_final) && isfinite(out->el_final);
}

static void rotctrl_pipeline_build(GtkRotCtrl *ctrl,
                                   gdouble target_az360,
                                   gdouble target_el,
                                   gdouble backend_min_az,
                                   gdouble backend_max_az,
                                   gdouble backend_min_el,
                                   gdouble backend_max_el,
                                   gdouble last_cmd_backend,
                                   gboolean have_ref,
                                   gboolean locked_k_valid,
                                   gint locked_k,
                                   gdouble endstop_margin_deg,
                                   gdouble lane_switch_penalty_deg,
                                   gboolean apply_elev_floor,
                                   RotCmdPipeline *out)
{
    gdouble ui_az = target_az360;
    gdouble ui_el = target_el;
    gboolean az_clamped = FALSE;
    gboolean el_clamped = FALSE;
    gdouble cmd_az = target_az360;
    gdouble cmd_el = target_el;
    rot_ui_mode_t ui_mode = ROT_UI_360;
    gint lane_k = 0;
    gdouble lane_ref_backend = 0.0;
    gboolean lane_used_margin = FALSE;
    gboolean lane_switched = FALSE;
    gboolean lane_clamped = FALSE;

    if (out != NULL)
        memset(out, 0, sizeof(*out));

    if (ctrl == NULL || out == NULL)
        return;

    if (ctrl->conf && ctrl->conf->aztype == ROT_AZ_TYPE_180)
        ui_mode = ROT_UI_NORTH_CENTERED;

    ui_az = rot_az360_to_ui(target_az360, ui_mode);
    ui_el = target_el;

    lane_ref_backend = have_ref ? last_cmd_backend : target_az360;
    cmd_az = rotctrl_choose_backend_lane(ctrl,
                                         target_az360,
                                         lane_ref_backend,
                                         backend_min_az,
                                         backend_max_az,
                                         locked_k_valid,
                                         locked_k,
                                         endstop_margin_deg,
                                         lane_switch_penalty_deg,
                                         &lane_k,
                                         &lane_used_margin,
                                         &lane_switched,
                                         &lane_clamped);
    az_clamped = lane_clamped;
    cmd_az = rotctrl_normalize_backend_az(cmd_az, backend_min_az, backend_max_az);

    cmd_el = rotctrl_clamp_el_for_backend(ctrl,
                                          cmd_el,
                                          backend_min_el,
                                          backend_max_el,
                                          &el_clamped);
    if (apply_elev_floor)
    {
        gdouble before_floor = cmd_el;
        cmd_el = rotor_apply_elev_floor(cmd_el, rotctrl_elev_floor(ctrl));
        if (ctrl != NULL && fabs(cmd_el - before_floor) > 1e-6)
        {
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        "rot elev floor: before=%.2f floor=%.2f -> %.2f",
                        before_floor, rotctrl_elev_floor(ctrl), cmd_el);
        }
    }

    out->az360 = rot_norm360(target_az360);
    out->el = target_el;
    out->ui_az = ui_az;
    out->ui_el = ui_el;
    out->cmd_az = cmd_az;
    out->cmd_el = cmd_el;
    out->az_clamped = az_clamped;
    out->el_clamped = el_clamped;
    out->lane_crosses_seam = FALSE;
    out->lane_used_margin = lane_used_margin;
    out->lane_switched = lane_switched;
    out->lane_k = lane_k;
    out->lane_ref_backend = lane_ref_backend;
}

static gint rotctrl_poll_period_ms(const GtkRotCtrl *ctrl)
{
    gint val = ROTCTLD_DEFAULT_POLL_PERIOD_MS;

    if (ctrl && ctrl->cal_active &&
        (ctrl->cal_state == ROT_AUTOCAL_SETTLE ||
         ctrl->cal_state == ROT_AUTOCAL_READY))
        return ROT_AUTOCAL_POLL_MS;

    if (ctrl && ctrl->conf && ctrl->conf->rotor_poll_period_ms > 0)
        val = ctrl->conf->rotor_poll_period_ms;
    else if (ctrl && ctrl->delay > 0)
        val = (gint)ctrl->delay;

    if (val < 100)
        val = 100;

    return val;
}

static gint rotctrl_getpos_timeout_ms(const GtkRotCtrl *ctrl)
{
    gint poll_ms = rotctrl_poll_period_ms(ctrl);
    gint timeout_ms = poll_ms * 3;

    if (timeout_ms < ROT_CMD_POS_TIMEOUT_MS)
        timeout_ms = ROT_CMD_POS_TIMEOUT_MS;
    if (timeout_ms > 6000)
        timeout_ms = 6000;

    return timeout_ms;
}

static gint rotctrl_cycles_from_ms_ceil(gint ms, gint poll_ms)
{
    if (poll_ms <= 0)
        poll_ms = 1;
    if (ms <= 0)
        return 0;
    return (ms + poll_ms - 1) / poll_ms;
}

static gint rotctrl_stale_ms(const GtkRotCtrl *ctrl)
{
    return rotctrl_stale_degraded_ms(ctrl);
}

static gint rotctrl_plot_stale_hide_ms(const GtkRotCtrl *ctrl)
{
    gint hold_ms = rotctrl_stale_degraded_ms(ctrl) * 3;

    /* Keep the last plotted rotor position through brief feedback gaps to
     * avoid visual jumping between commanded and measured markers.
     */
    if (hold_ms < 5000)
        hold_ms = 5000;

    return hold_ms;
}

static gint rotctrl_stale_warn_floor_ms(const GtkRotCtrl *ctrl)
{
    gint poll_ms = rotctrl_poll_period_ms(ctrl);
    gint floor_ms = poll_ms * 2;
    gint timeout_floor_ms = rotctrl_getpos_timeout_ms(ctrl) + poll_ms;

    /* Budget one slow get_position round-trip before warning. */
    if (floor_ms < timeout_floor_ms)
        floor_ms = timeout_floor_ms;

    if (floor_ms < 500)
        floor_ms = 500;

    return floor_ms;
}

static gint rotctrl_stale_degraded_floor_ms(const GtkRotCtrl *ctrl)
{
    gint poll_ms = rotctrl_poll_period_ms(ctrl);
    gint floor_ms = poll_ms * 3;
    gint warn_floor_ms = rotctrl_stale_warn_floor_ms(ctrl);
    gint timeout_floor_ms = rotctrl_getpos_timeout_ms(ctrl) + (poll_ms * 2);

    /* Require sustained misses beyond normal poll+timeout jitter. */
    if (floor_ms < timeout_floor_ms)
        floor_ms = timeout_floor_ms;

    if (floor_ms < warn_floor_ms)
        floor_ms = warn_floor_ms;

    return floor_ms;
}

static gint rotctrl_stale_warn_ms(const GtkRotCtrl *ctrl)
{
    gint val = ROT_CMD_POS_FRESH_MS;
    gint poll_ms = rotctrl_poll_period_ms(ctrl);
    gint floor_ms = rotctrl_stale_warn_floor_ms(ctrl);
    gint cycles = ROT_STALE_WARN_CYCLES;

    if (ctrl && ctrl->conf && ctrl->conf->rotor_stale_warn_ms > 0)
        val = ctrl->conf->rotor_stale_warn_ms;

    cycles = MAX(cycles, rotctrl_cycles_from_ms_ceil(val, poll_ms));
    cycles = MAX(cycles, rotctrl_cycles_from_ms_ceil(floor_ms, poll_ms));

    return cycles * poll_ms;
}

static gint rotctrl_stale_degraded_ms(const GtkRotCtrl *ctrl)
{
    gint val = ROTCTLD_DEFAULT_STALE_MS;
    gint poll_ms = rotctrl_poll_period_ms(ctrl);
    gint floor_ms = rotctrl_stale_degraded_floor_ms(ctrl);
    gint warn_ms = rotctrl_stale_warn_ms(ctrl);
    gint cycles = ROT_STALE_DEGRADED_CYCLES;
    gint warn_cycles = rotctrl_cycles_from_ms_ceil(warn_ms, poll_ms);

    if (ctrl && ctrl->conf && ctrl->conf->rotor_stale_degraded_ms > 0)
        val = ctrl->conf->rotor_stale_degraded_ms;
    else if (ctrl && ctrl->conf && ctrl->conf->rotor_position_stale_ms > 0)
        val = ctrl->conf->rotor_position_stale_ms;

    cycles = MAX(cycles, rotctrl_cycles_from_ms_ceil(val, poll_ms));
    cycles = MAX(cycles, rotctrl_cycles_from_ms_ceil(floor_ms, poll_ms));
    if (cycles < (warn_cycles + 1))
        cycles = warn_cycles + 1;

    return cycles * poll_ms;
}

static gint rotctrl_stale_hold_ms(const GtkRotCtrl *ctrl)
{
    gint val = ROT_STALE_ENTER_MS;

    if (ctrl && ctrl->conf && ctrl->conf->rotor_stale_hold_ms > 0)
        val = ctrl->conf->rotor_stale_hold_ms;

    if (val < rotctrl_stale_degraded_ms(ctrl))
        val = rotctrl_stale_degraded_ms(ctrl);

    return val;
}

static gint rotctrl_stale_park_ms(const GtkRotCtrl *ctrl)
{
    gint val = ROT_STALE_ENTER_MS;

    if (ctrl && ctrl->conf && ctrl->conf->rotor_stale_park_ms >= 0)
        val = ctrl->conf->rotor_stale_park_ms;

    if (val > 0 && val < rotctrl_stale_hold_ms(ctrl))
        val = rotctrl_stale_hold_ms(ctrl);

    return val;
}

static gint rotctrl_stale_resume_ms(const GtkRotCtrl *ctrl)
{
    gint val = ROT_STALE_EXIT_MS;

    if (ctrl && ctrl->conf && ctrl->conf->rotor_stale_resume_ms >= 0)
        val = ctrl->conf->rotor_stale_resume_ms;

    if (val < 0)
        val = 0;

    return val;
}

static gboolean rot_should_send(GtkRotCtrl *ctrl,
                                gdouble meas_az360,
                                gdouble meas_el,
                                gdouble tgt_az360,
                                gdouble tgt_el,
                                gdouble eps_az,
                                gdouble eps_el,
                                gboolean periodic_due,
                                gboolean not_at_target,
                                gint64 now_us,
                                gint64 min_interval_us,
                                const gchar **reason_out)
{
    gdouble d_az = rot_ang_dist_deg(meas_az360, tgt_az360);
    gdouble d_el = fabs(meas_el - tgt_el);

    if (ctrl == NULL)
        return FALSE;

    if (ctrl->last_send_us > 0 &&
        (now_us - ctrl->last_send_us) < min_interval_us)
    {
        if (reason_out)
            *reason_out = "HOLD_RATE";
        return FALSE;
    }

    if (periodic_due)
    {
        if (reason_out)
            *reason_out = "PERIODIC";
        return TRUE;
    }

    if (not_at_target)
    {
        if (reason_out)
            *reason_out = "NOT_AT_TARGET";
        return TRUE;
    }

    if (d_az >= eps_az || d_el >= eps_el)
    {
        if (reason_out)
            *reason_out = "THRESHOLD";
        return TRUE;
    }

    ctrl->above_eps_count = 0;
    if (reason_out)
        *reason_out = "HOLD_THRESHOLD";
    return FALSE;
}

static gdouble rotctrl_angle_epsilon(const GtkRotCtrl *ctrl)
{
    gdouble val = ROTCTLD_DEFAULT_ANGLE_EPS_DEG;

    if (ctrl && ctrl->conf && ctrl->conf->rotor_angle_epsilon_deg > 0.0)
        val = ctrl->conf->rotor_angle_epsilon_deg;

    if (val < 0.1)
        val = 0.1;

    return val;
}

static gdouble rotctrl_elev_floor(const GtkRotCtrl *ctrl)
{
    gdouble val = ROTCTLD_DEFAULT_ELEV_FLOOR_DEG;

    if (ctrl && ctrl->conf && ctrl->conf->rotor_elev_floor_deg >= 0.0)
        val = ctrl->conf->rotor_elev_floor_deg;

    if (val < 0.0)
        val = 0.0;

    return val;
}

static gdouble rotctrl_apply_el_overtravel(GtkRotCtrl *ctrl, gdouble target_el)
{
    gdouble el_out = target_el;
    gdouble el_min = 0.0;
    gdouble el_max = 180.0;

    if (ctrl == NULL || ctrl->conf == NULL || !ctrl->conf->el_overtravel_enable)
        return target_el;

    el_min = ctrl->conf->el_min_deg;
    el_max = ctrl->conf->el_max_deg;

    if (el_min > el_max)
    {
        gdouble tmp = el_min;
        el_min = el_max;
        el_max = tmp;
    }

    /* Future unwrap support can evaluate +/-360 elevation candidates here. */
    el_out = CLAMP(el_out, el_min, el_max);

    rot_term_log_verbose(ctrl, "gpredict:state",
                         "el overtravel enabled min=%.2f max=%.2f el_in=%.2f el_out=%.2f",
                         el_min, el_max, target_el, el_out);

    return el_out;
}

static gboolean rotctrl_stale_suppressed(const GtkRotCtrl *ctrl)
{
    if (ctrl == NULL)
        return TRUE;

    if (ctrl->conf && ctrl->conf->disable_pos_feedback_checks)
        return TRUE;

    if (ctrl->cal_active || ctrl->cal_hold_active)
        return TRUE;

    if (!ctrl->tracking)
        return TRUE;

    return (ctrl->target_state == ROT_TARGET_STATE_HOLD ||
            ctrl->target_state == ROT_TARGET_STATE_IDLE);
}

static gdouble rotctrl_clamp_el_for_backend(GtkRotCtrl *ctrl,
                                            gdouble target_el,
                                            gdouble backend_min_el,
                                            gdouble backend_max_el,
                                            gboolean *clamped_out)
{
    gdouble clamped = CLAMP(target_el, backend_min_el, backend_max_el);
    gboolean changed = fabs(clamped - target_el) > 1e-6;

    if (clamped_out)
        *clamped_out = changed;

    if (changed && ctrl != NULL)
    {
        rot_log_rate_limited(ctrl, &ctrl->backend_clamp_rate, 2000000,
                             SAT_LOG_LEVEL_WARN, "gpredict:warn",
                             "backend el clamp: target=%.2f range=(%.2f..%.2f) -> %.2f",
                             target_el, backend_min_el, backend_max_el, clamped);
    }

    return clamped;
}

static void rotctrl_format_utc_jd(gdouble jd, gchar *buf, gsize buflen)
{
    time_t unix_time;
    struct tm *utc_tm;

    if (buf == NULL || buflen == 0)
        return;

    unix_time = (time_t)((jd - 2440587.5) * 86400.0);
    utc_tm = gmtime(&unix_time);
    if (utc_tm == NULL)
    {
        g_strlcpy(buf, "unknown", buflen);
        return;
    }

    if (strftime(buf, buflen, "%Y-%m-%d %H:%M:%S", utc_tm) == 0)
        g_strlcpy(buf, "unknown", buflen);
}

static gboolean rotctrl_predict_at(GtkRotCtrl *ctrl,
                                   gdouble t,
                                   gdouble *az360_out,
                                   gdouble *el_out)
{
    sat_t sat;

    if (ctrl == NULL || ctrl->target == NULL || ctrl->qth == NULL)
        return FALSE;

    sat = *ctrl->target;
    predict_calc(&sat, ctrl->qth, t);
    if (!isfinite(sat.az) || !isfinite(sat.el))
        return FALSE;

    if (az360_out)
        *az360_out = rot_norm360(sat.az);
    if (el_out)
        *el_out = sat.el;

    return TRUE;
}

static gboolean rotctrl_find_next_aos(GtkRotCtrl *ctrl,
                                      gdouble t_start,
                                      gdouble lookahead_sec,
                                      gdouble elev_floor,
                                      gdouble lead_sec,
                                      gdouble *aos_t_out,
                                      gdouble *az360_out,
                                      gdouble *el_out)
{
    gdouble t_end;
    gdouble step_sec = ROT_PRETRACK_SEARCH_STEP_SEC;
    gdouble step = step_sec / secday;
    gdouble prev_t = 0.0;
    gdouble prev_el = 0.0;
    gdouble cur_el = 0.0;
    gboolean prev_valid = FALSE;
    gboolean found = FALSE;
    gdouble lo = 0.0;
    gdouble hi = 0.0;

    if (ctrl == NULL || ctrl->target == NULL || ctrl->qth == NULL)
        return FALSE;

    if (lookahead_sec <= 0.0)
        lookahead_sec = ROT_PRETRACK_LOOKAHEAD_SEC;
    if (step_sec <= 0.0)
        step_sec = 1.0;

    t_end = t_start + lookahead_sec / secday;

    if (rotctrl_predict_at(ctrl, t_start, NULL, &prev_el))
    {
        prev_t = t_start;
        prev_valid = TRUE;
        if (prev_el >= elev_floor)
        {
            found = TRUE;
            lo = prev_t;
            hi = prev_t;
        }
    }

    for (gdouble t = t_start + step; !found && t <= t_end + 1e-9; t += step)
    {
        if (!rotctrl_predict_at(ctrl, t, NULL, &cur_el))
        {
            prev_valid = FALSE;
            continue;
        }

        if (!prev_valid && cur_el >= elev_floor)
        {
            found = TRUE;
            lo = t - step;
            if (lo < t_start)
                lo = t_start;
            hi = t;
            break;
        }

        if (prev_valid && prev_el < elev_floor && cur_el >= elev_floor)
        {
            found = TRUE;
            lo = prev_t;
            hi = t;
            break;
        }

        prev_el = cur_el;
        prev_t = t;
        prev_valid = TRUE;
    }

    if (!found)
        return FALSE;

    if (hi > lo)
    {
        for (int i = 0; i < 32 && (hi - lo) * secday > ROT_PRETRACK_REFINE_SEC; i++)
        {
            gdouble mid = 0.5 * (lo + hi);
            gdouble mid_el = 0.0;

            if (!rotctrl_predict_at(ctrl, mid, NULL, &mid_el))
            {
                lo = mid;
                continue;
            }

            if (mid_el >= elev_floor)
                hi = mid;
            else
                lo = mid;
        }
    }

    {
        gdouble aos_t = hi;
        gdouble lead_t = aos_t + lead_sec / secday;
        gdouble az360 = 0.0;
        gdouble el = 0.0;

        if (!rotctrl_predict_at(ctrl, lead_t, &az360, &el))
        {
            if (!rotctrl_predict_at(ctrl, aos_t, &az360, &el))
                return FALSE;
        }

        if (el < elev_floor)
            el = elev_floor;

        if (aos_t_out)
            *aos_t_out = aos_t;
        if (az360_out)
            *az360_out = az360;
        if (el_out)
            *el_out = el;
    }

    return TRUE;
}

static void rotctrl_build_target_caps(GtkRotCtrl *ctrl,
                                      rot_target_caps_t *caps)
{
    rot_target_caps_t out = { 0 };
    gboolean backend_caps_valid = FALSE;
    gdouble backend_az_min = 0.0;
    gdouble backend_az_max = 0.0;
    gdouble backend_el_min = 0.0;
    gdouble backend_el_max = 0.0;

    if (ctrl && ctrl->conf) {
        out.az_min_deg = ctrl->conf->minaz;
        out.az_max_deg = ctrl->conf->maxaz;
        out.el_min_deg = ctrl->conf->minel;
        out.el_max_deg = ctrl->conf->maxel;
        out.az_wrap_mode = (ctrl->conf->aztype == ROT_AZ_TYPE_180)
                           ? ROT_TARGET_WRAP_180
                           : ROT_TARGET_WRAP_360;
    } else {
        out.az_min_deg = 0.0;
        out.az_max_deg = 360.0;
        out.el_min_deg = ROTCTRL_DEFAULT_MIN_EL;
        out.el_max_deg = ROTCTRL_DEFAULT_MAX_EL;
        out.az_wrap_mode = ROT_TARGET_WRAP_360;
    }

    if (ctrl)
    {
        g_mutex_lock(&ctrl->client.mutex);
        backend_caps_valid = ctrl->client.limits_valid;
        if (backend_caps_valid)
        {
            backend_az_min = ctrl->client.az_min;
            backend_az_max = ctrl->client.az_max;
            backend_el_min = ctrl->client.el_min;
            backend_el_max = ctrl->client.el_max;
        }
        g_mutex_unlock(&ctrl->client.mutex);
    }

    if (backend_caps_valid)
    {
        out.az_min_deg = backend_az_min;
        out.az_max_deg = backend_az_max;
        out.el_min_deg = backend_el_min;
        out.el_max_deg = backend_el_max;
    }

    out.clamp_policy = ROT_TARGET_CLAMP_REJECT;
    out.shortest_path = ROT_TARGET_SHORTEST_PATH;

    if (caps)
        *caps = out;
}

static gboolean rotctrl_target_is_valid(const rot_target_caps_t *caps,
                                        gdouble az,
                                        gdouble el,
                                        gdouble *az_norm_out,
                                        rot_target_invalid_reason_t *reason_out,
                                        gboolean *wrap_mismatch_out)
{
    rot_target_invalid_reason_t reason = ROT_TARGET_INVALID_NONE;
    gboolean ok = rot_target_is_valid(caps, az, el, az_norm_out, &reason);
    gboolean wrap_mismatch = (reason == ROT_TARGET_INVALID_WRAP_MISMATCH);

    if (!ok && wrap_mismatch)
        ok = TRUE;

    if (reason_out)
        *reason_out = reason;
    if (wrap_mismatch_out)
        *wrap_mismatch_out = wrap_mismatch;

    return ok;
}

static gboolean rotctrl_use_rotctld_caps(GtkRotCtrl *ctrl,
                                         gboolean caps_valid,
                                         gdouble caps_az_min,
                                         gdouble caps_az_max,
                                         gdouble caps_el_min,
                                         gdouble caps_el_max)
{
    (void)ctrl;
    (void)caps_az_min;
    (void)caps_az_max;
    (void)caps_el_min;
    (void)caps_el_max;

    if (!caps_valid)
        return FALSE;

    return TRUE;
}

static gdouble G_GNUC_UNUSED rotctrl_az_distance(const rot_target_caps_t *caps,
                                                 gdouble a,
                                                 gdouble b)
{
    if (caps)
    {
        gdouble span = caps->az_max_deg - caps->az_min_deg;
        if (span > 360.0 + 1e-6)
            return fabs(a - b);

        if (caps->shortest_path == ROT_TARGET_NO_WRAP)
            return fabs(a - b);

        if (caps->az_wrap_mode == ROT_TARGET_WRAP_180)
            return rot_target_az_distance_wrap180(a, b);
    }

    return rot_target_az_distance_wrap360(a, b);
}

static const gchar *rot_target_state_name(rot_target_state_t state)
{
    switch (state)
    {
    case ROT_TARGET_STATE_IDLE:
        return "IDLE";
    case ROT_TARGET_STATE_HOLD:
        return "HOLD";
    case ROT_TARGET_STATE_PRETRACK:
        return "PRETRACK";
    case ROT_TARGET_STATE_TRACKING_NORMAL:
        return "TRACKING_NORMAL";
    case ROT_TARGET_STATE_TRACKING_DEGRADED:
        return "TRACKING_DEGRADED";
    default:
        return "TRACKING_NORMAL";
    }
}

static void rot_target_state_set(GtkRotCtrl *ctrl,
                                 rot_target_state_t state,
                                 const gchar *reason)
{
    const gchar *from;
    const gchar *to;
    gint64 now_us;
    gdouble cur_az = 0.0;
    gdouble cur_el = 0.0;
    gboolean pos_valid = FALSE;
    rot_target_wrap_mode_t wrap_mode = ROT_TARGET_WRAP_360;

    if (ctrl == NULL)
        return;

    if (ctrl->target_state == state)
        return;

    from = rot_target_state_name(ctrl->target_state);
    to = rot_target_state_name(state);
    now_us = g_get_monotonic_time();

    if (ctrl->user_limits.valid)
        wrap_mode = ctrl->user_limits.wrap_mode;
    else if (ctrl->conf && ctrl->conf->aztype == ROT_AZ_TYPE_180)
        wrap_mode = ROT_TARGET_WRAP_180;

    g_mutex_lock(&ctrl->client.mutex);
    pos_valid = ctrl->client.pos_valid;
    cur_az = ctrl->client.azi_in;
    cur_el = ctrl->client.ele_in;
    g_mutex_unlock(&ctrl->client.mutex);
    if (pos_valid)
        cur_el = rotor_apply_elev_floor(cur_el, rotctrl_elev_floor(ctrl));

    sat_log_log(SAT_LOG_LEVEL_INFO,
                "rotor_target_state %s->%s reason=%s az=%.2f el=%.2f wrap=%s",
                from, to, reason ? reason : "none",
                cur_az, cur_el, rotctrl_wrap_mode_name(wrap_mode));
    rot_term_log_verbose(ctrl, "gpredict:state",
                         "rotor_target_state %s->%s reason=%s az=%.2f el=%.2f wrap=%s",
                         from, to, reason ? reason : "none",
                         cur_az, cur_el, rotctrl_wrap_mode_name(wrap_mode));

    {
        gboolean was_tracking =
            (ctrl->target_state == ROT_TARGET_STATE_PRETRACK ||
             ctrl->target_state == ROT_TARGET_STATE_TRACKING_NORMAL ||
             ctrl->target_state == ROT_TARGET_STATE_TRACKING_DEGRADED);
        gboolean now_tracking =
            (state == ROT_TARGET_STATE_PRETRACK ||
             state == ROT_TARGET_STATE_TRACKING_NORMAL ||
             state == ROT_TARGET_STATE_TRACKING_DEGRADED);

        if (now_tracking && !was_tracking)
            ctrl->force_next_send = TRUE;
    }

    ctrl->target_state = state;
    ctrl->target_state_since_us = now_us;
}

static gboolean G_GNUC_UNUSED rotctrl_find_first_valid_sample(GtkRotCtrl *ctrl,
                                                const rot_target_caps_t *caps,
                                                gdouble t_start,
                                                gdouble t_end,
                                                gdouble step_sec,
                                                gboolean use_flip,
                                                gdouble *out_az,
                                                gdouble *out_el,
                                                gdouble *out_t,
                                                rot_target_invalid_reason_t *reason_out)
{
    GArray *samples = NULL;
    rot_target_sample_t out = { 0 };
    gboolean ok = FALSE;
    rot_target_invalid_reason_t reason = ROT_TARGET_INVALID_NONE;

    if (ctrl == NULL || ctrl->conf == NULL || ctrl->target == NULL ||
        ctrl->qth == NULL || caps == NULL)
        return FALSE;

    if (t_end < t_start)
        return FALSE;

    if (step_sec <= 0.0)
        step_sec = ROT_PLAN_SAMPLE_DT_SEC;

    samples = g_array_new(FALSE, FALSE, sizeof(rot_target_sample_t));

    gdouble dt = step_sec / secday;
    for (gdouble t = t_start; t <= t_end + 1e-9; t += dt)
    {
        sat_t sat = *ctrl->target;
        gdouble az = 0.0;
        gdouble el = 0.0;
        RotTransformPre pre = { 0 };
        rot_target_sample_t sample = { 0 };

        predict_calc(&sat, ctrl->qth, t);
        az = sat.az;
        el = sat.el;

        if (use_flip && ctrl->conf->maxel >= 180.0)
        {
            el = 180.0 - el;
            if (az > 180.0)
                az -= 180.0;
            else
                az += 180.0;
        }

        if (ctrl->conf->aztype == ROT_AZ_TYPE_180 && az > 180.0)
            az -= 360.0;

        if (ctrl->conf->axis_mode == ROT_AXIS_MODE_AZ_ONLY)
            el = ctrl->conf->minel;

        rot_transform_prepare(&ctrl->transform, az, el, &pre);

        sample.t = t;
        sample.az = pre.az_abs;
        sample.el = pre.el_raw;
        g_array_append_val(samples, sample);
    }

    ok = rot_target_find_first_valid_sample(caps,
                                            (const rot_target_sample_t *)samples->data,
                                            samples->len,
                                            t_start, t_end,
                                            &out,
                                            &reason);

    if (ok)
    {
        sat_t sat = *ctrl->target;
        gdouble az = 0.0;
        gdouble el = 0.0;

        predict_calc(&sat, ctrl->qth, out.t);
        az = sat.az;
        el = sat.el;

        if (use_flip && ctrl->conf->maxel >= 180.0)
        {
            el = 180.0 - el;
            if (az > 180.0)
                az -= 180.0;
            else
                az += 180.0;
        }

        if (ctrl->conf->aztype == ROT_AZ_TYPE_180 && az > 180.0)
            az -= 360.0;

        if (ctrl->conf->axis_mode == ROT_AXIS_MODE_AZ_ONLY)
            el = ctrl->conf->minel;

        if (out_az)
            *out_az = az;
        if (out_el)
            *out_el = el;
        if (out_t)
            *out_t = out.t;
    }

    if (reason_out)
        *reason_out = reason;

    if (samples)
        g_array_free(samples, TRUE);

    return ok;
}

static void rot_transform_prepare(const RotTransformSnapshot *transform,
                                  gdouble in_az, gdouble in_el,
                                  RotTransformPre *pre)
{
    gdouble az = in_az;
    gdouble el = in_el;
    gdouble span_width = (transform->az_norm_mode == ROT_AZ_TYPE_480) ? 480.0 : 360.0;

    if (pre == NULL || transform == NULL)
        return;

    if (transform->az_norm_mode == ROT_AZ_TYPE_480)
        az = az_abs_to_span(az, AZSPAN_480);
    else
        az = norm360(az);

    if (transform->az_invert)
        az = span_width - az;
    if (transform->el_invert)
        el = 180.0 - el;

    az += transform->az_offset_deg;
    if (transform->az_norm_mode == ROT_AZ_TYPE_480)
        az = az_abs_to_span(az, AZSPAN_480);
    else
        az = norm360(az);
    el = el + transform->el_offset_deg;

    pre->az_abs = az;
    pre->el_raw = el;
}

static void rot_transform_snapshot_defaults(RotTransformSnapshot *transform)
{
    if (transform == NULL)
        return;

    transform->az_offset_deg = 0.0;
    transform->el_offset_deg = 0.0;
    transform->az_invert = FALSE;
    transform->el_invert = FALSE;
    transform->az_min_deg = 0.0;
    transform->az_max_deg = 360.0;
    transform->el_min_deg = 0.0;
    transform->el_max_deg = 90.0;
    transform->az_norm_mode = ROT_AZ_TYPE_360;
    transform->zenith_guard_enable = TRUE;
    transform->zenith_guard_el_deg = 85.0;
    transform->south_zero = FALSE;
}

static void rot_transform_snapshot_state_init(RotTransformSnapshotState *state)
{
    if (state == NULL)
        return;

    rot_transform_snapshot_defaults(&state->transform);
    state->version = 0;
    state->ready = FALSE;
}

static void rot_transform_snapshot_publish(GtkRotCtrl *ctrl, gboolean ready)
{
    RotTransformSnapshotState state;

    if (ctrl == NULL)
        return;

    state.transform = ctrl->transform;
    state.version = ++ctrl->transform_snapshot_version;
    state.ready = ready;

    g_mutex_lock(&ctrl->transform_snapshot_mutex);
    ctrl->transform_snapshot = state;
    g_mutex_unlock(&ctrl->transform_snapshot_mutex);
}

static gboolean rot_transform_snapshot_state_get(GtkRotCtrl *ctrl,
                                                 RotTransformSnapshotState *out)
{
    if (ctrl == NULL || out == NULL)
        return FALSE;

    g_mutex_lock(&ctrl->transform_snapshot_mutex);
    *out = ctrl->transform_snapshot;
    g_mutex_unlock(&ctrl->transform_snapshot_mutex);

    return out->ready;
}

static void rot_transform_update(GtkRotCtrl *ctrl)
{
    RotTransformSnapshot *transform;
    gboolean south_zero = FALSE;

    if (ctrl == NULL)
        return;

    south_zero = g_atomic_int_get(&ctrl->south_zero_cached) != 0;

    transform = &ctrl->transform;
    if (ctrl->calibration_active)
    {
        transform->az_offset_deg = 0.0;
        transform->el_offset_deg = 0.0;
    }
    else
    {
        gdouble user_az = ctrl->use_offset ? ctrl->az_offset_deg : 0.0;
        gdouble user_el = ctrl->use_offset ? ctrl->el_offset_deg : 0.0;
        transform->az_offset_deg = user_az - ctrl->calibration_offset_az;
        transform->el_offset_deg = user_el - ctrl->calibration_offset_el;
    }
    transform->az_invert = (ctrl->conf && ctrl->conf->invert_az);
    transform->el_invert = (ctrl->conf && ctrl->conf->invert_el);
    transform->az_norm_mode =
        (ctrl->conf && ctrl->conf->aztype == ROT_AZ_TYPE_180)
            ? ROT_AZ_TYPE_180
            : ROT_AZ_TYPE_360;
    transform->zenith_guard_enable = TRUE;
    transform->zenith_guard_el_deg = 85.0;
    transform->south_zero = south_zero;

    if (ctrl->conf)
    {
        rot_cmd_get_abs_az_limits(ctrl->conf,
                                  &transform->az_min_deg,
                                  &transform->az_max_deg);
        transform->el_min_deg = ctrl->conf->minel;
        transform->el_max_deg = ctrl->conf->maxel;
    }
    else
    {
        transform->az_min_deg = 0.0;
        transform->az_max_deg = 360.0;
        transform->el_min_deg = 0.0;
        transform->el_max_deg = 90.0;
    }

    if (transform->el_min_deg > transform->el_max_deg)
    {
        gdouble tmp = transform->el_min_deg;
        transform->el_min_deg = transform->el_max_deg;
        transform->el_max_deg = tmp;
    }

    rot_transform_snapshot_publish(ctrl, ctrl->conf != NULL);
}

static void apply_inverse_transform(const RotTransformSnapshot *transform,
                                    gdouble in_az, gdouble in_el,
                                    gdouble *out_az, gdouble *out_el)
{
    gdouble az = in_az;
    gdouble el = in_el;
    gdouble span_width = (transform->az_norm_mode == ROT_AZ_TYPE_480) ? 480.0 : 360.0;

    if (transform == NULL)
        return;

    if (transform->az_norm_mode == ROT_AZ_TYPE_480)
        az = az_abs_to_span(az, AZSPAN_480);
    else
        az = norm360(az);

    az -= transform->az_offset_deg;
    if (transform->az_norm_mode == ROT_AZ_TYPE_480)
        az = az_abs_to_span(az, AZSPAN_480);
    else
        az = norm360(az);
    el = el - transform->el_offset_deg;

    if (transform->az_invert)
        az = span_width - az;
    if (transform->el_invert)
        el = 180.0 - el;

    if (transform->az_norm_mode == ROT_AZ_TYPE_480)
        az = az_abs_to_span(az, AZSPAN_480);
    else
        az = norm360(az);

    if (transform->az_norm_mode == ROT_AZ_TYPE_180)
        az = az_abs_to_span(az, AZSPAN_PM180);
    else if (transform->az_norm_mode == ROT_AZ_TYPE_480)
        az = az_abs_to_span(az, AZSPAN_480);
    else
        az = az_abs_to_span(az, AZSPAN_360);

    if (out_az)
        *out_az = az;
    if (out_el)
        *out_el = el;
}

static void rotctrl_map_mech_to_user(GtkRotCtrl *ctrl,
                                     gdouble mech_az,
                                     gdouble mech_el,
                                     gdouble *user_az,
                                     gdouble *user_el)
{
    RotTransformSnapshotState snap_state;
    gdouble az_conf = mech_az;
    gdouble el_conf = mech_el;

    if (ctrl != NULL)
    {
        rot_transform_snapshot_state_init(&snap_state);
        if (rot_transform_snapshot_state_get(ctrl, &snap_state))
        {
            apply_inverse_transform(&snap_state.transform,
                                    mech_az, mech_el,
                                    &az_conf, &el_conf);
        }
        else
        {
            apply_inverse_transform(&ctrl->transform,
                                    mech_az, mech_el,
                                    &az_conf, &el_conf);
        }
        az_conf = rot_az_to_conf(ctrl->conf, az_conf);
    }

    if (user_az)
        *user_az = az_conf;
    if (user_el)
        *user_el = el_conf;
}

static gboolean rot_manual_input_event(GtkWidget *widget, GdkEvent *event,
                                       gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);

    (void)widget;
    (void)event;

    if (ctrl == NULL)
        return FALSE;

    if (ctrl->cal_hold_active)
        rotctrl_set_cal_hold(ctrl, FALSE, "manual_input");

    ctrl->manual_edit_until_us = g_get_monotonic_time() + 1000000;
    ctrl->manual_sync_pending = FALSE;
    ctrl->have_user_command = TRUE;
    ctrl->force_next_send = TRUE;
    ctrl->hold_position_on_engage = FALSE;
    return FALSE;
}

static gboolean rotctrl_should_sync_manual(GtkRotCtrl *ctrl)
{
    gint64 now;

    if (ctrl == NULL)
        return FALSE;

    if (!ctrl->manual_sync_pending)
        return FALSE;

    now = g_get_monotonic_time();
    if (now < ctrl->manual_edit_until_us)
        return FALSE;

    return TRUE;
}

static gboolean rotctrl_sync_manual_from_position(GtkRotCtrl *ctrl,
                                                  gdouble az_abs, gdouble el,
                                                  gdouble *setaz,
                                                  gdouble *setel)
{
    gdouble az_conf;
    gdouble el_conf;
    gdouble cur_az;
    gdouble cur_el;
    gboolean changed = FALSE;
    gint64 now;

    if (!rotctrl_should_sync_manual(ctrl))
        return FALSE;

    apply_inverse_transform(&ctrl->transform, az_abs, el, &az_conf, &el_conf);
    if (ctrl->conf && ctrl->conf->axis_mode == ROT_AXIS_MODE_AZ_ONLY)
        el_conf = ctrl->conf->minel;
    cur_az = gtk_rot_knob_get_value(GTK_ROT_KNOB(ctrl->AzSet));
    cur_el = gtk_rot_knob_get_value(GTK_ROT_KNOB(ctrl->ElSet));

    gdouble az_delta = (ctrl->conf && ctrl->conf->aztype == ROT_AZ_TYPE_480)
                       ? fabs(cur_az - az_conf)
                       : fabs(shortest_az_delta(cur_az, az_conf));

    if (az_delta >= 0.05 ||
        fabs(cur_el - el_conf) >= 0.05)
    {
        gtk_rot_knob_set_value(GTK_ROT_KNOB(ctrl->AzSet), az_conf);
        gtk_rot_knob_set_value(GTK_ROT_KNOB(ctrl->ElSet), el_conf);
        if (setaz)
            *setaz = az_conf;
        if (setel)
            *setel = el_conf;
        changed = TRUE;
    }

    if (changed)
    {
        now = g_get_monotonic_time();
        if (now - ctrl->last_manual_sync_log_us > 2000000)
        {
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rotor manual sync: az=%.2f el=%.2f", az_conf, el);
            ctrl->last_manual_sync_log_us = now;
        }
        ctrl->manual_sync_pending = FALSE;
    }

    return changed;
}

static gboolean rot_parse_first_number(const gchar *line, gdouble *out)
{
    if (line == NULL || out == NULL)
        return FALSE;

    const gchar *p = line;
    while (*p != '\0' && !g_ascii_isdigit(*p) && *p != '-' && *p != '+' && *p != '.')
        p++;

    if (*p == '\0')
        return FALSE;

    gchar *endptr = NULL;
    gdouble v = g_ascii_strtod(p, &endptr);
    if (endptr == p)
        return FALSE;

    *out = v;
    return TRUE;
}

static gboolean rot_parse_first_two_numbers(const gchar *line,
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
            gdouble v = g_ascii_strtod(p, &endptr);
            if (endptr != p)
            {
                if (!have_first)
                {
                    first = v;
                    have_first = TRUE;
                }
                else
                {
                    second = v;
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

typedef enum {
    ROT_DAEMON_UNKNOWN = 0,
    ROT_DAEMON_ROTCTLD = 1,
    ROT_DAEMON_RIGCTLD = 2
} rot_daemon_type_t;

struct RotctldProbeState {
    GtkRotCtrl  *ctrl;
    guint64      generation;
    gint64       deadline_us;
    guint        attempt;
    guint        non_rotctld_count;
    guint        delay_ms;
    gboolean     autostart_allowed;
    gboolean     spawn_attempted;
    gboolean     spawned;
    gint         scanning_child_pid;
    gboolean     scanning_child_validated;
    gboolean     last_good_exclusive;
    gint64       last_good_deadline_us;
    gchar       *last_good_device;
    gint         last_good_baud;
    gint64       autodetect_start_us;
    gint64       autodetect_candidate_start_us;
    GSList      *autodetect_list_full;
    gboolean     autodetect_limited;
    gchar       *spawn_summary;
    gint         ref_count;
    gboolean     cancelled;
    gboolean     autodetect_enabled;
    rotctld_autodetect_state_t autodetect_state;
    guint64      autodetect_generation;
    gint64       autodetect_state_since_us;
    gint64       autodetect_tcp_deadline_us;
    gint64       autodetect_port_free_deadline_us;
    gint64       autodetect_settle_until_us;
    GSList      *autodetect_list;
    GSList      *autodetect_next;
    guint        autodetect_count;
    gchar       *autodetect_device;
    gint         autodetect_baud;
    gint         autodetect_port;
    guint        autodetect_baud_index;
    GString     *autodetect_attempts;
    gboolean     validate_inflight;
    gboolean     validate_done;
    rotctld_pos_result_t validate_result;
    gint         validate_score;
    gint         validate_rprt;
    gchar       *validate_reason;
    gboolean     validate_dump_ok;
    gboolean     validate_pos_ok;
    gboolean     validate_timeout;
    gint         validate_timeout_ms;
    gint         validate_retries;
    gint         validate_retry_delay_ms;
    gint         best_score;
    gchar       *best_device;
    gint         best_baud;
    GThread     *validate_thread;
    GMutex       validate_mutex;
};

static gboolean G_GNUC_UNUSED rotctld_line_is_numeric(const gchar *line)
{
    const gchar *p = line;

    if (p == NULL || *p == '\0')
        return FALSE;

    for (; *p != '\0'; p++)
    {
        if (!g_ascii_isdigit(*p))
            return FALSE;
    }

    return TRUE;
}

static gchar *rotctld_truncate_line(const gchar *line, gsize max_len)
{
    gsize len = 0;

    if (line == NULL)
        return NULL;

    len = strlen(line);
    if (len <= max_len)
        return g_strdup(line);

    return g_strndup(line, max_len);
}

static void rotctld_extract_first_lines(const gchar *text,
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

static gboolean rotctld_response_is_valid(const gchar *text,
                                          gchar **first_line_out)
{
    gchar **lines = NULL;
    gchar *first_line = NULL;
    gboolean ok = FALSE;

    if (first_line_out)
        *first_line_out = NULL;

    if (text == NULL || *text == '\0')
        return FALSE;

    lines = g_strsplit(text, "\n", -1);
    for (gint i = 0; lines[i] != NULL; i++)
    {
        gchar *line = g_strstrip(lines[i]);

        if (line[0] == '\0')
            continue;

        if (first_line == NULL)
        {
            first_line = g_strdup(line);
            ok = g_str_has_prefix(line, "1");
            break;
        }
    }
    g_strfreev(lines);

    if (first_line_out)
        *first_line_out = first_line;
    else
        g_free(first_line);

    return ok;
}

static const gchar *rotctld_probe_result_name(rotctld_probe_result_t result)
{
    switch (result)
    {
    case ROTCTLD_PROBE_OK_ROTCTLD:
        return "OK_ROTCTLD";
    case ROTCTLD_PROBE_NOT_READY:
        return "NOT_READY";
    case ROTCTLD_PROBE_NOT_ROTCTLD:
        return "NOT_ROTCTLD";
    default:
        return "UNKNOWN";
    }
}

static rot_daemon_type_t rotctld_detect_daemon(const gchar *reply)
{
    if (reply == NULL || *reply == '\0')
        return ROT_DAEMON_UNKNOWN;

    if (rotctld_response_is_valid(reply, NULL))
        return ROT_DAEMON_ROTCTLD;

    gchar *lower;
    gboolean has_rot = FALSE;
    gboolean has_rig = FALSE;

    lower = g_ascii_strdown(reply, -1);
    if (lower == NULL)
        return ROT_DAEMON_UNKNOWN;

    has_rot = (g_strrstr(lower, "rotator") != NULL) ||
              (g_strrstr(lower, "rot_model") != NULL) ||
              (g_strrstr(lower, "azimuth") != NULL) ||
              (g_strrstr(lower, "elevation") != NULL);

    has_rig = (g_strrstr(lower, "rig_model") != NULL) ||
              (g_strrstr(lower, "rigctld") != NULL) ||
              (g_strrstr(lower, "vfo") != NULL) ||
              (g_strrstr(lower, "ptt") != NULL);

    g_free(lower);

    if (has_rot && !has_rig)
        return ROT_DAEMON_ROTCTLD;
    if (has_rig && !has_rot)
        return ROT_DAEMON_RIGCTLD;

    return ROT_DAEMON_UNKNOWN;
}

static gboolean rotctld_extract_model(const gchar *reply, gint *model_out)
{
    return parse_dump_state_model_id(reply, model_out);
}

static rot_daemon_type_t G_GNUC_UNUSED rotctld_dump_state(GtkRotCtrl *ctrl,
                                                         gint sock,
                                                         gchar *reply,
                                                         gsize reply_size)
{
    gchar cmd[] = "\\dump_state\n";

    if (reply == NULL || reply_size == 0)
        return ROT_DAEMON_UNKNOWN;

    reply[0] = '\0';
    if (!rotctld_socket_rw(ctrl, sock, cmd, reply, reply_size - 1))
        return ROT_DAEMON_UNKNOWN;

    return rotctld_detect_daemon(reply);
}

static gboolean G_GNUC_UNUSED rotctld_parse_position_reply_ex(const gchar *reply,
                                                              gdouble *az_out,
                                                              gdouble *el_out,
                                                              gint *rprt_code_out,
                                                              gboolean *have_az_out,
                                                              gboolean *have_el_out,
                                                              gboolean *have_rprt_out)
{
    gboolean have_az = FALSE;
    gboolean have_el = FALSE;
    gint rprt_code = 0;
    gboolean have_code = FALSE;
    gdouble a = 0.0;
    gdouble b = 0.0;

    if (reply == NULL)
        return FALSE;

    gchar **lines = g_strsplit_set(reply, "\r\n", -1);
    for (gint i = 0; lines[i] != NULL; i++) {
        gchar *line = g_strstrip(lines[i]);
        if (line[0] == '\0')
            continue;

        if (g_ascii_strcasecmp(line, "done") == 0)
            continue;

        if (g_str_has_prefix(line, "RPRT")) {
            if (rot_parse_rprt_code(line, &rprt_code))
                have_code = TRUE;
            continue;
        }

        if (!have_az && !have_el &&
            rot_parse_first_two_numbers(line, &a, &b))
        {
            if (az_out)
                *az_out = a;
            if (el_out)
                *el_out = b;
            have_az = TRUE;
            have_el = TRUE;
            continue;
        }

        if (!have_az && az_out && rot_parse_first_number(line, az_out)) {
            have_az = TRUE;
            continue;
        }

        if (!have_el && el_out && rot_parse_first_number(line, el_out)) {
            have_el = TRUE;
        }
    }
    g_strfreev(lines);

    if (rprt_code_out)
        *rprt_code_out = have_code ? rprt_code : 0;
    if (have_az_out)
        *have_az_out = have_az;
    if (have_el_out)
        *have_el_out = have_el;
    if (have_rprt_out)
        *have_rprt_out = have_code;

    return have_az && have_el;
}

static gboolean G_GNUC_UNUSED rotctld_parse_position_reply(const gchar *reply,
                                                          gdouble *az_out,
                                                          gdouble *el_out,
                                                          gint *rprt_code_out)
{
    return rotctld_parse_position_reply_ex(reply, az_out, el_out,
                                           rprt_code_out,
                                           NULL, NULL, NULL);
}

static gchar *rotctld_sanitize_reply(const gchar *reply, gsize max_len)
{
    if (reply == NULL || *reply == '\0')
        return g_strdup("(none)");

    gsize len = strlen(reply);
    gsize limit = MIN(len, max_len);
    GString *out = g_string_sized_new(limit * 2 + 8);

    for (gsize i = 0; i < limit; i++)
    {
        unsigned char c = (unsigned char)reply[i];
        if (c == '\n')
            g_string_append(out, "\\n");
        else if (c == '\r')
            g_string_append(out, "\\r");
        else if (c == '\t')
            g_string_append(out, "\\t");
        else if (g_ascii_isprint(c))
            g_string_append_c(out, (gchar)c);
        else
            g_string_append_c(out, '.');
    }

    if (len > limit)
        g_string_append(out, "...");

    return g_string_free(out, FALSE);
}

static gboolean G_GNUC_UNUSED rotctld_handshake(GtkRotCtrl *ctrl,
                                                gint sock,
                                                gboolean *rejected)
{
    gchar reply[256];
    gdouble az = 0.0;
    gdouble el = 0.0;
    gint rprt_code = 0;
    gboolean have_pos = FALSE;
    RotTransformSnapshotState snap_state;

    if (rejected)
        *rejected = FALSE;

    rot_transform_snapshot_state_init(&snap_state);
    if (ctrl != NULL)
        (void)rot_transform_snapshot_state_get(ctrl, &snap_state);

    if (!rotctld_socket_rw(ctrl, sock, "p\n", reply, sizeof(reply) - 1))
        return FALSE;

    have_pos = rotctld_parse_position_reply(reply, &az, &el, &rprt_code);
    if (rprt_code != 0) {
        if (rejected)
            *rejected = TRUE;
        return FALSE;
    }

    if (!have_pos) {
        if (rejected)
            *rejected = TRUE;
        return FALSE;
    }

    if (ctrl != NULL)
    {
        const RotTransformSnapshot *snap = &snap_state.transform;
        AzSpan span_mode = rotctrl_span_from_az_type(snap->az_norm_mode);
        gdouble az_abs = az_abs_to_span(az, span_mode);
        if (!rot_az_in_limits(az_abs,
                              snap->az_min_deg,
                              snap->az_max_deg) ||
            el < (snap->el_min_deg - 1e-2) ||
            el > (snap->el_max_deg + 1e-2))
        {
            sat_log_log(SAT_LOG_LEVEL_WARN,
                        "%s: rotctld position out of range az=%.2f el=%.2f (az=%.2f..%.2f el=%.2f..%.2f)",
                        __func__, az, el,
                        snap->az_min_deg,
                        snap->az_max_deg,
                        snap->el_min_deg,
                        snap->el_max_deg);
            return FALSE;
        }
    }

    if (ctrl != NULL)
    {
        const RotTransformSnapshot *snap = &snap_state.transform;
        g_mutex_lock(&ctrl->client.mutex);
        ctrl->az_hold_active = FALSE;
        if (snap->zenith_guard_enable &&
            el >= snap->zenith_guard_el_deg)
        {
            ctrl->az_hold_active = TRUE;
            ctrl->az_hold_value = az;
        }
        g_mutex_unlock(&ctrl->client.mutex);
    }

    gdouble mech_az = az;
    gdouble mech_el = el;
    gdouble user_az = 0.0;
    gdouble user_el = 0.0;

    rotctrl_map_mech_to_user(ctrl, mech_az, mech_el, &user_az, &user_el);

    g_mutex_lock(&ctrl->client.mutex);
    ctrl->client.azi_mech_in = mech_az;
    ctrl->client.ele_mech_in = mech_el;
    ctrl->client.azi_in = user_az;
    ctrl->client.ele_in = user_el;
    ctrl->client.pos_valid = TRUE;
    ctrl->client.pos_unknown = FALSE;
    ctrl->client.pos_cmd_ok = TRUE;
    ctrl->client.last_pos_us = g_get_monotonic_time();
    g_mutex_unlock(&ctrl->client.mutex);
    ctrl->manual_sync_pending = TRUE;

    return TRUE;
}

typedef enum {
    ROT_SET_OK = 0,
    ROT_SET_REJECTED = 1,
    ROT_SET_BACKEND_IO = 2,
    ROT_SET_IO_ERROR = 3
} rot_set_result_t;

static void format_rotctld_setpos(GtkRotCtrl *ctrl,
                                  gdouble az_in,
                                  gdouble el_in,
                                  gdouble *az_out,
                                  gdouble *el_out,
                                  gchar *cmd_out,
                                  gsize cmd_len)
{
    const gdouble eps = 1e-6;
    gdouble az = az_in;
    gdouble el = el_in;
    gboolean caps_valid = FALSE;
    gboolean caps_extended = FALSE;
    gdouble az_min = 0.0;
    gdouble az_max = 360.0;
    gdouble el_min = ROTCTRL_DEFAULT_MIN_EL;
    gdouble el_max = ROTCTRL_DEFAULT_MAX_EL;
    gchar azbuf[G_ASCII_DTOSTR_BUF_SIZE];
    gchar elbuf[G_ASCII_DTOSTR_BUF_SIZE];

    if (ctrl != NULL)
    {
        if (ctrl->conf)
        {
            el_min = ctrl->conf->minel;
            el_max = ctrl->conf->maxel;
        }
        g_mutex_lock(&ctrl->client.mutex);
        if (ctrl->client.limits_valid)
        {
            caps_valid = TRUE;
            az_min = ctrl->client.az_min;
            az_max = ctrl->client.az_max;
            el_min = ctrl->client.el_min;
            el_max = ctrl->client.el_max;
        }
        g_mutex_unlock(&ctrl->client.mutex);
    }

    if (caps_valid)
    {
        gdouble span = az_max - az_min;
        if (az_min > az_max)
            span += 360.0;
        caps_extended = (span > 360.0 + eps);

        if (el_min > el_max)
        {
            gdouble tmp = el_min;
            el_min = el_max;
            el_max = tmp;
        }
        el = CLAMP(el, el_min, el_max);
        az = rotctrl_normalize_az_to_limits(az, az_min, az_max);
        az = rotctrl_normalize_backend_az(az, az_min, az_max);
    }
    else
    {
        el = CLAMP(el, el_min, el_max);
        az = rotctrl_normalize_backend_az(az, 0.0, 360.0);
    }

    el = rotctrl_apply_el_overtravel(ctrl, el);

    if (!(caps_extended && ctrl != NULL && !ctrl->tracking))
        az = rotctrl_clamp_user_az_interval(ctrl, az);

    if (az_out)
        *az_out = az;
    if (el_out)
        *el_out = el;

    if (cmd_out != NULL && cmd_len > 0)
    {
        g_ascii_formatd(azbuf, sizeof(azbuf), "%.2f", az);
        g_ascii_formatd(elbuf, sizeof(elbuf), "%.2f", el);
        g_snprintf(cmd_out, cmd_len, "P %s %s\n", azbuf, elbuf);
    }
}

/**
 * Send new position to rotator device
 *
 * \param ctrl Pointer to the GtkRotCtrl widget
 * \param az The new Azimuth
 * \param el The new Elevation
 * \return TRUE if the new position has been sent successfully
 *         FALSE if an error occurred
 * 
 * \note The function does not perform any range check since the GtkRotKnob
 * should always keep its value within range.
 */
static rot_set_result_t rotctrl_send_position(GtkRotCtrl *ctrl,
                                              gdouble az,
                                              gdouble el,
                                              gint *rprt_out,
                                              gboolean use_setpos,
                                              gboolean apply_calib)
{
    gchar           txbuf[64];
    gchar           buffback[128];
    gchar           send_az_str[32];
    gchar           send_el_str[32];
    gdouble         az_send = az;
    gdouble         el_send = el;
    gdouble         az_mech = 0.0;
    gdouble         el_mech = 0.0;
    gint            rprt_code = 0;
    HamlibResponseInfo info = { 0 };
    gboolean ok = FALSE;
    const gchar *cmd_name = use_setpos ? "set_position" : "move";

    if (use_setpos && ctrl != NULL && ctrl->tracking_active)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    "%s: %s blocked during tracking",
                    __func__, cmd_name);
        rot_term_log(ctrl, "gpredict:err",
                     "%s blocked during tracking", cmd_name);
        if (rprt_out)
            *rprt_out = 0;
        return ROT_SET_REJECTED;
    }

    if (apply_calib)
        rotctrl_calib_apply_send(ctrl, az_send, el_send, &az_mech, &el_mech);
    else
    {
        az_mech = az_send;
        el_mech = el_send;
    }

    /* send command (ASCII-safe, locale independent) */
    format_rotctld_setpos(ctrl, az_mech, el_mech,
                          &az_send, &el_send,
                          txbuf, sizeof(txbuf));
    {
        gchar txline[sizeof(txbuf)];

        g_strlcpy(txline, txbuf, sizeof(txline));
        g_strchomp(txline);
        rot_term_log(ctrl, "gpredict:tx", "rotctld cmd: %s", txline);
    }

    rot_format_deg_2(send_az_str, sizeof(send_az_str), az_send);
    rot_format_deg_2(send_el_str, sizeof(send_el_str), el_send);
    rot_term_log_verbose(ctrl, "gpredict:tx",
                         "%s send=(%s, %s)",
                         cmd_name, send_az_str, send_el_str);
    
    ok = rotctld_client_set_position_checked(ctrl->client.client,
                                             az_send,
                                             el_send,
                                             &rprt_code,
                                             &info,
                                             buffback,
                                             sizeof(buffback));

    rot_term_log_verbose(ctrl, "gpredict:rx",
                         "%s reply=%s", cmd_name, buffback);

    /* Interpret reply:
     *  - RPRT 0                → success.
     *  - RPRT -5/-6            → backend I/O issue (rotctld stays up).
     *  - Other RPRT non-zero   → rejected.
     *  - Otherwise             → treat as rejected.
     */
    if (rprt_out)
        *rprt_out = 0;

    if (rprt_out)
        *rprt_out = rprt_code;

    if (ok)
        return ROT_SET_OK;

    if (info.used_multiline)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    "%s: rotctld protocol error: multiline reply to %s",
                    __func__, cmd_name);
        g_mutex_lock(&ctrl->client.mutex);
        g_strlcpy(ctrl->client.io_error_reason, "protocol error",
                  sizeof(ctrl->client.io_error_reason));
        g_mutex_unlock(&ctrl->client.mutex);
        return ROT_SET_IO_ERROR;
    }

    if (rprt_code == -5 || rprt_code == -6 || rprt_code == -8)
        return ROT_SET_BACKEND_IO;
    if (rprt_code != 0)
        return ROT_SET_REJECTED;

    if (rotctld_reply_is_dump_state(buffback))
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    "%s: rotctld desync: got dump_state while expecting RPRT",
                    __func__);
        rot_term_log(ctrl, "gpredict:err",
                     "rotctld desync: got dump_state while expecting RPRT");
        g_mutex_lock(&ctrl->client.mutex);
        g_strlcpy(ctrl->client.io_error_reason, "desync",
                  sizeof(ctrl->client.io_error_reason));
        g_mutex_unlock(&ctrl->client.mutex);
        return ROT_SET_IO_ERROR;
    }

    if (buffback[0] != '\0')
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    "%s: rotctld protocol error: expected RPRT, got '%s'",
                    __func__, buffback);
        g_mutex_lock(&ctrl->client.mutex);
        g_strlcpy(ctrl->client.io_error_reason, "protocol error",
                  sizeof(ctrl->client.io_error_reason));
        g_mutex_unlock(&ctrl->client.mutex);
        return ROT_SET_IO_ERROR;
    }

    if (!ok)
        return ROT_SET_IO_ERROR;

    return ROT_SET_IO_ERROR;
}

static gboolean rotctrl_set_position_guarded(GtkRotCtrl *ctrl,
                                             gdouble az,
                                             gdouble el,
                                             const gchar *context)
{
    const gdouble eps = 1e-6;
    gdouble send_az = az;
    gdouble send_el = el;
    gchar txbuf[64];
    gboolean caps_valid = FALSE;
    gboolean caps_extended = FALSE;
    gdouble az_min = 0.0;
    gdouble az_max = 0.0;
    gdouble el_min = ROTCTRL_DEFAULT_MIN_EL;
    gdouble el_max = ROTCTRL_DEFAULT_MAX_EL;

    if (ctrl == NULL || ctrl->client.client == NULL)
        return FALSE;

    if (ctrl->conf)
    {
        el_min = ctrl->conf->minel;
        el_max = ctrl->conf->maxel;
    }

    if (ctrl->tracking_active)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    "%s: set_position blocked during tracking (%s)",
                    __func__, context ? context : "unknown");
        rot_term_log(ctrl, "gpredict:err",
                     "set_position blocked during tracking (%s)",
                     context ? context : "unknown");
        return FALSE;
    }

    g_mutex_lock(&ctrl->client.mutex);
    caps_valid = ctrl->client.limits_valid;
    if (caps_valid)
    {
        az_min = ctrl->client.az_min;
        az_max = ctrl->client.az_max;
        el_min = ctrl->client.el_min;
        el_max = ctrl->client.el_max;
    }
    g_mutex_unlock(&ctrl->client.mutex);

    if (caps_valid)
    {
        gdouble span = az_max - az_min;
        if (az_min > az_max)
            span += 360.0;
        caps_extended = (span > 360.0 + eps);

        if (el_min > el_max)
        {
            gdouble tmp = el_min;
            el_min = el_max;
            el_max = tmp;
        }
        send_el = CLAMP(send_el, el_min, el_max);
        send_az = rotctrl_normalize_az_to_limits(send_az, az_min, az_max);
        send_az = rotctrl_normalize_backend_az(send_az, az_min, az_max);
    }
    else
    {
        send_el = CLAMP(send_el, el_min, el_max);
        send_az = rotctrl_normalize_backend_az(send_az, 0.0, 360.0);
    }

    if (!(caps_extended && ctrl != NULL && !ctrl->tracking))
        send_az = rotctrl_clamp_user_az_interval(ctrl, send_az);

    format_rotctld_setpos(ctrl, send_az, send_el,
                          &send_az, &send_el,
                          txbuf, sizeof(txbuf));
    {
        gchar txline[sizeof(txbuf)];

        g_strlcpy(txline, txbuf, sizeof(txline));
        g_strchomp(txline);
        rot_term_log(ctrl, "gpredict:tx", "rotctld cmd: %s", txline);
    }

    return rotctld_client_set_pos(ctrl->client.client, send_az, send_el);
}

/* Rotctl client thread */
static gpointer rotctld_client_thread(gpointer data)
{
    gdouble         elapsed_time;
    gdouble         azi = 0.0;
    gdouble         ele = 0.0;
    gboolean        io_error = FALSE;
    gdouble         backoff_sec = 0.5;
    const gdouble   backoff_max = 10.0;
    GtkRotCtrl     *ctrl = GTK_ROT_CTRL(data);
    RotLogRate      getpos_err_rate = { 0 };
    RotLogRate      setpos_err_rate = { 0 };
    RotLogRate      pos_skip_rate = { 0 };
    RotLogRate      pos_backoff_rate = { 0 };
    RotLogRate      transport_err_rate = { 0 };
    RotLogRate      reconnect_rate = { 0 };
    guint           reconnect_attempts = 0;
    guint64         session_gen = 0;

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                _("%s: rotctld_client_thread started"), __func__);

#if defined(__APPLE__) || defined(__linux__) || defined(__FreeBSD__)
    locale_t c_locale = newlocale(LC_NUMERIC_MASK, "C", (locale_t)0);
    locale_t old_locale = (locale_t)0;
    if (c_locale)
        old_locale = uselocale(c_locale);
#endif

    ctrl->client.timer = g_timer_new();
    ctrl->client.new_trg = FALSE;
    ctrl->client.use_setpos = FALSE;
    ctrl->client.apply_calib = FALSE;
    ctrl->client.allow_send_no_pos = FALSE;
    ctrl->client.stop_pending = FALSE;
    ctrl->client.running = TRUE;
    g_atomic_int_set(&ctrl->client.stop_requested, 0);
    g_mutex_lock(&ctrl->client.mutex);
    ctrl->client.send_quit = FALSE;
    ctrl->client.thread_done = FALSE;
    g_mutex_unlock(&ctrl->client.mutex);
    ctrl->client.socket = -1;
    ctrl->client.conn_id = 0;
    ctrl->client.last_cmd_us = 0;
    ctrl->client.last_cmd_ok_az = 0.0;
    ctrl->client.last_cmd_ok_el = 0.0;
    ctrl->client.last_cmd_ok_us = 0;
    ctrl->client.last_cmd_backend_az = 0.0;
    ctrl->client.last_cmd_backend_el = 0.0;
    ctrl->client.last_cmd_backend_valid = FALSE;
    ctrl->client.desired_user_az = 0.0;
    ctrl->client.desired_user_el = 0.0;
    ctrl->client.desired_backend_az = 0.0;
    ctrl->client.desired_backend_el = 0.0;
    ctrl->client.desired_raw_az = 0.0;
    ctrl->client.desired_raw_el = 0.0;
    ctrl->client.desired_valid = FALSE;
    ctrl->client.desired_allow = FALSE;
    ctrl->client.desired_tracking = FALSE;
    ctrl->client.force_pending = FALSE;
    ctrl->client.desired_update_us = 0;
    ctrl->client.last_err_deg = 0.0;
    ctrl->client.last_err_us = 0;
    ctrl->client.stall_since_us = 0;
    ctrl->client.last_pos_user_az = 0.0;
    ctrl->client.last_pos_user_el = 0.0;
    ctrl->client.last_pos_sample_us = 0;
    ctrl->client.last_err_to_cmd = 0.0;
    ctrl->client.last_err_time_us = 0;
    ctrl->client.move_grace_until_us = 0;
    ctrl->client.discrepancy_streak = 0;
    ctrl->client.wrong_way_count = 0;
    ctrl->client.discrepancy_pending = FALSE;
    ctrl->client.feedback_disabled_logged = FALSE;
    if (ctrl->client.client == NULL)
        ctrl->client.client = rotctld_client_new("rotctld");
    ctrl->client.cmd_rejected = FALSE;
    ctrl->client.reject_backoff_until_us = 0;
    ctrl->client.reject_backoff_sec = 0.5;
    ctrl->client.reject_backoff_log_us = 0;
    ctrl->client.limits_valid = FALSE;
    ctrl->client.south_zero = FALSE;
    g_atomic_int_set(&ctrl->south_zero_cached, 0);
    ctrl->client.daemon_ok = FALSE;
    ctrl->client.pos_valid = FALSE;
    ctrl->client.pos_invalid = FALSE;
    ctrl->client.pos_unknown = FALSE;
    ctrl->client.pos_cmd_ok = FALSE;
    ctrl->client.set_pos_ok = FALSE;
    ctrl->client.azi_mech_in = 0.0;
    ctrl->client.ele_mech_in = 0.0;
    ctrl->client.last_pos_us = 0;
    ctrl->client.last_pos_attempt_us = 0;
    ctrl->client.last_set_attempt_us = 0;
    ctrl->client.handshake_pos_ok = FALSE;
    ctrl->client.first_pos_deadline_us = 0;
    ctrl->client.transport_backoff_until_us = 0;
    ctrl->client.transport_backoff_sec = 0.5;
    ctrl->client.io_error_reason[0] = '\0';
    ctrl->client.last_pos_error[0] = '\0';
    ctrl->client.reconnect_failures = 0;
    ctrl->client.reconnect_degraded = FALSE;
    ctrl->client.pos_failures = 0;
    ctrl->client.pos_backoff_until_us = 0;
    ctrl->client.pos_backoff_sec = 0.5;
    ctrl->client.pos_degraded = FALSE;

    session_gen = rotctld_get_engage_generation(ctrl);
    g_mutex_lock(&ctrl->client.mutex);
    ctrl->client.thread_generation = session_gen;
    g_mutex_unlock(&ctrl->client.mutex);

    rot_term_log_verbose(ctrl, "gpredict:rx",
                         "rotctld client thread started for %s:%d",
                         ctrl->conf ? ctrl->conf->host : "(null)",
                         rotctld_effective_port(ctrl));
    if (ctrl->verbose_logging)
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "rotctld client thread started for %s:%d",
                    ctrl->conf ? ctrl->conf->host : "(null)",
                    rotctld_effective_port(ctrl));

    while (ctrl->client.running && !rotctld_stop_requested(ctrl))
    {
        if (rotctld_generation_stale(ctrl, session_gen))
            goto out_stop;
        g_timer_start(ctrl->client.timer);
        gboolean connected_now = FALSE;

        if (ctrl->client.socket == -1) {
            if (ctrl->client.transport_backoff_until_us > 0)
            {
                gint64 now_us = g_get_monotonic_time();
                if (now_us < ctrl->client.transport_backoff_until_us)
                {
                    rotctld_sleep_us(ctrl,
                                     ctrl->client.transport_backoff_until_us - now_us);
                    continue;
                }
                ctrl->client.transport_backoff_until_us = 0;
            }
            if (rotctld_stop_requested(ctrl) ||
                rotctld_generation_stale(ctrl, session_gen))
                goto out_stop;

            ctrl->client.socket = rotctld_socket_open(ctrl,
                                                      ctrl->conf->host,
                                                      rotctld_effective_port(ctrl));
            if (rotctld_stop_requested(ctrl) ||
                rotctld_generation_stale(ctrl, session_gen))
                goto out_stop;
            if (ctrl->client.socket == -1) {
                if (rotctld_stop_requested(ctrl) ||
                    rotctld_generation_stale(ctrl, session_gen))
                    goto out_stop;
                io_error = TRUE;
                reconnect_attempts++;
                g_mutex_lock(&ctrl->client.mutex);
                ctrl->client.io_error = TRUE;
                ctrl->client.daemon_ok = FALSE;
                ctrl->client.reconnect_failures = reconnect_attempts;
                ctrl->client.reconnect_degraded =
                    (reconnect_attempts >= ROTCTLD_RECONNECT_MAX_RETRIES);
                g_strlcpy(ctrl->client.io_error_reason, "connect failed",
                          sizeof(ctrl->client.io_error_reason));
                g_mutex_unlock(&ctrl->client.mutex);
                if (reconnect_attempts == ROTCTLD_RECONNECT_MAX_RETRIES)
                {
                    sat_log_log(SAT_LOG_LEVEL_WARN,
                                "%s: rotctld retry limit reached; marking DEGRADED",
                                __func__);
                    rot_term_log(ctrl, "gpredict:err",
                                 "rotctld retry limit reached; marking DEGRADED");
                }
                rot_log_rate_limited(ctrl, &reconnect_rate,
                                     ROTCTLD_FAILURE_LOG_INTERVAL_US,
                                     SAT_LOG_LEVEL_WARN, "gpredict:err",
                                     "rotctld connect failed (attempt %u/%u); retrying in %.1fs",
                                     reconnect_attempts,
                                     ROTCTLD_RECONNECT_MAX_RETRIES,
                                     backoff_sec);
                rot_term_log(ctrl, "gpredict:err",
                             "rotctld connect failed to %s:%d (attempt %u/%u); retrying in %.1fs",
                             ctrl->conf ? ctrl->conf->host : "(null)",
                             rotctld_effective_port(ctrl),
                             reconnect_attempts,
                             ROTCTLD_RECONNECT_MAX_RETRIES,
                             backoff_sec);
                if (rotctld_stop_requested(ctrl) ||
                    rotctld_generation_stale(ctrl, session_gen))
                    goto out_stop;
                rotctld_sleep_us(ctrl, (gint64)(backoff_sec * 1e6));
                backoff_sec = MIN(backoff_sec * 2.0, backoff_max);
                continue;
            }

            connected_now = TRUE;
        }

        if (connected_now)
        {
            backoff_sec = 0.5;
            io_error = FALSE;
            g_mutex_lock(&ctrl->client.mutex);
            ctrl->client.io_error = FALSE;
            ctrl->client.daemon_ok = FALSE;
            ctrl->client.reject_backoff_until_us = 0;
            ctrl->client.reject_backoff_sec = 0.5;
            ctrl->client.reject_backoff_log_us = 0;
            ctrl->client.consecutive_failures = 0;
            ctrl->client.last_failure_log_us = 0;
            ctrl->client.pos_valid = FALSE;
            ctrl->client.pos_invalid = FALSE;
            ctrl->client.pos_unknown = FALSE;
            ctrl->client.pos_cmd_ok = FALSE;
            ctrl->client.set_pos_ok = FALSE;
            ctrl->client.handshake_pos_ok = FALSE;
            ctrl->client.first_pos_deadline_us = 0;
            ctrl->client.last_pos_us = 0;
            ctrl->client.last_pos_attempt_us = 0;
            ctrl->client.last_set_attempt_us = 0;
            ctrl->client.transport_backoff_until_us = 0;
            ctrl->client.transport_backoff_sec = 0.5;
            ctrl->client.io_error_reason[0] = '\0';
            ctrl->client.last_pos_error[0] = '\0';
            g_mutex_unlock(&ctrl->client.mutex);
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        "%s: rotctld link re-established", __func__);
            rot_term_log_verbose(ctrl, "gpredict:rx",
                                 "rotctld link re-established");
            memset(&getpos_err_rate, 0, sizeof(getpos_err_rate));
            memset(&setpos_err_rate, 0, sizeof(setpos_err_rate));
            memset(&pos_skip_rate, 0, sizeof(pos_skip_rate));
            memset(&pos_backoff_rate, 0, sizeof(pos_backoff_rate));
            memset(&transport_err_rate, 0, sizeof(transport_err_rate));

            g_mutex_lock(&ctrl->client.mutex);
            ctrl->client.limits_valid = FALSE;
            ctrl->client.south_zero = FALSE;
            g_atomic_int_set(&ctrl->south_zero_cached, 0);
            ctrl->client.daemon_ok = FALSE;
            ctrl->client.pos_valid = FALSE;
            ctrl->client.pos_invalid = FALSE;
            ctrl->client.pos_unknown = FALSE;
            ctrl->client.pos_cmd_ok = FALSE;
            ctrl->client.set_pos_ok = FALSE;
            ctrl->client.handshake_pos_ok = FALSE;
            ctrl->client.first_pos_deadline_us = 0;
            ctrl->client.last_pos_us = 0;
            ctrl->client.last_pos_attempt_us = 0;
            ctrl->client.last_set_attempt_us = 0;
            ctrl->client.last_pos_error[0] = '\0';
            g_mutex_unlock(&ctrl->client.mutex);

            {
                gchar dump_state[4096];
                gchar pos_reply[256];
                gdouble hs_az = 0.0;
                gdouble hs_el = 0.0;
                gboolean handshake_ok = FALSE;
                gboolean pos_ok = FALSE;
                rot_daemon_type_t daemon = ROT_DAEMON_UNKNOWN;

                if (rotctld_stop_requested(ctrl) ||
                    rotctld_generation_stale(ctrl, session_gen))
                    goto out_stop;
                sat_log_log(SAT_LOG_LEVEL_INFO,
                            "%s: rotctld handshake start conn_id=%" G_GUINT64_FORMAT " host=%s port=%d",
                            __func__,
                            ctrl->client.conn_id,
                            ctrl->conf ? ctrl->conf->host : "(null)",
                            rotctld_effective_port(ctrl));
                dump_state[0] = '\0';
            for (gint attempt = 0; attempt < 2; attempt++)
            {
                if (attempt > 0)
                    rotctld_sleep_us(ctrl, (gint64)ROTCTLD_HANDSHAKE_RETRY_MS * 1000);
                if (rotctld_stop_requested(ctrl) ||
                    rotctld_generation_stale(ctrl, session_gen))
                    goto out_stop;
                dump_state[0] = '\0';
                pos_reply[0] = '\0';
                gboolean hs_ok = rotctld_client_handshake(ctrl->client.client,
                                                          ROTCTLD_SOCKET_TIMEOUT_MS,
                                                          &hs_az, &hs_el,
                                                          dump_state,
                                                          sizeof(dump_state),
                                                          pos_reply,
                                                          sizeof(pos_reply),
                                                          &pos_ok);
                if (rotctld_stop_requested(ctrl) ||
                    rotctld_generation_stale(ctrl, session_gen))
                    goto out_stop;
                if (ctrl->verbose_logging && dump_state[0] != '\0')
                    rot_term_log_verbose(ctrl, "gpredict:rx",
                                         "rotctld handshake dump_state:\n%s",
                                         dump_state);
                if (ctrl->verbose_logging && pos_reply[0] != '\0')
                        rot_term_log_verbose(ctrl, "gpredict:rx",
                                             "rotctld handshake p reply:\n%s",
                                             pos_reply);
                    if (hs_ok)
                    {
                        handshake_ok = TRUE;
                        break;
                    }
                }

                if (dump_state[0] != '\0')
                    daemon = rotctld_detect_daemon(dump_state);

                if (rotctld_stop_requested(ctrl) ||
                    rotctld_generation_stale(ctrl, session_gen))
                {
                    ctrl->client.running = FALSE;
                    break;
                }

                if (!handshake_ok)
                {
                    /* Reconnect once, then retry handshake. */
                    rotctld_socket_close_quiet(ctrl, &ctrl->client.socket);
                    rotctld_clear_rxbuf(ctrl);
                    ctrl->client.socket = rotctld_socket_open(ctrl,
                                                              ctrl->conf->host,
                                                              rotctld_effective_port(ctrl));
                    if (rotctld_stop_requested(ctrl) ||
                        rotctld_generation_stale(ctrl, session_gen))
                        goto out_stop;
                    if (ctrl->client.socket != -1)
                    {
                        dump_state[0] = '\0';
                        for (gint attempt = 0; attempt < 2; attempt++)
                        {
                            if (attempt > 0)
                                rotctld_sleep_us(ctrl, (gint64)ROTCTLD_HANDSHAKE_RETRY_MS * 1000);
                            if (rotctld_stop_requested(ctrl) ||
                                rotctld_generation_stale(ctrl, session_gen))
                                goto out_stop;
                            dump_state[0] = '\0';
                            pos_reply[0] = '\0';
                            gboolean hs_ok = rotctld_client_handshake(ctrl->client.client,
                                                                      ROTCTLD_SOCKET_TIMEOUT_MS,
                                                                      &hs_az, &hs_el,
                                                                      dump_state,
                                                                      sizeof(dump_state),
                                                                      pos_reply,
                                                                      sizeof(pos_reply),
                                                                      &pos_ok);
                            if (rotctld_stop_requested(ctrl) ||
                                rotctld_generation_stale(ctrl, session_gen))
                                goto out_stop;
                            if (ctrl->verbose_logging && dump_state[0] != '\0')
                                rot_term_log_verbose(ctrl, "gpredict:rx",
                                                     "rotctld handshake dump_state:\n%s",
                                                     dump_state);
                            if (ctrl->verbose_logging && pos_reply[0] != '\0')
                                rot_term_log_verbose(ctrl, "gpredict:rx",
                                                     "rotctld handshake p reply:\n%s",
                                                     pos_reply);
                            if (hs_ok)
                            {
                                handshake_ok = TRUE;
                                break;
                            }
                        }
                        if (dump_state[0] != '\0')
                            daemon = rotctld_detect_daemon(dump_state);
                    }
                }

                if (daemon != ROT_DAEMON_ROTCTLD &&
                    daemon != ROT_DAEMON_UNKNOWN)
                {
                    sat_log_log(SAT_LOG_LEVEL_ERROR,
                                "%s: non-rotctld daemon on %s:%d",
                                __func__,
                                ctrl->conf ? ctrl->conf->host : "(null)",
                                rotctld_effective_port(ctrl));
                    rot_term_log(ctrl, "gpredict:err",
                                 "Port %d already in use by non-rotctld service (host=%s)",
                                 rotctld_effective_port(ctrl),
                                 ctrl->conf ? ctrl->conf->host : "(null)");
                    rotctld_socket_close_quiet(ctrl, &ctrl->client.socket);
                    rotctld_clear_rxbuf(ctrl);
                    g_mutex_lock(&ctrl->client.mutex);
                    ctrl->client.daemon_ok = FALSE;
                    ctrl->client.io_error = TRUE;
                    g_mutex_unlock(&ctrl->client.mutex);
                    rot_schedule_wrong_daemon(ctrl,
                                              ctrl->conf ? ctrl->conf->host : NULL,
                                              rotctld_effective_port(ctrl));
                    ctrl->client.running = FALSE;
                    break;
                }

                if (!handshake_ok)
                {
                    if (rotctld_stop_requested(ctrl) ||
                        rotctld_generation_stale(ctrl, session_gen))
                        goto out_stop;
                    if (rotctld_io_recent(ctrl,
                                          (gint64)ROTCTLD_HANDSHAKE_GRACE_MS * 1000))
                    {
                        g_mutex_lock(&ctrl->client.mutex);
                        hs_az = ctrl->client.azi_mech_in;
                        hs_el = ctrl->client.ele_mech_in;
                        g_mutex_unlock(&ctrl->client.mutex);
                        handshake_ok = TRUE;
                        sat_log_log(SAT_LOG_LEVEL_WARN,
                                    "%s: rotctld handshake failed but recent I/O ok; keeping session",
                                    __func__);
                    }
                    else
                    {
                        (void)rotctld_note_io_failure(ctrl, "handshake");
                        sat_log_log(SAT_LOG_LEVEL_ERROR,
                                    "%s: rotctld handshake failed on %s:%d; retrying",
                                    __func__,
                                    ctrl->conf ? ctrl->conf->host : "(null)",
                                    rotctld_effective_port(ctrl));
                        rot_term_log(ctrl, "gpredict:err",
                                     "rotctld handshake failed on %s:%d; retrying",
                                     ctrl->conf ? ctrl->conf->host : "(null)",
                                     rotctld_effective_port(ctrl));
                        reconnect_attempts++;
                        rot_log_rate_limited(ctrl, &reconnect_rate,
                                             ROTCTLD_FAILURE_LOG_INTERVAL_US,
                                             SAT_LOG_LEVEL_WARN, "gpredict:err",
                                             "rotctld handshake failed (attempt %u/%u); backoff %.1fs",
                                             reconnect_attempts,
                                             ROTCTLD_RECONNECT_MAX_RETRIES,
                                             backoff_sec);
                        rotctld_socket_close_quiet(ctrl, &ctrl->client.socket);
                        rotctld_clear_rxbuf(ctrl);
                        g_mutex_lock(&ctrl->client.mutex);
                        ctrl->client.daemon_ok = FALSE;
                        ctrl->client.io_error = TRUE;
                        ctrl->client.reconnect_failures = reconnect_attempts;
                        ctrl->client.reconnect_degraded =
                            (reconnect_attempts >= ROTCTLD_RECONNECT_MAX_RETRIES);
                        g_strlcpy(ctrl->client.io_error_reason, "handshake failed",
                                  sizeof(ctrl->client.io_error_reason));
                        g_mutex_unlock(&ctrl->client.mutex);
                        if (reconnect_attempts == ROTCTLD_RECONNECT_MAX_RETRIES)
                        {
                            sat_log_log(SAT_LOG_LEVEL_WARN,
                                        "%s: rotctld retry limit reached; marking DEGRADED",
                                        __func__);
                            rot_term_log(ctrl, "gpredict:err",
                                         "rotctld retry limit reached; marking DEGRADED");
                        }
                        if (rotctld_stop_requested(ctrl) ||
                            rotctld_generation_stale(ctrl, session_gen))
                            goto out_stop;
                        rotctld_sleep_us(ctrl, (gint64)(backoff_sec * 1e6));
                        backoff_sec = MIN(backoff_sec * 2.0, backoff_max);
                        continue;
                    }
                }

                if (handshake_ok)
                    sat_log_log(SAT_LOG_LEVEL_INFO,
                                "%s: rotctld handshake complete (daemon ok) conn_id=%" G_GUINT64_FORMAT " pos_ok=%d",
                                __func__,
                                ctrl->client.conn_id,
                                pos_ok ? 1 : 0);

                if (ctrl != NULL)
                {
                    gint hs_rprt = 0;
                    gboolean hs_rprt_found =
                        rot_parse_rprt_code_any(pos_reply, &hs_rprt);
                    gboolean hs_pos_invalid =
                        (!pos_ok && hs_rprt_found && hs_rprt == -6);

                    g_mutex_lock(&ctrl->client.mutex);
                    ctrl->client.handshake_pos_ok = pos_ok;
                    ctrl->client.pos_invalid = hs_pos_invalid ? TRUE : FALSE;
                    if (pos_ok)
                        ctrl->client.first_pos_deadline_us = 0;
                    else
                        ctrl->client.first_pos_deadline_us =
                            g_get_monotonic_time() +
                            ((gint64)ROTCTLD_FIRST_POS_GRACE_MS * 1000);
                    if (hs_pos_invalid)
                        g_strlcpy(ctrl->client.last_pos_error, "rprt -6",
                                  sizeof(ctrl->client.last_pos_error));
                    g_mutex_unlock(&ctrl->client.mutex);

                    if (hs_pos_invalid)
                    {
                        sat_log_log(SAT_LOG_LEVEL_WARN,
                                    "%s: pos_invalid during handshake (RPRT -6)",
                                    __func__);
                        rot_term_log(ctrl, "gpredict:err",
                                     "pos_invalid during handshake (RPRT -6)");
                    }

                    if (!pos_ok)
                        sat_log_log(SAT_LOG_LEVEL_INFO,
                                    "%s: waiting for first position (grace %.1fs)",
                                    __func__,
                                    ROTCTLD_FIRST_POS_GRACE_MS / 1000.0);

                    if (pos_ok)
                    {
                        RotTransformSnapshotState snap_state;
                        const RotTransformSnapshot *snap = NULL;
                        AzSpan span_mode;
                        gdouble span_min = 0.0;
                        gdouble span_max = 0.0;
                        gdouble raw_az = hs_az;
                        gdouble mapped_az = 0.0;
                        gdouble mapped_el = hs_el;
                        gboolean el_clamped = FALSE;
                        gboolean needs_correction = FALSE;

                        rot_transform_snapshot_state_init(&snap_state);
                        (void)rot_transform_snapshot_state_get(ctrl, &snap_state);
                        snap = &snap_state.transform;
                        span_mode = rotctrl_span_from_az_type(snap->az_norm_mode);
                        rotctrl_span_bounds(span_mode, &span_min, &span_max);

                        mapped_az = az_norm_span(raw_az, span_mode);

                        if (ctrl->conf &&
                            ctrl->conf->axis_mode == ROT_AXIS_MODE_AZ_ONLY)
                        {
                            mapped_el = snap->el_min_deg;
                        }
                        else
                        {
                            if (mapped_el < snap->el_min_deg)
                            {
                                mapped_el = snap->el_min_deg;
                                el_clamped = TRUE;
                            }
                            else if (mapped_el > snap->el_max_deg)
                            {
                                mapped_el = snap->el_max_deg;
                                el_clamped = TRUE;
                            }
                        }
                        if (raw_az < (span_min - 1e-3) ||
                            raw_az > (span_max + 1e-3))
                            needs_correction = TRUE;
                        if (fabs(mapped_az - raw_az) > 1e-3 || el_clamped)
                            needs_correction = TRUE;

                        if (needs_correction)
                        {
                            sat_log_log(SAT_LOG_LEVEL_WARN,
                                        "%s: rotor initial position outside selected span: "
                                        "raw az=%.2f => mapped az=%.2f (span=%.0f..%.0f); "
                                        "skipping correction during handshake",
                                        __func__, raw_az, mapped_az,
                                        span_min, span_max);
                        }

                        hs_az = az_norm_span(raw_az, span_mode);
                        hs_el = mapped_el;

                        gdouble mech_az = hs_az;
                        gdouble mech_el = hs_el;
                        gdouble user_az = 0.0;
                        gdouble user_el = 0.0;

                        rotctrl_map_mech_to_user(ctrl,
                                                 mech_az,
                                                 mech_el,
                                                 &user_az,
                                                 &user_el);

                        g_mutex_lock(&ctrl->client.mutex);
                        ctrl->az_hold_active = FALSE;
                        if (snap->zenith_guard_enable &&
                            user_el >= snap->zenith_guard_el_deg)
                        {
                            ctrl->az_hold_active = TRUE;
                            ctrl->az_hold_value = user_az;
                        }
                        ctrl->client.azi_mech_in = mech_az;
                        ctrl->client.ele_mech_in = mech_el;
                        ctrl->client.azi_in = user_az;
                        ctrl->client.ele_in = user_el;
                        ctrl->client.azi_out = mech_az;
                        ctrl->client.ele_out = mech_el;
                        ctrl->client.raw_azi_out = mech_az;
                        ctrl->client.raw_ele_out = mech_el;
                        ctrl->client.pos_valid = TRUE;
                        ctrl->client.pos_invalid = FALSE;
                        ctrl->client.pos_unknown = FALSE;
                        ctrl->client.pos_cmd_ok = TRUE;
                        ctrl->client.last_pos_us = g_get_monotonic_time();
                        ctrl->client.last_pos_error[0] = '\0';
                        g_mutex_unlock(&ctrl->client.mutex);
                        rotctld_note_io_ok(ctrl);
                    }
                    else
                    {
                        g_mutex_lock(&ctrl->client.mutex);
                        ctrl->client.pos_valid = FALSE;
                        ctrl->client.pos_unknown = TRUE;
                        ctrl->client.pos_cmd_ok = FALSE;
                        ctrl->client.set_pos_ok = FALSE;
                        ctrl->client.last_pos_us = 0;
                        ctrl->client.last_pos_attempt_us = 0;
                        g_strlcpy(ctrl->client.last_pos_error, "no position",
                                  sizeof(ctrl->client.last_pos_error));
                        g_mutex_unlock(&ctrl->client.mutex);
                    }
                }

                {
                    const RotCaps *caps = NULL;
                    gdouble cur_az = 0.0;
                    gdouble cur_el = 0.0;
                    g_mutex_lock(&ctrl->client.mutex);
                    ctrl->client.daemon_ok = TRUE;
                    ctrl->client.reconnect_failures = 0;
                    ctrl->client.reconnect_degraded = FALSE;
                    ctrl->client.pos_failures = 0;
                    ctrl->client.pos_backoff_until_us = 0;
                    ctrl->client.pos_backoff_sec = 0.5;
                    ctrl->client.pos_degraded = FALSE;
                    caps = rotctld_client_get_caps(ctrl->client.client);
                    if (caps != NULL)
                    {
                        ctrl->client.limits_valid = caps->limits_valid;
                        ctrl->client.az_min = caps->az_min;
                        ctrl->client.az_max = caps->az_max;
                        ctrl->client.el_min = caps->el_min;
                        ctrl->client.el_max = caps->el_max;
                        ctrl->client.south_zero = caps->south_zero;
                        g_atomic_int_set(&ctrl->south_zero_cached,
                                         caps->south_zero ? 1 : 0);
                    }
                    else
                    {
                        ctrl->client.limits_valid = FALSE;
                        ctrl->client.south_zero = FALSE;
                        g_atomic_int_set(&ctrl->south_zero_cached, 0);
                    }
                    g_mutex_unlock(&ctrl->client.mutex);
                    reconnect_attempts = 0;

                    if (pos_ok)
                    {
                        for (gint attempt = 0; attempt < 2; attempt++)
                        {
                            if (attempt == 0)
                            {
                                cur_az = hs_az;
                                cur_el = hs_el;
                                gdouble mech_az = cur_az;
                                gdouble mech_el = cur_el;
                                gdouble user_az = 0.0;
                                gdouble user_el = 0.0;

                                rotctrl_map_mech_to_user(ctrl,
                                                         mech_az,
                                                         mech_el,
                                                         &user_az,
                                                         &user_el);
                                g_mutex_lock(&ctrl->client.mutex);
                                ctrl->client.azi_mech_in = mech_az;
                                ctrl->client.ele_mech_in = mech_el;
                                ctrl->client.azi_in = user_az;
                                ctrl->client.ele_in = user_el;
                                ctrl->client.azi_out = mech_az;
                                ctrl->client.ele_out = mech_el;
                                ctrl->client.raw_azi_out = mech_az;
                                ctrl->client.raw_ele_out = mech_el;
                                ctrl->client.pos_valid = TRUE;
                                ctrl->client.pos_unknown = FALSE;
                                ctrl->client.pos_cmd_ok = TRUE;
                                ctrl->client.last_pos_us = g_get_monotonic_time();
                                ctrl->client.last_pos_error[0] = '\0';
                                g_mutex_unlock(&ctrl->client.mutex);
                                rotctld_note_io_ok(ctrl);
                                break;
                            }
                            if (rotctld_client_get_pos(ctrl->client.client,
                                                       &cur_az, &cur_el))
                            {
                                gdouble mech_az = cur_az;
                                gdouble mech_el = cur_el;
                                gdouble user_az = 0.0;
                                gdouble user_el = 0.0;

                                rotctrl_map_mech_to_user(ctrl,
                                                         mech_az,
                                                         mech_el,
                                                         &user_az,
                                                         &user_el);
                                g_mutex_lock(&ctrl->client.mutex);
                                ctrl->client.azi_mech_in = mech_az;
                                ctrl->client.ele_mech_in = mech_el;
                                ctrl->client.azi_in = user_az;
                                ctrl->client.ele_in = user_el;
                                ctrl->client.azi_out = mech_az;
                                ctrl->client.ele_out = mech_el;
                                ctrl->client.raw_azi_out = mech_az;
                                ctrl->client.raw_ele_out = mech_el;
                                ctrl->client.pos_valid = TRUE;
                                ctrl->client.pos_unknown = FALSE;
                                ctrl->client.pos_cmd_ok = TRUE;
                                ctrl->client.last_pos_us = g_get_monotonic_time();
                                ctrl->client.last_pos_error[0] = '\0';
                                g_mutex_unlock(&ctrl->client.mutex);
                                rotctld_note_io_ok(ctrl);
                                break;
                            }
                            rotctld_sleep_us(ctrl, (gint64)ROTCTLD_HANDSHAKE_RETRY_MS * 1000);
                        }
                    }
                }
            }
            io_error = FALSE;
        }

        /* get latest commanded position from controller, but only
         * send a new command when new_trg is set. This avoids
         * hammering the rotor with repeated small corrections and
         * reduces "hunting" around the target position.
         */
        gboolean send_cmd = FALSE;
        gboolean backoff_active = FALSE;
        gboolean have_trg = FALSE;
        gboolean allow_no_pos = FALSE;
        gboolean feedback_disabled = FALSE;
        gboolean use_setpos = FALSE;
        gboolean apply_calib = FALSE;
        gboolean stop_pending = FALSE;
        const gchar *cmd_name = "move";
        const gchar *send_reason = "none";
        gint64 now_us = g_get_monotonic_time();
        gint64 backoff_until = 0;
        gint64 backoff_log_us = 0;
        gdouble raw_azi = 0.0;
        gdouble raw_ele = 0.0;
        gdouble desired_user_az = 0.0;
        gdouble desired_user_el = 0.0;
        gdouble desired_backend_az = 0.0;
        gdouble desired_backend_el = 0.0;
        gboolean desired_allow = FALSE;
        gboolean desired_tracking = FALSE;
        gboolean desired_valid = FALSE;
        gboolean force_pending = FALSE;
        gboolean pos_valid = FALSE;
        gboolean pos_degraded = FALSE;
        gboolean session_degraded = FALSE;
        gboolean backend_ioerr_pending = FALSE;
        gboolean reconnect_degraded = FALSE;
        gboolean daemon_ok = FALSE;
        gint64 last_pos_us = 0;
        gdouble cur_user_az = 0.0;
        gdouble cur_user_el = 0.0;
        gdouble last_cmd_backend_az = 0.0;
        gdouble last_cmd_backend_el = 0.0;
        gboolean last_cmd_backend_valid = FALSE;
        gint64 last_cmd_ok_us = 0;
        gdouble last_cmd_ok_az = 0.0;
        gdouble last_cmd_ok_el = 0.0;
        gint64 last_set_attempt_us = 0;
        gboolean new_trg = FALSE;
        gdouble fallback_az = 0.0;
        gdouble fallback_el = 0.0;
        gdouble fallback_raw_az = 0.0;
        gdouble fallback_raw_el = 0.0;
        gboolean pos_fresh = FALSE;
        gdouble delta_backend_az = 0.0;
        gdouble delta_backend_el = 0.0;
        gint64 since_cmd_us = G_MAXINT64;
        gint64 since_cmd_ms = -1;

        g_mutex_lock(&ctrl->client.mutex);
        desired_user_az = ctrl->client.desired_user_az;
        desired_user_el = ctrl->client.desired_user_el;
        desired_backend_az = ctrl->client.desired_backend_az;
        desired_backend_el = ctrl->client.desired_backend_el;
        raw_azi = ctrl->client.desired_raw_az;
        raw_ele = ctrl->client.desired_raw_el;
        desired_valid = ctrl->client.desired_valid;
        desired_allow = ctrl->client.desired_allow;
        desired_tracking = ctrl->client.desired_tracking;
        force_pending = ctrl->client.force_pending;
        new_trg = ctrl->client.new_trg;
        backoff_until = ctrl->client.reject_backoff_until_us;
        backoff_log_us = ctrl->client.reject_backoff_log_us;
        pos_valid = ctrl->client.pos_valid;
        allow_no_pos = ctrl->client.allow_send_no_pos;
        backend_ioerr_pending = ctrl->client.backend_ioerr_disengage_pending;
        pos_degraded = ctrl->client.pos_degraded;
        reconnect_degraded = ctrl->client.reconnect_degraded;
        daemon_ok = ctrl->client.daemon_ok;
        use_setpos = ctrl->client.use_setpos;
        apply_calib = ctrl->client.apply_calib;
        last_pos_us = ctrl->client.last_pos_us;
        cur_user_az = ctrl->client.azi_in;
        cur_user_el = ctrl->client.ele_in;
        last_cmd_backend_az = ctrl->client.last_cmd_backend_az;
        last_cmd_backend_el = ctrl->client.last_cmd_backend_el;
        last_cmd_backend_valid = ctrl->client.last_cmd_backend_valid;
        last_cmd_ok_us = ctrl->client.last_cmd_ok_us;
        last_cmd_ok_az = ctrl->client.last_cmd_ok_az;
        last_cmd_ok_el = ctrl->client.last_cmd_ok_el;
        last_set_attempt_us = ctrl->client.last_set_attempt_us;
        stop_pending = ctrl->client.stop_pending;
        fallback_az = ctrl->client.azi_out;
        fallback_el = ctrl->client.ele_out;
        fallback_raw_az = ctrl->client.raw_azi_out;
        fallback_raw_el = ctrl->client.raw_ele_out;
        if (stop_pending)
            ctrl->client.new_trg = FALSE;
        ctrl->client.stop_pending = FALSE;
        g_mutex_unlock(&ctrl->client.mutex);

        feedback_disabled = (ctrl->conf && ctrl->conf->disable_pos_feedback_checks);
        if (feedback_disabled && !ctrl->client.feedback_disabled_logged)
        {
            rot_term_log(ctrl, "gpredict:rx",
                         "pos feedback checks disabled (no encoder)");
            g_mutex_lock(&ctrl->client.mutex);
            ctrl->client.feedback_disabled_logged = TRUE;
            g_mutex_unlock(&ctrl->client.mutex);
        }

        session_degraded = backend_ioerr_pending ||
                           (!feedback_disabled && pos_degraded) ||
                           reconnect_degraded;
        if (feedback_disabled)
            allow_no_pos = TRUE;

        if (feedback_disabled)
        {
            pos_fresh = TRUE;
        }
        else if (pos_valid && last_pos_us > 0 && now_us > last_pos_us)
        {
            gint warn_ms = rotctrl_stale_warn_ms(ctrl);
            pos_fresh = ((now_us - last_pos_us) <= ((gint64)warn_ms * 1000));
        }

        if (!desired_valid && new_trg)
        {
            desired_backend_az = fallback_az;
            desired_backend_el = fallback_el;
            raw_azi = fallback_raw_az;
            raw_ele = fallback_raw_el;
            desired_user_az = desired_backend_az;
            desired_user_el = desired_backend_el;
            desired_valid = TRUE;
            desired_allow = TRUE;
            desired_tracking = FALSE;
            force_pending = TRUE;
            g_mutex_lock(&ctrl->client.mutex);
            ctrl->client.new_trg = FALSE;
            g_mutex_unlock(&ctrl->client.mutex);
        }

        azi = desired_backend_az;
        ele = desired_backend_el;
        have_trg = desired_valid;

        if (stop_pending)
        {
            have_trg = FALSE;
            send_cmd = FALSE;
            if (!rotctld_client_stop(ctrl->client.client))
            {
                rot_log_rate_limited(ctrl, &pos_skip_rate,
                                     ROTCTLD_FAILURE_LOG_INTERVAL_US,
                                     SAT_LOG_LEVEL_WARN, "gpredict:warn",
                                     "rotctld stop failed");
            }
            else
            {
                rot_log_rate_limited(ctrl, &pos_skip_rate,
                                     ROTCTLD_FAILURE_LOG_INTERVAL_US,
                                     SAT_LOG_LEVEL_INFO, "gpredict:tx",
                                     "rotctld stop sent");
            }
        }

        if (have_trg)
        {
            gdouble aim_deadband =
                (ctrl->threshold > 0.0) ? ctrl->threshold : 0.10;
            gboolean have_last_cmd = last_cmd_backend_valid || (last_cmd_ok_us > 0);
            gdouble ref_az = last_cmd_backend_valid ? last_cmd_backend_az : last_cmd_ok_az;
            gdouble ref_el = last_cmd_backend_valid ? last_cmd_backend_el : last_cmd_ok_el;
            gint64 last_attempt_us =
                (last_set_attempt_us > 0) ? last_set_attempt_us : last_cmd_ok_us;
            delta_backend_az = have_last_cmd
                                   ? fabs(shortest_az_delta(desired_backend_az, ref_az))
                                   : 0.0;
            delta_backend_el = have_last_cmd
                                   ? fabs(desired_backend_el - ref_el)
                                   : 0.0;
            since_cmd_us =
                (last_attempt_us > 0 && now_us > last_attempt_us)
                    ? (now_us - last_attempt_us)
                    : G_MAXINT64;
            if (since_cmd_us != G_MAXINT64)
                since_cmd_ms = since_cmd_us / 1000;
            gboolean aim_change =
                (!have_last_cmd ||
                 delta_backend_az >= aim_deadband ||
                 delta_backend_el >= aim_deadband);
            gboolean min_step_ok =
                (!have_last_cmd ||
                 delta_backend_az >= aim_deadband ||
                 delta_backend_el >= aim_deadband);
            gboolean min_period_ok =
                (last_attempt_us == 0 ||
                 since_cmd_us >= ((gint64)ROT_CMD_MIN_PERIOD_MS * 1000));
            gboolean trigger_send = (new_trg || force_pending);

            if (!desired_allow && !force_pending)
            {
                send_cmd = FALSE;
                send_reason = "not_allowed";
            }
            else if (!trigger_send)
            {
                send_cmd = FALSE;
                send_reason = "hold";
            }
            else if (!min_period_ok)
            {
                send_cmd = FALSE;
                send_reason = "cooldown";
            }
            else if (!pos_fresh && !allow_no_pos)
            {
                send_cmd = FALSE;
                send_reason = "no_pos";
            }
            else if (force_pending)
            {
                send_cmd = TRUE;
                send_reason = "force";
            }
            else if (aim_change && min_step_ok)
            {
                send_cmd = TRUE;
                send_reason = "aim_change";
            }
            else
            {
                send_cmd = TRUE;
                send_reason = "resend";
            }
        }

        if (use_setpos)
            cmd_name = "set_position";

        if (send_cmd && now_us < backoff_until)
            backoff_active = TRUE;

        {
            gboolean explicit_user_cmd = FALSE;
            rot_comm_status_t comm_state = ROT_COMM_STANDBY;
            if (desired_tracking ||
                ctrl->have_user_command ||
                ctrl->tracking_active ||
                allow_no_pos ||
                ctrl->park_requested ||
                ctrl->cal_active ||
                ctrl->cal_hold_active)
                explicit_user_cmd = TRUE;

            g_mutex_lock(&ctrl->client.mutex);
            comm_state = ctrl->comm_status;
            g_mutex_unlock(&ctrl->client.mutex);

            if (ctrl->hold_position_on_engage && explicit_user_cmd)
            {
                ctrl->hold_position_on_engage = FALSE;
                ctrl->hold_position_log_emitted = FALSE;
            }

            if (send_cmd && comm_state == ROT_COMM_LINK_LOST)
            {
                send_cmd = FALSE;
            }
            else if (send_cmd && ctrl->hold_position_on_engage && !explicit_user_cmd)
            {
                if (ctrl->verbose_logging)
                    rot_term_log_verbose(ctrl, "gpredict:rx",
                                         "hold: suppress setpos az=%.2f el=%.2f reason=%s",
                                         desired_backend_az,
                                         desired_backend_el,
                                         send_reason ? send_reason : "none");
                send_cmd = FALSE;
            }
        }

        if (send_cmd)
        {
            if (!daemon_ok)
            {
                send_cmd = FALSE;
                rot_log_rate_limited(ctrl, &pos_skip_rate,
                                     ROTCTLD_FAILURE_LOG_INTERVAL_US,
                                     SAT_LOG_LEVEL_WARN, "gpredict:err",
                                     "rotctld connection not verified; skipping %s",
                                     cmd_name);
            }
            else if (session_degraded && !allow_no_pos)
            {
                send_cmd = FALSE;
                rot_log_rate_limited(ctrl, &pos_skip_rate,
                                     ROTCTLD_FAILURE_LOG_INTERVAL_US,
                                     SAT_LOG_LEVEL_WARN, "gpredict:warn",
                                     "%s deferred: session degraded",
                                     cmd_name);
            }
            else if (!pos_valid && !allow_no_pos)
            {
                send_cmd = FALSE;
                rot_log_rate_limited(ctrl, &pos_skip_rate,
                                     ROTCTLD_FAILURE_LOG_INTERVAL_US,
                                     SAT_LOG_LEVEL_WARN, "gpredict:warn",
                                     "%s deferred: position unknown",
                                     cmd_name);
            }
        }

        if (backoff_active)
        {
            if (now_us - backoff_log_us > 2000000) {
                rot_term_log(ctrl, "gpredict:err",
                             "%s backoff active; retrying in %.1f s",
                             cmd_name,
                             (backoff_until - now_us) / 1e6);
                g_mutex_lock(&ctrl->client.mutex);
                ctrl->client.reject_backoff_log_us = now_us;
                g_mutex_unlock(&ctrl->client.mutex);
            }
            send_cmd = FALSE;
        }

        if (send_cmd && ctrl->verbose_logging)
        {
            rot_term_log(ctrl, "gpredict:tx",
                         "rot_supervise: send reason=%s dt_ms=%lld delta=(%.2f, %.2f) target=(%.2f, %.2f)",
                         send_reason,
                         (long long)since_cmd_ms,
                         delta_backend_az,
                         delta_backend_el,
                         azi,
                         ele);
        }

        if (send_cmd)
        {
            RotTransformSnapshotState snap_state;
            gboolean snapshot_ready = FALSE;
            gboolean target_ok = FALSE;
            gboolean clear_trg = FALSE;
            rot_target_caps_t caps = { 0 };
            rot_target_invalid_reason_t reason = ROT_TARGET_INVALID_NONE;
            gboolean wrap_mismatch = FALSE;
            gdouble norm_az = 0.0;
            gchar azs[32];
            gchar els[32];

            rot_transform_snapshot_state_init(&snap_state);
            snapshot_ready = rot_transform_snapshot_state_get(ctrl, &snap_state);
            rotctrl_build_target_caps(ctrl, &caps);
            target_ok = rotctrl_target_is_valid(&caps, azi, ele,
                                                &norm_az, &reason,
                                                &wrap_mismatch);
            if (wrap_mismatch)
                rotctrl_note_wrap_mismatch(ctrl);

            if (!snapshot_ready)
            {
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            "%s: defer %s; transform snapshot not ready",
                            __func__, cmd_name);
            }
            else if (!target_ok)
            {
                sat_log_log(SAT_LOG_LEVEL_WARN,
                            "%s: drop invalid target raw=(%.2f, %.2f) cmd=(%.2f, %.2f) norm=%.2f reason=%s snap=%" G_GUINT64_FORMAT,
                            __func__, raw_azi, raw_ele, azi, ele,
                            norm_az, rot_target_invalid_reason_name(reason),
                            snap_state.version);
                clear_trg = TRUE;
            }
            else
            {
                clear_trg = TRUE;
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            "%s: send raw=(%.2f, %.2f) cmd=(%.2f, %.2f) snap=%" G_GUINT64_FORMAT,
                            __func__, raw_azi, raw_ele, azi, ele,
                            snap_state.version);

                rot_format_deg_2(azs, sizeof(azs), azi);
                rot_format_deg_2(els, sizeof(els), ele);
                rot_term_log_verbose(ctrl, "gpredict:tx",
                                     "%s az=%s el=%s",
                                     cmd_name, azs, els);

                if (!daemon_ok) {
                    io_error = TRUE;
                    rot_term_log(ctrl, "gpredict:err",
                                 "rotctld connection not verified; skipping %s to %s:%d",
                                 cmd_name,
                                 ctrl->conf ? ctrl->conf->host : "(null)",
                                 rotctld_effective_port(ctrl));
                    rotctld_socket_close_quiet(ctrl, &ctrl->client.socket);
                    rotctld_clear_rxbuf(ctrl);
                    g_mutex_lock(&ctrl->client.mutex);
                    ctrl->client.limits_valid = FALSE;
                    ctrl->client.south_zero = FALSE;
                    g_atomic_int_set(&ctrl->south_zero_cached, 0);
                    ctrl->client.daemon_ok = FALSE;
                    if (clear_trg)
                        ctrl->client.new_trg = FALSE;
                    if (clear_trg)
                        ctrl->client.allow_send_no_pos = FALSE;
                    if (clear_trg)
                        ctrl->client.use_setpos = FALSE;
                    if (clear_trg)
                        ctrl->client.apply_calib = FALSE;
                    g_mutex_unlock(&ctrl->client.mutex);
                    continue;
                }

                if (ctrl->verbose_logging)
                {
                    gdouble send_az = azi;
                    gdouble send_el = ele;
                    gdouble mech_az = send_az;
                    gdouble mech_el = send_el;
                    gdouble log_backend_az = 0.0;
                    gdouble log_backend_el = 0.0;

                    if (apply_calib)
                        rotctrl_calib_apply_send(ctrl,
                                                 send_az, send_el,
                                                 &mech_az, &mech_el);

                    format_rotctld_setpos(ctrl,
                                          mech_az, mech_el,
                                          &send_az, &send_el,
                                          NULL, 0);

                    g_mutex_lock(&ctrl->client.mutex);
                    log_backend_az = ctrl->client.azi_mech_in;
                    log_backend_el = ctrl->client.ele_mech_in;
                    g_mutex_unlock(&ctrl->client.mutex);

                    rot_term_log_verbose(ctrl, "gpredict:tx",
                                         "rot_send backend_pos=(%.2f,%.2f) user_pos=(%.2f,%.2f) "
                                         "desired_backend=(%.2f,%.2f) desired_user=(%.2f,%.2f) "
                                         "send=(%.2f,%.2f)",
                                         log_backend_az, log_backend_el,
                                         cur_user_az, cur_user_el,
                                         desired_backend_az, desired_backend_el,
                                         desired_user_az, desired_user_el,
                                         send_az, send_el);
                }

                g_mutex_lock(&ctrl->client.mutex);
                ctrl->client.last_set_attempt_us = now_us;
                ctrl->client.last_cmd_backend_az = azi;
                ctrl->client.last_cmd_backend_el = ele;
                ctrl->client.last_cmd_backend_valid = TRUE;
                ctrl->client.force_pending = FALSE;
                g_mutex_unlock(&ctrl->client.mutex);

                gint rprt_code = 0;
                rot_set_result_t set_res =
                    rotctrl_send_position(ctrl,
                                          azi,
                                          ele,
                                          &rprt_code,
                                          use_setpos,
                                          apply_calib);
                gboolean backend_disengage = FALSE;

                if (set_res == ROT_SET_BACKEND_IO)
                {
                    if (rprt_code == -6)
                        backend_disengage =
                            rotctld_backend_ioerr_note(ctrl, cmd_name);
                    else
                        rotctld_backend_ioerr_reset(ctrl);
                }
                else
                {
                    rotctld_backend_ioerr_reset(ctrl);
                }

                if (set_res == ROT_SET_IO_ERROR)
                {
                    (void)rotctld_note_io_failure(ctrl, cmd_name);
                    io_error = TRUE;
                    rot_log_rate_limited(ctrl, &transport_err_rate,
                                         ROTCTLD_FAILURE_LOG_INTERVAL_US,
                                         SAT_LOG_LEVEL_ERROR, "gpredict:err",
                                         "%s failed: transport error",
                                         cmd_name);
                    g_mutex_lock(&ctrl->client.mutex);
                    if (ctrl->client.transport_backoff_sec <= 0.0)
                        ctrl->client.transport_backoff_sec = 0.5;
                    ctrl->client.transport_backoff_until_us =
                        now_us +
                        (gint64)(ctrl->client.transport_backoff_sec * 1e6);
                    ctrl->client.transport_backoff_sec =
                        MIN(ctrl->client.transport_backoff_sec * 2.0, backoff_max);
                    if (ctrl->client.io_error_reason[0] == '\0')
                        g_strlcpy(ctrl->client.io_error_reason, "transport error",
                                  sizeof(ctrl->client.io_error_reason));
                    g_mutex_unlock(&ctrl->client.mutex);
                }
                else if (set_res == ROT_SET_BACKEND_IO)
                {
                    rotctld_note_io_ok(ctrl);
                    rot_log_rate_limited(ctrl, &setpos_err_rate,
                                         ROTCTLD_FAILURE_LOG_INTERVAL_US,
                                         SAT_LOG_LEVEL_WARN, "gpredict:err",
                                         "%s backend error: %s (RPRT %d). "
                                         "Check rotor model, serial device path, baud rate, and rotctld -m/-r.",
                                         cmd_name,
                                         rot_rprt_error_string(rprt_code),
                                         rprt_code);
                    g_mutex_lock(&ctrl->client.mutex);
                    if (ctrl->client.last_pos_us <= 0)
                        ctrl->client.pos_valid = FALSE;
                    ctrl->client.pos_unknown = TRUE;
                    ctrl->client.pos_cmd_ok = TRUE;
                    g_mutex_unlock(&ctrl->client.mutex);
                }
                else if (set_res == ROT_SET_REJECTED)
                {
                    rotctld_note_io_ok(ctrl);
                    if (rprt_code != 0)
                    {
                        rot_log_rate_limited(ctrl, &setpos_err_rate,
                                             ROTCTLD_FAILURE_LOG_INTERVAL_US,
                                             SAT_LOG_LEVEL_WARN, "gpredict:err",
                                             "%s rejected by rotctld: %s (RPRT %d)",
                                             cmd_name,
                                             rot_rprt_error_string(rprt_code),
                                             rprt_code);
                    }
                    else
                    {
                        rot_log_rate_limited(ctrl, &setpos_err_rate,
                                             ROTCTLD_FAILURE_LOG_INTERVAL_US,
                                             SAT_LOG_LEVEL_WARN, "gpredict:err",
                                             "%s rejected by rotctld",
                                             cmd_name);
                    }
                }
                else
                {
                    rotctld_note_io_ok(ctrl);
                    rot_term_log_verbose(ctrl, "gpredict:rx",
                                         "%s accepted", cmd_name);
                }

                if (backend_disengage)
                    rot_schedule_backend_io_disengage(ctrl);

                g_mutex_lock(&ctrl->client.mutex);
                if (set_res == ROT_SET_BACKEND_IO) {
                    ctrl->client.pos_unknown = TRUE;
                    ctrl->client.pos_cmd_ok = TRUE;
                    if (ctrl->client.reject_backoff_sec <= 0.0)
                        ctrl->client.reject_backoff_sec = 0.5;
                    ctrl->client.reject_backoff_until_us =
                        now_us + (gint64)(ctrl->client.reject_backoff_sec * 1e6);
                    ctrl->client.reject_backoff_sec =
                        MIN(ctrl->client.reject_backoff_sec * 2.0, backoff_max);
                } else if (set_res == ROT_SET_REJECTED) {
                    if (rprt_code != 0) {
                        ctrl->client.pos_unknown = TRUE;
                        ctrl->client.pos_cmd_ok = TRUE;
                    }
                    ctrl->client.cmd_rejected = TRUE;
                    if (ctrl->client.reject_backoff_sec <= 0.0)
                        ctrl->client.reject_backoff_sec = 0.5;
                    ctrl->client.reject_backoff_until_us =
                        now_us + (gint64)(ctrl->client.reject_backoff_sec * 1e6);
                    ctrl->client.reject_backoff_sec =
                        MIN(ctrl->client.reject_backoff_sec * 2.0, backoff_max);
                } else if (set_res == ROT_SET_OK) {
                    ctrl->client.cmd_rejected = FALSE;
                    ctrl->client.reject_backoff_until_us = 0;
                    ctrl->client.reject_backoff_sec = 0.5;
                    ctrl->client.pos_cmd_ok = TRUE;
                    ctrl->client.set_pos_ok = TRUE;
                    ctrl->client.last_cmd_ok_az = azi;
                    ctrl->client.last_cmd_ok_el = ele;
                    ctrl->client.last_cmd_ok_us = now_us;
                    ctrl->client.last_cmd_backend_az = azi;
                    ctrl->client.last_cmd_backend_el = ele;
                    ctrl->client.last_cmd_backend_valid = TRUE;
                    ctrl->client.move_grace_until_us =
                        now_us + ((gint64)ROT_CMD_MOVE_GRACE_MS * 1000);
                    ctrl->client.discrepancy_pending = FALSE;
                }
                if (clear_trg)
                    ctrl->client.new_trg = FALSE;
                if (clear_trg)
                    ctrl->client.allow_send_no_pos = FALSE;
                if (clear_trg)
                    ctrl->client.use_setpos = FALSE;
                if (clear_trg)
                    ctrl->client.apply_calib = FALSE;
                g_mutex_unlock(&ctrl->client.mutex);
                clear_trg = FALSE;
            }

            if (clear_trg)
            {
                g_mutex_lock(&ctrl->client.mutex);
                ctrl->client.new_trg = FALSE;
                ctrl->client.allow_send_no_pos = FALSE;
                ctrl->client.use_setpos = FALSE;
                ctrl->client.apply_calib = FALSE;
                g_mutex_unlock(&ctrl->client.mutex);
            }
        }
        else
        {
            gchar azs[32];
            gchar els[32];
            rot_format_deg_2(azs, sizeof(azs), azi);
            rot_format_deg_2(els, sizeof(els), ele);
            rot_term_log_verbose(ctrl, "gpredict:rx",
                                 "idle: no new target (last_out=%s,%s)",
                                 azs, els);
        }

        if (!io_error && !send_cmd && ctrl->client.client != NULL)
        {
            gdouble cur_az = 0.0;
            gdouble cur_el = 0.0;
            gboolean pos_unknown = FALSE;
            gint64 now_us = g_get_monotonic_time();
            gint64 last_attempt = 0;
            const gint64 retry_us = ROTCTLD_POS_UNKNOWN_BACKOFF_US;
            gboolean do_retry = TRUE;
            gint64 pos_backoff_until_us = 0;
            gdouble pos_backoff_sec = 0.0;
            guint pos_failures = 0;
            HamlibResponseInfo info = { 0 };
            rotctld_pos_result_t pos_res = ROTCTLD_POS_IO_ERR;
            gboolean pos_timeout = FALSE;
            gboolean pos_rprt_err = FALSE;
            gboolean pos_rprt_io = FALSE;
            gboolean backend_disengage = FALSE;
            gboolean desync_detected = FALSE;
            gint pos_timeout_ms = rotctrl_getpos_timeout_ms(ctrl);
            gdouble sample_step_az = 0.0;
            gdouble sample_step_el = 0.0;
            gint64 sample_step_dt_us = 0;
            gboolean sample_step_valid = FALSE;
            gchar pos_reply[256];

            g_mutex_lock(&ctrl->client.mutex);
            pos_unknown = ctrl->client.pos_unknown;
            last_attempt = ctrl->client.last_pos_attempt_us;
            pos_backoff_until_us = ctrl->client.pos_backoff_until_us;
            pos_backoff_sec = ctrl->client.pos_backoff_sec;
            pos_failures = ctrl->client.pos_failures;
            if (pos_unknown && last_attempt > 0 &&
                (now_us - last_attempt) < retry_us)
                do_retry = FALSE;
            if (pos_backoff_until_us > 0 && now_us < pos_backoff_until_us)
                do_retry = FALSE;
            else
                ctrl->client.last_pos_attempt_us = now_us;
            g_mutex_unlock(&ctrl->client.mutex);

            if (!do_retry)
            {
                /* defer get_position retry while pos is unknown */
                if (pos_backoff_until_us > 0 && now_us < pos_backoff_until_us)
                {
                    rot_log_rate_limited(ctrl, &pos_backoff_rate,
                                         ROTCTLD_FAILURE_LOG_INTERVAL_US,
                                         SAT_LOG_LEVEL_WARN, "gpredict:warn",
                                         "get_position backoff active; retrying in %.1fs",
                                         (pos_backoff_until_us - now_us) / 1e6);
                }
            }
            else
            {
                memset(&info, 0, sizeof(info));
                pos_reply[0] = '\0';
                pos_res = rotctld_client_get_pos_ex_timeout(ctrl->client.client,
                                                            &cur_az, &cur_el,
                                                            &info,
                                                            pos_reply,
                                                            sizeof(pos_reply),
                                                            pos_timeout_ms,
                                                            0);
                if (ctrl->verbose_logging && pos_res == ROTCTLD_POS_OK)
                {
                    gchar *view = rotctld_sanitize_reply(pos_reply, 120);
                    rot_term_log_verbose(ctrl, "gpredict:rx",
                                         "get_position reply=%s parsed az=%.2f el=%.2f",
                                         view ? view : "(none)",
                                         cur_az,
                                         cur_el);
                    g_free(view);
                }
                if (rotctld_stop_requested(ctrl) ||
                    rotctld_generation_stale(ctrl, session_gen))
                    goto out_stop;
                if (pos_res == ROTCTLD_POS_PARSE_FAIL &&
                    rotctld_reply_is_dump_state(pos_reply))
                    desync_detected = TRUE;

                pos_timeout = (pos_res == ROTCTLD_POS_TIMEOUT);
                pos_rprt_err = (pos_res == ROTCTLD_POS_RPRT_ERR && info.saw_rprt);
                pos_rprt_io = pos_rprt_err &&
                              (info.rprt_code == -6 ||
                               info.rprt_code == -5 ||
                               info.rprt_code == -8);

                if (desync_detected)
                {
                    sat_log_log(SAT_LOG_LEVEL_ERROR,
                                "rotctld desync: got dump_state while expecting get_position");
                    rot_term_log(ctrl, "gpredict:err",
                                 "rotctld desync: got dump_state while expecting get_position");
                    rotctld_clear_rxbuf(ctrl);
                    rotctld_drain_transport(ctrl, 50);
                    g_mutex_lock(&ctrl->client.mutex);
                    if (ctrl->client.io_error_reason[0] == '\0')
                        g_strlcpy(ctrl->client.io_error_reason, "desync",
                                  sizeof(ctrl->client.io_error_reason));
                    g_mutex_unlock(&ctrl->client.mutex);
                    io_error = TRUE;
                    rotctld_socket_close_quiet(ctrl, &ctrl->client.socket);
                    goto get_pos_done;
                }
                else if (pos_res == ROTCTLD_POS_OK)
                {
                    gboolean caps_ok = FALSE;
                    gdouble cap_min = 0.0;
                    gdouble cap_max = 360.0;

                    g_mutex_lock(&ctrl->client.mutex);
                    caps_ok = ctrl->client.limits_valid;
                    if (caps_ok)
                    {
                        cap_min = ctrl->client.az_min;
                        cap_max = ctrl->client.az_max;
                    }
                    g_mutex_unlock(&ctrl->client.mutex);

                    if (caps_ok)
                        cur_az = rotctrl_normalize_backend_az(cur_az,
                                                              cap_min,
                                                              cap_max);
                    else
                        cur_az = rotctrl_normalize_backend_az(cur_az, 0.0, 360.0);
                    if (ctrl->verbose_logging)
                        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                                    "rot/angle: backend az=%.2f el=%.2f -> az360=%.2f",
                                    cur_az, cur_el, gp_backend_to_az360(cur_az));

                    if (pos_unknown)
                    {
                        sat_log_log(SAT_LOG_LEVEL_INFO,
                                    "get_position recovered: az=%.2f el=%.2f",
                                    cur_az, cur_el);
                    }
                    gdouble mech_az = cur_az;
                    gdouble mech_el = cur_el;
                    gdouble user_az = 0.0;
                    gdouble user_el = 0.0;
                    gdouble prev_user_az = 0.0;
                    gdouble prev_user_el = 0.0;
                    gint64 prev_sample_us = 0;
                    gint64 pos_now_us = 0;

                    rotctrl_map_mech_to_user(ctrl,
                                             mech_az,
                                             mech_el,
                                             &user_az,
                                             &user_el);

                    g_mutex_lock(&ctrl->client.mutex);
                    if (ctrl->client.last_pos_sample_us > 0)
                    {
                        prev_sample_us = ctrl->client.last_pos_sample_us;
                        prev_user_az = ctrl->client.last_pos_user_az;
                        prev_user_el = ctrl->client.last_pos_user_el;
                    }
                    pos_now_us = g_get_monotonic_time();
                    ctrl->client.azi_mech_in = mech_az;
                    ctrl->client.ele_mech_in = mech_el;
                    ctrl->client.azi_in = user_az;
                    ctrl->client.azi_az360 = gp_backend_to_az360(cur_az);
                    ctrl->client.ele_in = user_el;
                    ctrl->client.last_pos_user_az = user_az;
                    ctrl->client.last_pos_user_el = user_el;
                    ctrl->client.last_pos_sample_us = pos_now_us;
                    ctrl->client.pos_valid = TRUE;
                    ctrl->client.pos_invalid = FALSE;
                    ctrl->client.pos_unknown = FALSE;
                    ctrl->client.pos_cmd_ok = TRUE;
                    ctrl->client.handshake_pos_ok = TRUE;
                    ctrl->client.first_pos_deadline_us = 0;
                    ctrl->client.last_pos_us = pos_now_us;
                    ctrl->client.last_pos_error[0] = '\0';
                    g_mutex_unlock(&ctrl->client.mutex);
                    if (prev_sample_us > 0 && pos_now_us > prev_sample_us)
                    {
                        sample_step_dt_us = pos_now_us - prev_sample_us;
                        sample_step_az = rot_ang_dist_deg(user_az, prev_user_az);
                        sample_step_el = fabs(user_el - prev_user_el);
                        sample_step_valid = TRUE;
                    }
                    rotctld_note_io_ok(ctrl);
                    rotctld_backend_ioerr_reset(ctrl);
                    g_mutex_lock(&ctrl->client.mutex);
                    ctrl->client.pos_failures = 0;
                    ctrl->client.pos_backoff_until_us = 0;
                    ctrl->client.pos_backoff_sec = 0.5;
                    ctrl->client.pos_degraded = FALSE;
                    g_mutex_unlock(&ctrl->client.mutex);

                    {
                        gboolean do_probe = FALSE;
                        g_mutex_lock(&ctrl->client.mutex);
                        if (!ctrl->client.set_pos_ok)
                        {
                            gint64 last_probe_us = ctrl->client.last_set_attempt_us;
                            if (last_probe_us == 0 ||
                                (now_us - last_probe_us) > ROTCTLD_POS_UNKNOWN_BACKOFF_US)
                            {
                                ctrl->client.last_set_attempt_us = now_us;
                                do_probe = TRUE;
                            }
                        }
                        g_mutex_unlock(&ctrl->client.mutex);

                        if (do_probe)
                        {
                            if (rotctrl_set_position_guarded(ctrl,
                                                             cur_az,
                                                             cur_el,
                                                             "pos_probe"))
                            {
                                g_mutex_lock(&ctrl->client.mutex);
                                ctrl->client.set_pos_ok = TRUE;
                                g_mutex_unlock(&ctrl->client.mutex);
                            }
                        }
                    }
                }
                else if (pos_timeout || pos_rprt_err)
                {
                    gdouble delay_sec =
                        (pos_backoff_sec > 0.0) ? pos_backoff_sec : 0.5;
                    gchar pos_err[64] = { 0 };
                    gboolean mark_invalid = FALSE;

                    if (!pos_timeout)
                        rotctld_note_io_ok(ctrl);
                    if (pos_rprt_err &&
                        (info.rprt_code == -6 ||
                         info.rprt_code == -5 ||
                         info.rprt_code == -8))
                        backend_disengage =
                            rotctld_backend_ioerr_note(ctrl, "get_position");
                    else
                        rotctld_backend_ioerr_reset(ctrl);
                    if (pos_rprt_io)
                    {
                        rot_log_rate_limited(ctrl, &getpos_err_rate,
                                             ROTCTLD_FAILURE_LOG_INTERVAL_US,
                                             SAT_LOG_LEVEL_WARN, "gpredict:err",
                                             "get_position backend error: %s (RPRT %d). "
                                             "Check rotor model, serial device path, baud rate, and rotctld -m/-r.",
                                             rot_rprt_error_string(info.rprt_code),
                                             info.rprt_code);
                    }
                    else if (pos_rprt_err)
                    {
                        rot_log_rate_limited(ctrl, &getpos_err_rate,
                                             ROTCTLD_FAILURE_LOG_INTERVAL_US,
                                             SAT_LOG_LEVEL_WARN, "gpredict:err",
                                             "get_position failed: %s (RPRT %d), continuing without position",
                                             rot_rprt_error_string(info.rprt_code),
                                             info.rprt_code);
                    }
                    else if (pos_timeout)
                    {
                        rot_log_rate_limited(ctrl, &getpos_err_rate,
                                             ROTCTLD_FAILURE_LOG_INTERVAL_US,
                                             SAT_LOG_LEVEL_WARN, "gpredict:err",
                                             "get_position timeout, continuing without position");
                        g_strlcpy(pos_err, "timeout", sizeof(pos_err));
                    }
                    else
                    {
                        rot_log_rate_limited(ctrl, &getpos_err_rate,
                                             ROTCTLD_FAILURE_LOG_INTERVAL_US,
                                             SAT_LOG_LEVEL_WARN, "gpredict:err",
                                             "get_position reply missing az/el, continuing without position");
                        g_strlcpy(pos_err, "parse", sizeof(pos_err));
                    }
                    if (pos_rprt_err)
                        g_snprintf(pos_err, sizeof(pos_err), "rprt %d",
                                   info.rprt_code);
                    if (pos_rprt_err && info.rprt_code == -6)
                    {
                        mark_invalid = TRUE;
                        rot_log_rate_limited(ctrl, &getpos_err_rate,
                                             ROTCTLD_FAILURE_LOG_INTERVAL_US,
                                             SAT_LOG_LEVEL_WARN, "gpredict:err",
                                             "pos_invalid: get_position returned RPRT -6; backend NOT_READY");
                    }
                    g_mutex_lock(&ctrl->client.mutex);
                    if (ctrl->client.last_pos_us <= 0)
                        ctrl->client.pos_valid = FALSE;
                    if (mark_invalid)
                    {
                        ctrl->client.pos_valid = FALSE;
                        ctrl->client.pos_invalid = TRUE;
                    }
                    ctrl->client.pos_unknown = TRUE;
                    ctrl->client.pos_cmd_ok = TRUE;
                    if (ctrl->client.last_pos_us <= 0)
                        ctrl->client.last_pos_us = 0;
                    ctrl->client.pos_failures = ++pos_failures;
                    ctrl->client.pos_backoff_until_us =
                        now_us +
                        (gint64)(delay_sec * 1e6);
                    ctrl->client.pos_backoff_sec =
                        MIN(delay_sec * 2.0, backoff_max);
                    if (pos_err[0] != '\0')
                        g_strlcpy(ctrl->client.last_pos_error, pos_err,
                                  sizeof(ctrl->client.last_pos_error));
                    if (pos_failures >= ROTCTLD_POS_MAX_FAIL)
                        ctrl->client.pos_degraded = TRUE;
                    g_mutex_unlock(&ctrl->client.mutex);
                    rot_log_rate_limited(ctrl, &getpos_err_rate,
                                         ROTCTLD_FAILURE_LOG_INTERVAL_US,
                                         SAT_LOG_LEVEL_WARN, "gpredict:err",
                                         "get_position failed; attempt %u/%u backoff %.1fs (keeping connection)",
                                         pos_failures,
                                         ROTCTLD_POS_MAX_FAIL,
                                         delay_sec);
                }
                else
                {
                    gdouble delay_sec =
                        (pos_backoff_sec > 0.0) ? pos_backoff_sec : 0.5;
                    const gchar *pos_err = "io";

                    rotctld_backend_ioerr_reset(ctrl);
                    if (pos_res == ROTCTLD_POS_PARSE_FAIL)
                        pos_err = "parse";
                    g_mutex_lock(&ctrl->client.mutex);
                    ctrl->client.pos_failures = ++pos_failures;
                    ctrl->client.pos_unknown = TRUE;
                    ctrl->client.pos_cmd_ok = TRUE;
                    ctrl->client.pos_backoff_until_us =
                        now_us +
                        (gint64)(delay_sec * 1e6);
                    ctrl->client.pos_backoff_sec =
                        MIN(delay_sec * 2.0, backoff_max);
                    g_strlcpy(ctrl->client.last_pos_error, pos_err,
                              sizeof(ctrl->client.last_pos_error));
                    if (pos_failures >= ROTCTLD_POS_MAX_FAIL)
                        ctrl->client.pos_degraded = TRUE;
                    g_mutex_unlock(&ctrl->client.mutex);
                    rot_log_rate_limited(ctrl, &getpos_err_rate,
                                         ROTCTLD_FAILURE_LOG_INTERVAL_US,
                                         SAT_LOG_LEVEL_WARN, "gpredict:err",
                                         "get_position failed; attempt %u/%u backoff %.1fs (keeping connection)",
                                         pos_failures,
                                         ROTCTLD_POS_MAX_FAIL,
                                         delay_sec);
                }

                {
                    gboolean poll_ok = (pos_res == ROTCTLD_POS_OK);
                    gboolean poll_timeout = (pos_res == ROTCTLD_POS_TIMEOUT &&
                                             info.bytes == 0 &&
                                             pos_reply[0] == '\0');
                    gboolean poll_bad = (!poll_ok && !poll_timeout);
                    gboolean poll_stuck = FALSE;
                    gboolean poll_fail = FALSE;
                    guint discrepancy_streak = 0;
                    gboolean discrepancy_pending = FALSE;
                    rot_comm_status_t prev_status;
                    rot_comm_status_t new_status;
                    gboolean enter_link_lost = FALSE;
                    guint fail_streak = 0;

                    g_mutex_lock(&ctrl->client.mutex);
                    prev_status = ctrl->comm_status;
                    discrepancy_streak = ctrl->client.discrepancy_streak;
                    discrepancy_pending = ctrl->client.discrepancy_pending;
                    poll_stuck = (ctrl->tracking &&
                                  discrepancy_pending &&
                                  discrepancy_streak >=
                                      ROT_CMD_DISCREPANCY_STRIKES);
                    poll_fail = (!poll_ok) || poll_stuck;

                    if (!poll_fail)
                    {
                        if (ctrl->ok_streak < G_MAXUINT)
                            ctrl->ok_streak++;
                        ctrl->bad_streak = 0;
                        ctrl->timeout_streak = 0;
                    }
                    else
                    {
                        if (ctrl->bad_streak < G_MAXUINT)
                            ctrl->bad_streak++;
                        ctrl->ok_streak = 0;

                        if (poll_timeout)
                        {
                            if (ctrl->timeout_streak < G_MAXUINT)
                                ctrl->timeout_streak++;
                        }
                        else
                        {
                            ctrl->timeout_streak = 0;
                        }
                    }

                    new_status = prev_status;
                    if (!ctrl->engaged)
                    {
                        new_status = ROT_COMM_DISENGAGED;
                    }
                    else if (ctrl->bad_streak >= ROT_COMM_LINK_LOST_FAIL_POLLS)
                    {
                        new_status = ROT_COMM_LINK_LOST;
                    }
                    else if (ctrl->bad_streak >= ROT_COMM_DEGRADED_FAIL_POLLS)
                    {
                        new_status = ROT_COMM_DEGRADED;
                    }
                    else if (ctrl->ok_streak >= 2 && prev_status == ROT_COMM_DEGRADED)
                    {
                        new_status = ROT_COMM_STANDBY;
                    }
                    else if (poll_ok && prev_status == ROT_COMM_DISENGAGED)
                    {
                        new_status = ROT_COMM_STANDBY;
                    }

                    ctrl->comm_status = new_status;
                    fail_streak = ctrl->bad_streak;
                    enter_link_lost = (prev_status != ROT_COMM_LINK_LOST &&
                                       new_status == ROT_COMM_LINK_LOST);
                    g_mutex_unlock(&ctrl->client.mutex);

                    if (enter_link_lost)
                    {
                        const gchar *cause = poll_stuck ? "no motion" :
                                             (poll_timeout ? "timeout" :
                                              (poll_bad ? "invalid poll reply"
                                                        : "no feedback"));

                        rot_term_log(ctrl, "gpredict:err",
                                     "rotor link lost after %u failed poll(s) "
                                     "(degraded after %u); cause=%s",
                                     fail_streak,
                                     ROT_COMM_DEGRADED_FAIL_POLLS,
                                     cause);
                        ctrl->hold_position_on_engage = TRUE;
                        ctrl->hold_position_log_emitted = FALSE;
                        rotctld_request_thread_stop(ctrl, TRUE);
                        g_idle_add_full(G_PRIORITY_DEFAULT,
                                        rotctrl_link_lost_popup_cb,
                                        g_object_ref(ctrl),
                                        g_object_unref);
                        g_idle_add_full(G_PRIORITY_DEFAULT,
                                        rotctrl_link_lost_disengage_cb,
                                        g_object_ref(ctrl),
                                        g_object_unref);
                    }
                }

                if (!feedback_disabled)
                {
                    gint64 ref_cmd_us = 0;
                    gboolean recent_cmd = FALSE;
                    gboolean monitor_discrepancy = FALSE;
                    gboolean have_last_cmd =
                        last_cmd_backend_valid || (last_cmd_ok_us > 0);
                    gdouble cmd_az =
                        last_cmd_backend_valid ? last_cmd_backend_az : last_cmd_ok_az;
                    gdouble cmd_el =
                        last_cmd_backend_valid ? last_cmd_backend_el : last_cmd_ok_el;
                    gboolean discrepancy_hit = FALSE;
                    const gchar *disc_reason = NULL;
                    gdouble err_deg = 0.0;

                    if (last_set_attempt_us > ref_cmd_us)
                        ref_cmd_us = last_set_attempt_us;
                    if (last_cmd_ok_us > ref_cmd_us)
                        ref_cmd_us = last_cmd_ok_us;
                    if (ref_cmd_us > 0 && now_us >= ref_cmd_us &&
                        (now_us - ref_cmd_us) <=
                            ((gint64)ROT_CMD_MAX_SILENCE_MS * 1000))
                    {
                        recent_cmd = TRUE;
                    }
                    monitor_discrepancy = desired_tracking || recent_cmd;

                    if (monitor_discrepancy &&
                        pos_res == ROTCTLD_POS_OK && have_last_cmd)
                    {
                        gdouble az_err = fabs(shortest_az_delta(cur_az, cmd_az));
                        gdouble el_err = fabs(cur_el - cmd_el);
                        gdouble aim_deadband =
                            (ctrl->threshold > 0.0) ? ctrl->threshold : 0.10;
                        gdouble accept =
                            MAX(aim_deadband * 2.0, 0.5);
                        gdouble move_eps =
                            MAX(aim_deadband * 0.25, 0.15);
                        gdouble eps = rotctrl_angle_epsilon(ctrl);
                        gboolean within_eps =
                            (az_err <= eps && el_err <= eps);
                        gboolean below_min_step =
                            (az_err <= ROT_CMD_MIN_AZ_DEG &&
                             el_err <= ROT_CMD_MIN_EL_DEG);
                        gboolean sample_advancing =
                            (sample_step_valid &&
                             sample_step_dt_us > 0 &&
                             sample_step_dt_us <=
                                 ((gint64)ROT_CMD_MAX_SILENCE_MS * 1000) &&
                             (sample_step_az >= move_eps ||
                              sample_step_el >= move_eps));
                        gint64 grace_until = 0;
                        gdouble last_err = 0.0;
                        gint64 last_err_time = 0;
                        guint wrong_way = 0;

                        err_deg = MAX(az_err, el_err);

                        g_mutex_lock(&ctrl->client.mutex);
                        grace_until = ctrl->client.move_grace_until_us;
                        last_err = ctrl->client.last_err_to_cmd;
                        last_err_time = ctrl->client.last_err_time_us;
                        wrong_way = ctrl->client.wrong_way_count;
                        g_mutex_unlock(&ctrl->client.mutex);

                        if (now_us < grace_until)
                        {
                            last_err_time = now_us;
                            wrong_way = 0;
                        }
                        else if (within_eps || below_min_step || err_deg <= accept)
                        {
                            last_err_time = now_us;
                            wrong_way = 0;
                        }
                        else
                        {
                            if (last_err_time == 0)
                                last_err_time = now_us;
                            if (last_err > 0.0)
                            {
                                if (err_deg <= (last_err - 0.1))
                                {
                                    last_err_time = now_us;
                                    wrong_way = 0;
                                }
                                else if (err_deg >= (last_err + 0.1))
                                {
                                    if (sample_advancing)
                                    {
                                        last_err_time = now_us;
                                        wrong_way = 0;
                                    }
                                    else if (wrong_way < G_MAXUINT)
                                    {
                                        wrong_way++;
                                    }
                                }
                            }
                            if (wrong_way >= 3)
                            {
                                discrepancy_hit = TRUE;
                                disc_reason = "wrong_way";
                            }
                            else if ((now_us - last_err_time) >=
                                     ((gint64)ROT_CMD_STUCK_TIMEOUT_MS * 1000))
                            {
                                discrepancy_hit = TRUE;
                                disc_reason = "stuck";
                            }
                        }

                        g_mutex_lock(&ctrl->client.mutex);
                        ctrl->client.last_err_to_cmd = err_deg;
                        ctrl->client.last_err_time_us = last_err_time;
                        ctrl->client.wrong_way_count = wrong_way;
                        g_mutex_unlock(&ctrl->client.mutex);
                    }
                    else
                    {
                        g_mutex_lock(&ctrl->client.mutex);
                        ctrl->client.last_err_to_cmd = 0.0;
                        ctrl->client.last_err_time_us = 0;
                        ctrl->client.wrong_way_count = 0;
                        if (!monitor_discrepancy)
                        {
                            ctrl->client.discrepancy_streak = 0;
                            ctrl->client.discrepancy_pending = FALSE;
                        }
                        g_mutex_unlock(&ctrl->client.mutex);
                    }

                    if (discrepancy_hit)
                    {
                        guint streak = 0;
                        gboolean should_log = FALSE;
                        g_mutex_lock(&ctrl->client.mutex);
                        if (ctrl->client.discrepancy_streak < G_MAXUINT)
                            ctrl->client.discrepancy_streak++;
                        ctrl->client.discrepancy_pending = TRUE;
                        streak = ctrl->client.discrepancy_streak;
                        should_log =
                            (streak >= ROT_CMD_DISCREPANCY_STRIKES) &&
                            (streak == ROT_CMD_DISCREPANCY_STRIKES ||
                             (streak % ROT_CMD_DISCREPANCY_STRIKES) == 0);
                        g_mutex_unlock(&ctrl->client.mutex);

                        if (should_log)
                        {
                            sat_log_log(SAT_LOG_LEVEL_WARN,
                                        "rot discrepancy: reason=%s streak=%u err=%.2f",
                                        disc_reason ? disc_reason : "unknown",
                                        streak,
                                        err_deg);
                            rot_term_log(ctrl, "gpredict:warn",
                                         "rot discrepancy: reason=%s streak=%u err=%.2f",
                                         disc_reason ? disc_reason : "unknown",
                                         streak,
                                         err_deg);
                        }

                    }
                    else
                    {
                        g_mutex_lock(&ctrl->client.mutex);
                        ctrl->client.discrepancy_streak = 0;
                        ctrl->client.discrepancy_pending = FALSE;
                        g_mutex_unlock(&ctrl->client.mutex);
                    }
                }
                else
                {
                    g_mutex_lock(&ctrl->client.mutex);
                    ctrl->client.discrepancy_streak = 0;
                    ctrl->client.discrepancy_pending = FALSE;
                    ctrl->client.wrong_way_count = 0;
                    ctrl->client.last_err_time_us = 0;
                    ctrl->client.last_err_to_cmd = 0.0;
                    g_mutex_unlock(&ctrl->client.mutex);
                }
            }

get_pos_done:
            if (backend_disengage)
                rot_schedule_backend_io_disengage(ctrl);
        }

        if (io_error) {
            {
                gchar reason_buf[64] = { 0 };
                g_mutex_lock(&ctrl->client.mutex);
                g_strlcpy(reason_buf, ctrl->client.io_error_reason,
                          sizeof(reason_buf));
                g_mutex_unlock(&ctrl->client.mutex);
                rot_log_rate_limited(ctrl, &reconnect_rate,
                                     ROTCTLD_FAILURE_LOG_INTERVAL_US,
                                     SAT_LOG_LEVEL_WARN, "gpredict:err",
                                     "rotctld connection reset reason=%s",
                                     reason_buf[0] ? reason_buf : "unknown");
            }
            rotctld_socket_close_quiet(ctrl, &ctrl->client.socket);
            rotctld_clear_rxbuf(ctrl);
            g_mutex_lock(&ctrl->client.mutex);
            ctrl->client.limits_valid = FALSE;
            ctrl->client.south_zero = FALSE;
            g_atomic_int_set(&ctrl->south_zero_cached, 0);
            ctrl->client.daemon_ok = FALSE;
            ctrl->client.cmd_rejected = FALSE;
            ctrl->client.reject_backoff_until_us = 0;
            ctrl->client.pos_valid = FALSE;
            ctrl->client.pos_invalid = FALSE;
            ctrl->client.pos_unknown = FALSE;
            ctrl->client.pos_cmd_ok = FALSE;
            ctrl->client.set_pos_ok = FALSE;
            ctrl->client.handshake_pos_ok = FALSE;
            ctrl->client.first_pos_deadline_us = 0;
            ctrl->client.last_pos_us = 0;
            ctrl->client.last_pos_attempt_us = 0;
            ctrl->client.last_set_attempt_us = 0;
            if (ctrl->client.io_error_reason[0] == '\0')
                g_strlcpy(ctrl->client.io_error_reason, "io error",
                          sizeof(ctrl->client.io_error_reason));
            if (ctrl->client.last_pos_error[0] == '\0')
                g_strlcpy(ctrl->client.last_pos_error, "io error",
                          sizeof(ctrl->client.last_pos_error));
            g_mutex_unlock(&ctrl->client.mutex);
        }

        /* Encoder-less mode fallback: when feedback checks are explicitly
         * disabled, mirror the commanded point as measured state.
         */
        g_mutex_lock(&ctrl->client.mutex);
        if (feedback_disabled &&
            !io_error &&
            send_cmd &&
            !ctrl->client.cmd_rejected)
        {
            ctrl->client.azi_in = rot_az_to_conf(ctrl->conf, ctrl->client.azi_out);
            ctrl->client.ele_in = ctrl->client.ele_out;
        }
        ctrl->client.io_error = io_error;
        g_mutex_unlock(&ctrl->client.mutex);

        /* keep poll cadence deterministic; do not add extra duty-cycle delay */
        elapsed_time = g_timer_elapsed(ctrl->client.timer, NULL);
        gdouble poll_sec = rotctrl_poll_period_ms(ctrl) / 1000.0;
        gdouble sleep_sec = 0.0;
        if (elapsed_time < poll_sec)
            sleep_sec = poll_sec - elapsed_time;
        rotctld_sleep_us(ctrl, (gint64)(sleep_sec * 1e6));
    }

out_stop:
    g_mutex_lock(&ctrl->client.mutex);
    ctrl->client.running = FALSE;
    g_mutex_unlock(&ctrl->client.mutex);

    sat_log_log(SAT_LOG_LEVEL_INFO,
                _("%s: stopping rotctld client thread"), __func__);
    g_timer_destroy(ctrl->client.timer);
    {
        gboolean send_quit = FALSE;

        g_mutex_lock(&ctrl->client.mutex);
        send_quit = ctrl->client.send_quit;
        g_mutex_unlock(&ctrl->client.mutex);

        if (send_quit)
            rotctld_socket_close(ctrl, &ctrl->client.socket);
        else
            rotctld_socket_close_quiet(ctrl, &ctrl->client.socket);
    }

#if defined(__APPLE__) || defined(__linux__) || defined(__FreeBSD__)
    if (c_locale) {
        uselocale(old_locale);
        freelocale(c_locale);
    }
#endif

    g_mutex_lock(&ctrl->client.mutex);
    ctrl->client.thread_done = TRUE;
    g_mutex_unlock(&ctrl->client.mutex);

    return GINT_TO_POINTER(0);
}

/**
 * Update count down label.
 *
 * \param ctrl Pointer to the RotCtrl widget.
 * \param t The current time.
 * 
 * This function calculates the new time to AOS/LOS of the currently
 * selected target and updates the ctrl->SatCnt label widget.
 */
static void update_count_down(GtkRotCtrl * ctrl, gdouble t)
{
    gdouble         targettime;
    gdouble         delta;
    gchar           buff[32];
    guint64         total_sec = 0;
    guint           h, m, s;

    if (ctrl == NULL || ctrl->SatCnt == NULL ||
        !rotctrl_target_display_valid(ctrl->target) || !isfinite(t))
    {
        gtk_label_set_text(GTK_LABEL(ctrl->SatCnt), "--:--");
        return;
    }

    /* select AOS or LOS time depending on target elevation */
    if (ctrl->target->el < 0.0)
        targettime = ctrl->target->aos;
    else
        targettime = ctrl->target->los;

    if (!isfinite(targettime))
    {
        gtk_label_set_text(GTK_LABEL(ctrl->SatCnt), "--:--");
        return;
    }

    delta = targettime - t;
    if (!isfinite(delta))
    {
        gtk_label_set_text(GTK_LABEL(ctrl->SatCnt), "--:--");
        return;
    }

    if (delta < 0.0)
        delta = 0.0;

    /* convert julian date to seconds */
    delta = delta * 86400.0;
    if (!isfinite(delta))
    {
        gtk_label_set_text(GTK_LABEL(ctrl->SatCnt), "--:--");
        return;
    }
    if (delta > 359999.0)
        delta = 359999.0;
    total_sec = (guint64) llround(delta);
    s = (guint) total_sec;

    /* extract hours */
    h = (guint) floor(s / 3600);
    s -= 3600 * h;

    /* extract minutes */
    m = (guint) floor(s / 60);
    s -= 60 * m;

    if (h > 0)
        g_snprintf(buff, sizeof(buff), "%02u:%02u:%02u", h, m, s);
    else
        g_snprintf(buff, sizeof(buff), "%02u:%02u", m, s);

    gtk_label_set_text(GTK_LABEL(ctrl->SatCnt), buff);
}

static void update_aoslos_banner(GtkRotCtrl *ctrl, gdouble t)
{
    gchar          *buff;

    if (ctrl == NULL || ctrl->aoslos_banner == NULL)
        return;

    if (!rotctrl_target_display_valid(ctrl->target) || !isfinite(t))
        buff = NULL;
    else
        buff = predict_format_aoslos_countdown(ctrl->target, t, TRUE, TRUE);
    if (buff == NULL)
        buff = g_strdup(ROTCTRL_AOSLOS_PLACEHOLDER);

    gp_safe_label_set_markup(ctrl->aoslos_banner, buff);
    g_free(buff);
}

static void rotctrl_finish_tracking_pass_over(GtkRotCtrl *ctrl,
                                              const gchar *reason)
{
    const gchar *why = (reason && *reason) ? reason : "pass over";
    RotorStateSnapshot snap = { 0 };
    gboolean locked = FALSE;

    if (ctrl == NULL || !ctrl->tracking)
        return;

    sat_log_log(SAT_LOG_LEVEL_INFO,
                "tracking complete: %s; waiting for user input",
                why);
    rot_term_log(ctrl, "gpredict:state",
                 "tracking complete: %s; waiting for user input",
                 why);

    if (ctrl->track != NULL)
    {
        rotctrl_ui_begin_update(ctrl, "pass_over_track_off");
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->track), FALSE);
        rotctrl_ui_end_update(ctrl, "pass_over_track_off");
    }

    if (ctrl->LockBut != NULL)
        locked = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ctrl->LockBut));
    if (ctrl->MonitorCheckBox != NULL)
        gtk_widget_set_sensitive(ctrl->MonitorCheckBox, !locked);
    if (ctrl->AzSet != NULL)
        gtk_widget_set_sensitive(ctrl->AzSet, TRUE);
    if (ctrl->ElSet != NULL)
        gtk_widget_set_sensitive(ctrl->ElSet, TRUE);

    ctrl->tracking = FALSE;
    ctrl->tracking_active = FALSE;
    ctrl->tracking_session_los = 0.0;
    ctrl->target_state = ROT_TARGET_STATE_IDLE;
    ctrl->target_state_since_us = 0;
    ctrl->target_valid_since_us = 0;
    ctrl->target_invalid_since_us = 0;
    ctrl->setpoint_valid = FALSE;
    ctrl->last_target_update_us = 0;
    ctrl->last_send_us = 0;
    ctrl->above_eps_count = 0;
    ctrl->motion_err_valid = FALSE;
    ctrl->motion_stall_count = 0;
    rot_plan_reset(&ctrl->trajectory_plan);
    set_flipped_pass(ctrl);

    ctrl->have_user_command = FALSE;
    ctrl->hold_position_on_engage = TRUE;
    ctrl->hold_position_log_emitted = FALSE;
    ctrl->force_next_send = FALSE;
    ctrl->park_requested = FALSE;
    ctrl->stale_hold_active = FALSE;
    ctrl->pretrack_target_valid = FALSE;
    ctrl->pretrack_last_update_us = 0;
    ctrl->pretrack_aos_time = 0.0;
    ctrl->pretrack_wait_log_us = 0;
    ctrl->pretrack_wrap_valid = FALSE;
    ctrl->pretrack_wrap_user_az = 0.0;
    ctrl->pretrack_wrap_raw_az = 0.0;
    ctrl->pretrack_wrap_k = 0;
    rotctrl_tracking_policy_reset_reason(ctrl, "pass_over");

    snap.control_active = ctrl->engaged || ctrl->engage_pending || ctrl->ui_hard_error;
    snap.engaging = ctrl->engage_pending;
    snap.hard_error = ctrl->ui_hard_error;
    rotctrl_set_ui_status_detail(ctrl, "STANDBY");
    rotctrl_apply_ui_status(ctrl, &snap, "pass over");
}

/*
 * Update rotator control state.
 * 
 * This function is called by the parent, i.e. GtkSatModule, indicating that
 * the satellite data has been updated. The function updates the internal state
 * of the controller and the rotator.
 */
void gtk_rot_ctrl_update(GtkRotCtrl * ctrl, gdouble t)
{
    gchar          *buff;
    gboolean        target_valid = FALSE;

    ctrl->t = t;
    if (ctrl->target)
        ctrl->last_target_update_us = g_get_monotonic_time();

    if (ctrl->target)
    {
        if (!rotctrl_sync_sat_copy_from_module(ctrl, ctrl->target) &&
            ctrl->qth != NULL)
            predict_calc(ctrl->target, ctrl->qth, t);

        target_valid = rotctrl_target_display_valid(ctrl->target);
        if (!target_valid)
        {
            gtk_label_set_text(GTK_LABEL(ctrl->AzSat), " --- ");
            gtk_label_set_text(GTK_LABEL(ctrl->ElSat), " --- ");
            update_count_down(ctrl, t);
            update_aoslos_banner(ctrl, t);
            rotctrl_update_detached_labels(ctrl);
            return;
        }

        /* update target displays */
        buff = g_strdup_printf(FMTSTR, ctrl->target->az);
        gtk_label_set_text(GTK_LABEL(ctrl->AzSat), buff);
        g_free(buff);
        buff = g_strdup_printf(FMTSTR, ctrl->target->el);
        gtk_label_set_text(GTK_LABEL(ctrl->ElSat), buff);
        g_free(buff);

        update_count_down(ctrl, t);
        update_aoslos_banner(ctrl, t);
        rotctrl_update_detached_labels(ctrl);

        if (ctrl->tracking &&
            ctrl->tracking_session_los > 0.0 &&
            isfinite(ctrl->tracking_session_los) &&
            t >= ctrl->tracking_session_los)
        {
            rotctrl_finish_tracking_pass_over(ctrl, "pass LOS reached");
        }

        /*if the current pass is too far away */
        if ((ctrl->pass != NULL))
            if (qth_small_dist(ctrl->qth, ctrl->pass->qth_comp) > 1.0)
            {
                rot_plan_reset(&ctrl->trajectory_plan);
                free_pass(ctrl->pass);
                ctrl->pass = NULL;
                ctrl->pass = get_pass_no_min_el(ctrl->target, ctrl->qth, t, 3.0);
                if (ctrl->pass)
                {
                    set_flipped_pass(ctrl);
                    /* update polar plot */
                    rotctrl_set_pass_on_plots(ctrl, ctrl->pass);
                }
            }

        /* update next pass if necessary */
        if (ctrl->pass != NULL)
        {
            /* if we are not in the current pass */
            if ((ctrl->pass->aos > t) || (ctrl->pass->los < t))
            {
                /* the pass may not have met the minimum 
                   elevation, calculate the pass and plot it */
                if (ctrl->target->el >= 0.0)
                {
                    /* inside an unexpected/unpredicted pass */
                    rot_plan_reset(&ctrl->trajectory_plan);
                    free_pass(ctrl->pass);
                    ctrl->pass = NULL;
                    ctrl->pass = get_pass_no_min_el(ctrl->target, ctrl->qth, t, 3.0);
                    set_flipped_pass(ctrl);
                    rotctrl_set_pass_on_plots(ctrl, ctrl->pass);
                }
                else if (!isfinite(ctrl->pass->aos) ||
                         !isfinite(ctrl->pass->los) ||
                         (ctrl->pass->los < t))
                {
                    if (ctrl->tracking)
                        rotctrl_finish_tracking_pass_over(ctrl,
                                                          "pass ended (rollover)");

                    /* pass is stale/invalid; fetch the next upcoming pass */
                    rot_plan_reset(&ctrl->trajectory_plan);
                    free_pass(ctrl->pass);
                    ctrl->pass = NULL;
                    ctrl->pass = get_pass_no_min_el(ctrl->target, ctrl->qth, t, 3.0);
                    set_flipped_pass(ctrl);
                    /* update polar plot */
                    rotctrl_set_pass_on_plots(ctrl, ctrl->pass);
                }
            }
            else
            {
                /* inside a pass and target dropped below the 
                   horizon so look for a new pass */
                if (ctrl->target->el < 0.0)
                {
                    if (ctrl->tracking)
                        rotctrl_finish_tracking_pass_over(ctrl,
                                                          "satellite below horizon");

                    rot_plan_reset(&ctrl->trajectory_plan);
                    free_pass(ctrl->pass);
                    ctrl->pass = NULL;
                    ctrl->pass = get_pass_no_min_el(ctrl->target, ctrl->qth, t, 3.0);
                    set_flipped_pass(ctrl);
                    /* update polar plot */
                    rotctrl_set_pass_on_plots(ctrl, ctrl->pass);
                }
            }
        }
        else
        {
            /* we don't have any current pass; store the current one */
            rot_plan_reset(&ctrl->trajectory_plan);
            ctrl->pass = get_pass_no_min_el(ctrl->target, ctrl->qth, t, 3.0);

            set_flipped_pass(ctrl);
            /* update polar plot */
            rotctrl_set_pass_on_plots(ctrl, ctrl->pass);
        }
    }
    else
    {
        update_aoslos_banner(ctrl, t);
        rotctrl_update_detached_labels(ctrl);
    }
}

/* Select a satellite. */
void gtk_rot_ctrl_select_sat(GtkRotCtrl * ctrl, gint catnum)
{
    sat_t          *sat;
    int             i, n;

    /* find index in satellite list */
    n = g_slist_length(ctrl->sats);
    for (i = 0; i < n; i++)
    {
        sat = SAT(g_slist_nth_data(ctrl->sats, i));
        if (sat && sat->tle.catnr == catnum)
        {
            /* assume the index is the same in sat selector */
            rotctrl_combo_set_active_safe(ctrl,
                                          GTK_COMBO_BOX(ctrl->SatSel),
                                          i,
                                          G_CALLBACK(sat_selected_cb));
            (void)rotctrl_apply_sat_selection(ctrl, i, "programmatic");
            break;
        }
    }
}

void gtk_rot_ctrl_request_close(GtkRotCtrl *ctrl)
{
    if (!GTK_IS_ROT_CTRL(ctrl))
        return;

    /* Ask the worker to stop; caller can poll gtk_rot_ctrl_can_destroy(). */
    rotctld_request_thread_stop(ctrl, TRUE);
}

gboolean gtk_rot_ctrl_can_destroy(GtkRotCtrl *ctrl)
{
    if (!GTK_IS_ROT_CTRL(ctrl))
        return TRUE;

    if (ctrl->client.thread == NULL)
        return TRUE;

    return rotctld_collect_client_thread(ctrl, FALSE, "close-poll");
}

/*
 * Create azimuth control widgets.
 * 
 * This function creates and initialises the widgets for controlling the
 * azimuth of the the rotator.
 */
static GtkWidget *create_az_widgets(GtkRotCtrl * ctrl)
{
    GtkWidget      *frame;
    GtkWidget      *table;
    GtkWidget      *label;

    frame = gtk_frame_new(_("Azimuth"));

    table = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(table), 5);
    gtk_grid_set_column_spacing(GTK_GRID(table), 5);
    gtk_grid_set_row_spacing(GTK_GRID(table), 5);
    gtk_widget_set_halign(table, GTK_ALIGN_CENTER);
    gtk_container_add(GTK_CONTAINER(frame), table);

    ctrl->AzSet = gtk_rot_knob_new(0.0, 360.0, 180.0);
    gtk_widget_add_events(ctrl->AzSet,
                          GDK_BUTTON_PRESS_MASK | GDK_SCROLL_MASK);
    g_signal_connect(ctrl->AzSet, "button-press-event",
                     G_CALLBACK(rot_manual_input_event), ctrl);
    g_signal_connect(ctrl->AzSet, "scroll-event",
                     G_CALLBACK(rot_manual_input_event), ctrl);
    gtk_grid_attach(GTK_GRID(table), ctrl->AzSet, 0, 0, 3, 1);

    label = gtk_label_new(NULL);
    gp_safe_label_set_markup(label, _("Read:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 1, 1, 1);

    ctrl->AzRead = gtk_label_new(" --- ");
    g_object_set(ctrl->AzRead, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), ctrl->AzRead, 1, 1, 1, 1);

    return frame;
}

/*
 * Create elevation control widgets.
 * 
 * This function creates and initialises the widgets for controlling the
 * elevation of the the rotator.
 */
static GtkWidget *create_el_widgets(GtkRotCtrl * ctrl)
{
    GtkWidget      *frame;
    GtkWidget      *table;
    GtkWidget      *label;

    frame = gtk_frame_new(_("Elevation"));

    table = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(table), 5);
    gtk_grid_set_column_spacing(GTK_GRID(table), 5);
    gtk_grid_set_row_spacing(GTK_GRID(table), 5);
    gtk_widget_set_halign(table, GTK_ALIGN_CENTER);
    gtk_container_add(GTK_CONTAINER(frame), table);

    ctrl->ElSet = gtk_rot_knob_new(0.0, 90.0, 45.0);
    gtk_widget_add_events(ctrl->ElSet,
                          GDK_BUTTON_PRESS_MASK | GDK_SCROLL_MASK);
    g_signal_connect(ctrl->ElSet, "button-press-event",
                     G_CALLBACK(rot_manual_input_event), ctrl);
    g_signal_connect(ctrl->ElSet, "scroll-event",
                     G_CALLBACK(rot_manual_input_event), ctrl);
    gtk_grid_attach(GTK_GRID(table), ctrl->ElSet, 0, 0, 3, 1);

    label = gtk_label_new(NULL);
    gp_safe_label_set_markup(label, _("Read: "));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 1, 1, 1);

    ctrl->ElRead = gtk_label_new(" --- ");
    g_object_set(ctrl->ElRead, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), ctrl->ElRead, 1, 1, 1, 1);

    return frame;
}

/**
 * Manage toggle signals (tracking)
 *
 * \param button Pointer to the GtkToggle button.
 * \param data Pointer to the GtkRotCtrl widget.
 */
static void track_toggle_cb(GtkToggleButton * button, gpointer data)
{
    GtkRotCtrl     *ctrl = GTK_ROT_CTRL(data);
    gboolean        locked;
    gboolean        pos_recent = FALSE;
    gboolean        session_ready = FALSE;
    gboolean        requested;

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    if (ctrl->cal_hold_active)
        rotctrl_set_cal_hold(ctrl, FALSE, "track_toggle");

    locked = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ctrl->LockBut));
    requested = gtk_toggle_button_get_active(button);
    if (requested && !rotor_apply_ui_settings(ctrl, TRUE))
    {
        rotctrl_ui_begin_update(ctrl, "track_invalid_settings");
        gtk_toggle_button_set_active(button, FALSE);
        rotctrl_ui_end_update(ctrl, "track_invalid_settings");
        return;
    }

    ctrl->tracking = requested;
    if (requested)
        ctrl->hold_position_on_engage = FALSE;
    gtk_widget_set_sensitive(ctrl->MonitorCheckBox,
                             !(ctrl->tracking || locked));
    gtk_widget_set_sensitive(ctrl->AzSet, !ctrl->tracking);
    gtk_widget_set_sensitive(ctrl->ElSet, !ctrl->tracking);

    if (ctrl->tracking)
    {
        ctrl->force_next_send = TRUE;
        ctrl->setpoint_valid = FALSE;
        ctrl->setpoint_user_az = 0.0;
        ctrl->setpoint_user_el = 0.0;
        ctrl->setpoint_backend_az = 0.0;
        ctrl->setpoint_backend_el = 0.0;
    }

    if (!ctrl->tracking)
    {
        rot_plan_reset(&ctrl->trajectory_plan);
        set_flipped_pass(ctrl);
        ctrl->tracking_active = FALSE;
        ctrl->tracking_session_los = 0.0;
        ctrl->target_state = ROT_TARGET_STATE_IDLE;
        ctrl->target_state_since_us = 0;
        ctrl->target_valid_since_us = 0;
        ctrl->target_invalid_since_us = 0;
        ctrl->last_target_update_us = 0;
        ctrl->last_send_us = 0;
        ctrl->above_eps_count = 0;
        ctrl->park_requested = FALSE;
        ctrl->pretrack_target_valid = FALSE;
        ctrl->pretrack_last_update_us = 0;
        ctrl->pretrack_aos_time = 0.0;
        ctrl->pretrack_wait_log_us = 0;
        ctrl->pretrack_wrap_valid = FALSE;
        ctrl->pretrack_wrap_user_az = 0.0;
        ctrl->pretrack_wrap_raw_az = 0.0;
        ctrl->pretrack_wrap_k = 0;
        ctrl->seam_valid = FALSE;
        ctrl->seam_crossing_active = FALSE;
        ctrl->seam_crossing_sent = FALSE;
        ctrl->seam_crossing_lane_valid = FALSE;
        ctrl->seam_crossing_lane_k = 0;
        ctrl->seam_crossing_target_az360 = 0.0;
        ctrl->seam_crossing_since_us = 0;
        ctrl->wrap_acquire_active = FALSE;
        ctrl->wrap_acquire_sent = FALSE;
        ctrl->wrap_acquire_target_backend = 0.0;
        ctrl->wrap_acquire_target_az360 = 0.0;
        ctrl->wrap_acquire_target_k = 0;
        ctrl->wrap_acquire_since_us = 0;
        ctrl->wrap_acquire_last_log_us = 0;
        ctrl->pending_lane_valid = FALSE;
        ctrl->pending_lane_since_us = 0;
        ctrl->locked_lane_valid = FALSE;
        ctrl->last_target_valid = FALSE;
        ctrl->last_cmd_backend_az = 0.0;
        ctrl->last_cmd_backend_el = 0.0;
        ctrl->last_cmd_backend_valid = FALSE;
        ctrl->setpoint_user_az = 0.0;
        ctrl->setpoint_user_el = 0.0;
        ctrl->setpoint_backend_az = 0.0;
        ctrl->setpoint_backend_el = 0.0;
        ctrl->setpoint_valid = FALSE;
        ctrl->force_next_send = FALSE;
        ctrl->committed_user_az = 0.0;
        ctrl->committed_user_el = 0.0;
        ctrl->committed_raw_az360 = 0.0;
        ctrl->committed_raw_el = 0.0;
        ctrl->committed_backend_az = 0.0;
        ctrl->committed_backend_el = 0.0;
        ctrl->committed_valid = FALSE;
        ctrl->committed_since_us = 0;
        ctrl->last_keepalive_time_us = 0;
        ctrl->last_desired_update_us = 0;
        ctrl->pos_stale_active = FALSE;
        ctrl->last_stale_check_log_us = 0;
        ctrl->pos_stale_hyst_active = FALSE;
        ctrl->pos_stale_ready_hits = 0;
        ctrl->stale_hold_active = FALSE;
        ctrl->stale_hold_since_us = 0;
        ctrl->stale_resume_since_us = 0;
        ctrl->stale_recovered_pulse = FALSE;
        ctrl->motion_err_valid = FALSE;
        ctrl->motion_stall_count = 0;
        rotctrl_tracking_policy_reset_reason(ctrl, "track_off");
        return;
    }

    if (ctrl->tracking && ctrl->target && ctrl->qth)
    {
        if (ctrl->pass != NULL)
            free_pass(ctrl->pass);

        ctrl->pass = get_pass_no_min_el(ctrl->target, ctrl->qth, ctrl->t, 3.0);
        ctrl->tracking_session_los =
            (ctrl->pass != NULL && isfinite(ctrl->pass->los))
                ? ctrl->pass->los
                : 0.0;

        rot_plan_reset(&ctrl->trajectory_plan);
        rotctrl_tracking_policy_reset_reason(ctrl, "track_on");
        ctrl->pretrack_target_valid = FALSE;
        ctrl->pretrack_last_update_us = 0;
        ctrl->pretrack_aos_time = 0.0;
        ctrl->pretrack_wait_log_us = 0;
        ctrl->pretrack_wrap_valid = FALSE;
        ctrl->pretrack_wrap_user_az = 0.0;
        ctrl->pretrack_wrap_raw_az = 0.0;
        ctrl->pretrack_wrap_k = 0;
        ctrl->seam_valid = FALSE;
        ctrl->seam_crossing_active = FALSE;
        ctrl->seam_crossing_sent = FALSE;
        ctrl->seam_crossing_lane_valid = FALSE;
        ctrl->seam_crossing_lane_k = 0;
        ctrl->seam_crossing_target_az360 = 0.0;
        ctrl->seam_crossing_since_us = 0;
        ctrl->wrap_acquire_active = FALSE;
        ctrl->wrap_acquire_sent = FALSE;
        ctrl->wrap_acquire_target_backend = 0.0;
        ctrl->wrap_acquire_target_az360 = 0.0;
        ctrl->wrap_acquire_target_k = 0;
        ctrl->wrap_acquire_since_us = 0;
        ctrl->wrap_acquire_last_log_us = 0;
        ctrl->pending_lane_valid = FALSE;
        ctrl->pending_lane_since_us = 0;
        ctrl->locked_lane_valid = FALSE;
        ctrl->last_target_valid = FALSE;
        ctrl->last_cmd_backend_az = 0.0;
        ctrl->last_cmd_backend_el = 0.0;
        ctrl->last_cmd_backend_valid = FALSE;
        ctrl->committed_user_az = 0.0;
        ctrl->committed_user_el = 0.0;
        ctrl->committed_raw_az360 = 0.0;
        ctrl->committed_raw_el = 0.0;
        ctrl->committed_backend_az = 0.0;
        ctrl->committed_backend_el = 0.0;
        ctrl->committed_valid = FALSE;
        ctrl->committed_since_us = 0;
        ctrl->last_keepalive_time_us = 0;
        ctrl->last_desired_update_us = 0;

        pos_recent = rotctrl_pos_recent(ctrl,
                                        (gint64)rotctrl_stale_ms(ctrl) * 1000,
                                        NULL);
        session_ready = rotctrl_session_ready(ctrl, pos_recent);

        if (!session_ready)
        {
            set_flipped_pass(ctrl);
            if (ctrl->plot != NULL)
                rotctrl_set_pass_on_plots(ctrl, ctrl->pass);
            return;
        }

        if (!rot_build_tracking_plan(ctrl)) {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        "%s: unable to build trajectory – %s",
                        __func__,
                        ctrl->trajectory_plan.reason ? ctrl->trajectory_plan.reason : "trajectory planning failed");
            rot_show_plan_error(ctrl, ctrl->trajectory_plan.reason);
            gtk_toggle_button_set_active(button, FALSE);
            ctrl->tracking = FALSE;
            ctrl->tracking_session_los = 0.0;
            set_flipped_pass(ctrl);
            rotctrl_tracking_policy_reset_reason(ctrl, "track_off_no_session");
            return;
        }

        ctrl->flipped = (ctrl->trajectory_plan.mode == ROT_PLAN_MODE_FLIP);
        set_flipped_pass(ctrl);
        if (ctrl->plot != NULL)
            rotctrl_set_pass_on_plots(ctrl, ctrl->pass);

    }
    else if (ctrl->tracking) {
        rot_plan_reset(&ctrl->trajectory_plan);
        ctrl->tracking_session_los = 0.0;
        set_flipped_pass(ctrl);
        rotctrl_tracking_policy_reset_reason(ctrl, "track_off");
    }
}

static void freeze_clicked_cb(GtkButton *button, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);
    gdouble freeze_az = 0.0;
    gdouble freeze_el = 0.0;
    gboolean have_pos = FALSE;
    gboolean fresh_pos = FALSE;
    gint wait_ms = ROT_FREEZE_MIN_WAIT_MS;

    (void)button;

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    if (ctrl->cal_hold_active)
        rotctrl_set_cal_hold(ctrl, FALSE, "freeze");

    if (ctrl->tracking && ctrl->track)
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->track), FALSE);

    /* Latch freeze regardless of UI signal delivery order. */
    ctrl->tracking = FALSE;
    ctrl->tracking_active = FALSE;
    ctrl->tracking_session_los = 0.0;
    ctrl->target_state = ROT_TARGET_STATE_IDLE;
    ctrl->target_state_since_us = 0;
    ctrl->target_valid_since_us = 0;
    ctrl->target_invalid_since_us = 0;
    rot_plan_reset(&ctrl->trajectory_plan);
    set_flipped_pass(ctrl);
    rotctrl_tracking_policy_reset_reason(ctrl, "freeze");
    ctrl->pretrack_target_valid = FALSE;
    ctrl->pretrack_aos_time = 0.0;
    ctrl->pretrack_wait_log_us = 0;
    ctrl->pretrack_wrap_valid = FALSE;
    ctrl->pretrack_wrap_user_az = 0.0;
    ctrl->pretrack_wrap_raw_az = 0.0;
    ctrl->pretrack_wrap_k = 0;

    if (ctrl->engaged && ctrl->client.running)
    {
        g_mutex_lock(&ctrl->client.mutex);
        ctrl->client.stop_pending = TRUE;
        ctrl->client.new_trg = FALSE;
        ctrl->client.force_pending = FALSE;
        ctrl->client.desired_valid = FALSE;
        ctrl->client.desired_allow = FALSE;
        ctrl->client.desired_tracking = FALSE;
        g_mutex_unlock(&ctrl->client.mutex);
    }

    wait_ms = MAX(ROT_FREEZE_MIN_WAIT_MS, rotctrl_poll_period_ms(ctrl) * 2);
    have_pos = rotctrl_refresh_user_position(ctrl,
                                             wait_ms,
                                             &freeze_az,
                                             &freeze_el,
                                             &fresh_pos);

    if (!have_pos)
    {
        ctrl->manual_edit_until_us = 0;
        ctrl->manual_sync_pending = TRUE;
        ctrl->have_user_command = FALSE;
        ctrl->hold_position_on_engage = TRUE;
        ctrl->hold_position_log_emitted = FALSE;
        ctrl->park_requested = FALSE;
        ctrl->setpoint_valid = FALSE;
        ctrl->force_next_send = FALSE;
        sat_log_log(SAT_LOG_LEVEL_WARN,
                    "freeze: no known rotor position; stop latched");
        rot_term_log(ctrl, "gpredict:warn",
                     "freeze: no known rotor position; stop latched");
        return;
    }

    if (ctrl->conf && ctrl->conf->axis_mode == ROT_AXIS_MODE_AZ_ONLY)
        freeze_el = ctrl->conf->minel;

    gtk_rot_knob_set_value(GTK_ROT_KNOB(ctrl->AzSet), freeze_az);
    gtk_rot_knob_set_value(GTK_ROT_KNOB(ctrl->ElSet), freeze_el);
    /* Freeze is not a manual move request; keep hold engaged after STOP. */
    ctrl->manual_edit_until_us = 0;
    ctrl->manual_sync_pending = FALSE;
    ctrl->have_user_command = FALSE;
    ctrl->hold_position_on_engage = TRUE;
    ctrl->hold_position_log_emitted = FALSE;
    ctrl->park_requested = FALSE;
    ctrl->setpoint_valid = FALSE;
    ctrl->force_next_send = FALSE;

    if (have_pos)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "freeze: stop latched at az=%.2f el=%.2f source=%s",
                    freeze_az,
                    freeze_el,
                    fresh_pos ? "live" : "last_known");
        rot_term_log(ctrl, "gpredict:state",
                     "freeze: stop latched at az=%.2f el=%.2f source=%s",
                     freeze_az,
                     freeze_el,
                     fresh_pos ? "live" : "last_known");
    }
}

/**
 * Rotator controller timeout function
 *
 * \param data Pointer to the GtkRotCtrl widget.
 * \return Always TRUE to let the timer continue.
 */
static gboolean rot_ctrl_timeout_cb(gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);
    gdouble rotaz = 0.0, rotel = 0.0;
    gdouble rotaz_backend = 0.0, rotel_backend = 0.0;
    gdouble setaz = 0.0, setel = 45.0;
    gdouble target_az360 = 0.0, target_el = 0.0;
    gdouble target_cmd_az360 = 0.0, target_cmd_el = 0.0;
    gdouble display_az360 = 0.0, display_el = 0.0;
    RotTargetOut xform = { 0 };
    gdouble cmdaz = 0.0, cmdel = 0.0;
    gdouble cmdaz_plot = 0.0, cmdel_plot = 0.0;
    gchar *text;
    gboolean error = FALSE;
    gboolean cmd_rejected = FALSE;
    gboolean rotpos_valid = FALSE;
    gboolean pos_unknown = FALSE;
    gboolean pos_cmd_ok = FALSE;
    gboolean pos_send_ok = FALSE;
    gboolean pos_recent = FALSE;
    gint64 last_pos_us = 0;
    gint64 plot_pos_age_ms = -1;
    gboolean keep_rotor_plot = FALSE;
    guint pos_failures = 0;
    rot_comm_status_t comm_status = ROT_COMM_DISENGAGED;
    gboolean az_clamped = FALSE;
    gboolean el_clamped = FALSE;
    gboolean safety_clamped = FALSE;
    gboolean safety_avoided = FALSE;
    gdouble az_abs_cmd = 0.0;
    gdouble az_pred = 0.0;
    gdouble el_pred = 0.0;
    gdouble live_az360 = 0.0;
    gdouble live_el = 0.0;
    gboolean live_valid = FALSE;
    gdouble setpoint_az360 = 0.0;
    gdouble setpoint_el = 0.0;
    gdouble setpoint_user_az = 0.0;
    gdouble setpoint_user_el = 0.0;
    gboolean pred_valid = FALSE;
    gdouble meas_az360 = 0.0;
    gdouble meas_el = 0.0;
    gdouble last_cmd_backend = 0.0;
    gdouble last_cmd_ok_az = 0.0;
    gdouble last_cmd_az360 = 0.0;
    gdouble last_cmd_el = 0.0;
    gboolean have_last_cmd_backend = FALSE;
    gboolean have_last_cmd_phys = FALSE;
    gboolean below_horizon = FALSE;
    gboolean pretrack_active = FALSE;
    gboolean hold_below = FALSE;
    gboolean hold_no_target = FALSE;
    gboolean seam_cross_active = FALSE;
    gboolean seam_cmd_valid = FALSE;
    gdouble seam_cmd_az = 0.0;
    gint seam_lane_k = 0;
    gboolean wrap_mismatch = FALSE;
    gboolean wrap_resolve_failed = FALSE;
    rot_target_invalid_reason_t wrap_reason = ROT_TARGET_INVALID_NONE;
    gboolean wrap_bypass = FALSE;
    gdouble wrap_candidate_a = NAN;
    gdouble wrap_candidate_b = NAN;
    gdouble wrap_candidate = 0.0;
    gint wrap_candidate_k = 0;
    gboolean state_changed = FALSE;
    rot_plan_mode_t tracking_mode = ROT_PLAN_MODE_NORMAL;
    rot_ui_mode_t ui_mode = ROT_UI_360;
    gdouble elev_floor = 0.0;
    SpanConfig user_span_cfg = { 0 };
    SpanConfig backend_span_cfg = { 0 };
    rot_target_caps_t decision_caps = { 0 };
    gboolean user_span_extended = FALSE;
    gboolean backend_span_extended = FALSE;
    AzSpan backend_span_mode = AZSPAN_360;
    gboolean caps_valid = FALSE;
    gboolean use_caps = FALSE;
    gdouble caps_az_min = 0.0;
    gdouble caps_az_max = 0.0;
    gdouble caps_el_min = 0.0;
    gdouble caps_el_max = 0.0;
    gboolean caps_south_zero = FALSE;
    gdouble backend_az_min = 0.0;
    gdouble backend_az_max = 360.0;
    gdouble backend_el_min = ROTCTRL_DEFAULT_MIN_EL;
    gdouble backend_el_max = ROTCTRL_DEFAULT_MAX_EL;
    GtkWidget *status_label =
        g_object_get_data(G_OBJECT(ctrl), "rot-status-label");
    gboolean plan_active = rot_plan_matches_pass(ctrl);
    gboolean session_ready = FALSE;
    gchar last_pos_error[64] = { 0 };
        gboolean autocal_active = FALSE;
        gboolean autocal_direct = FALSE;
    gboolean cal_hold_active = FALSE;
    gboolean cal_force_send = FALSE;
    gboolean park_active = FALSE;

    pos_recent = rotctrl_pos_recent(ctrl,
                                    (gint64)rotctrl_stale_ms(ctrl) * 1000,
                                    &last_pos_us);
    session_ready = rotctrl_session_ready(ctrl, pos_recent);
    tracking_mode = plan_active
                    ? ctrl->trajectory_plan.mode
                    : (ctrl->flipped ? ROT_PLAN_MODE_FLIP : ROT_PLAN_MODE_NORMAL);
    if (ctrl->conf && ctrl->conf->maxel < 180.0)
        tracking_mode = ROT_PLAN_MODE_NORMAL;
    ctrl->out_of_range = FALSE;
    ctrl->tracking_active = FALSE;
    if (!session_ready)
        plan_active = FALSE;

    if (ctrl->tracking &&
        ctrl->tracking_session_los > 0.0 &&
        isfinite(ctrl->tracking_session_los) &&
        ctrl->t >= ctrl->tracking_session_los)
    {
        rotctrl_finish_tracking_pass_over(ctrl, "pass LOS reached");
        return TRUE;
    }

    if (ctrl->client.thread != NULL)
    {
        gboolean thread_done = FALSE;

        g_mutex_lock(&ctrl->client.mutex);
        thread_done = ctrl->client.thread_done;
        g_mutex_unlock(&ctrl->client.mutex);

        if (thread_done)
        {
            if (rotctld_collect_client_thread(ctrl, FALSE, "poll"))
            {
                sat_log_log(SAT_LOG_LEVEL_INFO,
                            "rotctld client thread joined");
                if (ctrl->engage_pending)
                {
                    rotctld_finish_engage(ctrl);
                    return TRUE;
                }
            }
        }
    }

    if (session_ready && ctrl->tracking && ctrl->pass && !plan_active) {
        rot_build_tracking_plan(ctrl);
        plan_active = rot_plan_matches_pass(ctrl);
        set_flipped_pass(ctrl);
        tracking_mode = plan_active
                        ? ctrl->trajectory_plan.mode
                        : (ctrl->flipped ? ROT_PLAN_MODE_FLIP : ROT_PLAN_MODE_NORMAL);
        if (ctrl->conf && ctrl->conf->maxel < 180.0)
            tracking_mode = ROT_PLAN_MODE_NORMAL;
    }

    if (session_ready && ctrl->tracking && ctrl->pass && plan_active)
        rotctrl_plan_log_if_pending(ctrl);

    ui_mode = (ctrl->conf && ctrl->conf->aztype == ROT_AZ_TYPE_180)
              ? ROT_UI_NORTH_CENTERED
              : ROT_UI_360;
    elev_floor = (ctrl->conf != NULL)
                     ? ctrl->conf->minel
                     : ROTCTRL_DEFAULT_MIN_EL;

    if (ctrl->tracking && ctrl->target && ctrl->conf)
    {
        gint64 now_us = g_get_monotonic_time();
        gdouble pretrack_window =
            (ctrl->pretrack_lookahead_sec > 0.0)
                ? ctrl->pretrack_lookahead_sec
                : ROT_PRETRACK_LOOKAHEAD_SEC;
        if (ctrl->pretrack_immediate)
            pretrack_window = MAX(pretrack_window, ROT_PRETRACK_LOOKAHEAD_SEC);
        gdouble aos_time = 0.0;
        gdouble aos_az360 = 0.0;
        gdouble aos_el = 0.0;
        gboolean aos_found = FALSE;

        live_valid = rotctrl_predict_at(ctrl,
                                        ctrl->t,
                                        &live_az360,
                                        &live_el);
        if (live_valid)
        {
            az_pred = live_az360;
            el_pred = live_el;
        }

        pred_valid = FALSE;
        {
            gdouble horizon_margin =
                ctrl->pretrack_target_valid ? ROT_PRETRACK_HYSTERESIS_DEG : 0.0;
            below_horizon = live_valid &&
                            (live_el < (elev_floor + horizon_margin));
        }
        if (below_horizon &&
            ctrl->pass != NULL &&
            isfinite(ctrl->pass->los) &&
            ctrl->t >= ctrl->pass->los)
        {
            rotctrl_finish_tracking_pass_over(ctrl, "LOS reached");
            return TRUE;
        }
        pretrack_active = FALSE;
        hold_below = FALSE;
        hold_no_target = FALSE;

        if (!live_valid)
        {
            sat_log_log(SAT_LOG_LEVEL_WARN,
                        "trk: predictor invalid; holding position");
            hold_below = TRUE;
        }

        if (below_horizon && ctrl->pretrack_enabled)
        {
            if (!ctrl->pretrack_target_valid)
            {
                aos_found = rotctrl_find_next_aos(ctrl,
                                                  ctrl->t,
                                                  pretrack_window,
                                                  elev_floor,
                                                  ROT_PRETRACK_AOS_OFFSET_SEC,
                                                  &aos_time,
                                                  &aos_az360,
                                                  &aos_el);

                if (!aos_found &&
                    ctrl->pass != NULL &&
                    isfinite(ctrl->pass->aos) &&
                    ctrl->pass->aos >= ctrl->t)
                {
                    gdouble pass_aos = ctrl->pass->aos;
                    gdouble pass_lead = pass_aos +
                                        (ROT_PRETRACK_AOS_OFFSET_SEC / secday);

                    if (!rotctrl_predict_at(ctrl, pass_lead, &aos_az360, &aos_el))
                        rotctrl_predict_at(ctrl, pass_aos, &aos_az360, &aos_el);

                    if (isfinite(aos_az360) && isfinite(aos_el))
                    {
                        aos_time = pass_aos;
                        aos_found = TRUE;
                        if (ctrl->verbose_logging)
                        {
                            gchar aos_buf[64] = { 0 };
                            const gchar *aos_str = "unknown";

                            rotctrl_format_utc_jd(pass_aos,
                                                  aos_buf,
                                                  sizeof(aos_buf));
                            if (aos_buf[0] != '\0')
                                aos_str = aos_buf;
                            sat_log_log(SAT_LOG_LEVEL_INFO,
                                        "pretrack aos(pass)=%s az=%.2f el=%.2f lead=%.1fs",
                                        aos_str,
                                        aos_az360,
                                        aos_el,
                                        ROT_PRETRACK_AOS_OFFSET_SEC);
                        }
                    }
                }

                if (aos_found)
                {
                    gdouble pretrack_el = MAX(ctrl->pretrack_min_el, elev_floor);
                    if (ctrl->conf != NULL)
                        pretrack_el = CLAMP(pretrack_el,
                                            ctrl->conf->minel,
                                            ctrl->conf->maxel);
                    ctrl->pretrack_target_az = aos_az360;
                    ctrl->pretrack_target_el = pretrack_el;
                    ctrl->pretrack_aos_time = aos_time;
                    ctrl->pretrack_target_valid = TRUE;
                    ctrl->pretrack_last_update_us = now_us;
                    ctrl->last_desired_update_us = now_us;
                    ctrl->force_next_send = TRUE;
                    if (ctrl->verbose_logging)
                    {
                        gchar aos_buf[64] = { 0 };
                        const gchar *aos_str = "unknown";

                        rotctrl_format_utc_jd(aos_time, aos_buf, sizeof(aos_buf));
                        if (aos_buf[0] != '\0')
                            aos_str = aos_buf;
                        sat_log_log(SAT_LOG_LEVEL_INFO,
                                    "pretrack aos=%s az=%.2f el=%.2f lead=%.1fs",
                                    aos_str,
                                    aos_az360,
                                    ctrl->pretrack_target_el,
                                    ROT_PRETRACK_AOS_OFFSET_SEC);
                    }
                }
                else
                {
                    gdouble plan_az = 0.0;
                    gdouble plan_el = 0.0;
                    gdouble plan_t = 0.0;
                    gdouble pretrack_el = MAX(ctrl->pretrack_min_el, elev_floor);

                    if (ctrl->conf != NULL)
                        pretrack_el = CLAMP(pretrack_el,
                                            ctrl->conf->minel,
                                            ctrl->conf->maxel);

                    if (rotctrl_find_pretrack_cmd(ctrl,
                                                       ctrl->t,
                                                       elev_floor,
                                                       &plan_az,
                                                       &plan_el,
                                                       &plan_t))
                    {
                        ctrl->pretrack_target_az = plan_az;
                        ctrl->pretrack_target_el = plan_el;
                        ctrl->pretrack_aos_time = plan_t;
                        ctrl->pretrack_target_valid = TRUE;
                        ctrl->pretrack_last_update_us = now_us;
                        ctrl->last_desired_update_us = now_us;
                        ctrl->force_next_send = TRUE;
                        if (ctrl->verbose_logging)
                        {
                            sat_log_log(SAT_LOG_LEVEL_INFO,
                                        "pretrack plan_entry az=%.2f el=%.2f",
                                        plan_az,
                                        plan_el);
                        }
                    }
                    else if (live_valid &&
                             live_el >= (elev_floor -
                                         ROT_PRETRACK_LIVE_FALLBACK_MARGIN_DEG))
                    {
                        ctrl->pretrack_target_az = live_az360;
                        ctrl->pretrack_target_el = pretrack_el;
                        ctrl->pretrack_aos_time = 0.0;
                        ctrl->pretrack_target_valid = TRUE;
                        ctrl->pretrack_last_update_us = now_us;
                        ctrl->last_desired_update_us = now_us;
                        ctrl->force_next_send = TRUE;
                        if (ctrl->verbose_logging)
                        {
                            sat_log_log(SAT_LOG_LEVEL_INFO,
                                        "pretrack fallback live az=%.2f el=%.2f",
                                        live_az360,
                                        pretrack_el);
                        }
                    }
                    else
                    {
                        ctrl->pretrack_target_valid = FALSE;
                        ctrl->pretrack_aos_time = 0.0;
                        ctrl->pretrack_last_update_us = now_us;
                        ctrl->last_desired_update_us = now_us;
                        ctrl->pretrack_wait_log_us = 0;
                        ctrl->pretrack_wrap_valid = FALSE;
                        ctrl->pretrack_wrap_user_az = 0.0;
                        ctrl->pretrack_wrap_raw_az = 0.0;
                        ctrl->pretrack_wrap_k = 0;
                    }
                }
            }

            if (ctrl->pretrack_target_valid)
            {
                if (ctrl->trajectory_plan.crosses_endstop &&
                    !ctrl->seam_crossing_active)
                {
                    ctrl->seam_crossing_active = TRUE;
                    ctrl->seam_crossing_sent = FALSE;
                    ctrl->seam_crossing_lane_valid = FALSE;
                    ctrl->seam_crossing_lane_k = 0;
                    ctrl->seam_crossing_target_az360 = ctrl->pretrack_target_az;
                    ctrl->seam_crossing_since_us = now_us;
                    ctrl->pending_lane_valid = FALSE;
                    ctrl->pending_lane_since_us = 0;
                    ctrl->locked_lane_valid = FALSE;
                    ctrl->seam_valid = FALSE;
                    sat_log_log(SAT_LOG_LEVEL_INFO,
                                "pretrack seam crossing detected target_az=%.2f crosses_endstop=1",
                                ctrl->pretrack_target_az);
                    rot_term_log(ctrl, "gpredict:state",
                                 "pretrack seam crossing detected target_az=%.2f",
                                 ctrl->pretrack_target_az);
                }

                pretrack_active = TRUE;
                az_pred = ctrl->pretrack_target_az;
                el_pred = ctrl->pretrack_target_el;
                pred_valid = isfinite(az_pred) && isfinite(el_pred);
                target_az360 = az_pred;
                target_el = el_pred;

                {
                    gboolean pretrack_done = FALSE;
                    const gchar *done_reason = NULL;

                    /* Handover to normal tracking as soon as the live
                     * satellite elevation is in-range. This prevents
                     * PRETRACK from sticking when AOS time is unknown or
                     * when hysteresis keeps below_horizon latched.
                     */
                    if (live_valid && live_el >= elev_floor)
                    {
                        pretrack_done = TRUE;
                        done_reason = "elev";
                    }
                    else if (ctrl->pretrack_aos_time > 0.0 &&
                             ctrl->t >= ctrl->pretrack_aos_time)
                    {
                        pretrack_done = TRUE;
                        done_reason = "aos";
                    }

                    if (ctrl->pretrack_aos_time > 0.0)
                    {
                        gdouble aos_in = (ctrl->pretrack_aos_time - ctrl->t) * secday;
                        if (aos_in < 0.0)
                            aos_in = 0.0;

                        if (now_us - ctrl->pretrack_wait_log_us >= ROT_PRETRACK_RECALC_US)
                        {
                            sat_log_log(SAT_LOG_LEVEL_INFO,
                                        "pretrack waiting: now_el=%.2f next_aos_in=%.1f",
                                        live_valid ? live_el : 0.0,
                                        aos_in);
                            ctrl->pretrack_wait_log_us = now_us;
                        }
                    }

                    if (pretrack_done)
                    {
                        pretrack_active = FALSE;
                        ctrl->pretrack_target_valid = FALSE;
                        ctrl->pretrack_aos_time = 0.0;
                        ctrl->pretrack_wrap_valid = FALSE;
                        ctrl->pretrack_wrap_user_az = 0.0;
                        ctrl->pretrack_wrap_raw_az = 0.0;
                        ctrl->pretrack_wrap_k = 0;
                        below_horizon = FALSE;
                        sat_log_log(SAT_LOG_LEVEL_INFO,
                                    "pretrack complete: reason=%s now_el=%.2f",
                                    done_reason ? done_reason : "handover",
                                    live_valid ? live_el : 0.0);
                    }
                }
            }
            else
            {
                hold_below = TRUE;
                hold_no_target = TRUE;
            }
        }

        if (!below_horizon)
        {
            ctrl->pretrack_target_valid = FALSE;
            ctrl->pretrack_aos_time = 0.0;
            ctrl->pretrack_wait_log_us = 0;
            ctrl->pretrack_wrap_valid = FALSE;
            ctrl->pretrack_wrap_user_az = 0.0;
            ctrl->pretrack_wrap_raw_az = 0.0;
            ctrl->pretrack_wrap_k = 0;
        }

        if (!below_horizon && !pretrack_active)
        {
            if (rotctrl_predict_at(ctrl,
                                   ctrl->t + (ROT_TRACK_LOOKAHEAD_SEC / secday),
                                   &setpoint_az360,
                                   &setpoint_el))
            {
                pred_valid = TRUE;
                az_pred = setpoint_az360;
                el_pred = setpoint_el;
                target_az360 = setpoint_az360;
                target_el = setpoint_el;
            }
        }

        if (below_horizon && !pretrack_active)
        {
            hold_below = TRUE;
            if (!hold_no_target)
            {
                setaz = gtk_rot_knob_get_value(GTK_ROT_KNOB(ctrl->AzSet));
                setel = gtk_rot_knob_get_value(GTK_ROT_KNOB(ctrl->ElSet));
                target_az360 = rot_ui_to_az360(setaz, ui_mode);
                target_el = setel;
                pred_valid = TRUE;
            }
        }

        if (!pred_valid && !hold_no_target)
        {
            hold_below = TRUE;
            setaz = gtk_rot_knob_get_value(GTK_ROT_KNOB(ctrl->AzSet));
            setel = gtk_rot_knob_get_value(GTK_ROT_KNOB(ctrl->ElSet));
            target_az360 = rot_ui_to_az360(setaz, ui_mode);
            target_el = setel;
            pred_valid = TRUE;
        }

        if (pred_valid && !hold_below &&
            tracking_mode == ROT_PLAN_MODE_FLIP &&
            ctrl->conf->maxel >= 180.0)
        {
            target_az360 = rot_norm360(target_az360 + 180.0);
            target_el = 180.0 - target_el;
        }

        if (ctrl->conf->axis_mode == ROT_AXIS_MODE_AZ_ONLY)
            target_el = ctrl->conf->minel;

        display_az360 = rot_norm360(target_az360);
        display_el = target_el;
        if (pred_valid)
        {
            gdouble xform_floor = -1.0;
            if (gp_rot_transform_target(ctrl,
                                        target_az360,
                                        target_el,
                                        xform_floor,
                                        &xform))
            {
                display_az360 = xform.az_after_southzero;
                display_el = xform.el_after_southzero;
                target_cmd_az360 = xform.az360_final;
                target_cmd_el = xform.el_final;
                az_pred = display_az360;
                el_pred = display_el;
            }
            else
            {
                pred_valid = FALSE;
            }
        }

        setaz = rot_az360_to_ui(display_az360, ui_mode);
        setel = display_el;

        if (pretrack_active && pred_valid && !ctrl->seam_valid)
        {
            ctrl->seam_az360 = rot_norm360(target_cmd_az360 + 180.0);
            ctrl->seam_valid = TRUE;
        }

        if (!ctrl->engaged)
        {
            gtk_rot_knob_set_value(GTK_ROT_KNOB(ctrl->AzSet), setaz);
            gtk_rot_knob_set_value(GTK_ROT_KNOB(ctrl->ElSet), setel);
        }
    }
    else
    {
        /* Not tracking: use current knob values */
        setaz = gtk_rot_knob_get_value(GTK_ROT_KNOB(ctrl->AzSet));
        setel = gtk_rot_knob_get_value(GTK_ROT_KNOB(ctrl->ElSet));
        target_az360 = rot_ui_to_az360(setaz, ui_mode);
        target_el = setel;
        display_az360 = target_az360;
        display_el = target_el;
        pred_valid = TRUE;
        if (gp_rot_transform_target(ctrl,
                                    target_az360,
                                    target_el,
                                    -1.0,
                                    &xform))
        {
            target_cmd_az360 = xform.az360_final;
            target_cmd_el = xform.el_final;
            az_pred = xform.az_after_southzero;
            el_pred = xform.el_after_southzero;
        }
        else
        {
            pred_valid = FALSE;
        }
        below_horizon = FALSE;
        pretrack_active = FALSE;
        hold_below = FALSE;
        ctrl->pretrack_target_valid = FALSE;
        ctrl->pretrack_aos_time = 0.0;
        ctrl->pretrack_wait_log_us = 0;
        ctrl->pretrack_wrap_valid = FALSE;
        ctrl->pretrack_wrap_user_az = 0.0;
        ctrl->pretrack_wrap_raw_az = 0.0;
        ctrl->pretrack_wrap_k = 0;
    }

    autocal_active = ctrl->cal_active;
    autocal_direct = autocal_active && ctrl->cal_direct_active;
    if (autocal_active)
    {
        gdouble cal_el = MAX(0.0, rotctrl_elev_floor(ctrl));
        if (ctrl->conf && ctrl->conf->axis_mode == ROT_AXIS_MODE_AZ_ONLY)
            cal_el = ctrl->conf->minel;
        setaz = 0.0;
        setel = cal_el;
        target_az360 = 0.0;
        target_el = cal_el;
        pred_valid = gp_rot_transform_target(ctrl,
                                             target_az360,
                                             target_el,
                                             -1.0,
                                             &xform);
        if (pred_valid)
        {
            display_az360 = xform.az_after_southzero;
            display_el = xform.el_after_southzero;
            target_cmd_az360 = xform.az360_final;
            target_cmd_el = xform.el_final;
            az_pred = xform.az_after_southzero;
            el_pred = xform.el_after_southzero;
        }
        else
        {
            display_az360 = target_az360;
            display_el = target_el;
            az_pred = 0.0;
            el_pred = 0.0;
        }
        below_horizon = FALSE;
        pretrack_active = FALSE;
        hold_below = FALSE;
    }

    cal_hold_active = ctrl->cal_hold_active;
    if (cal_hold_active && !autocal_active)
    {
        setaz = 0.0;
        setel = 0.0;
        target_az360 = 0.0;
        target_el = 0.0;
        pred_valid = gp_rot_transform_target(ctrl,
                                             target_az360,
                                             target_el,
                                             -1.0,
                                             &xform);
        if (pred_valid)
        {
            display_az360 = xform.az_after_southzero;
            display_el = xform.el_after_southzero;
            target_cmd_az360 = xform.az360_final;
            target_cmd_el = xform.el_final;
            az_pred = xform.az_after_southzero;
            el_pred = xform.el_after_southzero;
        }
        else
        {
            display_az360 = target_az360;
            display_el = target_el;
            target_cmd_az360 = 0.0;
            target_cmd_el = 0.0;
            az_pred = 0.0;
            el_pred = 0.0;
        }
        below_horizon = FALSE;
        pretrack_active = FALSE;
        hold_below = FALSE;
    }

    /* Handle I/O with rotctld client if running */
    if (ctrl->client.running)
    {
        if (g_mutex_trylock(&ctrl->client.mutex))
        {
            error = ctrl->client.io_error;
            cmd_rejected = ctrl->client.cmd_rejected;
            rotaz = ctrl->client.azi_in;
            rotel = ctrl->client.ele_in;
            rotaz_backend = ctrl->client.azi_mech_in;
            rotel_backend = ctrl->client.ele_mech_in;
            pos_unknown = ctrl->client.pos_unknown;
            pos_cmd_ok = ctrl->client.pos_cmd_ok;
            last_pos_us = ctrl->client.last_pos_us;
            pos_failures = ctrl->client.pos_failures;
            comm_status = ctrl->comm_status;
            g_strlcpy(last_pos_error, ctrl->client.last_pos_error,
                      sizeof(last_pos_error));
            if (ctrl->client.limits_valid)
            {
                caps_valid = TRUE;
                caps_az_min = ctrl->client.az_min;
                caps_az_max = ctrl->client.az_max;
                caps_el_min = ctrl->client.el_min;
                caps_el_max = ctrl->client.el_max;
                caps_south_zero = ctrl->client.south_zero;
            }
            g_mutex_unlock(&ctrl->client.mutex);
        }

        if (last_pos_us > 0)
        {
            gint64 now_us = g_get_monotonic_time();
            plot_pos_age_ms = (now_us - last_pos_us) / 1000;
            if (plot_pos_age_ms < 0)
                plot_pos_age_ms = 0;
            keep_rotor_plot =
                (plot_pos_age_ms < rotctrl_plot_stale_hide_ms(ctrl));
        }

        if (error)
        {
            rotpos_valid = FALSE;
            gtk_label_set_text(GTK_LABEL(ctrl->AzRead), _("UNKNOWN"));
            gtk_label_set_text(GTK_LABEL(ctrl->ElRead), _("UNKNOWN"));
            if (!keep_rotor_plot)
                rotctrl_set_rotor_pos_on_plots(ctrl, -10.0, -10.0);
        }
        else
        {
            use_caps = rotctrl_use_rotctld_caps(ctrl,
                                                caps_valid,
                                                caps_az_min,
                                                caps_az_max,
                                                caps_el_min,
                                                caps_el_max);
            rotctrl_update_user_limits(ctrl);
            rotctrl_update_backend_limits(ctrl,
                                          caps_valid,
                                          caps_az_min,
                                          caps_az_max,
                                          caps_el_min,
                                          caps_el_max,
                                          caps_south_zero);
            rotctrl_log_limits_on_engage(ctrl);
            rotctrl_note_wrap_mismatch(ctrl);

            user_span_cfg = rotctrl_span_from_limits(ctrl->conf,
                                                     FALSE,
                                                     caps_az_min,
                                                     caps_az_max,
                                                     caps_el_min,
                                                     caps_el_max,
                                                     &user_span_extended);
            if (caps_valid)
            {
                backend_span_cfg = rotctrl_span_from_limits(NULL,
                                                            TRUE,
                                                            caps_az_min,
                                                            caps_az_max,
                                                            caps_el_min,
                                                            caps_el_max,
                                                            &backend_span_extended);
            }
            else
            {
                backend_span_cfg = user_span_cfg;
                backend_span_extended = (backend_span_cfg.az_mode == AZ_MODE_EXTENDED);
            }
            backend_span_mode = rotctrl_span_mode_from_span_cfg(&backend_span_cfg);
            backend_az_min = caps_valid ? caps_az_min : user_span_cfg.az_min;
            backend_az_max = caps_valid ? caps_az_max : user_span_cfg.az_max;
            backend_el_min = caps_valid ? caps_el_min : user_span_cfg.el_min;
            backend_el_max = caps_valid ? caps_el_max : user_span_cfg.el_max;
            ctrl->span_extended = backend_span_extended;
            ctrl->span_mode = rotctrl_span_from_conf(ctrl->conf);
            rotpos_valid = pos_recent;
            pos_send_ok = pos_recent;

            if (rotpos_valid)
            {
                if (caps_valid)
                    rotaz_backend = rotctrl_normalize_backend_az(rotaz_backend,
                                                                caps_az_min,
                                                                caps_az_max);
                else
                    rotaz_backend = rotctrl_normalize_backend_az(rotaz_backend,
                                                                0.0, 360.0);

                if (backend_span_extended)
                {
                    ctrl->az_abs_cur =
                        rotctrl_normalize_az_to_limits(rotaz_backend,
                                                       backend_span_cfg.az_min,
                                                       backend_span_cfg.az_max);
                    ctrl->last_meas_span_az = NAN;
                }
                else
                {
                    gdouble meas_span = az_norm_span(rotaz_backend, backend_span_mode);
                    if (isnan(ctrl->last_meas_span_az))
                        ctrl->az_abs_cur = meas_span;
                    else
                        ctrl->az_abs_cur = az_unwrap_to_abs(ctrl->az_abs_cur,
                                                            meas_span,
                                                            backend_span_mode);
                    ctrl->last_meas_span_az = meas_span;
                }

                if (ctrl->last_cmd_time_us == 0)
                    ctrl->az_abs_last_cmd = ctrl->az_abs_cur;
            }

            if (rotpos_valid)
            {
                gdouble rotaz_disp = rotaz;
                text = g_strdup_printf("%.1f\302\260", rotaz_disp);
                gtk_label_set_text(GTK_LABEL(ctrl->AzRead), text);
                g_free(text);
                text = g_strdup_printf("%.1f\302\260", rotel);
                gtk_label_set_text(GTK_LABEL(ctrl->ElRead), text);
                g_free(text);

                gdouble rotaz_plot =
                    azel_normalize_az_0_360(rot_ui_to_az360(rotaz, ui_mode));
                rotctrl_set_rotor_pos_on_plots(ctrl, rotaz_plot, rotel);

                if (ctrl->conf != NULL)
                {
                    rotctrl_sync_manual_from_position(ctrl, ctrl->az_abs_cur, rotel_backend,
                                                      &setaz, &setel);
                }
            }
            else
            {
                if (comm_status == ROT_COMM_DEGRADED ||
                    comm_status == ROT_COMM_LINK_LOST)
                {
                    gtk_label_set_text(GTK_LABEL(ctrl->AzRead), _("UNKNOWN"));
                    gtk_label_set_text(GTK_LABEL(ctrl->ElRead), _("UNKNOWN"));
                    if (!keep_rotor_plot)
                        rotctrl_set_rotor_pos_on_plots(ctrl, -10.0, -10.0);
                }
                else
                {
                    gdouble rotaz_disp = rotaz;
                    text = g_strdup_printf("%.1f\302\260", rotaz_disp);
                    gtk_label_set_text(GTK_LABEL(ctrl->AzRead), text);
                    g_free(text);
                    text = g_strdup_printf("%.1f\302\260", rotel);
                    gtk_label_set_text(GTK_LABEL(ctrl->ElRead), text);
                    g_free(text);
                }
            }
        }

        if (autocal_active)
        {
            setaz = 0.0;
            setel = 0.0;
        }

        /* Unified angle conversion pipeline for manual, pretrack, tracking. */
        gdouble raw_cmd_az = target_cmd_az360;
        gdouble raw_cmd_el = target_cmd_el;
        RotCmdPipeline pipeline = { 0 };
        RotCmdPipeline desired_pipeline = { 0 };
        gdouble log_az_pred = az_pred;
        gdouble log_el_pred = el_pred;
        gdouble log_az_user = 0.0;
        gdouble meas_backend_az = 0.0;
        gdouble meas_backend_el = 0.0;
        gboolean fresh_feedback = FALSE;
        gint64 now_us = g_get_monotonic_time();
        gint64 last_cmd_us = 0;
        gint64 last_cmd_ok_us = 0;
        gdouble last_out_az = 0.0;
        gdouble last_out_el = 0.0;
        gboolean allow_send = FALSE;
        gdouble delta_az_phys = 0.0;
        gdouble delta_el_phys = 0.0;
        gdouble eps_az = 0.0;
        gdouble eps_el = 0.0;
        gdouble desired_user_az = 0.0;
        gdouble desired_user_el = 0.0;
        gdouble desired_raw_az = 0.0;
        gdouble desired_raw_el = 0.0;
        gdouble desired_backend_az = 0.0;
        gdouble desired_backend_el = 0.0;
        gdouble setpoint_backend_az = 0.0;
        gdouble setpoint_backend_el = 0.0;
        rot_cmd_decision_t decision_out = { 0 };
        gdouble last_cmd_user_az_log = 0.0;
        gdouble last_cmd_user_el_log = 0.0;
        gdouble az_delta_deg = 0.0;
        gdouble el_delta_deg = 0.0;
        gdouble deadband_az = 0.0;
        gdouble deadband_el = 0.0;
        gdouble threshold_deg = 0.0;
        gboolean send_ok = FALSE;
        gboolean force_send = FALSE;
        gboolean force_transition = FALSE;
        gboolean allow_no_pos_manual = FALSE;
        gboolean have_target = FALSE;
        gboolean not_at_target = FALSE;
        gboolean manual_override = FALSE;
        gboolean pos_stale_now = FALSE;
        gboolean pos_fresh = FALSE;
        gboolean moving_toward = FALSE;
        gboolean stopped_unexpected = FALSE;
        gdouble motion_err = 0.0;
        gdouble err_backend_az = 0.0;
        gdouble err_backend_el = 0.0;
        gboolean resend_due = FALSE;
        gdouble delta_backend_az = 0.0;
        gdouble delta_backend_el = 0.0;
        gdouble delta_user_az = 0.0;
        gdouble delta_user_el = 0.0;
        gboolean min_step_exceeded = FALSE;
        gint64 stall_since_us = 0;
        gint64 pos_age_ms = -1;
        gint64 target_age_ms = -1;
        rot_target_state_t desired_state = ROT_TARGET_STATE_IDLE;
        const gchar *state_reason = "idle";
        rot_cmd_action_t gate = ROT_CMD_ACTION_SUPPRESS;
        rot_cmd_reason_t reason = ROT_CMD_REASON_DEADBAND;
        const gchar *sched_reason = "HOLD_THRESHOLD";

        caps_valid = FALSE;
        g_mutex_lock(&ctrl->client.mutex);
        if (ctrl->client.limits_valid)
        {
            caps_valid = TRUE;
            caps_az_min = ctrl->client.az_min;
            caps_az_max = ctrl->client.az_max;
            caps_el_min = ctrl->client.el_min;
            caps_el_max = ctrl->client.el_max;
            caps_south_zero = ctrl->client.south_zero;
        }
        last_cmd_us = ctrl->client.last_cmd_us;
        last_cmd_ok_us = ctrl->client.last_cmd_ok_us;
        last_out_az = ctrl->client.azi_out;
        last_out_el = ctrl->client.ele_out;
        last_cmd_ok_az = ctrl->client.last_cmd_ok_az;
        last_cmd_backend = ctrl->client.last_cmd_backend_az;
        last_cmd_el = ctrl->client.last_cmd_backend_el;
        have_last_cmd_backend = ctrl->client.last_cmd_backend_valid;
        stall_since_us = ctrl->client.stall_since_us;
        g_mutex_unlock(&ctrl->client.mutex);

        if (have_last_cmd_backend)
        {
            last_cmd_az360 = rot_backend_pos_to_az360(last_cmd_backend);
            have_last_cmd_phys = TRUE;
        }
        else if (last_cmd_ok_us > 0)
        {
            last_cmd_backend = last_cmd_ok_az;
            last_cmd_az360 = rot_backend_pos_to_az360(last_cmd_backend);
            have_last_cmd_backend = TRUE;
            have_last_cmd_phys = TRUE;
        }
        else if (last_cmd_us > 0)
        {
            last_cmd_backend = last_out_az;
            last_cmd_el = last_out_el;
            last_cmd_az360 = rot_backend_pos_to_az360(last_cmd_backend);
            have_last_cmd_backend = TRUE;
            have_last_cmd_phys = TRUE;
        }
        else
        {
            have_last_cmd_phys = FALSE;
        }

        if (have_last_cmd_backend)
        {
            ctrl->last_cmd_az360 = last_cmd_az360;
            ctrl->last_cmd_el = last_cmd_el;
            ctrl->last_cmd_valid = TRUE;
        }
        else
        {
            ctrl->last_cmd_valid = FALSE;
        }

        use_caps = rotctrl_use_rotctld_caps(ctrl,
                                            caps_valid,
                                            caps_az_min,
                                            caps_az_max,
                                            caps_el_min,
                                            caps_el_max);
        rotctrl_update_user_limits(ctrl);
        rotctrl_update_backend_limits(ctrl,
                                      caps_valid,
                                      caps_az_min,
                                      caps_az_max,
                                      caps_el_min,
                                      caps_el_max,
                                      caps_south_zero);
        rotctrl_log_limits_on_engage(ctrl);
        rotctrl_note_wrap_mismatch(ctrl);

        user_span_cfg = rotctrl_span_from_limits(ctrl->conf,
                                                 FALSE,
                                                 caps_az_min,
                                                 caps_az_max,
                                                 caps_el_min,
                                                 caps_el_max,
                                                 &user_span_extended);
        if (caps_valid)
        {
            backend_span_cfg = rotctrl_span_from_limits(NULL,
                                                        TRUE,
                                                        caps_az_min,
                                                        caps_az_max,
                                                        caps_el_min,
                                                        caps_el_max,
                                                        &backend_span_extended);
        }
        else
        {
            backend_span_cfg = user_span_cfg;
            backend_span_extended = (backend_span_cfg.az_mode == AZ_MODE_EXTENDED);
        }
        backend_span_mode = rotctrl_span_mode_from_span_cfg(&backend_span_cfg);
        backend_az_min = caps_valid ? caps_az_min : user_span_cfg.az_min;
        backend_az_max = caps_valid ? caps_az_max : user_span_cfg.az_max;
        backend_el_min = caps_valid ? caps_el_min : user_span_cfg.el_min;
        backend_el_max = caps_valid ? caps_el_max : user_span_cfg.el_max;
        rotctrl_update_safety(ctrl, use_caps, caps_az_min, caps_az_max);

        ctrl->span_extended = backend_span_extended;
        ctrl->span_mode = rotctrl_span_from_conf(ctrl->conf);

        if (ctrl->conf && ctrl->conf->axis_mode == ROT_AXIS_MODE_AZ_ONLY)
            raw_cmd_el = ctrl->conf->minel;

        rot_transform_update(ctrl);

        fresh_feedback = rotpos_valid && pos_recent;
        if (fresh_feedback)
        {
            meas_backend_az = rotaz_backend;
            meas_backend_el = rotel_backend;
            meas_az360 = rot_backend_pos_to_az360(meas_backend_az);
            meas_el = meas_backend_el;
        }
        else if (have_last_cmd_backend)
        {
            meas_backend_az = last_cmd_backend;
            meas_backend_el = last_cmd_el;
            meas_az360 = rot_backend_pos_to_az360(meas_backend_az);
            meas_el = meas_backend_el;
        }
        else
        {
            meas_backend_az = last_out_az;
            meas_backend_el = last_out_el;
            meas_az360 = rot_backend_pos_to_az360(meas_backend_az);
            meas_el = meas_backend_el;
        }

        if (last_pos_us > 0 && now_us > last_pos_us)
            pos_age_ms = (now_us - last_pos_us) / 1000;
        if (ctrl->last_target_update_us > 0 && now_us > ctrl->last_target_update_us)
            target_age_ms = (now_us - ctrl->last_target_update_us) / 1000;

        ctrl->stale_recovered_pulse = FALSE;
        if (ctrl->cal_active)
        {
            ctrl->stale_hold_active = FALSE;
            ctrl->stale_hold_since_us = 0;
            ctrl->stale_resume_since_us = 0;
        }
        else if (pos_age_ms >= 0)
        {
            gint warn_ms = rotctrl_stale_warn_ms(ctrl);
            gint degraded_ms = rotctrl_stale_degraded_ms(ctrl);
            gint hold_ms = rotctrl_stale_hold_ms(ctrl);
            gint park_ms = rotctrl_stale_park_ms(ctrl);
            gint resume_ms = rotctrl_stale_resume_ms(ctrl);

            pos_fresh = (pos_age_ms <= warn_ms);

            if (pos_age_ms >= warn_ms && pos_age_ms < degraded_ms)
            {
                rot_log_rate_limited(ctrl, &ctrl->pos_warn_rate,
                                     ROTCTLD_FAILURE_LOG_INTERVAL_US,
                                     SAT_LOG_LEVEL_WARN, "gpredict:warn",
                                     "position warning: age=%lldms warn_ms=%d",
                                     (long long)pos_age_ms,
                                     warn_ms);
            }

            if (pos_age_ms >= hold_ms)
            {
                if (!ctrl->stale_hold_active)
                {
                    ctrl->stale_hold_active = TRUE;
                    ctrl->stale_hold_since_us = now_us;
                    ctrl->stale_resume_since_us = 0;
                    sat_log_log(SAT_LOG_LEVEL_WARN,
                                "rot hold: position stale age=%lldms hold_ms=%d",
                                (long long)pos_age_ms,
                                hold_ms);
                }
            }

            if (ctrl->stale_hold_active)
            {
                if (pos_fresh)
                {
                    if (ctrl->stale_resume_since_us == 0)
                        ctrl->stale_resume_since_us = now_us;
                    if (resume_ms == 0 ||
                        (now_us - ctrl->stale_resume_since_us) >=
                            ((gint64)resume_ms * 1000))
                    {
                        ctrl->stale_hold_active = FALSE;
                        ctrl->stale_resume_since_us = 0;
                        ctrl->stale_recovered_pulse = TRUE;
                        sat_log_log(SAT_LOG_LEVEL_INFO,
                                    "rot hold: recovered after %dms fresh",
                                    resume_ms);
                    }
                }
                else
                {
                    ctrl->stale_resume_since_us = 0;
                }
            }

            if (park_ms > 0 && pos_age_ms >= park_ms)
            {
                if (ctrl->park_pending_since_us == 0)
                {
                    ctrl->park_pending_since_us = now_us;
                    sat_log_log(SAT_LOG_LEVEL_WARN,
                                "rot stale: disconnect requested age=%lldms park_ms=%d",
                                (long long)pos_age_ms,
                                park_ms);
                    rotctld_request_thread_stop(ctrl, TRUE);
                }
            }
        }

        pos_stale_now = ctrl->tracking && ctrl->engaged && !pos_recent;
        if (ctrl->cal_active)
            pos_stale_now = FALSE;

        if (ctrl->verbose_logging &&
            (ctrl->last_stale_check_log_us == 0 ||
             (now_us - ctrl->last_stale_check_log_us) >= G_USEC_PER_SEC))
        {
            const gchar *stale_reason =
                pos_stale_now ? "pos_stale" : (pos_recent ? "pos_recent" : "target_recent");
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "stale_check: now_us=%lld last_target_us=%lld delta_ms=%lld stale_ms=%d => stale=%d reason=%s",
                        (long long)now_us,
                        (long long)ctrl->last_target_update_us,
                        (long long)target_age_ms,
                        rotctrl_stale_ms(ctrl),
                        pos_stale_now ? 1 : 0,
                        stale_reason);
            ctrl->last_stale_check_log_us = now_us;
        }

        if (pos_stale_now && !ctrl->pos_stale_active)
        {
            gint stale_ms = rotctrl_stale_ms(ctrl);

            if (pos_age_ms < 0)
                pos_age_ms = stale_ms;
            ctrl->pos_stale_active = TRUE;
            sat_log_log(SAT_LOG_LEVEL_WARN,
                        "rot stale: no position for %lldms; holding commands",
                        (long long)pos_age_ms);
        }
        else if (!pos_stale_now && ctrl->pos_stale_active)
        {
            ctrl->pos_stale_active = FALSE;
            if (rotpos_valid)
            {
                gboolean seeded_from_meas = FALSE;

                g_mutex_lock(&ctrl->client.mutex);
                if (!ctrl->client.last_cmd_backend_valid)
                {
                    ctrl->client.last_cmd_backend_az = meas_backend_az;
                    ctrl->client.last_cmd_backend_el = meas_backend_el;
                    ctrl->client.last_cmd_backend_valid = TRUE;
                    seeded_from_meas = TRUE;
                }
                g_mutex_unlock(&ctrl->client.mutex);

                if (seeded_from_meas)
                {
                    last_cmd_backend = meas_backend_az;
                    have_last_cmd_backend = TRUE;
                    if (!ctrl->locked_lane_valid)
                    {
                        gdouble meas_norm = gp_norm360(meas_backend_az);
                        ctrl->locked_lane_k =
                            (gint)lrint((meas_backend_az - meas_norm) / 360.0);
                        ctrl->locked_lane_valid = TRUE;
                    }
                    sat_log_log(SAT_LOG_LEVEL_INFO,
                                "rot stale: recovered; seeding refs from measured az=%.2f el=%.2f",
                                meas_backend_az,
                                meas_backend_el);
                }
                else
                {
                    sat_log_log(SAT_LOG_LEVEL_INFO,
                                "rot stale: recovered; keeping command reference");
                }
            }
            else
            {
                sat_log_log(SAT_LOG_LEVEL_INFO,
                            "rot stale: recovered");
            }
        }

        if (cal_hold_active)
        {
            desired_state = ROT_TARGET_STATE_HOLD;
            state_reason = "cal_hold";
        }
        else if (ctrl->tracking && !ctrl->engaged)
        {
            desired_state = ROT_TARGET_STATE_IDLE;
            state_reason = "disengaged";
        }
        else if (ctrl->tracking && ctrl->conf && ctrl->target)
        {
            if (hold_below)
            {
                desired_state = ROT_TARGET_STATE_HOLD;
                state_reason = "hold";
            }
            else if (pretrack_active)
            {
                desired_state = ROT_TARGET_STATE_PRETRACK;
                state_reason = "pretrack";
            }
            else if (!pred_valid)
            {
                desired_state = ROT_TARGET_STATE_HOLD;
                state_reason = "pred_invalid";
            }
            else
            {
                desired_state = ROT_TARGET_STATE_TRACKING_NORMAL;
                state_reason = "tracking";
            }
        }
        else if (ctrl->tracking)
        {
            desired_state = ROT_TARGET_STATE_HOLD;
            state_reason = "no_target";
        }
        else
        {
            desired_state = ROT_TARGET_STATE_IDLE;
            state_reason = "idle";
        }

        if (ctrl->seam_crossing_active)
        {
            if (desired_state != ROT_TARGET_STATE_PRETRACK)
            {
                sat_log_log(SAT_LOG_LEVEL_INFO,
                            "pretrack seam crossing cleared reason=%s",
                            rot_target_state_name(desired_state));
                rot_term_log(ctrl, "gpredict:state",
                             "pretrack seam crossing cleared reason=%s",
                             rot_target_state_name(desired_state));
                ctrl->seam_crossing_active = FALSE;
                ctrl->seam_crossing_sent = FALSE;
                ctrl->seam_crossing_lane_valid = FALSE;
                ctrl->seam_crossing_lane_k = 0;
                ctrl->seam_crossing_target_az360 = 0.0;
                ctrl->seam_crossing_since_us = 0;
            }
            else if (rotpos_valid && ctrl->seam_crossing_lane_valid)
            {
                gdouble meas_norm = gp_norm360(meas_backend_az);
                gint meas_k =
                    (gint)lrint((meas_backend_az - meas_norm) / 360.0);

                if (meas_k == ctrl->seam_crossing_lane_k)
                {
                    sat_log_log(SAT_LOG_LEVEL_INFO,
                                "pretrack seam crossing complete meas=%.2f lane_k=%d",
                                meas_backend_az,
                                ctrl->seam_crossing_lane_k);
                    rot_term_log(ctrl, "gpredict:state",
                                 "pretrack seam crossing complete meas=%.2f lane_k=%d",
                                 meas_backend_az,
                                 ctrl->seam_crossing_lane_k);
                    ctrl->seam_crossing_active = FALSE;
                    ctrl->seam_crossing_sent = FALSE;
                    ctrl->seam_crossing_lane_valid = FALSE;
                    ctrl->seam_crossing_lane_k = 0;
                    ctrl->seam_crossing_target_az360 = 0.0;
                    ctrl->seam_crossing_since_us = 0;
                }
            }
        }

        if (ctrl->wrap_acquire_active &&
            !(desired_state == ROT_TARGET_STATE_TRACKING_NORMAL ||
              desired_state == ROT_TARGET_STATE_TRACKING_DEGRADED))
        {
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        "TRACK wrap acquisition cleared state=%s",
                        rot_target_state_name(desired_state));
            rot_term_log(ctrl, "gpredict:state",
                         "TRACK wrap acquisition cleared state=%s",
                         rot_target_state_name(desired_state));
            ctrl->wrap_acquire_active = FALSE;
            ctrl->wrap_acquire_sent = FALSE;
            ctrl->wrap_acquire_target_backend = 0.0;
            ctrl->wrap_acquire_target_az360 = 0.0;
            ctrl->wrap_acquire_target_k = 0;
            ctrl->wrap_acquire_since_us = 0;
            ctrl->wrap_acquire_last_log_us = 0;
        }

        state_changed = (desired_state != ctrl->target_state);
        force_transition =
            state_changed &&
            ctrl->target_state == ROT_TARGET_STATE_PRETRACK &&
            (desired_state == ROT_TARGET_STATE_TRACKING_NORMAL ||
             desired_state == ROT_TARGET_STATE_TRACKING_DEGRADED);

        have_target = pred_valid || cal_hold_active ||
                      (hold_below && !hold_no_target);
        if (autocal_active && !autocal_direct)
            have_target = have_target || autocal_active;
        force_send = cal_force_send || ctrl->force_next_send || force_transition;
        if (ctrl->seam_crossing_active &&
            desired_state == ROT_TARGET_STATE_PRETRACK &&
            !ctrl->seam_crossing_sent)
            force_send = TRUE;

        gdouble ref_backend = fresh_feedback ? meas_backend_az : last_cmd_backend;
        gboolean have_ref_backend = fresh_feedback || have_last_cmd_backend;

        if (!have_last_cmd_backend && fresh_feedback)
        {
            g_mutex_lock(&ctrl->client.mutex);
            if (!ctrl->client.last_cmd_backend_valid)
            {
                ctrl->client.last_cmd_backend_az = meas_backend_az;
                ctrl->client.last_cmd_backend_el = meas_backend_el;
                ctrl->client.last_cmd_backend_valid = TRUE;
                if (ctrl->verbose_logging)
                    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                                "rot lane: seeding ref_backend from measured az=%.2f",
                                meas_backend_az);
            }
            g_mutex_unlock(&ctrl->client.mutex);
            last_cmd_backend = meas_backend_az;
            have_last_cmd_backend = TRUE;
        }

        ref_backend = have_last_cmd_backend ? last_cmd_backend : meas_backend_az;
        have_ref_backend = have_last_cmd_backend || rotpos_valid;

        desired_raw_az = target_cmd_az360;
        desired_raw_el = target_cmd_el;
        /* Keep desired_user in the same command domain as desired_raw.
         * Using pre-transform display az/el here causes wrap/normalization
         * jitter in inverted tracking/pretrack paths.
         */
        desired_user_az = rot_az360_to_ui(desired_raw_az, ui_mode);
        desired_user_el = desired_raw_el;
        threshold_deg = (ctrl->threshold > 0.0) ? ctrl->threshold : 0.10;
        eps_az = threshold_deg;
        eps_el = threshold_deg;
        deadband_az = threshold_deg;
        deadband_el = threshold_deg;

        rotctrl_build_target_caps(ctrl, &decision_caps);

        if (have_target && ctrl->tracking &&
            (desired_state == ROT_TARGET_STATE_PRETRACK ||
             desired_state == ROT_TARGET_STATE_TRACKING_NORMAL ||
             desired_state == ROT_TARGET_STATE_TRACKING_DEGRADED))
        {
            /* Wrap resolution must compare against a reference in the same
             * command domain as desired_user_az (UI-wrap of command az360),
             * not the display-mapped rotor readback.
             */
            gdouble ref_user_az = rot_az360_to_ui(meas_az360, ui_mode);
            gboolean have_ref_user = rotpos_valid || have_last_cmd_backend;
            gdouble resolved_user_az = desired_user_az;
            gdouble resolved_raw_az = desired_raw_az;
            gint resolved_k = 0;
            gboolean resolved = FALSE;
            rot_target_invalid_reason_t wrap_reason = ROT_TARGET_INVALID_NONE;
            gboolean wrap_mismatch_now = FALSE;

            if (desired_state == ROT_TARGET_STATE_PRETRACK && ctrl->pretrack_wrap_valid)
            {
                if (fabs(shortest_az_delta(desired_raw_az,
                                           ctrl->pretrack_wrap_raw_az)) > 1e-6)
                {
                    ctrl->pretrack_wrap_valid = FALSE;
                }
            }

            if (!rot_target_is_valid(&decision_caps,
                                     desired_user_az,
                                     desired_user_el,
                                     NULL,
                                     &wrap_reason) &&
                wrap_reason == ROT_TARGET_INVALID_WRAP_MISMATCH)
            {
                wrap_mismatch_now = TRUE;
            }

            if (desired_state == ROT_TARGET_STATE_PRETRACK && ctrl->pretrack_wrap_valid)
            {
                desired_user_az = ctrl->pretrack_wrap_user_az;
                desired_raw_az = ctrl->pretrack_wrap_raw_az;
            }
            else
            {
                resolved = rotctrl_resolve_wrap_candidate(&ctrl->user_limits,
                                                          &ctrl->backend_limits,
                                                          desired_user_az,
                                                          desired_user_el,
                                                          desired_raw_az,
                                                          ref_user_az,
                                                          have_ref_user,
                                                          ui_mode,
                                                          ref_backend,
                                                          have_ref_backend,
                                                          &resolved_user_az,
                                                          &resolved_raw_az,
                                                          &resolved_k,
                                                          NULL);
                if (resolved)
                {
                    wrap_resolve_failed = FALSE;
                    if (wrap_mismatch_now ||
                        fabs(resolved_user_az - desired_user_az) > 1e-6)
                    {
                        gdouble cand0 = desired_user_az;
                        gdouble cand1 = desired_user_az + 360.0;
                        gdouble cand2 = desired_user_az - 360.0;
                        gdouble cand3 = desired_user_az + 720.0;
                        gdouble cand4 = desired_user_az - 720.0;
                        gchar limits_buf[128] = { 0 };
                        if (ctrl->user_limits.valid)
                        {
                            if (ctrl->backend_limits.valid)
                            {
                                g_snprintf(limits_buf, sizeof(limits_buf),
                                           "user=(%.2f..%.2f) backend=(%.2f..%.2f)",
                                           ctrl->user_limits.az_min,
                                           ctrl->user_limits.az_max,
                                           ctrl->backend_limits.az_min,
                                           ctrl->backend_limits.az_max);
                            }
                            else
                            {
                                g_snprintf(limits_buf, sizeof(limits_buf),
                                           "user=(%.2f..%.2f) backend=unknown",
                                           ctrl->user_limits.az_min,
                                           ctrl->user_limits.az_max);
                            }
                        }
                        else
                        {
                            g_snprintf(limits_buf, sizeof(limits_buf),
                                       "user=unknown backend=%s",
                                       ctrl->backend_limits.valid ? "set" : "unknown");
                        }
                        sat_log_log(SAT_LOG_LEVEL_INFO,
                                    "wrap_resolve: raw_az=%.2f user_az=%.2f candidates=[%.2f %.2f %.2f %.2f %.2f] "
                                    "chosen=%.2f current=%.2f %s reason=%s",
                                    desired_raw_az,
                                    desired_user_az,
                                    cand0, cand1, cand2, cand3, cand4,
                                    resolved_user_az,
                                    ref_user_az,
                                    limits_buf,
                                    wrap_mismatch_now ? "wrap_mismatch" : "rewrap");
                        rot_term_log(ctrl, "gpredict:state",
                                     "wrap_resolve: raw_az=%.2f user_az=%.2f candidates=[%.2f %.2f %.2f %.2f %.2f] "
                                     "chosen=%.2f current=%.2f %s reason=%s",
                                     desired_raw_az,
                                     desired_user_az,
                                     cand0, cand1, cand2, cand3, cand4,
                                     resolved_user_az,
                                     ref_user_az,
                                     limits_buf,
                                     wrap_mismatch_now ? "wrap_mismatch" : "rewrap");
                    }
                    desired_user_az = resolved_user_az;
                    desired_raw_az = resolved_raw_az;
                    if (desired_state == ROT_TARGET_STATE_PRETRACK)
                    {
                        ctrl->pretrack_wrap_valid = TRUE;
                        ctrl->pretrack_wrap_user_az = resolved_user_az;
                        ctrl->pretrack_wrap_raw_az = resolved_raw_az;
                        ctrl->pretrack_wrap_k = resolved_k;
                    }
                }
                else if (wrap_mismatch_now)
                {
                    wrap_resolve_failed = TRUE;
                    gdouble cand0 = desired_user_az;
                    gdouble cand1 = desired_user_az + 360.0;
                    gdouble cand2 = desired_user_az - 360.0;
                    gdouble cand3 = desired_user_az + 720.0;
                    gdouble cand4 = desired_user_az - 720.0;
                    gchar limits_buf[128] = { 0 };
                    if (ctrl->user_limits.valid)
                    {
                        if (ctrl->backend_limits.valid)
                        {
                            g_snprintf(limits_buf, sizeof(limits_buf),
                                       "user=(%.2f..%.2f) backend=(%.2f..%.2f)",
                                       ctrl->user_limits.az_min,
                                       ctrl->user_limits.az_max,
                                       ctrl->backend_limits.az_min,
                                       ctrl->backend_limits.az_max);
                        }
                        else
                        {
                            g_snprintf(limits_buf, sizeof(limits_buf),
                                       "user=(%.2f..%.2f) backend=unknown",
                                       ctrl->user_limits.az_min,
                                       ctrl->user_limits.az_max);
                        }
                    }
                    else
                    {
                        g_snprintf(limits_buf, sizeof(limits_buf),
                                   "user=unknown backend=%s",
                                   ctrl->backend_limits.valid ? "set" : "unknown");
                    }
                    sat_log_log(SAT_LOG_LEVEL_WARN,
                                "wrap_resolve: raw_az=%.2f user_az=%.2f candidates=[%.2f %.2f %.2f %.2f %.2f] "
                                "chosen=none current=%.2f %s reason=no_candidate",
                                desired_raw_az,
                                desired_user_az,
                                cand0, cand1, cand2, cand3, cand4,
                                ref_user_az,
                                limits_buf);
                    rot_term_log(ctrl, "gpredict:warn",
                                 "wrap_resolve: raw_az=%.2f user_az=%.2f candidates=[%.2f %.2f %.2f %.2f %.2f] "
                                 "chosen=none current=%.2f %s reason=no_candidate",
                                 desired_raw_az,
                                 desired_user_az,
                                 cand0, cand1, cand2, cand3, cand4,
                                 ref_user_az,
                                 limits_buf);
                }
            }
        }

        if (have_target && ctrl->committed_valid)
        {
            az_delta_deg = rot_ang_dist_deg(desired_user_az, ctrl->committed_user_az);
            el_delta_deg = fabs(desired_user_el - ctrl->committed_user_el);
        }

        if (have_target && ctrl->tracking)
        {
            gboolean commit_allowed =
                (desired_state == ROT_TARGET_STATE_PRETRACK ||
                 desired_state == ROT_TARGET_STATE_TRACKING_NORMAL ||
                 desired_state == ROT_TARGET_STATE_TRACKING_DEGRADED);
            gboolean committed_was_valid = ctrl->committed_valid;
            gboolean commit_force = force_send || state_changed || !committed_was_valid;
            gboolean commit_due = FALSE;

            if (commit_allowed)
            {
                if (!pos_fresh && !commit_force)
                {
                    commit_due = FALSE;
                }
                else
                {
                    commit_due = commit_force ||
                                 (ctrl->committed_valid &&
                                  (az_delta_deg >= deadband_az ||
                                   el_delta_deg >= deadband_el));
                }
            }

            if (commit_due)
            {
                ctrl->committed_user_az = desired_user_az;
                ctrl->committed_user_el = desired_user_el;
                ctrl->committed_raw_az360 = desired_raw_az;
                ctrl->committed_raw_el = desired_raw_el;
                ctrl->committed_valid = TRUE;
                ctrl->committed_since_us = now_us;
            }
        }

        rotctrl_pipeline_build(ctrl,
                               desired_raw_az,
                               desired_raw_el,
                               backend_az_min,
                               backend_az_max,
                               backend_el_min,
                               backend_el_max,
                               ref_backend,
                               have_ref_backend,
                               ctrl->locked_lane_valid,
                               ctrl->locked_lane_k,
                               ROT_LANE_ENDSTOP_MARGIN_DEG,
                               ROT_LANE_SWITCH_PENALTY_DEG,
                               FALSE,
                               &desired_pipeline);

        seam_cross_active =
            (desired_state == ROT_TARGET_STATE_PRETRACK) &&
            ctrl->seam_crossing_active;
        seam_cmd_valid = FALSE;
        if (seam_cross_active)
        {
            seam_cmd_valid =
                rotctrl_choose_seam_crossing_lane(ctrl,
                                                  desired_raw_az,
                                                  ref_backend,
                                                  backend_az_min,
                                                  backend_az_max,
                                                  &seam_lane_k,
                                                  &seam_cmd_az);
            if (seam_cmd_valid)
            {
                desired_pipeline.cmd_az = seam_cmd_az;
                desired_pipeline.lane_k = seam_lane_k;
                desired_pipeline.lane_crosses_seam = TRUE;
            }
            else if (ctrl->verbose_logging)
            {
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            "pretrack seam crossing: no candidate for target=%.2f ref=%.2f",
                            desired_raw_az,
                            ref_backend);
            }
        }

        if ((desired_state == ROT_TARGET_STATE_TRACKING_NORMAL ||
             desired_state == ROT_TARGET_STATE_TRACKING_DEGRADED) &&
            ctrl->wrap_acquire_active)
        {
            desired_pipeline.cmd_az = ctrl->wrap_acquire_target_backend;
            desired_pipeline.lane_k = ctrl->wrap_acquire_target_k;
        }

        if (ctrl->cal_active || cal_hold_active)
        {
            desired_pipeline.cmd_az =
                rotctrl_normalize_backend_az(0.0, backend_az_min, backend_az_max);
            desired_pipeline.cmd_el = 0.0;
        }

        pipeline = desired_pipeline;
        desired_backend_az = pipeline.cmd_az;
        desired_backend_el = pipeline.cmd_el;
        raw_cmd_az = desired_raw_az;
        raw_cmd_el = desired_raw_el;

        log_az_user = desired_user_az;
        gdouble log_cur_user_az = rot_az360_to_ui(meas_az360, ui_mode);
        gdouble log_cur_user_el = meas_el;
        gdouble log_offset_az = pred_valid
                                ? rot_ang_diff_deg(xform.az_after_offsets,
                                                   xform.az_after_southzero)
                                : 0.0;
        gdouble log_offset_el = pred_valid
                                ? (xform.el_after_offsets - xform.el_after_southzero)
                                : 0.0;
        const gchar *wrap_name =
            (ui_mode == ROT_UI_NORTH_CENTERED) ? "pm180" : "360";
        az_clamped = pipeline.az_clamped;
        el_clamped = pipeline.el_clamped;
        cmdaz = pipeline.cmd_az;
        cmdel = pipeline.cmd_el;
        if (ctrl->verbose_logging)
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rot lane: tgt az360=%.2f ref_backend=%.2f -> cmd_backend=%.2f (k=%d) limits=[%.2f..%.2f]",
                        gp_norm360(raw_cmd_az),
                        pipeline.lane_ref_backend,
                        cmdaz,
                        pipeline.lane_k,
                        backend_az_min,
                        backend_az_max);
        az_abs_cmd = backend_span_extended
                     ? cmdaz
                     : (rotpos_valid
                        ? az_target_to_nearest_abs(ctrl->az_abs_cur,
                                                   cmdaz,
                                                   backend_span_mode)
                        : cmdaz);

        {
            rot_target_state_t prev_state = ctrl->target_state;

            state_changed = (desired_state != prev_state);
            if (state_changed)
            {
                if (desired_state == ROT_TARGET_STATE_PRETRACK ||
                    prev_state == ROT_TARGET_STATE_PRETRACK)
                {
                    ctrl->pretrack_wrap_valid = FALSE;
                    ctrl->pretrack_wrap_user_az = 0.0;
                    ctrl->pretrack_wrap_raw_az = 0.0;
                    ctrl->pretrack_wrap_k = 0;
                }
                if (prev_state == ROT_TARGET_STATE_PRETRACK &&
                    (desired_state == ROT_TARGET_STATE_TRACKING_NORMAL ||
                     desired_state == ROT_TARGET_STATE_TRACKING_DEGRADED))
                {
                    gchar tbuf[64] = { 0 };
                    const gchar *tstr = "unknown";

                    rotctrl_format_utc_jd(ctrl->t, tbuf, sizeof(tbuf));
                    if (tbuf[0] != '\0')
                        tstr = tbuf;
                    sat_log_log(SAT_LOG_LEVEL_INFO,
                                "MODE_SWITCH PRETRACK->TRACKING at t=%s",
                                tstr);
                    rot_term_log(ctrl, "gpredict:state",
                                 "MODE_SWITCH PRETRACK->TRACKING at t=%s",
                                 tstr);
                }
                if (desired_state == ROT_TARGET_STATE_TRACKING_NORMAL ||
                    desired_state == ROT_TARGET_STATE_TRACKING_DEGRADED)
                {
                    ctrl->wrap_acquire_active = FALSE;
                    ctrl->wrap_acquire_sent = FALSE;
                    ctrl->wrap_acquire_target_backend = 0.0;
                    ctrl->wrap_acquire_target_az360 = 0.0;
                    ctrl->wrap_acquire_target_k = 0;
                    ctrl->wrap_acquire_since_us = 0;
                    ctrl->wrap_acquire_last_log_us = 0;
                }
                rot_target_state_set(ctrl, desired_state, state_reason);
                ctrl->target_valid_since_us = 0;
                ctrl->target_invalid_since_us = 0;
                if (desired_state == ROT_TARGET_STATE_PRETRACK ||
                    desired_state == ROT_TARGET_STATE_TRACKING_NORMAL)
                {
                    if (!ctrl->locked_lane_valid ||
                        (pipeline.lane_switched &&
                         pipeline.lane_k != ctrl->locked_lane_k))
                    {
                        ctrl->locked_lane_k = pipeline.lane_k;
                        ctrl->locked_lane_valid = TRUE;
                        if (rotctrl_wrap_debug_enabled())
                        {
                            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                                        "rot lane lock: k=%d state=%s",
                                        ctrl->locked_lane_k,
                                        rot_target_state_name(desired_state));
                        }
                    }
                }
                if (desired_state == ROT_TARGET_STATE_PRETRACK)
                {
                    gchar aos_buf[64] = { 0 };
                    const gchar *aos_str = "unknown";
                    gboolean have_aos = (ctrl->pretrack_aos_time > 0.0);

                    if (have_aos)
                    {
                        rotctrl_format_utc_jd(ctrl->pretrack_aos_time,
                                              aos_buf,
                                              sizeof(aos_buf));
                        if (aos_buf[0] != '\0')
                            aos_str = aos_buf;
                    }
                    if (have_aos)
                    {
                        sat_log_log(SAT_LOG_LEVEL_INFO,
                                    "pretrack target=AOS %s az=%.2f el=%.2f lead=%.1fs cmd=%.2f",
                                    aos_str,
                                    az_pred, el_pred,
                                    ROT_PRETRACK_AOS_OFFSET_SEC,
                                    cmdaz);
                        rot_term_log_verbose(ctrl, "gpredict:state",
                                             "pretrack target=AOS %s az=%.2f el=%.2f lead=%.1fs cmd=%.2f",
                                             aos_str,
                                             az_pred, el_pred,
                                             ROT_PRETRACK_AOS_OFFSET_SEC,
                                             cmdaz);
                    }
                    else
                    {
                        sat_log_log(SAT_LOG_LEVEL_INFO,
                                    "pretrack target=FALLBACK az=%.2f el=%.2f cmd=%.2f",
                                    az_pred, el_pred, cmdaz);
                        rot_term_log_verbose(ctrl, "gpredict:state",
                                             "pretrack target=FALLBACK az=%.2f el=%.2f cmd=%.2f",
                                             az_pred, el_pred, cmdaz);
                    }
                }
                else if (desired_state == ROT_TARGET_STATE_HOLD)
                {
                    rot_term_log_verbose(ctrl, "gpredict:state",
                                         "hold reason=%s", state_reason);
                }
                else if (desired_state == ROT_TARGET_STATE_TRACKING_NORMAL ||
                         desired_state == ROT_TARGET_STATE_TRACKING_DEGRADED)
                {
                    rot_term_log_verbose(ctrl, "gpredict:state",
                                         "tracking target=live");
                }
            }
        }

        if (ctrl->tracking &&
            (ctrl->target_state == ROT_TARGET_STATE_PRETRACK ||
             ctrl->target_state == ROT_TARGET_STATE_TRACKING_NORMAL ||
             ctrl->target_state == ROT_TARGET_STATE_TRACKING_DEGRADED))
        {
            if (!ctrl->locked_lane_valid)
            {
                ctrl->locked_lane_k = pipeline.lane_k;
                ctrl->locked_lane_valid = TRUE;
                if (rotctrl_wrap_debug_enabled())
                {
                    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                                "rot lane lock: k=%d state=%s",
                                ctrl->locked_lane_k,
                                rot_target_state_name(ctrl->target_state));
                }
            }
            else if (pipeline.lane_switched &&
                     pipeline.lane_k != ctrl->locked_lane_k)
            {
                ctrl->locked_lane_k = pipeline.lane_k;
            }
        }

        ctrl->tracking_active = ctrl->tracking &&
                                ctrl->engaged &&
                                (ctrl->target_state == ROT_TARGET_STATE_TRACKING_NORMAL ||
                                 ctrl->target_state == ROT_TARGET_STATE_TRACKING_DEGRADED ||
                                 ctrl->target_state == ROT_TARGET_STATE_PRETRACK);
        if (cal_hold_active)
            ctrl->tracking_active = FALSE;

        if (rotpos_valid && !hold_below)
        {
            gdouble az_before = az_abs_cmd;
            gboolean crossed_before =
                rotctrl_stop_crosses(&ctrl->safety,
                                     ctrl->az_abs_cur, az_before);

            safety_clamped =
                !safety_project_command(&ctrl->safety,
                                        ctrl->az_abs_cur, &az_abs_cmd);

            if (crossed_before && !safety_clamped &&
                fabs(shortest_az_delta(az_abs_cmd, az_before)) > 1e-6)
                safety_avoided = TRUE;
        }

        if (backend_span_extended)
        {
            az_abs_cmd = rotctrl_normalize_az_to_limits(az_abs_cmd,
                                                       backend_az_min,
                                                       backend_az_max);
            cmdaz = az_abs_cmd;
        }
        else
        {
            cmdaz = az_abs_to_span(az_abs_cmd, backend_span_mode);
        }
        cmdaz = rotctrl_normalize_backend_az(cmdaz,
                                             backend_az_min,
                                             backend_az_max);

        {
            gdouble cmdaz_before_guard = cmdaz;
            if (!hold_below)
            {
                gdouble guard_el = desired_user_el;
                gboolean use_zenith_guard =
                    (ctrl->tracking ||
                     ctrl->park_requested ||
                     ctrl->cal_active ||
                     cal_hold_active);
                if (ctrl->tracking && live_valid)
                    guard_el = live_el;

                if (use_zenith_guard)
                {
                    rotctrl_apply_zenith_guard(ctrl, guard_el, ctrl->az_abs_cur,
                                               rotpos_valid, backend_span_mode,
                                               &cmdaz);
                }
                else
                {
                    /* In manual mode, honor explicit azimuth commands. */
                    ctrl->az_hold_active = FALSE;
                }
                if (backend_span_extended)
                    cmdaz = rotctrl_normalize_az_to_limits(cmdaz,
                                                           backend_az_min,
                                                           backend_az_max);

                if (fabs(shortest_az_delta(cmdaz, cmdaz_before_guard)) > 1e-6)
                {
                    if (backend_span_extended)
                    {
                        az_abs_cmd = cmdaz;
                    }
                    else
                    {
                        az_abs_cmd = az_target_to_nearest_abs(ctrl->az_abs_cur,
                                                              cmdaz, backend_span_mode);
                    }
                }
            }
        }

        if (ctrl->tracking || !backend_span_extended)
        {
            gdouble cmdaz_user = rotctrl_clamp_user_az_interval(ctrl, cmdaz);

            if (fabs(cmdaz_user - cmdaz) > 1e-6)
            {
                az_clamped = TRUE;
                cmdaz = cmdaz_user;

                if (backend_span_extended)
                {
                    az_abs_cmd = cmdaz;
                }
                else if (rotpos_valid)
                {
                    az_abs_cmd = az_target_to_nearest_abs(ctrl->az_abs_cur,
                                                          cmdaz,
                                                          backend_span_mode);
                }
                else
                {
                    az_abs_cmd = cmdaz;
                }
            }
        }

        desired_backend_az = cmdaz;
        desired_backend_el = cmdel;

        ctrl->committed_backend_az = cmdaz;
        ctrl->committed_backend_el = cmdel;

        if (ctrl->tracking)
        {
            /* Keep the controller marker in the same transformed/display
             * domain used for setpoint and command generation. This avoids
             * azimuth drift on flipped/inverted trajectories.
             */
            if (pred_valid)
            {
                cmdaz_plot = azel_normalize_az_0_360(display_az360);
                cmdel_plot = display_el;
            }
            else if (live_valid)
            {
                cmdaz_plot = azel_normalize_az_0_360(live_az360);
                cmdel_plot = live_el;
            }
            else
            {
                cmdaz_plot = azel_normalize_az_0_360(target_az360);
                cmdel_plot = target_el;
            }
        }
        else
        {
            cmdaz_plot = azel_normalize_az_0_360(rot_ui_to_az360(setaz, ui_mode));
            cmdel_plot = setel;
        }

        ctrl->out_of_range = safety_clamped || az_clamped || el_clamped;

        if (ctrl->verbose_logging && !ctrl->axis_swap_warned && rotpos_valid)
        {
            gdouble rotaz_check = backend_span_extended
                                  ? ctrl->az_abs_cur
                                  : az_abs_to_span(ctrl->az_abs_cur, backend_span_mode);
            const gdouble near_deg = 5.0;
            const gdouble low_el = 45.0;

            if ((cmdel > 90.0 &&
                 fabs(rotaz_check - cmdel) <= near_deg &&
                 rotel < low_el) ||
                (cmdaz > 90.0 &&
                 fabs(rotel - cmdaz) <= near_deg &&
                 rotaz_check < low_el))
            {
                sat_log_log(SAT_LOG_LEVEL_WARN, "%s: axis swap suspected", __func__);
                rot_term_log(ctrl, "gpredict:warn", "axis swap suspected");
                ctrl->axis_swap_warned = TRUE;
            }
        }

        manual_override = rotctrl_manual_override_active(ctrl);
        allow_no_pos_manual =
            (!ctrl->tracking) &&
            (manual_override || ctrl->have_user_command);
        {
            gboolean user_cmd_allowed =
                ctrl->have_user_command ||
                ctrl->tracking_active ||
                manual_override ||
                ctrl->park_requested ||
                ctrl->cal_active ||
                ctrl->cal_hold_active;

            if (ctrl->hold_position_on_engage && user_cmd_allowed)
            {
                ctrl->hold_position_on_engage = FALSE;
                ctrl->hold_position_log_emitted = FALSE;
            }
        }

        if (ctrl->wrap_acquire_active &&
            (desired_state == ROT_TARGET_STATE_TRACKING_NORMAL ||
             desired_state == ROT_TARGET_STATE_TRACKING_DEGRADED))
        {
            gdouble wrap_tol = MAX(k_meas_tol_deg, k_meas_quantum_deg);
            gdouble diff_backend =
                fabs(shortest_az_delta(meas_backend_az,
                                       ctrl->wrap_acquire_target_backend));
            if (rotpos_valid && diff_backend <= wrap_tol)
            {
                sat_log_log(SAT_LOG_LEVEL_INFO,
                            "TRACK wrap acquisition complete target=%.2f meas=%.2f",
                            ctrl->wrap_acquire_target_backend,
                            meas_backend_az);
                rot_term_log(ctrl, "gpredict:state",
                             "TRACK wrap acquisition complete target=%.2f meas=%.2f",
                             ctrl->wrap_acquire_target_backend,
                             meas_backend_az);
                ctrl->wrap_acquire_active = FALSE;
                ctrl->wrap_acquire_sent = FALSE;
                ctrl->wrap_acquire_target_backend = 0.0;
                ctrl->wrap_acquire_target_az360 = 0.0;
                ctrl->wrap_acquire_target_k = 0;
                ctrl->wrap_acquire_since_us = 0;
                ctrl->wrap_acquire_last_log_us = 0;
            }
            else if (ctrl->wrap_acquire_last_log_us == 0 ||
                     (now_us - ctrl->wrap_acquire_last_log_us) >= G_USEC_PER_SEC)
            {
                sat_log_log(SAT_LOG_LEVEL_INFO,
                            "TRACK wrap acquisition in progress delta=%.2f target=%.2f meas=%.2f",
                            diff_backend,
                            ctrl->wrap_acquire_target_backend,
                            meas_backend_az);
                rot_term_log(ctrl, "gpredict:state",
                             "TRACK wrap acquisition in progress delta=%.2f target=%.2f meas=%.2f",
                             diff_backend,
                             ctrl->wrap_acquire_target_backend,
                             meas_backend_az);
                ctrl->wrap_acquire_last_log_us = now_us;
            }
        }

        if (ctrl->last_cmd_valid)
        {
            last_cmd_user_az_log =
                rot_az360_to_ui(ctrl->last_cmd_az360, ui_mode);
            last_cmd_user_el_log = ctrl->last_cmd_el;
        }

        if (ctrl->setpoint_valid)
        {
            setpoint_user_az = ctrl->setpoint_user_az;
            setpoint_user_el = ctrl->setpoint_user_el;
            setpoint_backend_az = ctrl->setpoint_backend_az;
            setpoint_backend_el = ctrl->setpoint_backend_el;
        }
        else
        {
            setpoint_user_az = desired_user_az;
            setpoint_user_el = desired_user_el;
            setpoint_backend_az = cmdaz;
            setpoint_backend_el = cmdel;
        }

        if (ctrl->setpoint_valid)
        {
            delta_user_az = rot_ang_dist_deg(desired_user_az, setpoint_user_az);
            delta_user_el = fabs(desired_user_el - setpoint_user_el);
            if (backend_span_extended)
                delta_backend_az = fabs(cmdaz - setpoint_backend_az);
            else
                delta_backend_az = fabs(shortest_az_delta(cmdaz, setpoint_backend_az));
            delta_backend_el = fabs(cmdel - setpoint_backend_el);
        }
        else
        {
            delta_user_az = 0.0;
            delta_user_el = 0.0;
            delta_backend_az = 0.0;
            delta_backend_el = 0.0;
        }

        min_step_exceeded =
            (!ctrl->setpoint_valid ||
             delta_backend_az >= threshold_deg ||
             delta_backend_el >= threshold_deg);

        if (ctrl->tracking && ctrl->setpoint_valid && pos_fresh)
        {
            err_backend_az = backend_span_extended
                                 ? fabs(meas_backend_az - setpoint_backend_az)
                                 : fabs(shortest_az_delta(meas_backend_az,
                                                          setpoint_backend_az));
            err_backend_el = fabs(meas_backend_el - setpoint_backend_el);
            gboolean reached =
                (err_backend_az <= eps_az && err_backend_el <= eps_el);
            not_at_target = !reached;
        }

        if (ctrl->tracking)
            allow_send = ctrl->engaged && !ctrl->monitor;
        else
            allow_send = ctrl->engaged && !ctrl->monitor &&
                         (pos_send_ok ||
                          pos_cmd_ok ||
                          allow_no_pos_manual);
        if (cal_hold_active)
            allow_send = ctrl->engaged && !ctrl->monitor;
        if (ctrl->cal_active)
            allow_send = allow_send &&
                         (ctrl->cal_state == ROT_AUTOCAL_DRIVE_ZERO);
        {
            RotTransformSnapshotState snap_state;
            rot_transform_snapshot_state_init(&snap_state);
            if (!rot_transform_snapshot_state_get(ctrl, &snap_state))
                allow_send = FALSE;
        }
        if (!have_target)
            allow_send = FALSE;
        if (cal_force_send)
            allow_send = ctrl->engaged && !ctrl->monitor;
        if (autocal_direct)
            allow_send = FALSE;

        delta_az_phys = rot_ang_dist_deg(meas_az360, raw_cmd_az);
        delta_el_phys = fabs(raw_cmd_el - meas_el);

        if (!ctrl->tracking && last_cmd_us == 0 && ctrl->have_user_command)
            force_send = TRUE;

        park_active = ctrl->park_requested;
        if (park_active)
            force_send = TRUE;
        if (ctrl->hold_position_on_engage &&
            !(ctrl->have_user_command ||
              ctrl->tracking_active ||
              manual_override ||
              park_active ||
              ctrl->cal_active ||
              cal_hold_active))
        {
            if (!ctrl->hold_position_log_emitted)
            {
                ctrl->hold_position_log_emitted = TRUE;
                rot_term_log_verbose(ctrl, "gpredict:rx",
                                     "engage: holding position until user command");
                if (ctrl->verbose_logging)
                    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                                "engage: holding position until user command");
            }
            allow_send = FALSE;
            force_send = FALSE;
        }

        if (ctrl->tracking || park_active)
        {
            if (ctrl->setpoint_valid && pos_fresh)
            {
                gdouble err_az = backend_span_extended
                                 ? fabs(meas_backend_az - setpoint_backend_az)
                                 : fabs(shortest_az_delta(meas_backend_az,
                                                          setpoint_backend_az));
                gdouble err_el = fabs(meas_backend_el - setpoint_backend_el);
                gboolean reached =
                    (err_az <= eps_az && err_el <= eps_el);

                err_backend_az = err_az;
                err_backend_el = err_el;
                if (reached)
                {
                    ctrl->motion_err_valid = FALSE;
                    ctrl->motion_stall_count = 0;
                    moving_toward = FALSE;
                    stopped_unexpected = FALSE;
                    stall_since_us = 0;
                }
                else
                {
                    motion_err = err_az + err_el;

                    if (ctrl->motion_err_valid)
                    {
                        if (motion_err <= (ctrl->motion_err_mag - 0.05))
                        {
                            moving_toward = TRUE;
                            ctrl->motion_stall_count = 0;
                            stall_since_us = 0;
                        }
                        else
                        {
                            if (ctrl->motion_stall_count < G_MAXUINT)
                                ctrl->motion_stall_count++;
                        }
                    }
                    else
                    {
                        ctrl->motion_err_valid = TRUE;
                        ctrl->motion_stall_count = 0;
                        stall_since_us = 0;
                    }

                    ctrl->motion_err_mag = motion_err;

                    if (ctrl->motion_stall_count >= 2)
                    {
                        if (stall_since_us == 0)
                            stall_since_us = now_us;
                        if ((now_us - stall_since_us) >=
                            ((gint64)ROT_CMD_STUCK_TIMEOUT_MS * 1000))
                        {
                            stopped_unexpected = TRUE;
                        }
                    }
                    else
                    {
                        stall_since_us = 0;
                    }
                }
            }
            else
            {
                ctrl->motion_err_valid = FALSE;
                ctrl->motion_stall_count = 0;
                stall_since_us = 0;
            }

            if (ctrl->tracking)
            {
                resend_due = stopped_unexpected;

                rot_target_caps_t wrap_caps = decision_caps;
                if (wrap_caps.az_wrap_mode == ROT_TARGET_WRAP_180 &&
                    (wrap_caps.az_min_deg < -180.0 || wrap_caps.az_max_deg > 180.0))
                {
                    wrap_caps.az_wrap_mode = ROT_TARGET_WRAP_360;
                }
                wrap_reason = ROT_TARGET_INVALID_NONE;
                wrap_mismatch =
                    (!rot_target_is_valid(&wrap_caps,
                                          desired_user_az,
                                          desired_user_el,
                                          NULL,
                                          &wrap_reason) &&
                     wrap_reason == ROT_TARGET_INVALID_WRAP_MISMATCH);
                wrap_bypass =
                    wrap_mismatch &&
                    (desired_state == ROT_TARGET_STATE_TRACKING_NORMAL ||
                     desired_state == ROT_TARGET_STATE_TRACKING_DEGRADED);

                if ((desired_state == ROT_TARGET_STATE_TRACKING_NORMAL ||
                     desired_state == ROT_TARGET_STATE_TRACKING_DEGRADED) &&
                    wrap_mismatch)
                {
                    if (!ctrl->wrap_acquire_active)
                    {
                        if (rotctrl_pick_wrap_candidate(ctrl,
                                                        desired_raw_az,
                                                        ref_backend,
                                                        backend_az_min,
                                                        backend_az_max,
                                                        &wrap_candidate_a,
                                                        &wrap_candidate_b,
                                                        &wrap_candidate,
                                                        &wrap_candidate_k))
                        {
                            gdouble wrap_eps = threshold_deg;
                            gdouble wrap_tol = MAX(k_meas_tol_deg, k_meas_quantum_deg);
                            gdouble diff_backend =
                                fabs(shortest_az_delta(meas_backend_az,
                                                       wrap_candidate));

                            if (rotpos_valid && diff_backend <= (wrap_eps + wrap_tol))
                            {
                                wrap_bypass = TRUE;
                            }
                            else
                            {
                                ctrl->wrap_acquire_active = TRUE;
                                ctrl->wrap_acquire_sent = FALSE;
                                ctrl->wrap_acquire_target_backend = wrap_candidate;
                                ctrl->wrap_acquire_target_az360 = desired_raw_az;
                                ctrl->wrap_acquire_target_k = wrap_candidate_k;
                                ctrl->wrap_acquire_since_us = now_us;
                                ctrl->wrap_acquire_last_log_us = 0;
                                ctrl->force_next_send = TRUE;
                                sat_log_log(SAT_LOG_LEVEL_INFO,
                                            "TRACK wrap mismatch -> starting wrap acquisition "
                                            "meas=%.2f desired_user=%.2f cand=(%.2f, %.2f) chosen=%.2f",
                                            meas_backend_az,
                                            desired_user_az,
                                            wrap_candidate_a,
                                            wrap_candidate_b,
                                            wrap_candidate);
                                rot_term_log(ctrl, "gpredict:state",
                                             "TRACK wrap mismatch -> starting wrap acquisition "
                                             "meas=%.2f desired_user=%.2f cand=(%.2f, %.2f) chosen=%.2f",
                                             meas_backend_az,
                                             desired_user_az,
                                             wrap_candidate_a,
                                             wrap_candidate_b,
                                             wrap_candidate);
                            }
                        }
                        else
                        {
                            sat_log_log(SAT_LOG_LEVEL_WARN,
                                        "TRACK wrap mismatch: no backend candidate for az=%.2f range=(%.2f..%.2f)",
                                        desired_raw_az,
                                        backend_az_min,
                                        backend_az_max);
                            rot_term_log(ctrl, "gpredict:warn",
                                         "TRACK wrap mismatch: no backend candidate for az=%.2f range=(%.2f..%.2f)",
                                         desired_raw_az,
                                         backend_az_min,
                                         backend_az_max);
                        }
                    }
                }
            }

            {
                rot_cmd_decision_input_t decision_in = { 0 };
                rot_target_caps_t caps_for_decision = decision_caps;

                if (caps_for_decision.az_wrap_mode == ROT_TARGET_WRAP_180 &&
                    (caps_for_decision.az_min_deg < -180.0 ||
                     caps_for_decision.az_max_deg > 180.0))
                {
                    caps_for_decision.az_wrap_mode = ROT_TARGET_WRAP_360;
                }
                if (seam_cross_active &&
                    caps_for_decision.az_wrap_mode == ROT_TARGET_WRAP_180 &&
                    (caps_for_decision.az_min_deg < -180.0 ||
                     caps_for_decision.az_max_deg > 180.0))
                {
                    caps_for_decision.az_wrap_mode = ROT_TARGET_WRAP_360;
                }
                if ((ctrl->wrap_acquire_active || wrap_bypass) &&
                    caps_for_decision.az_wrap_mode == ROT_TARGET_WRAP_180)
                {
                    caps_for_decision.az_wrap_mode = ROT_TARGET_WRAP_360;
                }

                decision_in.mode =
                    park_active
                        ? ROT_CMD_MODE_PARK
                        : ((desired_state == ROT_TARGET_STATE_PRETRACK)
                               ? ROT_CMD_MODE_PRETRACK
                               : ROT_CMD_MODE_TRACKING);
                decision_in.allow_send = allow_send;
                decision_in.have_target = have_target;
                decision_in.force_send = force_send;
                decision_in.stale_hold = ctrl->stale_hold_active;
                decision_in.setpoint_valid = ctrl->setpoint_valid;
                decision_in.pos_fresh = pos_fresh;
                decision_in.last_good_age_ms = pos_age_ms;
                decision_in.not_at_target = not_at_target;
                decision_in.moving_toward = moving_toward;
                decision_in.stopped_unexpected = stopped_unexpected;
                decision_in.pos_recovered = ctrl->stale_recovered_pulse;
                decision_in.resend_due = resend_due;
                decision_in.now_us = now_us;
                decision_in.last_send_us = ctrl->last_send_us;
                decision_in.min_interval_us = ROT_SEND_MIN_INTERVAL_US;
                decision_in.desired_user_az = desired_user_az;
                decision_in.desired_user_el = desired_user_el;
                decision_in.desired_backend_az = desired_backend_az;
                decision_in.desired_backend_el = desired_backend_el;
                decision_in.setpoint_user_az = setpoint_user_az;
                decision_in.setpoint_user_el = setpoint_user_el;
                decision_in.last_cmd_user_az = last_cmd_user_az_log;
                decision_in.last_cmd_user_el = last_cmd_user_el_log;
                decision_in.delta_backend_az = delta_backend_az;
                decision_in.delta_backend_el = delta_backend_el;
                decision_in.deadband_az = deadband_az;
                decision_in.deadband_el = deadband_el;
                decision_in.min_step_az = threshold_deg;
                decision_in.min_step_el = threshold_deg;
                decision_in.target_change_az = deadband_az;
                decision_in.target_change_el = deadband_el;
                decision_in.caps = caps_for_decision;

                rot_cmd_decision_eval(&decision_in, &decision_out);
                send_ok = decision_out.send;
                reason = decision_out.reason;
                gate = decision_out.action;
                delta_user_az = decision_out.delta_user_az;
                delta_user_el = decision_out.delta_user_el;
                if (reason == ROT_CMD_REASON_RANGE && wrap_resolve_failed)
                    decision_out.range_reason = ROT_TARGET_INVALID_WRAP_MISMATCH;

                if (send_ok &&
                    !(force_send || resend_due || min_step_exceeded))
                {
                    send_ok = FALSE;
                    gate = ROT_CMD_ACTION_SUPPRESS;
                    reason = ROT_CMD_REASON_MIN_STEP;
                }
            }
        }
        else
        {
            stall_since_us = 0;
            if (ctrl->last_cmd_valid)
            {
                gdouble d_az_last = rot_ang_dist_deg(meas_az360, ctrl->last_cmd_az360);
                gdouble d_el_last = fabs(meas_el - ctrl->last_cmd_el);
                not_at_target = (d_az_last > eps_az || d_el_last > eps_el);
            }

            {
                gboolean manual_target_hold = FALSE;

                if (!ctrl->tracking && !force_send && ctrl->setpoint_valid &&
                    delta_user_az < deadband_az &&
                    delta_user_el < deadband_el)
                {
                    manual_target_hold = TRUE;
                }

                if (manual_target_hold && !resend_due && !stopped_unexpected)
                {
                    send_ok = FALSE;
                    sched_reason = "TARGET_HOLD";
                }
                else
                {
                    send_ok = force_send ||
                              rot_should_send(ctrl,
                                              meas_az360,
                                              meas_el,
                                              raw_cmd_az,
                                              raw_cmd_el,
                                              eps_az,
                                              eps_el,
                                              resend_due,
                                              not_at_target,
                                              now_us,
                                              ROT_SEND_MIN_INTERVAL_US,
                                              &sched_reason);
                }
            }

            if (!have_target)
            {
                gate = ROT_CMD_ACTION_SUPPRESS;
                reason = ROT_CMD_REASON_NO_TARGET;
            }
            else if (!allow_send)
            {
                gate = ROT_CMD_ACTION_SUPPRESS;
                reason = ROT_CMD_REASON_BLOCKED;
            }
            else if (send_ok)
            {
                gate = ROT_CMD_ACTION_SEND;
                reason = force_send ? ROT_CMD_REASON_FORCE
                                    : (resend_due ? ROT_CMD_REASON_RESEND
                                                  : ROT_CMD_REASON_TARGET);
            }
            else
            {
                gate = ROT_CMD_ACTION_SUPPRESS;
                reason = ROT_CMD_REASON_DEADBAND;
            }

            if (send_ok &&
                !(force_send || resend_due || min_step_exceeded))
            {
                send_ok = FALSE;
                gate = ROT_CMD_ACTION_SUPPRESS;
                reason = ROT_CMD_REASON_MIN_STEP;
            }
        }

        g_mutex_lock(&ctrl->client.mutex);
        ctrl->client.stall_since_us = stall_since_us;
        g_mutex_unlock(&ctrl->client.mutex);

        if (g_mutex_trylock(&ctrl->client.mutex))
        {
            ctrl->client.desired_user_az = desired_user_az;
            ctrl->client.desired_user_el = desired_user_el;
            ctrl->client.desired_backend_az = cmdaz;
            ctrl->client.desired_backend_el = cmdel;
            ctrl->client.desired_raw_az = raw_cmd_az;
            ctrl->client.desired_raw_el = raw_cmd_el;
            ctrl->client.desired_valid = have_target;
            ctrl->client.desired_allow = allow_send;
            ctrl->client.desired_tracking = ctrl->tracking;
            ctrl->client.desired_update_us = now_us;
            ctrl->client.allow_send_no_pos = allow_no_pos_manual;
            if (force_send && !ctrl->client.force_pending)
            {
                ctrl->client.force_pending = TRUE;
                if (ctrl->force_next_send)
                    ctrl->force_next_send = FALSE;
            }
            g_mutex_unlock(&ctrl->client.mutex);
        }

        if (allow_send && !send_ok && ctrl->verbose_logging)
        {
            if (now_us - ctrl->last_hold_log_us >= G_USEC_PER_SEC)
            {
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            "rot hold: reason=%s d_az=%.2f d_el=%.2f eps=(%.2f, %.2f)",
                            rot_cmd_reason_name(reason),
                            delta_az_phys,
                            delta_el_phys,
                            eps_az,
                            eps_el);
                ctrl->last_hold_log_us = now_us;
            }
        }

        if (reason == ROT_CMD_REASON_NO_POS && !pos_fresh && ctrl->tracking)
        {
            rot_log_rate_limited(ctrl, &ctrl->no_pos_gate_rate,
                                 ROTCTLD_FAILURE_LOG_INTERVAL_US,
                                 SAT_LOG_LEVEL_WARN, "gpredict:warn",
                                 "rot_cmd_gate: HOLD no_pos pos_fresh=0 last_good_age_ms=%lld",
                                 (long long)pos_age_ms);
        }

        if (ctrl->verbose_logging)
        {
            const gchar *mode_name =
                park_active ? "PARK" : (ctrl->tracking ? "TRACK" : "IDLE");
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rot_decision: mode=%s allow=%d have_target=%d force=%d stale_hold=%d "
                        "setpoint_valid=%d in_flight=%d pos_fresh=%d last_good_age_ms=%lld "
                        "desired_user=(%.2f,%.2f) setpoint_user=(%.2f,%.2f) last_cmd_user=(%.2f,%.2f) "
                        "delta_user=(%.2f,%.2f) deadband=(%.2f,%.2f) min_step=(%.2f,%.2f) action=%s reason=%s",
                        mode_name,
                        allow_send ? 1 : 0,
                        have_target ? 1 : 0,
                        force_send ? 1 : 0,
                        ctrl->stale_hold_active ? 1 : 0,
                        ctrl->setpoint_valid ? 1 : 0,
                        not_at_target ? 1 : 0,
                        pos_fresh ? 1 : 0,
                        (long long)pos_age_ms,
                        desired_user_az, desired_user_el,
                        setpoint_user_az, setpoint_user_el,
                        last_cmd_user_az_log, last_cmd_user_el_log,
                        delta_user_az, delta_user_el,
                        deadband_az, deadband_el,
                        threshold_deg, threshold_deg,
                        rot_cmd_action_name(gate),
                        rot_cmd_reason_name(reason));
        }

        if (allow_send && send_ok && ctrl->setpoint_valid &&
            fabs(delta_user_az) < 1e-6 && fabs(delta_user_el) < 1e-6 &&
            fabs(delta_backend_az) < 1e-6 && fabs(delta_backend_el) < 1e-6)
        {
            sat_log_log(SAT_LOG_LEVEL_WARN,
                        "rot_cmd_gate: SEND with zero delta reason=%s",
                        rot_cmd_reason_name(reason));
        }

        if (ctrl->tracking)
        {
            const gchar *mode_name =
                (desired_state == ROT_TARGET_STATE_PRETRACK) ? "PRETRACK" : "TRACKING";
            const gchar *action = rot_cmd_action_name(gate);
            const gchar *range_reason =
                (reason == ROT_CMD_REASON_RANGE)
                    ? rot_target_invalid_reason_name(decision_out.range_reason)
                    : "none";

            rot_term_log_verbose(ctrl, "gpredict:tx",
                                 "rot_cmd_gate: mode=%s desired_user=(%.2f,%.2f) setpoint_user=(%.2f,%.2f) "
                                 "last_cmd_user=(%.2f,%.2f) desired_backend=(%.2f,%.2f) setpoint_backend=(%.2f,%.2f) "
                                 "in_flight=%d stale_hold=%d pos_fresh=%d last_good_age_ms=%lld "
                                 "delta_user=(%.2f,%.2f) delta_backend=(%.2f,%.2f) action=%s reason=%s range=%s",
                                 mode_name,
                                 desired_user_az, desired_user_el,
                                 setpoint_user_az, setpoint_user_el,
                                 last_cmd_user_az_log, last_cmd_user_el_log,
                                 desired_backend_az, desired_backend_el,
                                 setpoint_backend_az, setpoint_backend_el,
                                 ctrl->setpoint_valid ? 1 : 0,
                                 ctrl->stale_hold_active ? 1 : 0,
                                 pos_fresh ? 1 : 0,
                                 (long long)pos_age_ms,
                                 delta_user_az, delta_user_el,
                                 delta_backend_az, delta_backend_el,
                                 action,
                                 rot_cmd_reason_name(reason),
                                 range_reason);
        }

        if (have_target)
        {
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        "tracking_decision: pred=(%.2f,%.2f) user=(%.2f,%.2f) "
                        "backend=(%.2f,%.2f) cur_backend=(%.2f,%.2f) "
                        "cur_user=(%.2f,%.2f) eps=(%.2f,%.2f) send=%d reason=%s "
                        "wrap=%s offsets=(%.2f,%.2f)",
                        az_pred, el_pred,
                        log_az_user, desired_user_el,
                        cmdaz, cmdel,
                        meas_backend_az, meas_backend_el,
                        log_cur_user_az, log_cur_user_el,
                        eps_az, eps_el,
                        (allow_send && send_ok) ? 1 : 0,
                        rot_cmd_reason_name(reason),
                        wrap_name,
                        log_offset_az, log_offset_el);
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        "trk: state=%s pred=%.2f/%.2f user=%.2f/%.2f cmd=%.2f/%.2f meas=%.2f/%.2f fresh=%d gate=%s reason=%s",
                        rot_target_state_name(ctrl->target_state),
                        az_pred, el_pred,
                        log_az_user, desired_user_el,
                        cmdaz, cmdel,
                        meas_backend_az, meas_backend_el,
                        fresh_feedback ? 1 : 0,
                        rot_cmd_action_name(gate),
                        rot_cmd_reason_name(reason));
        }

        if (now_us - ctrl->last_tick_log_us >= 1000000)
        {
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        "rotor tick target=(%.2f, %.2f) sent=(%.2f, %.2f) "
                        "delta=(%.2f, %.2f) engaged=%d reason=%s",
                        raw_cmd_az, raw_cmd_el,
                        cmdaz, cmdel,
                        cmdaz - last_out_az,
                        cmdel - last_out_el,
                        ctrl->engaged ? 1 : 0,
                        rot_cmd_reason_name(reason));
            ctrl->last_tick_log_us = now_us;
        }

        if (allow_send && send_ok &&
            g_mutex_trylock(&ctrl->client.mutex))
        {
            ctrl->client.azi_out = cmdaz;
            ctrl->client.ele_out = cmdel;
            ctrl->client.raw_azi_out = raw_cmd_az;
            ctrl->client.raw_ele_out = raw_cmd_el;
            ctrl->client.new_trg = TRUE;
            ctrl->client.allow_send_no_pos = allow_no_pos_manual;
            ctrl->client.use_setpos = cal_force_send;
            ctrl->client.apply_calib = FALSE;
            ctrl->client.last_cmd_us = now_us;
            if (reason == ROT_CMD_REASON_STOPPED ||
                reason == ROT_CMD_REASON_RESEND)
                ctrl->client.stall_since_us = now_us;
            g_mutex_unlock(&ctrl->client.mutex);

            ctrl->pending_lane_valid = FALSE;
            ctrl->pending_lane_since_us = 0;
            ctrl->last_send_us = now_us;
            ctrl->above_eps_count = 0;

            ctrl->az_abs_last_cmd = az_abs_cmd;
            ctrl->track_policy.last_cmd_az = az_abs_cmd;
            ctrl->track_policy.last_cmd_valid = TRUE;
            ctrl->last_cmd_time_us = now_us;
            ctrl->last_cmd_az360 = raw_cmd_az;
            ctrl->last_cmd_el = raw_cmd_el;
            ctrl->last_cmd_valid = TRUE;
            ctrl->last_cmd_backend_az = cmdaz;
            ctrl->last_cmd_backend_el = cmdel;
            ctrl->last_cmd_backend_valid = TRUE;
            ctrl->setpoint_backend_az = desired_backend_az;
            ctrl->setpoint_backend_el = desired_backend_el;
            ctrl->setpoint_user_az = desired_user_az;
            ctrl->setpoint_user_el = desired_user_el;
            ctrl->setpoint_valid = TRUE;
            ctrl->force_next_send = FALSE;
            ctrl->motion_err_valid = FALSE;
            ctrl->motion_stall_count = 0;
            if (reason == ROT_CMD_REASON_PARK)
            {
                rot_term_log(ctrl, "gpredict:tx",
                             "send target park user=(%.2f,%.2f) backend=(%.2f,%.2f) reason=park",
                             desired_user_az, desired_user_el,
                             cmdaz, cmdel);
                ctrl->park_requested = FALSE;
            }
            if (!pos_fresh || reason == ROT_CMD_REASON_RESEND)
                ctrl->last_keepalive_time_us = now_us;

            if (seam_cross_active && seam_cmd_valid)
            {
                if (!ctrl->seam_crossing_sent)
                {
                    sat_log_log(SAT_LOG_LEVEL_INFO,
                                "pretrack seam crossing move cmd=%.2f target=%.2f k=%d",
                                cmdaz,
                                desired_raw_az,
                                seam_lane_k);
                    rot_term_log(ctrl, "gpredict:tx",
                                 "pretrack seam crossing move cmd=%.2f target=%.2f k=%d",
                                 cmdaz,
                                 desired_raw_az,
                                 seam_lane_k);
                }
                ctrl->seam_crossing_sent = TRUE;
                ctrl->seam_crossing_lane_k = seam_lane_k;
                ctrl->seam_crossing_lane_valid = TRUE;
            }
            if (ctrl->wrap_acquire_active)
            {
                ctrl->wrap_acquire_sent = TRUE;
            }

            rot_log_rate_limited(ctrl, &ctrl->send_log_rate, 1000000,
                                 SAT_LOG_LEVEL_INFO, "gpredict:tx",
                                 "send target az_pred=%.2f az_user=%.2f az_backend=%.2f "
                                 "el_pred=%.2f el_backend=%.2f state=%s reason=%s",
                                 log_az_pred, log_az_user, cmdaz,
                                 log_el_pred, cmdel,
                                 rot_target_state_name(ctrl->target_state),
                                 rot_cmd_reason_name(reason));
        }

        if (ctrl->verbose_logging)
        {
            if (safety_clamped)
                rot_term_log_verbose(ctrl, "gpredict:warn",
                                     "safety clamp: cur=%.2f cmd=%.2f",
                                     ctrl->az_abs_cur, az_abs_cmd);
            else if (safety_avoided)
                rot_term_log_verbose(ctrl, "gpredict:warn",
                                     "safety stop avoided: cur=%.2f cmd=%.2f",
                                     ctrl->az_abs_cur, az_abs_cmd);
        }

        /* keep knobs in sync with the commanded position (logical) */
        gtk_rot_knob_set_value(GTK_ROT_KNOB(ctrl->AzSet), setaz);
        gtk_rot_knob_set_value(GTK_ROT_KNOB(ctrl->ElSet), setel);

        if (ctrl->verbose_logging)
        {
            gint64 log_now = g_get_monotonic_time();
            if (log_now - ctrl->last_debug_log_us >= 1000000)
            {
                gdouble seam_log = ctrl->seam_valid ? ctrl->seam_az360 : -1.0;
                gdouble pos360_log = rotpos_valid ? meas_az360 : -1.0;
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            "rotor map pred=%.2f/%.2f tgt360=%.2f seam=%.2f "
                            "raw=(%.2f, %.2f) user=%.2f state=%s span=%s "
                            "azlim=(%.2f, %.2f) cmd=(%.2f, %.2f) "
                            "clamp=az:%d el:%d reason=%s cmd_send=(%.2f, %.2f) "
                            "meas_b=(%.2f, %.2f) pos360=%.2f dphy=(%.2f, %.2f) "
                            "hold=%d",
                            az_pred, el_pred,
                            raw_cmd_az,
                            seam_log,
                            raw_cmd_az, raw_cmd_el,
                            log_az_user,
                            rot_target_state_name(ctrl->target_state),
                            rotctrl_span_mode_name(user_span_cfg.az_mode),
                            user_span_cfg.az_min, user_span_cfg.az_max,
                            pipeline.cmd_az, pipeline.cmd_el,
                            az_clamped ? 1 : 0,
                            el_clamped ? 1 : 0,
                            rot_cmd_reason_name(reason),
                            cmdaz, cmdel,
                            meas_backend_az, meas_backend_el,
                            pos360_log,
                            delta_az_phys, delta_el_phys,
                            ctrl->az_hold_active ? 1 : 0);
                ctrl->last_debug_log_us = log_now;
            }
        }

        /* check error status
         *
         * We treat errors as cumulative for the duration of an engage cycle:
         *  - Any iteration that sees an I/O error bumps errcnt.
         *  - We do NOT reset errcnt back to zero on a "clean" loop; this avoids
         *    races between the worker thread and the UI thread where a transient
         *    read of io_error==FALSE could clear the accumulated error state.
         *  - errcnt is explicitly reset when (re)engaging the rotor.
         */
        if (error)
        {
            if (ctrl->errcnt < G_MAXINT)
                ctrl->errcnt++;
        }
        else
        {
            ctrl->errcnt = 0;
        }

        if (ctrl->errcnt == MAX_ERROR_COUNT && ctrl->engaged)
        {
            sat_log_log(SAT_LOG_LEVEL_WARN,
                        _("%s: MAX_ERROR_COUNT (%d) reached. Keeping tracking alive, link down."),
                        __func__, MAX_ERROR_COUNT);
            rotctrl_set_ui_status_detail(ctrl, "LINK DOWN");
        }

        rotctrl_update_session_state(ctrl, pos_recent, pos_unknown,
                                     pos_cmd_ok, error,
                                     last_pos_us, pos_failures);

        /* update status label if present, based on data and feedback mode */
        if (status_label)
        {
            const gchar *status_text = NULL;
            const gdouble status_eps = rotctrl_angle_epsilon(ctrl);
            gboolean has_error = error || (ctrl->errcnt > 0);
            gboolean stale_only_degraded = FALSE;
            gboolean degraded_raw = FALSE;
            gboolean degraded_for_ui = FALSE;
            gboolean stuck_feedback = FALSE;
            gboolean degraded_clear_ok = FALSE;
            gboolean assume_standby_no_user_cmd = FALSE;
            gdouble status_target_az = az_abs_cmd;
            gdouble status_target_el = cmdel;
            gboolean status_moving = FALSE;
            gboolean pretrack_hold = FALSE;
            gboolean standby_state = FALSE;
            gboolean manual_setpoint_no_feedback = FALSE;
            RotorStateSnapshot snap = { 0 };
            guint discrepancy_streak = 0;
            gboolean discrepancy_pending = FALSE;

            if (ctrl->setpoint_valid)
            {
                status_target_az = setpoint_backend_az;
                status_target_el = setpoint_backend_el;
            }

            if (rotpos_valid)
            {
                gdouble status_az_err = backend_span_extended
                                        ? fabs(status_target_az - ctrl->az_abs_cur)
                                        : fabs(shortest_az_delta(status_target_az,
                                                                 ctrl->az_abs_cur));
                gdouble status_el_err = fabs(status_target_el - rotel_backend);
                gdouble status_eps_exit = status_eps;
                gdouble status_eps_enter = status_eps_exit * 1.35;

                if (ctrl->conf &&
                    ctrl->conf->axis_mode == ROT_AXIS_MODE_AZ_ONLY)
                    status_el_err = 0.0;

                if (ctrl->threshold > 0.0 && status_eps_exit < ctrl->threshold)
                    status_eps_exit = ctrl->threshold;
                if (status_eps_enter < (status_eps_exit + 0.2))
                    status_eps_enter = status_eps_exit + 0.2;

                if (ctrl->ui_status == ROTOR_UI_STATUS_MOVING)
                    status_moving = (status_az_err > status_eps_exit ||
                                     status_el_err > status_eps_exit);
                else
                    status_moving = (status_az_err > status_eps_enter ||
                                     status_el_err > status_eps_enter);
            }

            g_mutex_lock(&ctrl->client.mutex);
            discrepancy_streak = ctrl->client.discrepancy_streak;
            discrepancy_pending = ctrl->client.discrepancy_pending;
            g_mutex_unlock(&ctrl->client.mutex);
            stuck_feedback = (discrepancy_pending &&
                              discrepancy_streak >= ROT_CMD_DISCREPANCY_STRIKES);

            stale_only_degraded = (ctrl->session_state == ROT_SESSION_DEGRADED &&
                                   ctrl->pos_stale_hyst_active &&
                                   !ctrl->tracking &&
                                   !ctrl->stale_hold_active &&
                                   !stuck_feedback &&
                                   comm_status != ROT_COMM_DEGRADED &&
                                   !has_error &&
                                   !ctrl->ui_hard_error);
            degraded_raw = ((comm_status == ROT_COMM_DEGRADED) ||
                            ((ctrl->session_state == ROT_SESSION_DEGRADED) &&
                             !stale_only_degraded) ||
                            has_error);
            if (degraded_raw)
            {
                ctrl->ui_degraded_latched = TRUE;
                ctrl->ui_degraded_clear_hits = 0;
            }
            else if (ctrl->ui_degraded_latched)
            {
                degraded_clear_ok = (ctrl->engaged &&
                                     ctrl->session_state == ROT_SESSION_READY &&
                                     comm_status == ROT_COMM_STANDBY &&
                                     !ctrl->pos_stale_hyst_active &&
                                     !ctrl->stale_hold_active &&
                                     !stuck_feedback &&
                                     pos_fresh &&
                                     !has_error &&
                                     !ctrl->ui_hard_error);
                if (degraded_clear_ok)
                {
                    if (ctrl->ui_degraded_clear_hits < G_MAXUINT)
                        ctrl->ui_degraded_clear_hits++;
                    if (ctrl->ui_degraded_clear_hits >=
                        ROT_UI_DEGRADED_CLEAR_POLLS)
                    {
                        ctrl->ui_degraded_latched = FALSE;
                        ctrl->ui_degraded_clear_hits = 0;
                    }
                }
                else
                {
                    ctrl->ui_degraded_clear_hits = 0;
                }
            }

            degraded_for_ui = degraded_raw || ctrl->ui_degraded_latched;
            if (stuck_feedback)
            {
                degraded_for_ui = TRUE;
                status_moving = FALSE;
            }

            assume_standby_no_user_cmd =
                (ctrl->engaged &&
                 !ctrl->tracking &&
                 !ctrl->park_requested &&
                 !ctrl->have_user_command &&
                 !ctrl->cal_active &&
                 !ctrl->cal_hold_active);
            if (assume_standby_no_user_cmd)
                status_moving = FALSE;

            manual_setpoint_no_feedback =
                (ctrl->engaged &&
                 !ctrl->tracking &&
                 ctrl->setpoint_valid &&
                 !rotpos_valid &&
                 !assume_standby_no_user_cmd &&
                 !ctrl->cal_active &&
                 !ctrl->cal_hold_active);
            if (manual_setpoint_no_feedback)
                status_moving = TRUE;

            pretrack_hold = (ctrl->tracking &&
                             ctrl->target_state == ROT_TARGET_STATE_PRETRACK &&
                             !status_moving);

            if (!ctrl->engaged)
                status_text = _("DISENGAGED");
            else if (comm_status == ROT_COMM_LINK_LOST || has_error)
                status_text = _("LINK LOST");
            else if (degraded_for_ui)
                status_text = _("DEGRADED FEEDBACK");
            else if (ctrl->session_state == ROT_SESSION_CONNECTING)
                status_text = _("CONNECTING");
            else if (ctrl->session_state == ROT_SESSION_ENGAGING)
                status_text = _("ENGAGING");
            else if (pretrack_hold)
                status_text = _("PRETRACK");
            else if (ctrl->tracking &&
                     ctrl->target_state == ROT_TARGET_STATE_PRETRACK)
                status_text = _("MOVING");
            else if (ctrl->tracking &&
                     ctrl->target_state == ROT_TARGET_STATE_TRACKING_DEGRADED)
                status_text = _("MOVING");
            else if (assume_standby_no_user_cmd)
            {
                status_text = _("STANDBY");
                standby_state = TRUE;
            }
            else if (status_moving)
                status_text = _("MOVING");
            else if (ctrl->engaged &&
                     !ctrl->tracking &&
                     !ctrl->setpoint_valid &&
                     !ctrl->park_requested)
            {
                status_text = _("STANDBY");
                standby_state = TRUE;
            }
            else
                status_text = _("ON TARGET");

            rotctrl_set_ui_status_detail(ctrl, status_text);

            snap.control_active = (ctrl->engaged || ctrl->engage_pending ||
                                   ctrl->ui_hard_error);
            snap.engaging = (ctrl->engage_pending ||
                             ctrl->session_state == ROT_SESSION_CONNECTING ||
                             ctrl->session_state == ROT_SESSION_ENGAGING);
            snap.hard_error = ctrl->ui_hard_error;
            snap.link_lost = (comm_status == ROT_COMM_LINK_LOST);
            snap.degraded = degraded_for_ui;
            snap.moving = (!degraded_for_ui &&
                           !assume_standby_no_user_cmd &&
                           !pretrack_hold &&
                           ((ctrl->tracking &&
                             (ctrl->target_state == ROT_TARGET_STATE_PRETRACK ||
                              ctrl->target_state ==
                              ROT_TARGET_STATE_TRACKING_DEGRADED)) ||
                            status_moving));
            snap.pretracking = pretrack_hold;
            snap.on_target = (ctrl->engaged &&
                              !snap.moving &&
                              !snap.pretracking &&
                              !standby_state);
            rotctrl_apply_ui_status(ctrl, &snap, "poll");

            char cmdaz_str[32];
            char cmdel_str[32];
            char rotaz_str[32];
            char rotel_str[32];

            rot_format_deg_2(cmdaz_str, sizeof(cmdaz_str), cmdaz);
            rot_format_deg_2(cmdel_str, sizeof(cmdel_str), cmdel);
            gdouble rotaz_display = rotpos_valid
                                    ? az_abs_to_span(ctrl->az_abs_cur, ctrl->span_mode)
                                    : az_norm_span(rotaz, ctrl->span_mode);
            rot_format_deg_2(rotaz_str, sizeof(rotaz_str), rotaz_display);
            rot_format_deg_2(rotel_str, sizeof(rotel_str), rotel);

            if (ctrl->verbose_logging)
                sat_log_log(SAT_LOG_LEVEL_INFO,
                            "rotor status=%s set=(%s, %s) rot=(%s, %s) error=%d",
                            status_text,
                            cmdaz_str, cmdel_str,
                            rotaz_str, rotel_str,
                            error ? 1 : 0);
        }
    }
    else
    {
        /* client not running: ensure rotor pos is not visible */
        rotctrl_set_rotor_pos_on_plots(ctrl, -10.0, -10.0);
        ctrl->axis_swap_warned = FALSE;
        gtk_label_set_text(GTK_LABEL(ctrl->AzRead), "\342\200\224");
        gtk_label_set_text(GTK_LABEL(ctrl->ElRead), "\342\200\224");

        if (status_label)
        {
            const gchar *status_text = NULL;
            RotorStateSnapshot snap = { 0 };

            rotctrl_update_session_state(ctrl, FALSE, FALSE, FALSE, FALSE,
                                         0, 0);

            if (ctrl->session_state == ROT_SESSION_CONNECTING)
                status_text = _("CONNECTING");
            else if (ctrl->session_state == ROT_SESSION_ENGAGING)
                status_text = _("ENGAGING");
            else if (ctrl->session_state == ROT_SESSION_DEGRADED)
                status_text = _("DEGRADED");
            else
                status_text = _("DISENGAGED");

            rotctrl_set_ui_status_detail(ctrl, status_text);

            snap.control_active = (ctrl->engaged || ctrl->engage_pending ||
                                   ctrl->ui_hard_error);
            snap.engaging = (ctrl->engage_pending ||
                             ctrl->session_state == ROT_SESSION_CONNECTING ||
                             ctrl->session_state == ROT_SESSION_ENGAGING);
            snap.hard_error = ctrl->ui_hard_error;
            snap.link_lost = (ctrl->engaged &&
                              ctrl->comm_status == ROT_COMM_LINK_LOST);
            snap.degraded = (ctrl->session_state == ROT_SESSION_DEGRADED);
            snap.moving = FALSE;
            snap.on_target = FALSE;
            rotctrl_apply_ui_status(ctrl, &snap, "poll no-client");
        }
    }

    rotctrl_autocal_tick(ctrl, rotpos_valid, rotaz_backend, rotel_backend);

    /* update target object on polar plot */
    if (ctrl->target != NULL)
    {
        gdouble target_plot_az = ctrl->target->az;
        gdouble target_plot_el = ctrl->target->el;

        /* Keep the target marker in sync with the same live predictor used by
         * tracking logic; ctrl->target fields can lag behind in some sessions.
         */
        if (ctrl->tracking && live_valid)
        {
            target_plot_az = live_az360;
            target_plot_el = live_el;
        }

        rotctrl_set_target_pos_on_plots(ctrl, target_plot_az, target_plot_el);
    }

    /* update controller circle on polar plot */
    if (ctrl->conf != NULL)
    {
        gdouble dispaz = cmdaz_plot;
        gdouble dispel = cmdel_plot;

        if (!ctrl->client.running)
        {
            dispaz = gtk_rot_knob_get_value(GTK_ROT_KNOB(ctrl->AzSet));
            dispel = gtk_rot_knob_get_value(GTK_ROT_KNOB(ctrl->ElSet));
            if (ctrl->conf->aztype == ROT_AZ_TYPE_180 && dispaz < 0.0)
                dispaz += 360.0;
        }

        rotctrl_set_ctrl_pos_on_plots(ctrl, dispaz, dispel);
        rotctrl_queue_draw_plots(ctrl);
    }

    (void)cmd_rejected;
    (void)have_last_cmd_phys;
    return TRUE;
}
/**
 * Manage cycle delay changes.
 *
 * \param spin Pointer to the spin button.
 * \param data Pointer to the GtkRotCtrl widget.
 * 
 * This function is called when the user changes the value of the
 * cycle delay.
 */
static void delay_changed_cb(GtkSpinButton * spin, gpointer data)
{
    GtkRotCtrl     *ctrl = GTK_ROT_CTRL(data);

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    ctrl->delay = (guint) gtk_spin_button_get_value(spin);
    if (ctrl->conf)
    {
        ctrl->conf->cycle = ctrl->delay;
        ctrl->conf->rotor_poll_period_ms = ctrl->delay;
    }

    if (ctrl->delay > 1000)
    {
        if (!ctrl->cycle_perf_warned)
        {
            rot_show_message(ctrl, GTK_MESSAGE_WARNING,
                             _("Cycle delay warning"),
                             _("Cycle delay above 1000 ms may cause poor tracking performance."));
            ctrl->cycle_perf_warned = TRUE;
        }
    }
    else
    {
        ctrl->cycle_perf_warned = FALSE;
    }

    if (ctrl->timerid > 0)
        g_source_remove(ctrl->timerid);

    ctrl->timerid = g_timeout_add(ctrl->delay, rot_ctrl_timeout_cb, ctrl);
}

/**
 * Manage threshold changes
 *
 * \param spin Pointer to the spin button.
 * \param data Pointer to the GtkRotCtrl widget.
 * 
 * This function is called when the user changes the value of the
 * tolerance.
 */
static void threshold_changed_cb(GtkSpinButton * spin, gpointer data)
{
    GtkRotCtrl     *ctrl = GTK_ROT_CTRL(data);

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    ctrl->threshold = gtk_spin_button_get_value(spin);
    if (ctrl->conf)
        ctrl->conf->threshold = ctrl->threshold;
}

static gboolean rotctrl_parse_spin_value(GtkSpinButton *spin, gdouble *value)
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
static gboolean rotor_apply_ui_settings(GtkRotCtrl *ctrl, gboolean strict)
{
    GtkSpinButton *cycle_spin;
    GtkSpinButton *thld_spin;
    GtkAdjustment *adj;
    gdouble raw = 0.0;
    gdouble lower;
    gdouble upper;
    gdouble value;
    guint delay_ms;
    gdouble threshold_deg = 0.0;

    if (ctrl == NULL)
        return TRUE;

    cycle_spin = ctrl->cycle_spin ? GTK_SPIN_BUTTON(ctrl->cycle_spin) : NULL;
    thld_spin = ctrl->thld_spin ? GTK_SPIN_BUTTON(ctrl->thld_spin) : NULL;

    if (cycle_spin)
    {
        adj = gtk_spin_button_get_adjustment(cycle_spin);
        lower = gtk_adjustment_get_lower(adj);
        upper = gtk_adjustment_get_upper(adj);

        if (strict)
        {
            if (!rotctrl_parse_spin_value(cycle_spin, &raw))
            {
                rot_show_message(ctrl, GTK_MESSAGE_ERROR,
                                 _("Invalid cycle delay"),
                                 _("Cycle delay must be a valid number."));
                return FALSE;
            }
            if (raw < lower || raw > upper)
            {
                gchar *msg = g_strdup_printf(_("Cycle delay must be between %.0f and %.0f ms."),
                                             lower, upper);
                rot_show_message(ctrl, GTK_MESSAGE_ERROR,
                                 _("Invalid cycle delay"), msg);
                g_free(msg);
                return FALSE;
            }
            gtk_spin_button_set_value(cycle_spin, raw);
        }

        gtk_spin_button_update(cycle_spin);
        value = gtk_spin_button_get_value(cycle_spin);
        delay_ms = (guint)llround(value);
    }
    else
    {
        delay_ms = ctrl->delay;
    }

    if (cycle_spin)
    {
        ctrl->delay = delay_ms;
        if (ctrl->conf)
        {
            ctrl->conf->cycle = ctrl->delay;
            ctrl->conf->rotor_poll_period_ms = ctrl->delay;
        }

        if (ctrl->delay > 1000)
        {
            if (!ctrl->cycle_perf_warned)
            {
                rot_show_message(ctrl, GTK_MESSAGE_WARNING,
                                 _("Cycle delay warning"),
                                 _("Cycle delay above 1000 ms may cause poor tracking performance."));
                ctrl->cycle_perf_warned = TRUE;
            }
        }
        else
        {
            ctrl->cycle_perf_warned = FALSE;
        }

        if (ctrl->timerid > 0)
            g_source_remove(ctrl->timerid);

        ctrl->timerid = g_timeout_add(ctrl->delay, rot_ctrl_timeout_cb, ctrl);
    }

    if (thld_spin)
    {
        adj = gtk_spin_button_get_adjustment(thld_spin);
        lower = gtk_adjustment_get_lower(adj);
        upper = gtk_adjustment_get_upper(adj);

        gtk_spin_button_update(thld_spin);
        threshold_deg = gtk_spin_button_get_value(thld_spin);

        if (strict)
        {
            if (!isfinite(threshold_deg))
            {
                rot_show_message(ctrl, GTK_MESSAGE_ERROR,
                                 _("Invalid threshold"),
                                 _("Tracking threshold must be a valid number."));
                return FALSE;
            }
            if (threshold_deg <= 0.0 || threshold_deg > 180.0)
            {
                gchar *msg = g_strdup_printf(_("Tracking threshold must be between %.2f and %.2f degrees."),
                                             MAX(0.01, lower), MIN(180.0, upper));
                rot_show_message(ctrl, GTK_MESSAGE_ERROR,
                                 _("Invalid threshold"), msg);
                g_free(msg);
                return FALSE;
            }
        }

        ctrl->threshold = threshold_deg;
        if (ctrl->conf)
            ctrl->conf->threshold = ctrl->threshold;
    }

    if (strict && cycle_spin && thld_spin)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "Applied rotor settings: cycle_ms=%u threshold_deg=%.2f",
                    ctrl->delay, ctrl->threshold);
        rot_term_log(ctrl, "gpredict:rx",
                     "applied rotor settings: cycle_ms=%u threshold_deg=%.2f",
                     ctrl->delay, ctrl->threshold);
    }

    return TRUE;
}

static gboolean rotctrl_settings_focus_out_cb(GtkWidget *widget,
                                              GdkEventFocus *event,
                                              gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);

    (void)widget;
    (void)event;

    if (ctrl == NULL || ctrl->ui_updating)
        return FALSE;

    rotor_apply_ui_settings(ctrl, FALSE);
    return FALSE;
}

static void rotctrl_settings_activate_cb(GtkEntry *entry, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);

    (void)entry;

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    rotor_apply_ui_settings(ctrl, FALSE);
}

static void rotctrl_update_geometry_sensitivity(GtkRotCtrl *ctrl)
{
    gboolean az_el = FALSE;

    if (ctrl == NULL || ctrl->conf == NULL)
        return;

    az_el = (ctrl->conf->axis_mode == ROT_AXIS_MODE_AZ_EL);

    if (ctrl->min_el_spin)
        gtk_widget_set_sensitive(ctrl->min_el_spin, az_el);
    if (ctrl->max_el_spin)
        gtk_widget_set_sensitive(ctrl->max_el_spin, az_el);
}

static void rotctrl_update_geometry_widgets(GtkRotCtrl *ctrl)
{
    gint axis_index = 1;
    gint wrap_index = 1;
    gdouble minaz = 0.0;
    gdouble maxaz = 0.0;
    gboolean was_updating = FALSE;

    if (ctrl == NULL || ctrl->conf == NULL)
        return;

    was_updating = ctrl->ui_updating;
    rotctrl_ui_begin_update(ctrl, "geometry_widgets");

    axis_index = (ctrl->conf->axis_mode == ROT_AXIS_MODE_AZ_ONLY) ? 0 : 1;
    wrap_index = (ctrl->conf->aztype == ROT_AZ_TYPE_180) ? 0 : 1;

    if (ctrl->axis_mode_combo)
    {
        GtkTreeModel *model =
            gtk_combo_box_get_model(GTK_COMBO_BOX(ctrl->axis_mode_combo));
        if (model != NULL && gtk_tree_model_iter_n_children(model, NULL) > 0)
        {
            rotctrl_combo_set_active_safe(ctrl,
                                          GTK_COMBO_BOX(ctrl->axis_mode_combo),
                                          axis_index,
                                          G_CALLBACK(axis_mode_changed_cb));
        }
    }

    if (ctrl->wrap_mode_combo)
    {
        GtkTreeModel *model =
            gtk_combo_box_get_model(GTK_COMBO_BOX(ctrl->wrap_mode_combo));
        if (model != NULL && gtk_tree_model_iter_n_children(model, NULL) > 0)
        {
            rotctrl_combo_set_active_safe(ctrl,
                                          GTK_COMBO_BOX(ctrl->wrap_mode_combo),
                                          wrap_index,
                                          G_CALLBACK(wrap_mode_changed_cb));
        }
    }

    if (ctrl->min_az_spin)
    {
        g_signal_handlers_block_by_func(ctrl->min_az_spin,
                                        (gpointer)G_CALLBACK(rot_limits_changed_cb), ctrl);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ctrl->min_az_spin), ctrl->conf->minaz);
        g_signal_handlers_unblock_by_func(ctrl->min_az_spin,
                                          (gpointer)G_CALLBACK(rot_limits_changed_cb), ctrl);
    }

    if (ctrl->max_az_spin)
    {
        g_signal_handlers_block_by_func(ctrl->max_az_spin,
                                        (gpointer)G_CALLBACK(rot_limits_changed_cb), ctrl);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ctrl->max_az_spin), ctrl->conf->maxaz);
        g_signal_handlers_unblock_by_func(ctrl->max_az_spin,
                                          (gpointer)G_CALLBACK(rot_limits_changed_cb), ctrl);
    }

    if (ctrl->min_el_spin)
    {
        g_signal_handlers_block_by_func(ctrl->min_el_spin,
                                        (gpointer)G_CALLBACK(rot_limits_changed_cb), ctrl);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ctrl->min_el_spin), ctrl->conf->minel);
        g_signal_handlers_unblock_by_func(ctrl->min_el_spin,
                                          (gpointer)G_CALLBACK(rot_limits_changed_cb), ctrl);
    }

    if (ctrl->max_el_spin)
    {
        g_signal_handlers_block_by_func(ctrl->max_el_spin,
                                        (gpointer)G_CALLBACK(rot_limits_changed_cb), ctrl);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ctrl->max_el_spin), ctrl->conf->maxel);
        g_signal_handlers_unblock_by_func(ctrl->max_el_spin,
                                          (gpointer)G_CALLBACK(rot_limits_changed_cb), ctrl);
    }

    if (ctrl->az_endstop_spin)
    {
        minaz = ctrl->conf->minaz;
        maxaz = ctrl->conf->maxaz;
        if (minaz > maxaz)
        {
            gdouble tmp = minaz;
            minaz = maxaz;
            maxaz = tmp;
        }

        g_signal_handlers_block_by_func(ctrl->az_endstop_spin,
                                        (gpointer)G_CALLBACK(az_endstop_changed_cb), ctrl);
        gtk_spin_button_set_range(GTK_SPIN_BUTTON(ctrl->az_endstop_spin), minaz, maxaz);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ctrl->az_endstop_spin), ctrl->conf->azstoppos);
        g_signal_handlers_unblock_by_func(ctrl->az_endstop_spin,
                                          (gpointer)G_CALLBACK(az_endstop_changed_cb), ctrl);
    }

    if (ctrl->AzSet)
        gtk_rot_knob_set_range(GTK_ROT_KNOB(ctrl->AzSet), ctrl->conf->minaz, ctrl->conf->maxaz);
    if (ctrl->ElSet)
        gtk_rot_knob_set_range(GTK_ROT_KNOB(ctrl->ElSet), ctrl->conf->minel, ctrl->conf->maxel);

    rotctrl_update_geometry_sensitivity(ctrl);
    if (!was_updating)
        rotctrl_ui_end_update(ctrl, "geometry_widgets");
}

static void axis_mode_changed_cb(GtkComboBox *box, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);
    gint idx = gtk_combo_box_get_active(box);

    if (ctrl == NULL || ctrl->conf == NULL || ctrl->ui_updating)
        return;

    ctrl->conf->axis_mode =
        (idx == 0) ? ROT_AXIS_MODE_AZ_ONLY : ROT_AXIS_MODE_AZ_EL;

    rotctrl_update_geometry_sensitivity(ctrl);
    rot_transform_update(ctrl);
    rot_plan_reset(&ctrl->trajectory_plan);
}

static void wrap_mode_changed_cb(GtkComboBox *box, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);
    gint idx = gtk_combo_box_get_active(box);

    if (ctrl == NULL || ctrl->conf == NULL || ctrl->ui_updating)
        return;

    ctrl->conf->aztype = (idx == 0) ? ROT_AZ_TYPE_180 : ROT_AZ_TYPE_360;

    rot_transform_update(ctrl);
    rot_plan_reset(&ctrl->trajectory_plan);
    set_flipped_pass(ctrl);
    rotctrl_tracking_policy_reset_reason(ctrl, "wrap_change");
    ctrl->seam_valid = FALSE;
    ctrl->seam_crossing_active = FALSE;
    ctrl->seam_crossing_sent = FALSE;
    ctrl->seam_crossing_lane_valid = FALSE;
    ctrl->seam_crossing_lane_k = 0;
    ctrl->seam_crossing_target_az360 = 0.0;
    ctrl->seam_crossing_since_us = 0;
    ctrl->wrap_acquire_active = FALSE;
    ctrl->wrap_acquire_sent = FALSE;
    ctrl->wrap_acquire_target_backend = 0.0;
    ctrl->wrap_acquire_target_az360 = 0.0;
    ctrl->wrap_acquire_target_k = 0;
    ctrl->wrap_acquire_since_us = 0;
    ctrl->wrap_acquire_last_log_us = 0;
    ctrl->pending_lane_valid = FALSE;
    ctrl->pending_lane_since_us = 0;
    ctrl->last_target_valid = FALSE;
    ctrl->last_cmd_backend_az = 0.0;
    ctrl->last_cmd_backend_el = 0.0;
    ctrl->last_cmd_backend_valid = FALSE;
    ctrl->setpoint_user_az = 0.0;
    ctrl->setpoint_user_el = 0.0;
    ctrl->setpoint_backend_az = 0.0;
    ctrl->setpoint_backend_el = 0.0;
    ctrl->setpoint_valid = FALSE;
    ctrl->force_next_send = FALSE;
    ctrl->hold_position_on_engage = FALSE;
    ctrl->hold_position_log_emitted = FALSE;
    ctrl->committed_user_az = 0.0;
    ctrl->committed_user_el = 0.0;
    ctrl->committed_raw_az360 = 0.0;
    ctrl->committed_raw_el = 0.0;
    ctrl->committed_backend_az = 0.0;
    ctrl->committed_backend_el = 0.0;
    ctrl->committed_valid = FALSE;
    ctrl->committed_since_us = 0;
    ctrl->last_keepalive_time_us = 0;
    ctrl->last_desired_update_us = 0;
    ctrl->last_cmd_backend_az = 0.0;
    ctrl->last_cmd_backend_el = 0.0;
    ctrl->last_cmd_backend_valid = FALSE;
    ctrl->setpoint_user_az = 0.0;
    ctrl->setpoint_user_el = 0.0;
    ctrl->setpoint_backend_az = 0.0;
    ctrl->setpoint_backend_el = 0.0;
    ctrl->setpoint_valid = FALSE;
    ctrl->force_next_send = FALSE;
    ctrl->committed_user_az = 0.0;
    ctrl->committed_user_el = 0.0;
    ctrl->committed_raw_az360 = 0.0;
    ctrl->committed_raw_el = 0.0;
    ctrl->committed_backend_az = 0.0;
    ctrl->committed_backend_el = 0.0;
    ctrl->committed_valid = FALSE;
    ctrl->committed_since_us = 0;
    ctrl->last_keepalive_time_us = 0;
    ctrl->last_desired_update_us = 0;
}

static void rot_limits_changed_cb(GtkSpinButton *spin, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);
    gdouble minaz;
    gdouble maxaz;

    if (ctrl == NULL || ctrl->conf == NULL || ctrl->ui_updating)
        return;

    if (ctrl->min_az_spin)
        ctrl->conf->minaz = gtk_spin_button_get_value(GTK_SPIN_BUTTON(ctrl->min_az_spin));
    if (ctrl->max_az_spin)
        ctrl->conf->maxaz = gtk_spin_button_get_value(GTK_SPIN_BUTTON(ctrl->max_az_spin));
    if (ctrl->min_el_spin)
        ctrl->conf->minel = gtk_spin_button_get_value(GTK_SPIN_BUTTON(ctrl->min_el_spin));
    if (ctrl->max_el_spin)
        ctrl->conf->maxel = gtk_spin_button_get_value(GTK_SPIN_BUTTON(ctrl->max_el_spin));

    rotctrl_tracking_policy_reset_reason(ctrl, "limits_change");
    ctrl->seam_valid = FALSE;
    ctrl->seam_crossing_active = FALSE;
    ctrl->seam_crossing_sent = FALSE;
    ctrl->seam_crossing_lane_valid = FALSE;
    ctrl->seam_crossing_lane_k = 0;
    ctrl->seam_crossing_target_az360 = 0.0;
    ctrl->seam_crossing_since_us = 0;
    ctrl->wrap_acquire_active = FALSE;
    ctrl->wrap_acquire_sent = FALSE;
    ctrl->wrap_acquire_target_backend = 0.0;
    ctrl->wrap_acquire_target_az360 = 0.0;
    ctrl->wrap_acquire_target_k = 0;
    ctrl->wrap_acquire_since_us = 0;
    ctrl->wrap_acquire_last_log_us = 0;
    ctrl->pending_lane_valid = FALSE;
    ctrl->pending_lane_since_us = 0;
    ctrl->last_target_valid = FALSE;

    minaz = ctrl->conf->minaz;
    maxaz = ctrl->conf->maxaz;
    if (minaz > maxaz)
    {
        gdouble tmp = minaz;
        minaz = maxaz;
        maxaz = tmp;
    }

    if (ctrl->az_endstop_spin)
    {
        g_signal_handlers_block_by_func(ctrl->az_endstop_spin,
                                        (gpointer)G_CALLBACK(az_endstop_changed_cb), ctrl);
        gtk_spin_button_set_range(GTK_SPIN_BUTTON(ctrl->az_endstop_spin), minaz, maxaz);
        if (ctrl->conf->azstoppos < minaz)
            ctrl->conf->azstoppos = minaz;
        if (ctrl->conf->azstoppos > maxaz)
            ctrl->conf->azstoppos = maxaz;
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ctrl->az_endstop_spin), ctrl->conf->azstoppos);
        g_signal_handlers_unblock_by_func(ctrl->az_endstop_spin,
                                          (gpointer)G_CALLBACK(az_endstop_changed_cb), ctrl);
    }

    if (ctrl->AzSet)
        gtk_rot_knob_set_range(GTK_ROT_KNOB(ctrl->AzSet), ctrl->conf->minaz, ctrl->conf->maxaz);
    if (ctrl->ElSet)
        gtk_rot_knob_set_range(GTK_ROT_KNOB(ctrl->ElSet), ctrl->conf->minel, ctrl->conf->maxel);

    rot_transform_update(ctrl);
    rot_plan_reset(&ctrl->trajectory_plan);
    (void)spin;
}

static void az_endstop_changed_cb(GtkSpinButton *spin, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);

    if (ctrl == NULL || ctrl->conf == NULL || ctrl->ui_updating)
        return;

    ctrl->conf->azstoppos = gtk_spin_button_get_value(spin);
    rot_plan_reset(&ctrl->trajectory_plan);
}

/**
 * New rotor device selected.
 *
 * \param box Pointer to the rotor selector combo box.
 * \param data Pointer to the GtkRotCtrl widget.
 * 
 * This function is called when the user selects a new rotor controller
 * device.
 */
static void rot_selected_cb(GtkComboBox * box, gpointer data)
{
    GtkRotCtrl     *ctrl = GTK_ROT_CTRL(data);
    gboolean        was_updating = FALSE;

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    /* free previous configuration */
    if (ctrl->conf != NULL)
    {
        g_free(ctrl->conf->name);
        g_free(ctrl->conf->host);
        g_free(ctrl->conf->device);
        g_free(ctrl->conf->device_manual);
        g_free(ctrl->conf->last_good_device);
        g_free(ctrl->conf);
    }

    ctrl->conf = g_try_new(rotor_conf_t, 1);
    if (ctrl->conf == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%d: Failed to allocate memory for rotator config"),
                    __FILE__, __LINE__);
        return;
    }

    /* load new configuration */
    ctrl->conf->name =
        gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(box));
    if (rotor_conf_read(ctrl->conf))
    {
        gchar *conf_err = NULL;
        if (!rot_conf_validate(ctrl->conf, &conf_err)) {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s:%d: Invalid rotator configuration %s: %s"),
                        __FILE__, __LINE__,
                        ctrl->conf->name ? ctrl->conf->name : "(unnamed)",
                        conf_err ? conf_err : "unknown error");
            rot_show_conf_error(ctrl, conf_err);
            g_free(conf_err);
            g_free(ctrl->conf->name);
            if (ctrl->conf->host)
                g_free(ctrl->conf->host);
            if (ctrl->conf->device)
                g_free(ctrl->conf->device);
            if (ctrl->conf->device_manual)
                g_free(ctrl->conf->device_manual);
            if (ctrl->conf->last_good_device)
                g_free(ctrl->conf->last_good_device);
            g_free(ctrl->conf);
            ctrl->conf = NULL;
            return;
        }

        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("Loaded new rotator configuration %s"),
                    ctrl->conf->name);

        if (ctrl->conf->aztype == ROT_AZ_TYPE_480)
            ctrl->conf->aztype = ROT_AZ_TYPE_360;

        if (ctrl->conf->cycle < 100)
            ctrl->conf->cycle = 100;
        ctrl->conf->rotor_poll_period_ms = ctrl->conf->cycle;

        was_updating = ctrl->ui_updating;
        rotctrl_ui_begin_update(ctrl, "rot_selected");

        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ctrl->cycle_spin),
                                  ctrl->conf->cycle);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ctrl->thld_spin),
                                  ctrl->conf->threshold);

        rotctrl_update_geometry_widgets(ctrl);

        ctrl->use_offset = ctrl->conf->use_offset;
        ctrl->az_offset_deg = ctrl->conf->az_offset;
        ctrl->el_offset_deg = ctrl->conf->el_offset;
        ctrl->pretrack_enabled =
            ctrl->conf->slew_to_aos_while_below_horizon;
        ctrl->pretrack_lookahead_sec =
            (ctrl->conf->pretrack_seconds > 0.0)
                ? ctrl->conf->pretrack_seconds
                : ROT_PRETRACK_LOOKAHEAD_SEC;
        ctrl->pretrack_immediate = ctrl->conf->pretrack_immediate;
        ctrl->pretrack_min_el = ctrl->conf->pretrack_min_el;

        if (ctrl->offset_check) {
            g_signal_handlers_block_by_func(ctrl->offset_check,
                                            (gpointer)G_CALLBACK(offset_toggle_cb), ctrl);
            gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->offset_check),
                                         ctrl->use_offset);
            g_signal_handlers_unblock_by_func(ctrl->offset_check,
                                              (gpointer)G_CALLBACK(offset_toggle_cb), ctrl);
        }
        if (ctrl->az_offset_spin) {
            g_signal_handlers_block_by_func(ctrl->az_offset_spin,
                                            (gpointer)G_CALLBACK(az_offset_changed_cb), ctrl);
            gtk_spin_button_set_value(GTK_SPIN_BUTTON(ctrl->az_offset_spin),
                                      ctrl->az_offset_deg);
            g_signal_handlers_unblock_by_func(ctrl->az_offset_spin,
                                              (gpointer)G_CALLBACK(az_offset_changed_cb), ctrl);
        }
        if (ctrl->el_offset_spin) {
            g_signal_handlers_block_by_func(ctrl->el_offset_spin,
                                            (gpointer)G_CALLBACK(el_offset_changed_cb), ctrl);
            gtk_spin_button_set_value(GTK_SPIN_BUTTON(ctrl->el_offset_spin),
                                      ctrl->el_offset_deg);
            g_signal_handlers_unblock_by_func(ctrl->el_offset_spin,
                                              (gpointer)G_CALLBACK(el_offset_changed_cb), ctrl);
        }

        rotctrl_calib_reload(ctrl);
        rot_transform_update(ctrl);
        rotctrl_presets_load(ctrl);
        rotctrl_presets_refresh(ctrl);
        ctrl->target_state = ROT_TARGET_STATE_IDLE;
        ctrl->target_state_since_us = 0;
        ctrl->target_valid_since_us = 0;
        ctrl->target_invalid_since_us = 0;
        ctrl->span_mode = rotctrl_span_from_conf(ctrl->conf);
        ctrl->last_meas_span_az = NAN;
        rot_plan_reset(&ctrl->trajectory_plan);
        /* Update flipped when changing rotor if there is a plot */
        set_flipped_pass(ctrl);

        if (!was_updating)
            rotctrl_ui_end_update(ctrl, "rot_selected");
    }
    else
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%d: Failed to load rotator configuration %s"),
                    __FILE__, __LINE__, ctrl->conf->name);

        g_free(ctrl->conf->name);
        if (ctrl->conf->host)
            g_free(ctrl->conf->host);
        if (ctrl->conf->device)
            g_free(ctrl->conf->device);
        if (ctrl->conf->device_manual)
            g_free(ctrl->conf->device_manual);
        if (ctrl->conf->last_good_device)
            g_free(ctrl->conf->last_good_device);
        g_free(ctrl->conf);
        ctrl->conf = NULL;
        rotctrl_presets_clear(ctrl);
        rotctrl_presets_refresh(ctrl);
    }
}

/**
 * Monitor mode
 *
 * Inhibits command transmission
 */
static void rot_monitor_cb(GtkCheckButton * button, gpointer data)
{
    GtkRotCtrl     *ctrl = GTK_ROT_CTRL(data);

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    ctrl->monitor = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(button));
    gtk_widget_set_sensitive(ctrl->AzSet, !ctrl->monitor);
    gtk_widget_set_sensitive(ctrl->ElSet, !ctrl->monitor);
    gtk_widget_set_sensitive(ctrl->track, !ctrl->monitor);
    if (ctrl->freeze)
        gtk_widget_set_sensitive(ctrl->freeze, !ctrl->monitor);
}


static gboolean rotctld_list_contains(GSList *list, const gchar *value)
{
    for (GSList *iter = list; iter != NULL; iter = iter->next)
    {
        if (g_strcmp0(iter->data, value) == 0)
            return TRUE;
    }
    return FALSE;
}

static gchar *rotctld_autodetect_device(GtkRotCtrl *ctrl)
{
    GSList *list = NULL;
    gchar *picked = NULL;
    const gchar *current = NULL;
    guint count = 0;
    guint skipped = 0;
    gboolean prefer_cu = FALSE;
    gchar *filtered_list = NULL;
    gchar *preferred_current = NULL;
    const gchar *lookup_current = NULL;

    if (ctrl && ctrl->conf)
    {
        if (ctrl->conf->last_good_device && *ctrl->conf->last_good_device)
            current = ctrl->conf->last_good_device;
        else
            current = ctrl->conf->device;
    }

    prefer_cu = rotctld_autodetect_prefer_cu(ctrl ? ctrl->conf : NULL, current);
    list = rotctld_autodetect_filter_candidates(gp_serial_list_candidates(),
                                                &skipped);
    list = rotctld_autodetect_reorder_ports(list, prefer_cu);
    count = g_slist_length(list);
    rot_term_log(ctrl, "gpredict:rx",
                 "autodetect candidates=%u (filtered=%u)", count, skipped);
    filtered_list = rotctld_autodetect_join_list(list);
    if (filtered_list != NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "autodetect list=%s prefer=%s",
                    filtered_list,
                    prefer_cu ? "cu" : "tty");
        rot_term_log_verbose(ctrl, "gpredict:rx",
                             "autodetect list=%s prefer=%s",
                             filtered_list,
                             prefer_cu ? "cu" : "tty");
        g_free(filtered_list);
    }

    if (current && rotctld_list_contains(list, current))
    {
        lookup_current = current;
#ifdef __APPLE__
        if (rotctld_autodetect_is_tty(current))
        {
            preferred_current = rotctld_autodetect_cu_equivalent(current);
            if (preferred_current && rotctld_list_contains(list, preferred_current))
                lookup_current = preferred_current;
        }
#endif
        picked = g_strdup(lookup_current);
    }
    else if (list != NULL)
        picked = g_strdup(list->data);

    if (picked)
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: auto-selected serial device %s"),
                    __func__, picked);
    else
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: no USB serial devices found"),
                    __func__);

    g_free(preferred_current);
    gp_serial_free_candidates(list);
    return picked;
}

static gint rotctld_mgr_pid(const RotctldMgr *mgr)
{
    const gchar *pid = NULL;
    gchar *endp = NULL;
    long pid_val = -1;

    if (mgr == NULL)
        return -1;

    pid = rotctld_mgr_get_identifier(mgr);
    if (pid == NULL || *pid == '\0')
        return -1;

    pid_val = strtol(pid, &endp, 10);
    if (endp == pid || pid_val <= 1)
        return -1;

    if (pid_val > G_MAXINT)
        return -1;

    return (gint) pid_val;
}

static const gchar *rotctld_child_owner_name(GtkRotCtrl *ctrl,
                                             const RotctldProbeState *state,
                                             gint pid)
{
    if (pid <= 0)
        return "unknown";

    if (ctrl && ctrl->selected_child_pid > 0 &&
        ctrl->selected_child_pid == pid)
        return "selected";

    if (state && state->scanning_child_pid > 0 &&
        state->scanning_child_pid == pid)
        return "scanning";

    return "unknown";
}

static void rotctld_note_stop(GtkRotCtrl *ctrl, gint pid, const gchar *reason)
{
    if (ctrl == NULL)
        return;

    ctrl->last_stop_pid = pid;
    ctrl->last_stop_us = g_get_monotonic_time();
    g_free(ctrl->last_stop_reason);
    ctrl->last_stop_reason = reason ? g_strdup(reason) : NULL;

    if (pid > 0 && ctrl->selected_child_pid == pid)
    {
        ctrl->selected_child_pid = -1;
        ctrl->selected_child_port = 0;
    }
}

static void rotctld_log_exit(GtkRotCtrl *ctrl,
                             RotctldProbeState *state,
                             const gchar *context)
{
    gint exit_status = -1;
    gint exit_signal = 0;
    gboolean have_exit = FALSE;
    gint pid = -1;
    gint64 now_us = 0;
    gboolean expected = FALSE;
    const gchar *owner = "unknown";
    gchar *stderr_tail = NULL;

    if (ctrl == NULL || ctrl->rotctld_mgr == NULL)
        return;

    pid = rotctld_mgr_pid(ctrl->rotctld_mgr);
    now_us = g_get_monotonic_time();
    expected = (pid > 0 &&
                ctrl->last_stop_pid == pid &&
                ctrl->last_stop_us > 0 &&
                (now_us - ctrl->last_stop_us) < (5 * G_TIME_SPAN_SECOND));
    owner = rotctld_child_owner_name(ctrl, state, pid);
    have_exit = rotctld_mgr_get_exit_info(ctrl->rotctld_mgr,
                                          &exit_status,
                                          &exit_signal);
    stderr_tail = rotctld_mgr_get_log_tail(ctrl->rotctld_mgr);

    sat_log_log(SAT_LOG_LEVEL_WARN,
                "rotctld exited pid=%d status=%d signal=%d expected=%d owner=%s ctx=%s stop_reason=%s%s%s",
                pid,
                have_exit ? exit_status : -1,
                have_exit ? exit_signal : 0,
                expected ? 1 : 0,
                owner ? owner : "unknown",
                context ? context : "unknown",
                ctrl->last_stop_reason ? ctrl->last_stop_reason : "none",
                stderr_tail ? " stderr: " : "",
                stderr_tail ? stderr_tail : "");
    rot_term_log(ctrl, "gpredict:err",
                 "rotctld exited pid=%d status=%d signal=%d expected=%d owner=%s",
                 pid,
                 have_exit ? exit_status : -1,
                 have_exit ? exit_signal : 0,
                 expected ? 1 : 0,
                 owner ? owner : "unknown");

    if (pid > 0 && ctrl->selected_child_pid == pid)
    {
        ctrl->selected_child_pid = -1;
        ctrl->selected_child_port = 0;
    }

    g_free(stderr_tail);
}

static gint rotctld_autodetect_get_ms(const gchar *env_name,
                                      gint fallback_ms,
                                      gint min_ms,
                                      gint max_ms)
{
    const gchar *env = g_getenv(env_name);
    glong value = 0;

    if (env == NULL || *env == '\0')
        return fallback_ms;

    value = g_ascii_strtoll(env, NULL, 10);
    if (value <= 0)
        return fallback_ms;
    if (value < min_ms)
        return min_ms;
    if (value > max_ms)
        return max_ms;

    return (gint) value;
}

static void rotctld_process_stop(GtkRotCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    ctrl->az_hold_active = FALSE;
    ctrl->az_hold_value = 0.0;

    if (ctrl->rotctld_mgr)
    {
        gint pid = rotctld_mgr_pid(ctrl->rotctld_mgr);
        rotctld_note_stop(ctrl, pid, "stop");
        rotctld_mgr_terminate(&ctrl->rotctld_mgr);
    }
}

typedef struct {
    RotctldMgr *mgr;
    gchar      *pid;
} RotctldStopJob;

static gpointer rotctld_process_stop_thread(gpointer data)
{
    RotctldStopJob *job = data;

    if (job == NULL)
        return NULL;

#ifdef G_OS_UNIX
    if (job->pid && *job->pid)
    {
        gchar *endp = NULL;
        long pid_val = strtol(job->pid, &endp, 10);
        if (endp != job->pid && pid_val > 1)
        {
            pid_t pid = (pid_t)pid_val;
            (void)kill(pid, SIGTERM);
            g_usleep(300 * 1000);
            if (kill(pid, 0) == 0)
                (void)kill(pid, SIGKILL);
        }
    }
#endif

    if (job->mgr)
        rotctld_mgr_terminate(&job->mgr);

    g_free(job->pid);
    g_free(job);
    return NULL;
}

static void rotctld_process_stop_async(GtkRotCtrl *ctrl,
                                       const gchar *reason)
{
    RotctldStopJob *job = NULL;
    const gchar *pid = NULL;
    gint pid_val = -1;

    if (ctrl == NULL || ctrl->rotctld_mgr == NULL)
        return;

    ctrl->az_hold_active = FALSE;
    ctrl->az_hold_value = 0.0;

    pid = rotctld_mgr_get_identifier(ctrl->rotctld_mgr);
    pid_val = rotctld_mgr_pid(ctrl->rotctld_mgr);
    rotctld_note_stop(ctrl, pid_val, reason);
    job = g_new0(RotctldStopJob, 1);
    job->mgr = ctrl->rotctld_mgr;
    job->pid = g_strdup(pid);
    ctrl->rotctld_mgr = NULL;

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "rotctld stop async pid=%s reason=%s",
                pid ? pid : "unknown",
                reason ? reason : "unknown");
    rot_term_log_verbose(ctrl, "gpredict:rx",
                         "rotctld stop async pid=%s reason=%s",
                         pid ? pid : "unknown",
                         reason ? reason : "unknown");

    g_thread_new("rotctld-stop", rotctld_process_stop_thread, job);
}

static gchar *rotctld_resolve_device(GtkRotCtrl *ctrl,
                                     gboolean *from_autopick)
{
    const gchar *manual = NULL;
    const gchar *selected = NULL;

    if (from_autopick)
        *from_autopick = FALSE;

    if (ctrl == NULL || ctrl->conf == NULL)
        return NULL;

    manual = ctrl->conf->device_manual;
    selected = ctrl->conf->device;

    if (ctrl->conf->device_autopick &&
        (manual == NULL || *manual == '\0') &&
        (selected == NULL || *selected == '\0'))
    {
        if (from_autopick)
            *from_autopick = TRUE;
        return rotctld_autodetect_device(ctrl);
    }

    if (manual && *manual)
        return g_strdup(manual);

    if (selected && *selected)
        return g_strdup(selected);

    return NULL;
}

static gboolean rotctld_argv_has_verbosity(gchar **argv)
{
    if (argv == NULL)
        return FALSE;

    for (gint i = 0; argv[i] != NULL; i++)
    {
        if (g_str_has_prefix(argv[i], "-v"))
            return TRUE;
    }

    return FALSE;
}

static gchar **rotctld_append_verbosity(GtkRotCtrl *ctrl, gchar **argv)
{
    gint len;
    gchar **out;

    if (argv == NULL || rotctld_argv_has_verbosity(argv))
        return argv;

    len = g_strv_length(argv);
    out = g_new0(gchar *, len + 2);

    for (gint i = 0; i < len; i++)
        out[i] = g_strdup(argv[i]);

    out[len] = g_strdup(ctrl && ctrl->verbose_logging ? "-vvvv" : "-v");
    out[len + 1] = NULL;
    g_strfreev(argv);

    return out;
}

static const gchar *rotctld_bind_host(const gchar *host)
{
    if (host == NULL || *host == '\0')
        return "127.0.0.1";

    if (rotctld_mgr_host_is_local(host))
        return "127.0.0.1";

    return host;
}

static gchar **rotctld_force_bind_host(gchar **argv, const gchar *host)
{
    const gchar *bind_host = NULL;
    gint len;

    if (argv == NULL)
        return argv;

    bind_host = rotctld_bind_host(host);
    if (bind_host == NULL || *bind_host == '\0')
        return argv;

    len = g_strv_length(argv);

    for (gint i = 0; i < len; i++)
    {
        if (g_strcmp0(argv[i], "-T") == 0)
        {
            if (i + 1 < len && argv[i + 1] && argv[i + 1][0] != '-')
            {
                g_free(argv[i + 1]);
                argv[i + 1] = g_strdup(bind_host);
                return argv;
            }
            else
            {
                gchar **out = g_new0(gchar *, len + 2);

                for (gint j = 0; j <= i; j++)
                    out[j] = g_strdup(argv[j]);
                out[i + 1] = g_strdup(bind_host);
                for (gint j = i + 1; j < len; j++)
                    out[j + 1] = g_strdup(argv[j]);
                out[len + 1] = NULL;
                g_strfreev(argv);
                return out;
            }
        }
    }

    {
        gchar **out = g_new0(gchar *, len + 3);

        for (gint i = 0; i < len; i++)
            out[i] = g_strdup(argv[i]);
        out[len] = g_strdup("-T");
        out[len + 1] = g_strdup(bind_host);
        out[len + 2] = NULL;
        g_strfreev(argv);
        return out;
    }
}

static gchar **rotctld_force_model(gchar **argv, gint model)
{
    gchar *model_str = NULL;
    gint len;

    if (argv == NULL || model <= 0)
        return argv;

    model_str = g_strdup_printf("%d", model);
    len = g_strv_length(argv);

    for (gint i = 0; i < len; i++)
    {
        if (g_strcmp0(argv[i], "-m") == 0)
        {
            if (i + 1 < len && argv[i + 1] && argv[i + 1][0] != '-')
            {
                g_free(argv[i + 1]);
                argv[i + 1] = g_strdup(model_str);
                g_free(model_str);
                return argv;
            }
            else
            {
                gchar **out = g_new0(gchar *, len + 2);

                for (gint j = 0; j <= i; j++)
                    out[j] = g_strdup(argv[j]);
                out[i + 1] = g_strdup(model_str);
                for (gint j = i + 1; j < len; j++)
                    out[j + 1] = g_strdup(argv[j]);
                out[len + 1] = NULL;
                g_strfreev(argv);
                g_free(model_str);
                return out;
            }
        }
    }

    {
        gchar **out = g_new0(gchar *, len + 3);

        for (gint i = 0; i < len; i++)
            out[i] = g_strdup(argv[i]);
        out[len] = g_strdup("-m");
        out[len + 1] = g_strdup(model_str);
        out[len + 2] = NULL;
        g_strfreev(argv);
        g_free(model_str);
        return out;
    }
}

static gchar *rotctld_merge_config_timeout(const gchar *existing,
                                           gint timeout_ms,
                                           gboolean force_timeout)
{
    gboolean have_timeout = FALSE;
    GString *out = g_string_new(NULL);

    if (timeout_ms <= 0)
        timeout_ms = 1200;

    if (existing && *existing)
    {
        gchar **parts = g_strsplit(existing, ",", -1);
        for (gint i = 0; parts && parts[i] != NULL; i++)
        {
            gchar *trim = g_strstrip(parts[i]);
            if (trim[0] == '\0')
                continue;
            if (g_str_has_prefix(trim, "timeout="))
            {
                if (force_timeout)
                    continue;
                have_timeout = TRUE;
            }
            if (g_str_has_prefix(trim, "retries="))
                continue;
            if (out->len > 0)
                g_string_append_c(out, ',');
            g_string_append(out, trim);
        }
        g_strfreev(parts);
    }

    if (force_timeout || !have_timeout)
    {
        if (out->len > 0)
            g_string_append_c(out, ',');
        g_string_append_printf(out, "timeout=%d", timeout_ms);
    }

    return g_string_free(out, FALSE);
}

static gchar *G_GNUC_UNUSED rotctld_merge_config(const gchar *existing)
{
    return rotctld_merge_config_timeout(existing, 1200, FALSE);
}

static gchar **rotctld_force_config_timeout(gchar **argv,
                                            gint timeout_ms,
                                            gboolean force_timeout)
{
    gint len;

    if (argv == NULL)
        return argv;

    len = g_strv_length(argv);
    for (gint i = 0; i < len; i++)
    {
        if (g_strcmp0(argv[i], "-C") == 0)
        {
            gchar *merged = NULL;
            if (i + 1 < len && argv[i + 1] && argv[i + 1][0] != '-')
            {
                merged = rotctld_merge_config_timeout(argv[i + 1],
                                                      timeout_ms,
                                                      force_timeout);
                g_free(argv[i + 1]);
                argv[i + 1] = merged;
                return argv;
            }
            merged = rotctld_merge_config_timeout(NULL,
                                                  timeout_ms,
                                                  force_timeout);
            gchar **out = g_new0(gchar *, len + 2);
            for (gint j = 0; j <= i; j++)
                out[j] = g_strdup(argv[j]);
            out[i + 1] = merged;
            for (gint j = i + 1; j < len; j++)
                out[j + 1] = g_strdup(argv[j]);
            out[len + 1] = NULL;
            g_strfreev(argv);
            return out;
        }
    }

    {
        gchar **out = g_new0(gchar *, len + 3);
        for (gint i = 0; i < len; i++)
            out[i] = g_strdup(argv[i]);
        out[len] = g_strdup("-C");
        out[len + 1] = rotctld_merge_config_timeout(NULL,
                                                    timeout_ms,
                                                    force_timeout);
        out[len + 2] = NULL;
        g_strfreev(argv);
        return out;
    }
}

static gchar **rotctld_force_config(gchar **argv)
{
    return rotctld_force_config_timeout(argv, 1200, FALSE);
}

static gboolean rotctld_needs_extended_backend_timeout(const GtkRotCtrl *ctrl,
                                                       const gchar *device)
{
#ifdef G_OS_WIN32
    if (ctrl == NULL || ctrl->conf == NULL)
        return FALSE;
    if (!gp_serial_port_is_windows_com(device))
        return FALSE;

    return (ctrl->conf->protocol == ROT_PROTOCOL_SPID_ROT2PROG ||
            rot_conf_hamlib_model(ctrl->conf) == 901);
#else
    (void)ctrl;
    (void)device;
    return FALSE;
#endif
}

static gint rotctld_backend_timeout_ms(GtkRotCtrl *ctrl,
                                       const gchar *device,
                                       gboolean probing)
{
    gint timeout_ms = probing ? ROTCTLD_AUTODETECT_ROT_TIMEOUT_MS
                              : ROTCTLD_BACKEND_TIMEOUT_MS;

    if (rotctld_needs_extended_backend_timeout(ctrl, device))
        timeout_ms = MAX(timeout_ms, ROTCTLD_WIN32_ROT2PROG_TIMEOUT_MS);

    return timeout_ms;
}

static gint rotctld_validate_timeout_floor_ms(GtkRotCtrl *ctrl,
                                              const gchar *device)
{
    gint margin_ms = ROTCTLD_AUTODETECT_VALIDATE_MARGIN_MS;
    gint timeout_ms = rotctld_backend_timeout_ms(ctrl, device, TRUE);

    if (rotctld_needs_extended_backend_timeout(ctrl, device))
        margin_ms = MAX(margin_ms, ROTCTLD_WIN32_ROT2PROG_MARGIN_MS);

    return timeout_ms + margin_ms;
}

static gchar **rotctld_build_argv_from_command(GtkRotCtrl *ctrl,
                                               const gchar *cmdline)
{
    GError *error = NULL;
    gchar **argv = NULL;
    gint argc = 0;

    if (cmdline == NULL || *cmdline == '\0')
        return NULL;

    if (!g_shell_parse_argv(cmdline, &argc, &argv, &error))
    {
        rot_term_log(ctrl, "gpredict:err",
                     "Failed to parse rotctld command: %s",
                     error ? error->message : "unknown error");
        g_clear_error(&error);
        g_strfreev(argv);
        return NULL;
    }

    argv = rotctld_append_verbosity(ctrl, argv);
    argv = rotctld_force_config(argv);
    if (ctrl && ctrl->conf)
    {
        argv = rotctld_force_model(argv,
                                   rot_conf_hamlib_model(ctrl->conf));
        argv = rotctld_force_bind_host(argv, ctrl->conf->host);
    }
    return argv;
}

static gchar *rotctld_argv_to_string(gchar **argv)
{
    GString *buf = NULL;

    if (argv == NULL || argv[0] == NULL)
        return NULL;

    buf = g_string_new(NULL);
    for (gint i = 0; argv[i] != NULL; i++)
    {
        gchar *quoted = g_shell_quote(argv[i]);
        if (i > 0)
            g_string_append_c(buf, ' ');
        g_string_append(buf, quoted);
        g_free(quoted);
    }

    return g_string_free(buf, FALSE);
}

static gboolean rotctld_spawn_process(GtkRotCtrl *ctrl, gchar **argv)
{
    gchar *cmdline = NULL;
    gchar *error = NULL;
    RotctldMgr *mgr = NULL;

    if (ctrl == NULL || argv == NULL)
        return FALSE;

    cmdline = rotctld_argv_to_string(argv);
    rot_term_log_verbose(ctrl, "gpredict:tx",
                         "spawn rotctld: %s", cmdline ? cmdline : "(null)");
    g_free(cmdline);

    mgr = rotctld_mgr_spawn_argv(argv, &error);
    if (mgr == NULL)
    {
        rot_term_log(ctrl, "gpredict:err",
                     "Failed to start rotctld: %s",
                     error ? error : "unknown error");
        g_free(error);
        return FALSE;
    }

    if (ctrl->rotctld_mgr)
        rotctld_process_stop(ctrl);

    ctrl->rotctld_mgr = mgr;
    rotctld_mgr_set_log_callback(ctrl->rotctld_mgr, rotctld_log_cb, ctrl);
    rot_term_log_verbose(ctrl, "gpredict:rx",
                         "rotctld started pid=%s",
                         rotctld_mgr_get_identifier(ctrl->rotctld_mgr));

    return TRUE;
}

static gboolean G_GNUC_UNUSED rotctld_should_autodetect(const rotor_conf_t *conf)
{
    const gchar *env_device = g_getenv("GPREDICT_ROT_SERIAL");
    const gchar *env_cmd = g_getenv("GPREDICT_ROTCTLD_CMD");

    if (conf == NULL)
        return FALSE;

    if (env_device && *env_device)
        return FALSE;
    if (env_cmd && *env_cmd)
        return FALSE;

    if (!conf->device_autopick)
        return FALSE;

    if (conf->device_manual && *conf->device_manual)
        return FALSE;

    return TRUE;
}

static gboolean rotctld_autodetect_is_candidate(const gchar *candidate)
{
    gchar *base = NULL;
    gchar *lower = NULL;
    gboolean ok = FALSE;

    if (candidate == NULL || *candidate == '\0')
        return FALSE;

    base = g_path_get_basename(candidate);
    lower = g_ascii_strdown(base ? base : candidate, -1);
    if (lower == NULL)
    {
        g_free(base);
        return FALSE;
    }

    if (g_strrstr(lower, "bluetooth") != NULL ||
        g_strrstr(lower, "incoming") != NULL ||
        g_strrstr(lower, "debug-console") != NULL)
    {
        ok = FALSE;
    }
    else if (gp_serial_port_is_windows_com(candidate))
    {
        ok = TRUE;
    }
    else if (rotctld_autodetect_is_tty(candidate) ||
             rotctld_autodetect_is_cu(candidate))
    {
        if (g_strrstr(lower, "usbserial") != NULL ||
            g_strrstr(lower, "slab_usbtouart") != NULL ||
            g_strrstr(lower, "ttyacm") != NULL ||
            g_strrstr(lower, "ttyusb") != NULL ||
            g_strrstr(lower, "ftdi") != NULL ||
            g_strrstr(lower, "ft232") != NULL ||
            g_strrstr(lower, "usb") != NULL)
            ok = TRUE;
    }

    g_free(lower);
    g_free(base);
    return ok;
}

static gboolean rotctld_autodetect_is_tty(const gchar *candidate)
{
    gchar *base = NULL;
    gchar *lower = NULL;
    gboolean ok = FALSE;

    if (candidate == NULL || *candidate == '\0')
        return FALSE;

    base = g_path_get_basename(candidate);
    lower = g_ascii_strdown(base ? base : candidate, -1);
    if (lower == NULL)
    {
        g_free(base);
        return FALSE;
    }

    if (g_str_has_prefix(lower, "tty."))
        ok = TRUE;
    else if (g_str_has_prefix(lower, "tty"))
        ok = TRUE;
    else if (g_str_has_prefix(lower, "/dev/tty."))
        ok = TRUE;
    else if (g_str_has_prefix(lower, "/dev/tty"))
        ok = TRUE;

    g_free(lower);
    g_free(base);
    return ok;
}

static gboolean rotctld_autodetect_is_cu(const gchar *candidate)
{
    gchar *base = NULL;
    gchar *lower = NULL;
    gboolean ok = FALSE;

    if (candidate == NULL || *candidate == '\0')
        return FALSE;

    base = g_path_get_basename(candidate);
    lower = g_ascii_strdown(base ? base : candidate, -1);
    if (lower == NULL)
    {
        g_free(base);
        return FALSE;
    }

    if (g_str_has_prefix(lower, "cu."))
        ok = TRUE;
    else if (g_str_has_prefix(lower, "cu"))
        ok = TRUE;
    else if (g_str_has_prefix(lower, "/dev/cu."))
        ok = TRUE;
    else if (g_str_has_prefix(lower, "/dev/cu"))
        ok = TRUE;

    g_free(lower);
    g_free(base);
    return ok;
}

static gchar *G_GNUC_UNUSED rotctld_autodetect_tty_equivalent(const gchar *candidate)
{
#ifdef __APPLE__
    gchar *base = NULL;
    gchar *dir = NULL;
    gchar *lower = NULL;
    const gchar *suffix = NULL;
    gchar *tty_base = NULL;
    gchar *out = NULL;
    gboolean has_dot = FALSE;

    if (candidate == NULL || *candidate == '\0')
        return NULL;

    base = g_path_get_basename(candidate);
    dir = g_path_get_dirname(candidate);
    lower = g_ascii_strdown(base ? base : candidate, -1);
    if (lower == NULL)
    {
        g_free(base);
        g_free(dir);
        return NULL;
    }

    if (g_str_has_prefix(lower, "cu."))
    {
        suffix = base + 3;
        has_dot = TRUE;
    }
    else if (g_str_has_prefix(lower, "cu"))
    {
        suffix = base + 2;
        has_dot = FALSE;
    }

    if (suffix == NULL)
    {
        g_free(lower);
        g_free(base);
        g_free(dir);
        return NULL;
    }

    if (has_dot)
        tty_base = g_strdup_printf("tty.%s", suffix);
    else
        tty_base = g_strdup_printf("tty%s", suffix);

    if (dir && *dir && g_strcmp0(dir, ".") != 0)
        out = g_build_filename(dir, tty_base, NULL);
    else
        out = g_strdup(tty_base);

    g_free(tty_base);
    g_free(lower);
    g_free(base);
    g_free(dir);
    return out;
#else
    (void)candidate;
    return NULL;
#endif
}

static gchar *rotctld_autodetect_cu_equivalent(const gchar *candidate)
{
#ifdef __APPLE__
    gchar *base = NULL;
    gchar *dir = NULL;
    gchar *lower = NULL;
    const gchar *suffix = NULL;
    gchar *cu_base = NULL;
    gchar *out = NULL;
    gboolean has_dot = FALSE;

    if (candidate == NULL || *candidate == '\0')
        return NULL;

    base = g_path_get_basename(candidate);
    dir = g_path_get_dirname(candidate);
    lower = g_ascii_strdown(base ? base : candidate, -1);
    if (lower == NULL)
    {
        g_free(base);
        g_free(dir);
        return NULL;
    }

    if (g_str_has_prefix(lower, "tty."))
    {
        suffix = base + 4;
        has_dot = TRUE;
    }
    else if (g_str_has_prefix(lower, "tty"))
    {
        suffix = base + 3;
        has_dot = FALSE;
    }

    if (suffix == NULL)
    {
        g_free(lower);
        g_free(base);
        g_free(dir);
        return NULL;
    }

    if (has_dot)
        cu_base = g_strdup_printf("cu.%s", suffix);
    else
        cu_base = g_strdup_printf("cu%s", suffix);

    if (dir && *dir && g_strcmp0(dir, ".") != 0)
        out = g_build_filename(dir, cu_base, NULL);
    else
        out = g_strdup(cu_base);

    g_free(cu_base);
    g_free(lower);
    g_free(base);
    g_free(dir);
    return out;
#else
    (void)candidate;
    return NULL;
#endif
}

static gchar *rotctld_autodetect_candidate_key(const gchar *candidate)
{
    gchar *base = NULL;
    gchar *lower = NULL;
    gchar *key = NULL;

    if (candidate == NULL || *candidate == '\0')
        return NULL;

    base = g_path_get_basename(candidate);
    lower = g_ascii_strdown(base ? base : candidate, -1);
    if (lower == NULL)
    {
        g_free(base);
        return NULL;
    }

#ifdef __APPLE__
    if (g_str_has_prefix(lower, "tty.") || g_str_has_prefix(lower, "cu."))
        key = g_strdup(lower + 4);
    else if (g_str_has_prefix(lower, "tty") || g_str_has_prefix(lower, "cu"))
        key = g_strdup(lower + 3);
    else
        key = g_strdup(lower);
#else
    key = g_strdup(lower);
#endif

    g_free(lower);
    g_free(base);
    return key;
}

static gboolean rotctld_autodetect_prefer_cu(const rotor_conf_t *conf,
                                             const gchar *cached)
{
#ifdef __APPLE__
    (void)conf;
    (void)cached;
    return TRUE;
#else
    (void)conf;
    (void)cached;
#endif
    return FALSE;
}

static GSList *rotctld_autodetect_reorder_ports(GSList *list,
                                                gboolean prefer_cu)
{
    GSList *preferred = NULL;
    GSList *other = NULL;
    GSList *iter = NULL;

    if (list == NULL)
        return NULL;

    for (iter = list; iter != NULL; iter = iter->next)
    {
        const gchar *candidate = iter->data;
        gboolean match = prefer_cu
                         ? rotctld_autodetect_is_cu(candidate)
                         : rotctld_autodetect_is_tty(candidate);
        if (match)
            preferred = g_slist_append(preferred, (gpointer)candidate);
        else
            other = g_slist_append(other, (gpointer)candidate);
    }

    g_slist_free(list);
    return g_slist_concat(preferred, other);
}

static gchar *rotctld_autodetect_join_list(GSList *list)
{
    GString *out = NULL;
    GSList *iter = NULL;

    if (list == NULL)
        return NULL;

    out = g_string_new(NULL);
    for (iter = list; iter != NULL; iter = iter->next)
    {
        const gchar *candidate = iter->data;
        if (candidate == NULL)
            continue;
        if (out->len > 0)
            g_string_append(out, ", ");
        g_string_append(out, candidate);
    }

    return g_string_free(out, FALSE);
}

static gint rotctld_autodetect_candidate_score(const gchar *candidate)
{
    gchar *lower = NULL;
    gint score = 0;

    if (candidate == NULL || *candidate == '\0')
        return 0;

    lower = g_ascii_strdown(candidate, -1);
    if (lower == NULL)
        return 0;

#ifdef __APPLE__
    if (g_str_has_prefix(lower, "/dev/cu.") ||
        g_str_has_prefix(lower, "cu."))
        score += 15;
    if (g_str_has_prefix(lower, "/dev/tty.") ||
        g_str_has_prefix(lower, "tty."))
        score += 5;
#endif
    if (g_str_has_prefix(lower, "/dev/tty") &&
        !g_str_has_prefix(lower, "/dev/tty."))
        score -= 5;

    if (g_strrstr(lower, "usbserial") != NULL)
        score += 30;
    if (g_strrstr(lower, "slab_usbtouart") != NULL)
        score += 20;
    if (g_strrstr(lower, "ttyacm") != NULL)
        score += 15;
    if (g_strrstr(lower, "ttyusb") != NULL)
        score += 15;
    if (g_strrstr(lower, "ftdi") != NULL || g_strrstr(lower, "ft232") != NULL)
        score += 12;
    if (gp_serial_port_is_windows_com(candidate))
        score += 10;

    g_free(lower);
    return score;
}

static gint rotctld_autodetect_compare_candidates(gconstpointer a,
                                                  gconstpointer b)
{
    const gchar *cand_a = a;
    const gchar *cand_b = b;
    gint score_a = rotctld_autodetect_candidate_score(cand_a);
    gint score_b = rotctld_autodetect_candidate_score(cand_b);

    if (score_a != score_b)
        return score_b - score_a;

    return gpredict_strcmp(cand_a, cand_b);
}

static GSList *rotctld_autodetect_filter_candidates(GSList *candidates,
                                                    guint *filtered_out)
{
    GSList *filtered = NULL;
    GSList *item = NULL;
    GHashTable *by_key = NULL;
    guint skipped = 0;

    by_key = g_hash_table_new_full(g_str_hash, g_str_equal, g_free, g_free);

    for (item = candidates; item != NULL; item = item->next)
    {
        const gchar *candidate = item->data;
        gchar *key = NULL;
        gchar *existing = NULL;

        if (!rotctld_autodetect_is_candidate(candidate))
        {
            skipped++;
            continue;
        }

        key = rotctld_autodetect_candidate_key(candidate);
        if (key == NULL)
        {
            skipped++;
            continue;
        }

        existing = g_hash_table_lookup(by_key, key);
        if (existing == NULL)
        {
            g_hash_table_insert(by_key, key, g_strdup(candidate));
            key = NULL;
        }
        else
        {
#ifdef __APPLE__
            gboolean cand_cu = rotctld_autodetect_is_cu(candidate);
            gboolean exist_cu = rotctld_autodetect_is_cu(existing);

            if (cand_cu && !exist_cu)
            {
                g_hash_table_replace(by_key, g_strdup(key),
                                     g_strdup(candidate));
            }
#endif
            g_free(key);
        }
    }

    {
        GHashTableIter iter;
        gpointer key = NULL;
        gpointer value = NULL;

        g_hash_table_iter_init(&iter, by_key);
        while (g_hash_table_iter_next(&iter, &key, &value))
        {
            filtered = g_slist_append(filtered, g_strdup((const gchar *) value));
        }
    }

    gp_serial_free_candidates(candidates);
    g_hash_table_destroy(by_key);
    if (filtered_out)
        *filtered_out = skipped;

    return g_slist_sort(filtered, rotctld_autodetect_compare_candidates);
}

static gint rotctld_pick_ephemeral_port(gint avoid_port)
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

static GSList *rotctld_autodetect_prefer_device(GSList *list,
                                                const gchar *device)
{
    GSList *out = NULL;
    gboolean matched = FALSE;
    gchar *preferred = NULL;
    const gchar *lookup = device;

    if (device == NULL || *device == '\0')
        return list;

#ifdef __APPLE__
    if (rotctld_autodetect_is_tty(device))
    {
        preferred = rotctld_autodetect_cu_equivalent(device);
        if (preferred && rotctld_list_contains(list, preferred))
            lookup = preferred;
    }
#endif

    for (GSList *iter = list; iter != NULL; iter = iter->next)
    {
        if (g_strcmp0(iter->data, lookup) == 0)
        {
            matched = TRUE;
            break;
        }
    }

    if (!matched)
    {
        out = g_slist_prepend(out, g_strdup(lookup));
    }

    for (GSList *iter = list; iter != NULL; iter = iter->next)
    {
        if (matched && g_strcmp0(iter->data, lookup) == 0)
            continue;
        out = g_slist_append(out, g_strdup(iter->data));
    }

    g_free(preferred);
    gp_serial_free_candidates(list);
    return out;
}

static GSList *rotctld_autodetect_limit_list(GSList *list, guint limit)
{
    GSList *out = NULL;
    guint count = 0;

    if (list == NULL || limit == 0)
        return NULL;

    for (GSList *iter = list; iter != NULL && count < limit; iter = iter->next)
    {
        out = g_slist_append(out, g_strdup(iter->data));
        count++;
    }

    return out;
}

static GSList *rotctld_autodetect_remove_key(GSList *list,
                                             const gchar *key)
{
    GSList *out = NULL;

    if (list == NULL || key == NULL || *key == '\0')
        return list;

    for (GSList *iter = list; iter != NULL; iter = iter->next)
    {
        const gchar *candidate = iter->data;
        gchar *cand_key = rotctld_autodetect_candidate_key(candidate);
        gboolean match = (cand_key && g_strcmp0(cand_key, key) == 0);
        g_free(cand_key);

        if (!match)
            out = g_slist_append(out, g_strdup(candidate));
    }

    gp_serial_free_candidates(list);
    return out;
}

static gboolean rotctld_autodetect_has_next_baud(const RotctldProbeState *state,
                                                 const rotor_conf_t *conf)
{
    static const gint baud_list[] = { 9600, 4800, 19200, 38400 };

    if (state == NULL || conf == NULL)
        return FALSE;

    if (state->last_good_exclusive)
        return FALSE;

    if (conf->baud > 0)
        return state->autodetect_baud_index == 0;

    return state->autodetect_baud_index < (gint) G_N_ELEMENTS(baud_list);
}

static gboolean rotctld_autodetect_next_baud(RotctldProbeState *state,
                                             const rotor_conf_t *conf)
{
    static const gint baud_list[] = { 9600, 4800, 19200, 38400 };

    if (state == NULL || conf == NULL)
        return FALSE;

    if (state->last_good_exclusive)
        return FALSE;

    if (conf->baud > 0)
    {
        if (state->autodetect_baud_index > 0)
            return FALSE;
        state->autodetect_baud = conf->baud;
        state->autodetect_baud_index = 1;
        return TRUE;
    }

    if (state->autodetect_baud_index >= G_N_ELEMENTS(baud_list))
        return FALSE;

    state->autodetect_baud = baud_list[state->autodetect_baud_index++];
    return TRUE;
}

static void rotctld_autodetect_reset_baud(RotctldProbeState *state,
                                          const rotor_conf_t *conf)
{
    if (state == NULL)
        return;

    state->autodetect_baud_index = 0;
    state->autodetect_baud = 0;
    if (state->last_good_exclusive)
    {
        if (state->last_good_baud > 0)
            state->autodetect_baud = state->last_good_baud;
        else if (conf && conf->baud > 0)
            state->autodetect_baud = conf->baud;
        else if (conf)
            state->autodetect_baud = rot_protocol_default_baud(conf->protocol);
        state->autodetect_baud_index = 1;
        return;
    }
    if (conf && conf->baud <= 0 &&
        conf->last_good_baud > 0 &&
        conf->last_good_device &&
        state->autodetect_device &&
        g_strcmp0(state->autodetect_device, conf->last_good_device) == 0)
    {
        state->autodetect_baud = conf->last_good_baud;
        state->autodetect_baud_index = 1;
        return;
    }
    (void)rotctld_autodetect_next_baud(state, conf);
}

static void rotctld_autodetect_log_candidate(GtkRotCtrl *ctrl,
                                             RotctldProbeState *state,
                                             const gchar *device,
                                             gint baud)
{
    gint64 now_us = g_get_monotonic_time();
    gint port = state ? state->autodetect_port : 0;
    gint model = 0;

    if (device == NULL || *device == '\0')
        return;

    if (state)
    {
        state->autodetect_candidate_start_us = now_us;
        if (state->autodetect_attempts == NULL)
            state->autodetect_attempts = g_string_new(NULL);
        if (state->autodetect_attempts->len > 0)
            g_string_append(state->autodetect_attempts, ", ");
        if (baud > 0)
            g_string_append_printf(state->autodetect_attempts,
                                   "%s@%d:%d", device, baud, port);
        else
            g_string_append_printf(state->autodetect_attempts,
                                   "%s:%d", device, port);
    }

    if (ctrl && ctrl->conf)
        model = rot_conf_hamlib_model(ctrl->conf);

    rot_term_log(ctrl, "gpredict:rx",
                 "rotor probe: trying device=%s tcp_port=%d",
                 device, port);
    sat_log_log(SAT_LOG_LEVEL_INFO,
                "rotor probe: trying device=%s tcp_port=%d",
                device, port);

    if (baud > 0)
        rot_term_log(ctrl, "gpredict:rx",
                     "autodetect attempt=%llu candidate %s baud=%d port=%d model=%d",
                     (unsigned long long)(state ? state->autodetect_generation : 0),
                     device, baud, port, model);
    else
        rot_term_log(ctrl, "gpredict:rx",
                     "autodetect attempt=%llu candidate %s port=%d model=%d",
                     (unsigned long long)(state ? state->autodetect_generation : 0),
                     device, port, model);

    sat_log_log(SAT_LOG_LEVEL_INFO,
                "autodetect[%llu:%llu] candidate device=%s baud=%d port=%d model=%d ts=%.3fs",
                (unsigned long long) (state ? state->generation : 0),
                (unsigned long long) (state ? state->autodetect_generation : 0),
                device,
                baud,
                port,
                model,
                now_us / 1000000.0);
}

static gboolean rotctld_autodetect_stop_child(GtkRotCtrl *ctrl,
                                              RotctldProbeState *state,
                                              const gchar *reason)
{
    gint pid = -1;
    gint scanning_pid = -1;
    gint selected_pid = -1;
    const gchar *owner = "unknown";
    const gchar *device = NULL;

    if (ctrl == NULL || ctrl->rotctld_mgr == NULL)
        return FALSE;

    pid = rotctld_mgr_pid(ctrl->rotctld_mgr);
    scanning_pid = state ? state->scanning_child_pid : -1;
    selected_pid = ctrl->selected_child_pid;
    owner = rotctld_child_owner_name(ctrl, state, pid);
    device = state ? state->autodetect_device : NULL;

    if (pid > 0 && pid == selected_pid)
    {
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "autodetect stop skipped pid=%d dev=%s reason=%s scanning_pid=%d selected_pid=%d state=%s",
                    pid,
                    device ? device : "(null)",
                    reason ? reason : "unknown",
                    scanning_pid,
                    selected_pid,
                    rotctld_autodetect_state_name(
                        state ? state->autodetect_state : ROTCTLD_AUTODETECT_IDLE));
        rot_term_log_verbose(ctrl, "gpredict:rx",
                             "autodetect stop skipped pid=%d dev=%s reason=%s scanning_pid=%d selected_pid=%d state=%s",
                             pid,
                             device ? device : "(null)",
                             reason ? reason : "unknown",
                             scanning_pid,
                             selected_pid,
                             rotctld_autodetect_state_name(
                                 state ? state->autodetect_state : ROTCTLD_AUTODETECT_IDLE));
        return FALSE;
    }

    if (scanning_pid <= 0 || (pid > 0 && pid != scanning_pid))
    {
        sat_log_log(SAT_LOG_LEVEL_WARN,
                    "autodetect stop skipped pid=%d dev=%s reason=%s scanning_pid=%d selected_pid=%d owner=%s state=%s",
                    pid,
                    device ? device : "(null)",
                    reason ? reason : "unknown",
                    scanning_pid,
                    selected_pid,
                    owner ? owner : "unknown",
                    rotctld_autodetect_state_name(
                        state ? state->autodetect_state : ROTCTLD_AUTODETECT_IDLE));
        rot_term_log_verbose(ctrl, "gpredict:rx",
                             "autodetect stop skipped pid=%d dev=%s reason=%s scanning_pid=%d selected_pid=%d state=%s",
                             pid,
                             device ? device : "(null)",
                             reason ? reason : "unknown",
                             scanning_pid,
                             selected_pid,
                             rotctld_autodetect_state_name(
                                 state ? state->autodetect_state : ROTCTLD_AUTODETECT_IDLE));
        return FALSE;
    }

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "autodetect stop pid=%d dev=%s reason=%s scanning_pid=%d selected_pid=%d state=%s",
                pid,
                device ? device : "(null)",
                reason ? reason : "unknown",
                scanning_pid,
                selected_pid,
                rotctld_autodetect_state_name(
                    state ? state->autodetect_state : ROTCTLD_AUTODETECT_IDLE));
    rot_term_log_verbose(ctrl, "gpredict:rx",
                         "autodetect stop pid=%d dev=%s reason=%s scanning_pid=%d selected_pid=%d state=%s",
                         pid,
                         device ? device : "(null)",
                         reason ? reason : "unknown",
                         scanning_pid,
                         selected_pid,
                         rotctld_autodetect_state_name(
                             state ? state->autodetect_state : ROTCTLD_AUTODETECT_IDLE));

    rotctld_process_stop_async(ctrl, reason ? reason : "autodetect_stop");

    if (state)
    {
        state->scanning_child_pid = -1;
        state->scanning_child_validated = FALSE;
        state->spawned = FALSE;
    }

    return TRUE;
}

static const gchar *rotctld_autodetect_state_name(rotctld_autodetect_state_t state)
{
    switch (state)
    {
    case ROTCTLD_AUTODETECT_IDLE:
        return "IDLE";
    case ROTCTLD_AUTODETECT_START_CHILD:
        return "START_CHILD";
    case ROTCTLD_AUTODETECT_SETTLE:
        return "SETTLE";
    case ROTCTLD_AUTODETECT_WAIT_TCP_READY:
        return "WAIT_TCP_READY";
    case ROTCTLD_AUTODETECT_VALIDATE_IO:
        return "VALIDATE_IO";
    case ROTCTLD_AUTODETECT_SUCCESS:
        return "SUCCESS";
    case ROTCTLD_AUTODETECT_FAIL_NEXT:
        return "FAIL_NEXT";
    case ROTCTLD_AUTODETECT_ABORT:
        return "ABORT";
    default:
        return "UNKNOWN";
    }
}

static void rotctld_autodetect_transition(GtkRotCtrl *ctrl,
                                          RotctldProbeState *state,
                                          rotctld_autodetect_state_t next,
                                          const gchar *reason)
{
    gint64 now_us = g_get_monotonic_time();
    const gchar *prev = rotctld_autodetect_state_name(
        state ? state->autodetect_state : ROTCTLD_AUTODETECT_IDLE);
    const gchar *next_name = rotctld_autodetect_state_name(next);

    if (state == NULL)
        return;

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "autodetect[%llu:%llu] %s -> %s ts=%.3fs%s%s",
                (unsigned long long) state->generation,
                (unsigned long long) state->autodetect_generation,
                prev,
                next_name,
                now_us / 1000000.0,
                reason ? " reason=" : "",
                reason ? reason : "");
    if (ctrl)
        rot_term_log_verbose(ctrl, "gpredict:rx",
                             "autodetect: %s -> %s%s%s%s",
                             prev, next_name,
                             reason ? " (" : "",
                             reason ? reason : "",
                             reason ? ")" : "");

    state->autodetect_state = next;
    state->autodetect_state_since_us = now_us;
}

static void rotctld_autodetect_reset_validation(RotctldProbeState *state)
{
    if (state == NULL)
        return;

    state->scanning_child_validated = FALSE;

    g_mutex_lock(&state->validate_mutex);
    state->validate_inflight = FALSE;
    state->validate_done = FALSE;
    state->validate_result = ROTCTLD_POS_IO_ERR;
    state->validate_score = 0;
    state->validate_rprt = 0;
    g_free(state->validate_reason);
    state->validate_reason = NULL;
    state->validate_dump_ok = FALSE;
    state->validate_pos_ok = FALSE;
    state->validate_timeout = FALSE;
    g_mutex_unlock(&state->validate_mutex);
}

static void rotctld_autodetect_apply_success(GtkRotCtrl *ctrl,
                                             RotctldProbeState *state)
{
    gint pid = -1;
    gint64 now_us = g_get_monotonic_time();
    gdouble total_ms = 0.0;

    if (ctrl == NULL || state == NULL || state->autodetect_device == NULL)
        return;

    if (state->autodetect_baud > 0)
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "%s: autodetect selected device %s (validated) baud=%d port=%d",
                    __func__,
                    state->autodetect_device,
                    state->autodetect_baud,
                    state->autodetect_port);
    else
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "%s: autodetect selected device %s (validated) port=%d",
                    __func__, state->autodetect_device,
                    state->autodetect_port);
    if (state->autodetect_baud > 0)
        rot_term_log(ctrl, "gpredict:rx",
                     "autodetect: selected %s (validated) baud=%d port=%d",
                     state->autodetect_device,
                     state->autodetect_baud,
                     state->autodetect_port);
    else
        rot_term_log(ctrl, "gpredict:rx",
                     "autodetect: selected %s (validated) port=%d",
                     state->autodetect_device,
                     state->autodetect_port);

    if (state->autodetect_start_us > 0)
        total_ms = (now_us - state->autodetect_start_us) / 1000.0;
    rot_term_log_verbose(ctrl, "gpredict:rx",
                         "autodetect: total=%.0fms",
                         total_ms);

    if (state->scanning_child_pid > 0)
        pid = state->scanning_child_pid;
    else if (ctrl->rotctld_mgr)
        pid = rotctld_mgr_pid(ctrl->rotctld_mgr);

    ctrl->selected_child_pid = pid;
    ctrl->selected_child_port = state->autodetect_port;
    state->scanning_child_pid = -1;
    state->scanning_child_validated = FALSE;
    state->spawned = FALSE;
    state->autodetect_port_free_deadline_us = 0;
    state->autodetect_settle_until_us = 0;

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "autodetect select pid=%d dev=%s reason=selected state=%s",
                pid,
                state->autodetect_device ? state->autodetect_device : "(null)",
                rotctld_autodetect_state_name(state->autodetect_state));
    rot_term_log_verbose(ctrl, "gpredict:rx",
                         "autodetect select pid=%d dev=%s reason=selected",
                         pid,
                         state->autodetect_device ? state->autodetect_device : "(null)");

    if (ctrl->conf && ctrl->conf->device_autopick)
    {
        g_free(ctrl->conf->device);
        ctrl->conf->device = g_strdup(state->autodetect_device);
    }
    if (ctrl->conf)
    {
        g_free(ctrl->conf->last_good_device);
        ctrl->conf->last_good_device = g_strdup(state->autodetect_device);
        ctrl->conf->last_good_baud = state->autodetect_baud;
    }
    if (ctrl->conf && ctrl->conf->baud <= 0 &&
        state->autodetect_baud > 0)
        ctrl->conf->baud = state->autodetect_baud;
}

static void rotctld_autodetect_init(RotctldProbeState *state, GtkRotCtrl *ctrl)
{
    guint skipped = 0;
    const gchar *cached = NULL;
    const gchar *last_good = NULL;
    gboolean prefer_cu = FALSE;
    gchar *filtered_list = NULL;
    gint64 now_us = g_get_monotonic_time();
    GSList *full_list = NULL;
    guint full_count = 0;
    gboolean have_last_good = FALSE;

    if (state == NULL || state->autodetect_list != NULL)
        return;

    state->best_score = G_MININT;
    g_free(state->best_device);
    state->best_device = NULL;
    state->best_baud = 0;
    state->scanning_child_pid = -1;
    state->scanning_child_validated = FALSE;
    state->autodetect_start_us = now_us;
    state->autodetect_candidate_start_us = 0;
    state->last_good_exclusive = FALSE;
    state->last_good_deadline_us = 0;
    g_free(state->last_good_device);
    state->last_good_device = NULL;
    state->last_good_baud = 0;
    state->autodetect_port = 0;
    if (state->autodetect_attempts)
    {
        g_string_free(state->autodetect_attempts, TRUE);
        state->autodetect_attempts = NULL;
    }
    state->autodetect_limited = FALSE;
    if (state->autodetect_list_full)
    {
        gp_serial_free_candidates(state->autodetect_list_full);
        state->autodetect_list_full = NULL;
    }

    if (ctrl && ctrl->conf)
    {
        if (ctrl->conf->last_good_device && *ctrl->conf->last_good_device)
        {
            cached = ctrl->conf->last_good_device;
            last_good = ctrl->conf->last_good_device;
            if (ctrl->conf->last_good_baud > 0)
                state->last_good_baud = ctrl->conf->last_good_baud;
        }
        else
            cached = ctrl->conf->device;
    }

    prefer_cu = rotctld_autodetect_prefer_cu(ctrl ? ctrl->conf : NULL, cached);
    full_list =
        rotctld_autodetect_filter_candidates(gp_serial_list_candidates(),
                                             &skipped);
    full_list = rotctld_autodetect_reorder_ports(full_list, prefer_cu);
    full_list = rotctld_autodetect_prefer_device(full_list, cached);
    full_count = g_slist_length(full_list);

    have_last_good = (last_good && *last_good);
    if (have_last_good)
    {
        state->last_good_exclusive = TRUE;
        state->last_good_deadline_us =
            now_us + ((gint64) ROTCTLD_AUTODETECT_LASTGOOD_WINDOW_MS * 1000);
        state->last_good_device = g_strdup(last_good);
        state->autodetect_list = g_slist_append(NULL, g_strdup(last_good));
        state->autodetect_list_full = full_list;
        state->autodetect_next = state->autodetect_list;
        state->autodetect_count = g_slist_length(state->autodetect_list);
        rot_term_log(ctrl, "gpredict:rx",
                     "autodetect last_good=%s baud=%d window=%dms",
                     last_good,
                     state->last_good_baud,
                     ROTCTLD_AUTODETECT_LASTGOOD_WINDOW_MS);
    }
    else if (full_count > ROTCTLD_AUTODETECT_MAX_CANDIDATES)
    {
        state->autodetect_list_full = full_list;
        state->autodetect_list =
            rotctld_autodetect_limit_list(full_list,
                                          ROTCTLD_AUTODETECT_MAX_CANDIDATES);
        state->autodetect_limited = TRUE;
        state->autodetect_next = state->autodetect_list;
        state->autodetect_count = g_slist_length(state->autodetect_list);
    }
    else
    {
        state->autodetect_list = full_list;
        full_list = NULL;
        state->autodetect_next = state->autodetect_list;
        state->autodetect_count = g_slist_length(state->autodetect_list);
    }

    rot_term_log(ctrl, "gpredict:rx",
                 "autodetect candidates=%u (filtered=%u)%s",
                 state->autodetect_count,
                 skipped,
                 state->autodetect_limited ? " limited" : "");
    filtered_list = rotctld_autodetect_join_list(state->autodetect_list);
    if (filtered_list != NULL)
    {
        rot_term_log(ctrl, "gpredict:rx",
                     "rotor probe: enumerated devices: %s",
                     filtered_list);
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "rotor probe: enumerated devices: %s",
                    filtered_list);
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "autodetect list=%s prefer=%s",
                    filtered_list,
                    prefer_cu ? "cu" : "tty");
        rot_term_log_verbose(ctrl, "gpredict:rx",
                             "autodetect list=%s prefer=%s",
                             filtered_list,
                             prefer_cu ? "cu" : "tty");
        g_free(filtered_list);
    }
    else
    {
        rot_term_log(ctrl, "gpredict:rx",
                     "rotor probe: enumerated devices: (none)");
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "rotor probe: enumerated devices: (none)");
    }
}

static gboolean rotctld_autodetect_has_more(const RotctldProbeState *state)
{
    return state && state->autodetect_next != NULL;
}

static gchar *rotctld_autodetect_next_device(RotctldProbeState *state)
{
    gchar *device = NULL;

    if (state == NULL || state->autodetect_next == NULL)
        return NULL;

    device = g_strdup(state->autodetect_next->data);
    state->autodetect_next = state->autodetect_next->next;
    return device;
}

static RotctldProbeState *rotctld_probe_state_ref(RotctldProbeState *state)
{
    if (state == NULL)
        return NULL;

    g_atomic_int_inc(&state->ref_count);
    return state;
}

static void rotctld_probe_state_finalize(RotctldProbeState *state)
{
    if (state == NULL)
        return;

    g_warn_if_fail(state->validate_thread == NULL);
    g_mutex_clear(&state->validate_mutex);

    if (state->ctrl)
        g_object_unref(state->ctrl);

    g_free(state->spawn_summary);
    if (state->autodetect_list)
        gp_serial_free_candidates(state->autodetect_list);
    if (state->autodetect_list_full)
        gp_serial_free_candidates(state->autodetect_list_full);
    g_free(state->autodetect_device);
    g_free(state->best_device);
    g_free(state->last_good_device);
    g_free(state->validate_reason);
    if (state->autodetect_attempts)
        g_string_free(state->autodetect_attempts, TRUE);
    g_free(state);
}

static void rotctld_probe_state_unref(RotctldProbeState *state)
{
    if (state == NULL)
        return;

    if (g_atomic_int_dec_and_test(&state->ref_count))
        rotctld_probe_state_finalize(state);
}

static void rotctld_probe_state_detach(RotctldProbeState *state)
{
    GtkRotCtrl *ctrl = NULL;

    if (state == NULL)
        return;

    ctrl = state->ctrl;
    if (ctrl != NULL)
    {
        if (ctrl->rotctld_probe_state == state)
            ctrl->rotctld_probe_state = NULL;
        ctrl->rotctld_probe_id = 0;
    }
}

static void rotctld_probe_cancel(GtkRotCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    if (ctrl->rotctld_probe_id != 0)
    {
        g_source_remove(ctrl->rotctld_probe_id);
        ctrl->rotctld_probe_id = 0;
    }

    if (ctrl->rotctld_probe_state)
    {
        RotctldProbeState *state = ctrl->rotctld_probe_state;
        if (state->spawned && ctrl->rotctld_mgr)
            rotctld_process_stop_async(ctrl, "probe_cancel");
        state->cancelled = TRUE;
        rotctld_probe_state_detach(state);
        rotctld_probe_state_unref(state);
    }
}

static void rotctld_finish_engage(GtkRotCtrl *ctrl)
{
    guint64 generation = 0;

    if (ctrl == NULL)
        return;
    if (ctrl->LockBut &&
        !gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ctrl->LockBut)))
        return;

    generation = rotctld_get_engage_generation(ctrl);

    g_mutex_lock(&ctrl->client.mutex);
    ctrl->client.io_error  = FALSE;
    ctrl->client.cmd_rejected = FALSE;
    ctrl->client.send_quit = FALSE;
    ctrl->client.reject_backoff_until_us = 0;
    ctrl->client.reject_backoff_sec = 0.5;
    ctrl->client.reject_backoff_log_us = 0;
    ctrl->client.limits_valid = FALSE;
    ctrl->client.south_zero = FALSE;
    g_atomic_int_set(&ctrl->south_zero_cached, 0);
    ctrl->client.new_trg   = FALSE;
    if (ctrl->client.thread == NULL)
    {
        ctrl->client.pos_valid = FALSE;
        ctrl->client.pos_invalid = FALSE;
        ctrl->client.pos_unknown = FALSE;
        ctrl->client.pos_cmd_ok = FALSE;
        ctrl->client.set_pos_ok = FALSE;
        ctrl->client.last_pos_us = 0;
        ctrl->client.last_pos_attempt_us = 0;
        ctrl->client.last_set_attempt_us = 0;
    }
    ctrl->client.backend_ioerr_count = 0;
    ctrl->client.backend_ioerr_first_us = 0;
    ctrl->client.backend_ioerr_disengage_pending = FALSE;
    g_mutex_unlock(&ctrl->client.mutex);

    rotctrl_calib_reload(ctrl);

    /* Reset error counter when (re)engaging the rotor. */
    ctrl->errcnt = 0;
    ctrl->out_of_range = FALSE;
    ctrl->last_oob_log_us = 0;
    ctrl->manual_sync_pending = TRUE;
    ctrl->az_hold_active = FALSE;
    ctrl->az_hold_value = 0.0;
    ctrl->tracking_active = FALSE;
    ctrl->last_cmd_time_us = 0;
    ctrl->last_send_us = 0;
    ctrl->above_eps_count = 0;
    ctrl->axis_swap_warned = FALSE;
    ctrl->limits_logged = FALSE;
    ctrl->wrap_mismatch_logged = FALSE;
    memset(&ctrl->backend_limits, 0, sizeof(ctrl->backend_limits));
    memset(&ctrl->backend_clamp_rate, 0, sizeof(ctrl->backend_clamp_rate));
    memset(&ctrl->send_log_rate, 0, sizeof(ctrl->send_log_rate));
    memset(&ctrl->cmd_skip_rate, 0, sizeof(ctrl->cmd_skip_rate));
    rotctrl_update_user_limits(ctrl);
    g_mutex_lock(&ctrl->client.mutex);
    ctrl->client.last_cmd_us = 0;
    ctrl->client.last_cmd_ok_az = 0.0;
    ctrl->client.last_cmd_ok_el = 0.0;
    ctrl->client.last_cmd_ok_us = 0;
    ctrl->client.last_cmd_backend_az = 0.0;
    ctrl->client.last_cmd_backend_valid = FALSE;
    ctrl->client.last_pos_user_az = 0.0;
    ctrl->client.last_pos_user_el = 0.0;
    ctrl->client.last_pos_sample_us = 0;
    ctrl->client.last_err_to_cmd = 0.0;
    ctrl->client.last_err_time_us = 0;
    ctrl->client.move_grace_until_us = 0;
    ctrl->client.discrepancy_streak = 0;
    ctrl->client.wrong_way_count = 0;
    ctrl->client.discrepancy_pending = FALSE;
    ctrl->client.feedback_disabled_logged = FALSE;
    g_mutex_unlock(&ctrl->client.mutex);

    ctrl->last_cmd_az360 = 0.0;
    ctrl->last_cmd_el = 0.0;
    ctrl->last_cmd_valid = FALSE;
    ctrl->last_cmd_backend_az = 0.0;
    ctrl->last_cmd_backend_el = 0.0;
    ctrl->last_cmd_backend_valid = FALSE;
    ctrl->committed_user_az = 0.0;
    ctrl->committed_user_el = 0.0;
    ctrl->committed_raw_az360 = 0.0;
    ctrl->committed_raw_el = 0.0;
    ctrl->committed_backend_az = 0.0;
    ctrl->committed_backend_el = 0.0;
    ctrl->committed_valid = FALSE;
    ctrl->committed_since_us = 0;
    ctrl->last_keepalive_time_us = 0;
    ctrl->last_desired_update_us = 0;
    ctrl->last_cmd_backend_az = 0.0;
    ctrl->last_cmd_backend_el = 0.0;
    ctrl->last_cmd_backend_valid = FALSE;
    ctrl->committed_user_az = 0.0;
    ctrl->committed_user_el = 0.0;
    ctrl->committed_raw_az360 = 0.0;
    ctrl->committed_raw_el = 0.0;
    ctrl->committed_backend_az = 0.0;
    ctrl->committed_backend_el = 0.0;
    ctrl->committed_valid = FALSE;
    ctrl->committed_since_us = 0;
    ctrl->last_keepalive_time_us = 0;
    ctrl->last_desired_update_us = 0;
    ctrl->last_send_us = 0;
    ctrl->above_eps_count = 0;
    ctrl->last_hold_log_us = 0;
    ctrl->pending_lane_valid = FALSE;
    ctrl->pending_lane_since_us = 0;
    ctrl->seam_valid = FALSE;
    ctrl->seam_crossing_active = FALSE;
    ctrl->seam_crossing_sent = FALSE;
    ctrl->seam_crossing_lane_valid = FALSE;
    ctrl->seam_crossing_lane_k = 0;
    ctrl->seam_crossing_target_az360 = 0.0;
    ctrl->seam_crossing_since_us = 0;
    ctrl->wrap_acquire_active = FALSE;
    ctrl->wrap_acquire_sent = FALSE;
    ctrl->wrap_acquire_target_backend = 0.0;
    ctrl->wrap_acquire_target_az360 = 0.0;
    ctrl->wrap_acquire_target_k = 0;
    ctrl->wrap_acquire_since_us = 0;
    ctrl->wrap_acquire_last_log_us = 0;

    if (ctrl->client.thread != NULL)
    {
        gboolean thread_done = FALSE;
        gboolean thread_matches = FALSE;

        g_mutex_lock(&ctrl->client.mutex);
        thread_done = ctrl->client.thread_done;
        thread_matches = (ctrl->client.thread_generation == generation);
        g_mutex_unlock(&ctrl->client.mutex);

        if (thread_done)
        {
            if (!rotctld_collect_client_thread(ctrl, FALSE, "engage"))
                return;
        }

        if (ctrl->client.thread != NULL)
        {
            if (!thread_matches)
            {
                RotorStateSnapshot snap = { 0 };
                rotctld_request_thread_stop(ctrl, TRUE);
                rot_session_set_state(ctrl, ROT_SESSION_CONNECTING,
                                      "waiting for previous session", FALSE);
                rotctrl_set_ui_status_detail(ctrl, "ENGAGING");
                snap.control_active = TRUE;
                snap.engaging = TRUE;
                snap.hard_error = ctrl->ui_hard_error;
                rotctrl_apply_ui_status(ctrl, &snap,
                                        "waiting for previous session");
                return;
            }

            ctrl->engage_pending = FALSE;
            sat_log_log(SAT_LOG_LEVEL_WARN,
                        _("%s: rotctld client thread already running; reusing existing thread"),
                        __func__);
            gtk_widget_set_sensitive(ctrl->DevSel, FALSE);
            ctrl->engaged = TRUE;
            rot_session_set_state(ctrl, ROT_SESSION_ENGAGING,
                                  "thread reuse", FALSE);
            {
                RotorStateSnapshot snap = { 0 };
                rotctrl_set_ui_status_detail(ctrl, "ENGAGING");
                snap.control_active = TRUE;
                snap.engaging = TRUE;
                snap.hard_error = ctrl->ui_hard_error;
                rotctrl_apply_ui_status(ctrl, &snap, "thread reuse");
            }
            return;
        }
    }

    ctrl->engage_pending = FALSE;
    g_mutex_lock(&ctrl->client.mutex);
    ctrl->client.thread_done = FALSE;
    ctrl->client.thread_generation = generation;
    g_mutex_unlock(&ctrl->client.mutex);
    g_atomic_int_set(&ctrl->client.stop_requested, 0);

    ctrl->client.thread =
        g_thread_new("gpredict_rotctl", rotctld_client_thread, ctrl);

    gtk_widget_set_sensitive(ctrl->DevSel, FALSE);
    ctrl->engaged = TRUE;
    rot_session_set_state(ctrl, ROT_SESSION_ENGAGING,
                          "thread started", FALSE);
    {
        RotorStateSnapshot snap = { 0 };
        rotctrl_set_ui_status_detail(ctrl, "ENGAGING");
        snap.control_active = TRUE;
        snap.engaging = TRUE;
        snap.hard_error = ctrl->ui_hard_error;
        rotctrl_apply_ui_status(ctrl, &snap, "thread started");
    }
    rot_term_log(ctrl, "gpredict:rx",
                 "rotctld engage started %s:%d",
                 ctrl->conf ? ctrl->conf->host : "(null)",
                 rotctld_effective_port(ctrl));
}

static void rotctld_fail_engage(GtkRotCtrl *ctrl, gboolean error_reported)
{
    if (ctrl == NULL)
        return;

    ctrl->engage_pending = FALSE;
    ctrl->engaged = FALSE;
    ctrl->tracking_active = FALSE;
    ctrl->have_user_command = FALSE;
    ctrl->az_hold_active = FALSE;
    ctrl->az_hold_value = 0.0;
    ctrl->axis_swap_warned = FALSE;
    ctrl->selected_child_port = 0;
    ctrl->hold_position_on_engage = FALSE;
    ctrl->hold_position_log_emitted = FALSE;
    ctrl->ok_streak = 0;
    ctrl->bad_streak = 0;
    ctrl->timeout_streak = 0;
    ctrl->comm_status = ROT_COMM_DISENGAGED;
    if (ctrl->LockBut &&
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ctrl->LockBut)))
    {
        rotctrl_ui_begin_update(ctrl, "engage_failed");
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->LockBut), FALSE);
        rotctrl_ui_end_update(ctrl, "engage_failed");
    }
    g_mutex_lock(&ctrl->client.mutex);
    ctrl->client.send_quit = FALSE;
    g_mutex_unlock(&ctrl->client.mutex);
    rot_session_set_state(ctrl, ROT_SESSION_DISCONNECTED,
                          "engage failed", FALSE);
    gtk_widget_set_sensitive(ctrl->DevSel, TRUE);
    rotctrl_set_ui_hard_error(ctrl, "rotctld engage failed");
    rotctrl_set_ui_status_detail(ctrl, "ERROR: rotctld");
    {
        RotorStateSnapshot snap = { 0 };
        snap.control_active = TRUE;
        snap.hard_error = TRUE;
        rotctrl_apply_ui_status(ctrl, &snap, "engage failed");
    }
    if (!error_reported)
        rot_show_no_rotor_dialog(ctrl);
}

static gboolean rotctld_spawn_autostart(GtkRotCtrl *ctrl,
                                        const gchar *device_override,
                                        gint baud_override,
                                        gint port_override,
                                        gchar **spawn_summary_out)
{
    gchar *device = NULL;
    gchar *errmsg = NULL;
    gboolean auto_picked = FALSE;
    const gchar *env_cmd = g_getenv("GPREDICT_ROTCTLD_CMD");
    gchar **argv = NULL;
    gint port = 0;
    gint rotctld_timeout_ms =
        rotctld_backend_timeout_ms(ctrl, device_override, port_override > 0);

    if (spawn_summary_out)
        *spawn_summary_out = NULL;

    if (ctrl == NULL || ctrl->conf == NULL)
        return FALSE;

    if (ctrl->rotctld_mgr && rotctld_mgr_is_running(ctrl->rotctld_mgr))
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "%s: rotctld already running; skipping autostart",
                    __func__);
        if (spawn_summary_out)
            *spawn_summary_out = g_strdup("rotctld already running");
        return TRUE;
    }

#ifdef __APPLE__
    env_cmd = NULL;
#endif

    if (env_cmd != NULL && *env_cmd != '\0')
        argv = rotctld_build_argv_from_command(ctrl, env_cmd);

    if (argv != NULL)
    {
        if (rotctld_timeout_ms != 1200)
            argv = rotctld_force_config_timeout(argv,
                                                rotctld_timeout_ms,
                                                TRUE);
        if (spawn_summary_out)
            *spawn_summary_out = rotctld_argv_to_string(argv);
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rotctld spawn: protocol=%s model=%d timeout=%d",
                    rot_protocol_name(ctrl->conf->protocol),
                    rot_conf_hamlib_model(ctrl->conf),
                    rotctld_timeout_ms);
        if (!rotctld_spawn_process(ctrl, argv))
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: failed to start rotctld using '%s'"),
                        __func__, env_cmd ? env_cmd : "(auto)");
            g_strfreev(argv);
            return FALSE;
        }
        g_strfreev(argv);
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: started rotctld using '%s'"),
                    __func__, env_cmd ? env_cmd : "(auto)");
        return TRUE;
    }

    {
        gint model = rot_conf_hamlib_model(ctrl->conf);
        gint baud = 0;
        const gchar *env_device = g_getenv("GPREDICT_ROT_SERIAL");
        const gchar *env_baud = g_getenv("GPREDICT_ROT_BAUD");

        if (baud_override > 0)
            baud = baud_override;
        else
            baud = ctrl->conf->baud > 0 ? ctrl->conf->baud
                                        : rot_protocol_default_baud(
                                            ctrl->conf->protocol);

        if (env_baud && *env_baud)
        {
            glong tmp = g_ascii_strtoll(env_baud, NULL, 10);
            if (tmp > 0)
                baud = (gint) tmp;
        }

        if (device_override && *device_override)
        {
            device = g_strdup(device_override);
            auto_picked = TRUE;
        }
        else
        {
            device = rotctld_resolve_device(ctrl, &auto_picked);
        }
        if (env_device && *env_device)
        {
            g_free(device);
            device = g_strdup(env_device);
            auto_picked = FALSE;
        }
        if (device == NULL || *device == '\0')
        {
            const gchar *msg = auto_picked
                               ? "No USB serial devices found"
                               : "No serial device selected.";
            rot_term_log(ctrl, "gpredict:err", "%s", msg);
            g_free(device);
            return FALSE;
        }

        rotctld_timeout_ms =
            rotctld_backend_timeout_ms(ctrl, device, port_override > 0);

        if (ctrl->rotctld_mgr)
            rotctld_process_stop(ctrl);

        port = port_override > 0 ? port_override : ctrl->conf->port;
        if (spawn_summary_out)
        {
            const gchar *bind_host = rotctld_bind_host(ctrl->conf->host);
            *spawn_summary_out =
                g_strdup_printf("rotctld -m %d -r %s -s %d -C timeout=%d -T %s -t %d",
                                model, device, baud,
                                rotctld_timeout_ms,
                                bind_host ? bind_host : "",
                                port);
        }
        if (spawn_summary_out && *spawn_summary_out)
            rot_term_log_verbose(ctrl, "gpredict:tx",
                                 "spawn rotctld: %s", *spawn_summary_out);

        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rotctld spawn: protocol=%s model=%d timeout=%d",
                    rot_protocol_name(ctrl->conf->protocol),
                    model,
                    rotctld_timeout_ms);
        ctrl->rotctld_mgr =
            rotctld_mgr_spawn_timeout(ctrl->conf->host,
                                      port,
                                      model,
                                      device,
                                      baud,
                                      ctrl->verbose_logging,
                                      rotctld_timeout_ms,
                                      &errmsg);
        if (ctrl->rotctld_mgr == NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: failed to start rotctld: %s"),
                        __func__, errmsg ? errmsg : "unknown error");
            rot_term_log(ctrl, "gpredict:err",
                         "Failed to start rotctld: %s",
                         errmsg ? errmsg : "unknown error");
            g_free(errmsg);
            g_free(device);
            return FALSE;
        }

        rotctld_mgr_set_log_callback(ctrl->rotctld_mgr, rotctld_log_cb, ctrl);
        rot_term_log_verbose(ctrl, "gpredict:rx",
                             "rotctld started pid=%s",
                             rotctld_mgr_get_identifier(ctrl->rotctld_mgr));
        if (device_override && *device_override)
            rot_term_log(ctrl, "gpredict:rx",
                         "autodetect: spawned rotctld pid=%s device=%s baud=%d",
                         rotctld_mgr_get_identifier(ctrl->rotctld_mgr),
                         device_override, baud);
        g_free(device);
    }

    return TRUE;
}

typedef struct {
    RotctldProbeState *state;
    guint64            generation;
    guint64            autodetect_generation;
    gchar             *host;
    gint               port;
    gint               timeout_ms;
    gint               retries;
    gint               retry_delay_ms;
    gint               settle_ms;
} RotctldAutodetectWorker;

static rotctld_pos_result_t
rotctld_probe_position(const gchar *host,
                       gint port,
                       gint timeout_ms,
                       gint retries,
                       gint retry_delay_ms,
                       gint *rprt_out,
                       gchar **reason_out)
{
    RotctldClient *probe = NULL;
    gchar *errmsg = NULL;
    gdouble az = 0.0;
    gdouble el = 0.0;
    rotctld_pos_result_t res = ROTCTLD_POS_IO_ERR;
    gint rprt = 0;
    HamlibResponseInfo info = { 0 };
    gchar reply[512];
    gboolean have_az = FALSE;
    gboolean have_el = FALSE;
    gboolean have_rprt = FALSE;
    gint parsed_rprt = 0;
    gboolean no_reply = FALSE;
    const gchar *bind_host = NULL;

    if (rprt_out)
        *rprt_out = 0;
    if (reason_out)
        *reason_out = NULL;
    (void)retry_delay_ms;

    probe = rotctld_client_new("rotctld-probe");
    if (probe == NULL)
    {
        if (reason_out)
            *reason_out = g_strdup("alloc");
        return ROTCTLD_POS_IO_ERR;
    }

    bind_host = rotctld_bind_host(host);
    if (!rotctld_client_connect(probe,
                                bind_host ? bind_host : host,
                                port,
                                timeout_ms,
                                &errmsg))
    {
        if (reason_out)
            *reason_out = g_strdup_printf("connect: %s",
                                          errmsg ? errmsg : "failed");
        g_free(errmsg);
        rotctld_client_free(&probe);
        return ROTCTLD_POS_IO_ERR;
    }

    reply[0] = '\0';
    res = rotctld_client_get_pos_ex_timeout(probe,
                                            &az,
                                            &el,
                                            &info,
                                            reply,
                                            sizeof(reply),
                                            timeout_ms,
                                            retries);
    (void)rotctld_parse_position_reply_ex(reply,
                                          &az,
                                          &el,
                                          &parsed_rprt,
                                          &have_az,
                                          &have_el,
                                          &have_rprt);
    rprt = parsed_rprt;
    if (res == ROTCTLD_POS_OK && have_rprt && parsed_rprt != 0)
        res = ROTCTLD_POS_RPRT_ERR;
    if (res != ROTCTLD_POS_RPRT_ERR && have_az && have_el)
        res = ROTCTLD_POS_OK;
    no_reply = (info.bytes == 0 && reply[0] == '\0');
    if (rprt_out)
        *rprt_out = rprt;
    if (reason_out)
    {
        switch (res)
        {
        case ROTCTLD_POS_OK:
            break;
        case ROTCTLD_POS_RPRT_ERR:
            *reason_out = g_strdup_printf("rprt %d", rprt);
            break;
        case ROTCTLD_POS_PARSE_FAIL:
            if (no_reply && !have_az && !have_el)
                *reason_out = g_strdup("io_timeout");
            else if (have_az && !have_el)
                *reason_out = g_strdup("timeout_wait_el");
            else if (!have_az && have_el)
                *reason_out = g_strdup("timeout_wait_az");
            else
                *reason_out = g_strdup("parse_fail");
            break;
        case ROTCTLD_POS_TIMEOUT:
            if (no_reply)
                *reason_out = g_strdup("io_timeout");
            else
                *reason_out = g_strdup("timeout");
            break;
        case ROTCTLD_POS_IO_ERR:
        default:
            *reason_out = g_strdup("io_error");
            break;
        }
    }

    rotctld_client_free(&probe);
    return res;
}

static gboolean rotctld_autodetect_worker_done_cb(gpointer data)
{
    RotctldProbeState *state = data;
    GThread *thread = NULL;

    if (state == NULL)
        return G_SOURCE_REMOVE;

    g_mutex_lock(&state->validate_mutex);
    thread = state->validate_thread;
    state->validate_thread = NULL;
    g_mutex_unlock(&state->validate_mutex);

    if (thread)
        g_thread_join(thread);

    rotctld_probe_state_unref(state);
    return G_SOURCE_REMOVE;
}

static gpointer rotctld_autodetect_validate_thread(gpointer data)
{
    RotctldAutodetectWorker *worker = data;
    RotctldProbeState *state = NULL;
    RotctldClient *probe = NULL;
    gchar *errmsg = NULL;
    gdouble az = 0.0;
    gdouble el = 0.0;
    rotctld_pos_result_t res = ROTCTLD_POS_IO_ERR;
    rotctld_pos_result_t pos_res = ROTCTLD_POS_IO_ERR;
    gint rprt = 0;
    HamlibResponseInfo info = { 0 };
    gchar dump_state[4096];
    gboolean dump_ok = FALSE;
    gboolean pos_ok = FALSE;
    gboolean saw_timeout = FALSE;
    gint score = 0;
    gchar *reason = NULL;
    const gchar *bind_host = NULL;

    if (worker == NULL)
        return NULL;

    state = worker->state;
    bind_host = rotctld_bind_host(worker->host);
    probe = rotctld_client_new("rotctld-autodetect");
    if (probe == NULL)
    {
        reason = g_strdup("alloc");
        goto done;
    }

    if (!rotctld_client_connect(probe,
                                bind_host ? bind_host : worker->host,
                                worker->port,
                                worker->timeout_ms,
                                &errmsg))
    {
        reason = g_strdup_printf("connect: %s",
                                 errmsg ? errmsg : "failed");
        g_free(errmsg);
        goto done;
    }

    if (worker->settle_ms > 0)
        g_usleep((gulong) worker->settle_ms * 1000);

    {
        gchar pos_reply[512];
        HamlibResponseInfo pos_info = { 0 };
        gboolean have_az = FALSE;
        gboolean have_el = FALSE;
        gboolean have_rprt = FALSE;
        gint parsed_rprt = 0;
        gint64 start_us = 0;
        gint64 elapsed_ms = 0;
        gboolean no_reply = FALSE;

        {
            const gint max_attempts = 3;
            const gint retry_sleep_ms = 100;

            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "autodetect validate_io: send cmd=p timeout=%d attempts=%d",
                        worker->timeout_ms,
                        max_attempts);

            for (gint attempt = 0; attempt < max_attempts; attempt++)
            {
                gboolean ok = FALSE;
                gboolean timeout_err = FALSE;
                gboolean any_bytes = FALSE;
                const gchar *status = "unknown";

                memset(&pos_info, 0, sizeof(pos_info));
                pos_reply[0] = '\0';
                have_az = FALSE;
                have_el = FALSE;
                have_rprt = FALSE;
                parsed_rprt = 0;

                start_us = g_get_monotonic_time();
                ok = rotctld_client_request_raw_timeout(probe,
                                                        "p\n",
                                                        worker->timeout_ms,
                                                        pos_reply,
                                                        sizeof(pos_reply),
                                                        &pos_info);
                elapsed_ms = (g_get_monotonic_time() - start_us) / 1000;
                no_reply = (pos_info.bytes == 0 && pos_reply[0] == '\0');
                any_bytes = (pos_info.bytes > 0 || pos_reply[0] != '\0');

                (void)rotctld_parse_position_reply_ex(pos_reply,
                                                      &az,
                                                      &el,
                                                      &parsed_rprt,
                                                      &have_az,
                                                      &have_el,
                                                      &have_rprt);

                timeout_err = (pos_info.err == EAGAIN ||
                               pos_info.err == EWOULDBLOCK ||
                               pos_info.err == ETIMEDOUT);

                if (have_az && have_el)
                {
                    pos_ok = TRUE;
                    pos_res = ROTCTLD_POS_OK;
                }
                else if (have_rprt && parsed_rprt != 0)
                {
                    pos_ok = FALSE;
                    pos_res = ROTCTLD_POS_RPRT_ERR;
                }
                else if (!any_bytes && timeout_err)
                {
                    pos_ok = FALSE;
                    pos_res = ROTCTLD_POS_TIMEOUT;
                }
                else if (!ok && !timeout_err)
                {
                    pos_ok = FALSE;
                    pos_res = ROTCTLD_POS_IO_ERR;
                }
                else
                {
                    pos_ok = FALSE;
                    pos_res = ROTCTLD_POS_PARSE_FAIL;
                }

                if (pos_ok && have_rprt && parsed_rprt != 0)
                {
                    pos_ok = FALSE;
                    pos_res = ROTCTLD_POS_RPRT_ERR;
                }

                if (pos_res == ROTCTLD_POS_OK)
                    status = "ok";
                else if (pos_res == ROTCTLD_POS_TIMEOUT)
                    status = "timeout";
                else if (pos_res == ROTCTLD_POS_PARSE_FAIL)
                    status = "parse_fail";
                else if (pos_res == ROTCTLD_POS_RPRT_ERR)
                    status = "rprt_err";
                else
                    status = "io_error";

                {
                    gchar *view = rotctld_sanitize_reply(pos_reply, 80);
                    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                                "autodetect validate_io: p attempt=%d/%d bytes=%" G_GSIZE_FORMAT " ms=%lld err=%d status=%s view=%s",
                                attempt + 1,
                                max_attempts,
                                pos_info.bytes,
                                (long long) elapsed_ms,
                                pos_info.err,
                                status,
                                view ? view : "(none)");
                    g_free(view);
                }

                if (pos_ok)
                    break;

                if (attempt + 1 < max_attempts)
                    g_usleep((gulong) retry_sleep_ms * 1000);
            }
        }

        rprt = have_rprt ? parsed_rprt : 0;
        if (state && state->ctrl)
        {
            gchar az_buf[G_ASCII_DTOSTR_BUF_SIZE];
            gchar el_buf[G_ASCII_DTOSTR_BUF_SIZE];
            const gchar *az_txt = have_az
                                  ? g_ascii_formatd(az_buf, sizeof(az_buf), "%.2f", az)
                                  : "n/a";
            const gchar *el_txt = have_el
                                  ? g_ascii_formatd(el_buf, sizeof(el_buf), "%.2f", el)
                                  : "n/a";
            const gchar *status = "unknown";
            gchar *view = NULL;
            if (pos_res == ROTCTLD_POS_OK)
                status = "ok";
            else if (pos_res == ROTCTLD_POS_TIMEOUT)
                status = "timeout";
            else if (pos_res == ROTCTLD_POS_PARSE_FAIL)
                status = "parse_fail";
            else if (pos_res == ROTCTLD_POS_RPRT_ERR)
                status = "rprt_err";
            else
                status = "io_error";
            view = rotctld_sanitize_reply(pos_reply, 80);
            rot_term_log_verbose(state->ctrl, "gpredict:rx",
                                 "autodetect validate: az=%s el=%s rprt=%d flags[az=%d el=%d rprt=%d] bytes=%" G_GSIZE_FORMAT " ms=%lld status=%s view=%s",
                                 az_txt,
                                 el_txt,
                                 parsed_rprt,
                                 have_az ? 1 : 0,
                                 have_el ? 1 : 0,
                                 have_rprt ? 1 : 0,
                                 pos_info.bytes,
                                 (long long) elapsed_ms,
                                 status,
                                 view ? view : "(none)");
            g_free(view);
        }

        if (!pos_ok)
        {
            if (pos_res == ROTCTLD_POS_RPRT_ERR)
            {
                reason = g_strdup("rprt_err");
            }
            else if (no_reply && !have_az && !have_el)
            {
                saw_timeout = TRUE;
                reason = g_strdup("io_timeout");
            }
            else if (pos_res == ROTCTLD_POS_TIMEOUT)
            {
                saw_timeout = TRUE;
                reason = g_strdup(have_az ? "timeout_wait_el" : "timeout_wait_az");
            }
            else if (pos_res == ROTCTLD_POS_PARSE_FAIL)
            {
                if (have_az && !have_el)
                {
                    saw_timeout = TRUE;
                    reason = g_strdup("timeout_wait_el");
                }
                else if (!have_az && have_el)
                {
                    saw_timeout = TRUE;
                    reason = g_strdup("timeout_wait_az");
                }
                else
                {
                    reason = g_strdup("invalid_numeric");
                }
            }
            else
            {
                reason = g_strdup("io_error");
            }
        }

        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "autodetect validate_io: p parsed az=%.2f el=%.2f flags[az=%d el=%d rprt=%d] rprt=%d res=%d reason=%s bytes=%" G_GSIZE_FORMAT " ms=%lld",
                    az,
                    el,
                    have_az ? 1 : 0,
                    have_el ? 1 : 0,
                    have_rprt ? 1 : 0,
                    parsed_rprt,
                    pos_res,
                    reason ? reason : "ok",
                    pos_info.bytes,
                    (long long) elapsed_ms);
    }

    if (pos_ok)
    {
        memset(&info, 0, sizeof(info));
        dump_state[0] = '\0';
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "autodetect validate_io: send cmd=\\dump_state (post-validate)");
        dump_ok = rotctld_client_request_raw(probe, "\\dump_state\n",
                                             dump_state, sizeof(dump_state),
                                             &info);
        if (dump_ok)
            score += 2;
        else
            sat_log_log(SAT_LOG_LEVEL_WARN,
                        "autodetect: dump_state probe failed after validation (err=%d)",
                        info.err);
    }

    if (pos_ok)
        score += 3;
    if (saw_timeout)
        score -= 3;

    if (pos_ok)
    {
        saw_timeout = FALSE;
        g_free(reason);
        reason = NULL;
        res = ROTCTLD_POS_OK;
        if (state && state->ctrl)
            rot_term_log_verbose(state->ctrl, "gpredict:rx",
                                 "autodetect validate: accept az=%.2f el=%.2f",
                                 az, el);
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "autodetect validate_io: accept az=%.2f el=%.2f rprt=%d",
                    az, el, rprt);
    }
    else if (pos_res == ROTCTLD_POS_RPRT_ERR)
        res = ROTCTLD_POS_RPRT_ERR;
    else if (pos_res == ROTCTLD_POS_IO_ERR)
        res = ROTCTLD_POS_IO_ERR;
    else if (saw_timeout)
        res = ROTCTLD_POS_TIMEOUT;
    else
        res = ROTCTLD_POS_PARSE_FAIL;

done:
    if (probe)
        rotctld_client_free(&probe);

    if (state)
    {
        g_mutex_lock(&state->validate_mutex);
        /* Generation guard prevents stale worker results from taking effect. */
        if (!state->cancelled &&
            state->generation == worker->generation &&
            state->autodetect_generation == worker->autodetect_generation)
        {
            state->validate_done = TRUE;
            state->validate_inflight = FALSE;
            state->validate_result = res;
            state->validate_score = score;
            state->validate_rprt = rprt;
            g_free(state->validate_reason);
            state->validate_reason = reason ? reason : NULL;
            state->validate_dump_ok = dump_ok;
            state->validate_pos_ok = pos_ok;
            state->validate_timeout = saw_timeout;
            reason = NULL;
        }
        g_mutex_unlock(&state->validate_mutex);
    }

    g_free(reason);

    g_idle_add(rotctld_autodetect_worker_done_cb, worker->state);
    g_free(worker->host);
    g_free(worker);
    return NULL;
}

/* Autodetect state machine tick: main thread only; heavy I/O lives in worker.
 * States: START_CHILD -> SETTLE -> WAIT_TCP_READY -> VALIDATE_IO -> SUCCESS,
 *         with FAIL_NEXT/ABORT for retries and exhaustion.
 * Generation guards prevent stale callbacks from mutating state across attempts. */
static rotctld_autodetect_step_t rotctld_autodetect_step(GtkRotCtrl *ctrl,
                                                         RotctldProbeState *state,
                                                         gint64 now_us)
{
    gchar *first_line = NULL;
    gchar *full_text = NULL;
    gchar *trunc = NULL;
    rotctld_probe_result_t result = ROTCTLD_PROBE_NOT_READY;

    if (ctrl == NULL || state == NULL || ctrl->conf == NULL)
        return ROTCTLD_AUTODETECT_STEP_FAIL;

    switch (state->autodetect_state)
    {
    case ROTCTLD_AUTODETECT_IDLE:
        rotctld_autodetect_init(state, ctrl);
        if (state->autodetect_device == NULL)
            state->autodetect_device = rotctld_autodetect_next_device(state);
        if (state->autodetect_device == NULL)
        {
            rot_term_log(ctrl, "gpredict:err",
                         "autodetect failed: no serial devices found");
            return ROTCTLD_AUTODETECT_STEP_FAIL;
        }
        if (state->autodetect_baud_index == 0)
            rotctld_autodetect_reset_baud(state, ctrl->conf);
        state->autodetect_port = rotctld_pick_ephemeral_port(ctrl->conf->port);
        if (state->autodetect_port <= 0)
        {
            rot_term_log(ctrl, "gpredict:err",
                         "autodetect failed: unable to reserve probe port");
            return ROTCTLD_AUTODETECT_STEP_FAIL;
        }
        state->autodetect_generation++;
        state->non_rotctld_count = 0;
        rotctld_autodetect_log_candidate(ctrl, state,
                                         state->autodetect_device,
                                         state->autodetect_baud);
        rotctld_autodetect_reset_validation(state);
        state->autodetect_port_free_deadline_us = 0;
        rotctld_autodetect_transition(ctrl, state,
                                      ROTCTLD_AUTODETECT_START_CHILD,
                                      "start");
        return ROTCTLD_AUTODETECT_STEP_CONTINUE;

    case ROTCTLD_AUTODETECT_START_CHILD:
        if (now_us < state->autodetect_settle_until_us)
            return ROTCTLD_AUTODETECT_STEP_CONTINUE;
        if (ctrl->client.thread != NULL)
        {
            gboolean thread_done = FALSE;

            g_mutex_lock(&ctrl->client.mutex);
            thread_done = ctrl->client.thread_done;
            g_mutex_unlock(&ctrl->client.mutex);

            if (!thread_done)
            {
                rotctld_request_thread_stop(ctrl, TRUE);
                rotctld_socket_close_quiet(ctrl, &ctrl->client.socket);
                return ROTCTLD_AUTODETECT_STEP_CONTINUE;
            }

            if (!rotctld_collect_client_thread(ctrl, FALSE,
                                               "autodetect_start_child"))
                return ROTCTLD_AUTODETECT_STEP_CONTINUE;
        }

        if (ctrl->client.running || ctrl->client.socket != -1)
        {
            rotctld_request_thread_stop(ctrl, TRUE);
            rotctld_socket_close_quiet(ctrl, &ctrl->client.socket);
            return ROTCTLD_AUTODETECT_STEP_CONTINUE;
        }

        if (state->autodetect_port <= 0)
        {
            state->autodetect_port = rotctld_pick_ephemeral_port(ctrl->conf->port);
            if (state->autodetect_port <= 0)
            {
                rot_term_log(ctrl, "gpredict:err",
                             "autodetect failed: unable to reserve probe port");
                rotctld_autodetect_transition(ctrl, state,
                                              ROTCTLD_AUTODETECT_ABORT,
                                              "port_alloc_fail");
                return ROTCTLD_AUTODETECT_STEP_FAIL;
            }
        }

        if (state->autodetect_port_free_deadline_us == 0)
            state->autodetect_port_free_deadline_us =
                now_us + ((gint64) ROTCTLD_AUTODETECT_PORT_FREE_TIMEOUT_MS * 1000);

        result = rotctld_probe_identity(ctrl->conf->host,
                                        state->autodetect_port > 0
                                            ? state->autodetect_port
                                            : rotctld_effective_port(ctrl),
                                        ROTCTLD_AUTODETECT_TICK_MS,
                                        &first_line,
                                        &full_text);
        if (result != ROTCTLD_PROBE_NOT_READY &&
            now_us < state->autodetect_port_free_deadline_us)
        {
            g_free(first_line);
            g_free(full_text);
            return ROTCTLD_AUTODETECT_STEP_CONTINUE;
        }
        if (result != ROTCTLD_PROBE_NOT_READY)
        {
            sat_log_log(SAT_LOG_LEVEL_WARN,
                        "autodetect[%llu:%llu] port busy; spawning anyway",
                        (unsigned long long) state->generation,
                        (unsigned long long) state->autodetect_generation);
        }

        g_free(first_line);
        g_free(full_text);
        first_line = NULL;
        full_text = NULL;

        g_free(state->spawn_summary);
        state->spawn_summary = NULL;
        state->spawn_attempted = TRUE;
        state->spawned =
            rotctld_spawn_autostart(ctrl,
                                    state->autodetect_device,
                                    state->autodetect_baud,
                                    state->autodetect_port,
                                    &state->spawn_summary);
        if (!state->spawned)
        {
            state->scanning_child_pid = -1;
            state->scanning_child_validated = FALSE;
            rotctld_autodetect_transition(ctrl, state,
                                          ROTCTLD_AUTODETECT_FAIL_NEXT,
                                          "spawn_fail");
            return ROTCTLD_AUTODETECT_STEP_CONTINUE;
        }

        state->scanning_child_pid = rotctld_mgr_pid(ctrl->rotctld_mgr);
        state->scanning_child_validated = FALSE;
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "autodetect spawn pid=%d dev=%s reason=spawned state=%s",
                    state->scanning_child_pid,
                    state->autodetect_device ? state->autodetect_device : "(null)",
                    rotctld_autodetect_state_name(state->autodetect_state));
        rot_term_log_verbose(ctrl, "gpredict:rx",
                             "autodetect spawn pid=%d dev=%s reason=spawned",
                             state->scanning_child_pid,
                             state->autodetect_device ? state->autodetect_device : "(null)");

        {
            gint settle_ms =
                rotctld_autodetect_get_ms("GPREDICT_ROT_AUTODETECT_SPAWN_SETTLE_MS",
                                          ROTCTLD_AUTODETECT_SPAWN_SETTLE_MS,
                                          10, 2000);
            state->autodetect_settle_until_us =
                now_us + ((gint64) settle_ms * 1000);
        }
        state->autodetect_tcp_deadline_us =
            now_us + ((gint64) ROTCTLD_AUTODETECT_TCP_READY_TIMEOUT_MS * 1000);
        rotctld_autodetect_transition(ctrl, state,
                                      ROTCTLD_AUTODETECT_SETTLE,
                                      "spawned");
        return ROTCTLD_AUTODETECT_STEP_CONTINUE;

    case ROTCTLD_AUTODETECT_SETTLE:
        if (now_us < state->autodetect_settle_until_us)
            return ROTCTLD_AUTODETECT_STEP_CONTINUE;
        rotctld_autodetect_transition(ctrl, state,
                                      ROTCTLD_AUTODETECT_WAIT_TCP_READY,
                                      "settled");
        return ROTCTLD_AUTODETECT_STEP_CONTINUE;

    case ROTCTLD_AUTODETECT_WAIT_TCP_READY:
        if (state->spawned &&
            ctrl->rotctld_mgr &&
            !rotctld_mgr_is_running(ctrl->rotctld_mgr))
        {
            rotctld_log_exit(ctrl, state, "autodetect_child_exit");
            state->spawned = FALSE;
            state->scanning_child_pid = -1;
            state->scanning_child_validated = FALSE;
            rotctld_autodetect_transition(ctrl, state,
                                          ROTCTLD_AUTODETECT_FAIL_NEXT,
                                          "child_exit");
            return ROTCTLD_AUTODETECT_STEP_CONTINUE;
        }

        if (now_us < state->autodetect_settle_until_us)
            return ROTCTLD_AUTODETECT_STEP_CONTINUE;

        result = rotctld_probe_identity(ctrl->conf->host,
                                        state->autodetect_port > 0
                                            ? state->autodetect_port
                                            : rotctld_effective_port(ctrl),
                                        ROTCTLD_AUTODETECT_TICK_MS,
                                        &first_line,
                                        &full_text);
        trunc = rotctld_truncate_line(first_line, 120);
        rot_term_log_verbose(ctrl, "gpredict:rx",
                             "autodetect probe host=%s port=%d result=%s first_line=%s",
                             ctrl->conf ? ctrl->conf->host : "(null)",
                             state->autodetect_port > 0
                                 ? state->autodetect_port
                                 : rotctld_effective_port(ctrl),
                             rotctld_probe_result_name(result),
                             trunc ? trunc : "(none)");
        g_free(trunc);

        if (result == ROTCTLD_PROBE_OK_ROTCTLD)
        {
            gint detected_model = 0;
            gint desired_model = rot_conf_hamlib_model(ctrl->conf);
            gboolean have_model = rotctld_extract_model(full_text,
                                                        &detected_model);
            if (have_model && detected_model > 0 && desired_model > 0 &&
                detected_model != desired_model)
            {
                rot_term_log(ctrl, "gpredict:err",
                             "rotctld model mismatch (expected %d, got %d) at %s:%d",
                             desired_model, detected_model,
                             ctrl->conf ? ctrl->conf->host : "(null)",
                             state->autodetect_port > 0
                                 ? state->autodetect_port
                                 : rotctld_effective_port(ctrl));
                if (state->spawned)
                    rotctld_autodetect_stop_child(ctrl, state,
                                                  "autodetect_model_mismatch");
                g_free(first_line);
                g_free(full_text);
                return ROTCTLD_AUTODETECT_STEP_FAIL;
            }

            {
                gint settle_ms =
                    rotctld_autodetect_get_ms("GPREDICT_ROT_AUTODETECT_TCP_SETTLE_MS",
                                              ROTCTLD_AUTODETECT_TCP_SETTLE_MS,
                                              10, 2000);
                state->autodetect_settle_until_us =
                    now_us + ((gint64) settle_ms * 1000);
            }
            rotctld_autodetect_transition(ctrl, state,
                                          ROTCTLD_AUTODETECT_VALIDATE_IO,
                                          "tcp_ready");
            g_free(first_line);
            g_free(full_text);
            return ROTCTLD_AUTODETECT_STEP_CONTINUE;
        }

        if (result == ROTCTLD_PROBE_NOT_ROTCTLD)
            state->non_rotctld_count++;
        else
            state->non_rotctld_count = 0;

        if (result == ROTCTLD_PROBE_NOT_ROTCTLD &&
            state->non_rotctld_count >= 3)
        {
            rot_term_log(ctrl, "gpredict:err",
                         "Port %d already in use by non-rotctld service (host=%s, probe=%s)",
                         state->autodetect_port > 0
                             ? state->autodetect_port
                             : rotctld_effective_port(ctrl),
                         ctrl->conf ? ctrl->conf->host : "(null)",
                         rotctld_probe_result_name(result));
            rot_schedule_wrong_daemon(ctrl,
                                      ctrl->conf ? ctrl->conf->host : NULL,
                                      state->autodetect_port > 0
                                          ? state->autodetect_port
                                          : rotctld_effective_port(ctrl));
            if (state->spawned)
                rotctld_autodetect_stop_child(ctrl, state,
                                              "autodetect_non_rotctld");
            g_free(first_line);
            g_free(full_text);
            return ROTCTLD_AUTODETECT_STEP_FAIL;
        }

        if (now_us >= state->autodetect_tcp_deadline_us)
        {
            rot_term_log(ctrl, "gpredict:err",
                         "autodetect: tcp_deadline host=%s port=%d (spawn: %s)",
                         ctrl->conf ? ctrl->conf->host : "(null)",
                         state->autodetect_port > 0
                             ? state->autodetect_port
                             : rotctld_effective_port(ctrl),
                         state->spawn_summary ? state->spawn_summary : "unknown");
            rotctld_autodetect_transition(ctrl, state,
                                          ROTCTLD_AUTODETECT_FAIL_NEXT,
                                          "tcp_deadline");
        }

        g_free(first_line);
        g_free(full_text);
        return ROTCTLD_AUTODETECT_STEP_CONTINUE;

    case ROTCTLD_AUTODETECT_VALIDATE_IO:
    {
        gboolean inflight = FALSE;
        gboolean done = FALSE;
        GThread *thread = NULL;

        if (state->spawned &&
            ctrl->rotctld_mgr &&
            !rotctld_mgr_is_running(ctrl->rotctld_mgr))
        {
            rotctld_log_exit(ctrl, state, "autodetect_child_exit");
            state->spawned = FALSE;
            state->scanning_child_pid = -1;
            state->scanning_child_validated = FALSE;
            rotctld_autodetect_transition(ctrl, state,
                                          ROTCTLD_AUTODETECT_FAIL_NEXT,
                                          "child_exit");
            return ROTCTLD_AUTODETECT_STEP_CONTINUE;
        }

        if (now_us < state->autodetect_settle_until_us)
            return ROTCTLD_AUTODETECT_STEP_CONTINUE;

        g_mutex_lock(&state->validate_mutex);
        inflight = state->validate_inflight;
        done = state->validate_done;
        thread = state->validate_thread;
        g_mutex_unlock(&state->validate_mutex);

        if (!inflight && !done && thread == NULL)
        {
            gint validate_timeout_ms = state->validate_timeout_ms;
            gint validate_floor_ms =
                rotctld_validate_timeout_floor_ms(ctrl,
                                                  state->autodetect_device);
            RotctldAutodetectWorker *worker = g_new0(RotctldAutodetectWorker, 1);

            if (validate_timeout_ms < ROTCTLD_AUTODETECT_LASTGOOD_WINDOW_MS)
                validate_timeout_ms = ROTCTLD_AUTODETECT_LASTGOOD_WINDOW_MS;
            if (validate_timeout_ms < 2500)
                validate_timeout_ms = 2500;
            if (validate_timeout_ms < validate_floor_ms)
                validate_timeout_ms = validate_floor_ms;
            if (validate_timeout_ms > 8000)
                validate_timeout_ms = 8000;

            worker->state = rotctld_probe_state_ref(state);
            worker->generation = state->generation;
            worker->autodetect_generation = state->autodetect_generation;
            worker->host = g_strdup(ctrl->conf->host);
            worker->port = state->autodetect_port > 0
                           ? state->autodetect_port
                           : rotctld_effective_port(ctrl);
            worker->timeout_ms = validate_timeout_ms;
            worker->retries = state->validate_retries;
            worker->retry_delay_ms = state->validate_retry_delay_ms;
            worker->settle_ms =
                rotctld_autodetect_get_ms("GPREDICT_ROT_AUTODETECT_TCP_SETTLE_MS",
                                          ROTCTLD_AUTODETECT_TCP_SETTLE_MS,
                                          10, 2000);

            g_mutex_lock(&state->validate_mutex);
            state->validate_inflight = TRUE;
            state->validate_done = FALSE;
            state->validate_result = ROTCTLD_POS_IO_ERR;
            state->validate_score = 0;
            state->validate_rprt = 0;
            g_free(state->validate_reason);
            state->validate_reason = NULL;
            state->validate_dump_ok = FALSE;
            state->validate_pos_ok = FALSE;
            state->validate_timeout = FALSE;
            state->validate_thread =
                g_thread_new("rotctld-autodetect",
                             rotctld_autodetect_validate_thread,
                             worker);
            g_mutex_unlock(&state->validate_mutex);

            rot_term_log(ctrl, "gpredict:rx",
                         "autodetect: validating rotor IO (p, timeout=%dms)...",
                         validate_timeout_ms);
            return ROTCTLD_AUTODETECT_STEP_CONTINUE;
        }

        if (done)
        {
            rotctld_pos_result_t res = ROTCTLD_POS_IO_ERR;
            gint rprt = 0;
            gchar *reason = NULL;
            gboolean pos_ok = FALSE;
            gboolean dump_ok = FALSE;
            gboolean saw_timeout = FALSE;
            gint score = 0;

            g_mutex_lock(&state->validate_mutex);
            res = state->validate_result;
            rprt = state->validate_rprt;
            reason = state->validate_reason ? g_strdup(state->validate_reason) : NULL;
            score = state->validate_score;
            dump_ok = state->validate_dump_ok;
            pos_ok = state->validate_pos_ok;
            saw_timeout = state->validate_timeout;
            g_mutex_unlock(&state->validate_mutex);

            if (pos_ok)
            {
                gint64 now_us = g_get_monotonic_time();
                gdouble elapsed_ms = 0.0;
                gint pid = -1;
                if (state->autodetect_candidate_start_us > 0)
                    elapsed_ms =
                        (now_us - state->autodetect_candidate_start_us) / 1000.0;

                g_free(state->best_device);
                state->best_device = g_strdup(state->autodetect_device);
                state->best_baud = state->autodetect_baud;
                state->best_score = score;

                sat_log_log(SAT_LOG_LEVEL_INFO,
                            "autodetect[%llu:%llu] validated score=%d dump_ok=%d pos_ok=%d timeout=%d time=%.1fms",
                            (unsigned long long) state->generation,
                            (unsigned long long) state->autodetect_generation,
                            score,
                            dump_ok ? 1 : 0,
                            pos_ok ? 1 : 0,
                            saw_timeout ? 1 : 0,
                            elapsed_ms);
                rot_term_log(ctrl, "gpredict:rx",
                             "autodetect: candidate ok: %s baud=%d port=%d (%.0fms)",
                             state->autodetect_device,
                             state->autodetect_baud,
                             state->autodetect_port,
                             elapsed_ms);
                rot_term_log(ctrl, "gpredict:rx",
                             "rotor probe: device=%s SUCCESS",
                             state->autodetect_device);
                sat_log_log(SAT_LOG_LEVEL_INFO,
                            "rotor probe: device=%s SUCCESS",
                            state->autodetect_device);

                if (state->scanning_child_pid > 0)
                    pid = state->scanning_child_pid;
                else if (ctrl->rotctld_mgr)
                    pid = rotctld_mgr_pid(ctrl->rotctld_mgr);
                rot_term_log(ctrl, "gpredict:rx",
                             "autodetect: VALIDATE_IO OK -> SUCCESS selected pid=%d dev=%s baud=%d port=%d",
                             pid,
                             state->autodetect_device ? state->autodetect_device : "(null)",
                             state->autodetect_baud,
                             state->autodetect_port);
                sat_log_log(SAT_LOG_LEVEL_INFO,
                            "autodetect[%llu:%llu] VALIDATE_IO OK -> SUCCESS selected pid=%d dev=%s baud=%d port=%d",
                            (unsigned long long) state->generation,
                            (unsigned long long) state->autodetect_generation,
                            pid,
                            state->autodetect_device ? state->autodetect_device : "(null)",
                            state->autodetect_baud,
                            state->autodetect_port);

                state->scanning_child_validated = TRUE;
                g_free(reason);
                /* Bugfix: validated candidates are terminal; do not score/FAIL_NEXT/replace. */
                rotctld_autodetect_transition(ctrl, state,
                                              ROTCTLD_AUTODETECT_SUCCESS,
                                              "validated");
                return ROTCTLD_AUTODETECT_STEP_CONTINUE;
            }

            {
                gint64 now_us = g_get_monotonic_time();
                gdouble elapsed_ms = 0.0;
                const gchar *fail_reason =
                    (reason && *reason) ? reason : "validate_fail";

                if (state->autodetect_candidate_start_us > 0)
                    elapsed_ms =
                        (now_us - state->autodetect_candidate_start_us) / 1000.0;

                if (state->autodetect_baud > 0)
                    rot_term_log(ctrl, "gpredict:err",
                                 "autodetect: candidate failed: %s baud=%d port=%d (%s, %.0fms)",
                                 state->autodetect_device,
                                 state->autodetect_baud,
                                 state->autodetect_port,
                                 reason ? reason : "unknown",
                                 elapsed_ms);
                else
                    rot_term_log(ctrl, "gpredict:err",
                                 "autodetect: candidate failed: %s port=%d (%s, %.0fms)",
                                 state->autodetect_device,
                                 state->autodetect_port,
                                 reason ? reason : "unknown",
                                 elapsed_ms);

                rot_term_log(ctrl, "gpredict:err",
                             "rotor probe: device=%s get_position failed rprt=%d",
                             state->autodetect_device,
                             rprt);
                sat_log_log(SAT_LOG_LEVEL_WARN,
                            "rotor probe: device=%s get_position failed rprt=%d",
                            state->autodetect_device,
                            rprt);

                sat_log_log(SAT_LOG_LEVEL_WARN,
                            "autodetect[%llu:%llu] candidate failed reason=%s rprt=%d res=%d time=%.1fms",
                            (unsigned long long) state->generation,
                            (unsigned long long) state->autodetect_generation,
                            fail_reason,
                            rprt,
                            res,
                            elapsed_ms);
            }

            state->scanning_child_validated = FALSE;
            g_free(reason);
            rotctld_autodetect_transition(ctrl, state,
                                          ROTCTLD_AUTODETECT_FAIL_NEXT,
                                          "validate_fail");
            return ROTCTLD_AUTODETECT_STEP_CONTINUE;
        }

        return ROTCTLD_AUTODETECT_STEP_CONTINUE;
    }

    case ROTCTLD_AUTODETECT_SUCCESS:
        return ROTCTLD_AUTODETECT_STEP_SUCCESS;

    case ROTCTLD_AUTODETECT_FAIL_NEXT:
    {
        gboolean inflight = FALSE;
        gboolean validated_ok = FALSE;
        gboolean have_next_baud = FALSE;
        gboolean have_next_device = FALSE;
        gboolean keep_current = FALSE;
        g_mutex_lock(&state->validate_mutex);
        inflight = state->validate_inflight;
        g_mutex_unlock(&state->validate_mutex);

        if (inflight)
            return ROTCTLD_AUTODETECT_STEP_CONTINUE;

        if (state->last_good_exclusive)
        {
            gchar *key = NULL;
            guint full_count = 0;

            if (state->spawned)
                rotctld_autodetect_stop_child(ctrl, state,
                                              "autodetect_last_good_fail");
            rotctld_socket_close_quiet(ctrl, &ctrl->client.socket);
            rotctld_clear_rxbuf(ctrl);

            state->last_good_exclusive = FALSE;
            state->last_good_deadline_us = 0;

            if (state->autodetect_list_full)
            {
                key = rotctld_autodetect_candidate_key(state->last_good_device);
                if (key)
                    state->autodetect_list_full =
                        rotctld_autodetect_remove_key(state->autodetect_list_full,
                                                      key);
                g_free(key);
            }

            if (state->autodetect_list)
            {
                gp_serial_free_candidates(state->autodetect_list);
                state->autodetect_list = NULL;
            }

            if (state->autodetect_list_full)
                full_count = g_slist_length(state->autodetect_list_full);

            if (full_count > ROTCTLD_AUTODETECT_MAX_CANDIDATES)
            {
                state->autodetect_list =
                    rotctld_autodetect_limit_list(state->autodetect_list_full,
                                                  ROTCTLD_AUTODETECT_MAX_CANDIDATES);
                state->autodetect_limited = TRUE;
            }
            else
            {
                state->autodetect_list = state->autodetect_list_full;
                state->autodetect_list_full = NULL;
                state->autodetect_limited = FALSE;
            }

            state->autodetect_next = state->autodetect_list;
            state->autodetect_count = g_slist_length(state->autodetect_list);
            g_free(state->autodetect_device);
            state->autodetect_device = rotctld_autodetect_next_device(state);
            rotctld_autodetect_reset_baud(state, ctrl->conf);
            state->autodetect_port = rotctld_pick_ephemeral_port(ctrl->conf->port);
            if (state->autodetect_port <= 0)
            {
                rot_term_log(ctrl, "gpredict:err",
                             "autodetect failed: unable to reserve probe port");
                rotctld_autodetect_transition(ctrl, state,
                                              ROTCTLD_AUTODETECT_ABORT,
                                              "port_alloc_fail");
                return ROTCTLD_AUTODETECT_STEP_FAIL;
            }
            state->spawn_attempted = FALSE;
            state->spawned = FALSE;
            g_free(state->spawn_summary);
            state->spawn_summary = NULL;
            state->autodetect_port_free_deadline_us = 0;
            state->autodetect_generation++;
            state->non_rotctld_count = 0;
            state->autodetect_settle_until_us =
                now_us + ((gint64) ROTCTLD_AUTODETECT_COOLDOWN_MS * 1000);

            rot_term_log(ctrl, "gpredict:rx",
                         "autodetect: last_good failed; scanning %u candidates%s",
                         state->autodetect_count,
                         state->autodetect_limited ? " (limited)" : "");

            if (state->autodetect_device == NULL)
            {
                rotctld_autodetect_transition(ctrl, state,
                                              ROTCTLD_AUTODETECT_ABORT,
                                              "last_good_no_candidates");
                return ROTCTLD_AUTODETECT_STEP_FAIL;
            }

            rotctld_autodetect_log_candidate(ctrl, state,
                                             state->autodetect_device,
                                             state->autodetect_baud);
            rotctld_autodetect_transition(ctrl, state,
                                          ROTCTLD_AUTODETECT_START_CHILD,
                                          "last_good_fallback");
            return ROTCTLD_AUTODETECT_STEP_CONTINUE;
        }

        validated_ok = state->scanning_child_validated;
        have_next_baud = rotctld_autodetect_has_next_baud(state, ctrl->conf);
        have_next_device = rotctld_autodetect_has_more(state);
        keep_current = validated_ok && !have_next_baud && !have_next_device;

        if (keep_current && state->best_device != NULL)
        {
            if (g_strcmp0(state->autodetect_device, state->best_device) != 0)
            {
                g_free(state->autodetect_device);
                state->autodetect_device = g_strdup(state->best_device);
                state->autodetect_baud = state->best_baud;
            }
            rot_term_log(ctrl, "gpredict:rx",
                         "autodetect: best score=%d device=%s baud=%d",
                         state->best_score,
                         state->best_device,
                         state->best_baud);
            rotctld_autodetect_transition(ctrl, state,
                                          ROTCTLD_AUTODETECT_SUCCESS,
                                          "best_score");
            return ROTCTLD_AUTODETECT_STEP_CONTINUE;
        }

        if (state->spawned)
        {
            const gchar *stop_reason =
                validated_ok ? "autodetect_replace" : "autodetect_fail_next";
            rotctld_autodetect_stop_child(ctrl, state, stop_reason);
        }
        rotctld_socket_close_quiet(ctrl, &ctrl->client.socket);
        rotctld_clear_rxbuf(ctrl);

        if (state->autodetect_port_free_deadline_us == 0)
            state->autodetect_port_free_deadline_us =
                now_us + ((gint64) ROTCTLD_AUTODETECT_PORT_FREE_TIMEOUT_MS * 1000);

        result = rotctld_probe_identity(ctrl->conf->host,
                                        state->autodetect_port > 0
                                            ? state->autodetect_port
                                            : rotctld_effective_port(ctrl),
                                        ROTCTLD_AUTODETECT_TICK_MS,
                                        &first_line,
                                        &full_text);
        if (result != ROTCTLD_PROBE_NOT_READY &&
            now_us < state->autodetect_port_free_deadline_us)
        {
            g_free(first_line);
            g_free(full_text);
            return ROTCTLD_AUTODETECT_STEP_CONTINUE;
        }
        g_free(first_line);
        g_free(full_text);

        if (rotctld_autodetect_next_baud(state, ctrl->conf))
        {
            state->spawn_attempted = FALSE;
            state->spawned = FALSE;
            g_free(state->spawn_summary);
            state->spawn_summary = NULL;
            state->autodetect_port_free_deadline_us = 0;
            state->autodetect_generation++;
            state->non_rotctld_count = 0;
            state->autodetect_port = rotctld_pick_ephemeral_port(ctrl->conf->port);
            if (state->autodetect_port <= 0)
            {
                rot_term_log(ctrl, "gpredict:err",
                             "autodetect failed: unable to reserve probe port");
                rotctld_autodetect_transition(ctrl, state,
                                              ROTCTLD_AUTODETECT_ABORT,
                                              "port_alloc_fail");
                return ROTCTLD_AUTODETECT_STEP_FAIL;
            }
            rotctld_autodetect_log_candidate(ctrl, state,
                                             state->autodetect_device,
                                             state->autodetect_baud);
            rotctld_autodetect_reset_validation(state);
            state->autodetect_settle_until_us =
                now_us + ((gint64) ROTCTLD_AUTODETECT_COOLDOWN_MS * 1000);
            rotctld_autodetect_transition(ctrl, state,
                                          ROTCTLD_AUTODETECT_START_CHILD,
                                          "next_baud");
            return ROTCTLD_AUTODETECT_STEP_CONTINUE;
        }

        if (rotctld_autodetect_has_more(state))
        {
            g_free(state->autodetect_device);
            state->autodetect_device = rotctld_autodetect_next_device(state);
            rotctld_autodetect_reset_baud(state, ctrl->conf);
            state->spawn_attempted = FALSE;
            state->spawned = FALSE;
            g_free(state->spawn_summary);
            state->spawn_summary = NULL;
            state->autodetect_port_free_deadline_us = 0;
            state->autodetect_generation++;
            state->non_rotctld_count = 0;
            state->autodetect_port = rotctld_pick_ephemeral_port(ctrl->conf->port);
            if (state->autodetect_port <= 0)
            {
                rot_term_log(ctrl, "gpredict:err",
                             "autodetect failed: unable to reserve probe port");
                rotctld_autodetect_transition(ctrl, state,
                                              ROTCTLD_AUTODETECT_ABORT,
                                              "port_alloc_fail");
                return ROTCTLD_AUTODETECT_STEP_FAIL;
            }
            rotctld_autodetect_log_candidate(ctrl, state,
                                             state->autodetect_device,
                                             state->autodetect_baud);
            rotctld_autodetect_reset_validation(state);
            state->autodetect_settle_until_us =
                now_us + ((gint64) ROTCTLD_AUTODETECT_COOLDOWN_MS * 1000);
            rotctld_autodetect_transition(ctrl, state,
                                          ROTCTLD_AUTODETECT_START_CHILD,
                                          "next_device");
            return ROTCTLD_AUTODETECT_STEP_CONTINUE;
        }

        if (state->best_device != NULL)
        {
            g_free(state->autodetect_device);
            state->autodetect_device = g_strdup(state->best_device);
            state->autodetect_baud = state->best_baud;
            rot_term_log(ctrl, "gpredict:rx",
                         "autodetect: best score=%d device=%s baud=%d",
                         state->best_score,
                         state->best_device,
                         state->best_baud);
            rotctld_autodetect_transition(ctrl, state,
                                          ROTCTLD_AUTODETECT_SUCCESS,
                                          "best_score");
            return ROTCTLD_AUTODETECT_STEP_CONTINUE;
        }

        if (state->autodetect_limited &&
            state->autodetect_list_full != NULL)
        {
            guint full_count = g_slist_length(state->autodetect_list_full);
            state->autodetect_limited = FALSE;

            if (state->autodetect_list)
            {
                gp_serial_free_candidates(state->autodetect_list);
                state->autodetect_list = NULL;
            }

            state->autodetect_list = state->autodetect_list_full;
            state->autodetect_list_full = NULL;
            state->autodetect_next = state->autodetect_list;
            state->autodetect_count = g_slist_length(state->autodetect_list);
            g_free(state->autodetect_device);
            state->autodetect_device = rotctld_autodetect_next_device(state);
            rotctld_autodetect_reset_baud(state, ctrl->conf);
            state->spawn_attempted = FALSE;
            state->spawned = FALSE;
            g_free(state->spawn_summary);
            state->spawn_summary = NULL;
            state->autodetect_port_free_deadline_us = 0;
            state->autodetect_generation++;
            state->non_rotctld_count = 0;
            state->autodetect_port = rotctld_pick_ephemeral_port(ctrl->conf->port);
            if (state->autodetect_port <= 0)
            {
                rot_term_log(ctrl, "gpredict:err",
                             "autodetect failed: unable to reserve probe port");
                rotctld_autodetect_transition(ctrl, state,
                                              ROTCTLD_AUTODETECT_ABORT,
                                              "port_alloc_fail");
                return ROTCTLD_AUTODETECT_STEP_FAIL;
            }
            rotctld_autodetect_log_candidate(ctrl, state,
                                             state->autodetect_device,
                                             state->autodetect_baud);
            state->autodetect_settle_until_us =
                now_us + ((gint64) ROTCTLD_AUTODETECT_COOLDOWN_MS * 1000);
            rot_term_log(ctrl, "gpredict:rx",
                         "autodetect: expanding candidate list to %u", full_count);
            rotctld_autodetect_transition(ctrl, state,
                                          ROTCTLD_AUTODETECT_START_CHILD,
                                          "expand_candidates");
            return ROTCTLD_AUTODETECT_STEP_CONTINUE;
        }

        rotctld_autodetect_transition(ctrl, state,
                                      ROTCTLD_AUTODETECT_ABORT,
                                      "exhausted");
        if (state->autodetect_start_us > 0)
        {
            gdouble total_ms =
                (now_us - state->autodetect_start_us) / 1000.0;
            rot_term_log(ctrl, "gpredict:err",
                         "autodetect: exhausted after %.0fms",
                         total_ms);
        }
        return ROTCTLD_AUTODETECT_STEP_FAIL;
    }

    case ROTCTLD_AUTODETECT_ABORT:
        return ROTCTLD_AUTODETECT_STEP_FAIL;
    }

    return ROTCTLD_AUTODETECT_STEP_CONTINUE;
}

static gboolean rotctld_probe_retry_cb(gpointer data)
{
    RotctldProbeState *state = data;
    GtkRotCtrl *ctrl = state ? state->ctrl : NULL;
    gint64 now_us = g_get_monotonic_time();
    rotctld_probe_result_t result = ROTCTLD_PROBE_NOT_READY;
    gchar *first_line = NULL;
    gchar *full_text = NULL;
    gchar *trunc = NULL;
    gint probe_timeout_ms = 100;
    rotctld_autodetect_step_t autodetect_step = ROTCTLD_AUTODETECT_STEP_CONTINUE;
    gint probe_port = 0;

    if (ctrl == NULL)
    {
        rotctld_probe_state_unref(state);
        return G_SOURCE_REMOVE;
    }

    if (ctrl->conf == NULL)
    {
        rotctld_probe_state_detach(state);
        rotctld_probe_state_unref(state);
        return G_SOURCE_REMOVE;
    }

    if (rotctld_stop_requested(ctrl))
    {
        state->cancelled = TRUE;
        rotctld_probe_state_detach(state);
        rotctld_probe_state_unref(state);
        return G_SOURCE_REMOVE;
    }

    /* Generation guard: abort stale probe callbacks. */
    if (state->generation != rotctld_get_engage_generation(ctrl))
    {
        state->cancelled = TRUE;
        rotctld_probe_state_detach(state);
        rotctld_probe_state_unref(state);
        return G_SOURCE_REMOVE;
    }

    ctrl->rotctld_probe_id = 0;

    if (ctrl->LockBut &&
        !gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ctrl->LockBut)))
    {
        state->cancelled = TRUE;
        rotctld_probe_state_detach(state);
        rotctld_probe_state_unref(state);
        return G_SOURCE_REMOVE;
    }

    if (state->autodetect_enabled)
    {
        autodetect_step = rotctld_autodetect_step(ctrl, state, now_us);
        if (autodetect_step == ROTCTLD_AUTODETECT_STEP_SUCCESS)
        {
            rotctld_autodetect_apply_success(ctrl, state);
            rotctld_probe_state_detach(state);
            rotctld_finish_engage(ctrl);
            rotctld_probe_state_unref(state);
            return G_SOURCE_REMOVE;
        }
        if (autodetect_step == ROTCTLD_AUTODETECT_STEP_FAIL)
        {
            gboolean reported = FALSE;
            if (state->autodetect_attempts &&
                state->autodetect_attempts->len > 0)
            {
                gint model = rot_conf_hamlib_model(ctrl->conf);
                rot_term_log(ctrl, "gpredict:err",
                             "rotor probe: failed; attempted: %s",
                             state->autodetect_attempts->str);
                sat_log_log(SAT_LOG_LEVEL_WARN,
                            "rotor probe: failed; attempted: %s",
                            state->autodetect_attempts->str);
                gchar *detail = g_strdup_printf(
                    _("Rotor probe failed: no compatible rotor response "
                      "(dump_state + get_position). Model=%d. Tried: %s"),
                    model,
                    state->autodetect_attempts->str);
                rot_show_message(ctrl,
                                 GTK_MESSAGE_ERROR,
                                 _("Rotor error"),
                                 detail);
                g_free(detail);
                reported = TRUE;
            }
            rotctld_probe_state_detach(state);
            rotctld_fail_engage(ctrl, reported);
            rotctld_probe_state_unref(state);
            return G_SOURCE_REMOVE;
        }

        state->delay_ms = ROTCTLD_AUTODETECT_TICK_MS;
        ctrl->rotctld_probe_id =
            g_timeout_add_full(G_PRIORITY_DEFAULT,
                               state->delay_ms,
                               rotctld_probe_retry_cb,
                               state,
                               NULL);
        return G_SOURCE_REMOVE;
    }

    state->attempt++;
    probe_timeout_ms = ROTCTLD_AUTODETECT_TICK_MS;
    probe_port = rotctld_effective_port(ctrl);

    result = rotctld_probe_identity(ctrl->conf->host,
                                    probe_port,
                                    probe_timeout_ms,
                                    &first_line,
                                    &full_text);

    trunc = rotctld_truncate_line(first_line, 120);
    rot_term_log(ctrl, "gpredict:rx",
                 "probe #%u host=%s port=%d cmd=\\dump_state result=%s first_line=%s",
                 state->attempt,
                 ctrl->conf ? ctrl->conf->host : "(null)",
                 probe_port,
                 rotctld_probe_result_name(result),
                 trunc ? trunc : "(none)");
    g_free(trunc);

    if (result == ROTCTLD_PROBE_OK_ROTCTLD)
    {
        gint detected_model = 0;
        gint desired_model = rot_conf_hamlib_model(ctrl->conf);
        gchar *line1 = NULL;
        gchar *line2 = NULL;
        gboolean have_model = FALSE;
        rotctld_pos_result_t pos_res = ROTCTLD_POS_IO_ERR;
        gint rprt = 0;
        gchar *pos_reason = NULL;
        gboolean pos_ok = FALSE;

        have_model = rotctld_extract_model(full_text, &detected_model);
        rotctld_extract_first_lines(full_text, &line1, &line2);
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rotctld dump_state: line1=%s line2=%s model=%d",
                    line1 ? line1 : "(none)",
                    line2 ? line2 : "(none)",
                    detected_model);

        if (have_model && detected_model > 0 && desired_model > 0 &&
            detected_model != desired_model)
        {
            rot_term_log(ctrl, "gpredict:err",
                         "rotctld model mismatch (expected %d, got %d) at %s:%d line1=%s line2=%s",
                         desired_model, detected_model,
                         ctrl->conf ? ctrl->conf->host : "(null)",
                         probe_port,
                         line1 ? line1 : "(none)",
                         line2 ? line2 : "(none)");
            sat_log_log(SAT_LOG_LEVEL_WARN,
                        _("%s: rotctld model mismatch expected=%d got=%d line1=%s line2=%s"),
                        __func__, desired_model, detected_model,
                        line1 ? line1 : "(none)",
                        line2 ? line2 : "(none)");
            if (state->spawned && ctrl->rotctld_mgr)
                rotctld_process_stop_async(ctrl, "probe_model_mismatch");
            g_free(first_line);
            g_free(full_text);
            g_free(line1);
            g_free(line2);
            rotctld_probe_state_detach(state);
            rotctld_fail_engage(ctrl, TRUE);
            rotctld_probe_state_unref(state);
            return G_SOURCE_REMOVE;
        }

        {
            gint validate_timeout_ms = ROTCTLD_AUTODETECT_VALIDATE_TIMEOUT_MS;
            const gchar *device_hint = NULL;

            if (ctrl->conf != NULL)
            {
                if (ctrl->conf->last_good_device &&
                    *ctrl->conf->last_good_device)
                    device_hint = ctrl->conf->last_good_device;
                else if (ctrl->conf->device_manual &&
                         *ctrl->conf->device_manual)
                    device_hint = ctrl->conf->device_manual;
                else
                    device_hint = ctrl->conf->device;
            }

            if (validate_timeout_ms < ROTCTLD_AUTODETECT_LASTGOOD_WINDOW_MS)
                validate_timeout_ms = ROTCTLD_AUTODETECT_LASTGOOD_WINDOW_MS;
            if (validate_timeout_ms < 2500)
                validate_timeout_ms = 2500;
            if (validate_timeout_ms <
                rotctld_validate_timeout_floor_ms(ctrl, device_hint))
                validate_timeout_ms =
                    rotctld_validate_timeout_floor_ms(ctrl, device_hint);
            if (validate_timeout_ms > 8000)
                validate_timeout_ms = 8000;

            pos_res = rotctld_probe_position(ctrl->conf->host,
                                             probe_port,
                                             validate_timeout_ms,
                                             ROTCTLD_AUTODETECT_VALIDATE_RETRIES,
                                             ROTCTLD_AUTODETECT_VALIDATE_RETRY_DELAY_MS,
                                             &rprt,
                                             &pos_reason);
        }
        pos_ok = (pos_res == ROTCTLD_POS_OK);
        if (!pos_ok)
        {
            rot_term_log(ctrl, "gpredict:err",
                         "rotctld probe get_position failed host=%s port=%d (%s rprt=%d)",
                         ctrl->conf ? ctrl->conf->host : "(null)",
                         probe_port,
                         pos_reason ? pos_reason : "error",
                         rprt);
            sat_log_log(SAT_LOG_LEVEL_WARN,
                        "rotctld probe get_position failed host=%s port=%d res=%d rprt=%d reason=%s",
                        ctrl->conf ? ctrl->conf->host : "(null)",
                        probe_port,
                        pos_res,
                        rprt,
                        pos_reason ? pos_reason : "error");

            if (pos_res == ROTCTLD_POS_RPRT_ERR ||
                pos_res == ROTCTLD_POS_PARSE_FAIL)
            {
                if (pos_reason && *pos_reason)
                {
                    gchar *detail = g_strdup_printf("get_position failed: %s", pos_reason);
                    rot_schedule_cmd_reject(ctrl, detail);
                    g_free(detail);
                }
                else
                {
                    rot_schedule_cmd_reject(ctrl, NULL);
                }
                if (state->spawned && ctrl->rotctld_mgr)
                    rotctld_process_stop_async(ctrl, "probe_pos_reject");

                g_free(pos_reason);
                g_free(first_line);
                g_free(full_text);
                g_free(line1);
                g_free(line2);
                rotctld_probe_state_detach(state);
                rotctld_fail_engage(ctrl, TRUE);
                rotctld_probe_state_unref(state);
                return G_SOURCE_REMOVE;
            }

            result = ROTCTLD_PROBE_NOT_READY;
        }

        g_free(pos_reason);
        if (pos_ok)
        {
            g_free(first_line);
            g_free(full_text);
            g_free(line1);
            g_free(line2);
            rotctld_probe_state_detach(state);
            rotctld_finish_engage(ctrl);
            rotctld_probe_state_unref(state);
            return G_SOURCE_REMOVE;
        }

        g_free(line1);
        g_free(line2);
    }

    if (result == ROTCTLD_PROBE_NOT_ROTCTLD)
        state->non_rotctld_count++;
    else
        state->non_rotctld_count = 0;

    if (ctrl->rotctld_mgr && !rotctld_mgr_is_running(ctrl->rotctld_mgr))
    {
        rotctld_log_exit(ctrl, state, "probe_exit");

        g_free(first_line);
        g_free(full_text);
        rotctld_probe_state_detach(state);
        rotctld_fail_engage(ctrl, TRUE);
        rotctld_probe_state_unref(state);
        return G_SOURCE_REMOVE;
    }

    if (result == ROTCTLD_PROBE_NOT_ROTCTLD &&
        state->non_rotctld_count >= 3)
    {
        rot_term_log(ctrl, "gpredict:err",
                     "Port %d already in use by non-rotctld service (host=%s, probe=%s)",
                     probe_port,
                     ctrl->conf ? ctrl->conf->host : "(null)",
                     rotctld_probe_result_name(result));
        rot_schedule_wrong_daemon(ctrl,
                                  ctrl->conf ? ctrl->conf->host : NULL,
                                  probe_port);
        if (state->spawned)
            rotctld_process_stop_async(ctrl, "probe_non_rotctld");
        g_free(first_line);
        g_free(full_text);
        rotctld_probe_state_detach(state);
        rotctld_fail_engage(ctrl, TRUE);
        rotctld_probe_state_unref(state);
        return G_SOURCE_REMOVE;
    }

    if (now_us >= state->deadline_us)
    {
        if (state->spawned)
        {
            rot_term_log(ctrl, "gpredict:err",
                         "rotctld did not bind port host=%s port=%d argv=%s",
                         ctrl->conf ? ctrl->conf->host : "(null)",
                         probe_port,
                         state->spawn_summary ? state->spawn_summary : "unknown");
        }

        rot_term_log(ctrl, "gpredict:err",
                     "rotctld not reachable at %s:%d (spawn: %s)",
                     ctrl->conf ? ctrl->conf->host : "(null)",
                     probe_port,
                     state->spawn_summary ? state->spawn_summary : "unknown");
        if (state->spawned)
            rotctld_process_stop_async(ctrl, "probe_tcp_deadline");
        g_free(first_line);
        g_free(full_text);
        rotctld_probe_state_detach(state);
        rotctld_fail_engage(ctrl, FALSE);
        rotctld_probe_state_unref(state);
        return G_SOURCE_REMOVE;
    }

    if (result == ROTCTLD_PROBE_NOT_READY && !state->autostart_allowed)
    {
        if (!rot_host_is_local(ctrl->conf->host))
        {
            rot_term_log(ctrl, "gpredict:err",
                         "auto-start only supported for local host (host=%s)",
                         ctrl->conf->host ? ctrl->conf->host : "(missing)");
        }
        else if (!ctrl->conf->autostart)
        {
            rot_term_log(ctrl, "gpredict:err",
                         "rotctld autostart is disabled for %s",
                         ctrl->conf->name ? ctrl->conf->name : "(unnamed)");
        }

        g_free(first_line);
        g_free(full_text);
        rotctld_probe_state_detach(state);
        rotctld_fail_engage(ctrl, TRUE);
        rotctld_probe_state_unref(state);
        return G_SOURCE_REMOVE;
    }

    if (result == ROTCTLD_PROBE_NOT_READY &&
        state->autostart_allowed && !state->spawn_attempted)
    {
        if (state->autodetect_enabled)
        {
            state->autodetect_state = ROTCTLD_AUTODETECT_IDLE;
            autodetect_step = rotctld_autodetect_step(ctrl, state, now_us);
            g_free(first_line);
            g_free(full_text);
            if (autodetect_step == ROTCTLD_AUTODETECT_STEP_SUCCESS)
            {
                rotctld_autodetect_apply_success(ctrl, state);
                rotctld_probe_state_detach(state);
                rotctld_finish_engage(ctrl);
                rotctld_probe_state_unref(state);
                return G_SOURCE_REMOVE;
            }
            if (autodetect_step == ROTCTLD_AUTODETECT_STEP_FAIL)
            {
                rotctld_probe_state_detach(state);
                rotctld_fail_engage(ctrl, TRUE);
                rotctld_probe_state_unref(state);
                return G_SOURCE_REMOVE;
            }

            state->delay_ms = ROTCTLD_AUTODETECT_TICK_MS;
            ctrl->rotctld_probe_id =
                g_timeout_add_full(G_PRIORITY_DEFAULT,
                                   state->delay_ms,
                                   rotctld_probe_retry_cb,
                                   state,
                                   NULL);
            return G_SOURCE_REMOVE;
        }

        g_free(state->spawn_summary);
        state->spawn_summary = NULL;
        state->spawn_attempted = TRUE;
        state->spawned =
            rotctld_spawn_autostart(ctrl, NULL, 0, 0, &state->spawn_summary);
        if (!state->spawned)
        {
            g_free(first_line);
            g_free(full_text);
            rotctld_probe_state_detach(state);
            rotctld_fail_engage(ctrl, FALSE);
            rotctld_probe_state_unref(state);
            return G_SOURCE_REMOVE;
        }
    }

    if (state->delay_ms == 0)
        state->delay_ms = 50;
    else if (state->delay_ms < 500)
        state->delay_ms = MIN(state->delay_ms * 2, 500);

    ctrl->rotctld_probe_id =
        g_timeout_add_full(G_PRIORITY_DEFAULT,
                           state->delay_ms,
                           rotctld_probe_retry_cb,
                           state,
                           NULL);

    g_free(first_line);
    g_free(full_text);
    return G_SOURCE_REMOVE;
}

/* Probe rotctld identity using \\dump_state with strict validation. */
static rotctld_probe_result_t
rotctld_probe_identity(const gchar *host, gint port, gint timeout_ms,
                       gchar **first_line_out, gchar **full_text_out)
{
    const gchar *cmd = "\\dump_state\n";
    rotctld_probe_result_t result = ROTCTLD_PROBE_NOT_READY;
    gchar *best_first = NULL;
    gchar *best_full = NULL;
    gboolean resolved = FALSE;
    GResolver *resolver = NULL;
    GList *addrs = NULL;
    GError *error = NULL;
    gint64 timeout_us;

    if (first_line_out)
        *first_line_out = NULL;
    if (full_text_out)
        *full_text_out = NULL;

    if (host == NULL || *host == '\0' || port <= 0)
        return ROTCTLD_PROBE_NOT_READY;

    if (timeout_ms <= 0)
        timeout_ms = 100;

    timeout_us = (gint64) timeout_ms * 1000;

    if (rotctld_mgr_host_is_local(host))
    {
        GInetAddress *ipv4 = g_inet_address_new_from_string("127.0.0.1");
        GInetAddress *ipv6 = NULL;

        if (g_ascii_strcasecmp(host, "127.0.0.1") != 0)
            ipv6 = g_inet_address_new_from_string("::1");

        if (ipv4)
            addrs = g_list_append(addrs, ipv4);
        if (ipv6)
            addrs = g_list_append(addrs, ipv6);
    }
    else
    {
        resolver = g_resolver_get_default();
        addrs = g_resolver_lookup_by_name(resolver, host, NULL, &error);
        if (addrs == NULL)
        {
            g_clear_error(&error);
            g_object_unref(resolver);
            return ROTCTLD_PROBE_NOT_READY;
        }
        resolved = TRUE;
    }

    if (addrs == NULL)
    {
        if (resolver)
            g_object_unref(resolver);
        return ROTCTLD_PROBE_NOT_READY;
    }

    for (GList *iter = addrs; iter != NULL; iter = iter->next)
    {
        GInetAddress *addr = G_INET_ADDRESS(iter->data);
        GSocket *sock = NULL;
        GSocketAddress *sockaddr = NULL;
        gboolean connected = FALSE;
        gssize size = 0;

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
            connected = TRUE;
        }
        else if (g_error_matches(error, G_IO_ERROR, G_IO_ERROR_PENDING))
        {
            g_clear_error(&error);
            if (g_socket_condition_timed_wait(sock, G_IO_OUT, timeout_us, NULL,
                                              &error))
            {
                if (g_socket_check_connect_result(sock, &error))
                    connected = TRUE;
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

        if (connected)
        {
            size = g_socket_send(sock, cmd, strlen(cmd), NULL, &error);
            if (size < 0 &&
                g_error_matches(error, G_IO_ERROR, G_IO_ERROR_WOULD_BLOCK))
            {
                g_clear_error(&error);
                if (g_socket_condition_timed_wait(sock, G_IO_OUT, timeout_us,
                                                  NULL, &error))
                {
                    size = g_socket_send(sock, cmd, strlen(cmd), NULL, &error);
                }
            }
            if (size < 0)
                g_clear_error(&error);

            if (size >= 0)
            {
                GString *response = g_string_new(NULL);
                gint64 deadline_us =
                    g_get_monotonic_time() + ((gint64) timeout_ms * 1000);

                while (g_get_monotonic_time() < deadline_us)
                {
                    gint64 remaining_us =
                        deadline_us - g_get_monotonic_time();
                    gchar buffer[512];

                    if (remaining_us <= 0)
                        break;

                    if (!g_socket_condition_timed_wait(sock, G_IO_IN,
                                                       remaining_us, NULL,
                                                       &error))
                    {
                        g_clear_error(&error);
                        break;
                    }

                    size = g_socket_receive(sock, buffer, sizeof(buffer) - 1,
                                            NULL, &error);
                    if (size <= 0)
                    {
                        g_clear_error(&error);
                        break;
                    }

                    buffer[size] = '\0';
                    g_string_append_len(response, buffer, size);
                }

                if (response->len > 0)
                {
                    gchar *probe_first = NULL;
                    gboolean valid =
                        rotctld_response_is_valid(response->str,
                                                  &probe_first);

                    if (valid)
                    {
                        result = ROTCTLD_PROBE_OK_ROTCTLD;
                        g_free(best_first);
                        g_free(best_full);
                        best_first = probe_first;
                        best_full = g_strdup(response->str);
                        g_string_free(response, TRUE);
                        g_object_unref(sockaddr);
                        g_object_unref(sock);
                        break;
                    }

                    if (result != ROTCTLD_PROBE_OK_ROTCTLD)
                        result = ROTCTLD_PROBE_NOT_ROTCTLD;

                    if (best_full == NULL)
                    {
                        best_first = probe_first;
                        best_full = g_strdup(response->str);
                    }
                    else
                    {
                        g_free(probe_first);
                    }
                }

                g_string_free(response, TRUE);
            }
        }

        if (sockaddr)
            g_object_unref(sockaddr);
        if (sock)
            g_object_unref(sock);
    }

    if (resolved)
    {
        g_resolver_free_addresses(addrs);
        g_object_unref(resolver);
    }
    else
    {
        g_list_free_full(addrs, g_object_unref);
    }

    if (first_line_out)
        *first_line_out = best_first;
    else
        g_free(best_first);

    if (full_text_out)
        *full_text_out = best_full;
    else
        g_free(best_full);

    return result;
}

static gboolean rot_host_is_local(const gchar *host)
{
    return rotctld_mgr_host_is_local(host);
}

static gint rotctld_effective_port(GtkRotCtrl *ctrl)
{
    if (ctrl == NULL)
        return 0;
    if (ctrl->selected_child_port > 0)
        return ctrl->selected_child_port;
    return (ctrl->conf != NULL) ? ctrl->conf->port : 0;
}

/**
 * Ensure rotctld is running.
 *
 * Starts an asynchronous probe/retry loop that validates \\dump_state
 * output before declaring rotctld ready. The loop is responsible for
 * spawning rotctld (when autostart is enabled) and for retry/backoff.
 */
static rotctld_ensure_result_t rotctld_ensure_running(GtkRotCtrl *ctrl)
{
    RotctldProbeState *state = NULL;

    if (ctrl == NULL || ctrl->conf == NULL)
        return ROTCTLD_ENSURE_FAILED;

    if (ctrl->client.thread != NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "rotctld ensure: client thread already running; skipping probe");
        return ROTCTLD_ENSURE_READY;
    }

    {
        gint model = rot_conf_hamlib_model(ctrl->conf);
        gint baud = ctrl->conf->baud > 0 ? ctrl->conf->baud
                                         : rot_protocol_default_baud(
                                             ctrl->conf->protocol);
        const gchar *protocol_name = rot_protocol_name(ctrl->conf->protocol);
        if (protocol_name == NULL)
            protocol_name = "(unknown)";
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "rotctld ensure: host=%s port=%d autostart=%d "
                    "protocol=%d/%s model=%d baud=%d device=%s device_autopick=%d",
                    ctrl->conf->host ? ctrl->conf->host : "(null)",
                    ctrl->conf->port,
                    ctrl->conf->autostart ? 1 : 0,
                    ctrl->conf->protocol,
                    protocol_name,
                    model,
                    baud,
                    ctrl->conf->device ? ctrl->conf->device : "(none)",
                    ctrl->conf->device_autopick ? 1 : 0);
    }

    if (!rot_protocol_is_valid(ctrl->conf->protocol))
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: invalid rotator protocol %d"),
                    __func__, ctrl->conf->protocol);
        rot_term_log(ctrl, "gpredict:err",
                     "Invalid rotator protocol; cannot start rotctld.");
        return ROTCTLD_ENSURE_FAILED;
    }

    if (ctrl->rotctld_mgr != NULL)
        rotctld_process_stop_async(ctrl, "probe_restart");

    if (ctrl->rotctld_probe_state)
        return ROTCTLD_ENSURE_PENDING;

    state = g_new0(RotctldProbeState, 1);
    state->ctrl = g_object_ref(ctrl);
    state->generation = rotctld_get_engage_generation(ctrl);
    state->deadline_us = g_get_monotonic_time() + (5 * G_TIME_SPAN_SECOND);
    state->attempt = 0;
    state->non_rotctld_count = 0;
    state->delay_ms = 50;
    state->ref_count = 1;
    state->cancelled = FALSE;
    if (ctrl->conf->host && *ctrl->conf->host &&
        !rotctld_mgr_host_is_local(ctrl->conf->host))
    {
        rot_term_log(ctrl, "gpredict:err",
                     "rotor probe requires local host for serial scan (host=%s)",
                     ctrl->conf->host);
        rot_show_conf_error(ctrl, _("Rotor probe requires a local host."));
        rotctld_probe_state_unref(state);
        return ROTCTLD_ENSURE_FAILED;
    }

    state->autodetect_enabled = TRUE;
    state->autodetect_state = ROTCTLD_AUTODETECT_IDLE;
    state->autodetect_generation = 0;
    state->autodetect_state_since_us = 0;
    state->autodetect_tcp_deadline_us = 0;
    state->autodetect_port_free_deadline_us = 0;
    state->autodetect_settle_until_us = 0;
    state->validate_inflight = FALSE;
    state->validate_done = FALSE;
    state->validate_result = ROTCTLD_POS_IO_ERR;
    state->validate_score = 0;
    state->validate_rprt = 0;
    state->validate_reason = NULL;
    state->validate_dump_ok = FALSE;
    state->validate_pos_ok = FALSE;
    state->validate_timeout = FALSE;
    state->validate_timeout_ms = ROTCTLD_AUTODETECT_VALIDATE_TIMEOUT_MS;
    state->validate_retries = ROTCTLD_AUTODETECT_VALIDATE_RETRIES;
    state->validate_retry_delay_ms = ROTCTLD_AUTODETECT_VALIDATE_RETRY_DELAY_MS;
    state->best_score = G_MININT;
    state->best_device = NULL;
    state->best_baud = 0;
    g_mutex_init(&state->validate_mutex);
    state->autostart_allowed =
        rot_host_is_local(ctrl->conf->host) && ctrl->conf->autostart;
    state->spawn_attempted = FALSE;
    state->spawned = FALSE;
    state->scanning_child_pid = -1;
    state->scanning_child_validated = FALSE;
    state->last_good_exclusive = FALSE;
    state->last_good_deadline_us = 0;
    state->last_good_device = NULL;
    state->last_good_baud = 0;
    state->autodetect_start_us = 0;
    state->autodetect_candidate_start_us = 0;
    state->autodetect_list_full = NULL;
    state->autodetect_limited = FALSE;
    state->autodetect_port = 0;
    state->autodetect_attempts = NULL;
    state->spawn_summary = NULL;

    ctrl->rotctld_probe_state = state;
    ctrl->rotctld_probe_id =
        g_timeout_add_full(G_PRIORITY_DEFAULT, 0,
                           rotctld_probe_retry_cb, state, NULL);

    return ROTCTLD_ENSURE_PENDING;
}

typedef struct {
    GtkRotCtrl *ctrl;
    gchar      *host;
    gint        port;
} RotWrongDaemonInfo;

static void rot_show_message(GtkRotCtrl *ctrl,
                             GtkMessageType type,
                             const gchar *title,
                             const gchar *message)
{
    GtkWidget *toplevel;
    GtkWindow *parent = NULL;
    GtkWidget *dialog;

    if (ctrl == NULL)
        return;

    toplevel = gtk_widget_get_toplevel(GTK_WIDGET(ctrl));
    if (GTK_IS_WINDOW(toplevel))
        parent = GTK_WINDOW(toplevel);

    dialog = gtk_message_dialog_new(parent,
                                    GTK_DIALOG_DESTROY_WITH_PARENT,
                                    type,
                                    GTK_BUTTONS_OK,
                                    "%s",
                                    message ? message : "");
    if (title && *title)
        gtk_window_set_title(GTK_WINDOW(dialog), title);

    g_signal_connect_swapped(dialog, "response",
                             G_CALLBACK(gtk_widget_destroy), dialog);
    gtk_widget_show(dialog);
}

static gboolean rot_wrong_daemon_idle(gpointer data)
{
    RotWrongDaemonInfo *info = data;

    if (info == NULL)
        return G_SOURCE_REMOVE;

    rot_show_message(info->ctrl,
                     GTK_MESSAGE_ERROR,
                     _("Rotor error"),
                     _("Port already in use by a non-rotctld service."));

    if (info->ctrl)
    {
        RotorStateSnapshot snap = { 0 };
        rotctrl_set_ui_hard_error(info->ctrl, "port in use by non-rotctld");
        rotctrl_set_ui_status_detail(info->ctrl, "ERROR: rotctld");
        snap.control_active = TRUE;
        snap.hard_error = TRUE;
        rotctrl_apply_ui_status(info->ctrl, &snap, "wrong daemon");
    }

    g_free(info->host);
    g_free(info);

    return G_SOURCE_REMOVE;
}

static void rot_schedule_wrong_daemon(GtkRotCtrl *ctrl,
                                      const gchar *host, gint port)
{
    RotWrongDaemonInfo *info = g_new0(RotWrongDaemonInfo, 1);

    info->ctrl = ctrl;
    info->host = g_strdup(host);
    info->port = port;

    g_idle_add(rot_wrong_daemon_idle, info);
}

typedef struct {
    GtkRotCtrl     *ctrl;
    gchar          *reason;
} RotCmdRejectInfo;

static void rot_show_cmd_reject_error(GtkRotCtrl *ctrl, const gchar *reason)
{
    const gchar *msg = reason ? reason
                              : _("rotctld reachable but rejects position commands; "
                                  "check backend/model, limits, and axis mode");

    rot_show_message(ctrl,
                     GTK_MESSAGE_ERROR,
                     _("Rotor command rejected"),
                     msg);
}

static gboolean rot_cmd_reject_idle(gpointer data)
{
    RotCmdRejectInfo *info = data;

    if (info == NULL)
        return G_SOURCE_REMOVE;

    if (info->ctrl)
        rot_show_cmd_reject_error(info->ctrl, info->reason);

    if (info->ctrl)
    {
        RotorStateSnapshot snap = { 0 };
        rotctrl_set_ui_hard_error(info->ctrl, "rotctld command rejected");
        rotctrl_set_ui_status_detail(info->ctrl, "ERROR: rotctld");
        snap.control_active = TRUE;
        snap.hard_error = TRUE;
        rotctrl_apply_ui_status(info->ctrl, &snap, "command rejected");
    }

    g_free(info->reason);
    g_free(info);
    return G_SOURCE_REMOVE;
}

static void G_GNUC_UNUSED rot_schedule_cmd_reject(GtkRotCtrl *ctrl,
                                                  const gchar *reason)
{
    RotCmdRejectInfo *info = g_new0(RotCmdRejectInfo, 1);

    info->ctrl = ctrl;
    info->reason = reason ? g_strdup(reason) : NULL;
    g_idle_add(rot_cmd_reject_idle, info);
}

/**
 * Show an error dialog indicating no rotor/rotctld could be found.
 *
 * \param ctrl Pointer to the GtkRotCtrl widget.
 */
static void rot_show_no_rotor_dialog(GtkRotCtrl *ctrl)
{
    rot_show_message(ctrl,
                     GTK_MESSAGE_ERROR,
                     _("Rotor error"),
                     _("Unable to find a rotor!"));
}

static void rot_show_conf_error(GtkRotCtrl *ctrl, const gchar *reason)
{
    const gchar *msg = reason ? reason : _("Invalid rotor configuration.");

    rot_show_message(ctrl,
                     GTK_MESSAGE_ERROR,
                     _("Rotor configuration error"),
                     msg);
}

static void rot_show_plan_error(GtkRotCtrl *ctrl, const gchar *reason)
{
    const gchar *msg = reason ? reason
                              : _("No valid rotor trajectory for this pass.");
    rot_show_message(ctrl,
                     GTK_MESSAGE_ERROR,
                     _("Rotor trajectory blocked"),
                     msg);
}

static void rot_logs_toggle_cb(GtkToggleButton *button, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);
    gboolean visible;

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    if (ctrl == NULL || ctrl->term_view == NULL)
        return;

    visible = gtk_toggle_button_get_active(button);
    if (visible)
        rotctrl_capture_log_closed_height(ctrl);

    gp_term_view_set_visible(ctrl->term_view, visible);

    if (visible)
        rotctrl_schedule_resize(ctrl);
    else
        rotctrl_restore_log_height(ctrl);
}

static void rotctrl_force_toplevel_resize(GtkRotCtrl *ctrl)
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

static void rotctrl_capture_log_closed_height(GtkRotCtrl *ctrl)
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

static void rotctrl_restore_log_height(GtkRotCtrl *ctrl)
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

static gboolean rotctrl_resize_idle(gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);

    if (ctrl == NULL)
        return G_SOURCE_REMOVE;

    ctrl->resize_idle_id = 0;
    rotctrl_force_toplevel_resize(ctrl);
    return G_SOURCE_REMOVE;
}

static void rotctrl_schedule_resize(GtkRotCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    if (ctrl->resize_idle_id != 0)
        return;

    ctrl->resize_idle_id =
        g_idle_add_full(G_PRIORITY_LOW, rotctrl_resize_idle, ctrl, NULL);
}

static void rot_verbose_cb(GtkToggleButton *button, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    ctrl->verbose_logging = gtk_toggle_button_get_active(button);
    rot_term_log(ctrl, "gpredict:rx",
                 "verbose logging %s",
                 ctrl->verbose_logging ? "enabled" : "disabled");
}

static void rotctrl_reset_lock_button_visual(GtkRotCtrl *ctrl)
{
    if (ctrl == NULL || ctrl->LockBut == NULL)
        return;

    gtk_toggle_button_set_inconsistent(GTK_TOGGLE_BUTTON(ctrl->LockBut), FALSE);
    gtk_widget_unset_state_flags(ctrl->LockBut,
                                 GTK_STATE_FLAG_ACTIVE |
                                 GTK_STATE_FLAG_PRELIGHT |
                                 GTK_STATE_FLAG_FOCUSED |
                                 GTK_STATE_FLAG_SELECTED);
}

/**
 * Rotor locked.
 *
 * \param button Pointer to the "Engage" button.
 * \param data Pointer to the GtkRotCtrl widget.
 * 
 * This function is called when the user toggles the "Engage" button.
 */
static void rot_locked_cb(GtkToggleButton * button, gpointer data)
{
    GtkRotCtrl     *ctrl = GTK_ROT_CTRL(data);

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    if (!gtk_toggle_button_get_active(button))
    {
        gboolean will_send_quit = FALSE;
        const gchar *reason = NULL;

        if (ctrl->cal_hold_active)
            rotctrl_set_cal_hold(ctrl, FALSE, "disengage");

        ctrl->engaged = FALSE;
        ctrl->engage_pending = FALSE;
        ctrl->tracking_active = FALSE;
        ctrl->tracking_session_los = 0.0;
        ctrl->have_user_command = FALSE;
        ctrl->ok_streak = 0;
        ctrl->bad_streak = 0;
        ctrl->timeout_streak = 0;
        ctrl->comm_status = ROT_COMM_DISENGAGED;
        ctrl->target_state = ROT_TARGET_STATE_IDLE;
        ctrl->target_state_since_us = 0;
        ctrl->target_valid_since_us = 0;
        ctrl->target_invalid_since_us = 0;
        ctrl->locked_lane_valid = FALSE;
        ctrl->pos_stale_active = FALSE;
        ctrl->pos_stale_hyst_active = FALSE;
        ctrl->pos_stale_ready_hits = 0;
        g_mutex_lock(&ctrl->client.mutex);
        ctrl->engage_generation++;
        g_atomic_int_set(&ctrl->client.stop_requested, 1);
        g_mutex_unlock(&ctrl->client.mutex);
        rotctld_probe_cancel(ctrl);
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "rotctld disengage: cancel probe, request thread stop");
        rot_term_log(ctrl, "gpredict:rx",
                     "rotctld disengage: cancel probe, request thread stop");
        gtk_widget_set_sensitive(ctrl->DevSel, TRUE);
        will_send_quit = ctrl->client.thread != NULL || ctrl->client.running;
        reason = "user disengage";
        rot_session_set_state(ctrl, ROT_SESSION_DISCONNECTED,
                              reason, will_send_quit);
        rotctrl_clear_ui_hard_error(ctrl);
        rotctrl_set_ui_status_detail(ctrl, NULL);

        if (!ctrl->client.running && ctrl->client.thread == NULL)
        {
            /* client thread is not running; nothing to do */
            RotorStateSnapshot snap = { 0 };
            if (ctrl->rotctld_mgr)
                rotctld_process_stop_async(ctrl, "user_disengage");
            snap.control_active = FALSE;
            rotctrl_apply_ui_status(ctrl, &snap, "user disengage");
            return;
        }

        rotctld_request_thread_stop(ctrl, will_send_quit);
        if (ctrl->client.thread)
        {
            (void)rotctld_collect_client_thread(ctrl, FALSE, "disengage");
        }
        {
            RotorStateSnapshot snap = { 0 };
            snap.control_active = FALSE;
            rotctrl_apply_ui_status(ctrl, &snap, "user disengage");
        }
        ctrl->hold_position_on_engage = FALSE;
        ctrl->hold_position_log_emitted = FALSE;
        rotctrl_reset_lock_button_visual(ctrl);
    }
    else
    {
        if (ctrl->engaged || ctrl->engage_pending)
            return;

        /* Apply UI settings before starting any worker activity. */
        if (!rotor_apply_ui_settings(ctrl, TRUE))
        {
            rotctrl_ui_begin_update(ctrl, "engage_invalid_settings");
            gtk_toggle_button_set_active(button, FALSE);
            rotctrl_ui_end_update(ctrl, "engage_invalid_settings");
            return;
        }

        {
            gchar *conf_err = NULL;
            if (!rot_conf_validate(ctrl->conf, &conf_err)) {
                RotorStateSnapshot snap = { 0 };
                sat_log_log(SAT_LOG_LEVEL_ERROR,
                            _("%s: Controller does not have a valid configuration: %s"),
                            __func__, conf_err ? conf_err : "unknown error");
                rot_show_conf_error(ctrl, conf_err);
                gtk_toggle_button_set_active(button, FALSE);
                ctrl->engaged = FALSE;
                rotctrl_set_ui_hard_error(ctrl,
                                          conf_err ? conf_err :
                                          "invalid rotor configuration");
                rotctrl_set_ui_status_detail(ctrl, "ERROR: rotctld");
                snap.control_active = TRUE;
                snap.hard_error = TRUE;
                rotctrl_apply_ui_status(ctrl, &snap,
                                        "config validation failed");
                g_free(conf_err);
                return;
            }
        }

        /* Automatically disable tracking when engaging so the rotor does not
         * immediately jump to the current satellite target before calibration
         * or manual positioning.
         */
        if (ctrl->tracking) {
            gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->track), FALSE);
            ctrl->tracking = FALSE;
        }
        ctrl->target_state = ROT_TARGET_STATE_IDLE;
        ctrl->target_state_since_us = 0;
        ctrl->target_valid_since_us = 0;
        ctrl->target_invalid_since_us = 0;

        /* ensure we are not in monitor mode when engaging by default */
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->MonitorCheckBox), FALSE);
        ctrl->monitor = FALSE;
        ctrl->hold_position_on_engage = TRUE;
        ctrl->hold_position_log_emitted = FALSE;
        ctrl->have_user_command = FALSE;
        rotctrl_clear_ui_hard_error(ctrl);
        rotctrl_set_ui_status_detail(ctrl, NULL);

        {
            ctrl->engage_pending = TRUE;
            rot_session_set_state(ctrl, ROT_SESSION_CONNECTING,
                                  "engage requested", FALSE);
            {
                RotorStateSnapshot snap = { 0 };
                rotctrl_set_ui_status_detail(ctrl, "ENGAGING");
                snap.control_active = TRUE;
                snap.engaging = TRUE;
                snap.hard_error = ctrl->ui_hard_error;
                rotctrl_apply_ui_status(ctrl, &snap, "engage requested");
            }
            rotctld_probe_cancel(ctrl);
            g_mutex_lock(&ctrl->client.mutex);
            ctrl->engage_generation++;
            ctrl->client.thread_done = FALSE;
            ctrl->client.running = FALSE;
            ctrl->client.io_error = FALSE;
            ctrl->client.daemon_ok = FALSE;
            ctrl->client.socket = -1;
            ctrl->client.conn_id = 0;
            ctrl->client.reconnect_failures = 0;
            ctrl->client.reconnect_degraded = FALSE;
            ctrl->client.io_error_reason[0] = '\0';
            ctrl->client.transport_backoff_until_us = 0;
            ctrl->client.handshake_pos_ok = FALSE;
            ctrl->client.first_pos_deadline_us = 0;
            ctrl->client.pos_valid = FALSE;
            ctrl->client.pos_invalid = FALSE;
            ctrl->client.pos_unknown = FALSE;
            ctrl->client.pos_cmd_ok = FALSE;
            ctrl->client.set_pos_ok = FALSE;
            ctrl->client.pos_failures = 0;
            ctrl->client.pos_backoff_until_us = 0;
            ctrl->client.pos_backoff_sec = 0.5;
            ctrl->client.pos_degraded = FALSE;
            ctrl->client.send_quit = FALSE;
            ctrl->client.allow_send_no_pos = FALSE;
            ctrl->client.apply_calib = FALSE;
            ctrl->client.stop_pending = FALSE;
            g_atomic_int_set(&ctrl->client.stop_requested, 0);
            g_mutex_unlock(&ctrl->client.mutex);
            rotctld_ensure_result_t ensure = rotctld_ensure_running(ctrl);
            if (ensure == ROTCTLD_ENSURE_FAILED)
            {
                sat_log_log(SAT_LOG_LEVEL_ERROR,
                            _("%s: failed to connect to or start rotctld"),
                            __func__);
                rotctld_fail_engage(ctrl, FALSE);
                return;
            }
            if (ensure == ROTCTLD_ENSURE_PENDING)
            {
                RotorStateSnapshot snap = { 0 };
                gtk_widget_set_sensitive(ctrl->DevSel, FALSE);
                rotctrl_set_ui_status_detail(ctrl, "ENGAGING");
                snap.control_active = TRUE;
                snap.engaging = TRUE;
                snap.hard_error = ctrl->ui_hard_error;
                rotctrl_apply_ui_status(ctrl, &snap,
                                        "rotctld ensure pending");
                return;
            }
        }

        rotctld_finish_engage(ctrl);
    }
}


/**
 * Manage satellite selections
 *
 * \param satsel Pointer to the GtkComboBox.
 * \param data Pointer to the GtkRotCtrl widget.
 * 
 * This function is called when the user selects a new satellite.
 */
static gboolean rotctrl_apply_sat_selection(GtkRotCtrl *ctrl,
                                            gint index,
                                            const gchar *source)
{
    sat_t *selected = NULL;
    gdouble target_plot_az = -10.0;
    gdouble target_plot_el = -10.0;

    if (ctrl == NULL)
        return FALSE;

    selected = SAT(g_slist_nth_data(ctrl->sats, index));
    if (selected == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%s: Invalid satellite selection: %d"),
                    __FILE__, __func__, index);

        ctrl->target = NULL;
        if (ctrl->pass != NULL)
        {
            free_pass(ctrl->pass);
            ctrl->pass = NULL;
        }
        rotctrl_set_pass_on_plots(ctrl, NULL);
        rotctrl_set_target_pos_on_plots(ctrl, -10.0, -10.0);
        rotctrl_queue_draw_plots(ctrl);
        return FALSE;
    }

    ctrl->target = selected;
    (void)rotctrl_sync_sat_copy_from_module(ctrl, ctrl->target);
    rot_plan_reset(&ctrl->trajectory_plan);
    rotctrl_tracking_policy_reset_reason(ctrl, "target_change");
    ctrl->pretrack_target_valid = FALSE;
    ctrl->pretrack_last_update_us = 0;
    ctrl->pretrack_aos_time = 0.0;
    ctrl->pretrack_wait_log_us = 0;
    ctrl->pretrack_wrap_valid = FALSE;
    ctrl->pretrack_wrap_user_az = 0.0;
    ctrl->pretrack_wrap_raw_az = 0.0;
    ctrl->pretrack_wrap_k = 0;
    ctrl->seam_valid = FALSE;
    ctrl->seam_crossing_active = FALSE;
    ctrl->seam_crossing_sent = FALSE;
    ctrl->seam_crossing_lane_valid = FALSE;
    ctrl->seam_crossing_lane_k = 0;
    ctrl->seam_crossing_target_az360 = 0.0;
    ctrl->seam_crossing_since_us = 0;
    ctrl->wrap_acquire_active = FALSE;
    ctrl->wrap_acquire_sent = FALSE;
    ctrl->wrap_acquire_target_backend = 0.0;
    ctrl->wrap_acquire_target_az360 = 0.0;
    ctrl->wrap_acquire_target_k = 0;
    ctrl->wrap_acquire_since_us = 0;
    ctrl->wrap_acquire_last_log_us = 0;
    ctrl->pending_lane_valid = FALSE;
    ctrl->pending_lane_since_us = 0;
    ctrl->locked_lane_valid = FALSE;
    ctrl->last_target_valid = FALSE;

    if (ctrl->pass != NULL)
    {
        free_pass(ctrl->pass);
        ctrl->pass = NULL;
    }

    ctrl->pass = get_pass_no_min_el(ctrl->target, ctrl->qth, ctrl->t, 3.0);

    set_flipped_pass(ctrl);
    rotctrl_set_pass_on_plots(ctrl, ctrl->pass);

    if (isfinite(ctrl->target->az) && isfinite(ctrl->target->el) &&
        ctrl->target->el >= 0.0)
    {
        target_plot_az = ctrl->target->az;
        target_plot_el = ctrl->target->el;
    }
    rotctrl_set_target_pos_on_plots(ctrl, target_plot_az, target_plot_el);
    update_count_down(ctrl, ctrl->t);
    update_aoslos_banner(ctrl, ctrl->t);
    rotctrl_update_detached_labels(ctrl);
    rotctrl_queue_draw_plots(ctrl);

    sat_log_log(SAT_LOG_LEVEL_INFO,
                "target selection applied source=%s sat=%s catnr=%d",
                source ? source : "unknown",
                ctrl->target->nickname ? ctrl->target->nickname : "(none)",
                ctrl->target->tle.catnr);
    rot_term_log_verbose(ctrl, "gpredict:state",
                         "target selection applied source=%s sat=%s catnr=%d",
                         source ? source : "unknown",
                         ctrl->target->nickname ? ctrl->target->nickname : "(none)",
                         ctrl->target->tle.catnr);

    return TRUE;
}

static void sat_selected_cb(GtkComboBox * satsel, gpointer data)
{
    GtkRotCtrl     *ctrl = GTK_ROT_CTRL(data);
    gint            i;

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    i = gtk_combo_box_get_active(satsel);
    if (!rotctrl_apply_sat_selection(ctrl, i, "combo"))
        return;

    if (ctrl->module != NULL &&
        ctrl->target != NULL &&
        ctrl->module->target != ctrl->target->tle.catnr)
    {
        gtk_sat_module_select_sat(ctrl->module, ctrl->target->tle.catnr);
    }
}

/* Create target widgets */
static GtkWidget *create_target_widgets(GtkRotCtrl * ctrl)
{
    GtkWidget      *frame, *table, *label, *values_grid;
    gchar          *buff;
    guint           i, n;
    sat_t          *sat = NULL;

    buff = g_strdup(" --- ");

    table = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(table), 5);
    gtk_grid_set_column_homogeneous(GTK_GRID(table), FALSE);
    gtk_grid_set_column_spacing(GTK_GRID(table), 5);
    gtk_grid_set_row_spacing(GTK_GRID(table), 8);

    /* sat selector */
    ctrl->SatSel = gtk_combo_box_text_new();
    n = g_slist_length(ctrl->sats);

    for (i = 0; i < n; i++)
    {
        const gchar    *sat_name = _("Unknown");
        sat = SAT(g_slist_nth_data(ctrl->sats, i));
        if (sat)
        {
            if (sat->nickname != NULL && *sat->nickname != '\0')
                sat_name = sat->nickname;
            else if (sat->name != NULL && *sat->name != '\0')
                sat_name = sat->name;
            gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ctrl->SatSel),
                                           sat_name);
        }
    }
    gtk_combo_box_set_active(GTK_COMBO_BOX(ctrl->SatSel), -1);
    gtk_widget_set_tooltip_text(ctrl->SatSel, _("Select target object"));
    g_signal_connect(ctrl->SatSel, "changed", G_CALLBACK(sat_selected_cb),
                     ctrl);
    gp_ui_quarantine_register_combo(gtk_widget_get_toplevel(GTK_WIDGET(ctrl)),
                                    GTK_COMBO_BOX(ctrl->SatSel));
    gtk_grid_attach(GTK_GRID(table), ctrl->SatSel, 0, 0, 2, 1);

    /* tracking button */
    ctrl->track = gtk_toggle_button_new_with_label(_("Track"));
    gtk_widget_set_tooltip_text(ctrl->track,
                                _
                                ("Track the satellite when it is within range"));
    gtk_grid_attach(GTK_GRID(table), ctrl->track, 2, 0, 1, 1);
    g_signal_connect(ctrl->track, "toggled", G_CALLBACK(track_toggle_cb),
                     ctrl);

    ctrl->freeze = gtk_button_new_with_label(_("Freeze"));
    gtk_widget_set_tooltip_text(ctrl->freeze,
                                _("Stop tracking and hold the rotor at its current position"));
    gtk_grid_attach(GTK_GRID(table), ctrl->freeze, 2, 1, 1, 1);
    g_signal_connect(ctrl->freeze, "clicked", G_CALLBACK(freeze_clicked_cb),
                     ctrl);

    values_grid = gtk_grid_new();
    gtk_grid_set_column_spacing(GTK_GRID(values_grid), 5);
    gtk_grid_set_row_spacing(GTK_GRID(values_grid), 8);
    gtk_widget_set_halign(values_grid, GTK_ALIGN_START);
    gtk_widget_set_valign(values_grid, GTK_ALIGN_START);
    gtk_grid_attach(GTK_GRID(table), values_grid, 0, 1, 2, 3);

    /* Azimuth */
    label = gtk_label_new(_("Az:"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(values_grid), label, 0, 0, 1, 1);

    ctrl->AzSat = gtk_label_new(buff);
    g_object_set(ctrl->AzSat, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_label_set_width_chars(GTK_LABEL(ctrl->AzSat), 8);
    gtk_label_set_max_width_chars(GTK_LABEL(ctrl->AzSat), 8);
    gtk_label_set_single_line_mode(GTK_LABEL(ctrl->AzSat), TRUE);
    gtk_grid_attach(GTK_GRID(values_grid), ctrl->AzSat, 1, 0, 1, 1);

    /* Elevation */
    label = gtk_label_new(_("El:"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(values_grid), label, 0, 1, 1, 1);

    ctrl->ElSat = gtk_label_new(buff);
    g_object_set(ctrl->ElSat, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_label_set_width_chars(GTK_LABEL(ctrl->ElSat), 8);
    gtk_label_set_max_width_chars(GTK_LABEL(ctrl->ElSat), 8);
    gtk_label_set_single_line_mode(GTK_LABEL(ctrl->ElSat), TRUE);
    gtk_grid_attach(GTK_GRID(values_grid), ctrl->ElSat, 1, 1, 1, 1);

    /* count down */
    label = gtk_label_new(_("\316\224T:"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(values_grid), label, 0, 2, 1, 1);
    ctrl->SatCnt = gtk_label_new("--:--");
    g_object_set(ctrl->SatCnt, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_label_set_width_chars(GTK_LABEL(ctrl->SatCnt), 8);
    gtk_label_set_max_width_chars(GTK_LABEL(ctrl->SatCnt), 8);
    gtk_label_set_single_line_mode(GTK_LABEL(ctrl->SatCnt), TRUE);
    gtk_grid_attach(GTK_GRID(values_grid), ctrl->SatCnt, 1, 2, 1, 1);

    frame = gtk_frame_new(_("Target"));
    gtk_container_add(GTK_CONTAINER(frame), table);

    g_free(buff);

    return frame;
}

static GtkWidget *create_aoslos_banner_widgets(GtkRotCtrl *ctrl)
{
    GtkWidget      *frame;

    ctrl->aoslos_banner = gtk_label_new(NULL);
    gp_safe_label_set_markup(ctrl->aoslos_banner,
                             ROTCTRL_AOSLOS_PLACEHOLDER);
    gtk_widget_set_tooltip_text(ctrl->aoslos_banner,
                                _("The time remaining until the next AOS or "
                                  "LOS event"));
    g_object_set(ctrl->aoslos_banner, "xalign", 0.5f, "yalign", 0.5f, NULL);
    gtk_widget_set_margin_top(ctrl->aoslos_banner, 3);
    gtk_widget_set_margin_bottom(ctrl->aoslos_banner, 3);

    frame = gtk_frame_new(NULL);
    gtk_container_add(GTK_CONTAINER(frame), ctrl->aoslos_banner);

    return frame;
}

static GtkWidget *create_conf_widgets(GtkRotCtrl * ctrl)
{
    const gint      action_panel_width = 240;
    GtkWidget      *frame, *main_table, *label;
    GtkWidget      *device_row;
    GtkWidget      *cycle_row;
    GtkWidget      *threshold_row;
    GtkWidget      *engage_panel;
    GtkWidget      *status_panel;
    GtkWidget      *status_box;
    GtkWidget      *status_led;
    GtkWidget      *logging_row;
    GtkWidget      *monitor_row;
    GtkWidget      *status;
    GtkWidget      *verbose_check;
    GtkSizeGroup   *left_label_group;
    GDir           *dir = NULL; /* directory handle */
    GError         *error = NULL;       /* error flag and info */
    gchar          *dirname;    /* directory name */
    gchar         **vbuff;
    const gchar    *filename;   /* file name */
    gchar          *rotname;

    main_table = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(main_table), 5);
    gtk_grid_set_column_spacing(GTK_GRID(main_table), ROTCTRL_PANEL_SPACING);
    gtk_grid_set_row_spacing(GTK_GRID(main_table), 8);
    left_label_group = gtk_size_group_new(GTK_SIZE_GROUP_HORIZONTAL);

    device_row = gtk_grid_new();
    gtk_grid_set_column_spacing(GTK_GRID(device_row), 5);
    gtk_grid_set_row_spacing(GTK_GRID(device_row), 5);
    gtk_widget_set_halign(device_row, GTK_ALIGN_START);

    label = gtk_label_new(_("Device:"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_widget_set_halign(label, GTK_ALIGN_START);
    gtk_size_group_add_widget(left_label_group, label);
    gtk_grid_attach(GTK_GRID(device_row), label, 0, 0, 1, 1);

    ctrl->DevSel = gtk_combo_box_text_new();
    gtk_widget_set_tooltip_text(ctrl->DevSel,
                                _("Select antenna rotator device"));
    gtk_widget_set_halign(ctrl->DevSel, GTK_ALIGN_START);
    gtk_widget_set_size_request(ctrl->DevSel, 220, -1);

    /* open configuration directory */
    dirname = get_hwconf_dir();

    dir = g_dir_open(dirname, 0, &error);
    if (dir)
    {
        /* read each .rot file */
        GSList         *rots = NULL;
        gint            i;
        gint            n;

        while ((filename = g_dir_read_name(dir)))
        {
            if (g_str_has_suffix(filename, ".rot"))
            {
                vbuff = g_strsplit(filename, ".rot", 0);
                rots =
                    g_slist_insert_sorted(rots, g_strdup(vbuff[0]),
                                          (GCompareFunc) rot_name_compare);
                g_strfreev(vbuff);
            }
        }
        n = g_slist_length(rots);
        for (i = 0; i < n; i++)
        {
            rotname = g_slist_nth_data(rots, i);
            if (rotname)
            {
                gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT
                                               (ctrl->DevSel), rotname);
                g_free(rotname);
            }
        }
        g_slist_free(rots);
    }
    else
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%d: Failed to open hwconf dir (%s)"),
                    __FILE__, __LINE__, error->message);
        g_clear_error(&error);
    }

    g_free(dirname);
    if (dir)
        g_dir_close(dir);

    gtk_combo_box_set_active(GTK_COMBO_BOX(ctrl->DevSel), 0);
    g_signal_connect(ctrl->DevSel, "changed", G_CALLBACK(rot_selected_cb),
                     ctrl);
    gp_ui_quarantine_register_combo(gtk_widget_get_toplevel(GTK_WIDGET(ctrl)),
                                    GTK_COMBO_BOX(ctrl->DevSel));
    gtk_grid_attach(GTK_GRID(device_row), ctrl->DevSel, 1, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(main_table), device_row, 0, 0, 1, 1);

    /* Engage button */
    ctrl->LockBut = gtk_toggle_button_new_with_label(_("Engage"));
    gtk_widget_set_tooltip_text(ctrl->LockBut,
                                _("Engage the selected rotor device"));
    g_signal_connect(ctrl->LockBut, "toggled", G_CALLBACK(rot_locked_cb),
                     ctrl);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->LockBut), FALSE);
    gtk_widget_set_hexpand(ctrl->LockBut, TRUE);
    gtk_widget_set_halign(ctrl->LockBut, GTK_ALIGN_FILL);
    gtk_widget_set_valign(ctrl->LockBut, GTK_ALIGN_CENTER);
    engage_panel = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_widget_set_halign(engage_panel, GTK_ALIGN_CENTER);
    gtk_widget_set_size_request(engage_panel, action_panel_width, -1);
    gtk_box_pack_start(GTK_BOX(engage_panel), ctrl->LockBut, TRUE, TRUE, 0);
    gtk_grid_attach(GTK_GRID(main_table), engage_panel, 1, 0, 1, 1);
    rotctrl_reset_lock_button_visual(ctrl);

    /* Monitor checkbox */
    ctrl->MonitorCheckBox = gtk_check_button_new_with_label(_("Monitor"));
    gtk_widget_set_tooltip_text(ctrl->MonitorCheckBox,
                                _("Monitor rotator but do not send any "
                                  "position commands"));
    g_signal_connect(ctrl->MonitorCheckBox, "toggled",
                     G_CALLBACK(rot_monitor_cb), ctrl);
    gtk_widget_set_halign(ctrl->MonitorCheckBox, GTK_ALIGN_START);
    gtk_widget_set_valign(ctrl->MonitorCheckBox, GTK_ALIGN_CENTER);

    /* Logs toggle */
    ctrl->log_toggle = gtk_toggle_button_new_with_label(_("Show log"));
    gtk_widget_set_tooltip_text(ctrl->log_toggle,
                                _("Show or hide the rotor control log"));
    g_signal_connect(ctrl->log_toggle, "toggled",
                     G_CALLBACK(rot_logs_toggle_cb), ctrl);
    if (ctrl->term_view != NULL)
        gp_term_view_set_visible(ctrl->term_view, FALSE);
    gtk_widget_set_hexpand(ctrl->log_toggle, TRUE);
    gtk_widget_set_halign(ctrl->log_toggle, GTK_ALIGN_START);
    gtk_widget_set_valign(ctrl->log_toggle, GTK_ALIGN_CENTER);
    gtk_grid_attach(GTK_GRID(main_table), ctrl->log_toggle, 2, 0, 1, 1);

    /* cycle period */
    cycle_row = gtk_grid_new();
    gtk_grid_set_column_spacing(GTK_GRID(cycle_row), 5);
    gtk_grid_set_row_spacing(GTK_GRID(cycle_row), 5);
    gtk_widget_set_halign(cycle_row, GTK_ALIGN_START);

    label = gtk_label_new(_("Cycle:"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_widget_set_halign(label, GTK_ALIGN_START);
    gtk_size_group_add_widget(left_label_group, label);
    gtk_grid_attach(GTK_GRID(cycle_row), label, 0, 0, 1, 1);

    ctrl->cycle_spin = gtk_spin_button_new_with_range(100, 10000, 10);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(ctrl->cycle_spin), 0);
    gtk_widget_set_tooltip_text(ctrl->cycle_spin,
                                _("This parameter controls the delay between "
                                  "commands sent to the rotator."));
    gtk_widget_set_halign(ctrl->cycle_spin, GTK_ALIGN_START);
    gtk_entry_set_width_chars(GTK_ENTRY(ctrl->cycle_spin), 5);
    gtk_widget_set_size_request(ctrl->cycle_spin, 125, -1);
    g_signal_connect(ctrl->cycle_spin, "value-changed",
                     G_CALLBACK(delay_changed_cb), ctrl);
    g_signal_connect(ctrl->cycle_spin, "focus-out-event",
                     G_CALLBACK(rotctrl_settings_focus_out_cb), ctrl);
    g_signal_connect(ctrl->cycle_spin, "activate",
                     G_CALLBACK(rotctrl_settings_activate_cb), ctrl);
    gtk_grid_attach(GTK_GRID(cycle_row), ctrl->cycle_spin, 1, 0, 1, 1);

    label = gtk_label_new(_("msec"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_widget_set_halign(label, GTK_ALIGN_START);
    gtk_grid_attach(GTK_GRID(cycle_row), label, 2, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(main_table), cycle_row, 0, 1, 1, 1);

    /* Tolerance */
    threshold_row = gtk_grid_new();
    gtk_grid_set_column_spacing(GTK_GRID(threshold_row), 5);
    gtk_grid_set_row_spacing(GTK_GRID(threshold_row), 5);
    gtk_widget_set_halign(threshold_row, GTK_ALIGN_START);

    label = gtk_label_new(_("Threshold:"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_widget_set_halign(label, GTK_ALIGN_START);
    gtk_size_group_add_widget(left_label_group, label);
    gtk_grid_attach(GTK_GRID(threshold_row), label, 0, 0, 1, 1);

    ctrl->thld_spin = gtk_spin_button_new_with_range(0.01, 50.0, 0.01);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(ctrl->thld_spin), 2);
    gtk_widget_set_tooltip_text(ctrl->thld_spin,
                                _("This parameter sets the threshold that triggers "
                                  "new motion command to the rotator.\n"
                                  "If the difference between the target and "
                                  "rotator values is smaller than the "
                                  "threshold, no new commands are sent"));
    gtk_widget_set_halign(ctrl->thld_spin, GTK_ALIGN_START);
    gtk_entry_set_width_chars(GTK_ENTRY(ctrl->thld_spin), 5);
    gtk_widget_set_size_request(ctrl->thld_spin, 125, -1);
    g_signal_connect(ctrl->thld_spin, "value-changed",
                     G_CALLBACK(threshold_changed_cb), ctrl);
    g_signal_connect(ctrl->thld_spin, "focus-out-event",
                     G_CALLBACK(rotctrl_settings_focus_out_cb), ctrl);
    g_signal_connect(ctrl->thld_spin, "activate",
                     G_CALLBACK(rotctrl_settings_activate_cb), ctrl);
    gtk_grid_attach(GTK_GRID(threshold_row), ctrl->thld_spin, 1, 0, 1, 1);

    label = gtk_label_new(_("deg"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_widget_set_halign(label, GTK_ALIGN_START);
    gtk_grid_attach(GTK_GRID(threshold_row), label, 2, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(main_table), threshold_row, 0, 2, 1, 1);

    /* Status line */
    status_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    gtk_widget_set_halign(status_box, GTK_ALIGN_CENTER);
    gtk_widget_set_margin_top(status_box, 5);
    gtk_widget_set_margin_bottom(status_box, 5);
    gtk_widget_set_margin_start(status_box, 8);
    gtk_widget_set_margin_end(status_box, 8);

    status_led = status_indicator_new();
    gtk_box_pack_start(GTK_BOX(status_box), status_led, FALSE, FALSE, 0);

    status =
        gtk_label_new(rotor_ui_status_to_string(ROTOR_UI_STATUS_DISENGAGED));
    g_object_set(status, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_label_set_ellipsize(GTK_LABEL(status), PANGO_ELLIPSIZE_END);
    gtk_widget_set_halign(status, GTK_ALIGN_START);
    {
        PangoAttrList *attrs = pango_attr_list_new();
        pango_attr_list_insert(attrs,
                               pango_attr_scale_new(1.30));
        pango_attr_list_insert(attrs,
                               pango_attr_weight_new(PANGO_WEIGHT_MEDIUM));
        gtk_label_set_attributes(GTK_LABEL(status), attrs);
        pango_attr_list_unref(attrs);
    }
    gtk_box_pack_start(GTK_BOX(status_box), status, FALSE, FALSE, 0);

    status_panel = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 0);
    gtk_widget_set_halign(status_panel, GTK_ALIGN_CENTER);
    gtk_widget_set_size_request(status_panel, action_panel_width, 42);
    gtk_box_pack_start(GTK_BOX(status_panel), status_box, TRUE, TRUE, 0);
    gtk_grid_attach(GTK_GRID(main_table), status_panel, 1, 1, 1, 1);

    /* store pointer on the controller object for later updates */
    g_object_set_data(G_OBJECT(ctrl), "rot-status-indicator", status_led);
    g_object_set_data(G_OBJECT(ctrl), "rot-status-label", status);
    {
        RotorStateSnapshot snap = { 0 };
        snap.control_active = FALSE;
        rotctrl_apply_ui_status(ctrl, &snap, "widget init");
    }

    /* Verbose logging */
    logging_row = gtk_grid_new();
    gtk_grid_set_column_spacing(GTK_GRID(logging_row), 5);
    gtk_grid_set_row_spacing(GTK_GRID(logging_row), 5);
    gtk_widget_set_hexpand(logging_row, TRUE);
    gtk_widget_set_halign(logging_row, GTK_ALIGN_START);

    label = gtk_label_new(_("Logging:"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_widget_set_halign(label, GTK_ALIGN_START);
    gtk_grid_attach(GTK_GRID(logging_row), label, 0, 0, 1, 1);

    verbose_check = gtk_check_button_new_with_label(_("Verbose"));
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(verbose_check),
                                 ctrl->verbose_logging);
    gtk_widget_set_tooltip_text(verbose_check,
                                _("Include rotctld -vvvv output and extra "
                                  "diagnostic logging in the terminal."));
    g_signal_connect(verbose_check, "toggled",
                     G_CALLBACK(rot_verbose_cb), ctrl);
    gtk_widget_set_halign(verbose_check, GTK_ALIGN_START);
    gtk_widget_set_valign(verbose_check, GTK_ALIGN_CENTER);
    gtk_grid_attach(GTK_GRID(logging_row), verbose_check, 1, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(main_table), logging_row, 2, 1, 1, 1);

    monitor_row = gtk_grid_new();
    gtk_grid_set_column_spacing(GTK_GRID(monitor_row), 5);
    gtk_grid_set_row_spacing(GTK_GRID(monitor_row), 5);
    gtk_widget_set_hexpand(monitor_row, TRUE);
    gtk_widget_set_halign(monitor_row, GTK_ALIGN_START);

    label = gtk_label_new(_("Monitor:"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_widget_set_halign(label, GTK_ALIGN_START);
    gtk_grid_attach(GTK_GRID(monitor_row), label, 0, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(monitor_row), ctrl->MonitorCheckBox, 1, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(main_table), monitor_row, 2, 2, 1, 1);
    g_object_unref(left_label_group);

    ctrl->settings_window = gtk_widget_get_toplevel(GTK_WIDGET(main_table));

    frame = gtk_frame_new(_("Settings"));
    gtk_container_add(GTK_CONTAINER(frame), main_table);

    return frame;
}

static GtkWidget *create_calibration_widgets(GtkRotCtrl *ctrl)
{
    GtkWidget *frame, *grid, *label, *offset_grid, *autocal_button;

    frame = gtk_frame_new(_("Calibration"));

    grid = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(grid), 5);
    gtk_grid_set_column_spacing(GTK_GRID(grid), 5);
    gtk_grid_set_row_spacing(GTK_GRID(grid), 5);
    gtk_container_add(GTK_CONTAINER(frame), grid);

    autocal_button = gtk_button_new_with_label(_("Auto-calibration"));
    gtk_widget_set_tooltip_text(autocal_button,
                                _("Moves to 0/0 and prompts for alignment to TRUE NORTH."));
    g_signal_connect(autocal_button, "clicked",
                     G_CALLBACK(calib_autocal_clicked_cb), ctrl);
    gtk_grid_attach(GTK_GRID(grid), autocal_button, 0, 0, 2, 1);
    ctrl->calib_autocal_button = autocal_button;
    gtk_widget_set_sensitive(autocal_button, !ctrl->cal_active);

    ctrl->offset_check =
        gtk_check_button_new_with_label(_("Enable Offsets"));
    g_signal_connect(ctrl->offset_check, "toggled",
                     G_CALLBACK(offset_toggle_cb), ctrl);
    gtk_grid_attach(GTK_GRID(grid), ctrl->offset_check, 0, 1, 2, 1);

    offset_grid = gtk_grid_new();
    gtk_grid_set_column_spacing(GTK_GRID(offset_grid), 5);
    gtk_grid_set_row_spacing(GTK_GRID(offset_grid), 5);
    ctrl->calib_offsets_grid = offset_grid;

    label = gtk_label_new(_("Az offset (\302\260)"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(offset_grid), label, 0, 0, 1, 1);
    ctrl->az_offset_spin = gtk_spin_button_new_with_range(0.0, 359.99, 0.1);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(ctrl->az_offset_spin), 2);
    g_signal_connect(ctrl->az_offset_spin, "value-changed",
                     G_CALLBACK(az_offset_changed_cb), ctrl);
    gtk_grid_attach(GTK_GRID(offset_grid), ctrl->az_offset_spin, 1, 0, 1, 1);

    label = gtk_label_new(_("El offset (\302\260)"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(offset_grid), label, 0, 1, 1, 1);
    ctrl->el_offset_spin = gtk_spin_button_new_with_range(-180.0, 180.0, 0.1);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(ctrl->el_offset_spin), 2);
    g_signal_connect(ctrl->el_offset_spin, "value-changed",
                     G_CALLBACK(el_offset_changed_cb), ctrl);
    gtk_grid_attach(GTK_GRID(offset_grid), ctrl->el_offset_spin, 1, 1, 1, 1);

    gtk_grid_attach(GTK_GRID(grid), offset_grid, 0, 2, 2, 1);
    rotctrl_calib_update_offset_sensitivity(ctrl);

    return frame;
}

/**
 * Park the rotor at a "rest" position.
 *
 * Park at the configured station rest position (user space).
 */
static void G_GNUC_UNUSED
rot_park_zenith_cb(GtkButton *button, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);
    gdouble park_user_az = 0.0;
    gdouble park_user_el = 0.0;
    gdouble park_az360 = 0.0;
    gdouble park_backend_az = 0.0;
    gdouble park_backend_el = 0.0;
    rot_ui_mode_t ui_mode = ROT_UI_360;
    RotTargetOut xform = { 0 };
    gboolean xform_ok = FALSE;

    (void)button;

    /* Require a valid configuration and an engaged, running client. */
    if (!ctrl->conf || !ctrl->engaged || !ctrl->client.running) {
        rot_show_message(ctrl,
                         GTK_MESSAGE_WARNING,
                         _("Park rotor"),
                         _("Engage the rotator before parking it."));
        return;
    }

    if (ctrl->cal_hold_active)
        rotctrl_set_cal_hold(ctrl, FALSE, "park");

    if (ctrl->tracking && ctrl->track)
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->track), FALSE);

    ui_mode = (ctrl->conf && ctrl->conf->aztype == ROT_AZ_TYPE_180)
              ? ROT_UI_NORTH_CENTERED
              : ROT_UI_360;
    rot_get_park_position(ctrl, &park_user_az, &park_user_el);
    park_az360 = rot_ui_to_az360(park_user_az, ui_mode);
    xform_ok = gp_rot_transform_target(ctrl,
                                       park_az360,
                                       park_user_el,
                                       -1.0,
                                       &xform);
    park_backend_az = xform_ok ? xform.az360_final : park_az360;
    park_backend_el = xform_ok ? xform.el_final : park_user_el;

    sat_log_log(SAT_LOG_LEVEL_INFO,
                "park pressed: park_user=(%.2f,%.2f) park_backend=(%.2f,%.2f)",
                park_user_az, park_user_el,
                park_backend_az, park_backend_el);
    rot_term_log(ctrl, "gpredict:rx",
                 "park pressed: park_user=(%.2f,%.2f) park_backend=(%.2f,%.2f)",
                 park_user_az, park_user_el,
                 park_backend_az, park_backend_el);

    /* Route through the unified control loop: update knobs and allow manual send. */
    gtk_rot_knob_set_value(GTK_ROT_KNOB(ctrl->AzSet), park_user_az);
    gtk_rot_knob_set_value(GTK_ROT_KNOB(ctrl->ElSet), park_user_el);
    ctrl->manual_edit_until_us = g_get_monotonic_time() + 1000000;
    ctrl->manual_sync_pending = FALSE;
    ctrl->have_user_command = TRUE;
    ctrl->hold_position_on_engage = FALSE;
    ctrl->park_requested = TRUE;
    ctrl->force_next_send = TRUE;

    /* Inform the user what was commanded. */
    {
        gchar *msg = g_strdup_printf(_("The rotor has been commanded to AZ=%.2f\302\260, EL=%.2f\302\260."),
                                     park_user_az, park_user_el);
        rot_show_message(ctrl,
                         GTK_MESSAGE_INFO,
                         _("Park rotor"),
                         msg);
        g_free(msg);
    }
}

typedef struct {
    GtkRotCtrl *ctrl;
    gint index;
    GtkWidget *dialog;
    GtkWidget *name_entry;
    GtkWidget *az_spin;
    GtkWidget *el_spin;
    GtkWidget *delete_button;
} RotPresetDialog;

static gchar *rotctrl_presets_config_path(GtkRotCtrl *ctrl)
{
    gchar *dirname = NULL;
    gchar *path = NULL;

    if (ctrl == NULL || ctrl->conf == NULL || ctrl->conf->name == NULL)
        return NULL;

    dirname = get_hwconf_dir();
    path = g_strconcat(dirname, G_DIR_SEPARATOR_S,
                       ctrl->conf->name, ".rot", NULL);
    g_free(dirname);

    return path;
}

static void rotctrl_presets_clear(GtkRotCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    for (guint i = 0; i < ctrl->preset_count; i++)
    {
        g_free(ctrl->presets[i].name);
        ctrl->presets[i].name = NULL;
    }
    ctrl->preset_count = 0;
}

static gboolean rotctrl_presets_load(GtkRotCtrl *ctrl)
{
    GKeyFile *cfg = NULL;
    gchar *path = NULL;
    GError *error = NULL;
    gint count = 0;

    if (ctrl == NULL || ctrl->conf == NULL)
        return FALSE;

    path = rotctrl_presets_config_path(ctrl);
    if (path == NULL)
        return FALSE;

    cfg = g_key_file_new();
    if (!g_key_file_load_from_file(cfg, path,
                                   G_KEY_FILE_KEEP_COMMENTS |
                                   G_KEY_FILE_KEEP_TRANSLATIONS,
                                   NULL))
    {
        g_key_file_free(cfg);
        g_free(path);
        rotctrl_presets_clear(ctrl);
        return FALSE;
    }

    rotctrl_presets_clear(ctrl);

    if (!g_key_file_has_group(cfg, ROT_PRESET_GROUP))
    {
        g_key_file_free(cfg);
        g_free(path);
        return TRUE;
    }

    count = g_key_file_get_integer(cfg, ROT_PRESET_GROUP,
                                   ROT_PRESET_COUNT_KEY, &error);
    if (error != NULL)
    {
        g_clear_error(&error);
        count = 0;
    }

    for (gint i = 0; i < count && i < ROT_PRESET_MAX; i++)
    {
        gchar *key = NULL;
        gchar *name = NULL;
        gdouble az = 0.0;
        gdouble el = 0.0;

        key = g_strdup_printf("preset%d_name", i);
        name = g_key_file_get_string(cfg, ROT_PRESET_GROUP, key, &error);
        g_free(key);
        if (error != NULL)
        {
            g_clear_error(&error);
            continue;
        }

        key = g_strdup_printf("preset%d_az", i);
        az = g_key_file_get_double(cfg, ROT_PRESET_GROUP, key, &error);
        g_free(key);
        if (error != NULL)
        {
            g_clear_error(&error);
            g_free(name);
            continue;
        }

        key = g_strdup_printf("preset%d_el", i);
        el = g_key_file_get_double(cfg, ROT_PRESET_GROUP, key, &error);
        g_free(key);
        if (error != NULL)
        {
            g_clear_error(&error);
            g_free(name);
            continue;
        }

        ctrl->presets[ctrl->preset_count].name = name;
        ctrl->presets[ctrl->preset_count].az = az;
        ctrl->presets[ctrl->preset_count].el = el;
        ctrl->preset_count++;
    }

    g_key_file_free(cfg);
    g_free(path);
    return TRUE;
}

static gboolean rotctrl_presets_save(GtkRotCtrl *ctrl)
{
    GKeyFile *cfg = NULL;
    gchar *path = NULL;
    gchar *data = NULL;
    gsize len = 0;
    GError *error = NULL;

    if (ctrl == NULL || ctrl->conf == NULL)
        return FALSE;

    path = rotctrl_presets_config_path(ctrl);
    if (path == NULL)
        return FALSE;

    cfg = g_key_file_new();
    g_key_file_load_from_file(cfg, path,
                              G_KEY_FILE_KEEP_COMMENTS |
                              G_KEY_FILE_KEEP_TRANSLATIONS,
                              NULL);

    if (g_key_file_has_group(cfg, ROT_PRESET_GROUP))
        g_key_file_remove_group(cfg, ROT_PRESET_GROUP, NULL);

    g_key_file_set_integer(cfg, ROT_PRESET_GROUP,
                           ROT_PRESET_COUNT_KEY,
                           (gint) ctrl->preset_count);

    for (guint i = 0; i < ctrl->preset_count && i < ROT_PRESET_MAX; i++)
    {
        gchar *key = NULL;

        key = g_strdup_printf("preset%u_name", i);
        g_key_file_set_string(cfg, ROT_PRESET_GROUP, key,
                              ctrl->presets[i].name ? ctrl->presets[i].name : "");
        g_free(key);

        key = g_strdup_printf("preset%u_az", i);
        g_key_file_set_double(cfg, ROT_PRESET_GROUP, key,
                              ctrl->presets[i].az);
        g_free(key);

        key = g_strdup_printf("preset%u_el", i);
        g_key_file_set_double(cfg, ROT_PRESET_GROUP, key,
                              ctrl->presets[i].el);
        g_free(key);
    }

    data = g_key_file_to_data(cfg, &len, NULL);
    if (!g_file_set_contents(path, data, len, &error))
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: Failed to save presets: %s"),
                    __func__,
                    error ? error->message : "unknown error");
        g_clear_error(&error);
        g_free(data);
        g_key_file_free(cfg);
        g_free(path);
        return FALSE;
    }

    g_free(data);
    g_key_file_free(cfg);
    g_free(path);
    return TRUE;
}

static void rotctrl_presets_apply(GtkRotCtrl *ctrl,
                                  gint index,
                                  const gchar *name,
                                  gdouble az,
                                  gdouble el)
{
    if (ctrl == NULL || name == NULL || *name == '\0')
        return;

    if (index < 0)
    {
        if (ctrl->preset_count >= ROT_PRESET_MAX)
            return;
        index = (gint) ctrl->preset_count;
        ctrl->preset_count++;
    }
    else if ((guint) index >= ctrl->preset_count)
    {
        return;
    }

    g_free(ctrl->presets[index].name);
    ctrl->presets[index].name = g_strdup(name);
    ctrl->presets[index].az = az;
    ctrl->presets[index].el = el;

    rotctrl_presets_save(ctrl);
    rotctrl_presets_refresh(ctrl);
}

static void rotctrl_presets_delete(GtkRotCtrl *ctrl, gint index)
{
    if (ctrl == NULL)
        return;
    if (index < 0 || (guint) index >= ctrl->preset_count)
        return;

    g_free(ctrl->presets[index].name);
    ctrl->presets[index].name = NULL;

    for (guint i = (guint) index + 1; i < ctrl->preset_count; i++)
    {
        ctrl->presets[i - 1] = ctrl->presets[i];
    }

    if (ctrl->preset_count > 0)
        ctrl->preset_count--;

    if (ctrl->preset_count < ROT_PRESET_MAX)
    {
        ctrl->presets[ctrl->preset_count].name = NULL;
        ctrl->presets[ctrl->preset_count].az = 0.0;
        ctrl->presets[ctrl->preset_count].el = 0.0;
    }

    rotctrl_presets_save(ctrl);
    rotctrl_presets_refresh(ctrl);
}

static void rotctrl_preset_dialog_free(gpointer data)
{
    RotPresetDialog *ctx = data;

    g_free(ctx);
}

static void rotctrl_preset_dialog_response(GtkDialog *dialog,
                                           gint response_id,
                                           gpointer user_data)
{
    RotPresetDialog *ctx = user_data;
    GtkRotCtrl *ctrl = ctx ? ctx->ctrl : NULL;

    if (ctrl == NULL)
    {
        gtk_widget_destroy(GTK_WIDGET(dialog));
        return;
    }

    if (response_id == GTK_RESPONSE_OK)
    {
        const gchar *raw_name =
            gtk_entry_get_text(GTK_ENTRY(ctx->name_entry));
        gchar *name = g_strdup(raw_name ? raw_name : "");
        GtkWidget *error_dialog = NULL;
        gdouble az = gtk_spin_button_get_value(GTK_SPIN_BUTTON(ctx->az_spin));
        gdouble el = gtk_spin_button_get_value(GTK_SPIN_BUTTON(ctx->el_spin));

        g_strstrip(name);
        if (*name == '\0')
        {
            error_dialog = gtk_message_dialog_new(GTK_WINDOW(ctx->dialog),
                                                  GTK_DIALOG_MODAL |
                                                  GTK_DIALOG_DESTROY_WITH_PARENT,
                                                  GTK_MESSAGE_ERROR,
                                                  GTK_BUTTONS_OK,
                                                  "%s",
                                                  _("Preset name cannot be empty."));
            gtk_dialog_run(GTK_DIALOG(error_dialog));
            gtk_widget_destroy(error_dialog);
            gtk_widget_grab_focus(ctx->name_entry);
            g_free(name);
            return;
        }

        rotctrl_presets_apply(ctrl, ctx->index, name, az, el);
        g_free(name);
    }
    else if (response_id == ROT_PRESET_RESPONSE_DELETE)
    {
        rotctrl_presets_delete(ctrl, ctx->index);
    }

    gtk_widget_destroy(GTK_WIDGET(dialog));
}

static void rotctrl_preset_dialog_open(GtkRotCtrl *ctrl, gint index)
{
    RotPresetDialog *ctx = NULL;
    GtkWidget *dialog = NULL;
    GtkWidget *content = NULL;
    GtkWidget *grid = NULL;
    GtkWidget *label = NULL;
    GtkWidget *entry = NULL;
    GtkWidget *az_spin = NULL;
    GtkWidget *el_spin = NULL;
    gdouble preset_min_el = ROTCTRL_DEFAULT_MIN_EL;
    gdouble preset_max_el = ROTCTRL_DEFAULT_MAX_EL;
    GtkWidget *delete_btn = NULL;
    gboolean is_edit = FALSE;

    if (ctrl == NULL)
        return;

    is_edit = (index >= 0 && (guint) index < ctrl->preset_count);

    dialog = gtk_dialog_new_with_buttons(_("Preset position"),
                                         GTK_WINDOW(gtk_widget_get_toplevel(GTK_WIDGET(ctrl))),
                                         GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
                                         _("_Cancel"), GTK_RESPONSE_CANCEL,
                                         _("_Delete preset"), ROT_PRESET_RESPONSE_DELETE,
                                         _("_OK"), GTK_RESPONSE_OK,
                                         NULL);
    gtk_dialog_set_default_response(GTK_DIALOG(dialog), GTK_RESPONSE_OK);

    content = gtk_dialog_get_content_area(GTK_DIALOG(dialog));
    grid = gtk_grid_new();
    gtk_grid_set_row_spacing(GTK_GRID(grid), 6);
    gtk_grid_set_column_spacing(GTK_GRID(grid), 6);
    gtk_container_set_border_width(GTK_CONTAINER(grid), 8);
    gtk_container_add(GTK_CONTAINER(content), grid);

    label = gtk_label_new(_("Preset name"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(grid), label, 0, 0, 1, 1);

    entry = gtk_entry_new();
    gtk_grid_attach(GTK_GRID(grid), entry, 1, 0, 1, 1);

    label = gtk_label_new(_("Preset Az"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(grid), label, 0, 1, 1, 1);

    az_spin = gtk_spin_button_new_with_range(-360.0, 360.0, 0.1);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(az_spin), 2);
    gtk_grid_attach(GTK_GRID(grid), az_spin, 1, 1, 1, 1);

    label = gtk_label_new(_("Preset El"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(grid), label, 0, 2, 1, 1);

    if (ctrl && ctrl->conf)
    {
        preset_min_el = ctrl->conf->minel;
        preset_max_el = ctrl->conf->maxel;
    }
    if (preset_min_el > preset_max_el)
    {
        gdouble tmp = preset_min_el;
        preset_min_el = preset_max_el;
        preset_max_el = tmp;
    }

    el_spin = gtk_spin_button_new_with_range(preset_min_el,
                                             preset_max_el,
                                             0.1);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(el_spin), 2);
    gtk_grid_attach(GTK_GRID(grid), el_spin, 1, 2, 1, 1);

    if (is_edit)
    {
        gtk_entry_set_text(GTK_ENTRY(entry),
                           ctrl->presets[index].name ? ctrl->presets[index].name : "");
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(az_spin),
                                  ctrl->presets[index].az);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(el_spin),
                                  ctrl->presets[index].el);
    }

    delete_btn = gtk_dialog_get_widget_for_response(GTK_DIALOG(dialog),
                                                    ROT_PRESET_RESPONSE_DELETE);

    ctx = g_new0(RotPresetDialog, 1);
    ctx->ctrl = ctrl;
    ctx->index = is_edit ? index : -1;
    ctx->dialog = dialog;
    ctx->name_entry = entry;
    ctx->az_spin = az_spin;
    ctx->el_spin = el_spin;
    ctx->delete_button = delete_btn;

    g_signal_connect(dialog, "response",
                     G_CALLBACK(rotctrl_preset_dialog_response), ctx);
    g_object_set_data_full(G_OBJECT(dialog), "preset-dialog",
                           ctx, rotctrl_preset_dialog_free);

    gtk_widget_show_all(dialog);

    if (!is_edit && delete_btn != NULL)
        gtk_widget_hide(delete_btn);
}

static void rotctrl_preset_move(GtkRotCtrl *ctrl, gint index)
{
    gdouble az = 0.0;
    gdouble el = 0.0;
    const gchar *name = NULL;

    if (ctrl == NULL || index < 0 || (guint) index >= ctrl->preset_count)
        return;

    if (!ctrl->conf || !ctrl->engaged || !ctrl->client.running)
    {
        rot_show_message(ctrl,
                         GTK_MESSAGE_WARNING,
                         _("Preset position"),
                         _("Engage the rotator before moving to a preset."));
        return;
    }

    az = ctrl->presets[index].az;
    el = ctrl->presets[index].el;
    name = ctrl->presets[index].name ? ctrl->presets[index].name : _("Preset");

    if (ctrl->cal_hold_active)
        rotctrl_set_cal_hold(ctrl, FALSE, "preset");

    if (ctrl->tracking && ctrl->track)
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->track), FALSE);

    gtk_rot_knob_set_value(GTK_ROT_KNOB(ctrl->AzSet), az);
    gtk_rot_knob_set_value(GTK_ROT_KNOB(ctrl->ElSet), el);
    ctrl->manual_edit_until_us = g_get_monotonic_time() + 1000000;
    ctrl->manual_sync_pending = FALSE;
    ctrl->have_user_command = TRUE;
    ctrl->hold_position_on_engage = FALSE;
    ctrl->park_requested = FALSE;
    ctrl->force_next_send = TRUE;

    {
        gchar *msg = g_strdup_printf(_("moving to '%s' (AZ=%.2f EL=%.2f)"),
                                     name, az, el);
        rot_show_message(ctrl,
                         GTK_MESSAGE_INFO,
                         _("Preset position"),
                         msg);
        g_free(msg);
    }
}

static GtkWidget *rotctrl_preset_slot_widget(GtkRotCtrl *ctrl, guint index)
{
    GtkWidget *box = NULL;
    GtkWidget *main_btn = NULL;
    GtkWidget *edit_btn = NULL;
    const gchar *name = NULL;

    if (ctrl == NULL || index >= ctrl->preset_count)
        return NULL;

    name = ctrl->presets[index].name ? ctrl->presets[index].name : _("Preset");

    box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 2);

    main_btn = gtk_button_new_with_label(name);
    gtk_widget_set_size_request(main_btn, -1, ROT_PRESET_BUTTON_HEIGHT);
    gtk_widget_set_hexpand(main_btn, TRUE);
    gtk_widget_set_halign(main_btn, GTK_ALIGN_FILL);
    g_object_set_data(G_OBJECT(main_btn), "preset-index",
                      GINT_TO_POINTER((gint) index));
    g_signal_connect(main_btn, "clicked",
                     G_CALLBACK(rotctrl_preset_activate_cb), ctrl);

    edit_btn = gtk_button_new_with_label(_("Edit"));
    gtk_widget_set_hexpand(edit_btn, FALSE);
    gtk_widget_set_size_request(edit_btn, 48, ROT_PRESET_BUTTON_HEIGHT);
    g_object_set_data(G_OBJECT(edit_btn), "preset-index",
                      GINT_TO_POINTER((gint) index));
    g_signal_connect(edit_btn, "clicked",
                     G_CALLBACK(rotctrl_preset_edit_cb), ctrl);

    gtk_box_pack_start(GTK_BOX(box), main_btn, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(box), edit_btn, FALSE, FALSE, 0);

    return box;
}

static void rotctrl_presets_refresh(GtkRotCtrl *ctrl)
{
    GList *children = NULL;

    if (ctrl == NULL || ctrl->preset_grid == NULL)
        return;

    children = gtk_container_get_children(GTK_CONTAINER(ctrl->preset_grid));
    for (GList *item = children; item != NULL; item = item->next)
        gtk_widget_destroy(GTK_WIDGET(item->data));
    g_list_free(children);

    for (guint slot = 0; slot < ROT_PRESET_MAX; slot++)
    {
        guint row = slot / ROT_PRESET_COLS;
        guint col = slot % ROT_PRESET_COLS;
        GtkWidget *widget = NULL;

        if (slot < ctrl->preset_count)
        {
            widget = rotctrl_preset_slot_widget(ctrl, slot);
        }
        else if (slot == ctrl->preset_count &&
                 ctrl->preset_count < ROT_PRESET_MAX)
        {
            widget = gtk_button_new_with_label(_("Create preset"));
            gtk_widget_set_size_request(widget, -1, ROT_PRESET_BUTTON_HEIGHT);
            gtk_widget_set_hexpand(widget, TRUE);
            gtk_widget_set_halign(widget, GTK_ALIGN_FILL);
            g_signal_connect(widget, "clicked",
                             G_CALLBACK(rotctrl_preset_create_cb), ctrl);
        }
        else
        {
            widget = gtk_label_new("");
        }

        gtk_grid_attach(GTK_GRID(ctrl->preset_grid),
                        widget,
                        (gint) col, (gint) row, 1, 1);
    }

    gtk_widget_show_all(ctrl->preset_grid);
}

static void rotctrl_preset_create_cb(GtkButton *button, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);

    (void)button;
    rotctrl_preset_dialog_open(ctrl, -1);
}

static void rotctrl_preset_edit_cb(GtkButton *button, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);
    gint index = GPOINTER_TO_INT(
        g_object_get_data(G_OBJECT(button), "preset-index"));

    rotctrl_preset_dialog_open(ctrl, index);
}

static void rotctrl_preset_activate_cb(GtkButton *button, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);
    gint index = GPOINTER_TO_INT(
        g_object_get_data(G_OBJECT(button), "preset-index"));

    rotctrl_preset_move(ctrl, index);
}

/* Create preset position widgets */
static GtkWidget *create_cal_widgets(GtkRotCtrl * ctrl)
{
    GtkWidget *frame, *preset_grid;

    frame = gtk_frame_new(_("Preset positions"));

    preset_grid = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(preset_grid), 5);
    gtk_grid_set_column_spacing(GTK_GRID(preset_grid), 5);
    gtk_grid_set_row_spacing(GTK_GRID(preset_grid), 5);
    gtk_grid_set_row_homogeneous(GTK_GRID(preset_grid), TRUE);
    gtk_grid_set_column_homogeneous(GTK_GRID(preset_grid), TRUE);
    ctrl->preset_grid = preset_grid;
    gtk_container_add(GTK_CONTAINER(frame), preset_grid);
    rotctrl_presets_refresh(ctrl);

    return frame;
}

/* Create target widgets */
static GtkWidget *create_plot_widget(GtkRotCtrl * ctrl)
{
    GtkWidget      *frame;
    GtkWidget      *overlay;
    GtkWidget      *detach_button;

    ctrl->plot = gtk_polar_plot_new(ctrl->qth, ctrl->pass);

    overlay = gtk_overlay_new();
    gtk_widget_set_hexpand(overlay, TRUE);
    gtk_widget_set_vexpand(overlay, TRUE);
    gtk_container_add(GTK_CONTAINER(overlay), ctrl->plot);

    detach_button = gtk_button_new_with_label(_("Detach"));
    gtk_widget_set_halign(detach_button, GTK_ALIGN_END);
    gtk_widget_set_valign(detach_button, GTK_ALIGN_START);
    gtk_widget_set_margin_top(detach_button, 6);
    gtk_widget_set_margin_end(detach_button, 6);
    gtk_overlay_add_overlay(GTK_OVERLAY(overlay), detach_button);
    g_signal_connect(detach_button, "clicked",
                     G_CALLBACK(rotctrl_detach_plot_cb), ctrl);

    frame = gtk_frame_new(NULL);
    gtk_container_add(GTK_CONTAINER(frame), overlay);

    return frame;
}

/** Copy satellite from hash table to singly linked list. */
static void rotctrl_free_sat_list_entry(gpointer data, gpointer user_data)
{
    (void)user_data;
    gtk_sat_data_free_sat(SAT(data));
}

static void store_sats(gpointer key, gpointer value, gpointer user_data)
{
    GtkRotCtrl     *ctrl = GTK_ROT_CTRL(user_data);
    sat_t          *sat = SAT(value);
    sat_t          *sat_copy = NULL;

    (void)key;                  /* avoid unused variable warning */

    if (ctrl == NULL || sat == NULL)
        return;

    sat_copy = g_new0(sat_t, 1);
    gtk_sat_data_copy_sat(sat, sat_copy, ctrl->qth);
    if (sat_copy->nickname == NULL && sat_copy->name != NULL)
        sat_copy->nickname = g_strdup(sat_copy->name);

    ctrl->sats = g_slist_insert_sorted(ctrl->sats, sat_copy,
                                       (GCompareFunc)sat_name_compare);
}

/** Check that we have at least one .rot file */
static gboolean have_conf(void)
{
    GDir           *dir = NULL; /* directory handle */
    GError         *error = NULL;       /* error flag and info */
    gchar          *dirname;    /* directory name */
    const gchar    *filename;   /* file name */
    gint            i = 0;

    /* open configuration directory */
    dirname = get_hwconf_dir();

    dir = g_dir_open(dirname, 0, &error);
    if (dir)
    {
        /* read each .rot file */
        while ((filename = g_dir_read_name(dir)))
        {
            if (g_str_has_suffix(filename, ".rot"))
            {
                i++;
                /*once we have one we need nothing else */
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
    if (dir)
        g_dir_close(dir);

    return (i > 0) ? TRUE : FALSE;
}

static void rotctld_selftest(GtkRotCtrl *ctrl)
{
    gchar cmd[64];
    gboolean ok = TRUE;

    if (ctrl == NULL)
        return;

    format_rotctld_setpos(ctrl, 10.0, 20.0, NULL, NULL, cmd, sizeof(cmd));
    if (g_strcmp0(cmd, "P 10.00 20.00\n") != 0)
        ok = FALSE;

    {
        gboolean prev_valid = FALSE;
        gdouble prev_az_min = 0.0;
        gdouble prev_az_max = 0.0;
        gdouble prev_el_min = 0.0;
        gdouble prev_el_max = 0.0;

        g_mutex_lock(&ctrl->client.mutex);
        prev_valid = ctrl->client.limits_valid;
        prev_az_min = ctrl->client.az_min;
        prev_az_max = ctrl->client.az_max;
        prev_el_min = ctrl->client.el_min;
        prev_el_max = ctrl->client.el_max;
        ctrl->client.limits_valid = TRUE;
        ctrl->client.az_min = -180.0;
        ctrl->client.az_max = 450.0;
        ctrl->client.el_min = ROTCTRL_DEFAULT_MIN_EL;
        ctrl->client.el_max = ROTCTRL_DEFAULT_MAX_EL;
        g_mutex_unlock(&ctrl->client.mutex);

        format_rotctld_setpos(ctrl, 540.0, 190.0, NULL, NULL, cmd, sizeof(cmd));

        g_mutex_lock(&ctrl->client.mutex);
        ctrl->client.limits_valid = prev_valid;
        ctrl->client.az_min = prev_az_min;
        ctrl->client.az_max = prev_az_max;
        ctrl->client.el_min = prev_el_min;
        ctrl->client.el_max = prev_el_max;
        g_mutex_unlock(&ctrl->client.mutex);
    }

    {
        gdouble expect_el = ROTCTRL_DEFAULT_MAX_EL;
        if (ctrl->conf)
            expect_el = MIN(expect_el, ctrl->conf->maxel);
        gchar expected[64];
        g_snprintf(expected, sizeof(expected), "P 180.00 %.2f\n", expect_el);
        if (g_strcmp0(cmd, expected) != 0)
            ok = FALSE;
    }

    {
        gboolean prev_valid = FALSE;
        gdouble prev_az_min = 0.0;
        gdouble prev_az_max = 0.0;
        gdouble prev_el_min = 0.0;
        gdouble prev_el_max = 0.0;
        gdouble conf_prev_minaz = 0.0;
        gdouble conf_prev_maxaz = 360.0;
        rot_az_type_t conf_prev_type = ROT_AZ_TYPE_360;
        gboolean conf_restore = (ctrl->conf != NULL);

        if (conf_restore)
        {
            conf_prev_minaz = ctrl->conf->minaz;
            conf_prev_maxaz = ctrl->conf->maxaz;
            conf_prev_type = ctrl->conf->aztype;
            ctrl->conf->minaz = 0.0;
            ctrl->conf->maxaz = 360.0;
            ctrl->conf->aztype = ROT_AZ_TYPE_360;
        }

        g_mutex_lock(&ctrl->client.mutex);
        prev_valid = ctrl->client.limits_valid;
        prev_az_min = ctrl->client.az_min;
        prev_az_max = ctrl->client.az_max;
        prev_el_min = ctrl->client.el_min;
        prev_el_max = ctrl->client.el_max;
        ctrl->client.limits_valid = TRUE;
        ctrl->client.az_min = -180.0;
        ctrl->client.az_max = 180.0;
        ctrl->client.el_min = ROTCTRL_DEFAULT_MIN_EL;
        ctrl->client.el_max = ROTCTRL_DEFAULT_MAX_EL;
        g_mutex_unlock(&ctrl->client.mutex);

        format_rotctld_setpos(ctrl, 350.0, 20.0, NULL, NULL, cmd, sizeof(cmd));

        if (g_strcmp0(cmd, "P 350.00 20.00\n") != 0)
            ok = FALSE;

        g_mutex_lock(&ctrl->client.mutex);
        ctrl->client.limits_valid = prev_valid;
        ctrl->client.az_min = prev_az_min;
        ctrl->client.az_max = prev_az_max;
        ctrl->client.el_min = prev_el_min;
        ctrl->client.el_max = prev_el_max;
        g_mutex_unlock(&ctrl->client.mutex);

        if (conf_restore)
        {
            ctrl->conf->minaz = conf_prev_minaz;
            ctrl->conf->maxaz = conf_prev_maxaz;
            ctrl->conf->aztype = conf_prev_type;
        }
    }

    sat_log_log(ok ? SAT_LOG_LEVEL_INFO : SAT_LOG_LEVEL_WARN,
                "rotctld selftest %s", ok ? "ok" : "failed");
}

static void rotctrl_error_gate_selftest(void)
{
    struct {
        double meas_abs;
        double desired_backend;
        AzSpan span_mode;
        gboolean span_extended;
        double expect_err;
    } cases[] = {
        { 10.0, 350.0, AZSPAN_360, FALSE, 20.0 },
        { 370.0, 350.0, AZSPAN_360, FALSE, 20.0 },
        { 450.0, 460.0, AZSPAN_360, TRUE, 10.0 }
    };
    const double tol = 1e-6;

    for (guint i = 0; i < G_N_ELEMENTS(cases); i++)
    {
        double err = rotctrl_az_abs_error(cases[i].meas_abs,
                                          cases[i].desired_backend,
                                          cases[i].span_mode,
                                          cases[i].span_extended);
        if (fabs(err - cases[i].expect_err) > tol)
        {
            sat_log_log(SAT_LOG_LEVEL_WARN,
                        "rotctrl error gate selftest mismatch case=%u meas=%.2f desired=%.2f "
                        "span_ext=%d expect=%.2f got=%.2f",
                        i,
                        cases[i].meas_abs,
                        cases[i].desired_backend,
                        cases[i].span_extended ? 1 : 0,
                        cases[i].expect_err,
                        err);
        }
        else
        {
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rotctrl error gate selftest ok case=%u err=%.2f",
                        i, err);
        }
    }
}

static void gtk_rot_ctrl_init(GtkRotCtrl * ctrl,
			      gpointer g_class)
{
    (void)g_class;

    ctrl->sats = NULL;
    ctrl->target = NULL;
    ctrl->pass = NULL;
    ctrl->qth = NULL;
    ctrl->module = NULL;
    ctrl->module_sats = NULL;
    ctrl->plot = NULL;
    ctrl->detached_plots = NULL;
    ctrl->axis_mode_combo = NULL;
    ctrl->wrap_mode_combo = NULL;
    ctrl->min_az_spin = NULL;
    ctrl->max_az_spin = NULL;
    ctrl->min_el_spin = NULL;
    ctrl->max_el_spin = NULL;
    ctrl->az_endstop_spin = NULL;
    ctrl->preset_grid = NULL;
    ctrl->freeze = NULL;
    ctrl->preset_count = 0;

    ctrl->tracking = FALSE;
    ctrl->tracking_active = FALSE;
    ctrl->tracking_session_los = 0.0;
    ctrl->have_user_command = FALSE;
    ctrl->target_state = ROT_TARGET_STATE_IDLE;
    ctrl->target_state_since_us = 0;
    ctrl->target_valid_since_us = 0;
    ctrl->target_invalid_since_us = 0;
    ctrl->last_target_update_us = 0;
    ctrl->park_pending_since_us = 0;
    ctrl->park_requested = FALSE;
    ctrl->pretrack_enabled = TRUE;
    ctrl->pretrack_lookahead_sec = ROT_PRETRACK_LOOKAHEAD_SEC;
    ctrl->reacquire_hysteresis_sec = ROT_PRETRACK_REACQUIRE_SEC;
    ctrl->pretrack_immediate = TRUE;
    ctrl->pretrack_min_el = 1.0;
    ctrl->pretrack_target_az = 0.0;
    ctrl->pretrack_target_el = 0.0;
    ctrl->pretrack_aos_time = 0.0;
    ctrl->pretrack_last_update_us = 0;
    ctrl->pretrack_target_valid = FALSE;
    ctrl->pretrack_wait_log_us = 0;
    ctrl->pretrack_wrap_valid = FALSE;
    ctrl->pretrack_wrap_user_az = 0.0;
    ctrl->pretrack_wrap_raw_az = 0.0;
    ctrl->pretrack_wrap_k = 0;
    ctrl->seam_az360 = 0.0;
    ctrl->seam_valid = FALSE;
    ctrl->seam_crossing_active = FALSE;
    ctrl->seam_crossing_sent = FALSE;
    ctrl->seam_crossing_lane_valid = FALSE;
    ctrl->seam_crossing_lane_k = 0;
    ctrl->seam_crossing_target_az360 = 0.0;
    ctrl->seam_crossing_since_us = 0;
    ctrl->wrap_acquire_active = FALSE;
    ctrl->wrap_acquire_sent = FALSE;
    ctrl->wrap_acquire_target_backend = 0.0;
    ctrl->wrap_acquire_target_az360 = 0.0;
    ctrl->wrap_acquire_target_k = 0;
    ctrl->wrap_acquire_since_us = 0;
    ctrl->wrap_acquire_last_log_us = 0;
    ctrl->last_target_az360 = 0.0;
    ctrl->last_target_el = 0.0;
    ctrl->last_target_valid = FALSE;
    ctrl->pending_lane_backend = 0.0;
    ctrl->pending_lane_valid = FALSE;
    ctrl->pending_lane_since_us = 0;
    ctrl->locked_lane_k = 0;
    ctrl->locked_lane_valid = FALSE;
    ctrl->last_cmd_az360 = 0.0;
    ctrl->last_cmd_el = 0.0;
    ctrl->last_cmd_valid = FALSE;
    ctrl->setpoint_user_az = 0.0;
    ctrl->setpoint_user_el = 0.0;
    ctrl->setpoint_backend_az = 0.0;
    ctrl->setpoint_backend_el = 0.0;
    ctrl->setpoint_valid = FALSE;
    ctrl->force_next_send = FALSE;
    ctrl->plan_log_pending = FALSE;
    ctrl->plan_log_window_start = 0.0;
    ctrl->plan_log_window_end = 0.0;
    ctrl->plan_log_mode = ROT_PLAN_MODE_NORMAL;
    ctrl->plan_log_crosses_endstop = FALSE;
    ctrl->last_tracking_log_us = 0;
    ctrl->last_tracking_log_mode = ROT_PLAN_MODE_NORMAL;
    ctrl->last_tracking_log_cross_endstop = FALSE;
    ctrl->last_tracking_log_entry_reason[0] = '\0';
    ctrl->diagnostics_logged_generation = 0;
    ctrl->monitor  = FALSE;
    ctrl->engaged = FALSE;
    ctrl->engage_pending = FALSE;
    ctrl->engage_generation = 0;
    ctrl->session_state = ROT_SESSION_DISCONNECTED;
    ctrl->comm_status = ROT_COMM_DISENGAGED;
    ctrl->ui_status = ROTOR_UI_STATUS_DISENGAGED;
    ctrl->ui_degraded_latched = FALSE;
    ctrl->ui_degraded_clear_hits = 0;
    ctrl->ui_hard_error = FALSE;
    ctrl->ui_hard_error_reason[0] = '\0';
    ctrl->ui_status_detail[0] = '\0';
    ctrl->ok_streak = 0;
    ctrl->bad_streak = 0;
    ctrl->timeout_streak = 0;
    ctrl->delay = 300;      /* default: 300 ms control cycle */
    ctrl->timerid = 0;
    ctrl->threshold = 1.0;  /* default: 1 degree error tolerance */
    ctrl->errcnt = 0;
    ctrl->cycle_perf_warned = FALSE;
    ctrl->conf = NULL;
    ctrl->term_view = gp_term_view_new(_("Follow tail"), TRUE, FALSE);
    ctrl->log_toggle = NULL;
    ctrl->log_closed_height = 0;
    ctrl->ui_updating = FALSE;
    ctrl->pending_ui_refresh_id = 0;
    ctrl->resize_idle_id = 0;
    ctrl->rotctld_mgr = NULL;
    ctrl->verbose_logging = FALSE;
    ctrl->selected_child_pid = -1;
    ctrl->selected_child_port = 0;
    ctrl->last_stop_pid = -1;
    ctrl->last_stop_reason = NULL;
    ctrl->last_stop_us = 0;
    ctrl->rotctld_probe_state = NULL;
    ctrl->rotctld_probe_id = 0;
    ctrl->last_hold_log_us = 0;
    ctrl->last_stale_check_log_us = 0;
    memset(&ctrl->pos_warn_rate, 0, sizeof(ctrl->pos_warn_rate));

    /* Offset defaults */
    ctrl->use_offset   = FALSE;
    ctrl->az_offset_deg = 0.0;
    ctrl->el_offset_deg = 0.0;
    ctrl->offset_check = NULL;
    ctrl->az_offset_spin = NULL;
    ctrl->el_offset_spin = NULL;
    ctrl->rotor_id = NULL;
    ctrl->calib = (RotorCalib){ 0 };
    ctrl->calibration_offset_az = 0.0;
    ctrl->calibration_offset_el = 0.0;
    ctrl->settings_window = NULL;
    ctrl->chk_calib_enabled = NULL;
    ctrl->calib_offsets_grid = NULL;
    ctrl->calib_autocal_button = NULL;
    ctrl->spin_az_off = NULL;
    ctrl->spin_el_off = NULL;
    ctrl->backend_setpos = NULL;
    ctrl->backend_getpos = NULL;
    rot_transform_snapshot_defaults(&ctrl->transform);
    g_mutex_init(&ctrl->transform_snapshot_mutex);
    rot_transform_snapshot_state_init(&ctrl->transform_snapshot);
    ctrl->transform_snapshot_version = 0;
    g_atomic_int_set(&ctrl->south_zero_cached, 0);
    ctrl->az_hold_active = FALSE;
    ctrl->az_hold_value = 0.0;
    ctrl->az_abs_cur = 0.0;
    ctrl->az_abs_last_cmd = 0.0;
    ctrl->last_meas_span_az = NAN;
    ctrl->last_cmd_time_us = 0;
    ctrl->span_mode = AZSPAN_360;
    ctrl->span_extended = FALSE;
    memset(&ctrl->safety, 0, sizeof(ctrl->safety));
    memset(&ctrl->policy, 0, sizeof(ctrl->policy));
    rot_tracking_policy_reset(&ctrl->track_policy);
    ctrl->last_debug_log_us = 0;
    ctrl->last_tick_log_us = 0;
    ctrl->axis_swap_warned = FALSE;
    memset(&ctrl->user_limits, 0, sizeof(ctrl->user_limits));
    memset(&ctrl->backend_limits, 0, sizeof(ctrl->backend_limits));
    ctrl->limits_logged = FALSE;
    ctrl->wrap_mismatch_logged = FALSE;
    memset(&ctrl->backend_clamp_rate, 0, sizeof(ctrl->backend_clamp_rate));
    memset(&ctrl->send_log_rate, 0, sizeof(ctrl->send_log_rate));
    memset(&ctrl->cmd_skip_rate, 0, sizeof(ctrl->cmd_skip_rate));
    memset(&ctrl->pos_stale_rate, 0, sizeof(ctrl->pos_stale_rate));
    memset(&ctrl->pos_warn_rate, 0, sizeof(ctrl->pos_warn_rate));
    ctrl->pos_stale_hits = 0;
    ctrl->pos_stale_active = FALSE;
    ctrl->pos_stale_hyst_active = FALSE;
    ctrl->pos_stale_ready_hits = 0;
    ctrl->stale_hold_active = FALSE;
    ctrl->stale_hold_since_us = 0;
    ctrl->stale_resume_since_us = 0;
    ctrl->stale_recovered_pulse = FALSE;
    ctrl->out_of_range = FALSE;
    ctrl->last_oob_log_us = 0;
    ctrl->last_oob_raw_az = 0.0;
    ctrl->last_oob_raw_el = 0.0;
    ctrl->last_oob_mapped_az = 0.0;
    ctrl->last_oob_mapped_el = 0.0;
    ctrl->manual_edit_until_us = 0;
    ctrl->last_manual_sync_log_us = 0;
    ctrl->manual_sync_pending = FALSE;
    ctrl->calibration_retry_id = 0;
    ctrl->calibration_pending = FALSE;
    ctrl->cal_active = FALSE;
    ctrl->calibration_active = FALSE;
    ctrl->cal_state = ROT_AUTOCAL_IDLE;
    ctrl->cal_did_setpos = FALSE;
    ctrl->cal_direct_active = FALSE;
    ctrl->cal_hold_active = FALSE;
    ctrl->cal_hold_since_us = 0;
    ctrl->calib_dialog = NULL;
    ctrl->calib_backup = (RotorCalib){ 0 };
    ctrl->calib_backup_valid = FALSE;
    ctrl->cal_start_us = 0;
    ctrl->cal_timeout_us = 0;
    memset(&ctrl->cal_direct, 0, sizeof(ctrl->cal_direct));
    ctrl->cal_stable_count = 0;
    ctrl->cal_arrive_count = 0;
    ctrl->cal_ready_count = 0;
    ctrl->cal_last_log_us = 0;
    ctrl->cal_settle_start_us = 0;
    ctrl->cal_settle_start_us = 0;
    ctrl->cal_window_count = 0;
    ctrl->cal_window_idx = 0;
    ctrl->motion_err_mag = 0.0;
    ctrl->motion_err_valid = FALSE;
    ctrl->motion_stall_count = 0;
    memset(&ctrl->trajectory_plan, 0, sizeof(ctrl->trajectory_plan));
    rot_plan_reset(&ctrl->trajectory_plan);
    /* Reserved flag; keep FALSE (no special SEND-ONLY mode). */
    ctrl->send_only_mode = FALSE;

    g_mutex_init(&ctrl->client.mutex);
    ctrl->client.thread = NULL;
    ctrl->client.socket = -1;
    ctrl->client.conn_id = 0;
    ctrl->client.thread_generation = 0;
    ctrl->client.running = FALSE;
    ctrl->client.new_trg = FALSE;
    ctrl->client.use_setpos = FALSE;
    ctrl->client.apply_calib = FALSE;
    ctrl->client.allow_send_no_pos = FALSE;
    ctrl->client.stop_requested = 0;
    ctrl->client.send_quit = FALSE;
    ctrl->client.thread_done = TRUE;
    ctrl->client.azi_out = 0.0;
    ctrl->client.ele_out = 0.0;
    ctrl->client.raw_azi_out = 0.0;
    ctrl->client.raw_ele_out = 0.0;
    ctrl->client.azi_in = 0.0;
    ctrl->client.ele_in = 0.0;
    ctrl->client.azi_az360 = 0.0;
    ctrl->client.azi_mech_in = 0.0;
    ctrl->client.ele_mech_in = 0.0;
    ctrl->client.io_error = FALSE;
    ctrl->client.cmd_rejected = FALSE;
    ctrl->client.reject_backoff_until_us = 0;
    ctrl->client.reject_backoff_sec = 0.5;
    ctrl->client.reject_backoff_log_us = 0;
    ctrl->client.limits_valid = FALSE;
    ctrl->client.south_zero = FALSE;
    g_atomic_int_set(&ctrl->south_zero_cached, 0);
    ctrl->client.daemon_ok = FALSE;
    ctrl->client.last_rtt_us = 0;
    ctrl->client.last_cmd_us = 0;
    ctrl->client.last_cmd_ok_az = 0.0;
    ctrl->client.last_cmd_ok_el = 0.0;
    ctrl->client.last_cmd_ok_us = 0;
    ctrl->client.pos_valid = FALSE;
    ctrl->client.pos_invalid = FALSE;
    ctrl->client.pos_unknown = FALSE;
    ctrl->client.pos_cmd_ok = FALSE;
    ctrl->client.set_pos_ok = FALSE;
    ctrl->client.handshake_pos_ok = FALSE;
    ctrl->client.first_pos_deadline_us = 0;
    ctrl->client.last_pos_us = 0;
    ctrl->client.last_pos_attempt_us = 0;
    ctrl->client.last_set_attempt_us = 0;
    ctrl->client.transport_backoff_until_us = 0;
    ctrl->client.transport_backoff_sec = 0.5;
    ctrl->client.io_error_reason[0] = '\0';
    ctrl->client.last_pos_error[0] = '\0';
    ctrl->client.last_io_ok_us = 0;
    ctrl->client.consecutive_failures = 0;
    ctrl->client.last_failure_log_us = 0;
    ctrl->client.backend_ioerr_count = 0;
    ctrl->client.backend_ioerr_first_us = 0;
    ctrl->client.backend_ioerr_last_log_us = 0;
    ctrl->client.backend_ioerr_disengage_pending = FALSE;
    ctrl->client.stop_pending = FALSE;
    ctrl->client.rxbuf = NULL;

    rot_transform_update(ctrl);

    if (g_getenv("GPREDICT_ROT_PLAN_TEST"))
        rot_plan_debug_harness();
    if (g_getenv("GPREDICT_ROTCTLD_SELFTEST"))
        rotctld_selftest(ctrl);
    if (g_getenv("GPREDICT_ROT_ERRTEST"))
        rotctrl_error_gate_selftest();

    gp_rot_angle_selfcheck();
}

static void gtk_rot_ctrl_destroy(GtkWidget * widget)
{
    GtkRotCtrl     *ctrl = GTK_ROT_CTRL(widget);

    /* stop timer */
    if (ctrl->timerid > 0) {
        g_source_remove(ctrl->timerid);
        ctrl->timerid = 0;
    }
    if (ctrl->pending_ui_refresh_id != 0)
    {
        g_source_remove(ctrl->pending_ui_refresh_id);
        ctrl->pending_ui_refresh_id = 0;
    }

    while (ctrl->detached_plots != NULL)
    {
        RotDetachedPlot *entry = ctrl->detached_plots->data;

        if (entry != NULL && entry->window != NULL)
            gtk_widget_destroy(entry->window);
        else
            ctrl->detached_plots =
                g_list_delete_link(ctrl->detached_plots,
                                   ctrl->detached_plots);
    }
    ctrl->detached_plots = NULL;

    /* free configuration */
    if (ctrl->conf != NULL)
    {
        rotor_conf_save(ctrl->conf);
        g_free(ctrl->conf->name);
        g_free(ctrl->conf->host);
        g_free(ctrl->conf->device);
        g_free(ctrl->conf->device_manual);
        g_free(ctrl->conf->last_good_device);
        g_free(ctrl->conf);
        ctrl->conf = NULL;
    }

    rotctrl_presets_clear(ctrl);

    g_free(ctrl->rotor_id);
    ctrl->rotor_id = NULL;

    /* stop client thread */
    if (ctrl->client.thread)
    {
        /* Signal the thread to stop, then wait for it */
        rotctld_request_thread_stop(ctrl, TRUE);
        (void)rotctld_collect_client_thread(ctrl, TRUE, "destroy");
    }
    if (ctrl->client.rxbuf != NULL)
    {
        g_string_free(ctrl->client.rxbuf, TRUE);
        ctrl->client.rxbuf = NULL;
    }
    if (ctrl->client.client != NULL)
        rotctld_client_free(&ctrl->client.client);

    rotctld_probe_cancel(ctrl);
    rotctld_process_stop(ctrl);
    g_free(ctrl->last_stop_reason);
    ctrl->last_stop_reason = NULL;

    if (ctrl->term_view != NULL)
    {
        gp_term_view_free(ctrl->term_view);
        ctrl->term_view = NULL;
    }
    if (ctrl->LockBut != NULL)
    {
        g_signal_handlers_block_by_func(ctrl->LockBut,
                                        (gpointer)rot_locked_cb, ctrl);
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->LockBut), FALSE);
        g_signal_handlers_unblock_by_func(ctrl->LockBut,
                                          (gpointer)rot_locked_cb, ctrl);
        rotctrl_reset_lock_button_visual(ctrl);
    }
    ctrl->log_toggle = NULL;
    if (ctrl->resize_idle_id != 0)
    {
        g_source_remove(ctrl->resize_idle_id);
        ctrl->resize_idle_id = 0;
    }

    rot_plan_reset(&ctrl->trajectory_plan);

    if (ctrl->sats != NULL)
    {
        g_slist_foreach(ctrl->sats, rotctrl_free_sat_list_entry, NULL);
        g_slist_free(ctrl->sats);
        ctrl->sats = NULL;
        ctrl->target = NULL;
    }
    ctrl->module = NULL;
    ctrl->module_sats = NULL;

    (*GTK_WIDGET_CLASS(parent_class)->destroy) (widget);
}

static void gtk_rot_ctrl_class_init(GtkRotCtrlClass * class,
				    gpointer class_data)
{
    GtkWidgetClass *widget_class = (GtkWidgetClass *) class;

    (void)class_data;

    widget_class->destroy = gtk_rot_ctrl_destroy;
    parent_class = g_type_class_peek_parent(class);
}

GType gtk_rot_ctrl_get_type(void)
{
    static GType    gtk_rot_ctrl_type = 0;

    if (!gtk_rot_ctrl_type)
    {
        static const GTypeInfo gtk_rot_ctrl_info = {
            sizeof(GtkRotCtrlClass),
            NULL,               /* base_init */
            NULL,               /* base_finalize */
            (GClassInitFunc) gtk_rot_ctrl_class_init,
            NULL,               /* class_finalize */
            NULL,               /* class_data */
            sizeof(GtkRotCtrl),
            5,                  /* n_preallocs */
            (GInstanceInitFunc) gtk_rot_ctrl_init,
            NULL
        };

        gtk_rot_ctrl_type = g_type_register_static(GTK_TYPE_BOX,
                                                   "GtkRotCtrl",
                                                   &gtk_rot_ctrl_info, 0);
    }

    return gtk_rot_ctrl_type;
}

GtkWidget      *gtk_rot_ctrl_new(GtkSatModule * module)
{
    GtkRotCtrl     *rot_ctrl;
    GtkWidget      *table;
    GtkWidget      *plot_frame;
    GtkWidget      *main_grid;
    GtkWidget      *outer;
    GtkWidget      *spacer_top;
    GtkWidget      *spacer_bottom;
    GtkWidget      *az_frame;
    GtkWidget      *el_frame;
    GtkWidget      *az_el_row;
    GtkWidget      *target_frame;
    GtkWidget      *conf_frame;
    GtkWidget      *calib_frame;
    GtkWidget      *preset_frame;
    GtkWidget      *term_widget;
    GtkWidget      *banner_widget;

    /* check that we have rot conf */
    if (!have_conf())
        return NULL;

    rot_ctrl = GTK_ROT_CTRL(g_object_new(GTK_TYPE_ROT_CTRL, NULL));

    gp_ui_quarantine_install(gtk_widget_get_toplevel(GTK_WIDGET(rot_ctrl)));

    /* store QTH */
    rot_ctrl->qth = module->qth;
    rot_ctrl->module = module;
    rot_ctrl->module_sats = module->satellites;

    /* store satellites */
    g_hash_table_foreach(module->satellites, store_sats, rot_ctrl);

    /* store current time (don't know if real or simulated) */
    rot_ctrl->t = module->tmgCdnum;

    /* create contents */
    table = gtk_grid_new();
    gtk_grid_set_column_homogeneous(GTK_GRID(table), FALSE);
    gtk_grid_set_row_homogeneous(GTK_GRID(table), FALSE);
    gtk_grid_set_row_spacing(GTK_GRID(table), ROTCTRL_PANEL_SPACING);
    gtk_grid_set_column_spacing(GTK_GRID(table), ROTCTRL_PANEL_SPACING);
    gtk_container_set_border_width(GTK_CONTAINER(table), 0);

    az_frame = create_az_widgets(rot_ctrl);
    el_frame = create_el_widgets(rot_ctrl);
    target_frame = create_target_widgets(rot_ctrl);
    conf_frame = create_conf_widgets(rot_ctrl);
    calib_frame = create_calibration_widgets(rot_ctrl);
    preset_frame = create_cal_widgets(rot_ctrl);
    term_widget = gp_term_view_get_widget(rot_ctrl->term_view);
    banner_widget = create_aoslos_banner_widgets(rot_ctrl);

    gtk_widget_set_hexpand(az_frame, TRUE);
    gtk_widget_set_hexpand(el_frame, TRUE);
    gtk_widget_set_hexpand(target_frame, FALSE);
    gtk_widget_set_halign(target_frame, GTK_ALIGN_START);
    gtk_widget_set_hexpand(conf_frame, TRUE);
    gtk_widget_set_halign(conf_frame, GTK_ALIGN_FILL);
    gtk_widget_set_hexpand(calib_frame, FALSE);
    gtk_widget_set_halign(calib_frame, GTK_ALIGN_START);
    gtk_widget_set_vexpand(calib_frame, TRUE);
    gtk_widget_set_valign(calib_frame, GTK_ALIGN_FILL);
    gtk_widget_set_hexpand(preset_frame, TRUE);
    gtk_widget_set_hexpand(term_widget, TRUE);
    gtk_widget_set_hexpand(banner_widget, TRUE);

    gtk_widget_set_vexpand(preset_frame, FALSE);
    gtk_widget_set_valign(preset_frame, GTK_ALIGN_START);

    az_el_row = gtk_grid_new();
    gtk_grid_set_column_spacing(GTK_GRID(az_el_row), ROTCTRL_PANEL_SPACING);
    gtk_grid_set_column_homogeneous(GTK_GRID(az_el_row), TRUE);
    gtk_widget_set_hexpand(az_el_row, TRUE);
    gtk_widget_set_vexpand(az_el_row, FALSE);
    gtk_grid_attach(GTK_GRID(az_el_row), az_frame, 0, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(az_el_row), el_frame, 1, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(table), az_el_row, 0, 0, 3, 1);
    gtk_grid_attach(GTK_GRID(table), target_frame, 0, 1, 1, 1);
    gtk_grid_attach(GTK_GRID(table), conf_frame, 1, 1, 1, 1);
    gtk_grid_attach(GTK_GRID(table), calib_frame, 2, 1, 1, 1);
    gtk_grid_attach(GTK_GRID(table), preset_frame, 0, 2, 3, 1);
    gtk_grid_attach(GTK_GRID(table), term_widget, 0, 3, 3, 1);
    gtk_grid_attach(GTK_GRID(table), banner_widget, 0, 4, 3, 1);

    plot_frame = create_plot_widget(rot_ctrl);
    gtk_widget_set_hexpand(plot_frame, TRUE);
    gtk_widget_set_vexpand(plot_frame, TRUE);

    main_grid = gtk_grid_new();
    gtk_grid_set_column_spacing(GTK_GRID(main_grid), 10);
    gtk_grid_set_row_spacing(GTK_GRID(main_grid), 10);
    gtk_container_set_border_width(GTK_CONTAINER(main_grid), 0);
    gtk_widget_set_hexpand(main_grid, TRUE);
    gtk_widget_set_vexpand(main_grid, FALSE);
    gtk_grid_attach(GTK_GRID(main_grid), plot_frame, 0, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(main_grid), table, 1, 0, 1, 1);

    gtk_widget_set_hexpand(table, TRUE);
    gtk_widget_set_vexpand(table, FALSE);
    gtk_widget_set_valign(table, GTK_ALIGN_FILL);

    outer = gtk_grid_new();
    gtk_widget_set_hexpand(outer, TRUE);
    gtk_widget_set_vexpand(outer, TRUE);
    spacer_top = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    spacer_bottom = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    gtk_widget_set_vexpand(spacer_top, TRUE);
    gtk_widget_set_vexpand(spacer_bottom, TRUE);
    gtk_grid_attach(GTK_GRID(outer), spacer_top, 0, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(outer), main_grid, 0, 1, 1, 1);
    gtk_grid_attach(GTK_GRID(outer), spacer_bottom, 0, 2, 1, 1);

    gtk_box_pack_start(GTK_BOX(rot_ctrl), outer, TRUE, TRUE, 5);
    gtk_container_set_border_width(GTK_CONTAINER(rot_ctrl), 5);

    /* load initial rotator configuration */
    rot_selected_cb(GTK_COMBO_BOX(rot_ctrl->DevSel), rot_ctrl);

    /* start the control loop timer so we actually send commands */
    if (rot_ctrl->timerid == 0) {
        rot_ctrl->timerid = g_timeout_add(rot_ctrl->delay,
                                          rot_ctrl_timeout_cb,
                                          rot_ctrl);
    }

    return GTK_WIDGET(rot_ctrl);
}

static gchar *rotctrl_build_rotor_id(GtkRotCtrl *ctrl)
{
    const gchar *device = NULL;
    gint baud = 0;

    if (ctrl == NULL || ctrl->conf == NULL)
        return NULL;

    if (ctrl->conf->device_manual && *ctrl->conf->device_manual)
        device = ctrl->conf->device_manual;
    else if (ctrl->conf->device && *ctrl->conf->device)
        device = ctrl->conf->device;
    else if (ctrl->conf->last_good_device && *ctrl->conf->last_good_device)
        device = ctrl->conf->last_good_device;

    baud = (ctrl->conf->baud > 0)
           ? ctrl->conf->baud
           : rot_protocol_default_baud(ctrl->conf->protocol);

    if (device && *device)
        return g_strdup_printf("hamlib:%s@%d", device, baud);

    return g_strdup_printf("rotctld:%s:%d",
                           ctrl->conf->host ? ctrl->conf->host : "unknown",
                           ctrl->conf->port);
}

static bool rotctrl_calib_ensure_id(GtkRotCtrl *ctrl)
{
    gchar *rotor_id = NULL;

    if (ctrl == NULL)
        return false;

    if (ctrl->rotor_id && *ctrl->rotor_id)
        return true;

    rotor_id = rotctrl_build_rotor_id(ctrl);
    if (rotor_id == NULL)
        return false;

    g_free(ctrl->rotor_id);
    ctrl->rotor_id = rotor_id;
    return true;
}

static void rotctrl_calib_update_widgets(GtkRotCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    if (ctrl->offset_check)
    {
        g_signal_handlers_block_by_func(ctrl->offset_check,
                                        (gpointer)G_CALLBACK(offset_toggle_cb),
                                        ctrl);
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->offset_check),
                                     ctrl->use_offset);
        g_signal_handlers_unblock_by_func(ctrl->offset_check,
                                          (gpointer)G_CALLBACK(offset_toggle_cb),
                                          ctrl);
    }

    if (ctrl->az_offset_spin)
    {
        g_signal_handlers_block_by_func(ctrl->az_offset_spin,
                                        (gpointer)G_CALLBACK(az_offset_changed_cb),
                                        ctrl);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ctrl->az_offset_spin),
                                  ctrl->az_offset_deg);
        g_signal_handlers_unblock_by_func(ctrl->az_offset_spin,
                                          (gpointer)G_CALLBACK(az_offset_changed_cb),
                                          ctrl);
    }

    if (ctrl->el_offset_spin)
    {
        g_signal_handlers_block_by_func(ctrl->el_offset_spin,
                                        (gpointer)G_CALLBACK(el_offset_changed_cb),
                                        ctrl);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ctrl->el_offset_spin),
                                  ctrl->el_offset_deg);
        g_signal_handlers_unblock_by_func(ctrl->el_offset_spin,
                                          (gpointer)G_CALLBACK(el_offset_changed_cb),
                                          ctrl);
    }

    rotctrl_calib_update_offset_sensitivity(ctrl);
}

static void rotctrl_calib_update_offset_sensitivity(GtkRotCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    if (ctrl->calib_offsets_grid)
        gtk_widget_set_sensitive(ctrl->calib_offsets_grid, ctrl->use_offset);
}

static gboolean G_GNUC_UNUSED rotctrl_calib_read_mech_pos(GtkRotCtrl *ctrl,
                                            gdouble *mech_az,
                                            gdouble *mech_el)
{
    if (ctrl == NULL || mech_az == NULL || mech_el == NULL)
        return FALSE;

    g_mutex_lock(&ctrl->client.mutex);
    if (!ctrl->client.pos_valid)
    {
        g_mutex_unlock(&ctrl->client.mutex);
        return FALSE;
    }
    *mech_az = ctrl->client.azi_mech_in;
    *mech_el = ctrl->client.ele_mech_in;
    g_mutex_unlock(&ctrl->client.mutex);
    return TRUE;
}

static void rotctrl_calib_reload(GtkRotCtrl *ctrl)
{
    gchar *rotor_id = NULL;

    if (ctrl == NULL)
        return;

    rotor_id = rotctrl_build_rotor_id(ctrl);
    if (rotor_id == NULL)
    {
        g_clear_pointer(&ctrl->rotor_id, g_free);
        ctrl->calib = (RotorCalib){ 0 };
        ctrl->calibration_offset_az = 0.0;
        ctrl->calibration_offset_el = 0.0;
        rotctrl_calib_update_widgets(ctrl);
        return;
    }

    if (ctrl->rotor_id && g_strcmp0(ctrl->rotor_id, rotor_id) == 0)
    {
        g_free(rotor_id);
        rotctrl_calib_update_widgets(ctrl);
        return;
    }

    g_free(ctrl->rotor_id);
    ctrl->rotor_id = rotor_id;
    (void)calib_load(ctrl->rotor_id, &ctrl->calib);
    ctrl->calibration_offset_az = ctrl->calib.az_offset_deg;
    ctrl->calibration_offset_el = ctrl->calib.el_offset_deg;
    rotctrl_calib_update_widgets(ctrl);
}

static void rotctrl_calib_apply_send(GtkRotCtrl *ctrl,
                                     gdouble world_az,
                                     gdouble world_el,
                                     gdouble *mech_az,
                                     gdouble *mech_el)
{
    gdouble az_canon = wrap360(world_az);
    gdouble el = world_el;
    gdouble az_mech = az_canon;
    gdouble el_mech = el;

    if (ctrl != NULL && !ctrl->calibration_active)
    {
        az_mech = wrap360(az_mech - ctrl->calibration_offset_az);
        el_mech -= ctrl->calibration_offset_el;
    }

    if (ctrl != NULL)
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "[calib] send world=(%.2f,%.2f) -> mech=(%.2f,%.2f) "
                    "off=(az=%.2f,el=%.2f) en=%d",
                    az_canon, el,
                    az_mech, el_mech,
                    ctrl->calibration_offset_az,
                    ctrl->calibration_offset_el,
                    1);

    if (mech_az)
        *mech_az = az_mech;
    if (mech_el)
        *mech_el = el_mech;
}

static void G_GNUC_UNUSED rotctrl_calib_apply_read(GtkRotCtrl *ctrl,
                                     gdouble mech_az,
                                     gdouble mech_el,
                                     gdouble *world_az,
                                     gdouble *world_el,
                                     gdouble *mech_az_out,
                                     gdouble *mech_el_out)
{
    gdouble az_canon = wrap360(mech_az);
    gdouble el = mech_el;
    gdouble az_world = az_canon;
    gdouble el_world = el;

    if (ctrl != NULL && !ctrl->calibration_active)
    {
        az_world = wrap360(az_world + ctrl->calibration_offset_az);
        el_world += ctrl->calibration_offset_el;
    }

    if (ctrl != NULL)
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "[calib] read mech=(%.2f,%.2f) -> world=(%.2f,%.2f) "
                    "off=(az=%.2f,el=%.2f) en=%d",
                    az_canon, el,
                    az_world, el_world,
                    ctrl->calibration_offset_az,
                    ctrl->calibration_offset_el,
                    1);

    if (world_az)
        *world_az = az_world;
    if (world_el)
        *world_el = el_world;
    if (mech_az_out)
        *mech_az_out = az_canon;
    if (mech_el_out)
        *mech_el_out = el;
}

static gdouble rotctrl_calib_uncertainty_deg(GtkRotCtrl *ctrl)
{
    gdouble eps = 0.0;

    if (ctrl != NULL)
        eps = rotctrl_angle_epsilon(ctrl);

    if (eps <= 0.0)
        eps = 0.5;

    return eps;
}

static void rotctrl_set_cal_hold(GtkRotCtrl *ctrl,
                                 gboolean active,
                                 const gchar *reason)
{
    if (ctrl == NULL)
        return;

    if (ctrl->cal_hold_active == active)
        return;

    ctrl->cal_hold_active = active;
    ctrl->cal_hold_since_us = active ? g_get_monotonic_time() : 0;

    sat_log_log(SAT_LOG_LEVEL_INFO,
                "CAL_HOLD %s reason=%s",
                active ? "ON" : "OFF",
                reason ? reason : "none");
    rot_term_log(ctrl, "gpredict:state",
                 "CAL_HOLD %s reason=%s",
                 active ? "ON" : "OFF",
                 reason ? reason : "none");
}

static void rotctrl_autocal_close_dialog(GtkRotCtrl *ctrl)
{
    if (ctrl == NULL || ctrl->calib_dialog == NULL)
        return;

    GtkWidget *dialog = ctrl->calib_dialog;
    ctrl->calib_dialog = NULL;
    gtk_widget_destroy(dialog);
}

static gdouble rotctrl_autocal_normalize_zero_seam(gdouble az)
{
    gdouble norm = gp_norm360(az);

    if (!isfinite(norm))
        return az;

    if (norm <= ROT_AUTOCAL_SEAM_TOL_DEG ||
        norm >= (360.0 - ROT_AUTOCAL_SEAM_TOL_DEG))
        return 0.0;

    return norm;
}

static gboolean G_GNUC_UNUSED rotctrl_autocal_window_stats(const GtkRotCtrl *ctrl,
                                                           gdouble *spread_az,
                                                           gdouble *spread_el)
{
    if (spread_az)
        *spread_az = 0.0;
    if (spread_el)
        *spread_el = 0.0;

    if (ctrl == NULL)
        return FALSE;

    guint count = MIN(ctrl->cal_window_count, ROT_AUTOCAL_STABLE_WINDOW);
    if (count == 0)
        return FALSE;

    gdouble az_min = ctrl->cal_window_az[0];
    gdouble az_max = ctrl->cal_window_az[0];
    gdouble el_min = ctrl->cal_window_el[0];
    gdouble el_max = ctrl->cal_window_el[0];

    for (guint i = 1; i < ROT_AUTOCAL_STABLE_WINDOW; i++)
    {
        az_min = MIN(az_min, ctrl->cal_window_az[i]);
        az_max = MAX(az_max, ctrl->cal_window_az[i]);
        el_min = MIN(el_min, ctrl->cal_window_el[i]);
        el_max = MAX(el_max, ctrl->cal_window_el[i]);
    }

    if (spread_az)
        *spread_az = az_max - az_min;
    if (spread_el)
        *spread_el = el_max - el_min;

    return TRUE;
}

static gboolean rotctrl_autocal_estimate(const GtkRotCtrl *ctrl,
                                         gdouble *mean_az,
                                         gdouble *spread_az,
                                         gdouble *mean_el,
                                         gdouble *dev_el)
{
    guint count = 0;

    if (mean_az)
        *mean_az = 0.0;
    if (spread_az)
        *spread_az = 0.0;
    if (mean_el)
        *mean_el = 0.0;
    if (dev_el)
        *dev_el = 0.0;

    if (ctrl == NULL)
        return FALSE;

    count = MIN(ctrl->cal_window_count, ROT_AUTOCAL_STABLE_WINDOW);
    if (count == 0)
        return FALSE;

    gdouble sum_sin = 0.0;
    gdouble sum_cos = 0.0;
    gdouble el_sum = 0.0;

    for (guint i = 0; i < count; i++)
    {
        gdouble az_deg = ctrl->cal_window_az[i];
        gdouble rad = az_deg * (G_PI / 180.0);
        sum_sin += sin(rad);
        sum_cos += cos(rad);
        el_sum += ctrl->cal_window_el[i];
    }

    gdouble mean_rad = atan2(sum_sin, sum_cos);
    gdouble mean_az_deg = wrap360(mean_rad * (180.0 / G_PI));
    gdouble mean_el_val = el_sum / (gdouble)count;
    gdouble max_az_dev = 0.0;
    gdouble max_el_dev = 0.0;

    for (guint i = 0; i < count; i++)
    {
        gdouble az_dev =
            fabs(shortest_az_delta(ctrl->cal_window_az[i], mean_az_deg));
        gdouble el_dev = fabs(ctrl->cal_window_el[i] - mean_el_val);
        if (az_dev > max_az_dev)
            max_az_dev = az_dev;
        if (el_dev > max_el_dev)
            max_el_dev = el_dev;
    }

    if (mean_az)
        *mean_az = mean_az_deg;
    if (spread_az)
        *spread_az = max_az_dev;
    if (mean_el)
        *mean_el = mean_el_val;
    if (dev_el)
        *dev_el = max_el_dev;

    return TRUE;
}

static gboolean rotctrl_autocal_apply_mech(GtkRotCtrl *ctrl,
                                           gdouble mech_az,
                                           gdouble mech_el,
                                           const gchar **reason_out)
{
    RotorCalib updated = { 0 };
    gdouble uncertainty = 0.0;

    if (ctrl == NULL)
    {
        if (reason_out)
            *reason_out = "no_ctrl";
        return FALSE;
    }

    if (!rotctrl_calib_ensure_id(ctrl))
    {
        if (reason_out)
            *reason_out = "no_rotor_id";
        return FALSE;
    }

    mech_az = rotctrl_autocal_normalize_zero_seam(mech_az);
    uncertainty = rotctrl_calib_uncertainty_deg(ctrl);
    updated = ctrl->calib;
    calib_apply_mech_zero(&updated, mech_az, mech_el, uncertainty);

    if (!calib_save(ctrl->rotor_id, &updated))
    {
        if (reason_out)
            *reason_out = "save_failed";
        return FALSE;
    }

    ctrl->calib = updated;
    ctrl->calibration_offset_az = updated.az_offset_deg;
    ctrl->calibration_offset_el = updated.el_offset_deg;
    return TRUE;
}

static void rotctrl_autocal_finish(GtkRotCtrl *ctrl,
                                   gboolean ok,
                                   const gchar *reason)
{
    if (ctrl == NULL)
        return;

    ctrl->cal_active = FALSE;
    ctrl->calibration_active = FALSE;
    ctrl->cal_direct_active = FALSE;
    ctrl->cal_state = ok ? ROT_AUTOCAL_DONE : ROT_AUTOCAL_FAIL;
    ctrl->cal_did_setpos = FALSE;
    ctrl->cal_timeout_us = 0;
    memset(&ctrl->cal_direct, 0, sizeof(ctrl->cal_direct));
    rotctrl_autocal_close_dialog(ctrl);
    if (ctrl->calib_autocal_button)
        gtk_widget_set_sensitive(ctrl->calib_autocal_button, TRUE);

    if (ok)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "autocal: success reason=%s",
                    reason ? reason : "ok");
        rot_term_log(ctrl, "gpredict:rx",
                     "autocal: success reason=%s",
                     reason ? reason : "ok");
    }
    else if (reason && *reason)
    {
        sat_log_log(SAT_LOG_LEVEL_WARN, "autocal failed reason=%s", reason);
        rot_term_log(ctrl, "gpredict:warn",
                     "autocal failed reason=%s", reason);
    }

    if (!ok && reason && g_strcmp0(reason, "cancel") != 0 &&
        g_strcmp0(reason, "save_failed") != 0)
    {
        const gchar *detail = _("Calibration failed. Check rotor feedback and try again.");
        if (g_strcmp0(reason, "timeout") == 0)
            detail = _("Calibration timed out. Check rotor feedback and try again.");
        else if (g_strcmp0(reason, "disengaged") == 0)
            detail = _("Calibration aborted: rotor disengaged.");
        else if (g_strcmp0(reason, "no_position") == 0)
            detail = _("Calibration failed: no position feedback.");
        else if (g_strcmp0(reason, "pos_invalid") == 0)
            detail = _("Calibration aborted: position invalid (RPRT -6).");
        else if (g_strcmp0(reason, "no_motion") == 0)
            detail = _("Calibration failed: no motion detected.");
        else if (g_strcmp0(reason, "apply_fail") == 0 ||
                 g_strcmp0(reason, "save_failed") == 0)
            detail = _("Calibration failed to save. Check config permissions.");
        rot_show_message(ctrl,
                         GTK_MESSAGE_ERROR,
                         _("Calibration"),
                         detail);
    }

    if (!ok && ctrl->calib_backup_valid)
    {
        ctrl->calib = ctrl->calib_backup;
        ctrl->calibration_offset_az = ctrl->calib.az_offset_deg;
        ctrl->calibration_offset_el = ctrl->calib.el_offset_deg;
    }
    ctrl->calib_backup_valid = FALSE;
}

static void rotctrl_autocal_set_ok_sensitive(GtkRotCtrl *ctrl, gboolean enabled)
{
    if (ctrl == NULL || ctrl->calib_dialog == NULL)
        return;

    gtk_dialog_set_response_sensitive(GTK_DIALOG(ctrl->calib_dialog),
                                      GTK_RESPONSE_OK,
                                      enabled);
}

static gboolean rotctrl_autocal_queue_move(GtkRotCtrl *ctrl,
                                           gdouble az,
                                           gdouble el)
{
    if (ctrl == NULL || ctrl->client.client == NULL)
        return FALSE;

    g_mutex_lock(&ctrl->client.mutex);
    ctrl->client.azi_out = az;
    ctrl->client.ele_out = el;
    ctrl->client.raw_azi_out = az;
    ctrl->client.raw_ele_out = el;
    ctrl->client.new_trg = TRUE;
    ctrl->client.desired_user_az = az;
    ctrl->client.desired_user_el = el;
    ctrl->client.desired_backend_az = az;
    ctrl->client.desired_backend_el = el;
    ctrl->client.desired_raw_az = az;
    ctrl->client.desired_raw_el = el;
    ctrl->client.desired_valid = TRUE;
    ctrl->client.desired_allow = TRUE;
    ctrl->client.desired_tracking = FALSE;
    ctrl->client.force_pending = TRUE;
    ctrl->client.desired_update_us = g_get_monotonic_time();
    ctrl->client.allow_send_no_pos = TRUE;
    ctrl->client.use_setpos = FALSE;
    ctrl->client.apply_calib = FALSE;
    g_mutex_unlock(&ctrl->client.mutex);

    sat_log_log(SAT_LOG_LEVEL_INFO,
                "autocal: issuing move to (%.2f, %.2f)", az, el);
    rot_term_log(ctrl, "gpredict:tx",
                 "autocal: issuing move to (%.2f, %.2f)", az, el);
    return TRUE;
}

static void rotctrl_autocal_start(GtkRotCtrl *ctrl)
{
    gint64 now_us = 0;
    gdouble cur_az = 0.0;
    gdouble cur_el = 0.0;
    gboolean pos_valid = FALSE;
    gboolean pos_invalid = FALSE;
    gboolean running = FALSE;

    if (ctrl == NULL)
        return;

    if (ctrl->cal_active)
    {
        if (ctrl->calib_dialog)
            gtk_window_present(GTK_WINDOW(ctrl->calib_dialog));
        return;
    }

    if (ctrl->cal_hold_active)
        rotctrl_set_cal_hold(ctrl, FALSE, "autocal_start");

    g_mutex_lock(&ctrl->client.mutex);
    running = ctrl->client.running;
    pos_valid = ctrl->client.pos_valid;
    pos_invalid = ctrl->client.pos_invalid;
    cur_az = ctrl->client.azi_in;
    cur_el = ctrl->client.ele_in;
    g_mutex_unlock(&ctrl->client.mutex);

    if (!ctrl->engaged || !running || !pos_valid || pos_invalid)
    {
        sat_log_log(SAT_LOG_LEVEL_WARN,
                    "autocal start aborted engaged=%d running=%d pos_valid=%d pos_invalid=%d",
                    ctrl->engaged ? 1 : 0,
                    running ? 1 : 0,
                    pos_valid ? 1 : 0,
                    pos_invalid ? 1 : 0);
        rot_term_log(ctrl, "gpredict:warn",
                     "autocal start aborted engaged=%d running=%d pos_valid=%d pos_invalid=%d",
                     ctrl->engaged ? 1 : 0,
                     running ? 1 : 0,
                     pos_valid ? 1 : 0,
                     pos_invalid ? 1 : 0);
        rot_show_message(ctrl,
                         GTK_MESSAGE_ERROR,
                         _("Calibration"),
                         _("Calibration requires an engaged rotator with valid position feedback."));
        return;
    }

    now_us = g_get_monotonic_time();
    ctrl->cal_active = TRUE;
    ctrl->calibration_active = TRUE;
    ctrl->cal_state = ROT_AUTOCAL_DRIVE_ZERO;
    ctrl->cal_did_setpos = FALSE;
    ctrl->cal_direct_active = TRUE;
    ctrl->cal_start_us = now_us;
    ctrl->cal_timeout_us = 0;
    rot_autocal_direct_reset(&ctrl->cal_direct,
                             now_us,
                             pos_valid,
                             cur_az,
                             cur_el);
    ctrl->cal_stable_count = 0;
    ctrl->cal_arrive_count = 0;
    ctrl->cal_ready_count = 0;
    ctrl->cal_last_log_us = 0;
    ctrl->calib_backup = ctrl->calib;
    ctrl->calib_backup_valid = TRUE;
    rotctrl_autocal_close_dialog(ctrl);
    ctrl->cal_window_count = 0;
    ctrl->cal_window_idx = 0;
    ctrl->cal_timeout_us = ROT_AUTOCAL_TIMEOUT_US;

    if (ctrl->calib_autocal_button)
        gtk_widget_set_sensitive(ctrl->calib_autocal_button, FALSE);

    gdouble target_el = MAX(0.0, rotctrl_elev_floor(ctrl));
    if (ctrl->conf && ctrl->conf->axis_mode == ROT_AXIS_MODE_AZ_ONLY)
        target_el = ctrl->conf->minel;

    sat_log_log(SAT_LOG_LEVEL_INFO,
                "autocal start pos_valid=%d pos=(%.2f, %.2f) cmd=(0.00, %.2f)",
                pos_valid ? 1 : 0, cur_az, cur_el, target_el);
    rot_term_log(ctrl, "gpredict:rx",
                 "autocal start pos_valid=%d pos=(%.2f, %.2f) cmd=(0.00, %.2f)",
                 pos_valid ? 1 : 0, cur_az, cur_el, target_el);
    sat_log_log(SAT_LOG_LEVEL_INFO, "CAL_START");
    rot_term_log(ctrl, "gpredict:rx", "CAL_START");

    gtk_rot_knob_set_value(GTK_ROT_KNOB(ctrl->AzSet), 0.0);
    gtk_rot_knob_set_value(GTK_ROT_KNOB(ctrl->ElSet), target_el);

    (void)rotctrl_autocal_queue_move(ctrl, 0.0, target_el);

    {
        GtkWidget *toplevel = gtk_widget_get_toplevel(GTK_WIDGET(ctrl));
        GtkWindow *parent = GTK_IS_WINDOW(toplevel) ? GTK_WINDOW(toplevel) : NULL;
        GtkWidget *dialog = gtk_message_dialog_new(parent,
                                                   GTK_DIALOG_MODAL |
                                                   GTK_DIALOG_DESTROY_WITH_PARENT,
                                                   GTK_MESSAGE_INFO,
                                                   GTK_BUTTONS_OK_CANCEL,
                                                   _("Calibrating: moving to (0\302\260,0\302\260). "
                                                     "Align to TRUE NORTH, press OK."));
        gtk_window_set_title(GTK_WINDOW(dialog), _("Calibration"));
        gtk_dialog_set_default_response(GTK_DIALOG(dialog), GTK_RESPONSE_OK);
        gtk_dialog_set_response_sensitive(GTK_DIALOG(dialog),
                                          GTK_RESPONSE_OK,
                                          FALSE);
        g_signal_connect(dialog, "response",
                         G_CALLBACK(calib_autocal_response_cb), ctrl);
        ctrl->calib_dialog = dialog;
        gtk_widget_show(dialog);
    }
}

static void rotctrl_autocal_tick(GtkRotCtrl *ctrl,
                                 gboolean rotpos_valid,
                                 gdouble rotaz,
                                 gdouble rotel)
{
    if (ctrl == NULL || !ctrl->cal_active)
        return;

    ctrl->cal_timeout_us = ROT_AUTOCAL_TIMEOUT_US;

    gint64 now_us = g_get_monotonic_time();
    gint64 last_pos_us = 0;
    gint64 pos_age_ms = -1;
    gboolean pos_invalid = FALSE;
    gboolean sample_fresh = FALSE;
    gdouble stable_eps = MAX(ROT_AUTOCAL_STABLE_EPS_DEG, k_meas_quantum_deg);
    gdouble target_az = 0.0;
    gdouble target_el = 0.0;
    gdouble az_err = 0.0;
    gdouble el_err = 0.0;
    gdouble motion_eps = MAX(0.2, 0.5 * k_meas_quantum_deg);
    RotAutocalDirectResult direct = { 0 };

    g_mutex_lock(&ctrl->client.mutex);
    last_pos_us = ctrl->client.last_pos_us;
    pos_invalid = ctrl->client.pos_invalid;
    g_mutex_unlock(&ctrl->client.mutex);
    if (last_pos_us > 0 && now_us > last_pos_us)
        pos_age_ms = (now_us - last_pos_us) / 1000;
    sample_fresh = rotpos_valid && pos_age_ms >= 0 &&
                   pos_age_ms <= ROT_CMD_POS_FRESH_MS;
    if (pos_invalid)
    {
        sat_log_log(SAT_LOG_LEVEL_WARN,
                    "autocal abort: pos_invalid");
        rot_term_log(ctrl, "gpredict:warn",
                     "autocal abort: pos_invalid");
        rotctrl_autocal_finish(ctrl, FALSE, "pos_invalid");
        return;
    }
    if (!ctrl->engaged || !ctrl->client.running)
    {
        rotctrl_autocal_finish(ctrl, FALSE, "disengaged");
        return;
    }

    target_el = MAX(0.0, rotctrl_elev_floor(ctrl));
    if (ctrl->conf && ctrl->conf->axis_mode == ROT_AXIS_MODE_AZ_ONLY)
        target_el = ctrl->conf->minel;

    rot_autocal_direct_step(&ctrl->cal_direct,
                            now_us,
                            sample_fresh,
                            last_pos_us,
                            rotaz,
                            rotel,
                            target_az,
                            target_el,
                            ROT_AUTOCAL_ARRIVE_TOL_AZ_DEG,
                            ROT_AUTOCAL_ARRIVE_TOL_EL_DEG,
                            motion_eps,
                            (gint64)ROT_AUTOCAL_MOVE_INTERVAL_MS * 1000,
                            (gint64)ROT_AUTOCAL_NO_MOTION_MS * 1000,
                            (gint64)ROT_AUTOCAL_NO_POS_MS * 1000,
                            &direct);
    az_err = direct.err_az;
    el_err = direct.err_el;

    if (ctrl->cal_timeout_us > 0 &&
        (now_us - ctrl->cal_start_us) > ctrl->cal_timeout_us)
    {
        rotctrl_autocal_finish(ctrl, FALSE, "timeout");
        return;
    }

    if (direct.no_position &&
        (now_us - ctrl->cal_start_us) >=
            ((gint64)ROT_AUTOCAL_NO_POS_MS * 1000))
    {
        rotctrl_autocal_finish(ctrl, FALSE, "no_position");
        return;
    }

    if (direct.no_motion)
    {
        sat_log_log(SAT_LOG_LEVEL_WARN,
                    "autocal: no motion detected pos=(%.2f, %.2f) err=(%.2f, %.2f)",
                    rotaz, rotel, az_err, el_err);
        rot_term_log(ctrl, "gpredict:warn",
                     "autocal: no motion detected pos=(%.2f, %.2f) err=(%.2f, %.2f)",
                     rotaz, rotel, az_err, el_err);
        rotctrl_autocal_finish(ctrl, FALSE, "no_motion");
        return;
    }

    if (ctrl->cal_state == ROT_AUTOCAL_DRIVE_ZERO)
    {
        if (direct.issue_move)
        {
            (void)rotctrl_autocal_queue_move(ctrl, target_az, target_el);
        }

        if (sample_fresh)
        {
            if (az_err <= ROT_AUTOCAL_ARRIVE_TOL_AZ_DEG &&
                el_err <= ROT_AUTOCAL_ARRIVE_TOL_EL_DEG)
            {
                if (ctrl->cal_arrive_count < G_MAXUINT)
                    ctrl->cal_arrive_count++;
            }
            else
            {
                ctrl->cal_arrive_count = 0;
            }

            if (ctrl->cal_arrive_count >= ROT_AUTOCAL_ARRIVE_COUNT)
            {
                ctrl->cal_state = ROT_AUTOCAL_SETTLE;
                ctrl->cal_ready_count = 0;
                ctrl->cal_window_count = 0;
                ctrl->cal_window_idx = 0;
                ctrl->cal_settle_start_us = now_us;
                rotctrl_autocal_set_ok_sensitive(ctrl, FALSE);
                sat_log_log(SAT_LOG_LEVEL_INFO,
                            "autocal phase DRIVE_ZERO->SETTLE err=(%.2f, %.2f)",
                            az_err, el_err);
                rot_term_log(ctrl, "gpredict:state",
                             "autocal phase DRIVE_ZERO->SETTLE err=(%.2f, %.2f)",
                             az_err, el_err);
            }
        }
        else
        {
            ctrl->cal_arrive_count = 0;
        }
    }
    else if (ctrl->cal_state == ROT_AUTOCAL_SETTLE ||
             ctrl->cal_state == ROT_AUTOCAL_READY)
    {
        if (ctrl->cal_state == ROT_AUTOCAL_READY)
        {
            /* Hold READY once reached to avoid re-entry. */
        }
        else if (!sample_fresh)
        {
            ctrl->cal_window_count = 0;
            ctrl->cal_window_idx = 0;
            ctrl->cal_ready_count = 0;
        }
        else
        {
            gdouble az_err = fabs(shortest_az_delta(rotaz, target_az));
            gdouble el_err = fabs(rotel - target_el);
            gdouble hysteresis = 1.0;

            if (az_err > (ROT_AUTOCAL_ARRIVE_TOL_AZ_DEG + hysteresis) ||
                el_err > (ROT_AUTOCAL_ARRIVE_TOL_EL_DEG + hysteresis))
            {
                ctrl->cal_state = ROT_AUTOCAL_DRIVE_ZERO;
                ctrl->cal_arrive_count = 0;
                ctrl->cal_ready_count = 0;
                ctrl->cal_window_count = 0;
                ctrl->cal_window_idx = 0;
                ctrl->cal_did_setpos = FALSE;
                ctrl->cal_settle_start_us = 0;
                rotctrl_autocal_set_ok_sensitive(ctrl, FALSE);
                sat_log_log(SAT_LOG_LEVEL_INFO,
                            "autocal phase SETTLE->DRIVE_ZERO err=(%.2f, %.2f)",
                            az_err, el_err);
                rot_term_log(ctrl, "gpredict:state",
                             "autocal phase SETTLE->DRIVE_ZERO err=(%.2f, %.2f)",
                             az_err, el_err);
            }
            else
            {
                gdouble az = rotctrl_autocal_normalize_zero_seam(rotaz);
                gdouble el = rotel;
                guint idx = ctrl->cal_window_idx % ROT_AUTOCAL_STABLE_WINDOW;

                ctrl->cal_window_az[idx] = az;
                ctrl->cal_window_el[idx] = el;
                ctrl->cal_window_idx = (idx + 1) % ROT_AUTOCAL_STABLE_WINDOW;
                if (ctrl->cal_window_count < ROT_AUTOCAL_STABLE_WINDOW)
                    ctrl->cal_window_count++;

                gdouble mean_az = 0.0;
                gdouble spread_az = 0.0;
                gdouble mean_el = 0.0;
                gdouble dev_el = 0.0;
                gboolean have_stats =
                    rotctrl_autocal_estimate(ctrl,
                                             &mean_az,
                                             &spread_az,
                                             &mean_el,
                                             &dev_el);
                gboolean stable_enough =
                    have_stats &&
                    ctrl->cal_window_count >= ROT_AUTOCAL_STABLE_WINDOW &&
                    spread_az <= stable_eps &&
                    dev_el <= stable_eps;

                if (stable_enough)
                {
                    if (ctrl->cal_ready_count < G_MAXUINT)
                        ctrl->cal_ready_count++;
                }
                else
                {
                    ctrl->cal_ready_count = 0;
                    if (ctrl->cal_state == ROT_AUTOCAL_READY)
                    {
                        ctrl->cal_state = ROT_AUTOCAL_SETTLE;
                        ctrl->cal_settle_start_us = now_us;
                        rotctrl_autocal_set_ok_sensitive(ctrl, FALSE);
                    }
                }

                gboolean ready_by_stable =
                    stable_enough &&
                    ctrl->cal_ready_count >= ROT_AUTOCAL_READY_COUNT;
                if (ready_by_stable &&
                    ctrl->cal_state != ROT_AUTOCAL_READY)
                {
                    gint64 settle_ms =
                        (ctrl->cal_settle_start_us > 0)
                            ? (now_us - ctrl->cal_settle_start_us) / 1000
                            : 0;
                    ctrl->cal_state = ROT_AUTOCAL_READY;
                    ctrl->cal_settle_start_us = 0;
                    rotctrl_autocal_set_ok_sensitive(ctrl, TRUE);
                    sat_log_log(SAT_LOG_LEVEL_INFO,
                                "autocal phase SETTLE->READY settle_ms=%lld "
                                "mean=(%.2f, %.2f) spread=%.2f dev_el=%.2f buf=%u poll_ms=%d",
                                (long long)settle_ms,
                                mean_az, mean_el, spread_az, dev_el,
                                ctrl->cal_window_count,
                                rotctrl_poll_period_ms(ctrl));
                    rot_term_log(ctrl, "gpredict:state",
                                 "autocal phase SETTLE->READY settle_ms=%lld "
                                 "mean=(%.2f, %.2f) spread=%.2f dev_el=%.2f buf=%u poll_ms=%d",
                                 (long long)settle_ms,
                                 mean_az, mean_el, spread_az, dev_el,
                                 ctrl->cal_window_count,
                                 rotctrl_poll_period_ms(ctrl));
                }
            }
        }
    }

    if (ctrl->cal_state == ROT_AUTOCAL_SETTLE &&
        ctrl->cal_settle_start_us > 0 &&
        (now_us - ctrl->cal_settle_start_us) >=
            ((gint64)ROT_AUTOCAL_SETTLE_MAX_MS * 1000))
    {
        rotctrl_autocal_finish(ctrl, FALSE, "timeout");
        return;
    }

    if (now_us - ctrl->cal_last_log_us >= ROT_AUTOCAL_LOG_INTERVAL_US)
    {
        guint stable_count =
            (ctrl->cal_state == ROT_AUTOCAL_DRIVE_ZERO)
                ? ctrl->cal_arrive_count
                : ctrl->cal_ready_count;
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "autocal: state=%d pos_valid=%d pos=(%.2f, %.2f) err=(%.2f, %.2f) stable_count=%u",
                    ctrl->cal_state,
                    rotpos_valid ? 1 : 0,
                    rotaz, rotel,
                    az_err, el_err,
                    stable_count);
        rot_term_log_verbose(ctrl, "gpredict:rx",
                             "autocal: state=%d pos_valid=%d pos=(%.2f, %.2f) err=(%.2f, %.2f) stable_count=%u",
                             ctrl->cal_state,
                             rotpos_valid ? 1 : 0,
                             rotaz, rotel,
                             az_err, el_err,
                             stable_count);
        ctrl->cal_last_log_us = now_us;
    }
}

static void calib_autocal_response_cb(GtkDialog *dialog,
                                      gint response_id,
                                      gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);
    gdouble mech_az = 0.0;
    gdouble mech_el = 0.0;
    const gchar *reason = NULL;

    if (ctrl == NULL)
    {
        gtk_widget_destroy(GTK_WIDGET(dialog));
        return;
    }

    if (!ctrl->cal_active)
    {
        ctrl->calib_dialog = NULL;
        gtk_widget_destroy(GTK_WIDGET(dialog));
        return;
    }

    if (response_id == GTK_RESPONSE_OK)
    {
        gdouble mean_az = 0.0;
        gdouble spread_az = 0.0;
        gdouble mean_el = 0.0;
        gdouble dev_el = 0.0;
        gdouble stable_eps = MAX(ROT_AUTOCAL_STABLE_EPS_DEG, k_meas_quantum_deg);

        if (ctrl->cal_state != ROT_AUTOCAL_READY)
        {
            sat_log_log(SAT_LOG_LEVEL_WARN,
                        "CAL_OK rejected: autocal not READY state=%d",
                        ctrl->cal_state);
            rot_term_log(ctrl, "gpredict:warn",
                         "CAL_OK rejected: autocal not READY state=%d",
                         ctrl->cal_state);
            return;
        }

        if (!rotctrl_autocal_estimate(ctrl,
                                      &mean_az,
                                      &spread_az,
                                      &mean_el,
                                      &dev_el))
        {
            sat_log_log(SAT_LOG_LEVEL_WARN,
                        "CAL_OK rejected: no stable samples count=%u",
                        ctrl->cal_window_count);
            rot_term_log(ctrl, "gpredict:warn",
                         "CAL_OK rejected: no stable samples count=%u",
                         ctrl->cal_window_count);
            return;
        }

        ctrl->calib_dialog = NULL;
        gtk_widget_destroy(GTK_WIDGET(dialog));

        {
            gdouble az_raw = mean_az;
            mech_az = rotctrl_autocal_normalize_zero_seam(mean_az);
            mech_el = mean_el;
            if (fabs(az_raw - mech_az) > 1e-6)
            {
                sat_log_log(SAT_LOG_LEVEL_INFO,
                            "CAL_OK seam canonicalize az=%.2f -> %.2f",
                            az_raw, mech_az);
                rot_term_log(ctrl, "gpredict:rx",
                             "CAL_OK seam canonicalize az=%.2f -> %.2f",
                             az_raw, mech_az);
            }
        }

        if (spread_az > stable_eps ||
            dev_el > stable_eps)
        {
            sat_log_log(SAT_LOG_LEVEL_WARN,
                        "CAL_OK using unstable estimate spread=(%.2f, %.2f)",
                        spread_az, dev_el);
            rot_term_log(ctrl, "gpredict:warn",
                         "CAL_OK using unstable estimate spread=(%.2f, %.2f)",
                         spread_az, dev_el);
        }

        if (!rotctrl_autocal_apply_mech(ctrl, mech_az, mech_el, &reason))
        {
            rot_show_message(ctrl,
                             GTK_MESSAGE_ERROR,
                             _("Calibration"),
                             _("Unable to save calibration."));
            rotctrl_autocal_finish(ctrl, FALSE,
                                   reason ? reason : "save_failed");
            return;
        }

        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "autocal apply mech=(%.2f, %.2f) offsets=(%.2f, %.2f) "
                    "unc=(%.2f, %.2f) enabled=%d",
                    mech_az, mech_el,
                    ctrl->calibration_offset_az,
                    ctrl->calibration_offset_el,
                    ctrl->calib.az_uncertainty_deg,
                    ctrl->calib.el_uncertainty_deg,
                    1);
        rot_term_log(ctrl, "gpredict:rx",
                     "autocal apply mech=(%.2f, %.2f) offsets=(%.2f, %.2f) "
                     "unc=(%.2f, %.2f) enabled=%d",
                     mech_az, mech_el,
                     ctrl->calibration_offset_az,
                     ctrl->calibration_offset_el,
                     ctrl->calib.az_uncertainty_deg,
                     ctrl->calib.el_uncertainty_deg,
                     1);
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "CAL_OK pos=(%.2f, %.2f) stored_as_zero tol=(%.2f, %.2f)",
                    mech_az, mech_el,
                    ctrl->calib.az_uncertainty_deg,
                    ctrl->calib.el_uncertainty_deg);
        rot_term_log(ctrl, "gpredict:rx",
                     "CAL_OK pos=(%.2f, %.2f) stored_as_zero tol=(%.2f, %.2f)",
                     mech_az, mech_el,
                     ctrl->calib.az_uncertainty_deg,
                     ctrl->calib.el_uncertainty_deg);
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "CAL_OK estimate mean=(%.2f, %.2f) spread=(%.2f, %.2f)",
                    mean_az, mean_el,
                    spread_az, dev_el);
        rot_term_log(ctrl, "gpredict:rx",
                     "CAL_OK estimate mean=(%.2f, %.2f) spread=(%.2f, %.2f)",
                     mean_az, mean_el,
                     spread_az, dev_el);

        ctrl->force_next_send = TRUE;
        ctrl->setpoint_valid = FALSE;
        ctrl->last_target_valid = FALSE;
        ctrl->pretrack_target_valid = FALSE;
        ctrl->pretrack_wrap_valid = FALSE;
        ctrl->pretrack_wrap_user_az = 0.0;
        ctrl->pretrack_wrap_raw_az = 0.0;
        ctrl->pretrack_wrap_k = 0;
        ctrl->last_target_update_us = 0;
        rotctrl_set_cal_hold(ctrl, TRUE, "cal_ok");

        rotctrl_autocal_finish(ctrl, TRUE, "user_ok");
        return;
    }

    ctrl->calib_dialog = NULL;
    gtk_widget_destroy(GTK_WIDGET(dialog));

    g_mutex_lock(&ctrl->client.mutex);
    ctrl->client.stop_pending = TRUE;
    ctrl->client.new_trg = FALSE;
    g_mutex_unlock(&ctrl->client.mutex);
    sat_log_log(SAT_LOG_LEVEL_INFO, "CAL_CANCEL");
    rot_term_log(ctrl, "gpredict:rx", "CAL_CANCEL");
    rotctrl_autocal_finish(ctrl, FALSE, "cancel");
}

static void G_GNUC_UNUSED calib_enabled_toggled_cb(GtkToggleButton *button, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    ctrl->calib.enabled = gtk_toggle_button_get_active(button);
    sat_log_log(SAT_LOG_LEVEL_INFO,
                "rot calibration %s (az=%.2f el=%.2f)",
                ctrl->calib.enabled ? "enabled" : "disabled",
                ctrl->calib.az_offset_deg,
                ctrl->calib.el_offset_deg);
    if (rotctrl_calib_ensure_id(ctrl))
        (void)calib_save(ctrl->rotor_id, &ctrl->calib);
    rotctrl_calib_update_offset_sensitivity(ctrl);
}

static void G_GNUC_UNUSED calib_az_offset_changed_cb(GtkSpinButton *spin, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);
    gdouble value = gtk_spin_button_get_value(spin);

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    ctrl->calib.az_offset_deg = wrap360(value);
    if (rotctrl_calib_ensure_id(ctrl))
        (void)calib_save(ctrl->rotor_id, &ctrl->calib);
}

static void G_GNUC_UNUSED calib_el_offset_changed_cb(GtkSpinButton *spin, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);
    gdouble value = gtk_spin_button_get_value(spin);

    if (ctrl == NULL || ctrl->ui_updating)
        return;

    ctrl->calib.el_offset_deg = value;
    if (rotctrl_calib_ensure_id(ctrl))
        (void)calib_save(ctrl->rotor_id, &ctrl->calib);
}

static void calib_autocal_clicked_cb(GtkButton *button, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);

    (void)button;

    if (ctrl == NULL)
        return;

    if (!ctrl->conf || !ctrl->engaged || !ctrl->client.running)
    {
        rot_show_message(ctrl,
                         GTK_MESSAGE_WARNING,
                         _("Calibration"),
                         _("Engage the rotator before starting calibration."));
        return;
    }

    if (!rotctrl_calib_ensure_id(ctrl))
        return;

    rotctrl_autocal_start(ctrl);
}

/* Offset controls callbacks */
static void offset_toggle_cb(GtkToggleButton *button, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);
    if (ctrl == NULL || ctrl->ui_updating)
        return;
    ctrl->use_offset = gtk_toggle_button_get_active(button);
    if (ctrl->conf)
        ctrl->conf->use_offset = ctrl->use_offset;
    sat_log_log(SAT_LOG_LEVEL_INFO,
                "rot offsets %s (az=%.2f el=%.2f)",
                ctrl->use_offset ? "enabled" : "disabled",
                ctrl->az_offset_deg,
                ctrl->el_offset_deg);
    rotctrl_calib_update_offset_sensitivity(ctrl);
    rot_transform_update(ctrl);
    rot_plan_reset(&ctrl->trajectory_plan);
    if (ctrl->tracking && ctrl->pass &&
        rotctrl_session_ready(ctrl,
                              rotctrl_pos_recent(ctrl,
                                                 (gint64)rotctrl_stale_ms(ctrl) * 1000,
                                                 NULL)))
        rot_build_tracking_plan(ctrl);
}

static void az_offset_changed_cb(GtkSpinButton *spin, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);
    if (ctrl == NULL || ctrl->ui_updating)
        return;
    ctrl->az_offset_deg = gtk_spin_button_get_value(spin);
    if (ctrl->conf)
        ctrl->conf->az_offset = ctrl->az_offset_deg;
    rot_transform_update(ctrl);
    rot_plan_reset(&ctrl->trajectory_plan);
    if (ctrl->tracking && ctrl->pass &&
        rotctrl_session_ready(ctrl,
                              rotctrl_pos_recent(ctrl,
                                                 (gint64)rotctrl_stale_ms(ctrl) * 1000,
                                                 NULL)))
        rot_build_tracking_plan(ctrl);
}

static void el_offset_changed_cb(GtkSpinButton *spin, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);
    if (ctrl == NULL || ctrl->ui_updating)
        return;
    ctrl->el_offset_deg = gtk_spin_button_get_value(spin);
    if (ctrl->conf)
        ctrl->conf->el_offset = ctrl->el_offset_deg;
    rot_transform_update(ctrl);
    rot_plan_reset(&ctrl->trajectory_plan);
    if (ctrl->tracking && ctrl->pass &&
        rotctrl_session_ready(ctrl,
                              rotctrl_pos_recent(ctrl,
                                                 (gint64)rotctrl_stale_ms(ctrl) * 1000,
                                                 NULL)))
        rot_build_tracking_plan(ctrl);
}
