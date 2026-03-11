/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#ifndef __GTK_RIG_CTRL_H__
#define __GTK_RIG_CTRL_H__ 1

#include <glib.h>
#include <glib/gi18n.h>
#include <gtk/gtk.h>

#include "gtk-sat-module.h"
#include "predict-tools.h"
#include "radio-conf.h"
#include "payload-profile.h"
#include "ui-status.h"
#include "sgpsdp/sgp4sdp4.h"
#include "trsp-conf.h"

typedef struct _RigctldMgr RigctldMgr;
typedef struct _GpTermView GpTermView;
typedef struct _RigSession RigSession;
typedef struct _RigctldClient RigctldClient;
typedef struct _GTask GTask;
typedef struct _GCancellable GCancellable;


#define GTK_TYPE_RIG_CTRL          (gtk_rig_ctrl_get_type ())
#define GTK_RIG_CTRL(obj)          G_TYPE_CHECK_INSTANCE_CAST (obj,\
                                   gtk_rig_ctrl_get_type (),\
                                   GtkRigCtrl)

#define GTK_RIG_CTRL_CLASS(klass)  G_TYPE_CHECK_CLASS_CAST (klass,\
                                   gtk_rig_ctrl_get_type (),\
                                   GtkRigCtrlClass)

#define IS_GTK_RIG_CTRL(obj)       G_TYPE_CHECK_INSTANCE_TYPE (obj, gtk_rig_ctrl_get_type ())

typedef struct _gtk_rig_ctrl GtkRigCtrl;
typedef struct _GtkRigCtrlClass GtkRigCtrlClass;

typedef enum {
    RIGCTRL_CONN_DISCONNECTED = 0,
    RIGCTRL_CONN_CONNECTING,
    RIGCTRL_CONN_CONNECTED,
    RIGCTRL_CONN_DISCONNECTING
} rigctrl_conn_state_t;

struct _gtk_rig_ctrl {
    GtkBox          box;

    GtkWidget      *SatFreqDown;
    GtkWidget      *RigFreqDown;
    GtkWidget      *SatFreqUp;
    GtkWidget      *RigFreqUp;
    GtkWidget      *SatDopDown; /*!< Doppler shift down */
    GtkWidget      *SatDopUp;   /*!< Doppler shift up */
    GtkWidget      *LoDown;     /*!< LO of downconverter */
    GtkWidget      *LoUp;       /*!z LO of upconverter */

    /* target status labels */
    GtkWidget      *SatAz, *SatEl, *SatCnt;
    GtkWidget      *SatRng, *SatRngRate, *SatDop;

    /* other widgets */
    GtkWidget      *SatSel;     /*!< Satellite selector */
    GtkWidget      *TrspSel;    /*!< Transponder selector */
    GtkWidget      *DevSel;     /*!< Device selector */
    GtkWidget      *DevSel2;    /*!< Second device selector */
    GtkWidget      *LockBut;
    GtkWidget      *cycle_spin;      /*!< Update timer cycle */

    radio_conf_t   *conf;       /*!< Radio configuration */
    radio_conf_t   *conf2;      /*!< Secondary radio configuration */
    GSList         *trsplist;   /*!< List of available transponders */
    trsp_t         *trsp;       /*!< Pointer to the current transponder configuration */
    gboolean        trsplock;   /*!< Flag indicating whether uplink and downlink are lockled */

    GSList         *sats;       /*!< List of sats in parent module */
    sat_t          *target;     /*!< Target satellite */
    pass_t         *pass;       /*!< Next pass of target satellite */
    qth_t          *qth;        /*!< The QTH for this module */

    double          prev_ele;   /*!< Previous elevation (used for AOS/LOS signalling) */

    guint           delay;      /*!< Timeout delay. */
    guint           timerid;    /*!< Timer ID */

    gboolean        tracking;   /*!< Flag set when we are tracking a target. */
    gboolean        rx_track_enabled; /*!< RX tracking toggle */
    gboolean        tx_track_enabled; /*!< TX tracking toggle */
    gint64          menu_rx_hz; /*!< RX menu/base frequency (Hz) */
    gint64          menu_tx_hz; /*!< TX menu/base frequency (Hz) */
    gint64          target_radio_rx_hz; /*!< RX target rig frequency (Hz) */
    gint64          target_radio_tx_hz; /*!< TX target rig frequency (Hz) */
    GMutex          busy;       /*!< Flag set when control algorithm is busy. */
    gboolean        engaged;    /*!< Flag indicating that rig device is engaged. */
    gboolean        engage_pending; /*!< True while initial engage attempt is unresolved. */
    gint            errcnt;     /*!< Error counter. */

    gboolean        lastrxptt;  /*!< PTT state of last rx cycle. */
    gboolean        lasttxptt;  /*!< PTT state of last tx cycle. */

    gint64          lastrxf;    /*!< Last frequency sent to receiver (Hz). */
    gint64          lasttxf;    /*!< Last frequency sent to transmitter (Hz). */
    gint64          du, dd;     /*!< Last computed up/down Doppler shift (Hz). */
    PayloadProfile  payload_profile; /*!< Payload profile derived from menu selection. */
    gboolean        payload_profile_valid; /*!< TRUE when payload profile is initialized. */
    gboolean        payload_profile_logged; /*!< Avoid repeated payload profile logs. */
    gdouble         doppler_down_ema; /*!< Smoothed downlink Doppler. */
    gdouble         doppler_up_ema;   /*!< Smoothed uplink Doppler. */
    gboolean        doppler_ema_valid; /*!< TRUE once EMA is seeded. */
    gint64          last_doppler_calc_us; /*!< Last Doppler calc time (monotonic). */
    gint64          last_send_down_us; /*!< Last downlink send time (monotonic). */
    gint64          last_send_up_us;   /*!< Last uplink send time (monotonic). */
    gint64          last_sent_down_hz; /*!< Last downlink frequency command sent. */
    gint64          last_sent_up_hz;   /*!< Last uplink frequency command sent. */
    gint64          last_target_down_hz; /*!< Last downlink target computed. */
    gint64          last_target_up_hz;   /*!< Last uplink target computed. */
    gint64          last_doppler_log_us; /*!< Last doppler tick log (monotonic). */
    gint64          last_probe_log_us;   /*!< Last probe log time (monotonic). */
    gint            doppler_suppress_down; /*!< Last downlink suppression reason. */
    gint            doppler_suppress_up;   /*!< Last uplink suppression reason. */
    gint64          user_base_down_hz; /*!< User-entered downlink base frequency. */
    gint64          user_base_up_hz;   /*!< User-entered uplink base frequency. */
    gint64          doppler_down_hz;   /*!< Current downlink Doppler offset. */
    gint64          doppler_up_hz;     /*!< Current uplink Doppler offset. */
    gint64          rig_target_down_hz; /*!< Target rig downlink frequency. */
    gint64          rig_target_up_hz;   /*!< Target rig uplink frequency. */
    gint64          rig_actual_down_hz; /*!< Last observed rig downlink frequency. */
    gint64          rig_actual_up_hz;   /*!< Last observed rig uplink frequency. */
    gint64          last_valid_target_down_hz; /*!< Last valid downlink target sent. */
    gint64          last_valid_target_up_hz;   /*!< Last valid uplink target sent. */
    gint64          last_send_log_down_us; /*!< Last send log time (downlink). */
    gint64          last_send_log_up_us;   /*!< Last send log time (uplink). */
    gint64          last_calc_log_down_us; /*!< Last calc log time (downlink). */
    gint64          last_calc_log_up_us;   /*!< Last calc log time (uplink). */
    gint64          last_invalid_log_us;   /*!< Last invalid frequency log time. */
    gint64          last_verify_log_down_us; /*!< Last verify log time (downlink). */
    gint64          last_verify_log_up_us;   /*!< Last verify log time (uplink). */
    gboolean        verify_degraded_down; /*!< Downlink verify mismatch flag. */
    gboolean        verify_degraded_up;   /*!< Uplink verify mismatch flag. */
    gboolean        pending_manual_down;  /*!< Pending manual send (downlink). */
    gboolean        pending_manual_up;    /*!< Pending manual send (uplink). */
    gboolean        pending_preset_down;  /*!< Pending preset send (downlink). */
    gboolean        pending_preset_up;    /*!< Pending preset send (uplink). */
    gboolean        user_edit_down; /*!< User edited downlink base freq in session. */
    gboolean        user_edit_up;   /*!< User edited uplink base freq in session. */
    gboolean        suppress_user_base; /*!< Guard for programmatic base updates. */
    gboolean        xit_supported;      /*!< XIT supported (primary) */
    gboolean        xit_supported2;     /*!< XIT supported (secondary) */
    gboolean        last_rit_valid;     /*!< Have applied RIT offset (primary) */
    gboolean        last_xit_valid;     /*!< Have applied XIT offset (primary) */
    gboolean        last_xit_valid2;    /*!< Have applied XIT offset (secondary) */
    gint            last_rit_offset;    /*!< Last RIT offset sent (Hz) */
    gint            last_xit_offset;    /*!< Last XIT offset sent (Hz, primary) */
    gint            last_xit_offset2;   /*!< Last XIT offset sent (Hz, secondary) */
    GMutex          freq_cache_lock;   /*!< Protects cached base/doppler values */

    gint64          last_toggle_tx;     /*!< Last time when exec_toggle_tx_cycle() was executed (seconds)
                                           -1 indicates that an update should be performed ASAP */

    gint            sock, sock2;        /*!< Sockets for controlling the radio(s). */
    rigctrl_conn_state_t conn_state;    /*!< Primary connection state */
    rigctrl_conn_state_t conn_state2;   /*!< Secondary connection state */
    gboolean        opening;            /*!< Guard against re-entrant open */
    gboolean        opening2;           /*!< Guard against re-entrant open (secondary) */
    GTask          *open_task;          /*!< Async engage/open task */
    GCancellable   *open_cancellable;   /*!< Engage/open cancellation */
    guint           reconnect_source_id;  /*!< Scheduled reconnect source (primary) */
    guint           reconnect_source_id2; /*!< Scheduled reconnect source (secondary) */
    guint           close_pending_id;     /*!< Pending close handler source id */
    gint            pending_close_sock;  /*!< Deferred close fd (primary) */
    gint            pending_close_sock2; /*!< Deferred close fd (secondary) */
    gboolean        destroying;          /*!< Guard against teardown races */
    gint            reconnect_backoff_ms;   /*!< Exponential backoff for reconnect (primary). */
    gint            reconnect_backoff_ms2;  /*!< Exponential backoff for reconnect (secondary). */
    gint64          reconnect_next_us;      /*!< Next reconnect time (monotonic us, primary). */
    gint64          reconnect_next_us2;     /*!< Next reconnect time (monotonic us, secondary). */
    guint           reconnect_attempts;     /*!< Reconnect attempts in current engage cycle (primary). */
    guint           reconnect_attempts2;    /*!< Reconnect attempts in current engage cycle (secondary). */
    gboolean        link_lost_latched;      /*!< Latch primary link-lost status until user re-engages. */
    gboolean        link_lost_latched2;     /*!< Latch secondary link-lost status until user re-engages. */
    gboolean        rx_conn_error_reported; /*!< Avoid repeated connect error popups (primary). */
    gboolean        tx_conn_error_reported; /*!< Avoid repeated connect error popups (secondary). */
    gboolean        edit_primary;           /*!< Suppress reconnect while editing primary config. */
    gboolean        edit_secondary;         /*!< Suppress reconnect while editing secondary config. */
    GHashTable     *autostart_error_reported; /*!< Deduplicate autostart error dialogs. */
    GHashTable     *missing_model_reported;   /*!< Deduplicate missing model dialogs. */

    /* debug related */
    guint           wrops;
    guint           rdops;
    GString        *rigctld_rxbuf;     /*!< Buffered rigctld input (primary) */
    GString        *rigctld_rxbuf2;    /*!< Buffered rigctld input (secondary) */
    gchar          *rigctld_vfo_main_token;  /*!< Cached rigctld token for Main (primary) */
    gchar          *rigctld_vfo_sub_token;   /*!< Cached rigctld token for Sub (primary) */
    gchar          *rigctld_vfo_main_token2; /*!< Cached rigctld token for Main (secondary) */
    gchar          *rigctld_vfo_sub_token2;  /*!< Cached rigctld token for Sub (secondary) */
    gboolean        rigctld_vfo_map_logged;  /*!< Logged VFO mapping (primary) */
    gboolean        rigctld_vfo_map_logged2; /*!< Logged VFO mapping (secondary) */

    /* DL4PD */
    /* threads related stuff */
    /* add mutexes etc, to make threads reentrant! */
    GMutex          writelock;  /*!< Mutex for blocking write operation */
    GMutex          rig_ctrl_updatelock;        /*!< Mutex while updating widgets etc */
    GMutex          widgetsync; /*!< Mutex used while leaving (sync stuff) */
    GCond           widgetready;        /*!< Condition when work is done (sync stuff) */
    GAsyncQueue    *rigctlq;    /*!< Message queue to indicate something has changed */
    GThread        *rigctl_thread;      /*!< Pointer to current rigctl-thread */
    GThread        *main_thread;        /*!< GTK main thread owning this widget */

    RigctldMgr     *rigctld_mgr;        /*!< Auto-started rigctld manager (primary) */
    RigctldMgr     *rigctld_mgr2;       /*!< Auto-started rigctld manager (secondary) */
    gboolean        rigctld_spawned;    /*!< TRUE if primary rigctld was spawned by gpredict */
    gint            rigctld_spawn_pid;  /*!< PID for spawned primary rigctld (if known) */
    gboolean        rigctld_spawned2;   /*!< TRUE if secondary rigctld was spawned by gpredict */
    gint            rigctld_spawn_pid2; /*!< PID for spawned secondary rigctld (if known) */
    RigctldClient  *rig_client;        /*!< rigctld transport (primary) */
    RigctldClient  *rig_client2;       /*!< rigctld transport (secondary) */
    RigSession     *rig_session;        /*!< Serialized rigctld session (primary) */
    RigSession     *rig_session2;       /*!< Serialized rigctld session (secondary) */

    GpTermView     *term_view;          /*!< Embedded radio debug terminal */
    GtkWidget      *log_toggle;         /*!< Logs toggle button */
    GtkWidget      *log_verbose_toggle; /*!< Verbose rig logging toggle */
    rig_log_level_t log_level;          /*!< Rig log verbosity */
    guint           resize_idle_id;     /*!< Pending resize idle source id */
    gint            log_closed_height;  /*!< Cached height with log hidden */
    gboolean        ui_updating;        /*!< Guard against re-entrant UI updates */
    guint           pending_ui_refresh_id; /*!< Coalesced UI refresh id */
    gchar          *primary_rig_id;     /*!< Selected primary rig ID */
    gchar          *secondary_rig_id;   /*!< Selected secondary rig ID */
    GtkWidget      *status_label;       /*!< Weak-tracked command/status label */
    GtkWidget      *status_indicator_widget; /*!< Weak-tracked status indicator */
    gboolean        cmd_error;          /*!< Last command status */
    RadioUiStatus   ui_status;          /*!< Operator-facing status */
    gboolean        ui_hard_error;      /*!< Latched startup/configuration failure */
    gchar           ui_hard_error_reason[128];
    gchar           ui_status_detail[160];
    RigUiCommandWindow ui_cmd_window;   /*!< Last command outcomes */
};

struct _GtkRigCtrlClass {
    GtkVBoxClass    parent_class;
};

GType           gtk_rig_ctrl_get_type(void);
GtkWidget      *gtk_rig_ctrl_new(GtkSatModule * module);
void            gtk_rig_ctrl_update(GtkRigCtrl * ctrl, gdouble t);
void            gtk_rig_ctrl_select_sat(GtkRigCtrl * ctrl, gint catnum);

#endif /* __GTK_RIG_CTRL_H__ */
