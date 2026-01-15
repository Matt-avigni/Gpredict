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
#ifndef WIN32
#include <fcntl.h>
#endif

/* NETWORK */
#ifndef WIN32
#ifdef _WIN32
  #include <winsock2.h>   /* htons(), etc. */
  #include <ws2tcpip.h>
#else
  #include <arpa/inet.h>  /* htons(), etc. */
#endif
#include <netdb.h>              /* gethostbyname() */
#include <netinet/in.h>         /* struct sockaddr_in */
#include <sys/socket.h>         /* socket(), connect(), send() */
#else
#include <winsock2.h>
#endif
#ifdef G_OS_WIN32
#include <windows.h>
#endif

#include "compat.h"
#include "gp-term-view.h"
#include "gpredict-utils.h"
#include "gtk-freq-knob.h"
#include "gtk-rig-ctrl.h"
#include "rig-mode-dispatch.h"
#include "predict-tools.h"
#include "radio-conf.h"
#include "rigctld_mgr.h"
#include "rotctld-parse.h"
#include "serial-ports.h"
#include "sat-log.h"
#include "sat-cfg.h"
#include "sat-pref-rig-editor.h"
#include "trsp-conf.h"

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


#define AZEL_FMTSTR "%7.2f\302\260"
#define MAX_ERROR_COUNT 5
#define WR_DEL 5000             /* delay in usec to wait between write and read commands */
#define RIGCTLD_SOCKET_TIMEOUT_MS 3000
#define RIGCTLD_AUTODETECT_MAX_CANDIDATES 8
#define RIGCTLD_AUTODETECT_TOTAL_MS 10000
#define RIGCTLD_AUTODETECT_WAIT_MS 1500
#define RIGCTLD_AUTODETECT_PROBE_MS 1500
#define RIGCTLD_PROBE_SHORT_MS 300
#define RIGCTLD_STARTUP_TIMEOUT_MS 2000
#define RIGCTLD_STARTUP_POLL_MS 25
#define RIGCTLD_AUTOSTART_TIMEOUT_MS 2000
#define RIGCTLD_AUTOSTART_POLL_MS 20
#define RIGCTLD_HEALTH_TIMEOUT_MS 200
#define RIGCTLD_HEALTH_RETRIES 3
#define RIGCTLD_HEALTH_RETRY_DELAY_MS 50
#define RIGCTLD_AUTOSTART_MAX_RESTARTS 2
#define RIGCTLD_AUTOSTART_RETRY_DELAY_MS 150
#define RIGCTLD_MODEL_IC9700 3081
#define RIGCTLD_MODEL_IC905 3090
#define RIGCTLD_IC905_FALLBACK_TIMEOUT_MS 12000
#define RIGCTRL_RECONNECT_BACKOFF_MIN_MS 5000
#define RIGCTRL_RECONNECT_BACKOFF_MAX_MS 10000
#define RIGCTRL_RESPONSE_OPEN_CONFIG 1001
#define RIGCTRL_RESPONSE_DISABLE_AUTOSTART 1002
#define RIGCTRL_RESPONSE_SHOW_LOG 1003
#define RIGCTRL_TRSP_POPUP_MAX_HEIGHT 400

static GHashTable *rigctld_device_cache = NULL;
static GHashTable *rig_freq_cache = NULL;

typedef enum {
    RIGCTLD_PROBE_OK = 0,
    RIGCTLD_PROBE_NOT_READY,
    RIGCTLD_PROBE_MISMATCH
} rigctld_probe_result_t;

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

/* radio control functions */
static void     exec_rx_cycle(GtkRigCtrl * ctrl);
static void     exec_tx_cycle(GtkRigCtrl * ctrl);
static void     exec_trx_cycle(GtkRigCtrl * ctrl);
static void     exec_toggle_cycle(GtkRigCtrl * ctrl);
static void     exec_toggle_tx_cycle(GtkRigCtrl * ctrl);
static void     exec_full_duplex_main_sub_cycle(GtkRigCtrl * ctrl);
static void     exec_duplex_cycle(GtkRigCtrl * ctrl);
static void     exec_duplex_tx_cycle(GtkRigCtrl * ctrl);
static void     exec_dual_rig_cycle(GtkRigCtrl * ctrl);
static gboolean check_aos_los(GtkRigCtrl * ctrl);
static gboolean set_freq_simplex(GtkRigCtrl * ctrl, gint sock, gdouble freq);
static gboolean get_freq_simplex(GtkRigCtrl * ctrl, gint sock, gdouble * freq);
static gboolean get_freq_simplex_strict(GtkRigCtrl * ctrl, gint sock,
                                        gdouble * freq);
static gboolean set_freq_toggle(GtkRigCtrl * ctrl, gint sock, gdouble freq);
static gboolean set_toggle(GtkRigCtrl * ctrl, gint sock);
static gboolean unset_toggle(GtkRigCtrl * ctrl, gint sock);
static gboolean get_freq_toggle(GtkRigCtrl * ctrl, gint sock, gdouble * freq);
static gboolean get_freq_toggle_strict(GtkRigCtrl * ctrl, gint sock,
                                       gdouble * freq);
static gboolean get_ptt(GtkRigCtrl * ctrl, gint sock);
static gboolean set_ptt(GtkRigCtrl * ctrl, gint sock, gboolean ptt);
static gboolean set_rit(GtkRigCtrl * ctrl, gint sock, gdouble hz);
static gboolean set_xit(GtkRigCtrl * ctrl, gint sock, gdouble hz);
static void     apply_rit_xit_offsets(GtkRigCtrl * ctrl, gdouble rit,
                                      gdouble xit);
static void     update_rit_xit_offsets(GtkRigCtrl * ctrl);
static gboolean probe_rigctld(GtkRigCtrl *ctrl, gint sock,
                              const gchar *label,
                              gboolean *stale_device_out);
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
                                       gboolean *error_reported);
static gboolean open_rigctld_socket_host(const gchar *host, gint port,
                                         gint *sock,
                                         gint *err_out,
                                         gint *so_err_out);
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
static void     rig_logs_toggle_cb(GtkToggleButton *button, gpointer data);
static void     rig_term_log(GtkRigCtrl *ctrl, const gchar *prefix,
                             const gchar *fmt, ...) G_GNUC_PRINTF(3, 4);
static void     rig_term_log_tx(GtkRigCtrl *ctrl, const gchar *cmd);
static void     rig_term_log_rx(GtkRigCtrl *ctrl, const gchar *reply);
static void     rig_term_log_err_rprt(GtkRigCtrl *ctrl, const gchar *cmd,
                                      const gchar *reply, gint code);
static void     rigctrl_set_user_base_freq(GtkRigCtrl *ctrl,
                                           gboolean downlink,
                                           gdouble hz,
                                           gboolean mark_manual);
static gdouble  rigctrl_get_user_base_freq(GtkRigCtrl *ctrl,
                                           gboolean downlink);
static gboolean rigctrl_sat_freq_within_limits(const GtkRigCtrl *ctrl,
                                               gboolean downlink,
                                               gdouble sat_freq);
static void     rig_error_dialog_response(GtkDialog *dialog, gint response_id,
                                          gpointer data);
static void     rigctrl_show_log(GtkRigCtrl *ctrl);
static void     rigctrl_schedule_status(GtkRigCtrl *ctrl,
                                        const gchar *text,
                                        gboolean is_error);
static gboolean rigctrl_configure_trsp_popup_idle(gpointer data);
static void     rigctrl_trsp_combo_realize(GtkWidget *widget, gpointer data);
static gboolean rigctrl_trsp_combo_button_press(GtkWidget *widget,
                                                GdkEventButton *event,
                                                gpointer data);
static void     rigctrl_trsp_popup_show(GtkWidget *widget, gpointer data);
static void     rigctrl_trsp_popup_hide(GtkWidget *widget, gpointer data);
static GtkWidget *rigctrl_trsp_find_child(GtkWidget *widget,
                                          GType child_type);
static void     rigctrl_trsp_fix_expansion(GtkWidget *widget,
                                           GtkWidget *list);
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
static gboolean rigctld_try_autodetect_restart(GtkRigCtrl *ctrl,
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
static void     rigctrl_log_config(GtkRigCtrl *ctrl,
                                   const radio_conf_t *conf,
                                   const gchar *role);
static gboolean rigctrl_validate_mode(GtkRigCtrl *ctrl,
                                      const radio_conf_t *conf,
                                      const gchar *role);
static void     rigctrl_reset_reconnect(GtkRigCtrl *ctrl, gboolean secondary);
static gboolean rigctrl_reconnect_due(GtkRigCtrl *ctrl, gboolean secondary,
                                      gint64 now_us);
static void     rigctrl_schedule_reconnect(GtkRigCtrl *ctrl, gboolean secondary,
                                           const gchar *role);
static void     rigctrl_reset_error_gates(GtkRigCtrl *ctrl);
static void     rigctrl_fail_engage(GtkRigCtrl *ctrl);
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
static gboolean rigctrl_open(GtkRigCtrl * data);
static void     rigctrl_close(GtkRigCtrl * data);
static void     setconfig(gpointer data);
static void     remove_timer(GtkRigCtrl * data);

static void     start_timer(GtkRigCtrl * data);

/* Show a simple error dialog related to radio control */
static void
rig_show_error_dialog(GtkRigCtrl *ctrl,
                      const gchar *primary,
                      const gchar *secondary)
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
    gp_term_view_log(ctrl->term_view, "%s [%s] %s", stamp, prefix, msg);
    g_free(stamp);
    g_free(msg);
}

static void rig_term_log_tx(GtkRigCtrl *ctrl, const gchar *cmd)
{
    gchar *trim;

    if (ctrl == NULL || cmd == NULL)
        return;

    trim = g_strdup(cmd);
    g_strchomp(trim);
    g_strstrip(trim);
    if (*trim != '\0')
        rig_term_log(ctrl, "gpredict:tx", "%s", trim);
    g_free(trim);
}

static void rig_term_log_rx(GtkRigCtrl *ctrl, const gchar *reply)
{
    gchar *trim;

    if (ctrl == NULL || reply == NULL)
        return;

    trim = g_strdup(reply);
    g_strchomp(trim);
    g_strstrip(trim);
    if (*trim != '\0')
        rig_term_log(ctrl, "gpredict:rx", "%s", trim);
    g_free(trim);
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

static gboolean rig_parse_rprt_code(const gchar *reply, gint *code_out)
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

static gboolean rigctld_reply_indicates_stale_device(const gchar *reply)
{
    gint code = 0;

    if (reply == NULL)
        return FALSE;

    if (rig_parse_rprt_code(reply, &code) && code == -6)
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

    rig_term_log(ctrl, prefix, "%s", line);
}

static void rig_logs_toggle_cb(GtkToggleButton *button, gpointer data)
{
    GtkRigCtrl *ctrl = GTK_RIG_CTRL(data);
    gboolean visible;

    if (ctrl == NULL || ctrl->term_view == NULL)
        return;

    visible = gtk_toggle_button_get_active(button);
    gp_term_view_set_visible(ctrl->term_view, visible);
    rigctrl_schedule_resize(ctrl);
}

static void rigctrl_force_toplevel_resize(GtkRigCtrl *ctrl)
{
    GtkWidget *toplevel;

    if (ctrl == NULL)
        return;

    toplevel = gtk_widget_get_toplevel(GTK_WIDGET(ctrl));
    if (!GTK_IS_WINDOW(toplevel))
        return;

    gtk_widget_set_size_request(toplevel, -1, -1);
    gtk_widget_queue_resize(toplevel);
    gtk_window_resize(GTK_WINDOW(toplevel), 1, 1);
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

static void rigctrl_combo_set_active_blocked(GtkComboBox *box, gint index,
                                             GCallback cb, gpointer data)
{
    if (box == NULL)
        return;

    g_signal_handlers_block_by_func(box, cb, data);
    gtk_combo_box_set_active(box, index);
    g_signal_handlers_unblock_by_func(box, cb, data);
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
} RigStatusInfo;

static gboolean rig_status_idle(gpointer data)
{
    RigStatusInfo *info = data;
    GtkRigCtrl *ctrl;
    const gchar *label_text;

    if (info == NULL)
        return G_SOURCE_REMOVE;

    ctrl = info->ctrl;
    if (ctrl != NULL && ctrl->status_label != NULL)
    {
        label_text = info->text ? info->text :
            (info->is_error ? _("Error") : _("OK"));
        gtk_label_set_text(GTK_LABEL(ctrl->status_label), label_text);
        ctrl->cmd_error = info->is_error;
    }

    g_free(info->text);
    g_free(info);
    return G_SOURCE_REMOVE;
}

static void rigctrl_schedule_status(GtkRigCtrl *ctrl,
                                    const gchar *text,
                                    gboolean is_error)
{
    RigStatusInfo *info;

    if (ctrl == NULL || ctrl->status_label == NULL)
        return;

    if (!is_error && !ctrl->cmd_error)
        return;

    info = g_new0(RigStatusInfo, 1);
    info->ctrl = ctrl;
    info->text = g_strdup(text);
    info->is_error = is_error;
    g_idle_add(rig_status_idle, info);
}

static void rig_show_conn_error(GtkRigCtrl *ctrl,
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

static void rigctrl_reset_reconnect(GtkRigCtrl *ctrl, gboolean secondary)
{
    if (ctrl == NULL)
        return;

    if (secondary)
    {
        ctrl->reconnect_backoff_ms2 = 0;
        ctrl->reconnect_next_us2 = 0;
        ctrl->tx_conn_error_reported = FALSE;
    }
    else
    {
        ctrl->reconnect_backoff_ms = 0;
        ctrl->reconnect_next_us = 0;
        ctrl->rx_conn_error_reported = FALSE;
    }
}

static void rigctrl_reset_error_gates(GtkRigCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    /* Reset retry/error gating only when the user engages again. */
    rigctrl_reset_reconnect(ctrl, FALSE);
    rigctrl_reset_reconnect(ctrl, TRUE);

    if (ctrl->autostart_error_reported != NULL)
        g_hash_table_remove_all(ctrl->autostart_error_reported);
    if (ctrl->missing_model_reported != NULL)
        g_hash_table_remove_all(ctrl->missing_model_reported);
}

static gboolean rigctrl_reconnect_due(GtkRigCtrl *ctrl, gboolean secondary,
                                      gint64 now_us)
{
    gint64 next_us;

    if (ctrl == NULL)
        return FALSE;

    /* Pause retry attempts while the config editor is open. */
    if (rigctrl_editing_for_role(ctrl, secondary))
        return FALSE;

    next_us = secondary ? ctrl->reconnect_next_us2 : ctrl->reconnect_next_us;
    if (next_us <= 0)
        return TRUE;

    return (now_us >= next_us);
}

static void rigctrl_schedule_reconnect(GtkRigCtrl *ctrl, gboolean secondary,
                                       const gchar *role)
{
    gint    backoff;
    gint64  now_us;
    gint   *backoff_ptr;
    gint64 *next_ptr;

    if (ctrl == NULL)
        return;

    backoff_ptr = secondary ? &ctrl->reconnect_backoff_ms2
                            : &ctrl->reconnect_backoff_ms;
    next_ptr = secondary ? &ctrl->reconnect_next_us2
                         : &ctrl->reconnect_next_us;

    if (*backoff_ptr <= 0)
        backoff = RIGCTRL_RECONNECT_BACKOFF_MIN_MS;
    else
        backoff = MIN(*backoff_ptr * 2, RIGCTRL_RECONNECT_BACKOFF_MAX_MS);

    *backoff_ptr = backoff;
    now_us = g_get_monotonic_time();
    *next_ptr = now_us + ((gint64) backoff * 1000);

    sat_log_log(SAT_LOG_LEVEL_INFO,
                _("%s: scheduling %s reconnect in %d ms"),
                __func__, role ? role : _("rig"), backoff);
    rig_term_log(ctrl, "gpredict",
                 "reconnect %s in %d ms",
                 role ? role : "rig", backoff);
}

static GtkBoxClass *parent_class = NULL;

static void gtk_rig_ctrl_destroy(GtkWidget * widget)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(widget);

    if (ctrl->rigctl_thread != NULL)
    {
        g_mutex_lock(&ctrl->widgetsync);

        ctrl->engaged = 0;
        setconfig(ctrl);

        /* synchronization */
        g_cond_wait(&ctrl->widgetready, &ctrl->widgetsync);
        g_mutex_unlock(&ctrl->widgetsync);
        ctrl->rigctl_thread = NULL;
    }

    rigctld_terminate_spawned(ctrl, TRUE, &ctrl->rigctld_mgr2);
    rigctld_terminate_spawned(ctrl, FALSE, &ctrl->rigctld_mgr);

    if (ctrl->term_view != NULL)
    {
        gp_term_view_free(ctrl->term_view);
        ctrl->term_view = NULL;
    }
    ctrl->log_toggle = NULL;
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
    ctrl->prev_ele = 0.0;
    ctrl->sock = -1;
    ctrl->sock2 = -1;
    ctrl->reconnect_backoff_ms = 0;
    ctrl->reconnect_backoff_ms2 = 0;
    ctrl->reconnect_next_us = 0;
    ctrl->reconnect_next_us2 = 0;
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
    ctrl->term_view = gp_term_view_new(_("Follow tail"), TRUE, FALSE);
    ctrl->log_toggle = NULL;
    ctrl->resize_idle_id = 0;
    ctrl->primary_rig_id = NULL;
    ctrl->secondary_rig_id = NULL;
    ctrl->status_label = NULL;
    ctrl->cmd_error = FALSE;
    g_mutex_init(&(ctrl->busy));
    ctrl->engaged = FALSE;
    ctrl->engage_pending = FALSE;
    ctrl->delay = 1000;
    ctrl->timerid = 0;
    ctrl->errcnt = 0;
    ctrl->lastrxptt = FALSE;
    ctrl->lasttxptt = TRUE;
    ctrl->lastrxf = 0.0;
    ctrl->lasttxf = 0.0;
    ctrl->user_base_down_hz = 0.0;
    ctrl->user_base_up_hz = 0.0;
    ctrl->doppler_down_hz = 0.0;
    ctrl->doppler_up_hz = 0.0;
    ctrl->rig_target_down_hz = 0.0;
    ctrl->rig_target_up_hz = 0.0;
    ctrl->rig_actual_down_hz = 0.0;
    ctrl->rig_actual_up_hz = 0.0;
    ctrl->last_valid_target_down_hz = 0.0;
    ctrl->last_valid_target_up_hz = 0.0;
    ctrl->user_edit_down = FALSE;
    ctrl->user_edit_up = FALSE;
    ctrl->suppress_user_base = FALSE;
    ctrl->last_toggle_tx = -1;
}

GType gtk_rig_ctrl_get_type()
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
    gdouble         targettime;
    gdouble         delta;
    gchar          *buff;
    guint           h, m, s;
    gchar          *aoslos;

    /* select AOS or LOS time depending on target elevation */
    if (ctrl->target->el < 0.0)
    {
        targettime = ctrl->target->aos;
        aoslos = g_strdup_printf(_("AOS in"));
    }
    else
    {
        targettime = ctrl->target->los;
        aoslos = g_strdup_printf(_("LOS in"));
    }

    delta = targettime - t;

    /* convert julian date to seconds */
    s = (guint) (delta * 86400);

    /* extract hours */
    h = (guint) floor(s / 3600);
    s -= 3600 * h;

    /* extract minutes */
    m = (guint) floor(s / 60);
    s -= 60 * m;

    if (h > 0)
        buff =
            g_strdup_printf
            ("<span size='xx-large'><b>%s %02d:%02d:%02d</b></span>", aoslos,
             h, m, s);
    else
        buff =
            g_strdup_printf("<span size='xx-large'><b>%s %02d:%02d</b></span>",
                            aoslos, m, s);

    gtk_label_set_markup(GTK_LABEL(ctrl->SatCnt), buff);

    g_free(buff);
    g_free(aoslos);
}

static void rigctrl_set_user_base_freq(GtkRigCtrl *ctrl,
                                       gboolean downlink,
                                       gdouble hz,
                                       gboolean mark_manual)
{
    if (ctrl == NULL)
        return;

    ctrl->suppress_user_base = TRUE;
    if (downlink)
        gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->SatFreqDown), hz);
    else
        gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->SatFreqUp), hz);
    ctrl->suppress_user_base = FALSE;

    if (downlink)
        ctrl->user_base_down_hz = hz;
    else
        ctrl->user_base_up_hz = hz;

    if (mark_manual)
    {
        if (downlink)
            ctrl->user_edit_down = TRUE;
        else
            ctrl->user_edit_up = TRUE;
    }
}

static gdouble rigctrl_get_user_base_freq(GtkRigCtrl *ctrl,
                                          gboolean downlink)
{
    gdouble value = downlink ? ctrl->user_base_down_hz
                             : ctrl->user_base_up_hz;

    if (value <= 0.0)
    {
        GtkWidget *knob = downlink ? ctrl->SatFreqDown : ctrl->SatFreqUp;
        if (knob != NULL)
            value = gtk_freq_knob_get_value(GTK_FREQ_KNOB(knob));
    }

    return value;
}

static gboolean rigctrl_sat_freq_within_limits(const GtkRigCtrl *ctrl,
                                               gboolean downlink,
                                               gdouble sat_freq)
{
    gdouble min = 0.0;
    gdouble max = 0.0;

    if (ctrl == NULL || ctrl->trsp == NULL)
        return TRUE;

    if (downlink)
    {
        if (ctrl->trsp->downlow > 0.0 && ctrl->trsp->downhigh > 0.0)
        {
            min = MIN(ctrl->trsp->downlow, ctrl->trsp->downhigh);
            max = MAX(ctrl->trsp->downlow, ctrl->trsp->downhigh);
        }
    }
    else
    {
        if (ctrl->trsp->uplow > 0.0 && ctrl->trsp->uphigh > 0.0)
        {
            min = MIN(ctrl->trsp->uplow, ctrl->trsp->uphigh);
            max = MAX(ctrl->trsp->uplow, ctrl->trsp->uphigh);
        }
    }

    if (min <= 0.0 || max <= 0.0)
        return TRUE;

    return sat_freq >= min && sat_freq <= max;
}

static gdouble rigctrl_adjust_target(GtkRigCtrl *ctrl,
                                     gboolean downlink,
                                     gdouble base_sat,
                                     gdouble doppler,
                                     gdouble target_rig,
                                     gboolean *target_ok_out)
{
    gdouble sat_target = base_sat + doppler;
    gboolean ok = rigctrl_sat_freq_within_limits(ctrl, downlink, sat_target);
    gdouble last_valid = downlink ? ctrl->last_valid_target_down_hz
                                  : ctrl->last_valid_target_up_hz;

    if (!ok)
    {
        sat_log_log(SAT_LOG_LEVEL_WARN,
                    "rig update: side=%s target out of range sat=%.0f",
                    downlink ? "RX" : "TX", sat_target);
        if (target_ok_out)
            *target_ok_out = FALSE;
        if (last_valid > 0.0)
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

static gdouble rigctrl_compute_target(GtkRigCtrl *ctrl,
                                      gboolean downlink,
                                      gdouble lo,
                                      gboolean use_rit_xit,
                                      gdouble *base_sat_out,
                                      gdouble *doppler_out,
                                      gboolean *target_ok_out)
{
    gdouble base_sat = 0.0;
    gdouble doppler = 0.0;
    gdouble target_rig = 0.0;

    if (ctrl == NULL)
    {
        if (target_ok_out)
            *target_ok_out = FALSE;
        return 0.0;
    }

    base_sat = rigctrl_get_user_base_freq(ctrl, downlink);
    if (ctrl->tracking && !use_rit_xit)
        doppler = downlink ? ctrl->dd : ctrl->du;

    target_rig = base_sat + doppler - lo;

    if (base_sat_out)
        *base_sat_out = base_sat;
    if (doppler_out)
        *doppler_out = doppler;

    return rigctrl_adjust_target(ctrl, downlink, base_sat,
                                 doppler, target_rig, target_ok_out);
}

static void rigctrl_show_log(GtkRigCtrl *ctrl)
{
    if (ctrl == NULL)
        return;

    if (ctrl->log_toggle != NULL)
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->log_toggle), TRUE);
    else if (ctrl->term_view != NULL)
        gp_term_view_set_visible(ctrl->term_view, TRUE);

    rigctrl_schedule_resize(ctrl);
}

static gboolean rigctrl_configure_trsp_popup_idle(gpointer data)
{
    GtkComboBox *combo = GTK_COMBO_BOX(data);
    AtkObject *popup_acc;
    GtkWidget *popup_widget = NULL;
    GtkWidget *scrolled = NULL;
    gint attempt = 0;

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

        gtk_widget_set_hexpand(popup_widget, FALSE);
        gtk_widget_set_vexpand(popup_widget, FALSE);

        scrolled = popup_widget;
        while (scrolled != NULL && !GTK_IS_SCROLLED_WINDOW(scrolled))
            scrolled = gtk_widget_get_parent(scrolled);

        tree = rigctrl_trsp_find_child(popup_widget, GTK_TYPE_TREE_VIEW);
        if (GTK_IS_SCROLLED_WINDOW(scrolled))
        {
            gtk_widget_set_hexpand(scrolled, FALSE);
            gtk_widget_set_vexpand(scrolled, FALSE);
            gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scrolled),
                                           GTK_POLICY_NEVER,
                                           GTK_POLICY_AUTOMATIC);
            gtk_scrolled_window_set_propagate_natural_height(
                GTK_SCROLLED_WINDOW(scrolled), FALSE);
            gtk_widget_set_size_request(scrolled, -1,
                                        RIGCTRL_TRSP_POPUP_MAX_HEIGHT);
        }

        if (GTK_IS_TREE_VIEW(tree))
        {
            gtk_tree_view_set_headers_visible(GTK_TREE_VIEW(tree), FALSE);
            gtk_tree_view_set_fixed_height_mode(GTK_TREE_VIEW(tree), TRUE);
        }

        if (tree != NULL)
            rigctrl_trsp_fix_expansion(popup_widget, tree);

        if (g_object_get_data(G_OBJECT(popup_widget), "rigctrl-popup-hooked") == NULL)
        {
            g_object_set_data(G_OBJECT(popup_widget), "rigctrl-popup-hooked",
                              GINT_TO_POINTER(1));
            g_signal_connect(popup_widget, "map",
                             G_CALLBACK(rigctrl_trsp_popup_show), NULL);
            g_signal_connect(popup_widget, "hide",
                             G_CALLBACK(rigctrl_trsp_popup_hide), NULL);
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

static gboolean rigctrl_trsp_combo_button_press(GtkWidget *widget,
                                                GdkEventButton *event,
                                                gpointer data)
{
    (void)event;
    (void)data;

    if (widget == NULL)
        return FALSE;

    g_idle_add(rigctrl_configure_trsp_popup_idle, g_object_ref(widget));
    return FALSE;
}

static void rigctrl_trsp_popup_show(GtkWidget *widget, gpointer data)
{
    GtkWidget *scrolled = widget;
    GtkWidget *tree = NULL;
    GtkAdjustment *vadj;
    GtkTreePath *path;
    gint popup_min = 0, popup_nat = 0;
    gint scrolled_min = 0, scrolled_nat = 0;
    gint list_min = 0, list_nat = 0;

    (void)data;

    if (widget == NULL)
        return;

    while (scrolled != NULL && !GTK_IS_SCROLLED_WINDOW(scrolled))
        scrolled = gtk_widget_get_parent(scrolled);

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
    }

    gtk_widget_queue_resize(scrolled);
    gtk_widget_queue_resize(widget);

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
}

static void rigctrl_trsp_popup_hide(GtkWidget *widget, gpointer data)
{
    (void)data;

    if (widget == NULL)
        return;

    sat_log_log(SAT_LOG_LEVEL_DEBUG, _("%s: trsp popup hide"), __func__);
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

static void rigctrl_trsp_fix_expansion(GtkWidget *widget, GtkWidget *list)
{
    GList *children = NULL;
    GList *entry = NULL;

    if (widget == NULL)
        return;

    if (widget != list)
    {
        gtk_widget_set_hexpand(widget, FALSE);
        gtk_widget_set_vexpand(widget, FALSE);
    }
    else
    {
        gtk_widget_set_hexpand(widget, TRUE);
        gtk_widget_set_vexpand(widget, TRUE);
    }

    if (!GTK_IS_CONTAINER(widget))
        return;

    children = gtk_container_get_children(GTK_CONTAINER(widget));
    for (entry = children; entry != NULL; entry = entry->next)
        rigctrl_trsp_fix_expansion(GTK_WIDGET(entry->data), list);
    g_list_free(children);
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
    gdouble         satfreq;
    gchar          *buff;

    g_mutex_lock(&ctrl->rig_ctrl_updatelock);

    if (ctrl->target)
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

        /* Doppler shift down */
        satfreq = rigctrl_get_user_base_freq(ctrl, TRUE);
        ctrl->dd = -satfreq * (ctrl->target->range_rate / 299792.4580); // Hz
        ctrl->doppler_down_hz = ctrl->dd;
        buff = g_strdup_printf("%.0f Hz", ctrl->dd);
        gtk_label_set_text(GTK_LABEL(ctrl->SatDopDown), buff);
        g_free(buff);

        /* Doppler shift up */
        satfreq = rigctrl_get_user_base_freq(ctrl, FALSE);
        ctrl->du = satfreq * (ctrl->target->range_rate / 299792.4580);  // Hz
        ctrl->doppler_up_hz = ctrl->du;
        buff = g_strdup_printf("%.0f Hz", ctrl->du);
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

    g_mutex_unlock(&ctrl->rig_ctrl_updatelock);
}


/*
 * Track the downlink frequency by setting the uplink frequency
 * according to the lower limit of the downlink passband.
 */
static void track_downlink(GtkRigCtrl * ctrl)
{
    gdouble         delta, down, up;

    if (ctrl->trsp == NULL)
        return;

    /* ensure that we have a usable transponder config */
    if ((ctrl->trsp->downlow > 0) && (ctrl->trsp->uplow > 0))
    {
        down = gtk_freq_knob_get_value(GTK_FREQ_KNOB(ctrl->SatFreqDown));
        delta = down - ctrl->trsp->downlow;

        if (ctrl->trsp->invert)
            up = ctrl->trsp->uphigh - delta;
        else
            up = ctrl->trsp->uplow + delta;

        rigctrl_set_user_base_freq(ctrl, FALSE, up, FALSE);
    }
}

/*
 * Track the uplink frequency by setting the downlink frequency
 * according to the offset from the lower limit on the uplink passband.
 */
static void track_uplink(GtkRigCtrl * ctrl)
{
    gdouble         delta, down, up;

    if (ctrl->trsp == NULL)
        return;

    /* ensure that we have a usable transponder config */
    if ((ctrl->trsp->downlow > 0) && (ctrl->trsp->uplow > 0))
    {
        up = gtk_freq_knob_get_value(GTK_FREQ_KNOB(ctrl->SatFreqUp));
        delta = up - ctrl->trsp->uplow;

        if (ctrl->trsp->invert)
            down = ctrl->trsp->downhigh - delta;
        else
            down = ctrl->trsp->downlow + delta;

        rigctrl_set_user_base_freq(ctrl, TRUE, down, FALSE);
    }
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
                gtk_combo_box_set_active(GTK_COMBO_BOX(ctrl->SatSel), i);
                break;
            }
        }
    }
}

static void downlink_changed_cb(GtkFreqKnob * knob, gpointer data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);

    (void)knob;

    if (ctrl->suppress_user_base)
        return;

    ctrl->user_base_down_hz =
        gtk_freq_knob_get_value(GTK_FREQ_KNOB(ctrl->SatFreqDown));
    ctrl->user_edit_down = TRUE;

    if (ctrl->trsplock)
        track_downlink(ctrl);
}

static void uplink_changed_cb(GtkFreqKnob * knob, gpointer data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);

    (void)knob;

    if (ctrl->suppress_user_base)
        return;

    ctrl->user_base_up_hz =
        gtk_freq_knob_get_value(GTK_FREQ_KNOB(ctrl->SatFreqUp));
    ctrl->user_edit_up = TRUE;

    if (ctrl->trsplock)
        track_uplink(ctrl);
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
    GtkWidget      *label;

    label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(label), _("<b> Downlink </b>"));
    frame = gtk_frame_new(NULL);
    gtk_frame_set_label_align(GTK_FRAME(frame), 0.5, 0.5);
    gtk_frame_set_label_widget(GTK_FRAME(frame), label);

    vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    gtk_container_set_border_width(GTK_CONTAINER(vbox), 10);
    hbox1 = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    hbox2 = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);

    /* satellite downlink frequency */
    ctrl->SatFreqDown = gtk_freq_knob_new(145890000.0, TRUE);
    g_signal_connect(ctrl->SatFreqDown, "freq-changed",
                     G_CALLBACK(downlink_changed_cb), ctrl);
    gtk_box_pack_start(GTK_BOX(vbox), ctrl->SatFreqDown, TRUE, TRUE, 0);

    /* Downlink doppler */
    label = gtk_label_new(_("Doppler:"));
    gtk_widget_set_tooltip_text(label,
                                _("The Doppler shift according to the range "
                                  "rate and the currently selected downlink "
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
    gtk_box_pack_start(GTK_BOX(hbox2), label, TRUE, TRUE, 0);
    ctrl->RigFreqDown = gtk_freq_knob_new(145890000.0, FALSE);
    gtk_box_pack_start(GTK_BOX(hbox2), ctrl->RigFreqDown, TRUE, TRUE, 0);

    /* finish packing ... */
    gtk_box_pack_start(GTK_BOX(vbox), hbox1, TRUE, TRUE, 10);
    gtk_box_pack_start(GTK_BOX(vbox), hbox2, TRUE, TRUE, 0);
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
    GtkWidget      *label;

    label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(label), _("<b> Uplink </b>"));
    frame = gtk_frame_new(NULL);
    gtk_frame_set_label_align(GTK_FRAME(frame), 0.5, 0.5);
    gtk_frame_set_label_widget(GTK_FRAME(frame), label);

    vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    gtk_container_set_border_width(GTK_CONTAINER(vbox), 10);
    hbox1 = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    hbox2 = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);

    /* satellite uplink frequency */
    ctrl->SatFreqUp = gtk_freq_knob_new(145890000.0, TRUE);
    g_signal_connect(ctrl->SatFreqUp, "freq-changed",
                     G_CALLBACK(uplink_changed_cb), ctrl);
    gtk_box_pack_start(GTK_BOX(vbox), ctrl->SatFreqUp, TRUE, TRUE, 0);

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
    gtk_box_pack_start(GTK_BOX(hbox2), label, TRUE, TRUE, 0);
    ctrl->RigFreqUp = gtk_freq_knob_new(145890000.0, FALSE);
    gtk_box_pack_start(GTK_BOX(hbox2), ctrl->RigFreqUp, TRUE, TRUE, 0);

    gtk_box_pack_start(GTK_BOX(vbox), hbox1, TRUE, TRUE, 10);
    gtk_box_pack_start(GTK_BOX(vbox), hbox2, TRUE, TRUE, 0);
    gtk_container_add(GTK_CONTAINER(frame), vbox);

    return frame;
}


static void load_trsp_list(GtkRigCtrl * ctrl)
{
    trsp_t         *trsp = NULL;
    guint           i, n;

    if (ctrl->trsplist != NULL)
    {
        n = g_slist_length(ctrl->trsplist);
        for (i = 0; i < n; i++)
            gtk_combo_box_text_remove(GTK_COMBO_BOX_TEXT(ctrl->TrspSel), 0);

        free_transponders(ctrl->trsplist);
        ctrl->trsp = NULL;
    }

    /* check if there is a target satellite */
    if (ctrl->target == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s:%s: GtkSatModule has no target satellite."),
                    __FILE__, __func__);
        return;
    }

    /* read transponders for new target */
    ctrl->trsplist = read_transponders(ctrl->target->tle.catnr);
    n = g_slist_length(ctrl->trsplist);
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                _("%s:%s: Satellite %d has %d transponder modes."),
                __FILE__, __func__, ctrl->target->tle.catnr, n);

    if (n == 0)
        return;

    for (i = 0; i < n; i++)
    {
        trsp = (trsp_t *) g_slist_nth_data(ctrl->trsplist, i);
        gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ctrl->TrspSel),
                                       trsp->name);

        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    _("%s:%s: Read transponder '%s' for satellite %d"),
                    __FILE__, __func__, trsp->name, ctrl->target->tle.catnr);
    }

    ctrl->trsp = (trsp_t *) g_slist_nth_data(ctrl->trsplist, 0);
    if (gtk_combo_box_get_active(GTK_COMBO_BOX(ctrl->TrspSel)) != 0)
        gtk_combo_box_set_active(GTK_COMBO_BOX(ctrl->TrspSel), 0);
}

static gboolean have_conf()
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
        load_trsp_list(ctrl);
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

/*
 * Manage "Tune" events
 *
 * @param button Pointer to the GtkButton that received the signal.
 * @param data Pointer to the GtkRigCtrl structure.
 *
 * This function is called when the user clicks on the Tune button next to the
 * transponder selector. When clicked, the radio controller will set the RX and TX
 * frequencies to the middle of the transponder uplink/downlink bands.
 *
 * To avoid conflicts with manual frequency changes on the radio, the sync between
 * RIG and GPREDICT is invalidated after the tuning operation is performed.
 */
static void rigctrl_apply_trsp_preset(GtkRigCtrl *ctrl, gboolean mark_manual)
{
    gdouble         freq;

    if (ctrl->trsp == NULL)
        return;

    /* tune downlink */
    if ((ctrl->trsp->downlow > 0) && (ctrl->trsp->downhigh > 0))
    {
        freq = ctrl->trsp->downlow +
            labs((long)ctrl->trsp->downhigh - (long)ctrl->trsp->downlow) / 2;
        rigctrl_set_user_base_freq(ctrl, TRUE, freq, mark_manual);

        /* invalidate RIG<->GPREDICT sync */
        ctrl->lastrxf = 0.0;
    }

    /* tune uplink */
    if ((ctrl->trsp->uplow > 0) && (ctrl->trsp->uphigh > 0))
    {
        freq = ctrl->trsp->uplow +
            labs((long)ctrl->trsp->uphigh - (long)ctrl->trsp->uplow) / 2;
        rigctrl_set_user_base_freq(ctrl, FALSE, freq, mark_manual);

        /* invalidate RIG<->GPREDICT sync */
        ctrl->lasttxf = 0.0;
    }
}

static void trsp_tune_cb(GtkButton * button, gpointer data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);

    (void)button;

    rigctrl_apply_trsp_preset(ctrl, TRUE);
}

/*
 * Called when a new transponder is selected.
 * It updates ctrl->trsp with the new selection and issues a "tune" event.
 */
static void trsp_selected_cb(GtkComboBox * box, gpointer data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);
    gint            i, n;
    gboolean        allow_autofill = TRUE;

    i = gtk_combo_box_get_active(box);
    n = g_slist_length(ctrl->trsplist);

    if (i == -1)
    {
        /* clear transponder data */
        ctrl->trsp = NULL;
    }
    else if (i < n)
    {
        ctrl->trsp = (trsp_t *) g_slist_nth_data(ctrl->trsplist, i);
        allow_autofill = !ctrl->user_edit_down && !ctrl->user_edit_up;
        if (allow_autofill)
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
static void trsp_lock_cb(GtkToggleButton * button, gpointer data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);

    ctrl->trsplock = gtk_toggle_button_get_active(button);

    /* set uplink according to downlink */
    if (ctrl->trsplock)
        track_downlink(ctrl);
}

static void track_toggle_cb(GtkToggleButton * button, gpointer data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);

    ctrl->tracking = gtk_toggle_button_get_active(button);
    sat_log_log(SAT_LOG_LEVEL_DEBUG, "SATMODE: tracking %s",
                ctrl->tracking ? "on" : "off");
    if (is_full_duplex_main_sub_configured(ctrl->conf))
    {
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "FULL-DUPLEX MAIN/SUB: downlink -> %s, uplink -> %s",
                    vfo_name(ctrl->conf->downlink_vfo),
                    vfo_name(ctrl->conf->uplink_vfo));
    }

    /* invalidate sync with radio */
    ctrl->lastrxf = 0.0;
    ctrl->lasttxf = 0.0;
}

/* Called when the user changes the value of the cycle delay */
static void delay_changed_cb(GtkSpinButton * spin, gpointer data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);

    ctrl->delay = (guint) gtk_spin_button_get_value(spin);
    if (ctrl->conf)
        ctrl->conf->cycle = ctrl->delay;

    if (ctrl->engaged)
        start_timer(ctrl);
}

static void primary_rig_selected_cb(GtkComboBox * box, gpointer data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);
    gchar          *buff;
    gchar          *selected_id;

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

        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ctrl->cycle_spin),
                                  ctrl->conf->cycle);

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

    if (!gtk_toggle_button_get_active(button))
    {
        /* Disengage: close socket / stop worker thread */
        gtk_widget_set_sensitive(ctrl->DevSel, TRUE);
        gtk_widget_set_sensitive(ctrl->DevSel2, TRUE);
        ctrl->engaged = FALSE;
        ctrl->engage_pending = FALSE;
        rig_term_log(ctrl, "gpredict", "disengage");

        /* Notify worker thread about the new configuration/state */
        if (ctrl->rigctlq != NULL)
            setconfig(ctrl);
        /* The worker thread will clean up and exit; we just clear the
         * handle so a new thread can be started on the next engage.
         */
        ctrl->rigctl_thread = NULL;
    }
    else
    {
        /* User-initiated engage: clear error gating for a fresh attempt. */
        rigctrl_reset_error_gates(ctrl);
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
            rigctrl_fail_engage(ctrl);
            return;
        }

        if (!rigctrl_validate_mode(ctrl, ctrl->conf, _("receiver")))
        {
            rigctrl_fail_engage(ctrl);
            return;
        }

        if (ctrl->conf2 != NULL &&
            !rigctrl_validate_mode(ctrl, ctrl->conf2, _("uplink")))
        {
            rigctrl_fail_engage(ctrl);
            return;
        }

        /* Engage: start worker thread */
        gtk_widget_set_sensitive(ctrl->DevSel, FALSE);
        gtk_widget_set_sensitive(ctrl->DevSel2, FALSE);
        ctrl->engaged = TRUE;
        rig_term_log(ctrl, "gpredict", "engage");

        /* Start worker thread if not already running */
        if (ctrl->rigctl_thread == NULL)
        {
            ctrl->rigctlq = g_async_queue_new();
            ctrl->rigctl_thread =
                g_thread_new("rigctl_run", rigctl_run, ctrl);
        }

        /* Push initial configuration to the worker */
        setconfig(ctrl);
    }

    /* Always clear secondary rig configuration when toggling engage
     * state; this mirrors the previous behaviour.
     */
    ctrl->conf2 = NULL;
}

static GtkWidget *create_target_widgets(GtkRigCtrl * ctrl)
{
    GtkWidget      *frame, *table, *label, *track;
    GtkWidget      *tune, *trsplock, *hbox;
    GtkWidget      *trsp_label;
    gchar          *buff;
    guint           i, n;
    sat_t          *sat = NULL;

    buff = g_strdup_printf(AZEL_FMTSTR, 0.0);

    table = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(table), 5);
    gtk_grid_set_column_spacing(GTK_GRID(table), 5);
    gtk_grid_set_row_spacing(GTK_GRID(table), 5);

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
    gtk_widget_set_tooltip_text(ctrl->SatSel, _("Select target object"));
    g_signal_connect(ctrl->SatSel, "changed", G_CALLBACK(sat_selected_cb),
                     ctrl);
    gtk_grid_attach(GTK_GRID(table), ctrl->SatSel, 0, 0, 3, 1);

    /* tracking button */
    track = gtk_toggle_button_new_with_label(_("Track"));
    gtk_widget_set_tooltip_text(track,
                                _("Track the satellite transponder.\n"
                                  "Enabling this button will apply Doppler "
                                  "correction to the frequency of the radio."));
    gtk_grid_attach(GTK_GRID(table), track, 3, 0, 1, 1);
    g_signal_connect(track, "toggled", G_CALLBACK(track_toggle_cb), ctrl);

    /* Service/payload preset selector, apply, and lock buttons */
    trsp_label = gtk_label_new(_("Service / Payload preset"));
    g_object_set(trsp_label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), trsp_label, 0, 1, 1, 1);

    ctrl->TrspSel = gtk_combo_box_text_new();
    gtk_combo_box_set_popup_fixed_width(GTK_COMBO_BOX(ctrl->TrspSel), TRUE);
    gtk_widget_set_tooltip_text(ctrl->TrspSel,
                                _("Select a service/payload preset. "
                                  "Use Apply preset to copy its frequencies."));
    load_trsp_list(ctrl);
    g_signal_connect(ctrl->TrspSel, "realize",
                     G_CALLBACK(rigctrl_trsp_combo_realize), NULL);
    g_signal_connect(ctrl->TrspSel, "button-press-event",
                     G_CALLBACK(rigctrl_trsp_combo_button_press), NULL);
    g_signal_connect(ctrl->TrspSel, "changed", G_CALLBACK(trsp_selected_cb),
                     ctrl);
    gtk_grid_attach(GTK_GRID(table), ctrl->TrspSel, 1, 1, 2, 1);

    /* buttons */
    tune = gtk_button_new_with_label(_("Apply preset"));
    gtk_widget_set_tooltip_text(tune,
                                _("Apply the selected preset frequencies. "
                                  "The uplink and downlink will be set to the "
                                  "center of the transponder passband. In case "
                                  "of beacons, only the downlink will be set "
                                  "to the beacon frequency."));
    g_signal_connect(tune, "clicked", G_CALLBACK(trsp_tune_cb), ctrl);

    trsplock = gtk_toggle_button_new_with_label(_("L"));
    gtk_widget_set_tooltip_text(trsplock,
                                _("Lock the uplink and the downlink to each "
                                  "other. Whenever you change the downlink "
                                  "(in the controller), the uplink will track "
                                  "it according to whether "
                                  "the transponder is inverting or not. "
                                  "Similarly, if you change the uplink the "
                                  "downlink will track it automatically.\n\n"
                                  "If the downlink and uplink are initially "
                                  "out of sync when you enable this function, "
                                  "the current downlink frequency will be used "
                                  "as baseline for setting the new uplink "
                                  "frequency."));
    g_signal_connect(trsplock, "toggled", G_CALLBACK(trsp_lock_cb), ctrl);

    /* box for packing buttons */
    hbox = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 5);
    gtk_box_pack_start(GTK_BOX(hbox), tune, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(hbox), trsplock, TRUE, TRUE, 0);
    gtk_grid_attach(GTK_GRID(table), hbox, 3, 1, 1, 1);

    /* Azimuth */
    label = gtk_label_new(_("Az:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 2, 1, 1);
    ctrl->SatAz = gtk_label_new(buff);
    g_object_set(ctrl->SatAz, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), ctrl->SatAz, 1, 2, 1, 1);

    /* Elevation */
    label = gtk_label_new(_("El:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 3, 1, 1);
    ctrl->SatEl = gtk_label_new(buff);
    g_object_set(ctrl->SatEl, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), ctrl->SatEl, 1, 3, 1, 1);

    /* Range */
    label = gtk_label_new(_(" Range:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 2, 2, 1, 1);
    ctrl->SatRng = gtk_label_new("0 km");
    g_object_set(ctrl->SatRng, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), ctrl->SatRng, 3, 2, 1, 1);

    gtk_widget_set_tooltip_text(label,
                                _("This is the current distance between the "
                                  "satellite and the observer."));
    gtk_widget_set_tooltip_text(ctrl->SatRng,
                                _("This is the current distance between the "
                                  "satellite and the observer."));

    /* Range rate */
    label = gtk_label_new(_(" Rate:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 2, 3, 1, 1);
    ctrl->SatRngRate = gtk_label_new("0.0 km/s");
    g_object_set(ctrl->SatRngRate, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), ctrl->SatRngRate, 3, 3, 1, 1);

    gtk_widget_set_tooltip_text(label,
                                _("The rate of change for the distance between"
                                  " the satellite and the observer."));
    gtk_widget_set_tooltip_text(ctrl->SatRngRate,
                                _("The rate of change for the distance between"
                                  " the satellite and the observer."));

    frame = gtk_frame_new(_("Target"));
    gtk_container_add(GTK_CONTAINER(frame), table);
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
        index = -1;
    else if (index < 0)
        index = 0;

    rigctrl_combo_set_active_blocked(box, index, cb, data);
}

static void rigctrl_rebuild_device_selectors(GtkRigCtrl *ctrl,
                                             gboolean rebuilt)
{
    GSList *rigs;
    GSList *tx_rigs;
    GSList *iter;

    if (ctrl == NULL || ctrl->DevSel == NULL || ctrl->DevSel2 == NULL)
        return;

    rigs = rigctrl_collect_rig_names(FALSE, TRUE);
    tx_rigs = rigctrl_collect_rig_names(TRUE, FALSE);

    g_signal_handlers_block_by_func(ctrl->DevSel,
                                    G_CALLBACK(primary_rig_selected_cb), ctrl);
    g_signal_handlers_block_by_func(ctrl->DevSel2,
                                    G_CALLBACK(secondary_rig_selected_cb), ctrl);

    gtk_combo_box_text_remove_all(GTK_COMBO_BOX_TEXT(ctrl->DevSel));
    gtk_combo_box_text_remove_all(GTK_COMBO_BOX_TEXT(ctrl->DevSel2));

    for (iter = rigs; iter != NULL; iter = iter->next)
        gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ctrl->DevSel),
                                       iter->data);

    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ctrl->DevSel2),
                                   _("None"));
    for (iter = tx_rigs; iter != NULL; iter = iter->next)
        gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ctrl->DevSel2),
                                       iter->data);

    rigctrl_combo_restore_selection(GTK_COMBO_BOX(ctrl->DevSel),
                                    ctrl->primary_rig_id, FALSE,
                                    G_CALLBACK(primary_rig_selected_cb), ctrl);
    rigctrl_combo_restore_selection(GTK_COMBO_BOX(ctrl->DevSel2),
                                    ctrl->secondary_rig_id, TRUE,
                                    G_CALLBACK(secondary_rig_selected_cb), ctrl);

    g_signal_handlers_unblock_by_func(ctrl->DevSel,
                                      G_CALLBACK(primary_rig_selected_cb), ctrl);
    g_signal_handlers_unblock_by_func(ctrl->DevSel2,
                                      G_CALLBACK(secondary_rig_selected_cb), ctrl);

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
    GtkWidget      *frame, *table, *label;

    table = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(table), 5);
    gtk_grid_set_column_spacing(GTK_GRID(table), 5);
    gtk_grid_set_row_spacing(GTK_GRID(table), 5);

    /* Primary device */
    label = gtk_label_new(_("1. Device:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 0, 1, 1);

    ctrl->DevSel = gtk_combo_box_text_new();
    gtk_widget_set_tooltip_text(ctrl->DevSel,
                                _("Select primary radio device."
                                  "This device will be used for downlink and "
                                  "uplink unless you select a secondary device"
                                  " for uplink"));

    /* Secondary device */
    label = gtk_label_new(_("2. Device (TX):"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 1, 1, 1);

    ctrl->DevSel2 = gtk_combo_box_text_new();
    gtk_widget_set_tooltip_text(ctrl->DevSel2,
                                _("Select secondary radio device\n"
                                  "This device will be used for uplink"));

    rigctrl_rebuild_device_selectors(ctrl, FALSE);

    g_signal_connect(ctrl->DevSel, "changed",
                     G_CALLBACK(primary_rig_selected_cb), ctrl);
    gtk_grid_attach(GTK_GRID(table), ctrl->DevSel, 1, 0, 1, 1);
    g_signal_connect(ctrl->DevSel2, "changed",
                     G_CALLBACK(secondary_rig_selected_cb), ctrl);
    gtk_grid_attach(GTK_GRID(table), ctrl->DevSel2, 1, 1, 1, 1);

    /* Logs toggle */
    ctrl->log_toggle = gtk_toggle_button_new_with_label(_("Show log"));
    gtk_widget_set_tooltip_text(ctrl->log_toggle,
                                _("Show or hide the radio control log"));
    g_signal_connect(ctrl->log_toggle, "toggled",
                     G_CALLBACK(rig_logs_toggle_cb), ctrl);
    if (ctrl->term_view != NULL)
        gp_term_view_set_visible(ctrl->term_view, FALSE);
    gtk_grid_attach(GTK_GRID(table), ctrl->log_toggle, 2, 1, 1, 1);

    /* Engage button */
    ctrl->LockBut = gtk_toggle_button_new_with_label(_("Engage"));
    gtk_widget_set_tooltip_text(ctrl->LockBut,
                                _("Engage the selected radio device"));
    g_signal_connect(ctrl->LockBut, "toggled", G_CALLBACK(rig_engaged_cb),
                     ctrl);
    gtk_grid_attach(GTK_GRID(table), ctrl->LockBut, 2, 0, 1, 1);

    /* cycle period */
    label = gtk_label_new(_("Cycle:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 3, 1, 1);

    ctrl->cycle_spin = gtk_spin_button_new_with_range(10, 10000, 10);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(ctrl->cycle_spin), 0);
    gtk_widget_set_tooltip_text(ctrl->cycle_spin,
                                _("This parameter controls the delay between "
                                  "commands sent to the rig."));
    g_signal_connect(ctrl->cycle_spin, "value-changed",
                     G_CALLBACK(delay_changed_cb), ctrl);
    gtk_grid_attach(GTK_GRID(table), ctrl->cycle_spin, 1, 3, 1, 1);

    label = gtk_label_new(_("msec"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 2, 3, 1, 1);

    /* status */
    label = gtk_label_new(_("Status:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 4, 1, 1);

    ctrl->status_label = gtk_label_new(_("OK"));
    g_object_set(ctrl->status_label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), ctrl->status_label, 1, 4, 2, 1);

    frame = gtk_frame_new(_("Settings"));
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

static gboolean _send_rigctld_command(GtkRigCtrl * ctrl, gint sock,
                                      gchar * buff, gchar * buffout,
                                      gint sizeout)
{
    gint            written;
    gint            size;
    gint            rprt = 0;
    gboolean        rprt_error = FALSE;

    size = strlen(buff);

    rig_term_log_tx(ctrl, buff);
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                _("%s:%s: sending %d bytes to rigctld as \"%s\""),
                __FILE__, __func__, size, buff);
    /* send command */
    written = send(sock, buff, strlen(buff), 0);
    if (written != size)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: SIZE ERROR %d / %d"), __func__, written, size);
    }
    if (written == -1)
    {
        gchar *trim_cmd = g_strdup(buff);
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: rigctld port closed"), __func__);
        rig_term_log(ctrl, "gpredict:err",
                     "send failed (%s) cmd=%s",
                     strerror(errno),
                     trim_cmd ? g_strchomp(trim_cmd) : "(null)");
        g_free(trim_cmd);
        rigctrl_schedule_status(ctrl, _("Command send failed"), TRUE);
        rigctrl_handle_socket_error(ctrl, sock, "send");
        return FALSE;
    }
    /* try to read answer */
    size = recv(sock, buffout, sizeout - 1, 0);
    if (size == -1)
    {
        gchar *trim_cmd = g_strdup(buff);
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: rigctld port closed"), __func__);
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            rig_term_log(ctrl, "gpredict:err",
                         "timeout waiting for rigctld reply cmd=%s",
                         trim_cmd ? g_strchomp(trim_cmd) : "(null)");
        else
            rig_term_log(ctrl, "gpredict:err",
                         "recv failed (%s) cmd=%s",
                         strerror(errno),
                         trim_cmd ? g_strchomp(trim_cmd) : "(null)");
        g_free(trim_cmd);
        rigctrl_schedule_status(ctrl, _("Command receive failed"), TRUE);
        rigctrl_handle_socket_error(ctrl, sock, "recv");
        return FALSE;
    }

    buffout[size] = '\0';
    if (size == 0)
    {
        gchar *trim_cmd = g_strdup(buff);
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%s: Got 0 bytes from rigctld"), __FILE__, __func__);
        rig_term_log(ctrl, "gpredict:err",
                     "rigctld closed connection cmd=%s",
                     trim_cmd ? g_strchomp(trim_cmd) : "(null)");
        g_free(trim_cmd);
        rigctrl_schedule_status(ctrl, _("Rigctld closed connection"), TRUE);
        rigctrl_handle_socket_error(ctrl, sock, "recv");
        return FALSE;
    }
    else
    {
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    _("%s:%s: Read %d bytes from rigctld"),
                    __FILE__, __func__, size);
    }
    ctrl->wrops++;

    rig_term_log_rx(ctrl, buffout);
    if (rig_parse_rprt_code(buffout, &rprt) && rprt != 0)
    {
        gchar *trim_cmd = g_strdup(buff);
        gchar *trim_reply = g_strdup(buffout);
        char status_msg[64];

        rprt_error = TRUE;
        rig_term_log_err_rprt(ctrl, buff, buffout, rprt);

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
        rigctrl_schedule_status(ctrl, status_msg, TRUE);

        g_free(trim_cmd);
        g_free(trim_reply);
    }

    if (!rprt_error)
        rigctrl_schedule_status(ctrl, _("OK"), FALSE);

    return TRUE;
}

static gboolean send_rigctld_command(GtkRigCtrl * ctrl, gint sock,
                                     gchar * buff, gchar * buffout,
                                     gint sizeout)
{
    gboolean        retval;

    /* Enter critical section! */
    g_mutex_lock(&ctrl->writelock);

    retval = _send_rigctld_command(ctrl, sock, buff, buffout, sizeout);

    /* Leave critical section! */
    g_mutex_unlock(&ctrl->writelock);
    return (retval);
}

static inline gboolean check_set_response(gchar * buffback, gboolean retcode,
                                          const gchar * function)
{
    if (retcode == TRUE)
    {
        if (strncmp(buffback, "RPRT 0", 6) != 0)
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
        if (strncmp(buffback, "RPRT", 4) == 0)
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

static void rigctrl_log_config(GtkRigCtrl *ctrl,
                               const radio_conf_t *conf,
                               const gchar *role)
{
    const gchar *host;
    gint port;
    const gchar *label = (role != NULL) ? role : _("rig");

    if (ctrl == NULL || conf == NULL)
        return;

    host = conf->host ? conf->host : "(null)";
    port = conf->port;

    rig_term_log(ctrl, "gpredict",
                 "rig config (%s): model=%s mode=%s host=%s port=%d downlink_vfo=%s uplink_vfo=%s",
                 label,
                 radio_model_to_string(conf->radio_model),
                 radio_mode_to_string(conf->radio_mode),
                 host, port,
                 vfo_name(conf->downlink_vfo),
                 vfo_name(conf->uplink_vfo));
    sat_log_log(SAT_LOG_LEVEL_INFO,
                _("%s: rig config (%s) model=%s mode=%s host=%s port=%d"),
                __func__, label,
                radio_model_to_string(conf->radio_model),
                radio_mode_to_string(conf->radio_mode),
                host, port);
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

typedef enum {
    VFO_ROLE_DOWNLINK = 0,
    VFO_ROLE_UPLINK
} vfo_role_t;

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

static gboolean satmode_vfo_for_role(const radio_conf_t *conf,
                                     vfo_role_t role, vfo_t *vfo)
{
    if (!is_full_duplex_main_sub_configured(conf) || vfo == NULL)
        return FALSE;

    *vfo = (role == VFO_ROLE_DOWNLINK) ? conf->downlink_vfo : conf->uplink_vfo;
    return TRUE;
}

static gboolean select_satmode_vfo(GtkRigCtrl *ctrl, gint sock,
                                   vfo_role_t role, const gchar *action,
                                   gdouble freq, gboolean log_freq)
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
                        "FULL-DUPLEX MAIN/SUB: using %s for %s %.0f",
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
                               gint vfo_key, gdouble freq)
{
    gchar *key = NULL;
    gdouble *value = NULL;

    if (sock < 0 || freq <= 0.0)
        return;

    if (rig_freq_cache == NULL)
    {
        rig_freq_cache = g_hash_table_new_full(g_str_hash, g_str_equal,
                                               g_free, g_free);
    }

    key = g_strdup_printf("%d:%s:%d", sock, tag ? tag : "", vfo_key);
    value = g_new(gdouble, 1);
    *value = freq;
    g_hash_table_replace(rig_freq_cache, key, value);
}

static gboolean rigctrl_get_cached_freq(gint sock, const gchar *tag,
                                        gint vfo_key, gdouble *freq_out)
{
    gchar key[64];
    gdouble *value = NULL;

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
                                     gdouble freq, vfo_t vfo)
{
    gchar          *buff;
    gchar           buffback[128];
    gboolean        retcode;

    if (!ctrl->conf->vfo_opt)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    "FULL-DUPLEX MAIN/SUB: vfo_opt disabled; sending explicit VFO");
    }

    buff = g_strdup_printf("F %s %10.0f\x0a", vfo_name(vfo), freq);

    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "FULL-DUPLEX MAIN/SUB: set %s %.0f -> %s",
                vfo_name(vfo), freq, buffback);
    g_free(buff);

    return check_set_response(buffback, retcode, __func__);
}

static gboolean get_freq_simplex_vfo_internal(GtkRigCtrl *ctrl, gint sock,
                                              gdouble *freq, vfo_t vfo,
                                              gboolean allow_cache)
{
    gchar          *buff;
    gchar           buffback[128];
    gboolean        retcode;
    gboolean        retval = TRUE;
    gchar         **vbuff;

    if (!ctrl->conf->vfo_opt)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    "FULL-DUPLEX MAIN/SUB: vfo_opt disabled; requesting explicit VFO");
    }

    buff = g_strdup_printf("f %s\x0a", vfo_name(vfo));

    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    retcode = check_get_response(buffback, retcode, __func__);
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "FULL-DUPLEX MAIN/SUB: get %s -> %s",
                vfo_name(vfo), buffback);
    if (retcode)
    {
        vbuff = g_strsplit(buffback, "\n", 3);
        if (vbuff[0])
            *freq = g_ascii_strtod(vbuff[0], NULL);
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
                    "FULL-DUPLEX MAIN/SUB: get %s failed; using cached %.0f",
                    vfo_name(vfo), *freq);
        return TRUE;
    }

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "FULL-DUPLEX MAIN/SUB: get %s failed",
                vfo_name(vfo));
    return FALSE;
}

static gboolean get_freq_simplex_vfo(GtkRigCtrl *ctrl, gint sock,
                                     gdouble *freq, vfo_t vfo)
{
    return get_freq_simplex_vfo_internal(ctrl, sock, freq, vfo, TRUE);
}

static gboolean get_freq_simplex_vfo_strict(GtkRigCtrl *ctrl, gint sock,
                                            gdouble *freq, vfo_t vfo)
{
    return get_freq_simplex_vfo_internal(ctrl, sock, freq, vfo, FALSE);
}

static gboolean set_freq_toggle_vfo(GtkRigCtrl *ctrl, gint sock,
                                    gdouble freq, vfo_t vfo)
{
    gchar          *buff;
    gchar           buffback[128];
    gboolean        retcode;

    if (!ctrl->conf->vfo_opt)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    "FULL-DUPLEX MAIN/SUB: vfo_opt disabled; sending explicit VFO");
    }

    buff = g_strdup_printf("I %s %10.0f\x0a", vfo_name(vfo), freq);

    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "FULL-DUPLEX MAIN/SUB: set %s (toggle) %.0f -> %s",
                vfo_name(vfo), freq, buffback);
    g_free(buff);

    return check_set_response(buffback, retcode, __func__);
}

static gboolean get_freq_toggle_vfo_internal(GtkRigCtrl *ctrl, gint sock,
                                             gdouble *freq, vfo_t vfo,
                                             gboolean allow_cache)
{
    gchar          *buff;
    gchar           buffback[128];
    gboolean        retcode;
    gboolean        retval = TRUE;
    gchar         **vbuff;

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

    buff = g_strdup_printf("i %s\x0a", vfo_name(vfo));

    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    retcode = check_get_response(buffback, retcode, __func__);
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "FULL-DUPLEX MAIN/SUB: get %s (toggle) -> %s",
                vfo_name(vfo), buffback);
    if (retcode)
    {
        vbuff = g_strsplit(buffback, "\n", 3);
        if (vbuff[0])
            *freq = g_ascii_strtod(vbuff[0], NULL);
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
                    "FULL-DUPLEX MAIN/SUB: get %s (toggle) failed; using cached %.0f",
                    vfo_name(vfo), *freq);
        return TRUE;
    }

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "FULL-DUPLEX MAIN/SUB: get %s (toggle) failed",
                vfo_name(vfo));
    return FALSE;
}

static gboolean get_freq_toggle_vfo(GtkRigCtrl *ctrl, gint sock,
                                    gdouble *freq, vfo_t vfo)
{
    return get_freq_toggle_vfo_internal(ctrl, sock, freq, vfo, TRUE);
}

static gboolean get_freq_toggle_vfo_strict(GtkRigCtrl *ctrl, gint sock,
                                           gdouble *freq, vfo_t vfo)
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

    if (ctrl->conf == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: Controller does not have a valid configuration"),
                    __func__);
        return FALSE;
    }

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
    gdouble         readfreq = 0.0;
    gdouble         tmpfreq = 0.0;
    gdouble         base_sat = 0.0;
    gboolean        ptt = FALSE;
    gboolean        use_rit_xit =
        (ctrl->conf != NULL) ? ctrl->conf->supports_rit_xit : FALSE;
    vfo_t           sat_vfo = VFO_MAIN;
    gboolean        use_sat_vfo =
        satmode_vfo_for_role(ctrl->conf, VFO_ROLE_DOWNLINK, &sat_vfo);
    gdouble         base_freq = 0.0;
    gdouble         doppler_hz = 0.0;
    gdouble         sent_freq = 0.0;
    gdouble         target_up = 0.0;
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
    if ((ctrl->engaged) && (ctrl->lastrxf > 0.0) && (ptt == FALSE))
    {
        if (!select_satmode_vfo(ctrl, ctrl->sock, VFO_ROLE_DOWNLINK,
                                "reading downlink freq", 0.0, FALSE))
            ctrl->errcnt++;
        if (use_sat_vfo)
        {
            if (!get_freq_simplex_vfo(ctrl, ctrl->sock, &readfreq, sat_vfo))
                readfreq = ctrl->lastrxf;
        }
        else if (!get_freq_simplex(ctrl, ctrl->sock, &readfreq))
        {
            /* error => use a passive value */
            readfreq = ctrl->lastrxf;
        }
        else if (fabs(readfreq - ctrl->lastrxf) >= 1.0)
        {
            /* user might have altered radio frequency => update rig readback */
            gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqDown),
                                    readfreq);
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
    ctrl->rig_target_down_hz = tmpfreq;
    ctrl->rig_target_up_hz = target_up;

    gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqDown), tmpfreq);
    gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp), target_up);

    base_freq = base_sat - ctrl->conf->lo;
    sent_freq = tmpfreq;

    /* if device is engaged, send freq command to radio */
    if ((ctrl->engaged) && (ptt == FALSE) && target_ok &&
        (fabs(ctrl->lastrxf - tmpfreq) >= 1.0))
    {
        {
            gboolean set_ok;
            gboolean read_ok;
            gdouble  readback = 0.0;
            const gchar *vfo_label =
                use_sat_vfo ? vfo_name(sat_vfo) :
                (ctrl->conf->vfo_opt ? "currVFO" : "default");

            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rig update: side=RX base=%.0f doppler=%.0f sent=%.0f vfo=%s",
                        base_freq, doppler_hz, sent_freq, vfo_label);

            set_ok = select_satmode_vfo(ctrl, ctrl->sock, VFO_ROLE_DOWNLINK,
                                        "setting downlink freq", tmpfreq, TRUE) &&
                (use_sat_vfo ?
                 set_freq_simplex_vfo(ctrl, ctrl->sock, tmpfreq, sat_vfo) :
                 set_freq_simplex(ctrl, ctrl->sock, tmpfreq));
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
                read_ok = use_sat_vfo ?
                    get_freq_simplex_vfo_strict(ctrl, ctrl->sock, &readback, sat_vfo) :
                    get_freq_simplex_strict(ctrl, ctrl->sock, &readback);
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            "rig update: side=RX readback=%.0f ok=%d",
                            readback, read_ok ? 1 : 0);

                if (!read_ok)
                {
                    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                                "rig update: side=RX readback failed; keeping last");
                }
                else if (fabs(readback - sent_freq) <= 100.0)
                {
                    ctrl->errcnt = 0;
                    ctrl->lastrxf = readback;
                    ctrl->rig_actual_down_hz = readback;
                    gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqDown),
                                            readback);

                    /* This is only effective in RIG_TYPE_TRX mode.
                       Invalidate ctrl->lasttxf for two reasons.

                       1. Prevent dial feedback from changing the uplink frequency.
                       In the first TX cycle get_freq_simplex() returns the downlink
                       frequency instead of uplink. The mismatch would thus trigger
                       an uplink update as long as the VFO has not been updated.
                       2. Force updating the VFO in the first TX cycle.
                     */
                    if (ctrl->lastrxptt != ptt)
                        ctrl->lasttxf = 0.0;
                }
                else
                {
                    sat_log_log(SAT_LOG_LEVEL_ERROR,
                                "rig update: side=RX readback mismatch (sent=%.0f read=%.0f); tracking unsynced",
                                sent_freq, readback);
                    ctrl->errcnt++;
                    ctrl->lastrxf = 0.0;
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
    gdouble         readfreq = 0.0;
    gdouble         tmpfreq = 0.0;
    gdouble         base_sat = 0.0;
    gboolean        ptt = TRUE;
    gboolean        use_rit_xit =
        (ctrl->conf != NULL) ? ctrl->conf->supports_rit_xit : FALSE;
    gdouble         base_freq = 0.0;
    gdouble         doppler_hz = 0.0;
    gdouble         sent_freq = 0.0;
    gdouble         target_down = 0.0;
    gboolean        target_ok = TRUE;

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
    if ((ctrl->engaged) && (ctrl->lasttxf > 0.0) && (ptt == TRUE))
    {
        if (!get_freq_simplex(ctrl, ctrl->sock, &readfreq))
        {
            /* error => use a passive value */
            readfreq = ctrl->lasttxf;
        }
        else if (fabs(readfreq - ctrl->lasttxf) >= 1.0)
        {
            /* user might have altered radio frequency => update rig readback */
            gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp), readfreq);
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
    ctrl->rig_target_up_hz = tmpfreq;
    ctrl->rig_target_down_hz = target_down;

    gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp), tmpfreq);
    gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqDown), target_down);

    base_freq = base_sat - ctrl->conf->loup;
    sent_freq = tmpfreq;

    /* if device is engaged, send freq command to radio */
    if ((ctrl->engaged) && (ptt == TRUE) && target_ok &&
        (fabs(ctrl->lasttxf - tmpfreq) >= 1.0))
    {
        {
            gboolean set_ok;
            gboolean read_ok;
            gdouble  readback = 0.0;
            const gchar *vfo_label =
                ctrl->conf->vfo_opt ? "currVFO" : "default";

            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rig update: side=TX base=%.0f doppler=%.0f sent=%.0f vfo=%s",
                        base_freq, doppler_hz, sent_freq, vfo_label);

            set_ok = set_freq_simplex(ctrl, ctrl->sock, tmpfreq);
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
                read_ok = get_freq_simplex_strict(ctrl, ctrl->sock, &readback);
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            "rig update: side=TX readback=%.0f ok=%d",
                            readback, read_ok ? 1 : 0);

                if (!read_ok)
                {
                    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                                "rig update: side=TX readback failed; keeping last");
                }
                else if (fabs(readback - sent_freq) <= 100.0)
                {
                    ctrl->errcnt = 0;
                    ctrl->lasttxf = readback;
                    ctrl->rig_actual_up_hz = readback;
                    gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp),
                                            readback);

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
                        ctrl->lastrxf = 0.0;
                }
                else
                {
                    sat_log_log(SAT_LOG_LEVEL_ERROR,
                                "rig update: side=TX readback mismatch (sent=%.0f read=%.0f); tracking unsynced",
                                sent_freq, readback);
                    ctrl->errcnt++;
                    ctrl->lasttxf = 0.0;
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
    gdouble         tmpfreq = 0.0;
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
                                     use_rit_xit, NULL, NULL, &target_ok);
    ctrl->rig_target_up_hz = tmpfreq;
    gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp), tmpfreq);

    /* if device is engaged, send freq command to radio */
    if ((ctrl->engaged) && target_ok &&
        (fabs(ctrl->lasttxf - tmpfreq) >= 10.0))
    {
        if (set_freq_toggle(ctrl, ctrl->sock, tmpfreq))
        {
            /* reset error counter */
            ctrl->errcnt = 0;
        }
        else
        {
            ctrl->errcnt++;
        }

        /* store the last sent frequency even if an error occurred */
        ctrl->lasttxf = tmpfreq;
    }

}

static void exec_full_duplex_main_sub_cycle(GtkRigCtrl * ctrl)
{
    rig_mode_dispatch_t plan;
    gdouble         base_sat_down = 0.0;
    gdouble         base_sat_up = 0.0;
    gdouble         rigfreqd;
    gdouble         rigfrequ;
    gdouble         doppler_down = 0.0;
    gdouble         doppler_up = 0.0;
    gdouble         readback = 0.0;
    gboolean        set_ok;
    gboolean        read_ok;
    gboolean        down_ok = TRUE;
    gboolean        up_ok = TRUE;

    if (ctrl == NULL || ctrl->conf == NULL)
        return;

    rig_mode_dispatch(ctrl->conf->radio_mode,
                      ctrl->conf->downlink_vfo,
                      ctrl->conf->uplink_vfo,
                      &plan);
    if (!plan.send_downlink && !plan.send_uplink)
        return;

    rigfreqd = rigctrl_compute_target(ctrl, TRUE, ctrl->conf->lo, FALSE,
                                      &base_sat_down, &doppler_down,
                                      &down_ok);
    rigfrequ = rigctrl_compute_target(ctrl, FALSE, ctrl->conf->loup, FALSE,
                                      &base_sat_up, &doppler_up,
                                      &up_ok);
    ctrl->rig_target_down_hz = rigfreqd;
    ctrl->rig_target_up_hz = rigfrequ;

    gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqDown), rigfreqd);
    gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp), rigfrequ);

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
        else
        {
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rig update: mode=FULL_DUPLEX_MAIN_SUB side=RX base=%.0f doppler=%.0f sent=%.0f vfo=%s",
                        base_sat_down - ctrl->conf->lo,
                        doppler_down,
                        rigfreqd,
                        vfo_name(plan.downlink_vfo));

            set_ok = set_freq_simplex_vfo(ctrl, ctrl->sock,
                                          rigfreqd, plan.downlink_vfo);
            if (set_ok)
            {
                g_usleep(WR_DEL);
                read_ok = get_freq_simplex_vfo_strict(ctrl, ctrl->sock,
                                                      &readback,
                                                      plan.downlink_vfo);
                if (!read_ok)
                {
                    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                                "rig update: mode=FULL_DUPLEX_MAIN_SUB side=RX readback failed; keeping last");
                }
                else if (fabs(readback - rigfreqd) <= 100.0)
                {
                    ctrl->errcnt = 0;
                    ctrl->lastrxf = readback;
                    ctrl->rig_actual_down_hz = readback;
                    gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqDown),
                                            readback);
                }
                else
                {
                    ctrl->errcnt++;
                    ctrl->lastrxf = 0.0;
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
        else
        {
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rig update: mode=FULL_DUPLEX_MAIN_SUB side=TX base=%.0f doppler=%.0f sent=%.0f vfo=%s",
                        base_sat_up - ctrl->conf->loup,
                        doppler_up,
                        rigfrequ,
                        vfo_name(plan.uplink_vfo));

            set_ok = set_freq_simplex_vfo(ctrl, ctrl->sock,
                                          rigfrequ, plan.uplink_vfo);
            if (set_ok)
            {
                g_usleep(WR_DEL);
                read_ok = get_freq_simplex_vfo_strict(ctrl, ctrl->sock,
                                                      &readback,
                                                      plan.uplink_vfo);
                if (!read_ok)
                {
                    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                                "rig update: mode=FULL_DUPLEX_MAIN_SUB side=TX readback failed; keeping last");
                }
                else if (fabs(readback - rigfrequ) <= 100.0)
                {
                    ctrl->errcnt = 0;
                    ctrl->lasttxf = readback;
                    ctrl->rig_actual_up_hz = readback;
                    gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp),
                                            readback);
                }
                else
                {
                    ctrl->errcnt++;
                    ctrl->lasttxf = 0.0;
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
    gdouble         readfreq = 0.0;
    gdouble         tmpfreq = 0.0;
    gdouble         base_sat = 0.0;
    gboolean        dialchanged = FALSE;
    gboolean        use_rit_xit =
        (ctrl->conf != NULL) ? ctrl->conf->supports_rit_xit : FALSE;
    vfo_t           sat_vfo = VFO_SUB;
    gboolean        use_sat_vfo =
        satmode_vfo_for_role(ctrl->conf, VFO_ROLE_UPLINK, &sat_vfo);
    gdouble         base_freq = 0.0;
    gdouble         doppler_hz = 0.0;
    gdouble         sent_freq = 0.0;
    gdouble         target_down = 0.0;
    gboolean        target_ok = TRUE;

    if (is_full_duplex_main_sub_active(ctrl))
        use_rit_xit = FALSE;

    /* Dial feedback:
       If radio device is engaged read frequency from radio and compare it to the
       last set frequency. If different, it means that user has changed frequency
       on the radio dial => update transponder knob

       Note: If ctrl->lasttxf = 0.0 the sync has been invalidated (e.g. user pressed "tune")
       and no need to execute the dial feedback.
     */
    if ((ctrl->engaged) && (ctrl->lasttxf > 0.0))
    {
        if (!select_satmode_vfo(ctrl, ctrl->sock, VFO_ROLE_UPLINK,
                                "reading uplink freq", 0.0, FALSE))
            ctrl->errcnt++;
        if (use_sat_vfo)
        {
            if (!get_freq_toggle_vfo(ctrl, ctrl->sock, &readfreq, sat_vfo))
                readfreq = ctrl->lasttxf;
        }
        else if (!get_freq_toggle(ctrl, ctrl->sock, &readfreq))
        {
            /* error => use a passive value */
            readfreq = ctrl->lasttxf;
        }

        if (fabs(readfreq - ctrl->lasttxf) >= 1.0)
        {
            dialchanged = TRUE;

            /* user might have altered radio frequency => update rig readback */
            gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp), readfreq);
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
    ctrl->rig_target_up_hz = tmpfreq;
    ctrl->rig_target_down_hz = target_down;

    gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp), tmpfreq);
    gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqDown), target_down);

    base_freq = base_sat - ctrl->conf->loup;
    sent_freq = tmpfreq;

    /* if device is engaged, send freq command to radio */
    if ((ctrl->engaged) && target_ok &&
        (fabs(ctrl->lasttxf - tmpfreq) >= 1.0))
    {
        {
            gboolean set_ok;
            gboolean read_ok;
            gdouble  readback = 0.0;
            const gchar *vfo_label =
                use_sat_vfo ? vfo_name(sat_vfo) :
                (ctrl->conf->vfo_opt ? "currVFO" : "default");

            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "rig update: side=TX base=%.0f doppler=%.0f sent=%.0f vfo=%s",
                        base_freq, doppler_hz, sent_freq, vfo_label);

            set_ok = select_satmode_vfo(ctrl, ctrl->sock, VFO_ROLE_UPLINK,
                                        "setting uplink freq", tmpfreq, TRUE) &&
                (use_sat_vfo ?
                 set_freq_toggle_vfo(ctrl, ctrl->sock, tmpfreq, sat_vfo) :
                 set_freq_toggle(ctrl, ctrl->sock, tmpfreq));
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
                read_ok = use_sat_vfo ?
                    get_freq_toggle_vfo_strict(ctrl, ctrl->sock, &readback, sat_vfo) :
                    get_freq_toggle_strict(ctrl, ctrl->sock, &readback);
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            "rig update: side=TX readback=%.0f ok=%d",
                            readback, read_ok ? 1 : 0);

                if (!read_ok)
                {
                    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                                "rig update: side=TX readback failed; keeping last");
                }
                else if (fabs(readback - sent_freq) <= 100.0)
                {
                    ctrl->errcnt = 0;
                    ctrl->lasttxf = readback;
                    ctrl->rig_actual_up_hz = readback;
                    gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp),
                                            readback);
                }
                else
                {
                    sat_log_log(SAT_LOG_LEVEL_ERROR,
                                "rig update: side=TX readback mismatch (sent=%.0f read=%.0f); tracking unsynced",
                                sent_freq, readback);
                    ctrl->errcnt++;
                    ctrl->lasttxf = 0.0;
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
    gdouble         tmpfreq = 0.0;
    gdouble         readfreq = 0.0;
    gboolean        down_dialchanged = FALSE;
    gboolean        up_dialchanged = FALSE;
    gboolean        rx_use_rit_xit =
        (ctrl->conf != NULL) ? ctrl->conf->supports_rit_xit : FALSE;
    gboolean        tx_use_rit_xit =
        (ctrl->conf2 != NULL) ? ctrl->conf2->supports_rit_xit : FALSE;
    gboolean        down_ok = TRUE;
    gboolean        up_ok = TRUE;

    if (ctrl->engaged && (ctrl->lastrxf > 0.0))
    {
        if (!get_freq_simplex(ctrl, ctrl->sock, &readfreq))
            readfreq = ctrl->lastrxf;

        if (fabs(readfreq - ctrl->lastrxf) >= 1.0)
        {
            down_dialchanged = TRUE;
            gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqDown),
                                    readfreq);
            ctrl->lastrxf = readfreq;
            ctrl->rig_actual_down_hz = readfreq;
        }
    }

    if (down_dialchanged)
    {
        if (ctrl->conf2 == NULL)
            return;

        tmpfreq = rigctrl_compute_target(ctrl, FALSE, ctrl->conf2->loup,
                                         tx_use_rit_xit, NULL, NULL, &up_ok);
        ctrl->rig_target_up_hz = tmpfreq;
        gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp), tmpfreq);

        if ((ctrl->engaged) && up_ok &&
            (fabs(ctrl->lasttxf - tmpfreq) >= 1.0))
        {
            if (set_freq_simplex(ctrl, ctrl->sock2, tmpfreq))
            {
                ctrl->errcnt = 0;
                g_usleep(WR_DEL);
                if (get_freq_simplex_strict(ctrl, ctrl->sock2, &tmpfreq))
                {
                    ctrl->lasttxf = tmpfreq;
                    ctrl->rig_actual_up_hz = tmpfreq;
                    gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp),
                                            tmpfreq);
                }
                else
                {
                    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                                "rig update: side=TX readback failed; keeping last");
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
                                     rx_use_rit_xit, NULL, NULL, &down_ok);
    ctrl->rig_target_down_hz = tmpfreq;
    gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqDown), tmpfreq);

    if ((ctrl->engaged) && down_ok &&
        (fabs(ctrl->lastrxf - tmpfreq) >= 1.0))
    {
        if (set_freq_simplex(ctrl, ctrl->sock, tmpfreq))
        {
            ctrl->errcnt = 0;
            g_usleep(WR_DEL);
            if (get_freq_simplex_strict(ctrl, ctrl->sock, &tmpfreq))
            {
                ctrl->lastrxf = tmpfreq;
                ctrl->rig_actual_down_hz = tmpfreq;
                gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqDown),
                                        tmpfreq);
            }
            else
            {
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            "rig update: side=RX readback failed; keeping last");
            }
        }
        else
        {
            ctrl->errcnt++;
        }
    }

    if (ctrl->conf2 == NULL)
        return;

    if (ctrl->engaged && (ctrl->lasttxf > 0.0))
    {
        if (!get_freq_simplex(ctrl, ctrl->sock2, &readfreq))
            readfreq = ctrl->lasttxf;

        if (fabs(readfreq - ctrl->lasttxf) >= 1.0)
        {
            up_dialchanged = TRUE;
            gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp), readfreq);
            ctrl->lasttxf = readfreq;
            ctrl->rig_actual_up_hz = readfreq;
        }
    }

    if (up_dialchanged)
        return;

    tmpfreq = rigctrl_compute_target(ctrl, FALSE, ctrl->conf2->loup,
                                     tx_use_rit_xit, NULL, NULL, &up_ok);
    ctrl->rig_target_up_hz = tmpfreq;
    gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp), tmpfreq);

    if ((ctrl->engaged) && up_ok &&
        (fabs(ctrl->lasttxf - tmpfreq) >= 1.0))
    {
        if (set_freq_simplex(ctrl, ctrl->sock2, tmpfreq))
        {
            ctrl->errcnt = 0;
            g_usleep(WR_DEL);
            if (get_freq_simplex_strict(ctrl, ctrl->sock2, &tmpfreq))
            {
                ctrl->lasttxf = tmpfreq;
                ctrl->rig_actual_up_hz = tmpfreq;
                gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp),
                                        tmpfreq);
            }
            else
            {
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            "rig update: side=TX readback failed; keeping last");
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

static gboolean set_rit(GtkRigCtrl * ctrl, gint sock, gdouble hz)
{
    gchar          *buff;
    gchar           buffback[128];
    gboolean        retcode;
    radio_conf_t   *conf = get_conf_for_socket(ctrl, sock);
    gint            offset = (gint) llround(hz);

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

    return retcode;
}

static gboolean set_xit(GtkRigCtrl * ctrl, gint sock, gdouble hz)
{
    gchar          *buff;
    gchar           buffback[128];
    gboolean        retcode;
    radio_conf_t   *conf = get_conf_for_socket(ctrl, sock);
    gint            offset = (gint) llround(hz);

    if (conf != NULL && conf->vfo_opt)
        buff = g_strdup_printf("Z currVFO %d\x0a", offset);
    else
        buff = g_strdup_printf("Z %d\x0a", offset);

    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    g_free(buff);

    retcode = check_set_response(buffback, retcode, __func__);
    if (retcode == FALSE)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: Failed to set XIT offset to %d Hz"),
                    __func__, offset);
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
                          ctrl->tracking ? ctrl->dd : 0.0,
                          ctrl->tracking ? ctrl->du : 0.0);
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

    if (ctrl->engaged && ctrl->tracking)
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
static gboolean set_freq_simplex(GtkRigCtrl * ctrl, gint sock, gdouble freq)
{
    gchar          *buff;
    gchar           buffback[128];
    gboolean        retcode;

    if (ctrl->conf->vfo_opt)
        buff = g_strdup_printf("F currVFO %10.0f\x0a", freq);
    else
        buff = g_strdup_printf("F %10.0f\x0a", freq);
    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    g_free(buff);

    return (check_set_response(buffback, retcode, __func__));
}


/*
 * Set frequency in toggle mode
 *
 * Returns TRUE if the operation was successful, FALSE otherwise
 */
static gboolean set_freq_toggle(GtkRigCtrl * ctrl, gint sock, gdouble freq)
{
    gchar          *buff;
    gchar           buffback[128];
    gboolean        retcode;

    /* send command */
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                _("%s: set_freq_toggle vfo_opt=%d freq=%.0f"),
                __func__, ctrl->conf->vfo_opt, freq);
    if (ctrl->conf->vfo_opt)
    {
        buff = g_strdup_printf("I VFOA %10.0f\x0a", freq);
    }
    else
        buff = g_strdup_printf("I %10.0f\x0a", freq);

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
                                          gdouble * freq, gboolean allow_cache)
{
    gchar          *buff, **vbuff;
    gchar           buffback[128];
    gboolean        retcode;
    gboolean        retval = TRUE;
    const gchar    *label = ctrl->conf->vfo_opt ? "currVFO" : "default";
    const gint      cache_key = -1;

    if (ctrl->conf->vfo_opt)
        buff = g_strdup_printf("f currVFO\x0a");
    else
        buff = g_strdup_printf("f\x0a");
    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    retcode = check_get_response(buffback, retcode, __func__);
    if (retcode)
    {
        vbuff = g_strsplit(buffback, "\n", 3);
        if (vbuff[0])
            *freq = g_ascii_strtod(vbuff[0], NULL);
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

    if (allow_cache && rigctrl_get_cached_freq(sock, "f", cache_key, freq))
    {
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "get %s failed; using cached %.0f",
                    label, *freq);
        return TRUE;
    }

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "get %s failed",
                label);
    return FALSE;
}

static gboolean get_freq_simplex(GtkRigCtrl * ctrl, gint sock, gdouble * freq)
{
    return get_freq_simplex_internal(ctrl, sock, freq, TRUE);
}

static gboolean get_freq_simplex_strict(GtkRigCtrl * ctrl, gint sock,
                                        gdouble * freq)
{
    return get_freq_simplex_internal(ctrl, sock, freq, FALSE);
}

/*
 * Get vfo option
 *
 * Returns TRUE if the vfo option enabled was successful, FALSE otherwise
 */
static gboolean get_vfo_opt(GtkRigCtrl * ctrl, gint sock)
{
    gchar          *buff;
    gchar           buffback[128];
    gboolean        retcode;
    gboolean        retval = TRUE;

    buff = g_strdup_printf("\\set_vfo_opt 1\x0a");
    send_rigctld_command(ctrl, sock, buff, buffback, 128);
    // we don't really care about the return from set_vto_opt
    // we'll check to see if it worked next
    buff = g_strdup_printf("\\chk_vfo\x0a");
    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    retcode = check_get_response(buffback, retcode, __func__);
    if (retcode)
    {
        if (buffback[0]=='1') return TRUE;
        else return FALSE;
    }
    else
    {
        retval = FALSE;
    }

    g_free(buff);
    return retval;
}

/*
 * Get frequency when the radio is working toggle
 *
 * Returns TRUE if the operation was successful, FALSE otherwise
 */
static gboolean get_freq_toggle_internal(GtkRigCtrl * ctrl, gint sock,
                                         gdouble * freq, gboolean allow_cache)
{
    gchar          *buff, **vbuff;
    gchar           buffback[128];
    gboolean        retcode;
    gboolean        retval = TRUE;
    const gchar    *label = ctrl->conf->vfo_opt ? "currVFO" : "default";
    const gint      cache_key = -2;

    if (freq == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%d: NULL storage."), __FILE__, __LINE__);
        return FALSE;
    }

    /* send command */
    if (ctrl->conf->vfo_opt)
        buff = g_strdup_printf("i currVFO\x0a");
    else
        buff = g_strdup_printf("i\x0a");
    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    retcode = check_get_response(buffback, retcode, __func__);
    if (retcode)
    {
        vbuff = g_strsplit(buffback, "\n", 3);
        if (vbuff[0])
            *freq = g_ascii_strtod(vbuff[0], NULL);
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

    if (allow_cache && rigctrl_get_cached_freq(sock, "i", cache_key, freq))
    {
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "get %s (toggle) failed; using cached %.0f",
                    label, *freq);
        return TRUE;
    }

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "get %s (toggle) failed",
                label);
    return FALSE;
}

static gboolean get_freq_toggle(GtkRigCtrl * ctrl, gint sock, gdouble * freq)
{
    return get_freq_toggle_internal(ctrl, sock, freq, TRUE);
}

static gboolean get_freq_toggle_strict(GtkRigCtrl * ctrl, gint sock,
                                       gdouble * freq)
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
                                         gint *so_err_out)
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

    sat_log_log(SAT_LOG_LEVEL_ERROR,
                _("%s: Failed to connect to %s:%d (errno=%d so_error=%d)"),
                __func__, target, port, err, so_err);
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

static gboolean close_rigctld_socket(gint * sock, gboolean send_quit)
{
    gint            written;

    if (sock == NULL || *sock == -1)
        return TRUE;

    written = 0;
    if (send_quit)
    {
        written = send(*sock, "q\x0a", 2, 0);
        if (written != 2)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s:%s: Sent 2 bytes but sent %d."),
                        __FILE__, __func__, written);
        }
    }
#ifndef WIN32
    shutdown(*sock, SHUT_RDWR);
    close(*sock);
#else
    shutdown(*sock, SD_BOTH);
    closesocket(*sock);
#endif

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
    const gchar *cmd = "\\dump_state\n";
    gint  size;
    gchar *line1 = NULL;
    gchar *line2 = NULL;
    gint model = 0;
    gboolean parsed = FALSE;

    if (reply_out)
        *reply_out = NULL;
    if (model_out)
        *model_out = 0;

    if (!rigctld_connect_addrinfo(host, port, &sock))
        return RIGCTLD_PROBE_NOT_READY;

    rigctld_apply_socket_timeouts_ms(sock, timeout_ms);

    size = (gint) strlen(cmd);
    if (send(sock, cmd, size, 0) != size)
    {
        rigctld_close_fd(sock);
        return RIGCTLD_PROBE_NOT_READY;
    }

    size = (gint) recv(sock, buffer, sizeof(buffer) - 1, 0);
    if (size <= 0)
    {
        rigctld_close_fd(sock);
        return RIGCTLD_PROBE_NOT_READY;
    }

    buffer[size] = '\0';
    if (reply_out)
        *reply_out = g_strdup(buffer);

    rigctld_extract_first_lines(buffer, &line1, &line2);
    parsed = parse_dump_state_model_id(buffer, &model);
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
        return RIGCTLD_PROBE_MISMATCH;
    }

    g_free(line1);
    g_free(line2);
    rigctld_close_fd(sock);

    return RIGCTLD_PROBE_OK;
}

static gboolean rigctld_parse_frequency_response(const gchar *text,
                                                 gdouble *freq_out)
{
    gchar *trimmed = NULL;
    gchar **parts = NULL;
    gchar *endptr = NULL;
    gdouble freq = 0.0;
    gboolean ok = FALSE;

    if (freq_out)
        *freq_out = 0.0;

    if (text == NULL || *text == '\0')
        return FALSE;

    trimmed = g_strdup(text);
    g_strstrip(trimmed);
    if (*trimmed == '\0')
        goto out;

    parts = g_strsplit_set(trimmed, " \r\n\t", -1);
    if (parts[0] == NULL || *parts[0] == '\0')
        goto out;

    if (g_str_has_prefix(parts[0], "RPRT"))
        goto out;

    freq = g_ascii_strtod(parts[0], &endptr);
    if (endptr == parts[0])
        goto out;

    if (freq_out)
        *freq_out = freq;
    ok = TRUE;

out:
    g_strfreev(parts);
    g_free(trimmed);
    return ok;
}

static gboolean rigctld_probe_frequency(const gchar *host, gint port,
                                        gint timeout_ms, gint retries,
                                        gdouble *freq_out,
                                        gchar **reply_out)
{
    gint attempt = 0;
    gchar *last_reply = NULL;
    gdouble freq = 0.0;

    if (reply_out)
        *reply_out = NULL;

    for (attempt = 0; attempt < retries; attempt++)
    {
        gint sock = -1;
        gchar buffer[256];
        const gchar *cmd = "f\n";
        gint size;

        if (!rigctld_connect_addrinfo(host, port, &sock))
        {
            g_usleep(RIGCTLD_HEALTH_RETRY_DELAY_MS * 1000);
            continue;
        }

        rigctld_apply_socket_timeouts_ms(sock, timeout_ms);

        size = (gint) strlen(cmd);
        if (send(sock, cmd, size, 0) != size)
        {
            rigctld_close_fd(sock);
            g_usleep(RIGCTLD_HEALTH_RETRY_DELAY_MS * 1000);
            continue;
        }

        size = (gint) recv(sock, buffer, sizeof(buffer) - 1, 0);
        if (size > 0)
        {
            buffer[size] = '\0';
            g_free(last_reply);
            last_reply = g_strdup(buffer);
            if (rigctld_parse_frequency_response(buffer, &freq))
            {
                rigctld_close_fd(sock);
                if (freq_out)
                    *freq_out = freq;
                if (reply_out)
                    *reply_out = g_strdup(buffer);
                g_free(last_reply);
                return TRUE;
            }
        }

        rigctld_close_fd(sock);
        g_usleep(RIGCTLD_HEALTH_RETRY_DELAY_MS * 1000);
    }

    if (reply_out)
        *reply_out = last_reply;
    else
        g_free(last_reply);

    return FALSE;
}

static rigctld_probe_result_t rigctld_wait_for_ready(const gchar *host,
                                                     gint port,
                                                     gint timeout_ms,
                                                     gint expected_model,
                                                     gint *model_out)
{
    gint waited_ms = 0;
    const gint interval_ms = 200;
    rigctld_probe_result_t result = RIGCTLD_PROBE_NOT_READY;

    while (waited_ms < timeout_ms)
    {
        result = rigctld_probe_simple(host, port,
                                      RIGCTLD_PROBE_SHORT_MS,
                                      expected_model,
                                      model_out,
                                      NULL);
        if (result != RIGCTLD_PROBE_NOT_READY)
            return result;

        g_usleep(interval_ms * 1000);
        waited_ms += interval_ms;
    }

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
    gchar  *allowlist_lc = NULL;
    guint   filtered = 0;
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
    {
        guint count = g_slist_length(candidates);
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: auto-detect candidates=%u filtered=%u allowlist=%s"),
                    __func__, count, filtered,
                    (conf->rigctld_autodetect_match &&
                     *conf->rigctld_autodetect_match) ?
                        conf->rigctld_autodetect_match : "(none)");
        rig_term_log(ctrl, "gpredict",
                     "auto-detect candidates=%u filtered=%u allowlist=%s",
                     count, filtered,
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
        ((gint64) RIGCTLD_AUTODETECT_TOTAL_MS * 1000);

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
                rigctld_probe_simple(host, temp_port,
                                     RIGCTLD_AUTODETECT_PROBE_MS,
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
                rig_term_log(ctrl, "gpredict:err",
                             "auto-detect probe timeout for %s on %s:%d",
                             candidate, host, temp_port);
            else if (probe == RIGCTLD_PROBE_MISMATCH)
                rig_term_log(ctrl, "gpredict:err",
                             "auto-detect model mismatch expected=%d got=%d for %s",
                             expected_model, detected_model, candidate);
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
        return TRUE;

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
    gint        *sock_ptr = NULL;

    if (ctrl == NULL)
        return;

    if (sock == ctrl->sock)
    {
        role = _("receiver");
        secondary = FALSE;
        sock_ptr = &ctrl->sock;
    }
    else if (sock == ctrl->sock2)
    {
        role = _("uplink");
        secondary = TRUE;
        sock_ptr = &ctrl->sock2;
    }

    if (sock_ptr == NULL)
        return;

    sat_log_log(SAT_LOG_LEVEL_ERROR,
                _("%s: %s socket error during %s; disconnecting"),
                __func__, role, context ? context : "command");
    rig_term_log(ctrl, "gpredict:err",
                 "%s socket error during %s",
                 role ? role : "rig",
                 context ? context : "command");

    close_rigctld_socket(sock_ptr, rigctld_spawned_by_us(ctrl, secondary));
    rigctrl_schedule_reconnect(ctrl, secondary, role);
}


static gboolean ensure_rigctld_running(GtkRigCtrl *ctrl,
                                       radio_conf_t *conf,
                                       RigctldMgr **mgr,
                                       gboolean secondary,
                                       const gchar *role,
                                       gchar **connect_host,
                                       gboolean *error_reported)
{
    gchar          *host = NULL;
    gchar          *errmsg = NULL;
    gchar          *detail = NULL;
    gboolean        reported = FALSE;
    gboolean        ok = FALSE;
    gboolean        own_host = FALSE;
    gint            restart_attempts = 0;
    gboolean        is_ic905 = FALSE;
    gboolean        ic905_fallback = FALSE;

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
        gboolean is_ic905 = rigctld_is_ic905(conf);

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
                    "conn=%d model=%d baud=%d device=%s",
                    conf->host ? conf->host : "(null)",
                    conf->port,
                    conf->rigctld_autostart ? 1 : 0,
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

    if (!conf->rigctld_autostart)
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

    {
        gint listen_err = 0;
        gboolean listening =
            rigctld_try_connect_once(host, conf->port, 200, &listen_err);

        if (listening)
        {
            gint detected_model = 0;
            gint expected_model = rigctld_expected_model(conf);
            rigctld_probe_result_t probe =
                rigctld_wait_for_ready(host, conf->port, 1000,
                                       expected_model, &detected_model);

            if (probe == RIGCTLD_PROBE_OK)
            {
                if (conf->rigctld_conn == RIGCTLD_CONN_SERIAL &&
                    rigctld_mgr_host_is_local(conf->host))
                {
                    gdouble freq = 0.0;
                    gchar *reply = NULL;
                    gchar *log_tail = NULL;
                    gboolean stale = FALSE;

                    if (!rigctld_probe_frequency(host, conf->port,
                                                 RIGCTLD_HEALTH_TIMEOUT_MS,
                                                 RIGCTLD_HEALTH_RETRIES,
                                                 &freq, &reply))
                    {
                        if (mgr && *mgr)
                            log_tail = rigctld_mgr_get_log_tail(*mgr);
                        stale = rigctld_reply_indicates_stale_device(reply) ||
                            rigctld_log_tail_indicates_stale_device(log_tail);

                        if (stale)
                        {
                            sat_log_log(SAT_LOG_LEVEL_WARN,
                                        _("%s: rigctld listening but stale device detected; restarting autodetect"),
                                        __func__);
                            rig_term_log(ctrl, "gpredict:err",
                                         "rigctld listening but stale device detected; restarting autodetect");
                            if (rigctld_try_autodetect_restart(ctrl, conf, mgr,
                                                               secondary,
                                                               role, &reported,
                                                               &restart_attempts,
                                                               "stale device"))
                            {
                                g_free(reply);
                                g_free(log_tail);
                                goto restart_autostart;
                            }
                            if (mgr == NULL || *mgr == NULL)
                            {
                                schedule_rig_backend_error(ctrl, conf, role);
                                g_free(reply);
                                g_free(log_tail);
                                reported = TRUE;
                                goto out;
                            }
                        }

                        detail = g_strdup_printf(
                            _("rigctld did not respond to health probes on %s:%d."),
                            host, conf->port);
                        schedule_rig_autostart_error(ctrl, conf, role, detail);
                        g_free(detail);
                        g_free(reply);
                        g_free(log_tail);
                        reported = TRUE;
                        goto out;
                    }

                    sat_log_log(SAT_LOG_LEVEL_INFO,
                                _("%s: rigctld health probe ok (freq=%.0f)"),
                                __func__, freq);
                    rig_term_log(ctrl, "gpredict",
                                 "rigctld health probe ok (freq=%.0f)", freq);
                    g_free(reply);
                    g_free(log_tail);
                }

                rig_term_log(ctrl, "gpredict",
                             "rigctld reachable at %s:%d", host, conf->port);
                ok = TRUE;
                goto out;
            }

            if (probe == RIGCTLD_PROBE_MISMATCH)
            {
                rig_term_log(ctrl, "gpredict:err",
                             "rigctld model mismatch expected=%d got=%d at %s:%d",
                             expected_model, detected_model,
                             host, conf->port);
                detail = g_strdup_printf(
                    _("rigctld model mismatch at %s:%d.\nExpected %d, got %d."),
                    host, conf->port, expected_model, detected_model);
                schedule_rig_autostart_error(ctrl, conf, role, detail);
                g_free(detail);
                reported = TRUE;
                goto out;
            }

            if (conf->rigctld_conn == RIGCTLD_CONN_SERIAL &&
                rigctld_mgr_host_is_local(conf->host))
            {
                gchar *log_tail = NULL;
                gboolean stale = FALSE;

                if (mgr && *mgr)
                    log_tail = rigctld_mgr_get_log_tail(*mgr);
                stale = rigctld_log_tail_indicates_stale_device(log_tail);

                if (stale)
                {
                    sat_log_log(SAT_LOG_LEVEL_WARN,
                                _("%s: rigctld probe failed; stale device detected"),
                                __func__);
                    rig_term_log(ctrl, "gpredict:err",
                                 "rigctld probe failed; stale device detected");
                    if (rigctld_try_autodetect_restart(ctrl, conf, mgr,
                                                       secondary,
                                                       role, &reported,
                                                       &restart_attempts,
                                                       "stale device"))
                    {
                        g_free(log_tail);
                        goto restart_autostart;
                    }
                }
                g_free(log_tail);
            }

            detail = g_strdup_printf(
                _("rigctld at %s:%d is reachable but did not respond to probes."),
                host, conf->port);
            schedule_rig_autostart_error(ctrl, conf, role, detail);
            rig_term_log(ctrl, "gpredict:err",
                         "rigctld reachable but not responding at %s:%d",
                         host, conf->port);
            g_free(detail);
            reported = TRUE;
            goto out;
        }

        if (listen_err != 0)
        {
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        _("%s: rigctld not listening on %s:%d (err=%d)"),
                        __func__, host, conf->port, listen_err);
        }
    }

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

    if (conf->rigctld_autostart &&
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

restart_autostart:
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

    {
        gchar *exit_detail = NULL;
        gint waited_ms = 0;

        if (!rigctld_wait_tcp_listen(ctrl, *mgr, host, conf->port,
                                     RIGCTLD_AUTOSTART_TIMEOUT_MS,
                                     RIGCTLD_AUTOSTART_POLL_MS,
                                     &waited_ms, &exit_detail))
        {
            gchar *stderr_text = NULL;

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

            if (is_ic905 && !ic905_fallback)
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
                goto restart_autostart;
            }

            if (mgr)
                stderr_text = rigctld_mgr_get_log_tail(*mgr);

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
            reported = TRUE;
            goto out;
        }

        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: rigctld listening on %s:%d after %d ms"),
                    __func__, host, conf->port, waited_ms);
        rig_term_log(ctrl, "gpredict",
                     "rigctld listening on %s:%d after %d ms",
                     host, conf->port, waited_ms);
        g_free(exit_detail);
    }

    if (conf->rigctld_conn == RIGCTLD_CONN_SERIAL &&
        rigctld_mgr_host_is_local(conf->host))
    {
        gdouble freq = 0.0;
        gchar *reply = NULL;
        gchar *log_tail = NULL;
        gboolean stale = FALSE;

        if (!rigctld_probe_frequency(host, conf->port,
                                     RIGCTLD_HEALTH_TIMEOUT_MS,
                                     RIGCTLD_HEALTH_RETRIES,
                                     &freq, &reply))
        {
            if (mgr && *mgr)
                log_tail = rigctld_mgr_get_log_tail(*mgr);
            stale = rigctld_reply_indicates_stale_device(reply) ||
                rigctld_log_tail_indicates_stale_device(log_tail);

            if (stale)
            {
                sat_log_log(SAT_LOG_LEVEL_WARN,
                            _("%s: rigctld listening but stale device detected; restarting autodetect"),
                            __func__);
                rig_term_log(ctrl, "gpredict:err",
                             "rigctld listening but stale device detected; restarting autodetect");
                if (rigctld_try_autodetect_restart(ctrl, conf, mgr,
                                                   secondary,
                                                   role, &reported,
                                                   &restart_attempts,
                                                   "stale device"))
                {
                    g_free(reply);
                    g_free(log_tail);
                    goto restart_autostart;
                }
            }

            if (is_ic905 && !ic905_fallback)
            {
                sat_log_log(SAT_LOG_LEVEL_WARN,
                            _("%s: IC-905 rigctld not responding; retrying without civaddr"),
                            __func__);
                rig_term_log(ctrl, "gpredict:err",
                             "IC-905 rigctld not responding; retrying without civaddr");
                ic905_fallback = TRUE;
                rigctld_terminate_spawned(ctrl, secondary, mgr);
                g_free(reply);
                g_free(log_tail);
                goto restart_autostart;
            }

            detail = g_strdup_printf(
                _("rigctld did not respond to health probes on %s:%d."),
                host, conf->port);
            schedule_rig_autostart_error(ctrl, conf, role, detail);
            g_free(detail);
            g_free(reply);
            g_free(log_tail);
            reported = TRUE;
            goto out;
        }

        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: rigctld health probe ok (freq=%.0f)"),
                    __func__, freq);
        rig_term_log(ctrl, "gpredict",
                     "rigctld health probe ok (freq=%.0f)", freq);
        g_free(reply);
        g_free(log_tail);
    }

    {
        gint detected_model = 0;
        gint expected_model = rigctld_expected_model(conf);
        rigctld_probe_result_t probe =
            rigctld_wait_for_ready(host, conf->port, 2000,
                                   expected_model, &detected_model);
        if (probe != RIGCTLD_PROBE_OK)
        {
            gchar *stderr_text = NULL;
            gchar *log_tail = NULL;
            gboolean stale = FALSE;

            if (probe == RIGCTLD_PROBE_MISMATCH)
            {
                sat_log_log(SAT_LOG_LEVEL_ERROR,
                            _("%s: auto-start failed; rigctld model mismatch on %s:%d"),
                            __func__, host, conf->port);
                rig_term_log(ctrl, "gpredict:err",
                             "rigctld model mismatch expected=%d got=%d at %s:%d",
                             expected_model, detected_model,
                             host, conf->port);
            }
            else
            {
                sat_log_log(SAT_LOG_LEVEL_ERROR,
                            _("%s: auto-start failed; rigctld not responding on %s:%d"),
                            __func__, host, conf->port);
                rig_term_log(ctrl, "gpredict:err",
                             "rigctld not responding on %s:%d",
                             host, conf->port);
            }

            if (mgr && *mgr)
                stderr_text = rigctld_mgr_get_log_tail(*mgr);

            if (conf->rigctld_conn == RIGCTLD_CONN_SERIAL &&
                rigctld_mgr_host_is_local(conf->host))
            {
                if (mgr && *mgr)
                    log_tail = rigctld_mgr_get_log_tail(*mgr);
                stale = rigctld_log_tail_indicates_stale_device(log_tail);
                if (stale)
                {
                    sat_log_log(SAT_LOG_LEVEL_WARN,
                                _("%s: rigctld probe failed; stale device detected"),
                                __func__);
                    rig_term_log(ctrl, "gpredict:err",
                                 "rigctld probe failed; stale device detected");
                    if (rigctld_try_autodetect_restart(ctrl, conf, mgr,
                                                       secondary,
                                                       role, &reported,
                                                       &restart_attempts,
                                                       "stale device"))
                    {
                        g_free(stderr_text);
                        g_free(log_tail);
                        goto restart_autostart;
                    }
                }
            }

            if (probe != RIGCTLD_PROBE_MISMATCH &&
                is_ic905 && !ic905_fallback)
            {
                sat_log_log(SAT_LOG_LEVEL_WARN,
                            _("%s: IC-905 rigctld not responding; retrying without civaddr"),
                            __func__);
                rig_term_log(ctrl, "gpredict:err",
                             "IC-905 rigctld not responding; retrying without civaddr");
                ic905_fallback = TRUE;
                rigctld_terminate_spawned(ctrl, secondary, mgr);
                g_free(stderr_text);
                g_free(log_tail);
                goto restart_autostart;
            }

            if (probe == RIGCTLD_PROBE_MISMATCH)
            {
                if (stderr_text && *stderr_text)
                    detail = g_strdup_printf(
                        _("rigctld model mismatch at %s:%d.\nExpected %d, got %d.\n%s"),
                        host, conf->port, expected_model, detected_model,
                        stderr_text);
                else
                    detail = g_strdup_printf(
                        _("rigctld model mismatch at %s:%d.\nExpected %d, got %d."),
                        host, conf->port, expected_model, detected_model);
            }
            else
            {
                if (stderr_text && *stderr_text)
                    detail = g_strdup_printf(
                        _("rigctld did not respond to probes on %s:%d.\n%s"),
                        host, conf->port, stderr_text);
                else
                    detail = g_strdup_printf(
                        _("rigctld did not respond to probes on %s:%d."),
                        host, conf->port);
            }

            schedule_rig_autostart_error(ctrl, conf, role, detail);
            rigctld_terminate_spawned(ctrl, secondary, mgr);
            g_free(stderr_text);
            g_free(log_tail);
            g_free(detail);
            reported = TRUE;
            goto out;
        }
    }

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
    gboolean     reported = FALSE;
    gint         err = 0;
    gint         so_err = 0;

    if (!ensure_rigctld_running(ctrl, conf, mgr, secondary, role, &host,
                                &reported))
    {
        if (error_reported)
            *error_reported = reported;
        g_free(host);
        return FALSE;
    }

    if (conf == NULL || host == NULL)
    {
        if (error_reported)
            *error_reported = reported;
        g_free(host);
        return FALSE;
    }

    rig_term_log(ctrl, "gpredict",
                 "connecting to %s:%d", host, conf->port);
    if (!open_rigctld_socket_host(host, conf->port, sock, &err, &so_err))
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: Failed to connect to %s:%d"),
                    __func__, host, conf->port);
        rig_term_log(ctrl, "gpredict:err",
                     "connect failed to %s:%d (errno=%d so_error=%d)",
                     host, conf->port, err, so_err);
        g_free(host);
        if (error_reported)
            *error_reported = reported;
        return FALSE;
    }

    rig_term_log(ctrl, "gpredict", "connected to %s:%d", host, conf->port);
    g_free(host);
    if (error_reported)
        *error_reported = reported;
    return TRUE;
}

static gboolean probe_rigctld(GtkRigCtrl *ctrl, gint sock,
                              const gchar *label,
                              gboolean *stale_device_out)
{
    gchar           buffback[1024];
    gchar          *line1 = NULL;
    gchar          *line2 = NULL;
    gint            model = 0;
    gint            expected_model = 0;
    gboolean        ok;
    const gchar    *role = (label != NULL) ? label : _("rig");
    const radio_conf_t *conf = NULL;
    RigctldMgr     *mgr = NULL;
    gboolean        stale = FALSE;

    if (stale_device_out)
        *stale_device_out = FALSE;

    if (ctrl != NULL)
    {
        if (sock == ctrl->sock)
        {
            conf = ctrl->conf;
            mgr = ctrl->rigctld_mgr;
        }
        else if (sock == ctrl->sock2)
        {
            conf = ctrl->conf2;
            mgr = ctrl->rigctld_mgr2;
        }
        else
            conf = ctrl->conf;
    }
    expected_model = rigctld_expected_model(conf);

    ok = send_rigctld_command(ctrl, sock, "\\dump_state\n", buffback,
                              (gint) sizeof(buffback));
    if (!ok)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: probe failed to write/read (%s)"),
                    __func__, role);
        if (mgr != NULL)
        {
            gchar *log_tail = rigctld_mgr_get_log_tail(mgr);
            if (rigctld_log_tail_indicates_stale_device(log_tail))
                stale = TRUE;
            g_free(log_tail);
        }
        if (stale_device_out)
            *stale_device_out = stale;
        return FALSE;
    }

    rigctld_extract_first_lines(buffback, &line1, &line2);
    ok = parse_dump_state_model_id(buffback, &model);
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "rigctld dump_state: line1=%s line2=%s model=%d",
                line1 ? line1 : "(none)",
                line2 ? line2 : "(none)",
                model);

    if (!ok)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: probe got invalid dump_state from %s"),
                    __func__, role);
        if (rigctld_reply_indicates_stale_device(buffback))
            stale = TRUE;
        if (mgr != NULL)
        {
            gchar *log_tail = rigctld_mgr_get_log_tail(mgr);
            if (rigctld_log_tail_indicates_stale_device(log_tail))
                stale = TRUE;
            g_free(log_tail);
        }
        if (stale_device_out)
            *stale_device_out = stale;
        g_free(line1);
        g_free(line2);
        return FALSE;
    }

    if (expected_model > 0 && model > 0 && model != expected_model)
    {
        rig_term_log(ctrl, "gpredict:err",
                     "rigctld model mismatch expected=%d got=%d line1=%s line2=%s",
                     expected_model, model,
                     line1 ? line1 : "(none)",
                     line2 ? line2 : "(none)");
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: rigctld model mismatch expected=%d got=%d line1=%s line2=%s (%s)"),
                    __func__, expected_model, model,
                    line1 ? line1 : "(none)",
                    line2 ? line2 : "(none)",
                    role);
        g_free(line1);
        g_free(line2);
        return FALSE;
    }

    g_free(line1);
    g_free(line2);
    sat_log_log(SAT_LOG_LEVEL_INFO,
                _("%s: probe succeeded for %s (model=%d)"),
                __func__, role, model);
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

    if (ctrl != NULL)
    {
        if (ctrl->DevSel != NULL)
            gtk_widget_set_sensitive(ctrl->DevSel, TRUE);
        if (ctrl->DevSel2 != NULL)
            gtk_widget_set_sensitive(ctrl->DevSel2, TRUE);
        if (ctrl->LockBut != NULL)
        {
            g_signal_handlers_block_by_func(ctrl->LockBut,
                                            rig_engaged_cb, ctrl);
            gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->LockBut), FALSE);
            g_signal_handlers_unblock_by_func(ctrl->LockBut,
                                              rig_engaged_cb, ctrl);
            rig_engaged_cb(GTK_TOGGLE_BUTTON(ctrl->LockBut), ctrl);
        }
    }

    return G_SOURCE_REMOVE;
}

static void schedule_rig_disengage(GtkRigCtrl *ctrl)
{
    g_idle_add(rig_disengage_idle, ctrl);
}

static void rigctrl_fail_engage(GtkRigCtrl *ctrl)
{
    if (ctrl == NULL || !ctrl->engage_pending)
        return;

    ctrl->engage_pending = FALSE;
    ctrl->engaged = FALSE;
    schedule_rig_disengage(ctrl);
}

static void rigctrl_close(GtkRigCtrl * data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);

    ctrl->lastrxptt = FALSE;
    ctrl->lasttxptt = TRUE;
    ctrl->lasttxf = 0.0;
    ctrl->lastrxf = 0.0;

    apply_rit_xit_offsets(ctrl, 0.0, 0.0);

    remove_timer(ctrl);

    if ((ctrl->conf->type == RIG_TYPE_TOGGLE_AUTO) ||
        (ctrl->conf->type == RIG_TYPE_TOGGLE_MAN))
    {
        unset_toggle(ctrl, ctrl->sock);
    }

    if (ctrl->conf2 != NULL)
    {
        close_rigctld_socket(&(ctrl->sock2),
                             rigctld_spawned_by_us(ctrl, TRUE));
    }
    close_rigctld_socket(&(ctrl->sock),
                         rigctld_spawned_by_us(ctrl, FALSE));

    rigctld_terminate_spawned(ctrl, TRUE, &ctrl->rigctld_mgr2);
    rigctld_terminate_spawned(ctrl, FALSE, &ctrl->rigctld_mgr);

    rigctrl_reset_reconnect(ctrl, FALSE);
    rigctrl_reset_reconnect(ctrl, TRUE);
}

static gboolean rigctrl_open(GtkRigCtrl * data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);
    gboolean        rx_opened = FALSE;
    gboolean        tx_opened = FALSE;
    gboolean        tx_ok = TRUE;

    if (!ctrl->timerid)
        start_timer(ctrl);

    if (ctrl->sock < 0)
    {
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

            if (!open_rigctld_socket_with_autostart(ctrl, ctrl->conf,
                                                    &(ctrl->sock),
                                                    FALSE, _("receiver"),
                                                    &error_reported) ||
                !probe_rigctld(ctrl, ctrl->sock, _("receiver rig"),
                               &stale_device))
            {
                sat_log_log(SAT_LOG_LEVEL_ERROR,
                            _("%s: receiver rig open/probe failed"), __func__);
                close_rigctld_socket(&(ctrl->sock),
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
                rigctrl_fail_engage(ctrl);
                return FALSE;
            }
        }

        rigctrl_reset_reconnect(ctrl, FALSE);
        rx_opened = TRUE;

        /* check to see if vfo option is enabled */
        ctrl->conf->vfo_opt = get_vfo_opt(ctrl, ctrl->sock);
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                _("%s:%s: VFO opt=%d"), __FILE__,
                __func__, ctrl->conf->vfo_opt);
    }

    /* set initial frequency */
    if (ctrl->conf2 != NULL && ctrl->sock2 < 0)
    {
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

            tx_ok = open_rigctld_socket_with_autostart(
                ctrl, ctrl->conf2, &(ctrl->sock2), TRUE, _("uplink"),
                &error_reported);
            if (tx_ok)
                tx_ok = probe_rigctld(ctrl, ctrl->sock2, _("uplink rig"),
                                      &stale_device);
            if (!tx_ok)
            {
                sat_log_log(SAT_LOG_LEVEL_ERROR,
                            _("%s: uplink rig open/probe failed"), __func__);
                close_rigctld_socket(&(ctrl->sock2),
                                     rigctld_spawned_by_us(ctrl, TRUE));
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
            ctrl->conf2->vfo_opt = get_vfo_opt(ctrl, ctrl->sock2);
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    _("%s:%s: VFO opt2=%d"), __FILE__,
                    __func__, ctrl->conf2->vfo_opt);
        }
    }

    if (ctrl->sock < 0)
    {
        rigctrl_fail_engage(ctrl);
        return FALSE;
    }

    ctrl->engage_pending = FALSE;

    if ((rx_opened || tx_opened))
    {
        if (ctrl->conf2 != NULL)
        {
            if (ctrl->sock >= 0 && ctrl->sock2 >= 0)
                exec_dual_rig_cycle(ctrl);
        }
        else
        {
            if (is_full_duplex_main_sub_configured(ctrl->conf))
            {
                exec_full_duplex_main_sub_cycle(ctrl);
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
    }

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
        while (g_main_context_iteration(NULL, FALSE));

        if (t_ctrl == NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s:%s: ERROR: NO VALID ctrl-struct"), __FILE__,
                        __func__);
            continue;
        }

        if (t_ctrl->engaged)
        {
            gint64 now_us = g_get_monotonic_time();

            if (t_ctrl->sock < 0)
            {
                if (rigctrl_reconnect_due(t_ctrl, FALSE, now_us))
                {
                    rig_term_log(t_ctrl, "gpredict",
                                 "retry connect receiver");
                    if (!rigctrl_open(t_ctrl))
                    {
                        sat_log_log(SAT_LOG_LEVEL_ERROR,
                                    _("%s: failed to open receiver rig"),
                                    __func__);
                        if (t_ctrl->engaged)
                            rigctrl_schedule_reconnect(t_ctrl, FALSE,
                                                       _("receiver"));
                    }
                }
                if (t_ctrl->sock < 0)
                    continue;
            }

            if (t_ctrl->conf2 != NULL && t_ctrl->sock2 < 0)
            {
                gboolean attempted_tx = FALSE;

                if (rigctrl_reconnect_due(t_ctrl, TRUE, now_us))
                {
                    attempted_tx = TRUE;
                    rig_term_log(t_ctrl, "gpredict",
                                 "retry connect uplink");
                    if (!rigctrl_open(t_ctrl))
                    {
                        sat_log_log(SAT_LOG_LEVEL_ERROR,
                                    _("%s: failed to open uplink rig"),
                                    __func__);
                    }
                }
                if (attempted_tx && t_ctrl->sock2 < 0)
                {
                    if (t_ctrl->engaged)
                        rigctrl_schedule_reconnect(t_ctrl, TRUE, _("uplink"));
                }
            }

            if (!t_ctrl->timerid)
                start_timer(t_ctrl);
        }
        else
        {
            g_mutex_lock(&t_ctrl->widgetsync);

            if (t_ctrl->sock != -1)
                rigctrl_close(t_ctrl);

            if (t_ctrl->timerid)
                remove_timer(t_ctrl);

            g_cond_signal(&t_ctrl->widgetready);
            g_mutex_unlock(&t_ctrl->widgetsync);
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
                exec_full_duplex_main_sub_cycle(t_ctrl);
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

    if (t_ctrl->sock >= 0)
        rigctrl_close(t_ctrl);

    if (t_ctrl->timerid)
        remove_timer(t_ctrl);

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
    gtk_grid_set_row_spacing(GTK_GRID(table), 5);
    gtk_grid_set_column_spacing(GTK_GRID(table), 5);
    gtk_container_set_border_width(GTK_CONTAINER(table), 10);
    gtk_grid_attach(GTK_GRID(table), create_downlink_widgets(rigctrl),
                    0, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(table), create_uplink_widgets(rigctrl),
                    1, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(table), create_target_widgets(rigctrl),
                    0, 1, 1, 1);
    gtk_grid_attach(GTK_GRID(table), create_conf_widgets(rigctrl), 1, 1, 1, 1);
    gtk_grid_attach(GTK_GRID(table), gp_term_view_get_widget(rigctrl->term_view),
                    0, 2, 2, 1);
    gtk_grid_attach(GTK_GRID(table), create_count_down_widgets(rigctrl),
                    0, 3, 2, 1);

    gtk_container_add(GTK_CONTAINER(rigctrl), table);

    if (module->target > 0)
        gtk_rig_ctrl_select_sat(rigctrl, module->target);

    return widget;
}
