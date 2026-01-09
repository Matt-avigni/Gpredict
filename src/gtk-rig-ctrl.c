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

#include "compat.h"
#include "gp-term-view.h"
#include "gpredict-utils.h"
#include "gtk-freq-knob.h"
#include "gtk-rig-ctrl.h"
#include "predict-tools.h"
#include "radio-conf.h"
#include "rigctld_mgr.h"
#include "sat-log.h"
#include "sat-cfg.h"
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

/* radio control functions */
static void     exec_rx_cycle(GtkRigCtrl * ctrl);
static void     exec_tx_cycle(GtkRigCtrl * ctrl);
static void     exec_trx_cycle(GtkRigCtrl * ctrl);
static void     exec_toggle_cycle(GtkRigCtrl * ctrl);
static void     exec_toggle_tx_cycle(GtkRigCtrl * ctrl);
static void     exec_duplex_cycle(GtkRigCtrl * ctrl);
static void     exec_duplex_tx_cycle(GtkRigCtrl * ctrl);
static void     exec_dual_rig_cycle(GtkRigCtrl * ctrl);
static gboolean check_aos_los(GtkRigCtrl * ctrl);
static gboolean set_freq_simplex(GtkRigCtrl * ctrl, gint sock, gdouble freq);
static gboolean get_freq_simplex(GtkRigCtrl * ctrl, gint sock, gdouble * freq);
static gboolean set_freq_toggle(GtkRigCtrl * ctrl, gint sock, gdouble freq);
static gboolean set_toggle(GtkRigCtrl * ctrl, gint sock);
static gboolean unset_toggle(GtkRigCtrl * ctrl, gint sock);
static gboolean get_freq_toggle(GtkRigCtrl * ctrl, gint sock, gdouble * freq);
static gboolean get_ptt(GtkRigCtrl * ctrl, gint sock);
static gboolean set_ptt(GtkRigCtrl * ctrl, gint sock, gboolean ptt);
static gboolean set_rit(GtkRigCtrl * ctrl, gint sock, gdouble hz);
static gboolean set_xit(GtkRigCtrl * ctrl, gint sock, gdouble hz);
static void     apply_rit_xit_offsets(GtkRigCtrl * ctrl, gdouble rit,
                                      gdouble xit);
static void     update_rit_xit_offsets(GtkRigCtrl * ctrl);
static gboolean probe_rigctld(GtkRigCtrl *ctrl, gint sock,
                              const gchar *label);
static gboolean open_rigctld_socket_with_autostart(GtkRigCtrl *ctrl,
                                                   radio_conf_t *conf,
                                                   gint *sock,
                                                   gboolean secondary,
                                                   const gchar *role,
                                                   gboolean *error_reported);
static gboolean ensure_rigctld_running(GtkRigCtrl *ctrl,
                                       radio_conf_t *conf,
                                       RigctldMgr **mgr,
                                       const gchar *role,
                                       gchar **connect_host,
                                       gboolean *error_reported);
static gboolean open_rigctld_socket_host(const gchar *host, gint port,
                                         gint *sock);
static void     schedule_rig_conn_error(GtkRigCtrl *ctrl, radio_conf_t *conf,
                                        const gchar *role);
static void     schedule_rig_autostart_error(GtkRigCtrl *ctrl,
                                             const gchar *role,
                                             const gchar *detail);
static void     schedule_rig_disengage(GtkRigCtrl *ctrl);
static void     rig_logs_toggle_cb(GtkToggleButton *button, gpointer data);
static void     rig_term_log(GtkRigCtrl *ctrl, const gchar *prefix,
                             const gchar *fmt, ...) G_GNUC_PRINTF(3, 4);
static void     rig_term_log_tx(GtkRigCtrl *ctrl, const gchar *cmd);
static void     rig_term_log_rx(GtkRigCtrl *ctrl, const gchar *reply);
static void     rig_term_log_err_rprt(GtkRigCtrl *ctrl, const gchar *cmd,
                                      gint code);
static gboolean rig_parse_rprt_code(const gchar *reply, gint *code_out);
static const gchar *rig_rprt_error_string(gint code);
static void     rigctld_log_cb(RigctldMgr *mgr, const gchar *prefix,
                               const gchar *line, gpointer user_data);
static gboolean is_ic9700_satmode_configured(const radio_conf_t *conf);
static gboolean is_ic9700_satmode_active(const GtkRigCtrl *ctrl);
static const gchar *vfo_name(vfo_t vfo);
static void     rigctrl_reset_reconnect(GtkRigCtrl *ctrl, gboolean secondary);
static gboolean rigctrl_reconnect_due(GtkRigCtrl *ctrl, gboolean secondary,
                                      gint64 now_us);
static void     rigctrl_schedule_reconnect(GtkRigCtrl *ctrl, gboolean secondary,
                                           const gchar *role);
static void     rigctrl_handle_socket_error(GtkRigCtrl *ctrl, gint sock,
                                            const gchar *context);

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
                               GTK_DIALOG_MODAL |
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

    gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_destroy(dialog);
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
    g_free(conf);
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

static void rig_term_log_err_rprt(GtkRigCtrl *ctrl, const gchar *cmd,
                                  gint code)
{
    gchar *trim;

    if (ctrl == NULL || cmd == NULL)
        return;

    trim = g_strdup(cmd);
    g_strchomp(trim);
    g_strstrip(trim);
    if (*trim == '\0')
    {
        g_free(trim);
        return;
    }

    rig_term_log(ctrl, "gpredict:err", "%s failed: %s (%d)",
                 trim, rig_rprt_error_string(code), code);
    g_free(trim);
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
}

static void rig_show_conn_error(GtkRigCtrl *ctrl,
                                radio_conf_t *conf,
                                const gchar *role)
{
    const gchar    *host = (conf && conf->host) ? conf->host : "(null)";
    gint            port = conf ? conf->port : 0;
    const gchar    *label = (role != NULL) ? role : _("rig");
    gchar          *body;

    if (ctrl == NULL)
        return;

    body = g_strdup_printf(_("Unable to connect to rigctld (%s)\nHost: %s\nPort: %d"),
                           label, host, port);
    rig_show_error_dialog(ctrl, _("Unable to connect to rigctld"), body);
    g_free(body);
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

static gboolean rigctrl_reconnect_due(GtkRigCtrl *ctrl, gboolean secondary,
                                      gint64 now_us)
{
    gint64 next_us;

    if (ctrl == NULL)
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
        backoff = 250;
    else
        backoff = MIN(*backoff_ptr * 2, 5000);

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

    rigctld_mgr_terminate(&ctrl->rigctld_mgr2);
    rigctld_mgr_terminate(&ctrl->rigctld_mgr);

    if (ctrl->term_view != NULL)
    {
        gp_term_view_free(ctrl->term_view);
        ctrl->term_view = NULL;
    }
    ctrl->log_toggle = NULL;

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
    ctrl->rigctld_mgr = NULL;
    ctrl->rigctld_mgr2 = NULL;
    ctrl->term_view = gp_term_view_new(_("Follow tail"), TRUE, FALSE);
    ctrl->log_toggle = NULL;
    g_mutex_init(&(ctrl->busy));
    ctrl->engaged = FALSE;
    ctrl->delay = 1000;
    ctrl->timerid = 0;
    ctrl->errcnt = 0;
    ctrl->lastrxptt = FALSE;
    ctrl->lasttxptt = TRUE;
    ctrl->lastrxf = 0.0;
    ctrl->lasttxf = 0.0;
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
        satfreq = gtk_freq_knob_get_value(GTK_FREQ_KNOB(ctrl->SatFreqDown));
        ctrl->dd = -satfreq * (ctrl->target->range_rate / 299792.4580); // Hz
        buff = g_strdup_printf("%.0f Hz", ctrl->dd);
        gtk_label_set_text(GTK_LABEL(ctrl->SatDopDown), buff);
        g_free(buff);

        /* Doppler shift up */
        satfreq = gtk_freq_knob_get_value(GTK_FREQ_KNOB(ctrl->SatFreqUp));
        ctrl->du = satfreq * (ctrl->target->range_rate / 299792.4580);  // Hz
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

        gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->SatFreqUp), up);
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

        gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->SatFreqDown), down);
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

    if (ctrl->trsplock)
        track_downlink(ctrl);
}

static void uplink_changed_cb(GtkFreqKnob * knob, gpointer data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);

    (void)knob;

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
static void trsp_tune_cb(GtkButton * button, gpointer data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);
    gdouble         freq;

    (void)button;

    if (ctrl->trsp == NULL)
        return;

    /* tune downlink */
    if ((ctrl->trsp->downlow > 0) && (ctrl->trsp->downhigh > 0))
    {
        freq = ctrl->trsp->downlow +
            labs((long)ctrl->trsp->downhigh - (long)ctrl->trsp->downlow) / 2;
        gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->SatFreqDown), freq);

        /* invalidate RIG<->GPREDICT sync */
        ctrl->lastrxf = 0.0;
    }

    /* tune uplink */
    if ((ctrl->trsp->uplow > 0) && (ctrl->trsp->uphigh > 0))
    {
        freq = ctrl->trsp->uplow +
            labs((long)ctrl->trsp->uphigh - (long)ctrl->trsp->uplow) / 2;
        gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->SatFreqUp), freq);

        /* invalidate RIG<->GPREDICT sync */
        ctrl->lasttxf = 0.0;
    }
}

/*
 * Called when a new transponder is selected.
 * It updates ctrl->trsp with the new selection and issues a "tune" event.
 */
static void trsp_selected_cb(GtkComboBox * box, gpointer data)
{
    GtkRigCtrl     *ctrl = GTK_RIG_CTRL(data);
    gint            i, n;

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
        trsp_tune_cb(NULL, data);
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
    if (is_ic9700_satmode_configured(ctrl->conf))
    {
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "SATMODE IC-9700: downlink -> %s, uplink -> %s",
                    vfo_name(VFO_MAIN), vfo_name(VFO_SUB));
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

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                _("%s:%s: Primary device selected: %d"),
                __FILE__, __func__, gtk_combo_box_get_active(box));

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
        gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(box));
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
    gchar          *name1, *name2;


    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                _("%s:%s: Secondary device selected: %d"),
                __FILE__, __func__, gtk_combo_box_get_active(box));

    if (ctrl->conf2 != NULL)
    {
        free_radio_conf(ctrl->conf2);
        ctrl->conf2 = NULL;
    }

    if (gtk_combo_box_get_active(box) == 0)
    {
        /* first entry is "None" */
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
    name1 =
        gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(ctrl->DevSel));
    name2 =
        gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(ctrl->DevSel2));
    if (!g_strcmp0(name1, name2))
    {
        if (is_ic9700_satmode_configured(ctrl->conf))
        {
            /* Allow same rig: IC-9700 uses dual VFOs in SAT mode. */
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "SATMODE IC-9700: using primary rig for uplink (dual VFO)");
            g_free(name1);
            g_free(name2);
            if (ctrl->conf != NULL)
            {
                buff = g_strdup_printf(_("%.0f MHz"), ctrl->conf->loup / 1.0e6);
                gtk_label_set_text(GTK_LABEL(ctrl->LoUp), buff);
                g_free(buff);
            }

            return;
        }

        /* selected conf is the same as the primary one */
        g_free(name1);
        g_free(name2);
        if (ctrl->conf != NULL)
        {
            buff = g_strdup_printf(_("%.0f MHz"), ctrl->conf->loup / 1.0e6);
            gtk_label_set_text(GTK_LABEL(ctrl->LoUp), buff);
            g_free(buff);
        }
        gtk_combo_box_set_active(GTK_COMBO_BOX(ctrl->DevSel2), 0);

        return;
    }

    g_free(name1);
    g_free(name2);

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
        gtk_combo_box_text_get_active_text(GTK_COMBO_BOX_TEXT(box));
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

    if (ctrl->conf == NULL)
    {
        /* We don't have a working configuration – inform the user and
         * immediately revert the toggle.
         */
        rig_show_error_dialog(
            ctrl,
            _("Unable to engage radio"),
            _("No valid radio configuration is selected.\n"
              "Please create or select a radio configuration in\n"
              "Interfaces → Radios before engaging radio control."));
        gtk_toggle_button_set_active(button, FALSE);
        return;
    }

    if (!gtk_toggle_button_get_active(button))
    {
        /* Disengage: close socket / stop worker thread */
        gtk_widget_set_sensitive(ctrl->DevSel, TRUE);
        gtk_widget_set_sensitive(ctrl->DevSel2, TRUE);
        ctrl->engaged = FALSE;
        rig_term_log(ctrl, "gpredict", "disengage");

        /* Notify worker thread about the new configuration/state */
        setconfig(ctrl);
        /* The worker thread will clean up and exit; we just clear the
         * handle so a new thread can be started on the next engage.
         */
        ctrl->rigctl_thread = NULL;
    }
    else
    {
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

    /* Transponder selector, tune, and trsplock buttons */
    ctrl->TrspSel = gtk_combo_box_text_new();
    gtk_widget_set_tooltip_text(ctrl->TrspSel, _("Select a transponder"));
    load_trsp_list(ctrl);
    g_signal_connect(ctrl->TrspSel, "changed", G_CALLBACK(trsp_selected_cb),
                     ctrl);
    gtk_grid_attach(GTK_GRID(table), ctrl->TrspSel, 0, 1, 3, 1);

    /* buttons */
    tune = gtk_button_new_with_label(_("T"));
    gtk_widget_set_tooltip_text(tune,
                                _("Tune the radio to this transponder. "
                                  "The uplink and downlink will be set to the "
                                  "center of the transponder passband. In case "
                                  "of beacons, only the downlink will be tuned "
                                  "to the beacon frequency."));
    g_signal_connect(tune, "clicked", G_CALLBACK(trsp_tune_cb), ctrl);

    trsplock = gtk_toggle_button_new_with_label(_("L"));
    gtk_widget_set_tooltip_text(trsplock,
                                _("Lock the uplink and the downlink to each "
                                  "other. Whenever you change the downlink "
                                  "(in the controller or on the dial, the "
                                  "uplink will track it according to whether "
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
        cantx = (conf->type == RIG_TYPE_RX) ? conf->supports_full_duplex : TRUE;
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

static GtkWidget *create_conf_widgets(GtkRigCtrl * ctrl)
{
    GtkWidget      *frame, *table, *label;
    GDir           *dir = NULL; /* directory handle */
    GError         *error = NULL;       /* error flag and info */
    gchar          *dirname;    /* directory name */
    gchar         **vbuff;
    const gchar    *filename;   /* file name */
    gchar          *rigname;


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

    /* open configuration directory */
    dirname = get_hwconf_dir();

    dir = g_dir_open(dirname, 0, &error);
    if (dir)
    {
        /* read each .rig file */
        GSList         *rigs = NULL;
        gint            i;
        gint            n;

        while ((filename = g_dir_read_name(dir)))
        {
            if (g_str_has_suffix(filename, ".rig"))
            {
                vbuff = g_strsplit(filename, ".rig", 0);
                rigs = g_slist_insert_sorted(rigs, g_strdup(vbuff[0]),
                                             (GCompareFunc) rig_name_compare);
                g_strfreev(vbuff);
            }
        }
        n = g_slist_length(rigs);
        for (i = 0; i < n; i++)
        {
            rigname = g_slist_nth_data(rigs, i);
            if (rigname)
            {
                gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT
                                               (ctrl->DevSel), rigname);
                g_free(rigname);
            }
        }
        g_slist_free(rigs);
    }
    else
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%d: Failed to open hwconf dir (%s)"),
                    __FILE__, __LINE__, error->message);
        g_clear_error(&error);
    }

    g_dir_close(dir);

    gtk_combo_box_set_active(GTK_COMBO_BOX(ctrl->DevSel), 0);
    g_signal_connect(ctrl->DevSel, "changed",
                     G_CALLBACK(primary_rig_selected_cb), ctrl);
    gtk_grid_attach(GTK_GRID(table), ctrl->DevSel, 1, 0, 1, 1);

    /* Secondary device */
    label = gtk_label_new(_("2. Device (TX):"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 1, 1, 1);

    ctrl->DevSel2 = gtk_combo_box_text_new();
    gtk_widget_set_tooltip_text(ctrl->DevSel2,
                                _("Select secondary radio device\n"
                                  "This device will be used for uplink"));

    /* load config */
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ctrl->DevSel2),
                                   _("None"));
    gtk_combo_box_set_active(GTK_COMBO_BOX(ctrl->DevSel2), 0);

    dir = g_dir_open(dirname, 0, &error);
    if (dir)
    {
        /* read each .rig file */
        while ((filename = g_dir_read_name(dir)))
        {
            if (g_str_has_suffix(filename, ".rig"))
            {
                /* only add TX capable rigs */
                vbuff = g_strsplit(filename, ".rig", 0);
                if (is_rig_tx_capable(vbuff[0]))
                {
                    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT
                                                   (ctrl->DevSel2), vbuff[0]);
                }
                g_strfreev(vbuff);
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

    g_signal_connect(ctrl->DevSel2, "changed",
                     G_CALLBACK(secondary_rig_selected_cb), ctrl);
    gtk_grid_attach(GTK_GRID(table), ctrl->DevSel2, 1, 1, 1, 1);

    /* Logs button */
    ctrl->log_toggle = gtk_toggle_button_new_with_label(_("Logs"));
    gtk_widget_set_tooltip_text(ctrl->log_toggle,
                                _("Show or hide the radio control logs"));
    g_signal_connect(ctrl->log_toggle, "toggled",
                     G_CALLBACK(rig_logs_toggle_cb), ctrl);
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
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: rigctld port closed"), __func__);
        rig_term_log(ctrl, "gpredict:err",
                     "send failed (%s)", strerror(errno));
        rigctrl_handle_socket_error(ctrl, sock, "send");
        return FALSE;
    }
    /* try to read answer */
    size = recv(sock, buffout, sizeout - 1, 0);
    if (size == -1)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: rigctld port closed"), __func__);
        if (errno == EAGAIN || errno == EWOULDBLOCK)
            rig_term_log(ctrl, "gpredict:err",
                         "timeout waiting for rigctld reply");
        else
            rig_term_log(ctrl, "gpredict:err",
                         "recv failed (%s)", strerror(errno));
        rigctrl_handle_socket_error(ctrl, sock, "recv");
        return FALSE;
    }

    buffout[size] = '\0';
    if (size == 0)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%s: Got 0 bytes from rigctld"), __FILE__, __func__);
        rig_term_log(ctrl, "gpredict:err",
                     "rigctld closed connection");
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
        rig_term_log_err_rprt(ctrl, buff, rprt);

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

static gboolean is_ic9700_satmode_configured(const radio_conf_t *conf)
{
    return (conf != NULL) &&
        (conf->type == RIG_TYPE_DUPLEX) &&
        conf->supports_dual_vfo_sat;
}

static gboolean is_ic9700_satmode_active(const GtkRigCtrl *ctrl)
{
    return (ctrl != NULL) &&
        ctrl->tracking &&
        is_ic9700_satmode_configured(ctrl->conf);
}

static gboolean satmode_vfo_for_role(const radio_conf_t *conf,
                                     vfo_role_t role, vfo_t *vfo)
{
    if (!is_ic9700_satmode_configured(conf) || vfo == NULL)
        return FALSE;

    /* IC-9700 SAT mode expects Main/Sub and explicit VFO commands; avoid VFO switching. */
    *vfo = (role == VFO_ROLE_DOWNLINK) ? VFO_MAIN : VFO_SUB;
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

    sat_log_log(SAT_LOG_LEVEL_DEBUG, "SATMODE IC-9700: %s -> %s",
                vfo_role_name(role), vfo_name(vfo));
    if (action != NULL)
    {
        if (log_freq)
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "SATMODE IC-9700: using %s for %s %.0f",
                        vfo_name(vfo), action, freq);
        else
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        "SATMODE IC-9700: using %s for %s",
                        vfo_name(vfo), action);
    }

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
                    "SATMODE IC-9700: vfo_opt disabled; sending explicit VFO");
    }

    buff = g_strdup_printf("F %s %10.0f\x0a", vfo_name(vfo), freq);

    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "SATMODE IC-9700: set %s %.0f -> %s",
                vfo_name(vfo), freq, buffback);
    g_free(buff);

    return check_set_response(buffback, retcode, __func__);
}

static gboolean get_freq_simplex_vfo(GtkRigCtrl *ctrl, gint sock,
                                     gdouble *freq, vfo_t vfo)
{
    gchar          *buff;
    gchar           buffback[128];
    gboolean        retcode;
    gboolean        retval = TRUE;
    gchar         **vbuff;

    if (!ctrl->conf->vfo_opt)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    "SATMODE IC-9700: vfo_opt disabled; requesting explicit VFO");
    }

    buff = g_strdup_printf("f %s\x0a", vfo_name(vfo));

    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    retcode = check_get_response(buffback, retcode, __func__);
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "SATMODE IC-9700: get %s -> %s",
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
    return retval;
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
                    "SATMODE IC-9700: vfo_opt disabled; sending explicit VFO");
    }

    buff = g_strdup_printf("I %s %10.0f\x0a", vfo_name(vfo), freq);

    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "SATMODE IC-9700: set %s (toggle) %.0f -> %s",
                vfo_name(vfo), freq, buffback);
    g_free(buff);

    return check_set_response(buffback, retcode, __func__);
}

static gboolean get_freq_toggle_vfo(GtkRigCtrl *ctrl, gint sock,
                                    gdouble *freq, vfo_t vfo)
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
                    "SATMODE IC-9700: vfo_opt disabled; requesting explicit VFO");
    }

    buff = g_strdup_printf("i %s\x0a", vfo_name(vfo));

    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    retcode = check_get_response(buffback, retcode, __func__);
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "SATMODE IC-9700: get %s (toggle) -> %s",
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
    return retval;
}

static int get_vfos(GtkRigCtrl * ctrl, char *rx, char *tx)
{
    // fill rx/tx with vfo name plus space if not empty
    rx = tx = "";
    switch (ctrl->conf->vfoUp)
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
                    ctrl->conf->vfoUp);
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
    vfo_up = ctrl->conf->vfoUp;
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
                    ctrl->conf->vfoUp);
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
    gdouble         readfreq = 0.0, tmpfreq, satfreqd, satfrequ;
    gboolean        ptt = FALSE;
    gboolean        use_rit_xit =
        (ctrl->conf != NULL) ? ctrl->conf->supports_rit_xit : FALSE;
    vfo_t           sat_vfo = VFO_MAIN;
    gboolean        use_sat_vfo =
        satmode_vfo_for_role(ctrl->conf, VFO_ROLE_DOWNLINK, &sat_vfo);
    gdouble         base_freq = 0.0;
    gdouble         doppler_hz = 0.0;
    gdouble         sent_freq = 0.0;

    /* get PTT status */
    if (ctrl->engaged && ctrl->conf->ptt)
        ptt = get_ptt(ctrl, ctrl->sock);

    if (is_ic9700_satmode_active(ctrl))
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
                ctrl->errcnt++;
        }
        else if (!get_freq_simplex(ctrl, ctrl->sock, &readfreq))
        {
            /* error => use a passive value */
            ctrl->errcnt++;
        }
        else if (fabs(readfreq - ctrl->lastrxf) >= 1.0)
        {
            /* user might have altered radio frequency => update transponder knob */
            gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqDown),
                                    readfreq);
            ctrl->lastrxf = readfreq;

            /* doppler shift; only if we are tracking */
            if (ctrl->tracking)
            {
                if (use_rit_xit)
                    satfreqd = readfreq + ctrl->conf->lo;
                else
                    satfreqd = (readfreq - ctrl->dd + ctrl->conf->lo);
            }
            else
            {
                satfreqd = readfreq + ctrl->conf->lo;
            }
            gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->SatFreqDown),
                                    satfreqd);

            /* Update uplink if locked to downlink */
            if (ctrl->trsplock)
            {
                track_downlink(ctrl);
            }

            /* no need to forward track */
            return;
        }
    }

    /* now, forward tracking */

    /* If we are tracking, calculate the radio freq by applying both dopper shift
       and tranverter LO frequency. If we are not tracking, apply only LO frequency.
     */
    satfreqd = gtk_freq_knob_get_value(GTK_FREQ_KNOB(ctrl->SatFreqDown));
    satfrequ = gtk_freq_knob_get_value(GTK_FREQ_KNOB(ctrl->SatFreqUp));
    if (ctrl->tracking && !use_rit_xit)
    {
        /* downlink */
        gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqDown),
                                satfreqd + ctrl->dd - ctrl->conf->lo);
        /* uplink */
        gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp),
                                satfrequ + ctrl->du - ctrl->conf->loup);
    }
    else
    {
        gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqDown),
                                satfreqd - ctrl->conf->lo);
        gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp),
                                satfrequ - ctrl->conf->loup);
    }

    base_freq = satfreqd - ctrl->conf->lo;
    doppler_hz = (ctrl->tracking && !use_rit_xit) ? ctrl->dd : 0.0;
    sent_freq = base_freq + doppler_hz;
    tmpfreq = sent_freq;

    /* if device is engaged, send freq command to radio */
    if ((ctrl->engaged) && (ptt == FALSE) &&
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
                    get_freq_simplex_vfo(ctrl, ctrl->sock, &readback, sat_vfo) :
                    get_freq_simplex(ctrl, ctrl->sock, &readback);
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            "rig update: side=RX readback=%.0f ok=%d",
                            readback, read_ok ? 1 : 0);

                if (read_ok && fabs(readback - sent_freq) <= 100.0)
                {
                    ctrl->errcnt = 0;
                    ctrl->lastrxf = readback;

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
    gdouble         readfreq = 0.0, tmpfreq, satfreqd, satfrequ;
    gboolean        ptt = TRUE;
    gboolean        use_rit_xit =
        (ctrl->conf != NULL) ? ctrl->conf->supports_rit_xit : FALSE;
    gdouble         base_freq = 0.0;
    gdouble         doppler_hz = 0.0;
    gdouble         sent_freq = 0.0;

    /* get PTT status */
    if (ctrl->engaged && ctrl->conf->ptt)
    {
        ptt = get_ptt(ctrl, ctrl->sock);
    }

    if (is_ic9700_satmode_active(ctrl))
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
            ctrl->errcnt++;
        }
        else if (fabs(readfreq - ctrl->lasttxf) >= 1.0)
        {
            /* user might have altered radio frequency => update transponder knob */
            gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp), readfreq);
            ctrl->lasttxf = readfreq;

            /* doppler shift; only if we are tracking */
            if (ctrl->tracking)
            {
                if (use_rit_xit)
                    satfrequ = readfreq + ctrl->conf->loup;
                else
                    satfrequ = readfreq - ctrl->du + ctrl->conf->loup;
            }
            else
            {
                satfrequ = readfreq + ctrl->conf->loup;
            }
            gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->SatFreqUp), satfrequ);

            /* Follow with downlink if transponder is locked */
            if (ctrl->trsplock)
            {
                track_uplink(ctrl);
            }

            /* no need to forward track */
            return;
        }
    }

    /* now, forward tracking */

    /* If we are tracking, calculate the radio freq by applying both dopper shift
       and tranverter LO frequency. If we are not tracking, apply only LO frequency.
     */
    satfreqd = gtk_freq_knob_get_value(GTK_FREQ_KNOB(ctrl->SatFreqDown));
    satfrequ = gtk_freq_knob_get_value(GTK_FREQ_KNOB(ctrl->SatFreqUp));
    if (ctrl->tracking && !use_rit_xit)
    {
        /* downlink */
        gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqDown),
                                satfreqd + ctrl->dd - ctrl->conf->lo);
        /* uplink */
        gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp),
                                satfrequ + ctrl->du - ctrl->conf->loup);
    }
    else
    {
        gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqDown),
                                satfreqd - ctrl->conf->lo);
        gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp),
                                satfrequ - ctrl->conf->loup);
    }

    base_freq = satfrequ - ctrl->conf->loup;
    doppler_hz = (ctrl->tracking && !use_rit_xit) ? ctrl->du : 0.0;
    sent_freq = base_freq + doppler_hz;
    tmpfreq = sent_freq;

    /* if device is engaged, send freq command to radio */
    if ((ctrl->engaged) && (ptt == TRUE) &&
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
                read_ok = get_freq_simplex(ctrl, ctrl->sock, &readback);
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            "rig update: side=TX readback=%.0f ok=%d",
                            readback, read_ok ? 1 : 0);

                if (read_ok && fabs(readback - sent_freq) <= 100.0)
                {
                    ctrl->errcnt = 0;
                    ctrl->lasttxf = readback;

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
    gdouble         tmpfreq;
    gboolean        ptt = TRUE;

    if (ctrl->engaged && ctrl->conf->ptt)
    {
        ptt = get_ptt(ctrl, ctrl->sock);
    }

    /* if we are in TX mode do nothing */
    if (ptt == TRUE)
    {
        return;
    }

    /* Get the desired uplink frequency from controller */
    tmpfreq = gtk_freq_knob_get_value(GTK_FREQ_KNOB(ctrl->RigFreqUp));

    /* if device is engaged, send freq command to radio */
    if ((ctrl->engaged) && (fabs(ctrl->lasttxf - tmpfreq) >= 10.0))
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

static void exec_duplex_tx_cycle(GtkRigCtrl * ctrl)
{
    gdouble         readfreq = 0.0, tmpfreq, satfreqd, satfrequ;
    gboolean        dialchanged = FALSE;
    gboolean        use_rit_xit =
        (ctrl->conf != NULL) ? ctrl->conf->supports_rit_xit : FALSE;
    vfo_t           sat_vfo = VFO_SUB;
    gboolean        use_sat_vfo =
        satmode_vfo_for_role(ctrl->conf, VFO_ROLE_UPLINK, &sat_vfo);
    gdouble         base_freq = 0.0;
    gdouble         doppler_hz = 0.0;
    gdouble         sent_freq = 0.0;

    if (is_ic9700_satmode_active(ctrl))
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
                ctrl->errcnt++;
        }
        else if (!get_freq_toggle(ctrl, ctrl->sock, &readfreq))
        {
            /* error => use a passive value */
            readfreq = ctrl->lasttxf;
            ctrl->errcnt++;
        }

        if (fabs(readfreq - ctrl->lasttxf) >= 1.0)
        {
            dialchanged = TRUE;

            /* user might have altered radio frequency => update transponder knob */
            gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp), readfreq);
            ctrl->lasttxf = readfreq;

            /* doppler shift; only if we are tracking */
            if (ctrl->tracking)
            {
                if (use_rit_xit)
                    satfrequ = readfreq + ctrl->conf->loup;
                else
                    satfrequ = readfreq - ctrl->du + ctrl->conf->loup;
            }
            else
            {
                satfrequ = readfreq + ctrl->conf->loup;
            }
            gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->SatFreqUp), satfrequ);

            /* Follow with downlink if transponder is locked */
            if (ctrl->trsplock)
            {
                track_uplink(ctrl);
            }
        }
    }

    /* now, forward tracking */
    if (dialchanged)
    {
        /* no need to forward track */
        return;
    }

    /* If we are tracking, calculate the radio freq by applying both dopper shift
       and tranverter LO frequency. If we are not tracking, apply only LO frequency.
     */
    satfreqd = gtk_freq_knob_get_value(GTK_FREQ_KNOB(ctrl->SatFreqDown));
    satfrequ = gtk_freq_knob_get_value(GTK_FREQ_KNOB(ctrl->SatFreqUp));
    if (ctrl->tracking)
    {
        /* downlink */
        gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqDown),
                                use_rit_xit ?
                                satfreqd - ctrl->conf->lo :
                                satfreqd + ctrl->dd - ctrl->conf->lo);
        /* uplink */
        gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp),
                                use_rit_xit ?
                                satfrequ - ctrl->conf->loup :
                                satfrequ + ctrl->du - ctrl->conf->loup);
    }
    else
    {
        gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqDown),
                                satfreqd - ctrl->conf->lo);
        gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp),
                                satfrequ - ctrl->conf->loup);
    }

    base_freq = satfrequ - ctrl->conf->loup;
    doppler_hz = (ctrl->tracking && !use_rit_xit) ? ctrl->du : 0.0;
    sent_freq = base_freq + doppler_hz;
    tmpfreq = sent_freq;

    /* if device is engaged, send freq command to radio */
    if ((ctrl->engaged) && (fabs(ctrl->lasttxf - tmpfreq) >= 1.0))
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
                    get_freq_toggle_vfo(ctrl, ctrl->sock, &readback, sat_vfo) :
                    get_freq_toggle(ctrl, ctrl->sock, &readback);
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            "rig update: side=TX readback=%.0f ok=%d",
                            readback, read_ok ? 1 : 0);

                if (read_ok && fabs(readback - sent_freq) <= 100.0)
                {
                    ctrl->errcnt = 0;
                    ctrl->lasttxf = readback;
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
    gdouble         tmpfreq, readfreq, satfreqd, satfrequ;
    gboolean        dialchanged = FALSE;
    gboolean        rx_use_rit_xit =
        (ctrl->conf != NULL) ? ctrl->conf->supports_rit_xit : FALSE;
    gboolean        tx_use_rit_xit =
        (ctrl->conf2 != NULL) ? ctrl->conf2->supports_rit_xit : FALSE;

    /* Execute downlink cycle using ctrl->conf */
    if (ctrl->engaged && (ctrl->lastrxf > 0.0))
    {
        /* get frequency from receiver */
        if (!get_freq_simplex(ctrl, ctrl->sock, &readfreq))
        {
            /* error => use a passive value */
            readfreq = ctrl->lastrxf;
            ctrl->errcnt++;
        }

        if (fabs(readfreq - ctrl->lastrxf) >= 1.0)
        {
            dialchanged = TRUE;

            /* user might have altered radio frequency => update transponder knob */
            gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqDown),
                                    readfreq);
            ctrl->lastrxf = readfreq;

            /* doppler shift; only if we are tracking */
            if (ctrl->tracking)
            {
                if (rx_use_rit_xit)
                    satfreqd = readfreq + ctrl->conf->lo;
                else
                    satfreqd = readfreq - ctrl->dd + ctrl->conf->lo;
            }
            else
            {
                satfreqd = readfreq + ctrl->conf->lo;
            }
            gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->SatFreqDown),
                                    satfreqd);

            /* Update uplink if locked to downlink */
            if (ctrl->trsplock)
            {
                track_downlink(ctrl);
            }
        }
    }

    if (dialchanged)
    {
        /* update uplink */
        satfrequ = gtk_freq_knob_get_value(GTK_FREQ_KNOB(ctrl->SatFreqUp));
        if (ctrl->tracking && !tx_use_rit_xit)
        {
            gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp),
                                    satfrequ + ctrl->du - ctrl->conf2->loup);
        }
        else
        {
            gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp),
                                    satfrequ - ctrl->conf2->loup);
        }

        tmpfreq = gtk_freq_knob_get_value(GTK_FREQ_KNOB(ctrl->RigFreqUp));

        /* if device is engaged, send freq command to radio */
        if ((ctrl->engaged) && (fabs(ctrl->lasttxf - tmpfreq) >= 1.0))
        {
            if (set_freq_simplex(ctrl, ctrl->sock2, tmpfreq))
            {
                /* reset error counter */
                ctrl->errcnt = 0;

                /* give radio a chance to set frequency */
                g_usleep(WR_DEL);

                /* The actual frequency migh be different from what we have set */
                get_freq_simplex(ctrl, ctrl->sock2, &tmpfreq);
                ctrl->lasttxf = tmpfreq;
            }
            else
            {
                ctrl->errcnt++;
            }
        }
    }                           /* dialchanged on downlink */
    else
    {
        /* if no dial change on downlink perform forward tracking on downlink
           and execute uplink controller too */
        satfreqd = gtk_freq_knob_get_value(GTK_FREQ_KNOB(ctrl->SatFreqDown));
        if (ctrl->tracking)
        {
            /* downlink */
            gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqDown),
                                    rx_use_rit_xit ?
                                    satfreqd - ctrl->conf->lo :
                                    satfreqd + ctrl->dd - ctrl->conf->lo);
        }
        else
        {
            gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqDown),
                                    satfreqd - ctrl->conf->lo);
        }

        tmpfreq = gtk_freq_knob_get_value(GTK_FREQ_KNOB(ctrl->RigFreqDown));

        /* if device is engaged, send freq command to radio */
        if ((ctrl->engaged) && (fabs(ctrl->lastrxf - tmpfreq) >= 1.0))
        {
            if (set_freq_simplex(ctrl, ctrl->sock, tmpfreq))
            {
                /* reset error counter */
                ctrl->errcnt = 0;

                /* give radio a chance to set frequency */
                g_usleep(WR_DEL);

                /* The actual frequency migh be different from what we have set */
                get_freq_simplex(ctrl, ctrl->sock, &tmpfreq);
                ctrl->lastrxf = tmpfreq;
            }
            else
            {
                ctrl->errcnt++;
            }
        }

        /* Now execute uplink controller */

        /* check if uplink dial has changed */
        if ((ctrl->engaged) && (ctrl->lasttxf > 0.0))
        {
            if (!get_freq_simplex(ctrl, ctrl->sock2, &readfreq))
            {
                /* error => use a passive value */
                readfreq = ctrl->lasttxf;
                ctrl->errcnt++;
            }

            if (fabs(readfreq - ctrl->lasttxf) >= 1.0)
            {
                dialchanged = TRUE;

                gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp),
                                        readfreq);
                ctrl->lasttxf = readfreq;

                /* doppler shift; only if we are tracking */
                if (ctrl->tracking)
                {
                    if (tx_use_rit_xit)
                        satfrequ = readfreq + ctrl->conf2->loup;
                    else
                        satfrequ = readfreq - ctrl->du + ctrl->conf2->loup;
                }
                else
                {
                    satfrequ = readfreq + ctrl->conf2->loup;
                }
                gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->SatFreqUp),
                                        satfrequ);

                /* Follow with downlink if transponder is locked */
                if (ctrl->trsplock)
                {
                    track_uplink(ctrl);
                }
            }
        }

        if (dialchanged)
        {                       /* on uplink */
            /* update downlink */
            satfreqd =
                gtk_freq_knob_get_value(GTK_FREQ_KNOB(ctrl->SatFreqDown));
            if (ctrl->tracking)
            {
                gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqDown),
                                        rx_use_rit_xit ?
                                        satfreqd - ctrl->conf->lo :
                                        satfreqd + ctrl->dd - ctrl->conf->lo);
            }
            else
            {
                gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqDown),
                                        satfreqd - ctrl->conf->lo);
            }

            tmpfreq =
                gtk_freq_knob_get_value(GTK_FREQ_KNOB(ctrl->RigFreqDown));

            /* if device is engaged, send freq command to radio */
            if ((ctrl->engaged) && (fabs(ctrl->lastrxf - tmpfreq) >= 1.0))
            {
                if (set_freq_simplex(ctrl, ctrl->sock, tmpfreq))
                {
                    /* reset error counter */
                    ctrl->errcnt = 0;

                    /* give radio a chance to set frequency */
                    g_usleep(WR_DEL);

                    /* The actual frequency migh be different from what we have set */
                    get_freq_simplex(ctrl, ctrl->sock, &tmpfreq);
                    ctrl->lastrxf = tmpfreq;
                }
                else
                {
                    ctrl->errcnt++;
                }
            }
        }                       /* dialchanged on uplink */
        else
        {
            /* perform forward tracking on uplink */
            satfrequ = gtk_freq_knob_get_value(GTK_FREQ_KNOB(ctrl->SatFreqUp));
            if (ctrl->tracking)
            {
                gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp),
                                        tx_use_rit_xit ?
                                        satfrequ - ctrl->conf2->loup :
                                        satfrequ + ctrl->du -
                                        ctrl->conf2->loup);
            }
            else
            {
                gtk_freq_knob_set_value(GTK_FREQ_KNOB(ctrl->RigFreqUp),
                                        satfrequ - ctrl->conf2->loup);
            }

            tmpfreq = gtk_freq_knob_get_value(GTK_FREQ_KNOB(ctrl->RigFreqUp));

            /* if device is engaged, send freq command to radio */
            if ((ctrl->engaged) && (fabs(ctrl->lasttxf - tmpfreq) >= 1.0))
            {
                if (set_freq_simplex(ctrl, ctrl->sock2, tmpfreq))
                {
                    /* reset error counter */
                    ctrl->errcnt = 0;

                    /* give radio a chance to set frequency */
                    g_usleep(WR_DEL);

                    /* The actual frequency might be different from what we have set. */
                    get_freq_simplex(ctrl, ctrl->sock2, &tmpfreq);
                    ctrl->lasttxf = tmpfreq;
                }
                else
                {
                    ctrl->errcnt++;
                }
            }
        }                       /* else dialchange on uplink */
    }                           /* else dialchange on downlink */
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

    if (is_ic9700_satmode_active(ctrl))
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
        if (is_ic9700_satmode_configured(ctrl->conf))
            buff = g_strdup_printf("I currVFO %10.0f\x0a", freq);
        else
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
    buff = g_strdup_printf("S %s 1 %d\x0a", ctrl->conf->vfoDown==VFO_A?"VFOA":"VFOB", ctrl->conf->vfoDown);
    else
    buff = g_strdup_printf("S 1 %d\x0a", ctrl->conf->vfoDown);
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
        buff = g_strdup_printf("S VFOA 0 %d\x0a", ctrl->conf->vfoDown);
    else
        buff = g_strdup_printf("S 0 %d\x0a", ctrl->conf->vfoDown);
    retcode = send_rigctld_command(ctrl, sock, buff, buffback, 128);
    g_free(buff);

    return (check_set_response(buffback, retcode, __func__));
}

/*
 * Get frequency
 *
 * Returns TRUE if the operation was successful, FALSE otherwise
 */
static gboolean get_freq_simplex(GtkRigCtrl * ctrl, gint sock, gdouble * freq)
{
    gchar          *buff, **vbuff;
    gchar           buffback[128];
    gboolean        retcode;
    gboolean        retval = TRUE;

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
    return retval;
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
static gboolean get_freq_toggle(GtkRigCtrl * ctrl, gint sock, gdouble * freq)
{
    gchar          *buff, **vbuff;
    gchar           buffback[128];
    gboolean        retcode;
    gboolean        retval = TRUE;

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
    return retval;
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
                                         gint *sock)
{
    const gchar    *target = host;

    if (host == NULL || sock == NULL)
        return FALSE;

    if (port <= 0)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: Missing rigctld host/port"), __func__);
        return FALSE;
    }

    if (g_ascii_strcasecmp(host, "localhost") == 0)
        target = "127.0.0.1";

    if (rigctld_connect_addrinfo(target, port, sock))
        return TRUE;

    sat_log_log(SAT_LOG_LEVEL_ERROR,
                _("%s: Failed to connect to %s:%d"),
                __func__, target, port);
    return FALSE;
}

static gboolean close_rigctld_socket(gint * sock)
{
    gint            written;

    if (sock == NULL || *sock == -1)
        return TRUE;

    written = send(*sock, "q\x0a", 2, 0);
    if (written != 2)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%s: Sent 2 bytes but sent %d."),
                    __FILE__, __func__, written);
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

    close_rigctld_socket(sock_ptr);
    rigctrl_schedule_reconnect(ctrl, secondary, role);
}


static gboolean ensure_rigctld_running(GtkRigCtrl *ctrl,
                                       radio_conf_t *conf,
                                       RigctldMgr **mgr,
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

    if (error_reported)
        *error_reported = FALSE;

    if (connect_host)
        *connect_host = NULL;

    if (conf == NULL)
        return FALSE;

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
        schedule_rig_autostart_error(ctrl, role, detail);
        rig_term_log(ctrl, "gpredict:err",
                     "auto-start only supported for 127.0.0.1 (host=%s)",
                     conf->host ? conf->host : "(missing)");
        g_free(detail);
        reported = TRUE;
        goto out;
    }

    if (mgr && *mgr != NULL && !rigctld_mgr_is_running(*mgr))
        rigctld_mgr_terminate(mgr);

    if (rigctld_mgr_port_is_open(host, conf->port, 200))
    {
        rig_term_log(ctrl, "gpredict",
                     "rigctld reachable at %s:%d", host, conf->port);
        ok = TRUE;
        goto out;
    }

    if (mgr && *mgr == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: rigctld not reachable; attempting auto-start for %s:%d"),
                    __func__, host, conf->port);
        rig_term_log(ctrl, "gpredict",
                     "auto-start rigctld for %s:%d", host, conf->port);
        *mgr = rigctld_mgr_spawn(conf, host, &errmsg);
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
            schedule_rig_autostart_error(ctrl, role, detail);
            g_free(detail);
            g_free(errmsg);
            reported = TRUE;
            goto out;
        }

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

    if (!rigctld_mgr_wait_for_port(host, conf->port, 5000))
    {
        gchar *stderr_text = NULL;

        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: auto-start failed; rigctld not listening on %s:%d"),
                    __func__, host, conf->port);
        rig_term_log(ctrl, "gpredict:err",
                     "rigctld not listening on %s:%d",
                     host, conf->port);

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

        schedule_rig_autostart_error(ctrl, role, detail);
        rigctld_mgr_terminate(mgr);
        g_free(stderr_text);
        g_free(detail);
        reported = TRUE;
        goto out;
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

    if (!ensure_rigctld_running(ctrl, conf, mgr, role, &host, &reported))
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
    if (!open_rigctld_socket_host(host, conf->port, sock))
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: Failed to connect to %s:%d"),
                    __func__, host, conf->port);
        rig_term_log(ctrl, "gpredict:err",
                     "connect failed to %s:%d", host, conf->port);
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
                              const gchar *label)
{
    gchar           buffback[128];
    gboolean        ok;
    const gchar    *role = (label != NULL) ? label : _("rig");

    ok = send_rigctld_command(ctrl, sock, "f\x0a", buffback, 128);
    if (!ok)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: probe failed to write/read (%s)"),
                    __func__, role);
        return FALSE;
    }

    if (g_str_has_prefix(buffback, "RPRT"))
    {
        if (g_str_has_prefix(buffback, "RPRT 0"))
        {
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        _("%s: probe got RPRT 0 from %s"), __func__, role);
            return TRUE;
        }
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: probe got error reply from %s: %s"),
                    __func__, role, buffback);
        return FALSE;
    }

    sat_log_log(SAT_LOG_LEVEL_INFO,
                _("%s: probe succeeded for %s (reply: %s)"),
                __func__, role, buffback);
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
    gchar            *body;

    if (info == NULL)
        return G_SOURCE_REMOVE;

    body = g_strdup_printf(_("Unable to connect to rigctld (%s)\nHost: %s\nPort: %d"),
                           info->role ? info->role : _("rig"),
                           info->host ? info->host : "(null) - missing",
                           info->port);
    rig_show_error_dialog(info->ctrl,
                          _("Unable to connect to rigctld"),
                          body);
    g_free(body);
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
    gchar      *title;
    gchar      *body;
} RigAutostartErrorInfo;

static gboolean rig_autostart_error_idle(gpointer data)
{
    RigAutostartErrorInfo *info = data;

    if (info == NULL)
        return G_SOURCE_REMOVE;

    rig_show_error_dialog(info->ctrl,
                          info->title ? info->title : _("rigctld error"),
                          info->body ? info->body : "");
    g_free(info->title);
    g_free(info->body);
    g_free(info);

    return G_SOURCE_REMOVE;
}

static void schedule_rig_autostart_error(GtkRigCtrl *ctrl,
                                         const gchar *role,
                                         const gchar *detail)
{
    RigAutostartErrorInfo *info = g_new0(RigAutostartErrorInfo, 1);
    const gchar *label = (role != NULL) ? role : _("rig");

    info->ctrl = ctrl;
    info->title = g_strdup_printf(_("Unable to auto-start rigctld (%s)"), label);
    info->body = g_strdup(detail ? detail : "");

    g_idle_add(rig_autostart_error_idle, info);
}

static gboolean rig_disengage_idle(gpointer data)
{
    GtkRigCtrl *ctrl = GTK_RIG_CTRL(data);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->LockBut), FALSE);
    return G_SOURCE_REMOVE;
}

static void schedule_rig_disengage(GtkRigCtrl *ctrl)
{
    g_idle_add(rig_disengage_idle, ctrl);
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
        close_rigctld_socket(&(ctrl->sock2));
    }
    close_rigctld_socket(&(ctrl->sock));

    rigctld_mgr_terminate(&ctrl->rigctld_mgr2);
    rigctld_mgr_terminate(&ctrl->rigctld_mgr);

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
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: opening receiver rig %s:%d"), __func__,
                    ctrl->conf ? ctrl->conf->host : "(null)",
                    ctrl->conf ? ctrl->conf->port : 0);
        g_printerr("%s: opening receiver rig %s:%d\n", __func__,
                   ctrl->conf ? ctrl->conf->host : "(null)",
                   ctrl->conf ? ctrl->conf->port : 0);

        {
            gboolean error_reported = FALSE;

            if (!open_rigctld_socket_with_autostart(ctrl, ctrl->conf,
                                                    &(ctrl->sock),
                                                    FALSE, _("receiver"),
                                                    &error_reported) ||
                !probe_rigctld(ctrl, ctrl->sock, _("receiver rig")))
            {
                sat_log_log(SAT_LOG_LEVEL_ERROR,
                            _("%s: receiver rig open/probe failed"), __func__);
                close_rigctld_socket(&(ctrl->sock));
                if (!error_reported && !ctrl->rx_conn_error_reported)
                {
                    schedule_rig_conn_error(ctrl, ctrl->conf, _("receiver"));
                    ctrl->rx_conn_error_reported = TRUE;
                }
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
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: opening uplink rig %s:%d"), __func__,
                    ctrl->conf2 ? ctrl->conf2->host : "(null)",
                    ctrl->conf2 ? ctrl->conf2->port : 0);
        g_printerr("%s: opening uplink rig %s:%d\n", __func__,
                   ctrl->conf2 ? ctrl->conf2->host : "(null)",
                   ctrl->conf2 ? ctrl->conf2->port : 0);

        {
            gboolean error_reported = FALSE;

            tx_ok = open_rigctld_socket_with_autostart(
                ctrl, ctrl->conf2, &(ctrl->sock2), TRUE, _("uplink"),
                &error_reported);
            if (tx_ok)
                tx_ok = probe_rigctld(ctrl, ctrl->sock2, _("uplink rig"));
            if (!tx_ok)
            {
                sat_log_log(SAT_LOG_LEVEL_ERROR,
                            _("%s: uplink rig open/probe failed"), __func__);
                close_rigctld_socket(&(ctrl->sock2));
                if (!error_reported && !ctrl->tx_conn_error_reported)
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
        return FALSE;

    if ((rx_opened || tx_opened))
    {
        if (ctrl->conf2 != NULL)
        {
            if (ctrl->sock >= 0 && ctrl->sock2 >= 0)
                exec_dual_rig_cycle(ctrl);
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
                    rigctrl_schedule_reconnect(t_ctrl, TRUE, _("uplink"));
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
