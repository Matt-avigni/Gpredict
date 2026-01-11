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

/* NETWORK */
#ifndef WIN32
#ifdef _WIN32
  #include <winsock2.h>   /* htons(), etc. */
  #include <ws2tcpip.h>
#else
  #include <arpa/inet.h>  /* htons(), etc. */
#endif

#include <arpa/inet.h>          /* htons() */
#include <netdb.h>              /* gethostbyname() */
#include <netinet/in.h>         /* struct sockaddr_in */
#include <sys/socket.h>         /* socket(), connect(), send() */
#else
#include <winsock2.h>
#include <windows.h>
#endif

#include <errno.h>
#include <glib.h>
#include <glib/gi18n.h>
#include <gio/gio.h>
#include <gtk/gtk.h>
#include <math.h>
#if defined(__APPLE__) || defined(__linux__) || defined(__FreeBSD__)
#include <locale.h>
#if defined(__APPLE__)
#include <xlocale.h>
#endif
#endif
#include <string.h>             /* strerror() */
#include <stdarg.h>

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
#include "rotor-cmd-map.h"
#include "rotor-trajectory-planner.h"

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
#define ROT_PLAN_EL_WEIGHT 1.0
#define ROT_PLAN_NEAR_LIMIT_MARGIN 2.0
#define ROT_PLAN_NEAR_LIMIT_PENALTY 5.0

typedef struct {
    GThread        *thread;
    GMutex          mutex;
    gint            socket;
    gboolean        running, new_trg;
    gdouble         azi_out, ele_out;
    gdouble         azi_in, ele_in;
    gboolean        io_error;
    gboolean        cmd_rejected;
    gint64          reject_backoff_until_us;
    gdouble         reject_backoff_sec;
    gint64          reject_backoff_log_us;
    gboolean        limits_valid;
    gdouble         az_min, az_max;
    gdouble         el_min, el_max;
    gboolean        daemon_ok;
    GTimer         *timer;
} rotctld_client_t;

typedef enum {
    ROT_PLAN_MODE_NORMAL = 0,
    ROT_PLAN_MODE_FLIP   = 1
} rot_plan_mode_t;

typedef struct {
    gboolean        valid;
    rot_plan_mode_t mode;
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

struct _GtkRotCtrl {
    GtkBox          box;

    GtkWidget      *AzSet, *AzRead;
    GtkWidget      *ElSet, *ElRead;
    GtkWidget      *SatSel, *AzSat, *ElSat, *SatCnt;
    GtkWidget      *DevSel, *LockBut, *MonitorCheckBox;
    GtkWidget      *track, *cycle_spin, *thld_spin;
    GtkWidget      *plot;

    GSList         *sats;
    sat_t          *target;
    pass_t         *pass;
    qth_t          *qth;

    guint           delay, timerid;
    gdouble         threshold, t;
    gint            errcnt;

    gboolean        tracking, engaged, monitor, flipped;

    rotor_conf_t   *conf;
    rotctld_client_t client;
    rot_plan_t      trajectory_plan;

    GpTermView     *term_view;
    GtkWidget      *log_toggle;
    GSubprocess    *rotctld_proc;
    GThread        *rotctld_out_thread;
    GThread        *rotctld_err_thread;
    gboolean        rotctld_spawned;
    gboolean        verbose_logging;

    gboolean        use_offset;
    gdouble         az_offset_deg, el_offset_deg;
    GtkWidget      *offset_check;
    GtkWidget      *az_offset_spin;
    GtkWidget      *el_offset_spin;

    /* Reserved flag; currently always kept FALSE (no special SEND-ONLY mode). */
    gboolean        send_only_mode;

    gboolean        out_of_range;
    gint64          last_oob_log_us;
    gdouble         last_oob_raw_az, last_oob_raw_el;
    gdouble         last_oob_mapped_az, last_oob_mapped_el;
};

struct _GtkRotCtrlClass {
    GtkBoxClass     parent_class;
};


static GtkVBoxClass *parent_class = NULL;

/* Forward declaration for error dialog helper */

static void rot_show_no_rotor_dialog(GtkRotCtrl *ctrl);
static void rot_show_conf_error(GtkRotCtrl *ctrl, const gchar *reason);
static void rot_show_plan_error(GtkRotCtrl *ctrl, const gchar *reason);
static void rot_logs_toggle_cb(GtkToggleButton *button, gpointer data);
static void rot_verbose_cb(GtkToggleButton *button, gpointer data);
static void rotctrl_force_toplevel_resize(GtkRotCtrl *ctrl);
static void rot_schedule_wrong_daemon(GtkRotCtrl *ctrl,
                                      const gchar *host, gint port);
static void rot_schedule_cmd_reject(GtkRotCtrl *ctrl, const gchar *reason);
static void rot_log_out_of_range(GtkRotCtrl *ctrl,
                                 gdouble raw_az, gdouble raw_el,
                                 const rot_cmd_map_t *map);

static void rot_term_log(GtkRotCtrl *ctrl, const gchar *prefix,
                         const gchar *fmt, ...) G_GNUC_PRINTF(3, 4);
static void rot_term_log_verbose(GtkRotCtrl *ctrl, const gchar *prefix,
                                 const gchar *fmt, ...) G_GNUC_PRINTF(3, 4);
static void rot_term_log_tx(GtkRotCtrl *ctrl, const gchar *cmd);
static void rot_term_log_rx(GtkRotCtrl *ctrl, const gchar *cmd,
                            const gchar *reply);
static gboolean rot_parse_first_number(const gchar *line, gdouble *out);

static void rotctld_process_stop(GtkRotCtrl *ctrl);
static gboolean rotctld_spawn_process(GtkRotCtrl *ctrl, gchar **argv);
static gchar **rotctld_build_autostart_argv(GtkRotCtrl *ctrl);
static gchar **rotctld_build_argv_from_command(GtkRotCtrl *ctrl,
                                               const gchar *cmdline);

/* Offset controls callbacks */

static void offset_toggle_cb(GtkToggleButton *button, gpointer data);
static void az_offset_changed_cb(GtkSpinButton *spin, gpointer data);
static void el_offset_changed_cb(GtkSpinButton *spin, gpointer data);

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

static void rot_term_log(GtkRotCtrl *ctrl, const gchar *prefix,
                         const gchar *fmt, ...)
{
    va_list ap;
    gchar *msg = NULL;
    gchar *stamp = NULL;

    if (ctrl == NULL || ctrl->term_view == NULL || prefix == NULL || fmt == NULL)
        return;

    va_start(ap, fmt);
    msg = g_strdup_vprintf(fmt, ap);
    va_end(ap);

    if (msg == NULL)
        return;

    stamp = rot_term_format_timestamp();
    gp_term_view_log(ctrl->term_view, "%s [%s] %s", stamp, prefix, msg);
    g_free(stamp);
    g_free(msg);
}

static void rot_term_log_verbose(GtkRotCtrl *ctrl, const gchar *prefix,
                                 const gchar *fmt, ...)
{
    va_list ap;
    gchar *msg = NULL;
    gchar *stamp = NULL;

    if (ctrl == NULL || !ctrl->verbose_logging ||
        ctrl->term_view == NULL || prefix == NULL || fmt == NULL)
        return;

    va_start(ap, fmt);
    msg = g_strdup_vprintf(fmt, ap);
    va_end(ap);

    if (msg == NULL)
        return;

    stamp = rot_term_format_timestamp();
    gp_term_view_log(ctrl->term_view, "%s [%s] %s", stamp, prefix, msg);
    g_free(stamp);
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

static void rot_term_log_tx(GtkRotCtrl *ctrl, const gchar *cmd)
{
    gchar *trim;

    if (ctrl == NULL || cmd == NULL)
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

    if (g_str_has_prefix(trim_cmd, "P "))
    {
        if (rot_parse_rprt_code(trim_reply, &code))
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
        if (rot_parse_rprt_code(trim_reply, &code))
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
        rot_term_log(ctrl, "gpredict:rx", "dump_state ok");
    }
    else
    {
        rot_term_log(ctrl, "gpredict:rx", "%s", trim_reply);
    }

    g_free(trim_cmd);
    g_free(trim_reply);
}

static void rot_log_out_of_range(GtkRotCtrl *ctrl,
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


/* Open the rotcld socket. Returns file descriptor or -1 if an error occurs */
static gint rotctld_socket_open(const gchar * host, gint port)
{
    struct sockaddr_in ServAddr;
    struct hostent *h;
    gint            sock;
    gint            status;
    const gchar    *target = host;

    if (host == NULL)
        return -1;

    if (g_ascii_strcasecmp(host, "localhost") == 0)
        target = "127.0.0.1";

    sock = socket(PF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock == -1)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("Failed to create rotctl socket: %s"), strerror(errno));
        return sock;
    }

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                _("%s: Network socket created successfully"), __func__);

    memset(&ServAddr, 0, sizeof(ServAddr));
    ServAddr.sin_family = AF_INET;      /* Internet address family */
    h = gethostbyname(target);
    if (h == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("Name resolution of rotctld server %s failed."), target);
#ifdef WIN32
        closesocket(sock);
#else
        close(sock);
#endif
        return -1;
    }

    memcpy((char *)&ServAddr.sin_addr.s_addr, h->h_addr_list[0], h->h_length);
    ServAddr.sin_port = htons(port);    /* Server port */

    /* establish connection */
    status = connect(sock, (struct sockaddr *)&ServAddr, sizeof(ServAddr));
    if (status == -1)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("Connection to rotctld server at %s:%d failed: %s"),
                    target, port, strerror(errno));

#ifdef WIN32
        closesocket(sock);
#else
        close(sock);
#endif
        return -1;
    }

    sat_log_log(SAT_LOG_LEVEL_DEBUG, _("%s: Connection opened to %s:%d"),
                __func__, target, port);

    return sock;
}

/* Close a rotcld socket. First send a q command to cleanly shut down rotctld */
static void rotctld_socket_close(GtkRotCtrl *ctrl, gint * sock)
{
    gint            written;

    if (sock == NULL || *sock == -1)
        return;

    if (ctrl != NULL)
        rot_term_log_tx(ctrl, "q");

    /*shutdown the rotctld connect */
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
}

/* Close a rotctld socket without sending shutdown commands. */
static void rotctld_socket_close_quiet(gint *sock)
{
    if (sock == NULL || *sock == -1)
        return;

#ifndef WIN32
    shutdown(*sock, SHUT_RDWR);
    close(*sock);
#else
    shutdown(*sock, SD_BOTH);
    closesocket(*sock);
#endif

    *sock = -1;
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
    gint            written;
    gint            size;
    gint64          start_us = 0;
    const gchar    *host = (ctrl && ctrl->conf && ctrl->conf->host)
                           ? ctrl->conf->host
                           : "(null)";
    gint            port = (ctrl && ctrl->conf) ? ctrl->conf->port : 0;

    if (ctrl != NULL)
        rot_term_log_tx(ctrl, buff);

    if (ctrl != NULL && ctrl->verbose_logging)
        start_us = g_get_monotonic_time();

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                _("%s: sending command '%s'"), __func__, buff);
    size = strlen(buff);

    /* send command */
    written = send(sock, buff, size, 0);
    if (written != size)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: SIZE ERROR %d / %d"), __func__, written, size);
    }
    if (written == -1)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: rotctld Socket Down"), __func__);
        if (ctrl != NULL)
        {
            gchar *cmd = g_strdup(buff ? buff : "");
            g_strchomp(cmd);
            rot_term_log(ctrl, "gpredict:err",
                         "%s failed: send '%s' to %s:%d (%s)",
                         __func__, cmd, host, port, strerror(errno));
            g_free(cmd);
        }
        return FALSE;
    }

    /* try to read answer, but with a timeout using select() */
    {
#ifndef WIN32
        fd_set fds;
        struct timeval tv;
        int ret;
        FD_ZERO(&fds);
        FD_SET(sock, &fds);
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        ret = select(sock + 1, &fds, NULL, NULL, &tv);
        if (ret <= 0)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: select() timeout or error waiting for rotctld reply"), __func__);
            if (ctrl != NULL)
            {
                gchar *cmd = g_strdup(buff ? buff : "");
                g_strchomp(cmd);
                rot_term_log(ctrl, "gpredict:err",
                             "%s failed: timeout waiting for reply to '%s' from %s:%d",
                             __func__, cmd, host, port);
                g_free(cmd);
            }
            return FALSE;
        }
#else
        /* Windows version */
        fd_set fds;
        struct timeval tv;
        int ret;
        FD_ZERO(&fds);
        FD_SET(sock, &fds);
        tv.tv_sec = 1;
        tv.tv_usec = 0;
        ret = select(0, &fds, NULL, NULL, &tv);
        if (ret <= 0)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: select() timeout or error waiting for rotctld reply"), __func__);
            if (ctrl != NULL)
            {
                gchar *cmd = g_strdup(buff ? buff : "");
                g_strchomp(cmd);
                rot_term_log(ctrl, "gpredict:err",
                             "%s failed: timeout waiting for reply to '%s' from %s:%d",
                             __func__, cmd, host, port);
                g_free(cmd);
            }
            return FALSE;
        }
#endif
        /* Now safe to call recv() */
        size = recv(sock, buffout, sizeout, 0);
    }

    if (size == -1)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: rotctld Socket Down"), __func__);
        if (ctrl != NULL)
        {
            gchar *cmd = g_strdup(buff ? buff : "");
            g_strchomp(cmd);
            rot_term_log(ctrl, "gpredict:err",
                         "%s failed: recv error for '%s' from %s:%d (%s)",
                         __func__, cmd, host, port, strerror(errno));
            g_free(cmd);
        }
        return FALSE;
    }

    buffout[size] = '\0';
    if (size == 0)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%s: Got 0 bytes from rotctld"), __FILE__, __func__);
    }

    if (ctrl != NULL)
        rot_term_log_rx(ctrl, buff, buffout);

    if (ctrl != NULL && ctrl->verbose_logging && start_us > 0)
    {
        gint64 elapsed_us = g_get_monotonic_time() - start_us;
        rot_term_log_verbose(ctrl, "gpredict:rx",
                             "rotctld round-trip %.1f ms",
                             (gdouble) elapsed_us / 1000.0);
    }

    return TRUE;
}

static gint sat_name_compare(sat_t * a, sat_t * b)
{
    return (gpredict_strcmp(a->nickname, b->nickname));
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
    if (type == ROT_AZ_TYPE_360)
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
    rot_cmd_get_abs_az_limits(conf, az_min, az_max);
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

static gboolean rot_get_command(GtkRotCtrl *ctrl, gboolean plan_active,
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
    in.az_stop = normalize_az_0_360(ctrl->conf->azstoppos);
    {
        gdouble span = ctrl->conf->maxaz - ctrl->conf->minaz;
        if (span <= 0.0)
            span = 360.0;
        in.use_az_stop = (span < 359.0);
    }
    in.az_current = cur_az;
    in.el_current = cur_el;
    in.have_current = TRUE;

    normal_samples = rot_build_samples(ctrl, ctrl->pass,
                                       ROT_PLAN_MODE_NORMAL, t0, t1);
    in.samples = normal_samples;
    have_normal = rot_plan_build(&in, &normal);

    flip_allowed = (ctrl->conf->maxel >= 180.0);
    if (flip_allowed) {
        flip_samples = rot_build_samples(ctrl, ctrl->pass,
                                         ROT_PLAN_MODE_FLIP, t0, t1);
        in.samples = flip_samples;
        have_flip = rot_plan_build(&in, &flip);
    }

    if (have_normal)
        chosen = &normal;
    if (have_flip) {
        if (chosen == NULL ||
            (flip.trackable_pct > chosen->trackable_pct) ||
            (flip.trackable_pct == chosen->trackable_pct &&
             (flip.violation_mag < chosen->violation_mag ||
              (flip.violation_mag == chosen->violation_mag &&
               flip.total_motion < chosen->total_motion))))
            chosen = &flip;
    }

    if (chosen != NULL) {
        ctrl->trajectory_plan.valid = TRUE;
        ctrl->trajectory_plan.mode =
            (chosen == &flip) ? ROT_PLAN_MODE_FLIP : ROT_PLAN_MODE_NORMAL;
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
static gboolean get_pos(GtkRotCtrl * ctrl, gdouble * az, gdouble * el)
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

static gboolean rot_parse_first_number(const gchar *line, gdouble *out)
{
    if (line == NULL || out == NULL)
        return FALSE;

    const gchar *p = line;
    while (*p != '\0' && !g_ascii_isdigit(*p) && *p != '-' && *p != '+')
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

typedef enum {
    ROT_DAEMON_UNKNOWN = 0,
    ROT_DAEMON_ROTCTLD = 1,
    ROT_DAEMON_RIGCTLD = 2
} rot_daemon_type_t;

static rot_daemon_type_t rotctld_detect_daemon(const gchar *reply)
{
    gchar *lower;
    gboolean has_rot = FALSE;
    gboolean has_rig = FALSE;

    if (reply == NULL || *reply == '\0')
        return ROT_DAEMON_UNKNOWN;

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

static rot_daemon_type_t rotctld_dump_state(GtkRotCtrl *ctrl, gint sock,
                                            gchar *reply, gsize reply_size)
{
    gchar cmd[] = "\\dump_state\n";

    if (reply == NULL || reply_size == 0)
        return ROT_DAEMON_UNKNOWN;

    reply[0] = '\0';
    if (!rotctld_socket_rw(ctrl, sock, cmd, reply, reply_size - 1))
        return ROT_DAEMON_UNKNOWN;

    return rotctld_detect_daemon(reply);
}

static gboolean rotctld_parse_position_reply(const gchar *reply,
                                             gdouble *az_out, gdouble *el_out,
                                             gint *rprt_code_out)
{
    gboolean have_az = FALSE;
    gboolean have_el = FALSE;
    gint rprt_code = 0;
    gboolean have_code = FALSE;

    if (reply == NULL)
        return FALSE;

    gchar **lines = g_strsplit(reply, "\n", -1);
    for (gint i = 0; lines[i] != NULL; i++) {
        gchar *line = g_strstrip(lines[i]);
        if (line[0] == '\0')
            continue;

        if (g_str_has_prefix(line, "RPRT")) {
            if (rot_parse_rprt_code(line, &rprt_code))
                have_code = TRUE;
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

    return have_az && have_el;
}

static gboolean rotctld_handshake(GtkRotCtrl *ctrl, gint sock,
                                  gboolean *rejected)
{
    gchar reply[256];
    gchar txbuf[64];
    gchar azbuf[G_ASCII_DTOSTR_BUF_SIZE];
    gchar elbuf[G_ASCII_DTOSTR_BUF_SIZE];
    gdouble az = 0.0;
    gdouble el = 0.0;
    gint rprt_code = 0;
    gboolean have_pos = FALSE;

    if (rejected)
        *rejected = FALSE;

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

    g_ascii_formatd(azbuf, sizeof(azbuf), "%.2f", az);
    g_ascii_formatd(elbuf, sizeof(elbuf), "%.2f", el);
    g_snprintf(txbuf, sizeof(txbuf), "P %s %s\n", azbuf, elbuf);

    if (!rotctld_socket_rw(ctrl, sock, txbuf, reply, sizeof(reply) - 1))
        return FALSE;

    g_strstrip(reply);
    if (rot_parse_rprt_code(reply, &rprt_code) && rprt_code != 0) {
        if (rejected)
            *rejected = TRUE;
        return FALSE;
    }

    return TRUE;
}

typedef enum {
    ROT_SET_OK = 0,
    ROT_SET_REJECTED = 1,
    ROT_SET_IO_ERROR = 2
} rot_set_result_t;

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
static rot_set_result_t set_pos(GtkRotCtrl * ctrl, gdouble az, gdouble el)
{
    gchar           txbuf[64];
    gchar           buffback[128];
    gboolean        retcode;
    gchar           azbuf[G_ASCII_DTOSTR_BUF_SIZE];
    gchar           elbuf[G_ASCII_DTOSTR_BUF_SIZE];
    gchar           send_az_str[32];
    gchar           send_el_str[32];
    gdouble         az_send = az;
    gdouble         el_send = el;
    const gchar    *host = (ctrl && ctrl->conf && ctrl->conf->host)
                           ? ctrl->conf->host
                           : "(null)";
    gint            port = (ctrl && ctrl->conf) ? ctrl->conf->port : 0;

    rot_format_deg_2(send_az_str, sizeof(send_az_str), az_send);
    rot_format_deg_2(send_el_str, sizeof(send_el_str), el_send);
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                _("%s: set_pos send=(%s, %s)"),
                __func__, send_az_str, send_el_str);
    g_printerr("MISSION_SOPHIE: set_pos send=(%s, %s)\n",
               send_az_str, send_el_str);

    /* send command (ASCII-safe, locale independent) */
    g_ascii_formatd(azbuf, sizeof(azbuf), "%.2f", az_send);
    g_ascii_formatd(elbuf, sizeof(elbuf), "%.2f", el_send);
    g_snprintf(txbuf, sizeof(txbuf), "P %s %s\n", azbuf, elbuf);
    g_message("ROTCTLD TX: '%s'", txbuf);
    
    retcode = rotctld_socket_rw(ctrl, ctrl->client.socket, txbuf, buffback,
                                128);

    if (!retcode) {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: rotctld I/O error while sending P command"), __func__);
        g_printerr("MISSION_SOPHIE: set_pos I/O error\n");
        rotctld_socket_close_quiet(&ctrl->client.socket);
        return ROT_SET_IO_ERROR;
    }

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                _("%s: rotctld replied '%s'"), __func__, buffback);
    g_printerr("MISSION_SOPHIE: set_pos reply '%s'\n", buffback);

    /* Interpret reply:
     *  - If it starts with "RPRT 0"  → success.
     *  - If it starts with "RPRT "   → treat as rejected.
     *  - Otherwise (numeric az/el echo, or something else) → assume success.
     */
    g_strstrip(buffback);

    gint rprt_code = 0;
    if (rot_parse_rprt_code(buffback, &rprt_code)) {
        if (rprt_code == 0)
            return ROT_SET_OK;

        gchar *cmd = g_strdup(txbuf);
        g_strchomp(cmd);
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: rotctld rejected cmd '%s' reply '%s'"),
                    __func__, cmd, buffback);
        rot_term_log(ctrl, "gpredict:err",
                     "set_position rejected for %s:%d cmd=%s reply=%s",
                     host, port, cmd, buffback);
        g_free(cmd);
        return ROT_SET_REJECTED;
    }

    /* Legacy / non-Hamlib style replies (e.g. numeric echoes) are treated as OK. */
    return ROT_SET_OK;
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
    ctrl->client.running = TRUE;
    ctrl->client.socket = -1;
    ctrl->client.cmd_rejected = FALSE;
    ctrl->client.reject_backoff_until_us = 0;
    ctrl->client.reject_backoff_sec = 0.5;
    ctrl->client.reject_backoff_log_us = 0;
    ctrl->client.limits_valid = FALSE;
    ctrl->client.daemon_ok = FALSE;

    sat_log_log(SAT_LOG_LEVEL_INFO,
                "MISSION_SOPHIE: rotctld_client_thread started for %s:%d",
                ctrl->conf ? ctrl->conf->host : "(null)",
                ctrl->conf ? ctrl->conf->port : 0);

    rot_term_log_verbose(ctrl, "gpredict:rx",
                         "rotctld client thread started for %s:%d",
                         ctrl->conf ? ctrl->conf->host : "(null)",
                         ctrl->conf ? ctrl->conf->port : 0);

    g_printerr("MISSION_SOPHIE: rotctld_client_thread started for %s:%d\n",
                ctrl->conf ? ctrl->conf->host : "(null)",
                ctrl->conf ? ctrl->conf->port : 0);

    while (ctrl->client.running)
    {
        g_timer_start(ctrl->client.timer);

        if (ctrl->client.socket == -1) {
            ctrl->client.socket = rotctld_socket_open(ctrl->conf->host,
                                                      ctrl->conf->port);
            if (ctrl->client.socket == -1) {
                io_error = TRUE;
                g_mutex_lock(&ctrl->client.mutex);
                ctrl->client.io_error = TRUE;
                ctrl->client.daemon_ok = FALSE;
                g_mutex_unlock(&ctrl->client.mutex);
                sat_log_log(SAT_LOG_LEVEL_WARN,
                            "%s: rotctld link down, retrying in %.1fs",
                            __func__, backoff_sec);
                rot_term_log(ctrl, "gpredict:err",
                             "rotctld connect failed to %s:%d; retrying in %.1fs",
                             ctrl->conf ? ctrl->conf->host : "(null)",
                             ctrl->conf ? ctrl->conf->port : 0,
                             backoff_sec);
                g_usleep((gulong)(backoff_sec * 1e6));
                backoff_sec = MIN(backoff_sec * 2.0, backoff_max);
                continue;
            }

            backoff_sec = 0.5;
            io_error = FALSE;
            g_mutex_lock(&ctrl->client.mutex);
            ctrl->client.io_error = FALSE;
            ctrl->client.daemon_ok = FALSE;
            ctrl->client.reject_backoff_until_us = 0;
            ctrl->client.reject_backoff_sec = 0.5;
            ctrl->client.reject_backoff_log_us = 0;
            g_mutex_unlock(&ctrl->client.mutex);
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        "%s: rotctld link re-established", __func__);
            rot_term_log_verbose(ctrl, "gpredict:rx",
                                 "rotctld link re-established");

            g_mutex_lock(&ctrl->client.mutex);
            ctrl->client.limits_valid = FALSE;
            ctrl->client.daemon_ok = FALSE;
            g_mutex_unlock(&ctrl->client.mutex);

            {
                gchar reply[4096];
                rot_daemon_type_t daemon =
                    rotctld_dump_state(ctrl, ctrl->client.socket, reply,
                                       sizeof(reply));

                if (daemon != ROT_DAEMON_ROTCTLD) {
                    sat_log_log(SAT_LOG_LEVEL_ERROR,
                                "%s: non-rotctld daemon on %s:%d",
                                __func__,
                                ctrl->conf ? ctrl->conf->host : "(null)",
                                ctrl->conf ? ctrl->conf->port : 0);
                    rot_term_log(ctrl, "gpredict:err",
                                 "connected to non-rotctld server on %s:%d",
                                 ctrl->conf ? ctrl->conf->host : "(null)",
                                 ctrl->conf ? ctrl->conf->port : 0);
                    rotctld_socket_close_quiet(&ctrl->client.socket);
                    g_mutex_lock(&ctrl->client.mutex);
                    ctrl->client.daemon_ok = FALSE;
                    ctrl->client.io_error = TRUE;
                    g_mutex_unlock(&ctrl->client.mutex);
                    rot_schedule_wrong_daemon(ctrl,
                                              ctrl->conf ? ctrl->conf->host : NULL,
                                              ctrl->conf ? ctrl->conf->port : 0);
                    ctrl->client.running = FALSE;
                    break;
                }

                gboolean rejected = FALSE;
                if (!rotctld_handshake(ctrl, ctrl->client.socket, &rejected)) {
                    sat_log_log(SAT_LOG_LEVEL_ERROR,
                                "%s: rotctld handshake failed on %s:%d",
                                __func__,
                                ctrl->conf ? ctrl->conf->host : "(null)",
                                ctrl->conf ? ctrl->conf->port : 0);
                    rot_term_log(ctrl, "gpredict:err",
                                 "rotctld handshake failed on %s:%d",
                                 ctrl->conf ? ctrl->conf->host : "(null)",
                                 ctrl->conf ? ctrl->conf->port : 0);
                    if (rejected) {
                        rot_schedule_cmd_reject(
                            ctrl,
                            _("rotctld reachable but rejects position commands; "
                              "check backend/model, limits, and axis mode"));
                    }
                    rotctld_socket_close_quiet(&ctrl->client.socket);
                    g_mutex_lock(&ctrl->client.mutex);
                    ctrl->client.daemon_ok = FALSE;
                    ctrl->client.io_error = TRUE;
                    g_mutex_unlock(&ctrl->client.mutex);
                    ctrl->client.running = FALSE;
                    break;
                }

                g_mutex_lock(&ctrl->client.mutex);
                ctrl->client.daemon_ok = TRUE;
                g_mutex_unlock(&ctrl->client.mutex);
            }
        }

        io_error = FALSE;

        /* get latest commanded position from controller, but only
         * send a new command when new_trg is set. This avoids
         * hammering the rotor with repeated small corrections and
         * reduces "hunting" around the target position.
         */
        gboolean send_cmd = FALSE;
        gboolean backoff_active = FALSE;
        gint64 now_us = g_get_monotonic_time();
        gint64 backoff_until = 0;
        gint64 backoff_log_us = 0;

        g_mutex_lock(&ctrl->client.mutex);
        azi = ctrl->client.azi_out;
        ele = ctrl->client.ele_out;
        backoff_until = ctrl->client.reject_backoff_until_us;
        backoff_log_us = ctrl->client.reject_backoff_log_us;
        if (ctrl->client.new_trg)
        {
            if (now_us < backoff_until) {
                backoff_active = TRUE;
            } else {
                send_cmd = TRUE;
                ctrl->client.new_trg = FALSE;
            }
        }
        g_mutex_unlock(&ctrl->client.mutex);

        if (backoff_active)
        {
            if (now_us - backoff_log_us > 2000000) {
                rot_term_log(ctrl, "gpredict:err",
                             "set_position backoff active; retrying in %.1f s",
                             (backoff_until - now_us) / 1e6);
                g_mutex_lock(&ctrl->client.mutex);
                ctrl->client.reject_backoff_log_us = now_us;
                g_mutex_unlock(&ctrl->client.mutex);
            }
            send_cmd = FALSE;
        }

        if (send_cmd)
        {
            gchar azs[32];
            gchar els[32];
            gboolean daemon_ok = FALSE;
            rot_format_deg_2(azs, sizeof(azs), azi);
            rot_format_deg_2(els, sizeof(els), ele);
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        "MISSION_SOPHIE: client thread sending position to rotctld cmd=(%s, %s) engaged=%d monitor=%d",
                        azs, els,
                        ctrl->engaged ? 1 : 0,
                        ctrl->monitor ? 1 : 0);
            g_printerr("MISSION_SOPHIE: client thread sending position to rotctld cmd=(%s, %s) engaged=%d monitor=%d\n",
                       azs, els,
                       ctrl->engaged ? 1 : 0,
                       ctrl->monitor ? 1 : 0);

            g_mutex_lock(&ctrl->client.mutex);
            daemon_ok = ctrl->client.daemon_ok;
            g_mutex_unlock(&ctrl->client.mutex);

            if (!daemon_ok) {
                io_error = TRUE;
                rot_term_log(ctrl, "gpredict:err",
                             "rotctld connection not verified; skipping set_position to %s:%d",
                             ctrl->conf ? ctrl->conf->host : "(null)",
                             ctrl->conf ? ctrl->conf->port : 0);
                rotctld_socket_close_quiet(&ctrl->client.socket);
                g_mutex_lock(&ctrl->client.mutex);
                ctrl->client.limits_valid = FALSE;
                ctrl->client.daemon_ok = FALSE;
                g_mutex_unlock(&ctrl->client.mutex);
                continue;
            }

            rot_set_result_t set_res = set_pos(ctrl, azi, ele);
            if (set_res == ROT_SET_IO_ERROR)
            {
                io_error = TRUE;
                sat_log_log(SAT_LOG_LEVEL_ERROR,
                            _("%s: set_pos failed"), __func__);
                rot_term_log(ctrl, "gpredict:err",
                             "%s failed: set_position", __func__);
            }
            else if (set_res == ROT_SET_REJECTED)
            {
                sat_log_log(SAT_LOG_LEVEL_WARN,
                            _("%s: set_pos rejected by rotctld"), __func__);
                rot_term_log(ctrl, "gpredict:err",
                             "%s failed: set_position rejected", __func__);
            }
            else
            {
                sat_log_log(SAT_LOG_LEVEL_DEBUG,
                            _("%s: set_pos success"), __func__);
                rot_term_log_verbose(ctrl, "gpredict:rx",
                                     "set_position accepted");
            }

            g_mutex_lock(&ctrl->client.mutex);
            if (set_res == ROT_SET_REJECTED) {
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
            }
            g_mutex_unlock(&ctrl->client.mutex);
        }
        else
        {
            gchar azs[32];
            gchar els[32];
            rot_format_deg_2(azs, sizeof(azs), azi);
            rot_format_deg_2(els, sizeof(els), ele);
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        "MISSION_SOPHIE: client thread idle – no new target (last_out=(%s, %s))",
                        azs, els);
            g_printerr("MISSION_SOPHIE: idle – no new target (last_out=(%s, %s))\n",
                       azs, els);
            rot_term_log_verbose(ctrl, "gpredict:rx",
                                 "idle: no new target (last_out=%s,%s)",
                                 azs, els);
        }

        if (io_error) {
            rotctld_socket_close_quiet(&ctrl->client.socket);
            g_mutex_lock(&ctrl->client.mutex);
            ctrl->client.limits_valid = FALSE;
            ctrl->client.daemon_ok = FALSE;
            ctrl->client.cmd_rejected = FALSE;
            ctrl->client.reject_backoff_until_us = 0;
            g_mutex_unlock(&ctrl->client.mutex);
        }

        /* Treat last commanded az/el as the "measured" position for
         * display purposes, since some rotctld backends only return
         * RPRT codes to the "p" command and do not support true
         * position read-back.
         */
        g_mutex_lock(&ctrl->client.mutex);
        if (!io_error && send_cmd && !ctrl->client.cmd_rejected) {
            ctrl->client.azi_in = ctrl->client.azi_out;
            ctrl->client.ele_in = ctrl->client.ele_out;
        }
        ctrl->client.io_error = io_error;
        g_mutex_unlock(&ctrl->client.mutex);

        /* ensure rotctl duty cycle stays below 50%, but wait at least 700 ms */
        elapsed_time = MAX(g_timer_elapsed(ctrl->client.timer, NULL), 0.7);
        g_usleep((gulong)(elapsed_time * 1e6));
    }

    sat_log_log(SAT_LOG_LEVEL_INFO,
                "MISSION_SOPHIE: rotctld_client_thread stopping");

    g_printerr("MISSION_SOPHIE: rotctld_client_thread stopping\n");

    sat_log_log(SAT_LOG_LEVEL_INFO,
                _("%s: stopping rotctld client thread"), __func__);
    g_timer_destroy(ctrl->client.timer);
    rotctld_socket_close(ctrl, &ctrl->client.socket);

#if defined(__APPLE__) || defined(__linux__) || defined(__FreeBSD__)
    if (c_locale) {
        uselocale(old_locale);
        freelocale(c_locale);
    }
#endif

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
    gchar          *buff;
    guint           h, m, s;

    /* select AOS or LOS time depending on target elevation */
    if (ctrl->target->el < 0.0)
        targettime = ctrl->target->aos;
    else
        targettime = ctrl->target->los;

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
        buff = g_strdup_printf("%02d:%02d:%02d", h, m, s);
    else
        buff = g_strdup_printf("%02d:%02d", m, s);

    gtk_label_set_text(GTK_LABEL(ctrl->SatCnt), buff);

    g_free(buff);
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

    ctrl->t = t;

    if (ctrl->target)
    {
        /* update target displays */
        buff = g_strdup_printf(FMTSTR, ctrl->target->az);
        gtk_label_set_text(GTK_LABEL(ctrl->AzSat), buff);
        g_free(buff);
        buff = g_strdup_printf(FMTSTR, ctrl->target->el);
        gtk_label_set_text(GTK_LABEL(ctrl->ElSat), buff);
        g_free(buff);

        update_count_down(ctrl, t);

        /*if the current pass is too far away */
        if ((ctrl->pass != NULL))
            if (qth_small_dist(ctrl->qth, ctrl->pass->qth_comp) > 1.0)
            {
                rot_plan_reset(&ctrl->trajectory_plan);
                free_pass(ctrl->pass);
                ctrl->pass = NULL;
                ctrl->pass = get_pass(ctrl->target, ctrl->qth, t, 3.0);
                if (ctrl->pass)
                {
                    set_flipped_pass(ctrl);
                    /* update polar plot */
                    gtk_polar_plot_set_pass(GTK_POLAR_PLOT(ctrl->plot),
                                            ctrl->pass);
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
                    ctrl->pass = get_current_pass(ctrl->target, ctrl->qth, t);
                    set_flipped_pass(ctrl);
                    gtk_polar_plot_set_pass(GTK_POLAR_PLOT(ctrl->plot),
                                            ctrl->pass);
                }
                else if ((ctrl->target->aos - ctrl->pass->aos) >
                         (ctrl->delay / secday / 1000 / 4.0))
                {
                    /* the target is expected to appear in a new pass 
                       sufficiently later after the current pass says */

                    /* converted milliseconds to gpredict time and took a 
                       fraction of it as a threshold for deciding a new pass */

                    /* if the next pass is not the one for the target */
                    rot_plan_reset(&ctrl->trajectory_plan);
                    free_pass(ctrl->pass);
                    ctrl->pass = NULL;
                    ctrl->pass = get_pass(ctrl->target, ctrl->qth, t, 3.0);
                    set_flipped_pass(ctrl);
                    /* update polar plot */
                    gtk_polar_plot_set_pass(GTK_POLAR_PLOT(ctrl->plot),
                                            ctrl->pass);
                }
            }
            else
            {
                /* inside a pass and target dropped below the 
                   horizon so look for a new pass */
                if (ctrl->target->el < 0.0)
                {
                    rot_plan_reset(&ctrl->trajectory_plan);
                    free_pass(ctrl->pass);
                    ctrl->pass = NULL;
                    ctrl->pass = get_pass(ctrl->target, ctrl->qth, t, 3.0);
                    set_flipped_pass(ctrl);
                    /* update polar plot */
                    gtk_polar_plot_set_pass(GTK_POLAR_PLOT(ctrl->plot),
                                            ctrl->pass);
                }
            }
        }
        else
        {
            /* we don't have any current pass; store the current one */
            rot_plan_reset(&ctrl->trajectory_plan);
            if (ctrl->target->el > 0.0)
                ctrl->pass = get_current_pass(ctrl->target, ctrl->qth, t);
            else
                ctrl->pass = get_pass(ctrl->target, ctrl->qth, t, 3.0);

            set_flipped_pass(ctrl);
            /* update polar plot */
            gtk_polar_plot_set_pass(GTK_POLAR_PLOT(ctrl->plot), ctrl->pass);
        }
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
            gtk_combo_box_set_active(GTK_COMBO_BOX(ctrl->SatSel), i);
            break;
        }
    }
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
    gtk_container_add(GTK_CONTAINER(frame), table);

    ctrl->AzSet = gtk_rot_knob_new(0.0, 360.0, 180.0);
    gtk_grid_attach(GTK_GRID(table), ctrl->AzSet, 0, 0, 3, 1);

    label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(label), _("Read:"));
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
    gtk_container_add(GTK_CONTAINER(frame), table);

    ctrl->ElSet = gtk_rot_knob_new(0.0, 90.0, 45.0);
    gtk_grid_attach(GTK_GRID(table), ctrl->ElSet, 0, 0, 3, 1);

    label = gtk_label_new(NULL);
    gtk_label_set_markup(GTK_LABEL(label), _("Read: "));
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

    locked = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ctrl->LockBut));
    ctrl->tracking = gtk_toggle_button_get_active(button);
    gtk_widget_set_sensitive(ctrl->MonitorCheckBox,
                             !(ctrl->tracking || locked));
    gtk_widget_set_sensitive(ctrl->AzSet, !ctrl->tracking);
    gtk_widget_set_sensitive(ctrl->ElSet, !ctrl->tracking);

    if (!ctrl->tracking) {
        rot_plan_reset(&ctrl->trajectory_plan);
        set_flipped_pass(ctrl);
        return;
    }

    if (ctrl->tracking && ctrl->target && ctrl->qth)
    {
        if (ctrl->pass != NULL)
            free_pass(ctrl->pass);

        if (ctrl->target->el > 0.0)
            ctrl->pass = get_current_pass(ctrl->target, ctrl->qth, ctrl->t);
        else
            ctrl->pass = get_pass(ctrl->target, ctrl->qth, ctrl->t, 3.0);

        rot_plan_reset(&ctrl->trajectory_plan);

        if (!rot_build_tracking_plan(ctrl)) {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        "%s: unable to build trajectory – %s",
                        __func__,
                        ctrl->trajectory_plan.reason ? ctrl->trajectory_plan.reason : "trajectory planning failed");
            rot_show_plan_error(ctrl, ctrl->trajectory_plan.reason);
            gtk_toggle_button_set_active(button, FALSE);
            ctrl->tracking = FALSE;
            set_flipped_pass(ctrl);
            return;
        }

        ctrl->flipped = (ctrl->trajectory_plan.mode == ROT_PLAN_MODE_FLIP);
        set_flipped_pass(ctrl);
        if (ctrl->plot != NULL)
            gtk_polar_plot_set_pass(GTK_POLAR_PLOT(ctrl->plot), ctrl->pass);
    }
    else if (ctrl->tracking) {
        rot_plan_reset(&ctrl->trajectory_plan);
        set_flipped_pass(ctrl);
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
    gdouble setaz = 0.0, setel = 45.0;
    gdouble cmdaz = 0.0, cmdel = 0.0;
    rot_cmd_map_t cmd_map = { 0 };
    rot_cmd_map_status_t map_status = ROT_CMD_MAP_INVALID_CONF;
    gboolean cmd_from_plan = FALSE;
    gchar *text;
    gboolean error = FALSE;
    gboolean cmd_rejected = FALSE;
    sat_t sat_working, *sat;
    GtkWidget *status_label =
        g_object_get_data(G_OBJECT(ctrl), "rot-status-label");
    gboolean plan_active = rot_plan_matches_pass(ctrl);
    gboolean use_flip = plan_active
                        ? (ctrl->trajectory_plan.mode == ROT_PLAN_MODE_FLIP)
                        : ctrl->flipped;
    ctrl->out_of_range = FALSE;

    /* parameters for path predictions */
    gdouble time_delta;
    gdouble step_size;

    if (ctrl->tracking && ctrl->pass && !plan_active) {
        rot_build_tracking_plan(ctrl);
        plan_active = rot_plan_matches_pass(ctrl);
        use_flip = plan_active
                   ? (ctrl->trajectory_plan.mode == ROT_PLAN_MODE_FLIP)
                   : ctrl->flipped;
    }

    /* If we are tracking and the target satellite is within range, set the
     * rotor position controller knob values to the target values. If the
     * target satellite is out of range set the rotor controller to 0 deg El
     * and to the Az where the target sat is expected to come up or where it
     * last went down.
     */
    if (ctrl->tracking && ctrl->target && ctrl->conf)
    {
        if (ctrl->target->el < 0.0)
        {
            if (ctrl->pass != NULL)
            {
                if (ctrl->t < ctrl->pass->aos)
                {
                    /*
                     * PRE-TRACK: Satellite is below horizon but we already know the
                     * AOS azimuth from the predicted pass. Pre-position the rotor to
                     * the AOS azimuth and hold elevation at the lowest allowed value.
                     *
                     * This avoids the rotor sitting at an arbitrary park (e.g. 0/0)
                     * and only starting to move once the satellite is already in range.
                     */
                    if (plan_active && ctrl->trajectory_plan.valid) {
                        setaz = rot_az_to_conf(ctrl->conf,
                                               ctrl->trajectory_plan.start_az_cmd);
                        setel = ctrl->trajectory_plan.start_el_cmd;
                    }
                    else {
                        setaz = ctrl->pass->aos_az;
                        setel = (ctrl->conf ? ctrl->conf->minel : 0.0);
                    }
                }
                else if (ctrl->t > ctrl->pass->los)
                {
                    /* After LOS: park at configured park position. */
                    rot_get_park_position(ctrl, &setaz, &setel);
                }
                else
                {
                    /* In/near-pass but still below horizon: keep current pre-positioning. */
                    if (plan_active && ctrl->trajectory_plan.valid) {
                        setaz = rot_az_to_conf(ctrl->conf,
                                               ctrl->trajectory_plan.start_az_cmd);
                        setel = ctrl->trajectory_plan.start_el_cmd;
                    }
                    else {
                        setaz = ctrl->pass->aos_az;
                        setel = (ctrl->conf ? ctrl->conf->minel : 0.0);
                    }
                }
            }
            else
            {
                /* No current pass information: park at configured park position. */
                rot_get_park_position(ctrl, &setaz, &setel);
            }
        }
        else
        {
            setaz = ctrl->target->az;
            setel = ctrl->target->el;
        }

        /* if this is a flipped pass and the rotor supports it */
        if (use_flip && ctrl->conf->maxel >= 180.0)
        {
            setel = 180 - setel;
            if (setaz > 180)
                setaz -= 180;
            else
                setaz += 180;

            while (setaz > ctrl->conf->maxaz)
                setaz -= 360;
            while (setaz < ctrl->conf->minaz)
                setaz += 360;
        }

        if (ctrl->conf->aztype == ROT_AZ_TYPE_180 && setaz > 180.0)
            setaz -= 360.0;

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
    }

    /* Handle I/O with rotctld client if running and not in monitor mode */
    if (ctrl->client.running && !ctrl->monitor)
    {
        if (g_mutex_trylock(&ctrl->client.mutex))
        {
            error = ctrl->client.io_error;
            cmd_rejected = ctrl->client.cmd_rejected;
            rotaz = ctrl->client.azi_in;
            rotel = ctrl->client.ele_in;
            g_mutex_unlock(&ctrl->client.mutex);

            /* ensure Azimuth angle is 0-360 degrees */
            while (rotaz < 0.0)
                rotaz += 360.0;
            while (rotaz > 360.0)
                rotaz -= 360.0;

            if (error)
            {
                gtk_label_set_text(GTK_LABEL(ctrl->AzRead), _("ERROR"));
                gtk_label_set_text(GTK_LABEL(ctrl->ElRead), _("ERROR"));
                gtk_polar_plot_set_rotor_pos(GTK_POLAR_PLOT(ctrl->plot),
                                             -10.0, -10.0);
            }
            else
            {
                /* update display widgets */
                text = g_strdup_printf("%.2f\302\260", rotaz);
                gtk_label_set_text(GTK_LABEL(ctrl->AzRead), text);
                g_free(text);
                text = g_strdup_printf("%.2f\302\260", rotel);
                gtk_label_set_text(GTK_LABEL(ctrl->ElRead), text);
                g_free(text);

                if (ctrl->conf->aztype == ROT_AZ_TYPE_180 && rotaz < 0.0)
                {
                    gtk_polar_plot_set_rotor_pos(GTK_POLAR_PLOT(ctrl->plot),
                                                 rotaz + 360.0, rotel);
                }
                else
                {
                    gtk_polar_plot_set_rotor_pos(GTK_POLAR_PLOT(ctrl->plot),
                                                 rotaz, rotel);
                }
            }
        }

        /* if tolerance exceeded between desired (setaz/setel)
         * and current (rotaz/rotel) position
         */
        gdouble raw_cmd_az = 0.0;
        gdouble raw_cmd_el = 0.0;
        cmd_from_plan = rot_get_command(ctrl, plan_active, setaz, setel,
                                        ctrl->t, &raw_cmd_az, &raw_cmd_el);

        map_status = rot_cmd_map(ctrl->conf, raw_cmd_az, raw_cmd_el, &cmd_map);
        cmdaz = cmd_map.send_az;
        cmdel = cmd_map.send_el;

        if (map_status != ROT_CMD_MAP_OK) {
            ctrl->out_of_range = TRUE;
            rot_log_out_of_range(ctrl, raw_cmd_az, raw_cmd_el, &cmd_map);
        }

        if (map_status == ROT_CMD_MAP_OK &&
            (fabs(cmdaz - rotaz) > ctrl->threshold ||
             fabs(cmdel - rotel) > ctrl->threshold))
        {
            /* If tracking is enabled, refine the target to "lead" the pass
             * a bit into the future, like the original code did.
             * Manual mode skips this and just uses the knob values.
             */
            if (ctrl->tracking && ctrl->target && ctrl->conf &&
                !cmd_from_plan)
            {
                if (ctrl->target->el > 0.0)
                {
                    /* use a working copy so data does not get corrupted */
                    sat = memcpy(&sat_working, ctrl->target, sizeof(sat_t));

                    /* compute az/el in the future that is past end of pass
                     * or exceeds tolerance
                     */
                    if (ctrl->pass)
                        time_delta = ctrl->pass->los - ctrl->t;
                    else
                        time_delta = 1.0 / 72.0; /* ~20 minutes */

                    /* have a minimum time delta */
                    step_size = time_delta / 2.0;
                    if (step_size < ctrl->delay / 1000.0 / secday)
                        step_size = ctrl->delay / 1000.0 / secday;

                    /* search for a time when satellite is above horizon
                     * and at the edge of tolerance
                     */
                    while (step_size > (ctrl->delay / 1000.0 / 4.0 / secday))
                    {
                        predict_calc(sat, ctrl->qth, ctrl->t + time_delta);

                        /* update sat->az and sat->el to account for flips
                         * and az range
                         */
                        if (use_flip && ctrl->conf->maxel >= 180.0)
                        {
                            sat->el = 180.0 - sat->el;
                            if (sat->az > 180.0)
                                sat->az -= 180.0;
                            else
                                sat->az += 180.0;
                        }

                        if (ctrl->conf->aztype == ROT_AZ_TYPE_180 &&
                            sat->az > 180.0)
                            sat->az -= 360.0;

                        if (sat->el < 0.0 || sat->el > 180.0 ||
                            fabs(setaz - sat->az) > ctrl->threshold ||
                            fabs(setel - sat->el) > ctrl->threshold)
                        {
                            time_delta -= step_size;
                        }
                        else
                        {
                            time_delta += step_size;
                        }

                        step_size /= 2.0;
                    }

                    setel = sat->el;
                    setaz = sat->az;
                }
            }

            /* Now drive the rotor towards setaz/setel for BOTH:
             *  - tracking mode (satellite following)
             *  - manual mode (knob control)
             *
             * We only post a new target to the client thread if it
             * really changed compared to the last command, to avoid
             * "rotor talking alone".
             */

            /* Recompute command after any lead adjustments. */
            cmd_from_plan = rot_get_command(ctrl, plan_active, setaz, setel,
                                            ctrl->t, &raw_cmd_az, &raw_cmd_el);
            map_status = rot_cmd_map(ctrl->conf, raw_cmd_az, raw_cmd_el, &cmd_map);
            cmdaz = cmd_map.send_az;
            cmdel = cmd_map.send_el;
            if (map_status != ROT_CMD_MAP_OK) {
                ctrl->out_of_range = TRUE;
                rot_log_out_of_range(ctrl, raw_cmd_az, raw_cmd_el, &cmd_map);
            }

            /* keep knobs in sync with the commanded position (logical) */
            gtk_rot_knob_set_value(GTK_ROT_KNOB(ctrl->AzSet), setaz);
            gtk_rot_knob_set_value(GTK_ROT_KNOB(ctrl->ElSet), setel);

            if (map_status == ROT_CMD_MAP_OK &&
                g_mutex_trylock(&ctrl->client.mutex))
            {
                if (fabs(cmdaz - ctrl->client.azi_out) > ctrl->threshold ||
                    fabs(cmdel - ctrl->client.ele_out) > ctrl->threshold)
                {
                    ctrl->client.azi_out = cmdaz;
                    ctrl->client.ele_out = cmdel;
                    ctrl->client.new_trg = TRUE;
                }
                g_mutex_unlock(&ctrl->client.mutex);
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

            if (status_label)
                gtk_label_set_text(GTK_LABEL(status_label), _("LINK DOWN"));
        }

        /* update status label if present, based on data and feedback mode */
        if (status_label)
        {
            const gchar *status_text = NULL;
            gboolean has_error = error || (ctrl->errcnt > 0);

            if (ctrl->out_of_range)
                status_text = _("OUT OF RANGE");
            else if (cmd_rejected)
                status_text = _("CMD REJECTED");
            else if (has_error)
                status_text = _("LINK DOWN");
            else if (ctrl->tracking && plan_active &&
                     ctrl->trajectory_plan.valid &&
                     ctrl->trajectory_plan.status != ROT_PLAN_STATUS_FULL_TRACK)
                status_text = _("PARTIAL");
            else if (fabs(cmdaz - rotaz) > ctrl->threshold ||
                     fabs(cmdel - rotel) > ctrl->threshold)
                status_text = _("MOVING");
            else
                status_text = _("ON TARGET");

            gtk_label_set_text(GTK_LABEL(status_label), status_text);

            char cmdaz_str[32];
            char cmdel_str[32];
            char rotaz_str[32];
            char rotel_str[32];

            rot_format_deg_2(cmdaz_str, sizeof(cmdaz_str), cmdaz);
            rot_format_deg_2(cmdel_str, sizeof(cmdel_str), cmdel);
            rot_format_deg_2(rotaz_str, sizeof(rotaz_str), rotaz);
            rot_format_deg_2(rotel_str, sizeof(rotel_str), rotel);

            sat_log_log(SAT_LOG_LEVEL_INFO,
                        "MISSION_SOPHIE: status=%s set=(%s, %s) rot=(%s, %s) error=%d",
                        status_text,
                        cmdaz_str, cmdel_str,
                        rotaz_str, rotel_str,
                        error ? 1 : 0);
            g_printerr("MISSION_SOPHIE: status=%s set=(%s, %s) rot=(%s, %s) error=%d\n",
                       status_text,
                       cmdaz_str, cmdel_str,
                       rotaz_str, rotel_str,
                       error ? 1 : 0);
        }
    }
    else
    {
        /* client not running or monitor mode: ensure rotor pos is not visible */
        gtk_polar_plot_set_rotor_pos(GTK_POLAR_PLOT(ctrl->plot), -10.0, -10.0);

        if (status_label)
            gtk_label_set_text(GTK_LABEL(status_label), _("DISENGAGED"));
    }

    /* update target object on polar plot */
    if (ctrl->target != NULL)
    {
        gtk_polar_plot_set_target_pos(GTK_POLAR_PLOT(ctrl->plot),
                                      ctrl->target->az, ctrl->target->el);
    }

    /* update controller circle on polar plot */
    if (ctrl->conf != NULL)
    {
        gdouble dispaz = gtk_rot_knob_get_value(GTK_ROT_KNOB(ctrl->AzSet));
        gdouble dispel = gtk_rot_knob_get_value(GTK_ROT_KNOB(ctrl->ElSet));

        if (ctrl->conf->aztype == ROT_AZ_TYPE_180 && dispaz < 0.0)
            dispaz += 360.0;

        gtk_polar_plot_set_ctrl_pos(GTK_POLAR_PLOT(ctrl->plot), dispaz, dispel);
        gtk_widget_queue_draw(ctrl->plot);
    }

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

    ctrl->delay = (guint) gtk_spin_button_get_value(spin);
    if (ctrl->conf)
        ctrl->conf->cycle = ctrl->delay;

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

    ctrl->threshold = gtk_spin_button_get_value(spin);
    if (ctrl->conf)
        ctrl->conf->threshold = ctrl->threshold;
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

    /* free previous configuration */
    if (ctrl->conf != NULL)
    {
        g_free(ctrl->conf->name);
        g_free(ctrl->conf->host);
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
            g_free(ctrl->conf);
            ctrl->conf = NULL;
            return;
        }

        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("Loaded new rotator configuration %s"),
                    ctrl->conf->name);

        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ctrl->cycle_spin),
                                  ctrl->conf->cycle);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ctrl->thld_spin),
                                  ctrl->conf->threshold);

        /* update new ranges of the Az and El controller widgets */
        gtk_rot_knob_set_range(GTK_ROT_KNOB(ctrl->AzSet), ctrl->conf->minaz,
                               ctrl->conf->maxaz);
        gtk_rot_knob_set_range(GTK_ROT_KNOB(ctrl->ElSet), ctrl->conf->minel,
                               ctrl->conf->maxel);

        ctrl->use_offset = ctrl->conf->use_offset;
        ctrl->az_offset_deg = ctrl->conf->az_offset;
        ctrl->el_offset_deg = ctrl->conf->el_offset;

        if (ctrl->offset_check) {
            g_signal_handlers_block_by_func(ctrl->offset_check,
                                            G_CALLBACK(offset_toggle_cb), ctrl);
            gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->offset_check),
                                         ctrl->use_offset);
            g_signal_handlers_unblock_by_func(ctrl->offset_check,
                                              G_CALLBACK(offset_toggle_cb), ctrl);
        }
        if (ctrl->az_offset_spin) {
            g_signal_handlers_block_by_func(ctrl->az_offset_spin,
                                            G_CALLBACK(az_offset_changed_cb), ctrl);
            gtk_spin_button_set_value(GTK_SPIN_BUTTON(ctrl->az_offset_spin),
                                      ctrl->az_offset_deg);
            g_signal_handlers_unblock_by_func(ctrl->az_offset_spin,
                                              G_CALLBACK(az_offset_changed_cb), ctrl);
        }
        if (ctrl->el_offset_spin) {
            g_signal_handlers_block_by_func(ctrl->el_offset_spin,
                                            G_CALLBACK(el_offset_changed_cb), ctrl);
            gtk_spin_button_set_value(GTK_SPIN_BUTTON(ctrl->el_offset_spin),
                                      ctrl->el_offset_deg);
            g_signal_handlers_unblock_by_func(ctrl->el_offset_spin,
                                              G_CALLBACK(el_offset_changed_cb), ctrl);
        }

        rot_plan_reset(&ctrl->trajectory_plan);
        /* Update flipped when changing rotor if there is a plot */
        set_flipped_pass(ctrl);
    }
    else
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%d: Failed to load rotator configuration %s"),
                    __FILE__, __LINE__, ctrl->conf->name);

        g_free(ctrl->conf->name);
        if (ctrl->conf->host)
            g_free(ctrl->conf->host);
        g_free(ctrl->conf);
        ctrl->conf = NULL;
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

    ctrl->monitor = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(button));
    gtk_widget_set_sensitive(ctrl->AzSet, !ctrl->monitor);
    gtk_widget_set_sensitive(ctrl->ElSet, !ctrl->monitor);
    gtk_widget_set_sensitive(ctrl->track, !ctrl->monitor);
}


/* Try to find a likely serial device for a locally attached rotor.
 *
 * This is intentionally very simple and "taped together" for now:
 *  - On macOS we scan /dev for tty.usbserial*, ttyUSB*, tty.usbmodem*, etc.
 *  - We simply return the first match.
 *
 * The returned string must be freed by the caller.
 */
static gchar *rotctld_find_serial_device(void)
{
    gchar *device = NULL;

#if defined(__APPLE__)
    /* macOS: scan /dev for common USB serial device names. */
    GDir *dir = NULL;
    GError *error = NULL;
    const gchar *name;

    dir = g_dir_open("/dev", 0, &error);
    if (!dir) {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: failed to open /dev: %s"),
                    __func__,
                    error ? error->message : "unknown error");
        if (error)
            g_clear_error(&error);
        return NULL;
    }

    /* Choose the "best" candidate rather than the first one returned by
     * readdir(), which is effectively random. Prefer cu.* over tty.*.
     */
    int best_prio = 999;

    while ((name = g_dir_read_name(dir)) != NULL) {
        int prio = 0;

        if (g_str_has_prefix(name, "cu.usbserial"))
            prio = 1;
        else if (g_str_has_prefix(name, "cu.usbmodem"))
            prio = 2;
        else if (g_str_has_prefix(name, "tty.usbserial"))
            prio = 3;
        else if (g_str_has_prefix(name, "ttyUSB"))
            prio = 4;
        else if (g_str_has_prefix(name, "tty.usbmodem"))
            prio = 5;
        else
            continue;

        if (prio < best_prio) {
            g_free(device);
            device = g_strdup_printf("/dev/%s", name);
            best_prio = prio;
        }
    }

    g_dir_close(dir);

    if (device) {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: auto-selected serial device %s"),
                    __func__, device);
    } else {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: no candidate serial device found in /dev"),
                    __func__);
    }

#elif defined(G_OS_WIN32)
    /*
     * Windows: probe COM1..COM32 using the Win32 API.
     * We try to open each COM port with CreateFile; the first one
     * that opens successfully is assumed to be our rotor.
     *
     * Note: we return "COMx" (without the "\\\\.\\" prefix) because
     * Hamlib/rotctld expects the logical port name, not the Win32 path.
     */
    int i;

    for (i = 1; i <= 32; i++) {
        gchar *probe_path = g_strdup_printf("\\\\.\\\\COM%d", i);
        HANDLE h = CreateFileA(probe_path,
                               GENERIC_READ | GENERIC_WRITE,
                               0,
                               NULL,
                               OPEN_EXISTING,
                               0,
                               NULL);
        g_free(probe_path);

        if (h != INVALID_HANDLE_VALUE) {
            /* Found a usable COM port. */
            CloseHandle(h);
            device = g_strdup_printf("COM%d", i);
            break;
        }
    }

    if (device) {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: auto-selected serial device %s"),
                    __func__, device);
    } else {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: no candidate COM port found (COM1..COM32)"),
                    __func__);
    }

#else
    /* On other platforms we currently don't try to guess a device. */
    sat_log_log(SAT_LOG_LEVEL_ERROR,
                _("%s: auto-detection not implemented on this platform"),
                __func__);
#endif

    return device;
}

/* Try to locate the rotctld binary in PATH and in a few common
 * installation locations (especially for macOS/Homebrew).
 *
 * Returns a newly allocated string with the full path, or NULL
 * if nothing usable was found. Caller must g_free() the result.
 */
static gchar *rotctld_find_binary(void)
{
    gchar *prog = NULL;

    /* First, try whatever PATH Gpredict has. */
    prog = g_find_program_in_path("rotctld");
    if (prog != NULL)
        return prog;

#if defined(__APPLE__)
    /* Typical Homebrew locations on macOS. */
    const gchar *candidates[] = {
        "/opt/homebrew/bin/rotctld",  /* Apple Silicon default */
        "/usr/local/bin/rotctld",     /* Intel / older Homebrew */
        NULL
    };

    for (int i = 0; candidates[i] != NULL; i++) {
        if (g_file_test(candidates[i], G_FILE_TEST_IS_EXECUTABLE)) {
            return g_strdup(candidates[i]);
        }
    }
#endif

#if defined(G_OS_WIN32)
    /* Windows: rely on PATH for now; users typically install Hamlib
     * into a directory that is added to PATH. If needed, explicit
     * probing of common locations could be added here later.
     */
#endif

    sat_log_log(SAT_LOG_LEVEL_ERROR,
                _("%s: could not locate rotctld binary in PATH or common locations"),
                __func__);
    return NULL;
}

typedef struct {
    GtkRotCtrl        *ctrl;
    GDataInputStream  *stream;
    const gchar       *prefix;
} RotctldLogReader;

static gpointer rotctld_log_thread(gpointer data)
{
    RotctldLogReader *reader = data;
    GError *error = NULL;
    gchar *line = NULL;
    gsize length = 0;

    if (reader == NULL)
        return NULL;

    while ((line = g_data_input_stream_read_line(reader->stream, &length,
                                                 NULL, &error)) != NULL)
    {
        if (length > 0)
            rot_term_log(reader->ctrl, reader->prefix, "%s", line);
        g_free(line);
    }

    if (error != NULL)
    {
        rot_term_log(reader->ctrl, "gpredict:err",
                     "rotctld log read failed: %s", error->message);
        g_clear_error(&error);
    }

    g_object_unref(reader->stream);
    g_free(reader);
    return NULL;
}

static void rotctld_start_log_threads(GtkRotCtrl *ctrl)
{
    if (ctrl == NULL || ctrl->rotctld_proc == NULL)
        return;

    if (g_subprocess_get_stdout_pipe(ctrl->rotctld_proc) != NULL)
    {
        RotctldLogReader *reader = g_new0(RotctldLogReader, 1);
        reader->ctrl = ctrl;
        reader->prefix = "rotctld:out";
        reader->stream = g_data_input_stream_new(
            g_subprocess_get_stdout_pipe(ctrl->rotctld_proc));
        g_data_input_stream_set_newline_type(reader->stream,
                                             G_DATA_STREAM_NEWLINE_TYPE_ANY);
        ctrl->rotctld_out_thread =
            g_thread_new("rotctld-out", rotctld_log_thread, reader);
    }

    if (g_subprocess_get_stderr_pipe(ctrl->rotctld_proc) != NULL)
    {
        RotctldLogReader *reader = g_new0(RotctldLogReader, 1);
        reader->ctrl = ctrl;
        reader->prefix = "rotctld:err";
        reader->stream = g_data_input_stream_new(
            g_subprocess_get_stderr_pipe(ctrl->rotctld_proc));
        g_data_input_stream_set_newline_type(reader->stream,
                                             G_DATA_STREAM_NEWLINE_TYPE_ANY);
        ctrl->rotctld_err_thread =
            g_thread_new("rotctld-err", rotctld_log_thread, reader);
    }
}

static void rotctld_process_stop(GtkRotCtrl *ctrl)
{
    if (ctrl == NULL || ctrl->rotctld_proc == NULL)
        return;

    if (!g_subprocess_get_if_exited(ctrl->rotctld_proc))
        g_subprocess_force_exit(ctrl->rotctld_proc);

    g_subprocess_wait(ctrl->rotctld_proc, NULL, NULL);

    if (ctrl->rotctld_out_thread)
    {
        g_thread_join(ctrl->rotctld_out_thread);
        ctrl->rotctld_out_thread = NULL;
    }

    if (ctrl->rotctld_err_thread)
    {
        g_thread_join(ctrl->rotctld_err_thread);
        ctrl->rotctld_err_thread = NULL;
    }

    g_clear_object(&ctrl->rotctld_proc);
    ctrl->rotctld_spawned = FALSE;
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

static gchar **rotctld_build_autostart_argv(GtkRotCtrl *ctrl)
{
    GPtrArray *argv = NULL;
    gchar *device = NULL;
    gchar *rotctld_path = NULL;
    gint model = 603;
    gint baud = 9600;
    gint port = 4533;
    const gchar *env_device = NULL;
    const gchar *env_model = NULL;
    const gchar *env_baud = NULL;

    if (ctrl == NULL || ctrl->conf == NULL)
        return NULL;

    env_device = g_getenv("GPREDICT_ROT_SERIAL");
    if (env_device != NULL && *env_device != '\0')
    {
        device = g_strdup(env_device);
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: using serial device from GPREDICT_ROT_SERIAL: %s"),
                    __func__, device);
    }
    else
    {
        device = rotctld_find_serial_device();
    }

    if (!device)
        return NULL;

    rotctld_path = rotctld_find_binary();
    if (!rotctld_path)
    {
        g_free(device);
        return NULL;
    }

    env_model = g_getenv("GPREDICT_ROT_MODEL");
    env_baud = g_getenv("GPREDICT_ROT_BAUD");

    if (env_model != NULL && *env_model != '\0')
    {
        glong tmp = g_ascii_strtoll(env_model, NULL, 10);
        if (tmp > 0)
            model = (gint) tmp;
    }

    if (env_baud != NULL && *env_baud != '\0')
    {
        glong tmp = g_ascii_strtoll(env_baud, NULL, 10);
        if (tmp > 0)
            baud = (gint) tmp;
    }

    if (ctrl->conf && ctrl->conf->port > 0)
        port = ctrl->conf->port;

    argv = g_ptr_array_new_with_free_func(g_free);
    g_ptr_array_add(argv, g_strdup(rotctld_path));
    g_ptr_array_add(argv, g_strdup("-m"));
    g_ptr_array_add(argv, g_strdup_printf("%d", model));
    g_ptr_array_add(argv, g_strdup("-r"));
    g_ptr_array_add(argv, g_strdup(device));
    g_ptr_array_add(argv, g_strdup("-s"));
    g_ptr_array_add(argv, g_strdup_printf("%d", baud));
    g_ptr_array_add(argv, g_strdup("-T"));
    if (ctrl->conf && ctrl->conf->host && ctrl->conf->host[0] != '\0')
        g_ptr_array_add(argv, g_strdup(ctrl->conf->host));
    else
        g_ptr_array_add(argv, g_strdup("127.0.0.1"));
    g_ptr_array_add(argv, g_strdup("-t"));
    g_ptr_array_add(argv, g_strdup_printf("%d", port));
    g_ptr_array_add(argv, g_strdup(ctrl->verbose_logging ? "-vvvv" : "-v"));
    g_ptr_array_add(argv, NULL);

    sat_log_log(SAT_LOG_LEVEL_INFO,
                _("%s: prepared rotctld auto-start (model=%d baud=%d device=%s port=%d)"),
                __func__, model, baud, device, port);

    g_free(rotctld_path);
    g_free(device);

    return (gchar **) g_ptr_array_free(argv, FALSE);
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
    return argv;
}

static gchar *rotctld_argv_to_string(gchar **argv)
{
    if (argv == NULL)
        return NULL;

    return g_strjoinv(" ", argv);
}

static gboolean rotctld_spawn_process(GtkRotCtrl *ctrl, gchar **argv)
{
    GSubprocessLauncher *launcher = NULL;
    GError *error = NULL;
    gchar *cmdline = NULL;

    if (ctrl == NULL || argv == NULL)
        return FALSE;

    if (ctrl->rotctld_proc != NULL)
        rotctld_process_stop(ctrl);

    cmdline = rotctld_argv_to_string(argv);
    rot_term_log_verbose(ctrl, "gpredict:tx",
                         "spawn rotctld: %s", cmdline ? cmdline : "(null)");
    g_free(cmdline);

    launcher = g_subprocess_launcher_new(G_SUBPROCESS_FLAGS_STDIN_DEV_NULL |
                                         G_SUBPROCESS_FLAGS_STDOUT_PIPE |
                                         G_SUBPROCESS_FLAGS_STDERR_PIPE);
    ctrl->rotctld_proc =
        g_subprocess_launcher_spawnv(launcher,
                                     (const gchar * const *) argv,
                                     &error);
    g_object_unref(launcher);

    if (ctrl->rotctld_proc == NULL)
    {
        rot_term_log(ctrl, "gpredict:err",
                     "Failed to start rotctld: %s",
                     error ? error->message : "unknown error");
        g_clear_error(&error);
        return FALSE;
    }

    ctrl->rotctld_spawned = TRUE;
    rotctld_start_log_threads(ctrl);
    rot_term_log_verbose(ctrl, "gpredict:rx",
                         "rotctld started pid=%s",
                         g_subprocess_get_identifier(ctrl->rotctld_proc));

    return TRUE;
}

/**
 * Ensure rotctld is running.
 *
 * This helper will first try to connect to the configured rotctld
 * endpoint. If the connection succeeds, it assumes rotctld is already
 * running and immediately closes the temporary socket.
 *
 * If the connection fails, it will:
 *   1) Try to start rotctld using the GPREDICT_ROTCTLD_CMD environment
 *      variable, if it is set.
 *   2) If the env var is not set, try to auto-detect a local serial
 *      device and build a "best effort" rotctld command line for a
 *      GS-232B rotor on macOS.
 */
/* Probe an existing rotctld instance to see if it behaves like a real
 * rotator daemon (i.e., returns a recognizable dump_state response).
 *
 * This is used to avoid "ghost" connections to unrelated services or
 * misconfigured rotctld instances that only ever reply with RPRT codes.
 */
static rot_daemon_type_t
rotctld_probe_endpoint(GtkRotCtrl *ctrl, gboolean *connected)
{
    gint sock;
    gchar reply[4096];
    rot_daemon_type_t daemon = ROT_DAEMON_UNKNOWN;

    if (connected)
        *connected = FALSE;

    if (ctrl == NULL || ctrl->conf == NULL)
        return ROT_DAEMON_UNKNOWN;

    sock = rotctld_socket_open(ctrl->conf->host, ctrl->conf->port);
    if (sock == -1)
        return ROT_DAEMON_UNKNOWN;

    if (connected)
        *connected = TRUE;

    daemon = rotctld_dump_state(ctrl, sock, reply, sizeof(reply));

#ifndef WIN32
    shutdown(sock, SHUT_RDWR);
    close(sock);
#else
    shutdown(sock, SD_BOTH);
    closesocket(sock);
#endif

    if (daemon == ROT_DAEMON_ROTCTLD)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: rotctld at %s:%d returned a valid dump_state"),
                    __func__, ctrl->conf->host, ctrl->conf->port);
    }
    else
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: unexpected daemon at %s:%d; dump_state did not look like rotctld"),
                    __func__, ctrl->conf->host, ctrl->conf->port);
    }

    return daemon;
}

static gboolean rot_host_is_local(const gchar *host)
{
    if (host == NULL)
        return FALSE;

    if (g_ascii_strcasecmp(host, "localhost") == 0)
        return TRUE;

    if (g_strcmp0(host, "127.0.0.1") == 0)
        return TRUE;

    return FALSE;
}

static gboolean rotctld_ensure_running(GtkRotCtrl *ctrl,
                                       gboolean *error_reported)
{
    if (error_reported)
        *error_reported = FALSE;

    if (ctrl == NULL || ctrl->conf == NULL)
        return FALSE;

    if (ctrl->rotctld_proc != NULL &&
        g_subprocess_get_if_exited(ctrl->rotctld_proc))
        rotctld_process_stop(ctrl);

    /* Step 1: probe whether a usable rotctld is already running */
    {
        gboolean connected = FALSE;
        rot_daemon_type_t daemon = rotctld_probe_endpoint(ctrl, &connected);

        if (connected)
        {
            if (daemon == ROT_DAEMON_ROTCTLD)
                return TRUE;

            rot_term_log(ctrl, "gpredict:err",
                         "connected to non-rotctld server on %s:%d",
                         ctrl->conf ? ctrl->conf->host : "(null)",
                         ctrl->conf ? ctrl->conf->port : 0);
            rot_schedule_wrong_daemon(ctrl,
                                      ctrl->conf ? ctrl->conf->host : NULL,
                                      ctrl->conf ? ctrl->conf->port : 0);
            if (error_reported)
                *error_reported = TRUE;
            return FALSE;
        }
    }

    if (!rot_host_is_local(ctrl->conf->host))
    {
        rot_term_log(ctrl, "gpredict:err",
                     "auto-start only supported for 127.0.0.1 (host=%s)",
                     ctrl->conf->host ? ctrl->conf->host : "(missing)");
        return FALSE;
    }

    /* Step 2: build a command to start rotctld. Prefer the environment
     * variable if present, otherwise fall back to our auto-detection
     * logic for Matteo's GS-232B setup.
     */
    const gchar *env_cmd = g_getenv("GPREDICT_ROTCTLD_CMD");
    gchar **argv = NULL;

    if (env_cmd != NULL && *env_cmd != '\0')
        argv = rotctld_build_argv_from_command(ctrl, env_cmd);
    else
        argv = rotctld_build_autostart_argv(ctrl);

    if (argv == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: rotctld is not running and no usable command "
                      "line could be constructed."),
                    __func__);
        return FALSE;
    }

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

    /* Give rotctld a short time to come up, then re-probe with backoff. */
    {
        gboolean connected = FALSE;
        rot_daemon_type_t daemon = ROT_DAEMON_UNKNOWN;
        gboolean ok = FALSE;
        gint backoff_ms = 250;

        rot_term_log_verbose(ctrl, "gpredict:rx",
                             "waiting for rotctld to become reachable");

        for (gint attempt = 0; attempt < 5; attempt++)
        {
            if (attempt > 0)
                g_usleep((gulong) backoff_ms * 1000);

            daemon = rotctld_probe_endpoint(ctrl, &connected);
            if (connected)
            {
                if (daemon == ROT_DAEMON_ROTCTLD)
                {
                    ok = TRUE;
                    break;
                }

                rot_term_log(ctrl, "gpredict:err",
                             "connected to non-rotctld server on %s:%d",
                             ctrl->conf ? ctrl->conf->host : "(null)",
                             ctrl->conf ? ctrl->conf->port : 0);
                rot_schedule_wrong_daemon(ctrl,
                                          ctrl->conf ? ctrl->conf->host : NULL,
                                          ctrl->conf ? ctrl->conf->port : 0);
                if (ctrl->rotctld_spawned)
                    rotctld_process_stop(ctrl);
                if (error_reported)
                    *error_reported = TRUE;
                return FALSE;
            }

            backoff_ms = MIN(backoff_ms * 2, 2000);
        }

        if (!ok)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: rotctld did not become reachable at %s:%d"),
                        __func__, ctrl->conf->host, ctrl->conf->port);
            rot_term_log(ctrl, "gpredict:err",
                         "rotctld not reachable at %s:%d",
                         ctrl->conf->host, ctrl->conf->port);
            if (ctrl->rotctld_spawned)
                rotctld_process_stop(ctrl);
            return FALSE;
        }
    }

    sat_log_log(SAT_LOG_LEVEL_INFO,
                _("%s: rotctld is now reachable at %s:%d"),
                __func__, ctrl->conf->host, ctrl->conf->port);
    rot_term_log_verbose(ctrl, "gpredict:rx",
                         "rotctld reachable at %s:%d",
                         ctrl->conf->host, ctrl->conf->port);

    return TRUE;
}

typedef struct {
    GtkRotCtrl *ctrl;
    gchar      *host;
    gint        port;
} RotWrongDaemonInfo;

static gboolean rot_wrong_daemon_idle(gpointer data)
{
    RotWrongDaemonInfo *info = data;
    GtkWidget *toplevel;
    GtkWindow *parent = NULL;
    GtkWidget *dialog;
    GtkWidget *status_label;
    gchar *body;

    if (info == NULL)
        return G_SOURCE_REMOVE;

    toplevel = gtk_widget_get_toplevel(GTK_WIDGET(info->ctrl));
    if (GTK_IS_WINDOW(toplevel))
        parent = GTK_WINDOW(toplevel);

    body = g_strdup_printf(
        _("Rotor control must connect to rotctld (not rigctld).\n\nHost: %s\nPort: %d"),
        info->host ? info->host : _("(missing)"),
        info->port);

    dialog = gtk_message_dialog_new(parent,
                                    GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
                                    GTK_MESSAGE_ERROR,
                                    GTK_BUTTONS_OK,
                                    "%s", body);
    gtk_window_set_title(GTK_WINDOW(dialog), _("Rotor error"));
    gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_destroy(dialog);
    g_free(body);

    if (info->ctrl && info->ctrl->LockBut)
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(info->ctrl->LockBut), FALSE);

    if (info->ctrl)
    {
        status_label =
            g_object_get_data(G_OBJECT(info->ctrl), "rot-status-label");
        if (status_label)
            gtk_label_set_text(GTK_LABEL(status_label), _("ERROR: rotctld"));
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
    GtkWidget *toplevel = gtk_widget_get_toplevel(GTK_WIDGET(ctrl));
    GtkWindow *parent = NULL;
    const gchar *msg = reason ? reason
                              : _("rotctld reachable but rejects position commands; "
                                  "check backend/model, limits, and axis mode");

    if (GTK_IS_WINDOW(toplevel))
        parent = GTK_WINDOW(toplevel);

    GtkWidget *dialog = gtk_message_dialog_new(
        parent,
        GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
        GTK_MESSAGE_ERROR,
        GTK_BUTTONS_OK,
        "%s", msg);
    gtk_window_set_title(GTK_WINDOW(dialog), _("Rotor command rejected"));
    gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_destroy(dialog);
}

static gboolean rot_cmd_reject_idle(gpointer data)
{
    RotCmdRejectInfo *info = data;
    GtkWidget *status_label;

    if (info == NULL)
        return G_SOURCE_REMOVE;

    if (info->ctrl)
        rot_show_cmd_reject_error(info->ctrl, info->reason);

    if (info->ctrl && info->ctrl->LockBut)
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(info->ctrl->LockBut), FALSE);

    status_label = g_object_get_data(G_OBJECT(info->ctrl), "rot-status-label");
    if (status_label)
        gtk_label_set_text(GTK_LABEL(status_label), _("ERROR: rotctld"));

    g_free(info->reason);
    g_free(info);
    return G_SOURCE_REMOVE;
}

static void rot_schedule_cmd_reject(GtkRotCtrl *ctrl, const gchar *reason)
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
    GtkWidget *toplevel = gtk_widget_get_toplevel(GTK_WIDGET(ctrl));
    GtkWindow *parent = NULL;
    if (GTK_IS_WINDOW(toplevel))
        parent = GTK_WINDOW(toplevel);
    GtkWidget *dialog = gtk_message_dialog_new(
        parent,
        GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
        GTK_MESSAGE_ERROR,
        GTK_BUTTONS_OK,
        "%s", _("Unable to find a rotor!"));
    gtk_window_set_title(GTK_WINDOW(dialog), _("Rotor error"));
    gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_destroy(dialog);
}

static void rot_show_conf_error(GtkRotCtrl *ctrl, const gchar *reason)
{
    GtkWidget *toplevel = gtk_widget_get_toplevel(GTK_WIDGET(ctrl));
    GtkWindow *parent = NULL;
    const gchar *msg = reason ? reason : _("Invalid rotor configuration.");

    if (GTK_IS_WINDOW(toplevel))
        parent = GTK_WINDOW(toplevel);

    GtkWidget *dialog = gtk_message_dialog_new(
        parent,
        GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
        GTK_MESSAGE_ERROR,
        GTK_BUTTONS_OK,
        "%s", msg);
    gtk_window_set_title(GTK_WINDOW(dialog), _("Rotor configuration error"));
    gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_destroy(dialog);
}

static void rot_show_plan_error(GtkRotCtrl *ctrl, const gchar *reason)
{
    const gchar *msg = reason ? reason
                              : _("No valid rotor trajectory for this pass.");
    GtkWidget *toplevel = gtk_widget_get_toplevel(GTK_WIDGET(ctrl));
    GtkWindow *parent = NULL;

    if (GTK_IS_WINDOW(toplevel))
        parent = GTK_WINDOW(toplevel);

    GtkWidget *dialog = gtk_message_dialog_new(
        parent,
        GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
        GTK_MESSAGE_ERROR,
        GTK_BUTTONS_OK,
        "%s", msg);
    gtk_window_set_title(GTK_WINDOW(dialog), _("Rotor trajectory blocked"));
    gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_destroy(dialog);
}

static void rot_logs_toggle_cb(GtkToggleButton *button, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);

    if (ctrl == NULL || ctrl->term_view == NULL)
        return;

    gp_term_view_set_visible(ctrl->term_view,
                             gtk_toggle_button_get_active(button));
    rotctrl_force_toplevel_resize(ctrl);
}

static void rotctrl_force_toplevel_resize(GtkRotCtrl *ctrl)
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

static void rot_verbose_cb(GtkToggleButton *button, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);

    if (ctrl == NULL)
        return;

    ctrl->verbose_logging = gtk_toggle_button_get_active(button);
    rot_term_log(ctrl, "gpredict:rx",
                 "verbose logging %s",
                 ctrl->verbose_logging ? "enabled" : "disabled");
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
    GtkWidget      *status_label =
        g_object_get_data(G_OBJECT(ctrl), "rot-status-label");

    if (!gtk_toggle_button_get_active(button))
    {
        ctrl->engaged = FALSE;
        gtk_widget_set_sensitive(ctrl->DevSel, TRUE);
        gtk_label_set_text(GTK_LABEL(ctrl->AzRead), "---");
        gtk_label_set_text(GTK_LABEL(ctrl->ElRead), "---");

        if (!ctrl->client.running && ctrl->client.thread == NULL)
        {
            /* client thread is not running; nothing to do */
            if (status_label)
                gtk_label_set_text(GTK_LABEL(status_label), _("DISENGAGED"));
            return;
        }

        /* Instead of sending stop command, just signal thread to stop and shutdown socket to break blocking I/O. */
        ctrl->client.running = FALSE;
#ifndef WIN32
        if (ctrl->client.socket != -1)
            shutdown(ctrl->client.socket, SHUT_RDWR);
#else
        if (ctrl->client.socket != -1)
            shutdown(ctrl->client.socket, SD_BOTH);
#endif
        g_thread_join(ctrl->client.thread);
        ctrl->client.thread = NULL;
        ctrl->client.socket = -1;
        if (status_label)
            gtk_label_set_text(GTK_LABEL(status_label), _("DISENGAGED"));
    }
    else
    {
        {
            gchar *conf_err = NULL;
            if (!rot_conf_validate(ctrl->conf, &conf_err)) {
                sat_log_log(SAT_LOG_LEVEL_ERROR,
                            _("%s: Controller does not have a valid configuration: %s"),
                            __func__, conf_err ? conf_err : "unknown error");
                rot_show_conf_error(ctrl, conf_err);
                g_free(conf_err);
                gtk_toggle_button_set_active(button, FALSE);
                ctrl->engaged = FALSE;
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

        /* ensure we are not in monitor mode when engaging by default */
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->MonitorCheckBox), FALSE);
        ctrl->monitor = FALSE;

        /* ensure rotctld daemon is running (start it if needed) */
        {
            gboolean error_reported = FALSE;
            if (!rotctld_ensure_running(ctrl, &error_reported))
            {
                sat_log_log(SAT_LOG_LEVEL_ERROR,
                            _("%s: failed to connect to or start rotctld"),
                            __func__);

                /* Reset engage button and show error to the user */
                gtk_toggle_button_set_active(button, FALSE);
                if (status_label)
                    gtk_label_set_text(GTK_LABEL(status_label), _("ERROR: rotctld"));
                if (!error_reported)
                    rot_show_no_rotor_dialog(ctrl);
                return;
            }
        }

        /* Re-synchronise client state with current knob values so the rotor
         * does not move on its own when we first engage. We treat the
         * existing knob positions as the current physical position and do
         * not post a new target yet.
         */
        g_mutex_lock(&ctrl->client.mutex);
        ctrl->client.azi_out   = gtk_rot_knob_get_value(GTK_ROT_KNOB(ctrl->AzSet));
        ctrl->client.ele_out   = gtk_rot_knob_get_value(GTK_ROT_KNOB(ctrl->ElSet));
        ctrl->client.azi_in    = ctrl->client.azi_out;
        ctrl->client.ele_in    = ctrl->client.ele_out;
        ctrl->client.io_error  = FALSE;
        ctrl->client.cmd_rejected = FALSE;
        ctrl->client.reject_backoff_until_us = 0;
        ctrl->client.reject_backoff_sec = 0.5;
        ctrl->client.reject_backoff_log_us = 0;
        ctrl->client.limits_valid = FALSE;
        ctrl->client.new_trg   = FALSE;
        g_mutex_unlock(&ctrl->client.mutex);

        /* Reset error counter when (re)engaging the rotor. */
        ctrl->errcnt = 0;
        ctrl->out_of_range = FALSE;
        ctrl->last_oob_log_us = 0;

        if (ctrl->client.thread != NULL) {
            sat_log_log(SAT_LOG_LEVEL_WARN,
                        _("%s: rotctld client thread already running; reusing existing thread"),
                        __func__);
            gtk_widget_set_sensitive(ctrl->DevSel, FALSE);
            ctrl->engaged = TRUE;
            return;
        }

        ctrl->client.thread =
            g_thread_new("gpredict_rotctl", rotctld_client_thread, ctrl);

        /* Give the client thread a brief moment to attempt its initial connect. */
        g_usleep(100000);

        if (!ctrl->client.running)
        {
            gpointer ret = g_thread_join(ctrl->client.thread);
            ctrl->client.thread = NULL;
            ctrl->client.socket = -1;
            gtk_widget_set_sensitive(ctrl->DevSel, TRUE);
            ctrl->engaged = FALSE;
            gtk_toggle_button_set_active(button, FALSE);

            if (status_label)
                gtk_label_set_text(GTK_LABEL(status_label),
                                   _("ERROR: rotctld"));

            if (GPOINTER_TO_INT(ret) == -1)
                rot_show_no_rotor_dialog(ctrl);

            return;
        }

        gtk_widget_set_sensitive(ctrl->DevSel, FALSE);
        ctrl->engaged = TRUE;
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
static void sat_selected_cb(GtkComboBox * satsel, gpointer data)
{
    GtkRotCtrl     *ctrl = GTK_ROT_CTRL(data);
    gint            i;

    i = gtk_combo_box_get_active(satsel);
    if (i >= 0)
    {
        ctrl->target = SAT(g_slist_nth_data(ctrl->sats, i));
        rot_plan_reset(&ctrl->trajectory_plan);

        /* update next pass */
        if (ctrl->pass != NULL)
            free_pass(ctrl->pass);

        if (ctrl->target->el > 0.0)
            ctrl->pass = get_current_pass(ctrl->target, ctrl->qth, ctrl->t);
        else
            ctrl->pass = get_pass(ctrl->target, ctrl->qth, ctrl->t, 3.0);

        set_flipped_pass(ctrl);
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

    /* in either case, we set the new pass (even if NULL) on the polar plot */
    if (ctrl->plot != NULL)
        gtk_polar_plot_set_pass(GTK_POLAR_PLOT(ctrl->plot), ctrl->pass);
}

/* Create target widgets */
static GtkWidget *create_target_widgets(GtkRotCtrl * ctrl)
{
    GtkWidget      *frame, *table, *label;
    gchar          *buff;
    guint           i, n;
    sat_t          *sat = NULL;

    buff = g_strdup_printf(FMTSTR, 0.0);

    table = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(table), 5);
    gtk_grid_set_column_homogeneous(GTK_GRID(table), FALSE);
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
    gtk_grid_attach(GTK_GRID(table), ctrl->SatSel, 0, 0, 2, 1);

    /* tracking button */
    ctrl->track = gtk_toggle_button_new_with_label(_("Track"));
    gtk_widget_set_tooltip_text(ctrl->track,
                                _
                                ("Track the satellite when it is within range"));
    gtk_grid_attach(GTK_GRID(table), ctrl->track, 2, 0, 1, 1);
    g_signal_connect(ctrl->track, "toggled", G_CALLBACK(track_toggle_cb),
                     ctrl);

    /* Azimuth */
    label = gtk_label_new(_("Az:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 1, 1, 1);

    ctrl->AzSat = gtk_label_new(buff);
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), ctrl->AzSat, 1, 1, 1, 1);

    /* Elevation */
    label = gtk_label_new(_("El:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 2, 1, 1);

    ctrl->ElSat = gtk_label_new(buff);
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), ctrl->ElSat, 1, 2, 1, 1);

    /* count down */
    label = gtk_label_new(_("\316\224T:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 3, 1, 1);
    ctrl->SatCnt = gtk_label_new("00:00:00");
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), ctrl->SatCnt, 1, 3, 1, 1);

    frame = gtk_frame_new(_("Target"));
    gtk_container_add(GTK_CONTAINER(frame), table);

    g_free(buff);

    return frame;
}

static GtkWidget *create_conf_widgets(GtkRotCtrl * ctrl)
{
    GtkWidget      *frame, *main_table, *offset_table, *label, *outer;
    GDir           *dir = NULL; /* directory handle */
    GError         *error = NULL;       /* error flag and info */
    gchar          *dirname;    /* directory name */
    gchar         **vbuff;
    const gchar    *filename;   /* file name */
    gchar          *rotname;

    main_table = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(main_table), 5);
    gtk_grid_set_column_spacing(GTK_GRID(main_table), 5);
    gtk_grid_set_row_spacing(GTK_GRID(main_table), 5);

    label = gtk_label_new(_("Device:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(main_table), label, 0, 0, 1, 1);

    ctrl->DevSel = gtk_combo_box_text_new();
    gtk_widget_set_tooltip_text(ctrl->DevSel,
                                _("Select antenna rotator device"));

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
    gtk_grid_attach(GTK_GRID(main_table), ctrl->DevSel, 1, 0, 1, 1);

    /* Engage button */
    ctrl->LockBut = gtk_toggle_button_new_with_label(_("Engage"));
    gtk_widget_set_tooltip_text(ctrl->LockBut,
                                _("Engage the selected rotor device"));
    g_signal_connect(ctrl->LockBut, "toggled", G_CALLBACK(rot_locked_cb),
                     ctrl);
    gtk_grid_attach(GTK_GRID(main_table), ctrl->LockBut, 2, 0, 1, 1);

    /* Monitor checkbox */
    ctrl->MonitorCheckBox = gtk_check_button_new_with_label(_("Monitor"));
    gtk_widget_set_tooltip_text(ctrl->MonitorCheckBox,
                                _("Monitor rotator but do not send any "
                                  "position commands"));
    g_signal_connect(ctrl->MonitorCheckBox, "toggled",
                     G_CALLBACK(rot_monitor_cb), ctrl);
    gtk_grid_attach(GTK_GRID(main_table), ctrl->MonitorCheckBox, 1, 1, 1, 1);

    /* cycle period */
    label = gtk_label_new(_("Cycle:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(main_table), label, 0, 2, 1, 1);

    ctrl->cycle_spin = gtk_spin_button_new_with_range(10, 10000, 10);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(ctrl->cycle_spin), 0);
    gtk_widget_set_tooltip_text(ctrl->cycle_spin,
                                _("This parameter controls the delay between "
                                  "commands sent to the rotator."));
    g_signal_connect(ctrl->cycle_spin, "value-changed",
                     G_CALLBACK(delay_changed_cb), ctrl);
    gtk_grid_attach(GTK_GRID(main_table), ctrl->cycle_spin, 1, 2, 1, 1);

    label = gtk_label_new(_("msec"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(main_table), label, 2, 2, 1, 1);

    /* Tolerance */
    label = gtk_label_new(_("Threshold:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(main_table), label, 0, 3, 1, 1);

    ctrl->thld_spin = gtk_spin_button_new_with_range(0.01, 50.0, 0.01);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(ctrl->thld_spin), 2);
    gtk_widget_set_tooltip_text(ctrl->thld_spin,
                                _("This parameter sets the threshold that triggers "
                                  "new motion command to the rotator.\n"
                                  "If the difference between the target and "
                                  "rotator values is smaller than the "
                                  "threshold, no new commands are sent"));
    g_signal_connect(ctrl->thld_spin, "value-changed",
                     G_CALLBACK(threshold_changed_cb), ctrl);
    gtk_grid_attach(GTK_GRID(main_table), ctrl->thld_spin, 1, 3, 1, 1);

    label = gtk_label_new(_("deg"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(main_table), label, 2, 3, 1, 1);

    /* Status line */
    label = gtk_label_new(_("Status:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(main_table), label, 0, 4, 1, 1);

    GtkWidget *status = gtk_label_new(_("DISENGAGED"));
    g_object_set(status, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(main_table), status, 1, 4, 2, 1);

    /* store pointer on the controller object for later updates */
    g_object_set_data(G_OBJECT(ctrl), "rot-status-label", status);

    /* Verbose logging */
    label = gtk_label_new(_("Logging:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(main_table), label, 0, 5, 1, 1);

    GtkWidget *verbose_check = gtk_check_button_new_with_label(_("Verbose"));
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(verbose_check),
                                 ctrl->verbose_logging);
    gtk_widget_set_tooltip_text(verbose_check,
                                _("Include rotctld -vvvv output and extra "
                                  "diagnostic logging in the terminal."));
    g_signal_connect(verbose_check, "toggled",
                     G_CALLBACK(rot_verbose_cb), ctrl);
    gtk_grid_attach(GTK_GRID(main_table), verbose_check, 1, 5, 1, 1);

    /* Logs toggle */
    ctrl->log_toggle = gtk_toggle_button_new_with_label(_("Show log"));
    gtk_widget_set_tooltip_text(ctrl->log_toggle,
                                _("Show or hide the rotor control log"));
    g_signal_connect(ctrl->log_toggle, "toggled",
                     G_CALLBACK(rot_logs_toggle_cb), ctrl);
    if (ctrl->term_view != NULL)
        gp_term_view_set_visible(ctrl->term_view, FALSE);
    gtk_grid_attach(GTK_GRID(main_table), ctrl->log_toggle, 2, 1, 1, 1);

    /* Offsets UI in a compact secondary column */
    offset_table = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(offset_table), 5);
    gtk_grid_set_column_spacing(GTK_GRID(offset_table), 5);
    gtk_grid_set_row_spacing(GTK_GRID(offset_table), 5);

    GtkWidget *offset_check = gtk_check_button_new_with_label(_("Enable Az/El offsets"));
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(offset_check), ctrl->use_offset);
    gtk_widget_set_tooltip_text(offset_check,
                                _("Apply fixed software offsets to azimuth and elevation before sending commands to the rotor."));
    g_signal_connect(offset_check, "toggled",
                     G_CALLBACK(offset_toggle_cb), ctrl);
    ctrl->offset_check = offset_check;
    gtk_grid_attach(GTK_GRID(offset_table), offset_check, 0, 0, 2, 1);

    label = gtk_label_new(_("Az offset:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(offset_table), label, 0, 1, 1, 1);

    GtkWidget *az_spin = gtk_spin_button_new_with_range(-360.0, 360.0, 0.1);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(az_spin), 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(az_spin), ctrl->az_offset_deg);
    gtk_widget_set_tooltip_text(az_spin,
                                _("Azimuth offset in degrees. This value is added to the logical azimuth before sending commands to the rotor."));
    g_signal_connect(az_spin, "value-changed",
                     G_CALLBACK(az_offset_changed_cb), ctrl);
    ctrl->az_offset_spin = az_spin;
    gtk_grid_attach(GTK_GRID(offset_table), az_spin, 1, 1, 1, 1);

    label = gtk_label_new(_("deg"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(offset_table), label, 2, 1, 1, 1);

    label = gtk_label_new(_("El offset:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(offset_table), label, 0, 2, 1, 1);

    GtkWidget *el_spin = gtk_spin_button_new_with_range(-90.0, 90.0, 0.1);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(el_spin), 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(el_spin), ctrl->el_offset_deg);
    gtk_widget_set_tooltip_text(el_spin,
                                _("Elevation offset in degrees. This value is added to the logical elevation before sending commands to the rotor."));
    g_signal_connect(el_spin, "value-changed",
                     G_CALLBACK(el_offset_changed_cb), ctrl);
    ctrl->el_offset_spin = el_spin;
    gtk_grid_attach(GTK_GRID(offset_table), el_spin, 1, 2, 1, 1);

    label = gtk_label_new(_("deg"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(offset_table), label, 2, 2, 1, 1);

    GtkWidget *offset_frame = gtk_frame_new(_("Offsets"));
    gtk_container_add(GTK_CONTAINER(offset_frame), offset_table);

    /* Combine main settings and offsets side-by-side */
    outer = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(outer), 0);
    gtk_grid_set_column_spacing(GTK_GRID(outer), 10);
    gtk_grid_attach(GTK_GRID(outer), main_table, 0, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(outer), offset_frame, 1, 0, 1, 1);

    /* load initial rotator configuration */
    rot_selected_cb(GTK_COMBO_BOX(ctrl->DevSel), ctrl);

    frame = gtk_frame_new(_("Settings"));
    gtk_container_add(GTK_CONTAINER(frame), outer);

    return frame;
}


/**
 * Start a simple calibration sequence.
 *
 * For now this is intentionally very simple and "taped together":
 *  - It requires the rotor to be engaged and the rotctld client thread running.
 *  - It commands the rotor to AZ=0 / EL=0 via the existing client path.
 *  - It then shows a dialog asking the user to mechanically align the antenna
 *    to true North and level, and close the dialog when done.
 *
 * No offsets are stored yet; this just automates the "drive to 0/0 and prompt
 * the user" part of the calibration workflow.
 */
static void
rot_calibration_start_cb(GtkButton *button, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);
    GtkWidget *toplevel;
    GtkWindow *parent = NULL;

    (void)button;

    /* Require a valid configuration and an engaged, running client. */
    if (!ctrl->conf || !ctrl->engaged || !ctrl->client.running) {
        GtkWidget *dlg;

        toplevel = gtk_widget_get_toplevel(GTK_WIDGET(ctrl));
        if (GTK_IS_WINDOW(toplevel))
            parent = GTK_WINDOW(toplevel);

        dlg = gtk_message_dialog_new(parent,
                                     GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
                                     GTK_MESSAGE_WARNING,
                                     GTK_BUTTONS_OK,
                                     "%s",
                                     _("Engage the rotator before starting calibration."));
        gtk_window_set_title(GTK_WINDOW(dlg), _("Calibration"));
        gtk_dialog_run(GTK_DIALOG(dlg));
        gtk_widget_destroy(dlg);
        return;
    }

    /* Disable tracking while calibration is running so that the
     * control loop does not immediately override the 0/0 command
     * with a satellite target az/el.
     */
    if (ctrl->tracking) {
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->track), FALSE);
        ctrl->tracking = FALSE;
    }

    /* Command AZ/EL = 0/0 through the normal client path. */
    if (g_mutex_trylock(&ctrl->client.mutex))
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "MISSION_SOPHIE: calibration posting rotor target cmd=(0.00, 0.00) engaged=%d monitor=%d",
                    ctrl->engaged ? 1 : 0,
                    ctrl->monitor ? 1 : 0);

        ctrl->client.azi_out = 0.0;
        ctrl->client.ele_out = 0.0;
        ctrl->client.new_trg = TRUE;
        g_mutex_unlock(&ctrl->client.mutex);
    }
    else
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "MISSION_SOPHIE: calibration skipped command post – client mutex busy");
    }

    /* Also update the knobs so the UI reflects the commanded position. */
    gtk_rot_knob_set_value(GTK_ROT_KNOB(ctrl->AzSet), 0.0);
    gtk_rot_knob_set_value(GTK_ROT_KNOB(ctrl->ElSet), 0.0);

    /* Inform the user what to do next. */
    {
        GtkWidget *dlg;

        toplevel = gtk_widget_get_toplevel(GTK_WIDGET(ctrl));
        if (GTK_IS_WINDOW(toplevel))
            parent = GTK_WINDOW(toplevel);

        dlg = gtk_message_dialog_new(
            parent,
            GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
            GTK_MESSAGE_INFO,
            GTK_BUTTONS_OK,
            "%s",
            _("The rotor has been commanded to AZ=0°, EL=0°.\n\n"
              "Now mechanically align the antenna boom to true North\n"
              "and level. When you are done, click OK."));
        gtk_window_set_title(GTK_WINDOW(dlg), _("Calibration"));
        gtk_dialog_run(GTK_DIALOG(dlg));
        gtk_widget_destroy(dlg);
    }
}

/**
 * Park the rotor at a "rest" position.
 *
 * For Matteo's current station we define the park position as
 * AZ=0°, EL=90° (true North, antenna pointing straight up).
 *
 * This does NOT change the logical calibration (which remains
 * AZ=0°, EL=0° at the North horizon). It is only a convenience
 * command to move the rotor to a preferred rest position.
 */
static void
rot_park_zenith_cb(GtkButton *button, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);
    GtkWidget *toplevel;
    GtkWindow *parent = NULL;

    (void)button;

    /* Require a valid configuration and an engaged, running client. */
    if (!ctrl->conf || !ctrl->engaged || !ctrl->client.running) {
        GtkWidget *dlg;

        toplevel = gtk_widget_get_toplevel(GTK_WIDGET(ctrl));
        if (GTK_IS_WINDOW(toplevel))
            parent = GTK_WINDOW(toplevel);

        dlg = gtk_message_dialog_new(parent,
                                     GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
                                     GTK_MESSAGE_WARNING,
                                     GTK_BUTTONS_OK,
                                     "%s",
                                     _("Engage the rotator before parking it."));
        gtk_window_set_title(GTK_WINDOW(dlg), _("Park rotor"));
        gtk_dialog_run(GTK_DIALOG(dlg));
        gtk_widget_destroy(dlg);
        return;
    }

    /* Command AZ/EL = 0/90 through the normal client path. */
    if (g_mutex_trylock(&ctrl->client.mutex)) {
        ctrl->client.azi_out = 0.0;
        ctrl->client.ele_out = 90.0;
        ctrl->client.new_trg = TRUE;
        g_mutex_unlock(&ctrl->client.mutex);
    }

    /* Also update the knobs so the UI reflects the commanded position. */
    gtk_rot_knob_set_value(GTK_ROT_KNOB(ctrl->AzSet), 0.0);
    gtk_rot_knob_set_value(GTK_ROT_KNOB(ctrl->ElSet), 90.0);

    /* Inform the user what was commanded. */
    {
        GtkWidget *dlg;

        toplevel = gtk_widget_get_toplevel(GTK_WIDGET(ctrl));
        if (GTK_IS_WINDOW(toplevel))
            parent = GTK_WINDOW(toplevel);

        dlg = gtk_message_dialog_new(
            parent,
            GTK_DIALOG_MODAL | GTK_DIALOG_DESTROY_WITH_PARENT,
            GTK_MESSAGE_INFO,
            GTK_BUTTONS_OK,
            "%s",
            _("The rotor has been commanded to AZ=0°, EL=90° (park position).\n\n"
              "Verify that the antenna is pointing straight up over true North."));
        gtk_window_set_title(GTK_WINDOW(dlg), _("Park rotor"));
        gtk_dialog_run(GTK_DIALOG(dlg));
        gtk_widget_destroy(dlg);
    }
}

/* Create calibration widgets */
static GtkWidget *create_cal_widgets(GtkRotCtrl * ctrl)
{
    GtkWidget *frame, *grid, *label, *button;

    frame = gtk_frame_new(_("Calibration"));

    grid = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(grid), 5);
    gtk_grid_set_column_spacing(GTK_GRID(grid), 5);
    gtk_grid_set_row_spacing(GTK_GRID(grid), 5);
    gtk_container_add(GTK_CONTAINER(frame), grid);

    label = gtk_label_new(_("Auto-calibration"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(grid), label, 0, 0, 1, 1);

    button = gtk_button_new_with_label(_("Start"));
    gtk_widget_set_tooltip_text(button,
                                _("Send the rotor to AZ=0°, EL=0° and guide the\n"
                                  "mechanical alignment of the antenna."));
    g_signal_connect(button, "clicked",
                     G_CALLBACK(rot_calibration_start_cb), ctrl);
    gtk_grid_attach(GTK_GRID(grid), button, 1, 0, 1, 1);

    /* Park button: move rotor to AZ=0°, EL=90° (rest position) */
    label = gtk_label_new(_("Park (AZ=0°, EL=90°)"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(grid), label, 0, 1, 1, 1);

    button = gtk_button_new_with_label(_("Park"));
    gtk_widget_set_tooltip_text(button,
                                _("Send the rotor to AZ=0°, EL=90° as a rest/park position."));
    g_signal_connect(button, "clicked",
                     G_CALLBACK(rot_park_zenith_cb), ctrl);
    gtk_grid_attach(GTK_GRID(grid), button, 1, 1, 1, 1);

    return frame;
}

/* Create target widgets */
static GtkWidget *create_plot_widget(GtkRotCtrl * ctrl)
{
    GtkWidget      *frame;

    ctrl->plot = gtk_polar_plot_new(ctrl->qth, ctrl->pass);

    frame = gtk_frame_new(NULL);
    gtk_container_add(GTK_CONTAINER(frame), ctrl->plot);

    return frame;
}

/** Copy satellite from hash table to singly linked list. */
static void store_sats(gpointer key, gpointer value, gpointer user_data)
{
    GtkRotCtrl     *ctrl = GTK_ROT_CTRL(user_data);
    sat_t          *sat = SAT(value);

    (void)key;                  /* avoid unused variable warning */

    ctrl->sats = g_slist_insert_sorted(ctrl->sats, sat,
                                       (GCompareFunc) sat_name_compare);
}

/** Check that we have at least one .rot file */
static gboolean have_conf()
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

static void gtk_rot_ctrl_init(GtkRotCtrl * ctrl,
			      gpointer g_class)
{
    (void)g_class;

    ctrl->sats = NULL;
    ctrl->target = NULL;
    ctrl->pass = NULL;
    ctrl->qth = NULL;
    ctrl->plot = NULL;

    ctrl->tracking = FALSE;
    ctrl->monitor  = FALSE;
    ctrl->engaged = FALSE;
    ctrl->delay = 300;      /* default: 300 ms control cycle */
    ctrl->timerid = 0;
    ctrl->threshold = 1.0;  /* default: 1 degree error tolerance */
    ctrl->errcnt = 0;
    ctrl->conf = NULL;
    ctrl->term_view = gp_term_view_new(_("Follow tail"), TRUE, FALSE);
    ctrl->log_toggle = NULL;
    ctrl->rotctld_proc = NULL;
    ctrl->rotctld_out_thread = NULL;
    ctrl->rotctld_err_thread = NULL;
    ctrl->rotctld_spawned = FALSE;
    ctrl->verbose_logging = FALSE;

    /* Offset defaults */
    ctrl->use_offset   = FALSE;
    ctrl->az_offset_deg = 0.0;
    ctrl->el_offset_deg = 0.0;
    ctrl->offset_check = NULL;
    ctrl->az_offset_spin = NULL;
    ctrl->el_offset_spin = NULL;
    ctrl->out_of_range = FALSE;
    ctrl->last_oob_log_us = 0;
    ctrl->last_oob_raw_az = 0.0;
    ctrl->last_oob_raw_el = 0.0;
    ctrl->last_oob_mapped_az = 0.0;
    ctrl->last_oob_mapped_el = 0.0;
    memset(&ctrl->trajectory_plan, 0, sizeof(ctrl->trajectory_plan));
    rot_plan_reset(&ctrl->trajectory_plan);
    /* Default to conservative send-only until rotctld probe says otherwise. */
    ctrl->send_only_mode = TRUE;

    g_mutex_init(&ctrl->client.mutex);
    ctrl->client.thread = NULL;
    ctrl->client.socket = -1;
    ctrl->client.running = FALSE;
    ctrl->client.io_error = FALSE;
    ctrl->client.cmd_rejected = FALSE;
    ctrl->client.reject_backoff_until_us = 0;
    ctrl->client.reject_backoff_sec = 0.5;
    ctrl->client.reject_backoff_log_us = 0;
    ctrl->client.limits_valid = FALSE;
    ctrl->client.daemon_ok = FALSE;

    if (g_getenv("GPREDICT_ROT_PLAN_TEST"))
        rot_plan_debug_harness();
}

static void gtk_rot_ctrl_destroy(GtkWidget * widget)
{
    GtkRotCtrl     *ctrl = GTK_ROT_CTRL(widget);

    /* stop timer */
    if (ctrl->timerid > 0) {
        g_source_remove(ctrl->timerid);
        ctrl->timerid = 0;
    }

    /* free configuration */
    if (ctrl->conf != NULL)
    {
        rotor_conf_save(ctrl->conf);
        g_free(ctrl->conf->name);
        g_free(ctrl->conf->host);
        g_free(ctrl->conf);
        ctrl->conf = NULL;
    }

    /* stop client thread */
    if (ctrl->client.thread)
    {
        /* Signal the thread to stop, then wait for it */
        ctrl->client.running = FALSE;
#ifndef WIN32
        if (ctrl->client.socket != -1)
            shutdown(ctrl->client.socket, SHUT_RDWR);
#else
        if (ctrl->client.socket != -1)
            shutdown(ctrl->client.socket, SD_BOTH);
#endif
        g_thread_join(ctrl->client.thread);
        ctrl->client.thread = NULL;
        ctrl->client.socket = -1;
    }

    rotctld_process_stop(ctrl);

    if (ctrl->term_view != NULL)
    {
        gp_term_view_free(ctrl->term_view);
        ctrl->term_view = NULL;
    }
    ctrl->log_toggle = NULL;

    rot_plan_reset(&ctrl->trajectory_plan);

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

GType gtk_rot_ctrl_get_type()
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

    /* check that we have rot conf */
    if (!have_conf())
        return NULL;

    rot_ctrl = GTK_ROT_CTRL(g_object_new(GTK_TYPE_ROT_CTRL, NULL));

    /* store satellites */
    g_hash_table_foreach(module->satellites, store_sats, rot_ctrl);

    rot_ctrl->target = SAT(g_slist_nth_data(rot_ctrl->sats, 0));

    /* store current time (don't know if real or simulated) */
    rot_ctrl->t = module->tmgCdnum;

    /* store QTH */
    rot_ctrl->qth = module->qth;

    /* get next pass for target satellite */
    if (rot_ctrl->target)
    {
        if (rot_ctrl->target->el > 0.0)
        {
            rot_ctrl->pass = get_current_pass(rot_ctrl->target,
                                              rot_ctrl->qth, 0.0);
        }
        else
        {
            rot_ctrl->pass = get_next_pass(rot_ctrl->target,
                                           rot_ctrl->qth, 3.0);
        }
    }

    /* create contents */
    table = gtk_grid_new();
    gtk_grid_set_column_homogeneous(GTK_GRID(table), TRUE);
    gtk_grid_set_row_homogeneous(GTK_GRID(table), FALSE);
    gtk_grid_set_row_spacing(GTK_GRID(table), 5);
    gtk_grid_set_column_spacing(GTK_GRID(table), 5);
    gtk_container_set_border_width(GTK_CONTAINER(table), 0);
    gtk_grid_attach(GTK_GRID(table), create_az_widgets(rot_ctrl), 0, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(table), create_el_widgets(rot_ctrl), 1, 0, 1, 1);
    gtk_grid_attach(GTK_GRID(table), create_target_widgets(rot_ctrl),
                    0, 1, 1, 1);
    gtk_grid_attach(GTK_GRID(table), create_conf_widgets(rot_ctrl),
                    1, 1, 1, 1);
    gtk_grid_attach(GTK_GRID(table), create_cal_widgets(rot_ctrl), 0, 2, 2, 1);
    gtk_grid_attach(GTK_GRID(table),
                    gp_term_view_get_widget(rot_ctrl->term_view),
                    0, 3, 2, 1);

    gtk_box_pack_start(GTK_BOX(rot_ctrl), create_plot_widget(rot_ctrl),
                       TRUE, TRUE, 5);
    gtk_box_pack_start(GTK_BOX(rot_ctrl), table, FALSE, FALSE, 5);
    gtk_container_set_border_width(GTK_CONTAINER(rot_ctrl), 5);

    if (module->target > 0)
        gtk_rot_ctrl_select_sat(rot_ctrl, module->target);

    /* start the control loop timer so we actually send commands */
    if (rot_ctrl->timerid == 0) {
        rot_ctrl->timerid = g_timeout_add(rot_ctrl->delay,
                                          rot_ctrl_timeout_cb,
                                          rot_ctrl);
    }

    return GTK_WIDGET(rot_ctrl);
}

/* Offset controls callbacks */
static void offset_toggle_cb(GtkToggleButton *button, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);
    ctrl->use_offset = gtk_toggle_button_get_active(button);
    if (ctrl->conf)
        ctrl->conf->use_offset = ctrl->use_offset;
    rot_plan_reset(&ctrl->trajectory_plan);
    if (ctrl->tracking && ctrl->pass)
        rot_build_tracking_plan(ctrl);
}

static void az_offset_changed_cb(GtkSpinButton *spin, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);
    ctrl->az_offset_deg = gtk_spin_button_get_value(spin);
    if (ctrl->conf)
        ctrl->conf->az_offset = ctrl->az_offset_deg;
    rot_plan_reset(&ctrl->trajectory_plan);
    if (ctrl->tracking && ctrl->pass)
        rot_build_tracking_plan(ctrl);
}

static void el_offset_changed_cb(GtkSpinButton *spin, gpointer data)
{
    GtkRotCtrl *ctrl = GTK_ROT_CTRL(data);
    ctrl->el_offset_deg = gtk_spin_button_get_value(spin);
    if (ctrl->conf)
        ctrl->conf->el_offset = ctrl->el_offset_deg;
    rot_plan_reset(&ctrl->trajectory_plan);
    if (ctrl->tracking && ctrl->pass)
        rot_build_tracking_plan(ctrl);
}
