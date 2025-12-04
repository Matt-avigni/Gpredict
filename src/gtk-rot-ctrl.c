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
#include <gtk/gtk.h>
#include <math.h>
#include <string.h>             /* strerror() */

#include "compat.h"
#include "gpredict-utils.h"
#include "gtk-polar-plot.h"
#include "gtk-rot-knob.h"
#include "gtk-rot-ctrl.h"
#include "predict-tools.h"
#include "sat-log.h"


#define FMTSTR "%7.2f\302\260"
#define MAX_ERROR_COUNT 5


static GtkVBoxClass *parent_class = NULL;

/* Forward declaration for error dialog helper */
static void rot_show_no_rotor_dialog(GtkRotCtrl *ctrl);


/* Open the rotcld socket. Returns file descriptor or -1 if an error occurs */
static gint rotctld_socket_open(const gchar * host, gint port)
{
    struct sockaddr_in ServAddr;
    struct hostent *h;
    gint            sock;
    gint            status;

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
    h = gethostbyname(host);
    if (h == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("Name resolution of rotctld server %s failed."), host);
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
                    host, port, strerror(errno));

#ifdef WIN32
        closesocket(sock);
#else
        close(sock);
#endif
        return -1;
    }

    sat_log_log(SAT_LOG_LEVEL_DEBUG, _("%s: Connection opened to %s:%d"),
                __func__, host, port);

    return sock;
}

/* Close a rotcld socket. First send a q command to cleanly shut down rotctld */
static void rotctld_socket_close(gint * sock)
{
    gint            written;

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

/*
 * Send a command to rotctld and read the response.
 *
 * Inputs are the socket, a string command, and a buffer and length for
 * returning the output from rotctld.
 */
static gboolean rotctld_socket_rw(gint sock, gchar * buff, gchar * buffout,
                                  gint sizeout)
{
    gint            written;
    gint            size;

    g_print("rotctld_socket_rw: sending command '%s'\n", buff);
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
        g_print("rotctld_socket_rw: send() failed for command '%s'\n", buff);
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: rotctld Socket Down"), __func__);
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
        return FALSE;
    }

    buffout[size] = '\0';
    if (size == 0)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%s: Got 0 bytes from rotctld"), __FILE__, __func__);
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

static inline void set_flipped_pass(GtkRotCtrl * ctrl)
{
    if (ctrl->conf && ctrl->pass)
        ctrl->flipped = is_flipped_pass(ctrl->pass, ctrl->conf->aztype,
                                        ctrl->conf->azstoppos);
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

    g_print("MATTEO_DEBUG: get_pos stub using commanded values\n");

    /* Send-only mode: reuse last commanded values instead of asking rotctld. */
    g_mutex_lock(&ctrl->client.mutex);
    *az = ctrl->client.azi_out;
    *el = ctrl->client.ele_out;
    g_mutex_unlock(&ctrl->client.mutex);

    return TRUE;
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
static gboolean set_pos(GtkRotCtrl * ctrl, gdouble az, gdouble el)
{
    gchar          *buff;
    gchar           buffback[128];
    gboolean        retcode;

    g_print("set_pos: az=%.2f el=%.2f\n", az, el);

    /* send command */
    {
        gchar azbuf[G_ASCII_DTOSTR_BUF_SIZE];
        gchar elbuf[G_ASCII_DTOSTR_BUF_SIZE];

        g_ascii_dtostr(azbuf, sizeof(azbuf), az);
        g_ascii_dtostr(elbuf, sizeof(elbuf), el);

        buff = g_strdup_printf("P %s %s\x0a", azbuf, elbuf);
    }
    retcode = rotctld_socket_rw(ctrl->client.socket, buff, buffback, 128);
    g_free(buff);

    if (retcode == TRUE)
    {
        /* Ignore Hamlib RPRT codes here and treat any successful socket
         * round-trip as a successful command. Some backends may return
         * RPRT -5 or similar even though the command was accepted, and
         * we don't want that to disengage the rotor.
         */
        g_print("set_pos: rotctld replied '%s'\n", buffback);
    }

    return retcode;
}

/* Rotctl client thread */
static gpointer rotctld_client_thread(gpointer data)
{
    gdouble         elapsed_time;
    gdouble         azi = 0.0;
    gdouble         ele = 0.0;
    gboolean        io_error = FALSE;
    GtkRotCtrl     *ctrl = GTK_ROT_CTRL(data);

    g_print("IUT_ROT_DEBUG: rotctld_client_thread started\n");

    ctrl->client.socket = rotctld_socket_open(ctrl->conf->host,
                                              ctrl->conf->port);
    if (ctrl->client.socket == -1)
        return GINT_TO_POINTER(-1);

    ctrl->client.timer = g_timer_new();

    ctrl->client.new_trg = FALSE;
    ctrl->client.running = TRUE;

    while (ctrl->client.running)
    {
        g_timer_start(ctrl->client.timer);
        io_error = FALSE;

        /* get latest commanded position from controller, but only
         * send a new command when new_trg is set. This avoids
         * hammering the rotor with repeated small corrections and
         * reduces "hunting" around the target position.
         */
        gboolean send_cmd = FALSE;

        g_mutex_lock(&ctrl->client.mutex);
        azi = ctrl->client.azi_out;
        ele = ctrl->client.ele_out;
        if (ctrl->client.new_trg)
        {
            send_cmd = TRUE;
            ctrl->client.new_trg = FALSE;
        }
        g_mutex_unlock(&ctrl->client.mutex);

        if (send_cmd)
        {
            g_print("MATTEO_DEBUG: set_pos (new target) (engaged=%d monitor=%d az=%.2f el=%.2f)\n",
                    ctrl->engaged ? 1 : 0,
                    ctrl->monitor ? 1 : 0,
                    azi, ele);

            if (!set_pos(ctrl, azi, ele))
            {
                io_error = TRUE;
                g_print("MATTEO_DEBUG: set_pos FAILED\n");
            }
            else
            {
                g_print("MATTEO_DEBUG: set_pos SUCCESS\n");
            }
        }
        else
        {
            g_print("MATTEO_DEBUG: no new target, skipping set_pos\n");
        }

        /* Treat last commanded az/el as the "measured" position for
         * display purposes, since some rotctld backends only return
         * RPRT codes to the "p" command and do not support true
         * position read-back.
         */
        g_mutex_lock(&ctrl->client.mutex);
        ctrl->client.azi_in   = ctrl->client.azi_out;
        ctrl->client.ele_in   = ctrl->client.ele_out;
        ctrl->client.io_error = io_error;
        g_mutex_unlock(&ctrl->client.mutex);

        /* ensure rotctl duty cycle stays below 50%, but wait at least 700 ms */
        elapsed_time = MAX(g_timer_elapsed(ctrl->client.timer, NULL), 0.7);
        g_usleep(elapsed_time * 1e6);
    }

    g_print("Stopping rotctld client thread\n");
    g_timer_destroy(ctrl->client.timer);
    rotctld_socket_close(&ctrl->client.socket);

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
    gchar *text;
    gboolean error = FALSE;
    sat_t sat_working, *sat;
    GtkWidget *status_label =
        g_object_get_data(G_OBJECT(ctrl), "rot-status-label");

    /* parameters for path predictions */
    gdouble time_delta;
    gdouble step_size;

#define SAFE_AZI(azi) CLAMP(azi, ctrl->conf->minaz, ctrl->conf->maxaz)
#define SAFE_ELE(ele) CLAMP(ele, ctrl->conf->minel, ctrl->conf->maxel)

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
                    setaz = SAFE_AZI(ctrl->pass->aos_az);
                    setel = SAFE_ELE(0.0);
                }
                else if (ctrl->t > ctrl->pass->los)
                {
                    setaz = SAFE_AZI(ctrl->pass->los_az);
                    setel = SAFE_ELE(0.0);
                }
            }
        }
        else
        {
            setaz = SAFE_AZI(ctrl->target->az);
            setel = SAFE_ELE(ctrl->target->el);
        }

        /* if this is a flipped pass and the rotor supports it */
        if (ctrl->flipped && ctrl->conf->maxel >= 180.0)
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
        if (fabs(setaz - rotaz) > ctrl->threshold ||
            fabs(setel - rotel) > ctrl->threshold)
        {
            /* If tracking is enabled, refine the target to "lead" the pass
             * a bit into the future, like the original code did.
             * Manual mode skips this and just uses the knob values.
             */
            if (ctrl->tracking && ctrl->target && ctrl->conf)
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
                        if (ctrl->flipped && ctrl->conf->maxel >= 180.0)
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

                    setel = SAFE_ELE(sat->el);
                    setaz = SAFE_AZI(sat->az);
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

            /* keep knobs in sync with the commanded position */
            gtk_rot_knob_set_value(GTK_ROT_KNOB(ctrl->AzSet), setaz);
            gtk_rot_knob_set_value(GTK_ROT_KNOB(ctrl->ElSet), setel);

            if (g_mutex_trylock(&ctrl->client.mutex))
            {
                if (fabs(setaz - ctrl->client.azi_out) > ctrl->threshold ||
                    fabs(setel - ctrl->client.ele_out) > ctrl->threshold)
                {
                    ctrl->client.azi_out = setaz;
                    ctrl->client.ele_out = setel;
                    ctrl->client.new_trg = TRUE;
                }
                g_mutex_unlock(&ctrl->client.mutex);
            }
        }

        /* check error status */
        if (!error)
        {
            ctrl->errcnt = 0;
        }
        else
        {
            if (ctrl->errcnt >= MAX_ERROR_COUNT)
            {
                /* disengage device */
                gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->LockBut),
                                             FALSE);
                ctrl->engaged = FALSE;
                sat_log_log(SAT_LOG_LEVEL_ERROR,
                            _("%s: MAX_ERROR_COUNT (%d) reached. Disengaging device!"),
                            __func__, MAX_ERROR_COUNT);

                /* Reflect the failure in the status label and show a dialog
                 * so it's obvious that the rotor/rotctld backend is not usable.
                 */
                if (status_label)
                    gtk_label_set_text(GTK_LABEL(status_label),
                                       _("ERROR: link/rotator"));

                rot_show_no_rotor_dialog(ctrl);

                ctrl->errcnt = 0;
            }
            else
            {
                ctrl->errcnt++;
            }
        }

        /* update status label if present, based on real data */
        if (status_label)
        {
            const gchar *status_text = NULL;

            if (error)
                status_text = _("ERROR: link/rotator");
            else if (fabs(setaz - rotaz) > ctrl->threshold ||
                     fabs(setel - rotel) > ctrl->threshold)
                status_text = _("MOVING");
            else
                status_text = _("ON TARGET");

            gtk_label_set_text(GTK_LABEL(status_label), status_text);
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

/* Build a rotctld command line based on the current rotor config
 * and an auto-detected serial device.
 *
 * For now we hard-code:
 *   - Hamlib model 603 (Yaesu GS-232B)
 *   - 9600 baud
 *
 * This matches Matteo's current station setup and is deliberately
 * "taped together" rather than a generic solution.
 *
 * The returned string must be freed by the caller.
 */
static gchar *rotctld_build_autostart_command(GtkRotCtrl *ctrl)
{
    if (ctrl == NULL || ctrl->conf == NULL)
        return NULL;

    /* Determine the serial device to use. Allow an environment override
     * (GPREDICT_ROT_SERIAL), falling back to auto-detection otherwise.
     */
    gchar *device = NULL;
    const gchar *env_device = g_getenv("GPREDICT_ROT_SERIAL");

    if (env_device != NULL && *env_device != '\0') {
        device = g_strdup(env_device);
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: using serial device from GPREDICT_ROT_SERIAL: %s"),
                    __func__, device);
    } else {
        device = rotctld_find_serial_device();
    }

    if (!device)
        return NULL;

    /* Find full path to rotctld so we are not dependent on the GUI's PATH. */
    gchar *rotctld_path = rotctld_find_binary();
    if (!rotctld_path) {
        g_free(device);
        return NULL;
    }

    /* Default to Yaesu GS-232B (Hamlib model 603) at 9600 baud, but allow
     * overrides via environment variables GPREDICT_ROT_MODEL and
     * GPREDICT_ROT_BAUD for advanced setups.
     */
    gint model = 603;
    gint baud  = 9600;
    const gchar *env_model = g_getenv("GPREDICT_ROT_MODEL");
    const gchar *env_baud  = g_getenv("GPREDICT_ROT_BAUD");

    if (env_model != NULL && *env_model != '\0') {
        glong tmp = g_ascii_strtoll(env_model, NULL, 10);
        if (tmp > 0) {
            model = (gint) tmp;
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        _("%s: using Hamlib model %d from GPREDICT_ROT_MODEL"),
                        __func__, model);
        }
    }

    if (env_baud != NULL && *env_baud != '\0') {
        glong tmp = g_ascii_strtoll(env_baud, NULL, 10);
        if (tmp > 0) {
            baud = (gint) tmp;
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        _("%s: using baud rate %d from GPREDICT_ROT_BAUD"),
                        __func__, baud);
        }
    }

    const gint port  = ctrl->conf->port;

    /* Use -T 127.0.0.1 and -t <port> (correct host/port syntax for rotctld). */
    gchar *cmd = g_strdup_printf("\"%s\" -m %d -r %s -s %d -T 127.0.0.1 -t %d -vvvv",
                                 rotctld_path, model, device, baud, port);

    sat_log_log(SAT_LOG_LEVEL_INFO,
                _("%s: built auto-start command '%s'"),
                __func__, cmd);

    g_free(rotctld_path);
    g_free(device);
    return cmd;
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
 * rotator daemon (i.e., returns a valid az/el pair to the "p" command).
 *
 * This is used to avoid "ghost" connections to unrelated services or
 * misconfigured rotctld instances that only ever reply with RPRT codes.
 */
static gboolean
rotctld_probe_endpoint(GtkRotCtrl *ctrl)
{
    gint sock;
    gchar *cmd;
    gchar reply[128];
    gboolean ok = FALSE;

    if (ctrl == NULL || ctrl->conf == NULL)
        return FALSE;

    sock = rotctld_socket_open(ctrl->conf->host, ctrl->conf->port);
    if (sock == -1)
        return FALSE;

    cmd = g_strdup_printf("p\x0a");
    if (rotctld_socket_rw(sock, cmd, reply, sizeof(reply) - 1))
    {
        g_strstrip(reply);

        if (g_str_has_prefix(reply, "RPRT"))
        {
            /* Hamlib-style status reply: RPRT <code>.
             *
             * For Matteo's GS-232B/rotctld setup we treat *any* RPRT reply
             * as proof that we are talking to a live rotctld instance,
             * even if the backend does not support the "p" (position) query
             * and returns an error code such as RPRT -6.
             *
             * We no longer require RPRT 0 specifically here because we rely
             * on send-only control and do not depend on true position
             * read-back from rotctld.
             */
            ok = TRUE;
        }
        else
        {
            /* Legacy behaviour: try to parse az/el from the first two lines
             * for older backends that still return numeric az/el values.
             */
            gchar **lines = g_strsplit(reply, "\n", 3);
            if (lines[0] != NULL && lines[1] != NULL)
            {
                gchar *endptr1 = NULL;
                gchar *endptr2 = NULL;
                gdouble az = g_ascii_strtod(lines[0], &endptr1);
                gdouble el = g_ascii_strtod(lines[1], &endptr2);

                if (endptr1 != lines[0] && endptr2 != lines[1])
                    ok = TRUE;
            }
            g_strfreev(lines);
        }
    }

    g_free(cmd);

#ifndef WIN32
    shutdown(sock, SHUT_RDWR);
    close(sock);
#else
    shutdown(sock, SD_BOTH);
    closesocket(sock);
#endif

    if (ok)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: rotctld at %s:%d returned a valid position; using existing daemon."),
                    __func__, ctrl->conf->host, ctrl->conf->port);
    }
    else
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: rotctld at %s:%d did not return a valid position; treating as unusable."),
                    __func__, ctrl->conf->host, ctrl->conf->port);
    }

    return ok;
}

static gboolean rotctld_ensure_running(GtkRotCtrl *ctrl)
{
    gint tmp_sock;

    if (ctrl == NULL || ctrl->conf == NULL)
        return FALSE;

    /* Step 1: probe whether a usable rotctld is already running */
    if (rotctld_probe_endpoint(ctrl))
        return TRUE;

    /* Step 2: build a command to start rotctld. Prefer the environment
     * variable if present, otherwise fall back to our auto-detection
     * logic for Matteo's GS-232B setup.
     */
    const gchar *env_cmd = g_getenv("GPREDICT_ROTCTLD_CMD");
    gchar *auto_cmd = NULL;
    const gchar *cmd = env_cmd;

    if (cmd == NULL || *cmd == '\0')
    {
        auto_cmd = rotctld_build_autostart_command(ctrl);
        cmd = auto_cmd;
    }

    if (cmd == NULL || *cmd == '\0')
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: rotctld is not running and no usable command "
                      "line could be constructed."),
                    __func__);
        if (auto_cmd)
            g_free(auto_cmd);
        return FALSE;
    }

    GError *error = NULL;
    if (!g_spawn_command_line_async(cmd, &error))
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: failed to start rotctld using '%s': %s"),
                    __func__, cmd, error ? error->message : "unknown error");
        if (error)
            g_error_free(error);
        if (auto_cmd)
            g_free(auto_cmd);
        return FALSE;
    }

    sat_log_log(SAT_LOG_LEVEL_INFO,
                _("%s: started rotctld using '%s'"),
                __func__, cmd);

    if (auto_cmd)
        g_free(auto_cmd);

    /* Give rotctld a short time to come up, then re-probe. */
    g_usleep(500000); /* 500 ms */

    tmp_sock = rotctld_socket_open(ctrl->conf->host, ctrl->conf->port);
    if (tmp_sock == -1)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: rotctld did not become reachable at %s:%d"),
                    __func__, ctrl->conf->host, ctrl->conf->port);
        return FALSE;
    }

#ifndef WIN32
    shutdown(tmp_sock, SHUT_RDWR);
    close(tmp_sock);
#else
    shutdown(tmp_sock, SD_BOTH);
    closesocket(tmp_sock);
#endif

    sat_log_log(SAT_LOG_LEVEL_INFO,
                _("%s: rotctld is now reachable at %s:%d"),
                __func__, ctrl->conf->host, ctrl->conf->port);

    return TRUE;
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

        if (!ctrl->client.running)
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
        if (status_label)
            gtk_label_set_text(GTK_LABEL(status_label), _("DISENGAGED"));
    }
    else
    {
        if (ctrl->conf == NULL)
        {
            /* we don't have a working configuration */
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _
                        ("%s: Controller does not have a valid configuration"),
                        __func__);
            return;
        }

        /* ensure we are not in monitor mode when engaging by default */
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ctrl->MonitorCheckBox), FALSE);
        ctrl->monitor = FALSE;

        /* ensure rotctld daemon is running (start it if needed) */
        if (!rotctld_ensure_running(ctrl))
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: failed to connect to or start rotctld"),
                        __func__);

            /* Reset engage button and show error to the user */
            gtk_toggle_button_set_active(button, FALSE);
            if (status_label)
                gtk_label_set_text(GTK_LABEL(status_label), _("ERROR: rotctld"));
            rot_show_no_rotor_dialog(ctrl);
            return;
        }

        ctrl->client.thread =
            g_thread_new("gpredict_rotctl", rotctld_client_thread, ctrl);

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
    GtkWidget      *frame, *table, *label;
    GDir           *dir = NULL; /* directory handle */
    GError         *error = NULL;       /* error flag and info */
    gchar          *dirname;    /* directory name */
    gchar         **vbuff;
    const gchar    *filename;   /* file name */
    gchar          *rotname;

    table = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(table), 5);
    gtk_grid_set_column_spacing(GTK_GRID(table), 5);
    gtk_grid_set_row_spacing(GTK_GRID(table), 5);

    label = gtk_label_new(_("Device:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 0, 1, 1);

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
    gtk_grid_attach(GTK_GRID(table), ctrl->DevSel, 1, 0, 1, 1);

    /* Engage button */
    ctrl->LockBut = gtk_toggle_button_new_with_label(_("Engage"));
    gtk_widget_set_tooltip_text(ctrl->LockBut,
                                _("Engage the selected rotor device"));
    g_signal_connect(ctrl->LockBut, "toggled", G_CALLBACK(rot_locked_cb),
                     ctrl);
    gtk_grid_attach(GTK_GRID(table), ctrl->LockBut, 2, 0, 1, 1);

    /* Monitor checkbox */
    ctrl->MonitorCheckBox = gtk_check_button_new_with_label(_("Monitor"));
    gtk_widget_set_tooltip_text(ctrl->MonitorCheckBox,
                                _("Monitor rotator but do not send any "
                                  "position commands"));
    g_signal_connect(ctrl->MonitorCheckBox, "toggled",
                     G_CALLBACK(rot_monitor_cb), ctrl);
    gtk_grid_attach(GTK_GRID(table), ctrl->MonitorCheckBox, 1, 1, 1, 1);

    /* cycle period */
    label = gtk_label_new(_("Cycle:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 2, 1, 1);

    ctrl->cycle_spin = gtk_spin_button_new_with_range(10, 10000, 10);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(ctrl->cycle_spin), 0);
    gtk_widget_set_tooltip_text(ctrl->cycle_spin,
                                _("This parameter controls the delay between "
                                  "commands sent to the rotator."));
    g_signal_connect(ctrl->cycle_spin, "value-changed",
                     G_CALLBACK(delay_changed_cb), ctrl);
    gtk_grid_attach(GTK_GRID(table), ctrl->cycle_spin, 1, 2, 1, 1);

    label = gtk_label_new(_("msec"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 2, 2, 1, 1);

    /* Tolerance */
    label = gtk_label_new(_("Threshold:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 3, 1, 1);

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
    gtk_grid_attach(GTK_GRID(table), ctrl->thld_spin, 1, 3, 1, 1);

    label = gtk_label_new(_("deg"));
    g_object_set(label, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 2, 3, 1, 1);

    /* Status line */
    label = gtk_label_new(_("Status:"));
    g_object_set(label, "xalign", 1.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 4, 1, 1);

    GtkWidget *status = gtk_label_new(_("DISENGAGED"));
    g_object_set(status, "xalign", 0.0f, "yalign", 0.5f, NULL);
    gtk_grid_attach(GTK_GRID(table), status, 1, 4, 2, 1);

    /* store pointer on the controller object for later updates */
    g_object_set_data(G_OBJECT(ctrl), "rot-status-label", status);

    /* load initial rotator configuration */
    rot_selected_cb(GTK_COMBO_BOX(ctrl->DevSel), ctrl);

    frame = gtk_frame_new(_("Settings"));
    gtk_container_add(GTK_CONTAINER(frame), table);

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

    /* Command AZ/EL = 0/0 through the normal client path. */
    if (g_mutex_trylock(&ctrl->client.mutex)) {
        ctrl->client.azi_out = 0.0;
        ctrl->client.ele_out = 0.0;
        ctrl->client.new_trg = TRUE;
        g_mutex_unlock(&ctrl->client.mutex);
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

    g_mutex_init(&ctrl->client.mutex);
    ctrl->client.thread = NULL;
    ctrl->client.socket = -1;
    ctrl->client.running = FALSE;
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
    }

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
