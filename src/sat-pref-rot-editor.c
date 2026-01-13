/*
  Gpredict: Real-time satellite tracking and orbit prediction program

  Copyright (C)  2001-2017  Alexandru Csete, OZ9AEC.

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

#ifdef HAVE_CONFIG_H
#include <build-config.h>
#endif
#include <glib/gi18n.h>
#include <glib/gstdio.h>
#include <gio/gio.h>
#include <gtk/gtk.h>
#include <math.h>
#include <string.h>

#include "gpredict-utils.h"
#include "rotor-conf.h"
#include "rotctld_mgr.h"
#include "sat-cfg.h"
#include "sat-log.h"
#include "sat-pref-rot-editor.h"
#include "serial-ports.h"


extern GtkWidget *window;       /* dialog window defined in sat-pref.c */
static GtkWidget *dialog;       /* dialog window */
static GtkWidget *name;         /* Configuration name */
static GtkWidget *host;         /* host name or IP */
static GtkWidget *port;         /* port number */
static GtkWidget *autostart;
static GtkWidget *protocol;
static GtkWidget *baud;
static GtkWidget *device_combo;
static GtkWidget *device_refresh;
static GtkWidget *device_manual;
static GtkWidget *device_autopick;
static GtkWidget *device_status;
static GtkWidget *aztype;
static GtkWidget *minaz;
static GtkWidget *maxaz;
static GtkWidget *minel;
static GtkWidget *maxel;
static GtkWidget *azstoppos;
static GtkWidget *axismode;
static GtkWidget *invert_az;
static GtkWidget *invert_el;
static GtkWidget *use_offset;
static GtkWidget *az_offset;
static GtkWidget *el_offset;
static gboolean device_scan_in_progress = FALSE;
static GSList *device_cache = NULL;

static void rot_pref_show_dialog(GtkMessageType type,
                                 const gchar *primary,
                                 const gchar *secondary)
{
    GtkWindow *parent = dialog ? GTK_WINDOW(dialog) : NULL;
    GtkWidget *msg = gtk_message_dialog_new(parent,
                                            GTK_DIALOG_MODAL |
                                                GTK_DIALOG_DESTROY_WITH_PARENT,
                                            type,
                                            GTK_BUTTONS_OK,
                                            "%s",
                                            primary ? primary : "");
    if (secondary && *secondary)
    {
        gtk_message_dialog_format_secondary_text(GTK_MESSAGE_DIALOG(msg),
                                                 "%s",
                                                 secondary);
    }
    gtk_dialog_run(GTK_DIALOG(msg));
    gtk_widget_destroy(msg);
}

static gboolean rot_pref_list_contains(GSList *list, const gchar *value)
{
    for (GSList *iter = list; iter != NULL; iter = iter->next)
    {
        if (g_strcmp0(iter->data, value) == 0)
            return TRUE;
    }
    return FALSE;
}

static gboolean rot_pref_is_preferred_device(const gchar *path)
{
    gboolean match = FALSE;
    gchar *lower = NULL;

    if (path == NULL)
        return FALSE;

    lower = g_ascii_strdown(path, -1);
    if (lower)
    {
        match = (g_strrstr(lower, "usbserial") != NULL) ||
                (g_strrstr(lower, "usbmodem") != NULL) ||
                (g_strrstr(lower, "slab") != NULL) ||
                (g_strrstr(lower, "wch") != NULL);
    }
    g_free(lower);
    return match;
}

static gchar *rot_pref_pick_best_device(GSList *list, const gchar *current)
{
    if (list == NULL)
        return NULL;

    if (list->next == NULL)
        return g_strdup(list->data);

    if (current && rot_pref_list_contains(list, current))
    {
        gboolean current_preferred = rot_pref_is_preferred_device(current);
        gboolean have_preferred = FALSE;

        for (GSList *iter = list; iter != NULL; iter = iter->next)
        {
            if (rot_pref_is_preferred_device(iter->data))
            {
                have_preferred = TRUE;
                break;
            }
        }

        if (!have_preferred || current_preferred)
            return g_strdup(current);
    }

    for (GSList *iter = list; iter != NULL; iter = iter->next)
    {
        if (rot_pref_is_preferred_device(iter->data))
            return g_strdup(iter->data);
    }

#ifdef __APPLE__
    for (GSList *iter = list; iter != NULL; iter = iter->next)
    {
        if (g_str_has_prefix(iter->data, "/dev/cu."))
            return g_strdup(iter->data);
    }
#endif

    return g_strdup(list->data);
}

static void rot_pref_update_device_status(GSList *list)
{
    if (device_status == NULL)
        return;

    if (list == NULL)
        gtk_label_set_text(GTK_LABEL(device_status),
                           _("No serial devices found."));
    else
        gtk_label_set_text(GTK_LABEL(device_status), "");
}

static void rot_pref_update_device_combo(GSList *list,
                                         const gchar *current,
                                         gboolean autopick)
{
    gchar *selected = NULL;

    if (device_combo == NULL)
        return;

    gtk_combo_box_text_remove_all(GTK_COMBO_BOX_TEXT(device_combo));

    for (GSList *iter = list; iter != NULL; iter = iter->next)
    {
        const gchar *path = iter->data;
        gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(device_combo),
                                  path, path);
    }

    if (autopick)
        selected = rot_pref_pick_best_device(list, current);
    else if (current && rot_pref_list_contains(list, current))
        selected = g_strdup(current);

    if (selected)
        gtk_combo_box_set_active_id(GTK_COMBO_BOX(device_combo), selected);
    else
        gtk_combo_box_set_active(GTK_COMBO_BOX(device_combo), -1);

    rot_pref_update_device_status(list);
    g_free(selected);
}

typedef struct {
    GSList  *list;
    gchar   *current;
    gboolean autopick;
} RotDeviceScanResult;

static gboolean rot_pref_scan_devices_done(gpointer data);

static gpointer rot_pref_scan_devices_thread(gpointer data)
{
    RotDeviceScanResult *result = data;

    if (result == NULL)
        return NULL;

    result->list = gp_serial_list_candidates();
    g_idle_add(rot_pref_scan_devices_done, result);
    return NULL;
}

static gboolean rot_pref_scan_devices_done(gpointer data)
{
    RotDeviceScanResult *result = data;

    device_scan_in_progress = FALSE;
    if (device_refresh)
        gtk_widget_set_sensitive(device_refresh, TRUE);

    if (device_cache)
        gp_serial_free_candidates(device_cache);

    device_cache = result ? result->list : NULL;

    rot_pref_update_device_combo(device_cache,
                                 result ? result->current : NULL,
                                 result ? result->autopick : TRUE);

    g_free(result ? result->current : NULL);
    g_free(result);
    return G_SOURCE_REMOVE;
}

static void rot_pref_scan_devices_async(const gchar *current)
{
    RotDeviceScanResult *result = NULL;

    if (device_scan_in_progress)
        return;

    device_scan_in_progress = TRUE;
    if (device_refresh)
        gtk_widget_set_sensitive(device_refresh, FALSE);

    result = g_new0(RotDeviceScanResult, 1);
    result->current = current ? g_strdup(current) : NULL;
    result->autopick = device_autopick ?
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(device_autopick)) : TRUE;

    GThread *thread = g_thread_new("rot-device-scan",
                                   rot_pref_scan_devices_thread,
                                   result);
    g_thread_unref(thread);
}

static gchar *rot_pref_resolve_device(const gchar *current,
                                      gboolean autopick,
                                      gchar **detail)
{
    GSList *list = NULL;
    gchar *picked = NULL;

    if (detail)
        *detail = NULL;

    list = device_cache;
    if (list == NULL)
        list = gp_serial_list_candidates();

    if (autopick)
        picked = rot_pref_pick_best_device(list, current);
    else if (current && rot_pref_list_contains(list, current))
        picked = g_strdup(current);

    if (picked == NULL && detail)
        *detail = g_strdup("No serial devices detected");

    if (list != device_cache)
        gp_serial_free_candidates(list);

    return picked;
}

static void rotctld_test_connection_cb(GtkButton *button, gpointer data)
{
    const gchar *host_text = gtk_entry_get_text(GTK_ENTRY(host));
    gint port_val = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(port));
    gchar *device = NULL;
    gchar *device_note = NULL;
    gchar *detail = NULL;
    gchar *stderr_tail = NULL;
    gboolean ok = FALSE;
    gboolean spawned = FALSE;
    RotctldMgr *mgr = NULL;

    (void)button;
    (void)data;

    if (host_text == NULL || *host_text == '\0')
    {
        rot_pref_show_dialog(GTK_MESSAGE_ERROR,
                             _("rotctld connection failed"),
                             _("Missing host."));
        return;
    }

    if (port_val <= 0 || port_val > 65535)
    {
        rot_pref_show_dialog(GTK_MESSAGE_ERROR,
                             _("rotctld connection failed"),
                             _("Invalid port."));
        return;
    }

    if (device_manual &&
        gtk_entry_get_text_length(GTK_ENTRY(device_manual)) > 0)
    {
        device = g_strdup(gtk_entry_get_text(GTK_ENTRY(device_manual)));
    }
    else if (device_combo)
    {
        device = gtk_combo_box_text_get_active_text(
            GTK_COMBO_BOX_TEXT(device_combo));
    }

    if (device_autopick &&
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(device_autopick)) &&
        (device == NULL || *device == '\0'))
    {
        g_free(device);
        device = rot_pref_resolve_device(NULL, TRUE, &device_note);
    }

    if (rotctld_mgr_host_is_local(host_text) && device && *device &&
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(autostart)))
    {
        gint model = rot_protocol_to_hamlib_model(
            gtk_combo_box_get_active(GTK_COMBO_BOX(protocol)));
        gint baud_val = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(baud));
        gchar *error = NULL;

        if (baud_val <= 0)
            baud_val = rot_protocol_default_baud(
                gtk_combo_box_get_active(GTK_COMBO_BOX(protocol)));

        mgr = rotctld_mgr_spawn(host_text, port_val, model, device, baud_val,
                                TRUE, &error);
        if (mgr == NULL)
        {
            detail = g_strdup_printf("Host: %s\nPort: %d\nError: %s",
                                     host_text, port_val,
                                     error ? error : "spawn failed");
            rot_pref_show_dialog(GTK_MESSAGE_ERROR,
                                 _("rotctld connection failed"),
                                 detail);
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        "rotctld test spawn failed host=%s port=%d error=%s",
                        host_text, port_val,
                        error ? error : "unknown");
            g_free(error);
            g_free(detail);
            g_free(device);
            g_free(device_note);
            return;
        }

        spawned = TRUE;
        rotctld_mgr_set_log_callback(mgr, NULL, NULL);
    }

    ok = rotctld_mgr_wait_for_port(host_text, port_val, 2000);
    if (!ok)
        stderr_tail = mgr ? rotctld_mgr_get_log_tail(mgr) : NULL;

    if (ok)
    {
        detail = g_strdup_printf("Host: %s\nPort: %d%s",
                                 host_text, port_val,
                                 device ? "\nrotctld responded to dump_state"
                                        : "\nrotctld responded");
        rot_pref_show_dialog(GTK_MESSAGE_INFO,
                             _("rotctld connection OK"),
                             detail);
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "rotctld test ok host=%s port=%d", host_text, port_val);
    }
    else
    {
        detail = g_strdup_printf("Host: %s\nPort: %d\nError: %s%s%s",
                                 host_text, port_val,
                                 device_note ? device_note
                                             : "No response to dump_state",
                                 stderr_tail ? "\nLast rotctld stderr:\n" : "",
                                 stderr_tail ? stderr_tail : "");
        rot_pref_show_dialog(GTK_MESSAGE_ERROR,
                             _("rotctld connection failed"),
                             detail);
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    "rotctld test failed host=%s port=%d",
                    host_text, port_val);
    }

    if (spawned)
        rotctld_mgr_terminate(&mgr);

    g_free(detail);
    g_free(stderr_tail);
    g_free(device);
    g_free(device_note);
}

/* Update widgets from the currently selected row in the treeview */
static void update_widgets(rotor_conf_t * conf)
{
    /* configuration name */
    gtk_entry_set_text(GTK_ENTRY(name), conf->name);

    /* host */
    if (conf->host)
        gtk_entry_set_text(GTK_ENTRY(host), conf->host);

    /* port */
    if (conf->port > 0 && conf->port <= 65535)
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(port), conf->port);
    else
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(port), 4533); /* hamlib default? */

    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(autostart),
                                 conf->autostart);
    gtk_combo_box_set_active(GTK_COMBO_BOX(protocol), conf->protocol);
    if (conf->baud > 0)
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(baud), conf->baud);
    else
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(baud),
                                  rot_protocol_default_baud(conf->protocol));

    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(device_autopick),
                                 conf->device_autopick);
    gtk_entry_set_text(GTK_ENTRY(device_manual),
                       conf->device_manual ? conf->device_manual : "");
    rot_pref_scan_devices_async(conf->device);

    gtk_combo_box_set_active(GTK_COMBO_BOX(aztype), conf->aztype);

    /* az and el limits */
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(minaz), conf->minaz);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(maxaz), conf->maxaz);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(minel), conf->minel);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(maxel), conf->maxel);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(azstoppos), conf->azstoppos);
    gtk_combo_box_set_active(GTK_COMBO_BOX(axismode), conf->axis_mode);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(invert_az), conf->invert_az);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(invert_el), conf->invert_el);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(use_offset), conf->use_offset);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(az_offset), conf->az_offset);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(el_offset), conf->el_offset);
}

/* called when the user clicks on the CLEAR button */
static void clear_widgets()
{
    gtk_entry_set_text(GTK_ENTRY(name), "");
    gtk_entry_set_text(GTK_ENTRY(host), "127.0.0.1");
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(port), 4533);     /* hamlib default? */
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(autostart), TRUE);
    gtk_combo_box_set_active(GTK_COMBO_BOX(protocol), ROT_PROTOCOL_GS232B);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(baud),
                              rot_protocol_default_baud(ROT_PROTOCOL_GS232B));
    gtk_entry_set_text(GTK_ENTRY(device_manual), "");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(device_autopick), TRUE);
    rot_pref_scan_devices_async(NULL);
    gtk_combo_box_set_active(GTK_COMBO_BOX(aztype), ROT_AZ_TYPE_360);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(minaz), 0);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(maxaz), 360);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(minel), 0);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(maxel), 90);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(azstoppos), 0);
    gtk_combo_box_set_active(GTK_COMBO_BOX(axismode), ROT_AXIS_MODE_AZ_EL);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(invert_az), FALSE);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(invert_el), FALSE);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(use_offset), FALSE);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(az_offset), 0.0);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(el_offset), 0.0);
}

/*
 * This function is called when the contents of the name entry changes.
 * The primary purpose of this function is to check whether the char length
 * of the name is greater than zero, if yes enable the OK button of the dialog.
 */
static void name_changed(GtkWidget * widget, gpointer data)
{
    const gchar    *text;
    gchar          *entry, *end, *j;
    gint            len, pos;

    (void)data;

    /* step 1: ensure that only valid characters are entered
       (stolen from xlog, tnx pg4i)
     */
    entry = gtk_editable_get_chars(GTK_EDITABLE(widget), 0, -1);
    if ((len = g_utf8_strlen(entry, -1)) > 0)
    {
        end = entry + g_utf8_strlen(entry, -1);
        for (j = entry; j < end; ++j)
        {
            if (!gpredict_legal_char(*j))
            {
                gdk_display_beep(gdk_display_get_default());
                pos = gtk_editable_get_position(GTK_EDITABLE(widget));
                gtk_editable_delete_text(GTK_EDITABLE(widget), pos, pos + 1);
            }
        }
    }

    /* step 2: if name seems all right, enable OK button */
    text = gtk_entry_get_text(GTK_ENTRY(widget));

    if (g_utf8_strlen(text, -1) > 0)
    {
        gtk_dialog_set_response_sensitive(GTK_DIALOG(dialog),
                                          GTK_RESPONSE_OK, TRUE);
    }
    else
    {
        gtk_dialog_set_response_sensitive(GTK_DIALOG(dialog),
                                          GTK_RESPONSE_OK, FALSE);
    }
}

static void protocol_changed_cb(GtkComboBox * box, gpointer data)
{
    gint proto = gtk_combo_box_get_active(box);

    (void)data;

    if (baud)
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(baud),
                                  rot_protocol_default_baud(proto));
}

static void device_refresh_cb(GtkButton *button, gpointer data)
{
    gchar *current = NULL;

    (void)button;
    (void)data;

    if (device_combo)
        current = gtk_combo_box_text_get_active_text(
            GTK_COMBO_BOX_TEXT(device_combo));

    rot_pref_scan_devices_async(current);
    g_free(current);
}

static void device_autopick_toggled_cb(GtkToggleButton *button, gpointer data)
{
    gchar *current = NULL;

    (void)data;

    if (device_combo)
        current = gtk_combo_box_text_get_active_text(
            GTK_COMBO_BOX_TEXT(device_combo));

    rot_pref_update_device_combo(device_cache,
                                 current,
                                 gtk_toggle_button_get_active(button));
    g_free(current);
}

static void aztype_changed_cb(GtkComboBox * box, gpointer data)
{
    gint            type = gtk_combo_box_get_active(box);

    (void)data;

    switch (type)
    {
    case ROT_AZ_TYPE_360:
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(minaz), 0.0);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(maxaz), 360.0);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(azstoppos), 0.0);
        break;

    case ROT_AZ_TYPE_180:
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(minaz), -180.0);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(maxaz), +180.0);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(azstoppos), -180.0);
        break;

    default:
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%s: Invalid AZ rotator type."), __FILE__, __func__);
        break;
    }
}

static GtkWidget *create_editor_widgets(rotor_conf_t * conf)
{
    GtkWidget      *table;
    GtkWidget      *label;
    GtkWidget      *test_button;

    table = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(table), 5);
    gtk_grid_set_column_spacing(GTK_GRID(table), 5);
    gtk_grid_set_row_spacing(GTK_GRID(table), 5);

    /* Config name */
    label = gtk_label_new(_("Name"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 0, 1, 1);

    name = gtk_entry_new();
    gtk_entry_set_max_length(GTK_ENTRY(name), 25);
    gtk_widget_set_tooltip_text(name,
                                _("Enter a short name for this configuration, "
                                  " e.g. ROTOR-1.\n"
                                  "Allowed characters: 0..9, a..z, A..Z, - and _"));
    gtk_grid_attach(GTK_GRID(table), name, 1, 0, 3, 1);

    /* attach changed signal so that we can enable OK button when
       a proper name has been entered
     */
    g_signal_connect(name, "changed", G_CALLBACK(name_changed), NULL);

    /* Host */
    label = gtk_label_new(_("Host"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 1, 1, 1);

    host = gtk_entry_new();
    gtk_entry_set_max_length(GTK_ENTRY(host), 50);
    gtk_entry_set_text(GTK_ENTRY(host), "127.0.0.1");
    gtk_widget_set_tooltip_text(host,
                                _("Enter the host where rotctld is running. "
                                  "You can use both host name and IP address, "
                                  "e.g. 192.168.1.100\n\n"
                                  "If gpredict and rotctld are running on the "
                                  "same computer, use 127.0.0.1"));
    gtk_grid_attach(GTK_GRID(table), host, 1, 1, 3, 1); 

    /* port */
    label = gtk_label_new(_("Port"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 2, 1, 1);

    port = gtk_spin_button_new_with_range(1024, 65535, 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(port), 4533);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(port), 0);
    gtk_widget_set_tooltip_text(port,
                                _("Enter the port number where rotctld is "
                                  "listening. Default is 4533."));
    gtk_grid_attach(GTK_GRID(table), port, 1, 2, 1, 1);

    test_button = gtk_button_new_with_label(_("Test connection"));
    gtk_widget_set_tooltip_text(test_button,
                                _("Start rotctld if needed and query its status."));
    gtk_grid_attach(GTK_GRID(table), test_button, 2, 2, 2, 1);
    g_signal_connect(test_button, "clicked",
                     G_CALLBACK(rotctld_test_connection_cb), NULL);

    autostart = gtk_check_button_new_with_label(_("Auto start rotctld"));
    gtk_widget_set_tooltip_text(autostart,
                                _("Start rotctld automatically when connecting locally."));
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(autostart), TRUE);
    gtk_grid_attach(GTK_GRID(table), autostart, 1, 3, 3, 1);

    /* Protocol */
    label = gtk_label_new(_("Protocol"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 11, 1, 1);

    protocol = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(protocol),
                                   _("Yaesu GS-232B"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(protocol),
                                   _("SPID Rot1Prog"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(protocol),
                                   _("SPID Rot2Prog"));
    gtk_combo_box_set_active(GTK_COMBO_BOX(protocol), ROT_PROTOCOL_GS232B);
    gtk_grid_attach(GTK_GRID(table), protocol, 1, 4, 2, 1);
    g_signal_connect(G_OBJECT(protocol), "changed",
                     G_CALLBACK(protocol_changed_cb), NULL);

    /* Baud */
    label = gtk_label_new(_("Baud"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 5, 1, 1);

    baud = gtk_spin_button_new_with_range(300, 921600, 100);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(baud),
                              rot_protocol_default_baud(ROT_PROTOCOL_GS232B));
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(baud), 0);
    gtk_widget_set_tooltip_text(baud, _("Serial baud rate for rotctld."));
    gtk_grid_attach(GTK_GRID(table), baud, 1, 5, 1, 1);

    /* Device */
    label = gtk_label_new(_("Device"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 6, 1, 1);

    device_combo = gtk_combo_box_text_new();
    gtk_widget_set_tooltip_text(device_combo,
                                _("Select the serial device for your rotor."));
    gtk_grid_attach(GTK_GRID(table), device_combo, 1, 6, 2, 1);

    device_refresh = gtk_button_new_with_label(_("Refresh"));
    gtk_grid_attach(GTK_GRID(table), device_refresh, 3, 6, 1, 1);
    g_signal_connect(device_refresh, "clicked",
                     G_CALLBACK(device_refresh_cb), NULL);

    /* Manual path */
    label = gtk_label_new(_("Manual path"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 7, 1, 1);

    device_manual = gtk_entry_new();
    gtk_widget_set_tooltip_text(device_manual,
                                _("Override the device path (advanced)."));
    gtk_grid_attach(GTK_GRID(table), device_manual, 1, 7, 3, 1);

    device_autopick = gtk_check_button_new_with_label(_("Auto-pick best match"));
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(device_autopick), TRUE);
    gtk_grid_attach(GTK_GRID(table), device_autopick, 1, 8, 2, 1);
    g_signal_connect(device_autopick, "toggled",
                     G_CALLBACK(device_autopick_toggled_cb), NULL);

    device_status = gtk_label_new("");
    g_object_set(device_status, "xalign", 0.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), device_status, 1, 9, 3, 1);

    gtk_grid_attach(GTK_GRID(table),
                    gtk_separator_new(GTK_ORIENTATION_HORIZONTAL),
                    0, 10, 4, 1);

    /* Az-type */
    label = gtk_label_new(_("Az type"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 4, 1, 1);

    aztype = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(aztype),
                                   "0\302\260 \342\206\222 180\302\260 \342\206\222 360\302\260");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(aztype),
                                   "-180\302\260 \342\206\222 0\302\260 \342\206\222 +180\302\260");
    gtk_combo_box_set_active(GTK_COMBO_BOX(aztype), 0);
    gtk_widget_set_tooltip_text(aztype,
                                _("Select your azimuth range here. Note that "
                                  "gpredict assumes that 0\302\260 is at North "
                                  "and + direction is clockwise for both types"));
    gtk_grid_attach(GTK_GRID(table), aztype, 1, 11, 2, 1);
    g_signal_connect(G_OBJECT(aztype), "changed",
                     G_CALLBACK(aztype_changed_cb), NULL);

    /* Az and El limits */
    label = gtk_label_new(_(" Min Az"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 12, 1, 1);
    minaz = gtk_spin_button_new_with_range(-200, 100, 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(minaz), 0);
    gtk_spin_button_set_numeric(GTK_SPIN_BUTTON(minaz), TRUE);
    gtk_spin_button_set_wrap(GTK_SPIN_BUTTON(minaz), FALSE);
    gtk_grid_attach(GTK_GRID(table), minaz, 1, 12, 1, 1);

    label = gtk_label_new(_(" Max Az"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 2, 12, 1, 1);
    maxaz = gtk_spin_button_new_with_range(0, 450, 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(maxaz), 360);
    gtk_spin_button_set_numeric(GTK_SPIN_BUTTON(maxaz), TRUE);
    gtk_spin_button_set_wrap(GTK_SPIN_BUTTON(maxaz), FALSE);
    gtk_grid_attach(GTK_GRID(table), maxaz, 3, 12, 1, 1);

    label = gtk_label_new(_(" Min El"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 13, 1, 1);
    minel = gtk_spin_button_new_with_range(-10, 180, 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(minel), 0);
    gtk_spin_button_set_numeric(GTK_SPIN_BUTTON(minel), TRUE);
    gtk_spin_button_set_wrap(GTK_SPIN_BUTTON(minel), FALSE);
    gtk_grid_attach(GTK_GRID(table), minel, 1, 13, 1, 1);

    label = gtk_label_new(_(" Max El"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 2, 13, 1, 1);
    maxel = gtk_spin_button_new_with_range(-10, 180, 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(maxel), 90);
    gtk_spin_button_set_numeric(GTK_SPIN_BUTTON(maxel), TRUE);
    gtk_spin_button_set_wrap(GTK_SPIN_BUTTON(maxel), FALSE);
    gtk_grid_attach(GTK_GRID(table), maxel, 3, 13, 1, 1);

    label = gtk_label_new(_(" Azimuth end stop position"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 1, 14, 2, 1);
    azstoppos = gtk_spin_button_new_with_range(-180, 360, 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(azstoppos), 0);
    gtk_spin_button_set_numeric(GTK_SPIN_BUTTON(azstoppos), TRUE);
    gtk_spin_button_set_wrap(GTK_SPIN_BUTTON(azstoppos), FALSE);
    gtk_widget_set_tooltip_text(azstoppos,
                                _("Set the position of the azimuth end stop "
                                  "here, where 0\302\260 is at North, "
                                  "-180\302\260 is south, etc. "
                                  "The default for a 0\302\260 \342\206\222 "
                                  "180\302\260 \342\206\222 360\302\260 rotor "
                                  "is 0\302\260, and the default for a "
                                  "-180\302\260 \342\206\222 0\302\260 "
                                  "\342\206\222 +180\302\260 rotor is -180\302\260."));
    gtk_grid_attach(GTK_GRID(table), azstoppos, 3, 14, 1, 1);

    /* Axis mode */
    label = gtk_label_new(_("Axis mode"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 15, 1, 1);

    axismode = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(axismode),
                                   _("Azimuth + Elevation"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(axismode),
                                   _("Azimuth only"));
    gtk_combo_box_set_active(GTK_COMBO_BOX(axismode), ROT_AXIS_MODE_AZ_EL);
    gtk_widget_set_tooltip_text(axismode,
                                _("Select whether this rotor supports both azimuth and elevation."));
    gtk_grid_attach(GTK_GRID(table), axismode, 1, 15, 2, 1);

    /* Axis inversion */
    label = gtk_label_new(_("Invert"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 16, 1, 1);

    invert_az = gtk_check_button_new_with_label(_("Az"));
    gtk_grid_attach(GTK_GRID(table), invert_az, 1, 16, 1, 1);
    invert_el = gtk_check_button_new_with_label(_("El"));
    gtk_grid_attach(GTK_GRID(table), invert_el, 2, 16, 1, 1);

    /* Offsets */
    label = gtk_label_new(_("Offsets"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 17, 1, 1);

    use_offset = gtk_check_button_new_with_label(_("Enable"));
    gtk_grid_attach(GTK_GRID(table), use_offset, 1, 17, 1, 1);

    label = gtk_label_new(_(" Az offset"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 18, 1, 1);
    az_offset = gtk_spin_button_new_with_range(-360, 360, 0.1);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(az_offset), 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(az_offset), 0.0);
    gtk_grid_attach(GTK_GRID(table), az_offset, 1, 18, 1, 1);
    label = gtk_label_new(_("deg"));
    g_object_set(label, "xalign", 0.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 2, 18, 1, 1);

    label = gtk_label_new(_(" El offset"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 19, 1, 1);
    el_offset = gtk_spin_button_new_with_range(-90, 90, 0.1);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(el_offset), 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(el_offset), 0.0);
    gtk_grid_attach(GTK_GRID(table), el_offset, 1, 19, 1, 1);
    label = gtk_label_new(_("deg"));
    g_object_set(label, "xalign", 0.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 2, 19, 1, 1);

    if (conf->name != NULL)
        update_widgets(conf);
    else
        rot_pref_scan_devices_async(NULL);

    gtk_widget_show_all(table);

    return table;
}

/* Called when the user clicks the OK button */
static gboolean apply_changes(rotor_conf_t * conf)
{
    /* name */
    if (conf->name)
        g_free(conf->name);

    conf->name = g_strdup(gtk_entry_get_text(GTK_ENTRY(name)));

    /* host */
    if (conf->host)
        g_free(conf->host);

    conf->host = g_strdup(gtk_entry_get_text(GTK_ENTRY(host)));

    /* port */
    conf->port = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(port));

    conf->autostart = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(autostart));
    conf->protocol = gtk_combo_box_get_active(GTK_COMBO_BOX(protocol));
    conf->baud = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(baud));
    if (conf->baud <= 0)
        conf->baud = rot_protocol_default_baud(conf->protocol);

    if (conf->device)
        g_free(conf->device);
    conf->device = gtk_combo_box_text_get_active_text(
        GTK_COMBO_BOX_TEXT(device_combo));

    if (conf->device_manual)
        g_free(conf->device_manual);
    {
        const gchar *manual_text =
            gtk_entry_get_text(GTK_ENTRY(device_manual));
        conf->device_manual =
            (manual_text && *manual_text) ? g_strdup(manual_text) : NULL;
    }

    conf->device_autopick =
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(device_autopick));

    /* az type */
    conf->aztype = gtk_combo_box_get_active(GTK_COMBO_BOX(aztype));

    /* az and el ranges */
    conf->minaz = gtk_spin_button_get_value(GTK_SPIN_BUTTON(minaz));
    conf->maxaz = gtk_spin_button_get_value(GTK_SPIN_BUTTON(maxaz));
    conf->minel = gtk_spin_button_get_value(GTK_SPIN_BUTTON(minel));
    conf->maxel = gtk_spin_button_get_value(GTK_SPIN_BUTTON(maxel));

    /* az stop position */
    conf->azstoppos = gtk_spin_button_get_value(GTK_SPIN_BUTTON(azstoppos));

    /* axis mode */
    conf->axis_mode = gtk_combo_box_get_active(GTK_COMBO_BOX(axismode));

    /* axis inversion */
    conf->invert_az = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(invert_az));
    conf->invert_el = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(invert_el));

    /* offsets */
    conf->use_offset = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(use_offset));
    conf->az_offset = gtk_spin_button_get_value(GTK_SPIN_BUTTON(az_offset));
    conf->el_offset = gtk_spin_button_get_value(GTK_SPIN_BUTTON(el_offset));

    return TRUE;
}

/**
 * Add or edit a rotor configuration.
 *
 * @param conf Pointer to a rotator configuration.
 *
 * If conf->name is not NULL the widgets will be populated with the data.
 */
void sat_pref_rot_editor_run(rotor_conf_t * conf)
{
    gint            response;
    gboolean        finished = FALSE;

    /* create dialog and add contents */
    dialog = gtk_dialog_new_with_buttons(_("Edit rotator configuration"),
                                         GTK_WINDOW(window),
                                         GTK_DIALOG_MODAL |
                                         GTK_DIALOG_DESTROY_WITH_PARENT,
                                         "_Clear", GTK_RESPONSE_REJECT,
                                         "_Cancel", GTK_RESPONSE_CANCEL,
                                         "_Ok", GTK_RESPONSE_OK,
                                         NULL);

    /* disable OK button to begin with */
    gtk_dialog_set_response_sensitive(GTK_DIALOG(dialog),
                                      GTK_RESPONSE_OK, FALSE);

    gtk_container_add(GTK_CONTAINER
                      (gtk_dialog_get_content_area(GTK_DIALOG(dialog))),
                      create_editor_widgets(conf));

    /* this hacky-thing is to keep the dialog running in case the
       CLEAR button is plressed. OK and CANCEL will exit the loop
     */
    while (!finished)
    {
        response = gtk_dialog_run(GTK_DIALOG(dialog));

        switch (response)
        {
        case GTK_RESPONSE_OK:
            if (apply_changes(conf))
                finished = TRUE;
            else
                finished = FALSE;
            break;

        case GTK_RESPONSE_REJECT:
            /* CLEAR */
            clear_widgets();
            break;

        default:
            /* Everything else is considered CANCEL */
            finished = TRUE;
            break;
        }
    }

    gtk_widget_destroy(dialog);

    if (device_cache)
    {
        gp_serial_free_candidates(device_cache);
        device_cache = NULL;
    }
    device_scan_in_progress = FALSE;
}
