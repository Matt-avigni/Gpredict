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
#include "radio-conf.h"
#include "sat-cfg.h"
#include "sat-log.h"
#include "sat-pref-rig-editor.h"
#include "serial-ports.h"


extern GtkWidget *window;       /* dialog window defined in sat-pref.c */
static GtkWidget *dialog;       /* dialog window */
static GtkWidget *name;         /* config name */
static GtkWidget *host;         /* host */
static GtkWidget *port;         /* port number */
static GtkWidget *type;         /* rig type */
static GtkWidget *radio_model;  /* radio model */
static GtkWidget *radio_mode;   /* radio mode */
static GtkWidget *ptt;          /* PTT */
static GtkWidget *vfo;          /* VFO Up/Down selector */
static GtkWidget *lo;           /* local oscillator of downconverter */
static GtkWidget *loup;         /* local oscillator of upconverter */
static GtkWidget *sigaos;       /* AOS signalling */
static GtkWidget *siglos;       /* LOS signalling */
static GtkWidget *autostart;    /* auto-start rigctld */
static GtkWidget *rigctld_conn_type; /* rigctld connection type */
static GtkWidget *rigctld_auto_power_on; /* rigctld auto power-on */
static GtkWidget *rigctld_path; /* rigctld path */
static GtkWidget *rigctld_model; /* rigctld model */
static GtkWidget *rigctld_device_label; /* rigctld device label */
static GtkWidget *rigctld_device; /* rigctld device */
static GtkWidget *rigctld_device_find; /* rigctld device finder */
static GtkWidget *rigctld_baud_label; /* rigctld baud label */
static GtkWidget *rigctld_baud; /* rigctld baud */
static GtkWidget *rigctld_civaddr; /* rigctld CI-V address */
static GtkWidget *rigctld_extra_args; /* rigctld extra args */
static gint rigctld_model_custom = 0; /* remember custom rig model */
static radio_model_t last_radio_model = RADIO_MODEL_OTHER;

static void rig_pref_show_dialog(GtkMessageType type,
                                 const gchar *primary,
                                 const gchar *secondary);

static gboolean rigctld_conn_is_tcp(void)
{
    if (rigctld_conn_type == NULL)
        return FALSE;

    return gtk_combo_box_get_active(GTK_COMBO_BOX(rigctld_conn_type)) ==
        RIGCTLD_CONN_TCP;
}

static gboolean radio_model_has_preset(void)
{
    if (radio_model == NULL)
        return FALSE;

    return radio_model_to_hamlib_model(
        gtk_combo_box_get_active(GTK_COMBO_BOX(radio_model))) > 0;
}

static void update_rigctld_model_sensitivity(gboolean enabled)
{
    gboolean allow_edit = enabled && !radio_model_has_preset();

    if (rigctld_model != NULL)
        gtk_widget_set_sensitive(rigctld_model, allow_edit);
}

static void update_rigctld_connection_ui(gboolean enabled)
{
    gboolean is_tcp = rigctld_conn_is_tcp();

    if (rigctld_device_label != NULL)
    {
        gtk_label_set_text(GTK_LABEL(rigctld_device_label),
                           is_tcp ? _("Rig address") : _("Serial device"));
    }

    if (rigctld_device != NULL)
    {
        gtk_widget_set_tooltip_text(
            rigctld_device,
            is_tcp ? _("Rig TCP address for rigctld (host:port).")
                   : _("Serial device for rigctld (e.g. /dev/ttyUSB0)."));
    }
    if (rigctld_device_find != NULL)
        gtk_widget_set_sensitive(rigctld_device_find, enabled && !is_tcp);

    if (rigctld_baud_label != NULL)
        gtk_widget_set_sensitive(rigctld_baud_label, enabled && !is_tcp);
    if (rigctld_baud != NULL)
        gtk_widget_set_sensitive(rigctld_baud, enabled && !is_tcp);
}

static gboolean rig_pref_is_preferred_device(const gchar *path)
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
                (g_strrstr(lower, "wch") != NULL) ||
                (g_strrstr(lower, "ftdi") != NULL);
    }
    g_free(lower);
    return match;
}

static gboolean rig_pref_list_contains(GSList *list, const gchar *item)
{
    if (item == NULL)
        return FALSE;

    for (GSList *iter = list; iter != NULL; iter = iter->next)
    {
        if (g_strcmp0(iter->data, item) == 0)
            return TRUE;
    }
    return FALSE;
}

static gchar *rig_pref_pick_best_device(GSList *list, const gchar *current)
{
    if (list == NULL)
        return NULL;

    if (list->next == NULL)
        return g_strdup(list->data);

    if (current && rig_pref_list_contains(list, current))
    {
        gboolean current_preferred = rig_pref_is_preferred_device(current);
        gboolean have_preferred = FALSE;

        for (GSList *iter = list; iter != NULL; iter = iter->next)
        {
            if (rig_pref_is_preferred_device(iter->data))
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
        if (rig_pref_is_preferred_device(iter->data))
            return g_strdup(iter->data);
    }

    return g_strdup(list->data);
}

static void rigctld_find_port_cb(GtkButton *button, gpointer data)
{
    const gchar *current = NULL;
    GSList *candidates = NULL;
    gchar *picked = NULL;
    guint count = 0;

    (void)button;
    (void)data;

    if (rigctld_conn_is_tcp())
    {
        rig_pref_show_dialog(GTK_MESSAGE_INFO,
                             _("Find port is only available for serial rigs."),
                             NULL);
        return;
    }

    if (rigctld_device)
        current = gtk_entry_get_text(GTK_ENTRY(rigctld_device));

    candidates = gp_serial_list_candidates();
    count = g_slist_length(candidates);
    sat_log_log(SAT_LOG_LEVEL_INFO,
                "rigctld find port: candidates=%u", count);

    if (candidates == NULL)
    {
        rig_pref_show_dialog(GTK_MESSAGE_WARNING,
                             _("No serial ports found."),
                             NULL);
        return;
    }

    picked = rig_pref_pick_best_device(candidates, current);
    if (picked)
    {
        gtk_entry_set_text(GTK_ENTRY(rigctld_device), picked);
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "rigctld find port selected=%s", picked);
    }
    else
        rig_pref_show_dialog(GTK_MESSAGE_WARNING,
                             _("No serial ports found."),
                             NULL);

    g_free(picked);
    gp_serial_free_candidates(candidates);
}

static void apply_preset_rigctld_defaults(radio_model_t model,
                                          gboolean force)
{
    rigctld_preset_defaults_t preset;

    if (!radio_model_get_rigctld_defaults(model, &preset))
        return;

    if (host != NULL)
    {
        const gchar *current = gtk_entry_get_text(GTK_ENTRY(host));
        if (force || current == NULL || *current == '\0')
            gtk_entry_set_text(GTK_ENTRY(host),
                               preset.host ? preset.host : "");
    }

    if (port != NULL)
    {
        gint value = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(port));
        if (force || value <= 0)
            gtk_spin_button_set_value(GTK_SPIN_BUTTON(port), preset.port);
    }

    if (rigctld_conn_type != NULL && force)
        gtk_combo_box_set_active(GTK_COMBO_BOX(rigctld_conn_type), preset.conn);

    if (rigctld_device != NULL && force)
        gtk_entry_set_text(GTK_ENTRY(rigctld_device), "");

    if (rigctld_baud != NULL)
    {
        gint value =
            gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(rigctld_baud));
        if (force || value <= 0)
            gtk_spin_button_set_value(GTK_SPIN_BUTTON(rigctld_baud),
                                      preset.baud);
    }

    if (rigctld_civaddr != NULL)
    {
        const gchar *current = gtk_entry_get_text(GTK_ENTRY(rigctld_civaddr));
        if (force || current == NULL || *current == '\0')
            gtk_entry_set_text(GTK_ENTRY(rigctld_civaddr),
                               preset.civaddr ? preset.civaddr : "");
    }
}

static void update_autostart_sensitivity(gboolean enabled)
{
    gtk_widget_set_sensitive(rigctld_conn_type, enabled);
    gtk_widget_set_sensitive(rigctld_auto_power_on, enabled);
    gtk_widget_set_sensitive(rigctld_path, enabled);
    update_rigctld_model_sensitivity(enabled);
    gtk_widget_set_sensitive(rigctld_device, enabled);
    gtk_widget_set_sensitive(rigctld_civaddr, enabled);
    gtk_widget_set_sensitive(rigctld_extra_args, enabled);
    update_rigctld_connection_ui(enabled);
}

static void rigctld_model_changed(GtkSpinButton *spin, gpointer data)
{
    (void)data;

    if (radio_model == NULL || spin == NULL)
        return;

    if (gtk_combo_box_get_active(GTK_COMBO_BOX(radio_model)) ==
        RADIO_MODEL_OTHER)
    {
        rigctld_model_custom =
            gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(spin));
    }
}

static void rig_pref_show_dialog(GtkMessageType type,
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

static gboolean socket_send_all(GSocket *sock, const gchar *data, gsize len,
                                GError **error)
{
    gsize offset = 0;

    while (offset < len)
    {
        gssize sent = g_socket_send(sock, data + offset, len - offset, NULL,
                                    error);
        if (sent < 0)
            return FALSE;
        offset += (gsize)sent;
    }

    return TRUE;
}

static gboolean socket_read_reply(GSocket *sock, gchar *buffer, gsize size,
                                  gint timeout_ms, GError **error)
{
    gsize offset = 0;

    g_socket_set_blocking(sock, FALSE);
    if (!g_socket_condition_timed_wait(sock, G_IO_IN,
                                       (gint64) timeout_ms * 1000,
                                       NULL, error))
    {
        if (error && *error &&
            g_error_matches(*error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT))
        {
            g_clear_error(error);
            g_set_error(error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT,
                        "Timed out waiting for reply");
        }
        return FALSE;
    }

    while (offset < size - 1)
    {
        gssize n = g_socket_receive(sock, buffer + offset,
                                    size - 1 - offset, NULL, error);
        if (n > 0)
        {
            offset += (gsize)n;
            continue;
        }

        if (n == 0)
            break;

        if (error && *error &&
            g_error_matches(*error, G_IO_ERROR, G_IO_ERROR_WOULD_BLOCK))
        {
            g_clear_error(error);
            break;
        }
        return FALSE;
    }

    buffer[offset] = '\0';
    return offset > 0;
}

static gboolean rigctld_test_query(const gchar *host, gint port,
                                   gchar **reply_out, gchar **error_out)
{
    GSocketClient *client = NULL;
    GSocketConnection *conn = NULL;
    GSocket *sock = NULL;
    GError *error = NULL;
    gchar buffer[256];
    gboolean ok = FALSE;
    const gchar *cmd = "f\n";

    if (reply_out)
        *reply_out = NULL;
    if (error_out)
        *error_out = NULL;

    if (host == NULL || *host == '\0' || port <= 0)
    {
        if (error_out)
            *error_out = g_strdup("Missing host or port");
        return FALSE;
    }

    client = g_socket_client_new();
    g_socket_client_set_timeout(client, 3);
    conn = g_socket_client_connect_to_host(client, host, port, NULL, &error);
    if (conn == NULL)
    {
        if (error_out)
            *error_out = g_strdup(error ? error->message : "Connect failed");
        g_clear_error(&error);
        g_object_unref(client);
        return FALSE;
    }

    sock = g_socket_connection_get_socket(conn);
    g_socket_set_blocking(sock, TRUE);
    if (!socket_send_all(sock, cmd, strlen(cmd), &error))
    {
        if (error_out)
            *error_out = g_strdup(error ? error->message : "Send failed");
        g_clear_error(&error);
        g_object_unref(conn);
        g_object_unref(client);
        return FALSE;
    }

    if (!socket_read_reply(sock, buffer, sizeof(buffer), 1500, &error))
    {
        if (error_out)
            *error_out = g_strdup(error ? error->message : "No reply");
        g_clear_error(&error);
        g_object_unref(conn);
        g_object_unref(client);
        return FALSE;
    }

    g_strchomp(buffer);
    if (reply_out)
        *reply_out = g_strdup(buffer);

    if (g_str_has_prefix(buffer, "RPRT"))
        ok = FALSE;
    else
        ok = TRUE;

    g_object_unref(conn);
    g_object_unref(client);
    return ok;
}

static void rigctld_test_connection_cb(GtkButton *button, gpointer data)
{
    const gchar *host_text = gtk_entry_get_text(GTK_ENTRY(host));
    gint port_val = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(port));
    gchar *reply = NULL;
    gchar *err = NULL;
    gchar *detail = NULL;
    gboolean ok;

    (void)button;
    (void)data;

    ok = rigctld_test_query(host_text, port_val, &reply, &err);
    if (ok)
    {
        detail = g_strdup_printf("Host: %s\nPort: %d\nReply: %s",
                                 host_text, port_val,
                                 reply ? reply : "(none)");
        rig_pref_show_dialog(GTK_MESSAGE_INFO,
                             _("rigctld connection OK"),
                             detail);
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "rigctld test ok host=%s port=%d reply=%s",
                    host_text, port_val, reply ? reply : "(none)");
    }
    else
    {
        detail = g_strdup_printf("Host: %s\nPort: %d\nError: %s\nReply: %s",
                                 host_text, port_val,
                                 err ? err : "unknown",
                                 reply ? reply : "(none)");
        rig_pref_show_dialog(GTK_MESSAGE_ERROR,
                             _("rigctld connection failed"),
                             detail);
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    "rigctld test failed host=%s port=%d error=%s reply=%s",
                    host_text, port_val,
                    err ? err : "unknown",
                    reply ? reply : "(none)");
    }

    g_free(detail);
    g_free(reply);
    g_free(err);
}


static void clear_widgets()
{
    gtk_entry_set_text(GTK_ENTRY(name), "");
    gtk_entry_set_text(GTK_ENTRY(host), "127.0.0.1");
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(port), 4532);     /* hamlib default? */
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(lo), 0);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(loup), 0);
    gtk_combo_box_set_active(GTK_COMBO_BOX(type), RIG_TYPE_RX);
    gtk_combo_box_set_active(GTK_COMBO_BOX(radio_model), RADIO_MODEL_OTHER);
    rigctld_model_custom = 0;
    last_radio_model = RADIO_MODEL_OTHER;
    gtk_combo_box_set_active(GTK_COMBO_BOX(radio_mode), RADIO_MODE_SIMPLEX);
    gtk_combo_box_set_active(GTK_COMBO_BOX(ptt), PTT_TYPE_NONE);
    gtk_combo_box_set_active(GTK_COMBO_BOX(vfo), 0);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ptt), FALSE);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(sigaos), FALSE);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(siglos), FALSE);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(autostart), TRUE);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(rigctld_auto_power_on),
                                 FALSE);
    gtk_entry_set_text(GTK_ENTRY(rigctld_path), "");
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(rigctld_model), 0);
    gtk_combo_box_set_active(GTK_COMBO_BOX(rigctld_conn_type),
                             RIGCTLD_CONN_SERIAL);
    gtk_entry_set_text(GTK_ENTRY(rigctld_device), "");
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(rigctld_baud), 0);
    gtk_entry_set_text(GTK_ENTRY(rigctld_civaddr), "");
    gtk_entry_set_text(GTK_ENTRY(rigctld_extra_args), "");
    update_autostart_sensitivity(TRUE);
}

static void update_widgets(radio_conf_t * conf)
{
    /* configuration name */
    gtk_entry_set_text(GTK_ENTRY(name), conf->name);

    /* host name */
    if (conf->host)
        gtk_entry_set_text(GTK_ENTRY(host), conf->host);

    /* port */
    if (conf->port > 1023)
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(port), conf->port);
    else
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(port), 4532); /* hamlib default? */

    /* radio type */
    gtk_combo_box_set_active(GTK_COMBO_BOX(type), conf->type);

    /* radio model */
    gtk_combo_box_set_active(GTK_COMBO_BOX(radio_model), conf->radio_model);
    last_radio_model = conf->radio_model;
    if (conf->radio_model == RADIO_MODEL_OTHER)
        rigctld_model_custom = conf->rigctld_model;
    else
        rigctld_model_custom = 0;

    /* radio mode */
    gtk_combo_box_set_active(GTK_COMBO_BOX(radio_mode), conf->radio_mode);

    /* ptt */
    gtk_combo_box_set_active(GTK_COMBO_BOX(ptt), conf->ptt);

    /* vfo up/down */
    if (conf->uplink_vfo == VFO_MAIN && conf->downlink_vfo == VFO_SUB)
        gtk_combo_box_set_active(GTK_COMBO_BOX(vfo), 1);
    else if (conf->uplink_vfo == VFO_SUB && conf->downlink_vfo == VFO_MAIN)
        gtk_combo_box_set_active(GTK_COMBO_BOX(vfo), 2);
    else if (conf->uplink_vfo == VFO_A && conf->downlink_vfo == VFO_B)
        gtk_combo_box_set_active(GTK_COMBO_BOX(vfo), 3);
    else if (conf->uplink_vfo == VFO_B && conf->downlink_vfo == VFO_A)
        gtk_combo_box_set_active(GTK_COMBO_BOX(vfo), 4);
    else
        gtk_combo_box_set_active(GTK_COMBO_BOX(vfo), 0);

    /* lo down in MHz */
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(lo), conf->lo / 1000000.0);

    /* lo up in MHz */
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(loup), conf->loup / 1000000.0);

    /* AOS / LOS signalling */
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(sigaos), conf->signal_aos);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(siglos), conf->signal_los);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(autostart),
                                 conf->rigctld_autostart);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(rigctld_auto_power_on),
                                 conf->rigctld_auto_power_on);
    gtk_entry_set_text(GTK_ENTRY(rigctld_path), "");
    if (conf->rigctld_path)
        gtk_entry_set_text(GTK_ENTRY(rigctld_path), conf->rigctld_path);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(rigctld_model),
                              conf->rigctld_model);
    if (conf->rigctld_model <= 0)
    {
        gint preset = radio_model_to_hamlib_model(conf->radio_model);

        /* Prefer preset model ids when config lacks an explicit rig model. */
        if (preset > 0)
            gtk_spin_button_set_value(GTK_SPIN_BUTTON(rigctld_model), preset);
    }
    gtk_combo_box_set_active(GTK_COMBO_BOX(rigctld_conn_type),
                             conf->rigctld_conn);
    gtk_entry_set_text(GTK_ENTRY(rigctld_device), "");
    if (conf->rigctld_device)
        gtk_entry_set_text(GTK_ENTRY(rigctld_device), conf->rigctld_device);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(rigctld_baud),
                              conf->rigctld_baud);
    gtk_entry_set_text(GTK_ENTRY(rigctld_civaddr), "");
    if (conf->rigctld_civaddr)
        gtk_entry_set_text(GTK_ENTRY(rigctld_civaddr), conf->rigctld_civaddr);
    gtk_entry_set_text(GTK_ENTRY(rigctld_extra_args), "");
    if (conf->rigctld_extra_args)
        gtk_entry_set_text(GTK_ENTRY(rigctld_extra_args),
                           conf->rigctld_extra_args);
    apply_preset_rigctld_defaults(conf->radio_model, FALSE);
    update_autostart_sensitivity(conf->rigctld_autostart);
}

/*
 * Manage VFO changed signals.
 * Widget is the GtkComboBox that received the signal.
 *  
 * This function is called when the user selects a new VFO up/down combination.
 */
static void vfo_changed(GtkWidget * widget, gpointer data)
{
    (void)data;

    if (gtk_combo_box_get_active(GTK_COMBO_BOX(widget)) == 0 &&
        gtk_combo_box_get_active(GTK_COMBO_BOX(radio_mode)) !=
        RADIO_MODE_SIMPLEX)
        gtk_combo_box_set_active(GTK_COMBO_BOX(widget), 1);
}

/*
 * Manage ptt type changed signals.
 * Widget is the GtkComboBox that received the signal.
 *  
 * This function is called when the user selects a new ptt type.
 */
static void ptt_changed(GtkWidget * widget, gpointer data)
{
    (void)data;

    if (gtk_combo_box_get_active(GTK_COMBO_BOX(widget)) == PTT_TYPE_NONE &&
        gtk_combo_box_get_active(GTK_COMBO_BOX(type)) == RIG_TYPE_TRX)
    {
        /* not good, we need to have PTT for this type */
        gtk_combo_box_set_active(GTK_COMBO_BOX(widget), PTT_TYPE_CAT);
    }
}

/*
 * Manage name changes.
 *
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

/*
 * Manage rig type changed signals.
 * widget os the GtkComboBox that received the signal.
 *  
 * This function is called when the user selects a new radio type.
 */
static void type_changed(GtkWidget * widget, gpointer data)
{
    (void)data;

    /* PTT consistency */
    if (gtk_combo_box_get_active(GTK_COMBO_BOX(widget)) == RIG_TYPE_TRX)
    {
        if (gtk_combo_box_get_active(GTK_COMBO_BOX(ptt)) == PTT_TYPE_NONE)
        {
            gtk_combo_box_set_active(GTK_COMBO_BOX(ptt), PTT_TYPE_CAT);
        }
    }

    if ((gtk_combo_box_get_active(GTK_COMBO_BOX(widget)) ==
         RIG_TYPE_TOGGLE_AUTO) ||
        (gtk_combo_box_get_active(GTK_COMBO_BOX(widget)) ==
         RIG_TYPE_TOGGLE_MAN))
    {
        gtk_combo_box_set_active(GTK_COMBO_BOX(ptt), PTT_TYPE_CAT);
    }


    if (gtk_combo_box_get_active(GTK_COMBO_BOX(vfo)) == 0 &&
        gtk_combo_box_get_active(GTK_COMBO_BOX(radio_mode)) !=
        RADIO_MODE_SIMPLEX)
        gtk_combo_box_set_active(GTK_COMBO_BOX(vfo), 1);
}

static void radio_mode_changed(GtkWidget * widget, gpointer data)
{
    (void)data;

    if (gtk_combo_box_get_active(GTK_COMBO_BOX(widget)) !=
        RADIO_MODE_SIMPLEX &&
        gtk_combo_box_get_active(GTK_COMBO_BOX(vfo)) == 0)
        gtk_combo_box_set_active(GTK_COMBO_BOX(vfo), 1);
}

static void autostart_toggled(GtkToggleButton *button, gpointer data)
{
    (void)data;
    update_autostart_sensitivity(gtk_toggle_button_get_active(button));
}

static void rigctld_conn_changed(GtkComboBox *box, gpointer data)
{
    gboolean enabled;

    (void)box;
    (void)data;

    enabled = autostart != NULL &&
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(autostart));
    update_rigctld_connection_ui(enabled);
}

static void radio_model_changed(GtkComboBox *box, gpointer data)
{
    radio_model_t model;
    gint preset;
    gboolean enabled;

    (void)data;

    model = gtk_combo_box_get_active(GTK_COMBO_BOX(box));
    preset = radio_model_to_hamlib_model(model);
    if (rigctld_model != NULL)
    {
        if (last_radio_model == RADIO_MODEL_OTHER)
        {
            rigctld_model_custom =
                gtk_spin_button_get_value_as_int(
                    GTK_SPIN_BUTTON(rigctld_model));
        }

        if (preset > 0)
        {
            gtk_spin_button_set_value(GTK_SPIN_BUTTON(rigctld_model), preset);
        }
        else if (rigctld_model_custom > 0)
        {
            gtk_spin_button_set_value(GTK_SPIN_BUTTON(rigctld_model),
                                      rigctld_model_custom);
        }
    }

    last_radio_model = model;
    if (preset > 0)
        apply_preset_rigctld_defaults(model, TRUE);
    enabled = autostart != NULL &&
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(autostart));
    update_rigctld_model_sensitivity(enabled);
}

static void rig_pref_force_toplevel_resize(GtkWidget *widget)
{
    GtkWidget *toplevel;

    if (widget == NULL)
        return;

    toplevel = gtk_widget_get_toplevel(widget);
    if (!GTK_IS_WINDOW(toplevel))
        return;

    gtk_widget_set_size_request(toplevel, -1, -1);
    gtk_widget_queue_resize(toplevel);
    gtk_window_resize(GTK_WINDOW(toplevel), 1, 1);
}

static void advanced_expander_notify(GObject *obj, GParamSpec *pspec,
                                     gpointer data)
{
    GtkExpander *expander = GTK_EXPANDER(obj);

    (void)pspec;
    (void)data;

    if (!gtk_expander_get_expanded(expander))
        rig_pref_force_toplevel_resize(GTK_WIDGET(expander));
}

static GtkWidget *create_editor_widgets(radio_conf_t * conf)
{
    GtkWidget      *table;
    GtkWidget      *advanced_table;
    GtkWidget      *label;
    GtkWidget      *test_button;
    GtkWidget      *vbox;
    GtkWidget      *advanced_expander;

    vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);

    table = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(table), 5);
    gtk_grid_set_column_homogeneous(GTK_GRID(table), FALSE);
    gtk_grid_set_row_homogeneous(GTK_GRID(table), FALSE);
    gtk_grid_set_column_spacing(GTK_GRID(table), 5);
    gtk_grid_set_row_spacing(GTK_GRID(table), 5);

    /* Config name */
    label = gtk_label_new(_("Name"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 0, 1, 1);

    name = gtk_entry_new();
    gtk_entry_set_max_length(GTK_ENTRY(name), 25);
    g_signal_connect(name, "changed", G_CALLBACK(name_changed), NULL);
    gtk_widget_set_tooltip_text(name,
                                _("Enter a short name for this configuration, "
                                  "e.g. IC910-1.\n"
                                  "Allowed characters: "
                                  "0..9, a..z, A..Z, - and _"));
    gtk_grid_attach(GTK_GRID(table), name, 1, 0, 3, 1);

    /* Host */
    label = gtk_label_new(_("Host"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 1, 1, 1);

    host = gtk_entry_new();
    gtk_entry_set_max_length(GTK_ENTRY(host), 50);
    gtk_entry_set_text(GTK_ENTRY(host), "127.0.0.1");
    gtk_widget_set_tooltip_text(host,
                                _("Enter the host where rigctld is running. "
                                  "You can use both host name and IP address, "
                                  "e.g. 192.168.1.100\n\n"
                                  "If gpredict and rigctld are running on the "
                                  "same computer use 127.0.0.1"));
    gtk_grid_attach(GTK_GRID(table), host, 1, 1, 3, 1);

    /* port */
    label = gtk_label_new(_("Port"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 2, 1, 1);

    port = gtk_spin_button_new_with_range(1024, 65535, 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(port), 4532);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(port), 0);
    gtk_widget_set_tooltip_text(port,
                                _("Enter the port number where rigctld is "
                                  "listening"));
    gtk_grid_attach(GTK_GRID(table), port, 1, 2, 1, 1);

    test_button = gtk_button_new_with_label(_("Test connection"));
    gtk_widget_set_tooltip_text(test_button,
                                _("Connect to rigctld and query the current frequency."));
    gtk_grid_attach(GTK_GRID(table), test_button, 2, 2, 2, 1);
    g_signal_connect(test_button, "clicked",
                     G_CALLBACK(rigctld_test_connection_cb), NULL);

    /* radio type */
    label = gtk_label_new(_("Radio type"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    //gtk_table_attach_defaults(GTK_TABLE(table), label, 0, 1, 3, 4);
    gtk_grid_attach(GTK_GRID(table), label, 0, 3, 1, 1);

    type = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(type), _("RX only"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(type), _("TX only"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(type), _("Half-duplex"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(type), _("Full-duplex"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(type),
                                   _("FT817/857/897 (auto)"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(type),
                                   _("FT817/857/897 (manual)"));
    gtk_combo_box_set_active(GTK_COMBO_BOX(type), RIG_TYPE_RX);
    g_signal_connect(type, "changed", G_CALLBACK(type_changed), NULL);
    gtk_widget_set_tooltip_markup(type,
                                  _("<b>RX only:</b>  The radio shall only be "
                                    "used as receiver. If <i>Monitor PTT "
                                    "status</i> is checked the doppler tuning "
                                    "will be suspended while PTT is ON "
                                    "(manual TX). If not, the controller will "
                                    "always perform doppler tuning and "
                                    "you cannot use the same RIG for uplink.\n\n"
                                    "<b>TX only:</b>  The radio shall only be "
                                    "used for uplink. If <i>Monitor PTT status</i>"
                                    " is checked the doppler tuning will be "
                                    "suspended while PTT is OFF (manual RX).\n\n"
                                    "<b>Half-duplex:</b>  The radio should be "
                                    "used for both up- and downlink but not at "
                                    "the same time. This option requires "
                                    "that the PTT status is monitored (otherwise "
                                    "gpredict cannot know whether to tune the "
                                    "RX or the TX).\n\n"
                                    "<b>Full-duplex:</b>  The radio is a full-duplex"
                                    " radio, such as the IC910H. Gpredict will "
                                    "be continuously tuning both uplink and "
                                    "downlink simultaneously and not care about "
                                    "PTT setting.\n\n"
                                    "<b>FT817/857/897 (auto):</b> "
                                    "This is a special mode that can be used with "
                                    "YAESU FT-817, 857 and 897 radios. These radios"
                                    " do not allow computer control while in TX mode."
                                    " Therefore, TX Doppler correction is applied "
                                    "while the radio is in RX mode by toggling "
                                    "between VFO A/B.\n\n"
                                    "<b>FT817/857/897 (manual):</b> "
                                    "This is similar to the previous mode except"
                                    " that switching to TX is done by pressing the"
                                    " SPACE key on the keyboard. Gpredict will "
                                    "then update the TX Doppler before actually"
                                    " switching to TX."));
    gtk_grid_attach(GTK_GRID(table), type, 1, 3, 2, 1);

    /* radio model */
    label = gtk_label_new(_("Radio model"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 4, 1, 1);

    radio_model = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(radio_model), _("Other"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(radio_model), _("IC-9700"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(radio_model), _("IC-705"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(radio_model), _("IC-905"));
    gtk_combo_box_set_active(GTK_COMBO_BOX(radio_model), RADIO_MODEL_OTHER);
    gtk_widget_set_tooltip_text(radio_model,
                                _("Select the radio model used for mode-specific behavior.\n"
                                  "No automatic guessing is performed."));
    gtk_grid_attach(GTK_GRID(table), radio_model, 1, 4, 2, 1);
    g_signal_connect(radio_model, "changed",
                     G_CALLBACK(radio_model_changed), NULL);

    /* radio mode */
    label = gtk_label_new(_("Radio mode"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 5, 1, 1);

    radio_mode = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(radio_mode), _("Simplex"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(radio_mode), _("Split"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(radio_mode),
                                   _("Full-duplex MAIN/SUB"));
    gtk_combo_box_set_active(GTK_COMBO_BOX(radio_mode), RADIO_MODE_SIMPLEX);
    g_signal_connect(radio_mode, "changed",
                     G_CALLBACK(radio_mode_changed), NULL);
    gtk_widget_set_tooltip_text(radio_mode,
                                _("Simplex uses one VFO; Split uses rigctld split.\n"
                                  "Full-duplex MAIN/SUB always updates both VFOs each cycle."));
    gtk_grid_attach(GTK_GRID(table), radio_mode, 1, 5, 2, 1);

    /* ptt */
    label = gtk_label_new(_("PTT status"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 6, 1, 1);

    ptt = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ptt), _("None"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ptt), _("Read PTT"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ptt), _("Read DCD"));
    gtk_combo_box_set_active(GTK_COMBO_BOX(ptt), 0);
    g_signal_connect(ptt, "changed", G_CALLBACK(ptt_changed), NULL);
    gtk_widget_set_tooltip_markup(ptt,
                                  _("Select PTT type.\n\n"
                                    "<b>None:</b>\nDon't read PTT status from this radio.\n\n"
                                    "<b>Read PTT:</b>\nRead PTT status using get_ptt CAT command. "
                                    "You have to check that your radio and hamlib supports this.\n\n"
                                    "<b>Read DCD:</b>\nRead PTT status using get_dcd command. "
                                    "This can be used if your radio does not support the read_ptt "
                                    "CAT command and you have a special interface that can "
                                    "read squelch status and send it via CTS."));
    gtk_grid_attach(GTK_GRID(table), ptt, 1, 6, 2, 1);

    /* VFO Up/Down */
    label = gtk_label_new(_("VFO Up/Down"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 7, 1, 1);

    vfo = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(vfo), _("Not applicable"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(vfo),
                                   _("MAIN \342\206\221 / SUB \342\206\223"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(vfo),
                                   _("SUB \342\206\221 / MAIN \342\206\223"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(vfo),
                                   _("A \342\206\221 / B \342\206\223"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(vfo),
                                   _("B \342\206\221 / A \342\206\223"));
    gtk_combo_box_set_active(GTK_COMBO_BOX(vfo), 0);
    g_signal_connect(vfo, "changed", G_CALLBACK(vfo_changed), NULL);
    gtk_widget_set_tooltip_markup(vfo,
                                  _
                                   ("Select which VFO to use for uplink and downlink. "
                                    "This setting is used for full-duplex radios only, "
                                    "such as the IC-910H, FT-847 and the TS-2000.\n\n"
                                    "<b>IC-910H:</b> MAIN\342\206\221 / SUB\342\206\223\n"
                                    "<b>FT-847:</b> SUB\342\206\221 / MAIN\342\206\223\n"
                                    "<b>TS-2000:</b> B\342\206\221 / A\342\206\223"));
    gtk_grid_attach(GTK_GRID(table), vfo, 1, 7, 2, 1);

    /* Downconverter LO frequency */
    label = gtk_label_new(_("LO Down"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 8, 1, 1);

    lo = gtk_spin_button_new_with_range(-10000, 10000, 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(lo), 0);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(lo), 0);
    gtk_widget_set_tooltip_text(lo,
                                _
                                ("Enter the frequency of the local oscillator "
                                 " of the downconverter, if any."));
    gtk_grid_attach(GTK_GRID(table), lo, 1, 8, 2, 1);

    label = gtk_label_new(_("MHz"));
    g_object_set(label, "xalign", 0.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 3, 8, 1, 1);

    /* Upconverter LO frequency */
    label = gtk_label_new(_("LO Up"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 9, 1, 1);

    loup = gtk_spin_button_new_with_range(-10000, 10000, 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(loup), 0);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(loup), 0);
    gtk_widget_set_tooltip_text(loup,
                                _
                                ("Enter the frequency of the local oscillator "
                                 "of the upconverter, if any."));
    gtk_grid_attach(GTK_GRID(table), loup, 1, 9, 2, 1);

    label = gtk_label_new(_("MHz"));
    g_object_set(label, "xalign", 0.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 3, 9, 1, 1);

    /* AOS / LOS signalling */
    label = gtk_label_new(_("Signalling"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 10, 1, 1);

    sigaos = gtk_check_button_new_with_label(_("AOS"));
    gtk_grid_attach(GTK_GRID(table), sigaos, 1, 10, 1, 1);
    gtk_widget_set_tooltip_text(sigaos,
                                _("Enable AOS signalling for this radio."));

    siglos = gtk_check_button_new_with_label(_("LOS"));
    gtk_grid_attach(GTK_GRID(table), siglos, 2, 10, 1, 1);
    gtk_widget_set_tooltip_text(siglos,
                                _("Enable LOS signalling for this radio."));

    /* Auto-start rigctld */
    label = gtk_label_new(_("Auto-start local rigctld"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 11, 1, 1);

    autostart = gtk_check_button_new_with_label(_("Enable"));
    gtk_grid_attach(GTK_GRID(table), autostart, 1, 11, 1, 1);
    gtk_widget_set_tooltip_text(autostart,
                                _("Start rigctld automatically when engaging "
                                  "if it is not already running."));
    g_signal_connect(autostart, "toggled", G_CALLBACK(autostart_toggled), NULL);

    /* rigctld connection type */
    label = gtk_label_new(_("Connection type"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 12, 1, 1);

    rigctld_conn_type = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(rigctld_conn_type),
                                   _("Serial (USB)"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(rigctld_conn_type),
                                   _("TCP (LAN)"));
    gtk_combo_box_set_active(GTK_COMBO_BOX(rigctld_conn_type),
                             RIGCTLD_CONN_SERIAL);
    gtk_widget_set_tooltip_text(rigctld_conn_type,
                                _("Select how rigctld connects to the radio."));
    gtk_grid_attach(GTK_GRID(table), rigctld_conn_type, 1, 12, 2, 1);
    g_signal_connect(rigctld_conn_type, "changed",
                     G_CALLBACK(rigctld_conn_changed), NULL);

    /* rigctld model */
    label = gtk_label_new(_("Rig model"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 13, 1, 1);

    rigctld_model = gtk_spin_button_new_with_range(0, 99999, 1);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(rigctld_model), 0);
    gtk_widget_set_tooltip_text(rigctld_model,
                                _("Hamlib rig model number (e.g. 3081).\n"
                                  "Find your model id with: rigctl -l | "
                                  "grep -i 'IC-705'."));
    gtk_grid_attach(GTK_GRID(table), rigctld_model, 1, 13, 1, 1);
    g_signal_connect(rigctld_model, "value-changed",
                     G_CALLBACK(rigctld_model_changed), NULL);

    /* rigctld device */
    rigctld_device_label = gtk_label_new(_("Serial device"));
    g_object_set(rigctld_device_label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), rigctld_device_label, 0, 15, 1, 1);

    rigctld_device = gtk_entry_new();
    gtk_entry_set_max_length(GTK_ENTRY(rigctld_device), 200);
    gtk_widget_set_tooltip_text(rigctld_device,
                                _("Serial device for rigctld (e.g. /dev/ttyUSB0)."));
    gtk_grid_attach(GTK_GRID(table), rigctld_device, 1, 15, 2, 1);

    rigctld_device_find = gtk_button_new_with_label(_("Find port"));
    gtk_widget_set_tooltip_text(rigctld_device_find,
                                _("Scan serial ports and select a likely match."));
    gtk_grid_attach(GTK_GRID(table), rigctld_device_find, 3, 15, 1, 1);
    g_signal_connect(rigctld_device_find, "clicked",
                     G_CALLBACK(rigctld_find_port_cb), NULL);

    /* rigctld baud */
    rigctld_baud_label = gtk_label_new(_("Baud"));
    g_object_set(rigctld_baud_label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), rigctld_baud_label, 0, 16, 1, 1);

    rigctld_baud = gtk_spin_button_new_with_range(0, 1000000, 1);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(rigctld_baud), 0);
    gtk_widget_set_tooltip_text(rigctld_baud,
                                _("Serial baud rate for rigctld (e.g. 19200)."));
    gtk_grid_attach(GTK_GRID(table), rigctld_baud, 1, 16, 1, 1);

    /* Advanced rigctld options */
    advanced_table = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(advanced_table), 5);
    gtk_grid_set_column_spacing(GTK_GRID(advanced_table), 5);
    gtk_grid_set_row_spacing(GTK_GRID(advanced_table), 5);

    /* rigctld auto power-on */
    label = gtk_label_new(_("Auto power-on"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(advanced_table), label, 0, 0, 1, 1);

    rigctld_auto_power_on = gtk_check_button_new_with_label(_("Enable"));
    gtk_grid_attach(GTK_GRID(advanced_table), rigctld_auto_power_on, 1, 0, 1, 1);
    gtk_widget_set_tooltip_text(rigctld_auto_power_on,
                                _("Enable rigctld auto power-on if supported."));

    /* rigctld path */
    label = gtk_label_new(_("rigctld path"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(advanced_table), label, 0, 1, 1, 1);

    rigctld_path = gtk_entry_new();
    gtk_entry_set_max_length(GTK_ENTRY(rigctld_path), 200);
    gtk_widget_set_tooltip_text(rigctld_path,
                                _("Path to rigctld binary (leave empty to use bundled rigctld or PATH)."));
    gtk_grid_attach(GTK_GRID(advanced_table), rigctld_path, 1, 1, 3, 1);

    /* rigctld CI-V address */
    label = gtk_label_new(_("CI-V addr"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(advanced_table), label, 0, 2, 1, 1);

    rigctld_civaddr = gtk_entry_new();
    gtk_entry_set_max_length(GTK_ENTRY(rigctld_civaddr), 16);
    gtk_widget_set_tooltip_text(rigctld_civaddr,
                                _("Optional CI-V address (e.g. 0xA2)."));
    gtk_grid_attach(GTK_GRID(advanced_table), rigctld_civaddr, 1, 2, 2, 1);

    /* rigctld extra args */
    label = gtk_label_new(_("Extra args"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(advanced_table), label, 0, 3, 1, 1);

    rigctld_extra_args = gtk_entry_new();
    gtk_entry_set_max_length(GTK_ENTRY(rigctld_extra_args), 200);
    gtk_widget_set_tooltip_text(rigctld_extra_args,
                                _("Extra rigctld arguments (optional)."));
    gtk_grid_attach(GTK_GRID(advanced_table), rigctld_extra_args, 1, 3, 3, 1);

    advanced_expander = gtk_expander_new(_("Advanced..."));
    gtk_container_add(GTK_CONTAINER(advanced_expander), advanced_table);
    g_signal_connect(advanced_expander, "notify::expanded",
                     G_CALLBACK(advanced_expander_notify), NULL);

    if (conf->name != NULL)
        update_widgets(conf);
    else
        update_autostart_sensitivity(TRUE);

    gtk_box_pack_start(GTK_BOX(vbox), table, FALSE, FALSE, 0);
    gtk_box_pack_start(GTK_BOX(vbox), advanced_expander, FALSE, FALSE, 0);
    gtk_widget_show_all(vbox);

    return vbox;
}

/* Apply changes. Returns TRUE if things are ok, FALSE otherwise */
static gboolean apply_changes(radio_conf_t * conf)
{
    radio_model_t selected_model;
    radio_mode_t selected_mode;
    gchar *allowed = NULL;
    gchar *detail = NULL;

    selected_model = gtk_combo_box_get_active(GTK_COMBO_BOX(radio_model));
    selected_mode = gtk_combo_box_get_active(GTK_COMBO_BOX(radio_mode));

    if (!radio_mode_allowed_for_model(selected_model, selected_mode))
    {
        allowed = radio_mode_allowed_string(selected_model);
        detail = g_strdup_printf("Model: %s\nMode: %s\nAllowed: %s",
                                 radio_model_to_string(selected_model),
                                 radio_mode_to_string(selected_mode),
                                 allowed ? allowed : "none");
        rig_pref_show_dialog(GTK_MESSAGE_ERROR,
                             _("Unsupported radio mode"),
                             detail);
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    "invalid radio mode model=%s mode=%s allowed=%s",
                    radio_model_to_string(selected_model),
                    radio_mode_to_string(selected_mode),
                    allowed ? allowed : "none");
        g_free(detail);
        g_free(allowed);
        return FALSE;
    }

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

    /* lo down freq */
    conf->lo = 1000000.0 * gtk_spin_button_get_value(GTK_SPIN_BUTTON(lo));

    /* lo up freq */
    conf->loup = 1000000.0 * gtk_spin_button_get_value(GTK_SPIN_BUTTON(loup));

    /* rig type */
    conf->type = gtk_combo_box_get_active(GTK_COMBO_BOX(type));

    /* radio model */
    conf->radio_model = selected_model;

    /* radio mode */
    conf->radio_mode = selected_mode;

    /* ptt */
    conf->ptt = gtk_combo_box_get_active(GTK_COMBO_BOX(ptt));

    /* vfo up/down */
    switch (gtk_combo_box_get_active(GTK_COMBO_BOX(vfo)))
    {
    case 1:
        conf->uplink_vfo = VFO_MAIN;
        conf->downlink_vfo = VFO_SUB;
        break;
    case 2:
        conf->uplink_vfo = VFO_SUB;
        conf->downlink_vfo = VFO_MAIN;
        break;
    case 3:
        conf->uplink_vfo = VFO_A;
        conf->downlink_vfo = VFO_B;
        break;
    case 4:
        conf->uplink_vfo = VFO_B;
        conf->downlink_vfo = VFO_A;
        break;
    default:
        if (conf->radio_mode == RADIO_MODE_SIMPLEX)
        {
            conf->uplink_vfo = VFO_SUB;
            conf->downlink_vfo = VFO_MAIN;
        }
        else
        {
            conf->uplink_vfo = VFO_MAIN;
            conf->downlink_vfo = VFO_SUB;
        }
        break;
    }

    /* AOS / LOS signalling */
    conf->signal_aos = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(sigaos));
    conf->signal_los = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(siglos));
    conf->supports_dual_vfo_sat =
        (conf->radio_mode == RADIO_MODE_FULL_DUPLEX_MAIN_SUB);

    /* rigctld auto-start */
    conf->rigctld_autostart =
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(autostart));
    conf->rigctld_auto_power_on =
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(rigctld_auto_power_on));

    if (conf->rigctld_path)
        g_free(conf->rigctld_path);
    conf->rigctld_path =
        g_strdup(gtk_entry_get_text(GTK_ENTRY(rigctld_path)));

    conf->rigctld_model =
        gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(rigctld_model));
    if (conf->rigctld_model <= 0)
    {
        gint preset = radio_model_to_hamlib_model(conf->radio_model);

        /* Avoid saving an empty rig model when a preset exists. */
        if (preset > 0)
            conf->rigctld_model = preset;
    }

    conf->rigctld_conn =
        gtk_combo_box_get_active(GTK_COMBO_BOX(rigctld_conn_type));

    if (conf->rigctld_device)
        g_free(conf->rigctld_device);
    conf->rigctld_device =
        g_strdup(gtk_entry_get_text(GTK_ENTRY(rigctld_device)));

    conf->rigctld_baud =
        gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(rigctld_baud));

    if (conf->rigctld_civaddr)
        g_free(conf->rigctld_civaddr);
    conf->rigctld_civaddr =
        g_strdup(gtk_entry_get_text(GTK_ENTRY(rigctld_civaddr)));

    if (conf->rigctld_extra_args)
        g_free(conf->rigctld_extra_args);
    conf->rigctld_extra_args =
        g_strdup(gtk_entry_get_text(GTK_ENTRY(rigctld_extra_args)));

    return TRUE;
}

/* Add or edit a radio configuration */
void sat_pref_rig_editor_run(radio_conf_t * conf)
{
    gint            response;
    gboolean        finished = FALSE;

    /* create dialog and add contents */
    dialog = gtk_dialog_new_with_buttons(_("Edit radio configuration"),
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

    /* keep the dialog running when CLEAR button is plressed
     * OK and CANCEL will exit the loop
     */
    while (!finished)
    {
        response = gtk_dialog_run(GTK_DIALOG(dialog));

        switch (response)
        {
            /* OK */
        case GTK_RESPONSE_OK:
            if (apply_changes(conf))
                finished = TRUE;
            else
                finished = FALSE;
            break;

            /* CLEAR */
        case GTK_RESPONSE_REJECT:
            clear_widgets();
            break;

            /* Everything else is considered CANCEL */
        default:
            finished = TRUE;
            break;
        }
    }

    gtk_widget_destroy(dialog);
}
