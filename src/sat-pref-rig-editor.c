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
#include "ui-popup-quarantine.h"


extern GtkWidget *window;       /* dialog window defined in sat-pref.c */

typedef struct {
    GtkWidget *dialog;       /* dialog window */
    GtkWidget *name;         /* config name */
    GtkWidget *host;         /* host */
    GtkWidget *port;         /* port number */
    GtkWidget *type;         /* rig type */
    GtkWidget *radio_model;  /* radio model */
    GtkWidget *radio_mode;   /* radio mode */
    GtkWidget *ptt;          /* PTT */
    GtkWidget *vfo;          /* VFO Up/Down selector */
    GtkWidget *lo;           /* local oscillator of downconverter */
    GtkWidget *loup;         /* local oscillator of upconverter */
    GtkWidget *sigaos;       /* AOS signalling */
    GtkWidget *siglos;       /* LOS signalling */
    GtkWidget *autostart;    /* auto-start rigctld */
    GtkWidget *rigctld_conn_type; /* rigctld connection type */
    GtkWidget *rigctld_auto_power_on; /* rigctld auto power-on */
    GtkWidget *rigctld_path; /* rigctld path */
    GtkWidget *rigctld_model; /* rigctld model */
    GtkWidget *rigctld_device_label; /* rigctld device label */
    GtkWidget *rigctld_device; /* rigctld device */
    GtkWidget *rigctld_device_find; /* rigctld device finder */
    GtkWidget *rigctld_baud_label; /* rigctld baud label */
    GtkWidget *rigctld_baud; /* rigctld baud */
    GtkWidget *rigctld_civaddr; /* rigctld CI-V address */
    GtkWidget *rigctld_extra_args; /* rigctld extra args */
    gint rigctld_model_custom; /* remember custom rig model */
    radio_model_t last_radio_model;
    gboolean ui_updating;
    guint pending_ui_refresh_id;
} RigPrefUi;

typedef struct {
    RigPrefUi *ui;
    GtkWidget *combo;
    GtkWidget *entry;
    GSList    *candidates;
} RigPortDialogState;

typedef struct {
    radio_conf_t          *conf;
    RigPrefEditorDoneFunc  done;
    gpointer               user_data;
    gboolean               finished;
    RigPrefUi             *ui;
} RigPrefDialogState;

static void rig_pref_message_response(GtkDialog *dialog,
                                      gint response,
                                      gpointer user_data);
static void rig_pref_message_destroy(GtkWidget *widget, gpointer user_data);

static void rig_pref_show_dialog(RigPrefUi *ui,
                                 GtkMessageType type,
                                 const gchar *primary,
                                 const gchar *secondary);
static void rig_pref_dialog_response(GtkDialog *dialog,
                                     gint response,
                                     gpointer user_data);
static void rig_pref_dialog_destroy(GtkWidget *widget, gpointer user_data);
static void rig_pref_port_dialog_response(GtkDialog *dialog,
                                          gint response,
                                          gpointer user_data);
static void rig_pref_port_dialog_destroy(GtkWidget *widget, gpointer user_data);
static void rig_pref_ui_begin_update(RigPrefUi *ui, const gchar *reason);
static void rig_pref_ui_end_update(RigPrefUi *ui, const gchar *reason);
static gboolean rig_pref_form_is_valid(RigPrefUi *ui);
static void rig_pref_update_ok_button(RigPrefUi *ui);
static void rig_pref_on_field_changed(GtkWidget *widget, gpointer data);
static void name_changed(GtkWidget *widget, gpointer data);
static void type_changed(GtkWidget *widget, gpointer data);
static void radio_model_changed(GtkComboBox *box, gpointer data);
static void radio_mode_changed(GtkWidget *widget, gpointer data);
static void ptt_changed(GtkWidget *widget, gpointer data);
static void vfo_changed(GtkWidget *widget, gpointer data);
static void autostart_toggled(GtkToggleButton *toggle, gpointer data);
static void rigctld_conn_changed(GtkComboBox *box, gpointer data);
static void rigctld_model_changed(GtkSpinButton *spin, gpointer data);

static gboolean rigctld_conn_is_tcp(RigPrefUi *ui)
{
    if (ui == NULL || ui->rigctld_conn_type == NULL)
        return FALSE;

    return gtk_combo_box_get_active(GTK_COMBO_BOX(ui->rigctld_conn_type)) ==
        RIGCTLD_CONN_TCP;
}

static gboolean radio_model_has_preset(RigPrefUi *ui)
{
    if (ui == NULL || ui->radio_model == NULL)
        return FALSE;

    return radio_model_to_hamlib_model(
        gtk_combo_box_get_active(GTK_COMBO_BOX(ui->radio_model))) > 0;
}

static void update_rigctld_model_sensitivity(RigPrefUi *ui, gboolean enabled)
{
    gboolean allow_edit;

    if (ui == NULL)
        return;

    allow_edit = enabled && !radio_model_has_preset(ui);

    if (ui->rigctld_model != NULL)
        gtk_widget_set_sensitive(ui->rigctld_model, allow_edit);
}

static void rigcfg_update_vfo_sensitivity(RigPrefUi *ui)
{
    if (ui == NULL || ui->vfo == NULL)
        return;

    /* radio_mode uses a text-only combo; keep VFO enabled to avoid index mismatch. */
    gtk_widget_set_sensitive(ui->vfo, TRUE);
}

static void rig_pref_ui_begin_update(RigPrefUi *ui, const gchar *reason)
{
    if (ui == NULL)
        return;

    ui->ui_updating = TRUE;
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "rig-pref ui_begin_update %s",
                reason ? reason : "(none)");
}

static void rig_pref_ui_end_update(RigPrefUi *ui, const gchar *reason)
{
    if (ui == NULL)
        return;

    ui->ui_updating = FALSE;
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "rig-pref ui_end_update %s",
                reason ? reason : "(none)");
}

static gboolean rig_pref_form_is_valid(RigPrefUi *ui)
{
    gchar *trimmed;
    gboolean valid;

    if (ui == NULL || ui->name == NULL)
        return FALSE;

    trimmed = g_strdup(gtk_entry_get_text(GTK_ENTRY(ui->name)));
    g_strstrip(trimmed);
    valid = trimmed[0] != '\0';
    g_free(trimmed);

    return valid;
}

static void rig_pref_update_ok_button(RigPrefUi *ui)
{
    if (ui == NULL || ui->dialog == NULL)
        return;

    gtk_dialog_set_response_sensitive(GTK_DIALOG(ui->dialog),
                                      GTK_RESPONSE_OK,
                                      rig_pref_form_is_valid(ui));
}

static void rig_pref_on_field_changed(GtkWidget *widget, gpointer data)
{
    RigPrefUi *ui = data;

    (void)widget;

    if (ui != NULL && ui->ui_updating)
        return;

    rig_pref_update_ok_button(ui);
}

static void rig_pref_combo_set_active(GtkComboBox *combo, gint index,
                                      GCallback cb)
{
    if (combo == NULL)
        return;

    g_signal_handlers_block_by_func(combo, (gpointer)cb, NULL);
    gtk_combo_box_set_active(combo, index);
    g_signal_handlers_unblock_by_func(combo, (gpointer)cb, NULL);
}

static gboolean rig_pref_combo_popup_shown(GtkComboBox *combo)
{
    return gp_ui_combo_popup_shown(combo);
}


typedef struct
{
    RigPrefUi *ui;
    GtkComboBox *combo;
    gint index;
    GCallback cb;
} RigPrefComboUpdate;

static gboolean rig_pref_combo_set_active_idle(gpointer data)
{
    RigPrefComboUpdate *update = data;

    if (update == NULL || update->combo == NULL)
    {
        g_free(update);
        return G_SOURCE_REMOVE;
    }

    if (rig_pref_combo_popup_shown(update->combo))
        return G_SOURCE_CONTINUE;

    rig_pref_ui_begin_update(update->ui, "combo_deferred");
    rig_pref_combo_set_active(update->combo, update->index, update->cb);
    rig_pref_ui_end_update(update->ui, "combo_deferred");
    rig_pref_update_ok_button(update->ui);

    g_object_unref(update->combo);
    g_free(update);
    return G_SOURCE_REMOVE;
}

static void rig_pref_combo_set_active_deferred(RigPrefUi *ui,
                                               GtkComboBox *combo,
                                               gint index,
                                               GCallback cb)
{
    RigPrefComboUpdate *update;

    if (combo == NULL)
        return;

    update = g_new0(RigPrefComboUpdate, 1);
    update->ui = ui;
    update->combo = g_object_ref(combo);
    update->index = index;
    update->cb = cb;

    g_idle_add_full(G_PRIORITY_DEFAULT_IDLE,
                    rig_pref_combo_set_active_idle,
                    update,
                    NULL);
}

static void update_rigctld_connection_ui(RigPrefUi *ui, gboolean enabled)
{
    gboolean is_tcp;

    if (ui == NULL)
        return;

    is_tcp = rigctld_conn_is_tcp(ui);

    if (ui->rigctld_device_label != NULL)
    {
        gtk_label_set_text(GTK_LABEL(ui->rigctld_device_label),
                           is_tcp ? _("Rig address") : _("Serial device"));
    }

    if (ui->rigctld_device != NULL)
    {
        gtk_widget_set_tooltip_text(
            ui->rigctld_device,
            is_tcp ? _("Rig TCP address for rigctld (host:port).")
                   : _("Serial device for rigctld (e.g. /dev/ttyUSB0)."));
    }
    if (ui->rigctld_device_find != NULL)
        gtk_widget_set_sensitive(ui->rigctld_device_find, enabled && !is_tcp);

    if (ui->rigctld_baud_label != NULL)
        gtk_widget_set_sensitive(ui->rigctld_baud_label, enabled && !is_tcp);
    if (ui->rigctld_baud != NULL)
        gtk_widget_set_sensitive(ui->rigctld_baud, enabled && !is_tcp);
}

static void rigctld_find_port_cb(GtkButton *button, gpointer data)
{
    RigPrefUi *ui = data;
    const gchar *current = NULL;
    GSList *candidates = NULL;
    guint count = 0;
    GtkWidget *dialog;
    GtkWidget *content;
    GtkWidget *combo;
    GtkWidget *label;
    GtkWidget *toplevel;
    GtkWindow *parent = NULL;
    gint active_index = -1;
    gint index = 0;
    RigPortDialogState *state = NULL;

    (void)button;

    if (ui == NULL)
        return;

    if (rigctld_conn_is_tcp(ui))
    {
        rig_pref_show_dialog(ui, GTK_MESSAGE_INFO,
                             _("Find port is only available for serial rigs."),
                             NULL);
        return;
    }

    if (ui->rigctld_device)
        current = gtk_entry_get_text(GTK_ENTRY(ui->rigctld_device));

    candidates = gp_serial_list_candidates();
    count = g_slist_length(candidates);
    sat_log_log(SAT_LOG_LEVEL_INFO,
                "rigctld find port: candidates=%u", count);

    if (candidates == NULL)
    {
        rig_pref_show_dialog(ui, GTK_MESSAGE_WARNING,
                             _("No serial ports found."),
                             NULL);
        return;
    }

    toplevel = gtk_widget_get_toplevel(GTK_WIDGET(ui->rigctld_device));
    if (GTK_IS_WINDOW(toplevel))
        parent = GTK_WINDOW(toplevel);

    dialog = gtk_dialog_new_with_buttons(_("Select serial port"),
                                         parent,
                                         GTK_DIALOG_DESTROY_WITH_PARENT,
                                         _("_Cancel"),
                                         GTK_RESPONSE_CANCEL,
                                         _("_Select"),
                                         GTK_RESPONSE_OK,
                                         NULL);
    content = gtk_dialog_get_content_area(GTK_DIALOG(dialog));
    label = gtk_label_new(_("Choose a serial port:"));
    gtk_label_set_xalign(GTK_LABEL(label), 0.0);
    gtk_box_pack_start(GTK_BOX(content), label, FALSE, FALSE, 5);

    combo = gtk_combo_box_text_new();
    for (GSList *iter = candidates; iter != NULL; iter = iter->next)
    {
        const gchar *candidate = iter->data;

        if (candidate == NULL)
            continue;

        gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(combo), candidate);
        if (current && *current && g_strcmp0(current, candidate) == 0)
            active_index = index;
        index++;
    }
    if (active_index >= 0)
        gtk_combo_box_set_active(GTK_COMBO_BOX(combo), active_index);
    gtk_box_pack_start(GTK_BOX(content), combo, FALSE, FALSE, 5);

    state = g_new0(RigPortDialogState, 1);
    state->ui = ui;
    state->combo = combo;
    state->entry = ui->rigctld_device;
    state->candidates = candidates;

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "rig-pref port dialog show candidates=%u", count);
    g_signal_connect(dialog, "response",
                     G_CALLBACK(rig_pref_port_dialog_response), state);
    g_signal_connect(dialog, "destroy",
                     G_CALLBACK(rig_pref_port_dialog_destroy), state);

    gtk_widget_show_all(dialog);
}

static void rig_pref_port_dialog_response(GtkDialog *dialog,
                                          gint response,
                                          gpointer user_data)
{
    RigPortDialogState *state = user_data;

    if (state == NULL)
        return;

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "rig-pref port dialog response=%d", response);
    if (response == GTK_RESPONSE_OK)
    {
        gchar *picked = gtk_combo_box_text_get_active_text(
            GTK_COMBO_BOX_TEXT(state->combo));
        if (picked != NULL && *picked != '\0')
        {
            gtk_entry_set_text(GTK_ENTRY(state->entry), picked);
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        "rigctld find port selected=%s", picked);
        }
        else
        {
            rig_pref_show_dialog(state->ui, GTK_MESSAGE_WARNING,
                                 _("No serial port selected."),
                                 NULL);
        }
        g_free(picked);
    }

    gtk_widget_destroy(GTK_WIDGET(dialog));
}

static void rig_pref_port_dialog_destroy(GtkWidget *widget, gpointer user_data)
{
    RigPortDialogState *state = user_data;

    (void)widget;

    if (state == NULL)
        return;

    gp_serial_free_candidates(state->candidates);
    g_free(state);

    sat_log_log(SAT_LOG_LEVEL_DEBUG, "rig-pref port dialog destroyed");
}

static void apply_preset_rigctld_defaults(RigPrefUi *ui,
                                          radio_model_t model,
                                          gboolean force)
{
    rigctld_preset_defaults_t preset;

    if (!radio_model_get_rigctld_defaults(model, &preset))
        return;

    if (ui == NULL)
        return;

    if (ui->host != NULL)
    {
        const gchar *current = gtk_entry_get_text(GTK_ENTRY(ui->host));
        if (force || current == NULL || *current == '\0')
            gtk_entry_set_text(GTK_ENTRY(ui->host),
                               preset.host ? preset.host : "");
    }

    if (ui->port != NULL)
    {
        gint value = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(ui->port));
        if (force || value <= 0)
            gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->port), preset.port);
    }

    if (ui->rigctld_conn_type != NULL && force)
    {
        if (ui->ui_updating)
            rig_pref_combo_set_active(GTK_COMBO_BOX(ui->rigctld_conn_type),
                                      preset.conn, G_CALLBACK(rigctld_conn_changed));
        else
            rig_pref_combo_set_active_deferred(ui,
                                               GTK_COMBO_BOX(ui->rigctld_conn_type),
                                               preset.conn,
                                               G_CALLBACK(rigctld_conn_changed));
    }

    if (ui->rigctld_device != NULL && force)
        gtk_entry_set_text(GTK_ENTRY(ui->rigctld_device), "");

    if (ui->rigctld_baud != NULL)
    {
        gint value =
            gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(ui->rigctld_baud));
        if (force || value <= 0)
            gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->rigctld_baud),
                                      preset.baud);
    }

    if (ui->rigctld_civaddr != NULL)
    {
        const gchar *current = gtk_entry_get_text(GTK_ENTRY(ui->rigctld_civaddr));
        if (force || current == NULL || *current == '\0')
            gtk_entry_set_text(GTK_ENTRY(ui->rigctld_civaddr),
                               preset.civaddr ? preset.civaddr : "");
    }
}

static void update_autostart_sensitivity(RigPrefUi *ui, gboolean enabled)
{
    if (ui == NULL)
        return;

    gtk_widget_set_sensitive(ui->rigctld_conn_type, enabled);
    gtk_widget_set_sensitive(ui->rigctld_auto_power_on, enabled);
    gtk_widget_set_sensitive(ui->rigctld_path, enabled);
    update_rigctld_model_sensitivity(ui, enabled);
    gtk_widget_set_sensitive(ui->rigctld_device, enabled);
    gtk_widget_set_sensitive(ui->rigctld_civaddr, enabled);
    gtk_widget_set_sensitive(ui->rigctld_extra_args, enabled);
    update_rigctld_connection_ui(ui, enabled);
}

static void rigctld_model_changed(GtkSpinButton *spin, gpointer data)
{
    RigPrefUi *ui = data;

    if (ui == NULL || ui->ui_updating)
        return;

    if (ui->radio_model == NULL || spin == NULL)
        return;

    if (gtk_combo_box_get_active(GTK_COMBO_BOX(ui->radio_model)) ==
        RADIO_MODEL_OTHER)
    {
        ui->rigctld_model_custom =
            gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(spin));
    }
}

static void rig_pref_show_dialog(RigPrefUi *ui,
                                 GtkMessageType type,
                                 const gchar *primary,
                                 const gchar *secondary)
{
    GtkWindow *parent = ui && ui->dialog ? GTK_WINDOW(ui->dialog) : NULL;
    GtkWidget *msg = gtk_message_dialog_new(parent,
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
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "rig-pref message dialog show primary=%s",
                primary ? primary : "(null)");
    g_signal_connect(msg, "response",
                     G_CALLBACK(rig_pref_message_response), NULL);
    g_signal_connect(msg, "destroy",
                     G_CALLBACK(rig_pref_message_destroy), NULL);
    gtk_widget_show(msg);
}

static void rig_pref_message_response(GtkDialog *dialog,
                                      gint response,
                                      gpointer user_data)
{
    (void)user_data;

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "rig-pref message dialog response=%d", response);
    gtk_widget_destroy(GTK_WIDGET(dialog));
}

static void rig_pref_message_destroy(GtkWidget *widget, gpointer user_data)
{
    (void)widget;
    (void)user_data;

    sat_log_log(SAT_LOG_LEVEL_DEBUG, "rig-pref message dialog destroyed");
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
    RigPrefUi *ui = data;
    const gchar *host_text;
    gint port_val;
    gchar *reply = NULL;
    gchar *err = NULL;
    gboolean ok;

    (void)button;

    if (ui == NULL || ui->host == NULL || ui->port == NULL)
        return;

    host_text = gtk_entry_get_text(GTK_ENTRY(ui->host));
    port_val = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(ui->port));

    ok = rigctld_test_query(host_text, port_val, &reply, &err);
    if (ok)
    {
        rig_pref_show_dialog(ui, GTK_MESSAGE_INFO,
                             _("rigctld connection OK"),
                             _("See log for details."));
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "rigctld test ok host=%s port=%d reply=%s",
                    host_text, port_val, reply ? reply : "(none)");
    }
    else
    {
        rig_pref_show_dialog(ui, GTK_MESSAGE_ERROR,
                             _("rigctld connection failed"),
                             _("See log for details."));
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    "rigctld test failed host=%s port=%d error=%s reply=%s",
                    host_text, port_val,
                    err ? err : "unknown",
                    reply ? reply : "(none)");
    }

    g_free(reply);
    g_free(err);
}


static void clear_widgets(RigPrefUi *ui)
{
    if (ui == NULL)
        return;

    rig_pref_ui_begin_update(ui, "clear_widgets");

    gtk_entry_set_text(GTK_ENTRY(ui->name), "");
    gtk_entry_set_text(GTK_ENTRY(ui->host), "127.0.0.1");
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->port), 4532);     /* hamlib default? */
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->lo), 0);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->loup), 0);
    rig_pref_combo_set_active(GTK_COMBO_BOX(ui->type), RIG_TYPE_RX,
                              G_CALLBACK(type_changed));
    rig_pref_combo_set_active(GTK_COMBO_BOX(ui->radio_model), RADIO_MODEL_OTHER,
                              G_CALLBACK(radio_model_changed));
    ui->rigctld_model_custom = 0;
    ui->last_radio_model = RADIO_MODEL_OTHER;
    rig_pref_combo_set_active(GTK_COMBO_BOX(ui->radio_mode), RADIO_MODE_SIMPLEX,
                              G_CALLBACK(radio_mode_changed));
    rig_pref_combo_set_active(GTK_COMBO_BOX(ui->ptt), PTT_TYPE_NONE,
                              G_CALLBACK(ptt_changed));
    rig_pref_combo_set_active(GTK_COMBO_BOX(ui->vfo), 0,
                              G_CALLBACK(vfo_changed));
    rigcfg_update_vfo_sensitivity(ui);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ui->ptt), FALSE);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ui->sigaos), FALSE);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ui->siglos), FALSE);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ui->autostart), TRUE);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ui->rigctld_auto_power_on),
                                 FALSE);
    gtk_entry_set_text(GTK_ENTRY(ui->rigctld_path), "");
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->rigctld_model), 0);
    rig_pref_combo_set_active(GTK_COMBO_BOX(ui->rigctld_conn_type),
                              RIGCTLD_CONN_SERIAL,
                              G_CALLBACK(rigctld_conn_changed));
    gtk_entry_set_text(GTK_ENTRY(ui->rigctld_device), "");
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->rigctld_baud), 0);
    gtk_entry_set_text(GTK_ENTRY(ui->rigctld_civaddr), "");
    gtk_entry_set_text(GTK_ENTRY(ui->rigctld_extra_args), "");
    update_autostart_sensitivity(ui, TRUE);

    rig_pref_ui_end_update(ui, "clear_widgets");
    rig_pref_update_ok_button(ui);
}

static void update_widgets(RigPrefUi *ui, radio_conf_t * conf)
{
    if (ui == NULL || conf == NULL)
        return;

    rig_pref_ui_begin_update(ui, "update_widgets");

    /* configuration name */
    gtk_entry_set_text(GTK_ENTRY(ui->name), conf->name);

    /* host name */
    if (conf->host)
        gtk_entry_set_text(GTK_ENTRY(ui->host), conf->host);

    /* port */
    if (conf->port > 1023)
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->port), conf->port);
    else
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->port), 4532); /* hamlib default? */

    /* radio type */
    rig_pref_combo_set_active(GTK_COMBO_BOX(ui->type), conf->type,
                              G_CALLBACK(type_changed));

    /* radio model */
    rig_pref_combo_set_active(GTK_COMBO_BOX(ui->radio_model), conf->radio_model,
                              G_CALLBACK(radio_model_changed));
    ui->last_radio_model = conf->radio_model;
    if (conf->radio_model == RADIO_MODEL_OTHER)
        ui->rigctld_model_custom = conf->rigctld_model;
    else
        ui->rigctld_model_custom = 0;

    /* radio mode */
    rig_pref_combo_set_active(GTK_COMBO_BOX(ui->radio_mode), conf->radio_mode,
                              G_CALLBACK(radio_mode_changed));

    /* ptt */
    rig_pref_combo_set_active(GTK_COMBO_BOX(ui->ptt), conf->ptt,
                              G_CALLBACK(ptt_changed));

    /* vfo up/down */
    if (conf->uplink_vfo == VFO_MAIN && conf->downlink_vfo == VFO_SUB)
        rig_pref_combo_set_active(GTK_COMBO_BOX(ui->vfo), 1,
                                  G_CALLBACK(vfo_changed));
    else if (conf->uplink_vfo == VFO_SUB && conf->downlink_vfo == VFO_MAIN)
        rig_pref_combo_set_active(GTK_COMBO_BOX(ui->vfo), 2,
                                  G_CALLBACK(vfo_changed));
    else if (conf->uplink_vfo == VFO_A && conf->downlink_vfo == VFO_B)
        rig_pref_combo_set_active(GTK_COMBO_BOX(ui->vfo), 3,
                                  G_CALLBACK(vfo_changed));
    else if (conf->uplink_vfo == VFO_B && conf->downlink_vfo == VFO_A)
        rig_pref_combo_set_active(GTK_COMBO_BOX(ui->vfo), 4,
                                  G_CALLBACK(vfo_changed));
    else
        rig_pref_combo_set_active(GTK_COMBO_BOX(ui->vfo), 0,
                                  G_CALLBACK(vfo_changed));
    rigcfg_update_vfo_sensitivity(ui);

    /* lo down in MHz */
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->lo), conf->lo / 1000000.0);

    /* lo up in MHz */
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->loup), conf->loup / 1000000.0);

    /* AOS / LOS signalling */
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ui->sigaos), conf->signal_aos);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ui->siglos), conf->signal_los);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ui->autostart),
                                 conf->rigctld_autostart);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ui->rigctld_auto_power_on),
                                 conf->rigctld_auto_power_on);
    gtk_entry_set_text(GTK_ENTRY(ui->rigctld_path), "");
    if (conf->rigctld_path)
        gtk_entry_set_text(GTK_ENTRY(ui->rigctld_path), conf->rigctld_path);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->rigctld_model),
                              conf->rigctld_model);
    if (conf->rigctld_model <= 0)
    {
        gint preset = radio_model_to_hamlib_model(conf->radio_model);

        /* Prefer preset model ids when config lacks an explicit rig model. */
        if (preset > 0)
            gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->rigctld_model), preset);
    }
    rig_pref_combo_set_active(GTK_COMBO_BOX(ui->rigctld_conn_type),
                              conf->rigctld_conn,
                              G_CALLBACK(rigctld_conn_changed));
    gtk_entry_set_text(GTK_ENTRY(ui->rigctld_device), "");
    if (conf->rigctld_device)
        gtk_entry_set_text(GTK_ENTRY(ui->rigctld_device), conf->rigctld_device);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->rigctld_baud),
                              conf->rigctld_baud);
    gtk_entry_set_text(GTK_ENTRY(ui->rigctld_civaddr), "");
    if (conf->rigctld_civaddr)
        gtk_entry_set_text(GTK_ENTRY(ui->rigctld_civaddr), conf->rigctld_civaddr);
    gtk_entry_set_text(GTK_ENTRY(ui->rigctld_extra_args), "");
    if (conf->rigctld_extra_args)
        gtk_entry_set_text(GTK_ENTRY(ui->rigctld_extra_args),
                           conf->rigctld_extra_args);
    apply_preset_rigctld_defaults(ui, conf->radio_model, FALSE);
    update_autostart_sensitivity(ui, conf->rigctld_autostart);

    rig_pref_ui_end_update(ui, "update_widgets");
    rig_pref_update_ok_button(ui);
}

/*
 * Manage VFO changed signals.
 * Widget is the GtkComboBox that received the signal.
 *  
 * This function is called when the user selects a new VFO up/down combination.
 */
static void vfo_changed(GtkWidget * widget, gpointer data)
{
    RigPrefUi *ui = data;

    (void)widget;

    if (ui != NULL && ui->ui_updating)
        return;

    rigcfg_update_vfo_sensitivity(ui);
}

/*
 * Manage ptt type changed signals.
 * Widget is the GtkComboBox that received the signal.
 *  
 * This function is called when the user selects a new ptt type.
 */
static void ptt_changed(GtkWidget * widget, gpointer data)
{
    RigPrefUi *ui = data;

    if (ui != NULL && ui->ui_updating)
        return;

    if (gtk_combo_box_get_active(GTK_COMBO_BOX(widget)) == PTT_TYPE_NONE &&
        ui != NULL &&
        gtk_combo_box_get_active(GTK_COMBO_BOX(ui->type)) == RIG_TYPE_TRX)
    {
        /* not good, we need to have PTT for this type */
        rig_pref_combo_set_active_deferred(ui, GTK_COMBO_BOX(widget),
                                           PTT_TYPE_CAT,
                                           G_CALLBACK(ptt_changed));
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
    RigPrefUi     *ui = data;
    const gchar    *text;
    gchar          *entry, *end, *j;
    gint            len, pos;

    if (ui != NULL && ui->ui_updating)
        return;

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
    (void)text;
    rig_pref_update_ok_button(ui);
}

/*
 * Manage rig type changed signals.
 * widget os the GtkComboBox that received the signal.
 *  
 * This function is called when the user selects a new radio type.
 */
static void type_changed(GtkWidget * widget, gpointer data)
{
    RigPrefUi *ui = data;

    if (ui != NULL && ui->ui_updating)
        return;

    /* PTT consistency */
    if (gtk_combo_box_get_active(GTK_COMBO_BOX(widget)) == RIG_TYPE_TRX)
    {
        if (ui != NULL &&
            gtk_combo_box_get_active(GTK_COMBO_BOX(ui->ptt)) == PTT_TYPE_NONE)
        {
            rig_pref_combo_set_active_deferred(ui, GTK_COMBO_BOX(ui->ptt),
                                               PTT_TYPE_CAT,
                                               G_CALLBACK(ptt_changed));
        }
    }

    if ((gtk_combo_box_get_active(GTK_COMBO_BOX(widget)) ==
         RIG_TYPE_TOGGLE_AUTO) ||
        (gtk_combo_box_get_active(GTK_COMBO_BOX(widget)) ==
         RIG_TYPE_TOGGLE_MAN))
    {
        if (ui != NULL)
            rig_pref_combo_set_active_deferred(ui, GTK_COMBO_BOX(ui->ptt),
                                           PTT_TYPE_CAT,
                                           G_CALLBACK(ptt_changed));
    }
}

static void radio_mode_changed(GtkWidget * widget, gpointer data)
{
    RigPrefUi *ui = data;
    (void)widget;

    if (ui != NULL && ui->ui_updating)
        return;

    rigcfg_update_vfo_sensitivity(ui);
}

static void autostart_toggled(GtkToggleButton *button, gpointer data)
{
    RigPrefUi *ui = data;

    if (ui != NULL && ui->ui_updating)
        return;
    update_autostart_sensitivity(ui, gtk_toggle_button_get_active(button));
    rig_pref_update_ok_button(ui);
}

static void rigctld_conn_changed(GtkComboBox *box, gpointer data)
{
    RigPrefUi *ui = data;
    gboolean enabled;

    (void)box;

    if (ui != NULL && ui->ui_updating)
        return;

    enabled = ui != NULL && ui->autostart != NULL &&
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ui->autostart));
    update_rigctld_connection_ui(ui, enabled);
    rig_pref_update_ok_button(ui);
}

static void radio_model_changed(GtkComboBox *box, gpointer data)
{
    RigPrefUi *ui = data;
    radio_model_t model;
    gint preset;
    gboolean enabled;

    if (ui != NULL && ui->ui_updating)
        return;

    rig_pref_ui_begin_update(ui, "radio_model_changed");

    model = gtk_combo_box_get_active(GTK_COMBO_BOX(box));
    preset = radio_model_to_hamlib_model(model);
    if (ui != NULL && ui->rigctld_model != NULL)
    {
        if (ui->last_radio_model == RADIO_MODEL_OTHER)
        {
            ui->rigctld_model_custom =
                gtk_spin_button_get_value_as_int(
                    GTK_SPIN_BUTTON(ui->rigctld_model));
        }

        if (preset > 0)
        {
            gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->rigctld_model), preset);
        }
        else if (ui->rigctld_model_custom > 0)
        {
            gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->rigctld_model),
                                      ui->rigctld_model_custom);
        }
    }

    ui->last_radio_model = model;
    if (preset > 0)
        apply_preset_rigctld_defaults(ui, model, TRUE);
    enabled = ui != NULL && ui->autostart != NULL &&
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ui->autostart));
    update_rigctld_model_sensitivity(ui, enabled);

    rig_pref_ui_end_update(ui, "radio_model_changed");
    rig_pref_update_ok_button(ui);
}

static GtkWidget *create_editor_widgets(RigPrefUi *ui, radio_conf_t * conf)
{
    GtkWidget      *basic_box;
    GtkWidget      *top_grid;
    GtkWidget      *radio_frame;
    GtkWidget      *radio_grid;
    GtkWidget      *operation_frame;
    GtkWidget      *operation_grid;
    GtkWidget      *peripheral_frame;
    GtkWidget      *peripheral_grid;
    GtkWidget      *advanced_table;
    GtkWidget      *label;
    GtkWidget      *test_button;
    GtkWidget      *vbox;
    GtkWidget      *notebook;

    vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);

    basic_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);

    top_grid = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(top_grid), 5);
    gtk_grid_set_column_homogeneous(GTK_GRID(top_grid), FALSE);
    gtk_grid_set_row_homogeneous(GTK_GRID(top_grid), FALSE);
    gtk_grid_set_column_spacing(GTK_GRID(top_grid), 5);
    gtk_grid_set_row_spacing(GTK_GRID(top_grid), 5);

    /* Config name */
    label = gtk_label_new(_("Name"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(top_grid), label, 0, 0, 1, 1);

    ui->name = gtk_entry_new();
    gtk_entry_set_max_length(GTK_ENTRY(ui->name), 25);
    g_signal_connect(ui->name, "changed", G_CALLBACK(name_changed), ui);
    g_signal_connect(ui->name, "changed", G_CALLBACK(rig_pref_on_field_changed), ui);
    gtk_widget_set_tooltip_text(ui->name,
                                _("Enter a short name for this configuration, "
                                  "e.g. IC910-1.\n"
                                  "Allowed characters: "
                                  "0..9, a..z, A..Z, - and _"));
    gtk_grid_attach(GTK_GRID(top_grid), ui->name, 1, 0, 3, 1);

    /* Host */
    label = gtk_label_new(_("Host"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(top_grid), label, 0, 1, 1, 1);

    ui->host = gtk_entry_new();
    gtk_entry_set_max_length(GTK_ENTRY(ui->host), 50);
    gtk_entry_set_text(GTK_ENTRY(ui->host), "127.0.0.1");
    g_signal_connect(ui->host, "changed", G_CALLBACK(rig_pref_on_field_changed), ui);
    gtk_widget_set_tooltip_text(ui->host,
                                _("Enter the host where rigctld is running. "
                                  "You can use both host name and IP address, "
                                  "e.g. 192.168.1.100\n\n"
                                  "If gpredict and rigctld are running on the "
                                  "same computer use 127.0.0.1"));
    gtk_grid_attach(GTK_GRID(top_grid), ui->host, 1, 1, 3, 1);

    /* port */
    label = gtk_label_new(_("Port"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(top_grid), label, 0, 2, 1, 1);

    ui->port = gtk_spin_button_new_with_range(1024, 65535, 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->port), 4532);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(ui->port), 0);
    g_signal_connect(ui->port, "value-changed",
                     G_CALLBACK(rig_pref_on_field_changed), ui);
    gtk_widget_set_tooltip_text(ui->port,
                                _("Enter the port number where rigctld is "
                                  "listening"));
    gtk_grid_attach(GTK_GRID(top_grid), ui->port, 1, 2, 1, 1);

    test_button = gtk_button_new_with_label(_("Test connection"));
    gtk_widget_set_tooltip_text(test_button,
                                _("Connect to rigctld and query the current frequency."));
    gtk_grid_attach(GTK_GRID(top_grid), test_button, 2, 2, 2, 1);
    g_signal_connect(test_button, "clicked",
                     G_CALLBACK(rigctld_test_connection_cb), ui);

    gtk_box_pack_start(GTK_BOX(basic_box), top_grid, FALSE, FALSE, 0);

    radio_frame = gtk_frame_new(_("Radio settings"));
    radio_grid = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(radio_grid), 5);
    gtk_grid_set_column_homogeneous(GTK_GRID(radio_grid), FALSE);
    gtk_grid_set_row_homogeneous(GTK_GRID(radio_grid), FALSE);
    gtk_grid_set_column_spacing(GTK_GRID(radio_grid), 5);
    gtk_grid_set_row_spacing(GTK_GRID(radio_grid), 5);
    gtk_container_add(GTK_CONTAINER(radio_frame), radio_grid);
    gtk_box_pack_start(GTK_BOX(basic_box), radio_frame, FALSE, FALSE, 0);

    /* radio type */
    label = gtk_label_new(_("Radio type"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(radio_grid), label, 0, 0, 1, 1);

    ui->type = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->type), _("RX only"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->type), _("TX only"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->type), _("Half-duplex"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->type), _("Full-duplex"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->type),
                                   _("FT817/857/897 (auto)"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->type),
                                   _("FT817/857/897 (manual)"));
    gtk_combo_box_set_active(GTK_COMBO_BOX(ui->type), RIG_TYPE_RX);
    g_signal_connect(ui->type, "changed", G_CALLBACK(type_changed), ui);
    g_signal_connect(ui->type, "changed", G_CALLBACK(rig_pref_on_field_changed), ui);
    gp_ui_quarantine_register_combo(ui->dialog, GTK_COMBO_BOX(ui->type));
    gtk_widget_set_tooltip_markup(ui->type,
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
    gtk_grid_attach(GTK_GRID(radio_grid), ui->type, 1, 0, 2, 1);

    /* radio model */
    label = gtk_label_new(_("Radio model"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(radio_grid), label, 0, 1, 1, 1);

    ui->radio_model = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->radio_model), _("Other"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->radio_model), _("IC-9700"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->radio_model), _("IC-705"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->radio_model), _("IC-905"));
    gtk_combo_box_set_active(GTK_COMBO_BOX(ui->radio_model), RADIO_MODEL_OTHER);
    gtk_widget_set_tooltip_text(ui->radio_model,
                                _("Select the radio model used for mode-specific behavior.\n"
                                  "No automatic guessing is performed."));
    gtk_grid_attach(GTK_GRID(radio_grid), ui->radio_model, 1, 1, 2, 1);
    g_signal_connect(ui->radio_model, "changed",
                     G_CALLBACK(radio_model_changed), ui);
    g_signal_connect(ui->radio_model, "changed",
                     G_CALLBACK(rig_pref_on_field_changed), ui);
    gp_ui_quarantine_register_combo(ui->dialog, GTK_COMBO_BOX(ui->radio_model));

    /* rigctld model */
    label = gtk_label_new(_("Rig model"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(radio_grid), label, 0, 2, 1, 1);

    ui->rigctld_model = gtk_spin_button_new_with_range(0, 99999, 1);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(ui->rigctld_model), 0);
    gtk_widget_set_tooltip_text(ui->rigctld_model,
                                _("Hamlib rig model number (e.g. 3081).\n"
                                  "Find your model id with: rigctl -l | "
                                  "grep -i 'IC-705'."));
    gtk_grid_attach(GTK_GRID(radio_grid), ui->rigctld_model, 1, 2, 1, 1);
    g_signal_connect(ui->rigctld_model, "value-changed",
                     G_CALLBACK(rigctld_model_changed), ui);
    g_signal_connect(ui->rigctld_model, "value-changed",
                     G_CALLBACK(rig_pref_on_field_changed), ui);

    operation_frame = gtk_frame_new(_("Operation"));
    operation_grid = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(operation_grid), 5);
    gtk_grid_set_column_homogeneous(GTK_GRID(operation_grid), FALSE);
    gtk_grid_set_row_homogeneous(GTK_GRID(operation_grid), FALSE);
    gtk_grid_set_column_spacing(GTK_GRID(operation_grid), 5);
    gtk_grid_set_row_spacing(GTK_GRID(operation_grid), 5);
    gtk_container_add(GTK_CONTAINER(operation_frame), operation_grid);
    gtk_box_pack_start(GTK_BOX(basic_box), operation_frame, FALSE, FALSE, 0);

    /* radio mode */
    label = gtk_label_new(_("Radio mode"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(operation_grid), label, 0, 0, 1, 1);

    ui->radio_mode = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->radio_mode), _("Simplex"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->radio_mode), _("Split"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->radio_mode),
                                   _("Full-duplex MAIN/SUB"));
    gtk_combo_box_set_active(GTK_COMBO_BOX(ui->radio_mode), RADIO_MODE_SIMPLEX);
    g_signal_connect(ui->radio_mode, "changed",
                     G_CALLBACK(radio_mode_changed), ui);
    g_signal_connect(ui->radio_mode, "changed",
                     G_CALLBACK(rig_pref_on_field_changed), ui);
    gp_ui_quarantine_register_combo(ui->dialog, GTK_COMBO_BOX(ui->radio_mode));
    gtk_widget_set_tooltip_text(ui->radio_mode,
                                _("Simplex uses one VFO; Split uses rigctld split.\n"
                                  "Full-duplex MAIN/SUB always updates both VFOs each cycle."));
    gtk_grid_attach(GTK_GRID(operation_grid), ui->radio_mode, 1, 0, 2, 1);

    /* VFO Up/Down */
    label = gtk_label_new(_("VFO Up/Down"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(operation_grid), label, 0, 1, 1, 1);

    ui->vfo = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->vfo), _("Not applicable"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->vfo),
                                   _("MAIN \342\206\221 / SUB \342\206\223"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->vfo),
                                   _("SUB \342\206\221 / MAIN \342\206\223"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->vfo),
                                   _("A \342\206\221 / B \342\206\223"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->vfo),
                                   _("B \342\206\221 / A \342\206\223"));
    gtk_combo_box_set_active(GTK_COMBO_BOX(ui->vfo), 0);
    g_signal_connect(ui->vfo, "changed", G_CALLBACK(vfo_changed), ui);
    g_signal_connect(ui->vfo, "changed", G_CALLBACK(rig_pref_on_field_changed), ui);
    gp_ui_quarantine_register_combo(ui->dialog, GTK_COMBO_BOX(ui->vfo));
    gtk_widget_set_tooltip_markup(ui->vfo,
                                  _
                                   ("Select which VFO to use for uplink and downlink. "
                                    "This setting is used for full-duplex radios only, "
                                    "such as the IC-910H, FT-847 and the TS-2000.\n\n"
                                    "<b>IC-910H:</b> MAIN\342\206\221 / SUB\342\206\223\n"
                                    "<b>FT-847:</b> SUB\342\206\221 / MAIN\342\206\223\n"
                                    "<b>TS-2000:</b> B\342\206\221 / A\342\206\223"));
    gtk_grid_attach(GTK_GRID(operation_grid), ui->vfo, 1, 1, 2, 1);

    /* ptt */
    label = gtk_label_new(_("PTT status"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(operation_grid), label, 0, 2, 1, 1);

    ui->ptt = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->ptt), _("None"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->ptt), _("Read PTT"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->ptt), _("Read DCD"));
    gtk_combo_box_set_active(GTK_COMBO_BOX(ui->ptt), 0);
    g_signal_connect(ui->ptt, "changed", G_CALLBACK(ptt_changed), ui);
    g_signal_connect(ui->ptt, "changed", G_CALLBACK(rig_pref_on_field_changed), ui);
    gp_ui_quarantine_register_combo(ui->dialog, GTK_COMBO_BOX(ui->ptt));
    gtk_widget_set_tooltip_markup(ui->ptt,
                                  _("Select PTT type.\n\n"
                                    "<b>None:</b>\nDon't read PTT status from this radio.\n\n"
                                    "<b>Read PTT:</b>\nRead PTT status using get_ptt CAT command. "
                                    "You have to check that your radio and hamlib supports this.\n\n"
                                    "<b>Read DCD:</b>\nRead PTT status using get_dcd command. "
                                    "This can be used if your radio does not support the read_ptt "
                                    "CAT command and you have a special interface that can "
                                    "read squelch status and send it via CTS."));
    gtk_grid_attach(GTK_GRID(operation_grid), ui->ptt, 1, 2, 2, 1);

    /* Downconverter LO frequency */
    label = gtk_label_new(_("LO Down"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(operation_grid), label, 0, 3, 1, 1);

    ui->lo = gtk_spin_button_new_with_range(-10000, 10000, 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->lo), 0);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(ui->lo), 0);
    g_signal_connect(ui->lo, "value-changed",
                     G_CALLBACK(rig_pref_on_field_changed), ui);
    gtk_widget_set_tooltip_text(ui->lo,
                                _
                                ("Enter the frequency of the local oscillator "
                                 " of the downconverter, if any."));
    gtk_grid_attach(GTK_GRID(operation_grid), ui->lo, 1, 3, 1, 1);

    label = gtk_label_new(_("MHz"));
    g_object_set(label, "xalign", 0.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(operation_grid), label, 2, 3, 1, 1);

    /* Upconverter LO frequency */
    label = gtk_label_new(_("LO Up"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(operation_grid), label, 0, 4, 1, 1);

    ui->loup = gtk_spin_button_new_with_range(-10000, 10000, 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->loup), 0);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(ui->loup), 0);
    g_signal_connect(ui->loup, "value-changed",
                     G_CALLBACK(rig_pref_on_field_changed), ui);
    gtk_widget_set_tooltip_text(ui->loup,
                                _
                                ("Enter the frequency of the local oscillator "
                                 "of the upconverter, if any."));
    gtk_grid_attach(GTK_GRID(operation_grid), ui->loup, 1, 4, 1, 1);

    label = gtk_label_new(_("MHz"));
    g_object_set(label, "xalign", 0.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(operation_grid), label, 2, 4, 1, 1);

    peripheral_frame = gtk_frame_new(_("Peripheral settings"));
    peripheral_grid = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(peripheral_grid), 5);
    gtk_grid_set_column_homogeneous(GTK_GRID(peripheral_grid), FALSE);
    gtk_grid_set_row_homogeneous(GTK_GRID(peripheral_grid), FALSE);
    gtk_grid_set_column_spacing(GTK_GRID(peripheral_grid), 5);
    gtk_grid_set_row_spacing(GTK_GRID(peripheral_grid), 5);
    gtk_container_add(GTK_CONTAINER(peripheral_frame), peripheral_grid);
    gtk_box_pack_start(GTK_BOX(basic_box), peripheral_frame, FALSE, FALSE, 0);

    /* AOS / LOS signalling */
    label = gtk_label_new(_("Signalling"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(peripheral_grid), label, 0, 0, 1, 1);

    ui->sigaos = gtk_check_button_new_with_label(_("AOS"));
    g_signal_connect(ui->sigaos, "toggled",
                     G_CALLBACK(rig_pref_on_field_changed), ui);
    gtk_grid_attach(GTK_GRID(peripheral_grid), ui->sigaos, 1, 0, 1, 1);
    gtk_widget_set_tooltip_text(ui->sigaos,
                                _("Enable AOS signalling for this radio."));

    ui->siglos = gtk_check_button_new_with_label(_("LOS"));
    g_signal_connect(ui->siglos, "toggled",
                     G_CALLBACK(rig_pref_on_field_changed), ui);
    gtk_grid_attach(GTK_GRID(peripheral_grid), ui->siglos, 2, 0, 1, 1);
    gtk_widget_set_tooltip_text(ui->siglos,
                                _("Enable LOS signalling for this radio."));

    /* Auto-start rigctld */
    label = gtk_label_new(_("Auto-start local rigctld"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(peripheral_grid), label, 0, 1, 1, 1);

    ui->autostart = gtk_check_button_new_with_label(_("Enable"));
    gtk_grid_attach(GTK_GRID(peripheral_grid), ui->autostart, 1, 1, 1, 1);
    gtk_widget_set_tooltip_text(ui->autostart,
                                _("Start rigctld automatically when engaging "
                                  "if it is not already running."));
    g_signal_connect(ui->autostart, "toggled", G_CALLBACK(autostart_toggled), ui);
    g_signal_connect(ui->autostart, "toggled", G_CALLBACK(rig_pref_on_field_changed), ui);

    /* rigctld connection type */
    label = gtk_label_new(_("Connection type"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(peripheral_grid), label, 0, 2, 1, 1);

    ui->rigctld_conn_type = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->rigctld_conn_type),
                                   _("Serial (USB)"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->rigctld_conn_type),
                                   _("TCP (LAN)"));
    gtk_combo_box_set_active(GTK_COMBO_BOX(ui->rigctld_conn_type),
                             RIGCTLD_CONN_SERIAL);
    gtk_widget_set_tooltip_text(ui->rigctld_conn_type,
                                _("Select how rigctld connects to the radio."));
    gtk_grid_attach(GTK_GRID(peripheral_grid), ui->rigctld_conn_type, 1, 2, 2, 1);
    g_signal_connect(ui->rigctld_conn_type, "changed",
                     G_CALLBACK(rigctld_conn_changed), ui);
    g_signal_connect(ui->rigctld_conn_type, "changed",
                     G_CALLBACK(rig_pref_on_field_changed), ui);
    gp_ui_quarantine_register_combo(ui->dialog, GTK_COMBO_BOX(ui->rigctld_conn_type));

    /* rigctld baud */
    ui->rigctld_baud_label = gtk_label_new(_("Baud"));
    g_object_set(ui->rigctld_baud_label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(peripheral_grid), ui->rigctld_baud_label, 0, 3, 1, 1);

    ui->rigctld_baud = gtk_spin_button_new_with_range(0, 1000000, 1);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(ui->rigctld_baud), 0);
    gtk_widget_set_tooltip_text(ui->rigctld_baud,
                                _("Serial baud rate for rigctld (e.g. 19200)."));
    gtk_grid_attach(GTK_GRID(peripheral_grid), ui->rigctld_baud, 1, 3, 1, 1);
    g_signal_connect(ui->rigctld_baud, "value-changed",
                     G_CALLBACK(rig_pref_on_field_changed), ui);

    /* Advanced rigctld options */
    advanced_table = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(advanced_table), 5);
    gtk_grid_set_column_spacing(GTK_GRID(advanced_table), 5);
    gtk_grid_set_row_spacing(GTK_GRID(advanced_table), 5);

    /* rigctld auto power-on */
    label = gtk_label_new(_("Auto power-on"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(advanced_table), label, 0, 0, 1, 1);

    ui->rigctld_auto_power_on = gtk_check_button_new_with_label(_("Enable"));
    gtk_grid_attach(GTK_GRID(advanced_table), ui->rigctld_auto_power_on, 1, 0, 1, 1);
    gtk_widget_set_tooltip_text(ui->rigctld_auto_power_on,
                                _("Enable rigctld auto power-on if supported."));

    /* rigctld path */
    label = gtk_label_new(_("rigctld path"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(advanced_table), label, 0, 1, 1, 1);

    ui->rigctld_path = gtk_entry_new();
    gtk_entry_set_max_length(GTK_ENTRY(ui->rigctld_path), 200);
    gtk_widget_set_tooltip_text(ui->rigctld_path,
                                _("Path to rigctld binary (leave empty to use bundled rigctld or PATH)."));
    gtk_grid_attach(GTK_GRID(advanced_table), ui->rigctld_path, 1, 1, 3, 1);

    /* rigctld CI-V address */
    label = gtk_label_new(_("CI-V addr"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(advanced_table), label, 0, 2, 1, 1);

    ui->rigctld_civaddr = gtk_entry_new();
    gtk_entry_set_max_length(GTK_ENTRY(ui->rigctld_civaddr), 16);
    gtk_widget_set_tooltip_text(ui->rigctld_civaddr,
                                _("Optional CI-V address (e.g. 0xA2)."));
    gtk_grid_attach(GTK_GRID(advanced_table), ui->rigctld_civaddr, 1, 2, 2, 1);

    /* rigctld extra args */
    label = gtk_label_new(_("Extra args"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(advanced_table), label, 0, 3, 1, 1);

    ui->rigctld_extra_args = gtk_entry_new();
    gtk_entry_set_max_length(GTK_ENTRY(ui->rigctld_extra_args), 200);
    gtk_widget_set_tooltip_text(ui->rigctld_extra_args,
                                _("Extra rigctld arguments (optional)."));
    gtk_grid_attach(GTK_GRID(advanced_table), ui->rigctld_extra_args, 1, 3, 3, 1);

    /* rigctld device (manual override) */
    ui->rigctld_device_label = gtk_label_new(_("Serial device (manual)"));
    g_object_set(ui->rigctld_device_label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(advanced_table), ui->rigctld_device_label, 0, 4, 1, 1);

    ui->rigctld_device = gtk_entry_new();
    gtk_entry_set_max_length(GTK_ENTRY(ui->rigctld_device), 200);
    gtk_widget_set_tooltip_text(ui->rigctld_device,
                                _("Manual serial device override (e.g. /dev/cu.usbserial-1234)."));
    gtk_grid_attach(GTK_GRID(advanced_table), ui->rigctld_device, 1, 4, 2, 1);

    ui->rigctld_device_find = gtk_button_new_with_label(_("Choose..."));
    gtk_widget_set_tooltip_text(ui->rigctld_device_find,
                                _("Choose a serial port from the available list."));
    gtk_grid_attach(GTK_GRID(advanced_table), ui->rigctld_device_find, 3, 4, 1, 1);
    g_signal_connect(ui->rigctld_device_find, "clicked",
                     G_CALLBACK(rigctld_find_port_cb), ui);

    notebook = gtk_notebook_new();
    gtk_notebook_append_page(GTK_NOTEBOOK(notebook),
                             basic_box,
                             gtk_label_new(_("Basic")));
    gtk_notebook_append_page(GTK_NOTEBOOK(notebook),
                             advanced_table,
                             gtk_label_new(_("Advanced")));

    if (conf->name != NULL)
        update_widgets(ui, conf);
    else
        update_autostart_sensitivity(ui, TRUE);
    rigcfg_update_vfo_sensitivity(ui);
    rig_pref_update_ok_button(ui);

    gtk_box_pack_start(GTK_BOX(vbox), notebook, FALSE, FALSE, 0);
    gtk_widget_show_all(vbox);

    return vbox;
}

/* Apply changes. Returns TRUE if things are ok, FALSE otherwise */
static gboolean apply_changes(RigPrefUi *ui, radio_conf_t * conf)
{
    radio_model_t selected_model;
    radio_mode_t selected_mode;
    gchar *allowed = NULL;
    gchar *detail = NULL;

    if (ui == NULL || conf == NULL)
        return FALSE;

    selected_model = gtk_combo_box_get_active(GTK_COMBO_BOX(ui->radio_model));
    selected_mode = gtk_combo_box_get_active(GTK_COMBO_BOX(ui->radio_mode));

    if (!radio_mode_allowed_for_model(selected_model, selected_mode))
    {
        allowed = radio_mode_allowed_string(selected_model);
        detail = g_strdup_printf("Model: %s\nMode: %s\nAllowed: %s",
                                 radio_model_to_string(selected_model),
                                 radio_mode_to_string(selected_mode),
                                 allowed ? allowed : "none");
        rig_pref_show_dialog(ui, GTK_MESSAGE_ERROR,
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

    conf->name = g_strdup(gtk_entry_get_text(GTK_ENTRY(ui->name)));

    /* host */
    if (conf->host)
        g_free(conf->host);

    conf->host = g_strdup(gtk_entry_get_text(GTK_ENTRY(ui->host)));

    /* port */
    conf->port = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(ui->port));

    /* lo down freq */
    conf->lo = 1000000.0 * gtk_spin_button_get_value(GTK_SPIN_BUTTON(ui->lo));

    /* lo up freq */
    conf->loup = 1000000.0 * gtk_spin_button_get_value(GTK_SPIN_BUTTON(ui->loup));

    /* rig type */
    conf->type = gtk_combo_box_get_active(GTK_COMBO_BOX(ui->type));

    /* radio model */
    conf->radio_model = selected_model;

    /* radio mode */
    conf->radio_mode = selected_mode;

    /* ptt */
    conf->ptt = gtk_combo_box_get_active(GTK_COMBO_BOX(ui->ptt));

    /* vfo up/down */
    switch (gtk_combo_box_get_active(GTK_COMBO_BOX(ui->vfo)))
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
    conf->signal_aos =
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ui->sigaos));
    conf->signal_los =
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ui->siglos));
    conf->supports_dual_vfo_sat =
        (conf->radio_mode == RADIO_MODE_FULL_DUPLEX_MAIN_SUB);

    /* rigctld auto-start */
    conf->rigctld_autostart =
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ui->autostart));
    conf->rigctld_auto_power_on =
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ui->rigctld_auto_power_on));

    if (conf->rigctld_path)
        g_free(conf->rigctld_path);
    conf->rigctld_path =
        g_strdup(gtk_entry_get_text(GTK_ENTRY(ui->rigctld_path)));

    conf->rigctld_model =
        gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(ui->rigctld_model));
    if (conf->rigctld_model <= 0)
    {
        gint preset = radio_model_to_hamlib_model(conf->radio_model);

        /* Avoid saving an empty rig model when a preset exists. */
        if (preset > 0)
            conf->rigctld_model = preset;
    }

    conf->rigctld_conn =
        gtk_combo_box_get_active(GTK_COMBO_BOX(ui->rigctld_conn_type));

    if (conf->rigctld_device)
        g_free(conf->rigctld_device);
    conf->rigctld_device =
        g_strdup(gtk_entry_get_text(GTK_ENTRY(ui->rigctld_device)));

    conf->rigctld_baud =
        gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(ui->rigctld_baud));

    if (conf->rigctld_civaddr)
        g_free(conf->rigctld_civaddr);
    conf->rigctld_civaddr =
        g_strdup(gtk_entry_get_text(GTK_ENTRY(ui->rigctld_civaddr)));

    if (conf->rigctld_extra_args)
        g_free(conf->rigctld_extra_args);
    conf->rigctld_extra_args =
        g_strdup(gtk_entry_get_text(GTK_ENTRY(ui->rigctld_extra_args)));

    return TRUE;
}

static void rig_pref_dialog_response(GtkDialog *dialog,
                                     gint response,
                                     gpointer user_data)
{
    RigPrefDialogState *state = user_data;

    if (state == NULL)
        return;

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "rig-pref editor response=%d", response);
    switch (response)
    {
    case GTK_RESPONSE_OK:
        if (apply_changes(state->ui, state->conf))
        {
            if (state->done)
                state->done(state->conf, TRUE, state->user_data);
            state->finished = TRUE;
            state->done = NULL;
            gtk_widget_destroy(GTK_WIDGET(dialog));
        }
        break;

    case GTK_RESPONSE_REJECT:
        clear_widgets(state->ui);
        break;

    default:
        if (state->done)
            state->done(state->conf, FALSE, state->user_data);
        state->finished = TRUE;
        state->done = NULL;
        gtk_widget_destroy(GTK_WIDGET(dialog));
        break;
    }
}

static void rig_pref_dialog_destroy(GtkWidget *widget, gpointer user_data)
{
    RigPrefDialogState *state = user_data;
    RigPrefUi *ui;

    (void)widget;

    if (state == NULL)
        return;

    ui = state->ui;
    if (!state->finished && state->done)
        state->done(state->conf, FALSE, state->user_data);
    sat_log_log(SAT_LOG_LEVEL_DEBUG, "rig-pref editor destroyed");
    if (ui != NULL)
        ui->ui_updating = FALSE;

    if (ui != NULL && ui->pending_ui_refresh_id != 0)
    {
        g_source_remove(ui->pending_ui_refresh_id);
        ui->pending_ui_refresh_id = 0;
    }

    g_free(state);
}

/* Add or edit a radio configuration */
void sat_pref_rig_editor_run(radio_conf_t *conf,
                             RigPrefEditorDoneFunc done,
                             gpointer user_data)
{
    RigPrefDialogState *state;
    RigPrefUi *ui;
    GtkWidget *dialog;

    /* create dialog and add contents */
    dialog = gtk_dialog_new_with_buttons(_("Edit radio configuration"),
                                         GTK_WINDOW(window),
                                         GTK_DIALOG_DESTROY_WITH_PARENT,
                                         "_Clear", GTK_RESPONSE_REJECT,
                                         "_Cancel", GTK_RESPONSE_CANCEL,
                                         "_Ok", GTK_RESPONSE_OK,
                                         NULL);

    /* disable OK button to begin with */
    gtk_dialog_set_response_sensitive(GTK_DIALOG(dialog),
                                      GTK_RESPONSE_OK, FALSE);

    ui = g_new0(RigPrefUi, 1);
    ui->dialog = dialog;
    ui->last_radio_model = RADIO_MODEL_OTHER;
    g_object_set_data_full(G_OBJECT(dialog), "rig_pref_ui", ui, g_free);
    gp_ui_quarantine_install(dialog);
    gtk_container_add(GTK_CONTAINER
                      (gtk_dialog_get_content_area(GTK_DIALOG(dialog))),
                      create_editor_widgets(ui, conf));

    state = g_new0(RigPrefDialogState, 1);
    state->conf = conf;
    state->done = done;
    state->user_data = user_data;
    state->ui = ui;

    sat_log_log(SAT_LOG_LEVEL_DEBUG, "rig-pref editor created");
    g_signal_connect(dialog, "response",
                     G_CALLBACK(rig_pref_dialog_response), state);
    g_signal_connect(dialog, "destroy",
                     G_CALLBACK(rig_pref_dialog_destroy), state);

    gtk_widget_show_all(dialog);
    sat_log_log(SAT_LOG_LEVEL_DEBUG, "rig-pref editor shown");
}
