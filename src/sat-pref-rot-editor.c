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
#include "ui-popup-quarantine.h"


extern GtkWidget *window;       /* dialog window defined in sat-pref.c */

typedef struct {
    GtkWidget *dialog;       /* dialog window */
    GtkWidget *name;         /* Configuration name */
    GtkWidget *host;         /* host name or IP */
    GtkWidget *port;         /* port number */
    GtkWidget *autostart;
    GtkWidget *protocol;
    GtkWidget *hamlib_model;
    GtkWidget *hamlib_model_label;
    gint       hamlib_model_custom;
    GtkWidget *baud;
    GtkWidget *device_combo;
    GtkWidget *device_refresh;
    GtkWidget *device_manual;
    GtkWidget *device_manual_revealer;
    GtkWidget *device_autopick;
    GtkWidget *device_status;
    GtkWidget *aztype;
    GtkWidget *minaz;
    GtkWidget *maxaz;
    GtkWidget *minel;
    GtkWidget *maxel;
    GtkWidget *minel_label;
    GtkWidget *maxel_label;
    GtkWidget *axismode;
    GtkWidget *disable_pos_feedback;
    gboolean device_scan_in_progress;
    gboolean ui_updating;
    guint pending_ui_refresh_id;
    GSList *device_cache;
} RotPrefUi;
static const gchar *ROT_DEVICE_OTHER_ID = "other";

typedef struct {
    rotor_conf_t          *conf;
    RotPrefEditorDoneFunc  done;
    gpointer               user_data;
    gboolean               finished;
    RotPrefUi             *ui;
} RotPrefDialogState;

static void update_el_limits_sensitivity(RotPrefUi *ui);
static void axismode_changed_cb(GtkComboBox *box, gpointer data);
static void name_changed(GtkWidget *widget, gpointer data);
static gboolean rot_pref_form_is_valid(RotPrefUi *ui);
static void rot_pref_update_ok_button(RotPrefUi *ui);
static void rot_pref_on_field_changed(GtkWidget *widget, gpointer data);
static void protocol_changed_cb(GtkComboBox *box, gpointer data);
static void rot_pref_update_hamlib_model_ui(RotPrefUi *ui,
                                            rot_protocol_t proto,
                                            gint stored_model);
static void hamlib_model_changed_cb(GtkSpinButton *spin, gpointer data);
static void device_combo_changed_cb(GtkComboBox *box, gpointer data);
static void device_refresh_cb(GtkButton *button, gpointer data);
static void device_autopick_toggled_cb(GtkToggleButton *toggle, gpointer data);
static void aztype_changed_cb(GtkComboBox *box, gpointer data);
static void rot_pref_schedule_device_combo_refresh(RotPrefUi *ui);

static void rot_pref_ui_begin_update(RotPrefUi *ui, const gchar *reason)
{
    if (ui == NULL)
        return;

    ui->ui_updating = TRUE;
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "rot-pref ui_begin_update %s",
                reason ? reason : "(none)");
}

static void rot_pref_ui_end_update(RotPrefUi *ui, const gchar *reason)
{
    if (ui == NULL)
        return;

    ui->ui_updating = FALSE;
    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "rot-pref ui_end_update %s",
                reason ? reason : "(none)");
}

static gboolean rot_pref_combo_popup_shown(GtkComboBox *combo)
{
    gboolean shown = FALSE;

    if (combo == NULL)
        return FALSE;

    if (g_object_class_find_property(G_OBJECT_GET_CLASS(combo), "popup-shown"))
        g_object_get(combo, "popup-shown", &shown, NULL);

    return shown;
}


static gboolean rot_pref_form_is_valid(RotPrefUi *ui)
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

static void rot_pref_update_ok_button(RotPrefUi *ui)
{
    if (ui == NULL || ui->dialog == NULL)
        return;

    gtk_dialog_set_response_sensitive(GTK_DIALOG(ui->dialog),
                                      GTK_RESPONSE_OK,
                                      rot_pref_form_is_valid(ui));
}

static void rot_pref_on_field_changed(GtkWidget *widget, gpointer data)
{
    RotPrefUi *ui = data;

    (void)widget;

    if (ui != NULL && ui->ui_updating)
        return;

    rot_pref_update_ok_button(ui);
}

static void rot_pref_combo_set_active(GtkComboBox *combo, gint index,
                                      GCallback cb)
{
    if (combo == NULL)
        return;

    g_signal_handlers_block_by_func(combo, (gpointer)cb, NULL);
    gtk_combo_box_set_active(combo, index);
    g_signal_handlers_unblock_by_func(combo, (gpointer)cb, NULL);
}

static void rot_pref_combo_set_active_id(GtkComboBox *combo, const gchar *id,
                                         GCallback cb)
{
    if (combo == NULL)
        return;

    g_signal_handlers_block_by_func(combo, (gpointer)cb, NULL);
    gtk_combo_box_set_active_id(combo, id);
    g_signal_handlers_unblock_by_func(combo, (gpointer)cb, NULL);
}

static void rot_pref_message_response(GtkDialog *dialog,
                                      gint response,
                                      gpointer user_data)
{
    (void)user_data;

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "rot-pref message dialog response=%d", response);
    gtk_widget_destroy(GTK_WIDGET(dialog));
}

static void rot_pref_message_destroy(GtkWidget *widget, gpointer user_data)
{
    (void)widget;
    (void)user_data;

    sat_log_log(SAT_LOG_LEVEL_DEBUG, "rot-pref message dialog destroyed");
}

static void rot_pref_show_dialog(RotPrefUi *ui,
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
                "rot-pref message dialog show primary=%s",
                primary ? primary : "(null)");
    g_signal_connect(msg, "response",
                     G_CALLBACK(rot_pref_message_response), NULL);
    g_signal_connect(msg, "destroy",
                     G_CALLBACK(rot_pref_message_destroy), NULL);
    gtk_widget_show(msg);
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
                (g_strrstr(lower, "wch") != NULL) ||
                (g_strrstr(lower, "ftdi") != NULL);
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

static gboolean rot_pref_is_other_id(const gchar *id)
{
    return g_strcmp0(id, ROT_DEVICE_OTHER_ID) == 0;
}

static void rot_pref_update_device_ui_state(RotPrefUi *ui)
{
    gboolean autopick = FALSE;
    const gchar *active_id = NULL;
    gboolean show_manual = FALSE;

    if (ui == NULL)
        return;

    if (ui->device_autopick)
        autopick = gtk_toggle_button_get_active(
            GTK_TOGGLE_BUTTON(ui->device_autopick));

    if (ui->device_combo)
        active_id = gtk_combo_box_get_active_id(GTK_COMBO_BOX(ui->device_combo));

    if (!autopick)
    {
        show_manual = rot_pref_is_other_id(active_id);
        if (!show_manual && active_id == NULL && ui->device_manual)
            show_manual = gtk_entry_get_text_length(GTK_ENTRY(ui->device_manual)) > 0;
    }

    if (ui->device_combo)
        gtk_widget_set_sensitive(ui->device_combo, !autopick);

    if (ui->device_manual)
        gtk_widget_set_sensitive(ui->device_manual, show_manual);

    if (ui->device_manual_revealer)
        gtk_revealer_set_reveal_child(
            GTK_REVEALER(ui->device_manual_revealer), show_manual);
}

static const gchar *rot_protocol_id(rot_protocol_t protocol)
{
    switch (protocol)
    {
    case ROT_PROTOCOL_SPID_ROT1PROG:
        return "rot1prog";
    case ROT_PROTOCOL_SPID_ROT2PROG:
        return "rot2prog";
    case ROT_PROTOCOL_OTHER:
        return "other";
    case ROT_PROTOCOL_GS232B:
    default:
        return "gs232b";
    }
}

static rot_protocol_t rot_protocol_from_id(const gchar *id)
{
    if (g_strcmp0(id, "rot1prog") == 0)
        return ROT_PROTOCOL_SPID_ROT1PROG;
    if (g_strcmp0(id, "rot2prog") == 0)
        return ROT_PROTOCOL_SPID_ROT2PROG;
    if (g_strcmp0(id, "other") == 0)
        return ROT_PROTOCOL_OTHER;
    return ROT_PROTOCOL_GS232B;
}

static rot_protocol_t rot_protocol_from_combo(GtkComboBox *combo)
{
    const gchar *id = NULL;

    if (combo != NULL)
        id = gtk_combo_box_get_active_id(combo);

    return rot_protocol_from_id(id);
}

static void rot_pref_update_hamlib_model_ui(RotPrefUi *ui,
                                            rot_protocol_t proto,
                                            gint stored_model)
{
    gboolean editable = FALSE;
    gint model = 0;
    gboolean was_updating = FALSE;

    if (ui == NULL || ui->hamlib_model == NULL)
        return;

    if (proto == ROT_PROTOCOL_OTHER)
    {
        editable = TRUE;
        if (stored_model > 0)
            ui->hamlib_model_custom = stored_model;
        if (ui->hamlib_model_custom > 0)
            model = ui->hamlib_model_custom;
        else
            model = rot_protocol_to_hamlib_model(ROT_PROTOCOL_GS232B);
    }
    else
    {
        model = rot_protocol_to_hamlib_model(proto);
        if (model <= 0)
            model = rot_protocol_to_hamlib_model(ROT_PROTOCOL_GS232B);
    }

    was_updating = ui->ui_updating;
    if (!was_updating)
        rot_pref_ui_begin_update(ui, "hamlib_model");

    g_signal_handlers_block_by_func(ui->hamlib_model,
                                    (gpointer)G_CALLBACK(hamlib_model_changed_cb),
                                    ui);
    g_signal_handlers_block_by_func(ui->hamlib_model,
                                    (gpointer)G_CALLBACK(rot_pref_on_field_changed),
                                    ui);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->hamlib_model), model);
    g_signal_handlers_unblock_by_func(ui->hamlib_model,
                                      (gpointer)G_CALLBACK(rot_pref_on_field_changed),
                                      ui);
    g_signal_handlers_unblock_by_func(ui->hamlib_model,
                                      (gpointer)G_CALLBACK(hamlib_model_changed_cb),
                                      ui);

    gtk_widget_set_sensitive(ui->hamlib_model, editable);
    if (ui->hamlib_model_label)
        gtk_widget_set_sensitive(ui->hamlib_model_label, editable);

    if (!was_updating)
        rot_pref_ui_end_update(ui, "hamlib_model");
}

static void hamlib_model_changed_cb(GtkSpinButton *spin, gpointer data)
{
    RotPrefUi *ui = data;

    if (ui == NULL || ui->ui_updating)
        return;

    if (ui->protocol &&
        rot_protocol_from_combo(GTK_COMBO_BOX(ui->protocol)) == ROT_PROTOCOL_OTHER)
    {
        ui->hamlib_model_custom =
            gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(spin));
    }

    rot_pref_on_field_changed(GTK_WIDGET(spin), data);
}

static void rot_pref_update_device_status(RotPrefUi *ui,
                                          GSList *list,
                                          gboolean autopick)
{
    if (ui == NULL || ui->device_status == NULL)
        return;

    if (list == NULL)
    {
        gtk_label_set_text(GTK_LABEL(ui->device_status),
                           autopick
                           ? _("No USB serial devices found")
                           : _("No serial devices found."));
    }
    else
        gtk_label_set_text(GTK_LABEL(ui->device_status), "");
}

static void rot_pref_update_device_combo(RotPrefUi *ui,
                                         GSList *list,
                                         const gchar *current_id,
                                         gboolean autopick,
                                         const gchar *manual)
{
    gchar *selected = NULL;
    const gchar *current = current_id;
    GtkListStore *store = NULL;
    GtkTreeIter iter;
    guint rows = 0;

    if (ui == NULL || ui->device_combo == NULL)
        return;

    if (rot_pref_combo_popup_shown(GTK_COMBO_BOX(ui->device_combo)))
    {
        rot_pref_schedule_device_combo_refresh(ui);
        return;
    }

    store = gtk_list_store_new(2, G_TYPE_STRING, G_TYPE_STRING);
    for (GSList *iter_list = list; iter_list != NULL; iter_list = iter_list->next)
    {
        const gchar *path = iter_list->data;
        if (path == NULL)
            continue;
        gtk_list_store_append(store, &iter);
        gtk_list_store_set(store, &iter,
                           0, path,
                           1, path,
                           -1);
        rows++;
    }

    gtk_list_store_append(store, &iter);
    gtk_list_store_set(store, &iter,
                       0, _("Other..."),
                       1, ROT_DEVICE_OTHER_ID,
                       -1);
    rows++;

    if (rot_pref_is_other_id(current))
        current = NULL;

    if (autopick)
        selected = rot_pref_pick_best_device(list, current);
    else if ((manual && *manual) || rot_pref_is_other_id(current_id))
        selected = g_strdup(ROT_DEVICE_OTHER_ID);
    else if (current && rot_pref_list_contains(list, current))
        selected = g_strdup(current);

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "rot-pref device combo rebuild rows=%u selected=%s autopick=%d",
                rows,
                selected ? selected : "(none)",
                autopick ? 1 : 0);

    rot_pref_ui_begin_update(ui, "device_combo");
    g_signal_handlers_block_by_func(ui->device_combo,
                                    (gpointer)G_CALLBACK(device_combo_changed_cb),
                                    NULL);
    gtk_combo_box_set_model(GTK_COMBO_BOX(ui->device_combo), GTK_TREE_MODEL(store));
    gtk_combo_box_set_id_column(GTK_COMBO_BOX(ui->device_combo), 1);
    if (rows > 0)
    {
        if (selected)
            gtk_combo_box_set_active_id(GTK_COMBO_BOX(ui->device_combo), selected);
        else
            gtk_combo_box_set_active(GTK_COMBO_BOX(ui->device_combo), -1);
    }
    g_signal_handlers_unblock_by_func(ui->device_combo,
                                      (gpointer)G_CALLBACK(device_combo_changed_cb),
                                      NULL);
    rot_pref_ui_end_update(ui, "device_combo");
    g_object_unref(store);

    rot_pref_update_device_status(ui, list, autopick);
    rot_pref_update_device_ui_state(ui);
    g_free(selected);
}

static gboolean rot_pref_device_combo_refresh_idle(gpointer data)
{
    RotPrefUi *ui = data;
    const gchar *current_id = NULL;
    const gchar *manual_text = NULL;
    gboolean autopick = TRUE;

    if (ui == NULL)
        return G_SOURCE_REMOVE;

    if (ui->device_combo &&
        rot_pref_combo_popup_shown(GTK_COMBO_BOX(ui->device_combo)))
        return G_SOURCE_CONTINUE;

    ui->pending_ui_refresh_id = 0;

    if (ui->device_combo)
        current_id = gtk_combo_box_get_active_id(GTK_COMBO_BOX(ui->device_combo));
    if (ui->device_autopick)
        autopick = gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ui->device_autopick));
    if (ui->device_manual)
        manual_text = gtk_entry_get_text(GTK_ENTRY(ui->device_manual));

    rot_pref_update_device_combo(ui, ui->device_cache, current_id, autopick, manual_text);
    return G_SOURCE_REMOVE;
}

static void rot_pref_schedule_device_combo_refresh(RotPrefUi *ui)
{
    if (ui == NULL)
        return;

    if (ui->pending_ui_refresh_id != 0)
        return;

    sat_log_log(SAT_LOG_LEVEL_DEBUG, "rot-pref schedule device combo refresh");
    ui->pending_ui_refresh_id =
        g_idle_add(rot_pref_device_combo_refresh_idle, ui);
}

typedef struct {
    GWeakRef dialog_ref;
    GSList  *list;
    gchar   *current_id;
    gchar   *manual;
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
    GtkWidget *dialog;
    RotPrefUi *ui;

    dialog = result ? g_weak_ref_get(&result->dialog_ref) : NULL;
    ui = dialog ? g_object_get_data(G_OBJECT(dialog), "rot_pref_ui") : NULL;

    if (ui == NULL)
    {
        if (result && result->list)
            gp_serial_free_candidates(result->list);
        if (dialog)
            g_object_unref(dialog);
        if (result)
            g_weak_ref_clear(&result->dialog_ref);
        g_free(result ? result->current_id : NULL);
        g_free(result ? result->manual : NULL);
        g_free(result);
        return G_SOURCE_REMOVE;
    }

    ui->device_scan_in_progress = FALSE;
    if (ui->device_refresh)
        gtk_widget_set_sensitive(ui->device_refresh, TRUE);

    if (ui->device_cache)
        gp_serial_free_candidates(ui->device_cache);

    ui->device_cache = result ? result->list : NULL;

    rot_pref_update_device_combo(ui,
                                 ui->device_cache,
                                 result ? result->current_id : NULL,
                                 result ? result->autopick : TRUE,
                                 result ? result->manual : NULL);

    g_free(result ? result->current_id : NULL);
    g_free(result ? result->manual : NULL);
    if (result)
        g_weak_ref_clear(&result->dialog_ref);
    g_free(result);
    if (dialog)
        g_object_unref(dialog);
    return G_SOURCE_REMOVE;
}

static void rot_pref_scan_devices_async(RotPrefUi *ui,
                                        const gchar *current_id,
                                        gboolean force_autopick)
{
    RotDeviceScanResult *result = NULL;
    const gchar *manual_text = NULL;

    if (ui == NULL)
        return;

    if (ui->device_scan_in_progress)
        return;

    ui->device_scan_in_progress = TRUE;
    if (ui->device_refresh)
        gtk_widget_set_sensitive(ui->device_refresh, FALSE);

    result = g_new0(RotDeviceScanResult, 1);
    g_weak_ref_init(&result->dialog_ref, ui->dialog);
    result->current_id = current_id ? g_strdup(current_id) : NULL;
    if (force_autopick)
        result->autopick = TRUE;
    else
        result->autopick = ui->device_autopick ?
            gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ui->device_autopick)) : TRUE;
    if (ui->device_manual)
        manual_text = gtk_entry_get_text(GTK_ENTRY(ui->device_manual));
    result->manual = (manual_text && *manual_text) ? g_strdup(manual_text) : NULL;

    GThread *thread = g_thread_new("rot-device-scan",
                                   rot_pref_scan_devices_thread,
                                   result);
    g_thread_unref(thread);
}

static gchar *rot_pref_resolve_device(RotPrefUi *ui,
                                      const gchar *current,
                                      gboolean autopick,
                                      gchar **detail)
{
    GSList *list = NULL;
    gchar *picked = NULL;
    const gchar *use_current = current;

    if (detail)
        *detail = NULL;

    if (rot_pref_is_other_id(use_current))
        use_current = NULL;

    list = ui ? ui->device_cache : NULL;
    if (list == NULL)
        list = gp_serial_list_candidates();

    if (autopick)
        picked = rot_pref_pick_best_device(list, use_current);
    else if (use_current && rot_pref_list_contains(list, use_current))
        picked = g_strdup(use_current);

    if (picked == NULL && detail)
    {
        *detail = g_strdup(autopick
                           ? "No USB serial devices found"
                           : "No serial device selected");
    }

    if (ui == NULL || list != ui->device_cache)
        gp_serial_free_candidates(list);

    return picked;
}

static gchar *rot_pref_resolve_device_from_ui(RotPrefUi *ui,
                                              gboolean autopick,
                                              gchar **detail)
{
    const gchar *active_id = NULL;
    const gchar *manual_text = NULL;
    const gchar *current = NULL;

    if (detail)
        *detail = NULL;

    if (ui == NULL)
        return NULL;

    if (ui->device_combo)
        active_id = gtk_combo_box_get_active_id(GTK_COMBO_BOX(ui->device_combo));
    if (ui->device_manual)
        manual_text = gtk_entry_get_text(GTK_ENTRY(ui->device_manual));

    current = active_id;
    if (autopick)
        return rot_pref_resolve_device(ui, current, TRUE, detail);

    if (rot_pref_is_other_id(active_id) || active_id == NULL || *active_id == '\0')
    {
        if (manual_text && *manual_text)
            return g_strdup(manual_text);
        if (detail)
            *detail = g_strdup("No serial device selected");
        return NULL;
    }

    return g_strdup(active_id);
}
static void rotctld_test_connection_cb(GtkButton *button, gpointer data)
{
    RotPrefUi *ui = data;
    const gchar *host_text;
    const gchar *spawn_host;
    gint port_val;
    gchar *device = NULL;
    gchar *device_note = NULL;
    gchar *stderr_tail = NULL;
    gboolean ok = FALSE;
    gboolean spawned = FALSE;
    RotctldMgr *mgr = NULL;

    (void)button;

    if (ui == NULL || ui->host == NULL || ui->port == NULL)
        return;

    host_text = gtk_entry_get_text(GTK_ENTRY(ui->host));
    spawn_host = host_text;
    port_val = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(ui->port));

    if (host_text == NULL || *host_text == '\0')
    {
        rot_pref_show_dialog(ui, GTK_MESSAGE_ERROR,
                             _("rotctld connection failed"),
                             _("Missing host."));
        return;
    }

    if (port_val <= 0 || port_val > 65535)
    {
        rot_pref_show_dialog(ui, GTK_MESSAGE_ERROR,
                             _("rotctld connection failed"),
                             _("Invalid port."));
        return;
    }

    if (rotctld_mgr_host_is_local(host_text))
        spawn_host = "127.0.0.1";

    if (rotctld_mgr_host_is_local(host_text) &&
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ui->autostart)))
    {
        gboolean autopick = ui->device_autopick &&
            gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ui->device_autopick));

        device = rot_pref_resolve_device_from_ui(ui, autopick, &device_note);
        if (device == NULL || *device == '\0')
        {
            const gchar *msg = device_note ? device_note
                                           : _("No serial device selected.");
            rot_pref_show_dialog(ui, GTK_MESSAGE_ERROR,
                                 _("rotctld connection failed"),
                                 msg);
            g_free(device);
            g_free(device_note);
            return;
        }

        rot_protocol_t proto =
            rot_protocol_from_combo(GTK_COMBO_BOX(ui->protocol));
        if (!rot_protocol_is_valid(proto))
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        "rotctld test: invalid rotator protocol %d",
                        proto);
            rot_pref_show_dialog(ui, GTK_MESSAGE_ERROR,
                                 _("rotctld connection failed"),
                                 _("Invalid rotator protocol."));
            g_free(device);
            g_free(device_note);
            return;
        }

        gint model = rot_protocol_to_hamlib_model(proto);
        gint baud_val = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(ui->baud));
        gchar *error = NULL;

        if (proto == ROT_PROTOCOL_OTHER && ui->hamlib_model)
            model = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(ui->hamlib_model));
        if (model <= 0)
            model = rot_protocol_to_hamlib_model(ROT_PROTOCOL_GS232B);

        if (baud_val <= 0)
            baud_val = rot_protocol_default_baud(proto);

        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    "rotctld spawn: protocol=%s model=%d",
                    rot_protocol_name(proto),
                    model);
        mgr = rotctld_mgr_spawn(spawn_host, port_val, model, device, baud_val,
                                TRUE, &error);
        if (mgr == NULL)
        {
            rot_pref_show_dialog(ui, GTK_MESSAGE_ERROR,
                                 _("rotctld connection failed"),
                                 _("See log for details."));
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        "rotctld test spawn failed host=%s port=%d error=%s",
                        spawn_host, port_val,
                        error ? error : "unknown");
            g_free(error);
            g_free(device);
            g_free(device_note);
            return;
        }

        spawned = TRUE;
        rotctld_mgr_set_log_callback(mgr, NULL, NULL);
    }

    ok = rotctld_mgr_wait_for_port(spawn_host, port_val, 5000);
    if (!ok)
        stderr_tail = mgr ? rotctld_mgr_get_log_tail(mgr) : NULL;

    if (ok)
    {
        rot_pref_show_dialog(ui, GTK_MESSAGE_INFO,
                             _("rotctld connection OK"),
                             _("See log for details."));
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    "rotctld test ok host=%s port=%d", spawn_host, port_val);
    }
    else
    {
        rot_pref_show_dialog(ui, GTK_MESSAGE_ERROR,
                             _("rotctld connection failed"),
                             _("See log for details."));
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    "rotctld test failed host=%s port=%d",
                    spawn_host, port_val);
        if (stderr_tail && *stderr_tail)
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        "rotctld stderr tail:\n%s", stderr_tail);
    }

    if (spawned)
        rotctld_mgr_terminate(&mgr);

    g_free(stderr_tail);
    g_free(device);
    g_free(device_note);
}

/* Update widgets from the currently selected row in the treeview */
static void update_widgets(RotPrefUi *ui, rotor_conf_t * conf)
{
    if (ui == NULL || conf == NULL)
        return;

    rot_pref_ui_begin_update(ui, "update_widgets");

    /* configuration name */
    gtk_entry_set_text(GTK_ENTRY(ui->name), conf->name);

    /* host */
    if (conf->host)
        gtk_entry_set_text(GTK_ENTRY(ui->host), conf->host);

    /* port */
    if (conf->port > 0 && conf->port <= 65535)
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->port), conf->port);
    else
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->port), 4533); /* hamlib default? */

    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ui->autostart),
                                 conf->autostart);
    rot_pref_combo_set_active_id(GTK_COMBO_BOX(ui->protocol),
                                 rot_protocol_id(conf->protocol),
                                 G_CALLBACK(protocol_changed_cb));
    ui->hamlib_model_custom = conf->hamlib_model;
    rot_pref_update_hamlib_model_ui(ui, conf->protocol, conf->hamlib_model);
    if (conf->baud > 0)
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->baud), conf->baud);
    else
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->baud),
                                  rot_protocol_default_baud(conf->protocol));

    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ui->device_autopick),
                                 conf->device_autopick);
    gtk_entry_set_text(GTK_ENTRY(ui->device_manual),
                       conf->device_manual ? conf->device_manual : "");
    rot_pref_scan_devices_async(ui, conf->device, FALSE);
    rot_pref_update_device_ui_state(ui);

    rot_pref_combo_set_active(GTK_COMBO_BOX(ui->aztype),
                              (conf->aztype == ROT_AZ_TYPE_180)
                              ? ROT_AZ_TYPE_180
                              : ROT_AZ_TYPE_360,
                              G_CALLBACK(aztype_changed_cb));

    /* az and el limits */
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->minaz), conf->minaz);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->maxaz), conf->maxaz);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->minel), conf->minel);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->maxel), conf->maxel);
    rot_pref_combo_set_active(GTK_COMBO_BOX(ui->axismode), conf->axis_mode,
                              G_CALLBACK(axismode_changed_cb));
    update_el_limits_sensitivity(ui);
    if (ui->disable_pos_feedback)
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ui->disable_pos_feedback),
                                     conf->disable_pos_feedback_checks);

    rot_pref_ui_end_update(ui, "update_widgets");
    rot_pref_update_ok_button(ui);
}

/* called when the user clicks on the CLEAR button */
static void clear_widgets(RotPrefUi *ui)
{
    if (ui == NULL)
        return;

    rot_pref_ui_begin_update(ui, "clear_widgets");

    gtk_entry_set_text(GTK_ENTRY(ui->name), "");
    gtk_entry_set_text(GTK_ENTRY(ui->host), "127.0.0.1");
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->port), 4533);     /* hamlib default? */
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ui->autostart), TRUE);
    rot_pref_combo_set_active_id(GTK_COMBO_BOX(ui->protocol),
                                 rot_protocol_id(ROT_PROTOCOL_GS232B),
                                 G_CALLBACK(protocol_changed_cb));
    ui->hamlib_model_custom = rot_protocol_to_hamlib_model(ROT_PROTOCOL_GS232B);
    rot_pref_update_hamlib_model_ui(ui, ROT_PROTOCOL_GS232B,
                                    ui->hamlib_model_custom);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->baud),
                              rot_protocol_default_baud(ROT_PROTOCOL_GS232B));
    gtk_entry_set_text(GTK_ENTRY(ui->device_manual), "");
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ui->device_autopick), TRUE);
    rot_pref_scan_devices_async(ui, NULL, FALSE);
    rot_pref_update_device_ui_state(ui);
    rot_pref_combo_set_active(GTK_COMBO_BOX(ui->aztype), ROT_AZ_TYPE_360,
                              G_CALLBACK(aztype_changed_cb));
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->minaz), 0);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->maxaz), 360);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->minel), -5);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->maxel), 185);
    rot_pref_combo_set_active(GTK_COMBO_BOX(ui->axismode), ROT_AXIS_MODE_AZ_EL,
                              G_CALLBACK(axismode_changed_cb));
    update_el_limits_sensitivity(ui);
    if (ui->disable_pos_feedback)
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ui->disable_pos_feedback),
                                     FALSE);

    rot_pref_ui_end_update(ui, "clear_widgets");
    rot_pref_update_ok_button(ui);
}

/*
 * This function is called when the contents of the name entry changes.
 * The primary purpose of this function is to check whether the char length
 * of the name is greater than zero, if yes enable the OK button of the dialog.
 */
static void name_changed(GtkWidget * widget, gpointer data)
{
    RotPrefUi     *ui = data;
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
    rot_pref_update_ok_button(ui);
}

static void protocol_changed_cb(GtkComboBox * box, gpointer data)
{
    RotPrefUi *ui = data;
    rot_protocol_t proto = rot_protocol_from_combo(box);

    if (ui != NULL && ui->ui_updating)
        return;

    if (ui != NULL && ui->baud)
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->baud),
                                  rot_protocol_default_baud(proto));

    if (ui != NULL && ui->hamlib_model)
        rot_pref_update_hamlib_model_ui(ui, proto, ui->hamlib_model_custom);
}

static void device_refresh_cb(GtkButton *button, gpointer data)
{
    RotPrefUi *ui = data;
    const gchar *current_id = NULL;

    (void)button;

    if (ui != NULL && ui->ui_updating)
        return;

    if (ui != NULL && ui->device_combo)
        current_id = gtk_combo_box_get_active_id(
            GTK_COMBO_BOX(ui->device_combo));

    rot_pref_scan_devices_async(ui, current_id, TRUE);
}

static void device_autopick_toggled_cb(GtkToggleButton *button, gpointer data)
{
    RotPrefUi *ui = data;

    if (ui != NULL && ui->ui_updating)
        return;

    (void)button;
    rot_pref_schedule_device_combo_refresh(ui);
}

static void device_combo_changed_cb(GtkComboBox *box, gpointer data)
{
    RotPrefUi *ui = data;
    (void)box;

    if (ui != NULL && ui->ui_updating)
        return;

    rot_pref_update_device_ui_state(ui);
}

static void update_el_limits_sensitivity(RotPrefUi *ui)
{
    gboolean az_el = TRUE;

    if (ui == NULL)
        return;

    if (ui->axismode)
        az_el = (gtk_combo_box_get_active(GTK_COMBO_BOX(ui->axismode)) ==
                 ROT_AXIS_MODE_AZ_EL);

    if (ui->minel)
        gtk_widget_set_sensitive(ui->minel, az_el);
    if (ui->maxel)
        gtk_widget_set_sensitive(ui->maxel, az_el);
    if (ui->minel_label)
        gtk_widget_set_sensitive(ui->minel_label, az_el);
    if (ui->maxel_label)
        gtk_widget_set_sensitive(ui->maxel_label, az_el);
}

static void axismode_changed_cb(GtkComboBox *box, gpointer data)
{
    RotPrefUi *ui = data;
    (void)box;

    if (ui != NULL && ui->ui_updating)
        return;

    update_el_limits_sensitivity(ui);
}

static void aztype_changed_cb(GtkComboBox * box, gpointer data)
{
    RotPrefUi *ui = data;
    gint            type = gtk_combo_box_get_active(box);

    if (ui != NULL && ui->ui_updating)
        return;

    rot_pref_ui_begin_update(ui, "aztype_changed");
    switch (type)
    {
    case ROT_AZ_TYPE_360:
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->minaz), 0.0);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->maxaz), 360.0);
        break;

    case ROT_AZ_TYPE_180:
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->minaz), -180.0);
        gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->maxaz), +180.0);
        break;

    default:
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%s: Invalid AZ rotator type."), __FILE__, __func__);
        break;
    }
    rot_pref_ui_end_update(ui, "aztype_changed");
}

static GtkWidget *create_editor_widgets(RotPrefUi *ui, rotor_conf_t * conf)
{
    GtkWidget      *table;
    GtkWidget      *label;
    GtkWidget      *test_button;
    GtkWidget      *device_manual_row;

    table = gtk_grid_new();
    gtk_container_set_border_width(GTK_CONTAINER(table), 5);
    gtk_grid_set_column_spacing(GTK_GRID(table), 5);
    gtk_grid_set_row_spacing(GTK_GRID(table), 5);

    /* Config name */
    label = gtk_label_new(_("Name"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 0, 1, 1);

    ui->name = gtk_entry_new();
    gtk_entry_set_max_length(GTK_ENTRY(ui->name), 25);
    gtk_widget_set_tooltip_text(ui->name,
                                _("Enter a short name for this configuration, "
                                  " e.g. ROTOR-1.\n"
                                  "Allowed characters: 0..9, a..z, A..Z, - and _"));
    gtk_grid_attach(GTK_GRID(table), ui->name, 1, 0, 3, 1);

    /* attach changed signal so that we can enable OK button when
       a proper name has been entered
     */
    g_signal_connect(ui->name, "changed", G_CALLBACK(name_changed), ui);
    g_signal_connect(ui->name, "changed", G_CALLBACK(rot_pref_on_field_changed), ui);

    /* Host */
    label = gtk_label_new(_("Host"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 1, 1, 1);

    ui->host = gtk_entry_new();
    gtk_entry_set_max_length(GTK_ENTRY(ui->host), 50);
    gtk_entry_set_text(GTK_ENTRY(ui->host), "127.0.0.1");
    g_signal_connect(ui->host, "changed", G_CALLBACK(rot_pref_on_field_changed), ui);
    gtk_widget_set_tooltip_text(ui->host,
                                _("Enter the host where rotctld is running. "
                                  "You can use both host name and IP address, "
                                  "e.g. 192.168.1.100\n\n"
                                  "If gpredict and rotctld are running on the "
                                  "same computer, use 127.0.0.1"));
    gtk_grid_attach(GTK_GRID(table), ui->host, 1, 1, 3, 1); 

    /* port */
    label = gtk_label_new(_("Port"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 2, 1, 1);

    ui->port = gtk_spin_button_new_with_range(1024, 65535, 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->port), 4533);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(ui->port), 0);
    g_signal_connect(ui->port, "value-changed", G_CALLBACK(rot_pref_on_field_changed), ui);
    gtk_widget_set_tooltip_text(ui->port,
                                _("Enter the port number where rotctld is "
                                  "listening. Default is 4533."));
    gtk_grid_attach(GTK_GRID(table), ui->port, 1, 2, 1, 1);

    test_button = gtk_button_new_with_label(_("Test connection"));
    gtk_widget_set_tooltip_text(test_button,
                                _("Start rotctld if needed and query its status."));
    gtk_grid_attach(GTK_GRID(table), test_button, 2, 2, 2, 1);
    g_signal_connect(test_button, "clicked",
                     G_CALLBACK(rotctld_test_connection_cb), ui);

    ui->autostart = gtk_check_button_new_with_label(_("Auto-start local rotctld"));
    gtk_widget_set_tooltip_text(ui->autostart,
                                _("Start rotctld automatically when connecting to a local host."));
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ui->autostart), TRUE);
    gtk_grid_attach(GTK_GRID(table), ui->autostart, 1, 3, 3, 1);
    g_signal_connect(ui->autostart, "toggled",
                     G_CALLBACK(rot_pref_on_field_changed), ui);

    /* Protocol */
    label = gtk_label_new(_("Protocol"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 4, 1, 1);

    ui->protocol = gtk_combo_box_text_new();
    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(ui->protocol),
                              "gs232b", _("Yaesu GS-232B"));
    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(ui->protocol),
                              "rot1prog", _("SPID Rot1Prog"));
    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(ui->protocol),
                              "rot2prog", _("SPID Rot2Prog"));
    gtk_combo_box_text_append(GTK_COMBO_BOX_TEXT(ui->protocol),
                              "other", _("Other"));
    gtk_combo_box_set_active_id(GTK_COMBO_BOX(ui->protocol),
                                rot_protocol_id(ROT_PROTOCOL_GS232B));
    gtk_grid_attach(GTK_GRID(table), ui->protocol, 1, 4, 2, 1);
    g_message("rot-editor: protocol selector added");
    g_signal_connect(G_OBJECT(ui->protocol), "changed",
                     G_CALLBACK(protocol_changed_cb), ui);
    g_signal_connect(G_OBJECT(ui->protocol), "changed",
                     G_CALLBACK(rot_pref_on_field_changed), ui);
    gp_ui_quarantine_register_combo(ui->dialog, GTK_COMBO_BOX(ui->protocol));

    /* Hamlib model */
    ui->hamlib_model_label = gtk_label_new(_("Hamlib model"));
    g_object_set(ui->hamlib_model_label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), ui->hamlib_model_label, 0, 5, 1, 1);

    ui->hamlib_model = gtk_spin_button_new_with_range(1, 99999, 1);
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(ui->hamlib_model), 0);
    gtk_spin_button_set_numeric(GTK_SPIN_BUTTON(ui->hamlib_model), TRUE);
    gtk_spin_button_set_wrap(GTK_SPIN_BUTTON(ui->hamlib_model), FALSE);
    gtk_widget_set_tooltip_text(ui->hamlib_model,
                                _("Hamlib rotator model ID (numeric)."));
    gtk_grid_attach(GTK_GRID(table), ui->hamlib_model, 1, 5, 1, 1);
    g_signal_connect(ui->hamlib_model, "value-changed",
                     G_CALLBACK(hamlib_model_changed_cb), ui);

    ui->hamlib_model_custom = rot_protocol_to_hamlib_model(ROT_PROTOCOL_GS232B);
    rot_pref_update_hamlib_model_ui(ui, ROT_PROTOCOL_GS232B,
                                    ui->hamlib_model_custom);

    /* Baud */
    label = gtk_label_new(_("Baud"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 7, 1, 1);

    ui->baud = gtk_spin_button_new_with_range(300, 921600, 100);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->baud),
                              rot_protocol_default_baud(ROT_PROTOCOL_GS232B));
    gtk_spin_button_set_digits(GTK_SPIN_BUTTON(ui->baud), 0);
    gtk_widget_set_tooltip_text(ui->baud, _("Serial baud rate for rotctld."));
    gtk_grid_attach(GTK_GRID(table), ui->baud, 1, 6, 1, 1);
    g_signal_connect(ui->baud, "value-changed",
                     G_CALLBACK(rot_pref_on_field_changed), ui);

    /* Device */
    label = gtk_label_new(_("Device"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 6, 1, 1);

    ui->device_combo = gtk_combo_box_text_new();
    gtk_widget_set_tooltip_text(ui->device_combo,
                                _("Select the serial device for your rotor."));
    gtk_grid_attach(GTK_GRID(table), ui->device_combo, 1, 7, 2, 1);
    g_signal_connect(ui->device_combo, "changed",
                     G_CALLBACK(device_combo_changed_cb), ui);
    g_signal_connect(ui->device_combo, "changed",
                     G_CALLBACK(rot_pref_on_field_changed), ui);
    gp_ui_quarantine_register_combo(ui->dialog, GTK_COMBO_BOX(ui->device_combo));

    ui->device_refresh = gtk_button_new_with_label(_("Find port"));
    gtk_widget_set_tooltip_text(ui->device_refresh,
                                _("Scan for serial devices and pick the best match."));
    gtk_grid_attach(GTK_GRID(table), ui->device_refresh, 3, 7, 1, 1);
    g_signal_connect(ui->device_refresh, "clicked",
                     G_CALLBACK(device_refresh_cb), ui);

    /* Custom device path */
    device_manual_row = gtk_grid_new();
    gtk_grid_set_column_spacing(GTK_GRID(device_manual_row), 6);

    label = gtk_label_new(_("Custom device path"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(device_manual_row), label, 0, 0, 1, 1);

    ui->device_manual = gtk_entry_new();
    gtk_widget_set_tooltip_text(ui->device_manual,
                                _("Used only when Device=Other..."));
    gtk_grid_attach(GTK_GRID(device_manual_row), ui->device_manual, 1, 0, 3, 1);
    g_signal_connect(ui->device_manual, "changed",
                     G_CALLBACK(rot_pref_on_field_changed), ui);

    ui->device_manual_revealer = gtk_revealer_new();
    gtk_revealer_set_transition_type(GTK_REVEALER(ui->device_manual_revealer),
                                     GTK_REVEALER_TRANSITION_TYPE_SLIDE_DOWN);
    gtk_container_add(GTK_CONTAINER(ui->device_manual_revealer), device_manual_row);
    gtk_grid_attach(GTK_GRID(table), ui->device_manual_revealer, 0, 8, 4, 1);

    ui->device_autopick = gtk_check_button_new_with_label(_("Auto-detect port when empty"));
    gtk_widget_set_tooltip_text(ui->device_autopick,
                                _("Leave the device field empty and detect a serial port when connecting."));
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ui->device_autopick), TRUE);
    gtk_grid_attach(GTK_GRID(table), ui->device_autopick, 1, 9, 2, 1);
    g_signal_connect(ui->device_autopick, "toggled",
                     G_CALLBACK(device_autopick_toggled_cb), ui);
    g_signal_connect(ui->device_autopick, "toggled",
                     G_CALLBACK(rot_pref_on_field_changed), ui);

    ui->device_status = gtk_label_new("");
    g_object_set(ui->device_status, "xalign", 0.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), ui->device_status, 1, 10, 3, 1);

    gtk_grid_attach(GTK_GRID(table),
                    gtk_separator_new(GTK_ORIENTATION_HORIZONTAL),
                    0, 11, 4, 1);

    /* Tracking geometry */
    label = gtk_label_new(_("Tracking geometry"));
    g_object_set(label, "xalign", 0.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 12, 4, 1);

    /* Axis mode */
    label = gtk_label_new(_("Axis mode"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 13, 1, 1);

    ui->axismode = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->axismode),
                                   _("Azimuth + Elevation"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->axismode),
                                   _("Azimuth only"));
    gtk_combo_box_set_active(GTK_COMBO_BOX(ui->axismode), ROT_AXIS_MODE_AZ_EL);
    gtk_widget_set_tooltip_text(ui->axismode,
                                _("Select whether this rotor supports both azimuth and elevation."));
    gtk_grid_attach(GTK_GRID(table), ui->axismode, 1, 13, 2, 1);
    g_signal_connect(G_OBJECT(ui->axismode), "changed",
                     G_CALLBACK(axismode_changed_cb), ui);
    g_signal_connect(G_OBJECT(ui->axismode), "changed",
                     G_CALLBACK(rot_pref_on_field_changed), ui);
    gp_ui_quarantine_register_combo(ui->dialog, GTK_COMBO_BOX(ui->axismode));

    label = gtk_label_new(_("Wrap type"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 14, 1, 1);

    ui->aztype = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->aztype),
                                   _("Continuous"));
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(ui->aztype),
                                   _("North centered"));
    gtk_combo_box_set_active(GTK_COMBO_BOX(ui->aztype), ROT_AZ_TYPE_360);
    gtk_widget_set_tooltip_text(ui->aztype,
                                _("Select the azimuth wrap convention. "
                                  "0\302\260 is at North, clockwise is positive."));
    gtk_grid_attach(GTK_GRID(table), ui->aztype, 1, 14, 2, 1);
    g_signal_connect(G_OBJECT(ui->aztype), "changed",
                     G_CALLBACK(aztype_changed_cb), ui);
    g_signal_connect(G_OBJECT(ui->aztype), "changed",
                     G_CALLBACK(rot_pref_on_field_changed), ui);
    gp_ui_quarantine_register_combo(ui->dialog, GTK_COMBO_BOX(ui->aztype));

    /* Az and El limits */
    label = gtk_label_new(_(" Min Az"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 0, 15, 1, 1);
    ui->minaz = gtk_spin_button_new_with_range(-200, 100, 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->minaz), 0);
    gtk_spin_button_set_numeric(GTK_SPIN_BUTTON(ui->minaz), TRUE);
    gtk_spin_button_set_wrap(GTK_SPIN_BUTTON(ui->minaz), FALSE);
    gtk_grid_attach(GTK_GRID(table), ui->minaz, 1, 15, 1, 1);
    g_signal_connect(ui->minaz, "value-changed",
                     G_CALLBACK(rot_pref_on_field_changed), ui);

    label = gtk_label_new(_(" Max Az"));
    g_object_set(label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), label, 2, 15, 1, 1);
    ui->maxaz = gtk_spin_button_new_with_range(0, 480, 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->maxaz), 360);
    gtk_spin_button_set_numeric(GTK_SPIN_BUTTON(ui->maxaz), TRUE);
    gtk_spin_button_set_wrap(GTK_SPIN_BUTTON(ui->maxaz), FALSE);
    gtk_grid_attach(GTK_GRID(table), ui->maxaz, 3, 15, 1, 1);
    g_signal_connect(ui->maxaz, "value-changed",
                     G_CALLBACK(rot_pref_on_field_changed), ui);

    ui->minel_label = gtk_label_new(_(" Min El"));
    g_object_set(ui->minel_label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), ui->minel_label, 0, 16, 1, 1);
    ui->minel = gtk_spin_button_new_with_range(-5, 185, 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->minel), -5);
    gtk_spin_button_set_numeric(GTK_SPIN_BUTTON(ui->minel), TRUE);
    gtk_spin_button_set_wrap(GTK_SPIN_BUTTON(ui->minel), FALSE);
    gtk_grid_attach(GTK_GRID(table), ui->minel, 1, 16, 1, 1);
    g_signal_connect(ui->minel, "value-changed",
                     G_CALLBACK(rot_pref_on_field_changed), ui);

    ui->maxel_label = gtk_label_new(_(" Max El"));
    g_object_set(ui->maxel_label, "xalign", 1.0, "yalign", 0.5, NULL);
    gtk_grid_attach(GTK_GRID(table), ui->maxel_label, 2, 16, 1, 1);
    ui->maxel = gtk_spin_button_new_with_range(-5, 185, 1);
    gtk_spin_button_set_value(GTK_SPIN_BUTTON(ui->maxel), 185);
    gtk_spin_button_set_numeric(GTK_SPIN_BUTTON(ui->maxel), TRUE);
    gtk_spin_button_set_wrap(GTK_SPIN_BUTTON(ui->maxel), FALSE);
    gtk_grid_attach(GTK_GRID(table), ui->maxel, 3, 16, 1, 1);
    g_signal_connect(ui->maxel, "value-changed",
                     G_CALLBACK(rot_pref_on_field_changed), ui);

    gtk_grid_attach(GTK_GRID(table),
                    gtk_separator_new(GTK_ORIENTATION_HORIZONTAL),
                    0, 17, 4, 1);

    ui->disable_pos_feedback =
        gtk_check_button_new_with_label(_("Disable position feedback checks (no encoder)"));
    gtk_widget_set_tooltip_text(ui->disable_pos_feedback,
                                _("Allow sending commands even when no position feedback is available. "
                                  "Disables position discrepancy disconnect logic."));
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(ui->disable_pos_feedback), FALSE);
    gtk_grid_attach(GTK_GRID(table), ui->disable_pos_feedback, 0, 18, 4, 1);
    g_signal_connect(ui->disable_pos_feedback, "toggled",
                     G_CALLBACK(rot_pref_on_field_changed), ui);


    if (conf->name != NULL)
        update_widgets(ui, conf);
    else
    {
        rot_pref_scan_devices_async(ui, NULL, FALSE);
        rot_pref_update_device_ui_state(ui);
    }

    update_el_limits_sensitivity(ui);
    rot_pref_update_ok_button(ui);

    gtk_widget_show_all(table);

    return table;
}

/* Called when the user clicks the OK button */
static gboolean apply_changes(RotPrefUi *ui, rotor_conf_t * conf)
{
    if (ui == NULL || conf == NULL)
        return FALSE;

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

    conf->autostart =
        gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ui->autostart));
    conf->protocol = rot_protocol_from_combo(GTK_COMBO_BOX(ui->protocol));
    if (conf->protocol == ROT_PROTOCOL_OTHER && ui->hamlib_model)
        conf->hamlib_model =
            gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(ui->hamlib_model));
    else
        conf->hamlib_model = rot_protocol_to_hamlib_model(conf->protocol);
    if (conf->hamlib_model <= 0)
        conf->hamlib_model = rot_protocol_to_hamlib_model(ROT_PROTOCOL_GS232B);
    conf->baud = gtk_spin_button_get_value_as_int(GTK_SPIN_BUTTON(ui->baud));
    if (conf->baud <= 0)
        conf->baud = rot_protocol_default_baud(conf->protocol);

    {
        gboolean autopick = gtk_toggle_button_get_active(
            GTK_TOGGLE_BUTTON(ui->device_autopick));
        const gchar *active_id = NULL;
        const gchar *manual_text = NULL;

        if (ui->device_combo)
            active_id = gtk_combo_box_get_active_id(GTK_COMBO_BOX(ui->device_combo));
        if (ui->device_manual)
            manual_text = gtk_entry_get_text(GTK_ENTRY(ui->device_manual));

        if (conf->device)
            g_free(conf->device);
        conf->device = NULL;

        if (conf->device_manual)
            g_free(conf->device_manual);
        conf->device_manual = NULL;

        if (autopick)
        {
            conf->device = NULL;
            conf->device_manual = NULL;
        }
        else if (rot_pref_is_other_id(active_id) ||
                 active_id == NULL || *active_id == '\0')
        {
            if (manual_text && *manual_text)
            {
                conf->device = g_strdup(manual_text);
                conf->device_manual = g_strdup(manual_text);
            }
        }
        else
        {
            conf->device = g_strdup(active_id);
        }

        conf->device_autopick = autopick;
    }

    /* az type */
    conf->aztype = gtk_combo_box_get_active(GTK_COMBO_BOX(ui->aztype));

    /* az and el ranges */
    conf->minaz = gtk_spin_button_get_value(GTK_SPIN_BUTTON(ui->minaz));
    conf->maxaz = gtk_spin_button_get_value(GTK_SPIN_BUTTON(ui->maxaz));
    conf->minel = gtk_spin_button_get_value(GTK_SPIN_BUTTON(ui->minel));
    conf->maxel = gtk_spin_button_get_value(GTK_SPIN_BUTTON(ui->maxel));

    /* az stop position */

    /* axis mode */
    conf->axis_mode = gtk_combo_box_get_active(GTK_COMBO_BOX(ui->axismode));

    /* position feedback checks */
    if (ui->disable_pos_feedback)
        conf->disable_pos_feedback_checks =
            gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(ui->disable_pos_feedback));

    /* axis inversion */

    /* offsets */

    return TRUE;
}

static void rot_pref_dialog_response(GtkDialog *dialog,
                                     gint response,
                                     gpointer user_data)
{
    RotPrefDialogState *state = user_data;

    if (state == NULL)
        return;

    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                "rot-pref editor response=%d", response);
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

static void rot_pref_dialog_destroy(GtkWidget *widget, gpointer user_data)
{
    RotPrefDialogState *state = user_data;
    RotPrefUi *ui;

    (void)widget;

    if (state == NULL)
        return;

    ui = state->ui;
    if (!state->finished && state->done)
        state->done(state->conf, FALSE, state->user_data);
    sat_log_log(SAT_LOG_LEVEL_DEBUG, "rot-pref editor destroyed");
    if (ui != NULL)
        ui->ui_updating = FALSE;

    if (ui != NULL && ui->pending_ui_refresh_id != 0)
    {
        g_source_remove(ui->pending_ui_refresh_id);
        ui->pending_ui_refresh_id = 0;
    }

    if (ui != NULL && ui->device_cache)
    {
        gp_serial_free_candidates(ui->device_cache);
        ui->device_cache = NULL;
    }
    if (ui != NULL)
        ui->device_scan_in_progress = FALSE;

    g_free(state);
}

/**
 * Add or edit a rotor configuration.
 *
 * @param conf Pointer to a rotator configuration.
 *
 * If conf->name is not NULL the widgets will be populated with the data.
 */
void sat_pref_rot_editor_run(rotor_conf_t *conf,
                             RotPrefEditorDoneFunc done,
                             gpointer user_data)
{
    RotPrefDialogState *state;
    RotPrefUi *ui;
    GtkWidget *dialog;

    /* create dialog and add contents */
    dialog = gtk_dialog_new_with_buttons(_("Edit rotator configuration"),
                                         GTK_WINDOW(window),
                                         GTK_DIALOG_DESTROY_WITH_PARENT,
                                         "_Clear", GTK_RESPONSE_REJECT,
                                         "_Cancel", GTK_RESPONSE_CANCEL,
                                         "_Ok", GTK_RESPONSE_OK,
                                         NULL);

    /* disable OK button to begin with */
    gtk_dialog_set_response_sensitive(GTK_DIALOG(dialog),
                                      GTK_RESPONSE_OK, FALSE);

    ui = g_new0(RotPrefUi, 1);
    ui->dialog = dialog;
    g_object_set_data_full(G_OBJECT(dialog), "rot_pref_ui", ui, g_free);
    gp_ui_quarantine_install(dialog);
    gtk_container_add(GTK_CONTAINER
                      (gtk_dialog_get_content_area(GTK_DIALOG(dialog))),
                      create_editor_widgets(ui, conf));

    state = g_new0(RotPrefDialogState, 1);
    state->conf = conf;
    state->done = done;
    state->user_data = user_data;
    state->ui = ui;

    sat_log_log(SAT_LOG_LEVEL_DEBUG, "rot-pref editor created");
    g_signal_connect(dialog, "response",
                     G_CALLBACK(rot_pref_dialog_response), state);
    g_signal_connect(dialog, "destroy",
                     G_CALLBACK(rot_pref_dialog_destroy), state);

    gtk_widget_show_all(dialog);
    sat_log_log(SAT_LOG_LEVEL_DEBUG, "rot-pref editor shown");

}
