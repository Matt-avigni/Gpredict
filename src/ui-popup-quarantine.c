/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include "ui-popup-quarantine.h"

#include "sat-log.h"

#define GP_UI_QUARANTINE_STATE_KEY "gpredict-ui-quarantine-state"
#define GP_UI_QUARANTINE_COMBO_KEY "gpredict-ui-quarantine-combo"
#define GP_UI_QUARANTINE_POPUP_KEY "gpredict-ui-quarantine-popup"

typedef struct {
    GtkWidget *toplevel;
    GdkWindow *window;
    gint64 last_popup_popdown_us;
    gint64 quarantine_until_us;
    gboolean swallowed_press;
    gboolean swallowed_release;
    GtkComboBox *pending_combo;
    gboolean pending_press_swallowed;
    gboolean filter_installed;
    gulong realize_id;
    gulong unrealize_id;
    gulong destroy_id;
} GpUiQuarantine;

static gboolean gp_ui_quarantine_debug_enabled(void)
{
    static gint enabled = -1;

    if (enabled < 0)
    {
        const gchar *env = g_getenv("GP_UI_DEBUG");
        enabled = (env != NULL && *env != '\0') ? 1 : 0;
    }

    return enabled != 0;
}

#define GP_UI_LOG(...)                                              \
    do                                                              \
    {                                                               \
        if (gp_ui_quarantine_debug_enabled())                       \
            sat_log_log(SAT_LOG_LEVEL_DEBUG, __VA_ARGS__);          \
    } while (0)

static gboolean gp_ui_combo_get_popup_property(GtkComboBox *combo,
                                                gboolean *has_prop)
{
    gboolean shown = FALSE;

    if (has_prop != NULL)
        *has_prop = FALSE;

    if (combo == NULL)
        return FALSE;

    if (g_object_class_find_property(G_OBJECT_GET_CLASS(combo), "popup-shown"))
    {
        g_object_get(combo, "popup-shown", &shown, NULL);
        if (has_prop != NULL)
            *has_prop = TRUE;
    }

    return shown;
}

static gboolean gp_ui_combo_get_tracked_state(GtkComboBox *combo)
{
    if (combo == NULL)
        return FALSE;

    return GPOINTER_TO_INT(g_object_get_data(G_OBJECT(combo),
                                             GP_UI_QUARANTINE_POPUP_KEY));
}

static void gp_ui_combo_set_tracked_state(GtkComboBox *combo, gboolean shown)
{
    if (combo == NULL)
        return;

    g_object_set_data(G_OBJECT(combo), GP_UI_QUARANTINE_POPUP_KEY,
                      GINT_TO_POINTER(shown));
}

gboolean gp_ui_combo_popup_shown(GtkComboBox *combo)
{
    gboolean has_prop = FALSE;
    gboolean shown_prop = FALSE;
    gboolean shown_tracked = FALSE;

    if (combo == NULL)
        return FALSE;

    shown_prop = gp_ui_combo_get_popup_property(combo, &has_prop);
    shown_tracked = gp_ui_combo_get_tracked_state(combo);

    if (has_prop)
        return shown_prop || shown_tracked;

    return shown_tracked;
}

static void gp_ui_quarantine_reset_swallow(GpUiQuarantine *state)
{
    state->swallowed_press = FALSE;
    state->swallowed_release = FALSE;
}

static void gp_ui_quarantine_set_pending_combo(GpUiQuarantine *state,
                                               GtkComboBox *combo)
{
    if (state == NULL)
        return;

    if (state->pending_combo != NULL)
        g_object_remove_weak_pointer(G_OBJECT(state->pending_combo),
                                     (gpointer *)&state->pending_combo);

    state->pending_combo = combo;
    state->pending_press_swallowed = FALSE;

    if (state->pending_combo != NULL)
        g_object_add_weak_pointer(G_OBJECT(state->pending_combo),
                                  (gpointer *)&state->pending_combo);
}

static gboolean gp_ui_quarantine_event_on_widget(GpUiQuarantine *state,
                                                 GtkWidget *widget,
                                                 GdkEventButton *event)
{
    GtkAllocation alloc;
    gint wx = 0;
    gint wy = 0;
    gint toplevel_x = 0;
    gint toplevel_y = 0;
    gint64 ex = 0;
    gint64 ey = 0;

    if (state == NULL || widget == NULL || event == NULL)
        return FALSE;
    if (state->toplevel == NULL || state->window == NULL)
        return FALSE;
    if (!gtk_widget_get_mapped(widget))
        return FALSE;
    if (!gtk_widget_translate_coordinates(widget, state->toplevel, 0, 0, &wx, &wy))
        return FALSE;

    gtk_widget_get_allocation(widget, &alloc);
    gdk_window_get_origin(state->window, &toplevel_x, &toplevel_y);

    ex = (gint64)event->x_root;
    ey = (gint64)event->y_root;

    return ex >= (gint64)(toplevel_x + wx) &&
           ex < (gint64)(toplevel_x + wx + alloc.width) &&
           ey >= (gint64)(toplevel_y + wy) &&
           ey < (gint64)(toplevel_y + wy + alloc.height);
}

static gboolean gp_ui_quarantine_popup_idle(gpointer data)
{
    GtkComboBox *combo = GTK_COMBO_BOX(data);
    GtkWidget *widget = GTK_WIDGET(combo);

    if (combo == NULL)
        return G_SOURCE_REMOVE;

    if (gtk_widget_get_realized(widget) && gtk_widget_get_visible(widget))
        gtk_combo_box_popup(combo);

    return G_SOURCE_REMOVE;
}

static GdkFilterReturn gp_ui_quarantine_filter(GdkXEvent *xevent,
                                               GdkEvent *event,
                                               gpointer data)
{
    GpUiQuarantine *state = data;
    GdkEventButton *button_event;
    gint64 now_us;

    (void)xevent;

    if (state == NULL || event == NULL)
        return GDK_FILTER_CONTINUE;

    if (event->type != GDK_BUTTON_PRESS && event->type != GDK_BUTTON_RELEASE)
        return GDK_FILTER_CONTINUE;

    button_event = (GdkEventButton *)event;
    if (button_event->button != 1)
        return GDK_FILTER_CONTINUE;

    if (state->pending_combo != NULL &&
        !gp_ui_combo_popup_shown(state->pending_combo) &&
        gp_ui_quarantine_event_on_widget(state,
                                         GTK_WIDGET(state->pending_combo),
                                         button_event))
    {
        if (event->type == GDK_BUTTON_PRESS && !state->pending_press_swallowed)
        {
            state->pending_press_swallowed = TRUE;
            GP_UI_LOG("%s: swallow pending-combo press\n", __func__);
            return GDK_FILTER_REMOVE;
        }

        if (event->type == GDK_BUTTON_RELEASE && state->pending_press_swallowed)
        {
            GtkComboBox *combo = state->pending_combo;

            GP_UI_LOG("%s: swallow pending-combo release + reopen\n", __func__);
            gp_ui_quarantine_set_pending_combo(state, NULL);
            g_idle_add_full(G_PRIORITY_DEFAULT_IDLE,
                            gp_ui_quarantine_popup_idle,
                            g_object_ref(combo),
                            g_object_unref);
            return GDK_FILTER_REMOVE;
        }
    }

    now_us = g_get_monotonic_time();
    if (now_us >= state->quarantine_until_us)
        return GDK_FILTER_CONTINUE;

    if (event->type == GDK_BUTTON_PRESS && !state->swallowed_press)
    {
        state->swallowed_press = TRUE;
        GP_UI_LOG("%s: swallow press remaining=%lld us\n", __func__,
                  (long long)(state->quarantine_until_us - now_us));
        return GDK_FILTER_REMOVE;
    }

    if (event->type == GDK_BUTTON_RELEASE && !state->swallowed_release)
    {
        state->swallowed_release = TRUE;
        GP_UI_LOG("%s: swallow release remaining=%lld us\n", __func__,
                  (long long)(state->quarantine_until_us - now_us));
        return GDK_FILTER_REMOVE;
    }

    return GDK_FILTER_CONTINUE;
}

static void gp_ui_quarantine_update_combo_state(GpUiQuarantine *state,
                                                GtkComboBox *combo,
                                                gboolean shown)
{
    gboolean prev_shown;
    gint64 now_us;

    if (state == NULL || combo == NULL)
        return;

    prev_shown = gp_ui_combo_get_tracked_state(combo);
    if (prev_shown == shown)
        return;

    if (prev_shown && !shown)
    {
        now_us = g_get_monotonic_time();
        state->last_popup_popdown_us = now_us;
        state->quarantine_until_us = now_us + 250000;
        gp_ui_quarantine_reset_swallow(state);
        gp_ui_quarantine_set_pending_combo(state, combo);
        GP_UI_LOG("%s: popdown quarantine until=%lld us\n", __func__,
                  (long long)state->quarantine_until_us);
    }
    else if (shown && combo == state->pending_combo)
    {
        gp_ui_quarantine_set_pending_combo(state, NULL);
    }

    gp_ui_combo_set_tracked_state(combo, shown);
}

static void gp_ui_quarantine_combo_popup(GtkComboBox *combo, gpointer data)
{
    gp_ui_quarantine_update_combo_state(data, combo, TRUE);
}

static void gp_ui_quarantine_combo_popdown(GtkComboBox *combo, gpointer data)
{
    gp_ui_quarantine_update_combo_state(data, combo, FALSE);
}

static void gp_ui_quarantine_remove_filter(GpUiQuarantine *state)
{
    if (state == NULL)
        return;

    if (state->filter_installed && state->window != NULL)
    {
        gdk_window_remove_filter(state->window, gp_ui_quarantine_filter, state);
        state->filter_installed = FALSE;
    }
    state->window = NULL;
}

static void gp_ui_quarantine_realize(GtkWidget *widget, gpointer data)
{
    GpUiQuarantine *state = data;

    if (state == NULL || widget == NULL)
        return;

    state->window = gtk_widget_get_window(widget);
    if (state->window == NULL)
        return;

    gdk_window_add_filter(state->window, gp_ui_quarantine_filter, state);
    state->filter_installed = TRUE;

    GP_UI_LOG("%s: filter installed\n", __func__);
}

static void gp_ui_quarantine_unrealize(GtkWidget *widget, gpointer data)
{
    GpUiQuarantine *state = data;

    (void)widget;

    gp_ui_quarantine_remove_filter(state);
    GP_UI_LOG("%s: filter removed\n", __func__);
}

static void gp_ui_quarantine_destroy(GtkWidget *widget, gpointer data)
{
    GpUiQuarantine *state = data;

    (void)widget;

    gp_ui_quarantine_remove_filter(state);
}

static void gp_ui_quarantine_state_free(gpointer data)
{
    GpUiQuarantine *state = data;

    if (state == NULL)
        return;

    gp_ui_quarantine_set_pending_combo(state, NULL);
    gp_ui_quarantine_remove_filter(state);
    g_free(state);
}

static void gp_ui_quarantine_popup_notify(GObject *object,
                                          GParamSpec *pspec,
                                          gpointer data)
{
    GpUiQuarantine *state = data;
    GtkComboBox *combo = GTK_COMBO_BOX(object);
    gboolean shown;

    (void)pspec;

    if (state == NULL || combo == NULL)
        return;

    shown = gp_ui_combo_get_popup_property(combo, NULL);
    gp_ui_quarantine_update_combo_state(state, combo, shown);
}

void gp_ui_quarantine_install(GtkWidget *toplevel)
{
    GpUiQuarantine *state;

    if (toplevel == NULL)
        return;

    if (g_object_get_data(G_OBJECT(toplevel), GP_UI_QUARANTINE_STATE_KEY) != NULL)
        return;

    state = g_new0(GpUiQuarantine, 1);
    state->toplevel = toplevel;
    g_object_set_data_full(G_OBJECT(toplevel), GP_UI_QUARANTINE_STATE_KEY,
                           state, gp_ui_quarantine_state_free);

    state->realize_id = g_signal_connect(toplevel, "realize",
                                         G_CALLBACK(gp_ui_quarantine_realize), state);
    state->unrealize_id = g_signal_connect(toplevel, "unrealize",
                                           G_CALLBACK(gp_ui_quarantine_unrealize), state);
    state->destroy_id = g_signal_connect(toplevel, "destroy",
                                         G_CALLBACK(gp_ui_quarantine_destroy), state);

    if (gtk_widget_get_realized(toplevel))
        gp_ui_quarantine_realize(toplevel, state);

    GP_UI_LOG("%s: installed toplevel=%p\n", __func__, (void *)toplevel);
}

void gp_ui_quarantine_register_combo(GtkWidget *toplevel, GtkComboBox *combo)
{
    GpUiQuarantine *state;
    gboolean has_prop = FALSE;
    gboolean shown;

    if (toplevel == NULL || combo == NULL)
        return;

    gp_ui_quarantine_install(toplevel);
    state = g_object_get_data(G_OBJECT(toplevel), GP_UI_QUARANTINE_STATE_KEY);
    if (state == NULL)
        return;

    if (g_object_get_data(G_OBJECT(combo), GP_UI_QUARANTINE_COMBO_KEY) != NULL)
        return;

    g_object_set_data(G_OBJECT(combo), GP_UI_QUARANTINE_COMBO_KEY, state);

    shown = gp_ui_combo_get_popup_property(combo, &has_prop);
    gp_ui_combo_set_tracked_state(combo, shown);

    if (has_prop)
        g_signal_connect(combo, "notify::popup-shown",
                         G_CALLBACK(gp_ui_quarantine_popup_notify), state);
    g_signal_connect(combo, "popup",
                     G_CALLBACK(gp_ui_quarantine_combo_popup), state);
    g_signal_connect(combo, "popdown",
                     G_CALLBACK(gp_ui_quarantine_combo_popdown), state);

    GP_UI_LOG("%s: combo registered %p\n", __func__, (void *)combo);
}
