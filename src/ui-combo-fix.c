/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include "ui-combo-fix.h"

#include "sat-log.h"

#include <gdk/gdkkeysyms.h>

#define GP_COMBO_FIX_STATE_KEY "gpredict-combo-fix-state"
#define GP_COMBO_FIX_TARGET_KEY "gpredict-combo-fix-target"

typedef struct {
    gboolean armed;
    guint wait_id;
    guint tick_count;
    guint32 press_time;
    guint32 release_time;
    gint64 press_us;
    gint64 release_us;
    gint64 start_us;
} GpComboFixState;

#ifdef GP_UI_DEBUG
#define GP_UI_LOG(...) g_printerr(__VA_ARGS__)
#else
#define GP_UI_LOG(...) sat_log_log(SAT_LOG_LEVEL_DEBUG, __VA_ARGS__)
#endif

static void gp_combo_fix_state_free(gpointer data)
{
    GpComboFixState *state = data;

    if (state == NULL)
        return;

    if (state->wait_id != 0)
        g_source_remove(state->wait_id);

    g_free(state);
}

static GpComboFixState *gp_combo_fix_state(GtkComboBox *combo)
{
    GpComboFixState *state;

    state = g_object_get_data(G_OBJECT(combo), GP_COMBO_FIX_STATE_KEY);
    if (state == NULL)
    {
        state = g_new0(GpComboFixState, 1);
        g_object_set_data_full(G_OBJECT(combo), GP_COMBO_FIX_STATE_KEY,
                               state, gp_combo_fix_state_free);
    }

    return state;
}

static gboolean gp_combo_fix_popup_shown(GtkComboBox *combo)
{
    gboolean shown = FALSE;

    if (combo == NULL)
        return FALSE;

    if (g_object_class_find_property(G_OBJECT_GET_CLASS(combo), "popup-shown"))
        g_object_get(combo, "popup-shown", &shown, NULL);

    return shown;
}

static GdkWindow *gp_combo_fix_get_window(GtkComboBox *combo)
{
    GtkWidget *target;
    GdkWindow *window;

    if (combo == NULL)
        return NULL;

    target = g_object_get_data(G_OBJECT(combo), GP_COMBO_FIX_TARGET_KEY);
    if (target != NULL)
    {
        window = gtk_widget_get_window(target);
        if (window != NULL)
            return window;
    }

    return gtk_widget_get_window(GTK_WIDGET(combo));
}

static gboolean gp_combo_fix_button1_down(GtkComboBox *combo)
{
    GdkWindow       *window;
    GdkDisplay      *display;
    GdkSeat         *seat;
    GdkDevice       *pointer;
    GdkModifierType  mask = 0;

    window = gp_combo_fix_get_window(combo);
    if (window == NULL)
        return FALSE;

    display = gdk_window_get_display(window);
    if (display == NULL)
        return FALSE;

    seat = gdk_display_get_default_seat(display);
    if (seat == NULL)
        return FALSE;

    pointer = gdk_seat_get_pointer(seat);
    if (pointer == NULL)
        return FALSE;

    gdk_window_get_device_position(window, pointer, NULL, NULL, &mask);

    return (mask & GDK_BUTTON1_MASK) != 0;
}

static gboolean gp_combo_fix_wait_cb(gpointer data)
{
    GtkComboBox     *combo = GTK_COMBO_BOX(data);
    GtkWidget       *widget = GTK_WIDGET(combo);
    GpComboFixState *state = gp_combo_fix_state(combo);
    gint64           now_us = g_get_monotonic_time();

    state->tick_count += 1;

    if (!gp_combo_fix_button1_down(combo))
    {
        state->wait_id = 0;

        GP_UI_LOG("%s: open popup now=%lld ticks=%u\n", __func__,
                  (long long)now_us, state->tick_count);

        if (gtk_widget_get_realized(widget) && gtk_widget_get_visible(widget))
            gtk_combo_box_popup(combo);

        return G_SOURCE_REMOVE;
    }

    GP_UI_LOG("%s: waiting mask down now=%lld ticks=%u\n", __func__,
              (long long)now_us, state->tick_count);

    if (now_us - state->start_us > 300000)
    {
        state->wait_id = 0;
        GP_UI_LOG("%s: timeout after %lld us\n", __func__,
                  (long long)(now_us - state->start_us));
        return G_SOURCE_REMOVE;
    }

    return G_SOURCE_CONTINUE;
}

static void gp_combo_fix_schedule_wait(GtkComboBox *combo, const gchar *reason)
{
    GpComboFixState *state = gp_combo_fix_state(combo);
    gint64           now_us = g_get_monotonic_time();

    if (state->wait_id != 0)
        return;

    state->tick_count = 0;
    state->start_us = now_us;
    state->wait_id = g_timeout_add_full(G_PRIORITY_DEFAULT, 10,
                                        gp_combo_fix_wait_cb,
                                        g_object_ref(combo),
                                        g_object_unref);

    GP_UI_LOG("%s: schedule wait reason=%s now=%lld id=%u\n", __func__,
              reason ? reason : "unknown", (long long)now_us, state->wait_id);
}

static gboolean gp_combo_fix_button_press(GtkWidget *widget,
                                          GdkEventButton *event,
                                          gpointer data)
{
    GtkComboBox     *combo = GTK_COMBO_BOX(data);
    GpComboFixState *state = gp_combo_fix_state(combo);
    gint64           now_us = g_get_monotonic_time();

    (void)widget;

    if (event == NULL)
        return FALSE;

    if (event->button != 1)
        return FALSE;

    if (gp_combo_fix_popup_shown(combo))
        return FALSE;

    state->armed = TRUE;
    state->press_time = event->time;
    state->press_us = now_us;

    GP_UI_LOG("%s: press time=%u now=%lld consumed\n", __func__,
              event->time, (long long)now_us);

    return TRUE;
}

static gboolean gp_combo_fix_button_release(GtkWidget *widget,
                                            GdkEventButton *event,
                                            gpointer data)
{
    GtkComboBox     *combo = GTK_COMBO_BOX(data);
    GpComboFixState *state = gp_combo_fix_state(combo);
    gint64           now_us = g_get_monotonic_time();

    (void)widget;

    if (event == NULL)
        return FALSE;

    if (event->button != 1)
        return FALSE;

    if (!state->armed)
        return FALSE;

    if (gp_combo_fix_popup_shown(combo))
    {
        state->armed = FALSE;
        return FALSE;
    }

    state->armed = FALSE;
    state->release_time = event->time;
    state->release_us = now_us;

    GP_UI_LOG("%s: release time=%u now=%lld consumed\n", __func__,
              event->time, (long long)now_us);

    gp_combo_fix_schedule_wait(combo, "release");

    return TRUE;
}

static gboolean gp_combo_fix_key_press(GtkWidget *widget,
                                       GdkEventKey *event,
                                       gpointer data)
{
    GtkComboBox     *combo = GTK_COMBO_BOX(data);
    GpComboFixState *state = gp_combo_fix_state(combo);
    gint64           now_us = g_get_monotonic_time();

    (void)widget;

    if (event == NULL)
        return FALSE;

    if (event->keyval != GDK_KEY_space &&
        event->keyval != GDK_KEY_Return &&
        event->keyval != GDK_KEY_KP_Enter &&
        event->keyval != GDK_KEY_Down)
        return FALSE;

    if (gp_combo_fix_popup_shown(combo))
        return FALSE;

    state->press_time = event->time;
    state->press_us = now_us;

    GP_UI_LOG("%s: key=%u now=%lld consumed\n", __func__,
              event->keyval, (long long)now_us);

    gp_combo_fix_schedule_wait(combo, "key");

    return TRUE;
}

static gboolean gp_combo_fix_scroll(GtkWidget *widget,
                                    GdkEventScroll *event,
                                    gpointer data)
{
    (void)widget;
    (void)event;
    (void)data;

    return FALSE;
}

static GtkWidget *gp_combo_fix_find_toggle(GtkWidget *widget)
{
    GList *children;
    GList *iter;

    if (widget == NULL)
        return NULL;

    if (GTK_IS_TOGGLE_BUTTON(widget))
        return widget;

    if (!GTK_IS_CONTAINER(widget))
        return NULL;

    children = gtk_container_get_children(GTK_CONTAINER(widget));
    for (iter = children; iter != NULL; iter = iter->next)
    {
        GtkWidget *found = gp_combo_fix_find_toggle(GTK_WIDGET(iter->data));
        if (found != NULL)
        {
            g_list_free(children);
            return found;
        }
    }
    g_list_free(children);
    return NULL;
}

void gp_combo_fix_install(GtkComboBox *combo)
{
    GtkWidget *child;
    GtkWidget *target;

    if (combo == NULL)
        return;

    if (g_object_get_data(G_OBJECT(combo), GP_COMBO_FIX_TARGET_KEY) != NULL)
        return;

    child = gtk_bin_get_child(GTK_BIN(combo));
    target = gp_combo_fix_find_toggle(child);
    if (target == NULL)
        target = child != NULL ? child : GTK_WIDGET(combo);

    g_object_set_data(G_OBJECT(combo), GP_COMBO_FIX_TARGET_KEY, target);

    GP_UI_LOG("%s: hook target=%s\n", __func__, G_OBJECT_TYPE_NAME(target));

    gtk_widget_add_events(target,
                          GDK_BUTTON_PRESS_MASK |
                          GDK_BUTTON_RELEASE_MASK |
                          GDK_SCROLL_MASK |
                          GDK_KEY_PRESS_MASK);
    g_signal_connect(target, "button-press-event",
                     G_CALLBACK(gp_combo_fix_button_press), combo);
    g_signal_connect(target, "button-release-event",
                     G_CALLBACK(gp_combo_fix_button_release), combo);
    g_signal_connect(target, "key-press-event",
                     G_CALLBACK(gp_combo_fix_key_press), combo);
    g_signal_connect(target, "scroll-event",
                     G_CALLBACK(gp_combo_fix_scroll), NULL);
}
