/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include "ui-combo-guard.h"

#include "sat-log.h"

#define GP_COMBO_GUARD_STATE_KEY "gpredict-combo-guard-state"

typedef struct {
    GtkComboBox *combo;
    GtkWidget   *toplevel;
    gboolean     popup_shown;
    gboolean     armed;
    guint        poll_id;
    guint        poll_ticks;
    gint64       poll_start_us;
    gint64       popdown_guard_until_us;
    gint64       post_guard_until_us;
    guint        post_guard_id;
    guint32      press_time;
    guint32      release_time;
    gulong       notify_id;
    gulong       press_id;
    gulong       release_id;
    gulong       toplevel_press_id;
    gulong       toplevel_release_id;
} GpComboGuardState;

static gboolean gp_combo_guard_debug_enabled(void)
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
        if (gp_combo_guard_debug_enabled())                         \
            sat_log_log(SAT_LOG_LEVEL_DEBUG, __VA_ARGS__);          \
    } while (0)

static void gp_combo_guard_disconnect_post_guard(GpComboGuardState *state)
{
    if (state->post_guard_id != 0)
    {
        g_source_remove(state->post_guard_id);
        state->post_guard_id = 0;
    }

    if (state->toplevel != NULL && state->toplevel_press_id != 0)
    {
        g_signal_handler_disconnect(state->toplevel, state->toplevel_press_id);
        state->toplevel_press_id = 0;
    }

    if (state->toplevel != NULL && state->toplevel_release_id != 0)
    {
        g_signal_handler_disconnect(state->toplevel, state->toplevel_release_id);
        state->toplevel_release_id = 0;
    }
}

static void gp_combo_guard_state_free(gpointer data)
{
    GpComboGuardState *state = data;

    if (state == NULL)
        return;

    if (state->poll_id != 0)
        g_source_remove(state->poll_id);

    gp_combo_guard_disconnect_post_guard(state);

    g_free(state);
}

static GpComboGuardState *gp_combo_guard_state(GtkComboBox *combo)
{
    GpComboGuardState *state;

    state = g_object_get_data(G_OBJECT(combo), GP_COMBO_GUARD_STATE_KEY);
    if (state == NULL)
    {
        state = g_new0(GpComboGuardState, 1);
        g_object_set_data_full(G_OBJECT(combo), GP_COMBO_GUARD_STATE_KEY,
                               state, gp_combo_guard_state_free);
    }

    return state;
}

static gboolean gp_combo_guard_popup_shown(GtkComboBox *combo)
{
    gboolean shown = FALSE;

    if (combo == NULL)
        return FALSE;

    if (g_object_class_find_property(G_OBJECT_GET_CLASS(combo), "popup-shown"))
        g_object_get(combo, "popup-shown", &shown, NULL);

    return shown;
}

static GdkWindow *gp_combo_guard_get_window(GpComboGuardState *state)
{
    if (state->toplevel != NULL)
    {
        GdkWindow *window = gtk_widget_get_window(state->toplevel);
        if (window != NULL)
            return window;
    }

    return gtk_widget_get_window(GTK_WIDGET(state->combo));
}

static gboolean gp_combo_guard_button1_down(GpComboGuardState *state)
{
    GdkWindow       *window;
    GdkDisplay      *display;
    GdkSeat         *seat;
    GdkDevice       *pointer;
    GdkModifierType  mask = 0;

    window = gp_combo_guard_get_window(state);
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

static gboolean gp_combo_guard_post_guard_cb(gpointer data)
{
    GpComboGuardState *state = data;
    gint64             now_us = g_get_monotonic_time();

    if (now_us >= state->post_guard_until_us || !state->popup_shown)
    {
        GP_UI_LOG("%s: post-open guard disabled\n", __func__);
        gp_combo_guard_disconnect_post_guard(state);
        return G_SOURCE_REMOVE;
    }

    return G_SOURCE_CONTINUE;
}

static gboolean gp_combo_guard_toplevel_button(GtkWidget *widget,
                                               GdkEventButton *event,
                                               gpointer data)
{
    GpComboGuardState *state = data;
    gint64             now_us = g_get_monotonic_time();

    (void)widget;

    if (event == NULL)
        return FALSE;

    if (event->button != 1)
        return FALSE;

    if (!state->popup_shown)
        return FALSE;

    if (now_us >= state->post_guard_until_us)
        return FALSE;

    GP_UI_LOG("%s: swallow post-open click\n", __func__);

    return TRUE;
}

static void gp_combo_guard_enable_post_guard(GpComboGuardState *state)
{
    gint64 now_us = g_get_monotonic_time();

    if (state->toplevel == NULL)
        return;

    state->post_guard_until_us = now_us + 120000;

    if (state->toplevel_press_id == 0)
    {
        state->toplevel_press_id =
            g_signal_connect(state->toplevel, "button-press-event",
                             G_CALLBACK(gp_combo_guard_toplevel_button), state);
    }

    if (state->toplevel_release_id == 0)
    {
        state->toplevel_release_id =
            g_signal_connect(state->toplevel, "button-release-event",
                             G_CALLBACK(gp_combo_guard_toplevel_button), state);
    }

    if (state->post_guard_id != 0)
        g_source_remove(state->post_guard_id);

    state->post_guard_id = g_timeout_add_full(G_PRIORITY_DEFAULT, 20,
                                              gp_combo_guard_post_guard_cb,
                                              state, NULL);

    GP_UI_LOG("%s: post-open guard enabled until=%lld\n", __func__,
              (long long)state->post_guard_until_us);
}

static gboolean gp_combo_guard_popup_idle(gpointer data)
{
    GtkComboBox     *combo = GTK_COMBO_BOX(data);
    GpComboGuardState *state = gp_combo_guard_state(combo);
    GtkWidget       *widget = GTK_WIDGET(combo);

    GP_UI_LOG("%s: popup open\n", __func__);

    if (gtk_widget_get_realized(widget) && gtk_widget_get_visible(widget))
        gtk_combo_box_popup(combo);

    gp_combo_guard_enable_post_guard(state);

    return G_SOURCE_REMOVE;
}

static gboolean gp_combo_guard_poll_cb(gpointer data)
{
    GtkComboBox     *combo = GTK_COMBO_BOX(data);
    GpComboGuardState *state = gp_combo_guard_state(combo);
    gint64           now_us = g_get_monotonic_time();

    if (state->popup_shown)
    {
        state->poll_id = 0;
        return G_SOURCE_REMOVE;
    }

    state->poll_ticks += 1;

    if (now_us >= state->popdown_guard_until_us &&
        !gp_combo_guard_button1_down(state))
    {
        state->poll_id = 0;
        GP_UI_LOG("%s: ready to open now=%lld ticks=%u\n", __func__,
                  (long long)now_us, state->poll_ticks);
        g_idle_add_full(G_PRIORITY_DEFAULT_IDLE,
                        gp_combo_guard_popup_idle,
                        g_object_ref(combo),
                        g_object_unref);
        return G_SOURCE_REMOVE;
    }

    GP_UI_LOG("%s: waiting now=%lld ticks=%u\n", __func__,
              (long long)now_us, state->poll_ticks);

    if (now_us - state->poll_start_us > 300000)
    {
        state->poll_id = 0;
        GP_UI_LOG("%s: timeout after %lld us\n", __func__,
                  (long long)(now_us - state->poll_start_us));
        return G_SOURCE_REMOVE;
    }

    return G_SOURCE_CONTINUE;
}

static void gp_combo_guard_schedule_poll(GpComboGuardState *state,
                                         const gchar *reason)
{
    gint64 now_us = g_get_monotonic_time();

    if (state->poll_id != 0)
        return;

    state->poll_ticks = 0;
    state->poll_start_us = now_us;
    state->poll_id = g_timeout_add_full(G_PRIORITY_DEFAULT, 10,
                                        gp_combo_guard_poll_cb,
                                        g_object_ref(state->combo),
                                        g_object_unref);

    GP_UI_LOG("%s: poll start reason=%s now=%lld id=%u\n", __func__,
              reason ? reason : "unknown", (long long)now_us, state->poll_id);
}

static void gp_combo_guard_popup_notify(GObject *object,
                                        GParamSpec *pspec,
                                        gpointer data)
{
    GtkComboBox      *combo = GTK_COMBO_BOX(object);
    GpComboGuardState *state = data;
    gint64            now_us = g_get_monotonic_time();
    gboolean          shown = gp_combo_guard_popup_shown(combo);

    (void)pspec;

    state->popup_shown = shown;

    if (shown)
    {
        GP_UI_LOG("%s: popup shown\n", __func__);
        if (state->poll_id != 0)
        {
            g_source_remove(state->poll_id);
            state->poll_id = 0;
        }
    }
    else
    {
        state->popdown_guard_until_us = now_us + 250000;
        GP_UI_LOG("%s: popup hidden guard until=%lld\n", __func__,
                  (long long)state->popdown_guard_until_us);
        gp_combo_guard_disconnect_post_guard(state);
    }
}

static gboolean gp_combo_guard_button_press(GtkWidget *widget,
                                            GdkEventButton *event,
                                            gpointer data)
{
    GpComboGuardState *state = data;
    gint64             now_us = g_get_monotonic_time();

    (void)widget;

    if (event == NULL)
        return FALSE;

    if (event->button != 1)
        return FALSE;

    if (state->popup_shown)
        return FALSE;

    state->armed = TRUE;
    state->press_time = event->time;

    GP_UI_LOG("%s: press time=%u now=%lld consumed\n", __func__,
              event->time, (long long)now_us);

    return TRUE;
}

static gboolean gp_combo_guard_button_release(GtkWidget *widget,
                                              GdkEventButton *event,
                                              gpointer data)
{
    GpComboGuardState *state = data;
    gint64             now_us = g_get_monotonic_time();

    (void)widget;

    if (event == NULL)
        return FALSE;

    if (event->button != 1)
        return FALSE;

    if (!state->armed)
        return FALSE;

    if (state->popup_shown)
    {
        state->armed = FALSE;
        return FALSE;
    }

    state->armed = FALSE;
    state->release_time = event->time;

    GP_UI_LOG("%s: release time=%u now=%lld consumed\n", __func__,
              event->time, (long long)now_us);

    gp_combo_guard_schedule_poll(state,
                                 now_us < state->popdown_guard_until_us ?
                                 "popdown_guard" : "normal");

    return TRUE;
}

static void gp_combo_guard_toplevel_gone(gpointer data, GObject *where)
{
    GpComboGuardState *state = data;

    (void)where;

    state->toplevel = NULL;
    gp_combo_guard_disconnect_post_guard(state);
}

void gp_combo_guard_install(GtkComboBox *combo, GtkWidget *toplevel)
{
    GpComboGuardState *state;

    if (combo == NULL)
        return;

    if (g_object_get_data(G_OBJECT(combo), GP_COMBO_GUARD_STATE_KEY) != NULL)
        return;

    state = gp_combo_guard_state(combo);
    state->combo = combo;
    state->toplevel = toplevel;

    if (toplevel != NULL)
        g_object_weak_ref(G_OBJECT(toplevel), gp_combo_guard_toplevel_gone, state);

    state->notify_id = g_signal_connect(combo, "notify::popup-shown",
                                        G_CALLBACK(gp_combo_guard_popup_notify),
                                        state);
    state->press_id = g_signal_connect(combo, "button-press-event",
                                       G_CALLBACK(gp_combo_guard_button_press),
                                       state);
    state->release_id = g_signal_connect(combo, "button-release-event",
                                         G_CALLBACK(gp_combo_guard_button_release),
                                         state);

    gtk_widget_add_events(GTK_WIDGET(combo),
                          GDK_BUTTON_PRESS_MASK | GDK_BUTTON_RELEASE_MASK);

    GP_UI_LOG("%s: installed combo=%p toplevel=%p\n", __func__,
              (void *)combo, (void *)toplevel);
}
