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

static gboolean gp_ui_quarantine_popup_shown(GtkComboBox *combo)
{
    gboolean shown = FALSE;

    if (combo == NULL)
        return FALSE;

    if (g_object_class_find_property(G_OBJECT_GET_CLASS(combo), "popup-shown"))
        g_object_get(combo, "popup-shown", &shown, NULL);

    return shown;
}

static void gp_ui_quarantine_reset_swallow(GpUiQuarantine *state)
{
    state->swallowed_press = FALSE;
    state->swallowed_release = FALSE;
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

    gp_ui_quarantine_remove_filter(state);
    g_free(state);
}

static void gp_ui_quarantine_popup_notify(GObject *object,
                                          GParamSpec *pspec,
                                          gpointer data)
{
    GpUiQuarantine *state = data;
    GtkComboBox *combo = GTK_COMBO_BOX(object);
    gint64 now_us = g_get_monotonic_time();
    gboolean shown;
    gboolean prev_shown;

    (void)pspec;

    if (state == NULL || combo == NULL)
        return;

    shown = gp_ui_quarantine_popup_shown(combo);
    prev_shown = GPOINTER_TO_INT(g_object_get_data(object, GP_UI_QUARANTINE_POPUP_KEY));

    if (prev_shown && !shown)
    {
        state->last_popup_popdown_us = now_us;
        state->quarantine_until_us = now_us + 250000;
        gp_ui_quarantine_reset_swallow(state);
        GP_UI_LOG("%s: popdown quarantine until=%lld us\n", __func__,
                  (long long)state->quarantine_until_us);
    }

    g_object_set_data(object, GP_UI_QUARANTINE_POPUP_KEY,
                      GINT_TO_POINTER(shown));
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

    shown = gp_ui_quarantine_popup_shown(combo);
    g_object_set_data(G_OBJECT(combo), GP_UI_QUARANTINE_POPUP_KEY,
                      GINT_TO_POINTER(shown));

    g_signal_connect(combo, "notify::popup-shown",
                     G_CALLBACK(gp_ui_quarantine_popup_notify), state);

    GP_UI_LOG("%s: combo registered %p\n", __func__, (void *)combo);
}
