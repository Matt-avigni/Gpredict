/*
 * Copyright (C) 2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include <math.h>

#include "status_indicator.h"

#define STATUS_INDICATOR_SIZE 14
#define STATUS_INDICATOR_TICK_MS 50

typedef struct {
    UiSeverity               severity;
    StatusIndicatorPulseMode pulse_mode;
    gdouble                  phase;
    guint                    tick_id;
} StatusIndicatorState;

static const gchar STATUS_INDICATOR_STATE_KEY[] = "status-indicator-state";

static StatusIndicatorState *status_indicator_get_state(GtkWidget *indicator)
{
    if (indicator == NULL)
        return NULL;

    return g_object_get_data(G_OBJECT(indicator), STATUS_INDICATOR_STATE_KEY);
}

static void status_indicator_state_free(gpointer data)
{
    StatusIndicatorState *state = data;

    if (state == NULL)
        return;

    if (state->tick_id != 0)
    {
        g_source_remove(state->tick_id);
        state->tick_id = 0;
    }

    g_free(state);
}

static void status_indicator_color(UiSeverity severity,
                                   gdouble *r, gdouble *g, gdouble *b)
{
    if (r == NULL || g == NULL || b == NULL)
        return;

    switch (severity)
    {
    case UI_SEVERITY_GREY:
        *r = 0x9e / 255.0;
        *g = 0x9e / 255.0;
        *b = 0x9e / 255.0;
        return;
    case UI_SEVERITY_BLUE:
        *r = 0x3a / 255.0;
        *g = 0x7b / 255.0;
        *b = 0xd5 / 255.0;
        return;
    case UI_SEVERITY_GREEN:
        *r = 0x2e / 255.0;
        *g = 0x7d / 255.0;
        *b = 0x32 / 255.0;
        return;
    case UI_SEVERITY_AMBER:
        *r = 0xf9 / 255.0;
        *g = 0xa8 / 255.0;
        *b = 0x25 / 255.0;
        return;
    case UI_SEVERITY_RED:
    default:
        *r = 0xc6 / 255.0;
        *g = 0x28 / 255.0;
        *b = 0x28 / 255.0;
        return;
    }
}

static gdouble status_indicator_alpha(const StatusIndicatorState *state)
{
    gdouble wave;

    if (state == NULL || state->pulse_mode == STATUS_INDICATOR_PULSE_NONE)
        return 1.0;

    wave = 0.5 * (sin(state->phase) + 1.0);
    return 0.6 + (0.4 * wave);
}

static gboolean status_indicator_draw(GtkWidget *widget,
                                      cairo_t *cr,
                                      gpointer user_data)
{
    StatusIndicatorState *state = status_indicator_get_state(widget);
    GtkAllocation alloc = { 0 };
    gdouble red = 0.0;
    gdouble green = 0.0;
    gdouble blue = 0.0;
    gdouble alpha;
    gdouble border_scale = 0.72;
    gdouble radius;
    gdouble center_x;
    gdouble center_y;

    (void)user_data;

    if (state == NULL || cr == NULL)
        return FALSE;

    gtk_widget_get_allocation(widget, &alloc);
    if (alloc.width <= 2 || alloc.height <= 2)
        return FALSE;

    status_indicator_color(state->severity, &red, &green, &blue);
    alpha = status_indicator_alpha(state);

    radius = (MIN(alloc.width, alloc.height) - 2.0) * 0.5;
    center_x = alloc.width * 0.5;
    center_y = alloc.height * 0.5;

    cairo_arc(cr, center_x, center_y, radius, 0.0, 2.0 * G_PI);
    cairo_set_source_rgba(cr, red, green, blue, alpha);
    cairo_fill_preserve(cr);

    cairo_set_line_width(cr, 1.0);
    cairo_set_source_rgba(cr,
                          red * border_scale,
                          green * border_scale,
                          blue * border_scale,
                          1.0);
    cairo_stroke(cr);

    return FALSE;
}

static gboolean status_indicator_tick(gpointer data)
{
    GtkWidget *indicator = GTK_WIDGET(data);
    StatusIndicatorState *state = status_indicator_get_state(indicator);
    gdouble step;

    if (state == NULL)
        return G_SOURCE_REMOVE;

    if (state->pulse_mode == STATUS_INDICATOR_PULSE_NONE)
    {
        state->tick_id = 0;
        return G_SOURCE_REMOVE;
    }

    step = (state->pulse_mode == STATUS_INDICATOR_PULSE_FAST) ? 0.30 : 0.16;
    state->phase += step;
    if (state->phase > (2.0 * G_PI))
        state->phase -= (2.0 * G_PI);

    gtk_widget_queue_draw(indicator);
    return G_SOURCE_CONTINUE;
}

static void status_indicator_update_timer(GtkWidget *indicator,
                                          StatusIndicatorState *state)
{
    if (indicator == NULL || state == NULL)
        return;

    if (state->pulse_mode == STATUS_INDICATOR_PULSE_NONE)
    {
        if (state->tick_id != 0)
        {
            g_source_remove(state->tick_id);
            state->tick_id = 0;
        }
        return;
    }

    if (state->tick_id == 0)
    {
        state->tick_id = g_timeout_add_full(G_PRIORITY_DEFAULT,
                                            STATUS_INDICATOR_TICK_MS,
                                            status_indicator_tick,
                                            g_object_ref(indicator),
                                            g_object_unref);
    }
}

GtkWidget *status_indicator_new(void)
{
    GtkWidget *indicator = gtk_drawing_area_new();
    StatusIndicatorState *state = g_new0(StatusIndicatorState, 1);

    state->severity = UI_SEVERITY_GREY;
    state->pulse_mode = STATUS_INDICATOR_PULSE_NONE;
    state->phase = 0.0;
    state->tick_id = 0;

    gtk_widget_set_size_request(indicator,
                                STATUS_INDICATOR_SIZE,
                                STATUS_INDICATOR_SIZE);
    gtk_widget_set_halign(indicator, GTK_ALIGN_START);
    gtk_widget_set_valign(indicator, GTK_ALIGN_CENTER);

    g_object_set_data_full(G_OBJECT(indicator),
                           STATUS_INDICATOR_STATE_KEY,
                           state,
                           status_indicator_state_free);

    g_signal_connect(indicator, "draw",
                     G_CALLBACK(status_indicator_draw), NULL);

    return indicator;
}

void status_indicator_set_severity(GtkWidget *indicator, UiSeverity severity)
{
    StatusIndicatorState *state = status_indicator_get_state(indicator);

    if (state == NULL || state->severity == severity)
        return;

    g_debug("status indicator: severity %s -> %s",
            ui_severity_to_string(state->severity),
            ui_severity_to_string(severity));
    state->severity = severity;
    gtk_widget_queue_draw(indicator);
}

void status_indicator_set_pulse_mode(GtkWidget *indicator,
                                     StatusIndicatorPulseMode mode)
{
    StatusIndicatorState *state = status_indicator_get_state(indicator);

    if (state == NULL || state->pulse_mode == mode)
        return;

    state->pulse_mode = mode;
    if (mode == STATUS_INDICATOR_PULSE_NONE)
        state->phase = 0.0;
    status_indicator_update_timer(indicator, state);
    gtk_widget_queue_draw(indicator);
}

UiSeverity status_indicator_get_severity(GtkWidget *indicator)
{
    StatusIndicatorState *state = status_indicator_get_state(indicator);

    if (state == NULL)
        return UI_SEVERITY_GREY;

    return state->severity;
}

const char *ui_severity_to_string(UiSeverity severity)
{
    switch (severity)
    {
    case UI_SEVERITY_GREY:
        return "GREY";
    case UI_SEVERITY_BLUE:
        return "BLUE";
    case UI_SEVERITY_GREEN:
        return "GREEN";
    case UI_SEVERITY_AMBER:
        return "AMBER";
    case UI_SEVERITY_RED:
    default:
        return "RED";
    }
}

UiSeverity rotor_ui_status_to_severity(RotorUiStatus status)
{
    switch (status)
    {
    case ROTOR_UI_STATUS_ENGAGING:
    case ROTOR_UI_STATUS_MOVING:
        return UI_SEVERITY_BLUE;
    case ROTOR_UI_STATUS_ON_TARGET:
        return UI_SEVERITY_GREEN;
    case ROTOR_UI_STATUS_DEGRADED:
        return UI_SEVERITY_AMBER;
    case ROTOR_UI_STATUS_LINK_LOST:
    case ROTOR_UI_STATUS_ERROR:
        return UI_SEVERITY_RED;
    case ROTOR_UI_STATUS_DISENGAGED:
    case ROTOR_UI_STATUS_STANDBY:
    default:
        return UI_SEVERITY_GREY;
    }
}

UiSeverity radio_ui_status_to_severity(RadioUiStatus status)
{
    switch (status)
    {
    case RADIO_UI_STATUS_ENGAGING:
        return UI_SEVERITY_BLUE;
    case RADIO_UI_STATUS_STABLE:
        return UI_SEVERITY_GREEN;
    case RADIO_UI_STATUS_DEGRADED:
        return UI_SEVERITY_AMBER;
    case RADIO_UI_STATUS_LINK_LOST:
    case RADIO_UI_STATUS_ERROR:
        return UI_SEVERITY_RED;
    case RADIO_UI_STATUS_DISENGAGED:
    case RADIO_UI_STATUS_STANDBY:
    default:
        return UI_SEVERITY_GREY;
    }
}

StatusIndicatorPulseMode rotor_ui_status_to_pulse_mode(RotorUiStatus status)
{
    switch (status)
    {
    case ROTOR_UI_STATUS_ENGAGING:
        return STATUS_INDICATOR_PULSE_SLOW;
    case ROTOR_UI_STATUS_MOVING:
        return STATUS_INDICATOR_PULSE_FAST;
    case ROTOR_UI_STATUS_LINK_LOST:
        return STATUS_INDICATOR_PULSE_SLOW;
    default:
        return STATUS_INDICATOR_PULSE_NONE;
    }
}

StatusIndicatorPulseMode radio_ui_status_to_pulse_mode(RadioUiStatus status)
{
    switch (status)
    {
    case RADIO_UI_STATUS_ENGAGING:
        return STATUS_INDICATOR_PULSE_SLOW;
    case RADIO_UI_STATUS_LINK_LOST:
        return STATUS_INDICATOR_PULSE_SLOW;
    default:
        return STATUS_INDICATOR_PULSE_NONE;
    }
}
