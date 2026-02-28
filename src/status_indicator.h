/*
 * Copyright (C) 2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#ifndef STATUS_INDICATOR_H
#define STATUS_INDICATOR_H

#include <gtk/gtk.h>

#include "ui-status.h"

typedef enum {
    UI_SEVERITY_GREY = 0,
    UI_SEVERITY_BLUE,
    UI_SEVERITY_GREEN,
    UI_SEVERITY_AMBER,
    UI_SEVERITY_RED
} UiSeverity;

typedef enum {
    STATUS_INDICATOR_PULSE_NONE = 0,
    STATUS_INDICATOR_PULSE_SLOW,
    STATUS_INDICATOR_PULSE_FAST
} StatusIndicatorPulseMode;

GtkWidget *status_indicator_new(void);

void status_indicator_set_severity(GtkWidget *indicator, UiSeverity severity);
void status_indicator_set_pulse_mode(GtkWidget *indicator,
                                     StatusIndicatorPulseMode mode);
UiSeverity status_indicator_get_severity(GtkWidget *indicator);

const char *ui_severity_to_string(UiSeverity severity);

UiSeverity rotor_ui_status_to_severity(RotorUiStatus status);
UiSeverity radio_ui_status_to_severity(RadioUiStatus status);
StatusIndicatorPulseMode rotor_ui_status_to_pulse_mode(RotorUiStatus status);
StatusIndicatorPulseMode radio_ui_status_to_pulse_mode(RadioUiStatus status);

#endif
