/*
 * Copyright (C) 2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#ifndef UI_STATUS_H
#define UI_STATUS_H

#include <glib.h>

#define RIG_UI_RELIABILITY_WINDOW_SIZE 5u
#define RIG_UI_ACTIVE_WINDOW_MS 2000

typedef enum {
    ROTOR_UI_STATUS_DISENGAGED = 0,
    ROTOR_UI_STATUS_ENGAGING,
    ROTOR_UI_STATUS_STANDBY,
    ROTOR_UI_STATUS_MOVING,
    ROTOR_UI_STATUS_PRETRACK,
    ROTOR_UI_STATUS_ON_TARGET,
    ROTOR_UI_STATUS_DEGRADED,
    ROTOR_UI_STATUS_LINK_LOST,
    ROTOR_UI_STATUS_ERROR
} RotorUiStatus;

typedef enum {
    RADIO_UI_STATUS_DISENGAGED = 0,
    RADIO_UI_STATUS_ENGAGING,
    RADIO_UI_STATUS_STANDBY,
    RADIO_UI_STATUS_STABLE,
    RADIO_UI_STATUS_DEGRADED,
    RADIO_UI_STATUS_LINK_LOST,
    RADIO_UI_STATUS_ERROR
} RadioUiStatus;

typedef enum {
    RIG_UI_CMD_OK = 0,
    RIG_UI_CMD_COMMAND_REJECT,
    RIG_UI_CMD_LINK_FAIL
} RigUiCommandOutcome;

typedef struct {
    gboolean control_active;
    gboolean engaging;
    gboolean hard_error;
    gboolean link_lost;
    gboolean degraded;
    gboolean moving;
    gboolean pretracking;
    gboolean on_target;
} RotorStateSnapshot;

typedef struct {
    gboolean control_active;
    gboolean engaging;
    gboolean hard_error;
    gboolean link_lost;
    gboolean degraded;
    gboolean active_flow;
    guint    consecutive_link_failures;
    guint    link_fail_count_in_window;
    guint    total_fail_count_in_window;
    guint    ok_count_in_window;
} RigStateSnapshot;

typedef struct {
    RigUiCommandOutcome outcomes[RIG_UI_RELIABILITY_WINDOW_SIZE];
    guint               count;
    guint               next_idx;
    gint64              last_command_us;
} RigUiCommandWindow;

typedef struct {
    guint    ok_count;
    guint    reject_count;
    guint    link_fail_count;
    guint    total_fail_count;
    guint    consecutive_link_failures;
    gboolean active;
} RigUiCommandWindowStats;

const char *rotor_ui_status_to_string(RotorUiStatus s);
const char *radio_ui_status_to_string(RadioUiStatus s);

RotorUiStatus rotor_compute_ui_status(const RotorStateSnapshot *s);
RadioUiStatus radio_compute_ui_status(const RigStateSnapshot *s);

void rig_ui_command_window_init(RigUiCommandWindow *window);
void rig_ui_command_window_record(RigUiCommandWindow *window,
                                  RigUiCommandOutcome outcome,
                                  gint64 now_us);
void rig_ui_command_window_get_stats(const RigUiCommandWindow *window,
                                     gint64 now_us,
                                     gint64 active_window_ms,
                                     RigUiCommandWindowStats *stats);

#endif
