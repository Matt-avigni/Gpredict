/*
 * Copyright (C) 2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include "ui-status.h"

static guint rig_ui_window_index_from_newest(const RigUiCommandWindow *window,
                                             guint age)
{
    guint newest;

    newest = (window->next_idx + RIG_UI_RELIABILITY_WINDOW_SIZE - 1u) %
             RIG_UI_RELIABILITY_WINDOW_SIZE;
    return (newest + RIG_UI_RELIABILITY_WINDOW_SIZE - age) %
           RIG_UI_RELIABILITY_WINDOW_SIZE;
}

const char *rotor_ui_status_to_string(RotorUiStatus s)
{
    switch (s)
    {
    case ROTOR_UI_STATUS_DISENGAGED:
        return "DISENGAGED";
    case ROTOR_UI_STATUS_ENGAGING:
        return "ENGAGING";
    case ROTOR_UI_STATUS_STANDBY:
        return "STANDBY";
    case ROTOR_UI_STATUS_MOVING:
        return "MOVING";
    case ROTOR_UI_STATUS_ON_TARGET:
        return "ON TARGET";
    case ROTOR_UI_STATUS_DEGRADED:
        return "DEGRADED";
    case ROTOR_UI_STATUS_LINK_LOST:
        return "LINK LOST";
    case ROTOR_UI_STATUS_ERROR:
        return "ERROR";
    default:
        return "ERROR";
    }
}

const char *radio_ui_status_to_string(RadioUiStatus s)
{
    switch (s)
    {
    case RADIO_UI_STATUS_DISENGAGED:
        return "DISENGAGED";
    case RADIO_UI_STATUS_ENGAGING:
        return "ENGAGING";
    case RADIO_UI_STATUS_STANDBY:
        return "STANDBY";
    case RADIO_UI_STATUS_STABLE:
        return "STABLE";
    case RADIO_UI_STATUS_DEGRADED:
        return "DEGRADED";
    case RADIO_UI_STATUS_LINK_LOST:
        return "LINK LOST";
    case RADIO_UI_STATUS_ERROR:
        return "ERROR";
    default:
        return "ERROR";
    }
}

RotorUiStatus rotor_compute_ui_status(const RotorStateSnapshot *s)
{
    if (s == NULL)
        return ROTOR_UI_STATUS_DISENGAGED;

    if (!s->control_active)
        return ROTOR_UI_STATUS_DISENGAGED;
    if (s->engaging)
        return ROTOR_UI_STATUS_ENGAGING;
    if (s->hard_error)
        return ROTOR_UI_STATUS_ERROR;
    if (s->link_lost)
        return ROTOR_UI_STATUS_LINK_LOST;
    if (s->degraded)
        return ROTOR_UI_STATUS_DEGRADED;
    if (s->moving)
        return ROTOR_UI_STATUS_MOVING;
    if (s->on_target)
        return ROTOR_UI_STATUS_ON_TARGET;

    return ROTOR_UI_STATUS_STANDBY;
}

RadioUiStatus radio_compute_ui_status(const RigStateSnapshot *s)
{
    if (s == NULL)
        return RADIO_UI_STATUS_DISENGAGED;

    if (!s->control_active)
        return RADIO_UI_STATUS_DISENGAGED;
    if (s->engaging)
        return RADIO_UI_STATUS_ENGAGING;
    if (s->hard_error)
        return RADIO_UI_STATUS_ERROR;
    if (s->link_lost)
        return RADIO_UI_STATUS_LINK_LOST;
    if (!s->active_flow)
        return RADIO_UI_STATUS_STANDBY;
    if (s->consecutive_link_failures >= 2u ||
        s->link_fail_count_in_window >= 3u)
        return RADIO_UI_STATUS_LINK_LOST;
    if (s->degraded || s->total_fail_count_in_window >= 2u)
        return RADIO_UI_STATUS_DEGRADED;
    return RADIO_UI_STATUS_STABLE;
}

void rig_ui_command_window_init(RigUiCommandWindow *window)
{
    if (window == NULL)
        return;

    for (guint i = 0; i < RIG_UI_RELIABILITY_WINDOW_SIZE; i++)
        window->outcomes[i] = RIG_UI_CMD_OK;
    window->count = 0;
    window->next_idx = 0;
    window->last_command_us = 0;
}

void rig_ui_command_window_record(RigUiCommandWindow *window,
                                  RigUiCommandOutcome outcome,
                                  gint64 now_us)
{
    if (window == NULL)
        return;

    window->outcomes[window->next_idx] = outcome;
    window->next_idx = (window->next_idx + 1u) % RIG_UI_RELIABILITY_WINDOW_SIZE;
    if (window->count < RIG_UI_RELIABILITY_WINDOW_SIZE)
        window->count++;
    if (now_us > 0)
        window->last_command_us = now_us;
}

void rig_ui_command_window_get_stats(const RigUiCommandWindow *window,
                                     gint64 now_us,
                                     gint64 active_window_ms,
                                     RigUiCommandWindowStats *stats)
{
    guint consecutive = 0;
    gboolean stop_consecutive = FALSE;

    if (stats == NULL)
        return;

    stats->ok_count = 0;
    stats->reject_count = 0;
    stats->link_fail_count = 0;
    stats->total_fail_count = 0;
    stats->consecutive_link_failures = 0;
    stats->active = FALSE;

    if (window == NULL || window->count == 0)
        return;

    for (guint age = 0; age < window->count; age++)
    {
        guint idx = rig_ui_window_index_from_newest(window, age);
        RigUiCommandOutcome outcome = window->outcomes[idx];

        if (outcome == RIG_UI_CMD_OK)
        {
            stats->ok_count++;
        }
        else if (outcome == RIG_UI_CMD_COMMAND_REJECT)
        {
            stats->reject_count++;
            stats->total_fail_count++;
        }
        else
        {
            stats->link_fail_count++;
            stats->total_fail_count++;
        }

        if (!stop_consecutive)
        {
            if (outcome == RIG_UI_CMD_LINK_FAIL)
                consecutive++;
            else
                stop_consecutive = TRUE;
        }
    }

    stats->consecutive_link_failures = consecutive;

    if (active_window_ms > 0 &&
        window->last_command_us > 0 &&
        now_us >= window->last_command_us)
    {
        gint64 elapsed_us = now_us - window->last_command_us;
        gint64 active_window_us = active_window_ms * 1000;
        if (elapsed_us <= active_window_us)
            stats->active = TRUE;
    }
}
