/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include "rotor-decision.h"

#include <math.h>

#include "rot-angle.h"
#include "rotor-target.h"

static gdouble rot_decision_az_delta(const rot_target_caps_t *caps,
                                    gdouble a,
                                    gdouble b)
{
    if (caps && caps->az_wrap_mode == ROT_TARGET_WRAP_180)
        return rot_target_az_distance_wrap180(a, b);
    return rot_target_az_distance_wrap360(a, b);
}

const char *rot_cmd_action_name(rot_cmd_action_t action)
{
    switch (action)
    {
    case ROT_CMD_ACTION_SEND:
        return "SEND";
    case ROT_CMD_ACTION_SUPPRESS:
    default:
        return "SUPPRESS";
    }
}

const char *rot_cmd_reason_name(rot_cmd_reason_t reason)
{
    switch (reason)
    {
    case ROT_CMD_REASON_INVALID_INPUT:
        return "invalid_input";
    case ROT_CMD_REASON_NO_TARGET:
        return "no_target";
    case ROT_CMD_REASON_BLOCKED:
        return "blocked";
    case ROT_CMD_REASON_RANGE:
        return "range";
    case ROT_CMD_REASON_DEADBAND:
        return "deadband";
    case ROT_CMD_REASON_MIN_STEP:
        return "min_step";
    case ROT_CMD_REASON_RATE_LIMIT:
        return "rate_limit";
    case ROT_CMD_REASON_STALE_HOLD:
        return "stale_hold";
    case ROT_CMD_REASON_NO_POS:
        return "no_pos";
    case ROT_CMD_REASON_PARK:
        return "park";
    case ROT_CMD_REASON_STALE_RELAXED:
        return "stale_relaxed";
    case ROT_CMD_REASON_FORCE:
        return "force";
    case ROT_CMD_REASON_IN_FLIGHT:
        return "in_flight";
    case ROT_CMD_REASON_RESEND:
        return "resend";
    case ROT_CMD_REASON_TARGET_CHANGE:
        return "target_change";
    case ROT_CMD_REASON_REASSERT:
        return "reassert";
    case ROT_CMD_REASON_STOPPED:
        return "stopped";
    case ROT_CMD_REASON_TARGET:
        return "target";
    case ROT_CMD_REASON_INITIAL:
        return "initial";
    case ROT_CMD_REASON_NONE:
    default:
        return "none";
    }
}

void rot_cmd_decision_eval(const rot_cmd_decision_input_t *in,
                           rot_cmd_decision_t *out)
{
    gdouble delta_az = 0.0;
    gdouble delta_el = 0.0;

    if (out == NULL)
        return;

    out->send = FALSE;
    out->action = ROT_CMD_ACTION_SUPPRESS;
    out->reason = ROT_CMD_REASON_DEADBAND;
    out->range_reason = ROT_TARGET_INVALID_NONE;
    out->mode = ROT_CMD_MODE_OTHER;
    out->desired_backend_az = 0.0;
    out->desired_backend_el = 0.0;
    out->delta_user_az = 0.0;
    out->delta_user_el = 0.0;
    out->last_good_age_ms = -1;
    out->pos_fresh = FALSE;

    if (in == NULL)
    {
        out->reason = ROT_CMD_REASON_INVALID_INPUT;
        return;
    }

    out->mode = in->mode;
    out->desired_backend_az = in->desired_backend_az;
    out->desired_backend_el = in->desired_backend_el;
    out->last_good_age_ms = in->last_good_age_ms;
    out->pos_fresh = in->pos_fresh;

    if (!in->have_target)
    {
        out->reason = ROT_CMD_REASON_NO_TARGET;
        return;
    }

    if (!in->allow_send)
    {
        out->reason = ROT_CMD_REASON_BLOCKED;
        return;
    }

    {
        gdouble az_norm = 0.0;
        rot_target_invalid_reason_t reason = ROT_TARGET_INVALID_NONE;
        gboolean ok = rot_target_is_valid(&in->caps,
                                          in->desired_user_az,
                                          in->desired_user_el,
                                          &az_norm,
                                          &reason);
        if (!ok)
        {
            out->reason = ROT_CMD_REASON_RANGE;
            out->range_reason = reason;
            return;
        }
    }

    if (in->mode == ROT_CMD_MODE_PARK)
    {
        out->send = TRUE;
        out->action = ROT_CMD_ACTION_SEND;
        out->reason = ROT_CMD_REASON_PARK;
        return;
    }

    if (!in->pos_fresh && in->mode != ROT_CMD_MODE_PARK)
    {
        out->reason = ROT_CMD_REASON_NO_POS;
        return;
    }

    if (in->setpoint_valid)
    {
        delta_az = rot_decision_az_delta(&in->caps,
                                         in->desired_user_az,
                                         in->setpoint_user_az);
        delta_el = fabs(in->desired_user_el - in->setpoint_user_el);
        out->delta_user_az = delta_az;
        out->delta_user_el = delta_el;
    }

    {
        gdouble db_az = (in->deadband_az > 0.0) ? in->deadband_az : 0.0;
        gdouble db_el = (in->deadband_el > 0.0) ? in->deadband_el : 0.0;

        if (!in->force_send && in->setpoint_valid &&
            delta_az < db_az &&
            delta_el < db_el &&
            in->delta_backend_az < db_az)
        {
            out->reason = ROT_CMD_REASON_DEADBAND;
            return;
        }
    }

    if (in->stale_hold)
    {
        if (in->setpoint_valid)
        {
            gdouble change_az = (in->target_change_az > 0.0)
                                    ? in->target_change_az
                                    : in->deadband_az;
            gdouble change_el = (in->target_change_el > 0.0)
                                    ? in->target_change_el
                                    : in->deadband_el;

            if (in->delta_backend_az >= change_az ||
                delta_az >= change_az ||
                delta_el >= change_el)
            {
                out->send = TRUE;
                out->action = ROT_CMD_ACTION_SEND;
                out->reason = ROT_CMD_REASON_TARGET_CHANGE;
                return;
            }
        }
        out->reason = ROT_CMD_REASON_STALE_HOLD;
        return;
    }

    if (in->force_send && in->setpoint_valid)
    {
        gdouble db_az = (in->deadband_az > 0.0) ? in->deadband_az : 0.0;
        gdouble db_el = (in->deadband_el > 0.0) ? in->deadband_el : 0.0;

        if (delta_az <= db_az && delta_el <= db_el)
        {
            out->reason = ROT_CMD_REASON_DEADBAND;
            return;
        }
    }

    if (in->force_send)
    {
        out->send = TRUE;
        out->action = ROT_CMD_ACTION_SEND;
        out->reason = ROT_CMD_REASON_FORCE;
        return;
    }

    if (!in->pos_fresh && in->setpoint_valid)
    {
        gdouble stale_relaxed_az = (in->deadband_az > 0.0)
                                       ? (0.5 * in->deadband_az)
                                       : 0.0;
        gdouble stale_relaxed_el = (in->deadband_el > 0.0)
                                       ? (0.5 * in->deadband_el)
                                       : 0.0;

        if (delta_az >= stale_relaxed_az || delta_el >= stale_relaxed_el)
        {
            out->send = TRUE;
            out->action = ROT_CMD_ACTION_SEND;
            out->reason = ROT_CMD_REASON_STALE_RELAXED;
            return;
        }

        if (in->resend_due)
        {
            out->send = TRUE;
            out->action = ROT_CMD_ACTION_SEND;
            out->reason = ROT_CMD_REASON_RESEND;
            return;
        }
        out->reason = ROT_CMD_REASON_NO_POS;
        return;
    }

    if (!in->force_send && in->setpoint_valid &&
        in->min_step_az > 0.0 && in->min_step_el > 0.0 &&
        in->delta_backend_az < in->min_step_az &&
        in->delta_backend_el < in->min_step_el)
    {
        out->reason = ROT_CMD_REASON_MIN_STEP;
        return;
    }

    if (!in->force_send && in->min_interval_us > 0 &&
        in->last_send_us > 0 && in->now_us > in->last_send_us &&
        (in->now_us - in->last_send_us) < in->min_interval_us)
    {
        out->reason = ROT_CMD_REASON_RATE_LIMIT;
        return;
    }

    if (in->setpoint_valid && in->not_at_target && !in->stopped_unexpected)
    {
        if (in->resend_due)
        {
            out->send = TRUE;
            out->action = ROT_CMD_ACTION_SEND;
            out->reason = ROT_CMD_REASON_RESEND;
            return;
        }
        if (delta_az >= in->target_change_az ||
            delta_el >= in->target_change_el)
        {
            out->send = TRUE;
            out->action = ROT_CMD_ACTION_SEND;
            out->reason = ROT_CMD_REASON_TARGET_CHANGE;
            return;
        }
        if (in->pos_recovered)
        {
            out->send = TRUE;
            out->action = ROT_CMD_ACTION_SEND;
            out->reason = ROT_CMD_REASON_REASSERT;
            return;
        }

        out->reason = ROT_CMD_REASON_IN_FLIGHT;
        return;
    }

    if (in->stopped_unexpected)
    {
        out->send = TRUE;
        out->action = ROT_CMD_ACTION_SEND;
        out->reason = ROT_CMD_REASON_STOPPED;
        return;
    }

    if (in->resend_due)
    {
        out->send = TRUE;
        out->action = ROT_CMD_ACTION_SEND;
        out->reason = ROT_CMD_REASON_RESEND;
        return;
    }

    out->send = TRUE;
    out->action = ROT_CMD_ACTION_SEND;
    out->reason = in->setpoint_valid ? ROT_CMD_REASON_TARGET
                                     : ROT_CMD_REASON_INITIAL;
}

void rot_cmd_progress_reset(rot_cmd_progress_state_t *state)
{
    if (state == NULL)
        return;

    state->valid = FALSE;
    state->last_err_az = 0.0;
    state->last_err_el = 0.0;
    state->stall_count = 0;
}

void rot_cmd_progress_step(rot_cmd_progress_state_t *state,
                           gdouble err_az,
                           gdouble err_el,
                           gdouble eps_az,
                           gdouble eps_el,
                           gdouble improve_eps,
                           rot_cmd_progress_t *out)
{
    gboolean az_active = FALSE;
    gboolean el_active = FALSE;
    gboolean az_progressing = FALSE;
    gboolean el_progressing = FALSE;

    if (out != NULL)
        memset(out, 0, sizeof(*out));

    if (state == NULL)
        return;

    if (!isfinite(err_az) || !isfinite(err_el) ||
        !isfinite(eps_az) || !isfinite(eps_el))
    {
        rot_cmd_progress_reset(state);
        return;
    }

    if (eps_az < 0.0)
        eps_az = 0.0;
    if (eps_el < 0.0)
        eps_el = 0.0;
    if (improve_eps < 0.0)
        improve_eps = 0.0;

    az_active = (err_az > eps_az);
    el_active = (err_el > eps_el);

    if (out != NULL)
    {
        out->az_active = az_active;
        out->el_active = el_active;
    }

    if (!az_active && !el_active)
    {
        rot_cmd_progress_reset(state);
        return;
    }

    if (!state->valid)
    {
        state->valid = TRUE;
        state->last_err_az = err_az;
        state->last_err_el = err_el;
        state->stall_count = 0;
        return;
    }

    az_progressing = !az_active ||
                     (err_az <= (state->last_err_az - improve_eps));
    el_progressing = !el_active ||
                     (err_el <= (state->last_err_el - improve_eps));

    if (az_progressing && el_progressing)
        state->stall_count = 0;
    else if (state->stall_count < G_MAXUINT)
        state->stall_count++;

    state->last_err_az = err_az;
    state->last_err_el = err_el;

    if (out != NULL)
    {
        out->az_progressing = az_progressing;
        out->el_progressing = el_progressing;
        out->moving_toward =
            (az_active && az_progressing) ||
            (el_active && el_progressing);
        out->stalled = (state->stall_count >= 2);
    }
}
