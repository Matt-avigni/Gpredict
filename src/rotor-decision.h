/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#ifndef ROTOR_DECISION_H
#define ROTOR_DECISION_H

#include <stdint.h>

#include <glib.h>

#include "rotor-target.h"

typedef enum {
    ROT_CMD_MODE_OTHER = 0,
    ROT_CMD_MODE_PRETRACK,
    ROT_CMD_MODE_TRACKING,
    ROT_CMD_MODE_PARK
} rot_cmd_mode_t;

typedef enum {
    ROT_CMD_ACTION_SUPPRESS = 0,
    ROT_CMD_ACTION_SEND
} rot_cmd_action_t;

typedef enum {
    ROT_CMD_REASON_NONE = 0,
    ROT_CMD_REASON_INVALID_INPUT,
    ROT_CMD_REASON_NO_TARGET,
    ROT_CMD_REASON_BLOCKED,
    ROT_CMD_REASON_RANGE,
    ROT_CMD_REASON_DEADBAND,
    ROT_CMD_REASON_MIN_STEP,
    ROT_CMD_REASON_RATE_LIMIT,
    ROT_CMD_REASON_STALE_HOLD,
    ROT_CMD_REASON_NO_POS,
    ROT_CMD_REASON_PARK,
    ROT_CMD_REASON_STALE_RELAXED,
    ROT_CMD_REASON_FORCE,
    ROT_CMD_REASON_IN_FLIGHT,
    ROT_CMD_REASON_RESEND,
    ROT_CMD_REASON_TARGET_CHANGE,
    ROT_CMD_REASON_REASSERT,
    ROT_CMD_REASON_STOPPED,
    ROT_CMD_REASON_TARGET,
    ROT_CMD_REASON_INITIAL
} rot_cmd_reason_t;

typedef struct {
    rot_cmd_mode_t mode;
    gboolean allow_send;
    gboolean have_target;
    gboolean force_send;
    gboolean stale_hold;
    gboolean setpoint_valid;
    gboolean pos_fresh;
    gint64 last_good_age_ms;
    gboolean not_at_target;
    gboolean moving_toward;
    gboolean stopped_unexpected;
    gboolean pos_recovered;
    gboolean resend_due;
    gint64 now_us;
    gint64 last_send_us;
    gint64 min_interval_us;
    gdouble desired_user_az;
    gdouble desired_user_el;
    gdouble desired_backend_az;
    gdouble desired_backend_el;
    gdouble setpoint_user_az;
    gdouble setpoint_user_el;
    gdouble last_cmd_user_az;
    gdouble last_cmd_user_el;
    gdouble delta_backend_az;
    gdouble delta_backend_el;
    gdouble deadband_az;
    gdouble deadband_el;
    gdouble min_step_az;
    gdouble min_step_el;
    gdouble target_change_az;
    gdouble target_change_el;
    rot_target_caps_t caps;
} rot_cmd_decision_input_t;

typedef struct {
    gboolean send;
    rot_cmd_action_t action;
    rot_cmd_reason_t reason;
    rot_target_invalid_reason_t range_reason;
    rot_cmd_mode_t mode;
    gdouble desired_backend_az;
    gdouble desired_backend_el;
    gdouble delta_user_az;
    gdouble delta_user_el;
    gint64 last_good_age_ms;
    gboolean pos_fresh;
} rot_cmd_decision_t;

typedef struct {
    gboolean valid;
    gdouble last_err_az;
    gdouble last_err_el;
    guint stall_count;
} rot_cmd_progress_state_t;

typedef struct {
    gboolean az_active;
    gboolean el_active;
    gboolean az_progressing;
    gboolean el_progressing;
    gboolean moving_toward;
    gboolean stalled;
} rot_cmd_progress_t;

const char *rot_cmd_action_name(rot_cmd_action_t action);
const char *rot_cmd_reason_name(rot_cmd_reason_t reason);

void rot_cmd_decision_eval(const rot_cmd_decision_input_t *in,
                           rot_cmd_decision_t *out);
void rot_cmd_progress_reset(rot_cmd_progress_state_t *state);
void rot_cmd_progress_step(rot_cmd_progress_state_t *state,
                           gdouble err_az,
                           gdouble err_el,
                           gdouble eps_az,
                           gdouble eps_el,
                           gdouble improve_eps,
                           rot_cmd_progress_t *out);

#endif
