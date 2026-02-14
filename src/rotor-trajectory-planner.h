/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#ifndef ROTOR_TRAJECTORY_PLANNER_H
#define ROTOR_TRAJECTORY_PLANNER_H

#include <glib.h>

typedef enum {
    ROT_PLAN_STRATEGY_S1 = 0,
    ROT_PLAN_STRATEGY_S2 = 1
} rot_plan_strategy_t;

typedef enum {
    ROT_PLAN_STATUS_FULL_TRACK = 0,
    ROT_PLAN_STATUS_PARTIAL_CLAMPED = 1,
    ROT_PLAN_STATUS_PARTIAL_SEGMENTED = 2
} rot_plan_status_t;

typedef struct {
    gdouble         t;
    gdouble         az;
    gdouble         el;
} rot_plan_sample_t;

typedef struct {
    gdouble         t;
    gdouble         cmd_az;
    gdouble         cmd_el;
    gboolean        trackable;
} rot_plan_cmd_t;

typedef struct {
    const GArray   *samples;       /* rot_plan_sample_t */
    gdouble         az_min;
    gdouble         az_max;
    gdouble         el_min;
    gdouble         el_max;
    gdouble         az_stop;
    gboolean        use_az_stop;
    gdouble         az_current;
    gdouble         el_current;
    gboolean        have_current;
} rot_plan_input_t;

typedef struct {
    GArray         *cmds;           /* rot_plan_cmd_t */
    rot_plan_strategy_t strategy;
    rot_plan_status_t status;
    gdouble         trackable_pct;
    gdouble         violation_mag;
    gdouble         total_motion;
    gchar          *reason;
} rot_plan_result_t;

gdouble normalize_az_0_360(gdouble az);
gboolean rot_plan_build(const rot_plan_input_t *in, rot_plan_result_t *out);
void rot_plan_result_clear(rot_plan_result_t *res);

/* Optional debug harness, guarded by environment in caller. */
void rot_plan_debug_harness(void);

#endif
