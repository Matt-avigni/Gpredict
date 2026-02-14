/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#ifndef ROTOR_AUTOCAL_DIRECT_H
#define ROTOR_AUTOCAL_DIRECT_H

#include <glib.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    gboolean have_last_pos;
    gdouble  last_pos_az;
    gdouble  last_pos_el;
    gint64   last_pos_change_us;
    gint64   last_move_us;
} RotAutocalDirectState;

typedef struct {
    gboolean issue_move;
    gboolean no_motion;
    gboolean no_position;
    gdouble  err_az;
    gdouble  err_el;
} RotAutocalDirectResult;

void rot_autocal_direct_reset(RotAutocalDirectState *state,
                              gint64 now_us,
                              gboolean pos_valid,
                              gdouble pos_az,
                              gdouble pos_el);

void rot_autocal_direct_step(RotAutocalDirectState *state,
                             gint64 now_us,
                             gboolean pos_valid,
                             gint64 last_pos_us,
                             gdouble pos_az,
                             gdouble pos_el,
                             gdouble target_az,
                             gdouble target_el,
                             gdouble arrive_tol_az,
                             gdouble arrive_tol_el,
                             gdouble motion_eps,
                             gint64 move_interval_us,
                             gint64 no_motion_us,
                             gint64 no_pos_us,
                             RotAutocalDirectResult *out);

#ifdef __cplusplus
}
#endif

#endif
