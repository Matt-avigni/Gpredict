#include "rotor-autocal-direct.h"

#include <math.h>

#include "rot_math.h"

void rot_autocal_direct_reset(RotAutocalDirectState *state,
                              gint64 now_us,
                              gboolean pos_valid,
                              gdouble pos_az,
                              gdouble pos_el)
{
    if (state == NULL)
        return;

    state->have_last_pos = pos_valid;
    state->last_pos_az = pos_az;
    state->last_pos_el = pos_el;
    state->last_pos_change_us = now_us;
    state->last_move_us = 0;
}

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
                             RotAutocalDirectResult *out)
{
    RotAutocalDirectResult local = { 0 };

    if (state == NULL)
    {
        if (out)
            *out = local;
        return;
    }

    local.err_az = fabs(rot_delta_shortest_360(pos_az, target_az));
    local.err_el = fabs(pos_el - target_el);

    if (pos_valid)
    {
        if (!state->have_last_pos ||
            fabs(pos_az - state->last_pos_az) >= motion_eps ||
            fabs(pos_el - state->last_pos_el) >= motion_eps)
        {
            state->last_pos_change_us = now_us;
        }
        state->have_last_pos = TRUE;
        state->last_pos_az = pos_az;
        state->last_pos_el = pos_el;
    }

    if (no_pos_us > 0)
    {
        if (!pos_valid || last_pos_us <= 0 ||
            (now_us > last_pos_us &&
             (now_us - last_pos_us) >= no_pos_us))
            local.no_position = TRUE;
    }

    if (pos_valid &&
        state->last_move_us > 0 &&
        no_motion_us > 0 &&
        (now_us - state->last_move_us) >= no_motion_us &&
        (now_us - state->last_pos_change_us) >= no_motion_us &&
        (local.err_az > arrive_tol_az || local.err_el > arrive_tol_el))
    {
        local.no_motion = TRUE;
    }

    if (!local.no_position && !local.no_motion)
    {
        gboolean moved_since_move =
            (state->last_move_us == 0) ||
            (state->last_pos_change_us >= state->last_move_us);

        if ((local.err_az > arrive_tol_az || local.err_el > arrive_tol_el) &&
            moved_since_move &&
            (state->last_move_us == 0 ||
             (now_us - state->last_move_us) >= move_interval_us))
        {
            local.issue_move = TRUE;
            state->last_move_us = now_us;
        }
    }

    if (out)
        *out = local;
}
