/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include <assert.h>

#include "rotor-autocal-direct.h"

static void run_motion_success(void)
{
    RotAutocalDirectState st = { 0 };
    gint64 now_us = 0;
    gint64 last_pos_us = 0;
    gdouble az = 114.0;
    gdouble el = 176.0;
    gboolean issued = FALSE;

    rot_autocal_direct_reset(&st, now_us, TRUE, az, el);
    last_pos_us = now_us;

    for (int i = 0; i < 20; i++)
    {
        RotAutocalDirectResult out = { 0 };

        rot_autocal_direct_step(&st,
                                now_us,
                                TRUE,
                                last_pos_us,
                                az,
                                el,
                                0.0,
                                0.0,
                                2.0,
                                2.0,
                                0.5,
                                500000,
                                4000000,
                                4000000,
                                &out);

        if (out.issue_move)
            issued = TRUE;

        /* Simulate motion towards zero. */
        az -= 20.0;
        el -= 30.0;
        if (az < 0.0)
            az = 0.0;
        if (el < 0.0)
            el = 0.0;

        now_us += 200000;
        last_pos_us = now_us;
    }

    assert(issued);

    {
        RotAutocalDirectResult out = { 0 };
        rot_autocal_direct_step(&st,
                                now_us,
                                TRUE,
                                last_pos_us,
                                az,
                                el,
                                0.0,
                                0.0,
                                2.0,
                                2.0,
                                0.5,
                                500000,
                                4000000,
                                4000000,
                                &out);
        assert(out.err_az <= 2.0);
        assert(out.err_el <= 2.0);
    }
}

static void run_no_motion_abort(void)
{
    RotAutocalDirectState st = { 0 };
    gint64 now_us = 0;
    gint64 last_pos_us = 0;
    gdouble az = 114.0;
    gdouble el = 176.0;
    gboolean saw_no_motion = FALSE;

    rot_autocal_direct_reset(&st, now_us, TRUE, az, el);
    last_pos_us = now_us;

    for (int i = 0; i < 30; i++)
    {
        RotAutocalDirectResult out = { 0 };
        rot_autocal_direct_step(&st,
                                now_us,
                                TRUE,
                                last_pos_us,
                                az,
                                el,
                                0.0,
                                0.0,
                                2.0,
                                2.0,
                                0.5,
                                500000,
                                4000000,
                                4000000,
                                &out);
        if (out.no_motion)
        {
            saw_no_motion = TRUE;
            break;
        }

        now_us += 200000;
        last_pos_us = now_us;
    }

    assert(saw_no_motion);
}

static void run_move_interval_guard(void)
{
    RotAutocalDirectState st = { 0 };
    gint64 now_us = 0;
    gint64 last_pos_us = 0;
    gdouble az = 90.0;
    gdouble el = 45.0;
    guint moves = 0;

    rot_autocal_direct_reset(&st, now_us, TRUE, az, el);
    last_pos_us = now_us;

    for (int i = 0; i < 6; i++)
    {
        RotAutocalDirectResult out = { 0 };

        rot_autocal_direct_step(&st,
                                now_us,
                                TRUE,
                                last_pos_us,
                                az,
                                el,
                                0.0,
                                0.0,
                                2.0,
                                2.0,
                                0.1,
                                500000,
                                0,
                                0,
                                &out);
        if (out.issue_move)
            moves++;

        now_us += 100000;
        last_pos_us = now_us;
    }

    /* Expect no more than two moves within 600ms when interval is 500ms. */
    assert(moves <= 2);
    assert(moves >= 1);
}

static void run_no_position_flag(void)
{
    RotAutocalDirectState st = { 0 };
    RotAutocalDirectResult out = { 0 };

    rot_autocal_direct_reset(&st, 0, FALSE, 0.0, 0.0);

    rot_autocal_direct_step(&st,
                            0,
                            FALSE,
                            0,
                            0.0,
                            0.0,
                            0.0,
                            0.0,
                            2.0,
                            2.0,
                            0.1,
                            500000,
                            0,
                            1000000,
                            &out);
    assert(out.no_position);
}

int main(void)
{
    run_motion_success();
    run_no_motion_abort();
    run_move_interval_guard();
    run_no_position_flag();
    return 0;
}
