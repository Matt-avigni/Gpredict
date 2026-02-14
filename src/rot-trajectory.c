/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include "rot-trajectory.h"

#include <math.h>

#include "sat-log.h"

static gdouble rot_traj_adjust_into_range(gdouble cand,
                                          gdouble min_az,
                                          gdouble max_az)
{
    if (min_az <= max_az)
    {
        if (cand < min_az)
        {
            gdouble k = ceil((min_az - cand) / 360.0);
            cand += 360.0 * k;
        }
        else if (cand > max_az)
        {
            gdouble k = ceil((cand - max_az) / 360.0);
            cand -= 360.0 * k;
        }

        return rot_traj_clamp(cand, min_az, max_az);
    }

    /* Wrap-around range (e.g. min=300, max=60). */
    gdouble cand_norm = rot_traj_norm_360(cand);
    if (cand_norm >= min_az || cand_norm <= max_az)
        return cand;

    gdouble dmin = fabs(rot_traj_wrap_diff(cand_norm, min_az));
    gdouble dmax = fabs(rot_traj_wrap_diff(cand_norm, max_az));
    gdouble boundary = (dmin <= dmax) ? min_az : max_az;
    gdouble k = round((cand - boundary) / 360.0);
    return boundary + 360.0 * k;
}

gdouble rot_traj_select_continuous_az(gdouble target_norm,
                                      gdouble last_cmd_cont,
                                      gboolean have_last,
                                      gdouble min_az,
                                      gdouble max_az)
{
    gdouble target = rot_traj_norm_360(target_norm);

    if (!have_last)
        return rot_traj_adjust_into_range(target, min_az, max_az);

    gdouble k0 = round((last_cmd_cont - target) / 360.0);
    gdouble best = target;
    gdouble best_dist = G_MAXDOUBLE;

    for (int i = -1; i <= 1; i++)
    {
        gdouble cand = target + 360.0 * (k0 + i);
        cand = rot_traj_adjust_into_range(cand, min_az, max_az);

        gdouble dist = fabs(cand - last_cmd_cont);
        if (dist < best_dist)
        {
            best = cand;
            best_dist = dist;
        }
    }

    return best;
}

static void rot_traj_log_check(const gchar *label,
                               gdouble got,
                               gdouble expected,
                               gdouble tol,
                               gint *failures)
{
    gboolean ok = fabs(got - expected) <= tol;

    sat_log_log(ok ? SAT_LOG_LEVEL_INFO : SAT_LOG_LEVEL_ERROR,
                "ROT_TRAJ_TEST %s got=%.3f expected=%.3f",
                label ? label : "(unnamed)", got, expected);

    if (!ok && failures)
        (*failures)++;
}

void rot_traj_debug_harness(void)
{
    static gboolean ran = FALSE;
    gint failures = 0;

    if (ran)
        return;

    const gchar *env = g_getenv("GPREDICT_ROT_TRAJ_TEST");
    if (env == NULL || *env == '\0')
        return;

    ran = TRUE;

    rot_traj_log_check("norm_360(360)->0",
                       rot_traj_norm_360(360.0), 0.0, 1e-6, &failures);
    rot_traj_log_check("norm_360(-1)->359",
                       rot_traj_norm_360(-1.0), 359.0, 1e-6, &failures);
    rot_traj_log_check("norm_360(721)->1",
                       rot_traj_norm_360(721.0), 1.0, 1e-6, &failures);
    rot_traj_log_check("wrap_diff(1,359)->2",
                       rot_traj_wrap_diff(1.0, 359.0), 2.0, 1e-6, &failures);
    rot_traj_log_check("wrap_diff(359,1)->-2",
                       rot_traj_wrap_diff(359.0, 1.0), -2.0, 1e-6, &failures);

    gdouble cont = rot_traj_select_continuous_az(1.0, 359.0, TRUE, 0.0, 450.0);
    rot_traj_log_check("cont last=359 target=1 max=450->361",
                       cont, 361.0, 1e-6, &failures);
    cont = rot_traj_select_continuous_az(1.0, 359.0, TRUE, 0.0, 360.0);
    rot_traj_log_check("cont last=359 target=1 max=360->1",
                       cont, 1.0, 1e-6, &failures);

    sat_log_log(failures == 0 ? SAT_LOG_LEVEL_INFO : SAT_LOG_LEVEL_WARN,
                "ROT_TRAJ_TEST completed failures=%d", failures);
}
