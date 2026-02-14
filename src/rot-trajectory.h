/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#ifndef ROT_TRAJECTORY_H
#define ROT_TRAJECTORY_H

#include <glib.h>
#include <math.h>

static inline gdouble rot_traj_norm_360(gdouble az)
{
    gdouble v = fmod(az, 360.0);
    if (v < 0.0)
        v += 360.0;
    if (fabs(v - 360.0) < 1e-6)
        v = 0.0;
    return v;
}

static inline gdouble rot_traj_wrap_diff(gdouble a, gdouble b)
{
    gdouble delta = rot_traj_norm_360(a) - rot_traj_norm_360(b);

    if (delta > 180.0)
        delta -= 360.0;
    if (delta <= -180.0)
        delta += 360.0;
    return delta;
}

static inline gdouble rot_traj_clamp(gdouble x, gdouble lo, gdouble hi)
{
    if (x < lo)
        return lo;
    if (x > hi)
        return hi;
    return x;
}

gdouble rot_traj_select_continuous_az(gdouble target_norm,
                                      gdouble last_cmd_cont,
                                      gboolean have_last,
                                      gdouble min_az,
                                      gdouble max_az);

void rot_traj_debug_harness(void);

#endif
