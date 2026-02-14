/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include "rot_math.h"

#include <math.h>

#define ROT_MATH_EPS 1e-9
#define ROT_LANE_CAND_K_MIN (-3)
#define ROT_LANE_CAND_K_MAX (3)
#define ROT_LANE_SEAM_PENALTY 1000000.0
#define ROT_LANE_ENDSTOP_PENALTY 1000.0

static gboolean rot_lane_crosses_seam(gdouble start_num,
                                      gdouble end_num,
                                      gdouble seam360)
{
    gdouble seam = rot_norm360(seam360);
    gdouble lo = MIN(start_num, end_num);
    gdouble hi = MAX(start_num, end_num);

    if (!isfinite(start_num) || !isfinite(end_num))
        return FALSE;

    if (hi - lo < ROT_MATH_EPS)
        return FALSE;

    gint k_min = (gint)ceil((lo - seam) / 360.0 - 1e-9);
    gint k_max = (gint)floor((hi - seam) / 360.0 + 1e-9);
    return (k_min <= k_max);
}

gdouble rot_norm360(gdouble x)
{
    gdouble v = fmod(x, 360.0);
    if (v < 0.0)
        v += 360.0;
    if (v >= 360.0)
        v -= 360.0;
    return v;
}

gdouble rot_norm180(gdouble x)
{
    gdouble v = rot_norm360(x);
    if (v > 180.0)
        v -= 360.0;
    return v;
}

gdouble rot_delta_shortest_360(gdouble from360, gdouble to360)
{
    gdouble a = rot_norm360(from360);
    gdouble b = rot_norm360(to360);
    gdouble d = b - a;
    if (d > 180.0)
        d -= 360.0;
    else if (d < -180.0)
        d += 360.0;
    return d;
}

gdouble rot_ang_diff_deg(gdouble a, gdouble b)
{
    return rot_norm180(a - b);
}

gdouble rot_ang_dist_deg(gdouble a, gdouble b)
{
    return fabs(rot_ang_diff_deg(a, b));
}

gdouble rot_ui_to_az360(gdouble ui_az, rot_ui_mode_t mode)
{
    (void)mode;
    return rot_norm360(ui_az);
}

gdouble rot_az360_to_ui(gdouble az360, rot_ui_mode_t mode)
{
    if (mode == ROT_UI_NORTH_CENTERED)
        return rot_norm180(az360);
    return rot_norm360(az360);
}

gdouble rot_backend_pos_to_az360(gdouble az_backend)
{
    return rot_norm360(az_backend);
}

gboolean rot_backend_lane_select(gdouble target_az360,
                                 gdouble backend_min,
                                 gdouble backend_max,
                                 gdouble last_cmd_backend,
                                 gboolean have_last_cmd,
                                 gdouble seam360,
                                 gboolean seam_valid,
                                 gdouble endstop_margin_deg,
                                 RotLaneSelectResult *out)
{
    gdouble az_norm = rot_norm360(target_az360);
    gdouble best = az_norm;
    gdouble best_score = 0.0;
    gboolean best_cross = FALSE;
    gboolean best_used_margin = FALSE;
    gboolean best_clamped = FALSE;
    gboolean have_candidate = FALSE;
    gdouble margin = MAX(0.0, endstop_margin_deg);

    if (out)
        *out = (RotLaneSelectResult){ 0 };

    if (!out)
        return FALSE;

    if (!isfinite(backend_min) || !isfinite(backend_max))
    {
        out->az_cmd_backend = az_norm;
        return TRUE;
    }

    if (backend_min > backend_max)
    {
        gdouble tmp = backend_min;
        backend_min = backend_max;
        backend_max = tmp;
    }

    gdouble cand_list[ROT_LANE_CAND_K_MAX - ROT_LANE_CAND_K_MIN + 1];
    gboolean cand_ok[ROT_LANE_CAND_K_MAX - ROT_LANE_CAND_K_MIN + 1];
    gboolean cand_pref[ROT_LANE_CAND_K_MAX - ROT_LANE_CAND_K_MIN + 1];
    gint cand_count = 0;

    for (gint k = ROT_LANE_CAND_K_MIN; k <= ROT_LANE_CAND_K_MAX; k++)
    {
        gdouble cand = az_norm + (360.0 * k);
        gboolean in_limits = (cand >= backend_min - ROT_MATH_EPS &&
                              cand <= backend_max + ROT_MATH_EPS);
        cand_list[cand_count] = cand;
        cand_ok[cand_count] = in_limits;
        cand_pref[cand_count] = in_limits &&
                                (cand >= backend_min + margin - ROT_MATH_EPS) &&
                                (cand <= backend_max - margin + ROT_MATH_EPS);
        cand_count++;
    }

    gboolean have_pref = FALSE;
    for (gint i = 0; i < cand_count; i++)
    {
        if (cand_pref[i])
        {
            have_pref = TRUE;
            break;
        }
    }

    for (gint i = 0; i < cand_count; i++)
    {
        gdouble cand = cand_list[i];
        gboolean ok = cand_ok[i];
        gboolean preferred = cand_pref[i];
        gdouble score = 0.0;
        gboolean crosses = FALSE;
        gdouble dist_end = 0.0;

        if (!ok)
            continue;
        if (have_pref && !preferred)
            continue;

        if (have_last_cmd)
            score += fabs(cand - last_cmd_backend);

        if (seam_valid && have_last_cmd &&
            rot_lane_crosses_seam(last_cmd_backend, cand, seam360))
        {
            score += ROT_LANE_SEAM_PENALTY;
            crosses = TRUE;
        }

        dist_end = MIN(cand - backend_min, backend_max - cand);
        if (dist_end < margin)
            score += (margin - dist_end) * ROT_LANE_ENDSTOP_PENALTY;

        if (!have_candidate || score < best_score)
        {
            best = cand;
            best_score = score;
            best_cross = crosses;
            best_used_margin = have_pref;
            have_candidate = TRUE;
        }
    }

    if (!have_candidate)
    {
        /* No in-range candidate: clamp to nearest endpoint. */
        gdouble cand = az_norm;
        if (cand < backend_min)
            cand = backend_min;
        else if (cand > backend_max)
            cand = backend_max;
        best = cand;
        best_clamped = TRUE;
        best_score = have_last_cmd ? fabs(cand - last_cmd_backend) : 0.0;
    }

    out->az_cmd_backend = best;
    out->clamped = best_clamped;
    out->crosses_seam = best_cross;
    out->used_margin = best_used_margin;
    out->score = best_score;

    return have_candidate || best_clamped;
}
