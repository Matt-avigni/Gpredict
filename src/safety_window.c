/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include "safety_window.h"

#include <math.h>

static gboolean safety_path_crosses_band(double cur,
                                         double cand,
                                         double band_min,
                                         double band_max)
{
    double lo = MIN(cur, cand);
    double hi = MAX(cur, cand);

    return !(hi < band_min || lo > band_max);
}

gboolean safety_project_command(const RotorSafety *s,
                                double az_abs_cur,
                                double *az_abs_candidate_in_out)
{
    double cand;
    double win_min;
    double win_max;
    double span;
    gboolean clamped = FALSE;

    if (az_abs_candidate_in_out == NULL)
        return FALSE;

    cand = *az_abs_candidate_in_out;

    if (s == NULL || !s->enabled)
        return TRUE;

    win_min = s->win_min_abs;
    win_max = s->win_max_abs;
    if (win_max < win_min) {
        double tmp = win_min;
        win_min = win_max;
        win_max = tmp;
    }

    span = win_max - win_min;

    if (span > 0.0 && (cand < win_min || cand > win_max)) {
        double offset = fmod(cand - win_min, span);
        if (offset < 0.0)
            offset += span;
        cand = win_min + offset;
    }

    if (cand < win_min) {
        cand = win_min;
        clamped = TRUE;
    } else if (cand > win_max) {
        cand = win_max;
        clamped = TRUE;
    }

    if (s->stop_enabled && s->stop_margin_deg > 0.0) {
        double band_min = s->stop_abs - s->stop_margin_deg;
        double band_max = s->stop_abs + s->stop_margin_deg;
        gboolean crosses = safety_path_crosses_band(az_abs_cur, cand,
                                                    band_min, band_max);

        if (crosses && span > 0.0) {
            double alt1 = cand - span;
            double alt2 = cand + span;
            gboolean alt1_ok = (alt1 >= win_min && alt1 <= win_max) &&
                               !safety_path_crosses_band(az_abs_cur, alt1,
                                                         band_min, band_max);
            gboolean alt2_ok = (alt2 >= win_min && alt2 <= win_max) &&
                               !safety_path_crosses_band(az_abs_cur, alt2,
                                                         band_min, band_max);

            if (alt1_ok || alt2_ok) {
                cand = alt1_ok ? alt1 : alt2;
                crosses = FALSE;
            }
        }

        if (crosses) {
            double left_bound = MAX(win_min, band_min);
            double right_bound = MIN(win_max, band_max);

            if (az_abs_cur < band_min) {
                cand = MIN(cand, left_bound);
            } else if (az_abs_cur > band_max) {
                cand = MAX(cand, right_bound);
            } else {
                if (fabs(cand - left_bound) <= fabs(cand - right_bound))
                    cand = left_bound;
                else
                    cand = right_bound;
            }
            clamped = TRUE;
        }
    }

    if (cand < win_min)
        cand = win_min;
    if (cand > win_max)
        cand = win_max;

    *az_abs_candidate_in_out = cand;
    return !clamped;
}
