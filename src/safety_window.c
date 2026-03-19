/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include "safety_window.h"

#include <math.h>

enum {
    SAFETY_AZ_PERIOD_DEG = 360
};

static gboolean safety_path_crosses_band(double cur,
                                         double cand,
                                         double band_min,
                                         double band_max)
{
    double lo = MIN(cur, cand);
    double hi = MAX(cur, cand);

    return !(hi < band_min || lo > band_max);
}

static gboolean safety_pick_equivalent_in_window(double cand,
                                                 double ref,
                                                 double win_min,
                                                 double win_max,
                                                 double *cand_out)
{
    double best = 0.0;
    double best_dist = 0.0;
    gboolean found = FALSE;

    if (cand_out == NULL || win_max < win_min)
        return FALSE;

    for (int k = -4; k <= 4; k++) {
        double alias = cand + (SAFETY_AZ_PERIOD_DEG * (double)k);
        double dist = fabs(alias - ref);

        if (alias < win_min || alias > win_max)
            continue;

        if (!found || dist < best_dist) {
            best = alias;
            best_dist = dist;
            found = TRUE;
        }
    }

    if (found)
        *cand_out = best;

    return found;
}

static gboolean safety_pick_equivalent_avoiding_band(double cand,
                                                     double ref,
                                                     double win_min,
                                                     double win_max,
                                                     double band_min,
                                                     double band_max,
                                                     double *cand_out)
{
    double best = 0.0;
    double best_dist = 0.0;
    gboolean found = FALSE;

    if (cand_out == NULL || win_max < win_min)
        return FALSE;

    for (int k = -4; k <= 4; k++) {
        double alias = cand + (SAFETY_AZ_PERIOD_DEG * (double)k);
        double dist = fabs(alias - ref);

        if (alias < win_min || alias > win_max)
            continue;
        if (safety_path_crosses_band(ref, alias, band_min, band_max))
            continue;

        if (!found || dist < best_dist) {
            best = alias;
            best_dist = dist;
            found = TRUE;
        }
    }

    if (found)
        *cand_out = best;

    return found;
}

static double safety_clamp_to_window(double cand,
                                     double win_min,
                                     double win_max)
{
    if (cand < win_min)
        return (fabs(cand - win_min) <= fabs(cand - win_max)) ? win_min : win_max;
    if (cand > win_max)
        return (fabs(cand - win_min) <= fabs(cand - win_max)) ? win_min : win_max;
    return cand;
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

    if (cand < win_min || cand > win_max) {
        double projected = cand;

        if (safety_pick_equivalent_in_window(cand, az_abs_cur,
                                             win_min, win_max,
                                             &projected)) {
            cand = projected;
        } else {
            cand = safety_clamp_to_window(cand, win_min, win_max);
        }
        clamped = TRUE;
    }

    if (s->stop_enabled && s->stop_margin_deg > 0.0) {
        double band_min = s->stop_abs - s->stop_margin_deg;
        double band_max = s->stop_abs + s->stop_margin_deg;
        gboolean crosses = safety_path_crosses_band(az_abs_cur, cand,
                                                    band_min, band_max);

        if (crosses && span > 0.0) {
            double alt = cand;

            if (safety_pick_equivalent_avoiding_band(cand,
                                                     az_abs_cur,
                                                     win_min, win_max,
                                                     band_min, band_max,
                                                     &alt)) {
                cand = alt;
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
