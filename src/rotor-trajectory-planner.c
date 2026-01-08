/*
  Gpredict: Real-time satellite tracking and orbit prediction program

  This file implements a best-effort rotor trajectory planner. It
  evaluates multiple wrap strategies and always returns a safe plan,
  falling back to clamping or segmented tracking when needed.
*/
#include "rotor-trajectory-planner.h"

#include <math.h>

static gdouble angular_distance(gdouble a, gdouble b)
{
    gdouble na = normalize_az_0_360(a);
    gdouble nb = normalize_az_0_360(b);
    gdouble diff = fabs(na - nb);
    if (diff > 180.0)
        diff = 360.0 - diff;
    return diff;
}

gdouble normalize_az_0_360(gdouble az)
{
    gdouble v = fmod(az, 360.0);
    if (v < 0.0)
        v += 360.0;
    if (v >= 360.0)
        v -= 360.0;
    return v;
}

static gboolean az_in_range(gdouble az, gdouble min, gdouble max)
{
    if (min <= max)
        return (az >= min && az <= max);
    return (az >= min || az <= max);
}

static gdouble az_clamp(gdouble az, gdouble min, gdouble max)
{
    if (az_in_range(az, min, max))
        return az;

    /* Choose the nearest boundary on the circle. */
    gdouble dmin = angular_distance(az, min);
    gdouble dmax = angular_distance(az, max);
    return (dmin <= dmax) ? min : max;
}

static gdouble az_violation_mag(gdouble az, gdouble min, gdouble max)
{
    if (az_in_range(az, min, max))
        return 0.0;
    return MIN(angular_distance(az, min), angular_distance(az, max));
}

static gdouble unwrap_select(gdouble prev, gdouble az_norm,
                             rot_plan_strategy_t strategy)
{
    gdouble candidates[3];
    candidates[0] = az_norm;
    candidates[1] = az_norm + 360.0;
    candidates[2] = az_norm - 360.0;

    gdouble best = candidates[0];
    gdouble best_diff = fabs(candidates[0] - prev);
    gdouble worst = candidates[0];
    gdouble worst_diff = best_diff;

    for (int i = 1; i < 3; i++) {
        gdouble diff = fabs(candidates[i] - prev);
        if (diff < best_diff) {
            best = candidates[i];
            best_diff = diff;
        }
        if (diff > worst_diff) {
            worst = candidates[i];
            worst_diff = diff;
        }
    }

    return (strategy == ROT_PLAN_STRATEGY_S2) ? worst : best;
}

static gboolean crosses_endstop(gdouble prev_az, gdouble next_az,
                                gdouble endstop_az, gdouble az_min,
                                gdouble az_max)
{
    gdouble span = az_max - az_min;

    if (span <= 0.0)
        return FALSE;

    gdouble stop = endstop_az;
    while (stop < az_min)
        stop += span;
    while (stop > az_max)
        stop -= span;

    if (prev_az == next_az)
        return FALSE;

    if (prev_az < next_az) {
        for (gdouble s = stop; s <= next_az; s += span) {
            if (s > prev_az && s < next_az)
                return TRUE;
        }
    } else {
        for (gdouble s = stop; s >= next_az; s -= span) {
            if (s < prev_az && s > next_az)
                return TRUE;
        }
    }

    return FALSE;
}

static gdouble total_motion(const GArray *cmds, gboolean have_start,
                            gdouble start_az, gdouble start_el)
{
    if (cmds == NULL || cmds->len == 0)
        return 0.0;

    gdouble total = 0.0;
    gdouble prev_az;
    gdouble prev_el;
    guint idx = 0;

    if (have_start) {
        prev_az = start_az;
        prev_el = start_el;
    } else {
        rot_plan_cmd_t first = g_array_index(cmds, rot_plan_cmd_t, 0);
        prev_az = first.cmd_az;
        prev_el = first.cmd_el;
        idx = 1;
    }

    for (; idx < cmds->len; idx++) {
        rot_plan_cmd_t cmd = g_array_index(cmds, rot_plan_cmd_t, idx);
        total += angular_distance(cmd.cmd_az, prev_az);
        total += fabs(cmd.cmd_el - prev_el);
        prev_az = cmd.cmd_az;
        prev_el = cmd.cmd_el;
    }

    return total;
}

static gboolean build_for_strategy(const rot_plan_input_t *in,
                                   rot_plan_strategy_t strategy,
                                   rot_plan_result_t *out)
{
    GArray *raw = NULL;
    GArray *clamped = NULL;
    GArray *segmented = NULL;
    gboolean have_prev = FALSE;
    gdouble prev_az_cont = 0.0;
    gdouble prev_az = 0.0;
    gdouble prev_el = 0.0;
    guint trackable = 0;
    gdouble violation_mag = 0.0;

    raw = g_array_new(FALSE, FALSE, sizeof(rot_plan_cmd_t));
    clamped = g_array_new(FALSE, FALSE, sizeof(rot_plan_cmd_t));
    segmented = g_array_new(FALSE, FALSE, sizeof(rot_plan_cmd_t));

    if (in->have_current) {
        prev_az_cont = in->az_current;
        prev_az = in->az_current;
        prev_el = in->el_current;
        have_prev = TRUE;
    }

    for (guint i = 0; i < in->samples->len; i++) {
        rot_plan_sample_t sample =
            g_array_index(in->samples, rot_plan_sample_t, i);
        gdouble az_norm = normalize_az_0_360(sample.az);
        gdouble el = sample.el;

        if (!have_prev) {
            prev_az_cont = az_norm;
            prev_az = az_norm;
            prev_el = el;
            have_prev = TRUE;
        }

        gdouble az_cont = unwrap_select(prev_az_cont, az_norm, strategy);
        gdouble az_cmd = normalize_az_0_360(az_cont);
        gdouble el_cmd = el;

        gboolean in_limits = az_in_range(az_cmd, in->az_min, in->az_max) &&
                             el_cmd >= in->el_min && el_cmd <= in->el_max;

        if (in_limits)
            trackable++;
        else
            violation_mag += az_violation_mag(az_cmd, in->az_min, in->az_max) +
                             MAX(0.0, in->el_min - el_cmd) +
                             MAX(0.0, el_cmd - in->el_max);

        if (in->use_az_stop &&
            crosses_endstop(prev_az_cont, az_cont, in->az_stop,
                            in->az_min, in->az_max)) {
            in_limits = FALSE;
            violation_mag += az_violation_mag(az_cmd, in->az_min, in->az_max);
        }

        rot_plan_cmd_t raw_cmd = { sample.t, az_cmd, el_cmd, in_limits };
        g_array_append_val(raw, raw_cmd);

        rot_plan_cmd_t clamp_cmd = raw_cmd;
        clamp_cmd.cmd_az = az_clamp(az_cmd, in->az_min, in->az_max);
        clamp_cmd.cmd_el = CLAMP(el_cmd, in->el_min, in->el_max);
        clamp_cmd.trackable = in_limits;
        g_array_append_val(clamped, clamp_cmd);

        rot_plan_cmd_t seg_cmd = clamp_cmd;
        if (!in_limits) {
            if (segmented->len > 0) {
                rot_plan_cmd_t last = g_array_index(segmented, rot_plan_cmd_t,
                                                    segmented->len - 1);
                seg_cmd.cmd_az = last.cmd_az;
                seg_cmd.cmd_el = last.cmd_el;
            }
        }
        g_array_append_val(segmented, seg_cmd);

        prev_az_cont = az_cont;
        prev_az = az_cmd;
        prev_el = el_cmd;
    }

    out->strategy = strategy;
    out->trackable_pct = (in->samples->len > 0)
                         ? (100.0 * trackable / (gdouble)in->samples->len)
                         : 0.0;
    out->violation_mag = violation_mag;

    if (out->trackable_pct >= 100.0 - 1e-6) {
        out->status = ROT_PLAN_STATUS_FULL_TRACK;
        out->cmds = raw;
        out->total_motion = total_motion(raw, in->have_current,
                                         in->az_current, in->el_current);
        g_array_free(clamped, TRUE);
        g_array_free(segmented, TRUE);
    } else {
        gdouble clamp_motion = total_motion(clamped, in->have_current,
                                            in->az_current, in->el_current);
        gdouble seg_motion = total_motion(segmented, in->have_current,
                                          in->az_current, in->el_current);
        if (seg_motion <= clamp_motion) {
            out->status = ROT_PLAN_STATUS_PARTIAL_SEGMENTED;
            out->cmds = segmented;
            out->total_motion = seg_motion;
            g_array_free(clamped, TRUE);
        } else {
            out->status = ROT_PLAN_STATUS_PARTIAL_CLAMPED;
            out->cmds = clamped;
            out->total_motion = clamp_motion;
            g_array_free(segmented, TRUE);
        }
        g_array_free(raw, TRUE);
    }

    return TRUE;
}

static gboolean plan_is_better(const rot_plan_result_t *a,
                               const rot_plan_result_t *b)
{
    if (a->trackable_pct != b->trackable_pct)
        return (a->trackable_pct > b->trackable_pct);
    if (a->violation_mag != b->violation_mag)
        return (a->violation_mag < b->violation_mag);
    return (a->total_motion < b->total_motion);
}

gboolean rot_plan_build(const rot_plan_input_t *in, rot_plan_result_t *out)
{
    rot_plan_result_t s1 = { 0 };
    rot_plan_result_t s2 = { 0 };
    rot_plan_result_t *best = NULL;

    if (in == NULL || out == NULL || in->samples == NULL ||
        in->samples->len == 0)
        return FALSE;

    if (!build_for_strategy(in, ROT_PLAN_STRATEGY_S1, &s1))
        return FALSE;
    if (!build_for_strategy(in, ROT_PLAN_STRATEGY_S2, &s2)) {
        rot_plan_result_clear(&s1);
        return FALSE;
    }

    best = plan_is_better(&s1, &s2) ? &s1 : &s2;
    if (best == &s1) {
        rot_plan_result_clear(&s2);
        *out = s1;
    } else {
        rot_plan_result_clear(&s1);
        *out = s2;
    }

    if (out->trackable_pct >= 100.0 - 1e-6) {
        out->reason = g_strdup("full tracking available");
    } else {
        const gchar *mode = (out->status == ROT_PLAN_STATUS_PARTIAL_SEGMENTED)
                            ? "segmented"
                            : "clamped";
        out->reason = g_strdup_printf("partial tracking (%s): %.1f%% trackable",
                                      mode, out->trackable_pct);
    }

    return TRUE;
}

void rot_plan_result_clear(rot_plan_result_t *res)
{
    if (res == NULL)
        return;
    if (res->cmds) {
        g_array_free(res->cmds, TRUE);
        res->cmds = NULL;
    }
    if (res->reason) {
        g_free(res->reason);
        res->reason = NULL;
    }
}

void rot_plan_debug_harness(void)
{
    /* Simple self-check for wrap, negative az, and elevation overflow. */
    rot_plan_input_t in = { 0 };
    rot_plan_result_t out = { 0 };
    GArray *samples = g_array_new(FALSE, FALSE, sizeof(rot_plan_sample_t));

    in.samples = samples;
    in.az_min = 0.0;
    in.az_max = 360.0;
    in.el_min = 0.0;
    in.el_max = 90.0;
    in.have_current = FALSE;
    in.use_az_stop = FALSE;

    rot_plan_sample_t s1[] = {
        { 0.0, 350.0, 10.0 },
        { 1.0, 5.0, 12.0 },
        { 2.0, 15.0, 14.0 }
    };
    rot_plan_sample_t s2[] = {
        { 0.0, -170.0, 20.0 },
        { 1.0, -160.0, 25.0 },
        { 2.0, -150.0, 30.0 }
    };
    rot_plan_sample_t s3[] = {
        { 0.0, 120.0, 80.0 },
        { 1.0, 125.0, 95.0 },
        { 2.0, 130.0, 100.0 }
    };

    g_array_append_vals(samples, s1, G_N_ELEMENTS(s1));
    g_array_append_vals(samples, s2, G_N_ELEMENTS(s2));
    g_array_append_vals(samples, s3, G_N_ELEMENTS(s3));

    if (rot_plan_build(&in, &out)) {
        for (guint i = 0; i < out.cmds->len; i++) {
            rot_plan_cmd_t cmd = g_array_index(out.cmds, rot_plan_cmd_t, i);
            if (cmd.cmd_az < 0.0 || cmd.cmd_az > 360.0)
                g_warning("rot-plan: az out of range %.2f", cmd.cmd_az);
            if (cmd.cmd_el < in.el_min || cmd.cmd_el > in.el_max)
                g_warning("rot-plan: el out of range %.2f", cmd.cmd_el);
        }
        g_message("rot-plan: harness ok (strategy=%d status=%d trackable=%.1f%%)",
                  out.strategy, out.status, out.trackable_pct);
    }

    rot_plan_result_clear(&out);
    g_array_free(samples, TRUE);
}
