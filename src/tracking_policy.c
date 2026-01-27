#include "tracking_policy.h"

#include <math.h>
#include <stdlib.h>

#define TRACKING_DEFAULT_DEADBAND_DEG 0.2
#define TRACKING_DEFAULT_FREQ_DEADBAND_HZ 1.0

static gdouble tracking_angular_distance(gdouble a, gdouble b)
{
    gdouble na = azel_normalize_az_0_360(a);
    gdouble nb = azel_normalize_az_0_360(b);
    gdouble diff = fabs(na - nb);
    if (diff > 180.0)
        diff = 360.0 - diff;
    return diff;
}

static guint tracking_recommend_update_ms(guint base_ms, gint64 rtt_us)
{
    guint update_ms = (base_ms > 0) ? base_ms : 200;

    if (rtt_us > 0)
    {
        guint rtt_ms = (guint)((rtt_us + 999) / 1000);
        guint adaptive = rtt_ms * 2;
        if (adaptive > update_ms)
            update_ms = adaptive;
    }

    return update_ms;
}

void tracking_policy_eval(const TrackingPolicyInput *in,
                          TrackingPolicyOutput *out)
{
    AzElMapResult map = { 0 };
    gdouble az_deadband = TRACKING_DEFAULT_DEADBAND_DEG;
    gdouble el_deadband = TRACKING_DEFAULT_DEADBAND_DEG;
    gdouble freq_deadband = TRACKING_DEFAULT_FREQ_DEADBAND_HZ;
    guint update_ms = 0;
    guint freq_update_ms = 0;
    gboolean out_of_range = FALSE;

    if (out)
        memset(out, 0, sizeof(*out));

    if (in == NULL || out == NULL)
        return;

    if (in->az_deadband_deg > 0.0)
        az_deadband = in->az_deadband_deg;
    if (in->el_deadband_deg > 0.0)
        el_deadband = in->el_deadband_deg;
    if (in->freq_deadband_hz > 0.0)
        freq_deadband = in->freq_deadband_hz;

    (void)azel_map(&in->span,
                   in->target_az, in->target_el,
                   in->current_az, in->have_current_pos,
                   &map);

    out_of_range = map.az_clamped || map.el_clamped;

    update_ms = tracking_recommend_update_ms(in->base_update_ms,
                                             in->last_rtt_us);
    freq_update_ms = update_ms;

    out->cmd_az = map.cmd_az;
    out->cmd_el = map.cmd_el;
    out->az_unwrapped = map.az_unwrapped;
    out->az_clamped = map.az_clamped;
    out->el_clamped = map.el_clamped;
    out->out_of_range = out_of_range;
    out->recommended_update_ms = update_ms;
    out->az_deadband_deg = az_deadband;
    out->el_deadband_deg = el_deadband;

    if (!out_of_range && in->have_current_pos)
    {
        gdouble delta_az = 0.0;
        gdouble delta_el = fabs(out->cmd_el - in->current_el);

        if (in->span.az_mode == AZ_MODE_EXTENDED)
            delta_az = fabs(out->az_unwrapped - in->current_az);
        else
            delta_az = tracking_angular_distance(out->cmd_az, in->current_az);

        out->send_rotor =
            (delta_az > az_deadband) ||
            (delta_el > el_deadband) ||
            (in->max_refresh_ms > 0 && in->elapsed_ms >= in->max_refresh_ms);
    }
    else if (!out_of_range)
    {
        out->send_rotor = TRUE;
    }

    if (in->have_current_freq)
    {
        out->cmd_freq_hz = in->target_freq_hz;
        out->freq_delta_hz = in->target_freq_hz - in->current_freq_hz;
        out->freq_deadband_hz = freq_deadband;
        out->recommended_freq_update_ms = freq_update_ms;
        out->send_freq =
            (fabs(out->freq_delta_hz) >= freq_deadband) ||
            (in->max_refresh_ms > 0 && in->elapsed_ms >= in->max_refresh_ms);
    }
    else
    {
        out->cmd_freq_hz = in->target_freq_hz;
        out->freq_deadband_hz = freq_deadband;
        out->recommended_freq_update_ms = freq_update_ms;
        out->send_freq = TRUE;
    }
}

TrackDecision tracking_policy_decide(const TrackPolicy *p,
                                     double now_s,
                                     double az_abs_cur,
                                     double el_cur,
                                     double az_abs_target,
                                     double el_target,
                                     double az_abs_last_cmd,
                                     double el_last_cmd)
{
    TrackDecision out;
    double deadband = 0.0;
    double dt_s = 0.0;
    double az_target = az_abs_target;
    double el_target_value = el_target;
    double max_step = 0.0;
    double az_cmd = az_abs_last_cmd;
    double el_cmd = el_last_cmd;

    memset(&out, 0, sizeof(out));
    out.az_abs_cmd = az_abs_last_cmd;
    out.el_cmd = el_last_cmd;

    if (p == NULL)
        return out;

    if (p->deadband_deg > 0.0)
        deadband = p->deadband_deg;
    else
        deadband = TRACKING_DEFAULT_DEADBAND_DEG;

    if (now_s > 0.0)
        dt_s = now_s;

    if (p->smoothing_alpha_max > 0.0 &&
        p->smoothing_alpha_min > 0.0 &&
        p->smoothing_alpha_max >= p->smoothing_alpha_min)
    {
        double rtt_s = (p->rtt_ms > 0.0) ? (p->rtt_ms / 1000.0) : 0.0;
        double adapt = rtt_s / (rtt_s + 1.0);
        double alpha = p->smoothing_alpha_max -
                       adapt * (p->smoothing_alpha_max - p->smoothing_alpha_min);
        alpha = CLAMP(alpha, p->smoothing_alpha_min, p->smoothing_alpha_max);

        az_target = az_abs_last_cmd + alpha * (az_abs_target - az_abs_last_cmd);
        el_target_value = el_last_cmd + alpha * (el_target_value - el_last_cmd);
    }

    if (p->max_rate_deg_s > 0.0 && dt_s > 0.0)
        max_step = p->max_rate_deg_s * dt_s;

    if (p->max_step_deg > 0.0)
        max_step = (max_step > 0.0) ? MIN(max_step, p->max_step_deg)
                                    : p->max_step_deg;

    if (max_step > 0.0)
    {
        double az_delta = az_target - az_abs_last_cmd;
        double el_delta = el_target_value - el_last_cmd;

        if (fabs(az_delta) > max_step)
            az_cmd = az_abs_last_cmd + copysign(max_step, az_delta);
        else
            az_cmd = az_target;

        if (fabs(el_delta) > max_step)
            el_cmd = el_last_cmd + copysign(max_step, el_delta);
        else
            el_cmd = el_target_value;
    }
    else
    {
        az_cmd = az_target;
        el_cmd = el_target_value;
    }

    out.az_abs_cmd = az_cmd;
    out.el_cmd = el_cmd;

    if (fabs(az_target - az_abs_cur) <= deadband &&
        fabs(el_target_value - el_cur) <= deadband)
    {
        out.should_send = FALSE;
        return out;
    }

    if (fabs(az_cmd - az_abs_last_cmd) <= 1e-6 &&
        fabs(el_cmd - el_last_cmd) <= 1e-6)
    {
        out.should_send = FALSE;
        return out;
    }

    out.should_send = TRUE;
    return out;
}

static gint tracking_policy_choose_k(gint k_min, gint k_max)
{
    if (k_min > k_max)
        return 0;
    if (k_min <= 0 && k_max >= 0)
        return 0;
    if (abs(k_min) < abs(k_max))
        return k_min;
    if (abs(k_max) < abs(k_min))
        return k_max;
    return (k_min <= 0) ? k_min : k_max;
}

static gboolean tracking_policy_k_range(const rot_target_sample_t *samples,
                                        gsize sample_count,
                                        gdouble az_min,
                                        gdouble az_max,
                                        gdouble el_min,
                                        gdouble el_max,
                                        tracking_wrap_mode_t wrap_mode,
                                        gboolean flipped,
                                        gint *k_min_out,
                                        gint *k_max_out)
{
    (void)wrap_mode;
    const gdouble eps = 1e-6;
    gint k_min = G_MININT / 4;
    gint k_max = G_MAXINT / 4;
    gboolean used_samples = FALSE;
    gdouble minv = az_min;
    gdouble maxv = az_max;

    if (maxv < minv)
        maxv += 360.0;

    for (gsize i = 0; i < sample_count; i++)
    {
        gdouble az = samples[i].az;
        gdouble el = samples[i].el;

        if (flipped)
        {
            az += 180.0;
            el = 180.0 - el;
        }

        if (el < (el_min - eps) || el > (el_max + eps))
            continue;

        used_samples = TRUE;

        gdouble base = az;

        gdouble k_lo_d = (minv - base) / 360.0;
        gdouble k_hi_d = (maxv - base) / 360.0;
        gint k_lo = (gint)ceil(k_lo_d - 1e-9);
        gint k_hi = (gint)floor(k_hi_d + 1e-9);

        if (k_lo > k_hi)
            return FALSE;

        if (k_lo > k_min)
            k_min = k_lo;
        if (k_hi < k_max)
            k_max = k_hi;
    }

    if (!used_samples)
    {
        if (k_min_out)
            *k_min_out = 0;
        if (k_max_out)
            *k_max_out = 0;
        return TRUE;
    }

    if (k_min > k_max)
        return FALSE;

    if (k_min_out)
        *k_min_out = k_min;
    if (k_max_out)
        *k_max_out = k_max;

    return TRUE;
}

void rot_tracking_policy_reset(RotTrackingPolicy *p)
{
    if (p == NULL)
        return;

    p->wrap_mode = TRACKING_WRAP_CONTINUOUS;
    p->geom_mode = TRACKING_GEOM_NORMAL;
    p->az_min = 0.0;
    p->az_max = 360.0;
    p->el_min = 0.0;
    p->el_max = 180.0;
    p->seam = 0.0;
    p->preferred_k = 0;
    p->last_cmd_az = 0.0;
    p->last_cmd_valid = FALSE;
    p->chosen = FALSE;
    p->degraded = FALSE;
    p->logged_no_branch = FALSE;
    p->logged_empty_set = FALSE;
    p->window_start = 0.0;
    p->window_end = 0.0;
}

gboolean rot_tracking_policy_choose(RotTrackingPolicy *p,
                                    tracking_wrap_mode_t wrap_mode,
                                    gdouble az_min,
                                    gdouble az_max,
                                    gdouble el_min,
                                    gdouble el_max,
                                    gboolean allow_flip,
                                    const rot_target_sample_t *samples,
                                    gsize sample_count,
                                    gboolean *used_flip_out,
                                    gboolean *degraded_out)
{
    gint normal_min = 0;
    gint normal_max = 0;
    gint flip_min = 0;
    gint flip_max = 0;
    gboolean normal_ok = FALSE;
    gboolean flip_ok = FALSE;
    gboolean degraded = FALSE;
    gboolean use_flip = FALSE;

    if (used_flip_out)
        *used_flip_out = FALSE;
    if (degraded_out)
        *degraded_out = FALSE;

    if (p == NULL || samples == NULL || sample_count == 0)
        return FALSE;

    normal_ok = tracking_policy_k_range(samples, sample_count,
                                        az_min, az_max,
                                        el_min, el_max,
                                        wrap_mode, FALSE,
                                        &normal_min, &normal_max);

    if (allow_flip)
    {
        flip_ok = tracking_policy_k_range(samples, sample_count,
                                          az_min, az_max,
                                          el_min, el_max,
                                          wrap_mode, TRUE,
                                          &flip_min, &flip_max);
    }

    if (normal_ok)
    {
        use_flip = FALSE;
    }
    else if (flip_ok && allow_flip)
    {
        use_flip = TRUE;
    }
    else
    {
        use_flip = FALSE;
        degraded = TRUE;
    }

    p->wrap_mode = wrap_mode;
    p->geom_mode = use_flip ? TRACKING_GEOM_FLIPPED : TRACKING_GEOM_NORMAL;
    p->az_min = az_min;
    p->az_max = az_max;
    p->el_min = el_min;
    p->el_max = el_max;
    p->seam = az_min;
    p->preferred_k = use_flip
                     ? tracking_policy_choose_k(flip_min, flip_max)
                     : tracking_policy_choose_k(normal_min, normal_max);
    p->chosen = TRUE;
    p->degraded = degraded;
    p->logged_no_branch = FALSE;
    p->logged_empty_set = FALSE;
    p->last_cmd_valid = FALSE;

    if (used_flip_out)
        *used_flip_out = use_flip;
    if (degraded_out)
        *degraded_out = degraded;

    return TRUE;
}

gboolean rot_tracking_policy_select_cmd(const RotTrackingPolicy *p,
                                        gdouble az_pred,
                                        gdouble el_pred,
                                        gboolean have_measured,
                                        gdouble measured_az,
                                        gdouble last_cmd_az,
                                        gdouble *az_cmd_out,
                                        gdouble *el_cmd_out,
                                        gboolean *empty_set_out)
{
    const gdouble eps = 1e-6;
    gdouble az = az_pred;
    gdouble el = el_pred;
    gdouble ref = last_cmd_az;
    gboolean has_ref = TRUE;
    gdouble best = 0.0;
    gdouble best_diff = G_MAXDOUBLE;
    gboolean found = FALSE;
    gint k_min = 0;
    gint k_max = 0;

    if (empty_set_out)
        *empty_set_out = FALSE;

    if (p == NULL || !p->chosen)
        return FALSE;

    if (p->geom_mode == TRACKING_GEOM_FLIPPED)
    {
        az += 180.0;
        el = 180.0 - el;
    }

    if (el < (p->el_min - eps) || el > (p->el_max + eps))
    {
        if (empty_set_out)
            *empty_set_out = TRUE;
        return FALSE;
    }

    if (have_measured && isfinite(measured_az))
    {
        ref = measured_az;
        has_ref = TRUE;
    }
    else if (!isfinite(last_cmd_az))
    {
        has_ref = FALSE;
    }

    if (p->wrap_mode == TRACKING_WRAP_CONTINUOUS)
    {
        gdouble base = az;

        gdouble minv = p->az_min;
        gdouble maxv = p->az_max;
        if (maxv < minv)
            maxv += 360.0;

        gdouble k_lo_d = (minv - base) / 360.0;
        gdouble k_hi_d = (maxv - base) / 360.0;
        k_min = (gint)ceil(k_lo_d - 1e-9);
        k_max = (gint)floor(k_hi_d + 1e-9);
    }
    else
    {
        gdouble base = az;

        gdouble minv = p->az_min;
        gdouble maxv = p->az_max;
        if (maxv < minv)
            maxv += 360.0;

        gdouble k_lo_d = (minv - base) / 360.0;
        gdouble k_hi_d = (maxv - base) / 360.0;
        k_min = MAX((gint)ceil(k_lo_d - 1e-9), -1);
        k_max = MIN((gint)floor(k_hi_d + 1e-9), 1);
    }

    if (k_min > k_max)
    {
        if (empty_set_out)
            *empty_set_out = TRUE;
        return FALSE;
    }

    for (gint k = k_min; k <= k_max; k++)
    {
        gdouble base = az;
        gdouble cand = base + (360.0 * k);
        gdouble diff;

        if (cand < p->az_min - eps || cand > p->az_max + eps)
            continue;

        if (has_ref)
            diff = fabs(cand - ref);
        else
            diff = fabs(cand - (base + 360.0 * p->preferred_k));

        if (!found || diff < best_diff)
        {
            best = cand;
            best_diff = diff;
            found = TRUE;
        }
    }

    if (!found)
    {
        if (empty_set_out)
            *empty_set_out = TRUE;
        return FALSE;
    }

    if (az_cmd_out)
        *az_cmd_out = best;
    if (el_cmd_out)
        *el_cmd_out = el;

    return TRUE;
}
