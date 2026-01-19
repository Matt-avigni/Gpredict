#include "tracking_policy.h"

#include <math.h>

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
