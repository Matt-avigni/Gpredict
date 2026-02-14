/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include "rotor-target.h"

#include <math.h>

#include "azel_mapping.h"

#define ROT_TARGET_AZ_ABS_MIN -720.0
#define ROT_TARGET_AZ_ABS_MAX 720.0
#define ROT_TARGET_EL_ABS_MIN -90.0
#define ROT_TARGET_EL_ABS_MAX 270.0

static gboolean rot_target_az_in_limits(gdouble az, gdouble min, gdouble max)
{
    if (min <= max)
        return (az >= min && az <= max);
    return (az >= min || az <= max);
}

gdouble normalize_az_wrap360(gdouble az)
{
    return azel_normalize_az_0_360(az);
}

gdouble normalize_az_wrap180(gdouble az)
{
    return azel_normalize_az_neg180_pos180(az);
}

gdouble rot_target_az_distance_wrap360(gdouble a, gdouble b)
{
    gdouble na = normalize_az_wrap360(a);
    gdouble nb = normalize_az_wrap360(b);
    gdouble diff = fabs(na - nb);
    if (diff > 180.0)
        diff = 360.0 - diff;
    return diff;
}

gdouble rot_target_az_distance_wrap180(gdouble a, gdouble b)
{
    gdouble na = normalize_az_wrap180(a);
    gdouble nb = normalize_az_wrap180(b);
    gdouble delta = na - nb;

    if (delta >= 180.0)
        delta -= 360.0;
    if (delta < -180.0)
        delta += 360.0;

    return fabs(delta);
}

gboolean rot_target_is_valid(const rot_target_caps_t *caps,
                             gdouble az,
                             gdouble el,
                             gdouble *az_norm_out,
                             rot_target_invalid_reason_t *reason_out)
{
    const gdouble eps = 1e-6;
    gdouble az_norm = az;
    gdouble span = 0.0;
    gboolean span_extended = FALSE;

    if (reason_out)
        *reason_out = ROT_TARGET_INVALID_NONE;
    if (az_norm_out)
        *az_norm_out = az;

    if (caps == NULL)
    {
        if (reason_out)
            *reason_out = ROT_TARGET_INVALID_WRAP_MISMATCH;
        return FALSE;
    }

    if (isnan(az) || isnan(el))
    {
        if (reason_out)
            *reason_out = ROT_TARGET_INVALID_NAN;
        return FALSE;
    }
    if (!isfinite(az) || !isfinite(el))
    {
        if (reason_out)
            *reason_out = ROT_TARGET_INVALID_INF;
        return FALSE;
    }

    if (az < ROT_TARGET_AZ_ABS_MIN || az > ROT_TARGET_AZ_ABS_MAX)
    {
        if (reason_out)
            *reason_out = ROT_TARGET_INVALID_AZ_ABS_BOUNDS;
        return FALSE;
    }
    if (el < ROT_TARGET_EL_ABS_MIN || el > ROT_TARGET_EL_ABS_MAX)
    {
        if (reason_out)
            *reason_out = ROT_TARGET_INVALID_EL_ABS_BOUNDS;
        return FALSE;
    }

    span = caps->az_max_deg - caps->az_min_deg;
    span_extended = (span > 360.0 + eps);

    if (caps->az_wrap_mode == ROT_TARGET_WRAP_180)
    {
        if (caps->az_min_deg < -180.0 - eps || caps->az_max_deg > 180.0 + eps)
        {
            if (reason_out)
                *reason_out = ROT_TARGET_INVALID_WRAP_MISMATCH;
            return FALSE;
        }
        az_norm = normalize_az_wrap180(az);
    }
    else
    {
        if (!span_extended)
            az_norm = normalize_az_wrap360(az);
    }

    if (az_norm_out)
        *az_norm_out = az_norm;

    if (!rot_target_az_in_limits(az_norm, caps->az_min_deg, caps->az_max_deg))
    {
        if (reason_out)
            *reason_out = ROT_TARGET_INVALID_AZ_OUTSIDE_RANGE;
        return FALSE;
    }

    if (el < caps->el_min_deg)
    {
        if (reason_out)
            *reason_out = ROT_TARGET_INVALID_EL_BELOW_MIN;
        return FALSE;
    }

    if (el > caps->el_max_deg)
    {
        if (reason_out)
            *reason_out = ROT_TARGET_INVALID_EL_ABOVE_MAX;
        return FALSE;
    }

    return TRUE;
}

const gchar *rot_target_invalid_reason_name(rot_target_invalid_reason_t reason)
{
    switch (reason)
    {
    case ROT_TARGET_INVALID_NONE:
        return "OK";
    case ROT_TARGET_INVALID_NAN:
        return "NAN";
    case ROT_TARGET_INVALID_INF:
        return "INF";
    case ROT_TARGET_INVALID_AZ_ABS_BOUNDS:
        return "AZ_ABS_BOUNDS";
    case ROT_TARGET_INVALID_EL_ABS_BOUNDS:
        return "EL_ABS_BOUNDS";
    case ROT_TARGET_INVALID_AZ_OUTSIDE_RANGE:
        return "AZ_OUTSIDE_RANGE";
    case ROT_TARGET_INVALID_EL_BELOW_MIN:
        return "EL_BELOW_MIN";
    case ROT_TARGET_INVALID_EL_ABOVE_MAX:
        return "EL_ABOVE_MAX";
    case ROT_TARGET_INVALID_WRAP_MISMATCH:
        return "WRAP_MISMATCH";
    default:
        return "UNKNOWN";
    }
}

gboolean rot_target_find_first_valid_sample(const rot_target_caps_t *caps,
                                            const rot_target_sample_t *samples,
                                            gsize sample_count,
                                            gdouble t_start,
                                            gdouble t_end,
                                            rot_target_sample_t *out,
                                            rot_target_invalid_reason_t *reason_out)
{
    rot_target_invalid_reason_t last_reason = ROT_TARGET_INVALID_NONE;

    if (reason_out)
        *reason_out = ROT_TARGET_INVALID_NONE;
    if (samples == NULL || sample_count == 0 || caps == NULL)
        return FALSE;
    if (t_end < t_start)
        return FALSE;

    for (gsize i = 0; i < sample_count; i++)
    {
        if (samples[i].t < t_start || samples[i].t > t_end)
            continue;

        rot_target_invalid_reason_t reason = ROT_TARGET_INVALID_NONE;
        if (rot_target_is_valid(caps, samples[i].az, samples[i].el, NULL, &reason))
        {
            if (out)
                *out = samples[i];
            if (reason_out)
                *reason_out = ROT_TARGET_INVALID_NONE;
            return TRUE;
        }
        last_reason = reason;
    }

    if (reason_out)
        *reason_out = last_reason;
    return FALSE;
}
