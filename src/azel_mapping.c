/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include "azel_mapping.h"

#include <math.h>

#define AZEL_EPS 1e-6

static gdouble azel_angular_distance(gdouble a, gdouble b)
{
    gdouble na = azel_normalize_az_0_360(a);
    gdouble nb = azel_normalize_az_0_360(b);
    gdouble diff = fabs(na - nb);
    if (diff > 180.0)
        diff = 360.0 - diff;
    return diff;
}

static gdouble azel_round_nearest(gdouble v)
{
    return (v >= 0.0) ? floor(v + 0.5) : ceil(v - 0.5);
}

gdouble azel_normalize_az_0_360(gdouble az)
{
    gdouble v = fmod(az, 360.0);
    if (v < 0.0)
        v += 360.0;
    if (v >= 360.0)
        v -= 360.0;
    return v;
}

gdouble azel_normalize_az_neg180_pos180(gdouble az)
{
    gdouble v = fmod(az, 360.0);
    if (v > 180.0)
        v -= 360.0;
    if (v <= -180.0)
        v += 360.0;
    return v;
}

gdouble az_span_width(AzSpan span)
{
    switch (span) {
    case AZSPAN_480:
        return 480.0;
    case AZSPAN_PM180:
    case AZSPAN_360:
    default:
        return 360.0;
    }
}

gdouble az_abs_to_span(gdouble az_abs, AzSpan span)
{
    gdouble v = az_abs;

    switch (span) {
    case AZSPAN_PM180:
        return azel_normalize_az_neg180_pos180(az_abs);
    case AZSPAN_480:
    {
        const gdouble width = 480.0;
        if (az_abs >= 0.0 && az_abs <= width)
            return az_abs;
        v = fmod(az_abs, width);
        if (v < 0.0)
            v += width;
        return v;
    }
    case AZSPAN_360:
    default:
        if (az_abs >= 0.0 && az_abs <= 360.0)
            return az_abs;
        return azel_normalize_az_0_360(az_abs);
    }
}

gdouble az_norm_span(gdouble az, AzSpan span)
{
    return az_abs_to_span(az, span);
}

gdouble az_unwrap_to_abs(gdouble prev_abs, gdouble measured_span, AzSpan span)
{
    gdouble width = az_span_width(span);
    gdouble base = az_norm_span(measured_span, span);
    if (width <= 0.0)
        return base;

    gdouble k = azel_round_nearest((prev_abs - base) / width);
    return base + (k * width);
}

gdouble az_target_to_nearest_abs(gdouble cur_abs, gdouble target_span, AzSpan span)
{
    gdouble width = az_span_width(span);
    gdouble base = az_norm_span(target_span, span);
    if (width <= 0.0)
        return base;

    gdouble k = azel_round_nearest((cur_abs - base) / width);
    return base + (k * width);
}

static gboolean azel_az_in_limits(gdouble az, gdouble min, gdouble max)
{
    if (min <= max)
        return (az >= min && az <= max);
    return (az >= min || az <= max);
}

static gdouble azel_clamp_az(gdouble az, gdouble min, gdouble max,
                             gboolean use_wrap, gboolean *clamped)
{
    if (azel_az_in_limits(az, min, max))
    {
        if (clamped)
            *clamped = FALSE;
        return az;
    }

    if (clamped)
        *clamped = TRUE;

    if (!use_wrap)
        return (fabs(az - min) <= fabs(az - max)) ? min : max;

    {
        gdouble dmin = azel_angular_distance(az, min);
        gdouble dmax = azel_angular_distance(az, max);
        return (dmin <= dmax) ? min : max;
    }
}

static gdouble azel_select_extended(const SpanConfig *config,
                                    gdouble az_norm,
                                    gdouble current_az,
                                    gboolean have_current,
                                    gboolean *wrapped)
{
    gdouble best = az_norm;
    gdouble best_diff = G_MAXDOUBLE;
    gboolean found = FALSE;

    if (wrapped)
        *wrapped = FALSE;

    if (!config->allow_wrap)
        return az_norm;

    for (gint k = -2; k <= 2; k++)
    {
        gdouble cand = az_norm + 360.0 * k;
        if (!azel_az_in_limits(cand, config->az_min, config->az_max))
            continue;

        if (!found)
        {
            best = cand;
            found = TRUE;
            best_diff = have_current ? fabs(cand - current_az)
                                     : fabs(cand - az_norm);
            continue;
        }

        if (have_current && config->prefer_shortest_path)
        {
            gdouble diff = fabs(cand - current_az);
            if (diff < best_diff)
            {
                best = cand;
                best_diff = diff;
            }
        }
        else if (!have_current)
        {
            gdouble diff = fabs(cand - az_norm);
            if (diff < best_diff)
            {
                best = cand;
                best_diff = diff;
            }
        }
    }

    if (found && wrapped && fabs(best - az_norm) > AZEL_EPS)
        *wrapped = TRUE;

    return best;
}

SpanConfig azel_span_from_rotor_conf(const rotor_conf_t *conf,
                                     gdouble caps_az_min,
                                     gdouble caps_az_max,
                                     gdouble caps_el_min,
                                     gdouble caps_el_max,
                                     gboolean caps_valid)
{
    SpanConfig cfg;
    gdouble az_min = 0.0;
    gdouble az_max = 360.0;
    gdouble el_min = 0.0;
    gdouble el_max = 180.0;
    gdouble span = 360.0;

    memset(&cfg, 0, sizeof(cfg));

    if (conf != NULL)
    {
        az_min = conf->minaz;
        az_max = conf->maxaz;
        el_min = conf->minel;
        el_max = conf->maxel;
    }

    if (caps_valid)
    {
        az_min = caps_az_min;
        az_max = caps_az_max;
        el_min = caps_el_min;
        el_max = caps_el_max;
    }

    span = az_max - az_min;
    if (span <= 0.0)
        span += 360.0;

    if (conf != NULL && conf->aztype == ROT_AZ_TYPE_180)
    {
        cfg.az_mode = AZ_MODE_NEG180_POS180;
        cfg.az_min = -180.0;
        cfg.az_max = 180.0;
    }
    else if (conf != NULL && conf->aztype == ROT_AZ_TYPE_480)
    {
        cfg.az_mode = AZ_MODE_EXTENDED;
        cfg.az_min = az_min;
        cfg.az_max = az_max;
    }
    else if (span > 360.0 + AZEL_EPS)
    {
        cfg.az_mode = AZ_MODE_EXTENDED;
        cfg.az_min = az_min;
        cfg.az_max = az_max;
    }
    else
    {
        cfg.az_mode = AZ_MODE_0_360;
        cfg.az_min = az_min;
        cfg.az_max = az_max;
        if (span >= 359.0)
        {
            cfg.az_min = 0.0;
            cfg.az_max = 360.0;
        }
    }

    cfg.el_min = el_min;
    cfg.el_max = el_max;

    cfg.prefer_shortest_path =
        (cfg.az_mode == AZ_MODE_EXTENDED || cfg.az_mode == AZ_MODE_NEG180_POS180);
    cfg.allow_wrap =
        (cfg.az_mode == AZ_MODE_EXTENDED || cfg.az_mode == AZ_MODE_NEG180_POS180);
    cfg.treat_360_as_0 = (cfg.az_mode == AZ_MODE_0_360);

    return cfg;
}

gboolean azel_map(const SpanConfig *config,
                  gdouble target_az,
                  gdouble target_el,
                  gdouble current_az,
                  gboolean have_current,
                  AzElMapResult *out)
{
    gdouble az_norm = 0.0;
    gdouble az_unwrapped = 0.0;
    gdouble az_cmd = 0.0;
    gdouble el_cmd = 0.0;
    gboolean az_clamped = FALSE;
    gboolean el_clamped = FALSE;
    gboolean wrapped = FALSE;
    gboolean use_wrap = FALSE;

    if (out != NULL)
        memset(out, 0, sizeof(*out));

    if (config == NULL || out == NULL)
        return FALSE;

    if (config->az_mode == AZ_MODE_NEG180_POS180)
        az_norm = azel_normalize_az_neg180_pos180(target_az);
    else
        az_norm = azel_normalize_az_0_360(target_az);

    az_unwrapped = az_norm;
    az_cmd = az_norm;

    if (config->az_mode == AZ_MODE_EXTENDED)
    {
        az_unwrapped = azel_select_extended(config, az_norm,
                                            current_az, have_current,
                                            &wrapped);
        az_cmd = az_unwrapped;
        use_wrap = FALSE;
    }
    else if (config->az_mode == AZ_MODE_0_360)
    {
        az_cmd = az_norm;
        use_wrap = (config->az_min > config->az_max);
    }
    else
    {
        az_cmd = az_norm;
        use_wrap = FALSE;
    }

    az_cmd = azel_clamp_az(az_cmd, config->az_min, config->az_max,
                           use_wrap, &az_clamped);

    el_cmd = CLAMP(target_el, config->el_min, config->el_max);
    el_clamped = fabs(el_cmd - target_el) > AZEL_EPS;

    if (config->az_mode == AZ_MODE_0_360)
    {
        az_cmd = azel_normalize_az_0_360(az_cmd);
        if (config->treat_360_as_0 && fabs(az_cmd - 360.0) < AZEL_EPS)
            az_cmd = 0.0;
        if (az_cmd < 0.0)
            az_cmd = azel_normalize_az_0_360(az_cmd);
    }
    else if (config->az_mode == AZ_MODE_NEG180_POS180)
    {
        az_cmd = azel_normalize_az_neg180_pos180(az_cmd);
    }

    out->cmd_az = az_cmd;
    out->cmd_el = el_cmd;
    out->az_unwrapped = az_unwrapped;
    out->az_clamped = az_clamped;
    out->el_clamped = el_clamped;
    out->wrapped = wrapped;

    return TRUE;
}
