/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include "rotor-cmd-map.h"

#include <math.h>

#include "rotor-trajectory-planner.h"
#include "azel_mapping.h"

#define ROT_CMD_EPS 1e-6

static gdouble rot_cmd_angular_distance_abs(gdouble a, gdouble b)
{
    gdouble diff = fabs(normalize_az_0_360(a) - normalize_az_0_360(b));
    if (diff > 180.0)
        diff = 360.0 - diff;
    return diff;
}

static gdouble rot_cmd_normalize_abs(const rotor_conf_t *conf, gdouble az)
{
    if (conf && conf->aztype == ROT_AZ_TYPE_480)
    {
        if (az >= 0.0 && az <= 480.0)
            return az;
        gdouble v = fmod(az, 480.0);
        if (v < 0.0)
            v += 480.0;
        return v;
    }

    return normalize_az_0_360(az);
}

void rot_cmd_get_abs_az_limits(const rotor_conf_t *conf,
                               gdouble *az_min, gdouble *az_max)
{
    gdouble minaz = 0.0;
    gdouble maxaz = 360.0;

    if (conf != NULL) {
        minaz = conf->minaz;
        maxaz = conf->maxaz;
    }

    if (conf != NULL && conf->aztype == ROT_AZ_TYPE_480)
    {
        if (maxaz <= minaz)
            maxaz = minaz + 480.0;
    }
    else
    {
        gdouble span = maxaz - minaz;
        if (span <= 0.0)
            span += 360.0;

        if (span >= 359.0) {
            minaz = 0.0;
            maxaz = 360.0;
        } else {
            minaz = normalize_az_0_360(minaz);
            maxaz = normalize_az_0_360(maxaz);
        }
    }

    if (az_min)
        *az_min = minaz;
    if (az_max)
        *az_max = maxaz;
}

gdouble rot_cmd_az_to_conf(const rotor_conf_t *conf, gdouble az_abs)
{
    if (conf != NULL && conf->aztype == ROT_AZ_TYPE_480)
    {
        return az_abs_to_span(az_abs, AZSPAN_480);
    }

    gdouble az = normalize_az_0_360(az_abs);
    if (conf != NULL && conf->aztype == ROT_AZ_TYPE_180 && az > 180.0)
        az -= 360.0;
    return az;
}

gboolean rot_cmd_az_in_limits(gdouble az, gdouble min, gdouble max)
{
    if (min <= max)
        return (az >= min && az <= max);
    return (az >= min || az <= max);
}

gdouble rot_cmd_clamp_az_abs(gdouble az, gdouble min, gdouble max)
{
    gdouble span = max - min;
    if (span > 360.0 + 1e-6)
    {
        if (az < min)
            return min;
        if (az > max)
            return max;
        return az;
    }

    az = normalize_az_0_360(az);
    if (rot_cmd_az_in_limits(az, min, max))
        return az;

    gdouble dmin = rot_cmd_angular_distance_abs(az, min);
    gdouble dmax = rot_cmd_angular_distance_abs(az, max);
    return (dmin <= dmax) ? min : max;
}

rot_cmd_map_status_t rot_cmd_map(const rotor_conf_t *conf,
                                 gdouble target_az, gdouble target_el,
                                 rot_cmd_map_t *out)
{
    if (out != NULL) {
        out->mapped_az = target_az;
        out->mapped_el = target_el;
        out->send_az = target_az;
        out->send_el = target_el;
    }

    if (conf == NULL || out == NULL)
        return ROT_CMD_MAP_INVALID_CONF;

    if (conf->minel > conf->maxel)
        return ROT_CMD_MAP_INVALID_CONF;

    gdouble az = target_az;
    gdouble el = target_el;

    /* Config is the source of truth for axis mode. */
    if (conf->axis_mode == ROT_AXIS_MODE_AZ_ONLY)
        el = conf->minel;

    if (conf->invert_az)
        az = -az;
    if (conf->invert_el)
        el = -el;

    if (conf->use_offset) {
        az += conf->az_offset;
        el += conf->el_offset;
    }

    gdouble az_min = 0.0;
    gdouble az_max = 360.0;
    gdouble el_min = conf->minel;
    gdouble el_max = conf->maxel;
    rot_cmd_get_abs_az_limits(conf, &az_min, &az_max);

    if (conf->el_overtravel_enable && conf->el_min_deg < conf->el_max_deg)
    {
        el_min = conf->el_min_deg;
        el_max = conf->el_max_deg;
    }

    gdouble az_abs = rot_cmd_normalize_abs(conf, az);
    gdouble az_clamped_abs = rot_cmd_clamp_az_abs(az_abs, az_min, az_max);
    gdouble el_clamped = CLAMP(el, el_min, el_max);

    out->mapped_az = rot_cmd_az_to_conf(conf, az_abs);
    out->mapped_el = el;
    out->send_az = rot_cmd_az_to_conf(conf, az_clamped_abs);
    out->send_el = el_clamped;

    gboolean out_of_range =
        (fabs(az_abs - az_clamped_abs) > ROT_CMD_EPS) ||
        (fabs(el - el_clamped) > ROT_CMD_EPS);

    return out_of_range ? ROT_CMD_MAP_OUT_OF_RANGE : ROT_CMD_MAP_OK;
}
