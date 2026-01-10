#include "rotor-cmd-map.h"

#include <math.h>

#include "rotor-trajectory-planner.h"

#define ROT_CMD_EPS 1e-6

static gdouble rot_cmd_angular_distance_abs(gdouble a, gdouble b)
{
    gdouble diff = fabs(normalize_az_0_360(a) - normalize_az_0_360(b));
    if (diff > 180.0)
        diff = 360.0 - diff;
    return diff;
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

    if (az_min)
        *az_min = minaz;
    if (az_max)
        *az_max = maxaz;
}

gdouble rot_cmd_az_to_conf(const rotor_conf_t *conf, gdouble az_abs)
{
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
    rot_cmd_get_abs_az_limits(conf, &az_min, &az_max);

    gdouble az_abs = normalize_az_0_360(az);
    gdouble az_clamped_abs = rot_cmd_clamp_az_abs(az_abs, az_min, az_max);
    gdouble el_clamped = CLAMP(el, conf->minel, conf->maxel);

    out->mapped_az = rot_cmd_az_to_conf(conf, az_abs);
    out->mapped_el = el;
    out->send_az = rot_cmd_az_to_conf(conf, az_clamped_abs);
    out->send_el = el_clamped;

    gboolean out_of_range =
        (fabs(az_abs - az_clamped_abs) > ROT_CMD_EPS) ||
        (fabs(el - el_clamped) > ROT_CMD_EPS);

    return out_of_range ? ROT_CMD_MAP_OUT_OF_RANGE : ROT_CMD_MAP_OK;
}
