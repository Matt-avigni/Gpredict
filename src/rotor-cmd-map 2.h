/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#ifndef ROTOR_CMD_MAP_H
#define ROTOR_CMD_MAP_H

#include <glib.h>

#include "rotor-conf.h"

typedef enum {
    ROT_CMD_MAP_OK = 0,
    ROT_CMD_MAP_OUT_OF_RANGE = 1,
    ROT_CMD_MAP_INVALID_CONF = 2
} rot_cmd_map_status_t;

typedef struct {
    gdouble         mapped_az;
    gdouble         mapped_el;
    gdouble         send_az;
    gdouble         send_el;
} rot_cmd_map_t;

void   rot_cmd_get_abs_az_limits(const rotor_conf_t *conf,
                                 gdouble *az_min, gdouble *az_max);
gdouble rot_cmd_az_to_conf(const rotor_conf_t *conf, gdouble az_abs);
gboolean rot_cmd_az_in_limits(gdouble az_abs, gdouble min, gdouble max);
gdouble rot_cmd_clamp_az_abs(gdouble az_abs, gdouble min, gdouble max);

rot_cmd_map_status_t rot_cmd_map(const rotor_conf_t *conf,
                                 gdouble target_az, gdouble target_el,
                                 rot_cmd_map_t *out);

#endif
