/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#ifndef ROT_ANGLE_H
#define ROT_ANGLE_H

#include <glib.h>

#ifdef __cplusplus
extern "C" {
#endif

double gp_norm360(double az);
double gp_norm180(double az);
double gp_shortest_delta360(double from_az360, double to_az360);
double gp_clamp(double v, double lo, double hi);
double gp_backend_to_az360(double az_backend);
double gp_az360_to_backend_near(double az360,
                                double ref_backend,
                                double min_az,
                                double max_az,
                                int *out_k);

void gp_rot_angle_selfcheck(void);

#ifdef __cplusplus
}
#endif

#endif
