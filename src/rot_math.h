/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#ifndef ROT_MATH_H
#define ROT_MATH_H

#include <glib.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ROT_UI_360 = 0,
    ROT_UI_NORTH_CENTERED = 1
} rot_ui_mode_t;

typedef struct {
    gdouble az_cmd_backend;
    gboolean clamped;
    gboolean crosses_seam;
    gboolean used_margin;
    gdouble score;
} RotLaneSelectResult;

gdouble rot_norm360(gdouble x);

gdouble rot_norm180(gdouble x);

gdouble rot_delta_shortest_360(gdouble from360, gdouble to360);

gdouble rot_ang_diff_deg(gdouble a, gdouble b);

gdouble rot_ang_dist_deg(gdouble a, gdouble b);

gdouble rot_ui_to_az360(gdouble ui_az, rot_ui_mode_t mode);

gdouble rot_az360_to_ui(gdouble az360, rot_ui_mode_t mode);

gdouble rot_backend_pos_to_az360(gdouble az_backend);

gboolean rot_backend_lane_select(gdouble target_az360,
                                 gdouble backend_min,
                                 gdouble backend_max,
                                 gdouble last_cmd_backend,
                                 gboolean have_last_cmd,
                                 gdouble seam360,
                                 gboolean seam_valid,
                                 gdouble endstop_margin_deg,
                                 RotLaneSelectResult *out);

#ifdef __cplusplus
}
#endif

#endif
