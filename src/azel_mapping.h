/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#ifndef AZEL_MAPPING_H
#define AZEL_MAPPING_H

#include <glib.h>

#include "rotor-conf.h"

typedef enum {
    AZ_MODE_0_360 = 0,
    AZ_MODE_NEG180_POS180,
    AZ_MODE_EXTENDED
} azel_az_mode_t;

typedef enum {
    AZSPAN_PM180 = 0,
    AZSPAN_360,
    AZSPAN_480
} AzSpan;

typedef struct {
    azel_az_mode_t az_mode;
    gdouble        az_min;
    gdouble        az_max;
    gdouble        el_min;
    gdouble        el_max;
    gboolean       prefer_shortest_path;
    gboolean       allow_wrap;
    gboolean       treat_360_as_0;
} SpanConfig;

typedef struct {
    gdouble  cmd_az;
    gdouble  cmd_el;
    gdouble  az_unwrapped;
    gboolean az_clamped;
    gboolean el_clamped;
    gboolean wrapped;
} AzElMapResult;

gdouble azel_normalize_az_0_360(gdouble az);
gdouble azel_normalize_az_neg180_pos180(gdouble az);

gdouble az_norm_span(gdouble az, AzSpan span);
gdouble az_span_width(AzSpan span);
gdouble az_unwrap_to_abs(gdouble prev_abs, gdouble measured_span, AzSpan span);
gdouble az_target_to_nearest_abs(gdouble cur_abs, gdouble target_span, AzSpan span);
gdouble az_abs_to_span(gdouble az_abs, AzSpan span);

SpanConfig azel_span_from_rotor_conf(const rotor_conf_t *conf,
                                     gdouble caps_az_min,
                                     gdouble caps_az_max,
                                     gdouble caps_el_min,
                                     gdouble caps_el_max,
                                     gboolean caps_valid);

gboolean azel_map(const SpanConfig *config,
                  gdouble target_az,
                  gdouble target_el,
                  gdouble current_az,
                  gboolean have_current,
                  AzElMapResult *out);

#endif
