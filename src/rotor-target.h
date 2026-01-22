#ifndef ROTOR_TARGET_H
#define ROTOR_TARGET_H

#include <glib.h>

typedef enum {
    ROT_TARGET_WRAP_360 = 0,
    ROT_TARGET_WRAP_180 = 1
} rot_target_wrap_mode_t;

typedef enum {
    ROT_TARGET_CLAMP_REJECT = 0,
    ROT_TARGET_CLAMP_ALLOW = 1
} rot_target_clamp_policy_t;

typedef enum {
    ROT_TARGET_SHORTEST_PATH = 0,
    ROT_TARGET_NO_WRAP = 1
} rot_target_shortest_path_t;

typedef enum {
    ROT_TARGET_INVALID_NONE = 0,
    ROT_TARGET_INVALID_NAN,
    ROT_TARGET_INVALID_INF,
    ROT_TARGET_INVALID_AZ_ABS_BOUNDS,
    ROT_TARGET_INVALID_EL_ABS_BOUNDS,
    ROT_TARGET_INVALID_AZ_OUTSIDE_RANGE,
    ROT_TARGET_INVALID_EL_BELOW_MIN,
    ROT_TARGET_INVALID_EL_ABOVE_MAX,
    ROT_TARGET_INVALID_WRAP_MISMATCH
} rot_target_invalid_reason_t;

typedef struct {
    gdouble                     az_min_deg;
    gdouble                     az_max_deg;
    gdouble                     el_min_deg;
    gdouble                     el_max_deg;
    rot_target_wrap_mode_t      az_wrap_mode;
    rot_target_clamp_policy_t   clamp_policy;
    rot_target_shortest_path_t  shortest_path;
} rot_target_caps_t;

typedef struct {
    gdouble t;
    gdouble az;
    gdouble el;
} rot_target_sample_t;

gdouble normalize_az_wrap360(gdouble az);
gdouble normalize_az_wrap180(gdouble az);
gdouble rot_target_az_distance_wrap360(gdouble a, gdouble b);
gdouble rot_target_az_distance_wrap180(gdouble a, gdouble b);

gboolean rot_target_is_valid(const rot_target_caps_t *caps,
                             gdouble az,
                             gdouble el,
                             gdouble *az_norm_out,
                             rot_target_invalid_reason_t *reason_out);
const gchar *rot_target_invalid_reason_name(rot_target_invalid_reason_t reason);

gboolean rot_target_find_first_valid_sample(const rot_target_caps_t *caps,
                                            const rot_target_sample_t *samples,
                                            gsize sample_count,
                                            gdouble t_start,
                                            gdouble t_end,
                                            rot_target_sample_t *out,
                                            rot_target_invalid_reason_t *reason_out);

#endif
