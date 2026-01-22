#include <assert.h>
#include <math.h>

#include "rotor-target.h"

static void expect_close(double actual, double expected)
{
    assert(fabs(actual - expected) < 1e-6);
}

int main(void)
{
    expect_close(normalize_az_wrap360(361.0), 1.0);
    expect_close(normalize_az_wrap180(-181.0), 179.0);
    expect_close(rot_target_az_distance_wrap360(359.9, 0.1), 0.2);
    expect_close(rot_target_az_distance_wrap180(179.9, -179.9), 0.2);

    rot_target_caps_t caps = { 0 };
    caps.az_min_deg = -180.0;
    caps.az_max_deg = 180.0;
    caps.el_min_deg = 0.0;
    caps.el_max_deg = 90.0;
    caps.az_wrap_mode = ROT_TARGET_WRAP_180;

    rot_target_sample_t samples[] = {
        { 0.0, 10.0, -5.0 },
        { 1.0, 1000.0, 10.0 },
        { 2.0, 190.0, 45.0 },
        { 3.0, 30.0, 30.0 }
    };

    rot_target_sample_t out = { 0 };
    rot_target_invalid_reason_t reason = ROT_TARGET_INVALID_NONE;
    gboolean ok = rot_target_find_first_valid_sample(&caps,
                                                     samples,
                                                     G_N_ELEMENTS(samples),
                                                     0.0, 10.0,
                                                     &out,
                                                     &reason);
    assert(ok);
    expect_close(out.t, 2.0);

    return 0;
}
