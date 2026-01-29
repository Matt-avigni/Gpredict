#include <assert.h>
#include <math.h>

#include "rotor-target.h"
#include "rot_math.h"

static void expect_close(double actual, double expected)
{
    assert(fabs(actual - expected) < 1e-6);
}

static void expect_equiv_360(double actual, double expected)
{
    double d = fabs(rot_norm360(actual - expected));
    if (d > 180.0)
        d = 360.0 - d;
    assert(d < 1e-6);
}

int main(void)
{
    expect_close(normalize_az_wrap360(361.0), 1.0);
    expect_close(normalize_az_wrap180(-181.0), 179.0);
    expect_close(rot_target_az_distance_wrap360(359.9, 0.1), 0.2);
    expect_close(rot_target_az_distance_wrap180(179.9, -179.9), 0.2);

    rot_target_caps_t caps_norm = { 0 };
    caps_norm.az_min_deg = 0.0;
    caps_norm.az_max_deg = 360.0;
    caps_norm.el_min_deg = 0.0;
    caps_norm.el_max_deg = 90.0;
    caps_norm.az_wrap_mode = ROT_TARGET_WRAP_360;
    caps_norm.clamp_policy = ROT_TARGET_CLAMP_REJECT;
    caps_norm.shortest_path = ROT_TARGET_SHORTEST_PATH;

    gdouble az_norm = 0.0;
    rot_target_invalid_reason_t reason = ROT_TARGET_INVALID_NONE;
    gboolean ok = rot_target_is_valid(&caps_norm, -60.0, 10.0,
                                      &az_norm, &reason);
    assert(ok);
    expect_close(az_norm, 300.0);

    ok = rot_target_is_valid(&caps_norm, 10.0, -13.0, NULL, &reason);
    assert(!ok);
    assert(reason == ROT_TARGET_INVALID_EL_BELOW_MIN);

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
    reason = ROT_TARGET_INVALID_NONE;
    ok = rot_target_find_first_valid_sample(&caps,
                                            samples,
                                            G_N_ELEMENTS(samples),
                                            0.0, 10.0,
                                            &out,
                                            &reason);
    assert(ok);
    expect_close(out.t, 2.0);

    RotLaneSelectResult lane = { 0 };
    ok = rot_backend_lane_select(310.0, 0.0, 360.0, 120.0, TRUE,
                                 0.0, FALSE, 0.0, &lane);
    assert(ok);
    expect_close(lane.az_cmd_backend, 310.0);

    ok = rot_backend_lane_select(310.0, -180.0, 450.0, 120.0, TRUE,
                                 0.0, FALSE, 0.0, &lane);
    assert(ok);
    expect_equiv_360(lane.az_cmd_backend, 310.0);
    assert(fabs(lane.az_cmd_backend - 120.0) > 1.0);

    ok = rot_backend_lane_select(10.0, -180.0, 450.0, 350.0, TRUE,
                                 0.0, FALSE, 0.0, &lane);
    assert(ok);
    expect_close(lane.az_cmd_backend, 370.0);

    {
        double current = 350.0;
        double targets[] = { 350.0, 355.0, 359.0, 1.0, 5.0, 10.0 };
        for (guint i = 0; i < G_N_ELEMENTS(targets); i++)
        {
            double prev_target = (i > 0) ? targets[i - 1] : targets[i];
            ok = rot_backend_lane_select(targets[i],
                                         -180.0, 450.0,
                                         current, TRUE,
                                         0.0, FALSE, 0.0, &lane);
            assert(ok);
            expect_equiv_360(lane.az_cmd_backend, targets[i]);
            if (rot_ang_dist_deg(targets[i], prev_target) > 5.0)
            {
                assert(fabs(lane.az_cmd_backend - current) > 1.0);
            }
            current = lane.az_cmd_backend;
        }
    }

    return 0;
}
