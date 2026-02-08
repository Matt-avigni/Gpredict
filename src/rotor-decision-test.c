#include <assert.h>

#include <glib.h>

#include "rotor-decision.h"

static rot_target_caps_t default_caps(void)
{
    rot_target_caps_t caps = { 0 };
    caps.az_min_deg = 0.0;
    caps.az_max_deg = 360.0;
    caps.el_min_deg = 0.0;
    caps.el_max_deg = 90.0;
    caps.az_wrap_mode = ROT_TARGET_WRAP_360;
    caps.clamp_policy = ROT_TARGET_CLAMP_REJECT;
    caps.shortest_path = ROT_TARGET_SHORTEST_PATH;
    return caps;
}

static rot_cmd_decision_input_t base_input(void)
{
    rot_cmd_decision_input_t in = { 0 };
    in.mode = ROT_CMD_MODE_TRACKING;
    in.allow_send = TRUE;
    in.have_target = TRUE;
    in.force_send = FALSE;
    in.stale_hold = FALSE;
    in.setpoint_valid = TRUE;
    in.pos_fresh = TRUE;
    in.last_good_age_ms = 0;
    in.not_at_target = TRUE;
    in.moving_toward = TRUE;
    in.stopped_unexpected = FALSE;
    in.pos_recovered = FALSE;
    in.resend_due = FALSE;
    in.now_us = 1000000;
    in.last_send_us = 0;
    in.min_interval_us = 0;
    in.desired_user_az = 10.0;
    in.desired_user_el = 5.0;
    in.desired_backend_az = 10.0;
    in.desired_backend_el = 5.0;
    in.setpoint_user_az = 10.2;
    in.setpoint_user_el = 5.1;
    in.last_cmd_user_az = 10.0;
    in.last_cmd_user_el = 5.0;
    in.delta_backend_az = 2.0;
    in.delta_backend_el = 2.0;
    in.deadband_az = 1.0;
    in.deadband_el = 1.0;
    in.min_step_az = 0.8;
    in.min_step_el = 0.5;
    in.target_change_az = 2.0;
    in.target_change_el = 2.0;
    in.caps = default_caps();
    return in;
}

static void expect_reason(const char *actual, const char *expected)
{
    assert(actual != NULL);
    assert(expected != NULL);
    assert(g_strcmp0(actual, expected) == 0);
}

int main(void)
{
    rot_cmd_decision_t out = { 0 };

    /* Range clamp */
    rot_cmd_decision_input_t in = base_input();
    in.caps.az_max_deg = 180.0;
    in.desired_user_az = 200.0;
    in.resend_due = TRUE;
    in.delta_backend_az = 10.0;
    in.delta_backend_el = 10.0;
    rot_cmd_decision_eval(&in, &out);
    assert(!out.send);
    expect_reason(rot_cmd_reason_name(out.reason), "range");

    /* Deadband suppression */
    in = base_input();
    in.desired_user_az = 10.1;
    in.desired_user_el = 5.05;
    in.delta_backend_az = 0.1;
    in.delta_backend_el = 0.1;
    rot_cmd_decision_eval(&in, &out);
    assert(!out.send);
    expect_reason(rot_cmd_reason_name(out.reason), "deadband");

    /* Force send should not rearm when target unchanged */
    in = base_input();
    in.force_send = TRUE;
    in.desired_user_az = in.setpoint_user_az;
    in.desired_user_el = in.setpoint_user_el;
    in.delta_backend_az = 0.0;
    in.delta_backend_el = 0.0;
    rot_cmd_decision_eval(&in, &out);
    assert(!out.send);
    expect_reason(rot_cmd_reason_name(out.reason), "deadband");

    /* Stable target: suppress repeatedly even at high tick rates */
    in = base_input();
    in.desired_user_az = in.setpoint_user_az;
    in.desired_user_el = in.setpoint_user_el;
    in.delta_backend_az = 0.0;
    in.delta_backend_el = 0.0;
    for (int i = 0; i < 100; i++)
    {
        rot_cmd_decision_eval(&in, &out);
        assert(!out.send);
        expect_reason(rot_cmd_reason_name(out.reason), "deadband");
    }

    /* Track press -> PRETRACK immediate initial send */
    in = base_input();
    in.mode = ROT_CMD_MODE_PRETRACK;
    in.setpoint_valid = FALSE;
    in.not_at_target = FALSE;
    rot_cmd_decision_eval(&in, &out);
    assert(out.send);
    expect_reason(rot_cmd_reason_name(out.reason), "initial");

    /* Min-step suppression (both axes below threshold) */
    in = base_input();
    in.desired_user_az = 11.5;
    in.desired_user_el = 6.0;
    in.delta_backend_az = 0.4;
    in.delta_backend_el = 0.2;
    rot_cmd_decision_eval(&in, &out);
    assert(!out.send);
    expect_reason(rot_cmd_reason_name(out.reason), "min_step");

    /* Min-step OR logic (one axis exceeds threshold) */
    in = base_input();
    in.desired_user_az = 11.5;
    in.desired_user_el = 6.0;
    in.delta_backend_az = 1.2;
    in.delta_backend_el = 0.2;
    in.not_at_target = FALSE;
    rot_cmd_decision_eval(&in, &out);
    assert(out.send);
    expect_reason(rot_cmd_reason_name(out.reason), "target");

    /* Threshold-only path: 0.30 deg delta must send when threshold is 0.10 */
    in = base_input();
    in.deadband_az = 0.1;
    in.deadband_el = 0.1;
    in.min_step_az = 0.1;
    in.min_step_el = 0.1;
    in.target_change_az = 0.1;
    in.target_change_el = 0.1;
    in.desired_user_az = 10.3;
    in.desired_user_el = 5.0;
    in.setpoint_user_az = 10.0;
    in.setpoint_user_el = 5.0;
    in.delta_backend_az = 0.3;
    in.delta_backend_el = 0.0;
    rot_cmd_decision_eval(&in, &out);
    assert(out.send);
    expect_reason(rot_cmd_reason_name(out.reason), "target_change");

    /* In-flight suppression */
    in = base_input();
    in.desired_user_az = 13.0;
    in.desired_user_el = 6.5;
    in.setpoint_user_az = 11.8;
    in.setpoint_user_el = 5.9;
    rot_cmd_decision_eval(&in, &out);
    assert(!out.send);
    expect_reason(rot_cmd_reason_name(out.reason), "in_flight");

    /* No position feedback inside deadband */
    in = base_input();
    in.pos_fresh = FALSE;
    in.desired_user_az = 10.4;
    in.setpoint_user_az = 10.0;
    in.resend_due = FALSE;
    in.delta_backend_az = 0.1;
    in.delta_backend_el = 0.1;
    rot_cmd_decision_eval(&in, &out);
    assert(!out.send);
    expect_reason(rot_cmd_reason_name(out.reason), "no_pos");

    /* No position feedback blocks even large target changes */
    in = base_input();
    in.pos_fresh = FALSE;
    in.desired_user_az = 30.0;
    in.desired_user_el = 15.0;
    in.setpoint_user_az = 5.0;
    in.setpoint_user_el = 2.0;
    in.delta_backend_az = 20.0;
    in.delta_backend_el = 10.0;
    in.resend_due = TRUE;
    rot_cmd_decision_eval(&in, &out);
    assert(!out.send);
    expect_reason(rot_cmd_reason_name(out.reason), "no_pos");

    /* Resend safeguard */
    in = base_input();
    in.desired_user_az = 13.0;
    in.desired_user_el = 6.5;
    in.setpoint_user_az = 11.8;
    in.setpoint_user_el = 5.9;
    in.resend_due = TRUE;
    rot_cmd_decision_eval(&in, &out);
    assert(out.send);
    expect_reason(rot_cmd_reason_name(out.reason), "resend");

    /* Stale hold overrides force */
    in = base_input();
    in.stale_hold = TRUE;
    in.force_send = TRUE;
    in.desired_user_az = in.setpoint_user_az;
    in.desired_user_el = in.setpoint_user_el;
    in.delta_backend_az = 0.0;
    in.delta_backend_el = 0.0;
    rot_cmd_decision_eval(&in, &out);
    assert(!out.send);
    expect_reason(rot_cmd_reason_name(out.reason), "stale_hold");

    /* Stale hold allows send when target change is large */
    in = base_input();
    in.stale_hold = TRUE;
    in.delta_backend_az = 10.0;
    rot_cmd_decision_eval(&in, &out);
    assert(out.send);
    expect_reason(rot_cmd_reason_name(out.reason), "target_change");

    /* Force send overrides cooldown */
    in = base_input();
    in.force_send = TRUE;
    in.desired_user_az = 12.0;
    in.setpoint_user_az = 10.0;
    in.min_interval_us = 1000000;
    in.last_send_us = 900000;
    in.now_us = 950000;
    rot_cmd_decision_eval(&in, &out);
    assert(out.send);
    expect_reason(rot_cmd_reason_name(out.reason), "force");

    /* Pretrack -> tracking transition (target change beyond threshold) */
    in = base_input();
    in.desired_user_az = 20.0;
    in.desired_user_el = 5.0;
    in.setpoint_user_az = 10.0;
    in.setpoint_user_el = 5.0;
    rot_cmd_decision_eval(&in, &out);
    assert(out.send);
    expect_reason(rot_cmd_reason_name(out.reason), "target_change");

    /* Stale recovered reassert */
    in = base_input();
    in.desired_user_az = 11.4;
    in.desired_user_el = 5.4;
    in.setpoint_user_az = 10.2;
    in.setpoint_user_el = 5.1;
    in.pos_recovered = TRUE;
    rot_cmd_decision_eval(&in, &out);
    assert(out.send);
    expect_reason(rot_cmd_reason_name(out.reason), "reassert");

    /* Stopped unexpectedly */
    in = base_input();
    in.desired_user_az = 13.0;
    in.desired_user_el = 6.5;
    in.setpoint_user_az = 11.8;
    in.setpoint_user_el = 5.9;
    in.moving_toward = FALSE;
    in.stopped_unexpected = TRUE;
    rot_cmd_decision_eval(&in, &out);
    assert(out.send);
    expect_reason(rot_cmd_reason_name(out.reason), "stopped");

    /* Force send */
    in = base_input();
    in.force_send = TRUE;
    in.desired_user_az = 12.0;
    in.desired_user_el = 6.0;
    in.setpoint_user_az = 10.0;
    in.setpoint_user_el = 5.0;
    rot_cmd_decision_eval(&in, &out);
    assert(out.send);
    expect_reason(rot_cmd_reason_name(out.reason), "force");

    /* Force should not spam when target already applied */
    in = base_input();
    in.force_send = TRUE;
    in.desired_user_az = 20.0;
    in.desired_user_el = 10.0;
    in.setpoint_user_az = 10.0;
    in.setpoint_user_el = 5.0;
    rot_cmd_decision_eval(&in, &out);
    assert(out.send);
    expect_reason(rot_cmd_reason_name(out.reason), "force");

    in.setpoint_user_az = in.desired_user_az;
    in.setpoint_user_el = in.desired_user_el;
    in.force_send = TRUE;
    rot_cmd_decision_eval(&in, &out);
    assert(!out.send);
    expect_reason(rot_cmd_reason_name(out.reason), "deadband");

    return 0;
}
