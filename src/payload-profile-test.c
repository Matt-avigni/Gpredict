/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include <assert.h>
#include <math.h>

#include "payload-profile.h"

static void expect_no_doppler_shift(void)
{
    PayloadProfile profile = payload_profile_from_menu_value("fixed");
    gdouble base_down = 145800000.0;
    gdouble base_up = 435000000.0;
    gdouble out_down = 0.0;
    gdouble out_up = 0.0;

    profile.downlink_if_offset_hz = 0.0;
    profile.uplink_if_offset_hz = 0.0;

    assert(compute_rig_frequencies(&profile, base_down, base_up, 1000.0,
                                   &out_down, &out_up));
    assert(fabs(out_down - base_down) < 0.5);
    assert(fabs(out_up - base_up) < 0.5);
}

static void expect_doppler_shift(void)
{
    PayloadProfile profile = payload_profile_from_menu_value("linear");
    gdouble base_down = 145800000.0;
    gdouble base_up = 435000000.0;
    gdouble out_down = 0.0;
    gdouble out_up = 0.0;

    assert(compute_rig_frequencies(&profile, base_down, base_up, 1000.0,
                                   &out_down, &out_up));
    assert(fabs(out_down - base_down) > 0.5);
    assert(fabs(out_up - base_up) > 0.5);
}

int main(void)
{
    expect_no_doppler_shift();
    expect_doppler_shift();
    return 0;
}
