/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include <assert.h>
#include <math.h>

#include "rotor-angle.h"
#include "rotor_calibration.h"

static void expect_close(double actual, double expected)
{
    assert(fabs(actual - expected) < 1e-6);
}

int main(void)
{
    RotorCalib c = { 0 };

    calib_apply_mech_zero(&c, 10.0, -5.0, 0.7);
    expect_close(c.az_offset_deg, wrap360(-10.0));
    expect_close(c.el_offset_deg, 5.0);
    expect_close(c.az_uncertainty_deg, 0.7);
    expect_close(c.el_uncertainty_deg, 0.7);
    assert(c.enabled);

    c = (RotorCalib){ 0 };
    calib_apply_mech_zero(&c, 0.0, 0.0, 0.0);
    assert(c.az_uncertainty_deg > 0.0);
    assert(c.el_uncertainty_deg > 0.0);

    return 0;
}
