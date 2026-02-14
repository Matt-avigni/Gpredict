/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include <assert.h>
#include <math.h>

#include "rotor-cmd-map.h"

static void expect_close(gdouble actual, gdouble expected)
{
    assert(fabs(actual - expected) < 1e-6);
}

int main(void)
{
    rot_cmd_map_t map = { 0 };
    rot_cmd_map_status_t st;
    rotor_conf_t conf = { 0 };

    conf.axis_mode = ROT_AXIS_MODE_AZ_EL;
    conf.aztype = ROT_AZ_TYPE_360;
    conf.minaz = 0.0;
    conf.maxaz = 360.0;
    conf.minel = 0.0;
    conf.maxel = 90.0;
    conf.use_offset = FALSE;
    conf.az_offset = 0.0;
    conf.el_offset = 0.0;
    conf.invert_az = FALSE;
    conf.invert_el = FALSE;

    st = rot_cmd_map(&conf, 370.0, 10.0, &map);
    assert(st == ROT_CMD_MAP_OK);
    expect_close(map.send_az, 10.0);
    expect_close(map.send_el, 10.0);

    conf.aztype = ROT_AZ_TYPE_180;
    conf.minaz = -180.0;
    conf.maxaz = 180.0;
    st = rot_cmd_map(&conf, 190.0, 10.0, &map);
    assert(st == ROT_CMD_MAP_OK);
    expect_close(map.send_az, -170.0);

    conf.aztype = ROT_AZ_TYPE_360;
    conf.minaz = 0.0;
    conf.maxaz = 180.0;
    st = rot_cmd_map(&conf, 200.0, 10.0, &map);
    assert(st == ROT_CMD_MAP_OUT_OF_RANGE);

    conf.minaz = 0.0;
    conf.maxaz = 360.0;
    conf.minel = 0.0;
    conf.maxel = 90.0;
    st = rot_cmd_map(&conf, 10.0, -5.0, &map);
    assert(st == ROT_CMD_MAP_OUT_OF_RANGE);

    conf.use_offset = TRUE;
    conf.az_offset = 10.0;
    st = rot_cmd_map(&conf, 350.0, 10.0, &map);
    assert(st == ROT_CMD_MAP_OK);
    expect_close(map.send_az, 0.0);

    return 0;
}
