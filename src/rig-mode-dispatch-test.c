/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include <assert.h>

#include "rig-mode-dispatch.h"

static void expect_full_duplex(vfo_t downlink_vfo, vfo_t uplink_vfo)
{
    rig_mode_dispatch_t plan;

    rig_mode_dispatch(RADIO_MODE_FULL_DUPLEX_MAIN_SUB,
                      downlink_vfo,
                      uplink_vfo,
                      &plan);

    assert(plan.send_downlink);
    assert(plan.send_uplink);
    assert(plan.downlink_vfo == downlink_vfo);
    assert(plan.uplink_vfo == uplink_vfo);
}

int main(void)
{
    expect_full_duplex(VFO_MAIN, VFO_SUB);
    expect_full_duplex(VFO_A, VFO_B);

    return 0;
}
