/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#ifndef RIG_MODE_DISPATCH_H
#define RIG_MODE_DISPATCH_H 1

#include "radio-conf.h"

typedef struct {
    gboolean send_downlink;
    gboolean send_uplink;
    vfo_t    downlink_vfo;
    vfo_t    uplink_vfo;
} rig_mode_dispatch_t;

void rig_mode_dispatch(radio_mode_t mode,
                       vfo_t downlink_vfo,
                       vfo_t uplink_vfo,
                       rig_mode_dispatch_t *plan);

#endif
