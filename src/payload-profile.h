/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#ifndef PAYLOAD_PROFILE_H
#define PAYLOAD_PROFILE_H

#include <glib.h>

typedef enum {
    PAYLOAD_UNKNOWN = 0,
    PAYLOAD_FM,
    PAYLOAD_LINEAR_SSB,
    PAYLOAD_BEACON,
    PAYLOAD_DIGITAL,
    PAYLOAD_TRANSPONDER_LINEAR,
    PAYLOAD_CROSSBAND_REPEATER,
    PAYLOAD_FIXED_BEACON
} PayloadType;

typedef struct {
    PayloadType type;
    gboolean apply_doppler_downlink;
    gboolean apply_doppler_uplink;
    gboolean invert_uplink_sign;
    gdouble downlink_if_offset_hz;
    gdouble uplink_if_offset_hz;
    gdouble deadband_hz;
    gdouble ema_alpha;
    gint calc_hz;
    gint send_hz;
    gboolean enable_transponder_map;
} PayloadProfile;

PayloadProfile payload_profile_from_menu_value(const gchar *menu_value);
const gchar *payload_type_name(PayloadType type);

gboolean compute_rig_frequencies(const PayloadProfile *profile,
                                 gdouble base_downlink_hz,
                                 gdouble base_uplink_hz,
                                 gdouble range_rate_mps,
                                 gdouble *cmd_downlink_hz,
                                 gdouble *cmd_uplink_hz);

#endif
