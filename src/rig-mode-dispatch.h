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
