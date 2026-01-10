#include "rig-mode-dispatch.h"

void rig_mode_dispatch(radio_mode_t mode,
                       vfo_t downlink_vfo,
                       vfo_t uplink_vfo,
                       rig_mode_dispatch_t *plan)
{
    if (plan == NULL)
        return;

    plan->send_downlink = FALSE;
    plan->send_uplink = FALSE;
    plan->downlink_vfo = downlink_vfo;
    plan->uplink_vfo = uplink_vfo;

    switch (mode)
    {
    case RADIO_MODE_FULL_DUPLEX_MAIN_SUB:
        plan->send_downlink = TRUE;
        plan->send_uplink = TRUE;
        break;
    case RADIO_MODE_SPLIT:
        plan->send_downlink = TRUE;
        plan->send_uplink = TRUE;
        break;
    case RADIO_MODE_SIMPLEX:
    default:
        plan->send_downlink = TRUE;
        plan->send_uplink = FALSE;
        break;
    }
}
