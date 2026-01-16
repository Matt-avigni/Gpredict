#include "payload-profile.h"

#include <math.h>
#include <string.h>

static PayloadProfile payload_profile_default(void)
{
    PayloadProfile p;

    p.type = PAYLOAD_UNKNOWN;
    p.apply_doppler_downlink = TRUE;
    p.apply_doppler_uplink = FALSE;
    p.invert_uplink_sign = FALSE;
    p.downlink_if_offset_hz = 0.0;
    p.uplink_if_offset_hz = 0.0;
    p.deadband_hz = 1.0;
    p.ema_alpha = 0.2;
    p.calc_hz = 10;
    p.send_hz = 5;
    p.enable_transponder_map = FALSE;

    return p;
}

static gboolean payload_str_has(const gchar *haystack, const gchar *needle)
{
    if (haystack == NULL || needle == NULL)
        return FALSE;
    return (g_strrstr(haystack, needle) != NULL);
}

const gchar *payload_type_name(PayloadType type)
{
    switch (type)
    {
    case PAYLOAD_FM:
        return "FM";
    case PAYLOAD_LINEAR_SSB:
        return "LINEAR_SSB";
    case PAYLOAD_BEACON:
        return "BEACON";
    case PAYLOAD_DIGITAL:
        return "DIGITAL";
    case PAYLOAD_TRANSPONDER_LINEAR:
        return "TRANSPONDER_LINEAR";
    case PAYLOAD_CROSSBAND_REPEATER:
        return "CROSSBAND_REPEATER";
    case PAYLOAD_FIXED_BEACON:
        return "FIXED_BEACON";
    default:
        return "UNKNOWN";
    }
}

PayloadProfile payload_profile_from_menu_value(const gchar *menu_value)
{
    PayloadProfile p = payload_profile_default();

    if (menu_value == NULL || *menu_value == '\0')
        return p;

    gchar *lower = g_ascii_strdown(menu_value, -1);

    if (payload_str_has(lower, "fixed") ||
        payload_str_has(lower, "no doppler") ||
        payload_str_has(lower, "nodoppler"))
    {
        p.type = PAYLOAD_FIXED_BEACON;
        p.apply_doppler_downlink = FALSE;
        p.apply_doppler_uplink = FALSE;
        p.deadband_hz = 5.0;
        p.send_hz = 2;
    }
    else if (payload_str_has(lower, "linear") ||
             payload_str_has(lower, "transponder") ||
             payload_str_has(lower, "ssb") ||
             payload_str_has(lower, "usb") ||
             payload_str_has(lower, "lsb"))
    {
        p.type = PAYLOAD_TRANSPONDER_LINEAR;
        p.apply_doppler_downlink = TRUE;
        p.apply_doppler_uplink = TRUE;
        p.deadband_hz = 1.0;
        p.ema_alpha = 0.15;
        p.send_hz = 5;
        p.enable_transponder_map = TRUE;
    }
    else if (payload_str_has(lower, "crossband") ||
             payload_str_has(lower, "repeater"))
    {
        p.type = PAYLOAD_CROSSBAND_REPEATER;
        p.apply_doppler_downlink = TRUE;
        p.apply_doppler_uplink = TRUE;
        p.deadband_hz = 2.0;
        p.ema_alpha = 0.2;
        p.send_hz = 5;
    }
    else if (payload_str_has(lower, "beacon") ||
             payload_str_has(lower, "bcn"))
    {
        p.type = PAYLOAD_BEACON;
        p.apply_doppler_downlink = TRUE;
        p.apply_doppler_uplink = FALSE;
        p.deadband_hz = 2.0;
        p.send_hz = 2;
    }
    else if (payload_str_has(lower, "fm") ||
             payload_str_has(lower, "fmn") ||
             payload_str_has(lower, "nfm") ||
             payload_str_has(lower, "am"))
    {
        p.type = PAYLOAD_FM;
        p.apply_doppler_downlink = TRUE;
        p.apply_doppler_uplink = FALSE;
        p.deadband_hz = 5.0;
        p.ema_alpha = 0.3;
        p.send_hz = 2;
    }
    else if (payload_str_has(lower, "bpsk") ||
             payload_str_has(lower, "gmsk") ||
             payload_str_has(lower, "afsk") ||
             payload_str_has(lower, "fsk") ||
             payload_str_has(lower, "digital") ||
             payload_str_has(lower, "packet"))
    {
        p.type = PAYLOAD_DIGITAL;
        p.apply_doppler_downlink = TRUE;
        p.apply_doppler_uplink = FALSE;
        p.deadband_hz = 5.0;
        p.ema_alpha = 0.25;
        p.send_hz = 2;
    }
    else if (payload_str_has(lower, "cw"))
    {
        p.type = PAYLOAD_LINEAR_SSB;
        p.apply_doppler_downlink = TRUE;
        p.apply_doppler_uplink = FALSE;
        p.deadband_hz = 2.0;
        p.ema_alpha = 0.2;
        p.send_hz = 3;
    }

    g_free(lower);
    return p;
}

gboolean compute_rig_frequencies(const PayloadProfile *profile,
                                 gdouble base_downlink_hz,
                                 gdouble base_uplink_hz,
                                 gdouble range_rate_mps,
                                 gdouble *cmd_downlink_hz,
                                 gdouble *cmd_uplink_hz)
{
    if (profile == NULL)
        return FALSE;

    const gdouble c_mps = 299792458.0;

    gdouble down = base_downlink_hz;
    gdouble up = base_uplink_hz;

    /* Non-relativistic Doppler: df = (v/c)*f with sign by link direction. */
    if (profile->apply_doppler_downlink)
        down += -(range_rate_mps / c_mps) * base_downlink_hz;
    if (profile->apply_doppler_uplink)
    {
        gdouble du = (range_rate_mps / c_mps) * base_uplink_hz;
        if (profile->invert_uplink_sign)
            du = -du;
        up += du;
    }

    down += profile->downlink_if_offset_hz;
    up += profile->uplink_if_offset_hz;

    if (cmd_downlink_hz)
        *cmd_downlink_hz = down;
    if (cmd_uplink_hz)
        *cmd_uplink_hz = up;

    return TRUE;
}
