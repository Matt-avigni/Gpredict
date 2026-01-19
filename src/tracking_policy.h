#ifndef TRACKING_POLICY_H
#define TRACKING_POLICY_H

#include <glib.h>

#include "azel_mapping.h"
#include "payload-profile.h"

typedef struct RigCaps RigCaps;
typedef struct RotCaps RotCaps;

typedef struct {
    PayloadType     payload_type;
    const RigCaps  *rig_caps;
    const RotCaps  *rot_caps;
    SpanConfig      span;
    gdouble         target_az;
    gdouble         target_el;
    gdouble         current_az;
    gdouble         current_el;
    gboolean        have_current_pos;
    gdouble         target_freq_hz;
    gdouble         current_freq_hz;
    gboolean        have_current_freq;
    guint           elapsed_ms;
    guint           max_refresh_ms;
    guint           base_update_ms;
    gint64          last_rtt_us;
    gdouble         az_deadband_deg;
    gdouble         el_deadband_deg;
    gdouble         freq_deadband_hz;
} TrackingPolicyInput;

typedef struct {
    gdouble         cmd_az;
    gdouble         cmd_el;
    gdouble         az_unwrapped;
    gboolean        az_clamped;
    gboolean        el_clamped;
    gboolean        send_rotor;
    gboolean        out_of_range;
    guint           recommended_update_ms;
    gdouble         az_deadband_deg;
    gdouble         el_deadband_deg;
    gdouble         cmd_freq_hz;
    gdouble         freq_delta_hz;
    gboolean        send_freq;
    gdouble         freq_deadband_hz;
    guint           recommended_freq_update_ms;
} TrackingPolicyOutput;

void tracking_policy_eval(const TrackingPolicyInput *in,
                          TrackingPolicyOutput *out);

typedef struct {
    double deadband_deg;
    double max_rate_deg_s;
    double max_step_deg;
    double rtt_ms;
    double smoothing_alpha_min;
    double smoothing_alpha_max;
} TrackPolicy;

typedef struct {
    double   az_abs_cmd;
    double   el_cmd;
    gboolean should_send;
    gboolean clamped;
    gboolean crossed_stop_avoided;
} TrackDecision;

TrackDecision tracking_policy_decide(const TrackPolicy *p,
                                     double now_s,
                                     double az_abs_cur,
                                     double el_cur,
                                     double az_abs_target,
                                     double el_target,
                                     double az_abs_last_cmd,
                                     double el_last_cmd);

#endif
