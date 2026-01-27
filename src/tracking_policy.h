#ifndef TRACKING_POLICY_H
#define TRACKING_POLICY_H

#include <glib.h>

#include "azel_mapping.h"
#include "payload-profile.h"
#include "rotor-target.h"

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

typedef enum {
    TRACKING_WRAP_CONTINUOUS = 0,
    TRACKING_WRAP_NORTH_CENTERED = 1
} tracking_wrap_mode_t;

typedef enum {
    TRACKING_GEOM_NORMAL = 0,
    TRACKING_GEOM_FLIPPED = 1
} tracking_geom_mode_t;

typedef struct {
    tracking_wrap_mode_t wrap_mode;
    tracking_geom_mode_t geom_mode;
    gdouble az_min;
    gdouble az_max;
    gdouble el_min;
    gdouble el_max;
    gdouble seam;
    gint    preferred_k;
    gdouble last_cmd_az;
    gboolean last_cmd_valid;
    gboolean chosen;
    gboolean degraded;
    gboolean logged_no_branch;
    gboolean logged_empty_set;
    gdouble window_start;
    gdouble window_end;
} RotTrackingPolicy;

void rot_tracking_policy_reset(RotTrackingPolicy *p);
gboolean rot_tracking_policy_choose(RotTrackingPolicy *p,
                                    tracking_wrap_mode_t wrap_mode,
                                    gdouble az_min,
                                    gdouble az_max,
                                    gdouble el_min,
                                    gdouble el_max,
                                    gboolean allow_flip,
                                    const rot_target_sample_t *samples,
                                    gsize sample_count,
                                    gboolean *used_flip_out,
                                    gboolean *degraded_out);
gboolean rot_tracking_policy_select_cmd(const RotTrackingPolicy *p,
                                        gdouble az_pred,
                                        gdouble el_pred,
                                        gboolean have_measured,
                                        gdouble measured_az,
                                        gdouble last_cmd_az,
                                        gdouble *az_cmd_out,
                                        gdouble *el_cmd_out,
                                        gboolean *empty_set_out);

#endif
