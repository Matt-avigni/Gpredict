#ifndef SAFETY_WINDOW_H
#define SAFETY_WINDOW_H

#include <glib.h>

typedef struct {
    gboolean enabled;
    double   win_min_abs;
    double   win_max_abs;

    gboolean stop_enabled;
    double   stop_abs;
    double   stop_margin_deg;
} RotorSafety;

gboolean safety_project_command(const RotorSafety *s,
                                double az_abs_cur,
                                double *az_abs_candidate_in_out);

#endif
