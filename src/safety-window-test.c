#include <assert.h>
#include <math.h>

#include "safety_window.h"

static void expect_close(double actual, double expected)
{
    assert(fabs(actual - expected) < 1e-6);
}

int main(void)
{
    RotorSafety safety = { 0 };
    double cand = 0.0;

    safety.enabled = TRUE;
    safety.win_min_abs = 0.0;
    safety.win_max_abs = 360.0;
    safety.stop_enabled = FALSE;

    cand = 90.0;
    assert(safety_project_command(&safety, 80.0, &cand));
    expect_close(cand, 90.0);

    safety.win_min_abs = 0.0;
    safety.win_max_abs = 0.0;
    cand = 45.0;
    assert(!safety_project_command(&safety, 0.0, &cand));
    expect_close(cand, 0.0);

    safety.win_min_abs = 0.0;
    safety.win_max_abs = 360.0;
    safety.stop_enabled = TRUE;
    safety.stop_abs = 180.0;
    safety.stop_margin_deg = 1.0;

    cand = 360.0;
    assert(safety_project_command(&safety, 170.0, &cand));
    expect_close(cand, 0.0);

    cand = 190.0;
    assert(!safety_project_command(&safety, 170.0, &cand));
    expect_close(cand, 179.0);

    return 0;
}
