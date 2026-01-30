#include <assert.h>
#include <math.h>

#include "rot_math.h"

static void expect_close(gdouble actual, gdouble expected)
{
    assert(fabs(actual - expected) < 1e-6);
}

int main(void)
{
    RotLaneSelectResult out = { 0 };
    gdouble ui = rot_az360_to_ui(309.0, ROT_UI_NORTH_CENTERED);

    /* UI mapping should show -51 for a 309 azimuth in north-centered mode. */
    expect_close(ui, -51.0);

    /* Backend 0..360 should choose 309, not freeze at 120. */
    assert(rot_backend_lane_select(309.0,
                                   0.0,
                                   360.0,
                                   120.0,
                                   TRUE,
                                   0.0,
                                   FALSE,
                                   0.0,
                                   &out));
    expect_close(out.az_cmd_backend, 309.0);
    assert(fabs(out.az_cmd_backend - 120.0) > 1e-6);

    /* Backend -180..180 should choose the equivalent -51 lane. */
    assert(rot_backend_lane_select(309.0,
                                   -180.0,
                                   180.0,
                                   120.0,
                                   TRUE,
                                   0.0,
                                   FALSE,
                                   0.0,
                                   &out));
    expect_close(out.az_cmd_backend, -51.0);

    return 0;
}
