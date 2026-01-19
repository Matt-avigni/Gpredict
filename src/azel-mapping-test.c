#include <assert.h>
#include <math.h>

#include "azel_mapping.h"

static void expect_close(gdouble actual, gdouble expected)
{
    assert(fabs(actual - expected) < 1e-6);
}

int main(void)
{
    SpanConfig cfg = { 0 };
    SpanConfig cfg180 = { 0 };
    AzElMapResult out = { 0 };

    cfg.az_mode = AZ_MODE_0_360;
    cfg.az_min = 0.0;
    cfg.az_max = 360.0;
    cfg.el_min = 0.0;
    cfg.el_max = 90.0;
    cfg.prefer_shortest_path = FALSE;
    cfg.allow_wrap = FALSE;
    cfg.treat_360_as_0 = TRUE;

    assert(azel_map(&cfg, 1.0, 10.0, 359.0, TRUE, &out));
    expect_close(out.cmd_az, 1.0);
    expect_close(out.cmd_el, 10.0);

    assert(azel_map(&cfg, 360.0, 10.0, 0.0, TRUE, &out));
    expect_close(out.cmd_az, 0.0);

    cfg180.az_mode = AZ_MODE_NEG180_POS180;
    cfg180.az_min = -180.0;
    cfg180.az_max = 180.0;
    cfg180.el_min = 0.0;
    cfg180.el_max = 90.0;
    cfg180.prefer_shortest_path = TRUE;
    cfg180.allow_wrap = TRUE;
    cfg180.treat_360_as_0 = FALSE;

    assert(azel_map(&cfg180, -179.0, 5.0, 179.0, TRUE, &out));
    expect_close(out.cmd_az, -179.0);

    assert(azel_map(&cfg, 20.0, 100.0, 10.0, TRUE, &out));
    expect_close(out.cmd_el, 90.0);
    assert(out.el_clamped);

    expect_close(az_norm_span(181.0, AZSPAN_PM180), -179.0);
    expect_close(az_norm_span(-181.0, AZSPAN_PM180), 179.0);
    expect_close(az_unwrap_to_abs(179.0, -179.0, AZSPAN_PM180), 181.0);
    expect_close(az_abs_to_span(181.0, AZSPAN_PM180), -179.0);

    expect_close(az_norm_span(-1.0, AZSPAN_360), 359.0);
    expect_close(az_unwrap_to_abs(359.0, 1.0, AZSPAN_360), 361.0);

    expect_close(az_norm_span(490.0, AZSPAN_480), 10.0);
    expect_close(az_unwrap_to_abs(470.0, 10.0, AZSPAN_480), 490.0);
    expect_close(az_abs_to_span(480.0, AZSPAN_480), 480.0);

    expect_close(az_target_to_nearest_abs(350.0, 10.0, AZSPAN_360), 370.0);
    expect_close(az_target_to_nearest_abs(10.0, 350.0, AZSPAN_360), -10.0);

    return 0;
}
