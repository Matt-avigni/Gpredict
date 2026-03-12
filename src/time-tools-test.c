/*
 * Copyright (C) 2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include <math.h>

#include <glib.h>

#include "time-tools.h"

static gint failures = 0;

static void expect_true(gboolean cond, const gchar *msg)
{
    if (cond)
        return;

    g_printerr("FAIL: %s\n", msg);
    failures++;
}

static void expect_str_eq(const gchar *got,
                          const gchar *expected,
                          const gchar *msg)
{
    if (g_strcmp0(got, expected) == 0)
        return;

    g_printerr("FAIL: %s (got=%s expected=%s)\n",
               msg,
               got ? got : "(null)",
               expected ? expected : "(null)");
    failures++;
}

static void test_valid_utc_format(void)
{
    gchar buf[32] = { 0 };
    gint len = daynum_to_utc_str(buf, sizeof(buf),
                                 "%Y-%m-%d %H:%M:%S",
                                 2440587.5);

    expect_true(len > 0, "utc format returns data for Unix epoch");
    expect_str_eq(buf, "1970-01-01 00:00:00",
                  "utc format prints Unix epoch");
}

static void test_invalid_julian_dates(void)
{
    gchar buf[32] = "seed";

    expect_true(daynum_to_utc_str(buf, sizeof(buf), "%F %T", NAN) == 0,
                "utc format rejects NaN");
    expect_str_eq(buf, "", "utc format clears buffer for NaN");

    g_strlcpy(buf, "seed", sizeof(buf));
    expect_true(daynum_to_utc_str(buf, sizeof(buf), "%F %T", INFINITY) == 0,
                "utc format rejects infinity");
    expect_str_eq(buf, "", "utc format clears buffer for infinity");

    g_strlcpy(buf, "seed", sizeof(buf));
    expect_true(daynum_to_utc_str(buf, sizeof(buf), "%F %T", 1.0e20) == 0,
                "utc format rejects out-of-range Julian dates");
    expect_str_eq(buf, "", "utc format clears buffer for out-of-range dates");
}

int main(void)
{
    test_valid_utc_format();
    test_invalid_julian_dates();

    if (failures != 0)
        return 1;

    g_print("time-tools-test: ok\n");
    return 0;
}
