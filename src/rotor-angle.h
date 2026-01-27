/* -*- Mode: C; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
#ifndef ROTOR_ANGLE_H
#define ROTOR_ANGLE_H 1

#include <math.h>

/* Keep azimuth math consistent across wrap boundaries. */
static inline double wrap360(double deg)
{
    double val = fmod(deg, 360.0);
    if (val < 0.0)
        val += 360.0;
    return val;
}

/* Signed delta in [-180, 180] between azimuths. */
static inline double shortest_az_delta(double a, double b)
{
    double delta = wrap360(a) - wrap360(b);
    if (delta > 180.0)
        delta -= 360.0;
    else if (delta < -180.0)
        delta += 360.0;
    return delta;
}

/* Clamp small elevations to ground. */
static inline double rotor_apply_elev_floor(double el, double floor_deg)
{
    if (floor_deg > 0.0 && el <= floor_deg)
        return 0.0;
    return el;
}

#endif
