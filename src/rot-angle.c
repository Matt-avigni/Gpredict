#include "rot-angle.h"

#include <math.h>

static gboolean gp_env_enabled(const gchar *name)
{
    const gchar *val = g_getenv(name);
    if (!val || !*val)
        return FALSE;
    return (val[0] == '1' || val[0] == 'y' || val[0] == 'Y');
}

double gp_norm360(double az)
{
    double v = fmod(az, 360.0);
    if (v < 0.0)
        v += 360.0;
    if (v >= 360.0)
        v -= 360.0;
    return v;
}

double gp_norm180(double az)
{
    double v = gp_norm360(az);
    if (v > 180.0)
        v -= 360.0;
    return v;
}

double gp_shortest_delta360(double from_az360, double to_az360)
{
    double a = gp_norm360(from_az360);
    double b = gp_norm360(to_az360);
    double d = b - a;
    if (d > 180.0)
        d -= 360.0;
    else if (d <= -180.0)
        d += 360.0;
    return d;
}

double gp_clamp(double v, double lo, double hi)
{
    if (v < lo)
        return lo;
    if (v > hi)
        return hi;
    return v;
}

double gp_backend_to_az360(double az_backend)
{
    return gp_norm360(az_backend);
}

double gp_az360_to_backend_near(double az360,
                                double ref_backend,
                                double min_az,
                                double max_az,
                                int *out_k)
{
    double az_norm = gp_norm360(az360);
    double best = az_norm;
    double best_diff = INFINITY;
    gboolean found = FALSE;

    if (min_az > max_az)
    {
        double tmp = min_az;
        min_az = max_az;
        max_az = tmp;
    }

    int k_min = (int)ceil((min_az - az_norm) / 360.0 - 1e-9);
    int k_max = (int)floor((max_az - az_norm) / 360.0 + 1e-9);

    for (int k = k_min; k <= k_max; k++)
    {
        double cand = az_norm + (360.0 * k);
        if (cand < min_az - 1e-9 || cand > max_az + 1e-9)
            continue;
        double diff = fabs(cand - ref_backend);
        if (!found || diff < best_diff)
        {
            best = cand;
            best_diff = diff;
            found = TRUE;
            if (out_k)
                *out_k = k;
        }
    }

    if (found)
        return best;

    if (fabs(ref_backend - min_az) <= fabs(ref_backend - max_az))
    {
        if (out_k)
            *out_k = (int)lrint((min_az - az_norm) / 360.0);
        return min_az;
    }

    if (out_k)
        *out_k = (int)lrint((max_az - az_norm) / 360.0);
    return max_az;
}

void gp_rot_angle_selfcheck(void)
{
    static gsize once = 0;

    if (!g_once_init_enter(&once))
        return;

    if (gp_env_enabled("GP_DEBUG_ROT_ANGLE"))
    {
        g_assert(fabs(gp_backend_to_az360(360.0) - 0.0) < 1e-9);
        g_assert(fabs(gp_backend_to_az360(-2.0) - 358.0) < 1e-9);
        g_assert(fabs(gp_shortest_delta360(358.0, 2.0) - 4.0) < 1e-9);
        g_assert(fabs(gp_norm180(358.0) - (-2.0)) < 1e-9);
        g_assert(fabs(gp_norm360(-720.0) - 0.0) < 1e-9);
        g_assert(fabs(gp_clamp(5.0, 0.0, 3.0) - 3.0) < 1e-9);
    }

    g_once_init_leave(&once, 1);
}
