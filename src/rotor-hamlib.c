#include <math.h>
#include <string.h>
#include <hamlib/rotator.h>
#include "sat-log.h"
#include "rotor-conf.h"
#include "rotor-hamlib.h"

static ROT *rot = NULL;
static double az_offset = 0.0;   // mech - sky
static double az_min_mech = -180.0;
static double az_max_mech = 450.0;

/* Level 1 backend state */
static double last_cmd_az_sky = 0.0;
static double last_cmd_el_sky = 0.0;
static int    rot_initialized  = 0;
static int    last_move_ok     = 0;
static int    last_serial_err  = 0;

/* Minimum angular movement (deg) below which we do not send a new
 * command to the rotator. This avoids jitter and unnecessary wear. */
static const double MIN_MOVE_DEG = 0.2; /* ~0.2° threshold */

/*
 * Simple text status for UI / logging. Caller passes a buffer; we fill
 * it with a short human-readable summary of the last known state.
 */
void gp_hamlib_rot_status(char *buf, size_t len)
{
    if (!buf || len == 0) {
        return;
    }

    if (!rot_initialized) {
        snprintf(buf, len, "Rotator: not initialised");
        return;
    }

    if (last_serial_err) {
        snprintf(buf, len, "Rotator: SERIAL ERROR (last move failed)");
        return;
    }

    if (last_move_ok) {
        snprintf(buf, len,
                 "Rotator: OK, last cmd az=%.1f el=%.1f",
                 last_cmd_az_sky, last_cmd_el_sky);
    } else {
        snprintf(buf, len, "Rotator: idle / no recent move");
    }
}

// 1) init hamlib for given serial port + model
int gp_hamlib_rot_init(const rotor_conf_t *conf, int model)
{
    int ret;

    rig_set_debug(RIG_DEBUG_NONE);

    rot = rot_init(model);
    if (!rot) {
        sat_log_log(SAT_LOG_LEVEL_ERROR, "%s: rot_init failed\n", __func__);
        return -1;
    }

    rot->state.rotport.type.rig = RIG_PORT_SERIAL;

    // HACK for now: conf->host = "/dev/ttyUSB0", conf->port = 9600 (baud)
    strncpy(rot->state.rotport.pathname, conf->host,
            sizeof(rot->state.rotport.pathname) - 1);
    rot->state.rotport.parm.serial.rate = conf->port;

    // use caps from backend (will include your -180..450 range)
    az_min_mech = rot->caps->min_az;
    az_max_mech = rot->caps->max_az;

    ret = rot_open(rot);
    if (ret != RIG_OK) {
        sat_log_log(SAT_LOG_LEVEL_ERROR, "%s: rot_open failed (%d)\n", __func__, ret);
        rot_cleanup(rot);
        rot = NULL;
        return -1;
    }

    // TODO: compute az_offset from a sync / calibration step.
    // For now, assume mechanical zero == sky conf->minaz.
    az_offset = 0.0;

    rot_initialized = 1;
    last_move_ok    = 0;
    last_serial_err = 0;

    return 0;
}

// core mapping: sky az -> mechanical az with offset + 420° handling
static double sky_to_mech_az(double az_sky_deg, const rotor_conf_t *conf, double az_mech_current)
{
    // first, keep sky az in 0..360
    double az_sky = wrap_range(az_sky_deg, 0.0, 360.0);

    // candidate mechanical positions (360° multiples)
    double best = 0.0;
    double best_err = 1e9;
    for (int k = -1; k <= 1; ++k) {
        double m = az_sky + az_offset + 360.0 * k;
        m = wrap_range(m, az_min_mech, az_max_mech);

        // optional: avoid cable limits using conf->minaz/maxaz/azstoppos

        double err = fabs(m - az_mech_current);
        if (err < best_err) {
            best = m;
            best_err = err;
        }
    }
    return best;
}

// 2) set position: this is what Gpredict calls from its tracking loop
int gp_hamlib_rot_set_azel(double az_sky, double el_sky, const rotor_conf_t *conf)
{
    if (!rot) return -1;

    /* Ignore very small requested movements to avoid jitter. */
    if (rot_initialized &&
        fabs(az_sky - last_cmd_az_sky) < MIN_MOVE_DEG &&
        fabs(el_sky - last_cmd_el_sky) < MIN_MOVE_DEG) {
        return 0; /* treat as success, no new command sent */
    }

    double cur_az_mech = 0.0, cur_el_mech = 0.0;
    int ret = rot_get_position(rot, &cur_az_mech, &cur_el_mech);
    if (ret != RIG_OK) {
        sat_log_log(SAT_LOG_LEVEL_WARN, "%s: rot_get_position failed (%d)\n", __func__, ret);
        // still try to move with some default assumption if you want
    }

    double az_mech = sky_to_mech_az(az_sky, conf, cur_az_mech);

    // clamp elevation according to conf / caps
    double el_mech = el_sky;
    if (el_mech < conf->minel) el_mech = conf->minel;
    if (el_mech > conf->maxel) el_mech = conf->maxel;

    ret = rot_set_position(rot, az_mech, el_mech);
    if (ret != RIG_OK) {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    "%s: rot_set_position failed (%d)\n", __func__, ret);
        last_move_ok    = 0;
        last_serial_err = 1;
        return -1;
    }

    /* Update status on success */
    last_cmd_az_sky = az_sky;
    last_cmd_el_sky = el_sky;
    last_move_ok    = 1;
    last_serial_err = 0;

    return 0;
}

// 3) optional: read back position in sky frame
int gp_hamlib_rot_get_azel(double *az_sky, double *el_sky, const rotor_conf_t *conf)
{
    if (!rot) return -1;

    double az_mech = 0.0, el_mech = 0.0;
    int ret = rot_get_position(rot, &az_mech, &el_mech);
    if (ret != RIG_OK) return -1;

    // invert the mapping, simplest: just subtract offset and wrap 0..360
    double az = az_mech - az_offset;
    az = wrap_range(az, 0.0, 360.0);

    *az_sky = az;
    *el_sky = el_mech; // if no mechanical offset in elevation

    return 0;
}

void gp_hamlib_rot_close(void)
{
    if (rot) {
        rot_close(rot);
        rot_cleanup(rot);
        rot = NULL;
    }
    rot_initialized = 0;
    last_move_ok    = 0;
    last_serial_err = 0;
}