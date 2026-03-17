/* -*- Mode: C; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/*
    Gpredict: Real-time satellite tracking and orbit prediction program

    Copyright (C)  2001-2009  Alexandru Csete.

    Authors: Alexandru Csete <oz9aec@gmail.com>

    Comments, questions and bugreports should be submitted via
    http://sourceforge.net/projects/gpredict/
    More details can be found at the project home page:

            http://gpredict.oz9aec.net/
 
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
  
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
  
    You should have received a copy of the GNU General Public License
    along with this program; if not, visit http://www.fsf.org/
 

*/
#ifndef ROTOR_CONF_H
#define ROTOR_CONF_H 1

#include <glib.h>


typedef enum {
    ROT_AZ_TYPE_360 = 0,        /*!< Azimuth in range 0..360 */
    ROT_AZ_TYPE_180 = 1,        /*!< Azimuth in range -180..+180 */
    ROT_AZ_TYPE_480 = 2         /*!< Azimuth in range 0..480 */
} rot_az_type_t;

typedef enum {
    ROT_AXIS_MODE_AZ_EL = 0,
    ROT_AXIS_MODE_AZ_ONLY = 1
} rot_axis_mode_t;

typedef enum {
    ROT_PROTOCOL_GS232B = 0,
    ROT_PROTOCOL_SPID_ROT1PROG = 1,
    ROT_PROTOCOL_SPID_ROT2PROG = 2,
    ROT_PROTOCOL_OTHER = 3
} rot_protocol_t;

/** \brief Rotator configuration. */
typedef struct {
    gchar          *name;       /*!< Configuration file name, less .rot */
    gchar          *host;       /*!< hostname */
    gint            port;       /*!< port number */
    rot_protocol_t  protocol;   /*!< Rotator protocol selection */
    gint            hamlib_model; /*!< Hamlib model ID (used when protocol=Other) */
    gint            baud;       /*!< Serial baud rate */
    gchar          *device;     /*!< Selected serial device */
    gchar          *device_manual; /*!< Manual device override */
    gboolean        device_autopick; /*!< Auto-pick best serial device */
    gboolean        autostart;  /*!< Auto-start rotctld */
    gint            cycle;      /*!< cycle period in msec */
    rot_az_type_t   aztype;     /*!< Az type */
    /* Developer note: we do not infer rotor axis mode; config is source of truth. */
    rot_axis_mode_t axis_mode; /*!< Axis mode */
    gdouble         minaz;      /*!< Lower azimuth limit */
    gdouble         maxaz;      /*!< Upper azimuth limit */
    gdouble         minel;      /*!< Lower elevation limit */
    gdouble         maxel;      /*!< Upper elevation limit */
    gboolean        el_overtravel_enable; /*!< Enable custom elevation clamp for command output */
    gdouble         el_min_deg; /*!< Custom overtravel minimum elevation (degrees) */
    gdouble         el_max_deg; /*!< Custom overtravel maximum elevation (degrees) */
    gdouble         azstoppos;  /*!< absolute position of rotation stops; normally = minaz */
    gdouble         threshold;  /*!< Angle difference that triggers new motion command */
    gint            rotor_poll_period_ms; /*!< Poll interval for rotctld get_position */
    gint            rotor_position_stale_ms; /*!< Stale threshold for position age */
    guint           rotor_stale_debounce_count; /*!< Consecutive stale checks before degrade */
    gdouble         rotor_angle_epsilon_deg; /*!< Angle comparison tolerance */
    gdouble         rotor_elev_floor_deg; /*!< Elevation treated as ground */
    gboolean        use_offset; /*!< Apply configured az/el offsets (zenith guard is handled at runtime) */
    gdouble         az_offset;  /*!< Azimuth offset (degrees) */
    gdouble         el_offset;  /*!< Elevation offset (degrees) */
    gboolean        invert_az;  /*!< Invert azimuth axis */
    gboolean        invert_el;  /*!< Invert elevation axis */
    gdouble         pretrack_seconds; /*!< Slew-to-AOS lookahead window (seconds) */
    gboolean        slew_to_aos_while_below_horizon; /*!< Allow pretrack while below horizon */
    gboolean        pretrack_immediate; /*!< Enter pretrack immediately on Track */
    gdouble         pretrack_min_el; /*!< Elevation used during pretrack (degrees) */
    gint            rotor_stale_warn_ms; /*!< Warning threshold for stale position */
    gint            rotor_stale_degraded_ms; /*!< Degraded threshold for stale position */
    gint            rotor_stale_hold_ms; /*!< Hold threshold for stale position */
    gint            rotor_stale_park_ms; /*!< Park/disconnect threshold for stale position */
    gint            rotor_stale_resume_ms; /*!< Fresh period required to resume after hold */
    gboolean        disable_pos_feedback_checks; /*!< Disable position/encoder feedback checks */
    gchar          *last_good_device; /*!< Last validated serial device */
    gint            last_good_baud; /*!< Last validated baud rate */
} rotor_conf_t;

static inline void rot_conf_get_default_az_limits(rot_az_type_t aztype,
                                                  gdouble *minaz_out,
                                                  gdouble *maxaz_out,
                                                  gdouble *azstoppos_out)
{
    gdouble minaz = 0.0;
    gdouble maxaz = 360.0;
    gdouble azstoppos = 0.0;

    switch (aztype)
    {
    case ROT_AZ_TYPE_180:
        minaz = -180.0;
        maxaz = 180.0;
        azstoppos = -180.0;
        break;

    case ROT_AZ_TYPE_480:
        minaz = 0.0;
        maxaz = 480.0;
        azstoppos = 0.0;
        break;

    case ROT_AZ_TYPE_360:
    default:
        break;
    }

    if (minaz_out)
        *minaz_out = minaz;
    if (maxaz_out)
        *maxaz_out = maxaz;
    if (azstoppos_out)
        *azstoppos_out = azstoppos;
}

static inline void rot_conf_apply_default_az_limits(rotor_conf_t *conf,
                                                    gboolean reset_endstop)
{
    gdouble minaz = 0.0;
    gdouble maxaz = 360.0;
    gdouble azstoppos = 0.0;

    if (conf == NULL)
        return;

    rot_conf_get_default_az_limits(conf->aztype,
                                   &minaz,
                                   &maxaz,
                                   &azstoppos);

    conf->minaz = minaz;
    conf->maxaz = maxaz;

    if (reset_endstop ||
        conf->azstoppos < minaz ||
        conf->azstoppos > maxaz)
    {
        conf->azstoppos = azstoppos;
    }
}


gboolean        rotor_conf_read(rotor_conf_t * conf);
void            rotor_conf_save(rotor_conf_t * conf);
gboolean        rot_protocol_is_valid(rot_protocol_t protocol);
const gchar    *rot_protocol_name(rot_protocol_t protocol);
const gchar    *rot_protocol_model_name(rot_protocol_t protocol);
gint            rot_protocol_to_hamlib_model(rot_protocol_t protocol);
gint            rot_protocol_default_baud(rot_protocol_t protocol);
gint            rot_conf_hamlib_model(const rotor_conf_t *conf);

#endif
