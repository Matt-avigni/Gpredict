/*
    Gpredict: Real-time satellite tracking and orbit prediction program

    Copyright (C)  2001-2009  Alexandru Csete, OZ9AEC.

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
#ifndef SAT_PREF_ROT_DATA_H
#define SAT_PREF_ROT_DATA_H 1

/** Coumn definitions for rotator list. */
typedef enum {
    ROT_LIST_COL_NAME = 0,      /*!< File name. */
    ROT_LIST_COL_HOST,          /*!< Hostname */
    ROT_LIST_COL_PORT,          /*!< Port number */
    ROT_LIST_COL_PROTOCOL,      /*!< Rotator protocol */
    ROT_LIST_COL_BAUD,          /*!< Baud rate */
    ROT_LIST_COL_DEVICE,        /*!< Serial device */
    ROT_LIST_COL_DEVICE_MANUAL, /*!< Manual device override */
    ROT_LIST_COL_DEVICE_AUTOPICK, /*!< Auto-pick device */
    ROT_LIST_COL_AUTOSTART,     /*!< Auto-start rotctld */
    ROT_LIST_COL_CYCLE,         /*!< Cycle period in milliseconds. */
    ROT_LIST_COL_MINAZ,         /*!< Lower Az limit. */
    ROT_LIST_COL_MAXAZ,         /*!< Upper Az limit. */
    ROT_LIST_COL_MINEL,         /*!< Lower El limit. */
    ROT_LIST_COL_MAXEL,         /*!< Upper El limit. */
    ROT_LIST_COL_AZTYPE,        /*!< Azimuth type. */
    ROT_LIST_COL_AZSTOPPOS,     /*!< Position of the azimuth rotation stops.
                                   Should default to MINAZ, unless specified
                                   otherwise */
    ROT_LIST_COL_AXIS_MODE,     /*!< Axis mode (AZ only vs AZ/EL). */
    ROT_LIST_COL_USE_OFFSET,    /*!< Apply configured offsets. */
    ROT_LIST_COL_AZ_OFFSET,     /*!< Az offset. */
    ROT_LIST_COL_EL_OFFSET,     /*!< El offset. */
    ROT_LIST_COL_AZ_INVERT,     /*!< Invert azimuth. */
    ROT_LIST_COL_EL_INVERT,     /*!< Invert elevation. */
    ROT_LIST_COL_POLL_PERIOD_MS, /*!< Poll period in milliseconds. */
    ROT_LIST_COL_POS_STALE_MS, /*!< Position stale threshold in milliseconds. */
    ROT_LIST_COL_STALE_DEBOUNCE, /*!< Stale debounce count. */
    ROT_LIST_COL_STALE_WARN_MS, /*!< Stale warning threshold in milliseconds. */
    ROT_LIST_COL_STALE_DEGRADED_MS, /*!< Stale degraded threshold in milliseconds. */
    ROT_LIST_COL_STALE_HOLD_MS, /*!< Stale hold threshold in milliseconds. */
    ROT_LIST_COL_STALE_PARK_MS, /*!< Stale park threshold in milliseconds. */
    ROT_LIST_COL_STALE_RESUME_MS, /*!< Stale resume threshold in milliseconds. */
    ROT_LIST_COL_THRESHOLD,     /*!< Tracking threshold in degrees. */
    ROT_LIST_COL_ANGLE_EPSILON, /*!< Angle epsilon in degrees. */
    ROT_LIST_COL_ELEV_FLOOR,   /*!< Elevation floor in degrees. */
    ROT_LIST_COL_PRETRACK_SECONDS, /*!< Pretrack lookahead in seconds. */
    ROT_LIST_COL_PRETRACK_SLEW, /*!< Allow pretrack below horizon. */
    ROT_LIST_COL_PRETRACK_IMMEDIATE, /*!< Enter pretrack immediately on Track. */
    ROT_LIST_COL_PRETRACK_MIN_EL, /*!< Pretrack elevation in degrees. */
    ROT_LIST_COL_DISABLE_POS_FEEDBACK, /*!< Disable feedback checks. */
    ROT_LIST_COL_EL_OVERTRAVEL_ENABLE, /*!< Enable custom elevation overtravel clamp. */
    ROT_LIST_COL_EL_MIN_DEG,   /*!< Overtravel minimum elevation in degrees. */
    ROT_LIST_COL_EL_MAX_DEG,   /*!< Overtravel maximum elevation in degrees. */
    ROT_LIST_COL_HAMLIB_MODEL, /*!< Hamlib model ID. */
    ROT_LIST_COL_LAST_GOOD_DEVICE, /*!< Last validated serial device. */
    ROT_LIST_COL_LAST_GOOD_BAUD, /*!< Last validated baud rate. */
    ROT_LIST_COL_NUM            /*!< The number of fields in the list. */
} rotor_list_col_t;

#endif
