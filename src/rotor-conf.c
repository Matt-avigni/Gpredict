/* -*- Mode: C; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/*
    Gpredict: Real-time satellite tracking and orbit prediction program

    Copyright (C)  2001-2019  Alexandru Csete.

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
#include <gtk/gtk.h>
#include <glib/gi18n.h>
#include <hamlib/rotlist.h>
#include "sat-log.h"
#include "compat.h"

#include "rotor-conf.h"
#include "gpredict-utils.h"

#define GROUP           "Rotator"
#define KEY_HOST        "Host"
#define KEY_PORT        "Port"
#define KEY_PROTOCOL    "Protocol"
#define KEY_BAUD        "Baud"
#define KEY_DEVICE      "Device"
#define KEY_DEVICE_MANUAL "DeviceManual"
#define KEY_DEVICE_AUTOPICK "DeviceAutopick"
#define KEY_AUTOSTART   "Autostart"
#define KEY_CYCLE       "Cycle"
#define KEY_AZTYPE      "AzType"
#define KEY_MINAZ       "MinAz"
#define KEY_MAXAZ       "MaxAz"
#define KEY_MINEL       "MinEl"
#define KEY_MAXEL       "MaxEl"
#define KEY_AZSTOPPOS   "AzStopPos"
#define KEY_THLD        "Threshold"
#define KEY_AXIS_MODE   "AxisMode"
#define KEY_USE_OFFSET  "UseOffset"
#define KEY_AZ_OFFSET   "AzOffset"
#define KEY_EL_OFFSET   "ElOffset"
#define KEY_AZ_INVERT   "AzInvert"
#define KEY_EL_INVERT   "ElInvert"

#define DEFAULT_CYCLE_MS    1000
#define DEFAULT_THLD_DEG    5.0
#define DEFAULT_PROTOCOL    ROT_PROTOCOL_GS232B
#define DEFAULT_AUTOSTART   TRUE
#define DEFAULT_DEVICE_AUTOPICK TRUE

/* Hamlib rotator model IDs from hamlib/rotlist.h. */
#define ROT_HAMLIB_MODEL_GS232B    ROT_MODEL_GS232B
#define ROT_HAMLIB_MODEL_SPID_ROT2 ROT_MODEL_SPID_ROT2PROG
#define ROT_HAMLIB_MODEL_SPID_ROT1 ROT_MODEL_SPID_ROT1PROG

gboolean rot_protocol_is_valid(rot_protocol_t protocol)
{
    return protocol >= ROT_PROTOCOL_GS232B &&
           protocol <= ROT_PROTOCOL_SPID_ROT2PROG;
}

const gchar *rot_protocol_name(rot_protocol_t protocol)
{
    switch (protocol)
    {
    case ROT_PROTOCOL_SPID_ROT1PROG:
        return "rot1prog";
    case ROT_PROTOCOL_SPID_ROT2PROG:
        return "rot2prog";
    case ROT_PROTOCOL_GS232B:
    default:
        return "gs232b";
    }
}

const gchar *rot_protocol_model_name(rot_protocol_t protocol)
{
    switch (protocol)
    {
    case ROT_PROTOCOL_SPID_ROT1PROG:
        return "ROT_MODEL_SPID_ROT1PROG";
    case ROT_PROTOCOL_SPID_ROT2PROG:
        return "ROT_MODEL_SPID_ROT2PROG";
    case ROT_PROTOCOL_GS232B:
    default:
        return "ROT_MODEL_GS232B";
    }
}

gint rot_protocol_to_hamlib_model(rot_protocol_t protocol)
{
    switch (protocol)
    {
    case ROT_PROTOCOL_GS232B:
        return ROT_HAMLIB_MODEL_GS232B;
    case ROT_PROTOCOL_SPID_ROT2PROG:
        return ROT_HAMLIB_MODEL_SPID_ROT2;
    case ROT_PROTOCOL_SPID_ROT1PROG:
        return ROT_HAMLIB_MODEL_SPID_ROT1;
    default:
        return ROT_HAMLIB_MODEL_GS232B;
    }
}

gint rot_protocol_default_baud(rot_protocol_t protocol)
{
    (void) protocol;
    return 9600;
}

/**
 * \brief Read rotator configuration.
 * \param conf Pointer to a rotor_conf_t structure where the data will be
 *             stored.
 * 
 * This function reads a rotoator configuration from a .rot file into conf.
 * conf->name must contain the file name of the configuration (no path, just
 * file name and without the .rot extension).
 */
gboolean rotor_conf_read(rotor_conf_t * conf)
{
    GKeyFile       *cfg = NULL;
    gchar          *confdir;
    gchar          *fname;
    GError         *error = NULL;

    if (conf->name == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: NULL configuration name!"), __func__);
        return FALSE;
    }

    confdir = get_hwconf_dir();
    fname = g_strconcat(confdir, G_DIR_SEPARATOR_S, conf->name, ".rot", NULL);
    g_free(confdir);

    /* open .rot file */
    cfg = g_key_file_new();
    g_key_file_load_from_file(cfg, fname, 0, NULL);

    if (cfg == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: Could not load file %s\n"), __func__, fname);
        g_free(fname);

        return FALSE;
    }

    g_free(fname);

    /* read parameters */
    conf->host = g_key_file_get_string(cfg, GROUP, KEY_HOST, &error);
    if (error != NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: Error reading rotor conf from %s (%s)."),
                    __func__, conf->name, error->message);
        g_clear_error(&error);
        g_key_file_free(cfg);
        return FALSE;
    }

    conf->port = g_key_file_get_integer(cfg, GROUP, KEY_PORT, &error);
    if (error != NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: Error reading rotor conf from %s (%s)."),
                    __func__, conf->name, error->message);
        g_clear_error(&error);
        g_key_file_free(cfg);
        return FALSE;
    }

    conf->protocol = DEFAULT_PROTOCOL;
    if (g_key_file_has_key(cfg, GROUP, KEY_PROTOCOL, NULL))
    {
        conf->protocol = g_key_file_get_integer(cfg, GROUP, KEY_PROTOCOL, &error);
        if (error != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        _("%s: Protocol not defined for %s. Assuming GS-232B."),
                        __func__, conf->name);
            g_clear_error(&error);
            conf->protocol = DEFAULT_PROTOCOL;
        }
    }

    if (conf->protocol < ROT_PROTOCOL_GS232B ||
        conf->protocol > ROT_PROTOCOL_SPID_ROT2PROG)
    {
        conf->protocol = DEFAULT_PROTOCOL;
    }

    conf->baud = 0;
    if (g_key_file_has_key(cfg, GROUP, KEY_BAUD, NULL))
    {
        conf->baud = g_key_file_get_integer(cfg, GROUP, KEY_BAUD, &error);
        if (error != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        _("%s: Baud not defined for %s. Using protocol default."),
                        __func__, conf->name);
            g_clear_error(&error);
            conf->baud = 0;
        }
    }

    if (conf->baud <= 0)
        conf->baud = rot_protocol_default_baud(conf->protocol);

    conf->device = NULL;
    if (g_key_file_has_key(cfg, GROUP, KEY_DEVICE, NULL))
    {
        conf->device = g_key_file_get_string(cfg, GROUP, KEY_DEVICE, &error);
        if (error != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        _("%s: Device not defined for %s."),
                        __func__, conf->name);
            g_clear_error(&error);
            conf->device = NULL;
        }
    }

    conf->device_manual = NULL;
    if (g_key_file_has_key(cfg, GROUP, KEY_DEVICE_MANUAL, NULL))
    {
        conf->device_manual =
            g_key_file_get_string(cfg, GROUP, KEY_DEVICE_MANUAL, &error);
        if (error != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        _("%s: Manual device not defined for %s."),
                        __func__, conf->name);
            g_clear_error(&error);
            conf->device_manual = NULL;
        }
    }

    conf->device_autopick = DEFAULT_DEVICE_AUTOPICK;
    if (g_key_file_has_key(cfg, GROUP, KEY_DEVICE_AUTOPICK, NULL))
    {
        conf->device_autopick =
            g_key_file_get_boolean(cfg, GROUP, KEY_DEVICE_AUTOPICK, &error);
        if (error != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        _("%s: Device autopick not defined for %s. Assuming true."),
                        __func__, conf->name);
            g_clear_error(&error);
            conf->device_autopick = DEFAULT_DEVICE_AUTOPICK;
        }
    }

    conf->autostart = DEFAULT_AUTOSTART;
    if (g_key_file_has_key(cfg, GROUP, KEY_AUTOSTART, NULL))
    {
        conf->autostart =
            g_key_file_get_boolean(cfg, GROUP, KEY_AUTOSTART, &error);
        if (error != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        _("%s: Autostart not defined for %s. Assuming true."),
                        __func__, conf->name);
            g_clear_error(&error);
            conf->autostart = DEFAULT_AUTOSTART;
        }
    }

    /* cycle period and threshold are only saved if not default */
    if (g_key_file_has_key(cfg, GROUP, KEY_CYCLE, NULL))
    {
        conf->cycle = g_key_file_get_integer(cfg, GROUP, KEY_CYCLE, &error);
        if (error != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: Error reading rotor conf from %s (%s)."),
                        __func__, conf->name, error->message);
            g_clear_error(&error);
            g_key_file_free(cfg);
            return FALSE;
        }
        if (conf->cycle < 10)
            conf->cycle = 10;
    }
    else
    {
        conf->cycle = DEFAULT_CYCLE_MS;
    }

    if (g_key_file_has_key(cfg, GROUP, KEY_THLD, NULL))
    {
        conf->threshold = g_key_file_get_double(cfg, GROUP, KEY_THLD, &error);
        if (error != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: Error reading rotor conf from %s (%s)."),
                        __func__, conf->name, error->message);
            g_clear_error(&error);
            g_key_file_free(cfg);
            return FALSE;
        }
        if (conf->threshold < 0.1)
            conf->threshold = 0.1;
    }
    else
    {
        conf->threshold = DEFAULT_THLD_DEG;
    }

    conf->aztype = g_key_file_get_integer(cfg, GROUP, KEY_AZTYPE, &error);
    if (error != NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: Az type not defined for %s. Assuming 0..360\302\260"),
                    __func__, conf->name);
        g_clear_error(&error);

        conf->aztype = ROT_AZ_TYPE_360;
    }

    conf->minaz = g_key_file_get_double(cfg, GROUP, KEY_MINAZ, &error);
    if (error != NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: MinAz not defined for %s. Assuming 0\302\260."),
                    __func__, conf->name);
        g_clear_error(&error);
        conf->minaz = 0.0;
    }

    conf->maxaz = g_key_file_get_double(cfg, GROUP, KEY_MAXAZ, &error);
    if (error != NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: MaxAz not defined for %s. Assuming 360\302\260."),
                    __func__, conf->name);
        g_clear_error(&error);
        conf->maxaz = 360.0;
    }

    conf->minel = g_key_file_get_double(cfg, GROUP, KEY_MINEL, &error);
    if (error != NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: MinEl not defined for %s. Assuming 0\302\260."),
                    __func__, conf->name);
        g_clear_error(&error);
        conf->minel = 0.0;
    }

    conf->maxel = g_key_file_get_double(cfg, GROUP, KEY_MAXEL, &error);
    if (error != NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: MaxEl not defined for %s. Assuming 90\302\260."),
                    __func__, conf->name);
        g_clear_error(&error);
        conf->maxel = 90.0;
    }

    conf->azstoppos = g_key_file_get_double(cfg, GROUP, KEY_AZSTOPPOS, &error);
    if (error != NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: AzStopPos not defined for %s. Assuming at minaz (%f\302\260)."),
                    __func__, conf->name, conf->minaz);
        g_clear_error(&error);
        conf->azstoppos = conf->minaz;
    }

    conf->axis_mode = ROT_AXIS_MODE_AZ_EL;
    if (g_key_file_has_key(cfg, GROUP, KEY_AXIS_MODE, NULL))
    {
        conf->axis_mode = g_key_file_get_integer(cfg, GROUP, KEY_AXIS_MODE, &error);
        if (error != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        _("%s: AxisMode not defined for %s. Assuming AZ/EL."),
                        __func__, conf->name);
            g_clear_error(&error);
            conf->axis_mode = ROT_AXIS_MODE_AZ_EL;
        }
    }

    if (conf->axis_mode != ROT_AXIS_MODE_AZ_EL &&
        conf->axis_mode != ROT_AXIS_MODE_AZ_ONLY)
    {
        sat_log_log(SAT_LOG_LEVEL_INFO,
                    _("%s: Invalid AxisMode for %s. Assuming AZ/EL."),
                    __func__, conf->name);
        conf->axis_mode = ROT_AXIS_MODE_AZ_EL;
    }

    conf->use_offset = FALSE;
    if (g_key_file_has_key(cfg, GROUP, KEY_USE_OFFSET, NULL))
    {
        conf->use_offset = g_key_file_get_boolean(cfg, GROUP, KEY_USE_OFFSET, &error);
        if (error != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        _("%s: UseOffset not defined for %s. Assuming disabled."),
                        __func__, conf->name);
            g_clear_error(&error);
            conf->use_offset = FALSE;
        }
    }

    conf->az_offset = 0.0;
    if (g_key_file_has_key(cfg, GROUP, KEY_AZ_OFFSET, NULL))
    {
        conf->az_offset = g_key_file_get_double(cfg, GROUP, KEY_AZ_OFFSET, &error);
        if (error != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        _("%s: AzOffset not defined for %s. Assuming 0."),
                        __func__, conf->name);
            g_clear_error(&error);
            conf->az_offset = 0.0;
        }
    }

    conf->el_offset = 0.0;
    if (g_key_file_has_key(cfg, GROUP, KEY_EL_OFFSET, NULL))
    {
        conf->el_offset = g_key_file_get_double(cfg, GROUP, KEY_EL_OFFSET, &error);
        if (error != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        _("%s: ElOffset not defined for %s. Assuming 0."),
                        __func__, conf->name);
            g_clear_error(&error);
            conf->el_offset = 0.0;
        }
    }

    conf->invert_az = FALSE;
    if (g_key_file_has_key(cfg, GROUP, KEY_AZ_INVERT, NULL))
    {
        conf->invert_az = g_key_file_get_boolean(cfg, GROUP, KEY_AZ_INVERT, &error);
        if (error != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        _("%s: AzInvert not defined for %s. Assuming false."),
                        __func__, conf->name);
            g_clear_error(&error);
            conf->invert_az = FALSE;
        }
    }

    conf->invert_el = FALSE;
    if (g_key_file_has_key(cfg, GROUP, KEY_EL_INVERT, NULL))
    {
        conf->invert_el = g_key_file_get_boolean(cfg, GROUP, KEY_EL_INVERT, &error);
        if (error != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        _("%s: ElInvert not defined for %s. Assuming false."),
                        __func__, conf->name);
            g_clear_error(&error);
            conf->invert_el = FALSE;
        }
    }

    g_key_file_free(cfg);

    return TRUE;
}

/**
 * \brief Save rotator configuration.
 * \param conf Pointer to the rotator configuration.
 * 
 * This function saves the rotator configuration stored in conf to a
 * .rig file. conf->name must contain the file name of the configuration
 * (no path, just file name and without the .rot extension).
 */
void rotor_conf_save(rotor_conf_t * conf)
{
    GKeyFile       *cfg = NULL;
    gchar          *confdir;
    gchar          *fname;

    if (conf->name == NULL)
        return;

    /* create a config structure */
    cfg = g_key_file_new();

    g_key_file_set_string(cfg, GROUP, KEY_HOST, conf->host);
    g_key_file_set_integer(cfg, GROUP, KEY_PORT, conf->port);
    g_key_file_set_integer(cfg, GROUP, KEY_PROTOCOL, conf->protocol);
    g_key_file_set_integer(cfg, GROUP, KEY_BAUD, conf->baud);
    g_key_file_set_boolean(cfg, GROUP, KEY_DEVICE_AUTOPICK,
                           conf->device_autopick);
    g_key_file_set_boolean(cfg, GROUP, KEY_AUTOSTART, conf->autostart);
    g_key_file_set_integer(cfg, GROUP, KEY_AZTYPE, conf->aztype);
    g_key_file_set_double(cfg, GROUP, KEY_MINAZ, conf->minaz);
    g_key_file_set_double(cfg, GROUP, KEY_MAXAZ, conf->maxaz);
    g_key_file_set_double(cfg, GROUP, KEY_MINEL, conf->minel);
    g_key_file_set_double(cfg, GROUP, KEY_MAXEL, conf->maxel);
    g_key_file_set_double(cfg, GROUP, KEY_AZSTOPPOS, conf->azstoppos);
    g_key_file_set_integer(cfg, GROUP, KEY_AXIS_MODE, conf->axis_mode);
    g_key_file_set_boolean(cfg, GROUP, KEY_USE_OFFSET, conf->use_offset);
    g_key_file_set_double(cfg, GROUP, KEY_AZ_OFFSET, conf->az_offset);
    g_key_file_set_double(cfg, GROUP, KEY_EL_OFFSET, conf->el_offset);
    g_key_file_set_boolean(cfg, GROUP, KEY_AZ_INVERT, conf->invert_az);
    g_key_file_set_boolean(cfg, GROUP, KEY_EL_INVERT, conf->invert_el);

    if (conf->device && *conf->device)
        g_key_file_set_string(cfg, GROUP, KEY_DEVICE, conf->device);
    else
        g_key_file_remove_key(cfg, GROUP, KEY_DEVICE, NULL);

    if (conf->device_manual && *conf->device_manual)
        g_key_file_set_string(cfg, GROUP, KEY_DEVICE_MANUAL, conf->device_manual);
    else
        g_key_file_remove_key(cfg, GROUP, KEY_DEVICE_MANUAL, NULL);

    if (conf->cycle == DEFAULT_CYCLE_MS)
        g_key_file_remove_key(cfg, GROUP, KEY_CYCLE, NULL);
    else
        g_key_file_set_integer(cfg, GROUP, KEY_CYCLE, conf->cycle);

    if (conf->threshold == DEFAULT_THLD_DEG)
        g_key_file_remove_key(cfg, GROUP, KEY_THLD, NULL);
    else
        g_key_file_set_double(cfg, GROUP, KEY_THLD, conf->threshold);

    /* build filename */
    confdir = get_hwconf_dir();
    fname = g_strconcat(confdir, G_DIR_SEPARATOR_S, conf->name, ".rot", NULL);
    g_free(confdir);

    /* save information */
    gpredict_save_key_file(cfg, fname);

    /* cleanup */
    g_free(fname);
    g_key_file_free(cfg);
}
