/*
    Gpredict: Real-time satellite tracking and orbit prediction program

    Copyright (C)  2001-2015  Alexandru Csete.

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
#include "sat-log.h"
#include "compat.h"
#include "gpredict-utils.h"

#include "radio-conf.h"

#define GROUP           "Radio"
#define KEY_HOST        "Host"
#define KEY_PORT        "Port"
#define KEY_CYCLE       "Cycle"
#define KEY_LO          "LO"
#define KEY_LOUP        "LO_UP"
#define KEY_TYPE        "Type"
#define KEY_RADIO_MODEL "RADIO_MODEL"
#define KEY_RADIO_MODE  "RADIO_MODE"
#define KEY_PTT         "PTT"
#define KEY_VFO_DOWN    "VFO_DOWN"
#define KEY_VFO_UP      "VFO_UP"
#define KEY_DOWNLINK_VFO "DOWNLINK_VFO"
#define KEY_UPLINK_VFO  "UPLINK_VFO"
#define KEY_SIG_AOS     "SIGNAL_AOS"
#define KEY_SIG_LOS     "SIGNAL_LOS"
#define KEY_RIGCTLD_AUTOSTART   "RIGCTLD_AUTOSTART"
#define KEY_RIGCTLD_AUTO_POWER_ON "RIGCTLD_AUTO_POWER_ON"
#define KEY_RIGCTLD_PATH        "RIGCTLD_PATH"
#define KEY_RIGCTLD_MODEL       "RIGCTLD_MODEL"
#define KEY_RIGCTLD_CONN_TYPE   "RIGCTLD_CONN_TYPE"
#define KEY_RIGCTLD_DEVICE      "RIGCTLD_DEVICE"
#define KEY_RIGCTLD_BAUD        "RIGCTLD_BAUD"
#define KEY_RIGCTLD_CIVADDR     "RIGCTLD_CIVADDR"
#define KEY_RIGCTLD_EXTRA_ARGS  "RIGCTLD_EXTRA_ARGS"
#define KEY_RIGCTLD_AUTODETECT_MATCH "RIGCTLD_AUTODETECT_MATCH"
#define KEY_IC9700_SATMODE      "IC9700_SATMODE"

#define DEFAULT_CYCLE_MS    1000

static gboolean radio_model_valid(gint model)
{
    return model >= RADIO_MODEL_OTHER && model <= RADIO_MODEL_IC905;
}

static gboolean radio_mode_valid(gint mode)
{
    return mode >= RADIO_MODE_SIMPLEX && mode <= RADIO_MODE_FULL_DUPLEX_MAIN_SUB;
}

static gboolean radio_vfo_valid(gint vfo)
{
    return vfo == VFO_NONE || vfo == VFO_A || vfo == VFO_B ||
        vfo == VFO_MAIN || vfo == VFO_SUB;
}

static gboolean rigctld_device_looks_like_hostport(const gchar *device)
{
    const gchar *colon;
    const gchar *port;

    if (device == NULL || *device == '\0')
        return FALSE;

    colon = strchr(device, ':');
    if (colon == NULL || colon == device || *(colon + 1) == '\0')
        return FALSE;

    port = colon + 1;
    while (*port)
    {
        if (!g_ascii_isdigit(*port))
            return FALSE;
        port++;
    }

    return TRUE;
}

typedef struct {
    radio_model_t model;
    guint allowed_modes;
} radio_mode_profile_t;

#define RADIO_MODE_MASK_SIMPLEX (1u << RADIO_MODE_SIMPLEX)
#define RADIO_MODE_MASK_SPLIT (1u << RADIO_MODE_SPLIT)
#define RADIO_MODE_MASK_FULL_DUPLEX_MAIN_SUB (1u << RADIO_MODE_FULL_DUPLEX_MAIN_SUB)
#define RADIO_MODE_MASK_ALL (RADIO_MODE_MASK_SIMPLEX | RADIO_MODE_MASK_SPLIT | \
                             RADIO_MODE_MASK_FULL_DUPLEX_MAIN_SUB)

static const radio_mode_profile_t radio_mode_profiles[] = {
    { RADIO_MODEL_OTHER, RADIO_MODE_MASK_ALL },
    { RADIO_MODEL_IC9700, RADIO_MODE_MASK_ALL },
    { RADIO_MODEL_IC705, RADIO_MODE_MASK_SIMPLEX | RADIO_MODE_MASK_SPLIT },
    /* IC-905 full-duplex MAIN/SUB is not validated by current hamlib usage. */
    { RADIO_MODEL_IC905, RADIO_MODE_MASK_SIMPLEX | RADIO_MODE_MASK_SPLIT }
};

static guint radio_mode_mask_for_model(radio_model_t model)
{
    for (gsize i = 0; i < G_N_ELEMENTS(radio_mode_profiles); i++)
    {
        if (radio_mode_profiles[i].model == model)
            return radio_mode_profiles[i].allowed_modes;
    }

    return RADIO_MODE_MASK_ALL;
}

gboolean radio_mode_allowed_for_model(radio_model_t model, radio_mode_t mode)
{
    if (!radio_mode_valid(mode))
        return FALSE;

    return (radio_mode_mask_for_model(model) & (1u << mode)) != 0;
}

gchar *radio_mode_allowed_string(radio_model_t model)
{
    guint mask = radio_mode_mask_for_model(model);
    GString *out = g_string_new(NULL);
    radio_mode_t modes[] = {
        RADIO_MODE_SIMPLEX,
        RADIO_MODE_SPLIT,
        RADIO_MODE_FULL_DUPLEX_MAIN_SUB
    };

    for (gsize i = 0; i < G_N_ELEMENTS(modes); i++)
    {
        if ((mask & (1u << modes[i])) == 0)
            continue;

        if (out->len > 0)
            g_string_append(out, ", ");
        g_string_append(out, radio_mode_to_string(modes[i]));
    }

    if (out->len == 0)
        g_string_assign(out, "none");

    return g_string_free(out, FALSE);
}

const gchar *radio_model_to_string(radio_model_t model)
{
    switch (model)
    {
    case RADIO_MODEL_IC9700:
        return "IC-9700";
    case RADIO_MODEL_IC705:
        return "IC-705";
    case RADIO_MODEL_IC905:
        return "IC-905";
    case RADIO_MODEL_OTHER:
    default:
        return "OTHER";
    }
}

gint radio_model_to_hamlib_model(radio_model_t model)
{
    switch (model)
    {
    case RADIO_MODEL_IC9700:
        return 3081;
    case RADIO_MODEL_IC705:
        return 3085;
    case RADIO_MODEL_IC905:
        return 3090;
    case RADIO_MODEL_OTHER:
    default:
        return 0;
    }
}

gboolean radio_model_get_rigctld_defaults(radio_model_t model,
                                          rigctld_preset_defaults_t *out)
{
    rigctld_preset_defaults_t preset;

    switch (model)
    {
    case RADIO_MODEL_IC9700:
        preset.host = "127.0.0.1";
        preset.port = 4532;
        preset.conn = RIGCTLD_CONN_SERIAL;
        preset.baud = 115200;
        preset.civaddr = "0xA2";
        break;
    case RADIO_MODEL_IC705:
        preset.host = "127.0.0.1";
        preset.port = 4532;
        preset.conn = RIGCTLD_CONN_SERIAL;
        preset.baud = 115200;
        preset.civaddr = "0xA4";
        break;
    case RADIO_MODEL_IC905:
        preset.host = "127.0.0.1";
        preset.port = 4532;
        preset.conn = RIGCTLD_CONN_SERIAL;
        preset.baud = 115200;
        preset.civaddr = "0xAC";
        break;
    case RADIO_MODEL_OTHER:
    default:
        return FALSE;
    }

    if (out)
        *out = preset;

    return TRUE;
}

const gchar *radio_mode_to_string(radio_mode_t mode)
{
    switch (mode)
    {
    case RADIO_MODE_SIMPLEX:
        return "SIMPLEX";
    case RADIO_MODE_SPLIT:
        return "SPLIT";
    case RADIO_MODE_FULL_DUPLEX_MAIN_SUB:
        return "FULL_DUPLEX_MAIN_SUB";
    default:
        return "UNKNOWN";
    }
}

/**
 * \brief Read radio configuration.
 * \param conf Pointer to a radio_conf_t structure where the data will be
 *             stored.
 * \return TRUE if the configuration was read successfully, FALSE if an
 *         error has occurred.
 * 
 * This function reads a radio configuration from a .rig file into conf.
 * conf->name must contain the file name of the configuration (no path, just
 * file name).
 */
gboolean radio_conf_read(radio_conf_t * conf)
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
    fname = g_strconcat(confdir, G_DIR_SEPARATOR_S, conf->name, ".rig", NULL);
    g_free(confdir);

    conf->radio_model = RADIO_MODEL_OTHER;
    conf->radio_mode = RADIO_MODE_SIMPLEX;
    conf->downlink_vfo = VFO_MAIN;
    conf->uplink_vfo = VFO_SUB;
    conf->supports_rit_xit = FALSE;
    conf->supports_full_duplex = FALSE;
    conf->supports_dual_vfo_sat = FALSE;
    conf->rigctld_autostart = TRUE;
    conf->rigctld_auto_power_on = FALSE;
    conf->rigctld_path = NULL;
    conf->rigctld_model = 0;
    conf->rigctld_conn = RIGCTLD_CONN_SERIAL;
    conf->rigctld_device = NULL;
    conf->rigctld_baud = 0;
    conf->rigctld_civaddr = NULL;
    conf->rigctld_extra_args = NULL;
    conf->rigctld_autodetect_match = NULL;

    /* open .rig file */
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
                    _("%s: Error reading radio conf from %s (%s)."),
                    __func__, conf->name, error->message);
        g_clear_error(&error);
        g_key_file_free(cfg);
        return FALSE;
    }

    conf->port = g_key_file_get_integer(cfg, GROUP, KEY_PORT, &error);
    if (error != NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: Error reading radio conf from %s (%s)."),
                    __func__, conf->name, error->message);
        g_clear_error(&error);
        g_key_file_free(cfg);
        return FALSE;
    }

    /* cycle period is only saved if not default */
    if (g_key_file_has_key(cfg, GROUP, KEY_CYCLE, NULL))
    {
        conf->cycle = g_key_file_get_integer(cfg, GROUP, KEY_CYCLE, &error);
        if (error != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: Error reading radio conf from %s (%s)."),
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

    /* KEY_LO is optional */
    if (g_key_file_has_key(cfg, GROUP, KEY_LO, NULL))
    {
        conf->lo = g_key_file_get_double(cfg, GROUP, KEY_LO, &error);
        if (error != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: Error reading radio conf from %s (%s)."),
                        __func__, conf->name, error->message);
            g_clear_error(&error);
            g_key_file_free(cfg);
            return FALSE;
        }
    }
    else
    {
        conf->lo = 0.0;
    }

    /* KEY_LOUP is optional */
    if (g_key_file_has_key(cfg, GROUP, KEY_LOUP, NULL))
    {
        conf->loup = g_key_file_get_double(cfg, GROUP, KEY_LOUP, &error);
        if (error != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: Error reading radio conf from %s (%s)."),
                        __func__, conf->name, error->message);
            g_clear_error(&error);
            g_key_file_free(cfg);
            return FALSE;
        }
    }
    else
    {
        conf->loup = 0.0;
    }

    /* Radio type */
    conf->type = g_key_file_get_integer(cfg, GROUP, KEY_TYPE, &error);
    if (error != NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: Error reading radio conf from %s (%s)."),
                    __func__, conf->name, error->message);
        g_clear_error(&error);
        g_key_file_free(cfg);
        return FALSE;
    }

    /* PTT Type */
    conf->ptt = g_key_file_get_integer(cfg, GROUP, KEY_PTT, &error);
    if (error != NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: Error reading radio conf from %s (%s)."),
                    __func__, conf->name, error->message);
        g_clear_error(&error);
        g_key_file_free(cfg);
        return FALSE;
    }

    if (g_key_file_has_key(cfg, GROUP, KEY_RADIO_MODEL, NULL))
    {
        gint model = g_key_file_get_integer(cfg, GROUP, KEY_RADIO_MODEL, &error);
        if (error != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: Error reading radio conf from %s (%s)."),
                        __func__, conf->name, error->message);
            g_clear_error(&error);
        }
        else if (radio_model_valid(model))
        {
            conf->radio_model = (radio_model_t) model;
        }
    }

    if (g_key_file_has_key(cfg, GROUP, KEY_RADIO_MODE, NULL))
    {
        gint mode = g_key_file_get_integer(cfg, GROUP, KEY_RADIO_MODE, &error);
        if (error != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: Error reading radio conf from %s (%s)."),
                        __func__, conf->name, error->message);
            g_clear_error(&error);
        }
        else if (radio_mode_valid(mode))
        {
            conf->radio_mode = (radio_mode_t) mode;
        }
    }

    if (g_key_file_has_key(cfg, GROUP, KEY_IC9700_SATMODE, NULL))
    {
        gboolean ic9700_satmode =
            g_key_file_get_boolean(cfg, GROUP, KEY_IC9700_SATMODE, NULL);
        if (ic9700_satmode)
        {
            if (!g_key_file_has_key(cfg, GROUP, KEY_RADIO_MODE, NULL))
                conf->radio_mode = RADIO_MODE_FULL_DUPLEX_MAIN_SUB;
            if (!g_key_file_has_key(cfg, GROUP, KEY_RADIO_MODEL, NULL))
                conf->radio_model = RADIO_MODEL_IC9700;
        }
    }

    if (!g_key_file_has_key(cfg, GROUP, KEY_RADIO_MODE, NULL) &&
        conf->radio_mode == RADIO_MODE_SIMPLEX)
    {
        if (conf->type == RIG_TYPE_DUPLEX)
            conf->radio_mode = RADIO_MODE_SPLIT;
    }

    if (g_key_file_has_key(cfg, GROUP, KEY_DOWNLINK_VFO, NULL))
    {
        gint vfo = g_key_file_get_integer(cfg, GROUP, KEY_DOWNLINK_VFO, &error);
        if (error != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: Error reading radio conf from %s (%s)."),
                        __func__, conf->name, error->message);
            g_clear_error(&error);
        }
        else if (radio_vfo_valid(vfo))
        {
            conf->downlink_vfo = (vfo_t) vfo;
        }
    }
    else if (g_key_file_has_key(cfg, GROUP, KEY_VFO_DOWN, NULL))
    {
        gint vfo = g_key_file_get_integer(cfg, GROUP, KEY_VFO_DOWN, &error);
        if (error != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: Error reading radio conf from %s (%s)."),
                        __func__, conf->name, error->message);
            g_clear_error(&error);
        }
        else if (radio_vfo_valid(vfo))
        {
            conf->downlink_vfo = (vfo_t) vfo;
        }
    }

    if (g_key_file_has_key(cfg, GROUP, KEY_UPLINK_VFO, NULL))
    {
        gint vfo = g_key_file_get_integer(cfg, GROUP, KEY_UPLINK_VFO, &error);
        if (error != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: Error reading radio conf from %s (%s)."),
                        __func__, conf->name, error->message);
            g_clear_error(&error);
        }
        else if (radio_vfo_valid(vfo))
        {
            conf->uplink_vfo = (vfo_t) vfo;
        }
    }
    else if (g_key_file_has_key(cfg, GROUP, KEY_VFO_UP, NULL))
    {
        gint vfo = g_key_file_get_integer(cfg, GROUP, KEY_VFO_UP, &error);
        if (error != NULL)
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: Error reading radio conf from %s (%s)."),
                        __func__, conf->name, error->message);
            g_clear_error(&error);
        }
        else if (radio_vfo_valid(vfo))
        {
            conf->uplink_vfo = (vfo_t) vfo;
        }
    }

    /* Signal AOS and LOS */
    conf->signal_aos = g_key_file_get_boolean(cfg, GROUP, KEY_SIG_AOS, NULL);
    conf->signal_los = g_key_file_get_boolean(cfg, GROUP, KEY_SIG_LOS, NULL);

    if (g_key_file_has_key(cfg, GROUP, KEY_RIGCTLD_AUTOSTART, NULL))
        conf->rigctld_autostart =
            g_key_file_get_boolean(cfg, GROUP, KEY_RIGCTLD_AUTOSTART, NULL);
    if (g_key_file_has_key(cfg, GROUP, KEY_RIGCTLD_AUTO_POWER_ON, NULL))
        conf->rigctld_auto_power_on =
            g_key_file_get_boolean(cfg, GROUP,
                                   KEY_RIGCTLD_AUTO_POWER_ON, NULL);
    if (g_key_file_has_key(cfg, GROUP, KEY_RIGCTLD_PATH, NULL))
        conf->rigctld_path =
            g_key_file_get_string(cfg, GROUP, KEY_RIGCTLD_PATH, NULL);
    if (g_key_file_has_key(cfg, GROUP, KEY_RIGCTLD_MODEL, NULL))
        conf->rigctld_model =
            g_key_file_get_integer(cfg, GROUP, KEY_RIGCTLD_MODEL, NULL);
    if (g_key_file_has_key(cfg, GROUP, KEY_RIGCTLD_CONN_TYPE, NULL))
        conf->rigctld_conn =
            g_key_file_get_integer(cfg, GROUP, KEY_RIGCTLD_CONN_TYPE, NULL);
    if (conf->rigctld_conn != RIGCTLD_CONN_SERIAL &&
        conf->rigctld_conn != RIGCTLD_CONN_TCP)
        conf->rigctld_conn = RIGCTLD_CONN_SERIAL;
    if (g_key_file_has_key(cfg, GROUP, KEY_RIGCTLD_DEVICE, NULL))
        conf->rigctld_device =
            g_key_file_get_string(cfg, GROUP, KEY_RIGCTLD_DEVICE, NULL);
    if (!g_key_file_has_key(cfg, GROUP, KEY_RIGCTLD_CONN_TYPE, NULL) &&
        rigctld_device_looks_like_hostport(conf->rigctld_device))
        conf->rigctld_conn = RIGCTLD_CONN_TCP;
    if (g_key_file_has_key(cfg, GROUP, KEY_RIGCTLD_BAUD, NULL))
        conf->rigctld_baud =
            g_key_file_get_integer(cfg, GROUP, KEY_RIGCTLD_BAUD, NULL);
    if (g_key_file_has_key(cfg, GROUP, KEY_RIGCTLD_CIVADDR, NULL))
        conf->rigctld_civaddr =
            g_key_file_get_string(cfg, GROUP, KEY_RIGCTLD_CIVADDR, NULL);
    if (g_key_file_has_key(cfg, GROUP, KEY_RIGCTLD_EXTRA_ARGS, NULL))
        conf->rigctld_extra_args =
            g_key_file_get_string(cfg, GROUP, KEY_RIGCTLD_EXTRA_ARGS, NULL);
    if (g_key_file_has_key(cfg, GROUP, KEY_RIGCTLD_AUTODETECT_MATCH, NULL))
        conf->rigctld_autodetect_match =
            g_key_file_get_string(cfg, GROUP, KEY_RIGCTLD_AUTODETECT_MATCH, NULL);

    conf->supports_dual_vfo_sat =
        (conf->radio_mode == RADIO_MODE_FULL_DUPLEX_MAIN_SUB);
    conf->supports_full_duplex =
        (conf->radio_mode == RADIO_MODE_FULL_DUPLEX_MAIN_SUB) ||
        (conf->radio_mode == RADIO_MODE_SPLIT);
    conf->supports_rit_xit = (conf->radio_model == RADIO_MODEL_IC9700);

    g_key_file_free(cfg);
    sat_log_log(SAT_LOG_LEVEL_INFO,
                _("%s: Read radio configuration %s"), __func__, conf->name);

    return TRUE;
}

/**
 * \brief Save radio configuration.
 * \param conf Pointer to the radio configuration.
 * 
 * This function saves the radio configuration stored in conf to a
 * .rig file. conf->name must contain the file name of the configuration
 * (no path, just file name).
 */
void radio_conf_save(radio_conf_t * conf)
{
    GKeyFile       *cfg = NULL;
    gchar          *confdir;
    gchar          *fname;

    if (conf->name == NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s: NULL configuration name!"), __func__);
        return;
    }

    /* create a config structure */
    cfg = g_key_file_new();

    g_key_file_set_string(cfg, GROUP, KEY_HOST, conf->host);
    g_key_file_set_integer(cfg, GROUP, KEY_PORT, conf->port);
    g_key_file_set_double(cfg, GROUP, KEY_LO, conf->lo);
    g_key_file_set_double(cfg, GROUP, KEY_LOUP, conf->loup);
    g_key_file_set_integer(cfg, GROUP, KEY_TYPE, conf->type);
    g_key_file_set_integer(cfg, GROUP, KEY_RADIO_MODEL, conf->radio_model);
    g_key_file_set_integer(cfg, GROUP, KEY_RADIO_MODE, conf->radio_mode);
    g_key_file_set_integer(cfg, GROUP, KEY_PTT, conf->ptt);

    if (conf->cycle == DEFAULT_CYCLE_MS)
        g_key_file_remove_key(cfg, GROUP, KEY_CYCLE, NULL);
    else
        g_key_file_set_integer(cfg, GROUP, KEY_CYCLE, conf->cycle);

    g_key_file_set_integer(cfg, GROUP, KEY_UPLINK_VFO, conf->uplink_vfo);
    g_key_file_set_integer(cfg, GROUP, KEY_DOWNLINK_VFO, conf->downlink_vfo);
    if (conf->type == RIG_TYPE_DUPLEX)
    {
        g_key_file_set_integer(cfg, GROUP, KEY_VFO_UP, conf->uplink_vfo);
        g_key_file_set_integer(cfg, GROUP, KEY_VFO_DOWN, conf->downlink_vfo);
    }

    g_key_file_set_boolean(cfg, GROUP, KEY_SIG_AOS, conf->signal_aos);
    g_key_file_set_boolean(cfg, GROUP, KEY_SIG_LOS, conf->signal_los);
    g_key_file_set_boolean(cfg, GROUP, KEY_IC9700_SATMODE,
                           conf->radio_mode == RADIO_MODE_FULL_DUPLEX_MAIN_SUB);
    g_key_file_set_boolean(cfg, GROUP, KEY_RIGCTLD_AUTOSTART,
                           conf->rigctld_autostart);
    g_key_file_set_boolean(cfg, GROUP, KEY_RIGCTLD_AUTO_POWER_ON,
                           conf->rigctld_auto_power_on);
    if (conf->rigctld_path && *conf->rigctld_path)
        g_key_file_set_string(cfg, GROUP, KEY_RIGCTLD_PATH,
                              conf->rigctld_path);
    else
        g_key_file_remove_key(cfg, GROUP, KEY_RIGCTLD_PATH, NULL);
    if (conf->rigctld_model > 0)
        g_key_file_set_integer(cfg, GROUP, KEY_RIGCTLD_MODEL,
                               conf->rigctld_model);
    else
        g_key_file_remove_key(cfg, GROUP, KEY_RIGCTLD_MODEL, NULL);
    g_key_file_set_integer(cfg, GROUP, KEY_RIGCTLD_CONN_TYPE,
                           conf->rigctld_conn);
    if (conf->rigctld_device && *conf->rigctld_device)
        g_key_file_set_string(cfg, GROUP, KEY_RIGCTLD_DEVICE,
                              conf->rigctld_device);
    else
        g_key_file_remove_key(cfg, GROUP, KEY_RIGCTLD_DEVICE, NULL);
    if (conf->rigctld_baud > 0)
        g_key_file_set_integer(cfg, GROUP, KEY_RIGCTLD_BAUD,
                               conf->rigctld_baud);
    else
        g_key_file_remove_key(cfg, GROUP, KEY_RIGCTLD_BAUD, NULL);
    if (conf->rigctld_civaddr && *conf->rigctld_civaddr)
        g_key_file_set_string(cfg, GROUP, KEY_RIGCTLD_CIVADDR,
                              conf->rigctld_civaddr);
    else
        g_key_file_remove_key(cfg, GROUP, KEY_RIGCTLD_CIVADDR, NULL);
    if (conf->rigctld_extra_args && *conf->rigctld_extra_args)
        g_key_file_set_string(cfg, GROUP, KEY_RIGCTLD_EXTRA_ARGS,
                              conf->rigctld_extra_args);
    else
        g_key_file_remove_key(cfg, GROUP, KEY_RIGCTLD_EXTRA_ARGS, NULL);
    if (conf->rigctld_autodetect_match && *conf->rigctld_autodetect_match)
        g_key_file_set_string(cfg, GROUP, KEY_RIGCTLD_AUTODETECT_MATCH,
                              conf->rigctld_autodetect_match);
    else
        g_key_file_remove_key(cfg, GROUP, KEY_RIGCTLD_AUTODETECT_MATCH, NULL);

    confdir = get_hwconf_dir();
    fname = g_strconcat(confdir, G_DIR_SEPARATOR_S, conf->name, ".rig", NULL);
    g_free(confdir);

    gpredict_save_key_file(cfg, fname);

    g_free(fname);
    g_key_file_free(cfg);
}
