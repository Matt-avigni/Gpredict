#include "rotor_calibration.h"

#include <glib.h>
#include <glib/gi18n.h>
#include <glib/gstdio.h>
#include <math.h>

#include "gpredict-utils.h"
#include "rotor-angle.h"

#define DEFAULT_CALIB_UNCERTAINTY_DEG 0.5

static gchar *calib_sanitize_id(const char *rotor_id)
{
    gchar *safe = NULL;

    if (rotor_id == NULL || *rotor_id == '\0')
        return g_strdup("default");

    safe = g_strdup(rotor_id);
    for (gchar *p = safe; *p != '\0'; ++p)
    {
        switch (*p)
        {
        case '/':
        case ':':
        case '@':
        case ' ':
            *p = '_';
            break;
        default:
            break;
        }
    }

    return safe;
}

static gchar *calib_build_path(const char *rotor_id, gchar **dir_out)
{
    gchar *safe_id = NULL;
    gchar *dir = NULL;
    gchar *path = NULL;

    safe_id = calib_sanitize_id(rotor_id);
    dir = g_build_filename(g_get_user_config_dir(),
                           "gpredict",
                           "rotors",
                           safe_id,
                           NULL);
    path = g_build_filename(dir, "calibration.ini", NULL);

    if (dir_out)
        *dir_out = dir;
    else
        g_free(dir);

    g_free(safe_id);
    return path;
}

double wrap180(double az_deg)
{
    return wrap360(az_deg + 180.0) - 180.0;
}

bool calib_load(const char *rotor_id, RotorCalib *out)
{
    GKeyFile *keyfile = NULL;
    gchar *path = NULL;
    gboolean ok = FALSE;

    if (out == NULL)
        return false;

    *out = (RotorCalib){ 0 };

    if (rotor_id == NULL || *rotor_id == '\0')
        return false;

    path = calib_build_path(rotor_id, NULL);
    keyfile = g_key_file_new();

    if (g_key_file_load_from_file(keyfile, path,
                                  G_KEY_FILE_KEEP_COMMENTS,
                                  NULL))
    {
        out->az_uncertainty_deg = DEFAULT_CALIB_UNCERTAINTY_DEG;
        out->el_uncertainty_deg = DEFAULT_CALIB_UNCERTAINTY_DEG;
        out->enabled =
            g_key_file_get_boolean(keyfile, "calib", "enabled", NULL);
        out->az_offset_deg =
            g_key_file_get_double(keyfile, "calib", "az_offset_deg", NULL);
        out->el_offset_deg =
            g_key_file_get_double(keyfile, "calib", "el_offset_deg", NULL);
        if (g_key_file_has_key(keyfile, "calib", "az_uncertainty_deg", NULL))
            out->az_uncertainty_deg =
                g_key_file_get_double(keyfile, "calib", "az_uncertainty_deg", NULL);
        if (g_key_file_has_key(keyfile, "calib", "el_uncertainty_deg", NULL))
            out->el_uncertainty_deg =
                g_key_file_get_double(keyfile, "calib", "el_uncertainty_deg", NULL);
        ok = TRUE;
    }

    g_key_file_free(keyfile);
    g_free(path);
    return ok;
}

bool calib_save(const char *rotor_id, const RotorCalib *c)
{
    GKeyFile *keyfile = NULL;
    gchar *path = NULL;
    gchar *dir = NULL;
    gboolean ok = FALSE;

    if (rotor_id == NULL || *rotor_id == '\0' || c == NULL)
        return false;

    path = calib_build_path(rotor_id, &dir);
    if (g_mkdir_with_parents(dir, 0700) != 0)
    {
        g_free(dir);
        g_free(path);
        return false;
    }

    keyfile = g_key_file_new();
    g_key_file_set_boolean(keyfile, "calib", "enabled", c->enabled);
    g_key_file_set_double(keyfile, "calib", "az_offset_deg",
                          c->az_offset_deg);
    g_key_file_set_double(keyfile, "calib", "el_offset_deg",
                          c->el_offset_deg);
    g_key_file_set_double(keyfile, "calib", "az_uncertainty_deg",
                          c->az_uncertainty_deg);
    g_key_file_set_double(keyfile, "calib", "el_uncertainty_deg",
                          c->el_uncertainty_deg);

    ok = (gpredict_save_key_file(keyfile, path) == 0);

    g_key_file_free(keyfile);
    g_free(dir);
    g_free(path);
    return ok;
}

void calib_apply_mech_zero(RotorCalib *c,
                           double mech_az,
                           double mech_el,
                           double uncertainty_deg)
{
    double tol = uncertainty_deg;

    if (c == NULL)
        return;

    if (tol <= 0.0)
        tol = DEFAULT_CALIB_UNCERTAINTY_DEG;

    c->az_offset_deg = wrap360(-mech_az);
    c->el_offset_deg = -mech_el;
    c->az_uncertainty_deg = tol;
    c->el_uncertainty_deg = tol;
    c->enabled = true;
}

void world_to_mech(double *az_deg_canon, double *el_deg, const RotorCalib *c)
{
    if (az_deg_canon == NULL || el_deg == NULL || c == NULL || !c->enabled)
        return;

    *az_deg_canon = wrap360(*az_deg_canon - c->az_offset_deg);
    *el_deg -= c->el_offset_deg;
}

void mech_to_world(double *az_deg_canon, double *el_deg, const RotorCalib *c)
{
    if (az_deg_canon == NULL || el_deg == NULL || c == NULL || !c->enabled)
        return;

    *az_deg_canon = wrap360(*az_deg_canon + c->az_offset_deg);
    *el_deg += c->el_offset_deg;
}

bool calib_run_wizard(GtkWindow *parent,
                      const char *rotor_id,
                      fn_setpos send_setpos,
                      fn_getpos read_getpos,
                      RotorCalib *io_calib)
{
    GtkWidget *dialog = NULL;
    gint response = GTK_RESPONSE_CANCEL;
    double mech_az = 0.0;
    double mech_el = 0.0;
    RotorCalib updated = { 0 };

    if (io_calib == NULL || send_setpos == NULL || read_getpos == NULL)
        return false;

    if (!send_setpos(0.0, 0.0))
        return false;

    dialog = gtk_message_dialog_new(parent,
                                    GTK_DIALOG_MODAL |
                                    GTK_DIALOG_DESTROY_WITH_PARENT,
                                    GTK_MESSAGE_INFO,
                                    GTK_BUTTONS_OK_CANCEL,
                                    _("Auto-calibration: Program commanded AZ=0\302\260, EL=0\302\260.\n"
                                      "Align array to TRUE NORTH and zero elevation if needed. "
                                      "Press OK when stable."));
    gtk_dialog_set_default_response(GTK_DIALOG(dialog), GTK_RESPONSE_OK);
    response = gtk_dialog_run(GTK_DIALOG(dialog));
    gtk_widget_destroy(dialog);

    if (response != GTK_RESPONSE_OK)
        return false;

    if (!read_getpos(&mech_az, &mech_el))
        return false;

    updated = *io_calib;
    calib_apply_mech_zero(&updated, mech_az, mech_el,
                          DEFAULT_CALIB_UNCERTAINTY_DEG);

    (void)calib_save(rotor_id, &updated);
    *io_calib = updated;
    return true;
}
