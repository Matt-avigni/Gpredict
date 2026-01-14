#ifdef HAVE_CONFIG_H
#include <build-config.h>
#endif

#include <glib.h>

#include "serial-ports.h"

#ifdef __APPLE__

static gboolean is_preferred_cu_name(const gchar *name)
{
    return g_str_has_prefix(name, "cu.usb") ||
        g_str_has_prefix(name, "cu.SLAB_") ||
        g_str_has_prefix(name, "cu.wchusbserial") ||
        (g_strrstr(name, "ftdi") != NULL);
}

GSList *gp_serial_list_candidates_darwin(void)
{
    GDir   *dir = NULL;
    GSList *preferred = NULL;
    GSList *others = NULL;
    GSList *tty_list = NULL;
    const gchar *name = NULL;

    dir = g_dir_open("/dev", 0, NULL);
    if (dir == NULL)
        return NULL;

    while ((name = g_dir_read_name(dir)) != NULL)
    {
        gchar *path = NULL;

        if (!g_str_has_prefix(name, "cu.") &&
            !g_str_has_prefix(name, "tty."))
            continue;

        if (g_strrstr(name, "Bluetooth") != NULL)
            continue;
        if (g_strrstr(name, "debug-console") != NULL)
            continue;

        path = g_build_filename("/dev", name, NULL);
        if (g_str_has_prefix(name, "cu."))
        {
            if (is_preferred_cu_name(name))
                preferred = g_slist_append(preferred, path);
            else
                others = g_slist_append(others, path);
        }
        else
        {
            tty_list = g_slist_append(tty_list, path);
        }
    }

    g_dir_close(dir);

    if (preferred == NULL)
        return g_slist_concat(others, tty_list);

    return g_slist_concat(g_slist_concat(preferred, others), tty_list);
}

#endif
