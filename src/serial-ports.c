#include "serial-ports.h"

#if defined(__APPLE__)
GSList *gp_serial_list_candidates_darwin(void);
#elif defined(G_OS_WIN32)
GSList *gp_serial_list_candidates_win32(void);
#else
static GSList *gp_serial_list_candidates_unix(void)
{
    GDir *dir = NULL;
    GSList *preferred = NULL;
    GSList *others = NULL;
    const gchar *name = NULL;

    dir = g_dir_open("/dev", 0, NULL);
    if (dir == NULL)
        return NULL;

    while ((name = g_dir_read_name(dir)) != NULL)
    {
        gchar *path = NULL;

        if (g_str_has_prefix(name, "ttyUSB") ||
            g_str_has_prefix(name, "ttyACM") ||
            g_str_has_prefix(name, "tty.usb") ||
            g_str_has_prefix(name, "ttyAMA"))
        {
            path = g_build_filename("/dev", name, NULL);
            preferred = g_slist_append(preferred, path);
            continue;
        }

        if (g_str_has_prefix(name, "ttyS") ||
            g_str_has_prefix(name, "tty."))
        {
            path = g_build_filename("/dev", name, NULL);
            others = g_slist_append(others, path);
            continue;
        }
    }

    g_dir_close(dir);

    if (preferred == NULL)
        return others;

    return g_slist_concat(preferred, others);
}
#endif

GSList *gp_serial_list_candidates(void)
{
#if defined(__APPLE__)
    return gp_serial_list_candidates_darwin();
#elif defined(G_OS_WIN32)
    return gp_serial_list_candidates_win32();
#else
    return gp_serial_list_candidates_unix();
#endif
}

void gp_serial_free_candidates(GSList *list)
{
    g_slist_free_full(list, g_free);
}
