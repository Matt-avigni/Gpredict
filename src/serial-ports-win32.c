#ifdef HAVE_CONFIG_H
#include <build-config.h>
#endif

#include <glib.h>

#include "serial-ports.h"

#ifdef G_OS_WIN32
#include <windows.h>

static gboolean list_has_port(GSList *list, const gchar *port)
{
    for (GSList *item = list; item != NULL; item = item->next)
    {
        if (g_strcmp0(item->data, port) == 0)
            return TRUE;
    }

    return FALSE;
}

GSList *gp_serial_list_candidates_win32(void)
{
    GSList *list = NULL;
    HKEY key = NULL;
    LONG ret;
    DWORD index = 0;

    ret = RegOpenKeyExA(HKEY_LOCAL_MACHINE,
                        "HARDWARE\\DEVICEMAP\\SERIALCOMM",
                        0, KEY_READ, &key);
    if (ret != ERROR_SUCCESS)
        return NULL;

    while (TRUE)
    {
        char value_name[256];
        BYTE data[256];
        DWORD value_len = sizeof(value_name);
        DWORD data_len = sizeof(data);
        DWORD type = 0;

        ret = RegEnumValueA(key, index, value_name, &value_len, NULL, &type,
                            data, &data_len);
        if (ret == ERROR_NO_MORE_ITEMS)
            break;
        if (ret == ERROR_SUCCESS && type == REG_SZ && data_len > 0)
        {
            gchar *port = NULL;

            if (data_len >= sizeof(data))
                data[sizeof(data) - 1] = '\0';
            else
                data[data_len] = '\0';

            port = g_strdup((const gchar *) data);
            if (port != NULL && *port != '\0')
            {
                gchar *normalized = NULL;
                gint   number = 0;

                if (g_ascii_strncasecmp(port, "COM", 3) == 0)
                    number = (gint) g_ascii_strtoll(port + 3, NULL, 10);

                if (number >= 10)
                    normalized = g_strdup_printf("\\\\.\\%s", port);
                else
                    normalized = g_strdup(port);

                if (!list_has_port(list, normalized))
                    list = g_slist_append(list, normalized);
                else
                    g_free(normalized);
            }
            g_free(port);
        }
        index++;
    }

    RegCloseKey(key);

    return list;
}

#endif
