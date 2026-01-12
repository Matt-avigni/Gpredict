#include "serial-ports.h"

#if defined(__APPLE__)
GSList *gp_serial_list_candidates_darwin(void);
#elif defined(G_OS_WIN32)
GSList *gp_serial_list_candidates_win32(void);
#endif

GSList *gp_serial_list_candidates(void)
{
#if defined(__APPLE__)
    return gp_serial_list_candidates_darwin();
#elif defined(G_OS_WIN32)
    return gp_serial_list_candidates_win32();
#else
    return NULL;
#endif
}

void gp_serial_free_candidates(GSList *list)
{
    g_slist_free_full(list, g_free);
}
