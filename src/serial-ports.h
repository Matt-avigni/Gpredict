#ifndef SERIAL_PORTS_H
#define SERIAL_PORTS_H 1

#include <glib.h>

GSList *gp_serial_list_candidates(void);
void    gp_serial_free_candidates(GSList *list);

#endif
