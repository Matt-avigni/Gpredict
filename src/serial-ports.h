/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#ifndef SERIAL_PORTS_H
#define SERIAL_PORTS_H 1

#include <glib.h>

GSList *gp_serial_list_candidates(void);
gboolean gp_serial_port_is_windows_com(const gchar *candidate);
gint gp_serial_windows_com_number(const gchar *candidate);
void    gp_serial_free_candidates(GSList *list);

#endif
