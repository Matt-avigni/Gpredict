/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#ifndef TLE_AUTOUPDATE_H
#define TLE_AUTOUPDATE_H

#include <glib.h>

typedef struct _gtk_sat_module GtkSatModule;

void tle_autoupdate_start(GtkSatModule *module);

#endif /* TLE_AUTOUPDATE_H */
