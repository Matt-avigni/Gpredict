/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#ifndef UI_COMBO_GUARD_H
#define UI_COMBO_GUARD_H 1

#include <gtk/gtk.h>

void gp_combo_guard_install(GtkComboBox *combo, GtkWidget *toplevel);

#endif
