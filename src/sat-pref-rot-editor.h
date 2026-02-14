/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#ifndef SAT_PREF_ROT_EDITOR_H
#define SAT_PREF_ROT_EDITOR_H 1

#include <gtk/gtk.h>
#include "rotor-conf.h"

typedef void (*RotPrefEditorDoneFunc)(rotor_conf_t *conf,
                                      gboolean applied,
                                      gpointer user_data);

void            sat_pref_rot_editor_run(rotor_conf_t * conf,
                                        RotPrefEditorDoneFunc done,
                                        gpointer user_data);

#endif
