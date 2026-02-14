/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#ifndef ROTCTLD_PARSE_H
#define ROTCTLD_PARSE_H

#include <glib.h>

gboolean parse_dump_state_model_id(const gchar *reply, gint *model_out);
gboolean rotctld_parse_model(const gchar *reply, gint *model_out);

#endif
