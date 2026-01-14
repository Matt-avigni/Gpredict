#ifndef ROTCTLD_PARSE_H
#define ROTCTLD_PARSE_H

#include <glib.h>

gboolean parse_dump_state_model_id(const gchar *reply, gint *model_out);
gboolean rotctld_parse_model(const gchar *reply, gint *model_out);

#endif
