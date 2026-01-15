#ifndef SAT_PREF_RIG_EDITOR_H
#define SAT_PREF_RIG_EDITOR_H 1

#include <gtk/gtk.h>
#include "radio-conf.h"

typedef void (*RigPrefEditorDoneFunc)(radio_conf_t *conf,
                                      gboolean applied,
                                      gpointer user_data);

void            sat_pref_rig_editor_run(radio_conf_t * conf,
                                        RigPrefEditorDoneFunc done,
                                        gpointer user_data);

#endif
