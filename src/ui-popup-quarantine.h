#ifndef UI_POPUP_QUARANTINE_H
#define UI_POPUP_QUARANTINE_H 1

#include <gtk/gtk.h>

void gp_ui_quarantine_install(GtkWidget *toplevel);
void gp_ui_quarantine_register_combo(GtkWidget *toplevel, GtkComboBox *combo);
gboolean gp_ui_combo_popup_shown(GtkComboBox *combo);

#endif
