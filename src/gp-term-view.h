#ifndef GP_TERM_VIEW_H
#define GP_TERM_VIEW_H 1

#include <glib.h>
#include <gtk/gtk.h>

typedef struct _GpTermView GpTermView;

GpTermView *gp_term_view_new(const gchar *follow_label,
                             gboolean show_copy,
                             gboolean start_visible);
GtkWidget *gp_term_view_get_widget(GpTermView *view);
void       gp_term_view_set_visible(GpTermView *view, gboolean visible);
void       gp_term_view_clear(GpTermView *view);
void       gp_term_view_free(GpTermView *view);
void       gp_term_view_log(GpTermView *view, const gchar *fmt, ...)
    G_GNUC_PRINTF(2, 3);

#endif
