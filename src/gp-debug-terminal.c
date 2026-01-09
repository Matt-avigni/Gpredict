/*
  Gpredict: Real-time satellite tracking and orbit prediction program

  Copyright (C)  2024  Matteo Avigni

  This program is free software; you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation; either version 2 of the License, or
  (at your option) any later version.
  
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  
  You should have received a copy of the GNU General Public License
  along with this program; if not, visit http://www.fsf.org/
*/

#ifdef HAVE_CONFIG_H
#include <build-config.h>
#endif

#include "gp-debug-terminal.h"
#include "gp-term-view.h"

#include <glib/gi18n.h>
#include <stdarg.h>

struct _GpDbgTerm {
    GtkWidget  *window;
    GpTermView *view;
};

static gboolean gp_dbg_term_on_delete(GtkWidget *widget, GdkEventAny *event,
                                      gpointer user_data)
{
    (void)event;
    (void)user_data;

    gtk_widget_hide(widget);
    return TRUE;
}

GpDbgTerm *gp_dbg_term_new(const gchar *title)
{
    GpDbgTerm *term;

    term = g_new0(GpDbgTerm, 1);

    term->window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_default_size(GTK_WINDOW(term->window), 800, 400);
    gtk_window_set_title(GTK_WINDOW(term->window),
                         title ? title : _("Debug terminal"));
    gtk_window_set_destroy_with_parent(GTK_WINDOW(term->window), TRUE);
    g_signal_connect(term->window, "delete-event",
                     G_CALLBACK(gp_dbg_term_on_delete), term);

    term->view = gp_term_view_new(_("Auto-scroll"), TRUE, TRUE);
    gtk_container_add(GTK_CONTAINER(term->window),
                      gp_term_view_get_widget(term->view));

    return term;
}

void gp_dbg_term_free(GpDbgTerm *term)
{
    if (term == NULL)
        return;

    if (term->view)
    {
        gp_term_view_free(term->view);
        term->view = NULL;
    }

    if (GTK_IS_WIDGET(term->window))
        gtk_widget_destroy(term->window);

    g_free(term);
}

void gp_dbg_term_show(GpDbgTerm *term)
{
    if (term == NULL || term->window == NULL)
        return;

    gtk_widget_show_all(term->window);
    gtk_window_present(GTK_WINDOW(term->window));
}

void gp_dbg_term_log(GpDbgTerm *term, const gchar *fmt, ...)
{
    va_list ap;
    gchar *msg;

    if (term == NULL || term->view == NULL || fmt == NULL)
        return;

    va_start(ap, fmt);
    msg = g_strdup_vprintf(fmt, ap);
    va_end(ap);

    if (msg == NULL)
        return;

    gp_term_view_log(term->view, "%s", msg);
    g_free(msg);
}
