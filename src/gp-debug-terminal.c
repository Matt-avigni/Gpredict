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

#include <glib/gi18n.h>
#include <pango/pango.h>
#include <stdarg.h>

#define GP_DBG_TERM_MAX_LINES 10000

typedef struct {
    guint64        seq;
    gchar         *text;
} GpDbgTermLine;

struct _GpDbgTerm {
    GtkWidget     *window;
    GtkWidget     *text_view;
    GtkWidget     *auto_scroll;
    GtkTextMark   *end_mark;

    GQueue        *lines;       /* Ring buffer of GpDbgTermLine* */
    guint          trim_pending;
    guint          flush_source;
    guint64        next_seq;
    guint64        flushed_seq;

    GMutex         mutex;
};

static void
gp_dbg_term_free_line(gpointer data, gpointer user_data)
{
    GpDbgTermLine *line = data;

    (void)user_data;

    if (line == NULL)
        return;

    g_free(line->text);
    g_free(line);
}

static gboolean
gp_dbg_term_on_delete(GtkWidget *widget, GdkEventAny *event, gpointer user_data)
{
    (void)event;
    (void)user_data;

    gtk_widget_hide(widget);
    return TRUE;
}

static void
gp_dbg_term_clear_buffer(GpDbgTerm *term)
{
    GtkTextBuffer *buffer;
    GtkTextIter start;

    if (term == NULL)
        return;

    g_mutex_lock(&term->mutex);
    if (term->lines)
    {
        g_queue_foreach(term->lines, gp_dbg_term_free_line, NULL);
        g_queue_clear(term->lines);
    }

    term->trim_pending = 0;
    term->flushed_seq = term->next_seq;

    if (term->flush_source != 0)
    {
        g_source_remove(term->flush_source);
        term->flush_source = 0;
    }
    g_mutex_unlock(&term->mutex);

    buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(term->text_view));
    gtk_text_buffer_set_text(buffer, "", -1);
    gtk_text_buffer_get_start_iter(buffer, &start);
    if (term->end_mark != NULL)
        gtk_text_buffer_move_mark(buffer, term->end_mark, &start);
}

static void
gp_dbg_term_clear_clicked(GtkButton *button, gpointer user_data)
{
    GpDbgTerm *term = user_data;

    (void)button;

    gp_dbg_term_clear_buffer(term);
}

static void
gp_dbg_term_copy_clicked(GtkButton *button, gpointer user_data)
{
    GpDbgTerm *term = user_data;
    GtkTextBuffer *buffer;
    GtkClipboard *clipboard;
    GtkTextIter start, end;
    gchar *text;

    (void)button;

    if (term == NULL || term->text_view == NULL)
        return;

    buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(term->text_view));
    gtk_text_buffer_get_start_iter(buffer, &start);
    gtk_text_buffer_get_end_iter(buffer, &end);

    text = gtk_text_buffer_get_text(buffer, &start, &end, TRUE);

    clipboard = gtk_clipboard_get(GDK_SELECTION_CLIPBOARD);
    gtk_clipboard_set_text(clipboard, text, -1);

    g_free(text);
}

static void
gp_dbg_term_apply_trimming(GtkTextBuffer *buffer, guint trim_lines)
{
    GtkTextIter start, iter;
    guint i;

    if (trim_lines == 0)
        return;

    gtk_text_buffer_get_start_iter(buffer, &start);
    iter = start;

    for (i = 0; i < trim_lines; i++)
    {
        if (!gtk_text_iter_forward_line(&iter))
        {
            gtk_text_buffer_set_text(buffer, "", -1);
            return;
        }
        gtk_text_buffer_delete(buffer, &start, &iter);
        start = iter;
    }
}

static gboolean
gp_dbg_term_flush(gpointer user_data)
{
    GpDbgTerm *term = user_data;
    GtkTextBuffer *buffer;
    GtkTextIter end;
    GQueue new_lines = G_QUEUE_INIT;
    guint trim = 0;
    guint64 last_flushed = 0;
    GList *iter;

    if (term == NULL || term->text_view == NULL)
        return G_SOURCE_REMOVE;

    buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(term->text_view));

    g_mutex_lock(&term->mutex);
    trim = term->trim_pending;
    term->trim_pending = 0;
    last_flushed = term->flushed_seq;

    for (iter = term->lines ? term->lines->head : NULL; iter != NULL;
         iter = iter->next)
    {
        GpDbgTermLine *line = iter->data;
        if (line->seq > last_flushed)
        {
            g_queue_push_tail(&new_lines, line->text);
            if (line->seq > term->flushed_seq)
                term->flushed_seq = line->seq;
        }
    }

    term->flush_source = 0;
    g_mutex_unlock(&term->mutex);

    gp_dbg_term_apply_trimming(buffer, trim);

    while (!g_queue_is_empty(&new_lines))
    {
        const gchar *text = g_queue_pop_head(&new_lines);
        gtk_text_buffer_get_end_iter(buffer, &end);
        gtk_text_buffer_insert(buffer, &end, text, -1);
    }

    if (term->end_mark != NULL)
    {
        gtk_text_buffer_get_end_iter(buffer, &end);
        gtk_text_buffer_move_mark(buffer, term->end_mark, &end);

        if (gtk_toggle_button_get_active
            (GTK_TOGGLE_BUTTON(term->auto_scroll)))
        {
            gtk_text_view_scroll_to_mark(GTK_TEXT_VIEW(term->text_view),
                                         term->end_mark, 0.0, TRUE, 0.0, 0.0);
        }
    }

    return G_SOURCE_REMOVE;
}

static void
gp_dbg_term_schedule_flush(GpDbgTerm *term)
{
    if (term == NULL)
        return;

    g_mutex_lock(&term->mutex);
    if (term->flush_source == 0)
        term->flush_source = g_idle_add(gp_dbg_term_flush, term);
    g_mutex_unlock(&term->mutex);
}

GpDbgTerm *
gp_dbg_term_new(const gchar *title)
{
    GpDbgTerm *term;
    GtkWidget *outer;
    GtkWidget *scroller;
    GtkWidget *button_box;
    GtkWidget *button;
    GtkTextBuffer *buffer;
    GtkTextIter start;

    term = g_new0(GpDbgTerm, 1);
    term->lines = g_queue_new();
    term->next_seq = 1;
    term->flushed_seq = 0;
    term->trim_pending = 0;
    term->flush_source = 0;
    g_mutex_init(&term->mutex);

    term->window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_default_size(GTK_WINDOW(term->window), 800, 400);
    gtk_window_set_title(GTK_WINDOW(term->window),
                         title ? title : _("Debug terminal"));
    gtk_window_set_destroy_with_parent(GTK_WINDOW(term->window), TRUE);
    g_signal_connect(term->window, "delete-event",
                     G_CALLBACK(gp_dbg_term_on_delete), term);

    outer = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
    gtk_container_set_border_width(GTK_CONTAINER(outer), 8);
    gtk_container_add(GTK_CONTAINER(term->window), outer);

    scroller = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scroller),
                                   GTK_POLICY_AUTOMATIC,
                                   GTK_POLICY_AUTOMATIC);
    gtk_scrolled_window_set_shadow_type(GTK_SCROLLED_WINDOW(scroller),
                                        GTK_SHADOW_IN);
    gtk_box_pack_start(GTK_BOX(outer), scroller, TRUE, TRUE, 0);

    term->text_view = gtk_text_view_new();
    gtk_text_view_set_editable(GTK_TEXT_VIEW(term->text_view), FALSE);
    gtk_text_view_set_cursor_visible(GTK_TEXT_VIEW(term->text_view), FALSE);
    gtk_text_view_set_wrap_mode(GTK_TEXT_VIEW(term->text_view),
                                GTK_WRAP_NONE);
#if GTK_CHECK_VERSION(3, 16, 0)
    gtk_text_view_set_monospace(GTK_TEXT_VIEW(term->text_view), TRUE);
#else
    {
        PangoFontDescription *font;

        font = pango_font_description_from_string("Monospace");
        gtk_widget_override_font(term->text_view, font);
        pango_font_description_free(font);
    }
#endif
    gtk_container_add(GTK_CONTAINER(scroller), term->text_view);

    buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(term->text_view));
    gtk_text_buffer_set_text(buffer, "", -1);
    gtk_text_buffer_get_start_iter(buffer, &start);
    term->end_mark = gtk_text_buffer_create_mark(buffer, NULL, &start, FALSE);

    button_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    gtk_box_pack_start(GTK_BOX(outer), button_box, FALSE, FALSE, 0);

    button = gtk_button_new_with_label(_("Clear"));
    g_signal_connect(button, "clicked",
                     G_CALLBACK(gp_dbg_term_clear_clicked), term);
    gtk_box_pack_start(GTK_BOX(button_box), button, FALSE, FALSE, 0);

    button = gtk_button_new_with_label(_("Copy"));
    g_signal_connect(button, "clicked",
                     G_CALLBACK(gp_dbg_term_copy_clicked), term);
    gtk_box_pack_start(GTK_BOX(button_box), button, FALSE, FALSE, 0);

    term->auto_scroll = gtk_check_button_new_with_label(_("Auto-scroll"));
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(term->auto_scroll), TRUE);
    gtk_box_pack_end(GTK_BOX(button_box), term->auto_scroll, FALSE, FALSE, 0);

    return term;
}

void
gp_dbg_term_free(GpDbgTerm *term)
{
    if (term == NULL)
        return;

    gp_dbg_term_clear_buffer(term);

    if (term->lines)
        g_queue_free(term->lines);

    if (GTK_IS_WIDGET(term->window))
        gtk_widget_destroy(term->window);

    g_mutex_clear(&term->mutex);

    g_free(term);
}

void
gp_dbg_term_show(GpDbgTerm *term)
{
    if (term == NULL || term->window == NULL)
        return;

    gtk_widget_show_all(term->window);
    gtk_window_present(GTK_WINDOW(term->window));
}

void
gp_dbg_term_log(GpDbgTerm *term, const gchar *fmt, ...)
{
    va_list ap;
    gchar *msg;
    GpDbgTermLine *line;

    if (term == NULL || fmt == NULL)
        return;

    va_start(ap, fmt);
    msg = g_strdup_vprintf(fmt, ap);
    va_end(ap);

    if (msg == NULL)
        return;

    if (!g_str_has_suffix(msg, "\n"))
    {
        gchar *tmp = g_strconcat(msg, "\n", NULL);
        g_free(msg);
        msg = tmp;
    }

    line = g_new0(GpDbgTermLine, 1);
    line->text = msg;

    g_mutex_lock(&term->mutex);
    line->seq = term->next_seq++;
    g_queue_push_tail(term->lines, line);

    if (g_queue_get_length(term->lines) > GP_DBG_TERM_MAX_LINES)
    {
        GpDbgTermLine *old = g_queue_pop_head(term->lines);
        gp_dbg_term_free_line(old, NULL);
        term->trim_pending++;
    }
    g_mutex_unlock(&term->mutex);

    gp_dbg_term_schedule_flush(term);
}
