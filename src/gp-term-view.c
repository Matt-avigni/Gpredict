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

#include "gp-term-view.h"

#include <glib/gi18n.h>
#include <pango/pango.h>
#include <stdarg.h>

#define GP_TERM_VIEW_MAX_LINES 10000

typedef struct {
    guint64        seq;
    gchar         *text;
} GpTermViewLine;

struct _GpTermView {
    GtkWidget     *revealer;
    GtkWidget     *text_view;
    GtkWidget     *follow_toggle;
    GtkTextMark   *end_mark;

    GQueue        *lines;       /* Ring buffer of GpTermViewLine* */
    guint          trim_pending;
    guint          flush_source;
    guint64        next_seq;
    guint64        flushed_seq;

    GMutex         mutex;
};

static void gp_term_view_free_line(gpointer data, gpointer user_data)
{
    GpTermViewLine *line = data;

    (void)user_data;

    if (line == NULL)
        return;

    g_free(line->text);
    g_free(line);
}

static void gp_term_view_clear_buffer(GpTermView *view)
{
    GtkTextBuffer *buffer;
    GtkTextIter start;

    if (view == NULL)
        return;

    g_mutex_lock(&view->mutex);
    if (view->lines)
    {
        g_queue_foreach(view->lines, gp_term_view_free_line, NULL);
        g_queue_clear(view->lines);
    }

    view->trim_pending = 0;
    view->flushed_seq = view->next_seq;

    if (view->flush_source != 0)
    {
        g_source_remove(view->flush_source);
        view->flush_source = 0;
    }
    g_mutex_unlock(&view->mutex);

    buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(view->text_view));
    gtk_text_buffer_set_text(buffer, "", -1);
    gtk_text_buffer_get_start_iter(buffer, &start);
    if (view->end_mark != NULL)
        gtk_text_buffer_move_mark(buffer, view->end_mark, &start);
}

static void gp_term_view_clear_clicked(GtkButton *button, gpointer user_data)
{
    GpTermView *view = user_data;

    (void)button;

    gp_term_view_clear_buffer(view);
}

static void gp_term_view_copy_clicked(GtkButton *button, gpointer user_data)
{
    GpTermView *view = user_data;
    GtkTextBuffer *buffer;
    GtkClipboard *clipboard;
    GtkTextIter start, end;
    gchar *text;

    (void)button;

    if (view == NULL || view->text_view == NULL)
        return;

    buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(view->text_view));
    if (!gtk_text_buffer_get_selection_bounds(buffer, &start, &end))
    {
        gtk_text_buffer_get_start_iter(buffer, &start);
        gtk_text_buffer_get_end_iter(buffer, &end);
    }

    text = gtk_text_buffer_get_text(buffer, &start, &end, TRUE);

    clipboard = gtk_clipboard_get(GDK_SELECTION_CLIPBOARD);
    gtk_clipboard_set_text(clipboard, text, -1);

    g_free(text);
}

static void gp_term_view_apply_trimming(GtkTextBuffer *buffer, guint trim_lines)
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

static gboolean gp_term_view_flush(gpointer user_data)
{
    GpTermView *view = user_data;
    GtkTextBuffer *buffer;
    GtkTextIter end;
    GQueue new_lines = G_QUEUE_INIT;
    guint trim = 0;
    guint64 last_flushed = 0;
    GList *iter;

    if (view == NULL || view->text_view == NULL)
        return G_SOURCE_REMOVE;

    buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(view->text_view));

    g_mutex_lock(&view->mutex);
    trim = view->trim_pending;
    view->trim_pending = 0;
    last_flushed = view->flushed_seq;

    for (iter = view->lines ? view->lines->head : NULL; iter != NULL;
         iter = iter->next)
    {
        GpTermViewLine *line = iter->data;
        if (line->seq > last_flushed)
        {
            g_queue_push_tail(&new_lines, line->text);
            if (line->seq > view->flushed_seq)
                view->flushed_seq = line->seq;
        }
    }

    view->flush_source = 0;
    g_mutex_unlock(&view->mutex);

    gp_term_view_apply_trimming(buffer, trim);

    while (!g_queue_is_empty(&new_lines))
    {
        const gchar *text = g_queue_pop_head(&new_lines);
        gtk_text_buffer_get_end_iter(buffer, &end);
        gtk_text_buffer_insert(buffer, &end, text, -1);
    }

    if (view->end_mark != NULL)
    {
        gtk_text_buffer_get_end_iter(buffer, &end);
        gtk_text_buffer_move_mark(buffer, view->end_mark, &end);

        if (view->follow_toggle &&
            gtk_toggle_button_get_active(GTK_TOGGLE_BUTTON(view->follow_toggle)))
        {
            gtk_text_view_scroll_to_mark(GTK_TEXT_VIEW(view->text_view),
                                         view->end_mark, 0.0, TRUE, 0.0, 0.0);
        }
    }

    return G_SOURCE_REMOVE;
}

static void gp_term_view_schedule_flush(GpTermView *view)
{
    if (view == NULL)
        return;

    g_mutex_lock(&view->mutex);
    if (view->flush_source == 0)
        view->flush_source = g_idle_add(gp_term_view_flush, view);
    g_mutex_unlock(&view->mutex);
}

GpTermView *gp_term_view_new(const gchar *follow_label,
                             gboolean show_copy,
                             gboolean start_visible)
{
    GpTermView *view;
    GtkWidget *outer;
    GtkWidget *scroller;
    GtkWidget *button_box;
    GtkWidget *button;
    GtkTextBuffer *buffer;
    GtkTextIter start;
    const gchar *follow = follow_label ? follow_label : _("Auto-scroll");

    view = g_new0(GpTermView, 1);
    view->lines = g_queue_new();
    view->next_seq = 1;
    view->flushed_seq = 0;
    view->trim_pending = 0;
    view->flush_source = 0;
    g_mutex_init(&view->mutex);

    view->revealer = gtk_revealer_new();
    gtk_revealer_set_transition_type(GTK_REVEALER(view->revealer),
                                     GTK_REVEALER_TRANSITION_TYPE_SLIDE_DOWN);
    gtk_revealer_set_transition_duration(GTK_REVEALER(view->revealer), 150);
    gtk_revealer_set_reveal_child(GTK_REVEALER(view->revealer), start_visible);

    outer = gtk_box_new(GTK_ORIENTATION_VERTICAL, 6);
    gtk_container_set_border_width(GTK_CONTAINER(outer), 6);
    gtk_container_add(GTK_CONTAINER(view->revealer), outer);

    scroller = gtk_scrolled_window_new(NULL, NULL);
    gtk_scrolled_window_set_policy(GTK_SCROLLED_WINDOW(scroller),
                                   GTK_POLICY_AUTOMATIC,
                                   GTK_POLICY_AUTOMATIC);
    gtk_scrolled_window_set_shadow_type(GTK_SCROLLED_WINDOW(scroller),
                                        GTK_SHADOW_IN);
    gtk_widget_set_size_request(scroller, -1, 160);
    gtk_box_pack_start(GTK_BOX(outer), scroller, TRUE, TRUE, 0);

    view->text_view = gtk_text_view_new();
    gtk_text_view_set_editable(GTK_TEXT_VIEW(view->text_view), FALSE);
    gtk_text_view_set_cursor_visible(GTK_TEXT_VIEW(view->text_view), FALSE);
    gtk_text_view_set_wrap_mode(GTK_TEXT_VIEW(view->text_view), GTK_WRAP_NONE);
#if GTK_CHECK_VERSION(3, 16, 0)
    gtk_text_view_set_monospace(GTK_TEXT_VIEW(view->text_view), TRUE);
#else
    {
        PangoFontDescription *font;

        font = pango_font_description_from_string("Monospace");
        gtk_widget_override_font(view->text_view, font);
        pango_font_description_free(font);
    }
#endif
    gtk_container_add(GTK_CONTAINER(scroller), view->text_view);

    buffer = gtk_text_view_get_buffer(GTK_TEXT_VIEW(view->text_view));
    gtk_text_buffer_set_text(buffer, "", -1);
    gtk_text_buffer_get_start_iter(buffer, &start);
    view->end_mark = gtk_text_buffer_create_mark(buffer, NULL, &start, FALSE);

    button_box = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 6);
    gtk_box_pack_start(GTK_BOX(outer), button_box, FALSE, FALSE, 0);

    button = gtk_button_new_with_label(_("Clear"));
    g_signal_connect(button, "clicked",
                     G_CALLBACK(gp_term_view_clear_clicked), view);
    gtk_box_pack_start(GTK_BOX(button_box), button, FALSE, FALSE, 0);

    if (show_copy)
    {
        button = gtk_button_new_with_label(_("Copy"));
        g_signal_connect(button, "clicked",
                         G_CALLBACK(gp_term_view_copy_clicked), view);
        gtk_box_pack_start(GTK_BOX(button_box), button, FALSE, FALSE, 0);
    }

    view->follow_toggle = gtk_check_button_new_with_label(follow);
    gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(view->follow_toggle), TRUE);
    gtk_box_pack_end(GTK_BOX(button_box), view->follow_toggle, FALSE, FALSE, 0);

    return view;
}

GtkWidget *gp_term_view_get_widget(GpTermView *view)
{
    if (view == NULL)
        return NULL;

    return view->revealer;
}

void gp_term_view_set_visible(GpTermView *view, gboolean visible)
{
    if (view == NULL || view->revealer == NULL)
        return;

    if (visible)
    {
        gtk_widget_show(view->revealer);
        gtk_revealer_set_reveal_child(GTK_REVEALER(view->revealer), TRUE);
    }
    else
    {
        gtk_revealer_set_reveal_child(GTK_REVEALER(view->revealer), FALSE);
        gtk_widget_hide(view->revealer);
    }
}

void gp_term_view_clear(GpTermView *view)
{
    gp_term_view_clear_buffer(view);
}

void gp_term_view_free(GpTermView *view)
{
    if (view == NULL)
        return;

    gp_term_view_clear_buffer(view);

    if (view->lines)
        g_queue_free(view->lines);

    if (GTK_IS_WIDGET(view->revealer))
        gtk_widget_destroy(view->revealer);

    g_mutex_clear(&view->mutex);
    g_free(view);
}

void gp_term_view_log(GpTermView *view, const gchar *fmt, ...)
{
    va_list ap;
    gchar *msg;
    GpTermViewLine *line;

    if (view == NULL || fmt == NULL)
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

    line = g_new0(GpTermViewLine, 1);
    line->text = msg;

    g_mutex_lock(&view->mutex);
    line->seq = view->next_seq++;
    g_queue_push_tail(view->lines, line);

    if (g_queue_get_length(view->lines) > GP_TERM_VIEW_MAX_LINES)
    {
        GpTermViewLine *old = g_queue_pop_head(view->lines);
        gp_term_view_free_line(old, NULL);
        view->trim_pending++;
    }
    g_mutex_unlock(&view->mutex);

    gp_term_view_schedule_flush(view);
}
