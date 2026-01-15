/*
    Gpredict: Real-time satellite tracking and orbit prediction program

    Copyright (C)  2001-2017  Alexandru Csete, OZ9AEC.
 
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
#include <glib/gi18n.h>
#include <glib/gstdio.h>
#include <gtk/gtk.h>

#include "compat.h"
#include "gpredict-utils.h"
#include "rotor-conf.h"
#include "sat-cfg.h"
#include "sat-log.h"
#include "sat-pref-rot.h"
#include "sat-pref-rot-data.h"
#include "sat-pref-rot-editor.h"


extern GtkWidget *window;       /* dialog window defined in sat-pref.c */
static GtkWidget *addbutton;
static GtkWidget *editbutton;
static GtkWidget *delbutton;
static GtkWidget *rotlist;

typedef struct {
    GtkListStore *store;
    GtkTreeIter   iter;
    gboolean      have_iter;
} RotPrefEditorContext;

static void rot_pref_store_set(GtkListStore *store,
                               GtkTreeIter *iter,
                               const rotor_conf_t *conf)
{
    gtk_list_store_set(store, iter,
                       ROT_LIST_COL_NAME, conf->name,
                       ROT_LIST_COL_HOST, conf->host,
                       ROT_LIST_COL_PORT, conf->port,
                       ROT_LIST_COL_PROTOCOL, conf->protocol,
                       ROT_LIST_COL_BAUD, conf->baud,
                       ROT_LIST_COL_DEVICE, conf->device,
                       ROT_LIST_COL_DEVICE_MANUAL, conf->device_manual,
                       ROT_LIST_COL_DEVICE_AUTOPICK, conf->device_autopick,
                       ROT_LIST_COL_AUTOSTART, conf->autostart,
                       ROT_LIST_COL_MINAZ, conf->minaz,
                       ROT_LIST_COL_MAXAZ, conf->maxaz,
                       ROT_LIST_COL_MINEL, conf->minel,
                       ROT_LIST_COL_MAXEL, conf->maxel,
                       ROT_LIST_COL_AZTYPE, conf->aztype,
                       ROT_LIST_COL_AZSTOPPOS, conf->azstoppos,
                       ROT_LIST_COL_AXIS_MODE, conf->axis_mode,
                       ROT_LIST_COL_USE_OFFSET, conf->use_offset,
                       ROT_LIST_COL_AZ_OFFSET, conf->az_offset,
                       ROT_LIST_COL_EL_OFFSET, conf->el_offset,
                       ROT_LIST_COL_AZ_INVERT, conf->invert_az,
                       ROT_LIST_COL_EL_INVERT, conf->invert_el,
                       -1);
}

static void rot_pref_free_conf(rotor_conf_t *conf)
{
    if (conf == NULL)
        return;

    g_free(conf->name);
    g_free(conf->host);
    g_free(conf->device);
    g_free(conf->device_manual);
    g_free(conf);
}

static void rot_pref_editor_done(rotor_conf_t *conf,
                                 gboolean applied,
                                 gpointer user_data)
{
    RotPrefEditorContext *ctx = user_data;

    if (applied && conf != NULL && conf->name != NULL)
    {
        if (ctx != NULL && ctx->store != NULL)
        {
            if (!ctx->have_iter)
                gtk_list_store_append(ctx->store, &ctx->iter);
            rot_pref_store_set(ctx->store, &ctx->iter, conf);
        }
    }

    rot_pref_free_conf(conf);

    if (ctx != NULL)
    {
        if (ctx->store)
            g_object_unref(ctx->store);
        g_free(ctx);
    }
}


static void add_cb(GtkWidget * button, gpointer data)
{
    (void)button;
    (void)data;

    rotor_conf_t *conf = g_new0(rotor_conf_t, 1);
    RotPrefEditorContext *ctx = g_new0(RotPrefEditorContext, 1);

    *conf = (rotor_conf_t){
        .name = NULL,
        .host = NULL,
        .port = 4533,
        .protocol = ROT_PROTOCOL_GS232B,
        .baud = 0,
        .device = NULL,
        .device_manual = NULL,
        .device_autopick = TRUE,
        .autostart = TRUE,
        .minaz = 0,
        .maxaz = 360,
        .minel = 0,
        .maxel = 90,
        .aztype = ROT_AZ_TYPE_360,
        .azstoppos = 0,
        .axis_mode = ROT_AXIS_MODE_AZ_EL,
        .use_offset = FALSE,
        .az_offset = 0.0,
        .el_offset = 0.0,
        .invert_az = FALSE,
        .invert_el = FALSE,
    };

    conf->baud = rot_protocol_default_baud(conf->protocol);

    ctx->store =
        GTK_LIST_STORE(gtk_tree_view_get_model(GTK_TREE_VIEW(rotlist)));
    g_object_ref(ctx->store);
    ctx->have_iter = FALSE;

    sat_pref_rot_editor_run(conf, rot_pref_editor_done, ctx);
}

static void edit_cb(GtkWidget * button, gpointer data)
{
    GtkTreeModel   *model = gtk_tree_view_get_model(GTK_TREE_VIEW(rotlist));
    GtkTreeModel   *selmod;
    GtkTreeSelection *selection;
    GtkTreeIter     iter;
    rotor_conf_t   *conf = NULL;
    RotPrefEditorContext *ctx = NULL;

    (void)button;               /* avoid unused parameter compiler warning */
    (void)data;                 /* avoid unused parameter compiler warning */

    /* If there are no entries, we have a bug since the button should 
       have been disabled. */
    if (gtk_tree_model_iter_n_children(model, NULL) < 1)
    {

        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%s: Edit button should have been disabled."),
                    __FILE__, __func__);
        //gtk_widget_set_sensitive (button, FALSE);

        return;
    }

    /* get selected row
       FIXME: do we really need to work with two models?
     */
    selection = gtk_tree_view_get_selection(GTK_TREE_VIEW(rotlist));
    if (!gtk_tree_selection_get_selected(selection, &selmod, &iter))
    {
        GtkWidget      *dialog;

        dialog = gtk_message_dialog_new(GTK_WINDOW(window),
                                        GTK_DIALOG_DESTROY_WITH_PARENT,
                                        GTK_MESSAGE_ERROR,
                                        GTK_BUTTONS_OK,
                                        _
                                        ("Select the rotator you want to edit\n"
                                         "and try again!"));
        g_signal_connect_swapped(dialog, "response",
                                 G_CALLBACK(gtk_widget_destroy), dialog);
        gtk_widget_show(dialog);

        return;
    }

    conf = g_new0(rotor_conf_t, 1);
    *conf = (rotor_conf_t){
        .name = NULL,
        .host = NULL,
        .port = 4533,
        .protocol = ROT_PROTOCOL_GS232B,
        .baud = 0,
        .device = NULL,
        .device_manual = NULL,
        .device_autopick = TRUE,
        .autostart = TRUE,
        .minaz = 0,
        .maxaz = 360,
        .minel = 0,
        .maxel = 90,
        .aztype = ROT_AZ_TYPE_360,
        .azstoppos = 0,
        .axis_mode = ROT_AXIS_MODE_AZ_EL,
        .use_offset = FALSE,
        .az_offset = 0.0,
        .el_offset = 0.0,
        .invert_az = FALSE,
        .invert_el = FALSE,
    };

    conf->baud = rot_protocol_default_baud(conf->protocol);

    gtk_tree_model_get(model, &iter,
                       ROT_LIST_COL_NAME, &conf->name,
                       ROT_LIST_COL_HOST, &conf->host,
                       ROT_LIST_COL_PORT, &conf->port,
                       ROT_LIST_COL_PROTOCOL, &conf->protocol,
                       ROT_LIST_COL_BAUD, &conf->baud,
                       ROT_LIST_COL_DEVICE, &conf->device,
                       ROT_LIST_COL_DEVICE_MANUAL, &conf->device_manual,
                       ROT_LIST_COL_DEVICE_AUTOPICK, &conf->device_autopick,
                       ROT_LIST_COL_AUTOSTART, &conf->autostart,
                       ROT_LIST_COL_MINAZ, &conf->minaz,
                       ROT_LIST_COL_MAXAZ, &conf->maxaz,
                       ROT_LIST_COL_MINEL, &conf->minel,
                       ROT_LIST_COL_MAXEL, &conf->maxel,
                       ROT_LIST_COL_AZTYPE, &conf->aztype,
                       ROT_LIST_COL_AZSTOPPOS, &conf->azstoppos,
                       ROT_LIST_COL_AXIS_MODE, &conf->axis_mode,
                       ROT_LIST_COL_USE_OFFSET, &conf->use_offset,
                       ROT_LIST_COL_AZ_OFFSET, &conf->az_offset,
                       ROT_LIST_COL_EL_OFFSET, &conf->el_offset,
                       ROT_LIST_COL_AZ_INVERT, &conf->invert_az,
                       ROT_LIST_COL_EL_INVERT, &conf->invert_el,
                       -1);

    ctx = g_new0(RotPrefEditorContext, 1);
    ctx->store = GTK_LIST_STORE(model);
    g_object_ref(ctx->store);
    ctx->iter = iter;
    ctx->have_iter = TRUE;

    sat_pref_rot_editor_run(conf, rot_pref_editor_done, ctx);
}

static void delete_cb(GtkWidget * button, gpointer data)
{
    GtkTreeModel   *model = gtk_tree_view_get_model(GTK_TREE_VIEW(rotlist));
    GtkTreeSelection *selection;
    GtkTreeIter     iter;

    (void)button;
    (void)data;

    /* If there are no entries, we have a bug since the button should 
       have been disabled. */
    if (gtk_tree_model_iter_n_children(model, NULL) < 1)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%s: Delete button should have been disabled."),
                    __FILE__, __func__);
        //gtk_widget_set_sensitive (button, FALSE);

        return;
    }

    /* get selected row */
    selection = gtk_tree_view_get_selection(GTK_TREE_VIEW(rotlist));
    if (gtk_tree_selection_get_selected(selection, NULL, &iter))
    {
        gtk_list_store_remove(GTK_LIST_STORE(model), &iter);
    }
    else
    {
        GtkWidget      *dialog;

        dialog = gtk_message_dialog_new(GTK_WINDOW(window),
                                        GTK_DIALOG_DESTROY_WITH_PARENT,
                                        GTK_MESSAGE_ERROR,
                                        GTK_BUTTONS_OK,
                                        _
                                        ("Select the rotator you want to delete\n"
                                         "and try again!"));
        g_signal_connect_swapped(dialog, "response",
                                 G_CALLBACK(gtk_widget_destroy), dialog);
        gtk_widget_show(dialog);
    }
}

static void row_activated_cb(GtkTreeView * tree_view,
                             GtkTreePath * path,
                             GtkTreeViewColumn * column, gpointer user_data)
{
    (void)tree_view;
    (void)path;
    (void)column;
    (void)user_data;

    edit_cb(editbutton, NULL);
}
static GtkTreeModel *create_and_fill_model()
{
    GtkListStore   *liststore;  /* the list store data structure */
    GtkTreeIter     item;       /* new item added to the list store */
    GDir           *dir = NULL; /* directory handle */
    GError         *error = NULL;       /* error flag and info */
    gchar          *dirname;    /* directory name */
    gchar         **vbuff;
    const gchar    *filename;   /* file name */
    rotor_conf_t    conf;

    /* create a new list store */
    liststore = gtk_list_store_new(ROT_LIST_COL_NUM, G_TYPE_STRING,     // name
                                   G_TYPE_STRING,       // host
                                   G_TYPE_INT,  // port
                                   G_TYPE_INT,          // protocol
                                   G_TYPE_INT,          // baud
                                   G_TYPE_STRING,       // device
                                   G_TYPE_STRING,       // manual device
                                   G_TYPE_BOOLEAN,      // device autopick
                                   G_TYPE_BOOLEAN,      // autostart
                                   G_TYPE_DOUBLE,       // Min Az
                                   G_TYPE_DOUBLE,       // Max Az
                                   G_TYPE_DOUBLE,       // Min El
                                   G_TYPE_DOUBLE,       // Max El
                                   G_TYPE_INT,  // Az type
                                   G_TYPE_DOUBLE,       // Az Stop Position
                                   G_TYPE_INT,          // Axis mode
                                   G_TYPE_BOOLEAN,      // Use offset
                                   G_TYPE_DOUBLE,       // Az offset
                                   G_TYPE_DOUBLE,       // El offset
                                   G_TYPE_BOOLEAN,      // Az invert
                                   G_TYPE_BOOLEAN       // El invert
        );
    gtk_tree_sortable_set_sort_column_id(GTK_TREE_SORTABLE(liststore),
                                         ROT_LIST_COL_NAME,
                                         GTK_SORT_ASCENDING);
    /* open configuration directory */
    dirname = get_hwconf_dir();

    dir = g_dir_open(dirname, 0, &error);
    if (dir)
    {
        /* read each .rot file */
        while ((filename = g_dir_read_name(dir)))
        {
            if (g_str_has_suffix(filename, ".rot"))
            {
                vbuff = g_strsplit(filename, ".rot", 0);
                conf.name = g_strdup(vbuff[0]);
                g_strfreev(vbuff);
                if (rotor_conf_read(&conf))
                {
                    /* insert conf into liststore */
                    gtk_list_store_append(liststore, &item);
                    gtk_list_store_set(liststore, &item,
                                       ROT_LIST_COL_NAME, conf.name,
                                       ROT_LIST_COL_HOST, conf.host,
                                       ROT_LIST_COL_PORT, conf.port,
                                       ROT_LIST_COL_PROTOCOL, conf.protocol,
                                       ROT_LIST_COL_BAUD, conf.baud,
                                       ROT_LIST_COL_DEVICE, conf.device,
                                       ROT_LIST_COL_DEVICE_MANUAL, conf.device_manual,
                                       ROT_LIST_COL_DEVICE_AUTOPICK, conf.device_autopick,
                                       ROT_LIST_COL_AUTOSTART, conf.autostart,
                                       ROT_LIST_COL_MINAZ, conf.minaz,
                                       ROT_LIST_COL_MAXAZ, conf.maxaz,
                                       ROT_LIST_COL_MINEL, conf.minel,
                                       ROT_LIST_COL_MAXEL, conf.maxel,
                                       ROT_LIST_COL_AZTYPE, conf.aztype,
                                       ROT_LIST_COL_AZSTOPPOS, conf.azstoppos,
                                       ROT_LIST_COL_AXIS_MODE, conf.axis_mode,
                                       ROT_LIST_COL_USE_OFFSET, conf.use_offset,
                                       ROT_LIST_COL_AZ_OFFSET, conf.az_offset,
                                       ROT_LIST_COL_EL_OFFSET, conf.el_offset,
                                       ROT_LIST_COL_AZ_INVERT, conf.invert_az,
                                       ROT_LIST_COL_EL_INVERT, conf.invert_el,
                                       -1);

                    sat_log_log(SAT_LOG_LEVEL_DEBUG,
                                _("%s:%d: Read %s"),
                                __FILE__, __LINE__, filename);
                    /* clean up memory */
                    if (conf.name)
                        g_free(conf.name);

                    if (conf.host)
                        g_free(conf.host);
                    if (conf.device)
                        g_free(conf.device);
                    if (conf.device_manual)
                        g_free(conf.device_manual);
                }
                else
                {
                    /* there was an error */
                    sat_log_log(SAT_LOG_LEVEL_ERROR,
                                _("%s:%d: Failed to read %s"),
                                __FILE__, __LINE__, conf.name);

                    g_free(conf.name);
                }
            }
        }
    }
    else
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("%s:%d: Failed to open hwconf dir (%s)"),
                    __FILE__, __LINE__, error->message);
        g_clear_error(&error);
    }

    g_free(dirname);
    g_dir_close(dir);

    return GTK_TREE_MODEL(liststore);
}

/**
 * Create buttons.
 * @return A button box containing the buttons.
 *
 * This function creates and initialises the three buttons below the rot list.
 * The treeview widget is needed by the buttons when they are activated.
 *
 */
static GtkWidget *create_buttons(void)
{
    GtkWidget      *box;

    addbutton = gtk_button_new_with_label(_("Add New"));
    gtk_widget_set_tooltip_text(addbutton, _("Add a new rotator"));
    g_signal_connect(addbutton, "clicked", G_CALLBACK(add_cb), NULL);

    editbutton = gtk_button_new_with_label(_("Edit"));
    gtk_widget_set_tooltip_text(editbutton, _("Edit selected rotator"));
    g_signal_connect(editbutton, "clicked", G_CALLBACK(edit_cb), NULL);

    delbutton = gtk_button_new_with_label(_("Delete"));
    gtk_widget_set_tooltip_text(delbutton, _("Delete selected rotator"));
    g_signal_connect(delbutton, "clicked", G_CALLBACK(delete_cb), NULL);

    box = gtk_button_box_new(GTK_ORIENTATION_HORIZONTAL);
    gtk_button_box_set_layout(GTK_BUTTON_BOX(box), GTK_BUTTONBOX_START);

    gtk_container_add(GTK_CONTAINER(box), addbutton);
    gtk_container_add(GTK_CONTAINER(box), editbutton);
    gtk_container_add(GTK_CONTAINER(box), delbutton);

    return box;
}

/**
 * Render an angle.
 *
 * @param col Pointer to the tree view column.
 * @param renderer Pointer to the renderer.
 * @param model Pointer to the tree model.
 * @param iter Pointer to the tree iterator.
 * @param column The column number in the model.
 */
static void render_angle(GtkTreeViewColumn * col,
                         GtkCellRenderer * renderer,
                         GtkTreeModel * model,
                         GtkTreeIter * iter, gpointer column)
{
    gdouble         number;
    guint           coli = GPOINTER_TO_UINT(column);
    gchar          *text;

    (void)col;

    gtk_tree_model_get(model, iter, coli, &number, -1);

    text = g_strdup_printf("%.0f\302\260", number);
    g_object_set(renderer, "text", text, NULL);
    g_free(text);
}

/**
 * Render the azimuth type.
 *
 * @param col Pointer to the tree view column.
 * @param renderer Pointer to the renderer.
 * @param model Pointer to the tree model.
 * @param iter Pointer to the tree iterator.
 * @param column The column number in the model.
 */
static void render_aztype(GtkTreeViewColumn * col,
                          GtkCellRenderer * renderer,
                          GtkTreeModel * model,
                          GtkTreeIter * iter, gpointer column)
{
    gint            number;
    guint           coli = GPOINTER_TO_UINT(column);
    gchar          *text;

    (void)col;

    gtk_tree_model_get(model, iter, coli, &number, -1);

    switch (number)
    {
    case ROT_AZ_TYPE_360:
        text = g_strdup_printf
            ("0\302\260 \342\206\222 180\302\260 \342\206\222 360\302\260");
        break;

    case ROT_AZ_TYPE_180:
        text = g_strdup_printf
            ("-180\302\260 \342\206\222 0\302\260 \342\206\222 +180\302\260");
        break;

    default:
        text = g_strdup_printf(_("Unknown (%d)"), number);
        break;
    }

    g_object_set(renderer, "text", text, NULL);
    g_free(text);
}

static void create_rot_list()
{
    GtkTreeModel   *model;
    GtkCellRenderer *renderer;
    GtkTreeViewColumn *column;

    rotlist = gtk_tree_view_new();

    model = create_and_fill_model();
    gtk_tree_view_set_model(GTK_TREE_VIEW(rotlist), model);
    g_object_unref(model);

    /* Conf name */
    renderer = gtk_cell_renderer_text_new();
    column =
        gtk_tree_view_column_new_with_attributes(_("Config Name"), renderer,
                                                 "text", ROT_LIST_COL_NAME,
                                                 NULL);
    gtk_tree_view_insert_column(GTK_TREE_VIEW(rotlist), column, -1);

    /* Host */
    renderer = gtk_cell_renderer_text_new();
    column = gtk_tree_view_column_new_with_attributes(_("Host"), renderer,
                                                      "text",
                                                      ROT_LIST_COL_HOST, NULL);
    gtk_tree_view_insert_column(GTK_TREE_VIEW(rotlist), column, -1);

    /* port */
    renderer = gtk_cell_renderer_text_new();
    column = gtk_tree_view_column_new_with_attributes(_("Port"), renderer,
                                                      "text",
                                                      ROT_LIST_COL_PORT, NULL);
    gtk_tree_view_insert_column(GTK_TREE_VIEW(rotlist), column, -1);

    /* Az and el limits */
    renderer = gtk_cell_renderer_text_new();
    column = gtk_tree_view_column_new_with_attributes(_("Min Az"), renderer,
                                                      "text",
                                                      ROT_LIST_COL_MINAZ,
                                                      NULL);
    gtk_tree_view_column_set_cell_data_func(column, renderer, render_angle,
                                            GUINT_TO_POINTER
                                            (ROT_LIST_COL_MINAZ), NULL);
    gtk_tree_view_insert_column(GTK_TREE_VIEW(rotlist), column, -1);

    renderer = gtk_cell_renderer_text_new();
    column = gtk_tree_view_column_new_with_attributes(_("Max Az"), renderer,
                                                      "text",
                                                      ROT_LIST_COL_MAXAZ,
                                                      NULL);
    gtk_tree_view_column_set_cell_data_func(column, renderer, render_angle,
                                            GUINT_TO_POINTER
                                            (ROT_LIST_COL_MAXAZ), NULL);
    gtk_tree_view_insert_column(GTK_TREE_VIEW(rotlist), column, -1);

    renderer = gtk_cell_renderer_text_new();
    column = gtk_tree_view_column_new_with_attributes(_("Min El"), renderer,
                                                      "text",
                                                      ROT_LIST_COL_MINEL,
                                                      NULL);
    gtk_tree_view_column_set_cell_data_func(column, renderer, render_angle,
                                            GUINT_TO_POINTER
                                            (ROT_LIST_COL_MINEL), NULL);
    gtk_tree_view_insert_column(GTK_TREE_VIEW(rotlist), column, -1);

    renderer = gtk_cell_renderer_text_new();
    column = gtk_tree_view_column_new_with_attributes(_("Max El"), renderer,
                                                      "text",
                                                      ROT_LIST_COL_MAXEL,
                                                      NULL);
    gtk_tree_view_column_set_cell_data_func(column, renderer, render_angle,
                                            GUINT_TO_POINTER
                                            (ROT_LIST_COL_MAXEL), NULL);
    gtk_tree_view_insert_column(GTK_TREE_VIEW(rotlist), column, -1);

    /* Az type */
    renderer = gtk_cell_renderer_text_new();
    column =
        gtk_tree_view_column_new_with_attributes(_("Azimuth Type"), renderer,
                                                 "text", ROT_LIST_COL_AZTYPE,
                                                 NULL);
    gtk_tree_view_column_set_cell_data_func(column, renderer, render_aztype,
                                            GUINT_TO_POINTER
                                            (ROT_LIST_COL_AZTYPE), NULL);
    gtk_tree_view_insert_column(GTK_TREE_VIEW(rotlist), column, -1);

    g_signal_connect(rotlist, "row-activated", G_CALLBACK(row_activated_cb),
                     NULL);

}

GtkWidget      *sat_pref_rot_create()
{
    GtkWidget      *vbox;       /* vbox containing the list part and the details part */
    GtkWidget      *swin;

    vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 10);
    gtk_box_set_homogeneous(GTK_BOX(vbox), FALSE);
    gtk_container_set_border_width(GTK_CONTAINER(vbox), 10);

    /* create rot list and pack into scrolled window */
    create_rot_list();
    swin = gtk_scrolled_window_new(NULL, NULL);
    gtk_container_add(GTK_CONTAINER(swin), rotlist);

    gtk_box_pack_start(GTK_BOX(vbox), swin, TRUE, TRUE, 0);
    gtk_box_pack_start(GTK_BOX(vbox), create_buttons(), FALSE, FALSE, 0);

    return vbox;
}

/** User pressed cancel. Any changes to config must be cancelled. */
void sat_pref_rot_cancel()
{
}

/**
 * User pressed OK. Any changes should be stored in config.
 * 
 * First, all .rot files are deleted, whereafter the rotator configurations in
 * the rotlist are saved one by one.
 */
void sat_pref_rot_ok()
{
    GDir           *dir = NULL; /* directory handle */
    GError         *error = NULL;       /* error flag and info */
    gchar          *buff, *dirname;
    const gchar    *filename;
    GtkTreeIter     iter;       /* new item added to the list store */
    GtkTreeModel   *model;
    guint           i, n;

    rotor_conf_t    conf = {
        .name = NULL,
        .host = NULL,
        .port = 4533,
        .protocol = ROT_PROTOCOL_GS232B,
        .baud = 0,
        .device = NULL,
        .device_manual = NULL,
        .device_autopick = TRUE,
        .autostart = TRUE,
        .minaz = 0,
        .maxaz = 360,
        .minel = 0,
        .maxel = 90,
        .aztype = ROT_AZ_TYPE_360,
        .azstoppos = 0,
        .axis_mode = ROT_AXIS_MODE_AZ_EL,
        .use_offset = FALSE,
        .az_offset = 0.0,
        .el_offset = 0.0,
        .invert_az = FALSE,
        .invert_el = FALSE,
    };

    conf.baud = rot_protocol_default_baud(conf.protocol);


    /* delete all .rot files */
    dirname = get_hwconf_dir();

    dir = g_dir_open(dirname, 0, &error);
    if (dir)
    {
        /* read each .rot file */
        while ((filename = g_dir_read_name(dir)))
        {
            if (g_str_has_suffix(filename, ".rot"))
            {

                buff = g_strconcat(dirname, G_DIR_SEPARATOR_S, filename, NULL);
                if (g_remove(buff))
                    sat_log_log(SAT_LOG_LEVEL_ERROR,
                                _("%s: Failed to remove %s"), __func__, buff);
                g_free(buff);
            }
        }
    }

    g_free(dirname);
    g_dir_close(dir);

    /* create new .rot files for the radios in the rotlist */
    model = gtk_tree_view_get_model(GTK_TREE_VIEW(rotlist));
    n = gtk_tree_model_iter_n_children(model, NULL);
    for (i = 0; i < n; i++)
    {
        /* get radio conf */
        if (gtk_tree_model_iter_nth_child(model, &iter, NULL, i))
        {

            /* store conf */
            gtk_tree_model_get(model, &iter,
                               ROT_LIST_COL_NAME, &conf.name,
                               ROT_LIST_COL_HOST, &conf.host,
                               ROT_LIST_COL_PORT, &conf.port,
                               ROT_LIST_COL_PROTOCOL, &conf.protocol,
                               ROT_LIST_COL_BAUD, &conf.baud,
                               ROT_LIST_COL_DEVICE, &conf.device,
                               ROT_LIST_COL_DEVICE_MANUAL, &conf.device_manual,
                               ROT_LIST_COL_DEVICE_AUTOPICK, &conf.device_autopick,
                               ROT_LIST_COL_AUTOSTART, &conf.autostart,
                               ROT_LIST_COL_MINAZ, &conf.minaz,
                               ROT_LIST_COL_MAXAZ, &conf.maxaz,
                               ROT_LIST_COL_MINEL, &conf.minel,
                               ROT_LIST_COL_MAXEL, &conf.maxel,
                               ROT_LIST_COL_AZTYPE, &conf.aztype,
                               ROT_LIST_COL_AZSTOPPOS, &conf.azstoppos,
                               ROT_LIST_COL_AXIS_MODE, &conf.axis_mode,
                               ROT_LIST_COL_USE_OFFSET, &conf.use_offset,
                               ROT_LIST_COL_AZ_OFFSET, &conf.az_offset,
                               ROT_LIST_COL_EL_OFFSET, &conf.el_offset,
                               ROT_LIST_COL_AZ_INVERT, &conf.invert_az,
                               ROT_LIST_COL_EL_INVERT, &conf.invert_el,
                               -1);
            rotor_conf_save(&conf);

            /* free conf buffer */
            if (conf.name)
                g_free(conf.name);

            if (conf.host)
                g_free(conf.host);
            if (conf.device)
                g_free(conf.device);
            if (conf.device_manual)
                g_free(conf.device_manual);

        }
        else
        {
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: Failed to get ROT %s"), __func__, i);
        }
    }
}
