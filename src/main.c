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
#include <signal.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#ifdef G_OS_WIN32
#include <winsock2.h>
#endif

#include "compat.h"
#include "gtk-sat-selector.h"
#include "gui.h"
#include "first-time.h"
#include "tle-update.h"
#include "mod-mgr.h"
#include "sat-cfg.h"
#include "sat-log.h"

#ifndef GPREDICT_FULL_VERSION
#if defined(PACKAGE_STRING)
#define GPREDICT_FULL_VERSION PACKAGE_STRING
#elif defined(PACKAGE_VERSION)
#define GPREDICT_FULL_VERSION PACKAGE_VERSION
#elif defined(VERSION)
#define GPREDICT_FULL_VERSION VERSION
#else
#define GPREDICT_FULL_VERSION "unknown"
#endif
#endif


/* Main application widget. */
GtkWidget      *app;

/* Command line flag for cleaning TLE data. */
static gboolean cleantle = FALSE;

/* Command line flag for cleaning TRSP data */
static gboolean cleantrsp = FALSE;

/* Start application in fullscreen mode */
static gboolean fullscreen = FALSE;

/* Command line options. */
static GOptionEntry entries[] = {
    {"clean-tle", 0, 0, G_OPTION_ARG_NONE, &cleantle,
     "Clean the TLE data in user's configuration directory", NULL},
    {"clean-trsp", 0, 0, G_OPTION_ARG_NONE, &cleantrsp,
     "Clean the transponder data in user's configuration directory", NULL},
    {"fullscreen", 0, 0, G_OPTION_ARG_NONE, &fullscreen,
     "Start gpredict in fullscreen mode.", NULL},
    {NULL, 0, 0, 0, NULL, NULL, NULL}
};

const gchar    *dummy = N_("just to have a pot");

/* ID of TLE monitoring task */
static guint    tle_mon_id = 0;

/* flag indicating whether TLE update is running */
static gboolean tle_upd_running = FALSE;

/* flag indicating whether user has been notified of TLE update */
static gboolean tle_upd_note_sent = FALSE;


/* private function prototypes */
static void     gpredict_app_create(void);
static gint     gpredict_app_delete(GtkWidget *, GdkEvent *, gpointer);
static void     gpredict_app_destroy(GtkWidget *, gpointer);
static gboolean gpredict_app_config(GtkWidget *, GdkEventConfigure *,
                                    gpointer);
static void     gpredict_term_sig_handler(int sig);
static void     gpredict_fatal_sig_handler(int sig);
static void     gpredict_install_signal_handlers(void);
static gboolean tle_mon_task(gpointer data);
static void     tle_mon_stop(void);
static gpointer update_tle_thread(gpointer data);
static void     clean_tle(void);
static void     clean_trsp(void);

static void     InitWinSock2(void);
static void     CloseWinSock2(void);
#ifdef G_OS_WIN32
static void     gpredict_windows_bootstrap_log(const gchar *format, ...);
static void     gpredict_windows_init_runtime(void);
#else
#define gpredict_windows_bootstrap_log(...) ((void)0)
#define gpredict_windows_init_runtime() ((void)0)
#endif


int main(int argc, char *argv[])
{
    GError         *err = NULL;
    GOptionContext *context;
    guint           error = 0;

#ifdef G_OS_WIN32
    gpredict_windows_bootstrap_log("startup: entering main");
    gpredict_windows_init_runtime();
    gpredict_windows_bootstrap_log("startup: runtime initialized");
#endif

#ifdef ENABLE_NLS
#ifdef G_OS_WIN32
    {
        gchar *prefix = g_win32_get_package_installation_directory_of_module(NULL);
        gchar *locale_dir = g_build_filename(prefix, "share", "locale", NULL);

        bindtextdomain(PACKAGE, locale_dir);
        g_free(locale_dir);
        g_free(prefix);
    }
#else
    bindtextdomain(PACKAGE, PACKAGE_LOCALE_DIR);
#endif
    bind_textdomain_codeset(PACKAGE, "UTF-8");
    textdomain(PACKAGE);
#endif
    sat_log_init();
    gpredict_install_signal_handlers();
    gpredict_windows_bootstrap_log("startup: before gtk_init");
    if (!gtk_init_check(&argc, &argv))
    {
        gpredict_windows_bootstrap_log("startup: gtk_init_check failed");
        return 1;
    }
    gpredict_windows_bootstrap_log("startup: after gtk_init");
    {
        gchar *datadir = get_data_dir();
        gchar *prefix = NULL;
#ifdef G_OS_WIN32
        prefix = g_win32_get_package_installation_directory_of_module(NULL);
#else
        {
            gchar *parent = g_path_get_dirname(PACKAGE_DATA_DIR);
            prefix = g_path_get_dirname(parent);
            g_free(parent);
        }
#endif
        g_message("gpredict: version %s (prefix=%s datadir=%s)",
                  GPREDICT_FULL_VERSION,
                  prefix ? prefix : "(unknown)",
                  datadir ? datadir : "(unknown)");
        g_free(prefix);
        g_free(datadir);
    }

    context = g_option_context_new("");
    g_option_context_add_main_entries(context, entries, GETTEXT_PACKAGE);
    g_option_context_set_summary(context,
                                 _("Gpredict is a graphical real-time satellite "
                                   "tracking and orbit prediction program.\n"
                                   "Gpredict does not require any command line "
                                   "options for nominal operation."));
    g_option_context_add_group(context, gtk_get_option_group(TRUE));
    if (!g_option_context_parse(context, &argc, &argv, &err))
        g_print(_("Option parsing failed: %s\n"), err->message);

    sat_cfg_load();
    sat_log_set_level(sat_cfg_get_int(SAT_CFG_INT_LOG_LEVEL));
    sat_log_forensic(SAT_LOG_LEVEL_INFO, "main: runtime log level=%d",
                     sat_cfg_get_int(SAT_CFG_INT_LOG_LEVEL));

    if (cleantle)
        clean_tle();

    if (cleantrsp)
        clean_trsp();

    /* check that user settings are ok */
    error = first_time_check_run();
    if (error)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _("User config check failed (code %d). This is fatal.\n"
                      "A possible solution would be to remove the "
                      ".config/Gpredict data dir in your home directory"),
                     error);

        return 1;
    }

    /* create application */
    gpredict_windows_bootstrap_log("startup: before gpredict_app_create");
    gpredict_app_create();
    gpredict_windows_bootstrap_log("startup: after gpredict_app_create");
    gtk_widget_show_all(app);
    gpredict_windows_bootstrap_log("startup: after gtk_widget_show_all");
    if (fullscreen)
		gtk_window_fullscreen(GTK_WINDOW(app));

    //sat_debugger_run ();

    /* launch TLE monitoring task; 10 min interval */
    tle_mon_id = g_timeout_add(600000, tle_mon_task, NULL);

    InitWinSock2();

    gtk_main();
    sat_log_forensic(SAT_LOG_LEVEL_INFO, "main: gtk_main exited");

    g_option_context_free(context);

    sat_log_forensic(SAT_LOG_LEVEL_INFO, "main: sat_cfg_save begin");
    sat_cfg_save();
    sat_log_forensic(SAT_LOG_LEVEL_INFO, "main: sat_cfg_save end");
    sat_log_forensic(SAT_LOG_LEVEL_INFO, "main: sat_log_close begin");
    sat_log_close();
    sat_cfg_close();

    CloseWinSock2();

    return 0;
}

static void InitWinSock2(void)
{
#ifdef G_OS_WIN32
    WORD            wVersionRequested;
    WSADATA         wsaData;
    int             err;

    wVersionRequested = MAKEWORD(2, 2);

    err = WSAStartup(wVersionRequested, &wsaData);
    if (err != 0)
    {
        /* Tell the user that we could not find a usable */
        /* WinSock DLL.                                  */
        return;
    }

    /* Confirm that the WinSock DLL supports 2.2. */
    /* Note that if the DLL supports versions later    */
    /* than 2.2 in addition to 2.2, it will still return */
    /* 2.2 in wVersion since that is the version we      */
    /* requested.                                        */

    if (LOBYTE(wsaData.wVersion) != 2 || HIBYTE(wsaData.wVersion) != 2)
    {
        /* Tell the user that we could not find a usable */
        /* WinSock DLL.                                  */
        WSACleanup();
        return;
    }
#else
    return;
#endif
}

static void CloseWinSock2(void)
{
#ifdef G_OS_WIN32
    WSACleanup();
#else
    return;
#endif
}

#ifdef G_OS_WIN32
static void gpredict_windows_bootstrap_log(const gchar *format, ...)
{
    gchar *log_dir = NULL;
    gchar *log_file = NULL;
    FILE  *fp = NULL;
    va_list args;

    {
        const gchar *local_appdata = g_getenv("LOCALAPPDATA");
        if (local_appdata != NULL && *local_appdata != '\0')
            log_dir = g_build_filename(local_appdata, "Gpredict", "logs", NULL);
        else
            log_dir = g_build_filename(g_get_user_config_dir(), "Gpredict", "logs", NULL);
    }
    if (g_mkdir_with_parents(log_dir, 0700) != 0)
        goto cleanup;

    log_file = g_build_filename(log_dir, "bootstrap.log", NULL);
    fp = g_fopen(log_file, "a");
    if (fp == NULL)
        goto cleanup;

    va_start(args, format);
    vfprintf(fp, format, args);
    va_end(args);
    fputc('\n', fp);
    fclose(fp);

cleanup:
    g_free(log_dir);
    g_free(log_file);
}

static void gpredict_windows_setenv_if_unset(const gchar *name, const gchar *value)
{
    if (value != NULL && *value != '\0' && g_getenv(name) == NULL)
        g_setenv(name, value, TRUE);
}

static void gpredict_windows_prepend_env_path(const gchar *name,
                                              const gchar *value)
{
    const gchar *existing = g_getenv(name);
    gchar **parts = NULL;
    gchar *updated = NULL;

    if (value == NULL || *value == '\0')
        return;

    if (existing == NULL || *existing == '\0')
    {
        g_setenv(name, value, TRUE);
        return;
    }

    parts = g_strsplit(existing, G_SEARCHPATH_SEPARATOR_S, -1);
    for (gint i = 0; parts != NULL && parts[i] != NULL; i++)
    {
        if (g_strcmp0(parts[i], value) == 0)
        {
            g_strfreev(parts);
            return;
        }
    }
    g_strfreev(parts);

    updated = g_strdup_printf("%s%s%s", value, G_SEARCHPATH_SEPARATOR_S, existing);
    g_setenv(name, updated, TRUE);
    g_free(updated);
}

static void gpredict_windows_init_runtime(void)
{
    GDir           *pixbuf_versions = NULL;
    const gchar    *version_name = NULL;
    gchar          *prefix = NULL;
    gchar          *share_dir = NULL;
    gchar          *gio_module_dir = NULL;
    gchar          *schema_dir = NULL;
    gchar          *mime_dir = NULL;
    gchar          *pixbuf_root = NULL;
    gchar          *pixbuf_module_dir = NULL;
    gchar          *pixbuf_cache_file = NULL;
    gchar          *ca_bundle = NULL;
    gchar          *hamlib_override = NULL;

    prefix = g_win32_get_package_installation_directory_of_module(NULL);
    if (prefix == NULL || *prefix == '\0')
        goto cleanup;
    gpredict_windows_bootstrap_log("runtime: prefix=%s", prefix);

    share_dir = g_build_filename(prefix, "share", NULL);
    gio_module_dir = g_build_filename(prefix, "lib", "gio", "modules", NULL);
    schema_dir = g_build_filename(prefix, "share", "glib-2.0", "schemas", NULL);
    mime_dir = g_build_filename(prefix, "share", "mime", NULL);

    gpredict_windows_setenv_if_unset("GTK_DATA_PREFIX", prefix);
    gpredict_windows_setenv_if_unset("GTK_EXE_PREFIX", prefix);
    gpredict_windows_prepend_env_path("XDG_DATA_DIRS", share_dir);
    gpredict_windows_setenv_if_unset("GIO_MODULE_DIR", gio_module_dir);
    gpredict_windows_setenv_if_unset("GSETTINGS_SCHEMA_DIR", schema_dir);
    gpredict_windows_prepend_env_path("PATH", prefix);

    hamlib_override = g_strdup(g_getenv("GPREDICT_HAMLIB_DIR"));
    if (hamlib_override != NULL && *hamlib_override != '\0')
        gpredict_windows_prepend_env_path("PATH", hamlib_override);

    ca_bundle = g_build_filename(prefix, "curl-ca-bundle.crt", NULL);
    if (g_file_test(ca_bundle, G_FILE_TEST_EXISTS))
        gpredict_windows_setenv_if_unset("CURL_CA_BUNDLE", ca_bundle);
    gpredict_windows_bootstrap_log("runtime: mime_dir=%s exists=%d", mime_dir,
                                   g_file_test(mime_dir, G_FILE_TEST_IS_DIR));

    pixbuf_root = g_build_filename(prefix, "lib", "gdk-pixbuf-2.0", NULL);
    pixbuf_versions = g_dir_open(pixbuf_root, 0, NULL);
    if (pixbuf_versions == NULL)
        goto cleanup;

    while ((version_name = g_dir_read_name(pixbuf_versions)) != NULL)
    {
        gchar *candidate = g_build_filename(pixbuf_root, version_name, "loaders", NULL);
        if (g_file_test(candidate, G_FILE_TEST_IS_DIR))
        {
            pixbuf_module_dir = candidate;
            pixbuf_cache_file =
                g_build_filename(pixbuf_root, version_name, "loaders.cache", NULL);
            break;
        }
        g_free(candidate);
    }

    if (pixbuf_module_dir != NULL && pixbuf_cache_file != NULL)
    {
        gpredict_windows_setenv_if_unset("GDK_PIXBUF_MODULEDIR", pixbuf_module_dir);
        gpredict_windows_setenv_if_unset("GDK_PIXBUF_MODULE_FILE",
                                         pixbuf_cache_file);
        gpredict_windows_bootstrap_log("runtime: pixbuf_module_dir=%s exists=%d",
                                       pixbuf_module_dir,
                                       g_file_test(pixbuf_module_dir, G_FILE_TEST_IS_DIR));
        gpredict_windows_bootstrap_log("runtime: pixbuf_cache_file=%s exists=%d",
                                       pixbuf_cache_file,
                                       g_file_test(pixbuf_cache_file, G_FILE_TEST_EXISTS));
    }

    if (g_file_test(mime_dir, G_FILE_TEST_IS_DIR))
        gpredict_windows_setenv_if_unset("XDG_DATA_DIRS", share_dir);

cleanup:
    if (pixbuf_versions != NULL)
        g_dir_close(pixbuf_versions);
    g_free(prefix);
    g_free(share_dir);
    g_free(gio_module_dir);
    g_free(schema_dir);
    g_free(mime_dir);
    g_free(pixbuf_root);
    g_free(pixbuf_module_dir);
    g_free(pixbuf_cache_file);
    g_free(ca_bundle);
    g_free(hamlib_override);
}
#endif

/**
 * Create main application window.
 *
 * @return A new top level window as as GtkWidget.
 *
 * This function creates and initialises the main application window.
 * This function does not create any contents; that part is done in the
 * gpredict_gui package.
 *
 */
static void gpredict_app_create(void)
{
    gchar          *title;
    gchar          *icon;
    GError         *icon_error = NULL;

    /* create window title and file name for window icon  */
    title = g_strdup(_("Gpredict"));
    icon = app_logo_file_name();

    /* create window, add title and icon, restore size and position */
    app = gtk_window_new(GTK_WINDOW_TOPLEVEL);

    gtk_window_set_title(GTK_WINDOW(app), title);

    /* restore window position and size if requested by config */
    /* trunk/gtk/gtkblist.c */
    /* size is always restored */
    gtk_window_set_default_size(GTK_WINDOW(app),
                                sat_cfg_get_int(SAT_CFG_INT_WINDOW_WIDTH),
                                sat_cfg_get_int(SAT_CFG_INT_WINDOW_HEIGHT));

    /* position restored only if requested in config */
    if (sat_cfg_get_bool(SAT_CFG_BOOL_MAIN_WIN_POS))
    {
        gtk_window_move(GTK_WINDOW(app),
                        sat_cfg_get_int(SAT_CFG_INT_WINDOW_POS_X),
                        sat_cfg_get_int(SAT_CFG_INT_WINDOW_POS_Y));
    }

    gtk_container_add(GTK_CONTAINER(app), gui_create(app));
    if (g_file_test(icon, G_FILE_TEST_EXISTS))
    {
        if (!gtk_window_set_icon_from_file(GTK_WINDOW(app), icon, &icon_error))
        {
            gpredict_windows_bootstrap_log("startup: failed to load window icon %s: %s",
                                           icon,
                                           icon_error != NULL ? icon_error->message : "unknown error");
            g_clear_error(&icon_error);
        }
    }
    else
    {
        gpredict_windows_bootstrap_log("startup: missing window icon %s", icon);
    }

    g_free(title);
    g_free(icon);

    /* connect delete and destroy signals */
    g_signal_connect(G_OBJECT(app), "delete_event",
                     G_CALLBACK(gpredict_app_delete), NULL);
    g_signal_connect(G_OBJECT(app), "configure_event",
                     G_CALLBACK(gpredict_app_config), NULL);
    g_signal_connect(G_OBJECT(app), "destroy",
                     G_CALLBACK(gpredict_app_destroy), NULL);

    signal(SIGTERM, gpredict_term_sig_handler);
    signal(SIGINT, gpredict_term_sig_handler);
}

static void gpredict_install_signal_handlers(void)
{
    signal(SIGABRT, gpredict_fatal_sig_handler);
#ifdef SIGSEGV
    signal(SIGSEGV, gpredict_fatal_sig_handler);
#endif
#ifdef SIGBUS
    signal(SIGBUS, gpredict_fatal_sig_handler);
#endif
#ifdef SIGILL
    signal(SIGILL, gpredict_fatal_sig_handler);
#endif
#ifdef SIGFPE
    signal(SIGFPE, gpredict_fatal_sig_handler);
#endif
}

/**
 * Handle terminate signals.
 *
 * @param sig The signal that has been received.
 *
 * This function is used to handle termination signals received by the program.
 * The currently caught signals are SIGTERM, SIGINT and SIGABRT. When one of these
 * signals is received, the function sends an error message to logger and tries
 * to make a clean exit.
 */
static void gpredict_term_sig_handler(int sig)
{
    sat_log_forensic(SAT_LOG_LEVEL_WARN, "received signal: %d", sig);
    if (app != NULL)
        gtk_widget_destroy(app);
}

static void gpredict_fatal_sig_handler(int sig)
{
    sat_log_write_fatal_signal(sig, "fatal signal");
    signal(sig, SIG_DFL);
    raise(sig);
}

/**
 * Handle delete events.
 *
 * @param widget The widget which received the delete event signal.
 * @param event  Data structure describing the event.
 * @param data   User data (NULL).
 * @param return Always FALSE to indicate that the app should be destroyed.
 *
 * This function handles the delete event received by the main application
 * window (eg. when the window is closed by the WM). This function simply
 * returns FALSE indicating that the main application window should be
 * destroyed by emitting the destroy signal.
 *
 */
static gint gpredict_app_delete(GtkWidget * widget, GdkEvent * event,
                                gpointer data)
{
    (void)widget;
    (void)data;
    sat_log_forensic(SAT_LOG_LEVEL_INFO,
                     "main window delete_event type=%d",
                     event ? (gint) event->type : -1);
    return FALSE;
}

/**
 * Handle destroy signals.
 *
 * @param widget The widget which received the signal.
 * @param data   User data (NULL).
 *
 * This function is called when the main application window receives the
 * destroy signal, ie. it is destroyed. This function signals all daemons
 * and other threads to stop and exits the Gtk+ main loop.
 */
static void gpredict_app_destroy(GtkWidget * widget, gpointer data)
{
    (void)widget;
    (void)data;

    sat_log_forensic(SAT_LOG_LEVEL_INFO, "main window destroy begin");

    /* stop TLE monitoring task */
    tle_mon_stop();
    sat_log_forensic(SAT_LOG_LEVEL_INFO, "main window destroy tle monitor stopped");

    /* GUI timers are stopped automatically */
    mod_mgr_save_state();
    sat_log_forensic(SAT_LOG_LEVEL_INFO, "main window destroy module state saved");

    /* not good, have to use configure event instead (see API doc) */
    /*     gtk_window_get_size (GTK_WINDOW (app), &w, &h);
       sat_cfg_set_int (SAT_CFG_INT_WINDOW_WIDTH, w);
       sat_cfg_set_int (SAT_CFG_INT_WINDOW_HEIGHT, h);
     */

    gtk_main_quit();
    sat_log_forensic(SAT_LOG_LEVEL_INFO, "main window destroy gtk_main_quit requested");
}

/**
 * Snoop window position and size when main window receives configure event.
 *
 * @param widget Pointer to the gpredict main window.
 * @param event  Pointer to the even structure.
 * @param data   Pointer to user data (always NULL).
 *
 * This function is used to trap configure events in order to store the current
 * position and size of the main window.
 *
 * @note unfortunately GdkEventConfigure ignores the window gravity, while
 *       the only way we have of setting the position doesn't. We have to
 *       call get_position because it does pay attention to the gravity.
 *
 * @note The logic in the code has been borrowed from gaim/pidgin http://pidgin.im/
 *
 */
static gboolean gpredict_app_config(GtkWidget * widget,
                                    GdkEventConfigure * event,
                                    gpointer data)
{
    gint            x, y, w, h;

    (void)data;

    /* data is only useful when window is visible */
    if (gtk_widget_get_visible(widget))
        gtk_window_get_position(GTK_WINDOW(widget), &x, &y);
    else
        return FALSE;

#ifdef G_OS_WIN32
    /* Workaround for GTK+ bug # 169811 - "configure_event" is fired
       when the window is being maximized */
    if (gdk_window_get_state(gtk_widget_get_window(widget)) & GDK_WINDOW_STATE_MAXIMIZED)
    {
        return FALSE;
    }
#endif

    /* don't save off-screen positioning */
    /* gtk_menu_popup got deprecated in 3.22, first available in Ubuntu 18.04 */
#if GTK_MINOR_VERSION < 22
    w = gdk_screen_width();
    h = gdk_screen_height();
#else
    GdkWindow      *window;
    GdkDisplay     *display;
    GdkMonitor     *monitor;
    GdkRectangle    work_area;

    /* https://gitlab.gnome.org/GNOME/gtk/-/issues/1028 */
    window = gtk_widget_get_window(widget);
    display = gtk_widget_get_display(widget);
    monitor = gdk_display_get_monitor_at_window(display, window);
    gdk_monitor_get_workarea(monitor, &work_area);

    w = work_area.width;
    h = work_area.height;
#endif

    if (x < 0 || y < 0 || x + event->width > w || y + event->height > h)
    {
        return FALSE;
    }

    /* store the position and size */
    sat_cfg_set_int(SAT_CFG_INT_WINDOW_POS_X, x);
    sat_cfg_set_int(SAT_CFG_INT_WINDOW_POS_Y, y);
    sat_cfg_set_int(SAT_CFG_INT_WINDOW_WIDTH, event->width);
    sat_cfg_set_int(SAT_CFG_INT_WINDOW_HEIGHT, event->height);

    /* continue to handle event normally */
    return FALSE;
}

/**
 * Monitor TLE age.
 *
 * This function is called periodically in order to check
 * whether it is time to update the TLE elements.
 *
 * If the time to update the TLE has come, it will either notify
 * the user, or fork a separate task which will update the TLE data
 * in the background (depending on user settings).
 *
 * In case of notification, the task will be removed in order to
 * avoid a new notification the next time the taks would be run.
 */
static gboolean tle_mon_task(gpointer data)
{
    /*GtkWidget *selector; */
    glong           last, thrld;
    gint64          now;
    GtkWidget      *dialog;
    GError         *err = NULL;

    if (data != NULL)
    {
        sat_log_log(SAT_LOG_LEVEL_ERROR,
                    _
                    ("%s: Passed a non-null pointer which should never happen.\n"),
                    __func__);
    }

    /* get time of last update */
    last = sat_cfg_get_int(SAT_CFG_INT_TLE_LAST_UPDATE);

    /* get current time */
    now = g_get_real_time() / G_USEC_PER_SEC;

    /* threshold */
    switch (sat_cfg_get_int(SAT_CFG_INT_TLE_AUTO_UPD_FREQ))
    {
    case TLE_AUTO_UPDATE_MONTHLY:
        thrld = 2592000;
        break;

    case TLE_AUTO_UPDATE_WEEKLY:
        thrld = 604800;
        break;

    case TLE_AUTO_UPDATE_DAILY:
        thrld = 86400;
        break;

        /* set default to "infinite" */
    default:
        thrld = G_MAXLONG;
        break;
    }

    if ((now - last) < thrld)
    {
        /* too early */
        /*           sat_log_log (SAT_LOG_LEVEL_DEBUG, */
        /*                           _("%s: Threshold has not been passed yet."), */
        /*                           __func__, last, now, thrld); */
    }
    else
    {
        /* time to update */
        sat_log_log(SAT_LOG_LEVEL_DEBUG,
                    _("%s: Time threshold has been passed."), __func__);

        /* find out what to do */
        if (sat_cfg_get_int(SAT_CFG_INT_TLE_AUTO_UPD_ACTION) ==
            TLE_AUTO_UPDATE_GOAHEAD)
        {

            /* start update process in separate thread */
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        _("%s: Starting new update thread."), __func__);

            /** FIXME: store thread and destroy on exit? **/
            g_thread_try_new(_("gpredict_tle_update"), update_tle_thread, NULL,
                             &err);

            if (err != NULL)
                sat_log_log(SAT_LOG_LEVEL_ERROR,
                            _("%s: Failed to create TLE update thread (%s)"),
                            __func__, err->message);

        }
        else if (!tle_upd_note_sent)
        {
            /* notify user */
            dialog = gtk_message_dialog_new_with_markup(GTK_WINDOW(app),
                                                        GTK_DIALOG_DESTROY_WITH_PARENT,
                                                        GTK_MESSAGE_INFO,
                                                        GTK_BUTTONS_OK,
                                                        _
                                                        ("Your TLE files are getting out of date.\n"
                                                         "You can update them by selecting\n"
                                                         "<b>Edit -> Update TLE</b>\n"
                                                         "in the menubar."));

            /* Destroy the dialog when the user responds to it (e.g. clicks a button) */
            g_signal_connect_swapped(dialog, "response",
                                     G_CALLBACK(gtk_widget_destroy), dialog);

            gtk_widget_show_all(dialog);

            tle_upd_note_sent = TRUE;
        }
    }

    return TRUE;
}

/* Stop TLE monitoring and any pending updates. */
static void tle_mon_stop(void)
{
    gboolean        retcode;

    if (tle_mon_id)
    {
        retcode = g_source_remove(tle_mon_id);

        if (!retcode)
            sat_log_log(SAT_LOG_LEVEL_ERROR,
                        _("%s: Could not find TLE monitoring task (ID = %d)"),
                        __func__, tle_mon_id);

    }

    /* if TLE update is running wait until it is finished */
    while (tle_upd_running)
    {
        g_usleep(1000);
    }
}

/* Thread function which invokes TLE update */
static          gpointer update_tle_thread(gpointer data)
{
    (void)data;

    tle_upd_running = TRUE;
    tle_update_from_network(TRUE, NULL, NULL, NULL);
    mod_mgr_reload_sats();
    tle_upd_running = FALSE;

    return NULL;
}

/*
 * Clean TLE data.
 *
 * This function removes all .sat files from the user's configuration directory.
 * The function is called when gpreidict is executed with the --clean-tle
 * command line option.
 */
static void clean_tle(void)
{
    GDir           *targetdir;
    gchar          *targetdirname, *path;
    const gchar    *filename;

    /* Get trsp directory */
    targetdirname = get_satdata_dir();
    targetdir = g_dir_open(targetdirname, 0, NULL);

    sat_log_log(SAT_LOG_LEVEL_INFO,
                _("%s: Cleaning TLE data in %s"), __func__, targetdirname);

    while ((filename = g_dir_read_name(targetdir)))
    {
        if (g_str_has_suffix(filename, ".sat"))
        {
            /* remove .sat file */
            path = sat_file_name(filename);
            if G_UNLIKELY
                (g_unlink(path))
            {
                sat_log_log(SAT_LOG_LEVEL_ERROR,
                            _("%s: Failed to delete %s"), __func__, filename);
            }
            else
            {
                sat_log_log(SAT_LOG_LEVEL_INFO,
                            _("%s: Removed %s"), __func__, filename);
            }
            g_free(path);
        }
    }
    g_free(targetdirname);
}

/*
 * Clean transponder data.
 *
 * This function removes all .trsp files from the user's configuration directory.
 * The function is called when gpredict is executed with the --clean-trsp
 * command line option.
 */
static void clean_trsp(void)
{
    GDir           *targetdir;
    gchar          *targetdirname, *path;
    const gchar    *filename;

    /* Get trsp directory */
    targetdirname = get_trsp_dir();
    targetdir = g_dir_open(targetdirname, 0, NULL);

    sat_log_log(SAT_LOG_LEVEL_INFO,
                _("%s: Cleaning transponder data in %s"), __func__,
                targetdirname);

    while ((filename = g_dir_read_name(targetdir)))
    {
        if (g_str_has_suffix(filename, ".trsp"))
        {
            /* remove .trsp file */
            path = trsp_file_name(filename);
            if G_UNLIKELY
                (g_unlink(path))
            {
                sat_log_log(SAT_LOG_LEVEL_ERROR,
                            _("%s: Failed to delete %s"), __func__, filename);
            }
            else
            {
                sat_log_log(SAT_LOG_LEVEL_INFO,
                            _("%s: Removed %s"), __func__, filename);
            }
            g_free(path);
        }
    }
    g_free(targetdirname);
}
