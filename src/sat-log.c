/*
  Gpredict: Real-time satellite tracking and orbit prediction program

  Copyright (C)  2001-2017  Alexandru Csete, OZ9AEC.

  Comments, questions and bugreports should be submitted via
  http://sourceforge.net/projects/gpredict/
  More details can be found at the project home page:

  http://gpredict.oz9aec.net/

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
/**
 * Gpredict runtime and forensic logger.
 *
 * All runtime diagnostics are written into USER_CONF_DIR/logs as
 * per-session log files. The standard logger honours LOG/LEVEL while
 * the forensic path is always on and is used for startup, GLib
 * handlers, crash markers, and daemon/probe traces.
 */
#ifdef HAVE_CONFIG_H
#include <build-config.h>
#endif

#include <errno.h>
#include <fcntl.h>
#include <signal.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <glib/gi18n.h>
#include <glib/gprintf.h>
#include <glib/gstdio.h>
#include <gtk/gtk.h>

#ifdef G_OS_WIN32
#include <io.h>
#include <process.h>
#define SAT_LOG_WRITE_FD _write
#else
#include <unistd.h>
#define SAT_LOG_WRITE_FD write
#endif

#include "compat.h"
#include "sat-cfg.h"
#include "sat-log.h"

#define SAT_LOG_CRASH_TIMESTAMP "1970/01/01 00:00:00"

static gboolean initialised = FALSE;
static sat_log_level_t loglevel = SAT_LOG_LEVEL_DEBUG;
static gboolean debug_to_stderr = FALSE;
static gint log_fd = -1;
static gchar *log_dir_path = NULL;
static gchar *log_file_path = NULL;
static volatile sig_atomic_t fatal_signal_active = 0;
static GLogFunc previous_log_handler = NULL;
static GPrintFunc previous_print_handler = NULL;
static GPrintFunc previous_printerr_handler = NULL;

G_LOCK_DEFINE_STATIC(sat_log_lock);

/** String representation of debug levels. */
static const gchar *debug_level_str[] = {
    N_(" --- "),
    N_("ERROR"),
    N_(" WARN"),
    N_(" INFO"),
    N_("DEBUG")
};

static gchar *sat_log_build_dir_path(void);
static gboolean sat_log_filename_is_managed(const gchar *name);
static gchar *sat_log_build_session_path(const gchar *dir_path);
static gboolean sat_log_write_raw_locked(const gchar *msg);
static gboolean sat_log_write_raw_unlocked(const gchar *msg);
static void sat_log_write_message(sat_log_level_t level,
                                  const gchar *message,
                                  gboolean honour_filter);
static void sat_log_write_multiline(sat_log_level_t level,
                                    const gchar *prefix,
                                    const gchar *text,
                                    gboolean honour_filter);
static void sat_log_cleanup_dir(glong age_seconds, const gchar *skip_path);
static sat_log_level_t sat_log_glib_level_to_sat(GLogLevelFlags log_level);
static const gchar *sat_log_glib_level_name(GLogLevelFlags log_level);
static void sat_log_glib_handler(const gchar *log_domain,
                                 GLogLevelFlags log_level,
                                 const gchar *message,
                                 gpointer user_data);
static void sat_log_print_handler(const gchar *string);
static void sat_log_printerr_handler(const gchar *string);

static gchar *sat_log_build_dir_path(void)
{
    gchar *confdir = get_user_conf_dir();
    gchar *dir_path = g_build_filename(confdir, "logs", NULL);

    g_free(confdir);
    return dir_path;
}

gchar *sat_log_get_dir_path(void)
{
    if (log_dir_path != NULL)
        return g_strdup(log_dir_path);

    return sat_log_build_dir_path();
}

gchar *sat_log_get_current_path(void)
{
    if (log_file_path == NULL)
        return NULL;

    return g_strdup(log_file_path);
}

static gboolean sat_log_filename_is_managed(const gchar *name)
{
    if (name == NULL || *name == '\0')
        return FALSE;

    if (!g_str_has_suffix(name, ".log"))
        return FALSE;

    if (g_strcmp0(name, "gpredict.log") == 0)
        return TRUE;

    return g_str_has_prefix(name, "gpredict-");
}

gchar *sat_log_get_latest_path(void)
{
    gchar *dir_path = NULL;
    GDir *dir = NULL;
    GError *error = NULL;
    const gchar *name = NULL;
    gchar *best_path = NULL;
    time_t best_mtime = 0;

    dir_path = sat_log_get_dir_path();
    if (dir_path == NULL)
        return NULL;

    dir = g_dir_open(dir_path, 0, &error);
    if (dir == NULL)
    {
        g_clear_error(&error);
        g_free(dir_path);
        return NULL;
    }

    while ((name = g_dir_read_name(dir)) != NULL)
    {
        gchar *path = NULL;
        GStatBuf st;

        if (!sat_log_filename_is_managed(name))
            continue;

        path = g_build_filename(dir_path, name, NULL);
        if (g_stat(path, &st) != 0 || !S_ISREG(st.st_mode))
        {
            g_free(path);
            continue;
        }

        if (best_path == NULL ||
            st.st_mtime > best_mtime ||
            (st.st_mtime == best_mtime &&
             g_strcmp0(path, best_path) > 0))
        {
            g_free(best_path);
            best_path = path;
            best_mtime = st.st_mtime;
        }
        else
        {
            g_free(path);
        }
    }

    g_dir_close(dir);
    g_free(dir_path);
    return best_path;
}

static gint sat_log_get_pid(void)
{
#ifdef G_OS_WIN32
    return (gint)_getpid();
#else
    return (gint)getpid();
#endif
}

static gchar *sat_log_build_session_path(const gchar *dir_path)
{
    GDateTime *now = NULL;
    gchar *stamp = NULL;
    gchar *basename = NULL;
    gchar *path = NULL;

    if (dir_path == NULL)
        return NULL;

    now = g_date_time_new_now_local();
    stamp = g_date_time_format(now, "%Y%m%d-%H%M%S");
    basename = g_strdup_printf("gpredict-%s-p%d.log",
                               stamp ? stamp : "unknown",
                               sat_log_get_pid());
    path = g_build_filename(dir_path, basename, NULL);

    g_date_time_unref(now);
    g_free(stamp);
    g_free(basename);
    return path;
}

static gboolean sat_log_write_raw_unlocked(const gchar *msg)
{
    gsize len = 0;
    const gchar *p = NULL;

    if (msg == NULL)
        return FALSE;

    len = strlen(msg);
    p = msg;

    while (len > 0)
    {
        int written = SAT_LOG_WRITE_FD(log_fd, p, (unsigned int)len);

        if (written < 0)
        {
            if (errno == EINTR)
                continue;
            return FALSE;
        }

        p += written;
        len -= (gsize)written;
    }

    return TRUE;
}

static gboolean sat_log_write_raw_locked(const gchar *msg)
{
    gboolean ok = FALSE;

    if (msg == NULL)
        return FALSE;

    if (initialised && log_fd >= 0)
    {
        G_LOCK(sat_log_lock);
        ok = sat_log_write_raw_unlocked(msg);
        G_UNLOCK(sat_log_lock);

        if (!ok)
            g_fprintf(stderr, "CRITICAL: LOG ERROR\n");
        return ok;
    }

    g_fprintf(stderr, "%s", msg);
    return TRUE;
}

static void sat_log_write_message(sat_log_level_t level,
                                  const gchar *message,
                                  gboolean honour_filter)
{
    GDateTime *now = NULL;
    gchar *msg_time = NULL;
    gchar *msg = NULL;

    if (message == NULL)
        return;

    if (honour_filter && level > loglevel)
        return;

    now = g_date_time_new_now_local();
    msg_time = g_date_time_format(now, "%Y/%m/%d %H:%M:%S");
    msg = g_strdup_printf("%s%s%d%s%s\n",
                          msg_time ? msg_time : SAT_LOG_CRASH_TIMESTAMP,
                          SAT_LOG_MSG_SEPARATOR,
                          level,
                          SAT_LOG_MSG_SEPARATOR,
                          message);

    if G_UNLIKELY(debug_to_stderr)
        g_fprintf(stderr, "%s  %s  %s\n",
                  msg_time ? msg_time : SAT_LOG_CRASH_TIMESTAMP,
                  debug_level_str[level],
                  message);

    sat_log_write_raw_locked(msg);

    g_date_time_unref(now);
    g_free(msg_time);
    g_free(msg);
}

static void sat_log_write_multiline(sat_log_level_t level,
                                    const gchar *prefix,
                                    const gchar *text,
                                    gboolean honour_filter)
{
    gchar *copy = NULL;
    gchar **lines = NULL;

    if (text == NULL)
        return;

    copy = g_strdup(text);
    g_strchomp(copy);
    lines = g_strsplit(copy, "\n", -1);

    for (gint i = 0; lines[i] != NULL; i++)
    {
        gchar *trimmed = g_strdup(lines[i]);
        gchar *full = NULL;

        g_strstrip(trimmed);
        if (*trimmed == '\0')
        {
            g_free(trimmed);
            continue;
        }

        if (prefix && *prefix)
            full = g_strdup_printf("%s: %s", prefix, trimmed);
        else
            full = g_strdup(trimmed);

        sat_log_write_message(level, full, honour_filter);
        g_free(full);
        g_free(trimmed);
    }

    g_strfreev(lines);
    g_free(copy);
}

static void sat_log_cleanup_dir(glong age_seconds, const gchar *skip_path)
{
    gchar *dir_path = NULL;
    GDir *dir = NULL;
    GError *error = NULL;
    const gchar *name = NULL;
    time_t cutoff = 0;

    dir_path = sat_log_get_dir_path();
    if (dir_path == NULL)
        return;

    dir = g_dir_open(dir_path, 0, &error);
    if (dir == NULL)
    {
        g_clear_error(&error);
        g_free(dir_path);
        return;
    }

    if (age_seconds <= 0)
    {
        sat_log_write_message(SAT_LOG_LEVEL_INFO,
                              "sat_log_cleanup_dir: retention disabled; preserving previous session logs",
                              FALSE);
        g_dir_close(dir);
        g_free(dir_path);
        return;
    }

    cutoff = time(NULL) - age_seconds;

    while ((name = g_dir_read_name(dir)) != NULL)
    {
        gchar *path = NULL;
        GStatBuf st;
        gboolean delete_file = FALSE;

        if (!sat_log_filename_is_managed(name))
            continue;

        path = g_build_filename(dir_path, name, NULL);
        if (skip_path != NULL && g_strcmp0(path, skip_path) == 0)
        {
            g_free(path);
            continue;
        }

        if (g_stat(path, &st) != 0 || !S_ISREG(st.st_mode))
        {
            g_free(path);
            continue;
        }

        if (st.st_mtime <= cutoff)
            delete_file = TRUE;

        if (delete_file && g_remove(path) != 0)
        {
            gchar *warn = g_strdup_printf("%s: Failed to delete old log file %s",
                                          __func__, path);
            sat_log_write_message(SAT_LOG_LEVEL_WARN, warn, FALSE);
            g_free(warn);
        }

        g_free(path);
    }

    g_dir_close(dir);
    g_free(dir_path);
}

static sat_log_level_t sat_log_glib_level_to_sat(GLogLevelFlags log_level)
{
    if (log_level & (G_LOG_LEVEL_ERROR | G_LOG_LEVEL_CRITICAL))
        return SAT_LOG_LEVEL_ERROR;
    if (log_level & G_LOG_LEVEL_WARNING)
        return SAT_LOG_LEVEL_WARN;
    if (log_level & G_LOG_LEVEL_DEBUG)
        return SAT_LOG_LEVEL_DEBUG;
    return SAT_LOG_LEVEL_INFO;
}

static const gchar *sat_log_glib_level_name(GLogLevelFlags log_level)
{
    if (log_level & G_LOG_LEVEL_ERROR)
        return "ERROR";
    if (log_level & G_LOG_LEVEL_CRITICAL)
        return "CRITICAL";
    if (log_level & G_LOG_LEVEL_WARNING)
        return "WARNING";
    if (log_level & G_LOG_LEVEL_MESSAGE)
        return "MESSAGE";
    if (log_level & G_LOG_LEVEL_INFO)
        return "INFO";
    if (log_level & G_LOG_LEVEL_DEBUG)
        return "DEBUG";
    return "LOG";
}

static void sat_log_glib_handler(const gchar *log_domain,
                                 GLogLevelFlags log_level,
                                 const gchar *message,
                                 gpointer user_data)
{
    gchar *prefix = NULL;

    (void)user_data;

    prefix = g_strdup_printf("glib[%s/%s]",
                             log_domain ? log_domain : "default",
                             sat_log_glib_level_name(log_level));
    sat_log_write_multiline(sat_log_glib_level_to_sat(log_level),
                            prefix,
                            message ? message : "(null)",
                            FALSE);
    g_free(prefix);
}

static void sat_log_print_handler(const gchar *string)
{
    sat_log_write_multiline(SAT_LOG_LEVEL_INFO, "stdout",
                            string ? string : "(null)", FALSE);
}

static void sat_log_printerr_handler(const gchar *string)
{
    sat_log_write_multiline(SAT_LOG_LEVEL_WARN, "stderr",
                            string ? string : "(null)", FALSE);
}

/**
 * Initialise message logger.
 */
void sat_log_init(void)
{
    if (initialised)
        return;

    log_dir_path = sat_log_build_dir_path();
    if (log_dir_path == NULL)
        return;

    if (!g_file_test(log_dir_path, G_FILE_TEST_IS_DIR) &&
        g_mkdir_with_parents(log_dir_path, 0755) != 0)
    {
        g_fprintf(stderr, "ERROR: Could not create %s\n", log_dir_path);
        g_free(log_dir_path);
        log_dir_path = NULL;
        return;
    }

    log_file_path = sat_log_build_session_path(log_dir_path);
    if (log_file_path == NULL)
        return;

    log_fd = g_open(log_file_path,
                    O_CREAT | O_WRONLY | O_APPEND,
                    0644);
    if (log_fd < 0)
    {
        g_fprintf(stderr, "\n\nERROR: Failed to create %s\n%s\n\n",
                  log_file_path, g_strerror(errno));
        g_free(log_file_path);
        log_file_path = NULL;
        return;
    }

    previous_log_handler = g_log_set_default_handler(sat_log_glib_handler, NULL);
    previous_print_handler = g_set_print_handler(sat_log_print_handler);
    previous_printerr_handler = g_set_printerr_handler(sat_log_printerr_handler);

    initialised = TRUE;
    sat_log_forensic(SAT_LOG_LEVEL_INFO,
                     "%s: Session started file=%s",
                     __func__,
                     log_file_path);
}

/** Close message logger. */
void sat_log_close(void)
{
    gint age = 0;

    if (!initialised)
        return;

    sat_log_forensic(SAT_LOG_LEVEL_INFO, "%s: Session ended", __func__);

    g_log_set_default_handler(previous_log_handler ? previous_log_handler
                                                   : g_log_default_handler,
                              NULL);
    g_set_print_handler(previous_print_handler);
    g_set_printerr_handler(previous_printerr_handler);
    previous_log_handler = NULL;
    previous_print_handler = NULL;
    previous_printerr_handler = NULL;

    G_LOCK(sat_log_lock);
    if (log_fd >= 0)
    {
        g_close(log_fd, NULL);
        log_fd = -1;
    }
    G_UNLOCK(sat_log_lock);

    initialised = FALSE;

    age = sat_cfg_get_int(SAT_CFG_INT_LOG_CLEAN_AGE);
    sat_log_cleanup_dir(age, log_file_path);

    g_free(log_file_path);
    log_file_path = NULL;
    g_free(log_dir_path);
    log_dir_path = NULL;
}

/** Log messages from gpredict honouring the configured level. */
void sat_log_log(sat_log_level_t level, const gchar *fmt, ...)
{
    va_list ap;
    gchar *msg = NULL;

    if (level > loglevel)
        return;

    va_start(ap, fmt);
    msg = g_strdup_vprintf(fmt, ap);
    va_end(ap);

    if (msg == NULL)
        return;

    sat_log_write_multiline(level, NULL, msg, TRUE);
    g_free(msg);
}

/** Unfiltered forensic logging path. */
void sat_log_forensic(sat_log_level_t level, const gchar *fmt, ...)
{
    va_list ap;
    gchar *msg = NULL;

    va_start(ap, fmt);
    msg = g_strdup_vprintf(fmt, ap);
    va_end(ap);

    if (msg == NULL)
        return;

    sat_log_write_multiline(level, NULL, msg, FALSE);
    g_free(msg);
}

void sat_log_set_visible(gboolean visible)
{
    (void)visible;
}

void sat_log_set_level(sat_log_level_t level)
{
    if G_LIKELY(level <= SAT_LOG_LEVEL_DEBUG)
        loglevel = level;
}

static gsize sat_log_signal_u32_to_ascii(unsigned int value, char *buf)
{
    char tmp[16];
    gsize pos = 0;

    if (buf == NULL)
        return 0;

    if (value == 0)
    {
        buf[0] = '0';
        return 1;
    }

    while (value > 0 && pos < sizeof(tmp))
    {
        tmp[pos++] = (char)('0' + (value % 10u));
        value /= 10u;
    }

    for (gsize i = 0; i < pos; i++)
        buf[i] = tmp[pos - 1u - i];

    return pos;
}

void sat_log_write_fatal_signal(int sig, const char *detail)
{
    char line[256];
    const char *prefix = SAT_LOG_CRASH_TIMESTAMP "|1|fatal signal ";
    const char *suffix = " detail=";
    gsize used = 0;
    gsize prefix_len = sizeof(SAT_LOG_CRASH_TIMESTAMP "|1|fatal signal ") - 1;

    if (fatal_signal_active)
        return;

    fatal_signal_active = 1;

    memcpy(line + used, prefix, prefix_len);
    used += prefix_len;
    used += sat_log_signal_u32_to_ascii((unsigned int)((sig < 0) ? -sig : sig),
                                        line + used);

    if (detail != NULL && *detail != '\0')
    {
        gsize suffix_len = sizeof(" detail=") - 1;
        gsize detail_len = 0;

        memcpy(line + used, suffix, suffix_len);
        used += suffix_len;
        while (detail[detail_len] != '\0')
            detail_len++;
        if (detail_len > (sizeof(line) - used - 2))
            detail_len = sizeof(line) - used - 2;
        memcpy(line + used, detail, detail_len);
        used += detail_len;
    }

    line[used++] = '\n';
    line[used] = '\0';

    if (log_fd >= 0)
        (void)SAT_LOG_WRITE_FD(log_fd, line, (unsigned int)used);
    (void)SAT_LOG_WRITE_FD(2, line, (unsigned int)used);
}
