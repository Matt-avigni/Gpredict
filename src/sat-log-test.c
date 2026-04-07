/*
 * Copyright (C) 2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include <errno.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <time.h>
#include <utime.h>

#include <glib.h>
#include <glib/gstdio.h>

#include "sat-log.h"

extern gint sat_log_test_stub_log_age;
extern gint sat_log_test_stub_log_level;

static gchar *test_root = NULL;
static gint failures = 0;

static void failf(const gchar *fmt, ...)
{
    va_list ap;

    failures++;
    va_start(ap, fmt);
    vfprintf(stderr, fmt, ap);
    va_end(ap);
    fprintf(stderr, "\n");
}

static gboolean path_exists(const gchar *path)
{
    return path != NULL && g_file_test(path, G_FILE_TEST_EXISTS);
}

static gboolean remove_tree(const gchar *path)
{
    GDir *dir = NULL;
    GError *error = NULL;
    const gchar *name = NULL;

    if (path == NULL || !g_file_test(path, G_FILE_TEST_EXISTS))
        return TRUE;

    if (!g_file_test(path, G_FILE_TEST_IS_DIR))
        return g_remove(path) == 0;

    dir = g_dir_open(path, 0, &error);
    if (dir == NULL)
    {
        g_clear_error(&error);
        return FALSE;
    }

    while ((name = g_dir_read_name(dir)) != NULL)
    {
        gchar *child = g_build_filename(path, name, NULL);
        if (!remove_tree(child))
        {
            g_free(child);
            g_dir_close(dir);
            return FALSE;
        }
        g_free(child);
    }

    g_dir_close(dir);
    return g_rmdir(path) == 0;
}

static gchar *build_logs_dir(void)
{
    return g_build_filename(test_root, "Gpredict", "logs", NULL);
}

static void reset_logs_dir(void)
{
    gchar *conf_dir = g_build_filename(test_root, "Gpredict", NULL);

    (void)remove_tree(conf_dir);
    g_free(conf_dir);
}

static gboolean set_file_mtime(const gchar *path, time_t when)
{
    struct utimbuf times;

    if (path == NULL)
        return FALSE;

    times.actime = when;
    times.modtime = when;
    return utime(path, &times) == 0;
}

static gchar *read_file(const gchar *path)
{
    gchar *contents = NULL;
    gsize length = 0;

    if (path == NULL)
        return NULL;

    if (!g_file_get_contents(path, &contents, &length, NULL))
        return NULL;

    return contents;
}

static void test_session_file_creation(void)
{
    gchar *path = NULL;
    gchar *contents = NULL;

    reset_logs_dir();
    sat_log_test_stub_log_age = 3600;
    sat_log_test_stub_log_level = SAT_LOG_LEVEL_DEBUG;

    sat_log_init();
    path = sat_log_get_current_path();
    if (path == NULL)
    {
        failf("session creation: missing current log path");
        return;
    }
    if (!path_exists(path))
        failf("session creation: log file does not exist: %s", path);
    if (!g_str_has_suffix(path, ".log") ||
        g_strrstr(path, "gpredict-") == NULL)
        failf("session creation: unexpected log filename: %s", path);

    sat_log_forensic(SAT_LOG_LEVEL_INFO, "session-creation-marker");
    sat_log_close();

    contents = read_file(path);
    if (contents == NULL)
        failf("session creation: failed to read log file: %s", path);
    else
    {
        if (g_strstr_len(contents, -1, "session-creation-marker") == NULL)
            failf("session creation: marker missing from log");
        if (g_strstr_len(contents, -1, "Session started") == NULL)
            failf("session creation: session start missing from log");
    }

    g_free(contents);
    g_free(path);
}

static void test_latest_path(void)
{
    gchar *dir = NULL;
    gchar *older = NULL;
    gchar *newer = NULL;
    gchar *latest = NULL;
    time_t now = time(NULL);

    reset_logs_dir();
    dir = build_logs_dir();
    if (g_mkdir_with_parents(dir, 0755) != 0)
    {
        failf("latest path: failed to create logs dir %s", dir);
        g_free(dir);
        return;
    }

    older = g_build_filename(dir, "gpredict-20200101-000000-p1.log", NULL);
    newer = g_build_filename(dir, "gpredict-20200101-000001-p2.log", NULL);
    if (!g_file_set_contents(older, "older\n", -1, NULL) ||
        !g_file_set_contents(newer, "newer\n", -1, NULL))
    {
        failf("latest path: failed to create test files");
        goto cleanup;
    }
    if (!set_file_mtime(older, now - 60) || !set_file_mtime(newer, now - 1))
    {
        failf("latest path: failed to set mtimes");
        goto cleanup;
    }

    latest = sat_log_get_latest_path();
    if (g_strcmp0(latest, newer) != 0)
        failf("latest path: expected %s got %s", newer, latest ? latest : "(null)");

cleanup:
    g_free(latest);
    g_free(older);
    g_free(newer);
    g_free(dir);
}

static void test_cleanup_by_age(void)
{
    gchar *dir = NULL;
    gchar *old_path = NULL;
    gchar *fresh_path = NULL;
    gchar *current_path = NULL;
    time_t now = time(NULL);

    reset_logs_dir();
    dir = build_logs_dir();
    if (g_mkdir_with_parents(dir, 0755) != 0)
    {
        failf("cleanup: failed to create logs dir %s", dir);
        g_free(dir);
        return;
    }

    old_path = g_build_filename(dir, "gpredict-20200101-000000-p1.log", NULL);
    fresh_path = g_build_filename(dir, "gpredict-20200101-000001-p2.log", NULL);
    if (!g_file_set_contents(old_path, "old\n", -1, NULL) ||
        !g_file_set_contents(fresh_path, "fresh\n", -1, NULL))
    {
        failf("cleanup: failed to create log files");
        goto cleanup;
    }
    if (!set_file_mtime(old_path, now - 7200) ||
        !set_file_mtime(fresh_path, now - 60))
    {
        failf("cleanup: failed to set mtimes");
        goto cleanup;
    }

    sat_log_test_stub_log_age = 3600;
    sat_log_init();
    current_path = sat_log_get_current_path();
    sat_log_close();

    if (path_exists(old_path))
        failf("cleanup: old log should have been removed");
    if (!path_exists(fresh_path))
        failf("cleanup: fresh log should remain");
    if (!path_exists(current_path))
        failf("cleanup: current session log should remain");

cleanup:
    g_free(current_path);
    g_free(old_path);
    g_free(fresh_path);
    g_free(dir);
}

static void test_cleanup_disabled_keeps_previous_logs(void)
{
    gchar *dir = NULL;
    gchar *old_path = NULL;
    gchar *current_path = NULL;
    time_t now = time(NULL);

    reset_logs_dir();
    dir = build_logs_dir();
    if (g_mkdir_with_parents(dir, 0755) != 0)
    {
        failf("cleanup disabled: failed to create logs dir %s", dir);
        g_free(dir);
        return;
    }

    old_path = g_build_filename(dir, "gpredict-20200101-000000-p1.log", NULL);
    if (!g_file_set_contents(old_path, "old\n", -1, NULL))
    {
        failf("cleanup disabled: failed to create prior log");
        goto cleanup;
    }
    if (!set_file_mtime(old_path, now - 86400))
    {
        failf("cleanup disabled: failed to set mtime");
        goto cleanup;
    }

    sat_log_test_stub_log_age = 0;
    sat_log_init();
    current_path = sat_log_get_current_path();
    sat_log_close();

    if (!path_exists(old_path))
        failf("cleanup disabled: prior log should remain");
    if (!path_exists(current_path))
        failf("cleanup disabled: current session log should remain");

cleanup:
    g_free(current_path);
    g_free(old_path);
    g_free(dir);
}

static void test_glib_redirection(void)
{
    gchar *path = NULL;
    gchar *contents = NULL;

    reset_logs_dir();
    sat_log_test_stub_log_age = 3600;
    sat_log_init();
    path = sat_log_get_current_path();

    g_message("glib-message-marker");
    g_warning("glib-warning-marker");
    g_print("stdout-marker\n");
    g_printerr("stderr-marker\n");
    sat_log_close();

    contents = read_file(path);
    if (contents == NULL)
    {
        failf("glib redirect: failed to read log file");
    }
    else
    {
        if (g_strstr_len(contents, -1, "glib-message-marker") == NULL)
            failf("glib redirect: g_message missing");
        if (g_strstr_len(contents, -1, "glib-warning-marker") == NULL)
            failf("glib redirect: g_warning missing");
        if (g_strstr_len(contents, -1, "stdout-marker") == NULL)
            failf("glib redirect: g_print missing");
        if (g_strstr_len(contents, -1, "stderr-marker") == NULL)
            failf("glib redirect: g_printerr missing");
    }

    g_free(contents);
    g_free(path);
}

int main(void)
{
    GError *error = NULL;

    test_root = g_dir_make_tmp("gpredict-sat-log-test-XXXXXX", &error);
    if (test_root == NULL)
    {
        g_printerr("failed to create temp dir: %s\n",
                   error ? error->message : "unknown error");
        g_clear_error(&error);
        return 1;
    }

    g_setenv("XDG_CONFIG_HOME", test_root, TRUE);

    test_session_file_creation();
    test_latest_path();
    test_cleanup_by_age();
    test_cleanup_disabled_keeps_previous_logs();
    test_glib_redirection();

    if (!remove_tree(test_root))
        g_printerr("failed to remove temp dir %s: %s\n",
                   test_root, g_strerror(errno));
    g_free(test_root);

    if (failures > 0)
    {
        g_printerr("sat-log-test: %d failure(s)\n", failures);
        return 1;
    }

    g_print("sat-log-test: OK\n");
    return 0;
}
