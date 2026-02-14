/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

/*
    Gpredict: Real-time satellite tracking and orbit prediction program

    Silent, asynchronous TLE autoupdate for module satellites.
*/

#include <glib.h>
#include <glib/gi18n.h>
#include <glib/gstdio.h>
#include <gio/gio.h>
#include <string.h>

#ifdef HAVE_CONFIG_H
#include <build-config.h>
#endif

#ifdef G_OS_WIN32
#include "win32-fetch.h"
#else
#include <curl/curl.h>
#endif

#include "compat.h"
#include "gpredict-utils.h"
#include "sat-cfg.h"
#include "sat-log.h"
#include "sgpsdp/sgp4sdp4.h"
#include "gtk-sat-module.h"
#include "tle-autoupdate.h"

#define TLE_AUTOUPDATE_RUNNING_KEY "tle-autoupdate-running"
#define TLE_AUTOUPDATE_CANCELLABLE_KEY "tle-autoupdate-cancellable"

typedef struct {
    gchar *line1;
    gchar *line2;
    guint catnum;
    gdouble epoch;
    op_stat_t status;
} TleAutoupdateParsed;

typedef struct {
    GWeakRef module_ref;
    gchar *module_name;
    GArray *catnums;
} TleAutoupdateTask;

typedef struct {
    guint total;
    guint fetched;
    guint updated;
    gboolean cancelled;
    gchar *fail_reason;
} TleAutoupdateResult;

#ifndef G_OS_WIN32
static size_t tle_autoupdate_write_func(void *ptr, size_t size, size_t nmemb,
                                        void *stream)
{
    return fwrite(ptr, size, nmemb, (FILE *)stream);
}
#endif

static gchar *tle_autoupdate_build_url(const gchar *server_base, guint catnum)
{
    if (server_base && server_base[0] != '\0')
    {
        if (g_str_has_suffix(server_base, "/"))
            return g_strdup_printf("%sgp.php?CATNR=%u&FORMAT=TLE",
                                   server_base, catnum);
        return g_strdup_printf("%s/gp.php?CATNR=%u&FORMAT=TLE",
                               server_base, catnum);
    }

    return g_strdup_printf(
        "https://celestrak.org/NORAD/elements/gp.php?CATNR=%u&FORMAT=TLE",
        catnum);
}

static gboolean tle_autoupdate_fetch_to_file(const gchar *url,
                                             const gchar *path,
                                             const gchar *proxy,
                                             GCancellable *cancellable,
                                             gchar **error_out)
{
    FILE *outfile;

    if (cancellable && g_cancellable_is_cancelled(cancellable))
    {
        if (error_out)
            *error_out = g_strdup("cancelled");
        return FALSE;
    }

    outfile = g_fopen(path, "wb");
    if (outfile == NULL)
    {
        if (error_out)
            *error_out = g_strdup("open_failed");
        return FALSE;
    }

#ifdef G_OS_WIN32
    {
        int res = win32_fetch((char *)url, outfile, (char *)proxy,
                              (char *)"gpredict/win32");
        fclose(outfile);
        if (res != 0)
        {
            if (error_out)
                *error_out = g_strdup("fetch_failed");
            return FALSE;
        }
    }
#else
    {
        CURL *curl = curl_easy_init();
        CURLcode res;

        if (!curl)
        {
            fclose(outfile);
            if (error_out)
                *error_out = g_strdup("curl_init_failed");
            return FALSE;
        }

        if (proxy && proxy[0] != '\0')
            curl_easy_setopt(curl, CURLOPT_PROXY, proxy);

        curl_easy_setopt(curl, CURLOPT_URL, url);
        curl_easy_setopt(curl, CURLOPT_USERAGENT, "gpredict/curl");
        curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 5L);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 10L);
        curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, outfile);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION,
                         tle_autoupdate_write_func);

        res = curl_easy_perform(curl);
        curl_easy_cleanup(curl);
        fclose(outfile);

        if (res != CURLE_OK)
        {
            if (error_out)
                *error_out = g_strdup(curl_easy_strerror(res));
            return FALSE;
        }
    }
#endif

    return TRUE;
}

static gboolean tle_autoupdate_parse_file(const gchar *path,
                                          TleAutoupdateParsed *parsed,
                                          gchar **error_out)
{
    FILE *fp;
    gchar linebuf[256];
    GPtrArray *lines;
    gchar *line0 = NULL;
    gchar *line1 = NULL;
    gchar *line2 = NULL;
    gchar catstr[6];
    guint catnum;
    char tle_str[3][80];
    tle_t tle;

    fp = g_fopen(path, "r");
    if (!fp)
    {
        if (error_out)
            *error_out = g_strdup("read_failed");
        return FALSE;
    }

    lines = g_ptr_array_new_with_free_func(g_free);
    while (fgets(linebuf, sizeof(linebuf), fp))
    {
        g_strstrip(linebuf);
        if (linebuf[0] == '\0')
            continue;
        g_ptr_array_add(lines, g_strdup(linebuf));
    }
    fclose(fp);

    if (lines->len >= 2)
    {
        gchar *l0 = g_ptr_array_index(lines, 0);
        gchar *l1 = g_ptr_array_index(lines, 1);
        if (l0[0] == '1' && l1[0] == '2')
        {
            line0 = g_strdup("UNKNOWN");
            line1 = g_strdup(l0);
            line2 = g_strdup(l1);
        }
    }

    if (!line1 && lines->len >= 3)
    {
        gchar *l0 = g_ptr_array_index(lines, 0);
        gchar *l1 = g_ptr_array_index(lines, 1);
        gchar *l2 = g_ptr_array_index(lines, 2);
        if (l1[0] == '1' && l2[0] == '2')
        {
            line0 = g_strdup(l0);
            line1 = g_strdup(l1);
            line2 = g_strdup(l2);
        }
    }

    g_ptr_array_free(lines, TRUE);

    if (!line1 || !line2)
    {
        g_free(line0);
        g_free(line1);
        g_free(line2);
        if (error_out)
            *error_out = g_strdup("invalid_tle");
        return FALSE;
    }

    if (!Checksum_Good(line1) || !Checksum_Good(line2))
    {
        g_free(line0);
        g_free(line1);
        g_free(line2);
        if (error_out)
            *error_out = g_strdup("checksum_failed");
        return FALSE;
    }

    memset(tle_str, 0, sizeof(tle_str));
    g_strlcpy(tle_str[0], line0, sizeof(tle_str[0]));
    g_strlcpy(tle_str[1], line1, sizeof(tle_str[1]));
    g_strlcpy(tle_str[2], line2, sizeof(tle_str[2]));

    if (Get_Next_Tle_Set(tle_str, &tle) != 1)
    {
        g_free(line0);
        g_free(line1);
        g_free(line2);
        if (error_out)
            *error_out = g_strdup("parse_failed");
        return FALSE;
    }

    catstr[0] = line1[2];
    catstr[1] = line1[3];
    catstr[2] = line1[4];
    catstr[3] = line1[5];
    catstr[4] = line1[6];
    catstr[5] = '\0';
    catnum = (guint) g_ascii_strtod(catstr, NULL);

    parsed->line1 = line1;
    parsed->line2 = line2;
    parsed->catnum = catnum;
    parsed->epoch = tle.epoch;
    parsed->status = tle.status;

    g_free(line0);
    return TRUE;
}

static gboolean tle_autoupdate_get_existing_epoch(GKeyFile *satdata,
                                                  gdouble *epoch_out,
                                                  op_stat_t *status_out)
{
    gchar *tlestr1 = NULL;
    gchar *tlestr2 = NULL;
    gchar *rawtle = NULL;
    tle_t tle;
    gdouble epoch = 0.0;
    op_stat_t status = OP_STAT_UNKNOWN;

    if (g_key_file_has_key(satdata, "Satellite", "STATUS", NULL))
        status = g_key_file_get_integer(satdata, "Satellite", "STATUS", NULL);

    tlestr1 = g_key_file_get_string(satdata, "Satellite", "TLE1", NULL);
    tlestr2 = g_key_file_get_string(satdata, "Satellite", "TLE2", NULL);

    if (tlestr1 && tlestr2)
    {
        rawtle = g_strconcat(tlestr1, tlestr2, NULL);
        if (Good_Elements(rawtle))
        {
            Convert_Satellite_Data(rawtle, &tle);
            epoch = tle.epoch;
        }
        g_free(rawtle);
    }

    g_free(tlestr1);
    g_free(tlestr2);

    *epoch_out = epoch;
    *status_out = status;
    return TRUE;
}

static gboolean tle_autoupdate_update_sat(const TleAutoupdateParsed *parsed,
                                          gboolean *updated_out,
                                          gchar **error_out)
{
    GKeyFile *satdata;
    gchar *path;
    GError *error = NULL;
    gdouble old_epoch = 0.0;
    op_stat_t old_status = OP_STAT_UNKNOWN;
    gboolean update = FALSE;

    if (updated_out)
        *updated_out = FALSE;

    path = sat_file_name_from_catnum(parsed->catnum);
    if (!path)
    {
        if (error_out)
            *error_out = g_strdup("path_failed");
        return FALSE;
    }

    satdata = g_key_file_new();
    if (!g_key_file_load_from_file(satdata, path, G_KEY_FILE_KEEP_COMMENTS,
                                   &error))
    {
        g_clear_error(&error);
        g_key_file_free(satdata);
        g_free(path);
        if (error_out)
            *error_out = g_strdup("load_failed");
        return FALSE;
    }

    tle_autoupdate_get_existing_epoch(satdata, &old_epoch, &old_status);

    if (parsed->epoch > old_epoch)
        update = TRUE;
    else if (parsed->epoch == old_epoch &&
             parsed->status != OP_STAT_UNKNOWN &&
             parsed->status != old_status)
        update = TRUE;

    if (update)
    {
        g_key_file_set_string(satdata, "Satellite", "TLE1", parsed->line1);
        g_key_file_set_string(satdata, "Satellite", "TLE2", parsed->line2);
        if (parsed->status != OP_STAT_UNKNOWN)
            g_key_file_set_integer(satdata, "Satellite", "STATUS",
                                   parsed->status);

        if (gpredict_save_key_file(satdata, path))
        {
            g_key_file_free(satdata);
            g_free(path);
            if (error_out)
                *error_out = g_strdup("save_failed");
            return FALSE;
        }
    }

    g_key_file_free(satdata);
    g_free(path);
    if (updated_out && update)
        *updated_out = TRUE;
    return TRUE;
}

static void tle_autoupdate_task_free(TleAutoupdateTask *task)
{
    if (!task)
        return;

    g_weak_ref_clear(&task->module_ref);
    g_free(task->module_name);
    if (task->catnums)
        g_array_free(task->catnums, TRUE);
    g_free(task);
}

static void tle_autoupdate_result_free(TleAutoupdateResult *result)
{
    if (!result)
        return;

    g_free(result->fail_reason);
    g_free(result);
}

static void tle_autoupdate_thread(GTask *task, gpointer source_object,
                                  gpointer task_data,
                                  GCancellable *cancellable)
{
    TleAutoupdateTask *data = task_data;
    TleAutoupdateResult *result = g_new0(TleAutoupdateResult, 1);
    gchar *cache_dir;
    gchar *proxy;
    gchar *server;
    guint i;

    (void)source_object;

    proxy = sat_cfg_get_str(SAT_CFG_STR_TLE_PROXY);
    server = sat_cfg_get_str(SAT_CFG_STR_TLE_SERVER);

    cache_dir = sat_file_name("cache");
    if (cache_dir)
        g_mkdir_with_parents(cache_dir, 0700);

    for (i = 0; i < data->catnums->len; i++)
    {
        guint catnum = g_array_index(data->catnums, guint, i);
        gchar *url;
        gchar *path;
        gchar *err = NULL;
        TleAutoupdateParsed parsed = { 0 };
        gboolean updated = FALSE;

        result->total++;

        if (cancellable && g_cancellable_is_cancelled(cancellable))
        {
            result->cancelled = TRUE;
            break;
        }

        url = tle_autoupdate_build_url(server, catnum);
        path = g_strdup_printf("%s%stle-autoupdate-%u.tle",
                               cache_dir ? cache_dir : ".",
                               G_DIR_SEPARATOR_S, catnum);

        if (!tle_autoupdate_fetch_to_file(url, path, proxy, cancellable, &err))
        {
            if (!result->fail_reason && err)
                result->fail_reason = g_strdup(err);
            g_free(err);
            g_free(url);
            g_free(path);
            continue;
        }

        result->fetched++;

        if (!tle_autoupdate_parse_file(path, &parsed, &err))
        {
            if (!result->fail_reason && err)
                result->fail_reason = g_strdup(err);
            g_free(err);
            g_free(url);
            g_free(path);
            continue;
        }

        if (parsed.catnum != catnum)
        {
            if (!result->fail_reason)
                result->fail_reason = g_strdup("catnum_mismatch");
            g_free(parsed.line1);
            g_free(parsed.line2);
            g_free(url);
            g_free(path);
            continue;
        }

        if (!tle_autoupdate_update_sat(&parsed, &updated, &err))
        {
            if (!result->fail_reason && err)
                result->fail_reason = g_strdup(err);
            g_free(err);
        }
        else if (updated)
        {
            result->updated++;
        }

        g_free(parsed.line1);
        g_free(parsed.line2);
        g_remove(path);
        g_free(url);
        g_free(path);
    }

    if (cache_dir)
        g_free(cache_dir);
    g_free(proxy);
    g_free(server);

    g_task_return_pointer(task, result,
                          (GDestroyNotify) tle_autoupdate_result_free);
}

static void tle_autoupdate_done(GObject *source_object, GAsyncResult *res,
                                gpointer user_data)
{
    GTask *task = G_TASK(res);
    TleAutoupdateTask *data = g_task_get_task_data(task);
    TleAutoupdateResult *result = NULL;
    GtkSatModule *module = NULL;

    (void)source_object;
    (void)user_data;

    result = g_task_propagate_pointer(task, NULL);
    module = g_weak_ref_get(&data->module_ref);

    if (module)
    {
        g_object_set_data(G_OBJECT(module), TLE_AUTOUPDATE_RUNNING_KEY, NULL);
        g_object_set_data(G_OBJECT(module), TLE_AUTOUPDATE_CANCELLABLE_KEY,
                          NULL);
    }

    if (result && !result->cancelled)
    {
        if (result->fetched > 0)
        {
            if (module)
                gtk_sat_module_reload_sats_silent(module);
            if (result->updated > 0)
            {
                gint64 now = g_get_real_time() / G_USEC_PER_SEC;
                sat_cfg_set_int(SAT_CFG_INT_TLE_LAST_UPDATE, now);
            }
            sat_log_log(SAT_LOG_LEVEL_INFO,
                        _("tle_autoupdate: refreshed %u satellites for module '%s'"),
                        result->updated,
                        data->module_name ? data->module_name : "?");
        }
        else
        {
            sat_log_log(SAT_LOG_LEVEL_DEBUG,
                        _("tle_autoupdate: failed (%s), keeping existing TLEs"),
                        result->fail_reason ? result->fail_reason : "unknown");
        }
    }

    if (module)
        g_object_unref(module);
    if (result)
        tle_autoupdate_result_free(result);
}

void tle_autoupdate_start(GtkSatModule *module)
{
    GHashTableIter iter;
    gpointer key;
    TleAutoupdateTask *data;
    GTask *task;
    GCancellable *cancellable;
    GHashTable *seen;
    guint catnum;

    if (!module || !module->satellites)
        return;

    if (g_object_get_data(G_OBJECT(module), TLE_AUTOUPDATE_RUNNING_KEY))
        return;

    seen = g_hash_table_new(g_direct_hash, g_direct_equal);

    data = g_new0(TleAutoupdateTask, 1);
    g_weak_ref_init(&data->module_ref, G_OBJECT(module));
    data->module_name = g_strdup(module->name);
    data->catnums = g_array_new(FALSE, FALSE, sizeof(guint));

    g_hash_table_iter_init(&iter, module->satellites);
    while (g_hash_table_iter_next(&iter, &key, NULL))
    {
        catnum = *(guint *) key;
        if (catnum == 0)
            continue;
        if (g_hash_table_lookup(seen, GINT_TO_POINTER(catnum)))
            continue;
        g_hash_table_insert(seen, GINT_TO_POINTER(catnum), GINT_TO_POINTER(1));
        g_array_append_val(data->catnums, catnum);
    }

    g_hash_table_destroy(seen);

    if (data->catnums->len == 0)
    {
        tle_autoupdate_task_free(data);
        return;
    }

    g_object_set_data(G_OBJECT(module), TLE_AUTOUPDATE_RUNNING_KEY,
                      GINT_TO_POINTER(1));

    cancellable = g_cancellable_new();
    g_object_set_data_full(G_OBJECT(module), TLE_AUTOUPDATE_CANCELLABLE_KEY,
                           cancellable, g_object_unref);

    task = g_task_new(module, cancellable, tle_autoupdate_done, NULL);
    g_task_set_task_data(task, data, (GDestroyNotify) tle_autoupdate_task_free);
    g_task_run_in_thread(task, tle_autoupdate_thread);
    g_object_unref(task);
}
