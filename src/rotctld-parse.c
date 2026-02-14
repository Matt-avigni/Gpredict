/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include "rotctld-parse.h"

static gboolean rotctld_line_is_numeric(const gchar *line)
{
    const gchar *p = line;

    if (p == NULL || *p == '\0')
        return FALSE;

    for (; *p != '\0'; p++)
    {
        if (!g_ascii_isdigit(*p))
            return FALSE;
    }

    return TRUE;
}

static gboolean rotctld_parse_int_line(const gchar *line, gint *out)
{
    gchar *endptr = NULL;
    glong value;

    if (line == NULL || *line == '\0' || out == NULL)
        return FALSE;

    value = g_ascii_strtoll(line, &endptr, 10);
    if (endptr == line)
        return FALSE;

    *out = (gint) value;
    return TRUE;
}

gboolean parse_dump_state_model_id(const gchar *reply, gint *model_out)
{
    gchar *line1 = NULL;
    gchar *line2 = NULL;
    gboolean ok = FALSE;
    gint first_val = 0;
    gint second_val = 0;

    if (model_out)
        *model_out = 0;

    if (reply == NULL || *reply == '\0')
        return FALSE;

    {
        gchar **lines = g_strsplit(reply, "\n", -1);
        for (gint i = 0; lines[i] != NULL; i++)
        {
            gchar *line = g_strstrip(lines[i]);

            if (line[0] == '\0')
                continue;

            if (line1 == NULL)
            {
                line1 = g_strdup(line);
                continue;
            }
            if (line2 == NULL)
            {
                line2 = g_strdup(line);
                break;
            }
        }
        g_strfreev(lines);
    }

    if (line1 == NULL)
        goto out;

    if (!rotctld_parse_int_line(line1, &first_val))
        goto out;

    if (first_val == 1)
    {
        if (line2 == NULL || !rotctld_parse_int_line(line2, &second_val))
            goto out;
        if (model_out)
            *model_out = second_val;
        ok = TRUE;
        goto out;
    }

    if (model_out)
        *model_out = first_val;
    ok = TRUE;

out:
    g_free(line1);
    g_free(line2);
    return ok;
}

static gboolean rotctld_parse_first_number(const gchar *line, gdouble *out)
{
    const gchar *p = line;
    gchar *endptr = NULL;
    gdouble value = 0.0;

    if (line == NULL || out == NULL)
        return FALSE;

    while (*p != '\0' && !g_ascii_isdigit(*p) && *p != '-' && *p != '+')
        p++;

    if (*p == '\0')
        return FALSE;

    value = g_ascii_strtod(p, &endptr);
    if (endptr == p)
        return FALSE;

    *out = value;
    return TRUE;
}

gboolean rotctld_parse_model(const gchar *reply, gint *model_out)
{
    gboolean ok = FALSE;
    gint numeric_first = 0;
    gchar *first_line = NULL;
    gchar *second_line = NULL;

    if (model_out)
        *model_out = 0;

    if (reply == NULL || *reply == '\0')
        return FALSE;

    if (parse_dump_state_model_id(reply, model_out))
        return TRUE;

    gchar **lines = g_strsplit(reply, "\n", -1);
    for (gint i = 0; lines[i] != NULL; i++)
    {
        gchar *line = g_strstrip(lines[i]);
        gchar *lower = NULL;
        gdouble value = 0.0;

        if (line[0] == '\0')
            continue;

        if (first_line == NULL)
        {
            first_line = g_strdup(line);
        }
        else if (second_line == NULL)
        {
            second_line = g_strdup(line);
        }

        lower = g_ascii_strdown(line, -1);
        if (lower != NULL)
        {
            if (g_str_has_prefix(lower, "rot_model") ||
                g_str_has_prefix(lower, "rotator model"))
            {
                if (rotctld_parse_first_number(line, &value))
                {
                    if (model_out)
                        *model_out = (gint) value;
                    ok = TRUE;
                }
                g_free(lower);
                break;
            }
        }
        g_free(lower);

        if (rotctld_line_is_numeric(line) &&
            rotctld_parse_first_number(line, &value))
        {
            if (numeric_first == 0)
                numeric_first = (gint) value;
        }
    }
    g_strfreev(lines);

    if (ok)
    {
        g_free(first_line);
        g_free(second_line);
        return TRUE;
    }

    if (first_line != NULL)
    {
        gdouble first_value = 0.0;
        gdouble second_value = 0.0;

        if (rotctld_line_is_numeric(first_line) &&
            rotctld_parse_first_number(first_line, &first_value))
        {
            if (second_line != NULL &&
                rotctld_line_is_numeric(second_line) &&
                rotctld_parse_first_number(second_line, &second_value) &&
                (gint) first_value == 1)
            {
                if (model_out)
                    *model_out = (gint) second_value;
                g_free(first_line);
                g_free(second_line);
                return TRUE;
            }

            if ((gint) first_value != 1)
            {
                if (model_out)
                    *model_out = (gint) first_value;
                g_free(first_line);
                g_free(second_line);
                return TRUE;
            }
        }
    }

    if (numeric_first != 0 && numeric_first != 1)
    {
        if (model_out)
            *model_out = numeric_first;
        g_free(first_line);
        g_free(second_line);
        return TRUE;
    }

    g_free(first_line);
    g_free(second_line);
    return FALSE;
}
