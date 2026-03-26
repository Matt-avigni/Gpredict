/*
 * Copyright (C) 2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include "sat-log.h"

void sat_log_init(void)
{
}

void sat_log_close(void)
{
}

void sat_log_log(sat_log_level_t level, const char *fmt, ...)
{
    (void)level;
    (void)fmt;
}

void sat_log_set_visible(gboolean visible)
{
    (void)visible;
}

void sat_log_set_level(sat_log_level_t level)
{
    (void)level;
}
