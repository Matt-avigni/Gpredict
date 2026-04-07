/*
 * Copyright (C) 2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include "sat-cfg.h"
#include "sat-log.h"

gint sat_log_test_stub_log_age = 0;
gint sat_log_test_stub_log_level = SAT_LOG_LEVEL_DEBUG;

gint sat_cfg_get_int(sat_cfg_int_e param)
{
    switch (param)
    {
    case SAT_CFG_INT_LOG_CLEAN_AGE:
        return sat_log_test_stub_log_age;
    case SAT_CFG_INT_LOG_LEVEL:
        return sat_log_test_stub_log_level;
    default:
        return 0;
    }
}
