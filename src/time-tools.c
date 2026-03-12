/* -*- Mode: C; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*- */
/*
    Gpredict: Real-time satellite tracking and orbit prediction program

    Copyright (C)  2001-2009  Alexandru Csete, OZ9AEC.

    Authors: Alexandru Csete <oz9aec@gmail.com>

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

#include <glib.h>
#include <glib/gi18n.h>
#include <math.h>
//#include <sys/time.h>
#ifdef HAVE_CONFIG_H
#  include <build-config.h>
#endif
#include "sgpsdp/sgp4sdp4.h"
#include "time-tools.h"
#include "sat-cfg.h"
//#ifdef G_OS_WIN32
//#  include "libc_internal.h"
//#  include "libc_interface.h"
//#endif

#define TIME_TOOLS_MIN_VALID_UNIX_SEC G_GINT64_CONSTANT(-62135596800)
#define TIME_TOOLS_MAX_VALID_UNIX_SEC G_GINT64_CONSTANT(253402300799)

static gboolean daynum_to_time_t_checked(gdouble jultime, time_t *tim_out)
{
    long double unix_seconds_ld = 0.0L;
    gint64 unix_seconds = 0;

    if (tim_out == NULL || !isfinite(jultime))
        return FALSE;

    unix_seconds_ld = ((long double)jultime - 2440587.5L) * 86400.0L;
    if (!isfinite((gdouble)unix_seconds_ld))
        return FALSE;

    unix_seconds = (gint64)floorl(unix_seconds_ld);
    if (unix_seconds < TIME_TOOLS_MIN_VALID_UNIX_SEC ||
        unix_seconds > TIME_TOOLS_MAX_VALID_UNIX_SEC)
    {
        return FALSE;
    }

    if ((gint64)(time_t)unix_seconds != unix_seconds)
        return FALSE;

    *tim_out = (time_t)unix_seconds;
    return TRUE;
}

static gboolean daynum_to_tm_checked(time_t tim,
                                     gboolean use_local_time,
                                     struct tm *tm_out)
{
    if (tm_out == NULL)
        return FALSE;

#ifdef G_OS_WIN32
    if (use_local_time)
    {
        if (localtime_s(tm_out, &tim) != 0)
            return FALSE;
    }
    else
    {
        if (gmtime_s(tm_out, &tim) != 0)
            return FALSE;
    }
#else
    if (use_local_time)
    {
        if (localtime_r(&tim, tm_out) == NULL)
            return FALSE;
    }
    else
    {
        if (gmtime_r(&tim, tm_out) == NULL)
            return FALSE;
    }
#endif

    return TRUE;
}

static int daynum_to_str_internal(char *s,
                                  size_t max,
                                  const char *format,
                                  gdouble jultime,
                                  gboolean use_local_time)
{
    time_t tim = (time_t)0;
    struct tm tm_value = { 0 };
    size_t size = 0;

    if (s == NULL || max == 0 || format == NULL)
        return 0;

    s[0] = '\0';

    if (!daynum_to_time_t_checked(jultime, &tim))
        return 0;

    if (!daynum_to_tm_checked(tim, use_local_time, &tm_value))
        return 0;

    size = strftime(s, max, format, &tm_value);
    if (size < max)
        s[size] = '\0';
    else
        s[max - 1] = '\0';

    return (int)size;
}



/** \brief Get the current time.
 *
 * Read the system clock and return the current Julian day.
 */
gdouble
get_current_daynum(void)
{
    struct tm utc;
    GDateTime *now;
    double daynum;

    UTC_Calendar_Now (&utc);
    now = g_date_time_new_now_local();
    daynum = Julian_Date (&utc);
    daynum = daynum + (double)g_date_time_get_microsecond(now)/8.64e+10;
    g_date_time_unref(now);

    return daynum;
}

int
daynum_to_str(char *s, size_t max, const char *format, gdouble jultime){
    return daynum_to_str_internal(s, max, format, jultime,
                                  sat_cfg_get_bool(SAT_CFG_BOOL_USE_LOCAL_TIME));
}

int
daynum_to_utc_str(char *s, size_t max, const char *format, gdouble jultime)
{
    return daynum_to_str_internal(s, max, format, jultime, FALSE);
}

/* This function calculates the day number from m/d/y. */
/* Legacy code no longer in use
long
get_daynum_from_dmy (int d, int m, int y)
{

    long dn;
    double mm, yy;

    if (m<3)
    { 
        y--; 
        m+=12; 
    }

    if (y<57)
        y+=100;

    yy=(double)y;
    mm=(double)m;
    dn=(long)(floor(365.25*(yy-80.0))-floor(19.0+yy/100.0)+floor(4.75+yy/400.0)-16.0);
    dn+=d+30*m+(long)floor(0.6*mm-0.3);

    return dn;
}
*/
