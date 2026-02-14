/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#ifndef WIN32_FETCH_H
#define WIN32_FETCH_H

#include <stdio.h>

int win32_fetch(char *url, FILE *file, char *proxy, char *ua);

#endif
