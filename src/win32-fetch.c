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
#include "win32-fetch.h"

#if defined(G_OS_WIN32) || defined(_WIN32)
#include <curl/curl.h>

int win32_fetch(char *url, FILE *file, char *proxy, char *ua)
{
    CURL           *curl;
    CURLcode        res;

    if (url == NULL || file == NULL)
        return (int)CURLE_FAILED_INIT;

    curl = curl_easy_init();
    if (curl == NULL)
        return (int)CURLE_FAILED_INIT;

    curl_easy_setopt(curl, CURLOPT_URL, url);
    if (proxy != NULL && proxy[0] != '\0')
        curl_easy_setopt(curl, CURLOPT_PROXY, proxy);
    if (ua != NULL && ua[0] != '\0')
        curl_easy_setopt(curl, CURLOPT_USERAGENT, ua);
    curl_easy_setopt(curl, CURLOPT_CONNECTTIMEOUT, 10L);
    curl_easy_setopt(curl, CURLOPT_FOLLOWLOCATION, 1L);
    curl_easy_setopt(curl, CURLOPT_WRITEDATA, file);

    res = curl_easy_perform(curl);
    curl_easy_cleanup(curl);

    return (int)res;
}
#endif
