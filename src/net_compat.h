/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#ifndef GPREDICT_NET_COMPAT_H
#define GPREDICT_NET_COMPAT_H

#if defined(_WIN32) || defined(WIN32)
#ifndef _WIN32_WINNT
#define _WIN32_WINNT 0x0600
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

#ifndef CLOSESOCK
#define CLOSESOCK(fd) closesocket(fd)
#endif

#ifndef SOCKERRNO
#define SOCKERRNO WSAGetLastError()
#endif

static int net_inited = 0;

static inline int net_init(void)
{
    if (!net_inited)
    {
        WSADATA wsa;
        int err = WSAStartup(MAKEWORD(2, 2), &wsa);
        if (err != 0)
            return err;
        net_inited = 1;
    }
    return 0;
}

static inline void net_cleanup(void)
{
    if (net_inited)
    {
        WSACleanup();
        net_inited = 0;
    }
}

#else

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <errno.h>

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

#ifndef CLOSESOCK
#define CLOSESOCK(fd) close(fd)
#endif

#ifndef SOCKERRNO
#define SOCKERRNO errno
#endif

static inline int net_init(void)
{
    return 0;
}

static inline void net_cleanup(void)
{
}

#endif

#endif
