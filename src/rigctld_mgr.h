/*
 * Copyright (C) 2024-2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#ifndef RIGCTLD_MGR_H
#define RIGCTLD_MGR_H 1

#include <glib.h>

#include "radio-conf.h"

typedef struct _RigctldMgr RigctldMgr;
typedef void (*RigctldMgrLogFunc)(RigctldMgr *mgr,
                                  const gchar *prefix,
                                  const gchar *line,
                                  gpointer user_data);

gchar       *rigctld_mgr_normalize_host(const gchar *host);
gboolean     rigctld_mgr_host_is_local(const gchar *host);
gboolean     rigctld_mgr_port_is_open(const gchar *host, gint port,
                                      gint timeout_ms);
gboolean     rigctld_mgr_wait_for_port(const gchar *host, gint port,
                                       gint timeout_ms);

RigctldMgr  *rigctld_mgr_spawn(const radio_conf_t *conf,
                               const gchar *bind_host,
                               gchar **error_out,
                               gchar **cmdline_out,
                               const gchar *log_path);
gboolean     rigctld_mgr_is_running(const RigctldMgr *mgr);
gboolean     rigctld_mgr_get_exit_info(RigctldMgr *mgr,
                                       gboolean *exited,
                                       gint *exit_status,
                                       gint *exit_signal);
const gchar *rigctld_mgr_get_identifier(const RigctldMgr *mgr);
gchar       *rigctld_mgr_get_log_tail(RigctldMgr *mgr);
void         rigctld_mgr_set_log_callback(RigctldMgr *mgr,
                                          RigctldMgrLogFunc cb,
                                          gpointer user_data);
void         rigctld_mgr_terminate(RigctldMgr **mgr_ptr);

#endif
