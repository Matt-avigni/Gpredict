#ifndef ROTCTLD_MGR_H
#define ROTCTLD_MGR_H 1

#include <glib.h>

typedef struct _RotctldMgr RotctldMgr;
typedef void (*RotctldMgrLogFunc)(RotctldMgr *mgr,
                                  const gchar *prefix,
                                  const gchar *line,
                                  gpointer user_data);

gboolean     rotctld_mgr_host_is_local(const gchar *host);
gboolean     rotctld_mgr_wait_for_port(const gchar *host, gint port,
                                       gint timeout_ms);

RotctldMgr  *rotctld_mgr_spawn(const gchar *host, gint port, gint model,
                               const gchar *device, gint baud,
                               gboolean verbose, gchar **error_out);
RotctldMgr  *rotctld_mgr_spawn_argv(gchar **argv, gchar **error_out);
gboolean     rotctld_mgr_is_running(const RotctldMgr *mgr);
const gchar *rotctld_mgr_get_identifier(const RotctldMgr *mgr);
gchar       *rotctld_mgr_get_log_tail(RotctldMgr *mgr);
void         rotctld_mgr_set_log_callback(RotctldMgr *mgr,
                                          RotctldMgrLogFunc cb,
                                          gpointer user_data);
void         rotctld_mgr_terminate(RotctldMgr **mgr_ptr);

#endif
