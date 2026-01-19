#ifndef RIGCTLD_CLIENT_H
#define RIGCTLD_CLIENT_H

#include <glib.h>

#include "hamlib_transport.h"
#include "radio-conf.h"

typedef enum {
    RIGCTLD_CLIENT_STOPPED = 0,
    RIGCTLD_CLIENT_CONNECTING,
    RIGCTLD_CLIENT_PROBING,
    RIGCTLD_CLIENT_READY,
    RIGCTLD_CLIENT_DEGRADED
} rigctld_client_state_t;

typedef enum {
    RIG_STRATEGY_PLAIN_FREQ = 0,
    RIG_STRATEGY_SELECT_VFO,
    RIG_STRATEGY_VFO_OPT_ARGS
} rig_strategy_t;

typedef enum {
    RIG_QUIRK_NONE = 0,
    RIG_QUIRK_FORCE_MAIN_SUB = 1 << 0
} rig_quirk_t;

typedef struct RigCaps {
    gchar         *signature;
    gchar         *backend_version;
    gint           rig_model;
    gboolean       has_get_freq;
    gboolean       has_set_freq;
    gboolean       has_get_vfo;
    gboolean       has_set_vfo;
    gboolean       has_set_vfo_opt;
    gboolean       vfo_opt_enabled;
    gboolean       vfo_opt_unsafe;
    gboolean       prefer_main_sub_tokens;
    rig_strategy_t strategy;
    gchar         *default_vfo_token;
    gchar         *vfo_token_main;
    gchar         *vfo_token_sub;
    GPtrArray     *vfo_candidates;
    GHashTable    *vfo_working;
    guint          quirks;
} RigCaps;

typedef struct _RigctldClient RigctldClient;

RigctldClient        *rigctld_client_new(const gchar *label);
void                  rigctld_client_free(RigctldClient **client);
void                  rigctld_client_reset(RigctldClient *client);

gboolean              rigctld_client_connect(RigctldClient *client,
                                             const gchar *host,
                                             gint port,
                                             gint timeout_ms,
                                             gchar **error_out);
gboolean              rigctld_client_attach_fd(RigctldClient *client,
                                               gint fd,
                                               gchar **error_out);
void                  rigctld_client_close(RigctldClient *client);

rigctld_client_state_t rigctld_client_get_state(const RigctldClient *client);
const gchar           *rigctld_client_get_state_reason(const RigctldClient *client);
const RigCaps         *rigctld_client_get_caps(const RigctldClient *client);
HamlibTransport       *rigctld_client_get_transport(RigctldClient *client);

gboolean              rigctld_client_probe(RigctldClient *client,
                                           const radio_conf_t *conf,
                                           gint timeout_ms);

gboolean              rigctld_client_get_freq(RigctldClient *client,
                                              vfo_t vfo,
                                              gdouble *freq_out);
gboolean              rigctld_client_set_freq(RigctldClient *client,
                                              vfo_t vfo,
                                              gdouble freq_hz);
gboolean              rigctld_client_set_vfo(RigctldClient *client,
                                             const gchar *token);
gboolean              rigctld_client_set_vfo_opt(RigctldClient *client,
                                                 gboolean enable);

gboolean              rigctld_client_request_raw(RigctldClient *client,
                                                 const gchar *cmd,
                                                 gchar *out,
                                                 gsize out_len,
                                                 HamlibResponseInfo *info);

gint64                rigctld_client_last_rtt_us(const RigctldClient *client);

#endif
