#ifndef ROTCTLD_CLIENT_H
#define ROTCTLD_CLIENT_H

#include <glib.h>

#include "hamlib_transport.h"

typedef enum {
    ROTCTLD_CLIENT_STOPPED = 0,
    ROTCTLD_CLIENT_CONNECTING,
    ROTCTLD_CLIENT_PROBING,
    ROTCTLD_CLIENT_READY,
    ROTCTLD_CLIENT_DEGRADED
} rotctld_client_state_t;

typedef struct RotCaps {
    gchar     *signature;
    gint       model_id;
    gboolean   has_get_pos;
    gboolean   has_set_pos;
    gboolean   has_stop;
    gboolean   has_park;
    gboolean   limits_valid;
    gdouble    az_min;
    gdouble    az_max;
    gdouble    el_min;
    gdouble    el_max;
    guint      quirks;
} RotCaps;

typedef struct _RotctldClient RotctldClient;

RotctldClient        *rotctld_client_new(const gchar *label);
void                  rotctld_client_free(RotctldClient **client);
void                  rotctld_client_reset(RotctldClient *client);

gboolean              rotctld_client_connect(RotctldClient *client,
                                             const gchar *host,
                                             gint port,
                                             gint timeout_ms,
                                             gchar **error_out);
void                  rotctld_client_close(RotctldClient *client);

rotctld_client_state_t rotctld_client_get_state(const RotctldClient *client);
const gchar           *rotctld_client_get_state_reason(const RotctldClient *client);
const RotCaps         *rotctld_client_get_caps(const RotctldClient *client);
HamlibTransport       *rotctld_client_get_transport(RotctldClient *client);

gboolean              rotctld_client_probe(RotctldClient *client,
                                           gint timeout_ms);
gboolean              rotctld_client_handshake(RotctldClient *client,
                                               gint timeout_ms,
                                               gdouble *az_out,
                                               gdouble *el_out,
                                               gchar *dump_state_out,
                                               gsize dump_state_len,
                                               gchar *pos_reply_out,
                                               gsize pos_reply_len);
gssize                rotctld_client_clear_rxbuf(RotctldClient *client);

gboolean              rotctld_client_get_pos(RotctldClient *client,
                                             gdouble *az_out,
                                             gdouble *el_out);
gboolean              rotctld_client_set_pos(RotctldClient *client,
                                             gdouble az,
                                             gdouble el);
gboolean              rotctld_client_stop(RotctldClient *client);

gboolean              rotctld_client_request_raw(RotctldClient *client,
                                                 const gchar *cmd,
                                                 gchar *out,
                                                 gsize out_len,
                                                 HamlibResponseInfo *info);

gint64                rotctld_client_last_rtt_us(const RotctldClient *client);

#endif
