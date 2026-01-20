#ifndef HAMLIB_TRANSPORT_H
#define HAMLIB_TRANSPORT_H

#include <glib.h>

typedef struct _HamlibTransport HamlibTransport;

typedef enum {
    HAMLIB_READ_SINGLE = 0,
    HAMLIB_READ_MULTILINE_RPRT,
    HAMLIB_READ_MULTILINE_IDLE
} hamlib_read_mode_t;

typedef enum {
    HAMLIB_TERM_RPRT = 0,
    HAMLIB_TERM_RPRT_OR_DONE
} hamlib_term_t;

typedef struct {
    gboolean saw_rprt;
    gint     rprt_code;
    gboolean saw_done;
    gboolean used_multiline;
    gint     err;
    gint64   rtt_us;
} HamlibResponseInfo;

HamlibTransport *hamlib_transport_new(void);
void             hamlib_transport_free(HamlibTransport **transport);

gboolean         hamlib_transport_connect(HamlibTransport *transport,
                                          const gchar *host,
                                          gint port,
                                          gint timeout_ms,
                                          gchar **error_out);
gboolean         hamlib_transport_attach_fd(HamlibTransport *transport,
                                            gint fd,
                                            gchar **error_out);
void             hamlib_transport_close(HamlibTransport *transport);
gboolean         hamlib_transport_is_ready(const HamlibTransport *transport);

gboolean         hamlib_transport_request(HamlibTransport *transport,
                                          const gchar *cmd,
                                          hamlib_read_mode_t mode,
                                          hamlib_term_t term,
                                          gint base_timeout_ms,
                                          gint idle_timeout_ms,
                                          gint retries,
                                          gint retry_delay_ms,
                                          gchar *out,
                                          gsize out_len,
                                          HamlibResponseInfo *info);

gssize           hamlib_transport_drain(HamlibTransport *transport,
                                        gint idle_timeout_ms,
                                        gint *err_out);
gssize           hamlib_transport_clear_rxbuf(HamlibTransport *transport);

gint64           hamlib_transport_last_rtt_us(const HamlibTransport *transport);

#endif
