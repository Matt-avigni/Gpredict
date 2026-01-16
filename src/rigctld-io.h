#ifndef RIGCTLD_IO_H
#define RIGCTLD_IO_H

#include <glib.h>

GString *rigctld_rxbuf_new(gsize initial_size);
void rigctld_rxbuf_free(GString **buf);
void rigctld_rxbuf_clear(GString *buf);
void rigctld_io_lock_acquire(void);
void rigctld_io_lock_release(void);

typedef enum {
    RIGCTLD_READ_SINGLE = 0,
    RIGCTLD_READ_MULTILINE_RPRT,
    RIGCTLD_READ_MULTILINE_IDLE
} rigctld_read_mode_t;

gssize rigctld_read_reply_line(int fd,
                               GString *buf,
                               gchar *out,
                               gsize out_len,
                               gint timeout_ms,
                               gint *err_out);

gssize rigctld_read_response(int fd,
                             GString *buf,
                             rigctld_read_mode_t mode,
                             gchar *out,
                             gsize out_len,
                             gint base_timeout_ms,
                             gint idle_timeout_ms,
                             gboolean *saw_rprt,
                             gboolean *used_multiline,
                             gint *err_out);

gssize rigctld_read_dump_state(int fd,
                               GString *buf,
                               gchar *out,
                               gsize out_len,
                               gint base_timeout_ms,
                               gint idle_timeout_ms,
                               gint *err_out);

gssize rigctld_drain_idle(int fd,
                          GString *buf,
                          gint idle_timeout_ms,
                          gint *err_out);

#endif
