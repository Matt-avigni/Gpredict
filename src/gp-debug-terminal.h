/*
  Gpredict: Real-time satellite tracking and orbit prediction program

  Copyright (C)  2024  Matteo Avigni

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

#ifndef GP_DEBUG_TERMINAL_H
#define GP_DEBUG_TERMINAL_H

#include <gtk/gtk.h>

G_BEGIN_DECLS

typedef struct _GpDbgTerm GpDbgTerm;

GpDbgTerm *gp_dbg_term_new(const gchar *title);
void       gp_dbg_term_free(GpDbgTerm *term);
void       gp_dbg_term_show(GpDbgTerm *term);
void       gp_dbg_term_log(GpDbgTerm *term, const gchar *fmt, ...)
              G_GNUC_PRINTF(2, 3);

G_END_DECLS

#endif /* GP_DEBUG_TERMINAL_H */
