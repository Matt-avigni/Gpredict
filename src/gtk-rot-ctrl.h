#ifndef GTK_ROT_CTRL_H
#define GTK_ROT_CTRL_H

#include <gtk/gtk.h>

G_BEGIN_DECLS

#define GTK_TYPE_ROT_CTRL            (gtk_rot_ctrl_get_type())
#define GTK_ROT_CTRL(obj)            (G_TYPE_CHECK_INSTANCE_CAST ((obj), GTK_TYPE_ROT_CTRL, GtkRotCtrl))
#define GTK_IS_ROT_CTRL(obj)         (G_TYPE_CHECK_INSTANCE_TYPE ((obj), GTK_TYPE_ROT_CTRL))
#define GTK_ROT_CTRL_CLASS(klass)    (G_TYPE_CHECK_CLASS_CAST ((klass), GTK_TYPE_ROT_CTRL, GtkRotCtrlClass))
#define GTK_IS_ROT_CTRL_CLASS(klass) (G_TYPE_CHECK_CLASS_TYPE ((klass), GTK_TYPE_ROT_CTRL))
#define GTK_ROT_CTRL_GET_CLASS(obj)  (G_TYPE_INSTANCE_GET_CLASS ((obj), GTK_TYPE_ROT_CTRL, GtkRotCtrlClass))

typedef struct _GtkRotCtrl        GtkRotCtrl;
typedef struct _GtkRotCtrlClass   GtkRotCtrlClass;
typedef struct _gtk_sat_module    GtkSatModule;
typedef struct _GtkSatModuleClass GtkSatModuleClass;

/* GObject type getter */
GType      gtk_rot_ctrl_get_type   (void) G_GNUC_CONST;

/* Public API */
GtkWidget *gtk_rot_ctrl_new        (GtkSatModule *module);
void       gtk_rot_ctrl_update     (GtkRotCtrl *ctrl, gdouble t);
void       gtk_rot_ctrl_select_sat (GtkRotCtrl *ctrl, gint catnum);
void       gtk_rot_ctrl_request_close(GtkRotCtrl *ctrl);
gboolean   gtk_rot_ctrl_can_destroy(GtkRotCtrl *ctrl);

G_END_DECLS

#endif /* GTK_ROT_CTRL_H */
