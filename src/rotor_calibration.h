#ifndef ROTOR_CALIBRATION_H
#define ROTOR_CALIBRATION_H

#include <stdbool.h>

#include <gtk/gtk.h>

typedef struct RotorCalib {
    double az_offset_deg;
    double el_offset_deg;
    bool enabled;
} RotorCalib;

typedef bool (*fn_setpos)(double az_deg, double el_deg);
typedef bool (*fn_getpos)(double *az_deg, double *el_deg);

double wrap180(double az_deg);

bool calib_load(const char *rotor_id, RotorCalib *out);
bool calib_save(const char *rotor_id, const RotorCalib *c);

void world_to_mech(double *az_deg_canon, double *el_deg, const RotorCalib *c);
void mech_to_world(double *az_deg_canon, double *el_deg, const RotorCalib *c);

bool calib_run_wizard(GtkWindow *parent,
                      const char *rotor_id,
                      fn_setpos send_setpos,
                      fn_getpos read_getpos,
                      RotorCalib *io_calib);

#endif
