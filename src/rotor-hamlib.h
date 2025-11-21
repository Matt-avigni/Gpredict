#pragma once
#include "rotor-conf.h"

int gp_hamlib_rot_init(const rotor_conf_t *conf, int model);
int gp_hamlib_rot_set_azel(double az_sky, double el_sky, const rotor_conf_t *conf);
int gp_hamlib_rot_get_azel(double *az_sky, double *el_sky, const rotor_conf_t *conf);
void gp_hamlib_rot_close(void);