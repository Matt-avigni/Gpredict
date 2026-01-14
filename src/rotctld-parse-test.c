#include <glib.h>

#include "rotctld-parse.h"

int main(void)
{
    const gchar *sample_rot =
        "1\n"
        "603\n"
        "min_az=-180.000000\n"
        "max_az=450.000000\n"
        "min_el=0.000000\n"
        "max_el=180.000000\n";
    const gchar *sample_rig =
        "1\n"
        "3081\n"
        "0\n";
    gint model = 0;

    if (!parse_dump_state_model_id(sample_rot, &model))
        return 1;

    if (model != 603)
        return 2;

    model = 0;
    if (!parse_dump_state_model_id(sample_rig, &model))
        return 3;

    if (model != 3081)
        return 4;

    return 0;
}
