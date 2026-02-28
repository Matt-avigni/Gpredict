/*
 * Copyright (C) 2026 Matteo Avigni
 *
 * This file is part of Gpredict and distributed under the
 * GNU General Public License version 2 or later.
 */

#include <glib.h>

#include "ui-status.h"

static gint failures = 0;

static void expect_true(gboolean cond, const gchar *msg)
{
    if (cond)
        return;

    g_printerr("FAIL: %s\n", msg);
    failures++;
}

static void expect_rotor(RotorStateSnapshot snap,
                         RotorUiStatus expected,
                         const gchar *msg)
{
    RotorUiStatus got = rotor_compute_ui_status(&snap);

    if (got == expected)
        return;

    g_printerr("FAIL: %s (got=%s expected=%s)\n",
               msg,
               rotor_ui_status_to_string(got),
               rotor_ui_status_to_string(expected));
    failures++;
}

static void expect_radio_from_window(const RigUiCommandWindow *window,
                                     gint64 now_us,
                                     gboolean control_active,
                                     gboolean engaging,
                                     gboolean hard_error,
                                     gboolean link_lost,
                                     gboolean degraded,
                                     RadioUiStatus expected,
                                     const gchar *msg)
{
    RigUiCommandWindowStats stats = { 0 };
    RigStateSnapshot snap = { 0 };
    RadioUiStatus got;

    rig_ui_command_window_get_stats(window,
                                    now_us,
                                    RIG_UI_ACTIVE_WINDOW_MS,
                                    &stats);

    snap.control_active = control_active;
    snap.engaging = engaging;
    snap.hard_error = hard_error;
    snap.link_lost = link_lost;
    snap.degraded = degraded;
    snap.active_flow = stats.active;
    snap.consecutive_link_failures = stats.consecutive_link_failures;
    snap.link_fail_count_in_window = stats.link_fail_count;
    snap.total_fail_count_in_window = stats.total_fail_count;
    snap.ok_count_in_window = stats.ok_count;

    got = radio_compute_ui_status(&snap);
    if (got == expected)
        return;

    g_printerr("FAIL: %s (got=%s expected=%s window[ok=%u reject=%u link=%u consec=%u active=%d])\n",
               msg,
               radio_ui_status_to_string(got),
               radio_ui_status_to_string(expected),
               stats.ok_count,
               stats.reject_count,
               stats.link_fail_count,
               stats.consecutive_link_failures,
               stats.active ? 1 : 0);
    failures++;
}

static void run_rig_tests(void)
{
    RigUiCommandWindow window;
    gint64 t0 = 1000000;

    rig_ui_command_window_init(&window);

    /* a) 5 OK commands while active => STABLE */
    for (guint i = 0; i < 5; i++)
        rig_ui_command_window_record(&window, RIG_UI_CMD_OK, t0 + (i * 100000));
    expect_radio_from_window(&window, t0 + 600000, TRUE, FALSE, FALSE, FALSE, FALSE,
                             RADIO_UI_STATUS_STABLE,
                             "rig stable with 5 OK");

    /* b) 2 rejects in window while active => DEGRADED */
    rig_ui_command_window_init(&window);
    rig_ui_command_window_record(&window, RIG_UI_CMD_OK, t0 + 100000);
    rig_ui_command_window_record(&window, RIG_UI_CMD_COMMAND_REJECT, t0 + 200000);
    rig_ui_command_window_record(&window, RIG_UI_CMD_OK, t0 + 300000);
    rig_ui_command_window_record(&window, RIG_UI_CMD_COMMAND_REJECT, t0 + 400000);
    rig_ui_command_window_record(&window, RIG_UI_CMD_OK, t0 + 500000);
    expect_radio_from_window(&window, t0 + 600000, TRUE, FALSE, FALSE, FALSE, FALSE,
                             RADIO_UI_STATUS_DEGRADED,
                             "rig degraded with 2 rejects");

    /* c) 2 consecutive timeouts/link fails => LINK LOST */
    rig_ui_command_window_init(&window);
    rig_ui_command_window_record(&window, RIG_UI_CMD_OK, t0 + 100000);
    rig_ui_command_window_record(&window, RIG_UI_CMD_OK, t0 + 200000);
    rig_ui_command_window_record(&window, RIG_UI_CMD_OK, t0 + 300000);
    rig_ui_command_window_record(&window, RIG_UI_CMD_LINK_FAIL, t0 + 400000);
    rig_ui_command_window_record(&window, RIG_UI_CMD_LINK_FAIL, t0 + 500000);
    expect_radio_from_window(&window, t0 + 600000, TRUE, FALSE, FALSE, FALSE, FALSE,
                             RADIO_UI_STATUS_LINK_LOST,
                             "rig link lost with 2 consecutive link failures");

    /* d) 3 link fails in last 5 => LINK LOST */
    rig_ui_command_window_init(&window);
    rig_ui_command_window_record(&window, RIG_UI_CMD_LINK_FAIL, t0 + 100000);
    rig_ui_command_window_record(&window, RIG_UI_CMD_OK, t0 + 200000);
    rig_ui_command_window_record(&window, RIG_UI_CMD_LINK_FAIL, t0 + 300000);
    rig_ui_command_window_record(&window, RIG_UI_CMD_OK, t0 + 400000);
    rig_ui_command_window_record(&window, RIG_UI_CMD_LINK_FAIL, t0 + 500000);
    expect_radio_from_window(&window, t0 + 600000, TRUE, FALSE, FALSE, FALSE, FALSE,
                             RADIO_UI_STATUS_LINK_LOST,
                             "rig link lost with 3 link fails in window");

    /* e) inactive (connected) => STANDBY */
    rig_ui_command_window_init(&window);
    rig_ui_command_window_record(&window, RIG_UI_CMD_OK, t0 + 100000);
    {
        gint64 idle_now = window.last_command_us +
            ((RIG_UI_ACTIVE_WINDOW_MS + 200) * 1000);
        expect_radio_from_window(&window,
                                 idle_now,
                                 TRUE, FALSE, FALSE, FALSE, FALSE,
                                 RADIO_UI_STATUS_STANDBY,
                                 "rig standby when inactive");
    }

    /* inactive with stale reliability failures still => STANDBY */
    rig_ui_command_window_init(&window);
    rig_ui_command_window_record(&window, RIG_UI_CMD_OK, t0 + 100000);
    rig_ui_command_window_record(&window, RIG_UI_CMD_COMMAND_REJECT, t0 + 200000);
    rig_ui_command_window_record(&window, RIG_UI_CMD_LINK_FAIL, t0 + 300000);
    {
        gint64 idle_now = window.last_command_us +
            ((RIG_UI_ACTIVE_WINDOW_MS + 200) * 1000);
        expect_radio_from_window(&window,
                                 idle_now,
                                 TRUE, FALSE, FALSE, FALSE, FALSE,
                                 RADIO_UI_STATUS_STANDBY,
                                 "rig standby when inactive with stale failures");
    }

    /* f) startup/configuration failure => ERROR */
    rig_ui_command_window_init(&window);
    expect_radio_from_window(&window, t0 + 100000, TRUE, FALSE, TRUE, FALSE, FALSE,
                             RADIO_UI_STATUS_ERROR,
                             "rig error on hard error flag");
}

static void run_rotor_tests(void)
{
    RotorStateSnapshot snap = { 0 };

    /* a) not engaged => DISENGAGED */
    snap.control_active = FALSE;
    expect_rotor(snap, ROTOR_UI_STATUS_DISENGAGED, "rotor disengaged");

    /* b) connecting/handshake => ENGAGING */
    snap.control_active = TRUE;
    snap.engaging = TRUE;
    expect_rotor(snap, ROTOR_UI_STATUS_ENGAGING, "rotor engaging");

    /* c) moving => MOVING */
    snap.engaging = FALSE;
    snap.moving = TRUE;
    expect_rotor(snap, ROTOR_UI_STATUS_MOVING, "rotor moving");

    /* d) on target => ON TARGET */
    snap.moving = FALSE;
    snap.on_target = TRUE;
    expect_rotor(snap, ROTOR_UI_STATUS_ON_TARGET, "rotor on target");

    /* e) degraded => DEGRADED */
    snap.on_target = FALSE;
    snap.degraded = TRUE;
    expect_rotor(snap, ROTOR_UI_STATUS_DEGRADED, "rotor degraded");

    /* f) link lost => LINK LOST */
    snap.degraded = FALSE;
    snap.link_lost = TRUE;
    expect_rotor(snap, ROTOR_UI_STATUS_LINK_LOST, "rotor link lost");

    /* g) daemon/config failure => ERROR */
    snap.link_lost = FALSE;
    snap.hard_error = TRUE;
    expect_rotor(snap, ROTOR_UI_STATUS_ERROR, "rotor hard error");

    /* h) engaged but idle => STANDBY */
    snap.hard_error = FALSE;
    snap.control_active = TRUE;
    snap.engaging = FALSE;
    snap.degraded = FALSE;
    snap.link_lost = FALSE;
    snap.moving = FALSE;
    snap.on_target = FALSE;
    expect_rotor(snap, ROTOR_UI_STATUS_STANDBY, "rotor standby");
}

int main(void)
{
    expect_true(g_strcmp0(rotor_ui_status_to_string(ROTOR_UI_STATUS_ON_TARGET),
                          "ON TARGET") == 0,
                "rotor status string ON TARGET");
    expect_true(g_strcmp0(radio_ui_status_to_string(RADIO_UI_STATUS_STABLE),
                          "STABLE") == 0,
                "radio status string STABLE");

    run_rig_tests();
    run_rotor_tests();

    if (failures > 0)
    {
        g_printerr("ui-status-test: %d failure(s)\n", failures);
        return 1;
    }

    g_print("ui-status-test: OK\n");
    return 0;
}
