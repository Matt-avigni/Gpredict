## Process this file with automake to produce Makefile.in

SUBDIRS = sgpsdp

AM_CPPFLAGS = \
	@PACKAGE_CFLAGS@ -I.. \
	-DPACKAGE_DATA_DIR=\""$(datadir)/gpredict"\" \
	-DPACKAGE_PIXMAPS_DIR=\""$(datadir)/pixmaps/gpredict"\" \
	-DPACKAGE_LOCALE_DIR=\""$(prefix)/share/locale"\" \
	-DGTK_DISABLE_SINGLE_INCLUDES \
	-DG_DISABLE_DEPRECATED \
	-DGDK_DISABLE_DEPRECATED \
	-DGSEAL_ENABLE

## 	-DGTK_DISABLE_DEPRECATED \

## Added while preparing for Gtk+ 3 migration:
##  -DGTK_DISABLE_SINGLE_INCLUDES
##  -DG_DISABLE_DEPRECATED
##  -DGDK_DISABLE_DEPRECATED
##  -DGTK_DISABLE_DEPRECATED
##  -DGSEAL_ENABLE

bin_PROGRAMS = gpredict

gpredict_SOURCES = \
    rotor-autocal-direct.c \
    rotor-decision.c \
    rotor-target.c \
    rot-angle.c rot-angle.h \
    rot_math.c rot_math.h \
    ui-popup-quarantine.c ui-popup-quarantine.h \
    ui-status.c ui-status.h \
    rotor_calibration.c rotor_calibration.h \
    tle-autoupdate.c tle-autoupdate.h \
	nxjson/nxjson.c nxjson/nxjson.h \
    sgpsdp/sgp4sdp4.c \
    sgpsdp/sgp4sdp4.h \
    sgpsdp/sgp_in.c \
    sgpsdp/sgp_math.c \
    sgpsdp/sgp_obs.c \
    sgpsdp/sgp_time.c \
    sgpsdp/solar.c \
    about.c about.h \
    compat.c compat.h config-keys.h \
    first-time.c first-time.h \
    gp-debug-terminal.c gp-debug-terminal.h \
    gp-term-view.c gp-term-view.h \
    gpredict-help.c gpredict-help.h \
    gpredict-utils.c gpredict-utils.h \
    gtk-azel-plot.c gtk-azel-plot.h \
    gtk-event-list.c gtk-event-list.h \
    gtk-event-list-popup.c gtk-event-list-popup.h \
    gtk-freq-knob.c gtk-freq-knob.h \
    gtk-polar-plot.c gtk-polar-plot.h \
    gtk-polar-view.c gtk-polar-view.h \
    gtk-polar-view-popup.c gtk-polar-view-popup.h \
    gtk-rig-ctrl.c gtk-rig-ctrl.h \
    rig-mode-dispatch.c rig-mode-dispatch.h \
    hamlib_transport.c hamlib_transport.h \
    rigctld_client.c rigctld_client.h \
    rotctld_client.c rotctld_client.h \
    rigctld-io.c rigctld-io.h \
    rigctld_mgr.c rigctld_mgr.h \
    rotctld_mgr.c rotctld_mgr.h \
    rotctld-parse.c rotctld-parse.h \
    serial-ports.c serial-ports.h \
    serial-ports-darwin.c \
    serial-ports-win32.c \
    gtk-rot-ctrl.c gtk-rot-ctrl.h \
    gtk-rot-knob.c gtk-rot-knob.h \
    gtk-sat-data.c gtk-sat-data.h \
    gtk-sat-list.c gtk-sat-list.h \
    gtk-sat-list-popup.c gtk-sat-list-popup.h \
    gtk-sat-map.c gtk-sat-map.h \
    gtk-sat-map-popup.c gtk-sat-map-popup.h \
    gtk-sat-map-ground-track.c gtk-sat-map-ground-track.h \
    gtk-sat-module.c gtk-sat-module.h \
    gtk-sat-module-popup.c gtk-sat-module-popup.h \
    gtk-sat-module-tmg.c gtk-sat-module-tmg.h \
    gtk-sat-popup-common.c gtk-sat-popup-common.h \
    gtk-sat-selector.c gtk-sat-selector.h \
    gtk-single-sat.c gtk-single-sat.h \
    gtk-sky-glance.c gtk-sky-glance.h \
    gui.c gui.h \
    loc-tree.c loc-tree.h \
    locator.c locator.h \
    main.c \
    map-selector.c map-selector.h \
    map-tools.c map-tools.h \
    menubar.c menubar.h \
    mod-cfg.c mod-cfg.h \
    mod-cfg-get-param.c mod-cfg-get-param.h \
    mod-mgr.c mod-mgr.h \
    orbit-tools.c orbit-tools.h \
    pass-popup-menu.c pass-popup-menu.h \
    pass-to-txt.c pass-to-txt.h \
    predict-tools.c predict-tools.h \
    print-pass.c print-pass.h \
    qth-data.c qth-data.h \
    qth-editor.c qth-editor.h \
    radio-conf.c radio-conf.h \
    rotor-conf.c rotor-conf.h \
    rot-trajectory.c rot-trajectory.h \
    azel_mapping.c azel_mapping.h \
    rotor-cmd-map.c rotor-cmd-map.h \
    rotor-trajectory-planner.c rotor-trajectory-planner.h \
    tracking_policy.c tracking_policy.h \
    safety_window.c safety_window.h \
    payload-profile.c payload-profile.h \
    trsp-conf.c trsp-conf.h \
    trsp-update.c trsp-update.h \
    win32-fetch.c win32-fetch.h \
    sat-cfg.c sat-cfg.h \
    sat-info.c sat-info.h \
    sat-log.c sat-log.h \
    sat-log-browser.c sat-log-browser.h \
    sat-monitor.c sat-monitor.h \
    sat-pass-dialogs.c sat-pass-dialogs.h \
    sat-pref.c sat-pref.h \
    sat-pref-general.c sat-pref-general.h \
    sat-pref-formats.c sat-pref-formats.h \
    sat-pref-qth.c sat-pref-qth.h sat-pref-qth-data.h \
    sat-pref-qth-editor.c sat-pref-qth-editor.h \
    sat-pref-tle.c sat-pref-tle.h \
    sat-pref-debug.c sat-pref-debug.h \
    sat-pref-modules.c sat-pref-modules.h \
    sat-pref-layout.c sat-pref-layout.h \
    sat-pref-refresh.c sat-pref-refresh.h \
    sat-pref-map-view.c sat-pref-map-view.h \
    sat-pref-list-view.c sat-pref-list-view.h \
    sat-pref-polar-view.c sat-pref-polar-view.h \
    sat-pref-single-sat.c sat-pref-single-sat.h \
    sat-pref-interfaces.c sat-pref-interfaces.h \
    sat-pref-rig.c sat-pref-rig.h sat-pref-rig-data.h \
    sat-pref-rig-editor.c sat-pref-rig-editor.h \
    sat-pref-rot.c sat-pref-rot.h sat-pref-rot-data.h \
    sat-pref-rot-editor.c sat-pref-rot-editor.h \
    sat-pref-predict.c sat-pref-predict.h \
    sat-pref-conditions.c sat-pref-conditions.h \
    sat-pref-multi-pass.c sat-pref-multi-pass.h \
    sat-pref-single-pass.c sat-pref-single-pass.h \
    sat-pref-sky-at-glance.c sat-pref-sky-at-glance.h \
    sat-vis.c sat-vis.h \
    save-pass.c save-pass.h \
    time-tools.c time-tools.h \
    tle-tools.c tle-tools.h \
    tle-update.c tle-update.h \
    strnatcmp.c strnatcmp.h

##gpredict_LDADD = ./sgpsdp/libsgp4sdp4.a @PACKAGE_LIBS@
gpredict_LDADD = @PACKAGE_LIBS@

## $(INTLLIBS)

check_PROGRAMS = rotor-cmd-map-test rig-mode-dispatch-test \
    rotctld-parse-test rigctld-io-test \
    azel-mapping-test safety-window-test hamlib-mock-test \
    payload-profile-test ui-status-test
TESTS = rotor-cmd-map-test rig-mode-dispatch-test \
    rotctld-parse-test rigctld-io-test \
    azel-mapping-test safety-window-test hamlib-mock-test \
    payload-profile-test ui-status-test

rotor_cmd_map_test_SOURCES = \
    rotor-cmd-map-test.c \
    rotor-cmd-map.c \
    rotor-trajectory-planner.c
rotor_cmd_map_test_LDADD = @PACKAGE_LIBS@

rig_mode_dispatch_test_SOURCES = \
    rig-mode-dispatch-test.c \
    rig-mode-dispatch.c \
    rig-mode-dispatch.h
rig_mode_dispatch_test_LDADD = @PACKAGE_LIBS@

rotctld_parse_test_SOURCES = \
    rotctld-parse-test.c \
    rotctld-parse.c \
    rotctld-parse.h
rotctld_parse_test_LDADD = @PACKAGE_LIBS@

rigctld_io_test_SOURCES = \
    rigctld-io-test.c \
    rigctld-io.c \
    rigctld-io.h
rigctld_io_test_LDADD = @PACKAGE_LIBS@

azel_mapping_test_SOURCES = \
    azel-mapping-test.c \
    azel_mapping.c \
    azel_mapping.h
azel_mapping_test_LDADD = @PACKAGE_LIBS@

safety_window_test_SOURCES = \
    safety-window-test.c \
    safety_window.c \
    safety_window.h
safety_window_test_LDADD = @PACKAGE_LIBS@

hamlib_mock_test_SOURCES = \
    hamlib-mock-test.c \
    rigctld_client.c \
    rigctld_client.h \
    rotctld_client.c \
    rotctld_client.h \
    hamlib_transport.c \
    hamlib_transport.h \
    rotctld-parse.c \
    rotctld-parse.h \
    radio-conf.h \
    radio-conf-stub.c
hamlib_mock_test_LDADD = @PACKAGE_LIBS@

payload_profile_test_SOURCES = \
    payload-profile-test.c \
    payload-profile.c \
    payload-profile.h
payload_profile_test_LDADD = @PACKAGE_LIBS@

ui_status_test_SOURCES = \
    ui-status-test.c \
    ui-status.c \
    ui-status.h
ui_status_test_LDADD = @PACKAGE_LIBS@
