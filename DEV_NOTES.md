# GPredict Hamlib Notes

## Capability detection
- Rig: `rigctld_client_probe()` issues `\\dump_state` and safe command probes (`f`, `F`, VFO selection, `\\set_vfo_opt`) and stores results in `RigCaps`. Probe results are cached per session via `last_probe_us` to avoid spamming.
- Rotor: `rotctld_client_probe()` issues `\\dump_state` and probes `p`/`P`, parsing limits when available and caching results in `RotCaps`.
- Both clients use `hamlib_transport` for request/response framing, prompt stripping, timeouts, and single in-flight command enforcement.

## Adding a payload policy
- Start in `src/tracking_policy.h` and `src/tracking_policy.c`.
- Extend `TrackingPolicyInput` with any new inputs you need, keep `TrackingPolicyOutput` pure (no side effects).
- Update the tracking loop call site to fill the new input fields and consume the output.

## Rotor tracking core
- Absolute azimuth helpers live in `src/azel_mapping.h`/`src/azel_mapping.c` (`AzSpan`, `az_unwrap_to_abs`, `az_target_to_nearest_abs`, `az_abs_to_span`).
- Safety window enforcement is in `src/safety_window.h`/`src/safety_window.c`; tune `RotorSafety.stop_abs` and `stop_margin_deg` to model electronic stops.
- Policy tuning uses `TrackPolicy` in `src/tracking_policy.h`/`src/tracking_policy.c` (deadband, max rate, max step, RTT smoothing).

## Adding a rotor span mode
- Add a new enum value to `rot_az_type_t` in `src/rotor-conf.h` and update the UI in `src/sat-pref-rot-editor.c`.
- Extend `AzSpan` handling in `src/azel_mapping.h`/`src/azel_mapping.c`.
- Add or update edge-case tests in `src/azel-mapping-test.c`.

## Adding a rig quirk
- Add a signature match entry to `rig_quirks[]` in `src/rigctld_client.c`.
- Keep quirks keyed to `RigCaps.signature` (derived from `\\dump_state`), not model IDs.
- Use `caps->quirks` or `session->quirks` in logic instead of `if model == ...`.

## Testing
- Unit tests: `make check` (includes `azel-mapping-test`, `safety-window-test`, and `hamlib-mock-test`).
- Mock servers:
  - `python3 scripts/mock_rigctld.py --host 127.0.0.1 --port 4532`
  - `python3 scripts/mock_rotctld.py --host 127.0.0.1 --port 4533`
- `hamlib-mock-test` will skip automatically if Python is not available.
