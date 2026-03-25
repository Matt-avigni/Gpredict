# School Setup

This guide is for classroom or club setups using Gpredict with hamlib.

## Prerequisites
- Install hamlib so `rigctld` and `rotctld` are available.
- Connect the radio/rotator to the computer (USB or serial).
- Start `rigctld` and `rotctld` before engaging radio/rotator control.

Example commands (adjust model/device/baud to your hardware):
```
rigctld -m <rig_model> -r <serial_device> -s <baud> -t 4532
rotctld -m <rot_model> -r <serial_device> -s <baud> -t 4533
```

## Endpoints and Ports
- rigctld (radio control)
  - Host and Port fields are in the Radio configuration editor.
  - Default: `127.0.0.1` and `4532`.
- rotctld (rotator control)
  - Host and Port fields are in the Rotator configuration editor.
  - Default: `127.0.0.1` and `4533`.
- Use the "Test connection" button to verify each endpoint.

## Model and Mode Selection
- Choose the radio model in the Radio configuration editor.
- Pick a radio mode that matches the model:
  - IC-9700: Simplex, Split, or Full-duplex MAIN/SUB.
  - IC-705: Simplex or Split.
  - IC-905: Simplex or Split.

## Quick Troubleshooting
- Cannot connect:
  - Confirm `rigctld`/`rotctld` is running.
  - Verify the Host and Port fields (defaults are 127.0.0.1:4532 and 127.0.0.1:4533).
  - Try the "Test connection" buttons.
- RPRT -6 or other RPRT errors:
  - The daemon rejected the command. Check the hamlib model number and device settings.
  - Make sure the radio/rotator is powered on and connected.
- Test connection succeeds but control fails:
  - Verify the rig/rotator model, serial device path, and baud rate.
  - Restart `rigctld`/`rotctld` after changing settings.
