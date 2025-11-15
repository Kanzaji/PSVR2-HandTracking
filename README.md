# PSVR2 Hand Tracking
Mercury-Based SteamVR Hand Tracking driver for PSVR2.

This is currently W.I.P and is not in usable state for the end-user.

## Dev Setup
Requires:
- Monado repository in `monado` folder.
- SDL2-2.32.10 in `deps\SDL2-2.32.10`
- OpenCV in `deps\OpenCV`
- ONNXRuntime in `deps\ONNXRuntime`

After build, ONNXRuntime, SLD and OpenCV dlls need to be copied to the output.
You need to manually register the SteamVR driver to SteamVR.
