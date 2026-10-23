# Next phone session: 60 fps regular mode (2026-10-23 decision)

Decisions (user, 2026-10-23):
- VIO recordings and lens calibration use the **60 fps regular session** (`session std`), the same mode as live flight.
- **Focus fixed at 0.5 dpt** (about 2 m; app 0.6.6 default). Stepped focus may come later.
- **Manual exposure**: 2 ms, ISO 500. Earlier tests: std60 used 2 ms / ISO 400, and HS auto exposure chose 4 ms / ISO 243 indoors, so 2 ms / ISO 500 gives about the same brightness with half the motion blur. Adjust on site if the image is too dark.
- VIO baseline: `config/base_nozupt` (ZUPT off). Online intrinsics stay off.

Recording script: scratchpad `rec60.sh <ip:port> "<guide>" <secs> [exp_us] [iso]`. It waits for meta.json, then force-stops the app and prints the active camera clients, which must be empty.

## Order
1. Install arlat 0.6.6 (`phone_stereo/arlat/build/arlat.apk`) and check `app_version` in meta.
2. **Lens calibration at 0.5 dpt, std60, tripod.** Use the monitor board (mirror window off). Shoot several recordings with 4 s holds per pose: tilted board, board reaching the image edges, 2–3 distances.
   - Fit them jointly, then fit each recording separately.
   - Accept only if fx agrees within ±0.3 % and the principal point within ±3 px across recordings.
3. **Flight-like VIO recordings (std60).** Each starts still for ≥ 4 s (static init) with the board in view for GT.
   - Hover-like slow motion with half-stops: 6
   - Still, then fast takeoff-like motion: 5
   - Long stills, 30–60 s: 3
   - Fast rotation, roll/pitch like a drone: 3
4. **240 fps HS comparison**: 1–2 recordings of the same fast motions, to check rolling shutter at speed. The 60 regular readout is 6.5 ms vs 2.9 ms for 240 HS.
5. Optional: measure skew per mode (720p 240/120, 1080p 120) with 5 s clips.

Phone care: cool below 36 °C between takes (recording stops above 40 °C SKIN). Never leave the camera running. Delete from the phone only recordings verified on the PC, and only when the user asks.
