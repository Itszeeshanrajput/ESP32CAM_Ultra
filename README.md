# ESP32-CAM Ultra CCTV v6

Feature-complete, **memory-hardened** CCTV firmware for the AI-Thinker ESP32-CAM.

Live view, motion alarms and continuous / motion-only / time-lapse recording to microSD,
plus Telegram and MQTT notifications — served from a phone-friendly web UI that lives in
flash and is streamed to the client, so it stays responsive even on this board's tight
internal-RAM budget.

> **Why "memory-hardened"?** On the standard arduino-esp32 core, the WiFi + LWIP stack
> lives entirely in internal RAM (PSRAM is not used for it in prebuilt cores). That leaves
> ~40-45 KB of free heap with a camera attached, and a phone associating with the hotspot
> consumes another ~30-40 KB. This firmware was diagnosed and hardened against exactly that:
> heap/DMA floors, flood-guarded logging, self-healing watchdog, and guarded SD retries —
> so the AP and UI keep responding instead of freezing when memory gets tight.

---

## Features

- **Live MJPEG stream** on `:81` (authenticated, multi-viewer) + **Web UI** on `:80`
  (gzip served from flash, dark theme, JS polls JSON APIs — no big heap strings).
- **Motion detection** in PSRAM (32×24 grid scoring, sensitivity / cooldown / hit-frames,
  pre-roll + post-roll, event thumbnails and clips, optional PIR).
- **Recording**: continuous, motion-only or time-lapse; AVI v2 writer with the frame index
  in PSRAM; power-loss safe (header heartbeat every 4 s, automatic repair on boot);
  15-min chunk rollover; age/space retention cleanup.
- **Alerts**: Telegram (photo + text, `/snap` command) and MQTT (optional
  `PubSubClient` for Home Assistant).
- **Security**: hashed passwords with random salt, session cookies, CSRF tokens on all
  state-changing calls, forced password change on first login, role-based access
  (admin/viewer), authenticated stream endpoint, login lockout.
- **Wi-Fi**: up to 3 saved networks, automatic fallback hotspot, captive-portal DNS,
  mDNS (`http://esp32cam.local`).
- **Camera**: resolution / quality / brightness / contrast / saturation, white balance,
  day-night gain control, flash LED, night schedule.
- **Ops**: serial recovery commands (`RESETPW | FACTORY | REBOOT | INFO`), web OTA,
  health diagnostics (heap low-water, dropped frames, SD write timings, viewers).

## Requirements

| Item | Notes |
|---|---|
| Hardware | AI-Thinker ESP32-CAM + microSD (use a known-good card) + stable 5 V, ~2 A supply |
| Core | arduino-esp32 **2.0.14+ or 3.x** (developed/tested on **3.3.11**) |
| Board settings | **AI Thinker ESP32-CAM**, **PSRAM: Enabled**, partition **Minimal SPIFFS (1.9 MB APP with OTA)** |
| Optional library | `PubSubClient` (enables MQTT / Home Assistant support) |

## Build & flash

Keep **every file in one folder** named `ESP32CAM_Ultra_v6`, open the `.ino`, and build.

```bash
# arduino-cli
arduino-cli compile --fqbn esp32:esp32:esp32cam:PartitionScheme=min_spiffs ESP32CAM_Ultra_v6
```

Web UI edits are one step away from a reflash:

```bash
python3 tools/build_ui.py        # ui/index.html -> web_ui.h (gzip C array) - run after any UI change
```

**Flashing this board.** On boards where the USB-serial RTS line is wired to the EN/reset
pin (common on ESP32-CAM modules) the stock reset sequence kicks the chip out of download
mode. Use `esptool` with `--before no-reset`:

```bash
# causes the board's serial control pins to behave: no auto-reset into the ROM loader
esptool.py --port /dev/ttyUSB0 --baud 115200 --before no-reset \
  write-flash --flash-mode dio 0x0 ESP32CAM_Ultra_v6.ino.merged.bin
```

Sequence: **hold BOOT → tap RESET → keep holding BOOT** until the stub reports
`Stub flasher running` (~15 s at 115200), then release.

## First boot & default credentials

1. Connect to the hotspot **`ESP32CAM-XXXX`** (password **`cctv-XXXXXX`** — derived from the
   module's MAC, also printed on serial at boot).
2. Open `http://192.168.4.1`, sign in with **`admin` / `admin`** — you are forced to pick a
   real password.
3. Network tab: join your 2.4 GHz Wi-Fi. Afterwards reach the camera at `http://esp32cam.local`.
4. Serial is 115200 baud. Recovery commands: `RESETPW` `FACTORY` `REBOOT` `INFO`.

## Project layout

| File | Role |
|---|---|
| `ESP32CAM_Ultra_v6.ino` | setup/loop, watchdog, staged memory diagnostics, heap watchdog |
| `globals.h` | settings registry (NVS), accounts/sessions, logging ring, memory floors |
| `camera_tasks.h` | capture, motion, recording, stream, media server, maintenance task |
| `web.h` | HTTP API + auth on `:80` |
| `services.h` | logging task, Wi-Fi manager, Telegram, MQTT, serial console |
| `storage.h` | SD mount, paths, retention/space management |
| `avi.h` | power-loss-safe AVI v2 writer/reader |
| `web_ui.h` | **generated** — gzip of `ui/index.html` (run `tools/build_ui.py`) |
| `ui/index.html` | the web interface source |
| `tools/build_ui.py` | UI → header compiler |

## Memory architecture

- All frame buffers, the record frame pool, AVI index and motion data live in **PSRAM**.
- The web page + assets are **gzip in flash** and streamed (`send_P`) — no large heap strings.
- Firmware guards: `MEM_LOW` / `MEM_CRIT` / `MEM_DEAD` thresholds, a **DMA floor**
  (SDMMC ops are barred wholesale below it, since every SD op would fail `0x101`), retry
  throttles on the recorder and log, and a heap watchdog that reboots instead of freezing.
- The camera is initialised at the configured resolution (not always UXGA) to minimise the
  internal DMA footprint; resolution changes that fail to allocate revert safely.

## Changelog

### v6.1 — memory & stability hardening
- Root-caused UI/AP freezes: WiFi/LWIP occupy internal RAM; a client association and any
  SD traffic could starve the system. Added a hard **DMA floor** so SD activity stops
  entirely below it — the `0x101`/`0x107` error floods are eliminated, not just logged.
- SD no-card driver logs muted (expected condition, handled by our own throttled logs);
  first mount probe delayed, probe backoff to 5 min after repeated no-card results.
- Failed clip-open retries capped (once/2 s) and logged once/10 s; a 60 s silent SD fault
  window kicks in instead of a retry storm.
- Camera initialised at the configured resolution instead of UXGA (internal DMA savings);
  failed resolution changes revert instead of blacking the camera.
- SoftAP limited to 2 clients (smaller per-station IRAM footprint); pure-AP mode when no
  STA network is configured; task-stack trims; motion scoring at 2 Hz.
- Staged `[MEM] base/sdfs/camera/tasks` telemetry to see exactly where internal RAM goes.

### v6.0.1
- Compile/link fixes: `globals.h` was never included (root cause of the error flood);
  renamed `logf`→`logmsg`, `LOG_*`→`LVL_*`, `SI/SB/SS`→`CI/CB/CS` to avoid clashes on core 3.x.
- SD: 12 open files allowed, pull-ups + 3 mount retries (last at 10 MHz), remount only after
  5 errors and max once a minute; a full card is not treated as a fault.
- `/api/status` no longer touches SD (it used to block the web server); slow FAT/scan/repair
  jobs moved to a background task.
- Motion hold-off no longer breaks after 24.8 days of uptime.

## Known limits (honest list)

- No RTSP, light-sensor auto-night (schedule only), AI verification, or gzip/SSE push
  (UI polls every ~2 s). ESP32-CAM internal RAM is too small for RTSP + TLS + 3 viewers.
- microSD runs in 1-bit mode (board wiring). Marginal cards will be detected and throttled
  (clean log + auto remount), not flood the serial.
- Storage math: VGA @ 8 fps ≈ 10–17 GB/day — use Motion-only or short retention on small
  cards.
- **Never port-forward the camera.** Use a VPN (Tailscale / WireGuard) for remote access.

## License

[MIT](LICENSE)