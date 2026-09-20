#pragma once
/* ============================ ESP32-CAM Ultra CCTV v6 ============================
   Compile-time constants. Runtime settings live in the registry (globals.h).
   ================================================================================== */
#define FW_VERSION            "6.0.1"
#define HOSTNAME              "esp32cam"

// ---------- AI-Thinker pin map ----------
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define FLASH_LED_PIN       4
#define FLASH_LEDC_TIMER    LEDC_TIMER_1
#define FLASH_LEDC_CHANNEL  LEDC_CHANNEL_7
#define FLASH_LEDC_MODE     LEDC_LOW_SPEED_MODE
#define FLASH_MAX_DUTY      200
#define MAX_FLASH_ON_MS     (10UL * 60UL * 1000UL)

#define PIR_PIN             13   // optional PIR (free while SD runs in 1-bit mode)

// ---------- Tunables ----------
#define WDT_TIMEOUT_SEC       30
#define CAPTURE_FPS           15          // sensor grab rate
#define POOL_FRAMES           10          // recorder frame pool (PSRAM)
#define FRAME_BUF_SIZE        (96 * 1024) // recorder buffer; larger frames are skipped, never truncated
#define LATEST_BUF_SIZE       (144 * 1024)// live-view / snapshot buffers
#define PREROLL_BYTES         (768 * 1024)
#define PREROLL_MAX_FRAMES    48
#define AVI_MAX_INDEX         120000      // ~4 h at 8 fps, index lives in PSRAM
#define AVI_SYNC_MS           4000        // header heartbeat (power-loss safety)
#define FILE_CHUNK            (16 * 1024)
#define FILE_WORKERS          2
#define MAX_VIEWERS           3
#define MAX_SESSIONS          4
#define SESSION_TTL_MS        (12UL * 3600UL * 1000UL)
#define LOG_LINE              160
#define LOG_RING              60
#define EVENT_RING            40
#define RSSI_RING             48
#define SD_MIN_FREE_BYTES     (300ULL * 1024 * 1024)

// Telegram TLS verification. 0 = setInsecure() (works everywhere, MITM-able).
// Set to 1 after confirming your core has the x509 bundle (arduino-esp32 >= 2.0.14 / 3.x).
#define TELEGRAM_VERIFY_TLS   0

// Motion grid (analysis cells and user zone grid)
#define MG_W 32
#define MG_H 24
#define ZG_W 16
#define ZG_H 12
