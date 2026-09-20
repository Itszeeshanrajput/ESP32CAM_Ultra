/* ==========================================================================
   ESP32-CAM ULTRA CCTV v6
   Board: AI Thinker ESP32-CAM  |  Core: Arduino-ESP32 2.0.14+ or 3.x
   Tools > Partition Scheme: "Minimal SPIFFS (1.9MB APP with OTA)" or "Default 4MB with OTA"
   Tools > PSRAM: Enabled
   Optional library: PubSubClient (enables MQTT + Home Assistant discovery)

   Layout of this sketch (all files in this folder):
     config.h        constants + pins
     globals.h       logging, time, settings registry, accounts/sessions
     avi.h           AVI writer v2 / parser / repair
     storage.h       dated folders, retention, loop recording, events, boot repair
     services.h      log writer, flash, WiFi manager, Telegram, webhook, MQTT, OTA
     camera_tasks.h  camera, motion engine, capture/stream/file/record/maintenance tasks
     web.h           REST API + auth + routes
     web_ui.h        GENERATED gzip of ui/index.html  (python3 tools/build_ui.py)

   Task map  (core 0: camera + live)      capture(3) stream(2) netMedia(2)
             (core 1: storage + control)  record(2) file x2(1) maint(1) log(1) notify(1) loop(1)
   ========================================================================== */
#include "web.h"

SET_LOOP_TASK_STACK_SIZE(10 * 1024);   // web handlers build large strings (v6.1: trimmed 16K -> 10K for internal-RAM headroom)

void initWatchdog() {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t cfg = {}; cfg.timeout_ms = WDT_TIMEOUT_SEC * 1000; cfg.idle_core_mask = 0; cfg.trigger_panic = true;
  esp_task_wdt_reconfigure(&cfg);                        // core 3.x initialises the WDT itself
#else
  esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
#endif
}

void timeRestore() {                                     // keep filenames/retention sane after a reboot without NTP
  if (timeOK()) return;
  Preferences p; p.begin("sys", false); uint32_t e = p.getULong("lastep", 0); p.end();
  if (e > 1700000000UL) { struct timeval tv = {(time_t)(e + 60), 0}; settimeofday(&tv, NULL); timeApprox = true; }
}
void timePersist() {
  static uint32_t last = 0; if (!timeOK() || (last && millis() - last < 1800000UL)) return; last = millis();
  Preferences p; p.begin("sys", false); p.putULong("lastep", (uint32_t)time(nullptr)); p.end();
}

void memTrace(const char* tag) {                            // staged boot diagnostics: finds where internal RAM goes (v6.1)
  size_t fx = heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  size_t big = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  logmsg(LVL_INFO, "MEM", "%s stage: heap %u dma %u big %u psram %u", tag, (unsigned)ESP.getFreeHeap(), (unsigned)fx, (unsigned)big, (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}

void setup() {
  Serial.begin(115200); delay(200);
  Serial.println("\n=== ESP32-CAM ULTRA CCTV v" FW_VERSION " ===");
  // No-card / marginal-card SD errors are expected and handled by our own throttled logs - mute the IDf driver spam (v6.1)
  esp_log_level_set("sdmmc_common", ESP_LOG_NONE); esp_log_level_set("sdmmc_cmd", ESP_LOG_NONE);
  esp_log_level_set("vfs_fat_sdmmc", ESP_LOG_NONE); esp_log_level_set("diskio_sdmmc", ESP_LOG_NONE);
  setCpuFrequencyMhz(240);
  initWatchdog(); esp_task_wdt_add(NULL);

  logMx = xSemaphoreCreateMutex(); logQ = xQueueCreate(24, sizeof(LogLine));
  sdMutex = xSemaphoreCreateMutex(); viewMx = xSemaphoreCreateMutex();
  esp_reset_reason_t rr = esp_reset_reason(); lastResetReasonStr = resetReasonString(); wasBadReset = isBadReset(rr);

  settingsLoad(); loadAccounts(); applyTimezone(); timeRestore();
  pushResetHistory(lastResetReasonStr);
  if (S_pir) pinMode(PIR_PIN, INPUT_PULLDOWN);
  initFlashPWM();
  memTrace("base");                                          // after rtos + globals, before subsystem init

  if (sdMount()) sdCardMounted = true;      // free-space scan, repair and event load happen later in maintTask
  bool bufOk = allocBuffers();
  memTrace("sdfs");
  cameraReady = initCamera();
  notifyQ = xQueueCreate(4, sizeof(Notify)); fileQ = xQueueCreate(4, sizeof(FileJob));
  memTrace("camera");                                        // camera + app buffers live here (PSRAM expected)

  logmsg(LVL_INFO, "BOOT", "Firmware v%s started", FW_VERSION);
  logmsg(wasBadReset ? LVL_WARN : LVL_INFO, "BOOT", "Reset reason: %s", lastResetReasonStr.c_str());
  logmsg(cameraReady ? LVL_INFO : LVL_ERROR, "CAM", cameraReady ? "Camera OK" : "CAMERA INIT FAILED");
  logmsg(sdCardMounted ? LVL_INFO : LVL_WARN, "SD", sdCardMounted ? "Card mounted" : "No card - live view only");
  logmsg(recSupported ? LVL_INFO : LVL_WARN, "SYS", recSupported ? "PSRAM OK - recording enabled" : "No PSRAM/buffers - recording disabled, live view only");
  if (!bufOk) logmsg(LVL_ERROR, "SYS", "Buffer allocation incomplete");
  updateNight(true);

  wifiStart(); registerRoutes(); mediaServer.begin(); mediaServer.setNoDelay(true);

  xTaskCreatePinnedToCore(logTask,       "log",    4096,  NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(notifyTask,    "notify", 9216, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(recordTask,    "record", 8192,  NULL, 2, NULL, 1);
  xTaskCreatePinnedToCore(maintTask,     "maint",  5120,  NULL, 1, NULL, 1);
  for (int i = 0; i < FILE_WORKERS; i++) xTaskCreatePinnedToCore(fileTask, "file", 5120, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(captureTask,   "capture", 8192, NULL, 3, NULL, 0);
  xTaskCreatePinnedToCore(streamTask,    "stream", 4096,  NULL, 2, NULL, 0);
  xTaskCreatePinnedToCore(netMediaTask,  "media",  5120,  NULL, 2, NULL, 0);
  memTrace("tasks");                                         // after wifi + all tasks started

  logmsg(LVL_INFO, "BOOT", "Ready. UI :80  media :81  AP '%s' pass '%s' (only shown here)", apSsid, apPass);
  Serial.println("Serial recovery commands: RESETPW | FACTORY | REBOOT | INFO");
}

void loop() {
  esp_task_wdt_reset();
  uint32_t h = ESP.getFreeHeap(); if (h < minFreeHeapSeen) minFreeHeapSeen = h;
  if (h >= MEM_CRIT) { server.handleClient(); mqttService(); }     // web/MQTT need internal heap to build responses (v6.1)
  wifiService(); serialConsole();
  if (otaStarted) ArduinoOTA.handle();
  uint32_t now = millis();

  // ---- self-healing heap watchdog: reboot instead of an unrecoverable freeze (v6.1) ----
  static uint32_t memLowSince = 0;
  if (h < MEM_DEAD) {
    if (!memLowSince) memLowSince = millis();
    if ((uint32_t)(now - memLowSince) > 8000) {
      logmsg(LVL_ERROR, "MEM", "Heap critical (%u B free, min %u B). Rebooting to recover.", (unsigned)h, (unsigned)minFreeHeapSeen);
      Serial.flush(); vTaskDelay(pdMS_TO_TICKS(300)); requestReboot();
    }
  } else memLowSince = 0;

  // ---- 30 s heap telemetry so serial shows the exact memory trend (v6.1) ----
  static uint32_t lastMem = 0;
  if (now - lastMem > 30000) {
    lastMem = now;
    logmsg(LVL_INFO, "MEM", "heap %u min %u dma %u psram %u sd:%d/%d vw:%d",
           (unsigned)h, (unsigned)minFreeHeapSeen,
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
           (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM), (int)sdState, sdCardMounted ? 1 : 0, (int)viewerCount);
  }

  if (flashPulseUntil && reached(now, flashPulseUntil)) { flashPulseUntil = 0; setFlashDuty(flashBrightness); }
  if (manualFlashSince && now - manualFlashSince > MAX_FLASH_ON_MS) { flashBrightness = 0; manualFlashSince = 0; setFlashDuty(0); logmsg(LVL_WARN, "FLASH", "Auto-off after 10 min (safety)"); }

  static uint32_t lastSlow = 0;
  if (now - lastSlow > 30000) { lastSlow = now; updateNight(false); }
  timePersist();

  static uint32_t lastHeapLog = 0;
  if (now - lastHeapLog > 300000) { lastHeapLog = now; if (h < 40000) logmsg(LVL_WARN, "SYS", "Low heap %u B", (unsigned)h); }

  static bool bootNotified = false;                       // one system message per boot, once WiFi is up
  if (!bootNotified && WiFi.status() == WL_CONNECTED && now > 15000) {
    bootNotified = true;
    if (S_tgsys) { char t[100]; snprintf(t, sizeof t, "Online. Reset: %s. IP %s", lastResetReasonStr.c_str(), WiFi.localIP().toString().c_str()); notifySystem(t); }
  }
  delay(2);
}
