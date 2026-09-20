#pragma once
#include "services.h"

// =============================== CAMERA SETUP ================================
struct SizeOpt { int fs; const char* label; };
const SizeOpt SIZES[] = {
  {FRAMESIZE_QVGA, "QVGA 320x240"}, {FRAMESIZE_CIF, "CIF 400x296"}, {FRAMESIZE_HVGA, "HVGA 480x320"}, {FRAMESIZE_VGA, "VGA 640x480"},
  {FRAMESIZE_SVGA, "SVGA 800x600"}, {FRAMESIZE_XGA, "XGA 1024x768"}, {FRAMESIZE_HD, "HD 1280x720"},
  {FRAMESIZE_SXGA, "SXGA 1280x1024"}, {FRAMESIZE_UXGA, "UXGA 1600x1200"},
};
const int SIZES_N = sizeof(SIZES) / sizeof(SIZES[0]);

void applyGain() {   // only touches gain: never overwrites the user's contrast/saturation (v5 bug H8)
  sensor_t* s = esp_camera_sensor_get(); if (!s) return;
  bool boost = S_dyntune ? nightState : S_lowlight;
  s->set_gainceiling(s, boost ? GAINCEILING_16X : (S_dyntune ? GAINCEILING_4X : GAINCEILING_2X));
  s->set_aec2(s, boost ? 1 : 0);
}
void applyCamSettings() {
  sensor_t* s = esp_camera_sensor_get(); if (!s) return;
  int prev = (int)s->status.framesize;
  int fs = FRAMESIZE_VGA; for (int i = 0; i < SIZES_N; i++) if (SIZES[i].fs == S_fsize) fs = S_fsize;
  if (!psramFound() && fs > FRAMESIZE_CIF) fs = FRAMESIZE_CIF;
  if (fs != prev && s->set_framesize(s, (framesize_t)fs) != 0) {                       // upscale can fail (DMA) - revert instead of black camera (v6.1)
    logmsg(LVL_ERROR, "CAM", "Resolution change to %d failed - keeping %d", fs, prev);
    s->set_framesize(s, (framesize_t)prev); S_fsize = prev;
    const Setting* st = findSetting("fsize"); if (st) settingSave(*st);
  }
  s->set_quality(s, S_quality); s->set_brightness(s, S_bright); s->set_contrast(s, S_contrast); s->set_saturation(s, S_sat);
  s->set_whitebal(s, 1); s->set_awb_gain(s, 1); s->set_wb_mode(s, S_wb);
  s->set_vflip(s, S_vflip); s->set_hmirror(s, S_hmirror);
  s->set_exposure_ctrl(s, S_aec); if (!S_aec) s->set_aec_value(s, S_aecval);
  s->set_gain_ctrl(s, S_agc); if (!S_agc) s->set_agc_gain(s, S_agcgain);
  s->set_ae_level(s, S_aelevel); s->set_lenc(s, 1); s->set_dcw(s, 1); s->set_bpc(s, 1); s->set_wpc(s, 1); s->set_raw_gma(s, 1);
  applyGain(); motionHold(2500);
}
bool initCamera() {
  camera_config_t c; memset(&c, 0, sizeof c);            // zero-init (v5 left fields random)
  c.ledc_channel = LEDC_CHANNEL_0; c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM; c.pin_d1 = Y3_GPIO_NUM; c.pin_d2 = Y4_GPIO_NUM; c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM; c.pin_d5 = Y7_GPIO_NUM; c.pin_d6 = Y8_GPIO_NUM; c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM; c.pin_pclk = PCLK_GPIO_NUM; c.pin_vsync = VSYNC_GPIO_NUM; c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM; c.pin_sccb_scl = SIOC_GPIO_NUM; c.pin_pwdn = PWDN_GPIO_NUM; c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000; c.pixel_format = PIXFORMAT_JPEG;
  if (psramFound()) {                                    // init at the configured size, not UXGA: saves internal DMA RAM (v6.1)
    int fs = FRAMESIZE_VGA; for (int i = 0; i < SIZES_N; i++) if (SIZES[i].fs == S_fsize) fs = S_fsize;
    if (fs > FRAMESIZE_UXGA) fs = FRAMESIZE_UXGA;
    c.frame_size = (framesize_t)fs; c.jpeg_quality = 12; c.fb_count = 2; c.fb_location = CAMERA_FB_IN_PSRAM; c.grab_mode = CAMERA_GRAB_LATEST;
  } else { c.frame_size = FRAMESIZE_CIF; c.jpeg_quality = 16; c.fb_count = 1; c.fb_location = CAMERA_FB_IN_DRAM; c.grab_mode = CAMERA_GRAB_WHEN_EMPTY; }
  if (esp_camera_init(&c) != ESP_OK) return false;
  applyCamSettings(); return true;
}

bool computeNight() {
  if (S_nmode == 0) return false; if (S_nmode == 2) return true; if (!timeOK()) return false;
  time_t n = time(nullptr); struct tm t; localtime_r(&n, &t); int h = t.tm_hour;
  return S_nstart > S_nend ? (h >= S_nstart || h < S_nend) : (h >= S_nstart && h < S_nend);
}
void updateNight(bool force) {
  bool n = computeNight();
  if (n != nightState || force) { nightState = n; applyGain(); motionHold(3000); logmsg(LVL_INFO, "CAM", "Night mode %s", n ? "ON" : "OFF"); mqttMotion(motionOn, 0); }
}

// ============================== MOTION ENGINE ================================
// JPEG is decoded at 1/8 scale (fast), averaged into a 32x24 luminance grid, compared with a slow baseline.
// Global brightness shifts are cancelled, user zones mask the grid, flash/config changes are held off.
#define MD_RAW_SIZE      64000
#define MD_CELL_DELTA    4
uint8_t* mdRaw = nullptr; uint8_t mdCur[MG_W * MG_H], mdBase[MG_W * MG_H]; bool mdHaveBase = false;
volatile bool mdReset = true; uint8_t zoneOn[ZG_W * ZG_H];

void parseZones() {
  for (int i = 0; i < ZG_W * ZG_H; i++) {
    size_t ci = i / 4; int nib = 15;
    if (ci < strlen(S_mzones)) { char ch = S_mzones[ci]; nib = isdigit((unsigned char)ch) ? ch - '0' : (toupper(ch) - 'A' + 10); if (nib < 0 || nib > 15) nib = 15; }
    zoneOn[i] = (nib >> (3 - (i % 4))) & 1;
  }
}
int motionScore(camera_fb_t* fb, bool adapt) {
  if (!mdRaw) return -1;
  int dw = (fb->width + 7) / 8, dh = (fb->height + 7) / 8; if (dw * dh * 2 > MD_RAW_SIZE || dw < MG_W || dh < MG_H) return -1;
  if (!jpg2rgb565(fb->buf, fb->len, mdRaw, JPG_SCALE_8X)) return -1;
  static uint32_t sum[MG_W * MG_H]; static uint16_t cnt[MG_W * MG_H]; memset(sum, 0, sizeof sum); memset(cnt, 0, sizeof cnt);
  for (int y = 0; y < dh; y++) { int cy = y * MG_H / dh; for (int x = 0; x < dw; x++) {
    int cx = x * MG_W / dw; size_t i = ((size_t)y * dw + x) * 2; uint16_t v = ((uint16_t)mdRaw[i] << 8) | mdRaw[i + 1];
    sum[cy * MG_W + cx] += (v >> 5) & 0x3F; cnt[cy * MG_W + cx]++; } }
  long tot = 0; for (int c = 0; c < MG_W * MG_H; c++) { mdCur[c] = cnt[c] ? sum[c] / cnt[c] : 0; tot += mdCur[c]; }
  if (mdReset || !mdHaveBase) { memcpy(mdBase, mdCur, sizeof mdBase); mdHaveBase = true; mdReset = false; return 0; }
  long totB = 0; for (int c = 0; c < MG_W * MG_H; c++) totB += mdBase[c];
  int shift = (int)((tot - totB) / (MG_W * MG_H)), active = 0, changed = 0;
  for (int cy = 0; cy < MG_H; cy++) for (int cx = 0; cx < MG_W; cx++) {
    if (!zoneOn[(cy / 2) * ZG_W + (cx / 2)]) continue; int c = cy * MG_W + cx; active++;
    int d = (int)mdCur[c] - mdBase[c] - shift; if (d < 0) d = -d; if (d > MD_CELL_DELTA) changed++;
  }
  if (adapt) for (int c = 0; c < MG_W * MG_H; c++) mdBase[c] = (mdBase[c] * 7 + mdCur[c]) / 8;   // baseline frozen during events
  return active ? changed * 100 / active : 0;
}

// ================================ BUFFERS / QUEUES ===========================
struct QFrame { uint8_t* buf; size_t len; uint16_t w, h; uint8_t kind, score; time_t epoch; };
enum { K_VIDEO = 0, K_TL = 1, K_EVENT = 2 };
QueueHandle_t freeQ = nullptr, recQ = nullptr; QFrame framePool[POOL_FRAMES];
uint8_t *latestBuf = nullptr, *sendBuf = nullptr, *snapBuf = nullptr; size_t latestLen = 0, snapLen = 0;
volatile uint32_t latestSeq = 0; SemaphoreHandle_t latestMx = nullptr, snapMx = nullptr; volatile bool snapWanted = false;

bool allocBuffers() {
  latestMx = xSemaphoreCreateMutex(); snapMx = xSemaphoreCreateMutex();
  mdRaw = (uint8_t*)heap_caps_malloc(MD_RAW_SIZE, MALLOC_CAP_SPIRAM); if (!mdRaw) mdRaw = (uint8_t*)malloc(MD_RAW_SIZE);
  latestBuf = (uint8_t*)heap_caps_malloc(LATEST_BUF_SIZE, MALLOC_CAP_SPIRAM); sendBuf = (uint8_t*)heap_caps_malloc(LATEST_BUF_SIZE, MALLOC_CAP_SPIRAM);
  snapBuf = (uint8_t*)heap_caps_malloc(LATEST_BUF_SIZE, MALLOC_CAP_SPIRAM);
  freeQ = xQueueCreate(POOL_FRAMES, sizeof(QFrame)); recQ = xQueueCreate(POOL_FRAMES, sizeof(QFrame));
  recSupported = psramFound();
  if (recSupported) for (int i = 0; i < POOL_FRAMES; i++) {          // no PSRAM = live view only, never an undersized pool (v5 bug C3)
    framePool[i] = QFrame{(uint8_t*)heap_caps_malloc(FRAME_BUF_SIZE, MALLOC_CAP_SPIRAM), 0, 0, 0, 0, 0, 0};
    if (!framePool[i].buf) { recSupported = false; break; } xQueueSend(freeQ, &framePool[i], 0);
  }
  parseZones(); return latestBuf && sendBuf && snapBuf;
}
size_t getSnapshot(uint8_t* out, size_t cap) {
  if (!cameraReady || !snapBuf) return 0;
  snapWanted = true; uint32_t t0 = millis(); while (snapWanted && millis() - t0 < 1500) vTaskDelay(pdMS_TO_TICKS(10));
  if (snapWanted) { snapWanted = false; return 0; }
  if (xSemaphoreTake(snapMx, pdMS_TO_TICKS(500)) != pdTRUE) return 0;
  size_t n = snapLen; if (n > cap) n = 0; else memcpy(out, snapBuf, n); xSemaphoreGive(snapMx); return n;
}

static bool enqueueFrame(camera_fb_t* fb, uint8_t kind, uint8_t score) {
  if (!recSupported) return false;
  if (fb->len > FRAME_BUF_SIZE) { oversizeDropped += 1; return false; }   // skipped, never truncated (v5 bug C6)
  QFrame q; if (xQueueReceive(freeQ, &q, 0) != pdTRUE) { framesDropped += 1; return false; }
  memcpy(q.buf, fb->buf, fb->len); q.len = fb->len; q.w = fb->width; q.h = fb->height; q.kind = kind; q.score = score; q.epoch = time(nullptr);
  if (xQueueSend(recQ, &q, 0) != pdTRUE) { xQueueSend(freeQ, &q, 0); framesDropped += 1; return false; }
  return true;
}

// ================================ CAPTURE TASK ===============================
static void camRecover() {
  static int reinitFails = 0; camRecoveries += 1;
  logmsg(LVL_ERROR, "CAM", "No frames - reinitialising camera (%d)", reinitFails + 1);
  cameraReady = false; esp_camera_deinit(); vTaskDelay(pdMS_TO_TICKS(300));
  cameraReady = initCamera();
  if (cameraReady) { reinitFails = 0; mdReset = true; return; }
  if (++reinitFails >= 3) { logmsg(LVL_ERROR, "CAM", "Camera dead - rebooting"); requestReboot(); }
}
static void endMotion(uint32_t now) {
  motionOn = false; mqttMotion(false, 0);
  if (flashAuto) { flashAuto = false; setFlashDuty(flashBrightness); }
  logmsg(LVL_INFO, "MOTION", "Event ended (%lu s)", (unsigned long)((now - motionSince) / 1000));
}
static void registerMotion(camera_fb_t* fb, int score, bool pir, uint32_t now) {
  motionLastSeen = now; motionScoreLive = score > 100 ? 100 : score;
  if (motionOn) return;
  motionOn = true; motionSince = now; motionEpoch = time(nullptr); motionEvents += 1;
  logmsg(LVL_WARN, "MOTION", "%s detected (%d%%)", pir ? "PIR" : "Motion", score);
  enqueueFrame(fb, K_EVENT, (uint8_t)(score < 0 ? 0 : score > 100 ? 100 : score));
  if (S_nflash && nightState) { flashAuto = true; setFlashDuty(S_nduty); }
  notifyMotion(fb, score); mqttMotion(true, score);
}
void captureTask(void*) {
  esp_task_wdt_add(NULL);
  uint32_t lastCap = 0, lastRec = 0, lastMotion = 0, lastTl = 0, fpsT = millis(), fpsN = 0; int nofb = 0, hitStreak = 0, lastScore = 0; bool tlFirst = true;
  for (;;) {
    esp_task_wdt_reset(); uint32_t now = millis();
    if (otaBusy) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
    if (!cameraReady) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }
    if (now - lastCap < 1000 / CAPTURE_FPS) { vTaskDelay(pdMS_TO_TICKS(2)); continue; }
    lastCap = now;
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { if (++nofb >= 20) { nofb = 0; camRecover(); } vTaskDelay(pdMS_TO_TICKS(20)); continue; }   // frozen sensor watchdog (v5 bug C12)
    nofb = 0; fpsN++; if (now - fpsT >= 1000) { currentFPS = fpsN * 1000.0f / (now - fpsT); fpsN = 0; fpsT = now; }

    if (viewerCount > 0 && fb->len <= LATEST_BUF_SIZE && xSemaphoreTake(latestMx, 0) == pdTRUE) {
      memcpy(latestBuf, fb->buf, fb->len); latestLen = fb->len; latestSeq += 1; xSemaphoreGive(latestMx);
    }
    if (snapWanted) {
      if (fb->len <= LATEST_BUF_SIZE && xSemaphoreTake(snapMx, pdMS_TO_TICKS(50)) == pdTRUE) { memcpy(snapBuf, fb->buf, fb->len); snapLen = fb->len; xSemaphoreGive(snapMx); }
      else snapLen = 0;
      snapWanted = false;
    }

    // ---- motion (5 Hz) ----
    if (S_armed && (S_motion || S_pir) && now - lastMotion >= 500) {   // 2 Hz scoring: 1/3 the JPEG-decode churn inside IRAM (v6.1)
      lastMotion = now; bool pirHit = S_pir && digitalRead(PIR_PIN) == HIGH; int score = -1;
      bool holdoff = inHoldoff(now);
      if (holdoff) { mdReset = true; hitStreak = 0; }
      else if (S_motion) { score = motionScore(fb, lastScore < S_mthr); if (score >= 0) { lastScore = score; motionScoreLive = score > 100 ? 100 : score; } }
      bool hit = pirHit || (score >= S_mthr);
      if (hit) { hitStreak++; if (hitStreak >= S_mframes || pirHit) registerMotion(fb, score < 0 ? 0 : score, pirHit && score < S_mthr, now); }
      else hitStreak = 0;
    }
    if (motionOn && reached(now, motionLastSeen + (uint32_t)S_postroll * 1000UL)) endMotion(now);
    if (!S_armed && motionOn) endMotion(now);

    // ---- feed the recorder ----
    if (recSupported && sdCardMounted && !recPaused) {
      if ((S_recmode == 1 || S_recmode == 2) && now - lastRec >= 1000UL / S_recfps) { lastRec = now; enqueueFrame(fb, K_VIDEO, 0); }
      if (S_tl && (tlFirst || now - lastTl >= (uint32_t)S_tlint * 1000UL)) { tlFirst = false; lastTl = now; enqueueFrame(fb, K_TL, 0); }
    }
    esp_camera_fb_return(fb); vTaskDelay(1);
  }
}

// ============================ MEDIA SERVER (PORT 81) =========================
WiFiServer mediaServer(81); WiFiClient viewers[MAX_VIEWERS]; SemaphoreHandle_t viewMx = nullptr;
struct FileJob { WiFiClient* c; char path[16]; char query[192]; }; QueueHandle_t fileQ = nullptr;

static bool readLine(WiFiClient& c, char* buf, size_t n, uint32_t to) {
  size_t i = 0; uint32_t t0 = millis();
  while (millis() - t0 < to) {
    if (!c.connected() && !c.available()) return false;
    if (c.available()) { char ch = c.read(); if (ch == '\n') { buf[i] = 0; return true; } if (ch != '\r' && i < n - 1) buf[i++] = ch; }
    else vTaskDelay(1);
  }
  return false;
}
static bool qarg(const char* q, const char* key, char* out, size_t n) {
  size_t kl = strlen(key); const char* p = q;
  while (p && *p) {
    if (!strncmp(p, key, kl) && p[kl] == '=') {
      p += kl + 1; size_t i = 0;
      while (*p && *p != '&' && i < n - 1) { if (*p == '%' && p[1] && p[2]) { char h[3] = {p[1], p[2], 0}; out[i++] = (char)strtol(h, 0, 16); p += 3; } else if (*p == '+') { out[i++] = ' '; p++; } else out[i++] = *p++; }
      out[i] = 0; return true;
    }
    p = strchr(p, '&'); if (p) p++;
  }
  return false;
}
static void httpSimple(WiFiClient& c, int code, const char* msg) {
  char b[128]; int n = snprintf(b, sizeof b, "HTTP/1.1 %d %s\r\nContent-Length: 0\r\nConnection: close\r\n\r\n", code, msg); c.write((const uint8_t*)b, n);
}
static void setSndTimeout(WiFiClient& c, int ms) { int fd = c.fd(); if (fd >= 0) { struct timeval to = {ms / 1000, (ms % 1000) * 1000}; setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &to, sizeof to); } }
static bool sendAll(WiFiClient& c, const uint8_t* d, size_t n) {
  size_t sent = 0; uint32_t last = millis();
  while (sent < n) { size_t w = c.write(d + sent, min((size_t)4096, n - sent)); if (w) { sent += w; last = millis(); } else if (millis() - last > 4000 || !c.connected()) return false; else vTaskDelay(1); }
  return true;
}

void netMediaTask(void*) {
  for (;;) {
    WiFiClient c =
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
      mediaServer.accept();
#else
      mediaServer.available();
#endif
    if (!c) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }
    char line[256], method[8], target[224], cookie[160] = "";
    if (!readLine(c, line, sizeof line, 800) || sscanf(line, "%7s %223s", method, target) != 2) { c.stop(); continue; }
    for (int i = 0; i < 30; i++) { char h[200]; if (!readLine(c, h, sizeof h, 500) || !h[0]) break; if (!strncasecmp(h, "Cookie:", 7)) { const char* v = h + 7; while (*v == ' ') v++; strlcpy(cookie, v, sizeof cookie); } }
    if (roleFromCookie(cookie) < ROLE_VIEWER) { httpSimple(c, 401, "Unauthorized"); c.stop(); continue; }   // stream is authenticated (v5 bug C13)
    char* q = strchr(target, '?'); const char* query = ""; if (q) { *q = 0; query = q + 1; }
    if (!strcmp(target, "/stream")) {
      c.setNoDelay(true); setSndTimeout(c, 200);
      c.print("HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace; boundary=frame\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n");
      bool placed = false; xSemaphoreTake(viewMx, portMAX_DELAY);
      for (int i = 0; i < MAX_VIEWERS && !placed; i++) if (!viewers[i].connected()) { viewers[i] = c; placed = true; }
      xSemaphoreGive(viewMx);
      if (!placed) { c.print("--frame\r\n\r\n"); c.stop(); }
    } else if (!strcmp(target, "/dl") || !strcmp(target, "/frame") || !strcmp(target, "/thumb")) {
      FileJob j; j.c = new WiFiClient(c); strlcpy(j.path, target, sizeof j.path); strlcpy(j.query, query, sizeof j.query);
      if (xQueueSend(fileQ, &j, 0) != pdTRUE) { httpSimple(*j.c, 503, "Busy"); j.c->stop(); delete j.c; }
    } else { httpSimple(c, 404, "Not Found"); c.stop(); }
  }
}
void streamTask(void*) {
  uint32_t lastSeq = 0, lastSend = 0;
  for (;;) {
    int n = 0; xSemaphoreTake(viewMx, portMAX_DELAY);
    for (int i = 0; i < MAX_VIEWERS; i++) { if (viewers[i].connected()) n++; else viewers[i].stop(); }
    xSemaphoreGive(viewMx); viewerCount = n;
    if (!n) { vTaskDelay(pdMS_TO_TICKS(100)); continue; }
    if (!reached(millis(), lastSend + 1000UL / S_strfps) || latestSeq == lastSeq) { vTaskDelay(pdMS_TO_TICKS(5)); continue; }
    size_t len = 0;
    if (xSemaphoreTake(latestMx, pdMS_TO_TICKS(50)) == pdTRUE) { len = latestLen; if (len) memcpy(sendBuf, latestBuf, len); lastSeq = latestSeq; xSemaphoreGive(latestMx); }
    if (!len) continue; lastSend = millis();
    char hb[96]; int hl = snprintf(hb, sizeof hb, "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", (unsigned)len);
    xSemaphoreTake(viewMx, portMAX_DELAY);
    for (int i = 0; i < MAX_VIEWERS; i++) {                // a partial write would desync the multipart stream: drop that viewer
      if (!viewers[i].connected()) continue;
      if (viewers[i].write((const uint8_t*)hb, hl) != (size_t)hl || viewers[i].write(sendBuf, len) != len || viewers[i].write((const uint8_t*)"\r\n", 2) != 2) viewers[i].stop();
    }
    xSemaphoreGive(viewMx);
  }
}

// ---- file workers: downloads, thumbnails and frame-accurate playback (never block the web UI on :80) ----
static void serveJob(WiFiClient& c, FileJob& j, uint8_t* buf) {
  char file[48] = "", nb[12] = ""; setSndTimeout(c, 3000);
  if (!qarg(j.query, "file", file, sizeof file) || !validMediaPath(file)) { httpSimple(c, 400, "Bad file"); return; }
  if (!sdCardMounted) { httpSimple(c, 404, "No SD"); return; }
  char hb[220];
  if (!strcmp(j.path, "/frame")) {
    qarg(j.query, "n", nb, sizeof nb); uint32_t n = strtoul(nb, 0, 10); size_t len = 0; bool ok = false;
    if (sdLock(2000)) { File f = SD_MMC.open(file, FILE_READ); if (f) { AviInfo in; if (aviParse(f, in)) ok = aviGetFrame(f, in, n, buf, FRAME_BUF_SIZE, len); f.close(); } sdUnlock(); }
    if (!ok) { httpSimple(c, 404, "No frame"); return; }
    int hl = snprintf(hb, sizeof hb, "HTTP/1.1 200 OK\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\nCache-Control: no-store\r\nConnection: close\r\n\r\n", (unsigned)len);
    c.write((const uint8_t*)hb, hl); sendAll(c, buf, len); return;
  }
  bool isJpg = !strcasecmp(file + strlen(file) - 4, ".jpg"); uint32_t size = 0;
  if (!sdLock(2000)) { httpSimple(c, 503, "SD busy"); return; }
  File f = SD_MMC.open(file, FILE_READ); if (!f) { sdUnlock(); httpSimple(c, 404, "Not found"); return; } size = f.size(); sdUnlock();
  int hl = snprintf(hb, sizeof hb, "HTTP/1.1 200 OK\r\nContent-Type: %s\r\nContent-Length: %u\r\n%s%s%sCache-Control: private, max-age=3600\r\nConnection: close\r\n\r\n",
                    isJpg ? "image/jpeg" : "video/x-msvideo", (unsigned)size, isJpg ? "" : "Content-Disposition: attachment; filename=\"", isJpg ? "" : baseName(file), isJpg ? "" : "\"\r\n");
  c.write((const uint8_t*)hb, hl);
  uint32_t pos = 0;
  while (pos < size && c.connected()) {                     // short lock per 16 KB chunk: recorder keeps writing meanwhile
    size_t got = 0; if (!sdLock(1500)) break;
    f.seek(pos); got = f.read(buf, min((uint32_t)FILE_CHUNK, size - pos)); sdUnlock();
    if (!got || !sendAll(c, buf, got)) break; pos += got;
  }
  if (sdLock(1500)) { f.close(); sdUnlock(); }
}
void fileTask(void*) {
  uint8_t* buf = (uint8_t*)heap_caps_malloc(FRAME_BUF_SIZE, MALLOC_CAP_SPIRAM); if (!buf) buf = (uint8_t*)malloc(FILE_CHUNK);
  for (;;) { FileJob j; if (xQueueReceive(fileQ, &j, portMAX_DELAY) == pdTRUE) { serveJob(*j.c, j, buf); j.c->stop(); delete j.c; } }
}

// ================================ RECORDER TASK ==============================
struct PreFrame { uint8_t* buf; size_t len; uint16_t w, h; };
PreFrame pre[PREROLL_MAX_FRAMES]; int preHead = 0, preCount = 0; size_t preBytes = 0; time_t motionClipT = 0; int sdErrStreak = 0;
char tlDay[10] = "";

static void prePop() { free(pre[preHead].buf); preBytes -= pre[preHead].len; preHead = (preHead + 1) % PREROLL_MAX_FRAMES; preCount--; }
static void preClear() { while (preCount) prePop(); }
static void prePush(const QFrame& q) {
  if (S_preroll == 0) return; int maxF = min(PREROLL_MAX_FRAMES, (int)(S_preroll * S_recfps + 1));
  while (preCount >= maxF || (preCount && preBytes + q.len > PREROLL_BYTES)) prePop();
  uint8_t* b = (uint8_t*)heap_caps_malloc(q.len, MALLOC_CAP_SPIRAM); if (!b) return; memcpy(b, q.buf, q.len);
  pre[(preHead + preCount) % PREROLL_MAX_FRAMES] = PreFrame{b, q.len, q.w, q.h}; preCount++; preBytes += q.len;
}
static void sdFault();
static uint32_t opeLastTry = 0, opeLastLog = 0;         // throttle failed clip-open: 2 s retry gate, 10 s log gate (v6.1)
static bool openWriter(AviWriter& w, const char* prefix, time_t t, int fw, int fh, int fps, const char* label, bool patch) {
  if (ESP.getFreeHeap() < MEM_LOW || memDmaFree() < DMA_FLOOR) { // never churn SD while heap/DMA is starving (v6.1) - no SD ops can succeed 0x101
    if (millis() - recLowLogMs > 10000) { recLowLogMs = millis(); logmsg(LVL_ERROR, "REC", "Low memory - recorder idle"); }
    return false;
  }
  uint32_t now = millis(); if (now - opeLastTry < 2000) return false; opeLastTry = now;   // no 8x/sec retry storms on a failing card (v6.1)
  char path[64]; if (!buildPath(path, sizeof path, prefix, "avi", t)) { if (millis() - opeLastLog > 10000) { opeLastLog = millis(); logmsg(LVL_ERROR, "REC", "Cannot create folder for %s", prefix); } if (++sdErrStreak >= 5) sdFault(); return false; }
  char ds[24], nm[48]; tsHumanBuf(ds, sizeof ds); snprintf(nm, sizeof nm, "%s%s", S_camname, label);
  if (!w.begin(path, fw, fh, fps, nm, ds, patch)) { logmsg(LVL_ERROR, "REC", "Cannot open %s", path); if (++sdErrStreak >= 5) sdFault(); return false; }
  w.w = fw; w.h = fh; strlcpy(recFileNow, path, sizeof recFileNow); logmsg(LVL_INFO, "REC", "Opened %s (%dx%d)", path, fw, fh); return true;
}
static void closeWriter(AviWriter& w) {
  if (!w.isOpen()) return; uint32_t n = w.frames(); char p[72]; strlcpy(p, w.path(), sizeof p); w.finalize();
  logmsg(LVL_INFO, "REC", "Closed %s (%lu frames)", p, (unsigned long)n); if (!contWriter.isOpen() && !motionWriter.isOpen()) recFileNow[0] = 0;
}
static void closeAllWriters() { closeWriter(contWriter); closeWriter(motionWriter); closeWriter(tlWriter); preClear(); }
static int writeFrame(AviWriter& w, const uint8_t* b, size_t len) {
  uint32_t t0 = millis(); int r = w.addFrame(b, len); uint32_t dt = millis() - t0;
  sdWriteMsAvg = (sdWriteMsAvg * 7 + dt) / 8; if (dt > sdWriteMsMax) sdWriteMsMax = dt;
  if (r < 0) sdErrors += 1; else framesWritten += 1; return r;
}
static uint32_t chunkIdNow(time_t epoch) {
  uint32_t per = (uint32_t)S_chunk * 60; return timeOK() ? (uint32_t)((epoch + (time_t)S_tzmin * 60) / per) : millis() / (per * 1000UL);
}
static uint32_t lastRemount = 0;
static void sdFault() {                                    // needs 5 failures; a full card is not a fault; remounts rate-limited
  if (++sdErrStreak < 5) return;
  sdErrStreak = 0; closeAllWriters(); sdRefreshInfo(true);
  if (sdFreeB() < minFreeBytes()) { sdPaused = false; logmsg(LVL_WARN, "SD", "Write failed, card nearly full - freeing space"); return; }
  if (millis() - lastRemount < 60000) { sdPaused = true; return; }              // whole 60 s window silent: no SD writes at all (v6.1)
  lastRemount = millis(); sdPaused = true; logmsg(LVL_ERROR, "SD", "Repeated write errors - remounting card"); sdState = 2; sdCardMounted = false;
  notifySystem("SD card error - attempting recovery");
}

static void processFrame(QFrame& q) {
  if (q.kind == K_EVENT) {
    char p[64]; EventRec e; memset(&e, 0, sizeof e); e.ts = q.epoch; e.score = q.score; e.pir = false;
    if (buildPath(p, sizeof p, "ev", "jpg", q.epoch)) {
      File f = SD_MMC.open(p, FILE_WRITE); if (f) { f.write(q.buf, q.len); f.close(); strlcpy(e.thumb, p, sizeof e.thumb); }
    }
    if (S_recmode == 2) { char d[10], h[10]; dayStr(d, sizeof d, q.epoch); hmsStr(h, sizeof h, q.epoch); snprintf(e.clip, sizeof e.clip, "/%s/mot_%s.avi", d, h); motionClipT = q.epoch; }
    else if (contWriter.isOpen()) strlcpy(e.clip, contWriter.path(), sizeof e.clip);
    eventPush(e); eventAppendFile(e); return;
  }
  if (q.kind == K_TL) {
    if (!S_tl) return; char d[10]; dayStr(d, sizeof d, q.epoch);
    if (tlWriter.isOpen() && (strcmp(d, tlDay) || tlWriter.w != q.w || tlWriter.h != q.h)) closeWriter(tlWriter);
    if (!tlWriter.isOpen()) { if (!openWriter(tlWriter, "tl", q.epoch, q.w, q.h, 30, " Time-Lapse", false)) return; strlcpy(tlDay, d, sizeof tlDay); }
    int r = writeFrame(tlWriter, q.buf, q.len); if (r != 0) { closeWriter(tlWriter); if (r < 0) sdFault(); }
    return;
  }
  if (S_recmode == 1) {                                     // ---- continuous: wall-clock aligned chunks ----
    uint32_t cid = chunkIdNow(q.epoch);
    if (contWriter.isOpen() && (contWriter.chunkId != cid || contWriter.w != q.w || contWriter.h != q.h)) closeWriter(contWriter);
    if (!contWriter.isOpen()) { if (!openWriter(contWriter, "rec", q.epoch, q.w, q.h, S_recfps, "", true)) return; contWriter.chunkId = cid; }
    int r = writeFrame(contWriter, q.buf, q.len); if (r == 0) sdErrStreak = 0; else { closeWriter(contWriter); if (r < 0) sdFault(); }
  } else if (S_recmode == 2) {                              // ---- motion-only with pre-roll / post-roll ----
    bool active = motionOn;
    if (!motionWriter.isOpen()) {
      if (!active) { prePush(q); return; }
      if (!openWriter(motionWriter, "mot", motionClipT ? motionClipT : q.epoch, q.w, q.h, S_recfps, " Motion", true)) return;
      motionClipT = 0;
      for (int i = 0; i < preCount; i++) { PreFrame& pf = pre[(preHead + i) % PREROLL_MAX_FRAMES]; if (pf.w == q.w && pf.h == q.h) writeFrame(motionWriter, pf.buf, pf.len); }
      preClear();
    }
    int r = writeFrame(motionWriter, q.buf, q.len); if (r == 0) sdErrStreak = 0;
    if (r != 0 || !active || motionWriter.ageMs() >= (uint32_t)S_maxclip * 1000UL || motionWriter.w != q.w) { closeWriter(motionWriter); if (r < 0) sdFault(); }
  }
}
void recordTask(void*) {
  esp_task_wdt_add(NULL); QFrame q; bool paused = false;
  for (;;) {
    esp_task_wdt_reset();
    bool got = xQueueReceive(recQ, &q, pdMS_TO_TICKS(500)) == pdTRUE;
    if (got) {
      if (sdCardMounted && !recPaused && !sdPaused && sdLock(2000)) { processFrame(q); sdUnlock(); } else framesDropped += 1;
      xQueueSend(freeQ, &q, 0);
    }
    if (rebootRequested) {                                  // close files cleanly, then restart
      if (sdLock(4000)) { closeAllWriters(); sdUnlock(); }
      vTaskDelay(pdMS_TO_TICKS(600)); ESP.restart();
    }
    bool wantPause = recPaused || otaBusy;
    if ((wantPause && !paused) || (!got && sdCardMounted)) {  // idle housekeeping: mode/setting changes, pauses
      if (sdLock(1000)) {
        if (wantPause || S_recmode != 1) closeWriter(contWriter);
        if (wantPause || S_recmode != 2) { closeWriter(motionWriter); if (S_recmode != 2) preClear(); }
        if (wantPause || !S_tl) closeWriter(tlWriter);
        sdUnlock();
      }
    }
    paused = wantPause;
  }
}

// ============================= MAINTENANCE TASK ==============================
void maintTask(void*) {
  uint32_t lastRet = 0, lastMount = 0, lastInfo = 0, lastSweep = 0, lastScan = 0; uint32_t mountBackoff = 10000; int absentStreak = 0;
  bool booted = false, lowNotified = false;
  vTaskDelay(pdMS_TO_TICKS(15000));                 // let the camera + WiFi settle and the card warm up before first probe (v6.1)
  for (;;) {
    uint32_t now = millis();
    if (!sdCardMounted) {
      if (now - lastMount > mountBackoff && ESP.getFreeHeap() >= MEM_LOW && memDmaFree() >= DMA_FLOOR) {   // don't probe SD while heap/DMA recovers (v6.1)
        lastMount = now;
        if (sdLock(4000)) {
          SD_MMC.end(); bool ok = sdMount(); sdUnlock();
          if (ok) { sdCardMounted = true; sdState = 0; mountBackoff = 10000; absentStreak = 0; sdPaused = false; booted = false; sdInfoMs = 0; logmsg(LVL_INFO, "SD", "Card (re)mounted"); notifySystem("SD card recovered"); }
          else { sdState = 3; if (++absentStreak >= 3) mountBackoff = 300000; else mountBackoff = mountBackoff < 120000 ? mountBackoff * 2 : 120000; }   // after 3 no-card rounds, probe only every 5 min (v6.1)
        }
      }
    } else {
      if (ESP.getFreeHeap() >= MEM_LOW && memDmaFree() >= DMA_FLOOR) {                                     // skip sweeps/scans while heap/DMA recovers
        if (!booted) { booted = true; sdRefreshInfo(true); lastInfo = now; bootRepair(); eventsLoadTail(); }   // slow jobs live here, never in setup()
        if (!sdInfoMs || now - lastInfo > 120000) { lastInfo = now; sdRefreshInfo(true); }
        if (sdTotalB && sdFreeB() < minFreeBytes() && now - lastSweep > 15000) { lastSweep = now; spaceSweep(); }
        if (!lastRet || now - lastRet > 600000UL) { lastRet = now; retentionSweep(); }
        if (storScanReq && now - lastScan > 10000) { storScanReq = false; lastScan = now; storStatsMs = 0; storageScan(); }
      }
      bool low = sdTotalB && sdFreeB() < minFreeBytes() / 2;
      if (low && !lowNotified) { lowNotified = true; notifySystem("SD card almost full"); } else if (!low) lowNotified = false;
    }
    vTaskDelay(pdMS_TO_TICKS(3000));
  }
}

// ================================ CALLBACKS ==================================
void cbCam()    { applyCamSettings(); }
void cbNight()  { updateNight(true); }
void cbTime()   { applyTimezone(); if (WiFi.status() == WL_CONNECTED) wifiTimeStart(); }
void cbNet()    { wifiApplyTx(); }
void cbMotion() { if (S_pir) pinMode(PIR_PIN, INPUT_PULLDOWN); parseZones(); mdReset = true; motionHold(1500); }
