#pragma once
#include "storage.h"
#if __has_include(<PubSubClient.h>)
  #include <PubSubClient.h>
  #define HAVE_MQTT 1
#endif

size_t getSnapshot(uint8_t* out, size_t cap);   // camera_tasks.h

// ================================= LOG WRITER =================================
// Single writer: logging never blocks the recorder or the web server.
void logTask(void*) {
  LogLine ln; String batch; uint32_t firstAt = 0;
  for (;;) {
    if (xQueueReceive(logQ, &ln, pdMS_TO_TICKS(1000)) == pdTRUE) { if (batch.isEmpty()) firstAt = millis(); batch += ln.s; batch += '\n'; }
    if (batch.isEmpty()) continue;
    if (batch.length() < 2000 && millis() - firstAt < 10000) continue;
    if (!sdCardMounted || memDmaFree() < DMA_FLOOR) { if (batch.length() > 3000) batch = ""; continue; }   // SD file log also needs DMA (v6.1)
    if (sdLock(500)) {
      File f = SD_MMC.open(LOG_FILE, FILE_APPEND);
      if (f) {
        if (f.size() > 128 * 1024) { f.close(); SD_MMC.remove(LOG_FILE_OLD); SD_MMC.rename(LOG_FILE, LOG_FILE_OLD); f = SD_MMC.open(LOG_FILE, FILE_APPEND); }
        if (f) { f.write((const uint8_t*)batch.c_str(), batch.length()); f.close(); }
      }
      sdUnlock(); batch = "";
    } else if (batch.length() > 3000) batch = "";
  }
}

// ================================== FLASH LED =================================
uint32_t flashPulseUntil = 0;
void initFlashPWM() {
  ledc_timer_config_t t = {}; t.speed_mode = FLASH_LEDC_MODE; t.timer_num = FLASH_LEDC_TIMER;
  t.duty_resolution = LEDC_TIMER_8_BIT; t.freq_hz = 5000; t.clk_cfg = LEDC_AUTO_CLK; ledc_timer_config(&t);
  ledc_channel_config_t c = {}; c.gpio_num = FLASH_LED_PIN; c.speed_mode = FLASH_LEDC_MODE; c.channel = FLASH_LEDC_CHANNEL;
  c.timer_sel = FLASH_LEDC_TIMER; c.duty = 0; c.hpoint = 0; ledc_channel_config(&c);
}
void setFlashDuty(int d) {
  d = constrain(d, 0, FLASH_MAX_DUTY); ledc_set_duty(FLASH_LEDC_MODE, FLASH_LEDC_CHANNEL, d); ledc_update_duty(FLASH_LEDC_MODE, FLASH_LEDC_CHANNEL);
  motionHold(1500);    // brightness jumps must not look like motion
}

// ============================== WIFI MANAGER =================================
struct WNet { char ssid[33]; char pass[65]; };
WNet wnets[3]; int wnetCount = 0;
enum WState { W_IDLE, W_TRY, W_UP, W_LOST };
WState wstate = W_IDLE; int wtry = 0; uint32_t wtryStart = 0, connectedSince = 0, lostSince = 0;
bool apUp = false, wifiEverUp = false, tzSynced = false; DNSServer dnsSrv;
char apSsid[24], apPass[16], curSsid[33] = "", joinMsg[64] = "";
bool otaStarted = false; uint32_t wifiScanHoldUntil = 0;

void wifiLoad() {
  Preferences p; p.begin("wifi", false); wnetCount = 0;
  for (int i = 0; i < 3; i++) {
    char ks[4] = {'s', (char)('0' + i), 0, 0}, kp[4] = {'p', (char)('0' + i), 0, 0};
    String s = p.getString(ks, ""); if (!s.length()) continue;
    strlcpy(wnets[wnetCount].ssid, s.c_str(), 33); strlcpy(wnets[wnetCount].pass, p.getString(kp, "").c_str(), 65); wnetCount++;
  }
  p.end();
}
void wifiSaveAll() {
  Preferences p; p.begin("wifi", false); p.clear();
  for (int i = 0; i < wnetCount; i++) {
    char ks[4] = {'s', (char)('0' + i), 0, 0}, kp[4] = {'p', (char)('0' + i), 0, 0};
    p.putString(ks, wnets[i].ssid); p.putString(kp, wnets[i].pass);
  }
  p.end();
}
void wifiAdd(const char* ssid, const char* pass) {      // most recent network goes first
  int at = -1; for (int i = 0; i < wnetCount; i++) if (!strcmp(wnets[i].ssid, ssid)) at = i;
  if (at < 0) { if (wnetCount < 3) wnetCount++; at = wnetCount - 1; }
  for (int i = at; i > 0; i--) wnets[i] = wnets[i - 1];
  strlcpy(wnets[0].ssid, ssid, 33); strlcpy(wnets[0].pass, pass, 65); wifiSaveAll();
}
void wifiForget(const char* ssid) {
  for (int i = 0; i < wnetCount; i++) if (!strcmp(wnets[i].ssid, ssid)) { for (int k = i; k < wnetCount - 1; k++) wnets[k] = wnets[k + 1]; wnetCount--; break; }
  wifiSaveAll();
}
void wifiApplyTx() {
  wifi_power_t p = S_txpow >= 19 ? WIFI_POWER_19_5dBm : S_txpow >= 17 ? WIFI_POWER_17dBm : S_txpow >= 15 ? WIFI_POWER_15dBm :
                   S_txpow >= 13 ? WIFI_POWER_13dBm : S_txpow >= 11 ? WIFI_POWER_11dBm : WIFI_POWER_8_5dBm;
  WiFi.setTxPower(p);
}
void apStart() {
  if (apUp) return;
  bool haveSta = wnetCount > 0 || WiFi.status() == WL_CONNECTED;   // pure-AP if no STA network exists (v6.1 mem)
  WiFi.mode(haveSta ? WIFI_AP_STA : WIFI_AP);
  int ch = (WiFi.status() == WL_CONNECTED) ? WiFi.channel() : 1;
  WiFi.softAP(apSsid, apPass, ch > 0 ? ch : 1, 0, 2);   // max 2 clients: fewer per-station CCMP/AMPDU buffers in IRAM (v6.1)
  dnsSrv.start(53, "*", WiFi.softAPIP());                 // captive portal
  apUp = true; logmsg(LVL_INFO, "WIFI", "Setup hotspot '%s' on %s", apSsid, WiFi.softAPIP().toString().c_str());
}
void apStop() { if (!apUp) return; dnsSrv.stop(); WiFi.softAPdisconnect(true); WiFi.mode(WIFI_STA); apUp = false; logmsg(LVL_INFO, "WIFI", "Hotspot stopped"); }

void wifiBeginTry(int i) {
  wtry = i; wtryStart = millis(); wstate = W_TRY;
  IPAddress ip, gw, sn, dn;
  if (S_static && ip.fromString(S_sip) && gw.fromString(S_sgw) && sn.fromString(S_ssn)) { dn.fromString(S_sdns); WiFi.config(ip, gw, sn, dn); }
  else WiFi.config(IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0), IPAddress(0, 0, 0, 0));
  WiFi.begin(wnets[i].ssid, wnets[i].pass);
  snprintf(joinMsg, sizeof joinMsg, "Connecting to %s...", wnets[i].ssid);
  logmsg(LVL_INFO, "WIFI", "Trying %s", wnets[i].ssid);
}
void wifiTimeStart() {
  applyTimezone(); configTzTime(getenv("TZ"), S_ntp[0] ? S_ntp : "pool.ntp.org", "time.nist.gov");
}
void wifiOnUp() {
  wstate = W_UP; wifiEverUp = true; connectedSince = millis(); strlcpy(curSsid, WiFi.SSID().c_str(), sizeof curSsid);
  WiFi.setSleep(false); esp_wifi_set_ps(WIFI_PS_NONE); wifiApplyTx();
  snprintf(joinMsg, sizeof joinMsg, "Connected: %s", WiFi.localIP().toString().c_str());
  wifiTimeStart(); MDNS.begin(HOSTNAME); MDNS.addService("http", "tcp", 80);
  if (!otaStarted) {
    ArduinoOTA.setHostname(HOSTNAME); ArduinoOTA.setPassword(apPass);
    ArduinoOTA.onStart([]() { otaBusy = true; logmsg(LVL_WARN, "OTA", "Update started"); });
    ArduinoOTA.onEnd([]() { logmsg(LVL_WARN, "OTA", "Update finished"); });
    ArduinoOTA.onError([](ota_error_t) { otaBusy = false; });
    ArduinoOTA.begin(); otaStarted = true;
  }
  logmsg(LVL_INFO, "WIFI", "Connected to %s, IP %s (http://%s.local)", curSsid, WiFi.localIP().toString().c_str(), HOSTNAME);
}
void wifiStart() {
  uint8_t mac[6]; WiFi.macAddress(mac);
  snprintf(apSsid, sizeof apSsid, "ESP32CAM-%02X%02X", mac[4], mac[5]);
  snprintf(apPass, sizeof apPass, "cctv-%02X%02X%02X", mac[3], mac[4], mac[5]);
  WiFi.persistent(false); WiFi.setAutoReconnect(true); WiFi.setHostname(HOSTNAME);
  WiFi.mode(WIFI_STA); wifiLoad();
  if (wnetCount) wifiBeginTry(0); else apStart();
}
bool wifiScanStart() {
  wifiScanHoldUntil = millis() | 1; wifiScanHoldUntil = millis() + 20000; if (!wifiScanHoldUntil) wifiScanHoldUntil = 1;
  if (WiFi.status() != WL_CONNECTED) { WiFi.disconnect(false, false); delay(120); }   // an in-progress connect attempt aborts scans
  WiFi.scanDelete();
  return WiFi.scanNetworks(true, true) != WIFI_SCAN_FAILED;
}
void wifiService() {
  uint32_t now = millis();
  if (apUp) dnsSrv.processNextRequest();
  static bool wasHeld = false; bool held = wifiScanHoldUntil && !reached(now, wifiScanHoldUntil);
  if (!held) wifiScanHoldUntil = 0;
  if (held) { wasHeld = true; return; }
  if (wasHeld) { wasHeld = false; if (wstate == W_TRY && wnetCount) wifiBeginTry(wtry); }
  bool up = WiFi.status() == WL_CONNECTED;
  switch (wstate) {
    case W_TRY:
      if (up) { wifiOnUp(); }
      else if (now - wtryStart > 20000) {
        if (wtry + 1 < wnetCount) wifiBeginTry(wtry + 1);
        else { snprintf(joinMsg, sizeof joinMsg, "Could not connect. Check password / 2.4 GHz."); logmsg(LVL_WARN, "WIFI", "All networks failed - hotspot on"); apStart(); wstate = W_LOST; lostSince = now; }
      }
      break;
    case W_UP:
      if (!up) { wstate = W_LOST; lostSince = now; logmsg(LVL_WARN, "WIFI", "Link lost"); }
      else {
        if (apUp && now - connectedSince > 60000) apStop();           // hotspot is a fallback, not permanent
        if (!tzSynced && now - connectedSince > 30000) { tzSynced = true; timeApprox = false; }
        static uint32_t lastRssi = 0; if (now - lastRssi > 30000) { lastRssi = now; rssiRing[rssiHead] = WiFi.RSSI(); rssiHead = (rssiHead + 1) % RSSI_RING; }
      }
      break;
    case W_LOST:
      if (up) { logmsg(LVL_INFO, "WIFI", "Link restored"); wifiOnUp(); if (S_tgsys) notifySystem("WiFi restored"); }
      else if (wnetCount && now - lostSince > 45000) {               // never reboots for WiFi loss: recording does not need WiFi
        if (!apUp) apStart();
        lostSince = now; wifiBeginTry((wtry + 1) % wnetCount);
      }
      break;
    default: break;
  }
}

// ================================ NOTIFICATIONS ==============================
enum { N_PHOTO = 1, N_TEXT = 2, N_HOOK = 3 };
struct Notify { uint8_t kind; uint8_t* buf; size_t len; char text[128]; };
QueueHandle_t notifyQ = nullptr;
bool armedChangedByCmd = false;

#if TELEGRAM_VERIFY_TLS
extern const uint8_t rootca_crt_bundle_start[] asm("_binary_x509_crt_bundle_start");
extern const uint8_t rootca_crt_bundle_end[] asm("_binary_x509_crt_bundle_end");
#endif
static void tlsSetup(WiFiClientSecure& c) {
#if TELEGRAM_VERIFY_TLS
  #if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
    c.setCACertBundle(rootca_crt_bundle_start, rootca_crt_bundle_end - rootca_crt_bundle_start);
  #else
    c.setCACertBundle(rootca_crt_bundle_start);
  #endif
#else
  c.setInsecure();
#endif
  c.setTimeout(8);
}
static String urlEnc(const char* s) {
  String o; char b[4];
  for (; *s; s++) { if (isalnum((unsigned char)*s) || *s == '-' || *s == '_' || *s == '.' || *s == '~') o += *s; else { snprintf(b, sizeof b, "%%%02X", (unsigned char)*s); o += b; } }
  return o;
}
static bool tgStatusOk(WiFiClientSecure& c) {              // verifies "HTTP/1.1 200" instead of assuming success
  uint32_t t0 = millis(); while (!c.available() && c.connected() && millis() - t0 < 5000) vTaskDelay(pdMS_TO_TICKS(20));
  String l = c.readStringUntil('\n'); return l.indexOf(" 200") > 0;
}
bool tgSendPhoto(const uint8_t* buf, size_t len, const char* caption) {
  WiFiClientSecure c; tlsSetup(c); if (!c.connect("api.telegram.org", 443)) return false;
  const char* B = "----ESP32CCTV";
  String head = String("--") + B + "\r\nContent-Disposition: form-data; name=\"chat_id\"\r\n\r\n" + S_tgchat + "\r\n--" + B +
                "\r\nContent-Disposition: form-data; name=\"caption\"\r\n\r\n" + caption + "\r\n--" + B +
                "\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
  String tail = String("\r\n--") + B + "--\r\n";
  c.print(String("POST /bot") + S_tgtoken + "/sendPhoto HTTP/1.1\r\nHost: api.telegram.org\r\nContent-Length: " + String(head.length() + len + tail.length()) +
          "\r\nContent-Type: multipart/form-data; boundary=" + B + "\r\nConnection: close\r\n\r\n");
  c.print(head);
  size_t sent = 0; while (sent < len) { size_t n = c.write(buf + sent, min((size_t)2048, len - sent)); if (!n) break; sent += n; }
  c.print(tail);
  bool ok = sent == len && tgStatusOk(c); c.stop(); return ok;
}
bool tgSendText(const char* text) {
  WiFiClientSecure c; tlsSetup(c); if (!c.connect("api.telegram.org", 443)) return false;
  String body = String("chat_id=") + urlEnc(S_tgchat) + "&text=" + urlEnc(text);
  c.print(String("POST /bot") + S_tgtoken + "/sendMessage HTTP/1.1\r\nHost: api.telegram.org\r\nContent-Type: application/x-www-form-urlencoded\r\nContent-Length: " +
          String(body.length()) + "\r\nConnection: close\r\n\r\n" + body);
  bool ok = tgStatusOk(c); c.stop(); return ok;
}
void statusText(char* out, size_t n) {
  char ts[24]; tsHumanBuf(ts, sizeof ts);
  snprintf(out, n, "%s\nTime: %s\nArmed: %s | Rec mode: %ld\nSD: %s | Heap: %u KB\nRSSI: %d dBm | Up: %lu min", S_camname, ts, S_armed ? "yes" : "no",
           (long)S_recmode, sdCardMounted ? "ok" : "missing", (unsigned)(ESP.getFreeHeap() / 1024), WiFi.RSSI(), (unsigned long)(millis() / 60000));
}
static void tgPoll() {
  static long offset = 0;
  WiFiClientSecure c; tlsSetup(c); if (!c.connect("api.telegram.org", 443)) return;
  c.print(String("GET /bot") + S_tgtoken + "/getUpdates?limit=3&timeout=0&offset=" + offset + " HTTP/1.1\r\nHost: api.telegram.org\r\nConnection: close\r\n\r\n");
  String resp; uint32_t t0 = millis();
  while ((c.connected() || c.available()) && millis() - t0 < 6000 && resp.length() < 6000) { while (c.available()) resp += (char)c.read(); vTaskDelay(pdMS_TO_TICKS(10)); }
  c.stop();
  int pos = 0;
  while (true) {
    int a = resp.indexOf("\"update_id\":", pos); if (a < 0) break;
    long uid = resp.substring(a + 12).toInt(); offset = uid + 1;
    int nxt = resp.indexOf("\"update_id\":", a + 10); String chunk = resp.substring(a, nxt < 0 ? resp.length() : nxt); pos = a + 10;
    int ci = chunk.indexOf("\"chat\":{\"id\":"); int ti = chunk.indexOf("\"text\":\""); if (ci < 0 || ti < 0) continue;
    String chat = chunk.substring(ci + 13, chunk.indexOf(',', ci)); if (chat != String(S_tgchat)) continue;   // only the configured chat may command
    String cmd = chunk.substring(ti + 8, chunk.indexOf('"', ti + 8)); cmd.toLowerCase(); int at = cmd.indexOf('@'); if (at > 0) cmd = cmd.substring(0, at);
    logmsg(LVL_INFO, "TG", "Command %s", cmd.c_str());
    if (cmd == "/snap") {
      uint8_t* b = (uint8_t*)heap_caps_malloc(LATEST_BUF_SIZE, MALLOC_CAP_SPIRAM); size_t n = b ? getSnapshot(b, LATEST_BUF_SIZE) : 0;
      if (n) tgSendPhoto(b, n, S_camname); else tgSendText("Snapshot failed"); if (b) free(b);
    } else if (cmd == "/status") { char t[200]; statusText(t, sizeof t); tgSendText(t); }
    else if (cmd == "/arm")    { S_armed = true;  const Setting* s = findSetting("armed"); if (s) settingSave(*s); motionHold(2000); tgSendText("Armed"); }
    else if (cmd == "/disarm") { S_armed = false; const Setting* s = findSetting("armed"); if (s) settingSave(*s); tgSendText("Disarmed"); }
    else if (cmd == "/flash")  { flashPulseUntil = millis() + 15000; setFlashDuty(140); tgSendText("Flash on for 15 s"); }
    else if (cmd == "/help" || cmd == "/start") tgSendText("/snap /status /arm /disarm /flash");
  }
}
void notifyTask(void*) {
  Notify n; uint32_t lastPoll = 0;
  for (;;) {
    if (xQueueReceive(notifyQ, &n, pdMS_TO_TICKS(1000)) == pdTRUE) {
      bool wifiOk = WiFi.status() == WL_CONNECTED; bool tgReady = S_tg && S_tgtoken[0] && S_tgchat[0];
      if (wifiOk) {
        if (n.kind == N_PHOTO && tgReady) logmsg(tgSendPhoto(n.buf, n.len, n.text) ? LVL_INFO : LVL_WARN, "ALERT", "Telegram photo %s", "attempted");
        else if (n.kind == N_TEXT && tgReady) tgSendText(n.text);
        else if (n.kind == N_HOOK && !strncmp(S_whurl, "http://", 7)) {
          HTTPClient h; WiFiClient wc; if (h.begin(wc, S_whurl)) { h.addHeader("Content-Type", "application/json"); h.setTimeout(4000);
            String body = String("{\"camera\":\"") + jsonEsc(S_camname) + "\",\"event\":\"" + n.text + "\"}"; int code = h.POST(body); h.end(); logmsg(LVL_INFO, "ALERT", "Webhook -> %d", code); }
        }
      }
      if (n.buf) free(n.buf);
    }
    if (S_tg && S_tgcmd && S_tgtoken[0] && S_tgchat[0] && WiFi.status() == WL_CONNECTED && millis() - lastPoll > 5000) { lastPoll = millis(); tgPoll(); }
  }
}
void notifyMotion(camera_fb_t* fb, int score) {
  static uint32_t lastTg = 0, lastHook = 0; static bool tgFirst = true, hookFirst = true; uint32_t now = millis();
  char ts[24]; tsHumanBuf(ts, sizeof ts);
  if (S_tg && S_tgtoken[0] && S_tgchat[0] && (tgFirst || now - lastTg >= (uint32_t)S_tgcool * 1000UL)) {
    tgFirst = false; lastTg = now;
    uint8_t* b = (uint8_t*)heap_caps_malloc(fb->len, MALLOC_CAP_SPIRAM);
    if (b) { memcpy(b, fb->buf, fb->len); Notify n = {N_PHOTO, b, fb->len, ""}; snprintf(n.text, sizeof n.text, "Motion: %s at %s (%d%%)", S_camname, ts, score); if (xQueueSend(notifyQ, &n, 0) != pdTRUE) free(b); }
  }
  if (S_whurl[0] && (hookFirst || now - lastHook >= 10000)) { hookFirst = false; lastHook = now; Notify n = {N_HOOK, nullptr, 0, "motion"}; xQueueSend(notifyQ, &n, 0); }
}
void notifySystem(const char* text) {
  if (!notifyQ) return; Notify n = {N_TEXT, nullptr, 0, ""}; snprintf(n.text, sizeof n.text, "[%s] %s", S_camname, text); xQueueSend(notifyQ, &n, 0);
}

// =================================== MQTT ====================================
volatile bool mqPend = false, mqState = false; volatile int mqScore = 0;
void mqttMotion(bool on, int score) { mqState = on; mqScore = score; mqPend = true; }
#ifdef HAVE_MQTT
WiFiClient mqNet; PubSubClient mqtt(mqNet); uint32_t mqLastTry = 0, mqLastStatus = 0;
static void mqCb(char* topic, byte* payload, unsigned int len) {
  String t = topic; if (!t.endsWith("/arm/set")) return;
  bool on = len >= 2 && (payload[0] == 'O' || payload[0] == 'o') && (payload[1] == 'N' || payload[1] == 'n');
  S_armed = on; const Setting* s = findSetting("armed"); if (s) settingSave(*s);
}
void mqttService() {
  if (!S_mq || !S_mqhost[0] || WiFi.status() != WL_CONNECTED) return;
  String base = S_mqtopic; uint32_t now = millis();
  if (!mqtt.connected()) {
    if (now - mqLastTry < 10000) return; mqLastTry = now;
    mqtt.setServer(S_mqhost, S_mqport); mqtt.setCallback(mqCb); mqtt.setBufferSize(640);
    String cid = String(HOSTNAME) + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
    bool ok = S_mquser[0] ? mqtt.connect(cid.c_str(), S_mquser, S_mqpass, (base + "/availability").c_str(), 0, true, "offline")
                          : mqtt.connect(cid.c_str(), nullptr, nullptr, (base + "/availability").c_str(), 0, true, "offline");
    if (!ok) { logmsg(LVL_WARN, "MQTT", "Connect failed (%d)", mqtt.state()); return; }
    mqtt.publish((base + "/availability").c_str(), "online", true); mqtt.subscribe((base + "/arm/set").c_str());
    char pl[560]; String id = String(HOSTNAME) + "_" + String((uint32_t)ESP.getEfuseMac(), HEX);
    snprintf(pl, sizeof pl, "{\"name\":\"%s motion\",\"uniq_id\":\"%s_motion\",\"stat_t\":\"%s/motion\",\"pl_on\":\"ON\",\"pl_off\":\"OFF\",\"dev_cla\":\"motion\",\"avty_t\":\"%s/availability\",\"dev\":{\"ids\":[\"%s\"],\"name\":\"%s\",\"mf\":\"ESP32-CAM\"}}",
             S_camname, id.c_str(), base.c_str(), base.c_str(), id.c_str(), S_camname);
    mqtt.publish(("homeassistant/binary_sensor/" + id + "/motion/config").c_str(), pl, true);
    snprintf(pl, sizeof pl, "{\"name\":\"%s armed\",\"uniq_id\":\"%s_armed\",\"stat_t\":\"%s/armed\",\"cmd_t\":\"%s/arm/set\",\"avty_t\":\"%s/availability\",\"dev\":{\"ids\":[\"%s\"]}}",
             S_camname, id.c_str(), base.c_str(), base.c_str(), base.c_str(), id.c_str());
    mqtt.publish(("homeassistant/switch/" + id + "/armed/config").c_str(), pl, true);
    logmsg(LVL_INFO, "MQTT", "Connected to %s", S_mqhost); mqPend = true; mqState = motionOn;
  }
  mqtt.loop();
  if (mqPend) { mqPend = false; mqtt.publish((base + "/motion").c_str(), mqState ? "ON" : "OFF", true); }
  if (now - mqLastStatus > 60000) {
    mqLastStatus = now; char t[160]; snprintf(t, sizeof t, "{\"rssi\":%d,\"heap\":%u,\"fps\":%.1f,\"up\":%lu}", WiFi.RSSI(), (unsigned)ESP.getFreeHeap(), (double)currentFPS, (unsigned long)(now / 1000));
    mqtt.publish((base + "/status").c_str(), t); mqtt.publish((base + "/armed").c_str(), S_armed ? "ON" : "OFF", true);
  }
}
#else
void mqttService() {}
#endif

// ============================ FACTORY RESET / CONSOLE ========================
void factoryReset() {
  const char* ns[] = {"cctv", "auth", "wifi", "sys"};
  for (auto n : ns) { Preferences p; p.begin(n, false); p.clear(); p.end(); }
  logmsg(LVL_WARN, "SYS", "Factory reset - rebooting"); delay(300); ESP.restart();
}
// Serial recovery: RESETPW (admin/admin, forced change), FACTORY, REBOOT, INFO
void serialConsole() {
  static String line;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      line.trim(); line.toUpperCase();
      if (line == "RESETPW") { resetAdminToDefault(); for (auto& s : sessions) s.used = false; Serial.println("Admin reset to admin/admin (must change on next login)"); }
      else if (line == "FACTORY") factoryReset();
      else if (line == "REBOOT") requestReboot();
      else if (line == "INFO") Serial.printf("AP: %s / %s | IP: %s | heap %u (min %u) psram %u sd:%d/%d vw:%d\n", apSsid, apPass, WiFi.localIP().toString().c_str(), (unsigned)ESP.getFreeHeap(), (unsigned)minFreeHeapSeen, (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM), (int)sdState, sdCardMounted ? 1 : 0, (int)viewerCount);
      line = "";
    } else if (line.length() < 24) line += ch;
  }
}
