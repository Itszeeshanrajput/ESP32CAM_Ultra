#pragma once
#include <esp_heap_caps.h>                       // heap_caps_get_free_size (DMA floor, v6.1)
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Update.h>
#include <Preferences.h>
#include <FS.h>
#include <SD_MMC.h>
#include <time.h>
#include <sys/time.h>
#include <sys/socket.h>
#include <stdarg.h>
#include <vector>
#include <algorithm>
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "driver/ledc.h"
#include "mbedtls/sha256.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#if __has_include(<esp_arduino_version.h>)
  #include <esp_arduino_version.h>
#endif
#include "esp_heap_caps.h"
#include "config.h"

// =============================== RUNTIME STATE ===============================
volatile bool cameraReady = false, sdCardMounted = false, otaBusy = false;
volatile bool rebootRequested = false, recPaused = false, sdPaused = false;   // sdPaused: SD in fault window - stop all SD writes silently (v6.1)
bool recSupported = false;                 // needs PSRAM
SemaphoreHandle_t sdMutex = nullptr;

volatile float    currentFPS = 0;
volatile int      viewerCount = 0;
volatile bool     motionOn = false;
volatile uint8_t  motionScoreLive = 0;
volatile uint32_t motionEvents = 0, motionLastSeen = 0, motionSince = 0;
volatile time_t   motionEpoch = 0;
volatile bool     nightState = false;
volatile uint32_t framesWritten = 0, framesDropped = 0, oversizeDropped = 0, camRecoveries = 0;
volatile uint32_t sdWriteMsAvg = 0, sdWriteMsMax = 0, sdErrors = 0;
volatile int      sdState = 0;             // 0 ok, 1 degraded, 2 remounting, 3 missing
char     recFileNow[72] = "";
uint32_t minFreeHeapSeen = UINT32_MAX;

// ---- memory safety ceilings (v6.1): internal-heap starvation is what froze the AP ----
// Below MEM_LOW the recorder/SD remounts go idle; below MEM_CRIT the web UI stops serving;
// MEM_DEAD persisting for 8 s triggers a self-healing reboot instead of an unrecoverable hang.
const uint32_t MEM_LOW = 30000, MEM_CRIT = 15000, MEM_DEAD = 11000;
uint32_t recLowLogMs = 0;                       // openWriter low-memory log throttle
const size_t DMA_FLOOR = 4096;                  // below this SDMMC can't get a DMA buffer (err 0x101) - all SD ops barred (v6.1)
static inline size_t memDmaFree() { return heap_caps_get_free_size(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT); }
String   lastResetReasonStr = "";
bool     wasBadReset = false;
int      flashBrightness = 0;
uint32_t manualFlashSince = 0;
volatile bool flashAuto = false;
int8_t   rssiRing[RSSI_RING]; int rssiHead = 0;
volatile uint32_t motionHoldAt = 0, motionHoldMs = 0;   // motion analysis paused (flash / setting changes)
static inline void motionHold(uint32_t ms) { motionHoldAt = millis(); motionHoldMs = ms; }
static inline bool inHoldoff(uint32_t now) { return (uint32_t)(now - motionHoldAt) < motionHoldMs; }

struct EventRec { time_t ts; uint8_t score; bool pir; char thumb[40]; char clip[40]; };
EventRec eventRing[EVENT_RING]; int eventHead = 0, eventCount = 0;

// ================================== LOGGING ==================================
enum LogLevel { LVL_INFO, LVL_WARN, LVL_ERROR };
struct LogLine { char s[LOG_LINE]; };
QueueHandle_t logQ = nullptr;
SemaphoreHandle_t logMx = nullptr;
char logRing[LOG_RING][LOG_LINE]; int logHead = 0, logCount = 0;

bool timeOK() { return time(nullptr) > 1700000000; }
bool timeApprox = false;

void tsHumanBuf(char* b, size_t n) {
  if (timeOK()) { time_t t = time(nullptr); struct tm m; localtime_r(&t, &m); strftime(b, n, "%Y-%m-%d %H:%M:%S", &m); }
  else snprintf(b, n, "UP+%lus", (unsigned long)(millis() / 1000));
}
void dayStr(char* b, size_t n, time_t t) {
  if (timeOK()) { struct tm m; localtime_r(&t, &m); strftime(b, n, "%Y%m%d", &m); } else strlcpy(b, "nodate", n);
}
void hmsStr(char* b, size_t n, time_t t) {
  if (timeOK()) { struct tm m; localtime_r(&t, &m); strftime(b, n, "%H%M%S", &m); }
  else snprintf(b, n, "u%06lu", (unsigned long)(millis() / 1000));
}

// Non-blocking, allocation-free, safe from any task. Never touches the SD mutex.
void logmsg(LogLevel l, const char* cat, const char* fmt, ...) {
  char msg[112]; va_list ap; va_start(ap, fmt); vsnprintf(msg, sizeof(msg), fmt, ap); va_end(ap);
  LogLine ln; char ts[24]; tsHumanBuf(ts, sizeof ts);
  snprintf(ln.s, sizeof ln.s, "[%s] [%s] [%s] %s", ts, l == LVL_ERROR ? "ERR" : l == LVL_WARN ? "WARN" : "INFO", cat, msg);
  Serial.println(ln.s);
  if (logMx && xSemaphoreTake(logMx, pdMS_TO_TICKS(20)) == pdTRUE) {
    strlcpy(logRing[logHead], ln.s, LOG_LINE); logHead = (logHead + 1) % LOG_RING; if (logCount < LOG_RING) logCount++;
    xSemaphoreGive(logMx);
  }
  if (logQ) xQueueSend(logQ, &ln, 0);
}

// ================================ SD LOCKING =================================
bool sdLock(uint32_t ms) { return sdMutex && xSemaphoreTake(sdMutex, pdMS_TO_TICKS(ms)) == pdTRUE; }
void sdUnlock()          { if (sdMutex) xSemaphoreGive(sdMutex); }

// ================================== HELPERS ==================================
String jsonEsc(const char* s) {
  String o; o.reserve(strlen(s) + 4);
  for (; *s; s++) {
    char c = *s;
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if (c == '\n') o += "\\n"; else if (c == '\r') {}
    else if ((uint8_t)c < 0x20) o += ' ';
    else o += c;
  }
  return o;
}
static inline bool reached(uint32_t now, uint32_t deadline) { return (int32_t)(now - deadline) >= 0; } // wrap-safe

void randHex(char* out, int nBytes) { for (int i = 0; i < nBytes; i++) sprintf(out + 2 * i, "%02x", (unsigned)(esp_random() & 0xFF)); out[2 * nBytes] = 0; }

// =============================== SETTINGS REGISTRY ===========================
// One table drives: NVS persistence, /api/config JSON, POST validation, backup/restore, UI binding.
char    S_camname[32] = "Front Door";
int32_t S_recmode = 1;      // 0 off, 1 continuous, 2 motion-only (pre/post-roll)
int32_t S_chunk = 15, S_recfps = 8, S_retdays = 7, S_freepct = 5;
bool    S_motion = true, S_pir = false, S_armed = true;
int32_t S_mthr = 4, S_mcool = 45, S_mframes = 2, S_preroll = 3, S_postroll = 12, S_maxclip = 120;
char    S_mzones[49] = "FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF";
bool    S_nflash = false; int32_t S_nduty = 130, S_nmode = 1, S_nstart = 19, S_nend = 6;
int32_t S_fsize = FRAMESIZE_VGA, S_quality = 12, S_bright = 0, S_contrast = 1, S_sat = 0, S_wb = 0, S_agcgain = 0, S_aecval = 300, S_aelevel = 0;
bool    S_vflip = true, S_hmirror = false, S_lowlight = true, S_dyntune = true, S_aec = true, S_agc = true;
bool    S_tl = false; int32_t S_tlint = 10;
bool    S_tg = false, S_tgsys = true, S_tgcmd = false; char S_tgtoken[64] = "", S_tgchat[24] = ""; int32_t S_tgcool = 45;
bool    S_mq = false; char S_mqhost[48] = "", S_mquser[32] = "", S_mqpass[32] = "", S_mqtopic[32] = "esp32cam"; int32_t S_mqport = 1883;
char    S_whurl[96] = "";
int32_t S_tzmin = 300; char S_ntp[40] = "pool.ntp.org";
bool    S_static = false; char S_sip[16] = "", S_sgw[16] = "", S_ssn[16] = "255.255.255.0", S_sdns[16] = "8.8.8.8";
int32_t S_txpow = 15, S_strfps = 15;
bool    S_authon = true;

enum SType : uint8_t { T_INT, T_BOOL, T_STR };
struct Setting { const char* key; SType type; void* ptr; int32_t mn, mx; uint16_t smax; bool secret; void (*cb)(); };
void cbCam(); void cbNight(); void cbTime(); void cbNet(); void cbMotion();

#define CI(k, v, a, b, cb) {k, T_INT, &v, a, b, 0, false, cb}
#define CB(k, v, cb)       {k, T_BOOL, &v, 0, 1, 0, false, cb}
#define CS(k, v, sec, cb)  {k, T_STR, v, 0, 0, (uint16_t)sizeof(v), sec, cb}
const Setting SETTINGS[] = {
  CS("camname", S_camname, false, nullptr),
  CI("recmode", S_recmode, 0, 2, nullptr), CI("chunk", S_chunk, 1, 240, nullptr), CI("recfps", S_recfps, 2, 15, nullptr),
  CI("retdays", S_retdays, 0, 365, nullptr), CI("freepct", S_freepct, 1, 40, nullptr),
  CB("armed", S_armed, cbMotion), CB("motion", S_motion, cbMotion), CB("pir", S_pir, cbMotion),
  CI("mthr", S_mthr, 1, 50, cbMotion), CI("mcool", S_mcool, 2, 600, nullptr), CI("mframes", S_mframes, 1, 6, nullptr),
  CI("preroll", S_preroll, 0, 6, nullptr), CI("postroll", S_postroll, 3, 120, nullptr), CI("maxclip", S_maxclip, 10, 600, nullptr),
  CS("mzones", S_mzones, false, cbMotion),
  CB("nflash", S_nflash, nullptr), CI("nduty", S_nduty, 10, FLASH_MAX_DUTY, nullptr),
  CI("nmode", S_nmode, 0, 2, cbNight), CI("nstart", S_nstart, 0, 23, cbNight), CI("nend", S_nend, 0, 23, cbNight),
  CI("fsize", S_fsize, 0, 20, cbCam), CI("quality", S_quality, 8, 40, cbCam), CI("bright", S_bright, -2, 2, cbCam),
  CI("contrast", S_contrast, -2, 2, cbCam), CI("sat", S_sat, -2, 2, cbCam), CI("wb", S_wb, 0, 4, cbCam),
  CB("vflip", S_vflip, cbCam), CB("hmirror", S_hmirror, cbCam), CB("lowlight", S_lowlight, cbNight), CB("dyntune", S_dyntune, cbNight),
  CB("aec", S_aec, cbCam), CI("aecval", S_aecval, 0, 1200, cbCam), CB("agc", S_agc, cbCam), CI("agcgain", S_agcgain, 0, 30, cbCam),
  CI("aelevel", S_aelevel, -2, 2, cbCam),
  CB("tl", S_tl, nullptr), CI("tlint", S_tlint, 2, 300, nullptr),
  CB("tg", S_tg, nullptr), CS("tgtoken", S_tgtoken, true, nullptr), CS("tgchat", S_tgchat, false, nullptr),
  CI("tgcool", S_tgcool, 5, 3600, nullptr), CB("tgsys", S_tgsys, nullptr), CB("tgcmd", S_tgcmd, nullptr),
  CB("mq", S_mq, nullptr), CS("mqhost", S_mqhost, false, nullptr), CI("mqport", S_mqport, 1, 65535, nullptr),
  CS("mquser", S_mquser, false, nullptr), CS("mqpass", S_mqpass, true, nullptr), CS("mqtopic", S_mqtopic, false, nullptr),
  CS("whurl", S_whurl, false, nullptr),
  CI("tzmin", S_tzmin, -720, 840, cbTime), CS("ntp", S_ntp, false, cbTime),
  CB("static", S_static, nullptr), CS("sip", S_sip, false, nullptr), CS("sgw", S_sgw, false, nullptr),
  CS("ssn", S_ssn, false, nullptr), CS("sdns", S_sdns, false, nullptr),
  CI("txpow", S_txpow, 8, 20, cbNet), CI("strfps", S_strfps, 1, 15, nullptr),
  CB("authon", S_authon, nullptr),
};
const int SETTINGS_N = sizeof(SETTINGS) / sizeof(SETTINGS[0]);

const Setting* findSetting(const char* k) { for (int i = 0; i < SETTINGS_N; i++) if (!strcmp(SETTINGS[i].key, k)) return &SETTINGS[i]; return nullptr; }

// returns true if value changed
bool settingApply(const Setting& s, const String& v) {
  switch (s.type) {
    case T_INT:  { int32_t x = constrain((int32_t)v.toInt(), s.mn, s.mx); int32_t* p = (int32_t*)s.ptr; if (*p == x) return false; *p = x; return true; }
    case T_BOOL: { bool x = (v == "1" || v == "true" || v == "on"); bool* p = (bool*)s.ptr; if (*p == x) return false; *p = x; return true; }
    case T_STR:  {
      char tmp[128];
      if (s.secret) { if (v.length() == 0) return false; if (v == "-") tmp[0] = 0; else strlcpy(tmp, v.c_str(), sizeof tmp); }
      else strlcpy(tmp, v.c_str(), sizeof tmp);
      tmp[s.smax - 1] = 0;
      if (!strncmp(tmp, (char*)s.ptr, s.smax)) return false;
      strlcpy((char*)s.ptr, tmp, s.smax); return true;
    }
  }
  return false;
}

void settingSave(const Setting& s) {
  Preferences p; if (!p.begin("cctv", false)) return;
  if (s.type == T_INT) p.putInt(s.key, *(int32_t*)s.ptr);
  else if (s.type == T_BOOL) p.putBool(s.key, *(bool*)s.ptr);
  else p.putString(s.key, (char*)s.ptr);
  p.end();
}

void settingsLoad() {
  Preferences p; if (!p.begin("cctv", false)) return;
  for (int i = 0; i < SETTINGS_N; i++) {
    const Setting& s = SETTINGS[i];
    if (!p.isKey(s.key)) continue;
    if (s.type == T_INT) *(int32_t*)s.ptr = constrain(p.getInt(s.key, *(int32_t*)s.ptr), s.mn, s.mx);
    else if (s.type == T_BOOL) *(bool*)s.ptr = p.getBool(s.key, *(bool*)s.ptr);
    else { String v = p.getString(s.key, (char*)s.ptr); strlcpy((char*)s.ptr, v.c_str(), s.smax); }
  }
  p.end();
}

// secrets are never exported; the UI receives only a "is set" flag
void settingsJson(String& o, bool forBackup) {
  o += "{";
  bool first = true; char nb[16];
  for (int i = 0; i < SETTINGS_N; i++) {
    const Setting& s = SETTINGS[i];
    if (s.secret) continue;
    if (!first) o += ","; first = false;
    o += "\""; o += s.key; o += "\":";
    if (s.type == T_INT) { snprintf(nb, sizeof nb, "%ld", (long)*(int32_t*)s.ptr); o += nb; }
    else if (s.type == T_BOOL) o += *(bool*)s.ptr ? "true" : "false";
    else { o += "\""; o += jsonEsc((char*)s.ptr); o += "\""; }
  }
  o += "}";
}

// ================================ TIMEZONE ===================================
void applyTimezone() {
  char tz[24]; int m = abs(S_tzmin);
  snprintf(tz, sizeof tz, "UTC%c%d:%02d", S_tzmin >= 0 ? '-' : '+', m / 60, m % 60);
  setenv("TZ", tz, 1); tzset();
}

// ============================ ACCOUNTS / SESSIONS ============================
enum { ROLE_NONE = 0, ROLE_VIEWER = 1, ROLE_ADMIN = 2 };
struct Account { char user[24]; char salt[33]; char hash[65]; };
Account acctAdmin, acctViewer; bool pwMustChange = false;

struct Session { bool used; char sid[33]; char csrf[33]; uint8_t role; uint32_t expires; };
Session sessions[MAX_SESSIONS];
struct FailRec { uint32_t ip; uint8_t n; uint32_t until; };
FailRec fails[4];

void sha256Raw(const uint8_t* in, size_t len, uint8_t out[32]) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  mbedtls_sha256(in, len, out, 0);
#else
  mbedtls_sha256_ret(in, len, out, 0);
#endif
}

// salted + 1000x stretched SHA-256 (plaintext passwords are never stored)
void hashPassword(const char* salt, const char* pw, char out[65]) {
  uint8_t h[32]; String s = String(salt) + pw;
  sha256Raw((const uint8_t*)s.c_str(), s.length(), h);
  uint8_t buf[32 + 33];
  for (int i = 0; i < 1000; i++) { memcpy(buf, h, 32); size_t sl = strlen(salt); memcpy(buf + 32, salt, sl); sha256Raw(buf, 32 + sl, h); }
  for (int i = 0; i < 32; i++) sprintf(out + 2 * i, "%02x", h[i]);
  out[64] = 0;
}
bool ctEq(const char* a, const char* b) { size_t la = strlen(a), lb = strlen(b); if (la != lb) return false; uint8_t d = 0; for (size_t i = 0; i < la; i++) d |= a[i] ^ b[i]; return d == 0; }

void setAccount(Account& a, const char* user, const char* pw) {
  strlcpy(a.user, user, sizeof a.user); randHex(a.salt, 16); hashPassword(a.salt, pw, a.hash);
}
void saveAccounts() {
  Preferences p; if (!p.begin("auth", false)) return;
  p.putString("au", acctAdmin.user); p.putString("as", acctAdmin.salt); p.putString("ah", acctAdmin.hash);
  p.putString("vu", acctViewer.user); p.putString("vs", acctViewer.salt); p.putString("vh", acctViewer.hash);
  p.putBool("must", pwMustChange); p.end();
}
void resetAdminToDefault() { setAccount(acctAdmin, "admin", "admin"); pwMustChange = true; saveAccounts(); }
void loadAccounts() {
  Preferences p; p.begin("auth", false);
  String ah = p.getString("ah", "");
  if (ah.length() != 64) { p.end(); memset(&acctViewer, 0, sizeof acctViewer); resetAdminToDefault(); return; }
  strlcpy(acctAdmin.user, p.getString("au", "admin").c_str(), sizeof acctAdmin.user);
  strlcpy(acctAdmin.salt, p.getString("as", "").c_str(), sizeof acctAdmin.salt);
  strlcpy(acctAdmin.hash, ah.c_str(), sizeof acctAdmin.hash);
  strlcpy(acctViewer.user, p.getString("vu", "").c_str(), sizeof acctViewer.user);
  strlcpy(acctViewer.salt, p.getString("vs", "").c_str(), sizeof acctViewer.salt);
  strlcpy(acctViewer.hash, p.getString("vh", "").c_str(), sizeof acctViewer.hash);
  pwMustChange = p.getBool("must", false); p.end();
}

uint8_t verifyLogin(const char* u, const char* pw) {
  char h[65];
  if (!strcmp(u, acctAdmin.user)) { hashPassword(acctAdmin.salt, pw, h); if (ctEq(h, acctAdmin.hash)) return ROLE_ADMIN; }
  if (acctViewer.user[0] && !strcmp(u, acctViewer.user)) { hashPassword(acctViewer.salt, pw, h); if (ctEq(h, acctViewer.hash)) return ROLE_VIEWER; }
  return ROLE_NONE;
}

FailRec* failFor(uint32_t ip) {
  for (auto& f : fails) if (f.ip == ip) return &f;
  for (auto& f : fails) if (f.ip == 0) { f.ip = ip; return &f; }
  fails[0] = FailRec{ip, 0, 0}; return &fails[0];
}
bool lockedOut(uint32_t ip) { FailRec* f = failFor(ip); return f->until && !reached(millis(), f->until); }
void noteFail(uint32_t ip) { FailRec* f = failFor(ip); if (++f->n >= 5) { f->until = millis() + 300000UL; f->n = 0; if (!f->until) f->until = 1; } }
void noteOk(uint32_t ip) { FailRec* f = failFor(ip); f->n = 0; f->until = 0; }

Session* createSession(uint8_t role) {
  Session* slot = nullptr; uint32_t now = millis();
  for (auto& s : sessions) { if (s.used && reached(now, s.expires)) s.used = false; if (!s.used && !slot) slot = &s; }
  if (!slot) slot = &sessions[0]; // evict oldest slot
  slot->used = true; slot->role = role; slot->expires = now + SESSION_TTL_MS;
  randHex(slot->sid, 16); randHex(slot->csrf, 16); return slot;
}
Session* findSession(const char* sid) {
  uint32_t now = millis();
  for (auto& s : sessions) {
    if (!s.used) continue;
    if (reached(now, s.expires)) { s.used = false; continue; }
    if (ctEq(s.sid, sid)) { s.expires = now + SESSION_TTL_MS; return &s; }
  }
  return nullptr;
}
bool cookieGet(const char* cookies, const char* name, char* out, size_t n) {
  size_t nl = strlen(name); const char* p = cookies;
  while (p && *p) {
    while (*p == ' ' || *p == ';') p++;
    if (!strncmp(p, name, nl) && p[nl] == '=') { p += nl + 1; size_t i = 0; while (*p && *p != ';' && i < n - 1) out[i++] = *p++; out[i] = 0; return true; }
    p = strchr(p, ';');
  }
  return false;
}
uint8_t roleFromCookie(const char* cookies) {
  if (!S_authon) return ROLE_ADMIN;
  char sid[40]; if (!cookieGet(cookies, "sid", sid, sizeof sid)) return ROLE_NONE;
  Session* s = findSession(sid); return s ? s->role : (uint8_t)ROLE_NONE;
}

// ============================== RESET HISTORY ================================
String resetReasonString() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "Power-on";      case ESP_RST_SW: return "Software restart";
    case ESP_RST_PANIC: return "Panic / crash";   case ESP_RST_INT_WDT: return "Interrupt watchdog";
    case ESP_RST_TASK_WDT: return "Task watchdog"; case ESP_RST_WDT: return "Watchdog";
    case ESP_RST_BROWNOUT: return "Brownout (weak power supply)"; case ESP_RST_DEEPSLEEP: return "Deep sleep wake";
    default: return "Unknown";
  }
}
bool isBadReset(esp_reset_reason_t r) { return r == ESP_RST_PANIC || r == ESP_RST_INT_WDT || r == ESP_RST_TASK_WDT || r == ESP_RST_WDT || r == ESP_RST_BROWNOUT; }
String resetHistory() { Preferences p; p.begin("sys", true); String s = p.getString("rh", ""); p.end(); return s; }
void pushResetHistory(const String& reason) {
  Preferences p; p.begin("sys", false);
  String h = p.getString("rh", ""); char ts[24]; tsHumanBuf(ts, sizeof ts);
  h = reason + " @" + ts + ";" + h; if (h.length() > 320) h = h.substring(0, 320);
  p.putString("rh", h); p.end();
}

void requestReboot() { rebootRequested = true; }   // recorder finalises files, then restarts

// Cross-module prototypes
void notifyMotion(camera_fb_t* fb, int score);
void notifySystem(const char* text);
void mqttMotion(bool on, int score);
