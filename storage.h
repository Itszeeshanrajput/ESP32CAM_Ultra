#pragma once
#include "globals.h"
#include "avi.h"

AviWriter contWriter, motionWriter, tlWriter;
const char* EVENTS_FILE = "/events.jsonl";
const char* LOG_FILE = "/system.log";
const char* LOG_FILE_OLD = "/system_old.log";

uint64_t sdTotalB = 0, sdUsedB = 0; uint32_t sdInfoMs = 0;

static const char* baseName(const char* p) { const char* s = strrchr(p, '/'); return s ? s + 1 : p; }

bool validDayName(const char* d) {
  if (!strcmp(d, "nodate")) return true;
  if (strlen(d) != 8) return false;
  for (int i = 0; i < 8; i++) if (!isdigit((unsigned char)d[i])) return false;
  return true;
}

// Accepts only "/<day>/<name>.avi|jpg" : no traversal, no nested folders, no odd characters.
bool validMediaPath(const char* p) {
  if (!p || p[0] != '/' || strstr(p, "..")) return false;
  const char* s = strchr(p + 1, '/'); if (!s || strchr(s + 1, '/')) return false;
  size_t dl = s - (p + 1); if (dl == 0 || dl > 8) return false;
  char day[10]; memcpy(day, p + 1, dl); day[dl] = 0; if (!validDayName(day)) return false;
  size_t nl = strlen(s + 1); if (nl < 5 || nl > 39) return false;
  for (const char* c = s + 1; *c; c++) if (!(isalnum((unsigned char)*c) || *c == '_' || *c == '-' || *c == '.')) return false;
  const char* e = p + strlen(p) - 4; return !strcasecmp(e, ".avi") || !strcasecmp(e, ".jpg");
}
bool isOpenPath(const char* p) {
  return (contWriter.isOpen() && !strcmp(p, contWriter.path())) || (motionWriter.isOpen() && !strcmp(p, motionWriter.path())) ||
         (tlWriter.isOpen() && !strcmp(p, tlWriter.path()));
}

// ---------- mount / info ----------
bool sdMount() {
  if (memDmaFree() < DMA_FLOOR) return false;   // SDMMC can't get a DMA buffer -> every op fails 0x101; don't even probe (v6.1)
  pinMode(2, INPUT_PULLUP); pinMode(14, INPUT_PULLUP); pinMode(15, INPUT_PULLUP);   // ESP32-CAM has weak SD pull-ups
  for (int i = 0; i < 3; i++) {                                                       // retry, then fall back to a slower clock
    if (SD_MMC.begin("/sdcard", true, false, i == 2 ? 10000 : 20000, 12) && SD_MMC.cardType() != CARD_NONE) return true;   // 12 open files (default 5 was exceeded)
    SD_MMC.end(); delay(150);
  }
  return false;
}
// FAT free-space walk can take seconds on big cards: never under sdLock, only from the maintenance task.
void sdRefreshInfo(bool force) {
  if (!sdCardMounted) return;
  if (!force && sdInfoMs && millis() - sdInfoMs < 120000) return;
  sdTotalB = SD_MMC.totalBytes(); sdUsedB = SD_MMC.usedBytes(); sdInfoMs = millis();
}
uint64_t sdFreeB() { return sdTotalB > sdUsedB ? sdTotalB - sdUsedB : 0; }
uint64_t minFreeBytes() { uint64_t p = sdTotalB * (uint64_t)S_freepct / 100; return p > SD_MIN_FREE_BYTES ? p : SD_MIN_FREE_BYTES; }

// ---------- directory helpers ----------
struct FEnt { char n[40]; uint32_t size; bool dir; };
bool sdList(const char* dir, std::vector<FEnt>& out, size_t maxN = 600) {
  if (!sdLock(1500)) return false;
  File d = SD_MMC.open(dir);
  if (!d || !d.isDirectory()) { if (d) d.close(); sdUnlock(); return false; }
  File e = d.openNextFile();
  while (e && out.size() < maxN) {
    FEnt x; strlcpy(x.n, baseName(e.name()), sizeof x.n); x.size = e.size(); x.dir = e.isDirectory(); out.push_back(x);
    e = d.openNextFile();
  }
  d.close(); sdUnlock(); return true;
}
static bool dayLess(const String& a, const String& b) {         // "nodate" is treated as oldest
  bool an = a == "nodate", bn = b == "nodate"; if (an != bn) return an; return a < b;
}
std::vector<String> sdDays() {
  std::vector<FEnt> root; std::vector<String> d; sdList("/", root, 400);
  for (auto& e : root) if (e.dir && validDayName(e.n)) d.push_back(e.n);
  std::sort(d.begin(), d.end(), dayLess); return d;
}
static String timeKey(const char* n) { const char* u = strchr(n, '_'); return u ? String(u + 1) : String(n); }

// Caller holds sdLock. Creates the day folder and returns a name that does not exist yet (no truncation of old files).
bool buildPath(char* out, size_t n, const char* prefix, const char* ext, time_t t) {
  char day[10], hms[10], dir[16]; dayStr(day, sizeof day, t); hmsStr(hms, sizeof hms, t);
  snprintf(dir, sizeof dir, "/%s", day);
  if (!SD_MMC.exists(dir) && !SD_MMC.mkdir(dir)) return false;
  snprintf(out, n, "%s/%s_%s.%s", dir, prefix, hms, ext);
  for (int i = 1; SD_MMC.exists(out) && i < 60; i++) snprintf(out, n, "%s/%s_%s_%d.%s", dir, prefix, hms, i, ext);
  return true;
}

// ---------- retention & space management ----------
int deleteDay(const char* day) {
  std::vector<FEnt> files; char dir[16]; snprintf(dir, sizeof dir, "/%s", day);
  if (!sdList(dir, files)) return 0;
  int n = 0;
  for (auto& f : files) {
    if (f.dir) continue; char p[64]; snprintf(p, sizeof p, "%s/%s", dir, f.n);
    if (isOpenPath(p)) continue;
    esp_task_wdt_reset();
    if (sdLock(2000)) { if (SD_MMC.remove(p)) n++; sdUnlock(); }
    vTaskDelay(1);
  }
  if (sdLock(2000)) { SD_MMC.rmdir(dir); sdUnlock(); }
  return n;
}
void retentionSweep() {
  if (!sdCardMounted || S_retdays <= 0 || !timeOK()) return;
  char cut[10]; dayStr(cut, sizeof cut, time(nullptr) - (time_t)S_retdays * 86400);
  int total = 0;
  for (auto& d : sdDays()) if (d != "nodate" && strcmp(d.c_str(), cut) < 0) total += deleteDay(d.c_str());
  if (total) logmsg(LVL_WARN, "SD", "Retention removed %d file(s) older than %ld day(s)", total, (long)S_retdays);
}
// Loop recording: remove the OLDEST files (by time) until enough space is free. Never touches open files.
void spaceSweep() {
  if (!sdCardMounted) return;
  sdRefreshInfo(true); uint64_t freeB = sdFreeB(); if (freeB >= minFreeBytes()) return;
  int deleted = 0;
  for (int round = 0; round < 12 && freeB < minFreeBytes(); round++) {
    auto days = sdDays(); if (days.empty()) break;
    String day = days[0]; char dir[16]; snprintf(dir, sizeof dir, "/%s", day.c_str());
    std::vector<FEnt> files; sdList(dir, files);
    files.erase(std::remove_if(files.begin(), files.end(), [](const FEnt& f) { return f.dir; }), files.end());
    std::sort(files.begin(), files.end(), [](const FEnt& a, const FEnt& b) { return timeKey(a.n) < timeKey(b.n); });
    int cnt = 0;
    for (auto& f : files) {
      char p[64]; snprintf(p, sizeof p, "%s/%s", dir, f.n); if (isOpenPath(p)) continue;
      bool ok = false; if (sdLock(2000)) { ok = SD_MMC.remove(p); sdUnlock(); }
      if (ok) { freeB += f.size; deleted++; cnt++; }
      if (freeB >= minFreeBytes() || cnt >= 20) break;
      vTaskDelay(1);
    }
    if (cnt == 0) { bool gone = false; if (sdLock(2000)) { gone = SD_MMC.rmdir(dir); sdUnlock(); } if (!gone) break; }
  }
  sdRefreshInfo(true);
  if (deleted) logmsg(LVL_WARN, "SD", "Space manager removed %d oldest file(s), %lu MB free", deleted, (unsigned long)(sdFreeB() >> 20));
}

// ---------- storage breakdown (cached) ----------
struct StorStats { uint64_t rec = 0, mot = 0, tl = 0, img = 0, other = 0; uint32_t files = 0, days = 0; } storStats;
uint32_t storStatsMs = 0; volatile bool storScanReq = false;
void storageScan() {
  if (storStatsMs && millis() - storStatsMs < 60000) return;
  StorStats s; auto days = sdDays(); s.days = days.size();
  for (auto& d : days) {
    std::vector<FEnt> fl; char dir[16]; snprintf(dir, sizeof dir, "/%s", d.c_str()); sdList(dir, fl);
    for (auto& f : fl) {
      if (f.dir) continue; s.files++;
      if (!strncmp(f.n, "rec_", 4)) s.rec += f.size; else if (!strncmp(f.n, "mot_", 4)) s.mot += f.size;
      else if (!strncmp(f.n, "tl_", 3)) s.tl += f.size; else if (!strncmp(f.n, "ev_", 3) || !strncmp(f.n, "snap_", 5)) s.img += f.size;
      else s.other += f.size;
    }
    vTaskDelay(1);
  }
  storStats = s; storStatsMs = millis();
}

// ---------- event index ----------
void eventPush(const EventRec& e) { eventRing[eventHead] = e; eventHead = (eventHead + 1) % EVENT_RING; if (eventCount < EVENT_RING) eventCount++; }
void eventAppendFile(const EventRec& e) {                // caller holds sdLock
  File f = SD_MMC.open(EVENTS_FILE, FILE_APPEND); if (!f) return;
  if (f.size() > 256 * 1024) { f.close(); SD_MMC.remove("/events_old.jsonl"); SD_MMC.rename(EVENTS_FILE, "/events_old.jsonl"); f = SD_MMC.open(EVENTS_FILE, FILE_APPEND); if (!f) return; }
  char line[200]; int n = snprintf(line, sizeof line, "{\"ts\":%ld,\"score\":%u,\"pir\":%d,\"thumb\":\"%s\",\"clip\":\"%s\"}\n", (long)e.ts, e.score, e.pir, e.thumb, e.clip);
  f.write((const uint8_t*)line, n); f.close();
}
void eventsLoadTail() {
  if (!sdCardMounted || !sdLock(2000)) return;
  File f = SD_MMC.open(EVENTS_FILE, FILE_READ); if (!f) { sdUnlock(); return; }
  uint32_t sz = f.size(), start = sz > 9000 ? sz - 9000 : 0; f.seek(start);
  String all = f.readString(); f.close(); sdUnlock();
  int pos = start ? all.indexOf('\n') + 1 : 0;
  while (pos > 0 && pos < (int)all.length()) {
    int e = all.indexOf('\n', pos); if (e < 0) break; String ln = all.substring(pos, e); pos = e + 1;
    EventRec r; memset(&r, 0, sizeof r);
    int a = ln.indexOf("\"ts\":"); if (a < 0) continue; r.ts = ln.substring(a + 5).toInt();
    a = ln.indexOf("\"score\":"); r.score = a >= 0 ? ln.substring(a + 8).toInt() : 0;
    a = ln.indexOf("\"pir\":"); r.pir = a >= 0 && ln.charAt(a + 6) == '1';
    a = ln.indexOf("\"thumb\":\""); if (a >= 0) { int b = ln.indexOf('"', a + 9); strlcpy(r.thumb, ln.substring(a + 9, b).c_str(), sizeof r.thumb); }
    a = ln.indexOf("\"clip\":\""); if (a >= 0) { int b = ln.indexOf('"', a + 8); strlcpy(r.clip, ln.substring(a + 8, b).c_str(), sizeof r.clip); }
    eventPush(r);
  }
}

// ---------- boot-time repair of files left open by a power cut ----------
void bootRepair() {
  if (!sdCardMounted) return;
  auto days = sdDays(); int repaired = 0, looked = 0;
  for (int di = (int)days.size() - 1; di >= 0 && di >= (int)days.size() - 3 && repaired < 4; di--) {
    std::vector<FEnt> fl; char dir[16]; snprintf(dir, sizeof dir, "/%s", days[di].c_str()); sdList(dir, fl);
    for (auto& f : fl) {
      if (f.dir || f.size < 4096 || strcasecmp(f.n + strlen(f.n) - 4, ".avi")) continue;
      if (++looked > 40) return;
      char p[64]; snprintf(p, sizeof p, "%s/%s", dir, f.n);
      if (isOpenPath(p)) continue;                         // never "repair" the file being recorded
      File r = SD_MMC.open(p, FILE_READ); if (!r) continue;
      uint8_t h[52]; bool bad = r.read(h, 52) == 52 && !memcmp(h, "RIFF", 4) && !(aviRd32(h + 44) & 0x10); r.close();
      if (!bad) continue;
      int n = aviRepair(p);
      logmsg(n > 0 ? LVL_WARN : LVL_ERROR, "SD", "Repair %s: %d frames", p, n);
      if (n > 0) repaired++;
      esp_task_wdt_reset();
    }
  }
}
