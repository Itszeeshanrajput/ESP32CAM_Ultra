#pragma once
/* AVI/MJPEG engine v2
   - spec-correct idx1 (offsets relative to the 'movi' fourcc), dynamic index in PSRAM
   - header heartbeat every AVI_SYNC_MS so a power cut leaves a playable file
   - real measured FPS is patched into the header at close (playback speed = real time)
   - aviRepair() rebuilds idx1 for files that were never closed
   - aviParse()/aviGetFrame() power the frame-accurate in-browser player                */
#ifndef AVI_ALLOC
  #define AVI_ALLOC(n)      heap_caps_malloc((n), MALLOC_CAP_SPIRAM)
  #define AVI_REALLOC(p, n) heap_caps_realloc((p), (n), MALLOC_CAP_SPIRAM)
#endif

struct AviIdx { uint32_t off, size; };
struct AviInfo {
  bool ok = false, indexed = false;
  uint32_t moviFourcc = 0, moviSize = 0, frames = 0, usPerFrame = 0, w = 0, h = 0, flags = 0, idxPos = 0, strhLenPos = 0;
  float fps = 0;
};
static inline uint32_t aviRd32(const uint8_t* p) { uint32_t v; memcpy(&v, p, 4); return v; }
static inline void aviW32(File& f, uint32_t v) { f.write((const uint8_t*)&v, 4); }

bool aviParse(File& f, AviInfo& in) {
  uint8_t h[1536]; f.seek(0); size_t n = f.read(h, sizeof h);
  if (n < 128 || memcmp(h, "RIFF", 4) || memcmp(h + 8, "AVI ", 4)) return false;
  in.usPerFrame = aviRd32(h + 32); in.flags = aviRd32(h + 44); in.frames = aviRd32(h + 48);
  in.w = aviRd32(h + 64); in.h = aviRd32(h + 68);
  in.fps = in.usPerFrame ? 1000000.0f / in.usPerFrame : 0;
  for (size_t i = 12; i + 8 < n; i++) {
    if (!in.strhLenPos && !memcmp(h + i, "strh", 4)) in.strhLenPos = i + 8 + 32;
    if (i >= 8 && !memcmp(h + i, "movi", 4) && !memcmp(h + i - 8, "LIST", 4)) { in.moviFourcc = i; in.moviSize = aviRd32(h + i - 4); break; }
  }
  if (!in.moviFourcc) return false;
  in.indexed = (in.flags & 0x10) != 0;
  in.idxPos = in.moviFourcc + in.moviSize;
  in.ok = true; return true;
}

// Reads frame n into buf. Requires a finalised (indexed) file.
bool aviGetFrame(File& f, const AviInfo& in, uint32_t n, uint8_t* buf, size_t cap, size_t& len) {
  if (!in.indexed || n >= in.frames) return false;
  uint8_t e[16]; f.seek(in.idxPos + 8 + (uint32_t)n * 16); if (f.read(e, 16) != 16) return false;
  uint32_t off = aviRd32(e + 8), sz = aviRd32(e + 12);
  if (sz == 0 || sz > cap) return false;
  uint8_t ch[8]; f.seek(in.moviFourcc + off); if (f.read(ch, 8) != 8 || memcmp(ch, "00dc", 4)) return false;
  if (f.read(buf, sz) != sz) return false; len = sz; return true;
}

class AviWriter {
public:
  bool begin(const char* path, int w, int h, int fps, const char* name, const char* dateStr, bool patchRealFps) {
    f = SD_MMC.open(path, FILE_WRITE); if (!f) return false;
    open_ = true; strlcpy(path_, path, sizeof path_); frames_ = 0; maxFrame_ = 0; patchFps_ = patchRealFps;
    idx = nullptr; idxCap = idxCount = 0; firstMs_ = lastMs_ = 0; lastSync_ = openedMs_ = millis();
    fps = constrain(fps, 1, 60);
    fcc("RIFF"); riffPos = f.position(); u32(0); fcc("AVI ");
    fcc("LIST"); uint32_t hdrl = f.position(); u32(0); fcc("hdrl");
    fcc("avih"); u32(56);
    usPos = f.position(); u32(1000000UL / fps); u32(0); u32(0);
    flagsPos = f.position(); u32(0);                          // HASINDEX is set only when the file is closed cleanly
    framesAvih = f.position(); u32(0); u32(0); u32(1);
    suggAvih = f.position(); u32((uint32_t)w * h); u32(w); u32(h); u32(0); u32(0); u32(0); u32(0);
    fcc("LIST"); uint32_t strl = f.position(); u32(0); fcc("strl");
    fcc("strh"); u32(56); fcc("vids"); fcc("MJPG"); u32(0); u16(0); u16(0); u32(0);
    scalePos = f.position(); u32(1000); u32((uint32_t)fps * 1000); u32(0);
    framesStrh = f.position(); u32(0);
    suggStrh = f.position(); u32((uint32_t)w * h); u32(0xFFFFFFFF); u32(0); u16(0); u16(0); u16(w); u16(h);
    fcc("strf"); u32(40); u32(40); u32(w); u32(h); u16(1); u16(24); fcc("MJPG"); u32((uint32_t)w * h * 3); u32(0); u32(0); u32(0); u32(0);
    uint32_t a = f.position(); f.seek(strl); u32(a - strl - 4); f.seek(a);
    uint32_t b = f.position(); f.seek(hdrl); u32(b - hdrl - 4); f.seek(b);

    if (name && *name) {                                       // INFO list: camera name + start time
      fcc("LIST"); uint32_t info = f.position(); u32(0); fcc("INFO");
      infoChunk("INAM", name); infoChunk("IDIT", dateStr ? dateStr : "");
      uint32_t e = f.position(); f.seek(info); u32(e - info - 4); f.seek(e);
    }
    fcc("LIST"); moviPos = f.position(); u32(0); fcc("movi");
    return true;
  }

  // 0 ok, 1 index full (rotate file), -1 write error
  int addFrame(const uint8_t* d, size_t len) {
    if (!open_ || !f) return -1;
    if (idxCount >= AVI_MAX_INDEX) return 1;
    if (idxCount >= idxCap) {
      size_t nc = idxCap ? idxCap * 2 : 4096; if (nc > AVI_MAX_INDEX) nc = AVI_MAX_INDEX;
      AviIdx* p = (AviIdx*)AVI_REALLOC(idx, nc * sizeof(AviIdx)); if (!p) return 1;
      idx = p; idxCap = nc;
    }
    uint32_t pos = f.position(); uint8_t hdr[8]; uint32_t L = (uint32_t)len;
    memcpy(hdr, "00dc", 4); memcpy(hdr + 4, &L, 4);
    if (f.write(hdr, 8) != 8) return -1;
    if (f.write(d, len) != len) return -1;
    if (len & 1) { uint8_t z = 0; f.write(&z, 1); }
    idx[idxCount].off = pos - (moviPos + 4); idx[idxCount].size = L; idxCount++;
    frames_++; if (len > maxFrame_) maxFrame_ = len;
    uint32_t now = millis(); if (frames_ == 1) firstMs_ = now; lastMs_ = now;
    if (now - lastSync_ >= AVI_SYNC_MS) { syncHeader(); lastSync_ = now; }
    return 0;
  }

  void syncHeader() {
    if (!open_ || !f) return;
    uint32_t cur = f.position();
    f.seek(riffPos);    u32(cur - riffPos - 4);
    f.seek(framesAvih); u32(frames_);
    f.seek(framesStrh); u32(frames_);
    f.seek(moviPos);    u32(cur - (moviPos + 4));
    f.seek(cur); f.flush();
  }

  bool finalize() {
    if (!open_ || !f) return false;
    syncHeader();
    fcc("idx1"); u32((uint32_t)idxCount * 16);
    uint32_t tmp[64 * 4];
    for (size_t i = 0; i < idxCount;) {
      size_t n = min((size_t)64, idxCount - i);
      for (size_t k = 0; k < n; k++) { memcpy(&tmp[k * 4], "00dc", 4); tmp[k * 4 + 1] = 0x10; tmp[k * 4 + 2] = idx[i + k].off; tmp[k * 4 + 3] = idx[i + k].size; }
      f.write((const uint8_t*)tmp, n * 16); i += n;
    }
    uint32_t end = f.position();
    f.seek(riffPos); u32(end - riffPos - 4);
    f.seek(flagsPos); u32(0x10);
    f.seek(suggAvih); u32(maxFrame_); f.seek(suggStrh); u32(maxFrame_);
    if (patchFps_ && frames_ >= 2 && lastMs_ > firstMs_) {     // real FPS -> correct playback speed
      float fr = (frames_ - 1) * 1000.0f / (lastMs_ - firstMs_);
      if (fr >= 0.2f && fr <= 60.0f) {
        f.seek(usPos); u32((uint32_t)(1000000.0f / fr));
        f.seek(scalePos); u32(1000); u32((uint32_t)(fr * 1000.0f + 0.5f));
      }
    }
    f.close(); open_ = false;
    if (idx) { free(idx); idx = nullptr; } idxCap = idxCount = 0;
    return true;
  }

  bool isOpen() const { return open_; }
  uint32_t frames() const { return frames_; }
  const char* path() const { return path_; }
  uint32_t ageMs() const { return open_ ? (millis() - openedMs_) : 0; }
  int w = 0, h = 0; uint32_t chunkId = 0;                     // owner bookkeeping

private:
  File f; bool open_ = false, patchFps_ = false; char path_[72] = "";
  uint32_t frames_ = 0, maxFrame_ = 0, riffPos = 0, framesAvih = 0, framesStrh = 0, moviPos = 0, usPos = 0, flagsPos = 0;
  uint32_t scalePos = 0, suggAvih = 0, suggStrh = 0, lastSync_ = 0, firstMs_ = 0, lastMs_ = 0, openedMs_ = 0;
  AviIdx* idx = nullptr; size_t idxCap = 0, idxCount = 0;
  void u32(uint32_t v) { f.write((const uint8_t*)&v, 4); }
  void u16(uint16_t v) { f.write((const uint8_t*)&v, 2); }
  void fcc(const char* c) { f.write((const uint8_t*)c, 4); }
  void infoChunk(const char* id, const char* s) {             // size = len+1 (NUL); pad when that is odd
    uint32_t n = strlen(s) + 1; fcc(id); u32(n); f.write((const uint8_t*)s, n - 1); uint8_t z = 0; f.write(&z, 1);
    if (n & 1) f.write(&z, 1);
  }
};

// Rebuilds idx1 + header for an AVI that was never closed (power loss). Returns frames recovered or -1.
int aviRepair(const char* path) {
  File f = SD_MMC.open(path, "r+"); if (!f) return -1;
  AviInfo in; if (!aviParse(f, in) || in.indexed) { f.close(); return -1; }
  uint32_t fsz = f.size(), pos = in.moviFourcc + 4, n = 0, cap = 0; AviIdx* idx = nullptr;
  while (pos + 8 <= fsz) {
    uint8_t h[8]; f.seek(pos); if (f.read(h, 8) != 8 || memcmp(h, "00dc", 4)) break;
    uint32_t sz = aviRd32(h + 4); if (sz == 0 || sz > 512 * 1024 || pos + 8 + sz > fsz) break;
    if (n >= cap) { cap = cap ? cap * 2 : 4096; AviIdx* p = (AviIdx*)AVI_REALLOC(idx, cap * sizeof(AviIdx)); if (!p) break; idx = p; }
    idx[n].off = pos - in.moviFourcc; idx[n].size = sz; n++;
    pos += 8 + sz + (sz & 1);
  }
  if (n == 0) { if (idx) free(idx); f.close(); return -1; }
  f.seek(pos); f.write((const uint8_t*)"idx1", 4); aviW32(f, n * 16);
  for (uint32_t i = 0; i < n; i++) { f.write((const uint8_t*)"00dc", 4); aviW32(f, 0x10); aviW32(f, idx[i].off); aviW32(f, idx[i].size); }
  uint32_t end = f.position();
  f.seek(4); aviW32(f, end - 8);
  f.seek(in.moviFourcc - 4); aviW32(f, pos - in.moviFourcc);
  f.seek(48); aviW32(f, n);
  if (in.strhLenPos) { f.seek(in.strhLenPos); aviW32(f, n); }
  f.seek(44); aviW32(f, in.flags | 0x10);
  f.close(); free(idx); return (int)n;
}
