// =============================================================================
// WiFi-CSI Radar — multi-board console
//
//   FREENOVE_FNK0104B:
//     Single 240x320 ILI9341 display with FT6336U touchscreen.
//     Touch zones: top=radar area (unused), BL=thr-, BML=thr+, BMJ=cal, BR=menu.
//     CSI sensing without WiFi connection.
//
//   CARDPUTER_BOARD (default):
//     Dual-screen: built-in 1.14" (bottom) + external 2.8" ILI9341 (top).
//     Keyboard: c=calibrate, ,=threshold-, /=threshold+, `=settings menu.
//     On-device CSI capture (no WiFi connection required).
//
//   Both: CSI sensing uses the ESP32's own WiFi NIC in monitor mode —
//   no connection to an AP is needed or used.
// =============================================================================

#include <math.h>
#include <esp_heap_caps.h>

// ── Freenove FNK0104B ────────────────────────────────────────────────────────
#if defined(FREENOVE_FNK0104B)
#include <driver/i2c.h>
#include <driver/ledc.h>
#include <driver/timer.h>
#include "esp_wifi.h"
#include "esp_wifi_types.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "wifi_creds.h"  // WIFI_SSID / WIFI_PASS / WIFI_CHANNEL
#include "radar_link.h"
#include "freenove_display.h"

static FreenoveDisplay        display;
static lgfx::v1::LGFX_Sprite canvas(&display);
static bool                  displayReady = false;
static char                  gIpStr[16] = "no-ip";   // local STA IP, set by wifi event
static esp_netif_t* sta_netif = NULL;
static esp_netif_t* ap_netif = NULL;
static esp_netif_t* eth_netif = NULL;

// ── Cardputer ADV ─────────────────────────────────────────────────────────────
#else
#include <M5Cardputer.h>
#include <M5Unified.h>
#include "esp_wifi.h"
#include "radar_link.h"
#include "ext_panel.h"

static M5Canvas         canvas(&M5Cardputer.Display);   // built-in screen
static LGFX_ExtILI9341 extPanel;                       // external 2.8" ILI9341
static M5Canvas         topCanvas(&extPanel);           // top render buffer (240x180)
static bool             extReady = false;
#endif

static RadarLink        radar;

// Shared CSI state
static bool     wifiReady = true;

static float           gThreshold = 0.0f;   // 0% = EMA noise floor; >0% = manual threshold culling
#if !defined(FREENOVE_FNK0104B)
static const float     EXT_ZOOM = 320.0f / 240.0f;   // 240x180 * 1.333 = 320x240
#endif

// ── Color palettes & runtime settings ─────────────────────────────────────────
struct Palette { uint8_t aR, aG, aB, bR, bG, bB; const char* name; };
static const Palette kPalettes[] = {
    { 220,   0, 200,   0, 220, 220, "MAGENTA" },
    {   0, 220,  80,   0, 180, 100,  "GREEN"  },
    { 255, 180,   0, 220, 140,   0,  "AMBER"  },
    { 255,  40,  40, 255, 120,  40,  "RED"    },
    {  40, 120, 255,   0, 200, 255,  "BLUE"   },
};
static const uint8_t kNumPalettes = 5;

static uint8_t  gColorIdx = 0;
static uint8_t  gBright = 80;
static uint8_t  gExtBright = 80;
static uint16_t gColA = 0;
static uint16_t gColB = 0;
static bool     gMenuOpen = false;
static uint8_t  gMenuCursor = 0;

enum class VizMode : uint8_t { SCOPE = 0, DEVICE_LIST = 1, FFT_LINES = 2 };
static constexpr uint8_t kVizModeCount = 3;
static VizMode gVizMode = VizMode::FFT_LINES;  // default to FFT lines

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint16_t)(r & 0xF8) << 8) | ((uint16_t)(g & 0xFC) << 3) | (b >> 3);
}

static void applyPalette(uint8_t idx) {
  if (idx >= kNumPalettes) idx = 0;
  gColorIdx = idx;
  gColA = rgb565(kPalettes[idx].aR, kPalettes[idx].aG, kPalettes[idx].aB);
  gColB = rgb565(kPalettes[idx].bR, kPalettes[idx].bG, kPalettes[idx].bB);
}

static void applyBrightness() {
  #if defined(FREENOVE_FNK0104B)
  display.setBrightness((uint32_t)gBright * 255 / 100);
  #else
  M5Cardputer.Display.setBrightness((uint32_t)gBright * 255 / 100);
  #endif
}

static void saveSettings() {
  // Settings are compile-time defaults; no persistence
}

static void loadSettings() {
  // Use compile-time defaults
  if (gColorIdx >= kNumPalettes) gColorIdx = 0;
  if (gBright < 10 || gBright    > 100) gBright = 80;
  if (gExtBright < 10 || gExtBright > 100) gExtBright = 80;
  applyPalette(gColorIdx);
}

// ── CSI sensing (shared) ──────────────────────────────────────────────────────
static const int  kCsiWindow = 50;
static float      gCsiAmpBuf[kCsiWindow];
static float      gCsiPhaBuf[kCsiWindow];
static int        gCsiAmpIdx = 0;
static int        gCsiAmpFilled = 0;
static volatile float    gCsiMotion = 0.0f;
static volatile int8_t   gCsiRssi = -80;
static volatile uint32_t gCsiCount = 0;
static float      gCsiVarMax = 0.001f;
static float      gCsiVarMin = 0.0f;
static float      gCsiPhaVarMax = 0.001f;
static float      gCsiPhaVarMin = 0.0f;
static bool       gCalibrating = false;
static uint32_t   gCalStartMs = 0;
static const uint32_t kCalDurationMs = 10000;   // 10 s empty-room training

static void IRAM_ATTR promiscuousRxCb(void*, wifi_promiscuous_pkt_type_t) {}

// ── Doppler FFT analysis ────────────────────────────────────────────────────
// 128-point FFT at 15 Hz sampling → 0.117 Hz/bin, Nyquist = 7.5 Hz
// Human motion: 0.9–7.4 Hz (bins 8–63). Fan detection requires >120 Hz sampling.
// dl_fft: SIMD-optimized on S3 via aes3.S
// #define FFT_DEBUG  // set to enable serial debug output
#include "dl_fft.h"
static const int  kDopplerFrames = 64;        // 64 frames @ 15 Hz ≈ 4.3 s window
static const int  kDopplerFFTSize = 64;        // must be power of 2
static const float kDopplerHzPerBin = 15.0f / (float)kDopplerFFTSize;  // ≈ 0.234 Hz/bin
static const int  kDopplerHzBinLo = 4;        // ~0.9 Hz — above slow drift, clear of bin 5 noise
static const int  kDopplerHzBinHi = 31;        // ~7.3 Hz — Nyquist (7.5 Hz)

enum class DopplerClass { STATIC, HUMAN };

// Forward declaration (DeviceEntry defined later in this file)
struct DeviceEntry;

// Shared FFT engine and working buffer (one FFT at a time, all MACs share)
static dl_fft_f32_t* gDopplerHandle = nullptr;
static float* gDopplerFftWork;  // 256-float complex working buffer (PSRAM)
static float  gDopplerWin[kDopplerFrames];     // Hann window (not PSRAM — small)

// Allocate per-device PSRAM Doppler buffers for all device slots
static void initDoppler() {
    gDopplerFftWork = (float*)heap_caps_malloc(kDopplerFFTSize * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
    if (!gDopplerFftWork) {
        #ifdef FFT_DEBUG
        Serial.println("# Doppler: PSRAM alloc failed!");
        #endif
        return;
    }
    // Pre-compute Hann window
    for (int i = 0; i < kDopplerFFTSize; i++) {
        gDopplerWin[i] = 0.5f * (1.0f - cosf(2.0f * PI * i / (kDopplerFFTSize - 1)));
    }
    // Init dl_fft (SIMD-optimized on S3 via aes3.S)
    gDopplerHandle = dl_fft_f32_init(kDopplerFFTSize, MALLOC_CAP_SPIRAM);
    if (!gDopplerHandle) {
        #ifdef FFT_DEBUG
        Serial.println("# Doppler: dl_fft_f32_init failed!");
        #endif
        return;
    }
    #ifdef FFT_DEBUG
    Serial.printf("# Doppler: dl_fft OK, %d-point (%.1f Hz/bin)\n",
                  kDopplerFFTSize, kDopplerHzPerBin);
    #endif
}
// ── Per-MAC device table ─────────────────────────────────────────────────────
static constexpr int kMaxDevices = 16;
struct DeviceEntry {
  uint8_t  mac[6];
  int8_t   rssi = -100;
  int8_t   noise = -100;
  uint8_t  channel = 0;
  float    motion = 0.0f;   // perturbation 0..1
  uint32_t lastSeen = 0;
  uint16_t pktCount = 0;     // packets in current 1-second window
  uint16_t pktRate = 0;     // packets/sec (rolling)
  uint8_t  lowPpsSecs = 0;   // consecutive seconds with pktRate == 0
  uint8_t  lowRssiSecs = 0;  // consecutive seconds with rssi <= -80
  uint8_t  goodSecs = 0;     // consecutive seconds passing both thresholds
  bool     hidden = false;   // hidden if low for too long
  bool     visible = false;  // not shown until good for 10 s
  bool     valid = false;
  // Per-MAC Doppler FFT state (PSRAM buffers allocated lazily)
  float*   dopplerBuf = nullptr;   // 128-float circular time-domain buffer
  float*   dopplerFft = nullptr;   // 256-float complex FFT buffer
  int      dopplerIdx = 0;         // circular write index
  int      dopplerFilled = 0;      // samples accumulated
  float    dopplerHpState = 0.0f; // HP filter state
  float    dopplerHz = 0.0f;      // dominant frequency
  int      dopplerPeakBin = 0;    // bin of dominant frequency
  float    dopplerNoiseFloor[32] = {0};  // EMA noise floor per bin (dB magnitude)
  float    noiseFloorAlpha = 0.02f;      // EMA adaptation rate (2% per frame)
  DopplerClass dopplerClass = DopplerClass::STATIC;
};
static DeviceEntry  gDevices[kMaxDevices];

// Run per-device FFT: process device[i]'s Doppler buffer if full
static bool gDeviceListDirty = false;  // defined here so runDopplerFFTForDevice can set it
static void runDopplerFFTForDevice(int i) {
    DeviceEntry& d = gDevices[i];
    if (!d.dopplerBuf || !d.dopplerFft || !gDopplerHandle) return;
    if (d.dopplerFilled < kDopplerFrames) return;

    // Copy HP-filtered time-domain buffer to complex FFT input, apply Hann window
    for (int j = 0; j < kDopplerFFTSize; j++) {
        gDopplerFftWork[j * 2]     = d.dopplerBuf[j] * gDopplerWin[j]; // real × Hann
        gDopplerFftWork[j * 2 + 1] = 0.0f;                             // imag = 0
    }

    // Run FFT (SIMD on S3 — dl_fft handles bit-reverse internally)
    dl_fft_f32_run(gDopplerHandle, gDopplerFftWork);

    // Copy FFT result to device's buffer so the bar graph can read it later
    for (int j = 0; j < kDopplerFFTSize; j++) {
        d.dopplerFft[j * 2]     = gDopplerFftWork[j * 2];
        d.dopplerFft[j * 2 + 1] = gDopplerFftWork[j * 2 + 1];
    }

    // Per-bin magnitudes for this frame
    float mag32[32] = {0};
    for (int j = 1; j < kDopplerFFTSize / 2; j++) {
        float re = d.dopplerFft[j * 2];
        float im = d.dopplerFft[j * 2 + 1];
        mag32[j] = sqrtf(re * re + im * im);  // magnitude
        // Update EMA noise floor every frame (slow adaptation)
        d.dopplerNoiseFloor[j] = d.dopplerNoiseFloor[j] * (1.0f - d.noiseFloorAlpha)
                                 + mag32[j] * d.noiseFloorAlpha;
    }

    // Determine minMag threshold
    float peakRaw = 0.0f;
    for (int j = kDopplerHzBinLo; j <= kDopplerHzBinHi; j++) {
        if (mag32[j] > peakRaw) peakRaw = mag32[j];
    }
    // Manual threshold: fraction of peak. EMA threshold (0%): subtract noise floor, floor at 0
    float minMag = 0.0f;
    if (gThreshold > 0.0f && peakRaw > 0.0f) {
        minMag = gThreshold * peakRaw;  // manual fraction-of-peak culling
    }

    // Classify: peak must be meaningfully above human-band average
    int peakBin = 0;
    float peakSig = 0.0f;
    float bandSig = 0.0f;
    int bandCount = 0;
    for (int j = kDopplerHzBinLo; j <= kDopplerHzBinHi; j++) {
        float sig = mag32[j];
        if (gThreshold == 0.0f) {
            sig = mag32[j] - d.dopplerNoiseFloor[j];  // EMA noise subtraction
            if (sig < 0.0f) sig = 0.0f;
        } else if (sig < minMag) {
            sig = 0.0f;
        }
        if (sig > peakSig) { peakSig = sig; peakBin = j; }
        bandSig += sig;
        if (sig > 0.0f) bandCount++;
    }

    d.dopplerHz = peakBin * kDopplerHzPerBin;
    d.dopplerPeakBin = peakBin;

    float avgSig = bandCount > 0 ? bandSig / bandCount : 0.0f;
    if (peakBin == 0 || peakSig < avgSig * 2.0f) {
        d.dopplerClass = DopplerClass::STATIC;
    } else {
        d.dopplerClass = DopplerClass::HUMAN;
    }
    gDeviceListDirty = true;  // force redraw so HUMAN/STATIC updates immediately
    #ifdef FFT_DEBUG
    Serial.printf("# Doppler[%d] %02x:%02x:%02x:%02x:%02x:%02x  %.1f Hz  %s\n",
        i,
        d.mac[0], d.mac[1], d.mac[2], d.mac[3], d.mac[4], d.mac[5],
        d.dopplerHz,
        d.dopplerClass == DopplerClass::HUMAN ? "HUMAN" : "STATIC");
    #endif
}

static DeviceEntry  _sorted[kMaxDevices];   // stable sort order
static int          _sortedCount = 0;
static constexpr uint8_t kHideAfterLowPpsSecs = 10;  // seconds before hiding
static constexpr uint8_t kHideAfterLowRssiSecs = 10;

static void IRAM_ATTR csiCallback(void*, wifi_csi_info_t* info) {
  if (!info || !info->buf || info->len < 4) return;
  gCsiCount++;
  int8_t* b = info->buf;
  int  nPairs = info->len / 2;

  float ampSum = 0.0f, phaseSum = 0.0f;
  for (int i = 0; i < nPairs; i++) {
    float r = (float)b[2 * i];
    float im = (float)b[2 * i + 1];
    float amp = sqrtf(r * r + im * im);
    ampSum += amp;
    // Correct phase: atan2(im, r), not im/amp (that's sin(phase), not phase)
    phaseSum += atan2f(im, r);
  }
  float meanAmp = ampSum / (float)nPairs;
  float meanPhase = phaseSum / (float)nPairs;

  gCsiAmpBuf[gCsiAmpIdx] = meanAmp;
  gCsiPhaBuf[gCsiAmpIdx] = meanPhase;
  gCsiAmpIdx = (gCsiAmpIdx + 1) % kCsiWindow;
  if (gCsiAmpFilled < kCsiWindow) gCsiAmpFilled++;
  int n = gCsiAmpFilled;

  float vsum = 0.0f;
  for (int i = 0; i < n; i++) vsum += gCsiAmpBuf[i];
  float vmean = vsum / (float)n;
  float var = 0.0f;
  for (int i = 0; i < n; i++) { float d = gCsiAmpBuf[i] - vmean; var += d * d; }
  var /= (float)n;

  float psum = 0.0f;
  for (int i = 0; i < n; i++) psum += gCsiPhaBuf[i];
  float pmean = psum / (float)n;
  float pvar = 0.0f;
  for (int i = 0; i < n; i++) { float d = gCsiPhaBuf[i] - pmean; pvar += d * d; }
  pvar /= (float)n;

  if (gCsiVarMin < 0.0001f) gCsiVarMin = var;
  else gCsiVarMin += (var - gCsiVarMin) * ((var < gCsiVarMin) ? 0.1f : 0.002f);
  if (var > gCsiVarMax)  gCsiVarMax = var;
  else gCsiVarMax += (var - gCsiVarMax) * 0.005f;
  float range = gCsiVarMax - gCsiVarMin;
  float ampMotion = (range > 0.0001f) ? ((var - gCsiVarMin) / range) : 0.0f;
  if (ampMotion < 0.0f) ampMotion = 0.0f;
  if (ampMotion > 1.0f) ampMotion = 1.0f;

  if (gCsiPhaVarMin < 0.0001f) gCsiPhaVarMin = pvar;
  else gCsiPhaVarMin += (pvar - gCsiPhaVarMin) * ((pvar < gCsiPhaVarMin) ? 0.1f : 0.002f);
  if (pvar > gCsiPhaVarMax) gCsiPhaVarMax = pvar;
  else gCsiPhaVarMax += (pvar - gCsiPhaVarMax) * 0.005f;
  float prange = gCsiPhaVarMax - gCsiPhaVarMin;
  float phaMotion = (prange > 0.0001f) ? ((pvar - gCsiPhaVarMin) / prange) : 0.0f;
  if (phaMotion < 0.0f) phaMotion = 0.0f;
  if (phaMotion > 1.0f) phaMotion = 1.0f;

  gCsiMotion = 0.6f * ampMotion + 0.4f * phaMotion;
  gCsiRssi = info->rx_ctrl.rssi;

  // ── Update per-MAC device table ─────────────────────────────────
  uint8_t* mac = info->mac;
  int8_t   rssi = info->rx_ctrl.rssi;
  int8_t   noise = info->rx_ctrl.noise_floor;
  uint32_t now = millis();

  // Update per-MAC device table
  int slot = -1;
  bool isNew = false;
  for (int i = 0; i < kMaxDevices; i++) {
    if (!gDevices[i].valid) { slot = i; isNew = true; break; }
    if (memcmp(gDevices[i].mac, mac, 6) == 0) { slot = i; break; }
  }
  if (slot < 0) {
    uint32_t oldest = UINT32_MAX;
    for (int i = 0; i < kMaxDevices; i++) {
      if (gDevices[i].lastSeen < oldest) { oldest = gDevices[i].lastSeen; slot = i; }
    }
    isNew = true;
  }
  if (slot >= 0) {
    DeviceEntry& dev = gDevices[slot];
    if (isNew) {
      dev.goodSecs = 0;
      dev.visible = false;
      dev.hidden = false;
      dev.dopplerIdx = 0;
      dev.dopplerFilled = 0;
      dev.dopplerHz = 0.0f;
      dev.dopplerClass = DopplerClass::STATIC;
      dev.dopplerHpState = 0.0f;
      // Lazy PSRAM allocation for this device's Doppler buffers
      dev.dopplerBuf = (float*)heap_caps_malloc(kDopplerFrames * sizeof(float), MALLOC_CAP_SPIRAM);
      dev.dopplerFft = (float*)heap_caps_malloc(kDopplerFrames * 2 * sizeof(float), MALLOC_CAP_SPIRAM);
    }
    // Feed Doppler: HP filter then circular buffer
    if (dev.dopplerBuf) {
      dev.dopplerHpState = dev.dopplerHpState * 0.95f + meanAmp * 0.05f;
      float hpAmp = meanAmp - dev.dopplerHpState;
      dev.dopplerBuf[dev.dopplerIdx] = hpAmp;
      dev.dopplerIdx = (dev.dopplerIdx + 1) % kDopplerFrames;
      if (dev.dopplerFilled < kDopplerFrames) dev.dopplerFilled++;
    }
    memcpy(dev.mac, mac, 6);
    dev.rssi = rssi;
    dev.noise = noise;
    dev.channel = info->rx_ctrl.channel;
    dev.motion = gCsiMotion;
    dev.lastSeen = now;
    dev.pktCount++;
    dev.valid = true;
  }
}

static void enableCsi() {
  gCsiAmpIdx = 0; gCsiAmpFilled = 0;
  gCsiVarMax = 0.001f; gCsiVarMin = 0.0f;
  gCsiPhaVarMax = 0.001f; gCsiPhaVarMin = 0.0f;
  memset(gCsiAmpBuf, 0, sizeof(gCsiAmpBuf));
  memset(gCsiPhaBuf, 0, sizeof(gCsiPhaBuf));

  // 1. Enable promiscuous mode first
  esp_err_t err = esp_wifi_set_promiscuous(true);
  Serial.printf("# esp_wifi_set_promiscuous(true): %s\n", esp_err_to_name(err));

  // 2. Register CSI callback BEFORE enabling CSI
  err = esp_wifi_set_csi_rx_cb(csiCallback, nullptr);
  Serial.printf("# esp_wifi_set_csi_rx_cb: %s\n", esp_err_to_name(err));

  // 3. Configure CSI — disable channel_filter to avoid smoothing adjacent sub-carriers
  wifi_csi_config_t cfg = {};
  cfg.lltf_en = true;
  cfg.htltf_en = true;
  cfg.stbc_htltf2_en = true;
  cfg.ltf_merge_en = true;
  cfg.channel_filter_en = false;   // disable: keeps sub-carriers independent
  cfg.manu_scale = false;
  cfg.shift = 0;
  err = esp_wifi_set_csi_config(&cfg);
  Serial.printf("# esp_wifi_set_csi_config: %s\n", esp_err_to_name(err));

  // 4. Enable CSI
  err = esp_wifi_set_csi(true);
  Serial.printf("# esp_wifi_set_csi(true): %s\n", esp_err_to_name(err));

  // 5. Set filter to receive all packet types that carry CSI
  wifi_promiscuous_filter_t pf {};
  pf.filter_mask = WIFI_PROMIS_FILTER_MASK_MGMT | WIFI_PROMIS_FILTER_MASK_DATA;
  err = esp_wifi_set_promiscuous_filter(&pf);
  Serial.printf("# esp_wifi_set_promiscuous_filter: %s\n", esp_err_to_name(err));
}

// ── Calibration trigger (local, no UART peer needed) ─────────────────────────
static void startCalibration() {
  gCalibrating = true;
  gCalStartMs = millis();
  gCsiVarMax = 0.001f; gCsiVarMin = 0.0f;
  gCsiPhaVarMax = 0.001f; gCsiPhaVarMin = 0.0f;
  gCsiAmpIdx = 0; gCsiAmpFilled = 0;
  memset(gCsiAmpBuf, 0, sizeof(gCsiAmpBuf));
  memset(gCsiPhaBuf, 0, sizeof(gCsiPhaBuf));
  radar.calibrate();   // also send to UART peer if one exists
}

// ── CSI init ─────────────────────────────────────────────────────────────────
#if defined(FREENOVE_FNK0104B)
// static void wifiEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
//   if (event_id == IP_EVENT_STA_GOT_IP) {
//     auto* ev = (ip_event_got_ip_t*)event_data;
//     snprintf(gIpStr, sizeof(gIpStr), "%d.%d.%d.%d", IP2STR(&ev->ip_info.ip));
//     Serial.printf("# STA IP: %s\n", gIpStr);
//     initDoppler();
//     enableCsi();
//     wifiReady = true;
//   } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
//     Serial.printf("# WiFi disconnected\n");
//   }
// }

static void wifi_event_handler(void* event_handler_arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {

  static uint8_t s_retry_num = 0;

  if (event_base == WIFI_EVENT) {

    if (event_id == WIFI_EVENT_STA_START) {
      Serial.println("Event: WiFi Started");
      ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
    } else if (event_id == WIFI_EVENT_STA_CONNECTED) {
      Serial.println("Event: WiFi Connected");
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
      wifi_event_sta_disconnected_t* event = (wifi_event_sta_disconnected_t*)event_data;
      Serial.printf("Disconnected from %s, reason: %d\n", event->ssid, event->reason);
      Serial.println("Event: WiFi Lost Connection");
      if (s_retry_num < 5) {
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
        s_retry_num++;
        Serial.printf("Event Action: Retry to connect to the AP %d\n", s_retry_num);
      }
    } else if (event_id == WIFI_EVENT_STA_STOP) {
      Serial.println("Event: WiFi Stopped");
    } else if (event_id == WIFI_EVENT_AP_START) {
      Serial.println("Event: SoftAP Started");
      ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_connect());
    } else if (event_id == WIFI_EVENT_AP_STOP) {
      Serial.println("Event: SoftAP Stopped");
    } else if (event_id == WIFI_EVENT_AP_STACONNECTED) {
      Serial.println("Event: AP Client Connected");
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
      Serial.println("Event: AP Client Disconnected");
    } else {
      Serial.printf("Event: WiFi threw unidentified code %d\n", event_id);
    }

  } else if (event_base == IP_EVENT) {

    if (event_id == IP_EVENT_STA_GOT_IP) {
      Serial.println("Event: WiFi Got IP");
      ip_event_got_ip_t* event = (ip_event_got_ip_t*)event_data;
      snprintf(gIpStr, sizeof(gIpStr), "%d.%d.%d.%d", IP2STR(&event->ip_info.ip));
      Serial.printf("# STA IP: %s\n", gIpStr);
      s_retry_num = 0;
      delay(100);
      initDoppler();
      delay(100);
      enableCsi();
      delay(100);
      wifiReady = true;
    }

  }
}

static void initCsi() {
  return;
  // Event loop
  // esp_err_t err = esp_event_loop_create_default();
  // if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);

  // // Register handler BEFORE starting WiFi so we catch the GOT_IP event
  // ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifiEventHandler, nullptr));
  // ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_STA_DISCONNECTED, &wifiEventHandler, nullptr));

  // // WiFi init
  // wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  // ESP_ERROR_CHECK(esp_wifi_init(&cfg));
  // ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

  // wifi_config_t sta_cfg = {};
  // strncpy((char*)sta_cfg.sta.ssid, WIFI_SSID, sizeof(sta_cfg.sta.ssid));
  // strncpy((char*)sta_cfg.sta.password, WIFI_PASS, sizeof(sta_cfg.sta.password));
  // ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg));
  // ESP_ERROR_CHECK(esp_wifi_start());
  // ESP_ERROR_CHECK(esp_wifi_connect());

  // Pump the event loop until we have an IP (~1-3 sec typical)
  // Serial.printf("# Waiting for IP");
  // uint32_t start = millis();
  // while (!wifiReady) {
  //   delay(50);
  //   Serial.printf(".");
  //   if (millis() - start > 15000) {
  //     Serial.printf("\n# IP acquisition timed out\n");
  //     return;
  //   }
  // }
}

// ── CSI service ───────────────────────────────────────────────────────────────
static void serviceCsi() {
  static uint32_t seq = 0;
  static uint32_t last = 0;
  static int      holdCnt = 0;
  static float    heldMot = 0.0f;
  const  int     kHold = 150;   // ~10 s at 15 Hz

  uint32_t now = millis();
  if (now - last < 66) return;
  last = now;

  // Handle calibration countdown
  if (gCalibrating) {
    if (now - gCalStartMs >= kCalDurationMs) {
      gCalibrating = false;
    }
    // During calibration: inject CAL frames but don't update gCsiMotion range
    char line[48];
    snprintf(line, sizeof(line), "R,%lu,0,0.000,%d,CAL",
      (unsigned long)(++seq), (int)gCsiRssi);
    radar.injectLine(line);
    return;
  }

  float m = gCsiMotion;
  bool present;
  if (m > gThreshold) {
    holdCnt = kHold; heldMot = m; present = true;
  } else if (holdCnt > 0) {
    holdCnt--;
    float fade = (float)holdCnt / kHold;
    m = heldMot * (0.10f + 0.90f * fade);
    present = true;
  } else {
    present = false; m = 0.0f;
  }

  char line[48];
  snprintf(line, sizeof(line), "R,%lu,%d,%.3f,%d,RUN",
    (unsigned long)(++seq), (int)present, m, (int)gCsiRssi);
  radar.injectLine(line);
}

// ── Battery icon (template for both boards) ──────────────────────────────────
template<typename T>
static void drawBatIcon(T& c, int bx, int by) {
  int level = -1;
  bool chg = false;
  #if !defined(FREENOVE_FNK0104B)
  level = (int)M5.Power.getBatteryLevel();
  chg = (M5.Power.isCharging() == 1);
  #endif
  uint16_t col;
  if (level < 0)    col = c.color565(100, 100, 100);
  else if (chg)          col = c.color565(0, 220, 255);
  else if (level > 50)   col = c.color565(0, 200, 80);
  else if (level > 20)   col = c.color565(220, 160, 0);
  else                   col = c.color565(220, 30, 30);
  c.drawRect(bx, by, 14, 7, col);
  c.fillRect(bx + 14, by + 2, 2, 3, col);
  if (level > 0) {
    int fill = (level * 12 + 50) / 100;
    if (fill > 12) fill = 12;
    c.fillRect(bx + 1, by + 1, fill, 5, col);
  }
  if (chg) {
    c.drawFastHLine(bx + 4, by + 3, 5, c.color565(255, 255, 255));
    c.drawFastVLine(bx + 6, by + 1, 5, c.color565(255, 255, 255));
  }
}

// ── Settings menu ─────────────────────────────────────────────────────────────
static void menuAdjust(int dir) {
  switch (gMenuCursor) {
  case 0:
    gColorIdx = (gColorIdx + kNumPalettes + (uint8_t)(dir > 0 ? 1 : kNumPalettes - 1)) % kNumPalettes;
    applyPalette(gColorIdx);
    break;
    #if !defined(FREENOVE_FNK0104B)
  case 1:
    gBright = (uint8_t)constrain((int)gBright + (dir > 0 ? 10 : -10), 10, 100);
    M5Cardputer.Display.setBrightness((uint32_t)gBright * 255 / 100);
    break;
  case 2:
    gExtBright = (uint8_t)constrain((int)gExtBright + (dir > 0 ? 10 : -10), 10, 100);
    break;
    #endif
  default: break;
  }
}
#endif

#if !defined(FREENOVE_FNK0104B)
static void drawMenu() {
  const int W = canvas.width(), H = canvas.height();
  const uint16_t cBg = canvas.color565(4, 0, 8);
  const uint16_t cBar = canvas.color565(20, 0, 35);
  const uint16_t cDim = canvas.color565(70, 70, 70);
  const uint16_t cSep = canvas.color565(35, 0, 50);

  canvas.fillSprite(cBg);
  canvas.fillRect(0, 0, W, 14, cBar);
  canvas.drawFastHLine(0, 13, W, gColA);
  canvas.setTextSize(1); canvas.setTextColor(gColA, cBar);
  canvas.drawString(">> SETTINGS <<", (W - canvas.textWidth(">> SETTINGS <<")) / 2, 3);

  const char* const labels[] = { "PALETTE", "SCREEN", "EXT PANEL" };
  const int numItems = 3, rowH = 20, textOff = 6;
  for (int i = 0; i < numItems; i++) {
    int  y = 16 + i * rowH;
    bool sel = (i == (int)gMenuCursor);
    canvas.setTextColor(sel ? gColA : TFT_BLACK, cBg);
    canvas.drawString(">", 4, y + textOff);
    canvas.setTextColor(sel ? TFT_WHITE : cDim, cBg);
    canvas.drawString(labels[i], 16, y + textOff);
    char val[20];
    switch (i) {
    case 0: snprintf(val, sizeof(val), "%s", kPalettes[gColorIdx].name); break;
    case 1: snprintf(val, sizeof(val), "%d%%", (int)gBright); break;
    case 2: snprintf(val, sizeof(val), "%d%%", (int)gExtBright); break;
    default: val[0] = '\0'; break;
    }
    canvas.setTextColor(sel ? gColB : cDim, cBg);
    canvas.drawString(val, W - canvas.textWidth(val) - 8, y + textOff);
    if (i < numItems - 1) canvas.drawFastHLine(8, y + rowH - 2, W - 16, cSep);
  }

  canvas.fillRect(0, H - 14, W, 14, cBar);
  canvas.drawFastHLine(0, H - 14, W, gColA);
  canvas.setTextColor(cDim, cBar);
  canvas.drawString(";/.:nav  ,//:change  `:save", (W - canvas.textWidth(";/.:nav  ,//:change  `:save")) / 2, H - 11);
  canvas.pushSprite(0, 0);
}
#endif // !FREENOVE

// ── Cardputer bottom screen ───────────────────────────────────────────────────
#if !defined(FREENOVE_FNK0104B)
static void drawBottom() {
  const auto& s = radar.state();
  const int W = canvas.width();
  const int H = canvas.height();
  const bool linkOk = !radar.stale(750);
  const bool calib = (strcmp(s.mode, "CAL") == 0);
  const bool present = linkOk && s.presence;

  const uint16_t cMag = gColA;
  const uint16_t cCyan = gColB;
  const uint16_t cPurple = canvas.color565(70, 0, 70);
  const uint16_t cBar = canvas.color565(20, 0, 35);
  const uint16_t cBorder = canvas.color565(160, 0, 160);
  const uint16_t cCorner = canvas.color565(255, 120, 255);

  canvas.fillSprite(TFT_BLACK);
  canvas.fillRect(0, 0, W, 14, cBar);
  canvas.drawFastHLine(0, 13, W, cBorder);
  canvas.setTextSize(1);
  canvas.setTextColor(cMag, cBar);

  const char* title = "[ CSI-RADAR ]";
  canvas.setTextColor(cCyan, cBar);
  canvas.drawString(title, (W - canvas.textWidth(title)) / 2, 3);
  canvas.setTextColor(cMag, cBar);

  drawBatIcon(canvas, 2, 3);
  const char* tag = !linkOk ? "NO LINK" : (calib ? "CAL" : s.mode);
  uint16_t    pill = !linkOk ? TFT_RED : (calib ? canvas.color565(255, 140, 0) : canvas.color565(0, 180, 80));
  int tw = canvas.textWidth(tag) + 8;
  canvas.fillRoundRect(W - tw - 3, 1, tw, 11, 3, pill);
  canvas.setTextColor(TFT_WHITE, pill);
  canvas.drawString(tag, W - tw + 1, 3);

  uint16_t bg = present ? canvas.color565(60, 0, 60) : canvas.color565(0, 0, 25);
  uint16_t border = present ? cMag : cPurple;
  canvas.fillRoundRect(4, 17, W - 8, 34, 4, bg);
  canvas.drawRoundRect(4, 17, W - 8, 34, 4, border);
  canvas.setTextSize(2);
  const char* lbl = present ? ">> CONTACT <<" : "~~ CLEAR ~~";
  canvas.setTextColor(present ? canvas.color565(255, 100, 255) : cCyan, bg);
  canvas.drawString(lbl, (W - canvas.textWidth(lbl)) / 2, 25);

  canvas.setTextSize(1);
  const int gx = 4, gy = 54, gw = W - 8, gh = 50;
  canvas.drawRect(gx, gy, gw, gh, cBorder);
  int ty = gy + gh - 1 - (int)(gThreshold * (gh - 2));
  canvas.drawFastHLine(gx + 1, ty, gw - 2, canvas.color565(180, 0, 180));

  int cols = gw - 2, n = RadarLink::historySize();
  if (cols > n) cols = n;
  int xoff = (gw - 2) - cols;
  for (int i = 0; i < cols; ++i) {
    float v = radar.historyAt(n - cols + i);
    int barh = (int)(v * (gh - 2));
    if (barh < 0) barh = 0;
    if (barh > gh - 2) barh = gh - 2;
    if (barh == 0) continue;
    uint16_t col = (v > gThreshold)
      ? canvas.color565(220, 0, 200)
      : canvas.color565(0, 160, 200);
    canvas.drawFastVLine(gx + 1 + xoff + i, gy + gh - 1 - barh, barh, col);
  }

  char foot[48];
  snprintf(foot, sizeof(foot), "mot %3d%%  rssi %ddBm  thr %2d%%",
    (int)(s.motion * 100), s.rssi, (int)(gThreshold * 100));
  canvas.setTextColor(cCyan, TFT_BLACK);
  canvas.drawString(foot, 4, gy + gh + 4);
  canvas.setTextColor(cPurple, TFT_BLACK);
  canvas.drawString(",.mode [c]cal  [ [ ]thr-  [ ] ]thr+", 4, gy + gh + 15);

  canvas.drawFastHLine(0, 0, W, cBorder);
  canvas.drawFastHLine(0, H - 1, W, cBorder);
  canvas.drawFastVLine(0, 0, H, cBorder);
  canvas.drawFastVLine(W - 1, 0, H, cBorder);
  canvas.fillRect(0, 0, 2, 2, cCorner);
  canvas.fillRect(W - 2, 0, 2, 2, cCorner);
  canvas.fillRect(0, H - 2, 2, 2, cCorner);
  canvas.fillRect(W - 2, H - 2, 2, 2, cCorner);

  canvas.pushSprite(0, 0);
}
#endif // !FREENOVE

// ── Shared blip/ripple state ─────────────────────────────────────────────────
struct Blip { float ang; float rad; float strength; uint32_t birth; bool active; };
static Blip           blips[12];
static uint32_t       lastSpawn = 0;
static float          gLastSpawnAng = 0.0f;
static const uint32_t BLIP_LIFE = 15000;
struct Ripple { float ang; float rad; uint32_t birth; bool active; };
static Ripple         ripples[6];
static float          gBEAR = 0.0f;
static float          gLastBrg = 0.0f;

static float pickBlipAngle() {
  const float TAU = 6.2831853f;
  float r = ((float)random(0, 10000) / 10000.0f) * (TAU * 2.0f / 3.0f);
  const float s1 = TAU / 6.0f, s2 = TAU / 3.0f;
  if (r < s1)       return r;
  else if (r < s1 + s2)  return r - s1 + TAU / 3.0f;
  else                   return r - s1 - s2 + TAU * 5.0f / 6.0f;
}

// ── Matrix rain (shared) ─────────────────────────────────────────────────────
static const uint8_t RAIN_X[14] = { 3,9,15,21,27,33,39, 201,207,213,219,225,231,237 };
struct RainDrop { int16_t y; uint8_t speed; uint8_t tick; };
static RainDrop rain[14];
static bool     rainReady = false;

template<typename T>
static void drawRain(T& c, uint32_t now) {
  static const char GL[] = "0123456789ABCDEF!:;@#%";
  static const int  GLN = 22;
  static const int  TRAIL = 7;
  if (!rainReady) {
    for (int i = 0; i < 14; i++) {
      rain[i].y = (int16_t)random(-60, 160);
      rain[i].speed = 2 + (uint8_t)random(0, 3);
      rain[i].tick = (uint8_t)random(0, 4);
    }
    rainReady = true;
  }
  c.setTextSize(1);
  for (int i = 0; i < 14; i++) {
    if (++rain[i].tick >= rain[i].speed) {
      rain[i].tick = 0;
      rain[i].y += 8;
      if (rain[i].y > 180 + TRAIL * 8) {
        rain[i].y = -(int16_t)random(0, 60);
        rain[i].speed = 2 + (uint8_t)random(0, 3);
      }
    }
    for (int j = TRAIL - 1; j >= 0; j--) {
      int16_t ry = rain[i].y - j * 8;
      if (ry < 0 || ry >= 180) continue;
      char buf[2] = { GL[((uint32_t)(i * 13 + j * 7) + now / 350) % GLN], 0 };
      bool isCyan = (i % 2) == 0;
      if (j == 0) {
        c.setTextColor(isCyan ? c.color565(120, 255, 255) : c.color565(220, 200, 255), TFT_BLACK);
      } else {
        uint8_t b = (uint8_t)(210 * (TRAIL - j) / TRAIL);
        c.setTextColor(isCyan ? c.color565(0, b / 2, b) : c.color565(b / 2, 0, b), TFT_BLACK);
      }
      c.drawString(buf, RAIN_X[i], ry);
    }
  }
}

// ── Radar scope (shared drawing logic, both boards) ─────────────────────────
static void updateBlipsFromRadar(bool present, int rssi, float motion) {
  if (!present) return;
  const float R = 74.0f;
  float t = ((float)rssi + 45.0f) / (-33.0f);
  if (t < 0.0f) t = 0.0f; if (t > 1.0f) t = 1.0f;
  float targetRad = R * (0.25f + t * 0.65f);
  if (targetRad < R * 0.30f) targetRad = R * 0.30f;
  const float mergeThresh = R * 0.20f;

  int   closestIdx = -1;
  float closestDist = R;
  for (int i = 0; i < 12; ++i) {
    if (!blips[i].active || blips[i].birth + BLIP_LIFE <= (uint32_t)millis()) continue;
    blips[i].strength += (motion - blips[i].strength) * 0.12f;
    float d = fabsf(blips[i].rad - targetRad);
    if (d < closestDist) { closestDist = d; closestIdx = i; }
  }

  const float TAU = 6.2831853f;
  uint32_t now = millis();
  if (closestIdx >= 0 && closestDist <= mergeThresh) {
    blips[closestIdx].birth = now;
    blips[closestIdx].rad += (targetRad - blips[closestIdx].rad) * 0.05f;
  } else if (now - lastSpawn > 800) {
    int slot = -1;
    for (int i = 0; i < 12; ++i) { if (!blips[i].active) { slot = i; break; } }
    if (slot < 0) {
      uint32_t oldest = UINT32_MAX;
      for (int i = 0; i < 12; ++i) if (blips[i].birth < oldest) { oldest = blips[i].birth; slot = i; }
    }
    if (slot >= 0) {
      gLastSpawnAng = pickBlipAngle();
      blips[slot].ang = gLastSpawnAng;
      blips[slot].rad = targetRad;
      blips[slot].strength = motion;
      blips[slot].birth = now;
      blips[slot].active = true;
      lastSpawn = now;
      gLastBrg = fmodf((gLastSpawnAng + TAU / 4.0f) * (360.0f / TAU) + 360.0f, 360.0f);
      for (int r = 0; r < 6; ++r) {
        if (!ripples[r].active) { ripples[r] = { gLastSpawnAng, targetRad, now, true }; break; }
      }
    }
  }
}

template<typename T>
static void drawScope(T& c, int cx, int cy, int R, uint32_t now, bool cal) {
  const float TAU = 6.2831853f;
  const uint16_t cCyan = c.color565(0, 200, 220);
  const uint16_t cPurple = c.color565(70, 0, 70);
  const uint16_t cBorder = c.color565(160, 0, 160);
  const uint16_t scopeBg = c.color565(4, 0, 8);

  c.fillSprite(TFT_BLACK);
  #if !defined(FREENOVE_FNK0104B)
  drawRain(c, now);
  #endif

  c.fillCircle(cx, cy, R, scopeBg);
  c.drawCircle(cx, cy, R, c.color565(0, 80, 100));
  c.drawCircle(cx, cy, R * 2 / 3, c.color565(80, 0, 100));
  c.drawCircle(cx, cy, R / 3, c.color565(0, 50, 80));

  const uint16_t cXhair = c.color565(35, 0, 35);
  float nsAng = -TAU / 4.0f;
  c.drawLine(cx + (int)(R * cosf(nsAng)), cy + (int)(R * sinf(nsAng)),
    cx + (int)(R * cosf(nsAng + TAU / 2)), cy + (int)(R * sinf(nsAng + TAU / 2)), cXhair);
  c.drawLine(cx + (int)(R * cosf(0.0f)), cy + (int)(R * sinf(0.0f)),
    cx + (int)(R * cosf(TAU / 2.0f)), cy + (int)(R * sinf(TAU / 2.0f)), cXhair);
  for (int d = 0; d < 12; ++d) {
    float a = d * (TAU / 12.0f);
    c.drawLine(cx + (int)((R - 5) * cosf(a)), cy + (int)((R - 5) * sinf(a)),
      cx + (int)(R * cosf(a)), cy + (int)(R * sinf(a)), cPurple);
  }
  c.setTextSize(1);
  const char* cLbl[4] = { "N", "E", "S", "W" };
  const float cBase[4] = { -TAU / 4.0f, 0.0f, TAU / 4.0f, TAU / 2.0f };
  for (int ci = 0; ci < 4; ci++) {
    float a = cBase[ci];
    int lx = cx + (int)((R - 11) * cosf(a)) - 2;
    int ly = cy + (int)((R - 11) * sinf(a)) - 4;
    c.setTextColor(ci == 0 ? TFT_WHITE : cCyan, scopeBg);
    c.drawString(cLbl[ci], lx, ly);
  }
  c.setTextColor(c.color565(45, 0, 55), scopeBg);
  c.drawString("1", cx + R / 3 + 1, cy - 8);
  c.drawString("2", cx + R * 2 / 3 + 1, cy - 8);
  c.drawString("3", cx + R + 1, cy - 8);

  // Sweep
  float sweep = (float)(now % 6000) / 6000.0f * TAU;
  const int TRAIL = 22;
  for (int k = TRAIL; k >= 1; --k) {
    float a = sweep - k * 0.040f;
    float t = 1.0f - (float)k / TRAIL;
    float b2 = t * t;
    if (cal) {
      // Amber sweep during calibration
      uint8_t rv = (uint8_t)(255 * b2);
      uint8_t gv = (uint8_t)(140 * b2);
      c.drawLine(cx, cy, cx + (int)(R * cosf(a)), cy + (int)(R * sinf(a)),
        c.color565(rv, gv, 0));
    } else {
      uint8_t r = (uint8_t)(220 * b2);
      uint8_t bl = (uint8_t)(40 + 160 * b2);
      c.drawLine(cx, cy, cx + (int)(R * cosf(a)), cy + (int)(R * sinf(a)),
        c.color565(r, 0, bl));
    }
  }
  c.drawLine(cx, cy, cx + (int)(R * cosf(sweep)), cy + (int)(R * sinf(sweep)),
    cal ? c.color565(255, 200, 0) : c.color565(255, 80, 255));

  // Blips
  for (int i = 0; i < 12; ++i) {
    if (!blips[i].active) continue;
    uint32_t age = now - blips[i].birth;
    if (age > BLIP_LIFE) { blips[i].active = false; continue; }
    float fade = 1.0f - (float)age / BLIP_LIFE;
    int bx = cx + (int)(blips[i].rad * cosf(blips[i].ang));
    int by = cy + (int)(blips[i].rad * sinf(blips[i].ang));
    int sz = 2 + (int)(blips[i].strength * 4);
    float str = blips[i].strength;
    uint16_t col;
    if (str > 0.85f) col = c.color565((uint8_t)(255 * fade), (uint8_t)(180 * fade), (uint8_t)(255 * fade));
    else if (str > 0.70f) col = c.color565((uint8_t)(255 * fade), 0, (uint8_t)(200 * fade));
    else if (str > 0.50f) col = c.color565((uint8_t)(140 * fade), 0, (uint8_t)(255 * fade));
    else                  col = c.color565(0, (uint8_t)(200 * fade), (uint8_t)(255 * fade));
    c.fillCircle(bx, by, sz, col);
    if (fade > 0.6f) c.drawCircle(bx, by, sz + 2, col);
  }

  // Ripples
  for (int r = 0; r < 6; ++r) {
    if (!ripples[r].active) continue;
    uint32_t age = now - ripples[r].birth;
    if (age > 700) { ripples[r].active = false; continue; }
    float prog = (float)age / 700.0f;
    int bx = cx + (int)(ripples[r].rad * cosf(ripples[r].ang));
    int by = cy + (int)(ripples[r].rad * sinf(ripples[r].ang));
    int sz = 4 + (int)(prog * 22.0f);
    uint8_t fade = (uint8_t)(255 * (1.0f - prog));
    uint8_t rc = (uint8_t)(fade * (1.0f - prog));
    c.drawCircle(bx, by, sz, c.color565(rc, fade, fade));
    c.drawCircle(bx, by, sz + 3, c.color565(rc / 3, fade / 3, fade / 3));
  }
}

// ── Cardputer top screen (radar scope) ────────────────────────────────────────
#if !defined(FREENOVE_FNK0104B)
static void drawTop() {
  if (!extReady) return;
  const auto& s = radar.state();
  const bool linkOk = !radar.stale(750);
  const bool present = linkOk && s.presence;
  const uint32_t now = millis();

  const int W = topCanvas.width();   // 240
  const int H = topCanvas.height();  // 180
  const int cx = 120, cy = 90, R = 74;

  updateBlipsFromRadar(present, s.rssi, s.motion);
  drawScope(topCanvas, cx, cy, R, now, strcmp(s.mode, "CAL") == 0);

  // Title bar
  const uint16_t cBar = topCanvas.color565(20, 0, 35);
  const uint16_t cBorder = topCanvas.color565(160, 0, 160);
  topCanvas.fillRect(0, 0, W, 14, cBar);
  topCanvas.drawFastHLine(0, 13, W, cBorder);
  topCanvas.setTextSize(1);
  const char* title = "[ WiFi-CSI RADAR ]";
  topCanvas.setTextColor(gColA, cBar);
  topCanvas.drawString(title, (W - topCanvas.textWidth(title)) / 2, 3);
  drawBatIcon(topCanvas, 2, 3);
  const char* modeStr = "CSI OK";
  topCanvas.setTextColor(topCanvas.color565(0, 210, 80), cBar);
  topCanvas.drawString(modeStr, W - topCanvas.textWidth(modeStr) - 4, 3);

  // Status bar
  bool blink = (now % 600) < 300;
  topCanvas.fillRect(0, H - 14, W, 14, cBar);
  topCanvas.drawFastHLine(0, H - 14, W, cBorder);
  const char* ctlbl = present ? ">>CONTACT<<" : " scanning.. ";
  uint16_t ctCol = present
    ? (blink ? topCanvas.color565(255, 60, 255) : topCanvas.color565(110, 0, 100))
    : topCanvas.color565(55, 0, 70);
  topCanvas.setTextColor(ctCol, cBar);
  topCanvas.drawString(ctlbl, 14, H - 11);
  char midStr[14];
  if (present) snprintf(midStr, sizeof(midStr), "BRG:%03.0f", gLastBrg);
  else         snprintf(midStr, sizeof(midStr), "T:%d%%", (int)(gThreshold * 100));
  topCanvas.setTextColor(present ? topCanvas.color565(255, 140, 60) : topCanvas.color565(100, 0, 110), cBar);
  topCanvas.drawString(midStr, (W - topCanvas.textWidth(midStr)) / 2, H - 11);
  int contacts = 0;
  for (int i = 0; i < 12; i++) if (blips[i].active && (now - blips[i].birth) < BLIP_LIFE) contacts++;
  char stats[28];
  snprintf(stats, sizeof(stats), "C:%d M:%d%% %ddBm", contacts, (int)(s.motion * 100), s.rssi);
  topCanvas.setTextColor(gColB, cBar);
  topCanvas.drawString(stats, W - topCanvas.textWidth(stats) - 4, H - 11);

  // Frame + corners
  const uint16_t cCorner = topCanvas.color565(255, 120, 255);
  topCanvas.drawFastHLine(0, 0, W, cBorder);
  topCanvas.drawFastHLine(0, H - 1, W, cBorder);
  topCanvas.drawFastVLine(0, 0, H, cBorder);
  topCanvas.drawFastVLine(W - 1, 0, H, cBorder);
  topCanvas.fillRect(0, 0, 2, 2, cCorner);
  topCanvas.fillRect(W - 2, 0, 2, 2, cCorner);
  topCanvas.fillRect(0, H - 2, 2, 2, cCorner);
  topCanvas.fillRect(W - 2, H - 2, 2, 2, cCorner);

  topCanvas.pushRotateZoom(160, 120, 0.0f, EXT_ZOOM, EXT_ZOOM);
}
#endif // !FREENOVE

// ── Device list helpers ───────────────────────────────────────────────────

static void ageOutDevices() {
  uint32_t now = millis();
  for (int i = 0; i < kMaxDevices; i++) {
    if (!gDevices[i].valid) continue;
    if (now - gDevices[i].lastSeen > 10000) {
      gDevices[i].valid = false;
      gDeviceListDirty = true;
    }
  }
}

static void cycleVizMode() {
  gVizMode = (VizMode)((int)gVizMode + 1);
  if (gVizMode >= (VizMode)kVizModeCount) gVizMode = (VizMode)0;
}

// Called every ~second from loop() to drain pktCount → pktRate and manage hide/unhide.
static void drainPktRates(uint32_t now) {
  static uint32_t lastPpsSec = 0;
  if (now - lastPpsSec < 1000) return;
  lastPpsSec = now;
  for (int i = 0; i < kMaxDevices; i++) {
    if (!gDevices[i].valid) continue;
    gDevices[i].pktRate = gDevices[i].pktCount;
    gDevices[i].pktCount = 0;

    bool wasHidden = gDevices[i].hidden;

    if (gDevices[i].pktRate == 0) {
      gDevices[i].lowPpsSecs++;
      if (gDevices[i].lowPpsSecs >= kHideAfterLowPpsSecs) {
        gDevices[i].hidden = true;
      }
    } else {
      gDevices[i].lowPpsSecs = 0;
      if (gDevices[i].hidden && !gDevices[i].lowRssiSecs) {
        gDevices[i].hidden = false;  // PPS recovered; RSSI must still be failing to keep hidden
      }
    }
    if (gDevices[i].rssi <= -80) {
      gDevices[i].lowRssiSecs++;
      if (gDevices[i].lowRssiSecs >= kHideAfterLowRssiSecs) {
        gDevices[i].hidden = true;
      }
    } else {
      gDevices[i].lowRssiSecs = 0;
      if (gDevices[i].hidden && !gDevices[i].lowPpsSecs) {
        gDevices[i].hidden = false;  // RSSI recovered; PPS must still be failing to keep hidden
      }
    }

    // Only mark dirty on actual hidden-state transitions (not every call)
    if (wasHidden != gDevices[i].hidden) {
      gDeviceListDirty = true;
    }

    // Reveal: device must pass both thresholds for 10 s before showing
    if (gDevices[i].pktRate > 0 && gDevices[i].rssi > -80) {
      gDevices[i].goodSecs++;
      if (!gDevices[i].visible && gDevices[i].goodSecs >= 10) {
        gDevices[i].visible = true;
        gDeviceListDirty = true;
      }
    } else {
      gDevices[i].goodSecs = 0;
    }
  }
}

// Returns total visible device count for status bar.
static int drawDeviceList() {
  const int W = canvas.width();
  const int H = 240;
  const int RH = 15;
  const int N = H / RH;

  canvas.fillSprite(TFT_BLACK);
  uint32_t now = millis();

  // Re-sort only when dirty
  if (gDeviceListDirty) {
    int n = 0;
    for (int i = 0; i < kMaxDevices; i++) {
      if (!gDevices[i].valid) continue;
      if (now - gDevices[i].lastSeen > 10000) { gDevices[i].valid = false; continue; }
      if (!gDevices[i].visible) continue;
      if (gDevices[i].hidden) continue;
      memcpy(_sorted[n].mac, gDevices[i].mac, 6);
      _sorted[n].rssi = gDevices[i].rssi;
      _sorted[n].noise = gDevices[i].noise;
      _sorted[n].motion = gDevices[i].motion;
      _sorted[n].channel = gDevices[i].channel;
      _sorted[n].lastSeen = gDevices[i].lastSeen;
      _sorted[n].pktRate = gDevices[i].pktRate;
      _sorted[n].dopplerClass = gDevices[i].dopplerClass;
      _sorted[n].visible = true;
      _sorted[n].valid = true;
      n++;
    }
    for (int i = 1; i < n; i++) {
      DeviceEntry key = _sorted[i];
      int j = i - 1;
      while (j >= 0 && _sorted[j].rssi < key.rssi) {
        _sorted[j + 1] = _sorted[j];
        j--;
      }
      _sorted[j + 1] = key;
    }
    _sortedCount = n;
    gDeviceListDirty = false;
  }

  int totalCount = 0;
  for (int i = 0; i < kMaxDevices; i++) {
    if (gDevices[i].valid && gDevices[i].visible && now - gDevices[i].lastSeen <= 10000 && !gDevices[i].hidden)
      totalCount++;
  }

  canvas.setTextSize(1);
  for (int row = 0; row < N; row++) {
    int y = row * RH;
    if ((row & 1) == 0) canvas.fillRect(0, y, W, RH, canvas.color565(0, 0, 10));
    if (row >= _sortedCount) continue;

    const DeviceEntry* d = &_sorted[row];

    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
      d->mac[5], d->mac[4], d->mac[3], d->mac[2], d->mac[1], d->mac[0]);

    uint16_t macCol;
    if (d->rssi > -50) macCol = canvas.color565(0, 220, 80);
    else if (d->rssi > -65) macCol = canvas.color565(180, 200, 40);
    else if (d->rssi > -80) macCol = canvas.color565(220, 140, 0);
    else                    macCol = canvas.color565(200, 50, 50);

    canvas.setTextColor(macCol, TFT_BLACK);
    canvas.drawString(macStr, 0, y + 3);

    const int BAR_W = 10;
    int filled = (int)(d->motion * BAR_W);
    if (filled < 0) filled = 0;
    if (filled > BAR_W) filled = BAR_W;
    char barStr[BAR_W + 1];
    for (int i = 0; i < BAR_W; i++) barStr[i] = (i < filled) ? '\xDB' : '\xB1';
    barStr[BAR_W] = '\0';

    //canvas.setTextColor(barCol, TFT_BLACK);
    //canvas.drawString(barStr, 120, y + 3);

    // Doppler classification text (per-device)
    const char* dopplerTxt = (d->dopplerClass == DopplerClass::HUMAN) ? "HUMAN" : "STATIC";
    uint16_t dopplerCol = (d->dopplerClass == DopplerClass::HUMAN)
      ? canvas.color565(0, 220, 80)
      : canvas.color565(60, 60, 80);
    canvas.setTextColor(dopplerCol, TFT_BLACK);
    canvas.drawString(dopplerTxt, 120, y + 3);

    char statStr[16];
    snprintf(statStr, sizeof(statStr), "%4ddB %4dp/s", (int)d->rssi, (int)d->pktRate);
    canvas.setTextColor(canvas.color565(100, 100, 120), TFT_BLACK);
    canvas.drawString(statStr, W - canvas.textWidth(statStr) - 2, y + 3);
  }

  canvas.drawFastHLine(0, 239, W, canvas.color565(60, 0, 60));
  return totalCount;
}

// ── Freenove full-screen radar ────────────────────────────────────────────────
#if defined(FREENOVE_FNK0104B)
static void drawRadar() {
  if (!displayReady) return;
  const auto& s = radar.state();
  const bool linkOk = !radar.stale(750);
  const bool present = linkOk && s.presence;
  const uint32_t now = millis();

  // Full canvas black
  canvas.fillSprite(TFT_BLACK);

  int deviceCount = 0;
  if (gVizMode == VizMode::DEVICE_LIST) {
    deviceCount = drawDeviceList();
  } else if (gVizMode == VizMode::FFT_LINES) {
    // ── FFT multi-line viz: top 240px × 240px, 11 lines × 21px (20px bar + 1px gap) ──
    const int BAR_W = 6;
    const int GAP = 1;
    const int LINE_H = 20;
    const int LINE_GAP = 1;
    const int STEP = LINE_H + LINE_GAP;   // 21
    const int NBARS = 32;
    const int START_X = 1;
    const int MAX_LINES = 240 / STEP;     // 11
    const int BG_COL = canvas.color565(0, 0, 10);
    const int BORDER_COL = canvas.color565(60, 0, 60);

    // Collect up to MAX_LINES devices with FFT data, sorted by RSSI
    const DeviceEntry* sorted[MAX_LINES];
    int n = 0;
    for (int i = 0; i < kMaxDevices && n < MAX_LINES; i++) {
      if (!gDevices[i].valid || !gDevices[i].visible || gDevices[i].hidden) continue;
      if (gDevices[i].dopplerFilled < kDopplerFrames || !gDevices[i].dopplerFft) continue;
      // Insert by RSSI descending
      int pos = n;
      for (int j = 0; j < n; j++) {
        if (gDevices[i].rssi > sorted[j]->rssi) { pos = j; break; }
      }
      for (int k = n; k > pos; k--) sorted[k] = sorted[k - 1];
      sorted[pos] = &gDevices[i];
      n++;
    }

    for (int line = 0; line < MAX_LINES; line++) {
      int y = line * STEP;
      canvas.fillRect(0, y, 240, LINE_H, BG_COL);

      if (line < n && sorted[line]->dopplerFft) {
        const DeviceEntry* d = sorted[line];

        // Find peak magnitude for normalization (noise-subtracted if EMA mode)
        float mag32[32] = {0};
        float maxMag = 0.0f;
        for (int j = 1; j < NBARS; j++) {
          float re = d->dopplerFft[j * 2];
          float im = d->dopplerFft[j * 2 + 1];
          float mag = sqrtf(re * re + im * im);
          if (gThreshold == 0.0f) {
            mag = mag - d->dopplerNoiseFloor[j];  // EMA noise subtraction
            if (mag < 0.0f) mag = 0.0f;
          }
          mag32[j] = mag;
          if (mag > maxMag) maxMag = mag;
        }
        float scale = (maxMag > 0.0f) ? (float)LINE_H / maxMag : 0.0f;

        for (int j = 1; j < NBARS; j++) {
          int x = START_X + (j - 1) * (BAR_W + GAP);
          float mag = mag32[j];
          int barH = (int)(mag * scale);
          if (barH > LINE_H) barH = LINE_H;
          uint16_t col = canvas.color565(0, 160, 200);
          if (j >= kDopplerHzBinLo && j <= kDopplerHzBinHi) col = canvas.color565(220, 0, 200);
          if (mag > 0.0f && (int)j == d->dopplerPeakBin) col = canvas.color565(255, 255, 0);
          canvas.fillRect(x, y + LINE_H - barH, BAR_W, barH, col);
        }

        char pktStr[4];
        snprintf(pktStr, sizeof(pktStr), "%2d", (int)d->pktRate);
        canvas.setTextColor(canvas.color565(100, 100, 120), BG_COL);
        canvas.drawString(pktStr, 227, y + 6);
      }
      canvas.drawRect(0, y, 240, LINE_H, BORDER_COL);
    }
    deviceCount = n;
  } else {
    // Radar scope fills 230x230 box starting at (5,5) — R=115, cx=cy=120
    drawScope(canvas, 120, 120, 115, now, gCalibrating);
    updateBlipsFromRadar(present, s.rssi, s.motion);
  }

  // Status bar: 20px strip, rows 240-259
  const uint16_t cBar = canvas.color565(20, 0, 35);
  const uint16_t cBorder = canvas.color565(160, 0, 160);
  const uint16_t cCyan = canvas.color565(0, 200, 220);
  canvas.fillRect(0, 240, canvas.width(), 20, cBar);
  canvas.drawFastHLine(0, 240, canvas.width(), cBorder);
  canvas.drawFastHLine(0, 259, canvas.width(), cBorder);
  canvas.setTextSize(1);
  canvas.setTextColor(cCyan, cBar);

  int contacts = 0;
  if (gVizMode != VizMode::DEVICE_LIST && gVizMode != VizMode::FFT_LINES) {
    for (int i = 0; i < 12; i++) {
      if (blips[i].active && (now - blips[i].birth) < BLIP_LIFE) contacts++;
    }
  }

  int displayCount = (contacts > 0) ? contacts + 1 : deviceCount;

  char status[64];
  if (gCalibrating) {
    int remaining = (int)((kCalDurationMs - (now - gCalStartMs) + 999) / 1000);
    snprintf(status, sizeof(status), "CAL %ds step away!  %s", remaining, gIpStr);
  } else if (gThreshold == 0.0f) {
    snprintf(status, sizeof(status), "T:EMA C:%d M:%d%% %ddBm %s",
      displayCount, (int)(s.motion * 100), s.rssi, gIpStr);
  } else {
    snprintf(status, sizeof(status), "T:%2d%% C:%d M:%d%% %ddBm %s",
      (int)(gThreshold * 100), displayCount, (int)(s.motion * 100), s.rssi, gIpStr);
  }
  canvas.drawString(status, (canvas.width() - canvas.textWidth(status)) / 2, 246);

  // 4 touch buttons: 40px strip (chunky targets), rows 260-299
  canvas.setTextSize(1);
  const char* const btnLabel[4] = { "THR-", "THR+", "CAL", "MENU" };
  const uint16_t cDim = canvas.color565(80, 80, 80);
  const uint16_t cBtnBg = canvas.color565(12, 0, 20);
  const uint16_t cBtnBorder = canvas.color565(100, 0, 100);
  canvas.drawFastHLine(0, 260, canvas.width(), cBorder);           // top of button strip
  for (int i = 0; i < 4; i++) {
    int bx = i * (canvas.width() / 4);
    canvas.fillRect(bx + 2, 262, canvas.width() / 4 - 4, 36, cBtnBg);
    canvas.drawRect(bx + 2, 262, canvas.width() / 4 - 4, 36, cBtnBorder);
    canvas.setTextColor(cDim, cBtnBg);
    canvas.drawString(btnLabel[i], bx + (canvas.width() / 4 - canvas.textWidth(btnLabel[i])) / 2, 274);
  }
  canvas.drawFastHLine(0, 299, canvas.width(), cBorder);           // bottom of button strip
  for (int i = 1; i < 4; i++) {
    canvas.drawFastVLine(i * (canvas.width() / 4), 260, 40, cBorder);
  }

  // ── FFT bar graph: 32 bins (DC skipped), y=300-320, 6px wide bars, 1px gap ──
  // Hidden in FFT_LINES mode since the full multi-line FFT is already shown above
  {
    const int BAR_W = 6;
    const int GAP = 1;
    const int BAR_H = 20;
    const int START_X = 8;
    const int BAR_Y = 320 - BAR_H;  // 300
    const int NBARS = 32;

    if (gVizMode != VizMode::FFT_LINES) {
      // Find top RSSI visible device that has FFT data
      const DeviceEntry* top = nullptr;
      for (int i = 0; i < kMaxDevices; i++) {
        if (gDevices[i].valid && gDevices[i].visible && !gDevices[i].hidden && gDevices[i].dopplerFilled >= kDopplerFrames) {
          if (!top || gDevices[i].rssi > top->rssi) top = &gDevices[i];
        }
      }

      if (top && top->dopplerFft && gDopplerHandle) {
        float mag32[32] = {0};
        float maxMag = 0.0f;
        for (int j = 1; j < kDopplerFFTSize / 2; j++) {
          float re = top->dopplerFft[j * 2];
          float im = top->dopplerFft[j * 2 + 1];
          float mag = sqrtf(re * re + im * im);
          if (gThreshold == 0.0f) {
            mag = mag - top->dopplerNoiseFloor[j];  // EMA noise subtraction
            if (mag < 0.0f) mag = 0.0f;
          }
          mag32[j] = mag;
          if (mag > maxMag) maxMag = mag;
        }
        float scale = (maxMag > 0.0f) ? (float)BAR_H / maxMag : 0.0f;

        for (int j = 1; j < NBARS; j++) {           // bins 1-31 (skip DC bin 0)
          int x = START_X + (j - 1) * (BAR_W + GAP);
          float mag = mag32[j];
          int barH = (int)(mag * scale);
          if (barH > BAR_H) barH = BAR_H;
          uint16_t col = canvas.color565(0, 160, 200);
          if (j >= kDopplerHzBinLo && j <= kDopplerHzBinHi) col = canvas.color565(220, 0, 200);  // human band
          if (mag > 0.0f && (int)j == top->dopplerPeakBin) col = canvas.color565(255, 255, 0);  // peak bin
          canvas.fillRect(x, BAR_Y + BAR_H - barH, BAR_W, barH, col);
        }
      } else {
        canvas.fillRect(START_X, BAR_Y, NBARS * (BAR_W + GAP) - GAP, BAR_H, canvas.color565(0, 0, 10));
      }
      canvas.drawRect(START_X - 1, BAR_Y - 1, NBARS * (BAR_W + GAP) + 2, BAR_H + 2, canvas.color565(60, 0, 60));
    } else {
      // FFT_LINES mode: clear the strip to black
      canvas.fillRect(0, BAR_Y, 240, BAR_H, TFT_BLACK);
    }
  }

  canvas.pushSprite(0, 0);
}
#endif // FREENOVE

// ── Input handling ────────────────────────────────────────────────────────────
#if defined(FREENOVE_FNK0104B)
static void serviceKeys() {
  int zone = display.pollTouch();
  if (zone < 0) return;

  if (gMenuOpen) {
    if (zone == 0) gMenuCursor = (gMenuCursor + kNumPalettes - 1) % kNumPalettes;
    else if (zone == 1) gMenuCursor = (gMenuCursor + 1) % kNumPalettes;
    else if (zone == 2) menuAdjust(+1);
    else if (zone == 3) menuAdjust(-1);
    else if (zone == 4) { saveSettings(); gMenuOpen = false; }
  } else {
    if (zone == 0) cycleVizMode();
    else if (zone == 1) { gThreshold -= 0.05f; if (gThreshold < 0.0f) gThreshold = 0.0f; radar.setThreshold(gThreshold); } else if (zone == 2) { gThreshold += 0.05f; if (gThreshold > 0.95f) gThreshold = 0.95f; radar.setThreshold(gThreshold); } else if (zone == 3) startCalibration();
    else if (zone == 4) { gMenuOpen = true; gMenuCursor = 0; }
  }
}
#else
static void serviceKeys() {
  if (!M5Cardputer.Keyboard.isChange() || !M5Cardputer.Keyboard.isPressed()) return;
  auto st = M5Cardputer.Keyboard.keysState();
  for (char c : st.word) {
    if (gMenuOpen) {
      if (c == ';') gMenuCursor = (gMenuCursor + 2) % 3;
      else if (c == '.') gMenuCursor = (gMenuCursor + 1) % 3;
      else if (c == ',') menuAdjust(-1);
      else if (c == '/') menuAdjust(+1);
      else if (c == '`') { saveSettings(); gMenuOpen = false; }
    } else {
      if (c == '`') { gMenuOpen = true; gMenuCursor = 0; } else if (c == 'c' || c == 'C') radar.calibrate();
      else if (c == ',') { gThreshold -= 0.05f; if (gThreshold < 0.0f) gThreshold = 0.0f; radar.setThreshold(gThreshold); } else if (c == '/') { gThreshold += 0.05f; if (gThreshold > 0.95f) gThreshold = 0.95f; radar.setThreshold(gThreshold); }
    }
  }
}
#endif

// ── setup / loop ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);
  #if defined(FREENOVE_FNK0104B)
  displayReady = display.begin();
  display.setRotation(2);
  canvas.setColorDepth(16);
  canvas.createSprite(240, 320);
  #else
  auto cfg = M5.config();
  M5Cardputer.begin(cfg, true);
  M5Cardputer.Display.setRotation(1);
  M5Cardputer.Display.fillScreen(TFT_BLACK);
  canvas.setColorDepth(16);
  canvas.createSprite(M5Cardputer.Display.width(), M5Cardputer.Display.height());
  extPanel.init();
  extPanel.setRotation(7);
  extPanel.fillScreen(TFT_BLACK);
  topCanvas.setColorDepth(16);
  extReady = (topCanvas.createSprite(240, 180) != nullptr);
  #endif

  randomSeed(micros());
  loadSettings();
  applyBrightness();
  // initCsi();
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    // NVS is corrupt or full. This is unrecoverable.
    Serial.println("NVS partition corrupt! Erasing and restarting...");
    ESP_ERROR_CHECK_WITHOUT_ABORT(nvs_flash_erase());
    esp_restart();
  }
  ESP_ERROR_CHECK_WITHOUT_ABORT(ret); // Check for other errors
  Serial.println("NVS flash initialized.");
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_init());
  
  sta_netif = esp_netif_create_default_wifi_sta();
  ap_netif = esp_netif_create_default_wifi_ap();
  wifi_init_config_t wifi_initiation = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&wifi_initiation);
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_restore());
  esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
  esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

  wifi_country_t country = {
    .cc = "US",
    .schan = 1,
    .nchan = 14,
    .max_tx_power = 20,
    .policy = WIFI_COUNTRY_POLICY_MANUAL
  };
  esp_wifi_set_country(&country);

  esp_wifi_set_mode(WIFI_MODE_STA);
  // ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_start());

  // // Wait for WiFi to be fully ready
  vTaskDelay(pdMS_TO_TICKS(1000));

  // // Stop any auto-connection attempt
  // esp_wifi_disconnect();
  vTaskDelay(pdMS_TO_TICKS(100));
  Serial.println("Checking WiFi Stuff");

  // wifi_band_mode_t band_mode;
  // esp_wifi_get_band_mode(&band_mode);
  // Serial.printf("Band mode: %s\n", wifi_band_mode_to_string(band_mode));

  // wifi_protocols_t protocols;
  // esp_wifi_get_protocols(WIFI_IF_STA, &protocols);
  // print_wifi_protocols("2.4GHz protocols before set:", protocols.ghz_2g);
  // if (band_mode != WIFI_BAND_MODE_2G_ONLY) print_wifi_protocols("5GHz protocols before set:", protocols.ghz_5g);

  wifi_country_t country_check;
  esp_wifi_get_country(&country_check);
  Serial.printf("Country: %.2s, channels %d-%d\n", country_check.cc, country_check.schan, country_check.schan + country_check.nchan - 1);

  // wifi_protocols_t xprotocols;
  // wifi_bandwidths_t bw_config;

  // if (band_mode != WIFI_BAND_MODE_2G_ONLY) {
  //   xprotocols = {
  //     .ghz_2g = WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N,
  //     .ghz_5g = WIFI_PROTOCOL_11A | WIFI_PROTOCOL_11N
  //   };
  //   ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_protocols(WIFI_IF_STA, &xprotocols));
  //   bw_config = {
  //     .ghz_2g = WIFI_BW_HT40,
  //     .ghz_5g = WIFI_BW_HT40,
  //   };
  //   esp_err_t err = esp_wifi_set_bandwidths(WIFI_IF_STA, &bw_config);
  //   Serial.printf("Set bandwidths result: %d (%s)\n", err, esp_err_to_name(err));
  // } else {
  //   ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N));
  //   esp_err_t err = esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT40);
  //   Serial.printf("Set bandwidth result: %d (%s)\n", err, esp_err_to_name(err));
  // }

  // esp_wifi_get_protocols(WIFI_IF_STA, &xprotocols);

  // print_wifi_protocols("2.4GHz protocols after set:", xprotocols.ghz_2g);
  // if (band_mode != WIFI_BAND_MODE_2G_ONLY) print_wifi_protocols("5GHz protocols after set:", xprotocols.ghz_5g);
  // wifi_config_t sta_cfg = {};

  wifi_config_t wifi_sta_config = {};
  strncpy(reinterpret_cast<char*>(wifi_sta_config.sta.ssid), WIFI_SSID, sizeof(wifi_sta_config.sta.ssid));
  strncpy(reinterpret_cast<char*>(wifi_sta_config.sta.password), WIFI_PASS, sizeof(wifi_sta_config.sta.password));
  wifi_sta_config.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
  wifi_sta_config.sta.failure_retry_cnt = 5;
  wifi_sta_config.sta.threshold.authmode = WIFI_AUTH_WPA_PSK;
  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_set_config(WIFI_IF_STA, &wifi_sta_config));

  // ESP_ERROR_CHECK(esp_wifi_start());
  // ESP_ERROR_CHECK(esp_wifi_connect());

  ESP_ERROR_CHECK_WITHOUT_ABORT(esp_wifi_start());

}

void loop() {
  #if defined(FREENOVE_FNK0104B)
  serviceKeys();
  serviceCsi();
  uint32_t now = millis();

  // Poll for STA IP once connected — more reliable than events when Arduino core is involved
  // if (gIpStr[0] == 'n') {
  //   esp_netif_t* sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
  //   if (sta_netif) {
  //     esp_netif_ip_info_t ip_info;
  //     if (esp_netif_get_ip_info(sta_netif, &ip_info) == ESP_OK && ip_info.ip.addr != 0) {
  //       snprintf(gIpStr, sizeof(gIpStr), "%d.%d.%d.%d", IP2STR(&ip_info.ip));
  //       Serial.printf("# STA IP: %s\n", gIpStr);
  //     }
  //   }
  // }

  drainPktRates(now);
  // Run per-device Doppler FFT — sliding window, continuous as new samples arrive
  for (int i = 0; i < kMaxDevices; i++) {
    if (!gDevices[i].valid || !gDevices[i].dopplerBuf) continue;
    if (gDevices[i].dopplerFilled >= kDopplerFrames) {
      runDopplerFFTForDevice(i);  // runs every ~66 ms (every new sample once buffer is full)
    }
  }
  static uint32_t lastDraw = 0;
  if (now - lastDraw >= 125) {
    lastDraw = now;
    if (gMenuOpen) {
      // Simplified menu for Freenove
      canvas.fillSprite(canvas.color565(4, 0, 8));
      canvas.setTextSize(2);
      canvas.setTextColor(gColA, canvas.color565(4, 0, 8));
      canvas.drawString("SETTINGS", 60, 10);
      canvas.setTextSize(1);
      canvas.setTextColor(TFT_WHITE, canvas.color565(4, 0, 8));
      canvas.drawString("TOP:next  RIGHT:+  BOTTOM:save", 10, 290);
      canvas.pushSprite(0, 0);
    } else {
      drawRadar();
    }
  }
  #else
  M5Cardputer.update();
  serviceKeys();
  serviceCsi();

  uint32_t now = millis();
  drainPktRates(now);
  static uint32_t lastBot = 0, lastTop = 0;
  if (now - lastBot >= 33) { lastBot = now; if (gMenuOpen) drawMenu(); else drawBottom(); }
  if (extReady && now - lastTop >= 125) {
    lastTop = now;
    drawTop();
  }
  #endif
}
