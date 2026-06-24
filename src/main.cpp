// XIAO ESP32-S3 Sense - Underwater Motion Detector
// -------------------------------------------------
// - Grabs grayscale frames from the OV2640, detects motion via frame
//   differencing.
// - Drives two outputs while "active" (motion + adjustable hysteresis):
//     * DIGITAL_OUT_PIN  -> HIGH while active, LOW when idle
//     * SERVO_OUT_PIN    -> 2000us servo pulse while active, 1000us when idle
// - Hosts a WiFi SoftAP with a tuning web page (port 80) and an MJPEG
//   camera preview stream (port 81).
// - All tunable parameters persist to NVS flash.

#include <Arduino.h>
#include <WiFi.h>
#include <Preferences.h>
#include <esp_camera.h>
#include <esp_http_server.h>
#include <FS.h>
#include <SD_MMC.h>

#include "camera_pins.h"
#include "web_index.h"

// XIAO ESP32-S3 Sense onboard microSD slot, driven over SD_MMC in 1-bit mode.
// (Official Seeed camera-example mapping; coexists with the camera.)
static const int SD_CLK_PIN = 7;
static const int SD_CMD_PIN = 9;
static const int SD_D0_PIN  = 8;

// ---------------------------------------------------------------------------
// Output pins (free XIAO header pins, clear of the camera/PSRAM/SD lines).
//   D0 = GPIO1, D1 = GPIO2
// ---------------------------------------------------------------------------
static const int DIGITAL_OUT_PIN = 1;   // D0: digital "motion active" line
static const int SERVO_OUT_PIN   = 2;   // D1: servo-style PWM line

// LEDC (hardware PWM) config for the servo signal.
static const int      SERVO_LEDC_CHANNEL = 4;       // avoid camera's LEDC ch0
static const int      SERVO_LEDC_FREQ_HZ = 50;      // 20 ms servo frame
static const int      SERVO_LEDC_BITS    = 14;      // max LEDC res at 50 Hz on S3
static const uint32_t SERVO_PERIOD_US    = 20000;   // 1 / 50 Hz
static const int      SERVO_PULSE_IDLE_US   = 1000;
static const int      SERVO_PULSE_ACTIVE_US = 2000;

// ---------------------------------------------------------------------------
// SoftAP credentials
// ---------------------------------------------------------------------------
static const char *AP_SSID = "XIAO-Motion";
static const char *AP_PASS = "underwater";   // >= 8 chars; change as desired

// ---------------------------------------------------------------------------
// Tunable settings (persisted to NVS)
// ---------------------------------------------------------------------------
struct Settings {
  uint16_t pixelThreshold;     // per-pixel abs-diff that counts as "changed" (0-255)
  uint16_t minChangedPermille; // tenths of a percent of sampled pixels to trigger
  uint32_t hysteresisMs;       // hold time after last motion before releasing
  uint16_t intervalMs;         // time between detection frames
  uint16_t redGain;            // preview red level, percent (100 = unity)
  uint16_t greenGain;          // preview green level, percent
  uint16_t blueGain;           // preview blue level, percent
  uint8_t  wbMode;             // sensor white-balance preset: 0 auto,1 sunny,2 cloudy,3 office,4 home
  uint8_t  sdCapture;          // 1 = save JPEGs to SD during active periods
  uint16_t captureIntervalMs;  // min time between saved frames while active
};

static const Settings DEFAULTS = {
  /*pixelThreshold*/    25,
  /*minChangedPermille*/30,   // 3.0%
  /*hysteresisMs*/      3000,
  /*intervalMs*/        100,
  /*redGain*/           100,
  /*greenGain*/         100,
  /*blueGain*/          100,
  /*wbMode*/            0,    // auto
  /*sdCapture*/         0,    // off until enabled
  /*captureIntervalMs*/ 1000, // 1 frame/sec while active
};

static Settings g_set;
static Preferences g_prefs;

// ---------------------------------------------------------------------------
// Runtime state
// ---------------------------------------------------------------------------
static uint8_t *g_prevFrame = nullptr;   // previous grayscale frame (PSRAM)
static size_t   g_prevLen   = 0;
static bool     g_haveStat  = false;

static volatile bool     g_motion = false;   // motion in the latest frame
static volatile bool     g_active = false;   // motion + hysteresis
static volatile uint16_t g_changedPermille = 0;
static uint32_t g_lastMotionMs = 0;
static bool     g_motionSeen   = false;  // avoids a false "active" pulse at boot

// Latest JPEG snapshot shared with the MJPEG stream task.
static SemaphoreHandle_t g_jpgMutex = nullptr;
static uint8_t *g_jpg = nullptr;
static size_t   g_jpgLen = 0;

// SD card state
static bool     g_sdReady   = false;
static uint32_t g_bootId    = 0;   // unique per power-up, used in filenames
static uint32_t g_imgIndex  = 0;   // frame counter within this boot
static uint32_t g_imgSaved  = 0;   // images written this session
static uint32_t g_lastSaveMs = 0;

static httpd_handle_t g_webServer = nullptr;
static httpd_handle_t g_streamServer = nullptr;

// ---------------------------------------------------------------------------
// Servo / output helpers
// ---------------------------------------------------------------------------
static uint32_t pulseToDuty(uint32_t pulseUs) {
  const uint32_t maxDuty = (1u << SERVO_LEDC_BITS) - 1u;
  return (uint64_t)pulseUs * maxDuty / SERVO_PERIOD_US;
}

static void applyOutputs(bool active) {
  digitalWrite(DIGITAL_OUT_PIN, active ? HIGH : LOW);
  ledcWrite(SERVO_OUT_PIN,
            pulseToDuty(active ? SERVO_PULSE_ACTIVE_US : SERVO_PULSE_IDLE_US));
}

// ---------------------------------------------------------------------------
// Settings persistence
// ---------------------------------------------------------------------------
static void loadSettings() {
  g_prefs.begin("motion", true);
  g_set.pixelThreshold     = g_prefs.getUShort("pix", DEFAULTS.pixelThreshold);
  g_set.minChangedPermille = g_prefs.getUShort("area", DEFAULTS.minChangedPermille);
  g_set.hysteresisMs       = g_prefs.getULong("hyst", DEFAULTS.hysteresisMs);
  g_set.intervalMs         = g_prefs.getUShort("iv", DEFAULTS.intervalMs);
  g_set.redGain            = g_prefs.getUShort("rg", DEFAULTS.redGain);
  g_set.greenGain          = g_prefs.getUShort("gg", DEFAULTS.greenGain);
  g_set.blueGain           = g_prefs.getUShort("bg", DEFAULTS.blueGain);
  g_set.wbMode             = g_prefs.getUChar("wb", DEFAULTS.wbMode);
  g_set.sdCapture          = g_prefs.getUChar("sd", DEFAULTS.sdCapture);
  g_set.captureIntervalMs  = g_prefs.getUShort("ci", DEFAULTS.captureIntervalMs);
  g_prefs.end();
}

static void saveSettings() {
  g_prefs.begin("motion", false);
  g_prefs.putUShort("pix", g_set.pixelThreshold);
  g_prefs.putUShort("area", g_set.minChangedPermille);
  g_prefs.putULong("hyst", g_set.hysteresisMs);
  g_prefs.putUShort("iv", g_set.intervalMs);
  g_prefs.putUShort("rg", g_set.redGain);
  g_prefs.putUShort("gg", g_set.greenGain);
  g_prefs.putUShort("bg", g_set.blueGain);
  g_prefs.putUChar("wb", g_set.wbMode);
  g_prefs.putUChar("sd", g_set.sdCapture);
  g_prefs.putUShort("ci", g_set.captureIntervalMs);
  g_prefs.end();
}

// ---------------------------------------------------------------------------
// Camera
// ---------------------------------------------------------------------------
static bool initCamera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_QVGA;        // 320x240
  config.pixel_format = PIXFORMAT_RGB565;    // color preview; luma derived for diffing
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed: 0x%x\n", err);
    return false;
  }
  return true;
}

// Push the white-balance preset to the OV2640. Mode 0 = auto; 1-4 are the
// driver's fixed presets (sunny/cloudy/office/home), which require AWB gain on.
static void applyWhiteBalance() {
  sensor_t *s = esp_camera_sensor_get();
  if (!s) return;
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_wb_mode(s, g_set.wbMode);
}

// ---------------------------------------------------------------------------
// Motion detection on a grayscale frame
// ---------------------------------------------------------------------------
// Derive 8-bit luminance from a big-endian RGB565 pixel (matches the byte
// order the camera driver/JPEG encoder use).
static inline uint8_t rgb565Luma(uint8_t hb, uint8_t lb) {
  uint8_t r = hb & 0xF8;
  uint8_t g = ((hb & 0x07) << 5) | ((lb & 0xE0) >> 3);
  uint8_t b = (lb & 0x1F) << 3;
  return (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
}

static void detectMotion(camera_fb_t *fb) {
  // RGB565 -> one luma byte per pixel for the reference frame.
  const size_t numPixels = fb->len / 2;

  if (g_prevFrame == nullptr || g_prevLen != numPixels) {
    if (g_prevFrame) free(g_prevFrame);
    g_prevFrame = (uint8_t *)ps_malloc(numPixels);
    g_prevLen = numPixels;
    g_haveStat = false;
  }

  // Sample every 4th pixel; plenty for whole-scene motion and much cheaper.
  const size_t stride = 4;
  const uint16_t thr = g_set.pixelThreshold;
  size_t sampled = 0, changed = 0;

  if (!g_haveStat) {
    for (size_t p = 0; p < numPixels; p += stride)
      g_prevFrame[p] = rgb565Luma(fb->buf[p * 2], fb->buf[p * 2 + 1]);
    g_haveStat = true;
    g_changedPermille = 0;
    g_motion = false;
    return;
  }

  for (size_t p = 0; p < numPixels; p += stride) {
    uint8_t luma = rgb565Luma(fb->buf[p * 2], fb->buf[p * 2 + 1]);
    int d = (int)luma - (int)g_prevFrame[p];
    if (d < 0) d = -d;
    if ((uint16_t)d >= thr) changed++;
    g_prevFrame[p] = luma;  // update reference as we go
    sampled++;
  }

  uint16_t permille = sampled ? (uint16_t)((changed * 1000UL) / sampled) : 0;
  g_changedPermille = permille;
  g_motion = permille >= g_set.minChangedPermille;
}

// ---------------------------------------------------------------------------
// Publish the latest frame as JPEG for the preview stream
// ---------------------------------------------------------------------------
// Apply per-channel gain to an RGB565 frame in place. Detection has already
// consumed this buffer (luma), so it is safe to modify before JPEG encoding.
static void applyColorGain(camera_fb_t *fb) {
  const uint16_t rg = g_set.redGain, gg = g_set.greenGain, bg = g_set.blueGain;
  if (rg == 100 && gg == 100 && bg == 100) return;  // unity: nothing to do

  for (size_t i = 0; i + 1 < fb->len; i += 2) {
    uint8_t hb = fb->buf[i], lb = fb->buf[i + 1];
    uint16_t r5 = (hb >> 3) & 0x1F;
    uint16_t g6 = ((hb & 0x07) << 3) | (lb >> 5);
    uint16_t b5 = lb & 0x1F;

    r5 = (uint16_t)((r5 * rg) / 100); if (r5 > 31) r5 = 31;
    g6 = (uint16_t)((g6 * gg) / 100); if (g6 > 63) g6 = 63;
    b5 = (uint16_t)((b5 * bg) / 100); if (b5 > 31) b5 = 31;

    fb->buf[i]     = (uint8_t)((r5 << 3) | (g6 >> 3));
    fb->buf[i + 1] = (uint8_t)(((g6 & 0x07) << 5) | b5);
  }
}

static void publishJpeg(camera_fb_t *fb) {
  applyColorGain(fb);
  uint8_t *out = nullptr;
  size_t outLen = 0;
  if (!frame2jpg(fb, 80, &out, &outLen)) return;

  if (xSemaphoreTake(g_jpgMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    if (g_jpg) free(g_jpg);
    g_jpg = out;
    g_jpgLen = outLen;
    xSemaphoreGive(g_jpgMutex);
  } else {
    free(out);  // could not publish this frame; drop it
  }
}

// ---------------------------------------------------------------------------
// SD card (SD_MMC, 1-bit)
// ---------------------------------------------------------------------------
static bool initSD() {
  if (!SD_MMC.setPins(SD_CLK_PIN, SD_CMD_PIN, SD_D0_PIN)) {
    Serial.println("SD: setPins failed");
    return false;
  }
  // Try 1-bit mode at default speed, then fall back to the slow probing clock
  // (helps large/picky cards initialize).
  bool ok = SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_DEFAULT);
  if (!ok) ok = SD_MMC.begin("/sdcard", true, false, SDMMC_FREQ_PROBING);
  if (!ok) {
    Serial.println("SD: mount failed (no card / not FAT32 / unsupported)");
    return false;
  }

  uint8_t type = SD_MMC.cardType();
  if (type == CARD_NONE) {
    Serial.println("SD: no card detected");
    return false;
  }
  SD_MMC.mkdir("/motion");

  // One NVS write per boot gives every power-up a unique filename prefix.
  g_prefs.begin("motion", false);
  g_bootId = g_prefs.getULong("boot", 0) + 1;
  g_prefs.putULong("boot", g_bootId);
  g_prefs.end();

  Serial.printf("SD: mounted, %llu MB total, %llu MB free (bootId %lu)\n",
                SD_MMC.totalBytes() / (1024ULL * 1024ULL),
                (SD_MMC.totalBytes() - SD_MMC.usedBytes()) / (1024ULL * 1024ULL),
                (unsigned long)g_bootId);
  return true;
}

static void saveImageToSD() {
  if (!g_sdReady) return;

  // Copy the latest JPEG out from under the stream mutex, then write to SD.
  uint8_t *copy = nullptr;
  size_t   copyLen = 0;
  if (xSemaphoreTake(g_jpgMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    if (g_jpg && g_jpgLen) {
      copy = (uint8_t *)malloc(g_jpgLen);
      if (copy) { memcpy(copy, g_jpg, g_jpgLen); copyLen = g_jpgLen; }
    }
    xSemaphoreGive(g_jpgMutex);
  }
  if (!copy) return;

  char path[40];
  snprintf(path, sizeof(path), "/motion/m%lu_%05lu.jpg",
           (unsigned long)g_bootId, (unsigned long)g_imgIndex);
  File f = SD_MMC.open(path, FILE_WRITE);
  if (f) {
    f.write(copy, copyLen);
    f.close();
    g_imgIndex++;
    g_imgSaved++;
  } else {
    Serial.printf("SD: failed to open %s\n", path);
    g_sdReady = false;  // card likely pulled; stop trying until reboot
  }
  free(copy);
}

// ---------------------------------------------------------------------------
// HTTP handlers (port 80)
// ---------------------------------------------------------------------------
static esp_err_t indexHandler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t settingsHandler(httpd_req_t *req) {
  char buf[256];
  int n = snprintf(buf, sizeof(buf),
    "{\"pixelThreshold\":%u,\"minChangedPermille\":%u,"
    "\"hysteresisMs\":%lu,\"intervalMs\":%u,"
    "\"redGain\":%u,\"greenGain\":%u,\"blueGain\":%u,\"wbMode\":%u,"
    "\"sdCapture\":%u,\"captureIntervalMs\":%u}",
    g_set.pixelThreshold, g_set.minChangedPermille,
    (unsigned long)g_set.hysteresisMs, g_set.intervalMs,
    g_set.redGain, g_set.greenGain, g_set.blueGain, g_set.wbMode,
    g_set.sdCapture, g_set.captureIntervalMs);
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, buf, n);
}

static esp_err_t statusHandler(httpd_req_t *req) {
  char buf[200];
  int n = snprintf(buf, sizeof(buf),
    "{\"motion\":%s,\"active\":%s,\"changedPermille\":%u,"
    "\"sdReady\":%s,\"sdImages\":%lu}",
    g_motion ? "true" : "false",
    g_active ? "true" : "false",
    g_changedPermille,
    g_sdReady ? "true" : "false",
    (unsigned long)g_imgSaved);
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, buf, n);
}

static bool getQueryLong(httpd_req_t *req, const char *key, long *out) {
  char query[128];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return false;
  char val[32];
  if (httpd_query_key_value(query, key, val, sizeof(val)) != ESP_OK) return false;
  *out = strtol(val, nullptr, 10);
  return true;
}

static esp_err_t setHandler(httpd_req_t *req) {
  long v;
  if (getQueryLong(req, "pixelThreshold", &v))
    g_set.pixelThreshold = constrain(v, 1, 255);
  if (getQueryLong(req, "minChangedPermille", &v))
    g_set.minChangedPermille = constrain(v, 1, 1000);
  if (getQueryLong(req, "hysteresisMs", &v))
    g_set.hysteresisMs = constrain(v, 0, 120000);
  if (getQueryLong(req, "intervalMs", &v))
    g_set.intervalMs = constrain(v, 10, 5000);
  if (getQueryLong(req, "redGain", &v))
    g_set.redGain = constrain(v, 0, 400);
  if (getQueryLong(req, "greenGain", &v))
    g_set.greenGain = constrain(v, 0, 400);
  if (getQueryLong(req, "blueGain", &v))
    g_set.blueGain = constrain(v, 0, 400);
  if (getQueryLong(req, "wbMode", &v)) {
    g_set.wbMode = constrain(v, 0, 4);
    applyWhiteBalance();
  }
  if (getQueryLong(req, "sdCapture", &v))
    g_set.sdCapture = v ? 1 : 0;
  if (getQueryLong(req, "captureIntervalMs", &v))
    g_set.captureIntervalMs = constrain(v, 100, 60000);
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t saveHandler(httpd_req_t *req) {
  saveSettings();
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t resetHandler(httpd_req_t *req) {
  g_set = DEFAULTS;
  saveSettings();
  applyWhiteBalance();
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t captureHandler(httpd_req_t *req) {
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  esp_err_t res = ESP_FAIL;
  if (xSemaphoreTake(g_jpgMutex, pdMS_TO_TICKS(500)) == pdTRUE) {
    if (g_jpg && g_jpgLen) res = httpd_resp_send(req, (const char *)g_jpg, g_jpgLen);
    xSemaphoreGive(g_jpgMutex);
  }
  if (res != ESP_OK) httpd_resp_send_500(req);
  return res;
}

// ---------------------------------------------------------------------------
// Gallery: list / download / delete files in /motion
// ---------------------------------------------------------------------------
// Reject anything but a bare filename to prevent path traversal.
static bool validName(const char *n) {
  if (!n || !*n) return false;
  if (strstr(n, "..")) return false;
  for (const char *p = n; *p; p++)
    if (*p == '/' || *p == '\\') return false;
  return true;
}

static bool queryName(httpd_req_t *req, char *out, size_t outLen) {
  char query[160];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return false;
  if (httpd_query_key_value(query, "name", out, outLen) != ESP_OK) return false;
  return validName(out);
}

static esp_err_t listHandler(httpd_req_t *req) {
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  if (!g_sdReady) return httpd_resp_sendstr(req, "{\"sdReady\":false,\"files\":[]}");

  char head[112];
  uint64_t totalMB = SD_MMC.totalBytes() / (1024ULL * 1024ULL);
  uint64_t freeMB = (SD_MMC.totalBytes() - SD_MMC.usedBytes()) / (1024ULL * 1024ULL);
  snprintf(head, sizeof(head),
           "{\"sdReady\":true,\"totalMB\":%llu,\"freeMB\":%llu,\"files\":[",
           totalMB, freeMB);
  httpd_resp_sendstr_chunk(req, head);

  File dir = SD_MMC.open("/motion");
  bool first = true;
  int count = 0;
  const int LIMIT = 2000;
  if (dir) {
    for (File f = dir.openNextFile(); f && count < LIMIT; f = dir.openNextFile()) {
      if (f.isDirectory()) continue;
      const char *nm = f.name();
      const char *base = strrchr(nm, '/');
      base = base ? base + 1 : nm;
      char ent[128];
      snprintf(ent, sizeof(ent), "%s{\"name\":\"%s\",\"size\":%u}",
               first ? "" : ",", base, (unsigned)f.size());
      httpd_resp_sendstr_chunk(req, ent);
      first = false;
      count++;
    }
    dir.close();
  }
  httpd_resp_sendstr_chunk(req, "]}");
  httpd_resp_sendstr_chunk(req, NULL);
  return ESP_OK;
}

static esp_err_t fileHandler(httpd_req_t *req) {
  char name[64];
  if (!g_sdReady || !queryName(req, name, sizeof(name))) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad request");
    return ESP_FAIL;
  }
  char path[96];
  snprintf(path, sizeof(path), "/motion/%s", name);
  File f = SD_MMC.open(path);
  if (!f || f.isDirectory()) {
    if (f) f.close();
    httpd_resp_send_404(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  char disp[96];
  snprintf(disp, sizeof(disp), "attachment; filename=%s", name);
  httpd_resp_set_hdr(req, "Content-Disposition", disp);

  uint8_t buf[1024];
  size_t r;
  esp_err_t res = ESP_OK;
  while ((r = f.read(buf, sizeof(buf))) > 0) {
    if (httpd_resp_send_chunk(req, (const char *)buf, r) != ESP_OK) { res = ESP_FAIL; break; }
  }
  f.close();
  if (res == ESP_OK) httpd_resp_send_chunk(req, NULL, 0);
  return res;
}

static esp_err_t deleteHandler(httpd_req_t *req) {
  char name[64];
  if (!g_sdReady || !queryName(req, name, sizeof(name))) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "bad request");
    return ESP_FAIL;
  }
  char path[96];
  snprintf(path, sizeof(path), "/motion/%s", name);
  bool ok = SD_MMC.remove(path);
  httpd_resp_set_type(req, "application/json");
  return httpd_resp_sendstr(req, ok ? "{\"ok\":true}" : "{\"ok\":false}");
}

// ---------------------------------------------------------------------------
// MJPEG stream handler (port 81)
// ---------------------------------------------------------------------------
#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE =
  "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t streamHandler(httpd_req_t *req) {
  esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char partHeader[64];
  while (true) {
    uint8_t *copy = nullptr;
    size_t copyLen = 0;

    if (xSemaphoreTake(g_jpgMutex, pdMS_TO_TICKS(200)) == pdTRUE) {
      if (g_jpg && g_jpgLen) {
        copy = (uint8_t *)malloc(g_jpgLen);
        if (copy) { memcpy(copy, g_jpg, g_jpgLen); copyLen = g_jpgLen; }
      }
      xSemaphoreGive(g_jpgMutex);
    }

    if (copy) {
      res = httpd_resp_send_chunk(req, STREAM_BOUNDARY, strlen(STREAM_BOUNDARY));
      if (res == ESP_OK) {
        int hl = snprintf(partHeader, sizeof(partHeader), STREAM_PART, copyLen);
        res = httpd_resp_send_chunk(req, partHeader, hl);
      }
      if (res == ESP_OK)
        res = httpd_resp_send_chunk(req, (const char *)copy, copyLen);
      free(copy);
      if (res != ESP_OK) break;  // client disconnected
    }
    vTaskDelay(pdMS_TO_TICKS(40));  // ~25 fps cap for the preview
  }
  return res;
}

// ---------------------------------------------------------------------------
// Server setup
// ---------------------------------------------------------------------------
static void registerUri(httpd_handle_t s, const char *path, httpd_method_t m,
                        esp_err_t (*fn)(httpd_req_t *)) {
  httpd_uri_t uri = { path, m, fn, nullptr };
  httpd_register_uri_handler(s, &uri);
}

static void startServers() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.max_uri_handlers = 16;

  if (httpd_start(&g_webServer, &config) == ESP_OK) {
    registerUri(g_webServer, "/", HTTP_GET, indexHandler);
    registerUri(g_webServer, "/settings", HTTP_GET, settingsHandler);
    registerUri(g_webServer, "/status", HTTP_GET, statusHandler);
    registerUri(g_webServer, "/set", HTTP_GET, setHandler);
    registerUri(g_webServer, "/save", HTTP_GET, saveHandler);
    registerUri(g_webServer, "/reset", HTTP_GET, resetHandler);
    registerUri(g_webServer, "/capture", HTTP_GET, captureHandler);
    registerUri(g_webServer, "/list", HTTP_GET, listHandler);
    registerUri(g_webServer, "/file", HTTP_GET, fileHandler);
    registerUri(g_webServer, "/delete", HTTP_GET, deleteHandler);
  }

  config.server_port = 81;
  config.ctrl_port += 1;  // separate control socket for the stream server
  if (httpd_start(&g_streamServer, &config) == ESP_OK) {
    registerUri(g_streamServer, "/stream", HTTP_GET, streamHandler);
  }
}

// ---------------------------------------------------------------------------
// Setup / loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nXIAO ESP32-S3 Underwater Motion Detector");

  pinMode(DIGITAL_OUT_PIN, OUTPUT);
  digitalWrite(DIGITAL_OUT_PIN, LOW);
  // arduino-esp32 3.x LEDC API: bind a fixed channel to the pin so we never
  // collide with the camera's XCLK channel/timer.
  ledcAttachChannel(SERVO_OUT_PIN, SERVO_LEDC_FREQ_HZ, SERVO_LEDC_BITS,
                    SERVO_LEDC_CHANNEL);
  applyOutputs(false);

  loadSettings();
  g_jpgMutex = xSemaphoreCreateMutex();

  if (!initCamera()) {
    Serial.println("Halting: camera unavailable.");
    while (true) delay(1000);
  }
  applyWhiteBalance();

  g_sdReady = initSD();
  if (!g_sdReady) Serial.println("SD capture disabled (card not available).");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  IPAddress ip = WiFi.softAPIP();
  Serial.printf("SoftAP \"%s\" up. Open http://%s/\n", AP_SSID, ip.toString().c_str());

  startServers();
}

void loop() {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    delay(10);
    return;
  }

  detectMotion(fb);
  publishJpeg(fb);
  esp_camera_fb_return(fb);

  uint32_t now = millis();
  if (g_motion) { g_lastMotionMs = now; g_motionSeen = true; }
  bool active = g_motion ||
                (g_motionSeen && (now - g_lastMotionMs < g_set.hysteresisMs));
  bool rising = active && !g_active;
  if (active != g_active) {
    g_active = active;
    Serial.printf("[%lu] %s (changed %.1f%%)\n", (unsigned long)now,
                  active ? "ACTIVE" : "idle", g_changedPermille / 10.0);
  }
  applyOutputs(active);

  // Capture to SD throughout the active period: once on the rising edge, then
  // every captureIntervalMs while still active.
  if (active && g_set.sdCapture && g_sdReady) {
    if (rising || (now - g_lastSaveMs >= g_set.captureIntervalMs)) {
      saveImageToSD();
      g_lastSaveMs = now;
    }
  }

  delay(g_set.intervalMs);
}
