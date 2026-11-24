/*
 * BRAC University - CSE461: Section 6 (Group 5) ESP32-CAM Web & Serial Vision System
 * Target Hardware: AI-Thinker ESP32-CAM (OV2640) + MB USB Baseboard
 * 
 * FEATURES:
 * 1. Runs detection & prints to Serial Monitor AT ALL TIMES
 *    (even if web stream fails or no browser is open).
 * 2. Web Streaming: Live HTML dashboard at http://192.168.4.1 (Port 80).
 * 3. Power Stability: Brownout protection disabled to prevent power-spike crashes.
 */

#include "esp_camera.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"


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
#define FLASH_LED_PIN      4
#define STATUS_LED_PIN    33

#define FRAME_WIDTH       160   
#define FRAME_HEIGHT      120   
const float REAL_OBJECT_WIDTH_CM = 5.0f;
const float FOCAL_LENGTH_PX       = 140.0f;

struct ClassA_Thresholds {
  uint8_t min_v            = 160;  
  uint8_t max_s            = 50;    
  uint8_t max_channel_diff = 30;    
  uint16_t min_blob_area   = 200;   
  uint16_t max_blob_area   = 5000;  
  float min_aspect_ratio   = 0.40f;
  float max_aspect_ratio   = 2.20f;
  float min_solidity       = 0.25f;
} threshA;

volatile bool isTargetLocked    = false;
volatile float smoothedCx       = 80.0f;
volatile float smoothedCy       = 60.0f;
volatile float smoothedDistance = 0.0f;
String detectedObjectLabel      = "NONE";
uint16_t lockMinX = 0, lockMaxX = 0, lockMinY = 0, lockMaxY = 0;

volatile bool isClientStreaming = false;
unsigned long lastSerialPrintTime = 0;

httpd_handle_t server_httpd = NULL;

bool initCamera();
void startWebServer();
void processFrameAndOverlay(camera_fb_t *fb);
void drawBoundingBox(uint8_t* buf, int w, int h, int minX, int maxX, int minY, int maxY, uint16_t color);
void logSerialTelemetry();

static const char HTTP_INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>CSE461 - ESP32-CAM Vision System</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial, sans-serif; background-color: #121212; color: #ffffff; text-align: center; margin: 0; padding: 20px; }
    h2 { color: #00e676; margin-bottom: 5px; }
    h4 { color: #aaa; margin-top: 0; }
    .container { display: flex; flex-direction: column; align-items: center; justify-content: center; }
    .stream-card { background: #1e1e1e; padding: 15px; border-radius: 12px; box-shadow: 0 4px 15px rgba(0,0,0,0.5); }
    img { border-radius: 8px; width: 480px; height: 360px; image-rendering: pixelated; border: 2px solid #333; }
  </style>
</head>
<body>
  <div class="container">
    <h2>ESP32-CAM Target Vision Stream</h2>
    <h4>BRAC University - Group 5 CSE461</h4>
    <div class="stream-card">
      <img src="/stream" id="video-feed" />
    </div>
  </div>
</body>
</html>
)rawliteral";

static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, HTTP_INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t *fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char part_buf[64];

  res = httpd_resp_set_type(req, "multipart/x-mixed-replace; boundary=frame");
  if (res != ESP_OK) return res;

  isClientStreaming = true; 

  while (true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      delay(10);
      continue;
    }

    processFrameAndOverlay(fb);

    bool jpeg_converted = fmt2jpg(fb->buf, fb->len, fb->width, fb->height, PIXFORMAT_RGB565, 80, &_jpg_buf, &_jpg_buf_len);
    esp_camera_fb_return(fb);

    if (!jpeg_converted || !_jpg_buf) {
      delay(10);
      continue;
    }

    size_t hlen = snprintf(part_buf, 64, "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n", _jpg_buf_len);
    res = httpd_resp_send_chunk(req, part_buf, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, "\r\n--frame\r\n", 10);

    if (_jpg_buf) {
      free(_jpg_buf);
      _jpg_buf = NULL;
    }
    if (res != ESP_OK) break;
  }

  isClientStreaming = false;
  return res;
}

void startWebServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri  = { .uri = "/",       .method = HTTP_GET, .handler = index_handler,  .user_ctx = NULL };
  httpd_uri_t stream_uri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };

  if (httpd_start(&server_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(server_httpd, &index_uri);
    httpd_register_uri_handler(server_httpd, &stream_uri);
  }
}


void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  Serial.begin(115200);
  delay(1000);

  Serial.println(F("\n==========================================="));
  Serial.println(F("   BRAC UNIVERSITY - CSE461 VISION SYSTEM  "));
  Serial.println(F("==========================================="));

  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, HIGH);


  if (!initCamera()) {
    Serial.println(F("[ERROR] Camera Hardware Failed to Initialize!"));
    while (true) {
      digitalWrite(STATUS_LED_PIN, LOW); delay(100);
      digitalWrite(STATUS_LED_PIN, HIGH); delay(100);
    }
  }
  Serial.println(F("[OK] Camera Initialized Successfully."));


  WiFi.softAP("ESP32-CAM-Vision", "12345678");
  IPAddress IP = WiFi.softAPIP();

  Serial.println(F("[OK] Access Point Mode Started."));
  Serial.print(F("[INFO] Connect Laptop Wi-Fi to: ESP32-CAM-Vision (Password: 12345678)\n"));
  Serial.print(F("[INFO] Open Browser Stream at: http://"));
  Serial.println(IP);
  Serial.println(F("-------------------------------------------\n"));

  startWebServer();
}

void loop() {
  if (!isClientStreaming) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
      processFrameAndOverlay(fb);
      esp_camera_fb_return(fb);
    }
  }

  unsigned long now = millis();
  if (now - lastSerialPrintTime >= 250) {
    lastSerialPrintTime = now;
    logSerialTelemetry();
  }

  delay(10);
}

bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_RGB565;
  config.frame_size   = FRAMESIZE_QQVGA; // 160 x 120
  config.jpeg_quality = 12;
  config.fb_count     = 1;
  config.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) return false;

  sensor_t *s = esp_camera_sensor_get();
  if (s != NULL) {
    s->set_brightness(s, 1);
    s->set_contrast(s, 2);
    s->set_saturation(s, 0);
    s->set_whitebal(s, 1);
    s->set_gain_ctrl(s, 1);
    s->set_exposure_ctrl(s, 1);
  }
  return true;
}

inline void rgbToHsvFast(uint8_t r, uint8_t g, uint8_t b, uint8_t &v, uint8_t &s) {
  uint8_t cmax = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
  uint8_t cmin = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);
  uint8_t delta = cmax - cmin;
  v = cmax;
  s = (cmax == 0) ? 0 : (uint8_t)(((uint16_t)delta * 255) / cmax);
}

void processFrameAndOverlay(camera_fb_t *fb) {
  uint32_t countA = 0, sumXA = 0, sumYA = 0;
  uint16_t minXA = FRAME_WIDTH, maxXA = 0, minYA = FRAME_HEIGHT, maxYA = 0;
  uint8_t *buf = fb->buf;

  for (uint16_t y = 0; y < FRAME_HEIGHT; y += 2) {
    for (uint16_t x = 0; x < FRAME_WIDTH; x += 2) {
      size_t pixelIndex = (y * FRAME_WIDTH + x) * 2;
      uint16_t pixel = (buf[pixelIndex] << 8) | buf[pixelIndex + 1];

      uint8_t r = ((pixel >> 11) & 0x1F) << 3;
      uint8_t g = ((pixel >> 5)  & 0x3F) << 2;
      uint8_t b = (pixel & 0x1F) << 3;
      uint8_t v, s;
      rgbToHsvFast(r, g, b, v, s);

    
      if (v >= threshA.min_v && s <= threshA.max_s &&
          abs((int)r - (int)g) <= threshA.max_channel_diff &&
          abs((int)g - (int)b) <= threshA.max_channel_diff) {
        countA++;
        sumXA += x; sumYA += y;
        if (x < minXA) minXA = x; if (x > maxXA) maxXA = x;
        if (y < minYA) minYA = y; if (y > maxYA) maxYA = y;
      }
    }
  }

  bool detectedThisFrame = false;
  if (countA >= threshA.min_blob_area && countA <= threshA.max_blob_area) {
    float w = (float)(maxXA - minXA + 1);
    float h = (float)(maxYA - minYA + 1);
    float ar = w / h;
    float sol = (float)countA / ((w * h) / 4.0f);

    if (ar >= threshA.min_aspect_ratio && ar <= threshA.max_aspect_ratio && sol >= threshA.min_solidity) {
      detectedThisFrame = true;
      isTargetLocked = true;
      lockMinX = minXA; lockMaxX = maxXA; lockMinY = minYA; lockMaxY = maxYA;
      smoothedCx = (float)sumXA / (float)countA;
      smoothedCy = (float)sumYA / (float)countA;
      smoothedDistance = (REAL_OBJECT_WIDTH_CM * FOCAL_LENGTH_PX) / w;
      detectedObjectLabel = (sol < 0.55f) ? "PAPER_BALL" : "WHITE_SQUARE";
    }
  }

  if (!detectedThisFrame) {
    isTargetLocked = false;
    detectedObjectLabel = "NONE";
  }

  if (isTargetLocked) {
    drawBoundingBox(buf, FRAME_WIDTH, FRAME_HEIGHT, lockMinX, lockMaxX, lockMinY, lockMaxY, 0x07E0); // Green
    digitalWrite(STATUS_LED_PIN, LOW); 
  } else {
    digitalWrite(STATUS_LED_PIN, HIGH); 
  }
}

void drawBoundingBox(uint8_t* buf, int w, int h, int minX, int maxX, int minY, int maxY, uint16_t color) {
  uint8_t c1 = (color >> 8) & 0xFF;
  uint8_t c2 = color & 0xFF;

  for (int t = 0; t < 2; t++) {
    int x1 = minX - t, x2 = maxX + t, y1 = minY - t, y2 = maxY + t;
    if (x1 < 0) x1 = 0; if (x2 >= w) x2 = w - 1;
    if (y1 < 0) y1 = 0; if (y2 >= h) y2 = h - 1;

    for (int x = x1; x <= x2; x++) {
      size_t i1 = (y1 * w + x) * 2; buf[i1] = c1; buf[i1 + 1] = c2;
      size_t i2 = (y2 * w + x) * 2; buf[i2] = c1; buf[i2 + 1] = c2;
    }
    for (int y = y1; y <= y2; y++) {
      size_t i1 = (y * w + x1) * 2; buf[i1] = c1; buf[i1 + 1] = c2;
      size_t i2 = (y * w + x2) * 2; buf[i2] = c1; buf[i2 + 1] = c2;
    }
  }
}

void logSerialTelemetry() {
  if (isTargetLocked) {
    Serial.print(F("[LOCKED] Target: "));
    Serial.print(detectedObjectLabel);
    Serial.print(F(" | Dist: "));
    Serial.print(smoothedDistance, 1);
    Serial.print(F(" cm | Center (X,Y): ("));
    Serial.print((int)smoothedCx);
    Serial.print(F(", "));
    Serial.print((int)smoothedCy);
    Serial.println(F(")"));
  } else {
    Serial.println(F("[SEARCHING] Target: NONE | Dist: 0.0 cm"));
  }
}