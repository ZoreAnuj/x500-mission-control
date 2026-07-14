// Freenove ESP32-S3-WROOM CAM (OV2640) -> QVGA 320x240 JPEG MJPEG stream for the Drone Hoop policy.
// Matches the sim front-camera resolution. SoftAP so the companion Pi joins directly (one hop).
// Stream URL after boot (Serial @115200 on the CH343/COM port): http://192.168.4.1:81/stream
//
// Board = "ESP32S3 Dev Module", PSRAM = "OPI PSRAM", Flash 16MB, Partition "Huge APP",
// USB CDC On Boot = Disabled (Serial -> CH343 UART). See ../README.md for flashing + FOV match.
#include "esp_camera.h"
#include "esp_http_server.h"
#include <WiFi.h>

// As flashed on the drone's Freenove S3 (2026-07-08). "Ketu" is deliberately unique:
// an SSID that matches a nearby home/router network (we hit this with "zero") makes
// devices join the wrong net and 192.168.4.1 times out.
const char* AP_SSID = "Ketu";
const char* AP_PASS = "12345678";           // >= 8 chars

// --- Freenove ESP32-S3-WROOM CAM OV2640 pin map (== ESP32S3_EYE; verified vs Espressif camera_pins.h) ---
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 15
#define SIOD_GPIO_NUM 4
#define SIOC_GPIO_NUM 5
#define Y9_GPIO_NUM 16
#define Y8_GPIO_NUM 17
#define Y7_GPIO_NUM 18
#define Y6_GPIO_NUM 12
#define Y5_GPIO_NUM 10
#define Y4_GPIO_NUM 8
#define Y3_GPIO_NUM 9
#define Y2_GPIO_NUM 11
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 7
#define PCLK_GPIO_NUM 13

#define PART_BOUNDARY "frame"
static const char* CT  = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* BND = "\r\n--" PART_BOUNDARY "\r\n";
static const char* HDR = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
httpd_handle_t stream_httpd = NULL;

static esp_err_t stream_handler(httpd_req_t *req) {
  esp_err_t res = httpd_resp_set_type(req, CT);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  char part[64];
  while (true) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) { res = ESP_FAIL; break; }
    size_t hlen = snprintf(part, sizeof(part), HDR, fb->len);
    if (httpd_resp_send_chunk(req, BND, strlen(BND)) != ESP_OK ||
        httpd_resp_send_chunk(req, part, hlen) != ESP_OK ||
        httpd_resp_send_chunk(req, (const char*)fb->buf, fb->len) != ESP_OK) {
      esp_camera_fb_return(fb); res = ESP_FAIL; break;
    }
    esp_camera_fb_return(fb);      // return promptly so GRAB_LATEST keeps frames fresh
  }
  return res;
}

// Live sensor tuning without reflashing:  GET /ctrl?var=<name>&val=<int>
// vars: ae_level(-2..2)  aec_value(0..1200, implies manual exposure)  aec(0/1)
//       gainceiling(0..6 = 2x..128x)  agc(0/1)  wb_mode(0auto 1sunny 2cloudy...)
//       brightness(-2..2)  contrast(-2..2)  vflip(0/1)  hmirror(0/1)
static esp_err_t ctrl_handler(httpd_req_t *req) {
  char buf[96], var[24] = {0}, val[12] = {0};
  if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
    httpd_query_key_value(buf, "var", var, sizeof(var));
    httpd_query_key_value(buf, "val", val, sizeof(val));
  }
  sensor_t *s = esp_camera_sensor_get();
  int v = atoi(val);
  int ok = -1;
  if      (!strcmp(var, "ae_level"))    ok = s->set_ae_level(s, v);
  else if (!strcmp(var, "aec"))         ok = s->set_exposure_ctrl(s, v);
  else if (!strcmp(var, "aec_value")) { s->set_exposure_ctrl(s, 0); ok = s->set_aec_value(s, v); }
  else if (!strcmp(var, "gainceiling")) ok = s->set_gainceiling(s, (gainceiling_t)v);
  else if (!strcmp(var, "agc"))         ok = s->set_gain_ctrl(s, v);
  else if (!strcmp(var, "wb_mode"))     ok = s->set_wb_mode(s, v);
  else if (!strcmp(var, "brightness"))  ok = s->set_brightness(s, v);
  else if (!strcmp(var, "contrast"))    ok = s->set_contrast(s, v);
  else if (!strcmp(var, "vflip"))       ok = s->set_vflip(s, v);
  else if (!strcmp(var, "hmirror"))     ok = s->set_hmirror(s, v);
  httpd_resp_set_type(req, "text/plain");
  snprintf(buf, sizeof(buf), "%s=%s -> %s\n", var, val, ok == 0 ? "OK" : "ERR");
  return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;
  config.ctrl_port = 32768;
  httpd_uri_t uri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };
  httpd_uri_t ctl = { .uri = "/ctrl", .method = HTTP_GET, .handler = ctrl_handler, .user_ctx = NULL };
  if (httpd_start(&stream_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &uri);
    httpd_register_uri_handler(stream_httpd, &ctl);
  }
}

void setup() {
  Serial.begin(115200);

  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0; config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0=Y2_GPIO_NUM; config.pin_d1=Y3_GPIO_NUM; config.pin_d2=Y4_GPIO_NUM; config.pin_d3=Y5_GPIO_NUM;
  config.pin_d4=Y6_GPIO_NUM; config.pin_d5=Y7_GPIO_NUM; config.pin_d6=Y8_GPIO_NUM; config.pin_d7=Y9_GPIO_NUM;
  config.pin_xclk=XCLK_GPIO_NUM; config.pin_pclk=PCLK_GPIO_NUM; config.pin_vsync=VSYNC_GPIO_NUM;
  config.pin_href=HREF_GPIO_NUM; config.pin_sccb_sda=SIOD_GPIO_NUM; config.pin_sccb_scl=SIOC_GPIO_NUM;
  config.pin_pwdn=PWDN_GPIO_NUM; config.pin_reset=RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;              // -> 10000000 if you see "Failed to get frame on time"
  config.frame_size   = FRAMESIZE_QVGA;        // 320x240 = sim resolution
  config.pixel_format = PIXFORMAT_JPEG;        // JPEG is required for 30 fps over WiFi
  config.grab_mode    = CAMERA_GRAB_LATEST;    // low latency: always serve the newest frame
  config.fb_location  = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;                    // 0(best)-63; 10-12 good
  config.fb_count     = 2;                     // needs PSRAM (double-buffer)
  if (!psramFound()) { config.fb_count = 1; config.fb_location = CAMERA_FB_IN_DRAM; }

  if (esp_camera_init(&config) != ESP_OK) { Serial.println("camera init FAILED"); return; }

  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 1);      // match training orientation (live gRPC frames were upside-down); flip if wrong
  s->set_hmirror(s, 0);
  // Outdoor exposure defaults: full sun blew the image out with stock AE.
  s->set_ae_level(s, -2);                      // aim auto-exposure darker
  s->set_aec2(s, 1);                           // smarter AEC DSP algorithm
  s->set_gainceiling(s, (gainceiling_t)0);     // cap AGC at 2x (bright scenes need no gain)
  s->set_lenc(s, 1);                           // lens shading correction
  s->set_bpc(s, 1); s->set_wpc(s, 1);          // bad/white pixel correction
  // live re-tuning without reflash: http://192.168.4.1:81/ctrl?var=ae_level&val=-1  etc.

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("stream: http://"); Serial.print(WiFi.softAPIP()); Serial.println(":81/stream");
  Serial.print("AP SSID: "); Serial.println(AP_SSID);
  startCameraServer();
}

void loop() { delay(1000); }
