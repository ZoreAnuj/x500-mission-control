// ESP32-CAM (AI-Thinker, OV2640) -> QVGA 320x240 JPEG MJPEG stream for the Drone Hoop policy.
// Matches the sim front-camera resolution. SoftAP so the companion Pi joins directly (one hop).
// Stream URL after boot (Serial @115200): http://192.168.4.1:81/stream
//
// Arduino IDE: Board = "AI Thinker ESP32-CAM", PSRAM enabled. See README.md for flashing + FOV match.
#include "esp_camera.h"
#include "esp_http_server.h"
#include <WiFi.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

const char* AP_SSID = "dronecam";
const char* AP_PASS = "dronecam123";        // >= 8 chars

// --- AI-Thinker OV2640 pin map ---
#define PWDN_GPIO_NUM 32
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 0
#define SIOD_GPIO_NUM 26
#define SIOC_GPIO_NUM 27
#define Y9_GPIO_NUM 35
#define Y8_GPIO_NUM 34
#define Y7_GPIO_NUM 39
#define Y6_GPIO_NUM 36
#define Y5_GPIO_NUM 21
#define Y4_GPIO_NUM 19
#define Y3_GPIO_NUM 18
#define Y2_GPIO_NUM 5
#define VSYNC_GPIO_NUM 25
#define HREF_GPIO_NUM 23
#define PCLK_GPIO_NUM 22

#define PART_BOUNDARY "frame"
static const char* CT   = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* BND  = "\r\n--" PART_BOUNDARY "\r\n";
static const char* HDR  = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";
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

void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 81;
  config.ctrl_port = 32768;
  httpd_uri_t uri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };
  if (httpd_start(&stream_httpd, &config) == ESP_OK)
    httpd_register_uri_handler(stream_httpd, &uri);
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);   // MASKS brownout — still fix the 5V supply (README gotchas)
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

  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("stream: http://"); Serial.print(WiFi.softAPIP()); Serial.println(":81/stream");
  startCameraServer();
}

void loop() { delay(1000); }
