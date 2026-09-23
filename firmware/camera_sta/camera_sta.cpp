// See camera_sta.ino for the overview. All code lives here so the build
// does not depend on the sketch preprocessor generating prototypes.
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include "esp_camera.h"
#include "esp_http_server.h"
#include "camera_pins.h"
#include "secrets.h"

static const char *HOSTNAME = "elegoo-car";
static const uint16_t CMD_PORT = 100;
static const uint32_t WIFI_JOIN_TIMEOUT_MS = 20000;
static const uint32_t HEARTBEAT_PERIOD_MS = 1000;
static const uint8_t HEARTBEAT_MISS_LIMIT = 3;

WiFiServer cmdServer(CMD_PORT);
httpd_handle_t streamServer = nullptr;
httpd_handle_t captureServer = nullptr;

// ---------------------------------------------------------------- camera

static bool cameraInit() {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer = LEDC_TIMER_0;
  c.pin_d0 = Y2_GPIO_NUM;
  c.pin_d1 = Y3_GPIO_NUM;
  c.pin_d2 = Y4_GPIO_NUM;
  c.pin_d3 = Y5_GPIO_NUM;
  c.pin_d4 = Y6_GPIO_NUM;
  c.pin_d5 = Y7_GPIO_NUM;
  c.pin_d6 = Y8_GPIO_NUM;
  c.pin_d7 = Y9_GPIO_NUM;
  c.pin_xclk = XCLK_GPIO_NUM;
  c.pin_pclk = PCLK_GPIO_NUM;
  c.pin_vsync = VSYNC_GPIO_NUM;
  c.pin_href = HREF_GPIO_NUM;
  c.pin_sccb_sda = SIOD_GPIO_NUM;
  c.pin_sccb_scl = SIOC_GPIO_NUM;
  c.pin_pwdn = PWDN_GPIO_NUM;
  c.pin_reset = RESET_GPIO_NUM;
  c.xclk_freq_hz = 20000000;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_SVGA;   // 800x600, same as stock
  c.jpeg_quality = 12;
  c.fb_count = psramFound() ? 2 : 1;
  c.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    Serial.printf("camera: init failed 0x%x\n", err);
    return false;
  }
  return true;
}

static esp_err_t captureHandler(httpd_req_t *req) {
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    httpd_resp_send_500(req);
    return ESP_FAIL;
  }
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=capture.jpg");
  esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
  esp_camera_fb_return(fb);
  return res;
}

static const char *STREAM_BOUNDARY = "123456789000000000000987654321";

static esp_err_t streamHandler(httpd_req_t *req) {
  char ctype[96];
  snprintf(ctype, sizeof ctype, "multipart/x-mixed-replace;boundary=%s", STREAM_BOUNDARY);
  esp_err_t res = httpd_resp_set_type(req, ctype);
  if (res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  char part[96];
  for (;;) {
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("stream: frame capture failed");
      return ESP_FAIL;
    }
    int hlen = snprintf(part, sizeof part,
                        "\r\n--%s\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                        STREAM_BOUNDARY, (unsigned)fb->len);
    res = httpd_resp_send_chunk(req, part, hlen);
    if (res == ESP_OK) res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    if (res != ESP_OK) return res;   // client went away
  }
}

static void startHttp() {
  httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
  cfg.server_port = 80;
  cfg.ctrl_port = 32768;
  httpd_uri_t capture = { .uri = "/capture", .method = HTTP_GET, .handler = captureHandler, .user_ctx = nullptr };
  if (httpd_start(&captureServer, &cfg) == ESP_OK) httpd_register_uri_handler(captureServer, &capture);

  cfg.server_port = 81;
  cfg.ctrl_port = 32769;
  httpd_uri_t stream = { .uri = "/stream", .method = HTTP_GET, .handler = streamHandler, .user_ctx = nullptr };
  if (httpd_start(&streamServer, &cfg) == ESP_OK) httpd_register_uri_handler(streamServer, &stream);
}

// ------------------------------------------------------------------ wifi

static String chipSuffix() {
  uint64_t id = ESP.getEfuseMac();
  char buf[16];
  snprintf(buf, sizeof buf, "%04X%08X", (uint16_t)(id >> 32), (uint32_t)id);
  return String(buf);
}

static bool joinNetwork() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.setSleep(false);            // lower latency for the command link
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.printf("wifi: joining %s", WIFI_SSID);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < WIFI_JOIN_TIMEOUT_MS) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) return false;
  Serial.print("wifi: ip ");
  Serial.println(WiFi.localIP());
  return true;
}

static void startFallbackAp() {
  String ssid = "ELEGOO-" + chipSuffix();
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid.c_str(), "");
  Serial.print("wifi: fallback AP ");
  Serial.print(ssid);
  Serial.print(" ip ");
  Serial.println(WiFi.softAPIP());
}

// --------------------------------------------------------- command bridge

// Relays one client's JSON frames to the UNO and the UNO's replies back.
// Blocks until the client disconnects or misses HEARTBEAT_MISS_LIMIT beats.
static void serveClient(WiFiClient &client) {
  Serial.print("cmd: client ");
  Serial.println(client.remoteIP());
  digitalWrite(LED_PIN, LOW);      // stock polarity: LOW = lit

  String inFrame, outFrame;
  bool inBody = false;
  bool beatSeen = false;
  uint8_t beatsMissed = 0;
  uint32_t lastBeat = millis();

  while (client.connected()) {
    while (client.available()) {
      char ch = client.read();
      if (!inBody && ch == '{') inBody = true;
      if (inBody && ch != ' ') inFrame += ch;
      if (inBody && ch == '}') {
        inBody = false;
        if (inFrame == "{Heartbeat}") beatSeen = true;
        else Serial2.print(inFrame);
        inFrame = "";
      }
    }
    while (Serial2.available()) {
      char ch = Serial2.read();
      outFrame += ch;
      if (ch == '}') {
        client.print(outFrame);
        outFrame = "";
      }
    }
    if (millis() - lastBeat >= HEARTBEAT_PERIOD_MS) {
      lastBeat = millis();
      client.print("{Heartbeat}");
      if (beatSeen) { beatSeen = false; beatsMissed = 0; }
      else if (++beatsMissed >= HEARTBEAT_MISS_LIMIT) {
        Serial.println("cmd: client silent, dropping");
        break;
      }
    }
    delay(1);
  }
  client.stop();
  digitalWrite(LED_PIN, HIGH);
  Serial.println("cmd: client gone");
}

// ------------------------------------------------------------------ main

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, UNO_RX_PIN, UNO_TX_PIN);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Serial.println();
  Serial.println("elegoo-car camera_sta boot");
  bool cam = cameraInit();
  bool sta = joinNetwork();
  if (!sta) startFallbackAp();
  if (MDNS.begin(HOSTNAME)) {
    MDNS.addService("elegoo-cmd", "tcp", CMD_PORT);
    Serial.printf("mdns: %s.local\n", HOSTNAME);
  }
  if (cam) startHttp();
  cmdServer.begin();
  Serial.printf("ready: cmd tcp/%u, stream http/81, capture http/80\n", CMD_PORT);
}

void loop() {
  WiFiClient client = cmdServer.accept();
  if (client) {
    serveClient(client);
    return;
  }
  // No client: keep the UNO's chatter from filling the UART buffer.
  while (Serial2.available()) Serial2.read();
  delay(5);
}
