#include <WiFi.h>
#include "esp_camera.h"
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include "soc/soc.h" //disable brownout problems
#include "soc/rtc_cntl_reg.h"  //disable brownout problems

// camera pins
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

// led pins
#define FLASH_PIN 4
#define LED_PIN 33

// motor pins
#define MOTOR_1_PIN_1 12
#define MOTOR_1_PIN_2 13
#define MOTOR_2_PIN_1 15
#define MOTOR_2_PIN_2 14

// wifi credentials
const char *WIFI_SSID = "ssid";
const char *WIFI_PASS = "kuotakuhabis";

// create web server instance
AsyncWebServer server(80);
AsyncWebSocket socket("/ws");

typedef struct {
  camera_fb_t *fb;
  size_t index;
} camera_frame_t;

#define PART_BOUNDARY "123456789000000000000987654321"
static const char *STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *STREAM_PART = "Content-Type: %s\r\nContent-Length: %u\r\n\r\n";
static const char *JPG_CONTENT_TYPE = "image/jpeg";

class AsyncJpegStreamResponse : public AsyncAbstractResponse {
private:
  camera_frame_t _frame;
  size_t _index;
  size_t _jpg_buf_len;
  uint8_t *_jpg_buf;
  uint64_t lastAsyncRequest;

public:
  AsyncJpegStreamResponse() {
    _callback = nullptr;
    _code = 200;
    _contentLength = 0;
    _contentType = STREAM_CONTENT_TYPE;
    _sendContentLength = false;
    _chunked = true;
    _index = 0;
    _jpg_buf_len = 0;
    _jpg_buf = NULL;
    lastAsyncRequest = 0;
    memset(&_frame, 0, sizeof(camera_frame_t));
  }

  ~AsyncJpegStreamResponse() {
    if (_frame.fb) {
      if (_frame.fb->format != PIXFORMAT_JPEG) {
        free(_jpg_buf);
      }
      esp_camera_fb_return(_frame.fb);
    }
  }

  bool _sourceValid() const {
    return true;
  }

  virtual size_t _fillBuffer(uint8_t *buf, size_t maxLen) override {
    size_t ret = _content(buf, maxLen, _index);
    if (ret != RESPONSE_TRY_AGAIN) {
      _index += ret;
    }
    return ret;
  }

  size_t _content(uint8_t *buffer, size_t maxLen, size_t index) {
    if (!_frame.fb || _frame.index == _jpg_buf_len) {
      if (index && _frame.fb) {
        uint64_t end = (uint64_t)micros();
        int fp = (end - lastAsyncRequest) / 1000;
        lastAsyncRequest = end;

        if (_frame.fb->format != PIXFORMAT_JPEG) {
          free(_jpg_buf);
        }

        esp_camera_fb_return(_frame.fb);
        _frame.fb = NULL;
        _jpg_buf_len = 0;
        _jpg_buf = NULL;
      }

      if (maxLen < (strlen(STREAM_BOUNDARY) + strlen(STREAM_PART) + strlen(JPG_CONTENT_TYPE) + 8)) {
        return RESPONSE_TRY_AGAIN;
      }

      //get frame
      _frame.index = 0;
      _frame.fb = esp_camera_fb_get();

      if (_frame.fb == NULL) {
        log_e("Camera frame failed");
        return 0;
      }

      if (_frame.fb->format != PIXFORMAT_JPEG) {
        unsigned long st = millis();
        bool jpeg_converted = frame2jpg(_frame.fb, 80, &_jpg_buf, &_jpg_buf_len);

        if (!jpeg_converted) {
          log_e("JPEG compression failed");
          esp_camera_fb_return(_frame.fb);
          _frame.fb = NULL;
          _jpg_buf_len = 0;
          _jpg_buf = NULL;
          return 0;
        }

        log_i("JPEG: %lums, %uB", millis() - st, _jpg_buf_len);
      } else {
        _jpg_buf_len = _frame.fb->len;
        _jpg_buf = _frame.fb->buf;
      }

      //send boundary
      size_t blen = 0;
      if (index) {
        blen = strlen(STREAM_BOUNDARY);
        memcpy(buffer, STREAM_BOUNDARY, blen);
        buffer += blen;
      }

      //send header
      size_t hlen = sprintf((char *)buffer, STREAM_PART, JPG_CONTENT_TYPE, _jpg_buf_len);

      buffer += hlen;
      //send frame
      hlen = maxLen - hlen - blen;

      if (hlen > _jpg_buf_len) {
        maxLen -= hlen - _jpg_buf_len;
        hlen = _jpg_buf_len;
      }

      memcpy(buffer, _jpg_buf, hlen);
      _frame.index += hlen;
      return maxLen;
    }

    size_t available = _jpg_buf_len - _frame.index;

    if (maxLen > available) {
      maxLen = available;
    }

    memcpy(buffer, _jpg_buf + _frame.index, maxLen);
    _frame.index += maxLen;

    return maxLen;
  }
};

void initWiFi() {
  Serial.print("Connecting to WiFi");
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.print("\n");
  Serial.print("Connected. IP Address: ");
  Serial.print(WiFi.localIP());
  Serial.print("\n");
}

void initCamera() {
  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
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
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_VGA;
  config.jpeg_quality = 10;
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK) {
    Serial.printf("Failed to start the camera: 0x%x", err);
    ESP.restart();
  }

  else {
    sensor_t * s = esp_camera_sensor_get();

    s->set_brightness(s, 0);
    s->set_contrast(s, 0); 
    s->set_saturation(s, 0); 
    s->set_special_effect(s, 0);
    s->set_whitebal(s, 1);
    s->set_awb_gain(s, 1);
    s->set_wb_mode(s, 0); 
    s->set_exposure_ctrl(s, 1);
    s->set_aec2(s, 1); 
    s->set_ae_level(s, 1);
    s->set_aec_value(s, 1200);
    s->set_gain_ctrl(s, 1);
    s->set_agc_gain(s, 1);
    s->set_gainceiling(s, (gainceiling_t)4);
    s->set_bpc(s, 0);
    s->set_wpc(s, 0);
    s->set_raw_gma(s, 0);
    s->set_lenc(s, 0);
    s->set_hmirror(s, 0);
    s->set_vflip(s, 0);
    s->set_dcw(s, 1);
    s->set_colorbar(s, 0);

    Serial.println("Camera is ready.");
  }
}

void initServer() {
  socket.onEvent(handleSocketEvent);
  server.on("/stream", HTTP_GET, handleVideoStream);
  server.addHandler(&socket);
  server.begin();
  Serial.println("Server started.");
}

void handleVideoStream(AsyncWebServerRequest *request) {
  AsyncJpegStreamResponse *response = new AsyncJpegStreamResponse();

  if (!response) {
    request->send(501);
    return;
  }

  response->addHeader("Access-Control-Allow-Origin", "*");
  request->send(response);
}

void handleSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      handleCommand("0:0:0:0:0");
      break;
    case WS_EVT_DATA:
      handleSocketMessage(arg, data, len);
      break;
    case WS_EVT_ERROR:
      Serial.println("Websocket error");
      handleCommand("0:0:0:0:0");
      break;
  }
}

void handleSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo * info = (AwsFrameInfo*)arg;

  if(info->opcode == WS_TEXT){
    String command = "";

    for(size_t i=0; i < info->len; i++) {
      command += (char) data[i];
    }

    Serial.println(command);
    handleCommand(command);
  }
}

void handleCommand(String command) {
  int left     = getCommandValue(command, ':', 0).toInt();
  int right    = getCommandValue(command, ':', 1).toInt();
  int forward  = getCommandValue(command, ':', 2).toInt();
  int backward = getCommandValue(command, ':', 3).toInt();
  int flash    = getCommandValue(command, ':', 4).toInt();

  digitalWrite(MOTOR_2_PIN_1, left);
  digitalWrite(MOTOR_2_PIN_2, right);
  digitalWrite(MOTOR_1_PIN_1, forward);
  digitalWrite(MOTOR_1_PIN_2, backward);
  digitalWrite(FLASH_PIN,     flash);
}

String getCommandValue(String command, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = command.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (command.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }

  return found > index ? command.substring(strIndex[0], strIndex[1]) : "";
}

void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector

  Serial.begin(115200);
  Serial.setDebugOutput(false);

  // initialize digital pin as an output
  pinMode(MOTOR_1_PIN_1, OUTPUT);
  pinMode(MOTOR_1_PIN_2, OUTPUT);
  pinMode(MOTOR_2_PIN_1, OUTPUT);
  pinMode(MOTOR_2_PIN_2, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(FLASH_PIN, OUTPUT);

  // turn on the led
  digitalWrite(LED_PIN, LOW);

  // initialize wifi
  initWiFi();
  // initialize camera
  initCamera();
  // initialize server
  initServer();
}

void loop() {

}
