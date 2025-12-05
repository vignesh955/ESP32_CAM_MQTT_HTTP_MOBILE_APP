/*
  ESP32-CAM: WiFi + MQTT trigger + HTTP capture endpoint
  - Subscribe to topic: cam/commands/capture
  - Publish ready:    cam/status/ready  {"url":"http://<ip>/capture"}
  - Serve JPEG at:    http://<esp-ip>/capture

  Requirements:
  - Arduino core for ESP32
  - PubSubClient library
*/

#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include "esp_camera.h"
#include <WiFiClientSecure.h>

WiFiClientSecure wifiSecure;
PubSubClient mqttClient(wifiSecure);

// ---------------------------
// Camera pin configuration
// (Typical AI-Thinker / ESP32-CAM board)
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

#define LED_FLASH 4
#define BUZZER 14

// ---------------------------
// USER CONFIG - fill these
const char *WIFI_SSID = "IOT1";
const char *WIFI_PASS = "369369369";

const char *MQTT_BROKER = "0a50d954b7984f53a9c0d4e7ec990ed8.s1.eu.hivemq.cloud"; // or cloud broker domain
const uint16_t MQTT_PORT = 8883;
const char *MQTT_USER = "dhwani123";    // optional
const char *MQTT_PASSWD = "Dh12345678"; // optional

const char *MQTT_TOPIC_CMD = "esp32/cam/commands/capture";
const char *MQTT_TOPIC_READY = "esp32/cam/status/ready";
const char *MQTT_BEEP_TRIGGER = "esp32/cam/commands/beep";

const char hiveMQ_ca[] PROGMEM = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFGTCCBAGgAwIBAgISBY3q/k62gY9zzCKBwXeVZmRDMA0GCSqGSIb3DQEBCwUA
MDMxCzAJBgNVBAYTAlVTMRYwFAYDVQQKEw1MZXQncyBFbmNyeXB0MQwwCgYDVQQD
EwNSMTIwHhcNMjUxMDE5MTgxNDI5WhcNMjYwMTE3MTgxNDI4WjAfMR0wGwYDVQQD
DBQqLnMxLmV1LmhpdmVtcS5jbG91ZDCCASIwDQYJKoZIhvcNAQEBBQADggEPADCC
AQoCggEBAKVuz2sMPmxx2w/f81/YAEKTbNZMJPk2+ooLFg5hxXvReF+AwIT4XvZ+
MLhSKvFxmghJF+BB9WyhqrcJLGDCP4s6SOLWTYixEoTcaLUviqqn+06kYqDJ6E83
NGsc7T42DlPnzqcZZjPRed9rt4CP3RgeZlWyYZgiD8FoJG9gie8ytihF/FkGZT8T
N4Vkl2vQa3mfBWeeKrcuhcLPxqIWDz/30iYfLtEe5JYYScoCKTXcP9SUStjpR8pD
vfOWdvasOAuBy7yBbx01/4lcQt50hfbhTR/K14/D4rNkuuvU7ktSQnoxVXC8YDwG
zkny10DFt65mVYLNZcBQtOLHHOZGV30CAwEAAaOCAjkwggI1MA4GA1UdDwEB/wQE
AwIFoDAdBgNVHSUEFjAUBggrBgEFBQcDAQYIKwYBBQUHAwIwDAYDVR0TAQH/BAIw
ADAdBgNVHQ4EFgQUgsEjDU35+EWJKBsFxJ0lM0PXMi4wHwYDVR0jBBgwFoAUALUp
8i2ObzHom0yteD763OkM0dIwMwYIKwYBBQUHAQEEJzAlMCMGCCsGAQUFBzAChhdo
dHRwOi8vcjEyLmkubGVuY3Iub3JnLzAzBgNVHREELDAqghQqLnMxLmV1LmhpdmVt
cS5jbG91ZIISczEuZXUuaGl2ZW1xLmNsb3VkMBMGA1UdIAQMMAowCAYGZ4EMAQIB
MC4GA1UdHwQnMCUwI6AhoB+GHWh0dHA6Ly9yMTIuYy5sZW5jci5vcmcvNjguY3Js
MIIBBQYKKwYBBAHWeQIEAgSB9gSB8wDxAHcASZybad4dfOz8Nt7Nh2SmuFuvCoeA
GdFVUvvp6ynd+MMAAAGZ/eOokwAABAMASDBGAiEA5hLq4ze/NeLDJKSPI1IOnSpr
gWKanwFSky61clsLVtECIQDy35kv5Z20DEZaTUTRs0LTwOBtt4hCcsKfDGzfViY4
rQB2AJaXZL9VWJet90OHaDcIQnfp8DrV9qTzNm5GpD8PyqnGAAABmf3jqNEAAAQD
AEcwRQIgRXQhl7dwoxjkx7NWAR678ubZaB3fFXLAKdt1Yt3AxXYCIQDKyEyxTyRf
u54cglCG+XUyMDH3Sgyfqv6uK5OUWOAfbjANBgkqhkiG9w0BAQsFAAOCAQEAlcxa
UU01drpj2WUmbxkD076YtgR10JXLk0Lxy8K5dPd5DFFDcw5EiF8oDXGZtvQ91KEl
Ei+lqi7+oifj/lS7u15pMdg5n/EnCIHFysPixmLObRYNjJXpBgKHrVQspVReYxNe
ZwvtJKglvrvmHzLTLUHDeON9q9CkfUwJLHesI3R3fjlSxgpWwoJOnyQv+Csv1zHE
Nb/7JUJVk/vkWwZo/B/cnUc4WBmTD0RUeFKzKZzmy1gCtuAMeT+cPLraJU5zsKXt
pnpIucEvTexYuDbeuvnYckh2pqAALUKo23HJqAh1//HvxR7vkS87+xq/my4chgW+
UVH0XBkbReDupQk2xQ==
-----END CERTIFICATE-----
)EOF";

const char *root_ca = R"EOF(
-----BEGIN CERTIFICATE-----
MIIFBjCCAu6gAwIBAgIRAMISMktwqbSRcdxA9+KFJjwwDQYJKoZIhvcNAQELBQAw
TzELMAkGA1UEBhMCVVMxKTAnBgNVBAoTIEludGVybmV0IFNlY3VyaXR5IFJlc2Vh
cmNoIEdyb3VwMRUwEwYDVQQDEwxJU1JHIFJvb3QgWDEwHhcNMjQwMzEzMDAwMDAw
WhcNMjcwMzEyMjM1OTU5WjAzMQswCQYDVQQGEwJVUzEWMBQGA1UEChMNTGV0J3Mg
RW5jcnlwdDEMMAoGA1UEAxMDUjEyMIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIB
CgKCAQEA2pgodK2+lP474B7i5Ut1qywSf+2nAzJ+Npfs6DGPpRONC5kuHs0BUT1M
5ShuCVUxqqUiXXL0LQfCTUA83wEjuXg39RplMjTmhnGdBO+ECFu9AhqZ66YBAJpz
kG2Pogeg0JfT2kVhgTU9FPnEwF9q3AuWGrCf4yrqvSrWmMebcas7dA8827JgvlpL
Thjp2ypzXIlhZZ7+7Tymy05v5J75AEaz/xlNKmOzjmbGGIVwx1Blbzt05UiDDwhY
XS0jnV6j/ujbAKHS9OMZTfLuevYnnuXNnC2i8n+cF63vEzc50bTILEHWhsDp7CH4
WRt/uTp8n1wBnWIEwii9Cq08yhDsGwIDAQABo4H4MIH1MA4GA1UdDwEB/wQEAwIB
hjAdBgNVHSUEFjAUBggrBgEFBQcDAgYIKwYBBQUHAwEwEgYDVR0TAQH/BAgwBgEB
/wIBADAdBgNVHQ4EFgQUALUp8i2ObzHom0yteD763OkM0dIwHwYDVR0jBBgwFoAU
ebRZ5nu25eQBc4AIiMgaWPbpm24wMgYIKwYBBQUHAQEEJjAkMCIGCCsGAQUFBzAC
hhZodHRwOi8veDEuaS5sZW5jci5vcmcvMBMGA1UdIAQMMAowCAYGZ4EMAQIBMCcG
A1UdHwQgMB4wHKAaoBiGFmh0dHA6Ly94MS5jLmxlbmNyLm9yZy8wDQYJKoZIhvcN
AQELBQADggIBAI910AnPanZIZTKS3rVEyIV29BWEjAK/duuz8eL5boSoVpHhkkv3
4eoAeEiPdZLj5EZ7G2ArIK+gzhTlRQ1q4FKGpPPaFBSpqV/xbUb5UlAXQOnkHn3m
FVj+qYv87/WeY+Bm4sN3Ox8BhyaU7UAQ3LeZ7N1X01xxQe4wIAAE3JVLUCiHmZL+
qoCUtgYIFPgcg350QMUIWgxPXNGEncT921ne7nluI02V8pLUmClqXOsCwULw+PVO
ZCB7qOMxxMBoCUeL2Ll4oMpOSr5pJCpLN3tRA2s6P1KLs9TSrVhOk+7LX28NMUlI
usQ/nxLJID0RhAeFtPjyOCOscQBA53+NRjSCak7P4A5jX7ppmkcJECL+S0i3kXVU
y5Me5BbrU8973jZNv/ax6+ZK6TM8jWmimL6of6OrX7ZU6E2WqazzsFrLG3o2kySb
zlhSgJ81Cl4tv3SbYiYXnJExKQvzf83DYotox3f0fwv7xln1A2ZLplCb0O+l/AK0
YE0DS2FPxSAHi0iwMfW2nNHJrXcY3LLHD77gRgje4Eveubi2xxa+Nmk/hmhLdIET
iVDFanoCrMVIpQ59XWHkzdFmoHXHBV7oibVjGSO7ULSQ7MJ1Nz51phuDJSgAIU7A
0zrLnOrAj/dfrlEWRhCvAgbuwLZX1A2sjNjXoPOHbsPiy+lO1KF8/XY7
-----END CERTIFICATE-----
)EOF";

// ---------------------------
// Globals
WebServer server(80);
// WiFiClient wifiClient;
// PubSubClient mqttClient(wifiClient);

camera_fb_t *last_fb = nullptr; // pointer to last captured frame (must be returned)
unsigned long lastMqttReconnectAttempt = 0;
unsigned long lastWiFiReconnectAttempt = 0;

void alertBeep()
{
  digitalWrite(BUZZER, HIGH);
  delay(2000);
  digitalWrite(BUZZER, LOW);
}
// ---------------------------
// Camera init
void initCamera()
{
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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Tune these for reliability / speed
  // config.frame_size = FRAMESIZE_VGA; // use QVGA for faster, VGA for better quality
  config.frame_size = FRAMESIZE_SXGA;
  config.jpeg_quality = 12; // 10-12 is reasonable. Lower number = better quality but more bytes
  config.fb_count = 1;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.printf("Camera init failed: 0x%x\n", err);
    // halt here — can't proceed without camera
    while (true)
    {
      delay(1000);
    }
  }
}

void handleCapture()
{
  camera_fb_t *fb = nullptr;

  // If we already have a pre-captured frame
  // if (last_fb)
  // {
  //   fb = last_fb;
  //   last_fb = nullptr; // consume it
  // }
  // else
  // {

  // camera_fb_t *fb = esp_camera_fb_get();

  digitalWrite(LED_FLASH, HIGH);
  delay(200);
  fb = esp_camera_fb_get();
  digitalWrite(LED_FLASH, LOW);
  if (!fb)
  {
    server.send(500, "text/plain", "Camera capture failed");
    Serial.println("ERROR: capture failed on-demand");
    return;
  }
  // }

  // Correct HTTP headers
  server.setContentLength(fb->len);
  server.send(200, "image/jpeg");

  // Write JPEG bytes
  server.client().write(fb->buf, fb->len);

  // Now safe to release frame buffer
  esp_camera_fb_return(fb);

  Serial.println("Served capture successfully.");
}

// ---------------------------
// Helper: publish ready JSON with URL
void publishReadyUrl()
{
  IPAddress ip = WiFi.localIP();
  if (WiFi.status() != WL_CONNECTED)
    return;

  char urlBuf[128];
  snprintf(urlBuf, sizeof(urlBuf), "http://%u.%u.%u.%u/capture", ip[0], ip[1], ip[2], ip[3]);

  // JSON payload simple (no dependency)
  char payload[256];
  snprintf(payload, sizeof(payload), "{\"url\":\"%s\"}", urlBuf);

  mqttClient.publish(MQTT_TOPIC_READY, payload);
  Serial.printf("Published ready: %s\n", payload);
}

// ---------------------------
// MQTT message callback
void mqttCallback(char *topic, byte *payload, unsigned int length)
{
  Serial.printf("MQTT msg on %s\n", topic);
  // Note: payload isn't null terminated
  String msg;
  for (unsigned int i = 0; i < length; i++)
    msg += (char)payload[i];
  msg.trim();

  if (String(topic) == String(MQTT_TOPIC_CMD))
  {
    // Accept simple payloads like "capture" or empty; we'll capture regardless
    Serial.println("Capture command received via MQTT. Capturing image...");

    // If an earlier frame exists, free it first
    // if (last_fb)
    // {
    //   esp_camera_fb_return(last_fb);
    //   last_fb = nullptr;
    // }

    // digitalWrite(LED_FLASH, HIGH);
    // delay(200);
    // camera_fb_t *fb = esp_camera_fb_get();
    // digitalWrite(LED_FLASH, LOW);
    // if (!fb)
    // {
    //   Serial.println("ERROR: Camera capture failed in MQTT handler");
    //   return;
    // }

    // // Store for serving; DO NOT return fb now
    // last_fb = fb;

    handleCapture();
    // Publish URL (so app can GET)
    publishReadyUrl();
  }
  else if (String(topic) == String(MQTT_BEEP_TRIGGER))
  {
    Serial.println("Trigger command received from Mobile app");
    alertBeep();
  }
}

// ---------------------------
// MQTT connect (blocking-ish with short timeout)
boolean mqttConnect()
{
  if (!WiFi.isConnected())
    return false;

  Serial.print("Connecting to MQTT broker...");
  if (strlen(MQTT_USER) == 0)
  {
    if (mqttClient.connect("esp32cam-client"))
    {
      Serial.println("connected");
      return true;
    }
  }
  else
  {
    if (mqttClient.connect("esp32cam-client", MQTT_USER, MQTT_PASSWD))
    {
      Serial.println("connected (auth)");
      return true;
    }
  }
  Serial.printf("failed, rc=%d\n", mqttClient.state());
  return false;
}

// ---------------------------
// HTTP /capture handler
// void handleCapture()
// {
//   // If we have a previously-captured frame from MQTT trigger, serve that
//   if (last_fb)
//   {
//     server.sendHeader("Content-Type", "image/jpeg");
//     server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
//     server.sendHeader("Content-Length", String(last_fb->len));

//     //server.send(200);

//     WiFiClient client = server.client();
//     client.write(last_fb->buf, last_fb->len);

//     // Return and free
//     esp_camera_fb_return(last_fb);
//     last_fb = nullptr;
//     Serial.println("Served stored capture.");
//     return;
//   }

//   // Else capture on-demand
//   camera_fb_t *fb = esp_camera_fb_get();
//   if (!fb)
//   {
//     server.send(500, "text/plain", "Camera capture failed");
//     Serial.println("ERROR: capture failed on-demand");
//     return;
//   }

//   server.sendHeader("Content-Type", "image/jpeg");
//   server.sendHeader("Content-Disposition", "inline; filename=capture.jpg");
//   server.sendHeader("Content-Length", String(fb->len));

//   //server.send(200);

//   WiFiClient client = server.client();
//   client.write(fb->buf, fb->len);

//   esp_camera_fb_return(fb);
//   Serial.println("Served on-demand capture.");
// }

// ---------------------------
// Setup server routes and start
void startHttpServer()
{
  server.on("/capture", HTTP_GET, handleCapture);
  server.begin();
  Serial.println("HTTP server started.");
}

// ---------------------------
// Setup
void setup()
{
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_FLASH, OUTPUT);
  digitalWrite(LED_FLASH, LOW);
  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
  // Start Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 20000)
  {
    delay(300);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.print("WiFi connected. IP: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("WiFi connect failed. Continue anyway (check credentials).");
  }

  // Init camera
  initCamera();

  // MQTT setup
  wifiSecure.setCACert(root_ca);
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  // Start HTTP server last
  startHttpServer();
}

// ---------------------------
// Loop: ensure Wi-Fi & MQTT connectivity, handle client loops
void loop()
{
  // Wi-Fi reconnect if needed (non-blocking check)
  if (WiFi.status() != WL_CONNECTED)
  {
    unsigned long now = millis();
    if (now - lastWiFiReconnectAttempt > 5000)
    {
      lastWiFiReconnectAttempt = now;
      Serial.println("Trying WiFi reconnect...");
      WiFi.disconnect();
      WiFi.begin(WIFI_SSID, WIFI_PASS);
    }
  }

  // MQTT connect if needed
  if (!mqttClient.connected())
  {
    unsigned long now = millis();
    if (now - lastMqttReconnectAttempt > 3000)
    {
      lastMqttReconnectAttempt = now;
      if (mqttConnect())
      {
        // subscribe after connect
        mqttClient.subscribe(MQTT_TOPIC_CMD);
        Serial.printf("Subscribed to %s\n", MQTT_TOPIC_CMD);
        mqttClient.subscribe(MQTT_BEEP_TRIGGER);
        Serial.printf("Subscribed to %s\n", MQTT_BEEP_TRIGGER);
      }
    }
  }
  else
  {
    mqttClient.loop(); // process incoming messages
  }

  server.handleClient();
  delay(10);
}
