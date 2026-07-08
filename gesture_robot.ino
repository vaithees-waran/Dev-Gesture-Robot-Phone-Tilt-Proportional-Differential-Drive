/*************************************************************************
 *  LinkX Gesture Robot — Phone-Tilt Proportional Differential Drive
 *  Board   : Gbro STEM AI Robotics ESP32-S3 (LinkX Dev Board)
 *
 *  DESIGN NOTE: This build controls the rover using the PHONE's own
 *  built-in orientation sensors (read in the browser via the
 *  DeviceOrientation API) rather than a wearable MPU6050 — no extra
 *  hardware to wire, and it works the moment you open the dashboard.
 *  If you'd rather use a hand-worn MPU6050 instead, see the README
 *  "Alternative: wearable MPU6050" section for the 10-line swap.
 *
 *  FEATURES
 *   - AP dashboard @ 192.168.4.1, WebSocket-driven
 *   - Proportional differential drive: forward/back tilt = throttle,
 *     left/right tilt = steering, combined into independent left/right
 *     wheel PWM speeds (tank-style mixing)
 *   - Adjustable sensitivity + deadzone, client-side calibration
 *   - Dead-man safety: stops if no tilt update arrives within 400ms
 *   - RGB + buzzer feedback, LittleFS logging, OTA
 *
 *  LIBRARIES REQUIRED: ESPAsyncWebServer, AsyncTCP, ArduinoJson, LittleFS, ArduinoOTA
 *  SETUP: Upload /data to LittleFS BEFORE flashing this sketch.
 *************************************************************************/

#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <ArduinoOTA.h>

// ================= WIFI CONFIG =================
const char* AP_SSID     = "LinkX-GestureBot";
const char* AP_PASSWORD = "12345678";
const char* STA_SSID     = "";
const char* STA_PASSWORD = "";

// ================= MOTOR PINS (A = left wheel, B = right wheel) =================
#define IN1 11   // Left  dir 1
#define IN2 10   // Left  dir 2
#define PWMA 12  // Left  PWM
#define IN3 46   // Right dir 1
#define IN4 13   // Right dir 2
#define PWMB 3   // Right PWM

// ================= RGB + BUZZER =================
#define RED_PIN   45
#define GREEN_PIN 48
#define BLUE_PIN  47
#define BUZZER    14

const int pwmFreq = 5000;
const int pwmResolution = 8;
const int DEADZONE_DEG = 5;

// ================= STATE =================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

int leftSpeed = 0, rightSpeed = 0;   // -255..255, sign = direction
unsigned long lastTiltTime = 0;
const unsigned long DEADMAN_MS = 400;   // tighter than drive-pad projects: tilt streams continuously
bool wasMoving = false;

unsigned long lastBroadcast = 0;
const unsigned long BROADCAST_INTERVAL_MS = 150;

// ================= RGB / BUZZER =================
void setRGB(int r,int g,int b){ ledcWrite(RED_PIN,r); ledcWrite(GREEN_PIN,g); ledcWrite(BLUE_PIN,b); }
void rgbIdle(){ setRGB(0,60,0); }
void rgbMoving(){ setRGB(0,0,80); }
void rgbBoot(){ setRGB(80,0,80); }
void beep(int times,int onMs,int offMs){
  for(int i=0;i<times;i++){ digitalWrite(BUZZER,HIGH); delay(onMs); digitalWrite(BUZZER,LOW); if(i<times-1) delay(offMs); }
}

// ================= PROPORTIONAL MOTOR CONTROL =================
// speed: -255..255. Positive = forward, negative = backward, 0 = stop.
void setLeftMotor(int speed){
  speed = constrain(speed, -255, 255);
  if(speed >= 0){ digitalWrite(IN1,HIGH); digitalWrite(IN2,LOW); }
  else           { digitalWrite(IN1,LOW);  digitalWrite(IN2,HIGH); }
  ledcWrite(PWMA, abs(speed));
}
void setRightMotor(int speed){
  speed = constrain(speed, -255, 255);
  if(speed >= 0){ digitalWrite(IN3,HIGH); digitalWrite(IN4,LOW); }
  else           { digitalWrite(IN3,LOW);  digitalWrite(IN4,HIGH); }
  ledcWrite(PWMB, abs(speed));
}
void stopMotors(){ setLeftMotor(0); setRightMotor(0); }

// ================= LOGGING =================
void logEvent(const String &text){
  File f = LittleFS.open("/log.csv","a");
  if(!f) return;
  if(f.size() > 200000){
    f.close();
    LittleFS.remove("/log_old.csv");
    LittleFS.rename("/log.csv","/log_old.csv");
    f = LittleFS.open("/log.csv","a");
  }
  f.printf("%lu,%s\n", millis(), text.c_str());
  f.close();
}
void broadcastEvent(const String &text){
  logEvent(text);
  StaticJsonDocument<128> doc;
  doc["event"] = text;
  String out; serializeJson(doc, out);
  ws.textAll(out);
}

// ================= TILT -> DIFFERENTIAL DRIVE MIXING =================
void applyTilt(int betaDeg, int gammaDeg, int maxTilt){
  lastTiltTime = millis();
  if(maxTilt < 10) maxTilt = 35;

  // Deadzone
  if(abs(betaDeg) < DEADZONE_DEG) betaDeg = 0;
  if(abs(gammaDeg) < DEADZONE_DEG) gammaDeg = 0;

  int throttle = map(constrain(betaDeg, -maxTilt, maxTilt), -maxTilt, maxTilt, -255, 255);
  int steer    = map(constrain(gammaDeg, -maxTilt, maxTilt), -maxTilt, maxTilt, -255, 255);

  // Tank-style mixing: steer right = right wheel slows, left wheel speeds up (and vice versa)
  int left  = throttle + steer;
  int right = throttle - steer;
  left  = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  leftSpeed = left; rightSpeed = right;
  setLeftMotor(left);
  setRightMotor(right);

  bool moving = (left != 0 || right != 0);
  if(moving && !wasMoving){ rgbMoving(); digitalWrite(BUZZER,HIGH); delay(10); digitalWrite(BUZZER,LOW); }
  if(!moving && wasMoving){ rgbIdle(); }
  wasMoving = moving;
}

// ================= WEBSOCKET =================
void handleWsMessage(AsyncWebSocketClient *client, void *arg, uint8_t *data, size_t len){
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if(!(info->final && info->index==0 && info->len==len && info->opcode==WS_TEXT)) return;

  StaticJsonDocument<192> doc;
  if(deserializeJson(doc, data, len)) return;
  const char* cmd = doc["cmd"] | "";
  if(!strcmp(cmd,"tilt")){
    int beta  = doc["beta"]  | 0;
    int gamma = doc["gamma"] | 0;
    int maxTilt = doc["maxTilt"] | 35;
    applyTilt(beta, gamma, maxTilt);
  }
}
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len){
  if(type == WS_EVT_CONNECT){ broadcastEvent("Dashboard connected"); }
  else if(type == WS_EVT_DATA){ handleWsMessage(client, arg, data, len); }
}

// ================= SETUP =================
void setup(){
  Serial.begin(115200);
  delay(200);

  pinMode(IN1,OUTPUT); pinMode(IN2,OUTPUT); pinMode(IN3,OUTPUT); pinMode(IN4,OUTPUT);
  pinMode(BUZZER,OUTPUT);

  ledcAttach(PWMA,pwmFreq,pwmResolution);
  ledcAttach(PWMB,pwmFreq,pwmResolution);
  ledcAttach(RED_PIN,pwmFreq,pwmResolution);
  ledcAttach(GREEN_PIN,pwmFreq,pwmResolution);
  ledcAttach(BLUE_PIN,pwmFreq,pwmResolution);

  rgbBoot();
  stopMotors();

  if(!LittleFS.begin(true)) Serial.println("[ERR] LittleFS mount failed");
  else if(!LittleFS.exists("/log.csv")){
    File f = LittleFS.open("/log.csv","w");
    if(f){ f.println("millis,event"); f.close(); }
  }

  WiFi.mode(strlen(STA_SSID) ? WIFI_AP_STA : WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.print("[AP] IP: "); Serial.println(WiFi.softAPIP());
  if(strlen(STA_SSID)){
    WiFi.begin(STA_SSID, STA_PASSWORD);
    unsigned long start = millis();
    while(WiFi.status()!=WL_CONNECTED && millis()-start<8000) delay(300);
  }

  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");
  server.serveStatic("/log.csv", LittleFS, "/log.csv");
  server.onNotFound([](AsyncWebServerRequest *r){ r->send(404,"text/plain","Not found"); });
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();

  ArduinoOTA.setHostname("linkx-gesturebot");
  ArduinoOTA.onStart([](){ rgbBoot(); });
  ArduinoOTA.begin();

  beep(2,60,80);
  rgbIdle();
  logEvent("Boot complete");
  Serial.println("Connect to WiFi \"LinkX-GestureBot\" and open http://192.168.4.1");
}

// ================= LOOP =================
void loop(){
  ArduinoOTA.handle();
  ws.cleanupClients();
  unsigned long now = millis();

  // Dead-man safety: if tilt stream stops (screen locked, page closed), stop motors
  if((leftSpeed != 0 || rightSpeed != 0) && (now - lastTiltTime > DEADMAN_MS)){
    stopMotors();
    leftSpeed = 0; rightSpeed = 0;
    rgbIdle();
    broadcastEvent("Safety auto-stop (motion signal lost)");
  }

  if(now - lastBroadcast >= BROADCAST_INTERVAL_MS){
    lastBroadcast = now;
    StaticJsonDocument<128> doc;
    doc["left"] = leftSpeed;
    doc["right"] = rightSpeed;
    String out; serializeJson(doc, out);
    ws.textAll(out);
  }
}
