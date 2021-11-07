#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include "credentials.h"
#include "settings.h"

#include <Arduino.h>
#include <ServoEasing.hpp>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ESP32Servo.h>

#define X_MICROS_0_DEG 600
#define X_MICROS_180_DEG 2400
#define Y_MICROS_0_DEG 600
#define Y_MICROS_180_DEG 2400

#define TOUCH_PIN T0
#define TOUCH_THRESHOLD 5

Preferences preferences;

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;

AsyncWebServer webServer(80);

const int ledPin = 2;
const int potPin = 32;
const int strobePin = 35;
const int sirenPin = 23;
const int laserPin = 33;
const int xServoPin = 18;
const int yServoPin = 19;

int touchVal;
int potMin;
int potMax;
int xMinDegree, xMaxDegree;
int yMinDegree, yMaxDegree;

struct ServoControlStruct {
  uint16_t minDegree;
  uint16_t maxDegree;
};
ServoControlStruct ServoXControl;
ServoControlStruct ServoYControl;

ServoEasing ServoX;
ServoEasing ServoY;

bool strobeActive = false;
bool sirenActive = false;
bool laserActive = false;

uint32_t strobeStart;
uint32_t sirenStart;
uint32_t laserStart;

uint16_t strobeDuration;
uint16_t sirenDuration;
uint16_t laserDuration;

uint32_t laserMoveLast = 0;
uint16_t laserRandomDelay = random(500);

uint32_t retryConnectLast = 0;
uint16_t retryConnectInterval = 5000;

IPAddress mqttServer(192, 168, 1, 6);

// MQTT Callback Function Declaration
void mqttCallback(char* topic, byte* payload, uint8_t length);

WiFiClient wifiClient;
PubSubClient mqttClient(mqttServer, 1883, mqttCallback, wifiClient);

void mqttCallback(char* topic, byte* payload, uint8_t length) {
  Serial.print(F("MQTT message received ["));
  Serial.print(topic);
  Serial.print(F("] (length="));
  Serial.print(length);
  Serial.print(F(") "));
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

boolean mqttReconnect() {
  if (mqttClient.connect(MQTT_CLIENT, MQTT_USER, MQTT_PASS, MQTT_WILL_TOPIC, MQTT_WILL_QOS, MQTT_WILL_RETAIN, MQTT_WILL_MESSAGE)) {
    mqttClient.publish(MQTT_OUT_TOPIC, MQTT_PUB_MESSAGE);
    mqttClient.subscribe(MQTT_IN_TOPIC);
  }
  return mqttClient.connected();
}

/*void mqttReconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting MQTT connection...");

    if (mqttClient.connect("arduinoClient")) {
      Serial.println("connected");
      mqttClient.publish("outTopic", "hello world");
      mqttClient.subscribe("inTopic");
    }
    else {
      mqttClient.print("failed, rc=");
      mqttClient.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
  }*/

void blinkLed(bool blinkOn) {
  if (blinkOn == true) {
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
  }
  else {
    digitalWrite(ledPin, LOW);
    delay(100);
    digitalWrite(ledPin, HIGH);
  }
}

void calibratePot() {
  Serial.println(F("Calibrating potentiometer."));

  bool minSet = false;
  bool maxSet = false;

  Serial.println(F("Move potentiometer to MIN position then tap touch sensor to confirm."));
  delay(7500);
  while (minSet == false) {
    touchVal = touchRead(TOUCH_PIN);
    Serial.print(F("touchVal: ")); Serial.println(touchVal);

    if (touchVal < TOUCH_THRESHOLD) {
      potMin = analogRead(potPin);
      Serial.print(F("potMin: ")); Serial.println(potMin);
      minSet = true;
      blinkLed(false);
      while (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
        delay(100);
      }
      delay(1000);
    }
    delay(250);
  }
  preferences.putInt("potMin", potMin);

  delay(5000);
  blinkLed(false);

  Serial.println(F("Move potentiometer to MAX position then tap touch sensor to confirm."));
  delay(7500);
  while (maxSet == false) {
    touchVal = touchRead(TOUCH_PIN);
    Serial.print(F("touchVal: ")); Serial.println(touchVal);

    if (touchVal < TOUCH_THRESHOLD) {
      potMax = analogRead(potPin);
      Serial.print(F("potMax: ")); Serial.println(potMax);
      maxSet = true;
      blinkLed(false);
      while (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
        delay(100);
      }
      delay(1000);
    }
    delay(250);
  }
  preferences.putInt("potMax", potMax);
}

void setServoRange() {
  Serial.println(F("Calibrating servos."));

  bool xMinSet = false;
  bool xMaxSet = false;
  bool yMinSet = false;
  bool yMaxSet = false;

  while (xMinSet == false) {
    int servoVal = map(analogRead(potPin), potMin, potMax, 0, 180);
    ServoX.easeTo(servoVal, 50);

    touchVal = touchRead(TOUCH_PIN);
    Serial.print(F("touchVal: ")); Serial.println(touchVal);

    if (touchVal < TOUCH_THRESHOLD) {
      xMinDegree = servoVal;
      Serial.print(F("xMinDegree: ")); Serial.println(xMinDegree);
      xMinSet = true;
      blinkLed(false);
      while (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
        delay(100);
      }
      delay(1000);
    }
    delay(250);
  }

  delay(5000);
  blinkLed(false);

  while (xMaxSet == false) {
    int servoVal = map(analogRead(potPin), potMin, potMax, 0, 180);
    ServoX.easeTo(servoVal, 50);

    touchVal = touchRead(TOUCH_PIN);
    Serial.print(F("touchVal: ")); Serial.println(touchVal);

    if (touchVal < TOUCH_THRESHOLD) {
      xMaxDegree = servoVal;
      Serial.print(F("xMaxDegree: ")); Serial.println(xMaxDegree);
      xMaxSet = true;
      blinkLed(false);
      while (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
        delay(100);
      }
      delay(1000);
    }
    delay(250);
  }

  delay(5000);
  blinkLed(false);

  while (yMinSet == false) {
    int servoVal = map(analogRead(potPin), potMin, potMax, 0, 180);
    ServoY.easeTo(servoVal, 50);

    touchVal = touchRead(TOUCH_PIN);
    Serial.print(F("touchVal: ")); Serial.println(touchVal);

    if (touchVal < TOUCH_THRESHOLD) {
      yMinDegree = servoVal;
      Serial.print(F("yMinDegree: ")); Serial.println(yMinDegree);
      yMinSet = true;
      blinkLed(false);
      while (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
        delay(100);
      }
      delay(1000);
    }
    delay(250);
  }

  delay(5000);
  blinkLed(false);

  while (yMaxSet == false) {
    int servoVal = map(analogRead(potPin), potMin, potMax, 0, 180);
    ServoY.easeTo(servoVal, 50);

    touchVal = touchRead(TOUCH_PIN);
    Serial.print(F("touchVal: ")); Serial.println(touchVal);

    if (touchVal < TOUCH_THRESHOLD) {
      yMaxDegree = servoVal;
      Serial.print(F("yMaxDegree: ")); Serial.println(yMaxDegree);
      yMaxSet = true;
      blinkLed(false);
      while (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
        delay(100);
      }
      delay(1000);
    }
    delay(250);
  }
}

void setup(void) {
  // Built-in LED
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Strobe light
  pinMode(strobePin, OUTPUT);
  digitalWrite(strobePin, LOW);

  // Siren
  pinMode(sirenPin, OUTPUT);
  digitalWrite(sirenPin, LOW);

  // Laser
  pinMode(laserPin, OUTPUT);
  //digitalWrite(laserPin, HIGH);
  analogWrite(laserPin, 0);

  // Servo (X-axis)
  pinMode(xServoPin, OUTPUT);

  // Servo (Y-axis)
  pinMode(yServoPin, OUTPUT);

  Serial.begin(115200);
  Serial.println();

  preferences.begin("beanbag", false);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.println();
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(F("."));
  }
  Serial.println();
  Serial.print(F("Connected to "));
  Serial.println(ssid);
  Serial.print(F("IP address: "));
  Serial.println(WiFi.localIP());

  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", "Demonic Beanbag Neutralizer v0.1");
  });

  AsyncElegantOTA.begin(&webServer);    // Start ElegantOTA
  webServer.begin();
  Serial.println("HTTP server started");

  //mqttClient.setServer(mqttServer, 1883);
  //mqttClient.setCallback(mqttCallback);

  while (!mqttClient.connected()) {
    delay(5000);
    //bool mqttConnectResult = mqttReconnect();
    Serial.print(F("mqttConnectResult: ")); Serial.println(mqttReconnect());
  }

  potMin = preferences.getInt("potMin", -1);
  potMax = preferences.getInt("potMax", -1);

  bool potCalibRequired = false;

  if (potMin == -1 || potMax == -1) {
    potCalibRequired = true;
  }

  else {
    for (int x = 0; x < 10; x++) {
      blinkLed(true);
      delay(100);
    }
    delay(1000);

    if (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
      blinkLed(true);
      while (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
        delay(100);
      }
      potCalibRequired = true;
    }
  }

  if (potCalibRequired == true) {
    for (int x = 0; x < 3; x++) {
      digitalWrite(ledPin, HIGH);
      delay(500);
      digitalWrite(ledPin, LOW);
      delay(500);
    }

    digitalWrite(ledPin, HIGH);
    calibratePot();
    digitalWrite(ledPin, LOW);

    Serial.println(F("Restarting ESP32..."));
    delay(1000);
    ESP.restart();
  }

  for (int x = 0; x < 10; x++) {
    blinkLed(true);
    delay(100);
  }
  delay(1000);

  bool servoSetRequired = false;

  xMinDegree = preferences.getInt("xMinDegree", -1);
  xMaxDegree = preferences.getInt("xMaxDegree", -1);
  yMinDegree = preferences.getInt("yMinDegree", -1);
  yMaxDegree = preferences.getInt("yMaxDegree", -1);

  if (
    xMinDegree == -1 ||
    xMaxDegree == -1 ||
    yMinDegree == -1 ||
    yMaxDegree == -1
  ) {
    servoSetRequired = true;
  }

  else if (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
    blinkLed(true);
    while (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
      delay(100);
    }
    servoSetRequired = true;
  }

  /*
      Servo attach parameters:
      attach(int aPin, int aInitialDegree, int aMicrosecondsForServo0Degree, int aMicrosecondsForServo180Degree)
  */
  Serial.print(F("Attaching X servo at pin "));
  Serial.println(xServoPin);
  if (ServoX.attach(xServoPin, 90, X_MICROS_0_DEG, X_MICROS_180_DEG) == INVALID_SERVO) {
    Serial.println(F("error attaching X servo."));
  }
  else Serial.println(F("successfully mounted."));

  Serial.print(F("Attaching Y servo at pin "));
  Serial.println(yServoPin);
  if (ServoY.attach(yServoPin, 90, Y_MICROS_0_DEG, Y_MICROS_180_DEG) == INVALID_SERVO) {
    Serial.println(F("error attaching Y servo."));
    while (true) {
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
      delay(100);
    }
  }
  else Serial.println(F("successfully mounted."));

  if (servoSetRequired == true) {
    for (int x = 0; x < 5; x++) {
      digitalWrite(ledPin, HIGH);
      delay(500);
      digitalWrite(ledPin, LOW);
      delay(500);
    }

    digitalWrite(ledPin, HIGH);
    setServoRange();
    digitalWrite(ledPin, LOW);

    preferences.putInt("xMinDegree", xMinDegree);
    preferences.putInt("xMaxDegree", xMaxDegree);
    preferences.putInt("yMinDegree", yMinDegree);
    preferences.putInt("yMaxDegree", yMaxDegree);

    Serial.println(F("Restarting ESP32..."));
    delay(1000);
    ESP.restart();
  }

  ServoXControl.minDegree = xMinDegree;
  ServoXControl.maxDegree = xMaxDegree;
  ServoYControl.minDegree = yMinDegree;
  ServoYControl.maxDegree = yMaxDegree;

  Serial.print(F("ServoXControl.minDegree: ")); Serial.println(ServoXControl.minDegree);
  Serial.print(F("ServoXControl.maxDegree: ")); Serial.println(ServoXControl.maxDegree);
  Serial.print(F("ServoYControl.minDegree: ")); Serial.println(ServoYControl.minDegree);
  Serial.print(F("ServoYControl.maxDegree: ")); Serial.println(ServoYControl.maxDegree);

  Serial.println(F("Marking border of laser area."));
  ServoX.write(ServoXControl.minDegree);
  ServoY.write(ServoYControl.minDegree);

  delay(1000);

  analogWrite(laserPin, 255);
  ServoY.easeTo(ServoYControl.maxDegree, 50);
  ServoX.easeTo(ServoXControl.maxDegree, 50);
  ServoY.easeTo(ServoYControl.minDegree, 50);
  ServoX.easeTo(ServoXControl.minDegree, 50);
  analogWrite(laserPin, 0);

  delay(4000);

  digitalWrite(ledPin, HIGH);
}

uint8_t getRandomValue(ServoControlStruct * aServoControlStruct, ServoEasing * aServoEasing) {
  uint8_t tNewTargetAngle;
  do {
    tNewTargetAngle = random(aServoControlStruct->minDegree, aServoControlStruct->maxDegree);
  } while (tNewTargetAngle == aServoEasing->MicrosecondsOrUnitsToDegree(aServoEasing->mCurrentMicrosecondsOrUnits)); // do not accept current angle as new value
  return tNewTargetAngle;
}

void loop(void) {
  if (!mqttClient.connected()) {
    uint32_t msNow = millis();
    Serial.println(F("MQTT connection lost. Attempting to reconnect..."));
    if ((msNow - retryConnectLast) > retryConnectInterval) {
      if (mqttReconnect()) retryConnectLast = 0;
      else retryConnectLast = msNow;
    }
  }
  else {
    mqttClient.loop();

    if (laserActive) {
      if (!ServoX.isMoving()) {
        //delay(random(500));
        uint16_t laserRandomElapsed = millis() - laserMoveLast;
        Serial.print(F("laserRandomElapsed: ")); Serial.println(laserRandomElapsed);

        if (laserRandomElapsed > laserRandomDelay) {

          uint8_t tNewHorizontal = getRandomValue(&ServoXControl, &ServoX);
          uint8_t tNewVertical = getRandomValue(&ServoYControl, &ServoY);
          int tSpeed = random(10, 90);

          Serial.print(F("Move to horizontal="));
          Serial.print(tNewHorizontal);
          Serial.print(F(" vertical="));
          Serial.print(tNewVertical);
          Serial.print(F(" speed="));
          Serial.println(tSpeed);
          ServoX.setEaseTo(tNewHorizontal, tSpeed);
          ServoY.setEaseTo(tNewVertical, tSpeed);
          synchronizeAllServosAndStartInterrupt();

          laserRandomDelay = random(500);
          Serial.print(F("laserRandomDelay: ")); Serial.println(laserRandomDelay);
          laserMoveLast = millis();
        }
      }
    }
  }

  checkTimers();
}

void checkTimers() {
  /*
    bool strobeActive = false;
    bool sirenActive = false;
    bool laserActive = false;

    uint32_t strobeStart;
    uint32_t sirenStart;
    uint32_t laserStart;

    uint16_t strobeDuration;
    uint16_t sirenDuration;
    uint16_t laserDuration;
  */
  uint32_t msNow = millis();

  if (laserActive) {
    if ((msNow - laserStart) > laserDuration) {
      laserActive = false;
      analogWrite(laserPin, 0);
      Serial.println(F("Laser disabled."));
    }
  }

  if (sirenActive) {
    if ((msNow - sirenStart) > sirenDuration) {
      sirenActive = false;
      analogWrite(sirenPin, 0);
      Serial.println(F("Siren disabled."));
    }
  }

  if (strobeActive) {
    if ((msNow - strobeStart) > strobeDuration) {
      strobeActive = false;
      digitalWrite(strobePin, LOW);
      Serial.println(F("Strobe disabled."));
    }
  }
}
