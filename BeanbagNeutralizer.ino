#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include "credentials.h"

#include <Arduino.h>
#include <ServoEasing.hpp>
#include <Preferences.h>
#include <PubSubClient.h>

#define X_MICROS_0_DEG 0
#define X_MICROS_180_DEG 2500
#define Y_MICROS_0_DEG 0
#define Y_MICROS_180_DEG 2500

#define TOUCH_PIN T0
#define TOUCH_THRESHOLD 40

Preferences preferences;

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;

AsyncWebServer webServer(80);

const int ledPin = 2;
const int potPin = 32;
const int strobePin = 33;
const int sirenPin = 23;
const int laserPin = 25; // A0
const int xServoPin = 5;
const int yServoPin = 18;

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

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

IPAddress mqttServer(192, 168, 1, 6);

void mqttReconnect() {
  while (!mqttClient.connected()) {
    Serial.print(F("Attempting MQTT connection..."));

    if (mqttClient.connect("arduinoClient")) {
      Serial.println(F("connected"));
      mqttClient.publish("outTopic", "hello world");
      mqttClient.subscribe("inTopic");
    }
    else {
      mqttClient.print("failed, rc=");
      mqttClient.print(mqttClient.state());
      Serial.println(F(" try again in 5 seconds"));
      delay(5000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, uint8_t length) {
  Serial.print(F("MQTT message received ["));
  Serial.print(topic);
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void calibratePot() {
  Serial.println(F("Calibrating potentiometer."));

  bool minSet = false;
  bool maxSet = false;

  Serial.println(F("Move potentiometer to MIN position then tap touch sensor to confirm."));
  while (minSet == false) {
    if (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
      while (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
        delay(10);
      }
      potMin = analogRead(potPin);
      minSet = true;
    }
  }
  preferences.putInt("potMin", potMin);

  Serial.println(F("Move potentiometer to MAX position then tap touch sensor to confirm."));
  while (maxSet == false) {
    if (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
      while (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
        delay(10);
      }
      potMax = analogRead(potPin);
      maxSet = true;
    }
  }
  preferences.putInt("potMax", potMax);
}

void setServoRango() {
  Serial.println(F("Calibrating servos."));

  bool xMinSet = false;
  bool xMaxSet = false;
  bool yMinSet = false;
  bool yMaxSet = false;

  int servoPos;

  Serial.println(F("Move servo to minimum X value. (Left boundary)"));
  while (xMinSet == false) {
    servoPos = map(analogRead(potPin), potMin, potMax, 0, 270);
    ServoX.write(servoPos);
    delay(10);
    if (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
      xMinDegree = servoPos;
      xMinSet = true;
      while (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
        delay(10);
      }
      Serial.print(F("xMinDegree: ")); Serial.println(xMinDegree);
    }
  }

  Serial.println(F("Move servo to maximum X value. (Right boundary)"));
  while (xMaxSet == false) {
    servoPos = map(analogRead(potPin), potMin, potMax, 0, 270);
    ServoX.write(servoPos);
    delay(10);
    if (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
      xMaxDegree = servoPos;
      xMaxSet = true;
      while (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
        delay(10);
      }
      Serial.print(F("xMaxDegree: ")); Serial.println(xMaxDegree);
    }
  }

  Serial.println(F("Move servo to minimum Y value. (Bottom boundary)"));
  while (yMinSet == false) {
    servoPos = map(analogRead(potPin), potMin, potMax, 0, 270);
    ServoY.write(servoPos);
    delay(10);
    if (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
      yMinDegree = servoPos;
      yMinSet = true;
      while (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
        delay(10);
      }
      Serial.print(F("yMinDegree: ")); Serial.println(yMinDegree);
    }
  }

  Serial.println(F("Move servo to maximum Y value. (Top boundary)"));
  while (yMaxSet == false) {
    servoPos = map(analogRead(potPin), potMin, potMax, 0, 270);
    ServoY.write(servoPos);
    delay(10);
    if (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
      yMaxDegree = servoPos;
      yMaxSet = true;
      while (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
        delay(10);
      }
      Serial.print(F("yMaxDegree: ")); Serial.println(yMaxDegree);
    }
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
  digitalWrite(laserPin, HIGH);

  // Servo (X-axis)
  pinMode(xServoPin, OUTPUT);

  // Servo (Y-axis)
  pinMode(yServoPin, OUTPUT);

  Serial.begin(115200);
  Serial.println();

  preferences.begin("beanbag", false);

  potMin = preferences.getInt("potMin", -1);
  potMax = preferences.getInt("potMax", -1);

  bool potCalibRequired = false;
  if (potMin == -1 || potMax == -1) {
    potCalibRequired = true;
  }

  delay(1000);

  if (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    while (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
      delay(10);
    }
    potCalibRequired = true;
  }

  if (potCalibRequired == true) {
    digitalWrite(ledPin, HIGH);
    calibratePot();
    digitalWrite(ledPin, LOW);

    Serial.println(F("Restarting ESP32..."));
    delay(1000);
    ESP.restart();
  }

  for (int x = 0; x < 10; x++) {
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    delay(100);
  }
  //delay(1000);

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
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    while (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
      delay(10);
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
    Serial.println(F("Error attaching servo"));
  }

  Serial.print(F("Attaching Y servo at pin "));
  Serial.println(yServoPin);
  if (ServoY.attach(yServoPin, 90, Y_MICROS_0_DEG, Y_MICROS_180_DEG) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo"));
    while (true) {
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
      delay(100);
    }
  }

  if (servoSetRequired == true) {
    digitalWrite(ledPin, HIGH);
    setServoRango();
    digitalWrite(ledPin, LOW);

    //Serial.println(F("Restarting ESP32..."));
    //delay(1000);
    //ESP.restart();
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
  delay(500);
  analogWrite(laserPin, 255);
  ServoX.easeTo(ServoXControl.maxDegree, 50);
  ServoY.easeTo(ServoYControl.maxDegree, 50);
  ServoX.easeTo(ServoXControl.minDegree, 50);
  ServoY.easeTo(ServoYControl.minDegree, 50);
  analogWrite(laserPin, 0);

  delay(4000);

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

  digitalWrite(ledPin, HIGH);
}

uint8_t getRandomValue(ServoControlStruct *aServoControlStruct, ServoEasing *aServoEasing) {
  uint8_t tNewTargetAngle;
  do {
    tNewTargetAngle = random(aServoControlStruct->minDegree, aServoControlStruct->maxDegree);
  } while (tNewTargetAngle == aServoEasing->MicrosecondsOrUnitsToDegree(aServoEasing->mCurrentMicrosecondsOrUnits)); // do not accept current angle as new value
  return tNewTargetAngle;
}

void loop(void) {
  if (laserActive) {
    if (!ServoX.isMoving()) {
      delay(random(500));
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
    }
  }

  else {
    // DO OTHER STUFF
  }
}
