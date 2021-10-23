#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include "credentials.h"

#include <Arduino.h>
#include <ServoEasing.h>
#include <Preferences.h>

#define SERVO_MIN_DEGREE 0
#define SERVO_MAX_DEGREE 270

#define TOUCH_PIN T0
#define TOUCH_THRESHOLD 40

Preferences preferences;

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;

AsyncWebServer server(80);

const int ledPin = 2;
const int potPin = 32;
const int strobePin = 33;
const int sirenPin = 23;
const int laserPin = 25; // A0
const int xServoPin = 5;
const int yServoPin = 18;

int potMin;
int potMax;
int xMin, xMax;
int yMin, yMax;

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

  minSet = false;
  maxSet = false;

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
}

void calibrateServos() {
  Serial.println(F("Calibrating servos."));

  xSet = false;
  ySet = false;


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

  preferences.begin("bbneut", false);

  potMin = preferences.getInt("potMin", -1);
  potMax = preferences.getInt("potMax", -1);

  if (potMin == -1 || potMax == -1) {
    digitalWrite(ledPin, HIGH);
    calibratePot();
    digitalWrite(ledPin, LOW);
  }

  delay(1000);

  if (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
    digitalWrite(ledPin, HIGH);
    delay(100);
    digitalWrite(ledPin, LOW);
    while (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
      delay(10);
    }
    digitalWrite(ledPin, HIGH);
    calibratePot();
    digitalWrite(ledPin, LOW);
  }

  Serial.print(F("Attach servo at pin "));
  Serial.println(xServoPin);
  if (ServoX.attach(xServoPin, 67) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo"));
  }

  Serial.print(F("Attach servo at pin "));
  Serial.println(yServoPin);
  if (ServoY.attach(yServoPin, 67) == INVALID_SERVO) {
    Serial.println(F("Error attaching servo"));
    while (true) {
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
      delay(100);
    }
  }

  delay(1000);

  int touchDetected = 0;
  for (int i = 0; i < 5; i++) {
    if (touchRead(TOUCH_PIN) < TOUCH_THRESHOLD) {
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
      touchDetected++;
    }
    delay(500);
    if (touchDetected >= 3) {
      digitalWrite(ledPin, HIGH);
      calibrateServos();
      digitalWrite(ledPin, LOW);
      break;
    }
  }
  else {
    xMin = preferences.getInt("xMin", -1);
    xMax = preferences.getInt("xMax", -1);
    yMin = preferences.getInt("yMin", -1);
    yMax = preferences.getInt("yMax", -1);

    if (
      xMin == -1 ||
      xMax == -1 ||
      yMin == -1 ||
      yMax == -1
    ) {
      digitalWrite(ledPin, HIGH);
      calibrateServos();
      digitalWrite(ledPin, LOW);
    }
  }

  ServoXControl.minDegree = 45;
  ServoXControl.maxDegree = 90;
  ServoYControl.minDegree = 45;
  ServoYControl.maxDegree = 90;

  //ServoX.write(90);
  //ServoY.write(90);

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

  server.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", "Demonic Beanbag Neutralizer v0.1");
  });

  AsyncElegantOTA.begin(&server);    // Start ElegantOTA
  server.begin();
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
