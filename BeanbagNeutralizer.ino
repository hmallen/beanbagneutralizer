#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include "credentials.h"

#include <Arduino.h>
#include <ServoEasing.h>

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;

AsyncWebServer server(80);

const int ledPin = 2;
const int strobePin = 33;
const int sirenPin = 32;
const int laserPin = 25; // A0
const int xServoPin = 5;
const int yServoPin = 18;

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
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
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
