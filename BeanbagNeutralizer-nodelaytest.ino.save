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
#include <MqttLogger.h>
#include <ESP32Servo.h>

#define X_MICROS_0_DEG 600
#define X_MICROS_180_DEG 2400
#define Y_MICROS_0_DEG 600
#define Y_MICROS_180_DEG 2400

#ifndef MQTT_LOGGING
#define mqttLogger Serial
#endif

Preferences preferences;

const char* ssid = WIFI_SSID;
const char* password = WIFI_PASS;

AsyncWebServer webServer(80);

const int ledPin = 2;
const int touchPin = 4;
const int potPin = 32;
const int strobePin = 35;
const int sirenPin = 23;
const int laserPin = 33;
const int xServoPin = 18;
const int yServoPin = 19;

bool touchVal;
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
#ifdef MQTT_LOGGING
MqttLogger mqttLogger(mqttClient, MQTT_LOGGER_TOPIC, MqttLoggerMode::MqttAndSerialFallback);
#endif

void mqttCallback(char* topic, byte* payload, uint8_t length) {
  char topicMsg[32];
  strcpy(topicMsg, topic);

  mqttLogger.print("Message arrived [");
  //delay(500);
  mqttLogger.print(topic);
  //delay(500);
  mqttLogger.print("] ");
  //delay(500);
  for (int i = 0; i < length; i++) {
    mqttLogger.print((char)payload[i]);
  }
  delay(500);
  mqttLogger.println();

  String commandString = String(topicMsg);
  mqttLogger.print(F("[initial] commandString: "));
  //delay(500);
  mqttLogger.print(commandString);
  //delay(500);
  mqttLogger.println();
  if (commandString.indexOf('!') >= 0) {
    commandString.remove(0, (commandString.lastIndexOf('/') + 1));
    mqttLogger.print(F("[remove] commandString: "));
    //delay(500);
    mqttLogger.print(commandString);
    //delay(500);
    mqttLogger.println();
    handleCommand(commandString);
  }
  else {
    mqttLogger.print(F("MQTT data received is not a command."));
    //delay(500);
    mqttLogger.println();
  }
}

boolean mqttReconnect() {
  if (mqttClient.connect(MQTT_CLIENT, MQTT_USER, MQTT_PASS, MQTT_WILL_TOPIC, MQTT_WILL_QOS, MQTT_WILL_RETAIN, MQTT_WILL_MESSAGE)) {
    mqttClient.publish(MQTT_OUT_TOPIC, MQTT_PUB_MESSAGE);
    mqttClient.subscribe(MQTT_IN_TOPIC);
  }
  return mqttClient.connected();
}

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
  mqttLogger.println(F("Calibrating potentiometer."));

  bool minSet = false;
  bool maxSet = false;

  mqttLogger.println(F("Move potentiometer to MIN position then tap touch sensor to confirm."));
  //delay(7500);
  while (minSet == false) {
    touchVal = digitalRead(touchPin);
    mqttLogger.print(F("touchVal: ")); mqttLogger.println(touchVal);

    if (touchVal == LOW) {
      potMin = analogRead(potPin);
      mqttLogger.print(F("potMin: ")); mqttLogger.println(potMin);
      minSet = true;
      blinkLed(false);
      while (digitalRead(touchPin) == LOW) {
        delay(100);
      }
      delay(1000);
    }
    delay(250);
  }
  preferences.putInt("potMin", potMin);

  delay(5000);
  blinkLed(false);

  mqttLogger.println(F("Move potentiometer to MAX position then tap touch sensor to confirm."));
  //delay(7500);
  while (maxSet == false) {
    touchVal = digitalRead(touchPin);
    mqttLogger.print(F("touchVal: ")); mqttLogger.println(touchVal);

    if (touchVal == LOW) {
      potMax = analogRead(potPin);
      mqttLogger.print(F("potMax: ")); mqttLogger.println(potMax);
      maxSet = true;
      blinkLed(false);
      while (digitalRead(touchPin) == LOW) {
        delay(100);
      }
      delay(1000);
    }
    delay(250);
  }
  preferences.putInt("potMax", potMax);
}

void setServoRange() {
  analogWrite(laserPin, 255);

  mqttLogger.println(F("Calibrating servos."));

  bool xMinSet = false;
  bool xMaxSet = false;
  bool yMinSet = false;
  bool yMaxSet = false;

  while (xMinSet == false) {
    int servoVal = map(analogRead(potPin), potMin, potMax, 0, 180);
    ServoX.easeTo(servoVal, 50);

    touchVal = digitalRead(touchPin);
    mqttLogger.print(F("touchVal: ")); mqttLogger.println(touchVal);

    if (touchVal == LOW) {
      xMinDegree = servoVal;
      mqttLogger.print(F("xMinDegree: ")); mqttLogger.println(xMinDegree);
      xMinSet = true;
      blinkLed(false);
      while (digitalRead(touchPin) == LOW) {
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

    touchVal = digitalRead(touchPin);
    mqttLogger.print(F("touchVal: ")); mqttLogger.println(touchVal);

    if (touchVal == LOW) {
      xMaxDegree = servoVal;
      mqttLogger.print(F("xMaxDegree: ")); mqttLogger.println(xMaxDegree);
      xMaxSet = true;
      blinkLed(false);
      while (digitalRead(touchPin) == LOW) {
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

    touchVal = digitalRead(touchPin);
    mqttLogger.print(F("touchVal: ")); mqttLogger.println(touchVal);

    if (touchVal == LOW) {
      yMinDegree = servoVal;
      mqttLogger.print(F("yMinDegree: ")); mqttLogger.println(yMinDegree);
      yMinSet = true;
      blinkLed(false);
      while (digitalRead(touchPin) == LOW) {
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

    touchVal = digitalRead(touchPin);
    mqttLogger.print(F("touchVal: ")); mqttLogger.println(touchVal);

    if (touchVal == LOW) {
      yMaxDegree = servoVal;
      mqttLogger.print(F("yMaxDegree: ")); mqttLogger.println(yMaxDegree);
      yMaxSet = true;
      blinkLed(false);
      while (digitalRead(touchPin) == LOW) {
        delay(100);
      }
      delay(1000);
    }
    delay(250);
  }

  analogWrite(laserPin, 0);
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

  // Button
  pinMode(touchPin, INPUT_PULLUP);

  Serial.begin(115200);
  mqttLogger.println();

  preferences.begin("beanbag", false);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  mqttLogger.println();
  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    mqttLogger.print(F("."));
  }
  mqttLogger.println();
  mqttLogger.print(F("Connected to "));
  mqttLogger.println(ssid);
  mqttLogger.print(F("IP address: "));
  mqttLogger.println(WiFi.localIP());

  webServer.on("/", HTTP_GET, [](AsyncWebServerRequest * request) {
    request->send(200, "text/plain", "Demonic Beanbag Neutralizer v0.1");
  });

  AsyncElegantOTA.begin(&webServer);    // Start ElegantOTA
  webServer.begin();
  mqttLogger.println("HTTP server started");

  //mqttClient.setServer(mqttServer, 1883);
  //mqttClient.setCallback(mqttCallback);

  while (!mqttClient.connected()) {
    delay(5000);
    //bool mqttConnectResult = mqttReconnect();
    mqttLogger.print(F("mqttConnectResult: ")); mqttLogger.println(mqttReconnect());
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

    if (digitalRead(touchPin) == LOW) {
      blinkLed(true);
      while (digitalRead(touchPin) == LOW) {
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

    mqttLogger.println(F("Restarting ESP32..."));
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

  else if (digitalRead(touchPin) == LOW) {
    blinkLed(true);
    while (digitalRead(touchPin) == LOW) {
      delay(100);
    }
    servoSetRequired = true;
  }

  /*
      Servo attach parameters:
      attach(int aPin, int aInitialDegree, int aMicrosecondsForServo0Degree, int aMicrosecondsForServo180Degree)
  */
  mqttLogger.print(F("Attaching X servo at pin "));
  mqttLogger.println(xServoPin);
  if (ServoX.attach(xServoPin, 90, X_MICROS_0_DEG, X_MICROS_180_DEG) == INVALID_SERVO) {
    mqttLogger.println(F("error attaching X servo."));
  }
  else mqttLogger.println(F("successfully mounted."));

  mqttLogger.print(F("Attaching Y servo at pin "));
  mqttLogger.println(yServoPin);
  if (ServoY.attach(yServoPin, 90, Y_MICROS_0_DEG, Y_MICROS_180_DEG) == INVALID_SERVO) {
    mqttLogger.println(F("error attaching Y servo."));
    while (true) {
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
      delay(100);
    }
  }
  else mqttLogger.println(F("successfully mounted."));

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

    mqttLogger.println(F("Restarting ESP32..."));
    delay(1000);
    ESP.restart();
  }

  ServoXControl.minDegree = xMinDegree;
  ServoXControl.maxDegree = xMaxDegree;
  ServoYControl.minDegree = yMinDegree;
  ServoYControl.maxDegree = yMaxDegree;

  mqttLogger.print(F("ServoXControl.minDegree: ")); mqttLogger.println(ServoXControl.minDegree);
  mqttLogger.print(F("ServoXControl.maxDegree: ")); mqttLogger.println(ServoXControl.maxDegree);
  mqttLogger.print(F("ServoYControl.minDegree: ")); mqttLogger.println(ServoYControl.minDegree);
  mqttLogger.print(F("ServoYControl.maxDegree: ")); mqttLogger.println(ServoYControl.maxDegree);

  mqttLogger.println(F("Marking border of laser area."));
  ServoX.write(ServoXControl.minDegree);
  ServoY.write(ServoYControl.minDegree);

  analogWrite(laserPin, 255);

  delay(1000);

  ServoY.easeTo(ServoYControl.maxDegree, 50);
  delay(500);
  ServoX.easeTo(ServoXControl.maxDegree, 50);
  delay(500);
  ServoY.easeTo(ServoYControl.minDegree, 50);
  delay(500);
  ServoX.easeTo(ServoXControl.minDegree, 50);
  delay(500);

  analogWrite(laserPin, 0);

  //delay(4000);

  mqttLogger.println(F("Setup complete."));
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
    mqttLogger.println(F("MQTT connection lost. Attempting to reconnect..."));
    if ((msNow - retryConnectLast) > retryConnectInterval) {
      if (mqttReconnect()) retryConnectLast = 0;
      else retryConnectLast = msNow;
    }
  }
  else {
    mqttClient.loop();

    if (laserActive) {
      if (!ServoX.isMoving()) {
        delay(random(500));
        //uint16_t laserRandomElapsed = millis() - laserMoveLast;
        //mqttLogger.print(F("laserRandomElapsed: ")); mqttLogger.println(laserRandomElapsed);

        //if (laserRandomElapsed > laserRandomDelay) {

        uint8_t tNewHorizontal = getRandomValue(&ServoXControl, &ServoX);
        uint8_t tNewVertical = getRandomValue(&ServoYControl, &ServoY);
        int tSpeed = random(10, 90);

        /*mqttLogger.print(F("Move to horizontal="));
          mqttLogger.print(tNewHorizontal);
          mqttLogger.print(F(" vertical="));
          mqttLogger.print(tNewVertical);
          mqttLogger.print(F(" speed="));
          mqttLogger.println(tSpeed);*/
        ServoX.setEaseTo(tNewHorizontal, tSpeed);
        ServoY.setEaseTo(tNewVertical, tSpeed);
        synchronizeAllServosAndStartInterrupt();

        //laserRandomDelay = random(500);
        //mqttLogger.print(F("laserRandomDelay: ")); mqttLogger.println(laserRandomDelay);
        //laserMoveLast = millis();
        //}
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
      analogWrite(laserPin, 0);
      laserActive = false;
      mqttLogger.println(F("Laser disabled."));
      //delay(500);
    }
  }

  if (sirenActive) {
    if ((msNow - sirenStart) > sirenDuration) {
      analogWrite(sirenPin, 0);
      sirenActive = false;
      mqttLogger.println(F("Siren disabled."));
      //delay(500);
    }
  }

  if (strobeActive) {
    if ((msNow - strobeStart) > strobeDuration) {
      digitalWrite(strobePin, LOW);
      strobeActive = false;
      mqttLogger.println(F("Strobe disabled."));
      //delay(500);
    }
  }
}

void handleCommand(String command) {
  String item = command.substring((command.indexOf('!') + 1), command.indexOf('@'));
  mqttLogger.print(F("item: "));
  //delay(500);
  mqttLogger.print(item);
  //delay(500);
  mqttLogger.println();
  String action = command.substring(command.indexOf('@') + 1, command.indexOf('$'));
  mqttLogger.print(F("action: "));
  //delay(500);
  mqttLogger.print(action);
  //delay(500);
  mqttLogger.println();
  uint32_t duration = command.substring(command.indexOf('$') + 1).toInt() * 1000;
  mqttLogger.print(F("duration: "));
  //delay(500);
  mqttLogger.print(duration);
  //delay(500);
  mqttLogger.println();

  if (item.equalsIgnoreCase("laser")) {
    if (action.equalsIgnoreCase("on") || action.equalsIgnoreCase("1")) {
      analogWrite(laserPin, 255);
      laserActive = true;
      mqttLogger.println(F("Laser activated."));
      laserStart = millis();
      laserDuration = duration;
    }
    else if (action.equalsIgnoreCase("off") || action.equalsIgnoreCase("0")) {
      analogWrite(laserPin, 0);
      laserActive = false;
      mqttLogger.println(F("Laser disabled."));
      //delay(500);
    }
    else {
      mqttLogger.println(F("Unrecognized action in handleCommand()."));
      //delay(500);
    }
  }

  else if (item.equalsIgnoreCase("strobe")) {
    if (action.equalsIgnoreCase("on") || action.equalsIgnoreCase("1")) {
      digitalWrite(strobePin, HIGH);
      strobeActive = true;
      mqttLogger.println(F("Strobe activated."));
      strobeStart = millis();
      strobeDuration = duration;
    }
    else if (action.equalsIgnoreCase("off") || action.equalsIgnoreCase("0")) {
      digitalWrite(strobePin, LOW);
      strobeActive = false;
      mqttLogger.println(F("Strobe disabled."));
      //delay(500);
    }
    else {
      mqttLogger.println(F("Unrecognized action in handleCommand()."));
      //delay(500);
    }
  }

  else if (item.equalsIgnoreCase("siren")) {
    if (action.equalsIgnoreCase("on") || action.equalsIgnoreCase("1")) {
      analogWrite(sirenPin, 255);
      sirenActive = true;
      mqttLogger.println(F("Siren activated."));
      sirenStart = millis();
      sirenDuration = duration;
    }
    else if (action.equalsIgnoreCase("off") || action.equalsIgnoreCase("0")) {
      analogWrite(sirenPin, 0);
      sirenActive = false;
      mqttLogger.println(F("Siren disabled."));
      //delay(500);
    }
    else {
      mqttLogger.println(F("Unrecognized action in handleCommand()."));
      //delay(500);
    }
  }

  else {
    mqttLogger.println(F("Unrecognized item in handleCommand()."));
    //delay(500);
  }
}
