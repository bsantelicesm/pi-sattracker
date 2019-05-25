//---LIBS---
#include <math.h>
#include <EEPROM.h>

#define FWD true
#define REV false
#define STOP 0

#define AZCAL 10
#define ELCAL 20

//---GLOBAL VARS---
float azimuth = 0.0;
float elevation = 0.0;

float error = 0.5;

String command;
//---OBJECT CLASSES---
class Motor {
  int enablePin;
  int nonInvertingPin;
  int invertingPin;
  int feedbackPin;

  float calibration;
  int calAddr;

  public:
  int travelRange;

  Motor(int enable, int noninverting, int inverting, int feedback, int range, int cal) {
    enablePin = enable;
    nonInvertingPin = noninverting;
    invertingPin = inverting;
    feedbackPin = feedback;
    travelRange = range;
    calAddr = cal;

  }

  void init() {
    int pins[3] = {enablePin, nonInvertingPin, invertingPin};
    for (int i=0; i=2; i++) {
      pinMode(pins[i], OUTPUT);
    }
    float eepromData;
    EEPROM.get(calAddr, eepromData);
    if (isnan(eepromData)) {
      EEPROM.put(calAddr, 0);
    }
    else {
      calibration = eepromData;
    }
  }

  float angle() {
   float angle = map(analogRead(feedbackPin) - calibration, 0, 1023, 0, travelRange);
    return angle;
  }

  void move(bool direction, int speed) {
    if (direction = FWD) {
      digitalWrite(nonInvertingPin, HIGH);
      digitalWrite(invertingPin, LOW);
    }
    else if (direction = REV) {
      digitalWrite(nonInvertingPin, LOW);
      digitalWrite(invertingPin, HIGH);
    }

    analogWrite(enablePin, speed);
  }

  void calibrate() {
    calibration = analogRead(feedbackPin);
    EEPROM.put(calAddr, calibration);
  }

};
//---OBJECT CALLS---
Motor AZ(2, 3, 4, 5, A0, AZCAL);
Motor EL(6, 7, 8, 9, A1, ELCAL);
//---FUNCTIONS---

void easycommProcess(String command) {
  if (command.startsWith("AZ EL")) {
    Serial.print("AZ=");
    Serial.print(AZ.angle());
    Serial.print(" EL=");
    Serial.print(EL.angle());
  }

  else if(command.startsWith("AZ=")) {
    int azimuthEquals = command.indexOf("=");
    int elevationEquals = command.indexOf("=", azimuthEquals + 1);

    azimuth = command.substring(azimuthEquals, azimuthEquals + 5).toFloat();
    elevation = command.substring(elevationEquals, elevationEquals + 5).toFloat();
  }
}

bool withinRange(float var, float lowerLimit, float upperLimit) {
  if ((var > lowerLimit) && (var < upperLimit)) {
    return true;
  }
  else {
    return false;
  }
}

void calibrate2axis() {
  AZ.calibrate();
  EL.calibrate();
}

int getSpeed(float delta, int range) {
  return (map(abs(delta), 0, range, 0, 205) + 50);
}

void setup() {
  Serial.begin(9600); //begin serial communications @ 9600 baud
  attachInterrupt(digitalPinToInterrupt(2), calibrate2axis, FALLING);
}

void loop() {
  if (Serial.available() > 0) {
    easycommProcess(Serial.readString());
  }
  if (withinRange(AZ.angle(), azimuth - error, azimuth + error)) {
    AZ.move(FWD, STOP);
  }
  else {
    float AzDelta = azimuth - AZ.angle();
    int AzSpeed = getSpeed(AzDelta, AZ.travelRange);

    if (AzDelta > 0) {
      AZ.move(FWD, AzSpeed);
    }
    else if (AzDelta < 0) {
      AZ.move(REV, AzSpeed);
    }
  }
  if (withinRange(EL.angle(), elevation - error, elevation + error)) {
    EL.move(FWD, STOP);
  }
  else {
    float ElDelta = azimuth - EL.angle();
    int ElSpeed = getSpeed(ElDelta, EL.travelRange);
    if (ElDelta > 0) {
      EL.move(FWD, ElSpeed);
    }
    else if (ElDelta < 0) {
      EL.move(REV, ElSpeed);
    }
  }
}
