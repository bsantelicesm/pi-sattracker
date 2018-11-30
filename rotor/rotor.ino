#include <math.h>

#define AZ1 6
#define AZ2 7
#define EL1 8
#define EL2 9
#define AZEN 10
#define ELEN 11
//motor drive pins as stated in L298N datasheet

#define AZFB A0
#define ELFB A1
//feedback pot pins

float azimuth;
float elevation;
String command;
float setAz;
float setEl;
float offset = 0.5;
//global variables

float pots2degrees(unsigned char axisPot) { //axisPot corresponds to the feedback pot pin
  float degree = (analogRead(axisPot) * 0.352); //degrees in full length float
  return round(degree); //round to one decimal place
}

//main motor drive function
//axis: 0=Az, 1=El
//direction: 0=FWD, 1=REV
//speed: integer between 1 and 4
void motorDrive(int axis, int dir, int spd) {
    int motorA;
    int motorB;
    int motorEN; //variables for storing pins from the selected motor
    if (axis == 0) { //pins for az imuth axis
      motorA = 6;
      motorB = 7;
      motorEN = 10;
    }
   else if (axis == 1) { //pins for elevation axis
      motorA = 8;
      motorB = 9;
      motorEN = 11;
    }

    if (dir == 0) { //lM298N forward mode
      digitalWrite(motorA, HIGH); 
      digitalWrite(motorB, LOW);
    }
    else if (dir == 1) { //LM298N reverse mode
      digitalWrite(motorA, LOW);
      digitalWrite(motorB, HIGH);
    }

    if (spd == 0) {
      digitalWrite(motorEN, LOW); //shutdown motor by pulling the enable pin low
    }
    else if (spd == 1) {
      digitalWrite(motorEN, HIGH); //turn un by pulling enable pin high
    }
  }

void processCommands() { //processes EasyComm I commands coming from serial port.
  int firstSpace;
  int secondSpace; //variables to save the location of the first and second space
  //each command is terminated with a space

  if (Serial.available() > 0) { //check serial buffer
      command = Serial.readString();  //read command string
      if (command.startsWith("AZ EL")) {  //check for angle query command
        Serial.print("AZ=");
        Serial.print(azimuth);
        Serial.print(" ");
        Serial.print("EL=");
        Serial.println(elevation); //print current angles
      }
      else if (command.startsWith("AZ")) { //check for angle order command
        firstSpace = command.indexOf(' '); //find first space
        secondSpace = command.indexOf(' ', firstSpace + 1); //find second space

        String stringAz = (command.substring(3, firstSpace));
        String stringEl = (command.substring(firstSpace + 4, secondSpace)); //save substrings
        setAz = stringAz.toFloat();
        setEl = stringEl.toFloat(); //convert strings to floats and save on global variables
        Serial.print("AZ=");
        Serial.print(setAz);
        Serial.print(" ");
        Serial.print("EL=");
        Serial.println(setEl);  //return set values on the serial port
      }
  }
}

void setup() {
  pinMode(AZ1, OUTPUT);
  pinMode(AZ2, OUTPUT);
  pinMode(AZEN, OUTPUT); //pin mode for azimuth pins

  pinMode(EL1, OUTPUT);
  pinMode(EL2, OUTPUT);
  pinMode(ELEN, OUTPUT); //pin mode for elevation pins

  Serial.begin(9600); //begin serial communications @ 9600 baud
}

void loop() {
  azimuth = pots2degrees(AZFB);
  elevation = pots2degrees(ELFB); //save current rotor position

  processCommands(); //process commands coming from computer

//azimuth drive control
  if ((azimuth + offset) < setAz) { //forwards if azimuth is lower than set
    motorDrive(0, 0, 1);
  }
  else if ((azimuth - offset) > setAz) { //backwards if azimuth is higher than set
    motorDrive(0, 1, 1);
  }
  else { //stop if equal
    motorDrive(0, 1, 0);
  }

//elevation drive control
  if ((elevation + offset) < setEl) { //forwards if elevation is lower than set
    motorDrive(1, 0, 1);
  }
  else if ((elevation - offset) > setEl) { //backwards if elevation is higher than set
    motorDrive(1, 1, 1);
  }
  else { //stop if equal
    motorDrive(1, 1, 0);
  }
}
