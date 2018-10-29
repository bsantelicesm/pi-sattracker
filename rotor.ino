#define AZ1 6
#define AZ2 7
#define EL1 8
#define EL2 9
#define AZEN 10
#define ELEN 11
#define AZISEN A2
#define AZISEN A3
//motor drive pins as stated in L298N datasheet

#define AZFB = A1
#define ELFB = A2
//feedback pot pins

float azimuth;
float elevation;
String command;
float setAz;
float setEl;
//global variables

float pots2degrees(char axisPot) { //axisPot corresponds to the feedbacl pot pin
  return analogRead(axisPot) * (360/1023); //converts from analog value to degrees
}

//main motor drive function
//axis: 0=Az, 1=El
//direction: 0=FWD, 1=REV
//speed: integer between 1 and 4
void motorDrive(int axis, int direction, int speed) {
    int motorA;
    int motorB;
    int motorEN;

    switch (axis) { //select the right motor pins for each axis
      case 0:
        motorA = AZ1;
        motorB = AZ2;
        motorEN = AZEN;
        break
      case 1:
        motorA = EL1;
        motorB = EL2;
        motorEN = ELEN;
    }
    switch (direction) { //enable the direction lines for each direction
      case 0:
        digitalWrite(motorA, HIGH);
        digitalWrite(motorB, LOW);
      case 1:
        digitalWrite(motorA, LOW);
        digitalWrite(motorA, HIGH);
    }
    if (speed == 0){
        analogWrite(motorEN, (speed*64)-1) //send a PWM signal to the EN pin, to start the motor at each speed.
    }
    else {
      digitalWrite(motorEN, LOW); // if speed is 0, shutdown motor.
    }
  }

void processCommands() { //processes EasyComm I commands coming from serial port.
  int firstSpace;
  int secondSpace;

  if (Serial.available() > 0) { //check serial buffer
      command = Serial.readString();  //read command string
      if (command.startsWith("AZ EL")) {  //check for angle query command
        Serial.println("AZ=");
        Serial.print(azimuth);
        Serial.print("EL=");
        Serial.print(elevation); //print current angles
      }
      else {
        if (command.startsWith("AZ")) { //check for angle order command
          firstSpace = command.indexOf(' '); //find first space
          secondSpace = command.indexOf(' ', firstSpace + 1); //find second space

          setAz = (command.substring(3, firstSpace)).toFloat();
          setEl = (command.substring(firstSpace + 3, secondSpace)).toFloat(); //save values as floats
      }
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


}
