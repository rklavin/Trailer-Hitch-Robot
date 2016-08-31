#include <XBOXRECV.h>

const int actrCtrlPin1 = 2;             //Pin to control actuator movement
const int actrCtrlPin2 = 3;             //Pin to control actuator movement
const int hallSensorPin = A0;           //Pin to read Hall Effect Sensor

const int limitTop = 100;               //Value from Hall Effect Sensor for max extension
const int limitBot = 0;                 //Value from Hall Effect Sensor for max retraction

USB Usb;
XBOXRECV Xbox(&Usb);

void setup() {
  Serial.begin(115200);
  Usb.Init(); 
  pinMode(actrCtrlPin1, OUTPUT);
  pinMode(actrCtrlPin2, OUTPUT);
}

void loop() {
  Usb.Task();
  if (Xbox.XboxReceiverConnected) {
    int pos = analogRead(hallSensorPin);                  //Get actuator vertical position
    if (Xbox.getAnalogHat(RightHatY , 0) > 1000) {        //Check if Right Stick is in up direction
      if (pos < limitTop) {                               //Check if actuator can extend
        actuatorUp();
      } else {
        actuatorHold();
      }
    } else if(Xbox.getAnalogHat(LeftHatY, 0) < 1000) {    //Check if Right Stick is in down direction
      if (pos > limitBot) {                               //Check if actuator can retract
        actuatorDown();
      } else {
        actuatorHold();
      }
    } else {                                              //If Right Stick is not moved
      actuatorHold();
    }
    delay(1);
  }
delay(1);
}

void actuatorUp() {
  digitalWrite(actrCtrlPin1, HIGH);
  digitalWrite(actrCtrlPin2, LOW);
}

void actuatorDown() {
  digitalWrite(actrCtrlPin1, LOW);
  digitalWrite(actrCtrlPin2, HIGH);
}

void actuatorHold() {
  digitalWrite(actrCtrlPin1, LOW);
  digitalWrite(actrCtrlPin2, LOW);
}
