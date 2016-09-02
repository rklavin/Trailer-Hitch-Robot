#include <XBOXRECV.h>

const int actrCtrlPin = 2;              //Pin to control actuator movement
const int hallSensorPin = A0;           //Pin to read Hall Effect Sensor

const int limitTop = 100;               //Value from Hall Effect Sensor for max extension
const int limitBot = 0;                 //Value from Hall Effect Sensor for max retraction
const int actrUp = 100;                 //Value used to extend actuator
const int actrDown = 100;               //Value used to retract actuator
const int actrHold = 0;                 //Value used to hold actuator in position
int sensorOK = 0;


int motorDir = 8; //Dir 1
int motorSpeed = 9; //PWM 1
int motorDir2 = 10; //Dir 2
int motorSpeed2 = 11; //PWM2

int ultraSensorOK = 12;
int buzzardBackwards = 13;               //backwards buzzard

USB Usb;
XBOXRECV Xbox(&Usb);

void setup() {
  Serial.begin(115200);
  Usb.Init(); 
  pinMode(actrCtrlPin1, OUTPUT);
  pinMode(actrCtrlPin2, OUTPUT);
  pinMode(motorDir, OUTPUT); //Movement Part
  pinMode(motorDir2, OUTPUT); //Movement Part
  pinMode(motorSpeed, OUTPUT); //Movement Part
  pinMode(motorSpeed2, OUTPUT); //Movement Part
  pinMode(ultraSensorOK, INPUT); //sensor okay to move?
  pinMode(buzzardBackwards, INPUT); //beeping for backwards
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
    } 
    else if(Xbox.getAnalogHat(RightHatY, 0) < 1000) {   //Check if Right Stick is in down direction
      if (pos > limitBot) {                               //Check if actuator can retract
        actuatorDown();
      } 
      else {
        actuatorHold();
      }
    } 
    else if(Xbox.getAnalogHat(LeftHatY, 0) > 1000) {
      //going forward
      ForwardMovement();
    } 
    else if(Xbox.getAnalogHat(LeftHatY, 0) < 1000) {   
      //going backwards
      //buzzard
      BackwardMovement();
    }
    else if(Xbox.getAnalogHat(LeftHatX, 0) > 1000) {
      //going right???
    } 
    else if(Xbox.getAnalogHat(LeftHatX, 0) < 1000) {   
      //going left???
    }
    else {                                              //If Right Stick is not moved
      actuatorHold();
    }
    delay(1);
  }
delay(1);
}

void actuatorUp() {
  analogWrite(actrCtrlPin, actrUp);
}

void actuatorDown() {
  analogWrite(actrCtrlPin, actrDown);
}

void actuatorHold() {
  analogWrite(actrCtrlPin, actrHold);
}



//Stephanie Stuff for movement
void ForwardMovement() {

  if ultraSensorOK > 5 {
    sensorOK = 1;
    if ultraSensorOK >= 10 {
      //continue full speed ahead!
      
    }
    else if ultraSensorOk >= 8 {
      //continue a third of the speed
    }
    else if ultraSensorOK >= 6 {
      //continue 2 thirds of full speed
    }
    else {
      //continue a crawl of full speed
    }

  }
  else {
    sensorOK = 0;
    analogWrite(motorSpeed, 0);
    analogWrite(motorSpeed2, 0);
    //add in confirmation button
    //add in back movement is only allowed
    //Laura's function of button A needs to be called
    if PIN_BUTTON_A = LOW {
      ButtonA_pressed();
    }
    else{
    BackwardMovement();   //only allowed
    }
  }
}

void BackwardMovement() {
    sensorOK = 1;
    buzzardBackwards = 1; //figure out how to make it speak
  
    analogWrite(motorSpeed, 1); //motors same speed but slow
    analogWrite(motorSpeed2, 1); //motors same speed but slow
  
}
