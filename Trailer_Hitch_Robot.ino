#include <XBOXRECV.h>
#include <Servo.h>

//need a pin for emergency stop button(s)

//Actuator variables
const int actrCtrlPin = 2;              //Pin to control actuator movement
const int hallSensorPin = A0;           //Pin to read Hall Effect Sensor

Servo actuator;                         //Servo output for actuator controller

const int limitTop = 100;               //Value from Hall Effect Sensor for max extension
const int limitBot = 0;                 //Value from Hall Effect Sensor for max retraction
const int actrUp = 100;                 //Value used to extend actuator
const int actrDown = 100;               //Value used to retract actuator
//const int actrHold = 0;                 //Value used to hold actuator in position
bool emergencyStop = false;             //Boolean value, prevents all inputs while true, must reset arduino to continue use

//Motor variables
const int motorSpeed1Pin = 8;           //Pin to control speed of motor 1
const int motorSpeed2Pin = 9;           //Pin to control speed of motor 2

Servo motor1;                           //Servo output for motor controller 1
Servo motor2;                           //Servo output for motor controller 2

const int reverseMax = 46;              //Value to send to motor controller for maximum reverse speed
const int reverseMin = 90;              //Value to send to motor controller for minimum reverse speed
const int stationary = 92;              //Value to send to motor controller for no speed
const int forwardMin = 94;              //Value to send to motor controller for minimum forward speed
const int forwardMax = 141;             //Value to send to motor controller for maximum forward speed

//Controller variables
const int conThresh = 7500;             //Value to check joystick position against for activation (minimum press to move)
const int conLimit = 32767;             //Value for the limit of joystick position (needs verified)

USB Usb;
XBOXRECV Xbox(&Usb);

void setup() {
  Serial.begin(115200);
  Usb.Init(); 
  actuator.attach(actrCtrlPin);
  motor1.attach(motorSpeed1Pin);
  motor2.attach(motorSpeed2Pin);
}

void loop() {
  if (!emergencyStop) {
    Usb.Task();
    if (Xbox.XboxReceiverConnected) {
      bool moving = false;
      if (Xbox.getAnalogHat(LeftHatY, 0) > conThresh) {
        //going forward
        if (Xbox.getButtonClick(Y, 0)) {
          ForwardMovement(forwardMax);
        } else {
          ForwardMovement(forwardMax/4);
        }
        moving = true;
      } 
      else if (Xbox.getAnalogHat(LeftHatY, 0) < -conThresh) {   
        //going backwards
        //buzzard
        if (Xbox.getButtonClick(Y, 0)) {
          BackwardMovement(reverseMax);
        } else {
          BackwardMovement(reverseMax/4);
        }
        moving = true;
      }
      else if (Xbox.getAnalogHat(LeftHatX, 0) > conThresh) {
        //going right???
        moving = true;
      } 
      else if (Xbox.getAnalogHat(LeftHatX, 0) < -conThresh) {   
        //going left???
        moving = true;
      }
      else {
        NoMovement();
        moving = false;
      }

      if (!moving) {
        int pos = analogRead(hallSensorPin);                  //Get actuator vertical position
        pos = 10;
        if (Xbox.getAnalogHat(RightHatY , 0) > conThresh) {      //Check if Right Stick is in up direction
          if (pos < limitTop) {                               //Check if actuator can extend
            actuatorUp();
          } else {
            actuatorHold();
          }
        } 
        else if (Xbox.getAnalogHat(RightHatY, 0) < -conThresh) { //Check if Right Stick is in down direction
          if (pos > limitBot) {                               //Check if actuator can retract
            actuatorDown();
          } 
          else {
            actuatorHold();
          }
        } 
        else {                                              //If Right Stick is not moved
          actuatorHold();
        }
      }
      else {
        actuatorHold();
      }
      
     if (Xbox.getButtonClick(B, 0)) {
        // Button is pressed
        ButtonB_pressed();
      }
      else if (false) {
        //change false to pin emergency stop button is on
      }
      delay(1);
    }
  delay(1);
  }
  else {
    NoMovement();
  }
}

void actuatorUp() {
  int stick = Xbox.getAnalogHat(RightHatY, 0);
  int movement = map(stick, conThresh, conLimit, forwardMin, forwardMax);
  actuator.write(movement);          //or use actrUp to specify constant value
}

void actuatorDown() {
  int stick = Xbox.getAnalogHat(RightHatY, 0);
  int movement = map(stick, -conThresh, -conLimit, reverseMin, reverseMax);
  actuator.write(movement);          //or use actrDown to specify constant value
}

void actuatorHold() {
  actuator.write(stationary);
}



//Stephanie Stuff for movement
void ForwardMovement(int maxForward) {
  int stick = Xbox.getAnalogHat(LeftHatY, 0);
  int movement = map(stick, conThresh, conLimit, forwardMin, maxForward);
  motor1.write(movement);
  motor2.write(movement);
}

void BackwardMovement(int maxReverse) {
  int stick = Xbox.getAnalogHat(LeftHatY, 0);
  int movement = map(stick, -conThresh, -conLimit, reverseMin, maxReverse);
  motor1.write(movement);
  motor2.write(movement);
}

void NoMovement() {
  motor1.write(stationary);
  motor2.write(stationary);
}

//Laura

void ButtonA_pressed() {
  //Allow robot to be moved after proximity detection
}

void ButtonB_pressed() {
  //Stop all current robot movement and prevent future movements
  emergencyStop = true;
  motor1.write(stationary);
  motor2.write(stationary);
}
