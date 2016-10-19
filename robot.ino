#include <XBOXRECV.h>
#include <Servo.h>
#include <Math.h>

//need a pin for emergency stop button(s)

//System variables
const int restartDelay = 5000;          //Value in miliseconds, used when reset combination is pressed during emergency stop

//Actuator variables
const int actrCtrlPin = 8;              //Pin to control actuator movement
const int hallSensorPin = A0;           //Pin to read Hall Effect Sensor

Servo actuator;                         //Servo output for actuator controller

const int limitTop = 100;               //Value from Hall Effect Sensor for max extension
const int limitBot = 0;                 //Value from Hall Effect Sensor for max retraction
const int actrUp = 100;                 //Value used to extend actuator
const int actrDown = 100;               //Value used to retract actuator
//const int actrHold = 0;                 //Value used to hold actuator in position

//Global flags
bool sensorOK = true;                   //Boolean value checked when left stick is moved
bool emergencyStop = false;             //Boolean value, prevents all inputs while true, must reset arduino to continue use

//Motor variables
const int motorSpeed1Pin = 6;           //Pin to control speed of motor 1
const int motorSpeed2Pin = 7;           //Pin to control speed of motor 2

Servo motor1;                           //Servo output for motor controller 1
Servo motor2;                           //Servo output for motor controller 2

const int reverseMax = 46;              //Value to send to motor controller for maximum reverse speed
const int reverseMin = 90;              //Value to send to motor controller for minimum reverse speed
const int stationary = 92;              //Value to send to motor controller for no speed
const int forwardMin = 94;              //Value to send to motor controller for minimum forward speed
const int forwardMax = 141;             //Value to send to motor controller for maximum forward speed

//Speed factor variables
const int slowSpeed = 4;                //Value used in division of motor speed for when fast button is not pressed
const int turnSpeed = 4;                //Value used in division of motor speed for slowing one side when turning

//Ultrasonic sensor variables
const int ultraSensorTX = 14;           //Pin for trig
const int ultraSensorRX = 15;           //Pin for echo
const int buzzardBackwards = 13;        //Pin to send signal to buzzer

//Controller constant variables
const int joytr = 8000;             //Value to check joystick position against for activation (minimum press to move)
const int joym = 32000;             //Value for the limit of joystick position (needs verified)
const float joyadd = 0.174533;      //Value for 10 degrees in radians
const float joyatd = 0.087266;      //Value for 5 degrees in radians
const float pi = 3.141593;          //Value of pi
const float pi34 = 2.356194;        //Value of pi*(3/4)
const float pi2 = 1.570796;         //Value of pi/2
const float pi4 = 0.785398;         //Value of pi/4
const float npi = 0.0 - pi;         //Value of negative pi
const float npi34 = 0.0 - pi34;     //Value of negative pi*(3/4)
const float npi2 = 0.0 - pi2;       //Value of negative pi/2
const float npi4 = 0.0 - pi4;       //Value of negative pi/4

//Controller bound variables - variables in radians arranged from 0 to 2pi, atan2 is -pi to pi
const float joyrp = joyadd;         //Right turn positive bound
const float joyfrn = pi4 - joyadd;  //Forward right turn negative bound
const float joyfrp = pi4 + joyadd;  //Forward right turn postive bound
const float joyfn = pi2 - joyadd;   //Forward negative bound
const float joyfp = pi2 + joyadd;   //Forward positive bound
const float joyfln = pi34 - joyadd; //Forward left negative bound
const float joyflp = pi34 + joyadd; //Forward left positive bound
const float joyln = pi - joyadd;    //Left turn negative bound
const float joylp = npi + joyadd;   //Left turn positive bound
const float joybln = npi34 - joyadd;//Backward left turn negative bound
const float joyblp = npi34 + joyadd;//Backward left turn positive bound
const float joybn = npi2 - joyadd;  //Backward negative bound
const float joybp = npi2 + joyadd;  //Backward positive bound
const float joybrn = npi4 - joyadd; //Backward right turn negative bound
const float joybrp = npi4 + joyadd; //Backward right turn positive bound
const float joyrn = 0.0 - joyadd;   //Right turn negative bound

USB Usb;
XBOXRECV Xbox(&Usb);

void setup() {
  Usb.Init(); 
  actuator.attach(actrCtrlPin);
  motor1.attach(motorSpeed1Pin);
  motor2.attach(motorSpeed2Pin);
}

void loop() {
  Usb.Task();
  if (Xbox.XboxReceiverConnected) {
    if (!emergencyStop) {
      bool moving = false;
      if (sensorOK) {
        float joyy = Xbox.getAnalogHat(LeftHatY, 0);
        float joyx = Xbox.getAnalogHat(LeftHatX, 0);
        float joyr = sqrt(square(joyy) + square(joyx));
        if ( joyr > joytr ) {
          float joya = atan2(joyy, joyx);
          if ( (joya > joyrn) && (joya < joyrp) ) {         //Right Turn - Turn Motor Full Reverse
            
          }
          else if ( (joya >= joyrp) && (joya <= joyfrn) ) { //Forward Right - Turn Motor Reverse
          
          }
          else if ( (joya > joyfrn) && (joya < joyfrp) ) {  //Forward Right - Turn Motor Stopped
            motor2 = stationary;
          }
          else if ( (joya >= joyfrp) && (joya <= joyfn) ) { //Forward Right - Turn Motor Forward
            
          }
          else if ( (joya > joyfn) && (joya < joyfp) ) {    //Forward - Turn Motor Full Forward
            
          }
          else if ( (joya >= joyfp) && (joya <= joyfln) ) { //Forward Left - Turn Motor Forward
            
          }
          else if ( (joya > joyfln) && (joya < joyflp) ) {  //Forward Left - Turn Motor Stopped
            
          }
          else if ( (joya >= joyflp) && (joya <= joyln) ) { // Forward Left - Turn Motor Reverse
            
          }
          else if ( (joya > joyln) && (joya < joylp) ) {    //Left Turn - Turn Motor Full Reverse
            
          }
          else if ( (joya >= joylp) && (joya <= joybln) ) { //Back Left Turn - Turn Motor Forward
            
          }
          else if ( (joya > joybln) && (joya < joyblp) ) {  //Backward Left - Turn Motor Stopped
            
          }
          else if ( (joya >= joyblp) && (joya <=joybn) ) {  //Backward Left - Turn Motor Reverse
            
          }
          else if ( (joya > joybn) && (joya < joybp) ) {    //Backward - Turn Motor Full Reverse
            
          }
          else if ( (joya >= joybp) && (joya <= joybrn) ) { //Backward Right - Turn Motor Reverse
            
          }
          else if ( (joya > joybrn) && (joya < joybrp) ) {  //Backward Right - Turn Motor Stopped
            
          }
          else if ( (joya >= joybrp) && (joya <= joyrn) ) { //Backward Right - Turn Motor Forward
            
          }
        }
      } else {                      //Sensor detected object too close in front of robot     
        NoMovement();
        moving = false;
        if (Xbox.getButtonClick(A, 0)) {
         //A Button is pressed
         ButtonA_pressed();
        }
      }

      if (!moving) {
        //int pos = analogRead(hallSensorPin);                      //Get actuator vertical position
        //pos = 10;
        if (Xbox.getAnalogHat(RightHatY , 0) > conThresh) {         //Check if Right Stick is in up direction
          actuatorUp();
        } else if (Xbox.getAnalogHat(RightHatY, 0) < -conThresh) {  //Check if Right Stick is in down direction
          actuatorDown();
        } else {                                                    //If Right Stick is not moved
          actuatorHold();
        }
      } else {                                                      //If the robot is moving
        actuatorHold();
      }
      
      if ((Xbox.getButtonClick(B, 0)) || (false)) {                 //Emergency stop protocol
        //change false^ to check the pin the emergency stop button is on
        EmergencyStop();
      }
    } else {                                                        //If an emergency stop button has been pressed
      NoMovement();
      if ((Xbox.getButtonPress(A, 0)) && (Xbox.getButtonPress(B, 0)) && (Xbox.getButtonPress(X, 0)) && (Xbox.getButtonPress(Y, 0))) {
        delay(restartDelay);                                  //Combination of A+B+X+Y must be pressed to release emegency stop
        emergencyStop = false;
      }
    }
    delay(1);
  } else {              //If no controller is detected
    NoMovement();
  }
}

void actuatorUp() {
  //int stick = Xbox.getAnalogHat(RightHatY, 0);
  //int movement = map(stick, conThresh, conLimit, forwardMin, forwardMax);
  actuator.write(forwardMax);          //or use actrUp to specify constant value if not max forward
}

void actuatorDown() {
  //int stick = Xbox.getAnalogHat(RightHatY, 0);
  //int movement = map(stick, -conThresh, -conLimit, reverseMin, reverseMax);
  actuator.write(reverseMax);          //or use actrDown to specify constant value if not max reverse
}

void actuatorHold() {
  actuator.write(stationary);
}

void NoMovement() {
  motor1.write(stationary);
  motor2.write(stationary);
  actuator.write(stationary);
}

void ButtonA_pressed() {
  //Allow robot to be moved after proximity detection
  //Maybe something where sensorOK flag is set to true, then temporarily disable supersonic sensor to allow movement
}

void EmergencyStop() {
  //Stop all current robot movement and prevent future movements
  emergencyStop = true;
  motor1.write(stationary);
  motor2.write(stationary);
  actuator.write(stationary);
}
