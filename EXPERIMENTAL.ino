#include <XBOXRECV.h>
#include <Servo.h>

//need a pin for emergency stop button(s)

//System variables
const int restartDelay = 5000;          //Value in miliseconds, used when reset combination is pressed during emergency stop

//Actuator variables
const int actrCtrlPin = 2;              //Pin to control actuator movement
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
const int motorSpeed1Pin = 3;           //Pin to control speed of motor 1
const int motorSpeed2Pin = 9;           //Pin to control speed of motor 2

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
  //pinMode(buzzardBackwards, INPUT); //beeping for backwards
  pinMode(ultraSensorRX, INPUT);
  pinMode(ultraSensorTX, OUTPUT);
}

void loop() {
  Usb.Task();
  if (Xbox.XboxReceiverConnected) {
    if (!emergencyStop) {
      bool moving = false;
      if (sensorOK) {
        if (Xbox.getAnalogHat(LeftHatY, 0) > conThresh) {                             //Move forward
          if (Xbox.getButtonPress(Y, 0)) {
            ForwardMovement(forwardMax, reverseMax);
          } else {
            ForwardMovement((forwardMin + ((forwardMax - forwardMin) / slowSpeed)), (reverseMin - ((reverseMin - reverseMax) / slowSpeed)));
          }
          moving = true;
        } else if (Xbox.getAnalogHat(LeftHatY, 0) < -conThresh) {                     //Move backward
          if (Xbox.getButtonPress(Y, 0)) {
            BackwardMovement(forwardMax, reverseMax);
          } else {
            BackwardMovement((forwardMin + ((forwardMax - forwardMin) / slowSpeed)), (reverseMin - ((reverseMin - reverseMax) / slowSpeed)));
          }
          moving = true;
        } else if (Xbox.getAnalogHat(LeftHatX, 0) > conThresh) {                      //Turn right
          if (Xbox.getButtonPress(Y, 0)) {
            if (Xbox.getButtonPress(X, 0)) {                                          //If X is held it will turn in same direction, but in reverse
              RightMovement((reverseMin - ((reverseMin - reverseMax) / slowSpeed)), forwardMax);
            } else {
              RightMovement((forwardMin + ((forwardMax - forwardMin) / slowSpeed)), reverseMax);
            }
          } else {
            if (Xbox.getButtonPress(X, 0)) {
              RightMovement((reverseMin - ((reverseMin - reverseMax) / (slowSpeed * turnSpeed))), (forwardMin + ((forwardMax - forwardMin) / slowSpeed)));
            } else {
              RightMovement((forwardMin + ((forwardMax - forwardMin) / (slowSpeed * turnSpeed))), (reverseMin - ((reverseMin - reverseMax) / slowSpeed)));
            }
          }
          moving = true;
        } else if (Xbox.getAnalogHat(LeftHatX, 0) < -conThresh) {                     //Turn left
          if (Xbox.getButtonPress(Y, 0)) {
            if (Xbox.getButtonPress(X, 0)) {                                          //If X is held it will turn in same direction, but in reverse
              LeftMovement(reverseMax, (forwardMin + ((forwardMax - forwardMin) / slowSpeed)));
            } else {
              LeftMovement(forwardMax, (reverseMin - ((reverseMin - reverseMax) / slowSpeed)));
            }
          } else {
            if (Xbox.getButtonPress(X, 0)) {
              LeftMovement((reverseMin - ((reverseMin - reverseMax) / slowSpeed)), (forwardMin + ((forwardMax - forwardMin) / (slowSpeed * turnSpeed))));
            } else {
              LeftMovement((forwardMin + ((forwardMax - forwardMin) / slowSpeed)), (reverseMin - ((reverseMin - reverseMax) / (slowSpeed * turnSpeed))));
            }
          }
          moving = true;
        } else {                    //Joystick is not moved
          NoMovement();
          moving = false;
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



//Stephanie Stuff for movement
void ForwardMovement(int maxForward, int maxReverse) {
  int movement1 = stationary;
  int movement2 = stationary;
  long distance;
  int stick = Xbox.getAnalogHat(LeftHatY, 0);
  
  //Ultrasonic sensor polling code, calculate distance in centimeters
  distance = 100;
  
  if (distance > 25) {                //should give about 44 inches assuming 11 inch is 25
    sensorOK = true;
    if (distance >= 100) {
      //continue full speed ahead!
      movement1 = map(stick, conThresh, conLimit, reverseMin, maxReverse);
      movement2 = map(stick, conThresh, conLimit, forwardMin, maxForward);
    } else if (distance >= 75) {
      //continue at 2 thirds of full speed
      int max1 = forwardMin + (((maxForward - forwardMin)*2) / 3);
      int max2 = reverseMin - (((reverseMin - reverseMax)*2) / 3);
      movement1 = map(stick, conThresh, conLimit, reverseMin, max2);
      movement2 = map(stick, conThresh, conLimit, forwardMin, max1);
    } else if (distance >= 50) {     
      //continue at a third of full speed
      int max1 = forwardMin + ((maxForward - forwardMin) / 3);
      int max2 = reverseMin - ((reverseMin - reverseMax) / 3);
      movement1 = map(stick, conThresh, conLimit, reverseMin, max2);
      movement2 = map(stick, conThresh, conLimit, forwardMin, max1);
    } else { //25 should be about 11 inches away from it
      //continue a crawl of full speed
      int max1 = forwardMin + ((maxForward - forwardMin) / 6);
      int max2 = reverseMin - ((reverseMin - reverseMax) / 6);
      movement1 = map(stick, conThresh, conLimit, reverseMin, max2);
      movement2 = map(stick, conThresh, conLimit, forwardMin, max1);
    }
  } else {
    //Robot is too close, wait for confirmation to continue moving
    //Should give user feedback, check if blink middle LEDs is possible
    sensorOK = false;
  }
  motor1.write(movement1);
  motor2.write(movement2);
}

void BackwardMovement(int maxForward, int maxReverse) {
  //sensorOK = 1;
  //buzzardBackwards = 1; //figure out how to make it speak
  
  int stick = Xbox.getAnalogHat(LeftHatY, 0);
  int movement1 = map(stick, -conThresh, -conLimit, reverseMin, maxReverse);
  int movement2 = map(stick, -conThresh, -conLimit, forwardMin, maxForward);
  motor1.write(movement2);
  motor2.write(movement1);
}

void RightMovement(int maxForward, int maxReverse) {
  int stick = Xbox.getAnalogHat(LeftHatX, 0);
  int movement1 = map(stick, conThresh, conLimit, reverseMin, maxReverse);
  int movement2 = map(stick, conThresh, conLimit, forwardMin, maxForward);
  motor1.write(movement1);
  motor2.write(movement2);
}

void LeftMovement(int maxForward, int maxReverse) {
  int stick = Xbox.getAnalogHat(LeftHatX, 0);
  int movement1 = map(stick, -conThresh, -conLimit, reverseMin, maxReverse);
  int movement2 = map(stick, -conThresh, -conLimit, forwardMin, maxForward);
  motor1.write(movement1);
  motor2.write(movement2);
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
