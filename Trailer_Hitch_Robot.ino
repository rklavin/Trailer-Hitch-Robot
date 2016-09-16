#include <XBOXRECV.h>
#include <Servo.h>
#include "pitches.h"

//need a pin for emergency stop button(s)

//Actuator variables
const int actrCtrlPin = 2;              //Pin to control actuator movement
const int hallSensorPin = A0;           //Pin to read Hall Effect Sensor

Servo actuator;

const int limitTop = 100;               //Value from Hall Effect Sensor for max extension
const int limitBot = 0;                 //Value from Hall Effect Sensor for max retraction
const int actrUp = 100;                 //Value used to extend actuator
const int actrDown = 100;               //Value used to retract actuator
//const int actrHold = 0;                 //Value used to hold actuator in position
bool sensorOK = true;                   //Boolean value checked when left stick is moved
bool emergencyStop = false;             //Boolean value, prevents all inputs while true

//Motor variables
const int motorSpeed1Pin = 8;           //Pin to control speed of motor 1
const int motorSpeed2Pin = 9;           //Pin to control speed of motor 2

Servo motor1;                           //Servo output for motor controller 1
Servo motor2;                           //Servo output for motor controller 2

const int reverseMin = 90;              //Value to send to motor controller for minimum reverse speed
const int reverseMax = 41;              //Value to send to motor controller for maximum reverse speed
const int stationary = 92;              //Value to send to motor controller for no speed
const int forwardMin = 94;              //Value to send to motor controller for minimum forward speed
const int forwardMax = 143;             //Value to send to motor controller for maximum forward speed

//Ultrasonic sensor variables
const int ultraSensorTX = 14;           //Pin for trig
const int ultraSensorRX = 15;           //Pin for echo
const int buzzardBackwards = 13;        //Pin to send signal to buzzer

//Controller variables
const int conThresh = 7500;             //Value to check joystick position against for activation
const int conLimit = 32768;             //Value for the limit of joystick position (needs verified)

//These outputs are all for testing purposes 
int forwardLED = 22; 
int leftLED = 23;
int rightLED = 24;
int upLED = 25; 
int downLED = 26; 
int stopLED = 27;
int backwardLED = 30;

int melody[] = {                         //select the notes you want to play
  NOTE_E2/*, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4 */
};

int noteDurations[] = {                 //4= quarter note, 8 = either note
   1/*, 4, 4, 4 , 4, 4, 4, 4*/
};

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
  
  //Debug pins
  pinMode(forwardLED, OUTPUT);
  pinMode(leftLED, OUTPUT);
  pinMode(rightLED, OUTPUT);
  pinMode(upLED, OUTPUT);
  pinMode(downLED, OUTPUT);
  pinMode(stopLED, OUTPUT);
  pinMode(backwardLED, OUTPUT);
}

void loop() {
  if (!emergencyStop) {
    Usb.Task();
    if (Xbox.XboxReceiverConnected) {
      bool moving = false;
      if (sensorOK) {
        if (Xbox.getAnalogHat(LeftHatY, 0) > conThresh) {
          //going forward
          ForwardMovement();
          moving = true;
        } 
        else if (Xbox.getAnalogHat(LeftHatY, 0) < -conThresh) {   
          //going backwards
          //buzzard
          BackwardMovement();
          moving = true;
        }
        else if (Xbox.getAnalogHat(LeftHatX, 0) > conThresh) {
          //going right???
          digitalWrite(rightLED, HIGH);
          moving = true;
        } 
        else if (Xbox.getAnalogHat(LeftHatX, 0) < -conThresh) {   
          //going left???
          digitalWrite(leftLED, HIGH);
          moving = true;
        }
        else {
          NoMovement();
          digitalWrite(forwardLED, LOW);
          digitalWrite(leftLED, LOW);
          digitalWrite(rightLED, LOW);
          digitalWrite(backwardLED, LOW);
          moving = false;
        }
      }
      else {
        if (Xbox.getButtonClick(A, 0)) {
          // Button is pressed
         ButtonA_pressed();
        }
      }

      if (!moving) {
        int pos = analogRead(hallSensorPin);                  //Get actuator vertical position
        pos = 10;
        if (Xbox.getAnalogHat(RightHatY , 0) > conThresh) {      //Check if Right Stick is in up direction
          if (pos < limitTop) {                               //Check if actuator can extend
            actuatorUp();
            digitalWrite(upLED, HIGH);
          } else {
            actuatorHold();
            digitalWrite(upLED, LOW);
            digitalWrite(downLED, LOW);
          }
        } 
        else if (Xbox.getAnalogHat(RightHatY, 0) < -conThresh) { //Check if Right Stick is in down direction
          if (pos > limitBot) {                               //Check if actuator can retract
            actuatorDown();
            digitalWrite(downLED, HIGH);
          } 
          else {
            actuatorHold();
            digitalWrite(upLED, LOW);
            digitalWrite(downLED, LOW);
          }
        } 
        else {                                              //If Right Stick is not moved
          actuatorHold();
          digitalWrite(upLED, LOW);
          digitalWrite(downLED, LOW);
        }
      }
      else {
        actuatorHold();
        digitalWrite(upLED, LOW);
        digitalWrite(downLED, LOW);
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
    digitalWrite(stopLED, HIGH);
  }
}

void actuatorUp() {
  int stick = Xbox.getAnalogHat(RightHatY, 0);
  int movement = map(stick, conThresh, conLimit, forwardMin, forwardMax);
  actuator.write(movement);          //or use actrUp to specify constant value
  digitalWrite(upLED, HIGH);
}

void actuatorDown() {
  int stick = Xbox.getAnalogHat(RightHatY, 0);
  int movement = map(stick, -conThresh, -conLimit, reverseMin, reverseMax);
  actuator.write(movement);          //or use actrDown to specify constant value
  digitalWrite(downLED, HIGH);
}

void actuatorHold() {
  actuator.write(stationary);
  digitalWrite(upLED, LOW);
  digitalWrite(downLED, LOW);
}



//Stephanie Stuff for movement
void ForwardMovement() {
  long duration, distance;
  int stick = Xbox.getAnalogHat(LeftHatY, 0);
  
  //getting sensor to transmit??
  //will this give a delay?? We would need to make sure it is quick enough not to notice...
  digitalWrite(ultraSensorTX, LOW);
  delayMicroseconds(10);
  digitalWrite(ultraSensorTX, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultraSensorTX, LOW);
  
  duration = pulseIn(ultraSensorRX, HIGH);
  distance = (duration/2) / 29.1;     //distance is in cm and is what value was returned
  
  if (distance > 25) {                //should give about 44 inches assuming 11 inch is 25
    sensorOK = true;
    if (distance >= 100) {
      //continue full speed ahead!
      int movement = map(stick, conThresh, conLimit, forwardMin, forwardMax);
      motor1.write(movement);
      motor2.write(movement);
      digitalWrite(forwardLED, HIGH);
    }
    else if (distance >= 75) {
      //continue at 2 thirds of full speed
      int max = forwardMin + (((forwardMin - forwardMax) * 2) / 3);
      int movement = map(stick, conThresh, conLimit, forwardMin, forwardMax);
      motor1.write(movement);
      motor2.write(movement);
    }
    else if (distance >= 50) {     
      //continue at a third of full speed
      int max = forwardMin + ((forwardMin - forwardMax) / 3);
      int movement = map(stick, conThresh, conLimit, forwardMin, forwardMax);
      motor1.write(movement);
      motor2.write(movement);
    }
    else { //25 should be about 11 inches away from it
      //continue a crawl of full speed
      int max = forwardMin + ((forwardMin - forwardMax) / 6);
      int movement = map(stick, conThresh, conLimit, forwardMin, forwardMax);
      motor1.write(movement);
      motor2.write(movement);
    }
  }
  else {
    //Robot is too close, wait for confirmation to continue moving
    //Should give user feedback, check if blink middle LEDs is possible
    sensorOK = false;
    motor1.write(stationary);
    motor2.write(stationary);
  }
}

void BackwardMovement() {
  //sensorOK = 1;
  //buzzardBackwards = 1; //figure out how to make it speak
  
  int stick = Xbox.getAnalogHat(LeftHatY, 0);
  int movement = map(stick, -conThresh, -conLimit, reverseMin, reverseMax);
  motor1.write(movement);
  motor2.write(movement);
  digitalWrite(backwardLED, HIGH);
  
  //for (int thisNote = 0; thisNote < 3; thisNote++) {
    //use this to calculate noteDuration: quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int thisNote = 0;
    int noteDuration = 10000 / noteDurations[thisNote];
    tone(buzzardBackwards, melody[thisNote], noteDuration);
    //to distinguish the notes, set a minimum time between them.
    //the note's duration + 30% seems to work well:
    //int pauseBetweenNotes = noteDuration * 1.30;
    //delay(pauseBetweenNotes);
    //stop the tone playing:
    //tone(buzzardBackwards, melody[thisNote], noteDuration);
  noTone(13);
  //} 
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
