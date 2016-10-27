#include <XBOXRECV.h>
#include <Servo.h>
#include <Math.h>
#include <Wire.h>

//Sytem variables
const int restartDelay = 2500;          //Value in miliseconds, used when reset combination is pressed during emergency stop
bool curOK = true;
bool ultraOK = true;
bool emergencyStop = false;             //Boolean value, prevents all inputs while true, must reset arduino to continue use
bool moving = false;

//Motor variables
const float slowSpeed = 2.0;       //Value used in division of motor speed for when fast button is not pressed
const int motor1Pin = 6;           //Pin to control speed of motor 1
const int motor2Pin = 7;           //Pin to control speed of motor 2
float motor1Val;                   //Value written to motor1
float motor1ValR;                  //Radius percentage value for motor 1
float motor1ValA;                  //Angle percentage value for motor 1
float motor1ValT;                  //Temp value for motor 1
float motor2Val;                   //Value written to motor 2
float motor2ValR;                  //Radius percentage value for motor 2
float motor2ValA;                  //Angle percentage value for motor 2
float motor2ValT;                  //Temp value for motor 2

//Emergency stop variables
const int emergencyStopPin = 5;
int emergencyStopVal;

//Current sensor variables
const int motor1CurSensPin = 8;
const int motor2CurSensPin = 9;
const float curSenMaxFVol = 4.5;
const float curSenZeroVol = 2.5;
const float curSenMaxRVol = 0.5;
const float maxSensCur = 100.0;
const float maxMotorCurWarn = 50.0;
const float maxMotorCurFault = 80.0;
bool currentOK = false;
float maxFMotorCurRepWarn;
float maxRMotorCurRepWarn;
float motor1Cur;
float motor2Cur;

//Ultrasonic sensor variables
unsigned int HighLen1 = 0;
unsigned int HighLen2 = 0;
unsigned int LowLen1 = 0;
unsigned int LowLen2 = 0;
bool serial2Write = false;
bool serial3Write = false;
bool ultrasoundOK = false;
int loopCount = 0;
unsigned int msbSerial2;
unsigned int lsbSerial2;
unsigned int serial2Dist;
unsigned int msbSerial3;
unsigned int lsbSerial3;
unsigned int serial3Dist;

//Actuator variables
const int actrCtrlPin = 8;              //Pin to control actuator movement
float actrVal;

//Motor controller variables
const float reverseMax = 40.0;              //Value to send to motor controller for maximum reverse speed
const float reverseMin = 82.0;              //Value to send to motor controller for minimum reverse speed
const float stationary = 92.0;              //Value to send to motor controller for no speed
const float forwardMin = 102.0;             //Value to send to motor controller for minimum forward speed
const float forwardMax = 150.0;             //Value to send to motor controller for maximum forward speed

//Controller constant variables
const float joytr = 8000.0;         //Value to check joystick position against for activation (minimum press to move)
const float joym = 33000.0;         //Value for the limit of joystick position (needs verified)
const float joyadd = 0.087266;      //Value for absolute direction threshold - 5 degrees in radians
const float joyatd = 0.087266;      //Value for absolute turn threshold - 5 degrees in radians
const float pi = 3.141593;          //Value of pi
const float pi34 = 2.356194;        //Value of pi*(3/4)
const float pi2 = 1.570796;         //Value of pi/2
const float pi4 = 0.785398;         //Value of pi/4
const float npi = 0.0 - pi;         //Value of negative pi
const float npi34 = 0.0 - pi34;     //Value of negative pi*(3/4)
const float npi2 = 0.0 - pi2;       //Value of negative pi/2
const float npi4 = 0.0 - pi4;       //Value of negative pi/4
//Controller bound variables - variables in radians arranged from 0 to 2pi, atan2 is -pi to pi
const float joyrp = joyadd;                 //Right turn positive bound
const float joyfrn = pi4 - joyatd;          //Forward right turn negative bound
const float joyfrp = pi4 + joyatd;          //Forward right turn postive bound
const float joyfn = pi2 - joyadd;           //Forward negative bound
const float joyfp = pi2 + joyadd;           //Forward positive bound
const float joyfln = pi34 - joyatd;         //Forward left negative bound
const float joyflp = pi34 + joyatd;         //Forward left positive bound
const float joyln = pi - joyadd;            //Left turn negative bound
const float joylp = npi + joyadd;           //Left turn positive bound
const float joylpz = npi + (2.0 * joyatd);  //Backward left turn transisiton bound
const float joybln = npi34 - joyatd;        //Backward left turn negative bound
const float joyblp = npi34 + joyatd;        //Backward left turn positive bound
const float joybn = npi2 - joyadd;          //Backward negative bound
const float joybp = npi2 + joyadd;          //Backward positive bound
const float joybrn = npi4 - joyatd;         //Backward right turn negative bound
const float joybrp = npi4 + joyatd;         //Backward right turn positive bound
const float joyrpz = 0.0 - (2.0 * joyatd);  //Backward right turn transistion bound
const float joyrn = 0.0 - joyadd;           //Right turn negative bound

USB Usb;
XBOXRECV Xbox(&Usb);
Servo motor1;
Servo motor2;
Servo actuator;

void setup() {
    motor1.attach(motor1Pin);
    motor2.attach(motor2Pin);
    actuator.attach(actrCtrlPin);
    motor1.write(stationary);
    motor2.write(stationary);
    actuator.write(stationary);
    maxFMotorCurRepWarn = mapFloat(maxMotorCur, 0, maxSensCurWarn, curSenZeroVol, curSenMaxFVol);
    maxRMotorCurRepWarn = mapFloat(maxMotorCur, 0, maxSensCurWarn, curSenZeroVol, curSenMaxRVol);
    maxFMotorCurRepFault = mapFloat(maxMotorCur, 0, maxSensCurFault, curSenZeroVol, curSenMaxFVol);
    maxRMotorCurRepFault = mapFloat(maxMotorCur, 0, maxSensCurFault, curSenZeroVol, curSenMaxRVol);
    Serial.begin(115200);
    Serial1.begin(115200);
    Serial2.begin(9600);
    Serial3.begin(9600);
    Usb.Init(); 
}

void loop() {
    Usb.Task();
    if (Xbox.XboxReceiverConnected) {

        //Emergency Stop Sensor Read
        emergencyStopVal = digitalRead(5);  
        if (emergencyStopVal == 0) {
            emergencyStop = true;
        }
        //Current Sensor Read
        analogRead(motor1CurSensPin);
        delayMicroseconds(10);
        motor1Cur = analogRead(motor1CurSensPin);
        analogRead(motor2CurSensPin);
        delayMicroseconds(10);
        motor2Cur = analogRead(motor2CurSensPin);
        if ((motor1Cur >= maxFMotorCurRepWarn)&&(motor1Cur < maxFMotorCurRepFault)) {
          currentWarn;
        }
        else if ((motor1Cur <= maxRMotorCurRepWarn)&&(motor1Cur > maxRMotorCurRepFault)) {
          currentWarn;
        }
        else if ((motor1Cur >= maxFMotorCurRepFault)||(motor1Cur <= maxRMotorCurRepFault) {
          emergencyStop = true;
        }
        if ((motor2Cur >= maxFMotorCurRepWarn)&&(motor2Cur < maxFMotorCurRepFault)) {
          currentWarn;
        }
        else if ((motor2Cur <= maxRMotorCurRepWarn)&&(motor2Cur > maxRMotorCurRepFault)) {
          currentWarn;
        }
        else if ((motor2Cur >= maxFMotorCurRepFault)||(motor2Cur <= maxRMotorCurRepFault) {
          emergencyStop = true;
        }
        //Ultrasound Sensor Read
        if (serial2Write) { //Check current ultrasound process for Serial2
          if (Serial2.available() >= 2) { //Read returned ultrasound data
            msbSerial2 = Serial2.read();
            lsbSerial2 = Serial2.read();
            serial2Dist = msbSerial2 * 256 + lsbSerial2;
          }
          serial2Write = false;
        }
        else {  //Write ultrasound trigger command
          Serial2.flush();
          Serial2.write(0x55);
          serial3Write = true;
        }
        if (serial3Write) { //Check current ultrasound process for Serial3
          if (Serial2.available() >= 2) { //Read returned ultrasound data
            msbSerial3 = Serial3.read();
            lsbSerial3 = Serial3.read();
            serial3Dist = msbSerial3 * 256 + lsbSerial3;
          }
          serial3Write = false;
        }
        else {  //Write ultrasound trigger command
          Serial3.flush();
          Serial3.write(0x55);
          serial3Write = true;
        }
          
        
        if (!emergencyStop) {
            
           
            
            
            if (ultrasoundOK) {
                float joyy = (float)Xbox.getAnalogHat(LeftHatY, 0);
                float joyx = (float)Xbox.getAnalogHat(LeftHatX, 0);
                float joyr = (float)sqrt(square(joyy) + square(joyx));
                if (joyr > joytr) { //Limit joystick radius to threshold
                    moving = true;
                    if (joyr > joym ) { //Limit joystick radius to maximum
                        joyr = joym;    
                    }
                    float joya = (float)atan2(joyy, joyx);
                    if ((joya > joyrn)||(joya <= joylp)) {  //FORWARD QUADRANT
                        if ((joya > joyrn)&&(joya < joyrp)) { //Right Turn
                          motor1Val = (float)map(joyr, joytr, joym, forwardMin, forwardMax);
                          motor2Val = (float)map(joyr, joytr, joym, forwardMin, forwardMax);
                        }
                        else if ((joya >= joyrp)&&(joya <= joyfn)) {  //Forward Right Turn
                            motor1Val = (float)map(joyr, joytr, joym, forwardMin, forwardMax);
                            if ((joya >= joyrp)&&(joya <= joyfrn)) { //Turn Motor Reverse
                              motor2ValR = (float)mapFloat(joyr, joytr, joym, 0.0, 1.0);
                              motor2ValA = (float)mapFloat(joya, joyrp, joyfrn, 1.0, 0.0);
                              motor2Val = (float)mapFloat((motor2ValR * motor2ValA), 0.0, 1.0, forwardMin, forwardMax);
                            }
                            else if ((joya > joyfrn)&&(joya < joyfrp)) {  //Turn Motor Stopped
                              motor2Val = stationary;
                            }
                            else if ((joya >= joyfrp)&&(joya <= joyfn)) { //Turn Motor Forward
                              motor2ValR = (float)mapFloat(joyr, joytr, joym, 0.0, 1.0); 
                              motor2ValA = (float)mapFloat(joya, joyfrp, joyfn, 0.0, 1.0);
                              motor2Val = (float)mapFloat((motor2ValR * motor2ValA), 0.0, 1.0, reverseMin, reverseMax);
                            }
                        }
                        else if ((joya > joyfn)&&(joya < joyfp)) {  //Forward
                          motor1Val = (float)map(joyr, joytr, joym, forwardMin, forwardMax);
                          motor2Val = (float)map(joyr, joytr, joym, reverseMin, reverseMax);
                        }
                        else if ((joya >= joyfp)&&(joya <= joyln)) {  //Forward Left Turn
                            motor2Val = (float)map(joyr, joytr, joym, reverseMin, reverseMax);
                            if ((joya >= joyfp)&&(joya <= joyfln)) { //Turn Motor Forward
                              motor1ValR = (float)mapFloat(joyr, joytr, joym, 0.0, 1.0);
                              motor1ValA = (float)mapFloat(joya, joyfp, joyfln, 1.0, 0.0);
                              motor1Val = (float)mapFloat((motor1ValR * motor1ValA), 0.0, 1.0, forwardMin, forwardMax);
                            }
                            else if ((joya > joyfln)&&(joya < joyflp)) {  //Turn Motor Stopped
                              motor1Val = stationary;
                            }
                            else if ((joya >= joyflp)&&(joya <= joyln)) { //Turn Motor Reverse
                              motor1ValR = (float)mapFloat(joyr, joytr, joym, 0.0, 1.0); 
                              motor1ValA = (float)mapFloat(joya, joyflp, joyln, 0.0, 1.0);
                              motor1Val = (float)mapFloat((motor1ValR * motor1ValA), 0.0, 1.0, reverseMin, reverseMax);
                            }
                        }
                        else if ((joya > joyln)||(joya < joylp)) {  //Left Turn
                          motor1Val = (float)map(joyr, joytr, joym, reverseMin, reverseMax);
                          motor2Val = (float)map(joyr, joytr, joym, reverseMin, reverseMax);
                        }
                    }
                    else if ((joya > joylp)&&(joya <= joylpz)) {  //DEAD ZONE
                        motor1Val = stationary;
                        motor2Val = stationary;
                    }
                    else if ((joya > joylpz)&&(joya <= joyrpz)) { //BACKWARD QUADRANT
                        if ((joya >= joylp)&&(joya <= joybn)) {  //Backward Left Turn
                            motor2Val = (float)map(joyr, joytr, joym, forwardMin, forwardMax);
                            if ((joya > joylpz)&&(joya <= joybln)) { //Turn Motor Forward
                              motor1ValR = (float)mapFloat(joyr, joytr, joym, 0.0, 1.0);
                              motor1ValA = (float)mapFloat(joya, joylpz, joybln, 1.0, 0.0);
                              motor1Val = (float)mapFloat((motor1ValR * motor1ValA), 0.0, 1.0, forwardMin, forwardMax);
                            }
                            else if ((joya > joybln)&&(joya < joyblp)) {  //Turn Motor Stopped
                              motor1Val = stationary;
                            }
                            else if ((joya >= joyblp)&&(joya <=joybn)) {  //Turn Motor Reverse
                              motor1ValR = (float)mapFloat(joyr, joytr, joym, 0.0, 1.0); 
                              motor1ValA = (float)mapFloat(joya, joyblp, joybn, 0.0, 1.0);
                              motor1Val = (float)mapFloat((motor1ValR * motor1ValA), 0.0, 1.0, reverseMin, reverseMax);
                            }
                        }
                        else if ((joya > joybn)&&(joya < joybp)) {  //Backward
                          motor1Val = (float)map(joyr, joytr, joym, reverseMin, reverseMax);
                          motor2Val = (float)map(joyr, joytr, joym, forwardMin, forwardMax);
                        }
                        else if ((joya >= joybp)&&(joya <= joyrn)) {  //Backward Right Turn
                            motor1Val = (float)map(joyr, joytr, joym, reverseMin, reverseMax);
                            if ((joya >= joybp)&&(joya <= joybrn)) { //Turn Motor Reverse
                              motor2ValR = (float)mapFloat(joyr, joytr, joym, 0.0, 1.0);
                              motor2ValA = (float)mapFloat(joya, joybp, joybrn, 1.0, 0.0);
                              motor2Val = (float)mapFloat((motor2ValR * motor2ValA), 0.0, 1.0, forwardMin, forwardMax);
                            }
                            else if ((joya > joybrn)&&(joya < joybrp)) {  //Turn Motor Stopped
                              motor2Val = stationary;
                            }
                            else if ((joya > joybrp)&&(joya <= joyrpz)) { //Turn Motor Forward
                              motor2ValR = (float)mapFloat(joyr, joytr, joym, 0.0, 1.0); 
                              motor2ValA = (float)mapFloat(joya, joybrp, joyrpz, 0.0, 1.0);
                              motor2Val = (float)mapFloat((motor2ValR * motor2ValA), 0.0, 1.0, reverseMin, reverseMax);
                            }
                        }
                    }
                    else if ((joya > joyrpz)&&(joya <= joyrn)) {  //DEAD ZONE
                        motor1Val = stationary;
                        motor2Val = stationary;
                    }
                }
                else {  //No Left Joystick Movement = not moving
                    moving = false;
                    motor1Val = stationary;
                    motor2Val = stationary;
                    if (Xbox.getAnalogHat(RightHatY, 0) > joytr) { //Controll actuator based on the right stick
                      actrVal = forwardMax;
                    }
                    else if (Xbox.getAnalogHat(RightHatY, 0) < -joytr) {
                      actrVal = reverseMax;
                    }
                    else {
                      actrVal = stationary;
                    }
                }
                if (!Xbox.getButtonPress(Y, 0)) { //Slow down the motor speed if Y is not held
                    if (motor1Val >= forwardMin) {
                        motor1Val = ((motor1Val - forwardMin) / slowSpeed) + forwardMin;
                    }
                    else if (motor1Val =< reverseMin) {
                        motor1Val = reverseMin - ((reverseMin - motor1Val) / slowSpeed);
                    }
                    if (motor2Val >= forwardMin) {
                        motor2Val = ((motor2Val - forwardMin) / slowSpeed) + forwardMin;
                    }
                    else if (motor2Val =< reverseMin) {
                        motor2Val = reverseMin - ((reverseMin - motor2Val) / slowSpeed);
                    }
                }
                motor1.write(motor1Val);    //Write motor and actuator values
                motor2.write(motor2Val);
                actuator.write(actrVal);
            } 
            if (Xbox.getButtonPress(B, 0)) { //Emergency stop protocol
                emergencyStop = true;
                motor1.write(stationary);
                motor2.write(stationary);
                actuator.write(stationary);
            }
        }
        else {  //If an emergency stop has been issued
            if (Xbox.getButtonPress(A, 0)&&(Xbox.getButtonPress(B, 0)) { //Press A+B to reset emergency stop
                delay(restartDelay); 
                emergencyStop = false;
            }
        }
    }
}

void warnCurrent() {  //Warn about high current
  
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) { //Map function for float numbers
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
