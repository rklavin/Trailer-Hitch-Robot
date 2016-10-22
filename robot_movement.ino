#include <XBOXRECV.h>
#include <Servo.h>
#include <Math.h>

//System variables
const int restartDelay = 5000;          //Value in miliseconds, used when reset combination is pressed during emergency stop

//Global flags
bool sensorOK = true;                   //Boolean value checked when left stick is moved
bool emergencyStop = false;             //Boolean value, prevents all inputs while true, must reset arduino to continue use

//Motor variables
const float slowSpeed = 2.0;       //Value used in division of motor speed for when fast button is not pressed
const int motor1Pin = 6;           //Pin to control speed of motor 1
const int motor2Pin = 7;           //Pin to control speed of motor 2
float motor1Val;                   //Value written to motor1
float motor1ValR;                  //Radius value for motor 1
float motor1ValA;                  //Angle value for motor 1
float motor2Val;                   //Value written to motor 2
float motor2ValR;                  //Radius value for motor 2
float motor2ValA;                  //Angle value for motor 2
float motor1ValT;                  //Temp value for motor 1
float motor2ValT;                  //Temp value for motor 2

//Actuator variables
const int actrCtrlPin = 8;              //Pin to control actuator movement
const int hallSensorPin = A0;           //Pin to read Hall Effect Sensor

//Motor controller variables
const float reverseMax = 40.0;              //Value to send to motor controller for maximum reverse speed
const float reverseMin = 82.0;              //Value to send to motor controller for minimum reverse speed
const float stationary = 92.0;              //Value to send to motor controller for no speed
const float forwardMin = 102.0;             //Value to send to motor controller for minimum forward speed
const float forwardMax = 150.0;             //Value to send to motor controller for maximum forward speed

//Ultrasonic sensor variables
const int ultraSensorTX = 14;           //Pin for trig
const int ultraSensorRX = 15;           //Pin for echo
const int buzzardBackwards = 13;        //Pin to send signal to buzzer

//Controller constant variables
const float joytr = 8000.0;         //Value to check joystick position against for activation (minimum press to move)
const float joym = 33000.0;         //Value for the limit of joystick position (needs verified)
const float joyadd = 0.087266;      //Value for absolute direction threshold - 2.5 degrees in radians
const float joyatd = 0.087266;      //Value for absolute turn threshold - 2.5 degrees in radians
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
    Serial.begin(9600);
    Usb.Init(); 
}

void loop() {
    Usb.Task();
    if (Xbox.XboxReceiverConnected) {
        if (!emergencyStop) {
            bool moving = false;
            if (sensorOK) {
                float joyy = (float)Xbox.getAnalogHat(LeftHatY, 0);
                float joyx = (float)Xbox.getAnalogHat(LeftHatX, 0);
                float joyr = (float)sqrt(square(joyy) + square(joyx));
                if ( joyr > joytr ) { //Limit joystick radius to threshold
                    if (joyr > joym ) { //Limit joystick radius to maximum
                        joyr = joym;    
                    }
                    float joya = (float)atan2(joyy, joyx);
                    if ( (joya > joyrn) || (joya <= joylp) ) {  //FORWARD QUADRANT
                        if ( (joya > joyrn) && (joya < joyrp) ) { //Right Turn
                          motor1Val = (float)map(joyr, joytr, joym, forwardMin, forwardMax);
                          motor2Val = (float)map(joyr, joytr, joym, forwardMin, forwardMax);
                        }
                        else if ( (joya >= joyrp) && (joya <= joyfn) ) {  //Forward Right Turn
                            motor1Val = (float)map(joyr, joytr, joym, forwardMin, forwardMax);
                            if ( (joya >= joyrp) && (joya <= joyfrn) ) { //Turn Motor Reverse
                              motor2ValR = (float)mapFloat(joyr, joytr, joym, 0.0, 1.0);
                              motor2ValA = (float)mapFloat(joya, joyrp, joyfrn, 1.0, 0.0);
                              motor2ValT = (float)(motor2ValR * motor2ValA);
                              motor2Val = (float)mapFloat(motor2ValT, 0.0, 1.0, forwardMin, forwardMax);
                            }
                            else if ( (joya > joyfrn) && (joya < joyfrp) ) {  //Turn Motor Stopped
                              motor2Val = stationary;
                            }
                            else if ( (joya >= joyfrp) && (joya <= joyfn) ) { //Turn Motor Forward
                              motor2ValR = (float)mapFloat(joyr, joytr, joym, 0.0, 1.0); 
                              motor2ValA = (float)mapFloat(joya, joyfrp, joyfn, 0.0, 1.0);
                              motor2ValT = (float)(motor2ValR * motor2ValA); 
                              motor2Val = (float)mapFloat(motor2ValT, 0.0, 1.0, reverseMin, reverseMax);
                            }
                        }
                        else if ( (joya > joyfn) && (joya < joyfp) ) {  //Forward
                          motor1Val = (float)map(joyr, joytr, joym, forwardMin, forwardMax);
                          motor2Val = (float)map(joyr, joytr, joym, reverseMin, reverseMax);
                        }
                        else if ( (joya >= joyfp) && (joya <= joyln) ) {  //Forward Left Turn
                            motor2Val = (float)map(joyr, joytr, joym, reverseMin, reverseMax);
                            if ( (joya >= joyfp) && (joya <= joyfln) ) { //Turn Motor Forward
                              motor1ValR = (float)mapFloat(joyr, joytr, joym, 0.0, 1.0);
                              motor1ValA = (float)mapFloat(joya, joyfp, joyfln, 1.0, 0.0);
                              motor1ValT = (float)(motor1ValR * motor1ValA);
                              motor1Val = (float)mapFloat(motor1ValT, 0.0, 1.0, forwardMin, forwardMax);
                            }
                            else if ( (joya > joyfln) && (joya < joyflp) ) {  //Turn Motor Stopped
                              motor1Val = stationary;
                            }
                            else if ( (joya >= joyflp) && (joya <= joyln) ) { //Turn Motor Reverse
                              motor1ValR = (float)mapFloat(joyr, joytr, joym, 0.0, 1.0); 
                              motor1ValA = (float)mapFloat(joya, joyflp, joyln, 0.0, 1.0);
                              motor1ValT = (float)(motor1ValR * motor1ValA); 
                              motor1Val = (float)mapFloat(motor1ValT, 0.0, 1.0, reverseMin, reverseMax);
                            }
                        }
                        else if ( (joya > joyln) || (joya < joylp) ) {  //Left Turn
                          motor1Val = (float)map(joyr, joytr, joym, reverseMin, reverseMax);
                          motor2Val = (float)map(joyr, joytr, joym, reverseMin, reverseMax);
                        }
                    }
                    else if ( (joya > joylp) && (joya <= joylpz) ) {  //DEAD ZONE
                        motor1Val = stationary;
                        motor2Val = stationary;
                    }
                    else if ( (joya > joylpz) && (joya <= joyrpz) ) { //BACKWARD QUADRANT
                        if ( (joya >= joylp) && (joya <= joybn) ) {  //Backward Left Turn
                            motor2Val = (float)map(joyr, joytr, joym, forwardMin, forwardMax);
                            if ( (joya > joylpz) && (joya <= joybln) ) { //Turn Motor Forward
                              motor1ValR = (float)mapFloat(joyr, joytr, joym, 0.0, 1.0);
                              motor1ValA = (float)mapFloat(joya, joylpz, joybln, 1.0, 0.0);
                              motor1ValT = (float)(motor1ValR * motor1ValA);
                              motor1Val = (float)mapFloat(motor1ValT, 0.0, 1.0, forwardMin, forwardMax);
                            }
                            else if ( (joya > joybln) && (joya < joyblp) ) {  //Turn Motor Stopped
                              motor1Val = stationary;
                            }
                            else if ( (joya >= joyblp) && (joya <=joybn) ) {  //Turn Motor Reverse
                              motor1ValR = (float)mapFloat(joyr, joytr, joym, 0.0, 1.0); 
                              motor1ValA = (float)mapFloat(joya, joyblp, joybn, 0.0, 1.0);
                              motor1ValT = (float)(motor1ValR * motor1ValA); 
                              motor1Val = (float)mapFloat(motor1ValT, 0.0, 1.0, reverseMin, reverseMax);
                            }
                        }
                        else if ( (joya > joybn) && (joya < joybp) ) {  //Backward
                          motor1Val = (float)map(joyr, joytr, joym, reverseMin, reverseMax);
                          motor2Val = (float)map(joyr, joytr, joym, forwardMin, forwardMax);
                        }
                        else if ( (joya >= joybp) && (joya <= joyrn) ) {  //Backward Right Turn
                            motor1Val = (float)map(joyr, joytr, joym, reverseMin, reverseMax);
                            if ( (joya >= joybp) && (joya <= joybrn) ) { //Turn Motor Reverse
                              motor2ValR = (float)mapFloat(joyr, joytr, joym, 0.0, 1.0);
                              motor2ValA = (float)mapFloat(joya, joybp, joybrn, 1.0, 0.0);
                              motor2ValT = (float)(motor2ValR * motor2ValA);
                              motor2Val = (float)mapFloat(motor2ValT, 0.0, 1.0, forwardMin, forwardMax);
                            }
                            else if ( (joya > joybrn) && (joya < joybrp) ) {  //Turn Motor Stopped
                              motor2Val = stationary;
                            }
                            else if ( (joya > joybrp) && (joya <= joyrpz) ) { //Turn Motor Forward
                              motor2ValR = (float)mapFloat(joyr, joytr, joym, 0.0, 1.0); 
                              motor2ValA = (float)mapFloat(joya, joybrp, joyrpz, 0.0, 1.0);
                              motor2ValT = (float)(motor2ValR * motor2ValA); 
                              motor2Val = (float)mapFloat(motor2ValT, 0.0, 1.0, reverseMin, reverseMax);
                            }
                        }
                    }
                    else if ( (joya > joyrpz) && (joya <= joyrn) ) {  //DEAD ZONE
                        motor1Val = stationary;
                        motor2Val = stationary;
                    }
                }
                else {  //No Joystick Movement
                    motor1Val = stationary;
                    motor2Val = stationary;
                }
                /*if ( !Xbox.getButtonPress(Y, 0) ) {
                    if ( motor1Val > forwardMin ) {
                        motor1Val = map(motor1Val, forwardMin, forwardMax, forwardMin, ( forwardMax / slowSpeed ));
                    }
                    else if ( motor1Val < reverseMin ) {
                        motor1Val = map(motor1Val, reverseMin, reverseMax, reverseMin, ( reverseMax / slowSpeed ));
                    }
                    if ( motor2Val > forwardMin ) {
                        motor2Val = map(motor2Val, forwardMin, forwardMax, forwardMin, ( forwardMax / slowSpeed ));
                    }
                    else if ( motor2Val < reverseMin ) {
                        motor2Val = map(motor2Val, reverseMin, reverseMax, reverseMin, ( reverseMax / slowSpeed ));
                    }
                }*/
                motor1.write(motor1Val);
                motor2.write(motor2Val);
                
               /* Serial.print(motor1Val);
                Serial.print(",");
                Serial.println(motor2Val);*/
            } 
            if ((Xbox.getButtonClick(B, 0)) || (false)) { //Emergency stop protocol
                //change false^ to check the pin the emergency stop button is on
                EmergencyStop();
            }
        }
        else {  //If an emergency stop button has been pressed
            if ((Xbox.getButtonPress(A, 0)) && (Xbox.getButtonPress(B, 0)) && (Xbox.getButtonPress(X, 0)) && (Xbox.getButtonPress(Y, 0))) {
                delay(restartDelay);  //Combination of A+B+X+Y must be pressed to release emegency stop
                emergencyStop = false;
            }
        }
    }
}

void EmergencyStop() {
  //Stop all current robot movement and prevent future movements
  emergencyStop = true;
  motor1.write(stationary);
  motor2.write(stationary);
  actuator.write(stationary);
}

//float mapFloat(float a, float b, float c, float d, float e) {
//  long temp = map(a*100.0, b*100.0, c*100.0, d*100.0, e*100.0);
//  return temp / 100.0;
//}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
