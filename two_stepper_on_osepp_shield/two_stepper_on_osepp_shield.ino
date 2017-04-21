#include <Servo.h>

// ConstantSpeed.pde
// -*- mode: C++ -*-
//
// Shows how to run AccelStepper in the simplest,
// fixed speed mode with no accelerations
// Requires the AFMotor library (https://github.com/adafruit/Adafruit-Motor-Shield-library)
// And AccelStepper with AFMotor support (https://github.com/adafruit/AccelStepper)
// Public domain!

// #include <AccelStepper.h>
#include <AFMotor.h>

#include <NewPing.h>

AF_Stepper motor1(200, 1);
AF_Stepper motor2(200, 2);



#define TRIG_PIN A5 // Pin A4 on the Motor Drive Shield connected to the ultrasonic sensor
#define ECHO_PIN A4 // Pin A5 on the Motor Drive Shield connected to the ultrasonic sensor
#define MAX_DISTANCE_POSSIBLE 400 // sets maximum useable sensor measuring distance to 1000cm
#define MAX_SPEED 120 // sets speed of DC traction motors to 120/256 or about 47% of full speed - to reduce power draining.
NewPing sonar(TRIG_PIN, ECHO_PIN, MAX_DISTANCE_POSSIBLE); // sets up sensor library to use the correct pins to measure distance.
Servo servo1; //, servo2;
 

uint8_t numstep = SINGLE;


// you can change these to DOUBLE or INTERLEAVE or MICROSTEP!
void forwardstep2() {  
  motor2.onestep(FORWARD, numstep);
}
void backwardstep2() {  
  motor2.onestep(BACKWARD, numstep);
}


// you can change these to DOUBLE or INTERLEAVE or MICROSTEP!
void forwardstep1() {  
  motor1.onestep(FORWARD, numstep);
}
void backwardstep1() {  
  motor1.onestep(BACKWARD, numstep);
}

//AccelStepper stepper1(forwardstep1, backwardstep1); // use functions to step
//AccelStepper stepper2(forwardstep2, backwardstep2); // use functions to step

bool stopRunning = true;

int mySpeed = 50;
int accel = 0;
long stepCount = 0;
long maxCount = 100000;

int lastAngle = 90;

void setup()
{  
   Serial.begin(57600);           // set up Serial library at 9600 bps
   Serial.println("Stepper test!");
  
   // stepper1.setSpeed(mySpeed);	
   // stepper2.setSpeed(mySpeed);
   //servo2.attach(9);
   //servo2.write(90);
   servo1.attach(10);
   servo1.write(90);
   
}

/*
void setMySpeed(int myspeed) {
  stepper1.setSpeed(myspeed);
  stepper2.setSpeed(myspeed);
}

void setAccel(int accelx) {
  stepper1.setAcceleration(accelx);
  stepper2.setAcceleration(accelx);
}
*/

int getClearance() {
  int distance = sonar.ping_cm();
  return distance;
}

int takeReading() {
  int distance = sonar.ping_cm();
  delay(50);
  Serial.println("distance " + String(distance));
  return distance;
}

void gotoAngle(int angle) {
  servo1.write(angle);
  delay(100);
  
}

/*
void scanSurrounding(int mystep) {
  Serial.print("#s");
  servo1.write(0);
  delay(100);
  for (int i = 0; i <= 180; i += mystep) {
    servo1.write(i);
    delay(30);
    long t1 = millis();
    int distance = sonar.ping_cm();
    delay(30);
    long t2 = millis();
    Serial.print('[');
    Serial.print(t2 - t1);
    Serial.print(']');
    if (i != 0) {
      Serial.print(',');
    }
    Serial.print(distance);
  }
  servo1.write(90);
  Serial.println("$");
}
*/


char ch;
long delayMs = 3;

long rotDelayMultiplier = 2;

float kStepsPerCM = 11.1;
// wheel is ~ 2cm
// end to end including both wheels is 22.5cm
// net = 20.4 => radius = 10.25cm
// for theta deg => (theta * pi/180) radians => (theta*pi/180*10.25) wheel movement
// for number of steps => theta * (pi/180) * 10.25 * kSTepsPerCM * 10 / distanceScaleFactor
// => theta * (pi/180) * 10.25 * 100/9
// float kStepsPerDeg = (pi/180) * 10.25 * kStepsPerCM;
float pi = 3.1416;
float kDistancePerDeg = (pi/180) * 10.25;

float kMinDistanceToProceed = 10;
String getNextCommand();

void loop()
{
  String cmd = getNextCommand();
  if (cmd.length() == 0) {
    delay(1);
    return;
  }

  if (cmd[0] == 'x') {
    motor1.release();
    motor2.release();
  }

  if (cmd[0] == 'f') {
    /*
    if (lastAngle != 90) {
      servo2.write(90);
      delay(1000);
      lastAngle = 90;
    }
    */
    int distance = cmd.substring(1).toInt();
    distance = (int)(kStepsPerCM * distance);
    // wiring is screwed up !!
    uint8_t dir = BACKWARD;
    if (distance < 0) {
      dir = FORWARD;
      distance = abs(distance);
    }
    // bool too_close = false;
    for (int i = 0; i < distance; i++) {
      /*
      if (i % 100 == 0) {
        while (takeReading() < kMinDistanceToProceed) {
          delay(100);
        }
      }
      */
      delay(delayMs);
      motor1.onestep(dir, SINGLE);
      motor2.onestep(dir, SINGLE);
    }
    /*
    if (too_close) {
      Serial.print("#o$");
    }
    */
    motor1.release();
    motor2.release();
    
    
  }

  if (cmd[0] == 'r') {
    /*
    if (lastAngle != 0) {
      servo2.write(0);
      delay(1000);
      lastAngle = 0;
    }
    */
    int angle = cmd.substring(1).toInt();
    angle = (int)(kStepsPerCM * kDistancePerDeg * angle);
    // wiring is screwed up again !!!!
    
    uint8_t dir = BACKWARD;
    uint8_t revdir = FORWARD;
    if (angle < 0) {
      dir = FORWARD;
      revdir = BACKWARD;
      angle = abs(angle);
    }
    for (int i = 0; i < angle; i++) {
      delay(rotDelayMultiplier * delayMs);
      motor1.onestep(dir, SINGLE);
      motor2.onestep(revdir, SINGLE);
    }
    motor1.release();
    motor2.release();    
    
  }

  /*
  if (cmd[0] == 'S') {
    int mystep = cmd.substring(1).toInt();
    // scanSurrounding(mystep);
  }
  */

  if (cmd[0] == 'd') {
    delayMs = cmd.substring(1).toInt();
    Serial.println("delayMs" + String(delayMs));
  }

  if (cmd[0] == 's') {
    kStepsPerCM =  cmd.substring(1).toFloat();
    
    Serial.println("kStepsPerCM" + String(kStepsPerCM));
  }

  if (cmd[0] == 'g') {
    int angle = cmd.substring(1).toInt();
    gotoAngle(angle);
    
  }

  if (cmd[0] == 'f' || cmd[0] == 'r' || cmd[0] == 'g') {
    Serial.println("#r$");
  }

  /*
  if (cmd[0] == 't') {
    takeReading();
  }
  */
}


String accumulated = "";

String getNextCommand() {
  while(Serial.available()) {
    int inByte = Serial.read();
    char val = (char)inByte;
    if (val == ';') {
      String nextCommand = accumulated;
      accumulated = "";
      Serial.println(nextCommand);
      return nextCommand;
    } else {
      accumulated = accumulated + val;
    } 
  }
  // Because we didnt find a semicolon
  return "";
}

