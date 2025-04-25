#include <Servo.h>
#include <Wire.h>               // Must include Wire library for I2C
#include "SparkFun_MMA8452Q.h"  // Click here to get the library: http://librarymanager/All#SparkFun_MMA8452Q

#include <PID_v1.h>

MMA8452Q accel;  // create instance of the MMA8452 class
Servo myservo;   // create servo object to control a servo


//Define Variables we'll be connecting to in PID library
double Setpoint, Input, Output;
const int DELTA_LIMIT = 25;

// Timing variables
unsigned long timingVar;
double pidTimingInterval;

// Servo angles
int servoAngle, lastServoAngle = 0;

//Define Tuning Parameters
double Kp = 1, Ki = 0.05, Kd = 0.25;

int potPin = A0;  // Input pin from the potentiometer

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);


void setup() {
  Serial.begin(9600);
  Serial.println("MMA8452Q Basic Servo driving Code!");

  Serial.println("Driving servo to midpoint...");
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  myservo.write(90);  // start the servo at its mid-point
  Setpoint = 0;

  Wire.begin();

  // indefinitely stall if accelerometer isn't returning data?
  while (accel.begin() == false) {
    Serial.println("Not Connected. Please check connections and read the hookup guide.");
    delay(250);
  }
  //  ODR_6 means set sample rate to 6.25Hz
  //  ODR_12 means set sample rate to 12.5Hz
  //  ODR_50 50Hz
  //  See: https://www.nxp.com/docs/en/data-sheet/MMA8452Q.pdf

  // Set the accelerometer and PID interval to the same time-frame.
  accel.setDataRate(ODR_12);
  pidTimingInterval = 1000.0 / 12.5;
  myPID.SetSampleTime(pidTimingInterval);

  // Misc PID settings
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-90.0, 90.0);
}

void loop() {
  // Ensure sample time has passed before applying calculations.
  if (accel.available() && millis() > timingVar + pidTimingInterval) {
    timingVar = millis();
    // The accelerometer measures the angle of the gimbal. We use a rolling average to smooth the input.
    Input = rollingAverageIn(readAngle());

    tunePid();
    // PID is aiming for zero...
    myPID.Compute();

    // cameraAngle = Output;  //- Input;
    // Update the servo.
    lastServoAngle = servoAngle;
    servoAngle = map(Output, 90, -90, 0, 180);

    // restrict movement to a certain amount of degrees...
    servoAngle = limitMovement(servoAngle, lastServoAngle);

    myservo.write(servoAngle);
  }
}


double readAngle() {
  // Get the raw data
  //accel.read();  // Is this necessary? ##
  double x = accel.getCalculatedX();
  double y = accel.getCalculatedY();
  double z = accel.getCalculatedZ();

  // Calculate tilt angles in degrees
  //double tilt_x = atan2(x, sqrt(y * y + z * z)) * (180 / PI);
  double tilt_y = atan2(y, sqrt(x * x + z * z)) * (180 / PI);

  return tilt_y;
}


double mapd(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

const int WINDOW_SIZE = 4;
double valuesIn[WINDOW_SIZE];
int insertPos = 0;

double rollingAverageIn(double newValue) {
  valuesIn[insertPos] = newValue;
  insertPos = (insertPos + 1) % WINDOW_SIZE;
  double sum = 0.0;
  for (int i = 0; i < WINDOW_SIZE; i++) {
    sum += valuesIn[i];
  }
  return sum / WINDOW_SIZE;
}

void tunePid() {
  // Kp=.37; Ki=.02; Kd=.02;
  Kp = 0.93;
  Ki = 0.0034;
  Kd = 0.0025;  // + mapd(analogRead(potPin), 0, 1023, 0, Kp/10);

  myPID.SetTunings(Kp, Ki, Kd);
}

int limitMovement(int requestedMovement, int lastServoAngle) {
  int deltaLimit = DELTA_LIMIT;  //mapd(analogRead(potPin), 0, 1023, 0, 20);

  if (requestedMovement - lastServoAngle > deltaLimit) {
    return lastServoAngle + deltaLimit;
  }
  if (requestedMovement - lastServoAngle < -deltaLimit) {
    return lastServoAngle - deltaLimit;
  }
  return requestedMovement;
}

void debugLog() {
  Serial.print("Kp=");
  Serial.print(Kp);
  Serial.print(",Ki=");
  Serial.print(Ki, 4);
  Serial.print(",Kd=");
  Serial.print(Kd, 4);
  Serial.print(",△Limit=");
  Serial.print(DELTA_LIMIT);
  Serial.print(",in=");
  Serial.print(Input);
  Serial.print(",out=");
  Serial.print(Output);
  Serial.print(",servoAngle=");
  Serial.print(servoAngle - 90);

  Serial.println("");
}