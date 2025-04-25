#include <Servo.h>
#include <Wire.h>               // Must include Wire library for I2C
#include "SparkFun_MMA8452Q.h"  // Click here to get the library: http://librarymanager/All#SparkFun_MMA8452Q
#include <AccelStepper.h>
#include <PID_v1.h>

MMA8452Q accel;  // create instance of the MMA8452 class


//Define Variables we'll be connecting to in PID library
double Setpoint, Input, Output;

// Timing variables
unsigned long timingVar;
double pidTimingInterval;

// Stepper angle
int stepperAngle = 0;
const int stepsPerRevolution = 2048;

//Define Tuning Parameters
double Kp = 1, Ki = 0.05, Kd = 0.25;

int potPin = A0;  // Input pin from the potentiometer

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
AccelStepper stepper = AccelStepper(AccelStepper::FULL4WIRE, 8, 10, 9, 11);

void setup() {
  Serial.begin(9600);
  Serial.println("MMA8452Q Basic Stepper driving Code!");

  Serial.println("Driving stepper to midpoint...");
  stepper.moveTo(0);
  stepper.setAcceleration(250);
  stepper.setMaxSpeed(rpmToStepsPerSecond(5));
  // Blocking until position set.
  stepper.runToPosition();
  Setpoint = 0;

  Wire.begin();

  // indefinitely stall if accelerometer isn't returning data?
  int attempt = 0;
  while (accel.begin() == false) {
    Serial.println("Not Connected. Please check connections and read the hookup guide.");
    Serial.print("Attempt: ");
    Serial.print(attempt++);
    delay(1000);
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
    // Update the stepper.
    //stepperAngle = map(Output, 90, -90, 0, 180);

    if (stepper.isRunning()) {
      // Stepper is still heading for last angle.
      // Stop it now and give new instructions!
      stepper.stop();
    }

    stepper.moveTo(angleInAbsoluteSteps(Output));
    stepper.setMaxSpeed(rpmToStepsPerSecond(5));
    stepper.setAcceleration(250);
  }
  stepper.run();
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

int angleInAbsoluteSteps(int angle) {
  return map(angle, -180, 180, -stepsPerRevolution / 2, stepsPerRevolution / 2);
}

float rpmToStepsPerSecond(int desiredRpm) {
  return desiredRpm * 60 * stepsPerRevolution;
}

void debugLog() {
  Serial.print("Kp=");
  Serial.print(Kp);
  Serial.print(",Ki=");
  Serial.print(Ki, 4);
  Serial.print(",Kd=");
  Serial.print(Kd, 4);
  Serial.print(",in=");
  Serial.print(Input);
  Serial.print(",out=");
  Serial.print(Output);

  Serial.println("");
}