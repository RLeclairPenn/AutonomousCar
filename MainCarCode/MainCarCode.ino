#include <Servo.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>

#define servoPin 7 // pin for servo signal

#define frontPingTrigPin 22 // ping sensor trigger pin (output from Arduino)
#define frontPingEchoPin 24 // ping sensor echo pin (input to Arduino)
#define frontPingGrndPin 26 // ping sensor ground pin (use digital pin as ground)

#define rightPingTrigPin 23 // ping sensor trigger pin (output from Arduino)
#define rightPingEchoPin 25 // ping sensor echo pin (input to Arduino)
#define rightPingGrndPin 27 // ping sensor ground pin (use digital pin as ground)

#define leftPingTrigPin 6 // ping sensor trigger pin (output from Arduino)
#define leftPingEchoPin 5 // ping sensor echo pin (input to Arduino)
#define leftPingGrndPin 4 // ping sensor ground pin (use digital pin as ground)

#define motorFwdPin 8 // HIGH for FWD; LOW for REV
#define motorRevPin 9 // LOW for FWD; HIGH for REV
#define motorLPWMPin 10 // Left Motor Speed Control
#define motorRPWMPin 11 // Right Motor Speed Control

float frontPingDistanceCM = 0.0;
float rightPingDistanceCM = 0.0;
float leftPingDistanceCM = 0.0;


// IMU setup ---------------------------------------------------------------------------------------
//PINS ON MEGA FOR THE IMU

#define LSM9DS1_SCK 52  //BDK-mega
#define LSM9DS1_MISO 50 //BDK-mega
#define LSM9DS1_MOSI 51 //BDK-mega
#define LSM9DS1_XGCS 49 //BDK-mega
#define LSM9DS1_MCS 47 //BDK-mega

// tell sensor library which pins for accel & gyro data
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

// ------------------------------------------------------------------------------------------------

static double error;
Servo steeringServo;
const long ping_timeout = 5000;
const double desiredDistance = 30.0; // 30 CM from wall
double servoAngleDeg = 0.0; //Steering angle delta
int motor_speed = 0;
double deltaI = 0.0;
double deltaD = 0.0;
double dt;
double Kp = 5;
double Ki = 0.1;
double Kd = 0.5;
static double DerivError;

void setup() {
  // Enable Serial Communications
  Serial.begin(115200);

  // Initialize Front Ping Sensor
  pinMode(frontPingGrndPin, OUTPUT); digitalWrite(frontPingGrndPin, LOW);
  pinMode(frontPingTrigPin, OUTPUT);
  pinMode(frontPingEchoPin, INPUT);

  // Initialize Right Ping Sensor
  pinMode(rightPingGrndPin, OUTPUT); digitalWrite(rightPingGrndPin, LOW);
  pinMode(rightPingTrigPin, OUTPUT);
  pinMode(rightPingEchoPin, INPUT);

   // Initialize Left Ping Sensor
  pinMode(leftPingGrndPin, OUTPUT); digitalWrite(leftPingGrndPin, LOW);
  pinMode(leftPingTrigPin, OUTPUT);
  pinMode(leftPingEchoPin, INPUT);

  // Initialize Motor PWM Pins
  pinMode(motorFwdPin, OUTPUT); digitalWrite(motorFwdPin, LOW);
  pinMode(motorRevPin, OUTPUT); digitalWrite(motorRevPin, LOW);
  pinMode(motorLPWMPin, OUTPUT);

  // Initialize Servo
  steeringServo.attach(servoPin);
  setServoAngle(servoAngleDeg);

  //Initialize IMU 
  if (!lsm.begin())
    {
      Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
      while (1);
    }
  // set ranges for sensor
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
}

void loop() {
  static float v = 1.0;
  static float currX = 0.0;
  static float currY = 0.0;
  // Setting up for previousTime
  unsigned static long previousTime = micros();
  // Setting up for heading 
  static double heading = 0;
  // this boolean value is to know whether to mantain heading hold or not
  bool hasAWall;
  
  //Getting the gyro data
  lsm.read();  /* ask it to read in the data */
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);  

  //Check for front collision
  if (frontPingDistanceCM <= 25) {
    motor_speed = 0;
  }
  else {
    motor_speed = 0; // 50 is the usual speed
  }
  
  //Move Forward
  moveMotor(motor_speed, true);

  // Gets right ping distance
  getRightPingDistanceCM();
  //Serial.print("Right Distance: ");
  //Serial.println(rightPingDistanceCM, DEC);

  // Gets left ping distance
  getLeftPingDistanceCM();
  //Serial.print("Left Distance: ");
  //Serial.println(leftPingDistanceCM, DEC);

  // Gets front ping distance
  getFrontPingDistanceCM();
  //Serial.print("Front Distance: ");
  //Serial.println(frontPingDistanceCM, DEC);
  //Serial.print("\n");

  //Heading
  double curr = g.gyro.z - 1.82;
  heading += curr * dt;
  correctAngle = heading * 3.14159265 / 180;
  currX += v * dt * cos(heading * 3.14159265 / 180);
  currY += v * dt * sin(heading * 3.14159265 / 180);
  
  // Serial.print(currX);
  // Serial.print(";");
  // Serial.println(currY);
  // Setting up dt
  dt = ((micros() - previousTime) * 0.000001);
  previousTime = micros();
  
  setServoAngle(servoAngleDeg);
  //Serial.print("Servo Angle: ");
  //Serial.println(servoAngleDeg);
  
  
}



void setServoAngle(double sDeg)
{
  //
  //  Update ServoCenter_us as Required for installation bias
  //  CAREFUL: make small increments (~100) to iterate
  //  100us is about 20deg (higher values --> more right steering)
  //  wrong ServoCenter values can damage servo
  //
  double ServoCenter_us = 1250;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                     00.0;
  double ServoScale_us = 8.0;    // micro-seconds per degree
  //
  //  NEVER send a servo command without constraining servo motion!
  //  -->large servo excursions could damage hardware or servo<--
  //
  double t_us = constrain(ServoCenter_us + ServoScale_us * sDeg, ServoCenter_us - 150, ServoCenter_us + 150);
  steeringServo.writeMicroseconds(t_us);
}

void getFrontPingDistanceCM()
{
  //
  // 3000 us timeout implies maximum distance is 51cm
  // but in practice, actual max larger?
  //
  const long timeout_us = ping_timeout;
  //
  // pingTrigPin = trigger pin
  // pingEchoPin = echo pin
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  //
  digitalWrite(frontPingTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(frontPingTrigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(frontPingTrigPin, LOW);
  //
  // The echo pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  //
  unsigned long echo_time;
  echo_time = pulseIn(frontPingEchoPin, HIGH, timeout_us);
  if (echo_time == 0)
  {
    echo_time = timeout_us;
  }
  //
  // return the distance in centimeters
  // distance = (10^-6) * (echo_time_us) * (speed of sound m/s) * (100 cm/m) / 2
  // divide by 2 because we measure "round trip" time for echo
  // (0.000001 * echo_time_us * 340.0 * 100.0 / 2.0)
  // = 0.017*echo_time
  //
  frontPingDistanceCM = constrain(0.017 * echo_time, 5.0, 50.0);
}

void getRightPingDistanceCM()
{
  //
  // 3000 us timeout implies maximum distance is 51cm
  // but in practice, actual max larger?
  //
  const long timeout_us = ping_timeout;
  //
  // pingTrigPin = trigger pin
  // pingEchoPin = echo pin
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  //
  digitalWrite(rightPingTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(rightPingTrigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(rightPingTrigPin, LOW);
  //
  // The echo pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  //
  unsigned long echo_time;
  echo_time = pulseIn(rightPingEchoPin, HIGH, timeout_us);
  if (echo_time == 0)
  {
    echo_time = timeout_us;
  }
  //
  // return the distance in centimeters
  // distance = (10^-6) * (echo_time_us) * (speed of sound m/s) * (100 cm/m) / 2
  // divide by 2 because we measure "round trip" time for echo
  // (0.000001 * echo_time_us * 340.0 * 100.0 / 2.0)
  // = 0.017*echo_time
  //
  rightPingDistanceCM = constrain(0.017 * echo_time, 5.0, 50.0);
}

void getLeftPingDistanceCM()
{
  //
  // 3000 us timeout implies maximum distance is 51cm
  // but in practice, actual max larger?
  //
  const long timeout_us = ping_timeout;
  //
  // pingTrigPin = trigger pin
  // pingEchoPin = echo pin
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  //
  digitalWrite(leftPingTrigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(leftPingTrigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(leftPingTrigPin, LOW);
  //
  // The echo pin is used to read the signal from the PING))): a HIGH
  // pulse whose duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  //
  unsigned long echo_time;
  echo_time = pulseIn(leftPingEchoPin, HIGH, timeout_us);
  if (echo_time == 0)
  {
    echo_time = timeout_us;
  }
  //
  // return the distance in centimeters
  // distance = (10^-6) * (echo_time_us) * (speed of sound m/s) * (100 cm/m) / 2
  // divide by 2 because we measure "round trip" time for echo
  // (0.000001 * echo_time_us * 340.0 * 100.0 / 2.0)
  // = 0.017*echo_time
  //
  leftPingDistanceCM = constrain(0.017 * echo_time, 5.0, 50.0);
}

//Motor Function take in speed integer from 0-100
// direction = True for Forward, False for Backward
void moveMotor(int motor_speed, bool direction) {
  //Move Forward
  if (direction) {
    digitalWrite(motorFwdPin, HIGH);
    digitalWrite(motorRevPin, LOW);
  }
  else {
    digitalWrite(motorFwdPin, LOW);
    digitalWrite(motorRevPin, HIGH);
  }
byte motorPWM = map(motor_speed, 0, 100, 0, 255);
  analogWrite(motorLPWMPin, motorPWM);
  analogWrite(motorRPWMPin, motorPWM);
}
