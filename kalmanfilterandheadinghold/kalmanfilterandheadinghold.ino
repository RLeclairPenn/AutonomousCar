#include <Servo.h>
#include <math.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <BasicLinearAlgebra.h>

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


using namespace BLA;
// ------------------------------------------------------------------------------------------------
BLA::Matrix<3> x_hat_matrix = {0,0,0};



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
double PWM = 50;

//send data
const int RECEIVE_REGISTER_SIZE = 8;
const int SEND_REGISTER_SIZE = 8;
float receive_registers[RECEIVE_REGISTER_SIZE];
float send_registers[SEND_REGISTER_SIZE];
int current_send_register = 7;


//doing the kalman filter at the start of the process
const int DO_KALMAN_UPDATE_COMMAND = 100;
int STRING_COMMAND = 10;
int UPDATE_SEND_REGISTER = 11;


void setup() {
  // Enable Serial Communications
  Serial.begin(115200);


  // join i2c bus with address #8
  Wire.begin(0x8);  

  // tell the slave device what it should do when the master sends it data
  // receiveEvent needs to be a method with a void return type and a single parameter
  // of type int that indicates how many bytes are being written (this includes the command byte)        
  Wire.onReceive(receiveEvent); 

  // tell the slave device what it should do when the master asks for data. Since the master won't send
  // a command saying what data it wants, you will need to get creative in order to send the master
  // different kinds of data
  Wire.onRequest(sendData);

  
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
  ServoCenter_us = 800 + analogRead(A8) /2;
  steeringServo.attach(servoPin);
  double servoAngleDeg = 0;
  setServoAngle(servoAngleDeg);

 
  //Initialize IMU 
  if (!lsm.begin())
    {
      Serial.println("Oops ... unable to initialize the LSM9DS1. Check your wiring!");
      while (1);
    }
   Serial.println("Found LSM9DS1 9DOF");
    //
    // set ranges for sensor
    //
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
    motor_speed = 0;
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
  double currangle;
  heading += curr * dt;
  currangle = heading * 3.14159265 / 180;
  Serial.print(heading);
  Serial.print(";");
  Serial.println(micros() / 1000000.0);
  
  currX += v * dt * cos(currangle);
  currY += v * dt * sin(currangle);
  
  // Serial.print(currX);
  // Serial.print(";");
  // Serial.println(currY);
  // Setting up dt
  dt = ((micros() - previousTime) * 0.000001);
  previousTime = micros();
  
  setServoAngle(servoAngleDeg);
  //Serial.print("Servo Angle: ");
  //Serial.println(servoAngleDeg);
  x_hat_matrix = x_hat_matrix + {currX,currY,heading};
  
}

void kalman_update() {
  kalman_update();
  Serial.println("doing kalman update");
}


void kalman_update() {
  BLA::Matrix<3,3> Q_matrix = {1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0};
  BLA::Matrix<3,3> R_matrix = {1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0};
  BLA::Matrix<3,3> I_matrix = {1.0, 0.0, 0.0,
                               0.0, 1.0, 0.0,
                               0.0, 0.0, 1.0};//these should be changed due to the test.
  BLA::Matrix<3> x_k_matrix;
  BLA::Matrix<3> x_hat_prime_matrix;
  
  static BLA::Matrix<3> x_hat_last_matrix = {0.0,0.0,0.0};//may need to change depending on actual last coords
  BLA::Matrix<3> p_prime_matrix;
  BLA::Matrix<3> p_matrix;
  static BLA::Matrix<3> p_last_matrix= {1.0,1.0,1.0};
  static double t_last = 0.0;
  double dt = ((micros() - previousTime) * 0.000001);
  double v = -0.00002*PWM*PWM + 0.0083*PWM + 0.1836;
  BLA::Matrix<2> z_k_matrix = {0.0,0.0};//input from measurement
  double x_p;
  double y_p;
  double L_c;
  
  BLA::Matrix<3,3> A_k_matrix = {1.0, 0.0, dt*v*sin(x_hat_last_matrix(2)),
                                 0.0, 1.0, dt*v*sin(x_hat_last_matrix(2)),
                                 0.0, 0.0, 1.0};

  BLA::Matrix<3> x_hat_prime_update_matrix = ;
  x_hat_prime_matrix = x_hat_last_matrix + x_hat_prime_update_matrix; 
  BLA::Matrix<2,3> H_k_matrix = {-cos(x_hat_last_matrix(2)), -sin(x_hat_last_matrix(2), -(x_p - x_hat_prime_matrix(0))*sin(x_hat_prime_matrix(2)) + (y_p - x_hat_prime_matrix(1))*cos(x_hat_prime_matrix(2)),
                          -cos(x_hat_last_matrix(2)), sin(x_hat_last_matrix(2),-(y_p - x_hat_prime_matrix(1))*sin(x_hat_prime_matrix(2)) - (x_p - x_hat_prime_matrix(0))*cos(x_hat_prime_matrix(2))};

  BLA::Matrix<2> z_hat_prime = {(x_p - x_hat_prime_matrix(0))*cos(x_hat_prime_matrix(2)) + (y_p - x_hat_prime_matrix(1))*sin(x_hat_prime_matrix(2)) - L_c,(y_p - x_hat_prime_matrix(1))*cos(x_hat_prime_matrix(2)) - (x_p - x_hat_prime_matrix(0))*sin(x_hat_prime_matrix(2))};
  p_prime_matrix = A_k_matrix*p_last_matrix*Transpose(A_k_matrix) + Q_matrix;
  
  //update th primary calculation for p and x_hat
  
  double K_k_matrix = p_prime_matrix*Transpose(H_k_matrix)*Invert(H_k_matrix*p_prime_matrix*Transpose(H_k_matrix)+R_matrix);
  //Calculate the kalman filter
  x_hat_matrix = x_hat_prime_matrix + K_k_matrix*(z_k_matrix-z_hat_prime);
  p_matrix= (I_matrix-K_k_matrix*H_k_matrix)*p_prime_matrix;


 
  p_last_matrix =p_matrix;
  x_hat_last_matrix = x_hat_matrix;

 
  /*Serial.print(x_k_matrix); Serial.print("\t");
  Serial.print(x_hat_matrix); Serial.print("\t");
  Serial.print(K_k_matrix); Serial.print("\t");
  Serial.println(p_matrix);*/
  return x_hat_matrix; 
}

void receiveEvent(int howMany) {
  String full_datastring = "";
  
  while (Wire.available()) { // loops through all the bytes that are sent over the wire
    // read in the information on the wire one byte at a time.
    // c can be set to any one byte primitive type such as char, int, byte. This type will cause c to be treated differently
    char c = Wire.read(); 
    full_datastring = full_datastring + c;
  }
  // extract the command byte based on the command byte have different behavior
  byte command = full_datastring.charAt(0);

  // read in a string and interpret data as string
  if(command == STRING_COMMAND)
  {
    Serial.println("received data string");
    String data = full_datastring.substring(1);
    Serial.println(data);
  }

  // called before the Pi reads from a register in send register. Updates which register the
  // Pi will read from.
  if(command == UPDATE_SEND_REGISTER){
    int data = full_datastring.substring(1).toInt();
    current_send_register = data;
    Serial.println("updating send register");
    Serial.println(current_send_register);
  }

  // Pi can trigger commands by sending a command byte mapped to a command
  if(command == DO_KALMAN_UPDATE_COMMAND)
  {
    kalman_update();
  }

  // If the command is inside the default write register range then write to the write register
  if(command >= 0 && command <= RECEIVE_REGISTER_SIZE)
  {
    // received a float and therefore write to a register

    
    //Serial.println("Received Data Float. Writing to register " + String(command));
    float data = full_datastring.substring(1).toFloat();
    Serial.println(data);
    receive_registers[command] = data;
  }
   
  // Take care not to continue calling Wire.read() if there is nothing on the wire as you will get nonsense typically
}

// function that exectures whenever the master requests it. This function cannot have any parameters, and so
// you will need to take care in order to have the function write different types of information to the master
// one suggestion is to send a command from the master to the slave telling it what data you want and then having the Arduino write it
void sendData() {
  // send the data to the master
  char data[8];
  dtostrf(send_registers[current_send_register],8, 4, data);
  Serial.println(data);
  Wire.write(data);
 
}


void kalman_update() {
  kalman_update();
  Serial.println("doing kalman update");
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
