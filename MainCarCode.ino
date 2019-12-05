#include <Servo.h>
#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>
#include <BasicLinearAlgebra.h>
#include <Wire.h>
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
#define motorRPWMPin 11 // Righ、t Motor Speed Control

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
int motor_speed = 50; //
double deltaI = 0.0;
double deltaD = 0.0;
double dt;
double Kp = 5;
double Ki = 0.1;
double Kd = 0.5;
static double DerivError;

// The below is to do arduino communication
const int RECEIVE_REGISTER_SIZE = 8;
const int SEND_REGISTER_SIZE = 8;
float receive_registers[RECEIVE_REGISTER_SIZE];
float send_registers[SEND_REGISTER_SIZE];
int current_send_register = 3;
int DO_KALMAN_UPDATE_COMMAND = 100;
int STRING_COMMAND = 10;
int UPDATE_SEND_REGISTER = 11;
int hasReceivedPING = 0;

void setup() {
  // Enable Serial Communications
  Serial.begin(9600);

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
}

// function that executes whenever data is received from master
// this function is registered as an event, see setup()
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
//    Serial.println("received data string");
//    String data = full_datastring.substring(1);
//    Serial.println(data);
  }

  // called before the Pi reads from a register in send register. Updates which register the
  // Pi will read from.
  if(command == UPDATE_SEND_REGISTER){
    int data = full_datastring.substring(1).toInt();
    current_send_register = data;
//    Serial.println("updating send register");
//    Serial.println(current_send_register);
  }

  // Pi can trigger commands by sending a command byte mapped to a command
  if(command == DO_KALMAN_UPDATE_COMMAND)
  {
    hasReceivedPING = 1;
  }
  else {
    hasReceivedPING = 0;
  }

  // If the command is inside the default write register range then write to the write register
  if(command >= 0 && command <= RECEIVE_REGISTER_SIZE)
  {
    // received a float and therefore write to a register
//    Serial.println("Received Data Float. Writing to register " + String(command));
    float data = full_datastring.substring(1).toFloat();
//    Serial.println(data);
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
//  Serial.println(data);
  Wire.write(data);
 
}


void loop() {
  static float currX = 0.0; // input this into the matrix
  static float currY = 0.0; // input this into the matrix
  static double heading = 0.0; // input this into the matrix
  
  // Setting up for previousTime
  static double previousTime = millis();
  double delayTsec = 0.01;
  delay(1000*delayTsec);

  // Necessary Stuff for the Kalman Filter
  double Lc = 16; //length of car
  double V = 50.0; //speed of car (calibrated)
  double xp = 1000; //pos of cone in world coordinate system
  double yp = 100; //pos of cone in world coordinate system
    
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
  
  getRightPingDistanceCM();// Gets right ping distance
  getLeftPingDistanceCM();// Gets left ping distance
  getFrontPingDistanceCM();// Gets front ping distance
  
  //Heading
  double curr = g.gyro.z - 1.82;
  heading += curr * dt;
  double correctAngle = heading * 3.14159265 / 180;
  
  static BLA::Matrix<3> x_hat = {currX, currY, correctAngle}; // initial estimte of car's position, world coordinate
  
  static BLA::Matrix<3,3> P = {0,0,0,
                               0,0,0,
                               0,0,0}; // we know exactly where we start
  double TT = delayTsec*delayTsec;
  double QQ = 100.0; // increasing means less certain of DR
  BLA::Matrix<3,3> Q = {QQ*TT,0,0,  //uncertainty of Dead Reckoning
                        0,QQ*TT,0,
                        0,0,0.00001*QQ*TT};
  double RR = 1000.0; // increasing this means less certainty of Measure.
  BLA::Matrix<2,2> R = {RR*TT, 0, // uncertainty of camera measurements
                          0,  RR*TT};
                          
  dt = (millis() - previousTime) / 1000.0;
  previousTime = millis();

  

  //--------dead reckoning-----
  BLA::Matrix<3> x_hat_prime;
  x_hat_prime(0) = x_hat(0) + V * dt * cos(x_hat(2));
  x_hat_prime(1) = x_hat(1) + V * dt * sin(x_hat(2));
  x_hat_prime(2) = x_hat(2);

  x_hat(0) = x_hat_prime(0);
  x_hat(1) = x_hat_prime(1);
  x_hat(2) = correctAngle;
  // end of dead reckoning

  if (hasReceivedPING==1) {
   float received = receive_registers[7];
    
//    Serial.print("What we received the location of the x y");
//    Serial.println(received);
  }
  
  if (hasReceivedPING==0) {
    float cxp = receive_registers[7];
    float cyp = receive_registers[6];
    //SHOULD ONLY BE DONE WHEN THERE ARE MEASUREMENTS --------------------------------------------------------------------------------------------------
    //TODO: Link the register things to these two measurements
    double meas_x_t = cxp; // camera coordinates
    double meas_y_t = cyp;
    BLA::Matrix<2> z= {meas_x_t, meas_y_t}; // input our measurements
  
    if (meas_x_t < 10.0) {
      moveMotor(0, true);
      //while(1);
    }
    //--------------calc expected Pos of cone------
  
    double expect_x = (xp - x_hat_prime(0)) * cos(x_hat_prime(2)) + (yp - x_hat_prime(1)) * sin(x_hat_prime(2)) - Lc; // camera coordinates 
    double expect_y = (yp - x_hat_prime(1)) * cos(x_hat_prime(2)) - (xp - x_hat_prime(0)) * sin(x_hat_prime(2));
  
    BLA::Matrix<2> z_hat_prime = {expect_x, expect_y};
  
    
  //-----------Calc P_prime-----
  
    BLA::Matrix<3,3> A = {1, 0, -V * dt * sin(x_hat_prime(2)),
                            0, 1, V * dt * cos(x_hat_prime(2)),
                            0, 0, 1};
  
    BLA::Matrix<3,3> P_prime;
  
    BLA::Matrix<3,3> Inter = A * P;
  
    Multiply(Inter,~A,P_prime);
  
    P_prime += Q;
  
  //--------Calc K-----
    BLA::Matrix<2,3> H = {-cos(x_hat_prime(2)), -sin(x_hat_prime(2)), expect_y,
                           sin(x_hat_prime(2)), -cos(x_hat_prime(2)), -(expect_x+Lc)};
  
    BLA::Matrix<2,3> Inter2 = H * P_prime;
  
    BLA::Matrix<2,2> Inter3 = Inter2 * ~H + R;
  
    BLA::Matrix<2,2> Inter3_I = Inter3.Inverse();
  
    BLA::Matrix<3,2> Inter4 = ~H * Inter3_I;
  
    BLA::Matrix<3,2> K = P_prime * Inter4;
  
  //-------------Correct Position---------
    BLA::Matrix<3> x_cor;
    Multiply(K,(z - z_hat_prime),x_cor);
    x_hat = x_hat_prime + x_cor;
   
  
  //------------Calc new P---
  
    BLA::Matrix<3,3> I = {1,0,0,
                          0,1,0,
                          0,0,1};
  
    BLA::Matrix<3,3> Inter5 = I - (K * H);
    Multiply(Inter5,P_prime,P);
  //Serial.print("Psi: "); Serial.print(x_hat(2), 2); Serial.println("\t");
  Serial.print("X: "); Serial.print(x_hat(0), 2); Serial.println("\t");
  //Serial.print("Y: "); Serial.print(x_hat(1), 2); Serial.println("\t");
    //-----------------------------------------------------------------------------------------------------------------------
  }
  
  setServoAngle(servoAngleDeg); // This is to set the angle
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
