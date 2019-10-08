// MegaIMUTest.ino
// BDK:ESE421:2019C
// Week 2 Lab Sketch -- Test IMU
//
// Need the Adafruit Sensor Library
//

#include <SPI.h>
#include <Adafruit_LSM9DS1.h>
#include <Adafruit_Sensor.h>

//
// IMU uses SPI -- here are the pins on the Mega
// (Pins 49 & 47 are user selection)
//
#define LSM9DS1_SCK 52  //BDK-mega
#define LSM9DS1_MISO 50 //BDK-mega
#define LSM9DS1_MOSI 51 //BDK-mega
#define LSM9DS1_XGCS 49 //BDK-mega
#define LSM9DS1_MCS 47 //BDK-mega

//
// tell sensor library which pins for accel & gyro data
//
Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(LSM9DS1_XGCS, LSM9DS1_MCS);

void setup() {
    Serial.begin(115200);
    Serial.println("MegaIMUTest");
  
//
// initialize gyro / mag / accel
//
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

//////////////////////////////////////////////////////////////////
void loop() {
//
//  get the gyro / accel / mag data
//
    lsm.read();  /* ask it to read in the data */
    sensors_event_t a, m, g, temp;
    lsm.getEvent(&a, &m, &g, &temp);
    //Serial.print("Accel X: "); Serial.print(a.acceleration.x); Serial.print(" m/s^2");
    //Serial.print("\tY: "); Serial.print(a.acceleration.y);     Serial.print(" m/s^2 ");
    //Serial.print("\tZ: "); Serial.print(a.acceleration.z);     Serial.println(" m/s^2 ");

    //Serial.print("Mag X: "); Serial.print(m.magnetic.x);   Serial.print(" gauss");
    //Serial.print("\tY: "); Serial.print(m.magnetic.y);     Serial.print(" gauss");
    //Serial.print("\tZ: "); Serial.print(m.magnetic.z);     Serial.println(" gauss");
//
//    Serial.print("Gyro X: "); Serial.print(g.gyro.x);   Serial.print(" dps");
//    Serial.print("\tY: "); Serial.print(g.gyro.y);      Serial.print(" dps");
//    Serial.print("\tZ: "); Serial.print(g.gyro.z);      Serial.println(" dps");

    
    double curr = g.gyro.z;
    
    // put your main code here, to run repeatedly:
    static double gyroxprev = curr; // Starting condition for Vout previous
    static double RC = 10; // RC of 1
    unsigned long deltaTime = 0.02; // delta time of 20 ms
    double filtered = gyroxprev + (deltaTime / RC) * (curr - gyroxprev); //Equation to get Vout 
    gyroxprev = curr; //Setting Vprev for next loop
  
    
    Serial.println(filtered);
    
    delay(20);
}
