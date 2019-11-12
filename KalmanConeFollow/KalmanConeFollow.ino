
// This is the structure we will use to keep track of our car
struct Car {
  float x;
  float y;
  float psi;
};

Car ourCar = {0.0, 0.0, 0.0};

float ourMap[3][2] = {{0.0, 0.0}, {5.0, 0.0}, {0.0, 3.0}};

float xHatDot;
float yHatDot;
float psiHatDot;
float deltaTime;
float psi; // We have to find the psi from our imu.

void setup() {
  // put your setup code here, to run once:
  deltaTime = 0.02;
}

void loop() {
  // put your main code here, to run repeatedly:
  float psiHat = 0.0; // TO DO, CALCULATE THE ANGLE WE ARE HEADING AT
  float VHat = 0.3; // TO DO, FIGURE OUT OUR ACTUAL SPEED
  float L = 0.1; // TO DO, FIGURE OUT WHAT THIS IS
  float r_imu = 0.0; // TO DO, GET IT FROM THE IMU
  static float xHatKPrev = 0.0; // This will be updated at the end of the cycle
  static float psiPrev = psi;
  
  yHatDot = VHat * sin(psiHat);
  xHatDot = VHat * cos(psiHat);
  psiHatDot = r_imu;

  float xHatKPrime = xHatKPrev + delta * VHat * cos(psiPrev);
  
  psiPrev = psi;  
  xHatKPrev = xHatK;
  
  delay(20);
}
