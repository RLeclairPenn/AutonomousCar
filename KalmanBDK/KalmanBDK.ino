#include <BasicLinearAlgebra.h>

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Setup Completed");
}

void loop() {
  static double lastMillis = millis();
  double delayTsec = 0.01;
  delay(1000*delayTsec);

  double Lc = 16; //length of car
  double V = 50.0; //speed of car (calibrated)
  double xp = 1000; //pos of cone in world coordinate system
  double yp = 100; //pos of cone in world coordinate system
  
  static BLA::Matrix<3> x_true = {0, 0, 0.0}; // This is the true starting point
  static BLA::Matrix<3> x_hat = {-100, 25, 0.2}; // initial estimte of car's position, world coordinate
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

  double dt = (millis() - lastMillis) / 1000.0;
  lastMillis = millis();

  //--------true position update & true measurement-----
  x_true(0) += V * dt * cos(x_true(2));
  x_true(1) += V * dt * sin(x_true(2));
  
  double meas_x_t = (xp - x_true(0)) * cos(x_true(2)) + (yp - x_true(1)) * sin(x_true(2)) - Lc; // camera coordinates
  double meas_y_t = (yp - x_true(1)) * cos(x_true(2)) - (xp - x_true(0)) * sin(x_true(2));

  if (meas_x_t < 10) { while(1); }
  
  BLA::Matrix<2> z= {meas_x_t, meas_y_t}; // input our measurements

  //--------dead reckoning-----
  BLA::Matrix<3> x_hat_prime;
  x_hat_prime(0) = x_hat(0) + V * dt * cos(x_hat(2));
  x_hat_prime(1) = x_hat(1) + V * dt * sin(x_hat(2));
  x_hat_prime(2) = x_hat(2);

  x_hat(0) = x_hat_prime(0);
  
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
  

  Serial.print(x_true(2)); Serial.print(";");
  Serial.print(x_hat(2)); Serial.print(";");
  Serial.print(x_true(0)); Serial.print(";");
  Serial.print(x_hat(0)); Serial.print(";");
  Serial.print(x_true(1)); Serial.print(";");
  Serial.print(x_hat(1)); Serial.print(";");
  Serial.print(z(0)); Serial.print(";");
  Serial.print(z_hat_prime(0)); Serial.print(";");
  Serial.print(z(1)); Serial.print(";");
  Serial.print(z_hat_prime(1)); Serial.println("");
}
