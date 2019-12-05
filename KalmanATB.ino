#include <BasicLinearAlgebra.h>
#include <Wire.h>

const int RECEIVE_REGISTER_SIZE = 8;
const int SEND_REGISTER_SIZE = 8;
float receive_registers[RECEIVE_REGISTER_SIZE];
float send_registers[SEND_REGISTER_SIZE];
int current_send_register = 3;
int DO_KALMAN_UPDATE_COMMAND = 100;
int STRING_COMMAND = 10;
int UPDATE_SEND_REGISTER = 11;

float meas_x;
float meas_y;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Setup Completed");
  Wire.begin(0x8);
  meas_x = 0;
  meas_y = 0;
  Wire.onReceive(receiveEvent);
}

void loop() {
  static double lastMillis = millis();
  double delayTsec = 0.01;
  delay(1000*delayTsec);

  double Lc = 16; //length of car
  double V = 0.0; //speed of car (calibrated)
  double xp = 390; //pos of cone in world coordinate system
  double yp = 40; //pos of cone in world coordinate system
  
  static BLA::Matrix<3> x_true = {0, 0, 0.0};
  static BLA::Matrix<3> x_hat = {20, 10, 6.0}; // initial estimte of car's position
  static BLA::Matrix<3,3> P = {0,0,0,
                               0,0,0,
                               0,0,0};
  double TT = delayTsec*delayTsec;
  double QQ = 50.0;
  BLA::Matrix<3,3> Q = {QQ*TT,0,0,  //uncertainty of Dead Reckoning
                        0,QQ*TT,0,
                        0,0,0.00001*QQ*TT};
  double RR = 1000.0;
  BLA::Matrix<2,2> R = {RR*TT, 0,
                          0,  RR*TT};

  double dt = (millis() - lastMillis) / 1000.0;
  lastMillis = millis();


  //--------dead reckoning-----
  BLA::Matrix<3> x_hat_prime;
  x_hat_prime(0) = x_hat(0) + V * dt * cos(x_hat(2));
  x_hat_prime(1) = x_hat(1) + V * dt * sin(x_hat(2));
  x_hat_prime(2) = x_hat(2);
  
  //--------------calc expected Pos of cone------

  double expect_x = (xp - x_hat_prime(0)) * cos(x_hat_prime(2)) + (yp - x_hat_prime(1)) * sin(x_hat_prime(2)) - Lc;
  double expect_y = (yp - x_hat_prime(1)) * cos(x_hat_prime(2)) - (xp - x_hat_prime(0)) * sin(x_hat_prime(2));

  BLA::Matrix<2> z_hat_prime = {expect_x, expect_y};

// Update Measurement
  
  BLA::Matrix<2> z = {receive_registers[1], receive_registers[2]};
  
  
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
  BLA::Matrix<3> x_cor = K * (z - z_hat_prime);
  x_hat = x_hat_prime + x_cor;


//------------Calc new P---

  BLA::Matrix<3,3> I = {1,0,0,
                        0,1,0,
                        0,0,1};

  BLA::Matrix<3,3> Inter5 = I - (K * H);
  Multiply(Inter5,P_prime,P);
  
  Serial.print("Psi: "); Serial.print(x_hat(2), 2); Serial.print("\t");
  Serial.print("X: "); Serial.print(x_hat(0), 2); Serial.print("\t");
  Serial.print("Y: "); Serial.print(x_hat(1), 2); Serial.print("\t");
  Serial.print("Z_X: "); Serial.print(z(0), 2); Serial.print("\t");
  Serial.print("Z_X_Prime: "); Serial.print(z_hat_prime(0), 2); Serial.print("\t");
  Serial.print("Z_Y: "); Serial.print(z(1), 2); Serial.print("\t");
  Serial.print("Z_Y_Prime: "); Serial.print(z_hat_prime(1), 2); Serial.println("\t");
}

void receiveEvent(int howMany) {
  //Serial.println("Receiving...");
  String full_datastring = "";
  while (Wire.available()) {
     char c = Wire.read(); 
     full_datastring = full_datastring + c;
  }
  
  byte command = full_datastring.charAt(0);
  
  if(command >= 0 && command <= RECEIVE_REGISTER_SIZE)
  {
    float data = full_datastring.substring(1).toFloat();
    //Serial.print("Command: ");
    //Serial.print(command);
    //Serial.print(", Value: ");
    //Serial.println(data);
    receive_registers[command] = data;
  }
}
