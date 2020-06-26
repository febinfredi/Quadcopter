#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Wire.h>
#include <Servo.h>

RF24 radio(8, 7); // CE, CSN

const byte address[6] = "00001";  // nrf communication address

//Declaring servo objects
Servo motorFL;  //front left motor
Servo motorFR;  //front right motor
Servo motorBL;  //back left motor
Servo motorBR;  //back right motor

int motorFL_pin = 5;  // pins for pwm controlling of motor speed
int motorFR_pin = 3;
int motorBL_pin = 6;
int motorBR_pin = 9;

// variables for IMU
int c = 0;
float accx, accy, accz, acc_x, acc_y, acc_error_x, acc_error_y;
float gyrox, gyroy, gyroz, gyro_angle_x, gyro_angle_y, gyro_angle_z, gyro_error_x, gyro_error_y, gyro_error_z;
float previousTime, currentTime, elapsedTime, prev_time, current_time, elapsed_time;
float roll_mpu, pitch_mpu, yaw_mpu, roll_error , pitch_error;
float throttle_val, roll_tx, pitch_tx;

// variables for PID
float roll_p, roll_i, roll_d, pitch_p, pitch_i, pitch_d, roll_pid, pitch_pid, roll_prev_error, pitch_prev_error;
float roll_kp = 1.8, roll_ki = 0.0, roll_kd = 1.55;
float pitch_kp = 8.0, pitch_ki = 0.0, pitch_kd = 0.0;

struct data_pack {     // structure for storing data from transmitter
  byte throttle;
  byte xval;
  byte yval;
  byte sw;
};

data_pack data;     // structure object "data"


void init_mpu() {
  Wire.begin();
  Wire.beginTransmission(0x68);      //mpu I2C address is hex68 when AD0 is LOW according to datasheet
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  //reset
  Wire.endTransmission(true);
  // Configure Accelerometer
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);                  //ACCEl_CONFIG address
  Wire.write(0x10);                  //+-8g full scale
  Wire.endTransmission(true);
  // Configure Gyro
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);                  //GYRO_CONFIG address
  Wire.write(0x10);                  //1000dps full scale
  Wire.endTransmission(true);
}


void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error.
  // Read accelerometer values 200 times
  while (c < 200) {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);
    accx = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    accy = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    accz = (Wire.read() << 8 | Wire.read()) / 4096.0 ;
    // Sum all readings
    acc_error_x = acc_error_x + ((atan((accy) / sqrt(pow((accx), 2) + pow((accz), 2))) * 180 / PI));
    acc_error_y = acc_error_y + ((atan(-1 * (accx) / sqrt(pow((accy), 2) + pow((accz), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  acc_error_x = acc_error_x / 200;
  acc_error_y = acc_error_y / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(0x68);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6, true);
    gyrox = Wire.read() << 8 | Wire.read();
    gyroy = Wire.read() << 8 | Wire.read();
    gyroz = Wire.read() << 8 | Wire.read();
    // Sum all readings
    gyro_error_x = gyro_error_x + (gyrox / 32.8);
    gyro_error_y = gyro_error_y + (gyroy / 32.8);
    gyro_error_z = gyro_error_z + (gyroz / 32.8);
    c++;
  }
  //Divide the sum by 200 to get the error value
  gyro_error_x = gyro_error_x / 200;
  gyro_error_y = gyro_error_y / 200;
  gyro_error_z = gyro_error_z / 200;
  // Print the error values on the Serial Monitor
  /*Serial.print("AccErrorX: ");
    Serial.println(acc_error_x);
    Serial.print("AccErrorY: ");
    Serial.println(acc_error_y);
    Serial.print("GyroErrorX: ");
    Serial.println(gyro_error_x);
    Serial.print("GyroErrorY: ");
    Serial.println(gyro_error_y);
    Serial.print("GyroErrorZ: ");
    Serial.println(gyro_error_z);
    Serial.println(" ");
    Serial.println(" ");*/
}


void read_mpu() {
  //Accelerometer data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); //start from ACCEL_XOUT register
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true); //start reading upto 6 registers (3B to 40)

  accx = (Wire.read() << 8 | Wire.read()) / 4096.0;  //X axis value; for +-8g divide raw value by 4096 according to datasheet
  accy = (Wire.read() << 8 | Wire.read()) / 4096.0;  //Y axis value
  accz = (Wire.read() << 8 | Wire.read()) / 4096.0;  //Z axis value
  acc_x = (atan(accy / sqrt(pow(accx, 2) + pow(accz, 2))) * 180 / PI) - acc_error_x;  // 180/PI for rad to deg
  acc_y = (atan(-1 * accx / sqrt(pow(accy, 2) + pow(accz, 2))) * 180 / PI) - acc_error_y;

  //Gyro data
  previousTime = currentTime;
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000;

  Wire.beginTransmission(0x68);
  Wire.write(0x43);  //start from GYRO_XOUT register
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true); //start reading upto 6 registers (43 to  48)

  gyrox = ((Wire.read() << 8 | Wire.read()) / 32.8);
  gyroy = ((Wire.read() << 8 | Wire.read()) / 32.8);
  gyroz = ((Wire.read() << 8 | Wire.read()) / 32.8);
  gyrox = gyrox - gyro_error_x;
  gyroy = gyroy - gyro_error_y;
  gyroz = gyroz - gyro_error_z;
  gyro_angle_x = (gyrox * elapsedTime);
  gyro_angle_y = (gyroy * elapsedTime);
  gyro_angle_z = (gyroz * elapsedTime);

  //Complimentary filter
  roll_mpu = 0.98 * (roll_mpu + gyro_angle_x) + 0.02 * (acc_x);
  pitch_mpu = 0.98 * (pitch_mpu + gyro_angle_y) + 0.02 * (acc_y);
  yaw_mpu = gyro_angle_z + yaw_mpu;
}


void setup() {

  //Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address); // 00001
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
  init_mpu();
  calculate_IMU_error();
  pinMode(A0, INPUT);

  /*pinMode(motorFL_pin, OUTPUT);
    pinMode(motorFR_pin, OUTPUT);
    pinMode(motorBL_pin, OUTPUT);
    pinMode(motorBR_pin, OUTPUT);*/

  motorFL.attach(motorFL_pin, 1000, 2000);
  motorFR.attach(motorFR_pin, 1000, 2000);
  motorBL.attach(motorBL_pin, 1000, 2000);
  motorBR.attach(motorBR_pin, 1000, 2000);

  motorFR.writeMicroseconds(1000);
  motorFL.writeMicroseconds(1000);
  motorBR.writeMicroseconds(1000);
  motorBL.writeMicroseconds(1000);
}


void loop() {
  if (radio.available() > 0) {
    radio.read(&data, sizeof(data_pack));  //read data from tx

    /*throttle_val = int(data.throttle);
      if (throttle_val < 127) {
      throttle_val = 127;
      }
      if (throttle_val > 216) {
      throttle_val = 216;
      }*/

    throttle_val = map(int(data.throttle), 0, 255, 1000, 1700);  // map from 1000microsecs-1700microsecs pulse for pwm control of motor, 300microsecs reserved for pid control
    roll_tx = map(int(data.xval), 0, 255, 15, -15);         // map for roll from -10 degrees to 10 degrees
    pitch_tx = map(int(data.yval), 0, 255, -15, 15);        // map for pitch from -10 degrees to 10 degrees

    if (roll_tx == 1) {   //compensating for roll zero error when stick is in middle position
      roll_tx = 0;
    }
    if (pitch_tx == -1) {   //compensating for pitch zero error when stick is in middle position
      pitch_tx = 0;
    }
    //Serial.println("Throttle: " + String((throttle_val)));
    //Serial.print(" Roll_tx: " + String(roll_tx));
    //Serial.print(" Pitch_tx: " + String(pitch_tx));
    //Serial.println(" SW: " + String(data.sw));
  }

  if (data.sw == 0) {
    read_mpu();  //read data from mpu
    /*Serial.print("Roll_mpu: ");
      Serial.print(roll_mpu);
      Serial.print("    Pitch_mpu:  ");
      Serial.print(pitch_mpu);
      Serial.print("    Yaw_mpu:  ");
      Serial.println(yaw_mpu);*/

    // PID control
    prev_time = current_time;   // calculation of time duration for PID measurements
    current_time = millis();
    elapsed_time = (current_time - prev_time) / 1000;

    roll_error = roll_mpu - roll_tx;    // roll error = [output value(roll angle from mpu) - desired value(roll angle from transmitter)]
    pitch_error = pitch_mpu - pitch_tx; // roll error = [output value(pitch angle from mpu) - desired value(pitch angle from transmitter)]

    // Proportional control: p_out = p_const * error
    roll_p = roll_kp * roll_error;
    pitch_p = pitch_kp * pitch_error;

    // Integral control: i_out = integral(i_const * error)dt
    roll_i += roll_ki * roll_error;
    pitch_i += pitch_ki * pitch_error;

    // Derivative control: d_out = d_const*(d(error)/dt)
    roll_d = roll_kd * ((roll_error - roll_prev_error) / elapsed_time);
    pitch_d = pitch_kd * ((pitch_error - pitch_prev_error) / elapsed_time);

    roll_prev_error = roll_error; // current error will be previous error for next cycle
    pitch_prev_error = pitch_error;

    // pid_error = p_out + i_out + d_out
    roll_pid = roll_p + roll_i + roll_d;
    pitch_pid = pitch_p + pitch_i + pitch_d;
    //Serial.println("Roll PID: " + String(roll_pid));
    //Serial.println("Pitch PID: " + String(pitch_pid));

    // limiting pid_error limit to 150 so as to not go to very high error value
    if (roll_pid < -300) { // limiting roll_pid
      roll_pid = -300;
    }
    if (roll_pid > 300) {
      roll_pid = 300;
    }

    if (pitch_pid < -300) {  // limiting pitch_pid
      pitch_pid = -300;
    }
    if (pitch_pid > 300) {
      pitch_pid = 300;
    }

    pitch_pid = 0;

    if (throttle_val > 1020) {

      motorFR.writeMicroseconds(throttle_val - roll_pid - pitch_pid);
      motorBL.writeMicroseconds(throttle_val + roll_pid + pitch_pid);
      motorFL.writeMicroseconds(throttle_val + roll_pid - pitch_pid);
      motorBR.writeMicroseconds(throttle_val - roll_pid + pitch_pid);
    }

    else {
      motorFR.writeMicroseconds(1000);
      motorFL.writeMicroseconds(1000);
      motorBR.writeMicroseconds(1000);
      motorBL.writeMicroseconds(1000);
    }

    /*analogWrite(motorFR_pin, throttle_val - roll_pid - pitch_pid);
      analogWrite(motorFL_pin, throttle_val + roll_pid - pitch_pid);
      analogWrite(motorBR_pin, throttle_val - roll_pid + pitch_pid);
      analogWrite(motorBL_pin, throttle_val + roll_pid + pitch_pid);*/
  }
}
