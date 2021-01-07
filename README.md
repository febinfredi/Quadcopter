# Quad

Quadcopter

Note: This was my first quad project and was a learning curve for me and so the quad after completetion was unstable in flight and was very hard to control and crashed everytime within 15 to 20 seconds after takeoff. But this project was a challenge and was basically to increase my knowledge in drones which it did unquestionably.  

Control module:
Entire control module is DIY.
MPU6050 used for IMU which is not the best but still manageable.
Arduino Nano was used for microcontroller which was a disadvantage as the pwm pins are not fast enough.

Body:
The body is made of FR4 fiber board and wooden sticks as 4 arms.
This resulted in lot of vibrations which affected the stability of quad as IMU filter was not good enough to isolate these vibrations.

Stabilization:
PID loop was implemeted for flight stabilization.
P, I and D values are not optimal as even after many trial and error, I could not find the optimal values which made quad very wobbly.
MPU6050 had no magnetometer hence yaw control was not without error as gyro drift was always present

Microcontroller:
Arduino Nano

Sensors:
MPU6050 IMU,
 nrf24l01 pa + lna

Modules:
AMS1117 3.3V regulator, 
 Simonk 30A esc

Motors:
1400KV Brushless motors

Propellers:
9x4.5 propeller

Body:
FR4 Fiber board and 5mm thick wooden sticks

