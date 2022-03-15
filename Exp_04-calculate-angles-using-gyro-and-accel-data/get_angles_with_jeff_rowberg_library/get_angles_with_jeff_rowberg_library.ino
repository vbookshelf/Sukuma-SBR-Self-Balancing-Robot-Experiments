#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

MPU6050 mpu;

long raw_acc_x, raw_acc_y, raw_acc_z;
int raw_temp;
long raw_gyro_y, raw_gyro_z;

long loop_timer;
float angle_pitch, angle_roll;
long raw_gyro_x = 0.0;

void print_raw_data() {

  Serial.print("raw_acc_x: ");
  Serial.print(raw_acc_x);

  Serial.print(" raw_acc_y: ");
  Serial.print(raw_acc_y);

  Serial.print(" raw_acc_z: ");
  Serial.print(raw_acc_z);

  Serial.print(" raw_gyro_x: ");
  Serial.print(raw_gyro_x);

  Serial.print(" raw_gyro_y: ");
  Serial.print(raw_gyro_y);

  Serial.print(" raw_gyro_z: ");
  Serial.println(raw_gyro_z);
  
}





void setup() {  
  mpu.initialize();
  Serial.begin(57600);

  // Initialize the timer
  loop_timer = micros();
}

void loop() {  
  
  // read acceleration and gyroscope values
  raw_acc_x = mpu.getAccelerationX();
  raw_acc_y = mpu.getAccelerationY();
  raw_acc_z = mpu.getAccelerationZ();

  raw_gyro_x = mpu.getRotationX();
  raw_gyro_y = mpu.getRotationY();
  raw_gyro_z = mpu.getRotationZ();

  //print_raw_data();

  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  angle_pitch += raw_gyro_x * 0.0000611;                                   //Calculate the traveled pitch angle and add this to the angle_pitch variable
  angle_roll += raw_gyro_y * 0.0000611; 

  Serial.print("pitch_angle: ");
  Serial.println(angle_pitch);

  //Wait until the loop_timer reaches 4000us (250Hz) before starting the next loop
  while(micros() - loop_timer < 4000); 
  
  //Reset the loop timer                                
  loop_timer = micros();

  
}
