
// Stepper Motors with Arduino - Controlling Bipolar & Unipolar stepper motors
// https://www.youtube.com/watch?v=0qwrnUeSpYQ&t=952s

/*
  Stepper M otor Demonstration 1
  Stepper-Demo1.ino
  Demonstrates 28BYJ-48 Unipolar Stepper with ULN2003 Driver
  Uses Arduino Stepper Library

  DroneBot Workshop 2018
  https://dronebotworkshop.com
*/

//Include the Arduino Stepper Library
#include <Stepper.h>

// Define Constants

// Number of steps per internal motor revolution 
const float STEPS_PER_REV = 32; 

//  Amount of Gear Reduction
const float GEAR_RED = 64;

// Number of steps per geared output rotation
const float STEPS_PER_OUT_REV = STEPS_PER_REV * GEAR_RED;

// Define Variables

// Number of Steps Required
int StepsRequired;

// Create Instance of Stepper Class
// Specify Pins used for motor coils
// The pins used are:
// Stepper Motor A: 8,9,10,11 
// Stepper Motor A: 4,5,6,7

// Connected to ULN2003 Motor Driver IN1, IN2, IN3, IN4 
// Motor A: Pin 8 --> IN1, Pin 9 --> IN2, PIN 10 --> IN3, Pin 11 --> IN4
// Motor B: Pin 4 --> IN1, Pin 5 --> IN2, PIN 6 --> IN3, Pin 7 --> IN4

// Pins entered in sequence IN1-IN3-IN2-IN4 for proper step sequencing
Stepper A_steppermotor(STEPS_PER_REV, 8, 10, 9, 11);
Stepper B_steppermotor(STEPS_PER_REV, 4, 6, 5, 7);

void setup()
{
// Nothing  (Stepper Library sets pins as outputs)
}

void loop()
{

   // Rotate CW 1/2 turn slowly
  StepsRequired  =  STEPS_PER_OUT_REV / 2; 
  
  A_steppermotor.setSpeed(200);
  B_steppermotor.setSpeed(200);    
   
  A_steppermotor.step(StepsRequired);
  B_steppermotor.step(StepsRequired);
  
  delay(500);
  
  // Rotate CCW full turn quickly
  StepsRequired  =  - STEPS_PER_OUT_REV; 
   
  A_steppermotor.setSpeed(1000);
  B_steppermotor.setSpeed(1000);
  
  A_steppermotor.step(StepsRequired);
  B_steppermotor.step(StepsRequired);
  
  delay(1000);

}
