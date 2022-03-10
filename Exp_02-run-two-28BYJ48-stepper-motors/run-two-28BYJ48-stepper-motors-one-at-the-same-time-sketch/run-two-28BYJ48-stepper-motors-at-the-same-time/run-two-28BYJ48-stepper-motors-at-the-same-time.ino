
// Stepper Motors with Arduino - Controlling Bipolar & Unipolar stepper motors
// https://www.youtube.com/watch?v=0qwrnUeSpYQ&t=952s

// Lesson 9B - Driving two stepper motors at the same time - Arduino a Quick Start Guide
// ICT Tools for Teachers
// https://www.youtube.com/watch?v=nw8XaPs8qSs&t=730s




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



////////////////////////////

// DEFINE FUNCTIONS

////////////////////////////

void move_forward(int motor_speed) {

  // Set the speed
  A_steppermotor.setSpeed(motor_speed);
  B_steppermotor.setSpeed(motor_speed);    

  // Move only one step.
  // The motors will move one step each time the main loop is run.
  A_steppermotor.step(1);
  B_steppermotor.step(1);
  
}


void reverse(int motor_speed) {

  // Set the speed
  A_steppermotor.setSpeed(motor_speed);
  B_steppermotor.setSpeed(motor_speed);    

  // Move only one step.
  // The motors will move one step each time the main loop is run.
  // A negative number causes the motors to turn counter clockwise.
  A_steppermotor.step(-1);
  B_steppermotor.step(-1);
  
}

////////////////////////////
////////////////////////////



void setup() 
{
// Nothing  (Stepper Library sets pins as outputs)
}


void loop() 
{

  // Move forward (clockwise)
  // There's only one step each time the loop runs
  move_forward(1000);
  
  // Reverse (counter clockwise)
  //reverse(500);

}
