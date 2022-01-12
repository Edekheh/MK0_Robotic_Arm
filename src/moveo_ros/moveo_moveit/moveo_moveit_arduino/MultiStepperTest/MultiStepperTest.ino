// MultiStepper.pde
// -*- mode: C++ -*-
// Use MultiStepper class to manage multiple steppers and make them all move to 
// the same position at the same time for linear 2d (or 3d) motion.

#include <AccelStepper.h>
#include <MultiStepper.h>

#define X_STEP_PIN 54
#define X_DIR_PIN 55
#define X_ENABLE_PIN 38

#define Y_STEP_PIN 60
#define Y_DIR_PIN 61
#define Y_ENABLE_PIN 56

#define Z_STEP_PIN 46
#define Z_DIR_PIN 48
#define Z_ENABLE_PIN 62

#define A_STEP_PIN 26
#define A_DIR_PIN 28
#define A_ENABLE_PIN 24

#define B_STEP_PIN 36
#define B_DIR_PIN 34
#define B_ENABLE_PIN 30

AccelStepper joint1(1,X_STEP_PIN, X_DIR_PIN);
AccelStepper joint2(1,Y_STEP_PIN, Y_DIR_PIN);
AccelStepper joint3(1,Z_STEP_PIN, Z_DIR_PIN);
AccelStepper joint4(1,A_STEP_PIN, A_DIR_PIN);
AccelStepper joint5(1,B_STEP_PIN, B_DIR_PIN);

// Up to 10 steppers can be handled as a group by MultiStepper
MultiStepper steppers;

//test with uint8 converted to long
unsigned int x = 1000;

void setup() {
  Serial.begin(250000);

  // Configure each stepper
  joint1.setMaxSpeed(1000);
  joint2.setMaxSpeed(1000);
  joint3.setMaxSpeed(1000);
  joint4.setMaxSpeed(1000);
  joint5.setMaxSpeed(1000);

  // Then give them to MultiStepper to manage
  steppers.addStepper(joint1);
  steppers.addStepper(joint2);
  steppers.addStepper(joint3);
  steppers.addStepper(joint4);
  steppers.addStepper(joint5);
  
  digitalWrite(9, 1); //fan led
  digitalWrite(X_ENABLE_PIN,0);
  analogWrite(Y_ENABLE_PIN,0);
  analogWrite(Z_ENABLE_PIN,0);
  analogWrite(A_ENABLE_PIN,0);
  analogWrite(B_ENABLE_PIN,0);
}

void loop() {
  long positions[5]; // Array of desired stepper positions

  // Back of the envelope calculation for microsteps/revolution, where positions[i] is the number of steps (or microsteps).
  positions[0] = 0; //4100 microsteps is 1/8 revolutions ----> 32800 microsteps/rev
  //32800/2 to 1/4 obrotu
  positions[1] = 0; //2000 is 40/360 revolutions ---> 18000 microsteps/rev
  //10050 to 1/4 obrotu
  positions[2] = 0; //4000 is 20/360 revolutions ---> 72000 microsteps/rev
  //7200
  positions[3] = 4000; //820 is 1/4 revolution (200steps/revolution * 16microsteps/step (since microstepping) ~= 32800 microsteps/rev)
  //4500 to 1/4 obrotu
  positions[4] = 0; //2000 is 50/360 revolution ---> 14400
  //5400 to 1/4 obrotu
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1);
  
  // Move to a different coordinate
  positions[0] = 0;
  positions[1] = 0;
  positions[2] = 0;
  positions[3] = 0;
  positions[4] = 0;
  steppers.moveTo(positions);
  steppers.runSpeedToPosition(); // Blocks until all are in position
  delay(1);
}
