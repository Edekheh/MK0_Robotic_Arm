/* Purpose: This sketch uses ROS as well as MultiStepper, AccelStepper, and Servo libraries to control the 
 * BCN3D Moveo robotic arm. In this setup, a Ramps 1.4 shield is used on top of an Arduino Mega 2560.  
 * Subscribing to the following ROS topics: 1) joint_steps, 2) gripper_angle
 *    1) joint_steps is computed from the simulation in PC and sent Arduino via rosserial.  It contains
 *       the steps (relative to the starting position) necessary for each motor to move to reach the goal position.
 *    2) gripper_angle contains the necessary gripper angle to grasp the object when the goal state is reached 
 * 
 * Publishing to the following ROS topics: joint_steps_feedback
 *    1) joint_steps_feedback is a topic used for debugging to make sure the Arduino is receiving the joint_steps data
 *       accurately
 *       
 * Author: Jesse Weisberg
 */
#if (ARDUINO >= 100)
  #include <Arduino.h>
#else
  #include <WProgram.h>
#endif
#include <ros.h>

#include <moveo_moveit/ArmJointState.h>
#include <Servo.h> 
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <math.h>
#include <std_msgs/Int16.h>
#include <std_msgs/UInt16.h>
#include <AccelStepper.h>
#include <MultiStepper.h>

#define Analog_Speed_Pin 66
#define Joint_Max_Speed 2000
#define Joint_Min_Speed 200

//converter pins
#define Work_Indicator_Pin 44
#define Holding_Indicator_Pin 64

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

Servo gripper;
MultiStepper steppers;

int joint_step[6];
int joint_status = 0;
unsigned int analog_read_velocity = 0;

ros::NodeHandle nh;
std_msgs::Int16 msg;

//instantiate publisher (for debugging purposes)
//ros::Publisher steps("joint_steps_feedback",&msg);

void arm_cb(const moveo_moveit::ArmJointState& arm_steps){
  joint_status = 1;
  joint_step[0] = arm_steps.position1;
  joint_step[1] = arm_steps.position2;
  joint_step[2] = arm_steps.position3;
  joint_step[3] = arm_steps.position4;
  joint_step[4] = arm_steps.position5;
  joint_step[5] = arm_steps.position6; //gripper position <0-180>
}

void gripper_cb( const std_msgs::UInt16& cmd_msg){
  gripper.write(cmd_msg.data); // Set servo angle, should be from 0-180  
  //digitalWrite(13, HIGH-digitalRead(13));  // Toggle led  
}

//instantiate subscribers
ros::Subscriber<moveo_moveit::ArmJointState> arm_sub("joint_steps",arm_cb); //subscribes to joint_steps on arm
ros::Subscriber<std_msgs::UInt16> gripper_sub("gripper_angle", gripper_cb); //subscribes to gripper position
//to publish from terminal: rostopic pub gripper_angle std_msgs/UInt16 <0-180>

void setup() {
  //put your setup code here, to run once:
  //Serial.begin(57600);
  pinMode(13,OUTPUT);
  joint_status = 1;

  nh.initNode();
  nh.subscribe(arm_sub);
  nh.subscribe(gripper_sub);
  //nh.advertise(steps);

  // Configure each stepper
  joint1.setMaxSpeed(1500);
  joint2.setMaxSpeed(1500);
  joint3.setMaxSpeed(1500);
  joint4.setMaxSpeed(1500);
  joint5.setMaxSpeed(1500);
  joint1.setAcceleration(0.5);
  joint1.setAcceleration(0.5);
  joint1.setAcceleration(0.5);
  joint1.setAcceleration(0.5);
  joint1.setAcceleration(0.5);

  pinMode(Analog_Speed_Pin,INPUT);
  
  // Then give them to MultiStepper to manage
  steppers.addStepper(joint1);
  steppers.addStepper(joint2);
  steppers.addStepper(joint3);
  steppers.addStepper(joint4);
  steppers.addStepper(joint5);

  // Configure gripper servo
  gripper.attach(11);
  
  digitalWrite(9, 1); //fan led
  digitalWrite(X_ENABLE_PIN,0);
  analogWrite(Y_ENABLE_PIN,0);
  analogWrite(Z_ENABLE_PIN,0);
  analogWrite(A_ENABLE_PIN,0);
  analogWrite(B_ENABLE_PIN,0);

}

void loop() {
  if (joint_status == 1) // If command callback (arm_cb) is being called, execute stepper command
  { 
    long positions[5];  // Array of desired stepper positions must be long
    positions[0] = joint_step[0]; // negated since the real robot rotates in the opposite direction as ROS
    positions[1] = -joint_step[1]; 
    positions[2] = -joint_step[2]; 
    positions[3] = joint_step[3]; 
    positions[4] = joint_step[4]; 
    digitalWrite(Work_Indicator_Pin,1);
    digitalWrite(Holding_Indicator_Pin,0);
    // Publish back to ros to check if everything's correct
    //msg.data=positions[4];
    //steps.publish(&msg);

    steppers.moveTo(positions);
    nh.spinOnce();
    steppers.runSpeedToPosition(); // Blocks until all are in position
    gripper.write(joint_step[5]);  // move gripper after manipulator reaches goal   
  }
  //digitalWrite(13, HIGH-digitalRead(13)); //toggle led
  joint_status = 0;
  digitalWrite(Work_Indicator_Pin,0);
  digitalWrite(Holding_Indicator_Pin,1);
  nh.spinOnce();
  delay(1);
  //changeJointsSpeeds();
}

bool if_speed_changed() {
  if(analogRead(Analog_Speed_Pin) > analog_read_velocity + 10 || analogRead(Analog_Speed_Pin) < analog_read_velocity - 10)  {
    analog_read_velocity = analogRead(Analog_Speed_Pin);
    return 1;
  }
  return 0;
}
void changeJointsSpeeds() {
    if(if_speed_changed())  {
    unsigned int setJointsSpeed = 1.5*analog_read_velocity;
    if(setJointsSpeed>Joint_Max_Speed) setJointsSpeed = Joint_Max_Speed;
    if(setJointsSpeed<Joint_Min_Speed) setJointsSpeed = Joint_Min_Speed;
    
  joint1.setMaxSpeed(setJointsSpeed);
  joint2.setMaxSpeed(setJointsSpeed);
  joint3.setMaxSpeed(setJointsSpeed);
  joint4.setMaxSpeed(setJointsSpeed);
  joint5.setMaxSpeed(setJointsSpeed);
  steppers.addStepper(joint1);
  steppers.addStepper(joint2);
  steppers.addStepper(joint3);
  steppers.addStepper(joint4);
  steppers.addStepper(joint5);
  }
}
