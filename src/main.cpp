// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FL                   motor         16              
// Controller1          controller                    
// FR                   motor         17              
// BR                   motor         14              
// BL                   motor         11              
// IntakeUp             motor         6               
// Sorter               motor         3               
// IntakeLeft           motor         7               
// IntakeRight          motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FL                   motor         16              
// Controller1          controller                    
// FR                   motor         17              
// BR                   motor         14              
// BL                   motor         11              
// IntakeUp             motor         6               
// Sorter               motor         3               
// IntakeLeft           motor         7               
// IntakeRight          motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----
// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FL                   motor         16              
// Controller1          controller                    
// FR                   motor         17              
// BR                   motor         14              
// BL                   motor         11              
// IntakeUp             motor         6               
// Sorter               motor         3               
// IntakeLeft           motor         7               
// IntakeRight          motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// FL                   motor         16              
// Controller1          controller                    
// FR                   motor         17              
// BR                   motor         14              
// BL                   motor         11              
// IntakeUp             motor         6               
// Sorter               motor         3               
// IntakeLeft           motor         7               
// IntakeRight          motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/


motor allMotors[] = {FL,FR,BL,BR,IntakeLeft,IntakeRight,IntakeUp,Sorter};
motor driveMotors[] = {FL,FR,BL,BR};
motor intakeMotors[] = {IntakeLeft,IntakeRight,IntakeUp};

int arrSize = sizeof(allMotors)/sizeof(allMotors[0]);

// fwd constants
double leftkP = 1;
double rightkP = 1;
double leftkI = 0.000;
double rightkI = 0.000;
double leftkD = 0.0;
double rightkD = 0.0;
double leftIntActZone = 10.0;
double rightIntActZone = 10.0;

//strafe constants
double frontkP = 0.7;
double backkP = 0.7;
double frontkI = 0.0003;
double backkI = 0.0003;
double frontkD = 0.13;
double backkD = 0.13;
double frontIntActZone = 10.0;
double backIntActZone = 10.0;

//fwd variables
int leftError; //current - desired
int rightError;
int leftPrevError = 0; //error 20 msec ago
int rightPrevError = 0;
int leftDerivative; //error - prevError
int rightDerivative;
int leftTotalError = 0; //error + totalError
int rightTotalError = 0;
int leftIntegral;
int rightIntegral;
double leftPower;
double rightPower;

//strafe variables
int frontError; //current - desired
int backError;
int frontPrevError = 0; //error 20 msec ago
int backPrevError = 0;
int frontDerivative; //error - prevError
int backDerivative;
int frontTotalError = 0; //error + totalError
int backTotalError = 0;
int frontIntegral;
int backIntegral;
double frontPower;
double backPower;


motor_group Fdrive(FR,FL);
motor_group Bdrive(BR,BL);
motor_group Rdrive(FR,BR);
motor_group Ldrive(FL,BL);

void stop(){
  for(int i =0; i<arrSize;i=i+1){
    allMotors[i].stop();
  }
}
void reset(){
  for(int i =0; i<arrSize;i=i+1){
    allMotors[i].resetRotation();
  }
}
void start(){
  for(int i =0; i<arrSize;i=i+1){
    allMotors[i].spin(forward);
  }
}

//format of array: target, high speed, low speed
void fwdPID(float leftArr[], float rightArr[], int t){
  Brain.Timer.reset();
 
  leftArr[0] = leftArr[0]/.03;
  rightArr[0] = rightArr[0]/.03;
  while(Brain.Timer.time(msec) <= t){
    while((abs(Ldrive.position(degrees)) < abs(leftArr[0])) || (abs(Rdrive.position(degrees)) < abs(rightArr[0]))) {
      int Lposition = (FL.position(degrees)+BL.position(degrees))/2;
      int Rposition = (FR.position(degrees)+BR.position(degrees))/2;
      //int AvgPosition = (Lposition + Rposition)/2;
      
      //P
      leftError = leftArr[0] - Lposition;
      rightError = leftArr[0] - Rposition;

      //I
      leftTotalError += leftError;
      if(abs(leftError) <= abs(leftIntActZone)){
        leftIntegral = leftTotalError*leftkI;
      }
      else{
        leftIntegral = 0;
      }

      rightTotalError += rightError;
      if(abs(rightError) <= abs(rightIntActZone)){
      rightIntegral = rightTotalError*rightkI;
      }
      else{
        rightIntegral = 0;
      }
      //D
      leftDerivative = leftError - leftPrevError;
      rightDerivative = rightError - rightPrevError;

      leftPower = (leftError*leftkP) + leftIntegral + (leftDerivative*leftkD);
      rightPower = (rightError*rightkP) + rightIntegral + (rightDerivative*rightkD);

      if(abs(leftPower) > abs(leftArr[1])){
        leftPower = leftArr[1];
      }
      else if(abs(leftPower) < abs(leftArr[2])){
        leftPower = leftArr[2];
      }

      if(abs(rightPower) > abs(rightArr[1])){
        rightPower = rightArr[1];
      }
      else if(abs(rightPower) < abs(rightArr[2])){
        rightPower = rightArr[2];
      }

      Ldrive.spin(forward, leftPower, rpm);
      Rdrive.spin(forward, rightPower, rpm);

      leftPrevError = leftError;
      rightPrevError = rightError;
      vex::task::sleep(20);
    }
  }

}
void strafePID(float frontArr[], float backArr[], int t){
  motor_group Fdrive(FR,FL);
  motor_group Bdrive(BR,BL);
  Brain.Timer.reset();
  int frontValue = frontArr[0]/.03490658503;
  int backValue = backArr[0]/.03490658503;
  while(Brain.Timer.time(msec) <= t){
  while(((abs(FR.position(degrees)) < abs(frontValue)-1) || (abs(FL.position(degrees)) < abs(frontValue)-1) || (abs(BR.position(degrees)) < abs(backValue)-1) || abs(BL.position(degrees)) < abs(backValue)-1)){
    int Fposition = ((FR.position(degrees)+FL.position(degrees))/2);
    int Bposition = ((BR.position(degrees)+BL.position(degrees))/2);
    //int AvgPosition = (Lposition + Rposition)/2;
    
    //P
    frontError = frontValue - Fposition;
    backError = backValue - Bposition;

    //I
    frontTotalError += frontError;
    if(abs(frontError) <= abs(frontIntActZone)){
      frontIntegral = frontTotalError*frontkI;
    }
    else{
      frontIntegral = 0;
    }

    backTotalError += backError;
    if(abs(backError) <= abs(backIntActZone)){
    backIntegral = backTotalError*backkI;
    }
    else{
      backIntegral = 0;
    }
    //D
    frontDerivative = frontError - frontPrevError;
    backDerivative = backError - backPrevError;

    frontPower = (frontError*frontkP) + frontIntegral + (frontDerivative*frontkD);
    backPower = (backError*rightkP) + backIntegral + (backDerivative*backkD);

    if(abs(frontPower) > abs(frontArr[1])){
      frontPower = frontArr[1];
    }
    else if(abs(frontPower) < abs(frontArr[2])){
      frontPower = frontArr[2];
    }
    if(abs(backPower) > abs(backArr[1])){
      backPower = backArr[1];
    }
    else if(abs(backPower) < abs(backArr[2])){
      backPower = backArr[2];
    }

    FR.spin(forward, -frontPower, rpm);
    FL.spin(forward, frontPower, rpm);
    BR.spin(forward, backPower, rpm);
    BL.spin(forward, -backPower, rpm);

    frontPrevError = frontError;
    backPrevError = backError;
    vex::task::sleep(20);
  }
  }
  Fdrive.stop();
  Bdrive.stop();
}
void leftPID(float leftArr[], int t){
  
  Brain.Timer.reset();
  int leftValue = leftArr[0]/.03490658503;
  while(Brain.Timer.time(msec) <= t){
  while(((abs(Ldrive.position(degrees)) < abs(leftValue)-1))){
    int Lposition = (FL.position(degrees)+BL.position(degrees))/2;
   
    //int AvgPosition = (Lposition + Rposition)/2;
    
    //P
    leftError = leftValue - Lposition;

    //I
    leftTotalError += leftError;
    if(abs(leftError) <= abs(leftIntActZone)){
      leftIntegral = leftTotalError*leftkI;
    }
    else{
      leftIntegral = 0;
    }
    
    //D
    leftDerivative = leftError - leftPrevError;

    leftPower = (leftError*leftkP) + leftIntegral + (leftDerivative*leftkD);

    if(abs(leftPower) > abs(leftArr[1])){
      leftPower = leftArr[1];
    }

    else if(abs(leftPower) < abs(leftArr[2])){
      leftPower = leftArr[2];
    }

    Ldrive.spin(forward, leftPower, rpm);;

    leftPrevError = leftError;
    vex::task::sleep(20);
  }
  }
}
void RightPID(float rightArr[], int t){
  
  Brain.Timer.reset();
  int rightValue = rightArr[0]/.03490658503;
  while(Brain.Timer.time(msec) <= t){
  while(((abs(Rdrive.position(degrees)) < abs(rightValue)-1)) ){
    int Rposition = (FR.position(degrees)+BR.position(degrees))/2;
    //int AvgPosition = (Lposition + Rposition)/2;
    
    //P
    rightError = rightValue - Rposition;

    //I
    rightTotalError += rightError;
    if(abs(rightError) <= abs(rightIntActZone)){
    rightIntegral = rightTotalError*rightkI;
    }
    else{
      rightIntegral = 0;
    }
    //D
    rightDerivative = rightError - rightPrevError;

    rightPower = (rightError*rightkP) + rightIntegral + (rightDerivative*rightkD);

    if(abs(rightPower) > abs(rightArr[1])){
      rightPower = rightArr[1];
    }

    else if(abs(rightPower) < abs(rightArr[2])){
      rightPower = rightArr[2];
    }

    Rdrive.spin(forward, rightPower, rpm);
    

    rightPrevError = rightError;
    vex::task::sleep(20);
  }
  }
}


void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    BR.setVelocity((Controller1.Axis3.position()-Controller1.Axis1.position()+Controller1.Axis4.position()),percent);
    BL.setVelocity((Controller1.Axis3.position()+Controller1.Axis1.position()-Controller1.Axis4.position()), percent);
    FR.setVelocity(Controller1.Axis3.position()-Controller1.Axis1.position()-Controller1.Axis4.position(), percent);
    FL.setVelocity(Controller1.Axis3.position()+Controller1.Axis1.position()+Controller1.Axis4.position(), percent);


    

    
    int drift=30;
    int shoot = 50;
    int intake = 40;


    if(Controller1.ButtonRight.pressing()){

      BR.setVelocity(-drift,percent);
      BL.setVelocity(drift, percent);
      FR.setVelocity(drift, percent);
      FL.setVelocity(-drift, percent);


    }
    if(Controller1.ButtonLeft.pressing()){
      BR.setVelocity(drift,percent);
      BL.setVelocity(-drift, percent);
      FR.setVelocity(-drift, percent);
      FL.setVelocity(drift, percent);
    }

    if(Controller1.ButtonL2.pressing()){
      IntakeUp.setVelocity(shoot,percent);
      Sorter.setVelocity(-shoot,percent);
    }else{
      IntakeUp.setVelocity(0,percent);
      Sorter.setVelocity(0,percent);

    }
    if(Controller1.ButtonR1.pressing()){
      IntakeLeft.setVelocity(intake,percent);
      IntakeRight.setVelocity(intake,percent);
    }else{
      IntakeUp.setVelocity(0,percent);
      Sorter.setVelocity(0,percent);

    }
    if(Controller1.ButtonR1.pressing()){
      IntakeLeft.setVelocity(-intake,percent);
      IntakeRight.setVelocity(-intake,percent);
    }else{
      IntakeUp.setVelocity(0,percent);
      Sorter.setVelocity(0,percent);

    }
    // BR.spin(forward);
    // BL.spin(forward);
    // FR.spin(forward);
    // FL.spin(forward);
    // IntakeLeft.spin(forward);
    // IntakeRight.spin(forward);
    // IntakeUp.spin(forward);
    // Sorter.spin(forward);

    start();




    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
