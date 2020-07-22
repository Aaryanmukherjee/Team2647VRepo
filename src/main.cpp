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
void fwdPID(){

}
void strafePID(){

}
void leftPID(){

}
void rightPID(){

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

//hello
    //L1 shoot; L2 sort; R1 intake, R2 Out


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
