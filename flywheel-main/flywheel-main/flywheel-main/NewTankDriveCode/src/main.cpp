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
// Controller1          controller                    
// rightFront           motor         6               
// leftFront            motor         3               
// leftBack             motor         8               
// rightBack            motor         14              
// Intake               motor         9               
// Expansion            digital_out   A               
// flywheel             motor         11              
// flywheel2            motor         15              
// indexer              motor         16              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>
#include <string>

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

//auton selector 
int autonselect = 1;
int numOfAutons = 9;

int getSign (double inputValue) {
  if (inputValue > 0){
    return 1;
  }
  else if (inputValue < 0){
    return -1;
  }
  else return 0;
}

void driveFunction(){
  
}
//PID settings
double kP = 0.0;
double kI = 0.0;
double kD = 0.0;
double turnkP = 0.0;
double turnkI = 0.0;
double turnkD = 0.0;

int error; //SensorValue - DesiredValue --- positional value
int prevError = 0; //position 20 milliseconds ago
int derivative; //difference between error and previous error --- calculates speed
int totalError = 0;// totalError = totalError + error --- integral converts position to absement

//turn variables
int turnError; //SensorValue - DesiredValue --- positional value
int turnPrevError = 0; //position 20 milliseconds ago
int turnDerivative; //difference between error and previous error --- calculates speed
int turnTotalError = 0;

int desiredValue = 0;
int desiredTurnValue = 0;

bool enabledrivePID = true;
//switch to reset the Drive
bool resetDriveSensors = false;


int drivePID(){
  
  while(enabledrivePID){

    if(resetDriveSensors){
      resetDriveSensors = false;

      rightFront.setPosition(0, degrees);
      leftFront.setPosition(0, degrees);
      leftBack.setPosition(0, degrees);
      rightBack.setPosition(0, degrees);
    }
    //get the position of the motors
    int rightFrontPosition = rightFront.position(degrees);
    int leftFrontPosition = leftFront.position(degrees);
    int leftBackPosition = leftBack.position(degrees);
    int rightBackPosition = rightBack.position(degrees);
    //////////////////////////////////////////////////////////////////////////////////
    //lateral movement PID
    ////////////////////////////////////////////////////////////////////////
    //get the average of the four motors
    int averagePosition = (rightFrontPosition + leftFrontPosition + leftBackPosition + rightBackPosition)/4;

    error = averagePosition - desiredValue;

    derivative = error - prevError;

    //absement = position * time -- this is the integral
    totalError += error;

    //add everything up to a mootor power
    double lateralMotorPower = (error * kP + derivative * kD + totalError * kI)/12;

    //////////////////////////////////////////////////////////////////////////////////
    //turning PID
    int turnDifference = (rightFrontPosition - leftFrontPosition);

    turnError = turnDifference - desiredTurnValue;

    turnDerivative = turnError - turnPrevError;

    //absement = position * time -- this is the integral
    turnTotalError += turnError;

    //add everything up to a mootor power
    double turnMotorPower = (turnError * turnkP + turnDerivative * turnkD + turnTotalError * turnkI)/12.0;
    ////////////////////////////////////////////////////////////////////////
    //putting in the motor power into the motor statements
    rightFront.spin(forward, lateralMotorPower + turnMotorPower, percent);
    leftFront.spin(forward, lateralMotorPower - turnMotorPower, percent);
    leftBack.spin(forward, lateralMotorPower - turnMotorPower, percent);
    rightBack.spin(forward, lateralMotorPower + turnMotorPower, percent);

    prevError = error;
    turnPrevError = turnError;

    vex::task::sleep(20);
  }

  return 1;

}

void simpleDrive(){
  int forwardAmount = Controller1.Axis3.position();
  int turnAmount = Controller1.Axis1.position();
  rightFront.setVelocity(100, percent);
  leftFront.setVelocity(100, percent);
  leftBack.setVelocity(100, percent);
  rightBack.setVelocity(100, percent);

  rightFront.spin(reverse,forwardAmount - turnAmount, percent);
  leftFront.spin(forward, forwardAmount + turnAmount, percent);
  leftBack.spin(reverse, forwardAmount + turnAmount, percent);
  rightBack.spin(forward, forwardAmount - turnAmount, percent);
}

/*void rollerCode(){
  Intake.setVelocity(100, percent);
  

  if(Controller1.ButtonUp.pressing()){
    Intake.spin(forward);
    
  }
  else if(Controller1.ButtonDown.pressing()){
    Intake.spin(reverse);
    
  }
  else{
    Intake.stop();
    
  }
}
*/
void IndexerCode(){
  indexer.setVelocity(100, percent);
  if(Controller1.ButtonR1.pressing()){
    indexer.spin(forward);
  }
  else if(Controller1.ButtonR2.pressing()){
    indexer.spin(reverse);
  }
  else{
    indexer.stop();
  }
}

void intakeCode(){
  Intake.setVelocity(80,percent);
  if(Controller1.ButtonL1.pressing()){
    Intake.spin(forward);
    
  }
  else if(Controller1.ButtonL2.pressing()){
    Intake.spin(reverse);
  }
  else{
    Intake.stop();
  }
}

///////////////////////////////////////////////////////////////////////
//////////////////////////flywheel pid/////////////////////////////////

bool Controller1XY = true;

double fly_kp = 0.1; // how fast it increases
double fly_ki = 0.3; // how much offshoot/range of fluctuation
double fly_kd = 0.00005; // how many fluctuations are there
double speed_margin = 0;
double speed_marg_pct = 2;
bool flyescvar = false;
int speed_volt = 0;

//flywheel spin

void flywheel_spin_fwd(double flywheel_target_speed_pct) {
  
  flywheel.setVelocity(flywheel_target_speed_pct, pct);
  flywheel2.setVelocity(flywheel_target_speed_pct, pct);
  flywheel.spin(directionType::fwd);
  flywheel2.spin(directionType::rev);
}

//flywheel spin PID code
void flywheel_PID(double flywheel_target_speed_pct){
double averagevolt = 0;
double preverror = 0;
double errorsum = 0;
double error = 0;
double derivative = 0;
double flywheel_target_speed_volt = (flywheel_target_speed_pct/100)*12;
Controller1.Screen.setCursor(1,1);
Controller1.Screen.print("         ");
wait(20,msec);
 
 while (flyescvar == false) {
    averagevolt = flywheel.voltage();
    error = flywheel_target_speed_volt - averagevolt;
    derivative = preverror - error;
    errorsum += error;
    preverror = error;
    speed_margin = fabs((error/flywheel_target_speed_volt) * 100);
    speed_volt =  error * fly_kp + fly_ki * errorsum + fly_kd * derivative;
    wait(20,msec);
  
    if(speed_margin <= speed_marg_pct) {
      flyescvar = true;
    } else {
        flywheel.spin(forward, speed_volt, volt);
    }
    wait(20, msec);
  }
 Controller1.Screen.setCursor(3,9);
 Controller1.Screen.print("DONE");
 wait(20,msec);
 
 // Maintain the speed
 flywheel.spin(forward, speed_volt, volt);
}
bool flywheelStart = false;
void flywheelMovement() {
    if(Controller1.ButtonDown.pressing()){
      flywheel.setVelocity(65, pct);
      flywheel2.setVelocity(65, pct);
      flywheel.spin(forward);
      flywheel2.spin(reverse);
      Controller1XY = false;
    } else if(Controller1.ButtonUp.pressing()) {
      flywheel.setVelocity(52, pct);
      flywheel2.setVelocity(52, pct);
      flywheel.spin(forward);
      flywheel2.spin(reverse);
      Controller1XY = false;
    } else if(!Controller1XY) {
      flywheel.setStopping(coast);
      flywheel2.setStopping(coast);
      flywheel.stop();
      flywheel2.stop();
    }
    if(Controller1.ButtonDown.pressing()){
      flywheel.setVelocity(65, pct);
      flywheel2.setVelocity(65, pct);
      flywheel.spin(forward);
      flywheel2.spin(reverse);
      Controller1XY = false;
    } 
    else if(Controller1.ButtonUp.pressing()) {
      flywheel.setVelocity(52, pct);
      flywheel2.setVelocity(52, pct);
      flywheel.spin(forward);
      flywheel2.spin(reverse);
      Controller1XY = false;
    } else if(!Controller1XY) {
      flywheel.setStopping(coast);
      flywheel2.setStopping(coast);
      flywheel.stop();
      flywheel2.stop();
    }
}

 int selected = 0;
std::string autons[9] = {"Disabled", "1 Roller Red perp side", "2 Roller Red parallel side", "3 Roller Blue", "4 Roller Blue perp side" , "5 red shoot disk into high goal parallel side" , "6 blue shoot disk into high goal parallel side", "7 red awp" , "8 blue awp"};
int size = sizeof(autons);

bool elevated = false;

void autonSelector(){
  Controller1.Screen.clearScreen();
  task::sleep(100);
  while(true){
    Controller1.Screen.clearScreen();
    task::sleep(100);
    Controller1.Screen.clearLine(2);
    Controller1.Screen.setCursor(2,1);
    Controller1.Screen.print((autons[selected] + ",").c_str()); //e=mc^2
    Controller1.Screen.newLine();
    Controller1.Screen.print((elevated ? "Elevated" : "Default"));
    task::sleep(100);
     if(Controller1.ButtonRight.pressing()){
      elevated = !elevated;
        if (!elevated) {
          selected = (selected + 1 + size) % size;
        }
     }else if(Controller1.ButtonLeft.pressing()){
       elevated = !elevated;
       if (elevated) {
        selected = (selected - 1 + size) % size;
       }
     }else if(Controller1.ButtonA.pressing()){
       task::sleep(100);
       if(Controller1.ButtonA.pressing()){
         goto slctEnd;
       }
     }
   }
   slctEnd:
   Controller1.rumble("..");
}
void pre_auton(void) {
// Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  autonSelector();
  Expansion.set(true);
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
  
  switch(selected){
    case 0:{ //Disabled
      break;
    }
    case 1:{ //1 Roller Red perp side
    //shoot out disc to low goal
     flywheel.setVelocity(70, percent);
     flywheel2.setVelocity(70, percent);
     flywheel.spinFor(forward, 100, degrees, false);
     flywheel.spinFor(reverse, 100, degrees);
     wait(0.5, seconds);
     //turn
     rightFront.spinFor(forward, 200, degrees, false);
     leftBack.spinFor(forward, 200, degrees);
     //move forward towards roller
     rightFront.spinFor(forward, 100, degrees, false);
     leftFront.spinFor(forward, 100, degrees);
     //turn to realign with the roller
     leftFront.spinFor(reverse, 100, degrees, false);
     rightBack.spinFor(reverse, 100, degrees);
     //move back a bit to get closer to the roller
     rightFront.spinFor(reverse, 20, degrees, false);
     leftFront.spinFor(reverse, 20, degrees);
     //spin the roller
     Intake.setVelocity(100, percent);
     Intake.spinFor(forward, 1000, degrees);
     
     


      break;
    }
    case 2:{ //2 Roller Red parallel side 
    //move towards the roller
      rightFront.spinFor(reverse, 40, degrees, false);
      leftFront.spinFor(forward, 40, degrees, false);
    //spin the roller
      Intake.setVelocity(100, percent);
      Intake.spinFor(reverse, 500, degrees);
    //move back to original position
      rightFront.spinFor(forward, 40, degrees, false);
      leftFront.spinFor(reverse, 40, degrees);
      break;
    }
    case 3: { //3 Roller Blue
    //move forward towards roller
      rightFront.spinFor(reverse,40, degrees, false);
      leftFront.spinFor(forward, 40, degrees, false);
    //spin the roller
      Intake.setVelocity(100, percent);
      Intake.spinFor(reverse, 500, degrees, false);
    //move back to original position
      wait(0.5, seconds);
      rightFront.spinFor(forward, 40, degrees, false);
      leftFront.spinFor(reverse, 40, degrees);
      /*
      wait(0.5, seconds);
      //turn towards low goal
      rightFront.spinFor(reverse, 1000, degrees, false);
      leftBack.spinFor(reverse, 1000, degrees, false);
      wait(0.5, seconds);
      //move towards goal
      rightFront.spinFor(reverse, 1000, degrees, false);
      leftFront.spinFor(forward, 1000, degrees);
      break;
      */
    }
    case 4: { //4 Roller Blue perp side
    rightFront.spinFor(reverse, 270, degrees, false);
    leftFront.spinFor(reverse, 270, degrees, false);
    wait(1, seconds);
    Intake.spinFor(reverse, 1250, degrees, true);
      /*//shoot out to low goal
      rightFront.setVelocity(70, percent);
      leftFront.setVelocity(70, percent);
      rightBack.setVelocity(70, percent);
      leftBack.setVelocity(70, percent);
      rightFront.spinFor(forward, 450, degrees,false);
      rightBack.spinFor(reverse, 450, degrees, false);
      leftBack.spinFor(forward, 450, degrees, false);
      leftFront.spinFor(reverse, 450, degrees, false);
      flywheel.setVelocity(90, percent);
      flywheel2.setVelocity(90, percent);
      flywheel.spinFor(forward, 3000, degrees, false);
      flywheel2.spinFor(reverse, 3000, degrees, false);
      wait(1, seconds);
      indexer.spinFor(forward, 500, degrees, false);
      wait(2, seconds);
      indexer.spinFor(forward, 500, degrees, false);
      wait(2, seconds);
      Intake.setVelocity(100, percent);
      //move forward towards roller
      rightFront.spinFor(reverse, 1450, degrees,false);
      rightBack.spinFor(forward, 1450, degrees, false);
      leftBack.spinFor(reverse, 1450, degrees, false);
      leftFront.spinFor(forward, 1450, degrees, false);
      //move towards roller
      wait(1, seconds);
      //turn to realign with the roller

      rightFront.spinFor(reverse, 540, degrees, false);
      leftBack.spinFor(forward, 540, degrees, false);
      wait(1 , seconds);
      rightFront.spinFor(reverse, 50, degrees, false);
      leftFront.spinFor(forward, 50, degrees, false);
      wait(3, seconds);
      Intake.spinFor(forward, 50, degrees, true);
      

      //shoot out to low goal
      flywheel.setVelocity(70, percent);
      flywheel2.setVelocity(70, percent);
      indexer.spinFor(forward, 1000, degrees, false);
      flywheel.spinFor(forward, 100, degrees, false);
      flywheel.spinFor(reverse, 100, degrees);
      wait(0.5, seconds);
*/
     break;

      break;
    }
    case 5: { //5 red shoot disk into high goal parallel side
      //move forward torwards disk
      rightFront.spinFor(forward, 2000, degrees, false);
      leftFront.spinFor(reverse, 2000, degrees);
      //turn towards disc
      rightFront.spinFor(forward, 100, degrees, false);
      leftBack.spinFor(forward, 100, degrees);
      //move forward and intake the disc
      rightFront.spinFor(forward, 100, degrees, false);
      leftFront.spinFor(reverse, 100, degrees, false);
      Intake.spinFor(forward, 1000, degrees);
      //turn to shooting position
      leftBack.spinFor(forward, 1000, degrees, false);
      rightFront.spinFor(forward, 1000, degrees);
      //shoot out the disc ._.
      //set velocity
      flywheel.setVelocity(70,percent);
      flywheel2.setVelocity(70,percent);
      //then shoot
      indexer.spinFor(forward, 1000, degrees, false);
      flywheel.spinFor(forward, 100, degrees, false);
      flywheel2.spinFor(forward, 100, degrees);
      break;
    }
    case 6: {//6 blue shoot disk into high goal parallel side
      //move forward torwards disk
      rightFront.spinFor(forward, 2000, degrees, false);
      leftFront.spinFor(reverse, 2000, degrees);
      //turn towards disc
      rightFront.spinFor(forward, 100, degrees, false);
      leftBack.spinFor(forward, 100, degrees);
      //move forward and intake the disc
      rightFront.spinFor(forward, 100, degrees, false);
      leftFront.spinFor(reverse, 100, degrees, false);
      Intake.spinFor(forward, 1000, degrees);
      //move forward and intake the disc
      rightFront.spinFor(forward, 100, degrees, false);
      leftFront.spinFor(reverse, 100, degrees, false);
      Intake.spinFor(forward, 1000, degrees);
      //move forward and intake the disc
      rightFront.spinFor(forward, 100, degrees, false);
      leftFront.spinFor(reverse, 100, degrees, false);
      Intake.spinFor(forward, 1000, degrees);
      //turn to shooting position
      leftBack.spinFor(forward, 1000, degrees, false);
      rightFront.spinFor(forward, 1000, degrees);
      //shoot out the disc ._.
      //set velocity
      flywheel.setVelocity(95,percent);
      flywheel2.setVelocity(95,percent);
      //then shoot
      indexer.spinFor(100, degrees, false);
      flywheel.spinFor(forward, 100, degrees, false);
      flywheel2.spinFor(forward, 100, degrees);
      break;
    }
    case 7: {//7 red awp
      //roll the roller first ._.
      rightFront.spinFor(forward, 20, degrees, false);
      leftFront.spinFor(forward, 20, degrees);
      Intake.spinFor(forward, 1000, degrees);
      //then move forward
      rightFront.spinFor(forward, 500, degrees, false);
      leftFront.spinFor(reverse, 500, degrees);
      //turn towards the disc
      rightFront.spinFor(forward, 100, degrees, false);
      leftBack.spinFor(forward, 100, degrees);
      //move towards disc and intake it
      rightFront.spinFor(forward, 1000, degrees, false);
      leftFront.spinFor(reverse, 1000, degrees, false);

      Intake.spinFor(forward, 1000, degrees);
      //turn towards the high goal to shoot
      rightFront.spinFor(reverse, 100, degrees, false);
      leftBack.spinFor(reverse, 100, degrees);
      //start up the flywheel and use indexer to shoot the disc out
      indexer.spinFor(forward, 1000, degrees, false);
      flywheel.spinFor(forward, 200, degrees, false);
      flywheel2.spinFor(forward, 200, degrees);
      //turn towards roller
      rightFront.spinFor(forward, 100, degrees, false);
      leftFront.spinFor(forward, 100, degrees);
      //move forward towards the roller
      rightFront.spinFor(forward, 1000, degrees, false);
      leftFront.spinFor(reverse, 1000, degrees);
      //turn the roller
      Intake.spinFor(forward, 1000, degrees);

      break;

    }
    case 8: { //8 blue awp
      //roll the roller first ._.
      rightFront.spinFor(forward, 20, degrees, false);
      leftFront.spinFor(forward, 20, degrees);
      Intake.spinFor(forward, 1000, degrees);
      //then move forward
      rightFront.spinFor(forward, 500, degrees, false);
      leftFront.spinFor(reverse, 500, degrees);
      //turn towards the disc
      rightFront.spinFor(forward, 100, degrees, false);
      leftBack.spinFor(forward, 100, degrees);
      //move towards disc and intake it
      rightFront.spinFor(forward, 1000, degrees, false);
      leftFront.spinFor(reverse, 1000, degrees, false);

      Intake.spinFor(forward, 1000, degrees);
      //turn towards the high goal to shoot
      rightFront.spinFor(reverse, 100, degrees, false);
      leftBack.spinFor(reverse, 100, degrees);
      //start up the flywheel and use indexer to shoot the disc out
      indexer.spinFor(forward, 1000, degrees, false);
      flywheel.spinFor(forward, 200, degrees, false);
      flywheel2.spinFor(forward, 200, degrees);
      //turn towards roller
      rightFront.spinFor(forward, 100, degrees, false);
      leftFront.spinFor(forward, 100, degrees);
      //move forward towards the roller
      rightFront.spinFor(forward, 1000, degrees, false);
      leftFront.spinFor(reverse, 1000, degrees);
      //turn the roller
      Intake.spinFor(forward, 1000, degrees);
      break;
    }
}
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
    simpleDrive();
    //rollerCode();
    intakeCode();
    IndexerCode();
    //flywheelMovement();
    ///////////////////////////////////flywheel pid calling//////////////////////////////////////////////////
    if (Controller1.ButtonUp.pressing() == true){
      flywheel_PID(60.0);
    }
    if((Controller1.ButtonDown.pressing() == true) && (Controller1.ButtonUp.pressing() == false)){
      flywheel_PID(100.0);
    }
    if((Controller1.ButtonUp.pressing() == false) && (Controller1.ButtonDown.pressing() == false)){
      flywheel.stop();
    }
    
    //expansion using two pistons being controlled together to trigger the catapults
    //these pistons will use boolean values to send info to the brain
    //so we will be using simple detection for this
    
    if(Controller1.ButtonA.pressing() && Controller1.ButtonB.pressing()){
      Expansion.set(false);
    }
    else if(Controller1.ButtonB.pressing()){
      Expansion.set(true);
    }
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
