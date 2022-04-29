/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       19480                                                     */
/*    Created:      Wed Apr 20 2022                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "stdio.h"
// #pragma platform(VEX2)
// #pragma competitionControl(Competition)
//#include "Vex_Competition_Includes.c"

using namespace vex;

// typedef struct {
// 	tSensors input_sensor;
// 	float target;
// 	float kP;
// 	float kI;
// 	float kD;
// 	float error;
// 	float previous_error;
// 	float integral;
// 	float integral_threshold;
// 	float derivative;
// } PIDController;

const float WHEEL_DIAMETER = 10.16 * 0.01; // m
const float WHEEL_CIRCUMFERENCE = M_PI * WHEEL_DIAMETER;
// const float rotConst = 0;
// const float liftRatio = 0;
const float speed = 150;
// bool taskk = 0;
// bool up = 1;
// float diag_wheel_dist = 45;
// float robot_circumference = 0.5*2*3.1415926536*diag_wheel_dist;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain Brain;

// define your global instances of motors and other devices here
competition Competition;
controller Controller1 = vex::controller();
motor TLMotor = vex::motor(vex::PORT8);
motor TRMotor = vex::motor(vex::PORT10, true);
motor BLMotor = vex::motor(vex::PORT7);
motor BRMotor = motor(vex::PORT9, true);
motor arm1 = motor(vex::PORT11, gearSetting::ratio36_1);
motor arm2 = motor(vex::PORT12, gearSetting::ratio36_1, true);
motor conveyor = motor(PORT13);
pneumatics pist1 = pneumatics(Brain.ThreeWirePort.A);
pneumatics pist3 = pneumatics(Brain.ThreeWirePort.C);
inertial inertial_s = inertial(vex::PORT20);
encoder opt1 = encoder(Brain.ThreeWirePort.H);
encoder opt2 = encoder(Brain.ThreeWirePort.G);

void L1();
void L2();
void R1();
void R2();
void X();
float setspeed = 1;

/*________________________________________________________AUTONOMOUS________________________________________________________*/


//counterclockwise
void move(float dist) {

  TLMotor.spinFor(dist/WHEEL_CIRCUMFERENCE, rev, 150, velocityUnits::pct, false);
  TRMotor.spinFor(dist/WHEEL_CIRCUMFERENCE, rev, 150, velocityUnits::pct, false);
  BLMotor.spinFor(dist/WHEEL_CIRCUMFERENCE, rev, 150, velocityUnits::pct, false);
  BRMotor.spinFor(dist/WHEEL_CIRCUMFERENCE, rev, 150, velocityUnits::pct, true);
  while (BRMotor.isSpinning()) {
    task::sleep(20);
  }

}

float robot_diam = 21.2602916255 * 2.54 * 0.01;
float robot_circum = M_PI * robot_diam;
// 14x16 inches, 21.2602916255 inches diam from TR to BL wheels

void r_turn(float angle) {

  TLMotor.spinFor((angle/360) * robot_circum / WHEEL_CIRCUMFERENCE, rev, 150, velocityUnits::pct, false);
  TRMotor.spinFor(-(angle/360) * robot_circum / WHEEL_CIRCUMFERENCE, rev, 150, velocityUnits::pct, false);
  BLMotor.spinFor((angle/360) * robot_circum / WHEEL_CIRCUMFERENCE, rev, 150, velocityUnits::pct, false);
  BRMotor.spinFor(-(angle/360) * robot_circum / WHEEL_CIRCUMFERENCE, rev, 150, velocityUnits::pct, true);
  while (BRMotor.isSpinning()) {
    task::sleep(20);
  }

}

void swingturn(float angle, bool t_state) {

  if (t_state) {
    TLMotor.spinFor((angle/360) * robot_circum, rev, 100, velocityUnits::pct, false);
    TRMotor.spinFor((angle/360) * robot_circum, rev, 50, velocityUnits::pct, false);
    BLMotor.spinFor((angle/360) * robot_circum, rev, 100, velocityUnits::pct, false);
    BRMotor.spinFor((angle/360) * robot_circum, rev, 50, velocityUnits::pct, true);
  }
  
  else {
    TLMotor.spinFor((angle/360) * robot_circum / robot_diam, rev, 50, velocityUnits::pct, false);
    TRMotor.spinFor((angle/360) * robot_circum / robot_diam, rev, 100, velocityUnits::pct, false);
    BLMotor.spinFor((angle/360) * robot_circum / robot_diam, rev, 50, velocityUnits::pct, false);
    BRMotor.spinFor((angle/360) * robot_circum / robot_diam, rev, 100, velocityUnits::pct, true);
  }
}

void rev_swingturn(float angle, bool t_state) {

  if (t_state) {
    TLMotor.spinFor(-(angle/360) * robot_circum / WHEEL_CIRCUMFERENCE, deg, 100, velocityUnits::pct, false);
    TRMotor.spinFor(-(angle/360) * robot_circum / WHEEL_CIRCUMFERENCE, deg, 50, velocityUnits::pct, false);
    BLMotor.spinFor(-(angle/360) * robot_circum / WHEEL_CIRCUMFERENCE, deg, 100, velocityUnits::pct, false);
    BRMotor.spinFor(-(angle/360) * robot_circum / WHEEL_CIRCUMFERENCE, deg, 50, velocityUnits::pct, true);
  }
  
  else {
    TLMotor.spinFor(-(angle/360) * robot_circum / WHEEL_CIRCUMFERENCE, rev, 50, velocityUnits::pct, false);
    TRMotor.spinFor(-(angle/360) * robot_circum / WHEEL_CIRCUMFERENCE, rev, 100, velocityUnits::pct, false);
    BLMotor.spinFor(-(angle/360) * robot_circum / WHEEL_CIRCUMFERENCE, rev, 50, velocityUnits::pct, false);
    BRMotor.spinFor(-(angle/360) * robot_circum / WHEEL_CIRCUMFERENCE, rev, 100, velocityUnits::pct, true);
  }
}


void lift(float angle, bool state1) {
  if (state1) {
    arm1.spinFor(100, deg, false);
    arm2.spinFor(100, deg, false);
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print(state1);
    state1 = 0;
  }

  //move down
  else {
    arm1.spinFor(-100, deg, false);
    arm2.spinFor(-100, deg, false);
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print(state1);
    state1 = 1;
  }
}

int signnum_c(int x) {
  if (x < 0) {
    return -1;
  }
  else if (x > 0) {
    return 1;
  }
  return 0;
}

//settings
double kP = 0.3;
double kI = 0.1;
double kD = 0.05;

double t_kP = 0.3;
double t_kI = 0.1;
double t_kD = 0.09;

double a_kP = 0.9;
double a_kI = 0.5;
double a_kD = 0.4;

//change in time between sensor data
float dT = 20;
int maxTurnIntegral = 300;
int maxIntegral = 300;
int integralBound = 20;

//autonomous settings:
//lateral
int desiredValue = 0;
int error = 0; //positional error
int preverror = 0; //position 20 ms ago
int derivative; //error - preverror
int totalerror; //sum of error

//turn movement
int turnDesiredValue = 0;
int turnError = 0; //positional error
int turnPreverror = 0; //position 20 ms ago
int turnDerivative; //error - preverror
int turnTotalError; //sum of error

//arm movement
int armDesiredValue = 0;
int armError = 0;
int armPrevError = 0;
int armDerivative;
int armTotalError;

bool resetDriveSensors = false;
bool PID_quit = false;
bool lateral = 1;
bool spiny = 0;

double lateralMotorPower;
double turnMotorPower;

//var for use

bool enabledrivePID = true;
int drivePID() {

  while (enabledrivePID) {

    if (resetDriveSensors) {
       
      resetDriveSensors = false;
      TLMotor.setRotation(0, deg);
      TRMotor.setRotation(0, deg);
    }

    //undefined identifier if this line isn't here... may just be my editor...
    

    /*------------lateral movement PID------------*/

    int leftMotorPos = TLMotor.rotation(deg);
    int rightMotorPos = TRMotor.rotation(deg);
     //get position of motor

    //get average of two motor
    int averagePosition;
    averagePosition = (leftMotorPos + rightMotorPos)/2;

    //Proportional
    error = desiredValue - averagePosition;

    if (abs(error) < 1) {
      PID_quit = true;
    }

    if (abs(turnError) < 1) {
      PID_quit = true;
    }

    //Derivative
    derivative = error - preverror;

    //Velocity -> position -> absement = Integral
    if (abs(error) < integralBound) {
      totalerror += error;
    }

    else {
      totalerror = 0;
    }

    if (error == 0) {
      totalerror = 0;
    }

    if (abs(totalerror) > maxIntegral) {
      totalerror = signnum_c(totalerror) * maxIntegral;
    }
    //make sure integral doesn't exceed limit

    //kD and kI are better represented as kP/T, and kP*T
    double lateralMotorPower = kP * error + kD * derivative + kI * totalerror;



    /*------------turn movement PID------------*/
    //get difference of wheel turn

    int turnDifference = leftMotorPos - rightMotorPos;

    //Proportional
    turnError = turnDesiredValue - turnDifference;
 
    //Derivative
    turnDerivative = turnError - turnPreverror;

    //Velocity -> position -> absement = Integral
    if (abs(turnError) < integralBound) {
      turnTotalError += turnError;
    }

    else {
      turnTotalError = integralBound - 0.1;
    }

    if (turnError == 0) {
      turnTotalError = 0;
    }

    if (abs(turnTotalError) > maxTurnIntegral) {
      turnTotalError = signnum_c(turnTotalError) * maxTurnIntegral;
    }

    double turnMotorPower = t_kP * turnError + t_kD * turnDerivative + t_kI * turnTotalError; 

    /*------------arm movement PID------------*/

    int arm1Pos = arm1.rotation(deg);

    //Proportional
    armError = armDesiredValue - arm1Pos;
 
    //Derivative
    armDerivative = armError - armPrevError;

    //Velocity -> position -> absement = Integral
    if (abs(armError) < integralBound) {
      armTotalError += armError;
    }

    else {
      armTotalError = integralBound - 0.1;
    }

    if (armError == 0) {
      armTotalError = 0;
    }

    if (abs(armTotalError) > maxTurnIntegral) {
      armTotalError = signnum_c(armTotalError) * maxTurnIntegral;
    }

    double armMotorPower = a_kP * turnError + a_kD * turnDerivative + a_kI * turnTotalError; 


    if (lateral) {
    TRMotor.spin(fwd, lateralMotorPower, voltageUnits::volt);
    TLMotor.spin(fwd, lateralMotorPower, voltageUnits::volt);

    BRMotor.spin(fwd, lateralMotorPower, voltageUnits::volt);
    BLMotor.spin(fwd, lateralMotorPower, voltageUnits::volt);
    }

    else if (spiny) {
    while (inertial_s.yaw() <= 90) { 
        TRMotor.spin(fwd, turnMotorPower, voltageUnits::volt);
        TLMotor.spin(fwd, -turnMotorPower, voltageUnits::volt);

        BRMotor.spin(fwd, turnMotorPower, voltageUnits::volt);
        BLMotor.spin(fwd, -turnMotorPower, voltageUnits::volt);

      }
    }

    //arm1.spin(fwd, armMotorPower, voltageUnits::volt);

    preverror = error;
    vex::task::sleep(dT);

  }
  return 0;
}

int printtocontroller() {
  while (1) {

  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("PID lateral %3.2f", lateralMotorPower);

  Controller1.Screen.setCursor(3, 1);
  Controller1.Screen.print("PID turn %3.2f", turnMotorPower);

  task::sleep(500);
  Controller1.Screen.clearLine(1);
  Controller1.Screen.clearLine(5);
  }

  return 0;
}

//values to convert setPID args to metres for convenience
float lateral_convert = 360/WHEEL_CIRCUMFERENCE;
float rotation_convert = 360/WHEEL_CIRCUMFERENCE;

void setPID(float val, char a) {
  resetDriveSensors = true;
  desiredValue = lateral_convert * val;
  turnDesiredValue = rotation_convert * val;

  //activate lateral movement
  if (a == 'f') {
    lateral = 1;
    spiny = 0;
  }

  //activate spin movement 
  else if (a == 's') {
    lateral = 0;
    spiny = 1;
  }
  PID_quit = false;
  while (!PID_quit) {
    task::sleep(20);
  }
}

void skills() {
  rev_swingturn(setspeed*2, false);
  move(setspeed*5);
  // R2(); //grip base
  // move(-setspeed*10);
  // turn(setspeed*50);
  // lift(10, true); //raise arm
  // move(setspeed*50);
  // setspeed = 0.5;
  // move(setspeed*50);
  // turn(-setspeed*50);
  // lift(-1, false); //lower arm on base
  // R2(); //ungrip
  // lift(1, true); //move arm up a bit  
  // lift(-50, false); //bring arm down
  // turn(setspeed*50);
}

void  preauton(void){


}

void auton(void) {

  vex::task drivep = task(drivePID);
  vex::task printcon = task(printtocontroller);
  Controller1.Screen.print("PID activated");

  arm1.setBrake(brake);
  arm2.setBrake(brake);
  resetDriveSensors = true;
  // R2();
  //-0.24 in logger m
  // move(1.3);
  // R2();
  // move(-100);
  // r_turn(90);
  setPID(1, 0);
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("complete");
  while (PID_quit) {
    task::sleep(20);
  }
  setPID(-1, 0);
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("complete 2");
  while (PID_quit) {
    task::sleep(20);
  }

  //rotate
  setPID(0, 90);
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print("complete 3");
  while (PID_quit) {
    task::sleep(20);
  }

  R2();
  setPID(0.8663, 0);


  vex::task::sleep(5000);
  Controller1.Screen.print("PID #1");

  //Controller1.Screen.print("PID activated");

  vex::task::sleep(1000);

  setPID(1000, 1000);
  
  Controller1.Screen.print("PID 2");

  vex::task::sleep(1000);

  setPID(0,1000000000);

  Controller1.Screen.print("PID 3");

  /* start */

  


}

/*________________________________________________________DRIVER________________________________________________________*/

bool state1 = 1;
bool convey_state = 1;

void L1() {
  //pneumatics and conveyor
  
  if (state1) {
    pist3.open();
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print(pist3.value());
    
    state1 = 0;
    convey_state = 0;
  }
  else {
    pist3.close();
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print(pist3.value());

    state1 = 1;
    convey_state = 1;
  }
  

  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print(convey_state);
  
}

void L2() {
  //conveyor manual
  convey_state = 0;
}

bool state2 = 1;
int i_position;

void R1() {
  //arm

  //move up
  if (state2) {
    arm1.spinFor(650, deg, 100, velocityUnits::pct, false);
    arm2.spinFor(650, deg, 100, velocityUnits::pct, false);
    state2 = 0;
  }

  //move down
  else {
    arm1.spinFor(-650, deg, 100, velocityUnits::pct, false);
    arm2.spinFor(-650, deg, 100, velocityUnits::pct, false);
    state2 = 1;
  }

  //this_thread::sleep_for(500);
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print(state2);
}

bool state3 = 1;

void R2() {
//move lock/grip device

  if (state3) {
    pist1.open();
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("open");
    state3 = 0;
  }

  else {
    pist1.close();
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("close");
    state3 = 1;
  }
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print(state3);
}


void X() {
  //drive up ramp
  vex::task drivep = task(drivePID);

  setPID(1000, 0);

  while (TRMotor.isSpinning()) {
    task::sleep(20);
  }

  TRMotor.spinFor(1, rev, 150, velocityUnits::pct, true);
  TLMotor.spinFor(1, rev, 150, velocityUnits::pct, true);
  BLMotor.spinFor(1, rev, 150, velocityUnits::pct, false);
  BRMotor.spinFor(1, rev, 150, velocityUnits::pct, false);
}

void Up() {
  arm1.spinFor(100,rotationUnits::deg,150, velocityUnits::pct,false);
  arm2.spinFor(100,rotationUnits::deg,150, velocityUnits::pct,false);
  while(arm1.isSpinning()){
    task::sleep(30);
  }
}

void Down() {
  arm1.spinFor(-100,rotationUnits::deg,150, velocityUnits::pct,false);
  arm2.spinFor(-100,rotationUnits::deg,150, velocityUnits::pct,false);
  while(arm1.isSpinning()){
    task::sleep(30);
  }
}

void A()  {
//move goal from black clamp to large arm

}

void B() {
//flip see saw

}

void Y() {

//move goal from large arm to black clamp
}

/*motor lock = motor(PORT20);
bool state4 =1;
void LOCK() {
  if (state4) {
    lock.spinFor(fwd, 120, deg, 100, velocityUnits::pct);
    state4 = 0;
  }
  else {
    lock.spinFor(fwd, -120, deg, 100, velocityUnits::pct);
    state4 = 1;
  }
  
}*/


void usercontrol(void) {

  enabledrivePID = false;
  
  i_position = arm1.rotation(deg);

  Controller1.Screen.print("Hello");

  while(1) {

    Controller1.ButtonR1.pressed(R1);
    
    Controller1.ButtonR2.pressed(R2);
    if (1) {
      Controller1.ButtonL1.pressed(L1);
    }
    else if (Controller1.ButtonL2.pressing()) {
      convey_state = 0;
      }

    Controller1.ButtonL2.pressed(L2);
    Controller1.ButtonX.pressed(X);
    Controller1.ButtonUp.pressed(Up);
    Controller1.ButtonDown.pressed(Down);

    if (0) {

    }
    
    if (abs(Controller1.Axis3.position()) > 15 || abs(Controller1.Axis1.position()) > 15) {
    TLMotor.spin(vex::directionType::fwd, speed*(Controller1.Axis3.position() + Controller1.Axis1.position()*0.75)/100, vex::velocityUnits::pct);
    TRMotor.spin(vex::directionType::fwd, speed*(Controller1.Axis3.position() - Controller1.Axis1.position()*0.75)/100, vex::velocityUnits::pct);
    BLMotor.spin(vex::directionType::fwd, speed*(Controller1.Axis3.position() + Controller1.Axis1.position()*0.75)/100, vex::velocityUnits::pct);
    BRMotor.spin(vex::directionType::fwd, speed*(Controller1.Axis3.position() - Controller1.Axis1.position()*0.75)/100, vex::velocityUnits::pct);
    }

    else {
      TLMotor.stop();
      TRMotor.stop();
      BLMotor.stop();
      BRMotor.stop();
    }

    //Controller1.ButtonA.pressed(LOCK);


    if (convey_state) {
      conveyor.spin(fwd, 100, pct);
    }
    else {
      conveyor.setBrake(brake);
      conveyor.stop();
    }

    if (arm1.position(deg) < i_position) {
      // arm1.setBrake(brake);
      // arm2.setBrake(brake);
    }

    Brain.Screen.print(opt1.position(deg));
    Brain.Screen.print(opt2.position(deg));
    Brain.Screen.clearScreen();

  task::sleep(10);

  inertial::quaternion inertialSensorQuaternion = inertial_s.orientation();

        Brain.Screen.printAt( 20,  30, "GX  %8.3f", inertial_s.gyroRate( xaxis, dps ) );
        Brain.Screen.printAt( 20,  45, "GY  %8.3f", inertial_s.gyroRate( yaxis, dps ) );
        Brain.Screen.printAt( 20,  60, "GZ  %8.3f", inertial_s.gyroRate( zaxis, dps ) );

        Brain.Screen.printAt( 20,  90, "AX  %8.3f", inertial_s.acceleration( xaxis ) );
        Brain.Screen.printAt( 20, 105, "AY  %8.3f", inertial_s.acceleration( yaxis ) );
        Brain.Screen.printAt( 20, 120, "AZ  %8.3f", inertial_s.acceleration( zaxis ) );

        Brain.Screen.printAt( 20, 150, "A   %8.3f", inertialSensorQuaternion.a );
        Brain.Screen.printAt( 20, 165, "B   %8.3f", inertialSensorQuaternion.b );
        Brain.Screen.printAt( 20, 180, "C   %8.3f", inertialSensorQuaternion.c );
        Brain.Screen.printAt( 20, 195, "D   %8.3f", inertialSensorQuaternion.d );

        Brain.Screen.printAt( 150, 30, "Roll     %7.2f", inertial_s.roll() );
        Brain.Screen.printAt( 150, 45, "Pitch    %7.2f", inertial_s.pitch() );
        Brain.Screen.printAt( 150, 60, "Yaw      %7.2f", inertial_s.yaw() );

        Brain.Screen.printAt( 150, 90, "Heading  %7.2f", inertial_s.heading() );
        Brain.Screen.printAt( 150,105, "Rotation %7.2f", inertial_s.rotation() );

        if( inertial_s.isCalibrating() )
          Brain.Screen.printAt( 20,225, "Calibration  In Progress" );
        else
          Brain.Screen.printAt( 20,225, "Calibration  Done" );

        Brain.Screen.render();
  }
}

int main() {

  Competition.autonomous(auton);
  Competition.drivercontrol(usercontrol);

}