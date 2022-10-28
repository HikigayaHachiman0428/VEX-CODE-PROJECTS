#include "vex.h"
#include "cmath"
#include <iostream> 
#include <string> 
using namespace vex;
using signature = vision::signature;
using code = vision::code;

bool PlayAuto=false;
brain  Brain;
controller Controller1 = controller(primary);

motor LF = motor(PORT1, ratio18_1, false);             motor RF = motor(PORT9, ratio18_1,true);
motor LB = motor(PORT12, ratio18_1, false);             motor RB = motor(PORT6, ratio18_1,true);
motor_group LeftDriveSmart = motor_group(LF,LB);        motor_group RightDriveSmart = motor_group(RF,RB);

motor UpClaw = motor(PORT10, ratio18_1, true);           motor DownClaw = motor(PORT18, ratio18_1, false); 
motor BackClaw = motor(PORT19, ratio18_1,true); 
motor_group Claw_Group=motor_group(UpClaw, DownClaw,BackClaw);
   
motor Lift = motor(PORT21, ratio18_1,true );   

inertial Inert_Constant = inertial(PORT8);
rotation Angle = rotation(PORT4, false);


bool RemoteControlCodeEnabled = true;
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;
bool isMovingL = true;
bool isMovingR = true;
bool lowtorque =true;
bool show_claw=false;

timer drivertime;
int i=500000;

// define a task that will handle monitoring inputs from Controller1


bool first;
double real_LClawV,real_RClawV,ideal_LClawV=40,ideal_RClawV=40,LClaw_torque,ideal_RClaw_torque,LClaw_temperature,RClaw_temperature;

void c_lift(int velocity, double target){
  timer reg_time;
  reg_time.reset();
  int allowance =5;
    Lift.setVelocity(velocity,percent);
    while(reg_time.time()<1000 && (!(Lift.position(degrees)<target+allowance&&Lift.position(degrees)>target-allowance)))
    {
      if (Lift.position(degrees)>target)
        Lift.spin(reverse);
      else 
        Lift.spin(fwd);
    }
    Lift.setVelocity(0, percent);
}

/*
int fast_up_clawopen(){timer reg_time; reg_time.reset(); while(reg_time.time()<1000 && (!(0<UpClaw.position(degrees)&&UpClaw.position(degrees)<10))){UpClaw.setVelocity(-100, percent);UpClaw.spin(fwd);}UpClaw.setVelocity(0, percent);return 0;}
int fast_down_clawopen(){timer reg_time; reg_time.reset();while(reg_time.time()<500 &&(!(92<DownClaw.position(degrees)&& DownClaw.position(degrees)<100))){DownClaw.setVelocity(-100, percent);DownClaw.spin(fwd);}DownClaw.setVelocity(0, percent);return 0;}
int fast_up_clawclose(){timer reg_time; reg_time.reset();while(reg_time.time()<1000 &&(!(745<UpClaw.position(degrees)&&UpClaw.position(degrees)< 755))){UpClaw.setVelocity(100, percent);UpClaw.spin(fwd);}UpClaw.setVelocity(0, percent);return 0;}
int fast_down_clawclose(){timer reg_time; reg_time.reset();while (reg_time.time()<500 && (!(135<DownClaw.position(degrees)&&DownClaw.position(degrees)<145))){DownClaw.setVelocity(100, percent);DownClaw.spin(fwd);}DownClaw.setVelocity(0, percent);return 0;}
*/
int printing(){

  while(true)
    if((drivertime>300&&!Competition.isAutonomous())||(Competition.isAutonomous()&&drivertime>800)){
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print(Inert_Constant.heading());
      Controller1.Screen.print(" ");
      Controller1.Screen.print(Brain.Battery.voltage());
      Controller1.Screen.print("V  ");
      Controller1.Screen.print(Lift.temperature());
      Controller1.Screen.newLine();
      // Controller1.Screen.print(Angle.angle());
      // Controller1.Screen.print("`C ");
      // Controller1.Screen.newLine();
      // Controller1.Screen.print("up=");
      // Controller1.Screen.print(UpClaw.position(degrees));
      // Controller1.Screen.newLine();
      // Controller1.Screen.print(" down=");
      // Controller1.Screen.print(DownClaw.position(degrees));
      Controller1.Screen.print(" back=");
      Controller1.Screen.print(BackClaw.position(degrees));
      Controller1.Screen.newLine();
      Controller1.Screen.print("\tLift=");
      Controller1.Screen.print(Lift.position(degrees));
      // Controller1.Screen.print(Lift.position(degrees));
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(5, 20);
      Brain.Screen.print("angle");
      Brain.Screen.print(Inert_Constant.heading());
      Brain.Screen.print("    ");
      if (PlayAuto == true)
        Brain.Screen.print("play auto");
      drivertime.reset();
    }
  return 0;
}

int rc_auto_loop_function_Controller1() 
{
  while(true) {
    if(first){
      Lift.setVelocity(100, percent);
      Lift.setMaxTorque(100, percent);
      Lift.stop(coast);
      UpClaw.setVelocity(100, percent);
      UpClaw.setMaxTorque(100, percent);
      UpClaw.stop(coast);
      DownClaw.setVelocity(100, percent);
      DownClaw.setMaxTorque(100, percent);
      DownClaw.stop(coast);
      BackClaw.setVelocity(100, percent);
      BackClaw.setMaxTorque(100, percent);
      BackClaw.stop(coast);
      first=false;
    }
    double LOW_LIMIT = 108.36; double UP_LIMIT = 3.86;
    if(!Competition.isAutonomous()) {
      timer special_this;
        int Left_verticle=Controller1.Axis3.position();
        int left_horizontal=Controller1.Axis4.position();
        //控制左右
        int right_verticle=Controller1.Axis2.position();
        //控制前后
         left_horizontal*=0.6;
      int drivetrainLeftSideSpeed =Left_verticle + left_horizontal;
      int drivetrainRightSideSpeed =Left_verticle - left_horizontal;
      if(abs(left_horizontal)>10){
        if(!lowtorque){
          LeftDriveSmart.setMaxTorque(100, percent);
          RightDriveSmart.setMaxTorque(100, percent);
          lowtorque=true;
        }
      }else
        if(lowtorque){
          LeftDriveSmart.setMaxTorque(100,percent);
          RightDriveSmart.setMaxTorque(100,percent);
          lowtorque=true;
        }
        int Lift_angle=Angle.angle(); double Lift_v = right_verticle; 
      if(Lift_v<5){
        if(Lift_angle>LOW_LIMIT-7){
          Lift_v=fmax(Lift_v,-15);
          Lift.setMaxTorque(20, percent);
        }else if(Lift_angle>LOW_LIMIT-15){
          Lift_v=fmax(Lift_v,-25);
          Lift.setMaxTorque(40, percent);
        }
        else
          Lift.setMaxTorque(100, percent);
      }
      else if(Lift_v>5){
        if(Lift_angle<UP_LIMIT+7){
          Lift_v=fmin(Lift_v,15);
          Lift.setMaxTorque(20, percent);
        }else if(Lift_angle<UP_LIMIT+15){
          Lift_v=fmin(Lift_v,25);
          Lift.setMaxTorque(40, percent);
        }
        else
          Lift.setMaxTorque(100, percent);
      }
      if(fabs(Lift_v)>5){
        Lift.setVelocity(Lift_v, percent);
        Lift.spin(fwd);
      }else{
        Lift.stop(hold);
      }
      
      if (abs(drivetrainLeftSideSpeed)<10) {
        if (isMovingL) {    
          LeftDriveSmart.stop(coast);
          isMovingL = false;
        }
      } else  isMovingL = true;
      

      if (abs(drivetrainRightSideSpeed)<10) {
        if (isMovingR) {
          RightDriveSmart.stop(coast);
          isMovingR = false;
        }
      } else  isMovingR = true;
      
      if (isMovingL) {
        LeftDriveSmart.setVelocity(drivetrainLeftSideSpeed, percent);
        LeftDriveSmart.spin(forward);
      }
      if (isMovingR) {
        RightDriveSmart.setVelocity(drivetrainRightSideSpeed, percent);
        RightDriveSmart.spin(forward);
      }
      if(Controller1.ButtonR1.pressing()){
        UpClaw.setVelocity(100, percent);
        UpClaw.spin(fwd);
      }
      else if(Controller1.ButtonR2.pressing()&&UpClaw.position(degrees)>-130){
        UpClaw.setVelocity(-100, percent);
        UpClaw.spin(fwd);
      }
      else
        UpClaw.stop(brake);

      if(Controller1.ButtonL1.pressing()){
        DownClaw.setVelocity(100, percent);
        DownClaw.spin(fwd);
      }
      else if(Controller1.ButtonL2.pressing() && DownClaw.position(degrees)>-130){
        DownClaw.setVelocity(-100, percent);
        DownClaw.spin(fwd);
      }
      else
        DownClaw.stop(brake);

      if(Controller1.ButtonUp.pressing()&& BackClaw.position(degrees)<0){
        BackClaw.setVelocity(100, percent);
        BackClaw.spin(fwd);
      }
      else if(Controller1.ButtonDown.pressing() ){
        BackClaw.setVelocity(-100, percent);
        BackClaw.spin(fwd);
      }
      else
        BackClaw.stop(brake);
      // if  (Controller1.ButtonUp.pressing()) c_fast_up_clawopen();
      // if (Controller1.ButtonDown.pressing()) c_fast_down_clawopen();
      // if (Controller1.ButtonLeft.pressing()) c_fast_up_clawclose();
      // if (Controller1.ButtonRight.pressing()) c_fast_down_clawclose();
      }
      // wait before repeating the process
      if(Controller1.ButtonLeft.pressing()&&Controller1.ButtonRight.pressing()){//陀螺仪+爪子编码器初始化
        Inert_Constant.calibrate();
        Controller1.Screen.clearScreen();
        Controller1.Screen.setCursor(1,1);
        Controller1.Screen.print("calibrating");
        while(Inert_Constant.isCalibrating()){
          Controller1.Screen.clearScreen();
          Controller1.Screen.setCursor(1,1);
          Controller1.Screen.print("calibrating");
          wait(100,msec);
        }
        Inert_Constant.setHeading(180,deg);
        wait(100,msec);
        Controller1.Screen.clearScreen();
        Lift.setPosition(0, degrees);
        Claw_Group.setPosition(0, degrees);
      }
      if(Controller1.ButtonLeft.pressing()){
        Inert_Constant.setHeading(180, deg);
        Lift.setPosition(0, degrees);
        Claw_Group.setPosition(0, degrees);

      }
      
      if(Controller1.ButtonX.pressing()){//测试自动
        first=true;
        PlayAuto=true;
        Competition.test_auton();
  while(PlayAuto) wait(500000,msec);
      }
    }
  wait(20, msec);
  return 0;
}
void vexcodeInit( void ) {
  first=true;
  task driver_print(printing);
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
  wait(100, msec);
  Brain.Screen.clearScreen();
}