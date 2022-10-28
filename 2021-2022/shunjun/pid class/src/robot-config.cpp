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

// double liff_logic(int velocity){
//     double klift;
//     double target;
//     if(Angle.angle()>296)
//       target = 318;
//     else
//       target = 200;
//     klift = fabs(Angle.angle()-target + 10) /50;
//     if ((target > Angle.angle() && velocity<0) || (target < Angle.angle() && velocity >0)) return velocity * klift;
//     else  return velocity; 
// }
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
int c_up_lift(){c_lift(100, 1300);return 0;}
int c_down_lift(){c_lift(100, 0);return 0;}
int c_take(){c_lift(100,450);return 0;}
int c_fast_up_clawopen(){UpClaw.setVelocity(-100, percent);while(!(762<UpClaw.position(degrees)&&UpClaw.position(degrees)<767)){UpClaw.spin(fwd);}return 0;}
int c_fast_down_clawopen(){DownClaw.setVelocity(-100, percent);while(!(100<DownClaw.position(degrees)&& DownClaw.position(degrees)<160)){DownClaw.spin(fwd);}return 0;}
int c_fast_up_clawclose(){UpClaw.setVelocity(100, percent);while(!(0<UpClaw.position(degrees)&&UpClaw.position(degrees)< 5)){UpClaw.spin(fwd);}return 0;}
int c_fast_down_clawclose(){DownClaw.setVelocity(100, percent);while (!(0<DownClaw.position(degrees)&&DownClaw.position(degrees)<5)){DownClaw.spin(fwd);}return 0;}
  
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
      DownClaw.setVelocity(40, percent);
      DownClaw.setMaxTorque(100, percent);
      DownClaw.stop(coast);
      first=false;
    }
    double LOW_LIMIT = 108.36; double UP_LIMIT = 3.86;
    if(!Competition.isAutonomous()) {
      timer special_this;
      bool one_control = true;
      if(one_control)
      {
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
      if(Controller1.ButtonR1.pressing())
        UpClaw.spin(fwd);
      else if(Controller1.ButtonR2.pressing()&&UpClaw.position(degrees)>-130)
        UpClaw.spin(reverse);
      else
        UpClaw.stop(brake);
      if(Controller1.ButtonL1.pressing())
        DownClaw.spin(fwd);
      else if(Controller1.ButtonL2.pressing() && DownClaw.position(degrees)>-130)
        DownClaw.spin(reverse);
      else
        DownClaw.stop(brake);
      double backclawspeed;
      if(Controller1.ButtonDown.pressing()&& Controller1.ButtonUp.pressing()==0) backclawspeed = 100;
      if(Controller1.ButtonDown.pressing()==0 && Controller1.ButtonUp.pressing()) backclawspeed = -100;
      if(Controller1.ButtonDown.pressing()==0 && Controller1.ButtonUp.pressing()==0) backclawspeed =0;
      BackClaw.setMaxTorque(100, percent);BackClaw.setVelocity(backclawspeed, percent); BackClaw.spin(reverse);
      // if  (Controller1.ButtonA.pressing())  c_up_lift();
      // if (Controller1.ButtonB.pressing()) c_down_lift();
      // if  (Controller1.ButtonUp.pressing()) c_fast_up_clawopen();
      // if (Controller1.ButtonDown.pressing()) c_fast_down_clawopen();
      // if (Controller1.ButtonLeft.pressing()) c_fast_up_clawclose();
      // if (Controller1.ButtonRight.pressing()) c_fast_down_clawclose();
      }
      else{
       // Dmove()
      double zhixing;
      double zhuanwan;
      double torque=100;
      zhixing=Controller1.Axis3.position(pct);
      zhuanwan=Controller1.Axis1.position(pct);
      if(fabs(zhixing)<5) zhixing=0;
      if(fabs(zhuanwan)<5) zhuanwan=0;
      LF.spin(forward); LB.spin(forward);
      RF.spin(forward);RB.spin(forward);
      LF.setMaxTorque(torque, percent); LB.setMaxTorque(torque, percent);
      RF.setMaxTorque(torque, percent); RB.setMaxTorque(torque, percent);
      LF.setVelocity((zhixing+0.6*zhuanwan)*0.8,percent); LB.setVelocity((zhixing+0.6*zhuanwan)*0.8,percent);
      RF.setVelocity((zhixing-0.6*zhuanwan)*0.8,percent); RB.setVelocity((zhixing-0.6*zhuanwan)*0.8,percent);
//wait(30,msec);
/*zuoqian.stop();
zuohou.stop();
youqian.stop();
youhou.stop();*/
      if(Controller1.ButtonB.pressing()==1) c_up_lift();
      if(Controller1.ButtonA.pressing()) c_take();
      if(Controller1.ButtonX.pressing()) c_down_lift();
      double Lift_v=0; 
      if(Controller1.ButtonL1.pressing()==1 && Controller1.ButtonL2.pressing()==0) Lift_v=100;
      if(Controller1.ButtonL1.pressing()==0 && Controller1.ButtonL2.pressing()==1) Lift_v=-100;
      if(Controller1.ButtonL1.pressing()==0 && Controller1.ButtonL2.pressing()==0) Lift_v=0;
      int Lift_angle=Angle.angle();
      if(Lift_v<5){
        if(Lift_angle<LOW_LIMIT+1){
          Lift_v=fmax(Lift_v,-25);
          Lift.setMaxTorque(25, percent);
        }else if(Lift_angle<LOW_LIMIT+5){
          Lift_v=fmax(Lift_v,-40);
          Lift.setMaxTorque(50, percent);
        }
        else
          Lift.setMaxTorque(100, percent);
      }
      else if(Lift_v>5){
        if(Lift_angle>UP_LIMIT-1){
          Lift_v=fmin(Lift_v,25);
          Lift.setMaxTorque(25, percent);
        }else if(Lift_angle>UP_LIMIT-5){
          Lift_v=fmin(Lift_v,40);
          Lift.setMaxTorque(50, percent);
        }
        else
          Lift.setMaxTorque(100, percent);
      }
      if(fabs(Lift_v)>5)
      {
        Lift.setVelocity(Lift_v, percent);
        Lift.spin(fwd);
      }
      else
      {
        Lift.stop(hold);
      }
      int clawspeed;
      if(Controller1.ButtonR1.pressing()==1 && Controller1.ButtonR2.pressing()==0) clawspeed=100;
      if(Controller1.ButtonR1.pressing()==0 && Controller1.ButtonR2.pressing()==1) clawspeed=-100;
      if(Controller1.ButtonR1.pressing()==0 && Controller1.ButtonR2.pressing()==0) clawspeed=0;
      DownClaw.setMaxTorque(100,percent); DownClaw.setVelocity(clawspeed,percent);
      if (clawspeed<0 && (fabs(DownClaw.position(degrees))>=130.0))DownClaw.stop(brake);
      else if (clawspeed>0 && fabs(DownClaw.position(degrees))<=0.0) DownClaw.stop(brake);
      else DownClaw.spin(forward); 
      int clawspeedup=0;
      if(Controller1.ButtonA.pressing()==1 && Controller1.ButtonLeft.pressing()==0) clawspeedup=-100;
      if(Controller1.ButtonA.pressing()==0 && Controller1.ButtonLeft.pressing()==1) clawspeedup=100;
      if(Controller1.ButtonA.pressing()==0 && Controller1.ButtonLeft.pressing()==0) clawspeedup=0;
      UpClaw.setMaxTorque(100,percent); UpClaw.setVelocity(clawspeedup,percent);
      if (clawspeedup<0 && (fabs(UpClaw.position(degrees))>130.0)) UpClaw.stop(brake);
      else UpClaw.spin(fwd);
      int backclawspeed;
      if(Controller1.ButtonDown.pressing()&& Controller1.ButtonB.pressing()==0) backclawspeed = 100;
      if(Controller1.ButtonDown.pressing()==0 && Controller1.ButtonB.pressing()) backclawspeed = -100;
      if(Controller1.ButtonDown.pressing()==0 && Controller1.ButtonB.pressing()==0) backclawspeed =0;
      BackClaw.setMaxTorque(100, percent);BackClaw.setVelocity(backclawspeed, percent); BackClaw.spin(fwd);
      // if(backclawspeed<0 && )
      // if  (Controller1.ButtonY.pressing()) c_up_lift();
      // if (Controller1.ButtonB.pressing())c_down_lift();
      // // if  (Controller1.ButtonA.pressing())  c_up_lift();
      // if (Controller1.ButtonB.pressing()) c_down_lift();
      // if  (Controller1.ButtonUp.pressing()) c_fast_up_clawopen();
      // if (Controller1.ButtonDown.pressing()) c_fast_down_clawopen();
      // // if (Controller1.ButtonLeft.pressing()) c_fast_up_clawclose发的这个就，码农，G｜Eet而她沟通好();
      // if (Controller1.ButtonRight.pressing()) c_fast_down_clawclose();
      }
      // wait before repeating the process
      if(Controller1.ButtonDown.pressing()&&Controller1.ButtonRight.pressing()){//陀螺仪+爪子编码器初始化
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
      if(Controller1.ButtonUp.pressing()){
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
  }
  return 0;
}
void vexcodeInit( void ) {
  first=true;
  task driver_print(printing);
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
  wait(100, msec);
  Brain.Screen.clearScreen();
}