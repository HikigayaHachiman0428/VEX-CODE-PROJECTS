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

motor LF = motor(PORT20, ratio18_1, false);             motor RF = motor(PORT9, ratio18_1,true);
motor LM = motor(PORT10, ratio18_1, false);             motor RM = motor(PORT2, ratio18_1,true);
motor LB = motor(PORT11, ratio18_1, false);              motor RB = motor(PORT19, ratio18_1, true); 
motor_group LeftDriveSmart = motor_group(LF,LB,LM);        motor_group RightDriveSmart = motor_group(RF,RB,RM);

motor FrontClaw = motor(PORT1, ratio18_1,false);          
motor BackClaw = motor(PORT12, ratio18_1,true); 
motor_group Claw_Group=motor_group(FrontClaw,BackClaw);
   

inertial Inert_Constant = inertial(PORT8);


bool RemoteControlCodeEnabled = true;
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;
bool isMovingL = true;
bool isMovingR = true;
bool lowtorque =true;
bool show_claw=false;
bool detect_ball_down;
bool detect_ball_up;
int hue_down;
int hue_up;
int up_is_blue;//1是蓝，0是红，-1是没
int down_is_blue;//1是蓝，0是红，-1是没
int myplaycolor=0;//蓝方是1，红方是0
bool HOLD=false;
bool RUSH=false;
const int MIN=-1200;

timer drivertime;
int i=500000;

// define a task that will handle monitoring inputs from Controller1


bool first=true;
double real_LClawV,real_RClawV,ideal_LClawV=100,ideal_RClawV=100,LClaw_torque,ideal_RClaw_torque,LClaw_temperature,RClaw_temperature;

bool front =true;
bool claw_moving=false;

int printing(){

  while(true)
    if((drivertime>100&&!Competition.isAutonomous())||(Competition.isAutonomous()&&drivertime>400)){
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      // if(front)Controller1.Screen.print("front");else Controller1.Screen.print("reverse");
      // Controller1.Screen.newLine();
      Controller1.Screen.print("F: ");
      Controller1.Screen.print(FrontClaw.position(deg));
      Controller1.Screen.print("  ");
      Controller1.Screen.print("B: ");
      Controller1.Screen.print(BackClaw.position(deg));
      
      Controller1.Screen.newLine();
      Controller1.Screen.print(Brain.Battery.voltage());
      Controller1.Screen.print("V  ");

      Controller1.Screen.newLine();
      Controller1.Screen.print(Inert_Constant.heading());
      Controller1.Screen.print("degrees");
      Brain.Screen.clearScreen();
      Brain.Screen.setCursor(5, 20);
      
      Brain.Screen.print("angle");
      Brain.Screen.print(Inert_Constant.heading());
      Brain.Screen.print("    ");
      
      
      drivertime.reset();
    }
  return 0;
}

int Lowest_CLaw(){
  claw_moving=true;
  if(front){
    FrontClaw.setVelocity(-100, percent);
    FrontClaw.setMaxTorque(100, percent);
  }else{
    BackClaw.setVelocity(-100, percent);
    BackClaw.setMaxTorque(100, percent);
  }
  while(front?FrontClaw.position(deg):BackClaw.position(deg)>MIN+50){
    wait(5,msec);
  }
  if(front){
    FrontClaw.stop(coast);
  }else 
    BackClaw.stop(coast);
  claw_moving=false;


  return 0;

}
int rc_auto_loop_function_Controller1() {
  
  task driver_print(printing);
  timer drivetraintimer;
  while(true) {
    
    
      
      double Go_V,Turn_V;
      double Left_Horizontal,Right_Horizontal,Left_Verticle,Right_verticle;
  
      if(true){//无实际用途,初始数据赋值
        Left_Horizontal=Controller1.Axis4.position();   Right_Horizontal=Controller1.Axis1.position();
        Left_Verticle=Controller1.Axis3.position();     Right_verticle=Controller1.Axis2.position();

        double ClawV=Right_verticle;
        if(fabs(Right_Horizontal)<30&&fabs(ClawV)>25) Right_Horizontal=0;
        Go_V=Left_Verticle;
        Turn_V=Left_Horizontal+Right_Horizontal;
        //Turn_V=Turn_V*(0.25+fabs(Turn_V)/150.0);
        //Turn_V=Turn_V*(0.35+pow(fabs(Turn_V),5)/16889971532);//1.69+e10=pow(150,4.7)  (30,10)(50,20)(81,50)加速度中等
        Turn_V=Turn_V*(0.1+pow(fabs(Turn_V),4.2)/306729757);//3.06+e8=pow(150,3.9) (60,10)(88,50)加速度慢
        //Turn_V=Turn_V*(0.62+pow(fabs(Turn_V),5.8)/930040636712);//9.3+e11=pow(150,5.5) (30,20)(70,50)(90,80)加速度快
         
        
        if(fabs(ClawV)<10)ClawV=0;
        if(fabs(ClawV)>70)ClawV*=2;
        if(fabs(Turn_V)<5) Turn_V=0;
        ClawV*=1.4;
        double Lv = Go_V + Turn_V;
        double Rv = Go_V - Turn_V;
        LF.spin(forward);LM.spin(forward); LB.spin(forward);RF.spin(forward);RM.spin(forward);RB.spin(forward);
        LF.setMaxTorque(100, percent); RF.setMaxTorque(100, percent);
        LM.setMaxTorque(100, percent); RM.setMaxTorque(100, percent);
        LB.setMaxTorque(100, percent); RB.setMaxTorque(100, percent);
        if(front){
          LF.setVelocity(-Rv,percent); LM.setVelocity(-Rv,percent);LB.setVelocity(-Rv,percent);
          RF.setVelocity(-Lv,percent); RM.setVelocity(-Lv,percent);RB.setVelocity(-Lv,percent);
          if(claw_moving){
          }else if(Controller1.ButtonDown.pressing()){
            task moving_down(Lowest_CLaw);
          }else if(ClawV<-5){
            BackClaw.setVelocity(ClawV, percent);
            BackClaw.setMaxTorque(100,percent);
            BackClaw.spin(forward);
          }else if(ClawV>5&&!(BackClaw.position(deg)>-30)){
            BackClaw.setVelocity(ClawV, percent);
            BackClaw.setMaxTorque(100,percent);
            BackClaw.spin(forward);
          } else
            BackClaw.stop(brake);

          if (Controller1.ButtonR1.pressing()&&!(FrontClaw.position(deg)>-30)) {
            FrontClaw.setVelocity(100, percent);
            FrontClaw.setMaxTorque(100, percent);
            FrontClaw.spin(forward);
          } else if (Controller1.ButtonR2.pressing()) {
            FrontClaw.setVelocity(-100, percent);
            FrontClaw.setMaxTorque(100, percent);
            FrontClaw.spin(forward);
          }else {
            FrontClaw.stop(brake);
          }


        }else{
          
          LF.setVelocity(Lv,percent); LM.setVelocity(Lv,percent);LB.setVelocity(Lv,percent);
          RF.setVelocity(Rv,percent); RM.setVelocity(Rv,percent);RB.setVelocity(Rv,percent);
          if(claw_moving){
          }else if(Controller1.ButtonDown.pressing()){
            task moving_down(Lowest_CLaw);
          }else if(ClawV<-5){
            FrontClaw.setVelocity(ClawV, percent);
            FrontClaw.setMaxTorque(100,percent);
            FrontClaw.spin(forward);
          }else if(ClawV>5&&!(FrontClaw.position(deg)>-30)){
            FrontClaw.setVelocity(ClawV, percent);
            FrontClaw.setMaxTorque(100,percent);
            FrontClaw.spin(forward);
          } else 
            FrontClaw.stop(brake);

          if (Controller1.ButtonR1.pressing()&&!(BackClaw.position(deg)>-30)) {
            BackClaw.setVelocity(100, percent);
            BackClaw.setMaxTorque(100, percent);
            BackClaw.spin(forward);
          } else if (Controller1.ButtonR2.pressing()) {
            BackClaw.setVelocity(-100, percent);
            BackClaw.setMaxTorque(100, percent);
            BackClaw.spin(forward);
          }else {
            BackClaw.stop(brake);
          }
        }
      } 

    
      if(Controller1.ButtonL1.pressing()){
        front=true;
        wait(50,msec);
      }
      if(Controller1.ButtonL2.pressing()){
        front=false;
        wait(50,msec);
    }

    // wait before repeating the process
    if(Controller1.ButtonDown.pressing()&&Controller1.ButtonRight.pressing()){//陀螺仪初始化
      Inert_Constant.calibrate();
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print("calibrating");
      while(Inert_Constant.isCalibrating()){
        wait(100,msec);
      }
      Inert_Constant.setHeading(180,deg);
      wait(100,msec);
      Controller1.Screen.clearScreen();
    }
    if(Controller1.ButtonUp.pressing()&&!(Controller1.ButtonRight.pressing()))
      Inert_Constant.setHeading(180, deg);
    
    if(Controller1.ButtonX.pressing())//测试自动
    {
      first=true;
      PlayAuto=true;
      Competition.test_auton();

      while(PlayAuto) wait(50,msec);
    }
    if(Controller1.ButtonY.pressing())//测试自动
    {
      show_claw=!show_claw;

    }

    if(Controller1.ButtonLeft.pressing()&&Controller1.ButtonA.pressing()){
      myplaycolor=myplaycolor==0?1:0;
      wait(200,msec);
    }
    wait(20, msec);
  }
  return 0;
  
} 
void vexcodeInit( void ) {
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  wait(200, msec);
  Inert_Constant.calibrate();
  Brain.Screen.print("Calibrating Gyro for Drivetrain");
  while (Inert_Constant.isCalibrating()){
    wait(25, msec);
  }
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1,1);


  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
  wait(50, msec);
  Brain.Screen.clearScreen();
}