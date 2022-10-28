#include "cmath"
#include "vex.h"
#include <iostream>
#include <string>
/*
  @drawer_cat(action;sensing;setting;constructor):void行为;Accesser返回数值;mutator改变数值;创造一个instance
  @brief 简单介绍
  @param 行参定义
  */
/**
 * @drawer_cat{action}
 * @brief Sets the cursor to the row and column number set in the parameters.
 * @param num set the number which is going to be printed
 */

using namespace std;
using namespace vex;
competition Competition;

void pre_auton(void) {}
const double TIME_TO_ENDURE = 50; //多长时间改变一次Yaw的影响（避免疯狂扭头）

double Yaw;
double Gyro_Started;
timer Go_timer;

// const系数
// Go 参数
const int num_of_status = 4;
const double UNIT_RATIO =
    0.39; // to change the unit-----1000represent one block(60cm)
const double Kencoder = 0.007; // 0.025;
const double KGyro = 1.2;
string Go_manner = "normal";
// normal
double v_normal[] = {1, 0.6, 0.24, 0.08};
double d_normal[] = {0.7, 0.85, 0.93, 0.98};
double t_normal[] = {80, 50, 50, 40};
double dec_normal[] = {1, 0.4, 0.16, 0.1};
// fast
double v_fast[] = {1, 0.73, 0.22, 0.1};
double d_fast[] = {0.7, 0.88, 0.97, 1};
double t_fast[] = {80, 60, 60, 45};
double dec_fast[] = {1, 0.4, 0.16, 0.1};
// slow
double v_slow[] = {0.8, 0.55, 0.38, 0.17};
double d_slow[] = {0.5, 0.75, 0.87, 0.98};
double t_slow[] = {75, 40, 60, 40};
double dec_slow[] = {0.7, 0.3, 0.1, 0.08};

//转弯
// 90
const double RKP = 0.45; /// 0.35
const double RKI = 0.75;

const double RKP_FAST = 0.7;
const double RKI_FAST = 0.4;

const int PIDTURN_HIGHSPEED = 60;
const int PIDTURN_MIDDLESPEED = 35;
//临界值

const double PIDTURN_HIGHSPEED_INCREMENT = 17.5;
const double PIDTURN_MIDDLESPEED_INCREMENT = 11.5;
const double PIDTURN_LOWSPEED_INCREMENT = 5;
//初始速度

const int PIDTURN_STOPTIME = 20;
const double PIDTURN_ERROR_ALLOWENCE = 3;

// 45
const double ti_lim = 300;
const double RKP_SMALL = 0.1;
const double RKI_SMALL = 0.15;

const double RKP_SMALL_FAST = 0.7;
const double RKI_SMALL_FAST = 0.4;

const int SMALL_PIDTURN_HIGHSPEED = 41;
const double SMALL_PIDTURN_MIDDLESPEED = 29;

const double SMALL_PIDTURN_HIGHSPEED_INCREMENT = 8.9;
const double SMALL_PIDTURN_MIDDLESPEED_INCREMENT = 6.9;
const double SMALL_PIDTURN_LOWSPEED_INCREMENT = 6.3;

const int SMALL_PIDTURN_STOPTIME = 10;
const double SMALL_PIDTURN_ERROR_ALLOWENCE = 5;

const int SMALL_PIDTURN_STOPTIME_FAST = 500;
const double SMALL_PIDTURN_ERROR_ALLOWENCE_FAST = 0.5;
// 135
const double RKP_BIG = 0.5;
const double RKI_BIG = 0.3;

const double RKP_BIG_FAST = 0.7;
const double RKI_BIG_FAST = 0.4;

const int BIG_PIDTURN_HIGHSPEED = 75;
const int BIG_PIDTURN_MIDDLESPEED = 45;

const int BIG_PIDTURN_HIGHSPEED_INCREMENT = 25;
const int BIG_PIDTURN_MIDDLESPEED_INCREMENT = 17;
const int BIG_PIDTURN_LOWSPEED_INCREMENT = 7;

const int BIG_PIDTURN_STOPTIME = 20;
const double BIG_PIDTURN_ERROR_ALLOWENCE = 5;

const int BIG_PIDTURN_STOPTIME_FAST = 80;
const double BIG_PIDTURN_ERROR_ALLOWENCE_FAST = 1;

timer TURN_TIMER;
timer TURN_TIMER2;

//外层函数
//基础模块
timer auto_time;
int sgn(double num) { return (num >= 0) ? 1 : -1; } //正负性判断，返回值
double av_LEncoder() {
  return (LF.position(degrees) + LB.position(degrees)) / 2;
}
double av_REncoder() {
  return (RF.position(degrees) + RB.position(degrees)) / 2;
}
double av_Encoder() { return (av_LEncoder() + av_REncoder()) / 2; }
double av_CLEncoder() {
  return (FrontClaw.position(degrees) + BackClaw.position(degrees)) / 2;
}
double CalYaw() {
  Yaw = Kencoder * (av_LEncoder() - av_REncoder()) +
        KGyro * (Inert_Constant.heading() - Gyro_Started);
  return Yaw;
} //顺时针偏转多少——右轮可以变快多少
void wait(int time) { wait(time, msec); }
void INERT_PRINT() {
  Controller1.Screen.clearScreen();
  Controller1.Screen.setCursor(1, 1);
  Controller1.Screen.print(Inert_Constant.heading());
  Controller1.Screen.newLine();
  Controller1.Screen.print((int)auto_time.time(msec) / 1000);
  Controller1.Screen.print("s");
  Controller1.Screen.newLine();
  Controller1.Screen.print(Brain.Battery.voltage());
  Controller1.Screen.print("V");
}
void set_Gyro() {
  Gyro_Started = Inert_Constant.heading();
} //陀螺仪设成现在面朝方向
void set_Gyro(double angle) { Gyro_Started = angle; } //陀螺仪设成传递数值
void set_encoder(double value) {
  LeftDriveSmart.setPosition(value, degrees);
  RightDriveSmart.setPosition(value, degrees);
}
void set_encoder() { set_encoder(0); }

void set_torque(double tq1, double tq2, double tq3, double tq4, double tq5,
                double tq6) {
  LF.setMaxTorque(tq1, percent);
  RF.setMaxTorque(tq2, percent);
  LB.setMaxTorque(tq3, percent);
  RB.setMaxTorque(tq4, percent);
  LM.setMaxTorque(tq5, percent);
  RM.setMaxTorque(tq6, percent);
}
void set_torque(double tq) { set_torque(tq, tq, tq, tq, tq, tq); }
void set_torque(double Ltq, double Rtq) {
  set_torque(Ltq, Rtq, Ltq, Rtq, Ltq, Rtq);
}
void set_V(double LF_V, double RF_V, double LB_V, double RB_V, double LM_V,
           double RM_V) {
  LF.spin(fwd, LF_V, percent);
  RF.spin(fwd, RF_V, percent);
  LB.spin(fwd, LB_V, percent);
  RB.spin(fwd, RB_V, percent);
  LM.spin(fwd, LM_V, percent);
  RM.spin(fwd, RM_V, percent);
}
void set_V(double LV, double RV) { set_V(LV, RV, LV, RV, LV, RV); }
void set_V(double V) { set_V(V, V, V, V, V, V); }

// void shift_V          (double V){set_V(V,-V,-V,V);}//正值向右平移
// void shift_V          (double V,double
// error){set_V(V+sgn(V)*error,-V-sgn(V)*error,-V+sgn(V)*error,V-sgn(V)*error);}//顺时针补偿

void stop(brakeType manner) {
  LeftDriveSmart.stop(manner);
  RightDriveSmart.stop(manner);
}
void stop_all(brakeType manner) {
  stop(manner);
  Claw_Group.setVelocity(0, percent);
  Claw_Group.stop(manner);
}

//无精准要求行为
void basic_zhuangqiang(double velocity, double time) {
  set_torque(fabs(velocity));
  set_V(velocity);
  wait(time);
}
void zhuangqiang(double velocity, double time) {
  set_torque(fabs(velocity));
  set_V(velocity);
  wait(time);
  set_torque(100);
  stop(brake);
  wait(5);
  set_encoder();
}
void zhuangqiang(double velocity, double time, bool slide) {
  set_torque(fabs(velocity));
  set_V(velocity);
  wait(time);
  set_torque(100);
  stop(slide ? coast : brake);
  wait(5);
  set_encoder();
}
void stop_slowly(int time) {
  double LF_V = LF.velocity(percent), RF_V = RF.velocity(percent),
         LB_V = LB.velocity(percent), RB_V = RB.velocity(percent),
         LM_V = LM.velocity(percent), RM_V = RM.velocity(percent);
  int T = time / 10;
  double per;
  for (double i = 0; i < T; i++) {
    per = 1 - i / T;
    set_V(LF_V * per, RF_V * per, LB_V * per, RB_V * per, LM_V * per,
          RM_V * per);
    wait(10);
  }
  stop(brake);
  wait(10);
  set_encoder();
}

void basic_turn(double velocity, double angle) { //电机不停
  double init = Inert_Constant.heading();
  set_encoder();
  set_V(velocity, -velocity);
  while (fabs(init - Inert_Constant.heading()) < angle) {
    wait(2);
  }
}

// Go要求：distance>800(mm)
void set_Go_manner(string manner) { Go_manner = manner; }
void set_Go_manner() { Go_manner = "normal"; }
void basic_move(double velocity, double Enco) { //电机不停
  Enco *= UNIT_RATIO;
  set_encoder();
  set_V(velocity * 1, velocity * 1);
  while (fabs(av_LEncoder()) < Enco) {
    wait(20);
  }
}
void Go_move(double velocity, double distance, double torque, double decay) {
  set_torque(torque);
  set_V(velocity, velocity);
  while (fabs(av_Encoder()) < fabs(distance)) {
    if (Go_timer.time(msec) > TIME_TO_ENDURE) {
      CalYaw();
      set_V(velocity - Yaw * decay, velocity + Yaw * decay);
      Go_timer.reset();
    }
    wait(5);
  }
}
void Go_logic(double velocity, double displacement) {
  displacement *= UNIT_RATIO; //距离优化系数，1000是一米
  bool Far = displacement > 1300;
  double v[num_of_status], d[num_of_status], t[num_of_status],
      dec[num_of_status];
  if (Go_manner == "normal")
    for (int i = 0; i < num_of_status; i++) {
      v[i] = v_normal[i];
      d[i] = d_normal[i];
      t[i] = t_normal[i];
      dec[i] = dec_normal[i];
    }
  else if (Go_manner == "fast")
    for (int i = 0; i < num_of_status; i++) {
      v[i] = v_fast[i];
      d[i] = d_fast[i];
      t[i] = t_fast[i];
      dec[i] = dec_fast[i];
    }
  else if (Go_manner == "slow")
    for (int i = 0; i < num_of_status; i++) {
      v[i] = v_slow[i];
      d[i] = d_slow[i];
      t[i] = t_slow[i];
      dec[i] = dec_slow[i];
    }

  int bl = sgn(velocity);
  //启动阶段
  set_torque(Far ? 30 : 13, Far ? 30 : 20);
  basic_move(20 * bl, Far ? 50 : 25);
  set_torque(Far ? 55 : 35, Far ? 55 : 35);
  basic_move(45 * bl, Far ? 50 : 25);
  set_torque(Far ? 80 : 55, Far ? 80 : 55);
  basic_move(60 * bl, Far ? 80 : 35);
  //高速阶段
  for (int i = 0; i < num_of_status; i++)
    Go_move(velocity * v[i], displacement * d[i], t[i], dec[i]);
  //刹车阶段
  set_V(0);
  stop(brake);
  set_torque(100);
  wait(50);
  set_encoder();
}
void Go(double velocity, double distance, double DEG) {
  //初步处理数据
  int bl = sgn(velocity) * sgn(distance);
  velocity = fabs(velocity) * bl;
  distance = fabs(distance) * bl;
  velocity = fabs(velocity) > 90 ? 90 * bl : velocity;
  set_Gyro(DEG);
  Go_logic(velocity, distance);
}
void Go(double velocity, double distance) {
  Go(velocity * 0.9, distance, Inert_Constant.heading());
}
void Go_fast(double velocity, double distance, double DEG) {
  //初步处理数据
  int bl = sgn(velocity) * sgn(distance);
  velocity = fabs(velocity) * bl;
  distance = fabs(distance) * bl;
  velocity = fabs(velocity) > 95 ? 95 * bl : velocity;
  set_Gyro(DEG);
  set_Go_manner("fast");
  Go_logic(velocity, distance);
  set_Go_manner();
}
void Go_fast(double velocity, double distance) {
  Go_fast(velocity, distance, Inert_Constant.heading());
}

void Go_slow(double velocity, double distance, double DEG) {
  //初步处理数据
  int bl = sgn(velocity) * sgn(distance);
  velocity = fabs(velocity) * bl;
  distance = fabs(distance) * bl;
  velocity = fabs(velocity) > 80 ? 80 * bl : velocity;
  set_Gyro(DEG);
  set_Go_manner("slow");
  Go_logic(velocity, distance);
  set_Go_manner();
}
void Go_slow(double velocity, double distance) {
  Go_slow(velocity, distance, Inert_Constant.heading());
}

// turn
double turn_to_angle;
bool fast_turn = false;
double thisKP, thisKI;
double this_HIGHSPEED, this_MIDDLESPEED, this_HIGHSPEED_INCREMENT,
    this_MIDDLESPEED_INCREMENT, this_LOWSPEED_INCREMENT, this_STOPTIME,
    this_ERROR_ALLOWENCE;
double this_turn_time;
void Constant_selecter(double error) {
  if (error < 60) {
    Controller1.Screen.print(60);
    thisKP = fast_turn ? RKP_SMALL_FAST : RKP_SMALL;
    thisKI = fast_turn ? RKI_SMALL_FAST : RKI_SMALL;
    this_HIGHSPEED = SMALL_PIDTURN_HIGHSPEED;
    this_MIDDLESPEED = SMALL_PIDTURN_MIDDLESPEED;
    this_HIGHSPEED_INCREMENT = SMALL_PIDTURN_HIGHSPEED_INCREMENT;
    this_MIDDLESPEED_INCREMENT = SMALL_PIDTURN_MIDDLESPEED_INCREMENT;
    this_LOWSPEED_INCREMENT = SMALL_PIDTURN_LOWSPEED_INCREMENT;
    this_STOPTIME =
        fast_turn ? SMALL_PIDTURN_STOPTIME : SMALL_PIDTURN_STOPTIME_FAST;
    this_ERROR_ALLOWENCE = fast_turn ? SMALL_PIDTURN_ERROR_ALLOWENCE
                                     : SMALL_PIDTURN_ERROR_ALLOWENCE_FAST;
  } else if (error < 110) {
    Controller1.Screen.print(90);
    thisKP = fast_turn ? RKP_FAST : RKP;
    thisKI = fast_turn ? RKI_FAST : RKI;
    this_HIGHSPEED = PIDTURN_HIGHSPEED;
    this_MIDDLESPEED = PIDTURN_MIDDLESPEED;
    this_HIGHSPEED_INCREMENT = PIDTURN_HIGHSPEED_INCREMENT;
    this_MIDDLESPEED_INCREMENT = PIDTURN_MIDDLESPEED_INCREMENT;
    this_LOWSPEED_INCREMENT = PIDTURN_LOWSPEED_INCREMENT;
    this_STOPTIME = PIDTURN_STOPTIME;
    this_ERROR_ALLOWENCE = PIDTURN_ERROR_ALLOWENCE;
  } else {
    Controller1.Screen.print(error);
    thisKP = fast_turn ? RKP_BIG_FAST : RKP_BIG;
    thisKI = fast_turn ? RKI_BIG_FAST : RKI_BIG;
    this_HIGHSPEED = BIG_PIDTURN_HIGHSPEED;
    this_MIDDLESPEED = BIG_PIDTURN_MIDDLESPEED;
    this_HIGHSPEED_INCREMENT = BIG_PIDTURN_HIGHSPEED_INCREMENT;
    this_MIDDLESPEED_INCREMENT = BIG_PIDTURN_MIDDLESPEED_INCREMENT;
    this_LOWSPEED_INCREMENT = BIG_PIDTURN_LOWSPEED_INCREMENT;
    this_STOPTIME =
        fast_turn ? BIG_PIDTURN_STOPTIME : BIG_PIDTURN_STOPTIME_FAST;
    this_ERROR_ALLOWENCE = fast_turn ? BIG_PIDTURN_ERROR_ALLOWENCE
                                     : BIG_PIDTURN_ERROR_ALLOWENCE_FAST;
  }
}
bool special_turn;
void turnto_logic(double velocity, double angle) {
  timer autoturn_time_limit;
  autoturn_time_limit.reset();
  int bl = sgn(angle);
  bool Far = fabs(angle) > 60;
  double speed, error, turned;
  set_Gyro();
  Constant_selecter(fabs(angle));
  //启动阶段
  set_torque(Far ? 35 : 30);
  basic_turn(20 * bl, Far ? 3.5 : 1);
  set_torque(Far ? 50 : 40);
  basic_turn(35 * bl, Far ? 6.5 : 2);
  set_torque(Far ? 70 : 50);
  basic_turn(50 * bl, Far ? 8.3 : 3);
  set_torque(100);
  TURN_TIMER.clear();
  timer special_timer;
  special_timer.reset();
  while (1) {
    turned = Inert_Constant.heading(degrees) - Gyro_Started; //已经转了角度
    turned += turned < -180 ? 360 : 0;
    turned -= turned > 180 ? 360 : 0;
    error = angle - turned; //还需要转的角度
    bl = sgn(error);
    error *= bl;                //绝对值
    speed = error * thisKP;     // 90
    if (error > this_HIGHSPEED) // 70
      speed += this_HIGHSPEED_INCREMENT;
    else if (error > this_MIDDLESPEED) // 51
      speed += this_MIDDLESPEED_INCREMENT;
    else {
      speed += this_LOWSPEED_INCREMENT;
      speed *= thisKI;
    } // 25
    speed = bl * min(speed, fabs(velocity));
    set_V(speed, -speed);
    if (abs(Inert_Constant.heading() - turn_to_angle) < this_ERROR_ALLOWENCE) {
      if (TURN_TIMER.time() > this_STOPTIME)
        break;
    }
    if (autoturn_time_limit.time() > this_turn_time)
      break;
    wait(10, msec);
    if (special_turn == true && special_timer.time()>500)
    {
      break;
    }
  }
  set_torque(100);
  set_V(0);
  stop(brake);
  wait(100);
}
void turnto_client(double velocity,
                   double angle) { //默认90度时的转弯2(转到angle度，正值顺时针)
  turn_to_angle = angle;
  angle -= Inert_Constant.heading();
  angle += angle < -180 ? 360 : 0;
  angle -= angle > 180 ? 360 : 0;
  set_encoder();
  set_Gyro();
  int bl = sgn(angle);
  angle = fabs(angle) * bl * 1; //陀螺仪误差
  turnto_logic(velocity, angle);
}
void turnto(double velocity, double angle, int ttime) {
  this_turn_time = ttime;
  turnto_client(velocity, angle);
}
void turnto(double velocity, double angle) { turnto(velocity, angle, 250000); }
void turnto(double velocity, double angle, bool fast) {
  fast_turn = fast;
  turnto(velocity, angle);
  fast_turn = false;
}
void turnto(double velocity, double angle, bool fast, int ttime) {
  fast_turn = fast;
  turnto(velocity, angle, ttime);
  fast_turn = false;
}

//功能性机构
void end_auto() {
  Claw_Group.setVelocity(100, percent);
  Claw_Group.setMaxTorque(100, percent);
  stop_all(coast);
  PlayAuto = false;
}

//赛季特别方法
void front_claw(bool up) {
  timer reg_time;
  reg_time.reset();
  int time_lim = 3000;
  int direct = up ? 1 : -1;
  int degree_lim = up ? -100 : -830;
  if (!up)
    while (reg_time.time() < time_lim &&
           FrontClaw.position(degrees) >= degree_lim) {
      FrontClaw.setVelocity(direct * 65, percent);
      FrontClaw.spin(fwd);
    }
  else {
    while (reg_time.time() < time_lim &&
           FrontClaw.position(degrees) <= degree_lim) {
      FrontClaw.setVelocity(direct * 80, percent);
      FrontClaw.spin(fwd);
    }
  }
  FrontClaw.stop(hold);
}

void front_claw( int speed, bool up) {
  timer reg_time;
  reg_time.reset();
  int time_lim = 2000;
  int direct = up ? 1 : -1;
  int degree_lim = up ? -100 : -830;
  if (!up)
    while (reg_time.time() < time_lim &&
           FrontClaw.position(degrees) >= degree_lim) {
      FrontClaw.setVelocity(direct * speed, percent);
      FrontClaw.spin(fwd);
    }
  else {
    while (reg_time.time() < time_lim &&
           FrontClaw.position(degrees) <= degree_lim) {
      FrontClaw.setVelocity(direct * 80, percent);
      FrontClaw.spin(fwd);
    }
  }
  FrontClaw.stop(hold);
}

void front_claw(bool up, int inti) {
  FrontClaw.setPosition(inti, degrees);
  front_claw(up);
}
void back_claw(bool up) {
  timer reg_time;
  reg_time.reset();
  int time_lim = 2000;
  int direct = up ? 1 : -1;
  int degree_lim = up ? -120 : -855;
  if (!up)
    while (reg_time.time() < time_lim &&
           BackClaw.position(degrees) >= degree_lim) {
      BackClaw.setVelocity(direct * 65, percent);
      BackClaw.spin(fwd);
    }
  else {
    while (reg_time.time() < time_lim &&
           BackClaw.position(degrees) <= degree_lim) {
      BackClaw.setVelocity(direct * 80, percent);
      BackClaw.spin(fwd);
    }
  }
  BackClaw.stop(hold);
}
void back_claw(bool up, int inti) {
  BackClaw.setPosition(inti, degrees);
  back_claw(up);
}
// task
int front_claw_down() {
  front_claw(75,false);
  return 0;
}
int back_claw_down() {
  back_claw(0);
  return 0;
}
bool front_claw_holding;
int front_keep_holding(){
  while (front_claw_holding){
    FrontClaw.setVelocity(-30, percent);
    FrontClaw.setMaxTorque(70, percent);
  }
  return 0;
}

bool back_claw_holding;
int back_keep_holding(){
  while (back_claw_holding){
    BackClaw.setVelocity(30, percent);
    BackClaw.setMaxTorque(70, percent);
  }
  return 0;
}

int special_task1(){//small current
  wait(250,msec);
  FrontClaw.setVelocity(-15, percent);
  FrontClaw.setMaxTorque(25, percent);
  FrontClaw.spin(fwd);
  wait(800,msec);
  FrontClaw.setMaxTorque(100, percent);
  FrontClaw.stop(coast);
  return 0;
}
int special_task2(){//small current
    back_claw_down();
  return 0;
}
int special_task3(){//small current
  wait(250,msec);
  BackClaw.setVelocity(-15, percent);
  BackClaw.setMaxTorque(25, percent);
  BackClaw.spin(fwd);
  wait(800,msec);
  BackClaw.setMaxTorque(100, percent);
  BackClaw.stop(coast);
  return 0;
}

void Auto(void) {
  timer auto_timer;
  bool COMPILING = 0;
  set_encoder();
  if (COMPILING) {
    Controller1.Screen.print("compiling");
    while (true) {
      if (Controller1.ButtonA.pressing()) {
        auto_timer.reset();
        while (auto_timer.time() < ti_lim)
          turnto(100, 360);
      }
      if (Controller1.ButtonB.pressing()) {
        auto_timer.reset();
        while (auto_timer.time() < ti_lim)
          turnto(100, 315);
      }
      if (Controller1.ButtonY.pressing())
        end_auto();
    }
    // Go(100,400);
  } else {
    //初始化： 200
    timer auto_special_time;
    task t1(front_claw_down);
    wait(800);
    Go(50,-300);
    wait(300);
    front_claw(0); 
    turnto(50,205);
    wait(100);
    task special1(special_task1);
    Go(80,2300);
    wait(200);
    
    task special2(special_task2);
    front_claw(1);
    set_V(30, 30, 30, 30, 30, 30);
    wait(300,msec);
    stop(brake);
    // front_claw_holding = true;
    // task T1_always(front_keep_holding);
    wait(100);
    turnto(100, 98);
    //task special3(special_task3);
    wait(100);
    Go(60, -1020);
    wait(100);
    back_claw(1);
    back_claw_holding = true;
    // task T2_always(back_keep_holding);
    turnto(100, 180);
    wait(100);
    Go(70,-1300);
    stop_all(hold);
  }
}
int main() {
  vexcodeInit();
  Competition.autonomous(Auto);
  pre_auton();
  // set_encoder();
  while (true) {
    wait(20, msec);
  }
}