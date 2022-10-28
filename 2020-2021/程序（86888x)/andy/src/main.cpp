/*
  @drawer_cat(action;sensing;setting;constructor):void行为;Accesser返回数值;mutator改变数值;创造一个instance
  @brief 简单介绍
  @param 行参定义
*/
#include "vex.h"
#include "cmath"
#include <iostream> 
#include <string> 
  using namespace std;
  using namespace vex;
  competition Competition;

  void pre_auton(void) {
    //vexcodeInit();
  }
  const double TIME_TO_ENDURE=50;//多长时间改变一次Yaw的影响（避免疯狂扭头）
  const double Kencoder=0;  //0.025;
  const double KGyro=2.5;
  double Yaw;
  double Gyro_Started;
  timer Go_timer;

  
  const double RKP=0.14;
  const double RKI=1;
  const double RKP_FAST=0.7;
  const double RKI_FAST=0.4;
  const double RKP_SMALL=0.6;
  const double RKI_SMALL=0.3;
  const double RKP_SMALL_FAST=0.7;
  const double RKI_SMALL_FAST=0.6;
  const double RKP_BIG=0.7;
  const double RKI_BIG=0.6;
  const double RKP_BIG_FAST=0.8;
  const double RKI_BIG_FAST=0.8;



  timer TURN_TIMER;
  timer TURN_TIMER2;
/**
     * @drawer_cat{action}
     * @brief Sets the cursor to the row and column number set in the parameters.
     * @param num set the number which is going to be printed
     */


//外层函数
  //基础模块
    timer auto_time;
    int sgn               (double num){return (num>=0)?1:-1;}//正负性判断，返回值
    double av_LEncoder    (){return (LF.position(degrees)+LB.position(degrees))/2;} 
    double av_REncoder    (){return (RF.position(degrees)+RB.position(degrees))/2;} 
    double av_Encoder     (){return (av_LEncoder()+av_REncoder())/2;}    
    double CalYaw         (){Yaw=Kencoder*(av_LEncoder()-av_REncoder())+KGyro*(Inert_Constant.heading()-Gyro_Started);return Yaw;}//顺时针偏转多少——右轮可以变快多少
    void wait             (int time){wait(time,msec);}
    void INERT_PRINT      (){Controller1.Screen.clearScreen();Controller1.Screen.setCursor(1, 1);Controller1.Screen.print(Inert_Constant.heading());Controller1.Screen.newLine(); Controller1.Screen.print((int)auto_time.time(msec)/1000);Controller1.Screen.print("s");Controller1.Screen.newLine();Controller1.Screen.print(Brain.Battery.voltage());Controller1.Screen.print("V");}
    void set_Gyro         (){Gyro_Started=Inert_Constant.heading();}//陀螺仪设成现在面朝方向
    void set_Gyro         (double angle){Gyro_Started=angle;}//陀螺仪设成传递数值
    void set_encoder      (double value){LeftDriveSmart.setPosition(value,degrees); RightDriveSmart.setPosition(value,degrees);}
    void set_encoder      (){set_encoder(0);}


    void set_torque       (double tq1,double tq2,double tq3,double tq4){LF.setMaxTorque(tq1,percent); RF.setMaxTorque(tq2, percent); LB.setMaxTorque(tq3, percent); RB.setMaxTorque(tq4,percent);}
    void set_torque       (double tq){set_torque(tq,tq,tq,tq);}
    void set_V            (double LF_V,double RF_V,double LB_V,double RB_V){LF.spin(fwd,LF_V,percent);RF.spin(fwd,RF_V,percent);LB.spin(fwd,LB_V,percent);RB.spin(fwd,RB_V,percent);}
    void set_V            (double LV,double RV){set_V(LV, RV, LV, RV);}
    void set_V            (double V){set_V(V,V,V,V);}
  
    void shift_V          (double V){set_V(V,-V,-V,V);}//正值向右平移
    void shift_V          (double V,double error){set_V(V+sgn(V)*error,-V-sgn(V)*error,-V+sgn(V)*error,V-sgn(V)*error);}//顺时针补偿

    void stop             (brakeType manner){LeftDriveSmart.stop(manner);RightDriveSmart.stop(manner);}
    void stop_all(brakeType manner){
      stop(manner);
      Claw_Group.setVelocity(0,percent);  Claw_Group.stop(manner);
      Gun.setVelocity(0,percent);  Gun.stop(manner);}
   

  
  //无精准要求行为
    void zhuangqiang(double velocity,double time){
      set_torque(fabs(velocity));
      set_V(velocity);
      wait(time);
      set_torque(100);
      stop(brake);
      wait(5);
      set_encoder(); }
    void zhuangqiang(double velocity,double time,bool slide){  
      set_torque(fabs(velocity));
      set_V(velocity);
      wait(time);
      set_torque(100);
      stop(slide?coast:brake);
      wait(5);
      set_encoder();}
    void stop_slowly(int time){
      double LF_V=LF.velocity(percent),RF_V=RF.velocity(percent),LB_V=LB.velocity(percent),RB_V=RB.velocity(percent);
      int T=time/10;double per;
      for(double i=0;i<T;i++){
        per=1-i/T;
        set_V(LF_V*per,RF_V*per,LB_V*per,RB_V*per);
        wait(10);
      }
      stop(brake);
      wait(10);
      set_encoder();}
    void basic_move(double velocity,double Enco){//电机不停
      set_encoder();set_V(velocity);
      while(fabs(av_LEncoder())<Enco){wait(20);}}
    void basic_turn(double velocity,double angle){//电机不停
      double init=Inert_Constant.heading();set_encoder();set_V(velocity,-velocity);
      while(fabs(init-Inert_Constant.heading())<angle){wait(2);}}
    void Go_move(double velocity,double distance,double torque,double decay){
      set_torque(torque);set_V(velocity,velocity);
      while(fabs(av_Encoder())<fabs(distance)){
        if(Go_timer.time(msec)>TIME_TO_ENDURE){CalYaw();set_V(velocity-Yaw*decay,velocity+Yaw*decay);Go_timer.reset();}
        wait(5);
      }}

  //Go要求：distance>800(mm)
    //Go 参数
      const int num_of_status=4;
      string Go_manner="normal";
      void set_Go_manner(string manner){Go_manner=manner;}
      void set_Go_manner(){Go_manner="normal";}
      
      //normal
        double v_normal[]={1,0.6,0.24,0.12};
        double d_normal[]={0.7,0.85,0.92,0.98};
        double t_normal[]={100,80,40,40};
        double dec_normal[]={1,0.35,0.08,0};
      
      //fast
        double v_fast[]={1,0.7,0.4,0.25};
        double d_fast[]={0.72,0.85,0.92,0.98};
        double t_fast[]={100,80,40,40};
        double dec_fast[]={1,0.45,0.13,0};
        
      //slow
        double v_slow[]={1,0.7,0.5,0.33};
        double d_slow[]={0.5,0.7,0.85,0.98};
        double t_slow[]={100,80,40,40};
        double dec_slow[]={1,0.2,0.05,0};

    void Go_logic(double velocity,double displacement){
      displacement*=0.52;//距离优化系数，1000是一米
      double v[num_of_status],d[num_of_status],t[num_of_status],dec[num_of_status];
      if(Go_manner=="normal")for(int i=0;i<num_of_status;i++){v[i]=v_normal[i];d[i]=d_normal[i];t[i]=t_normal[i];dec[i]=dec_normal[i];}
      else if (Go_manner=="fast")for(int i=0;i<num_of_status;i++){v[i]=v_fast[i];d[i]=d_fast[i];t[i]=t_fast[i];dec[i]=dec_fast[i];}
      else if (Go_manner=="slow")for(int i=0;i<num_of_status;i++){v[i]=v_slow[i];d[i]=d_slow[i];t[i]=t_slow[i];dec[i]=dec_slow[i];}
      bool Far=displacement>450;
      int bl=sgn(velocity);         
      //启动阶段
        set_torque(Far?30:20);basic_move(10*bl,Far?50:25);
        set_torque(Far?45:30);basic_move(40*bl,Far?50:25);
        set_torque(Far?70:50);basic_move(55*bl,Far?80:35);
      //高速阶段
        for(int i=0;i<num_of_status;i++)Go_move(velocity*v[i],displacement*d[i],t[i],dec[i]);
      //刹车阶段
        set_torque(100);
        stop(brake);
        wait(5);
        set_encoder();  
    }
    void Go(double velocity,double distance,double DEG){
      //初步处理数据
        int bl=sgn(velocity)*sgn(distance);velocity=fabs(velocity)*bl;distance=fabs(distance)*bl;
        velocity=fabs(velocity)>90? 90*bl:velocity; 
        set_Gyro(DEG);                
      Go_logic(velocity,distance);
    }
    void Go(double velocity,double distance){
      Go(velocity,distance,Inert_Constant.heading());
    }
    void Go_fast(double velocity,double distance,double DEG){
      //初步处理数据
        
        int bl=sgn(velocity)*sgn(distance);velocity=fabs(velocity)*bl;distance=fabs(distance)*bl;
        velocity=fabs(velocity)>95? 95*bl:velocity; 
        set_Gyro(DEG);
        set_Go_manner("fast");     
      Go_logic(velocity,distance);
      set_Go_manner();
    }
    void Go_fast(double velocity,double distance){
      Go_fast(velocity,distance,Inert_Constant.heading());
    }

    void Go_slow(double velocity,double distance,double DEG){
      //初步处理数据
        int bl=sgn(velocity)*sgn(distance);velocity=fabs(velocity)*bl;distance=fabs(distance)*bl;
        velocity=fabs(velocity)>80? 80*bl:velocity; 
        set_Gyro(DEG);
        set_Go_manner("slow");     
      Go_logic(velocity,distance);
      set_Go_manner();
    }
    void Go_slow(double velocity,double distance){
      Go_slow(velocity,distance,Inert_Constant.heading());
    }

  
  
  
  
  
  //turn
    bool fast_turn=false;

    void turnto_logic(double velocity,double angle){
      bool Far=fabs(angle)>60;
      int bl=sgn(velocity); 
      double speed,error,turned;
      set_Gyro(); 
      //启动阶段
        set_torque(Far?25:25);basic_turn(25*bl,Far?4:2);
        set_torque(Far?40:30);basic_turn(45*bl,Far?7:4.5);
        set_torque(Far?60:40);basic_turn(70*bl,Far?10:6.5);
        set_torque(100);
      double thisKP,thisKI;
      if(fabs(angle)<60){
        thisKP=fast_turn?RKP_SMALL_FAST:RKP_SMALL;
        thisKI=fast_turn?RKI_SMALL_FAST:RKI_SMALL;
      }
      else if (fabs(angle)<110) {
        thisKP=fast_turn?RKP_FAST:RKP;
        thisKI=fast_turn?RKI_FAST:RKI;
      }
      else {
        thisKP=fast_turn?RKP_BIG_FAST:RKP_BIG;
        thisKI=fast_turn?RKI_BIG_FAST:RKI_BIG;
      }
      
      while(1){
        turned=Inert_Constant.heading(degrees)-Gyro_Started;//已经转了角度
        //turned+=turned<-180?360:0;
        //turned-=turned>180?360:0;
        error=angle-turned;//还需要转的角度
        bl=sgn(error);
        error*=bl;
        speed=error*thisKP;//90
        if(error>50)//70 
          speed+=45;
        else if(error>30)//51
          speed+=16; 
        else{speed+=2; speed*=thisKI;}//25
        speed=bl*min(speed,fabs(velocity));
        set_V(speed,-speed);
        if(error<1){if(TURN_TIMER.time()>50)break;}else TURN_TIMER.clear();
        wait(5,msec);
      }
      set_torque(100);
      stop(brake);
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1, 1);
      Controller1.Screen.print("doneeeeeeee");
      wait(400);

      fast_turn=false;
    }
    void turnto(double velocity, double angle){//默认90度时的转弯2(转到angle度，正值顺时针)
      angle-=Inert_Constant.heading();
      angle+=angle<-180?360:0;
      angle-=angle>180?360:0;
      set_encoder(); set_Gyro();
      int bl=sgn(angle);
      angle=fabs(angle)*bl*0.99;//陀螺仪误差
      velocity=fabs(velocity)*bl;   
      turnto_logic(velocity, angle); 
    }
    void turnto(double velocity, double angle,bool fast){
      fast_turn=fast;
      turnto(velocity,angle);
      fast_turn=false;
    }


  //功能性机构
    void Claw(double velocity,double time){
      double rotate1=time*72000/60000;//时间和转角的转换200rpm
      Claw_Group.rotateFor(fwd,rotate1,deg,velocity*2,rpm,false);
    }
    void Claw(double velocity){
      LClaw.setVelocity(velocity,percent);RClaw.setVelocity(velocity,percent);
      Claw_Group.setMaxTorque(100,percent);
      LClaw.spin(fwd);RClaw.spin(fwd);
    }
    void Claw_torque(double q){
      LClaw.setMaxTorque(q,percent);RClaw.setMaxTorque(q,percent);
    }

    void Tu(double velocity,double time){
      double rotate1=time*72000/60000;//时间和转角的转换200rpm
      double rotate2=time*216000/60000;//600rpm
      if(velocity>0){
        Gun.rotateFor(fwd,rotate1,deg,velocity*6,rpm,false);//2
        Gun.rotateFor(fwd,rotate2,deg,velocity*6,rpm,false);
      }
      if(velocity<0){
        Gun.rotateFor(reverse,rotate1,deg,-velocity*6,rpm,false);//2
        Gun.rotateFor(reverse,rotate2,deg,-velocity*6,rpm,false);
      }
    }
    void Tu(double velocity){
      Gun.setVelocity(velocity,percent);//Gun2.setVelocity(100,percent);
      Gun.spin(fwd);
      if(velocity==0){Gun.stop(coast);Gun.setMaxTorque(100,percent);}
      
    }
    void Tuwithtorque(double velocity){
      Gun.setVelocity(velocity,percent);//Gun2.setVelocity(100,percent);
      Gun.setMaxTorque(abs(velocity), percent);
      Gun.spin(fwd);
      if(velocity==0){Gun.stop(coast);Gun.setMaxTorque(100,percent);}
      
    }
    void Tu2(){
      Gun.setVelocity(100,percent);  Gun.setVelocity(-100,percent);
      Gun.spin(fwd);
    }
    void Tu_torque(double q){
      Gun.setMaxTorque(q,percent);Gun.setMaxTorque(q,percent);
    }



void Auto(void) {
  timer auto_timer;
  bool COMPILING=true;
  set_encoder();
  if(COMPILING)
    set_Gyro(180);

    turnto(100, 90);
    
  
  set_Gyro(180);
  
  
  
  
}
/*
  void usercontrol() {
  int LFv,LBv,RFv,RBv;
  int Go_V,Turn_V,Shift_V;
  bool SHIFT=false;
  int Left_Horizontal,Right_Horizontal,Left_Verticle,Right_verticle;
  bool DrivetrainNeedsToBeStopped_Controller1 = true;
  Gun.setMaxTorque(100,percent);Claw_Group.setMaxTorque(100,percent);
  while (1) {
    if(Brain.Timer.value()>0.5){//测试传感器
      INERT_PRINT();
      Brain.Timer.reset();
    }

    if(true){//无实际用途,初始数据赋值
      Left_Horizontal=Controller1.Axis4.position();   Right_Horizontal=Controller1.Axis1.position();
      Left_Verticle=Controller1.Axis3.position();     Right_verticle=Controller1.Axis2.position();
      Go_V=Left_Verticle;
      Turn_V=Right_Horizontal*(abs(Right_Horizontal)<40? 0.3:abs(Right_Horizontal)<88?0.5:1);//turning velocity
      Shift_V=0;//turning velocity
      SHIFT=Controller1.ButtonY.pressing()||Controller1.ButtonA.pressing();
      LFv = Go_V + Turn_V+Shift_V;
      LBv = Go_V + Turn_V-Shift_V;
      RFv = Go_V - Turn_V-Shift_V;
      RBv = Go_V - Turn_V+Shift_V;
    }
    
    LF.spin(fwd);LB.spin(fwd);RF.spin(fwd);RB.spin(fwd);
    if(SHIFT){
      int RIGHT=Controller1.ButtonA.pressing()?1:-1;//是否向右平移
      timer SHIFT_TIMING;
      SHIFT_TIMING.reset();
      while(false){//平移半自动
        if(SHIFT_TIMING.time(msec)<100){
          set_torque(35);
          shift_V(18*RIGHT);
        }else{
          if(Controller1.ButtonUp.pressing()){
            set_torque(75);
            shift_V(80*RIGHT);
          }else{
            set_torque(60);
            shift_V(30*RIGHT,2);
          }
        }
        SHIFT=Controller1.ButtonY.pressing()||Controller1.ButtonA.pressing();
        wait(50);
        
      }
    }else{
      if(abs(Turn_V)<40&&abs(Turn_V)>5){//小幅调整第扭矩
        LF.setMaxTorque(35,percent);
        LB.setMaxTorque(35,percent);
        RF.setMaxTorque(35,percent);
        RB.setMaxTorque(35,percent);
      }else if(abs(Turn_V)<70&&abs(Turn_V)>5){
        LF.setMaxTorque(50,percent);
        LB.setMaxTorque(50,percent);
        RF.setMaxTorque(50,percent);
        RB.setMaxTorque(50,percent);
      }else{
        LF.setMaxTorque(80,percent);
        LB.setMaxTorque(80,percent);
        RF.setMaxTorque(80,percent);
        RB.setMaxTorque(80,percent);
      }
    
      if (abs(Shift_V)<5&&abs(Go_V)<5&&abs(Turn_V)<5) {LeftDriveSmart.stop(brake);   RightDriveSmart.stop(brake);DrivetrainNeedsToBeStopped_Controller1=true;}
      else{LF.spin(fwd);LB.spin(fwd);RF.spin(fwd);RB.spin(fwd);DrivetrainNeedsToBeStopped_Controller1=false;}
      if(!DrivetrainNeedsToBeStopped_Controller1) {
        LF.setVelocity(LFv, percent);
        LB.setVelocity(LBv,percent);
        RF.setVelocity(RFv, percent);
        RB.setVelocity(RBv,percent);
      }
    }
          
      
    

    if(Controller1.ButtonL1.pressing()){//爪子吸球
      Claw_Group.setVelocity(100,percent);
      Claw_Group.spin(fwd);
    }
    if(Controller1.ButtonL2.pressing()){//爪子吐球
      Claw_Group.setVelocity(100,percent);
      Claw_Group.spin(reverse);
    }     
      
    if(Controller1.ButtonR1.pressing()){//向上吐球
       Gun.setVelocity(100,percent);
       Gun.spin(fwd);
      }
    if(Controller1.ButtonR2.pressing()){//向下吐球
        Gun.setVelocity(100,percent);
        Gun.spin(reverse);
    }
          
    if(!(Controller1.ButtonL1.pressing()||Controller1.ButtonL2.pressing()))   Claw_Group.stop(coast);
    if(!(Controller1.ButtonR1.pressing()||Controller1.ButtonR2.pressing()))   Gun.stop(coast);
    if(Controller1.ButtonDown.pressing()&&Controller1.ButtonRight.pressing()){//陀螺仪初始化
      Inert_Constant.calibrate();
      Controller1.Screen.clearScreen();
      Controller1.Screen.setCursor(1,1);
      Controller1.Screen.print("calibrating");
      while(Inert_Constant.isCalibrating()){
        wait(100);
      }
      Inert_Constant.setHeading(180,deg);
      wait(100);
      Controller1.Screen.clearScreen();
    }
    if(Controller1.ButtonUp.pressing()&&!(Controller1.ButtonRight.pressing()))
      Inert_Constant.setHeading(180, deg);
    if(Controller1.ButtonX.pressing())    autonomous();//测试自动
      

    wait(20, msec);
  }
  
  
  
  }*/



int main(){
  vexcodeInit();
  Competition.autonomous(Auto);
  //Competition.drivercontrol(usercontrol);
  pre_auton();
  while (true){
    wait(20, msec);
    if(PlayAuto)
      Auto();
  }
}
