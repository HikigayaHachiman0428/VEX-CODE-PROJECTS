# VEX VRC code version 2.7

## 直接使用

操作手直接下载git，然后在v5code里打开下载至手柄使用即可。如需键位调整请微信联系

## 软件组调整

*src*文件夹内部为`main`和`robot configuration`，分别为主驱动和机器端口匹配。若操作手需要调整 <u>手动建位</u>，则在`main`中修改`usercontrol`函数；若是新机器，则在`robot configuration`内调整马达端口配对。

*include*文件夹中:

1. `vex.h`为官方头文件，且需匹配机器ID。机器ID普遍标记在主控后方贴纸上，若没有请联系硬件组。
2. `robot-config.h`中的`extern`需对应 *src* 中的`robot configuration`。
3. `basic functions.h`; `my-gyro.h`;`my-timer.h`都是底层函数和对于官方提供的函数的包装，直接使用即可。
4. `auton-functions.h`和`autonomous.h`都是根据赛题所设计的自动函数和自动路线，需要根据赛季进行调整。
5. `PID.h`内PID控制的逻辑应用和运动函数，`parameters.h`为PID内的参数。***请不要修改`PID.h`,否则可能会损坏机器！***。每一台新机器都需要对`parameters.h`进行调整来适配机器性能。