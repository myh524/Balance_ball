//
// Created by sytnocui on 2025/1/10.
//

#ifndef BALANCEBALL_ROBOROLY_H
#define BALANCEBALL_ROBOROLY_H


extern const float h; //步长 单位s
extern const float t_d; //周期 单位s
extern const float f_d; //频率 单位 1/s
extern const float w_d ; //单位：rad/s


extern float sin_time;
extern float robot_time;
extern const float total_time;
extern const float deg;

void RoboRolyWalkUpdate(); //简化版的，合了之前的setpoint，ctrl和speed update


#endif //BALANCEBALL_ROBOROLY_H
