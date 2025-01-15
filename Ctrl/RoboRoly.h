//
// Created by sytnocui on 2025/1/10.
//

#ifndef BALANCEBALL_ROBOROLY_H
#define BALANCEBALL_ROBOROLY_H


typedef enum
{
    fsm_waitToCalib= 0,
    fsm_waitToStart,
    fsm_roll,
    fsm_walk,
    fsm_stop
} RoboRoly_FSM_e;


extern const float h; //步长 单位s
extern const float t_d; //周期 单位s
extern const float f_d; //频率 单位 1/s
extern const float w_d ; //单位：rad/s


extern float sin_time;
extern float robot_time;
extern float roll_time;
extern float walk_time;


extern const float deg;
extern const float k_c;



extern int roll_period_count;
extern const int roll_total_period;
extern int walk_period_count;
extern const int walk_total_period;
extern const int walk_half_period;

extern const float roll_d_sin_A;
extern const float yaw_d_sin_A;
extern const float roll_d_phase;
extern const float yaw_d_phase;

extern float xd_yaw_pro0;
extern float xd_yaw_pro1;

extern float xd_yaw_turn_offset_0;
extern float xd_yaw_turn_offset_1;

extern float plot_data[6];

extern RoboRoly_FSM_e robot_fsm;

void RoboRolyWalkUpdate(); //简化版的，合了之前的setpoint，ctrl和speed update


#endif //BALANCEBALL_ROBOROLY_H
