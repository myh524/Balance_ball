//
// Created by sytnocui on 2025/1/10.
//

#include "RoboRoly.h"
#include "utils/ctrl_math.h"

#include <math.h>
#include <interface_can.h>
#include <memory.h>
#include "tim.h"
#include "senser_types.h"
#include "attitude.h"
#include "ctrl_types.h"
#include "lqr.h"
#include "utils/attitude_utils.h"
#include "nouse/ctrl.h"


const float h = 0.01f; //步长 单位s
const float t_d = 1.353f; //周期 单位s
const float f_d = 1/t_d; //频率 单位 1/s
const float w_d = 2 * M_PIf * f_d ; //单位：rad/s


float robot_time = 0; //开机计时时间
float roll_time = 0; //roll计时时间
float walk_time = 0; //walk计时时间
float sin_time = 0; //单周期时间，过了固有周期会清零

int roll_period_count = 0;
const int roll_total_period = 5;
int walk_period_count = 0;
const int walk_total_period = 10;
const int walk_half_period = walk_total_period / 2;

const float deg = M_PIf / 180;


const float roll_d_sin_A = 30 * deg;
const float yaw_d_sin_A = 30 * deg;
const float roll_d_phase = -M_PIf;
const float yaw_d_phase = roll_d_phase + M_PIf/2; //暂时不用了

//计算转弯
//const float k_c = 5 * deg; //转弯的速度 单位和仿真的不同，为：deg/周期
const float k_c = 8 * deg; //转弯的速度 单位和仿真的不同，为：deg/周期

//用于记录带转弯的期望的转角
float xd_yaw_pro0 = 0;
float xd_yaw_pro1 = 0;

float plot_data[6];

//转弯相关
float xd_yaw_turn_offset_0 = 0;
float xd_yaw_turn_offset_1 = 0;


RoboRoly_FSM_e robot_fsm = fsm_waitToCalib;


void RoboRolyWalkUpdate()
{
    ///////////////////////////////////////////////////
    // 实际状态赋值
    ///////////////////////////////////////////////////
    xa_pitch.x = state_eul.pitch;
    xa_pitch.dx = state_deul.pitch;
    xa_roll.x = state_eul.roll;
    xa_roll.dx = state_deul.roll;
    xa_yaw.x = state_eul.yaw;
    xa_yaw.dx = state_deul.yaw;

    ///////////////////////////////////////////////////
    // 期望状态赋值
    ///////////////////////////////////////////////////
    update_x_d(&xa_pitch);
    update_x_d(&xa_roll);
    update_x_d(&xa_yaw);



    // 转弯相关
    xd_yaw_turn_offset_0 = 0;
    xd_yaw_turn_offset_1 = 0;
    if (robot_fsm == fsm_walk){
//        if (walk_period_count < walk_half_period)
        if (true)
        {
            xd_yaw_turn_offset_0 = k_c * (sin_time * f_d + (float)walk_period_count);
            xd_yaw_turn_offset_1 = k_c * f_d;
        } else
        {
            xd_yaw_turn_offset_0 = k_c * ((float)walk_total_period - sin_time * f_d - (float)walk_period_count);
            xd_yaw_turn_offset_1 = -k_c * f_d;
        }
    }
    xa_yaw.x_d += xd_yaw_turn_offset_0;
    xa_yaw.dx_d += xd_yaw_turn_offset_1;
    //记录，供画图
    xd_yaw_pro0 = xa_yaw.x_d;
    xd_yaw_pro1 = xa_yaw.dx_d;


    ///////////////////////////////////////////////////
    // LQR控制器
    ///////////////////////////////////////////////////

//   xa_k = [x_{k} dx_{k} | xd_{k}  dxd_{k} | u_{k-1} ]

    const static float F_pitch[5]= {0.39004f, 1.3872465f, -1.22441938521261f, -1.43267121f, 0.1143467239f};
    const static float F_roll[5]= {0.39004f, 1.3872465086f, -0.386598045243584f, -1.39867824278894f, 0.1143467238929f};
    //yaw的这个权重调整后在matlab重新生成
//    const static float F_yaw[5]= {23.6762444052160f, 24.1615412389158f, 20.5010253203498f, 24.1764796523058f, 0.434720700061298f};
    const static float F_yaw[5]= {38.1774f,   38.8566f,  -22.8989f,  -38.7880f,    0.7077f};
    memcpy(xa_pitch.F, F_pitch, sizeof(F_pitch));
    memcpy(xa_roll.F, F_roll, sizeof(F_roll));
    memcpy(xa_yaw.F, F_yaw, sizeof(F_yaw));

    update_u(&xa_pitch);
    update_u(&xa_roll);


    // 归一化yaw轴角度，从 -pi~pi变到 -无穷~+无穷
    xa_yaw.x = normalize_angle(xa_yaw.x_d, xa_yaw.x);
    update_u(&xa_yaw);

    plot_data[0] = xa_pitch.u;
    plot_data[1] = xa_roll.u;
    plot_data[2] = xa_yaw.u;
    plot_data[3] = xa_pitch.x_d - xa_pitch.x;
    plot_data[4] = xa_roll.x_d - xa_roll.x;
    plot_data[5] = xa_yaw.x_d - xa_yaw.x;

    /////////////////////////////////////////////////////////////
    //重要！！！ 计算完u之后，把加上的转弯偏置移除，以防下次影响Ad的计算。
    xa_yaw.x_d -= xd_yaw_turn_offset_0;
    xa_yaw.dx_d -= xd_yaw_turn_offset_1;
    /////////////////////////////////////////////////////////////

    ///////////////////////////////////////////////////
    // 转速分配等
    ///////////////////////////////////////////////////
    //转速分配矩阵  由于角动量守恒反作用原理，整个矩阵都乘以 -1
    const static float speedMatrix[3][3]={
            {0.8164966f, 0.0f, -0.5773503f},
            {-0.4082483f, 0.7071068f, -0.5773503f},
            {-0.4082483f, -0.7071068f, -0.5773503f}};

    float K = 40.0f;
    motorCmd.m1 = K * (speedMatrix[0][0] * xa_roll.u + speedMatrix[0][1] * xa_pitch.u + speedMatrix[0][2] * xa_yaw.u);
    motorCmd.m2 = K * (speedMatrix[1][0] * xa_roll.u + speedMatrix[1][1] * xa_pitch.u + speedMatrix[1][2] * xa_yaw.u);
    motorCmd.m3 = K * (speedMatrix[2][0] * xa_roll.u + speedMatrix[2][1] * xa_pitch.u + speedMatrix[2][2] * xa_yaw.u);

    //发送目标数据，实施控制
    Driver_Torque_Control(M1,motorCmd.m1);
    Driver_Torque_Control(M2,motorCmd.m2);
    Driver_Torque_Control(M3,motorCmd.m3);



}

