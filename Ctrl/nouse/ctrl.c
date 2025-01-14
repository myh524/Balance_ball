//
// Created by 10798 on 2022/12/14.
//

#include "ctrl.h"
#include "pid.h"
#include "utils/ctrl_math.h"

#include <math.h>
#include <interface_can.h>
#include "tim.h"
#include "senser_types.h"
#include "attitude.h"
#include "ctrl_types.h"


//ctrl_rc_t ctrl_rc;
//ctrl_state_t ctrl_state;
//ctrl_setpoint_t ctrl_setpoint;
//ctrl_setpoint_t ctrl_setpoint_offboard;
//ctrl_out_t ctrl_out;
//ctrl_out_t ctrl_out_sum; //计算转速饱和用的
motorCmd_t motorCmd = {0.0f, 0.0f, 0.0f};

////PID
//pid_calc_t roll_pid;
//pid_calc_t pitch_pid;
//pid_calc_t yaw_pid;


//void CtrlStateUpdate(const Axis3f* _gyro_f, const attitude_t* _attitude, ctrl_state_t* _state){
//    //通过姿态解算获得的实际值，存在state里，无需进行单位换算
//    //到这里应该是正负都已经弄好的,如果不对记得看看
//    _state->attitudeRate.fsm_roll = _gyro_f->x;
//    _state->attitudeRate.pitch = _gyro_f->y;
//    _state->attitudeRate.yaw = _gyro_f->z;
//
//    ////注意！！！在这里用的是弧度制
//    _state->attitude.fsm_roll = _attitude->fsm_roll;
//    _state->attitude.pitch = _attitude->pitch;
//    _state->attitude.yaw = _attitude->yaw;
//}
//void CtrlPIDInit(void){
//    //初始化PID
//    //外环 注意现在是弧度值
//    PID_init(&roll_pid, PID_POSITION, 2.0f,0,0, 100,0);
//    PID_init(&pitch_pid, PID_POSITION, 2.0f,0,0, 100,0);
//    PID_init(&yaw_pid, PID_POSITION, 2.0f,0,0, 100,0);
//
//}
//
//void CtrlSetpointUpdate(const ctrl_rc_t* _rc, ctrl_setpoint_t* _setpoint){
//    // 通过rc获得期望值，存在setpoint里
//    if(_rc->mode == RC_MODE_STABILIZED) //自稳模式
//    {
//        //期望角度
//        _setpoint->attitude.fsm_roll = 0;
//        _setpoint->attitude.pitch = 0;
//        _setpoint->attitude.yaw = 0;
//    }
//    else if(_rc->mode == RC_MODE_TORQUE) //定姿模式
//    {
//        // 期望角度
//        _setpoint->torque.fsm_roll = _rc->fsm_roll;
//        _setpoint->torque.pitch = _rc->pitch;
//        _setpoint->torque.yaw = _rc->yaw;
//    }
//    else if(_rc->mode == RC_MODE_WALK)
//    {
//        //TODO:新模式
//    }
//    else
//    {
//        //什么mode都没识别到，按自稳的给，全给0
//        _setpoint->attitude.fsm_roll = 0;
//        _setpoint->attitude.pitch = 0;
//        _setpoint->attitude.yaw = 0;
//        _setpoint->attitudeRate.fsm_roll = 0;
//        _setpoint->attitudeRate.pitch = 0;
//        _setpoint->attitudeRate.yaw = 0;
//        _setpoint->torque.fsm_roll = 0;
//        _setpoint->torque.pitch = 0;
//        _setpoint->torque.yaw = 0;
//    }
//
//}
//
//
//void CtrlUpdate(const ctrl_rc_t* _rc, const ctrl_state_t* _state, ctrl_setpoint_t* _setpoint, ctrl_out_t* _out){
//    //先判断是否解锁
//    if(_rc->armed == RC_ARMED_YES){
//
//        if(_rc->mode == RC_MODE_STABILIZED) //自稳模式
//        {
//            //角度环
//            _out->fsm_roll =  PID_calc(&roll_pid, _state->attitude.fsm_roll, _setpoint->attitude.fsm_roll);
//            _out->pitch = PID_calc(&pitch_pid, _state->attitude.pitch, _setpoint->attitude.pitch);
//            _out->yaw = PID_calc(&yaw_pid, _state->attitude.yaw, _setpoint->attitude.yaw);
//        }
//        else if(_rc->mode == RC_MODE_TORQUE) //开环力矩模式
//        {
//            //开环力矩模式现在为直传
//            _out->fsm_roll =  _setpoint->torque.fsm_roll;
//            _out->pitch = _setpoint->torque.pitch;
//            _out->yaw =  _setpoint->torque.yaw;
//
//        }
//        else if(_rc->mode == RC_MODE_WALK)
//        {
//
//        }
//        else
//        {
//            //什么mode都没识别到，全给0
//            _out->fsm_roll = 0;
//            _out->pitch = 0;
//            _out->yaw = 0;
//        }
//
//    }
//    else {
//        //没解锁
//        _out->fsm_roll = 0;
//        _out->pitch = 0;
//        _out->yaw = 0;
//    }
//}
//
//
//
//////---------------------------------------------about-motor-speed-------------------------------------------
//void DriverSpeedUpdate(const ctrl_rc_t* _rc, const ctrl_out_t* _out, ctrl_out_t* _out_sum, motorCmd_t* _motor) {
//
//    //转速分配矩阵  由于角动量守恒反作用原理，整个矩阵都乘以 -1
//    const static float speedMatrix[3][3]={
//            {0.8164966f, 0.0f, -0.5773503f},
//            {-0.4082483f, 0.7071068f, -0.5773503f},
//            {-0.4082483f, -0.7071068f, -0.5773503f}};
//
//    //先判断是否解锁
//    if(_rc->armed == RC_ARMED_YES){
//
//        if(_rc->mode == RC_MODE_STABILIZED
//        || _rc->mode == RC_MODE_TORQUE ) //自稳模式 力矩模式
//        {
//            float K = 5.0f;
//            _motor->m1 = K * (speedMatrix[0][0] * _out->fsm_roll + speedMatrix[0][1] * _out->pitch + speedMatrix[0][2] * _out->yaw);
//            _motor->m2 = K * (speedMatrix[1][0] * _out->fsm_roll + speedMatrix[1][1] * _out->pitch + speedMatrix[1][2] * _out->yaw);
//            _motor->m3 = K * (speedMatrix[2][0] * _out->fsm_roll + speedMatrix[2][1] * _out->pitch + speedMatrix[2][2] * _out->yaw);
//        }
//        else if(_rc->mode == RC_MODE_WALK){
//
//        }
//        else
//        {
//            _motor->m1 = 0.0f;
//            _motor->m2 = 0.0f;
//            _motor->m3 = 0.0f;
//        }
//
//    }
//    else {
//        //没解锁
//        _motor->m1 = 0.0f;
//        _motor->m2 = 0.0f;
//        _motor->m3 = 0.0f;
//    }
//
//    //限幅
//    _motor->m1 = constrainf(_motor->m1,-100.0f,100.0f);
//    _motor->m2 = constrainf(_motor->m2,-100.0f,100.0f);
//    _motor->m3 = constrainf(_motor->m3,-100.0f,100.0f);
//}
//void DriverCmdSend(ctrl_rc_t* _rc, motorCmd_t * _motor){
//    //再次检测是否解锁
//    //这里因为可能没收到ibus信息，故不能检测到no就return
//    if(_rc->armed == RC_ARMED_YES){
//        //输出三个转轴的转速
//        Driver_Velocity_Control(M1,_motor->m1);
//        Driver_Velocity_Control(M2,_motor->m2);
//        Driver_Velocity_Control(M3,_motor->m3);
//    } else {
//        //输出三个转轴的转速
//        Driver_Velocity_Control(M1,0);
//        Driver_Velocity_Control(M2,0);
//        Driver_Velocity_Control(M3,0);
//    }
//
//}


