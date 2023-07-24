//
// Created by 10798 on 2022/12/14.
//

#include "ctrl.h"
#include "pid.h"
#include "utils/ctrl_math.h"

#include <math.h>
#include <mpu6050.h>
#include "tim.h"
#include "ctrl_types.h"
#include "senser_types.h"

//TODO:增加这几个东西的初始化函数
ctrl_state_t ctrl_state;
ctrl_setpoint_t ctrl_setpoint;
ctrl_setpoint_t ctrl_setpoint_offboard;
ctrl_out_t ctrl_out;

//PID
pid_calc_t roll_pid;
pid_calc_t pitch_pid;
pid_calc_t yaw_pid;
pid_calc_t rollRate_pid;
pid_calc_t pitchRate_pid;
pid_calc_t yawRate_pid;


void CtrlPIDInit(void){
    //初始化PID
    //外环
    PID_init(&roll_pid, PID_POSITION, 3.0f,0,0, 100,0);
    PID_init(&pitch_pid, PID_POSITION, 3.0f,0,0, 100,0);
    PID_init(&yaw_pid, PID_POSITION, 0.2f,0,0, 100,0);
    //内环 不要d，因为角速度本来就很斗
    PID_init(&rollRate_pid, PID_POSITION, 0.2f,0,0, 100,0);
    PID_init(&pitchRate_pid, PID_POSITION, 0.2f,0,0, 100,0);
    PID_init(&yawRate_pid, PID_POSITION, 0.1f,0,0, 100,0);   //原来P = 0.2f


}

void CtrlStateUpdate(const Axis3f* _gyro_f, const attitude_t* _attitude, ctrl_state_t* _state){
    //通过姿态解算获得的实际值，存在state里，无需进行单位换算
    //TODO:到这里应该是正负都已经弄好的,如果不对记得看看
    _state->attitudeRate.roll = _gyro_f->y;
    _state->attitudeRate.pitch = _gyro_f->x;
    _state->attitudeRate.yaw = _gyro_f->z;

    _state->attitude.roll = _attitude->roll;
    _state->attitude.pitch = _attitude->pitch;
    _state->attitude.yaw = _attitude->yaw;
}


void CtrlSetpointUpdate(const ctrl_rc_t* _rc, ctrl_setpoint_t* _setpoint, ctrl_setpoint_t* _setpoint_offboard){
    //通过rc获得期望值，存在setpoint里
    if(_rc->mode == RC_MODE_STABILIZED) //自稳模式
    {
        //期望角度，单位从-1~1转换到rad，范围：-1rad~1rad，感觉和px4差不多
        _setpoint->attitude.roll = scaleRangef(_rc->roll,-1.0f,1.0f,-1.0f,1.0f);
        _setpoint->attitude.pitch = scaleRangef(_rc->pitch,-1.0f,1.0f,-1.0f,1.0f);
        //yaw轴是期望角速度，单位rad/s 范围：TODO:这个暂时瞎猜的范围
        _setpoint->attitudeRate.yaw = scaleRangef(_rc->yaw,-1.0f,1.0f,-5.0f,5.0f);
        //油门无需缩放范围，-1~1
        _setpoint->thrust = _rc->thrust;

    }
    else if(_rc->mode == RC_MODE_POSITION) //定点模式 //TODO:定点模式先和自稳一样，光流和加速度计融合还有油门控高都之后再写
    {


    }
    else if(_rc->mode == RC_MODE_OFFBOARD) //OFFBOARD模式
    {

    }
    else
    {
        //什么mode都没识别到，按自稳的给，全给0
        _setpoint->attitude.roll = 0;
        _setpoint->attitude.pitch = 0;
        _setpoint->attitudeRate.yaw = 0;
        _setpoint->thrust = -1;
    }

}


void CtrlUpdate(const ctrl_rc_t* _rc, const ctrl_state_t* _state, ctrl_setpoint_t* _setpoint, ctrl_out_t* _out){
    //先判断是否解锁
    if(_rc->armed == RC_ARMED_YES){

        if(_rc->mode == RC_MODE_STABILIZED) //自稳模式
        {
            //先算外环pid
            _setpoint->attitudeRate.roll = PID_calc(&roll_pid, _state->attitude.roll, _setpoint->attitude.roll);
            _setpoint->attitudeRate.pitch = PID_calc(&pitch_pid, _state->attitude.pitch, _setpoint->attitude.pitch);
            //再算内环pid
            _out->roll =  PID_calc(&rollRate_pid, _state->attitudeRate.roll, _setpoint->attitudeRate.roll);
            _out->pitch = PID_calc(&pitchRate_pid, _state->attitudeRate.pitch, _setpoint->attitudeRate.pitch);
            //yaw只有角速度环
            _out->yaw = PID_calc(&yawRate_pid, _state->attitudeRate.yaw, _setpoint->attitudeRate.yaw);
            //油门直接赋值，都已经归一化好了
            _out->thrust = _setpoint->thrust;

        }
        else if(_rc->mode == RC_MODE_POSITION) //定点模式 //TODO:定点模式先和自稳一样，光流和加速度计融合还有油门控高都之后再写
        {


        }
        else if(_rc->mode == RC_MODE_OFFBOARD) //OFFBOARD模式
        {

        }
        else
        {
            //什么mode都没识别到，全给0，油门给-1
            _out->roll = 0;
            _out->pitch = 0;
            _out->yaw = 0;
            _out->thrust = -1;
        }

    }
    else {
        //没解锁
        _out->roll = 0;
        _out->pitch = 0;
        _out->yaw = 0;
        _out->thrust = -1;
    }
}

