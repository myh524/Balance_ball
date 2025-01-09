//
// Created by 10798 on 2022/12/14.
//

#ifndef ATK_F405_FW_CTRL_H
#define ATK_F405_FW_CTRL_H


#ifdef __cplusplus
extern "C" {
#endif

#include <ctrl_types.h>
#include <senser_types.h>
#include "stdio.h"
#include "pid.h"

extern float ctrl_time;
extern ctrl_rc_t ctrl_rc;
extern ctrl_state_t ctrl_state;
extern ctrl_setpoint_t ctrl_setpoint;
extern ctrl_setpoint_t ctrl_setpoint_offboard;
extern ctrl_out_t ctrl_out;
extern ctrl_out_t ctrl_out_sum;
extern motorCmd_t motorCmd;

extern pid_calc_t roll_pid;
extern pid_calc_t pitch_pid;
extern pid_calc_t yaw_pid;



//这些都暂时不用了
//void CtrlPIDInit(void);
//void CtrlSetpointUpdate(const ctrl_rc_t* _rc, ctrl_setpoint_t* _setpoint);
//void CtrlUpdate(const ctrl_rc_t* _rc,const ctrl_state_t* _state, ctrl_setpoint_t* _setpoint, ctrl_out_t* _out);
//void DriverSpeedUpdate(const ctrl_rc_t* _rc, const ctrl_out_t* _out, ctrl_out_t* _out_sum, motorCmd_t* _motor);
//void DriverCmdSend(ctrl_rc_t* _rc, motorCmd_t * _motor);
//void CtrlStateUpdate(const Axis3f* _gyro_f, const attitude_t* _attitude, ctrl_state_t* _state);

void RoboRolyWalkUpdate(); //简化版的，合了之前的setpoint，ctrl和speed update



#ifdef __cplusplus
}
#endif

#endif //ATK_F405_FW_CTRL_H
