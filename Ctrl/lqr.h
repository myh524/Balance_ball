//
// Created by sytnocui on 2025/1/10.
//

#ifndef BALANCEBALL_LQR_H
#define BALANCEBALL_LQR_H


#include <stdint-gcc.h>

typedef struct
{
    //   xa_k = [x_{k} dx_{k} | xd_{k}  dxd_{k} | u_{k-1} ]
    float x;	  //角度
    float dx;	  //角速度
    float x_d;		//角度期望
    float dx_d;		//角速度期望
    float u;   //上一帧的控制量

    float A_d[2][2]; //期望值的状态转移矩阵
    float F[5]; //反馈矩阵

} lqr_xa_t;




extern lqr_xa_t xa_pitch;
extern lqr_xa_t xa_roll;
extern lqr_xa_t xa_yaw;


//void update_w_d(float _t_d);
void update_x_d(lqr_xa_t* _xa);
void update_u(lqr_xa_t* _xa);

void get_A_d(float _w_d, float _h, lqr_xa_t* _xa);


#endif //BALANCEBALL_LQR_H
