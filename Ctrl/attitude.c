//
// Created by 10798 on 2023/1/9.
//

#include "attitude.h"


#include "ctrl_types.h"
#include "senser_types.h"
#include <math.h>
#include <utils/ctrl_math.h>


#include <icm42688.h>
#include "tim.h"

////---------------------------------陀螺仪相关变量------------------------------
Axis3i16 acc = {0,0,0};
Axis3i16 gyro = {0,0,0};
Axis3i16 acc_raw = {0,0,0};
Axis3i16 gyro_raw = {0,0,0};
Axis3i16 acc_drift = {0,0,0};
Axis3i16 gyro_drift = {0,0,0};
Axis3i16 acc_body = {0,0,0};
Axis3i16 gyro_body = {0,0,0};
Axis3f acc_f = {0,0,0};
Axis3f gyro_f = {0,0,0};
////---------------------------------姿态相关变量--------------------------------
//姿态四元数
float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
//欧拉角
attitude_t state_eul = {0, 0.0f, 0.0f, 0.0f};
//角度制的欧拉角
attitude_t state_attitude_angle = {0,0.0f,0.0f,0.0f};
//欧拉角变化率
attitude_t state_deul = {0,0.0f,0.0f,0.0f};
////--------------------------------------------------------------------------

////--------------------------------------------姿态解算
// Definitions

#define sampleFreq	100.0f			// sample frequency in Hz
#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain     (2.0f * 0.5f)
#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
//volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

void MahonyAHRSupdateIMU(float _q[4], float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    // 只在加速度计数据有效时才进行运算
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        // 将加速度计得到的实际重力加速度向量v单位化
        recipNorm = 1.0f/sqrtf(ax * ax + ay * ay + az * az); //该函数返回平方根的倒数
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity
        // 通过四元数得到理论重力加速度向量g
        // 注意，这里实际上是矩阵第三列*1/2，在开头对Kp Ki的宏定义均为2*增益
        // 这样处理目的是减少乘法运算量
        halfvx = _q[1] * _q[3] - _q[0] * _q[2];
        halfvy = _q[0] * _q[1] + _q[2] * _q[3];
        halfvz = _q[0] * _q[0] - 0.5f + _q[3] * _q[3];

        // Error is sum of cross product between estimated and measured direction of gravity
        // 对实际重力加速度向量v与理论重力加速度向量g做外积
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        // 在PI补偿器中积分项使能情况下计算并应用积分项
        if(twoKi > 0.0f) {
            // integral error scaled by Ki
            // 积分过程
            integralFBx += twoKi * halfex * (1.0f / sampleFreq);
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);

            // apply integral feedback
            // 应用误差补偿中的积分项
            gx += integralFBx;
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            // prevent integral windup
            // 避免为负值的Ki时积分异常饱和
            integralFBx = 0.0f;
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        // 应用误差补偿中的比例项
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    // 微分方程迭代求解
    gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = _q[0];
    qb = _q[1];
    qc = _q[2];
    _q[0] += (-qb * gx - qc * gy - _q[3] * gz);
    _q[1] += (qa * gx + qc * gz - _q[3] * gy);
    _q[2] += (qa * gy - qb * gz + _q[3] * gx);
    _q[3] += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    // 单位化四元数 保证四元数在迭代过程中保持单位性质
    recipNorm = 1.0f/sqrtf(_q[0] * _q[0] + _q[1] * _q[1] + _q[2] * _q[2] + _q[3] * _q[3]);
    _q[0] *= recipNorm;
    _q[1] *= recipNorm;
    _q[2] *= recipNorm;
    _q[3] *= recipNorm;

    //Mahony官方程序到此结束，使用时只需在函数外进行四元数反解欧拉角即可完成全部姿态解算过程
}





void AttitudeRadianToAngle(attitude_t* radian, attitude_t* angle){

    angle->roll = radian->roll * RAD2DEG;
    angle->pitch = radian->pitch * RAD2DEG;
    angle->yaw = radian->yaw * RAD2DEG;
}

void CoordinateRotation(Axis3i16* _acc, Axis3i16* _gyro, Axis3i16* _acc_body, Axis3i16* _gyro_body){
    // double CoordinateMat[3][3] =
    // {-0.408248290463863,    -0.408248290463863,  0.816496580927726,
    //  -0.707106781186548,     0.707106781186547,  0,
    //  -0.577350269189626,    -0.577350269189626,  -0.577350269189626};

    double CoordinateMat[3][3] =
    {1.0,0.0,0.0,
     0.0,1.0,0.0,
     0.0,0.0,1.0,};
    _acc_body->x = CoordinateMat[0][0] * _acc->x + CoordinateMat[0][1] * _acc->y + CoordinateMat[0][2] * _acc->z;
    _acc_body->y = CoordinateMat[1][0] * _acc->x + CoordinateMat[1][1] * _acc->y + CoordinateMat[1][2] * _acc->z;
    _acc_body->z = CoordinateMat[2][0] * _acc->x + CoordinateMat[2][1] * _acc->y + CoordinateMat[2][2] * _acc->z;

    _gyro_body->x = CoordinateMat[0][0] * _gyro->x + CoordinateMat[0][1] * _gyro->y + CoordinateMat[0][2] * _gyro->z;
    _gyro_body->y = CoordinateMat[1][0] * _gyro->x + CoordinateMat[1][1] * _gyro->y + CoordinateMat[1][2] * _gyro->z;
    _gyro_body->z = CoordinateMat[2][0] * _gyro->x + CoordinateMat[2][1] * _gyro->y + CoordinateMat[2][2] * _gyro->z;
}