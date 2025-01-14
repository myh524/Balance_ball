//
// Created by sytnocui on 2025/1/10.
//

#include <math.h>
#include "attitude_utils.h"
#include "ctrl_math.h"
#include "senser_types.h"

// 定义一个小的容忍度，用于奇异性检测
#define EPSILON 1e-6


void normalizeQuaternion(float _q[4]) {
    // 计算四元数的模长，结果是1/模长
    float magnitude_1 = 1 / sqrtf(_q[0] * _q[0] + _q[1] * _q[1] + _q[2] * _q[2] + _q[3] * _q[3]);

    // 如果模长不为零，进行标准化
    if (magnitude_1 > 0.0f) {
        _q[0] *= magnitude_1;
        _q[1] *= magnitude_1;
        _q[2] *= magnitude_1;
        _q[3] *= magnitude_1;
    }
}

//这个函数和MATLAB算的一模一样
void QuaternionToEuler_ZYX(float _q[4], attitude_t* _eul)
{
    // 四元数反算欧拉角
    // 使用子蕤那嫖的正点原子的估算，asinf没有换，正点原子也没有换

    normalizeQuaternion(_q);

    float w = _q[0], x = _q[1], y = _q[2], z = _q[3];
    // fsm_roll
    float sinr_cosp = 2 * (w*x + y*z);
    float cosr_cosp = 1 - 2 * (x*x + y*y);
    _eul->roll = atan2_approx(sinr_cosp, cosr_cosp);
    // pitch
    float sinp = 2 * (w*y - z*x);
    if (fabsf(sinp) >= 1 - EPSILON) //奇异值处理
        _eul->pitch = copysignf(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        _eul->pitch = asinf(sinp);
    // yaw
    float siny_cosp = 2 * (w*z + x*y);
    float cosy_cosp = 1 - 2 * (y*y + z*z);
    _eul->yaw = atan2_approx(siny_cosp, cosy_cosp);
}


void QuaternionToEuler_YXZ(float _q[4], attitude_t* _eul)
{
    // 传入的结果是指针，结果返回到传入参数地址内
    // 四元数反算欧拉角
    // 使用子蕤那嫖的正点原子的估算，asinf没有换，正点原子也没有换
    // 自己写的不知道为什么就是不好使，chatgpt一下场，生成的立马好使了

    normalizeQuaternion(_q);

    float w = _q[0], x = _q[1], y = _q[2], z = _q[3];

    // 计算旋转矩阵的元素
//    float R11 = 1 - 2 * (y * y + z * z);
//    float R12 = 2 * (x * y - w * z);
    float R13 = 2 * (x * z + w * y);
    float R21 = 2 * (x * y + w * z);
    float R22 = 1 - 2 * (x * x + z * z);
    float R23 = 2 * (y * z - w * x);
//    float R31 = 2 * (x * z - w * y);
//    float R32 = 2 * (y * z + w * x);
    float R33 = 1 - 2 * (x * x + y * y);

    // pitch
    _eul->pitch = atan2_approx(R13, R33);

    // yaw
    _eul->yaw = atan2_approx(R21, R22);

    // fsm_roll
    if (fabsf(-R23) >= 1 - EPSILON) //奇异值处理
        _eul->roll = copysignf(M_PIf / 2, -R23); // use 90 degrees if out of range
    else
        _eul->roll = asinf(-R23);

}


void QuaternionToEuler_ZXY(float _q[4], attitude_t* _eul)
{
    // 传入的结果是指针，结果返回到传入参数地址内
    // 四元数反算欧拉角
    // 使用子蕤那嫖的正点原子的估算，asinf没有换，正点原子也没有换
    // 自己写的不知道为什么就是不好使，chatgpt一下场，生成的立马好使了

    normalizeQuaternion(_q);

    float w = _q[0], x = _q[1], y = _q[2], z = _q[3];

    // 计算旋转矩阵的元素
//    float R11 = 1 - 2 * (y * y + z * z);
    float R12 = 2 * (x * y - w * z);
//    float R13 = 2 * (x * z + w * y);
//    float R21 = 2 * (x * y + w * z);
    float R22 = 1 - 2 * (x * x + z * z);
//    float R23 = 2 * (y * z - w * x);
    float R31 = 2 * (x * z - w * y);
    float R32 = 2 * (y * z + w * x);
    float R33 = 1 - 2 * (x * x + y * y);

    // yaw
    _eul->yaw = atan2_approx(-R12, R22);

    // pitch
    _eul->pitch = atan2_approx(-R31, R33);

    // fsm_roll
    if (fabsf(R32) >= 1 - EPSILON) //奇异值处理
        _eul->roll = copysignf(M_PIf / 2, R32); // use 90 degrees if out of range
    else
        _eul->roll = asinf(R32);

}


void QuaternionToRotm(const float _q[4], float _R[3][3])
{
    // 输入: 四元数 _q (_q[0]=w, _q[1]=x, _q[2]=y, _q[3]=z)
    // 输出: 3x3 姿态矩阵 _R

    float w = _q[0], x = _q[1], y = _q[2], z = _q[3];

    // 计算旋转矩阵元素
    _R[0][0] = 1 - 2 * (y * y + z * z);
    _R[0][1] = 2 * (x * y - z * w);
    _R[0][2] = 2 * (x * z + y * w);

    _R[1][0] = 2 * (x * y + z * w);
    _R[1][1] = 1 - 2 * (x * x + z * z);
    _R[1][2] = 2 * (y * z - x * w);

    _R[2][0] = 2 * (x * z - y * w);
    _R[2][1] = 2 * (y * z + x * w);
    _R[2][2] = 1 - 2 * (x * x + y * y);
}


// Function to compute _deul from w and eul in ZXY rotation order。
// 所有顺序都是 x y z
void w2deul_zxy(const attitude_t* _eul, const Axis3f* _w, attitude_t* _deul)
{

    float cos_theta = cosf(_eul->pitch);
    float sin_theta = sinf(_eul->pitch);

    float tan_phi = tanf(_eul->roll);
    float sec_phi = 1/ cosf(_eul->roll); // sec = 1/cos

    // Compute the matrix M
    float M[3][3];
    M[0][0] = cos_theta;
    M[0][1] = 0;
    M[0][2] = sin_theta;

    M[1][0] = sin_theta * tan_phi;
    M[1][1] = 1;
    M[1][2] = -cos_theta * tan_phi;

    M[2][0] = -sin_theta * sec_phi;
    M[2][1] = 0;
    M[2][2] = cos_theta * sec_phi;

    float w_vec[3] = {_w->x, _w->y, _w->z};
    float deul_vec[3];

    // Compute _deul = invM * w
    for (int i = 0; i < 3; ++i) {
        deul_vec[i] = 0;
        for (int j = 0; j < 3; ++j) {
            deul_vec[i] += M[i][j] * w_vec[j];
        }
    }

    _deul->roll = deul_vec[0];
    _deul->pitch = deul_vec[1];
    _deul->yaw = deul_vec[2];

}


float normalize_angle(float ref, float angle)
{
    float raw_error = ref - angle;
    float normalized_error = fmodf(raw_error + M_PIf, 2 * M_PIf) - M_PIf;
    float adjusted_angle = ref - normalized_error;
    return adjusted_angle;
}