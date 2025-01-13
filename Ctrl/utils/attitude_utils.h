//
// Created by sytnocui on 2025/1/10.
//

#ifndef BALANCEBALL_ATTITUDE_UTILS_H
#define BALANCEBALL_ATTITUDE_UTILS_H

#include "ctrl_types.h"
#include "senser_types.h"

void normalizeQuaternion(float _q[4]);
void QuaternionToEuler_ZYX(float _q[4], attitude_t* _eul);
void QuaternionToEuler_YXZ(float _q[4], attitude_t* _eul);
void QuaternionToEuler_ZXY(float _q[4], attitude_t* _eul);
void QuaternionToRotm(const float _q[4], float _R[3][3]);

void w2deul_zxy(const attitude_t* _eul, const Axis3f* _w, attitude_t* _deul);

float normalize_angle(float ref, float angle);

#endif //BALANCEBALL_ATTITUDE_UTILS_H
