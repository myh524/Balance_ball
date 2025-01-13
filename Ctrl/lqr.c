//
// Created by sytnocui on 2025/1/10.
//

#include "lqr.h"
#include "utils/ctrl_math.h"

lqr_xa_t xa_pitch;
lqr_xa_t xa_roll;
lqr_xa_t xa_yaw;





//void update_w_d(float _t_d)
//{
//    t_d = _t_d;
//    f_d = 1/t_d;
//    w_d = 2 * M_PIf * f_d;
//}

void update_x_d(lqr_xa_t* _xa)
{
    float new_x_d = _xa->A_d[0][0] * _xa->x_d + _xa->A_d[0][1] * _xa->dx_d;
    float new_dx_d = _xa->A_d[1][0] * _xa->x_d + _xa->A_d[1][1] * _xa->dx_d;

    _xa->x_d = new_x_d;
    _xa->dx_d = new_dx_d;
}


void update_u(lqr_xa_t* _xa)
{
    _xa->u += -1 * (_xa->F[0] * _xa->x
            + _xa->F[1] * _xa->dx
            + _xa->F[2] * _xa->x_d
            + _xa->F[3] * _xa->dx_d
            + _xa->F[4] * _xa->u);
}



// 计算矩阵指数的函数
void get_A_d(float _w_d, float _h, lqr_xa_t* _xa)
{
    // 初始化系统矩阵 A
    float A[2][2] = {
            {0.0f,         1.0f},
            {-_w_d * _w_d, 0.0f}
    };

    // 初始化单位矩阵 I
    float I[2][2] = {
            {1.0f, 0.0f},
            {0.0f, 1.0f}
    };

    // 初始化结果矩阵为单位矩阵（从第 0 项开始）
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            _xa->A_d[i][j] = I[i][j];
        }
    }

    // 临时矩阵，用于存储 A^n
    float temp[2][2] = {
            {0.0f, 0.0f},
            {0.0f, 0.0f}
    };

    // 初始 temp = A*_h
    for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 2; j++) {
            temp[i][j] = A[i][j] * _h;
        }
    }

    // Taylor 展开
    float factorial = 1.0f; // 阶乘
    int max_terms = 4;     // 最大展开项数
    for (int n = 1; n <= max_terms; n++) {
        // 将 temp 加入结果
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                _xa->A_d[i][j] += temp[i][j] / factorial;
            }
        }

        // 计算下一个 temp = temp * (A*_h)
        float new_temp[2][2] = {0.0f};
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                for (int k = 0; k < 2; k++) {
                    new_temp[i][j] += temp[i][k] * (A[k][j] * _h);
                }
            }
        }

        // 更新 temp
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                temp[i][j] = new_temp[i][j];
            }
        }

        // 更新阶乘
        factorial *= (n + 1);
    }
}
