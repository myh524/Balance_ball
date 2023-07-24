//
// Created by 10798 on 2022/10/5.
//
#include <mpu6050.h>
#include <stdbool.h>
#include <tim.h>
#include <interface_can.h>
#include <attitude.h>
#include <utils/ctrl_math.h>
#include "common_inc.h"
#include "can.h"
#include "ctrl.h"


/// timer

extern float Pitch_Kalman;






/* Thread Definitions -----------------------------------------------------*/

/* Timer Callbacks -------------------------------------------------------*/

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    //tim3定时器中断
    if (htim->Instance == TIM3) {
        //TODO:改名
        float brushless = 0.0f;

        //读取角速度和加速度
        mpu6000AccAndGyroRead(&acc, &gyro);
        mpu6000AccTransformUnit(&acc,&acc_f,&acc_drift);//转换单位
        mpu6000GyroTransformUnit(&gyro,&gyro_f,&gyro_drift);//转换单位

        //姿态解算
        //这里传入的参数坐标系有一个转换关系，源数据是东北天坐标系，抄的这个算法是北西天坐标系
        MahonyAHRSupdateIMU(q, gyro_f.y, -gyro_f.x, -gyro_f.z, acc_f.y, -acc_f.x, acc_f.z);
        // 四元数反解欧拉角
        AttitudeQuaternionToEulerAngle(q,&state_attitude);
        // 转换单位
        AttitudeRadianToAngle(&state_attitude,&state_attitude_angle);

        //TODO:
        //PID控制
        brushless = PIDControl(Pitch_Kalman,0);

        //更新当前姿态和目标姿态
        CtrlStateUpdate(&gyro_f,&state_attitude,&ctrl_state);
        CtrlSetpointUpdate(&ctrl_rc, &ctrl_setpoint, &ctrl_setpoint_offboard);
        //更新姿态控制pid
        CtrlUpdate(&ctrl_rc, &ctrl_state, &ctrl_setpoint, &ctrl_out);


        //转速分配
        const static float speedMatrix[3][3]={ {0.8164966f, 0.0f, -0.5773503f},
                                              {-0.4082483f, 0.7071068f, -0.5773503f},
                                              {-0.4082483f, -0.7071068f, -0.5773503f}};
        //TODO：乘之

        //限幅
        constrainf(,-80.0f,80.0f);
        constrainf(,-80.0f,80.0f);
        constrainf(,-80.0f,80.0f);

        //输出三个转轴的转速
        //TODO:需不需要三轴一块发？
        Driver_control(M0,brushless);
        Driver_control(M1,brushless);
        Driver_control(M2,brushless);
    }
}

/* UART Callbacks -------------------------------------------------------*/
//空闲中断在这里进行处理
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance==USART2)
    {
        //串口dma+空闲中断，接受wifi的指令
    }
}




/* Default Entry -------------------------------------------------------*/
void Main(void) {

    //初始化电机
    Drive_Init(M0);
    Drive_Init(M1);
    Drive_Init(M2);

    HAL_TIM_Base_Start_IT(&htim3);//控制中断

    //TODO:串口DMA还没配呢

    uint32_t run_time = 0;

    while (true){
        ////--------------------------计时-------------------------
        run_time = HAL_GetTick();
        ////------------------------------------------------------





        ////=================计时====================
        run_time = HAL_GetTick() - run_time;
        ////========================================

        //Blink
        HAL_GPIO_TogglePin(RGB_B_GPIO_Port,RGB_B_Pin);
        //-----------------------------Delay
        HAL_Delay(50);
    }
}


