//
// Created by 10798 on 2022/10/5.
//
#include <stdbool.h>
#include <tim.h>
#include <interface_can.h>
#include <attitude.h>
#include <utils/ctrl_math.h>
#include <icm20602.h>
#include <ibus.h>
#include <retarget.h>
#include "common_inc.h"
#include "can.h"
#include "ctrl.h"
#include "ctrl_sin.h"



/* Thread Definitions -----------------------------------------------------*/

/* Timer Callbacks -------------------------------------------------------*/


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    //tim3定时器中断
    if (htim->Instance == TIM3) {

        //读取角速度和加速度
        icm20602AccAndGyroRead(&acc, &gyro);
        icm20602AccTransformUnit(&acc,&acc_f,&acc_drift);//转换单位
        icm20602GyroTransformUnit(&gyro,&gyro_f,&gyro_drift);//转换单位

        //姿态解算
        //这里传入的参数坐标系有一个转换关系，源数据是东北天坐标系，抄的这个算法是北西天坐标系
        MahonyAHRSupdateIMU(q, gyro_f.y, -gyro_f.x, -gyro_f.z, acc_f.y, -acc_f.x, acc_f.z);
        // 四元数反解欧拉角
        AttitudeQuaternionToEulerAngle(q,&state_attitude);
        // 转换单位
        AttitudeRadianToAngle(&state_attitude,&state_attitude_angle);

        //PID控制更新
        //更新当前姿态和目标姿态
        CtrlStateUpdate(&gyro_f,&state_attitude,&ctrl_state);
        CtrlSetpointUpdate(&ctrl_rc, &ctrl_setpoint);
        //更新姿态控制pid
        CtrlUpdate(&ctrl_rc, &ctrl_state, &ctrl_setpoint, &ctrl_out);
        //更新飞轮转速到驱动板
        DriverSpeedUpdate(&ctrl_rc, &ctrl_out, &motorSpeed);
        DriverCmdSend(&ctrl_rc, &motorSpeed);

        //----------------------------------------wifi通信发送-----------------------------------

        //TODO：实时给上位机发送自身姿态数据，包括四元数与欧拉角，printf逗号隔开即可
        //TODO：上位机写一个pygame虚拟游戏手柄，用来自稳模式玩赛车游戏

        //-------------------------------------------------------------------------------------

        //更新CTRL forward模式的时间
        ctrl_time++;
        if(ctrl_time > period){
            ctrl_time = 0;
        }

    }
}


/* UART Callbacks -------------------------------------------------------*/
//空闲中断在这里进行处理
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    if(huart->Instance==USART2)
    {
        //串口dma+空闲中断，接受wifi的指令
        Ibus_Read_Channels(ibus_rx_buffer,RC_Channels);
        Ibus_Get_Control_Value(RC_Channels, &ctrl_rc);
        //重新打开DMA接收 idle中断
        HAL_UARTEx_ReceiveToIdle_DMA(&WIFI_UART, ibus_rx_buffer, sizeof(ibus_rx_buffer));
    }
}

/* Default Entry -------------------------------------------------------*/
void Main(void) {
    //！！！
    //NOTE:注意！！！printf已经重定向到WIFI，gh 3p 的那个串口目前没用
    //！！！
    RetargetInit(&WIFI_UART);

    //init icm20602
    icm20602Init();

    //初始化电机
    Drive_Init(M0);
    Drive_Init(M1);
    Drive_Init(M2);

    //启动定时器
    HAL_TIM_Base_Start_IT(&htim3);

    //wifi 串口DMA空闲中断 enable
    HAL_UARTEx_ReceiveToIdle_DMA(&WIFI_UART, ibus_rx_buffer, sizeof(ibus_rx_buffer));

    //PID
    CtrlPIDInit();

    while (true){

        // Check for UART errors and restart recieve DMA transfer if required
        if (WIFI_UART.ErrorCode != HAL_UART_ERROR_NONE)
        {
            HAL_UART_AbortReceive(&WIFI_UART);
            HAL_UARTEx_ReceiveToIdle_DMA(&WIFI_UART, ibus_rx_buffer, sizeof(ibus_rx_buffer));
        }

        //Blink
        HAL_GPIO_TogglePin(RGB_B_GPIO_Port,RGB_B_Pin);
        //-----------------------------Delay
        HAL_Delay(100);
    }
}


