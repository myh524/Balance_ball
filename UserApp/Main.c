//
// Created by 10798 on 2022/10/5.
//
#include <stdbool.h>
#include <tim.h>
#include <interface_can.h>
#include <attitude.h>
#include <utils/ctrl_math.h>
#include <icm42688.h>
#include <interface_uart.h>
#include <retarget.h>
#include "common_inc.h"
#include "can.h"
#include "ctrl.h"
#include "ctrl_sin.h"

// 定义用哪个陀螺仪
enum IMU_CONFIG{
    IMU_USE_NONE,
    IMU_USE_ICM20602,
    IMU_USE_ICM42688P,
} imu_config;

uint16_t safe_time = 0;
/* Thread Definitions -----------------------------------------------------*/

/* Timer Callbacks -------------------------------------------------------*/


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    //tim3定时器中断
    if (htim->Instance == TIM3) {

        //读取角速度和加速度
        if(imu_config == IMU_USE_ICM42688P){
            icm42688AccAndGyroRead(&acc_raw, &gyro_raw);
            ImuTransformer(&acc_raw, &acc, &gyro_raw, &gyro);

            CoordinateRotation(&acc, &gyro, &acc_body, &gyro_body);

            icm42688AccTransformUnit(&acc_body,&acc_f,&acc_drift);//转换单位
            icm42688GyroTransformUnit(&gyro_body,&gyro_f,&gyro_drift);//转换单位
        }

        //姿态解算
        //这里传入的参数坐标系有一个转换关系
        MahonyAHRSupdateIMU(q, gyro_f.x, gyro_f.y, gyro_f.z, acc_f.x, acc_f.y, acc_f.z);
        // 四元数反解欧拉角
        AttitudeQuaternionToEulerAngle(q,&state_attitude);
        // 转换单位
        AttitudeRadianToAngle(&state_attitude,&state_attitude_angle);

        //暂时写的，不用遥控了
        ctrl_rc.roll = state_attitude.roll;
        ctrl_rc.pitch = state_attitude.pitch;
        ctrl_rc.yaw = state_attitude.yaw;

        //PID控制更新
        //更新当前姿态和目标姿态
        ////注意！！！在这里用的是弧度制
        //TODO:PID还没写呢
//        CtrlStateUpdate(&gyro_f,&state_attitude,&ctrl_state);
//        CtrlSetpointUpdate(&ctrl_rc, &ctrl_setpoint);
//        //更新姿态控制pid
//        CtrlUpdate(&ctrl_rc, &ctrl_state, &ctrl_setpoint, &ctrl_out);
        //更新飞轮转速到驱动板
        //TODO：这个函数现在是极简版
        DriverSpeedUpdate(&ctrl_rc, &ctrl_out,&ctrl_out_sum, &motorSpeed);

        //添加安全保护，如果一定时间内没有收到串口的数据，则停车
        safe_time++;
        if (safe_time >= 100){
            ctrl_rc.armed = RC_ARMED_NO;
            safe_time = 100;
        }

        DriverCmdSend(&ctrl_rc, &motorSpeed);

        //更新CTRL forward模式的时间
        ctrl_time += 10;
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
        WIFIRead(wifi_rx_buffer, &ctrl_rc);

        //如果正常收到了串口指令，则安全计时会一直清零
        safe_time = 0;


        if (ctrl_time >= 500){
            HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_RESET);
        } else{
            HAL_GPIO_WritePin(LED_GPIO_Port,LED_Pin,GPIO_PIN_SET);
        }

        //重新打开DMA接收 idle中断
        HAL_UARTEx_ReceiveToIdle_DMA(&WIFI_UART, wifi_rx_buffer, sizeof(wifi_rx_buffer));
    }
}

/* Default Entry -------------------------------------------------------*/
void Main(void) {
    //！！！
    //NOTE:注意！！！printf已经重定向到WIFI，gh 3p 的那个串口目前没用
    //！！！
    RetargetInit(&WIFI_UART);

    //陀螺仪自检
    HAL_Delay(1000);
    if (!icm42688_self_check()){
        imu_config = IMU_USE_ICM42688P;
    }

    //陀螺仪初始化
    if(imu_config == IMU_USE_ICM42688P){
        icm42688Init();
        HAL_Delay(100);
//        ICM42688P_Gyro_And_Acc_Calibrate(&gyro_drift, &acc_drift);
    }

    HAL_Delay(100);
    //初始化电机
    //TODO:改成刘沐航写的
//    Drive_Clear_Error(M0);
//    Drive_Clear_Error(M1);
//    Drive_Clear_Error(M2);
//    Drive_Init(M0);
//    Drive_Init(M1);
//    Drive_Init(M2);

    //PID
    CtrlPIDInit();

    //wifi 串口DMA空闲中断 enable
    HAL_UARTEx_ReceiveToIdle_DMA(&WIFI_UART, wifi_rx_buffer, sizeof(wifi_rx_buffer));

    //启动定时器
    HAL_TIM_Base_Start_IT(&htim3);

    while (1){

        // Check for UART errors and restart recieve DMA transfer if required
        if (WIFI_UART.ErrorCode != HAL_UART_ERROR_NONE)
        {
            HAL_UART_AbortReceive(&WIFI_UART);
            HAL_UARTEx_ReceiveToIdle_DMA(&WIFI_UART, wifi_rx_buffer, sizeof(wifi_rx_buffer));
        }

        //----------------------------------------wifi通信发送-----------------------------------

        //TODO：实时给上位机发送自身姿态数据，包括四元数与欧拉角，printf逗号隔开即可
        //TODO：上位机写一个pygame虚拟游戏手柄，用来自稳模式玩赛车游戏

        //-------------------------------------------------------------------------------------

//        printf("%d,%d,%d,%d,%d,%d\r\n", acc_body.x, acc_body.y, acc_body.z, gyro_body.x, gyro_body.y, gyro_body.z);
//        printf("%.3f,%.3f,%.3f\r\n", state_attitude_angle.roll, state_attitude_angle.pitch, state_attitude_angle.yaw);
        printf("%.3f,%.3f,%.3f\r\n", motorSpeed.m1, motorSpeed.m2, motorSpeed.m3);
//        printf("Hello World!\r\n");

        //Blink
//        HAL_GPIO_TogglePin(LED_GPIO_Port,LED_Pin);
        //-----------------------------Delay
        HAL_Delay(100);
    }
}
