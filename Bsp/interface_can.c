//
// Created by 10798 on 2023/7/23.
//

#include "interface_can.h"
#include "ctrl_types.h"


CAN_TxHeaderTypeDef txMessage;
CAN_RxHeaderTypeDef rxMessage;


//物理层代码
void can_send_message(uint32_t id, uint8_t *buf,uint8_t len)
{
    uint32_t tx_mail = CAN_TX_MAILBOX0;
//    uint32_t tx_mail;
    txMessage.ExtId = id;
    txMessage.DLC = len;
    txMessage.IDE = CAN_ID_EXT;
    txMessage.RTR = CAN_RTR_DATA;
    txMessage.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan, &txMessage, buf, &tx_mail) != HAL_OK)
    {
        /* Transmission request Error */
        Error_Handler();
    }
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
}

void can_send_cmd(uint16_t id)
{
    uint32_t tx_mail = CAN_TX_MAILBOX0;
//    uint32_t tx_mail;
    txMessage.StdId = id;
    txMessage.DLC = 4;
    txMessage.IDE = CAN_ID_STD;
    txMessage.RTR = CAN_RTR_REMOTE;
    txMessage.TransmitGlobalTime = DISABLE;

    if (HAL_CAN_AddTxMessage(&hcan, &txMessage, NULL, &tx_mail) != HAL_OK)
    {
        /* Transmission request Error */
        Error_Handler();
    }
    while(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) == 0);
}

//驱动层代码

void Drive_Init(uint32_t CAN_ID)
{
    can_send_cmd((CAN_ID<<5) + 0x005); //电机进入闭环模式
}

void Driver_control(uint32_t CAN_ID, int16_t target)
{
    driver_data _data;
    uint8_t send_ch[8];
    send_ch[0] = 0xF5;
    if(target > 0){
        send_ch[1] = 0x00;
    }
    else{
        send_ch[1] = 0x01;
        target = -target;
    }
    send_ch[2] = 0xff;
    send_ch[3] = 0xff;
    //控制数据
    _data.data_16 = (uint16_t)target;
    for(uint8_t i=4;i<6;i++)
    {
        send_ch[i]=_data.data_ch[i-4];
    }
    send_ch[6] = 0x00;
    send_ch[7] = 0x6B;
    can_send_message(CAN_ID<<8, send_ch,8);
}

void Drive_Clear_Error(uint32_t CAN_ID)
{
    can_send_cmd((CAN_ID<<5) + 0x018); //清除错误
}


void DriverCmdSend(ctrl_rc_t* _rc, motorSpeed_t * _motor){
    //再次检测是否解锁
    //这里因为可能没收到ibus信息，故不能检测到no就return
    if(_rc->armed == RC_ARMED_YES){
        //输出三个转轴的转速
        Driver_control(M0,(int16_t)_motor->m1);
        Driver_control(M1,(int16_t)_motor->m2);
        Driver_control(M2,(int16_t)_motor->m3);
    } else {
        //输出三个转轴的转速
        Driver_control(M0,0);
        Driver_control(M1,0);
        Driver_control(M2,0);

    }


}
