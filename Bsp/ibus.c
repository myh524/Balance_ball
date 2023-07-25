//
// Created by 10798 on 2023/1/4.
//

#include <utils/ctrl_math.h>
#include "ibus.h"



//--------------------------------------------------------------------------------
//DMA读出来的
uint8_t ibus_rx_buffer[IBUS_BUFFER_LENGTH] = {0};
//最后获得的
uint16_t RC_Channels[IBUS_USER_CHANNELS] = {0};


////注：本函数只适用于用通道<=12；13-14好不好用不知道，15-18一定不好用，因为没有处理前面插空的通道
void Ibus_Read_Channels(const uint8_t* rx_buffer, uint16_t* channels)
{
    //校验前暂存各通道值的地方
    uint16_t channel_buffer[IBUS_MAX_CHANNLES] = {0};
    //用于最后两个字节的校验
    uint16_t checksum_cal = 0;
    uint16_t checksum_ibus = 0;

    if(rx_buffer[0] == IBUS_LENGTH && rx_buffer[1] == IBUS_COMMAND40)
    {
        //校验值计算
        checksum_cal = 0xffff - rx_buffer[0] - rx_buffer[1];

        for(int i = 0; i < IBUS_MAX_CHANNLES; i++)
        {
            //各通道的值赋值，猜测ch15以后的遥控器根本就不支持，所以处理的时候默认全是0，故高位不需要覆盖
            channel_buffer[i] = (uint16_t)(rx_buffer[i * 2 + 3] << 8 | rx_buffer[i * 2 + 2]);
            //校验值计算
            checksum_cal = checksum_cal - rx_buffer[i * 2 + 3] - rx_buffer[i * 2 + 2];
        }

        //末尾两个是校验数
        checksum_ibus = rx_buffer[31] << 8 | rx_buffer[30];

        //如果校验通过，就把暂存的赋给最后的全局变量
        if(checksum_cal == checksum_ibus)
        {
            //只赋值前几个用的，不用的就不赋值
            for(int j = 0; j < IBUS_USER_CHANNELS; j++)
            {
                channels[j] = channel_buffer[j];
            }
        }
    }
}


void Ibus_Get_Control_Value(uint16_t* channels,ctrl_rc_t* rc){
    //TODO:这里加上参数检查，如果不在1000-2000就报错
    //遥控信号处理
    //摇杆通道归一化到-1~1
    //注意pitch要反向，-1和1颠倒着写
    rc->roll = scaleRangef(channels[RC_CHANNLE_ROLL],1000,2000,-1,1);
    rc->pitch = scaleRangef(channels[RC_CHANNLE_PITCH],1000,2000,1,-1);
    rc->thrust = scaleRangef(channels[RC_CHANNLE_THRUST],1000,2000,-1,1);
    rc->yaw = scaleRangef(channels[RC_CHANNLE_YAW],1000,2000,1,-1);
    //拨杆通道直接赋值
    rc->armed = channels[RC_CHANNLE_ARMED];
    rc->mode = channels[RC_CHANNLE_MODE];
}