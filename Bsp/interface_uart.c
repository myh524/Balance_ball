//
// Created by 10798 on 2023/1/4.
//

#include <string.h>
#include <utils/ctrl_math.h>
#include "interface_uart.h"
#include "attitude.h"

#define BYTE0(x) (*((char *)(&(x))))
#define BYTE1(x) (*((char *)(&(x)) +1))
//--------------------------------------------------------------------------------
//DMA读出来的
uint8_t wifi_rx_buffer[WIFI_BUFFER_LENGTH];

void WIFIRead(uint8_t* _rx_buf, ctrl_rc_t* _rc)
{
    memcpy(&_rc->armed, &_rx_buf[0], sizeof(uint16_t));
    memcpy(&_rc->mode, &_rx_buf[2], sizeof(uint16_t));
    memcpy(&_rc->roll, &_rx_buf[4], sizeof(float));
    memcpy(&_rc->pitch, &_rx_buf[8], sizeof(float));
    memcpy(&_rc->yaw, &_rx_buf[12], sizeof(float));
    _rc->thrust = 0.0f;

}

void ANOTC_Quaternion_Upload(void)
{
    uint8_t _buffer[50] = {0};
    uint8_t _cnt = 0;

    //计算需要传的数据
    //TODO:q改个名，或者封个结构体
    int16_t _q0 = (int16_t) (q[0] * 10000);
    int16_t _q1 = (int16_t) (q[1] * 10000);
    int16_t _q2 = (int16_t) (q[2] * 10000);
    int16_t _q3 = (int16_t) (q[3] * 10000);

    //帧头等信息
    _buffer[_cnt++] = 0xAA;
    _buffer[_cnt++] = 0xFF;
    _buffer[_cnt++] = 0x04;
    _buffer[_cnt++] = 9;

    //_q0 * 10000
    _buffer[_cnt++] = BYTE0(_q0);
    _buffer[_cnt++] = BYTE1(_q0);

    //_q1 * 10000
    _buffer[_cnt++] = BYTE0(_q1);
    _buffer[_cnt++] = BYTE1(_q1);

    //_q2 * 10000
    _buffer[_cnt++] = BYTE0(_q2);
    _buffer[_cnt++] = BYTE1(_q2);

    //_q3 * 10000
    _buffer[_cnt++] = BYTE0(_q3);
    _buffer[_cnt++] = BYTE1(_q3);

    //FUSION _STA：融合状态
    _buffer[_cnt++] = 1;

    //校验位
    uint8_t _sumcheck = 0;
    uint8_t _addcheck = 0;
    for(uint8_t i=0; i < (_buffer[3]+4); i++)
    {
        _sumcheck += _buffer[i]; //从帧头开始，对每一字节进行求和，直到DATA区结束
        _addcheck += _sumcheck; //每一字节的求和操作，进行一次sumcheck的累加
    }
    _buffer[_cnt++] = _sumcheck;
    _buffer[_cnt++] = _addcheck;

    //CDC_Transmit_FS(_buffer, _cnt);
    HAL_UART_Transmit_DMA(&huart2, _buffer, sizeof(_buffer));
}