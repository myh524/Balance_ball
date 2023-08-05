//
// Created by 10798 on 2023/1/4.
//

#ifndef ATK_F405_FW_IBUS_H
#define ATK_F405_FW_IBUS_H

#ifdef __cplusplus
extern "C" {
#endif


#include "main.h"
#include "usart.h"
#include "ctrl_types.h"

/* User configuration */
#define IBUS_BUFFER_LENGTH      32
#define IBUS_USER_CHANNELS		6			// Use 6 channels
/* User configuration */

#define IBUS_LENGTH				0x20	// 第一个字节，固定的       32 bytes
#define IBUS_COMMAND40			0x40	// 第二个字节，固定的       Command to set servo or motor speed is always 0x40
#define IBUS_MAX_CHANNLES		14

#define I2BUS_BUFFER_LENGTH     16

extern uint8_t ibus_rx_buffer[IBUS_BUFFER_LENGTH];
extern uint16_t RC_Channels[IBUS_USER_CHANNELS];

extern uint8_t i2bus_rx_buffer[I2BUS_BUFFER_LENGTH];

typedef union{
    uint8_t data_u8[2];
    uint16_t data_u16;
}isarmed;

typedef union{
    uint8_t data_u8[2];
    uint16_t data_u16;
}mode;

typedef union{
    uint8_t data_u8[4];
    float data_f;
}Euler_data;

typedef struct {
    isarmed _data_1;
    mode _data_2;
    Euler_data _data_roll;
    Euler_data _data_pitch;
    Euler_data _data_yaw;
}I2BUS_config;

extern I2BUS_config I2BUS;

//TODO：下面这个函数应该写在UART统一的接口文件中，而不是IBUS驱动文件中
void Ibus_Read_Channels(const uint8_t* rx_buffer, uint16_t* channels);
void Ibus_Get_Control_Value(uint16_t* channels, ctrl_rc_t* rc);
void I2bus_Read(uint8_t* rx_buffer, I2BUS_config* _I2BUS);



#ifdef __cplusplus
}
#endif

#endif //ATK_F405_FW_IBUS_H
