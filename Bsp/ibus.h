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

extern uint8_t ibus_rx_buffer[IBUS_BUFFER_LENGTH];
extern uint16_t RC_Channels[IBUS_USER_CHANNELS];

//TODO：下面这个函数应该写在UART统一的接口文件中，而不是IBUS驱动文件中
void Ibus_Read_Channels(const uint8_t* rx_buffer, uint16_t* channels);
void Ibus_Get_Control_Value(uint16_t* channels, ctrl_rc_t* rc);




#ifdef __cplusplus
}
#endif

#endif //ATK_F405_FW_IBUS_H
