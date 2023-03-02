#include "modlab.h"

void timer_init()
{
    HAL_TIM_Base_Start(&htim1);
}

void delay_us(uint16_t us)
{
    __HAL_TIM_SET_COUNTER(&htim1, 0); // set the counter value a 0
    while (__HAL_TIM_GET_COUNTER(&htim1) < us)
        ; // wait for the counter to reach the us input in the parameter
}

// CAN PROTOCOL

void send_cmd (uint16_t dev, uint8_t cmd)
{
    #ifdef HBC
    dev |= 0x500;
    #endif
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.StdId = dev;
    TxHeader.DLC = 1;
    TxData[0] = cmd;
    transmit();
}
void send_get (uint16_t dev, uint8_t cmd, uint8_t reg)
{
    #ifdef HBC
    dev |= 0x500;
    #endif
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.StdId = dev;
    TxHeader.DLC = 2;
    TxData[0] = cmd;
    TxData[1] = reg;
    transmit();
}

void send_set8 (uint16_t dev, uint8_t reg, uint8_t val)
{
    #ifdef HBC
    dev |= 0x500;
    #endif
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.StdId = dev;
    TxHeader.DLC = 3;
    TxData[0] = CMD_SET8;
    TxData[1] = reg;
    TxData[2] = val;
    transmit();
}

void send_set16 (uint16_t dev, uint8_t reg, uint16_t val)
{
    #ifdef HBC
    dev |= 0x500;
    #endif
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.StdId = dev;
    TxHeader.DLC = 4;
    TxData[0] = CMD_SET16;
    TxData[1] = reg;
    TxData[2] = val >> 8;
    TxData[3] = (uint8_t)(val & 0x00FF);
    transmit();
}