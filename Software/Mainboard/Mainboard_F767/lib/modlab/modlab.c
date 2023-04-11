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

void can_parse(void)
{
  uint16_t messagetype = (RxHeader.StdId & 0xF00);
  if(messagetype == ADDR_ACK)
  {
    switch (RxData[0])
    {
    case 0x01: //Reset Module
      rec_ack = RxData[1];
      break;
    case 0x05:
      set_dev();
      break;
    case 0x10: //Set 8bit
      rec_ack = RxData[1];
      break;
    case 0x11: //Get 8bit
      rec_ack = RxData[1];
      rec_8 = RxData[2];
      break;
    case 0x14: //Set 16bit
      rec_ack = RxData[1];
      break;
    case 0x15: //get 16bit
      rec_ack = RxData[1];
      rec_16 = RxData[2] << 8 | RxData[3];
      break;
    case 0x20: //Get Status
      rec_ack = RxData[1];
      rec_8 = RxData[2];
      break;
    case 0x40: //Enable Output
      rec_ack = RxData[1];
      break;
    case 0x41: //Disable Output
      rec_ack = RxData[1];
      break;
    case 0x60: //Set Operation mode
      rec_ack = RxData[1];
      break;
    case 0x61: //Get Operation Mode
      rec_ack = RxData[1];
      rec_8 = RxData[2];
      break;

    default:
      rec_ack = 0xFF;
      break;
    }
  }
  else if (messagetype == ADDR_AUTO)
  {
    if(RxData[0] == AUTO_DONE)
    {
      loopblock = 1;
    }
  }
  else if(messagetype == ADDR_NACK)
  {

  }
  send_ok = 1;

  if(can_debug)
  {
    int buflen = sprintf(out_buf,"[can_r] 0x%03x ", (uint16_t)RxHeader.StdId);
    for(uint8_t i = 0; i < RxHeader.DLC; i++)
    {
      buflen += sprintf(out_buf+buflen, "%02x ", RxData[i]);
    }
    sprintf(out_buf+buflen, "\n");
    CDC_Transmit_FS((uint8_t *)out_buf, strlen(out_buf));
  }
}

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

void send_set_opmode (uint16_t dev, uint8_t mode)
{
    #ifdef HBC
    dev |= 0x500;
    #endif
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.StdId = dev;
    TxHeader.DLC = 2;
    TxData[0] = CMD_SETMODE;
    TxData[1] = mode;
    transmit();
}

void scan_dev(void)
{
  dev_diode = 0;
  dev_fgen = 0;
  dev_smps = 0;
  dev_sympsu = 0;
  dev_load = 0;
    //scan Diode Tester
    for(uint8_t i = 0; i < 4; i++)
    {
        send_cmd(0x20+i, CMD_PING);
    }
    //scan SymPSU
    for(uint8_t i = 0; i < 4; i++)
    {
        send_cmd(0x30+i, CMD_PING);
    }
    //scan SMPS Tester
    for(uint8_t i = 0; i < 4; i++)
    {
        send_cmd(0x3C+i, CMD_PING);
    }
    //scan F-Gen
    for(uint8_t i = 0; i < 4; i++)
    {
        send_cmd(0x40+i, CMD_PING);
    }
    //scan Load
    for(uint8_t i = 0; i < 4; i++)
    {
        send_cmd(0x50+i, CMD_PING);
    }
}

void set_dev(void)
{
  uint8_t devid_root = RxHeader.StdId & 0x0FC;
  uint8_t devid_sub = RxHeader.StdId & 0x003;
  switch (devid_root)
  {
    case 0x20:
      dev_diode |= 1 << devid_sub;
      break;
    case 0x30:
      dev_sympsu |= 1 << devid_sub;
      break;
    case 0x3C:
      dev_smps |= 1 << devid_sub;
      break;
    case 0x40:
      dev_fgen |= 1 << devid_sub;
      break;
    case 0x50:
      dev_load |= 1 << devid_sub;
      break;
  }
}

void transmit(void)
{
  send_ok = 0;
  #ifdef HBC
  if(can_debug)
  {
    HAL_GPIO_WritePin(LED_USB_Port, LED_USB, 1);
    int buflen = sprintf(out_buf,"[can_s] 0x%03x ", (uint16_t)TxHeader.StdId);
    for(uint8_t i = 0; i < TxHeader.DLC; i++)
    {
      buflen += sprintf(out_buf+buflen, "%02x ", TxData[i]);
    }
    sprintf(out_buf+buflen, "\n");
    CDC_Transmit_FS((uint8_t *)out_buf, strlen(out_buf));
    HAL_GPIO_WritePin(LED_USB_Port, LED_USB, 0);
  }
  #endif
  HAL_GPIO_WritePin(LED_CAN_Port, LED_CAN, 1);
  HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
  uint32_t temp = HAL_GetTick();
  while(!send_ok && (HAL_GetTick() - temp < 2));
  HAL_GPIO_WritePin(LED_CAN_Port, LED_CAN, 0);

}
