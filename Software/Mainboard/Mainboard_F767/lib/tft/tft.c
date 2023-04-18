#include "tft.h"

// Initialize Variables
void tft_init(void)
{
  // general variables
  tft_page = 0;
  tft_data = 0;
  tft_cmd = 0;
  module = 0;
  outstate = 0;
  setdata = 0; // Indicator for data received
  setout = 0;  // indicator for Output to set
  readdata = 0;
  setdebug = 0;

  // Diode Tester Variables
  start_index = 0;
  stop_index = 0;
  diode_mode = 0;

  // Function Generator Variables
  frequency = 0;
  amplitude = 0;
  offset = 0;
}

void tft_parse(void)
{
  
  // get Command and look for further data
  if (!tft_data)
  {
    HAL_GPIO_WritePin(LED_TFT_Port, LED_TFT, 1);
    tft_cmd = RxBuffer[0];
    tft_data = 1;
    if (tft_cmd == CMD_PAGESET)
    {
      HAL_UART_Receive_IT(&huart1, RxBuffer, 1);
    }
    else if (tft_cmd == CMD_SETDATA)
    {
      if (tft_page == PAGE_DTEST)
      {
        HAL_UART_Receive_IT(&huart1, RxBuffer, 4);
      }
      else if (tft_page == PAGE_FGEN)
      {
        HAL_UART_Receive_IT(&huart1, RxBuffer, 9);
      }
      else if (tft_page == PAGE_SYMPSU)
      {
        HAL_UART_Receive_IT(&huart1, RxBuffer, 9);
      }
      else if (tft_page == PAGE_SMPS)
      {
        HAL_UART_Receive_IT(&huart1, RxBuffer, 5);
      }
      else if (tft_page == PAGE_LOAD)
      {
        HAL_UART_Receive_IT(&huart1, RxBuffer, 5);
      }
    }
    else if (tft_cmd == CMD_SETOUT)
    {
      HAL_UART_Receive_IT(&huart1, RxBuffer, 1);
    }
    else if (tft_cmd == CMD_DEBUG)
    {
      HAL_UART_Receive_IT(&huart1, RxBuffer, 1);
    }
    else if (tft_cmd == CMD_READDAT)
    {
      tft_data = 0;
      readdata = 1;
      HAL_UART_Receive_IT(&huart1, RxBuffer, 1);
      HAL_GPIO_WritePin(LED_TFT_Port, LED_TFT, 0);
    }
    else
    {
      tft_data = 0;
      HAL_UART_Receive_IT(&huart1, RxBuffer, 1);
      HAL_GPIO_WritePin(LED_TFT_Port, LED_TFT, 0);
    }
  }
  // get Data to command
  else
  {
    tft_data = 0;
    if (tft_cmd == CMD_PAGESET)
    {
      tft_page = RxBuffer[0];
      setdata = 1;
    }
    else if (tft_cmd == CMD_SETDATA)
    {
      if (tft_page == PAGE_DTEST)
      {
        module = RxBuffer[0] - 1;
        diode_mode = RxBuffer[1];
        start_index = RxBuffer[2];
        stop_index = RxBuffer[3];
        setout = 1;
      }
      else if (tft_page == PAGE_FGEN)
      {
        module = RxBuffer[0] - 1;
        frequency = (RxBuffer[4] << 24) + (RxBuffer[3] << 16) + (RxBuffer[2] << 8) + (RxBuffer[1]);
        amplitude = (RxBuffer[6] << 8) + (RxBuffer[5]);
        offset = (RxBuffer[8] << 8) + (RxBuffer[7]);
      }
      else if (tft_page == PAGE_SYMPSU)
      {
        //<01-04> <LSB_MSB Vpos> <LSB_MSB Ipos> <LSB_MSB Vneg> <LSB_MSB Ineg>
        module = RxBuffer[0] - 1;
        sympsu_vpos_set = (RxBuffer[2] << 8) + (RxBuffer[1]);
        sympsu_ipos_set = (RxBuffer[4] << 8) + (RxBuffer[3])*10;
        sympsu_vneg_set = (RxBuffer[6] << 8) + (RxBuffer[5]);
        sympsu_ineg_set = (RxBuffer[8] << 8) + (RxBuffer[7])*10;
      }
      else if (tft_page == PAGE_SMPS)
      {
        //<01-04> <LSB_MSB Vpos> <LSB_MSB Ipos>
        module = RxBuffer[0] - 1;
        smps_v_set = (RxBuffer[2] << 8) + (RxBuffer[1]);
        smps_i_set = (RxBuffer[4] << 8) + (RxBuffer[3]);
      }
      else if (tft_page == PAGE_LOAD)
      {
        //<01-04> <LSB_MSB Vpos> <LSB_MSB Ipos>
        module = RxBuffer[0] - 1;
        load_set_0 = (RxBuffer[2] << 8) + (RxBuffer[1]);
        load_set_1 = (RxBuffer[4] << 8) + (RxBuffer[3]);
      }
      setdata = 1;
    }
    else if (tft_cmd == CMD_SETOUT)
    {
      module = ((RxBuffer[0] & 0xF0) >> 4) - 1;
      outstate = RxBuffer[0] & 0x0F;
      setout = 1;
    }
    else if (tft_cmd == CMD_DEBUG)
    {
      setdebug = RxBuffer[0];
      setout = 1;
    }

    if (tft_debug)
    {
      char usb_out[50];
      uint8_t len = sprintf(usb_out, "[tft_s] %02X", tft_cmd);
      for (int i = 0; i < sizeof(RxBuffer); i++)
      {
        len += sprintf(usb_out + len, " %02X", RxBuffer[i]);
      }
      len += sprintf(usb_out + len, "\n");
      CDC_Transmit_FS((uint8_t *)usb_out, len);
    }
    HAL_UART_Receive_IT(&huart1, RxBuffer, 1);
    HAL_GPIO_WritePin(LED_TFT_Port, LED_TFT, 0);
  }
}

void tft_send(char commandstring[60], uint8_t len)
{
  HAL_GPIO_WritePin(LED_TFT_Port, LED_TFT, 1);
  uint8_t CmdEnd[3] = {0xff, 0xff, 0xff};
  HAL_UART_Transmit(&huart1, (uint8_t *)commandstring, len, 1);
  HAL_UART_Transmit(&huart1, CmdEnd, 3, 1);
  if (tft_debug)
  {
    HAL_GPIO_WritePin(LED_USB_Port, LED_USB, 1);
    char usb_out[50];
    len = sprintf(usb_out, "[tft_s] %s\n", commandstring);
    CDC_Transmit_FS((uint8_t *)usb_out, len);
    HAL_GPIO_WritePin(LED_USB_Port, LED_USB, 0);
  }
  HAL_GPIO_WritePin(LED_TFT_Port, LED_TFT, 0);
}

void tft_reset(void)
{
  sprintf(tft_string, "rest");
  tft_send(tft_string, 4);
}

/**
 * Main Page Function
 */

void main_page_loop(void)
{
  if (setdebug != 0)
  {
    uint8_t mode = (setdebug & 0xF0) >> 4;
    switch (mode)
    {
    case 1:
      can_debug = setdebug & 0x0F;
      break;
    case 2:
      //tft_debug = setdebug & 0x0F;
      break;
    case 3:
      sim_debug = setdebug & 0x0F;
      break;
    case 4:
      scan_dev();
      break;
    }
    setdebug = 0;
  }
}

/**
 * Befehl für Display-Reset: "rest" + ff ff ff
 */

/**
 * Buttons für Module:
 * Main Page: 	b0 (55 00)
 * Diode Page: 	b1 (55 01)
 * F-Gen Page: 	b2 (55 02)
 * SymPSU Page: b3 (55 03)
 * SMPS Page: 	b4 (55 04)
 *
 */

/**
 * TFT-Steuerung Diode Tester:
 * Diode Tester ausgewählt:
 * 55 01
 * Kanäle aktivieren:
 * 	"r<0-3>.aph=127" + ff ff ff zum aktivieren
 *  "r<0-3>.aph=25" + ff ff ff zum deaktivieren
 *
 * Daten für Messung:
 * 0x5C <01-04> <01-04> <00-15> <00-15>
 * INST |Kanal| |Mode | |Start| |Stop |
 *
 * Start/Stop als Index: 00 = 2mA; 15 = 37mA
 *
 * Messwerte in x(0-15), selber Index wie Start/Stop:
 * "x<index>.val=<Zahlenwert in ASCII>" + ff ff ff
 * Zahlenwert in 1/100
 *
 */

void enable_diode(uint8_t ch)
{
  uint8_t len = sprintf(tft_string, "r%d.aph=127", ch);
  tft_send(tft_string, len);
}

void disable_diode(uint8_t ch)
{
  uint8_t len = sprintf(tft_string, "r%d.aph=25", ch);
  tft_send(tft_string, len);
}

void diode_page_loop(void)
{
  uint8_t len;
  //enable channels on tft at startup
  if (enableblock == 1)
  {
    enableblock = 0;
    for (int i = 0; i < 4; i++)
    {
      if (sim_debug)
      {
        enable_diode(i);
      }
      else if ((dev_diode >> i) & 0x01)
      {
        enable_diode(i);
      }
      else
      {
        disable_diode(i);
      }
    }
  }

  //start measurement if data received
  if (setout && (start_index <= stop_index))
  {
    for(uint8_t i = 0; i < 14; i++)
    {
      if(i < 7)
          len = sprintf(tft_string, "x%d.val=0", i);
        else
          len = sprintf(tft_string, "x%d.val=0", i+1);
        tft_send(tft_string, len);
    }
    setout = 0;
    loopblock = 0;
    send_set8(ADDR_DiodeTester + module, START_CURR, start_index);
    send_set8(ADDR_DiodeTester + module, END_CURR, stop_index);
    send_set_opmode(ADDR_DiodeTester + module, diode_mode);
    send_cmd(ADDR_DiodeTester + module, CMD_ENOUT);
  }

  //get measured values and display them
  if (loopblock)
  {
    loopblock = 0;
    for (uint8_t i = start_index; i <= stop_index; i++)
    {
      send_get(ADDR_DiodeTester + module, CMD_GET16, MA2 + i);
      uint32_t temp_time = HAL_GetTick();
      while (!send_ok && ((HAL_GetTick() - temp_time) < 10));
      if (send_ok)
      {
        if(i < 7)
          len = sprintf(tft_string, "x%d.val=%d", i, rec_16);
        else
          len = sprintf(tft_string, "x%d.val=%d", i+1, rec_16);
        tft_send(tft_string, len);
      }
    }
  }
}

/**
 * TFT-Steuerung F-Gen:
 * 55 02
 * Kanäle sichtbar schalten:
 * 	"vis t<47-50>,0" + ff ff ff zum aktivieren
 * 	"vis t<47-50>,1" + ff ff ff zum deaktivieren
 *
 * Daten für Frequenz, Amplitude, Offset:
 * 0x5C <01-04> <LSB...MSB F> <LSB_MSB A> <LSB_MSB O>
 * INST  1 Byte    4 Byte       2 Byte      2 Byte
 *
 * Ausgang freischalten: 0x50 + <10/11> <20/21> <30/31> <40/41>
 */

void enable_fgen(uint8_t ch)
{
  uint8_t len = sprintf(tft_string, "vis t%d,0", 47 + ch);
  tft_send(tft_string, len);
}

void disable_fgen(uint8_t ch)
{
  uint8_t len = sprintf(tft_string, "vis t%d,1", 47 + ch);
  tft_send(tft_string, len);
}

void fgen_page_loop(void)
{
  if (enableblock == 1)
  {
    enableblock = 0;
    for (int i = 0; i < 4; i++)
    {
      if (sim_debug)
      {
        enable_fgen(i);
      }
      else if((dev_fgen >> i) & 0x01)
      {
        enable_fgen(i);
      }
      else
      {
        disable_fgen(i);
      }
    }
  }

  if(setdata)
  {
    setdata = 0;
    fgen_setdata();
  }

  if(setout)
  {
    setout = 0;
    if(outstate)
    {
      send_cmd(ADDR_WaveGen + module, CMD_ENOUT);
    }
    else
    {
      send_cmd(ADDR_WaveGen + module, CMD_DISOUT);
    }
  }
}

void fgen_setdata(void)
{
  send_set32(ADDR_WaveGen + module, FREQUENCY, frequency);
  send_set16(ADDR_WaveGen + module, GAIN, amplitude);
  send_set16(ADDR_WaveGen + module, OFFSET, offset);
}
/**
 * TFT-Steuerung SymPSU:
 * SymPSU ausgewählt:
 * 55 03
 * Kanäle sichtbar schalten:
 * 	"vis t(79-82),0" + ff ff ff zum aktivieren
 * 	"vis t<79-82>,1" + ff ff ff zum deaktivieren
 * Daten für Spannungen/Ströme:
 * 0x5C + <01-04> <LSB_MSB Vpos> <LSB_MSB Ipos> <LSB_MSB Vneg> <LSB_MSB Ineg>
 * Messwerte anzeigen: "<boxname>.val=<Zahlenwert in ASCII>" + ff ff ff
 * Zahlenwert in 1/100
 *        |  Kanal 1  | Kanal 2 | Kanal 3 | Kanal 4 |
 * -------+-----------+---------+---------+---------+
 *  V_POS | x4		  | x12		| x20	  | x28     |
 *  I_POS | x6		  | x14		| x22	  | x30		|
 *  V_NEG | x5		  | x13		| x21	  | x29     |
 *  I_NEG | x7		  | x15		| x23	  | x31		|
 *
 * Spannung freischalten: 0x50 + <10/11> <20/21> <30/31> <40/41>
 *
 */

void enable_sympsu(uint8_t ch)
{
  uint8_t len = sprintf(tft_string, "vis t%d,0", 79 + ch);
  tft_send(tft_string, len);
}

void disable_sympsu(uint8_t ch)
{
  uint8_t len = sprintf(tft_string, "vis t%d,1", 79 + ch);
  tft_send(tft_string, len);
}

void sympsu_page_loop(void)
{
  //enable channels on tft at startup
  if (enableblock == 1)
  {
    enableblock = 0;
    for (int i = 0; i < 4; i++)
    {
      if (sim_debug)
      {
        enable_sympsu(i);
      }
      else if ((dev_sympsu >> i) & 0x01)
      {
        enable_sympsu(i);
      }
      else
      {
        disable_sympsu(i);
      }
    }
    pagetimer = HAL_GetTick();
  }

  //periodically get voltage and current values
  if(readdata)
  {
    for(int j = 0; j < 4; j++)
    {
      if((dev_sympsu >> j) & 0x01)
      {
        sympsu_get_data(j);
      }
    }
    readdata = 0;
  }

  //set values if received
  if(setdata)
  {
    setdata = 0;
    sympsu_set_data(module);
  }

  //set output if desired
  if(setout)
  {
    setout = 0;
    if(outstate)
    {
      send_cmd(ADDR_SymPSU + module, CMD_ENOUT);
    }
    else
    {
      send_cmd(ADDR_SymPSU + module, CMD_DISOUT);
    }
  }

}

void sympsu_get_data(uint8_t ch)
{
  for (uint8_t i = 0; i < 4; i++)
  {
    uint8_t box;
    if(i == 0)
      box = 4;
    else if (i == 1)
      box = 6;
    else if (i == 2)
      box = 5;
    else
      box = 7;
    send_get(ADDR_SymPSU + ch, CMD_GET16, V_POS_GET + i);
    uint32_t temp_time = HAL_GetTick();
    while (!send_ok && ((HAL_GetTick() - temp_time) < 5));
    if (send_ok)
    {
      uint8_t len = sprintf(tft_string, "x%d.val=%d", (box + (ch*8)), rec_16);
      tft_send(tft_string, len);
    }
  }
}

void sympsu_set_data(uint8_t ch)
{
  send_set16(ADDR_SymPSU + ch, V_POS_SET, sympsu_vpos_set);
  send_set16(ADDR_SymPSU + ch, I_POS_SET, sympsu_ipos_set);
  send_set16(ADDR_SymPSU + ch, V_NEG_SET, sympsu_vneg_set);
  send_set16(ADDR_SymPSU + ch, I_NEG_SET, sympsu_ineg_set);
}

/**
 * TFT-Steuerung SMPS:
 * SMPS ausgewählt:
 * 55 04
 * Kanäle sichtbar schalten:
 * 	"vis t<43-46>,0" + ff ff ff zum aktivieren
 * 	"vis t<43-46>,1" + ff ff ff zum deaktivieren
 * Daten für Spannungen/Ströme:
 * 0x5C + <01-04> <LSB_MSB Vpos> <LSB_MSB Ipos>
 * Messwerte anzeigen: "<boxname>.val=<Zahlenwert in ASCII>" + ff ff ff
 * Zahlenwert in 1/100
 *        |  Kanal 1  | Kanal 2 | Kanal 3 | Kanal 4 |
 * -------+-----------+---------+---------+---------+
 *  V_POS | x2		    | x6		  | x10	    | x14     |
 *  I_POS | x3		    | x7		  | x11	    | x15		  |
 *
 * Spannung freischalten: 0x50 + <10/11> <20/21> <30/31> <40/41>
 *
 */

void enable_smps(uint8_t ch)
{
  uint8_t len = sprintf(tft_string, "vis t%d,0", 43 + ch);
  tft_send(tft_string, len);
}

void disable_smps(uint8_t ch)
{
  uint8_t len = sprintf(tft_string, "vis t%d,1", 43 + ch);
  tft_send(tft_string, len);
}

void smps_page_loop(void)
{
  //enable channels on tft at startup
  if (enableblock == 1)
  {
    enableblock = 0;
    for (int i = 0; i < 4; i++)
    {
      if (sim_debug)
      {
        enable_smps(i);
      }
      else if ((dev_smps >> i) & 0x01)
      {
        enable_smps(i);
      }
      else
      {
        disable_smps(i);
      }
    }
    pagetimer = HAL_GetTick();
  }

  //periodically get voltage and current values
  if(readdata)
  {
    for(int j = 0; j < 4; j++)
    {
      if((dev_smps >> j) & 0x01)
      {
        smps_get_data(j);
      }
    }
    readdata = 0;
  }

  //set values if received
  if(setdata)
  {
    setdata = 0;
    smps_set_data(module);
  }

  //set output if desired
  if(setout)
  {
    setout = 0;
    if(outstate)
    {
      send_cmd(ADDR_SMPS + module, CMD_ENOUT);
    }
    else
    {
      send_cmd(ADDR_SMPS + module, CMD_DISOUT);
    }
  }

}

void smps_get_data(uint8_t ch)
{
  for (uint8_t i = 0; i < 2; i++)
  {
    uint8_t box;
    if(i == 0)
      box = 2;
    else
      box = 3;
    send_get(ADDR_SMPS + ch, CMD_GET16, V_Get + i);
    uint32_t temp_time = HAL_GetTick();
    while (!send_ok && ((HAL_GetTick() - temp_time) < 5));
    if (send_ok)
    {
      uint8_t len = sprintf(tft_string, "x%d.val=%d", (box + (ch*4)), rec_16);
      tft_send(tft_string, len);
    }
  }
}

void smps_set_data(uint8_t ch)
{
  send_set16(ADDR_SMPS + ch, V_Set, smps_v_set);
  send_set16(ADDR_SMPS + ch, I_Set, smps_i_set);
}


/**
 * TFT-Steuerung E-Load:
 * E-Load ausgewählt:
 * 55 05
 * Kanäle sichtbar schalten:
 * 	"vis t<51-54>,0" + ff ff ff zum aktivieren
 * 	"vis t<51-54>,1" + ff ff ff zum deaktivieren
 * Daten für Spannungen/Ströme:
 * 0x5C + <01-04> <LSB_MSB Vpos> <LSB_MSB Ipos>
 * Messwerte anzeigen: "<boxname>.val=<Zahlenwert in ASCII>" + ff ff ff
 * Zahlenwert in 1/100
 *        |  Kanal 1  | Kanal 2 | Kanal 3 | Kanal 4 |
 * -------+-----------+---------+---------+---------+
 *  CH 1  | x1		    | x5		  | x9 	    | x13     |
 *  CH 2  | x3		    | x7		  | x11	    | x15		  |
 *  Temp  | n0        | n1      | n2      | n3      |
 *
 */

void enable_load(uint8_t ch)
{
  uint8_t len = sprintf(tft_string, "vis t%d,0", 51 + ch);
  tft_send(tft_string, len);
}

void disable_load(uint8_t ch)
{
  uint8_t len = sprintf(tft_string, "vis t%d,1", 51 + ch);
  tft_send(tft_string, len);
}

void load_page_loop(void)
{
  //enable channels on tft at startup
  if (enableblock == 1)
  {
    enableblock = 0;
    for (int i = 0; i < 4; i++)
    {
      if (sim_debug)
      {
        enable_load(i);
      }
      else if ((dev_load >> i) & 0x01)
      {
        enable_load(i);
      }
      else
      {
        disable_load(i);
      }
    }
    pagetimer = HAL_GetTick();
  }

  //periodically get current values
  if(readdata)
  {
    for(int j = 0; j < 4; j++)
    {
      if((dev_load >> j) & 0x01)
      {
        load_get_data(j);
      }
    }
    readdata = 0;
  }

  //set values if received
  if(setdata)
  {
    setdata = 0;
    load_set_data(module);
  }

  //set output if desired
  if(setout)
  {
    setout = 0;
    if(outstate)
    {
      send_cmd(ADDR_Load + module, CMD_ENOUT);
    }
    else
    {
      send_cmd(ADDR_Load + module, CMD_DISOUT);
    }
  }
}

void load_get_data(uint8_t ch)
{
  
  send_get(ADDR_Load + ch, CMD_GET16, LOAD_CH0_Get);
  uint32_t temp_time = HAL_GetTick();
  while (!send_ok && ((HAL_GetTick() - temp_time) < 5));
  if (send_ok)
  {
    uint8_t len = sprintf(tft_string, "x%d.val=%d", (1 + (ch*4)), rec_16);
    tft_send(tft_string, len);
  }

  send_get(ADDR_Load + ch, CMD_GET16, LOAD_CH1_Get);
  temp_time = HAL_GetTick();
  while (!send_ok && ((HAL_GetTick() - temp_time) < 5));
  if (send_ok)
  {
    uint8_t len = sprintf(tft_string, "x%d.val=%d", (3 + (ch*4)), rec_16);
    tft_send(tft_string, len);
  }

  send_get(ADDR_Load + ch, CMD_GET8, LOAD_TEMP_Get);
  temp_time = HAL_GetTick();
  while (!send_ok && ((HAL_GetTick() - temp_time) < 5));
  if (send_ok)
  {
    uint8_t len = sprintf(tft_string, "n%d.val=%d", ch, rec_8);
    tft_send(tft_string, len);
  }
  
}

void load_set_data(uint8_t ch)
{
  send_set16(ADDR_Load + ch, LOAD_CH0_Set, load_set_0);
  send_set16(ADDR_Load + ch, LOAD_CH1_Set, load_set_1);
}
