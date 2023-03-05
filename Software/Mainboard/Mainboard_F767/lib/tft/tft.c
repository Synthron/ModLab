#include "tft.h"

//Initialize Variables
void tft_init(void)
{
    //general variables
    tft_page = 0;
    tft_data = 0;
    tft_cmd = 0;
    module = 0;
    outstate = 0;
    setdata = 0; // Indicator for data received
    setout = 0;  // indicator for Output to set

    //Diode Tester Variables
    start_index = 0;
    stop_index = 0;
    diode_mode = 0;

    //Function Generator Variables
    frequency = 0;
    amplitude = 0;
    offset = 0;
}


void tft_parse(void)
{
      //get Command and look for further data
  if(!tft_data)
  {
    tft_cmd = RxBuffer[0];
    tft_data = 1;
    if(tft_cmd == CMD_PAGESET)
    {
      HAL_UART_Receive_IT(&huart1, RxBuffer, 1);
    }
    else if(tft_cmd == CMD_SETDATA)
    {
      if(tft_page == PAGE_DTEST)
      {
        HAL_UART_Receive_IT(&huart1, RxBuffer, 4);
      }
      else if(tft_page == PAGE_FGEN)
      {
        HAL_UART_Receive_IT(&huart1, RxBuffer, 9);
      }
      else if(tft_page == PAGE_SYMPSU)
      {
        HAL_UART_Receive_IT(&huart1, RxBuffer, 9);
      }
    }
    else if(tft_cmd == CMD_SETOUT)
    {
      HAL_UART_Receive_IT(&huart1, RxBuffer, 1);
    }
  }
  //get Data to command
  else
  {
    if(tft_cmd == CMD_PAGESET)
    {
      tft_page = RxBuffer[0];
    }
    else if(tft_cmd == CMD_SETDATA)
    {
      if(tft_page == PAGE_DTEST)
      {
        module = RxBuffer[0]-1;
        diode_mode = RxBuffer[1];
        start_index = RxBuffer[2];
        stop_index = RxBuffer[3];
      }
      else if(tft_page == PAGE_FGEN)
      {
        module = RxBuffer[0]-1;
        frequency = (RxBuffer[4] << 24) + (RxBuffer[3] << 16) + (RxBuffer[2] << 8) + (RxBuffer[1]);
        amplitude = (RxBuffer[6] << 8) + (RxBuffer[5]);
        offset = (RxBuffer[8] << 8) + (RxBuffer[7]);
      }
      else if(tft_page == PAGE_SYMPSU)
      {
        //<01-04> <LSB_MSB Vpos> <LSB_MSB Ipos> <LSB_MSB Vneg> <LSB_MSB Ineg>
        module = RxBuffer[0]-1;
        sympsu_vpos_set = (RxBuffer[2] << 8) + (RxBuffer[1]);
        sympsu_ipos_set = (RxBuffer[4] << 8) + (RxBuffer[3]);
        sympsu_vneg_set = (RxBuffer[6] << 8) + (RxBuffer[5]);
        sympsu_ineg_set = (RxBuffer[8] << 8) + (RxBuffer[7]);
      }
      setdata = 1;
    }
    else if(tft_cmd == CMD_SETOUT)
    {
      module = ((RxBuffer[0] & 0xF0) >> 4)-1;
      outstate = RxBuffer[0] & 0x0F;
      setout = 1;
    }
    
  }
}

void tft_send(char commandstring[60], uint8_t len)
{
    uint8_t CmdEnd[3] = {0xff,0xff,0xff};
    HAL_UART_Transmit(&huart1, (uint8_t*)commandstring, len,1);
    HAL_UART_Transmit(&huart1, CmdEnd, 3,1);

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
    uint8_t len = sprintf(tft_string, "vis t%d,0", 47+ch);
    tft_send(tft_string, len);
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

void enable_smps(uint8_t ch)
{
    uint8_t len = sprintf(tft_string, "vis t%d,0", 79+ch);
    tft_send(tft_string, len);
}