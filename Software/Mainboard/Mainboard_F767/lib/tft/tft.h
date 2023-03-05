/** @file tft.h
 * 
 * @brief Library for TFT Display on USART1 Bus
 *
 * @author Synthron
 * 
 */ 

#ifndef TFT_H
#define TFT_H

#include "usart.h"
#include <stdio.h>

#define PAGE_MAIN   0
#define PAGE_DTEST  1
#define PAGE_FGEN   2
#define PAGE_SYMPSU 3
#define PAGE_SMPS   4

#define CMD_PAGESET 0x55
#define CMD_SETDATA 0x5C
#define CMD_SETOUT  0x50

//general function prototypes
void tft_init (void);
void tft_parse (void);
void tft_send (char commandstring[60], uint8_t len);

//diode tester function prototypes

//fgen function prototypes
void enable_fgen(uint8_t ch);

//sympsu function prototypes
void enable_smps(uint8_t ch);

//smps function prototypes

//general variables
uint8_t RxBuffer[30];
char tft_string[60];
uint8_t tft_page;
uint8_t tft_data;
uint8_t tft_cmd;
uint8_t module;
uint8_t outstate;
uint8_t setdata; // Indicator for data received
uint8_t setout;  // indicator for Output to set

//Diode Tester Variables
uint8_t start_index;
uint8_t stop_index;
uint8_t diode_mode;
uint16_t diode_values[15];

//Function Generator Variables
uint32_t frequency;
uint16_t amplitude;
uint16_t offset;


//SYMPSU Variables
uint16_t sympsu_vpos_set;
uint16_t sympsu_vneg_set;
uint16_t sympsu_ipos_set;
uint16_t sympsu_ineg_set;

uint16_t sympsu_vpos_get[4];
uint16_t sympsu_vneg_get[4];
uint16_t sympsu_ipos_get[4];
uint16_t sympsu_ineg_get[4];

//SMPS Variables


#endif /* TFT_H */

/*** end of file ***/
