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
#include "modlab.h"
#include "usb_mon.h"
#include <string.h>

#define PAGE_MAIN   0
#define PAGE_DTEST  1
#define PAGE_FGEN   2
#define PAGE_SYMPSU 3
#define PAGE_SMPS   4
#define PAGE_LOAD   5

#define CMD_PAGESET 0x55
#define CMD_SETDATA 0x5C
#define CMD_SETOUT  0x50
#define CMD_DEBUG   0x5A
#define CMD_READDAT 0x53

//general function prototypes
void tft_init (void);
void tft_parse (void);
void tft_send (char commandstring[60], uint8_t len);
void main_page_loop(void);
void tft_reset(void);

//diode tester function prototypes
void enable_diode(uint8_t ch);
void disable_diode(uint8_t ch);
void diode_page_loop(void);

//fgen function prototypes
void enable_fgen(uint8_t ch);
void disable_fgen(uint8_t ch);
void fgen_setdata(void);
void fgen_page_loop(void);

//sympsu function prototypes
void enable_sympsu(uint8_t ch);
void disable_sympsu(uint8_t ch);
void sympsu_page_loop (void);
void sympsu_get_data(uint8_t ch);
void sympsu_set_data(uint8_t ch);

//smps function prototypes
void enable_smps(uint8_t ch);
void disable_smps(uint8_t ch);
void smps_page_loop(void);
void smps_get_data(uint8_t ch);
void smps_set_data(uint8_t ch);

//load function prototypes
void enable_load(uint8_t ch);
void disable_load(uint8_t ch);
void load_page_loop(void);
void load_set_data(uint8_t ch);
void load_get_data(uint8_t ch);

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
uint8_t setdebug; // indicator for debug settings
uint8_t readdata; // indicator to load cyclic data to TFT
uint8_t loopblock;
uint8_t enableblock;
uint32_t pagetimer;

uint8_t can_debug;
uint8_t tft_debug;
uint8_t sim_debug;

uint8_t dev_diode;
uint8_t dev_fgen;
uint8_t dev_sympsu;
uint8_t dev_smps;
uint8_t dev_load;

//Diode Tester Variables
uint8_t start_index;
uint8_t stop_index;
uint8_t diode_mode;
uint16_t diode_values[15];

//Function Generator Variables
uint8_t wave;
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
uint16_t smps_v_set;
uint16_t smps_i_set;
uint16_t smps_v_get[4];
uint16_t smps_i_get[4];

//Load Variables
uint16_t load_set_0;
uint16_t load_set_1;

#endif /* TFT_H */

/*** end of file ***/
