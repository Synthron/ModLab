/** @file modlab.h
 * 
 * @brief Library for ModLab for Devices
 *
 * @author Synthron
 * 
 */ 

#ifndef MODLAB_H
#define MODLAB_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "tim.h"
#include "can.h"
#ifdef SYMPSU
#include "adc.h"
#endif
#ifdef SMPS
#include "adc.h"
#endif
#ifdef LOAD
#include "adc.h"
#endif

//Device Defines
#define ADDR_HBC 0x510
#define ADDR_DiodeTester 0x520
#define ADDR_SymPSU 0x530
#define ADDR_SMPS 0x53C
#define ADDR_WaveGen 0x540
#define ADDR_Load 0x550

//Message Types Define
#define ADDR_AUTO 0x700
#define ADDR_ACK  0x600
#define ADDR_NACK 0x300


//Command Defines
#define CMD_RESET   0x01 //Reset Module
#define CMD_PING    0x05 //Reset Module
#define CMD_SET8    0x10 //Set 8bit
#define CMD_GET8    0x11 //Get 8bit
#define CMD_SET16   0x14 //Set 16bit
#define CMD_GET16   0x15 //get 16bit
#define CMD_SET32   0x18 //set 32bit
#define CMD_GET32   0x18 //get 32bit
#define CMD_GETSTAT 0x20 //Get Status
#define CMD_ENOUT   0x40 //Enable Output
#define CMD_DISOUT  0x41 //Disable Output
#define CMD_SETMODE 0x60 //Set Operation mode
#define CMD_GETMODE 0x61 //Get Operation Mode

//Auto Message Defines
#define AUTO_DONE   0x10 // Conversion finished

//Diode Tester Params
#define MA2         0x10
#define MA5         0x11
#define MA7         0x12
#define MA10        0x13
#define MA12        0x14
#define MA15        0x15
#define MA17        0x16
#define MA20        0x17
#define MA22        0x18
#define MA25        0x19
#define MA27        0x1A
#define MA30        0x1B
#define MA32        0x1C
#define MA35        0x1D
#define MA37        0x1E
#define START_CURR  0x20
#define END_CURR    0x21
#define MODE_SINGLE 0
#define MODE_COARSE 1
#define MODE_MEDIUM 2
#define MODE_FINE   3

//SymPSU Params
#define V_POS_SET 0x10
#define I_POS_SET 0x11
#define V_NEG_SET 0x12
#define I_NEG_SET 0x13
#define V_POS_GET 0x20
#define I_POS_GET 0x21
#define V_NEG_GET 0x22
#define I_NEG_GET 0x23

//SMPS Params
#define V_Set 0x10
#define I_Set 0x11
#define V_Get 0x20
#define I_Get 0x21

//LOAD Params
#define LOAD_CH0_Set 0x10
#define LOAD_CH1_Set 0x11
#define LOAD_CH0_Get 0x20
#define LOAD_CH1_Get 0x21
#define LOAD_TEMP_Get 0x22

//Waveform Params
#define FREQUENCY   0x10
#define GAINOUT     0x20
#define OFFSETOUT   0x30
#define SQUAREWAVE     0
#define SINEWAVE       1
#define TRIANGLEWAVE   3

CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;

void timer_init();
void delay_us(uint16_t us);

void send_cmd (uint16_t dev, uint8_t cmd);
void send_rec (uint16_t dev, uint8_t cmd, uint8_t stat);
void send_rec8 (uint16_t dev, uint8_t cmd, uint8_t stat, uint8_t val);
void send_rec16 (uint16_t dev, uint8_t cmd, uint8_t stat, uint16_t val);
void send_get (uint16_t dev, uint8_t cmd, uint8_t reg);
void send_set8 (uint16_t dev, uint8_t reg, uint8_t val);
void send_set16 (uint16_t dev, uint8_t reg, uint16_t val);
void transmit(void);

#ifdef SYMPSU
void ADC_Select_CH0 (void);
void ADC_Select_CH1 (void);
void ADC_Select_CH2 (void);
void ADC_Select_CH3 (void);
#endif

#ifdef SMPS
void ADC_Select_CH0 (void);
void ADC_Select_CH1 (void);
#endif

#ifdef LOAD
void ADC_Select_CH0 (void);
void ADC_Select_CH1 (void);
void ADC_Select_CH2 (void);
#endif

#ifdef __cplusplus
}
#endif


#endif /* MODLAB_H */

/*** end of file ***/
