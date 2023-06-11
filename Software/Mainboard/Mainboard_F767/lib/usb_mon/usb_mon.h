/** @file usb_mon.h
 * 
 * @brief Library for USB Monitor for ModLab
 *
 * @author Synthron
 * 
 */ 

#ifndef USB_MON_H
#define USB_MON_H

#include <stdio.h>
#include <string.h>
#include "tft.h"
#include "modlab.h"


// overview, send CAN commands
uint8_t mon_page;
//            | debugs 
//  | | | | | | tft | can
uint8_t mon_flags; 

#endif /* USB_MON_H */

/*** end of file ***/
