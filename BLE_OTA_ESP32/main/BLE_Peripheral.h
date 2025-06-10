/* @file  BLE_Peripheral.h
   @brief header file for BLE peripheral with GATT server  
   @author Avinashee Tech
*/
#ifndef _main_h_
#define _main_h_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//OTA state enum
enum{
    OTA_NOP,
    OTA_REQUEST,
    OTA_REQUEST_ACK,
    OTA_REQUEST_NAK,
    OTA_DONE,
    OTA_DONE_ACK,
    OTA_DONE_NAK
};


//Attributes State Machine for server attribute table

/*Device Info*/
enum{
    IDX_SVC_INFO,
    IDX_CHAR_MANUF_NAME,
    IDX_CHAR_MANUF_NAME_VAL,
    IDX_CHAR_MODEL_NUM,
    IDX_CHAR_MODEL_NUM_VAL,
    IDX_NB_INFO
};

/*OTA*/
enum{

    IDX_SVC_OTA,
    IDX_CHAR_CTRL,
    IDX_CHAR_VAL_CTRL,
    IDX_CHAR_CFG_CTRL,
    IDX_CHAR_DATA,
    IDX_CHAR_VAL_DATA,
    IDX_NB_OTA
};

#endif