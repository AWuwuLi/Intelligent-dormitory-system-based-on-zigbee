/**************************************************************************************************
  Filename:       SampleApp.c
  Revised:        $Date: 2009-03-18 15:56:27 -0700 (Wed, 18 Mar 2009) $
  Revision:       $Revision: 19453 $

  Description:    Sample Application (no Profile).


  Copyright 2007 Texas Instruments Incorporated. All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights
  granted under the terms of a software license agreement between the user
  who downloaded the software, his/her employer (which must be your employer)
  and Texas Instruments Incorporated (the "License").  You may not use this
  Software unless you agree to abide by the terms of the License. The License
  limits your use, and you acknowledge, that the Software may not be modified,
  copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio
  frequency transceiver, which is integrated into your product.  Other than for
  the foregoing purpose, you may not use, reproduce, copy, prepare derivative
  works of, modify, distribute, perform, display or sell this Software and/or
  its documentation for any purpose.cs tx

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE
  PROVIDED  AS IS?WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED,
  INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF MERCHANTABILITY, TITLE,
  NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE. IN NO EVENT SHALL
  TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER
  LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
  INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE
  OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT
  OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
  (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/*********************************************************************
  This application isn't intended to do anything useful, it is
  intended to be a simple example of an application's structure.

  This application sends it's messages either as broadcast or
  broadcast filtered group messages.  The other (more normal)
  message addressing is unicast.  Most of the other sample
  applications are written to support the unicast message model.

  Key control:
    SW1:  Sends a flash command to all devices in Group 1.
    SW2:  Adds/Removes (toggles) this device in and out
          of Group 1.  This will enable and disable the
          reception of the flash command.
*********************************************************************/

/*********************************************************************
 * INCLUDES
 */
#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "MT_UART.h"
#include "MT_APP.h"
#include "MT.h"
#include "DHT11.h"
#include "stdio.h"
#include "string.h"
uint8 led1_flag = 0;
uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;
uint8 led2_flag = 0;
/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// This list should be filled with Application specific Cluster IDs.
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
    {
        SAMPLEAPP_PERIODIC_CLUSTERID,
        SAMPLEAPP_FLASH_CLUSTERID};

const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
    {
        SAMPLEAPP_ENDPOINT,             //  int Endpoint;
        SAMPLEAPP_PROFID,               //  uint16 AppProfId[2];
        SAMPLEAPP_DEVICEID,             //  uint16 AppDeviceId[2];
        SAMPLEAPP_DEVICE_VERSION,       //  int   AppDevVer:4;
        SAMPLEAPP_FLAGS,                //  int   AppFlags:4;
        SAMPLEAPP_MAX_CLUSTERS,         //  uint8  AppNumInClusters;
        (cId_t *)SampleApp_ClusterList, //  uint8 *pAppInClusterList;
        SAMPLEAPP_MAX_CLUSTERS,         //  uint8  AppNumInClusters;
        (cId_t *)SampleApp_ClusterList  //  uint8 *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in SampleApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t SampleApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 SampleApp_TaskID; // Task ID for internal task/event processing
                        // This variable will be received when
                        // SampleApp_Init() is called.
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID; // This is the unique message ID (counter)

afAddrType_t SampleApp_Periodic_DstAddr; // ?
afAddrType_t SampleApp_Flash_DstAddr;    // ��
afAddrType_t SampleApp_P2P_DstAddr;      // ?

aps_Group_t SampleApp_Group;



/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys(uint8 shift, uint8 keys);
void SampleApp_MessageMSGCB(afIncomingMSGPacket_t *pckt);
void SampleApp_SendPeriodicMessage(void);
void SampleApp_SendFlashMessage(uint16 flashTime);
/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SampleApp_Init(uint8 task_id)
{

  unsigned char tmp[10];
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;

  MT_UartInit();                  //   ? ?
  MT_UartRegisterTaskID(task_id); //? ?
  HalUARTWrite(0, "EndDevice-2\r\n", 13);
  osal_memset(tmp, 0, 10);
  tmp[0] = HAL_UART_DMA + 0x30;
  tmp[1] = HAL_UART_ISR + 0x30;
  tmp[2] = HAL_UART_USB + 0x30;
  HalUARTWrite(0, tmp, 6);
  P0SEL &= 0x7f; // P0_7   �� ?  io

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

#if defined(BUILD_ALL_DEVICES)
  // The "Demo" target is setup to have BUILD_ALL_DEVICES and HOLD_AUTO_START
  // We are looking at a jumper (defined in SampleAppHw.c) to be jumpered
  // together - if they are - we will start up a coordinator. Otherwise,
  // the device will start as a router.
  if (readCoordinatorJumper())
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES

#if defined(HOLD_AUTO_START)
  // HOLD_AUTO_START is a compile option that will surpress ZDApp
  //  from starting the device and wait for the application to
  //  start the device.
  ZDOInitDevice(0);
#endif

  // Setup for the periodic message's destination address
  // Broadcast to everyone
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = 0x0000;

  // Setup for the flash command's destination address - Group 1
  SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;
  SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;

  SampleApp_P2P_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast; // ?
  SampleApp_P2P_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_P2P_DstAddr.addr.shortAddr = 0xFFFF; //    ��

  // Fill out the endpoint description.
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_epDesc.task_id = &SampleApp_TaskID;
  SampleApp_epDesc.simpleDesc = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
  SampleApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister(&SampleApp_epDesc);

  // Register for all key events - This app will handle all key events
  RegisterForKeys(SampleApp_TaskID);

  // By default, all devices start out in Group 1
  SampleApp_Group.ID = 0x0001;
  osal_memcpy(SampleApp_Group.name, "Group 1", 7);
  aps_AddGroup(SAMPLEAPP_ENDPOINT, &SampleApp_Group);

#if defined(LCD_SUPPORTED)
  HalLcdWriteString("SampleApp", HAL_LCD_LINE_1);
#endif
}

uint16 SampleApp_ProcessEvent(uint8 task_id, uint16 events)
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id; // Intentionally unreferenced parameter

  if (events & SYS_EVENT_MSG)
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive(SampleApp_TaskID);
    while (MSGpkt)
    {
      switch (MSGpkt->hdr.event)
      {
      // Received when a key is pressed
      case KEY_CHANGE:
        SampleApp_HandleKeys(((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys);
        break;

      case CMD_SERIAL_MSG:
        mtOSALSerialData_t *p = (mtOSALSerialData_t *)MSGpkt; // 串口数据指针
        HalUARTWrite(HAL_UART_PORT_0, &p->msg[1], p->msg[0]); // 向串�?发送数�?        uint8 len = p->msg[0];                                // 获取数据长度
        uint8 *data_ptr = &p->msg[1];                         // 指向实际数据
        if (memcmp(data_ptr, "OFF", strlen("OFF")) == 0)
        {
          HalLedSet(HAL_LED_3, HAL_LED_MODE_ON);
        }
        else if (memcmp(data_ptr, "ON", strlen("ON")) == 0)
        {
          HalLedSet(HAL_LED_3, HAL_LED_MODE_OFF);
        }
        break;

      default:
        break;
      }

      // Release the memory
      osal_msg_deallocate((uint8 *)MSGpkt);

      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive(SampleApp_TaskID);
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }
  return 0;
}

void SampleApp_HandleKeys(uint8 shift, uint8 keys)
{
  (void)shift; // Intentionally unreferenced parameter

  if (keys & HAL_KEY_SW_6)
  {

    led1_flag = !led1_flag;
    HalLedSet(HAL_LED_1, led1_flag);
    SampleApp_SendFlashMessage(0);
  }
  if (keys & HAL_KEY_SW_7)
  {

    led2_flag = !led2_flag;
    HalLedSet(HAL_LED_2, led2_flag);
  }
}

void SampleApp_SendFlashMessage(uint16 flashTime)
{

  if (AF_DataRequest(&SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                     SAMPLEAPP_FLASH_CLUSTERID,
                     1,
                     &led1_flag,
                     &SampleApp_TransID,
                     AF_DISCV_ROUTE,
                     AF_DEFAULT_RADIUS) == afStatus_SUCCESS)
  {
    // HalLedBlink(HAL_LED_2, 3, 50, 500);
  }
  else
  {
    // Error occurred in request to send.
  }
}

/*********************************************************************
*********************************************************************/
