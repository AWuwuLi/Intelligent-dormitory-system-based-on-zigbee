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
#include <stdlib.h>

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//需要修改的是下面5行

#define  token    "version=2018-10-31&res=products%2F9dBJ0I0C3F%2Fdevices%2Fyinxingru&et=1751180218&method=md5&sign=JLWxKRqwjaSDZIrUqwrgVg%3D%3D" //token工具计算值
#define  devid    "yinxingru"           //新版本onenet平台设备id
#define  proid     "9dBJ0I0C3F"                 //新版本onenet平台产品id
#define  LYSSID    "yinxingru"                   // 修改你路由器的SSId
#define  LYPASSWD  "woshiyxr"  
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define WIFIRESET   P0_6             // P0.6口控制wifi reset
//WIFI连接命令
#define AT_C       "AT\r\n"
#define CWMODE_C   "AT+CWMODE=1\r\n"    //STA模式
#define CIPMODE_C  "AT+CIPMODE=0\r\n"   //非透传模式
#define MQTTCONN "AT+MQTTCONN=0,\"mqtts.heclouds.com\",1883,1\r\n"//连接mqtt服务器

#define RELAY P0_6 //继电器控制引脚
char ibusy = 0;  //判断是否在发送数据
int kmg=0;  
uint8 flat = 1;
uint8 relay,hum_old,hum_new;  
uint8 adc=0;
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

// 应用程序支持的Cluster ID列表
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_PERIODIC_CLUSTERID,
  SAMPLEAPP_FLASH_CLUSTERID
};

const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
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
uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SampleApp_Init() is called.
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SampleApp_Periodic_DstAddr; //广播
afAddrType_t SampleApp_Flash_DstAddr;    //组播
afAddrType_t SampleApp_P2P_DstAddr;      //点播

aps_Group_t SampleApp_Group;

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;



/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendFlashMessage( uint16 flashTime );
void ReSetWifi(void);
void ClearRAM(uint8* ram,uint32 n);
/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */


/****************************************************************************
* 名    称: ReSetWifi()
* 功    能: 低电平复位wifi模块
* 入口参数: 无
* 出口参数: 无
****************************************************************************/
void ReSetWifi(void)
{
  //P0DIR |= 0x40;                  //P0.6定义为输出
  //WIFIRESET = 0;                  //低电平复位---------------------
 //Delay_ms(500);
 // WIFIRESET = 1;                  //高电平工作------------
  Delay_ms(500);
}
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
void SampleApp_Init( uint8 task_id )
{ 
  
  unsigned char tmp[10];
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;
  
  MT_UartInit();                  //串口初始化
  MT_UartRegisterTaskID(task_id); //注册串口任务
  HalUARTWrite(0, "CoordinatorZB\r\n", 15);
  osal_memset(tmp,0,10);
  tmp[0] = HAL_UART_DMA+0x30;
  tmp[1] = HAL_UART_ISR+0x30;
  tmp[2] = HAL_UART_USB+0x30;
  HalUARTWrite(0, tmp, 6);
    
   P2SEL &= 0xfe;                  //P0_7配置成通用io 温湿度
   ReSetWifi();
   Delay_ms(1000);                //等待模块启动
   
   HalUARTWrite(1,"AT\r\n",strlen("AT\r\n")); //发送AT检测WiFi模块

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

 #if defined ( BUILD_ALL_DEVICES )
  // The "Demo" target is setup to have BUILD_ALL_DEVICES and HOLD_AUTO_START
  // We are looking at a jumper (defined in SampleAppHw.c) to be jumpered
  // together - if they are - we will start up a coordinator. Otherwise,
  // the device will start as a router.
  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES

#if defined ( HOLD_AUTO_START )
  // HOLD_AUTO_START is a compile option that will surpress ZDApp
  //  from starting the device and wait for the application to
  //  start the device.
  ZDOInitDevice(0);
#endif

//  Setup for the periodic message's destination address
//  Broadcast to everyone
//  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
//  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
//  SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = 0x0000; 
  

  // Setup for the flash command's destination address - Group 1
  SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;
  SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;
  
  SampleApp_P2P_DstAddr.addrMode = (afAddrMode_t)Addr16Bit; //点播 
  SampleApp_P2P_DstAddr.endPoint = SAMPLEAPP_ENDPOINT; 
  SampleApp_P2P_DstAddr.addr.shortAddr = 0x0000;            //发给协调器

  // Fill out the endpoint description.
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_epDesc.task_id = &SampleApp_TaskID;
  SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
  SampleApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &SampleApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( SampleApp_TaskID );

  // By default, all devices start out in Group 1
  SampleApp_Group.ID = 0x0001;
  osal_memcpy( SampleApp_Group.name, "Group 1", 7 );
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );

#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "SampleApp", HAL_LCD_LINE_1 );
#endif
}


/*********************************************************************
 * @fn      SampleApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */

uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
  char ATSTRCSTX[256];  //mqtt连接缓存
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter
  kmg++;
  if(kmg>3000)
  {
     kmg=0;
     SystemReset();
  }
  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        // Received when a key is pressed
        case KEY_CHANGE:
          SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          LCD_P8x16Str(0, 0, "2022442778.com ");
          break;

        // Received when a messages is received (OTA) for this endpoint
        case AF_INCOMING_MSG_CMD:
          SampleApp_MessageMSGCB( MSGpkt );
          break;
      case CMD_SERIAL_MSG:
        mtOSALSerialData_t *p = (mtOSALSerialData_t *)MSGpkt;
        HalUARTWrite(HAL_UART_PORT_0, &p->msg[1], p->msg[0]); 
        //把接收到的数据串口1打印出来 前面是字符串 后面是长度
        if(flat==6) //收到WIFI和平台交互数据
        LCD_P8x16Str(0, 4, "Get UART1 Data .");
        LCD_P8x16Str(0, 6, "                ");
        LCD_P8x16Str(0, 6, LYSSID);
        if(flat==1) //发AT命令 模式設置STA
        {
           if(strstr((char const *)(&p->msg[1]),"OK"))
           {
              HalUARTWrite(1,CWMODE_C,strlen(CWMODE_C));
              HalUARTWrite(0, "ZIGBEE-WIFI OK\r\n", 16);
              flat=2;
              Delay_ms(500);
           }
           else if(strstr((char const *)(&p->msg[1]),"MQTTCONNECTED"))  //失败就重启
           {
       
              flat=5;    
           }
            else if(strstr((char const *)(&p->msg[1]),"ERROR"))  //失败就重启
           {
              ReSetWifi();//重启WiFi
              Delay_ms(1000);
              HalUARTWrite(1,"AT\r\n",4);
              flat=1;    
           }
        }
         else if(flat==2) //链接WiFi路由器
        {
           if(strstr((char const *)(&p->msg[1]),"OK"))
           {
              memset(ATSTRCSTX,0,256);
              sprintf(ATSTRCSTX,"AT+CWJAP=\"%s\",\"%s\"\r\n",LYSSID,LYPASSWD);
              HalUARTWrite(1,(unsigned char*)ATSTRCSTX, strlen(ATSTRCSTX));//连接本地WiFi
              flat=3;
              Delay_ms(1000);Delay_ms(1000);
           }
        }
        
         else if(flat==3) //設置為發AT命令模式传输模式
        {
           if(strstr((char const *)(&p->msg[1]),"OK"))
           { 
             //设置普通传输模式 也就是發AT命令模式      
             HalUARTWrite(1,CIPMODE_C, strlen(CIPMODE_C));
            
             flat=4;
             Delay_ms(500);
           }
           else 
           {
              ReSetWifi();//重启WiFi
              Delay_ms(1000);
              HalUARTWrite(1,"AT\r\n",4);
              flat=1;    
           }
        }
        else if(flat==4)  
        {
           if(strstr((char const *)(&p->msg[1]),"OK"))
           {
             //发送AT指令 设置mqtt登录信息 配置秘鑰 和設備id和產品id 創思通信
             memset(ATSTRCSTX,0,256);
             sprintf(ATSTRCSTX,"AT+MQTTUSERCFG=0,1,\"%s\",\"%s\",\"%s\",0,0,\"\"\r\n",devid,proid,token);
             HalUARTWrite(1,(unsigned char*)ATSTRCSTX, strlen(ATSTRCSTX));
             Delay_ms(1000);
             //发送AT指令 登录mqtt
             HalUARTWrite(1,MQTTCONN, strlen(MQTTCONN));//創思連接到ONENET這個是ONENET的域名
             flat=5;
             Delay_ms(500);
           }
        }
         else if(flat==5) 
        {
           if(strstr((char const *)(&p->msg[1]),"MQTTCONNECTED"))
           {   
             //发送AT指令，订阅mqtt主题
             memset(ATSTRCSTX,0,256);
             sprintf(ATSTRCSTX,"AT+MQTTSUB=0,\"$sys/%s/%s/thing/property/post/reply\",1\r\n",proid,devid);
             HalUARTWrite(1,(unsigned char*)ATSTRCSTX, strlen(ATSTRCSTX));
             flat=6;
             Delay_ms(500);
           }
        }

         break;
        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:  //协调器不执行定时发送命令
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( //(SampleApp_NwkState == DEV_ZB_COORD) ||//协调器不执行定时发送命令
                 (SampleApp_NwkState == DEV_ROUTER)
              || (SampleApp_NwkState == DEV_END_DEVICE) )
          {
            // Start sending the periodic message in a regular interval.
            osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
          }
          else
          {
            // Device is no longer in the network
          }
          break;

        default:
          break;
      }
      Delay_ms(10);
      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in SampleApp_Init()). 协调器不执行定时发送函数
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  {
    // Send the periodic message
    SampleApp_SendPeriodicMessage();

    // Setup to send message again in normal period (+ a little jitter)
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */
/*********************************************************************
 * @fn      SampleApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter
  
  if ( keys & HAL_KEY_SW_1 )
  {
    /* This key sends the Flash Command is sent to Group 1.
     * This device will not receive the Flash Command from this
     * device (even if it belongs to group 1).
     */
    SampleApp_SendFlashMessage( SAMPLEAPP_FLASH_DURATION );
  }

  if ( keys & HAL_KEY_SW_2 )
  {
    /* The Flashr Command is sent to Group 1.
     * This key toggles this device in and out of group 1.
     * If this device doesn't belong to group 1, this application
     * will not receive the Flash command sent to group 1.
     */
    aps_Group_t *grp;
    grp = aps_FindGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    if ( grp )
    {
      // Remove from the group
      aps_RemoveGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    }
    else
    {
      // Add to the flash group
      aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
    }
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
*
 * @param   none
 *
 * @return  none
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  uint16 flashTime;
  char HttpData[256];
  char check[3];//这个数组用于检查温湿度数据是否正确以及用于合并多个终端数据的中间变量
  char tem[3];
  char hum[3];
  char guang[2];
  //char kaiguan[2];
  uint8 led1_flag;
  P0DIR |= 0x40; // 配置P0.6为输出
  memset(HttpData,0,256);
  memset(tem,0,3);
  memset(hum,0,3);
  switch ( pkt->clusterId )
  {
    case SAMPLEAPP_P2P_CLUSTERID:

      break;    
    case SAMPLEAPP_PERIODIC_CLUSTERID:
      HalUARTWrite(0, "\r\n================\r\n", 20);                          //提示接收到数据
      HalUARTWrite(0, pkt->cmd.Data, pkt->cmd.DataLength);                      //输出接收到的数据{"tem1":23,"hum1":58} 长度是21
      HalUARTWrite(0, "\r\n================\r\n", 20);                          //提示接收到数据
      LCD_P8x16Str(0, 6, "                ");
      LCD_P8x16Str(0, 6, LYPASSWD);
      if(ibusy==1) //还在发数据的时候返回
        return ;
       //tcp连接成功并且收到终端数据后向平台发送温湿度
      if(flat==6)
      {
        //flag的作用是设备刚上电第一次读取数据时，读出的数据为“01”，不正确且会被上传，添加flag后设备会在第二次才开始上传数据
        if(pkt->cmd.DataLength==30)
        {
          //memset(check,0,3);
          //strncpy(check,(char const *)pkt->cmd.Data+8,2);         
          if(1)//(check[0]!='0'&&check[1]!='1') //去除温度01 
          {
            //memset(check,0,3);
            //strncpy(check,(char const *)pkt->cmd.Data+18,2);
            if(1)//(check[0]!='0'&&check[1]!='1') //去除湿度01
            {
                ibusy = 1;  //进入发射函数 开始忙碌
                //把接收到的消息进行一个字符串截取操作，截取字符串当中的温湿度数值
                memset(check,0,3);
                strncpy(check,(char const *)pkt->cmd.Data+5,1); //得到终端几
                
                if(check[0]=='1') //终端1的数据
                {
                  strncpy(tem,(char const *)pkt->cmd.Data+8,2);
                  strncpy(hum,(char const *)pkt->cmd.Data+18,2);
                  strncpy(guang,(char const *)pkt->cmd.Data+28,1);
                  hum_new = atoi(hum);
                  //P0DIR |= 0x40;
                  if( hum_new >= 70 /*&& hum_old < 80*/)
                  {
                    //relay ^= 1;
                    //RELAY = relay;
                    RELAY = 1;
                    HalLedBlink(HAL_LED_2, 3, 50, 500);
                  }
                  else
                  {
                    //relay ^= 1;
                    //RELAY = relay;
                    RELAY = 0;
                    //HalLedBlink(HAL_LED_2, 3, 50, 500);
                  }
                  
                  sprintf(HttpData,"AT+MQTTPUB=0,\"$sys/%s/%s/thing/property/post\",\"{\\\"id\\\":\\\"2313\\\"\\\,\\\"version\\\":\\\"1.0\\\"\\\,\\\"params\\\":{\\\"temperature\\\":{\\\"value\\\":%d}\\\,\\\"humidity\\\":{\\\"value\\\":%d}}}\",0,0\r\n",proid,devid, atoi(tem), atoi(hum));
                  HalUARTWrite(1,(unsigned char *) HttpData,strlen(HttpData));
                  Delay_ms(500);
                  sprintf(HttpData,"AT+MQTTPUB=0,\"$sys/%s/%s/thing/property/post\",\"{\\\"id\\\":\\\"2313\\\"\\\,\\\"version\\\":\\\"1.0\\\"\\\,\\\"params\\\":{\\\"adc\\\":{\\\"value\\\":%d}\\\,\\\"guang\\\":{\\\"value\\\":%d}}}\",0,0\r\n",proid,devid, adc, atoi(guang));
                  HalUARTWrite(1,(unsigned char *) HttpData,strlen(HttpData));
                    
                }
                
                
                memset(check,0,3);
                //HalUARTWrite(1,(unsigned char *) HttpData,strlen(HttpData));
                ibusy = 0;//忙碌结束 
            }
          }
         
        }
       
        LCD_P8x16Str(0, 4, "Send NET Data ..");

      }
      else
      {
        SystemReset();
      }
      break;

    case SAMPLEAPP_FLASH_CLUSTERID:
       
          led1_flag = (uint8)pkt->cmd.Data[0];
         /* if(led1_flag == 1)
            HalLedSet(HAL_LED_2,HAL_LED_MODE_ON);
          else
            HalLedBlink(HAL_LED_2, 3, 50, 500);*/
          HalLedSet(HAL_LED_1,led1_flag);
        
      break;
  }
  P0DIR &= 0xbf;
}

/*********************************************************************
 * @fn      SampleApp_SendPeriodicMessage
 *
 * @brief   Send the periodic message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPeriodicMessage( void )
{
  byte i, temp[3], humidity[3], strTemp[21];
  Delay_ms(500);
  DHT11();             //获取温湿度
  Delay_ms(500);
  //将温湿度的转换成字符串,供LCD显示  
  temp[0] = wendu_shi+0x30;
  temp[1] = wendu_ge+0x30;
  temp[2] = '\0';
  humidity[0] = shidu_shi+0x30;
  humidity[1] = shidu_ge+0x30;
  humidity[2] = '\0';
  //将数据整合后方便发给协调器显示
  //{"temp": ,"hum": }
//  osal_memcpy(strTemp, temp, 2); 
//  osal_memcpy(&strTemp[2], "  ", 2);
//  osal_memcpy(&strTemp[4], humidity, 3);
  
   osal_memcpy(strTemp,"{\"temp1\":", 9); 
   osal_memcpy(&strTemp[9],temp, 2);
   osal_memcpy(&strTemp[11],",\"hum1\":", 8);
   osal_memcpy(&strTemp[19],humidity, 2);
   osal_memcpy(&strTemp[21],"}", 1);
  
  //获得的温湿度通过串口输出到电脑显示
  HalUARTWrite(0, "T&H:", 4);
  HalUARTWrite(0, strTemp, 22);
  HalUARTWrite(0, "\n",1);

  //输出到LCD显示
  for(i=0; i<3; i++)
  {   
    if(i==0)
    {
      LCD_P16x16Ch(i*16,4,i*16);
      LCD_P16x16Ch(i*16,6,(i+3)*16);
    }
    else
    {
      LCD_P16x16Ch(i*16,4,i*16);
      LCD_P16x16Ch(i*16,6,i*16);        
    }
  } 
  LCD_P8x16Str(44, 4, temp);
  LCD_P8x16Str(44, 6, humidity);
  
    if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       22,
                       strTemp,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}

/*********************************************************************
 * @fn      SampleApp_SendFlashMessage
 *
 * @brief   Send the flash message to group 1.
 *
 * @param   flashTime - in milliseconds
 *
 * @return  none
 */
void SampleApp_SendFlashMessage( uint16 flashTime )
{
  uint8 buffer[3];
  buffer[0] = (uint8)(SampleAppFlashCounter++);
  buffer[1] = LO_UINT16( flashTime );
  buffer[2] = HI_UINT16( flashTime );

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       3,
                       buffer,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}



/*********************************************************************
*********************************************************************/
