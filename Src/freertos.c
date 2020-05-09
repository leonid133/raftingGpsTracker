/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "usart.h"
#include "i2c.h"
#include "spi.h"
#include "dma.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//uint8_t b1;
//uint8_t b2;
//
//typedef struct {
//	uint8_t usart;
//	uint8_t byte;
//} myMes;

/* $GPRMC
   * note: a siRF chipset will not support magnetic headers.
   * $GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,ddmmyy,x.x,a*hh
   * ex: $GPRMC,230558.501,A,4543.8901,N,02112.7219,E,1.50,181.47,230213,,,A*66,
   *
   * WORDS:
   *  0    = $GPRMC - Recommended Minimum Specific GNSS Data
   *  1    = UTC of position fix
   *  2    = Data status (V=navigation receiver warning)
   *  3    = Latitude of fix
   *  4    = N or S
   *  5    = Longitude of fix
   *  6    = E or W
   *  7    = Speed over ground in knots
   *  8    = Track made good in degrees True, Bearing This indicates the direction that the device is currently moving in,
   *       from 0 to 360, measured in “azimuth”.
   *  9    = UT date
   *  10   = Magnetic variation degrees (Easterly var. subtracts from true course)
   *  11   = E or W
   *  12   = Checksum
   */

//typedef struct {
//	char Time[12];
//	char Status[2];
//	char SLatitude[16];
//	char NS[3];
//	char SLongitude[12];
//	char EW[3];
//	char SpeedKnot[8];
//	char Course[10];
//	char Data[12];
//	char MagneticCourse[12];
//	char EWCourse[3];
//	char Checksum[4];
//} myGPRMC;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define LCD_ADDR (0x27 << 1)

#define PIN_RS    (1 << 0)
#define PIN_EN    (1 << 2)
#define BACKLIGHT (1 << 3)

//#define LCD_BACKLIGHT 0x08
//#define LCD_NOBACKLIGHT 0x00

#define LCD_DELAY_MS 5

#define ARRAY_LEN(x)   (sizeof(x) / sizeof((x)[0]))

#define UART_DMA_BUFFER_SIZE 1024

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */


char Time[12]="";
char Status[2]="";
char SLatitude[16]="";
char NS[3]="";
char SLongitude[16]="";
char EW[3]="";
char SpeedKnot[8]="";
char Course[10]="";
char Data[12]="";
char MagneticCourse[12]="";
char EWCourse[3]="";
char Checksum[4]="";
int ComputedChecksum=0;

char *const GPRMC[]={Time,Status,SLatitude,NS,SLongitude,EW,SpeedKnot,Course,Data,MagneticCourse,EWCourse,Checksum,Checksum};
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId Usart1RxHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void Parser(unsigned char data)
{
	static uint8_t flagChecksum = 0;
	static unsigned char ByteCount=0xff;
	static unsigned int MsgType;
	static char *MsgTxt=(char*)&MsgType;

	static unsigned char ComaPoint=0xff;
	static unsigned char CharPoint=0;
	if(data=='$'){
		ByteCount=0;
		ComaPoint=0xff;
		MsgTxt=(char*)&MsgType;
		flagChecksum=ComputedChecksum=0;
		return;
	}
	if(ByteCount==0xff) return;
	ByteCount++;
	if(ByteCount<=1) {ComputedChecksum^=data;  return; }
	if(ByteCount<6&&ByteCount>1)
    {
        *MsgTxt=data;   //и делаем из него число
        MsgTxt++;
        ComputedChecksum^=data;
        return;
    }

	switch(MsgType)
        {
        case    0x434D5250:      		                   //GPRMC
        		if(data==',') {ComaPoint++; CharPoint=0; GPRMC[ComaPoint][0]=0; ComputedChecksum^=data; return;}
        		if(data=='*') {flagChecksum++; CharPoint=0;GPRMC[ComaPoint][CharPoint]=0;return;}
        		if(ComaPoint > ARRAY_LEN(GPRMC) || CharPoint > 16 || data=='\r' || data=='\n')
        		{
        			MsgType=0;return;
        		}

        		GPRMC[ComaPoint][CharPoint++]=data;
        		GPRMC[ComaPoint][CharPoint]=0;
        		if(flagChecksum==0) {ComputedChecksum^=data;}
        		else{snprintf(GPRMC[12], sizeof(GPRMC[12]), "%02X", ComputedChecksum);}
        		return;
//        case    0x434D524E:                             //GNRMC
//                if(data==',') {ComaPoint++;     CharPoint=0;RMC[ComaPoint][0]=0;return;}
//                if(data=='*') {MsgType=0;return;}
//                RMC[ComaPoint][CharPoint++]=data;
//                RMC[ComaPoint][CharPoint]=0;
//                return;
//        case    0x41474750:                             //PGGA
//        case    0x4147474e:                             //NGGA
//                if(data==',')  {ComaPoint++;    CharPoint=0;GGA[ComaPoint][0]=0;return;}
//                if(data=='*') {MsgType=0;return;}
//                GGA[ComaPoint][CharPoint++]=data;
//                GGA[ComaPoint][CharPoint]=0;
//                return;
//        case    0x47545650:             //PVTG
//                if(data==',')  {ComaPoint++;    CharPoint=0;VTG[ComaPoint][0]=0;return;}
//                if(data=='*') {return;}
//                VTG[ComaPoint][CharPoint++]=data;
//                VTG[ComaPoint][CharPoint]=0;
//                return;
//        case    0x4754564e:             //NVTG
//                if(data==',')  {ComaPoint++;    CharPoint=0;VTG[ComaPoint][0]=0;return;}
//                if(data=='*') {return;}
//                VTG[ComaPoint][CharPoint++]=data;
//                VTG[ComaPoint][CharPoint]=0;
//                return;
//        case    0x56534750:             //PGSV
//                if(data==',')  {ComaPoint++;    CharPoint=0;GSV[ComaPoint][0]=0;return;}
//                if(data=='*')  {GPS_COUNT=AsciiToInt(ViewSat);MsgType=0;return;}
//                GSV[ComaPoint][CharPoint++]=data;
//                GSV[ComaPoint][CharPoint]=0;
//                return;
//        case    0x5653474c:             //LGSV
//                if(data==',')  {ComaPoint++;    CharPoint=0;GSV[ComaPoint][0]=0;return;}
//                if(data=='*') {GLONAS_COUNT=AsciiToInt(ViewSat);MsgType=0;return;}
//                GSV[ComaPoint][CharPoint++]=data;
//                GSV[ComaPoint][CharPoint]=0;
//                return;
        default:        ByteCount=0xff;break;
        }
	ByteCount=0xff;
}

//void ProcessUartQueue(void)
//{
//	myMes w;
//	uint8_t scr[82]; // screen buffer
//	uint8_t buf[18];
//	uint8_t t1[18]; // temp buffer
//	uint8_t t2[18*3];
//	uint8_t oldusart=0;
//	uint8_t char_count=0;
//	uint8_t newline=0;
//	char str[512]={0};
//
//	//	  if(HAL_UART_Receive_DMA(&huart1, &b1,1)==HAL_OK)
//	if(HAL_UART_Receive_DMA(&huart1, (uint8_t*)str,512)==HAL_OK)
//	{
//		HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
//	}
//
//	xQueueReceive( RecQHandle, &w, portMAX_DELAY );
//
//	if(oldusart!=w.usart || char_count>16)
//		{
//			oldusart=w.usart;
//			newline=1;
//			char_count=0;
//		}
//	buf[char_count++]=w.byte;
//
//	//	if(hUsbDeviceFS.dev_state==USBD_STATE_CONFIGURED)
//	//			{
//	//				HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_13);
//					t1[0]=0;
//					t2[0]=0;
//					scr[0]=0;
//
//					if(newline==1)
//					{
//						newline=0;
//						sprintf((char *) &scr[0],"\n\rU%d ",w.usart);
//					}
//					else
//					{
//						sprintf((char *) &scr[0],"\rU%d ",w.usart);
//					}
//
//					// now print 16 bytes
//
//					for(int i=0;i<char_count;i++)
//						{
//							if(buf[i]>0x1f && buf[i]<0x7f)
//							{
//								sprintf((char *) &t2[0],"%c",buf[i]);
//							}
//							else
//							{
//								sprintf((char *) &t2[0],".");
//							}
//							strcat((char *)&t1[0],(char *)&t2[0]);
//						}
//					//padding
//					for(int i=char_count-1;i<16;i++)
//						{
//							sprintf((char *) &t2[0]," ");
//							strcat((char *)&t1[0],(char *)&t2[0]);
//						}
//					strcat((char *) &scr[0],(char *)&t1[0]);
//
//					// and hex
//					t2[0]=0;
//					for(int i=0;i<char_count;i++)
//						{
//							sprintf((char *) &t1[0]," %02X",buf[i]);
//							strcat((char *)&t2[0],(char *)&t1[0]);
//						}
//					strcat((char *) &scr[0],(char *)&t2[0]);
//		// Do you want some debug output?
//					sprintf((char *) &t1[0]," %d",char_count);
//					strcat((char *) &scr[0],(char *)&t1[0]);
//	//				CDC_Transmit_FS(&scr[0],strlen((char *)&scr[0]));
//
//	//			}
//
//}

//void SendTestMessage(void)
//{
//  	// StarWars
//
//	uint8_t c3po[]="Sir, the possibility of successfully navigating an asteroid field is approximately 3,720 to 1 \r\n";
//	HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&c3po,sizeof(c3po)-1);
//
//}

//void LCD_GetAddr(void)
//{
//	HAL_StatusTypeDef res;
//	for(uint16_t i = 0; i < 128; i++) {
//		res = HAL_I2C_IsDeviceReady(&hi2c1, i << 1, 1, 10);
//		if(res == HAL_OK) {
//			char msg[64];
//			snprintf(msg, sizeof(msg), "0x%02X", i);
//		}
//	}
//}

HAL_StatusTypeDef LCD_SendInternal(uint8_t lcd_addr, uint8_t data,
                                   uint8_t flags) {
    HAL_StatusTypeDef res;
    for(int i=0;i<512;i++) {
//    for(;;) {
        res = HAL_I2C_IsDeviceReady(&hi2c1, lcd_addr, 1,
                                    HAL_MAX_DELAY);
        if(res == HAL_OK)
            break;
    }

    uint8_t up = data & 0xF0;
    uint8_t lo = (data << 4) & 0xF0;

    uint8_t data_arr[4];
    data_arr[0] = up|flags|BACKLIGHT|PIN_EN;
    data_arr[1] = up|flags|BACKLIGHT;
    data_arr[2] = lo|flags|BACKLIGHT|PIN_EN;
    data_arr[3] = lo|flags|BACKLIGHT;
    //I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size)
    res = HAL_I2C_Master_Transmit(&hi2c1, lcd_addr, data_arr,
                                  sizeof(data_arr), HAL_MAX_DELAY);
//    res = HAL_I2C_Master_Transmit_DMA(&hi2c1, lcd_addr, data_arr,
//                                     sizeof(data_arr));
    HAL_Delay(LCD_DELAY_MS);
    return res;
}

void LCD_SendCommand(uint8_t lcd_addr, uint8_t cmd) {
    LCD_SendInternal(lcd_addr, cmd, 0);
}

void LCD_SendData(uint8_t lcd_addr, uint8_t data) {
    LCD_SendInternal(lcd_addr, data, PIN_RS);
}

void LCD_Init(uint8_t lcd_addr) {
    // 4-bit mode, 2 lines, 5x7 format
    LCD_SendCommand(lcd_addr, 0b00110000);
    // display & cursor home (keep this!)
    LCD_SendCommand(lcd_addr, 0b00000010);
    // display on, right shift, underline off, blink off
    LCD_SendCommand(lcd_addr, 0b00001100);
    // clear display (optional here)
    LCD_SendCommand(lcd_addr, 0b00000001);
}

void LCD_SendString(uint8_t lcd_addr, char *str) {
    while(*str) {
        LCD_SendData(lcd_addr, (uint8_t)(*str));
        str++;
    }
}

void WriteToLCD(char *str, char *str2) {
	LCD_SendCommand(LCD_ADDR, 0b00000001);

    // set address to 0x00
	LCD_SendCommand(LCD_ADDR, 0b10000000);
	LCD_SendString(LCD_ADDR, str);

	// set address to 0x40
	LCD_SendCommand(LCD_ADDR, 0b11000000);
	LCD_SendString(LCD_ADDR, str2);
}

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
//{
//	myMes m;
//
//	switch((uint32_t)UartHandle->Instance)
//	{
//		case (uint32_t)USART1:
//
//		  m.usart=1;
//		  m.byte=b1;
//		  xQueueSendFromISR( RecQHandle, &m, portMAX_DELAY  );
//		  // Do this need?
////		  HAL_UART_Receive_DMA(&huart1, &b1,1);
//		  break;
//		case (uint32_t)USART2:
//
//		  m.usart=2;
//		  m.byte=b2;
//		  xQueueSendFromISR( RecQHandle, &m, portMAX_DELAY  );
////		  HAL_UART_Receive_DMA(&huart2, &b2,1);
//			break;
//	}
//}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartUsart1Rx(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];
  
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
  *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
  *ppxIdleTaskStackBuffer = &xIdleStack[0];
  *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
  /* place for user code */
}                   
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Usart1Rx */
  osThreadDef(Usart1Rx, StartUsart1Rx, osPriorityLow, 0, 128);
  Usart1RxHandle = osThreadCreate(osThread(Usart1Rx), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  LCD_Init(LCD_ADDR);
  char *const msg_arr[]={"Time","Status","SLatitude","NS","SLongitude","EW","SpeedKnot","Course","Data","MagneticCourse","EWCourse","Checksum","Calculated Checksum"};
  uint8_t cntr =0;
  /* Infinite loop */
  for(;;)
  {

  //char *const GPRMC[]={Time,Status,SLatitude,NS,SLongitude,EW,SpeedKnot,Course,Data,MagneticCourse,EWCourse,Checksum};

//	WriteToLCD(GPRMC[12], GPRMC[11]);
	  if(GPRMC[12] == GPRMC[11]) {
		  WriteToLCD(msg_arr[cntr], GPRMC[cntr]);
		  cntr = (cntr>=ARRAY_LEN(msg_arr)-3) ? 0 : cntr + 1;
		  HAL_GPIO_TogglePin(Led_GPIO_Port, Led_Pin);
	  }
	osDelay(5000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartUsart1Rx */
/**
* @brief Function implementing the Usart1Rx thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUsart1Rx */
void StartUsart1Rx(void const * argument)
{
  /* USER CODE BEGIN StartUsart1Rx */
  static uint8_t buffer[UART_DMA_BUFFER_SIZE];
  HAL_UART_Receive_DMA(&huart1, (uint8_t*)buffer, UART_DMA_BUFFER_SIZE);
  size_t dma_head = 0, dma_tail = 0;
  /* Infinite loop */
  for(;;)
  {
	  do
	  {
	      __disable_irq();
	      dma_tail = UART_DMA_BUFFER_SIZE - huart1.hdmarx->Instance->CNDTR;
	      __enable_irq();

	     if(dma_tail!=dma_head)
	     {
	       	if(dma_head < dma_tail)
	       	{
	   		    for(register size_t i=dma_head; i<dma_tail; i++)
	      	    {
	   		    	Parser(buffer[i]);
				}
			}
			else
			{
				for(register size_t i=dma_head; i<UART_DMA_BUFFER_SIZE; i++)
	  		    {
					Parser(buffer[i]);
	  		    }
				for(register size_t i=0; i<dma_tail; i++)
				{
					Parser(buffer[i]);

				}
			}
			dma_head=dma_tail;
		  }
	  }while(dma_head!=(UART_DMA_BUFFER_SIZE- huart1.hdmarx->Instance->CNDTR));

	osDelay(500);
  }

  /* USER CODE END StartUsart1Rx */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
