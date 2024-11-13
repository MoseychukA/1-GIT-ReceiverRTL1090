#include "main.h"

#include "LCD_ini.h"

extern ILI9341_Options_t ILI9341_Opts;
extern volatile GUI_TIMER_TIME OS_TimeMS;

static __IO uint32_t TimingDelay;

//============================================================================
void SysTick_Handler (void)
{
	TimingDelay--;	
	OS_TimeMS++;				
}

void delay_ms(uint32_t ms)
{
TimingDelay = (ms);

while(TimingDelay);
}
//============================================================================

int main(void)
{  
  RCC_ClocksTypeDef RCC_ClockFreq;
 
	uint16_t x, y;
	GUI_PID_STATE TS_Pid_State;
	
	SystemInit();
  SysTick_Config(SystemCoreClock/1000);	
  RCC_GetClocksFreq(&RCC_ClockFreq);
  NVIC_Configuration();
	
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_CRC, ENABLE);
	
	GUI_Init();
	//GUI_DispString("Hello world!");	

	TouchInit();
	TouchCalibrate(); 
	
	//WM_HWIN hDialog=CreateWindow();
	GUIDEMO_Main();	 
	
	while(1){
		
		TS_Pid_State.Pressed = TouchReadXY( &x , &y, true);				
		if (TS_Pid_State.Pressed){
			TS_Pid_State.x = x;
			TS_Pid_State.y = y;		  
		}
		
		GUI_TOUCH_StoreStateEx(&TS_Pid_State);				
    GUI_CURSOR_Show ( );
    GUI_Exec ( );	   
		
	}	
  return 1;
}


void NVIC_Configuration(void)
{
  NVIC_InitTypeDef NVIC_InitStructure;

  /* Configure the NVIC Preemption Priority Bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
  NVIC_Init(&NVIC_InitStructure);
}
//******************************************************************************

