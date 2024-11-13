#include "spi1.h"
//#include "DMA_STM32F10x.h"

//void DMA1_Channel3_IRQHandler(void)
//{
//   if( DMA_GetITStatus(DMA1_IT_TC3) == SET) {
//      DMA_ClearITPendingBit(DMA1_IT_TC3);
//      //NSS_OFF();
//		  //TFT_CS_HIGH;
//      }
//}
//void SPI1_IRQHandler(void) 
//	{
//		
//	if (SPI_I2S_GetITStatus(SPI1, SPI_I2S_IT_TXE) == SET) // данные отправлены в передатчик, регистр данных опустел - 7
//	{
//	 SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_TXE);	
//	 //TFT_CS_SET;	
//	 SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, DISABLE);	
////	 spi8();  // для 16-бит SPI, возврат на 8-бит
////   DMA_Cmd(DMA1_Channel3, DISABLE);                     // âûêëþ÷èëè äìà óàðòà3
////	 DMA_SetCurrDataCounter(DMA1_Channel3, 65535);
////	 DMA_Cmd(DMA1_Channel3, ENABLE);                      // âêëþ÷èëè äìà óàðò3				
//	}
//}
	
void spi1_init(void) {
   	SPI_InitTypeDef spi1;
    GPIO_InitTypeDef gpio;
	
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);  // тактирование порта
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);  // тактирование SPI1 
 

    GPIO_StructInit(&gpio);	 
    gpio.GPIO_Pin =  GPIO_Pin_5 | GPIO_Pin_7;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOA,&gpio);
	
    gpio.GPIO_Pin =  GPIO_Pin_6 ;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(GPIOA,&gpio);	
	
    SPI_I2S_DeInit(SPI1);
    SPI_StructInit(&spi1);
 
    spi1.SPI_Mode = SPI_Mode_Master;
    spi1.SPI_DataSize = SPI_DataSize_8b;
    spi1.SPI_NSS = SPI_NSS_Soft;//SPI_NSS_Hard;//
    spi1.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_4; //SPI_BaudRatePrescaler_4; 14MHz for 56MHz CPU
    spi1.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi1.SPI_CPOL = SPI_CPOL_Low;
    spi1.SPI_CPHA = SPI_CPHA_1Edge;
		
// Настройка прерываний
//	 SPI_I2S_ClearITPendingBit(SPI1, SPI_I2S_IT_TXE);		
//   SPI_I2S_ITConfig(SPI1, SPI_I2S_IT_TXE, ENABLE);	
//	 NVIC_EnableIRQ(SPI1_IRQn);	

    SPI_Init(SPI1,&spi1);
    SPI_Cmd(SPI1,ENABLE);
}


//void SPI1_SetDMA_RxTx(uint32_t TxDummy, uint16_t NumByte   ,uint16_t MODE_TX   )
//{  	
//   DMA_InitTypeDef    DMA_InitStructure;
//   //NVIC_InitTypeDef   NVIC_InitStructure;

//   DMA_Cmd(DMA1_Channel3, DISABLE); 
//   RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
//   DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)(&SPI1->DR) ;
//   DMA_InitStructure.DMA_BufferSize = NumByte;
//   DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//   DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
//   DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
//   DMA_InitStructure.DMA_Mode =DMA_Mode_Normal;
//   DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;
//   DMA_InitStructure.DMA_M2M = DMA_M2M_Disable; 
//  
//   if (MODE_TX == 1)
//   DMA_InitStructure.DMA_MemoryInc            = DMA_MemoryInc_Disable;
//	 else DMA_InitStructure.DMA_MemoryInc            = DMA_MemoryInc_Enable;
//		 
////      DMA_InitStructure.DMA_MemoryBaseAddr       = (uint32_t)&RxDummy;//(uint32_t)&RxDummy;   //при записи пишем в переменную-пустышку без инкремента
////      DMA_InitStructure.DMA_MemoryInc            = DMA_MemoryInc_Disable;
////      DMA_InitStructure.DMA_DIR                  = DMA_DIR_PeripheralDST;
////      DMA_Init(DMA1_Channel2, &DMA_InitStructure);

//   DMA_InitStructure.DMA_MemoryBaseAddr       = TxDummy; //(uint32_t)&MemAddr;
//   DMA_InitStructure.DMA_DIR                  = DMA_DIR_PeripheralDST;
//   DMA_Init(DMA1_Channel3, &DMA_InitStructure);


//	 //SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
//	 SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
//	 
//   //настройка прерываний DMA
//   //Прерывания только по Rx, когда байт уже ушел	 
//	 //DMA_ITConfig(DMA1_Channel3, DMA_IT_TC, ENABLE); 	 
//   //NVIC_EnableIRQ(DMA1_Channel3_IRQn);
//	 
//   DMA_Cmd(DMA1_Channel3, ENABLE); 
//	 
////   NVIC_InitStructure.NVIC_IRQChannel = SPI_FRAM_Rx_DMA_IRQ;
////   NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = SPI_FRAM_Rx_DMA_IRQ_PRIORITY;
////   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
////   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
////   NVIC_Init(&NVIC_InitStructure);
//}
