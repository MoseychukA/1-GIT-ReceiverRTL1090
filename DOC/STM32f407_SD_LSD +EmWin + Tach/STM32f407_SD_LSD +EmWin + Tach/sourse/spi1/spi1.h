#ifndef __SPI_1_H
#define __SPI_1_H

#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include <stm32f10x_spi.h>

void spi1_init(void);
void SPI1_SetDMA_RxTx(uint32_t TxDummy, uint16_t NumByte   ,uint16_t MODE_TX   );

#endif /* __SPI_1_H */
