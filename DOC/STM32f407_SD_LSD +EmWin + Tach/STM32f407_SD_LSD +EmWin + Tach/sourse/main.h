#ifndef MAIN_H
#define MAIN_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx.h"

#include "tft_lcd.h"
#include "TouchPanel.h"
#include "GUI.h"
#include "WindowDLG.h"
#include "GUIDEMO.h"

void delay_ms(uint32_t ms);
void NVIC_Configuration(void);

#endif
