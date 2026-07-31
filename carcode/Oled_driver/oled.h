#ifndef __OLED_H
#define __OLED_H

#include <stdint.h>

#define OLED_WIDTH     128U
#define OLED_HEIGHT     64U
#define OLED_PAGE_COUNT  8U

void OLED_Init(void);
void OLED_Clear(void);
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr);
void OLED_ShowString(uint8_t x, uint8_t y, const char *chr);
void OLED_SetPos(uint8_t x, uint8_t y);

#endif
