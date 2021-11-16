/*
 * lcd.h
 *
 *  Created on: Nov 14, 2021
 *      Author: hersh
 */

#ifndef INC_LCD_H_
#define INC_LCD_H_

#include "stm32f4xx_hal.h"

//initialize lcd
void lcd_init(I2C_HandleTypeDef* i2c_handle);

//send a byte (used for configuring)
void lcd_transmit_byte(uint8_t data);

//set where the cursor is
void lcd_set_cursor(uint8_t col, uint8_t row);

//clear the display
void lcd_clear();

//transmit a single char at the cursor pos
void lcd_transmit_char(char c);

//transmit a string at the cursor pos
void lcd_transmit_string(char* str);


#endif /* INC_LCD_H_ */
