/*
 * lcd.c
 *
 *  Created on: Nov 14, 2021
 *      Author: hersh
 */

#include "lcd.h"

#define LCD_I2C_ADDRESS 0x4E

static I2C_HandleTypeDef* hi2c;

void lcd_init(I2C_HandleTypeDef* i2c_handle) {
	hi2c = i2c_handle;

	//Initialize LCD with nibble method
	lcd_transmit_byte(0x02);
	HAL_Delay(20);

	//Function set
	//DL=0, 4 bit mode
	//N=1, 2 line display
	//F=0 5x8 dots
	lcd_transmit_byte(0x28);
	HAL_Delay(5);

	//clear display
	lcd_transmit_byte(0x01);
	HAL_Delay(5);

	//Entry mode set
	//I/D=1, increment cursor
	//S=0, no shifting
	lcd_transmit_byte(0x06);
	HAL_Delay(5);

	//Display control
	//D=1, display on
	//C=0, cursor off
	//B=0, cursor blink off
	lcd_transmit_byte(0x0C);
}

void lcd_transmit_byte(uint8_t data) {
	uint8_t buf[4];
	uint8_t data_upper = data & 0xF0;
	uint8_t data_lower = (data << 4) & 0xF0;

	//upper bits - EN=1, RS=0 => 0x0C
	//lower bits - EN=0, RS=0 => 0x08

	buf[0] = data_upper | 0x0C;
	buf[1] = data_upper | 0x08;
	buf[2] = data_lower | 0x0C;
	buf[3] = data_lower | 0x08;

	HAL_I2C_Master_Transmit(hi2c, LCD_I2C_ADDRESS, (uint8_t *) buf, 4, 100);
}

void lcd_set_cursor(uint8_t col, uint8_t row) {

	uint8_t ddram_addr = 0x80;

	//row 0 offset = 0
	//row 1 offset = 0x40
	uint8_t pos = col;
	if (row > 0) {
		pos += 0x40;
	}

	lcd_transmit_byte(ddram_addr | pos);
}

void lcd_clear() {
	lcd_transmit_byte(0x01);
}

void lcd_transmit_char(char c) {
	uint8_t buf[4];
	uint8_t data_upper = c & 0xF0;
	uint8_t data_lower = (c << 4) & 0xF0;

	//upper bits - EN=1, RS=1 => 0x0D
	//lower bits - EN=0, RS=1 => 0x09

	buf[0] = data_upper | 0x0D;
	buf[1] = data_upper | 0x09;
	buf[2] = data_lower | 0x0D;
	buf[3] = data_lower | 0x09;

	HAL_I2C_Master_Transmit(hi2c, LCD_I2C_ADDRESS, (uint8_t *) buf, 4, 100);
}

void lcd_transmit_string(char* str) {
	while (*str) {
		lcd_transmit_char(*str++);
	}
}
