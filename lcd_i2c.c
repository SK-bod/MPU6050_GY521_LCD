/*
 * lcd_i2c.c
 *
 *  Created on: 11 maj 2021
 *      Author: Stasiek
 *
 */
//4 BIT MODE
#include "lcd_i2c.h"
extern I2C_HandleTypeDef hi2c1; //handler

#define SLAVE_LCD_ADDRESS 0x4E //device Slave Address

void lcd_send_cmd (char cmd) //send command
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C; //en=1, rs=0
	data_t[1] = data_u|0x08; //en=0, rs=0
	data_t[2] = data_l|0x0C; //en=1, rs=0
	data_t[3] = data_l|0x08; //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_LCD_ADDRESS, (uint8_t*)data_t, 4 ,100);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D; //en=1, rs=0
	data_t[1] = data_u|0x09; //en=0, rs=0
	data_t[2] = data_l|0x0D; //en=1, rs=0
	data_t[3] = data_l|0x09; //en=0, rs=0
	HAL_I2C_Master_Transmit(&hi2c1, SLAVE_LCD_ADDRESS,(uint8_t *) data_t,4,100);
}

void lcd_clear (void)
{
	lcd_send_cmd (0x80);
	for(int i=0;i<80;i++)
	{
		lcd_send_data(' ');
	}
}

void lcd_put_cur(int row, int col) //put cursor
{
	switch(row)
	{
		case 0:
			col |= 0x80;
			break;
		case 1:
			col |=0xC0;
			break;
	}

	lcd_send_cmd (col);
}

void lcd_init (void)
{
	//4 bit initialization
	HAL_Delay(50);
	lcd_send_cmd(0x30);
	HAL_Delay(5);
	lcd_send_cmd(0x30);
	HAL_Delay(1);
	lcd_send_cmd(0x30);
	HAL_Delay(10);
	lcd_send_cmd(0x20);
	HAL_Delay(10);

	//display initialization
	lcd_send_cmd(0x28); // Function set DL=0 (4bit mode), N=1 (2 line display)
	HAL_Delay(1);
	lcd_send_cmd (0x08);//Display on/off control -->D=0, C=0, B=0 ->display off
	HAL_Delay(1);
	lcd_send_cmd (0x01);//clear display
	HAL_Delay(1);
	HAL_Delay(1);
	lcd_send_cmd(0x06);//Entry mode set --> I/D = 1 (increment cursor) & S=0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd(0x0C);//Display on/off control --> D=1, C and B=0. (Cursor and blibk)
}

void lcd_send_string(char *str)
{
	while(*str) lcd_send_data(*str++);
}
