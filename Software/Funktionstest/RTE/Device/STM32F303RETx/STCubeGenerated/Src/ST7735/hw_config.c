/****************************************************************************************
*
* Hardware configuration and low level functions
*
* The idea of HW initialization and configuration have taken on from
* http://vg.ucoz.ru/load/stm32_ickhodnye_teksty_programm_na_si/stm32_biblioteka_podkljuchenija_displeja_na_kontrollere_st7735_dlja_stm32f4/16-1-0-41
****************************************************************************************/
#include <stdio.h>
/*
#include "stm32f30x.h"
#include "stm32f3_discovery.h"
#include "platform_config.h"
#include "stm32f30x_gpio.h"
#include "stm32f30x_rcc.h"
#include "stm32f30x_spi.h"
*/
#include "stm32f3xx_hal.h"

#include "hw_config.h"

static __IO uint32_t TimingDelay;

#ifndef __ENABLE_NOT_STABLE
// not work :( It is possible that GPIO pin doesn't switch to input mode, but I don't know hot to do it.
void receive_data(const uint8_t cmd, uint8_t *data, uint8_t cnt) {
    uint8_t i;
    LCD_DC0;
    //while(SPI_I2S_GetFlagStatus(SPI2, SPI_FLAG_TXE) == RESET);
		LCD_CS0;
    HAL_SPI_Transmit(&hspi2, data, 1, HAL_MAX_DELAY);//SPI_SendData(SPI2, cmd);
		LCD_CS1;
		//while(SPI2->SR & SPI_SR_BSY);
    //while(SPI_I2S_GetFlagStatus(SPI2, SPI_FLAG_TXE) == RESET);
    LCD_DC1;
    LCD_DC0;
    LCD_DC1;
    //SPI_BiDirectionalLineConfig(SPI2,SPI_Direction_Rx);
    //for(i=0; i<cnt; i++) {
        //		while(SPI2->SR & SPI_SR_BSY);
        /*if( SPI2->SR & SPI_I2S_FLAG_OVR )
            STM_EVAL_LEDOn(LED3);
        else
            STM_EVAL_LEDOff(LED3);
        if( SPI2->SR & I2S_FLAG_UDR )
            STM_EVAL_LEDOn(LED7);
        else
            STM_EVAL_LEDOff(LED7);
			*/
        //while (SPI_I2S_GetFlagStatus(SPI2, SPI_I2S_FLAG_RXNE) == RESET);
        //HAL_SPI_Receive(&hspi2, data, 1, HAL_MAX_DELAY);//SPI_ReceiveData8(SPI2);
    //}
		HAL_SPI_Receive(&hspi2, data, 1, HAL_MAX_DELAY);
    LCD_DC0;
    //SPI_BiDirectionalLineConfig(SPI2,SPI_Direction_Tx);
    LCD_CS1;
    LCD_CS0;
}

#endif

// Send byte via SPI to controller
void lcd7735_senddata(const uint8_t data) {
#ifdef LCD_TO_SPI2
    //while(HAL_SPI_GetState(&hspi2) == RESET);
		LCD_CS0;
    HAL_SPI_Transmit(&hspi2, &data, 1, HAL_MAX_DELAY);//SPI_SendData(SPI2, data);
		LCD_CS1;
#else
    unsigned char i;
    for(i=0; i<8; i++) {
        if (data & 0x80) LCD_MOSI1;
        else LCD_MOSI0;
        data = data<<1;
        LCD_SCK0;
        LCD_SCK1;
    }
#endif
}

// Send byte via SPI to controller
void lcd7735_senddata16(const uint16_t data) {
	uint8_t buf[2];
	buf[0] = data >> 8;
	buf[1] = data & 0xFF;
#ifdef LCD_TO_SPI2
    //while(SPI_I2S_GetFlagStatus(SPI2, SPI_FLAG_TXE) == RESET);
		LCD_CS0;
    HAL_SPI_Transmit(&hspi2, buf, 2, HAL_MAX_DELAY);//SPI_I2S_SendData(SPI2, data);
		LCD_CS1;
#else
    lcd7735_senddata(data >> 8);
    lcd7735_senddata(data &0xFF);
#endif
}

// Send control command to controller
void lcd7735_sendCmd(const uint8_t cmd) {
    LCD_DC0;
    lcd7735_senddata(cmd);
#ifdef LCD_TO_SPI2
    while(SPI2->SR & SPI_SR_BSY);
#endif
}

// Send parameters o command to controller
void lcd7735_sendData(const uint8_t data) {
    LCD_DC1;
    lcd7735_senddata(data);
#ifdef LCD_TO_SPI2
    while(SPI2->SR & SPI_SR_BSY);
#endif
}


/*
void delay_ms(uint32_t delay_value) {
    TimingDelay = delay_value;
    while( TimingDelay != 0 );
}
*/
void TimingDelay_Decrement(void) {
    if (TimingDelay != 0x00) TimingDelay--;
    //else STM_EVAL_LEDToggle(LED4);
}

