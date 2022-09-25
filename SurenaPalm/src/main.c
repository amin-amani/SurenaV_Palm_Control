#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "sdkconfig.h"
#include "driver/uart.h"
#include "BMP280.h"


#define BLINK_GPIO 2
#define BMP_SCK 14
#define BMP_MISO 12
#define BMP_MOSI 13
#define BMP_CS 15


spi_device_handle_t spi;
//---------------------------------------------------------------------------------------------

//====================================================================================================================

void SPISetChipSelect(bool value )
{
 gpio_set_level(BMP_CS, value);
}
//====================================================================================================================
uint8_t SPISend(uint8_t registerAddress)
{
    spi_transaction_t t;
    esp_err_t ret;
    uint8_t rxBuff;
     memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length=8;                     //Command is 8 bits
    t.tx_buffer=&registerAddress;               //The data is the cmd itself
    t.rx_buffer=&rxBuff;
    t.user=(void*)0;                //D/C needs to be set to 0
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    return rxBuff;
}


//====================================================================================================================
void GPIOInit()
{
    gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(BMP_CS);
    gpio_set_direction(BMP_CS, GPIO_MODE_OUTPUT);
}
//====================================================================================================================
void SPIInit()
{
 esp_err_t ret;
    spi_bus_config_t buscfg={
        .miso_io_num=BMP_MISO,
        .mosi_io_num=BMP_MOSI,
        .sclk_io_num=BMP_SCK,
        .quadwp_io_num=-1,
        .quadhd_io_num=-1,
        .max_transfer_sz=8
    };
    spi_device_interface_config_t devcfg={

        .clock_speed_hz=4*1000*1000,           //Clock out at 4 MHz
        .mode=0,                                //SPI mode 0
        // .spics_io_num=BMP_CS,               //CS pin
        .queue_size=7,                          //We want to be able to queue 7 transactions at a time
        // .pre_cb=lcd_spi_pre_transfer_callback,  //Specify pre-transfer callback to handle D/C line
    };

   
    ret=spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret=spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
   
}
//====================================================================================================================
void app_main() 
{
    GPIOInit();
    SPIInit();
    BMP280_config_t bmpconf;
    bmpconf.SPIChpSelectFunction=SPISetChipSelect;
    bmpconf.spiSendFunction=SPISend;

    BMP280Init(bmpconf);
while (1)
{
    printf("Register ID=%x\n",BMP280ReadRegister(BMP280_REGISTER_CHIPID));
    gpio_set_level(BLINK_GPIO, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(BLINK_GPIO, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
}

}