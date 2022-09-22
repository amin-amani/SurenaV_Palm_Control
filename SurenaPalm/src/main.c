#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "sdkconfig.h"
#include "driver/uart.h"

#define BLINK_GPIO 2
#define GPIO_INPUT_IO_0     12
#define GPIO_INPUT_IO_1     14
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))
//====================================================================================================================
void GPIOInit()
{
        gpio_config_t io_conf = {};
   //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
      
    gpio_pad_select_gpio(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

}
void app_main() 
{
GPIOInit();
while (1)
{
    /* code */  
    gpio_set_level(BLINK_GPIO, 0);
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(BLINK_GPIO, 1);
    vTaskDelay(500 / portTICK_PERIOD_MS);
}



}