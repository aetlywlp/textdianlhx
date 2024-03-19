#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#define   LED_PIN GPIO_NUM_2

void app_main(void)
{
    esp_rom_gpio_pad_select_gpio(LED_PIN);
    gpio_set_direction(LED_PIN,GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN,0);
    while (1)
    {
        gpio_set_level(LED_PIN,0);
        vTaskDelay(1000/portTICK_PERIOD_MS);
        gpio_set_level(LED_PIN,1);
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
    

}