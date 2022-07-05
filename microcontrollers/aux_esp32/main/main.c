/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_spi_flash.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gptimer.h"
#include "driver/pulse_cnt.h"
#include "driver/mcpwm.h"

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define I2C_SLAVE_SCL_IO 5
#define I2C_SLAVE_SDA_IO 4
#define I2C_SLAVE_PORT  I2C_NUMBER(0)
#define ESP_SLAVE_ADDR 0x28

typedef struct motor_control_context{
    //mcpwm_unit_t
    int mcpwm_num; // set MCPWM unit(0-1)
    //mcpwm_timer_t
    int timer_num; // set MCPWM timer (0-2)
    mcpwm_io_signals_t genA; //MCPWMXA
    int genA_gpio_num; //gpio pin for genA
     mcpwm_io_signals_t genB; //MCPWMXB
    int genB_gpio_num; //gpio pin for genB
} motor_control_context;

static esp_err_t i2c_slave_init(void){
    int i2c_slave_port = I2C_SLAVE_PORT;
        i2c_config_t conf_slave = {
        .sda_io_num = I2C_SLAVE_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_SLAVE_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .mode = I2C_MODE_SLAVE,
        .slave.addr_10bit_en = 0,
        .slave.slave_addr = ESP_SLAVE_ADDR,
    };
    esp_err_t err = i2c_param_config(i2c_slave_port, &conf_slave);
    if (err != ESP_OK) {
        return err;
    }
    //byte per each motor, so 6*8 + 3 = 51 (assuming buffer size is in bits, parity + start + stop)
    return i2c_driver_install(i2c_slave_port, conf_slave.mode, 51, 4, 0);
}

void app_main(void)
{
    motor_control_context motors[6] = {
        {0, 0, MCPWM0A, 23, MCPWM0B, 22},
        {0, 1, MCPWM1A, 21, MCPWM1B, 19},
        {0, 2, MCPWM2A, 18, MCPWM2B, 5},
        {1, 0, MCPWM0A, 17, MCPWM0B, 16},
        {1, 1, MCPWM1A, 4, MCPWM1B, 0},
        {1, 2, MCPWM2A, 2, MCPWM2B, 15}
    };

    ESP_ERROR_CHECK(mcpwm_gpio_init(motors[0].mcpwm_num, motors[0].genA, BDC_MCPWM_GENA_GPIO_NUM));
    ESP_ERROR_CHECK(mcpwm_gpio_init(BDC_MCPWM_UNIT, MCPWM0B, BDC_MCPWM_GENB_GPIO_NUM));

    ESP_ERROR_CHECK(mcpwm_gpio_init(BDC_MCPWM_UNIT, MCPWM0A, BDC_MCPWM_GENA_GPIO_NUM));
    ESP_ERROR_CHECK(mcpwm_gpio_init(BDC_MCPWM_UNIT, MCPWM0B, BDC_MCPWM_GENB_GPIO_NUM));

    ESP_ERROR_CHECK(mcpwm_gpio_init(BDC_MCPWM_UNIT, MCPWM0A, BDC_MCPWM_GENA_GPIO_NUM));
    ESP_ERROR_CHECK(mcpwm_gpio_init(BDC_MCPWM_UNIT, MCPWM0B, BDC_MCPWM_GENB_GPIO_NUM));

    ESP_ERROR_CHECK(mcpwm_gpio_init(BDC_MCPWM_UNIT, MCPWM0A, BDC_MCPWM_GENA_GPIO_NUM));
    ESP_ERROR_CHECK(mcpwm_gpio_init(BDC_MCPWM_UNIT, MCPWM0B, BDC_MCPWM_GENB_GPIO_NUM));

    ESP_ERROR_CHECK(mcpwm_gpio_init(BDC_MCPWM_UNIT, MCPWM0A, BDC_MCPWM_GENA_GPIO_NUM));
    ESP_ERROR_CHECK(mcpwm_gpio_init(BDC_MCPWM_UNIT, MCPWM0B, BDC_MCPWM_GENB_GPIO_NUM));

    ESP_ERROR_CHECK(mcpwm_gpio_init(BDC_MCPWM_UNIT, MCPWM0A, BDC_MCPWM_GENA_GPIO_NUM));
    ESP_ERROR_CHECK(mcpwm_gpio_init(BDC_MCPWM_UNIT, MCPWM0B, BDC_MCPWM_GENB_GPIO_NUM));

    uint8_t * data;
    size_t dataSize = 6;
    ESP_ERROR_CHECK(i2c_slave_init());

    while(true){
        for(int i = 0; i < dataSize; i++)
            data[i] = NULL;
        i2c_slave_read_buffer(I2C_SLAVE_PORT, data, dataSize, 1000 / portTICK_PERIOD_MS);
        printf("Hello world!\n");
    }

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    printf("silicon revision %d, ", chip_info.revision);

    printf("%uMB %s flash\n", spi_flash_get_chip_size() / (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

    for (int i = 10; i >= 0; i--) {
        printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}