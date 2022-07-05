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

int8_t sign_byte(uint8_t x)
{
  return ((x >= (1 << 7))
          ? -(UINT8_MAX - x + 1)
          : x);
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

    mcpwm_config_t pwm_config = {
        .frequency = 1500,
        .cmpr_a = 0,
        .cmpr_b = 0,
        .counter_mode = MCPWM_UP_COUNTER,
        .duty_mode = MCPWM_DUTY_MODE_0
    };

    for(int i = 0; i < 6; i++){
        ESP_ERROR_CHECK(mcpwm_gpio_init(motors[i].mcpwm_num, motors[i].genA, motors[i].genA_gpio_num));
        ESP_ERROR_CHECK(mcpwm_gpio_init(motors[i].mcpwm_num, motors[i].genB, motors[i].genB_gpio_num));
        ESP_ERROR_CHECK(mcpwm_init(motors[i].mcpwm_num, motors[i].timer_num, &pwm_config));
    }


    

    uint8_t * data = NULL;
    int8_t * decoded_data = NULL;
    size_t dataSize = 6;
    ESP_ERROR_CHECK(i2c_slave_init());

    while(true){
        for(int i = 0; i < dataSize; i++){
            data[i] = NULL;
            decoded_data[i] = NULL;
        }
        i2c_slave_read_buffer(I2C_SLAVE_PORT, data, dataSize, 1000 / portTICK_PERIOD_MS);

        for(int i = 0; i < dataSize; i++){
            if(data[i] != NULL)
                decoded_data[i] = sign_byte(data[i]);
            if(data[i] != NULL && decoded_data[i] > 0){
                float duty_cycle = decoded_data[i] / 127.0;
                mcpwm_set_signal_low(motors[i].mcpwm_num, motors[i].timer_num, motors[i].genB);
                mcpwm_set_duty(motors[i].mcpwm_num, motors[i].timer_num, motors[i].genA, duty_cycle);
                mcpwm_set_duty_type(motors[i].mcpwm_num, motors[i].timer_num, motors[i].genA, MCPWM_DUTY_MODE_0);
            } else if (data[i] != NULL && decoded_data[i] < 0){
                float duty_cycle = -decoded_data[i] / 128.0;
                mcpwm_set_signal_low(motors[i].mcpwm_num, motors[i].timer_num, motors[i].genA);
                mcpwm_set_duty(motors[i].mcpwm_num, motors[i].timer_num, motors[i].genB, duty_cycle);
                mcpwm_set_duty_type(motors[i].mcpwm_num, motors[i].timer_num, motors[i].genB, MCPWM_DUTY_MODE_0);
            } else {
                mcpwm_set_signal_low(motors[i].mcpwm_num, motors[i].timer_num, motors[i].genA);
                mcpwm_set_signal_low(motors[i].mcpwm_num, motors[i].timer_num, motors[i].genB);
            }
        }
    }
}