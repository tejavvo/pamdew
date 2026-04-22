#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

/* ---------- I2C ---------- */
#define I2C_PORT I2C_NUM_0
#define SDA_PIN 21
#define SCL_PIN 22
#define I2C_FREQ 400000

#define MAX30102_ADDR 0x57
#define MPU6050_ADDR  0x68

#define MAX_INT_PIN GPIO_NUM_4
#define MPU_INT_PIN GPIO_NUM_5

SemaphoreHandle_t i2c_mutex;
TaskHandle_t max_task_handle;
TaskHandle_t mpu_task_handle;

/* ---------- I2C ---------- */
void i2c_init()
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
    };
    conf.master.clk_speed = I2C_FREQ;
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

esp_err_t i2c_write(uint8_t dev, uint8_t reg, uint8_t data)
{
    uint8_t buf[2] = {reg, data};
    return i2c_master_write_to_device(I2C_PORT, dev, buf, 2, pdMS_TO_TICKS(10));
}

esp_err_t i2c_read(uint8_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(I2C_PORT, dev, &reg, 1, data, len, pdMS_TO_TICKS(10));
}

/* ---------- SENSOR INIT ---------- */
void max30102_init()
{
    i2c_write(MAX30102_ADDR, 0x09, 0x03); // SpO2 mode
    i2c_write(MAX30102_ADDR, 0x0A, 0x27); // sample rate ~100Hz
    i2c_write(MAX30102_ADDR, 0x02, 0xC0); // enable interrupts
}

void mpu6050_init()
{
    i2c_write(MPU6050_ADDR, 0x6B, 0x00); // wake up
    i2c_write(MPU6050_ADDR, 0x38, 0x01); // data ready interrupt
}

/* ---------- READ + PARSE ---------- */
void read_max30102()
{
    uint8_t buf[6];
    i2c_read(MAX30102_ADDR, 0x07, buf, 6);

    uint32_t red = ((buf[0]<<16)|(buf[1]<<8)|buf[2]) & 0x3FFFF;
    uint32_t ir  = ((buf[3]<<16)|(buf[4]<<8)|buf[5]) & 0x3FFFF;

    printf("MAX -> RED:%lu IR:%lu\n", red, ir);
}

void read_mpu6050()
{
    uint8_t buf[6];
    i2c_read(MPU6050_ADDR, 0x3B, buf, 6);

    int16_t ax = (buf[0]<<8)|buf[1];
    int16_t ay = (buf[2]<<8)|buf[3];
    int16_t az = (buf[4]<<8)|buf[5];

    printf("MPU -> AX:%d AY:%d AZ:%d\n", ax, ay, az);
}

/* ---------- ISR ---------- */
static void IRAM_ATTR max_isr(void *arg)
{
    BaseType_t hp = pdFALSE;
    vTaskNotifyGiveFromISR(max_task_handle, &hp);
    if (hp) portYIELD_FROM_ISR();
}

static void IRAM_ATTR mpu_isr(void *arg)
{
    BaseType_t hp = pdFALSE;
    vTaskNotifyGiveFromISR(mpu_task_handle, &hp);
    if (hp) portYIELD_FROM_ISR();
}

/* ---------- TASKS ---------- */
void max_task(void *arg)
{
    while (1)
    {
        /* Wait up to 1 s for an interrupt; if none arrives, poll anyway */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        read_max30102();
        xSemaphoreGive(i2c_mutex);
    }
}

void mpu_task(void *arg)
{
    while (1)
    {
        /* Wait up to 1 s for an interrupt; if none arrives, poll anyway */
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));

        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        read_mpu6050();
        xSemaphoreGive(i2c_mutex);
    }
}

/* ---------- GPIO ---------- */
void gpio_init_interrupts()
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL<<MAX_INT_PIN) | (1ULL<<MPU_INT_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,
    };
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(MAX_INT_PIN, max_isr, NULL);
    gpio_isr_handler_add(MPU_INT_PIN, mpu_isr, NULL);
}

/* ---------- MAIN ---------- */
void app_main()
{
    printf("app_main: starting up\n");

    i2c_init();
    printf("app_main: I2C initialised\n");

    i2c_mutex = xSemaphoreCreateMutex();

    max30102_init();
    mpu6050_init();

    xTaskCreate(max_task, "max_task", 4096, NULL, 3, &max_task_handle);
    xTaskCreate(mpu_task, "mpu_task", 4096, NULL, 2, &mpu_task_handle);
    printf("app_main: tasks created, setting up GPIO interrupts\n");

    gpio_init_interrupts();
    printf("app_main: ready\n");
}
