#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/gpio.h"
#include "time.h"

// I2C defines
// This example will use I2C0 on GPIO8 (SDA) and GPIO9 (SCL) running at 400KHz.
// Pins can be changed, see the GPIO function select table in the datasheet for information on GPIO assignments
#define I2C_PORT i2c0
#define I2C_SDA 8
#define I2C_SCL 9
#define POWER 13
#define PWR_MGMT_1 0x6B //107
#define GYRO_CONFIG 0x1B //27
#define ACCEL_CONFIG 0x1C //28
#define MEASURE_START 0x3B //Accel measurements start
#define MPU6050 0x68
#define LED_PIN 25

int writeRegister(i2c_inst_t * i2c, 
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes);

int readRegister(i2c_inst_t * i2c,
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes);

bool reserved_addr(uint8_t addr);
typedef struct {
    uint8_t 
    accelXH, 
    accelXL,
    accelYH,
    accelYL,
    accelZH,
    accelZL,
    tempH,
    tempL,
    gyroXH,
    gyroXL,
    gyroYH,
    gyroYL,
    gyroZH,
    gryoZL;
} info;

int main()
{
    stdio_init_all();

    // gpio_init(POWER);
    // gpio_set_dir(POWER, true);
    // gpio_put(POWER, true);

    // I2C Initialisation. Using it at 400Khz.
    i2c_init(I2C_PORT, 400*1000);
    
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, true);

    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else
            ret = i2c_read_blocking(I2C_PORT, addr, &rxdata, 1, false);

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    printf("Done.\n");
    info measurements = (info) {0};
    uint8_t powerManagementSettings = 0x00;
    uint8_t gyroConfigSettings = 0x00;
    uint8_t accelConfigSettings = 0x00;

    writeRegister(I2C_PORT, MPU6050, PWR_MGMT_1, &powerManagementSettings, 1);
    writeRegister(I2C_PORT, MPU6050, GYRO_CONFIG, &gyroConfigSettings, 1);
    writeRegister(I2C_PORT, MPU6050, ACCEL_CONFIG, &accelConfigSettings, 1);

    printf("%u\n", sizeof(info));
    uint8_t accelMeasurements[] = {0, 0};
    uint8_t regAddress = 0x47;
    uint16_t final = 0;
    bool neg = false;
    while (true)
    {
        // printf("%u\n", measurements.gyroXH << 8 + measurements.gyroXL);
        sleep_ms(100);
        readRegister(I2C_PORT, MPU6050, regAddress, (uint8_t*)&accelMeasurements, 2);

        final = (accelMeasurements[0] << 8) + accelMeasurements[1];
        neg = false;
        if ((accelMeasurements[0] >> 7) == 1)
        {
            neg = true;
        }
        if (neg)
        {
            final = ((~final) & 0xFFFF) + 1;
        }
        // printf("%u\n", final);
        // printf("%u, %u, %u\n", accelMeasurementsH, accelMeasurementsL, ret);
        // final = final & 0xFF00;
        // printf("%u, %u, %u, %d\n", final, accelMeasurementsH, accelMeasurementsL, neg);
        if (neg && final > 256)
        {
            printf("right\n");
            gpio_put(LED_PIN, true);
        }
        if (!neg && final > 256)
        {
            printf("left\n");
            gpio_put(LED_PIN, false);
        }
        printf("%u\n", time_us_32());
        // readRegister(I2C_PORT, MPU6050, MEASURE_START, (uint8_t*)&measurements, 14);
    }
}

bool reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

int writeRegister(i2c_inst_t * i2c, 
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes)
{
    if (nbytes < 0)
    {
        return 0;
    }
    int bytesWritten = 0;
    uint8_t * msg = malloc(sizeof(uint8_t) * (nbytes + 1));
    msg[0] = reg;
    memcpy(msg + 1, buf, nbytes);
    bytesWritten = i2c_write_blocking(i2c, addr, msg, nbytes + 1, false);

    free(msg);
    msg = NULL;
    return bytesWritten;
}
int readRegister(i2c_inst_t * i2c,
                const uint addr,
                const uint8_t reg,
                uint8_t *buf,
                const uint8_t nbytes)
{
    if (nbytes < 1)
    {
        return 0;
    }
    int bytesRead = 0;
    i2c_write_blocking(i2c, addr, &reg, 1, true);
    bytesRead = i2c_read_blocking(i2c, addr, buf, nbytes, false);
    return bytesRead;
}