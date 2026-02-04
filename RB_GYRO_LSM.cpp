#include "RB_GYRO.h"
#include <util/delay.h>
#include <string.h>
#include "avr/wdt.h"

// LSM6DS3 Constants
#define LSM6DS3_ADDR 0x6A      // Default I2C Address (SA0 to GND)
#define CTRL1_XL     0x10      // Accel Control Register
#define CTRL2_G      0x11      // Gyro Control Register
#define CTRL3_C      0x12      // Control Register 3 (Auto-inc)
#define OUTX_L_G     0x22      // Start of Gyro data

RB_GYRO::RB_GYRO(void) : RB_SoftI2CMaster(0) {
    RB_SoftI2CMaster::SetMode(1);
    Device_Address = LSM6DS3_ADDR;
}

RB_GYRO::RB_GYRO(uint8_t port) : RB_SoftI2CMaster(port) {
    RB_SoftI2CMaster::SetMode(1);
    Device_Address = LSM6DS3_ADDR;
}

// Low-level Read/Write remain mostly the same, using your existing Software I2C
void RB_GYRO::ReadData(uint8_t start_regaddress, uint8_t *buffer, uint8_t datalen) {
    RB_SoftI2CMaster::I2C_Star();
    RB_SoftI2CMaster::I2C_Write(Device_Address << 1 | 0x00);
    RB_SoftI2CMaster::I2C_GetAck();
    RB_SoftI2CMaster::I2C_Write(start_regaddress);
    RB_SoftI2CMaster::I2C_GetAck();
    RB_SoftI2CMaster::I2C_Stop();
    _delay_us(15);
    RB_SoftI2CMaster::I2C_Star();
    RB_SoftI2CMaster::I2C_Write(Device_Address << 1 | 0x01);
    RB_SoftI2CMaster::I2C_GetAck();
    datalen = datalen - 1;
    while (datalen--) {
        *buffer = I2C_Read();
        I2C_PutAck(0);
        buffer++;
    }
    *buffer = I2C_Read();
    RB_SoftI2CMaster::I2C_PutAck(1);
    RB_SoftI2CMaster::I2C_Stop();
}

void RB_GYRO::WriteReg(uint8_t start_regaddress, uint8_t buffer) {
    RB_SoftI2CMaster::I2C_Star();
    RB_SoftI2CMaster::I2C_Write(Device_Address << 1 | 0x00);
    RB_SoftI2CMaster::I2C_GetAck();
    RB_SoftI2CMaster::I2C_Write(start_regaddress);
    RB_SoftI2CMaster::I2C_GetAck();
    RB_SoftI2CMaster::I2C_Write(buffer);
    RB_SoftI2CMaster::I2C_GetAck();
    RB_SoftI2CMaster::I2C_Stop();
}

uint8_t RB_GYRO::begin(void) {
    // Sensitivity for FS = +/- 250 dps is approx 8.75 mdps/LSB
    // 1000 / 8.75 = 114.28 LSB per dps
    gSensitivity = 114.28;

    AngleX = AngleY = AngleZ = 0;
    GyroX = GyroY = GyroZ = 0;
    AccX = AccY = AccZ = 0;
    GyroXoffset = GyroYoffset = GyroZoffset = 0;

    _delay_ms(100);
    
    // 1. Power on Accel: 104Hz, +/- 2g (0x40)
    WriteReg(CTRL1_XL, 0x40); 
    // 2. Power on Gyro: 104Hz, 250 dps (0x40)
    WriteReg(CTRL2_G, 0x40);
    // 3. Enable Block Data Update and Register Address Auto-Increment (0x04)
    WriteReg(CTRL3_C, 0x04 | 0x40); // 0x40 for IF_INC

    _delay_ms(50);
    deviceCalibration();
    return 1;
}

void RB_GYRO::Update(void) {
    static unsigned long last_time = 0;
    double dt, filter_coefficient, ax, ay;

    // LSM6DS3: Read 12 bytes starting from 0x22
    // 0x22-0x27 = Gyro (X,Y,Z), 0x28-0x2D = Accel (X,Y,Z)
    ReadData(OUTX_L_G, i2cData, 12);

    // IMPORTANT: LSM6DS3 is Little-Endian (Low byte at lower address)
    GyroX = (int16_t)(i2cData[1] << 8 | i2cData[0]);
    GyroY = (int16_t)(i2cData[3] << 8 | i2cData[2]);
    GyroZ = (int16_t)(i2cData[5] << 8 | i2cData[4]);
    
    AccX = (int16_t)(i2cData[7] << 8 | i2cData[6]);
    AccY = (int16_t)(i2cData[9] << 8 | i2cData[8]);
    AccZ = (int16_t)(i2cData[11] << 8 | i2cData[10]);

    // Apply offset and sensitivity to Gyro
    GyroX = (GyroX - GyroXoffset) / gSensitivity;
    GyroY = (GyroY - GyroYoffset) / gSensitivity;
    GyroZ = (GyroZ - GyroZoffset) / gSensitivity;

    // Angle calculation (remains same logic as your MPU code)
    ax = atan2(AccX, sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / 3.14159;
    ay = atan2(AccY, sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / 3.14159;

    dt = (double)(millis() - last_time) / 1000.0;
    last_time = millis();

    if (AccZ > 0) {
        AngleX -= GyroY * dt;
        AngleY += GyroX * dt;
    } else {
        AngleX += GyroY * dt;
        AngleY -= GyroX * dt;
    }
    AngleZ += GyroZ * dt;
    AngleZ -= 360 * floor(AngleZ / 360);

    filter_coefficient = 0.5 / (0.5 + dt);
    AngleX = AngleX * filter_coefficient + ax * (1 - filter_coefficient);
    AngleY = AngleY * filter_coefficient + ay * (1 - filter_coefficient);
}

void RB_GYRO::deviceCalibration(void) {
    uint16_t num = 500;
    long xsum = 0, ysum = 0, zsum = 0;

    for (uint16_t x = 0; x < num; x++) {
        ReadData(OUTX_L_G, i2cData, 6); // Read only Gyro X,Y,Z
        xsum += (int16_t)(i2cData[1] << 8 | i2cData[0]);
        ysum += (int16_t)(i2cData[3] << 8 | i2cData[2]);
        zsum += (int16_t)(i2cData[5] << 8 | i2cData[4]);
        wdt_reset();
        _delay_ms(2);
    }
    GyroXoffset = xsum / (long)num;
    GyroYoffset = ysum / (long)num;
    GyroZoffset = zsum / (long)num;
}
