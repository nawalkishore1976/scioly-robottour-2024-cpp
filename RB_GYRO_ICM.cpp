#include "RB_GYRO_ICM.h"

// Registers
#define REG_BANK_SEL      0x7F
#define B0_USER_CTRL      0x03
#define B0_PWR_MGMT_1     0x06
#define B0_INT_PIN_CFG    0x0F // Used for Bypass
#define B0_ACCEL_XOUT     0x2D
#define B0_GYRO_XOUT      0x33
#define B2_GYRO_CFG       0x01
#define B2_ACCEL_CFG      0x14

// Mag Registers (AK09916)
#define MAG_CNTL2         0x31 // Control 2
#define MAG_HXL           0x11 // X-axis data start

RB_GYRO_ICM::RB_GYRO_ICM(uint8_t port) : RB_SoftI2CMaster(port) {
    totalZ = 0; localZOffset = 0;
}

void RB_GYRO_ICM::SelectBank(uint8_t bank) {
    WriteReg(ICM_ADDR, REG_BANK_SEL, (bank << 4));
}

uint8_t RB_GYRO_ICM::begin() {
    gSensitivity = 131.0; 
    
    SelectBank(0);
    WriteReg(ICM_ADDR, B0_PWR_MGMT_1, 0x80); // Reset
    delay(100);
    WriteReg(ICM_ADDR, B0_PWR_MGMT_1, 0x01); // Wake
    
    // --- ENABLE BYPASS MODE FOR MAG ---
    // Disable I2C Master mode first
    WriteReg(ICM_ADDR, B0_USER_CTRL, 0x00);
    // Enable Bypass (Bit 1 = 1) so we can see 0x0C
    WriteReg(ICM_ADDR, B0_INT_PIN_CFG, 0x02); 
    
    // --- CONFIGURE MAG ---
    // Set Mag to Continuous Measurement Mode 4 (100Hz)
    WriteReg(MAG_ADDR, MAG_CNTL2, 0x08); 
    
    SelectBank(2);
    WriteReg(ICM_ADDR, B2_GYRO_CFG, 0x01);  // 250dps
    WriteReg(ICM_ADDR, B2_ACCEL_CFG, 0x01); // +/- 2g
    
    SelectBank(0);
    return 1;
}

void RB_GYRO_ICM::Update(uint8_t flags) {
    static unsigned long last_u = 0;
    unsigned long now_u = micros();
    double dt = (last_u == 0) ? 0 : (double)(now_u - last_u) / 1000000.0;
    last_u = now_u;

    SelectBank(0);
    if (flags & 0x01) { // Accel
        ReadData(ICM_ADDR, B0_ACCEL_XOUT, 6, i2cData);
        AccX = (int16_t)(i2cData[0] << 8 | i2cData[1]);
        AccY = (int16_t)(i2cData[2] << 8 | i2cData[3]);
        AccZ = (int16_t)(i2cData[4] << 8 | i2cData[5]);
    }
    if (flags & 0x02) { // Gyro
        ReadData(ICM_ADDR, B0_GYRO_XOUT, 6, i2cData);
        GyroZ = (int16_t)(i2cData[4] << 8 | i2cData[5]); // Z is the 3rd pair
        totalZ += ((GyroZ - GyroZoffset) / gSensitivity) * dt;
    }
}

void RB_GYRO_ICM::UpdateMag() {
    uint8_t magBuf[8]; // Mag output is 6 bytes + ST2 status
    // Read starting from X-axis Low (AK09916 is Little Endian)
    ReadData(MAG_ADDR, MAG_HXL, 8, magBuf);
    
    // AK09916 Data is Little Endian (unlike ICM)
    MagX = (int16_t)(magBuf[1] << 8 | magBuf[0]);
    MagY = (int16_t)(magBuf[3] << 8 | magBuf[2]);
    MagZ = (int16_t)(magBuf[5] << 8 | magBuf[4]);
}

// Separate Read/Write Helpers to support different Device Addresses
void RB_GYRO_ICM::WriteReg(uint8_t dev_addr, uint8_t reg, uint8_t val) {
    I2C_Star();
    I2C_Write(dev_addr << 1 | 0);
    I2C_GetAck();
    I2C_Write(reg);
    I2C_GetAck();
    I2C_Write(val);
    I2C_GetAck();
    I2C_Stop();
}

void RB_GYRO_ICM::ReadData(uint8_t dev_addr, uint8_t reg, uint8_t len, uint8_t *buf) {
    I2C_Star();
    I2C_Write(dev_addr << 1 | 0);
    I2C_GetAck();
    I2C_Write(reg);
    I2C_GetAck();
    I2C_Stop();
    
    I2C_Star();
    I2C_Write(dev_addr << 1 | 1);
    I2C_GetAck();
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = I2C_Read();
        I2C_PutAck(i == (len - 1));
    }
    I2C_Stop();
}
