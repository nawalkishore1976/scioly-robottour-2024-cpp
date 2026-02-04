#include "RB_GYROMAGNET.h"
#include <util/delay.h>
#include <string.h>
#include "avr/wdt.h"
#include <math.h>
#include <EEPROM.h>

// FSR sensitivity lookup tables (LSB per unit)
const double GYRO_SENSITIVITY[] = {131.0, 65.5, 32.8, 16.4}; // LSB/°/s for ±250,500,1000,2000
const double ACCEL_SENSITIVITY[] = {16384.0, 8192.0, 4096.0, 2048.0}; // LSB/g for ±2,4,8,16g
const double MAG_SENSITIVITY = 0.15; // µT/LSB (fixed for ±4900µT range)

// Power consumption estimates (mA) - based on datasheet
const float POWER_CONSUMPTION[] = {
    2.5,  // Normal mode (all sensors active)
    0.8,  // Low power mode  
    0.006 // Sleep mode (6µA)
};

RB_GYROMAGNET::RB_GYROMAGNET(void) : RB_SoftI2CMaster(0)
{
    RB_SoftI2CMaster::SetMode(1);
    Device_Address = ICM20948_ADDRESS_AD0_LOW;
    currentBank = 0xFF;
    
    // Default configuration for optimal power/performance
    current_gyro_fsr = GYRO_FSR_250DPS;
    current_accel_fsr = ACCEL_FSR_2G;  
    current_sample_rate = SAMPLE_RATE_LOW_POWER;
    current_power_mode = POWER_MODE_NORMAL;
    
    updateSensitivity();
}

bool RB_GYROMAGNET::beginOptimized(GyroFSR gyro_fsr, AccelFSR accel_fsr, SampleRate sample_rate)
{
    Serial.println("🚀 ICM20948 Optimized Initialization");
    Serial.println("Features: 2.5mW Power | ±250-2000°/s | ±2-16g | ±4900µT | DMP | Android Support");
    
    current_gyro_fsr = gyro_fsr;
    current_accel_fsr = accel_fsr;
    current_sample_rate = sample_rate;
    
    delay(100);
    
    // Basic initialization
    selectBank(ICM20948_BANK_0);
    
    // Check WHO_AM_I
    ReadData(ICM20948_WHO_AM_I, i2cData, 1);
    if (i2cData[0] != 0xEA) {
        Serial.println("❌ ICM20948 not found!");
        return false;
    }
    
    // Reset device
    WriteReg(ICM20948_PWR_MGMT_1, 0x80);
    delay(100);
    
    // Configure power management for optimal 2.5mW consumption
    configurePowerOptimal();
    
    // Configure gyroscope FSR
    setGyroFSR(gyro_fsr);
    
    // Configure accelerometer FSR
    setAccelFSR(accel_fsr);
    
    // Set sample rate for power optimization
    setSampleRate(sample_rate);
    
    // Configure 16-bit ADCs with programmable filters
    setDLPF(DLPF_119HZ, DLPF_119HZ); // Good balance of noise vs power
    
    // Initialize magnetometer (±4900µT range)
    if (!initMagnetometer()) {
        Serial.println("⚠️  Magnetometer initialization failed");
    }
    
    // Initialize auxiliary I2C interface
    initAuxI2C();
    
    Serial.print("✅ ICM20948 initialized - Gyro: ±");
    Serial.print(250 * (1 << gyro_fsr)); Serial.print("°/s, Accel: ±");
    Serial.print(2 * (1 << accel_fsr)); Serial.println("g");
    
    estimated_power_mw = calculatePowerConsumption();
    Serial.print("Estimated power consumption: "); 
    Serial.print(estimated_power_mw, 2); Serial.println(" mW");
    
    return true;
}

// === PROGRAMMABLE FSR CONFIGURATION ===

void RB_GYROMAGNET::setGyroFSR(GyroFSR fsr)
{
    selectBank(ICM20948_BANK_2);
    
    ReadData(ICM20948_GYRO_CONFIG_1, i2cData, 1);
    i2cData[0] = (i2cData[0] & 0xF9) | (fsr << 1); // Clear bits 2:1, set new FSR
    WriteReg(ICM20948_GYRO_CONFIG_1, i2cData[0]);
    
    current_gyro_fsr = fsr;
    gSensitivity = GYRO_SENSITIVITY[fsr];
    
    Serial.print("Gyro FSR set to ±"); 
    Serial.print(250 * (1 << fsr)); Serial.println("°/s");
}

void RB_GYROMAGNET::setAccelFSR(AccelFSR fsr)
{
    selectBank(ICM20948_BANK_2);
    
    ReadData(ICM20948_ACCEL_CONFIG, i2cData, 1);
    i2cData[0] = (i2cData[0] & 0xF9) | (fsr << 1); // Clear bits 2:1, set new FSR
    WriteReg(ICM20948_ACCEL_CONFIG, i2cData[0]);
    
    current_accel_fsr = fsr;
    aSensitivity = ACCEL_SENSITIVITY[fsr];
    
    Serial.print("Accel FSR set to ±"); 
    Serial.print(2 * (1 << fsr)); Serial.println("g");
}

// === POWER MANAGEMENT (2.5mW OPTIMIZATION) ===

void RB_GYROMAGNET::configurePowerOptimal(void)
{
    selectBank(ICM20948_BANK_0);
    
    // Wake up and select best clock source
    WriteReg(ICM20948_PWR_MGMT_1, 0x01); // Auto-select clock
    delay(30);
    
    // Enable all sensors but optimize for power
    WriteReg(ICM20948_PWR_MGMT_2, 0x00); // Enable gyro and accel
    
    // Configure low power mode
    WriteReg(ICM20948_LP_CONFIG, 0x40); // Enable gyro cycle mode for power savings
    
    current_power_mode = POWER_MODE_NORMAL;
    Serial.println("✅ Power optimized for 2.5mW operation");
}

void RB_GYROMAGNET::enableLowPowerMode(void)
{
    selectBank(ICM20948_BANK_0);
    
    // Enable low power mode
    WriteReg(ICM20948_LP_CONFIG, 0x60); // Enable accel and gyro cycle modes
    WriteReg(ICM20948_PWR_MGMT_1, 0x21); // Low power mode + auto clock
    
    current_power_mode = POWER_MODE_LOW_POWER;
    estimated_power_mw = POWER_CONSUMPTION[POWER_MODE_LOW_POWER];
    
    Serial.print("🔋 Low power mode enabled ("); 
    Serial.print(estimated_power_mw, 1); Serial.println(" mW)");
}

void RB_GYROMAGNET::enableSleepMode(void)
{
    selectBank(ICM20948_BANK_0);
    WriteReg(ICM20948_PWR_MGMT_1, 0x41); // Sleep mode
    
    current_power_mode = POWER_MODE_SLEEP;
    estimated_power_mw = POWER_CONSUMPTION[POWER_MODE_SLEEP];
    
    Serial.println("😴 Sleep mode enabled (6µA consumption)");
}

void RB_GYROMAGNET::wakeUp(void)
{
    selectBank(ICM20948_BANK_0);
    WriteReg(ICM20948_PWR_MGMT_1, 0x01); // Wake up
    delay(30);
    
    current_power_mode = POWER_MODE_NORMAL;
    estimated_power_mw = calculatePowerConsumption();
    
    Serial.println("⏰ Device awakened");
}

float RB_GYROMAGNET::getCurrentConsumption(void)
{
    return estimated_power_mw;
}

// === ENHANCED DATA ACCESS WITH FSR AWARENESS ===

void RB_GYROMAGNET::Update(void)
{
    static unsigned long last_time = 0;
    double dt, filter_coefficient;
    double ax, ay, az;
    
    selectBank(ICM20948_BANK_0);
    
    // Read all sensor data (16-bit ADCs)
    ReadData(ICM20948_ACCEL_XOUT_H, i2cData, 12);
    
    // Parse accelerometer (FSR-aware)
    AccX = (i2cData[0] << 8) | i2cData[1];
    AccY = (i2cData[2] << 8) | i2cData[3];
    AccZ = (i2cData[4] << 8) | i2cData[5];
    
    // Parse gyroscope (FSR-aware) 
    GyroX = ((i2cData[6] << 8) | i2cData[7]) - calibration.gyroOffset[0];
    GyroY = ((i2cData[8] << 8) | i2cData[9]) - calibration.gyroOffset[1];
    GyroZ = ((i2cData[10] << 8) | i2cData[11]) - calibration.gyroOffset[2];
    
    // Read magnetometer (±4900µT range)
    readMagnetometer();
    
    // Calculate angles with current FSR sensitivity
    double gyroXdps = GyroX / gSensitivity;
    double gyroYdps = GyroY / gSensitivity;
    double gyroZdps = GyroZ / gSensitivity;
    
    // Accelerometer angles (FSR-compensated)
    ax = atan2(AccX, sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180.0 / M_PI;
    ay = atan2(AccY, sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180.0 / M_PI;
    
    dt = (double)(millis() - last_time) / 1000.0;
    last_time = millis();
    
    if (dt > 0) {
        // Integrate gyroscope
        if (AccZ > 0) {
            AngleX = AngleX - gyroYdps * dt;
            AngleY = AngleY + gyroXdps * dt;
        } else {
            AngleX = AngleX + gyroYdps * dt;
            AngleY = AngleY - gyroXdps * dt;
        }
        AngleZ += gyroZdps * dt;
        AngleZ = AngleZ - 360.0 * floor(AngleZ / 360.0);
        
        // Complementary filter
        filter_coefficient = 0.98;
        AngleX = AngleX * filter_coefficient + ax * (1.0 - filter_coefficient);
        AngleY = AngleY * filter_coefficient + ay * (1.0 - filter_coefficient);
    }
}

// === FSR-AWARE GETTERS ===

double RB_GYROMAGNET::getGyroX(void) { return GyroX / gSensitivity; }
double RB_GYROMAGNET::getGyroY(void) { return GyroY / gSensitivity; }  
double RB_GYROMAGNET::getGyroZ(void) { return GyroZ / gSensitivity; }

int16_t RB_GYROMAGNET::getGyroXRaw(void) { return GyroX; }
int16_t RB_GYROMAGNET::getGyroYRaw(void) { return GyroY; }
int16_t RB_GYROMAGNET::getGyroZRaw(void) { return GyroZ; }

float RB_GYROMAGNET::getAccXg(void) { 
    return (AccX + calibration.accelOffset[0]) * calibration.accelScale[0] / aSensitivity; 
}
float RB_GYROMAGNET::getAccYg(void) { 
    return (AccY + calibration.accelOffset[1]) * calibration.accelScale[1] / aSensitivity; 
}
float RB_GYROMAGNET::getAccZg(void) { 
    return (AccZ + calibration.accelOffset[2]) * calibration.accelScale[2] / aSensitivity; 
}

float RB_GYROMAGNET::getAccelMagnitude(void) {
    float ax = getAccXg();
    float ay = getAccYg(); 
    float az = getAccZg();
    return sqrt(ax*ax + ay*ay + az*az);
}

// === 3-AXIS COMPASS (±4900µT) ===

float RB_GYROMAGNET::getMagXuT(void) {
    return (MagX - calibration.magOffset[0]) * calibration.magScale[0] * MAG_SENSITIVITY;
}

float RB_GYROMAGNET::getMagYuT(void) {
    return (MagY - calibration.magOffset[1]) * calibration.magScale[1] * MAG_SENSITIVITY;
}

float RB_GYROMAGNET::getMagZuT(void) {
    return (MagZ - calibration.magOffset[2]) * calibration.magScale[2] * MAG_SENSITIVITY;
}

float RB_GYROMAGNET::getMagMagnitude(void) {
    float mx = getMagXuT();
    float my = getMagYuT();
    float mz = getMagZuT();
    return sqrt(mx*mx + my*my + mz*mz);
}

double RB_GYROMAGNET::getCompassTiltCompensated(void) {
    // Tilt-compensated compass using accelerometer and magnetometer
    float ax = getAccXg();
    float ay = getAccYg();
    float az = getAccZg();
    float mx = getMagXuT();
    float my = getMagYuT();
    float mz = getMagZuT();
    
    // Normalize accelerometer
    float norm = sqrt(ax*ax + ay*ay + az*az);
    ax /= norm; ay /= norm; az /= norm;
    
    // Tilt compensation
    float pitch = asin(-ax);
    float roll = atan2(ay, az);
    
    float mag_x = mx * cos(pitch) + mz * sin(pitch);
    float mag_y = mx * sin(roll) * sin(pitch) + my * cos(roll) - mz * sin(roll) * cos(pitch);
    
    double heading = atan2(mag_y, mag_x) * 180.0 / M_PI;
    if (heading < 0) heading += 360.0;
    
    return heading;
}

// === ANDROID SUPPORT FEATURES ===

void RB_GYROMAGNET::enableAndroidOrientation(void) {
    // Configure sensor for Android compatibility
    setGyroFSR(GYRO_FSR_2000DPS);   // Android typically uses ±2000°/s
    setAccelFSR(ACCEL_FSR_8G);      // Android typically uses ±8g
    setSampleRate(SAMPLE_RATE_250HZ); // 250Hz is common for Android
    
    Serial.println("📱 Android orientation support enabled");
}

void RB_GYROMAGNET::getAndroidOrientation(float *azimuth, float *pitch, float *roll) {
    Update();
    
    *azimuth = getCompassTiltCompensated(); // 0-360° (North = 0°)
    *pitch = getAngleY();                   // -180 to +180°
    *roll = getAngleX();                    // -90 to +90°
}

// === AUXILIARY I2C INTERFACE ===

bool RB_GYROMAGNET::initAuxI2C(void) {
    selectBank(ICM20948_BANK_0);
    WriteReg(ICM20948_USER_CTRL, 0x20); // Enable I2C master mode
    
    selectBank(ICM20948_BANK_3);
    WriteReg(ICM20948_I2C_MST_CTRL, 0x4D); // 400kHz I2C master clock
    WriteReg(ICM20948_I2C_MST_DELAY_CTRL, 0x01);
    
    selectBank(ICM20948_BANK_0);
    Serial.println("🔗 Auxiliary I2C interface initialized (400kHz)");
    return true;
}

void RB_GYROMAGNET::writeAuxI2C(uint8_t slave_addr, uint8_t reg_addr, uint8_t data) {
    selectBank(ICM20948_BANK_3);
    
    WriteReg(ICM20948_I2C_SLV0_ADDR, slave_addr);     // Slave address (write)
    WriteReg(ICM20948_I2C_SLV0_REG, reg_addr);        // Register address  
    WriteReg(ICM20948_I2C_SLV0_DO, data);             // Data to write
    WriteReg(ICM20948_I2C_SLV0_CTRL, 0x81);           // Enable, write 1 byte
    
    delay(2);
    selectBank(ICM20948_BANK_0);
}

uint8_t RB_GYROMAGNET::readAuxI2C(uint8_t slave_addr, uint8_t reg_addr) {
    selectBank(ICM20948_BANK_3);
    
    WriteReg(ICM20948_I2C_SLV0_ADDR, slave_addr | 0x80); // Slave address (read)
    WriteReg(ICM20948_I2C_SLV0_REG, reg_addr);           // Register address
    WriteReg(ICM20948_I2C_SLV0_CTRL, 0x81);              // Enable, read 1 byte
    
    delay(2);
    selectBank(ICM20948_BANK_0);
    
    ReadData(ICM20948_EXT_SLV_SENS_DATA_00, i2cData, 1);
    return i2cData[0];
}

// === HERMETICALLY SEALED MEMS STATUS ===

bool RB_GYROMAGNET::selfTest(void) {
    Serial.println("🔬 Performing MEMS self-test...");
    
    // Self-test procedure for hermetically sealed MEMS
    selectBank(ICM20948_BANK_2);
    
    // Enable self-test for gyroscope
    WriteReg(ICM20948_GYRO_CONFIG_2, 0x38); // Enable XYZ self-test
    delay(50);
    
    // Read self-test values
    ReadData(ICM20948_GYRO_XOUT_H, i2cData, 6);
    int16_t gyro_st[3];
    gyro_st[0] = (i2cData[0] << 8) | i2cData[1];
    gyro_st[1] = (i2cData[2] << 8) | i2cData[3];
    gyro_st[2] = (i2cData[4] << 8) | i2cData[5];
    
    // Disable self-test
    WriteReg(ICM20948_GYRO_CONFIG_2, 0x00);
    
    // Check if values are within expected range (MEMS integrity)
    bool test_passed = (abs(gyro_st[0]) > 200 && abs(gyro_st[0]) < 3000) &&
                       (abs(gyro_st[1]) > 200 && abs(gyro_st[1]) < 3000) &&  
                       (abs(gyro_st[2]) > 200 && abs(gyro_st[2]) < 3000);
    
    Serial.print("MEMS Self-test: ");
    Serial.println(test_passed ? "✅ PASSED" : "❌ FAILED");
    
    return test_passed;
}

void RB_GYROMAGNET::getDeviceInfo(void) {
    Serial.println("\n=== ICM20948 Device Information ===");
    Serial.println("Features:");
    Serial.println("• Lowest Power 9-Axis: 2.5 mW");
    Serial.println("• 3-Axis Gyro: ±250/500/1000/2000 dps");
    Serial.println("• 3-Axis Accel: ±2/4/8/16 g");  
    Serial.println("• 3-Axis Compass: ±4900 µT");
    Serial.println("• Onboard DMP for Android");
    Serial.println("• Auxiliary I2C: 400 kHz");
    Serial.println("• 16-bit ADCs + Programmable Filters");
    Serial.println("• 7 MHz SPI / 400 kHz I2C");
    Serial.println("• Digital Temperature Sensor");
    Serial.println("• VDD: 1.71V - 3.6V");
    Serial.println("• MEMS: Hermetically Sealed");
    
    selectBank(ICM20948_BANK_0);
    ReadData(ICM20948_WHO_AM_I, i2cData, 1);
    Serial.print("Device ID: 0x"); Serial.println(i2cData[0], HEX);
    
    Serial.print("Current Power: "); Serial.print(estimated_power_mw, 2); Serial.println(" mW");
    Serial.print("Gyro FSR: ±"); Serial.print(250 * (1 << current_gyro_fsr)); Serial.println(" dps");
    Serial.print("Accel FSR: ±"); Serial.print(2 * (1 << current_accel_fsr)); Serial.println(" g");
}
