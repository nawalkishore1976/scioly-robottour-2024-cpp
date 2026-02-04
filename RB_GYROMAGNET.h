#ifndef RB_GYROMAGNET_H_
#define RB_GYROMAGNET_H_

#include <inttypes.h>
#include <Arduino.h>
#include "RB_PORT.h" 
#include "RB_SOFTI2CMASTER.h"

// ICM20948 Full-Scale Range (FSR) Options
enum GyroFSR {
    GYRO_FSR_250DPS  = 0,  // ±250 dps
    GYRO_FSR_500DPS  = 1,  // ±500 dps  
    GYRO_FSR_1000DPS = 2,  // ±1000 dps
    GYRO_FSR_2000DPS = 3   // ±2000 dps
};

enum AccelFSR {
    ACCEL_FSR_2G  = 0,     // ±2g
    ACCEL_FSR_4G  = 1,     // ±4g
    ACCEL_FSR_8G  = 2,     // ±8g
    ACCEL_FSR_16G = 3      // ±16g
};

enum MagFSR {
    MAG_FSR_4900uT = 0     // ±4900 µT (magnetometer has fixed range)
};

// Low Power Modes
enum PowerMode {
    POWER_MODE_NORMAL = 0,
    POWER_MODE_LOW_POWER = 1,
    POWER_MODE_SLEEP = 2
};

// Sample Rate Options (optimized for 2.5mW power consumption)
enum SampleRate {
    SAMPLE_RATE_1000HZ = 0,
    SAMPLE_RATE_500HZ  = 1,
    SAMPLE_RATE_250HZ  = 2,
    SAMPLE_RATE_125HZ  = 3,
    SAMPLE_RATE_62_5HZ = 4,
    SAMPLE_RATE_31_25HZ = 5,
    SAMPLE_RATE_15_625HZ = 6,
    SAMPLE_RATE_LOW_POWER = 7  // Optimized for minimum power
};

// Digital Low Pass Filter Options
enum DLPF_CFG {
    DLPF_196HZ = 0,
    DLPF_152HZ = 1,
    DLPF_119HZ = 2,
    DLPF_51HZ  = 3,
    DLPF_24HZ  = 4,
    DLPF_12HZ  = 5,
    DLPF_6HZ   = 6,
    DLPF_473HZ = 7
};

class RB_GYROMAGNET : public RB_SoftI2CMaster
{
public:
    RB_GYROMAGNET(void);
    RB_GYROMAGNET(uint8_t port);
    
    // === CORE INITIALIZATION ===
    bool begin(void);
    bool beginOptimized(GyroFSR gyro_fsr = GYRO_FSR_250DPS, 
                       AccelFSR accel_fsr = ACCEL_FSR_2G,
                       SampleRate sample_rate = SAMPLE_RATE_LOW_POWER);
    
    // === POWER MANAGEMENT (2.5mW optimization) ===
    void setPowerMode(PowerMode mode);
    PowerMode getPowerMode(void);
    void enableLowPowerMode(void);
    void enableSleepMode(void);
    void wakeUp(void);
    float getCurrentConsumption(void); // Estimate current consumption
    
    // === PROGRAMMABLE FSR CONFIGURATION ===
    void setGyroFSR(GyroFSR fsr);
    void setAccelFSR(AccelFSR fsr);
    GyroFSR getGyroFSR(void);
    AccelFSR getAccelFSR(void);
    
    // === SAMPLE RATE AND FILTERS ===
    void setSampleRate(SampleRate rate);
    void setCustomSampleRate(uint16_t rate_hz);
    void setDLPF(DLPF_CFG gyro_dlpf, DLPF_CFG accel_dlpf);
    SampleRate getSampleRate(void);
    
    // === 16-BIT ADC DATA ACCESS ===
    void Update(void);
    void FastUpdate(void);
    
    // Gyroscope (with FSR-aware scaling)
    double getAngleX(void);
    double getAngleY(void);
    double getAngleZ(void);
    double getGyroX(void);      // Raw degrees/second
    double getGyroY(void);      
    double getGyroZ(void);      
    int16_t getGyroXRaw(void);  // Raw 16-bit ADC value
    int16_t getGyroYRaw(void);  
    int16_t getGyroZRaw(void);  
    
    // Accelerometer (with FSR-aware scaling)  
    int16_t getAccX(void);      // Raw ADC
    int16_t getAccY(void);      
    int16_t getAccZ(void);      
    float getAccXg(void);       // Calibrated g-force
    float getAccYg(void);       
    float getAccZg(void);       
    float getAccelMagnitude(void); // Total acceleration magnitude
    
    // 3-Axis Compass (±4900 µT range)
    int16_t getMagX(void);      // Raw ADC
    int16_t getMagY(void);      
    int16_t getMagZ(void);      
    float getMagXuT(void);      // Calibrated µT
    float getMagYuT(void);      
    float getMagZuT(void);      
    float getMagMagnitude(void); // Total magnetic field strength
    double getCompass(void);    // Compass heading (0-360°)
    double getCompassTiltCompensated(void); // Tilt-compensated heading
    
    // === DIGITAL TEMPERATURE SENSOR ===
    float getTemperature(void);
    float getTemperatureF(void); // Fahrenheit
    
    // === ONBOARD DMP (Digital Motion Processor) ===
    bool initDMP(void);
    bool isDMPEnabled(void);
    void setDMPEnabled(bool enabled);
    bool readDMPPacket(void);
    void getDMPQuaternion(float *q);
    void getDMPEuler(float *euler);
    void getDMPYawPitchRoll(float *ypr);
    void getDMPGravity(float *gravity);
    void getDMPLinearAccel(float *linear_accel);
    void getDMPWorldAccel(float *world_accel);
    
    // === AUXILIARY I2C INTERFACE ===
    bool initAuxI2C(void);
    void writeAuxI2C(uint8_t slave_addr, uint8_t reg_addr, uint8_t data);
    uint8_t readAuxI2C(uint8_t slave_addr, uint8_t reg_addr);
    void readAuxI2C(uint8_t slave_addr, uint8_t reg_addr, uint8_t *buffer, uint8_t length);
    bool isAuxI2CReady(void);
    
    // === ANDROID SUPPORT FEATURES ===
    void enableAndroidOrientation(void);
    void getAndroidOrientation(float *azimuth, float *pitch, float *roll);
    void getRotationMatrix(float *matrix); // 3x3 rotation matrix
    void getLinearAcceleration(float *linear_accel);
    void getGravityVector(float *gravity);
    
    // === ADVANCED CALIBRATION SYSTEM ===
    void calibrateAll(void);
    bool calibrateGyroscope(uint16_t samples = 1000);
    bool calibrateAccelerometer(uint16_t samples = 1000);
    bool calibrateMagnetometer(uint16_t samples = 2000, uint16_t timeout_ms = 30000);
    void saveCalibration(void);
    bool loadCalibration(void);
    bool isFullyCalibrated(void);
    
    // === MOTION DETECTION ===
    void enableMotionDetection(uint8_t threshold);
    void enableWakeOnMotion(uint8_t threshold);
    bool getMotionDetected(void);
    bool getFreeFallDetected(void);
    void enableStepCounter(void);
    uint16_t getStepCount(void);
    
    // === FIFO BUFFER (16-bit data) ===
    void enableFIFO(bool gyro, bool accel, bool mag, bool temp);
    void resetFIFO(void);
    uint16_t getFIFOCount(void);
    bool isFIFOOverflow(void);
    void readFIFOPacket(uint8_t *buffer);
    
    // === COMMUNICATION INTERFACES ===
    void enableSPI(uint32_t freq = 7000000); // 7MHz SPI support
    void enableI2C(uint32_t freq = 400000);  // 400kHz Fast Mode I2C
    
    // === HERMETICALLY SEALED MEMS STATUS ===
    bool selfTest(void);
    bool getMEMSStatus(void);
    void getDeviceInfo(void);
    
private:
    // Current configuration
    GyroFSR current_gyro_fsr;
    AccelFSR current_accel_fsr;
    SampleRate current_sample_rate;
    PowerMode current_power_mode;
    
    // Sensitivity values (FSR-dependent)
    double gSensitivity;
    double aSensitivity;
    double mSensitivity;
    
    // Power consumption tracking
    float estimated_power_mw;
    
    // [Previous private members remain the same...]
    
    // Helper functions for new features
    void updateSensitivity(void);
    void configurePowerOptimal(void);
    float calculatePowerConsumption(void);
    void loadDMPAndroidFirmware(void);
    void configureAuxI2C(void);
};

#endif
