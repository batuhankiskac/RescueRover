#pragma once

#include <Arduino.h>

class TwoWire;

struct sensors_event_t {
    struct {
        float x;
        float y;
        float z;
    } acceleration;
};

enum mpu6050_accel_range_t { MPU6050_RANGE_2_G };
enum mpu6050_bandwidth_t { MPU6050_BAND_44_HZ };

class Adafruit_MPU6050 {
public:
    class Accelerometer {
    public:
        bool getEvent(sensors_event_t *event);
    };

    bool begin(uint8_t address, TwoWire *wire);
    void setAccelerometerRange(mpu6050_accel_range_t range);
    void setFilterBandwidth(mpu6050_bandwidth_t bandwidth);
    Accelerometer *getAccelerometerSensor();

private:
    Accelerometer accelerometer_;
};
