#ifndef _AK8975_H_
#define _AK8975_H_
#include <Wire.h>

// see datasheet: https://www.robotpark.com/image/data/PRO/91462/AK8975.pdf

const uint8_t AK8975_ADDRESS_00 = 0x0C;
const uint8_t AK8975_ADDRESS_01 = 0x0D;
const uint8_t AK8975_ADDRESS_10 = 0x0E;
const uint8_t AK8975_ADDRESS_11 = 0x0F;
const uint8_t AK8975_DEFAULT_ADDRESS = AK8975_ADDRESS_01;

const uint8_t AK8975_RA_WIA = 0x00;
const uint8_t AK8975_RA_INFO = 0x01;
const uint8_t AK8975_RA_ST1 = 0x02;
const uint8_t AK8975_RA_HXL = 0x03;
const uint8_t AK8975_RA_HXH = 0x04;
const uint8_t AK8975_RA_HYL = 0x05;
const uint8_t AK8975_RA_HZL = 0x07;
const uint8_t AK8975_RA_HZH = 0x08;
const uint8_t AK8975_RA_HYH = 0x06;
const uint8_t AK8975_RA_ST2 = 0x09;
const uint8_t AK8975_RA_CNTL = 0x0A;
const uint8_t AK8975_RA_RSV = 0x0B; // RESERVED, DO NOT USE
const uint8_t AK8975_RA_ASTC = 0x0C;
const uint8_t AK8975_RA_TS1 = 0x0D; // SHIPMENT TEST, DO NOT USE
const uint8_t AK8975_RA_TS2 = 0x0E; // SHIPMENT TEST, DO NOT USE
const uint8_t AK8975_RA_I2CDIS = 0x0F;
const uint8_t AK8975_RA_ASAX = 0x10;
const uint8_t AK8975_RA_ASAY = 0x11;
const uint8_t AK8975_RA_ASAZ = 0x12;
const uint8_t AK8975_ST1_DRDY_BIT = 0;

const uint8_t AK8975_MODE_POWERDOWN = 0x0;
const uint8_t AK8975_MODE_SINGLE = 0x1;
const uint8_t AK8975_MODE_SELFTEST = 0x8;
const uint8_t AK8975_MODE_FUSEROM = 0xF;

const uint8_t AK8975_ST2_HOFL_BIT = 3;
const uint8_t AK8975_ST2_DERR_BIT = 2;

const uint8_t AK8975_CNTL_MODE_BIT = 3;
const uint8_t AK8975_CNTL_MODE_LENGTH = 4;

class AK8975 {
public:
    AK8975(TwoWire& wire, uint8_t address = AK8975_DEFAULT_ADDRESS);

    void initialize();
    bool test_connection();

    // WIA register
    uint8_t get_device_id();

    // INFO register
    uint8_t get_info();

    // ST1 register
    bool get_data_ready();

    // H* registers
    struct CompassReading {
      int16_t x;
      int16_t y;
      int16_t z;
    };

    bool reading_is_pending = false;
    void begin_reading();
    void finish_reading(CompassReading* reading);

    void get_reading(CompassReading* reading);

    int16_t get_reading_x();
    int16_t get_reading_y();
    int16_t get_reading_z();

    // ST2 register
    bool get_overflow_status();
    bool get_data_error();

    // CNTL register
    uint8_t get_mode();
    void set_mode(uint8_t mode);
    void reset();

    // ASTC register
    void set_self_test(bool enabled);

    void dump_registers();

    // I2CDIS
    void disable_i2c(); // um, why...?

    // ASA* registers
    void get_adjustment(CompassReading * adjustment);
    void set_adjustment(const CompassReading& adjustment);
    uint8_t get_adjustment_x();
    void set_adjustment_x(uint8_t x);
    uint8_t get_adjustment_y();
    void set_adjustment_y(uint8_t y);
    uint8_t get_adjustment_z();
    void set_adjustment_z(uint8_t z);

private:
    TwoWire& wire;
    uint8_t dev_addr;
    uint8_t buffer[6];
    uint8_t mode;
};


class AK8975Compass {
  public:
    AK8975 magnetometer;

    AK8975::CompassReading last_reading;

    AK8975Compass(TwoWire& wire, uint8_t address = AK8975_DEFAULT_ADDRESS)
      : magnetometer(wire, address) {
    }

    // reads the magnetometer, don't call more than once per 10ms
    void update() {
      if (magnetometer.reading_is_pending) {
        magnetometer.finish_reading(&last_reading);
      }
      magnetometer.begin_reading();
    }


    float x_middle = 0.0f;
    float y_middle = 0.0f;
    float z_middle = 0.0f;
    float x_scale = 1.0f;
    float y_scale = 1.0f;
    float z_scale = 1.0f;

    void set_calibration(uint16_t x_min, uint16_t x_max,
                         uint16_t y_min, uint16_t y_max,
                         uint16_t z_min, uint16_t z_max) {
      x_middle = (x_min + x_max) / 2.0f;
      y_middle = (y_min + y_max) / 2.0f;
      z_middle = (z_min + z_max) / 2.0f;

      x_scale = 1.0f / ((x_max - x_min) / 2.0f);
      y_scale = 1.0f / ((y_max - y_min) / 2.0f);
      z_scale = 1.0f / ((z_max - z_min) / 2.0f);
    }

    void get_calibrated_reading(float *x, float *y, float *z) {
      *x = (last_reading.x - x_middle) * x_scale;
      *y = (last_reading.y - y_middle) * y_scale;
      *z = (last_reading.z - z_middle) * z_scale;
    }

    float get_azimuth_degrees() {
      float x = (last_reading.x - x_middle) * x_scale;
      float y = (last_reading.y - y_middle) * y_scale;

      // convert to degrees
      float azimuth = atan2(y, x) * 180.0 / M_PI;
      return azimuth < 0 ? azimuth + 360 : azimuth;
    }

};

#endif /* _AK8975_H_ */