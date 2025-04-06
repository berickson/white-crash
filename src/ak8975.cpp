#include "ak8975.h"

AK8975::AK8975(TwoWire& wire, uint8_t address)
: wire(wire), dev_addr(address)
{
}

void AK8975::initialize()
{
}

bool AK8975::test_connection()
{
  wire.beginTransmission(dev_addr);
  wire.write(AK8975_RA_WIA);
  if (wire.endTransmission() == 0)
  {
    wire.requestFrom(dev_addr, uint8_t(1));
    if (wire.available())
    {
      auto val = wire.read();
      if ( (val == 0x48))
      {
        return true;
      }
      // Serial.printf("AK8975::test_connection: unexpected value %02x\n", val);
    }
  }
  return false;
}

uint8_t AK8975::get_device_id()
{
  wire.beginTransmission(dev_addr);
  wire.write(AK8975_RA_WIA);
  wire.endTransmission();
  wire.requestFrom(dev_addr, uint8_t(1));
  if (wire.available())
  {
    return wire.read();
  }
  return 0;
}

void AK8975::dump_registers() {
  // print out the hex register number and the hex value for the first 64 registers
  for (uint8_t reg = 0; reg < 0xff; reg++) {
    wire.beginTransmission(dev_addr);
    wire.write(reg);
    wire.endTransmission();
    wire.requestFrom(dev_addr, uint8_t(1));
    if (wire.available()) {
      // Serial.printf("0x%02x: 0x%02x\n", reg, wire.read());
    }
  }
}

uint8_t AK8975::get_info()
{
  wire.beginTransmission(dev_addr);
  wire.write(AK8975_RA_INFO);
  wire.endTransmission();
  wire.requestFrom(dev_addr, uint8_t(1));
  if (wire.available())
  {
    return wire.read();
  }
  return 0;
}

bool AK8975::get_data_ready()
{
  wire.beginTransmission(dev_addr);
  wire.write(AK8975_RA_ST1);
  wire.endTransmission();
  wire.requestFrom(dev_addr, uint8_t(1));
  if (wire.available())
  {
    return (wire.read() & AK8975_ST1_DRDY_BIT) != 0;
  }
  return false;
}

// use with finish_reading
// call this, then wait at least 10ms, then call finish_reading
void AK8975::begin_reading()
{
  wire.beginTransmission(dev_addr);
  wire.write(AK8975_RA_CNTL);
  wire.write(AK8975_MODE_SINGLE);
  wire.endTransmission();
  reading_is_pending = true;
}

void AK8975::finish_reading(AK8975::CompassReading * reading)
{
  wire.beginTransmission(dev_addr);
  wire.write(AK8975_RA_HXL);
  wire.endTransmission();
  wire.requestFrom(dev_addr, uint8_t(6));
  if (wire.available() >= 6)
  {
    reading->x = wire.read() | (((int16_t)wire.read()) << 8);
    reading->y = wire.read() | (((int16_t)wire.read()) << 8);
    reading->z = wire.read() | (((int16_t)wire.read()) << 8);
  }
  reading_is_pending = false;
}



void AK8975::get_reading(AK8975::CompassReading * reading)
{
  begin_reading();
  delay(10);
  finish_reading(reading);
}

  int16_t AK8975::get_reading_x()
  {
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_CNTL);
    wire.write(AK8975_MODE_SINGLE);
    wire.endTransmission();
    delay(10);
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_HXL);
    wire.endTransmission();
    wire.requestFrom(dev_addr, uint8_t(2));
    if (wire.available() >= 2)
    {
      return (((int16_t)wire.read()) << 8) | wire.read();
    }
    return 0;
  }

  int16_t AK8975::get_reading_y()
  {
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_CNTL);
    wire.write(AK8975_MODE_SINGLE);
    wire.endTransmission();
    delay(10);
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_HYL);
    wire.endTransmission();
    wire.requestFrom(dev_addr, uint8_t(2));
    if (wire.available() >= 2)
    {
      return (((int16_t)wire.read()) << 8) | wire.read();
    }
    return 0;
  }

  int16_t AK8975::get_reading_z()
  {
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_CNTL);
    wire.write(AK8975_MODE_SINGLE);
    wire.endTransmission();
    delay(10);
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_HZL);
    wire.endTransmission();
    wire.requestFrom(dev_addr, uint8_t(2));
    if (wire.available() >= 2)
    {
      return (((int16_t)wire.read()) << 8) | wire.read();
    }
    return 0;
  }

  bool AK8975::get_overflow_status()
  {
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_ST2);
    wire.endTransmission();
    wire.requestFrom(dev_addr, uint8_t(1));
    if (wire.available())
    {
      return (wire.read() & AK8975_ST2_HOFL_BIT) != 0;
    }
    return false;
  }

  bool AK8975::get_data_error()
  {
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_ST2);
    wire.endTransmission();
    wire.requestFrom(dev_addr, uint8_t(1));
    if (wire.available())
    {
      return (wire.read() & AK8975_ST2_DERR_BIT) != 0;
    }
    return false;
  }

  uint8_t AK8975::get_mode()
  {
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_CNTL);
    wire.endTransmission();
    wire.requestFrom(dev_addr, uint8_t(1));
    if (wire.available())
    {
      return (wire.read() >> AK8975_CNTL_MODE_BIT) & ((1 << AK8975_CNTL_MODE_LENGTH) - 1);
    }
    return 0;
  }

  void AK8975::set_mode(uint8_t mode)
  {
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_CNTL);
    wire.write((mode & ((1 << AK8975_CNTL_MODE_LENGTH) - 1)) << AK8975_CNTL_MODE_BIT);
    wire.endTransmission();
  }

  void AK8975::reset()
  {
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_CNTL);
    wire.write(AK8975_MODE_POWERDOWN);
    wire.endTransmission();
  }

  void AK8975::set_self_test(bool enabled)
  {
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_ASTC);
    wire.write(enabled ? 1 : 0);
    wire.endTransmission();
  }

  void AK8975::disable_i2c()
  {
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_I2CDIS);
    wire.write(1);
    wire.endTransmission();
  }

  void AK8975::get_adjustment(CompassReading * adjustment)
  {
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_CNTL);
    wire.write(0x0F);
    wire.endTransmission();
    delay(10);
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_ASAX);
    wire.endTransmission();
    wire.requestFrom(dev_addr, uint8_t(3));
    if (wire.available() >= 3)
    {
      adjustment->x = wire.read();
      adjustment->y = wire.read();
      adjustment->z = wire.read();
    }
  }

  void AK8975::set_adjustment(const CompassReading& adjustment)
   {
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_CNTL);
    wire.write(0x0F);
    wire.endTransmission();
    delay(10);
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_ASAX);
    wire.write(adjustment.x);
    wire.write(adjustment.y);
    wire.write(adjustment.z);
    wire.endTransmission();
  }

  uint8_t AK8975::get_adjustment_x()
  {
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_CNTL);
    wire.write(0x0F);
    wire.endTransmission();
    delay(10);
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_ASAX);
    wire.endTransmission();
    wire.requestFrom(dev_addr, uint8_t(1));
    if (wire.available())
    {
      return wire.read();
    }
    return 0;
  }

  void AK8975::set_adjustment_x(uint8_t x)
  {
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_CNTL);
    wire.write(0x0F);
    wire.endTransmission();
    delay(10);
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_ASAX);
    wire.write(x);
    wire.endTransmission();
  }

  uint8_t AK8975::get_adjustment_y()
  {
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_CNTL);
    wire.write(0x0F);
    wire.endTransmission();
    delay(10);
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_ASAY);
    wire.endTransmission();
    wire.requestFrom(dev_addr, uint8_t(1));
    if (wire.available())
    {
      return wire.read();
    }
    return 0;
  }

  void AK8975::set_adjustment_y(uint8_t y)
  {
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_CNTL);
    wire.write(0x0F);
    wire.endTransmission();
    delay(10);
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_ASAY);
    wire.write(y);
    wire.endTransmission();
  }

  uint8_t AK8975::get_adjustment_z()
  {
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_CNTL);
    wire.write(0x0F);
    wire.endTransmission();
    delay(10);
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_ASAZ);
    wire.endTransmission();
    wire.requestFrom(dev_addr, uint8_t(1));
    if (wire.available())
    {
      return wire.read();
    }
    return 0;
  }

  void AK8975::set_adjustment_z(uint8_t z)
  {
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_CNTL);
    wire.write(0x0F);
    wire.endTransmission();
    delay(10);
    wire.beginTransmission(dev_addr);
    wire.write(AK8975_RA_ASAZ);
    wire.write(z);
    wire.endTransmission();
  }
