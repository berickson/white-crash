#pragma once
#include "Arduino.h"

class QuadratureEncoder {
  public:
  const int pin_sensor_a; // sensor that triggers first for forward
  const int pin_sensor_b;
  float meters_per_tick = 0.0;;

  volatile unsigned long last_odometer_a_us = 0;
  volatile unsigned long last_odometer_b_us = 0;
  volatile unsigned long odometer_ab_us = 0;

  volatile long odometer_a = 0;
  volatile long odometer_b = 0;


  
  QuadratureEncoder(int _pin_sensor_a, int _pin_sensor_b, float _meters_per_tick) :
    pin_sensor_a(_pin_sensor_a),
    pin_sensor_b(_pin_sensor_b),
    meters_per_tick(_meters_per_tick)
  {
    pinMode(pin_sensor_a, INPUT);
    pinMode(pin_sensor_b, INPUT);
  }

  float get_meters() {
    return meters_per_tick * (odometer_a + odometer_b) / 2;
  }
  
  void sensor_a_changed() {
    last_odometer_a_us=micros();
    if(digitalRead(pin_sensor_a)==digitalRead(pin_sensor_b)){
      --odometer_a;
      odometer_ab_us = last_odometer_a_us - last_odometer_b_us;
    } else {
      ++odometer_a;
    }
  }
  
  void sensor_b_changed() {
    last_odometer_b_us=micros();
    if(digitalRead(pin_sensor_a)==digitalRead(pin_sensor_b)){
      ++odometer_b;
      odometer_ab_us = last_odometer_b_us - last_odometer_a_us;
    } else {
      --odometer_b;
    }
  }
};