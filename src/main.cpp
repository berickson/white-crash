#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "drv8833.h"

Adafruit_MPU6050 mpu;
DRV8833 left_motor;
DRV8833 right_motor;

// pin assignments
const int pin_left_fwd = 2;
const int pin_left_rev = 3;
const int pin_right_fwd = 4;
const int pin_right_rev = 5;

const int pin_test = 6;

static bool has_mpu = false;

void setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(1); // wait pause until serial console opens
  Serial.write("tank-train\n");

  if (has_mpu) {
      if (!mpu.begin()) {
      Serial.write("failed to connect to mpu\n");
    }
  Serial.write("mpu connected\n");
 }
 
  left_motor.init(pin_left_fwd, pin_left_rev);
  right_motor.init(pin_right_fwd, pin_right_rev);
  pinMode(pin_test, OUTPUT);

  if (has_mpu) { 
    setup_mpu();
  }
}

void setup_mpu()
{
  // setup motion detection
  mpu.setHighPassFilter(MPU6050_HIGHPASS_0_63_HZ);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(20);
  mpu.setInterruptPinLatch(true); // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
}

unsigned long last_loop_time_ms = 0;
unsigned long loop_time_ms = 0;
// returns true if loop time passes through n ms boundary
bool every_n_ms(unsigned long last_loop_ms, unsigned long loop_ms, unsigned long ms) {
  return (last_loop_ms % ms) + (loop_ms - last_loop_ms) >= ms;
}

void read_mpu() {
  if(mpu.getMotionInterruptStatus()) {
    /* Get new sensor events with the readings */
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    /* Print out the values */
    Serial.print("AccelX:");
    Serial.print(a.acceleration.x);
    Serial.print(",");
    Serial.print("AccelY:");
    Serial.print(a.acceleration.y);
    Serial.print(",");
    Serial.print("AccelZ:");
    Serial.print(a.acceleration.z);
    Serial.print(", ");
    Serial.print("GyroX:");
    Serial.print(g.gyro.x);
    Serial.print(",");
    Serial.print("GyroY:");
    Serial.print(g.gyro.y);
    Serial.print(",");
    Serial.print("GyroZ:");
    Serial.print(g.gyro.z);
    Serial.println("");
  }
}

void cycle_motors() {
  static bool accelerate = true;
  static double rate = 0.0;
  auto static last_execute_ms = 0;
  if (last_execute_ms > 0) {
    // time from full forward, to full reverse and back
    const double cycle_time_ms = 5000;
    double elapsed_ms = loop_time_ms - last_execute_ms;
    double delta = 4.0 * elapsed_ms / cycle_time_ms;
    // Serial.printf("elapsed ms: %0.4f delta: %0.4f rate: %0.4f\n",elapsed_ms, delta, rate);
    if(accelerate) {
      rate += delta;
      if(rate >= 1.0) {
        accelerate = false;
        // Serial.write("decelerating\n");
      }
    } else {
      rate -= delta;
      if(rate <= -1.0) {
        accelerate = true;
        // Serial.write("accelerating\n");
      }
    }
    // Serial.printf("post delta: %0.4f rate: %0.4f\n", delta, rate);
  }
  left_motor.go(rate);

  last_execute_ms = loop_time_ms;
}


void loop() {
  last_loop_time_ms = loop_time_ms;
  loop_time_ms = millis();
  if (has_mpu) read_mpu();
  if (every_n_ms(last_loop_time_ms, loop_time_ms, 10)) {
    cycle_motors();
  }

  if (every_n_ms(last_loop_time_ms, loop_time_ms, 1)) {
    digitalWrite(pin_test, loop_time_ms % 2);
  }
}
