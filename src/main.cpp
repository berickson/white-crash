#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "drv8833.h"
#include "CRSFforArduino.hpp"
#include "TinyGPS++.h"
#include "quadrature_encoder.h"
#include <QMC5883LCompass.h>


//////////////////////////////////
// pin assignments

const int pin_left_fwd = 2;
const int pin_left_rev = 3;
const int pin_right_fwd = 4;
const int pin_right_rev = 5;
const int pin_compass_sda = 6;
const int pin_compass_scl = 7;
const int pin_test = 8;

const int pin_built_in_led = 15;

const int pin_left_encoder_a = 33;
const int pin_left_encoder_b = 34;
const int pin_right_encoder_a = 35;
const int pin_right_encoder_b = 36;
const int pin_gps_rx = 37;
const int pin_gps_tx = 38;
const int pin_csrf_rx = 39;
const int pin_csrf_tx = 40;

static bool has_mpu = false;


//////////////////////////////////
// Globals

Adafruit_MPU6050 mpu;
DRV8833 left_motor;
DRV8833 right_motor;
HardwareSerial crsf_serial(0);
HardwareSerial gps_serial(1);
TinyGPSPlus gps;
QMC5883LCompass compass;
QuadratureEncoder left_encoder(pin_left_encoder_a, pin_left_encoder_b);
QuadratureEncoder right_encoder(pin_right_encoder_a, pin_right_encoder_b);

int rx_str = 0;
int rx_esc = 0;
int rx_aux = 0;



// set up crsf serial to use pin_csrf_rx and pin_csrf_tx
CRSFforArduino crsf = CRSFforArduino(&crsf_serial);

void left_a_changed() {
  left_encoder.sensor_a_changed();
}
void left_b_changed() {
  left_encoder.sensor_b_changed();
}
void right_a_changed() {
  right_encoder.sensor_a_changed();
}
void right_b_changed() {
  right_encoder.sensor_b_changed();
}

void setup() {
  Serial.begin(115200);
  Wire.setPins(pin_compass_sda, pin_compass_scl);
  Wire.begin();

  Serial.write("tank-train\n");
  crsf_serial.begin(420000, SERIAL_8N1, pin_csrf_rx, pin_csrf_tx);
  gps_serial.begin(115200, SERIAL_8N1, pin_gps_rx, pin_gps_tx);
  compass.init();

  // quadrature encoders
  attachInterrupt(digitalPinToInterrupt(pin_left_encoder_a), left_a_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_left_encoder_b), left_b_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_right_encoder_a), right_a_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_right_encoder_b), right_b_changed, CHANGE);


  if (has_mpu) {
      if (!mpu.begin()) {
      Serial.write("failed to connect to mpu\n");
    }
  Serial.write("mpu connected\n");
 }

  // CRSF

  // Initialize CRSF
  if (!crsf.begin())
  {
    // TODO: report setup errors some better way, maybe blink LED
      while (1)
      {
         Serial.println("CRSF for Arduino initialization failed!");
         delay(1000);
      }
  }

  crsf.setRcChannelsCallback([](serialReceiverLayer::rcChannels_t * rc_channels) {
    const int axis_count = 15;
    static float axes[axis_count];

    if(rc_channels->failsafe ){
      rx_str = 0;
      rx_esc = 0;
      rx_aux = 0;

    } else {
      rx_str = crsf.getChannel(1);
      rx_esc = crsf.getChannel(2);
      rx_aux = crsf.getChannel(3);
    }
  });


 
  left_motor.init(pin_left_fwd, pin_left_rev);
  right_motor.init(pin_right_fwd, pin_right_rev);
  pinMode(pin_test, OUTPUT);
  pinMode(pin_built_in_led, OUTPUT);

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
  auto static fast_decay = true;
  if (last_execute_ms > 0) {
    // time from full forward, to full reverse and back
    const double cycle_time_ms =  10000;
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
        fast_decay = !fast_decay;
        // Serial.write("accelerating\n");
      }
    }
    // Serial.printf("post delta: %0.4f rate: %0.4f\n", delta, rate);
  }
  left_motor.go(rate,  fast_decay);
  right_motor.go(rate, fast_decay);

  last_execute_ms = loop_time_ms;
}

void display_gps_info() {
  Serial.print(F("Location: ")); 
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F("  Date/Time: "));
  if (gps.date.isValid())
  {
    Serial.print(gps.date.month());
    Serial.print(F("/"));
    Serial.print(gps.date.day());
    Serial.print(F("/"));
    Serial.print(gps.date.year());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.print(F(" "));
  if (gps.time.isValid())
  {
    if (gps.time.hour() < 10) Serial.print(F("0"));
    Serial.print(gps.time.hour());
    Serial.print(F(":"));
    if (gps.time.minute() < 10) Serial.print(F("0"));
    Serial.print(gps.time.minute());
    Serial.print(F(":"));
    if (gps.time.second() < 10) Serial.print(F("0"));
    Serial.print(gps.time.second());
    Serial.print(F("."));
    if (gps.time.centisecond() < 10) Serial.print(F("0"));
    Serial.print(gps.time.centisecond());
  }
  else
  {
    Serial.print(F("INVALID"));
  }

  Serial.println();
}

void loop() {
  bool gps_ready = false;
  last_loop_time_ms = loop_time_ms;
  loop_time_ms = millis();
  if (has_mpu) read_mpu();
  while (gps_serial.available()) {
      if(gps.encode(gps_serial.read())) {
        gps_ready = true;
      }
  }
  if (every_n_ms(last_loop_time_ms, loop_time_ms, 1000)) {
    display_gps_info();
    gps_ready = false;
  } 

  if (every_n_ms(last_loop_time_ms, loop_time_ms, 100)) {
    compass.read();
    Serial.printf("compass: %d [%d, %d, %d]\n", compass.getAzimuth(), compass.getX(), compass.getY(), compass.getZ());
  }

  if (every_n_ms(last_loop_time_ms, loop_time_ms, 250)) {
    digitalWrite(pin_built_in_led, !digitalRead(pin_built_in_led));
  }

  if (every_n_ms(last_loop_time_ms, loop_time_ms, 10)) {
    // cycle_motors();
    
    double speed = 0.0;
    double str_speed = 0.0;

    // 0 means bad data
    if(rx_esc > 0) {
      if(rx_esc > 1000) {
        speed = (rx_esc - 992) / (1810 - 992.);
      }
      else if(rx_esc < 980) {
        speed = (rx_esc - 980) / (980 - 176.);
      } else {
        speed = 0.0;
      }
    }
    if(rx_str > 0) {
      if(rx_str > 1000) {
        str_speed = (rx_str - 992) / (1810 - 992.);
      } else if (rx_str < 980) {
        str_speed = (rx_str - 980) / (980 - 176.);
      } else {
        str_speed = 0.0;
      }
    }

    if( abs(speed) + abs(str_speed) > 1.0) {
      double scale = 1.0 / (abs(speed) + abs(str_speed));
      speed *= scale;
      str_speed *= scale;
    }

    float right_speed = speed + str_speed;
    float left_speed = speed - str_speed;

    // Serial.printf("speed: %0.4f str_speed: %0.4f left_speed: %0.4f right_speed: %0.4f\n", 
    //   speed, 
    //   str_speed, 
    //   left_speed, 
    //   right_speed);

    right_motor.go(right_speed, false);
    left_motor.go(left_speed, false); 
    
   // right_motor.go(1.0, false); 
  }

  if (every_n_ms(last_loop_time_ms, loop_time_ms, 100)) {
    Serial.printf("esc: %d str: %d left_odo: %d  right_odo: %d\n",rx_esc, rx_str, left_encoder.odometer_a, right_encoder.odometer_a);
  }

  crsf.update(); // update as fast as possible, will call callbacks when data is ready
  if (every_n_ms(last_loop_time_ms, loop_time_ms, 200))
  {
    crsf.telemetryWriteBattery(1200, 10, 300, 71);
    crsf.telemetryWriteCustomFlightMode("TANK", false);
    if(gps.location.isValid()) {
      crsf.telemetryWriteGPS(gps.location.lat(), gps.location.lng(), gps.altitude.meters(), gps.speed.kmph(), gps.course.deg(), gps.satellites.value());
    }
    //crsf.telemetryWriteFlightMode(serialReceiverLayer::FLIGHT_MODE_ACRO);
    // crsf.telemetryWriteCustomFlightMode("TANK", false);
  }

  if (every_n_ms(last_loop_time_ms, loop_time_ms, 100)) {
    //Serial.printf("str: %d esc: %d aux: %d\n", rx_str, rx_esc, rx_aux);
  }

  digitalWrite(pin_test, !digitalRead(pin_test));
}
