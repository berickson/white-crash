#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "drv8833.h"
#include "CRSFforArduino.hpp"
#include "TinyGPS++.h"
#include "quadrature_encoder.h"
#include <QMC5883LCompass.h>
#include <vector>
#include "Fsm.h"


// hard code some interesting gps locations
class lat_lon {
public:
  double lat;
  double lon;
};



lat_lon sidewalk_in_front_of_driveway = {33.802051, -118.123404};
lat_lon sidewalk_by_jimbos_house = {33.802057, -118.123248};

std::vector<lat_lon> route_waypoints = {
  sidewalk_by_jimbos_house,
  sidewalk_in_front_of_driveway,
};


//////////////////////////////////
// pin assignments

const int pin_left_fwd = 2;
const int pin_left_rev = 3;
const int pin_right_fwd = 4;
const int pin_right_rev = 5;
const int pin_compass_sda = 6;
const int pin_compass_scl = 7;
const int pin_test = 8;
const int pin_battery_voltage = 9;

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

//////////////////////////////////
// Interrupt handlers

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

void handle_crsf_message(serialReceiverLayer::rcChannels_t * rc_channels) {
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
}


void  _mpu()
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

// sets motor speeds to go to a lat/lon
// returns true if arrived
bool go_toward_lat_lon(lat_lon destination) {
  if (!gps.location.isValid()) {
    left_motor.go(0);
    right_motor.go(0);
    return false;
  }

  double distance_remaining = gps.distanceBetween(gps.location.lat(), gps.location.lng(), destination.lat, destination.lon);
  if (distance_remaining < 2.0) {
    left_motor.go(0);
    right_motor.go(0);
    return true;
  }

  // subtract courseTo from 360 to get postive ccw
  double desired_bearing_degrees = 360. - gps.courseTo(gps.location.lat(), gps.location.lng(), destination.lat, destination.lon);
  double current_heading_degrees = compass.getAzimuth();

  double heading_error = desired_bearing_degrees - current_heading_degrees;
  while (heading_error > 180) {
    heading_error -= 360;
  }
  while (heading_error < -180) {
    heading_error += 360;
  }


  double left_motor_speed = 0.0;
  double right_motor_speed = 0.0;
  std::string direction;

    if (heading_error < -20) {
      // turn left
      left_motor_speed = 0.7;
      right_motor_speed = 1.0;
      direction = "left";
    } else if (heading_error > 20) {
      // turn right
      left_motor_speed = 1.0;
      right_motor_speed = 0.7;
      direction = "left";
    } else {
      // go straight
      left_motor_speed = 1.0;
      right_motor_speed = 1.0;
      direction = "straight";
    }


    //printf("left_motor_speed: %0.4f right_motor_speed: %0.4f\n", left_motor_speed, right_motor_speed);
    left_motor.go(left_motor_speed);
    right_motor.go(right_motor_speed);

    if (every_n_ms(last_loop_time_ms, loop_time_ms, 1000)) {
      Serial.printf("destination: %f %f direction: %s, distance_remaining: %0.4f, desired_bearing_degrees: %0.4f current_heading_degrees: %0.4f heading_error: %0.4f\n",
        destination.lat,
        destination.lon,
        direction.c_str(),
        distance_remaining,
        desired_bearing_degrees,
        current_heading_degrees,
        heading_error);
    }
    return false;
}

void update_motor_speeds()
{
  // stop on failsafe
  if (rx_esc == 0 || rx_str == 0) {
    right_motor.go(0);
    left_motor.go(0);
    return;
  }

  double speed = 0.0;
  double str_speed = 0.0;

  // calibration constants
  const double min_esc = 176;
  const double max_esc = 1810;
  const double min_str = 176;
  const double max_str = 1810;
  const double esc_deadband_low = 980;
  const double esc_deadband_high = 1000;
  const double str_deadband_low = 980;
  const double str_deadband_high = 1000;

  if (rx_esc > esc_deadband_high) {
    speed = (rx_esc - esc_deadband_high) / (max_esc - esc_deadband_high);
  }
  else if (rx_esc < esc_deadband_low) {
    speed = (rx_esc - esc_deadband_low) / (esc_deadband_low - min_esc);
  }
  else {
    speed = 0.0;
  }

  if (rx_str > str_deadband_high) {
    str_speed = (rx_str - str_deadband_high) / (max_str - str_deadband_high);
  }
  else if (rx_str < str_deadband_low) {
    str_speed = (rx_str - str_deadband_low) / (str_deadband_low - min_str);
  }
  else {
    str_speed = 0.0;
  }

  if (abs(speed) + abs(str_speed) > 1.0) {
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
}

//////////////////////////////////
// Finite state machine
class HandMode : public Task {
  public:
  HandMode() {
    name = "hand";
  }

  // implement the execute method
  virtual void execute() override{
    update_motor_speeds();
  }

} hand_mode;

class OffMode : public Task {
  public:
  OffMode() {
    name = "off";
  }

  // implement the execute method
  virtual void execute() override{
    left_motor.go(0);
    right_motor.go(0);
  }

} off_mode;

class AutoMode : public Task {
  public:
  AutoMode() {
    name = "auto";
  }
  int step = 0;

  void begin() override {
    done = false;
    Serial.write("auto mode begin\n");
    step = 0;
  }

  void execute() override {
    bool arrived = go_toward_lat_lon(route_waypoints[step]);
    if (arrived) {
      Serial.printf("arrived at waypoint %d ", step);
      step++;
      if (step >= route_waypoints.size()) {
        done = true;
        Serial.write(", done with route\n");
      } else {
        Serial.printf("going to waypoint %d\n", step);
      }
    }
  }

  void end() override {
    Serial.write("auto mode end\n");
    step = 0;
  }

} auto_mode;

class FailsafeMode : public Task {
  public:
  FailsafeMode() {
    name = "failsafe";
  }
  void execute() override {
    Serial.write("failsafe mode\n");
    left_motor.go(0);
    right_motor.go(0);
  }
} failsafe_mode;

std::vector<Task *> tasks = {
  &hand_mode,
  &off_mode,
  &auto_mode,
  &failsafe_mode,
};

std::vector<Fsm::Edge> edges = {
  // from, event, to
  Fsm::Edge("auto", "done", "hand"),
  Fsm::Edge("*", "auto", "auto"),
  Fsm::Edge("*", "failsafe", "failsafe"),
  Fsm::Edge("*", "hand", "hand"),
  Fsm::Edge("*", "off", "off"),
};

Fsm fsm(tasks, edges);

//////////////////////////////////
// Main setup and loop

void setup() {
  Serial.begin(115200);
  Wire.setPins(pin_compass_sda, pin_compass_scl);
  Wire.begin();

  fsm.begin();

  Serial.write("tank-train\n");
  crsf_serial.begin(420000, SERIAL_8N1, pin_csrf_rx, pin_csrf_tx);
  gps_serial.begin(115200, SERIAL_8N1, pin_gps_rx, pin_gps_tx);
  compass.init();
  compass.setCalibration(-950, 675, -1510, 47, 0, 850);
  // see https://www.magnetic-declination.com/
  compass.setMagneticDeclination(11, 24);


  // quadrature encoders
  attachInterrupt(digitalPinToInterrupt(pin_left_encoder_a), left_a_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_left_encoder_b), left_b_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_right_encoder_a), right_a_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_right_encoder_b), right_b_changed, CHANGE);

  // mpu
  // if (has_mpu) {
  //     if (!mpu.begin()) {
  //     Serial.write("failed to connect to mpu\n");
  //   }
  //   Serial.write("mpu connected\n");
  //   setup_mpu();
  // }

  // crsf
  if (!crsf.begin()) {
    while (true){
        Serial.println("CRSF for Arduino initialization failed!");
        delay(1000);
    }
  }

  crsf.setRcChannelsCallback(handle_crsf_message);
 
  left_motor.init(pin_left_fwd, pin_left_rev);
  right_motor.init(pin_right_fwd, pin_right_rev);
  pinMode(pin_test, OUTPUT);
  pinMode(pin_built_in_led, OUTPUT);
  pinMode(pin_battery_voltage, INPUT);
}

double v_bat = NAN;

void loop() {
  last_loop_time_ms = loop_time_ms;
  loop_time_ms = millis();
  if (has_mpu) read_mpu();
  while (gps_serial.available()) {
    gps.encode(gps_serial.read());
  }

  bool every_10_ms = every_n_ms(last_loop_time_ms, loop_time_ms, 10);
  bool every_100_ms = every_n_ms(last_loop_time_ms, loop_time_ms, 100);
  bool every_1000_ms = every_n_ms(last_loop_time_ms, loop_time_ms, 1000);

  if (isnan(v_bat) || every_1000_ms) {
    analogReadResolution(12);
    int sum = 0;
    for (int i = 0; i < 16; i++) {
      sum += analogRead(pin_battery_voltage);
    }
    v_bat = 8.4 * sum / 36823.;
    Serial.printf("vbat: %0.4f sum: %d\n", v_bat, sum);
  }


  if (every_100_ms) {
    fsm.execute();
  }

  // read mode from rx_aux channel
  enum aux_mode_t {aux_mode_failsafe, aux_mode_hand, aux_mode_off, aux_mode_auto};


  if (every_10_ms) {
    static aux_mode_t last_aux_mode = aux_mode_failsafe;
    aux_mode_t aux_mode;
    if (rx_aux == 0) {
      aux_mode = aux_mode_failsafe;
    } else if (rx_aux < 500) {
      aux_mode = aux_mode_hand;
    } else if (rx_aux < 1200) { 
      aux_mode = aux_mode_off;
    } else {
      aux_mode = aux_mode_auto;
    }
    if (aux_mode != last_aux_mode) {
      switch (aux_mode) {
        case aux_mode_failsafe:
          fsm.set_event("failsafe");
          break;
        case aux_mode_hand:
          fsm.set_event("hand");
          break;
        case aux_mode_off:
          fsm.set_event("off");
          break;
        case aux_mode_auto:
          fsm.set_event("auto");
          break;
        default:
          fsm.set_event("failsafe");
      }
    }
    last_aux_mode = aux_mode;
  }
  

  if (every_n_ms(last_loop_time_ms, loop_time_ms, 100)) {
    compass.read();

    // update magnetometer limits
    static int min_x = std::numeric_limits<int>::max();
    static int min_y = std::numeric_limits<int>::max();
    static int min_z = std::numeric_limits<int>::max();
    static int max_x = std::numeric_limits<int>::min();
    static int max_y = std::numeric_limits<int>::min();
    static int max_z = std::numeric_limits<int>::min();

    int x = compass.getX();
    int y = compass.getY();
    int z = compass.getZ();

    if (x < min_x) min_x = x;
    if (y < min_y) min_y = y;
    if (z < min_z) min_z = z;
    if (x > max_x) max_x = x;
    if (y > max_y) max_y = y;
    if (z > max_z) max_z = z;

    // Serial.printf("compass: %d [%d, %d, %d] limits (%d, %d, %d, %d %d %d)\n", 
    //   compass.getAzimuth(), 
    //   compass.getX(), compass.getY(), compass.getZ(),
    //   min_x, max_x, min_y, max_y, min_z, max_z
    // );
  }

  // blink for a few ms every second to show signs of life
  if (millis() % 1000 < 5) {
    digitalWrite(pin_built_in_led, HIGH);
  } else {
    digitalWrite(pin_built_in_led, LOW);
  }

  // if (every_n_ms(last_loop_time_ms, loop_time_ms, 100)) {
  //   Serial.printf("esc: %d str: %d left_odo: %d  right_odo: %d\n",rx_esc, rx_str, left_encoder.odometer_a, right_encoder.odometer_a);
  // }

  crsf.update(); // update as fast as possible, will call callbacks when data is ready
  if (every_n_ms(last_loop_time_ms, loop_time_ms, 200))
  {
    crsf.telemetryWriteBattery(v_bat * 100, 0, 0, 0);
    crsf.telemetryWriteCustomFlightMode(fsm.current_task->name, false);
    crsf.telemetryWriteAttitude(0, 0, compass.getAzimuth()*10);
    if(gps.location.isValid()) {
      crsf.telemetryWriteGPS(
        gps.location.lat(), 
        gps.location.lng(), 
        gps.altitude.meters(), 
        gps.speed.kmph(), 
        gps.course.deg(), 
        gps.satellites.value());
    } else {
      crsf.telemetryWriteGPS(0, 0, 0, 0, 0, 0);
    }
  
  }

  if (every_n_ms(last_loop_time_ms, loop_time_ms, 100)) {
    // Serial.printf("mode: %s str: %d esc: %d aux: %d\n", fsm.current_task->name, rx_str, rx_esc, rx_aux);
  }

  digitalWrite(pin_test, !digitalRead(pin_test));
}


