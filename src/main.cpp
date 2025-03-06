#include <Arduino.h>
#include <QMC5883LCompass.h>
#include <VL53L1X.h>
#include "async_SparkFun_TMF882X_Library.h"
#include <SPIFFS.h>
#include <Wire.h>

#include <vector>
#include <algorithm>

#include "Fsm.h"
#include "TinyGPS++.h"
#include "drv8833.h"
#include "quadrature_encoder.h"
#include "speedometer.h"

// micro ros
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/range.h>

#include "RunStatistics.h"
#include "StuckChecker.h"
#include "secrets/wifi_login.h"

#define has_tof 0


#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library

// calibration constants
//float meters_per_odometer_tick = 0.00008204; // S
float meters_per_odometer_tick = 0.00065136; // M

// hard code some interesting gps locations
class lat_lon {
 public:
  double lat;
  double lon;
};

lat_lon sidewalk_in_front_of_driveway = {33.802051, -118.123404};
lat_lon mid1 = {33.8020525,	-118.123365};
lat_lon mid2 = {33.802054,	-118.123326};
lat_lon mid3 = {33.8020555,	-118.123287};
lat_lon sidewalk_by_jimbos_house = {33.802057, -118.123248};

std::vector<lat_lon> route_waypoints = {
    sidewalk_in_front_of_driveway,
    mid1,
    mid2,
    mid3,
    sidewalk_by_jimbos_house,
    mid3,
    mid2,
    mid1,
    sidewalk_in_front_of_driveway,
};

// where we store the compass calibration
const char *compass_calibration_file_path = "/compass_calibration.txt";

//////////////////////////////////
// pin assignments

const int pin_right_rev = 2;
const int pin_right_fwd = 3;
const int pin_left_rev = 4;
const int pin_left_fwd = 5;

const int pin_left_tof_power = 6;
const int pin_center_tof_power = 7;
const int pin_right_tof_power = 13;

const int pin_compass_sda = 18;
const int pin_compass_scl = 21;
const int pin_test = 8;
const int pin_tof_sda = 16;
const int pin_tof_scl = 17;

const int pin_built_in_led = 15;
const int pin_battery_voltage = 14;


const int pin_right_encoder_a = 33;
const int pin_right_encoder_b = 34;
const int pin_left_encoder_b = 35;
const int pin_left_encoder_a = 36;
const int pin_gps_rx = 37;
const int pin_gps_tx = 38;
const int pin_crsf_rx = 39;
const int pin_crsf_tx = 40;

//////////////////////////////////
// i2c addresses
const uint8_t left_tof_i2c_address = 0x14;
const uint8_t center_tof_i2c_address = 0x15;
const uint8_t right_tof_i2c_address = 0x16;

//////////////////////////////////
// Globals

DRV8833 left_motor;
DRV8833 right_motor;
HardwareSerial crsf_serial(0);
HardwareSerial gps_serial(1);
TinyGPSPlus gps;  // currently only used for distanceBetween and courseTo
SFE_UBLOX_GNSS gnss;
#if has_tof
Async_SparkFun_TMF882X  tof_sensor;
tmf882x_msg_meas_results tof_sensor_results;
#endif
VL53L1X left_tof_distance_sensor_raw;
VL53L1X center_tof_distance_sensor_raw;
VL53L1X right_tof_distance_sensor_raw;

struct TofSensor{
  TofSensor(const char * name, VL53L1X * tof_sensor, int power_pin, uint8_t i2c_address) : sensor(tof_sensor), power_pin(power_pin), i2c_address(i2c_address) {
    strncpy(this->name, name, sizeof(this->name));
  }
  VL53L1X * sensor;
  int power_pin;
  uint8_t i2c_address;
  float distance  = NAN;
  bool running = false;
  rcl_publisher_t publisher;
  char name[20];
  uint32_t turn_off_ms = 0;

};
TofSensor left_tof_sensor("left",&left_tof_distance_sensor_raw, pin_left_tof_power, left_tof_i2c_address);
TofSensor center_tof_sensor = {"center",&center_tof_distance_sensor_raw, pin_center_tof_power, center_tof_i2c_address};
TofSensor right_tof_sensor = {"right",&right_tof_distance_sensor_raw, pin_right_tof_power, right_tof_i2c_address};

std::vector<TofSensor * > tof_sensors = {
  &left_tof_sensor,
  &center_tof_sensor,
  &right_tof_sensor};


uint32_t tof_distance_sensor_timing_budget_ms = 50;
float tof_distance = std::numeric_limits<float>::quiet_NaN();

QMC5883LCompass compass;
QuadratureEncoder left_encoder(pin_left_encoder_a, pin_left_encoder_b, meters_per_odometer_tick);
QuadratureEncoder right_encoder(pin_right_encoder_a, pin_right_encoder_b, meters_per_odometer_tick);

Speedometer left_speedometer;
Speedometer right_speedometer;

int rx_str = 0;
int rx_esc = 0;
int rx_aux = 0;
bool button_sc_pressed = false;
double v_bat = NAN;


RunStatistics gps_stats("gps");
RunStatistics log_stats("logf");
RunStatistics loop_stats("loop");
RunStatistics crsf_stats("crsf");
RunStatistics compass_stats("compass");
RunStatistics telemetry_stats("telemetry");
RunStatistics serial_read_stats("serial_read");
RunStatistics crsf_parse_stats("crsf_parse");
RunStatistics tof_stats("tof");
RunStatistics tof_distance_stats("tof_distance");

StuckChecker left_stuck_checker;
StuckChecker right_stuck_checker;

struct spad_mode_info {
  uint8_t id;
  uint8_t columns;
  uint8_t rows;
  uint8_t fov_degrees;
  uint8_t y_fov_degrees;
};

// based on https://look.ams-osram.com/m/52236c476132a095/original/TMF8820-21-28-Multizone-Time-of-Flight-Sensor.pdf
const spad_mode_info spad_mode_1 = {1, 3, 3, 33, 32}; // id, rows, columns, x_fov_degrees, y_fov_degrees
const spad_mode_info spad_mode_2 = {2, 3, 3, 33, 37};
const spad_mode_info spad_mode_3 = {3, 3, 3, 33, 45};
const spad_mode_info spad_mode_6 = {6, 3, 3, 41, 52};
const spad_mode_info spad_mode_11 = {11, 3, 3, 33, 32};
const spad_mode_info spad_mode_12 = {12, 3, 3, 33, 32};
const spad_mode_info spad_mode_7 = {7, 4, 4, 41, 52};
const spad_mode_info spad_mode_4 = {4, 4, 4, 33, 47};
const spad_mode_info spad_mode_5 = {5, 4, 4, 33, 47};
const spad_mode_info spad_mode_13 = {13, 4, 4, 33, 42};
const spad_mode_info spad_mode_10 = {10, 3, 6, 33, 60};




uint8_t tof_spad_rows;
uint8_t tof_spad_columns;

//////////////////////////////////
// Micro Ros

rcl_publisher_t log_publisher;
rcl_publisher_t battery_publisher;
sensor_msgs__msg__Range tof_distance_msg;
std_msgs__msg__String log_msg;
std_msgs__msg__Float32 battery_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      error_loop();                \
    }                              \
  }
#define RCSOFTCHECK(fn)            \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
    }                              \
  }

// logs to ros and serial
void log(std_msgs__msg__String &msg) {
  BlockTimer bt(log_stats);
  RCSOFTCHECK(rcl_publish(&log_publisher, &msg, NULL));
  Serial.write(msg.data.data, msg.data.size);
  Serial.write("\n");
}

void logf(const char *format, ...) {
  va_list args;
  va_start(args, format);
  log_msg.data.size = vsnprintf(log_msg.data.data, log_msg.data.capacity, format, args);
  va_end(args);

  log(log_msg);
}

void error_loop() {
}

void setup_micro_ros() {
  Serial.printf("setting up micro ros\n");
  IPAddress agent_ip(192, 168, 86, 66);
  size_t agent_port = 8888;

  char *ssid = const_cast<char *>(wifi_ssid);
  char *psk = const_cast<char *>(wifi_password);

  
  set_microros_wifi_transports(ssid, psk, agent_ip, agent_port);
  // checkto see if we can connect to the agent


  // delay(2000);
  // uint32_t timeout_ms = 1000;
  // rmw_uros_sync_session(timeout_ms);

  allocator = rcl_get_default_allocator();
  configTime(0, 0, "pool.ntp.org");

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_wifi_node", "", &support));

  // create publishers

  RCCHECK(rclc_publisher_init_best_effort(
      &log_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "/tank/log"));

  RCCHECK(rclc_publisher_init_best_effort(
      &battery_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/tank/battery"));
  
  for (auto tof : tof_sensors) {
    char topic[30];
    snprintf(topic, sizeof(topic), "/tank/%s_distance", tof->name);
    RCCHECK(rclc_publisher_init_best_effort(
        &tof->publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        topic));
  }

  Serial.printf("micro ros initialized\n");
}

//////////////////////////////////
// CRSF - include here so it can use logging, etc.

#include "Crsf.hpp"

// set up crsf serial to use pin_csrf_rx and pin_csrf_tx
Crsf crsf(crsf_serial);

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

void handle_rc_message(crsf_ns::RcData &rc_data) {
  if (rc_data.failsafe) {
    rx_str = 0;
    rx_esc = 0;
    rx_aux = 0;

  } else {
    rx_str = rc_data.channels[0];
    rx_esc = rc_data.channels[1];
    rx_aux = rc_data.channels[2];
    button_sc_pressed = crsf_ns::crsf_rc_channel_to_bool(rc_data.channels[6]);
  }
}

unsigned long last_loop_time_ms = 0;
unsigned long loop_time_ms = 0;

// returns true if loop time passes through n ms boundary
bool every_n_ms(unsigned long last_loop_ms, unsigned long loop_ms, unsigned long ms) {
  return (last_loop_ms % ms) + (loop_ms - last_loop_ms) >= ms;
}

// sets motor speeds to go to a lat/lon
// returns true if arrived
bool go_toward_lat_lon(lat_lon destination, float * meters_to_next_waypoint) {

  const float max_speed = 0.5;


  if (gnss.getGnssFixOk() == 0) {
    left_motor.go(0);
    right_motor.go(0);
    return false;
  }

  double gnss_lat = gnss.getLatitude(0) / 1E7;
  double gnss_lon = gnss.getLongitude(0) / 1E7;

  double distance_remaining = gps.distanceBetween(gnss_lat, gnss_lon, destination.lat, destination.lon);
  *meters_to_next_waypoint = distance_remaining;
  if (distance_remaining < 2.0) {
    left_motor.go(0);
    right_motor.go(0);
    return true;
  }

  float compass_mounting_angle_degrees = 0.0; // m
  // float compass_mounting_angle_degrees = 180.0; // s

  // subtract courseTo from 360 to get postive ccw
  double desired_bearing_degrees = 360. - gps.courseTo(gnss_lat, gnss_lon, destination.lat, destination.lon);
  double current_heading_degrees = compass.getAzimuth() + compass_mounting_angle_degrees; // chip is mounted 180 degrees off

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
    left_motor_speed = 0.7 * max_speed;
    right_motor_speed = max_speed;
    direction = "left";
  } else if (heading_error > 20) {
    // turn right
    left_motor_speed = max_speed;
    right_motor_speed = 0.7 * max_speed;
    direction = "left";
  } else {
    // go straight
    left_motor_speed = max_speed;
    right_motor_speed = max_speed;
    direction = "straight";
  }

  // printf("left_motor_speed: %0.4f right_motor_speed: %0.4f\n", left_motor_speed, right_motor_speed);
  left_motor.go(left_motor_speed);
  right_motor.go(right_motor_speed);

  if (every_n_ms(last_loop_time_ms, loop_time_ms, 100)) {
    logf("speed: %f, %f destination: %f %f direction: %s, distance_remaining: %0.4f, desired_bearing_degrees: %0.4f current_heading_degrees: %0.4f heading_error: %0.4f\n",
         left_motor_speed,
         right_motor_speed, 
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

void update_motor_speeds() {
  // stop on failsafe
  if (rx_esc == 0 || rx_str == 0) {
    right_motor.go(0);
    left_motor.go(0);
    return;
  }

  double speed = crsf_ns::crsf_rc_channel_to_float(rx_esc);
  double str_speed = crsf_ns::crsf_rc_channel_to_float(rx_str);

  // don't move if speed is too low
  if (abs(speed) < 0.05) {
    speed = 0.0;
    if (abs(str_speed) < 0.05) {
      str_speed = 0.0;
    }
  }

  if (abs(speed) + abs(str_speed) > 1.0) {
    double scale = 1.0 / (abs(speed) + abs(str_speed));
    speed *= scale;
    str_speed *= scale;
  }

  float right_speed = speed + str_speed;
  float left_speed = speed - str_speed;

  right_motor.go(right_speed, false);
  left_motor.go(left_speed, false);
}

void update_compass_calibration(int min_x, int max_x, int min_y, int max_y, int min_z, int max_z) {

  compass.setCalibration(min_x, max_x, min_y, max_y, min_z, max_z);
  logf("updated compass calibration to %d %d %d %d %d %d\n", min_x, max_x, min_y, max_y, min_z, max_z);
}
//////////////////////////////////
// Finite state machine
class HandMode : public Task {
 public:
  HandMode() {
    name = "hand";
  }

  // implement the execute method
  virtual void execute() override {
    update_motor_speeds();
  }

} hand_mode;

class CalibrateCompassMode : public Task {
 public:
  int min_x, max_x, min_y, max_y, min_z, max_z;

  CalibrateCompassMode() {
    name = "cal-comp";
  }

  virtual void begin() override {
    logf("calibrate compass mode\n");
    compass.clearCalibration();
    compass.read(); // reading forces the calibration to apply
    min_x = min_y = min_z = std::numeric_limits<int>::max();
    max_x = max_y = max_z = std::numeric_limits<int>::min();
  }

  // implement the execute method
  virtual void execute() override {
    update_motor_speeds();

    int x = compass.getX();
    int y = compass.getY();
    int z = compass.getZ();

    logf("compass_xyz: %d %d %d\n", x, y, z);
    if (x < min_x) min_x = x;
    if (x > max_x) max_x = x;
    if (y < min_y) min_y = y;
    if (y > max_y) max_y = y;
    if (z < min_z) min_z = z;
    if (z > max_z) max_z = z;
  }

  virtual void end() override {
    update_compass_calibration(min_x, max_x, min_y, max_y, min_z, max_z);

    logf("calibrated compass: %d %d %d %d %d %d\n", min_x, max_x, min_y, max_y, min_z, max_z);

    // write to flash memory
    {
      File file = SPIFFS.open(compass_calibration_file_path, FILE_WRITE);
      if (!file) {
        logf("failed to open file for writing\n");
      } else {
        file.printf("%d,%d,%d,%d,%d,%d\n", min_x, max_x, min_y, max_y, min_z, max_z);
        file.close();
      }
    }
  }

} calibrate_compass_mode;

class OffMode : public Task {
 public:
  OffMode() {
    name = "off";
  }

  // implement the execute method
  virtual void execute() override {
    left_motor.go(0);
    right_motor.go(0);
  }

} off_mode;

class AutoMode : public Task {
 public:
  bool unsticking = false;
  float meters_to_next_waypoint = 0;
  float unstick_left_start_meters;
  float unstick_right_start_meters;

  AutoMode() {
    name = "auto";
  }
  int step = 0;

  void begin() override {
    done = false;
    Serial.write("auto mode begin\n");
    step = 0;
  }

  void get_display_string(char *buffer, int buffer_size) override {
    snprintf(buffer, buffer_size, "wp%d %3.1f", step, meters_to_next_waypoint);
  }

  void execute() override {

    if (unsticking) {
      const float minimum_unstick_distance = 0.5;
      float left_unstick_distance = abs(left_encoder.get_meters() - unstick_left_start_meters);
      float right_unstick_distance = abs(right_encoder.get_meters() - unstick_right_start_meters);
      logf("unsticking left: %0.4f right: %0.4f\n", left_unstick_distance, right_unstick_distance);
      // considered unstuck when both motors have gone minimum_unstick_distance absolute
      if ((left_unstick_distance > minimum_unstick_distance) && (right_unstick_distance > minimum_unstick_distance)) {
        logf("unstuck\n");
        unsticking = false;
      } else {
        left_motor.go(-1);
        right_motor.go(-1);
        return;
      }
    }

    
    // if you are blocked, go backward
    if (left_stuck_checker.is_stuck() || right_stuck_checker.is_stuck()) {
      unstick_left_start_meters = left_encoder.get_meters();
      unstick_right_start_meters = right_encoder.get_meters();
      left_motor.go(0);
      right_motor.go(0);
      unsticking = true;
      return;
    }
    bool arrived = go_toward_lat_lon(route_waypoints[step], &meters_to_next_waypoint);
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
    &calibrate_compass_mode,
};

std::vector<Fsm::Edge> edges = {
    // from, event, to
    Fsm::Edge("auto", "done", "hand"),
    Fsm::Edge("hand", "sc-click", "cal-comp"),
    Fsm::Edge("cal-comp", "sc-click", "hand"),
    Fsm::Edge("cal-comp", "done", "hand"),
    Fsm::Edge("*", "auto", "auto"),
    Fsm::Edge("*", "failsafe", "failsafe"),
    Fsm::Edge("auto", "hand", "hand"),
    Fsm::Edge("off", "hand", "hand"),
    Fsm::Edge("*", "off", "off"),
};

Fsm fsm(tasks, edges);

void ros_thread(void *arg) {
  while (true) {
    RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL));
    delay(1000);
  }
}


bool load_compass_calibration_from_spiffs() {
  if (!SPIFFS.exists(compass_calibration_file_path)) {
    logf("No calibration file found");
    return false;
  }

  File file = SPIFFS.open(compass_calibration_file_path, FILE_READ);
  if (!file) {
    logf("Failed to open calibration file");
    return false;
  }

  String content = file.readStringUntil('\n');
  file.close();

  // Parse comma-separated values
  int comma1 = content.indexOf(',');
  int comma2 = content.indexOf(',', comma1 + 1);
  int comma3 = content.indexOf(',', comma2 + 1);
  int comma4 = content.indexOf(',', comma3 + 1);
  int comma5 = content.indexOf(',', comma4 + 1);

  if (comma1 == -1 || comma2 == -1 || comma3 == -1 || comma4 == -1 || comma5 == -1) {
    logf("Invalid calibration format");
    return false;
  }

  int min_x = content.substring(0, comma1).toInt();
  int max_x = content.substring(comma1 + 1, comma2).toInt();
  int min_y = content.substring(comma2 + 1, comma3).toInt();
  int max_y = content.substring(comma3 + 1, comma4).toInt();
  int min_z = content.substring(comma4 + 1, comma5).toInt();
  int max_z = content.substring(comma5 + 1).toInt();

  update_compass_calibration(min_x, max_x, min_y, max_y, min_z, max_z);

  return true;
}

void print_tof_results(struct tmf882x_msg_meas_results *myResults)
{

    // print out results
    Serial.println("Measurement:");
    Serial.print("Result Number: "); Serial.print(myResults->result_num);
    Serial.print(" Number of Results: "); Serial.println(myResults->num_results);       

    for(int i = 0; i < myResults->num_results; ++i) 
    {
      auto & result = myResults->results[i];

        uint32_t pseudo_index = result.channel - 1 + 8 * (result.sub_capture);
        Serial.print("pseudo_idx: "); Serial.print(pseudo_index);
        Serial.print("    conf: "); Serial.print(result.confidence);
        Serial.print(" distance mm: "); Serial.print(result.distance_mm);
        Serial.print(" channel: "); Serial.print(result.channel);
        Serial.print(" sub_capture: "); Serial.print(result.sub_capture);
        Serial.print(" target_idx: "); Serial.print(result.ch_target_idx);

        Serial.println();

    }
    Serial.print(" photon: "); Serial.print(myResults->photon_count);   
    Serial.print(" ref photon: "); Serial.print(myResults->ref_photon_count);
    Serial.print(" ALS: "); Serial.println(myResults->ambient_light); Serial.println();

}

void tof_measurement_callback(struct tmf882x_msg_meas_results *results) {
  BlockTimer bt(tof_stats);

  static uint32_t callback_count = 0;
  callback_count++;

  const int max_channel_count = 32;
  uint32_t channels[max_channel_count]; // note, they send 1 based channels, so element 0 is unused
  uint32_t confidences[max_channel_count];
  memset(channels, 0, sizeof(channels));
  memset(confidences, 0, sizeof(confidences));
  uint32_t channel_min = std::numeric_limits<uint32_t>::max();
  uint32_t channel_max = std::numeric_limits<uint32_t>::min();

  uint32_t num_spads = tof_spad_rows * tof_spad_columns;

  for(uint32_t i = 0; i < results->num_results; ++i) {
    auto & result = results->results[i];
    channel_max = std::max(channel_max, result.channel);
    channel_min = std::min(channel_min, result.channel);
    uint32_t idx = result.channel - 1 + 8 * result.sub_capture;
    if(result.ch_target_idx == 0) {
      channels[idx] = result.distance_mm;
      confidences[idx] = result.confidence;
    }
  }

  Serial.printf("callback_count: %d max confidence: %d\n", callback_count, *std::max_element(confidences, confidences + max_channel_count));

  for(uint32_t i = 0; i < num_spads; ++i) {
    // Serial.printf("%4d ",channels[i]);
    auto d = channels[i];
    auto c = " ";
    if (d > 0 && confidences[i] >= 255) {
      if (d < 100) {
        c = "*";
      } else if (d < 300) {
        c = "o";
      } else if (d < 1000) {
        c = ".";
      } else {
        c = " ";
      }
    }
    Serial.print(c);



    // Serial.printf("%d: %d: %s: conf:%d",i,d, c, confidences[i]);
    if (i % tof_spad_columns == tof_spad_columns - 1) {
      Serial.println();
    }
  }

  // print all fields of the results
  // print_tof_results(results);
//  Serial.println();
}

void start_tof_distance_sensor(TofSensor & tof) {
  pinMode(tof.power_pin, OUTPUT);
  digitalWrite(tof.power_pin, HIGH);
  tof.sensor->setBus(&Wire1);
  bool use_2v8 = false;
  delay(50);
  while (!tof.sensor->init(use_2v8)) {
    Serial.println("Failed to detect and initialize VL53L1X distance sensor!");
    delay(50);
  }
  Serial.printf("VL53L1X %s sensor initialized\n", tof.name);
  tof.sensor->setAddress(tof.i2c_address);

  /*
  The VL53L1X has three distance modes (DM): short, medium, and long.
  Long distance mode allows the longest possible ranging distance of 4 m to be reached. However, this maximum
  ranging distance is impacted by ambient light.
  Short distance mode is more immune to ambient light, but its maximum ranging distance is typically limited to 1.3m.
  */
  tof.sensor->setDistanceMode(VL53L1X::Short);

  /*
  20 ms is the minimum timing budget and can be used only in short distance mode.
  33 ms is the minimum timing budget which can work for all distance modes.
  140 ms is the timing budget which allows the maximum distance of 4 m (in the dark on a white chart) to be reached with long distance mode
  */
  tof.sensor->setMeasurementTimingBudget(tof_distance_sensor_timing_budget_ms * 1000);
  tof.sensor->setTimeout(2);  // timeout ms for synchronous measurements
  tof.sensor->startContinuous(2 * tof_distance_sensor_timing_budget_ms);
  tof.running = true;
}

void turn_off_tof_distance_sensor(TofSensor & tof) {
  digitalWrite(tof.power_pin, LOW);
  tof.running = false;
  tof.turn_off_ms = millis();
}

//////////////////////////////////
// Main setup and loop

void setup() {
  pinMode(pin_built_in_led, OUTPUT);
  digitalWrite(pin_built_in_led, HIGH);
  Serial.begin(115200);

  SPIFFS.begin();

  // preallocate log message for all logging
  log_msg.data.data = (char *)malloc(200);
  log_msg.data.capacity = 200;
  log_msg.data.size = 0;
  setup_micro_ros();

  tof_distance_msg.header.frame_id.data = const_cast<char *>("tof");
  tof_distance_msg.header.frame_id.size = 3;
  tof_distance_msg.header.frame_id.capacity = 3;
  tof_distance_msg.min_range = 0.03;
  tof_distance_msg.max_range = 1.0;
  tof_distance_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
  tof_distance_msg.field_of_view = 27 * M_PI / 180; // 27 degrees

  Wire.setPins(pin_compass_sda, pin_compass_scl);
  Wire.begin();
  Wire.setTimeOut(5); // ms

  Wire1.setPins(pin_tof_sda, pin_tof_scl);
  Wire1.begin();
  Wire1.setTimeOut(5); // ms

  left_speedometer.meters_per_tick = meters_per_odometer_tick;
  right_speedometer.meters_per_tick = meters_per_odometer_tick;





  fsm.begin();

  Serial.write("tank-train\n");
  crsf_serial.setRxBufferSize(4096);
  crsf_serial.begin(420000, SERIAL_8N1, pin_crsf_rx, pin_crsf_tx);
  gps_serial.setRxBufferSize(4096);
  gps_serial.begin(115200, SERIAL_8N1, pin_gps_rx, pin_gps_tx);

  //delay(10000); /// just wait for serial monitor to open

  gnss.enableDebugging(Serial, true);
  for(int i = 0; i < 2; ++i) {
    //gnss.factoryReset();
    delay(1000);
    if (gnss.begin(gps_serial)) {
      Serial.printf("GPS started\n");
      break;
    }
    Serial.printf("GPS failed to start, retrying\n");
    delay(1000);
  } 


  compass.init();
  if (!load_compass_calibration_from_spiffs()) {
    compass.setCalibration(-950, 675, -1510, 47, 0, 850);
  }
  // see https://www.magnetic-declination.com/
  compass.setMagneticDeclination(11, 24);

  start_tof_distance_sensor(left_tof_sensor);
  start_tof_distance_sensor(right_tof_sensor);



 //tof_distance_sensor.readSingle(false); // read single measurement, false means async



  #if has_tof
  tof_sensor.setSampleDelay(1);
  if(!tof_sensor.begin(Wire1))
  {
    while(1) {
      Serial.println("Error - The TMF882X failed to initialize - is the board connected?");
      delay(1000);
    }
  } else {
      Serial.println("TMF882X started.");
  }
  #endif

  // see https://look.ams-osram.com/m/52236c476132a095/original/TMF8820-21-28-Multizone-Time-of-Flight-Sensor.pdf
  // page 23
  // and https://github.com/sparkfun/SparkFun_Qwiic_TMF882X_Arduino_Library/blob/main/docs/api_setup.md
  // tmf882x_mode_app_config tofConfig;
  // tof_sensor.getTMF882XConfig(tofConfig);
  // tofConfig.spad_map_id = 6;
  // if (!tof_sensor.setTMF882XConfig(tofConfig)){
  //   while(true) {
  //     Serial.println("Error - The TMF882X failed to set config");
  //     delay(1000);
  //   }
  // }
  #if 0
  const spad_mode_info &spad_mode = spad_mode_6;
  while (!tof_sensor.setCurrentSPADMap(spad_mode.id)) {
    Serial.println("Error - The TMF882X failed to set SPAD map");
    delay(1000);
  }
  tof_spad_rows = spad_mode.rows;
  tof_spad_columns = spad_mode.columns;

  tof_sensor.setMeasurementHandler(tof_measurement_callback);
  #endif

  // quadrature encoders

  attachInterrupt(digitalPinToInterrupt(pin_left_encoder_a), left_a_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_left_encoder_b), left_b_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_right_encoder_a), right_a_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_right_encoder_b), right_b_changed, CHANGE);

  // crsf
  crsf.begin();
  crsf.update();

  crsf.set_rc_callback(handle_rc_message);

  left_motor.init(pin_left_fwd, pin_left_rev);
  right_motor.init(pin_right_fwd, pin_right_rev);
  pinMode(pin_test, OUTPUT);
  pinMode(pin_built_in_led, OUTPUT);
  pinMode(pin_battery_voltage, INPUT);

  // create a thread for ros stuff
  xTaskCreatePinnedToCore(
      ros_thread,
      "ros_thread",
      8192,
      NULL,
      1,
      NULL,
      1);

  // reset all serial data
  crsf_serial.flush();
  gps_serial.flush();
  /*
  while(crsf_serial.available()) {
    crsf_serial.read();
  }
  while(gps_serial.available()) {
    gps_serial.read();
  }
    */
  #if has_tof
  tof_sensor.async_startMeasuring();
  logf("%s", "*********************** setup complete ***********************");
  #endif
  digitalWrite(pin_built_in_led, 0);
}


class HangChecker {
 public:
  const char *name;
  unsigned long timeout_ms;
  unsigned long start_ms;
  HangChecker(const char *name, unsigned long timeout_ms = 100) {
    this->name = name;
    this->timeout_ms = timeout_ms;
    this->start_ms = millis();
  }

  ~HangChecker() {
    unsigned long now = millis();
    int elapsed = now - start_ms;
    if (elapsed > timeout_ms) {
      logf("HANG: %s", name);
      // left_motor.go(0);
      // right_motor.go(0);
      // digitalWrite(pin_built_in_led, HIGH);
      // while (true) {
      //   Serial.printf("%s hanged for %d ms\n", name, elapsed);

      //   delay(1000);
      // }
    }
  }
};

uint8_t estimate_lipo_battery_percent_from_voltage(float v) {
  // https://www.rchelicopterfun.com/rc-lipo-batteries.html
  // 4.2v = 100%
  // 3.7v = 50%
  // 3.3v = 20%
  // 3.0v = 0%

  // guess the number of cells
  int num_cells = v < 4.4  ? 1 : (v < 9 ? 2 : 3);

  float v_per_cell = v / num_cells;


  if (v_per_cell > 4.1) {
    return 100;
  } else if (v_per_cell < 3.5) {
    return 0;
  } else {
    return 100 * (v_per_cell - 3.5) / (4.1 - 3.5);
  }

}

void loop() {
  BlockTimer bt(loop_stats);
  last_loop_time_ms = loop_time_ms;
  loop_time_ms = millis();

  bool every_1_ms = every_n_ms(last_loop_time_ms, loop_time_ms, 1);
  bool every_10_ms = every_n_ms(last_loop_time_ms, loop_time_ms, 10);
  bool every_100_ms = every_n_ms(last_loop_time_ms, loop_time_ms, 100);
  bool every_200_ms = every_n_ms(last_loop_time_ms, loop_time_ms, 200);
  bool every_1000_ms = every_n_ms(last_loop_time_ms, loop_time_ms, 1000);

  #if has_tof
  if(every_100_ms) {
    BlockTimer bt(tof_stats);
    tof_sensor.async_updateMeasuring();
  }
  #endif

  if (every_10_ms) {
    left_speedometer.update_from_sensor(micros(), left_encoder.odometer_a, left_encoder.last_odometer_a_us, left_encoder.odometer_b, left_encoder.last_odometer_b_us);
    right_speedometer.update_from_sensor(micros(), right_encoder.odometer_a, right_encoder.last_odometer_a_us, right_encoder.odometer_b, right_encoder.last_odometer_b_us);
    // logf("v_bat: %3.2f (%%,v): left (%0.2f, %0.2f) right (%0.2f, %0.2f)", 
    //   v_bat,
    //   left_motor.get_setpoint(),
    //   left_speedometer.get_velocity(),
    //   right_motor.get_setpoint(), 
    //   right_speedometer.get_velocity()
    // );
  }

  // tof distance - you need to wait 3ms longer than the 
  if (every_n_ms(last_loop_time_ms, loop_time_ms, 2 * tof_distance_sensor_timing_budget_ms + 10)) {
    for (auto tof : tof_sensors) {
      // extra block to limit scope of BlockTimer
      {
        if(!tof->running) {
          // wait one second to turn back on
          if (millis() - tof->turn_off_ms > 1000) {
            start_tof_distance_sensor(*tof);
          }
          continue;
        }
        auto sensor = tof->sensor;
        BlockTimer bt(tof_distance_stats);

        auto start = millis();
        if (sensor->dataReady()) {
          sensor->read(false); // read pending measurement
          auto elapsed_ms = millis() - start;
          if (false && elapsed_ms > 60) {
            logf("tof read took %d ms, resetting sensor", elapsed_ms);
            turn_off_tof_distance_sensor(*tof);
          }
          if (sensor->ranging_data.range_status == VL53L1X::RangeValid) {
            tof->distance = sensor->ranging_data.range_mm / 1000.0;
  
          } else {
            tof->distance = std::numeric_limits<float>::quiet_NaN();
          }

          struct timespec ts;
          clock_gettime(CLOCK_REALTIME, &ts);
          tof_distance_msg.header.stamp.sec = ts.tv_sec;
          tof_distance_msg.header.stamp.nanosec =ts.tv_nsec;
      
          tof_distance_msg.range = tof->distance;    
          RCSOFTCHECK(rcl_publish(&(tof->publisher), &tof_distance_msg, NULL));
      
        }        
      }
    }
  }

  if (every_10_ms) {
    BlockTimer bt(gps_stats);
    HangChecker hc("gps");
    gnss.checkUblox();
  }

  if (every_1000_ms) {
    HangChecker hc("encoders");
    // logf("Encoders left: %fm (%d,%d) right: %fm (%d,%d) ms: %d, %d",
    //     left_encoder.get_meters(),
    //     left_encoder.odometer_a,
    //     left_encoder.odometer_b,
    //     right_encoder.get_meters(),
    //     right_encoder.odometer_a,
    //     right_encoder.odometer_b,
    //     left_encoder.odometer_ab_us,
    //     right_encoder.odometer_ab_us);

    // logf("Gps Checksums passed: %d failed: %d chars: %d sentences: %d",
    //       gps.passedChecksum(),
    //       gps.failedChecksum(),
    //       gps.charsProcessed(),
    //       gps.sentencesWithFix()
    //     );

    // logf("meters traveled: %0.4f, %0.4f",
    //      left_encoder.odometer_a * meters_per_odometer_tick,
    //      right_encoder.odometer_a * meters_per_odometer_tick);


  }
  
  if (every_n_ms(last_loop_time_ms, loop_time_ms, 10000)) {
    for (auto stats : {gps_stats, log_stats, loop_stats, crsf_stats, compass_stats, telemetry_stats, serial_read_stats, crsf_parse_stats, tof_distance_stats}) {
      stats.to_log_msg(&log_msg);
      log(log_msg);
    }
  }

  if (isnan(v_bat) || every_1000_ms) {
    HangChecker hc("battery");
    analogReadResolution(12);
    int sum = 0;
    const int sample_count = 100;
    for (int i = 0; i < sample_count; i++) {
      sum += analogRead(pin_battery_voltage);
    }


    // const float v_bat_scale = 0.003714; // white-crash s
    const float v_bat_scale = 0.0077578; // white-crash m
  

    
    v_bat = v_bat_scale * sum  / sample_count;

    battery_msg.data = v_bat;
  }

  if (every_100_ms) {
    HangChecker hc("fsm");
    fsm.execute();
  }

  // read mode from rx_aux channel
  enum aux_mode_t { aux_mode_failsafe,
                    aux_mode_hand,
                    aux_mode_off,
                    aux_mode_auto };

  // if a motor is on for more than 1 second with no progress, report it as stuck
  if (every_100_ms) {
    left_stuck_checker.update(left_motor.get_setpoint(), left_encoder.odometer_a);
    right_stuck_checker.update(right_motor.get_setpoint(), right_encoder.odometer_a);

    if (left_stuck_checker.is_stuck()) {
      logf("Left motor is stuck", left_motor.get_setpoint(), left_encoder.odometer_a);
    }

    if (right_stuck_checker.is_stuck()) {
      logf("Right motor is stuck", right_motor.get_setpoint(), right_encoder.odometer_a);
    }
  }

  if (every_10_ms) {
    HangChecker hc("mode event");
    static aux_mode_t last_aux_mode = aux_mode_failsafe;
    static bool last_button_sc_pressed = false;

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

    // sc event
    if (button_sc_pressed && !last_button_sc_pressed) {
      fsm.set_event("sc-click");
      logf("sc-click");
    }
    last_aux_mode = aux_mode;
    last_button_sc_pressed = button_sc_pressed;
  }

  if (every_100_ms) {
    BlockTimer bt(compass_stats);
    HangChecker hc("compass");
    compass.read();

    // update magnetometer limits
    int x = compass.getX();
    int y = compass.getY();
    int z = compass.getZ();

    // Serial.printf("compass: %d [%d, %d, %d]\n",
    //   compass.getAzimuth(),
    //   compass.getX(), compass.getY(), compass.getZ()    );
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
  if (every_10_ms) {
    BlockTimer bt(crsf_stats);
    HangChecker hc("crsf");
    // HangChecker hc("crsf");
    crsf.update();  // update as fast as possible, will call callbacks when data is ready
  }
  if (every_n_ms(last_loop_time_ms, loop_time_ms, 200)) {
    BlockTimer bt(telemetry_stats);
    HangChecker hc("telemetry");

    uint8_t percent = estimate_lipo_battery_percent_from_voltage(v_bat);

    crsf.send_battery(v_bat, 0, 0, percent);
    char display_string[16];
    fsm.current_task->get_display_string(display_string, 16);
    crsf.send_flight_mode(display_string);
    crsf.send_attitude(0, 0, compass.getAzimuth());

    //0=no fix, 1=dead reckoning, 2=2D, 3=3D, 4=GNSS, 5=Time fix

    if (gnss.getGnssFixOk(0)) {
      crsf.send_gps(
          gnss.getLatitude(0),
          gnss.getLongitude(0),
          gnss.getGroundSpeed(0) * 1E-3,
          gnss.getHeading(0) * 1E-5,
          gnss.getAltitude(0) * 1E-3,
          gnss.getSIV(0));
    } else {
      crsf.send_gps(0, 0, 0, 0, 0, 0);
    }


    // if (gps.location.isValid()) {
    //   crsf.send_gps(
    //       gps.location.lat(),
    //       gps.location.lng(),
    //       gps.altitude.meters(),
    //       gps.speed.kmph(),
    //       gps.course.deg(),
    //       gps.satellites.value());
    // } else {
    //   crsf.send_gps(0, 0, 0, 0, 0, 0);
    // }
  }

  if (every_n_ms(last_loop_time_ms, loop_time_ms, 100)) {
    // Serial.printf("mode: %s str: %d esc: %d aux: %d\n", fsm.current_task->name, rx_str, rx_esc, rx_aux);
  }

  digitalWrite(pin_test, !digitalRead(pin_test));
  delay(1);
}
