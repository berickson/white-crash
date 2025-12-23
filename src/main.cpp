#include <Arduino.h>
#include <QMC5883LCompass.h>
#include <VL53L1X.h>
#include <SPIFFS.h>
#include <Wire.h>

#include <vector>
#include <algorithm>

#include "Fsm.h"
#include "TinyGPS++.h"
#include "drv8833.h"
#include "quadrature_encoder.h"
#include "speedometer.h"

#include "driver/uart.h" // to clear the serial buffer

// micro ros
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rcl_interfaces/msg/log.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <sensor_msgs/msg/range.h>

#include <white_crash_msgs/msg/update.h>


#include "RunStatistics.h"
#include "StuckChecker.h"
#include "secrets/wifi_login.h"

class Severity {
  public: enum SeverityLevel {
    DEBUG = 10,
    INFO = 20,
    WARN = 30,
    ERROR = 40,
    FATAL = 50
  };
};

#include <SparkFun_u-blox_GNSS_Arduino_Library.h> // https://github.com/sparkfun/SparkFun_Ublox_Arduino_Library

#include "ak8975.h"

// Feature enable/disable
const bool use_gnss = true; // sparkfun ublox
const bool use_gps = false;  // tinygps
const bool enable_stats_logging = true;

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


// Live ESP32 Mini Kit 32
// see https://doc.riot-os.org/group__boards__esp32__mh-et-live-minikit.html

// pins broken out on the board

/*
const int pin_built_in_led = 2; // ok
const int pin_clk_dont_use = 6;
const int pin_cmd_dont_use = 11;
const int pin_gpio_0_dont_use = 0;
// const int pin_gpio_2 = 2;
const int pin_gpio_4 = 4;
const int pin_gpio_5 = 5;
// const int pin_gpio_12 = 12;
const int pin_gpio_16 = 16;
const int pin_gpio_17 = 17;
const int pin_gpio_18 = 18;
const int pin_gpio_19 = 19;
const int pin_gpio_21 = 21;
const int pin_gpio_22 = 22;
const int pin_gpio_23 = 23;
const int pin_gpio_25 = 25;
const int pin_gpio_26 = 26;
const int pin_gpio_27 = 27;
const int pin_gpio_32 = 32;
const int pin_gpio_33 = 33;
const int pin_gpio_34_input_only = 34;
const int pin_gpio_35_input_only = 35;
const int pin_rx_dont_use = 3; // used for USB serial
const int pin_sd0_dont_use = 7;  // reboots the board if used
const int pin_sd1_dont_use = 8;  
const int pin_sd2_dont_use = 9;  // reboots the board if used
const int pin_sd3_dont_use = 10; // probably bad to use, could interfere with boot, could be used as output after boot if careful
const int pin_svn_input_only = 39; // GPIO 39 (SVN) for analog input
const int pin_svp_input_only = 36; // GPIO 36 (SVP) for analog input
const int pin_tck = 13; // ok (used for JTAG)
const int pin_tdi_output_only = 12; // ok (used for JTAG)
const int pin_tdo = 15; // ok (used for JTAG)
const int pin_tms = 14; // ok (used for JTAG) // connecting to GPIO 14 (TMS) is not recommended as it can interfere with the ESP32's boot process.
const int pin_tx_dont_use = 1; // used for USB serial

// pins mapping / connections

const int pin_sda = pin_gpio_4; // yellow
const int pin_scl = pin_gpio_16; // blue

const int pin_crsf_rx = pin_gpio_17; // green
const int pin_crsf_tx = pin_tdi_output_only; // white


const int pin_gps_tx = pin_gpio_25; // green
const int pin_gps_rx = pin_gpio_22; // yellow

const int pin_battery_voltage = pin_svp_input_only; // blue


const int pin_right_encoder_a = pin_gpio_35_input_only; // ;  // green
const int pin_right_encoder_b = pin_gpio_27;  // yellow

const int pin_left_encoder_b = pin_gpio_34_input_only; // green
const int pin_left_encoder_a = pin_gpio_33; // yellow

const int pin_left_rev = pin_gpio_21;  // blue
const int pin_left_fwd = pin_gpio_32;  // green
const int pin_right_rev = pin_gpio_26;
const int pin_right_fwd = pin_gpio_18;

 
// S2 Mini


const int pin_left_tof_power = pin_gpio_19; 
const int pin_center_tof_power = pin_gpio_23;
const int pin_right_tof_power = pin_gpio_5;

*/

// LOLIN pins
// S3 Mini pin mapping - ACTUAL WIRING
// I2C - using default S3 I2C pins
const int pin_sda = 34;   // I2C SDA (default for S3, was GPIO 4 on old board)
const int pin_scl = 35;   // I2C SCL (default for S3, was GPIO 16 on old board)

// CRSF (radio control)
const int pin_crsf_rx = 38;  // CRSF RX (was GPIO 17)
const int pin_crsf_tx = 36;  // CRSF TX (was TDI)

// GPS
const int pin_gps_tx = 9;   // GPS TX (was GPIO 25)
const int pin_gps_rx = 10;  // GPS RX (was GPIO 22)

// Battery voltage (ADC)
const int pin_battery_voltage = 1;  // ADC input (was SVP/GPIO 36)

// Encoders
const int pin_right_encoder_a = 2;   // Right encoder A (was GPIO 35)
const int pin_right_encoder_b = 13;  // Right encoder B (was GPIO 27)
const int pin_left_encoder_a = 4;    // Left encoder A (was GPIO 33)
const int pin_left_encoder_b = 12;   // Left encoder B (was GPIO 34)

// Motor control
const int pin_left_fwd = 33;  // Left motor forward (was GPIO 32)
const int pin_left_rev = 37;  // Left motor reverse (was GPIO 21)
const int pin_right_fwd = 17;  // Right motor forward (was GPIO 18)
const int pin_right_rev = 16;  // Right motor reverse (was GPIO 26)

// ToF power control
const int pin_left_tof_power = 6;    // Left ToF power (was GPIO 19)
const int pin_center_tof_power = 7;  // Center ToF power (was GPIO 23)
const int pin_right_tof_power = 8;   // Right ToF power (was GPIO 5)






//////////////////////////////////
// i2c addresses
const uint8_t left_tof_i2c_address = 0x14;
const uint8_t center_tof_i2c_address = 0x15;
const uint8_t right_tof_i2c_address = 0x16;

//////////////////////////////////
// Globals

DRV8833 left_motor;
DRV8833 right_motor; 
HardwareSerial crsf_serial(1);
HardwareSerial gnss_serial(2);
TinyGPSPlus gps;  // currently only used for distanceBetween and courseTo
SFE_UBLOX_GNSS gnss;
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
TofSensor tof_left("left",&left_tof_distance_sensor_raw, pin_left_tof_power, left_tof_i2c_address);
TofSensor tof_center = {"center",&center_tof_distance_sensor_raw, pin_center_tof_power, center_tof_i2c_address};
TofSensor tof_right = {"right",&right_tof_distance_sensor_raw, pin_right_tof_power, right_tof_i2c_address};

std::vector<TofSensor * > tof_sensors = {
  &tof_left,
  &tof_center,
  &tof_right};


/*
20 ms is the minimum timing budget and can be used only in short distance mode.
33 ms is the minimum timing budget which can work for all distance modes.
140 ms is the timing budget which allows the maximum distance of 4 m (in the dark on a white chart) to be reached with long distance mode
*/
uint32_t tof_timing_budget_ms = 33;

AK8975Compass compass(Wire, 0x0E);
QuadratureEncoder left_encoder(pin_left_encoder_a, pin_left_encoder_b, meters_per_odometer_tick);
QuadratureEncoder right_encoder(pin_right_encoder_a, pin_right_encoder_b, meters_per_odometer_tick);

Speedometer left_speedometer;
Speedometer right_speedometer;

int rx_str = 0;
int rx_esc = 0;
int rx_aux = 0;
bool button_sc_pressed = false;
bool toggle_a_enabled = false;
bool toggle_b_enabled = false;
float p1_knob_percent = NAN;
float p2_knob_percent = NAN;
double v_bat = NAN;

bool ros_ready = false;

char temporary_display_string[16] = {0};
unsigned long temporary_display_string_set_ms = 0;
const unsigned long temporary_display_string_timeout_ms = 1000;

void set_temporary_display_string(const char * str) {
  strncpy(temporary_display_string, str, sizeof(temporary_display_string));
  temporary_display_string_set_ms = millis();
}


// virtual vbat floor and ceiling give the maximum range to set the voltage
// from the remote controller
const float virtual_vbat_ceiling = 12.6;
const float virtual_vbat_floor = 2.0;

// virtual_vbat makes the car act like
// it has a battery with a lower voltage
// set by the controller
float virtual_vbat = virtual_vbat_ceiling;

bool avoid_collisions = false;



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



//////////////////////////////////
// Micro Ros

rcl_publisher_t log_publisher;
rcl_publisher_t rosout_publisher;
rcl_publisher_t battery_publisher;
rcl_publisher_t update_publisher;
rcl_publisher_t nav_sat_fix_publisher;

sensor_msgs__msg__Range tof_distance_msg;
std_msgs__msg__String log_msg;
std_msgs__msg__Float32 battery_msg;
rcl_interfaces__msg__Log rosout_msg;
white_crash_msgs__msg__Update update_msg;
sensor_msgs__msg__NavSatFix nav_sat_fix_msg;

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn)                \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
      Serial.printf("ROS Function Call Failed %s\n", #fn);                \
    }                              \
  }
#define RCSOFTCHECK(fn)            \
  {                                \
    rcl_ret_t temp_rc = fn;        \
    if ((temp_rc != RCL_RET_OK)) { \
    }                              \
  }


void set_stamp(builtin_interfaces__msg__Time & stamp) {
  struct timespec ts;
  clock_gettime(CLOCK_REALTIME, &ts);
  stamp.sec = ts.tv_sec;
  stamp.nanosec = ts.tv_nsec;
}
  
// logs to ros and serial
void log(std_msgs__msg__String &msg) {
  BlockTimer bt(log_stats);
  if (ros_ready) {
    RCSOFTCHECK(rcl_publish(&log_publisher, &msg, NULL));
  }
  Serial.write(msg.data.data, msg.data.size);
  Serial.write("\n");
}

void log_rosout(rcl_interfaces__msg__Log &msg) {
  BlockTimer bt(log_stats);
  if (ros_ready) {
    RCSOFTCHECK(rcl_publish(&rosout_publisher, &msg, NULL));
  }
}


void logf(Severity::SeverityLevel severity, const char *format, ...) {
  va_list args;
  va_start(args, format);
  log_msg.data.size = vsnprintf(log_msg.data.data, log_msg.data.capacity, format, args);
  rosout_msg.msg.size = vsnprintf(rosout_msg.msg.data, rosout_msg.msg.capacity, format, args);
  
  va_end(args);

  rosout_msg.level = severity;
  log(log_msg);
  log_rosout(rosout_msg);
}

void logf(const char *format, ...) {
  va_list args;
  va_start(args, format);
  log_msg.data.size = vsnprintf(log_msg.data.data, log_msg.data.capacity, format, args);
  rosout_msg.msg.size = vsnprintf(rosout_msg.msg.data, rosout_msg.msg.capacity, format, args);

  // set the timestamp
  set_stamp(rosout_msg.stamp);

  va_end(args);

  rosout_msg.level = Severity::INFO;
  log(log_msg);
  log_rosout(rosout_msg);
}

void error_loop() {
  while (true) {
    Serial.printf("error in micro_ros, error_loop() entered\n");
    delay(1000);
  }
}

void create_ros_node_and_publishers() { 
  // create node
  const char *node_name = "white_crash";
  RCCHECK(rclc_node_init_default(&node, node_name, "", &support));
  Serial.printf("Created ROS node %s\n", node_name);

  // create publishers

  RCCHECK(rclc_publisher_init_best_effort(
      &log_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "/white_crash/log"));
  
  RCCHECK(rclc_publisher_init_best_effort(
      &rosout_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(rcl_interfaces, msg, Log),
      "/rosout_best_effort"));

  RCCHECK(rclc_publisher_init_best_effort(
      &battery_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
      "/white_crash/battery"));
  
  RCCHECK(rclc_publisher_init_best_effort(
      &update_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(white_crash_msgs, msg, Update),
      "/white_crash/update"));
  
  RCCHECK(rclc_publisher_init_best_effort(
      &nav_sat_fix_publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
      "/white_crash/gps/fix"));
  
  for (auto tof : tof_sensors) {
    char topic[30];
    snprintf(topic, sizeof(topic), "/white_crash/%s_distance", tof->name);
    RCCHECK(rclc_publisher_init_best_effort(
        &tof->publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
        topic));
  }
}

void destroy_ros_node_and_publishers() { 
  RCCHECK(rcl_publisher_fini(&log_publisher, &node));
  RCCHECK(rcl_publisher_fini(&battery_publisher, &node));
  RCCHECK(rcl_publisher_fini(&rosout_publisher, &node));
  RCCHECK(rcl_publisher_fini(&update_publisher, &node));
  for (auto tof : tof_sensors) {
    RCCHECK(rcl_publisher_fini(&tof->publisher, &node));
  }
  RCCHECK(rcl_publisher_fini(&nav_sat_fix_publisher, &node));

  RCCHECK(rcl_node_fini(&node));
}



void setup_micro_ros_wifi() {
  // go through wifi_logins and find the first available wifi
  // network that matches the ssid, but don't connect to it yet

  ConnectionInfo * connection_info = NULL;
  while (connection_info == NULL) {
    int network_count = WiFi.scanNetworks();
    for (auto &wifi_login : wifi_logins) {
      for (int i = 0; i < network_count; i++) {
        String ssid = WiFi.SSID(i);
        if (ssid == wifi_login.ssid) {
          connection_info = &wifi_login;
          break;
        }
      }
      if (connection_info != NULL) {
        break;
      }
    }
    if (connection_info == NULL) {
      Serial.printf("No wifi networks found, retrying in 5 seconds\n");
      delay(5000);
    }
  }

  Serial.printf("Found wifi network %s, connecting to %s\n", connection_info->ssid, connection_info->ssid);

  delay(5000); // wait for serial monitor to connect
  Serial.printf("setting up micro ros\n");

	WiFi.begin(connection_info->ssid, connection_info->password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  IPAddress agent_ip;
  WiFi.hostByName(connection_info->micro_ros_hostname, agent_ip);
  Serial.printf("Connected to wifi, agent ip: %s %s\n",connection_info->micro_ros_hostname, agent_ip.toString().c_str());
 
  static struct micro_ros_agent_locator locator;
  locator.address = agent_ip;
  locator.port = connection_info->micro_ros_port;

  rmw_uros_set_custom_transport(
      false,
      (void *) &locator,
      platformio_transport_open,
      platformio_transport_close,
      platformio_transport_write,
      platformio_transport_read
  );

  allocator = rcl_get_default_allocator();
  configTime(0, 0, "pool.ntp.org");



  // Wait for NTP sync with timeout
  const int NTP_TIMEOUT_MS = 5000;
  const int NTP_CHECK_INTERVAL_MS = 100;
  int waited_ms = 0;
  bool time_valid = false;

  struct tm timeinfo;
  while (!time_valid && waited_ms < NTP_TIMEOUT_MS) {
      time_valid = getLocalTime(&timeinfo);
      if (!time_valid) {
          delay(NTP_CHECK_INTERVAL_MS);
          waited_ms += NTP_CHECK_INTERVAL_MS;
      }
  }
  
  // set gnss time to ntp time utc
  struct timeval tv;
  if (gettimeofday(&tv, NULL) != 0) {
    Serial.println("Failed to obtain time");
    return;
  }

  gmtime_r(&tv.tv_sec, &timeinfo);  // Convert to UTC time structure

  // print all fields of timeinfo
  Serial.printf("UTC time: %d-%02d-%02d %02d:%02d:%02d\n",
                timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);


  // // Set GPS time using UTC values from timeinfo with microsecond precision
  // if (gnss.setUTCTimeAssistance(
  //   timeinfo.tm_year + 1900,    // Years since 1900
  //   timeinfo.tm_mon + 1,        // Months since January (0-11)
  //   timeinfo.tm_mday,           // Day of month
  //   timeinfo.tm_hour,           // Hour (0-23)
  //   timeinfo.tm_min,            // Minutes
  //   timeinfo.tm_sec,            // Seconds
  //   tv.tv_usec * 1000,         // Convert microseconds to nanoseconds
  //   1

  // )) {
  //   Serial.println("Time assistance successful with ns precision");
  // } else {
  //   Serial.println("Time assistance failed");
  // }
  //   // Set approximate position assistance
  //   // Convert degrees to degrees * 1E-7 for u-blox
  //   int32_t lat = sidewalk_in_front_of_driveway.lat * 1E7;
  //   int32_t lon = sidewalk_in_front_of_driveway.lon * 1E7;
  //   int32_t alt = 10 * 100; // ~10m altitude in centimeters
  //   uint32_t posAcc = 5000;  // 50m accuracy in centimeters

  //   if (gnss.setPositionAssistanceLLH(lat, lon, alt, posAcc)) {
  //       Serial.println("Position assistance successful");
  //   } else {
  //       Serial.println("Position assistance failed");
  //   }

    // gnss.saveConfiguration();
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
  static bool first_time = true;
  static int message_count = 0;
  static float last_virtual_bat = NAN;
  static bool last_avoid_collisions = false;
  ++message_count;

  if (rc_data.failsafe) {
    rx_str = 0;
    rx_esc = 0;
    rx_aux = 0;

  } else {
    rx_str = rc_data.channels[0];
    rx_esc = rc_data.channels[1];
    rx_aux = rc_data.channels[2];
    button_sc_pressed = crsf_ns::crsf_rc_channel_to_bool(rc_data.channels[6]);
    toggle_a_enabled = crsf_ns::crsf_rc_channel_to_bool(rc_data.channels[8]);
    toggle_b_enabled = crsf_ns::crsf_rc_channel_to_bool(rc_data.channels[9]);
    p1_knob_percent = crsf_ns::crsf_rc_channel_to_ratio(rc_data.channels[4]);
    p2_knob_percent = crsf_ns::crsf_rc_channel_to_ratio(rc_data.channels[5]);
  }

  // set parameters based on rc input
  virtual_vbat = virtual_vbat_floor + (virtual_vbat_ceiling - virtual_vbat_floor) * p1_knob_percent;
  avoid_collisions = toggle_a_enabled;

  if (first_time) {
    last_virtual_bat = virtual_vbat;
    last_avoid_collisions = avoid_collisions;
    first_time = false;
  }

  if (fabs(last_virtual_bat - virtual_vbat)>0.03) {
    char str[20];
    snprintf(str, sizeof(str), "v_max: %0.1f", virtual_vbat);
    set_temporary_display_string(str);
    last_virtual_bat = virtual_vbat;
  }
  if (avoid_collisions != last_avoid_collisions) {
    if (avoid_collisions) {
      set_temporary_display_string("avoid");
    } else {
      set_temporary_display_string("no avoid");
    }
    last_avoid_collisions = avoid_collisions;
  }


  // log the inputs
  if (false && message_count % 100 == 0) {
    logf("rc str: %d esc: %d aux: %d sc: %d a: %d b: %d p1: %0.4f p2: %0.4f",
        rx_str,
        rx_esc,
        rx_aux,
        button_sc_pressed,
        toggle_a_enabled,
        toggle_b_enabled,
        p1_knob_percent,
        p2_knob_percent);
  }
  if (false && message_count % 100 == 0) {
    // log all rc channels 0..15
    logf("rc channels: 0:%d 1:%d 2:%d 3:%d 4:%d 5:%d 6:%d 7:%d 8:%d 9:%d 10:%d 11:%d 12:%d 13:%d 14:%d 15:%d",
        rc_data.channels[0],
        rc_data.channels[1],
        rc_data.channels[2],
        rc_data.channels[3],
         rc_data.channels[4],
         rc_data.channels[5],
         rc_data.channels[6],
         rc_data.channels[7],
         rc_data.channels[8],
         rc_data.channels[9],
         rc_data.channels[10],
         rc_data.channels[11],
         rc_data.channels[12],
         rc_data.channels[13],
         rc_data.channels[14],
         rc_data.channels[15]);

  }
}

unsigned long last_loop_time_ms = 0;
unsigned long loop_time_ms = 0;

// returns true if loop time passes through n ms boundary
bool every_n_ms(unsigned long last_loop_ms, unsigned long loop_ms, unsigned long ms) {
  return (last_loop_ms % ms) + (loop_ms - last_loop_ms) >= ms;
}

float scale_motor_command_to_virtual_battery(float command) {
  // motor.go commands range from -1.0 to 1.0
  // we want the motor to act like it is running on a battery with voltage
  // of virtual_vbat. Note, this should only decrease the command, not increase it.

  if (virtual_vbat >= v_bat) {
    return command;
  }

  // scale the command to the virtual battery
  command *= virtual_vbat / v_bat;
  if (command > 1.0) {
    command = 1.0;
  }
  if (command < -1.0) {
    command = -1.0;
  }
  return command;

} 

// sets motor speeds to go to a lat/lon
// returns true if arrived
bool go_toward_lat_lon(lat_lon destination, float * meters_to_next_waypoint) {

  const float max_speed = 0.5;

  if (gnss.getLatitude(0) == 0) {
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
  double current_heading_degrees = compass.get_azimuth_degrees() + compass_mounting_angle_degrees;

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
  left_motor.go(scale_motor_command_to_virtual_battery(left_motor_speed));
  right_motor.go(scale_motor_command_to_virtual_battery(right_motor_speed));

  if (every_n_ms(last_loop_time_ms, loop_time_ms, 100)) {
    logf("speed: %f, %f destination: %f %f direction: %s, distance_remaining: %0.4f, desired_bearing_degrees: %0.4f current_heading_degrees: %0.4f heading_error: %0.4f",
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

  float speed = crsf_ns::crsf_rc_channel_to_float(rx_esc);
  float str_speed = crsf_ns::crsf_rc_channel_to_float(rx_str);

  // only avoid collisions when moving forward
  if (avoid_collisions && speed > 0) {
    // if we are moving forward and the tof is less than 0.9 meters, turn or slow to avoid collions
    bool collision_ahead = tof_center.distance < 0.9;
    bool collision_right = tof_right.distance < 0.9;
    bool collision_left = tof_left.distance < 0.9;
    float collision_distance = std::min(tof_center.distance, std::min(tof_right.distance, tof_left.distance));

    float left_distance = isnan(tof_left.distance) ? std::numeric_limits<float>::max() : tof_left.distance;
    float right_distance = isnan(tof_right.distance) ? std::numeric_limits<float>::max() : tof_right.distance;

    // turn or slow to avoid collisions
    if (collision_right && (right_distance < left_distance)) {
      str_speed = -0.25;
    } else if (collision_left && (left_distance < right_distance)) {
      str_speed = 0.25;
    } else if (collision_right && collision_left || collision_ahead) {
      auto max_speed = min(speed, collision_distance  - 0.1f);
      if (max_speed < 0) {
        max_speed = 0;
      }
      
      speed = max_speed;
    } 
  }

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

  right_motor.go(scale_motor_command_to_virtual_battery(right_speed), false);
  left_motor.go(scale_motor_command_to_virtual_battery(left_speed), false);
}

void update_compass_calibration(int min_x, int max_x, int min_y, int max_y, int min_z, int max_z) {

  compass.set_calibration(min_x, max_x, min_y, max_y, min_z, max_z);
  logf("updated compass calibration to %d %d %d %d %d %d", min_x, max_x, min_y, max_y, min_z, max_z);
}
//////////////////////////////////
// Finite state machine
class HandMode : public Task {
 public:
  HandMode() {
    name = "hand";
  }

  virtual void begin() override {
    logf("hand mode begin");
  }

  // implement the execute method
  virtual void execute() override {
    update_motor_speeds();
  }

} hand_mode;

class CalibrateCompassMode : public Task {
 public:
  int min_x = std::numeric_limits<int>::max();
  int max_x = std::numeric_limits<int>::min();
  int min_y = std::numeric_limits<int>::max();
  int max_y = std::numeric_limits<int>::min();
  int min_z = std::numeric_limits<int>::max();
  int max_z = std::numeric_limits<int>::min();

  CalibrateCompassMode() {
    name = "cal-comp";
  }

  virtual void begin() override {
    logf("calibrate compass mode");
    min_x = min_y = min_z = std::numeric_limits<int>::max();
    max_x = max_y = max_z = std::numeric_limits<int>::min();
  }

  // implement the execute method
  virtual void execute() override {
    update_motor_speeds();

    int x = compass.last_reading.x;
    int y = compass.last_reading.y;
    int z = compass.last_reading.z;

    logf("compass_xyz: %d %d %d", x, y, z);
    if (x < min_x) min_x = x;
    if (x > max_x) max_x = x;
    if (y < min_y) min_y = y;
    if (y > max_y) max_y = y;
    if (z < min_z) min_z = z;
    if (z > max_z) max_z = z;
  }

  virtual void end() override {
    update_compass_calibration(min_x, max_x, min_y, max_y, min_z, max_z);

    logf("calibrated compass: %d %d %d %d %d %d", min_x, max_x, min_y, max_y, min_z, max_z);

    // write to flash memory
    {
      File file = SPIFFS.open(compass_calibration_file_path, FILE_WRITE);
      if (!file) {
        logf("failed to open file for writing");
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

  void begin() override {
    done = false;
    logf("off mode begin");
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
  float unstick_left_start_meters = NAN;
  float unstick_right_start_meters = NAN;

  AutoMode() {
    name = "auto";
  }
  int step = 0;

  void begin() override {
    done = false;
    logf("auto mode begin");
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
      logf("unsticking left: %0.4f right: %0.4f", left_unstick_distance, right_unstick_distance);
      // considered unstuck when both motors have gone minimum_unstick_distance absolute
      if ((left_unstick_distance > minimum_unstick_distance) && (right_unstick_distance > minimum_unstick_distance)) {
        logf("unstuck");
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
    Fsm::Edge("auto", "rc-moved", "hand"),
    Fsm::Edge("off", "hand", "hand"),
    Fsm::Edge("*", "off", "off"),
};

Fsm fsm(tasks, edges);

void ros_thread(void *arg) {
  // setup_micro_ros_wifi will hang if it can't connect towifi
  // calling in a separate thread allows the rest of the system to continue
  setup_micro_ros_wifi();

  // connect, montior connection, and reconnect if necessary
  while (true) {
    while (ros_ready == false) {
      Serial.printf("Attempting to connect to micro-ROS agent\n");
      if (rclc_support_init(&support, 0, NULL, &allocator) == RCL_RET_OK) {
        create_ros_node_and_publishers();    
        ros_ready = true;
        logf("Connected to micro-ROS agent");
      } else {
        delay(1000);
      }
    }
    // make sure ros is still connected
    const int timout_ms = 100;
    if (rmw_uros_ping_agent(timout_ms, 10) != RMW_RET_OK) {
      ros_ready = false;
      Serial.printf("Lost connection to ROS agent\n");
      Serial.printf("Shutting down rclc_support\n");
      destroy_ros_node_and_publishers();
      rclc_support_fini(&support);
      Serial.printf("rclc_support shut down\n");
    }

    delay(100); // sleep to allow other tasks to run
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

void start_tof_distance_sensor(TofSensor & tof) {
  pinMode(tof.power_pin, OUTPUT);
  digitalWrite(tof.power_pin, HIGH);
  tof.sensor->setBus(&Wire);
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

  tof.sensor->setMeasurementTimingBudget(tof_timing_budget_ms * 1000);
  tof.sensor->setTimeout(2);  // timeout ms for synchronous measurements
  tof.sensor->startContinuous(tof_timing_budget_ms);
  tof.running = true;
}

void turn_off_tof_distance_sensor(TofSensor & tof) {
  digitalWrite(tof.power_pin, LOW);
  tof.running = false;
  tof.turn_off_ms = millis();
}

#ifdef pin_built_in_led
void flash_forever() {
  pinMode(pin_built_in_led, OUTPUT);
  while(true) {
    Serial.println("off");
    digitalWrite(pin_built_in_led, LOW);
    delay(250);
    Serial.println("on");
    digitalWrite(pin_built_in_led, HIGH);
    delay(250);
  }
}
#endif



//////////////////////////////////
// Main setup and loop

void setup() {


#ifdef pin_built_in_led
  pinMode(pin_built_in_led, OUTPUT);
  digitalWrite(pin_built_in_led, HIGH);
#endif
  Serial.begin(115200);

  // note: it takes about 1.8 sconds after boot for serial messages to show in platformio

  // int i = 0;
  // do {
  //   Serial.printf("white-crash has been running for %.1f seconds\n", i / 10.0);
  //   Serial.flush();
  //   delay(100);
  //   ++i;
  // } while(true);

  SPIFFS.begin();


  // preallocate log message for all logging
  log_msg.data.data = (char *)malloc(200);
  log_msg.data.capacity = 200;
  log_msg.data.size = 0;

  // preallocate log_msg_ros for all ros logging
  rosout_msg.level = rcl_interfaces__msg__Log__INFO;
  rosout_msg.name.data = (char *)malloc(20);
  rosout_msg.name.capacity = 20;
  strncpy(rosout_msg.name.data, "white_crash", 20);
  rosout_msg.name.size = strlen("white_crash");
  rosout_msg.msg.data = (char *)malloc(200);
  rosout_msg.msg.capacity = 200;
  rosout_msg.msg.size = 0;
  rosout_msg.file.data = (char *)malloc(20);
  rosout_msg.file.capacity = 20;
  strncpy(rosout_msg.file.data, "", 20);
  rosout_msg.file.size = 0;
  rosout_msg.function.data = (char *)malloc(20);
  rosout_msg.function.capacity = 20;
  strncpy(rosout_msg.function.data, "", 20);
  rosout_msg.function.size = 0;
  rosout_msg.line = 0;


  tof_distance_msg.header.frame_id.data = (char *)malloc(20);
  tof_distance_msg.header.frame_id.capacity = 20;
  strncpy(tof_distance_msg.header.frame_id.data, "tof_frame", 20);
  tof_distance_msg.header.frame_id.size = strlen("tof_frame");

  nav_sat_fix_msg.header.frame_id.data = (char *)malloc(20);
  nav_sat_fix_msg.header.frame_id.capacity = 20;
  strncpy(nav_sat_fix_msg.header.frame_id.data, "gps_frame", 20);
  nav_sat_fix_msg.header.frame_id.size = strlen("gps_frame");

  tof_distance_msg.min_range = 0.03;
  tof_distance_msg.max_range = 1.0;
  tof_distance_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
  tof_distance_msg.field_of_view = 27 * M_PI / 180; // 27 degrees

  Wire.setPins(pin_sda, pin_scl);
  Wire.begin();
  Wire.setTimeOut(5); // ms


  //Wire1.setPins(pin_tof_sda, pin_tof_scl);
  //Wire1.begin();
  //Wire1.setTimeOut(5); // ms

  left_speedometer.meters_per_tick = meters_per_odometer_tick;
  right_speedometer.meters_per_tick = meters_per_odometer_tick;

  fsm.begin();

  Serial.write("white_crash\n");
  crsf_serial.setRxBufferSize(4096);
  crsf_serial.begin(420000, SERIAL_8N1, pin_crsf_rx, pin_crsf_tx);
  gnss_serial.setRxBufferSize(4096);

  gnss_serial.begin(38400, SERIAL_8N1, pin_gps_rx, pin_gps_tx);

  // In setup(), after gnss.begin():
  if (use_gnss) {
    
    if (gnss.begin(gnss_serial, 2000, true)) {

      // gnss.factoryReset();
      // while (true) {
      //   Serial.println("reset gnss");
      //   delay(1000);
      // }

        
      // // Configure message output
      // gnss.setUART1Output(COM_TYPE_NMEA | COM_TYPE_UBX);
      gnss.setUART1Output(COM_TYPE_NMEA);
      gnss.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);  // Time and date
      gnss.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);  // Fix data
      gnss.enableNMEAMessage(UBX_NMEA_ZDA, COM_PORT_UART1);  // Precise time/date

      // Enable multiple GNSS constellations
      gnss.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS);     // GPS USA
      gnss.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO); // Galileo Europe
      gnss.enableGNSS(true, SFE_UBLOX_GNSS_ID_BEIDOU);  // BeiDou China
      gnss.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS); // GLONASS Russia

      // gnss.setAutoPVT(true); // Automatically send Position, Velocity, Time messages
      
      // gnss.setAopCfg(true); // Enable AOP (Assisted Orbit Prediction) for faster startup

      // gnss.setDynamicModel(DYN_MODEL_PORTABLE);
      // gnss.setUTCTimeAssistance(2025, 4, 6, 12, 0, 0, 0, 3600, 0, 3);


      // // Configure time base and update rate
      gnss.setMeasurementRate(100);     // Measurements every 100ms
      gnss.setNavigationFrequency(10);  // Navigation rate 10Hz

      // // Save configuration
      bool success = gnss.saveConfiguration();
      Serial.printf("GPS configuration %s\n", success ? "saved" : "failed to save");

      // Serial.println("GPS configured");
    }
    else {
      Serial.println("GPS failed to initialize");
    }
  }

  if (!load_compass_calibration_from_spiffs()) {
    compass.set_calibration(-950, 675, -1510, 47, 0, 850);
  }
  // see https://www.magnetic-declination.com/
  // compass.setMagneticDeclination(11, 24);


  start_tof_distance_sensor(tof_left);
  start_tof_distance_sensor(tof_right);
  start_tof_distance_sensor(tof_center);

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
  right_motor.go(0);
  left_motor.go(0);
#ifdef pin_built_in_led
  pinMode(pin_built_in_led, OUTPUT);
  pinMode(pin_battery_voltage, INPUT);
#endif
  // The input voltage of ADC will be attenuated, extending 
  // the range of measurement to up to approx. 2600 mV. 
  // (1V input = ADC reading of 1575).
  analogSetPinAttenuation(pin_battery_voltage, ADC_11db); 
  adcAttachPin(pin_battery_voltage);

  // create a thread for ros stuff
  if(true) {
    xTaskCreatePinnedToCore(
        ros_thread,
        "ros_thread",
        16384,
        NULL,
        1,
        NULL,
        1);
  }
  // reset all serial data
  uart_flush_input(0);
  uart_flush_input(1);

#ifdef pin_built_in_led
  digitalWrite(pin_built_in_led, 0);
#endif
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
      logf(Severity::WARN, "HANG: %s took %d ms", name, elapsed);
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

  delay(1); // give the system a little time to breathe
  BlockTimer bt(loop_stats);
  last_loop_time_ms = loop_time_ms;
  loop_time_ms = millis();

  bool every_1_ms = every_n_ms(last_loop_time_ms, loop_time_ms, 1);
  bool every_10_ms = every_n_ms(last_loop_time_ms, loop_time_ms, 10);
  bool every_100_ms = every_n_ms(last_loop_time_ms, loop_time_ms, 100);
  bool every_200_ms = every_n_ms(last_loop_time_ms, loop_time_ms, 200);
  bool every_1000_ms = every_n_ms(last_loop_time_ms, loop_time_ms, 1000);
  bool every_minute = every_n_ms(last_loop_time_ms, loop_time_ms, 60 * 1000);

  if (every_1000_ms) {
    Serial.println("in loop");
  }

  if (every_100_ms) {
    left_speedometer.update_from_sensor(micros(), left_encoder.odometer_a, left_encoder.last_odometer_a_us, left_encoder.odometer_b, left_encoder.last_odometer_b_us);
    right_speedometer.update_from_sensor(micros(), right_encoder.odometer_a, right_encoder.last_odometer_a_us, right_encoder.odometer_b, right_encoder.last_odometer_b_us);

    update_msg.battery_voltage = v_bat;
    update_msg.left_speed = left_speedometer.get_velocity();
    update_msg.right_speed = right_speedometer.get_velocity();
    update_msg.left_motor_command = left_motor.get_setpoint();
    update_msg.right_motor_command = right_motor.get_setpoint();
    update_msg.rx_esc = crsf_ns::crsf_rc_channel_to_float(rx_esc);
    update_msg.rx_str = crsf_ns::crsf_rc_channel_to_float(rx_str);
    update_msg.yaw_degrees = compass.get_azimuth_degrees();
    update_msg.mag_x = compass.last_reading.x;
    update_msg.mag_y = compass.last_reading.y;
    update_msg.mag_z = compass.last_reading.z;

    if (ros_ready) {
      // publish update message
      RCSOFTCHECK(rcl_publish(&update_publisher, &update_msg, NULL));
    }
  }


  // tof distance
  if (every_n_ms(last_loop_time_ms, loop_time_ms, tof_timing_budget_ms)) {
    for (auto tof : tof_sensors) {
      // extra block to limit scope of BlockTimer
      {
        {
          auto sensor = tof->sensor;
          BlockTimer bt(tof_distance_stats);

          auto start = millis();
          if (sensor->read(true)) {
            if (sensor->ranging_data.range_status == VL53L1X::RangeValid) {
              tof->distance = sensor->ranging_data.range_mm / 1000.0;
    
            } else {
              // printf("VL53L1X %s distance status: %d\n", tof->name, sensor->ranging_data.range_status);
              tof->distance = std::numeric_limits<float>::quiet_NaN();
            }      
          }
        }
        if (ros_ready)
        {
          set_stamp(tof_distance_msg.header.stamp);

          tof_distance_msg.range = tof->distance;
          if (tof_distance_msg.radiation_type != sensor_msgs__msg__Range__INFRARED) {
            logf( 
              Severity::ERROR, 
              "tof_distance_msg.radiation_type != sensor_msgs__msg__Range__INFRARED, value: %d", 
              tof_distance_msg.radiation_type);
            
            tof_distance_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
          }    
          RCSOFTCHECK(rcl_publish(&(tof->publisher), &tof_distance_msg, NULL));
        }    
      }
    }
  }

  if (use_gnss && every_10_ms) {
    BlockTimer bt(gps_stats);
    HangChecker hc("gps");
    gnss.checkUblox();
    gnss.checkCallbacks();
  }

  if (use_gps && every_10_ms) {
    BlockTimer bt(gps_stats);
    HangChecker hc("gps");
    while (gnss_serial.available()) {
      auto c = gnss_serial.read();
      gps.encode(c);
      Serial.write(c);
    }
  }

  if (use_gps && every_1000_ms ) {
    auto & location = gps.location;
    // log all fields with logf

    logf( "chars processed: %d checksum ok: %d checksum bad: %d valid: %d lat: %0.6f lon: %0.6f age: %d second: %d satellites: %d",
      gps.charsProcessed(),
      gps.passedChecksum(),
      gps.failedChecksum(),
      location.isValid(),
      location.lat(),
      location.lng(),
      location.age(),
      gps.time.second(),
      gps.satellites.value()
    );
  }

if (use_gnss && every_100_ms) {
  // publish nav_sat_fix message
  nav_sat_fix_msg.status.service = sensor_msgs__msg__NavSatStatus__SERVICE_GPS;

  // set status based on fix type
  switch (gnss.getFixType(0)) {
    case 0:
      nav_sat_fix_msg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_NO_FIX;
      break;
    case 1:
      nav_sat_fix_msg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_FIX;
      break;
    case 2:
      nav_sat_fix_msg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_SBAS_FIX;
      break;
    case 3:
      nav_sat_fix_msg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_GBAS_FIX;
      break;
    default:
      nav_sat_fix_msg.status.status = sensor_msgs__msg__NavSatStatus__STATUS_NO_FIX;
      break;
  }

  if (gnss.getFixType(0) == 0) {
    nav_sat_fix_msg.latitude = NAN;
    nav_sat_fix_msg.longitude = NAN;
    nav_sat_fix_msg.altitude = NAN;
  } else {
    nav_sat_fix_msg.position_covariance_type = sensor_msgs__msg__NavSatFix__COVARIANCE_TYPE_APPROXIMATED;
    nav_sat_fix_msg.latitude = gnss.getLatitude(0) * 1e-7;
    nav_sat_fix_msg.longitude = gnss.getLongitude(0) * 1e-7;
    nav_sat_fix_msg.altitude = gnss.getAltitude(0) * 1e-3;
  }

  // publish nav_sat_fix message
  if (ros_ready) {
    set_stamp(nav_sat_fix_msg.header.stamp);
    RCSOFTCHECK(rcl_publish(&nav_sat_fix_publisher, &nav_sat_fix_msg, NULL));
  }

};  

// In your logging code, add more debug info:
if (use_gnss && every_1000_ms) {
  logf("gps date: (%d) %d-%d-%d %02d:%02d:%02d (%f,%f) fix: %d siv: %d",
    gnss.getTimeValid(0),
    gnss.getYear(0),
    gnss.getMonth(0),
    gnss.getDay(0),
    gnss.getHour(0),
    gnss.getMinute(0),
    gnss.getSecond(0),
    gnss.getLatitude(0) * 1e-7,
    gnss.getLongitude(0) * 1e-7,
    gnss.getFixType(0),
    gnss.getSIV(0)  // Satellites in view
  );
}


  if (use_gnss && every_1000_ms) {
    gnss.getFixType(0);
//    Serial.printf("gps fix type: %d\n", gnss.getFixType()); 
  }

  if (every_1000_ms) {
    if (ros_ready) {
      RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL));
    }
  }


  if (every_100_ms) {
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
  
  if (enable_stats_logging && every_minute) {
    for (auto stats : {gps_stats, log_stats, loop_stats, crsf_stats, compass_stats, telemetry_stats, serial_read_stats, crsf_parse_stats, tof_distance_stats}) {
      stats.to_log_msg(&log_msg);
      log(log_msg);
    }
  }

  if (isnan(v_bat) || every_1000_ms) {
    HangChecker hc("battery");
    analogReadResolution(12);
    int sum = 0;
    const int sample_count = 30;
    for (int i = 0; i < sample_count; i++) {
      sum += analogRead(pin_battery_voltage);
    }


    // const float v_bat_scale = 0.003714; // white-crash s
    const float v_bat_scale = 0.0077578 * 7.9 / 30. * 7.96 / 3.06; // white-crash m    
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
    static int last_rx_str = 0;
    static int last_rx_esc = 0;

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

    // rc moved event
    {
      using namespace crsf_ns;
      if (get_input_position(last_rx_str) != get_input_position(rx_str) || get_input_position(last_rx_esc) != get_input_position(rx_esc)) {
        fsm.set_event("rc-moved");
      }
    }

    last_aux_mode = aux_mode;
    last_button_sc_pressed = button_sc_pressed;
    last_rx_str = rx_str;
    last_rx_esc = rx_esc;
  }

  if (every_100_ms) {
    BlockTimer bt(compass_stats);
    HangChecker hc("compass");
    compass.update();
  }

  // blink for a few ms every second to show signs of life
#ifdef pin_built_in_led
  if (millis() % 1000 < 5) {
    digitalWrite(pin_built_in_led, HIGH);
  } else {
    digitalWrite(pin_built_in_led, LOW);
  }
#endif

  if (every_1_ms) {
    BlockTimer bt(crsf_stats);
    HangChecker hc("crsf");
    crsf.update();  // update as fast as possible, will call callbacks when data is ready
  }

  if (every_n_ms(last_loop_time_ms, loop_time_ms, 200)) {
    BlockTimer bt(telemetry_stats);
    HangChecker hc("telemetry");

    uint8_t percent = estimate_lipo_battery_percent_from_voltage(v_bat);

    crsf.send_battery(v_bat, 0, 0, percent);
    char display_string[16];

    // if we have a temporary display string, and the timeout is not up, use it
    if (temporary_display_string_timeout_ms > 0 && millis() - temporary_display_string_set_ms < temporary_display_string_timeout_ms) {
      snprintf(display_string, sizeof(display_string), "%s", temporary_display_string);
    } else {
      fsm.current_task->get_display_string(display_string, 16);
    }


    crsf.send_flight_mode(display_string);
    crsf.send_attitude(0, 0, compass.get_azimuth_degrees());

    if (use_gnss) {
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
  }
}

