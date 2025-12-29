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

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

#include "quadrature_encoder.h"
#include "speedometer.h"

#include "driver/uart.h" // to clear the serial buffer

// Watchdog and diagnostics
#include <esp_task_wdt.h>
#include <esp_system.h>  // for esp_reset_reason()
#include <esp_core_dump.h> // for esp_core_dump_init()
#include <Preferences.h>  // for flash storage of diagnostics

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
#include <geometry_msgs/msg/twist.h>

#include <white_crash_msgs/msg/update.h>


#include "RunStatistics.h"
#include "StuckChecker.h"
#include "secrets/wifi_login.h"

//////////////////////////////////
// Diagnostic State Tracking (RTC Memory)

// RTC memory survives watchdog resets but not power cycles
// Fast SRAM with unlimited write endurance - safe to update every loop
// RTC_NOINIT_ATTR preserves data even across panic/abort resets
RTC_NOINIT_ATTR struct DiagnosticState {
  uint32_t magic;
  uint32_t boot_count;
  uint32_t last_loop_ms;
  uint32_t consecutive_wdt_resets;
  char last_location[40];
  esp_reset_reason_t last_reset;
} diag;

const uint32_t DIAG_MAGIC = 0xC0FFEE42;

Preferences diagnostics_prefs;

// Save diagnostics to flash storage (survives power cycles)
void save_diagnostics_to_flash() {
  diagnostics_prefs.begin("white_crash", false);  // Read-write mode
  diagnostics_prefs.putUInt("magic", diag.magic);
  diagnostics_prefs.putUInt("boot_count", diag.boot_count);
  diagnostics_prefs.putUInt("wdt_resets", diag.consecutive_wdt_resets);
  diagnostics_prefs.putString("last_loc", diag.last_location);
  diagnostics_prefs.putUInt("last_reset", static_cast<uint32_t>(diag.last_reset));
  diagnostics_prefs.end();
}

// Load diagnostics from flash storage (for reporting after power cycle)
void load_diagnostics_from_flash() {
  diagnostics_prefs.begin("white_crash", true);  // Read-only mode
  if (diagnostics_prefs.isKey("magic")) {
    uint32_t flash_magic = diagnostics_prefs.getUInt("magic", 0);
    if (flash_magic == DIAG_MAGIC) {
      uint32_t crash_boot_count = diagnostics_prefs.getUInt("boot_count", 0);
      uint32_t wdt_resets = diagnostics_prefs.getUInt("wdt_resets", 0);
      
      // Sanity check: crash boot count should be <= current boot count
      // If not, flash data is stale/corrupted - clear it
      if (crash_boot_count > diag.boot_count || crash_boot_count == 0) {
        Serial.println("\n=== STALE/INVALID CRASH DATA IN FLASH ===");
        Serial.printf("Flash boot count (%lu) > current (%lu) - clearing flash data\n", 
                      crash_boot_count, diag.boot_count);
        Serial.println("=== FLASH DATA CLEARED ===\n");
        diagnostics_prefs.end();
        
        // Clear the stale data
        diagnostics_prefs.begin("white_crash", false);  // Read-write mode
        diagnostics_prefs.clear();
        diagnostics_prefs.end();
        return;
      }
      
      // Only show if there were actual crashes (not just power_on initialization)
      if (wdt_resets > 0) {
        uint32_t boots_since_crash = diag.boot_count - crash_boot_count;
        
        Serial.println("\n=== PREVIOUS CRASH DATA FROM FLASH ===");
        Serial.printf("Crash occurred %lu boot(s) ago (boot #%lu, current boot #%lu)\n", 
                      boots_since_crash, crash_boot_count, diag.boot_count);
        Serial.printf("Consecutive WDT resets: %u\n", wdt_resets);
        Serial.printf("Last location: %s\n", diagnostics_prefs.getString("last_loc", "unknown").c_str());
        uint32_t reset_reason = diagnostics_prefs.getUInt("last_reset", 0);
        Serial.printf("Last reset reason: ");
        switch(static_cast<esp_reset_reason_t>(reset_reason)) {
          case ESP_RST_POWERON:   Serial.println("Power-on"); break;
          case ESP_RST_SW:        Serial.println("Software reset"); break;
          case ESP_RST_PANIC:     Serial.println("Exception/panic"); break;
          case ESP_RST_INT_WDT:   Serial.println("Interrupt watchdog"); break;
          case ESP_RST_TASK_WDT:  Serial.println("TASK WATCHDOG TIMEOUT"); break;
          case ESP_RST_WDT:       Serial.println("Other watchdog"); break;
          case ESP_RST_BROWNOUT:  Serial.println("Brownout"); break;
          default:                Serial.printf("Unknown (%u)\n", reset_reason); break;
        }
        Serial.println("=== END CRASH DATA ===");
      }
    }
  }
  diagnostics_prefs.end();
}

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
float meters_per_odometer_tick = 0.000653; // M

// hard code some interesting gps locations
class lat_lon {
 public:
  double lat;
  double lon;
};


// from https://cdip.ucsd.edu/m/deployment/id/997/?
float magnetic_declination_degrees = 11.888; // add this to heading to go from magnetic heading to compass heading

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

float contest_max_can_distance = 0.9; // about 3 feet

// where we store the compass calibration
const char *compass_calibration_file_path = "/compass_calibration.txt";
const char *bno055_calibration_file_path = "/bno055_calibration.txt";

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
const int pin_sda = 34;   // Yellow I2C SDA
const int pin_scl = 35;   // Blue I2C SCL

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
const int pin_left_fwd = 17;  // Left motor forward (was GPIO 32)
const int pin_left_rev = 16;  // Left motor reverse (was GPIO 21)
const int pin_right_fwd = 33;  // Right motor forward (was GPIO 18)
const int pin_right_rev = 37;  // Right motor reverse (was GPIO 26)

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
uint32_t tof_timing_budget_ms = 20;

AK8975Compass compass(Wire, 0x0E);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
imu::Vector<3> bno_orientation_degrees;
float compass_heading_degrees = NAN;
imu::Vector<3> bno_acceleration;
imu::Vector<3> bno_mag;
imu::Vector<3> bno_gyro_dps;


QuadratureEncoder left_encoder(pin_left_encoder_a, pin_left_encoder_b, meters_per_odometer_tick);
QuadratureEncoder right_encoder(pin_right_encoder_a, pin_right_encoder_b, meters_per_odometer_tick);

Speedometer left_speedometer;
Speedometer right_speedometer;

int rx_str = 0;
int rx_esc = 0;
int rx_aux = 0;
bool button_sc_pressed = false;
bool button_sd_pressed = false;
bool toggle_a_enabled = false;
bool toggle_b_enabled = false;
float p1_knob_percent = NAN;
float p2_knob_percent = NAN;
double v_bat = NAN;

bool ros_ready = false;
bool crash_data_published_to_ros = false;

// Mutex for protecting sensor data shared between i2c_sensor_thread and main loop
SemaphoreHandle_t sensor_data_mutex = NULL;

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


RunStatistics gps_stats("gps");
RunStatistics log_stats("logf");
RunStatistics loop_stats("loop");
RunStatistics crsf_stats("crsf");
RunStatistics compass_stats("compass");
RunStatistics telemetry_stats("telemetry");
RunStatistics serial_read_stats("serial_read");
RunStatistics rc_callback_stats("rc_callback");
RunStatistics crsf_parse_stats("crsf_parse");
RunStatistics tof_stats("tof");
RunStatistics tof_distance_stats("tof_distance");
RunStatistics bno_stats("bno");
RunStatistics i2c_thread_stats("i2c_thread");

StuckChecker left_stuck_checker;
StuckChecker right_stuck_checker;

//////////////////////////////////
// Helpers
// converts nan values to max, good for range sensor math
float nan_to_max(float v) {
  return isnan(v) ? std::numeric_limits<float>::max() : v;
}

// ensure degress [0,360) for d
float normalize_compass_degrees(float d) {
  while (d < 0.0) {
    d += 360.0;
  }
  while (d>=360.0) {
    d -= 360.0;
  }
  return d;
}

//////////////////////////////////
// Micro Ros

rcl_publisher_t log_publisher;
rcl_publisher_t rosout_publisher;
rcl_publisher_t battery_publisher;
rcl_publisher_t update_publisher;
rcl_publisher_t nav_sat_fix_publisher;

rcl_subscription_t cmd_vel_subscription;
rclc_executor_t executor;

sensor_msgs__msg__Range tof_distance_msg;
std_msgs__msg__String log_msg;
std_msgs__msg__Float32 battery_msg;
rcl_interfaces__msg__Log rosout_msg;
white_crash_msgs__msg__Update update_msg;
sensor_msgs__msg__NavSatFix nav_sat_fix_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

// cmd_vel state
float cmd_vel_linear = 0.0;
float cmd_vel_angular = 0.0;
unsigned long cmd_vel_last_received_ms = 0;
const unsigned long cmd_vel_timeout_ms = 500;

// Twist control state
bool twist_control_enabled = false;
float twist_target_linear = 0.0;
float twist_target_angular = 0.0;
float twist_target_accel_linear = 0.0;
float twist_target_accel_angular = 0.0;

// Internal ramping state (reset when control disabled to prevent windup)
float twist_ramped_linear = 0.0;
float twist_ramped_angular = 0.0;

// Actual ramped/effective values after internal ramping (for logging)
float twist_effective_linear = 0.0;
float twist_effective_angular = 0.0;
float twist_effective_accel_linear = 0.0;
float twist_effective_accel_angular = 0.0;
float v_left_target_effective = 0.0;
float v_right_target_effective = 0.0;

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

// Publish crash diagnostics to ROS (called after ROS connection established)
void publish_crash_data_to_ros() {
  diagnostics_prefs.begin("white_crash", true);  // Read-only mode
  if (diagnostics_prefs.isKey("magic")) {
    uint32_t flash_magic = diagnostics_prefs.getUInt("magic", 0);
    if (flash_magic == DIAG_MAGIC) {
      uint32_t wdt_resets = diagnostics_prefs.getUInt("wdt_resets", 0);
      uint32_t crash_boot_count = diagnostics_prefs.getUInt("boot_count", 0);
      
      // Sanity check: ignore stale/invalid data
      if (crash_boot_count > diag.boot_count || crash_boot_count == 0 || wdt_resets == 0) {
        diagnostics_prefs.end();
        return;
      }
      
      uint32_t boots_since_crash = diag.boot_count - crash_boot_count;
      String last_loc = diagnostics_prefs.getString("last_loc", "unknown");
      
      if (boots_since_crash == 0) {
        // Fresh crash from this boot session
        logf(Severity::WARN, "*** crash detected: System hung at '%s' (%u consecutive WDT resets) ***", 
             last_loc.c_str(), wdt_resets);
      } else {
        // Stale crash from previous boot(s)
        logf(Severity::INFO, "*** old crash data: System hung at '%s' %lu boot(s) ago (%u consecutive WDT resets) ***", 
             last_loc.c_str(), boots_since_crash, wdt_resets);
      }
    }
  }
  diagnostics_prefs.end();
}

// cmd_vel subscription callback
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  cmd_vel_linear = msg->linear.x;
  cmd_vel_angular = msg->angular.z;
  cmd_vel_last_received_ms = millis();
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

  // create cmd_vel subscription
  RCCHECK(rclc_subscription_init_best_effort(
      &cmd_vel_subscription,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/white_crash/cmd_vel"));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscription, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));
}

void destroy_ros_node_and_publishers() { 
  RCCHECK(rclc_executor_fini(&executor));
  RCCHECK(rcl_subscription_fini(&cmd_vel_subscription, &node));
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

  // delay(5000); // wait for serial monitor to connect
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
    button_sd_pressed =  crsf_ns::crsf_rc_channel_to_bool(rc_data.channels[7]);
    toggle_a_enabled = crsf_ns::crsf_rc_channel_to_bool(rc_data.channels[8]);
    toggle_b_enabled = crsf_ns::crsf_rc_channel_to_bool(rc_data.channels[9]);
    p1_knob_percent = crsf_ns::crsf_rc_channel_to_ratio(rc_data.channels[4]);
    p2_knob_percent = crsf_ns::crsf_rc_channel_to_ratio(rc_data.channels[5]);
  }

  // set parameters based on rc input
  virtual_vbat = virtual_vbat_floor + (virtual_vbat_ceiling - virtual_vbat_floor) * p1_knob_percent;

  if (first_time) {
    last_virtual_bat = virtual_vbat;
    first_time = false;
  }

  if (fabs(last_virtual_bat - virtual_vbat)>0.03) {
    char str[20];
    snprintf(str, sizeof(str), "v_max: %0.1f", virtual_vbat);
    set_temporary_display_string(str);
    last_virtual_bat = virtual_vbat;
  }

  // log the inputs
  if (false && message_count % 100 == 0) {
    logf("rc str: %d esc: %d aux: %d sc: %d sd: %d a: %d b: %d p1: %0.4f p2: %0.4f",
        rx_str,
        rx_esc,
        rx_aux,
        button_sc_pressed,
        button_sd_pressed,
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
  
  // Read compass heading with mutex protection
  double current_heading_degrees;
  xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
  current_heading_degrees = compass_heading_degrees; // use bno, it is better
  xSemaphoreGive(sensor_data_mutex);

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
  // str_speed is [-1,1] where negative is left, positive is right
  float str_speed = crsf_ns::crsf_rc_channel_to_float(rx_str);

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

  float right_speed = speed - str_speed;
  float left_speed = speed + str_speed;

  right_motor.go(scale_motor_command_to_virtual_battery(right_speed), false);
  left_motor.go(scale_motor_command_to_virtual_battery(left_speed), false);
}

void update_compass_calibration(int min_x, int max_x, int min_y, int max_y, int min_z, int max_z) {

  compass.set_calibration(min_x, max_x, min_y, max_y, min_z, max_z);
  logf("updated compass calibration to %d %d %d %d %d %d", min_x, max_x, min_y, max_y, min_z, max_z);
}

//////////////////////////////////
// PI Controller for Speed Control

/**
 * PI Controller with Feedforward for velocity control.
 * Outputs voltage command from target velocity, actual velocity, and time delta.
 * Uses feedforward model from Step 1 characterization plus PI correction.
 */
class PIController {
 private:
  // Feedforward model parameters (from Step 1 characterization)
  // V_ff(v) = ff_offset + ff_gain * v
  const float ff_offset = 0.271;  // Volts
  const float ff_gain = 1.800;    // V/(m/s)
  
  // Acceleration feedforward gain (Phase 3B)
  float ff_accel = 0.4;  // V/(m/s²) - reduced significantly, was causing overshoot during acceleration
  
  // PI gains (tuned from tracking data)
  float k_p = 4.0;   // V/(m/s) - HIGH for aggressive braking during deceleration
  float k_i = 3.0;    // V/(m/s²) - disabled to prevent oscillations
  
  // Controller state
  float error_integral = 0.0;  // Accumulated error (m·s)
  float last_target_sign = 0.0; // Track sign changes to reset integral
  
  // Anti-windup limits
  const float max_voltage = 12.0;  // Volts
  const float min_voltage = -12.0; // Volts
  const float max_integral = 1.0;  // Max accumulated error (m·s) - limits integral windup
  
 public:
  PIController() {}
  
  /**
   * Compute control output.
   * @param v_target Target velocity (m/s)
   * @param v_actual Actual velocity from speedometer (m/s)
   * @param dt Time step (seconds)
   * @param accel_target Expected acceleration (m/s²) for feedforward, default 0.0
   * @return Voltage command (Volts)
   */
  float compute(float v_target, float v_actual, float dt, float accel_target = 0.0) {
    // Reset integral if target changes sign (forward ↔ reverse)
    // This prevents accumulated error from one direction affecting the other
    float current_target_sign = (v_target > 0.01) ? 1.0 : ((v_target < -0.01) ? -1.0 : 0.0);
    if (last_target_sign != 0.0 && current_target_sign != 0.0 && 
        last_target_sign != current_target_sign) {
      error_integral = 0.0;
    }
    last_target_sign = current_target_sign;
    
    // Reset integral if target is near zero (stopped) to prevent drift
    if (abs(v_target) < 0.01) {
      error_integral = 0.0;
    }
    
    // Feedforward term - handles most of the steady-state control
    // For reverse motion, the offset must be negative
    float v_feedforward;
    if (v_target >= 0) {
      v_feedforward = ff_offset + ff_gain * v_target;
    } else {
      v_feedforward = -ff_offset + ff_gain * v_target;
    }
    
    // PI correction
    float error = v_target - v_actual;
    error_integral += error * dt;
    
    // Clamp integral to prevent unbounded growth
    error_integral = constrain(error_integral, -max_integral, max_integral);
    
    float v_pi = k_p * error + k_i * error_integral;
    
    // Acceleration feedforward term
    float v_accel = ff_accel * accel_target;
    
    // Total voltage command
    float v_total = v_feedforward + v_accel + v_pi;
    
    // Anti-windup: back-calculate integral if output saturates
    // This prevents integral from growing while output is clamped
    if (v_total > max_voltage) {
      v_total = max_voltage;
      // Back-calculate what integral should be at saturation
      float max_pi = v_total - v_feedforward;
      error_integral = (max_pi - k_p * error) / k_i;
    } else if (v_total < min_voltage) {
      v_total = min_voltage;
      float min_pi = v_total - v_feedforward;
      error_integral = (min_pi - k_p * error) / k_i;
    }
    
    return v_total;
  }
  
  /**
   * Reset controller state (e.g., when starting control)
   */
  void reset() {
    error_integral = 0.0;
    last_target_sign = 0.0;
  }
  
  /**
   * Set controller gains (for tuning)
   */
  void set_gains(float p, float i) {
    k_p = p;
    k_i = i;
  }
  
  /**
   * Set acceleration feedforward gain (for tuning)
   */
  void set_accel_gain(float accel_ff) {
    ff_accel = accel_ff;
  }
};

// Instantiate PI controllers for left and right wheels
PIController left_wheel_controller;
PIController right_wheel_controller;

// Acceleration limits for internal ramping (Phase 3B)
const float max_linear_accel = 1.0;  // m/s² - maximum linear acceleration
const float max_angular_accel = 2.0; // rad/s² - maximum angular acceleration

//////////////////////////////////
// Differential Drive Kinematics

// Track width (wheelbase) in meters - approximate for now
// TODO: Calibrate this value through rotation tests
const float track_width = 0.20;  // 20cm estimate

// Angular velocity PI control gains
// Proportional gain - immediate response to error
const float angle_k_p = 0.5;  // Initial value, tune empirically
// Integral gain - eliminates steady-state error
const float angle_k_i = 2.0;  // Initial value, tune empirically

// Angular velocity controller state
float angular_error_integral = 0.0;
const float max_angular_integral = 2.0;  // rad - limits integral windup

/**
 * Convert linear and angular velocity to individual wheel velocities.
 * Includes closed-loop angular velocity control using gyro feedback.
 * @param v_linear Linear velocity (m/s), positive = forward
 * @param omega_angular_cmd Commanded angular velocity (rad/s), positive = counter-clockwise
 * @param omega_angular_measured Measured angular velocity from gyro (rad/s)
 * @param v_left Output left wheel velocity (m/s)
 * @param v_right Output right wheel velocity (m/s)
 */
void diff_drive_kinematics(float v_linear, float omega_angular_cmd, float omega_angular_measured, float dt, float &v_left, float &v_right) {
  // Compute angular velocity error
  float angular_error = omega_angular_cmd - omega_angular_measured;
  
  // Reset integral if not commanding motion (prevent drift)
  if (abs(omega_angular_cmd) < 0.01) {
    angular_error_integral = 0.0;
  } else {
    // Accumulate error for integral term
    angular_error_integral += angular_error * dt;
    // Clamp integral to prevent unbounded growth
    angular_error_integral = constrain(angular_error_integral, -max_angular_integral, max_angular_integral);
  }
  
  // PI correction
  float angular_correction = angle_k_p * angular_error + angle_k_i * angular_error_integral;
  
  // Effective angular velocity includes commanded value plus correction
  float omega_effective = omega_angular_cmd + angular_correction;
  
  // Compute wheel velocities using corrected angular velocity
  v_left = v_linear - (omega_effective * track_width / 2.0);
  v_right = v_linear + (omega_effective * track_width / 2.0);
}

/**
 * Convert linear and angular velocity to individual wheel velocities.
 * Open-loop version without feedback (for compatibility).
 * @param v_linear Linear velocity (m/s), positive = forward
 * @param omega_angular Angular velocity (rad/s), positive = counter-clockwise
 * @param v_left Output left wheel velocity (m/s)
 * @param v_right Output right wheel velocity (m/s)
 */
void diff_drive_kinematics_open_loop(float v_linear, float omega_angular, float &v_left, float &v_right) {
  v_left = v_linear - (omega_angular * track_width / 2.0);
  v_right = v_linear + (omega_angular * track_width / 2.0);
}

/**
 * Set twist velocity targets with optional acceleration specification.
 * Call this from autonomous modes to specify desired linear and angular velocities.
 * The actual control loop runs separately in the main loop.
 * 
 * @param v_linear Linear velocity (m/s), positive = forward
 * @param omega_angular Angular velocity (rad/s), positive = counter-clockwise (left turn)
 * @param accel_linear Expected linear acceleration (m/s²):
 *                     - 0.0 = no explicit intent, apply internal ramping (default)
 *                     - Non-zero = explicit acceleration for feedforward
 * @param accel_angular Expected angular acceleration (rad/s²), similar behavior
 */
void set_twist_target(float v_linear, float omega_angular, 
                      float accel_linear = 0.0, 
                      float accel_angular = 0.0) {
  twist_target_linear = v_linear;
  twist_target_angular = omega_angular;
  twist_target_accel_linear = accel_linear;
  twist_target_accel_angular = accel_angular;
}

/**
 * Enable twist control.
 * Resets PI controllers and sets up for velocity control.
 * Call this in mode begin() methods.
 */
void enable_twist_control() {
  twist_control_enabled = true;
  twist_target_linear = 0.0;
  twist_target_angular = 0.0;
  twist_target_accel_linear = 0.0;
  twist_target_accel_angular = 0.0;
  // Reset ramped state to prevent windup from previous session
  twist_ramped_linear = 0.0;
  twist_ramped_angular = 0.0;
  left_wheel_controller.reset();
  right_wheel_controller.reset();
  angular_error_integral = 0.0;
}

/**
 * Disable twist control and stop motors.
 * Resets all controller state and stops motors safely.
 * Call this in mode end() methods.
 */
void disable_twist_control() {
  twist_control_enabled = false;
  twist_target_linear = 0.0;
  twist_target_angular = 0.0;
  twist_target_accel_linear = 0.0;
  twist_target_accel_angular = 0.0;
  // Reset ramped state to prevent windup when re-enabled
  twist_ramped_linear = 0.0;
  twist_ramped_angular = 0.0;
  left_wheel_controller.reset();
  right_wheel_controller.reset();
  angular_error_integral = 0.0;
  // Safety: stop motors
  left_motor.go(0);
  right_motor.go(0);
}

/**
 * Update twist control loop.
 * Reads from twist_target_linear and twist_target_angular globals.
 * Runs PI controllers and applies motor commands.
 * Called automatically from main loop at 10ms intervals when twist_control_enabled.
 */
void update_twist_control() {
  // Deadband: if targets are near zero, just stop motors and don't run control
  // This prevents small movements from sensor noise when no command is given
  const float target_deadband = 0.01; // 0.01 m/s or 0.01 rad/s
  if (abs(twist_target_linear) < target_deadband && abs(twist_target_angular) < target_deadband) {
    left_motor.go(0);
    right_motor.go(0);
    // Also reset ramped state to prevent jump when command arrives
    twist_ramped_linear = 0.0;
    twist_ramped_angular = 0.0;
    return;
  }
  
  // Measure actual time step for accurate integration
  static unsigned long last_call_millis = 0;
  unsigned long current_millis = millis();
  float dt = 0.01; // Default to 10ms if first call
  
  if (last_call_millis > 0) {
    dt = (current_millis - last_call_millis) / 1000.0; // Convert ms to seconds
    // Sanity check: clamp dt to reasonable range (1-100ms)
    dt = constrain(dt, 0.001, 0.100);
  }
  last_call_millis = current_millis;
  
  // Apply ramping if acceleration not explicitly specified
  // Use global ramped state (reset when control disabled)
  float effective_linear_accel = twist_target_accel_linear;
  float effective_angular_accel = twist_target_accel_angular;
  
  // Linear velocity ramping
  if (twist_target_accel_linear == 0.0) {
    // No explicit acceleration - apply internal ramping
    float linear_error = twist_target_linear - twist_ramped_linear;
    float max_change = max_linear_accel * dt;
    
    if (abs(linear_error) <= max_change) {
      twist_ramped_linear = twist_target_linear;
      effective_linear_accel = linear_error / dt;  // Actual acceleration applied
    } else {
      twist_ramped_linear += (linear_error > 0) ? max_change : -max_change;
      effective_linear_accel = (linear_error > 0) ? max_linear_accel : -max_linear_accel;
    }
  } else {
    // Explicit acceleration specified - use directly
    twist_ramped_linear = twist_target_linear;
    effective_linear_accel = twist_target_accel_linear;
  }
  
  // Angular velocity ramping
  if (twist_target_accel_angular == 0.0) {
    // No explicit acceleration - apply internal ramping
    float angular_error = twist_target_angular - twist_ramped_angular;
    float max_change = max_angular_accel * dt;
    
    if (abs(angular_error) <= max_change) {
      twist_ramped_angular = twist_target_angular;
      effective_angular_accel = angular_error / dt;  // Actual acceleration applied
    } else {
      twist_ramped_angular += (angular_error > 0) ? max_change : -max_change;
      effective_angular_accel = (angular_error > 0) ? max_angular_accel : -max_angular_accel;
    }
  } else {
    // Explicit acceleration specified - use directly
    twist_ramped_angular = twist_target_angular;
    effective_angular_accel = twist_target_accel_angular;
  }
  
  // Get measured angular velocity from BNO055 gyroscope (Z-axis)
  // bno_gyro_dps is now read in i2c_sensor_thread - read with mutex protection
  // Convert from deg/s to rad/s
  float measured_angular_vel;
  xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
  measured_angular_vel = bno_gyro_dps.z() * (M_PI / 180.0);
  xSemaphoreGive(sensor_data_mutex);
  
  // Apply differential drive kinematics with angular velocity PI feedback
  // Use ramped targets for smooth motion
  float v_left_target, v_right_target;
  diff_drive_kinematics(twist_ramped_linear, twist_ramped_angular, measured_angular_vel, dt, v_left_target, v_right_target);
  
  // Export effective values for logging
  twist_effective_linear = twist_ramped_linear;
  twist_effective_angular = twist_ramped_angular;
  twist_effective_accel_linear = effective_linear_accel;
  twist_effective_accel_angular = effective_angular_accel;
  v_left_target_effective = v_left_target;
  v_right_target_effective = v_right_target;
  
  // Get actual velocities from speedometers
  float v_left_actual = left_speedometer.get_velocity();
  float v_right_actual = right_speedometer.get_velocity();
  
  // Compute linear accelerations for each wheel from differential drive
  // accel_left/right ≈ effective_linear_accel ± (effective_angular_accel * track_width / 2)
  float accel_left = effective_linear_accel - (effective_angular_accel * track_width / 2.0);
  float accel_right = effective_linear_accel + (effective_angular_accel * track_width / 2.0);
  
  // Compute voltage commands via PI controllers with acceleration feedforward
  float v_left_cmd = left_wheel_controller.compute(v_left_target, v_left_actual, dt, accel_left);
  float v_right_cmd = right_wheel_controller.compute(v_right_target, v_right_actual, dt, accel_right);
  
  // Convert voltage to PWM
  float pwm_left = 0.0;
  float pwm_right = 0.0;
  
  if (!isnan(v_bat) && v_bat > 0.1) {
    pwm_left = v_left_cmd / v_bat;
    pwm_right = v_right_cmd / v_bat;
    
    // Clamp to valid PWM range [-1, 1]
    pwm_left = constrain(pwm_left, -1.0, 1.0);
    pwm_right = constrain(pwm_right, -1.0, 1.0);
  }
  
  // Apply motor commands
  left_motor.go(pwm_left);
  right_motor.go(pwm_right);
}

//////////////////////////////////
// Finite state machine

class CmdVelAutoMode : public Task {
 public:
  CmdVelAutoMode() {
    name = "auto";
  }

  virtual void begin() override {
    logf("cmd_vel auto mode begin");
    enable_twist_control();
  }

  virtual void execute() override {
    // Check if we've received a recent cmd_vel message
    unsigned long time_since_last_cmd = millis() - cmd_vel_last_received_ms;
    bool have_recent_cmd = cmd_vel_last_received_ms > 0 && time_since_last_cmd < cmd_vel_timeout_ms;
    
    // Track state transitions for logging
    static bool was_following_cmd = false;
    
    if (have_recent_cmd) {
      // Log transition from not following to following
      if (!was_following_cmd) {
        logf("cmd_vel: now following commands");
        was_following_cmd = true;
      }
      
      // We have a recent cmd_vel, set the target
      set_twist_target(cmd_vel_linear, cmd_vel_angular);
    } else {
      // Log transition from following to not following
      if (was_following_cmd) {
        logf("cmd_vel: stopped - no recent messages (%lu ms since last)", time_since_last_cmd);
        was_following_cmd = false;
      }
      
      // Timeout - set zero velocity target
      set_twist_target(0.0, 0.0);
    }
  }

  virtual void end() override {
    logf("cmd_vel auto mode end");
    disable_twist_control();
  }

  virtual void get_display_string(char * buffer, int buffer_size) override {
    unsigned long time_since_last_cmd = millis() - cmd_vel_last_received_ms;
    if (cmd_vel_last_received_ms > 0 && time_since_last_cmd < cmd_vel_timeout_ms) {
      snprintf(buffer, buffer_size, "v%.1f w%.1f", cmd_vel_linear, cmd_vel_angular);
    } else {
      snprintf(buffer, buffer_size, "Auto: wait");
    }
  }

} cmd_vel_auto_mode;

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
    logf("off mode begin");
  }

  // implement the execute method
  virtual void execute() override {
    left_motor.go(0);
    right_motor.go(0);
  }

} off_mode;

/*
Open-loop characterization mode for Step 1 of speed control implementation.
Commands fixed voltages (2V, 4V, 6V, 8V, 10V) and logs commanded voltage
and steady-state velocity to ROS for feedforward model fitting.
*/
class OpenLoopCharacterizationMode : public Task {
 public:
  // Voltage sequence to test
  const float test_voltages[5] = {2.0, 4.0, 6.0, 8.0, 10.0};
  int current_voltage_index = 0;
  
  // Timing for settling and measurement
  unsigned long state_start_time = 0;
  const unsigned long settle_time_ms = 2000;  // Wait 2s for velocity to stabilize
  const unsigned long measure_time_ms = 1000; // Measure for 1s at steady state
  
  enum State {
    SETTLING,
    MEASURING,
    COMPLETE
  } state = SETTLING;
  
  // Accumulate measurements during MEASURING state
  float left_velocity_sum = 0;
  float right_velocity_sum = 0;
  int measurement_count = 0;
  
  OpenLoopCharacterizationMode() {
    name = "open-loop";
  }

  virtual void begin() override {
    logf("Open-loop characterization mode begin");
    current_voltage_index = 0;
    state = SETTLING;
    state_start_time = millis();
    left_velocity_sum = 0;
    right_velocity_sum = 0;
    measurement_count = 0;
  }

  // Convert voltage command to PWM duty cycle [-1, 1]
  float voltage_to_pwm(float voltage) {
    if (isnan(v_bat) || v_bat < 0.1) {
      logf("ERROR: Invalid battery voltage: %0.2f", v_bat);
      return 0.0;
    }
    float pwm = voltage / v_bat;
    // Clamp to valid range
    if (pwm > 1.0) pwm = 1.0;
    if (pwm < -1.0) pwm = -1.0;
    return pwm;
  }

  virtual void execute() override {
    // Check if we've completed all voltages
    if (current_voltage_index >= 5) {
      logf("Open-loop characterization complete!");
      set_done();
      return;
    }
    
    float target_voltage = test_voltages[current_voltage_index];
    float pwm_command = voltage_to_pwm(target_voltage);
    
    unsigned long elapsed = millis() - state_start_time;
    
    switch (state) {
      case SETTLING:
        // Apply voltage command to both motors
        left_motor.go(pwm_command);
        right_motor.go(pwm_command);
        
        if (elapsed >= settle_time_ms) {
          logf("Settling complete for %0.1fV, starting measurements", target_voltage);
          state = MEASURING;
          state_start_time = millis();
          left_velocity_sum = 0;
          right_velocity_sum = 0;
          measurement_count = 0;
        }
        break;
        
      case MEASURING:
        // Continue applying voltage command
        left_motor.go(pwm_command);
        right_motor.go(pwm_command);
        
        // Accumulate velocity measurements
        left_velocity_sum += left_speedometer.get_velocity();
        right_velocity_sum += right_speedometer.get_velocity();
        measurement_count++;
        
        if (elapsed >= measure_time_ms) {
          // Calculate average velocities
          float left_velocity_avg = left_velocity_sum / measurement_count;
          float right_velocity_avg = right_velocity_sum / measurement_count;
          
          logf("Voltage: %0.2fV, Left velocity: %0.3f m/s, Right velocity: %0.3f m/s", 
               target_voltage, left_velocity_avg, right_velocity_avg);
          
          // Move to next voltage
          current_voltage_index++;
          state = SETTLING;
          state_start_time = millis();
        }
        break;
        
      case COMPLETE:
        // Should not reach here, but stop motors just in case
        left_motor.go(0);
        right_motor.go(0);
        break;
    }
  }

  virtual void end() override {
    logf("Open-loop characterization mode end");
    left_motor.go(0);
    right_motor.go(0);
  }

  virtual void get_display_string(char * buffer, int buffer_size) override {
    if (current_voltage_index < 5) {
      snprintf(buffer, buffer_size, "OL: %0.1fV %s", 
               test_voltages[current_voltage_index],
               state == SETTLING ? "settling" : "measuring");
    } else {
      snprintf(buffer, buffer_size, "OL: Complete");
    }
  }

} open_loop_characterization_mode;

/*
PI Control Test Mode for Step 2 of speed control implementation.
Tests the PI controller with feedforward by commanding a sequence of velocities
and logging commanded vs actual velocities to ROS for validation.
*/
class PIControlTestMode : public Task {
 public:
  // Test sequence: linear velocity commands (m/s)
  // Format: {v_linear, omega_angular, duration_ms}
  struct TestCommand {
    float v_linear;       // m/s
    float omega_angular;  // rad/s
    unsigned long duration_ms;
    const char* description;
  };
  
  const TestCommand test_sequence[8] = {
    {0.0, 0.0, 2000, "Stopped"},
    {0.5, 0.0, 5000, "Slow forward"},
    {1.0, 0.0, 5000, "Fast forward"},
    {0.5, 0.0, 3000, "Decelerate"},
    {0.0, 0.0, 2000, "Stop"},
    {-0.5, 0.0, 3000, "Reverse"},
    {0.0, 3.0, 3000, "Rotate left"},
    {0.0, -3.0, 3000, "Rotate right"}
  };
  
  int current_command_index = 0;
  unsigned long command_start_time = 0;
  
  PIControlTestMode() {
    name = "pi-test";
  }

  virtual void begin() override {
    logf("PI Control Test Mode begin");
    current_command_index = 0;
    command_start_time = millis();
    
    // Enable twist control (resets controllers automatically)
    enable_twist_control();
  }

  virtual void execute() override {
    // Check if test sequence is complete
    if (current_command_index >= 8) {
      logf("PI Control Test complete!");
      set_twist_target(0.0, 0.0);  // Stop
      set_done();
      return;
    }
    
    const TestCommand& cmd = test_sequence[current_command_index];
    unsigned long elapsed = millis() - command_start_time;
    
    // Execute current command
    set_twist_target(cmd.v_linear, cmd.omega_angular);
    
    // Log data for analysis (already happening in main loop via ROS)
    // The Update message includes velocities, PWM commands, battery voltage, etc.
    
    // Check if it's time to move to next command
    if (elapsed >= cmd.duration_ms) {
      logf("Command complete: %s (v=%0.2f, omega=%0.2f)", 
           cmd.description, cmd.v_linear, cmd.omega_angular);
      current_command_index++;
      command_start_time = millis();
    }
  }

  virtual void end() override {
    logf("PI Control Test Mode end");
    disable_twist_control();
  }

  virtual void get_display_string(char * buffer, int buffer_size) override {
    if (current_command_index < 8) {
      const TestCommand& cmd = test_sequence[current_command_index];
      snprintf(buffer, buffer_size, "PI: %s", cmd.description);
    } else {
      snprintf(buffer, buffer_size, "PI: Complete");
    }
  }

} pi_control_test_mode;


/*
this task will rotate the robot in place until is pointing
to the center of a can placed within 3 feet of the robot.
*/

class FindCanMode : public Task {
public:
  float rotate_throttle = 0.25;

  FindCanMode() {
    name = "find-can";
  }

  void begin() override {
    // randomly turn left or right at every attempt
    bool rotate_left = ((millis() % 2) == 1);


    logf("find can mode begin (rotating %s)", rotate_left ? "left" : "right");

    enable_twist_control();

    float rotation_radians_per_second = rotate_left ? 2.0 : -2.0;
    float velocity = 0.0;

    set_twist_target(velocity, rotation_radians_per_second);

  }

  void execute() override {
    // Read TOF distances with mutex protection
    float tof_left_dist, tof_center_dist, tof_right_dist;
    xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
    tof_left_dist = tof_left.distance;
    tof_center_dist = tof_center.distance;
    tof_right_dist = tof_right.distance;
    xSemaphoreGive(sensor_data_mutex);

    float distance = std::min({nan_to_max(tof_left_dist), nan_to_max(tof_center_dist), nan_to_max(tof_right_dist)});
    // turn left
    logf("distance: %.2f", distance);

    if (distance < contest_max_can_distance) {
      set_done();
      logf("found the can, ending mode");
    }
  }

  void end() override {

    disable_twist_control();
    
    left_motor.go(0);
    right_motor.go(0);
    logf("find-can mode complete");
  }
} find_can_mode;

float velocity_for_stop_distance(float distance, float accel) {
  return sqrt(2 * fabs(distance) * fabs(accel));
}


class GoToCanMode : public Task {
public:
  const float goal_distance = 0.05;
  float last_seen_millis = 0;
  float max_approach_velocity = 2.0;
  float max_angular_velocity = 0.5;
  float max_accel = 2.5;
  float last_ms = millis();
  float last_v = 0.0;

  GoToCanMode() {
    name = "go-to-can";
  }

  void begin() override {
    last_seen_millis = millis();
    logf("go-to-can mode begin");
    enable_twist_control();
    last_ms = millis();
    last_v = 0.0;

  }

  void execute() override {
    // Read TOF distances with mutex protection
    float right_distance, left_distance, center_distance;
    xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
    right_distance = nan_to_max(tof_right.distance);
    left_distance = nan_to_max(tof_left.distance);
    center_distance = nan_to_max(tof_center.distance);
    xSemaphoreGive(sensor_data_mutex);
    
    uint32_t time_since_seen;

    float min_distance = std::min({center_distance, left_distance, right_distance});
    if (min_distance < contest_max_can_distance) {
      last_seen_millis = millis();
      time_since_seen = 0;
    } else {
      time_since_seen = millis() - last_seen_millis;
    }
    logf("distance to can %.2f", min_distance);


    const float steering_ratio = 1.5;
    const uint32_t can_lost_timout_ms = 2000;


    float stop_velocity = velocity_for_stop_distance(min_distance-goal_distance, max_accel);

    float accel_velocity = last_v + (millis() - last_ms) / 1000.0f * max_accel;

    float approach_velocity = std::min<float>({stop_velocity, accel_velocity, max_approach_velocity});

    // if any distance is close enough, we are done
    if (min_distance <= goal_distance ) {
      set_done();
      logf("we are close enough to the can");
    } else if (time_since_seen > can_lost_timout_ms) {
        logf("lost the can, giving up");
        set_done("lost-can");
    } else if (center_distance == min_distance) {
      // if center is the min distance, go there
      set_twist_target( approach_velocity, 0, max_accel);
    } else if (right_distance == min_distance) {
      set_twist_target( approach_velocity, -max_angular_integral, max_accel);
    } else {
      // left distance must be the min, go there
      set_twist_target( approach_velocity, max_angular_integral, max_accel);
    }

  }

  void end() override {
    disable_twist_control();
    left_motor.go(0);
    right_motor.go(0);
    logf("go-to-can mode complete");
  }

} go_to_can_mode;

class WaypointFollowingMode : public Task {
 public:
  bool unsticking = false;
  float meters_to_next_waypoint = 0;
  float unstick_left_start_meters = NAN;
  float unstick_right_start_meters = NAN;

  WaypointFollowingMode() {
    name = "waypoint-following";
  }
  int step = 0;

  void begin() override {
    logf("waypoint-following mode begin");
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
        set_done();
        Serial.write(", done with route\n");
      } else {
        Serial.printf("going to waypoint %d\n", step);
      }
    }
  }

  void end() override {
    Serial.write("waypoint-following mode end\n");
    step = 0;
  }

} waypoint_following_mode;

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
    &open_loop_characterization_mode,
    &pi_control_test_mode,
    &cmd_vel_auto_mode,
    &waypoint_following_mode,
    &failsafe_mode,
    &calibrate_compass_mode,
    &find_can_mode,
    &go_to_can_mode
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
    Fsm::Edge("hand", "sd-click", "find-can"),
    Fsm::Edge("find-can", "rc-moved", "hand"),
    Fsm::Edge("find-can", "done", "go-to-can"),
    Fsm::Edge("go-to-can", "done", "hand"),
    Fsm::Edge("go-to-can", "lost-can", "find-can"),
    Fsm::Edge("go-to-can", "rc-moved", "hand"),
    //Fsm::Edge("hand", "sd-click", "open-loop"),
    //Fsm::Edge("hand", "sd-click", "pi-test"),
    Fsm::Edge("pi-test", "done", "hand"),
    Fsm::Edge("pi-test", "rc-moved", "hand"),
    Fsm::Edge("open-loop", "done", "hand"),
    Fsm::Edge("open-loop", "rc-moved", "hand"),
    Fsm::Edge("*", "off", "off"),
};

Fsm fsm(tasks, edges);

void i2c_sensor_thread(void *arg) {
  // Register this task with watchdog
  esp_task_wdt_add(NULL);
  
  Serial.printf("[%lu ms] I2C sensor thread started on core %d\n", millis(), xPortGetCoreID());
  
  unsigned long last_tof_ms = 0;
  unsigned long last_bno_ms = 0;
  unsigned long last_compass_ms = 0;
  int tof_sensor_index = 0;
  
  while (true) {
    // Small delay to yield to other tasks, 
    // put before time so it doesn't count
    // against our stats
    delay(1);

    BlockTimer bt(i2c_thread_stats);  // Measure full thread iteration
    
    // Feed watchdog
    esp_task_wdt_reset();
    
    unsigned long now = millis();
    
    // TOF sensors - staggered reads
    // Read one sensor every ~7ms to spread I2C load
    if (now - last_tof_ms >= 7) {
      BlockTimer bt(tof_stats);  // Measure TOF I2C operation time
      
      if (tof_sensor_index < tof_sensors.size()) {
        auto tof = tof_sensors[tof_sensor_index];
        auto sensor = tof->sensor;
        
        // Read from I2C (no mutex needed for Wire)
        if (sensor->read(false)) {
          float new_distance;
          if (sensor->ranging_data.range_status == VL53L1X::RangeValid) {
            new_distance = sensor->ranging_data.range_mm / 1000.0;
          } else {
            new_distance = std::numeric_limits<float>::quiet_NaN();
          }
          
          // Quick mutex-protected write
          xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
          tof->distance = new_distance;
          xSemaphoreGive(sensor_data_mutex);
        }
        
        tof_sensor_index++;
        if (tof_sensor_index >= tof_sensors.size()) {
          tof_sensor_index = 0;
        }
      }
      last_tof_ms += 7;  // Fixed interval to prevent drift
    }
    
    // BNO055 - read every 10ms
    if (now - last_bno_ms >= 10) {
      BlockTimer bt(bno_stats);  // Measure BNO055 I2C operation time
      
      // Read from I2C (no mutex needed for Wire)
      imu::Vector<3> orientation = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
      imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
      imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
      imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
      float heading = normalize_compass_degrees(orientation.x() + 90 + magnetic_declination_degrees);
      
      // Quick mutex-protected write of all BNO data
      xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
      bno_orientation_degrees = orientation;
      bno_acceleration = accel;
      bno_gyro_dps = gyro;
      bno_mag = mag;
      compass_heading_degrees = heading;
      xSemaphoreGive(sensor_data_mutex);
      
      last_bno_ms += 10;  // Fixed interval to prevent drift
    }
    
    // Compass - read every 100ms
    if (now - last_compass_ms >= 100) {
      BlockTimer bt(compass_stats);  // Measure compass I2C operation time
      
      // Read from I2C (no mutex needed for Wire)
      // compass.update() modifies compass.last_reading directly
      // Note: compass.last_reading is a simple struct of 3 ints - reads/writes are atomic enough
      // for our purposes (worst case: one corrupted sample every 100ms during race condition)
      compass.update();
      
      last_compass_ms += 100;  // Fixed interval to prevent drift
    }
    
  }
}

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
        
        // Publish crash data once after connecting to ROS
        if (!crash_data_published_to_ros) {
          publish_crash_data_to_ros();
          crash_data_published_to_ros = true;
        }
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

    // spin executor to process subscriptions
    if (ros_ready) {
      rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
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

bool save_bno055_calibration_to_spiffs(const adafruit_bno055_offsets_t& offsets) {
  File file = SPIFFS.open(bno055_calibration_file_path, FILE_WRITE);
  if (!file) {
    logf("Failed to open BNO055 calibration file for writing");
    return false;
  }

  // Write all 11 offset values as comma-separated integers
  file.printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
    offsets.accel_offset_x, offsets.accel_offset_y, offsets.accel_offset_z,
    offsets.mag_offset_x, offsets.mag_offset_y, offsets.mag_offset_z,
    offsets.gyro_offset_x, offsets.gyro_offset_y, offsets.gyro_offset_z,
    offsets.accel_radius, offsets.mag_radius);
  
  file.close();
  logf("BNO055 calibration saved to SPIFFS");
  return true;
}

bool load_bno055_calibration_from_spiffs() {
  if (!SPIFFS.exists(bno055_calibration_file_path)) {
    logf("No BNO055 calibration file found");
    return false;
  }

  File file = SPIFFS.open(bno055_calibration_file_path, FILE_READ);
  if (!file) {
    logf("Failed to open BNO055 calibration file");
    return false;
  }

  String content = file.readStringUntil('\n');
  file.close();

  // Parse comma-separated values - expecting 11 int16_t values
  int commas[10];
  commas[0] = content.indexOf(',');
  for (int i = 1; i < 10; i++) {
    commas[i] = content.indexOf(',', commas[i-1] + 1);
    if (commas[i] == -1) {
      logf("Invalid BNO055 calibration format - missing comma %d", i);
      return false;
    }
  }

  adafruit_bno055_offsets_t offsets;
  offsets.accel_offset_x = content.substring(0, commas[0]).toInt();
  offsets.accel_offset_y = content.substring(commas[0] + 1, commas[1]).toInt();
  offsets.accel_offset_z = content.substring(commas[1] + 1, commas[2]).toInt();
  offsets.mag_offset_x = content.substring(commas[2] + 1, commas[3]).toInt();
  offsets.mag_offset_y = content.substring(commas[3] + 1, commas[4]).toInt();
  offsets.mag_offset_z = content.substring(commas[4] + 1, commas[5]).toInt();
  offsets.gyro_offset_x = content.substring(commas[5] + 1, commas[6]).toInt();
  offsets.gyro_offset_y = content.substring(commas[6] + 1, commas[7]).toInt();
  offsets.gyro_offset_z = content.substring(commas[7] + 1, commas[8]).toInt();
  offsets.accel_radius = content.substring(commas[8] + 1, commas[9]).toInt();
  offsets.mag_radius = content.substring(commas[9] + 1).toInt();

  bno.setSensorOffsets(offsets);
  logf("BNO055 calibration loaded from SPIFFS");
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

void gnss_init_thread(void *parameter) {
  Serial.printf("[%lu ms] GNSS init thread started\n", millis());
  
  Serial.printf("[%lu ms] Initializing GNSS module\n", millis());
  if (gnss.begin(gnss_serial, 2000, true)) {
    Serial.printf("[%lu ms] GNSS module detected\n", millis());
    
    // Configure message output
    Serial.printf("[%lu ms] Setting UART1 output\n", millis());
    gnss.setUART1Output(COM_TYPE_NMEA);
    Serial.printf("[%lu ms] Enabling NMEA messages\n", millis());
    gnss.enableNMEAMessage(UBX_NMEA_RMC, COM_PORT_UART1);  // Time and date
    gnss.enableNMEAMessage(UBX_NMEA_GGA, COM_PORT_UART1);  // Fix data
    gnss.enableNMEAMessage(UBX_NMEA_ZDA, COM_PORT_UART1);  // Precise time/date
    Serial.printf("[%lu ms] NMEA messages enabled\n", millis());

    // Enable multiple GNSS constellations
    Serial.printf("[%lu ms] Enabling GNSS constellations\n", millis());
    gnss.enableGNSS(true, SFE_UBLOX_GNSS_ID_GPS);     // GPS USA
    gnss.enableGNSS(true, SFE_UBLOX_GNSS_ID_GALILEO); // Galileo Europe
    gnss.enableGNSS(true, SFE_UBLOX_GNSS_ID_BEIDOU);  // BeiDou China
    gnss.enableGNSS(true, SFE_UBLOX_GNSS_ID_GLONASS); // GLONASS Russia
    Serial.printf("[%lu ms] GNSS constellations enabled\n", millis());

    // Configure time base and update rate
    Serial.printf("[%lu ms] Configuring GNSS measurement rate\n", millis());
    gnss.setMeasurementRate(100);     // Measurements every 100ms
    gnss.setNavigationFrequency(10);  // Navigation rate 10Hz

    // Save configuration
    Serial.printf("[%lu ms] Saving GNSS configuration\n", millis());
    bool success = gnss.saveConfiguration();
    Serial.printf("[%lu ms] GPS configuration %s\n", millis(), success ? "saved" : "failed to save");
  }
  else {
    Serial.printf("[%lu ms] GPS failed to initialize\n", millis());
  }
  
  Serial.printf("[%lu ms] GNSS init thread completed\n", millis());
  vTaskDelete(NULL);
}

//////////////////////////////////
// Diagnostic Initialization

void init_diagnostics() {
  esp_reset_reason_t reason = esp_reset_reason();
  
  Serial.println("\n\n=== WHITE CRASH BOOT ===");
  
  // Initialize or increment boot count FIRST (before loading flash data)
  if (diag.magic == DIAG_MAGIC) {
    diag.boot_count++;
  } else {
    // First boot ever - initialize
    diag.magic = DIAG_MAGIC;
    diag.boot_count = 1;
    diag.consecutive_wdt_resets = 0;
    strncpy(diag.last_location, "power_on", sizeof(diag.last_location));
  }
  
  Serial.printf("Boot #%lu\n", diag.boot_count);
  
  // On power-on reset, check if we have crash data in flash from previous power cycle
  if (reason == ESP_RST_POWERON) {
    load_diagnostics_from_flash();
  }
  
  // Print reset reason
  Serial.printf("Reset reason: ");
  switch(reason) {
    case ESP_RST_POWERON:   Serial.println("Power-on"); break;
    case ESP_RST_SW:        Serial.println("Software reset"); break;
    case ESP_RST_PANIC:     Serial.println("Exception/panic"); break;
    case ESP_RST_INT_WDT:   Serial.println("Interrupt watchdog"); break;
    case ESP_RST_TASK_WDT:  Serial.println("TASK WATCHDOG TIMEOUT"); break;
    case ESP_RST_WDT:       Serial.println("Other watchdog"); break;
    case ESP_RST_BROWNOUT:  Serial.println("Brownout"); break;
    default:                Serial.printf("Unknown (%d)\n", reason); break;
  }
  
  if (reason == ESP_RST_TASK_WDT || reason == ESP_RST_INT_WDT) {
    diag.consecutive_wdt_resets++;
    Serial.printf("\n*** WATCHDOG RESET #%lu ***\n", diag.consecutive_wdt_resets);
    Serial.printf("*** SYSTEM HUNG AT: %s ***\n\n", diag.last_location);
    
    // Save crash data to flash so it survives power cycles
    save_diagnostics_to_flash();
    Serial.println("Crash data saved to flash\n");
    
    // Take action after repeated hangs at same location
    if (diag.consecutive_wdt_resets > 3) {
      Serial.printf("\n!!! REPEATED HANGS AT %s !!!\n", diag.last_location);
      Serial.println("!!! CONSIDER DISABLING THIS SENSOR !!!\n");
    }
  } else {
    diag.consecutive_wdt_resets = 0;
  }
  
  diag.last_reset = reason;
  Serial.println("=== BOOT COMPLETE ===\n");
}


//////////////////////////////////
// Main setup and loop

void setup() {

  // ensure power is turned off for all the tof sensors
  for (auto pin : {pin_left_tof_power, pin_center_tof_power, pin_right_tof_power}) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }


  // Initialize serial first so diagnostics can print
  Serial.begin(3000000);
  // note: it takes about 1.8 seconds after boot for serial messages to show in platformio
  delay(2000);  // Wait for serial to be ready

  // Initialize diagnostics and print boot information
  init_diagnostics();

  // Configure hardware watchdog timer (3 second timeout)
  // Using RTC_NOINIT_ATTR preserves diagnostic data even with panic=true
  esp_task_wdt_init(3, true);  // 3 seconds, panic on timeout
  esp_task_wdt_add(NULL);      // Add current task to watchdog
  esp_core_dump_init(); // enable core dump
  Serial.printf("[%lu ms] Watchdog timer configured (3 second timeout)\n", millis());

#ifdef pin_built_in_led
  pinMode(pin_built_in_led, OUTPUT);
  digitalWrite(pin_built_in_led, HIGH);
#endif
  Serial.printf("[%lu ms] Setup starting\n", millis());


  // int i = 0;
  // do {
  //   Serial.printf("white-crash has been running for %.1f seconds\n", i / 10.0);
  //   Serial.flush();
  //   delay(100);
  //   ++i;
  // } while(true);

  Serial.printf("[%lu ms] Starting SPIFFS\n", millis());
  SPIFFS.begin();
  Serial.printf("[%lu ms] SPIFFS initialized\n", millis());


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

  Serial.printf("[%lu ms] Log messages allocated\n", millis());
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

  Serial.printf("[%lu ms] Initializing I2C\n", millis());
  Wire.setPins(pin_sda, pin_scl);
  Wire.setClock(400000);
  Wire.begin();
  Wire.setTimeOut(5); // ms
  Serial.printf("[%lu ms] I2C initialized\n", millis());

  // Initialize sensor data mutex
  Serial.printf("[%lu ms] Creating sensor data mutex\n", millis());
  sensor_data_mutex = xSemaphoreCreateMutex();
  if (sensor_data_mutex == NULL) {
    Serial.println("ERROR: Failed to create sensor_data_mutex");
    while(1);
  }
  Serial.printf("[%lu ms] Sensor data mutex created\n", millis());


  Serial.printf("Starting BNO055\n");
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.println("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  Serial.println("bno055 begin successful");

  bno.setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1);
  bno.setAxisSign(Adafruit_BNO055::REMAP_SIGN_P1);

  Serial.printf("Loading BNO055 calibration\n");
  load_bno055_calibration_from_spiffs();

  // Crystal must be configured AFTER loading calibration data into BNO055
  bno.setExtCrystalUse(true);

  //Wire1.setPins(pin_tof_sda, pin_tof_scl);
  //Wire1.begin();
  //Wire1.setTimeOut(5); // ms

  left_speedometer.meters_per_tick = meters_per_odometer_tick;
  right_speedometer.meters_per_tick = meters_per_odometer_tick;

  fsm.begin();

  Serial.write("white_crash\n");
  gnss_serial.setRxBufferSize(4096);

  Serial.printf("[%lu ms] Starting GNSS serial\n", millis());
  gnss_serial.begin(38400, SERIAL_8N1, pin_gps_rx, pin_gps_tx);
  Serial.printf("[%lu ms] GNSS serial started\n", millis());

  // Start GNSS initialization in a separate thread so it doesn't block setup
  if (use_gnss) {
    Serial.printf("[%lu ms] Starting GNSS init thread\n", millis());
    xTaskCreatePinnedToCore(
        gnss_init_thread,
        "gnss_init",
        8192,
        NULL,
        1,
        NULL,
        1);
  }

  Serial.printf("[%lu ms] Loading compass calibration\n", millis());
  if (!load_compass_calibration_from_spiffs()) {
    compass.set_calibration(-950, 675, -1510, 47, 0, 850);
  }
  Serial.printf("[%lu ms] Compass calibration loaded\n", millis());
  // see https://www.magnetic-declination.com/
  // compass.setMagneticDeclination(11, 24);

  Serial.printf("[%lu ms] Starting TOF sensors\n", millis());
  start_tof_distance_sensor(tof_left);
  start_tof_distance_sensor(tof_right);
  start_tof_distance_sensor(tof_center);
  Serial.printf("[%lu ms] TOF sensors started\n", millis());

  // quadrature encoders
  Serial.printf("[%lu ms] Setting up encoders\n", millis());

  attachInterrupt(digitalPinToInterrupt(pin_left_encoder_a), left_a_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_left_encoder_b), left_b_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_right_encoder_a), right_a_changed, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pin_right_encoder_b), right_b_changed, CHANGE);
  Serial.printf("[%lu ms] Encoders setup complete\n", millis());

  Serial.printf("[%lu ms] Starting CRSF serial\n", millis());
  crsf_serial.setRxBufferSize(500);
  crsf_serial.begin(420000, SERIAL_8N1, pin_crsf_rx, pin_crsf_tx);
  Serial.printf("[%lu ms] CRSF serial started\n", millis());

  // crsf
  Serial.printf("[%lu ms] Starting CRSF\n", millis());
  // Clear any data that accumulated in the buffer before CRSF initialization
  while(crsf_serial.available()) {
    crsf_serial.read();
  }
  Serial.printf("[%lu ms] CRSF serial buffer cleared\n", millis());
  crsf.begin();
  crsf.update();

  crsf.set_rc_callback(handle_rc_message);
  Serial.printf("[%lu ms] CRSF initialized\n", millis());

  Serial.printf("[%lu ms] Initializing motors\n", millis());
  left_motor.init(pin_left_fwd, pin_left_rev);
  right_motor.init(pin_right_fwd, pin_right_rev);
  right_motor.go(0);
  left_motor.go(0);
  Serial.printf("[%lu ms] Motors initialized\n", millis());
#ifdef pin_built_in_led
  pinMode(pin_built_in_led, OUTPUT);
  pinMode(pin_battery_voltage, INPUT);
#endif
  // The input voltage of ADC will be attenuated, extending 
  // the range of measurement to up to approx. 2600 mV. 
  // (1V input = ADC reading of 1575).
  analogSetPinAttenuation(pin_battery_voltage, ADC_11db); 
  adcAttachPin(pin_battery_voltage);
  Serial.printf("[%lu ms] ADC configured\n", millis());

  // create I2C sensor thread on Core 0
  Serial.printf("[%lu ms] Creating I2C sensor thread\n", millis());
  xTaskCreatePinnedToCore(
      i2c_sensor_thread,
      "i2c_sensors",
      8192,
      NULL,
      1,
      NULL,
      0); // Core 0 - separate from main loop
  Serial.printf("[%lu ms] I2C sensor thread created\n", millis());

  // create a thread for ros stuff
  Serial.printf("[%lu ms] Creating ROS thread\n", millis());
  if(true) {
    xTaskCreatePinnedToCore(
        ros_thread,
        "ros_thread",
        16384,
        NULL,
        1,
        NULL,
        1); // this is the same core as loop. TBD: move to core zero?
  }
  Serial.printf("[%lu ms] ROS thread created\n", millis());

#ifdef pin_built_in_led
  digitalWrite(pin_built_in_led, 0);
#endif
  Serial.printf("[%lu ms] Setup complete!\n", millis());
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
    
    // Track location in diagnostic state (RTC SRAM - fast & unlimited writes)
    strncpy(diag.last_location, name, sizeof(diag.last_location) - 1);
    diag.last_location[sizeof(diag.last_location) - 1] = '\0';
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
  // Feed watchdog timer - must be first line to prevent watchdog reset
  esp_task_wdt_reset();
  
  delay(1); // give the system a little time to breathe
  
  // Track last successful loop time in RTC memory
  diag.last_loop_ms = millis();
  BlockTimer bt(loop_stats);
  last_loop_time_ms = loop_time_ms;
  loop_time_ms = millis();

  bool every_1_ms = every_n_ms(last_loop_time_ms, loop_time_ms, 1);
  bool every_10_ms = every_n_ms(last_loop_time_ms, loop_time_ms, 10);
  bool every_100_ms = every_n_ms(last_loop_time_ms, loop_time_ms, 100);
  bool every_200_ms = every_n_ms(last_loop_time_ms, loop_time_ms, 200);
  bool every_1000_ms = every_n_ms(last_loop_time_ms, loop_time_ms, 1000);
  bool every_minute = every_n_ms(last_loop_time_ms, loop_time_ms, 60 * 1000);

  if (every_100_ms) {
    left_speedometer.update_from_sensor(micros(), left_encoder.odometer_a, left_encoder.last_odometer_a_us, left_encoder.odometer_b, left_encoder.last_odometer_b_us);
    right_speedometer.update_from_sensor(micros(), right_encoder.odometer_a, right_encoder.last_odometer_a_us, right_encoder.odometer_b, right_encoder.last_odometer_b_us);

    update_msg.battery_voltage = v_bat;
    update_msg.left_speed = left_speedometer.get_velocity();
    update_msg.right_speed = right_speedometer.get_velocity();
    update_msg.left_motor_command = left_motor.get_setpoint();
    update_msg.left_odometer_ticks = left_encoder.odometer_a;
    update_msg.right_odometer_ticks=right_encoder.odometer_a;
    update_msg.right_motor_command = right_motor.get_setpoint();
    update_msg.rx_esc = crsf_ns::crsf_rc_channel_to_float(rx_esc);
    update_msg.rx_str = crsf_ns::crsf_rc_channel_to_float(rx_str);
    
    // Read compass and BNO data with mutex protection (compass.last_reading written in i2c_sensor_thread)
    xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
    // Compute azimuth from cached compass reading instead of calling get_azimuth_degrees() which might do I2C
    update_msg.yaw_degrees = compass.get_azimuth_degrees();
    update_msg.mag_x = compass.last_reading.x;
    update_msg.mag_y = compass.last_reading.y;
    update_msg.mag_z = compass.last_reading.z;
    update_msg.bno_yaw_degrees = compass_heading_degrees;
    update_msg.bno_pitch_degrees = bno_orientation_degrees.y();
    update_msg.bno_roll_degrees = bno_orientation_degrees.z();
    update_msg.bno_acceleration_x = bno_acceleration.x();
    update_msg.bno_acceleration_y = bno_acceleration.y();
    update_msg.bno_acceleration_z = bno_acceleration.z();
    update_msg.bno_gyro_x = bno_gyro_dps.x();
    update_msg.bno_gyro_y = bno_gyro_dps.y();
    update_msg.bno_gyro_z = bno_gyro_dps.z();
    update_msg.bno_mag_x = bno_mag.x();
    update_msg.bno_mag_y = bno_mag.y();
    update_msg.bno_mag_z = bno_mag.z();
    xSemaphoreGive(sensor_data_mutex);

    // Populate twist control targets (NAN if not in twist control mode)
    if (twist_control_enabled) {
      update_msg.twist_target_linear = twist_target_linear;
      update_msg.twist_target_angular = twist_target_angular;
      update_msg.twist_target_accel_linear = twist_target_accel_linear;
      update_msg.twist_target_accel_angular = twist_target_accel_angular;
      
      // Use the effective (ramped) targets that were actually sent to the controllers
      update_msg.v_left_target = v_left_target_effective;
      update_msg.v_right_target = v_right_target_effective;
    } else {
      update_msg.twist_target_linear = NAN;
      update_msg.twist_target_angular = NAN;
      update_msg.twist_target_accel_linear = NAN;
      update_msg.twist_target_accel_angular = NAN;
      update_msg.v_left_target = NAN;
      update_msg.v_right_target = NAN;
    }

    if (ros_ready) {
      // publish update message
      RCSOFTCHECK(rcl_publish(&update_publisher, &update_msg, NULL));
    }
  }

  // BNO055 calibration check (I2C reads moved to i2c_sensor_thread)
  if (every_1000_ms) {
    static bool calibration_achieved = false;
    uint8_t system, gyro, accel, mag;
    bno.getCalibration(&system, &gyro, &accel, &mag);
//    Serial.printf("Bno Calibration status sytem: %d gyro %d accel: %d mag: %d\n", system, gyro, accel, mag);

    // save calibration constants once we are fully calibrated
    if (!calibration_achieved && system == 3 && gyro == 3 && accel == 3 && mag == 3) {
      Serial.printf("Saving bno055 calibration\n");
      adafruit_bno055_offsets_t offsets;
      if (bno.getSensorOffsets(offsets)) {
        if (save_bno055_calibration_to_spiffs(offsets)) {
          calibration_achieved = true;
        }
      }
    }
  }

  // TOF distance reading (moved to i2c_sensor_thread, ROS publishing still in main loop)
  if (every_n_ms(last_loop_time_ms, loop_time_ms, tof_timing_budget_ms)) {
    BlockTimer bt(tof_distance_stats);  // Measure TOF publishing time
    for (auto tof : tof_sensors) {
      if (ros_ready)
      {
        // Read TOF distance with mutex protection
        float tof_distance;
        xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
        tof_distance = tof->distance;
        xSemaphoreGive(sensor_data_mutex);
        
        set_stamp(tof_distance_msg.header.stamp);
        tof_distance_msg.range = tof_distance;
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

  if (use_gnss && every_10_ms) {
    BlockTimer bt(gps_stats);
    HangChecker hc("gps");
    gnss.checkUblox();
    gnss.checkCallbacks();
  }

  if (use_gps && every_10_ms) {
    BlockTimer bt(gps_stats);
    HangChecker hc("gps_serial");
    
    // Limit iterations to prevent infinite loop
    int char_count = 0;
    const int MAX_GPS_CHARS = 100;
    
    while (char_count < MAX_GPS_CHARS && gnss_serial.available()) {
      auto c = gnss_serial.read();
      gps.encode(c);
      Serial.write(c);
      char_count++;
    }
    
    if (char_count >= MAX_GPS_CHARS) {
      logf(Severity::WARN, "GPS: max chars (%d) processed, may have more buffered", char_count);
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
if (use_gnss && every_minute) {
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
    // if (millis()>30000) delay(10000); // fake hang
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
    for (auto stats : {gps_stats, log_stats, loop_stats, crsf_stats, compass_stats, telemetry_stats, serial_read_stats, crsf_parse_stats, tof_distance_stats, rc_callback_stats, bno_stats, tof_stats, i2c_thread_stats}) {
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


    const float v_bat_scale = 0.00441892139; // white-crash m    
    v_bat = v_bat_scale * sum  / sample_count;
    battery_msg.data = v_bat;
  }

  if (every_10_ms) {
    HangChecker hc("fsm");
    fsm.execute();
  }

  // Update twist control at 100Hz when enabled
  if (twist_control_enabled && every_10_ms) {
    update_twist_control();
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
    static bool last_button_sd_pressed = false;
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

    // sd event
    if (button_sd_pressed && !last_button_sd_pressed) {
      fsm.set_event("sd-click");
      logf("sd-click");
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
    last_button_sd_pressed = button_sd_pressed;
    last_rx_str = rx_str;
    last_rx_esc = rx_esc;
  }

  // Compass reading moved to i2c_sensor_thread (every 100ms)
  // compass.update() now called in i2c_sensor_thread

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
    crsf.send_attitude(bno_orientation_degrees.y(), bno_orientation_degrees.y(), compass_heading_degrees);

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

