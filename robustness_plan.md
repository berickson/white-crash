# Robustness Plan for White Crash Robot

## Problem Statement
The robot occasionally stops responding completely:
- No ROS messages
- No remote control response  
- No telemetry
- Loop appears to stop running

## Root Causes
1. **I2C hangs** - BNO055, compass, ToF sensors can block indefinitely
2. **Serial loops** - GPS serial read could loop forever if data arrives faster than processing
3. **Library blocking calls** - Third-party libraries may have hidden blocking behavior
4. **Hardware bus lockup** - I2C or SPI bus can enter stuck state

## Protection Strategy

### 1. Hardware Watchdog Timer (PRIMARY DEFENSE)
**Status:** To be implemented

The ESP32 hardware watchdog is the only reliable protection against complete hangs.

```cpp
#include <esp_task_wdt.h>

void setup() {
  // Configure watchdog for 3 second timeout
  esp_task_wdt_init(3, true);  // 3 seconds, panic on timeout
  esp_task_wdt_add(NULL);      // Add current task to watchdog
}

void loop() {
  esp_task_wdt_reset();  // Must be first line - feed the watchdog
  delay(1);
  // ...rest of loop code...
}
```

**How it works:**
- Watchdog timer counts down from 3 seconds
- `esp_task_wdt_reset()` resets the timer back to 3 seconds
- If timer reaches 0 (loop hasn't run for 3 seconds), ESP32 automatically reboots
- After reboot, robot recovers and continues operating

### 2. Diagnostic State Tracking (CRASH FORENSICS)
**Status:** To be implemented

Use ESP32 RTC memory to preserve diagnostic information across watchdog resets.

**RTC Memory Characteristics:**
- 8KB total RTC SLOW memory on ESP32-S3
- Fast SRAM (same speed as regular RAM)
- Unlimited write endurance (SRAM, not flash - no wear concerns)
- Survives watchdog resets and software resets
- Lost on power cycle or deep sleep
- Safe to write every loop iteration
- **System usage:** ~1-2KB used by WiFi calibration & system state
- **Available for diagnostics:** ~6-7KB
- **Diagnostic structure size:** ~1.5KB (plenty of room)

**Why RTC Memory:**
- Persists even if serial/ROS disconnected
- Can read diagnostics by connecting serial after crash
- Fast enough for high-frequency updates
- Large enough for diagnostic logs + circular buffer

#### Enhanced HangChecker with Persistent Diagnostics
Extend the existing HangChecker class to automatically track diagnostic state:

```cpp
// In RTC memory - survives watchdog reset (8KB available, SRAM - fast & unlimited writes)
RTC_DATA_ATTR struct DiagnosticState {
  uint32_t magic;
  uint32_t boot_count;
  uint32_t last_loop_ms;
  uint32_t consecutive_wdt_resets;
  char last_location[40];
  esp_reset_reason_t last_reset;
  
  // Circular log buffer for last N events (survives crash)
  struct LogEntry {
    uint32_t timestamp_ms;
    char location[24];
    uint16_t duration_ms;
  };
  static const int LOG_SIZE = 50;
  LogEntry log[LOG_SIZE];
  uint16_t log_index;
  
  // Statistics
  uint32_t total_hangs;
  uint32_t max_loop_time_ms;
} diag;

const uint32_t DIAG_MAGIC = 0xC0FFEE42;

// Helper to add entries to diagnostic log (safe to call frequently)
void diag_log(const char* location, uint16_t duration_ms = 0) {
  auto& entry = diag.log[diag.log_index % DiagnosticState::LOG_SIZE];
  entry.timestamp_ms = millis();
  strncpy(entry.location, location, sizeof(entry.location) - 1);
  entry.location[sizeof(entry.location) - 1] = '\0';
  entry.duration_ms = duration_ms;
  diag.log_index++;
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
    
    // Automatically track location in diagnostic state (RTC SRAM - fast & unlimited writes)
    strncpy(diag.last_location, name, sizeof(diag.last_location) - 1);
    diag.last_location[sizeof(diag.last_location) - 1] = '\0';
  }

  ~HangChecker() {
    unsigned long now = millis();
    int elapsed = now - start_ms;
    
    // Log to RTC memory circular buffer (persists through crash)
    diag_log(name, elapsed);
    
    if (elapsed > timeout_ms) {
      // Only log to serial/ROS if available, but data is in RTC memory either way
      logf(Severity::WARN, "HANG: %s took %d ms", name, elapsed);
      diag.total_hangs++;
    }
    
    // Track max loop time
    if (elapsed > diag.max_loop_time_ms) {
      diag.max_loop_time_ms = elapsed;
    }
  }
};

// Initialize diagnostics in setup()
void init_diagnostics() {
  esp_reset_reason_t reason = esp_reset_reason();
  
  // Always print diagnostics to serial (even if previous run had no serial)
  Serial.begin(3000000);
  delay(2000);  // Give serial time to connect
  
  Serial.println("\n\n=== WHITE CRASH BOOT ===");
  
  if (diag.magic == DIAG_MAGIC) {
    // Valid diagnostic data from previous run - ALWAYS print to serial
    Serial.printf("Boot #%lu, Last reset reason: ", diag.boot_count);
    
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
    
    Serial.printf("Last loop: %lu ms ago\n", millis() - diag.last_loop_ms);
    Serial.printf("Last location: %s\n", diag.last_location);
    Serial.printf("Total hangs detected: %lu\n", diag.total_hangs);
    Serial.printf("Max loop time: %lu ms\n", diag.max_loop_time_ms);
    
    if (reason == ESP_RST_TASK_WDT || reason == ESP_RST_INT_WDT) {
      diag.consecutive_wdt_resets++;
      Serial.printf("\n*** WATCHDOG RESET #%lu ***\n", diag.consecutive_wdt_resets);
      Serial.printf("*** SYSTEM HUNG AT: %s ***\n\n", diag.last_location);
      
      // Print circular log (last 50 operations before hang)
      Serial.println("Last 50 operations before hang:");
      int start = (diag.log_index > DiagnosticState::LOG_SIZE) ? 
                  (diag.log_index - DiagnosticState::LOG_SIZE) : 0;
      for (int i = start; i < diag.log_index; i++) {
        auto& entry = diag.log[i % DiagnosticState::LOG_SIZE];
        Serial.printf("  [%lu ms] %s (%u ms)\n", 
                     entry.timestamp_ms, entry.location, entry.duration_ms);
      }
      
      // Take action after repeated hangs at same location
      if (diag.consecutive_wdt_resets > 3) {
        Serial.printf("\n!!! REPEATED HANGS AT %s !!!\n", diag.last_location);
        Serial.println("!!! CONSIDER DISABLING THIS SENSOR !!!\n");
        // Could automatically disable problematic sensors here
      }
    } else {
      diag.consecutive_wdt_resets = 0;
    }
    
    diag.boot_count++;
  } else {
    // First boot or power cycle - initialize
    Serial.println("First boot - initializing diagnostics");
    diag.magic = DIAG_MAGIC;
    diag.boot_count = 1;
    diag.consecutive_wdt_resets = 0;
    diag.log_index = 0;
    diag.total_hangs = 0;
    diag.max_loop_time_ms = 0;
    strncpy(diag.last_location, "power_on", sizeof(diag.last_location));
  }
  
  diag.last_reset = reason;
  Serial.println("=== BOOT COMPLETE ===\n");
}

// Update tracking in loop()
void loop() {
  esp_task_wdt_reset();
  delay(1);
  
  diag.last_loop_ms = millis();
  
  // Existing HangChecker usage automatically updates diag.last_location
  if (every_10_ms) {
    HangChecker hc("bno055");  // Automatically sets diag.last_location = "bno055"
    bno_orientation_degrees = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    // ...
  }
}
```

**Benefits:**
- **Works without serial/ROS** - Data persists in RTC memory across crash
- **Fast & unlimited writes** - RTC is SRAM, not flash (no wear concerns)
- **Circular log buffer** - See last 50 operations before crash
- **Just connect serial after crash** - Full diagnostic info available
- **Automatic tracking** - HangChecker usage already throughout code
- **Catches intermittent issues** - Survives multiple crashes until power cycle

**How to diagnose after crash:**
1. Robot stops responding (watchdog triggers reset)
2. Robot automatically reboots
3. Connect serial terminal anytime after reboot
4. Press reset button - see full diagnostic dump including:
   - What location caused hang
   - Last 50 operations with timestamps
   - Number of consecutive watchdog resets
   - Total hangs and max loop times

### 3. Bounded Serial Loops (PREVENT INFINITE LOOPS)
**Status:** To be implemented

The GPS serial read can loop forever if data arrives faster than processing:

```cpp
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
```

### 4. Existing Protections (ALREADY IN PLACE)
- **Wire.setTimeOut(5)** - Low-level I2C timeout (5ms)
- **HangChecker** - Logs operations that take longer than expected
- **BlockTimer** - Tracks execution time statistics

## Implementation Priority

1. **Hardware Watchdog** - Critical, implement first
2. **Enhanced HangChecker with diagnostics** - High value, minimal code changes
3. **GPS serial loop limit** - Low risk, quick fix
4. **Monitor and analyze** - Review diagnostic logs after watchdog resets

## Testing Strategy

1. Deploy with watchdog enabled
2. Monitor for watchdog resets in logs
3. Review `diag.last_location` to identify problematic sensors/code sections
4. If specific sensor causes repeated hangs, add sensor-specific recovery or disable feature

## Future Enhancements

- Automatic sensor disable after N consecutive hangs at same location
- I2C bus reset capability after detection of hung bus
- Reduce I2C polling frequency for problematic sensors
- Move sensor reads to separate FreeRTOS tasks with timeouts (complex, low priority)

## Notes

- **Watchdog is the only true protection** - Everything else just reduces hang likelihood
- **RTC memory is fast SRAM** - Same speed as regular RAM, unlimited writes
- **~6-7KB available** for user data (WiFi uses ~1-2KB for calibration)
- **Diagnostic structure uses ~1.5KB** - plenty of room for expansion
- **RTC memory survives soft resets** but not power cycles or deep sleep  
- **ESP32 automatically tracks reset reason** via `esp_reset_reason()`
- **Cannot safely kill hung threads** - FreeRTOS doesn't support forced task termination
- **Three-second watchdog timeout** provides plenty of margin (loop runs every ~10ms)
- **Diagnostics work without serial/ROS** - Data persists in RTC memory until power cycle
- **Connect serial after crash** - Press reset to see full diagnostic dump
- **8KB RTC memory available** - Plenty for diagnostics + circular buffer
