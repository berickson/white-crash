# Robustness Plan for White Crash Robot

## Problem Statement
The robot occasionally stops responding completely:
- No ROS messages
- No remote control response  
- No telemetry
- Loop appears to stop running

## Possible Causes
1. **I2C hangs** - BNO055, compass, ToF sensors can block indefinitely
2. **Serial loops** - GPS serial read could loop forever if data arrives faster than processing
3. **Library blocking calls** - Third-party libraries may have hidden blocking behavior
4. **Hardware bus lockup** - I2C or SPI bus can enter stuck state

## Protection Strategy

### 1. Hardware Watchdog Timer (PRIMARY DEFENSE)
**Status:** ✅ IMPLEMENTED

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
**Status:** ✅ IMPLEMENTED

Use ESP32 RTC memory during operation to track diagnostic state, then persist to flash storage on watchdog reset so crash data survives power cycles.

**Two-Stage Diagnostic Strategy:**

**Stage 1: RTC Memory (During Operation)**
- 8KB total RTC SLOW memory on ESP32-S3
- Fast SRAM (same speed as regular RAM)
- Unlimited write endurance (SRAM, not flash - no wear concerns)
- Safe to write every loop iteration
- Tracks: last location only (simple and reliable)
- Survives watchdog resets and software resets
- Lost on power cycle or deep sleep

**Stage 2: Flash Storage (On Watchdog Reset)**
- On watchdog reset, copy RTC memory diagnostic data to flash (Preferences library)
- Flash storage survives power cycles
- Data available for retrieval anytime after crash, even after multiple power cycles
- Will be printed to serial on every boot
- Will be published as ROS warning message after ROS connection established

**Why This Approach:**
- RTC memory is fast enough for frequent updates during operation
- Flash storage ensures crash data survives power cycles
- No serial/ROS connection needed during crash
- Can retrieve crash data days/weeks later by connecting serial or checking ROS logs
- Diagnostic info automatically published to ROS for remote monitoring

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

  // Add datetime here if we have it
} diag;

const uint32_t DIAG_MAGIC = 0xC0FFEE42;

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
    
    if (elapsed > timeout_ms) {
      // Only log to serial/ROS if available
      logf(Severity::WARN, "HANG: %s took %d ms", name, elapsed);
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
    
    if (reason == ESP_RST_TASK_WDT || reason == ESP_RST_INT_WDT) {
      diag.consecutive_wdt_resets++;
      Serial.printf("\n*** WATCHDOG RESET #%lu ***\n", diag.consecutive_wdt_resets);
      Serial.printf("*** SYSTEM HUNG AT: %s ***\n\n", diag.last_location);
      
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
- **Works without serial/ROS** - Data saved to flash, survives power cycles
- **Fast tracking during operation** - RTC SRAM has unlimited writes, no wear concerns
- **Persistent crash reports** - Flash storage means data available days/weeks later
- **Simple and reliable** - Tracks only last location, no complex logging
- **Automatic reporting** - Prints to serial on boot, publishes to ROS when connected
- **Remote monitoring** - Crash reports appear in ROS logs without physical access
- **Catches intermittent issues** - Flash storage survives multiple power cycles

**How crash reporting works:**
1. During operation: Diagnostics tracked in RTC memory (fast, no flash wear)
2. Watchdog triggers: RTC data copied to flash storage (Preferences)
3. After reboot: Crash report printed to serial on every boot
4. After ROS connects: Crash report published as ROS warning message
5. Days/weeks later: Can still retrieve crash data from flash by:
   - Connecting serial and resetting
   - Checking ROS logs for warning messages
   - Crash report persists until explicitly cleared

**What gets reported:**
- Last location where system hung
- Number of consecutive watchdog resets
- Boot count and reset reason

**Note:** Crash reports will print/publish repeatedly until cleared. Manual clearing mechanism to be implemented when this becomes annoying.

### 3. Bounded Serial Loops (PREVENT INFINITE LOOPS)
**Status:** ✅ IMPLEMENTED

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

1. ✅ **Hardware Watchdog** - COMPLETE
2. ✅ **Enhanced HangChecker with diagnostics** - COMPLETE
3. ✅ **GPS serial loop limit** - COMPLETE
4. ✅ **Flash persistence** - COMPLETE
5. **Monitor and analyze** - Review diagnostic logs after watchdog resets

## Implementation Summary

**All critical protections are now implemented:**

1. **Hardware Watchdog Timer**
   - 3-second timeout configured in `setup()`
   - Reset called at start of every `loop()` iteration
   - Automatic reboot on hang

2. **Diagnostic State Tracking**
   - RTC memory tracks last location during operation
   - `HangChecker` automatically updates tracking on every use
   - 9 critical code sections instrumented with `HangChecker`

3. **Flash Persistence** 
   - Crash data saved to flash on watchdog reset
   - Survives power cycles for later analysis
   - Automatic reporting on boot via serial
   - Shows previous crash data from flash on power-on reset

4. **GPS Serial Loop Bounds**
   - Maximum 100 characters per iteration
   - Warning logged if limit reached

5. **Existing Protections**
   - I2C timeout: 5ms
   - `HangChecker` for operation timing
   - `BlockTimer` for performance stats
   - `StuckChecker` for wheel monitoring

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
- **Diagnostic structure is tiny** - Only ~100 bytes for essential crash info
- **RTC memory survives soft resets** but not power cycles or deep sleep  
- **ESP32 automatically tracks reset reason** via `esp_reset_reason()`
- **Cannot safely kill hung threads** - FreeRTOS doesn't support forced task termination
- **Three-second watchdog timeout** provides plenty of margin (loop runs every ~10ms)
- **Diagnostics work without serial/ROS** - Data persists in flash storage across power cycles
- **Simple = reliable** - Tracking only last location avoids complexity
