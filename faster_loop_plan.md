## Plan: Accelerate Main Loop from 50Hz to 100Hz+

Current loop runs at ~50Hz (20ms) with I2C sensors consuming 18ms (TOF 13ms + BNO055 5ms + Compass 2ms). Move ALL I2C operations to dedicated Core 0 thread to free main loop for 100Hz+ control rates and eliminate I2C bus contention.

### Steps

1. **Create I2C sensor thread on Core 0** - Add `i2c_sensor_thread()` function pinned to Core 0 with 8KB stack, similar to [ros_thread](src/main.cpp#L1924), register with watchdog via `esp_task_wdt_add()`. Thread has exclusive Wire access - no Wire mutex needed.

2. **Add ONE data mutex for all sensor data** - Declare `SemaphoreHandle_t sensor_data_mutex` global, initialize with `xSemaphoreCreateMutex()` in [setup()](src/main.cpp#L2225). Use single mutex for all sensor variables - simple and sufficient.

3. **Move TOF reads to thread** - Relocate [TOF reading block](src/main.cpp#L2588-2625) from loop to thread, stagger reads (sensor 0 at 0ms, sensor 1 at 7ms, sensor 2 at 14ms). After reading each sensor: `xSemaphoreTake(sensor_data_mutex, portMAX_DELAY)` → update `tof->distance` → `xSemaphoreGive(sensor_data_mutex)`. Keep mutex hold time minimal (just the variable assignment).

4. **Move BNO055 reads to thread** - Relocate [BNO055 reading block](src/main.cpp#L2558-2565) from loop to thread, read at 10ms intervals. After I2C reads: take mutex → batch update all 4 vectors (`bno_orientation_degrees`, `bno_acceleration`, `bno_gyro_dps`, `bno_mag`, `compass_heading_degrees`) → release mutex.

5. **Move compass reads to thread** - Relocate [compass reading block](src/main.cpp#L2897-2901) from loop to thread, read at 100ms intervals. After I2C read: take mutex → update `compass.last_reading` and computed `compass_heading_degrees` → release mutex. Thread now owns ALL I2C access.

6. **Main loop reads sensor data with same mutex** - In main loop, when you need sensor data: `xSemaphoreTake(sensor_data_mutex, portMAX_DELAY)` → copy needed values to local variables → `xSemaphoreGive(sensor_data_mutex)`. Example: take mutex, copy all 3 TOF distances + BNO gyro for this loop iteration, release mutex, then use local copies for rest of loop.

7. **Remove all I2C blocks from main loop** - Delete moved TOF, BNO055, and compass code from [loop()](src/main.cpp#L2502). Main loop only reads sensor data via mutex-protected access. Verify loop timing via stats.

8. **Test and tune frequencies** - Measure new loop time (expect ~2-5ms), adjust sensor read rates if needed, verify motor control stability at 100Hz+

### Data Protection Pattern

**In I2C thread (writer):**
```cpp
// Read from I2C (no mutex needed for Wire)
sensor->read(false);
float new_distance = sensor->ranging_data.range_mm / 1000.0;

// Quick mutex-protected write
xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
tof->distance = new_distance;  // Single assignment - fast
xSemaphoreGive(sensor_data_mutex);
```

**In main loop (reader):**
```cpp
// Quick mutex-protected read
float local_left, local_center, local_right;
xSemaphoreTake(sensor_data_mutex, portMAX_DELAY);
local_left = tof_left.distance;
local_center = tof_center.distance;
local_right = tof_right.distance;
xSemaphoreGive(sensor_data_mutex);

// Use local copies for rest of loop - no mutex needed
float min_distance = std::min({local_left, local_center, local_right});
```

### Further Considerations

1. **ROS message publishing** - TOF/BNO ROS publishing currently in main loop. Keep in loop reading from shared data with mutex, or move to I2C thread? Trade-off: main loop simplicity vs timing precision

2. **Data staleness acceptable?** - Thread runs async, main loop sees slightly stale sensor data (max 20ms for TOF). Should be fine for motor control with 100Hz loop

3. **Mutex timeout** - Using `portMAX_DELAY` means wait forever. Since mutex hold time is tiny (just assignments), this is safe. Could add timeout + error handling if paranoid.
