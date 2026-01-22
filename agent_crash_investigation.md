# micro-ROS Agent Crash Investigation

## Issue
Agent crashes with: `BadParamException: Unexpected byte value in Cdr::deserialize(bool), expected 0 or 1`

Crashes are **intermittent** - often during `run_force_characterization.py` tests. Does NOT correlate with ESP32 kernel panics.

## Affected Messages (from hex dumps)
- `sensor_msgs/NavSatFix` - frame_id `gps_frame`
- `sensor_msgs/Range` - frame_id `tof_frame`
- `std_msgs/String` - log messages

## Thread Architecture

| Thread | Core | Publishes |
|--------|------|-----------|
| `loop()` | 1 | update, tof/range, nav_sat_fix, battery |
| `ros_thread` | 1 | (manages connection, runs executor) |
| `i2c_sensor_thread` | 0 | (no publishing - reads sensors only) |
| `logf()` | any | log, rosout (called from multiple contexts) |

**Key finding**: Publishing happens from multiple threads without mutex protection around `rcl_publish()` calls.

## Potential Race Conditions

1. **`logf()` called from any context** - publishes to `log_publisher` and `rosout_publisher`
   - Called from main loop, callbacks, possibly ISR contexts
   - No mutex around the publish

2. **Shared message buffers** - `log_msg`, `rosout_msg` are global
   - `vsnprintf` fills buffer, then publishes
   - Another thread could call `logf()` mid-publish

3. **`ros_ready` flag** - checked without mutex
   - Thread could start publishing while `ros_thread` is reconnecting

## Known Corruption
Code already has defensive check (line ~3391):
```cpp
if (tof_distance_msg.radiation_type != sensor_msgs__msg__Range__INFRARED) {
  // Memory corruption detected!
```

## Action Items

- [x] Rebuild micro-ROS library - did not fix
- [x] Add mutex around all `rcl_publish()` calls - DONE
- [x] Test if mutex fixes issue - **FIXED**

## Resolution
Adding `publish_mutex` around all `rcl_publish()` calls fixed the agent crashes. The race condition was caused by concurrent publishing from multiple threads (main loop + subscription callbacks via executor) corrupting the XRCE-DDS serialization buffer.

Note: ESP32 kernel panics in lwip stack are a **separate issue** (pre-existing) - unrelated to the agent CDR crashes.

## Timeline
- 2026-01-21: Issue identified, docker image updated, full rebuild started
- 2026-01-21: Rebuild did not fix, added `publish_mutex` around all 7 `rcl_publish()` calls
- 2026-01-21: **RESOLVED** - Agent no longer crashes during force characterization tests

---

# ESP32 lwip Crash Investigation

## Issue
Intermittent ESP32 kernel panics on Core 0 during force characterization tests.

```
Guru Meditation Error: Core 0 panic'ed (LoadProhibited)
EXCVADDR: 0x001c0106  (invalid memory address)
```

## Backtrace (decoded)
```
lwip_standard_chksum    <- crash here (reading invalid memory)
inet_cksum_pseudo_base
inet_chksum_pseudo
ip_chksum_pseudo
udp_sendto_if_src
udp_sendto_if
udp_sendto
lwip_netconn_do_send
tcpip_thread
```

## Analysis
Crash occurs in lwip's UDP checksum calculation - reading corrupted/freed network buffer data.

**Core 0 runs:**
- `i2c_sensor_thread` - I2C reads for ToF, BNO055, compass
- `tcpip_thread` (lwip) - network stack
- WiFi tasks

**Core 1 runs:**
- `loop()` - main application
- `ros_thread` - micro-ROS connection management

## Possible Causes

1. **Memory corruption** - heap fragmentation or buffer overflow corrupting lwip buffers
2. **Stack overflow** - i2c_sensor_thread (8KB stack) or other tasks
3. **micro-ROS internal race** - even with publish_mutex, internal lwip buffers may have issues
4. **WiFi/network instability** - packet corruption or timing issues

## Action Items
- [ ] Check stack high water marks for all tasks
- [ ] Monitor heap fragmentation
- [ ] Check if crash correlates with specific I2C activity
- [ ] Consider increasing task stack sizes

---

# Remaining Agent CDR Crashes (Post-Mutex Fix)

## Observation
After adding `publish_mutex`, agent crashes are **much less frequent** but still occur:
- Only 2 deserialization errors over extended idle period (vs immediate crashes before)
- Both errors on `NavSatFix` message (`gps_frame`)
- Agent eventually crashed after 2nd error

**Foxglove also sees corruption** (confirms packet-level issue):
- `/white_crash/left_distance` - Invalid array length: 1920296723
- `/white_crash/gps/fix` - Offset outside DataView bounds
- `/rosout_best_effort` - Invalid array length: 68

## Analysis
The mutex fixed the primary race condition (concurrent logf + loop publishing). Remaining errors likely from:
1. Internal micro-ROS/XRCE-DDS buffer management
2. Network-level packet corruption
3. Timing-dependent serialization bugs in the library

## Upstream Issue
**The agent should NOT crash on deserialization errors.** This is a bug in eProsima Fast-CDR:
- It throws `BadParamException` which is uncaught
- Should log error and drop packet instead

## Potential Workarounds
1. **Restart wrapper** - ✅ Already implemented
2. **Patch agent locally** - Catch `BadParamException` in Fast-CDR/XRCE-DDS, log and continue
3. **Report upstream** - File issue with micro-ROS/eProsima

## Local Agent Patch Plan
Target: Make agent resilient to malformed packets

**Root cause found**: In `InputMessage.hpp` line ~205, `deserialize()` only catches `NotEnoughMemoryException` but NOT `BadParamException`.

```cpp
// Current (crashes on BadParamException):
catch(eprosima::fastcdr::exception::NotEnoughMemoryException &)

// Fix (catch all FastCDR exceptions):
catch(eprosima::fastcdr::exception::Exception &)
```

Repos:
- `eProsima/Micro-XRCE-DDS-Agent` - [InputMessage.hpp#L205](https://github.com/eProsima/Micro-XRCE-DDS-Agent/blob/main/include/uxr/agent/message/InputMessage.hpp#L205)

**Status**: ✅ Patched agent built and tested - survives malformed packets with error log instead of crash.

Fork: https://github.com/berickson/Micro-XRCE-DDS-Agent
