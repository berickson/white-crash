# Command line over ROS Topics - Implementation Plan

**Goal:** Add a minimal command-line REPL interface using two ROS string topics for remote command execution and response streaming. Enables scriptable control of robot modes, parameters, and diagnostics without physical buttons.

**Topics:**
- Subscribe: `/white_crash/command` (std_msgs/String) - Incoming commands
- Publish: `/white_crash/command_response` (std_msgs/String) - Command results

**Initial Commands:**
- `set-event <name>` - Trigger FSM event (e.g., `set-event force-char`, `set-event hand`, `set-event off`)

**IMPORTANT IMPLEMENTATION PROTOCOL:**
- ⚠️ **Minimal implementation first** - Only set-event command initially
- ⚠️ **Get permission before expanding** - No feature creep without approval

---

## Step 1: Add ROS Topic Infrastructure

**File:** [src/main.cpp](src/main.cpp)

**Add global variables** (around line 405, near other ROS globals):
```cpp
// Command REPL infrastructure
rcl_subscription_t command_subscription;
rcl_publisher_t command_response_publisher;
std_msgs__msg__String command_msg;
std_msgs__msg__String command_response_msg;
```

**Update executor capacity** in [create_ros_node_and_publishers()](src/main.cpp#L614):
```cpp
// Change from 1 to 2 subscriptions
RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
```

---

## Step 2: Implement Command Callback

**File:** [src/main.cpp](src/main.cpp)

**Add callback function** (after [cmd_vel_callback](src/main.cpp#L543), around line 550):
```cpp
// Command REPL callback
void command_callback(const void *msgin) {
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  
  // Parse command from msg->data.data (null-terminated string)
  char response[128];
  
  // Simple parser: check if starts with "set-event "
  if (strncmp(msg->data.data, "set-event ", 10) == 0) {
    // Extract event name (everything after "set-event ")
    const char* event_name = msg->data.data + 10;
    
    // Trigger FSM event
    fsm.set_event(event_name);
    
    snprintf(response, sizeof(response), "OK: Triggered event '%s'", event_name);
    logf("REPL: %s", response);
  } else {
    snprintf(response, sizeof(response), "ERROR: Unknown command '%s'", msg->data.data);
    logf("REPL: %s", response);
  }
  
  // Send response
  send_repl_response(response);
}
```

---

## Step 3: Wire Up ROS Subscriptions/Publishers

**File:** [src/main.cpp](src/main.cpp)

**In [create_ros_node_and_publishers()](src/main.cpp#L554)** - Add after other publishers (around line 605):
```cpp
// Command REPL publisher
RCCHECK(rclc_publisher_init_best_effort(
    &command_response_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/white_crash/command_response"));

// Command REPL subscription
RCCHECK(rclc_subscription_init_best_effort(
    &command_subscription,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "/white_crash/command"));
```

**Add to executor** (after [cmd_vel subscription](src/main.cpp#L617), around line 620):
```cpp
// Add command REPL subscription to executor
RCCHECK(rclc_executor_add_subscription(
    &executor, 
    &command_subscription, 
    &command_msg, 
    &command_callback, 
    ON_NEW_DATA));
```

**In [destroy_ros_node_and_publishers()](src/main.cpp#L618)** - Add cleanup:
```cpp
RCCHECK(rcl_subscription_fini(&command_subscription, &node));
RCCHECK(rcl_publisher_fini(&command_response_publisher, &node));
```

---

## Step 4: Add Response Helper Function

**File:** [src/main.cpp](src/main.cpp)

**Add helper function** (around line 540, before callbacks):
```cpp
// Helper to publish REPL response
void send_repl_response(const char* response_text) {
  if (!ros_ready) return;
  
  // Set string data using micro_ros_string_utilities
  command_response_msg.data.data = (char*)response_text;
  command_response_msg.data.size = strlen(response_text);
  command_response_msg.data.capacity = strlen(response_text) + 1;
  
  RCSOFTCHECK(rcl_publish(&command_response_publisher, &command_response_msg, NULL));
}
```

---

## Testing

**Trigger force characterization mode:**
```bash
ros2 topic pub --once /white_crash/command std_msgs/String "{data: 'set-event force-char'}"
```

**Monitor responses:**
```bash
ros2 topic echo /white_crash/command_response
```

**Return to hand mode:**
```bash
ros2 topic pub --once /white_crash/command std_msgs/String "{data: 'set-event hand'}"
```

---

## Python Client

**File:** `scripts/command_line.py`

Simple Python client for sending commands and receiving responses:

```python
#!/usr/bin/env python3
"""
Simple REPL client for white_crash robot command interface.
Sends commands to /white_crash/command and listens for responses.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
import sys


class ReplClient(Node):
    def __init__(self):
        super().__init__('repl_client')
        
        # Use best-effort QoS to match robot
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT
        )
        
        # Publisher for commands
        self.cmd_pub = self.create_publisher(
            String,
            '/white_crash/command',
            qos_profile
        )
        
        # Subscriber for responses
        self.response_sub = self.create_subscription(
            String,
            '/white_crash/command_response',
            self.response_callback,
            qos_profile
        )
        
        self.get_logger().info('REPL client ready. Waiting for responses...')
    
    def response_callback(self, msg):
        """Handle response from robot"""
        print(f"Response: {msg.data}")
    
    def send_command(self, command):
        """Send a command to the robot"""
        msg = String()
        msg.data = command
        self.cmd_pub.publish(msg)
        self.get_logger().info(f'Sent: {command}')


def main(args=None):
    rclpy.init(args=args)
    client = ReplClient()
    
    if len(sys.argv) > 1:
        # Command-line mode: send command and wait briefly for response
        command = ' '.join(sys.argv[1:])
        client.send_command(command)
        
        # Spin briefly to receive response
        import time
        for _ in range(10):  # Wait up to 1 second
            rclpy.spin_once(client, timeout_sec=0.1)
            time.sleep(0.1)
    else:
        # Interactive mode: read commands from stdin
        print("Interactive REPL mode. Type commands (e.g., 'set-event force-char') or 'quit' to exit.")
        print()
        
        import threading
        import select
        
        def spin_thread():
            while rclpy.ok():
                rclpy.spin_once(client, timeout_sec=0.1)
        
        # Start ROS spin in background thread
        thread = threading.Thread(target=spin_thread, daemon=True)
        thread.start()
        
        try:
            while True:
                # Check if input is available (non-blocking)
                if sys.stdin in select.select([sys.stdin], [], [], 0.1)[0]:
                    line = sys.stdin.readline().strip()
                    if line.lower() in ['quit', 'exit', 'q']:
                        break
                    if line:
                        client.send_command(line)
        except KeyboardInterrupt:
            print("\nExiting...")
    
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Usage:**

Command-line mode:
```bash
python3 scripts/command_line.py set-event force-char
python3 scripts/command_line.py set-event hand
```

Interactive mode:
```bash
python3 scripts/command_line.py
# Then type commands interactively
set-event force-char
set-event hand
quit
```

---

## Further Considerations

1. **Command extensibility** - Future commands: `get battery`, `set k_p 4.0`, `status`, `help`. Add incrementally as needed, not now.

2. **Long responses** - Command may run for a while and send back a lot of text. Will want to send one line at a time, and limit each line

3. **Run from serial** - Run commands from the serial prompt,

2. **String memory management** - Current approach uses stack strings. If responses get large, may need micro_ros_string utilities or static buffers.

3. **Error handling** - Invalid mode names will be silently ignored by FSM. Should we validate mode names first?

4. **Response format** - Using `OK:` / `ERROR:` prefix for easy parsing. Good enough for MVP?

5. **Thread safety** - Callback runs in ros_thread on core 1. FSM access should be safe (no mutex needed), but verify during testing.
