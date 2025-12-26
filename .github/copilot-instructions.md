## General guidelines
This is a potentially dangerous and exacting project. If there is any ambiguity on what the user is asking for, get clarifications before proceding.

Always get permission before changing any files, regardless of whether they are code files, plans, etc.


## PlatformIO Task Execution

PlatformIO commands cannot be run directly from the command line in this workspace. Instead, use the VS Code task system:

run_vscode_command with:

commandId: "workbench.action.tasks.runTask"
args: ["PlatformIO: <TaskName> (lolin_s3_mini)"]

**To run PlatformIO tasks:**
**Available PlatformIO tasks for lolin_s3_mini:**
- `PlatformIO: Build (lolin_s3_mini)` - Compile the project
- `PlatformIO: Upload (lolin_s3_mini)` - Flash to device
- `PlatformIO: Monitor (lolin_s3_mini)` - Open serial monitor
- `PlatformIO: Upload and Monitor (lolin_s3_mini)` - Flash and monitor
- `PlatformIO: Clean (lolin_s3_mini)` - Clean build files
- `PlatformIO: Full Clean (lolin_s3_mini)` - Full clean including dependencies (use when ROS messages change)

**Example:**
```json
{
  "commandId": "workbench.action.tasks.runTask",
  "args": ["PlatformIO: Build (lolin_s3_mini)"]
}