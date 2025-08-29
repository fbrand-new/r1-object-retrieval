# Check Object Grasped Module

## Overview

The `checkObjectGrasped` module is designed to verify if a specific object has been successfully grasped by the R1 robot using a **non-blocking state machine** approach. It performs two main functions:

1. **Gaze Control**: Retrieves the current transform of the robot's hand via ROS2 TF and commands the gaze controller to look at the hand.

2. **Object Detection Verification**: Monitors an object detection port and collects a configurable number of observations to determine grasp success based on detection statistics.

## State Machine

The module operates with four external states that are continuously published on a status port:

- **UNKNOWN**: Initial state, accepts all RPC commands
- **CHECKING**: Actively checking for grasped object (includes gaze command + observations)
- **SUCCESS**: Object successfully detected above threshold (60% by default)  
- **FAILED**: Object detection below threshold

### Internal Implementation:

The module actually has an internal `GAZE` state that:
1. Sends the gaze command exactly **once**
2. Appears externally as "CHECKING" 
3. Immediately transitions to observation collection

### State Transitions:

```
UNKNOWN → GAZE (via "check" command)
GAZE → CHECKING (automatically after sending gaze command)
CHECKING → SUCCESS (if detection rate ≥ threshold)
CHECKING → FAILED (if detection rate < threshold)  
SUCCESS/FAILED → UNKNOWN (via "stop" command or new "check" command)
```

## Features

- **Non-blocking Operation**: RPC calls return immediately, status is published continuously
- **ROS2 TF Integration**: Direct access to TF transforms without YARP abstraction
- **Configurable Detection Logic**: Set observation count and success threshold
- **Single Gaze Command**: Sends gaze command exactly once per check cycle
- **Real-time Status**: Continuous status publishing with progress information

## Configuration

The module can be configured through the `checkObjectGrasped.ini` file:

```ini
[DEFAULT]
robot                           cer                    # Robot name
period                          1.0                    # Module update period (1Hz)
target_object                   bottle                 # Object to detect
hand_frame                      r_hand                 # Hand frame name in TF
base_frame                      mobile_base_body_link  # Base frame name in TF
observation_count               5                      # Number of observations to collect
success_threshold               0.6                    # Success rate threshold (0.0-1.0)
detection_port                  /checkObjectGrasped/detections:i  # Input port for detections
rpc_port                        /checkObjectGrasped/rpc           # RPC command port
status_port                     /checkObjectGrasped/status:o      # Status output port
gaze_port                       /checkObjectGrasped/gaze:o        # Gaze command output port
```

## Ports

### Input Ports
- `/checkObjectGrasped/detections:i`: Receives object detection data as Bottle messages
- `/checkObjectGrasped/rpc`: RPC port for commands

### Output Ports
- `/checkObjectGrasped/status:o`: Continuously publishes current state and progress
- `/checkObjectGrasped/gaze:o`: Sends gaze commands to look at hand

## RPC Commands

- `check`: Start checking for grasped object (transitions to CHECKING state)
- `stop`: Stop checking and reset to UNKNOWN state
- `status`: Get current state (UNKNOWN/CHECKING/SUCCESS/FAILED)
- `set_target <object_name>`: Set the target object to detect
- `get_target`: Get the current target object name
- `help`: Show available commands

**Note**: Commands are only accepted in UNKNOWN, SUCCESS, or FAILED states. During CHECKING, commands return "busy".

## Usage

1. **Start the module**:
   ```bash
   checkObjectGrasped --from checkObjectGrasped.ini
   ```

2. **Connect to object detection module**:
   ```bash
   yarp connect /objectDetection/output:o /checkObjectGrasped/detections:i
   ```

3. **Start checking**:
   ```bash
   yarp rpc /checkObjectGrasped/rpc
   >> check
   ```

4. **Monitor status**:
   ```bash
   yarp read ... /checkObjectGrasped/status:o
   ```

5. **Connect gaze controller**:
   ```bash
   yarp connect /checkObjectGrasped/gaze:o /gazeController/rpc:i
   ```

## Detection Data Format

The module expects detection data in the following Bottle format:
```
[object_name confidence x y w h object_name confidence x y w h ...]
```

Where:
- `object_name`: String name of the detected object
- `confidence`: Detection confidence (0.0-1.0)
- `x, y, w, h`: Bounding box coordinates and dimensions

## Status Output Format

The status port continuously publishes the current state:

```
# For UNKNOWN, SUCCESS, FAILED states:
[STATE_NAME]

# For CHECKING state (with progress):
[CHECKING current_observations total_observations positive_detections]
```

Example: `[CHECKING 3 5 2]` means 3/5 observations completed, 2 positive detections.

## Gaze Command Format

The module sends gaze commands in the format:
```
[look_at_point x y z]
```

Where x, y, z are the hand coordinates in the base frame.

## Dependencies

- YARP
- yarp-devices-ros2 (for TF integration)
- ROS2 TF2 system running on the robot

## Notes

- The module requires a running ROS2 TF2 system with the robot's hand transforms published
- Object detection confidence threshold is set to 0.5 by default
- The module automatically resets detection history on timeout or when changing target objects
