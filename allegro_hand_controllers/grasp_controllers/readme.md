# Allegro Hand Grasp Controllers

This package provides two advanced, action-based controllers for the Allegro Hand, designed for use with the `ros2_control` framework. These controllers allow a user to command the hand to specific joint positions or execute pre-defined grasp types.

## Controllers

This package provides two controller plugins that implement `controller_interface::ControllerInterface`.

> **Note:** This package only supports Allegro Hand V4.

--- 

### 1. Position Grasp Controller

* **Plugin Name**: `position_controllers/AllegroHandGraspController`
* **Description**: This controller accepts an action goal containing target positions for each of the 16 joints. It is ideal for tasks requiring the hand to move to a precise posture.
* **Action Type**: `control_msgs::action::ParallelGripperCommand`

### 2. Effort Grasp Controller

* **Plugin Name**: `effort_controllers/AllegroHandGraspController`
* **Description**: This controller executes pre-defined grasp types via a string command. Internally, it uses the **`BHand` library** to calculate the necessary joint torques to achieve the specified grasp. This is useful for complex interactions where force control is desired.
* **Action Type**: `allegro_hand_control_msgs/action/GraspCommand`
* **Supported Commands**:
    * `home`
    * `ready`
    * `grasp_3` (three-finger grasp)
    * `grasp_4` (four-finger grasp)
    * `pinch_it` (index & thumb pinch)
    * `pinch_mt` (middle & thumb pinch)
    * `envelop` (enveloping grasp)
    * `gravcomp` (gravity compensation)
    * `off` (turn joints off)

---

## Controller Parameters

Both controllers share the following parameters, which can be configured in the controller's YAML configuration file.

| Parameter | Type | Description |
| :--- | :--- | :--- |
| **`joints`** | `string[]` | A list of joint names to be managed by the controller. |
| **`goal_tolerance`** | `double` | The tolerance for considering a goal as reached. |
| **`allow_stalling`** | `bool` | Determines if the action should return success if the gripper stalls while moving to the goal. |
| **`stall_timeout`** | `double` | The time in seconds after which a lack of movement is considered a stall. |
| **`stall_velocity_threshold`** | `double` | The maximum velocity threshold used to detect a stall. |
| **`action_monitor_rate`** | `double` | The frequency (Hz) at which the action status is monitored. |

---

## Usage

These controllers are loaded as plugins by the `ros2_control` controller manager.

### Configuration Example

Below is an example snippet from a `ros2_controllers.yaml` file for loading the controllers:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 333  # Hz 

    # Position-based Grasp Controller
    allegro_hand_posture_controller:
      type: position_controllers/AllegroHandGraspController
      joints:
        - ah_joint00
        # ... (all 16 joints listed)
        - ah_joint33
      goal_tolerance: 0.01
      allow_stalling: True
      stall_timeout: 0.5
      stall_velocity_threshold: 0.01

    # Effort-based Grasp Controller
    allegro_hand_grasp_controller:
      type: effort_controllers/AllegroHandGraspController
      joints:
        - ah_joint00
        # ... (all 16 joints listed)
        - ah_joint33
      allow_stalling: True
      stall_timeout: 0.5
      stall_velocity_threshold: 0.01
```

### Sending Action Goals

You can test the controllers from the terminal using the `ros2 action send_goal` command.

#### Position Posture Controller Example

Send a **scissors pose** goal to the `allegro_hand_posture_controller` controller.

```bash
ros2 action send_goal /allegro_hand_posture_controller/posture_cmd control_msgs/action/ParallelGripperCommand '{
"command": {
    "name": [
    "ah_joint00", "ah_joint01", "ah_joint02", "ah_joint03",
    "ah_joint10", "ah_joint11", "ah_joint12", "ah_joint13",
    "ah_joint20", "ah_joint21", "ah_joint22", "ah_joint23",
    "ah_joint30", "ah_joint31", "ah_joint32", "ah_joint33"
    ],
    "position": [
    0.876451688853475, 1.2216305543672838, 1.4264144404001213, 1.6641368528133413,
    0.09960998262586793, -0.002163044412170033, -0.033398812297706866, 0.07304468498730649,
    0.144526588225768, 0.029139390304994982, -0.0029649433088131255, -0.018537053421023006,
    0.07774173357748533, 1.6961666504428443, 1.4743204682667252, 0.7695174550487038
    ]
}
}'
```

#### Grasp Controller Example

Send a **four-finger grasp** command to the `allegro_hand_grasp_controller` controller.

```bash
ros2 action send_goal /allegro_hand_grasp_controller/grasp_cmd allegro_hand_control_msgs/action/GraspCommand '{ "command": "grasp_4" }'
```

Send an **envelop grasp** command.

```bash
ros2 action send_goal /allegro_hand_grasp_controller/grasp_cmd allegro_hand_control_msgs/action/GraspCommand '{ "command": "envelop" }'
```

