# Allegro Hand Grasp Library

`allegro_hand_grasp_library` is a refactored version of the legacy **BHand library**.

> **Note:** This package only supports Allegro Hand V4.

---

## Build & Installation

```bash
colcon build --packages-select allegro_hand_grasp_library
```

---

## Usage

### Dependencies
Before using the library, the following dependency packages need to be installed.

- `pinocchio`
- `spdlog`
- `yaml-cpp-vendor`
- `eigen3`
- `boost`

### Basic Usage Procedure

The basic usage flow of the library is as follows: **[Object Creation] → [Motion Setting] → [Control Loop]**.

#### C++ Example Code

```cpp
#include <allegro_hand_grasp_library/ah_grasp.hpp>
#include <map>
#include <string>
#include <vector>

// 1. Create Library Object
// ------------------------------------
// Set the options struct.
AllegroHandGrasp::Options options;
options.device = AllegroHandGrasp::Options::HandDevice::HAND_V4; // Select the hand version
options.urdf_string = " ... Pass URDF file content as a string ... "; // Robot URDF
options.reference_frame_name = "palm_link"; // Reference coordinate system (palm link)
options.fingertip_link_names = {"thumb_tip", "index_tip", "middle_tip", "ring_tip"}; // Tip link names (in order: thumb, index, middle, ring)

// Create the object via the factory method.
AllegroHandGrasp::Ptr grasp_controller = AllegroHandGrasp::Create(options);

// (Optional) Custom gains can be set with a YAML-formatted string.
// std::string custom_gains_yaml = "home:\n  thumb:\n    kp: [1000, 700, 600, 600]\n    kd: [50, 50, 50, 40]";
// grasp_controller->set_gain_table(custom_gains_yaml);

// 2. Set Motion Type
// ------------------------------------
// Select the desired grasp motion.
grasp_controller->set_motion_type(AllegroHandGrasp::MotionType::MOTION_TYPE_GRASP_3);


// 3. Control Loop
// ------------------------------------
// Repeat the following process according to the control cycle (e.g., 100Hz).
double dt = 0.01; // Control cycle (seconds)
while (true) {
    // Get the current joint states (position, velocity) of the hand.
    std::map<std::string, std::pair<double, double>> current_joint_states;
    // ... (Read actual joint values from the robot) ...

    // Pass the current state to the library.
    grasp_controller->set_joint_state(current_joint_states);

    // Update the control algorithm by one step.
    grasp_controller->update(dt);

    // Get the calculated desired joint torques.
    std::map<std::string, double> desired_joint_torques;
    grasp_controller->get_joint_torque(desired_joint_torques);

    // Send the calculated torques to the hand motors.
    // ... (Send torque command to the robot) ...
}
```

---

## Available Motions

The following built-in motions can be used via the `set_motion_type()` function. Each motion is defined by the `AllegroHandGrasp::MotionType` enum.

- `MOTION_TYPE_HOME`: Move to the default home position
- `MOTION_TYPE_READY`: Move to a ready-to-grasp pose
- `MOTION_TYPE_GRASP_3`: Three-finger grasp (thumb, index, middle)
- `MOTION_TYPE_GRASP_4`: Four-finger grasp
- `MOTION_TYPE_PINCH_IT`: Pinch with index finger and thumb
- `MOTION_TYPE_PINCH_MT`: Pinch with middle finger and thumb
- `MOTION_TYPE_ENVELOP`: Power grip that envelops an object
- `MOTION_TYPE_NONE`: Turn off motor torque