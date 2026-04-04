# HYPER — ROS 2 Study Club Curriculum

A 4-week hands-on ROS 2 course built for 2nd-year students. Each week combines a short concept lecture, an in-class lab, and a take-home assignment.

**Stack:** Ubuntu 24.04 · ROS 2 Jazzy · Python 3 · C++20 · Gazebo · RViz

---

## Schedule Overview

| Week | Date | Topic |
|---|---|---|
| 1 | Mar 11 | Ecosystem, Workspace & Packages |
| 2 | Mar 18 | Topics, Services & Custom Interfaces |
| 3 | Mar 25 | URDF & RViz |
| 4 | Apr 2 | Gazebo, Xacro & Parameters |

---

## Week 1 — Ecosystem, Workspace & Packages

**Concepts:** ROS 2 middleware, DDS, the Computational Graph. Creating `hyper_ws`, understanding the `src/` layout. Package anatomy for both C++ and Python (`ros2 pkg create --build-type ament_python`). Role of `package.xml` and `setup.py`.

**Lab:** Write a "Hello World" node in both C++ and Python that prints a message to the console every second using a timer.

**Assignment: "Making Python and C++ Talk"**

Add two packages inside `hyper_ws`:

- `pyhello` — publishes `"Hello World from pyhello!"` to a topic every 1 second and echoes it to the terminal
- `cpphello` — subscribes to `/hello_topic` and prints `"I heard: 'Hello World from pyhello!'"` to the terminal

**Submission:** Split VS Code terminal — one side running `ros2 run pyhello <node>`, the other running `ros2 run cpphello <node>`.

---

## Week 2 — Topics, Services & Custom Interfaces

**Concepts:**
- **Topics (Pub/Sub):** Continuous data streams — the "radio broadcast" model
- **Services (Client/Server):** Request-response — the "phone call" model
- **Interfaces:** Using standard messages (`std_msgs`, `geometry_msgs`) and declaring dependencies in `package.xml`
- Visualizing topics with `rqt_graph` and `rqt_plot`
- Custom interface generation via `rosidl_default_generators`

**Lab:** Write a `CustomMsg.msg` and `CustomSrv.srv` from scratch.

**Assignment: "Smart Battery Management System"**

Create two packages inside `hyper_ws`:

**`my_robot_interfaces`** — defines custom message and service types:
- `BatteryStatus.msg` — voltage, charge level, status string, LED color
- `SetLedColor.srv` — request: desired color; response: success boolean

**`smart_battery_system`** — Python package containing two nodes:
- **Node A:** Initializes battery at 5V, 100% charge, status `"Normal"`, LED `green`. Decrements charge by 5% every second and publishes to `/battery_info`. When charge drops below 20%, sets status to `"Low"`.
- **Node B:** Subscribes to `/battery_info`. When charge drops below 20%, sends a service request to change LED color to `red`.

**Submission:**
- Split terminal showing Node A and Node B running simultaneously
- `rqt_graph` screenshot of the two nodes (disable *Hide: Debug* if nodes don't appear)
- `rqt_plot` showing battery charge level over time

---

## Week 3 — URDF & RViz

**Concepts:** Defining links (arms, wheels) and joints (revolute, continuous). Adding `<collision>` and `<inertial>` properties to prevent physics explosions in simulation. Writing a launch file that brings up RViz + `joint_state_publisher_gui` in one command.

**Lab:** Download a simple 1-DOF leg URDF, spawn it in Gazebo, and move it with `joint_state_publisher_gui`.

**Assignment: "Mini Manipulator"**

Create a `hyper_description` package inside `hyper_ws` and write a URDF satisfying these conditions:

1. **Structure:** At least 3 links (including a fixed base) and 2 moving joints (`revolute` or `continuous`)
2. **Design:** Use `<material>` tags to give each link a distinct color — basic geometry (`cylinder`, `box`) is fine
3. **Launch:** A single command `ros2 launch hyper_description display.launch.py` must open both RViz and `joint_state_publisher_gui`

**Submission:**
- Screenshot of the URDF/Xacro source
- Screen recording or GIF showing both joints moving smoothly via the GUI sliders

---

## Week 4 — Gazebo, Xacro & Parameters

**Concepts:**

- **Gazebo Sim:** A full physics world with gravity $g \approx 9.81\ \text{m/s}^2$, friction, and collision — beyond RViz's pure visualization
- **Xacro:** URDF macro extension using `${property}` variables and `include` statements for reusable robot models
- **SDF (Simulation Description Format):** Gazebo-native format for describing physics engines, lighting, and terrain
- **YAML Parameters:** Manage controller configs, joint names, and gain values without touching source code
- **`ros_gz_bridge`:** Translates between ROS 2 DDS and Gazebo Transport in real time

**Lab:** Update a provided `mini_pupper` URDF to work with Jazzy, visualize it in RViz, and inspect its inertial properties via Gazebo's Component Inspector.

**Assignment: "Pupper's First Steps — Gait Controller Integration"**

Connect the provided `quadruped_controller` package (includes gait generation and IK logic) to the Gazebo environment built in the lab.

1. **Bridge Setup:** Modify `gazebo.launch.py` so that `/clock`, `/scan`, `/tf`, and `/camera/image_raw` are correctly bridged between ROS 2 and Gazebo via `ros_gz_bridge`

2. **Control Loop:**
   - **Input:** `/cmd_vel` (`Twist`) from `teleop_twist_keyboard`
   - **Logic:** Generate a Trot gait pattern from forward velocity $v_x$ and yaw rate $\omega_z$, then solve inverse kinematics ($IK$) to produce 12 joint target angles
   - **Output:** Publish computed angles as `trajectory_msgs/JointTrajectory` to `/leg_controller/joint_trajectory`

**Submission:**
- Screen recording of Pupper walking in Gazebo under `teleop_twist_keyboard` control
- `rqt_graph` screenshot clearly showing the chain: Teleop → Controller → Bridge → Gazebo
