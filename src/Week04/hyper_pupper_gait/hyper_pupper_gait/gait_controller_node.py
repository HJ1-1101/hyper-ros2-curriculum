"""
Gait controller node for Mini Pupper.

Pipeline
--------
/cmd_vel (geometry_msgs/Twist)
    └─► TrotGaitPlanner.step(dt, vx, vy, omega)   → foot positions in hip frames
            └─► leg_ik(foot_pos, leg)              → (θ_hip, θ_upper, θ_lower) × 4 legs
                    └─► /leg_controller/joint_trajectory (trajectory_msgs/JointTrajectory)

Interface with hyper_pupper_complete
-------------------------------------
Subscribed topics (provided by hyper_pupper_complete + teleop):
  /cmd_vel           geometry_msgs/Twist   Desired body velocity

Published topics (consumed by hyper_pupper_complete):
  /leg_controller/joint_trajectory   trajectory_msgs/JointTrajectory
      Joint order in every message:
        lf_hip_joint, lf_upper_leg_joint, lf_lower_leg_joint,
        lh_hip_joint, lh_upper_leg_joint, lh_lower_leg_joint,
        rf_hip_joint, rf_upper_leg_joint, rf_lower_leg_joint,
        rh_hip_joint, rh_upper_leg_joint, rh_lower_leg_joint

Student exercise (connecting the packages)
------------------------------------------
The controller will publish joint commands as soon as it starts, but the
simulation needs the ros2_control controllers to be active.  Students must:

  1. Launch the Gazebo simulation:
        ros2 launch hyper_pupper_complete gazebo.launch.py

  2. Activate ros2_control controllers (in a new terminal):
        ros2 control load_controller --set-state active joint_state_broadcaster
        ros2 control load_controller --set-state active leg_controller

  3. Run the gait package:
        ros2 launch hyper_pupper_gait gait_control.launch.py

  Once all three are running, the robot should stand and respond to keyboard input.

ROS 2 parameters
----------------
  body_height   float   Base_link height above ground [m].    Default: 0.07
  gait_period   float   Stride duration [s].                  Default: 0.50
  step_height   float   Peak swing height [m].                Default: 0.03
  duty_factor   float   Fraction of cycle in stance.          Default: 0.50
  ctrl_rate     float   Control loop frequency [Hz].          Default: 50.0
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from hyper_pupper_gait.kinematics import leg_ik, LEG_NAMES
from hyper_pupper_gait.gait_planner import TrotGaitPlanner


# Joint name order expected by the leg_controller (must match controllers.yaml)
JOINT_NAMES = [
    'lf_hip_joint', 'lf_upper_leg_joint', 'lf_lower_leg_joint',
    'lh_hip_joint', 'lh_upper_leg_joint', 'lh_lower_leg_joint',
    'rf_hip_joint', 'rf_upper_leg_joint', 'rf_lower_leg_joint',
    'rh_hip_joint', 'rh_upper_leg_joint', 'rh_lower_leg_joint',
]


class GaitControllerNode(Node):

    def __init__(self):
        super().__init__('gait_controller')

        # ---- ROS parameters ----
        self.declare_parameter('body_height',  0.07)
        self.declare_parameter('gait_period',  0.50)
        self.declare_parameter('step_height',  0.03)
        self.declare_parameter('duty_factor',  0.50)
        self.declare_parameter('ctrl_rate',   50.0)

        body_height = self.get_parameter('body_height').value
        gait_period = self.get_parameter('gait_period').value
        step_height = self.get_parameter('step_height').value
        duty_factor = self.get_parameter('duty_factor').value
        ctrl_rate   = self.get_parameter('ctrl_rate').value

        # ---- Gait planner ----
        self._planner = TrotGaitPlanner(
            gait_period=gait_period,
            duty_factor=duty_factor,
            step_height=step_height,
            body_height=body_height,
        )

        # ---- Current velocity command ----
        self._vx    = 0.0
        self._vy    = 0.0
        self._omega = 0.0

        # ---- ROS interfaces ----
        self._cmd_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_cb,
            10,
        )

        self._traj_pub = self.create_publisher(
            JointTrajectory,
            '/leg_controller/joint_trajectory',
            10,
        )

        # ---- Control timer ----
        self._dt = 1.0 / ctrl_rate
        self._timer = self.create_timer(self._dt, self._control_cb)

        # Time_from_start for each trajectory point: one control step ahead.
        # Using 2× dt gives the controller a small look-ahead buffer.
        self._tfs_ns = int(2 * self._dt * 1e9)

        self.get_logger().info(
            f'GaitController started | body_height={body_height:.3f} m  '
            f'gait_period={gait_period:.2f} s  step_height={step_height:.3f} m  '
            f'ctrl_rate={ctrl_rate:.0f} Hz'
        )

    # ------------------------------------------------------------------
    # Subscriber callback
    # ------------------------------------------------------------------

    def _cmd_vel_cb(self, msg: Twist):
        self._vx    = msg.linear.x
        self._vy    = msg.linear.y
        self._omega = msg.angular.z

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_cb(self):
        # Advance gait phase and get desired foot positions
        foot_positions, _ = self._planner.step(
            self._dt, self._vx, self._vy, self._omega
        )

        # Run IK for every leg
        positions = []
        ik_ok = True
        for leg in LEG_NAMES:
            try:
                th, tu, tl = leg_ik(foot_positions[leg], leg)
                positions.extend([th, tu, tl])
            except Exception as exc:
                self.get_logger().warn(
                    f'IK failed for {leg}: {exc}', throttle_duration_sec=1.0
                )
                ik_ok = False
                break

        if not ik_ok:
            return

        # Build and publish JointTrajectory.
        # Leave header.stamp at zero — JointTrajectoryController interprets a
        # zero stamp as "start from now" using its own clock, which avoids any
        # wall-time vs sim-time mismatch.
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start = Duration(
            sec=0,
            nanosec=self._tfs_ns,
        )
        msg.points = [point]

        self._traj_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GaitControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
