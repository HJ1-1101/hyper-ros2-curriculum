"""
Keyboard teleoperation node for Mini Pupper.

Reads raw keyboard input (non-blocking, no Enter required) and publishes a
geometry_msgs/Twist message on /cmd_vel at 10 Hz.

Key bindings
------------
  W / S       forward / backward    (linear.x)
  A / D       strafe left / right   (linear.y)
  Q / E       turn CCW / CW         (angular.z)
  Space       emergency stop
  X or Ctrl-C quit

Hold a key to maintain velocity. Release all keys to stop.

Usage
-----
    ros2 run hyper_pupper_gait teleop_key_node

or via the launch file:
    ros2 launch hyper_pupper_gait gait_control.launch.py

Note: this node requires a terminal that supports raw-mode TTY input.
"""

import sys
import time
import termios
import tty
import select

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


# ---------------------------------------------------------------------------
# Velocity settings
# ---------------------------------------------------------------------------
LINEAR_VEL   = 0.20   # m/s
ANGULAR_VEL  = 0.80   # rad/s

# How long after the last keypress before velocity is zeroed (seconds).
# This must be longer than the OS key-repeat interval (~30 ms) so holding
# a key keeps non-zero velocity, but short enough that release feels snappy.
KEY_TIMEOUT  = 0.15   # s

PUBLISH_RATE = 10.0   # Hz


HELP_TEXT = """
Mini Pupper Keyboard Teleoperation
-----------------------------------
  W / S     forward / backward
  A / D     strafe left  / right
  Q / E     rotate CCW   / CW
  Space     STOP (zero all velocities)
  X         Quit

Hold a key to maintain velocity.
Release to stop (after ~150 ms).
-----------------------------------
"""

# Each key maps to a (vx, vy, omega) tuple that is set directly.
KEY_BINDINGS = {
    'w': ( LINEAR_VEL,   0.0,          0.0         ),
    's': (-LINEAR_VEL,   0.0,          0.0         ),
    'a': ( 0.0,          LINEAR_VEL,   0.0         ),
    'd': ( 0.0,         -LINEAR_VEL,   0.0         ),
    'q': ( 0.0,          0.0,          ANGULAR_VEL ),
    'e': ( 0.0,          0.0,         -ANGULAR_VEL ),
}


class TeleopKeyNode(Node):

    def __init__(self):
        super().__init__('teleop_key')
        self._pub   = self.create_publisher(Twist, '/cmd_vel', 10)
        self._timer = self.create_timer(1.0 / PUBLISH_RATE, self._publish_cb)

        self._vx    = 0.0
        self._vy    = 0.0
        self._omega = 0.0
        self._last_key_time = 0.0   # monotonic time of last direction keypress

        print(HELP_TEXT)

    # ------------------------------------------------------------------
    # Terminal helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _get_key(timeout=0.05):
        """Read one character from stdin without blocking."""
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            return sys.stdin.read(1)
        return ''

    # ------------------------------------------------------------------
    # Timer callback
    # ------------------------------------------------------------------

    def _publish_cb(self):
        # Decay velocity to zero if no key has been pressed recently
        if time.monotonic() - self._last_key_time > KEY_TIMEOUT:
            if self._vx != 0.0 or self._vy != 0.0 or self._omega != 0.0:
                self._vx = self._vy = self._omega = 0.0
                print('\r[stopped]                             ',
                      end='', flush=True)

        msg = Twist()
        msg.linear.x  = self._vx
        msg.linear.y  = self._vy
        msg.angular.z = self._omega
        self._pub.publish(msg)

    # ------------------------------------------------------------------
    # Key processing
    # ------------------------------------------------------------------

    def _apply_key(self, key):
        if key == ' ':
            self._vx = self._vy = self._omega = 0.0
            self._last_key_time = 0.0   # force immediate decay lock
            print('\r[STOP]                                ',
                  end='', flush=True)
            return True

        if key in KEY_BINDINGS:
            self._vx, self._vy, self._omega = KEY_BINDINGS[key]
            self._last_key_time = time.monotonic()
            print(
                f'\rvx={self._vx:+.2f} m/s  '
                f'vy={self._vy:+.2f} m/s  '
                f'ω={self._omega:+.2f} rad/s   ',
                end='', flush=True,
            )
            return True

        return False

    # ------------------------------------------------------------------
    # Main run loop
    # ------------------------------------------------------------------

    def run(self):
        """Blocking run loop — call instead of rclpy.spin()."""
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setraw(sys.stdin.fileno())
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                key = self._get_key(timeout=0.05)

                if key in ('x', 'X', '\x03'):   # x or Ctrl-C
                    break

                if key:
                    self._apply_key(key)
                # No key → _publish_cb handles timeout-based decay

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            self._vx = self._vy = self._omega = 0.0
            self._publish_cb()
            print('\nTeleop node stopped.')


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
