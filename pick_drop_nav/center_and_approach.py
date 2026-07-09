#!/usr/bin/env python3

import rclpy, math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Float32, String
from geometry_msgs.msg import Twist


class CenterAndApproach(Node):

    PICK_LOWER   = 'lower_servo'
    PICK_ADVANCE = 'advance'
    PICK_RAISE   = 'raise_servo'
    PICK_REVERSE = 'reverse'
    PICK_DONE    = 'pick_done'

    DEP_LOWER    = 'dep_lower_servo'
    DEP_REVERSE  = 'dep_reverse'
    DEP_RAISE    = 'dep_raise_servo'
    DEP_DONE     = 'dep_done'

    SERVO_DOWN = 0.0    
    SERVO_UP   = -180.0  
    PICK_DIST  = 0.15
    PICK_SPEED = 0.06
    SERVO_WAIT = 2.0

    def __init__(self):
        super().__init__('center_and_approach')

        self.declare_parameter('stop_dist',    0.15)
        self.declare_parameter('center_thr',   0.05)
        self.declare_parameter('yaw_thr',      0.3)
        self.declare_parameter('k_offset',    0.3)
        self.declare_parameter('k_yaw',        0.8)
        self.declare_parameter('w_min',        0.10)
        self.declare_parameter('v_approach',   0.08)
        self.declare_parameter('brake_margin', 0.10)
        self.declare_parameter('timeout',      0.7)
        self.declare_parameter('lost_proceed_dist', 0.45)
        self.declare_parameter('pick_advance_dist', 0.15)
        self.declare_parameter('standalone',   False)

        self.stop_dist    = self.get_parameter('stop_dist').value
        self.center_thr   = self.get_parameter('center_thr').value
        self.yaw_thr      = self.get_parameter('yaw_thr').value
        self.k_offset     = self.get_parameter('k_offset').value
        self.k_yaw        = self.get_parameter('k_yaw').value
        self.w_min        = self.get_parameter('w_min').value
        self.v_approach   = self.get_parameter('v_approach').value
        self.brake_margin = self.get_parameter('brake_margin').value
        self.timeout      = self.get_parameter('timeout').value
        self.lost_proceed = self.get_parameter('lost_proceed_dist').value
        self.pick_adv     = self.get_parameter('pick_advance_dist').value
        self.standalone   = self.get_parameter('standalone').value

        self.create_subscription(
            Float32MultiArray, '/aruco_target', self._aruco_cb, 10)
        self.create_subscription(
            String, '/mission_state', self._mission_cb, 10)

        self.pub      = self.create_publisher(Twist,   '/cmd_vel',    10)
        self.pub_srv  = self.create_publisher(Float32, '/ServoAngle', 10)
        self.pub_stat = self.create_publisher(String,  '/ca_status',  10)

        self.create_timer(0.1, self._loop)

        self._offset_x = 0.0
        self._dist     = float('inf')
        self._yaw      = 0.0
        self._alpha_k  = 0.0
        self._stamp    = None
        self._yaw_buf  = []
        self._YAW_N    = 5

        self._phase          = 'idle'
        self._mission_active = self.standalone
        self._deposit_active = False
        self._pick_offset_x  = 0.0
        self._pick_step      = None
        self._pick_step_start = None

    def _mission_cb(self, msg: String):
        was_pick    = self._mission_active
        was_deposit = self._deposit_active

        self._mission_active = (msg.data == 'pick')
        self._deposit_active = (msg.data == 'deposit')

        if not self._mission_active and was_pick and self._phase not in ('picking',):
            self._stop()

        if not self._deposit_active and was_deposit and self._phase not in ('depositing',):
            self._stop()

        if self._mission_active and not was_pick:
            self._phase = 'idle'
            self._yaw_buf.clear()

        if self._deposit_active and not was_deposit:
            self._phase           = 'depositing'
            self._pick_step       = self.DEP_LOWER
            self._pick_step_start = self._now()

    def _aruco_cb(self, msg: Float32MultiArray):
        if len(msg.data) < 4:
            return
        self._offset_x = float(msg.data[0])
        self._dist     = float(msg.data[1])
        raw_yaw        = float(msg.data[2])
        self._alpha_k  = float(msg.data[3])
        self._stamp    = self.get_clock().now()

        self._yaw_buf.append(raw_yaw)
        if len(self._yaw_buf) > self._YAW_N:
            self._yaw_buf.pop(0)
        self._yaw = sum(self._yaw_buf) / len(self._yaw_buf)

        if self._phase == 'idle' and self._mission_active:
            self._phase = 'centering'

    def _aruco_visible(self):
        if self._stamp is None:
            return False
        return (self.get_clock().now() - self._stamp).nanoseconds / 1e9 < self.timeout

    def _apply_w_min(self, w):
        if 0 < abs(w) < self.w_min:
            w = math.copysign(self.w_min, w)
        return w

    def _publish(self, v, w):
        msg = Twist()
        msg.linear.x  = float(v)
        msg.angular.z = float(w)
        self.pub.publish(msg)

    def _stop(self):
        self._publish(0.0, 0.0)

    def _set_servo(self, angle):
        msg = Float32()
        msg.data = float(angle)
        self.pub_srv.publish(msg)
        self.get_logger().info(f'Servo → {angle}°')

    def _pub_status(self, status: str):
        msg = String()
        msg.data = status
        self.pub_stat.publish(msg)

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _loop(self):

        if self._phase == 'picking':
            self._pub_status('running')
            self._do_picking()
            return

        if self._phase == 'depositing':
            self._pub_status('running')
            self._do_depositing()
            return

        if self._phase == 'done':
            self._pub_status('done')
            return

        active = self._mission_active or self._deposit_active or self.standalone
        if not active:
            self._pub_status('idle')
            return 

        if not self._aruco_visible() and self._phase not in ('idle',):
            if (self._phase in ('approaching', 'aligning')
                    and self._dist <= self.lost_proceed):

                self._stop()
                self._pick_offset_x   = self._offset_x
                self._phase           = 'picking'
                self._pick_step       = self.PICK_LOWER
                self._pick_step_start = self._now()

                return

            self._phase = 'idle'
            self._stop()

        self._pub_status('running' if self._phase != 'idle' else 'idle')

        if   self._phase == 'idle':        pass
        elif self._phase == 'centering':   self._do_centering()
        elif self._phase == 'approaching': self._do_approaching()
        elif self._phase == 'aligning':    self._do_aligning()

    def _do_centering(self):
        if abs(self._offset_x) < self.center_thr:
            self._phase = 'approaching'

            return
        w = -self.k_offset * self._offset_x
        w = self._apply_w_min(w)
        self._publish(0.0, w)

    def _do_approaching(self):
        if self._dist <= self.stop_dist + self.brake_margin:
            self._stop()
            self._phase = 'aligning'

            return
        w = -self.k_offset * self._offset_x * 0.8
        w = self._apply_w_min(w) if abs(self._offset_x) > self.center_thr else w
        remaining = self._dist - self.stop_dist
        v = max(0.06, min(self.v_approach, 0.5 * remaining))
        self._publish(v, w)

    def _do_aligning(self):
        offset_ok = abs(self._offset_x) < self.center_thr
        yaw_ok    = abs(self._yaw)      < self.yaw_thr

        if offset_ok and yaw_ok:
            self._stop()
            self._pick_offset_x   = self._offset_x
            self._phase           = 'picking'
            self._pick_step       = self.PICK_LOWER
            self._pick_step_start = self._now()
            
            return

        yaw_weight = max(0.0, 1.0 - abs(self._offset_x) / 0.15)
        w = (-self.k_offset * self._offset_x
             + self.k_yaw * 0.3 * yaw_weight * self._yaw)
        w = max(-0.20, min(0.20, w))
        w = self._apply_w_min(w)
        self._publish(0.0, w)

    def _do_picking(self):
        now     = self._now()
        elapsed = now - self._pick_step_start

        if self._pick_step == self.PICK_LOWER:
            self._stop()
            self._set_servo(self.SERVO_DOWN)
            if elapsed >= self.SERVO_WAIT:
                self._pick_step       = self.PICK_ADVANCE
                self._pick_step_start = now


        elif self._pick_step == self.PICK_ADVANCE:
            dist_done = elapsed * self.PICK_SPEED
            if dist_done >= self.pick_adv:
                self._stop()
                self._pick_step       = self.PICK_RAISE
                self._pick_step_start = now

            else:
                w = -self.k_offset * self._pick_offset_x * 5.0
                self._publish(self.PICK_SPEED, w)

        elif self._pick_step == self.PICK_RAISE:
            self._stop()
            self._set_servo(self.SERVO_UP)
            if elapsed >= self.SERVO_WAIT:
                self._pick_step       = self.PICK_REVERSE
                self._pick_step_start = now

        elif self._pick_step == self.PICK_REVERSE:
            dist_done = elapsed * self.PICK_SPEED
            if dist_done >= self.PICK_DIST:
                self._stop()
                self._phase     = 'done'
                self._pick_step = self.PICK_DONE

            else:
                self._publish(-self.PICK_SPEED, 0.0)

    def _do_depositing(self):

        now     = self._now()
        elapsed = now - self._pick_step_start

        if self._pick_step == self.DEP_LOWER:
            self._stop()
            self._set_servo(self.SERVO_DOWN)
            if elapsed >= self.SERVO_WAIT:
                self._pick_step       = self.DEP_REVERSE
                self._pick_step_start = now


        elif self._pick_step == self.DEP_REVERSE:
            dist_done = elapsed * self.PICK_SPEED
            if dist_done >= self.PICK_DIST + 0.1:
                self._stop()
                self._pick_step       = self.DEP_RAISE
                self._pick_step_start = now

            else:
                self._publish(-self.PICK_SPEED, 0.0)

        elif self._pick_step == self.DEP_RAISE:
            self._stop()
            self._set_servo(self.SERVO_UP)
            if elapsed >= self.SERVO_WAIT:
                self._pick_step = self.DEP_DONE
                self._phase     = 'done'


def main(args=None):
    rclpy.init(args=args)
    node = CenterAndApproach()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()