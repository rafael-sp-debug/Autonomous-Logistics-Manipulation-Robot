#!/usr/bin/env python3

import rclpy, math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from turtlesim.msg import Pose
from std_msgs.msg import String, Float32
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import TransformStamped


class MissionCoordinator(Node):

    # Estados de la misión
    IDLE                = 'idle'
    NAVIGATE_TO_PICKUP  = 'navigate_to_pickup'
    PICK                = 'pick'
    NAVIGATE_TO_DROPOFF = 'navigate_to_dropoff'
    DEPOSIT             = 'deposit'
    NAVIGATE_TO_ORIGIN  = 'navigate_to_origin'
    DONE                = 'done'

    def __init__(self):
        super().__init__('mission_coordinator')

        self.declare_parameter('pickup_x',           2.0)
        self.declare_parameter('pickup_y',           0.0)
        self.declare_parameter('dropoff_x',          2.0)
        self.declare_parameter('dropoff_y',          2.0)
        self.declare_parameter('nav_tolerance',      0.25) 
        self.declare_parameter('ca_trigger_dist',    1.0)  
        self.declare_parameter('servo_deposit_angle', 0.0)
        self.declare_parameter('servo_carry_angle',  -180.0)
        self.declare_parameter('servo_wait',          2.0)   

        self.pickup_x    = self.get_parameter('pickup_x').value
        self.pickup_y    = self.get_parameter('pickup_y').value
        self.dropoff_x   = self.get_parameter('dropoff_x').value
        self.dropoff_y   = self.get_parameter('dropoff_y').value
        self.nav_tol     = self.get_parameter('nav_tolerance').value
        self.ca_trig     = self.get_parameter('ca_trigger_dist').value
        self.srv_deposit = self.get_parameter('servo_deposit_angle').value
        self.srv_carry   = self.get_parameter('servo_carry_angle').value
        self.srv_wait    = self.get_parameter('servo_wait').value

        self.pub_target  = self.create_publisher(Pose,   '/target',        10)
        self.pub_state   = self.create_publisher(String, '/mission_state', 10)
        self.pub_servo   = self.create_publisher(Float32,'/ServoAngle',    10)

        self.create_subscription(Odometry,         '/odom',          self._odom_cb,    10)
        self.create_subscription(String,            '/ca_status',     self._ca_cb,      10)
        self.create_subscription(Float32MultiArray, '/aruco_target',  self._aruco_cb,   10)

        self._state        = self.IDLE
        self._pose         = None          
        self._ca_status    = 'idle'        
        self._deposit_t    = None          
        self._prev_state   = None
        self._aruco_stamp  = None          
        self._aruco_dist   = float('inf')    
        self._ARUCO_TIMEOUT = 2.0          

        self.create_timer(0.2, self._tick)

    def _odom_cb(self, msg: Odometry):
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        q   = msg.pose.pose.orientation
        yaw = math.atan2(
            2*(q.w*q.z + q.x*q.y),
            1 - 2*(q.y*q.y + q.z*q.z))
        self._pose = [x, y, yaw]

        if self._state == self.IDLE:
            self._transition(self.NAVIGATE_TO_PICKUP)

    def _ca_cb(self, msg: String):
        self._ca_status = msg.data

    def _aruco_cb(self, msg):
        if len(msg.data) >= 2:
            self._aruco_stamp = self._now()
            self._aruco_dist  = float(msg.data[1])   # d_k

    def _tick(self):
        if self._pose is None:
            return

        self._publish_state()

        if self._state == self.NAVIGATE_TO_PICKUP:
            self._send_target(self.pickup_x, self.pickup_y)
            near_pickup = self._dist_to(self.pickup_x, self.pickup_y) < self.ca_trig
            aruco_seen  = self._aruco_visible()

            if near_pickup and aruco_seen:

                self._send_target(self._pose[0], self._pose[1])
                
                self._transition(self.PICK)
            
        elif self._state == self.PICK:

            if self._ca_status == 'done':
                self._transition(self.NAVIGATE_TO_DROPOFF)

        elif self._state == self.NAVIGATE_TO_DROPOFF:
            self._send_target(self.dropoff_x, self.dropoff_y)
            if self._dist_to(self.dropoff_x, self.dropoff_y) < self.nav_tol:
                self._send_target(self._pose[0], self._pose[1])
                self._deposit_t = self._now()
                self._set_servo(self.srv_deposit)
                self._transition(self.DEPOSIT)

        elif self._state == self.DEPOSIT:
            if self._now() - self._deposit_t >= self.srv_wait:
                self._set_servo(self.srv_carry)   
                self._transition(self.NAVIGATE_TO_ORIGIN)

        elif self._state == self.NAVIGATE_TO_ORIGIN:
            self._send_target(0.0, 0.0)
            if self._dist_to(0.0, 0.0) < self.nav_tol:
                self._send_target(0.0, 0.0)
                self._transition(self.DONE)

        elif self._state == self.DONE:
            pass

    def _transition(self, new_state: str):
        self._state = new_state

    def _dist_to(self, x: float, y: float) -> float:
        dx = x - self._pose[0]
        dy = y - self._pose[1]
        return math.sqrt(dx*dx + dy*dy)

    def _send_target(self, x: float, y: float):
        msg      = Pose()
        msg.x    = float(x)
        msg.y    = float(y)
        msg.theta = 0.0
        self.pub_target.publish(msg)

    def _set_servo(self, angle: float):
        msg      = Float32()
        msg.data = float(angle)
        self.pub_servo.publish(msg)
        self.get_logger().info(f'Servo → {angle}°')

    def _publish_state(self):
        msg      = String()
        msg.data = self._state
        self.pub_state.publish(msg)

    def _aruco_visible(self) -> bool:
        if self._aruco_stamp is None:
            return False
        return self._now() - self._aruco_stamp < self._ARUCO_TIMEOUT

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None):
    rclpy.init(args=args)
    node = MissionCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()