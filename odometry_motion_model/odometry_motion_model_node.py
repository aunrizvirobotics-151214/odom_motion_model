#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, Quaternion, Point
from tf_transformations import euler_from_quaternion, quaternion_from_euler


def inverse_motion_model(pose_t_1, pose_t):
    x0, y0, th0 = pose_t_1
    x1, y1, th1 = pose_t

    delta_trans = np.sqrt((x1 - x0) ** 2 + (y1 - y0) ** 2)
    delta_rot1  = _wrap(np.arctan2(y1 - y0, x1 - x0) - th0)

    if abs(delta_rot1) > np.pi / 2:
        delta_trans = -delta_trans
        delta_rot1  = _wrap(delta_rot1 - np.pi)

    delta_rot2 = _wrap(th1 - th0 - delta_rot1)
    return delta_rot1, delta_trans, delta_rot2


def probability_density(mean, variance):
    variance = max(variance, 1e-5)
    return (1.0 / np.sqrt(2.0 * np.pi * variance)) * np.exp(-0.5 * mean ** 2 / variance)


def motion_model(x_t, x_t_1, u_t, alpha, marginalise_p3=False):
    r1h, dth, r2h = inverse_motion_model(u_t[0], u_t[1])
    r1,  dt,  r2  = inverse_motion_model(x_t_1, x_t)
    p1 = probability_density(r1h - r1, alpha[0]*r1**2 + alpha[1]*dt**2)
    p2 = probability_density(dth - dt, alpha[2]*dt**2 + alpha[3]*(r1**2 + r2**2))
    p3 = probability_density(r2h - r2, alpha[0]*r2**2 + alpha[1]*dt**2)
    return p1 * p2 if marginalise_p3 else p1 * p2 * p3


def _get_sample(std):
    if std < 1e-9:
        return 0.0
    return 0.5 * sum(np.random.uniform(-std, std) for _ in range(12))


def sample_motion_model(pose_t_1, u_t, alpha):
    r1h, dth, r2h = inverse_motion_model(u_t[0], u_t[1])
    
    r1  = r1h  - _get_sample(np.sqrt(alpha[0]*r1h**2 + alpha[1]*dth**2))
    dt  = dth  - _get_sample(np.sqrt(alpha[2]*dth**2  + alpha[3]*(r1h**2 + r2h**2)))
    r2  = r2h  - _get_sample(np.sqrt(alpha[0]*r2h**2 + alpha[1]*dth**2))

    x0, y0, th0 = pose_t_1
    return x0 + dt*np.cos(th0+r1), y0 + dt*np.sin(th0+r1), _wrap(th0+r1+r2)


def _wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))

def _yaw_to_quat(yaw):
    q = quaternion_from_euler(0.0, 0.0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

def _odom_to_pose(msg):
    p = msg.pose.pose
    _, _, yaw = euler_from_quaternion([p.orientation.x, p.orientation.y,
                                       p.orientation.z, p.orientation.w])
    return [p.position.x, p.position.y, yaw]


class OdometryMotionModelNode(Node):

    def __init__(self):
        super().__init__('odometry_motion_model')

        self.declare_parameter('alpha1',         0.1)
        self.declare_parameter('alpha2',         0.1)
        self.declare_parameter('alpha3',         0.01)
        self.declare_parameter('alpha4',         0.01)
        self.declare_parameter('num_samples',    1000)
        self.declare_parameter('marginalise_p3', False)
        self.declare_parameter('fixed_frame',    'odom')

        self.alpha = [self.get_parameter(f'alpha{i}').value for i in range(1, 5)]
        self.N     = int(self.get_parameter('num_samples').value)
        self.marg  = self.get_parameter('marginalise_p3').value
        self.frame = self.get_parameter('fixed_frame').value

        self.prev_odom_pose = None
        self.prev_map_pose  = None
        self.samples        = None

        self.warmup_count = 0
        self.WARMUP_N     = 50

        self.odom_path           = Path()
        self.estimated_path      = Path()
        self.odom_path.header.frame_id      = self.frame
        self.estimated_path.header.frame_id = self.frame

        qos_latched = QoSProfile(depth=10,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.pub_pose      = self.create_publisher(PoseStamped, '/estimated_pose', 10)
        self.pub_cloud     = self.create_publisher(PoseArray,   '/particle_cloud', 10)
        self.pub_path_odom = self.create_publisher(Path,        '/odom_path',      qos_latched)
        self.pub_path_est  = self.create_publisher(Path,        '/particle_path',  qos_latched)

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.get_logger().info('OdometryMotionModelNode ready.')

    def _odom_cb(self, msg: Odometry):
        now          = msg.header.stamp
        current_pose = _odom_to_pose(msg)

        # slide anchor through warmup so it reflects a settled pose
        if self.warmup_count < self.WARMUP_N:
            self.warmup_count   += 1
            self.prev_odom_pose  = current_pose
            return

        # first post-warmup message: init particles and publish immediately
        if self.samples is None:
            self.samples        = np.zeros((self.N, 3))
            self.samples[:, 0]  = current_pose[0]
            self.samples[:, 1]  = current_pose[1]
            self.samples[:, 2]  = current_pose[2]
            self.prev_odom_pose = current_pose
            self.prev_map_pose  = current_pose[:]
            self._append_odom_path(now, current_pose)
            self._publish_all(now, current_pose)
            return

        # gate on velocity — ignore if robot is still moving or coasting
        linear_vel  = np.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
        angular_vel = abs(msg.twist.twist.angular.z)
        if linear_vel < 0.01 and angular_vel < 0.01:
            return

        # pose diff threshold — only propagate meaningful motion
        dx  = current_pose[0] - self.prev_odom_pose[0]
        dy  = current_pose[1] - self.prev_odom_pose[1]
        dth = _wrap(current_pose[2] - self.prev_odom_pose[2])
        if np.sqrt(dx**2 + dy**2) < 5e-3 and abs(dth) < 1.75e-3:
            return

        u_t = [self.prev_odom_pose, current_pose]
        self.prev_odom_pose = current_pose

        new_samples = np.zeros_like(self.samples)
        for i in range(self.N):
            new_samples[i] = sample_motion_model(list(self.samples[i]), u_t, self.alpha)
        self.samples = new_samples

        mx  = np.mean(self.samples[:, 0])
        my  = np.mean(self.samples[:, 1])
        mth = _wrap(np.arctan2(np.mean(np.sin(self.samples[:, 2])),
                               np.mean(np.cos(self.samples[:, 2]))))
        estimated_pose     = [mx, my, mth]
        self.prev_map_pose = estimated_pose

        self._append_odom_path(now, current_pose)
        self._append_estimated_path(now, estimated_pose)
        self._publish_all(now, estimated_pose)

    def _publish_all(self, stamp, pose):
        self._publish_pose(stamp, pose)
        self._publish_cloud(stamp)
        self._publish_paths(stamp)

    def _publish_pose(self, stamp, pose):
        msg = PoseStamped()
        msg.header.stamp     = stamp
        msg.header.frame_id  = self.frame
        msg.pose.position    = Point(x=pose[0], y=pose[1], z=0.0)
        msg.pose.orientation = _yaw_to_quat(pose[2])
        self.pub_pose.publish(msg)

    def _publish_cloud(self, stamp):
        msg = PoseArray()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.frame
        for s in self.samples:
            p = Pose()
            p.position    = Point(x=float(s[0]), y=float(s[1]), z=0.0)
            p.orientation = _yaw_to_quat(float(s[2]))
            msg.poses.append(p)
        self.pub_cloud.publish(msg)

    def _append_odom_path(self, stamp, pose):
        ps = PoseStamped()
        ps.header.stamp     = stamp
        ps.header.frame_id  = self.frame
        ps.pose.position    = Point(x=pose[0], y=pose[1], z=0.0)
        ps.pose.orientation = _yaw_to_quat(pose[2])
        self.odom_path.header.stamp = stamp
        self.odom_path.poses.append(ps)

    def _append_estimated_path(self, stamp, pose):
        ps = PoseStamped()
        ps.header.stamp     = stamp
        ps.header.frame_id  = self.frame
        ps.pose.position    = Point(x=pose[0], y=pose[1], z=0.0)
        ps.pose.orientation = _yaw_to_quat(pose[2])
        self.estimated_path.header.stamp = stamp
        self.estimated_path.poses.append(ps)

    def _publish_paths(self, stamp):
        self.odom_path.header.stamp      = stamp
        self.estimated_path.header.stamp = stamp
        self.pub_path_odom.publish(self.odom_path)
        self.pub_path_est.publish(self.estimated_path)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryMotionModelNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()