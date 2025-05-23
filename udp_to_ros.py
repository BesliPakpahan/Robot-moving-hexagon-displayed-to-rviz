#!/usr/bin/env python3
import rospy
import socket
import math
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf.transformations import quaternion_from_euler
import time

class HexagonPathVisualizer:
    def _init_(self):  # ← ← ← Perbaikan di sini
        rospy.init_node('hexagon_path_visualizer')

        # UDP Configuration
        self.udp_ip = "0.0.0.0"
        self.udp_port = 4210
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.01)
        self.sock.bind((self.udp_ip, self.udp_port))

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = time.time()
        # self.speed = 0.1  # m/s
        # self.side_length = 1.0
        self.speed = 1  # m/s
        self.side_length = 1.5

        # Hexagon tracking
        self.expected_angles = [60 * i for i in range(6)]
        self.current_side = 0
        self.hexagon_complete = False
        self.side_start_positions = [(0.0, 0.0)] * 6
        self.side_completion_threshold = 0.8

        # ROS Publishers
        self.pose_pub = rospy.Publisher('/robot_pose', PoseStamped, queue_size=10)
        self.path_pub = rospy.Publisher('/robot_path', Path, queue_size=10, latch=True)

        # Path message
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
        self.path_seq = 0

        rospy.loginfo("Hexagon Path Visualizer initialized")

    def update_robot_pose(self, yaw_deg, is_moving):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        if is_moving and dt > 0:
            yaw_rad = math.radians(yaw_deg)
            self.x += self.speed * math.cos(yaw_rad) * dt
            self.y += self.speed * math.sin(yaw_rad) * dt

        self.yaw = yaw_deg

        if not self.hexagon_complete and self.current_side < 6:
            start_x, start_y = self.side_start_positions[self.current_side]
            distance = math.sqrt((self.x - start_x)*2 + (self.y - start_y)*2)  # ← ← ← Perbaikan

            if distance >= self.side_length * self.side_completion_threshold:
                rospy.loginfo(f"Completing side {self.current_side+1} at ({self.x:.2f}, {self.y:.2f})")
                self.complete_current_side()

    def complete_current_side(self):
        corner_pose = self.create_pose_message()
        self.path_msg.poses.append(corner_pose)

        self.current_side += 1
        if self.current_side < 6:
            self.side_start_positions[self.current_side] = (self.x, self.y)
            rospy.loginfo(f"Starting side {self.current_side+1} at ({self.x:.2f}, {self.y:.2f})")
        else:
            self.hexagon_complete = True
            rospy.loginfo("Hexagon path completed!")

        self.publish_path()

    def create_pose_message(self):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.header.seq = self.path_seq
        pose.pose.position.x = self.x
        pose.pose.position.y = self.y
        pose.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, math.radians(self.yaw))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        return pose

    def publish_path(self):
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_msg.header.seq = self.path_seq
        self.path_pub.publish(self.path_msg)
        self.path_seq += 1

    def parse_udp_message(self, data):
        try:
            decoded = data.decode().strip()
            if decoded.startswith("ANGLE:"):
                parts = decoded.split(',')
                yaw_deg = float(parts[0].split(':')[1])
                is_moving = parts[1].split(':')[1] == "1" if len(parts) > 1 else True
                return yaw_deg, is_moving
        except Exception as e:
            rospy.logwarn(f"UDP parse error: {e}")
        return None, None

    def distance_to_last_pose(self, new_pose):
        if not self.path_msg.poses:
            return float('inf')
        last = self.path_msg.poses[-1]
        return math.sqrt((new_pose.pose.position.x - last.pose.position.x)**2 +
                         (new_pose.pose.position.y - last.pose.position.y)**2)

    def run(self):
        self.side_start_positions[0] = (self.x, self.y)
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            try:
                data, _ = self.sock.recvfrom(1024)
                yaw_deg, is_moving = self.parse_udp_message(data)

                if yaw_deg is not None:
                    self.update_robot_pose(yaw_deg, is_moving)
                    pose = self.create_pose_message()
                    self.pose_pub.publish(pose)

                    if not self.path_msg.poses or self.distance_to_last_pose(pose) > 0.05:
                        self.path_msg.poses.append(pose)
                        self.publish_path()

            except socket.timeout:
                if rospy.get_param("~simulate", False):
                    if not self.hexagon_complete:
                        simulated_angle = self.expected_angles[self.current_side % 6]
                        self.update_robot_pose(simulated_angle, True)
                        pose = self.create_pose_message()
                        self.pose_pub.publish(pose)
                        if not self.path_msg.poses or self.distance_to_last_pose(pose) > 0.05:
                            self.path_msg.poses.append(pose)
                            self.publish_path()
                        rospy.sleep(1.0)
            except Exception as e:
                rospy.logerr(f"Error: {e}")

            rate.sleep()

if _name_ == '_main_':
    visualizer = None
    try:
        visualizer = HexagonPathVisualizer()
        rospy.set_param('~simulate', False)
        visualizer.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if visualizer and hasattr(visualizer, 'sock'):
            visualizer.sock.close()
