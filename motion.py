#!/usr/bin/env python3
import rospy
import socket
import re
from std_msgs.msg import String

class MotionCommandSender:
    def _init_(self):
        rospy.init_node('motion_command_sender')

        # Ambil parameter dengan validasi
        # self.udp_ip = self.validate_ip(rospy.get_param('~udp_ip', '192.168.63.213'))
        self.udp_ip = self.validate_ip(rospy.get_param('~udp_ip', '192.168.241.157'))
        self.udp_port = self.validate_port(rospy.get_param('~udp_port', 4211))

        # Konfigurasi socket dengan timeout
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.settimeout(0.5)  # Timeout 500ms

        # Antrian pesan dan flag koneksi
        self.command_queue = []
        self.connection_active = False

        # Subscriber dan Timer
        rospy.Subscriber('/motion_command', String, self.command_callback)
        rospy.Timer(rospy.Duration(0.1), self.process_queue)  # Proses antrian tiap 100ms

        rospy.on_shutdown(self.cleanup)
        rospy.loginfo(f"UDP sender ready. Target: {self.udp_ip}:{self.udp_port}")
        rospy.spin()

    def validate_ip(self, ip):
        """Validasi format alamat IP"""
        if not re.match(r'^\d{1,3}(\.\d{1,3}){3}$', ip):
            raise ValueError(f"Invalid IP address format: {ip}")
        return ip

    def validate_port(self, port):
        """Validasi nomor port"""
        if not 0 < port < 65535:
            raise ValueError(f"Invalid port number: {port}")
        return port

    def command_callback(self, msg):
        """Handle incoming ROS commands"""
        try:
            command = msg.data.strip()
            if self.validate_command(command):
                self.command_queue.append(command)
                rospy.logdebug(f"Queued command: {command}")
            else:
                rospy.logwarn(f"Invalid command rejected: {command}")
        except Exception as e:
            rospy.logerr(f"Command processing error: {str(e)}")

    def validate_command(self, command):
        """Validasi format perintah dasar"""
        valid_commands = ['START', 'STOP', 'RESET', 'SPEED:']
        return any(command.startswith(cmd) for cmd in valid_commands)

    def process_queue(self, event):
        """Proses antrian pesan dengan retry"""
        while self.command_queue:
            command = self.command_queue.pop(0)
            try:
                self.sock.sendto(command.encode('utf-8'), (self.udp_ip, self.udp_port))
                rospy.loginfo(f"Success: {command}")
                self.connection_active = True
            except socket.timeout:
                rospy.logwarn("Timeout - Retrying...")
                self.command_queue.insert(0, command)  # Masukkan kembali ke antrian
                self.connection_active = False
                break
            except Exception as e:
                rospy.logerr(f"Critical error: {str(e)}")
                self.connection_active = False
                break

    def cleanup(self):
        """Shutdown handler"""
        self.sock.close()
        rospy.loginfo("Clean shutdown completed")

if _name_ == '_main_':
    try:
        MotionCommandSender()
    except rospy.ROSInterruptException:
        pass
