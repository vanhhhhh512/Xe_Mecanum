#!/usr/bin/env python3
import select
import sys
import termios
import tty
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class MecanumControl(Node):
    def __init__(self):
        super().__init__('mecanum_control', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)
        ])
        self.pub_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.pub_joint = self.create_publisher(JointState, 'joint_states_numpad', 10)
        self.pub_traj = self.create_publisher(JointTrajectory, 'set_joint_trajectory', 10)
        
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(JointState, '/joint_states', self.encoder_callback, 10)
        
        self.real_positions = {}
        self.real_velocities = {
            'Link1_joint': 0.0, 'Link2_joint': 0.0,
            'Banhtraitruoc_joint': 0.0, 'Banhtruocphai_joint': 0.0, 
            'Banhsautrai_joint': 0.0, 'Banhsauphai_joint': 0.0
        }
        
        self.last_joint_time = self.get_clock().now()
        self.last_positions = {'Link1_joint': 0.0, 'Link2_joint': 0.0}
        
        self.timer = self.create_timer(0.02, self.loop)

        self.vx, self.vy, self.wz = 0.0, 0.0, 0.0
        self.v_j1, self.v_j2 = 0.0, 0.0
        self.j1, self.j2 = 0.0, 0.0
        
        self.current_display = "Chưa có lệnh nào"
        self.speed = 0.512
        
        self.dir_fl = -1.0  
        self.dir_fr = 1.0   
        self.dir_rl = 1.0   
        self.dir_rr = -1.0  
        
        self.wheel_names = ['Banhtraitruoc_joint', 'Banhtruocphai_joint', 'Banhsautrai_joint', 'Banhsauphai_joint']
        self.all_joint_names = ['Link1_joint', 'Link2_joint'] + self.wheel_names
        self.wheel_positions = [0.0, 0.0, 0.0, 0.0]
        
        self.wheel_radius = 0.02
        self.base_length = 0.16
        self.base_width = 0.275
        
        self.last_time = self.get_clock().now()
        self.last_key_time = self.get_clock().now()
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.print_static_menu()

    def print_static_menu(self):
        sys.stdout.write("\033[H\033[J") 
        print("======================================================================")
        print("   HỆ THỐNG ĐIỀU KHIỂN & ĐO VẬN TỐC BÁNH XE TỪ ENCODER")
        print("======================================================================")
        print(" HƯỚNG DI CHUYỂN (1-9)   |   TAY MÁY & XOAY")
        print("   [7]   [8]   [9]       |   Q/E: Xoay trái/phải")
        print("   [4]   [5]   [6]       |   U/I: Link 1 Phải/Trái")
        print("   [1]   [2]   [3]       |   J/K: Link 2 Xuống/Lên")
        print("----------------------------------------------------------------------")

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.02)
        key = sys.stdin.read(1) if rlist else ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def encoder_callback(self, msg):
        now = self.get_clock().now()
        dt = (now - self.last_joint_time).nanoseconds * 1e-9
        self.last_joint_time = now

        for i, name in enumerate(msg.name):
            pos = msg.position[i] if len(msg.position) > i else 0.0
            self.real_positions[name] = pos
            
            if name in ['Link1_joint', 'Link2_joint']:
                if dt > 0:
                    raw_vel = (pos - self.last_positions[name]) / dt
                    self.real_velocities[name] = 0.7 * self.real_velocities[name] + 0.3 * raw_vel
                self.last_positions[name] = pos

    def odom_callback(self, msg):
        odom_vx = msg.twist.twist.linear.x
        odom_vy = msg.twist.twist.linear.y
        odom_wz = msg.twist.twist.angular.z
        
        L, W, r = self.base_length/2.0, self.base_width/2.0, self.wheel_radius
        f = (1.0 / r)
        
        self.real_velocities['Banhtraitruoc_joint'] = f * (odom_vx - odom_vy - (L+W)*odom_wz) * self.dir_fl
        self.real_velocities['Banhtruocphai_joint'] = f * (odom_vx + odom_vy + (L+W)*odom_wz) * self.dir_fr
        self.real_velocities['Banhsautrai_joint'] = f * (odom_vx + odom_vy - (L+W)*odom_wz) * self.dir_rl
        self.real_velocities['Banhsauphai_joint'] = f * (odom_vx - odom_vy + (L+W)*odom_wz) * self.dir_rr

    def loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        
        if dt <= 0: return

        if abs(self.vx) < 0.001: self.vx = 0.0
        if abs(self.vy) < 0.001: self.vy = 0.0
        if abs(self.wz) < 0.001: self.wz = 0.0
        if abs(self.v_j1) < 0.01: self.v_j1 = 0.0
        if abs(self.v_j2) < 0.01: self.v_j2 = 0.0

        self.j1 = max(-3.14, min(3.14, self.j1 + self.v_j1 * dt))
        self.j2 = max(-3.99, min(3.99, self.j2 + self.v_j2 * dt))
        
        L, W, r = self.base_length/2.0, self.base_width/2.0, self.wheel_radius
        f = (1.0 / r)
        
        vel_fl = f * (self.vx - self.vy - (L+W)*self.wz)
        vel_fr = f * (self.vx + self.vy + (L+W)*self.wz)
        vel_rl = f * (self.vx + self.vy - (L+W)*self.wz)
        vel_rr = f * (self.vx - self.vy + (L+W)*self.wz)
        
        w_vels = [
            vel_fl * self.dir_fl,
            vel_fr * self.dir_fr,
            vel_rl * self.dir_rl,
            vel_rr * self.dir_rr
        ]
        
        if any(abs(v) > 0.001 for v in [self.vx, self.vy, self.wz]):
            for i in range(4):
                self.wheel_positions[i] += w_vels[i] * dt
        else:
            w_vels = [0.0, 0.0, 0.0, 0.0]

        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = self.vx, self.vy, self.wz
        self.pub_vel.publish(t)

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = self.all_joint_names
        js.position = [float(self.j1), float(self.j2)] + [float(p) for p in self.wheel_positions]
        js.velocity = [float(self.v_j1), float(self.v_j2)] + [float(v) for v in w_vels]
        self.pub_joint.publish(js)
        
        traj = JointTrajectory()
        traj.header.stamp = now.to_msg()
        traj.header.frame_id = 'base_link'
        traj.joint_names = self.all_joint_names
        p = JointTrajectoryPoint()
        p.positions = [float(self.j1), float(self.j2)] + [float(pos) for pos in self.wheel_positions]
        p.velocities = [float(self.v_j1), float(self.v_j2)] + [float(vel) for vel in w_vels]
        
        p.time_from_start.sec = 0
        p.time_from_start.nanosec = 20000000 
        
        traj.points.append(p)
        self.pub_traj.publish(traj)

        def get_wheel_state(v):
            if v > 0.01: return "Tiến"
            elif v < -0.01: return "Lùi"
            else: return "Dừng"

        def get_link1_state(v):
            if v > 0.01: return "Phải"
            elif v < -0.01: return "Trái"
            else: return "Dừng"

        def get_link2_state(v):
            if v > 0.01: return "Xuống"
            elif v < -0.01: return "Lên"
            else: return "Dừng"

        states_dict = {
            'Link1_joint': get_link1_state(self.v_j1),
            'Link2_joint': get_link2_state(self.v_j2),
            'Banhtraitruoc_joint': get_wheel_state(w_vels[0] * self.dir_fl),
            'Banhtruocphai_joint': get_wheel_state(w_vels[1] * self.dir_fr),
            'Banhsautrai_joint': get_wheel_state(w_vels[2] * self.dir_rl),
            'Banhsauphai_joint': get_wheel_state(w_vels[3] * self.dir_rr)
        }

        display_text = f"\033[9;1H\033[J" 
        display_text += f">>> Lệnh hiện tại: {self.current_display}\n"
        display_text += "="*70 + "\n"
        display_text += f" {'TÊN KHỚP':<20} | {'LỆNH':<6} | {'GÓC THỰC (rad)':<16} | {'V.TỐC (rad/s)'}\n"
        display_text += "-"*70 + "\n"
        
        for name in self.all_joint_names:
            pos = self.real_positions.get(name, 0.0)
            vel = self.real_velocities.get(name, 0.0)
            
            if name in ['Banhtraitruoc_joint', 'Banhsauphai_joint']:
                pos = -pos
                vel = -vel
                
            vel_str = f"{vel:>13.4f}" if abs(vel) < 0.0001 else f"{vel:>+13.4f}"
            display_text += f" {name:<20} | {states_dict[name]:<6} | {pos:>16.4f} | {vel_str}\n"

        display_text += "="*70 + "\n"

        if display_text != getattr(self, 'last_full_display', ''):
            sys.stdout.write(display_text)
            sys.stdout.flush()
            self.last_full_display = display_text

    def update_control(self, key):
        now = self.get_clock().now()
        if key:
            self.last_key_time = now
            k = key.lower()
            self.vx = self.vy = self.wz = self.v_j1 = self.v_j2 = 0.0
            
            if k == '8': self.vx, self.vy, self.wz, msg = self.speed, 0.0, 0.0, "Nút 8: ĐI THẲNG"
            elif k == '2': self.vx, self.vy, self.wz, msg = -self.speed, 0.0, 0.0, "Nút 2: ĐI LÙI"
            elif k == '4': self.vx, self.vy, self.wz, msg = 0.0, self.speed, 0.0, "Nút 4: SANG TRÁI"
            elif k == '6': self.vx, self.vy, self.wz, msg = 0.0, -self.speed, 0.0, "Nút 6: SANG PHẢI"
            elif k == '7': self.vx, self.vy, self.wz, msg = self.speed, self.speed, 0.0, "Nút 7: CHÉO TRÁI LÊN"
            elif k == '9': self.vx, self.vy, self.wz, msg = self.speed, -self.speed, 0.0, "Nút 9: CHÉO PHẢI LÊN"
            elif k == '1': self.vx, self.vy, self.wz, msg = -self.speed, self.speed, 0.0, "Nút 1: CHÉO TRÁI XUỐNG"
            elif k == '3': self.vx, self.vy, self.wz, msg = -self.speed, -self.speed, 0.0, "Nút 3: CHÉO PHẢI XUỐNG"
            elif k == '5': self.vx, self.vy, self.wz, msg = 0.0, 0.0, 0.0, "Nút 5: DỪNG"
            elif k in ['q']: self.vx, self.vy, self.wz, msg = 0.0, 0.0, 1.8, "Nút Q: XOAY TRÁI TẠI CHỖ"
            elif k in ['e']: self.vx, self.vy, self.wz, msg = 0.0, 0.0, -1.8, "Nút E: XOAY PHẢI TẠI CHỖ"
            
            elif k in ['u']: self.v_j1, msg = 1.5, "Phím U: Link 1 Xoay Phải"
            elif k in ['i']: self.v_j1, msg = -1.5, "Phím I: Link 1 Xoay Trái"
            elif k == 'j': self.v_j2, msg = 1.5, "Phím J: Link 2 Xuống"
            elif k == 'k': self.v_j2, msg = -1.5, "Phím K: Link 2 Lên"
            else: msg = f"Phím lạ ({k})"
            self.current_display = msg
        else:
            if (now - self.last_key_time).nanoseconds * 1e-9 > 0.12:
                self.vx = self.vy = self.wz = self.v_j1 = self.v_j2 = 0.0
                self.current_display = "ĐANG DỪNG"

def main():
    rclpy.init(); node = MecanumControl()
    try:
        while rclpy.ok():
            key = node.get_key(); node.update_control(key); rclpy.spin_once(node, timeout_sec=0.01)
    finally:
        node.pub_vel.publish(Twist()); termios.tcsetattr(sys.stdin, termios.TCSADRAIN, node.settings)
        sys.stdout.write("\033[H\033[J")
        sys.stdout.flush()
        node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__': main()