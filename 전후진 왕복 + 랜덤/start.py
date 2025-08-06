import sys
import threading
import time
import random
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout
import rospy
from geometry_msgs.msg import Twist
from pymycobot.mycobot import MyCobot

# 전역 변수
stop_flag = False
cmd_vel_pub = None
mc = None

def key_input_thread():
    global stop_flag
    while not stop_flag:
        time.sleep(0.1)
        
def move_agv_loop():
    global stop_flag, cmd_vel_pub
    rate = rospy.Rate(10)
    print("[AGV] Start")

    while not rospy.is_shutdown() and not stop_flag:
        # Forward 
        move_cmd = Twist()
        move_cmd.linear.x = 0.1
        start_time = time.time()
        while time.time() - start_time < 2 and not stop_flag:
            cmd_vel_pub.publish(move_cmd)
            rate.sleep()

        cmd_vel_pub.publish(Twist())  # 정지
        time.sleep(2)

        # Backward
        move_cmd.linear.x = -0.1
        start_time = time.time()
        while time.time() - start_time < 2 and not stop_flag:
            cmd_vel_pub.publish(move_cmd)
            rate.sleep()

        cmd_vel_pub.publish(Twist())  # 정지
        time.sleep(2)

    cmd_vel_pub.publish(Twist())  # 마지막 정지
    print("[AGV] Stop")

def move_arm_loop():
    global stop_flag, mc
    print("[ARM] Start")
    while not stop_flag:
        angles = [random.randint(-45, 45) for _ in range(6)]
        print(f"[ARM] Sending angles: {angles}")
        mc.send_angles(angles, 80)
        time.sleep(3)
    print("[ARM] Stop")

class ControlApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AGV + ARM Controller")
        self.layout = QVBoxLayout()

        self.start_button = QPushButton("▶ 실행")
        self.stop_button = QPushButton("■ 중지")

        self.start_button.clicked.connect(self.start_motion)
        self.stop_button.clicked.connect(self.stop_motion)

        self.layout.addWidget(self.start_button)
        self.layout.addWidget(self.stop_button)
        self.setLayout(self.layout)

        self.arm_thread = None
        self.agv_thread = None

    def start_motion(self):
        global stop_flag
        stop_flag = False
        self.arm_thread = threading.Thread(target=move_arm_loop)
        self.agv_thread = threading.Thread(target=move_agv_loop)
        self.arm_thread.start()
        self.agv_thread.start()

    def stop_motion(self):
        global stop_flag
        stop_flag = True
        print("[System] 중지 명령 전달됨")

if __name__ == "__main__":
    rospy.init_node('agv_arm_gui', anonymous=False)
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    try:
        mc = MyCobot('/dev/ttyACM0')
        time.sleep(2)
        if not mc.is_controller_connected():
            print("[ARM] 연결 실패: 전원 확인")
            sys.exit(1)
        print("[ARM] 연결 성공")
    except Exception as e:
        print(f"[ARM] 연결 오류: {e}")
        sys.exit(1)

    app = QApplication(sys.argv)
    win = ControlApp()
    win.show()
    sys.exit(app.exec_())
