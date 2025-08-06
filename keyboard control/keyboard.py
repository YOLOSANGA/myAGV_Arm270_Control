import sys
import threading
import time
import random
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout
from PyQt5.QtCore import Qt
import rospy
from geometry_msgs.msg import Twist
from pymycobot.mycobot import MyCobot

# 전역 변수
stop_flag = False
cmd_vel_pub = None
mc = None

# 키보드 입력 상태 저장용
key_state = {
    'w': False, 's': False, 'a': False, 'd': False,
    'q': False, 'e': False, 'z': False, 'c': False,
    'x': False
}

class ControlApp(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("AGV + ARM 키보드 컨트롤")

        self.setGeometry(100, 100, 400, 300)
        self.layout = QVBoxLayout()

        # 개선된 안내문과 가운데 정렬
        self.info_label = QLabel("""<b>AGV 제어 키:</b><br>
        Q: 좌회전 전진  W: 전진  E: 우회전 전진<br>
        A: 좌측 이동   S: 후진  D: 우측 이동<br><br>
        <b>G 키:</b> ARM 랜덤 동작 1회 실행<br>
        <b>ESC 키:</b> 종료""")
        self.info_label.setAlignment(Qt.AlignCenter)  # 텍스트 가운데 정렬
        self.info_label.setStyleSheet("font-size: 14px; padding: 10px;")
        self.layout.addWidget(self.info_label)
        self.setLayout(self.layout)

        # 키 입력 감지 활성화
        self.setFocusPolicy(Qt.StrongFocus)

        # 주기적으로 키 상태에 따라 AGV 제어
        self.key_loop_thread = threading.Thread(target=self.key_control_loop)
        self.key_loop_thread.daemon = True
        self.key_loop_thread.start()

    def keyPressEvent(self, event):
        key = event.text().lower()
        if key in key_state:
            key_state[key] = True
        elif key == 'g':
            self.move_arm_once()
        elif event.key() == Qt.Key_Escape:
            global stop_flag
            stop_flag = True
            print("[System] ESC: 종료 명령")
            self.close()

    def keyReleaseEvent(self, event):
        key = event.text().lower()
        if key in key_state:
            key_state[key] = False

    def key_control_loop(self):
        global stop_flag, cmd_vel_pub
        rate = rospy.Rate(10)
        print("[AGV] 키보드 제어 시작")

        while not rospy.is_shutdown() and not stop_flag:
            twist = Twist()
            if key_state['w']: twist.linear.x += 0.1
            if key_state['s']: twist.linear.x -= 0.1
            if key_state['a']: twist.linear.y += 0.1
            if key_state['d']: twist.linear.y -= 0.1
            if key_state['q']: twist.angular.z += 0.5
            if key_state['e']: twist.angular.z -= 0.5
            if key_state['z']: twist.linear.x += 0.1; twist.angular.z += 0.5
            if key_state['c']: twist.linear.x += 0.1; twist.angular.z -= 0.5
            if key_state['x']: twist = Twist()  # 정지

            cmd_vel_pub.publish(twist)
            rate.sleep()

        cmd_vel_pub.publish(Twist())
        print("[AGV] 키보드 제어 종료")

    def move_arm_once(self):
        global mc
        angles = [random.randint(-45, 45) for _ in range(6)]
        print(f"[ARM] 랜덤 각도 실행: {angles}")
        mc.send_angles(angles, 80)

if __name__ == "__main__":
    rospy.init_node('agv_arm_keyboard_gui', anonymous=False)
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
