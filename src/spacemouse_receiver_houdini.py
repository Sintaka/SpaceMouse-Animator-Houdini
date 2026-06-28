# spacemouse_receiver.pypanel
"""
SpaceMouse 接收端 - Houdini Python Panel
直接控制 /obj/cam1

设备: 3Dconnexion SpaceExplorer (VID:046D PID:C627)
轴映射: Tx=左右 Ty=前後 Tz=上下 | Rx=Pitch Ry=Roll Rz=Yaw
"""
import hou
import socket
import json
from PySide6 import QtCore, QtWidgets

UDP_PORT = 9876

class SpaceMouseReceiver(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        
        # UDP Socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", UDP_PORT))
        self.sock.setblocking(False)
        
        # 定时器 (60 Hz)
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_from_spacemouse)
        
        # 状态
        self.active = False
        self.sensitivity_trans = 0.05  # 增大平移灵敏度
        self.sensitivity_rot = 2.0     # 增大旋转灵敏度
        
        # UI
        self.init_ui()
    
    def init_ui(self):
        layout = QtWidgets.QVBoxLayout()
        
        # 状态显示
        self.status_label = QtWidgets.QLabel("等待 SpaceMouse 数据...")
        layout.addWidget(self.status_label)
        
        # 启动/停止按钮
        self.toggle_btn = QtWidgets.QPushButton("启动")
        self.toggle_btn.clicked.connect(self.toggle)
        layout.addWidget(self.toggle_btn)
        
        # 灵敏度
        sens_group = QtWidgets.QGroupBox("灵敏度")
        sens_layout = QtWidgets.QFormLayout()
        self.trans_spin = QtWidgets.QDoubleSpinBox()
        self.trans_spin.setRange(0.001, 1.0)
        self.trans_spin.setValue(0.05)
        self.trans_spin.setSingleStep(0.005)
        self.rot_spin = QtWidgets.QDoubleSpinBox()
        self.rot_spin.setRange(0.1, 10.0)
        self.rot_spin.setValue(2.0)
        self.rot_spin.setSingleStep(0.1)
        sens_layout.addRow("平移:", self.trans_spin)
        sens_layout.addRow("旋转:", self.rot_spin)
        sens_group.setLayout(sens_layout)
        layout.addWidget(sens_group)
        
        self.setLayout(layout)
    
    def toggle(self):
        if self.active:
            self.timer.stop()
            self.toggle_btn.setText("启动")
            self.active = False
        else:
            self.timer.start(4)  # ~60 FPS
            self.toggle_btn.setText("停止")
            self.active = True
    
    def update_from_spacemouse(self):
        """从 UDP 读取数据并更新 cam1"""
        try:
            data, addr = self.sock.recvfrom(1024)
            packet = json.loads(data.decode('utf-8'))
            
            tx, ty, tz = packet['translation']
            rx, ry, rz = packet['rotation']
            
            # 归一化并应用灵敏度
            tx_norm = tx / 350.0 * self.trans_spin.value()
            ty_norm = ty / 350.0 * self.trans_spin.value()
            tz_norm = tz / 350.0 * self.trans_spin.value()
            rx_norm = rx / 350.0 * self.rot_spin.value()
            ry_norm = ry / 350.0 * self.rot_spin.value()
            rz_norm = rz / 350.0 * self.rot_spin.value()
            
            # 更新状态显示 (带轴标签)
            self.status_label.setText(
                f"Tx:{tx_norm:+.3f} Ty:{ty_norm:+.3f} Tz:{tz_norm:+.3f} | "
                f"Rx:{rx_norm:+.3f} Ry:{ry_norm:+.3f} Rz:{rz_norm:+.3f}"
            )
            
            # 移动 cam1
            self.move_cam1(tx_norm, ty_norm, tz_norm, rx_norm, ry_norm, rz_norm)
        
        except BlockingIOError:
            pass  # 没有数据时不移动
        except Exception as e:
            self.status_label.setText(f"错误: {e}")
    
    def move_cam1(self, tx, ty, tz, rx, ry, rz):
        """移动 /obj/cam1
        SpaceExplorer 轴映射 (实测):
          Tx(左右)  → Houdini X
          Ty(前後)  → Houdini Z  注意: 不是上下!
          Tz(上下)  → Houdini Y  注意: 不是前後!
          Rx(Pitch) → Houdini RX
          Ry(Roll)  → Houdini RZ 注意: 交换!
          Rz(Yaw)   → Houdini RY 注意: 交换!
        """
        try:
            cam = hou.node('/obj/cam1')
            if not cam:
                return

            # 获取当前变换
            t_curr = cam.parmTuple('t').eval()
            r_curr = cam.parmTuple('r').eval()

            # 平移映射:
            #   Tx → Houdini X (左右), 同向
            #   Tz → Houdini Y (上下), SpaceMouse +下压 = 向下 = Houdini -Y
            #   Ty → Houdini Z (前後), SpaceMouse +後推 = 拉近 = Houdini -Z
            t_new = (
                t_curr[0] + tx,       # Tx → X
                t_curr[1] - tz,       # Tz → Y (翻转: +下压 → -Y)
                t_curr[2] - ty,       # Ty → Z (翻转: +後推 → -Z)
            )

            # 旋转映射:
            #   Rx → Houdini RX (Pitch)
            #   Rz → Houdini RY (Yaw)  — 交换!
            #   Ry → Houdini RZ (Roll) — 交换!
            r_new = (
                r_curr[0] - rx,       # Rx → RX (Pitch)
                r_curr[1] + rz,       # Rz → RY (Yaw)
                r_curr[2] + ry,       # Ry → RZ (Roll)
            )

            cam.parmTuple('t').set(t_new)
            cam.parmTuple('r').set(r_new)

        except Exception as e:
            pass


def createInterface():
    return SpaceMouseReceiver()
