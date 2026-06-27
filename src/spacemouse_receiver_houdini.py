# spacemouse_receiver.pypanel
"""
SpaceMouse 接收端 - Houdini Python Panel
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
        self.mode = "viewport"  # viewport / object / point
        self.sensitivity_trans = 0.01
        self.sensitivity_rot = 0.5
        
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
        
        # 模式选择
        mode_group = QtWidgets.QGroupBox("控制模式")
        mode_layout = QtWidgets.QVBoxLayout()
        self.mode_viewport = QtWidgets.QRadioButton("视口相机")
        self.mode_object = QtWidgets.QRadioButton("选中物体")
        self.mode_viewport.setChecked(True)
        mode_layout.addWidget(self.mode_viewport)
        mode_layout.addWidget(self.mode_object)
        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)
        
        # 灵敏度
        sens_group = QtWidgets.QGroupBox("灵敏度")
        sens_layout = QtWidgets.QFormLayout()
        self.trans_spin = QtWidgets.QDoubleSpinBox()
        self.trans_spin.setRange(0.001, 1.0)
        self.trans_spin.setValue(0.01)
        self.trans_spin.setSingleStep(0.001)
        self.rot_spin = QtWidgets.QDoubleSpinBox()
        self.rot_spin.setRange(0.1, 10.0)
        self.rot_spin.setValue(0.5)
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
            self.timer.start(16)  # ~60 FPS
            self.toggle_btn.setText("停止")
            self.active = True
    
    def update_from_spacemouse(self):
        """从 UDP 读取数据并更新 Houdini"""
        try:
            data, addr = self.sock.recvfrom(1024)
            packet = json.loads(data.decode('utf-8'))
            
            tx, ty, tz = packet['translation']
            rx, ry, rz = packet['rotation']
            
            # 归一化
            tx_norm = tx / 350.0 * self.trans_spin.value()
            ty_norm = ty / 350.0 * self.trans_spin.value()
            tz_norm = tz / 350.0 * self.trans_spin.value()
            rx_norm = rx / 350.0 * self.rot_spin.value()
            ry_norm = ry / 350.0 * self.rot_spin.value()
            rz_norm = rz / 350.0 * self.rot_spin.value()
            
            # 更新状态显示
            self.status_label.setText(
                f"T:({tx_norm:+.3f}, {ty_norm:+.3f}, {tz_norm:+.3f}) "
                f"R:({rx_norm:+.3f}, {ry_norm:+.3f}, {rz_norm:+.3f})"
            )
            
            # 根据模式执行动作
            if self.mode_viewport.isChecked():
                self.move_viewport(tx_norm, ty_norm, tz_norm, rx_norm, ry_norm, rz_norm)
            elif self.mode_object.isChecked():
                self.move_selected_object(tx_norm, ty_norm, tz_norm, rx_norm, ry_norm, rz_norm)
        
        except BlockingIOError:
            pass  # 没有数据
        except Exception as e:
            self.status_label.setText(f"错误: {e}")
    
    def move_viewport(self, tx, ty, tz, rx, ry, rz):
        """移动 /obj/cam1 相机"""
        try:
            cam = hou.node('/obj/cam1')
            if cam is None:
                self.status_label.setText("错误: /obj/cam1 不存在")
                return
            
            # 检查相机是否有变换参数
            if not cam.parmTuple('t') or not cam.parmTuple('r'):
                self.status_label.setText("错误: cam1 没有变换参数")
                return
            
            # 获取当前值
            t_curr = cam.parmTuple('t').eval()
            r_curr = cam.parmTuple('r').eval()
            
            # 应用相对位移
            t_new = (t_curr[0] + tx, t_curr[1] + ty, t_curr[2] + tz)
            r_new = (r_curr[0] + rx, r_curr[1] + ry, r_curr[2] + rz)
            
            cam.parmTuple('t').set(t_new)
            cam.parmTuple('r').set(r_new)
            
        except Exception as e:
            self.status_label.setText(f"错误: {e}")

    
    def move_selected_object(self, tx, ty, tz, rx, ry, rz):
        """移动选中的物体"""
        selected = hou.selectedNodes()
        if not selected:
            return
        
        obj = selected[0]
        if not obj.parmTuple('t') or not obj.parmTuple('r'):
            return
        
        # 获取当前值
        t_curr = obj.parmTuple('t').eval()
        r_curr = obj.parmTuple('r').eval()
        
        # 应用相对位移
        t_new = (t_curr[0] + tx, t_curr[1] + ty, t_curr[2] + tz)
        r_new = (r_curr[0] + rx, r_curr[1] + ry, r_curr[2] + rz)
        
        obj.parmTuple('t').set(t_new)
        obj.parmTuple('r').set(r_new)


def createInterface():
    return SpaceMouseReceiver()
