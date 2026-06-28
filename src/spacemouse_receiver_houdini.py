# spacemouse_receiver.pypanel
"""
SpaceMouse 接收端 — 第一人称相机控制 /obj/cam1
==============================================
使用 numpy 矩阵运算实现真正的局部空间移动+旋转

设备: 3Dconnexion SpaceExplorer (VID:046D PID:C627)
轴映射 (实测):
  Tx: 左(-)/右(+) → 沿相机 X 轴侧移      Rx: Pitch 前倾(-)/後仰(+) → 绕相机 X 轴
  Ty: 前(-)/後(+) → 沿相机 Z 轴前后      Ry: Roll  顺时针(-)/逆时针(+) → 绕相机 Z 轴
  Tz: 下(+)/上(-) → 沿世界 Y 轴升降      Rz: Yaw   顺时针(-)/逆时针(+) → 绕世界 Y 轴

Houdini 矩阵约定 (row-major):
  v_world = v_local * M
  Row 0 = 相机 right     Row 1 = 相机 up
  Row 2 = 相机 fwd (+Z)  Row 3 = 世界位置
  setAt(row, col) / at(row, col)
"""
import hou
import socket
import json
import numpy as np
from PySide6 import QtCore, QtWidgets

UDP_PORT = 9876
CAM_PATH = '/obj/cam1'


# ═══════════════════════════════════════════════════════════════
# 矩阵工具
# ═══════════════════════════════════════════════════════════════

def mat4_to_numpy(m):
    """hou.Matrix4 → numpy 4x4"""
    return np.array([
        [m.at(0, 0), m.at(0, 1), m.at(0, 2), m.at(0, 3)],
        [m.at(1, 0), m.at(1, 1), m.at(1, 2), m.at(1, 3)],
        [m.at(2, 0), m.at(2, 1), m.at(2, 2), m.at(2, 3)],
        [m.at(3, 0), m.at(3, 1), m.at(3, 2), m.at(3, 3)],
    ], dtype=np.float64)


def numpy_to_mat4(pos, rot):
    """pos(3,) + rot(3,3) → hou.Matrix4 (平移在第4行)"""
    m = hou.Matrix4()
    for i in range(3):
        for j in range(3):
            m.setAt(i, j, float(rot[i, j]))
        m.setAt(i, 3, 0.0)
    m.setAt(3, 0, float(pos[0]))
    m.setAt(3, 1, float(pos[1]))
    m.setAt(3, 2, float(pos[2]))
    m.setAt(3, 3, 1.0)
    return m


def rodrigues(axis, angle_rad):
    """绕任意轴旋转的 3x3 矩阵 (column-vector convention → 行向量右乘其转置)"""
    axis = axis / np.linalg.norm(axis)
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    t = 1.0 - c
    x, y, z = axis
    return np.array([
        [t*x*x + c,     t*x*y - s*z, t*x*z + s*y],
        [t*x*y + s*z,   t*y*y + c,   t*y*z - s*x],
        [t*x*z - s*y,   t*y*z + s*x, t*z*z + c],
    ], dtype=np.float64)


def orthonormalize(rot):
    """轻量正交化 — 保留 forward (row2), 重新推导 right 和 up"""
    fwd = rot[2, :].copy()
    fwd /= np.linalg.norm(fwd)
    right = rot[0, :].copy()
    right -= np.dot(right, fwd) * fwd
    right /= np.linalg.norm(right)
    up = np.cross(fwd, right)
    up /= np.linalg.norm(up)
    out = rot.copy()
    out[0, :] = right
    out[1, :] = up
    out[2, :] = fwd
    return out


# ═══════════════════════════════════════════════════════════════
# 主面板
# ═══════════════════════════════════════════════════════════════

class SpaceMouseReceiver(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()

        # UDP
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", UDP_PORT))
        self.sock.setblocking(False)

        # 定时器 ~250Hz
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_from_spacemouse)

        self.active = False
        self.frame_count = 0

        self.init_ui()

    # ── UI ────────────────────────────────────────────────

    def init_ui(self):
        layout = QtWidgets.QVBoxLayout()

        # 位姿信息显示
        self.status_label = QtWidgets.QLabel("等待 SpaceMouse 数据...")
        self.status_label.setWordWrap(True)
        self.status_label.setMinimumHeight(110)
        self.status_label.setStyleSheet(
            "QLabel { font-family: Consolas, monospace; font-size: 12px;"
            " background: #1e1e1e; color: #d4d4d4; padding: 8px;"
            " border: 1px solid #444; border-radius: 4px; }")
        layout.addWidget(self.status_label)

        # 按钮
        btn_row = QtWidgets.QHBoxLayout()

        self.toggle_btn = QtWidgets.QPushButton("▶  启动")
        self.toggle_btn.clicked.connect(self.toggle)
        self.toggle_btn.setMinimumHeight(30)
        btn_row.addWidget(self.toggle_btn)

        self.pose_btn = QtWidgets.QPushButton("📷 打印位姿")
        self.pose_btn.clicked.connect(self.print_pose)
        self.pose_btn.setMinimumHeight(30)
        btn_row.addWidget(self.pose_btn)

        self.reset_btn = QtWidgets.QPushButton("↺ 复位 cam1")
        self.reset_btn.clicked.connect(self.reset_cam1)
        self.reset_btn.setMinimumHeight(30)
        btn_row.addWidget(self.reset_btn)

        layout.addLayout(btn_row)

        # 主灵敏度
        sens = QtWidgets.QGroupBox("主灵敏度 (原始值 / 350 × 幅值)")
        sens_lay = QtWidgets.QFormLayout()

        self.t_spin = QtWidgets.QDoubleSpinBox()
        self.t_spin.setRange(0.0001, 10.0)
        self.t_spin.setValue(0.05)
        self.t_spin.setSingleStep(0.005)
        self.t_spin.setDecimals(5)
        sens_lay.addRow("平移 T:", self.t_spin)

        self.r_spin = QtWidgets.QDoubleSpinBox()
        self.r_spin.setRange(0.0001, 10.0)
        self.r_spin.setValue(1.0)
        self.r_spin.setSingleStep(0.1)
        self.r_spin.setDecimals(5)
        sens_lay.addRow("旋转 R (°):", self.r_spin)

        sens.setLayout(sens_lay)
        layout.addWidget(sens)

        # 逐轴增益 (-1.0 ~ 1.0, 默认 1.0)
        gain = QtWidgets.QGroupBox("逐轴增益 (±1.0, 负值=翻转方向)")
        gain_lay = QtWidgets.QGridLayout()

        labels_t = ["Tx (左右)", "Ty (前後)", "Tz (上下)"]
        labels_r = ["Rx (Pitch)", "Ry (Roll)", "Rz (Yaw)"]

        self.gain_t = []
        self.gain_r = []

        for col, (label, default) in enumerate(zip(labels_t, [1.0, 1.0, 1.0])):
            lb = QtWidgets.QLabel(label)
            lb.setStyleSheet("QLabel { font-size: 9pt; }")
            gain_lay.addWidget(lb, 0, col)
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(-1.0, 1.0)
            sp.setValue(default)
            sp.setSingleStep(0.1)
            sp.setDecimals(3)
            gain_lay.addWidget(sp, 1, col)
            self.gain_t.append(sp)

        for col, (label, default) in enumerate(zip(labels_r, [1.0, 1.0, -1.0])):
            lb = QtWidgets.QLabel(label)
            lb.setStyleSheet("QLabel { font-size: 9pt; }")
            gain_lay.addWidget(lb, 2, col)
            sp = QtWidgets.QDoubleSpinBox()
            sp.setRange(-1.0, 1.0)
            sp.setValue(default)
            sp.setSingleStep(0.1)
            sp.setDecimals(3)
            gain_lay.addWidget(sp, 3, col)
            self.gain_r.append(sp)

        gain.setLayout(gain_lay)
        layout.addWidget(gain)

        # 说明
        hint = QtWidgets.QLabel(
            "第一人称: 平移沿相机局部轴 | "
            "Yaw 绕世界 Y | Pitch 绕相机 X | Roll 绕相机 Z")
        hint.setWordWrap(True)
        hint.setStyleSheet("QLabel { color: #888; font-size: 9pt; padding: 4px; }")
        layout.addWidget(hint)

        self.setLayout(layout)

    # ── 启动/停止 ────────────────────────────────────────

    def toggle(self):
        if self.active:
            self.timer.stop()
            self.toggle_btn.setText("▶  启动")
            self.active = False
            self.status_label.setText("已停止")
        else:
            self.timer.start(4)  # 250 Hz
            self.toggle_btn.setText("⏸  停止")
            self.active = True
            self.status_label.setText("已启动，等待数据...")

    # ── 复位 ──────────────────────────────────────────────

    def reset_cam1(self):
        cam = hou.node(CAM_PATH)
        if cam is None:
            self.status_label.setText("错误: " + CAM_PATH + " 不存在")
            return
        cam.parmTuple('t').set((0.0, 1.5, 5.0))
        cam.parmTuple('r').set((0.0, 0.0, 0.0))
        self.status_label.setText("cam1 已复位 → pos(0, 1.5, 5)  rot(0,0,0)")

    # ── 打印位姿 ──────────────────────────────────────────

    def print_pose(self):
        cam = hou.node(CAM_PATH)
        if cam is None:
            self.status_label.setText("错误: " + CAM_PATH + " 不存在")
            return

        m = mat4_to_numpy(cam.worldTransform())
        pos = m[3, :3]
        rot = m[:3, :3]
        rx = cam.parm('rx').eval()
        ry = cam.parm('ry').eval()
        rz = cam.parm('rz').eval()
        view_dir = -rot[2, :]

        info = (
            f"=== /obj/cam1 位姿 ===\n"
            f"位置 : ({pos[0]:.4f}, {pos[1]:.4f}, {pos[2]:.4f})\n"
            f"旋转 : rx={rx:.4f}° ry={ry:.4f}° rz={rz:.4f}°\n"
            f"视野 : ({view_dir[0]:+.4f}, {view_dir[1]:+.4f}, {view_dir[2]:+.4f})\n"
            f"Right: ({rot[0,0]:+.4f}, {rot[0,1]:+.4f}, {rot[0,2]:+.4f})\n"
            f"Up   : ({rot[1,0]:+.4f}, {rot[1,1]:+.4f}, {rot[1,2]:+.4f})"
        )
        print(info)
        self.status_label.setText(info)

    # ── UDP 接收 + 移动 ──────────────────────────────────

    def update_from_spacemouse(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            pkt = json.loads(data.decode('utf-8'))

            tx, ty, tz = pkt['translation']
            rx, ry, rz = pkt['rotation']

            t_sens = self.t_spin.value()
            r_sens = self.r_spin.value()

            # 逐轴增益
            gt = np.array([g.value() for g in self.gain_t], dtype=np.float64)
            gr = np.array([g.value() for g in self.gain_r], dtype=np.float64)

            tv = np.array([tx, ty, tz], dtype=np.float64) / 350.0 * t_sens * gt
            rv = np.array([rx, ry, rz], dtype=np.float64) / 350.0 * r_sens * gr

            self.move_cam1(tv[0], tv[1], tv[2], rv[0], rv[1], rv[2])
            self._update_display(tx, ty, tz, rx, ry, rz, tv, rv)

        except BlockingIOError:
            pass
        except Exception as e:
            self.status_label.setText(f"错误: {e}")

    def _update_display(self, tx, ty, tz, rx, ry, rz, tv, rv):
        cam = hou.node(CAM_PATH)
        if cam is None:
            return
        p = cam.parmTuple('t').eval()
        r = cam.parmTuple('r').eval()
        self.status_label.setText(
            f"in  T:({tx:+4d},{ty:+4d},{tz:+4d})  R:({rx:+4d},{ry:+4d},{rz:+4d})\n"
            f"out T:({tv[0]:+.5f},{tv[1]:+.5f},{tv[2]:+.5f})  "
            f"R:({rv[0]:+.4f}°,{rv[1]:+.4f}°,{rv[2]:+.4f}°)\n"
            f"cam → pos({p[0]:.3f},{p[1]:.3f},{p[2]:.3f})  "
            f"rot({r[0]:.2f}°,{r[1]:.2f}°,{r[2]:.2f}°)")

    # ── 核心: 第一人称相机控制 ───────────────────────────

    def move_cam1(self, tx, ty, tz, rx, ry, rz):
        """第一人称相机控制 — 直接操作 /obj/cam1 世界变换矩阵

        SpaceExplorer → 相机动作:
          tx = 左(-)/右(+)    → 沿相机 X 轴侧移
          ty = 前(-)/後(+)    → 沿相机 Z 轴前后 (ty+ = 後退)
          tz = 下(+)/上(-)    → 沿世界 Y 轴升降 (tz+ = 下降)
          rx = 前倾(-)/後仰(+) → Pitch, 绕相机 X 轴
          ry = Roll 顺(-)/逆(+) → Roll,  绕相机 Z 轴
          rz = Yaw  顺(-)/逆(+) → Yaw,   绕世界 Y 轴
        """
        cam = hou.node(CAM_PATH)
        if cam is None:
            return

        # 1. 获取当前世界变换
        m = mat4_to_numpy(cam.worldTransform())
        pos = m[3, :3].copy()
        rot = m[:3, :3].copy()

        # 2. 相机局部轴 (行向量 = 局部轴在世界空间的方向)
        cam_right  = rot[0, :]     # 局部 X
        cam_fwd    = rot[2, :]     # 局部 Z
        world_up   = np.array([0.0, 1.0, 0.0])

        # 3. 平移 — 沿相机局部轴
        pos += tx * cam_right          # 左右
        pos += ty * cam_fwd            # 前後
        pos[1] -= tz                   # 上下 (世界 Y)

        # 4. 旋转 — 顺序: Yaw → Pitch → Roll
        #    对行向量 M:  M_new = M @ rodrigues(axis, angle).T

        if abs(rz) > 1e-10:            # Yaw: 绕世界 Y
            R = rodrigues(world_up, np.radians(rz))
            rot = rot @ R.T

        if abs(rx) > 1e-10:            # Pitch: 绕相机 X (right)
            R = rodrigues(rot[0, :], np.radians(rx))
            rot = rot @ R.T

        if abs(ry) > 1e-10:            # Roll: 绕相机 Z (forward)
            R = rodrigues(rot[2, :], np.radians(ry))
            rot = rot @ R.T

        # 5. 防浮点漂移
        rot = orthonormalize(rot)

        # 6. 写回世界变换
        cam.setWorldTransform(numpy_to_mat4(pos, rot))


def createInterface():
    return SpaceMouseReceiver()
