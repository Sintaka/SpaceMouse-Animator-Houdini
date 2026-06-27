"""
SpaceMouse UDP 广播服务
读取 SpaceExplorer 原始数据，计算相对位移，通过 UDP 发送
"""
import hid
import struct
import socket
import json
import time
from collections import deque

# ===== 配置 =====
VENDOR_3DCONNEXION = 0x046D
PRODUCT_SPACEEXPLORER = 0xC627
UDP_HOST = "127.0.0.1"
UDP_PORT = 9876
SEND_RATE = 60  # Hz

# ===== SpaceMouse 数据处理 =====
class SpaceMouseReader:
    def __init__(self):
        self.dev = None
        self.last_translation = [0, 0, 0]
        self.last_rotation = [0, 0, 0]
        self.button_state = 0
        
    def connect(self):
        """连接 SpaceExplorer"""
        devices = [d for d in hid.enumerate() 
                   if d['vendor_id'] == VENDOR_3DCONNEXION 
                   and d['product_id'] == PRODUCT_SPACEEXPLORER
                   and d.get('usage_page') == 1 
                   and d.get('usage') == 8]
        
        if not devices:
            raise RuntimeError("未找到 SpaceExplorer")
        
        self.dev = hid.device()
        self.dev.open_path(devices[0]['path'])
        print(f"✓ 已连接: {self.dev.get_product_string()}")
    
    def read_motion(self):
        """读取一帧运动数据，返回 (tx, ty, tz, rx, ry, rz, buttons)"""
        data = self.dev.read(64, timeout_ms=5)
        if not data:
            return None
        
        report_id = data[0]
        
        # Report 2: 平移
        if report_id == 2 and len(data) >= 7:
            tx = struct.unpack('<h', bytes(data[1:3]))[0]
            ty = struct.unpack('<h', bytes(data[3:5]))[0]
            tz = struct.unpack('<h', bytes(data[5:7]))[0]
            self.last_translation = [tx, ty, tz]
        
        # Report 3: 旋转
        elif report_id == 3 and len(data) >= 7:
            rx = struct.unpack('<h', bytes(data[1:3]))[0]
            ry = struct.unpack('<h', bytes(data[3:5]))[0]
            rz = struct.unpack('<h', bytes(data[5:7]))[0]
            self.last_rotation = [rx, ry, rz]
        
        # Report 1: 按钮
        elif report_id == 1 and len(data) >= 5:
            self.button_state = struct.unpack('<I', bytes(data[1:5]))[0]
        
        return {
            'translation': self.last_translation,
            'rotation': self.last_rotation,
            'buttons': self.button_state
        }
    
    def close(self):
        if self.dev:
            self.dev.close()


# ===== UDP 广播 =====
class UDPBroadcaster:
    def __init__(self, host, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.addr = (host, port)
        print(f"✓ UDP 广播地址: {host}:{port}")
    
    def send(self, data_dict):
        """发送 JSON 数据包"""
        msg = json.dumps(data_dict).encode('utf-8')
        self.sock.sendto(msg, self.addr)
    
    def close(self):
        self.sock.close()


# ===== 主循环 =====
def main():
    print("=" * 60)
    print("  SpaceMouse UDP 广播服务")
    print("=" * 60)
    
    reader = SpaceMouseReader()
    reader.connect()
    
    broadcaster = UDPBroadcaster(UDP_HOST, UDP_PORT)
    
    frame_time = 1.0 / SEND_RATE
    frame_count = 0
    
    print(f"\n正在广播 (速率: {SEND_RATE} Hz, Ctrl+C 退出)...\n")
    
    try:
        while True:
            start = time.perf_counter()
            
            motion = reader.read_motion()
            if motion:
                # 构造数据包
                packet = {
                    'timestamp': time.time(),
                    'frame': frame_count,
                    'translation': motion['translation'],
                    'rotation': motion['rotation'],
                    'buttons': motion['buttons']
                }
                
                broadcaster.send(packet)
                frame_count += 1
                
                # 调试输出
                if frame_count % 30 == 0:
                    tx, ty, tz = motion['translation']
                    rx, ry, rz = motion['rotation']
                    print(f"[{frame_count:6d}] T:({tx:+4d},{ty:+4d},{tz:+4d}) "
                          f"R:({rx:+4d},{ry:+4d},{rz:+4d}) BTN:{motion['buttons']:08X}")
            
            # 限制帧率
            elapsed = time.perf_counter() - start
            if elapsed < frame_time:
                time.sleep(frame_time - elapsed)
    
    except KeyboardInterrupt:
        print("\n\n正在停止...")
    finally:
        reader.close()
        broadcaster.close()
        print("已关闭")


if __name__ == "__main__":
    main()
