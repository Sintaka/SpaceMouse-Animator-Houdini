"""
SpaceMouse UDP 广播服务
读取 SpaceExplorer 原始数据，实时广播（无限速模式）
"""
import hid
import struct
import socket
import json
import time
import threading

# ===== 配置 =====
VENDOR_3DCONNEXION = 0x046D
PRODUCT_SPACEEXPLORER = 0xC627
UDP_HOST = "127.0.0.1"
UDP_PORT = 9876

# ===== SpaceMouse 数据处理 =====
class SpaceMouseReader:
    def __init__(self, broadcaster):
        self.dev = None
        self.broadcaster = broadcaster
        self.translation = [0, 0, 0]
        self.rotation = [0, 0, 0]
        self.button_state = 0
        self.running = False
        self.frame_count = 0
        self.lock = threading.Lock()
        
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
        self.dev.set_nonblocking(0)  # 阻塞模式，等待数据
        print(f"✓ 已连接: {self.dev.get_product_string()}")
    
    def start_reading(self):
        """启动读取+发送线程"""
        self.running = True
        self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
        self.read_thread.start()
    
    def _read_loop(self):
        """后台线程：读取数据后立刻发送"""
        while self.running:
            try:
                # 阻塞读取，有数据就处理
                data = self.dev.read(64, timeout_ms=1)
                if not data:
                    continue
                
                report_id = data[0]
                should_send = False
                
                with self.lock:
                    # Report 1: 平移
                    if report_id == 1 and len(data) >= 7:
                        self.translation[0] = struct.unpack('<h', bytes(data[1:3]))[0]
                        self.translation[1] = struct.unpack('<h', bytes(data[3:5]))[0]
                        self.translation[2] = struct.unpack('<h', bytes(data[5:7]))[0]
                        should_send = True
                        
                    # Report 2: 旋转（先试试这个）
                    elif report_id == 2 and len(data) >= 7:
                        self.rotation[0] = struct.unpack('<h', bytes(data[1:3]))[0]
                        self.rotation[1] = struct.unpack('<h', bytes(data[3:5]))[0]
                        self.rotation[2] = struct.unpack('<h', bytes(data[5:7]))[0]
                        should_send = True
                    
                    # 立刻发送并打印
                    if should_send:
                        packet = {
                            'timestamp': time.time(),
                            'frame': self.frame_count,
                            'translation': self.translation[:],
                            'rotation': self.rotation[:],
                            'buttons': self.button_state
                        }
                        self.broadcaster.send(packet)
                        
                        # 每次发送都打印
                        tx, ty, tz = self.translation
                        rx, ry, rz = self.rotation
                        print(f"[{self.frame_count:6d}] T:({tx:+5d},{ty:+5d},{tz:+5d}) "
                              f"R:({rx:+5d},{ry:+5d},{rz:+5d}) BTN:{self.button_state:08X}")
                        
                        self.frame_count += 1
            
            except Exception as e:
                if self.running:
                    print(f"读取错误: {e}")
                    time.sleep(0.1)
    
    def close(self):
        self.running = False
        if hasattr(self, 'read_thread'):
            self.read_thread.join(timeout=1.0)
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
        try:
            msg = json.dumps(data_dict).encode('utf-8')
            self.sock.sendto(msg, self.addr)
        except Exception as e:
            print(f"发送错误: {e}")
    
    def close(self):
        self.sock.close()


# ===== 主循环 =====
def main():
    print("=" * 60)
    print("  SpaceMouse UDP 广播服务（暴力发送模式）")
    print("=" * 60)
    
    broadcaster = UDPBroadcaster(UDP_HOST, UDP_PORT)
    reader = SpaceMouseReader(broadcaster)
    reader.connect()
    reader.start_reading()
    
    print(f"\n正在广播 (每个数据包立刻发送，Ctrl+C 退出)...\n")
    
    try:
        # 主线程只等待退出
        while True:
            time.sleep(1)
    
    except KeyboardInterrupt:
        print("\n\n正在停止...")
    finally:
        reader.close()
        broadcaster.close()
        print("已关闭")


if __name__ == "__main__":
    main()
