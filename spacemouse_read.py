# spacemouse_test.py - 最简单的 SpaceExplorer 数据读取
import hid
import struct
import time

VENDOR_3DCONNEXION = 0x046D
PRODUCT_SPACEEXPLORER = 0xC627

print("正在搜索 SpaceExplorer (VID=0x046D, PID=0xC627)...")

# 列出所有 3Dconnexion 设备
devices = [d for d in hid.enumerate() if d['vendor_id'] == VENDOR_3DCONNEXION]
if not devices:
    print("错误: 未找到任何 3Dconnexion 设备")
    print("\n请检查:")
    print("  1. 设备是否已插入")
    print("  2. 驱动是否已安装")
    print("  3. 设备管理器中是否显示为 '3Dconnexion SpaceExplorer'")
    exit(1)

print(f"\n找到 {len(devices)} 个 3Dconnexion 设备:")
for d in devices:
    print(f"  - PID=0x{d['product_id']:04X}, "
          f"Usage Page={d.get('usage_page', '?')}, "
          f"Usage={d.get('usage', '?')}")
    print(f"    路径: {d['path']}")

# 找到 SpaceExplorer (PID=0xC627, Usage Page=1, Usage=8)
target = None
for d in devices:
    if d['product_id'] == PRODUCT_SPACEEXPLORER:
        # 优先选择 Multi-axis Controller (usage_page=1, usage=8)
        if d.get('usage_page') == 1 and d.get('usage') == 8:
            target = d
            break
        # 备选：任何 0xC627
        if target is None:
            target = d

if target is None:
    print("\n错误: 未找到 SpaceExplorer (PID=0xC627)")
    exit(1)

print(f"\n正在打开: PID=0x{target['product_id']:04X}")

dev = hid.device()
try:
    dev.open_path(target['path'])
except IOError as e:
    print(f"\n打开失败: {e}")
    print("\n可能原因:")
    print("  1. 3DxWare 服务正在独占设备")
    print("     解决: 以管理员身份运行 'net stop 3DxService'")
    print("  2. 权限不足")
    print("     解决: 以管理员身份运行此脚本")
    exit(1)

print("✓ 已连接!")
print(f"制造商: {dev.get_manufacturer_string()}")
print(f"产品名: {dev.get_product_string()}")
print("\n移动 SpaceExplorer 查看数据 (Ctrl+C 退出)\n")
print(f"{'报告ID':>6} {'原始数据 (hex)':>50}")
print("-" * 60)

try:
    count = 0
    while True:
        data = dev.read(64, timeout_ms=100)
        if data:
            count += 1
            hex_str = ' '.join(f'{b:02X}' for b in data[:13])
            print(f"{count:>6}  {hex_str}")
            
            # 解析 Report ID 2 (平移)
            if data[0] == 2 and len(data) >= 7:
                tx = struct.unpack('<h', bytes(data[1:3]))[0]
                ty = struct.unpack('<h', bytes(data[3:5]))[0]
                tz = struct.unpack('<h', bytes(data[5:7]))[0]
                print(f"       -> 平移: X={tx:+5d}  Y={ty:+5d}  Z={tz:+5d}")
            
            # 解析 Report ID 3 (旋转)
            elif data[0] == 3 and len(data) >= 7:
                rx = struct.unpack('<h', bytes(data[1:3]))[0]
                ry = struct.unpack('<h', bytes(data[3:5]))[0]
                rz = struct.unpack('<h', bytes(data[5:7]))[0]
                print(f"       -> 旋转: RX={rx:+5d}  RY={ry:+5d}  RZ={rz:+5d}")
            
            # 解析 Report ID 1 (按钮)
            elif data[0] == 1 and len(data) >= 5:
                btn = struct.unpack('<I', bytes(data[1:5]))[0]
                if btn:
                    print(f"       -> 按钮: 0x{btn:08X}")

except KeyboardInterrupt:
    print("\n\n已停止")
finally:
    dev.close()
