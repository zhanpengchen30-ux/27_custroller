"""
VOFA 串口采集工具：自动读串口、存 CSV。

用法：
python collect.py COM39 115200 scan1.csv

按 Ctrl+C 停止，自动保存。
"""
import sys, time, csv
import serial

def main():
    if len(sys.argv) < 4:
        print("用法: python collect.py 端口 波特率 输出文件")
        print("示例: python collect.py COM39 115200 scan1.csv")
        return

    port = sys.argv[1]
    baud = int(sys.argv[2])
    outfile = sys.argv[3]

    ser = serial.Serial(port, baud, timeout=1)
    print(f"已连接 {port} @ {baud}，开始采集...")
    print(f"机械臂动作采完后按 Ctrl+C 停止")

    rows = []
    try:
        while True:
            line = ser.readline().decode('ascii', errors='ignore').strip()
            if not line:
                continue
            parts = line.split(',')
            if len(parts) == 6:
                try:
                    vals = [float(x) for x in parts]
                    rows.append(vals)
                    if len(rows) % 100 == 0:
                        print(f"已采集 {len(rows)} 行，q1={vals[0]:.3f}")
                except ValueError:
                    pass
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

    with open(outfile, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(rows)
    print(f"\n保存 {len(rows)} 行到 {outfile}")

if __name__ == '__main__':
    main()
