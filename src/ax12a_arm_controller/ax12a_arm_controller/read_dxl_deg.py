#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
功能：读取某个 Dynamixel（AX-12A, 协议1.0）的当前角度（度），用于标定夹爪端点。
使用方法：
  1) 先停止你的驱动节点，避免抢占串口
  2) 手动把夹爪拨到“闭合”或“张开”
  3) 运行本脚本，记录输出的 Present Position = XX.xx deg
"""

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS
import sys

# ===== 根据你的实际情况修改下面三项 =====
PORT_NAME   = '/dev/ttyUSB0'   # 串口
BAUDRATE    = 1000000          # 波特率
DXL_ID      = 5                # 夹爪的舵机ID
# =====================================

ADDR_PRESENT_POSITION = 36     # 2 bytes
RESOLUTION = 1023
DEGREE_RANGE = 300.0

def pos_to_deg(pos: int) -> float:
    """将舵机刻度(0..1023)转换为角度(度)。"""
    p = max(0, min(RESOLUTION, int(pos)))
    return p / RESOLUTION * DEGREE_RANGE

def main():
    """主函数：打开串口，读取位置，打印角度。"""
    port = PortHandler(PORT_NAME)
    pkt  = PacketHandler(1.0)
    if not port.openPort():
        print(f'ERROR: 打不开串口 {PORT_NAME}')
        sys.exit(1)
    if not port.setBaudRate(BAUDRATE):
        print(f'ERROR: 设置波特率失败 {BAUDRATE}')
        sys.exit(1)

    pos, rc, err = pkt.read2ByteTxRx(port, DXL_ID, ADDR_PRESENT_POSITION)
    if rc != COMM_SUCCESS or err != 0:
        print(f'ERROR: 读取失败 rc={rc} err={err}')
        sys.exit(1)

    deg = pos_to_deg(int(pos))
    print(f'Present Position = {deg:.2f} deg')
    port.closePort()

if __name__ == '__main__':
    main()
