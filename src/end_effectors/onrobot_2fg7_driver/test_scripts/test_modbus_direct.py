#!/usr/bin/env python3
"""
Direct Modbus RTU test for OnRobot 2FG7 gripper.
Tests grip/release without ROS2 — just raw Modbus over socat.

Prerequisites:
  - UR teach pendant: Tool I/O = User, RS485 = 1Mbps/Even parity, Voltage = 24V
  - 2FG7 connected via Quick Changer to tool connector

Usage:
  python3 test_modbus_direct.py [--robot-ip 192.168.1.101]
"""

import subprocess
import time
import os
from pathlib import Path
import argparse

import pymodbus
PYMODBUS_V3 = int(pymodbus.__version__.split('.')[0]) >= 3
if PYMODBUS_V3:
    from pymodbus.client import ModbusSerialClient
else:
    from pymodbus.client.sync import ModbusSerialClient

SOCAT_PTY = "/tmp/ttyUR"
SLAVE_ID = 0x41
BAUD_RATE = 1000000


def start_socat(robot_ip):
    os.system('pkill -f "socat.*ttyUR" 2>/dev/null')
    time.sleep(1)
    try:
        Path(SOCAT_PTY).unlink()
    except OSError:
        pass

    proc = subprocess.Popen(
        ['socat', f'pty,link={SOCAT_PTY},raw,ignoreeof', f'tcp:{robot_ip}:54321'],
        stdout=subprocess.DEVNULL, stderr=subprocess.PIPE
    )
    time.sleep(3)

    if not Path(SOCAT_PTY).exists():
        print('[FAIL] socat failed to create PTY')
        proc.terminate()
        return None
    print(f'[OK] socat bridge: {robot_ip}:54321 -> {SOCAT_PTY}')
    return proc


def main():
    parser = argparse.ArgumentParser(description='Test OnRobot 2FG7 via direct Modbus RTU')
    parser.add_argument('--robot-ip', default='192.168.1.101', help='UR robot IP')
    args = parser.parse_args()

    socat = start_socat(args.robot_ip)
    if not socat:
        return

    client = ModbusSerialClient(
        method='rtu', port=SOCAT_PTY, baudrate=BAUD_RATE,
        parity='N', stopbits=1, bytesize=8, timeout=0.5,
    )
    client.connect()
    kw = {'slave': SLAVE_ID} if PYMODBUS_V3 else {'unit': SLAVE_ID}

    def read_status():
        r = client.read_holding_registers(0x0100, 8, **kw)
        return r.registers if not r.isError() else None

    # Read initial state
    time.sleep(0.5)
    s = read_status()
    if not s:
        print('[FAIL] No response from gripper. Check:')
        print('  1. Tool voltage = 24V')
        print('  2. RS485: Baud=1000000, Parity=Even')
        print('  3. Quick Changer properly seated')
        client.close()
        socat.terminate()
        return

    print(f'[OK] Gripper responding: ext_width={s[1]/10:.1f}mm, busy={s[0]}, status={s[7]}')

    # Close
    print('\n--- Closing to 25mm ---')
    client.write_registers(0x0000, [250, 20, 50, 1], **kw)
    for i in range(15):
        time.sleep(0.4)
        s = read_status()
        if s:
            print(f'  [{i}] ext={s[1]/10:.1f}mm busy={s[0]} status={s[7]}')
            if s[0] == 0 and i > 0:
                break

    time.sleep(1)

    # Open
    print('\n--- Opening to 60mm ---')
    client.write_registers(0x0000, [600, 20, 50, 1], **kw)
    for i in range(15):
        time.sleep(0.4)
        s = read_status()
        if s:
            print(f'  [{i}] ext={s[1]/10:.1f}mm busy={s[0]} status={s[7]}')
            if s[0] == 0 and i > 0:
                break

    client.close()
    socat.terminate()
    socat.wait()
    print('\n[OK] Test complete.')


if __name__ == '__main__':
    main()
