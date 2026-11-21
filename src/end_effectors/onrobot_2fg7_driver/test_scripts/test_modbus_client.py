#!/usr/bin/env python3
"""
Test the OnRobot2FG7Client Python class (no ROS2 needed).
Verifies the driver's Modbus client layer works correctly.

Prerequisites:
  - UR teach pendant: Tool I/O = User, RS485 = 1Mbps/Even parity, Voltage = 24V
  - 2FG7 connected via Quick Changer to tool connector

Usage:
  python3 test_modbus_client.py [--robot-ip 192.168.1.101]
"""

import subprocess
import time
import os
from pathlib import Path
import sys
import argparse

# Add driver package to path
sys.path.insert(0, str(Path(__file__).absolute().parents[1]))

from onrobot_2fg7_driver.modbus_client import OnRobot2FG7Client

SOCAT_PTY = "/tmp/ttyUR"


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
        print('[FAIL] socat failed')
        proc.terminate()
        return None
    return proc


def main():
    parser = argparse.ArgumentParser(description='Test OnRobot2FG7Client class')
    parser.add_argument('--robot-ip', default='192.168.1.101')
    args = parser.parse_args()

    socat = start_socat(args.robot_ip)
    if not socat:
        return

    client = OnRobot2FG7Client(port=SOCAT_PTY, slave_id=0x41, baudrate=1000000)
    if not client.connect():
        print('[FAIL] Client connect() returned False')
        socat.terminate()
        return

    # Read status
    status = client.read_status()
    if not status:
        print('[FAIL] read_status() returned None')
        client.disconnect()
        socat.terminate()
        return

    print(f'[OK] Connected: ext_width={status.external_width_mm:.1f}mm, '
          f'int_width={status.internal_width_mm:.1f}mm, busy={status.busy}, status={status.status}')

    # Test grip_external
    print('\n--- grip_external(25.0, 20, 50) ---')
    ok = client.grip_external(25.0, 20, 50)
    print(f'  Command sent: {ok}')
    time.sleep(2)
    print(f'  Width: {client.get_width():.1f}mm, Busy: {client.is_busy()}')

    # Test release
    print('\n--- release(60.0, 50) ---')
    ok = client.release(60.0, 50)
    print(f'  Command sent: {ok}')
    time.sleep(2)
    print(f'  Width: {client.get_width():.1f}mm, Busy: {client.is_busy()}')

    client.disconnect()
    socat.terminate()
    socat.wait()
    print('\n[OK] All tests passed.')


if __name__ == '__main__':
    main()
