#!/usr/bin/env python3
"""
Low-level Modbus RTU client for the OnRobot 2FG7 gripper.

Register map (reverse-engineered from OnRobot ToolDaemon binary):
  Write 0x0000 (4 regs): [width_x10, force_N, speed_pct, mode]
    mode: 1=external grip, 2=internal grip
  Read 0x0100 (8 regs): [busy, ext_width_x10, int_width_x10, r3, r4, r5, r6, status]
  Read 0x0400 (6 regs): Static config data

Width values are in 1/10 mm (e.g., 300 = 30.0mm).
"""

from dataclasses import dataclass
from pymodbus.client import ModbusSerialClient

# Modbus register addresses
REG_COMMAND = 0x0000
REG_STATUS = 0x0100
REG_CONFIG = 0x0400

# Command modes
MODE_EXTERNAL = 1
MODE_INTERNAL = 2

# Default communication parameters
DEFAULT_SLAVE_ID = 0x41
DEFAULT_PORT = "/tmp/ttyUR"
DEFAULT_BAUD = 1000000


@dataclass
class GripperStatus:
    """Parsed gripper status from Modbus registers."""
    busy: bool
    external_width_mm: float
    internal_width_mm: float
    status: int
    raw_registers: list


class OnRobot2FG7Client:
    """Modbus RTU client for OnRobot 2FG7 gripper."""

    def __init__(self, port=DEFAULT_PORT, slave_id=DEFAULT_SLAVE_ID,
                 baudrate=DEFAULT_BAUD, timeout=0.5, logger=None):
        self.slave_id = slave_id
        self.logger = logger
        self._kw = {'slave': slave_id}

        self.client = ModbusSerialClient(
            port=port,
            baudrate=baudrate,
            parity='N',  # Our side is always N (virtual PTY); UR hardware handles Even
            stopbits=1,
            bytesize=8,
            timeout=timeout,
        )

    def connect(self) -> bool:
        return self.client.connect()

    def disconnect(self):
        self.client.close()

    def _write_command(self, width_mm: float, force_n: int, speed_pct: int, mode: int) -> bool:
        """Write grip command to registers 0x0000."""
        width_x10 = max(0, int(round(width_mm * 10)))
        force_n = max(0, min(140, int(force_n)))
        speed_pct = max(1, min(100, int(speed_pct)))
        mode = max(1, min(2, int(mode)))

        result = self.client.write_registers(
            REG_COMMAND, [width_x10, force_n, speed_pct, mode], **self._kw
        )
        if result.isError():
            if self.logger:
                self.logger.error(f"Modbus write error: {result}")
            return False
        return True

    def read_status(self) -> 'GripperStatus | None':
        """Read status registers 0x0100 (8 regs)."""
        result = self.client.read_holding_registers(REG_STATUS, 8, **self._kw)
        if result.isError():
            if self.logger:
                self.logger.error(f"Modbus read error: {result}")
            return None

        regs = result.registers
        return GripperStatus(
            busy=bool(regs[0]),
            external_width_mm=regs[1] / 10.0,
            internal_width_mm=regs[2] / 10.0,
            status=regs[7],
            raw_registers=regs,
        )

    def grip_external(self, width_mm: float, force_n: int = 40, speed_pct: int = 100) -> bool:
        """Close fingers externally (grip an object between the outer surfaces)."""
        return self._write_command(width_mm, force_n, speed_pct, MODE_EXTERNAL)

    def grip_internal(self, width_mm: float, force_n: int = 40, speed_pct: int = 100) -> bool:
        """Close fingers internally (grip an object between the inner surfaces)."""
        return self._write_command(width_mm, force_n, speed_pct, MODE_INTERNAL)

    def release(self, width_mm: float, speed_pct: int = 100) -> bool:
        """Open fingers to the specified width."""
        return self._write_command(width_mm, 20, speed_pct, MODE_EXTERNAL)

    def get_width(self) -> 'float | None':
        """Get current external width in mm."""
        status = self.read_status()
        return status.external_width_mm if status else None

    def is_busy(self) -> bool:
        """Check if gripper is currently moving."""
        status = self.read_status()
        return status.busy if status else False
