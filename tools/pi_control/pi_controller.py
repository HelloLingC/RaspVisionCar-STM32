#!/usr/bin/env python3
"""
RaspVisionCar STM32 control client (matches current firmware protocol).

Protocol:
- Frame: SOF1 SOF2 VER TYPE SEQ LEN PAYLOAD CRC16_LE
- SOF: 0xA5 0x5A
- VER: 0x01
- CRC: CRC16/CCITT-FALSE over bytes [VER..PAYLOAD]
"""

from __future__ import annotations

import argparse
import struct
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import Dict, List, Optional

import serial


SOF1 = 0xA5
SOF2 = 0x5A
VER = 0x01
MAX_PAYLOAD = 48
MIN_FRAME_LEN = 8


class MsgType(IntEnum):
    SET_TWIST = 0x01
    ARM = 0x02
    DISARM = 0x03
    ESTOP = 0x04
    CLEAR_ESTOP = 0x05
    HEARTBEAT = 0x06
    STATUS = 0x80


class ControlState(IntEnum):
    DISARMED = 0
    ARMED = 1
    ESTOP_LATCHED = 2


class ControlError(IntEnum):
    NONE = 0
    BAD_LEN = 1
    BAD_CRC = 2
    BAD_SEQ = 3
    TIMEOUT = 4
    STATE = 5
    BAD_PAYLOAD = 6


@dataclass
class StatusPacket:
    control_state: int
    last_error: int
    last_rx_seq: int
    target_left_rpm: int
    target_right_rpm: int
    meas_left_rpm: int
    meas_right_rpm: int
    cmd_v_mmps: int
    cmd_w_mradps: int
    odom_left: int
    odom_right: int
    age_set_twist_ms: int
    age_link_ms: int
    drop_crc: int
    drop_seq: int

    @staticmethod
    def from_payload(payload: bytes) -> "StatusPacket":
        if len(payload) != 36:
            raise ValueError(f"STATUS payload must be 36 bytes, got {len(payload)}")

        (
            control_state,
            last_error,
            last_rx_seq,
            _reserved,
            target_left_rpm,
            target_right_rpm,
            meas_left_rpm,
            meas_right_rpm,
            cmd_v_mmps,
            cmd_w_mradps,
            odom_left,
            odom_right,
            age_set_twist_ms,
            age_link_ms,
            drop_crc,
            drop_seq,
        ) = struct.unpack("<BBBBhhhhhhiiHHII", payload)

        return StatusPacket(
            control_state=control_state,
            last_error=last_error,
            last_rx_seq=last_rx_seq,
            target_left_rpm=target_left_rpm,
            target_right_rpm=target_right_rpm,
            meas_left_rpm=meas_left_rpm,
            meas_right_rpm=meas_right_rpm,
            cmd_v_mmps=cmd_v_mmps,
            cmd_w_mradps=cmd_w_mradps,
            odom_left=odom_left,
            odom_right=odom_right,
            age_set_twist_ms=age_set_twist_ms,
            age_link_ms=age_link_ms,
            drop_crc=drop_crc,
            drop_seq=drop_seq,
        )

    def state_name(self) -> str:
        try:
            return ControlState(self.control_state).name
        except ValueError:
            return f"UNKNOWN({self.control_state})"

    def error_name(self) -> str:
        try:
            return ControlError(self.last_error).name
        except ValueError:
            return f"UNKNOWN({self.last_error})"


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class RaspVisionCarClient:
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.02) -> None:
        self.ser = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        self._seq = 0
        self._rx = bytearray()
        self.stats: Dict[str, int] = {
            "bad_sof": 0,
            "bad_len": 0,
            "bad_crc": 0,
            "decode_err": 0,
            "status_frames": 0,
        }

    def close(self) -> None:
        if self.ser.is_open:
            self.ser.close()

    def __enter__(self) -> "RaspVisionCarClient":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()

    def _next_seq(self) -> int:
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFF
        return seq

    def _build_frame(self, msg_type: int, payload: bytes) -> bytes:
        if len(payload) > MAX_PAYLOAD:
            raise ValueError(f"payload too large: {len(payload)} > {MAX_PAYLOAD}")

        seq = self._next_seq()
        body = bytes([VER, msg_type & 0xFF, seq, len(payload)]) + payload
        crc = crc16_ccitt_false(body)
        return bytes([SOF1, SOF2]) + body + struct.pack("<H", crc)

    def send(self, msg_type: int, payload: bytes = b"") -> None:
        frame = self._build_frame(msg_type, payload)
        self.ser.write(frame)

    def arm(self) -> None:
        self.send(MsgType.ARM)

    def disarm(self) -> None:
        self.send(MsgType.DISARM)

    def estop(self) -> None:
        self.send(MsgType.ESTOP)

    def clear_estop(self) -> None:
        self.send(MsgType.CLEAR_ESTOP)

    def heartbeat(self) -> None:
        self.send(MsgType.HEARTBEAT)

    def set_twist(self, v_mmps: int, w_mradps: int, reverse_allowed: bool = True) -> None:
        flags = 0x01 if reverse_allowed else 0x00
        payload = struct.pack("<hhBB", int(v_mmps), int(w_mradps), flags, 0)
        self.send(MsgType.SET_TWIST, payload)

    def _decode_one_frame(self, frame: bytes) -> Optional[tuple[int, int, bytes]]:
        if len(frame) < MIN_FRAME_LEN:
            self.stats["decode_err"] += 1
            return None
        if frame[0] != SOF1 or frame[1] != SOF2:
            self.stats["bad_sof"] += 1
            return None
        if frame[2] != VER:
            self.stats["decode_err"] += 1
            return None

        msg_type = frame[3]
        seq = frame[4]
        length = frame[5]
        expected = MIN_FRAME_LEN + length
        if expected != len(frame):
            self.stats["bad_len"] += 1
            return None

        payload = frame[6 : 6 + length]
        crc_recv = struct.unpack("<H", frame[-2:])[0]
        crc_calc = crc16_ccitt_false(frame[2:-2])
        if crc_recv != crc_calc:
            self.stats["bad_crc"] += 1
            return None

        return msg_type, seq, payload

    def poll_status(self) -> List[StatusPacket]:
        out: List[StatusPacket] = []
        chunk = self.ser.read(self.ser.in_waiting or 1)
        if chunk:
            self._rx.extend(chunk)

        while True:
            if len(self._rx) < 2:
                break

            sof_idx = self._rx.find(bytes([SOF1, SOF2]))
            if sof_idx < 0:
                self.stats["bad_sof"] += len(self._rx)
                self._rx.clear()
                break
            if sof_idx > 0:
                self.stats["bad_sof"] += sof_idx
                del self._rx[:sof_idx]

            if len(self._rx) < 6:
                break

            length = self._rx[5]
            if length > MAX_PAYLOAD:
                self.stats["bad_len"] += 1
                del self._rx[0]
                continue

            frame_len = MIN_FRAME_LEN + length
            if len(self._rx) < frame_len:
                break

            frame = bytes(self._rx[:frame_len])
            del self._rx[:frame_len]

            decoded = self._decode_one_frame(frame)
            if decoded is None:
                continue

            msg_type, _seq, payload = decoded
            if msg_type == MsgType.STATUS:
                try:
                    status = StatusPacket.from_payload(payload)
                    out.append(status)
                    self.stats["status_frames"] += 1
                except Exception:
                    self.stats["decode_err"] += 1

        return out


def print_status(status: StatusPacket) -> None:
    print(
        f"state={status.state_name():<13} err={status.error_name():<10} "
        f"seq={status.last_rx_seq:3d} "
        f"cmd(v={status.cmd_v_mmps:5d}, w={status.cmd_w_mradps:5d}) "
        f"tar(l={status.target_left_rpm:4d}, r={status.target_right_rpm:4d}) "
        f"meas(l={status.meas_left_rpm:4d}, r={status.meas_right_rpm:4d}) "
        f"age(cmd={status.age_set_twist_ms:4d}ms link={status.age_link_ms:4d}ms) "
        f"drops(crc={status.drop_crc} seq={status.drop_seq})"
    )


def cmd_monitor(client: RaspVisionCarClient, duration: float) -> None:
    end = time.monotonic() + duration if duration > 0 else float("inf")
    while time.monotonic() < end:
        for status in client.poll_status():
            print_status(status)
        time.sleep(0.01)


def cmd_twist(client: RaspVisionCarClient, v_mmps: int, w_mradps: int, duration: float, reverse_allowed: bool) -> None:
    period = 1.0 / 50.0
    hb_period = 1.0 / 10.0
    next_twist = time.monotonic()
    next_hb = time.monotonic()
    end = time.monotonic() + duration if duration > 0 else float("inf")

    try:
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_twist:
                client.set_twist(v_mmps=v_mmps, w_mradps=w_mradps, reverse_allowed=reverse_allowed)
                next_twist += period
            if now >= next_hb:
                client.heartbeat()
                next_hb += hb_period

            for status in client.poll_status():
                print_status(status)

            time.sleep(0.002)
    finally:
        client.set_twist(0, 0, reverse_allowed=True)


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="RaspVisionCar STM32 protocol client")
    p.add_argument("--port", required=True, help="serial port, e.g. /dev/ttyS0 or /dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=115200)

    sub = p.add_subparsers(dest="cmd", required=True)

    sub.add_parser("arm")
    sub.add_parser("disarm")
    sub.add_parser("estop")
    sub.add_parser("clear-estop")
    sub.add_parser("heartbeat")

    p_twist = sub.add_parser("twist")
    p_twist.add_argument("--v-mmps", type=int, required=True)
    p_twist.add_argument("--w-mradps", type=int, default=0)
    p_twist.add_argument("--duration", type=float, default=2.0, help="seconds, <=0 means run forever")
    p_twist.add_argument("--no-reverse", action="store_true")

    p_mon = sub.add_parser("monitor")
    p_mon.add_argument("--duration", type=float, default=10.0, help="seconds, <=0 means run forever")

    return p


def main() -> None:
    args = build_arg_parser().parse_args()

    with RaspVisionCarClient(port=args.port, baudrate=args.baud) as client:
        if args.cmd == "arm":
            client.arm()
        elif args.cmd == "disarm":
            client.disarm()
        elif args.cmd == "estop":
            client.estop()
        elif args.cmd == "clear-estop":
            client.clear_estop()
        elif args.cmd == "heartbeat":
            client.heartbeat()
        elif args.cmd == "twist":
            cmd_twist(
                client=client,
                v_mmps=args.v_mmps,
                w_mradps=args.w_mradps,
                duration=args.duration,
                reverse_allowed=not args.no_reverse,
            )
        elif args.cmd == "monitor":
            cmd_monitor(client=client, duration=args.duration)
        else:
            raise RuntimeError(f"unsupported cmd: {args.cmd}")

        if args.cmd in {"arm", "disarm", "estop", "clear-estop", "heartbeat"}:
            deadline = time.monotonic() + 1.0
            while time.monotonic() < deadline:
                for status in client.poll_status():
                    print_status(status)
                time.sleep(0.01)

        print("client_stats:", client.stats)


if __name__ == "__main__":
    main()
