# Pi-STM32 Motion Control Protocol

This document describes the current binary control protocol implemented in STM32 firmware and the matching Python client.

## 1. Overview

The Raspberry Pi is the motion decision maker.  
STM32 is the actuator/safety controller:

- Parse and validate serial frames
- Enforce safety state machine (`DISARMED`, `ARMED`, `ESTOP_LATCHED`)
- Convert twist commands to wheel RPM
- Run motor PID + feedforward
- Publish compact status frames

## 2. Serial Settings

- Port: configured on host (for example `/dev/ttyS0` or `/dev/ttyUSB0`)
- Baud: `115200`
- Data bits: `8`
- Parity: `None`
- Stop bits: `1`

## 3. Frame Format

All messages (Pi->STM32 and STM32->Pi) use the same frame:

- `SOF1` (1 byte): `0xA5`
- `SOF2` (1 byte): `0x5A`
- `VER` (1 byte): `0x01`
- `TYPE` (1 byte)
- `SEQ` (1 byte)
- `LEN` (1 byte, payload length, `0..48`)
- `PAYLOAD` (`LEN` bytes)
- `CRC16_LO` (1 byte)
- `CRC16_HI` (1 byte)

Total frame size = `8 + LEN`.

### CRC

- Algorithm: `CRC-16/CCITT-FALSE`
- Poly: `0x1021`
- Init: `0xFFFF`
- Input/Output reflection: no
- Final XOR: none
- CRC coverage: bytes `VER..PAYLOAD` (not including SOF or CRC bytes)

## 4. Message Types

### 4.1 Pi -> STM32

- `0x01 SET_TWIST`, payload length `6`
  - `int16 v_mmps`
  - `int16 w_mradps`
  - `uint8 control_flags` (`bit0 = reverse_allowed`)
  - `uint8 reserved`
- `0x02 ARM`, payload length `0`
- `0x03 DISARM`, payload length `0`
- `0x04 ESTOP`, payload length `0`
- `0x05 CLEAR_ESTOP`, payload length `0`
- `0x06 HEARTBEAT`, payload length `0`

All multibyte values are little-endian.

### 4.2 STM32 -> Pi

- `0x80 STATUS`, payload length `36`
  - `uint8 control_state`
  - `uint8 last_error`
  - `uint8 last_rx_seq`
  - `uint8 reserved`
  - `int16 target_left_rpm`
  - `int16 target_right_rpm`
  - `int16 meas_left_rpm`
  - `int16 meas_right_rpm`
  - `int16 cmd_v_mmps`
  - `int16 cmd_w_mradps`
  - `int32 odom_left`
  - `int32 odom_right`
  - `uint16 age_set_twist_ms`
  - `uint16 age_link_ms`
  - `uint32 drop_crc`
  - `uint32 drop_seq`

Python `struct` format for status payload:

```python
"<BBBBhhhhhhiiHHII"
```

## 5. Sequence Rules

STM32 accepts a frame sequence when:

- First frame initializes sequence state
- Afterwards: `delta = (seq - last_seq) mod 256` must be in `1..127`

Rejected as stale/duplicate:

- `delta == 0`
- `delta > 127`

## 6. Safety State Machine

States:

- `0 DISARMED`
- `1 ARMED`
- `2 ESTOP_LATCHED`

Transitions:

- Boot -> `DISARMED`
- `ARM`: `DISARMED -> ARMED`
- `DISARM`: any state -> `DISARMED`
- `ESTOP`: any state -> `ESTOP_LATCHED`
- `CLEAR_ESTOP`: `ESTOP_LATCHED -> DISARMED`

Behavior:

- `SET_TWIST` is ignored while `ESTOP_LATCHED`
- Motion output is produced only in `ARMED`
- `SET_TWIST` timeout: `200 ms`
  - If stale, target outputs go to zero immediately

## 7. Control Pipeline in STM32

1. UART DMA receives bytes
2. Ring buffer extracts complete frames by SOF + LEN
3. Protocol layer validates version/length/CRC
4. Session layer validates sequence and state transitions
5. Twist -> wheel RPM conversion (`mm/s`, `mrad/s`)
6. Clamp + slew limit on RPM
7. Motor controller applies targets to PID + feedforward loop

## 8. Kinematics Defaults (current firmware)

- `wheel_radius_mm = 33.0`
- `track_width_mm = 140.0`
- `max_linear_mmps = 900`
- `max_angular_mradps = 4500`
- `max_wheel_rpm = 220`
- `max_rpm_tep_per_cycle = 30`

## 9. Host-Side Recommended Behavior

- Send `SET_TWIST` at `50 Hz`
- Send `HEARTBEAT` at `10 Hz`
- Send `ARM` once before movement
- On emergency: send `ESTOP`
- Recovery flow: `CLEAR_ESTOP` -> `ARM` -> resume `SET_TWIST`
- Consume `STATUS` continuously for monitoring

## 10. Python Client

Matching Python script:

- `tools/pi_control/pi_controller.py`

Dependency:

```bash
pip3 install pyserial
```

Examples:

```bash
# Arm
python3 tools/pi_control/pi_controller.py --port /dev/ttyS0 arm

# Drive for 5 seconds (v=300 mm/s, w=0)
python3 tools/pi_control/pi_controller.py --port /dev/ttyS0 twist --v-mmps 300 --w-mradps 0 --duration 5

# Monitor status frames
python3 tools/pi_control/pi_controller.py --port /dev/ttyS0 monitor --duration 10

# Emergency stop
python3 tools/pi_control/pi_controller.py --port /dev/ttyS0 estop
```

## 11. Troubleshooting

- No response from STM32:
  - check port and baud
  - verify TX/RX/GND wiring
  - ensure frame SOF is `A5 5A`
- Car does not move:
  - check `control_state` in status (`ARMED` required)
  - verify `age_set_twist_ms <= 200`
  - ensure not in `ESTOP_LATCHED`
- Frequent dropped frames:
  - inspect `drop_crc` and `drop_seq`
  - reduce serial noise, check cable/ground
  - ensure monotonic sequence on Pi sender
