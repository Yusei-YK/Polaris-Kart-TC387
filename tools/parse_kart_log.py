#!/usr/bin/env python3
"""Convert TC387 KLOG v1 UART recordings to CSV and summarize subject 1."""

from __future__ import annotations

import argparse
import csv
import re
import struct
from pathlib import Path
from typing import Dict, List, Tuple


FRAME_SIZE = 64
MAGIC = struct.pack("<I", 0x474F4C4B)  # raw bytes: KLOG
VERSION = 1
END_MARK = 0x55AA

MODE_NAMES = {
    0: "IDLE",
    1: "SUBJECT_1",
    2: "SUBJECT_2",
    3: "REMOTE",
    4: "FAULT",
}

STAGE_NAMES = {
    0: "WAIT_START",
    1: "CONE_ROUTE",
    2: "GARAGE_APPROACH",
    3: "REVERSE_IN",
    4: "FINISHED",
    5: "S1_FAULT",
}


def decode_hex_text(raw: bytes) -> bytes:
    text = raw.decode("ascii", errors="strict")
    text = re.sub(r"0[xX]", "", text)
    pairs = re.findall(r"[0-9A-Fa-f]{2}", text)
    return bytes(int(pair, 16) for pair in pairs)


def load_stream(path: Path, input_format: str) -> Tuple[bytes, str]:
    raw = path.read_bytes()
    if input_format == "raw":
        return raw, "raw"
    if input_format == "hex":
        return decode_hex_text(raw), "hex"

    if MAGIC in raw:
        return raw, "raw"

    try:
        decoded = decode_hex_text(raw)
    except UnicodeDecodeError:
        return raw, "raw"
    if MAGIC in decoded:
        return decoded, "hex"
    return raw, "raw"


def unpack_frame(frame: bytes) -> Dict[str, object]:
    flags = frame[7]
    sequence, tick_5ms = struct.unpack_from("<II", frame, 8)
    skipped, playback_index = struct.unpack_from("<HH", frame, 16)
    target, left_speed, right_speed, left_pwm, right_pwm = struct.unpack_from(
        "<hhhhh", frame, 20
    )
    yaw = struct.unpack_from("<h", frame, 30)[0]
    yaw_unwrapped = struct.unpack_from("<i", frame, 32)[0]
    yaw_rate, yaw_bias, target_yaw = struct.unpack_from("<hhh", frame, 36)
    steer_raw = struct.unpack_from("<H", frame, 42)[0]
    steer_target, steer_pwm = struct.unpack_from("<hh", frame, 44)
    odom_x, odom_y = struct.unpack_from("<ii", frame, 48)
    odom_dist = struct.unpack_from("<I", frame, 56)[0]
    acc_norm = struct.unpack_from("<H", frame, 60)[0]

    mode = frame[5]
    stage = frame[6]
    return {
        "sequence": sequence,
        "tick_5ms": tick_5ms,
        "t_ms": tick_5ms * 5,
        "mode": mode,
        "mode_name": MODE_NAMES.get(mode, f"UNKNOWN_{mode}"),
        "stage": stage,
        "stage_name": STAGE_NAMES.get(stage, f"UNKNOWN_{stage}"),
        "playback_running": int(bool(flags & (1 << 0))),
        "speed_enabled": int(bool(flags & (1 << 1))),
        "remote_online": int(bool(flags & (1 << 2))),
        "remote_sw3_low": int(bool(flags & (1 << 3))),
        "head_loop_enabled": int(bool(flags & (1 << 4))),
        "angle_loop_enabled": int(bool(flags & (1 << 5))),
        "skipped_frame_count": skipped,
        "playback_index": playback_index,
        "speed_target_pulse_5ms": target / 100.0,
        "left_speed_pulse_5ms": left_speed / 100.0,
        "right_speed_pulse_5ms": right_speed / 100.0,
        "left_pwm": left_pwm,
        "right_pwm": right_pwm,
        "yaw_wrapped_deg": yaw / 100.0,
        "yaw_unwrapped_deg": yaw_unwrapped / 100.0,
        "yaw_rate_dps": yaw_rate / 100.0,
        "yaw_bias_dps": yaw_bias / 100.0,
        "target_yaw_deg": target_yaw / 100.0,
        "steer_raw": steer_raw,
        "steer_target_delta": steer_target,
        "steer_pwm": steer_pwm,
        "odom_x_m": odom_x / 1000.0,
        "odom_y_m": odom_y / 1000.0,
        "odom_dist_m": odom_dist / 1000.0,
        "acc_norm_g": acc_norm / 1000.0,
    }


def parse_frames(data: bytes) -> Tuple[List[Dict[str, object]], int, int]:
    frames: List[Dict[str, object]] = []
    offset = 0
    skipped_bytes = 0
    damaged_frames = 0

    while offset < len(data):
        found = data.find(MAGIC, offset)
        if found < 0:
            skipped_bytes += len(data) - offset
            break
        skipped_bytes += found - offset
        if found + FRAME_SIZE > len(data):
            skipped_bytes += len(data) - found
            break

        frame = data[found : found + FRAME_SIZE]
        version = frame[4]
        end_mark = struct.unpack_from("<H", frame, 62)[0]
        if version != VERSION or end_mark != END_MARK:
            damaged_frames += 1
            skipped_bytes += 1
            offset = found + 1
            continue

        frames.append(unpack_frame(frame))
        offset = found + FRAME_SIZE

    return frames, skipped_bytes, damaged_frames


def first_stage_time(frames: List[Dict[str, object]], stage: int, start_ms: int) -> int | None:
    for frame in frames:
        if (
            frame["mode"] == 1
            and frame["stage"] == stage
            and int(frame["t_ms"]) >= start_ms
        ):
            return int(frame["t_ms"])
    return None


def print_summary(
    frames: List[Dict[str, object]], skipped_bytes: int, damaged_frames: int, source_format: str
) -> None:
    print(f"输入格式: {source_format}")
    print(f"有效帧: {len(frames)}")
    print(f"跳过字节: {skipped_bytes}")
    print(f"损坏候选帧: {damaged_frames}")
    if not frames:
        print("没有找到KLOG v1有效帧。")
        return

    print(f"MCU累计未发送帧: {max(int(f['skipped_frame_count']) for f in frames)}")

    start_ms = first_stage_time(frames, 1, 0)
    if start_ms is None:
        print("本文件没有进入科目一 CONE_ROUTE。")
        return

    garage_ms = first_stage_time(frames, 2, start_ms)
    reverse_ms = first_stage_time(frames, 3, start_ms)
    finish_ms = first_stage_time(frames, 4, start_ms)

    print(f"科目一启动: {start_ms / 1000.0:.3f} s")
    if garage_ms is not None:
        print(f"绕桩阶段: {(garage_ms - start_ms) / 1000.0:.3f} s")
    if garage_ms is not None and reverse_ms is not None:
        print(f"库口停车等待: {(reverse_ms - garage_ms) / 1000.0:.3f} s")
    if reverse_ms is not None and finish_ms is not None:
        print(f"倒库阶段: {(finish_ms - reverse_ms) / 1000.0:.3f} s")

    if finish_ms is not None:
        print(f"科目一总时间: {(finish_ms - start_ms) / 1000.0:.3f} s")
        return

    subject1_frames = [f for f in frames if f["mode"] == 1 and int(f["t_ms"]) >= start_ms]
    last = subject1_frames[-1] if subject1_frames else frames[-1]
    max_index = max(int(f["playback_index"]) for f in subject1_frames) if subject1_frames else 0
    print("科目一未进入FINISHED。")
    print(f"最后状态: {last['mode_name']} / {last['stage_name']}")
    print(f"运行时长: {(int(last['t_ms']) - start_ms) / 1000.0:.3f} s")
    print(f"最大playback_index: {max_index}")
    print(
        "最后位置: "
        f"x={float(last['odom_x_m']):.3f} m, "
        f"y={float(last['odom_y_m']):.3f} m, "
        f"dist={float(last['odom_dist_m']):.3f} m"
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="记录器导出的原始二进制或HEX文本")
    parser.add_argument("-o", "--output", type=Path, help="输出CSV路径")
    parser.add_argument(
        "--input-format", choices=("auto", "raw", "hex"), default="auto", help="输入格式"
    )
    args = parser.parse_args()

    output = args.output or args.input.with_suffix(".csv")
    data, source_format = load_stream(args.input, args.input_format)
    frames, skipped_bytes, damaged_frames = parse_frames(data)

    if frames:
        with output.open("w", newline="", encoding="utf-8-sig") as csv_file:
            writer = csv.DictWriter(csv_file, fieldnames=list(frames[0].keys()))
            writer.writeheader()
            writer.writerows(frames)
        print(f"CSV: {output}")

    print_summary(frames, skipped_bytes, damaged_frames, source_format)
    return 0 if frames else 1


if __name__ == "__main__":
    raise SystemExit(main())
