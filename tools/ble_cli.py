#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ble_cli.py —— 机械手 BLE 指令工具（本项目自写，不依赖原版上位机）

为什么需要它：
  上位机是打包好的 exe，改不动；调试协议帧时手敲 `$S1,120*7D` 极易打错，
  而错一个字符的表现是"手指不动"，看不出是格式错、校验错还是没连上。
  这个工具负责把"指令"翻译成"帧"，并把固件回传的日志和命令对齐。

三种用法：
  1) 离线自测（不需要板子、不需要 bleak）
       python ble_cli.py --selftest
  2) 扫描 / 连接后发指令
       python ble_cli.py --scan
       python ble_cli.py --send "servo_1=120"          # 旧格式，原样发
       python ble_cli.py --frame "S1,120"              # 自动补帧头与校验
       python ble_cli.py --frame "GFIST"
  3) 连发压测（配合固件 test=stream 观察）
       python ble_cli.py --frame "S1,120" --repeat 20 --interval 0.5

依赖：bleak（只有联网部分需要；--selftest 不需要）
       pip install bleak
"""

import argparse
import asyncio
import sys

# ---- 与固件保持一致的常量 ------------------------------------------------
DEVICE_NAME = "xiaochu"
SERVICE_UUID = "89d60870-9908-4472-8f8c-e5b3e6573cd1"
WRITE_CHAR_UUID = "39dea685-a63e-44b2-8819-9a202581f8fe"

# 固件内置用例里的期望校验值，改校验算法时这里会第一时间报错。
# 如果这里和 docs/protocol.md 不一致，以手算/固件实测为准——别改代码去迁就文档。
SELFTEST_VECTORS = [
    ("S1,120", 0x7D),
    ("S1,90", 0x47),
    ("GFIST", 0x4F),
    ("GFIVE", 0x5B),
    ("S1,45", 0x4F),
]


def body_checksum(body: str) -> int:
    """与固件 parseFrame() 一致：body 逐字节异或。"""
    v = 0
    for ch in body.encode("utf-8"):
        v ^= ch
    return v


def build_frame(body: str) -> str:
    """把 'S1,120' 变成 '$S1,120*7D'。"""
    return "$%s*%02X" % (body, body_checksum(body))


def parse_frame(frame: str):
    """解析帧，返回 (ok, body, 期望校验值)。

    ok=False 时第二项是错误原因，第三项是"按当前算法应该是什么校验值"。
    这个返回值设计是为了让报错能直接告诉你"应该是多少"，而不是只说错了。
    """
    if not frame.startswith("$"):
        return False, "缺少帧头 '$'", None
    if "*" not in frame:
        return False, "缺少 '*' 分隔符", None
    body, _, chk = frame[1:].rpartition("*")
    if len(chk) != 2:
        return False, "校验位不是 2 位", None
    try:
        given = int(chk, 16)
    except ValueError:
        return False, "校验位不是十六进制", None
    want = body_checksum(body)
    if given != want:
        return False, "校验不匹配", want
    return True, body, want


def run_selftest() -> int:
    """离线验证校验算法与固件内置用例一致。不需要板子。"""
    failed = 0
    print("[selftest] 校验向量")
    for body, expect in SELFTEST_VECTORS:
        got = body_checksum(body)
        ok = got == expect
        failed += 0 if ok else 1
        print("  %-8s 期望 %02X  实际 %02X  %s"
              % (body, expect, got, "PASS" if ok else "FAIL"))

    print("[selftest] 组帧 / 拆帧往返")
    for body, _ in SELFTEST_VECTORS:
        frame = build_frame(body)
        ok, parsed, _ = parse_frame(frame)
        good = ok and parsed == body
        failed += 0 if good else 1
        print("  %-8s -> %-12s 回读 %-8s %s"
              % (body, frame, parsed if ok else parsed, "PASS" if good else "FAIL"))

    print("[selftest] 应当被拒收的帧")
    bad_frames = [
        ("S1,120*7D", "缺帧头"),
        ("$S1,120", "缺分隔符"),
        ("$S1,120*0", "校验位长度不对"),
        ("$S1,120*ZZ", "校验位非十六进制"),
        ("$S1,120*00", "校验值错误"),
    ]
    for frame, why in bad_frames:
        ok, msg, want = parse_frame(frame)
        good = not ok
        failed += 0 if good else 1
        extra = "" if want is None else "（应为 %02X）" % want
        print("  %-12s %-6s %s%s" % (frame, why, "PASS" if good else "FAIL", extra))

    print()
    print("[selftest] %s" % ("全部通过" if failed == 0 else "有 %d 项失败" % failed))
    return 0 if failed == 0 else 1


# ---- BLE 部分：只有用到时才 import bleak --------------------------------
async def cmd_scan(timeout=5.0):
    from bleak import BleakScanner
    print("[scan] %.0f 秒 ..." % timeout)
    devices = await BleakScanner.discover(timeout=timeout)
    if not devices:
        print("[scan] 没发现任何设备。板子是否上电、是否已在广播？")
        return 1
    hit = False
    for d in devices:
        mark = ""
        if d.name and DEVICE_NAME in d.name:
            mark = "  <- 目标"
            hit = True
        print("  %-40s %s%s" % (d.address, d.name or "(无名)", mark))
    if not hit:
        print("[scan] 没找到名为 %s 的设备" % DEVICE_NAME)
        return 1
    return 0


async def _connect_and_send(payloads, interval):
    from bleak import BleakClient

    print("[ble] 扫描 ...")
    from bleak import BleakScanner
    dev = await BleakScanner.find_device_by_filter(
        lambda d, ad: d.name and DEVICE_NAME in d.name, timeout=8.0)
    if dev is None:
        print("[ble] 未找到 %s。ESP32-C3 只允许一个连接，手机是否还连着？" % DEVICE_NAME)
        return 1

    print("[ble] 连接 %s ..." % (dev.name or dev.address))
    async with BleakClient(dev) as client:
        print("[ble] 已连接: %s" % client.is_connected)
        for i, p in enumerate(payloads, 1):
            await client.write_gatt_char(WRITE_CHAR_UUID, p.encode("utf-8"))
            print("[tx %02d] %s" % (i, p))
            if interval and i < len(payloads):
                await asyncio.sleep(interval)
        print("[ble] 发送完毕，共 %d 条" % len(payloads))
    return 0


def main():
    ap = argparse.ArgumentParser(description="机械手 BLE 指令工具")
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--selftest", action="store_true", help="离线校验算法自测（不需要硬件）")
    g.add_argument("--scan", action="store_true", help="扫描附近的 BLE 设备")
    g.add_argument("--send", metavar="TEXT", help="原样发送一条文本（旧格式，如 servo_1=120）")
    g.add_argument("--frame", metavar="BODY", help="发送带帧头与校验的帧（如 S1,120 或 GFIST）")
    ap.add_argument("--repeat", type=int, default=1, help="重复次数（默认 1）")
    ap.add_argument("--interval", type=float, default=0.5, help="重复间隔秒（默认 0.5）")
    args = ap.parse_args()

    if args.selftest:
        return run_selftest()

    try:
        if args.scan:
            return asyncio.run(cmd_scan())
        if args.send:
            payloads = [args.send] * args.repeat
        else:
            payloads = [build_frame(args.frame)] * args.repeat
        return asyncio.run(_connect_and_send(payloads, args.interval))
    except ImportError:
        print("缺少依赖 bleak：pip install bleak")
        return 2
    except KeyboardInterrupt:
        return 130


if __name__ == "__main__":
    sys.exit(main())
