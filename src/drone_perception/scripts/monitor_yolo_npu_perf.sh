#!/usr/bin/env bash
# ==============================================================================
# YOLO NPU Performance Monitor
#
# 只做旁路监控：先启动 YOLO 推理节点，再运行本脚本观察 RK3588 NPU/进程压力。
#
# 用法：
#   ./src/drone_perception/scripts/monitor_yolo_npu_perf.sh
#   ./src/drone_perception/scripts/monitor_yolo_npu_perf.sh --interval 0.5
#   ./src/drone_perception/scripts/monitor_yolo_npu_perf.sh --duration 30 --no-clear
#   sudo ./src/drone_perception/scripts/monitor_yolo_npu_perf.sh --interval 0.5
# ==============================================================================
set -euo pipefail

python3 -u - "$@" <<'PY'
import argparse
import glob
import os
import re
import signal
import time
from datetime import datetime


CLK_TCK = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
PAGE_SIZE = os.sysconf("SC_PAGE_SIZE")
CPU_COUNT = os.cpu_count() or 1


def read_text(path):
    try:
        with open(path, "r", encoding="utf-8", errors="replace") as handle:
            return handle.read().strip(), None
    except PermissionError:
        return None, "permission denied"
    except OSError as exc:
        return None, exc.strerror or str(exc)


def one_line(text, limit=160):
    text = re.sub(r"\s+", " ", text.strip())
    if len(text) > limit:
        return text[: limit - 3] + "..."
    return text


def format_bytes(value):
    units = ["B", "KB", "MB", "GB"]
    value = float(value)
    for unit in units:
        if value < 1024.0 or unit == units[-1]:
            return f"{value:.1f}{unit}"
        value /= 1024.0
    return f"{value:.1f}GB"


def format_freq_value(value):
    if value is None:
        return "n/a"
    try:
        freq = float(value)
    except (TypeError, ValueError):
        return one_line(str(value), 80)
    if freq >= 1_000_000_000:
        return f"{freq / 1_000_000_000:.2f}GHz"
    if freq >= 1_000_000:
        return f"{freq / 1_000_000:.0f}MHz"
    if freq >= 1_000:
        return f"{freq / 1_000:.0f}KHz"
    return f"{freq:.0f}Hz"


def format_percent(value):
    if value is None:
        return "n/a"
    return f"{value:.1f}%"


def parse_percent_numbers(text):
    values = []
    for match in re.finditer(r"(?<![A-Za-z0-9.])([0-9]+(?:\.[0-9]+)?)\s*%?", text):
        try:
            value = float(match.group(1))
        except ValueError:
            continue
        if 0.0 <= value <= 100.0:
            values.append(value)
    return values


def parse_devfreq_load(text):
    if not text:
        return None, None
    text = one_line(text, 120)
    match = re.search(r"([0-9]+(?:\.[0-9]+)?)\s*@\s*([0-9]+)\s*Hz", text)
    if match:
        return float(match.group(1)), int(match.group(2))
    numbers = parse_percent_numbers(text)
    if numbers:
        return numbers[0], None
    return None, None


def parse_core_load(text):
    if not text:
        return None
    text = one_line(text, 240)
    pairs = re.findall(r"(?:core|Core)\s*([0-2])\s*[:=]\s*([0-9]+(?:\.[0-9]+)?)\s*%?", text)
    if pairs:
        loads = {int(core): float(load) for core, load in pairs}
        if all(core in loads for core in (0, 1, 2)):
            return [loads[0], loads[1], loads[2]]
    numbers = parse_percent_numbers(text)
    if len(numbers) >= 3:
        return numbers[:3]
    return None


def read_proc_stat(pid):
    text, _ = read_text(f"/proc/{pid}/stat")
    if not text:
        return None
    try:
        fields = text.rsplit(")", 1)[1].strip().split()
        return {
            "ticks": int(fields[11]) + int(fields[12]),
            "threads": int(fields[17]),
        }
    except (IndexError, ValueError):
        return None


def read_proc_rss(pid):
    text, _ = read_text(f"/proc/{pid}/statm")
    if not text:
        return 0
    try:
        return int(text.split()[1]) * PAGE_SIZE
    except (IndexError, ValueError):
        return 0


def read_cmdline(pid):
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as handle:
            return handle.read().replace(b"\x00", b" ").decode("utf-8", errors="replace").strip()
    except OSError:
        return ""


def read_comm(pid):
    text, _ = read_text(f"/proc/{pid}/comm")
    return text or ""


def find_pids(pattern):
    pattern = pattern.lower()
    ignored = {os.getpid(), os.getppid()}
    pids = []
    for path in glob.glob("/proc/[0-9]*"):
        pid = int(os.path.basename(path))
        if pid in ignored:
            continue
        comm = read_comm(pid)
        cmdline = read_cmdline(pid)
        haystack = f"{comm} {cmdline}".lower()
        if "monitor_yolo_npu_perf.sh" in haystack:
            continue
        if pattern in comm.lower() or pattern in haystack:
            pids.append(pid)
    return sorted(set(pids))


def read_process_snapshot(pids):
    alive = []
    ticks = 0
    rss = 0
    threads = 0
    for pid in pids:
        stat = read_proc_stat(pid)
        if stat is None:
            continue
        alive.append(pid)
        ticks += stat["ticks"]
        threads += stat["threads"]
        rss += read_proc_rss(pid)
    return alive, ticks, rss, threads


def read_system_cpu():
    text, _ = read_text("/proc/stat")
    if not text:
        return None
    fields = text.splitlines()[0].split()
    if not fields or fields[0] != "cpu":
        return None
    values = [int(item) for item in fields[1:]]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return sum(values), idle


def read_temperatures():
    result = []
    for zone in sorted(glob.glob("/sys/class/thermal/thermal_zone*")):
        name, _ = read_text(os.path.join(zone, "type"))
        raw, _ = read_text(os.path.join(zone, "temp"))
        if not name or not raw:
            continue
        try:
            value = float(raw)
        except ValueError:
            continue
        if abs(value) > 1000:
            value /= 1000.0
        lower = name.lower()
        if any(key in lower for key in ("soc", "cpu", "gpu", "npu", "center", "thermal")):
            result.append(f"{name}={value:.1f}C")
    return "  ".join(result[:6]) if result else "n/a"


def discover_npu_devfreq():
    devices = []
    for path in sorted(glob.glob("/sys/class/devfreq/*")):
        probe = f"{os.path.basename(path)} {os.path.realpath(path)}".lower()
        if "npu" in probe or "rknpu" in probe:
            devices.append(path)
    return devices


def read_npu_snapshot():
    snapshot = {
        "devfreq": [],
        "debugfs_raw": None,
        "debugfs_err": None,
        "core_loads": None,
    }
    debug_path = "/sys/kernel/debug/rknpu/load"
    if os.path.exists(debug_path):
        raw, err = read_text(debug_path)
        snapshot["debugfs_raw"] = raw
        snapshot["debugfs_err"] = err
        if raw:
            snapshot["core_loads"] = parse_core_load(raw)

    for dev in discover_npu_devfreq():
        name = os.path.basename(dev)
        load_raw, load_err = read_text(os.path.join(dev, "load"))
        freq_raw, _ = read_text(os.path.join(dev, "cur_freq"))
        gov_raw, _ = read_text(os.path.join(dev, "governor"))
        load_value, sample_freq = parse_devfreq_load(load_raw)
        snapshot["devfreq"].append({
            "name": name,
            "load_raw": load_raw,
            "load_err": load_err,
            "load": load_value,
            "sample_freq": sample_freq,
            "cur_freq": int(freq_raw) if freq_raw and freq_raw.isdigit() else None,
            "governor": gov_raw or "n/a",
        })
    return snapshot


def scan_rknpu_holders(limit=12):
    holders = []
    for path in glob.glob("/proc/[0-9]*/fd/*"):
        try:
            target = os.readlink(path)
        except OSError:
            continue
        lower_target = target.lower()
        base_target = os.path.basename(lower_target)
        is_npu_device = (
            "rknpu" in lower_target
            or (lower_target.startswith("/dev/") and "npu" in base_target)
        )
        if not is_npu_device:
            continue
        parts = path.split("/")
        if len(parts) < 4:
            continue
        pid = int(parts[2])
        holders.append(f"{pid}:{read_comm(pid) or '?'}")
    return ", ".join(sorted(set(holders))[:limit]) or "n/a"


class Stats:
    def __init__(self):
        self.values = {}

    def add(self, key, value):
        if value is None:
            return
        self.values.setdefault(key, []).append(float(value))

    def summary(self):
        lines = []
        for key in sorted(self.values):
            data = self.values[key]
            if not data:
                continue
            lines.append(
                f"{key}: min={min(data):.1f} avg={sum(data) / len(data):.1f} max={max(data):.1f}"
            )
        return lines


def parse_args():
    parser = argparse.ArgumentParser(description="Monitor YOLO process and RK3588 NPU status.")
    parser.add_argument("--process", default=os.environ.get("YOLO_NPU_PROCESS", "qr_vision_node"),
                        help="进程匹配关键词，默认 qr_vision_node")
    parser.add_argument("--interval", type=float, default=float(os.environ.get("YOLO_NPU_INTERVAL", "1.0")),
                        help="刷新间隔秒数，默认 1.0")
    parser.add_argument("--duration", type=float, default=0.0,
                        help="持续时间秒数；0 表示一直运行，默认 0")
    parser.add_argument("--once", action="store_true", help="只采样一次")
    parser.add_argument("--no-clear", action="store_true", help="不清屏，连续输出")
    parser.add_argument("--show-raw", action="store_true", help="显示 NPU load 原始文本")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.interval <= 0:
        raise SystemExit("--interval must be greater than 0")

    stats = Stats()
    stop = False

    def handle_signal(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    prev_cpu = read_system_cpu()
    prev_pids = find_pids(args.process)
    prev_alive, prev_ticks, _, _ = read_process_snapshot(prev_pids)
    start_time = time.time()
    time.sleep(min(args.interval, 0.2))

    while not stop:
        loop_start = time.time()
        cpu_now = read_system_cpu()
        pids = find_pids(args.process)
        alive, ticks, rss, threads = read_process_snapshot(pids)
        npu = read_npu_snapshot()

        system_cpu = None
        proc_cpu = None
        if prev_cpu and cpu_now:
            total_delta = cpu_now[0] - prev_cpu[0]
            idle_delta = cpu_now[1] - prev_cpu[1]
            if total_delta > 0:
                system_cpu = (total_delta - idle_delta) * 100.0 / total_delta
                proc_delta = ticks - prev_ticks if alive == prev_alive else 0
                proc_cpu = max(0.0, proc_delta * CPU_COUNT * 100.0 / total_delta)

        stats.add("system_cpu_pct", system_cpu)
        stats.add("process_cpu_pct", proc_cpu)
        stats.add("process_rss_mb", rss / (1024 * 1024) if alive else None)
        stats.add("process_threads", threads if alive else None)

        npu_lines = []
        if npu["core_loads"] is not None:
            for idx, value in enumerate(npu["core_loads"]):
                stats.add(f"npu_core{idx}_pct", value)
            npu_lines.append("debugfs cores: " + "  ".join(
                f"core{idx}={value:.1f}%" for idx, value in enumerate(npu["core_loads"])
            ))
        elif npu["debugfs_err"]:
            npu_lines.append(f"debugfs cores: {npu['debugfs_err']}")
        else:
            npu_lines.append("debugfs cores: n/a")

        for item in npu["devfreq"]:
            stats.add(f"{item['name']}_load_pct", item["load"])
            stats.add(f"{item['name']}_freq_mhz", item["cur_freq"] / 1_000_000 if item["cur_freq"] else None)
            parts = [
                f"{item['name']}: load={format_percent(item['load'])}",
                f"freq={format_freq_value(item['cur_freq'])}",
                f"governor={item['governor']}",
            ]
            if item["sample_freq"] is not None:
                parts.append(f"sample={format_freq_value(item['sample_freq'])}")
            if item["load_err"] and item["load"] is None:
                parts.append(f"load_err={item['load_err']}")
            npu_lines.append(" ".join(parts))

        if not npu["devfreq"] and npu["core_loads"] is None:
            npu_lines.append("未找到 NPU devfreq；请确认在 RK3588 板端运行")

        lines = [
            "YOLO NPU 性能监控（只读，不启动节点）",
            f"时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}  interval={args.interval:.2f}s  process={args.process}",
            f"YOLO进程: pid={','.join(map(str, alive)) if alive else 'not found'}  CPU={format_percent(proc_cpu)}  RSS={format_bytes(rss) if alive else 'n/a'}  threads={threads if alive else 'n/a'}",
            f"系统CPU: {format_percent(system_cpu)}",
            f"温度: {read_temperatures()}",
            "NPU: " + "  |  ".join(npu_lines),
            f"rknpu设备占用者: {scan_rknpu_holders()}",
        ]
        if args.show_raw:
            raw_debug = one_line(npu["debugfs_raw"] or "n/a", 220)
            raw_devfreq = "; ".join(
                f"{item['name']} load_raw={one_line(item['load_raw'] or 'n/a', 80)}"
                for item in npu["devfreq"]
            ) or "n/a"
            lines.extend([f"RAW debugfs: {raw_debug}", f"RAW devfreq: {raw_devfreq}"])
        lines.extend([
            "",
            "说明: devfreq load=总体负载；debugfs cores 才是三核分别占用，读不到时请 sudo 或挂载 debugfs。",
            "退出: Ctrl+C",
        ])

        if not args.no_clear:
            print("\033[2J\033[H", end="")
        print("\n".join(lines), flush=True)

        prev_cpu = cpu_now
        prev_alive = alive
        prev_ticks = ticks

        if args.once:
            break
        if args.duration > 0 and time.time() - start_time >= args.duration:
            break
        elapsed = time.time() - loop_start
        time.sleep(max(0.0, args.interval - elapsed))

    summary = stats.summary()
    if summary:
        print("\n采样汇总:")
        print("\n".join(summary))


if __name__ == "__main__":
    main()
PY
