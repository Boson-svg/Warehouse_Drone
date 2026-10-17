#!/usr/bin/env bash
# ==============================================================================
# QR Vision Performance Monitor
#
# 只做旁路监控，不启动 qr_vision_node。
# 默认每秒刷新一次：YOLO 进程 CPU/RSS、系统 CPU/内存、温度、NPU load/freq、日志 FPS。
#
# 用法：
#   ./src/drone_perception/scripts/monitor_qr_perf.sh
#   ./src/drone_perception/scripts/monitor_qr_perf.sh --interval 0.5
#
# 如果 NPU load 读不到，通常是 debugfs 权限问题，可以尝试：
#   sudo ./src/drone_perception/scripts/monitor_qr_perf.sh
# ==============================================================================
set -euo pipefail

DEFAULT_HOME="${HOME}"
if [[ -n "${SUDO_USER:-}" && "${SUDO_USER}" != "root" ]]; then
  SUDO_HOME="$(getent passwd "${SUDO_USER}" | cut -d: -f6 || true)"
  if [[ -n "${SUDO_HOME}" ]]; then
    DEFAULT_HOME="${SUDO_HOME}"
  fi
fi

export QR_PERF_DEFAULT_HOME="${DEFAULT_HOME}"

python3 -u - "$@" <<'PY'
import argparse
import glob
import os
import re
import signal
import sys
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


def one_line(text, limit=120):
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


def format_freq(raw):
    if raw is None:
        return "n/a"
    raw = raw.strip()
    if not raw:
        return "n/a"
    if raw.isdigit():
        value = int(raw)
        if value >= 1_000_000:
            return f"{value / 1_000_000:.0f}MHz"
        if value >= 1_000:
            return f"{value / 1_000:.0f}MHz"
    return one_line(raw, 80)


def format_hz(value):
    try:
        hz = float(value)
    except (TypeError, ValueError):
        return "n/a"
    if hz >= 1_000_000_000:
        return f"{hz / 1_000_000_000:.2f}GHz"
    if hz >= 1_000_000:
        return f"{hz / 1_000_000:.0f}MHz"
    if hz >= 1_000:
        return f"{hz / 1_000:.0f}KHz"
    return f"{hz:.0f}Hz"


def read_system_cpu():
    text, _ = read_text("/proc/stat")
    if not text:
        return None
    first = text.splitlines()[0].split()
    if not first or first[0] != "cpu":
        return None
    values = [int(item) for item in first[1:]]
    idle = values[3] + (values[4] if len(values) > 4 else 0)
    return sum(values), idle


def read_meminfo():
    result = {}
    text, _ = read_text("/proc/meminfo")
    if not text:
        return None
    for line in text.splitlines():
        key, _, rest = line.partition(":")
        parts = rest.strip().split()
        if parts:
            result[key] = int(parts[0]) * 1024
    total = result.get("MemTotal")
    available = result.get("MemAvailable")
    if total is None or available is None:
        return None
    return total, total - available


def read_cpu_freqs():
    values = []
    for path in glob.glob("/sys/devices/system/cpu/cpu[0-9]*/cpufreq/scaling_cur_freq"):
        text, _ = read_text(path)
        if text and text.isdigit():
            values.append(int(text) / 1000.0)
    if not values:
        return "n/a"
    return f"{min(values):.0f}-{max(values):.0f}MHz"


def read_proc_times(pid):
    text, _ = read_text(f"/proc/{pid}/stat")
    if not text:
        return None
    try:
        after_comm = text.rsplit(")", 1)[1].strip().split()
        return int(after_comm[11]) + int(after_comm[12])
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
            data = handle.read().replace(b"\x00", b" ").strip()
        return data.decode("utf-8", errors="replace")
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
        if "monitor_qr_perf.sh" in haystack:
            continue
        if pattern in comm.lower() or pattern in haystack:
            pids.append(pid)
    return sorted(set(pids))


def read_process_snapshot(pids):
    total_ticks = 0
    rss = 0
    alive = []
    for pid in pids:
        ticks = read_proc_times(pid)
        if ticks is None:
            continue
        total_ticks += ticks
        rss += read_proc_rss(pid)
        alive.append(pid)
    return alive, total_ticks, rss


def read_temperatures():
    zones = []
    for zone in sorted(glob.glob("/sys/class/thermal/thermal_zone*")):
        name, _ = read_text(os.path.join(zone, "type"))
        raw_temp, _ = read_text(os.path.join(zone, "temp"))
        if not name or not raw_temp:
            continue
        try:
            value = float(raw_temp)
        except ValueError:
            continue
        if abs(value) > 1000:
            value /= 1000.0
        zones.append((name, value))
    preferred = []
    others = []
    for item in zones:
        lower = item[0].lower()
        if any(key in lower for key in ("soc", "cpu", "gpu", "npu", "package", "thermal")):
            preferred.append(item)
        else:
            others.append(item)
    selected = (preferred + others)[:5]
    if not selected:
        return "n/a"
    return "  ".join(f"{name}={temp:.1f}C" for name, temp in selected)


def discover_npu_devfreq():
    result = []
    for path in sorted(glob.glob("/sys/class/devfreq/*")):
        name = os.path.basename(path)
        real = os.path.realpath(path)
        probe = f"{name} {real}".lower()
        if "npu" in probe or "rknpu" in probe:
            result.append(path)
    return result


def parse_rknpu_core_load(raw):
    if not raw:
        return None
    text = one_line(raw, 240)
    pairs = re.findall(r"(?:core|Core)\s*([0-2])\s*[:=]\s*([0-9]+(?:\.[0-9]+)?)\s*%?", text)
    if pairs:
        loads = {int(core): float(load) for core, load in pairs}
        if all(core in loads for core in (0, 1, 2)):
            return "  ".join(f"core{core}={loads[core]:.1f}%" for core in (0, 1, 2))

    numbers = [float(item) for item in re.findall(r"(?<![A-Za-z0-9.])([0-9]+(?:\.[0-9]+)?)\s*%?", text)]
    percent_like = [value for value in numbers if 0.0 <= value <= 100.0]
    if len(percent_like) >= 3:
        return "  ".join(f"core{idx}={percent_like[idx]:.1f}%" for idx in range(3))
    return text


def parse_devfreq_load(raw):
    if not raw:
        return None
    text = one_line(raw, 80)
    match = re.search(r"([0-9]+(?:\.[0-9]+)?)\s*@\s*([0-9]+)\s*Hz", text)
    if match:
        return f"overall={float(match.group(1)):.1f}%  sample_freq={format_hz(match.group(2))}"
    return text


def read_npu_status():
    core_parts = []
    extra_parts = []

    load_path = "/sys/kernel/debug/rknpu/load"
    if os.path.exists(load_path):
        text, err = read_text(load_path)
        if text:
            core_parts.append("debugfs " + parse_rknpu_core_load(text))
        elif err:
            core_parts.append(f"debugfs={err}")

    freq_path = "/sys/kernel/debug/rknpu/load_frequency"
    if os.path.exists(freq_path):
        text, err = read_text(freq_path)
        if text:
            extra_parts.append(f"load_frequency={one_line(text, 80)}")
        elif err:
            extra_parts.append(f"load_frequency={err}")

    for dev in discover_npu_devfreq():
        name = os.path.basename(dev)
        load, _ = read_text(os.path.join(dev, "load"))
        freq, _ = read_text(os.path.join(dev, "cur_freq"))
        status = []
        if load:
            parsed_load = parse_devfreq_load(load)
            if parsed_load:
                status.append(parsed_load)
        status.append(f"freq={format_freq(freq)}")
        extra_parts.append(f"{name}: " + " ".join(status))

    if core_parts:
        if extra_parts:
            return "  ".join(core_parts) + "  |  " + "  ".join(extra_parts)
        return "  ".join(core_parts)
    if extra_parts:
        return "三核占用=n/a (需要读取 /sys/kernel/debug/rknpu/load，尝试 sudo 或挂载 debugfs)  |  " + "  ".join(extra_parts)
    return "n/a (未找到 rknpu/npu sysfs；在 RK3588 上可尝试 sudo 或挂载 debugfs)"


class LogFpsReader:
    def __init__(self, log_dir, process_pattern):
        self.log_dir = os.path.expanduser(log_dir)
        self.process_pattern = process_pattern.lower()
        self.offsets = {}
        self.last_fps = None
        self.last_source = None
        self.last_time = None
        self.regex = re.compile(r"\bfps\s*[=:]\s*([0-9]+(?:\.[0-9]+)?)", re.IGNORECASE)

    def candidate_files(self):
        roots = []
        if os.path.isdir(self.log_dir):
            roots.append(self.log_dir)
        fallback = os.path.join(
            os.environ.get("QR_PERF_DEFAULT_HOME", os.path.expanduser("~")),
            ".ros",
            "log",
            "latest",
        )
        if fallback not in roots and os.path.isdir(fallback):
            roots.append(fallback)
        files = []
        for root in roots:
            files.extend(glob.glob(os.path.join(root, "**", "*.log"), recursive=True))
            files.extend(glob.glob(os.path.join(root, "**", "*.log.*"), recursive=True))
        files = [path for path in files if os.path.isfile(path)]
        files.sort(key=lambda item: os.path.getmtime(item), reverse=True)
        preferred = [path for path in files if self.process_pattern in os.path.basename(path).lower()]
        return (preferred + files)[:20]

    def poll(self):
        for path in self.candidate_files():
            try:
                size = os.path.getsize(path)
            except OSError:
                continue
            offset = self.offsets.get(path)
            if offset is None or offset > size:
                offset = max(0, size - 8192)
            try:
                with open(path, "r", encoding="utf-8", errors="replace") as handle:
                    handle.seek(offset)
                    data = handle.read()
                    self.offsets[path] = handle.tell()
            except OSError:
                continue
            for match in self.regex.finditer(data):
                try:
                    self.last_fps = float(match.group(1))
                    self.last_source = os.path.basename(path)
                    self.last_time = time.time()
                except ValueError:
                    pass
        if self.last_fps is None:
            return "n/a (等待 ROS 日志里出现 fps=...)"
        age = time.time() - self.last_time if self.last_time else 0.0
        return f"{self.last_fps:.1f}  source={self.last_source}  age={age:.0f}s"


def parse_args():
    parser = argparse.ArgumentParser(description="Monitor qr_vision_node performance without launching it.")
    parser.add_argument("--interval", type=float, default=float(os.environ.get("QR_PERF_INTERVAL", "1.0")),
                        help="刷新间隔秒数，默认 1.0")
    parser.add_argument("--process", default=os.environ.get("QR_PERF_PROCESS_PATTERN", "qr_vision_node"),
                        help="进程匹配关键词，默认 qr_vision_node")
    parser.add_argument("--log-dir", default=os.environ.get(
        "QR_PERF_ROS_LOG_DIR",
        os.path.join(os.environ.get("QR_PERF_DEFAULT_HOME", os.path.expanduser("~")), ".ros/log/latest"),
    ), help="ROS 日志目录，默认 ~/.ros/log/latest")
    parser.add_argument("--once", action="store_true", help="只采样一次，用于测试脚本是否可运行")
    parser.add_argument("--no-clear", action="store_true", help="不清屏，按行连续输出")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.interval <= 0:
        raise SystemExit("--interval must be greater than 0")

    stop = False

    def handle_signal(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    fps_reader = LogFpsReader(args.log_dir, args.process)
    prev_total = read_system_cpu()
    prev_pids = find_pids(args.process)
    prev_alive, prev_proc_ticks, _ = read_process_snapshot(prev_pids)
    time.sleep(min(args.interval, 0.2))

    while not stop:
        start = time.time()
        cpu_now = read_system_cpu()
        pids = find_pids(args.process)
        alive, proc_ticks, proc_rss = read_process_snapshot(pids)

        system_cpu = "n/a"
        proc_cpu = "n/a"
        if prev_total and cpu_now:
            total_delta = cpu_now[0] - prev_total[0]
            idle_delta = cpu_now[1] - prev_total[1]
            if total_delta > 0:
                system_cpu = f"{(total_delta - idle_delta) * 100.0 / total_delta:.1f}%"
                proc_delta = proc_ticks - prev_proc_ticks if alive == prev_alive else 0
                proc_cpu = f"{max(0.0, proc_delta * CPU_COUNT * 100.0 / total_delta):.1f}%"

        meminfo = read_meminfo()
        if meminfo:
            total_mem, used_mem = meminfo
            mem_text = f"{format_bytes(used_mem)}/{format_bytes(total_mem)} ({used_mem * 100.0 / total_mem:.1f}%)"
        else:
            mem_text = "n/a"

        pid_text = ",".join(str(pid) for pid in alive) if alive else "not found"
        proc_rss_text = format_bytes(proc_rss) if alive else "n/a"
        lines = [
            "QR Vision 性能监控（只读，不启动节点）",
            f"时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}  interval={args.interval:.2f}s  process={args.process}",
            f"YOLO进程: pid={pid_text}  CPU={proc_cpu}  RSS={proc_rss_text}",
            f"系统资源: CPU={system_cpu}  CPU频率={read_cpu_freqs()}  内存={mem_text}",
            f"温度: {read_temperatures()}",
            f"NPU: {read_npu_status()}",
            f"日志FPS: {fps_reader.poll()}",
            "",
            "提示: 如果 FPS 一直是 n/a，说明脚本还没在 ~/.ros/log/latest 中读到 qr_vision_node 的 fps 日志。",
            "      如果 NPU load 是 n/a/permission denied，在 OrangePi/RK3588 上尝试 sudo 运行本脚本。",
            "退出: Ctrl+C",
        ]

        if not args.no_clear:
            print("\033[2J\033[H", end="")
        print("\n".join(lines), flush=True)

        prev_total = cpu_now
        prev_alive = alive
        prev_proc_ticks = proc_ticks

        if args.once:
            break
        elapsed = time.time() - start
        time.sleep(max(0.0, args.interval - elapsed))


if __name__ == "__main__":
    main()
PY
