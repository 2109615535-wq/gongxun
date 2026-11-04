"""为本项目选择与 Lite2 同版的本地 RKNN Runtime；不改系统库。"""

import ctypes
import functools
import hashlib
from importlib.metadata import version as package_version
import logging
import os
from pathlib import Path
import platform
import re


_KNOWN_HASHES = {
    "2.3.2": "d31fc19c85b85f6091b2bd0f6af9d962d5264a4e410bfb536402ec92bac738e8",
}
_STANDARD_LEVELS = (
    (logging.CRITICAL, "FATAL"), (logging.CRITICAL, "CRITICAL"),
    (logging.ERROR, "ERROR"), (logging.WARNING, "WARN"),
    (logging.WARNING, "WARNING"), (logging.INFO, "INFO"),
    (logging.DEBUG, "DEBUG"), (logging.NOTSET, "NOTSET"),
)
_selected_runtime = None


def _restore_logging_names():
    # Lite2 2.3.2 导入会替换标准日志级别映射；保留其短名并补回标准名。
    for number, name in _STANDARD_LEVELS:
        logging.addLevelName(number, name)


def _select_runtime(base, lite_version):
    target = Path(base) / f"runtime-{lite_version}" / "librknnrt.so"
    if not target.is_file():
        raise RuntimeError(
            f"缺少与 Lite2 {lite_version} 同版的本地运行库：{target}。"
            "请先准备并验证官方 ARM64 Linux 运行库；不会回退到旧系统库。"
        )
    data = target.read_bytes()
    if (len(data) < 20 or data[:4] != b"\x7fELF" or data[4:6] != b"\x02\x01"
            or int.from_bytes(data[18:20], "little") != 183):
        raise RuntimeError(f"运行库不是 AArch64 小端 64 位 ELF：{target}")
    versions = set(value.decode("ascii") for value in re.findall(
        rb"librknnrt version:\s*(\d+\.\d+\.\d+)", data))
    if versions != {lite_version}:
        raise RuntimeError(f"运行库内部版本 {sorted(versions)} 与 Lite2 {lite_version} 不一致。")
    expected = _KNOWN_HASHES.get(lite_version)
    if expected is not None and hashlib.sha256(data).hexdigest() != expected:
        raise RuntimeError(f"官方运行库 SHA256 校验失败：{target}，请重新核对文件。")
    return target.resolve()


def configure_rknn_runtime():
    """在加载 RKNN 模型之前调用；当前进程内生效，可重复调用。"""
    global _selected_runtime
    if _selected_runtime is not None:
        return _selected_runtime
    if platform.system() != "Linux" or platform.machine().lower() != "aarch64":
        raise RuntimeError("RKNN 本地运行库配置只适用于 ARM64 Linux 板端。")
    lite_version = package_version("rknn-toolkit-lite2").split("+", 1)[0]
    target = _select_runtime(Path(__file__).resolve().parent, lite_version)

    # 已加载过其他版本时停止，避免一个进程混入两个 Runtime。
    maps = Path("/proc/self/maps")
    if maps.is_file():
        for line in maps.read_text().splitlines():
            if "librknnrt.so" in line:
                loaded = Path(line.split(maxsplit=5)[-1]).resolve()
                if loaded != target:
                    raise RuntimeError(f"进程已加载其他 RKNN 库：{loaded}；请重启程序后再配置。")

    original_cdll = ctypes.CDLL

    @functools.wraps(original_cdll)
    def project_cdll(name, *args, **kwargs):
        if isinstance(name, (str, bytes, os.PathLike)):
            if Path(os.fsdecode(name)).name == "librknnrt.so":
                name = str(target)
        return original_cdll(name, *args, **kwargs)

    # Lite2 2.3.2 向 ctypes.CDLL 传入固定的 /usr/lib/librknnrt.so。
    # 仅重定向这一个库，其他库名称与参数完整透传。
    ctypes.CDLL = project_cdll
    try:
        from rknnlite.api import RKNNLite  # noqa: F401
    except Exception:
        ctypes.CDLL = original_cdll
        raise
    finally:
        _restore_logging_names()
    _selected_runtime = target
    print(f"[RKNN] Lite2 {lite_version}，自动使用本地运行库：{target}", flush=True)
    return target
