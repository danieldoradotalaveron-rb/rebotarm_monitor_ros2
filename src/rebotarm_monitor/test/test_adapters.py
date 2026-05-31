"""Tests for the outbound adapters (sysfs + process inspector)."""

from __future__ import annotations

from pathlib import Path

from rebotarm_monitor.adapters import (
    FakeProcessInspector,
    FakeSysFsReader,
    ProcessSnapshot,
    RealSysFsReader,
)


def test_fake_sysfs_reader_returns_registered_values():
    fake = FakeSysFsReader(
        {
            "/sys/class/net/can0/operstate": "up\n",
            "/sys/class/net/can0/carrier": "1\n",
            "/sys/class/net/can0/statistics/rx_packets": "42",
        }
    )
    assert fake.read_text("/sys/class/net/can0/operstate") == "up"
    assert fake.read_int("/sys/class/net/can0/carrier") == 1
    assert fake.read_int("/sys/class/net/can0/statistics/rx_packets") == 42
    assert fake.read_int("/sys/class/net/can0/missing") is None
    assert fake.read_text("/sys/class/net/can0/missing") is None


def test_fake_sysfs_is_dir_uses_path_prefixes():
    fake = FakeSysFsReader({"/sys/class/net/can0/statistics/rx_packets": "0"})
    assert fake.is_dir("/sys/class/net/can0") is True
    assert fake.is_dir("/sys/class/net/can0/statistics") is True
    assert fake.is_dir("/sys/class/net/can1") is False


def test_real_sysfs_reader_reads_filesystem(tmp_path: Path):
    iface_dir = tmp_path / "can0"
    stats_dir = iface_dir / "statistics"
    stats_dir.mkdir(parents=True)
    (iface_dir / "operstate").write_text("up\n")
    (iface_dir / "carrier").write_text("1\n")
    (stats_dir / "rx_packets").write_text("123\n")
    (stats_dir / "rx_errors").write_text("not-a-number\n")

    reader = RealSysFsReader()
    assert reader.is_dir(str(iface_dir)) is True
    assert reader.is_dir(str(tmp_path / "missing")) is False
    assert reader.read_text(str(iface_dir / "operstate")) == "up"
    assert reader.read_int(str(iface_dir / "carrier")) == 1
    assert reader.read_int(str(stats_dir / "rx_packets")) == 123
    assert reader.read_int(str(stats_dir / "rx_errors")) is None
    assert reader.read_text(str(iface_dir / "missing")) is None


def test_fake_process_inspector_round_trip():
    snap = ProcessSnapshot(
        pid=4242,
        status="running",
        cpu_percent=12.5,
        rss_mb=128.0,
        num_threads=8,
        num_fds=42,
        create_time=1_700_000_000.0,
    )
    fake = FakeProcessInspector(snapshot=snap)
    assert fake.available() is True
    assert fake.find("reBotArmController") is snap
    assert fake.refresh(4242) is snap

    fake.set_available(False)
    assert fake.available() is False

    fake.set_available(True)
    fake.set_snapshot(None)
    assert fake.find("anything") is None
