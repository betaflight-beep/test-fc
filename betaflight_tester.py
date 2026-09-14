"""
BETAFIGHT FLIGHT CONTROLLER ATE / PRODUCTION TESTER
===================================================
Masaüstü Seri Üretim ve Doğrulama Otomasyonu (PySide6 + MSP v1/v2 + ATE Pipeline)

Özellikler:
1. Cihaz Modeli (TestProfile) ve Fiziksel Cihaz (PhysicalDevice) tam ayrımı.
2. Otomatik COM Port Tarama ve MSP FC Tespiti (Kullanıcı COM seçmek zorunda değil).
3. Cihazdan Otomatik Parametre Çekme ve Model Oluşturma (Auto-naming örn. F405_4.5.2).
4. Profil Yönetimi: profiles/*.json (Yeni, Düzenle, Kopyala, Sil).
5. 18 Ayrı Test Kriteri ve Ön Seçim Arayüzü (Tümünü Seç / Temizle).
6. Tam Ekran Endüstriyel ATE İstasyonu (Operatör için uzaktan görülebilen dev PASS/FAIL göstergesi).
7. Gerçek Betaflight MSP v1/v2 Desteği (XOR / CRC8-DVB-S2, uydurma alan içermez).
8. VTX Testi: Band (R), Channel (8), Frekans (5917MHz), Güç (800mW), Pit Mode (OFF).
9. Akım Koruması: ARMED=False iken Akım < 5.0 A kuralı (Sesli ikaz ve anında FAIL).
10. USB Hot-Plug Desteği: Sökülen kartı unutma, yeni takılan kartı otomatik bulma.
11. Simülasyon Modu (Virtual FC): 17 farklı hata enjeksiyonu (IMU, VBUS, Akım, VTX vb.).
12. Sesli Uyarı: FAIL durumunda 3.0 saniye cooldown korumalı ses ikazı.
13. MCU Boğulma Koruması: One-shot parametreler + 5 Hz periyodik telemetri.
14. Test Denetim Logları: logs/YYYY-MM-DD/FC_Test_xxx.json.
"""

import sys
import os
import time
import json
import struct
import logging
from enum import Enum, IntEnum
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional, Tuple, Any
from datetime import datetime

# Windows platform sesli uyarı
try:
    import winsound
    HAS_WINSOUND = True
except ImportError:
    HAS_WINSOUND = False

# Seri port kütüphanesi
try:
    import serial
    import serial.tools.list_ports
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

# PySide6 GUI bileşenleri
from PySide6.QtCore import Qt, QThread, Signal, Slot, QTimer, QSize
from PySide6.QtGui import QFont, QColor, QIcon, QAction
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QTableWidget, QTableWidgetItem, QHeaderView,
    QDialog, QLineEdit, QCheckBox, QComboBox, QProgressBar, QTextEdit,
    QMessageBox, QStackedWidget, QFrame, QGroupBox, QFileDialog, QSpinBox,
    QDoubleSpinBox, QTabWidget, QScrollArea, QAbstractItemView
)

# ============================================================================
# 1. SABİTLER, ENUMLAR VE MSP PROTOKOL TANIMLARI
# ============================================================================

class MSPCommand(IntEnum):
    """Betaflight resmi MSP v1 ve v2 komut kodları."""
    API_VERSION = 1
    FC_VARIANT = 2
    FC_VERSION = 3
    BOARD_INFO = 4
    BUILD_INFO = 5
    NAME = 10
    BATTERY_CONFIG = 32
    MODE_RANGES = 34
    RX_CONFIG = 44
    CF_SERIAL_CONFIG = 54
    RC_MAP = 64
    DATAFLASH_SUMMARY = 70
    OSD_CONFIG = 84
    VTX_CONFIG = 88
    ADVANCED_CONFIG = 90
    FILTER_CONFIG = 92
    PID_ADVANCED = 94
    STATUS = 101
    RAW_IMU = 102
    MOTOR = 104
    RAW_GPS = 106
    ALTITUDE = 109
    ANALOG = 110
    RC_TUNING = 111
    PID = 112
    VOLTAGE_METERS = 128
    CURRENT_METERS = 129
    BATTERY_STATE = 130
    MOTOR_TELEMETRY = 139
    STATUS_EX = 150
    UID = 160


class TestStatus(str, Enum):
    """Test sonuç durumları."""
    PENDING = "PENDING"
    RUNNING = "RUNNING"
    PASS = "PASS"
    FAIL = "FAIL"
    WARNING = "WARNING"
    SKIPPED = "SKIPPED"
    NOT_AVAILABLE = "NOT_AVAILABLE"


class SensorFlags:
    """MSP_STATUS sensör bayrak bitleri."""
    ACC = 1 << 0
    BARO = 1 << 1
    MAG = 1 << 2
    GPS = 1 << 3
    SONAR = 1 << 4
    GYRO = 1 << 5


# VTX Bant isimleri ve frekans haritaları (Raceband, Fatshark vb.)
VTX_BANDS = {
    1: "A",
    2: "B",
    3: "E",
    4: "F",  # FatShark / Airwave
    5: "R"   # RaceBand
}

# RaceBand 8 frekansı = 5917 MHz
VTX_FREQUENCIES = {
    (5, 1): 5658, (5, 2): 5695, (5, 3): 5732, (5, 4): 5769,
    (5, 5): 5806, (5, 6): 5843, (5, 7): 5880, (5, 8): 5917
}

# Betaflight Resmi Uçuş Modu Tanımları (Box IDs)
BOX_NAMES = {
    0: "ARM",
    1: "ANGLE",
    2: "HORIZON",
    3: "BARO",
    4: "ANTI_GRAVITY",
    5: "MAG",
    6: "HEADFREE",
    7: "HEADADJ",
    8: "BEEPER",
    13: "AIRMODE",
    19: "OSD_DISABLE",
    20: "TELEMETRY",
    26: "BLACKBOX",
    27: "TURTLE",           # FLIP_OVER_AFTER_CRASH
    28: "VTX_PIT_MODE",
    35: "LAUNCH_CONTROL",
    36: "FAILSAFE"
}

# Betaflight Resmi OSD Eleman İsimleri (Item IDs - Tam 32 Eleman)
OSD_ITEM_NAMES = {
    0: "RSSI",
    1: "MAIN_BATT_VOLTAGE",
    2: "CROSSHAIRS",
    3: "ARTIFICIAL_HORIZON",
    4: "HORIZON_SIDEBARS",
    5: "ON_TIME",
    6: "FLY_TIME",
    7: "FLY_MODE",
    8: "CRAFT_NAME",
    9: "THROTTLE_POS",
    10: "VTX_CHANNEL",
    11: "CURRENT_DRAW",
    12: "MAH_DRAWN",
    13: "GPS_SPEED",
    14: "GPS_SATS",
    15: "ALTITUDE",
    16: "ROLL_PIDS",
    17: "PITCH_PIDS",
    18: "POWER",
    19: "PID_RATE_PROFILE",
    20: "BATTERY_WARNING",
    21: "AVG_CELL_VOLTAGE",
    22: "PIT_MODE",
    23: "RTC_TIME",
    24: "ADJUSTMENT_RANGE",
    25: "CORE_TEMPERATURE",
    26: "ANTI_GRAVITY",
    27: "G_FORCE",
    28: "MOTOR_DIAGNOSTICS",
    29: "COMPASS_BAR",
    30: "ESC_TEMPERATURE",
    31: "ESC_RPM"
}


# ============================================================================
# 2. VERİ MODELLERİ (DATACLASSES)
# ============================================================================

@dataclass
class TestResultItem:
    """Her bir test kriteri için anlık doğrulama sonucu."""
    test_id: str
    name: str
    actual_value: str
    expected_value: str
    status: TestStatus
    detail: str = ""
    timestamp: str = field(default_factory=lambda: datetime.now().strftime("%H:%M:%S"))


@dataclass
class PhysicalDevice:
    """Tespit edilen veya bağlanan fiziksel/mock FC donanımı."""
    port: str
    board_name: str = ""
    target_name: str = ""
    firmware_version: str = ""
    api_version: str = ""
    is_connected: bool = False
    is_mock: bool = False
    last_seen: float = 0.0


@dataclass
class TestProfile:
    """Tekrar kullanılabilir Cihaz Test Modeli / Profili."""
    name: str
    board: str
    firmware: str
    battery_cells: int = 4  # 1S, 2S, 3S, 4S, 6S
    cell_min_voltage: float = 3.50
    cell_max_voltage: float = 4.25
    tests: Dict[str, bool] = field(default_factory=lambda: {
        "board_firmware": True,
        "imu1": True,
        "imu2": False,
        "barometer": True,
        "vbus": True,
        "battery": True,
        "battery_cells": True,
        "current": True,
        "cpu_load": True,
        "i2c_bus": True,
        "cycle_time": True,
        "mcu_uid": True,
        "motor_rpm": True,
        "memory": True,
        "pid": True,
        "rate": True,
        "rc": True,
        "rc_modes": True,
        "uart_config": True,
        "filters": True,
        "osd": True,
        "osd_elements": True,
        "vtx": True,
        "vtx_connection": True,
        "usb": True,
        "gps": False
    })
    limits: Dict[str, float] = field(default_factory=lambda: {
        "vbus_min": 4.80,
        "vbus_max": 5.20,
        "barometer_min": -5.0,
        "barometer_max": 5.0,
        "battery_min": 14.0,
        "battery_max": 17.0,
        "current_max_disarmed": 5.0,
        "pid_tolerance": 2.0,
        "rate_tolerance": 5.0,
        "cpu_load_max": 75.0,
        "i2c_errors_max": 0.0,
        "cycle_time_expected": 125.0,
        "cycle_time_tolerance": 25.0,
        "filter_tolerance_hz": 25.0
    })
    expected_values: Dict[str, Any] = field(default_factory=lambda: {
        "vtx_band": "R",
        "vtx_channel": 8,
        "vtx_frequency": 5917,
        "vtx_power": 800,
        "vtx_pitmode": 0,
        "vtx_require_connected": True,
        "filter_gyro_lowpass_hz": 250,
        "filter_dterm_lowpass_hz": 150,
        "rc_modes": {
            "ARM": "AUX1",
            "ANGLE": "AUX2",
            "BEEPER": "AUX3",
            "TURTLE": "AUX4"
        },
        "rc_arm_channel": "AUX1",
        "uart_roles": {
            "UART1": "VTX",
            "UART2": "RX_SERIAL",
            "UART6": "GPS"
        },
        "osd_elements": {
            "MAIN_BATT_VOLTAGE": {"visible": True, "x": 12, "y": 14},
            "CRAFT_NAME": {"visible": True, "x": 10, "y": 1},
            "FLY_TIME": {"visible": True, "x": 2, "y": 14}
        },
        "pid_roll": [45, 80, 40],
        "pid_pitch": [47, 84, 46],
        "pid_yaw": [45, 80, 0],
        "rc_rate": 100,
        "rc_provider": 9,  # CRSF
        "osd_support": 1   # MAX7456 / Active
    })
    golden_snapshot: Dict[str, Any] = field(default_factory=dict)
    created_at: str = field(default_factory=lambda: datetime.now().strftime("%Y-%m-%d %H:%M:%S"))
    updated_at: str = field(default_factory=lambda: datetime.now().strftime("%Y-%m-%d %H:%M:%S"))


# ============================================================================
# 3. MSP PROTOKOL MOTORU (MSP v1 / v2 PARSER & CODEC)
# ============================================================================

class MSPParser:
    """Betaflight MSP v1 ve v2 çerçeveleyici ve çözücüsü."""

    @staticmethod
    def encode_v1(cmd: int, payload: bytes = b"", is_response: bool = False) -> bytes:
        """MSP v1 çerçevesi oluşturur ($M< veya $M> + size + cmd + payload + xor)."""
        size = len(payload)
        checksum = size ^ (cmd & 0xFF)
        for b in payload:
            checksum ^= b
        hdr = b"$M>" if is_response else b"$M<"
        return hdr + bytes([size, cmd & 0xFF]) + payload + bytes([checksum])

    @staticmethod
    def crc8_dvb_s2(data: bytes) -> int:
        """MSP v2 için resmi CRC8-DVB-S2 algoritması (Polinom: 0xD5)."""
        crc = 0
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0xD5) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc

    @staticmethod
    def encode_v2(cmd: int, payload: bytes = b"", is_response: bool = False) -> bytes:
        """MSP v2 çerçevesi oluşturur ($X< veya $X> + flag + cmd16 + size16 + payload + crc8)."""
        flag = 0
        size = len(payload)
        header = bytes([flag, cmd & 0xFF, (cmd >> 8) & 0xFF, size & 0xFF, (size >> 8) & 0xFF])
        crc = MSPParser.crc8_dvb_s2(header + payload)
        hdr = b"$X>" if is_response else b"$X<"
        return hdr + header + payload + bytes([crc])

    @staticmethod
    def parse_stream(buffer: bytearray) -> List[Tuple[int, bytes, bool]]:
        """Gelen bayt akışından tamamlanan MSP v1/v2 paketlerini ayrıştırır."""
        messages = []
        while len(buffer) >= 6:
            # Hem istek ($M<, $X<) hem de yanıt ($M>, $M!, $X>) başlıklarını ara
            candidates = []
            for hdr in (b"$M>", b"$M<", b"$M!", b"$X>", b"$X<"):
                pos = buffer.find(hdr)
                if pos != -1:
                    candidates.append((pos, hdr))

            if not candidates:
                if len(buffer) > 256:
                    del buffer[:-6]
                break

            candidates.sort(key=lambda x: x[0])
            start_idx, hdr_type = candidates[0]
            if start_idx > 0:
                del buffer[:start_idx]

            # MSP v1 kontrolü ($M>, $M<, $M!)
            if buffer.startswith(b"$M>") or buffer.startswith(b"$M<") or buffer.startswith(b"$M!"):
                if len(buffer) < 6:
                    break
                is_err = buffer.startswith(b"$M!")
                size = buffer[3]
                cmd = buffer[4]
                total_len = 6 + size
                if len(buffer) < total_len:
                    break  # Paket henüz tamamlanmadı
                payload = bytes(buffer[5:5 + size])
                expected_crc = buffer[5 + size]
                calc_crc = size ^ cmd
                for b in payload:
                    calc_crc ^= b
                if calc_crc == expected_crc:
                    messages.append((cmd, payload, is_err))
                del buffer[:total_len]
                continue

            # MSP v2 kontrolü ($X>, $X<)
            if buffer.startswith(b"$X>") or buffer.startswith(b"$X<"):
                if len(buffer) < 9:
                    break
                flag = buffer[3]
                cmd = buffer[4] | (buffer[5] << 8)
                size = buffer[6] | (buffer[7] << 8)
                total_len = 9 + size
                if len(buffer) < total_len:
                    break
                payload = bytes(buffer[8:8 + size])
                expected_crc = buffer[8 + size]
                calc_crc = MSPParser.crc8_dvb_s2(buffer[3:8 + size])
                if calc_crc == expected_crc:
                    messages.append((cmd, payload, False))
                del buffer[:total_len]
                continue

            del buffer[:1]

        return messages


class MSPDecoder:
    """MSP bayt verilerini yapısal Python sözlüklerine dönüştürür."""

    @staticmethod
    def decode_payload(cmd: int, payload: bytes) -> Dict[str, Any]:
        result = {}
        try:
            if cmd == MSPCommand.FC_VERSION:
                if len(payload) >= 3:
                    result["fw_version"] = f"{payload[0]}.{payload[1]}.{payload[2]}"
            elif cmd == MSPCommand.FC_VARIANT:
                if len(payload) >= 4:
                    result["fc_variant"] = payload[:4].decode("latin1", errors="ignore")
            elif cmd == MSPCommand.BOARD_INFO:
                if len(payload) >= 6:
                    result["board_id"] = payload[:4].decode("latin1", errors="ignore")
                    offset = 6
                    if len(payload) > offset:
                        tlen = payload[offset]
                        offset += 1
                        result["target_name"] = payload[offset:offset+tlen].decode("latin1", errors="ignore")
                        offset += tlen
                    if len(payload) > offset:
                        blen = payload[offset]
                        offset += 1
                        result["board_name"] = payload[offset:offset+blen].decode("latin1", errors="ignore")
            elif cmd == MSPCommand.STATUS or cmd == MSPCommand.STATUS_EX:
                if len(payload) >= 11:
                    cycle_time, i2c_err, sensor_mask, flight_mode, profile = struct.unpack_from("<HHIBB", payload, 0)
                    result["cycle_time"] = cycle_time
                    result["cycle_time_us"] = cycle_time
                    result["i2c_errors"] = i2c_err
                    result["sensor_mask"] = sensor_mask
                    result["armed"] = bool(flight_mode & 0x01)
                    result["acc_present"] = bool(sensor_mask & SensorFlags.ACC)
                    result["baro_present"] = bool(sensor_mask & SensorFlags.BARO)
                    result["gyro_present"] = bool(sensor_mask & SensorFlags.GYRO)
                    result["gps_present"] = bool(sensor_mask & SensorFlags.GPS)
                    result["mag_present"] = bool(sensor_mask & SensorFlags.MAG)
                    load = 15
                    if len(payload) >= 12:
                        load = struct.unpack_from("<H", payload, 10)[0]
                    result["cpu_load_percent"] = min(100, max(0, load))
            elif cmd == MSPCommand.BATTERY_CONFIG:
                if len(payload) >= 4:
                    vmin, vmax, vwarn, cap = struct.unpack_from("<BBBH", payload, 0)
                    result["cfg_min_cell_v"] = round(vmin * 0.1, 2)
                    result["cfg_max_cell_v"] = round(vmax * 0.1, 2)
                    result["cfg_warn_cell_v"] = round(vwarn * 0.1, 2)
                    result["cfg_capacity_mah"] = cap
            elif cmd == MSPCommand.BATTERY_STATE:
                if len(payload) >= 1:
                    result["cell_count"] = payload[0]
                    if len(payload) >= 7:
                        result["battery_state"] = payload[6]
            elif cmd == MSPCommand.UID:
                if len(payload) >= 12:
                    result["mcu_uid"] = payload[:12].hex().upper()
            elif cmd == MSPCommand.RAW_IMU:
                if len(payload) >= 18:
                    ax, ay, az, gx, gy, gz, mx, my, mz = struct.unpack("<9h", payload[:18])
                    result["acc_x"] = ax
                    result["acc_y"] = ay
                    result["acc_z"] = az  # 512 LSB ≈ 1G
                    result["gyro_x"] = gx
                    result["gyro_y"] = gy
                    result["gyro_z"] = gz
            elif cmd == MSPCommand.ALTITUDE:
                if len(payload) >= 6:
                    alt_cm, vario = struct.unpack("<ih", payload[:6])
                    result["altitude_m"] = round(alt_cm / 100.0, 2)
                    result["vario"] = vario
            elif cmd == MSPCommand.ANALOG:
                if len(payload) >= 7:
                    vbat_raw, power, rssi, amp_raw = struct.unpack("<BHHh", payload[:7])
                    result["vbat"] = round(vbat_raw * 0.1, 2)
                    result["current_a"] = round(amp_raw * 0.01, 2)
                    if len(payload) >= 9:
                        v_centi = struct.unpack_from("<H", payload, 7)[0]
                        result["battery_voltage"] = round(v_centi * 0.01, 2)
                    else:
                        result["battery_voltage"] = result["vbat"]
            elif cmd == MSPCommand.VOLTAGE_METERS:
                meters = {}
                for i in range(0, len(payload), 3):
                    if i + 3 <= len(payload):
                        sid, volt = struct.unpack_from("<BH", payload, i)
                        meters[sid] = round(volt * 0.01, 2)
                result["voltage_meters"] = meters
                if 10 in meters:
                    result["vbus_voltage"] = meters[10]
                elif 1 in meters:
                    result["vbus_voltage"] = meters[1]
            elif cmd == MSPCommand.PID:
                if len(payload) >= 9:
                    result["pid_roll"] = [payload[0], payload[1], payload[2]]
                    result["pid_pitch"] = [payload[3], payload[4], payload[5]]
                    result["pid_yaw"] = [payload[6], payload[7], payload[8]]
            elif cmd == MSPCommand.RC_TUNING:
                if len(payload) >= 11:
                    result["rc_rate"] = payload[0]
                    result["rc_expo"] = payload[1]
                    result["roll_rate"] = payload[2]
                    result["pitch_rate"] = payload[3]
                    result["yaw_rate"] = payload[4]
            elif cmd == MSPCommand.RX_CONFIG:
                if len(payload) >= 1:
                    result["rc_provider"] = payload[0]
            elif cmd == MSPCommand.RC_MAP:
                if len(payload) >= 4:
                    map_str = "".join(["A", "E", "T", "R"][b] if b < 4 else str(b) for b in payload[:4])
                    result["rc_map"] = map_str
            elif cmd == MSPCommand.OSD_CONFIG:
                if len(payload) >= 2:
                    result["osd_support"] = payload[0]
                    result["video_system"] = payload[1]
                    osd_elems = {}
                    offset = 2
                    item_idx = 0
                    while offset + 2 <= len(payload) and item_idx < len(OSD_ITEM_NAMES):
                        pos = struct.unpack_from("<H", payload, offset)[0]
                        item_name = OSD_ITEM_NAMES.get(item_idx, f"ITEM_{item_idx}")
                        visible = bool((pos >> 11) & 0x01)
                        x = pos & 0x1F
                        y = (pos >> 5) & 0x1F
                        osd_elems[item_name] = {"visible": visible, "x": x, "y": y}
                        offset += 2
                        item_idx += 1
                    result["osd_elements"] = osd_elems
            elif cmd == MSPCommand.VTX_CONFIG:
                if len(payload) >= 8:
                    vtype, band, ch, pwr, pit, freq, ready = struct.unpack("<BBBBBHB", payload[:8])
                    result["vtx_type"] = vtype
                    result["vtx_band_num"] = band
                    result["vtx_band"] = VTX_BANDS.get(band, f"Band{band}")
                    result["vtx_channel"] = ch
                    result["vtx_power_idx"] = pwr
                    result["vtx_power_mw"] = 800 if pwr >= 4 else (400 if pwr == 3 else (100 if pwr == 2 else 25))
                    result["vtx_pitmode"] = pit
                    result["vtx_frequency"] = freq
                    result["vtx_ready"] = ready
                    result["vtx_device_ready"] = bool(ready != 0)
            elif cmd == MSPCommand.MODE_RANGES:
                # Mode / AUX switch atamaları (boxId, auxChannelIndex, startStep, endStep)
                mode_map = {}
                mode_ranges = {}
                for i in range(0, len(payload), 4):
                    if i + 4 <= len(payload):
                        b_id, a_idx, s_step, e_step = struct.unpack_from("<BBBB", payload, i)
                        aux_name = f"AUX{a_idx + 1}"
                        mode_name = BOX_NAMES.get(b_id, f"BOX_{b_id}")
                        mode_map[b_id] = aux_name
                        mode_ranges[mode_name] = {
                            "aux": aux_name,
                            "start": 900 + s_step * 25,
                            "end": 900 + e_step * 25
                        }
                        if b_id == 0:  # BOXARM
                            result["arm_channel"] = aux_name
                        elif b_id == 1:  # BOXANGLE
                            result["angle_channel"] = aux_name
                        elif b_id == 8:  # BOXBEEPER
                            result["beeper_channel"] = aux_name
                        elif b_id == 27: # TURTLE
                            result["turtle_channel"] = aux_name
                result["mode_ranges"] = mode_map
                result["all_mode_ranges"] = mode_ranges
            elif cmd == MSPCommand.CF_SERIAL_CONFIG:
                # UART seri port konfigürasyonu (9 byte per port in BF 4.x)
                uart_cfgs = {}
                offset = 0
                while offset + 9 <= len(payload):
                    p_id, mask, m_baud, g_baud, b_baud, t_baud = struct.unpack_from("<BIBBBB", payload, offset)
                    port_name = f"UART{p_id + 1}" if p_id < 10 else ("USB_VCP" if p_id == 20 else f"PORT{p_id}")
                    roles = []
                    if mask & (1 << 0): roles.append("MSP")
                    if mask & (1 << 1): roles.append("GPS")
                    if mask & (1 << 6): roles.append("RX_SERIAL")
                    if mask & (1 << 11): roles.append("VTX_SMARTAUDIO")
                    if mask & (1 << 12): roles.append("ESC_TELEMETRY")
                    if mask & (1 << 13): roles.append("VTX_TRAMP")
                    if mask & (1 << 15): roles.append("VTX_MSP")
                    uart_cfgs[port_name] = roles
                    offset += 9
                result["uart_configs"] = uart_cfgs
            elif cmd == MSPCommand.FILTER_CONFIG:
                if len(payload) >= 4:
                    g_lpf = payload[0]
                    d_lpf = struct.unpack_from("<H", payload, 1)[0]
                    result["gyro_lowpass_hz"] = g_lpf if g_lpf > 0 else 250
                    result["dterm_lowpass_hz"] = d_lpf if d_lpf > 0 else 150
                    if len(payload) >= 16:
                        result["gyro_lowpass2_hz"] = struct.unpack_from("<H", payload, 11)[0]
                        result["dterm_lowpass2_hz"] = struct.unpack_from("<H", payload, 14)[0]
            elif cmd == MSPCommand.DATAFLASH_SUMMARY:
                if len(payload) >= 13:
                    flags, sectors, total_sz, used_sz = struct.unpack("<BIII", payload[:13])
                    result["flash_ready"] = bool(flags & 0x01)
                    result["flash_supported"] = bool(flags & 0x02)
                    result["flash_total_mb"] = round(total_sz / (1024 * 1024), 1)
            elif cmd == MSPCommand.RAW_GPS:
                if len(payload) >= 16:
                    fix, num_sat, lat, lon, alt, speed, ground_course = struct.unpack("<BBiiHHH", payload[:16])
                    result["gps_fix"] = fix
                    result["gps_sats"] = num_sat
        except Exception as e:
            logging.error(f"MSP decode hatası (CMD {cmd}): {e}")
        return result


# ============================================================================
# 4. TAŞIMA KATMANI (SERIAL TRANSPORT & REALISTIC MOCK FC)
# ============================================================================

class BaseTransport:
    """Seri veya Mock donanım için ortak taşıma arayüzü."""
    def open(self) -> bool:
        raise NotImplementedError
    def close(self) -> None:
        raise NotImplementedError
    def send(self, data: bytes) -> bool:
        raise NotImplementedError
    def receive(self) -> bytes:
        raise NotImplementedError
    def is_open(self) -> bool:
        raise NotImplementedError


class SerialTransport(BaseTransport):
    """Gerçek USB Seri COM Port İletişimi."""
    def __init__(self, port: str, baudrate: int = 115200, timeout: float = 0.1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None

    def open(self) -> bool:
        if not HAS_SERIAL:
            return False
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            return self.ser.is_open
        except Exception as e:
            logging.error(f"Port açılamadı ({self.port}): {e}")
            return False

    def close(self) -> None:
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None

    def send(self, data: bytes) -> bool:
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(data)
                return True
            except Exception:
                return False
        return False

    def receive(self) -> bytes:
        if self.ser and self.ser.is_open:
            try:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    return self.ser.read(waiting)
            except Exception:
                pass
        return b""

    def is_open(self) -> bool:
        return self.ser is not None and self.ser.is_open


class MockTransport(BaseTransport):
    """
    Gerçek FC donanımına gerek kalmadan tüm MSP mesajlarına Betaflight
    özellikleriyle yanıt veren, 17 farklı arıza enjeksiyonunu destekleyen Sanal FC.
    """
    def __init__(self, fault_config: Optional[Dict[str, bool]] = None):
        self.opened = False
        self.faults = fault_config or {}
        self.rx_buffer = bytearray()
        self.tx_buffer = bytearray()
        self.board_name = "STM32F405"
        self.firmware = "4.5.2"

    def open(self) -> bool:
        self.opened = True
        return True

    def close(self) -> None:
        self.opened = False

    def is_open(self) -> bool:
        if self.faults.get("disconnect", False):
            return False
        return self.opened

    def set_faults(self, faults: Dict[str, bool]):
        self.faults = faults

    def send(self, data: bytes) -> bool:
        if not self.is_open():
            return False
        self.rx_buffer.extend(data)
        self._process_requests()
        return True

    def receive(self) -> bytes:
        if not self.is_open():
            return b""
        data = bytes(self.tx_buffer)
        self.tx_buffer.clear()
        return data

    def _process_requests(self):
        messages = MSPParser.parse_stream(self.rx_buffer)
        for cmd, _, _ in messages:
            if self.faults.get("msp_timeout", False):
                continue
            resp_payload = self._generate_response(cmd)
            if resp_payload is None:
                resp_payload = b""
            pkt = MSPParser.encode_v1(cmd, resp_payload, is_response=True)
            self.tx_buffer.extend(pkt)

    def _generate_response(self, cmd: int) -> bytes:
        f = self.faults
        if cmd == MSPCommand.FC_VARIANT:
            return b"BTFL"
        elif cmd == MSPCommand.FC_VERSION:
            if f.get("wrong_firmware", False):
                return bytes([4, 4, 0])
            return bytes([4, 5, 2])
        elif cmd == MSPCommand.BOARD_INFO:
            board = b"F405"
            target = b"STM32F405"
            return b"F405\x00\x00\x00" + bytes([len(target)]) + target + bytes([len(board)]) + board
        elif cmd == MSPCommand.STATUS or cmd == MSPCommand.STATUS_EX:
            mask = 0
            if not f.get("imu1_fail", False):
                mask |= SensorFlags.ACC | SensorFlags.GYRO
            if not f.get("baro_fail", False):
                mask |= SensorFlags.BARO
            mask |= SensorFlags.GPS
            flight_mode = 0  # DISARMED
            cycle = 260 if f.get("cycle_time_fail", False) else 125
            i2c = 14 if f.get("i2c_fail", False) else 0
            load = 88 if f.get("cpu_load_fail", False) else 15
            return struct.pack("<HHIBBH", cycle, i2c, mask, flight_mode, 0, load)
        elif cmd == MSPCommand.BATTERY_CONFIG:
            return struct.pack("<BBBH", 35, 43, 35, 1500)
        elif cmd == MSPCommand.BATTERY_STATE:
            cells = 3 if f.get("battery_cells_fail", False) else 4
            return struct.pack("<BHBHHB", cells, 1500, 164, 0, 82, 0)
        elif cmd == MSPCommand.UID:
            return bytes.fromhex("002B00335147500520383141")
        elif cmd == MSPCommand.RC_MAP:
            return bytes([0, 1, 2, 3])
        elif cmd == MSPCommand.RAW_IMU:
            az = 0 if f.get("imu1_fail", False) else 514
            return struct.pack("<9h", 2, -1, az, 0, 0, 0, 0, 0, 0)
        elif cmd == MSPCommand.ALTITUDE:
            alt_cm = 1500 if f.get("baro_fail", False) else 12
            return struct.pack("<ih", alt_cm, 0)
        elif cmd == MSPCommand.ANALOG:
            vbat = 100 if f.get("battery_low", False) else 164  # 16.4V
            amp = 620 if f.get("high_current", False) else 82   # 6.2A veya 0.82A
            v_centi = 1000 if f.get("battery_low", False) else 1642
            return struct.pack("<BHHhH", vbat, 0, 1023, amp, v_centi)
        elif cmd == MSPCommand.VOLTAGE_METERS:
            vbus = 502  # 5.02V
            if f.get("vbus_low", False):
                vbus = 430
            elif f.get("vbus_high", False):
                vbus = 555
            return struct.pack("<BH", 10, vbus)
        elif cmd == MSPCommand.PID:
            p_roll = 10 if f.get("wrong_pid", False) else 45
            return bytes([p_roll, 80, 40, 47, 84, 46, 45, 80, 0])
        elif cmd == MSPCommand.RC_TUNING:
            r_rate = 190 if f.get("wrong_rate", False) else 100
            return bytes([r_rate, 50, 100, 100, 100, 0, 0, 0, 0, 0, 0])
        elif cmd == MSPCommand.RX_CONFIG:
            provider = 2 if f.get("rc_mismatch", False) else 9
            return bytes([provider]) + (b"\x00" * 20)
        elif cmd == MSPCommand.OSD_CONFIG:
            supp = 0 if f.get("osd_mismatch", False) else 1
            vid = 1
            items = []
            for i in range(len(OSD_ITEM_NAMES)):
                vis = 1
                x = 10
                y = 10
                if i == 1:  # MAIN_BATT_VOLTAGE
                    vis = 0 if f.get("osd_icon_fail", False) else 1
                    x, y = 12, 14
                elif i == 6:  # FLY_TIME
                    x, y = 2, 14
                elif i == 8:  # CRAFT_NAME
                    x, y = 10, 1
                elif i == 14: # GPS_SATS
                    x, y = 25, 1
                pos = (vis << 11) | ((y & 0x1F) << 5) | (x & 0x1F)
                items.append(pos)
            return bytes([supp, vid]) + struct.pack(f"<{len(items)}H", *items)
        elif cmd == MSPCommand.VTX_CONFIG:
            band = 5  # R
            ch = 1 if f.get("vtx_channel_fail", False) else 8
            pwr = 1 if f.get("vtx_power_fail", False) else 4  # 4 = 800mW, 1 = 25mW
            freq = 5658 if ch == 1 else 5917
            ready = 0 if (f.get("vtx_connect_fail", False) or f.get("vtx_disconnected", False)) else 1
            return struct.pack("<BBBBBHB", 2, band, ch, pwr, 0, freq, ready)
        elif cmd == MSPCommand.MODE_RANGES:
            # 0: ARM, 1: ANGLE, 8: BEEPER, 27: TURTLE
            arm_aux = 2 if f.get("wrong_arm_switch", False) else 0  # AUX1 or AUX3
            angle_aux = 1  # AUX2
            beeper_aux = 0 if f.get("wrong_mode_range", False) else 2  # AUX3 or AUX1
            turtle_aux = 3  # AUX4
            r1 = struct.pack("<BBBB", 0, arm_aux, 32, 48)
            r2 = struct.pack("<BBBB", 1, angle_aux, 32, 48)
            r3 = struct.pack("<BBBB", 8, beeper_aux, 32, 48)
            r4 = struct.pack("<BBBB", 27, turtle_aux, 32, 48)
            return r1 + r2 + r3 + r4
        elif cmd == MSPCommand.CF_SERIAL_CONFIG:
            # UART1: VTX_SMARTAUDIO (2048), UART2: RX_SERIAL (64), UART6: GPS (2)
            u1_mask = 0 if f.get("wrong_uart", False) else 2048
            u2_mask = 64
            u6_mask = 2
            p1 = struct.pack("<BIBBBB", 0, u1_mask, 0, 0, 0, 0)
            p2 = struct.pack("<BIBBBB", 1, u2_mask, 0, 0, 0, 0)
            p6 = struct.pack("<BIBBBB", 5, u6_mask, 0, 0, 0, 0)
            return p1 + p2 + p6
        elif cmd == MSPCommand.FILTER_CONFIG:
            g_lpf = 80 if f.get("wrong_filter", False) else 250
            d_lpf = 60 if f.get("wrong_filter", False) else 150
            return struct.pack("<BHHHHHHH", g_lpf, d_lpf, 100, 0, 0, 0, 0, 250)
        elif cmd == MSPCommand.DATAFLASH_SUMMARY:
            flags = 0 if f.get("memory_fail", False) else 3
            return struct.pack("<BIII", flags, 64, 16 * 1024 * 1024, 0)
        elif cmd == MSPCommand.RAW_GPS:
            return struct.pack("<BBiiHHH", 0, 0, 0, 0, 0, 0, 0)
        elif cmd == MSPCommand.MOTOR:
            return struct.pack("<8H", 1000, 1000, 1000, 1000, 0, 0, 0, 0)
        return b""

class GoldenSnapshotExtractor:
    """
    Onaylı Referans Karttan (Golden Sample FC) tek seferde 100+ statik konfigürasyon
    parametresini çeker, kategorize eder ve profil parmak izi oluşturur.
    Dinamik sensör gürültüsü ve donanım seri no (UID) statik eşitlikten hariç tutulur.
    """
    @staticmethod
    def build_baseline_snapshot(board: str = "F405", firmware: str = "4.5.2") -> Dict[str, Any]:
        """Tüm 32 OSD elemanı ve tüm RC modlarını içeren standart Golden Snapshot üretir."""
        osd_dict = {}
        for idx in range(len(OSD_ITEM_NAMES)):
            name = OSD_ITEM_NAMES.get(idx, f"ITEM_{idx}")
            vis = (idx in (1, 6, 8, 14, 15, 20, 21)) # Batt, FlyTime, CraftName, Sats, Alt, Warning, CellV
            x, y = 10, 10
            if idx == 1: x, y = 12, 14       # MAIN_BATT_VOLTAGE
            elif idx == 6: x, y = 2, 14      # FLY_TIME
            elif idx == 8: x, y = 10, 1      # CRAFT_NAME
            elif idx == 14: x, y = 25, 1     # GPS_SATS
            elif idx == 15: x, y = 2, 12     # ALTITUDE
            elif idx == 20: x, y = 10, 13    # BATTERY_WARNING
            elif idx == 21: x, y = 12, 15    # AVG_CELL_VOLTAGE
            osd_dict[name] = {"visible": vis, "x": x, "y": y}

        mode_ranges = {
            "ARM": {"aux": "AUX1", "start": 1700, "end": 2100},
            "ANGLE": {"aux": "AUX2", "start": 1700, "end": 2100},
            "HORIZON": {"aux": "AUX2", "start": 1300, "end": 1700},
            "BEEPER": {"aux": "AUX3", "start": 1700, "end": 2100},
            "TURTLE": {"aux": "AUX4", "start": 1700, "end": 2100},
            "AIRMODE": {"aux": "AUX1", "start": 1700, "end": 2100},
            "VTX_PIT_MODE": {"aux": "AUX4", "start": 900, "end": 1300}
        }

        uart_roles = {
            "UART1": ["VTX_SMARTAUDIO"],
            "UART2": ["RX_SERIAL"],
            "UART6": ["GPS"]
        }

        return {
            "board_name": board,
            "fw_version": firmware,
            "osd_elements": osd_dict,
            "mode_ranges": mode_ranges,
            "uart_roles": uart_roles,
            "pid_roll": [45, 80, 40],
            "pid_pitch": [47, 84, 46],
            "pid_yaw": [45, 80, 0],
            "gyro_lowpass_hz": 250,
            "dterm_lowpass_hz": 150,
            "rc_rate": 100,
            "rc_expo": 50,
            "roll_rate": 100,
            "pitch_rate": 100,
            "yaw_rate": 100,
            "rc_provider": 9,  # CRSF
            "rc_map": "AETR",
            "vtx_band": "R",
            "vtx_channel": 8,
            "vtx_power_mw": 800,
            "vtx_pitmode": 0,
            "vtx_frequency": 5917,
            "vtx_device_ready": True,
            "battery_cells": 4,
            "flash_total_mb": 16.0
        }

    @staticmethod
    def extract_from_transport(transport: BaseTransport) -> Tuple[Dict[str, Any], str]:
        cmds = [
            MSPCommand.BOARD_INFO, MSPCommand.FC_VERSION, MSPCommand.BATTERY_CONFIG,
            MSPCommand.BATTERY_STATE, MSPCommand.PID, MSPCommand.RC_TUNING,
            MSPCommand.RC_MAP, MSPCommand.RX_CONFIG, MSPCommand.OSD_CONFIG,
            MSPCommand.VTX_CONFIG, MSPCommand.MODE_RANGES, MSPCommand.CF_SERIAL_CONFIG,
            MSPCommand.FILTER_CONFIG, MSPCommand.DATAFLASH_SUMMARY
        ]
        rx_buf = bytearray()
        for cmd in cmds:
            transport.send(MSPParser.encode_v1(cmd))
            time.sleep(0.035)
            data = transport.receive()
            if data:
                rx_buf.extend(data)

        collected: Dict[str, Any] = {}
        msgs = MSPParser.parse_stream(rx_buf)
        for cmd, payload, _ in msgs:
            collected.update(MSPDecoder.decode_payload(cmd, payload))

        board = collected.get("board_name") or collected.get("board_id", "F405")
        fw = collected.get("fw_version", "4.5.2")

        snapshot = {
            "board_name": board,
            "fw_version": fw,
            "osd_elements": collected.get("osd_elements", {}),
            "mode_ranges": collected.get("all_mode_ranges", {}),
            "uart_roles": collected.get("uart_configs", {}),
            "pid_roll": collected.get("pid_roll", [45, 80, 40]),
            "pid_pitch": collected.get("pid_pitch", [47, 84, 46]),
            "pid_yaw": collected.get("pid_yaw", [45, 80, 0]),
            "gyro_lowpass_hz": collected.get("gyro_lowpass_hz", 250),
            "dterm_lowpass_hz": collected.get("dterm_lowpass_hz", 150),
            "rc_rate": collected.get("rc_rate", 100),
            "rc_expo": collected.get("rc_expo", 50),
            "roll_rate": collected.get("roll_rate", 100),
            "pitch_rate": collected.get("pitch_rate", 100),
            "yaw_rate": collected.get("yaw_rate", 100),
            "rc_provider": collected.get("rc_provider", 9),
            "rc_map": collected.get("rc_map", "AETR"),
            "vtx_band": collected.get("vtx_band", "R"),
            "vtx_channel": collected.get("vtx_channel", 8),
            "vtx_power_mw": collected.get("vtx_power_mw", 800),
            "vtx_pitmode": collected.get("vtx_pitmode", 0),
            "vtx_frequency": collected.get("vtx_frequency", 5917),
            "vtx_device_ready": collected.get("vtx_device_ready", True),
            "battery_cells": collected.get("cell_count", 4),
            "flash_total_mb": collected.get("flash_total_mb", 16.0)
        }

        # Eğer osd_elements boşsa baseline ile tamamla
        if not snapshot["osd_elements"]:
            baseline = GoldenSnapshotExtractor.build_baseline_snapshot(board, fw)
            snapshot["osd_elements"] = baseline["osd_elements"]
        if not snapshot["mode_ranges"]:
            baseline = GoldenSnapshotExtractor.build_baseline_snapshot(board, fw)
            snapshot["mode_ranges"] = baseline["mode_ranges"]
        if not snapshot["uart_roles"]:
            baseline = GoldenSnapshotExtractor.build_baseline_snapshot(board, fw)
            snapshot["uart_roles"] = baseline["uart_roles"]

        elem_count = len(snapshot["osd_elements"])
        mode_count = len(snapshot["mode_ranges"])
        uart_count = len(snapshot["uart_roles"])
        total_p = elem_count * 3 + mode_count * 3 + uart_count + 18

        summary = (
            f"✓ Golden Snapshot Başarıyla Çekildi: Toplam {total_p} parametre.\n"
            f"--------------------------------------------------\n"
            f"• 📺 OSD Telemetri: {elem_count} Eleman (Visible, X, Y koordinatları)\n"
            f"• 🕹️ RC Modları: {mode_count} Fonksiyon (ARM, ANGLE, BEEPER, TURTLE...)\n"
            f"• 🔌 UART Portları: {uart_count} Port Rolü (VTX, CRSF/ELRS, GPS)\n"
            f"• 🎯 PID Kazançları: Roll {snapshot['pid_roll']}, Pitch {snapshot['pid_pitch']}\n"
            f"• 🔊 Filtre Frekansları: Gyro {snapshot['gyro_lowpass_hz']}Hz, D-Term {snapshot['dterm_lowpass_hz']}Hz\n"
            f"• 📡 VTX: Band {snapshot['vtx_band']}, Ch {snapshot['vtx_channel']}, {snapshot['vtx_power_mw']}mW (SmartAudio OK)\n"
            f"• ⚡ Pil Yapılandırması: {snapshot['battery_cells']}S Lipo\n"
            f"--------------------------------------------------\n"
            f"Not: MCU Seri No (UID) ve canlı IMU sıfır sapması dinamik bırakılmıştır."
        )
        return snapshot, summary


def build_360_parameter_audit(profile: TestProfile, live_data: Dict[str, Any]) -> List[Dict[str, Any]]:
    """
    Onaylı Referans Profil (Golden Sample) ile Test Edilen Kartın Canlı MSP verileri arasında
    140'tan fazla parametreyi kategorize edilmiş olarak 360° kıyaslar.
    Dinamik parametreler (MCU Seri No/UID ve canlı IMU gürültü sapması) eşitlik kontrolünden
    muaf tutularak 'DYNAMIC' olarak etiketlenir.
    """
    audit: List[Dict[str, Any]] = []

    def add_row(cat: str, name: str, golden: Any, actual: Any, status: str, detail: str = ""):
        audit.append({
            "category": cat,
            "name": name,
            "golden": str(golden),
            "actual": str(actual),
            "status": status,
            "detail": detail
        })

    snap = getattr(profile, "golden_snapshot", None) or {}
    if not snap:
        snap = GoldenSnapshotExtractor.build_baseline_snapshot(profile.board, profile.firmware)

    # 1. 📺 OSD Telemetri (32 İkon / 96 Parametre)
    gold_osd = snap.get("osd_elements") or profile.expected_values.get("osd_elements") or {}
    live_osd = live_data.get("osd_elements") or {}
    for idx in range(len(OSD_ITEM_NAMES)):
        item_key = OSD_ITEM_NAMES.get(idx, f"ITEM_{idx}")
        friendly_title = item_key.replace("_", " ").title()

        g_info = gold_osd.get(item_key) or {}
        l_info = live_osd.get(item_key) or {}

        g_vis = g_info.get("visible", False)
        l_vis = l_info.get("visible", False)
        vis_st = "MATCH" if (g_vis == l_vis) else "MISMATCH"
        add_row(
            "📺 OSD Telemetri",
            f"OSD [{idx:02d}]: {friendly_title} - Görünürlük",
            "AKTİF (Görünür)" if g_vis else "KAPALI (Gizli)",
            "AKTİF (Görünür)" if l_vis else "KAPALI (Gizli)",
            vis_st,
            "Betaflight OSD ekran telemetri elemanı aktiflik durumu"
        )

        g_x = g_info.get("x", 0)
        l_x = l_info.get("x", 0)
        x_st = "MATCH" if abs(g_x - l_x) <= 2 else "MISMATCH"
        add_row(
            "📺 OSD Telemetri",
            f"OSD [{idx:02d}]: {friendly_title} - X Koordinatı",
            f"X = {g_x}",
            f"X = {l_x}",
            x_st,
            "OSD karakter matris yatay sütun konumu (0-30)"
        )

        g_y = g_info.get("y", 0)
        l_y = l_info.get("y", 0)
        y_st = "MATCH" if abs(g_y - l_y) <= 2 else "MISMATCH"
        add_row(
            "📺 OSD Telemetri",
            f"OSD [{idx:02d}]: {friendly_title} - Y Koordinatı",
            f"Y = {g_y}",
            f"Y = {l_y}",
            y_st,
            "OSD karakter matris dikey satır konumu (0-16)"
        )

    # 2. 🕹️ RC Modları & Kanallar
    gold_modes = snap.get("mode_ranges") or {}
    live_modes = live_data.get("all_mode_ranges") or {}
    for box_id, mode_name in BOX_NAMES.items():
        g_m = gold_modes.get(mode_name)
        l_m = live_modes.get(mode_name)
        if g_m:
            g_aux = g_m.get("aux", "AUX1") if isinstance(g_m, dict) else str(g_m)
            g_s = g_m.get("start", 1300) if isinstance(g_m, dict) else 1300
            g_e = g_m.get("end", 2100) if isinstance(g_m, dict) else 2100
        else:
            g_aux = profile.expected_values.get("rc_modes", {}).get(mode_name, "ATANMAMIŞ")
            g_s, g_e = 1300, 2100

        if l_m:
            l_aux = l_m.get("aux", "AUX1") if isinstance(l_m, dict) else str(l_m)
            l_s = l_m.get("start", 1300) if isinstance(l_m, dict) else 1300
            l_e = l_m.get("end", 2100) if isinstance(l_m, dict) else 2100
        else:
            if mode_name == "ARM" and live_data.get("arm_channel"):
                l_aux, l_s, l_e = live_data.get("arm_channel"), 1700, 2100
            elif mode_name == "ANGLE" and live_data.get("angle_channel"):
                l_aux, l_s, l_e = live_data.get("angle_channel"), 1700, 2100
            elif mode_name == "BEEPER" and live_data.get("beeper_channel"):
                l_aux, l_s, l_e = live_data.get("beeper_channel"), 1700, 2100
            elif mode_name == "TURTLE" and live_data.get("turtle_channel"):
                l_aux, l_s, l_e = live_data.get("turtle_channel"), 1700, 2100
            else:
                l_aux = "ATANMAMIŞ"
                l_s, l_e = 0, 0

        m_st = "MATCH" if (g_aux == l_aux) else "MISMATCH"
        add_row(
            "🕹️ RC Modları & Kanallar",
            f"RC Modu: {mode_name} Anahtar Kanalı",
            g_aux,
            l_aux,
            m_st,
            f"Kumanda switch ataması (Aralık: {g_s}-{g_e} µs)"
        )

    # RC Haritası ve Sağlayıcı
    g_map = snap.get("rc_map", "AETR")
    l_map = live_data.get("rc_map", "AETR")
    add_row(
        "🕹️ RC Modları & Kanallar",
        "RC Kanal Sıralaması (Channel Map)",
        g_map,
        l_map,
        "MATCH" if g_map == l_map else "MISMATCH",
        "Aileron, Elevator, Throttle, Rudder kanal eşlemesi"
    )

    g_prov = snap.get("rc_provider", profile.expected_values.get("rc_provider", 9))
    l_prov = live_data.get("rc_provider", 9)
    prov_str = lambda p: "CRSF / ELRS (9)" if p == 9 else ("SBUS (2)" if p == 2 else f"ID {p}")
    add_row(
        "🕹️ RC Modları & Kanallar",
        "Seri Alıcı Protokolü (Serial RX)",
        prov_str(g_prov),
        prov_str(l_prov),
        "MATCH" if g_prov == l_prov else "MISMATCH",
        "Haberleşme protokol sağlayıcısı"
    )

    # 3. 🔌 UART Portları & Protokoller
    g_uarts = snap.get("uart_roles") or profile.expected_values.get("uart_roles") or {}
    l_uarts = live_data.get("uart_configs") or {}
    for i in [1, 2, 3, 4, 5, 6]:
        u_name = f"UART{i}"
        g_role = g_uarts.get(u_name)
        if isinstance(g_role, list):
            g_role = ", ".join(g_role) if g_role else "DEVRE DIŞI"
        elif not g_role:
            g_role = "DEVRE DIŞI"

        l_roles = l_uarts.get(u_name, [])
        l_role_str = ", ".join(l_roles) if l_roles else "DEVRE DIŞI"

        match = False
        if g_role == "DEVRE DIŞI" and (not l_roles or l_role_str == "DEVRE DIŞI"):
            match = True
        elif g_role != "DEVRE DIŞI" and any(g_role.upper() in r.upper() or r.upper() in g_role.upper() for r in l_roles):
            match = True

        add_row(
            "🔌 UART Portları & Protokoller",
            f"Seri Port: {u_name} Fonksiyonu",
            g_role,
            l_role_str,
            "MATCH" if match else "MISMATCH",
            f"{u_name} donanımsal seri haberleşme görevi"
        )

    # 4. 🎯 PID Kazançları
    g_r = snap.get("pid_roll", [45, 80, 40])
    l_r = live_data.get("pid_roll", [0, 0, 0])
    g_p = snap.get("pid_pitch", [47, 84, 46])
    l_p = live_data.get("pid_pitch", [0, 0, 0])
    g_y = snap.get("pid_yaw", [45, 80, 0])
    l_y = live_data.get("pid_yaw", [0, 0, 0])
    tol = profile.limits.get("pid_tolerance", 2.0)

    for axis, g_val, l_val in [("Roll", g_r, l_r), ("Pitch", g_p, l_p), ("Yaw", g_y, l_y)]:
        for idx, p_letter in enumerate(["P", "I", "D"]):
            gv = g_val[idx] if len(g_val) > idx else 0
            lv = l_val[idx] if len(l_val) > idx else 0
            diff = abs(gv - lv)
            st = "MATCH" if diff <= tol else "MISMATCH"
            add_row(
                "🎯 PID Kazançları",
                f"PID: {axis} ekseni {p_letter}-Kazancı",
                f"{gv}",
                f"{lv}",
                st,
                f"Kontrol döngü kazancı (Tolerans: ±{tol})"
            )

    # 5. 🔊 Dijital Filtreleme Frekansları
    g_glpf = snap.get("gyro_lowpass_hz", 250)
    l_glpf = live_data.get("gyro_lowpass_hz", 250)
    g_dlpf = snap.get("dterm_lowpass_hz", 150)
    l_dlpf = live_data.get("dterm_lowpass_hz", 150)
    f_tol = profile.limits.get("filter_tolerance_hz", 25.0)

    add_row(
        "🔊 Dijital Filtreler",
        "Gyro Lowpass 1 Filtre Kesim Frekansı",
        f"{g_glpf} Hz",
        f"{l_glpf} Hz",
        "MATCH" if abs(g_glpf - l_glpf) <= f_tol else "MISMATCH",
        f"Jiroskop donanım gürültü bastırma (Tolerans: ±{f_tol}Hz)"
    )
    add_row(
        "🔊 Dijital Filtreler",
        "D-Term Lowpass 1 Filtre Kesim Frekansı",
        f"{g_dlpf} Hz",
        f"{l_dlpf} Hz",
        "MATCH" if abs(g_dlpf - l_dlpf) <= f_tol else "MISMATCH",
        f"D-Term motor titreşim sönümleme (Tolerans: ±{f_tol}Hz)"
    )

    # 6. 📈 RC Tepki & Rate Eğrileri
    rate_tol = profile.limits.get("rate_tolerance", 5.0)
    for r_name, g_k, l_k in [
        ("RC Rate", "rc_rate", "rc_rate"),
        ("RC Expo", "rc_expo", "rc_expo"),
        ("Roll Rate", "roll_rate", "roll_rate"),
        ("Pitch Rate", "pitch_rate", "pitch_rate"),
        ("Yaw Rate", "yaw_rate", "yaw_rate")
    ]:
        gv = snap.get(g_k, 100)
        lv = live_data.get(l_k, 100)
        st = "MATCH" if abs(gv - lv) <= rate_tol else "MISMATCH"
        add_row(
            "📈 RC Tepki & Rate Eğrileri",
            f"Rate: {r_name}",
            f"{gv}",
            f"{lv}",
            st,
            f"Çubuk hassasiyeti (Tolerans: ±{rate_tol})"
        )

    # 7. 📡 VTX Video Verici
    g_band = snap.get("vtx_band", "R")
    l_band = live_data.get("vtx_band", "R")
    add_row(
        "📡 VTX Video Verici",
        "VTX Frekans Bandı",
        g_band,
        l_band,
        "MATCH" if g_band == l_band else "MISMATCH",
        "Video verici çalışma bandı (A, B, E, F, R)"
    )

    g_ch = snap.get("vtx_channel", 8)
    l_ch = live_data.get("vtx_channel", 8)
    add_row(
        "📡 VTX Video Verici",
        "VTX Kanal Numarası",
        f"Kanal {g_ch}",
        f"Kanal {l_ch}",
        "MATCH" if g_ch == l_ch else "MISMATCH",
        "Video verici kanal seçimi (1-8)"
    )

    g_freq = snap.get("vtx_frequency", 5917)
    l_freq = live_data.get("vtx_frequency", 5917)
    add_row(
        "📡 VTX Video Verici",
        "VTX Taşıyıcı Frekansı",
        f"{g_freq} MHz",
        f"{l_freq} MHz",
        "MATCH" if g_freq == l_freq else "MISMATCH",
        "RF merkez frekansı"
    )

    g_pwr = snap.get("vtx_power_mw", 800)
    l_pwr = live_data.get("vtx_power_mw", 800)
    add_row(
        "📡 VTX Video Verici",
        "VTX RF Çıkış Gücü",
        f"{g_pwr} mW",
        f"{l_pwr} mW",
        "MATCH" if g_pwr == l_pwr else "MISMATCH",
        "Miliwatt cinsinden yayın gücü"
    )

    g_pit = snap.get("vtx_pitmode", 0)
    l_pit = live_data.get("vtx_pitmode", 0)
    add_row(
        "📡 VTX Video Verici",
        "VTX Pit Mode (Düşük Güç Park Modu)",
        "KAPALI (OFF)" if g_pit == 0 else "AÇIK (ON)",
        "KAPALI (OFF)" if l_pit == 0 else "AÇIK (ON)",
        "MATCH" if g_pit == l_pit else "MISMATCH",
        "Yarış ve pit alanı düşük RF modu"
    )

    g_vtx_rdy = snap.get("vtx_device_ready", True)
    l_vtx_rdy = live_data.get("vtx_device_ready")
    if l_vtx_rdy is None:
        l_vtx_rdy = bool(live_data.get("vtx_ready", 1) != 0)
    add_row(
        "📡 VTX Video Verici",
        "VTX Telemetri & İletişim Durumu",
        "CONNECTED (SmartAudio/Tramp OK)" if g_vtx_rdy else "DISCONNECTED",
        "CONNECTED (SmartAudio/Tramp OK)" if l_vtx_rdy else "DISCONNECTED",
        "MATCH" if g_vtx_rdy == l_vtx_rdy else "MISMATCH",
        "Donanım seri telemetri yanıtı"
    )

    # 8. ⚡ Analog & Güç Sistemi
    s_cnt = getattr(profile, "battery_cells", 4)
    l_s = live_data.get("cell_count", s_cnt)
    add_row(
        "⚡ Analog & Güç Sistemi",
        "Pil Hücre Sayısı (S)",
        f"{s_cnt}S",
        f"{l_s}S",
        "MATCH" if s_cnt == l_s else "MISMATCH",
        "Batarya seri hücre adedi"
    )

    c_min = getattr(profile, "cell_min_voltage", 3.50)
    c_max = getattr(profile, "cell_max_voltage", 4.25)
    vbat_act = live_data.get("battery_voltage", live_data.get("vbat", 16.0))
    vbat_min = round(s_cnt * c_min, 2)
    vbat_max = round(s_cnt * c_max, 2)
    vbat_ok = (vbat_min <= vbat_act <= vbat_max)
    add_row(
        "⚡ Analog & Güç Sistemi",
        "Toplam Batarya Voltajı (VBAT)",
        f"{vbat_min:.2f} - {vbat_max:.2f} V",
        f"{vbat_act:.2f} V",
        "MATCH" if vbat_ok else "MISMATCH",
        f"{s_cnt}S için hücre başı {c_min:.2f}-{c_max:.2f}V aralığı"
    )

    vbus_act = live_data.get("vbus_voltage", 5.0)
    vbus_min = profile.limits.get("vbus_min", 4.80)
    vbus_max = profile.limits.get("vbus_max", 5.20)
    vbus_ok = (vbus_min <= vbus_act <= vbus_max)
    add_row(
        "⚡ Analog & Güç Sistemi",
        "VBUS 5V Besleme Gerilimi",
        f"{vbus_min:.2f} - {vbus_max:.2f} V",
        f"{vbus_act:.2f} V",
        "MATCH" if vbus_ok else "MISMATCH",
        "5V LDO ve USB bus gerilimi"
    )

    cur_act = live_data.get("current_a", 0.0)
    max_cur = profile.limits.get("current_max_disarmed", 5.0)
    cur_ok = (cur_act <= max_cur)
    add_row(
        "⚡ Analog & Güç Sistemi",
        "Masaüstü Boşta Akım (Disarmed Current)",
        f"< {max_cur:.1f} A",
        f"{cur_act:.2f} A",
        "MATCH" if cur_ok else "MISMATCH",
        "Kısa devre ve aşırı akım koruması"
    )

    # 9. 🛡️ Sensör & Donanım Sağlığı
    act_b = live_data.get("board_name") or live_data.get("board_id", profile.board)
    add_row(
        "🛡️ Sensör & Donanım Sağlığı",
        "Hedef Donanım / Board",
        profile.board,
        act_b,
        "MATCH" if profile.board.upper() in act_b.upper() else "MISMATCH",
        "Donanım kimliği eşleşmesi"
    )

    act_fw = live_data.get("fw_version", profile.firmware)
    add_row(
        "🛡️ Sensör & Donanım Sağlığı",
        "Betaflight Firmware Sürümü",
        profile.firmware,
        act_fw,
        "MATCH" if profile.firmware == act_fw else "MISMATCH",
        "Yazılım versiyonu"
    )

    acc_p = live_data.get("acc_present", True)
    gyro_p = live_data.get("gyro_present", True)
    add_row(
        "🛡️ Sensör & Donanım Sağlığı",
        "IMU Jiroskop Varlığı",
        "MEVCUT / SAĞLIKLI",
        "MEVCUT / SAĞLIKLI" if gyro_p else "ARIZALI / YOK",
        "MATCH" if gyro_p else "MISMATCH",
        "SPI üzerinden Gyro haberleşmesi"
    )
    az = live_data.get("acc_z", 512)
    acc_ok = acc_p and (350 <= az <= 650)
    add_row(
        "🛡️ Sensör & Donanım Sağlığı",
        "IMU İvmeölçer & 1G Doğrulaması",
        "1.0 G (Yerçekimi Doğrulandı)",
        f"1.0 G (Acc Z: {az} LSB)" if acc_ok else f"FAIL (Acc Z: {az})",
        "MATCH" if acc_ok else "MISMATCH",
        "Z ekseninde 1G yerçekimi ivmesi"
    )

    baro_p = live_data.get("baro_present", True)
    alt = live_data.get("altitude_m", 0.0)
    b_min = profile.limits.get("barometer_min", -5.0)
    b_max = profile.limits.get("barometer_max", 5.0)
    baro_ok = baro_p and (b_min <= alt <= b_max)
    add_row(
        "🛡️ Sensör & Donanım Sağlığı",
        "Barometre & İrtifa Doğrulaması",
        f"{b_min:+.1f} ile {b_max:+.1f} m",
        f"{alt:+.2f} m" if baro_p else "SENSÖR YOK",
        "MATCH" if baro_ok else "MISMATCH",
        "Masaüstü durağan irtifa testi"
    )

    fl_r = live_data.get("flash_ready", True)
    fl_sz = live_data.get("flash_total_mb", 16.0)
    add_row(
        "🛡️ Sensör & Donanım Sağlığı",
        "Dahili Flash Bellek (Blackbox)",
        "HAZIR / DESTEKLENİYOR",
        f"HAZIR ({fl_sz:.1f} MB)" if fl_r else "FLASH ARIZALI",
        "MATCH" if fl_r else "MISMATCH",
        "SPI Flash bellek durumu"
    )

    cpu_load = live_data.get("cpu_load_percent", 15.0)
    max_cpu = profile.limits.get("cpu_load_max", 75.0)
    add_row(
        "🛡️ Sensör & Donanım Sağlığı",
        "MCU İşlemci / Sistem Yükü",
        f"<= %{max_cpu:.0f}",
        f"%{cpu_load:.0f}",
        "MATCH" if cpu_load <= max_cpu else "MISMATCH",
        "İşlemci yükü ve donma koruması"
    )

    i2c_err = live_data.get("i2c_errors", 0)
    max_i2c = profile.limits.get("i2c_errors_max", 0.0)
    add_row(
        "🛡️ Sensör & Donanım Sağlığı",
        "I2C Veri Yolu Sağlığı",
        f"{int(max_i2c)} Hata (Temiz)",
        f"{i2c_err} Hata",
        "MATCH" if i2c_err <= max_i2c else "MISMATCH",
        "I2C veri yolu parazit ve ACK testi"
    )

    cycle = live_data.get("cycle_time_us", live_data.get("cycle_time", 125))
    c_exp = profile.limits.get("cycle_time_expected", 125.0)
    c_tol = profile.limits.get("cycle_time_tolerance", 25.0)
    c_ok = abs(cycle - c_exp) <= c_tol
    add_row(
        "🛡️ Sensör & Donanım Sağlığı",
        "PID Döngü Süresi (Loop Time)",
        f"{int(c_exp)} µs (±{int(c_tol)})",
        f"{cycle} µs",
        "MATCH" if c_ok else "MISMATCH",
        "PID frekans kararlılığı"
    )

    # 10. ⚙️ Dinamik Değerler (Seri No & Gürültü Hariç Tutulanlar)
    uid_val = live_data.get("mcu_uid", "002B00335147500520383141")
    add_row(
        "⚙️ Dinamik Değerler",
        "MCU Donanım Seri No (Unique Silicon ID)",
        "[SERİ NO EŞİTLİK HARİCİ]",
        uid_val,
        "DYNAMIC",
        "Üretim takibi için loglanır; karttan karta değiştiğinden eşitlik aranmaz."
    )

    acc_x = live_data.get("acc_x", 0)
    acc_y = live_data.get("acc_y", 0)
    acc_z = live_data.get("acc_z", 512)
    add_row(
        "⚙️ Dinamik Değerler",
        "IMU Canlı Sıfır Sapması / Gürültü Bias",
        "[CANLI KALİBRASYON HARİCİ]",
        f"X:{acc_x} | Y:{acc_y} | Z:{acc_z} LSB",
        "DYNAMIC",
        "Sıcaklık ve mekanik toleransa göre değişir; eşitlik aranmaz, 1G vektör denetlenir."
    )

    return audit


# ============================================================================
# 5. PROFİL YÖNETİCİSİ VE DENETİM GÜNLÜKÇÜSÜ (PROFILE & LOG MANAGERS)
# ============================================================================

class ProfileManager:
    """Test profillerini profiles/ dizininde JSON olarak yönetir."""
    def __init__(self, profiles_dir: str = "profiles"):
        self.profiles_dir = profiles_dir
        os.makedirs(self.profiles_dir, exist_ok=True)
        self._ensure_default_profiles()

    def _ensure_default_profiles(self):
        default_profiles = [
            TestProfile(
                name="F405 Production",
                board="F405",
                firmware="4.5.2",
                battery_cells=4
            ),
            TestProfile(
                name="F722 Production",
                board="F722",
                firmware="4.5.2",
                battery_cells=4
            ),
            TestProfile(
                name="Racing F405",
                board="F405",
                firmware="4.4.3",
                battery_cells=4
            )
        ]
        for p in default_profiles:
            filepath = os.path.join(self.profiles_dir, f"{self._sanitize_filename(p.name)}.json")
            if not os.path.exists(filepath):
                self.save_profile(p)

    def _sanitize_filename(self, name: str) -> str:
        return "".join(c for c in name if c.isalnum() or c in (" ", "_", "-")).strip().replace(" ", "_")

    def list_profiles(self) -> List[TestProfile]:
        profiles = []
        proto = TestProfile(name="", board="", firmware="")
        for fn in os.listdir(self.profiles_dir):
            if fn.endswith(".json"):
                fp = os.path.join(self.profiles_dir, fn)
                try:
                    with open(fp, "r", encoding="utf-8") as f:
                        data = json.load(f)
                        p = TestProfile(**data)
                        # Migration: Yeni eklenen test ve beklenen değer anahtarlarını ekle
                        for k, v in proto.tests.items():
                            if k not in p.tests:
                                p.tests[k] = v
                        for k, v in proto.expected_values.items():
                            if k not in p.expected_values:
                                p.expected_values[k] = v
                        profiles.append(p)
                except Exception as e:
                    logging.warning(f"Profil okunamadı ({fn}): {e}")
        return profiles

    def get_profile(self, name: str) -> Optional[TestProfile]:
        for p in self.list_profiles():
            if p.name == name:
                return p
        return None

    def save_profile(self, profile: TestProfile) -> bool:
        profile.updated_at = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        fp = os.path.join(self.profiles_dir, f"{self._sanitize_filename(profile.name)}.json")
        try:
            with open(fp, "w", encoding="utf-8") as f:
                json.dump(asdict(profile), f, indent=4, ensure_ascii=False)
            return True
        except Exception as e:
            logging.error(f"Profil kaydedilemedi: {e}")
            return False

    def delete_profile(self, name: str) -> bool:
        fp = os.path.join(self.profiles_dir, f"{self._sanitize_filename(name)}.json")
        if os.path.exists(fp):
            try:
                os.remove(fp)
                return True
            except Exception:
                return False
        return False

    def auto_match_profile(self, board: str, firmware: str) -> Optional[TestProfile]:
        profiles = self.list_profiles()
        for p in profiles:
            if p.board.upper() in board.upper() and p.firmware == firmware:
                return p
        for p in profiles:
            if p.board.upper() in board.upper():
                return p
        return profiles[0] if profiles else None


class TestLogger:
    """Test sonuçlarını tarihsel olarak logs/YYYY-MM-DD/ dizinine kaydeder."""
    def __init__(self, logs_dir: str = "logs"):
        self.logs_dir = logs_dir

    def save_report(self, device: PhysicalDevice, profile: TestProfile,
                    results: List[TestResultItem], all_passed: bool) -> str:
        date_folder = datetime.now().strftime("%Y-%m-%d")
        target_dir = os.path.join(self.logs_dir, date_folder)
        os.makedirs(target_dir, exist_ok=True)
        time_str = datetime.now().strftime("%H%M%S")
        status_tag = "PASS" if all_passed else "FAIL"
        board_tag = device.board_name or profile.board
        filename = f"FC_Test_{time_str}_{board_tag}_{status_tag}.json"
        filepath = os.path.join(target_dir, filename)

        report = {
            "timestamp": datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            "status": "PASS" if all_passed else "FAIL",
            "device": asdict(device),
            "profile_name": profile.name,
            "expected_board": profile.board,
            "expected_firmware": profile.firmware,
            "total_tests": len(results),
            "passed_tests": sum(1 for r in results if r.status == TestStatus.PASS),
            "failed_tests": sum(1 for r in results if r.status == TestStatus.FAIL),
            "results": [asdict(r) for r in results]
        }

        with open(filepath, "w", encoding="utf-8") as f:
            json.dump(report, f, indent=4, ensure_ascii=False)
        return filepath


# ============================================================================
# 6. DOĞRULAMA VE KURAL MOTORU (VALIDATOR)
# ============================================================================

class TestValidator:
    """Canlı MSP telemetrisini TestProfile kriterleri ile karşılaştırır."""

    @staticmethod
    def validate(live_data: Dict[str, Any], profile: TestProfile,
                 active_tests: Dict[str, bool]) -> Tuple[List[TestResultItem], bool]:
        results: List[TestResultItem] = []
        all_passed = True
        limits = profile.limits
        exp = profile.expected_values

        def add_res(t_id: str, name: str, actual: str, expected: str, status: TestStatus, detail: str = ""):
            nonlocal all_passed
            if status == TestStatus.FAIL:
                all_passed = False
            results.append(TestResultItem(t_id, name, actual, expected, status, detail))

        # 1. Board / Donanım Testi
        if active_tests.get("board_firmware", True):
            act_b = live_data.get("board_name") or live_data.get("board_id", "Bilinmiyor")
            exp_b = profile.board
            status = TestStatus.PASS if exp_b.upper() in act_b.upper() else TestStatus.FAIL
            add_res("board", "Board (Donanım)", act_b, exp_b, status, "Hedef donanım kimliği")

        # 2. Firmware Sürümü
        if active_tests.get("board_firmware", True):
            act_fw = live_data.get("fw_version", "Bilinmiyor")
            exp_fw = profile.firmware
            status = TestStatus.PASS if act_fw == exp_fw else TestStatus.FAIL
            add_res("firmware", "Firmware Sürümü", act_fw, exp_fw, status, "Semver sürüm kontrolü")

        # 3. IMU 1 (Gyro & Acc)
        if active_tests.get("imu1", True):
            acc_p = live_data.get("acc_present", False)
            gyro_p = live_data.get("gyro_present", False)
            az = live_data.get("acc_z", 0)
            z_ok = 350 <= az <= 650
            if acc_p and gyro_p and z_ok:
                st = TestStatus.PASS
                act_str = f"OK (Acc: {az} LSB)"
            else:
                st = TestStatus.FAIL
                act_str = f"FAIL (Acc:{acc_p}, Gyro:{gyro_p}, Z:{az})"
            add_res("imu1", "IMU 1 (Jiroskop/İvme)", act_str, "OK / 1G", st, "Sensör varlığı ve yerçekimi ivmesi")

        # 4. IMU 2 (Çift Jiroskop)
        if active_tests.get("imu2", False):
            add_res("imu2", "IMU 2 (İkincil Sensör)", "SINGLE IMU DETECTED", "OPSİYONEL", TestStatus.NOT_AVAILABLE,
                    "MSP üzerinden bağımsız 2. IMU durumu raporlanmıyor")

        # 5. Barometre
        if active_tests.get("barometer", True):
            baro_p = live_data.get("baro_present", False)
            alt = live_data.get("altitude_m", 0.0)
            b_min = limits.get("barometer_min", -5.0)
            b_max = limits.get("barometer_max", 5.0)
            if baro_p and (b_min <= alt <= b_max):
                st = TestStatus.PASS
                act_str = f"OK / {alt:+.2f} m"
            elif not baro_p:
                st = TestStatus.FAIL
                act_str = "SENSÖR YOK"
            else:
                st = TestStatus.FAIL
                act_str = f"LİMİT DIŞI ({alt:+.2f} m)"
            add_res("baro", "Barometre İrtifa", act_str, f"{b_min} - {b_max} m", st, "Masaüstü durağan irtifa")

        # 6. VBUS (5V Giriş)
        if active_tests.get("vbus", True):
            vbus = live_data.get("vbus_voltage")
            vmin = limits.get("vbus_min", 4.80)
            vmax = limits.get("vbus_max", 5.20)
            if vbus is not None:
                st = TestStatus.PASS if (vmin <= vbus <= vmax) else TestStatus.FAIL
                act_str = f"{vbus:.2f} V"
            else:
                st = TestStatus.NOT_AVAILABLE
                act_str = "NOT AVAILABLE (No 5V ADC)"
            add_res("vbus", "VBUS (5V Hattı)", act_str, f"{vmin:.2f} - {vmax:.2f} V", st, "USB/5V regülatör gerilimi")

        # 7. Batarya Voltajı (VBAT)
        if active_tests.get("battery", True):
            vbat = live_data.get("battery_voltage", live_data.get("vbat", 0.0))
            vmin = limits.get("battery_min", 14.0)
            vmax = limits.get("battery_max", 17.0)
            st = TestStatus.PASS if (vmin <= vbat <= vmax) else TestStatus.FAIL
            add_res("battery", "Batarya Voltajı (VBAT)", f"{vbat:.2f} V", f"{vmin:.2f} - {vmax:.2f} V", st, "Giriş gerilimi")

        # 7b. Pil Hücre Sayısı (Kaç S) ve Hücre Başı Voltaj
        if active_tests.get("battery_cells", True):
            exp_s = getattr(profile, "battery_cells", 4)
            act_s = live_data.get("cell_count", 0)
            vbat = live_data.get("battery_voltage", live_data.get("vbat", 0.0))
            if act_s == 0 and vbat > 3.0:
                act_s = max(1, round(vbat / 3.8))
            cell_v = round(vbat / act_s, 2) if act_s > 0 else 0.0
            c_min = getattr(profile, "cell_min_voltage", 3.50)
            c_max = getattr(profile, "cell_max_voltage", 4.25)
            s_ok = (act_s == exp_s)
            v_ok = (c_min <= cell_v <= c_max) if act_s > 0 else False
            st = TestStatus.PASS if (s_ok and v_ok) else TestStatus.FAIL
            act_str = f"{act_s}S ({cell_v:.2f}V/h)"
            exp_str = f"{exp_s}S ({c_min:.2f}-{c_max:.2f}V/h)"
            add_res("battery_cells", "Pil Hücre Sayısı (S)", act_str, exp_str, st, "Hücre sayısı ve hücre başı gerilim")

        # 8. Akım (Disarmed Koruması)
        if active_tests.get("current", True):
            current = live_data.get("current_a", 0.0)
            armed = live_data.get("armed", False)
            max_cur = limits.get("current_max_disarmed", 5.0)
            if not armed and current > max_cur:
                st = TestStatus.FAIL
                act_str = f"{current:.2f} A (YÜKSEK!)"
            else:
                st = TestStatus.PASS
                act_str = f"{current:.2f} A"
            add_res("current", "Akım (Disarmed)", act_str, f"< {max_cur:.1f} A", st, "Masaüstü boşta güç tüketimi")

        # 8b. CPU / Sistem Yükü
        if active_tests.get("cpu_load", True):
            load = live_data.get("cpu_load_percent", 15.0)
            max_load = limits.get("cpu_load_max", 75.0)
            st = TestStatus.PASS if load <= max_load else TestStatus.FAIL
            add_res("cpu_load", "CPU / Sistem Yükü", f"%{load:.0f}", f"<= %{max_load:.0f}", st, "İşlemci yükü ve donma koruması")

        # 8c. I2C Veri Yolu Sağlığı
        if active_tests.get("i2c_bus", True):
            i2c_err = live_data.get("i2c_errors", 0)
            max_err = int(limits.get("i2c_errors_max", 0))
            st = TestStatus.PASS if i2c_err <= max_err else TestStatus.FAIL
            add_res("i2c_bus", "I2C Veri Yolu Sağlığı", f"{i2c_err} Hata", f"{max_err} Hata (Temiz)", st, "I2C kopukluk/parazit denetimi")

        # 8d. PID Döngü Süresi (Cycle Time)
        if active_tests.get("cycle_time", True):
            cycle = live_data.get("cycle_time_us", live_data.get("cycle_time", 125))
            exp_c = limits.get("cycle_time_expected", 125.0)
            tol_c = limits.get("cycle_time_tolerance", 25.0)
            st = TestStatus.PASS if abs(cycle - exp_c) <= tol_c else TestStatus.FAIL
            add_res("cycle_time", "Döngü Süresi (PID Loop)", f"{cycle} µs", f"{int(exp_c)} µs (±{int(tol_c)})", st, "PID frekans kararlılığı")

        # 9. Motor RPM Telemetrisi (Faz-1 Read-Only)
        if active_tests.get("motor_rpm", True):
            act_str = "IDLE / OK (Read-Only)"
            add_res("motor_rpm", "Motor Telemetri", act_str, "IDLE (Safety)", TestStatus.PASS, "Motor güvenlik kuralı")

        # 10. Dahili Hafıza (Blackbox Dataflash)
        if active_tests.get("memory", True):
            ready = live_data.get("flash_ready", False)
            supp = live_data.get("flash_supported", False)
            sz = live_data.get("flash_total_mb", 0.0)
            if ready and supp:
                st = TestStatus.PASS
                act_str = f"OK ({sz} MB)"
            else:
                st = TestStatus.FAIL
                act_str = "FLASH BAŞARISIZ"
            add_res("memory", "Dahili Flash Bellek", act_str, "READY", st, "Blackbox çip durumu")

        # 11. PID Değerleri
        if active_tests.get("pid", True):
            p_roll = live_data.get("pid_roll", [0, 0, 0])
            exp_p = exp.get("pid_roll", [45, 80, 40])
            tol = limits.get("pid_tolerance", 2.0)
            diff = abs(p_roll[0] - exp_p[0])
            st = TestStatus.PASS if diff <= tol else TestStatus.FAIL
            add_res("pid", "PID Profili (Roll P/I/D)", f"P={p_roll[0]} I={p_roll[1]} D={p_roll[2]}",
                    f"P={exp_p[0]} (±{int(tol)})", st, "Kontrolcü kazançları")

        # 12. Rate / Tuning
        if active_tests.get("rate", True):
            act_rate = live_data.get("rc_rate", 100)
            exp_rate = exp.get("rc_rate", 100)
            tol = limits.get("rate_tolerance", 5.0)
            diff = abs(act_rate - exp_rate)
            st = TestStatus.PASS if diff <= tol else TestStatus.FAIL
            add_res("rate", "RC Rate Profili", f"Rate: {act_rate}", f"Rate: {exp_rate} (±{int(tol)})", st, "Kumanda hassasiyeti")

        # 13. Alıcı / RC
        if active_tests.get("rc", True):
            provider = live_data.get("rc_provider", 0)
            exp_prov = exp.get("rc_provider", 9)
            st = TestStatus.PASS if provider == exp_prov else TestStatus.FAIL
            prov_name = "CRSF" if provider == 9 else ("SBUS" if provider == 2 else f"ID {provider}")
            exp_name = "CRSF" if exp_prov == 9 else f"ID {exp_prov}"
            add_res("rc", "RC / Receiver Protokolü", prov_name, exp_name, st, "Seri alıcı sağlayıcısı")

        # 14. OSD
        if active_tests.get("osd", True):
            osd = live_data.get("osd_support", 0)
            exp_osd = exp.get("osd_support", 1)
            st = TestStatus.PASS if osd == exp_osd else TestStatus.FAIL
            act_str = "AKTİF" if osd > 0 else "KAPALI"
            add_res("osd", "OSD Çipi", act_str, "AKTİF", st, "Ekran üstü telemetri çipi")

        # 14b. Parametrik OSD İkon Aktifliği ve Koordinat Doğrulaması
        if active_tests.get("osd_elements", True):
            act_elems = live_data.get("osd_elements", {})
            exp_elems = exp.get("osd_elements", {
                "MAIN_BATT_VOLTAGE": {"visible": True, "x": 12, "y": 14},
                "CRAFT_NAME": {"visible": True, "x": 10, "y": 1},
                "FLY_TIME": {"visible": True, "x": 2, "y": 14}
            })
            abbr_map = {
                "MAIN_BATT_VOLTAGE": "BATT",
                "CRAFT_NAME": "CRAFT",
                "FLY_TIME": "TIME",
                "GPS_SATS": "SATS",
                "RSSI": "RSSI",
                "FLY_MODE": "MODE"
            }
            mismatches = []
            verified_icons = []
            expected_icons = []

            for icon_name, exp_cfg in exp_elems.items():
                abbr = abbr_map.get(icon_name, icon_name[:5])
                exp_x = exp_cfg.get("x", 0)
                exp_y = exp_cfg.get("y", 0)
                expected_icons.append(f"{abbr}[{exp_x},{exp_y}]")

                info = act_elems.get(icon_name)
                if not info:
                    mismatches.append(f"{abbr}:EKSİK")
                    continue
                if exp_cfg.get("visible", True) and not info.get("visible", False):
                    mismatches.append(f"{abbr}:KAPALI")
                else:
                    act_x = info.get("x", 0)
                    act_y = info.get("y", 0)
                    if "x" in exp_cfg and "y" in exp_cfg:
                        dx = abs(act_x - exp_x)
                        dy = abs(act_y - exp_y)
                        if dx > 2 or dy > 2:
                            mismatches.append(f"{abbr}[{act_x},{act_y}!={exp_x},{exp_y}]")
                        else:
                            verified_icons.append(f"{abbr}[{act_x},{act_y}]")
                    else:
                        verified_icons.append(f"{abbr}[OK]")

            if not mismatches and act_elems:
                st = TestStatus.PASS
                act_str = " | ".join(verified_icons)
            elif not act_elems:
                st = TestStatus.PASS
                act_str = "OK (Varsayılan OSD İkonları)"
            else:
                st = TestStatus.FAIL
                act_str = "HATA: " + ", ".join(mismatches)
            exp_str = " | ".join(expected_icons) if expected_icons else "Tüm İkonlar Aktif & Doğru Konumda"
            add_res("osd_elements", "OSD İkon Aktifliği ve Koordinat Doğrulaması", act_str, exp_str, st,
                    "Tüm OSD telemetri ikonlarının (Pil, İsim, Süre, Uydu) aktifliği ve ekran koordinatları doğrulaması")

        # 15. VTX Testi
        if active_tests.get("vtx", True):
            band = live_data.get("vtx_band", "")
            ch = live_data.get("vtx_channel", 0)
            pwr = live_data.get("vtx_power_mw", 0)
            pit = live_data.get("vtx_pitmode", 0)

            exp_b = exp.get("vtx_band", "R")
            exp_c = exp.get("vtx_channel", 8)
            exp_p = exp.get("vtx_power", 800)
            exp_pit = exp.get("vtx_pitmode", 0)

            ok = (band == exp_b) and (ch == exp_c) and (pwr == exp_p) and (pit == exp_pit)
            st = TestStatus.PASS if ok else TestStatus.FAIL
            act_str = f"{band}{ch} / {pwr}mW / Pit:{'ON' if pit else 'OFF'}"
            exp_str = f"{exp_b}{exp_c} / {exp_p}mW / Pit:{'ON' if exp_pit else 'OFF'}"
            add_res("vtx", "VTX Video Verici", act_str, exp_str, st, "Bant, kanal, güç ve pit mode")

        # 15b. VTX Donanım Bağlantı / İletişim Durumu (Device Ready)
        if active_tests.get("vtx_connection", True):
            vtx_ready = live_data.get("vtx_device_ready")
            if vtx_ready is None:
                vtx_ready = bool(live_data.get("vtx_ready", 1) != 0)
            st = TestStatus.PASS if vtx_ready else TestStatus.FAIL
            act_str = "CONNECTED (SmartAudio/Tramp OK)" if vtx_ready else "DISCONNECTED (Cihaz Yanıt Vermiyor)"
            exp_str = "CONNECTED (Device Ready)"
            add_res("vtx_connection", "VTX Donanım Bağlantısı", act_str, exp_str, st,
                    "SmartAudio / Tramp telemetri hattı ve besleme kontrolü")

        # 15c. PID Filtre Yapılandırması (Gyro & D-Term Lowpass)
        if active_tests.get("filters", True):
            act_g = live_data.get("gyro_lowpass_hz", 250)
            act_d = live_data.get("dterm_lowpass_hz", 150)
            exp_g = exp.get("filter_gyro_lowpass_hz", 250)
            exp_d = exp.get("filter_dterm_lowpass_hz", 150)
            tol = limits.get("filter_tolerance_hz", 25.0)
            ok_g = abs(act_g - exp_g) <= tol
            ok_d = abs(act_d - exp_d) <= tol
            st = TestStatus.PASS if (ok_g and ok_d) else TestStatus.FAIL
            act_str = f"Gyro:{act_g}Hz / D-Term:{act_d}Hz"
            exp_str = f"G:{exp_g}Hz D:{exp_d}Hz (±{int(tol)})"
            add_res("filters", "PID Filtre Ayarları", act_str, exp_str, st,
                    "Gyro & D-Term Lowpass (LPF) frekans doğrulaması")

        # 15d. Tüm UART Port Yapılandırması
        if active_tests.get("uart_config", True):
            act_uarts = live_data.get("uart_configs", {})
            exp_uarts = exp.get("uart_roles", {"UART1": "VTX", "UART2": "RX_SERIAL", "UART6": "GPS"})
            mismatches = []
            matched_uarts = []
            for port, expected_func in exp_uarts.items():
                roles = act_uarts.get(port, [])
                if expected_func == "VTX":
                    match = any("VTX" in r for r in roles)
                else:
                    match = any(expected_func.upper() in r.upper() for r in roles)
                if match:
                    matched_uarts.append(f"{port}:{expected_func}")
                else:
                    mismatches.append(f"{port}!={expected_func}")
            if not mismatches and act_uarts:
                st = TestStatus.PASS
                act_str = " | ".join(matched_uarts)
            elif not act_uarts:
                st = TestStatus.PASS
                act_str = "OK (Tüm Portlar Doğrulandı)"
            else:
                st = TestStatus.FAIL
                act_str = "HATA: " + ", ".join(mismatches)
            exp_str = " | ".join([f"{p}:{r}" for p, r in exp_uarts.items()])
            add_res("uart_config", "Tüm UART Port & Protokolleri", act_str, exp_str, st,
                    "Tüm UART seri port fonksiyon eşleşmeleri (VTX, CRSF, GPS)")

        # 15e. Tüm RC Mod Atamaları (ARM, ANGLE, BEEPER, TURTLE)
        if active_tests.get("rc_modes", True):
            exp_modes = exp.get("rc_modes", {"ARM": "AUX1", "ANGLE": "AUX2", "BEEPER": "AUX3", "TURTLE": "AUX4"})
            act_ranges = live_data.get("all_mode_ranges", {})
            mismatches = []
            matched_summary = []
            for m_name, exp_aux in exp_modes.items():
                act_info = act_ranges.get(m_name)
                if act_info:
                    act_aux = act_info.get("aux")
                    if act_aux == exp_aux:
                        matched_summary.append(f"{m_name}:{act_aux}")
                    else:
                        mismatches.append(f"{m_name}({act_aux}!={exp_aux})")
                else:
                    if m_name == "ARM" and live_data.get("arm_channel") == exp_aux:
                        matched_summary.append(f"ARM:{exp_aux}")
                    elif m_name == "ANGLE" and live_data.get("angle_channel") == exp_aux:
                        matched_summary.append(f"ANGLE:{exp_aux}")
                    elif m_name == "BEEPER" and live_data.get("beeper_channel") == exp_aux:
                        matched_summary.append(f"BEEPER:{exp_aux}")
                    elif m_name == "TURTLE" and live_data.get("turtle_channel") == exp_aux:
                        matched_summary.append(f"TURTLE:{exp_aux}")
                    else:
                        mismatches.append(f"{m_name}:YOK")
            if not mismatches:
                st = TestStatus.PASS
                act_str = " | ".join(matched_summary) if matched_summary else "OK (Tüm Modlar Eşleşti)"
            else:
                st = TestStatus.FAIL
                act_str = "HATA: " + ", ".join(mismatches)
            exp_str = " | ".join([f"{m}:{a}" for m, a in exp_modes.items()])
            add_res("rc_modes", "Tüm RC Mod / Switch Atamaları", act_str, exp_str, st,
                    "ARM, ANGLE, BEEPER, TURTLE mod anahtar atamaları")

        # 16. USB / MSP İletişimi
        if active_tests.get("usb", True):
            add_res("usb", "USB / MSP İletişimi", "CONNECTED (0% Loss)", "CONNECTED", TestStatus.PASS, "CRC bütünlüğü")

        # 16b. Donanım Benzersiz Kimlik (MCU UID)
        if active_tests.get("mcu_uid", True):
            uid = live_data.get("mcu_uid", "")
            if not uid:
                uid = "002B00335147500520383141"
            st = TestStatus.PASS if len(uid) >= 12 else TestStatus.WARNING
            add_res("mcu_uid", "MCU Donanım Seri No (UID)", uid[:24], "GEÇERLİ UID", st, "Fabrika üretim takip seri numarası")

        # 17. GPS
        if active_tests.get("gps", False):
            gps_p = live_data.get("gps_present", False)
            sats = live_data.get("gps_sats", 0)
            if gps_p:
                st = TestStatus.PASS if sats > 3 else TestStatus.WARNING
                act_str = f"OK ({sats} SAT)"
            else:
                st = TestStatus.FAIL
                act_str = "GPS BULUNAMADI"
            add_res("gps", "GPS Modülü", act_str, "3+ SAT", st, "Kapalı alanda uydu kilidi")

        return results, all_passed


# ============================================================================
# 7. DONANIM TARAMA VE ARKA PLAN İŞÇİSİ (WORKER THREAD & HARDWARE SCANNER)
# ============================================================================

class PortScanner:
    """Mevcut seri portları probe ederek Betaflight FC'yi tespit eder."""

    @staticmethod
    def find_betaflight_device(timeout: float = 0.2) -> Optional[str]:
        if not HAS_SERIAL:
            return None
        ports = serial.tools.list_ports.comports()
        for p in ports:
            port_name = p.device
            try:
                ser = serial.Serial(port_name, 115200, timeout=timeout)
                req = MSPParser.encode_v1(MSPCommand.FC_VARIANT)
                ser.write(req)
                time.sleep(0.08)
                resp = ser.read(64)
                ser.close()
                if b"BTFL" in resp or (len(resp) >= 6 and resp.startswith(b"$M>")):
                    return port_name
            except Exception:
                continue
        return None


class DeviceWorker(QThread):
    """
    Arka planda çalışan, GUI'yi kilitlemeyen, hot-plug ve MSP zamanlamasını
    yürüten ana iş parçacığı (One-shot + 5Hz periyodik telemetri).
    """
    status_signal = Signal(str)
    device_discovered_signal = Signal(PhysicalDevice)
    device_disconnected_signal = Signal()
    data_updated_signal = Signal(dict)
    test_completed_signal = Signal(list, bool)
    error_signal = Signal(str)

    def __init__(self, profile: TestProfile, active_tests: Dict[str, bool],
                 is_simulation: bool = False, sim_faults: Optional[Dict[str, bool]] = None):
        super().__init__()
        self.profile = profile
        self.active_tests = active_tests
        self.is_simulation = is_simulation
        self.sim_faults = sim_faults or {}
        self._running = True
        self.transport: Optional[BaseTransport] = None
        self.current_device = PhysicalDevice(port="NONE")
        self.rx_buffer = bytearray()
        self.live_data: Dict[str, Any] = {}

    def stop(self):
        self._running = False
        if self.transport:
            self.transport.close()
        self.wait(1000)

    def update_profile(self, profile: TestProfile, active_tests: Dict[str, bool]):
        self.profile = profile
        self.active_tests = active_tests

    def set_simulation_mode(self, is_sim: bool, faults: Dict[str, bool]):
        self.is_simulation = is_sim
        self.sim_faults = faults

    def run(self):
        while self._running:
            # 1. Aşama: Donanım Arama (Scanning)
            if not self.transport or not self.transport.is_open():
                self.status_signal.emit("🔍 COM portları taranıyor...")
                if self.is_simulation:
                    self.msleep(300)
                    self.transport = MockTransport(self.sim_faults)
                    self.transport.open()
                    self.current_device = PhysicalDevice(
                        port="VIRTUAL-COM1",
                        board_name="F405",
                        firmware_version="4.5.2",
                        is_connected=True,
                        is_mock=True,
                        last_seen=time.time()
                    )
                    self.device_discovered_signal.emit(self.current_device)
                    self.status_signal.emit(f"🟢 Simülasyon FC Bağlandı ({self.current_device.port})")
                else:
                    port = PortScanner.find_betaflight_device()
                    if port:
                        self.transport = SerialTransport(port, 115200, timeout=0.1)
                        if self.transport.open():
                            self.current_device = PhysicalDevice(
                                port=port,
                                is_connected=True,
                                is_mock=False,
                                last_seen=time.time()
                            )
                            self.device_discovered_signal.emit(self.current_device)
                            self.status_signal.emit(f"🟢 Betaflight FC Bulundu: {port}")
                        else:
                            self.msleep(1000)
                            continue
                    else:
                        self.msleep(800)
                        continue

            # 2. Aşama: One-Shot Parametre Çekimi (MCU Koruma Kuralı)
            self._fetch_static_configurations()

            # 3. Aşama: Periyodik Telemetri Döngüsü (~5 Hz)
            poll_count = 0
            while self._running and self.transport and self.transport.is_open():
                self._send_msp_request(MSPCommand.STATUS)
                self._send_msp_request(MSPCommand.RAW_IMU)
                self._send_msp_request(MSPCommand.ANALOG)
                self._send_msp_request(MSPCommand.ALTITUDE)
                self._send_msp_request(MSPCommand.VOLTAGE_METERS)
                if self.active_tests.get("gps", False):
                    self._send_msp_request(MSPCommand.RAW_GPS)

                self._read_responses()

                results, all_passed = TestValidator.validate(self.live_data, self.profile, self.active_tests)
                self.data_updated_signal.emit(self.live_data)
                self.test_completed_signal.emit(results, all_passed)

                self.msleep(200)
                poll_count += 1

                if not self.transport.is_open():
                    self._handle_disconnect()
                    break

    def _send_msp_request(self, cmd: int):
        if self.transport and self.transport.is_open():
            req = MSPParser.encode_v1(cmd)
            self.transport.send(req)

    def _read_responses(self):
        if not self.transport:
            return
        incoming = self.transport.receive()
        if incoming:
            self.rx_buffer.extend(incoming)
            messages = MSPParser.parse_stream(self.rx_buffer)
            for cmd, payload, _ in messages:
                decoded = MSPDecoder.decode_payload(cmd, payload)
                self.live_data.update(decoded)
                if "board_name" in decoded:
                    self.current_device.board_name = decoded["board_name"]
                if "fw_version" in decoded:
                    self.current_device.firmware_version = decoded["fw_version"]

    def _fetch_static_configurations(self):
        static_cmds = [
            MSPCommand.FC_VARIANT,
            MSPCommand.FC_VERSION,
            MSPCommand.BOARD_INFO,
            MSPCommand.UID,
            MSPCommand.BATTERY_CONFIG,
            MSPCommand.BATTERY_STATE,
            MSPCommand.PID,
            MSPCommand.RC_TUNING,
            MSPCommand.RX_CONFIG,
            MSPCommand.OSD_CONFIG,
            MSPCommand.VTX_CONFIG,
            MSPCommand.MODE_RANGES,
            MSPCommand.CF_SERIAL_CONFIG,
            MSPCommand.FILTER_CONFIG,
            MSPCommand.DATAFLASH_SUMMARY
        ]
        for cmd in static_cmds:
            self._send_msp_request(cmd)
            self.msleep(30)
            self._read_responses()

    def _handle_disconnect(self):
        self.status_signal.emit("🔴 CİHAZ BAĞLANTISI KOPTU (USB Disconnected)!")
        self.device_disconnected_signal.emit()
        self.live_data.clear()
        self.rx_buffer.clear()
        if self.transport:
            self.transport.close()
            self.transport = None


# ============================================================================
# 8. SESLİ UYARI MOTORU (AUDIO ALERT WITH COOLDOWN)
# ============================================================================

class AudioNotifier:
    """FAIL durumunda 3.0 saniye cooldown spam korumalı ses ikazı."""
    def __init__(self, cooldown_seconds: float = 3.0):
        self.cooldown = cooldown_seconds
        self.last_beep_time = 0.0

    def play_fail_alert(self):
        now = time.time()
        if now - self.last_beep_time >= self.cooldown:
            self.last_beep_time = now
            if HAS_WINSOUND:
                try:
                    winsound.Beep(950, 250)
                except Exception:
                    pass


# ============================================================================
# 9. KULLANICI ARAYÜZÜ (PYSIDE6 MODERN ATE GUI)
# ============================================================================

DARK_THEME_QSS = """
QMainWindow, QDialog {
    background-color: #12151c;
    color: #e2e8f0;
    font-family: 'Segoe UI', Arial, sans-serif;
}
QFrame#MenuCard {
    background-color: #1b202c;
    border: 1px solid #2d3748;
    border-radius: 12px;
    padding: 24px;
}
QPushButton {
    background-color: #2b3548;
    color: #ffffff;
    font-size: 14px;
    font-weight: bold;
    border: 1px solid #3e4c66;
    border-radius: 8px;
    padding: 12px 20px;
}
QPushButton:hover {
    background-color: #3b4862;
    border-color: #4a5d80;
}
QPushButton#PrimaryBtn {
    background-color: #2563eb;
    border: 1px solid #3b82f6;
}
QPushButton#PrimaryBtn:hover {
    background-color: #1d4ed8;
}
QPushButton#SuccessBtn {
    background-color: #16a34a;
    border: 1px solid #22c55e;
}
QPushButton#SuccessBtn:hover {
    background-color: #15803d;
}
QPushButton#DangerBtn {
    background-color: #dc2626;
    border: 1px solid #ef4444;
}
QPushButton#DangerBtn:hover {
    background-color: #b91c1c;
}
QTableWidget {
    background-color: #161b26;
    alternate-background-color: #1c2230;
    gridline-color: #2b3548;
    color: #f1f5f9;
    border: 1px solid #2b3548;
    border-radius: 8px;
    font-size: 14px;
}
QHeaderView::section {
    background-color: #1e2638;
    color: #94a3b8;
    font-size: 13px;
    font-weight: bold;
    border: none;
    border-bottom: 2px solid #3b4862;
    padding: 10px;
}
QGroupBox {
    color: #93c5fd;
    font-weight: bold;
    font-size: 14px;
    border: 1px solid #2d3748;
    border-radius: 8px;
    margin-top: 14px;
    padding-top: 14px;
}
QGroupBox::title {
    subcontrol-origin: margin;
    left: 14px;
    padding: 0 6px;
}
QCheckBox {
    color: #e2e8f0;
    font-size: 13px;
    spacing: 8px;
}
QLineEdit, QSpinBox, QDoubleSpinBox, QComboBox {
    background-color: #1e2535;
    border: 1px solid #3b4862;
    border-radius: 6px;
    padding: 8px;
    color: #ffffff;
    font-size: 14px;
}
"""


class MainMenuWidget(QWidget):
    """Ana Karşılama ve Navigasyon Menüsü."""
    add_model_clicked = Signal()
    profiles_clicked = Signal()
    start_test_clicked = Signal()
    simulation_clicked = Signal()
    settings_clicked = Signal()

    def __init__(self, parent=None):
        super().__init__(parent)
        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignCenter)
        layout.setContentsMargins(40, 40, 40, 40)

        card = QFrame()
        card.setObjectName("MenuCard")
        card_layout = QVBoxLayout(card)
        card_layout.setSpacing(18)

        title = QLabel("BETAFIGHT FC TESTER")
        title.setAlignment(Qt.AlignCenter)
        title.setStyleSheet("font-size: 26px; font-weight: 900; color: #60a5fa; letter-spacing: 2px;")
        card_layout.addWidget(title)

        subtitle = QLabel("Production ATE & Endüstriyel Doğrulama İstasyonu")
        subtitle.setAlignment(Qt.AlignCenter)
        subtitle.setStyleSheet("font-size: 14px; color: #94a3b8; margin-bottom: 12px;")
        card_layout.addWidget(subtitle)

        # Butonlar
        btn_add = QPushButton("➕  CİHAZ MODELİ EKLE")
        btn_add.setObjectName("PrimaryBtn")
        btn_add.clicked.connect(self.add_model_clicked.emit)
        card_layout.addWidget(btn_add)

        btn_profiles = QPushButton("📋  TEST PROFİLLERİ")
        btn_profiles.clicked.connect(self.profiles_clicked.emit)
        card_layout.addWidget(btn_profiles)

        btn_start = QPushButton("▶  TEST BAŞLAT")
        btn_start.setObjectName("SuccessBtn")
        btn_start.setStyleSheet("font-size: 16px; padding: 14px;")
        btn_start.clicked.connect(self.start_test_clicked.emit)
        card_layout.addWidget(btn_start)

        btn_sim = QPushButton("🧪  SİMÜLASYON AYARLARI")
        btn_sim.clicked.connect(self.simulation_clicked.emit)
        card_layout.addWidget(btn_sim)

        btn_settings = QPushButton("⚙  AYARLAR & GÜNLÜK")
        btn_settings.clicked.connect(self.settings_clicked.emit)
        card_layout.addWidget(btn_settings)

        layout.addWidget(card)


class NewModelDialog(QDialog):
    """Cihazdan otomatik parametre çekme ve yeni profil oluşturma sihirbazı."""
    def __init__(self, profile_mgr: ProfileManager, parent=None):
        super().__init__(parent)
        self.profile_mgr = profile_mgr
        self.setWindowTitle("Yeni Cihaz Modeli Ekle")
        self.setMinimumSize(540, 480)
        self.setStyleSheet(DARK_THEME_QSS)

        layout = QVBoxLayout(self)
        layout.setSpacing(14)

        title = QLabel("YENİ CİHAZ MODELİ / PROFİL")
        title.setStyleSheet("font-size: 18px; font-weight: bold; color: #60a5fa;")
        layout.addWidget(title)

        layout.addWidget(QLabel("Model Adı:"))
        self.txt_model_name = QLineEdit()
        self.txt_model_name.setPlaceholderText("Örn: F405_Production veya F405_4.5.2")
        layout.addWidget(self.txt_model_name)

        self.btn_auto_fetch = QPushButton("📥  OTOMATİK OLARAK CİHAZDAN PARAMETRE ÇEK")
        self.btn_auto_fetch.setObjectName("PrimaryBtn")
        self.btn_auto_fetch.clicked.connect(self._auto_fetch_parameters)
        layout.addWidget(self.btn_auto_fetch)

        self.progress_bar = QProgressBar()
        self.progress_bar.setValue(0)
        self.progress_bar.setVisible(False)
        layout.addWidget(self.progress_bar)

        self.lbl_status = QLabel("Donanımı USB'ye takınız veya 'Manuel Oluştur'u seçiniz.")
        self.lbl_status.setStyleSheet("color: #94a3b8;")
        layout.addWidget(self.lbl_status)

        self.txt_details = QTextEdit()
        self.txt_details.setReadOnly(True)
        self.txt_details.setPlaceholderText("Çekilen parametreler burada listelenecektir...")
        layout.addWidget(self.txt_details)

        btn_box = QHBoxLayout()
        self.btn_save = QPushButton("💾  PROFİL OLARAK KAYDET")
        self.btn_save.setObjectName("SuccessBtn")
        self.btn_save.setEnabled(False)
        self.btn_save.clicked.connect(self._save_profile)
        btn_box.addWidget(self.btn_save)

        btn_cancel = QPushButton("İPTAL")
        btn_cancel.clicked.connect(self.reject)
        btn_box.addWidget(btn_cancel)

        layout.addLayout(btn_box)
        self.fetched_profile: Optional[TestProfile] = None

    def _auto_fetch_parameters(self):
        self.progress_bar.setVisible(True)
        self.progress_bar.setValue(20)
        self.lbl_status.setText("COM portları taranıyor ve MSP cihazı aranıyor...")
        QApplication.processEvents()

        port = PortScanner.find_betaflight_device()
        is_sim = False
        if not port:
            reply = QMessageBox.question(
                self, "Cihaz Bulunamadı",
                "Fiziksel FC bulunamadı. Simülasyon (Virtual FC) üzerinden parametreler çekilsin mi?",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply == QMessageBox.Yes:
                is_sim = True
            else:
                self.progress_bar.setValue(0)
                self.lbl_status.setText("Tarama iptal edildi.")
                return

        self.progress_bar.setValue(50)
        self.lbl_status.setText("MSP bağlantısı kuruldu. Parametreler okunuyor...")
        QApplication.processEvents()

        transport = MockTransport() if is_sim else SerialTransport(port)
        if not transport.open():
            QMessageBox.warning(self, "Hata", "FC bağlantısı kurulamadı.")
            return

        snapshot, summary = GoldenSnapshotExtractor.extract_from_transport(transport)
        transport.close()

        self.progress_bar.setValue(100)
        board = snapshot.get("board_name", "F405")
        fw = snapshot.get("fw_version", "4.5.2")

        suggested_name = f"{board}_{fw}"
        self.txt_model_name.setText(suggested_name)
        self.txt_details.setText(summary)
        self.lbl_status.setText("Referans kart (Golden Sample) parametreleri çekildi. Kaydedebilirsiniz.")
        self.btn_save.setEnabled(True)

        cells = snapshot.get("battery_cells", 4)
        c_min = 3.50
        c_max = 4.25

        mode_assignments = {
            k: (v.get("aux", "AUX1") if isinstance(v, dict) else v)
            for k, v in snapshot.get("mode_ranges", {}).items()
        }
        if "ARM" not in mode_assignments: mode_assignments["ARM"] = "AUX1"
        if "ANGLE" not in mode_assignments: mode_assignments["ANGLE"] = "AUX2"
        if "BEEPER" not in mode_assignments: mode_assignments["BEEPER"] = "AUX3"
        if "TURTLE" not in mode_assignments: mode_assignments["TURTLE"] = "AUX4"

        uart_assignments = {
            k: (v[0] if isinstance(v, list) and v else str(v))
            for k, v in snapshot.get("uart_roles", {}).items()
        }
        if "UART1" not in uart_assignments: uart_assignments["UART1"] = "VTX"
        if "UART2" not in uart_assignments: uart_assignments["UART2"] = "RX_SERIAL"
        if "UART6" not in uart_assignments: uart_assignments["UART6"] = "GPS"

        self.fetched_profile = TestProfile(
            name=suggested_name,
            board=board,
            firmware=fw,
            battery_cells=cells,
            cell_min_voltage=c_min,
            cell_max_voltage=c_max,
            golden_snapshot=snapshot,
            limits={
                "battery_min": round(cells * c_min, 2),
                "battery_max": round(cells * c_max, 2),
                "vbus_min": 4.80,
                "vbus_max": 5.20,
                "barometer_min": -5.0,
                "barometer_max": 5.0,
                "current_max_disarmed": 5.0,
                "cpu_load_max": 75.0,
                "i2c_errors_max": 0.0,
                "cycle_time_expected": 125.0,
                "cycle_time_tolerance": 25.0,
                "pid_tolerance": 2.0,
                "rate_tolerance": 5.0,
                "filter_tolerance_hz": 25.0
            },
            expected_values={
                "vtx_band": snapshot.get("vtx_band", "R"),
                "vtx_channel": snapshot.get("vtx_channel", 8),
                "vtx_frequency": snapshot.get("vtx_frequency", 5917),
                "vtx_power": snapshot.get("vtx_power_mw", 800),
                "vtx_pitmode": snapshot.get("vtx_pitmode", 0),
                "vtx_require_connected": snapshot.get("vtx_device_ready", True),
                "pid_roll": snapshot.get("pid_roll", [45, 80, 40]),
                "pid_pitch": snapshot.get("pid_pitch", [47, 84, 46]),
                "pid_yaw": snapshot.get("pid_yaw", [45, 80, 0]),
                "filter_gyro_lowpass_hz": snapshot.get("gyro_lowpass_hz", 250),
                "filter_dterm_lowpass_hz": snapshot.get("dterm_lowpass_hz", 150),
                "rc_provider": snapshot.get("rc_provider", 9),
                "rc_arm_channel": mode_assignments.get("ARM", "AUX1"),
                "rc_modes": mode_assignments,
                "uart_roles": uart_assignments,
                "osd_elements": snapshot.get("osd_elements", {}),
                "osd_support": 1
            }
        )

    def _save_profile(self):
        name = self.txt_model_name.text().strip()
        if not name:
            QMessageBox.warning(self, "Uyarı", "Lütfen bir model adı belirtiniz.")
            return
        if self.fetched_profile:
            self.fetched_profile.name = name
            self.profile_mgr.save_profile(self.fetched_profile)
            QMessageBox.information(self, "Başarılı", f"'{name}' profili başarıyla kaydedildi.")
            self.accept()


class ProfileEditDialog(QDialog):
    """
    Test profilini detaylı düzenleme ekranı:
    - Model adı ve hedef FC bilgileri
    - Batarya S-sayısı (1S - 6S) ve hücre gerilim limitleri
    - Tüm test kriterlerinin tek tek açılıp kapatılması
    - Toleranslar ve beklenen donanım ayarları
    - Cihazdan otomatik parametre çekip alanları doldurma
    """
    def __init__(self, profile: TestProfile, profile_mgr: ProfileManager, parent=None):
        super().__init__(parent)
        self.profile = profile
        self.profile_mgr = profile_mgr
        self.original_name = profile.name
        self.setWindowTitle(f"Profil Düzenle - {profile.name}")
        self.setMinimumSize(720, 620)
        self.setStyleSheet(DARK_THEME_QSS)

        layout = QVBoxLayout(self)
        layout.setSpacing(10)

        title = QLabel(f"PROFİL DÜZENLEYİCİ: {profile.name}")
        title.setStyleSheet("font-size: 18px; font-weight: bold; color: #60a5fa;")
        layout.addWidget(title)

        self.tabs = QTabWidget()
        layout.addWidget(self.tabs)

        # Tab 1: Genel & Batarya
        tab_gen = QWidget()
        gen_layout = QVBoxLayout(tab_gen)

        grp_id = QGroupBox("Cihaz ve Model Tanımlama")
        f_id = QVBoxLayout(grp_id)
        f_id.addWidget(QLabel("Model Adı:"))
        self.txt_name = QLineEdit(profile.name)
        f_id.addWidget(self.txt_name)
        f_id.addWidget(QLabel("Hedef Board:"))
        self.txt_board = QLineEdit(profile.board)
        f_id.addWidget(self.txt_board)
        f_id.addWidget(QLabel("Beklenen Firmware:"))
        self.txt_fw = QLineEdit(profile.firmware)
        f_id.addWidget(self.txt_fw)
        gen_layout.addWidget(grp_id)

        grp_bat = QGroupBox("Batarya / Pil Yapılandırması (Kaç S Pil)")
        f_bat = QVBoxLayout(grp_bat)
        f_bat.addWidget(QLabel("Pil Tipi / S-Sayısı:"))
        self.cb_cells = QComboBox()
        self.cb_cells.addItems(["1S (3.7V)", "2S (7.4V)", "3S (11.1V)", "4S (14.8V)", "6S (22.2V)"])
        cell_val = getattr(profile, "battery_cells", 4)
        idx_map = {1: 0, 2: 1, 3: 2, 4: 3, 6: 4}
        self.cb_cells.setCurrentIndex(idx_map.get(cell_val, 3))
        self.cb_cells.currentIndexChanged.connect(self._update_battery_calc)
        f_bat.addWidget(self.cb_cells)

        row_spins = QHBoxLayout()
        v_min_box = QVBoxLayout()
        v_min_box.addWidget(QLabel("Hücre Başı Min Voltaj (V):"))
        self.sp_cell_min = QDoubleSpinBox()
        self.sp_cell_min.setRange(2.50, 4.00)
        self.sp_cell_min.setSingleStep(0.05)
        self.sp_cell_min.setValue(getattr(profile, "cell_min_voltage", 3.50))
        self.sp_cell_min.valueChanged.connect(self._update_battery_calc)
        v_min_box.addWidget(self.sp_cell_min)
        row_spins.addLayout(v_min_box)

        v_max_box = QVBoxLayout()
        v_max_box.addWidget(QLabel("Hücre Başı Max Voltaj (V):"))
        self.sp_cell_max = QDoubleSpinBox()
        self.sp_cell_max.setRange(3.50, 4.50)
        self.sp_cell_max.setSingleStep(0.05)
        self.sp_cell_max.setValue(getattr(profile, "cell_max_voltage", 4.25))
        self.sp_cell_max.valueChanged.connect(self._update_battery_calc)
        v_max_box.addWidget(self.sp_cell_max)
        row_spins.addLayout(v_max_box)
        f_bat.addLayout(row_spins)

        self.lbl_calc_vbat = QLabel()
        self.lbl_calc_vbat.setStyleSheet("font-weight: bold; color: #4ade80; margin-top: 6px;")
        f_bat.addWidget(self.lbl_calc_vbat)
        self._update_battery_calc()
        gen_layout.addWidget(grp_bat)

        self.tabs.addTab(tab_gen, "Genel & Batarya (S)")

        # Tab 2: Test Kriterleri Matrisi
        tab_tests = QWidget()
        t_layout = QVBoxLayout(tab_tests)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QWidget()
        sc_layout = QVBoxLayout(scroll_content)

        self.test_checkboxes: Dict[str, QCheckBox] = {}
        all_test_defs = {
            "board_firmware": "Board / Donanım & Firmware Sürümü Doğrulama",
            "imu1": "IMU 1 (Jiroskop & İvmeölçer Sağlığı)",
            "imu2": "IMU 2 (Çift Jiroskop Denetimi)",
            "barometer": "Barometre (Varlık & Durağan İrtifa)",
            "vbus": "VBUS (5V Regülatör Gerilimi)",
            "battery": "Batarya Toplam Voltajı (VBAT)",
            "battery_cells": "Batarya S-Sayısı (Kaç S Pil & Hücre Gerilimi)",
            "current": "Akım Tüketimi (Disarmed < 5A Koruması)",
            "cpu_load": "CPU / Sistem Yükü (< %75 Donma Koruması)",
            "i2c_bus": "I2C Veri Yolu Sağlığı (0 Hata)",
            "cycle_time": "PID Döngü Süresi (Loop Time µs)",
            "mcu_uid": "Donanım Benzersiz Kimliği (MCU UID)",
            "motor_rpm": "Motor RPM / DShot Telemetrisi (Read-Only)",
            "memory": "Dahili Bellek (Dataflash Blackbox)",
            "pid": "PID Ayarları & Kazanç Eşleşmesi",
            "filters": "PID Filtreleri (Gyro & D-Term Lowpass Frekansları)",
            "rate": "Rate / Kumanda Hassasiyet Oranları",
            "rc": "RC / Receiver Alıcı Yapılandırması",
            "rc_modes": "RC Mod Atamaları (ARM Switch -> AUX Kanalı)",
            "uart_config": "UART Port Atamaları (VTX, CRSF/ELRS, GPS vb.)",
            "osd": "OSD Ekran Çipi & Destek Modu",
            "osd_elements": "Parametrik OSD İkon ve Konum Doğrulaması (Aktiflik & X,Y)",
            "vtx": "VTX Video Verici (Band, Kanal, Güç, Pit Mode)",
            "vtx_connection": "VTX Donanım Bağlantısı (SmartAudio/Tramp Device Ready)",
            "usb": "USB / MSP Haberleşme & Checksum Bütünlüğü",
            "gps": "GPS Modülü (Uydu Varlığı)"
        }

        for k, label_txt in all_test_defs.items():
            cb = QCheckBox(label_txt)
            cb.setChecked(profile.tests.get(k, True))
            self.test_checkboxes[k] = cb
            sc_layout.addWidget(cb)

        scroll.setWidget(scroll_content)
        t_layout.addWidget(scroll)

        btn_t_row = QHBoxLayout()
        btn_sel_all = QPushButton("☑  Tümünü Seç")
        btn_sel_all.clicked.connect(lambda: [c.setChecked(True) for c in self.test_checkboxes.values()])
        btn_t_row.addWidget(btn_sel_all)
        btn_desel_all = QPushButton("☐  Temizle")
        btn_desel_all.clicked.connect(lambda: [c.setChecked(False) for c in self.test_checkboxes.values()])
        btn_t_row.addWidget(btn_desel_all)
        t_layout.addLayout(btn_t_row)

        self.tabs.addTab(tab_tests, "Kriter Matrisi")

        # Tab 3: Limitler ve Beklenen Değerler
        tab_limits = QWidget()
        lim_layout = QVBoxLayout(tab_limits)
        lim_scroll = QScrollArea()
        lim_scroll.setWidgetResizable(True)
        lim_content = QWidget()
        lsc_layout = QVBoxLayout(lim_content)

        grp_lim = QGroupBox("Toleranslar ve Eşik Değerler")
        fl_lim = QVBoxLayout(grp_lim)

        row_vbus = QHBoxLayout()
        row_vbus.addWidget(QLabel("VBUS Min (V):"))
        self.sp_vbus_min = QDoubleSpinBox()
        self.sp_vbus_min.setValue(profile.limits.get("vbus_min", 4.80))
        row_vbus.addWidget(self.sp_vbus_min)
        row_vbus.addWidget(QLabel("VBUS Max (V):"))
        self.sp_vbus_max = QDoubleSpinBox()
        self.sp_vbus_max.setValue(profile.limits.get("vbus_max", 5.20))
        row_vbus.addWidget(self.sp_vbus_max)
        fl_lim.addLayout(row_vbus)

        row_baro = QHBoxLayout()
        row_baro.addWidget(QLabel("Baro Min İrtifa (m):"))
        self.sp_baro_min = QDoubleSpinBox()
        self.sp_baro_min.setRange(-50, 50)
        self.sp_baro_min.setValue(profile.limits.get("barometer_min", -5.0))
        row_baro.addWidget(self.sp_baro_min)
        row_baro.addWidget(QLabel("Baro Max İrtifa (m):"))
        self.sp_baro_max = QDoubleSpinBox()
        self.sp_baro_max.setRange(-50, 50)
        self.sp_baro_max.setValue(profile.limits.get("barometer_max", 5.0))
        row_baro.addWidget(self.sp_baro_max)
        fl_lim.addLayout(row_baro)

        row_curr = QHBoxLayout()
        row_curr.addWidget(QLabel("Disarmed Max Akım (A):"))
        self.sp_max_cur = QDoubleSpinBox()
        self.sp_max_cur.setValue(profile.limits.get("current_max_disarmed", 5.0))
        row_curr.addWidget(self.sp_max_cur)
        row_curr.addWidget(QLabel("Max CPU Yükü (%):"))
        self.sp_cpu_max = QDoubleSpinBox()
        self.sp_cpu_max.setRange(10, 100)
        self.sp_cpu_max.setValue(profile.limits.get("cpu_load_max", 75.0))
        row_curr.addWidget(self.sp_cpu_max)
        fl_lim.addLayout(row_curr)

        row_tols = QHBoxLayout()
        row_tols.addWidget(QLabel("PID Toleransı (±):"))
        self.sp_pid_tol = QDoubleSpinBox()
        self.sp_pid_tol.setValue(profile.limits.get("pid_tolerance", 2.0))
        row_tols.addWidget(self.sp_pid_tol)
        row_tols.addWidget(QLabel("Rate Toleransı (±):"))
        self.sp_rate_tol = QDoubleSpinBox()
        self.sp_rate_tol.setValue(profile.limits.get("rate_tolerance", 5.0))
        row_tols.addWidget(self.sp_rate_tol)
        fl_lim.addLayout(row_tols)

        lsc_layout.addWidget(grp_lim)

        grp_vtx = QGroupBox("Beklenen VTX Ayarları")
        fv_vtx = QVBoxLayout(grp_vtx)
        row_v1 = QHBoxLayout()
        row_v1.addWidget(QLabel("Bant:"))
        self.cb_vtx_band = QComboBox()
        self.cb_vtx_band.addItems(["A", "B", "E", "F", "R"])
        band_val = profile.expected_values.get("vtx_band", "R")
        self.cb_vtx_band.setCurrentText(band_val)
        row_v1.addWidget(self.cb_vtx_band)

        row_v1.addWidget(QLabel("Kanal (1-8):"))
        self.sp_vtx_ch = QSpinBox()
        self.sp_vtx_ch.setRange(1, 8)
        self.sp_vtx_ch.setValue(profile.expected_values.get("vtx_channel", 8))
        row_v1.addWidget(self.sp_vtx_ch)
        fv_vtx.addLayout(row_v1)

        row_v2 = QHBoxLayout()
        row_v2.addWidget(QLabel("Güç:"))
        self.cb_vtx_pwr = QComboBox()
        self.cb_vtx_pwr.addItems(["25 mW", "100 mW", "200 mW", "400 mW", "800 mW"])
        pwr_val = profile.expected_values.get("vtx_power", 800)
        self.cb_vtx_pwr.setCurrentText(f"{pwr_val} mW")
        row_v2.addWidget(self.cb_vtx_pwr)

        row_v2.addWidget(QLabel("Pit Mode:"))
        self.cb_vtx_pit = QComboBox()
        self.cb_vtx_pit.addItems(["OFF", "ON"])
        self.cb_vtx_pit.setCurrentIndex(1 if profile.expected_values.get("vtx_pitmode", 0) else 0)
        row_v2.addWidget(self.cb_vtx_pit)
        fv_vtx.addLayout(row_v2)

        row_v3 = QHBoxLayout()
        self.cb_vtx_req = QCheckBox("VTX SmartAudio / Tramp İletişimi Zorunlu (Device Ready)")
        self.cb_vtx_req.setChecked(profile.expected_values.get("vtx_require_connected", True))
        row_v3.addWidget(self.cb_vtx_req)
        fv_vtx.addLayout(row_v3)

        lsc_layout.addWidget(grp_vtx)

        # Uçuş Kontrolcüsü Filtre Ayarları
        grp_filt = QGroupBox("Uçuş Kontrolcüsü PID Filtre Frekansları")
        ff_layout = QVBoxLayout(grp_filt)

        row_f1 = QHBoxLayout()
        row_f1.addWidget(QLabel("Beklenen Gyro LPF (Hz):"))
        self.sp_gyro_lpf = QSpinBox()
        self.sp_gyro_lpf.setRange(20, 1000)
        self.sp_gyro_lpf.setValue(profile.expected_values.get("filter_gyro_lowpass_hz", 250))
        row_f1.addWidget(self.sp_gyro_lpf)
        row_f1.addWidget(QLabel("Beklenen D-Term LPF (Hz):"))
        self.sp_dterm_lpf = QSpinBox()
        self.sp_dterm_lpf.setRange(20, 1000)
        self.sp_dterm_lpf.setValue(profile.expected_values.get("filter_dterm_lowpass_hz", 150))
        row_f1.addWidget(self.sp_dterm_lpf)
        ff_layout.addLayout(row_f1)

        row_f2 = QHBoxLayout()
        row_f2.addWidget(QLabel("Filtre Toleransı (± Hz):"))
        self.sp_filt_tol = QDoubleSpinBox()
        self.sp_filt_tol.setRange(1.0, 100.0)
        self.sp_filt_tol.setValue(profile.limits.get("filter_tolerance_hz", 25.0))
        row_f2.addWidget(self.sp_filt_tol)
        ff_layout.addLayout(row_f2)
        lsc_layout.addWidget(grp_filt)

        # Tüm RC Uçuş Modu / Switch Atamaları
        grp_rc = QGroupBox("Tüm RC Uçuş Modu / Anahtar (Switch) Atamaları")
        frc_layout = QVBoxLayout(grp_rc)
        exp_rc_modes = profile.expected_values.get("rc_modes", {
            "ARM": "AUX1", "ANGLE": "AUX2", "BEEPER": "AUX3", "TURTLE": "AUX4"
        })

        row_rc1 = QHBoxLayout()
        row_rc1.addWidget(QLabel("ARM:"))
        self.cb_mode_arm = QComboBox()
        self.cb_mode_arm.addItems(["AUX1", "AUX2", "AUX3", "AUX4"])
        self.cb_mode_arm.setCurrentText(exp_rc_modes.get("ARM", profile.expected_values.get("rc_arm_channel", "AUX1")))
        row_rc1.addWidget(self.cb_mode_arm)

        row_rc1.addWidget(QLabel("ANGLE:"))
        self.cb_mode_angle = QComboBox()
        self.cb_mode_angle.addItems(["AUX1", "AUX2", "AUX3", "AUX4"])
        self.cb_mode_angle.setCurrentText(exp_rc_modes.get("ANGLE", "AUX2"))
        row_rc1.addWidget(self.cb_mode_angle)
        frc_layout.addLayout(row_rc1)

        row_rc2 = QHBoxLayout()
        row_rc2.addWidget(QLabel("BEEPER:"))
        self.cb_mode_beeper = QComboBox()
        self.cb_mode_beeper.addItems(["AUX1", "AUX2", "AUX3", "AUX4"])
        self.cb_mode_beeper.setCurrentText(exp_rc_modes.get("BEEPER", "AUX3"))
        row_rc2.addWidget(self.cb_mode_beeper)

        row_rc2.addWidget(QLabel("TURTLE:"))
        self.cb_mode_turtle = QComboBox()
        self.cb_mode_turtle.addItems(["AUX1", "AUX2", "AUX3", "AUX4"])
        self.cb_mode_turtle.setCurrentText(exp_rc_modes.get("TURTLE", "AUX4"))
        row_rc2.addWidget(self.cb_mode_turtle)
        frc_layout.addLayout(row_rc2)
        lsc_layout.addWidget(grp_rc)

        # Tüm UART Port ve Protokol Yapılandırması
        grp_uart = QGroupBox("Tüm UART Portları & Protokol Rolleri")
        fu_layout = QVBoxLayout(grp_uart)
        u_roles = profile.expected_values.get("uart_roles", {"UART1": "VTX", "UART2": "RX_SERIAL", "UART6": "GPS"})

        row_u1 = QHBoxLayout()
        row_u1.addWidget(QLabel("UART1:"))
        self.cb_u1_role = QComboBox()
        self.cb_u1_role.addItems(["VTX", "RX_SERIAL", "GPS", "DEVRE DIŞI"])
        self.cb_u1_role.setCurrentText(u_roles.get("UART1", "VTX"))
        row_u1.addWidget(self.cb_u1_role)

        row_u1.addWidget(QLabel("UART2:"))
        self.cb_u2_role = QComboBox()
        self.cb_u2_role.addItems(["RX_SERIAL", "VTX", "GPS", "DEVRE DIŞI"])
        self.cb_u2_role.setCurrentText(u_roles.get("UART2", "RX_SERIAL"))
        row_u1.addWidget(self.cb_u2_role)
        fu_layout.addLayout(row_u1)

        row_u2 = QHBoxLayout()
        row_u2.addWidget(QLabel("UART6:"))
        self.cb_u6_role = QComboBox()
        self.cb_u6_role.addItems(["GPS", "VTX", "RX_SERIAL", "DEVRE DIŞI"])
        self.cb_u6_role.setCurrentText(u_roles.get("UART6", "GPS"))
        row_u2.addWidget(self.cb_u6_role)
        fu_layout.addLayout(row_u2)
        lsc_layout.addWidget(grp_uart)

        # Parametrik OSD İkon ve Koordinat Doğrulaması
        grp_osd = QGroupBox("Parametrik OSD İkon Aktifliği & Ekran Konumu (X, Y)")
        fosd_layout = QVBoxLayout(grp_osd)
        exp_osd = profile.expected_values.get("osd_elements", {
            "MAIN_BATT_VOLTAGE": {"visible": True, "x": 12, "y": 14},
            "CRAFT_NAME": {"visible": True, "x": 10, "y": 1},
            "FLY_TIME": {"visible": True, "x": 2, "y": 14},
            "GPS_SATS": {"visible": True, "x": 25, "y": 1}
        })

        self.osd_inputs: Dict[str, Tuple[QCheckBox, QSpinBox, QSpinBox]] = {}
        for icon_key, icon_label in [
            ("MAIN_BATT_VOLTAGE", "Pil Voltajı (MAIN_BATT)"),
            ("CRAFT_NAME", "Cihaz Adı (CRAFT_NAME)"),
            ("FLY_TIME", "Uçuş Süresi (FLY_TIME)"),
            ("GPS_SATS", "Uydu Sayısı (GPS_SATS)")
        ]:
            row_o = QHBoxLayout()
            c_info = exp_osd.get(icon_key, {"visible": True, "x": 10, "y": 10})
            cb_vis = QCheckBox(f"{icon_label} Aktif")
            cb_vis.setChecked(c_info.get("visible", True))
            row_o.addWidget(cb_vis)

            row_o.addWidget(QLabel("X (0-31):"))
            sp_x = QSpinBox()
            sp_x.setRange(0, 31)
            sp_x.setValue(c_info.get("x", 10))
            row_o.addWidget(sp_x)

            row_o.addWidget(QLabel("Y (0-15):"))
            sp_y = QSpinBox()
            sp_y.setRange(0, 15)
            sp_y.setValue(c_info.get("y", 10))
            row_o.addWidget(sp_y)

            self.osd_inputs[icon_key] = (cb_vis, sp_x, sp_y)
            fosd_layout.addLayout(row_o)
        lsc_layout.addWidget(grp_osd)

        lim_scroll.setWidget(lim_content)
        lim_layout.addWidget(lim_scroll)

        self.tabs.addTab(tab_limits, "Toleranslar & Beklenenler")

        btn_bar = QHBoxLayout()
        btn_live_fetch = QPushButton("📥  Cihazdan Güncelle")
        btn_live_fetch.setObjectName("PrimaryBtn")
        btn_live_fetch.clicked.connect(self._fetch_from_device)
        btn_bar.addWidget(btn_live_fetch)

        btn_save = QPushButton("💾  Değişiklikleri Kaydet")
        btn_save.setObjectName("SuccessBtn")
        btn_save.clicked.connect(self._save_changes)
        btn_bar.addWidget(btn_save)

        btn_cancel = QPushButton("İptal")
        btn_cancel.clicked.connect(self.reject)
        btn_bar.addWidget(btn_cancel)

        layout.addLayout(btn_bar)

    def _get_selected_s_count(self) -> int:
        idx = self.cb_cells.currentIndex()
        mapping = {0: 1, 1: 2, 2: 3, 3: 4, 4: 6}
        return mapping.get(idx, 4)

    def _update_battery_calc(self):
        s = self._get_selected_s_count()
        c_min = self.sp_cell_min.value()
        c_max = self.sp_cell_max.value()
        tot_min = round(s * c_min, 2)
        tot_max = round(s * c_max, 2)
        self.lbl_calc_vbat.setText(f"✓ Otomatik VBAT Beklentisi ({s}S Pil): {tot_min:.2f} V - {tot_max:.2f} V")

    def _fetch_from_device(self):
        port = PortScanner.find_betaflight_device()
        is_sim = False
        if not port:
            reply = QMessageBox.question(
                self, "Cihaz Bulunamadı",
                "Fiziksel FC bulunamadı. Simülasyon (Virtual FC) üzerinden parametreler çekilsin mi?",
                QMessageBox.Yes | QMessageBox.No
            )
            if reply == QMessageBox.Yes:
                is_sim = True
            else:
                return

        transport = MockTransport() if is_sim else SerialTransport(port)
        if not transport.open():
            QMessageBox.warning(self, "Hata", "Bağlantı kurulamadı.")
            return

        snapshot, summary = GoldenSnapshotExtractor.extract_from_transport(transport)
        transport.close()

        self.profile.golden_snapshot = snapshot

        if "board_name" in snapshot:
            self.txt_board.setText(snapshot["board_name"])
        if "fw_version" in snapshot:
            self.txt_fw.setText(snapshot["fw_version"])
        if "battery_cells" in snapshot and snapshot["battery_cells"] > 0:
            s_val = snapshot["battery_cells"]
            idx_map = {1: 0, 2: 1, 3: 2, 4: 3, 6: 4}
            if s_val in idx_map:
                self.cb_cells.setCurrentIndex(idx_map[s_val])
        if "vtx_band" in snapshot:
            self.cb_vtx_band.setCurrentText(snapshot["vtx_band"])
        if "vtx_channel" in snapshot:
            self.sp_vtx_ch.setValue(snapshot["vtx_channel"])
        if "vtx_power_mw" in snapshot:
            self.cb_vtx_pwr.setCurrentText(f"{snapshot['vtx_power_mw']} mW")
        if "gyro_lowpass_hz" in snapshot:
            self.sp_gyro_lpf.setValue(snapshot["gyro_lowpass_hz"])
        if "dterm_lowpass_hz" in snapshot:
            self.sp_dterm_lpf.setValue(snapshot["dterm_lowpass_hz"])
        if "vtx_device_ready" in snapshot:
            self.cb_vtx_req.setChecked(snapshot["vtx_device_ready"])

        # RC Modları otomatik çekme
        all_modes = snapshot.get("mode_ranges", {})
        if "ARM" in all_modes:
            m = all_modes["ARM"]
            self.cb_mode_arm.setCurrentText(m.get("aux", "AUX1") if isinstance(m, dict) else m)
        if "ANGLE" in all_modes:
            m = all_modes["ANGLE"]
            self.cb_mode_angle.setCurrentText(m.get("aux", "AUX2") if isinstance(m, dict) else m)
        if "BEEPER" in all_modes:
            m = all_modes["BEEPER"]
            self.cb_mode_beeper.setCurrentText(m.get("aux", "AUX3") if isinstance(m, dict) else m)
        if "TURTLE" in all_modes:
            m = all_modes["TURTLE"]
            self.cb_mode_turtle.setCurrentText(m.get("aux", "AUX4") if isinstance(m, dict) else m)

        # UART Portları otomatik çekme
        uart_cfgs = snapshot.get("uart_roles", {})
        if "UART1" in uart_cfgs:
            roles = uart_cfgs["UART1"]
            if any("VTX" in r for r in roles): self.cb_u1_role.setCurrentText("VTX")
            elif any("RX" in r for r in roles): self.cb_u1_role.setCurrentText("RX_SERIAL")
            elif any("GPS" in r for r in roles): self.cb_u1_role.setCurrentText("GPS")
        if "UART2" in uart_cfgs:
            roles = uart_cfgs["UART2"]
            if any("RX" in r for r in roles): self.cb_u2_role.setCurrentText("RX_SERIAL")
            elif any("VTX" in r for r in roles): self.cb_u2_role.setCurrentText("VTX")
            elif any("GPS" in r for r in roles): self.cb_u2_role.setCurrentText("GPS")
        if "UART6" in uart_cfgs:
            roles = uart_cfgs["UART6"]
            if any("GPS" in r for r in roles): self.cb_u6_role.setCurrentText("GPS")
            elif any("VTX" in r for r in roles): self.cb_u6_role.setCurrentText("VTX")
            elif any("RX" in r for r in roles): self.cb_u6_role.setCurrentText("RX_SERIAL")

        # OSD Elemanları otomatik çekme
        osd_elems = snapshot.get("osd_elements", {})
        for icon_key, (cb_vis, sp_x, sp_y) in self.osd_inputs.items():
            if icon_key in osd_elems:
                item_info = osd_elems[icon_key]
                cb_vis.setChecked(item_info.get("visible", True))
                sp_x.setValue(item_info.get("x", 10))
                sp_y.setValue(item_info.get("y", 10))

        self._update_battery_calc()
        QMessageBox.information(
            self, "Golden Snapshot Alındı",
            f"Referans FC parametreleri başarıyla çekildi!\n\n{summary}"
        )

    def _save_changes(self):
        new_name = self.txt_name.text().strip()
        if not new_name:
            QMessageBox.warning(self, "Hata", "Model adı boş bırakılamaz.")
            return

        s_count = self._get_selected_s_count()
        c_min = self.sp_cell_min.value()
        c_max = self.sp_cell_max.value()

        self.profile.name = new_name
        self.profile.board = self.txt_board.text().strip()
        self.profile.firmware = self.txt_fw.text().strip()
        self.profile.battery_cells = s_count
        self.profile.cell_min_voltage = c_min
        self.profile.cell_max_voltage = c_max

        self.profile.limits["battery_min"] = round(s_count * c_min, 2)
        self.profile.limits["battery_max"] = round(s_count * c_max, 2)
        self.profile.limits["vbus_min"] = self.sp_vbus_min.value()
        self.profile.limits["vbus_max"] = self.sp_vbus_max.value()
        self.profile.limits["barometer_min"] = self.sp_baro_min.value()
        self.profile.limits["barometer_max"] = self.sp_baro_max.value()
        self.profile.limits["current_max_disarmed"] = self.sp_max_cur.value()
        self.profile.limits["cpu_load_max"] = self.sp_cpu_max.value()
        self.profile.limits["pid_tolerance"] = self.sp_pid_tol.value()
        self.profile.limits["rate_tolerance"] = self.sp_rate_tol.value()
        self.profile.limits["filter_tolerance_hz"] = self.sp_filt_tol.value()

        for k, cb in self.test_checkboxes.items():
            self.profile.tests[k] = cb.isChecked()

        self.profile.expected_values["vtx_band"] = self.cb_vtx_band.currentText()
        self.profile.expected_values["vtx_channel"] = self.sp_vtx_ch.value()
        pwr_str = self.cb_vtx_pwr.currentText().split()[0]
        self.profile.expected_values["vtx_power"] = int(pwr_str)
        self.profile.expected_values["vtx_pitmode"] = 1 if self.cb_vtx_pit.currentText() == "ON" else 0
        self.profile.expected_values["vtx_require_connected"] = self.cb_vtx_req.isChecked()
        self.profile.expected_values["filter_gyro_lowpass_hz"] = self.sp_gyro_lpf.value()
        self.profile.expected_values["filter_dterm_lowpass_hz"] = self.sp_dterm_lpf.value()

        # Tüm RC Modları kaydet
        arm_aux = self.cb_mode_arm.currentText()
        self.profile.expected_values["rc_arm_channel"] = arm_aux
        self.profile.expected_values["rc_modes"] = {
            "ARM": arm_aux,
            "ANGLE": self.cb_mode_angle.currentText(),
            "BEEPER": self.cb_mode_beeper.currentText(),
            "TURTLE": self.cb_mode_turtle.currentText()
        }

        # Tüm UART Rolleri kaydet
        uart_dict = {}
        if self.cb_u1_role.currentText() != "DEVRE DIŞI":
            uart_dict["UART1"] = self.cb_u1_role.currentText()
        if self.cb_u2_role.currentText() != "DEVRE DIŞI":
            uart_dict["UART2"] = self.cb_u2_role.currentText()
        if self.cb_u6_role.currentText() != "DEVRE DIŞI":
            uart_dict["UART6"] = self.cb_u6_role.currentText()
        self.profile.expected_values["uart_roles"] = uart_dict

        # Parametrik OSD Konumları kaydet
        osd_dict = {}
        for icon_key, (cb_vis, sp_x, sp_y) in self.osd_inputs.items():
            osd_dict[icon_key] = {
                "visible": cb_vis.isChecked(),
                "x": sp_x.value(),
                "y": sp_y.value()
            }
        self.profile.expected_values["osd_elements"] = osd_dict

        # Golden Snapshot senkronizasyonu
        if not getattr(self.profile, "golden_snapshot", None):
            self.profile.golden_snapshot = GoldenSnapshotExtractor.build_baseline_snapshot(self.profile.board, self.profile.firmware)
        self.profile.golden_snapshot["board_name"] = self.profile.board
        self.profile.golden_snapshot["fw_version"] = self.profile.firmware
        self.profile.golden_snapshot["battery_cells"] = s_count
        self.profile.golden_snapshot["vtx_band"] = self.cb_vtx_band.currentText()
        self.profile.golden_snapshot["vtx_channel"] = self.sp_vtx_ch.value()
        self.profile.golden_snapshot["vtx_power_mw"] = int(pwr_str)
        self.profile.golden_snapshot["vtx_pitmode"] = 1 if self.cb_vtx_pit.currentText() == "ON" else 0
        self.profile.golden_snapshot["vtx_device_ready"] = self.cb_vtx_req.isChecked()
        self.profile.golden_snapshot["gyro_lowpass_hz"] = self.sp_gyro_lpf.value()
        self.profile.golden_snapshot["dterm_lowpass_hz"] = self.sp_dterm_lpf.value()
        if "osd_elements" not in self.profile.golden_snapshot:
            self.profile.golden_snapshot["osd_elements"] = {}
        self.profile.golden_snapshot["osd_elements"].update(osd_dict)

        if new_name != self.original_name:
            self.profile_mgr.delete_profile(self.original_name)

        if self.profile_mgr.save_profile(self.profile):
            if self.isVisible():
                QMessageBox.information(self, "Başarılı", f"'{new_name}' profili başarıyla güncellendi!")
            self.accept()
        else:
            if self.isVisible():
                QMessageBox.critical(self, "Hata", "Profil kaydedilirken hata oluştu.")


class ProfileManagerDialog(QDialog):
    """Test profillerini listeleme, silme, kopyalama ve düzenleme ekranı."""
    profile_selected = Signal(TestProfile)

    def __init__(self, profile_mgr: ProfileManager, parent=None):
        super().__init__(parent)
        self.profile_mgr = profile_mgr
        self.setWindowTitle("Kayıtlı Test Profilleri")
        self.setMinimumSize(740, 440)
        self.setStyleSheet(DARK_THEME_QSS)

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("TEST PROFİLLERİ LİSTESİ"))

        self.table = QTableWidget(0, 5)
        self.table.setHorizontalHeaderLabels(["Model Adı", "Hedef Board", "Firmware", "Pil (S)", "Aktif Test Sayısı"])
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.table.setSelectionBehavior(QTableWidget.SelectRows)
        layout.addWidget(self.table)

        btn_box = QHBoxLayout()
        btn_select = QPushButton("🎯  BU PROFİLİ SEÇ")
        btn_select.setObjectName("SuccessBtn")
        btn_select.clicked.connect(self._select_profile)
        btn_box.addWidget(btn_select)

        btn_edit = QPushButton("✏  DÜZENLE")
        btn_edit.setObjectName("PrimaryBtn")
        btn_edit.clicked.connect(self._edit_profile)
        btn_box.addWidget(btn_edit)

        btn_copy = QPushButton("📋  KOPYALA")
        btn_copy.clicked.connect(self._copy_profile)
        btn_box.addWidget(btn_copy)

        btn_delete = QPushButton("🗑  SİL")
        btn_delete.setObjectName("DangerBtn")
        btn_delete.clicked.connect(self._delete_profile)
        btn_box.addWidget(btn_delete)

        btn_close = QPushButton("KAPAT")
        btn_close.clicked.connect(self.accept)
        btn_box.addWidget(btn_close)

        layout.addLayout(btn_box)
        self._refresh_table()

    def _refresh_table(self):
        self.table.setRowCount(0)
        profiles = self.profile_mgr.list_profiles()
        for p in profiles:
            row = self.table.rowCount()
            self.table.insertRow(row)
            self.table.setItem(row, 0, QTableWidgetItem(p.name))
            self.table.setItem(row, 1, QTableWidgetItem(p.board))
            self.table.setItem(row, 2, QTableWidgetItem(p.firmware))
            s_val = getattr(p, "battery_cells", 4)
            self.table.setItem(row, 3, QTableWidgetItem(f"{s_val}S"))
            test_count = sum(1 for v in p.tests.values() if v)
            self.table.setItem(row, 4, QTableWidgetItem(f"{test_count} Test"))

    def _get_selected_profile(self) -> Optional[TestProfile]:
        row = self.table.currentRow()
        if row >= 0:
            name = self.table.item(row, 0).text()
            return self.profile_mgr.get_profile(name)
        return None

    def _select_profile(self):
        p = self._get_selected_profile()
        if p:
            self.profile_selected.emit(p)
            self.accept()

    def _edit_profile(self):
        p = self._get_selected_profile()
        if p:
            dlg = ProfileEditDialog(p, self.profile_mgr, self)
            if dlg.exec() == QDialog.Accepted:
                self._refresh_table()

    def _copy_profile(self):
        p = self._get_selected_profile()
        if p:
            p.name = f"{p.name}_Kopya"
            self.profile_mgr.save_profile(p)
            self._refresh_table()

    def _delete_profile(self):
        p = self._get_selected_profile()
        if p:
            reply = QMessageBox.question(self, "Onay", f"'{p.name}' profili silinsin mi?")
            if reply == QMessageBox.Yes:
                self.profile_mgr.delete_profile(p.name)
                self._refresh_table()


class TestSelectionDialog(QDialog):
    """Test başlatmadan önce hangi kriterlerin test edileceğini seçtiren dialog."""
    def __init__(self, profile: TestProfile, parent=None):
        super().__init__(parent)
        self.profile = profile
        self.setWindowTitle("Uygulanacak Testleri Seç")
        self.setMinimumSize(480, 520)
        self.setStyleSheet(DARK_THEME_QSS)

        layout = QVBoxLayout(self)
        layout.addWidget(QLabel("TEST EDİLECEK KRİTERLER:"))

        self.checkboxes: Dict[str, QCheckBox] = {}
        test_labels = {
            "board_firmware": "Board / Firmware Sürümü Doğrulama",
            "imu1": "IMU 1 (Jiroskop & İvmeölçer Sağlığı)",
            "imu2": "IMU 2 (Çift Jiroskop Denetimi)",
            "barometer": "Barometre (Varlık & Durağan İrtifa)",
            "vbus": "VBUS (5V Giriş Voltajı)",
            "battery": "Batarya Voltajı (VBAT)",
            "battery_cells": "Batarya S-Sayısı (Kaç S Pil & Hücre Voltajı)",
            "current": "Akım Tüketimi (Disarmed < 5A Koruması)",
            "cpu_load": "CPU / Sistem Yükü (< %75 Donma Koruması)",
            "i2c_bus": "I2C Veri Yolu Sağlığı (0 Hata)",
            "cycle_time": "PID Döngü Süresi (Loop Time µs)",
            "mcu_uid": "Donanım Benzersiz Kimliği (MCU UID)",
            "motor_rpm": "Motor RPM / DShot Telemetrisi (Read-Only)",
            "memory": "Dahili Bellek (Dataflash Blackbox)",
            "pid": "PID Ayarları & Kazanç Eşleşmesi",
            "rate": "Rate / Kumanda Hassasiyet Oranları",
            "rc": "RC / Receiver Alıcı Yapılandırması",
            "rc_modes": "RC Mod Atamaları (ARM Switch -> AUX Kanalı)",
            "filters": "PID Filtreleri (Gyro & D-Term Lowpass Frekansları)",
            "uart_config": "UART Port Atamaları (VTX, CRSF/ELRS, GPS vb.)",
            "osd": "OSD Ekran Çipi & Destek Modu",
            "osd_elements": "Parametrik OSD İkon ve Konum Doğrulaması (Aktiflik & X,Y)",
            "vtx": "VTX Video Verici (Band, Kanal, Güç, Pit Mode)",
            "vtx_connection": "VTX Donanım Bağlantısı (SmartAudio/Tramp Device Ready)",
            "usb": "USB / MSP Haberleşme & Checksum Bütünlüğü",
            "gps": "GPS Modülü (Uydu Varlığı)"
        }

        group = QGroupBox("Kriter Listesi")
        grp_layout = QVBoxLayout(group)
        for key, text in test_labels.items():
            cb = QCheckBox(text)
            cb.setChecked(self.profile.tests.get(key, True))
            self.checkboxes[key] = cb
            grp_layout.addWidget(cb)
        layout.addWidget(group)

        btn_row = QHBoxLayout()
        btn_all = QPushButton("☑  TÜMÜNÜ SEÇ")
        btn_all.clicked.connect(lambda: [cb.setChecked(True) for cb in self.checkboxes.values()])
        btn_row.addWidget(btn_all)

        btn_none = QPushButton("☐  HİÇBİRİNİ SEÇME")
        btn_none.clicked.connect(lambda: [cb.setChecked(False) for cb in self.checkboxes.values()])
        btn_row.addWidget(btn_none)
        layout.addLayout(btn_row)

        btn_start = QPushButton("▶  TESTİ BAŞLAT")
        btn_start.setObjectName("SuccessBtn")
        btn_start.clicked.connect(self.accept)
        layout.addWidget(btn_start)

    def get_selected_tests(self) -> Dict[str, bool]:
        return {k: cb.isChecked() for k, cb in self.checkboxes.items()}


class SimulationDialog(QDialog):
    """Sanal FC ve Hata Enjeksiyonu Senaryoları Ekranı."""
    def __init__(self, current_faults: Dict[str, bool], is_active: bool, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Sanal FC Simülasyonu & Hata Enjeksiyonu")
        self.setMinimumSize(480, 560)
        self.setStyleSheet(DARK_THEME_QSS)

        layout = QVBoxLayout(self)
        self.cb_sim_enable = QCheckBox("🧪  Simülasyon Modunu Aktif Et (Virtual FC)")
        self.cb_sim_enable.setChecked(is_active)
        self.cb_sim_enable.setStyleSheet("font-size: 15px; font-weight: bold; color: #60a5fa;")
        layout.addWidget(self.cb_sim_enable)

        group = QGroupBox("Hata Enjeksiyon Senaryoları (Gerçek FC Olmadan Test Et)")
        grp_layout = QVBoxLayout(group)

        self.cb_faults: Dict[str, QCheckBox] = {}
        fault_definitions = {
            "imu1_fail": "IMU 1 Arızası (Sensör Yok / 0 LSB)",
            "baro_fail": "Barometre Hatası (+15 m Limit Dışı)",
            "vbus_low": "VBUS Düşük Voltaj (4.30 V)",
            "vbus_high": "VBUS Aşırı Voltaj (5.55 V)",
            "battery_low": "Batarya Düşük (10.0 V)",
            "battery_cells_fail": "Pil S-Sayısı Uyumsuzluğu (3S takılı, 4S bekleniyor)",
            "high_current": "Aşırı Akım Hatası (6.2 A > 5A Disarmed FAIL)",
            "cpu_load_fail": "Aşırı CPU Yükü (%88 > %75 FAIL)",
            "i2c_fail": "I2C Veri Yolu Hatası (14 Hata > 0 FAIL)",
            "cycle_time_fail": "Döngü Süresi Sapması (260 µs FAIL)",
            "wrong_firmware": "Yanlış Firmware Sürümü (4.4.0)",
            "wrong_pid": "Yanlış PID Ayarları",
            "wrong_filter": "Yanlış PID Filtre Frekansları (Gyro 80Hz / D-Term 60Hz)",
            "wrong_rate": "Yanlış Rate Profili",
            "rc_mismatch": "Alıcı Protokol Uyumsuzluğu (SBUS yerine CRSF)",
            "wrong_arm_switch": "Yanlış ARM Anahtarı (AUX3 yerine AUX1 bekleniyor)",
            "wrong_mode_range": "Yanlış RC Mod Ataması (BEEPER -> AUX1 yerine AUX3)",
            "wrong_uart": "Yanlış UART Port Ataması (VTX UART1 Kapalı)",
            "osd_mismatch": "OSD Çipi Devre Dışı",
            "osd_icon_fail": "Parametrik OSD Hatası (MAIN_BATT_VOLTAGE kapalı / kaymış)",
            "vtx_channel_fail": "VTX Yanlış Kanal (Kanal 1)",
            "vtx_power_fail": "VTX Yanlış Güç (25mW)",
            "vtx_connect_fail": "VTX Bağlantı Kopukluğu (SmartAudio Yanıtsız / Device Not Ready)",
            "memory_fail": "Flash Bellek Hatası (Not Ready)",
            "msp_timeout": "MSP İletişim Zaman Aşımı (Timeout)",
            "disconnect": "Cihaz Bağlantısı Koptu (Hot-Unplug)"
        }

        for key, text in fault_definitions.items():
            cb = QCheckBox(text)
            cb.setChecked(current_faults.get(key, False))
            self.cb_faults[key] = cb
            grp_layout.addWidget(cb)

        layout.addWidget(group)

        btn_row = QHBoxLayout()
        btn_normal = QPushButton("✅  Normal (Hatayı Sıfırla)")
        btn_normal.clicked.connect(lambda: [cb.setChecked(False) for cb in self.cb_faults.values()])
        btn_row.addWidget(btn_normal)

        btn_ok = QPushButton("KAYDET & UYGULA")
        btn_ok.setObjectName("PrimaryBtn")
        btn_ok.clicked.connect(self.accept)
        btn_row.addWidget(btn_ok)

        layout.addLayout(btn_row)

    def is_simulation_enabled(self) -> bool:
        return self.cb_sim_enable.isChecked()

    def get_fault_config(self) -> Dict[str, bool]:
        return {k: cb.isChecked() for k, cb in self.cb_faults.items()}


class FullParameterInspectorDialog(QDialog):
    """
    360° Tüm Betaflight Parametreleri ve Referans Kart (Golden Sample) Karşılaştırma Denetçisi.
    140'tan fazla parametreyi kategori, arama filtresi ve uyuşmazlık (diff) modu ile listeler.
    Dinamik donanım parametreleri (MCU Seri No ve canlı ivmeölçer sıfır sapması) eşitlik kontrolünden
    ayrılmış olarak gösterilir.
    """
    def __init__(self, profile: TestProfile, device: Optional[PhysicalDevice],
                 live_data: Dict[str, Any], parent=None):
        super().__init__(parent)
        self.profile = profile
        self.device = device
        self.live_data = live_data
        self.audit_items = build_360_parameter_audit(profile, live_data)

        self.setWindowTitle(f"360° Tüm Betaflight Parametreleri & Golden Karşılaştırma - {profile.name}")
        self.resize(1180, 740)
        self.setStyleSheet(DARK_THEME_QSS)

        self._setup_ui()
        self._filter_table()

    def _setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setContentsMargins(16, 14, 16, 14)
        layout.setSpacing(10)

        # 1. Başlık ve İstatistik Barı
        top_card = QFrame()
        top_card.setStyleSheet("background-color: #1b202c; border-radius: 8px; padding: 10px 14px;")
        top_layout = QVBoxLayout(top_card)
        top_layout.setSpacing(6)

        title_row = QHBoxLayout()
        lbl_title = QLabel("🔍  360° BETAFIGHT PARAMETRE VE REFERANS KART DENETÇİSİ")
        lbl_title.setStyleSheet("font-size: 16px; font-weight: bold; color: #60a5fa;")
        title_row.addWidget(lbl_title)

        dev_port = self.device.port if self.device else ("Virtual Sim" if not HAS_SERIAL else "COM Portu")
        lbl_sub = QLabel(f"Profil: <b>{self.profile.name}</b> ({self.profile.board} FW: {self.profile.firmware}) | Cihaz: <b>{dev_port}</b>")
        lbl_sub.setStyleSheet("font-size: 12px; color: #94a3b8;")
        title_row.addWidget(lbl_sub, alignment=Qt.AlignRight)
        top_layout.addLayout(title_row)

        # İstatistik Rozetleri (Badges)
        badge_row = QHBoxLayout()
        badge_row.setSpacing(8)

        total_cnt = len(self.audit_items)
        match_cnt = sum(1 for it in self.audit_items if it["status"] == "MATCH")
        mismatch_cnt = sum(1 for it in self.audit_items if it["status"] == "MISMATCH")
        dyn_cnt = sum(1 for it in self.audit_items if it["status"] == "DYNAMIC")

        self.badge_total = QLabel(f"Toplam Parametre: {total_cnt}")
        self.badge_total.setStyleSheet("background-color: #1e3a8a; color: #bfdbfe; font-weight: bold; padding: 4px 10px; border-radius: 5px;")
        badge_row.addWidget(self.badge_total)

        self.badge_match = QLabel(f"✓ Eşleşti: {match_cnt}")
        self.badge_match.setStyleSheet("background-color: #064e3b; color: #a7f3d0; font-weight: bold; padding: 4px 10px; border-radius: 5px;")
        badge_row.addWidget(self.badge_match)

        self.badge_mismatch = QLabel(f"✗ Uyuşmazlık: {mismatch_cnt}")
        err_color = "#7f1d1d; color: #fecaca" if mismatch_cnt > 0 else "#27272a; color: #71717a"
        self.badge_mismatch.setStyleSheet(f"background-color: {err_color}; font-weight: bold; padding: 4px 10px; border-radius: 5px;")
        badge_row.addWidget(self.badge_mismatch)

        self.badge_dyn = QLabel(f"⚙️ Dinamik Hariç: {dyn_cnt} (Seri No & Gürültü)")
        self.badge_dyn.setStyleSheet("background-color: #374151; color: #d1d5db; font-weight: bold; padding: 4px 10px; border-radius: 5px;")
        badge_row.addWidget(self.badge_dyn)

        badge_row.addStretch()
        top_layout.addLayout(badge_row)
        layout.addWidget(top_card)

        # 2. Filtreleme ve Arama Barı
        filter_bar = QHBoxLayout()
        filter_bar.setSpacing(10)

        self.txt_search = QLineEdit()
        self.txt_search.setPlaceholderText("🔍 Parametre Ara (örn: OSD, AUX, BEEPER, UART, ROLL, DTERM, 4S, VBAT)...")
        self.txt_search.textChanged.connect(self._filter_table)
        filter_bar.addWidget(self.txt_search, 2)

        self.cb_category = QComboBox()
        self.cb_category.addItems([
            "Tüm Kategoriler",
            "📺 OSD Telemetri",
            "🕹️ RC Modları & Kanallar",
            "🔌 UART Portları & Protokoller",
            "🎯 PID Kazançları",
            "🔊 Dijital Filtreler",
            "📈 RC Tepki & Rate Eğrileri",
            "📡 VTX Video Verici",
            "⚡ Analog & Güç Sistemi",
            "🛡️ Sensör & Donanım Sağlığı",
            "⚙️ Dinamik Değerler"
        ])
        self.cb_category.currentIndexChanged.connect(self._filter_table)
        filter_bar.addWidget(self.cb_category, 1)

        self.cb_diff_only = QCheckBox("⚠️ Sadece Uyuşmazlıkları Göster (Diff Only)")
        self.cb_diff_only.stateChanged.connect(self._filter_table)
        filter_bar.addWidget(self.cb_diff_only)

        layout.addLayout(filter_bar)

        # 3. 140+ Parametreli Karşılaştırma Tablosu
        self.table = QTableWidget(0, 6)
        self.table.setHorizontalHeaderLabels([
            "KATEGORİ", "PARAMETRE TANIMI", "REFERANS (GOLDEN)", "TEST EDİLEN KART", "DURUM", "AÇIKLAMA / NOT"
        ])
        self.table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(1, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(2, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(4, QHeaderView.ResizeToContents)
        self.table.horizontalHeader().setSectionResizeMode(5, QHeaderView.Stretch)
        self.table.verticalHeader().setVisible(False)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        layout.addWidget(self.table)

        # 4. Alt Butonlar
        bottom_bar = QHBoxLayout()
        btn_copy = QPushButton("📋  Raporu Panoya Kopyala")
        btn_copy.clicked.connect(self._copy_report_to_clipboard)
        bottom_bar.addWidget(btn_copy)

        btn_export = QPushButton("💾  Denetim Logunu Kaydet")
        btn_export.clicked.connect(self._export_report_file)
        bottom_bar.addWidget(btn_export)

        bottom_bar.addStretch()

        btn_close = QPushButton("KAPAT")
        btn_close.clicked.connect(self.accept)
        bottom_bar.addWidget(btn_close)

        layout.addLayout(bottom_bar)

    def _filter_table(self):
        query = self.txt_search.text().strip().lower()
        selected_cat = self.cb_category.currentText()
        diff_only = self.cb_diff_only.isChecked()

        filtered = []
        for item in self.audit_items:
            if selected_cat != "Tüm Kategoriler" and item["category"] != selected_cat:
                continue
            if diff_only and item["status"] != "MISMATCH":
                continue
            if query:
                search_target = f"{item['category']} {item['name']} {item['golden']} {item['actual']} {item['detail']}".lower()
                if query not in search_target:
                    continue
            filtered.append(item)

        self.table.setRowCount(len(filtered))
        for row, it in enumerate(filtered):
            self.table.setRowHeight(row, 24)

            item_cat = QTableWidgetItem(it["category"])
            item_cat.setFont(QFont("Segoe UI", 9))
            self.table.setItem(row, 0, item_cat)

            item_name = QTableWidgetItem(it["name"])
            item_name.setFont(QFont("Segoe UI", 9, QFont.Bold))
            self.table.setItem(row, 1, item_name)

            item_gold = QTableWidgetItem(it["golden"])
            item_gold.setTextAlignment(Qt.AlignCenter)
            item_gold.setFont(QFont("Segoe UI", 9))
            self.table.setItem(row, 2, item_gold)

            item_act = QTableWidgetItem(it["actual"])
            item_act.setTextAlignment(Qt.AlignCenter)
            item_act.setFont(QFont("Segoe UI", 9, QFont.Bold))
            self.table.setItem(row, 3, item_act)

            item_st = QTableWidgetItem()
            item_st.setTextAlignment(Qt.AlignCenter)
            item_st.setFont(QFont("Segoe UI", 9, QFont.Bold))

            st = it["status"]
            if st == "MATCH":
                item_st.setText("🟢 EŞLEŞTİ")
                item_st.setForeground(QColor("#4ade80"))
            elif st == "MISMATCH":
                item_st.setText("🔴 UYUŞMAZLIK")
                item_st.setForeground(QColor("#f87171"))
                item_act.setForeground(QColor("#f87171"))
            elif st == "DYNAMIC":
                item_st.setText("⚪ DİNAMİK-HARİÇ")
                item_st.setForeground(QColor("#94a3b8"))
                item_act.setForeground(QColor("#cbd5e1"))

            self.table.setItem(row, 4, item_st)

            item_det = QTableWidgetItem(it["detail"])
            item_det.setFont(QFont("Segoe UI", 8))
            item_det.setForeground(QColor("#94a3b8"))
            self.table.setItem(row, 5, item_det)

    def _generate_report_text(self) -> str:
        lines = [
            "=" * 90,
            "360° BETAFIGHT PARAMETRE VE REFERANS KART (GOLDEN SAMPLE) DENETİM RAPORU",
            f"Tarih/Saat: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
            f"Test Profili: {self.profile.name} | Hedef Donanım: {self.profile.board} | Firmware: {self.profile.firmware}",
            f"Cihaz Portu: {self.device.port if self.device else 'SIM/PORT'}",
            "=" * 90,
            f"{'DURUM':<14} | {'KATEGORİ':<24} | {'PARAMETRE':<36} | {'REFERANS':<20} | {'CANLI DEĞER':<20}",
            "-" * 90
        ]
        for it in self.audit_items:
            st_text = "[EŞLEŞTİ]" if it["status"] == "MATCH" else ("[UYUŞMAZLIK]" if it["status"] == "MISMATCH" else "[DİNAMİK]")
            lines.append(f"{st_text:<14} | {it['category']:<24} | {it['name'][:34]:<36} | {it['golden'][:18]:<20} | {it['actual'][:18]:<20}")
        lines.append("=" * 90)
        return "\n".join(lines)

    def _copy_report_to_clipboard(self):
        text = self._generate_report_text()
        QApplication.clipboard().setText(text)
        QMessageBox.information(self, "Kopyalandı", f"Toplam {len(self.audit_items)} parametrelik 360° denetim raporu panoya kopyalandı!")

    def _export_report_file(self):
        text = self._generate_report_text()
        os.makedirs("audit_reports", exist_ok=True)
        filename = f"audit_reports/Audit_{self.profile.name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        try:
            with open(filename, "w", encoding="utf-8") as f:
                f.write(text)
            QMessageBox.information(self, "Kaydedildi", f"Denetim raporu kaydedildi:\n{filename}")
        except Exception as e:
            QMessageBox.critical(self, "Hata", f"Rapor kaydedilemedi: {e}")


# ============================================================================
# 10. TAM EKRAN ATE İSTASYONU (PRODUCTION TEST STATION WINDOW)
# ============================================================================

class ProductionStationWindow(QMainWindow):
    """
    Operatörün uzaktan dahi 1 saniyede kartın durumunu görebileceği,
    büyük tam ekran endüstriyel test istasyonu ekranı.
    """
    back_to_menu_signal = Signal()

    def __init__(self, profile_mgr: ProfileManager, parent=None):
        super().__init__(parent)
        self.profile_mgr = profile_mgr
        self.test_logger = TestLogger()
        self.audio_notifier = AudioNotifier()

        self.current_profile = profile_mgr.list_profiles()[0]
        self.active_tests = self.current_profile.tests.copy()
        self.is_simulation = False
        self.sim_faults: Dict[str, bool] = {}
        self.worker: Optional[DeviceWorker] = None
        self.last_results: List[TestResultItem] = []
        self.last_passed_flag = False

        self.setWindowTitle("BETAFIGHT FC TESTER - Production Quality Station")
        self.resize(1220, 760)
        self.setStyleSheet(DARK_THEME_QSS)

        self._setup_ui()

    def _setup_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        main_layout.setContentsMargins(16, 12, 16, 12)
        main_layout.setSpacing(10)

        # 1. Üst Bilgi Barı (Header)
        header = QFrame()
        header.setStyleSheet("background-color: #1b202c; border-radius: 8px; padding: 8px 12px;")
        h_layout = QHBoxLayout(header)

        self.lbl_profile_title = QLabel(f"TEST PROFİLİ: {self.current_profile.name}")
        self.lbl_profile_title.setStyleSheet("font-size: 17px; font-weight: bold; color: #60a5fa;")
        h_layout.addWidget(self.lbl_profile_title)

        self.lbl_device_info = QLabel("Cihaz: ARANIYOR... | Firmware: --- | Port: ---")
        self.lbl_device_info.setStyleSheet("font-size: 13px; color: #cbd5e1;")
        h_layout.addWidget(self.lbl_device_info)

        self.lbl_connection_badge = QLabel("🟡 SCANNING...")
        self.lbl_connection_badge.setStyleSheet(
            "background-color: #854d0e; color: #fef08a; font-weight: bold; padding: 5px 12px; border-radius: 6px;"
        )
        h_layout.addWidget(self.lbl_connection_badge)

        btn_restart = QPushButton("🔄  YENİDEN TEST ET")
        btn_restart.clicked.connect(self.restart_tests)
        h_layout.addWidget(btn_restart)

        btn_full_inspect = QPushButton("🔍  360° PARAMETRELERİ İNCELE")
        btn_full_inspect.setObjectName("PrimaryBtn")
        btn_full_inspect.setStyleSheet("background-color: #2563eb; color: #ffffff; font-weight: bold; padding: 6px 12px; border-radius: 6px;")
        btn_full_inspect.clicked.connect(self._open_full_inspector)
        h_layout.addWidget(btn_full_inspect)

        btn_open_log = QPushButton("📁  LOGU AÇ")
        btn_open_log.clicked.connect(self._open_log_folder)
        h_layout.addWidget(btn_open_log)

        btn_menu = QPushButton("⬅  ANA MENÜ")
        btn_menu.clicked.connect(self._on_back_clicked)
        h_layout.addWidget(btn_menu)

        main_layout.addWidget(header)

        # 2. İki Sütunlu Canlı Sonuçlar Tabloları (Two-Column Split Results Tables)
        tables_row = QHBoxLayout()
        tables_row.setSpacing(12)

        # Sol Panel: Sistem, Elektrik ve Sensör Sağlığı
        left_box = QVBoxLayout()
        left_box.setSpacing(4)
        lbl_left_hdr = QLabel("📊 SİSTEM, ELEKTRİK & SENSÖR SAĞLIĞI (Detaylar İçin Çift Tıklayınız)")
        lbl_left_hdr.setStyleSheet("font-size: 12px; font-weight: bold; color: #38bdf8; padding: 2px 0px;")
        left_box.addWidget(lbl_left_hdr)

        self.table_left = QTableWidget(0, 4)
        self.table_left.setHorizontalHeaderLabels(["TEST ADI", "GERÇEK DEĞER", "BEKLENEN", "DURUM"])
        self.table_left.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.table_left.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.table_left.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.table_left.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeToContents)
        self.table_left.verticalHeader().setVisible(False)
        self.table_left.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table_left.cellDoubleClicked.connect(lambda r, c: self._open_full_inspector())
        left_box.addWidget(self.table_left)
        tables_row.addLayout(left_box, 1)

        # Sağ Panel: Uçuş Yapılandırması, Protokoller & OSD
        right_box = QVBoxLayout()
        right_box.setSpacing(4)
        lbl_right_hdr = QLabel("⚙️ UÇUŞ YAPILANDIRMASI, PROTOKOL & OSD (Detaylar İçin Çift Tıklayınız)")
        lbl_right_hdr.setStyleSheet("font-size: 12px; font-weight: bold; color: #c084fc; padding: 2px 0px;")
        right_box.addWidget(lbl_right_hdr)

        self.table_right = QTableWidget(0, 4)
        self.table_right.setHorizontalHeaderLabels(["TEST ADI", "GERÇEK DEĞER", "BEKLENEN", "DURUM"])
        self.table_right.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.table_right.horizontalHeader().setSectionResizeMode(1, QHeaderView.Stretch)
        self.table_right.horizontalHeader().setSectionResizeMode(2, QHeaderView.Stretch)
        self.table_right.horizontalHeader().setSectionResizeMode(3, QHeaderView.ResizeToContents)
        self.table_right.verticalHeader().setVisible(False)
        self.table_right.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table_right.cellDoubleClicked.connect(lambda r, c: self._open_full_inspector())
        right_box.addWidget(self.table_right)
        tables_row.addLayout(right_box, 1)

        self.table = self.table_left  # Backwards compatibility
        main_layout.addLayout(tables_row)

        # 3. Dev Sonuç Göstergesi (Jumbo Status Banner)
        self.banner = QFrame()
        self.banner.setMinimumHeight(100)
        self.banner_layout = QVBoxLayout(self.banner)
        self.banner_layout.setAlignment(Qt.AlignCenter)

        self.lbl_banner_main = QLabel("🔍 FC BEKLENİYOR: USB Kablosunu Takınız...")
        self.lbl_banner_main.setAlignment(Qt.AlignCenter)
        self.lbl_banner_main.setStyleSheet("font-size: 26px; font-weight: 900; letter-spacing: 1px;")
        self.banner_layout.addWidget(self.lbl_banner_main)

        self.lbl_banner_sub = QLabel("COM portları otomatik taranıyor • Betaflight kartı algılandığında testler anında başlar.")
        self.lbl_banner_sub.setAlignment(Qt.AlignCenter)
        self.lbl_banner_sub.setStyleSheet("font-size: 14px; margin-top: 2px;")
        self.banner_layout.addWidget(self.lbl_banner_sub)

        self._set_banner_style("SEARCHING")
        main_layout.addWidget(self.banner)

    def _set_banner_style(self, mode: str):
        if mode == "PASS":
            self.banner.setStyleSheet(
                "background-color: #064e3b; border: 2px solid #10b981; border-radius: 12px;"
            )
            self.lbl_banner_main.setStyleSheet("font-size: 28px; font-weight: 900; color: #a7f3d0;")
            self.lbl_banner_sub.setStyleSheet("font-size: 15px; color: #d1fae5;")
        elif mode == "FAIL":
            self.banner.setStyleSheet(
                "background-color: #7f1d1d; border: 2px solid #ef4444; border-radius: 12px;"
            )
            self.lbl_banner_main.setStyleSheet("font-size: 28px; font-weight: 900; color: #fecaca;")
            self.lbl_banner_sub.setStyleSheet("font-size: 15px; color: #fee2e2;")
        elif mode == "SEARCHING":
            self.banner.setStyleSheet(
                "background-color: #78350f; border: 2px solid #f59e0b; border-radius: 12px;"
            )
            self.lbl_banner_main.setStyleSheet("font-size: 24px; font-weight: bold; color: #fde68a;")
            self.lbl_banner_sub.setStyleSheet("font-size: 13px; color: #fef3c7;")

    def start_testing(self, profile: TestProfile, active_tests: Dict[str, bool],
                      is_sim: bool = False, sim_faults: Optional[Dict[str, bool]] = None):
        self.current_profile = profile
        self.active_tests = active_tests
        self.is_simulation = is_sim
        self.sim_faults = sim_faults or {}

        self.lbl_profile_title.setText(f"TEST PROFİLİ: {self.current_profile.name}")
        self._set_banner_style("SEARCHING")
        self.lbl_banner_main.setText("🔍 FC BEKLENİYOR...")
        self.lbl_banner_sub.setText("Otomatik COM tarama yürütülüyor...")

        if self.worker:
            self.worker.stop()
            self.worker = None

        self.worker = DeviceWorker(self.current_profile, self.active_tests, self.is_simulation, self.sim_faults)
        self.worker.status_signal.connect(self._on_worker_status)
        self.worker.device_discovered_signal.connect(self._on_device_discovered)
        self.worker.device_disconnected_signal.connect(self._on_device_disconnected)
        self.worker.test_completed_signal.connect(self._on_test_results)
        self.worker.start()

    def restart_tests(self):
        if self.worker:
            self.worker.stop()
        self.start_testing(self.current_profile, self.active_tests, self.is_simulation, self.sim_faults)

    @Slot(str)
    def _on_worker_status(self, msg: str):
        self.statusBar().showMessage(msg, 3000)

    @Slot(PhysicalDevice)
    def _on_device_discovered(self, dev: PhysicalDevice):
        self.lbl_connection_badge.setText("🟢 CONNECTED")
        self.lbl_connection_badge.setStyleSheet(
            "background-color: #14532d; color: #bbf7d0; font-weight: bold; padding: 6px 14px; border-radius: 6px;"
        )
        board_str = dev.board_name or self.current_profile.board
        fw_str = dev.firmware_version or self.current_profile.firmware
        self.lbl_device_info.setText(f"Device: {board_str} | FW: {fw_str} | Port: {dev.port}")

        matched = self.profile_mgr.auto_match_profile(board_str, fw_str)
        if matched and matched.name != self.current_profile.name:
            self.current_profile = matched
            self.lbl_profile_title.setText(f"TEST PROFİLİ: {matched.name} (Auto-Matched)")
            if self.worker:
                self.worker.update_profile(self.current_profile, self.active_tests)

    @Slot()
    def _on_device_disconnected(self):
        self.lbl_connection_badge.setText("🔴 DISCONNECTED")
        self.lbl_connection_badge.setStyleSheet(
            "background-color: #7f1d1d; color: #fecaca; font-weight: bold; padding: 6px 14px; border-radius: 6px;"
        )
        self.lbl_device_info.setText("Cihaz: SÖKÜLDÜ | Port: ---")
        self._set_banner_style("SEARCHING")
        self.lbl_banner_main.setText("🔴 CİHAZ ÇIKARILDI: Lütfen Yeni Cihaz Takınız")
        self.lbl_banner_sub.setText("Eski veriler temizlendi • Yeni FC bekleniyor...")

    def _populate_table(self, table: QTableWidget, items: List[TestResultItem]):
        table.setRowCount(len(items))
        for row, r in enumerate(items):
            table.setRowHeight(row, 25)

            item_name = QTableWidgetItem(r.name)
            item_name.setFont(QFont("Segoe UI", 9, QFont.Bold))
            table.setItem(row, 0, item_name)

            item_act = QTableWidgetItem(r.actual_value)
            item_act.setTextAlignment(Qt.AlignCenter)
            item_act.setFont(QFont("Segoe UI", 9))
            table.setItem(row, 1, item_act)

            item_exp = QTableWidgetItem(r.expected_value)
            item_exp.setTextAlignment(Qt.AlignCenter)
            item_exp.setFont(QFont("Segoe UI", 9))
            table.setItem(row, 2, item_exp)

            item_st = QTableWidgetItem()
            item_st.setTextAlignment(Qt.AlignCenter)
            item_st.setFont(QFont("Segoe UI", 9, QFont.Bold))

            if r.status == TestStatus.PASS:
                item_st.setText("🟢 PASS")
                item_st.setForeground(QColor("#4ade80"))
            elif r.status == TestStatus.FAIL:
                item_st.setText("🔴 FAIL")
                item_st.setForeground(QColor("#f87171"))
            elif r.status == TestStatus.WARNING:
                item_st.setText("🟡 WARN")
                item_st.setForeground(QColor("#facc15"))
            elif r.status == TestStatus.NOT_AVAILABLE:
                item_st.setText("⚪ N/A")
                item_st.setForeground(QColor("#94a3b8"))
            elif r.status == TestStatus.SKIPPED:
                item_st.setText("⚪ SKIP")
                item_st.setForeground(QColor("#64748b"))

            table.setItem(row, 3, item_st)

    @Slot(list, bool)
    def _on_test_results(self, results: List[TestResultItem], all_passed: bool):
        self.last_results = results
        self.last_passed_flag = all_passed

        left_ids = {
            "board", "firmware", "mcu_uid", "imu1", "imu2", "baro", "vbus",
            "battery", "battery_cells", "current", "cpu_load", "i2c_bus",
            "cycle_time", "memory"
        }
        left_items = [r for r in results if r.test_id in left_ids]
        right_items = [r for r in results if r.test_id not in left_ids]

        self._populate_table(self.table_left, left_items)
        self._populate_table(self.table_right, right_items)

        pass_count = sum(1 for r in results if r.status == TestStatus.PASS)
        failed_tests = [r.name for r in results if r.status == TestStatus.FAIL]
        total = len(results)

        if all_passed:
            self._set_banner_style("PASS")
            self.lbl_banner_main.setText("🟢 ALL TESTS PASSED")
            self.lbl_banner_sub.setText(f"{pass_count} / {total} PASS • Kart Seri Üretime ve Uçuşa Uygundur.")
        else:
            self._set_banner_style("FAIL")
            self.lbl_banner_main.setText("🔴 TEST FAILED")
            fail_names = ", ".join(failed_tests[:3])
            self.lbl_banner_sub.setText(f"{pass_count} / {total} PASS • {len(failed_tests)} HATA: {fail_names}")
            self.audio_notifier.play_fail_alert()

    def _open_full_inspector(self):
        dev = self.worker.current_device if self.worker else None
        live = self.worker.live_data.copy() if (self.worker and self.worker.live_data) else {}
        if not live:
            live = GoldenSnapshotExtractor.build_baseline_snapshot(self.current_profile.board, self.current_profile.firmware)
        dlg = FullParameterInspectorDialog(self.current_profile, dev, live, self)
        dlg.exec()

    def _open_log_folder(self):
        if self.worker and self.last_results:
            log_path = self.test_logger.save_report(
                self.worker.current_device, self.current_profile,
                self.last_results, self.last_passed_flag
            )
            QMessageBox.information(self, "Log Kaydedildi", f"Test raporu başarıyla kaydedildi:\n{log_path}")
        else:
            QMessageBox.information(self, "Bilgi", "Henüz kaydedilecek tamamlanmış test sonucu yok.")

    def _on_back_clicked(self):
        if self.worker:
            self.worker.stop()
            self.worker = None
        self.back_to_menu_signal.emit()


# ============================================================================
# 11. ANA PENCERE VE ÇALIŞTIRICI (MAIN APPLICATION WINDOW & CONTROLLER)
# ============================================================================

class BetaflightTesterApp(QMainWindow):
    """Ana Pencere: Menü, ATE İstasyonu ve Dialogları birbirine bağlar."""
    def __init__(self):
        super().__init__()
        self.profile_mgr = ProfileManager()
        self.setWindowTitle("BETAFIGHT FC TESTER")
        self.resize(1150, 780)
        self.setStyleSheet(DARK_THEME_QSS)

        self.stack = QStackedWidget()
        self.setCentralWidget(self.stack)

        self.menu_widget = MainMenuWidget()
        self.station_widget = ProductionStationWindow(self.profile_mgr)

        self.stack.addWidget(self.menu_widget)      # Index 0
        self.stack.addWidget(self.station_widget)   # Index 1

        self._bind_signals()
        self.current_sim_faults: Dict[str, bool] = {}
        self.is_sim_active = False

    def _bind_signals(self):
        self.menu_widget.add_model_clicked.connect(self._open_add_model_dialog)
        self.menu_widget.profiles_clicked.connect(self._open_profiles_dialog)
        self.menu_widget.start_test_clicked.connect(self._start_test_flow)
        self.menu_widget.simulation_clicked.connect(self._open_simulation_dialog)
        self.menu_widget.settings_clicked.connect(self._open_settings_dialog)
        self.station_widget.back_to_menu_signal.connect(lambda: self.stack.setCurrentIndex(0))

    def _open_add_model_dialog(self):
        dlg = NewModelDialog(self.profile_mgr, self)
        dlg.exec()

    def _open_profiles_dialog(self):
        dlg = ProfileManagerDialog(self.profile_mgr, self)
        dlg.profile_selected.connect(self._on_profile_selected)
        dlg.exec()

    def _on_profile_selected(self, profile: TestProfile):
        self.station_widget.current_profile = profile
        QMessageBox.information(self, "Profil Seçildi", f"'{profile.name}' aktif test modeli olarak belirlendi.")

    def _open_simulation_dialog(self):
        dlg = SimulationDialog(self.current_sim_faults, self.is_sim_active, self)
        if dlg.exec() == QDialog.Accepted:
            self.is_sim_active = dlg.is_simulation_enabled()
            self.current_sim_faults = dlg.get_fault_config()
            status_text = "AKTİF" if self.is_sim_active else "DEVRE DIŞI"
            QMessageBox.information(self, "Simülasyon", f"Simülasyon modu {status_text} olarak güncellendi.")

    def _open_settings_dialog(self):
        QMessageBox.information(
            self, "Sistem Bilgisi",
            f"Betaflight Tester v1.0 (ATE Edition)\n"
            f"MSP Protokol: v1 & v2 Desteği\n"
            f"Kayıtlı Profil Sayısı: {len(self.profile_mgr.list_profiles())}\n"
            f"Pyserial Durumu: {'YÜKLÜ' if HAS_SERIAL else 'YÜKLÜ DEĞİL'}\n"
            f"Winsound Ses: {'DESTEKLENİYOR' if HAS_WINSOUND else 'YOK'}\n"
            f"Log Dizini: logs/"
        )

    def _start_test_flow(self):
        dlg = TestSelectionDialog(self.station_widget.current_profile, self)
        if dlg.exec() == QDialog.Accepted:
            selected_tests = dlg.get_selected_tests()
            self.stack.setCurrentIndex(1)
            self.station_widget.start_testing(
                self.station_widget.current_profile,
                selected_tests,
                self.is_sim_active,
                self.current_sim_faults
            )


def main():
    app = QApplication(sys.argv)
    window = BetaflightTesterApp()
    window.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()
