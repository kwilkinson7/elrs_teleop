import struct

class CRSFTelemetrySender:
    def __init__(self, serial_port):
        self.ser = serial_port

    def _crsf_crc(self, data):
        crc = 0
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc <<= 1
                crc &= 0xFF
        return crc

    def _send_packet(self, sensor_id, payload: bytes):
        body = bytearray([0x10, sensor_id]) + payload  # TYPE = 0x10 (Telemetry payload)
        crc = self._crsf_crc(body)
        frame = bytearray([0xC8, len(body)]) + body + bytearray([crc])
        self.ser.write(frame)

    def send_voltage(self, volts: float):
        val = int(27)
        self._send_packet(0x10, struct.pack(">H", val))
        # testing
        self._send_packet(0x21, struct.pack(">H", 13))
        self._send_packet(0x22, struct.pack(">H", 2))
        self._send_packet(0x23, struct.pack(">H", 4))

    def send_mode(self, mode: int):
        self._send_packet(0x22, bytes([mode]))

    def send_cpu_percent(self, percent: float):
        val = int(percent * 10)
        self._send_packet(0x10, struct.pack(">H", val))

    def send_heading(self, yaw_deg: float):
        val = int(yaw_deg * 10)
        self._send_packet(0x21, struct.pack(">H", val))
