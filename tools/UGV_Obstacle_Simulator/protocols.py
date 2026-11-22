"""Wire formats, copied from what the original Weeder_Simulator.exe sends (captured 2026-11-20)."""
from __future__ import annotations

import math
import time

MAX_BUF = 4096


def build_scan_telegram(ranges_mm: list[int], counter: int) -> bytes:
    """One LMS151 'sRN LMDscandata' reply, ranges in mm (hex), 0..180 deg in 0.5 deg steps."""
    c = f"{counter & 0xFFFF:X}"
    t = f"{int(time.monotonic() * 1e6) & 0xFFFFFFFF:X}"
    head = (f"\x02sRN LMDscandata 0 1 9BF210 0 0 {c} {c} {t} {t} 0 0 7 0 0 1388 168 0 1 "
            f"DIST1 3F800000 00000000 0 1388 {len(ranges_mm):X} ")
    body = " ".join(f"{int(r):X}" for r in ranges_mm)
    return (head + body + " 0 0 0 0 0 0\x03").encode("ascii")


class VCParser:
    """Stream parser for '# <steer> <speed> <wdog> #' frames (split or concatenated)."""

    def __init__(self):
        self.buf = ""

    def feed(self, data: bytes) -> list[tuple[float, float, int]]:
        self.buf += data.decode("ascii", errors="replace")
        out = []
        while True:
            i = self.buf.find("#")
            if i < 0:
                self.buf = ""
                break
            j = self.buf.find("#", i + 1)
            if j < 0:
                self.buf = self.buf[i:][-MAX_BUF:]
                break
            parts = self.buf[i + 1:j].split()
            frame = None
            if len(parts) == 3:
                try:
                    nums = [float(x) for x in parts]
                    if all(math.isfinite(x) for x in nums):
                        frame = (nums[0], nums[1], int(nums[2]))
                except (ValueError, OverflowError):
                    frame = None
            if frame is not None:
                out.append(frame)
                self.buf = self.buf[j + 1:]
            else:
                # the closing '#' may really be the opener of the next frame
                self.buf = self.buf[j:]
        return out


class ScanRequestParser:
    """Counts complete STX ... LMDscandata ... ETX requests in a byte stream."""

    def __init__(self):
        self.buf = b""

    def feed(self, data: bytes) -> int:
        self.buf += data
        n = 0
        while True:
            i = self.buf.find(b"\x02")
            if i < 0:
                self.buf = b""
                break
            j = self.buf.find(b"\x03", i + 1)
            if j < 0:
                self.buf = self.buf[i:][-MAX_BUF:]
                break
            if b"LMDscandata" in self.buf[i + 1:j]:
                n += 1
            self.buf = self.buf[j + 1:]
        return n
