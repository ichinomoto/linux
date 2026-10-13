#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0+
"""Usage: python3 tools/dm200-mkresource.py kernel.dtb resource.img"""
import struct
import sys
from pathlib import Path

dtb = Path(sys.argv[1]).read_bytes()
header = struct.pack("<4sHHBBBxI", b"RSCE", 0, 0, 1, 1, 1, 1)
entry = struct.pack("<4s256sII", b"ENTR", b"rk-kernel.dtb", 2, len(dtb))
Path(sys.argv[2]).write_bytes(
    header.ljust(512, b"\0") + entry.ljust(512, b"\0")
    + dtb + b"\0" * (-len(dtb) % 512)
)
