#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
"""Small dependency-free validator for CinePi GPR outputs."""
from __future__ import annotations

import hashlib
import struct
import subprocess
from pathlib import Path

EXPECTED_TIFF_HEADERS = (b"II*\x00", b"MM\x00*")
CUSTOM_MAGIC = b"CPRVC5B3"


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for block in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_ifd0_scalars(path: Path) -> tuple[dict[int, int], set[int]]:
    data = path.read_bytes()
    if len(data) < 8 or data[:4] not in EXPECTED_TIFF_HEADERS:
        raise RuntimeError(f"{path.name} is not a TIFF/DNG file")
    endian = "<" if data[:2] == b"II" else ">"
    ifd = struct.unpack_from(endian + "I", data, 4)[0]
    if ifd + 2 > len(data):
        raise RuntimeError(f"{path.name} has an invalid IFD offset")
    count = struct.unpack_from(endian + "H", data, ifd)[0]
    if ifd + 2 + count * 12 + 4 > len(data):
        raise RuntimeError(f"{path.name} has a truncated IFD")
    values: dict[int, int] = {}
    tag_ids: set[int] = set()
    for index in range(count):
        offset = ifd + 2 + index * 12
        tag, field_type, item_count = struct.unpack_from(endian + "HHI", data, offset)
        tag_ids.add(tag)
        inline = data[offset + 8:offset + 12]
        if field_type == 1 and item_count <= 4:
            values[tag] = int.from_bytes(inline[:item_count], "little" if endian == "<" else "big")
        elif item_count == 1 and field_type == 3:
            values[tag] = struct.unpack_from(endian + "H", inline, 0)[0]
        elif item_count == 1 and field_type == 4:
            values[tag] = struct.unpack_from(endian + "I", inline, 0)[0]
    return values, tag_ids


def validate_gpr(decoder: Path, gpr: Path, decoded: Path, expected_bytes: int,
                 width: int, height: int, log: Path) -> dict[str, object]:
    if not gpr.is_file() or gpr.stat().st_size == 0:
        raise RuntimeError(f"GPR output is missing or empty: {gpr}")
    prefix = gpr.read_bytes()[:16]
    if prefix.startswith(CUSTOM_MAGIC):
        raise RuntimeError(f"{gpr.name} is a CPRVC5B3 packet, not GPR")
    if prefix[:4] not in EXPECTED_TIFF_HEADERS:
        raise RuntimeError(f"{gpr.name} lacks a TIFF/DNG header: {prefix[:8].hex()}")
    tags, tag_ids = read_ifd0_scalars(gpr)
    required = {256: width, 257: height, 259: 9, 262: 32803}
    for tag, expected in required.items():
        if tags.get(tag) != expected:
            raise RuntimeError(f"{gpr.name} tag {tag} is {tags.get(tag)!r}; expected {expected}")
    if 50706 not in tags:
        raise RuntimeError(f"{gpr.name} lacks DNGVersion tag 50706")

    # CinePi GP-LOG2 must remain log after VC-5 decode and Adobe RAW import.
    # A DNG LinearizationTable would inverse the log curve on import and turn
    # the file back into linear RAW, which is the regression this validator is
    # specifically intended to prevent.
    if 50712 in tag_ids:  # 0xC618 LinearizationTable
        raise RuntimeError(f"{gpr.name} contains LinearizationTable; Adobe would decode GP-LOG2 to linear")
    if tags.get(50717) != 4095:  # 0xC61D WhiteLevel
        raise RuntimeError(f"{gpr.name} WhiteLevel is {tags.get(50717)!r}; expected GP-LOG2 code maximum 4095")
    if tags.get(274) != 1:
        raise RuntimeError(f"{gpr.name} Orientation is {tags.get(274)!r}; expected 1")
    for forbidden_tag, name in ((51009, "OpcodeList2/GainMap"), (51022, "OpcodeList3/WarpRectilinear")):
        if forbidden_tag in tag_ids:
            raise RuntimeError(f"{gpr.name} unexpectedly contains {name}")

    decoded.unlink(missing_ok=True)
    proc = subprocess.run(
        [str(decoder), "-i", str(gpr), "-o", str(decoded)],
        text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
    )
    log.write_text(proc.stdout, encoding="utf-8")
    if proc.returncode:
        raise RuntimeError(f"SDK decode failed ({proc.returncode}) for {gpr.name}\n{proc.stdout}")
    actual = decoded.stat().st_size if decoded.is_file() else 0
    if actual != expected_bytes:
        raise RuntimeError(f"{gpr.name} decoded to {actual} bytes; expected {expected_bytes}")
    return {
        "tiff_dng_header": prefix[:4].hex(),
        "image_width_tag": tags[256],
        "image_height_tag": tags[257],
        "compression_tag": tags[259],
        "photometric_cfa_tag": tags[262],
        "dng_version_tag_present": True,
        "gplog2_linearization_table_absent": True,
        "gplog2_white_level": tags[50717],
        "orientation_tag": tags[274],
        "gopro_lens_opcodes_absent": True,
        "custom_packet_magic_absent": True,
        "official_sdk_decode_pass": True,
        "decoded_bytes": actual,
        "sha256": sha256(gpr),
    }
