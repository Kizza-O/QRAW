#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Kieran Olsson
"""Shared GP-LOG2 GPR-to-DNG export for Raspberry Pi CinePi runs."""

from __future__ import annotations

import argparse
import json
import os
import struct
import subprocess
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path
from typing import Any

ROOT = Path(__file__).resolve().parent


class DngValidationError(RuntimeError):
    pass


def _type_size(type_id: int) -> int:
    sizes = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8, 7: 1, 9: 4, 10: 8}
    if type_id not in sizes:
        raise DngValidationError(f"Unsupported TIFF field type {type_id}.")
    return sizes[type_id]


def _read_ifd(path: Path) -> tuple[str, dict[int, tuple[int, int, bytes]]]:
    data = path.read_bytes()
    if len(data) < 8:
        raise DngValidationError(f"{path.name} is too small to be TIFF/DNG.")
    if data[:2] == b"II":
        endian = "<"
    elif data[:2] == b"MM":
        endian = ">"
    else:
        raise DngValidationError(f"{path.name} lacks a TIFF byte-order marker.")
    if struct.unpack_from(endian + "H", data, 2)[0] != 42:
        raise DngValidationError(f"{path.name} lacks the TIFF magic value.")
    ifd_offset = struct.unpack_from(endian + "I", data, 4)[0]
    if ifd_offset + 2 > len(data):
        raise DngValidationError(f"{path.name} has an invalid first IFD offset.")
    count = struct.unpack_from(endian + "H", data, ifd_offset)[0]
    tags: dict[int, tuple[int, int, bytes]] = {}
    for index in range(count):
        pos = ifd_offset + 2 + index * 12
        if pos + 12 > len(data):
            raise DngValidationError(f"{path.name} has a truncated IFD.")
        tag, type_id, value_count = struct.unpack_from(endian + "HHI", data, pos)
        byte_count = _type_size(type_id) * value_count
        if byte_count <= 4:
            raw = data[pos + 8:pos + 8 + byte_count]
        else:
            value_offset = struct.unpack_from(endian + "I", data, pos + 8)[0]
            if value_offset + byte_count > len(data):
                raise DngValidationError(f"{path.name} tag {tag} points outside the file.")
            raw = data[value_offset:value_offset + byte_count]
        tags[tag] = (type_id, value_count, raw)
    return endian, tags


def _numeric_values(endian: str, field: tuple[int, int, bytes] | None) -> list[float]:
    if field is None:
        return []
    type_id, count, raw = field
    if type_id in (1, 7):
        return [float(value) for value in raw[:count]]
    if type_id == 3:
        return [float(value) for value in struct.unpack(endian + "H" * count, raw)]
    if type_id == 4:
        return [float(value) for value in struct.unpack(endian + "I" * count, raw)]
    if type_id == 9:
        return [float(value) for value in struct.unpack(endian + "i" * count, raw)]
    if type_id in (5, 10):
        fmt = "II" if type_id == 5 else "ii"
        values = []
        for index in range(count):
            numerator, denominator = struct.unpack_from(endian + fmt, raw, index * 8)
            values.append(float(numerator) / float(denominator) if denominator else 0.0)
        return values
    raise DngValidationError(f"Unsupported numeric TIFF type {type_id}.")


def _matrix_determinant(values: list[float]) -> float | None:
    if len(values) != 9:
        return None
    return (
        values[0] * (values[4] * values[8] - values[5] * values[7])
        - values[1] * (values[3] * values[8] - values[5] * values[6])
        + values[2] * (values[3] * values[7] - values[4] * values[6])
    )


def _profile(path: Path) -> dict[str, Any]:
    endian, tags = _read_ifd(path)
    return {
        "width": int(_numeric_values(endian, tags.get(256))[0]),
        "height": int(_numeric_values(endian, tags.get(257))[0]),
        "cfa_pattern": [int(v) for v in _numeric_values(endian, tags.get(33422))],
        "color_matrix_1": _numeric_values(endian, tags.get(50721)),
        "color_matrix_2": _numeric_values(endian, tags.get(50722)),
        "as_shot_neutral": _numeric_values(endian, tags.get(50728)),
        "illuminant_1": int(_numeric_values(endian, tags.get(50778))[0]) if 50778 in tags else None,
        "illuminant_2": int(_numeric_values(endian, tags.get(50779))[0]) if 50779 in tags else None,
        "forward_matrix_1": _numeric_values(endian, tags.get(50964)),
        "forward_matrix_2": _numeric_values(endian, tags.get(50965)),
    }


def _values_match(actual: list[float], expected: list[float], tolerance: float = 1.0e-5) -> bool:
    return len(actual) == len(expected) and all(
        abs(float(a) - float(b)) <= tolerance for a, b in zip(actual, expected)
    )


def inspect_dng(path: Path) -> dict[str, Any]:
    endian, tags = _read_ifd(path)
    required = {
        256: "ImageWidth", 257: "ImageLength", 258: "BitsPerSample",
        259: "Compression", 50706: "DNGVersion", 50712: "LinearizationTable",
        50714: "BlackLevel", 50717: "WhiteLevel",
    }
    missing = [name for tag, name in required.items() if tag not in tags]
    if missing:
        raise DngValidationError(f"{path.name} is missing required DNG fields: {', '.join(missing)}.")
    width = int(_numeric_values(endian, tags[256])[0])
    height = int(_numeric_values(endian, tags[257])[0])
    bits = int(_numeric_values(endian, tags[258])[0])
    compression = int(_numeric_values(endian, tags[259])[0])
    white_level = int(_numeric_values(endian, tags[50717])[0])
    black_levels = _numeric_values(endian, tags[50714])
    linearization = [int(value) for value in _numeric_values(endian, tags[50712])]
    if width <= 0 or height <= 0:
        raise DngValidationError(f"{path.name} has invalid dimensions.")
    if bits != 16:
        raise DngValidationError(f"{path.name} BitsPerSample is {bits}, expected 16.")
    if compression != 1:
        raise DngValidationError(f"{path.name} Compression is {compression}, expected uncompressed 1.")
    if white_level != 65535:
        raise DngValidationError(f"{path.name} WhiteLevel is {white_level}, expected 65535.")
    if black_levels and any(abs(value) > 1.0e-9 for value in black_levels):
        raise DngValidationError(f"{path.name} BlackLevel is {black_levels}, expected zero.")
    if len(linearization) != 4096:
        raise DngValidationError(f"{path.name} LinearizationTable has {len(linearization)} entries, expected 4096.")
    if linearization[0] != 0 or linearization[-1] != 65535:
        raise DngValidationError(
            f"{path.name} LinearizationTable endpoints are {linearization[0]} and {linearization[-1]}, expected 0 and 65535."
        )
    if any(a > b for a, b in zip(linearization, linearization[1:])):
        raise DngValidationError(f"{path.name} LinearizationTable is not monotonic.")
    matrix2 = _numeric_values(endian, tags.get(50722))
    determinant2 = _matrix_determinant(matrix2)
    if determinant2 is not None and abs(determinant2) <= 1.0e-12:
        raise DngValidationError(f"{path.name} contains a singular ColorMatrix2.")
    return {
        "file": path.name, "width": width, "height": height,
        "bits_per_sample": bits, "compression": compression,
        "white_level": white_level, "black_levels": black_levels,
        "linearization_entries": len(linearization),
        "linearization_first": linearization[0],
        "linearization_last": linearization[-1],
    }


def validate_profile_match(source_gpr: Path, output_dng: Path) -> None:
    source = _profile(source_gpr)
    output = _profile(output_dng)
    for field in ("width", "height", "cfa_pattern"):
        if output[field] != source[field]:
            raise DngValidationError(
                f"{output_dng.name} {field}={output[field]!r}, source GPR uses {source[field]!r}."
            )
    for field in ("color_matrix_1", "forward_matrix_1", "as_shot_neutral"):
        expected = source[field]
        if expected and not _values_match(output[field], expected):
            raise DngValidationError(f"{output_dng.name} {field} does not match the source GPR.")
    if source["illuminant_1"] is not None and output["illuminant_1"] != source["illuminant_1"]:
        raise DngValidationError(
            f"{output_dng.name} illuminant_1={output['illuminant_1']}, source GPR uses {source['illuminant_1']}."
        )
    # The Adobe DNG SDK may omit a redundant second calibration when both
    # profiles declare the same illuminant. Require it only when it is distinct.
    if source["illuminant_2"] is not None and source["illuminant_2"] != source["illuminant_1"]:
        for field in ("color_matrix_2", "forward_matrix_2"):
            expected = source[field]
            if expected and not _values_match(output[field], expected):
                raise DngValidationError(f"{output_dng.name} {field} does not match the source GPR.")
        if output["illuminant_2"] != source["illuminant_2"]:
            raise DngValidationError(f"{output_dng.name} illuminant_2 does not match the source GPR.")


def locate_decoder(explicit: Path | None = None) -> Path:
    if explicit is not None:
        candidate = explicit.expanduser().resolve()
        if not candidate.is_file():
            raise FileNotFoundError(f"Requested GPR decoder does not exist: {candidate}")
        if not os.access(candidate, os.X_OK):
            raise PermissionError(f"Requested GPR decoder is not executable: {candidate}")
        return candidate

    candidates = [
        ROOT / "cinepi_qraw_bench" / "build" / "gpr_decode_validator",
        ROOT / "third_party" / "gpr" / "build" / "source" / "app" / "cinepi_gpr_sdk" / "cinepi_gpr_sdk",
    ]
    for candidate in candidates:
        if candidate.is_file() and os.access(candidate, os.X_OK):
            return candidate.resolve()
    raise FileNotFoundError(
        "Could not find an executable cinepi_gpr_sdk/gpr_decode_validator. "
        "Build the package first."
    )


def _convert_one(decoder: Path, source: Path, destination: Path) -> dict[str, Any]:
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name(destination.stem + ".part.dng")
    temporary.unlink(missing_ok=True)
    try:
        result = subprocess.run(
            [str(decoder), "-i", str(source), "-o", str(temporary)],
            text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
        )
        if result.returncode:
            raise RuntimeError(f"GPR-to-DNG conversion failed for {source.name}:\n{result.stdout}")
        if not temporary.is_file() or temporary.stat().st_size == 0:
            raise RuntimeError(f"No DNG was created for {source.name}.")
        metadata = inspect_dng(temporary)
        validate_profile_match(source, temporary)
        dng_bytes = temporary.stat().st_size
        os.replace(temporary, destination)
        metadata["file"] = destination.name
        metadata.update({"source_gpr": source.name, "dng_bytes": dng_bytes})
        return metadata
    finally:
        temporary.unlink(missing_ok=True)


def export_gpr_folder(
    input_dir: Path,
    output_dir: Path,
    decoder: Path | None = None,
    workers: int | None = None,
) -> list[dict[str, Any]]:
    input_dir = input_dir.resolve()
    output_dir = output_dir.resolve()
    if not input_dir.is_dir():
        raise NotADirectoryError(f"GPR input directory does not exist: {input_dir}")
    if input_dir == output_dir:
        raise ValueError("DNG output directory must be different from the GPR input directory.")
    decoder = locate_decoder(decoder)
    gpr_files = sorted(path for path in input_dir.iterdir() if path.is_file() and path.suffix.lower() == ".gpr")
    if not gpr_files:
        raise FileNotFoundError(f"No GPR files were found in {input_dir}.")
    output_dir.mkdir(parents=True, exist_ok=True)
    for stale in output_dir.glob("*.part.dng"):
        stale.unlink()
    destination_keys: set[str] = set()
    for source in gpr_files:
        key = f"{source.stem}.dng".casefold()
        if key in destination_keys:
            raise ValueError(
                f"Multiple GPR inputs map to the same DNG filename: {source.stem}.dng"
            )
        destination_keys.add(key)
    requested_workers = workers if workers is not None else (os.cpu_count() or 1)
    if requested_workers < 1:
        raise ValueError("DNG worker count must be at least 1.")
    worker_count = min(requested_workers, len(gpr_files))
    manifest: list[dict[str, Any]] = []
    with ThreadPoolExecutor(max_workers=worker_count) as executor:
        futures = {
            executor.submit(_convert_one, decoder, source, output_dir / f"{source.stem}.dng"): source
            for source in gpr_files
        }
        completed = 0
        for future in as_completed(futures):
            manifest.append(future.result())
            completed += 1
            print(f"Exported and validated DNG {completed}/{len(gpr_files)}", flush=True)
    manifest.sort(key=lambda item: item["source_gpr"])
    manifest_path = output_dir / "dng_manifest.json"
    manifest_temporary = output_dir / "dng_manifest.json.part"
    manifest_temporary.write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    os.replace(manifest_temporary, manifest_path)
    return manifest


def main() -> int:
    parser = argparse.ArgumentParser(description="Convert a folder of CinePi GP-LOG2 GPR files to validated DNG.")
    parser.add_argument("input_dir", type=Path)
    parser.add_argument("output_dir", type=Path, nargs="?")
    parser.add_argument("--decoder", type=Path)
    parser.add_argument("--workers", type=int)
    args = parser.parse_args()
    output = args.output_dir or args.input_dir.with_name(args.input_dir.name + "_DNG")
    export_gpr_folder(args.input_dir, output, decoder=args.decoder, workers=args.workers)
    print(f"Validated DNG output: {output.resolve()}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
