# Third-Party Notices

QRAW Encoder is licensed under Apache-2.0 (see `LICENSE`). It builds on, links
against and redistributes the components below, each under its own licence.
Nothing here is copyleft: the whole stack is permissive, and Apache-2.0 is
compatible with all of it.

---

## 1. GPR (General Purpose Raw) SDK — the core dependency

| | |
|---|---|
| Upstream | https://github.com/gopro/gpr |
| Copyright | 2018 GoPro, Inc. |
| Licence | **Apache-2.0 OR MIT** (dual, at your option) |
| Location | `third_party/gpr/` (submodule) |
| Modified | **Yes** |

QRAW takes the Apache-2.0 option, which is why this project is Apache-2.0.

**Statement of changes** — required by Apache-2.0 §4(b). **22 files modified,
5 added.** The authoritative per-file list is `CHANGES-QRAW.md` in the fork
itself, which is verified against upstream rather than asserted. Summary:

| Area | Files | Nature |
|---|---|---|
| VC-5 encoder | 6 modified, 1 new | AArch64 entropy variants (`cinepi_vle_*`), fused quantise-in-scan, Component-Aware Quantisation, a MedHigh quality row, and a path for externally pre-transformed coefficient planes |
| VC-5 common | 3 modified | Flat-log passthrough curve, per-band pitch/storage extension, 64-bit bitstream casts |
| VC-5 decoder | 1 modified | 12-bit GP-Log2 code packing instead of 14-bit linear |
| GPR SDK | 6 modified | Container-template retention, RAW16 ingest, a pre-encoded VC-5 wrap entry point, forward matrices |
| Adobe DNG SDK | 1 modified | `dng_flags.h` endianness detection for AArch64. Adobe's BSD header untouched; portability only. |
| Build/tools | 5 modified, 4 new | CMake minimum version, gpr_tools build fixes, two new library targets, fork documentation |

GoPro's copyright headers are retained unmodified in every file. QRAW claims
copyright only in the added lines.

---

## 2. Components vendored inside the GPR SDK

Redistributed as part of GPR, not separately chosen by this project.

| Component | Copyright | Licence | Path |
|---|---|---|---|
| Adobe DNG SDK | 1999–2014 Adobe Systems Inc. | 3-clause BSD | `source/lib/dng_sdk/` |
| Adobe XMP Core | 1999–2014 Adobe Systems Inc. | 3-clause BSD | `source/lib/xmp_core/` |
| Expat | 1998–2000 Thai Open Source Software Center Ltd, Clark Cooper | MIT | `source/lib/expat_lib/` |
| MD5 (ITU/ISO/IEC reference) | 2010–2015 ITU/ISO/IEC | 3-clause BSD | `source/lib/md5_lib/` |
| tiny_jpeg | Sergio Gonzalez | Public domain / unlicence | `source/lib/tiny_jpeg/` |
| cJSON | Dave Gamble and contributors | MIT | `source/app/common/cJSON/` |

**Note on the two Adobe BSD licences:** both carry a no-endorsement clause. Do
not use "Adobe" in the project name, logo, marketing copy or GitHub topics in a
way that implies endorsement. Saying the output "is read by Adobe Camera Raw"
is a factual statement and fine; calling it "Adobe DNG Encoder" is not.

**Note on the MD5 licence:** the ITU/ISO/IEC header explicitly disclaims any
patent grant. This is inherited from upstream and applies to the reference MD5
implementation only.

---

## 3. cinepi-raw and cinepi-gui — not in this repository

The camera application that exercises this encoder is
[cinepi-raw](https://github.com/cinepi/cinepi-raw) (BSD-2-Clause, © 2022 Csaba
Nagy), with a GUI derived from cinepi-gui. **Neither is present here.** The
QRAW integration patches for both live in the separate **cinepi-qraw**
repository, where they are distributed as unified diffs rather than as copies,
so that upstream's `SPDX-License-Identifier: BSD-2-Clause` header and Csaba
Nagy's copyright stay attached to the files they belong to.

This repository is the encoder and its benchmark. It contains no BSD-2-Clause
code.

## 4. Build and runtime dependencies (not redistributed)

Linked against or included at build time; not shipped in this repository.

| Component | Licence | Relationship |
|---|---|---|
| libcamera | LGPL-2.1-or-later | Dynamically linked by cinepi-raw. Dynamic linking keeps QRAW's own licence unaffected; do not statically link it into a proprietary binary without meeting LGPL §6. |
| rpicam-apps | BSD-2-Clause | Base of cinepi-raw |
| Vulkan-Headers (Khronos) | Apache-2.0 | Headers only, GPU path |
| Vulkan loader / Mesa V3DV | MIT | Runtime |
| Redis / hiredis | BSD-3-Clause | IPC for the camera UI |
| spdlog | MIT | Logging, via cinepi-raw |

---

## 5. Formats, curves and names

**GP-Log2** as used here is the Protune-family companding curve
`y = log(1 + Sx) / log(1 + S)` with `S = 599`. A mathematical function is not
copyrightable, and this is an independent implementation — but "GP-Log" and
"Protune" are GoPro names. QRAW uses them descriptively, to say which curve its
output carries so that third-party decoders interpret it correctly.

**The `.gpr` extension** is retained on output files because every third-party
decoder identifies the container that way. QRAW is the name of *this encoder*,
not of the container it writes.

---

## Obtaining full licence texts

- Apache-2.0 — `LICENSE` in this repository
- GPR's MIT option — `third_party/gpr/LICENSE-MIT`
- Adobe DNG SDK BSD — `third_party/gpr/source/lib/dng_sdk/BSD-License.txt`
- Adobe XMP BSD — `third_party/gpr/source/lib/xmp_core/BSD-License.txt`

---

*This inventory is maintained by hand. If you add a dependency, add it here in
the same commit.*
