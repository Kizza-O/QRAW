# Contributing to QRAW Encoder

## Licence of contributions

By submitting a pull request you agree that your contribution is licensed under
**Apache-2.0**, matching the project.

Sign off each commit (`git commit -s`), which asserts the
[Developer Certificate of Origin](https://developercertificate.org/). QRAW uses
DCO rather than a CLA — no paperwork, no copyright assignment, and you keep
your own copyright.

Add an SPDX header to any new file:

```c
/* SPDX-License-Identifier: Apache-2.0
 * Copyright <year> <your name>
 */
```

## Changes to the GPR SDK

Do not modify `third_party/gpr/` in this repository. It is a submodule pointing
at a fork of GoPro's SDK. Open a PR against
[`gpr-qraw`](https://github.com/<you>/gpr-qraw) instead, mark the change
in-line with a `/* QRAW: ... */` comment, and add a row to `CHANGES-QRAW.md`.
Keeping every GoPro-derived change in one auditable tree is what makes the
licensing legible.

## Performance claims

The encoder is pinned and gate-proven. A patch that changes encode behaviour
needs, in the PR description:

- **The correctness gate first.** `tools/verify.sh --library` must pass, or you
  must explain precisely why the CRC changed and what the new one is.
- **A matched A/B.** Same rig, same scene, same session, both directions.
  Cross-session comparisons are noise; an attached display alone moves the
  number by 7%.
- **Run-to-run spread**, not a single figure. If the delta is inside the
  spread, the result is *neutral* — which is a real and useful result, and
  several documented findings are exactly that.

Numbers from a different machine, a different scene, or a single run are not
evidence. This is not pedantry: it is the entire reason the campaign in `docs/`
converged.

## What is welcome

- Ports to other AArch64 platforms (RK3588 in particular)
- Sensors other than IMX585
- Build fixes, packaging, documentation
- Negative results, written up properly

## What is not

- Re-opening settled questions without new measurement. `docs/` records why
  things were rejected; read the relevant finding first.
- Alternative encoder paths. The package deliberately carries one production
  encoder. Variants belong in a branch.

## Reporting bugs

Include: Pi model and RAM, kernel (`uname -a`), page size, sensor, resolution
and frame rate, quality mode, and the launch line. A `.gpr` that decodes wrongly
is worth more than a description of how it looks wrong.
