# Third-party notices

Two parts of `fd_core` are ports, translated by hand into C++ from published code under permissive licences: FaSTTUBe's
cone path planner and Paul Dierckx's FITPACK spline fit (decision 0031). Their notices come first. The components after
them are built from source, which is not kept in Git, and linked into targets outside `fd_core`. Each entry records the
decision that adopted it. Code read for structure only, and code under a licence this project does not take, is listed
last.

## FaSTTUBe ft-fsd-path-planning 0.4.3.2 (ported)

- Used by: `core/src/cone_path.cpp` and `core/include/fd/cone_path.hpp`, a C++ port of its cone sorting, cone matching
  with virtual cones and path calculation for a trackdrive, with the helpers they use. Decision 0031.
- Licence: MIT License. Copyright (c) 2022 Panagiotis.
- Source: the published package as downloaded to this machine
  (`ft-fsd-path-planning-main`, `pyproject.toml` version 0.4.3.2); not kept in Git. The Python ran unmodified in an
  isolated environment to produce `tests/fixtures/cone_path` (see `tools/cone_path_fixtures/README.md`).

The MIT licence requires the copyright notice and permission notice in all copies or substantial portions:

> Copyright (c) 2022 Panagiotis
>
> Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
> documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
> rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit
> persons to whom the Software is furnished to do so, subject to the following conditions:
>
> The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
> Software.
>
> THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
> WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
> COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
> OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

## FITPACK (P. Dierckx), parametric curve fit (ported)

- Used by: `core/src/fitpack.cpp` and `core/src/fitpack.hpp`, a C++ port of `parcur`, `fppara`, `fpback`, `fpbspl`,
  `fpdisc`, `fpgivs`, `fpknot`, `fprati`, `fprota` and `splev` with SciPy's extrapolation, which FaSTTUBe's path
  calculation calls through `scipy.interpolate.splprep` and `splev`. Decision 0031.
- Source: netlib's dierckx collection, <https://www.netlib.org/dierckx/>, fetched into `.tools/fitpack-reference`
  (not kept in Git) with the SHA-256 of each file in its `SHA256SUMS`. Netlib's files carry no licence text of their
  own; SciPy distributes the same routines under its BSD 3-Clause licence, Copyright (c) 2001-2002 Enthought, Inc.
  2003-2024, SciPy Developers, which is the licence this port is taken under. SciPy 1.15.3 produced the reference fits in
  `tests/fixtures/fitpack`.

SciPy's BSD 3-Clause licence requires, in a binary distribution, its copyright notice, the three conditions and the
disclaimer, which travel in `.tools/fsd-python/Lib/site-packages/scipy-1.15.3.dist-info/LICENSE.txt`.

## OSQP v1.0.0

- Used by: `fd_raceline` (statically linked `osqpstatic`), and through it `fd_headless`, `fd_desktop` and
  `fd_raceline_tests`. Decision 0020.
- Licence: Apache License 2.0.
- Source: <https://github.com/osqp/osqp/releases/download/v1.0.0/osqp-v1.0.0-src.tar.gz>, SHA-256
  `ec0bb8fd34625d0ea44274ab3e991aa56e3e360ba30935ae62476557b101c646`, fetched by `scripts/bootstrap-osqp.ps1` into
  `.tools/osqp/v1.0.0`; licence text in `.tools/osqp/v1.0.0/LICENSE`, notice in `.tools/osqp/v1.0.0/NOTICE`.
- Not modified. Built with its shared library, demo, unit tests, printing, profiling and interrupt handling off.

OSQP's NOTICE names its copyright holders (2019: Bartolomeo Stellato, Goran Banjac, Paul Goulart, Stephen Boyd), credits
Stanford University and the University of Oxford, and lists the two external modules below; the file itself travels with
any binary.

## QDLDL v0.1.8

- Used by: OSQP's direct linear-system solver, compiled into `osqpstatic`. Decision 0020.
- Licence: Apache License 2.0. Copyright (c) 2018, Paul Goulart, Bartolomeo Stellato, Goran Banjac.
- Source: <https://github.com/osqp/qdldl/archive/refs/tags/v0.1.8.tar.gz>, SHA-256
  `ecf113fd6ad8714f16289eb4d5f4d8b27842b6775b978c39def5913f983f6daa`, fetched by `scripts/bootstrap-osqp.ps1` into
  `.tools/osqp/qdldl-v0.1.8`; licence text in its `LICENSE`. Not modified.

## AMD (SuiteSparse), as distributed inside OSQP

- Used by: OSQP's fill-reducing ordering, compiled into `osqpstatic`. Decision 0020.
- Licence: BSD 3-Clause. Copyright (c) 1996-2015, Timothy A. Davis, Patrick R. Amestoy, and Iain S. Duff.
- Source: `.tools/osqp/v1.0.0/algebra/_common/lin_sys/qdldl/amd`, licence text in its `LICENSE`. Not modified.

Nothing is distributed today. A binary distribution of an application linking `fd_core` must include FaSTTUBe's MIT
notice and SciPy's BSD 3-Clause notice above; one linking `fd_raceline` must also include the Apache License 2.0 text,
OSQP's NOTICE, and the AMD copyright notice, conditions and disclaimer from the files named above.

## Run beside the build, never linked or redistributed

The theoretical best lap (decision 0034) runs two outside tools from `tools/optimal_lap/`, as separate processes on
files; neither is linked into any target, copied into this repository or distributed with it. Both were downloaded once
with the user's approval into `.tools/`, which Git ignores.

- fastest-lap 0.5 (MIT, Juan Manzanero): the prebuilt Windows release `fastest-lap-w10-x64-msvc2022-v0.5.zip`, SHA-256
  `81947ae517c7029278ed7cef0c06e74a2664cec3130a573d66647483bd08831c` (computed on download; the release states none),
  loaded through its own Python wrapper. The release carries Ipopt 3.14.10 (Eclipse Public License 2.0), MUMPS 5.5.1
  (CeCILL-C) and Intel runtime libraries under Intel's own terms, each as its authors built it. Its vehicle and track
  database was read for the file formats; the Formula One style profile's values came from its Limebeer 2014 file
  (decision 0033).
- TUM's global_racetrajectory_optimization (LGPL-3.0): imported from where it was downloaded by
  `tools/optimal_lap/cross_check_tum.py`, which writes nothing into it, in a Python 3.8 environment with its pinned
  requirements (casadi 3.5.1 LGPL-3.0, numpy, scipy, matplotlib and scikit-learn BSD, trajectory_planning_helpers
  LGPL-3.0, quadprog GPL-2.0-or-later), all in `.tools/tum-python`.

## Desktop recorded driving audio (decision 0045)

- `apps/desktop/sample_loop.*`: variable-rate sample-player cursor/interpolation adapted from
  [ExhaustNote](https://github.com/FASTSHIFT/ExhaustNote), commit
  `5760db9c30d59a7fdaaec7eb079df7b9b4700e3c`, `core/src/sample_player.cpp`.
  MIT, Copyright (c) 2026 VIFEX. Complete notice: `assets/audio-licenses/ExhaustNote-MIT.txt`.
  Changes: float sample ownership, double cursor, input checks, looping seam treatment and filtered multirate levels.
  None of ExhaustNote's vehicle/transmission simulation or synthetic demo samples is used.
- `f1v10.wav` from Speed Dreams SVN r9651, `trunk/data/cars/models/mp1-diamond-r25/`.
  The pack's `readme.txt` credits Copyright 2020 Xavier Bertaux and licenses the artwork under the Free Art License.
  Original WAV embedded unchanged; the playback processing (DC removal, normalization, seam overlap, filtering and
  pitch/load changes) is an adaptation of that sound under the same Free Art License 1.3. See the full license in
  `assets/audio-licenses/Free-Art-License-1.3.txt`. It remains a separable audio work, not simulation code.
  Original available at <https://svn.code.sf.net/p/speed-dreams/code/!svn/bc/9651/trunk/data/cars/models/mp1-diamond-r25/f1v10.wav>.
- `skid_tyres.wav` from Speed Dreams SVN r9651, `trunk/data/data/sound/`, CC0-1.0 per its `SoundCredits.txt`:
  [LukaCafuka's recording](https://freesound.org/people/LukaCafuka/sounds/752837/), sampling and mixing by Overshot.
  <https://creativecommons.org/publicdomain/zero/1.0/>. Same playback preparation as the engine sample.

`scripts/bootstrap-audio.ps1` fetches these assets and original credits with SHA-256 verification; the source manifest
is `assets/audio-licenses/sources.json`. Downloaded media remains ignored under `artifacts/audio/`. Builds embed the
original WAVs and audio-license text; redistribution must preserve these credits and the Free Art License for the
engine audio and its adaptations. No commercial-game audio was extracted. Other recordings examined numerically
under `out/audio-research/` are not included in the application.

## Read, not taken

- PacSim (MIT): its sensor models, dead time and perception sensor were read for structure and the
  models' form (decisions 0029, 0030), and its competition logic and configuration for the rules they encode (decision
  0032); nothing was copied. Its Formula Student track files trace real competitions' courses whose origin the
  repository does not state, so they are read at run time from where they lie (`--course FILE`) and never copied here.
- FS-FEUP's Delaunay path planner (GPL-3.0): only its published description was read. `core/src/delaunay_path.cpp` was
  written from that description alone, never from its code (decision 0031).
