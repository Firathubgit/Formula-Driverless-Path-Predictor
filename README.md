# Formula Driverless Path Predictor

**Make autonomous racing decisions visible.** A native C++20 and Qt 6 racing simulator that shows *why* a driverless
car accelerates, holds speed, brakes or changes line, and lets you take the wheel yourself.






<img width="2864" height="1640" alt="image" src="https://github.com/user-attachments/assets/8881017f-3e37-40cc-a3f7-d45b2615f5ad" />

<img width="2858" height="1646" alt="image" src="https://github.com/user-attachments/assets/fcc44681-565f-4124-8109-b84355053b63" />



*A lap in the desktop application, shown at three times speed.*

## Highlights

- **Vehicle physics, from a kinematic bicycle to a four-wheel car.** Pacejka-style tires with load sensitivity and
  combined slip, rotating wheels that lock and spin, quasi-static load transfer, drag and downforce, and setup controls
  whose effects are each proven by a test.
- **Planning you can see.** A three-second prediction ribbon coloured by acceleration, a G-G-V envelope derived from the
  car itself, a minimum-curvature racing line (OSQP), and a lattice planner that compares each action by its own speed
  profile and labels the one it picks.
- **Model predictive contouring control (MPCC)** as an optimising controller, measured against the plant it drives.
- **Driving from its own senses.** Noisy, delayed instruments and simulated cone perception, a port of FaSTTUBe's cone
  path planner, and Formula Student judging after PacSim: laps, sectors, cones down, off course, DNF.
- **The theoretical best lap.** A car's minimum-time lap solved offline with fastest-lap, cross-checked against TUM's
  formulation, and shown as a translucent "offline optimum" car with sector deltas and setup sensitivities.
- **Drive it yourself** with an Xbox controller, including a rental-kart mode on a traced indoor track with a coaching
  line, practice lap timing and adjustable driving assists.
- **Every run is a checked contract.** Recordings replay deterministically and are validated file-by-file on load.

## Quick start (Windows)

Requirements: Windows 10/11, Visual Studio 2022 with the C++ workload (CMake included), PowerShell, and Python 3 for the
helper scripts.

```powershell
./scripts/bootstrap-qt.ps1      # once: a project-local Qt 6.8.3 kit
./scripts/bootstrap-osqp.ps1    # optional: racing line and MPCC
./scripts/build.ps1 -Desktop    # build and run every test suite
./scripts/run.ps1               # launch the desktop application
```

`./scripts/build.ps1` builds only the Qt-free core and its tests. The headless runner records and validates experiments:

```powershell
.\build\core\apps\headless\Release\fd_headless.exe --help
.\build\core\apps\headless\Release\fd_headless.exe --plant four-wheel --laps 1 --out out\runs\first-lap
.\build\core\apps\headless\Release\fd_headless.exe --check-recording out\runs\first-lap
```

## Project layout

| Path | What it holds |
| --- | --- |
| `core/` | `fd_core`: plants, tires, planning, prediction, perception and judging; plain C++20, no external dependencies |
| `adapters/` | Simulation loop, sensors and the recording contract |
| `raceline/`, `mpcc/` | Optional OSQP-based racing line and MPCC targets |
| `apps/headless/` | Command-line experiment runner and validators |
| `apps/desktop/` | Qt Quick 3D application |
| `tools/` | Offline tooling: optimal lap, track tracing, fixtures, showroom |
| `tests/` | Physics, contract and failure-case test suites run by CTest |
| `docs/` | Architecture, status, architecture decision records and evidence |

## Documentation

- [GUIDE.md](GUIDE.md): every feature, with commands that reproduce its results
- [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md): modules, frames and data contracts
- [docs/STATUS.md](docs/STATUS.md): what is verified, with evidence
- [docs/decisions/](docs/decisions/): why each design choice was made
- [CONTEXT.md](CONTEXT.md): domain vocabulary

## Honest limits

Everything is simulated: perception samples ground-truth cones, and the known track and ideal state are stated
assumptions wherever they apply. Vehicle parameters are published, derived, fitted or chosen, and each is labelled as
such; none are measured on a real car. MPCC is not real time on the reference machine. ROS integration is planned but
not built.

## Third-party code

Ported code and external tools, and their licences, are listed in [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).
Downloaded tools, the showroom's car models and media, and build outputs are not part of this repository.
