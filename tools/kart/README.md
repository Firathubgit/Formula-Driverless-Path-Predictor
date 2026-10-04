# The Göteborg kart, fitted to the poster's line

Decision 0046. The operator's poster paints one racing line round Gokartcentralen Göteborg in three colours (gas, coast,
brake). These tools ask what a kart must be like for that line to be right, and fit the simulator's rental kart to it and
to real lap times.

These tools retain that historical fitted reference. The desktop Realistic kart
uses 40% higher longitudinal/lateral tire friction at the user's request
(decision 0055); the fitted lap times below are not claims about that tune.

Needs Python 3 with numpy and scipy (the project's `.tools/tum-python` has both) and a built `fd_headless`
(`./scripts/build.ps1`).

| File | What it does |
| --- | --- |
| `kart_model.py` | Smooths the poster's coloured line (`tracks/gokartcentralen-goteborg-guide.csv`) into a path with honest curvature, laps a kart's grip, power and brakes round it in a quasi-steady-state model, derives the plant's envelope through `fd_headless`, drives the plant round the line and compares its gas, coast and brake with the poster's colours. |
| `make_poster_line.py` | Writes `tracks/gokartcentralen-goteborg-poster-line.csv`, the smoothed line as a closed centreline file the plant is driven round. |
| `fit_kart.py` | Runs the whole comparison for the earlier and the fitted kart and prints it; `--evidence FILE` keeps the numbers. |

```powershell
.\.tools\tum-python\Scripts\python.exe tools\kart\make_poster_line.py
.\.tools\tum-python\Scripts\python.exe tools\kart\fit_kart.py --evidence out\kart\evidence.json
```

It takes about seven minutes: the plant is driven for three laps at five shares of its envelope, for each kart.

## Tuning a kart yourself

`fd_headless` takes `--car-set NAME=VALUE` (repeatable) after `--profile`: any four-wheel car value by its member name, a tire
value as `tires.NAME`, `front_tire.NAME` or `rear_tire.NAME`, and the configuration's `wheelbase_m`, `max_steering_rad` and
`max_speed_mps`; an unknown name lists the valid ones. The steering rate and the aim of Pure Pursuit are configuration, set
with `--config FILE` (`max_steering_rate_radps=3.0`, `lookahead_base_m=2.0`). For example, a stiffer sideways grip:

```powershell
.\build\core\apps\headless\Release\fd_headless.exe --profile gokartcentralen-realistic --car-set tires.peak_friction_lateral_low=1.4 --car-set tires.peak_friction_lateral_high=1.4 --write-envelope out\kart\envelope-try
```

and `--track tracks/gokartcentralen-goteborg-poster-line.csv --track-width 3 --speed-plan envelope --envelope-fraction 0.9`
drives it round the poster's line at nine tenths of its limit.

## What the fit found

The line is consistent with the real kart; the earlier simulated kart was not. On the line a kart of the rental kart's 261 kg and
14 hp with a grip of 1.0 laps in 34.1 s in the quasi-steady-state model, the operator's own standard for a quick kart being
34.0 s, and the model's gas, coast and brake match the poster's colours to within 3 m over 95% of the lap. The simulator's
plant gave only 37.5 s on the whole of its envelope, because it corners at 0.89 g at best, only 0.46 g at 4 m/s (a tire's slip
angles at a hairpin's radius), and spun when its tires were made softer to help. The fitted kart's tires grip 1.35 sideways and
1.0 along (published kart tires grip more sideways than along: Lot's racing kart 1.5 and 0.9) and its steering is a kart's quick
3 rad/s, which brings the plant's own lap on the line to 34.5 s at the whole of its envelope.
