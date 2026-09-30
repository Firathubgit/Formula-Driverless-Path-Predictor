Foundry Circuit is generated deterministically in `fd::make_preset_track()`.
It is a closed counterclockwise rounded polygon with a long main straight and
18, 24, 26, 32 and 38 metre constant-radius bends. Samples are spaced at most
0.6 metres along exact line/arc lengths. Width is 10 metres.

The route is its centerline, not an optimized racing line. Tangency is continuous;
curvature has steps at straight-to-arc joins. Pure Pursuit tracks the sampled path,
and conservative grip allocation leaves room for tracking error. A headless run
exports the exact track samples as `track.csv`, and `fd::load_recording` reads those samples
back for replay. There is still no external or editable track loader.

## Gokartcentralen Göteborg

`gokartcentralen-goteborg.csv` is the indoor kart track at Gokartcentralen Göteborg (Bergslagsgatan 6), traced from the
operator's published track poster by `tools/tracks/trace_poster.py` (decision 0038): the middle of the drawn asphalt,
clockwise from the start line toward T1, resampled every metre and scaled to the published 400 m. The published width is
6 m. Conditioned with 0.02 m of smoothing it is 399.5 m with the poster's ten turns, hairpins of 3.5 to 6 m at the centre.
The desktop drives it in gokart mode; the headless runner with `--track tracks/gokartcentralen-goteborg.csv --track-width 6`.
