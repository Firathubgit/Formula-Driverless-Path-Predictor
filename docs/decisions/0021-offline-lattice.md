# 0021: An offline lattice along the line the car drives

Date: 2026-09-17. Status: accepted for TrackWayFastPlan Phase 5.1, amended the same day by decision 0022: the reference
deviation term saturates at 40000 per metre, 4 m off the reference, instead of 10000. The evidence below predates that.

Phase 5 is a local planner that looks a few metres ahead: re-plan from the actual state every control tick over a
spatial lattice around the racing line, return an action set, and give each action its own speed profile. Phase 5.1 is
the offline lattice. Source: Stahl et al., ITSC 2019, as implemented in race_stack's vendored copy of TUM's
`graph_ltpl/offline_graph`, reimplemented because that code is LGPL-3.0. How: layers along the Phase 3 racing line,
spaced more densely in corners; nodes spread across each layer's normal within the track; edges as polynomial splines,
using the Werling polynomials; edges exceeding the steering curvature pruned; an offline cost from race-line deviation
and average and peak curvature. Phase 5's visible item that belongs here: the lattice drawn faintly ahead of the car.

The plan's recommended order puts Phases 4 and 5 in either order. Phase 4.1 needs the fastest-lap download, which the
user has not approved; Phase 5.1 needs no download, so it comes first.

TUM's `gen_node_skeleton.py`, `gen_edges.py`, `prune_graph.py`, `gen_offline_cost.py`, `main_offline_callback.py`,
`variable_step_size.py` and ForzaETH's `ltpl_config_offline.ini` were read for structure. race_stack's frenet-planner
(Apache-2.0) was read for its quintic; the polynomial below is written from its closed form, and no code was copied from
either.

## Decision

**`make_lattice(reference, config, options)`** in `fd_core` (`core/lattice.hpp`) returns plain data: layers, their
nodes, and the edges between them with their samples and offline cost, plus how many edges were generated and removed
for each reason. `Lattice::edges_from(layer, node)` lists a node's edges. It needs no dependency and runs no search.
Domain terms are in `CONTEXT.md`, started this session.

1. **Layers** start at the reference's start and follow each other at `straight_layer_spacing_m` (6 m), or at
   `curve_layer_spacing_m` (4 m) when the reference curves more than `curve_threshold_1pm` (0.02 1/m) anywhere within the
   straight spacing ahead, as TUM's variable step size does. A last gap longer than its spacing is split in two.
2. **Nodes** lie on each layer's left normal every `lateral_spacing_m` (0.5 m), as whole multiples of it, so the
   reference node is exactly at offset zero, and as far toward each corridor edge as `vehicle_half_width_m` (0.9 m, the
   local planner's) allows. The corridor comes from the track's own edges when it has them (decision 0020), otherwise
   half the width. A reference closer to an edge than the half width anywhere is refused.
3. **Edges** join each node to every node of the next layer within `lateral_change_per_metre` (0.3, TUM's) of sideways
   movement per metre of station, measured from where the start node's slope would carry it; the last layer joins the
   first. An edge is an offset from the reference that changes with station as the quintic Hermite curve with each end's
   node slope and zero second derivative at both ends, Werling et al. 2010's lateral quintic in station rather than time.
   Chained edges therefore keep continuous heading and curvature. Samples are no further apart than `sample_spacing_m`
   (0.5 m); each carries the edge's own curvature in closed form: with `P = 1 - k d`,
   `(P (k P + d'') + d' (k' d + 2 k d')) / (P^2 + d'^2)^(3/2)`, from the reference's curvature `k`, its rate `k'` and the
   offset's derivatives.
4. **Node slopes.** A node's slope is zero at the reference node and, toward each corridor edge, a growing share of that
   edge's own slope, all of it at the usable limit, TUM's `variable_heading`. On a centreline every slope is zero.
5. **Removal**, counted per reason: edges curving beyond `tan(max_steering_rad)/wheelbase_m`, or reaching the
   reference's centre of curvature; then edges whose samples leave the corridor less the half width between layers; then,
   repeatedly, edges into nodes nothing leaves and out of nodes nothing reaches, TUM's pruning of dead ends.
6. **Offline cost**, TUM's terms with ForzaETH's weights: for an edge of length `L` ending `d` off the reference,
   `7500 (mean |curvature|)^2 L + 2500 (max - min curvature)^2 L + 0 L + min(10000 |d|, 10000) L`, every weight in
   `LatticeCostWeights`.

**Headless.** `fd_headless --write-lattice DIR`, with `--track` and `--line racing` as for a run, lays the lattice along
the line a run would drive and writes `lattice.json`, `layers.csv`, `nodes.csv`, `edges.csv` and `edge_samples.csv` into
a fresh directory without a lap, and prints the counts. `--obstruct`, which the offline lattice ignores, is refused with
it, as is a non-empty directory.

**Desktop.** The bridge lays the lattice along the track shown, live or recorded, whenever that track changes: the
preset, the racing line, or a recording's track. The scene draws, as faint single-pixel lines, the edges leaving the
layers within 60 m ahead of the car's station, the prediction's reach at the speed cap, and redraws them only when the
car passes a layer. The legend names it.

## Found by building it

- **Nodes that follow the racing line's heading cannot keep their place on the track.** The first lattice gave every node
  the reference's heading, as ForzaETH's configuration does (`variable_heading=False`). On the preset centreline that
  lost nothing. On the racing line, which crosses Foundry Circuit at about 0.24 m per metre, only 1075 of 1785 nodes
  kept an edge out, 1257 edges were dead ends, and in one layer the usable nodes spanned an eighth of the corridor. A path
  at a fixed place on the track moves across the line at that rate, and edges forced parallel to the line at every node
  must bend twice per layer to do it. With node slopes following the corridor, 1736 nodes stayed usable, and with 4 m
  corner spacing, 1585 of 1586.
- **3 m between layers is too short for this car.** A step of one node, 0.5 m, over 3 m peaks at about 0.32 1/m
  (`5.77 × 0.5 / 3²`), beyond the default car's 0.236 1/m, so on 3 m corner layers only parallel edges survived. At 4 m
  it is about 0.18 1/m, which with the racing line's 0.03 1/m stays within the limit. ForzaETH's 1/10-scale values, 1 m
  and 0.1 m for a 0.7 m turning radius, use about 40% of their car's limit for such a step; 4 m uses about 76% of ours.
- **The inside of a tight corner traps nodes.** 5.5 to 6 m inside the preset's 18 m corners, a metre of reference is
  about 0.7 m of path, so any step across curves several times the limit, and those nodes are dead ends. That is
  geometry, not a defect.
- **Cost.** Laying Foundry Circuit's lattice takes under 0.01 s on the reference host (measurement only).

## Verification

`tests/lattice_tests.cpp` (`fd_lattice`, 10 groups, 0.3 s):

- layers on the preset follow within 6 m, and within 4 m wherever it curves more than 0.02 1/m in the 6 m ahead: 100
  layers, 54 before straights and 46 before corners;
- nodes lie every 0.5 m from -4 to 4 m on the preset and from -6 to 2 m on a reference 2 m right of the corridor's
  centre, each layer with its reference node at zero, on a unit normal square to the reference and to its left;
- 9412 edges are generated on the preset, one per pair of nodes within the allowance; each starts and ends at its nodes'
  positions within 1e-9 m, its offset moves steadily between them, its samples are at most 0.5 m apart, and its length is
  measured along them;
- beside a 40 m circle traced every 0.05 m, parallel edges have the curvature of a circle of radius R - d within 4e-18
  1/m, and edges changing offset, up to 0.447 1/m, match the curvature through their neighbouring samples within
  0.0015 1/m; dropping the offset's second derivative from the formula fails this group;
- at the default car's 0.236 1/m, 1690 of 9412 edges are removed and the sharpest left curves 0.2356 1/m; at 0.098 1/m
  5984 are removed; the reference node's edge to the next reference node survives at every layer and lies on the
  reference;
- narrowing the corridor's left side to 2 m between two layers removes 5 edges that would cross it, and every edge left
  stays inside the corridor at every sample; a constant corridor removes none;
- that narrowing leaves 61 dead ends, removed until every node is either reached and left or unused, and every generated
  edge is kept or counted;
- each edge's cost is the stated formula of its samples; on a straight, 6 m between layers with a length weight of 3,
  staying on the reference costs 18, ending 0.5 m off 30621, 1 m off 62962, and running parallel 2 m off 120018;
- beside a corridor whose edges swing 2 m either way four times a lap, edges leave and reach their nodes with their
  share of the edge's slope, up to 0.099, within 0.0005; 1637 of 1648 nodes stay usable, the rest deep inside corners
  where the path is at most 0.81 m per metre of reference; with node slopes forced to zero this group fails;
- options out of range and a reference closer to an edge than the half width at one sample between layers are refused
  with their reason.

`fd_raceline` gains a group: the lattice along the Foundry racing line has 96 layers, 1585 of 1586 nodes usable and
7034 edges, and the narrowest layer's usable nodes span 94% of its corridor; with node slopes forced to zero it fails.
`fd_recording_contract` writes the preset's lattice and checks each table's rows against the reported counts and
`lattice.json`, refuses a non-empty directory and `--obstruct`, and lays the racing line's lattice within its corridor's
edges. The desktop UI verification (404 checks) checks that the lattice is laid along the preset at start, along the
racing line the car drives and along a replayed recording's track, that the scene draws exactly the edges leaving the
layers within 60 m ahead, that nothing drawn is further than the horizon from the car, and that the drawing moves on as
the car passes layers; its capture is `docs/assets/lattice.png`.

Lattices written with `--write-lattice` into fresh `out/lattices` directories:

| Directory | Line | Layers | Usable nodes | Edges kept | Removed: steering / corridor / dead ends |
| --- | --- | ---: | ---: | ---: | ---: |
| `preset` | preset centreline | 100 | 1700 of 1700 | 7722 of 9412 | 1690 / 0 / 0 |
| `centreline` | conditioned centreline | 100 | 1700 of 1700 | 7720 of 9380 | 1660 / 0 / 0 |
| `preset-racing-line` | racing line of the preset | 96 | 1585 of 1586 | 7034 of 8889 | 1854 / 0 / 1 |
| `racing-line` | racing line of the conditioned centreline | 96 | 1584 of 1585 | 7032 of 8880 | 1847 / 0 / 1 |
| `traced-racing-line` | racing line of the noisy trace | 97 | 1604 of 1604 | 6982 of 8901 | 1919 / 0 / 0 |
| `racing-line-margin-0.3` | racing line, 0.3 m margin | 97 | 1576 of 1583 | 6815 of 8780 | 1951 / 1 / 13 |

Each took 0.008 to 0.011 s to lay; the sharpest edge kept in each curves 0.2358 1/m against the 0.2358 1/m limit, and
usable nodes average 4.3 to 4.5 edges out. Source fingerprint
`dbf72e2b12aa058ba8dbcf5ea9725a52f36e20d267d5db52d982db16d6b13a39`. `scripts/build.ps1 -Desktop`: 18/18 suites passed,
263.25 s of tests, 404 QML checks, no runtime warnings, the UI check taking 58.1 s of its 120. `scripts/build.ps1`: 17/17
suites passed, 203.20 s of tests. All 135 recordings in `out/runs` still load and validate with this build. The build
scripts now print a blank stderr line as blank rather than as `System.Management.Automation.RemoteException`.

## What this is not

Not a planner: nothing searches the lattice, and the car still chooses among five lateral offsets when a blockage is
stated (decision 0004); Phase 5.2 searches it online and returns an action set, Phase 5.3 gives each action its own
speed profile. Not a claim that an edge is drivable at speed: edges are pruned by the kinematic steering limit only, with
no speed, grip or envelope. Not collision checking: nodes and edges keep the rear axle a half width inside the corridor,
not a swept body. The offline cost's weights are ForzaETH's scaled values, not tuned for this car.

## Consequences

- Phase 5.2 can search the lattice from the node nearest the car and prune edges against stated blockages, which will
  need to be stated in the lattice's reference coordinates to work on the racing line.
- Any future reference, including a drawn track, gets a lattice the same way.
- `CONTEXT.md` now holds the project's domain vocabulary; later terms (action, action set) belong there.
