"""Fit the Göteborg rental kart to the poster's coloured line and to real lap times (decision 0046).

    python fit_kart.py [--evidence FILE] [--headless fd_headless.exe]

Prints, and optionally writes as evidence JSON:

1. what the poster's line asks of any kart: its length, its tightest smoothed radius, the metres of gas, coast and brake;
2. the quasi-steady-state lap on that line for a kart of the rental kart's mass and 14 hp at several grips, against the
   operator's own 34.0 s standard for a quick kart;
3. the project's plant on the line, for the earlier rental kart and for the fitted one: the lap its performance envelope
   allows, then the laps it drives at shares of that envelope, the cones and excursions, and how well its gas, coast
   and brake match the poster's colours at the same places.

The fitted kart's numbers are those of core/vehicle_profiles.cpp; this script is how they were chosen and how they are
checked again. The targets: the whole envelope at about 34.5 s, and the user's real best of 36.38 s at close to nine tenths.
"""

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kart_model as km

OPERATOR_STANDARD_S = 34.0   # Gokartcentralen's Speedkart qualifying time for a standard kart
DRIVER_BEST_S = 36.38        # the user's best of their laps there, Gokartcentralen's own leaderboard
SHARES = (1.0, 0.95, 0.9, 0.85, 0.8)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--headless", default=km.HEADLESS, help="the built fd_headless")
    parser.add_argument("--evidence", help="write the numbers here as JSON")
    parser.add_argument("--out", default=os.path.join(km.ROOT, "out", "kart"), help="where the plant's runs are written")
    parser.add_argument("--profiles", nargs="*", default=["gokartcentralen-rsx2", "gokartcentralen-realistic"])
    args = parser.parse_args()

    line = km.PosterLine()
    smallest = 1 / float(np.abs(line.kappa).max())
    evidence = {"poster_line": {"length_m": line.length_m, "smallest_radius_m": smallest,
                                "gas_coast_brake_m": line.colour_metres()}}
    print("The poster's line: %.1f m, tightest smoothed radius %.2f m, gas %.0f m, coast %.0f m, brake %.0f m" % (
        line.length_m, smallest, *line.colour_metres()))

    print("\nA kart of 261 kg and 14 hp on it, one grip both ways, braking 0.8 g, the 60 km/h controller (quasi-steady state):")
    evidence["quasi_steady_state"] = {}
    for mu in (0.9, 1.0, 1.1, 1.2):
        v, a, t = km.qss_lap(line, mu, 261, 14 * 735.49875, 1650, 0.78, 60 / 3.6, 0.8)
        exact, near = km.agreement(line, km.colours_of(a))
        evidence["quasi_steady_state"][str(mu)] = {"lap_s": t, "colours_exact": exact, "colours_within_3m": near}
        print("  grip %.1f: %.2f s (the operator's standard is %.1f s); colours match %.0f%%, %.0f%% within 3 m" % (
            mu, t, OPERATOR_STANDARD_S, exact * 100, near * 100))

    evidence["plant"] = {}
    for profile in args.profiles:
        table = km.envelope_table(profile, headless=args.headless)
        _, _, whole = km.envelope_lap(line, table, 1.0)
        brake_g = abs(table[8.0][0, 3]) / km.G
        print("\n%s: its envelope allows %.2f s on the line (lateral %.2f g at 6 m/s, %.2f g at 8 m/s; braking %.2f g)" % (
            profile, whole, table[6.0][-1, 1] / km.G, table[8.0][-1, 1] / km.G, brake_g))
        entry = {"envelope_lap_s": whole, "braking_g": brake_g, "lateral_g": {str(v): table[v][-1, 1] / km.G for v in (4.0, 6.0, 8.0, 12.0)},
                 "driven": {}}
        for share in SHARES:
            out = os.path.join(args.out, "%s-%d" % (profile, round(share * 100)))
            run = km.drive_plant(profile, share, out, headless=args.headless)
            colours = km.plant_colours(line, out) if run["laps_s"] else None
            flying = run["laps_s"][-1] if run["laps_s"] else None
            entry["driven"][str(share)] = {"flying_lap_s": flying, "cones": run["cones"], "off_course": run["off_course"],
                                           "rear_sideslip_rad": run["rear_sideslip_rad"], "colours": colours}
            print("  %3d%% of the envelope: flying lap %s; %d cones, %d off course, rear sideslip %.2f rad%s" % (
                share * 100, ("%.2f s" % flying) if flying else "none", run["cones"], run["off_course"], run["rear_sideslip_rad"],
                (", colours match %.0f%% (%.0f%% within 3 m)" % (colours["exact"] * 100, colours["within_3m"] * 100)) if colours else ""))
        evidence["plant"][profile] = entry

    if "gokartcentralen-realistic" in evidence["plant"]:
        driven = evidence["plant"]["gokartcentralen-realistic"]["driven"]
        laps = [(share, driven[str(share)]["flying_lap_s"]) for share in SHARES if driven[str(share)]["flying_lap_s"]]
        # the share at which the plant laps as the user's best did: interpolate
        shares, times = zip(*sorted(laps, key=lambda p: p[1]))
        at_best = float(np.interp(DRIVER_BEST_S, times, shares))
        evidence["driver_best_share"] = at_best
        print("\nThe fitted kart laps in %.2f s at the whole of its envelope; %.2f s, a driver's best, is %.0f%% of it." % (
            driven["1.0"]["flying_lap_s"], DRIVER_BEST_S, at_best * 100))
    if args.evidence:
        with open(args.evidence, "w", encoding="utf-8") as f:
            json.dump(evidence, f, indent=2)
            f.write("\n")
        print("Evidence written to", args.evidence)


if __name__ == "__main__":
    main()
