# 0055 — Forty percent more tire grip in the desktop realistic kart

2026-10-04. The user requested 40% more overall grip because the kart felt like
ice, and described a roughly 200 kg kart plus a 60–80 kg driver.

The desktop Realistic kart multiplies both axles' longitudinal and lateral
friction coefficients, including the extrapolation floor, by 1.4. Longitudinal
peak friction is now 1.4 and lateral peak friction 1.89. At the same load and
slip, both tire-force components rise by 40%, including combined and sliding
slip. Peak slips and sliding fractions are unchanged. This is a user-chosen
tune, not a new measured fit. The offline `gokartcentralen-realistic` profile
remains the historical lap-fit reference; its earlier lap times do not describe
this desktop tune.

Mass stays 261 kg: the existing 186 kg kart plus 75 kg driver already falls
within the requested approximate range. Power, speed cap, inertia, coasting
steering allowance, vibration settings and wall contacts are unchanged.
The grip aid derives its envelope from the tuned car. Ordinary tire dynamics
still advance the plant: no forced rolling, yaw blending or Forgiveness.

Kart entry and setup reset obtain the same fresh tune from the reference, so
resetting or switching kart variants never compounds the multiplier. UI checks
compare actual configured tire forces to the baseline across loads and pure,
combined and large slips, and check reset/switch behavior and retained mass.
