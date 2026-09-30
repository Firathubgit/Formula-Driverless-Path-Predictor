# 0043 — Five local Forgiveness presets

2026-09-28. The user requests a very simple local memory for up to five saved
Forgiveness setups. Use five numbered slots, Save (Replace for an occupied slot),
Load and Delete in the existing settings panel. No accounts, database server,
names, synchronization or extra dialog. Occupied slots have a dot.

Each slot keeps Overall level, Overall/Manual selection, all five manual values
and the toggle state. Load enables Forgiveness and queues the saved settings
through the existing tick-boundary setter; saving/deleting changes no physics.
Neither action resets the drive or lap clock. Slots remain available even when
Forgiveness is switched off and survive app restarts and leaving kart mode.

`AssistancePresets` uses one versioned JSON file with exactly five optional slots
in Qt's AppLocalDataLocation: on this Windows installation,
`%LOCALAPPDATA%/Formula Driverless/Formula Driverless/forgiveness-presets.json`.
It validates types and every range on read and before save, bounds input size,
and uses QSaveFile's atomic commit before updating its in-memory slots. Read or
write errors are shown in the panel; unreadable files are not overwritten.
This is desktop-local preference storage, not a simulation recording.

The storage tests use temporary directories. Desktop UI verification passes a
separate temporary path to Bridge so automated Save/Delete never touch the
person's real presets. Tests cover all five slots, reload persistence, replacement,
deletion, bounds, invalid settings, malformed files and storage failure. UI
checks use the real buttons, change then reload a manual setup, save/delete slot
five, and verify that loading queues a tick without moving/resetting the kart.

As a one-time local convenience, the user's screenshot settings were saved into
the previously nonexistent real slot 1: Manual, acceleration 34, braking 14,
steering 40, grip 100, speed 79; Overall retains 100. No existing file was replaced.
This personal setup is local data, not a hardcoded application default.

Build and render results are recorded in `docs/STATUS.md`.
