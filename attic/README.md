# Attic

Things that were built, tried on the bench and dropped. Not in the build, not tested; kept because the project has no history yet.

- `two-magnet/`: `MagProbeFit` (a rigid two-magnet probe fitted as one thing: tip position, shaft direction, roll, and a learnable shape - spacing, each magnet's angle to the shaft, the back magnet's turn round it) and its Unity tests. Dropped on 2026-09-17 after two bench sessions: on this array the fields such a probe makes are 0.1-0.3 mT against the array's own ~0.05 mT of unexplained error, so no shape fitted better than 5-8 % and the tip came out no better than one magnet's. The full account is in `docs/wireless-probe-sensing.md`. Orientation now comes from the one magnet's pole (magnetised along the shaft), `MAGLOC_TIP_OFFSET_MM`.
