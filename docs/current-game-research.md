# Implementation notes for this autonomous

This code revision is based on your specific constraints and robot photos/config screenshots:

1. Use wheel motor encoder feedback only.
2. No inertial/vision/distance sensors in the autonomous control loop.
3. Collect ground balls and cycle scoring between high/middle goals.
4. Run top output upward for high cycle, then downward for middle cycle.
5. End by parking in the red parking zone.

## What changed from the previous version

- Removed inertial-based `smartdrive` usage.
- Replaced heading turns with wheel-encoder arc turns.
- Added explicit conveyor modes for:
  - collect/upward feed
  - high-goal feed
  - middle-goal down-feed at top stage
  - reverse purge
- Added a longer 1-minute style autonomous cycle sequence with a park finish.

## Field reality reminder

Because encoder-only dead-reckoning drifts with slip/contact, this route should be tuned with iterative field tests. Start by tuning wheel diameter and track width constants, then trim each turn/distance per cycle.
