# Active sonar steering — research and implementation

Research date: 2026-09-22. Source discovery used alphaXiv pages and search;
primary paper abstracts/full text on arXiv were checked where alphaXiv did
not expose the paper text. No alphaXiv connector is installed in this session.

## What the research says

1. **An Active Sense and Avoid System for Flying Robots in Dynamic
   Environments**, Chen et al., 2020/2021.
   [alphaXiv](https://www.alphaxiv.org/abs/2010.04977),
   [primary abstract](https://arxiv.org/abs/2010.04977).
   Uses an independently rotating stereo camera. Its sensing objective
   balances obstacle tracking, heading observation, exploration and sensor
   movement; planning checks collision uncertainty. This supports choosing
   where to look independently of where to drive. It is a flying robot with
   richer depth observations, not validation of our ultrasonic hardware.

2. **Safety-Aware Perception for Autonomous Collision Avoidance in Dynamic
   Environments**, Bena, Zhao and Nguyen, 2024.
   [alphaXiv](https://www.alphaxiv.org/abs/2403.13929),
   [primary full text](https://arxiv.org/html/2403.13929v1).
   Optimizes sensor pointing to observe collision risk using control barrier
   functions and field-of-view quality. The transferable idea is to prioritize
   observations that could change the next control decision. Our small
   heuristic is not their CBF controller and inherits none of its guarantees.

3. **VFH+ based shared control for remotely operated mobile robots**, Pappas
   et al., 2020.
   [alphaXiv](https://www.alphaxiv.org/abs/2011.05228),
   [primary abstract](https://arxiv.org/abs/2011.05228).
   Blends VFH+ obstacle avoidance with an operator's commands and evaluates
   safety/task completion in a disaster-response setting. This is the reference
   named in the existing roam code, not the original VFH/VFH+ publication.
   Keep the polar-direction choice and preference for the previous direction;
   do not let an empty, unobserved histogram sector masquerade as free space.

## Adaptation to this car (engineering decisions, not paper results)

At 12 incoming readings per second the car still sees only one sonar cone at
a time. The sensing budget includes servo travel, acoustic measurement, serial
transport and verification of an opening. More repeated front readings do not
provide knowledge about the left or right side.

The proposed behavior is: approach at bounded speed -> glance 20 degrees ->
recheck forward -> look 40 degrees if needed -> confirm an opening -> gradually
curve toward that opening. If the observed braking corridor runs out, stop the
wheels and keep looking. A confirmed wide side opening can trigger short
gyro-observed turning pulses in place when a forward arc lacks braking room.
No automatic blind reversing is added. A dead end
may still require manual repositioning; this version does not promise escape
from every cul-de-sac.

Source diagnosis:

- V3 N=5 changes the global motor mode, then blocks for 200 ms. Gyro updates,
  telemetry and motor timeout processing cannot run during that delay.
- V3 telemetry has no pan angle. A side reading can accidentally refresh an
  ahead clearance estimate in naive stream consumers.
- Original roam starts unobserved bins as free and does not rotate observations
  with gyro heading. Its sonar range constant still says 150 cm.
- Dash repeatedly filters the same latest frame as if it were new, and its
  curve direction comes from a past stop rather than a current side observation.
- The Safety differential-drive watchdog checked only *before* its deadline;
  an expired renewal could therefore escape the host watchdog. This is fixed.

Implementation:

- New N=28 D1=angle pan command is nonblocking and leaves the drive mode alone.
  Old N=5 is retained for compatibility. The new controller never uses it.
- N=25 D1=100 D2=1 selects extended telemetry:
  `{T_dist_yaw10_L_M_R_ground_mV_pan_seq_sample_ms}`.
  Pan is negative during a conservative settling interval. Positive means the
  interval elapsed; it is **not measured shaft position**. Sequence and sample
  milliseconds are unsigned 16-bit values and wrap. The host rejects duplicate,
  old, moving-servo and delayed observations. Zero distance means unknown.
- Streamed wheel commands require renewal within 500 ms on the UNO in active
  mode. The host also stops on stale data and expires differential renewals.
- Observations retain gyro-relative world bearing, expire and lose usable range
  as the car spends its conservative movement budget. Unknown directions are
  ineligible for steering. The smaller of two readings supplies clearance;
  two new far readings are required before it can increase. A close return
  can stop immediately. No echo does not establish free space.
- The steering score favors clearance, small changes and staying with the last
  chosen side. Obstacles are enlarged by an approximate body radius plus sonar
  angular uncertainty. Wheel increases are rate limited; braking is immediate.
- Speed is bounded by `v*latency + v²/(2*deceleration) <= clearance - margin`,
  with extra time reserved when looking away. These physical parameters are
  conservative starting assumptions and require on-floor measurement.

## Limitations and next measurements

The simulator uses planar geometry, a cone of ideal rays, and approximate wheel
kinematics. It cannot validate echoes from fabric/glass, floor reflections,
wheel slip, motor stiction, servo backlash or moving people. The line sensors
are not dependable cliff sensors. Use a supervised level area away from stairs.

Before increasing speed, measure command-to-response latency, pan settling,
low-speed stiction, actual stopping distance and telemetry dropout under motor
load. A motor-current/no-motion observation or wheel encoders would improve
stall detection; unchanging sonar alone cannot distinguish a stopped car from
travel parallel to a wall.

The available LiDAR is TF-Luna (identified from the user's photos), a single
beam sensor, not a 360-degree scanner. It could improve servo scans, but neither
it nor this reactive controller alone delivers a reliable room reconstruction.

Implementation evidence is in GATES-active-roam.md and build/active-roam-simulation.
Hardware state and any uncompleted physical checks must be recorded separately.

## Offline results and unresolved behavior

The three required scenarios (visible-wall corridor, central obstacle, noisy
central obstacle) pass the progress, no-contact and speed/slew assertions.
These are synthetic outcomes, not measured car performance. Full trajectories
and counts are in `build/active-roam-simulation/*.json`.

Five additional variations changed the noise seed or wheel response. None
made contact. Four passed the obstacle within 30 simulated seconds. The faster
plant (0.30 cm/s per PWM, versus 0.24 nominal) did not: it reached a boundary
and remained in cautious recovery. Some runs still stop frequently. This is a
known performance limitation; it must not be reported as seamless navigation.
The real charged-battery response needs measurement before further tuning.
