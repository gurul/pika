# Improving the active-sonar car

Research swarm and implementation review, 2026-09-22.

The immediate defect was an information loop: select a heading, find its allowed
speed too low, then keep pointing the sonar at that rejected heading. The first
repair also had to account for the time spent looking sideways; otherwise it
alternated brief drive attempts and stops without gaining useful information.

The implemented host-side repair ranks executable choices first, includes the
scan-time allowance before committing to a heading, and keeps widening the search
when nothing is executable. It keeps the existing close-obstacle, freshness and
unknown-space stops. No battery-voltage cutoff was reinstated.

## What the research supports

| Approach | Useful idea for this car | Status and limits |
|---|---|---|
| [Active Sense and Avoid (alphaXiv)](https://www.alphaxiv.org/abs/2010.04977) | Schedule sensing separately from steering: inspect unseen directions while stopped and prioritize the travel corridor while moving. | Applied as a simpler moving/stopped search distinction. The published system uses a rotating stereo camera on a flying robot; its performance does not transfer to this sonar. |
| [VFH+](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated1/borenstein_VFHplus.pdf) | Retain obstacle enlargement and steering preference, but restrict commitment to usable directions. | Existing VFH-like memory retained. Heading clearance alone does not establish clearance around an approaching curve. |
| [Dynamic Window Approach](https://www.ri.cmu.edu/pub_files/pub1/fox_dieter_1997_1/fox_dieter_1997_1.pdf) | Filter stopping-admissible, reachable motions before optimizing preference. | This ordering informed the fix. A full dynamic window needs a defensible velocity estimate; PWM is not measured speed. |
| [Curved Openspace](https://www.frontiersin.org/journals/neurorobotics/articles/10.3389/fnbot.2022.850013/full) | Evaluate motor paths and aim sonar at the observations needed for the promising path. | Best next prototype: a small set of calibrated arcs. The paper used a richer three-transducer sonar head. Its effective ping rate dropped from 12 Hz without head movement to 3 Hz for large movements, illustrating why telemetry rate is not panoramic sensing rate. |
| [Curvature-Velocity Method](https://www.cs.cmu.edu/~reids/papers/cvm.pdf) | Check distance along curves rather than only a final direction. | Useful for the arc prototype. The original robot had denser ranging and dead reckoning. |
| [Single-sonar edge search](https://link.springer.com/article/10.1007/s41870-020-00513-w) | Widen the scan until an obstacle edge is found, then confirm it with repeated measurements. | Progressive search and confirmation fit this sensor. One-degree sweeps and blind reversing are unsuitable here. |
| [Regulated Pure Pursuit](https://www.alphaxiv.org/abs/2305.20026), [Dynamic Window Pure Pursuit](https://www.alphaxiv.org/abs/2601.15006) | Preserve intended curvature while regulating speed and acceleration. | Later control work: both expect a path/state representation that this reactive controller does not yet provide. |

These applications and rankings are engineering judgments, not claims of reproducing
the papers' results. The sonar, navigation and independent code-review agents wrote
separate evidence reports in `build/sonar-methods-research.md`,
`build/navigation-methods-research.md`, and `build/stall-review.md`.

## Why increasing the speed cap did not help

At the recorded 44 cm clearance, the current model reserves 30 cm and estimates
braking with 45 cm/s² deceleration and 0.30 cm/s per PWM. It permits 68.72 PWM
with the sonar centered and 41.05 PWM during a side glance, before the steering
penalty. Both are below the configured 70 PWM runnable-speed assumption. These
are model calculations, not measured stopping distances. The repair finds a
different observed route; it does not relabel the rejected corridor as safe.

The recorded obstacle also occupies about 42 degrees on either side after sonar
ambiguity and body-width enlargement. Looking only 20 degrees sideways repeatedly
cannot resolve a route beyond it. Wider observations are necessary.

## Next work, ranked

1. **Establish a reliable motion-time link.** New command logs record reply strings
   and latency; each decision records the exact telemetry snapshot it consumed.
   A lost pan acknowledgement can be confirmed by newer matching angle telemetry
   while wheels remain stopped. Missing all telemetry still aborts. Physical
   retests exposed loss of all replies after motion began; its cause is not yet
   established. Fix or characterize this before increasing speed.
2. **Calibrate motor and braking response.** Measure per-wheel deadband, gyro yaw
   rate and external forward displacement over repeated runs. Use measured bounds
   in the braking model. The gyro can measure turning, not forward speed.
3. **Prototype five to seven motion arcs.** Evaluate straight, shallow and tighter
   curves against the whole swept body footprint; schedule sonar to resolve the
   best candidate's missing observations. Use gyro feedback for angular rate.
4. **Measure scan deadlines.** Record command-to-two-settled-readings latency by
   angle, including its upper tail. Use this to budget speed before looking away.
5. **Improve state and coverage if needed.** Encoders and denser ranging address
   limits a different planner cannot remove. They are future hardware work.

## Evidence and evaluation

The regression suite contains blocked and unknown-space controls, a discoverable
wide opening, the 44 cm lock and 50–60 cm variations, lost-pan-ACK confirmation,
and decision-input consistency. The recorded stationary segment is replayed without
inventing measurements for new requested angles. Simulation separately supplies
independent geometry and ground-truth contact/progress checks.

For further comparisons, freeze inputs and measure time to a usable opening,
unique settled scan angles, motion duty, externally measured distance, stops per
meter, yaw reversals, minimum clearance, and response to stale/invalid input.
An all-blocked scene must remain stopped; less stopped time alone is not success.
Neither simulation nor sent PWM commands demonstrate smooth physical navigation.
