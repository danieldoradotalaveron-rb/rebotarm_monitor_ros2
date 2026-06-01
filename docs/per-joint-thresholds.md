# Per-joint diagnostic thresholds (B601-DM)

Why the monitor ships per-joint torque and velocity thresholds, where the
numbers come from, and how to think about them. This is **explanation**: it
does not replace the parameter reference in
[`src/rebotarm_monitor/README.md`](../src/rebotarm_monitor/README.md).

## Why per-joint, not global

The B601-DM uses two motor families:

- **joint1, joint2, joint3** — Damiao DM-J4340P-2EC (4340P-class).
- **joint4, joint5, joint6** — Damiao DM-J4310-2EC (4310-class).

A single global threshold either over-warns on the wrist (small motor) or
under-warns on the shoulder (large motor). The monitor ships one map per
joint instead.

## Shipped values

Defined in `rebotarm_monitor/parameters.py` as Python constants. They are
**not** ROS parameters — `rclpy.declare_parameter` rejects `dict`, so
YAML/launch/CLI overrides do not apply to the maps themselves. Edit the
constants and rebuild. The **profile selection** *is* a ROS parameter
(`payload_profile`) so users switch profiles without editing code.

| Map | joint1–3 (4340P) | joint4–6 (4310) |
|-----|------------------|-----------------|
| `per_joint_max_abs_velocity_rad_s` (profile-independent) | 6.0 rad/s | 20.0 rad/s |
| `per_joint_max_abs_torque_nm` (varies by profile) | see "Payload profiles" | see "Payload profiles" |
| `per_joint_idle_torque_warn_nm` (varies by profile) | see "Payload profiles" | see "Payload profiles" |

Unknown joints fall back to the global `idle_torque_warn_nm` and
`max_abs_joint_torque_nm` scalars in `config/monitor.yaml` (those globals are
**not** profile-aware; they exist for non-B601 setups).

## Where the velocity numbers come from

Damiao datasheets (also published on the Seeed Studio wiki) at 24 V supply:

| Motor | No-load | Rated | Datasheet VMAX |
|-------|---------|-------|----------------|
| DM-J4340P-2EC | 52.5 rpm = 5.50 rad/s | 36 rpm = 3.77 rad/s | 8 rad/s |
| DM-J4310-2EC | 200 rpm = 20.94 rad/s | 120 rpm = 12.57 rad/s | 30 rad/s |

The shipped WARN threshold sits just above the published 24 V no-load speed
for each family (6.0 vs 5.50 rad/s, 20.0 vs 20.94 rad/s). The intent is to
flag the joint moving well outside its design envelope at this supply.

## Payload profiles

The torque thresholds depend on what the gripper is carrying. A single set
of numbers is either too tight (false WARNs every time the operator picks
up a tool) or too loose (real overload hidden by a generous limit). The
monitor ships **three discrete profiles** keyed to the expected payload mass.

### Selecting a profile

The launch argument `payload_profile` switches between them at startup:

```bash
ros2 launch rebotarm_monitor monitor.launch.py payload_profile:=medium
```

Valid values (lowercase, whitespace tolerated): `light` (default), `medium`,
`rated`. Unknown values fail the node at startup with a clear error rather
than silently falling back. In `rqt_robot_monitor`, the active profile is
shown under **RebotArm → System → monitor_config** (Message column); each
per-joint row also carries a `payload_profile` KeyValue.

The profile is **static per session**: changing payload mid-run requires a
node restart. Payload also does not change between commands in normal use,
so this is acceptable; if dynamic switching ever becomes necessary it would
land as a separate topic, not a parameter reload.

### Profile reference

The values below were computed from URDF gravity (see "Derivation" below).
**Velocity, idle_velocity, position_jump and torque_jump are not profile-
aware**: motor envelope and discontinuity thresholds do not change with
payload.

| Profile | Payload | Use case | Idle j1/j2/j3/j4/j5/j6 (Nm) | Max j1-3/j4-6 (Nm) |
|---------|---------|----------|------------------------------|---------------------|
| `light` (default) | 0.5 kg | Teleop / data collection / demos / no significant payload | 1 / 8 / 8 / 2.5 / 1.5 / 1 | 9 / 3 |
| `medium` | 1.0 kg | Pick-and-place with moderate tooling (camera, small fixture) | 1 / 10 / 10 / 3 / 2 / 1 | 12 / 4 |
| `rated` | 1.5 kg | Manufacturer spec maximum payload / heavy tool operation | 1 / 14 / 12 / 4 / 2.5 / 1 | 18 / 5.5 |

Each profile keeps `idle < max` so the diagnostic sequence
`elevated → WARN idle → WARN high torque` stays ordered.

### Derivation

The shipped values were computed by running static-gravity forward dynamics
over the URDF (`src/rebotarm_bringup/description/urdf/reBot-DevArm_fixend.urdf`):

- Sampled ~16k poses across the reachable joint workspace.
- For each pose, computed `|τ_gravity(q)|` per joint from link masses + COMs.
- Re-ran for each payload (0.5 / 1.0 / 1.5 kg added at the end-effector).
- Idle threshold ≈ `p95(|τ|)` capped below the max threshold (so absolute
  high-torque WARN can still fire above idle).
- Max threshold = motor rated (light), 1.33× rated (medium), 2× rated for
  4340P or ~80 % of peak for 4310 (rated). Always below datasheet peak.

Reference numbers (p95 of `|τ|` across the reachable workspace):

| Joint | Motor | 0.5 kg p95 | 1.0 kg p95 | 1.5 kg p95 | Rated / Peak |
|-------|-------|------------|------------|------------|--------------|
| joint1 | 4340P | 0.00 | 0.00 | 0.00 | 9 / 27 Nm |
| joint2 | 4340P | 17.25 | 20.03 | 22.91 | 9 / 27 Nm |
| joint3 | 4340P | 8.56 | 10.35 | 12.21 | 9 / 27 Nm |
| joint4 | 4310 | 2.72 | 3.57 | 4.42 | 3 / 7 Nm |
| joint5 | 4310 | 1.20 | 1.72 | 2.24 | 3 / 7 Nm |
| joint6 | 4310 | 0.00 | 0.00 | 0.00 | 3 / 7 Nm |

For `joint2` the URDF p95 already exceeds the motor's rated torque at any
payload. That is a real limit of the arm: in some extreme back-folded poses
the shoulder motor cannot statically hold the weight at rated torque. The
idle thresholds trip deliberately in those poses; absolute-torque WARN takes
over beyond max.

### Caveats

- URDF masses are CAD-exported from SolidWorks, not measured. Real masses
  can differ by ±10–20 % (cables, motor potting, end-effector mods not in CAD).
- Profiles are still **not** empirically validated. The baseline TODO (record
  idle torque on the real arm across representative poses with payload)
  supersedes the URDF computation when available.
- Globals in `monitor.yaml` (`idle_torque_warn_nm`, `max_abs_joint_torque_nm`)
  are **not** profile-aware. They only apply to joints not listed in the B601
  map (i.e. when the monitor runs on a different arm), so the inconsistency
  has no operational impact for B601.

### Known edge cases

1. **Operator forgets to switch profile after changing payload.** If the
   gripper carries 1.5 kg but the monitor runs in `light`, joints 2/3/4 will
   flag elevated and may emit WARN in non-holding contexts. The WARN is
   *informative* (the joint really is operating above the light envelope).
   The fix is to restart the monitor with the matching profile. In rqt,
   check **System → monitor_config** (Message column) or the
   `payload_profile` KeyValue on a per-joint row.
2. **URDF home pose (q=0) is a stretched pose.** Joint3 sees ~9.3 Nm of
   gravity with a 0.5 kg payload, which equals the `light` max threshold.
   If the arm boots in this pose with the monitor at `light`, a transient
   WARN can fire on joint3. Operational rest poses fold the elbow, so this
   is mostly a corner case at first power-up.
3. **Folded-back pose (q2 ≈ -π, q3 ≈ -π) WARNs in every profile.** Even
   without payload, joint2 sees ~16 Nm of gravity there — above any
   reasonable max. This is correct: the pose exceeds the motor's healthy
   envelope and the operator should avoid it.

## What these thresholds are not

- **Not a firmware safety stop.** Datasheet VMAX is a drive parameter; the
  monitor cannot verify that the firmware actually clamps to it at runtime.
  Treat the WARN as a diagnostic signal, not as a guarantee.
- **Not a mode-aware policy.** The same threshold is applied whether the arm
  is in POS_VEL hold, trajectory execution, gravity compensation, or MIT
  streaming. The driver's POS_VEL `vlim` in `src/rebotarm_bringup/config/arm.yaml`
  (5.0 rad/s for joints 1–3, 3.0 rad/s for joints 4–6) is tighter than the
  envelope values; the monitor does not consume it yet.
- **Not empirically validated for this arm.** The numbers are derived from
  datasheets at 24 V, not from a baseline of healthy motion logged on the
  installed B601.

## Open work

- Mode-aware velocity diagnostics that consume `control_context`
  (`gravity_compensation` / `position_hold` / `normal_or_unknown`) and tighten
  the threshold for POS_VEL using the driver's `vlim`.
- Empirical baseline (normal motion, gravity compensation, full trajectories
  with payload) before treating any of these values as final tuning.
- Empirical idle-torque baseline on the real arm to refine each profile's
  per-joint values above. The URDF computation is a starting calibration,
  not the final number.
- Pose-aware idle threshold (live forward kinematics + simplified dynamics)
  would beat any payload-coarse heuristic but requires a dynamics model in
  the monitor; deferred until profiles prove insufficient.
- Optional `torque_jump` profile-awareness if the global value emits false
  positives under heavy payload during accelerated motion.

## Where to change them

| You want to change | Edit |
|--------------------|------|
| Values inside an existing profile | `src/rebotarm_monitor/rebotarm_monitor/parameters.py` (`_B601_PROFILES`) |
| Add a new profile | `_B601_PROFILES` + a test in `test_per_joint_limits.py` |
| The global fallback for unknown joints | `src/rebotarm_monitor/config/monitor.yaml` |
| How a joint compares its value to a limit | `trackers/per_joint.py` + `domain/per_joint_limits.py` |
| Default profile at launch | `src/rebotarm_monitor/launch/monitor.launch.py` (`DeclareLaunchArgument`) |

Tests covering the per-joint resolution live in
`src/rebotarm_monitor/test/test_factories.py` and
`src/rebotarm_monitor/test/test_per_joint_limits.py`.
