# Terrain-adaptive CPG safety framework

This package is a software scaffold. It does not contain a calibrated slope/step-to-CPG model and must not be treated as a real-machine terrain controller.

## Runtime graph

`/terrain/modulation` (`TerrainModulation`) and the unified external command `/cpg/gait_mode_cmd` enter `terrain_parameter_scheduler`. The scheduler also consumes `/fused/odometry`, `/fused/contact_state`, and `/cpg/debug`. After validity, freshness, confidence, estimator, contact-window, and gait-provenance checks, it publishes internal `/cpg/gait_mode_target` plus `/cpg/parameter_target` to `legs_cpg`.

`legs_cpg` retains its Hopf oscillator, Fourier gait library, start/stop state machine, and C2 gait switch. It publishes `/cpg/joint_ref`, the legacy `/cpg/switch_debug`, and structured `/cpg/debug`. No phase reset or phase correction is applied. Contact phase is observation-only in `/terrain/scheduler_debug`.

The terrain message carries explicit validity flags for mode, slope, step height, step depth, boundary distance, and confidence. Producers must leave unavailable values invalid; numeric placeholders are not measurements.

## Safe defaults

`config/terrain_scheduler.yaml` disables mode switching and continuous modulation because timeout, confidence, covariance, transition, and mapping parameters have not been calibrated. Its verified gait whitelist contains only `flat_walk`. The existing `slope_up` library entry remains available for code compatibility but is marked `TODO(data_required)` because it is based on flat-walking placeholder data.

Lookup-table, interpolation, regression, and first-transition-step insertion points are separate functions in `scheduler_core.hpp`; each returns unavailable until measured data exists. Production limits are neutral and `legs_cpg` rejects continuous targets while `parameter_target_timeout_s` is non-positive.

## Build and launch

```bash
cd /home/lsy/vins_ws
colcon build --packages-up-to terrain_adaptive_cpg cpg_controller
source install/setup.bash
ros2 launch terrain_adaptive_cpg terrain_adaptive_cpg.launch.py
```

This launch does not start a hardware or impedance controller. With the current production configuration it is observation-only and holds the verified flat gait parameters.

Run the synthetic offline test (also no hardware/impedance controller):

```bash
ros2 launch terrain_adaptive_cpg simulation_only_test.launch.py
```

The synthetic values live only in `config/simulation_only_scheduler.yaml`. Controller, joint-state, and joint-reference outputs are remapped under `/simulation_only/`, so the test cannot attach to the normal real-machine controller topics. The test covers invalid flags, rejected confidence, timeout fallback, parameter clamp/rate/filter behavior, support-window gating, and C2 mode switching. `slope_up` is exercised only under the explicit `simulation_only` exception.

Record the validation topics:

```bash
ros2 launch terrain_adaptive_cpg record_terrain_validation.launch.py output:=terrain_validation_bag
```

## Data still required

- Labelled terrain transitions with message/processing latency, confidence calibration, and false-positive/false-negative outcomes.
- Synchronized slope, step height, step depth, boundary distance, pelvis odometry/covariance, contact state, and the intended switch instant.
- Motion-capture or encoder gait cycles for each real slope/stair geometry and subject/setup, including accepted joint limits.
- Estimator ground-truth error paired with published covariance to select a covariance gate.
- Repeated left/right contact events paired with CPG phase to design—but not yet enable—phase correction.
- Transition trials sufficient to set timeout, change-rate, filter, switching-distance, and safety thresholds.

All corresponding implementation sites are marked `TODO(data_required)`.
