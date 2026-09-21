# Exoskeleton raw-data recorder

This package records source sensor data and controller context without starting
OpenVINS, elevation mapping, plane segmentation, or the fused state estimator.

Two profiles are provided:

- `raw_topics.yaml`: raw camera/IMU, encoders, hardware diagnostics, and TF.
- `impedance_debug_topics.yaml`: the same evidence plus required CPG references
  and impedance-controller diagnostics.

The separate `robot_bringup/record_state_estimation.launch.py` recorder is kept
for later full-system demonstrations that also retain online algorithm outputs.

## Start and record

```bash
source /opt/ros/humble/setup.bash
source /home/lsy/vins_ws/install/setup.bash
ros2 launch exo_data_recorder recording_camera.launch.py
```

The default recording camera profile publishes the point cloud and keeps the
existing decimation, spatial, and temporal filters. `raw_camera.launch.py` is
retained as an explicit unfiltered alternative for filter-development tests.

Start the exoskeleton hardware/controller in another terminal using the normal
bringup or tuning launch. Do not start mapping or state-estimation nodes merely
to collect a raw dataset.

Preflight the default raw profile:

```bash
ros2 run exo_data_recorder check_topics \
  --config /home/lsy/vins_ws/install/exo_data_recorder/share/exo_data_recorder/config/raw_topics.yaml
```

Record into a new directory:

```bash
ros2 launch exo_data_recorder record.launch.py \
  output:=/home/lsy/bags/raw_YYYYMMDD_HHMMSS
```

For impedance tuning, pass the debug profile to both preflight and recording:

```bash
ros2 launch exo_data_recorder record.launch.py \
  topics_config:=/home/lsy/vins_ws/install/exo_data_recorder/share/exo_data_recorder/config/impedance_debug_topics.yaml \
  output:=/home/lsy/bags/impedance_YYYYMMDD_HHMMSS
```

Stop with `Ctrl-C`, wait for the recorder to finish writing metadata, then run:

```bash
ros2 run exo_data_recorder verify_bag /home/lsy/bags/raw_YYYYMMDD_HHMMSS \
  --config /home/lsy/vins_ws/install/exo_data_recorder/share/exo_data_recorder/config/raw_topics.yaml
```
