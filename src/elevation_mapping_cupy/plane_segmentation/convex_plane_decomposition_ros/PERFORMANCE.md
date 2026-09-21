# Plane segmentation offline performance measurement

The node consumes `GridMap`, not raw point clouds. Run these measurements with a
synthetic publisher or a read-only bag, and label the result accordingly. Synthetic
results demonstrate repeatability and software throughput only; they do not establish
real-terrain accuracy.

Start the node:

```bash
ros2 launch convex_plane_decomposition_ros plane_segmentation.launch.py
```

Record the source and output rates and serialized bandwidth for at least five minutes:

```bash
timeout 300 ros2 topic hz /elevation_mapping_node/elevation_map_raw
timeout 300 ros2 topic hz /plane_segmentation_node/planar_terrain
timeout 300 ros2 topic bw /plane_segmentation_node/planar_terrain
ros2 topic echo --once /plane_segmentation_node/diagnostics
```

The diagnostics report input/processing rates, latest ROI rows and columns, overwritten
pending-map count, rejected-map count, output bytes, and rolling p50/p95/p99/max processing
time. An overwritten pending map is intentional latest-map behavior, not an accumulated queue.

For process CPU, resident memory, host memory, and long-run RSS trend, find the node PID and
sample it independently from the ROS executor:

```bash
pgrep -n plane_segmentat
ps -p PID -o pid,etime,%cpu,%mem,rss,vsz,cmd
vmstat 1 300
pidstat -r -u -p PID 1 300
```

If `pidstat` is unavailable, install the Ubuntu `sysstat` package or sample the `ps` command
periodically. Compare the first and final RSS and retain the full time series. This node has no
CUDA/CuPy calls; GPU and VRAM are therefore `not used`, which can be confirmed while running
with `nvidia-smi pmon -s um -d 1` when NVIDIA tooling is present.

## 2026-09-08 synthetic baseline

This baseline used the installed Release build on ROS 2 Humble, Linux 6.8.0-138-generic,
an Intel Core i5-13600KF (20 logical CPUs), and 32 GB host memory. The installed NVIDIA
GeForce RTX 4070 Ti SUPER was not used by the node. Input was the explicitly synthetic
publisher in this package: a 75 x 75 cell, 3 m x 3 m map at 0.04 m resolution and 15 Hz.
The configured target-frame ROI produced a 33 x 39 cell processing map. For this
baseline the target frame was the gravity-aligned `global` frame.

The measured input and output rates were 15.000 Hz and 5.000 Hz. After separating
RViz-only layers from the machine-readable result, a short repeat measurement gave a
serialized `PlanarTerrain` size of 42,192 bytes per message (42.19 KB and approximately
216 KB/s). The debug segmentation map, including the observed-cell visualization mask,
was 57.75 KB per message (approximately 303 KB/s as reported by `ros2 topic bw`). Boundaries and insets were each
0.58 KB per message, and the 1 Hz diagnostics were approximately 583 bytes. Over the
earlier five-minute, 300-sample run, the node processed 1,535 maps, rejected none, and overwrote
3,068 pending maps as intended. Processing time was 0.744 ms p50, 0.835 ms p95,
1.082 ms p99, and 1.128 ms maximum. Sampled process CPU averaged 0.07%, and RSS stayed
at 28,032 KiB from first to last sample with a 0 KiB observed range. Host available memory
changed from 27,575,914,496 to 27,449,819,136 bytes while other host workloads remained
uncontrolled, so that host-wide change is background context rather than node attribution.
The five-minute CPU and memory run preceded the RViz-only masking loop, observed-support
gate, and the current 2 m square ROI, so it must be repeated on the current revision before final performance sign-off. These synthetic results do not
validate real-terrain accuracy or long-duration deployment.
