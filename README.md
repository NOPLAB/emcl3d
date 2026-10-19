# emcl3d

Copyright (C) 2026 nop <noplab90@gmail.com>

Licensed under the GNU Lesser General Public License version 3 only
(`LGPL-3.0-only`). See `LICENSE` for the LGPL terms and `COPYING` for
the incorporated GPL version 3 terms.

## Reference implementation and attribution

The Monte Carlo localization and expansion-resetting design was developed with
reference to [CIT-Autonomous-Robot-Lab/emcl2_ros2](https://github.com/CIT-Autonomous-Robot-Lab/emcl2_ros2),
at commit `3eacea2dc70d9d29eb0d501d01eb5df001814776`.
The reference source credits:

> Copyright (C) 2022 Ryuichi Ueda <ryuichiueda@gmail.com>

The inspected upstream `COPYING` contains the GNU LGPL version 3 text, while
`package.xml` declares `LGPL` without a version. All inspected C++ headers
and implementations carry `LGPL-3.0-or-later` SPDX identifiers except
`src/LikelihoodFieldMap.cpp`, which carries `BSD-3-Clause`.
`include/emcl2/emcl2_node.h` also notes that some lines originate from AMCL
under the LGPL. These are upstream file-specific notices; they are not
replaced by emcl3d's `LGPL-3.0-only` declaration or its author attribution.

## Overview

Standalone ROS 2 Humble 6DoF Monte Carlo localization against a PCD point map.
The package owns `map -> odom`; an external odometry source owns
`odom -> base_link`. It does not depend on GLIM or generate odometry.

The filter uses SE(3) particles, nearest-map-point likelihoods, effective-sample-size
resampling, and bounded expansion resetting after repeated low-alpha observations
with map support. Translation and rotation are both estimated. Alpha is the
fraction of particles with enough inlier points, **not a convergence probability**.

## Build and start

Use a ROS 2 Humble workspace with Eigen, PCL, and the ROS dependencies in
`package.xml` installed:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select emcl3d --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch emcl3d emcl3d.launch.xml \
  map_path:=/absolute/path/map.pcd cloud_topic:=/deskewed_points
```

For recorded data, add `use_sim_time:=true` and play the bag with `--clock`.
`params_file:=/absolute/path/settings.yaml` replaces the default configuration.
The launch file is XML and starts executable `emcl3d_node`.

The map must contain finite XYZ points expressed in `map_frame`; its coordinates
and the cloud coordinates must use metres. A missing, unreadable, or empty map
fails startup. The map is voxelized once and remains static.

## Input and TF contract

| Input | Type | Contract |
| --- | --- | --- |
| `cloud_topic` (default `/points`) | `sensor_msgs/PointCloud2` | Already deskewed into the sensor frame at `header.stamp`; finite scalar FLOAT32/FLOAT64 XYZ fields. |
| `initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | Full 6D pose and symmetric positive-semidefinite 6x6 covariance, including translation/rotation cross-correlation. |
| TF | `odom_frame -> base_frame` | Available at each accepted cloud's exact stamp. |
| TF | `base_frame -> cloud.header.frame_id` | Available at that same exact stamp; identity if the cloud is already in `base_frame`. |

This node does **not** deskew raw spinning scans or interpret per-point time
fields. The upstream deskewer must transform every point into the common frame
and instant named by the message header. Both node TF lookups use that instant,
never TF's "latest" sentinel. Dynamic sensor extrinsics are therefore supported.

The default executor and callback group serialize filter callbacks. DDS retains
one cloud and the application retains one latest-cloud slot; a newer pending
cloud replaces the older one. A wall timer retries missing TF without blocking
callbacks, for at most `tf_wait_timeout` seconds measured with a steady clock.
Clouds exceeding `max_cloud_age`, beyond `future_tolerance`, with zero stamps,
or at/before the last accepted stamp are dropped. Zero ROS time waits for the
first valid clock and cloud; it never becomes a "latest TF" request.

Motion is `inverse(previous_odom_from_base) * current_odom_from_base`. The
baseline advances only after a successful filter update, so dropped clouds and
TF retries neither lose nor double-apply motion. The first observation uses
identity motion unless an initial pose established a stamped odometry baseline.
On ROS clock rollback, the node clears pending observations and the odometry
baseline, retaining the last map belief; the next observation starts a new
motion epoch. This is not automatic global relocalization after a physical
teleport. Send `initialpose` when the physical pose changes discontinuously.

Cloud validation checks storage dimensions, row padding, field offsets/types,
endianness, finite coordinates, and `max_input_points`. Uniform index sampling
limits work to at most `max_eval_points`, then voxel deduplication runs in the
base frame. Points outside sensor-frame Euclidean distance `[min_range, max_range]`
are discarded before applying the sensor extrinsic. At least `min_valid_points`
must survive. A malformed or insufficient
cloud is rejected without changing the motion baseline.

## Initialization and published results

`initial_pose` is `[x, y, z, roll, pitch, yaw]`, with rotation `Rz(yaw) Ry(pitch)
Rx(roll)`. `initial_variance` specifies the corresponding diagonal variances in
fixed map axes. This prior is used until an `initialpose` message is accepted.

For `initialpose`, nonzero stamps refer to the pose's exact instant; stamp zero
means current ROS time, not latest TF. The message frame is transformed to
`map_frame` at that instant, and the complete covariance is rotated by
`diag(R, R)`, preserving its cross terms. Translation and rotation covariance
components use fixed reference-frame axes, consistent with ROS pose covariance.
The node requires stamped odometry at initialization, rejects clouds preceding
that stamp, and integrates motion from the initial pose's instant to the next
cloud. Missing TF or invalid pose/covariance rejects the entire initialization.

| Output | Type | Frame and stamp |
| --- | --- | --- |
| `mcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | `map_frame`, accepted cloud stamp; full 6x6 covariance. |
| `particlecloud` | `geometry_msgs/PoseArray` | `map_frame`, same stamp; all particles. |
| `alpha` | `std_msgs/Float64` | Same update, but this message type has no header. |
| `diagnostics` | `diagnostic_msgs/DiagnosticArray` | Current ROS time; includes last accepted observation stamp, alpha, ESS, support ratio, reset flag, and counters. |
| TF `map_frame -> odom_frame` | `geometry_msgs/TransformStamped` | Accepted cloud stamp; `map_from_base * inverse(odom_from_base)`. |

Set `publish_tf: false` when another process owns `map -> odom`. Output pose and
TF are observation-stamped, not extrapolated or future-dated, and are published
only on accepted observations. A predicted or weakly supported estimate may
still be published; consumers must inspect diagnostics.

An OK diagnostic requires current-point inlier support, alpha above its
threshold, a base pose within the map bounding box plus `inlier_distance`,
bounded position and rotation covariance, and no expansion reset on that update.
Covariance limits `max_position_stddev` and `max_rotation_stddev` bound the
largest principal-axis standard deviation of each 3x3 covariance block.
Out-of-map poses, large covariance, missing support, and expansion resets are
warnings, regardless of alpha. Even OK is not a convergence guarantee.
An observation older than `max_cloud_age` makes diagnostics stale. A filter
exception disables updates until a new valid `initialpose` or clock epoch
reinitializes it, preventing a partially applied motion from being retried.
`processing_duration_ms` reports steady-clock processing time for the last
accepted observation (TF lookup, preprocessing, filter update, and support checks,
excluding publishing and time awaiting TF). `expansion_reset_count` counts all
expansion resets since node startup.

## Parameters

`config/emcl3d.yaml` lists every application parameter with usable defaults except
the required map path. Parameters are read at startup; restart to change them.
Lengths are metres, angles radians, noise vectors standard deviations, and
covariance values squared units.

- `num_particles` must be 2 through 1,000,000. `random_seed` is a nonnegative
  signed 32-bit integer for reproducible runs.
- `z_hit` and `z_random` must each be in `(0, 1]`; neither may be zero.
  `min_inlier_ratio` must be in `(0, 1]`. `resample_ess_ratio` and
  `alpha_threshold` must be in `[0, 1]`.
- `sigma_hit`, `likelihood_temperature`, `inlier_distance`, both voxel sizes,
  age/TF wait limits, and diagnostic standard-deviation limits must be finite
  and positive. `future_tolerance` may be zero.
- `min_range` must be finite and nonnegative; `max_range` must be finite and
  strictly greater than `min_range`.
- The four noise/expansion vectors must have three finite nonnegative values.
  Initialization arrays must have six finite values, and variances must be
  nonnegative.
- `min_valid_points` and `reset_confirmation_updates` must be positive.
  `min_valid_points <= max_eval_points <= max_input_points`; the maximum
  evaluation limit is 1,000,000, and the maximum input limit is 100,000,000.
  Frame names must be distinct, nonempty, and have no leading slash.

Run package tests after building:

```bash
colcon test --packages-select emcl3d --event-handlers console_direct+
colcon test-result --verbose
```
