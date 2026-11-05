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

Use a ROS 2 Humble workspace with Eigen, PCL, OpenMP, and the ROS dependencies in
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

### Optional Vulkan execution

The default `EMCL3D_ENABLE_VULKAN=OFF` build does not discover Vulkan, compile shaders,
or link the Vulkan loader. CPU dependencies remain those in `package.xml`.
To include the optional backend, install its additional development dependencies:

```bash
sudo apt-get install libvulkan-dev glslang-tools
colcon build --packages-select emcl3d --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DEMCL3D_ENABLE_VULKAN=ON
```

Select `compute_backend: cpu` or `compute_backend: vulkan` in the YAML passed through
`params_file`, then restart. The default is `cpu`, including in Vulkan-enabled builds.
For direct execution:

```bash
ros2 run emcl3d emcl3d_node --ros-args \
  -p map_path:=/absolute/path/map.pcd -p compute_backend:=vulkan -p num_threads:=2
```

An unknown backend, an uncompiled Vulkan backend, an unavailable hardware GPU,
or an exceeded device buffer/dispatch limit fails explicitly. There is no silent
CPU fallback. CPU software Vulkan devices are not accepted as GPU execution.
A Vulkan-enabled binary still needs the Vulkan loader even when selecting CPU;
use a CPU-only build when that dependency must be absent.

The backend uses official Vulkan-Hpp bindings and embedded SPIR-V targeting Vulkan
1.1. It requires four storage buffers and a 64-invocation compute workgroup, checked
against the selected device. It does not require FP64, subgroups, ray tracing,
or Vulkan 1.3 features. Map geometry remains static on the GPU; transformed queries
are submitted in batches, not one dispatch per point. Scalar diagnostic queries
remain on CPU. Square roots, double-precision likelihoods, normalization and random
sampling retain their CPU implementation and order. GPU FP32 arithmetic is checked
with numerical tolerances, not a cross-device bitwise-equality promise.

GPU support depends on the GPU **and its installed driver**, not the CPU brand:

| Hardware family | Driver requirement |
| --- | --- |
| Raspberry Pi 4/5 | Mesa V3DV with the V3D kernel driver. |
| Rockchip/Mali | A supported PanVK or vendor Vulkan implementation; check the exact Mali model. |
| Intel, NVIDIA, AMD | An installed Vulkan compute driver meeting the checked capabilities. |

Pi generations and older Mali devices without hardware Vulkan are not supported
by this backend; CPU execution remains available. Consult the
[Mesa V3D](https://docs.mesa3d.org/drivers/v3d.html) and
[Panfrost/PanVK](https://docs.mesa3d.org/drivers/panfrost.html) support tables.
Library portability does not imply the same performance or memory capacity on each GPU.

Vulkan-enabled package tests include real GPU dispatch/readback and require a hardware
GPU. CI runs CPU tests on x86_64/ARM64 and separately compiles the GPU backend and
validates its shader on both architectures; those compile jobs do not claim GPU execution.

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
fixed map axes. This prior is used until an `initialpose` message is accepted,
unless `global_localization.run_on_startup` suppresses it.

For `initialpose`, nonzero stamps refer to the pose's exact instant; stamp zero
means current ROS time, not latest TF. The message frame is transformed to
`map_frame` at that instant, and the complete covariance is rotated by
`diag(R, R)`, preserving its cross terms. Translation and rotation covariance
components use fixed reference-frame axes, consistent with ROS pose covariance.
The node requires stamped odometry at initialization, rejects clouds preceding
that stamp, and integrates motion from the initial pose's instant to the next
cloud. Missing TF or invalid pose/covariance rejects the entire initialization.
An accepted manual pose is published immediately, including `map -> odom`,
without requiring a cloud or map support. This also applies outside map bounds;
the diagnostic remains a warning until an observation confirms support.

| Output | Type | Frame and stamp |
| --- | --- | --- |
| `mcl_pose` | `geometry_msgs/PoseWithCovarianceStamped` | `map_frame`, accepted cloud or manual initialization stamp; full 6x6 covariance. |
| `particlecloud` | `geometry_msgs/PoseArray` | `map_frame`, same stamp; all particles. |
| `alpha` | `std_msgs/Float64` | Same update, but this message type has no header. |
| `diagnostics` | `diagnostic_msgs/DiagnosticArray` | Current ROS time; includes last accepted observation stamp, alpha, ESS, support ratio, reset flag, and counters. |
| TF `map_frame -> odom_frame` | `geometry_msgs/TransformStamped` | Accepted cloud or manual initialization stamp; `map_from_base * inverse(odom_from_base)`. |

Set `publish_tf: false` when another process owns `map -> odom`. Output pose and
TF use the observation or manual initialization stamp, not extrapolated or
future-dated stamps. A predicted or weakly supported estimate may
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

## Global localization

Global localization searches XY and yaw against the PCD map, then hands a
successful estimate to the normal 6DoF tracker. It isn't a 6D global search.
`global_localization.search_mode` accepts only `xy_yaw`; set
`fixed_z_roll_pitch: [z, roll, pitch]` to the known height and attitude.
All-zero `search_bounds: [0.0, 0.0, 0.0, 0.0]` uses the PCD XY bounds; otherwise supply
`[min_x, max_x, min_y, max_y]` with finite increasing intervals.
`yaw_range` is an increasing `[min, max]` interval in radians spanning at most
`2*pi`, defaulting to `[-pi, pi]`.

The private action `~/global_localization` resolves to
`/emcl3d/global_localization` with the default node name:

```bash
ros2 action send_goal /emcl3d/global_localization emcl3d/action/GlobalLocalization \
  '{estimate_only: false}' --feedback
ros2 action send_goal /emcl3d/global_localization emcl3d/action/GlobalLocalization \
  '{estimate_only: true}' --feedback
```

A goal may override XY bounds with `use_search_bounds: true` and
`search_bounds: [min_x, max_x, min_y, max_y]`. Invalid bounds and concurrent
goals are rejected, including goals sent while startup localization is waiting.
Cancellation stops the active request without applying its result. An accepted
manual `initialpose` cancels global localization and takes precedence.
Feedback reports the stage, evaluated and total candidates, best inlier ratio,
and elapsed seconds. Results include `success`, `applied`, a status `code` and
`message`, a pose with covariance, and competing hypotheses with scores and
inlier ratios.

`apply_result: true` allows a successful search to initialize tracking.
An estimate-only goal never applies its result; `apply_result: false` also
prevents application, even for a goal with `estimate_only: false`.
The result pose refers to the original search observation's stamp in `map_frame`.
Application carries that estimate forward using the newest valid observation's
odometry snapshot and initializes tracking at that newer stamp. A stale handoff
fails rather than applying an old pose.

`run_on_startup` defaults to `false`. When enabled, the configured startup prior
is suppressed until a global result is successfully applied or a manual
`initialpose` is accepted. Enabling startup localization requires
`apply_result: true`; otherwise startup fails explicitly.
Startup makes one attempt, without automatic retries; failure leaves it unlocalized.
Send another action goal or a manual pose to recover.
`startup_wait_timeout` limits waiting for a recent valid cloud, its stamped TF,
and any required free-space grid, for both startup and action requests.
The separate `timeout` starts when search begins and is checked at batch
boundaries, so an in-flight batch can exceed the limit. Both use steady-clock seconds.
Failed or ambiguous searches preserve an existing tracking belief.

Node failure and interruption codes include `NO_CLOUD`, `NO_FREE_SPACE`,
`TIMEOUT`, `INVALID_INPUT`, `SEARCH_ERROR`, `STALE_OBSERVATION`, `CLOCK_RESET`,
`MANUAL_INITIALPOSE`, and `CANCELED`. Core search reasons use lowercase values
such as `success`, `ambiguous_match`, and `insufficient_match`; preserve case
when interpreting `code`.

With the default `free_cells_only: true`, publish a separate
`nav_msgs/OccupancyGrid` on the relative topic `map`, configurable through
`free_space_topic`. The subscription uses reliable, transient-local QoS and
requires the grid's frame to equal `map_frame`; no grid-frame TF is applied.
Free space isn't inferred from PCD points. Cells must be known and below
`occupancy_threshold`; unknown, occupied, and out-of-grid cells are excluded
throughout the `footprint_radius`. Setting `free_cells_only: false` removes this
placement constraint, not the PCD scoring requirement.

`num_candidates` controls the coarse pose-search budget, `max_eval_points` caps
the observation points scored per pose, and `batch_size` limits poses scored in
one step. `refinement_levels` adds local refinement, while `num_hypotheses`
retains competing solutions. Larger budgets don't guarantee global uniqueness.
Coarse candidates form a reproducible uniform XY/yaw lattice. The budget and
progress count attempted poses, including poses rejected by the free-space mask;
refinement adds to the reported total. Refinement rotates around the scan centroid
to avoid coupling a yaw correction to an unreachable base translation.
Both CPU and Vulkan require `batch_size * max_eval_points <= 16000000`;
there's no automatic downsizing. Vulkan device limits may impose further
restrictions. No measured performance claim is implied by these defaults.

Global scoring reuses top-level `sigma_hit`, `z_hit`, `z_random`,
`inlier_distance`, `num_threads`, `min_valid_points`, `random_seed`, and the
selected `compute_backend`. The global point cap is separate from the tracking
cap. Acceptance uses the nested `min_inlier_ratio`, `max_mean_squared_error`,
`ambiguity_score_margin`, `mode_separation_xy`, `mode_separation_yaw`,
`max_position_stddev`, and `max_yaw_stddev` settings. The global position limit
defaults to 0.5 metres and the yaw limit to 0.2 radians; these aren't the
top-level diagnostic limits. Scores are normalized Gaussian-mixture match
values, not probabilities or a guarantee of a unique global pose.
The global position limit bounds the square root of the XY covariance trace.
Covariance describes the winning local mode and includes a search-resolution
floor; it does not describe uncertainty across rejected competing locations.

## Parameters

`config/emcl3d.yaml` lists every application parameter with usable defaults except
the required map path. Parameters are read at startup; restart to change them.
Lengths are metres, angles radians, noise vectors standard deviations, and
covariance values squared units.

- `num_particles` must be 2 through 1,000,000. `random_seed` is a nonnegative
  signed 32-bit integer for reproducible runs.
- `num_threads` defaults to `0`. Zero or any negative signed 32-bit value uses
  all available CPU cores reported by the OpenMP runtime. A positive value
  requests that many likelihood workers (`1` runs serially). Set it in the YAML
  passed through `params_file` and restart the node. OpenMP runtime limits may
  constrain the actual team size; no global threading settings are changed.
  CPU workers parallelize query preparation and per-particle likelihoods: random sampling,
  normalization, estimation, and resampling retain serial order, giving identical
  results across thread counts for the same inputs and seed. The loaded map is
  immutable during queries; library callers must not reload it concurrently.
- `compute_backend` defaults to `cpu`; `vulkan` requires the optional build and
  a supported hardware GPU. This is a startup selection, not a dynamic parameter.
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
