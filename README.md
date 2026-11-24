# emcl3d

[![Humble CI](https://img.shields.io/github/check-runs/NOPLAB/emcl3d/main?nameFilter=ROS%202%20humble%20build%20and%20test%20%28ubuntu-24.04%29&label=Humble)](https://github.com/NOPLAB/emcl3d/actions/workflows/ci.yaml?query=branch%3Amain)
[![Jazzy CI](https://img.shields.io/github/check-runs/NOPLAB/emcl3d/main?nameFilter=ROS%202%20jazzy%20build%20and%20test%20%28ubuntu-24.04%29&label=Jazzy)](https://github.com/NOPLAB/emcl3d/actions/workflows/ci.yaml?query=branch%3Amain)
[![Lyrical CI](https://img.shields.io/github/check-runs/NOPLAB/emcl3d/main?nameFilter=ROS%202%20lyrical%20build%20and%20test%20%28ubuntu-24.04%29&label=Lyrical)](https://github.com/NOPLAB/emcl3d/actions/workflows/ci.yaml?query=branch%3Amain)

## Overview

ROS 2 localization with six degrees of freedom against a static PCD point map.
The filter estimates translation and rotation using SE(3) particles,
nearest-map-point likelihoods, effective-sample-size resampling, and bounded
expansion resetting.

The package owns `map -> odom`; an external source owns `odom -> base_link`.
It does not generate odometry or depend on GLIM. CPU execution is the default,
with an optional Vulkan backend for map-distance queries.

## Requirements

CI covers these ROS 2 distributions on x86_64 and ARM64:

| ROS 2 | Container OS |
| --- | --- |
| Humble | Ubuntu 22.04 |
| Jazzy | Ubuntu 24.04 |
| Lyrical | Ubuntu 26.04 |

Install ROS 2, `colcon`, and `rosdep`, and initialize rosdep before building.
[package.xml](package.xml) declares the CPU dependencies, including Eigen, PCL,
OpenMP, and the ROS interfaces.

Runtime inputs are a PCD map, external odometry TF, and already deskewed point
clouds. The map must contain finite XYZ points in `map_frame`; map and cloud
coordinates use metres. A missing, unreadable, or empty map fails startup.
The map is voxelized once and remains static.

## Build

The examples use Humble. Replace `humble` with `jazzy` or `lyrical` to use another
supported distribution.

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/NOPLAB/emcl3d.git
cd ~/ros2_ws
rosdep update --rosdistro "$ROS_DISTRO"
rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -y
colcon build --packages-select emcl3d \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
```

### Optional Vulkan build

The default `EMCL3D_ENABLE_VULKAN=OFF` build does not discover Vulkan, compile
shaders, or link the Vulkan loader. To include the backend, install its additional
development dependencies and rebuild:

```bash
sudo apt-get install libvulkan-dev glslang-tools
colcon build --packages-select emcl3d --cmake-args \
  -DCMAKE_BUILD_TYPE=Release -DEMCL3D_ENABLE_VULKAN=ON
```

See [Compute backend](#compute-backend) for runtime selection and hardware
requirements.

## Run

From the workspace root:

```bash
source install/setup.bash
ros2 launch emcl3d emcl3d.launch.xml \
  map_path:=/absolute/path/map.pcd cloud_topic:=/deskewed_points
```

The XML launch file starts `emcl3d_node` and loads
[config/emcl3d.yaml](config/emcl3d.yaml). Override it with
`params_file:=/absolute/path/settings.yaml`. Parameters are read at startup;
restart the node after changing them.

For recorded data, add `use_sim_time:=true` and play the bag with `--clock`.
Supply a prior with `initial_pose` and `initial_variance`, send an `initialpose`
message, or enable [Global localization](#global-localization).

## ROS interfaces

| Input | Type | Contract |
| --- | --- | --- |
| `cloud_topic` (default `/points`) | `sensor_msgs/msg/PointCloud2` | Already deskewed into the sensor frame at `header.stamp`; finite scalar FLOAT32/FLOAT64 XYZ fields. |
| `initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | Full 6D pose and symmetric positive-semidefinite 6x6 covariance, including translation/rotation cross-correlation. |
| `map` (when global localization requires free space) | `nav_msgs/msg/OccupancyGrid` | Separate free-space grid in `map_frame`; reliable, transient-local QoS. |
| TF | `odom_frame -> base_frame` | Available at each accepted cloud's exact stamp. |
| TF | `base_frame -> cloud.header.frame_id` | Available at that same exact stamp; identity if the cloud is already in `base_frame`. |

| Output | Type | Frame and stamp |
| --- | --- | --- |
| `mcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | `map_frame`, accepted cloud or manual initialization stamp; full 6x6 covariance. |
| `particlecloud` | `geometry_msgs/msg/PoseArray` | `map_frame`, same stamp; all particles. |
| `alpha` | `std_msgs/msg/Float64` | Same update, but this message type has no header. |
| `diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | Current ROS time; observation stamp, alpha, ESS, support ratio, reset flag, and counters. |
| `map_frame -> odom_frame` | TF | Accepted cloud or initialization stamp; `map_from_base * inverse(odom_from_base)`. |

Topics above are relative unless a leading slash is shown. The private action
`~/global_localization` normally resolves to `/emcl3d/global_localization`
and uses `emcl3d/action/GlobalLocalization`.

## Configuration

### Parameters

[config/emcl3d.yaml](config/emcl3d.yaml) lists every application parameter with
usable defaults except the required `map_path`. Lengths are metres, angles are
radians, noise vectors are standard deviations, and covariance values are
squared units.

| Setting | Default | Purpose |
| --- | --- | --- |
| `map_frame`, `odom_frame`, `base_frame` | `map`, `odom`, `base_link` | TF frame names |
| `cloud_topic` | `/points` | Deskewed input cloud |
| `publish_tf` | `true` | Publish `map -> odom` |
| `initial_pose` | Six zeros | `[x, y, z, roll, pitch, yaw]` prior |
| `initial_variance` | `[1, 1, 1, 0.1, 0.1, 0.5]` | Diagonal prior covariance |
| `num_particles` | `500` | Tracking particles |
| `num_threads` | `0` | CPU likelihood workers; nonpositive uses all available cores |
| `compute_backend` | `cpu` | `cpu` or the optional `vulkan` backend |
| `max_eval_points` | `2000` | Tracking point budget |
| `min_valid_points` | `20` | Minimum usable points per observation |
| `min_noise_scale` | `0.0` | Lower bound of the motion-noise scale; positive keeps particles diffusing while stationary |
| `global_localization.run_on_startup` | `false` | Run one global search before tracking |

A positive `num_threads` requests that many OpenMP workers; `1` runs serially.
Runtime limits may constrain the team size. Query preparation and likelihoods
run in parallel; random sampling, normalization, estimation, and resampling keep
their serial order. Identical inputs and seeds therefore give identical CPU
results across thread counts. Library callers must not reload the map during
queries.

Each update perturbs particles by `translation_noise` and `rotation_noise`
multiplied by `max(min_noise_scale, sqrt(|translation| + |rotation angle|))` of
the odometry step. With the default zero, a stationary robot adds no noise, so
resampling narrows the particle set and the estimate stops following the
observation. A small positive value such as `0.05` keeps correcting it while
stationary.

Parameter validation enforces:

- `num_particles` is 2 through 1,000,000. `random_seed` is a nonnegative signed
  32-bit integer; `num_threads` is a signed 32-bit integer.
- `z_hit`, `z_random`, and `min_inlier_ratio` are in `(0, 1]`.
  `resample_ess_ratio` and `alpha_threshold` are in `[0, 1]`.
- `sigma_hit`, `likelihood_temperature`, `inlier_distance`, voxel sizes, age/TF
  wait limits, and diagnostic standard-deviation limits are finite and positive.
  `future_tolerance` may be zero.
- `min_range` is finite and nonnegative; `max_range` is finite and greater.
- Noise and expansion vectors have three finite nonnegative values.
  `min_noise_scale` is finite and nonnegative.
  Initialization arrays have six finite values; variances are nonnegative.
- `min_valid_points` and `reset_confirmation_updates` are positive, and
  `min_valid_points <= max_eval_points <= max_input_points`. Evaluation is capped
  at 1,000,000 points and input at 100,000,000 points.
- Frame names are distinct, nonempty, and have no leading slash.

### Compute backend

Set `compute_backend: cpu` or `compute_backend: vulkan` in the YAML passed through
`params_file`, then restart. The default is `cpu`, even in Vulkan-enabled builds.
For direct execution:

```bash
ros2 run emcl3d emcl3d_node --ros-args \
  -p map_path:=/absolute/path/map.pcd -p compute_backend:=vulkan -p num_threads:=2
```

An unknown backend, an uncompiled Vulkan backend, an unavailable hardware GPU,
or exceeded device limits fails explicitly. There is no silent CPU fallback.
Software Vulkan devices are not accepted as GPU execution. A Vulkan-enabled
binary needs the Vulkan loader even in CPU mode; build with Vulkan disabled
when that dependency must be absent.

The backend uses Vulkan-Hpp and embedded SPIR-V targeting Vulkan 1.1. The device
must support four storage buffers and a 64-invocation compute workgroup.
FP64, subgroups, ray tracing, and Vulkan 1.3 are not required.

| Hardware family | Driver requirement |
| --- | --- |
| Raspberry Pi 4/5 | Mesa V3DV with the V3D kernel driver |
| Rockchip/Mali | A supported PanVK or vendor Vulkan implementation; check the exact Mali model |
| Intel, NVIDIA, AMD | A Vulkan compute driver meeting the checked capabilities |

Support depends on the GPU and installed driver. Pi generations and older Mali
devices without hardware Vulkan use CPU execution instead. Consult the
[Mesa V3D](https://docs.mesa3d.org/drivers/v3d.html) and
[Panfrost/PanVK](https://docs.mesa3d.org/drivers/panfrost.html) support tables.
Portability does not imply equal performance or memory capacity.

Map geometry stays on the GPU, and transformed queries are submitted in batches.
Scalar diagnostic queries, square roots, double-precision likelihoods,
normalization, and random sampling remain on CPU. GPU FP32 results use numerical
tolerances, not a promise of bitwise equality across devices.

## How it works

### Observation timing and preprocessing

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

### Initialization and published results

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

Set `publish_tf: false` when another process owns `map -> odom`. Output pose and
TF use the observation or manual initialization stamp, not extrapolated or
future-dated stamps. A predicted or weakly supported estimate may
still be published; consumers must inspect diagnostics.

### Diagnostics

Alpha is the fraction of particles with enough inlier points, **not a convergence
probability**. An OK diagnostic requires current-point inlier support, alpha above its
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

### Global localization

Global localization searches XY and yaw against the PCD map, then hands a
successful estimate to the normal 6DoF tracker. It isn't a 6D global search.
`global_localization.search_mode` accepts only `xy_yaw`; set
`fixed_z_roll_pitch: [z, roll, pitch]` to the known height and attitude.
All-zero `search_bounds: [0.0, 0.0, 0.0, 0.0]` uses the PCD XY bounds; otherwise supply
`[min_x, max_x, min_y, max_y]` with finite increasing intervals.
`yaw_range` is an increasing `[min, max]` interval in radians spanning at most
`2*pi`, defaulting to `[-pi, pi]`.

#### Action requests

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

#### Startup behavior

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

#### Free-space constraints

With the default `free_cells_only: true`, publish a separate
`nav_msgs/msg/OccupancyGrid` on the relative topic `map`, configurable through
`free_space_topic`. The subscription uses reliable, transient-local QoS and
requires the grid's frame to equal `map_frame`; no grid-frame TF is applied.
Free space isn't inferred from PCD points. Cells must be known and below
`occupancy_threshold`; unknown, occupied, and out-of-grid cells are excluded
throughout the `footprint_radius`. Setting `free_cells_only: false` removes this
placement constraint, not the PCD scoring requirement.

#### Search budget and acceptance

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

## Testing

CI builds and runs the CPU tests on Humble, Jazzy, and Lyrical, on both x86_64
and ARM64. Each combination also has a separate Vulkan compile job that builds
the benchmark and validates the shader with `spirv-val`.

The badges at the top track each distribution's x86_64 build-and-test job on
`main`; follow them to view the complete matrix. CI uploads CPU test results and
checks the installed launch file.

From the workspace root after building:

```bash
source install/setup.bash
colcon test --packages-select emcl3d \
  --event-handlers console_direct+ --return-code-on-test-failure
colcon test-result --verbose
```

Tests cover the particle filter, global localization, and ROS node behavior.
Vulkan-enabled tests include real GPU dispatch and readback and need a supported
hardware GPU. CI's Vulkan compile jobs do not execute those tests or claim GPU
runtime coverage.

## License and attribution

Copyright (C) 2026 nop <noplab90@gmail.com>.

Licensed under the GNU Lesser General Public License version 3 only
(`LGPL-3.0-only`). See [LICENSE](LICENSE) for the LGPL terms and
[COPYING](COPYING) for the incorporated GPL version 3 terms.

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
