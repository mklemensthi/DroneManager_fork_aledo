# Scenario-Driven Multi-Drone Data Generation — Upright LiDAR Version

This document explains the high-level functionality and configuration options of `scenario_driven_multi_drone_upright_lidar.py`.

The script is designed for Unity + PX4 + DroneManager synthetic LiDAR data generation. It generates **scenario-driven drone trajectories** instead of purely local random walks. The core idea is to first choose a meaningful drone encounter scenario, then generate synchronized anchor configurations for all drones, and finally fly smooth waypoint paths through those anchors.

The goal is to cover the important configuration space for drone detection, tracking, and trajectory prediction: range, bearing, elevation, field-of-view boundary cases, close drone pairs, vertical separation, multi-drone density, and hard association cases.

---

## 1. Main Concept

The script follows this process:

1. **Choose a scenario class**  
   Example: one drone crossing the LiDAR volume, two drones vertically separated, or six drones in a stress-test configuration.

2. **Determine the number of drones**  
   Each scenario has a fixed drone count from 0 to 6.

3. **Generate synchronized anchor configurations**  
   An anchor is a target configuration at one key moment. In a two-drone scenario, one anchor contains one 3D position for `drone1` and one 3D position for `drone2`.

4. **Convert anchors into waypoint paths**  
   Each drone receives a list of waypoints. These paths are smooth enough for simulation and can later be adapted to real MoCap experiments.

5. **Execute the scenario with DroneManager/PX4**  
   The drones arm, take off, fly through the scenario anchors, then land.

6. **Write a manifest file**  
   The manifest stores the selected scenario, random seed, anchor positions, LiDAR-relative metadata, and drone configuration. This makes every generated run traceable and analyzable.

---

## 2. Coordinate System

The script uses a global NED-like coordinate system:

```text
x = north
y = east
z = down
```

A negative `z` value means the object is above the ground. For example:

```python
LIDAR_NED = [0.0, 0.0, -4.0]
```

means the LiDAR is placed 4 m above the ground.

---

## 3. Upright LiDAR Geometry

This version assumes the LiDAR is **upright**, not lying sideways. The LiDAR rotates 360 degrees around the vertical `z/down` axis. The vertical axis passes through the center of the scan volume, like the axis through the hole of a cone-shaped donut.

The LiDAR has:

```python
LIDAR_RANGE_M = 4.0
FOV_DEG = 45.0
```

The 360-degree horizontal scan means that any bearing around the LiDAR is possible. The vertical field of view limits how far above or below the LiDAR plane a point may be.

For a point relative to the LiDAR:

```text
north_offset = point_north - lidar_north
east_offset  = point_east  - lidar_east
down_offset  = point_down  - lidar_down
```

Define horizontal range:

```text
rho = sqrt(north_offset^2 + east_offset^2)
```

A point is valid if:

```text
MIN_HORIZONTAL_DISTANCE_M <= rho <= LIDAR_RANGE_M
abs(down_offset) <= rho * tan(FOV_DEG / 2)
```

For `FOV_DEG = 45`, the half-angle is 22.5 degrees, so:

```text
abs(down_offset) <= rho * tan(22.5°)
                 ≈ 0.414 * rho
```

This means the vertical allowance grows with horizontal distance. Close to the LiDAR axis, the valid vertical space is small. Farther away, the valid vertical space becomes larger.

The `MIN_HORIZONTAL_DISTANCE_M` setting intentionally avoids sampling points too close to the vertical LiDAR axis, because the vertical field-of-view height becomes very small there.

---

## 4. Important User Settings

### Random Seed

```python
RANDOM_SEED = None
```

Use a fixed integer to make scenario generation reproducible:

```python
RANDOM_SEED = 49
```

Use `None` to get a different random scenario realization every run.

---

### Force a Specific Scenario

```python
FORCE_SCENARIO_NAME = None
```

If this is `None`, the script randomly samples a scenario according to the weights in `SCENARIO_LIBRARY`.

To force one specific scenario, set:

```python
FORCE_SCENARIO_NAME = "two_drone_vertical_stack"
```

This is useful when you want to generate many runs of one scenario type.

---

### Manifest Output

```python
RUN_ID_PREFIX = "scenario_run"
MANIFEST_ROOT = Path("C:/Datasets/UnityLidarDrone_scenario_manifests")
PRINT_GENERATED_PATHS = True
WRITE_MANIFEST = True
```

The manifest is a JSON file describing what was generated. It does not contain the LiDAR point cloud itself. It is meant for later analysis of your data coverage.

Recommended: keep `WRITE_MANIFEST = True`.

---

## 5. LiDAR Geometry Settings

```python
LIDAR_NED = [0.0, 0.0, -4.0]
LIDAR_RANGE_M = 4.0
FOV_DEG = 45.0
MIN_HORIZONTAL_DISTANCE_M = 0.75
MIN_INTER_DRONE_DISTANCE_M = 0.75
```

| Setting | Meaning |
|---|---|
| `LIDAR_NED` | Position of the LiDAR in global NED coordinates. |
| `LIDAR_RANGE_M` | Maximum horizontal range from the LiDAR. |
| `FOV_DEG` | Vertical full field of view. Half-angle is `FOV_DEG / 2`. |
| `MIN_HORIZONTAL_DISTANCE_M` | Avoids sampling too close to the vertical LiDAR axis. |
| `MIN_INTER_DRONE_DISTANCE_M` | Minimum allowed 3D distance between drones at each anchor. |

The current values are simulation-scale settings. If you later scale the Unity scene to a more realistic LiDAR range, update `LIDAR_RANGE_M`, the coverage bins, pairwise distance bins, and flight timeouts accordingly.

---

## 6. Coverage Bins

Coverage bins define where drones should appear in the observable space. They are used to avoid relying on uncontrolled random trajectories.

### 6.1 North Offset Bins

The script keeps the name `AXIS_DISTANCE_BINS` for compatibility, but in the upright LiDAR version these bins should be understood as **absolute north/forward offset bins**, not true LiDAR range bins.

```python
AXIS_DISTANCE_BINS = {
    "near": (0.75, 1.50),
    "mid": (1.50, 2.75),
    "far": (2.75, 3.60),
    "edge_range": (3.60, 4.00),
    "any": (0.75, 4.00),
}
```

These bins are mainly useful for constructing paths such as left/right crossings at a fixed `north` coordinate. The true horizontal LiDAR range is:

```text
rho = sqrt(north_offset^2 + east_offset^2)
```

So a point with moderate north offset can still have a large horizontal range if it also has a large east offset.

---

### 6.2 Lateral Fraction Bins

The script keeps the old name `RADIAL_FRACTION_BINS`, but in the upright version this controls the magnitude of the **east/lateral offset** relative to what is possible at a chosen north offset.

For a fixed north offset:

```text
max_east_offset = sqrt(LIDAR_RANGE_M^2 - north_offset^2)
lateral_fraction = abs(east_offset) / max_east_offset
```

```python
RADIAL_FRACTION_BINS = {
    "center": (0.00, 0.30),
    "middle": (0.30, 0.65),
    "edge_fov": (0.65, 1.00),
    "any": (0.00, 1.00),
}
```

Interpretation:

| Bin | Meaning |
|---|---|
| `center` | Near the selected north line. |
| `middle` | Moderate lateral offset. |
| `edge_fov` | Near the outer horizontal range boundary. |
| `any` | No specific lateral preference. |

---

### 6.3 Side Fraction Bins

Side fraction controls left/right placement:

```text
side_fraction = east_offset / max_east_offset_at_north
```

```python
SIDE_FRACTION_BINS = {
    "left": (-1.00, -0.33),
    "center": (-0.33, 0.33),
    "right": (0.33, 1.00),
    "any": (-1.00, 1.00),
}
```

Use these for generating left-to-right crossings, right-to-left crossings, and multi-drone horizontal separation.

---

### 6.4 Vertical Fraction Bins

Vertical fraction controls whether the drone is above, level with, or below the LiDAR plane.

```text
max_vertical_offset = horizontal_range * tan(FOV_DEG / 2)
vertical_fraction = down_offset / max_vertical_offset
```

In NED coordinates, a negative down offset means the drone is above the LiDAR plane, and a positive down offset means it is below it.

```python
VERTICAL_FRACTION_BINS = {
    "above": (-1.00, -0.33),
    "level": (-0.33, 0.33),
    "below": (0.33, 1.00),
    "any": (-1.00, 1.00),
}
```

These bins are especially important for preventing the detector from learning a single-height prior.

---

### 6.5 Pairwise Distance Bins

These bins define relations between two drones.

```python
PAIRWISE_DISTANCE_BINS = {
    "very_close": (0.75, 1.00),
    "close": (1.00, 1.50),
    "medium": (1.50, 2.50),
    "far": (2.50, 4.50),
}
```

They are used for close-pair scenarios, far-apart scenarios, mixed hotspots, and stress tests.

---

## 7. Scenario Classes

Each scenario has:

| Field | Meaning |
|---|---|
| `name` | String used to identify or force the scenario. |
| `num_drones` | Number of drones used in the scene. |
| `num_anchors` | Number of key waypoint configurations generated. |
| `weight` | Probability weight when random scenario selection is used. |
| `description` | Human-readable explanation. |
| `generator` | Internal generator type used by the script. |
| `constraints` | Optional high-level constraints such as close pair, far target, or edge target. |

### Available Scenario Names

```text
empty_scene
single_drone_full_volume_coverage
single_drone_crossing
single_drone_vertical_climb_descent
single_drone_edge_of_fov
two_drone_far_apart
two_drone_close_pair
two_drone_vertical_stack
two_drone_opposite_crossing
three_drone_mixed_hotspot
four_drone_emergency_scene
five_drone_dispersed_hotspot
six_drone_stress_test
```

---

## 8. Scenario Details

### 8.1 `empty_scene`

**Drones:** 0  
**Purpose:** false-positive control.

This scenario records background-only data. No PX4 drones are connected. The script simply waits for `EMPTY_SCENE_DURATION_S`, allowing Unity to record LiDAR data without any drone target.

Use this to test whether the detector hallucinates drones in empty scenes.

---

### 8.2 `single_drone_full_volume_coverage`

**Drones:** 1  
**Purpose:** broad single-object detector coverage.

This scenario makes one drone visit diverse regions of the upright LiDAR volume. It covers near, mid, far, and edge-range positions; center and lateral positions; and above/level/below vertical fractions.

Use this as one of the main scenarios for detector training, because every detector needs to see the drone across the observable volume before multi-drone complexity is added.

---

### 8.3 `single_drone_crossing`

**Drones:** 1  
**Purpose:** wide horizontal crossing through the 360-degree LiDAR volume.

This scenario generates one drone crossing left-to-right or right-to-left. In the upright LiDAR version, this is no longer constrained by a sideways cone axis. Instead, the drone can travel broadly across the horizontal scan volume while remaining inside the vertical FOV.

This scenario is useful for:

```text
single-object tracking
velocity estimation
short-horizon trajectory prediction
left/right crossing behavior
```

It is also one of the most useful scenarios for visually checking whether your geometry is correct, because the lateral travel should now be much wider than in the old sideways-bicone version.

---

### 8.4 `single_drone_vertical_climb_descent`

**Drones:** 1  
**Purpose:** vertical motion and altitude robustness.

This scenario keeps the drone at a roughly similar bearing while changing its height relative to the LiDAR plane. It is designed to test whether the detector and tracker can handle vertical movement rather than assuming that drones always appear at one height.

Use this scenario to improve and evaluate:

```text
z-localization
vertical tracking
climb/descent trajectory prediction
```

---

### 8.5 `single_drone_edge_of_fov`

**Drones:** 1  
**Purpose:** difficult boundary and sparse-observation cases.

This scenario places the drone near the edge of the LiDAR field of view or near the horizontal range boundary. These cases are often harder because the drone may produce fewer points or may only be visible for a shorter period.

Use this for:

```text
edge-of-FOV recall
late acquisition
partial visibility
sparse point returns
```

---

### 8.6 `two_drone_far_apart`

**Drones:** 2  
**Purpose:** multi-object detection without heavy association ambiguity.

This scenario places two drones far apart. It tests whether the detector can find multiple targets in one scene without requiring the tracker to solve close-pair ambiguity.

Use this as the first multi-drone training scenario before moving to close pairs and stress tests.

---

### 8.7 `two_drone_close_pair`

**Drones:** 2  
**Purpose:** close-object separation.

This scenario creates a close pair while still respecting the minimum inter-drone safety distance. It is important because detectors and trackers often fail when two objects are close together.

Use this to test:

```text
non-maximum suppression behavior
box separation
missed detections in close pairs
identity switches
pairwise drone relations
```

---

### 8.8 `two_drone_vertical_stack`

**Drones:** 2  
**Purpose:** similar horizontal position, different altitude.

This scenario places two drones near the same horizontal location but at different vertical offsets. In the upright LiDAR version, this is a critical scenario because the LiDAR scans horizontally while vertical visibility is constrained by the vertical FOV.

Use this to test whether the detector collapses different-altitude targets into one detection or predicts a mean height.

---

### 8.9 `two_drone_opposite_crossing`

**Drones:** 2  
**Purpose:** crossing and association ambiguity.

This scenario makes two drones cross through the LiDAR volume in opposite directions. Near the crossing midpoint, the tracker may need to preserve identity despite small spatial separation or similar trajectories.

Use this to test:

```text
multi-object tracking
identity switches
velocity estimation
trajectory prediction under crossing motion
```

---

### 8.10 `three_drone_mixed_hotspot`

**Drones:** 3  
**Purpose:** small multi-drone hotspot.

This scenario includes one close relationship and one more dispersed target. It is a bridge between simple two-drone cases and dense stress tests.

Use this to evaluate whether the model can handle:

```text
one close pair plus one independent drone
mixed near/far observations
multi-object association
```

---

### 8.11 `four_drone_emergency_scene`

**Drones:** 4  
**Purpose:** realistic emergency-scene style complexity.

This scenario imitates a situation where several drones may be present around an incident, landing zone, or point of interest. It includes mixed positions and can include close or edge targets.

Use this for more difficult tracking and benchmark data, especially if your research topic emphasizes helicopter detect-and-avoid in operationally complex environments.

---

### 8.12 `five_drone_dispersed_hotspot`

**Drones:** 5  
**Purpose:** dense but dispersed multi-drone field.

This scenario spreads five drones across the observable region. It is less focused on close-pair ambiguity and more focused on whether the detector can handle many simultaneous targets.

Use this mainly for stress testing and benchmark evaluation, not as the dominant training scenario.

---

### 8.13 `six_drone_stress_test`

**Drones:** 6  
**Purpose:** maximum supported drone-count stress case.

This scenario combines close, far, and edge targets. It is intentionally difficult and should reveal failure modes in detection, tracking, and association.

Use this for:

```text
stress testing
multi-object robustness
failure-mode discovery
benchmark scenarios
```

Do not let this scenario dominate the detector training set unless you expect real data to contain dense six-drone scenes frequently.

---

## 9. Recommended Scenario Usage

### Detector coverage

Use mostly:

```text
single_drone_full_volume_coverage
single_drone_crossing
single_drone_vertical_climb_descent
single_drone_edge_of_fov
two_drone_far_apart
two_drone_close_pair
two_drone_vertical_stack
```

These scenarios help the detector learn appearance across range, bearing, elevation, FOV boundaries, and pairwise relations.

### Tracking and prediction

Use:

```text
single_drone_crossing
single_drone_vertical_climb_descent
two_drone_opposite_crossing
two_drone_close_pair
three_drone_mixed_hotspot
four_drone_emergency_scene
```

These scenarios create useful temporal behavior and association challenges.

### Stress testing

Use:

```text
four_drone_emergency_scene
five_drone_dispersed_hotspot
six_drone_stress_test
```

These should not dominate the training set, but they are valuable for evaluating failure modes.

---

## 10. PX4 and DroneManager Settings

```python
AUTO_START_PX4 = False
STOP_PX4_ON_EXIT = True
LOAD_EXTERNAL_PLUGIN = True
```

### `AUTO_START_PX4`

If `False`, start the required PX4 SITL instances manually before running the script.

If `True`, the script attempts to launch PX4 instances automatically using the configured WSL path.

### `STOP_PX4_ON_EXIT`

If `True`, PX4 processes started by this script are terminated when the script exits. This only affects processes started by the script itself.

### `LOAD_EXTERNAL_PLUGIN`

If `True`, DroneManager loads the external plugin. Keep this enabled if Unity depends on the external stream.

---

## 11. How to Run the Script

### Step 1: Choose a scenario mode

Randomly sample scenarios:

```python
FORCE_SCENARIO_NAME = None
```

Force one scenario:

```python
FORCE_SCENARIO_NAME = "two_drone_vertical_stack"
```

### Step 2: Choose a random seed

For new randomized runs:

```python
RANDOM_SEED = None
```

For reproducible runs:

```python
RANDOM_SEED = 49
```

### Step 3: Start PX4

If using manual PX4 startup:

```python
AUTO_START_PX4 = False
```

Start as many PX4 instances as the chosen scenario requires. A two-drone scenario needs two PX4 instances; a six-drone scenario needs six.

If using automatic PX4 startup:

```python
AUTO_START_PX4 = True
PX4_AUTOPILOT_WSL_DIR = "/mnt/c/Users/mklemensthi/Documents/TONIC/PX4-Autopilot"
```

Make sure the PX4 path and binary path are correct.

### Step 4: Start Unity recording

Start your Unity scene and ensure the LiDAR exporter is ready. The Python script controls drone motion; Unity records the LiDAR dataset.

### Step 5: Run the script

From the environment where DroneManager is available:

```bash
python scenario_driven_multi_drone_upright_lidar.py
```

The script will:

1. select or load the forced scenario,
2. generate anchors,
3. write a manifest,
4. connect to the drones,
5. take off,
6. fly through the anchors,
7. land and disarm.

---

## 12. Empty Scene Behavior

The scenario `empty_scene` uses zero drones.

In that case, the script does not connect to PX4 drones. It simply waits for:

```python
EMPTY_SCENE_DURATION_S = 20.0
```

This is useful for recording background-only data and testing false positives.

---

## 13. Manifest Files

Every generated run can write a manifest file to:

```text
C:/Datasets/UnityLidarDrone_scenario_manifests
```

The manifest contains:

```text
run_id
random_seed
scenario name
scenario description
number of drones
LiDAR settings
drone configurations
anchor positions
LiDAR-relative metadata for each anchor
```

In the upright LiDAR version, the metadata is especially useful for analyzing:

```text
horizontal range
bearing
north offset
east offset
down offset
vertical fraction
lateral fraction
distance to horizontal range edge
distance to vertical FOV edge
```

Use these manifests later to verify that your generated data actually covers the intended scenario space.

---

## 14. Recommended Dataset Strategy

### Detector pretraining data

Generate broad coverage. Use many scenario classes and many different random seeds. Later, rebalance or downsample frames so one long trajectory does not dominate the dataset.

### Tracking and prediction data

Keep full trajectories. These are useful for testing identity consistency, velocity estimation, missed detections, and future trajectory prediction.

### Benchmark data

Use scenario classes and random seeds that were not used during training. The benchmark should contain unseen trajectories and stress cases.

---

## 15. Common Adjustments

### Generate many runs of one scenario

```python
FORCE_SCENARIO_NAME = "single_drone_crossing"
RANDOM_SEED = None
```

Then run the script multiple times.

### Reproduce one exact run

```python
RANDOM_SEED = 49
FORCE_SCENARIO_NAME = "two_drone_close_pair"
```

This should reproduce the same anchor generation, assuming the code and settings are unchanged.

### Increase scenario difficulty

Use more stress-test scenarios:

```text
four_drone_emergency_scene
five_drone_dispersed_hotspot
six_drone_stress_test
```

or increase their weights in `SCENARIO_LIBRARY`.

### Increase spatial scale

If you increase `LIDAR_RANGE_M`, also update:

```text
AXIS_DISTANCE_BINS
PAIRWISE_DISTANCE_BINS
MIN_HORIZONTAL_DISTANCE_M
MIN_INTER_DRONE_DISTANCE_M
FLY_TO_TIMEOUT_S
```

Otherwise, the generator may still behave like a small 4 m test scene.

---

## 16. Important Notes

- The script generates scenario waypoints, not the LiDAR dataset itself.
- Unity is still responsible for recording point clouds and annotations.
- The manifest is metadata for analysis and traceability.
- This version assumes an upright 360-degree spinning LiDAR with vertical FOV.
- The old sideways-bicone mental model no longer applies.
- For detector training, do not blindly use all frames from every trajectory. Long smooth trajectories produce highly correlated neighboring frames.
- For tracking and trajectory prediction, keep full sequences because temporal continuity is the actual training/evaluation signal.
- Multi-drone stress scenarios are important for robustness, but they should not dominate detector training unless expected real-world data is also dense with drones.

---

## 17. Practical Starting Recommendation

For the first large generation batch, use mostly:

```text
single_drone_full_volume_coverage
single_drone_crossing
single_drone_vertical_climb_descent
single_drone_edge_of_fov
two_drone_far_apart
two_drone_close_pair
two_drone_vertical_stack
two_drone_opposite_crossing
```

After training a first model, evaluate it on:

```text
three_drone_mixed_hotspot
four_drone_emergency_scene
five_drone_dispersed_hotspot
six_drone_stress_test
```

Then generate more data for whichever scenario classes fail most often.
