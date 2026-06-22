#!/usr/bin/env python3

# 20000 Lidar Point Clouds are a good baseline for training the detection model (see nuscenes paper)
# Lidar records data at 20 Hz - 1000 seconds of flight time or 16 minutes in total

"""
scenario_driven_multi_drone.py

Scenario-driven multi-drone data generation for Unity + DroneManager + PX4.

This script replaces the old local random-walk generator with a scenario-driven
generator:

    1. Choose a scenario class.
    2. Choose the number of drones from that scenario.
    3. Generate synchronized anchor configurations that cover important spatial
       and multi-drone relations.
    4. Convert the anchors into smooth waypoint paths that can be flown by PX4.
    5. Log a manifest so every generated run can be traced and analyzed later.

Coordinate convention:
    Global NED-like coordinates:
        x = north
        y = east
        z = down
    Altitude above ground corresponds to negative z/down values.

LiDAR geometry model:
    The LiDAR is modeled as an upright 360-degree spinning LiDAR.
    Its azimuth coverage is horizontal around the vertical z/down axis.
    The vertical field of view is centered around the horizontal plane.

    In LiDAR-relative coordinates:
        horizontal_range = sqrt(north_offset^2 + east_offset^2)
        vertical_offset  = down_offset

    A point is valid if:
        MIN_HORIZONTAL_DISTANCE_M <= horizontal_range <= LIDAR_RANGE_M
        abs(vertical_offset) <= horizontal_range * tan(FOV_DEG / 2)

    This is a cone-like annular/donut volume around the vertical axis,
    not a bicone pointing along the north/x direction.
"""

from __future__ import annotations

import asyncio
import json
import math
import os
import random
import subprocess
from dataclasses import asdict, dataclass, field
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from dronemanager.core import DroneManager
from dronemanager.drone import DroneMAVSDK


# =============================================================================
# USER CONFIG
# =============================================================================

# Set to a scenario name to force one scenario.
# Leave as None to sample according to SCENARIO_LIBRARY weights.
FORCE_SCENARIO_NAME = "single_drone_full_volume_coverage"
RANDOM_SEED = None
RUN_ID_PREFIX = "scenario_run"
MANIFEST_ROOT = Path("C:/Datasets/UnityLidarDrone_scenario_manifests")

PRINT_GENERATED_PATHS = True
WRITE_MANIFEST = True


# =============================================================================
# LIDAR / SCENARIO GEOMETRY CONFIG
# =============================================================================

# LiDAR position in global NED-like coordinates.
# Example: z/down = -4 means LiDAR is 4 m above the ground.
LIDAR_NED = [0.0, 0.0, -4.0]

# Current simulation-scale range. Increase later when the world/sensor is scaled.
LIDAR_RANGE_M = 4.0

# Vertical full field of view. The LiDAR rotates 360 degrees horizontally.
# Half-angle is FOV_DEG / 2 above/below the horizontal plane.
FOV_DEG = 45.0

# Avoid points too close to the vertical LiDAR axis.
# Near the axis, the vertical FOV height becomes very small and paths can pass
# through poorly observable regions.
MIN_HORIZONTAL_DISTANCE_M = 0.75

# Minimum physical separation between drones at every anchor.
MIN_INTER_DRONE_DISTANCE_M = 0.75

PAUSE_BETWEEN_WAYPOINTS_S = 1.0
FLY_TO_TIMEOUT_S = 20.0
EMPTY_SCENE_DURATION_S = 20.0

# Whether to load DroneManager's external plugin for Unity visualization/streaming.
LOAD_EXTERNAL_PLUGIN = True


# =============================================================================
# PX4 / DRONE STARTUP CONFIG
# =============================================================================

AUTO_START_PX4 = False
STOP_PX4_ON_EXIT = True

PX4_AUTOPILOT_WSL_DIR = "/mnt/c/Users/mklemensthi/Documents/TONIC/PX4-Autopilot"
PX4_BINARY = "./build/px4_sitl_default/bin/px4"
PX4_SYS_AUTOSTART = "4001"
PX4_SIM_MODEL = "gz_x500"

# PX4 -i 1 ... -i 6.
PX4_FIRST_INSTANCE_INDEX = 1

PX4_START_DELAY_BETWEEN_DRONES_S = 5.0
PX4_WAIT_AFTER_FIRST_DRONE_S = 20.0


# =============================================================================
# COVERAGE BINS
# =============================================================================

# Absolute north/forward offset bins. These are not sensor range bins.
# They are useful for constructing straight left/right crossings at fixed x.
AXIS_DISTANCE_BINS = {
    "near": (0.75, 1.50),
    "mid": (1.50, 2.75),
    "far": (2.75, 3.60),
    "edge_range": (3.60, 4.00),
    "any": (0.75, 4.00),
}

# Lateral fraction magnitude bins. For a fixed north offset x, the maximum
# east offset is sqrt(LIDAR_RANGE_M^2 - x^2). The lateral fraction is
# abs(east_offset) / max_east_offset_at_x.
RADIAL_FRACTION_BINS = {
    "center": (0.00, 0.30),
    "middle": (0.30, 0.65),
    "edge_fov": (0.65, 1.00),
    "any": (0.00, 1.00),
}

# Side fraction is east_offset / max_east_offset_at_x.
SIDE_FRACTION_BINS = {
    "left": (-1.00, -0.33),
    "center": (-0.33, 0.33),
    "right": (0.33, 1.00),
    "any": (-1.00, 1.00),
}

# Vertical fraction is down_offset / max_vertical_offset_at_horizontal_range.
# In NED, negative down_offset is above the sensor plane, positive is below.
VERTICAL_FRACTION_BINS = {
    "above": (-1.00, -0.33),
    "level": (-0.33, 0.33),
    "below": (0.33, 1.00),
    "any": (-1.00, 1.00),
}

PAIRWISE_DISTANCE_BINS = {
    "very_close": (0.75, 1.00),
    "close": (1.00, 1.50),
    "medium": (1.50, 2.50),
    "far": (2.50, 4.50),
}


# =============================================================================
# SCENARIO DEFINITIONS
# =============================================================================

@dataclass(frozen=True)
class ScenarioSpec:
    name: str
    num_drones: int
    num_anchors: int
    weight: float
    description: str
    generator: str
    constraints: Dict[str, object] = field(default_factory=dict)


SCENARIO_LIBRARY: List[ScenarioSpec] = [
    ScenarioSpec(
        name="empty_scene",
        num_drones=0,
        num_anchors=1,
        weight=0.05,
        generator="empty",
        description="No drone present. Useful for false-positive control.",
    ),
    ScenarioSpec(
        name="single_drone_full_volume_coverage",
        num_drones=1,
        num_anchors=10,
        weight=0.22,
        generator="single_coverage_tour",
        description="One drone visits broad forward, lateral, elevation, and FOV-edge bins.",
    ),
    ScenarioSpec(
        name="single_drone_crossing",
        num_drones=1,
        num_anchors=8,
        weight=0.10,
        generator="single_crossing",
        description="One drone performs a wide left/right crossing through the horizontal 360-degree LiDAR volume.",
    ),
    ScenarioSpec(
        name="single_drone_vertical_climb_descent",
        num_drones=1,
        num_anchors=8,
        weight=0.10,
        generator="single_vertical",
        description="One drone changes altitude while remaining near a similar bearing.",
    ),
    ScenarioSpec(
        name="single_drone_edge_of_fov",
        num_drones=1,
        num_anchors=8,
        weight=0.08,
        generator="single_edge",
        description="One drone moves near the edge of the LiDAR field of view.",
    ),
    ScenarioSpec(
        name="two_drone_far_apart",
        num_drones=2,
        num_anchors=7,
        weight=0.08,
        generator="two_far_apart",
        description="Two drones remain well separated across the field of view.",
    ),
    ScenarioSpec(
        name="two_drone_close_pair",
        num_drones=2,
        num_anchors=7,
        weight=0.08,
        generator="two_close_pair",
        description="Two drones form a close pair while still respecting safety separation.",
    ),
    ScenarioSpec(
        name="two_drone_vertical_stack",
        num_drones=2,
        num_anchors=7,
        weight=0.08,
        generator="two_vertical_stack",
        description="Two drones have similar horizontal position but different altitude.",
    ),
    ScenarioSpec(
        name="two_drone_opposite_crossing",
        num_drones=2,
        num_anchors=8,
        weight=0.06,
        generator="two_opposite_crossing",
        description="Two drones cross the sensor volume in opposite directions.",
    ),
    ScenarioSpec(
        name="three_drone_mixed_hotspot",
        num_drones=3,
        num_anchors=7,
        weight=0.05,
        generator="multi_mixed",
        description="Three drones with one close relation and one dispersed target.",
        constraints={"include_close_pair": True, "include_far_target": True},
    ),
    ScenarioSpec(
        name="four_drone_emergency_scene",
        num_drones=4,
        num_anchors=6,
        weight=0.04,
        generator="multi_mixed",
        description="Four drones in a mixed emergency-scene style layout.",
        constraints={"include_close_pair": True, "include_edge_target": True},
    ),
    ScenarioSpec(
        name="five_drone_dispersed_hotspot",
        num_drones=5,
        num_anchors=6,
        weight=0.025,
        generator="multi_mixed",
        description="Five drones dispersed across the observable volume.",
        constraints={"include_far_target": True, "prefer_dispersed": True},
    ),
    ScenarioSpec(
        name="six_drone_stress_test",
        num_drones=6,
        num_anchors=5,
        weight=0.015,
        generator="multi_mixed",
        description="Six-drone stress test with close, far, and edge targets.",
        constraints={
            "include_close_pair": True,
            "include_far_target": True,
            "include_edge_target": True,
        },
    ),
]


# =============================================================================
# VECTOR / GEOMETRY HELPERS
# =============================================================================

Vec3 = List[float]
AnchorConfig = Dict[str, Vec3]


def vec_sub(a: Vec3, b: Vec3) -> Vec3:
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def vec_norm(v: Vec3) -> float:
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def distance(a: Vec3, b: Vec3) -> float:
    return vec_norm(vec_sub(a, b))


def half_angle_rad() -> float:
    return math.radians(FOV_DEG / 2.0)


def max_vertical_offset_for_horizontal_range(horizontal_range: float) -> float:
    """
    Vertical half-height of the LiDAR field of view at a given horizontal range.

    The LiDAR is upright and scans 360 degrees around the vertical z/down axis.
    Its vertical FOV is centered around the horizontal plane. Therefore the
    admissible vertical offset grows linearly with horizontal range:

        |down_offset| <= horizontal_range * tan(FOV_DEG / 2)
    """
    return horizontal_range * math.tan(half_angle_rad())


# Backward-compatible name used by some old comments/scripts.
# In the upright-LiDAR model, this is the vertical half-height at horizontal range.
def max_radius_for_axis(axis_abs: float) -> float:
    return max_vertical_offset_for_horizontal_range(axis_abs)


def max_east_offset_for_north_offset(north_offset: float) -> float:
    """
    For a fixed north/x offset, compute the largest east/y offset that remains
    inside the horizontal LiDAR range circle.

    horizontal_range^2 = north_offset^2 + east_offset^2 <= LIDAR_RANGE_M^2
    """
    remaining = LIDAR_RANGE_M ** 2 - north_offset ** 2
    return math.sqrt(max(0.0, remaining))


def horizontal_range_of_relative(rel: Vec3) -> float:
    return math.sqrt(rel[0] ** 2 + rel[1] ** 2)


def is_inside_lidar_bicone(point_ned: Vec3) -> bool:
    """
    Check whether a point lies inside the upright 360-degree LiDAR volume.

    This function keeps the old name for compatibility with the rest of the
    script, but the geometry is no longer a bicone pointing along north/x.

    Valid region, relative to the LiDAR:
        rho = sqrt(north^2 + east^2)
        MIN_HORIZONTAL_DISTANCE_M <= rho <= LIDAR_RANGE_M
        abs(down) <= rho * tan(FOV_DEG / 2)
    """
    rel = vec_sub(point_ned, LIDAR_NED)
    north = rel[0]
    east = rel[1]
    down = rel[2]

    horizontal_range = math.sqrt(north * north + east * east)

    if horizontal_range < MIN_HORIZONTAL_DISTANCE_M:
        return False
    if horizontal_range > LIDAR_RANGE_M:
        return False

    max_vertical = max_vertical_offset_for_horizontal_range(horizontal_range)
    return abs(down) <= max_vertical + 1e-9


def lidar_relative_metadata(point_ned: Vec3) -> Dict[str, float]:
    rel = vec_sub(point_ned, LIDAR_NED)
    north = rel[0]
    east = rel[1]
    down = rel[2]

    horizontal_range = math.sqrt(north * north + east * east)
    range_3d = math.sqrt(north * north + east * east + down * down)
    bearing_rad = math.atan2(east, north)
    bearing_deg = math.degrees(bearing_rad)

    max_vertical = max(max_vertical_offset_for_horizontal_range(horizontal_range), 1e-9)
    max_east_at_north = max(max_east_offset_for_north_offset(north), 1e-9)

    lateral_fraction = east / max_east_at_north
    lateral_fraction_abs = abs(lateral_fraction)
    vertical_fraction = down / max_vertical
    horizontal_range_fraction = horizontal_range / max(LIDAR_RANGE_M, 1e-9)

    return {
        # Old aliases kept to avoid breaking manifest consumers.
        "axis": north,
        "axis_abs": abs(north),
        "east": east,
        "down": down,
        "range_3d": range_3d,
        "radial": abs(east),
        "radial_fraction": lateral_fraction_abs,
        "side_fraction": lateral_fraction,
        "vertical_fraction": vertical_fraction,
        "distance_to_fov_edge_fraction": max(0.0, 1.0 - abs(vertical_fraction)),

        # New explicit upright-LiDAR metadata.
        "north_offset": north,
        "east_offset": east,
        "down_offset": down,
        "horizontal_range": horizontal_range,
        "horizontal_range_fraction": horizontal_range_fraction,
        "bearing_rad": bearing_rad,
        "bearing_deg": bearing_deg,
        "max_vertical_offset": max_vertical,
        "max_east_offset_at_north": max_east_at_north,
        "lateral_fraction": lateral_fraction,
        "lateral_fraction_abs": lateral_fraction_abs,
        "distance_to_horizontal_range_edge_m": max(0.0, LIDAR_RANGE_M - horizontal_range),
        "distance_to_vertical_fov_edge_fraction": max(0.0, 1.0 - abs(vertical_fraction)),
    }


def in_interval(value: float, interval: Tuple[float, float]) -> bool:
    lo, hi = interval
    return lo <= value <= hi


def make_point_from_fractions(axis: float, side_fraction: float, vertical_fraction: float) -> Vec3:
    """
    Construct a point using the upright-LiDAR geometry.

    Parameters
    ----------
    axis:
        North/x offset from the LiDAR in meters. This is retained from the old
        script name, but it is no longer the LiDAR optical axis. It is simply
        the forward/backward coordinate used for straight crossing trajectories.

    side_fraction:
        East/y offset as a fraction of the maximum east/y offset that is still
        inside the horizontal range circle at this north/x offset.

        side_fraction = -1.0 -> left horizontal range boundary
        side_fraction =  0.0 -> center line at this north/x offset
        side_fraction = +1.0 -> right horizontal range boundary

    vertical_fraction:
        Down/z offset as a fraction of the vertical FOV half-height at the
        resulting horizontal range.

        vertical_fraction = -1.0 -> upper vertical FOV boundary
        vertical_fraction =  0.0 -> LiDAR horizontal plane
        vertical_fraction = +1.0 -> lower vertical FOV boundary
    """
    if abs(axis) >= LIDAR_RANGE_M:
        # Exactly at the horizontal range edge there is no lateral room left.
        # Use a tiny margin so side fractions remain meaningful.
        axis = math.copysign(LIDAR_RANGE_M - 1e-6, axis)

    max_east = max_east_offset_for_north_offset(axis)
    east = side_fraction * max_east

    horizontal_range = math.sqrt(axis * axis + east * east)
    max_vertical = max_vertical_offset_for_horizontal_range(horizontal_range)
    down = vertical_fraction * max_vertical

    point = [
        LIDAR_NED[0] + axis,
        LIDAR_NED[1] + east,
        LIDAR_NED[2] + down,
    ]

    if not is_inside_lidar_bicone(point):
        raise ValueError(
            "Point generated from fractions is outside the upright LiDAR volume. "
            f"axis={axis:.3f}, side_fraction={side_fraction:.3f}, "
            f"vertical_fraction={vertical_fraction:.3f}, point={point}"
        )

    return point


def make_point_from_polar(horizontal_range: float, bearing_rad: float, vertical_fraction: float) -> Vec3:
    """
    Construct a point from horizontal polar coordinates around the upright LiDAR.

    horizontal_range:
        Horizontal distance from the LiDAR in the north/east plane.

    bearing_rad:
        Azimuth angle around the vertical axis. 0 means +north, pi/2 means +east.

    vertical_fraction:
        Down/z offset as fraction of the vertical FOV half-height.
    """
    horizontal_range = min(max(horizontal_range, MIN_HORIZONTAL_DISTANCE_M), LIDAR_RANGE_M)
    north = horizontal_range * math.cos(bearing_rad)
    east = horizontal_range * math.sin(bearing_rad)
    down = vertical_fraction * max_vertical_offset_for_horizontal_range(horizontal_range)

    point = [
        LIDAR_NED[0] + north,
        LIDAR_NED[1] + east,
        LIDAR_NED[2] + down,
    ]

    if not is_inside_lidar_bicone(point):
        raise ValueError("Point generated from polar coordinates is outside the upright LiDAR volume.")

    return point


def sample_point_from_bins(
    axis_bin: str = "any",
    radial_bin: str = "any",
    side_bin: str = "any",
    vertical_bin: str = "any",
    axis_sign: str = "any",
    max_attempts: int = 5000,
) -> Vec3:
    """
    Sample a point inside the upright 360-degree LiDAR volume while satisfying
    the existing coverage-bin interface.

    The parameter names are kept for compatibility with the original scenario
    functions, but their meaning is updated:

    axis_bin:
        Absolute north/x offset bin.

    radial_bin:
        Lateral magnitude bin, i.e. abs(east_offset) divided by the maximum
        east offset possible at the sampled north/x offset.

    side_bin:
        Left/center/right sign bin for east/y offset.

    vertical_bin:
        Above/level/below bin for down/z offset as a fraction of the vertical
        FOV half-height.

    axis_sign:
        "front" -> positive north offset
        "back"  -> negative north offset
        "any"   -> both front and back
    """
    # Avoid incompatible lateral-magnitude / side-sign requests.
    # Example: radial_bin="center" means |side_fraction| <= 0.30, while
    # side_bin="left" requires side_fraction <= -0.33. No point can satisfy
    # both. In such cases we keep the semantic side request and relax the
    # lateral magnitude to the nearest feasible bin.
    if radial_bin == "center" and side_bin in ("left", "right"):
        radial_bin = "middle"
    if radial_bin == "edge_fov" and side_bin == "center":
        side_bin = random.choice(["left", "right"])

    axis_lo, axis_hi = AXIS_DISTANCE_BINS[axis_bin]
    lateral_lo, lateral_hi = RADIAL_FRACTION_BINS[radial_bin]
    side_interval = SIDE_FRACTION_BINS[side_bin]
    vertical_interval = VERTICAL_FRACTION_BINS[vertical_bin]

    for _ in range(max_attempts):
        if axis_sign == "front":
            sign = 1.0
        elif axis_sign == "back":
            sign = -1.0
        elif axis_sign == "any":
            sign = random.choice([-1.0, 1.0])
        else:
            raise ValueError("axis_sign must be 'front', 'back', or 'any'.")

        axis_abs = random.uniform(axis_lo, min(axis_hi, LIDAR_RANGE_M - 1e-6))
        north = sign * axis_abs

        max_east = max_east_offset_for_north_offset(north)
        if max_east <= 1e-9:
            continue

        # Sample lateral fraction with a requested magnitude and requested side.
        if side_bin == "left":
            side_sign = -1.0
        elif side_bin == "right":
            side_sign = 1.0
        elif side_bin == "center":
            side_sign = random.choice([-1.0, 1.0])
        else:
            side_sign = random.choice([-1.0, 1.0])

        if side_bin == "center":
            # Center should stay near zero even if radial_bin is broad.
            lo = max(lateral_lo, 0.0)
            hi = min(lateral_hi, SIDE_FRACTION_BINS["center"][1])
            if lo > hi:
                continue
            side_mag = random.uniform(lo, hi)
        else:
            side_mag = random.uniform(lateral_lo, lateral_hi)

        side_fraction = side_sign * side_mag

        # Respect explicit side interval after constructing side fraction.
        if not in_interval(side_fraction, side_interval):
            continue

        vertical_fraction = random.uniform(*vertical_interval)
        point = make_point_from_fractions(north, side_fraction, vertical_fraction)

        if not is_inside_lidar_bicone(point):
            continue

        return point

    raise RuntimeError(
        "Could not sample a point with bins "
        f"axis={axis_bin}, radial={radial_bin}, side={side_bin}, "
        f"vertical={vertical_bin}, axis_sign={axis_sign}."
    )


def sample_point_near(base: Vec3, distance_bin: str, max_attempts: int = 5000) -> Vec3:
    lo, hi = PAIRWISE_DISTANCE_BINS[distance_bin]

    for _ in range(max_attempts):
        candidate = sample_point_from_bins("any", "any", "any", "any", "any")
        d = distance(base, candidate)
        if lo <= d <= hi:
            return candidate

    raise RuntimeError(f"Could not sample point near base with distance bin {distance_bin}.")


def all_positions_valid(positions: AnchorConfig) -> bool:
    for point in positions.values():
        if not is_inside_lidar_bicone(point):
            return False

    names = list(positions.keys())
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            if distance(positions[names[i]], positions[names[j]]) < MIN_INTER_DRONE_DISTANCE_M:
                return False

    return True


def pairwise_distances(positions: AnchorConfig) -> List[float]:
    names = list(positions.keys())
    values = []
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            values.append(distance(positions[names[i]], positions[names[j]]))
    return values


def has_close_pair(positions: AnchorConfig, max_close_distance: float = 1.50) -> bool:
    return any(d <= max_close_distance for d in pairwise_distances(positions))


def has_far_pair_or_target(positions: AnchorConfig, min_distance: float = 2.50) -> bool:
    distances = pairwise_distances(positions)
    if distances and max(distances) >= min_distance:
        return True

    for point in positions.values():
        meta = lidar_relative_metadata(point)
        if meta["horizontal_range"] >= AXIS_DISTANCE_BINS["far"][0]:
            return True

    return False


def has_edge_target(positions: AnchorConfig) -> bool:
    for point in positions.values():
        meta = lidar_relative_metadata(point)
        if meta["lateral_fraction_abs"] >= RADIAL_FRACTION_BINS["edge_fov"][0]:
            return True
        if meta["horizontal_range"] >= AXIS_DISTANCE_BINS["edge_range"][0]:
            return True
        if abs(meta["vertical_fraction"]) >= VERTICAL_FRACTION_BINS["below"][0]:
            return True
    return False


def satisfies_scenario_constraints(positions: AnchorConfig, spec: ScenarioSpec) -> bool:
    if not all_positions_valid(positions):
        return False

    constraints = spec.constraints

    if constraints.get("include_close_pair") and not has_close_pair(positions):
        return False
    if constraints.get("include_far_target") and not has_far_pair_or_target(positions):
        return False
    if constraints.get("include_edge_target") and not has_edge_target(positions):
        return False
    if constraints.get("prefer_dispersed"):
        distances = pairwise_distances(positions)
        if distances and min(distances) < 1.25:
            return False

    return True


# =============================================================================
# SCENARIO SAMPLING
# =============================================================================

def get_scenario_by_name(name: str) -> ScenarioSpec:
    for spec in SCENARIO_LIBRARY:
        if spec.name == name:
            return spec
    known = ", ".join(s.name for s in SCENARIO_LIBRARY)
    raise KeyError(f"Unknown scenario '{name}'. Known scenarios: {known}")


def sample_scenario() -> ScenarioSpec:
    if FORCE_SCENARIO_NAME is not None:
        return get_scenario_by_name(FORCE_SCENARIO_NAME)

    weights = [spec.weight for spec in SCENARIO_LIBRARY]
    return random.choices(SCENARIO_LIBRARY, weights=weights, k=1)[0]


def drone_id(index: int) -> str:
    return f"drone{index + 1}"


def generate_anchor_sequence(spec: ScenarioSpec) -> List[AnchorConfig]:
    if spec.generator == "empty":
        return [{}]
    if spec.generator == "single_coverage_tour":
        return generate_single_coverage_tour(spec)
    if spec.generator == "single_crossing":
        return generate_single_crossing(spec)
    if spec.generator == "single_vertical":
        return generate_single_vertical(spec)
    if spec.generator == "single_edge":
        return generate_single_edge(spec)
    if spec.generator == "two_far_apart":
        return generate_two_far_apart(spec)
    if spec.generator == "two_close_pair":
        return generate_two_close_pair(spec)
    if spec.generator == "two_vertical_stack":
        return generate_two_vertical_stack(spec)
    if spec.generator == "two_opposite_crossing":
        return generate_two_opposite_crossing(spec)
    if spec.generator == "multi_mixed":
        return generate_multi_mixed(spec)
    raise KeyError(f"Unknown scenario generator: {spec.generator}")


def generate_single_coverage_tour(spec: ScenarioSpec) -> List[AnchorConfig]:
    axis_bins = ["near", "mid", "far", "edge_range", "mid", "far", "near", "edge_range", "mid", "far"]
    radial_bins = ["center", "middle", "middle", "edge_fov", "middle", "edge_fov", "middle", "middle", "edge_fov", "middle"]
    side_bins = ["center", "left", "right", "left", "center", "right", "left", "center", "left", "right"]
    vertical_bins = ["level", "below", "above", "level", "above", "below", "level", "above", "below", "level"]

    anchors: List[AnchorConfig] = []
    for k in range(spec.num_anchors):
        point = sample_point_from_bins(
            axis_bin=axis_bins[k % len(axis_bins)],
            radial_bin=radial_bins[k % len(radial_bins)],
            side_bin=side_bins[k % len(side_bins)],
            vertical_bin=vertical_bins[k % len(vertical_bins)],
            axis_sign="any",
        )
        anchors.append({"drone1": point})

    return anchors


def generate_single_crossing(spec: ScenarioSpec) -> List[AnchorConfig]:
    """
    One drone crosses left/right through the horizontal LiDAR range circle.

    The old script treated the LiDAR as a bicone along north/x. In the upright
    LiDAR model, the wide crossing is created by fixing a north/x offset and
    sweeping east/y from one horizontal range boundary to the other.
    """
    # Keep the crossing away from the vertical LiDAR axis so the whole straight
    # segment remains observable. Smaller absolute axis gives longer travel,
    # but too small would pass close to the LiDAR axis/hole.
    axis = random.choice([-1.0, 1.0]) * random.uniform(1.00, 2.25)

    # Keep vertical motion moderate so this scenario mostly tests horizontal
    # left/right coverage.
    vertical_fraction = random.choice([-0.15, 0.0, 0.15])
    start_side = random.choice([-0.95, 0.95])
    end_side = -start_side

    anchors: List[AnchorConfig] = []
    for k in range(spec.num_anchors):
        t = k / max(1, spec.num_anchors - 1)
        side_fraction = (1.0 - t) * start_side + t * end_side
        vf = vertical_fraction + 0.10 * math.sin(2.0 * math.pi * t)
        anchors.append({"drone1": make_point_from_fractions(axis, side_fraction, vf)})

    return anchors



def generate_single_vertical(spec: ScenarioSpec) -> List[AnchorConfig]:
    axis = random.choice([-1.0, 1.0]) * random.uniform(1.75, 3.25)
    side_fraction = random.uniform(-0.25, 0.25)
    start_vertical = random.choice([-0.75, 0.75])
    end_vertical = -start_vertical

    anchors: List[AnchorConfig] = []
    for k in range(spec.num_anchors):
        t = k / max(1, spec.num_anchors - 1)
        vertical_fraction = (1.0 - t) * start_vertical + t * end_vertical
        anchors.append({"drone1": make_point_from_fractions(axis, side_fraction, vertical_fraction)})

    return anchors


def generate_single_edge(spec: ScenarioSpec) -> List[AnchorConfig]:
    anchors: List[AnchorConfig] = []
    for _ in range(spec.num_anchors):
        anchors.append({
            "drone1": sample_point_from_bins(
                axis_bin=random.choice(["far", "edge_range", "mid"]),
                radial_bin="edge_fov",
                side_bin=random.choice(["left", "right", "any"]),
                vertical_bin=random.choice(["above", "below", "level", "any"]),
                axis_sign="any",
            )
        })
    return anchors


def generate_two_far_apart(spec: ScenarioSpec) -> List[AnchorConfig]:
    anchors: List[AnchorConfig] = []

    for _ in range(spec.num_anchors):
        for _attempt in range(5000):
            p1 = sample_point_from_bins(
                axis_bin=random.choice(["near", "mid"]),
                radial_bin=random.choice(["center", "middle"]),
                side_bin=random.choice(["left", "center", "right"]),
                vertical_bin=random.choice(["above", "level", "below"]),
                axis_sign="any",
            )
            p2 = sample_point_from_bins(
                axis_bin=random.choice(["far", "edge_range", "mid"]),
                radial_bin=random.choice(["middle", "edge_fov"]),
                side_bin=random.choice(["left", "right"]),
                vertical_bin=random.choice(["above", "level", "below"]),
                axis_sign="any",
            )
            config = {"drone1": p1, "drone2": p2}
            if all_positions_valid(config) and distance(p1, p2) >= PAIRWISE_DISTANCE_BINS["far"][0]:
                anchors.append(config)
                break
        else:
            raise RuntimeError("Could not sample two_drone_far_apart anchor.")

    return anchors


def generate_two_close_pair(spec: ScenarioSpec) -> List[AnchorConfig]:
    anchors: List[AnchorConfig] = []

    for _ in range(spec.num_anchors):
        for _attempt in range(5000):
            base = sample_point_from_bins(
                axis_bin=random.choice(["near", "mid", "far"]),
                radial_bin=random.choice(["center", "middle", "edge_fov"]),
                side_bin=random.choice(["left", "center", "right", "any"]),
                vertical_bin=random.choice(["above", "level", "below", "any"]),
                axis_sign="any",
            )
            other = sample_point_near(base, distance_bin=random.choice(["very_close", "close"]))
            config = {"drone1": base, "drone2": other}
            if all_positions_valid(config) and has_close_pair(config):
                anchors.append(config)
                break
        else:
            raise RuntimeError("Could not sample two_drone_close_pair anchor.")

    return anchors


def generate_two_vertical_stack(spec: ScenarioSpec) -> List[AnchorConfig]:
    anchors: List[AnchorConfig] = []

    for k in range(spec.num_anchors):
        t = k / max(1, spec.num_anchors - 1)
        side_fraction = random.uniform(-0.20, 0.20)
        low_v = 0.60 - 0.20 * math.sin(2.0 * math.pi * t)
        high_v = -0.60 + 0.20 * math.sin(2.0 * math.pi * t)

        for _attempt in range(2000):
            axis = random.choice([-1.0, 1.0]) * random.uniform(2.30, 3.80)
            p1 = make_point_from_fractions(axis, side_fraction, low_v)
            p2 = make_point_from_fractions(axis, side_fraction, high_v)
            config = {"drone1": p1, "drone2": p2}
            if all_positions_valid(config):
                anchors.append(config)
                break
        else:
            raise RuntimeError("Could not create valid vertical stack anchor.")

    return anchors


def generate_two_opposite_crossing(spec: ScenarioSpec) -> List[AnchorConfig]:
    """
    Two drones cross in opposite east/west directions at a fixed north/x offset.
    They are vertically separated near the middle to avoid violating the minimum
    inter-drone distance.
    """
    axis = random.choice([-1.0, 1.0]) * random.uniform(1.25, 2.50)

    anchors: List[AnchorConfig] = []
    for k in range(spec.num_anchors):
        t = k / max(1, spec.num_anchors - 1)
        side1 = -0.90 + 1.80 * t
        side2 = 0.90 - 1.80 * t

        # Separation is strongest around the crossing midpoint.
        mid_weight = math.sin(math.pi * t)
        v_sep = 0.15 + 0.45 * mid_weight
        p1 = make_point_from_fractions(axis, side1, v_sep)
        p2 = make_point_from_fractions(axis, side2, -v_sep)
        config = {"drone1": p1, "drone2": p2}

        if not all_positions_valid(config):
            raise RuntimeError("Could not create valid opposite crossing anchor.")

        anchors.append(config)

    return anchors



def generate_multi_mixed(spec: ScenarioSpec) -> List[AnchorConfig]:
    anchors: List[AnchorConfig] = []

    for _ in range(spec.num_anchors):
        for _attempt in range(10000):
            positions: AnchorConfig = {}

            for i in range(spec.num_drones):
                positions[drone_id(i)] = sample_point_from_bins(
                    axis_bin=random.choice(["near", "mid", "far", "edge_range"]),
                    radial_bin=random.choice(["center", "middle", "edge_fov", "any"]),
                    side_bin=random.choice(["left", "center", "right", "any"]),
                    vertical_bin=random.choice(["above", "level", "below", "any"]),
                    axis_sign="any",
                )

            if spec.constraints.get("include_close_pair") and spec.num_drones >= 2:
                base = sample_point_from_bins(
                    axis_bin=random.choice(["mid", "far"]),
                    radial_bin=random.choice(["middle", "edge_fov"]),
                    side_bin=random.choice(["left", "center", "right"]),
                    vertical_bin=random.choice(["above", "level", "below"]),
                    axis_sign="any",
                )
                positions["drone1"] = base
                positions["drone2"] = sample_point_near(base, "close")

            if spec.constraints.get("include_edge_target") and spec.num_drones >= 1:
                positions[drone_id(spec.num_drones - 1)] = sample_point_from_bins(
                    axis_bin=random.choice(["far", "edge_range"]),
                    radial_bin="edge_fov",
                    side_bin=random.choice(["left", "right", "any"]),
                    vertical_bin=random.choice(["above", "level", "below", "any"]),
                    axis_sign="any",
                )

            if spec.constraints.get("include_far_target") and spec.num_drones >= 1:
                positions[drone_id(spec.num_drones - 1)] = sample_point_from_bins(
                    axis_bin=random.choice(["far", "edge_range"]),
                    radial_bin=random.choice(["middle", "edge_fov"]),
                    side_bin=random.choice(["left", "right", "any"]),
                    vertical_bin=random.choice(["above", "level", "below", "any"]),
                    axis_sign="any",
                )

            if satisfies_scenario_constraints(positions, spec):
                anchors.append(positions)
                break
        else:
            raise RuntimeError(f"Could not sample valid multi_mixed anchor for {spec.name}.")

    return anchors


def anchors_to_paths(anchors: List[AnchorConfig]) -> Dict[str, List[Vec3]]:
    if not anchors or not anchors[0]:
        return {}

    names = sorted(anchors[0].keys())
    paths: Dict[str, List[Vec3]] = {name: [] for name in names}

    for anchor in anchors:
        for name in names:
            paths[name].append(anchor[name])

    return paths


def generate_scenario_global_paths(spec: ScenarioSpec) -> Tuple[List[AnchorConfig], Dict[str, List[Vec3]]]:
    anchors = generate_anchor_sequence(spec)
    paths = anchors_to_paths(anchors)
    return anchors, paths


# =============================================================================
# DRONE CONFIG AND PX4 HELPERS
# =============================================================================

def validate_num_drones(num_drones: int) -> None:
    if not 0 <= num_drones <= 6:
        raise ValueError("num_drones must be between 0 and 6.")


def px4_instance_index_for_drone_number(drone_number: int) -> int:
    return PX4_FIRST_INSTANCE_INDEX + (drone_number - 1)


def mavsdk_port_for_px4_instance(px4_instance_index: int) -> int:
    return 14540 + px4_instance_index


def build_drone_config(num_drones: int) -> Dict[str, Dict[str, object]]:
    validate_num_drones(num_drones)
    configs: Dict[str, Dict[str, object]] = {}

    for drone_number in range(1, num_drones + 1):
        name = f"drone{drone_number}"
        px4_idx = px4_instance_index_for_drone_number(drone_number)
        mavsdk_port = mavsdk_port_for_px4_instance(px4_idx)

        configs[name] = {
            "connection": f"udp://:{mavsdk_port}",
            "home_offset_ned": [0.0, 0.0, 0.0],
            "yaw": 0.0,
            "px4_instance_index": px4_idx,
            "spawn_pose": [0.0, float(px4_idx * 2), 0.0, 0.0, 0.0, 0.0],
        }

    return configs


def build_px4_command(cfg: Dict[str, object]) -> str:
    px4_idx = int(cfg["px4_instance_index"])
    x, y, z, roll, pitch, yaw = cfg["spawn_pose"]
    pose_string = f"{x},{y},{z},{roll},{pitch},{yaw}"

    return (
        f'cd "{PX4_AUTOPILOT_WSL_DIR}" && '
        f'PX4_SYS_AUTOSTART={PX4_SYS_AUTOSTART} '
        f'PX4_SIM_MODEL={PX4_SIM_MODEL} '
        f'PX4_GZ_MODEL_POSE="{pose_string}" '
        f'{PX4_BINARY} -i {px4_idx}'
    )


def start_px4_instances(drone_configs: Dict[str, Dict[str, object]]) -> List[subprocess.Popen]:
    if not AUTO_START_PX4:
        return []

    processes: List[subprocess.Popen] = []
    print("\nStarting PX4 SITL instances...")
    print("=" * 80)

    for idx, (name, cfg) in enumerate(drone_configs.items()):
        command = build_px4_command(cfg)
        print(f"[{name}] PX4 command:")
        print(command)
        print()

        if os.name == "nt":
            proc = subprocess.Popen(
                ["wsl.exe", "bash", "-lc", command],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        else:
            proc = subprocess.Popen(
                ["bash", "-lc", command],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

        processes.append(proc)

        if PX4_START_DELAY_BETWEEN_DRONES_S > 0:
            import time
            time.sleep(PX4_START_DELAY_BETWEEN_DRONES_S)

        if idx == 0 and PX4_WAIT_AFTER_FIRST_DRONE_S > 0:
            print(f"Waiting {PX4_WAIT_AFTER_FIRST_DRONE_S} seconds after first PX4 startup...\n")
            import time
            time.sleep(PX4_WAIT_AFTER_FIRST_DRONE_S)

    print(f"Started {len(processes)} PX4 process(es).")
    return processes


def stop_px4_instances(processes: List[subprocess.Popen]) -> None:
    if not processes:
        return

    print("\nStopping PX4 SITL instances...")
    for proc in processes:
        if proc.poll() is None:
            proc.terminate()

    import time
    time.sleep(3.0)

    for proc in processes:
        if proc.poll() is None:
            proc.kill()

    print("PX4 processes stopped.")


# =============================================================================
# PATH CONVERSION AND PRINTING
# =============================================================================

def global_ned_to_drone_local(global_point_ned: Vec3, home_offset_ned: Vec3) -> Vec3:
    return [
        global_point_ned[0] - home_offset_ned[0],
        global_point_ned[1] - home_offset_ned[1],
        global_point_ned[2] - home_offset_ned[2],
    ]


def altitude_from_local_ned(local_point: Vec3) -> float:
    return max(1.0, -local_point[2])


def print_paths(spec: ScenarioSpec, global_paths: Dict[str, List[Vec3]]) -> None:
    print("\nGenerated scenario:")
    print("=" * 80)
    print(f"Scenario: {spec.name}")
    print(f"Description: {spec.description}")
    print(f"Number of drones: {spec.num_drones}")
    print(f"Number of anchors: {spec.num_anchors}")
    print("=" * 80)

    if spec.num_drones == 0:
        print("Empty scene: no drone waypoints.")
        return

    for name, path in global_paths.items():
        print(f"\n{name}:")
        for idx, p in enumerate(path):
            meta = lidar_relative_metadata(p)
            print(
                f"  Anchor {idx + 1:02d}: "
                f"global_ned=[{p[0]:7.2f}, {p[1]:7.2f}, {p[2]:7.2f}], "
                f"north={meta['north_offset']:6.2f}, "
                f"radial_frac={meta['radial_fraction']:5.2f}, "
                f"side_frac={meta['side_fraction']:6.2f}, "
                f"vertical_frac={meta['vertical_fraction']:6.2f}"
            )


def create_run_id() -> str:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return f"{RUN_ID_PREFIX}_{stamp}"


def write_manifest(
    run_id: str,
    spec: ScenarioSpec,
    anchors: List[AnchorConfig],
    global_paths: Dict[str, List[Vec3]],
    drone_configs: Dict[str, Dict[str, object]],
) -> Path:
    MANIFEST_ROOT.mkdir(parents=True, exist_ok=True)

    manifest = {
        "run_id": run_id,
        "random_seed": RANDOM_SEED,
        "created_at": datetime.now().isoformat(timespec="seconds"),
        "lidar": {
            "lidar_ned": LIDAR_NED,
            "lidar_range_m": LIDAR_RANGE_M,
            "fov_deg": FOV_DEG,
            "min_horizontal_distance_m": MIN_HORIZONTAL_DISTANCE_M,
            "min_inter_drone_distance_m": MIN_INTER_DRONE_DISTANCE_M,
        },
        "scenario": asdict(spec),
        "drone_configs": drone_configs,
        "anchors": anchors,
        "path_metadata": {
            name: [lidar_relative_metadata(point) for point in path]
            for name, path in global_paths.items()
        },
    }

    out_path = MANIFEST_ROOT / f"{run_id}_{spec.name}.json"
    with out_path.open("w", encoding="utf-8") as f:
        json.dump(manifest, f, indent=2)

    print("\nWrote scenario manifest:")
    print(out_path)
    return out_path


# =============================================================================
# DRONEMANAGER HELPERS
# =============================================================================

async def connect_drone(dm: DroneManager, name: str, connection: str) -> None:
    print(f"[{name}] Connecting on {connection}")
    await dm.connect_to_drone(name=name, drone_address=connection, timeout=500)

    if name not in dm.drones:
        raise RuntimeError(f"[{name}] Failed to connect on {connection}")

    print(f"[{name}] Connected successfully")


async def arm_and_takeoff_to_first_waypoint(dm: DroneManager, name: str, first_local_wp: Vec3) -> None:
    takeoff_altitude = altitude_from_local_ned(first_local_wp)

    print(f"[{name}] Arming")
    await dm.arm(name)
    await asyncio.sleep(2.0)

    print(f"[{name}] Taking off to {takeoff_altitude:.2f} m")
    await dm.takeoff(name, altitude=takeoff_altitude)
    await asyncio.sleep(8.0)


async def safe_fly_to(dm: DroneManager, name: str, local_wp: Vec3, yaw: float, timeout_s: float) -> bool:
    try:
        print(
            f"[{name}] fly_to local=[{local_wp[0]:.2f}, {local_wp[1]:.2f}, {local_wp[2]:.2f}], "
            f"yaw={yaw:.1f}"
        )
        await asyncio.wait_for(dm.fly_to(name, local=local_wp, yaw=yaw), timeout=timeout_s)
        return True
    except asyncio.TimeoutError:
        print(f"[{name}] WARNING: fly_to timeout. Continuing to next waypoint.")
        return False


async def fly_scenario_path(dm: DroneManager, name: str, local_path: List[Vec3], yaw: float) -> None:
    print(f"[{name}] Starting scenario path with {len(local_path)} anchors.")

    for idx, local_wp in enumerate(local_path):
        print(f"[{name}] Anchor {idx + 1}/{len(local_path)}")
        await safe_fly_to(dm=dm, name=name, local_wp=local_wp, yaw=yaw, timeout_s=FLY_TO_TIMEOUT_S)

        if PAUSE_BETWEEN_WAYPOINTS_S > 0:
            await asyncio.sleep(PAUSE_BETWEEN_WAYPOINTS_S)

    print(f"[{name}] Scenario path finished.")


async def land_and_disarm(dm: DroneManager, name: str) -> None:
    print(f"[{name}] Landing")
    await dm.land(name)

    while name in dm.drones and dm.drones[name].in_air:
        await asyncio.sleep(0.5)

    print(f"[{name}] Disarming")
    await dm.disarm(name)


# =============================================================================
# MAIN
# =============================================================================

async def main() -> None:
    if RANDOM_SEED is not None:
        random.seed(RANDOM_SEED)

    run_id = create_run_id()
    scenario = sample_scenario()

    drone_configs = build_drone_config(scenario.num_drones)
    anchors, global_paths = generate_scenario_global_paths(scenario)

    if PRINT_GENERATED_PATHS:
        print_paths(scenario, global_paths)

    if WRITE_MANIFEST:
        write_manifest(
            run_id=run_id,
            spec=scenario,
            anchors=anchors,
            global_paths=global_paths,
            drone_configs=drone_configs,
        )

    px4_processes: List[subprocess.Popen] = []

    if AUTO_START_PX4:
        px4_processes = start_px4_instances(drone_configs)

    local_paths: Dict[str, List[Vec3]] = {}
    for name, global_path in global_paths.items():
        home_offset = drone_configs[name]["home_offset_ned"]
        local_paths[name] = [global_ned_to_drone_local(global_wp, home_offset) for global_wp in global_path]

    dm = DroneManager(DroneMAVSDK, log_to_console=True)

    if LOAD_EXTERNAL_PLUGIN:
        await dm.load_plugin("external")

    try:
        if scenario.num_drones == 0:
            print(f"Empty scene. Waiting {EMPTY_SCENE_DURATION_S:.1f}s for recording.")
            await asyncio.sleep(EMPTY_SCENE_DURATION_S)
            return

        for name, cfg in drone_configs.items():
            await connect_drone(dm, name, str(cfg["connection"]))
            await asyncio.sleep(2.0)

        print("Connected drones:", list(dm.drones.keys()))
        await asyncio.sleep(2.0)

        await asyncio.gather(*[
            arm_and_takeoff_to_first_waypoint(dm=dm, name=name, first_local_wp=local_paths[name][0])
            for name in drone_configs.keys()
        ])

        print("Waiting 10s after takeoff.")
        await asyncio.sleep(10.0)

        await asyncio.gather(*[
            fly_scenario_path(
                dm=dm,
                name=name,
                local_path=local_paths[name],
                yaw=float(drone_configs[name]["yaw"]),
            )
            for name in drone_configs.keys()
        ])

        await asyncio.gather(*[
            land_and_disarm(dm, name)
            for name in drone_configs.keys()
        ])

    finally:
        print("[main] Closing DroneManager")
        await dm.close()

        if STOP_PX4_ON_EXIT:
            stop_px4_instances(px4_processes)


if __name__ == "__main__":
    asyncio.run(main())
