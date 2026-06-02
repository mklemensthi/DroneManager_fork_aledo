#In PX4, for each drone have one terminal open and increase drone index and start position:
#
# PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="0,2,0,0,0,0" ./build/px4_sitl_default/bin/px4 -i 1
# PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="0,4,0,0,0,0" ./build/px4_sitl_default/bin/px4 -i 2
# PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="0,6,0,0,0,0" ./build/px4_sitl_default/bin/px4 -i 3
# PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="0,8,0,0,0,0" ./build/px4_sitl_default/bin/px4 -i 4
# PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="0,10,0,0,0,0" ./build/px4_sitl_default/bin/px4 -i 5
# PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="0,12,0,0,0,0" ./build/px4_sitl_default/bin/px4 -i 6

import asyncio
import math
import random
from dataclasses import dataclass
from typing import Dict, List, Tuple

from dronemanager.core import DroneManager
from dronemanager.drone import DroneMAVSDK


# ============================================================
# USER CONFIG
# ============================================================

RANDOM_SEED = 42

NUM_WAYPOINTS = 40

# LiDAR / funnel definition in global NED-like coordinates.
# NED: [north, east, down]
# 150 m height above ground means down = -150.
LIDAR_NED = [0.0, 0.0, -4.0]

LIDAR_RANGE_M = 4.0
FOV_DEG = 45.0

# Avoid points too close to the cone apex, because the valid cross-section
# becomes tiny there.
MIN_AXIS_DISTANCE_M = 0.75

# Consecutive waypoint step length.
STEP_DISTANCE_MIN_M = 0.5
STEP_DISTANCE_MAX_M = 1.0

# Keep drones separated at each waypoint index.
MIN_INTER_DRONE_DISTANCE_M = 0.75

# Safety timeout for each fly_to command.
FLY_TO_TIMEOUT_S = 20.0

# Optional pause after each synchronized waypoint.
PAUSE_BETWEEN_WAYPOINTS_S = 1.0

# Whether to load DroneManager's external plugin for Unity visualization.
# Set to False if you only want Gazebo/PX4 and no Unity stream.
LOAD_EXTERNAL_PLUGIN = True


# Important:
# home_offset_ned should match the PX4_GZ_MODEL_POSE spawn position.
#
# Example:
# PX4_GZ_MODEL_POSE="0,2,0,0,0,0" means:
# home_offset_ned = [0, 2, 0]
#
# If all drones are spawned at the same Gazebo origin, use [0, 0, 0],
# but for multi-drone simulation you should spawn them separated.
DRONES = {
    "drone1": {
        "connection": "udp://:14541",
        "home_offset_ned": [0.0, 0.0, 0.0],
        "yaw": 0.0,
    },
    "drone2": {
        "connection": "udp://:14542",
        "home_offset_ned": [0.0, 0.0, 0.0],
        "yaw": 0.0,
    },
    "drone3": {
        "connection": "udp://:14543",
        "home_offset_ned": [0.0, 0.0, 0.0],
        "yaw": 0.0,
    },
    "drone4": {
        "connection": "udp://:14544",
        "home_offset_ned": [0.0, 0.0, 0.0],
        "yaw": 0.0,
    },
    "drone5": {
        "connection": "udp://:14545",
        "home_offset_ned": [0.0, 0.0, 0.0],
        "yaw": 0.0,
    },
    "drone6": {
        "connection": "udp://:14546",
        "home_offset_ned": [0.0, 0.0, 0.0],
        "yaw": 0.0,
    },
}


# ============================================================
# GEOMETRY HELPERS
# ============================================================

def vec_sub(a, b):
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def vec_add(a, b):
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def vec_norm(v):
    return math.sqrt(v[0] ** 2 + v[1] ** 2 + v[2] ** 2)


def distance(a, b):
    return vec_norm(vec_sub(a, b))


def random_unit_vector_3d():
    """
    Uniform random direction on a sphere.
    """
    z = random.uniform(-1.0, 1.0)
    theta = random.uniform(0.0, 2.0 * math.pi)
    r = math.sqrt(max(0.0, 1.0 - z * z))
    return [r * math.cos(theta), r * math.sin(theta), z]


def is_inside_lidar_bicone(point_ned: List[float]) -> bool:
    """
    Checks whether point_ned lies inside a double cone / bicone.

    Axis is the north direction through the LiDAR.
    Perpendicular plane is east/down.

    LiDAR-relative coordinates:
        axis distance = north offset
        radial distance = sqrt(east_offset^2 + down_offset^2)

    Full FOV is FOV_DEG, so half-angle is FOV_DEG / 2.
    """
    rel = vec_sub(point_ned, LIDAR_NED)

    axis = rel[0]  # north axis
    east = rel[1]
    down = rel[2]

    abs_axis = abs(axis)

    if abs_axis < MIN_AXIS_DISTANCE_M:
        return False

    if abs_axis > LIDAR_RANGE_M:
        return False

    half_angle_rad = math.radians(FOV_DEG / 2.0)
    max_radius = abs_axis * math.tan(half_angle_rad)

    radial = math.sqrt(east * east + down * down)

    return radial <= max_radius


def sample_random_point_in_lidar_bicone() -> List[float]:
    """
    Samples one random point inside the LiDAR bicone.

    The sampling is not mathematically perfect uniform volume sampling,
    but it is good enough for trajectory generation and gives broad coverage.
    """
    half_angle_rad = math.radians(FOV_DEG / 2.0)

    while True:
        sign = random.choice([-1.0, 1.0])
        axis = sign * random.uniform(MIN_AXIS_DISTANCE_M, LIDAR_RANGE_M)

        max_radius = abs(axis) * math.tan(half_angle_rad)

        # sqrt(random) gives a more uniform disk area distribution.
        radial = math.sqrt(random.random()) * max_radius
        angle = random.uniform(0.0, 2.0 * math.pi)

        east = radial * math.cos(angle)
        down = radial * math.sin(angle)

        point = [
            LIDAR_NED[0] + axis,
            LIDAR_NED[1] + east,
            LIDAR_NED[2] + down,
        ]

        if is_inside_lidar_bicone(point):
            return point


def sample_next_point_near_previous(previous_point: List[float]) -> List[float]:
    """
    Samples a new point inside the bicone at a distance of 5-15 m
    from the previous point.
    """
    for _ in range(1000):
        step_length = random.uniform(STEP_DISTANCE_MIN_M, STEP_DISTANCE_MAX_M)
        direction = random_unit_vector_3d()

        candidate = [
            previous_point[0] + step_length * direction[0],
            previous_point[1] + step_length * direction[1],
            previous_point[2] + step_length * direction[2],
        ]

        if is_inside_lidar_bicone(candidate):
            return candidate

    # Fallback: if local rejection fails, sample anywhere in the bicone.
    return sample_random_point_in_lidar_bicone()


def global_ned_to_drone_local(global_point_ned: List[float], home_offset_ned: List[float]) -> List[float]:
    """
    Converts a global/simulation NED point into the local point used by
    a specific PX4 drone.

    If PX4_GZ_MODEL_POSE="0,4,0,0,0,0", then home_offset_ned=[0,4,0].
    """
    return [
        global_point_ned[0] - home_offset_ned[0],
        global_point_ned[1] - home_offset_ned[1],
        global_point_ned[2] - home_offset_ned[2],
    ]


def altitude_from_local_ned(local_point: List[float]) -> float:
    """
    DroneManager takeoff altitude is positive up.
    Local NED down is negative for altitude above ground.
    """
    return max(1.0, -local_point[2])


# ============================================================
# PATH GENERATION
# ============================================================

def generate_multi_drone_global_paths() -> Dict[str, List[List[float]]]:
    """
    Generates NUM_WAYPOINTS global NED waypoints for each drone.

    Properties:
    - first point for every drone is widely random inside the bicone
    - consecutive points are 5-15 m apart
    - drones are separated from each other at every waypoint index
    """
    drone_ids = list(DRONES.keys())
    paths = {drone_id: [] for drone_id in drone_ids}

    # Generate first waypoint with inter-drone separation.
    for drone_id in drone_ids:
        for _ in range(1000):
            candidate = sample_random_point_in_lidar_bicone()

            too_close = False
            for other_id in drone_ids:
                if len(paths[other_id]) > 0:
                    if distance(candidate, paths[other_id][0]) < MIN_INTER_DRONE_DISTANCE_M:
                        too_close = True
                        break

            if not too_close:
                paths[drone_id].append(candidate)
                break
        else:
            raise RuntimeError(f"Could not generate separated start point for {drone_id}")

    # Generate subsequent waypoints.
    for waypoint_idx in range(1, NUM_WAYPOINTS):
        for drone_id in drone_ids:
            previous = paths[drone_id][-1]

            for _ in range(1000):
                candidate = sample_next_point_near_previous(previous)

                too_close = False
                for other_id in drone_ids:
                    if len(paths[other_id]) > waypoint_idx:
                        if distance(candidate, paths[other_id][waypoint_idx]) < MIN_INTER_DRONE_DISTANCE_M:
                            too_close = True
                            break

                if not too_close:
                    paths[drone_id].append(candidate)
                    break
            else:
                raise RuntimeError(
                    f"Could not generate waypoint {waypoint_idx} for {drone_id}"
                )

    return paths


def print_paths(global_paths: Dict[str, List[List[float]]]):
    print("\nGenerated global NED waypoints:")
    print("=" * 80)

    for drone_id, path in global_paths.items():
        print(f"\n{drone_id}:")
        for idx, p in enumerate(path):
            rel_to_lidar = vec_sub(p, LIDAR_NED)
            print(
                f"  WP {idx + 1}: "
                f"global_ned=[{p[0]:7.2f}, {p[1]:7.2f}, {p[2]:7.2f}], "
                f"rel_to_lidar=[{rel_to_lidar[0]:7.2f}, {rel_to_lidar[1]:7.2f}, {rel_to_lidar[2]:7.2f}]"
            )


# ============================================================
# DRONEMANAGER HELPERS
# ============================================================

async def connect_drone(dm, drone_id: str, connection: str):
    print(f"[{drone_id}] Connecting on {connection}")

    await dm.connect_to_drone(
        name=drone_id,
        drone_address=connection,
        timeout=30,
    )

    if drone_id not in dm.drones:
        raise RuntimeError(f"[{drone_id}] Failed to connect on {connection}")

    print(f"[{drone_id}] Connected successfully")


async def arm_and_takeoff_to_first_waypoint(dm, drone_id: str, first_local_wp: List[float]):
    takeoff_altitude = altitude_from_local_ned(first_local_wp)

    print(f"[{drone_id}] Arming")
    await dm.arm(drone_id)

    await asyncio.sleep(2.0)

    print(f"[{drone_id}] Taking off to {takeoff_altitude:.2f} m")
    await dm.takeoff(drone_id, altitude=takeoff_altitude)

    # Give PX4 time to stabilize.
    await asyncio.sleep(8.0)


async def safe_fly_to(dm, drone_id: str, local_wp: List[float], yaw: float, timeout_s: float):
    try:
        print(
            f"[{drone_id}] fly_to local=[{local_wp[0]:.2f}, {local_wp[1]:.2f}, {local_wp[2]:.2f}], "
            f"yaw={yaw:.1f}"
        )

        await asyncio.wait_for(
            dm.fly_to(drone_id, local=local_wp, yaw=yaw),
            timeout=timeout_s,
        )

        return True

    except asyncio.TimeoutError:
        print(f"[{drone_id}] WARNING: fly_to timeout. Continuing to next waypoint.")
        return False


async def fly_random_path(
    dm,
    drone_id: str,
    local_path: List[List[float]],
    yaw: float,
):
    print(f"[{drone_id}] Starting random path with {len(local_path)} waypoints.")

    for idx, local_wp in enumerate(local_path):
        print(f"[{drone_id}] Waypoint {idx + 1}/{len(local_path)}")

        ok = await safe_fly_to(
            dm=dm,
            drone_id=drone_id,
            local_wp=local_wp,
            yaw=yaw,
            timeout_s=FLY_TO_TIMEOUT_S,
        )

        if not ok:
            # For data generation, I would continue instead of aborting.
            # If you want stricter behavior, replace this with `break`.
            pass

        await asyncio.sleep(PAUSE_BETWEEN_WAYPOINTS_S)

    print(f"[{drone_id}] Random path finished.")


async def land_and_disarm(dm, drone_id: str):
    print(f"[{drone_id}] Landing")
    await dm.land(drone_id)

    while drone_id in dm.drones and dm.drones[drone_id].in_air:
        await asyncio.sleep(0.5)

    print(f"[{drone_id}] Disarming")
    await dm.disarm(drone_id)


# ============================================================
# MAIN
# ============================================================

async def main():
    random.seed(RANDOM_SEED)

    # 1. Generate paths in global NED space.
    global_paths = generate_multi_drone_global_paths()
    print_paths(global_paths)

    # 2. Convert global paths to each drone's local PX4 frame.
    local_paths = {}

    for drone_id, global_path in global_paths.items():
        home_offset = DRONES[drone_id]["home_offset_ned"]

        local_paths[drone_id] = [
            global_ned_to_drone_local(global_wp, home_offset)
            for global_wp in global_path
        ]

    dm = DroneManager(DroneMAVSDK, log_to_console=True)

    if LOAD_EXTERNAL_PLUGIN:
        await dm.load_plugin("external")

    try:
        # 3. Connect all drones.
        await asyncio.gather(*[
            connect_drone(dm, drone_id, cfg["connection"])
            for drone_id, cfg in DRONES.items()
        ])

        print("Connected drones:", list(dm.drones.keys()))

        await asyncio.sleep(2.0)

        # 4. Arm and take off to altitude near first waypoint.
        await asyncio.gather(*[
            arm_and_takeoff_to_first_waypoint(
                dm=dm,
                drone_id=drone_id,
                first_local_wp=local_paths[drone_id][0],
            )
            for drone_id in DRONES.keys()
        ])

        print("Waiting 10s after takeoff.")
        await asyncio.sleep(10.0)

        # 5. Fly all random paths concurrently.
        await asyncio.gather(*[
            fly_random_path(
                dm=dm,
                drone_id=drone_id,
                local_path=local_paths[drone_id],
                yaw=DRONES[drone_id]["yaw"],
            )
            for drone_id in DRONES.keys()
        ])

        # 6. Land all drones.
        await asyncio.gather(*[
            land_and_disarm(dm, drone_id)
            for drone_id in DRONES.keys()
        ])

    finally:
        print("[main] Closing DroneManager")
        await dm.close()


if __name__ == "__main__":
    asyncio.run(main())