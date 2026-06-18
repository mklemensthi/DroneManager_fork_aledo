#In PX4, for each drone have one terminal open and increase drone index and start position:
#
# PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="0,0,0,0,0,0" ./build/px4_sitl_default/bin/px4 -i 1
# PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="0,8,0,0,0,0" ./build/px4_sitl_default/bin/px4 -i 2
# PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="0,16,0,0,0,0" ./build/px4_sitl_default/bin/px4 -i 3
# PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="0,24,0,0,0,0" ./build/px4_sitl_default/bin/px4 -i 4
# PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="0,32,0,0,0,0" ./build/px4_sitl_default/bin/px4 -i 5
# PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_x500 PX4_GZ_MODEL_POSE="0,40,0,0,0,0" ./build/px4_sitl_default/bin/px4 -i 6

import asyncio
import math
import random
import os
import subprocess
from dataclasses import dataclass
from typing import Dict, List, Tuple

from dronemanager.core import DroneManager
from dronemanager.drone import DroneMAVSDK


# ============================================================
# USER CONFIG
# ============================================================

RANDOM_SEED = 42


NUM_WAYPOINTS = 20

# LiDAR definition in global NED-like coordinates.
# NED: [north, east, down]
# z/down = -4 means the LiDAR is 4 m above the ground.
#
# Geometry model:
#   Upright 360-degree spinning LiDAR.
#   The LiDAR rotates around the vertical z/down axis.
#   Its vertical field of view is FOV_DEG.
#
# Valid points satisfy:
#   horizontal_range = sqrt(north_offset^2 + east_offset^2)
#   MIN_HORIZONTAL_DISTANCE_M <= horizontal_range <= LIDAR_RANGE_M
#   abs(down_offset) <= horizontal_range * tan(FOV_DEG / 2)
LIDAR_NED = [0.0, 0.0, -4.0]

LIDAR_RANGE_M = 4.0
FOV_DEG = 45.0

# Avoid points very close to the LiDAR vertical axis.
# Near the sensor centerline the valid vertical FOV height becomes very small.
MIN_HORIZONTAL_DISTANCE_M = 0.75

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


# ============================================================
# PX4 / DRONE STARTUP CONFIG
# ============================================================

# Choose how many drones to use.
# Valid range: 1 to 6.
NUM_ACTIVE_DRONES = 3

# If True, this script starts PX4 SITL instances automatically via WSL.
AUTO_START_PX4 = False

# If True, this script tries to terminate the PX4 processes at the end.
STOP_PX4_ON_EXIT = True

# Path inside WSL to your PX4-Autopilot folder.
PX4_AUTOPILOT_WSL_DIR = "/mnt/c/Users/mklemensthi/Documents/TONIC/PX4-Autopilot"

PX4_BINARY = "./build/px4_sitl_default/bin/px4"
PX4_SYS_AUTOSTART = "4001"
PX4_SIM_MODEL = "gz_x500"

# You have been using -i 1 ... -i 6.
PX4_FIRST_INSTANCE_INDEX = 1


# Startup timing.
PX4_START_DELAY_BETWEEN_DRONES_S = 5.0
PX4_WAIT_AFTER_ALL_STARTED_S = 20.0


def validate_num_drones(num_drones: int):
    if not 1 <= num_drones <= 6:
        raise ValueError("NUM_ACTIVE_DRONES must be between 1 and 6.")



def px4_instance_index_for_drone_number(drone_number: int) -> int:
    """
    drone1 -> PX4 -i 1
    drone2 -> PX4 -i 2
    ...
    """
    return PX4_FIRST_INSTANCE_INDEX + (drone_number - 1)


def mavsdk_port_for_px4_instance(px4_instance_index: int) -> int:
    """
    PX4 SITL convention:
        -i 1 -> udp://:14541
        -i 2 -> udp://:14542
        ...
    """
    return 14540 + px4_instance_index


def build_drone_config(num_drones: int):
    validate_num_drones(num_drones)

    drones = {}

    for drone_number in range(1, num_drones + 1):
        drone_id = f"drone{drone_number}"

        px4_idx = px4_instance_index_for_drone_number(drone_number)
        mavsdk_port = mavsdk_port_for_px4_instance(px4_idx)

        drones[drone_id] = {
            "connection": f"udp://:{mavsdk_port}",
            "home_offset_ned": [0.0, 0.0, 0.0],
            "yaw": 0.0,
            "px4_instance_index": px4_idx,
            "spawn_pose": [0.0, float(px4_idx*2), 0.0, 0.0, 0.0, 0.0],
        }

    return drones


DRONES = build_drone_config(NUM_ACTIVE_DRONES)

def build_px4_command(drone_id: str, cfg: dict) -> str:
    px4_idx = cfg["px4_instance_index"]
    x, y, z, roll, pitch, yaw = cfg["spawn_pose"]

    pose_string = f"{x},{y},{z},{roll},{pitch},{yaw}"

    command = (
        f'cd "{PX4_AUTOPILOT_WSL_DIR}" && '
        f'PX4_SYS_AUTOSTART={PX4_SYS_AUTOSTART} '
        f'PX4_SIM_MODEL={PX4_SIM_MODEL} '
        f'PX4_GZ_MODEL_POSE="{pose_string}" '
        f'{PX4_BINARY} -i {px4_idx}'
    )

    return command


def start_px4_instances():
    """
    Starts PX4 SITL instances.

    If this Python script runs on Windows, it calls:
        wsl.exe bash -lc "<px4 command>"

    If this Python script runs inside WSL/Linux, it calls:
        bash -lc "<px4 command>"
    """
    if not AUTO_START_PX4:
        return []

    processes = []

    print("\nStarting PX4 SITL instances...")
    print("=" * 80)

    for drone_id, cfg in DRONES.items():
        command = build_px4_command(drone_id, cfg)

        print(f"[{drone_id}] PX4 command:")
        print(command)
        print()

        if os.name == "nt":
            # Running Python on Windows, launch inside WSL.
            proc = subprocess.Popen(
                ["wsl.exe", "bash", "-lc", command],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
        else:
            # Running Python already inside WSL/Linux.
            proc = subprocess.Popen(
                ["bash", "-lc", command],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

        processes.append(proc)

        # Starting all PX4 instances at exactly the same time can be unstable.
        # A small delay helps Gazebo/PX4 initialize cleanly.
        if PX4_START_DELAY_BETWEEN_DRONES_S > 0:
            import time
            time.sleep(PX4_START_DELAY_BETWEEN_DRONES_S)
        
        #if this is the first iteration add another sleep
        if drone_id == "drone1" and PX4_START_DELAY_BETWEEN_DRONES_S > 0:
            print(f"Waiting {PX4_WAIT_AFTER_ALL_STARTED_S} seconds for PX4/Gazebo startup...\n")
            time.sleep(PX4_WAIT_AFTER_ALL_STARTED_S)

    print(f"Started {len(processes)} PX4 process(es).")
    

    return processes


def stop_px4_instances(processes):
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



def max_vertical_offset_for_horizontal_range(horizontal_range: float) -> float:
    """
    Maximum allowed vertical/down offset for a given horizontal range.

    The LiDAR is modeled as an upright 360-degree spinning sensor:
        horizontal_range = sqrt(north_offset^2 + east_offset^2)

    The vertical field of view creates an upper and lower cone around
    the horizontal scan plane:
        abs(down_offset) <= horizontal_range * tan(FOV_DEG / 2)
    """
    return horizontal_range * math.tan(math.radians(FOV_DEG / 2.0))


def is_inside_lidar_bicone(point_ned: List[float]) -> bool:
    """
    Backward-compatible name, but the geometry is no longer a sideways LiDAR volume.

    Checks whether point_ned lies inside an upright 360-degree LiDAR volume.

    LiDAR-relative coordinates:
        north_offset = rel[0]
        east_offset  = rel[1]
        down_offset  = rel[2]

    The LiDAR rotates around the vertical down/z axis. Therefore, the
    horizontal range is computed in the north/east plane and the FOV limits
    the allowed vertical offset.

    Valid region:
        MIN_HORIZONTAL_DISTANCE_M <= sqrt(north^2 + east^2) <= LIDAR_RANGE_M
        abs(down) <= sqrt(north^2 + east^2) * tan(FOV_DEG / 2)
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

    max_vertical_offset = max_vertical_offset_for_horizontal_range(horizontal_range)

    return abs(down) <= max_vertical_offset


def sample_random_point_in_lidar_bicone() -> List[float]:
    """
    Backward-compatible name.

    Samples one random point inside the upright 360-degree LiDAR volume.

    The point is sampled by:
        1. choosing a horizontal range in the north/east plane,
        2. choosing a random bearing around the LiDAR,
        3. choosing a vertical/down offset inside the vertical FOV at that range.
    """
    for _ in range(10000):
        # Area-uniform sampling in the horizontal annulus.
        horizontal_range = math.sqrt(
            random.uniform(
                MIN_HORIZONTAL_DISTANCE_M * MIN_HORIZONTAL_DISTANCE_M,
                LIDAR_RANGE_M * LIDAR_RANGE_M,
            )
        )

        bearing = random.uniform(0.0, 2.0 * math.pi)

        north = horizontal_range * math.cos(bearing)
        east = horizontal_range * math.sin(bearing)

        max_vertical_offset = max_vertical_offset_for_horizontal_range(horizontal_range)
        down = random.uniform(-max_vertical_offset, max_vertical_offset)

        point = [
            LIDAR_NED[0] + north,
            LIDAR_NED[1] + east,
            LIDAR_NED[2] + down,
        ]

        if is_inside_lidar_bicone(point):
            return point

    raise RuntimeError("Could not sample a valid point inside the upright LiDAR volume.")


def sample_next_point_near_previous(previous_point: List[float]) -> List[float]:
    """
    Samples a new point inside the upright LiDAR volume at a local step distance
    from the previous point.

    This preserves the original "random local trajectory" behavior, but uses the
    corrected upright LiDAR validity check.
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

    # Fallback: if local rejection fails, sample anywhere in the LiDAR volume.
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
    - first point for every drone is widely random inside the upright LiDAR volume
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



def lidar_relative_metadata(point_ned: List[float]) -> dict:
    rel = vec_sub(point_ned, LIDAR_NED)
    north = rel[0]
    east = rel[1]
    down = rel[2]

    horizontal_range = math.sqrt(north * north + east * east)
    max_vertical_offset = max(max_vertical_offset_for_horizontal_range(horizontal_range), 1e-9)
    bearing_rad = math.atan2(east, north)
    bearing_deg = math.degrees(bearing_rad)

    return {
        "north": north,
        "east": east,
        "down": down,
        "horizontal_range": horizontal_range,
        "range_3d": math.sqrt(horizontal_range * horizontal_range + down * down),
        "bearing_deg": bearing_deg,
        "vertical_fraction": down / max_vertical_offset,
        "distance_to_vertical_fov_edge_fraction": max(0.0, 1.0 - abs(down) / max_vertical_offset),
    }


def print_paths(global_paths: Dict[str, List[List[float]]]):
    print("\nGenerated global NED waypoints:")
    print("=" * 80)
    print("Geometry: upright 360-degree LiDAR")
    print("Valid region: horizontal_range <= LIDAR_RANGE_M and abs(down_offset) <= horizontal_range * tan(FOV/2)")
    print("=" * 80)

    for drone_id, path in global_paths.items():
        print(f"\n{drone_id}:")
        for idx, p in enumerate(path):
            rel_to_lidar = vec_sub(p, LIDAR_NED)
            meta = lidar_relative_metadata(p)
            print(
                f"  WP {idx + 1}: "
                f"global_ned=[{p[0]:7.2f}, {p[1]:7.2f}, {p[2]:7.2f}], "
                f"rel_to_lidar=[{rel_to_lidar[0]:7.2f}, {rel_to_lidar[1]:7.2f}, {rel_to_lidar[2]:7.2f}], "
                f"h_range={meta['horizontal_range']:6.2f}, "
                f"bearing={meta['bearing_deg']:7.1f} deg, "
                f"vertical_frac={meta['vertical_fraction']:6.2f}"
            )

# ============================================================
# DRONEMANAGER HELPERS
# ============================================================

async def connect_drone(dm, drone_id: str, connection: str):
    print(f"[{drone_id}] Connecting on {connection}")

    await dm.connect_to_drone(
        name=drone_id,
        drone_address=connection,
        timeout=500,
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

    px4_processes = []

    if AUTO_START_PX4:
        px4_processes = start_px4_instances()

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
        for drone_id, cfg in DRONES.items():
            await connect_drone(dm, drone_id, cfg["connection"])
            await asyncio.sleep(2.0)

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

        if STOP_PX4_ON_EXIT:
            stop_px4_instances(px4_processes)


if __name__ == "__main__":
    asyncio.run(main())