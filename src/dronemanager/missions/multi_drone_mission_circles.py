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

from dronemanager.core import DroneManager
from dronemanager.drone import DroneMAVSDK


DRONES = {
    "drone1": {
        "connection": "udp://:14541",  # PX4 -i 1
        "radius": 3.9,
        "height": 4.7,
        "num_waypoints": 14,
        "yaw": 0.0,
    },
    "drone2": {
        "connection": "udp://:14542",  # PX4 -i 2
        "radius": 3.0,
        "height": 3.2,
        "num_waypoints": 14,
        "yaw": 0.0,
    },
    "drone3": {
        "connection": "udp://:14543",  # PX4 -i 3
        "radius": 2.5,
        "height": 4.1,
        "num_waypoints": 14,
        "yaw": 0.0,
    },
        "drone4": {
        "connection": "udp://:14544",  # PX4 -i 4
        "radius": 3.5,
        "height": 4.5,
        "num_waypoints": 14,
        "yaw": 0.0,
    },
        "drone5": {
        "connection": "udp://:14545",  # PX4 -i 5
        "radius": 1.8,
        "height": 3.9,
        "num_waypoints": 14,
        "yaw": 0.0,
    },
        "drone6": {
        "connection": "udp://:14546",  # PX4 -i 6
        "radius": 3.5,
        "height": 4.1,
        "num_waypoints": 14,
        "yaw": 0.0,
    },
}


def generate_circle_waypoints(
    radius: float,
    height: float,
    num_waypoints: int,
    center_north: float = 0.0,
    center_east: float = 0.0,
    close_loop: bool = True,
):
    """
    Generate circular waypoints in PX4/DroneManager local NED coordinates.

    local = [north, east, down]
    height above ground -> down = -height
    """
    if num_waypoints < 3:
        raise ValueError("num_waypoints must be at least 3.")

    waypoints = []

    for i in range(num_waypoints):
        angle = 2.0 * math.pi * i / num_waypoints

        north = center_north + radius * math.cos(angle)
        east = center_east + radius * math.sin(angle)
        down = -height

        waypoints.append([north, east, down])

    if close_loop:
        waypoints.append(waypoints[0])

    return waypoints


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


async def arm_and_takeoff(dm, drone_id: str, height: float):
    print(f"[{drone_id}] Arming")
    await dm.arm(drone_id)

    await asyncio.sleep(4.0)

    print(f"[{drone_id}] Taking off to {height} m")
    await dm.takeoff(drone_id, altitude=height)

    # Give PX4 time to reach/stabilize near takeoff height.
    await asyncio.sleep(5.0)


async def fly_circle(dm, drone_id: str, radius: float, height: float, num_waypoints: int, yaw: float = 0.0):
    waypoints = generate_circle_waypoints(
        radius=radius,
        height=height,
        num_waypoints=num_waypoints,
        close_loop=True,
    )

    print(
        f"[{drone_id}] Flying circle: "
        f"radius={radius}, height={height}, waypoints={num_waypoints}"
    )

    for idx, wp in enumerate(waypoints):
        print(f"[{drone_id}] Waypoint {idx + 1}/{len(waypoints)}: {wp}")
        await dm.fly_to(drone_id, local=wp, yaw=yaw)

        # Small pause makes the path less aggressive and easier to debug.
        await asyncio.sleep(0.2)

    print(f"[{drone_id}] Circle finished")


async def land_and_disarm(dm, drone_id: str):
    print(f"[{drone_id}] Landing")
    await dm.land(drone_id)

    while dm.drones[drone_id].in_air:
        await asyncio.sleep(0.5)

    print(f"[{drone_id}] Disarming")
    await dm.disarm(drone_id)


async def main():
    dm = DroneManager(DroneMAVSDK, log_to_console=True)
    await dm.load_plugin("external")

    try:
        # 1. Connect both drones.
        await asyncio.gather(*[
            connect_drone(dm, drone_id, cfg["connection"])
            for drone_id, cfg in DRONES.items()
        ])
        
        await asyncio.sleep(2.0)

        # 2. Arm and take off both drones.
        await asyncio.gather(*[
            arm_and_takeoff(dm, drone_id, cfg["height"])
            for drone_id, cfg in DRONES.items()
        ])
        print("Waiting 10s after takeoff!")
        await asyncio.sleep(10.0)

        # 3. Fly circles concurrently.
        await asyncio.gather(*[
            fly_circle(
                dm=dm,
                drone_id=drone_id,
                radius=cfg["radius"],
                height=cfg["height"],
                num_waypoints=cfg["num_waypoints"],
                yaw=cfg["yaw"],
            )
            for drone_id, cfg in DRONES.items()
        ])

        # 4. Land both drones.
        await asyncio.gather(*[
            land_and_disarm(dm, drone_id)
            for drone_id in DRONES.keys()
        ])

    finally:
        print("[main] Closing DroneManager")
        await dm.close()


if __name__ == "__main__":
    asyncio.run(main())