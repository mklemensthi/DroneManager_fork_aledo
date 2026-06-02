import asyncio
import socket
import subprocess
from pathlib import Path

from dronemanager.core import DroneManager
from dronemanager.drone import DroneMAVSDK


UNITY_EXE = Path(
    r"C:\Users\mklemensthi\Documents\TONIC\AL3DO\Unity_envs\Env_Builds\LiDARTest.exe"
)

UNITY_CONTROL_HOST = "127.0.0.1"
UNITY_CONTROL_PORT = 50050
UNITY_READY_PORT = 50051


def send_unity_command(command, host=UNITY_CONTROL_HOST, port=UNITY_CONTROL_PORT):
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.sendto(command.encode("utf-8"), (host, port))
    print(f"[Python] Sent Unity command: {command}")


async def wait_for_unity_message(expected_message, port=50051, timeout=60):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    sock.settimeout(1.0)

    print(f"[Python] Waiting for {expected_message}...")

    elapsed = 0
    try:
        while elapsed < timeout:
            try:
                data, addr = sock.recvfrom(1024)
                msg = data.decode("utf-8").strip()
                print(f"[Python] Received from Unity: {msg} from {addr}")

                if msg == expected_message:
                    print(f"[Python] {expected_message} received.")
                    return
            except socket.timeout:
                await asyncio.sleep(1)
                elapsed += 1

        raise TimeoutError(f"Unity did not send {expected_message} in time.")
    finally:
        sock.close()

def launch_unity():
    if not UNITY_EXE.exists():
        raise FileNotFoundError(f"Unity executable not found: {UNITY_EXE}")

    print(f"[Python] Launching Unity: {UNITY_EXE}")

    return subprocess.Popen(
        [str(UNITY_EXE)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )

import math
import asyncio


def generate_circle_waypoints(
    radius: float,
    height: float,
    num_waypoints: int,
    center_north: float = 0.0,
    center_east: float = 0.0,
    close_loop: bool = True,
):
    """
    Generate circular waypoints in DroneManager/PX4 local NED coordinates.

    Args:
        radius: Circle radius in meters.
        height: Flight height above takeoff/ground in meters.
                Converted to NED down = -height.
        num_waypoints: Number of waypoints around the circle.
        center_north: Circle center north coordinate.
        center_east: Circle center east coordinate.
        close_loop: If True, appends the first waypoint again at the end.

    Returns:
        List of local NED waypoints: [[north, east, down], ...]
    """
    if num_waypoints < 3:
        raise ValueError("num_waypoints must be at least 3 for a circle.")

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


def yaw_towards_center(north: float, east: float, center_north: float = 0.0, center_east: float = 0.0):
    """
    Compute an approximate yaw angle so the drone faces the circle center.

    Note:
        Depending on DroneManager/PX4 yaw convention, you may need to flip signs
        or add 90/180 degrees. Start with constant yaw=0 first, then test this.
    """
    delta_north = center_north - north
    delta_east = center_east - east

    return math.degrees(math.atan2(delta_east, delta_north))


async def fly_circle(
    dm,
    drone_id: str,
    radius: float,
    height: float,
    num_waypoints: int,
    center_north: float = 0.0,
    center_east: float = 0.0,
    yaw_mode: str = "constant",
    constant_yaw: float = 0.0,
    pause_at_waypoint: float = 0.2,
):
    """
    Fly a circular trajectory using DroneManager fly_to commands.

    yaw_mode:
        "constant"      -> always use constant_yaw
        "face_center"   -> yaw points approximately toward circle center
        "tangent"       -> yaw follows direction of travel approximately
    """
    waypoints = generate_circle_waypoints(
        radius=radius,
        height=height,
        num_waypoints=num_waypoints,
        center_north=center_north,
        center_east=center_east,
        close_loop=True,
    )

    for idx, wp in enumerate(waypoints):
        north, east, down = wp
        #Start recording from the second waypoint to avoid start data at center.
        if idx == 1:
            send_unity_command("START_RECORDING")
        if yaw_mode == "constant":
            yaw = constant_yaw

        elif yaw_mode == "face_center":
            yaw = yaw_towards_center(
                north=north,
                east=east,
                center_north=center_north,
                center_east=center_east,
            )

        elif yaw_mode == "tangent":
            # Tangent direction for counter-clockwise motion.
            # Again, verify convention in your setup.
            yaw = yaw_towards_center(
                north=north,
                east=east,
                center_north=center_north,
                center_east=center_east,
            ) + 90.0

        else:
            raise ValueError(f"Unknown yaw_mode: {yaw_mode}")

        print(
            f"Waypoint {idx + 1}/{len(waypoints)}: "
            f"local=[{north:.2f}, {east:.2f}, {down:.2f}], yaw={yaw:.1f}"
        )

        await dm.fly_to(drone_id, local=[north, east, down], yaw=yaw)

        if pause_at_waypoint > 0:
            await asyncio.sleep(pause_at_waypoint)


async def main():
    unity_process = launch_unity()

    try:
        await wait_for_unity_message("UNITY_READY")

        dm = DroneManager(DroneMAVSDK, log_to_console=True)
        await dm.load_plugin("external")
        await dm.connect_to_drone("tom")

        # Wait until Unity actually spawned tom from DroneManager's external stream.
        await wait_for_unity_message("UNITY_DRONE_READY")

        await dm.arm("tom")
        await asyncio.sleep(2)
        await dm.takeoff("tom", altitude=1.0)

        await asyncio.sleep(3)

        #send_unity_command("START_RECORDING")


        # Small buffer so the first frames include stable hover.
        await asyncio.sleep(1)
        await fly_circle(
            dm=dm,
            drone_id="tom",
            radius=2.0,
            height=1.0,
            num_waypoints=16,
            center_north=0.0,
            center_east=0.0,
            yaw_mode="constant",
            constant_yaw=0.0,
            pause_at_waypoint=0.0,
        )
        await fly_circle(
            dm=dm,
            drone_id="tom",
            radius=2.5,
            height=1.0,
            num_waypoints=16,
            center_north=0.0,
            center_east=0.0,
            yaw_mode="face_center",
            constant_yaw=0.0,
            pause_at_waypoint=0.0,
        )
        await fly_circle(
            dm=dm,
            drone_id="tom",
            radius=3.5,
            height=1.0,
            num_waypoints=16,
            center_north=0.0,
            center_east=0.0,
            yaw_mode="tangent",
            constant_yaw=0.0,
            pause_at_waypoint=0.0,
        )

        # await dm.fly_to("tom", local=[0, -2, -1], yaw=0)
        # await dm.fly_to("tom", local=[-2, -2, -1], yaw=0)
        # await dm.fly_to("tom", local=[-2, 2, -1], yaw=0)
        # await dm.fly_to("tom", local=[2, 2, -1], yaw=0)
        # await dm.fly_to("tom", local=[2, -2, -1], yaw=0)
        # await dm.fly_to("tom", local=[0, -2, -1], yaw=0)

        send_unity_command("STOP_RECORDING")

        await asyncio.sleep(1)

        await dm.land("tom")

        while dm.drones["tom"].in_air:
            await asyncio.sleep(0.5)

        await dm.disarm("tom")
        await dm.close()
        await asyncio.sleep(3)

        send_unity_command("QUIT")

    except Exception as e:
        print(f"[Python] Error: {e}")

        # Try to finalize recording and close Unity even on failure.
        try:
            send_unity_command("STOP_RECORDING")
            await asyncio.sleep(0.5)
            send_unity_command("QUIT")
        except Exception:
            pass

        raise

    finally:
        # If Unity did not quit from command, terminate it.
        await asyncio.sleep(2)

        if unity_process.poll() is None:
            print("[Python] Terminating Unity process.")
            unity_process.terminate()


asyncio.run(main())