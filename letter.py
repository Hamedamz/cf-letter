"""Single CF: takeoff, follow absolute-coords waypoints, land."""

import json
import os
import sys
import time
from datetime import datetime

import numpy as np
from pycrazyswarm import Crazyswarm


def create_trajectory_from_file(file_path, takeoff_altitude):
    waypoints = []
    with open(file_path, "r") as f:
        trajectory = json.load(f)

    fps = trajectory["fps"]
    start_position = trajectory["start_position"]
    segments = trajectory["segments"]

    #  go to start position
    x0, y0, z0 = 0, 0, takeoff_altitude
    y, x, z = start_position
    z = takeoff_altitude + z
    dx, dy, dz = x - x0, y - y0, z - z0

    # move to start point in 3 seconds
    for i in range(1, 3 * fps + 1):
        waypoints.append([x0 + i * dx / 3 * fps, y0 + i * dy / 3 * fps, z0 + i * dz / 3 * fps])

    # stay at start point for 1 second
    for i in range(fps):
        waypoints.append([x, y, z])

    for i in range(3):
        for segment in segments:
            positions = segment["position"]
            velocities = segment["velocity"]
            state = segment["state"]
            if state == "RETURN" and i == 2:
                break

            for p, v in zip(positions, velocities):
                y, x, z = p
                vy, vx, vz = v
                # self.send_position_target(x, y, -self.takeoff_altitude-z)
                # self.send_position_velocity_target(x, y, -self.takeoff_altitude - z, vx, vy, -vz)
                waypoints.append([x, y, takeoff_altitude + z])

    #  go to start position
    for _ in range(fps):
        waypoints.append([0, 0, takeoff_altitude])

    return np.array(waypoints), fps


def main():
    args = sys.argv[1:]
    Z = 0.6
    TAKEOFF_DURATION = 2.5
    WAYPOINTS, fps = create_trajectory_from_file(args[0], Z)

    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cf = swarm.allcfs.crazyflies[0]

    cf.takeoff(targetHeight=Z, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)

    trajectory_log = []
    for i, p in enumerate(WAYPOINTS):
        # cf.goTo(p, yaw=0.0, duration=GOTO_DURATION)

        cf.cmdPosition(p, yaw=0)

        # Log the current position from Vicon
        pos = cf.position()
        trajectory_log.append(pos)

        trajectory_log.append({
            "frame_id": i,
            "tvec": pos.tolist(),
            "time": time.time() * 1000
        })

        timeHelper.sleepForRate(fps)

    cf.land(targetHeight=0.05, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)

    now = datetime.now()
    formatted = now.strftime("%H_%M_%S_%m_%d_%Y")
    file_path = os.path.join("logs", f"vicon_{formatted}.json")
    with open(file_path, "w") as f:
        json.dump({"frames": trajectory_log}, f)
    print(f"Vicon log saved in {file_path}")


if __name__ == "__main__":
    main()
