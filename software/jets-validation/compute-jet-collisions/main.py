# Import libraries
import numpy as np
import resolve_robotics_uri_py
import matplotlib.pyplot as plt
from pathlib import Path
from scipy.io import loadmat

# Import custom classes
from src.robot import Robot


def main():
    # Get root path
    root = Path(__file__).parent
    # Initialize robot object
    robot_name = "iRonCub-Mk1"
    urdf_path = str(
        resolve_robotics_uri_py.resolve_robotics_uri("package://iRonCub-Mk1/model.urdf")
    )
    robot = Robot(robot_name, urdf_path)

    # Load simulation data from mat file
    file = root / "mat" / "12-40_joint-positions.mat"
    data = loadmat(file)
    timestamps = data["time"].flatten()
    joint_pos = data["jointPos"] * np.pi / 180

    pitch = 90
    yaw = 0

    # Initialize collisions dict
    robot.set_state(pitch, yaw, joint_pos[0, :])
    colls = robot.compute_collision_volume()
    collisions = {key: [] for key in colls.keys()}
    for i in range(len(timestamps)):
        s = joint_pos[i, :]
        robot.set_state(pitch, yaw, s)
        coll_vol = robot.compute_collision_volume()
        for key in colls.keys():
            collisions[key].append(coll_vol[key])
        print(f"Processing {i+1}/{len(timestamps)}", end="\r", flush=True)

    # Plot collisions data
    plt.figure(figsize=(10, 6))
    for key in collisions.keys():
        bool_values = np.array(collisions[key]) > 0.0
        plt.plot(timestamps, bool_values, label=f"{key} Collisions")
    plt.xlabel("Time (s)")
    plt.ylabel("Collision Event")
    plt.title("Collision Data Over Time")
    plt.legend()
    plt.grid()

    plt.figure(figsize=(10, 6))
    for key in collisions.keys():
        values = np.array(collisions[key])
        plt.plot(timestamps, values, label=f"{key} Collisions")
    plt.xlabel("Time (s)")
    plt.ylabel("Collision Volume Fraction")
    plt.title("Collision Data Over Time")
    plt.legend()
    plt.grid()
    plt.show()

    # Visualize single timestamp
    timestamp = 26.0
    index = np.argmin(np.abs(timestamps - timestamp))
    robot.set_state(pitch, yaw, joint_pos[index, :])
    robot.visualize_with_collision_spheres(robot_name, non_blocking=False)

    return


if __name__ == "__main__":
    main()
