import warnings
import gymnasium as gym
import numpy as np
from urdfenvs.robots.generic_urdf.generic_diff_drive_robot import GenericDiffDriveRobot
from mpscenes.obstacles.urdf_obstacle import UrdfObstacle
from urdfenvs.urdf_common.urdf_env import UrdfEnv
import pybullet as p
import pybullet_data
import os
from urdfenvs.scene_examples.obstacles import (
    sphereObst1,
    sphereObst2,
    urdfObst1,
    dynamicSphereObst3,
    wall_obstacles,
)
import logging

# Ensure this is one of the first lines of execution.
# This configures the root logger to show INFO level messages.
logging.basicConfig(level=logging.INFO)
import math

import inspect
print(f"UrdfEnv class loaded from: {inspect.getfile(UrdfEnv)}")


def run_albert(n_steps=1000, render=False, goal=True, obstacles=True):
    robots = [
        GenericDiffDriveRobot(
            urdf="albert.urdf",
            mode="vel",
            actuated_wheels=["wheel_right_joint", "wheel_left_joint"],
            castor_wheels=["rotacastor_right_joint", "rotacastor_left_joint"],
            wheel_radius = 0.08,
            wheel_distance = 0.494,
            spawn_rotation = 0,
            facing_direction = 'x',
        ),
    ]
    env: UrdfEnv = UrdfEnv(
        dt=0.01, robots=robots, render=render
    )
    # print(robots[0].__dict__.keys()) 
    # print(robots[0]._joint_names) 
    # [X_position(1), Y_position(1), theta_orientation(1), arm base rotation, all joints, finger left and right(2)]
    # ob = env.reset(
    #     pos=np.array([2.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.5, 0.0, 1.8, 0.5])
    # )

    print(env._robots[0].__dict__.keys()) # Debugging line to check robot attributes
    ob = robots[0].reset(
        pos=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.5, 0.0, 1.8, 1.0, 0.0, 0.0]),
        vel=np.zeros(12),
        mount_position=None,
        mount_orientation=None,
    )
    robot_id = env._robots[0]._robot
    client_id = env._cid

    pybullet_path = pybullet_data.getDataPath()
    base_dir = os.path.dirname(os.path.abspath(__file__)) # Python file directory
    repo_root = os.path.dirname(base_dir) # Move up to the repository root
    custom_asset_path = os.path.join(repo_root, "urdfenvs", "assets") # assets directory present in urdfenvs

    table_urdf = os.path.join(pybullet_path, "table", "table.urdf")
    tray_urdf = os.path.join(pybullet_path, "tray", "traybox.urdf")
    lego_urdf = os.path.join(pybullet_path, "lego", "lego.urdf")
    cube_urdf = os.path.join(pybullet_path, "cube_small.urdf") 
    racecar_urdf = os.path.join(pybullet_path, "racecar", "racecar.urdf")

    tableDict = {
    "type": "urdf",
    "geometry": {"position": [2.5, 0, 0]},
    "urdf": table_urdf,
    }
    table = UrdfObstacle(name="table", content_dict=tableDict)

    trayDict = {
    "type": "urdf",
    "geometry": {"position": [2.8, 0, 0.64]},
    "urdf": tray_urdf,
    }
    tray = UrdfObstacle(name="tray", content_dict=trayDict)

    legoDict = {
    "type": "urdf",
    "geometry": {"position": [2.8, -0.1, 0.68]},
    "urdf": lego_urdf,
    "scaling": 2.0,
    }
    lego = UrdfObstacle(name="lego", content_dict=legoDict)

    cubeDict = {
    "type": "urdf",
    "geometry": {"position": [2.9, -0.075, 0.7]},
    "urdf": cube_urdf,
    "scaling": 2.0,
    }
    cube_small = UrdfObstacle(name="cube_small", content_dict=cubeDict)

    racecarDict = {
    "type": "urdf",
    "geometry": {"position": [2.0, 0, 0.64]},
    "urdf": racecar_urdf,
    }
    racecar = UrdfObstacle(name="racecar", content_dict=racecarDict)
    print("--- DEBUG: Starting to add obstacles ---")
    env.add_obstacle(table)
    env.add_obstacle(tray)
    env.add_obstacle(lego)
    env.add_obstacle(cube_small)
    env.add_obstacle(racecar)
    if obstacles:
        env.add_obstacle(urdfObst1)

    desired_cube_mass = 3
    for obst_id, obst in env.get_obstacles().items():
        try:
            if obst.name() == "cube_small":
                p.changeDynamics(obst_id, -1, mass=desired_cube_mass, physicsClientId=client_id)
        except Exception:
            continue

    banana_urdf = os.path.join(repo_root, "urdfenvs", "assets", "banana", "banana.urdf")
    bananaDict = {
    "type": "urdf",
    "geometry": {"position": [2.8, 0.125, 0.67]},
    "urdf": banana_urdf,
    }
    banana = UrdfObstacle(name="banana", content_dict=bananaDict)

    env.add_obstacle(banana)
    # env.update_obstacles()

    # THIS LOADS THE OBJECTS USING THE loadURDF FUNCTION but is not added as an obstacle to the env
    # p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_id)
    
    # table = p.loadURDF("table/table.urdf", basePosition=[2.5, 0, 0],)
    # tray = p.loadURDF("tray/traybox.urdf", basePosition=[2.8, 0, 0.64])
    # lego = p.loadURDF("lego/lego.urdf", basePosition=[2.8, -0.1, 0.68], globalScaling=2.0)
    # cube_small = p.loadURDF("cube_small.urdf", basePosition=[2.9, -0.075, 0.7], globalScaling=2.0)
    # p.changeDynamics(cube_small, -1, mass=3, physicsClientId=client_id)
    # racecar = p.loadURDF("racecar/racecar.urdf", basePosition=[2.0, 0, 0.64])

    # base_dir = os.path.dirname(os.path.abspath(__file__)) # Python file directory
    # repo_root = os.path.dirname(base_dir) # Move up to the repository root
    # custom_asset_path = os.path.join(repo_root, "urdfenvs", "assets") # assets directory present in urdfenvs

    # p.setAdditionalSearchPath(custom_asset_path, physicsClientId=client_id)
    # banana = p.loadURDF("banana/banana.urdf", basePosition=[2.8, 0.125, 0.67])



    action = np.zeros(env.n())
    # action[0] = 0.2
    # action[1] = 0.0
    # action[5] = -0.1
    # ob = env.reset(
    #     pos=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.5, 0.0, 1.8, 0.5])
    # )



    # 1. Get Mobile Base Position (x, y, z) and Orientation (Quat)
    base_pos_quat = p.getBasePositionAndOrientation(robot_id, physicsClientId=client_id)
    # print("Mobile Base:", base_pos_quat)
    # print(f"Initial observation : {ob}")
    history = []
    for _ in range(n_steps):
        ob, *_ = env.step(action)
        history.append(ob)
    env.close()
    return history


if __name__ == "__main__":
    show_warnings = False
    warning_flag = "default" if show_warnings else "ignore"
    with warnings.catch_warnings():
        warnings.filterwarnings(warning_flag)
        run_albert(render=True)
