import warnings
import gymnasium as gym
import numpy as np
from urdfenvs.robots.generic_urdf.generic_diff_drive_robot import GenericDiffDriveRobot
from urdfenvs.urdf_common.urdf_env import UrdfEnv
import pybullet as p
import pybullet_data
import os


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
            facing_direction = '-y',
        ),
    ]
    env: UrdfEnv = UrdfEnv(
        dt=0.01, robots=robots, render=render
    )
    
    client_id = env._cid

    p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=client_id)
    
    table = p.loadURDF("table/table.urdf", basePosition=[2.5, 0, 0])
    tray = p.loadURDF("tray/traybox.urdf", basePosition=[2.8, 0, 0.64])
    lego = p.loadURDF("lego/lego.urdf", basePosition=[2.8, -0.1, 0.68], globalScaling=2.0)
    cube_small = p.loadURDF("cube_small.urdf", basePosition=[2.9, -0.075, 0.7], globalScaling=2.0)
    p.changeDynamics(cube_small, -1, mass=3, physicsClientId=client_id)
    racecar = p.loadURDF("racecar/racecar.urdf", basePosition=[2.0, 0, 0.64])

    base_dir = os.path.dirname(os.path.abspath(__file__)) # Python file directory
    repo_root = os.path.dirname(base_dir) # Move up to the repository root
    custom_asset_path = os.path.join(repo_root, "urdfenvs", "assets") # assets directory present in urdfenvs

    p.setAdditionalSearchPath(custom_asset_path, physicsClientId=client_id)
    banana = p.loadURDF("banana/banana.urdf", basePosition=[2.8, 0.125, 0.67])
    
    action = np.zeros(env.n())
    action[0] = 0.2
    action[1] = 0.0
    action[5] = -0.1
    ob = env.reset(
        pos=np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0, -1.5, 0.0, 1.8, 0.5])
    )
    print(f"Initial observation : {ob}")
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
