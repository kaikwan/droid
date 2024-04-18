from droid.robot_env import RobotEnv
from droid.trajectory_utils.misc import custom_replay_trajectory

trajectory_folderpath = "/home/sasha/DROID/data/success/2023-02-16/Thu_Feb_16_16:27:00_2023"
action_space = "cartesian_position"

# Make the robot env
env = RobotEnv(action_space=action_space)

# Replay Trajectory #
h5_filepath = trajectory_folderpath + "/trajectory.h5"
custom_replay_trajectory(env, filepath=h5_filepath)
