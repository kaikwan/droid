from droid.controllers.gello_controller import GELLOPolicy
# from droid.controllers.oculus_controller import VRPolicy
from droid.robot_env import RobotEnv
from droid.trajectory_utils.misc import collect_trajectory

# Make the robot env
env = RobotEnv(action_space="joint_position", gripper_action_space="policy")
# controller = VRPolicy()
controller = GELLOPolicy()

print("Ready")
collect_trajectory(env, controller=controller, save_filepath="/home/arhan/projects/clean-up-the-kitchen/data/traj.hdf5")
