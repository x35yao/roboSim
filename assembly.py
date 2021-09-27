import pybullet as p
import pybullet_data as pd
import math
import time
import numpy as np
import quaternion
import argparse
import collections
from scipy.spatial.transform import Rotation as R

import robo_sim
import robot_params

SQRT_2_INV = 0.7071067812  # 1/sqrt(2)
BASE_ORN = [-SQRT_2_INV, 0, 0, SQRT_2_INV]
DEFAULT_ORN = [0, 0, 0, 1]
BASE_NORM = [0, 0, 1]

r = R.from_quat(BASE_ORN)

fps = 240.
timeStep = 1. / fps
Mesh = collections.namedtuple(
    'Mesh',
    'urdf_path mesh_pos mesh_orn orig_axis is_fixed num_copies randomize_pose')
meshes = {}
num_copies = 1

meshes['ground_plane'] = Mesh(urdf_path="plane.urdf",
                              mesh_pos=[0, 0, 0],
                              mesh_orn=BASE_ORN,
                              orig_axis=[0, 1, 0],
                              is_fixed=True,
                              num_copies=1,
                              randomize_pose=False)
meshes['nut_hole'] = Mesh(urdf_path="/meshes/urdfs/holder.urdf",
                          mesh_pos=[-0.0270, -0.0506, -0.6780],
                          mesh_orn=BASE_ORN,
                          orig_axis=BASE_NORM,
                          is_fixed=True,
                          num_copies=1,
                          randomize_pose=False)
meshes['bolt_hole'] = Mesh(urdf_path="/meshes/urdfs/holder.urdf",
                           mesh_pos=[0.129, -0.01, -0.525],
                           mesh_orn=BASE_ORN,
                           orig_axis=BASE_NORM,
                           is_fixed=True,
                           num_copies=1,
                           randomize_pose=False)
meshes['nut'] = Mesh(urdf_path="/meshes/urdfs/nut.urdf",
                     mesh_pos=[0.4, 0, 0],
                     mesh_orn=BASE_ORN,
                     orig_axis=BASE_NORM,
                     is_fixed=False,
                     num_copies=num_copies,
                     randomize_pose=True)
meshes['bolt'] = Mesh(urdf_path="meshes/urdfs/bolt.urdf",
                      mesh_pos=[0.5, 0, 0],
                      mesh_orn=BASE_ORN,
                      orig_axis=BASE_NORM,
                      is_fixed=False,
                      num_copies= num_copies,
                      randomize_pose=True)
meshes['jig'] = Mesh(urdf_path="/meshes/urdfs/jig.urdf",
                     mesh_pos=[0, -0.3, -0.6],
                     mesh_orn=BASE_ORN,
                     orig_axis=BASE_NORM,
                     is_fixed=True,
                     num_copies=1,
                     randomize_pose=False)

# Simulation CLI Arguments
parser = argparse.ArgumentParser(
    description='Robotics grasping simulation using nut/bolt system')
parser.add_argument('-o',
                    '--out_dir',
                    type=str,
                    default="data/assembly/",
                    help='Output directory to store captured images')
args = parser.parse_args()


def make_np_quat(bt_quat):
  return np.quaternion(bt_quat[3], bt_quat[0], bt_quat[1], bt_quat[2])


def make_bt_quat(np_quat):
  print(np_quat)
  return [np_quat.x, np_quat.y, np_quat.z, np_quat.w]


def init_bt_env(bt):
  bt.connect(bt.GUI)
  bt.configureDebugVisualizer(bt.COV_ENABLE_Y_AXIS_UP, 1)
  bt.configureDebugVisualizer(bt.COV_ENABLE_GUI, 0)
  bt.setPhysicsEngineParameter(maxNumCmdPer1ms=1000)
  bt.resetDebugVisualizerCamera(cameraDistance=1.3,
                                cameraYaw=38,
                                cameraPitch=-22,
                                cameraTargetPosition=[0, -0.13, -0.6])
  bt.setAdditionalSearchPath(pd.getDataPath())
  bt.setGravity(0, -9.8, 0)
  bt.setRealTimeSimulation(0)


def main():
  np.set_printoptions(formatter={'float': lambda x: "{0:0.4f}".format(x)})
  init_bt_env(p)

  robot_state = ([0, 0, 0], BASE_ORN)
  robot = robo_sim.RoboSim(p, robot_params.PandaParams(), robot_state, meshes,
                           args.out_dir)
  while (1):
    # continue
    robot.step(automate_assembly = True)
    time.sleep(timeStep)


if __name__ == '__main__':
  main()
