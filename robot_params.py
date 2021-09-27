import math
import numpy as np


class PandaParams(object):

  def __init__(self):
    self.name = "panda"
    self.num_dofs = 7
    self.end_effector_idx = 11  #8
    self.first_finger_idx = 9
    self.num_fingers = 2
    # Lower limits for null space (TODO: set them to proper range)
    self.ll = [-7] * self.num_dofs
    # Upper limits for null space (TODO: set them to proper range)
    self.ul = [7] * self.num_dofs
    # Joint ranges for null space (TODO: set them to proper range)
    self.jr = [7] * self.num_dofs
    # Joint positions
    self.initial_joint_pos = [
        0.98, 0.458, 0.31, -2.24, -0.30, 2.66, 2.32, 0.02, 0.02
    ]
    # Rest poses for null space
    self.rp = self.initial_joint_pos
    self.urdf_path = "franka_panda/panda.urdf"
    # Max reach for Franka panda is 850mm, setting to 830mm for safety
    self.max_reach = 0.83
    self.min_radius = 0.25
    self.max_angles = np.array([math.pi, math.pi / 2, math.pi / 2])
    self.min_angles = np.array([0, -math.pi / 2, -math.pi / 2])
    self.angle_range = self.max_angles - self.min_angles
    # Initial position is where the whole robot is pointing upward
    self.gripper_norm = [0, 0, 1]

    # Initial height of robot gripper (in vertical axis)
    self.initial_gripper_height = 0.2
    # Default orientation of robot gripper
    self.default_orn = np.array([math.pi / 2, 0, 0])
    # Default position of robot gripper
    self.default_pos = [0.1376, 0.2656, -0.5222]
