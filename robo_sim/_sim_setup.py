from ._requirements import *


def _load_meshes(self, meshes, flags) -> None:
  """Loads mesh objects and robot into simulation.
    Stores URDF ID into self.meshes{} for future reference.
    Stores vector describing initial principal axis pose in self.norms{}.
    """
  # Stores all other mesh objects
  self.meshes = {}
  self.norms = {}
  # Load meshes
  for mesh_name in meshes:
    mesh = meshes[mesh_name]
    self.meshes[mesh_name] = []
    self.norms[mesh_name] = []
    for i in range(mesh.num_copies):
      pos = mesh.mesh_pos
      orn = mesh.mesh_orn
      if mesh.randomize_pose:
        pos[0] = np.random.uniform(pos[0] - 0.05, pos[0] + 0.05)
        pos[2] = np.random.uniform(pos[2] - 0.05, pos[2] + 0.05)
        if mesh_name == 'bolt':
          theta = 80
          alpha = np.random.uniform(0, 360)
          r = R.from_euler('xz', [theta, alpha], degrees=True)
          orn = r.as_quat()
        else:
          theta = np.random.uniform(0, 60)
          r = R.from_euler('z', theta, degrees=True)
          orn = r.as_quat()

      self.meshes[mesh_name].append(
          self.bt.loadURDF(mesh.urdf_path,
                           pos,
                           baseOrientation=orn,
                           useFixedBase=mesh.is_fixed,
                           flags=flags))
      # Store initial pose since all transformations are applied based on this.
      self.norms[mesh_name].append(np.array(mesh.orig_axis))

  # Mesh object of robot used in simulation
  self.robot = self.bt.loadURDF(self.robot_params.urdf_path,
                                self.robot_pos,
                                self.robot_orn,
                                useFixedBase=True,
                                flags=flags)


def _remove_meshes(self, mesh_names: [str] = None) -> None:
  """Removes all meshes objects of specified type from simulation."""
  if mesh_names is None:
    mesh_names = self.meshes.keys()

  print("Removing all {} from simulation".format(mesh_names))
  for mesh_name in mesh_names:
    meshes = self.meshes[mesh_name]

    i = len(meshes) - 1

    # Deletion is expensive, so iterate backwards in list
    while i >= 0:
      # Remove from simulation and delete from list of meshes we're tracking.
      self.bt.removeBody(meshes[i])
      del meshes[i]

      i -= 1

  self.bt.removeBody(self.robot)


def _init_joints(self, joint_pos) -> None:
  """Initializes all joints in the robot.

    Removes all damping from joints, centers fingers, enables a force torque
    sensor, and resets joints to a specified initial position.
    """
  self._constrain_fingers_centerered()

  index = 0
  for joint_idx in range(self.bt.getNumJoints(self.robot)):
    self.bt.changeDynamics(self.robot,
                           joint_idx,
                           linearDamping=0,
                           angularDamping=0)
    self.bt.enableJointForceTorqueSensor(self.robot, joint_idx, True)

    joint_type = self.bt.getJointInfo(self.robot, joint_idx)[2]
    if (joint_type == self.bt.JOINT_PRISMATIC):
      self.bt.resetJointState(self.robot, joint_idx, joint_pos[index])
      index += 1

    if (joint_type == self.bt.JOINT_REVOLUTE):
      self.bt.resetJointState(self.robot, joint_idx, joint_pos[index])
      index += 1

  self._step_sim(200)
  pos, orn = self.get_endeffector_real_pose(self.robot_params)
  # Offset initial gripper height (manually calibrated)
  pos += np.array([0, self.robot_params.initial_gripper_height, 0])
  self._set_robot_pose(pos, orn)
  # Used to center gripper with jig if needed
  self.initial_pos = self._get_robot_pos()
  self.radius, self.theta = self._cart2polar(self.initial_pos)

  finger_interior_normal, finger_pos = self.get_finger_interior_normal(
      self.robot_params)
  finger_ground_proj, poi = self.get_mesh_ground_plane_proj(
      finger_interior_normal, finger_pos)

  self.gripper_initial_theta = math.atan2(finger_ground_proj[2],
                                          finger_ground_proj[0])

  self.move_robot(pos, orn)


def _constrain_fingers_centerered(self) -> None:
  """Creates a constraint to keep the gripper finger centered."""
  constraint = self.bt.createConstraint(self.robot,
                                        9,
                                        self.robot,
                                        10,
                                        jointType=self.bt.JOINT_GEAR,
                                        jointAxis=[1, 0, 0],
                                        parentFramePosition=[0, 0, 0],
                                        childFramePosition=[0, 0, 0])
  self.bt.changeConstraint(constraint, gearRatio=-1, erp=0.1, maxForce=50)


def get_endeffector_real_pose(self, robot_params=None, as_euler=True):
  """Computes position and orientation of the endeffector in world frame.
  Set as_euler: False to return orientation in quaternion format.
  Returns (pos, orn).
  """
  if robot_params is None:
    robot_params = self.robot_params

  endeffector_state = self.bt.getLinkState(
      self.robot,
      linkIndex=robot_params.end_effector_idx,
      computeLinkVelocity=False,
      computeForwardKinematics=True)

  # Return (worldLinkFramePos, worldLinkFrameOrientation)
  pos, orn = endeffector_state[4], endeffector_state[5]
  if as_euler:
    orn = self.bt.getEulerFromQuaternion(orn)
  return (np.array(pos), np.array(orn))


def get_endeffector_real_pos_polar(self, robot_params=None):
  """Returns real (current) position of endeffector in polar coords
  after it's been projected onto ground plane (birds-eye view)."""
  if robot_params is None:
    robot_params = self.robot_params

  pos, orn = self.get_endeffector_real_pose(robot_params)
  return self._cart2polar(pos)


def get_finger_interior_normal(self, robot_params=None):
  """Calculates and returns vector joining the interior of the gripper fingers.
  Returns: (np.array(interior_normal), np.array(finger_position))
  """

  if robot_params is None:
    robot_params = self.robot_params

  finger1_pos = np.array(
      self.bt.getLinkState(self.robot,
                           linkIndex=robot_params.first_finger_idx,
                           computeLinkVelocity=False,
                           computeForwardKinematics=True)[0])
  finger2_pos = np.array(
      self.bt.getLinkState(self.robot,
                           linkIndex=robot_params.first_finger_idx + 1,
                           computeLinkVelocity=False,
                           computeForwardKinematics=True)[0])
  interior_normal = finger2_pos - finger1_pos

  return (interior_normal, finger1_pos)
