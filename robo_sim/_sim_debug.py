from ._requirements import *


def print_quat(self, name, quat):
  """Print decomposed quaternion (unit normal vector and rotation angle).
  n̂ = [x y z]
  quat = [x*sin(theta/2), y*sin(theta/2), z*sin(theta/2), cos(theta/2)]
  """
  theta = math.acos(quat[3]) * 2
  normal = np.array(quat[0:3]) / math.sin(theta / 2)
  if theta != 0:
    print("{} normal: {}, theta: {}".format(name, normal, theta))
  else:
    print("{} quat: {}".format(name, quat))


def get_mesh_principal_axis(self, mesh_name: str, copy_idx: int = 0):
  """Calculate current principal axis of mesh.
  Args:
    mesh_name: name of mesh object type
    copy_idx: Index of mesh object you wish to query
  Return: (principal_axis[x,y,z], mesh_position[x,y,z])
  """
  mesh_pos, mesh_orn = self.bt.getBasePositionAndOrientation(
      self.meshes[mesh_name][copy_idx])
  r = R.from_quat(mesh_orn)
  principal_axis = r.apply(self.norms[mesh_name][copy_idx])
  return (principal_axis, mesh_pos)


def get_mesh_pose(self,
                  mesh_name: str,
                  copy_idx: int = 0,
                  as_euler: bool = True):
  """Returns pose (position, orientation) of an object.
  Args:
    mesh_name: name of mesh object type
    copy_idx: Index of mesh object you wish to query
    as_euler: If set to false, orientation is returned in quaternion format
  """
  mesh_pos, mesh_orn = self.bt.getBasePositionAndOrientation(
      self.meshes[mesh_name][copy_idx])

  if as_euler:
    mesh_orn = self.bt.getEulerFromQuaternion(mesh_orn)

  return (mesh_pos, mesh_orn)


def get_mesh_ground_plane_proj(self, mesh_principal, mesh_pos):
  """Calculates the vector resulting from projecting the mesh principal axis onto
  the ground plane.
  Args:
    mesh_principal: Mesh principal axis[x,y,z]
    mesh_pos: Mesh position[x,y,z]
  Returns:
    (ground_plane_proj: calculated projection,
    poi: point of intersection between mesh_principal and ground plane)
  """
  poi = self.compute_vec2plane_intersection(
      vec=mesh_principal,
      vec_point=mesh_pos,
      plane_norm=self.norms['ground_plane'][0],
      plane_point=[0, 0, 0])
  ground_plane_proj = self.compute_vec2plane_projection(
      vec=mesh_principal, plane_norm=self.norms['ground_plane'][0])

  return ground_plane_proj, poi


def visualize_poses(self) -> None:
  """Draws all principal axis vectors, robot limits, and endeffector finger
  normals for debug visualization."""
  for mesh_name in self.norms:
    if mesh_name == 'jig':
      continue

    mesh_principal, mesh_pos = self.get_mesh_principal_axis(mesh_name)
    self.draw_line(start_point=mesh_pos, vec=mesh_principal, length_scale=-1.0)
    # print("Mesh: {}, norm: {}, pos: {}".format(mesh_name, v_norm_rot, mesh_pos))
    ground_plane_proj, poi = self.get_mesh_ground_plane_proj(
        mesh_principal, mesh_pos)
    if ground_plane_proj is not None:
      print("mesh: {}, poi: {}, norm: {}".format(mesh_name, poi,
                                                 ground_plane_proj))
      self.draw_line(start_point=poi,
                     vec=ground_plane_proj,
                     length_scale=1.0,
                     colour=[1, 140 / 255, 0])

  gripper_state = self.bt.getLinkState(self.robot,
                                       self.robot_params.end_effector_idx, True,
                                       True)
  gripper_pos = gripper_state[4]
  gripper_orn = gripper_state[5]
  gripper_norm_rot = R.from_quat(gripper_orn).apply(
      self.robot_params.gripper_norm)
  self.draw_line(gripper_pos, gripper_norm_rot)
  print("Gripper norm: {}, pos: {}".format(mesh_principal, gripper_pos))

  finger_interior_normal, finger1_pos = self.get_finger_interior_normal(
      self.robot_params)
  self.draw_line(start_point=finger1_pos,
                 vec=finger_interior_normal,
                 length_scale=1.0)

  self.draw_robot_limits()


def draw_robot_limits(self) -> None:
  """Draws circles depicting robot limits based on min/max radius."""
  origin = [0, 0, 0]
  num_edges = 50  # Number of edges used for approximating circle.
  min_radius_lines = self.draw_circle(origin=origin,
                                      radius=self.robot_params.min_radius,
                                      num_edges=num_edges)

  max_radius_lines = self.draw_circle(origin=origin,
                                      radius=self.robot_params.max_reach,
                                      num_edges=num_edges)

  time.sleep(1.0)

  for line in min_radius_lines:
    # Remove previously drawn lines visualizing min radius
    self.bt.removeUserDebugItem(line)

  for line in max_radius_lines:
    # Remove previously drawn lines visualizing max radius
    self.bt.removeUserDebugItem(line)


def draw_circle(self,
                origin: [float],
                radius: float,
                num_edges: int,
                edge_lifetime: int = 0):
  """Draws circle in simulation of specified resolution.
  Args:
    origin: Circle origin (in 3D cartesian coordinates)
    radius: Circlce radius
    num_edges: Controls resolution; more edges=>better circle
    edge_lifetime: How long to keep a line segment in the circle alive for
  Returns:
    lines: List of references to line objects in simulation.
  """
  arc_points = np.linspace(0, 2 * math.pi, num_edges)
  height = 0  # Ground plane
  origin = np.array(origin)
  lines = []

  if num_edges < 2:
    raise ValueError('Cannot draw circle with less than 2 points.')

  for i in range(1, len(arc_points)):
    cur_theta = arc_points[i]
    prev_theta = arc_points[i - 1]

    cur_point = self._compute_cart_from_polar(height, radius, cur_theta)
    cur_point += origin
    prev_point = self._compute_cart_from_polar(height, radius, prev_theta)
    prev_point += origin

    line = cur_point - prev_point
    lines.append(
        self.draw_line(start_point=prev_point,
                       vec=line,
                       length_scale=1.0,
                       colour=[1, 1, 0],
                       lifetime=edge_lifetime,
                       line_width=4.0))

  return lines


def draw_line(self,
              start_point: np.ndarray,
              vec: np.ndarray,
              length_scale: float = 0.1,
              colour: [float] = [0, 1.0, 0],
              lifetime: float = 1.0,
              line_width: float = 1.0):
  """Helper function to draw debug line with specified properties.
  Args:
    start_point: 3D cartesian point specifying where line begins
    vec: 3D Cartesian vector specifying line direction
    length_scale: Scalar factor to direction vector controlling line magnitude
    colour: RGB colour of line as percent [0,255]/255
    lifetime: Number of seconds to maintain line for
    line_width: Controls line thickness
  Returns:
    Reference to newly created line object (store this to erase in future)
  """
  return self.bt.addUserDebugLine(start_point,
                                  start_point + (vec * length_scale),
                                  colour,
                                  lifeTime=lifetime,
                                  lineWidth=line_width)
