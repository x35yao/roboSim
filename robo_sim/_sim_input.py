from ._requirements import *
from .sim_states import SimStates

SHIFT_KEY_VALUE = 65306


def _get_keyboard_input(self) -> None:
    """Updates simulation state based on keyboard input."""
    keys = self.bt.getKeyboardEvents()
    if (len(keys) > 0):
        for k, v in keys.items():
            if v & self.bt.KEY_WAS_TRIGGERED:
                if k == SHIFT_KEY_VALUE:
                    # Toggle shift key flag to allow mapping uppercase letters.
                    self.shift_key_pressed = True
                elif self.shift_key_pressed:
                    # Set state to the uppercase equivalent of key.
                    self.state = ord(chr(k).upper())
                else:
                    self.state = k
            if v & self.bt.KEY_WAS_RELEASED:
                if k == SHIFT_KEY_VALUE:
                    self.shift_key_pressed = False
                else:
                    self.state = SimStates.ready


def _process_state(self) -> None:
    """Processes internal state and execute appropriate action.
    """
    # If function is called which executes a motion sequence, set flag to True.
    # Set to False by default so caller must move robot to calculated pose.
    motion_executed = False
    pos, orn = self._get_robot_pose()
    dx, dy, dz = [0, 0, 0]
    dalpha, dbeta, dgamma = [0, 0, 0]

    if self.state == SimStates.assemble:
        self.assemble()
        motion_executed = True
    elif self.state == SimStates.clean:
        self.clean()
        motion_executed = True
    elif self.state == SimStates.scan_world:
        self.get_world_states()
        motion_executed = True
    elif self.state == SimStates.capture_image:
        self.capture_image()
        motion_executed = True
    elif self.state == SimStates.reset:
        self._init_joints(self.robot_params.initial_joint_pos)
        motion_executed = True
    elif self.state == SimStates.visualize_pose:
        self.visualize_poses()
        motion_executed = True
    elif self.state == SimStates.pick_up:
        # Pick up object directly underneath current gripper position
        # 0.01m has been empirically found to be a good vertical height for pick up
        target_pos = [pos[0], 0.01, pos[2]]
        # Align gripper orn to default orn (pointing down perp to ground)
        target_orn = self._align_orns(orn, exclude_vertical_axis=True)
        self.execute_pick_up(target_pos, target_orn)
        motion_executed = True
    elif self.state == SimStates.pick_up_bolt_head:
        # Pick up bolt head directly underneath current gripper position
        # 0.11m has been empirically found to be a good vertical height for pick up
        target_pos = [pos[0], 0.11, pos[2]]
        # Align gripper orn to default orn (pointing down perp to ground)
        target_orn = self._align_orns(orn, exclude_vertical_axis=True)
        self.execute_pick_up(target_pos, target_orn)
        motion_executed = True
    elif self.state == SimStates.put_down:
        # Put down object in gripper directly underneath current gripper position
        # 0.12m has been empirically found to be a good vertical height for release
        target_pos = [pos[0], 0.12, pos[2]]
        self.execute_put_down(target_pos, orn)
        motion_executed = True
    elif self.state == SimStates.orient_bolt:
        self.orient_gripper_bolt()
        motion_executed = True
    elif self.state == SimStates.orient_bolt_with_bolt_hole:
        # Orient bolt so trunk is perpendicular to bolt hole opening
        self.execute_put_down_bolt('bolt_hole', 0.115)
        motion_executed = True
    elif self.state == SimStates.put_bolt_head_nut_hole:
        self.execute_put_down_bolt('nut', 0.115)
        motion_executed = True

    elif self.state == SimStates.orient_nut:
        # TODO: Fix, this causes sim to fail way too often
        orn_mesh = self._get_mesh_orn(mesh_name='nut')
        # print("nut mesh: ", np.degrees(orn_mesh))
        r = R.from_euler('xyz', orn_mesh)
        R_matrix = r.as_matrix()
        R_matrix_new = np.zeros((3, 3))
        # print(R_matrix[:, 2])
        if R_matrix[1, 2] > 0:
            R_matrix_new[:, 0] = R_matrix[:, 0]
            R_matrix_new[:, 1] = -R_matrix[:, 1]
            R_matrix_new[:, 2] = -R_matrix[:, 2]
        else:
            R_matrix_new[:, 0] = R_matrix[:, 0]
            R_matrix_new[:, 1] = R_matrix[:, 1]
            R_matrix_new[:, 2] = R_matrix[:, 2]
        r_new = R.from_matrix(R_matrix_new)
        orn = r_new.as_euler('xyz')
    elif self.state == SimStates.default_pose:
        # Set pose to default pose, except retain cur orn around vertical
        orn = self._align_orns(target_orn=orn, exclude_vertical_axis=True)
        pos = self._get_default_gripper_pos()
    elif self.state == SimStates.goto_nut:
        pos = self._get_mesh_pos(mesh_name='nut', height=0.3)
        orn = self._get_default_gripper_orn()
    elif self.state == SimStates.goto_bolt:
        pos = self._get_mesh_pos(mesh_name='bolt', height=0.3)
        orn = self._align_orns(target_orn=orn, exclude_vertical_axis=True)
    elif self.state == SimStates.goto_nut_hole:
        pos = self._get_mesh_pos(mesh_name='nut_hole', height=0.3)
        # orn = self._get_default_gripper_orn()
    elif self.state == SimStates.goto_bolt_hole:
        pos = self._get_mesh_pos(mesh_name='bolt_hole', height=0.3)
        orn = self._get_default_gripper_orn()
    elif self.state == SimStates.pick_up_from_bin:
        target_pos = [pos[0], 0.05, pos[2]]
        # Align gripper orn to default orn (pointing down perp to ground)
        target_orn = self._align_orns(orn, exclude_vertical_axis=True)
        self.execute_pick_up(target_pos, target_orn)
        motion_executed = True
    elif self.state == SimStates.goto_bin:
        offset = 0.15
        pos = self._get_mesh_pos(mesh_name='bin_target', height=0.3)
        # The object origin is off by 0.15 in x direction
        pos[0] = pos[0] - offset
        orn = self._get_default_gripper_orn()
    elif self.state == SimStates.put_in_bin:
        target_pos = [pos[0], 0.12, pos[2]]
        pos, orn = self._get_robot_pose()
        self.execute_put_down(target_pos, orn)
        motion_executed = True


    # Change finger width
    elif self.state == SimStates.gripper_close:
        self.finger_target = 0.01
    elif self.state == SimStates.gripper_open:
        self.finger_target = 0.04

    # Translate gripper
    elif self.state == SimStates.x_pos:
        dx = self.delta_pos
    elif self.state == SimStates.y_pos:
        dy = self.delta_pos
    elif self.state == SimStates.z_pos:
        dz = self.delta_pos
    elif self.state == SimStates.x_neg:
        dx = -1.0 * self.delta_pos
    elif self.state == SimStates.y_neg:
        dy = -1.0 * self.delta_pos
    elif self.state == SimStates.z_neg:
        dz = -1.0 * self.delta_pos

    # Rotate gripper
    elif self.state == SimStates.rot_x_pos:
        dalpha = self.delta_theta
    elif self.state == SimStates.rot_y_pos:
        dbeta = self.delta_theta
    elif self.state == SimStates.rot_z_pos:
        dgamma = self.delta_theta
    elif self.state == SimStates.rot_x_neg:
        dalpha = -1.0 * self.delta_theta
    elif self.state == SimStates.rot_y_neg:
        dbeta = -1.0 * self.delta_theta
    elif self.state == SimStates.rot_z_neg:
        dgamma = -1.0 * self.delta_theta

    # Add calculated offset to current pose
    pos = pos + np.array([dx, dy, dz])
    orn = orn + np.array([dalpha, dbeta, dgamma])

    # If action hasn't been executed already, move robot to target pose.
    if not motion_executed:
        self.move_robot(pos, orn, num_sim_steps=1)


def _get_mesh_pos(self, mesh_name: str, height: float, copy_idx: int = 0):
    """Returns position of mesh as a numpy array [x,y,z]. NOTE: y-coordinate of
    mesh is replaced with height argument.
    Args:
      mesh_name: Name of mesh (must have already been loaded into sim)
      height: Height to set mesh vertical component to. (Useful to set this
        to y-coordinate of gripper (since some meshes might be beneath ground plane.)
      copy_idx: Index of mesh object you wish to query
    """
    mesh_pos, mesh_orn = self.bt.getBasePositionAndOrientation(
        self.meshes[mesh_name][copy_idx])
    return np.array([mesh_pos[0], height, mesh_pos[2]])


def _get_mesh_orn(self, mesh_name: str, copy_idx: int = 0):
    """Returns orientation of mesh as a numpy array in Euler angles format [alpha, beta, gamma].
    Args:
      mesh_name: Name of mesh (must have already been loaded into sim)
      copy_idx: Index of mesh object you wish to query
    """
    mesh_pos, mesh_orn = self.bt.getBasePositionAndOrientation(
        self.meshes[mesh_name][copy_idx])
    return np.array(self.bt.getEulerFromQuaternion(mesh_orn))


def _get_default_gripper_orn(self):
    """Returns numpy array containing default gripper orientation."""
    return np.copy(self.robot_params.default_orn)


def _get_default_gripper_pos(self):
    """Returns numpy array containing default gripper position."""
    return np.copy(self.robot_params.default_pos)


def _align_orns(self,
                target_orn,
                ref_orn=None,
                axes=[0, 1, 2],
                exclude_vertical_axis=False):
    """Aligns target orientation with values of the reference orientation for the
    specified axes.

    E.g. target_orn = [3.14, 3.14, 3.14], ref_orn=[0, 0.1, 1.57], axes=[0,2]
         return: [0, 3.14, 1.57]

    Returns: target_orn (post alignment)
    """
    if ref_orn is None:
        ref_orn = self._get_default_gripper_orn()

    for axis in axes:
        if (axis == self.vertical_axis_idx and exclude_vertical_axis == True):
            continue
        target_orn[axis] = ref_orn[axis]

    return target_orn
