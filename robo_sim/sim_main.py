from ._requirements import *
from .sim_states import SimStates
from .world import World, Thing
from .objects import Nut, Bolt, Jig, Gripper
from .planner import Planner
from .utils import *

COORD_TYPE_RADIUS = 0
COORD_TYPE_THETA = 1


def step(self, keyboard_input=True, automate_assembly = False, automate_clean = False):
    """Steps simulation based on user input and current state."""
    if automate_assembly:
        self.state = SimStates.assemble

    if automate_clean:
        self.state = SimStates.clean

    # Get keyboard input, map input to sim state, compute desired pose.
    if keyboard_input:
        self._get_keyboard_input()

    # Process updated sim state and execute appropriate action.
    self._process_state()


def assemble(self):
    noise = 0
    goal = 'LOCAL3'
    models_nut = list(np.load('./robo_sim/models_nut.npy'))
    models_bolt = list(np.load('./robo_sim/models_bolt.npy'))
    global_state_vec = np.load('./robo_sim/global_state_vec.npy')
    num_states = 5
    p = Planner(self.get_world_states(),
              num_states,
              noise=0)
    should_reset_sim = False
    self.num_trials = 0
    while not p.is_state_satisfied(goal):
        should_reset_sim = self.run_watchdog(['nut', 'bolt'])
        if should_reset_sim:
            # Don't log data collected so far since this run was unsuccessful
            return self.reset(log_data=False)

        p.world = self.get_world_states()
        p.action_found = False
        if p.make_plan(goal): # Successfully planned
            action = p.current_action
            self.planner_action = action
            print('Cur Action: {}'.format(action))
            # raise
            self.execute_action(action)
            self.num_trials += 1
        else: # Can't plan due to lack of objcts
            break
    self.record_snapshot('ASSEMBLED')
    return self.reset(log_data=True)

def clean(self):
    self.objects_to_record = ['nut', 'bolt', 'bin_origin', 'bin_target']
    bin_target = self.meshes['bin_target'][0]
    task_finished = False
    self.num_trials = 0
    while not task_finished:
        #Loop until there is one object is put in the target bin
        should_reset_sim = self.run_watchdog_bin_picking(['nut', 'bolt'])
        if should_reset_sim:
            # Don't log data collected so far since this run was unsuccessful
            return self.reset(log_data=False)
        ind_nut = self.meshes['nut']
        ind_bolt = self.meshes['bolt']
        ind_to_clean = ind_nut + ind_bolt
        target_obj = np.random.choice(['nut', 'bolt'])
        target_ind = self.meshes[target_obj][0]
        if target_ind in ind_nut:
            # this object is a nut
            action = 'CLEAN_NUT'
        elif target_ind in ind_bolt:
            action = 'CLEAN_BOLT'
        self.execute_action(action)
        if _is_in_bin(self, target_ind, bin_target, 0.4):
            task_finished = True
        self.num_trials += 1
    self.record_snapshot('Cleaned')
    return self.reset(log_data=True)

def run_watchdog_bin_picking(self, mesh_names):
    bin_origin = self.meshes['bin_origin'][0]
    should_reset_simulation = False
    for mesh_name in mesh_names:
        meshes = self.meshes[mesh_name]
        for i, mesh in enumerate(meshes):
            if not _is_in_bin(self, mesh, bin_origin, 0.4):
                self.bt.removeBody(mesh)
                del meshes[i]
        self.meshes[mesh_name] = meshes
        if (len(meshes) < 1):
            # If all meshes of a certain type have been removed, reset the entire
            # simulation so that we can start afresh.
            print("No {} left. Resetting simulation environment.".format(mesh_name))
            should_reset_simulation = True
            return should_reset_simulation

    if self.count_failed_motions > 7:
        # If we've had too many cases where robot failed to reach target pose,
        # it's probably in a bad configuration. Data will be noisy anyway so reset.
        print(
            "Num failed motions ({}) exceeded watchdog threshold. Resetting simulation."
            .format(self.count_failed_motions))
        should_reset_simulation = True
    if self.num_trials > 10:
        should_reset_simulation = True
    return should_reset_simulation

def run_watchdog(self, mesh_names):
    should_reset_simulation = False
    for mesh_name in mesh_names:
        meshes = self.meshes[mesh_name]
        norms = self.norms[mesh_name]
        assert (len(meshes) == len(norms))
        i = len(meshes) - 1

# Deletion is expensive, so iterate backwards in list
    while i >= 0:
        mesh_pos, mesh_orn = self.bt.getBasePositionAndOrientation(meshes[i])
        within_bounds = self.validate_position_mesh(pos=np.array(mesh_pos),
                                                  mesh_name=mesh_name)

        if not within_bounds:
        # Remove from simulation and delete from list of meshes we're tracking.
            print("{} #{} is out of bounds, removing from simulation.".format(
                mesh_name, i))
            self.bt.removeBody(meshes[i])
            del meshes[i]
            del norms[i]

        i -= 1

    self.meshes[mesh_name] = meshes

    if (len(meshes) < 1):
        # If all meshes of a certain type have been removed, reset the entire
        # simulation so that we can start afresh.
        print("No {} left. Resetting simulation environment.".format(mesh_name))
        should_reset_simulation = True
        return should_reset_simulation

    if self.count_failed_motions > 7:
        # If we've had too many cases where robot failed to reach target pose,
        # it's probably in a bad configuration. Data will be noisy anyway so reset.
        print(
            "Num failed motions ({}) exceeded watchdog threshold. Resetting simulation."
            .format(self.count_failed_motions))
        should_reset_simulation = True
    if self.num_trials > 10:
        should_reset_simulation = True
    return should_reset_simulation


def execute_action(self, action):
    self.record_snapshot(action)
  # if action == 'PICK_UP_NUT_FROM_TABLE':
  #   self.state = SimStates.goto_nut
  #   self.step(keyboard_input=False)
  #   self.state = SimStates.orient_nut
  #   self.step(keyboard_input=False)
  #   self.state = SimStates.pick_up
  #   self.step(keyboard_input=False)
    if action == 'PUT_NUT_IN_JIG':
        self.state = SimStates.goto_nut
        self.step(keyboard_input=False)
        # self.state = SimStates.orient_nut
        # self.step(keyboard_input=False)
        self.state = SimStates.pick_up
        self.step(keyboard_input=False)
        self.state = SimStates.goto_nut_hole
        self.step(keyboard_input=False)
        self.state = SimStates.put_down
        self.step(keyboard_input=False)
    # self._step_sim(500)
    # self.capture_image()
  # elif action == 'PICK_UP_BOLT_FROM_TABLE':
  #   self.state = SimStates.goto_bolt
  #   self.step(keyboard_input=False)
  #   self.state = SimStates.orient_bolt
  #   self.step(keyboard_input=False)
  #   self.state = SimStates.pick_up
  #   self.step(keyboard_input=False)
    elif action == 'PUT_BOLT_IN_JIG':
        self.state = SimStates.goto_bolt
        self.step(keyboard_input=False)
        self.state = SimStates.orient_bolt
        self.step(keyboard_input=False)
        self.state = SimStates.pick_up
        self.step(keyboard_input=False)
        self.state = SimStates.goto_bolt_hole
        self.step(keyboard_input=False)
        self.state = SimStates.orient_bolt_with_bolt_hole
        self.step(keyboard_input=False)
    # self._step_sim(500)
    # self.capture_image()
  # elif action == 'PICK_UP_BOLT_FROM_JIG':
  #   self.state = SimStates.goto_bolt
  #   self.step(keyboard_input=False)
  #   self.state = SimStates.pick_up_bolt_head
  #   self.step(keyboard_input=False)
    elif action == 'ASSEMBLE':
        self.state = SimStates.goto_bolt
        self.step(keyboard_input=False)
        self.state = SimStates.pick_up_bolt_head
        self.step(keyboard_input=False)
        self.state = SimStates.goto_nut_hole
        self.step(keyboard_input=False)
        self.state = SimStates.put_bolt_head_nut_hole
        self.step(keyboard_input=False)
    # self._step_sim(500)
    # self.capture_image()
    elif action == 'EMPTY_GRIPPER':
        self.state = SimStates.put_down
        self.step(keyboard_input=False)
    elif action == 'CLEAN_NUT':
        self.state = SimStates.goto_nut
        self.step(keyboard_input = False)
        self.state = SimStates.pick_up_from_bin
        self.step(keyboard_input = False)
        self.state = SimStates.goto_bin
        self.step(keyboard_input = False)
        self.state = SimStates.put_in_bin
        self.step(keyboard_input = False)
    elif action == 'CLEAN_BOLT':
        self.state = SimStates.orient_bolt
        self.step(keyboard_input = False)
        self.state = SimStates.pick_up_from_bin
        self.step(keyboard_input = False)
        self.state = SimStates.goto_bin
        self.step(keyboard_input = False)
        self.state = SimStates.put_in_bin
        self.step(keyboard_input = False)

def pred_label(pos, models):
    dists = []
    for model in models:
        means = model
        # covirance = model[1]
        distance = p_norm_distance(pos, means, p = -2)
        dists.append(distance)
    model_ind = np.argmin(dists)
    return model_ind

def p_norm_distance(x, y, p):
    difference = np.abs(x - y)
    return np.sum(difference ** p) ** (1 / p)

def pred_label(pos, models):
    dists = []
    for model in models:
        means = model
        # covirance = model[1]
        distance = p_norm_distance(pos, means, p=-2)
        dists.append(distance)
    model_ind = np.argmin(dists)
    return model_ind


def p_norm_distance(x, y, p):
    difference = np.abs(x - y)
    return np.sum(difference**p)**(1 / p)


def get_world_states(self):

    '''Get a binary vector with dimension m. m is the number of local states.'''
    models_nut = list(np.load('./robo_sim/models_nut.npy'))
    models_bolt = list(np.load('./robo_sim/models_bolt.npy'))
    num_local_states = len(models_nut + models_bolt)
    arr =  np.zeros(num_local_states)
    data = self.take_snapshot(None)
    for obj in data['objs']:
        pos = obj['pos']
        if obj['class'] == 'nut':
            ind = pred_label(pos, models_nut)
            arr[ind] = 1
        elif obj['class'] == 'bolt':
            ind = pred_label(pos, models_bolt)
            arr[ind + len(models_nut)] = 1
    return arr


def move_robot(self,
               target_pos,
               target_orn,
               finger_target=None,
               num_sim_steps=1,
               max_retries=300):
    """Move robot to target gripper pose."""
    if finger_target is None:
        finger_target = self.finger_target

    # Validate input to make sure that pose is attainable. If not, set to attainable pose.
    target_pos, target_orn = self.validate_pose(target_pos, target_orn)

    # Set pose for gripper fingers (open or close).
    for joint_idx in range(
        self.robot_params.first_finger_idx,
        self.robot_params.first_finger_idx + self.robot_params.num_fingers):
        self.bt.setJointMotorControl2(self.robot,
                                      joint_idx,
                                      self.bt.POSITION_CONTROL,
                                      finger_target,
                                      force=25)
    self._step_sim(1)
    self.finger_target = finger_target

    # Execute smooth motion planning sequence to get to target pos and orn.
    target_reached = self.move_gripper_smoothly(target_pos, target_orn,
                                                max_retries)
    if not target_reached:
        cur_pos, cur_orn = self.get_endeffector_real_pose()
        # print(
        #     "Failed to reach target pos: {}, orn: {}. Cur pos: {}, orn: {}. Count failed motions: {}"
        #     .format(target_pos, np.degrees(target_orn), cur_pos,
        #             np.degrees(cur_orn), self.count_failed_motions))
        self.count_failed_motions += 1
        self.count_successful_motions = 0
    else:
        self.count_successful_motions += 1
        if (self.count_successful_motions > 5):
        # If we've had a run of successful executions, then reset both counters.
            self.count_failed_motions = 0

    self._step_sim(num_sim_steps)
    pos, orn = self.get_endeffector_real_pose()
    self._set_robot_pose(pos, orn)
    return target_reached


def validate_pose(self, target_pos, target_orn):
    target_pos = self.validate_position(target_pos)
    target_orn = self.validate_orn(target_orn)
    return (target_pos, target_orn)


def move_gripper_smoothly_orn(self,
                              target_theta,
                              axis_idx,
                              delta_coord,
                              pos,
                              max_retries=500):
    cur_pos, cur_orn = self.get_endeffector_real_pose(self.robot_params)
    travel_dir = np.sign(target_theta - cur_orn[axis_idx])
    cur_delta = np.abs(target_theta - cur_orn[axis_idx])

    delta_coord = np.copy(delta_coord)
    starting_delta = np.copy(delta_coord)
    if (cur_delta < starting_delta):
        delta_coord = cur_delta / 2.0

    # Only change the specified axis coordinate. Keep other 2 axes static.
    initial_orn_static = np.copy(cur_orn)

    target_coord_reached = False
    num_iterations = 0
    while not target_coord_reached:
        if num_iterations > max_retries:
            return False
        else:
            num_iterations += 1

        if cur_delta < self.epsilon_theta:
            target_coord_reached = True
        elif cur_delta > 2 * starting_delta:
            delta_coord = starting_delta
        else:
            # Move in smaller increments as we get closer to target coordinate.
            delta_coord = cur_delta / 2.0

        new_orn = np.copy(initial_orn_static)
        new_orn[axis_idx] = (cur_orn[axis_idx] + travel_dir * delta_coord)

        new_pos, new_orn = self.validate_pose(pos, new_orn)

        if ((axis_idx == self.vertical_axis_idx) and
            (np.sign(new_orn[axis_idx]) != np.sign(target_theta))):
            # If we overshot, validator will put angle in the opposite quadrant.
            # print("old orn: ", np.degrees(new_orn))
            new_orn[axis_idx] = target_theta
            # print("before validation: ", np.degrees(new_orn))
            new_pos, new_orn = self.validate_pose(pos, new_orn)
            # print("after validation: ", np.degrees(new_orn))

        self.move_gripper(new_pos, new_orn)
        self._step_sim(10)

        cur_pos, cur_orn = self.get_endeffector_real_pose(self.robot_params)
        travel_dir = np.sign(target_theta - cur_orn[axis_idx])
        cur_delta = np.abs(target_theta - cur_orn[axis_idx])
        # print(
        #     "axis: {}, cur_orn: {}, target_theta: {}, cur_delta: {}, new_orn: {}, delta_coord: {}"
        #     .format(axis_idx, np.degrees(cur_orn), np.degrees(target_theta),
        #             np.degrees(cur_delta), np.degrees(new_orn),
        #             np.degrees(delta_coord)))

    return True


def move_gripper_smoothly_polar(self,
                                target_coord,
                                coord_type,
                                delta_coord,
                                gripper_height,
                                orn,
                                max_retries=500):
    # Stores position as a list [Radius, Theta]
    cur_pos_polar = self.convert_lists2np(
        self.get_endeffector_real_pos_polar(self.robot_params))
    travel_dir = np.sign(target_coord - cur_pos_polar[coord_type])
    cur_delta = np.abs(target_coord - cur_pos_polar[coord_type])

    # Create a copy of input since this will be modified
    starting_delta = np.copy(delta_coord)
    delta_coord = np.copy(delta_coord)
    if (cur_delta < starting_delta):
        delta_coord = cur_delta / 2.0

    target_coord_reached = False

    # If changing radius, keep theta constant. Else vice-versa.
    static_coord_type = (COORD_TYPE_THETA if
                         (coord_type == COORD_TYPE_RADIUS) else COORD_TYPE_RADIUS)
    static_coord = np.copy(cur_pos_polar[static_coord_type])
    epsilon = (self.epsilon_theta if
               (coord_type == COORD_TYPE_THETA) else self.epsilon_pos)

    num_iterations = 0
    while not target_coord_reached:
        if num_iterations > max_retries:
            return False
        else:
            num_iterations += 1

        if cur_delta < epsilon:
            target_coord_reached = True
        elif cur_delta > 2 * starting_delta:
            delta_coord = starting_delta
        else:
            # Move in smaller increments as we get closer to target coordinate.
            delta_coord = cur_delta / 2.0

        cur_pos_polar[coord_type] = (cur_pos_polar[coord_type] +
                                     travel_dir * delta_coord)
        cur_pos_polar[static_coord_type] = static_coord
        new_pos = self._compute_cart_from_polar(gripper_height,
                                                cur_pos_polar[COORD_TYPE_RADIUS],
                                                cur_pos_polar[COORD_TYPE_THETA])
        # print("moving gripper to: ", new_pos)
        new_pos, new_orn = self.validate_pose(new_pos, orn)
        self.move_gripper(new_pos, new_orn)
        self._step_sim(5)

        cur_pos_polar = self.convert_lists2np(
            self.get_endeffector_real_pos_polar(self.robot_params))
        travel_dir = np.sign(target_coord - cur_pos_polar[coord_type])
        cur_delta = np.abs(target_coord - cur_pos_polar[coord_type])

        print_radius_cur = cur_pos_polar[0]
        print_radius_theta = cur_pos_polar[1]
        print_target_coord = np.copy(target_coord)

        if coord_type == 1:
            print_target_coord = np.degrees(print_target_coord)
        # print(
        #     "coord_type: {}, cur_radius: {}, cur_theta: {}, target_coord: {}, cur_delta: {}, delta_coord: {}, new_pos: {}"
        #     .format(coord_type, print_radius_cur, print_radius_theta,
        #             print_target_coord, cur_delta, delta_coord, new_pos))

    return True


def move_gripper_smoothly_cart(self,
                               target_pos,
                               axis_idx,
                               delta_coord,
                               orn,
                               max_retries=500):
    cur_pos, cur_orn = self.get_endeffector_real_pose(self.robot_params)
    travel_dir = np.sign(target_pos[axis_idx] - cur_pos[axis_idx])
    cur_delta = np.abs(target_pos[axis_idx] - cur_pos[axis_idx])

    # Create a copy of input since this will be modified
    starting_delta = np.copy(delta_coord)
    delta_coord = np.copy(delta_coord)
    if (cur_delta < starting_delta):
        delta_coord = cur_delta / 2.0

    # Only change the specified axis coordinate. Keep other 2 axis static.
    initial_pos_static = np.copy(cur_pos)

    target_coord_reached = False
    num_iterations = 0
    while not target_coord_reached:
        if num_iterations > max_retries:
            return False
        else:
            num_iterations += 1

        if cur_delta < self.epsilon_pos:
            target_coord_reached = True
        elif cur_delta > 2 * starting_delta:
            delta_coord = starting_delta
        else:
            # Move in smaller increments as we get closer to target coordinate.
            delta_coord = cur_delta / 2.0

        new_pos = np.copy(initial_pos_static)
        new_pos[axis_idx] = (cur_pos[axis_idx] + travel_dir * delta_coord)

        # print("moving gripper to: ", new_pos)
        new_pos, new_orn = self.validate_pose(new_pos, orn)
        self.move_gripper(new_pos, new_orn)
        self._step_sim(5)

        cur_pos, cur_orn = self.get_endeffector_real_pose(self.robot_params)
        travel_dir = np.sign(target_pos[axis_idx] - cur_pos[axis_idx])
        cur_delta = np.abs(target_pos[axis_idx] - cur_pos[axis_idx])
        # print(
        #     "move cart, cur_pos: {}, target_pos: {}, diff: {}, cur_delta: {}, new_pos: {}"
        #     .format(cur_pos, target_pos, (target_pos - cur_pos), cur_delta,
        #             new_pos))

    return True


def move_gripper_smoothly(self, target_pos, target_orn, max_retries=500):
    """Executes smooth motion planning sequence between cur pose and target pose.
    Moves gripper incrementally instead of making abrupt changes. This reduces
    chance of object escaping grasp.
    Note: Motion path is not shortest path between positions (i.e. direct line).
    Currently, a parametric path is computed where the gripper travels along the
    perimeter of a circle of current radius, and then travels along radial arm.
    """
    # initial_pos, initial_orn = self._get_robot_pose()
    initial_pos, initial_orn = self.get_endeffector_real_pose()

    if np.array_equal(initial_pos, target_pos) and np.array_equal(
        initial_orn, target_orn):
        # If already at target pose
        return

    # Convert current and target positions from Cartesian to polar
    target_radius, target_theta = self._cart2polar(target_pos)
    cur_radius, cur_theta = self.get_endeffector_real_pos_polar(self.robot_params)

    # print("target_pos: {}, target_theta: {}, cur_theta: {}".format(
    #     target_pos, np.degrees(target_theta), np.degrees(cur_theta)))

    # Execute polar path planning and bring gripper to target angle.
    if not (self.move_gripper_smoothly_polar(
        target_coord=target_theta,
        coord_type=COORD_TYPE_THETA,
        delta_coord=0.1,
        gripper_height=initial_pos[self.vertical_axis_idx],
        orn=initial_orn,
        max_retries=max_retries)):
        return False
    cur_pos, cur_orn = self.get_endeffector_real_pose(self.robot_params)
    # print("after planar motion theta, cur_pos: {}, target_pos: {}".format(
    #     cur_pos, target_pos))

    # Execute polar path planning and bring gripper to target radius.
    if not (self.move_gripper_smoothly_polar(
        target_coord=target_radius,
        coord_type=COORD_TYPE_RADIUS,
        delta_coord=0.1,
        gripper_height=initial_pos[self.vertical_axis_idx],
        orn=initial_orn,
        max_retries=max_retries)):
        return False
    cur_pos, cur_orn = self.get_endeffector_real_pose(self.robot_params)
    # print("after planar motion radius, cur_pos: {}, target_pos: {}, diff: {}".
    #       format(cur_pos, target_pos, (target_pos - cur_pos)))

    intermediate_pos = np.copy(cur_pos)
    # Move gripper to target orientation.
    for idx in range(0, 3):
        cur_pos, cur_orn = self.get_endeffector_real_pose(self.robot_params)
        delta_coord = 0.2
        if idx == self.vertical_axis_idx:
            delta_coord = 0.1
        # print("before orn rotation {}, cur_orn: {}, target_orn: {}".format(
        #     idx, np.degrees(cur_orn), np.degrees(target_orn)))
        if not (self.move_gripper_smoothly_orn(target_theta=target_orn[idx],
                                               axis_idx=idx,
                                               delta_coord=delta_coord,
                                               pos=intermediate_pos,
                                               max_retries=max_retries)):
            return False
    cur_pos, cur_orn = self.get_endeffector_real_pose(self.robot_params)
    # print(
    #     "after orn rot, cur_pos: {}, target_pos: {}, cur_orn: {}, target_orn: {}".
    #     format(cur_pos, target_pos, np.degrees(cur_orn), np.degrees(target_orn)))

    # Execute cartesian path planning and bring gripper to target height.
    intermediate_orn = np.copy(cur_orn)
    if not (self.move_gripper_smoothly_cart(target_pos=target_pos,
                                            axis_idx=self.vertical_axis_idx,
                                            delta_coord=0.1,
                                            orn=intermediate_orn,
                                            max_retries=max_retries)):
        return False
    cur_pos, cur_orn = self.get_endeffector_real_pose(self.robot_params)
    # print(
    #     "after gripper height, actual_pos: {}, target_pos: {}, diff: {}, actual_orn: {}, target_orn: {}"
    #     .format(cur_pos, target_pos, (target_pos - cur_pos), np.degrees(cur_orn),
    #             np.degrees(target_orn)))

    return True


def pcontrol_motion(self, cur_pos, target_pos, target_orn, axis_idx, tolerance):
    infinite_count = 0
    while (not self._compare_coords_equal(cur_pos[axis_idx], target_pos[axis_idx],
                                          self.epsilon_pos)):
        target_pos, target_orn = self.validate_pose(target_pos, target_orn)
        self.move_gripper(target_pos, target_orn)
        cur_pos, cur_orn = self.get_endeffector_real_pose(self.robot_params)
        self._step_sim(1)
        infinite_count += 1
        if (infinite_count > 100):
            print("infinite count reached, breaking")
            break
        # print("pcontrol axis {}: actual: {}, target: {}".format(
        #     axis_idx, cur_pos, target_pos))


def _compute_cart_from_polar(self, height, radius, theta):
    new_x = radius * math.sin(theta)
    new_z = -radius * math.cos(theta)
    return [new_x, height, new_z]


def _cart2polar(self, pos):
    radius = math.sqrt(pos[0]**2 + pos[2]**2)
    theta = math.atan2(pos[0], -pos[2])
    return (radius, theta)


def move_gripper(self, target_pos, target_orn):
    """Moves gripper to target pose."""
    # # Convert orientation to quat form if passed Euler angles
    # if len(target_orn) == 3:
    #   target_orn = self.bt.getQuaternionFromEuler(target_orn)

    # Calculate joint poses using IK
    joint_poses = self.bt.calculateInverseKinematics(
        self.robot,
        self.robot_params.end_effector_idx,
        target_pos,
        self.bt.getQuaternionFromEuler(target_orn),
        self.robot_params.ll,
        self.robot_params.ul,
        self.robot_params.jr,
        self.robot_params.rp,
        maxNumIterations=20)
    # print("Joint Poses: ", joint_poses)
    # self.bt.setJointMotorControlArray(self.robot,
    #                                   list(range(self.robot_params.num_dofs)),
    #                                   self.bt.POSITION_CONTROL,
    #                                   joint_poses[:self.robot_params.num_dofs],
    #                                   forces=([7 * 240.] *
    #                                           self.robot_params.num_dofs))

    # Set joints to target poses
    for joint_idx in range(self.robot_params.num_dofs):
        self.bt.setJointMotorControl2(self.robot,
                                      joint_idx,
                                      self.bt.POSITION_CONTROL,
                                      joint_poses[joint_idx],
                                      force=7 * 240.)
    self._step_sim(1)
    self._set_robot_pose(pos=target_pos, orn=target_orn)

    joint_states = self.bt.getJointStates(self.robot,
                                          list(range(self.robot_params.num_dofs)))
    # print("Joint State: ", [state[0] for state in joint_states])


def _compare_coords_equal(self, coord1, coord2, delta_thresh):
    """Compares if two coordinates are equal, subject to provided threshold."""
    if (abs(coord1 - coord2) <= delta_thresh):
        return True
    else:
        return False


def _calc_next_coord(self, target_coord, cur_coord):
    """Computes the next coordinate gripper should move to on path between cur
    position and target position. Gripper moves in increments of self.delta_pos.
    """
    target_coord_reached = False
    next_coord = cur_coord
    delta = self.delta_pos

    if (self._compare_coords_equal(cur_coord, target_coord, delta)):
        # If already at desired pos
        target_coord_reached = True
    elif (cur_coord < target_coord):
        next_coord = cur_coord + delta
    elif (cur_coord > target_coord):
        next_coord = cur_coord - delta
    else:
        raise ValueError(
            'Current coord equals target coord, but equality check failed.')

    return (target_coord_reached, next_coord)


def validate_position(self, pos):
    """Checks if a given position is within reach of the robot arm/gripper."""
    # Magnitude of position vector
    arm_length = np.linalg.norm(pos)
    # Magnitude of the projection of position vector onto ground plane.
    radius, theta = self._cart2polar(pos)

    if (radius < self.robot_params.min_radius):
        # If the target is within the self-collision radius of robot
        return self._compute_cart_from_polar(pos[self.vertical_axis_idx],
                                             self.robot_params.min_radius, theta)
    elif arm_length <= self.robot_params.max_reach:
        # If target is within reach of robot
        return pos
    else:
        # If out of bounds, return current position
        return self._get_robot_pos()


def validate_position_mesh(self, mesh_name, pos):
    """Checks if a mesh is in a position where it can be reached by the gripper."""
    # Magnitude of position vector
    arm_length = np.linalg.norm(pos)
    # Magnitude of the projection of position vector onto ground plane.
    radius, theta = self._cart2polar(pos)

    if (radius < self.robot_params.min_radius):
        # If the target is within the self-collision radius of robot
        return False
    elif arm_length > self.robot_params.max_reach:
        # If target is outside reach of robot
        return False
    else:
        # Check if object is stuck inside jig
        if mesh_name == 'nut':
            if (pos[1] < 0.002 and (pos[0] < 0.225 and pos[0] > -0.225) and
                (pos[2] > -0.75 and pos[2] < -0.45)):
                return False
        elif mesh_name == 'bolt':
            if (pos[1] < 0.04 and (pos[0] < 0.225 and pos[0] > -0.225) and
                (pos[2] > -0.75 and pos[2] < -0.45)):
                return False

    # Mesh position is valid
    return True


def validate_orn(self, orn):
    """Checks if a given orientation is within the limits of the robot arm."""
    for idx, angle in enumerate(orn):
        if (self.robot_params.max_angles[idx] == 0):
            raise ValueError(
                'Validation failed. Max angle for axis {} set to zero.'.format(idx))
        elif idx == self.vertical_axis_idx:
            orn[idx] = angle % self.robot_params.angle_range[idx]
            if orn[idx] > self.robot_params.max_angles[idx]:
                orn[idx] = orn[idx] - self.robot_params.angle_range[idx]
        else:
            # For rotations around all other axes other than vertical.
            # Do not wrap around. Prevent robot from going beyond max.
            if orn[idx] > self.robot_params.max_angles[idx]:
                orn[idx] = self.robot_params.max_angles[idx]
            elif orn[idx] < self.robot_params.min_angles[idx]:
                orn[idx] = self.robot_params.min_angles[idx]

    return orn


def freeze_obj(self, mesh_name, copy_idx=0):
    """Makes mesh object static (immovable) by reloading it into sim."""
    return
    # pos, orn = self.bt.getBasePositionAndOrientation(self.meshes[mesh_name][copy_idx])

    # # Remove mesh from simulation.
    # self.bt.removeBody(self.meshes[mesh_name][copy_idx])

    # # Reload mesh into sim as fixed object.
    # urdf_path, *rest, flags = self.nut_mesh
    # self.meshes[mesh_name] = self.bt.loadURDF(urdf_path,
    #                                           pos,
    #                                           baseOrientation=orn,
    #                                           useFixedBase=True,
    #                                           flags=flags)


def smart_caller(self, condition, func, positional_args, keyword_args):
    """Wrapper method which only calls a function if the passed condition
    evaluates to True.
    """
    if condition:
        return func(*positional_args, **keyword_args)
    else:
        return False


def execute_pick_up(self, target_pos, target_orn):
    finger_target = 0.04  # Open finger
    prev_pos = self._get_robot_pos()

    self.move_robot(prev_pos, target_orn, finger_target, num_sim_steps=15)
    self.move_robot(target_pos, target_orn, num_sim_steps=50)
    # Now bring gripper back up holding obj
    finger_target = 0.01
    self.move_robot(target_pos, target_orn, finger_target, num_sim_steps=100)
    self.move_robot(prev_pos, target_orn, num_sim_steps=15)


def execute_put_down(self, target_pos, target_orn):
    prev_pos = self._get_robot_pos()
    self.move_robot(target_pos, target_orn, num_sim_steps=200)

    finger_target = 0.04
    self.move_robot(target_pos, target_orn, finger_target, num_sim_steps=200)

    cur_pos, cur_orn = self.get_endeffector_real_pose(self.robot_params)
    travel_dir = np.sign(cur_orn[2])
    translational_offset = np.array([0, 0, 0])
    new_pos = cur_pos - travel_dir * translational_offset
    self.move_robot(new_pos, cur_orn, num_sim_steps=50)

    new_pos[self.vertical_axis_idx] = prev_pos[self.vertical_axis_idx]
    self.move_robot(new_pos, cur_orn, num_sim_steps=50)

    self.move_robot(new_pos, self._get_default_gripper_orn(), num_sim_steps=100)


def execute_put_down_bolt(self, target_mesh_name, mesh_release_height):
    # Move gripper over target mesh.
    cur_pos, cur_orn = self.get_endeffector_real_pose(self.robot_params)
    mesh_pos = self._get_mesh_pos(target_mesh_name,
                                  height=cur_pos[self.vertical_axis_idx])
    self.move_robot(mesh_pos, cur_orn, num_sim_steps=50)

    new_orn = self._get_default_gripper_orn()
    self.move_robot(mesh_pos, new_orn, num_sim_steps=50)

    # Orient gripper so that it's vertical pointing up.
    new_orn, rot_dir = self.orient_bolt_vertical()
    self.move_robot(mesh_pos, new_orn, num_sim_steps=200)

    # Move gripper by some offset to center bolt perfectly over mesh.
    cur_pos, cur_orn = self.get_endeffector_real_pose(self.robot_params)
    bolt_pos = self._get_mesh_pos('bolt', height=cur_pos[self.vertical_axis_idx])
    translational_offset = mesh_pos - bolt_pos

    new_pos = cur_pos + translational_offset
    self.move_robot(new_pos, cur_orn, num_sim_steps=50)

    new_pos[self.vertical_axis_idx] = mesh_release_height
    self.execute_put_down(new_pos, cur_orn)
    cur_pos, cur_orn = self.get_endeffector_real_pose(self.robot_params)


def execute_put_down_bolt_head(self, target_mesh_name, bolt_release_height):
    # Move gripper over target mesh.
    cur_pos, cur_orn = self.get_endeffector_real_pose(self.robot_params)
    mesh_pos = self._get_mesh_pos(target_mesh_name,
                                  height=cur_pos[self.vertical_axis_idx])
    self.move_robot(mesh_pos, cur_orn, num_sim_steps=50)

    new_orn, rot_dir = self.orient_bolt_vertical()
    self.move_robot(mesh_pos, new_orn, num_sim_steps=200)


def orient_bolt_vertical(self):
    """Orients gripper such that the bolt shaft is parallel with vertical axis."""
    # bolt_principal, bolt_pos = self.get_mesh_principal_axis('bolt')

    bolt_pos, bolt_orn = self.get_mesh_pose('bolt')
    gripper_pos, gripper_orn = self.get_endeffector_real_pose(self.robot_params)
    bolt_axis, bolt_pos = self.get_mesh_principal_axis('bolt')
    # alpha = math.atan2()
    gamma = math.atan2(bolt_axis[0], bolt_axis[1])
    # print("\n\n\n\n\n\nBolt orn: ", np.degrees(bolt_orn), 'gamma: ',
    #       np.degrees(gamma))
    # print("Gripper orn: ", np.degrees(gripper_orn))
    # print("\n\n\n\n\n")

    gripper_orn[2] = gripper_orn[2] + gamma
    return (gripper_orn, np.sign(gamma))


def orient_gripper_bolt(self):
    """Orients gripper fingers perpendicular to the principal axis of a bolt."""
    bolt_principal, bolt_pos = self.get_mesh_principal_axis('bolt')
    bolt_ground_proj, poi = self.get_mesh_ground_plane_proj(
        bolt_principal, bolt_pos)

    finger_interior_normal, finger_pos = self.get_finger_interior_normal(
        self.robot_params)
    finger_ground_proj, poi = self.get_mesh_ground_plane_proj(
        finger_interior_normal, finger_pos)

    # print("bolt_ground: {}, finger_ground: {}".format(bolt_ground_proj,
    #                                                   finger_ground_proj))

    # theta = self.compute_vec2vec_angle(finger_ground_proj, bolt_ground_proj)
    # theta_finger = self.compute_vec2vec_angle(finger_ground_proj, [0, 0, 1])
    # theta_bolt = self.compute_vec2vec_angle(bolt_ground_proj, [0, 0, 1])

    atan2_range = math.pi  # atan2 range is from [-pi, pi]
    theta_finger = self.compute_vec_angle(finger_ground_proj)
    theta_bolt = self.compute_vec_angle(bolt_ground_proj)

    # print("\n")
    # print("bolt_principal: {}, bolt_proj: {}, bolt_theta: {}".format(
    #     bolt_principal, bolt_ground_proj, np.degrees(theta_bolt)))
    # print("finger: {}, finger_proj: {}, finger_theta: {}".format(
    #     finger_interior_normal, finger_ground_proj, np.degrees(theta_finger)))

    orn = self._get_robot_orn()
    gripper_pos, gripper_orn = self.get_endeffector_real_pose(self.robot_params)
    travel_dir_sign = 1.0
    if (theta_bolt > (math.pi / 2)):
        travel_dir_sign = -1.0

    # print("cur_orn: {}, travel_dir: {}, actual_orn: {}".format(
    #     np.degrees(orn), travel_dir_sign, np.degrees(gripper_orn)))

    theta_diff = math.pi / 2
    theta_new = theta_bolt + (travel_dir_sign * theta_diff)
    theta_real = theta_new + (gripper_orn[self.vertical_axis_idx] - theta_finger)
    orn[self.vertical_axis_idx] = theta_new
    # print(
    #     "theta_diff: {}, theta_new: {}, theta_real: {}, new_orn: {}, initial_theta: {}"
    #     .format(np.degrees(theta_diff), np.degrees(theta_new),
    #             np.degrees(theta_real), orn,
    #             np.degrees(self.gripper_initial_theta)))

    # orn[self.vertical_axis_idx] = orn[
    #     self.vertical_axis_idx] + travel_dir_sign * theta_left
    # print("theta: {}, theta_finger: {}, theta_bolt: {}".format(
    #     np.degrees(theta),
    #     np.degrees(math.atan2(finger_ground_proj[2], finger_ground_proj[0])),
    #     np.degrees(math.atan2(bolt_ground_proj[2], bolt_ground_proj[0]))))
    # theta_new = (math.pi / 2) - theta
    bolt_pos, bolt_principal = self.convert_lists2np(bolt_pos, bolt_principal)
    bolt_pos = bolt_pos - 0.01 * bolt_principal
    bolt_pos[self.vertical_axis_idx] = self._get_gripper_height()

    bolt_pos, orn = self.validate_pose(bolt_pos, orn)
    # print("Setting orn to: {}".format(np.degrees(orn)))

    # if (theta_new < 0):
    #   theta_new = theta_new + math.pi

    # orn[1] = orn[1] + theta_new
    # print("theta: {}, theta_new: {}, prev_orn: {}, new_orn: {}".format(
    #     np.degrees(theta), np.degrees(theta_new),
    #     np.degrees(self._get_robot_orn()), np.degrees(orn)))

    self.move_robot(bolt_pos, orn)

    finger_interior_normal, finger_pos = self.get_finger_interior_normal(
        self.robot_params)
    finger_ground_proj_new, poi = self.get_mesh_ground_plane_proj(
        finger_interior_normal, finger_pos)
    theta_finger_new = self.compute_vec_angle(finger_ground_proj_new)

    # print("old_finger_theta: {}, bolt_theta: {}, new_finger_theta: {}\n".format(
    #     np.degrees(theta_finger), np.degrees(theta_bolt),
    #     np.degrees(theta_finger_new)))


def _get_gripper_height(self):
    return self.prev_pos[self.vertical_axis_idx]


def _get_robot_pos(self):
    return np.copy(self.prev_pos)


def _get_robot_orn(self):
    return np.copy(self.prev_orn)


def _get_robot_pose(self):
    """Returns a copy of the robot pose as a tuple of (pos, orn)."""
    # return (self.get_endeffector_real_pose())
    # self._set_robot_pose(pos, orn)
    return (self._get_robot_pos(), self._get_robot_orn())


def _get_robot_pos_polar(self):
    return (np.copy(self.radius), np.copy(self.theta))


def _set_robot_pose(self, pos, orn):
    self.prev_pos = np.copy(pos)
    self.prev_orn = np.copy(orn)
    self.radius, self.theta = self._cart2polar(pos)


def _step_sim(self, num_steps):
    for i in range(num_steps):
        self.bt.stepSimulation()


def get_finger_dist(self, robot_params):
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
    dist = np.sqrt(sum((finger2_pos - finger1_pos)**2))

    return dist


def _is_there_nut_in_jig(self):
    for kind in self.meshes:
        if kind == 'nut':
            for i in range(len(self.meshes[kind])):
                idx = self.meshes[kind][i]
                mesh_pos, mesh_orn = self.bt.getBasePositionAndOrientation(idx)
                if mesh_pos[1] < 0.06 and mesh_pos[1] > 0.002:
                    return True
                else:
                    return False


def _is_there_bolt_in_jig(self):
    bolt_hole_pos, bolt_hole_orn = self.bt.getBasePositionAndOrientation(
        self.meshes['bolt_hole'][0])
    for kind in self.meshes:
        if kind == 'bolt':
            for i in range(len(self.meshes[kind])):
                idx = self.meshes[kind][i]
                mesh_pos, mesh_orn = self.bt.getBasePositionAndOrientation(idx)
                if mesh_pos[1] <= 0.09 and mesh_pos[1] > 0.04 and abs(
                    mesh_pos[0] - bolt_hole_pos[0]) < 0.03:
                    return True
                else:
                    return False


def _get_key(val, my_dict):
    for key, value in my_dict.items():
        if val == value:
            return key

    return "key doesn't exist"

def _is_in_bin(self, obj, bin, r):
    '''
    Check if an item is in a certain bin.

    obj: index of the object to check.
    bin: index of the bin to check.

    return: True if the object is in the bin.
    '''
    pos_bin,_ = self.bt.getBasePositionAndOrientation(bin)
    pos_obj,_ = self.bt.getBasePositionAndOrientation(obj)
    # The object center of the bin is 0.15 off in the x direction
    offset = 0.15
    dist_to_bin = np.sqrt((pos_obj[0] - (pos_bin[0] - offset))**2 + (pos_obj[2] - pos_bin[2])**2)
    # print(obj,pos_obj, pos_bin, dist_to_bin)
    if dist_to_bin < r:
        # The radius of the bowl is greater than 0.4 but it is hard to grasp at the edge.
        return True
    else:
        return False
