from ._requirements import *
from .sim_states import SimStates
from .world import World, Thing
from .objects import Nut, Bolt, Jig, Gripper


class RoboSim(object):
  """Wrapper around a PyBullet environment that provides an interface
  for robotic grasping simulation.

  Simulation can execute specific pre-planned actions but can also be modified
  to create custom action sequences. This is done by passing desired gripper
  pose (position, orientation) to move_robot().

  Attributes:
  Note: All attributes labelled CONST should not change after initialization.
    bt:
      Bullet client object; this is the internal physics engine.
    robot_params (CONST):
      Struct which stores configuration parameters of robot in simulation. See
      'robot_params.py' for example on what struct layout should look like.
    robot_pos (CONST):
      Position of robot base link.
    robot_orn (CONST):
      Orientation of robot base link.
    state:
      Current state of simulation. This is modified every time an action is
      executed or state transition occurs.
    finger_target:
      Distance between fingers of endeffector. This is modified every time
      gripper opens or closes.
    vertical_axis_idx (CONST):
      Value of index corresponding to vertical axis in [x,y,z] space.
    delta_pos (CONST):
      The position delta by which the gripper is moved at each timestep.
    delta_theta (CONST):
      The angular delta by which the gripper is moved at each timestep.
    epsilon_pos (CONST):
      The tolerance (in metres) used to determine translational pose similarity.
    epsilon_theta (CONST):
      The tolerance (in rad) used to determine rotational pose similarity.
    shift_key_pressed:
      Stores state of whether or not a shift key press was recorded.
    sim_start_time (CONST):
      Stores the time when the simulation was started (used for timestamps).
    out_data_dir (CONST):
      Stores path to directory where output data will be logged.
    out_data_dir_created:
      Stores state of whether or not output data directory has been created.
    saved_img_id:
      Unique, monotonic ID used for labelling captured images.
    world:
      A symbolic environment wrapper that the planner can interact with.

  """

  def __init__(self, bullet_client, robot_params, robot_state, meshes, out_dir):
    self.bt = bullet_client
    self.bt.setPhysicsEngineParameter(solverResidualThreshold=0)

    # Defines all configurations for this specific robot
    self.robot_params = robot_params
    # self.models_nut = np.load('models_nut.npy')
    # self.models_bolt = np.load('models_bolt.npy')
    # Position and orientation of robot base
    self.robot_pos = np.array(robot_state[0])
    self.robot_orn = np.array(robot_state[1])

    # Initial state of simulation
    self.state = SimStates.ready
    # Initial distance between fingers of endeffector
    self.finger_target = 0

    # Set vertical axis to y-axis (positive y-axis points up)
    self.vertical_axis_idx = 1
    # 2mm delta used for trans/rotational motion
    self.delta_pos = 0.002
    # 15 deg theta used for trans/rotational motion
    self.delta_theta = (math.pi * 15.0) / 180.0
    # 0.5mm tolerance used for validating translational pose similarity
    self.epsilon_pos = 0.0005
    # 0.5deg tolerance used for validating rotational pose similarity
    self.epsilon_theta = 0.5 * math.pi / 180.0
    self.shift_key_pressed = False

    # Capture sim start time for use as logging timestamp
    self.sim_start_time = int(time.time())
    # Create unique dir for storing output data based on timestamp
    self.out_data_dir = out_dir + str(self.sim_start_time)
    # Directory will not be created until the first image gets captured
    # (prevents unnecessary OS calls)
    self.out_data_dir_created = False
    # Set initial ID to 0 (counts up)
    self.saved_img_id = 0
    # Improves loading performance for files with similar graphics assets
    self.flags = (self.bt.URDF_ENABLE_CACHED_GRAPHICS_SHAPES)

    self.objects_to_record = ['nut', 'bolt', 'nut_hole', 'bolt_hole']
    self.data = []
    # Unique, positively increasing integer ID used to represent order of
    # state snapshots in logged data. Resets back to 0 when a new sim begins.
    self.snapshot_id = 0
    # Unique, positively increasing integer ID used to keep track of how many
    # successful autonomous assembly sequences have been executed.
    self.seq_id = 0
    # Stores the current state the planner thinks the world is in.
    self.planner_action = 'None'
    # Keeps track of number of times sim was unable to reach target pose.
    # If thresholds are exceeded, watchdog will automatically reset simulation.
    self.count_failed_motions = 0
    self.count_successful_motions = 0

    # A symbolic environment wrapper that the planner can interact with
    self.world = World()
    # The id of the target object
    self.target_obj = None
    # Load mesh objects into the simulation
    self.mesh_params = meshes
    self._load_meshes(meshes, self.flags)

    # Reset robot orientation to predefined pose
    self._init_joints(robot_params.initial_joint_pos)

  def reset(self, log_data=True):
    """Resets simulation back to original state and reloads objects into the
    environment.
    """
    if log_data:
      self.log_data()
    else:
      self.data = []
      self.snapshot_id = 0

    self.count_failed_motions = 0
    self.count_successful_motions = 0

    # Removes all objects from simulation
    self._remove_meshes()
    # Reloads all objects into simulation
    self._load_meshes(self.mesh_params, self.flags)

    # Reset robot orientation to predefined pose
    self._init_joints(self.robot_params.initial_joint_pos)
    print("Reset to initial joint config")

    # Reset state back to ready
    self.state = SimStates.ready

  #Imported methods
  from ._sim_setup import (_load_meshes, _remove_meshes, _init_joints,
                           _constrain_fingers_centerered,
                           get_endeffector_real_pose,
                           get_finger_interior_normal,
                           get_endeffector_real_pos_polar)
  from .sim_math import (compute_vec2plane_intersection,
                         compute_vec2plane_projection, convert_lists2np,
                         normalize_vector, compute_vec2vec_projection,
                         compute_vec2vec_angle, compute_vec_angle)
  from ._sim_input import (_get_keyboard_input, _process_state, _get_mesh_pos,
                           _get_default_gripper_orn, _get_default_gripper_pos,
                           _get_mesh_orn, _align_orns)
  from ._sim_output import capture_image, _save_image, _verify_dir_exists
  from ._sim_debug import (print_quat, visualize_poses, draw_line,
                           get_mesh_principal_axis, get_mesh_ground_plane_proj,
                           get_mesh_pose, draw_robot_limits, draw_circle)
  from ._sim_data_handler import (_update_snapshot, record_snapshot, take_snapshot, dump_data,
                                  log_data)
  from .sim_main import (
      step, get_world_states, _is_there_nut_in_jig, _is_there_bolt_in_jig,
      move_robot, move_gripper_smoothly, move_gripper, assemble, clean, excute_action,
      get_finger_dist, _compare_coords_equal, _calc_next_coord,
      validate_position, validate_orn, freeze_obj, execute_pick_up,
      execute_put_down, orient_gripper_bolt, _set_robot_pose, _get_robot_pos,
      _get_robot_orn, _get_robot_pose, _get_gripper_height, _step_sim,
      validate_pose, _cart2polar, _get_robot_pos_polar,
      _compute_cart_from_polar, pcontrol_motion, move_gripper_smoothly_polar,
      move_gripper_smoothly_cart, move_gripper_smoothly_orn,
      orient_bolt_vertical, execute_put_down_bolt, execute_put_down_bolt_head,
      run_watchdog, run_watchdog_bin_picking, validate_position_mesh)
