from enum import IntEnum


class SimStates(IntEnum):
  """Simple struct to store mapping between keyboard inputs and simulation state."""
  ready = 0

  x_pos = ord('x')
  x_neg = ord('X')
  y_pos = ord('y')
  y_neg = ord('Y')
  z_pos = ord('z')
  z_neg = ord('Z')

  rot_x_pos = ord('i')
  rot_x_neg = ord('I')
  rot_y_pos = ord('j')
  rot_y_neg = ord('J')
  rot_z_pos = ord('k')
  rot_z_neg = ord('K')

  goto_nut = ord('n')
  goto_bolt = ord('b')
  goto_nut_hole = ord('N')
  goto_bolt_hole = ord('B')
  goto_bin = ord('m')

  pick_up = ord('h')
  put_down = ord('H')
  pick_up_bolt_head = ord('q')
  put_bolt_head_nut_hole = ord('Q')
  orient_bolt = ord('t')
  orient_bolt_with_bolt_hole = ord('T')
  orient_nut = ord('u')
  pick_up_from_bin = ord('e')
  put_in_bin = ord('E')

  gripper_open = ord('o')
  gripper_close = ord('c')

  capture_image = ord('p')
  reset = ord('r')
  visualize_pose = ord('d')
  default_pose = ord('D')

  scan_world = ord('s')
  assemble = ord('A')
  clean = ord('C')
