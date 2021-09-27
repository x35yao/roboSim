from .hrr import bind, normalize, random_vectors, CleanupMemory
import numpy as np
from .world import *

def get_new_world(n):
    world = World()
    Gripper(world,0)
    Jig(world,0)
    for i in range(n):
        Nut(world, i)
        Bolt(world, i)
    return world

def build_cms(dim=512, noise=0):
  noise_std = noise * np.sqrt(1 / dim)
  basics = [
      'NUT', 'BOLT', 'TABLE', 'GRIPPER', 'BIN', 'LOCATION', 'ASSEMBLE_STATE',
      'LOAD_STATE', 'TARGET', 'ASSEMBLED', 'NOT_READY', 'PRE', 'EMPTY',
      'LOADED', 'READY', 'STUCK', 'BLOCKED', 'OBJECT', 'ACTION', 'NONE', 'JIG',
      'NUT_SLOT_BLOCKED', 'NUT_SLOT_LOADED', 'NUT_SLOT_EMPTY',
      'BOLT_SLOT_BLOCKED', 'BOLT_SLOT_LOADED', 'BOLT_SLOT_EMPTY'
  ]

  states = [
      'BOLT_NUT_IN_BIN', 'BOLT_NUT_IN_GRIPPER', 'BOLT_NUT_ASSEMBLED',
      'BOLT_IN_JIG', 'BOLT_IN_GRIPPER', 'BOLT_READY_TO_ASSEMBLE',
      'BOLT_ON_TABLE', 'NUT_READY_TO_ASSEMBLE', 'NUT_IN_GRIPPER',
      'NUT_ON_TABLE', 'GRIPPER_EMPTY', 'GRIPPER_LOADED', 'JIG_NUT_SLOT_EMPTY',
      'JIG_NUT_SLOT_BLOCKED', 'JIG_NUT_SLOT_LOADED', 'JIG_BOLT_SLOT_EMPTY',
      'JIG_BOLT_SLOT_BLOCKED', 'JIG_BOLT_SLOT_LOADED', 'BOLT_NUT_STUCK'
  ]

  actions = [ 'ASSEMBLE', 'PUT_BOLT_IN_JIG', 'PUT_NUT_IN_JIG' ]

  ID = basics + states + actions
  cm_ID = CleanupMemory(ID, dim, noise_std)
  cm_SP = CleanupMemory([], dim, noise_std)
  return cm_ID, cm_SP


def build_state(item, locations, assemble_state, cm_ID, noise_std):
  '''
    item: nut or bolt
    locations: location of the item
    assemble_state: 'READY', 'NOT_READY', 'ASSEMBLED', 'STUCK'
    '''
  sum_of_location_vecs = 0
  for i in locations:
    sum_of_location_vecs += cm_ID.get(i)
  state_built = bind(cm_ID.get(item),
                     sum_of_location_vecs + cm_ID.get(assemble_state),
                     noise_std)
  return state_built


def add_state_action(cm_ID, cm_SP, noise_std):

  bolt_nut_in_bin = build_state('BOLT', ['BIN'], 'ASSEMBLED', cm_ID, noise_std)
  bolt_nut_in_gripper = build_state('BOLT', ['GRIPPER'], 'ASSEMBLED', cm_ID,
                                    noise_std)
  bolt_nut_assembled = build_state('BOLT', ['TABLE', 'NUT'], 'ASSEMBLED', cm_ID,
                                   noise_std)
  bolt_nut_stuck = build_state('NUT', ['JIG'], 'STUCK', cm_ID, noise_std)
  bolt_in_jig = build_state('BOLT', ['JIG'], 'READY', cm_ID, noise_std)
  bolt_ready_to_assemble = build_state('BOLT', ['GRIPPER'], 'READY', cm_ID,
                                       noise_std)
  bolt_in_gripper = build_state('BOLT', ['GRIPPER'], 'NOT_READY', cm_ID,
                                noise_std)
  bolt_on_table = build_state('BOLT', ['TABLE'], 'NOT_READY', cm_ID, noise_std)
  nut_ready_to_assemble = build_state('NUT', ['JIG'], 'READY', cm_ID, noise_std)
  nut_in_gripper = build_state('NUT', ['GRIPPER'], 'NOT_READY', cm_ID,
                               noise_std)
  nut_on_table = build_state('NUT', ['TABLE'], 'NOT_READY', cm_ID, noise_std)

  gripper_empty = bind(cm_ID.get('GRIPPER'),
                       cm_ID.get('EMPTY'),
                       noise_std=noise_std)
  gripper_loaded = bind(cm_ID.get('GRIPPER'),
                        cm_ID.get('LOADED'),
                        noise_std=noise_std)
  jig_nut_slot_loaded = bind(cm_ID.get('JIG'),
                             cm_ID.get('NUT_SLOT_LOADED'),
                             noise_std=noise_std)
  jig_nut_slot_empty = bind(cm_ID.get('JIG'),
                            cm_ID.get('NUT_SLOT_EMPTY'),
                            noise_std=noise_std)
  jig_nut_slot_blocked = bind(cm_ID.get('JIG'),
                              cm_ID.get('NUT_SLOT_BLOCKED'),
                              noise_std=noise_std)
  jig_bolt_slot_loaded = bind(cm_ID.get('JIG'),
                              cm_ID.get('BOLT_SLOT_LOADED'),
                              noise_std=noise_std)
  jig_bolt_slot_empty = bind(cm_ID.get('JIG'),
                             cm_ID.get('BOLT_SLOT_EMPTY'),
                             noise_std=noise_std)
  jig_bolt_slot_blocked = bind(cm_ID.get('JIG'),
                               cm_ID.get('BOLT_SLOT_BLOCKED'),
                               noise_std=noise_std)

  cm_SP.add('BOLT_NUT_IN_BIN', vector=bolt_nut_in_bin)
  cm_SP.add('BOLT_NUT_IN_GRIPPER', vector=bolt_nut_in_gripper)
  cm_SP.add('BOLT_NUT_ASSEMBLED', vector=bolt_nut_assembled)
  cm_SP.add('BOLT_IN_JIG', vector=bolt_in_jig)
  cm_SP.add('BOLT_READY_TO_ASSEMBLE', vector=bolt_ready_to_assemble)
  cm_SP.add('BOLT_IN_GRIPPER', vector=bolt_in_gripper)
  cm_SP.add('BOLT_ON_TABLE', vector=bolt_on_table)
  cm_SP.add('NUT_READY_TO_ASSEMBLE', vector=nut_ready_to_assemble)
  cm_SP.add('NUT_IN_GRIPPER', vector=nut_in_gripper)
  cm_SP.add('NUT_ON_TABLE', vector=nut_on_table)
  cm_SP.add('GRIPPER_EMPTY', vector=gripper_empty)
  cm_SP.add('GRIPPER_LOADED', vector=gripper_loaded)
  cm_SP.add('JIG_NUT_SLOT_EMPTY', vector=jig_nut_slot_empty)
  cm_SP.add('JIG_NUT_SLOT_LOADED', vector=jig_nut_slot_loaded)
  cm_SP.add('JIG_NUT_SLOT_BLOCKED', vector=jig_nut_slot_blocked)
  cm_SP.add('JIG_BOLT_SLOT_EMPTY', vector=jig_bolt_slot_empty)
  cm_SP.add('JIG_BOLT_SLOT_LOADED', vector=jig_bolt_slot_loaded)
  cm_SP.add('JIG_BOLT_SLOT_BLOCKED', vector=jig_bolt_slot_blocked)
  cm_SP.add('BOLT_NUT_STUCK', vector=bolt_nut_stuck)

  put_bolt_nut_in_bin = bind(cm_ID.get('PRE'), cm_ID.get('BOLT_NUT_IN_GRIPPER'),
                             noise_std)
  pick_up_bolt_nut = bind(cm_ID.get('PRE'), cm_ID.get('BOLT_NUT_ASSEMBLED'),
                          noise_std)
  assemble = bind(
      cm_ID.get('PRE'), 0.4 * cm_ID.get('BOLT_IN_JIG') +
      0.6 * cm_ID.get('NUT_READY_TO_ASSEMBLE'), noise_std)
  # pick_up_bolt_from_jig = bind(cm_ID.get('PRE'), cm_ID.get('BOLT_IN_JIG'),
  #                              noise_std)
  put_bolt_in_jig = bind(
      cm_ID.get('PRE'), 0.45 * cm_ID.get('BOLT_ON_TABLE') +
      0.55 * cm_ID.get('JIG_BOLT_SLOT_EMPTY'), noise_std)
  # pick_up_bolt_from_table = bind(
  #     cm_ID.get('PRE'),
  #     0.45 * cm_ID.get('BOLT_ON_TABLE') + 0.55 * cm_ID.get('GRIPPER_EMPTY'),
  #     noise_std)
  put_nut_in_jig = bind(
      cm_ID.get('PRE'), 0.45 * cm_ID.get('NUT_ON_TABLE') +
      0.55 * cm_ID.get('JIG_NUT_SLOT_EMPTY'), noise_std)
  # pick_up_nut_from_table = bind(
  #     cm_ID.get('PRE'),
  #     0.45 * cm_ID.get('NUT_ON_TABLE') + 0.55 * cm_ID.get('GRIPPER_EMPTY'),
  #     noise_std)
  # empty_gripper = bind(cm_ID.get('PRE'), cm_ID.get('GRIPPER_LOADED'), noise_std)
  empty_jig_for_nut = bind(cm_ID.get('PRE'), cm_ID.get('JIG_NUT_SLOT_BLOCKED'),
                           noise_std)
  empty_jig_for_bolt = bind(cm_ID.get('PRE'),
                            cm_ID.get('JIG_BOLT_SLOT_BLOCKED'), noise_std)
  disassemble = bind(cm_ID.get('PRE'), cm_ID.get('BOLT_NUT_STUCK'), noise_std)

  cm_SP.add('PUT_BOLT_NUT_IN_BIN', put_bolt_nut_in_bin)
  cm_SP.add('PICK_UP_BOLT_NUT', pick_up_bolt_nut)
  cm_SP.add('ASSEMBLE', assemble)
  # cm_SP.add('PICK_UP_BOLT_FROM_JIG', pick_up_bolt_from_jig)
  cm_SP.add('PUT_BOLT_IN_JIG', put_bolt_in_jig)
  # cm_SP.add('PICK_UP_BOLT_FROM_TABLE', pick_up_bolt_from_table)
  cm_SP.add('PUT_NUT_IN_JIG', put_nut_in_jig)
  # cm_SP.add('PICK_UP_NUT_FROM_TABLE', pick_up_nut_from_table)
  # cm_SP.add('EMPTY_GRIPPER', empty_gripper)
  cm_SP.add('EMPTY_JIG_FOR_NUT', empty_jig_for_nut)
  cm_SP.add('EMPTY_JIG_FOR_BOLT', empty_jig_for_bolt)
  cm_SP.add('DISASSEMBLE', disassemble)


def initial_conditions(world, case):
  if case == 'case1':
    # Nomal case
    pass
  elif case == 'case2':
    # Assembled on table
    world.things[2].set_state('ASSEMBLED')
    world.things[3].set_state('ASSEMBLED')
  elif case == 'case3':
    # Assembled in jig
    world.things[2].set_state('ASSEMBLED')
    world.things[2].set_state('JIG')
    world.things[3].set_state('ASSEMBLED')
    world.things[3].set_state('NUT')
  elif case == 'case4':
    # Before assemble
    world.things[0].set_state('LOADED')
    world.things[1].set_state('NUT_SLOT_LOADED')
    world.things[2].set_state('JIG')
    world.things[2].set_state('READY')
    world.things[3].set_state('GRIPPER')
    world.things[3].set_state('READY')
  elif case == 'case5':
    # Bolt in gripper
    world.things[0].set_state('LOADED')
    world.things[3].set_state('GRIPPER')
  elif case == 'case6':
    # Nut in jig and gripper
    world.things[0].set_state('LOADED')
    world.things[1].set_state('NUT_SLOT_LOADED')
    world.things[2].set_state('JIG')
    world.things[2].set_state('READY')
    world.things[4].set_state('GRIPPER')
  elif case == 'case7':
    # Jig nut slot is blocked
    world.things[1].set_state('NUT_SLOT_BLOCKED')
    world.things[2].set_state('JIG')
  elif case == 'case8':
    # Bolt and nut stuck
    world.things[1].set_state('NUT_SLOT_LOADED')
    world.things[2].set_state('STUCK')
    world.things[2].set_state('JIG')
    world.things[3].set_state('STUCK')
    world.things[3].set_state('NUT')
  elif case == 'case9':
    # Mount with 0.8 chance to block the jig
    for thing in world.things:
      if isinstance(thing, Nut):
        thing.possibility = 0.8
  elif case == 'case10':
    # Assemble with 0.8 chance to stuck
    for thing in world.things:
      if isinstance(thing, Bolt):
        thing.possibility = 0.8
  elif case == 'case11':
    # Mount with 0.8 chance to block the jig and Assemble with 0.8 chance to stuck
    for thing in world.things:
      if isinstance(thing, Bolt):
        thing.possibility = 0.8
    for thing in world.things:
      if isinstance(thing, Nut):
        thing.possibility = 0.8
