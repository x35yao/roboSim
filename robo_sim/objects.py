"""
things classes with possible states and actions.
"""

from .world import World, Thing
import random


class Nut(Thing):

  def __init__(self, world, id):
    possible_states = {
        'LOCATION': ('TABLE', 'GRIPPER', 'JIG', 'BIN'),
        'ASSEMBLE_STATE': ('NOT_READY', 'READY', 'ASSEMBLED', 'STUCK')
    }
    Thing.__init__(self,
                   world,
                   'NUT' + str(id), ['NUT'],
                   possible_states=possible_states)
    self.gripper = [
        thing for thing in world.things if isinstance(thing, Gripper)
    ][0]
    self.jig = [thing for thing in world.things if isinstance(thing, Jig)][0]

  def pick_up(self, success_rate=1):
    if self.get_state()['LOCATION'] == 'TABLE':
      if random.random() < success_rate:
        self.set_state('GRIPPER')
        self.gripper.set_state('LOADED')

  def put_on_table(self, success_rate=1):
    if self.get_state()['LOCATION'] == 'GRIPPER':
      if random.random() < success_rate:
        self.set_state('TABLE')
        self.gripper.set_state('EMPTY')

  def put_in_jig(self, success_rate=1):
    if self.get_state()['LOCATION'] == 'GRIPPER':
      if random.random() < success_rate:
        self.jig.set_state('NUT_SLOT_LOADED')
        self.set_state('READY')
      else:
        self.jig.set_state('NUT_SLOT_BLOCKED')
      self.set_state('JIG')
      self.gripper.set_state('EMPTY')

  def disassemble(self, success_rate=1):
    self.attached_bolt = self._get_bolt_attached()
    if self.get_state()['ASSEMBLE_STATE'] == 'STUCK' and self.get_state(
    )['LOCATION'] == 'JIG':
      if random.random() < success_rate:
        self.set_state('READY')
        self.attached_bolt.set_state('READY')
        self.attached_bolt.set_state('GRIPPER')
        self.gripper.set_state('LOADED')

  def _get_bolt_attached(self):
    result = None
    for thing in self.world.things:
      if isinstance(thing, Bolt) and thing.get_state()['LOCATION'] == 'NUT':
        result = thing
        break
    return result


class Bolt(Thing):

  def __init__(self, world, id):
    self.world = world
    self.attached_nut = None
    possible_states = {
        'LOCATION': ('TABLE', 'GRIPPER', 'NUT', 'BIN', 'JIG'),
        'ASSEMBLE_STATE': ('NOT_READY', 'READY', 'ASSEMBLED', 'STUCK')
    }
    Thing.__init__(self,
                   world,
                   'BOLT' + str(id), ['BOLT'],
                   possible_states=possible_states)
    self.gripper = [
        thing for thing in world.things if isinstance(thing, Gripper)
    ][0]
    self.jig = [thing for thing in world.things if isinstance(thing, Jig)][0]

  def pick_up(self, success_rate=1):
    if self.get_state()['LOCATION'] == 'TABLE':
      if random.random() < success_rate:
        self.set_state('GRIPPER')
        self.gripper.set_state('LOADED')

  def put_on_table(self, success_rate=1):
    if self.get_state()['LOCATION'] == 'GRIPPER':
      if random.random() < success_rate:
        self.set_state('TABLE')
        self.gripper.set_state('EMPTY')

  def put_in_jig(self, success_rate=1):
    if self.get_state()['LOCATION'] == 'GRIPPER':
      if random.random() < success_rate:
        self.jig.set_state('BOLT_SLOT_LOADED')
      else:
        self.jig.set_state('BOLT_SLOT_BLOCKED')
      self.set_state('JIG')
      self.gripper.set_state('EMPTY')

  def pick_up_from_jig(self, success_rate=1):
    if self.get_state()['LOCATION'] == 'JIG' and self.jig.get_state(
    )['BOLT_SLOT'] == 'BOLT_SLOT_LOADED':
      if random.random() < success_rate:
        self.set_state('GRIPPER')
        self.set_state('READY')
        self.gripper.set_state('LOADED')
        self.jig.set_state('BOLT_SLOT_EMPTY')

  def assemble(self, success_rate=1):
    self.attached_nut = self._get_nut_in_jig()
    if self.get_state(
    )['ASSEMBLE_STATE'] == 'READY' and self.attached_nut is not None:
      if random.random() < success_rate:
        self.set_state('ASSEMBLED')
        self.attached_nut.set_state('ASSEMBLED')
      else:
        self.set_state('STUCK')
        self.attached_nut.set_state('STUCK')
      self.set_state('NUT')
      self.gripper.set_state('EMPTY')

  def disassemble(self, success_rate=1):
    self.attached_nut = self._get_nut_in_jig()
    if self.get_state()['ASSEMBLE_STATE'] == 'STUCK' and self.get_state(
    )['LOCATION'] == 'NUT':
      if random.random() < success_rate:
        self.set_state('GRIPPER')
        self.set_state('READY')
        self.attached_nut.set_state('READY')
        self.gripper.set_state('LOADED')

  def pick_up_bolt_nut(self, success_rate=1):
    self.attached_nut = self._get_nut_attached()
    if (self.get_state()['LOCATION'] in [
        'NUT', 'TABLE'
    ]) and (self.get_state()['ASSEMBLE_STATE'] == 'ASSEMBLED'):
      if random.random() < success_rate:
        self.set_state('GRIPPER')
        self.attached_nut.set_state('GRIPPER')
        self.attached_nut.jig.set_state('NUT_SLOT_EMPTY')
        self.gripper.set_state('LOADED')

  def put_in_bin(self, success_rate=1):
    self.attached_nut = self._get_nut_attached()
    if (self.get_state()['LOCATION']
        == 'GRIPPER') and (self.get_state()['ASSEMBLE_STATE'] == 'ASSEMBLED'):
      if random.random() < success_rate:
        self.set_state('BIN')
        self.attached_nut.set_state('BIN')
        self.gripper.set_state('EMPTY')

  def _get_nut_in_jig(self):
    result = None
    for thing in self.world.things:
      if isinstance(thing, Nut) and thing.get_state()['LOCATION'] == 'JIG':
        result = thing
        break
    return result

  def _get_nut_attached(self):
    result = None
    for thing in self.world.things:
      if isinstance(thing,
                    Nut) and thing.get_state()['ASSEMBLE_STATE'] == 'ASSEMBLED':
        result = thing
        break
    return result


class Gripper(Thing):

  def __init__(self, world, id):
    possible_states = {'LOAD_STATE': ('EMPTY', 'LOADED')}
    Thing.__init__(self,
                   world,
                   'GRIPPER' + str(id), ['GRIPPER'],
                   possible_states=possible_states)
    self.attached_item = None

  def _get_item_in_gripper(self):
    for thing in self.world.things:
      if (isinstance(thing, Nut) or isinstance(
          thing, Bolt)) and thing.get_state()['LOCATION'] == 'GRIPPER':
        result = thing
        break
    return result

  def empty_gripper(self, success_rate=1):
    if random.random() < success_rate:
      self.attached_item = self._get_item_in_gripper()
      self.attached_item.set_state('TABLE')
      self.set_state('EMPTY')


class Jig(Thing):

  def __init__(self, world, id):
    possible_states = {
        'NUT_SLOT': ('NUT_SLOT_EMPTY', 'NUT_SLOT_LOADED', 'NUT_SLOT_BLOCKED'),
        'BOLT_SLOT':
            ('BOLT_SLOT_EMPTY', 'BOLT_SLOT_LOADED', 'BOLT_SLOT_BLOCKED')
    }
    Thing.__init__(self,
                   world,
                   'JIG' + str(id), ['JIG'],
                   possible_states=possible_states)

  def _get_nut_in_jig(self):
    for thing in self.world.things:
      if isinstance(thing, Nut) and thing.get_state()['LOCATION'] == 'JIG':
        result = thing
        break
    return result

  def _get_bolt_in_jig(self):
    for thing in self.world.things:
      if isinstance(thing, Bolt) and thing.get_state()['LOCATION'] == 'JIG':
        result = thing
        break
    return result

  def empty_jig_for_nut(self, success_rate=1):
    if random.random() < success_rate:
      self.attached_nut = self._get_nut_in_jig()
      self.attached_nut.set_state('TABLE')
      self.set_state('NUT_SLOT_EMPTY')

  def empty_jig_for_bolt(self, success_rate=1):
    if random.random() < success_rate:
      self.attached_bolt = self._get_bolt_in_jig()
      self.attached_nut.set_state('TABLE')
      self.set_state('BOLT_SLOT_EMPTY')
