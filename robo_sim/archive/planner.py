from .objects import Gripper, Jig, Nut, Bolt
import numpy as np
from .hrr import normalize, bind, unbind


class Planner(object):

  def __init__(self, world, cm_ID, cm_SP, noise=0):

    self.world = world
    self.plan = []
    self.current_goal = None
    self.nut = None
    self.bolt = None
    self.gripper = [
        thing for thing in world.things if isinstance(thing, Gripper)
    ][0]
    self.jig = [thing for thing in world.things if isinstance(thing, Jig)][0]
    self.current_action = None
    self.action_found = False
    self.dim = 512
    self.noise = noise
    self.noise_std = self.noise * np.sqrt(1 / self.dim)

    self.associate_memory = {
        'BOLT_NUT_IN_BIN': ['PUT_BOLT_NUT_IN_BIN'],
        'BOLT_NUT_IN_GRIPPER': ['PICK_UP_BOLT_NUT'],
        'BOLT_NUT_ASSEMBLED': ['ASSEMBLE'],
        # 'BOLT_READY_TO_ASSEMBLE': ['PICK_UP_BOLT_FROM_JIG'],
        'BOLT_IN_JIG': ['PUT_BOLT_IN_JIG'],
        'BOLT_IN_GRIPPER': ['PICK_UP_BOLT_FROM_TABLE'],
        'NUT_READY_TO_ASSEMBLE': ['DISASSEMBLE', 'PUT_NUT_IN_JIG'],
        'NUT_IN_GRIPPER': ['PICK_UP_NUT_FROM_TABLE'],
        'JIG_NUT_SLOT_EMPTY': ['EMPTY_JIG_FOR_NUT'],
        'JIG_BOLT_SLOT_EMPTY': ['EMPTY_JIG_FOR_BOLT'],
        'GRIPPER_EMPTY': ['EMPTY_GRIPPER'],
        'NUT_ON_TABLE': ['NONE'],
        'BOLT_ON_TABLE': ['NONE'],
        'BOLT_NUT_STUCK': ['NONE']
    }
    self.cm_ID = cm_ID
    self.cm_SP = cm_SP

  def get_preconditions(self, action):
    '''
        get preconditions for an action
        '''
    pres = []
    action_vec = self.cm_SP.get(action)
    sum_pres = unbind(action_vec,
                      self.cm_ID.get('PRE'),
                      noise_std=self.noise_std)

    while True:
      index, clean, similarity = self.cm_ID.cleanup(sum_pres)
      if similarity > 0.3:
        pres.append(self.cm_ID.concepts[index])
        sum_pres = sum_pres - clean * similarity
      else:
        break
    return pres

  def get_action(self):
    '''
        Return True when an action is found and False otherwise.
        The action found is updated to self.current_action
        '''
    if not self.action_found:
      self.current_action = None
      if self.is_state_satisfied(self.current_goal):
        return self.action_found
      else:
        actions = self.associate_memory[self.current_goal]
        if actions == ['NONE'] and self.current_goal == 'BOLT_NUT_STUCK':
          return
        elif actions == ['NONE'] and self.current_goal != 'BOLT_NUT_STUCK':
          print(
              'Cannot find an executable action because theres no bolt/nut no table'
          )
          return self.action_found
        else:
          for action in actions:
            # print(action)
            if self._is_action_executable(action):
              self.action_found = True
              self.current_action = action
              self.plan.append(self.get_preconditions(action))
              return self.action_found
            else:
              pre_conditions = self.get_preconditions(action)
              self.plan.append(pre_conditions)
              for pre in pre_conditions:
                self.current_goal = pre
                self.get_action()
          return self.action_found

  def make_plan(self, goal):
    self.current_goal = goal
    self.plan.append([goal])
    flag = self.get_action()
    if not flag:
      self.plan = []
    self.action_found = False
    return flag
  def is_state_satisfied(self, state):
    obj = self._get_obj(state)
    obj_states = self._get_obj_states(state, obj)
    #         print(obj, obj_states)
    if obj == 'BOLT' or obj == 'NUT':
      for thing in self.world.things:
        if thing.kinds[0] == obj and all(
            [state in obj_states for state in thing.get_state().values()]):
          if obj == 'NUT':
            self.nut = thing
          elif obj == 'BOLT':
            self.bolt = thing
          return True
    elif obj == 'GRIPPER' or obj == 'JIG':
      for thing in self.world.things:
        if thing.kinds[0] == obj and all(
            [state in thing.get_state().values() for state in obj_states]):
          return True
    return False

  def _is_action_executable(self, action):
    pre_conditions = self.get_preconditions(action)
    for pre in pre_conditions:
      if not self.is_state_satisfied(pre):
        return False
    return True

  def _get_obj(self, pre):
    '''Since states are defined as obj*(locations + assemble_state) or obj*(assemblle_state),
           this function is used to find the obj.
        '''
    pre_vec = self.cm_SP.get(pre)
    similarities1 = []
    similarities2 = []
    objs = ['NUT', 'BOLT', 'JIG', 'GRIPPER']
    for obj in objs:
      obj_states_vec = unbind(pre_vec,
                              self.cm_ID.get(obj),
                              noise_std=self.noise_std)
      index, clean, similarity = self.cm_ID.cleanup(obj_states_vec)
      obj_states_vec = obj_states_vec - similarity * clean
      similarities1.append(similarity)
      index, clean, similarity = self.cm_ID.cleanup(obj_states_vec)
      similarities2.append(similarity)
    if max(similarities2) > 0.3:
      ind = np.argmax(similarities2)
    else:
      ind = np.argmax(similarities1)
    return objs[ind]

  def _get_obj_states(self, pre, obj):

    pre_vec = self.cm_SP.get(pre)
    obj_states_vec = unbind(pre_vec,
                            self.cm_ID.get(obj),
                            noise_std=self.noise_std)
    obj_states = []
    while True:
      index, clean, similarity = self.cm_ID.cleanup(obj_states_vec)
      if similarity > 0.3:
        obj_states.append(self.cm_ID.concepts[index])
        obj_states_vec = obj_states_vec - clean * similarity
      else:
        break
    return obj_states
