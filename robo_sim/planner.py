from .hrr import normalize, bind, unbind, random_vectors, CleanupMemory
import numpy as np
import pickle

class Planner(object):

    def __init__(self, world, num_states, noise=0):
        '''
        world: a vector of global state at the movement

        '''
        self.world = world
        self.dim = 512
        self.current_goal = None
        self.current_action = None
        self.action_found = False
        self.plan = []
        self.noise = noise
        self.noise_std = self.noise * np.sqrt(1 / self.dim)
        # self.global_state_vec = global_state_vec
        self.actions = ['PUT_NUT_IN_JIG', 'PUT_BOLT_IN_JIG', 'ASSEMBLE']
        self.local_states = [f'LOCAL{i}' for i in range(num_states)]
        self.basics = ['PRE']
        ID = self.basics + self.local_states + self.actions
        self.cm_ID = CleanupMemory(ID, self.dim, self.noise_std)
        self.cm_SP = self.build_preconditions()
        self.associate_memory = self.build_associate_memory()

    def build_preconditions(self):
        cm_SP = CleanupMemory([], self.dim, self.noise_std)
        with open('./robo_sim/preconditions.pkl', 'rb') as f:
            preconditions = pickle.load(f)
        for action in self.actions:
            precondition_indices = preconditions[action]
            precondition_sum = np.zeros(self.dim)
            for ind in precondition_indices:
                precondition_sum = precondition_sum + self.cm_ID.get(f'LOCAL{ind}')
            action_vec = bind(self.cm_ID.get('PRE'), precondition_sum,
                                     self.noise_std)
            cm_SP.add(action, action_vec)
        return cm_SP

    def build_associate_memory(self):
        associate_memory= {}
        with open('./robo_sim/effects.pkl', 'rb') as f:
            effects = pickle.load(f)
        for action in self.actions:
            associate_memory[f'LOCAL{effects[action][0]}'] = action
        print(associate_memory)
        return associate_memory

    # def update_preconditions(self, action):
    #     ind = [i for i, x in enumerate(self.actions) if x == action][0]
    #     self.global_state_vec[ind] = self.global_state_vec[ind] + self.world
    #     self.global_state_vec[ind] = (self.global_state_vec[ind] == np.max(self.global_state_vec[ind])).astype(int)
    #     self.build_preconditions()
    #     np.save('./robo_sim/global_state_vec.npy', self.global_state_vec)

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

    def is_state_satisfied(self, state):
        state_ind = [i for i, x in enumerate(self.world) if x == 1]
        current_state = [f'LOCAL{i}' for i in state_ind]
        if state in current_state:
            return True
        return False

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
                action= self.associate_memory[self.current_goal]
                self.action_candidate = action
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
                        try:
                            self.get_action()
                        except:
                            x = input(f'Candidate action is {self.action_candidate}, press y to update the preconditions')
                            if x == 'y':
                                self.update_preconditions(self.action_candidate)
                                self.action_found = True
                                self.current_action = self.action_candidate
                                return self.action_found
                            else:
                                print('Planning failed')
                                return self.action_found
                return self.action_found

    def make_plan(self, goal):
        self.current_goal = goal
        self.plan.append([goal])
        flag = self.get_action()
        if not flag:
            self.plan = []
            self.action_found = False
        return flag

    def _is_action_executable(self, action):
        pre_conditions = self.get_preconditions(action)
        for pre in pre_conditions:
            if not self.is_state_satisfied(pre):
                return False
        return True
