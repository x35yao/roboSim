from ._requirements import *
import pickle


def _update_snapshot(self, state, snapshot):
  for feature in state:
    for val in feature:
      snapshot.append(val)


def record_snapshot(self,
                    cur_state,
                    target_pos=None,
                    target_orn=None,
                    target_finger_dist=None):
  cur_snapshot = {}
  cur_snapshot['snapshot'] = self.snapshot_id
  self.snapshot_id += 1
  cur_snapshot['action'] = cur_state
  cur_snapshot['objs'] = []
  # Record poses of all objects of interest
  for mesh_name in self.objects_to_record:
    meshes = self.meshes[mesh_name]
    for idx in range(len(meshes)):
      mesh_pos, mesh_orn = self.get_mesh_pose(mesh_name=mesh_name,
                                              copy_idx=idx,
                                              as_euler=False)

      # Store data of object label, pos [x,y,z], orn [x,y,z,w]
      mesh_state = {}
      mesh_state['class'] = mesh_name
      mesh_state['pos'] = mesh_pos
      mesh_state['orn'] = mesh_orn
      mesh_state['idx'] = idx
      cur_snapshot['objs'].append(mesh_state)

  gripper_state = {}
  # Add current gripper pose to snapshot
  cur_gripper_pos, cur_gripper_orn = self.get_endeffector_real_pose(
      as_euler=False)
  gripper_state['cur_pos'] = cur_gripper_pos
  gripper_state['cur_orn'] = cur_gripper_orn

  # Add current endeffector finger distance
  gripper_state['cur_finger_dist'] = self.finger_target

  # Add current endeffector finger forces [Fx, Fy, Fz, Mx, My, Mz]
  finger_reaction_forces = self.bt.getJointState(self.robot, 6)[2]
  gripper_state['cur_finger_forces'] = finger_reaction_forces

  # Add target pose
  gripper_state['target_pos'] = target_pos
  gripper_state['target_orn'] = target_orn
  # Add target finger distance
  gripper_state['target_finger_dist'] = target_finger_dist

  # Append new row for this snapshot
  cur_snapshot['gripper'] = gripper_state
  self.data.append(cur_snapshot)

def take_snapshot(self,
                    cur_state,
                    target_pos=None,
                    target_orn=None,
                    target_finger_dist=None):
  cur_snapshot = {}
  cur_snapshot['snapshot'] = self.snapshot_id
  self.snapshot_id += 1
  cur_snapshot['action'] = cur_state
  cur_snapshot['objs'] = []
  # Record poses of all objects of interest
  for mesh_name in self.objects_to_record:
    meshes = self.meshes[mesh_name]
    for idx in range(len(meshes)):
      mesh_pos, mesh_orn = self.get_mesh_pose(mesh_name=mesh_name,
                                              copy_idx=idx,
                                              as_euler=False)

      # Store data of object label, pos [x,y,z], orn [x,y,z,w]
      mesh_state = {}
      mesh_state['class'] = mesh_name
      mesh_state['pos'] = mesh_pos
      mesh_state['orn'] = mesh_orn
      mesh_state['idx'] = idx
      cur_snapshot['objs'].append(mesh_state)

  gripper_state = {}
  # Add current gripper pose to snapshot
  cur_gripper_pos, cur_gripper_orn = self.get_endeffector_real_pose(
      as_euler=False)
  gripper_state['cur_pos'] = cur_gripper_pos
  gripper_state['cur_orn'] = cur_gripper_orn

  # Add current endeffector finger distance
  gripper_state['cur_finger_dist'] = self.finger_target

  # Add current endeffector finger forces [Fx, Fy, Fz, Mx, My, Mz]
  finger_reaction_forces = self.bt.getJointState(self.robot, 6)[2]
  gripper_state['cur_finger_forces'] = finger_reaction_forces

  # Add target pose
  gripper_state['target_pos'] = target_pos
  gripper_state['target_orn'] = target_orn
  # Add target finger distance
  gripper_state['target_finger_dist'] = target_finger_dist

  # Append new row for this snapshot
  cur_snapshot['gripper'] = gripper_state
  return cur_snapshot

def take_snapshot(self,
                  cur_state,
                  target_pos=None,
                  target_orn=None,
                  target_finger_dist=None):
  cur_snapshot = {}
  cur_snapshot['snapshot'] = self.snapshot_id
  self.snapshot_id += 1
  cur_snapshot['action'] = cur_state
  cur_snapshot['objs'] = []
  # Record poses of all objects of interest
  for mesh_name in self.objects_to_record:
    meshes = self.meshes[mesh_name]
    for idx in range(len(meshes)):
      mesh_pos, mesh_orn = self.get_mesh_pose(mesh_name=mesh_name,
                                              copy_idx=idx,
                                              as_euler=False)

      # Store data of object label, pos [x,y,z], orn [x,y,z,w]
      mesh_state = {}
      mesh_state['class'] = mesh_name
      mesh_state['pos'] = mesh_pos
      mesh_state['orn'] = mesh_orn
      mesh_state['idx'] = idx
      cur_snapshot['objs'].append(mesh_state)

  gripper_state = {}
  # Add current gripper pose to snapshot
  cur_gripper_pos, cur_gripper_orn = self.get_endeffector_real_pose(
      as_euler=False)
  gripper_state['cur_pos'] = cur_gripper_pos
  gripper_state['cur_orn'] = cur_gripper_orn

  # Add current endeffector finger distance
  gripper_state['cur_finger_dist'] = self.finger_target

  # Add current endeffector finger forces [Fx, Fy, Fz, Mx, My, Mz]
  finger_reaction_forces = self.bt.getJointState(self.robot, 6)[2]
  gripper_state['cur_finger_forces'] = finger_reaction_forces

  # Add target pose
  gripper_state['target_pos'] = target_pos
  gripper_state['target_orn'] = target_orn
  # Add target finger distance
  gripper_state['target_finger_dist'] = target_finger_dist

  # Append new row for this snapshot
  cur_snapshot['gripper'] = gripper_state
  return cur_snapshot


def dump_data(self, outfile_path):
  with open(outfile_path, "ab") as outfile:
    pickle.dump(self.data, outfile)

  self.data = []


def log_data(self):
  self._verify_dir_exists(self.out_data_dir)
  filename = self.out_data_dir + "/data.pickle"
  self.dump_data(filename)
  print("Logged sequence {} of length {} to file {}".format(
      self.seq_id, self.snapshot_id, filename))
  self.snapshot_id = 0
  self.seq_id += 1
