import pybullet as p
import pybullet_data as pd
import os

p.connect(p.DIRECT)
name = "blender_jig_005"
name_in = os.path.join("/home/saeejithn/brain/roboSim/meshes/bolt/" + name + ".obj")
name_out = name + "_vhacd.obj"
name_log = "log.txt"
p.vhacd(name_in, name_out, name_log, alpha=0.04,resolution=50000 )