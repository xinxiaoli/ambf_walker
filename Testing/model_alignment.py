from Model import Exoskeleton
from Utlities import Plotter
from ambf_client import Client
import numpy as np
import time
exo = Exoskeleton.Exoskeleton(65,1.57)
plotter = Plotter.Plotter(exo)
client = Client()
client.connect()

h = client.get_obj_handle("hex/Hip")

time.sleep(5)
h.set_pos(0,0,2.3)
h.set_rpy(0,0,0)
q = [0.0]*6
while 1:
    h.set_joint_pos(0,0)
    h.set_joint_pos(3, 0)
    h.set_joint_pos(2, 0)
    h.set_joint_pos(5,0)
    q = [0.0] * 6
    print h.get_joint_names()
    q = h.get_all_joint_pos()
    #print q
    q.insert(0,0.0)
    exo.update_joints(np.asarray(q), np.asarray([0.0]*7))
    plotter.update()


