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
h.set_pos(0,0,1.0)
time.sleep(5)
q = [0.0]*6
while 1:
    #q = h.get_all_joint_pos()
    q.insert(0,0.0)
    q[2] = 0.717
    exo.update_joints(np.asarray(q), np.asarray([0.0]*7))
    plotter.update()


