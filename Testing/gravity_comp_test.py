from Model import Exoskeleton
from Utlities import Plotter
from ambf_client import Client
import time
import numpy as np

exo = Exoskeleton.Exoskeleton(65,1.57)
plotter = Plotter.Plotter(exo)

client = Client()
client.connect()
h = client.get_obj_handle("Hip")

while 1:
   q = h.get_all_joint_pos()
   q = np.array([0.0] * 7)
   qd = np.array([0.0] * 7)
   exo.update_joints(q,qd)
   plotter.update()
   time.sleep(0.1)