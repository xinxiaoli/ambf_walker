from Model import Exoskeleton
from Utlities import Plotter
from ambf_client import Client
from lib.Python import RMP_runner
import numpy as np
import time

exo = Exoskeleton.Exoskeleton(65,1.57)
plotter = Plotter.Plotter(exo)
client = Client()
client.connect()
h = client.get_obj_handle("hex/Hip")
h.set_pos(0,0,2.3)
h.set_rpy(0.0,0.0,0.0)
q = [0.0]*6

Lankle_runner = RMP_runner.RMP_runner("../config/ankle_left.xml")
Lhip_runner = RMP_runner.RMP_runner("../config/hip_left.xml")
Lknee_runner = RMP_runner.RMP_runner("../config/knee_left.xml")


Rhip_runner = RMP_runner.RMP_runner("../config/hip_right.xml")
Rknee_runner = RMP_runner.RMP_runner("../config/knee_right.xml")
Rankle_runner = RMP_runner.RMP_runner("../config/ankle_right.xml")


while 1:

    Lhip, Lhipd, Lhipdd = Lhip_runner.step(1.0)
    Lknee, Lkneed, Lkneedd = Lknee_runner.step(1.0)
    Lankle, Lankled, Lankledd = Lankle_runner.step(1.0)
    #
    Rhip, Rhipd, Rhipdd = Rhip_runner.step(1.0)
    Rknee, Rkneed, Rkneedd = Rknee_runner.step(1.0)
    Rankle, Rankled, Rankledd = Rankle_runner.step(1.0)

    h.set_joint_pos(0, Lhip)
    h.set_joint_pos(1, Lankle)
    h.set_joint_pos(2, Lknee)
    h.set_joint_pos(3, Rhip)
    h.set_joint_pos(4, Rankle)
    h.set_joint_pos(5, Rknee)

    q = h.get_all_joint_pos()
    #print q
    q.insert(0,0.0)
    exo.update_joints(np.asarray(q), np.asarray([0.0]*7))
    plotter.update()


