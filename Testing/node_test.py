from Simulation import AMBF
import numpy as np


sim = AMBF.AMBF("revolute", 52,1.57)

cmd = np.asarray([0]*7)

while 1:

   cmd[5] = -100
   sim.send_command(cmd)





