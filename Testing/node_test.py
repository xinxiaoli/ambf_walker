from Simulation import AMBF
import numpy as np
from Utlities import Plotter

sim = AMBF.AMBF("revolute", 52, 1.57)
plot = Plotter.Plotter(sim)
cmd = np.asarray([0]*7)

while 1:
   cmd[0] = 100
   sim.send_command(cmd)
   #sim.fk()
   plot.update()


