# Import the Client from ambf_client package
from ambf_client import Client
import time
from Utlities import Plotter
from Model import Exoskeleton
_client = Client()
# Create a instance of the client
_client.connect()
LARRE = Exoskeleton.Exoskeleton(_client, 56, 1.56)
print "jf"
leg_plot = Plotter.Plotter(LARRE)
while 1:
    leg_plot.update()


_client.clean_up()

