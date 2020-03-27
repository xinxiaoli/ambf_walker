from Model import Exoskeleton
from Utlities import Plotter
from ambf_client import Client
import time



# Create a instance of the client
_client = Client()

# Connect the client which in turn creates callable objects from ROS topics
# and initiates a shared pool of threads for bidrectional communication
_client.connect()

# You can print the names of objects found
print(_client.get_obj_names())
time.sleep(2)
handle = _client.get_obj_handle('Hip')
print handle.get_all_joint_pos()