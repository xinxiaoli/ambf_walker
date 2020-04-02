
from ambf_client import Client
import time

_client = Client()

_client.connect()

handle = _client.get_obj_handle('Hip')
handle.set_pos(0,0,0)
for i in range(0, 5000):
    time.sleep(0.001) # Sleep for a while to see the effect of the command before moving on
