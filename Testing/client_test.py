# Import the Client from ambf_client package
from ambf_client import Client
import time

_client = Client()
# Create a instance of the client
_client.connect()

h = _client.get_obj_handle("hex/Hip")
print h.get_joint_names()
h.set_joint_effort("Hip-Leftthigh", 100)
time.sleep(5)
_client.clean_up()

