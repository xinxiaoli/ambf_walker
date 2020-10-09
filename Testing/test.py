
from ambf_client import Client
from rospy import Rate
_client = Client()
_client.connect()
handle = _client.get_obj_handle('Hip')
rate = Rate(1000)
while 1:
    handle.set_pos(0,0,0)
    rate.sleep()