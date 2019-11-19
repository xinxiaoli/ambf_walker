import time
from ambf_client import Client
client = Client()
client.connect()


objs  = client.get_obj_names()
objs.remove("lights/light_left")
objs.remove("lights/light_right")
objs.remove("default_camera")
objs.remove("World")


for obj in objs:
    print obj
    h = client.get_obj_handle(obj)
    time.sleep(0.5)

    print h. get_inertia()
