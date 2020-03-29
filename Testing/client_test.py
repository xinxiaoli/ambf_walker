# Import the Client from ambf_client package
from ambf_client import Client
import time
from Utlities import Plotter
import rospy
from std_msgs.msg import Float32MultiArray
from Model import Exoskeleton
_client = Client()
# Create a instance of the client
_client.connect()
LARRE = Exoskeleton.Exoskeleton(_client, 56, 1.56)
leg_plot = Plotter.Plotter(LARRE)
#pub=rospy.Publisher('qd',Float32MultiArray, queue_size=1)
while 1:
    leg_plot.update()
    #msg = Float32MultiArray()
    #msg.data = LARRE._handle.get_all_joint_vel()
    #pub.publish(msg)


_client.clean_up()

