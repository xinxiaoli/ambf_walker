# Import the Client from ambf_client package
from ambf_client import Client
import time
from Utlities import Plotter
import rospy
from std_msgs.msg import Float32MultiArray
from Model import Exoskeleton
from Main import BodyController

_client = Client()
# Create a instance of the client
_client.connect()
LARRE = Exoskeleton.Exoskeleton(_client, 56, 1.56)
leg_plot = Plotter.Plotter(LARRE)
crl = BodyController.BodyController(LARRE)
#pub=rospy.Publisher('qd',Float32MultiArray, queue_size=1)
rate = rospy.Rate(1000)   #1000hz
while not rospy.is_shutdown():
    leg_plot.update()
    crl.calc_gravity()
    rate.sleep()
    #msg = Float32MultiArray()
    #msg.data = LARRE._handle.get_all_joint_vel()
    #pub.publish(msg)


_client.clean_up()

