from .simulator import SimAdapter, Simulator
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import numpy as np
import cv2


class DiRASimAdapter(SimAdapter):
    r"""
    This adapter is aimed to work with DiRA's competition simulators.

    These simulators use `rosbridge` to connect and interact with our platform.
    Each of them provides:
        - dira_rgb_topic: a topic to receive a color image
        - dira_speed_topic: a topic to set driving speed
        - dira_angle_topic: a topic to set driving angle
    """

    def __init__(self, sim: Simulator):
        self.sim = sim

        dira_rgb_topic = rospy.get_param('~dira_rgb_topic', None)
        if dira_rgb_topic is None:
            raise ValueError('dira_rgb_topic is not set. Please set it in launch file or via rosparam before'
                             ' running this node')
        rospy.loginfo('dira_rgb_topic is set to %s', dira_rgb_topic)

        dira_speed_topic = rospy.get_param('~dira_speed_topic', None)
        if dira_speed_topic is None:
            raise ValueError('dira_speed_topic is not set. Please set it in launch file or via rosparam before'
                             'running this node')
        rospy.loginfo('dira_speed_topic is set to %s', dira_speed_topic)

        dira_angle_topic = rospy.get_param('~dira_angle_topic', None)
        if dira_angle_topic is None:
            raise ValueError('dira_angle_topic is not set. Please set it in launch file or via rosparam before'
                             'running this node')
        rospy.loginfo('dira_angle_topic is set to %s', dira_angle_topic)

        queue_size = rospy.get_param('~queue_size', default=10)

        self.img_sub = rospy.Subscriber(dira_rgb_topic,
                                        CompressedImage, self._image_callback)

        self.velocity_pub = rospy.Publisher(dira_speed_topic, Float32, queue_size=queue_size)
        self.angle_pub = rospy.Publisher(dira_angle_topic, Float32, queue_size=queue_size)

    def _image_callback(self, image: CompressedImage):
        image = cv2.imdecode(np.frombuffer(image.data, np.uint8), cv2.IMREAD_COLOR)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.sim.recv_image(image)

    def send_command(self, cmd_vel: Twist):
        speed = cmd_vel.linear.x
        angle = cmd_vel.angular.z
        self.velocity_pub.publish(Float32(data=speed))
        self.angle_pub.publish(Float32(data=angle))
