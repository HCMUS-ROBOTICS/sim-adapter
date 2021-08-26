from typing import Optional
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist


class SimAdapter:

    r"""
    This class is to parse `cmd_vel` to corresponding low-level data that its
    simulator can understand.

    Note:
        This class is one-way only. If you want to send data, i.e. image, from your
        simulator, be sure to call the corresponding method of the `Simulator`'s instance
    """

    def on_start(self):
        r"""Starting callback

        This method is being called before the rospy.spin loop.

        It is necessary for some systems that they need to initialize stuffs before
        the spinning loop starts.
        """
        pass

    def on_exit(self):
        r"""Exit callback

        This method is being called after the rospy.spin loop.

        It is necessary for some systems that they need to release stuffs after
        the spinning loop ends.
        """

    def send_command(self, cmd_vel: Twist):
        pass


class Simulator:

    WIN_SHOW_IMAGE = 'Frame'

    def __init__(self) -> None:
        self.adapter: Optional[SimAdapter] = None

        self.is_show_image = rospy.get_param('~is_show_image', False)
        if self.is_show_image:
            cv2.namedWindow(self.WIN_SHOW_IMAGE, cv2.WINDOW_NORMAL)

        rgb_topic = rospy.get_param('~rgb_topic', '/camera/rgb/image/compressed')
        rospy.loginfo('rgb_topic is set to %s', rgb_topic)

        cmd_topic = rospy.get_param('~cmd_topic', '/cmd_vel')
        rospy.loginfo('cmd_topic is set to %s', cmd_topic)

        queue_size = rospy.get_param('~queue_size', default=10)
        rospy.loginfo('queue_size is %d', queue_size)

        self.image_pub = rospy.Publisher(rgb_topic, CompressedImage, queue_size=queue_size)
        self.cmd_sub = rospy.Subscriber(cmd_topic, Twist, self.callback_send_command)

        rospy.on_shutdown(cv2.destroyAllWindows)
        
    def set_adapter(self, adapter: SimAdapter):
        r"""Set the adapter for this simulator

        Args:
            adapter: an instance of `SimAdapter`
        """
        self.adapter = adapter

    def callback_send_command(self, cmd_vel: Twist):
        r"""Callback of `cmd_topic`
        
        This callback will pass the cmd_vel to the simulator by its adapter

        Args:
            cmd_vel: a driving command containing speed and angle
        """
        self.adapter.send_command(cmd_vel)

    def recv_image(self, image: np.ndarray):
        r"""Publish image

        Every simulator adapter should call this method to publish its image instead
        of publishing itself.

        Args:
            image: an image to be published to `rgb_topic`
        """
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()

        self.image_pub.publish(msg)

        if self.is_show_image:
            cv2.imshow(self.WIN_SHOW_IMAGE, image)
            cv2.waitKey(1)

    def run_loop(self):
        r"""Running loop

        This will hang the process to receive as well as publish messages via callbacks
        initialized from the constructor.
        """

        self.adapter.on_start()
        rospy.spin()
        self.adapter.on_exit()
        rospy.signal_shutdown('Node shutdown gracefully.')
