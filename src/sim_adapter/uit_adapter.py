from .simulator import SimAdapter, Simulator
import rospy
from geometry_msgs.msg import Twist
import numpy as np
import cv2

import threading
import tornado
import socketio

from PIL import Image
from io import BytesIO
import base64

class UITSimAdapter(SimAdapter):
    r"""
    This adapter is aimed to work with UIT-RaceCar's competition simulators.

    These simulators use `socketio` to connect and interact with our platform.
    """

    def __init__(self, sim: Simulator):
        self.sim = sim
        self.sio = socketio.AsyncServer(async_mode='tornado')

        self.speed = 0.0
        self.angle = 0.0

        self.sio.on('telemetry', self.telemetry)
        self.sio.on('connect', self.connect)

    def on_start(self):
        port = int(rospy.get_param('~uit_port', default=4567))
        hostname = rospy.get_param('~uit_hostname', default='127.0.0.1')

        rospy.loginfo('Listening to %s:%d', hostname, port)

        app = tornado.web.Application(
            [
                (r"/socket.io/", socketio.get_tornado_handler(self.sio)),
            ],
        )

        self.server = app.listen(port, address=hostname)
        self.thread = threading.Thread(target=tornado.ioloop.IOLoop.current().start)
        self.thread.start()

    def on_exit(self):
        def exit_callback():
            self.server.stop()
            tornado.ioloop.IOLoop.current().stop()

        tornado.ioloop.IOLoop.current().add_callback(exit_callback)
        self.thread.join()

    async def connect(self, sid, environ):
        rospy.loginfo('Connect to socket id: %s', sid)
        await self.send_control()

    async def telemetry(self, sid, data):
        if data:
            image = Image.open(BytesIO(base64.b64decode(data["image"])))
            image = np.asarray(image)
            image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

            self.sim.recv_image(image)
            await self.send_control()

        else:
            await self.sio.emit('manual', data={}, skip_sid=True)

    async def send_control(self):
        await self.sio.emit(
            "steer",
            data={
                'steering_angle': str(self.angle),
                'throttle': str(self.speed),
            },
            skip_sid=True)

    def send_command(self, cmd_vel: Twist):
        self.speed = cmd_vel.linear.x
        self.angle = cmd_vel.angular.z
