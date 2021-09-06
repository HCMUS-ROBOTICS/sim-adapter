import base64
import threading
from io import BytesIO

import cv2
import numpy as np
import rospy
import socketio
import tornado
from geometry_msgs.msg import Twist
from PIL import Image
from socketio.asyncio_manager import AsyncManager

from .simulator import SimAdapter, Simulator


class UITSocketManager(AsyncManager):

    def __init__(self, auto_connect: bool = False):
        super().__init__()
        self.auto_connect = auto_connect

    def sid_from_eio_sid(self, eio_sid, namespace):
        sid = super().sid_from_eio_sid(eio_sid, namespace)
        if sid is None and self.auto_connect:
            rospy.loginfo('Client did not send connect. Auto connect')
            sid = self.connect(eio_sid, namespace)
        return sid


class UITSimAdapter(SimAdapter):
    r"""
    This adapter is aimed to work with UIT-RaceCar's competition simulators.

    These simulators use `socketio` to connect and interact with our platform.
    """

    def __init__(self, sim: Simulator):
        self.sim = sim

        auto_connect = rospy.get_param('~auto_connect', True)

        self.sio = socketio.AsyncServer(
            client_manager=UITSocketManager(auto_connect),
            async_mode='tornado',
        )

        self.speed = 0.0
        self.angle = 0.0

        self.sio.on('telemetry', self.telemetry)
        self.sio.on('connect', self.connect)
        self.sio.on('disconnect', self.disconnect)

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
        await self.send_control(sid)

    def disconnect(self, sid):
        rospy.loginfo('Disconnect to socket id: %s', sid)

    async def telemetry(self, sid, data):
        if data:
            image = Image.open(BytesIO(base64.b64decode(data["image"])))
            image = np.asarray(image)
            self.sim.recv_image(image)
            await self.send_control(sid)

        else:
            await self.sio.emit('manual', data={}, to=sid)

    async def send_control(self, sid):
        await self.sio.emit(
            "steer",
            data={
                'steering_angle': str(self.angle),
                'throttle': str(self.speed),
            },
            to=sid,
        )

    def send_command(self, cmd_vel: Twist):
        self.speed = cmd_vel.linear.x
        self.angle = cmd_vel.angular.z
