#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import asyncio

host = "127.0.0.1"
port = 9090

PI = 3.1415926535897


class Movement:

    def __init__(self):
        rospy.init_node('move_robot_node', anonymous=False)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.rate = rospy.Rate(25)
        self.move = Twist()

    def publish(self):
        # publish only once
        while not rospy.is_shutdown():
            connections = move.pub.get_num_connections()
            if connections > 0:
                self.pub.publish(self.move)
                print(f"Publish twist: {self.move}")
                break
            move.rate.sleep()

    def reset(self):
        self.move.linear.x = 0.0
        self.move.linear.y = 0.0
        self.move.linear.z = 0.0
        self.move.angular.x = 0.0
        self.move.angular.y = 0.0
        self.move.angular.z = 0.0

    def forward(self, speed):
        print("move forward")
        self.reset()
        self.move.linear.x = speed
        self.publish()

    def backward(self, speed):
        print("move forward")
        self.reset()
        self.move.linear.x -= speed;
        self.publish()

    def left(self, speed):
        print("move left")
        self.reset()
        self.move.linear.y = speed
        self.publish()

    def right(self, speed):
        print("move right")
        self.reset()
        self.move.linear.y -= speed;
        self.publish()

    def stop(self):
        self.reset()
        self.publish()

    def rotate_left(self, speed):
        print("move left")
        self.reset()
        self.move.angular.z = speed
        self.publish()

    def rotate_right(self, speed):
        print("move right")
        self.reset()
        self.move.linear.z -= speed;
        self.publish()


move = Movement()


class UDPServer:
    def connection_made(self, transport):
        self.transport = transport

    def datagram_received(self, data, addr):
        message = data
        print('Received %r from %s' % (int(message[-1]), addr))

        move.rotate_left(int(message[-1])/255)


async def main():
    print("Starting UDP server")
    loop = asyncio.get_running_loop()

    # One protocol instance will be created to serve all
    # client requests.
    transport, protocol = await loop.create_datagram_endpoint(
        lambda: UDPServer(), local_addr=(host, port))

    print(f"Created server: \nhost - {host}, port - {port}")
    try:
        await asyncio.sleep(60)  # Serve for 1 minute.
    finally:
        move.stop()
        transport.close()


asyncio.run(main())
