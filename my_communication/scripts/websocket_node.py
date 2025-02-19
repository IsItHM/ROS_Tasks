#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import websockets
import asyncio

class ESP32ROSBridge:
    def __init__(self):
        rospy.init_node('esp32_bridge', anonymous=True)
        self.pub = rospy.Publisher('esp32_messages', String, queue_size=10)
        self.uri = "ws://192.168.0.191:81"  # Replace with your ESP32's IP address
        self.websocket = None
        
    async def connect(self):
        try:
            self.websocket = await websockets.connect(self.uri)
            rospy.loginfo("Connected to ESP32")
            return True
        except Exception as e:
            rospy.logerr(f"Connection failed: {e}")
            return False

    async def receive_and_respond(self):
        while not rospy.is_shutdown():
            try:
                # Receive message from ESP32
                message = await self.websocket.recv()
                rospy.loginfo(f"Received from ESP32: {message}")
                
                # Publish to ROS topic
                self.pub.publish(String(message))
                
                # Send response back to ESP32
                response = "Message received by ROS"
                await self.websocket.send(response)
                rospy.loginfo(f"Sent to ESP32: {response}")
                
            except Exception as e:
                rospy.logerr(f"Error: {e}")
                break

    async def main(self):
        connected = await self.connect()
        if connected:
            await self.receive_and_respond()

if __name__ == '__main__':
    bridge = ESP32ROSBridge()
    asyncio.get_event_loop().run_until_complete(bridge.main())