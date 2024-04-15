import sys

import sys
sys.path.append("/home/aarun/Research/current_projects/machina_hw/src/sensor_pub")


from sensor_interfaces.srv import GetSensorData
from sensor_interfaces.msg import SensorData
import rclpy
from rclpy.node import Node
import time
from multiprocessing import Process

PERIOD = 1/500 # period at which topic should be published
NUM_SAMPLES = 1

class SensorClient(Node):

    def __init__(self, topic_name, service_name):
        super().__init__("client_"+service_name)
        self.cli = self.create_client(GetSensorData, service_name)
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetSensorData.Request()
        self.response = None
        self.packet_num = False
        self.topic_name = topic_name

        # create a publisher to publish the sensor data
        self.pub = self.create_publisher(SensorData, "/"+topic_name, 10)
        self.timer = self.create_timer(PERIOD, self.publish_data)

    def send_request(self, num_samples):
        tic = time.time()
        self.req.num_samples = num_samples
        future = self.cli.call_async(self.req)

        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                try:
                    self.response = future.result()
                    self.new_data = True
                    break
                except Exception as e:
                    self.get_logger().error('Service call failed %r' % (e,))
                    break
        self.get_logger().info(f"Service call completed in {time.time() - tic} s")

    def publish_data(self):
        if self.response is not None:
            if not self.new_data:
                self.get_logger().warn(
                    'Publishing stale data')

            msg = SensorData()
            msg.data = self.response.sensor_data.tolist()
            self.pub.publish(msg)

            self.new_data = False # the published data is now stale
        else: 
            self.get_logger().info(
                    f"Sensor data for {self.topic_name} topic not yet ready to be published")


def main1(args=None):
    rclpy.init(args=args)

    # create a client to query the first sensor service
    topic_name1 = "sensor1"
    service_name1 = "get_sensor1_data"
    node1 = SensorClient(topic_name1, service_name1)

    try: 
        while rclpy.ok():
            node1.send_request(NUM_SAMPLES)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

def main2(args=None):
    rclpy.init(args=args)

    # create another client
    topic_name2 = "sensor2"
    service_name2 = "get_sensor2_data"
    node2 = SensorClient(topic_name2, service_name2)

    try: 
        while rclpy.ok():
            node2.send_request(NUM_SAMPLES)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()