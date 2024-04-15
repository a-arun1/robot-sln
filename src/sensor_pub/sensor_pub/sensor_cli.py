import sys

import sys
sys.path.append("/home/aarun/Research/current_projects/machina_hw/src/sensor_pub")


from sensor_interfaces.srv import GetSensorData
from sensor_interfaces.msg import SensorData
import rclpy
from rclpy.node import Node
import time

PERIOD = 1/500 # period at which topic should be published
NUM_SAMPLES = 1

class SensorClient(Node):

    def __init__(self, topic_name):
        super().__init__('sensor_client')
        self.cli = self.create_client(GetSensorData, 'get_sensor_data')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetSensorData.Request()
        self.response = None
        self.new_data = False

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
        if self.response is not None and self.new_data:
            if not self.new_data:
                self.get_logger().warn(
                    'Publishing stale data')

            msg = SensorData()
            msg.data = self.response.sensor_data.tolist()
            self.pub.publish(msg)

            self.new_data = False # the published data is now stale
        else: 
            self.get_logger().info(
                    'Sensor data not yet ready to be published')


def main(args=None):
    rclpy.init(args=args)
    topic_name = "sensor1"
    node = SensorClient(topic_name)
    try:
        while rclpy.ok():  # Run forever until interrupted
            node.send_request(NUM_SAMPLES)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()