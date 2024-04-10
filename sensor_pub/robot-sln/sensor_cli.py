import sys

from sensor_pub.srv import GetSensorData
from sensor_pub.msg import SensorData
import rclpy
from rclpy.node import Node

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
        self.req.num_samples = num_samples
        future = self.client.call_async(request)

        # self.get_logger().info('Sending request: {} + {} = ?'.format(request.a, request.b))

        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                try:
                    self.response = future.result()
                    sef.new_data = True
                    break
                except Exception as e:
                    self.get_logger().error('Service call failed %r' % (e,))
                    break

    def publish_data():
        if self.response is not None:
            if not self.new_data:
                sensor_client.get_logger().warn(
                    'Publishing stale data')

            # TODO: create message 
            msg = SensorData()
            msg.data = self.response
            self.pub.publish(msg)

            self.new_data = False # the published data is now stale
        else: 
            sensor_client.get_logger().info(
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