from sensor_interfaces.srv import GetSensorData

import rclpy
from rclpy.node import Node
from threading import Thread
import socket
import sys
# sys.path.append("/home/aarun/Research/current_projects/machina_hw/src/sensor_pub")
# TODO remove this and automate it

import numpy as np

from .third_party.sensor import Sensor

class SensorService(Node):
    """A class to create a service that can be used to get sensor data from the sensors"""
    def __init__(self):
        super().__init__("sensor_service")
        self.ip_address1 = "127.0.0.3"
        self.ip_address2 = "127.0.0.4"

        self.srv1 = self.create_service(GetSensorData, "get_sensor1_data", self.get_sensor1_data_callback)
        self.srv2 = self.create_service(GetSensorData, "get_sensor2_data", self.get_sensor2_data_callback)


        # Start the sensors in a separate threads
        sensor1 = Sensor(self.ip_address1, 10000, 2000, 0.001) # Define a sensor with 2000Hz sampling rate and 1ms delay
        t1 = Thread(target = sensor1.run)
        t1.daemon = True

        t1.start() # TODO: how is this thread stopped?

        sensor2 = Sensor(self.ip_address2, 10000, 2000, 0.001) # Define a sensor with 2000Hz sampling rate and 1ms delay
        t2 = Thread(target = sensor2.run)
        t2.daemon = True

        t2.start() # TODO: how is this thread stopped?


        # Make a connection to both sensors and keep it open when the service is running 
        self.sock1 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock2 = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect the socket to the port where the server is listening
        server_address1 = (self.ip_address1, 10000)
        server_address2 = (self.ip_address2, 10000)
        
        self.get_logger().info('connecting to {} port {}'.format(*server_address1))
        self.sock1.connect(server_address1)


        self.get_logger().info('connecting to {} port {}'.format(*server_address2))
        self.sock2.connect(server_address2)

    def get_sensor1_data_callback(self, request, response):
        """A callback function to get the sensor data from sensor 1
        Args:
            request: A request object with the number of samples that is needed
            response: A response object with the sensor data"""

        # Send the number of samples that is needed from the sensor
        num_samples = request.num_samples
        message_string = str(num_samples)
        message = message_string.encode()
        self.sock1.sendall(message)

        byte_data = self.sock1.recv(10000)
        data = np.frombuffer(byte_data)
        response.sensor_data = list(data) # 1D numpy array with NUM_SAMPLES*6 elements

        self.get_logger().info("Access Sensor 1 data")

        return response

    def get_sensor2_data_callback(self, request, response):
        """A callback function to get the sensor data from sensor 2"""

        # Send the number of samples that is needed from the sensor
        num_samples = request.num_samples
        message_string = str(num_samples)
        message = message_string.encode()
        self.sock2.sendall(message)

        byte_data = self.sock2.recv(10000)
        data = np.frombuffer(byte_data)
        response.sensor_data = list(data) # 1D numpy array with NUM_SAMPLES*6 elements
        self.get_logger().info("Access Sensor 2 data")
        return response


def main(args=None):
    """Main function to create a node and run the sensor service"""

    rclpy.init(args=args)
    sensor_service = SensorService()

    try:
        rclpy.spin(sensor_service)
    except KeyboardInterrupt:
        pass

    sensor_service.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()