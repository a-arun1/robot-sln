from sensor_interfaces.srv import GetSensorData

import rclpy
from rclpy.node import Node
from threading import Thread
import socket
import sys
sys.path.append("/home/aarun/Research/current_projects/machina_hw/src/sensor_pub")
# TODO remove this and automate it

import numpy as np

from third_party.sensor import Sensor

class SensorService(Node):

    def __init__(self, ip_address):
        super().__init__('sensor_service')
        self.srv = self.create_service(GetSensorData, 'get_sensor_data', self.get_sensor_data_callback)
        self.ip_address = ip_address

        # Start the sensor in a separate thread
        sensor1 = Sensor(self.ip_address, 10000, 2000, 0.001) # Define a sensor with 2000Hz sampling rate and 1ms delay
        t1 = Thread(target = sensor1.run)
        t1.daemon = True

        t1.start() # TODO: how is this thread stopped?

        # Make a connection to this sensor and keep it open when the service is running 
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect the socket to the port where the server is listening
        server_address = (self.ip_address, 10000)
        self.get_logger().info('connecting to {} port {}'.format(*server_address))

        self.sock.connect(server_address)

    def get_sensor_data_callback(self, request, response):
        # Send the number of samples that is needed from the sensor
        num_samples = request.num_samples
        message_string = str(num_samples)
        message = message_string.encode()
        self.sock.sendall(message)

        byte_data = self.sock.recv(10000)
        data = np.frombuffer(byte_data)
        # import pdb
        # pdb.set_trace()
        response.sensor_data = list(data) # 1D numpy array with NUM_SAMPLES*6 elements

        return response


def main(args=None):
    rclpy.init(args=args)
    ip_address = '127.0.0.3'
    sensor_service = SensorService(ip_address)

    rclpy.spin(sensor_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()