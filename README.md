## Sensor Simulator 

There are two packages within this ROS2 workspace: 
1. `sensor_interfaces`: Contains the `.srv` and the `.msg` formats for the custom sensor service and custom sensor message. 
2. `sensor_pub`: contins the ROS2 node definitions for the service server and the service clients. 

### Sensor Interfaces: 

We need to create a separate package for the sensor interfaces as new message and service format descritions need to be provided in the C++ package. 
1. `GetSensorData.srv`: this service takes in `num_samples` as the input request and quries those many samples from the sensor. The response is a 1D list of 6DoF data from the sensor. The length of the list is 6 x `num_samples` and it is of type float64. 
2. `SensorData.msg`: this message format is published within the `sensorX` topic name by the service client. It consistents of the sensor data response from the service server.  

### Sensor Pub:

`setup.py` configures the node executable names for the server and the two sensor clients. They can be conveniently run by calling the following calls in separate terminals: 

`ros2 run sensor_pub service`

`ros2 run sensor_pub client1`

`ros2 run sensor_pub client2`


The service server defines two simulated sensors at IP addresses 127.0.0.3 and 127.0.0.4, and socket 10000. Both sensors have a random delay between 1-2 ms. Two separate callbacks are defined for two differnt services. The service client sets up a caller for each of these services and publishes the returned data to two topics: `/sensor1` and `/sensor2` at a rate of 500 Hz. 