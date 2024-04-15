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

### Branch `faster`

There are two branches provided as part of this solution: [master]() and [faster](). The master branch contains a naive sampling strategy wherein 1 sample for each sensor is read at a time. However, when the server is serving two sensor clients simultaneously, the turn-around time for the sampling is ~8 ms. This means for a many samples, "stale" samples are published, as apparent by the ROS warnings. 

Instead of this naive implementation, in branch [faster](), we sample a larger number of sensor readings at a time. The main thought process is that there is a ~3 ms start-up cost (which includes sensor delay, additional uniform delay and processing delay) to access sensor data, after which there's linear cost of ~0.5 ms (sampling delay). Furthermore, if two clients are served simultaneously, then these costs double. Given these numbers, we'd like the average sample access time to be ~2 ms to reduce the number of stale data we repeat. In other words:

(2 (0.5 x + 3)) / x = 2
==> x = 6,

where x is th number of samples extracted. These samples are stored in a numpy array and for each publisher callback the next row of data is reported. When a new sample is read, the row index is reset to 0 and new data is reported. This reduces stale data warnings however at the cost of reporting slightly older data. 

  