# UbiSense ROS2 Node

The node can be setup using the following parameters in the yaml file.
Initially there was a IP address, which is not really required as the Ubisense system for braodcating the packets over UDP is setup in a way such that it will braodcast the packets to the device with set IP. So, on the node we just filter the messages using the port number(of the device on which the node is running) and the tag number.

```yaml
ubisense_uwb_publisher:
  ros__parameters:
    # udp_ip: "192.168.222.96"        # This should be the IP of the Ubisense server
    udp_port: 38633                   # This should be the port to which the Ubisense server is broadcasting (your device needs to listen to this port)
    tag: "0011ce000000f963"           # Tag for which the position data needs to be collected (check the tag backside for this value)
    debug: false                      # 0 connect to dummy message to spin the engine , 1 to connect to Ubisense to get actual data
    topic: "ubisense_uwb/pose"        # Topic name to which the position data needs to be published
    logging_level: 1                  # Logging level for the node
```

<hr>

## *So it is important to setup the port number and the tag number in the yaml file.<br>*
## *Also the Ubisense Systems needs to be configured with the IP address of the device on which the node is running.*

<hr>


<br>
<br>
<br>



This is a ROS2 node that is used to communicate with the UbiSense system. The UbiSense system uses the Ultra Wide Band(UWB) technology to track the location of the robots. Exisitng ubisense system publishes the data from individual tag over the UDP server. The UbiSense ROS node is used to communicate with the UDP server and get the pose data of the tag. The UbiSense ROS node is a publisher node that publishes the a pose with covariance stamped msg.

Any UWB based system is made up of beaocns and tag. Ideally beaocns are attached to the surrounding enviornemnt. The tag is attached to the robot. The beacons are used to triangulate the position of the tag. The UbiSense system uses the beacons to triangulate the position of the tag. The UbiSense system publishes the data from the tag over the UDP server. The UbiSense ROS node is used to communicate with the UDP server and get the pose data of the tag. The UbiSense ROS node is a publisher node that publishes the a pose with covariance stamped msg.

*The node was initially developed for ROS by my Rsearch Porjects supervisor [Mr. Angelos Plastropoulos]. This is a ROS2 port for the same with some Bug fixes, most of the funcitonlaity reamins the same.*


<hr>


<br>
<br>



## Known Issues 

1. Issue wehre the socket/port is not being closed properly when the node is being shutdown.
This causes the port to be occupied and doesnot allow the code to be rerun. To fix this issue we need to close the udp socket properly. This chould be implemented in the shutdown method of the node.(probably by overdiding it)



## Fixes

1. The publisher node was inheriting another node(ubisense). To fix this the `ubisense`was converted into a normal class, with the relevant methods to be called. 

2. remove error in the covariance matrix

```python
  # msg.covariance[0] = ubisense_msg.variance ** 2
  # msg.covariance[7] = ubisense_msg.variance ** 2
  # msg.covariance[14] = ubisense_msg.variance ** 2

  # replaced with 
  msg.covariance[0] = ubisense_msg.variance
  msg.covariance[7] = ubisense_msg.variance
  msg.covariance[14] = ubisense_msg.variance
```
Since we are already getting the varince from the Ubisense server, we can use it directly in the covariance matrix. The covariance matrix is a 6x6 matrix, where the diagonal elements are the variance of the x,y,z and the orientation. Assuming the variance in x and y direction are the same


<hr>
<br>

## Installation

- The code is developed using the colcon build system. The repository is just the package from the workspace.