# UbiSense ROS2 Node

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

## Installation

- The code is developed using the colcon build system. The repository is just the package from the workspace.