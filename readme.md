## Issues 

1. Issue wehre the socket/port is not being closed properly when the node is being shutdown.
This causes the port to be occupied and doesnot allow the code to be rerun. To fix this issue we need to close the udp socket properly. This chould be implemented in the shutdown method of the node.(probably by overdiding it)



# Fixes

1. The publisher node was inheriting another node(ubisense). To fix this the `ubisense`was converted into a normal class, with the relevant methods to be called. 