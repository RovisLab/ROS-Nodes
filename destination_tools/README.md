# destination_tools
  
  This folder contains packages for the destination point of the robots.
  We have one package that contains the type of message transmitted via the topic named "coord_topic".
  The "coord_sender" package advertise to the upper topic the "x, y" coordinates of the goal.For now these coordinates are hardcoded,but in the future the node will be modified to subscribe to the topic of the object recognition node in order to publish the correct coordinates.
  the "multi_nav" packages subscribes to our topic and sends the data to the "/move_base_simple/goal" using the move_base action server.

