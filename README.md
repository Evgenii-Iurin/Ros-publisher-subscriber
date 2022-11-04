<div align="center">
<h1>Ros-publisher-subscriber</h1>
</div>

- [Publisher](#publisher)

Głównym celem jest przedstwienie oraz porównanie dwóch węzłów: węzeł wysyłający dane (publisher) oraz węzeł słuchający te dane (subscriber) w programie ROS. 


## Publisher

Zacznyjmy od importowania potrzebnych bibliotek - `rospy` oraz `std_msgs`. 
```py
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

if __name__ == '__main__':
	rospy.init_node('robot_news_radio_transmitter')
	pub = rospy.Publisher("/robot_news_radio", String, queue_size=10)
	rate = rospy.Rate(2)
	while not rospy.is_shutdown():
		msg = String()
		msg.data = "Hi from the robot news radio!"
		pub.publish(msg)
		rate.sleep()
	rospy.logininfo("Node was stopped")
```
