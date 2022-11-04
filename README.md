<div align="center">
<h1>Ros-publisher-subscriber</h1>
</div>

- [Publisher](#publisher)
	- Krok po kroku
	- Cały kod

Głównym celem jest przedstwienie oraz porównanie kodu dwóch węzłów: węzeł wysyłający dane (publisher) oraz węzeł słuchający te dane (subscriber) w programie ROS. 


## Publisher

### Krok po kroku
Zacznyjmy od importu potrzebnych bibliotek - `rospy` oraz `std_msgs`. Std_msgs to jest pakiet, który zawiera w sobie różne typy danych dla inforamcji, którą wysyłamy. W danym przypadku będziemy wysyłali zwykłą wiadomość, czyli typu `String`. 
```py
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
```

```py
if __name__ == '__main__':							# ta linijka jest opcjonalna. Ona nie ma nic wspólnego z ROSem. Ten kod	
										# będzie działać bez tego
	rospy.init_node('transmitter')
	pub = rospy.Publisher("/robot_news_radio", String, queue_size=10)
	rate = rospy.Rate(2)
	while not rospy.is_shutdown():
		msg = String()
		msg.data = "Hi from the robot news radio!"
		pub.publish(msg)
		rate.sleep()
	rospy.logininfo("Node was stopped")
```
