<div align="center">
<h1>Ros: publisher & subscriber</h1>
</div>
Głównym celem jest przedstawienie i porównanie kodu dwóch węzłów: węzeł wysyłający dane (publisher) oraz węzeł słuchający te dane (subscriber) w programie ROS. 

- [Publisher](#publisher)
	- [Krok po kroku](#krok-po-kroku)
	- [Sprawdzamy czy działa](#zobaczmy-jak-to-działa)

# Publisher

### Krok po kroku

- ##### <ins>BIBLIOTEKI</ins>

Zacznyjmy od importu potrzebnych bibliotek - `rospy` oraz `std_msgs`. `Std_msgs` to jest pakiet, który zawiera w sobie różne typy danych dla przedstawienia inforamcji, którą wysyłamy. W danym przypadku będziemy wysyłali zwykłą wiadomość, czyli typu `String`. 
```py
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
```
- ##### <ins>KANAŁ</ins>

Do stworzenia publisher'a potrzebujemy jednej linijki: `pub = rospy.Publisher("/radio", String, queue_size=10)`, która tworzy "kanał" na który `publisher` wysyła dane a `subscriber` pobiera te dane. To znaczy, że `subscriber` nie może bezpośrednio otrzymywać dane od jednego z węzłów, tylko przez "kanały". W naszym przypadku tym kanałem będzie "/radio". Także musimy ustawić typ danych wiadomości oraz rozmiar buforu.
- ##### <ins>CZĘSTOTLIWOŚĆ</ins>
Następnym krokiem ustawiamy częstotliwość, z którą dane będą wysyłać się do kanału. 
- ##### <ins>TWORZENIE I WYSŁANIE WIADOMOŚCI</ins>
Ostatnim krokiem jest stworzenie wiadomości i wysłanie za pomocą `msg.publish()`
```py
if __name__ == '__main__':							# ta linijka jest opcjonalna. Ona nie ma nic wspólnego z ROSem. Ten kod	
										# będzie działać i bez tego
	rospy.init_node('transmitter')						# Inicjacja nazwy węzła
	pub = rospy.Publisher("/radio", String, queue_size=10)			# "/Radio" - "mostek" przez który przepływają dane
										# String - typ danych wiadomości
										# queue_size - rozmiar buforu
	rate = rospy.Rate(2)							# Częstotliwość wysyłania danych - 2 Hz
	while not rospy.is_shutdown():						# Póki węzeł jest aktywny
		msg = String()							
		msg.data = "Hello from the robot radio!"			# Treść wiadomości, którą wysyłamy
		pub.publish(msg)						# wysyła wiadomość do "mostka", czyli do "/Radio", stworzonego przez nas
		rate.sleep()							 
	rospy.logininfo("Node was stopped")					# opcjonalna linijka informująca, że węzeł został zatrzymany
```
### Zobaczmy, jak to działa
Teraz możemy zobaczyć czy działa nam wysłanie danych
```
$ roscore
$ python3 transmitter.py							# uwaga: musimy dać komendę [chmod -x transmitter.py], żeby kod był
										# wykonywalny
```
W terminalu nic nie będzie wyświetlane, dlatego, że nie mamy sluchacza,  nie musimy tworzyć go, możemy to zrobić za pomocą `rostopic`.
```
$ rostopic echo /radio
>>> data: "Hello from the robot radio!"
>>> data: "Hello from the robot radio!"
```
Gratulacje, stworzyliśmy publisher

# Subscriber
```py
#! /usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback_receive_radio_data(msg):
    rospy.loginfo("Message received : ")
    rospy.loginfo(msg)

if __name__== '__main__':
    rospy.init_node("smartphone")						 # init node name
    sub = rospy.Subscriber("/radio", String, callback_receive_radio_data)
    rospy.spin()
```
