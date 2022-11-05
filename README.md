<div align="center">
<h1>Ros: publisher & subscriber</h1>
</div>
Głównym celem jest przedstawienie i porównanie kodu dwóch węzłów: węzeł wysyłający dane (publisher) oraz węzeł słuchający te dane (subscriber) w programie ROS. 

- [Porównanine](#porównanie)
- [Publisher](#publisher)
	- [Krok po kroku](#krok-po-kroku)
	- [Sprawdzamy wynik](#zobaczmy-jak-to-działa)
- [Subscriber](#subscriber)
	- [Krok po kroku](#krok-po-kroku-1)
- [Węzły w postaci grafów](#grafy)

# Porównanie
<table>
<tr>
<th>Publisher</th>
<th>Subscriber</th>
</tr>
<tr>
<td>
  
```py
#!/usr/bin/env python3

import rospy
from std_msgs.msg import String

if __name__ == '__main__':
	rospy.init_node('transmitter')
	pub = rospy.Publisher("/radio", String, queue_size=10)
	rate = rospy.Rate(2)
	while not rospy.is_shutdown():
		msg = String()
		msg.data = "Hi from the robot  radio!"
		pub.publish(msg)
		rate.sleep()
```
  
</td>
<td>

```py
#! /usr/bin/env python3

import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo(msg)

if __name__== '__main__':
    rospy.init_node("listener")
    sub = rospy.Subscriber("/radio", String, callback)
    rospy.spin()
```

</td>
</tr>
</table>

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
Gratulacje, publisher jest gotowy

# Subscriber
### Krok po kroku
Program dla `subscriber` prawie nie różni się od `publisher`. Musimy zainportować takie same biblioteki - `rospy` oraz `std_msgs`. Subscriber tworzy się za pomocą `rospy.Subscriber()`, nadajemy nazwę "kanału" od którego węzeł będzie otrzymywał informację, typ danych oraz funkcję, która przyjmuje `data`.

Trzeba zwrócić uwagę, że jeżeli nadamy typ danych, który nie jest zgodny z typem przychodzących informacji, to dane nie zostaną zarejestrowane. 

Funkcja `callback` przyjmuje dane, więc w dowolny sposób mozemy nimi operować, w naszym przypadku po prostu pokazujemy je w terminalu. 

```py
#! /usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(msg):
    rospy.loginfo("Message received : ")
    rospy.loginfo(msg)

if __name__== '__main__':
    rospy.init_node("listener")							# inicjacja węzła
    sub = rospy.Subscriber("/radio", String, callback)				# tworzy moduł, który przyjmuje dane od stworzonego wcześniej "kanału"
    	# /radio - nazwa "kanału" z którego będzie pobierana informacja
    	# String - typ danych
    	# callback - funkcja, która jest wywołana kiedy przychodzą nowe dane
    rospy.spin()
```


# Grafy
Mając dwa węzły, możemy przedstawić ich w postaci grafów, żeby zobaczyć ich połączenie w grafie węzłów. 
```
$ source catkin_ws/devel/setup.bash
$ rosrun my_robot_tutorials transmitter.py
$ rosrun my_robot_tutorials Listener.py 
$ rosrun rqt_graph rqt_graph
```
Jak widać dwa węzły są związane przez kanał `/radio`. 
<img title="graph" alt="graph" src="/pic/node_graph.png">

# Wniosek

