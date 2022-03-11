# Skripsi

See documentation on my Notion
<br>
https://dramatic-middle-7c9.notion.site/SKRIPSI-c7b654bd0ebc4cd78e7dc0772060c5e5
<br>
## How to Run Robot
<br>
<ul>
  <li>Terminal 1: roscore</li>
  <li>Terminal 2: cd ~/catkin_ws/src/enoid_interface</li>
  <li>run python3 -m http.server</li>
  <li>Terminal 3: rosrun rosbridge_server rosbridge_websocket</li>
  <li>Terminal 4: rosrun enoid_walk walk_node.py</li>
  <li>if permission USB0 error, sudo chmod a+rw /dev/ttyUSB0</li>
  <li>Terminal 5: rosrun enoid_orientation mpu_node.py</li>
  <li>Open web browser, type 192.168.137.93:8000 *localhost:port</li>
</ul>

Author : Mikael Nias
