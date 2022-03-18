"""THIS IS TROUBLESHOOTING DOCUMENTATION WHILE WORKING ON THIS THESIS"""

"""
1.  Q: Selalu menjalankan command permission "sudo chmod 777 /dev/USB0"
    A: Menambahkan USER ke dialout group.
        -> Verifikasi grup mana yang memiliki akses ke device : ls -l /dev/ttyACM0 /dev/ttyUSB0
        -> Menambahkan user ke group : sudo usermod -a -G dialou USER
    S: https://answers.ros.org/question/63813/how-can-i-shell-programming-in-launch-file/

2.  Q: Menambahkan .py file untuk di launch di .launch ROS
    A: <node pkg="xsens_driver" name="mtnode" type="mtnode.py" output="screen"> 
        ..
       </node>
        Dengan type adalah nama .py file nya
    S: https://answers.ros.org/question/229841/how-launch-a-python-script-with-roslaunch/

3.  Q: Dapat representasi matriks untuk bezier pada fungsi preview_control.update_footstep dari mana?
    A: P = (1−t)^2P1 + 2(1−t)tP2 + t^2P3
    S: https://javascript.info/bezier-curve
       https://towardsdatascience.com/b%C3%A9zier-curve-bfffdadea212 eq.6

4.  Q: Ip mikael@ip
    A: ssh mikael@192.168.137.154
       ssh mikael@192.168.137.93
"""