#!/usr/local/bin/python

import RPi.GPIO as GPIO
import time
import rospy
from std_msgs.msg import Int32

GPIO.setmode(GPIO.BCM)

#define the pin that goes to the circuit
pin_to_circuit = 4

com_pub = rospy.Publisher('fsr_2', Int32, queue_size=1)


def rc_time (pin_to_circuit):
    count = 0

    #Output on the pin for
    GPIO.setup(pin_to_circuit, GPIO.OUT)
    GPIO.output(pin_to_circuit, GPIO.LOW)
    time.sleep(0.1)

    #Change the pin back to input
    GPIO.setup(pin_to_circuit, GPIO.IN)

    #Count until the pin goes high
    while (GPIO.input(pin_to_circuit) == GPIO.LOW):
        count += 1
        if (count > 1000):
            print("ANGKAT KIRI " + "I" * 10)
            com_pub.publish(0)
        else:
            print('NAPAK KIRI' + "I" * 10)
            com_pub.publish(1)

def toggle_check():
    time.sleep(2/1000)

    return True

def main():
    rospy.init_node('fsr1_node', anonymous=False)
    rate = rospy.Rate(30)

    while not rospy.is_shutdown():
        value = rc_time(pin_to_circuit)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()
