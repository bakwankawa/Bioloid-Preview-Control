#! /usr/bin/python3

import rospy

from preview_controller import *
from inverse_kinematic import *
from servo_controller import *

FOOT_DISTANCE = 6.5 / 1000
COM_HEIGHT = 230 / 1000
X_OFFSET = 5 / 1000
COM_SWING = 115.5 / 1000

COM = np.matrix([X_OFFSET, 0.0, COM_HEIGHT])
LEFT = np.matrix([0.0, 38.5 / 1000 + FOOT_DISTANCE, 0.0])
RIGHT = np.matrix([0.0, -38.5 / 1000 - FOOT_DISTANCE, 0.0])

def stand(ik):
    global JOINTS, finish, step, delta_step, COM, LEFT, RIGHT,  com_msg
    COM = np.matrix([X_OFFSET, 0.0, COM_HEIGHT])
    LEFT = np.matrix([0.0, 38.5 / 1000 + FOOT_DISTANCE, 0.0])
    RIGHT = np.matrix([0.0, -38.5 / 1000 - FOOT_DISTANCE, 0.0])
    JOINTS = ik.solve(COM, LEFT, RIGHT)

def walk(x_move):
    global JOINTS, COM, LEFT, RIGHT

    pc.update_walking_pattern() 
    JOINTS = ik.solve(pc.com_pose, pc.l_foot, pc.r_foot)

def main():
    global JOINTS
    rospy.init_node('sevo_test', anonymous=False)
    rate = rospy.Rate(30)

    pc = PreviewControl(COM_HEIGHT, COM_SWING, X_OFFSET, FOOT_DISTANCE)
    ik = InverseKinematic()
    sc = ServoController()

    stand(ik)

    n = 0
    dt = 0.01

    while not rospy.is_shutdown():
        walk(0.25)

        sc.sync_write_pos(JOINTS)

        if (n%200 == 0):
            sc.sync_read_pos()
        
        n += dt

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass