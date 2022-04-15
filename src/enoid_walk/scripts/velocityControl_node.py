#! /usr/bin/python3

import rospy
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Int32, Bool

from preview_controller import *
from inverse_kinematic import *
from servo_controller import *
from fuzzy_logic import *

ori_data = np.array([.0, .0, .0])
gyr_data = np.array([.0, .0, .0])
igl_data = np.array([.0, .0, .0])

cmd_vel = Vector3()
gain_ctrl = Twist()
feedback_mode = Int32()
walk_mode = Int32()
walk_cmd = Int32()
push_data = Bool()
com_msg = Vector3()
offset_status = Int32()
offset_param = Vector3()
fsr_1 = Int32()
fsr_2 = Int32()

FOOT_DISTANCE = 6.5 / 1000
COM_HEIGHT = 230 / 1000
X_OFFSET = 5 / 1000
COM_SWING = 115.5 / 1000

COM = np.matrix([X_OFFSET, 0.0, COM_HEIGHT])
LEFT = np.matrix([0.0, 38.5 / 1000 + FOOT_DISTANCE, 0.0])
RIGHT = np.matrix([0.0, -38.5 / 1000 - FOOT_DISTANCE, 0.0])

uCOM = np.matrix([X_OFFSET, 0.0, COM_HEIGHT])
uLEFT = np.matrix([0.0, 38.5 / 1000 + FOOT_DISTANCE, 0.0])
uRIGHT = np.matrix([0.0, -38.5 / 1000 - FOOT_DISTANCE, 0.0])

debug = False
flag = False
step = False
finish = False
JOINTS = np.zeros(12)

start = 0.0
now = 0.0
delta_step = 0.0

# velocity control
dsp = True
ssp = False
x = np.zeros((2,2))
y = np.zeros((2,2))
v_com_x = np.zeros((2,2))
v_com_y = np.zeros((2,2))

def ori_callback(msg):
    ori_data[0] = msg.x
    ori_data[1] = msg.y 

def gyr_callback(msg):
    gyr_data[0] = msg.x 
    gyr_data[1] = msg.y

def vel_callback(msg):
    global cmd_vel
    cmd_vel = msg

def cmd_callback(msg):
    global walk_cmd
    walk_cmd = msg

def walk_mode_callback(msg):
    global walk_mode
    walk_mode = msg

def feedback_callback(msg):
    global feedback_mode
    feedback_mode = msg

def gain_callback(msg):
    global gain_ctrl
    gain_ctrl = msg

def push_callback(msg):
    global push_data
    push_data = msg

def offset_status_callback(msg):
    global offset_status
    offset_status = msg

def offset_callback(msg):
    global offset_param
    offset_param = msg

def fsr1_callback(msg):
    global fsr_1
    fsr_1 = msg

def fsr2_callback(msg):
    global fsr_2
    fsr_2 = msg

def bezier_curve(phase, p_start, p_cnt, p_end):
    if(phase >= 1):
        phase = 1

    t = np.matrix([1, phase,  phase**2])
    coef = np.matrix([[1, 0, 0],
                      [-2, 2, 0],
                      [1, -2, 1]])
    point = np.vstack((p_start, p_cnt, p_end))

    path = t * coef * point

    return path

def stand(ik):
    global JOINTS, finish, step, delta_step, COM, LEFT, RIGHT,  com_msg
    COM = np.matrix([X_OFFSET, 0.0, COM_HEIGHT])
    LEFT = np.matrix([0.0, 38.5 / 1000 + FOOT_DISTANCE, 0.0])
    RIGHT = np.matrix([0.0, -38.5 / 1000 - FOOT_DISTANCE, 0.0])
    JOINTS = ik.solve(COM, LEFT, RIGHT)
    finish = False
    step = False
    delta_step = 0.0
    com_msg.x = COM[0,0]
    com_msg.y = COM[0,1]
    com_msg.z = COM[0,2]


def walk_test(pc, ik):
    global JOINTS, COM, LEFT, RIGHT, com_msg

    pc.update_walking_pattern()

    if pc.walking_ready and not debug:
        JOINTS = ik.solve(pc.com_pose, pc.l_foot, pc.r_foot)
        com_msg.x = pc.com_pose[0,0]
        com_msg.y = pc.com_pose[0,1]
        com_msg.z = pc.com_pose[0,2]
    else:
        JOINTS = ik.solve(COM, LEFT, RIGHT)
    
        com_msg.x = COM[0,0]
        com_msg.y = COM[0,1]
        com_msg.z = COM[0,2]


def push_test(fz, ik):
    global flag, finish, delta_step,step, start, now, JOINTS, COM, LEFT, RIGHT,uCOM, uLEFT, uRIGHT, com_msg
    time_step = .1

    if finish == False:

        if not(push_data.data) and not(step):
            # print("IN 1")
            JOINTS = ik.solve(COM, LEFT, RIGHT)
            start = rospy.Time.now().to_sec()
            delta_step = fz.compute(-ori_data[1] * 180 /
                                    np.pi, gyr_data[1] * 180 / np.pi)/100
            uCOM = COM
            uLEFT = LEFT
            uRIGHT = RIGHT
            # delta_step = 0.07
        elif not(finish):
            # print("STEP")
            now = rospy.Time.now().to_sec() - start
            phase = now / time_step
            
            if phase >= 1:
                phase = 1
                finish = True

            COM = bezier_curve(phase, uCOM, np.matrix(
                [X_OFFSET + (delta_step / 4.0), -0.03, COM_HEIGHT]), np.matrix([X_OFFSET + (delta_step / 2.0), -0.005, COM_HEIGHT]))

            if True:
                LEFT = bezier_curve(phase, uLEFT, np.matrix(
                    [(delta_step / 2.0), 48/1000, 0.08]), np.matrix([(delta_step), 58/1000, 0.0]))
            else:
                RIGHT = bezier_curve(phase, uRIGHT, np.matrix(
                    [(delta_step / 2.0), -48/1000, 0.05]), np.matrix([(delta_step), -48/1000, 0.05]))

            JOINTS = ik.solve(COM, LEFT, RIGHT)

            step = True

        com_msg.x = COM[0,0]
        com_msg.y = COM[0,1]
        com_msg.z = COM[0,2]

    
def final_test(pc, fz, ik):
    global flag, finish, delta_step,step, start, now, JOINTS, COM, LEFT, RIGHT, uCOM, uLEFT, uRIGHT, push_data, com_msg
    time_step = .1
    pc.update_walking_pattern()
    if finish == False:
        if pc.walking_ready and not debug:
            if not(push_data.data) and not(step):
                # print("WALK")
                JOINTS = ik.solve(pc.com_pose, pc.l_foot, pc.r_foot)
                start = rospy.Time.now().to_sec()
                delta_step = fz.compute(-ori_data[1] * 180 /
                                                np.pi, gyr_data[1] * 180 / np.pi)/100
                uCOM = pc.com_pose
                uLEFT = pc.l_foot
                uRIGHT = pc.r_foot

                com_msg.x = uCOM[0,0]
                com_msg.y = uCOM[0,1]
                com_msg.z = uCOM[0,2]

            elif not(finish):
                # print("STEP")
                now = rospy.Time.now().to_sec() - start
                phase = now / time_step
                
                if phase >= 1:
                    phase = 1
                    finish = True

                COM = bezier_curve(phase, uCOM, np.matrix(
                    [X_OFFSET + (delta_step / 4.0), -0.03, COM_HEIGHT]), np.matrix([X_OFFSET + (delta_step / 2.0), 0.01, COM_HEIGHT]))

                if pc.support_foot == -1:
                    LEFT = bezier_curve(phase, uLEFT, np.matrix(
                        [(delta_step / 2.0),  48/1000, 0.08]), np.matrix([(delta_step),  62/1000, 0.0]))
                else:
                    RIGHT = bezier_curve(phase, uRIGHT, np.matrix(
                        [(delta_step / 2.0), -48/1000, 0.08]), np.matrix([(delta_step), -62/1000, 0.0]))

                JOINTS = ik.solve(COM, LEFT, RIGHT)

                step = True

                com_msg.x = COM[0,0]
                com_msg.y = COM[0,1]
                com_msg.z = COM[0,2]
        else:
            JOINTS = ik.solve(COM, LEFT, RIGHT)
            com_msg.x = COM[0,0]
            com_msg.y = COM[0,1]
            com_msg.z = COM[0,2]

def velocity_step(pc):
    global v_com_x, v_com_y, x, y, time_step_done
    v_com_x = (x[0][1] - x[0][0]) / time_step_done
    v_com_y = (y[0][0] - y[0][1]) / time_step_done
    
    print(f"v_com_x : {v_com_x}, v_com_y : {v_com_y}")

def main():
    global X_OFFSET, COM, com_msg, cmd_vel, offset_param, fsr_1, fsr_2, dsp, ssp, x, y, time_step_done
    rospy.init_node('velocityControl_walk', anonymous=False)
    rospy.Subscriber("ori_data", Vector3, ori_callback)
    rospy.Subscriber("gyr_data", Vector3, gyr_callback)
    rospy.Subscriber("walk_vel", Vector3, vel_callback)
    rospy.Subscriber("walk_cmd_status", Int32, cmd_callback)
    rospy.Subscriber("walk_mode_status", Int32, walk_mode_callback)
    rospy.Subscriber("feedback_status", Int32, feedback_callback)
    rospy.Subscriber("gain_control", Twist, gain_callback)
    rospy.Subscriber("offset_status", Int32, offset_status_callback)
    rospy.Subscriber("offset_param", Twist, offset_callback)
    rospy.Subscriber("fsr_1", Int32, fsr1_callback)
    rospy.Subscriber("fsr_2", Int32, fsr2_callback)
    com_pub = rospy.Publisher('com_data', Vector3, queue_size=1)

    rate = rospy.Rate(30)

    pc = PreviewControl(COM_HEIGHT, COM_SWING, X_OFFSET, FOOT_DISTANCE)
    ik = InverseKinematic()
    sc = ServoController()
    fz = FuzzyLogic()
    
    ik.TILT = 10
    stand(ik)

    rospy.loginfo("E-NOID WALK")

    n = 0
    time_step_done = 0

    while not rospy.is_shutdown():

        # ganti parameter v dari websocket
        pc.cmd_x = cmd_vel.x
        rospy.loginfo("cmd_x: %s", str(cmd_vel.x))

        # tunning offset robot
        ik.hip_offset = offset_param.x * np.pi / 180
        ik.ankle_offset = offset_param.y * np.pi / 180
        ik.TILT = offset_param.z * np.pi / 180
        
        if walk_cmd.data == -1:

            # print("RESET WALK")
            pc = PreviewControl(COM_HEIGHT, COM_SWING, X_OFFSET, FOOT_DISTANCE)
            stand(ik)

        elif walk_cmd.data == 1:

            if walk_mode.data == 1:
                X_OFFSET = 18 /1000
                walk_test(pc, ik)
            else:
                stand(ik)
                X_OFFSET = 5 /1000

        sc.sync_write_pos(JOINTS)

        com_pub.publish(com_msg)

        if fsr_1.data == 1 and fsr_2.data == 1 and dsp == True:
            time_step_done = time_step
            print(velocity_step(pc))
            dsp = False
            ssp = True
            start_step = rospy.Time.now().to_sec()
            x[0][0] = pc.x[0,0]
            y[0][0] = pc.y[0,0]
        elif ((fsr_1.data == 1 and fsr_2.data == 0) or (fsr_1.data == 0 and fsr_2.data == 1) and ssp == True):
            time_step = rospy.Time.now().to_sec() - start_step
        else:
            x[0][1] = pc.x[0,0]
            y[0][1] = pc.y[0,0]
            dsp = True
            ssp = False


        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
