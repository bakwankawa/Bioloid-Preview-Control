#! /usr/bin/python3

from turtle import left
import rospy
from geometry_msgs.msg import Vector3, Twist
from std_msgs.msg import Int32, Bool, Float32

from preview_controller import *
from inverse_kinematic import *
from servo_controller import *
from fuzzy_logic import *
from rot_mat import *

ori_data = np.array([.0, .0, .0])
gyr_data = np.array([.0, .0, .0])
igl_data = np.array([.0, .0, .0])
comp_data = np.array([.0, .0, .0])

cmd_vel = Vector3()
gain_ctrl = Twist()
feedback_mode = Int32()
walk_mode = Int32()
walk_cmd = Int32()
push_data = Bool()
com_msg = Vector3()
com_real_msg = Vector3()
vel_msg = Vector3()
offset_status = Int32()
offset_param = Vector3()
# fsr1 = Int32()
# fsr2 = Int32()
fsr1 = Float32()
fsr2 = Float32()

FOOT_DISTANCE = 7.5 / 1000
COM_HEIGHT = 230 / 1000
X_OFFSET = 5 / 1000        # 18 / 1000
Y_OFFSET = -2 / 1000
COM_SWING = 115.5 / 1000

COM = np.matrix([X_OFFSET, Y_OFFSET, COM_HEIGHT])
LEFT = np.matrix([0.0, 38.5 / 1000 + FOOT_DISTANCE, 0.0])
RIGHT = np.matrix([0.0, -38.5 / 1000 - FOOT_DISTANCE, 0.0])

uCOM = np.matrix([X_OFFSET, Y_OFFSET, COM_HEIGHT])
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
ssp_done = False
x0 = np.matrix(np.zeros(3)).T
y0 = np.matrix(np.zeros(3)).T
x0_real = np.matrix(np.zeros(3)).T
y0_real = np.matrix(np.zeros(3)).T
# error_com = []
com_real = np.array(np.matrix([.0, .0, .0]).T)

def ori_callback(msg):
    ori_data[0] = msg.x
    ori_data[1] = msg.y 

def gyr_callback(msg):
    gyr_data[0] = msg.x 
    gyr_data[1] = msg.y

def comp_callback(msg):
    comp_data[0] = msg.x 
    comp_data[1] = msg.y
    comp_data[2] = msg.z

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
    global fsr1
    fsr1 = msg

def fsr2_callback(msg):
    global fsr2
    fsr2 = msg

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

def Rx(theta):
    return np.matrix([[1, 0, 0],
                    [0, np.cos(theta), -np.sin(theta)],
                    [0, np.sin(theta), np.cos(theta)]])


def Ry(theta):
    return np.matrix([[np.cos(theta), 0, np.sin(theta)],
                    [0, 1, 0],
                    [-np.sin(theta), 0, np.cos(theta)]])


def Rz(theta):
    return np.matrix([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]])

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

    com_real_msg.x = COM[0,0] * ori_data[0]
    com_real_msg.y = COM[0,1] * ori_data[1]
    com_real_msg.z = COM[0,2] * ori_data[2]


def walk_test(pc, ik):
    global JOINTS, COM, LEFT, RIGHT, com_msg

    pc.update_walking_pattern()

    if pc.walking_ready and not debug:
        JOINTS = ik.solve(pc.com_pose, pc.l_foot, pc.r_foot)
        com_msg.x = pc.com_pose[0,0]
        com_msg.y = pc.com_pose[0,1]
        com_msg.z = pc.com_pose[0,2]
        
        com_real_msg.x = pc.com_pose[0,0] * ori_data[0]
        com_real_msg.y = pc.com_pose[0,1] * ori_data[1]
        com_real_msg.z = pc.com_pose[0,2] * ori_data[2]
    else:
        JOINTS = ik.solve(COM, LEFT, RIGHT)
    
        com_msg.x = COM[0,0]
        com_msg.y = COM[0,1]
        com_msg.z = COM[0,2]

        com_real_msg.x = COM[0,0] * ori_data[0]
        com_real_msg.y = COM[0,1] * ori_data[1]
        com_real_msg.z = COM[0,2] * ori_data[2]


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

def velocity_step(pc, rm, time_step):
    global x0, y0, x0_real, y0_real, com_real
    # current_com = np.array(np.matrix([.0, .0, .0]).T)

    # current_com = rm.Rz(0) * rm.Ry(ori_data[1]) * rm.Rx(ori_data[0]) * np.matrix([pc.com_pose[0,0], pc.com_pose[0,1], pc.com_pose[0,2]]).T

    v_com_x = (pc.x[0,0] - x0[0,0]) / time_step
    # v_com_y = (pc.y[0,0] - y0[0,0]) / time_step

    # v_com_real_x = ((pc.x[0,0] * gyr_data[0]) - x0_real[0,0]) / time_step  # NI DPT DARI MANA DA KOK DIKALI GYRO
    # v_com_real_y = ((pc.y[0,0] * gyr_data[1]) - y0_real[0,0]) / time_step
    # v_com_real_x = ((current_com[0,0]) - com_real[0,0]) / time_step

    # v_com_error_x = v_com_real_x - v_com_x
    # v_com_error_x = v_com_real_x - v_com_x

    # vel_msg.x = v_com_error_x
    vel_msg.x = v_com_x
    # print(f"v_com_x : {v_com_x}, v_com_y : {v_com_y}")
    # print(f"pc.x : {v_com_x}, x0 : {v_com_real_x}")
    # rospy.loginfo("v_com_x: %s", str(v_com_x))
    # rospy.loginfo("v_error_x: %s", str(v_com_x))
    # return v_com_error_x
    return v_com_x

def velocity_feedback(pc, rm, error_v_com):
    global error_com, vel_threshold
    # com_real = rm.Rz(0) * rm.Ry(ori_data[1]) * rm.Rx(ori_data[0]) * np.matrix([pc.com_pose[0,0], pc.com_pose[0,1], pc.com_pose[0,2]]).T
    # error_com = com_real[0,0] - pc.com_pose[0,0]

    if abs(error_v_com) > vel_threshold: # belum di def gain utk cmd x offset
        pc.cmd_x_offset = 0.005

    # cara kedua bisa tambah offset com_pose sebelum di sikat servo
    # cara ketiga ada di komen bawah

def main():
    global X_OFFSET, COM, com_msg, cmd_vel, offset_param, fsr1, fsr2, dsp, ssp, ssp_done, x0, y0, x0_real, y0_real, vel_threshold, com_real
    rospy.init_node('velocityControl_walk', anonymous=False)
    rospy.Subscriber("ori_data", Vector3, ori_callback)
    rospy.Subscriber("gyr_data", Vector3, gyr_callback)
    rospy.Subscriber("walk_vel", Vector3, vel_callback)
    rospy.Subscriber("walk_cmd_status", Int32, cmd_callback)
    rospy.Subscriber("walk_mode_status", Int32, walk_mode_callback)
    rospy.Subscriber("feedback_status", Int32, feedback_callback)
    rospy.Subscriber("gain_control", Twist, gain_callback)
    rospy.Subscriber("offset_status", Int32, offset_status_callback)
    rospy.Subscriber("offset_param", Vector3, offset_callback)
    rospy.Subscriber("comp_data", Vector3, comp_callback)
    # rospy.Subscriber("fsr1", Int32, fsr1_callback)
    # rospy.Subscriber("fsr2", Int32, fsr2_callback)
    rospy.Subscriber("fsr1", Float32, fsr1_callback)
    rospy.Subscriber("fsr2", Float32, fsr2_callback)
    com_pub = rospy.Publisher('com_data', Vector3, queue_size=1)
    com_real_pub = rospy.Publisher('com_real_data', Vector3, queue_size=1)
    vel_pub = rospy.Publisher('vel_data', Vector3, queue_size=1)

    rate = rospy.Rate(30)

    pc = PreviewControl(COM_HEIGHT, COM_SWING, X_OFFSET, FOOT_DISTANCE)
    ik = InverseKinematic()
    sc = ServoController()
    fz = FuzzyLogic()
    rm = RotMat()
    
    ik.TILT = 15
    stand(ik)

    # TUNNING ISENG VC
    # pc.swing_height = 0.045
    # pc.t_step = 0.2
    # pc.dsp_ratio = 0.07
    left_first = True
    right_first = False
    right_done = False
    vel_threshold = 0.01
    vel_error = 0.0

    time_step_vel = 0.0
    # time_step_done = 0.0

    rospy.loginfo("E-NOID WALK")

    while not rospy.is_shutdown():

        # ganti parameter v dari websocket
        pc.cmd_x = cmd_vel.x
        # rospy.loginfo("cmd_x: %s", str(cmd_vel.x))
        # rospy.loginfo("fsr1_data: %s", str(fsr1.data))
        # rospy.loginfo("hip_offset: %s", str(offset_param.x))
        # rospy.loginfo("pc.x: %s", str(pc.x))

        # tunning offset robot
        ik.hip_offset = -offset_param.x * np.pi / 180
        ik.ankle_offset = -offset_param.y * np.pi / 180
        # ik.TILT = offset_param.z * np.pi / 180

        if walk_cmd.data == -1:

            # print("RESET WALK")
            pc = PreviewControl(COM_HEIGHT, COM_SWING, X_OFFSET, FOOT_DISTANCE)
            stand(ik)


        elif walk_cmd.data == 1:

            # if walk_mode.data == 1 and (feedback_mode.data == 2):
            if walk_mode.data == 1:
                ik.TILT = 15
                X_OFFSET = 18 /1000
                # velocity_feedback(pc, rm, vel_error)
                walk_test(pc, ik)
            else:
                stand(ik)
                ik.TILT = 15
                X_OFFSET = 18 /1000

                
            if ((feedback_mode.data == 1) or (feedback_mode.data == 2)) and not(finish):
                # print(ori_data[1] * 180/np.pi)
                # print(gyr_data[1] * 180/np.pi)
                
                # print("ANKLE")
                igl_data[1] +=  ori_data[1]/30

                delta_roll = gain_ctrl.linear.x * ori_data[0] + gain_ctrl.angular.x * gyr_data[0]
                # delta_roll2 = gain_ctrl.linear.z * ori_data[0] + gain_ctrl.angular.x * gyr_data[0]
                delta_pitch = -gain_ctrl.linear.y * ori_data[1] + -gain_ctrl.angular.y * gyr_data[1] + gain_ctrl.angular.x * igl_data[1]
                JOINTS[3] += delta_pitch
                JOINTS[8] -= delta_pitch
                # JOINTS[4] += delta_roll2
                JOINTS[9] -= delta_roll

        if fsr2.data > 35 and left_first == True:
            if right_done == True:
                time_step_vel = rospy.Time.now().to_sec() - start_step
                # x[0][1] = pc.x[0,0]
                # y[0][1] = pc.y[0,0]
                vel_error = velocity_step(pc, rm, time_step_vel)
                """bikin if error meleibihi tresshold v, panggil ulang IKSolve (buat ambil joint invers dari com pose after error)
                # pc.com_pose += v_com_error_x * gain
                # QUESTIONS: COM dinamis gini mengaruhi r foot dan l foot yg akan dieksekusi servo gak?
                # JOINTS = ik.solve(pc.com_pose, pc.l_foot, pc.r_foot)"""
                right_done = False

            start_step = rospy.Time.now().to_sec()
            x0 = pc.x
            y0 = pc.y
            # x0_real = pc.x * gyr_data[0]
            # y0_real = pc.y * gyr_data[1]
            # com_real = rm.Rz(0) * rm.Ry(ori_data[1]) * rm.Rx(ori_data[0]) * np.matrix([pc.com_pose[0,0], pc.com_pose[0,1], pc.com_pose[0,2]]).T
            right_first = True
            left_first = False
        elif fsr1.data > 40 and right_first == True:
            time_step_vel = rospy.Time.now().to_sec() - start_step
            vel_error = velocity_step(pc, rm, time_step_vel)
            right_done = True
            start_step = rospy.Time.now().to_sec()
            x0 = pc.x
            y0 = pc.y
            # x0_real = pc.x * gyr_data[0]
            # y0_real = pc.y * gyr_data[1]
            # com_real = rm.Rz(0) * rm.Ry(ori_data[1]) * rm.Rx(ori_data[0]) * np.matrix([pc.com_pose[0,0], pc.com_pose[0,1], pc.com_pose[0,2]]).T
            right_first = False
            left_first = True

        sc.sync_write_pos(JOINTS)
        
        com_pub.publish(com_msg)
        # com_real_pub.publish(com_real_msg)
        vel_pub.publish(vel_msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
