#! /usr/bin/python3

from dynamixel_sdk import *             

class ServoController:
  
    def __init__(self):
        self.JOINT = [6,7,8,9,10, 1,2,3,4,5]
        self.ADDR_GOAL_POSITION = 30
        self.LEN_AX_GOAL_POSITION = 2
        self.AX_PRESENT_POSITION = 36
        self.LEN_AX_PRESENT_POSITION = 2
        self.ADDR_TORQUE_ENABLE = 24
        self.TORQUE_ENABLE = 1
        self.BAUDRATE = 1000000
        self.PROTOCOL_VERSION = 1

        self.portHandler = PortHandler("/dev/ttyUSB0")
        self.packetHandler = PacketHandler(1.0)

        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, self.ADDR_GOAL_POSITION, self.LEN_AX_GOAL_POSITION)
        # self.partialSyncRead = Protocol1PacketHandler()
        
        self.param_goal_position = [0 for i in range(len(self.JOINT))]
        self.JOINT_INIT = [508, 512, 512, 512, 508, 506, 512, 512, 512, 520]

        # OPEN PORT
        if not self.portHandler.openPort():
            print("FAILED OPEN PORT")

        # SET BAUDRATE
        if not self.portHandler.setBaudRate(self.BAUDRATE):
            print("FAILED SET BAUDRATE")

        #CHECK JOINT RESPONSE
        self.ping()

        #ENABLE JOINT TORQUE
        self.torque_on()

        self.present_position = [None] * 10
        self.servo_value = []

    def ping(self):
        for id in range(len(self.JOINT)):
            #PING
            _, _, dxl_error = self.packetHandler.ping(self.portHandler, self.JOINT[id])
            if dxl_error != 0:
                print("JOINT [%d] NOT RESPONSE" % self.JOINT[id])
            else:
                print("JOINT [%d] GIVE RESPONSE" % self.JOINT[id])

    def torque_on(self):
        for id in range(len(self.JOINT)):
            #ENABLE TORQUE /ID
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, self.JOINT[id], self.ADDR_TORQUE_ENABLE, self.TORQUE_ENABLE)
            
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            if dxl_error != 0:
                print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def sync_write_pos(self, goal):
        #SET GOAL POS
        for id in range(len(self.JOINT)):
            goal_tmp = self.dec2bin(goal[id], id) 


            param_goal_position = [DXL_LOBYTE(DXL_LOWORD(int(goal_tmp))), DXL_HIBYTE(DXL_LOWORD((goal_tmp)))]

            dxl_addparam_result = self.groupSyncWrite.addParam(self.JOINT[id], param_goal_position)
            if dxl_addparam_result != True:
                print("[ID:%03d] GROUPSYNC ADD PARAM FAILED" % id)
                self.portHandler.closePort()
                quit()

        #SEND GOAL POS
        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.portHandler.closePort()
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        #CLEAR PARAM
        self.groupSyncWrite.clearParam()

    def sync_read_pos(self):
        # for id in range(10):
        #         self.present_position[id] = self.packetHandler.read2ByteTxRx(self.portHandler, self.JOINT[id], self.AX_PRESENT_POSITION)
        self.present_position[4] = self.packetHandler.read2ByteTxRx(self.portHandler, self.JOINT[4], self.AX_PRESENT_POSITION)
        self.present_position[9] = self.packetHandler.read2ByteTxRx(self.portHandler, self.JOINT[9], self.AX_PRESENT_POSITION)
        self.present_position[3] = self.packetHandler.read2ByteTxRx(self.portHandler, self.JOINT[3], self.AX_PRESENT_POSITION)
        self.present_position[8] = self.packetHandler.read2ByteTxRx(self.portHandler, self.JOINT[8], self.AX_PRESENT_POSITION)
        self.present_position[2] = self.packetHandler.read2ByteTxRx(self.portHandler, self.JOINT[2], self.AX_PRESENT_POSITION)
        self.present_position[7] = self.packetHandler.read2ByteTxRx(self.portHandler, self.JOINT[7], self.AX_PRESENT_POSITION)
        self.present_position[1] = self.packetHandler.read2ByteTxRx(self.portHandler, self.JOINT[2], self.AX_PRESENT_POSITION)
        self.present_position[6] = self.packetHandler.read2ByteTxRx(self.portHandler, self.JOINT[7], self.AX_PRESENT_POSITION)
        self.present_position[0] = self.packetHandler.read2ByteTxRx(self.portHandler, self.JOINT[2], self.AX_PRESENT_POSITION)
        self.present_position[5] = self.packetHandler.read2ByteTxRx(self.portHandler, self.JOINT[7], self.AX_PRESENT_POSITION)

        print(self.present_position)
    
    def dec2bin(self, value, id):
        goal = self.JOINT_INIT[id] - int(value * 1023/5.23599)

        if goal > 1023:
            goal = 1023
        elif goal < 0:
            goal = 0

        return int(goal)