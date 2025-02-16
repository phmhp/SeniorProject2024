# settings.py
class Params:
    def __init__(self):
        self.RingBF = []
        self.cRing = 0
        self.Main_list = []
        self.limitBF = 10
        self.byCHK = None
        self.state = None
        
        self.cPut = 0  # write pointer
        self.cGet = 0  # read pointer

        self.sID = 1  # ID
        self.sVelo = 57600
        self.MDUI = 184 # RMID
        self.MMI = 172  # TMID
        
        self.DATA1 = 1

        self.PID4 = 4
        self.PID_ROBOT_MONITOR = 253
        self.PID_ROBOT_CMD = 252

        self.robot_cmd_data = 6
        self.rc_d0 = 1 # servo
        self.rc_d1 = 0
        self.rc_d2 = 0
        self.rc_d3 = 0
        self.rc_d4 = 0
        self.rc_d5 = 0
    
        
    def pid_robot_monitor(self,mode):
        # 로봇의 상태 데이터 받아오기
        if mode == 'sum':
            result = self.MDUI + self.MMI + self.sID + self.PID4 + self.DATA1 + self.PID_ROBOT_MONITOR
        elif mode in ['put', 'display']:
            result = [self.MDUI, self.MMI, self.sID, self.PID4, self.DATA1, self.PID_ROBOT_MONITOR]
        return result

    def pid_robot_cmd(self,mode):
        # 로봇 이동 명령
        if mode == 'sum':
            result = self.MDUI + self.MMI + self.sID + self.PID_ROBOT_CMD + self.robot_cmd_data + self.rc_d0 + self.rc_d1 + self.rc_d2 + self.rc_d3 + self.rc_d4 + self.rc_d5
        elif mode in ['put', 'display']:
            result = [self.MDUI, self.MMI, self.sID, self.PID_ROBOT_CMD, self.robot_cmd_data, self.rc_d0, self.rc_d1, self.rc_d2, self.rc_d3, self.rc_d4, self.rc_d5]
        return result
