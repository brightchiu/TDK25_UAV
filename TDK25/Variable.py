# TDK25 飛行組 競賽程式碼 - 變數模組
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞

# 版本資訊
# 程式序號 TDK25-VARIABLE
# 版號 02
# 最後編輯日 2021/07/03

# 程式說明
# 存取程式所有狀態變數、命令值
# 以單一變數對應方式儲存
# 大寫表示以便識別

# 注意事項

# 模組匯入
import platform
import time

# 副程式匯入
import TDK25.Tool as Tool


class StateVariable:
    def __init__(self):
        self.timer = Tool.Timer()
        self.set_mission_time()

        # 狀態機標籤
        self.FLAG_STATE = 'CONNECT'

        # 實機或模擬狀態判定
        os_name = platform.platform()
        if os_name[0:5] == 'macOS':
            self.FLAG_SIM = True
        elif os_name[0:7] == 'Windows':
            self.FLAG_SIM = True
        else:
            self.FLAG_SIM = False

        if self.FLAG_SIM:
            pass

        # 偵測模式標籤
        self.FLAG_DETECT_MODE = ''
        self.FLAG_DETECT_COLOR = ''

        # 高度
        self.ALT_ABS = 0                # 絕對高度
        self.ALT_REL = 0                # 相對高度
        self.ALT_LIDAR = 0
        self.ALT_LASER = 0
        self.ALT_LIDAR_RAW = 0
        self.ALT_US = 0

        self.ALT_VEL_REL = 0
        self.ALT_VEL_ABS = 0

        # 姿態
        self.ATT_ROLL = 0
        self.ATT_PITCH = 0
        self.ATT_YAW = 0
        self.UAV_ROLL = 0
        self.UAV_PITCH = 0
        self.UAV_YAW = 0

        # 航向、線夾角
        self.HDG = 0
        self.ANG = 0

        # 目標顏色面積
        self.AREA = 0
        self.STRENGTH = 0

        self.POS_X = 0
        self.POS_Y = 0

        self.VEL_X = 0
        self.VEL_Y = 0

        self.VEL_X_OF = 0
        self.VEL_Y_OF = 0

        # 遙控器數值
        self.RC = {'1': 0, '2': 0, '3': 0, '4': 0, '5': 0, '6': 0, '7': 0, '8': 0}

        # 線程狀態
        self.FLAG_SR_ALT = False
        self.FLAG_SR_CAM = False
        self.FLAG_CTRL_POS = False
        self.FLAG_CTRL_VEL = False
        self.FLAG_CTRL_HDG = False
        self.FLAG_CT = False

        # 子線程、背景線程狀態
        self.FLAG_CAM = False

        # 線程狀態
        self.FLAG_THREAD = False
        self.FLAG_THREAD_READY = False
        self.FLAG_THREAD_RESET = False
        
        self.FLAG_MONITOR = False

    # 設定任務時間
    def set_mission_time(self):
        self.timer.set_start()
        self.MS_TIME = time.localtime()

    # 任務經過時間
    @property
    def get_time_since_mission_start(self):
        return self.timer.elapsed_time()

    # 任務執行時刻
    @property
    def get_mission_time_string(self):
        return '%d-%d %d-%d-%d' % (self.MS_TIME[1], self.MS_TIME[2],
                                   self.MS_TIME[3], self.MS_TIME[4], self.MS_TIME[5])


class Command:
    def __init__(self):
        self.ALT_REL = 0
        self.ALT_ABS = 0
        self.ALT_VEL = 0

        self.POS_X = 0
        self.POS_Y = 0

        self.VEL_X = 0
        self.VEL_Y = 0

        self.ANG = 0

        # 姿態傳送命令
        self.ATT = {'ROLL': 0, 'PITCH': 0, 'YAW': 0, 'YAW_RATE': 0, 'VZ_RATE': 0}

        self.SW_LAND = False
        self.SW_REC = False
        self.SW_RESET = False
        self.SW_TRACK_OP_FWD = True

        # 感測器開關
        self.SW_THR_SR_ALT = True
