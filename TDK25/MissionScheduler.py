# TDK25 飛行組 競賽程式碼 - 任務模組
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞

# 版本資訊
# 程式序號 TDK25-MISSION
# 版號 02
# 最後編輯日 2021/07/05

# 程式說明
# 背景任務進度控制
# 包含起飛、高度變更、偵測模式選擇
#
# 後期擴充修改模組

# 注意事項

# 模組匯入
import math
import time
from dronekit import VehicleMode
import threading

# 副程式匯入
from TDK25.Tool import PrintLayout
import TDK25.Tool as Tool


class MissionScheduler:
    # 初始化線程
    def __init__(self, obj_handler, bg_thread, picker, simulation=False):
        # 取得外部統一變數
        self.OBJ_HANDLER = obj_handler
        self.STATE = obj_handler.STATE
        self.CMD = obj_handler.CMD
        self.PARAM = obj_handler.PARAM
        self.UAV = obj_handler.UAV
        self.BG_THR = bg_thread
        self.picker = picker

        # 任務顏色
        self.mission_color = 'green'
        
        self.LED = Tool.Led_Control()

        # 模擬用參數
        self.simulation = simulation
        self.sim_duration = self.PARAM.value['MS_SIM_DURATION']

    # 主執行指令
    def execute(self):
        """任務流程執行指令"""
        # 線程就緒檢查
        if not self.STATE.FLAG_THREAD_READY:
            return

        # 飛行程序
        while True:
            try:
                time.sleep(2)
                PrintLayout.info('<選擇清單> 1: 2: 3: 4:', False)
                ms_number = int(input('> '))
            except ValueError:
                PrintLayout.warning('輸入數值錯誤')
                ms_number = 0
            else:
                self.STATE.FLAG_MONITOR = True
                self.set_mission_color('red_dot_grab_on')                    # 設定降落色塊，機身顯示對應顏色
                # 前往指定狀態
                if ms_number == 1:
                    self.set_detect_mode('position', 'green')
                    self.takeoff(1.2)
                    # self.set_detect_mode('position', 'green')
                    self.hover(target_alt=1.2, duration=10, wait_alt=True)
                    self.landing()
                elif ms_number == 2:
                    self.takeoff(0.5)                                 # 飛到一公尺高
                    self.set_tracking(color='red_light', takeoff=True)  # 設定為循跡模式
                    self.wait_color_arrive('red_light', 0.1)        # 開始往前飛行，等待紅色面積抵達閥值
                    self.set_positioning('red_light')               # 紅燈定位
                    self.wait_color_leave('red_light')              # 等待紅燈消失
                    self.set_tracking(color='green')              # 設定為循跡模式，偵測紅點面積
                    self.wait_color_arrive('green', 0.1)        # 開始往前飛行，等待紅色面積抵達閥值
                    self.landing()
                elif ms_number == 3:
                    self.set_detect_mode('position', 'red_dot_grab_on')
                    self.LED.red()
                    self.takeoff(0.5)
                    self.LED.green()
                    self.hover(target_alt=0.5, duration=5, wait_alt=True)
                    failed_count = 0
                    self.set_altitude(0.15)
                    while not self.picker.pick_up():                 # 抓捕指令
                        self.set_altitude(0.5)                          # 更改高度至0.5m
                        # self.wait_lock(res=0.1)                         # 等待鎖定位置(解析可選)
                        self.set_altitude(0.15)                          # 更改高度至0.1m
                        failed_count += 1
                        if failed_count > 2:
                            break
                    self.hover(target_alt=0.5, duration=5, wait_alt=True)
                    self.picker.release()
                    self.landing()
                    
                elif ms_number == 4:
                    pass
                else:
                    ms_number = 0
                    self.STATE.FLAG_MONITOR = False
                    PrintLayout.warning('清單輸入錯誤，請重選\n')
                
            if ms_number:
                self.STATE.FLAG_MONITOR = False
                self.LED.turn_off()
                break
        # """
        # self.set_detect_mode('position', 'red_dot_drop_off')
        # self.LED.green()
        # self.takeoff(1)
        # self.set_tracking(color='green')
        # self.wait_color_leave('green')
        # self.set_tracking(color='red_dot_grab_on')
        # self.wait_color_arrive('red_dot_grab_on',area=0.1)
        # self.set_positioning('red_dot_grab_on')
        
        """

        # 比賽飛行流程
        self.set_mission_color('red_dot')                    # 設定降落色塊，機身顯示對應顏色

        # 起飛區
        self.takeoff(1)                                 # 飛到一公尺高
        self.set_tracking(color='red_light', takeoff=True)  # 設定為循跡模式
        self.wait_color_arrive('red_light', 0.20)       # 開始往前飛行，等待紅色面積抵達閥值

        # 號誌區
        self.set_positioning('red_light')               # 紅燈定位
        self.set_altitude(1.5)                          # 更改高度至1.5m
        self.wait_color_leave('red_light')              # 等待紅燈消失
        self.set_tracking(color='red_dot')              # 設定為循跡模式，偵測紅點面積
        self.wait_color_arrive('red_dot', 0.015)        # 開始往前飛行，等待紅點面積抵達閥值

        # 鹽山取物
        self.set_positioning('silver')                  # 鐵盒定位
        self.set_altitude(0.5)                          # 更改高度至0.5m
        self.wait_lock(res=0.1)                         # 等待鎖定位置(解析可選)
        self.set_altitude(0.1)                          # 更改高度至0.1m
        
        failed_count = 0
        while not self.picker.pick_up()                 # 抓捕指令
            self.set_altitude(0.5)                          # 更改高度至0.5m
            self.wait_lock(res=0.1)                         # 等待鎖定位置(解析可選)
            self.set_altitude(0.1)                          # 更改高度至0.1m
            failed_count += 1
            if failed_count > 3:
                break
            
        PrintLayout.fly_result('貨物拾取完成(模擬)')
        self.set_positioning('red_dot')                 # 紅點定位
        self.set_altitude(0.5, wait_alt=True)           # 更改高度至0.5m，含等待
        self.set_tracking(color='blue')                 # 設定為循跡模式，偵測藍色面積
        self.set_altitude(1)                            # 更改高度至1m
        self.wait_color_arrive('blue', 0.4)             # 開始往前飛行，等待藍色面積抵達閥值

        # 投放區
        self.set_positioning('blue')                    # 藍色塊定位
        self.wait_lock(res=0.2)                         # 等待鎖定位置(解析可選)
        self.set_altitude(0.3, wait_alt=True)           # 更改高度至0.3m，含等待
        self.picker.release()                           # 釋放貨物
        PrintLayout.fly_result('釋放貨物完成(模擬)')
        self.set_altitude(0.8, wait_alt=True)           # 更改高度至0.8m，含等待
        self.set_tracking(color=self.mission_color)     # 設定為循跡模式，偵測任務顏色面積
        self.wait_color_arrive(self.mission_color, 1)   # 開始往前飛行，等待任務顏色面積抵達閥值

        # 降落區
        self.set_positioning(self.mission_color)        # 任務顏色塊定位
        self.wait_lock(res=0.3)                         # 等待鎖定位置(解析可選)
        self.landing()                                  # 降落程序

        """
        # 飛行程序結束
        if self.simulation:
            self.STATE.FLAG_STATE = 'PANEL'
        PrintLayout.fly_result('飛行任務已完成')

    # 啟動線程
    def start(self):
        """啟動線程"""
        PrintLayout.fly_cmd('開始執行飛行任務')
        self.thread_process = threading.Thread(target=self.execute)
        self.thread_process.start()

    # 起飛指令
    def takeoff(self, target_alt=1.0, ramp=True):
        """起飛指令"""
        # 解鎖無人機
        self.UAV.arm(wait=False)
        PrintLayout.fly_result('無人機已解鎖')

        # 切換至無衛星導航模式
        if self.UAV.version.autopilot_type == 3:
            self.UAV.mode = VehicleMode('GUIDED_NOGPS')
        elif self.UAV.version.autopilot_type == 12:
            self.UAV.mode = VehicleMode('OFFBOARD')
        PrintLayout.fly_cmd('設定為無衛星導航模式')

        # 等待馬達動力到達平衡點
        Tool.Timer.wait(3)
        PrintLayout.fly_cmd('開始執行飛行任務')

        # 設定目標高度
        self.set_altitude(target_alt, wait_alt=True, ramp=ramp)

    # 設定高度
    def set_altitude(self, target_alt, wait_alt=False, ramp=True):
        """設定高度"""
        if wait_alt or ramp:
            self.wait_arrive_altitude(target_alt, wait_alt=wait_alt, ramp=ramp)
        else:
            self.CMD.ALT_REL = target_alt
            PrintLayout.fly_cmd('更改目標高度至 %.2f m\n' % target_alt)

        if self.simulation:
            self.STATE.ALT_REL = target_alt

    # 以斜坡方式變高，並等待高度抵達
    def wait_arrive_altitude(self, target_alt, wait_alt=True, ramp=True):
        """設定高度並等待高度抵達"""
        # 執行前檢查(是否進入整定區間)
        settle_low_bound = (target_alt - math.fabs(target_alt - self.CMD.ALT_REL)
                            * (1 - self.PARAM.value['MS_ALT_TRACK_CUTOFF_RATE'] / 100))
        settle_up_bound = (target_alt + math.fabs(target_alt - self.CMD.ALT_REL)
                           * (1 - self.PARAM.value['MS_ALT_TRACK_CUTOFF_RATE'] / 100))

        if settle_low_bound <= self.STATE.ALT_REL <= settle_up_bound:
            self.CMD.ALT_REL = target_alt
            PrintLayout.fly_result('已更改且已抵達至目標高度 %.2f m\n' % target_alt)
            return

        # 設定目標高度
        PrintLayout.fly_cmd('更改至目標高度 %.2f m，等待抵達中...\n' % target_alt)
        if not ramp:
            self.CMD.ALT_REL = target_alt
        delta_alt = target_alt - self.STATE.ALT_REL

        # 計時器設定
        i = 0
        timer = Tool.Timer()

        # 等待抵達目標高度
        while self.STATE.ALT_REL > settle_up_bound or self.STATE.ALT_REL < settle_low_bound:

            # 斜坡變高
            if ramp:
                if delta_alt > 0:
                    self.CMD.ALT_REL += self.PARAM.value['MS_ALT_TRACK_SLOPE'] * (1 / self.PARAM.value['FREQ_MS'])
                    if self.CMD.ALT_REL > target_alt:
                        self.CMD.ALT_REL = target_alt
                        ramp = False
                else:
                    self.CMD.ALT_REL -= self.PARAM.value['MS_ALT_TRACK_SLOPE'] * (1 / self.PARAM.value['FREQ_MS'])
                    if self.CMD.ALT_REL < target_alt:
                        self.CMD.ALT_REL = target_alt
                        ramp = False

            # 強制跳出(遙控器背後撥桿下撥讀值)
            if self.STATE.RC['7'] > self.PARAM.value['MS_RC_PASS_LIMIT']:
                return

            # 模擬超時
            if self.simulation and timer.elapsed_time() > self.sim_duration:
                break

            # 斜坡命令完成跳出(不等待模式)
            if not ramp and not wait_alt:
                break

            # 迴圈等待
            i += 1
            timer.wait_clock_arrive_ms(i * (1000 / self.PARAM.value['FREQ_MS']))

        # 回報抵達目標高度
        PrintLayout.fly_result('已抵達目標高度 %.2f m\n' % self.CMD.ALT_REL)

    # 懸停等待(無其他定位或循跡指令)
    def hover(self, target_alt=1.0, duration=5.0, wait_alt=False):
        """懸停等待(指令佔用)"""
        # 檢查高度是否抵達
        if target_alt != self.CMD.ALT_REL:
            if wait_alt:
                self.wait_arrive_altitude(target_alt)
            else:
                self.set_altitude(target_alt)

        PrintLayout.fly_cmd('懸停高度 %.2f m，持續 %.1f 秒' % (target_alt, duration))

        # 計時器設定
        i = 0
        timer = Tool.Timer()

        # 等待時間抵達
        while timer.elapsed_time() < duration:
            # 強制跳出(遙控器背後撥桿下撥讀值)
            if self.STATE.RC['7'] > self.PARAM.value['MS_RC_PASS_LIMIT']:
                return

            # 模擬超時
            if self.simulation and timer.elapsed_time() > self.sim_duration:
                break

            # 迴圈等待
            i += 1
            timer.wait_clock_arrive_ms(i * (1000 / self.PARAM.value['FREQ_MS']))

        # 回報抵達目標高度
        PrintLayout.fly_result('懸停狀態結束\n')

    # 降落指令
    def landing(self):
        """降落指令"""
        # 啟動降落模式(控制器複寫指令)
        PrintLayout.fly_cmd('開始降落至地面\n')
        # 設定目標高度
        self.set_altitude(-0.1, wait_alt=False, ramp=True)
        self.CMD.SW_LAND = True

        # 計時器設定
        timer = Tool.Timer()

        # 等待抵達地面
        while self.STATE.ALT_REL > self.PARAM.value['MS_LANDING_CUTOFF']:

            # 強制跳出(遙控器背後撥桿撥到底，且已超時方可跳出)
            if timer.elapsed_time() > self.PARAM.value['MS_LANDING_TIMEOUT']:
                if self.STATE.RC['7'] > self.PARAM.value['MS_RC_PASS_LIMIT']:
                    break

            # 模擬超時
            if self.simulation and timer.elapsed_time() > self.sim_duration:
                break

        # 回報降落成功
        PrintLayout.fly_result('已降落地面')

        # 上鎖無人機
        while self.CMD.ATT['VZ_RATE']:
            pass
        timer.wait_ms(500)
        PrintLayout.fly_result('油門已怠速，等候自動鎖定')
        self.UAV.disarm(wait=True)
        PrintLayout.fly_result('無人機鎖定完成')
        self.LED.turn_off()

        # 關閉降落模式
        self.CMD.SW_LAND = False

        # 切換至自穩模式
        if self.UAV.version.autopilot_type == 3:
            self.UAV.mode = VehicleMode('STABILIZE')
        elif self.UAV.version.autopilot_type == 12:
            self.UAV.mode = VehicleMode('STABILIZED')
        PrintLayout.fly_cmd('設定為自穩模式')

    # 設定影像辨識目標
    def set_detect_mode(self, mode, color):
        """設定影像辨識目標"""
        if mode == 'position':
            PrintLayout.fly_cmd('影像辨識切換至定位模式，定位 %s 顏色' % color)
        elif mode == 'tracking':
            PrintLayout.fly_cmd('影像辨識切換至循跡模式，目標 %s 顏色' % color)
        else:
            if mode == 'take_off' and color == 'red_light':
                PrintLayout.fly_cmd('影像辨識切換起飛區模式，目標 %s 顏色' % color)
            else:
                PrintLayout.warning('影像辨識目標-參數輸入無效')
                return
        self.BG_THR.set_camera_mode(mode, color)

    # 設定降落色塊，機身顯示對應顏色
    def set_mission_color(self, color):
        # 儲存顏色資訊
        self.mission_color = color

        # 切換燈號
        # 操作燈號模組
        if color == 'red_dot_grab_on':
            self.LED.red()
        elif color == 'green':
            self.LED.green()
        else:
            pass

    # 設定為循跡模式
    def set_tracking(self, color, takeoff=False):
        """設定為循跡模式"""
        if takeoff:
            self.set_detect_mode('take_off', 'red_light')
        else:
            self.set_detect_mode('tracking', color)

        # 設定前行速度(暫定以下速度命令執行)
        if not self.CMD.SW_TRACK_OP_FWD:
            self.CMD.VEL_X = 1
            PrintLayout.fly_cmd('循跡模式，以%.2fm/s速度前進中\n' % self.CMD.VEL_X)

    # 設定為定位模式
    def set_positioning(self, color):
        """設定為定位模式"""
        self.set_detect_mode('position', color)

    # 等待顏色面積抵達閥值
    def wait_color_arrive(self, color, area=0.0):
        """等待顏色面積抵達閥值(定位只接受同色)"""
        # 檢查偵測狀態
        Tool.Timer.wait_ms(200)
        if self.STATE.FLAG_DETECT_MODE == 'position':
            color = self.STATE.FLAG_DETECT_COLOR
        # if color != self.STATE.FLAG_DETECT_COLOR:
        #     self.set_detect_mode(self.STATE.FLAG_DETECT_MODE, color)

        # 等待抵達指定顏色
        PrintLayout.fly_cmd('等待抵達 %s 色塊上方\n' % color)

        # 無輸入時使用預設值
        if not area:
            area = self.PARAM.value['MS_AREA_ARRIVE_LIMIT']

        # 開路推進模式
        if self.CMD.SW_TRACK_OP_FWD:
            PrintLayout.fly_cmd('以開路控制前進速度，%.2f deg 模式\n' % self.PARAM.value['MS_TRACK_OP_FWD_ANG'])

        # 計時器設定
        i = 0
        timer = Tool.Timer()

        # 等待抵達目標面積
        while self.STATE.AREA < area:
            # 開路控制作動模式(限循跡使用)
            if self.CMD.SW_TRACK_OP_FWD and (self.STATE.FLAG_DETECT_MODE == 'tracking'
                                             or self.STATE.FLAG_DETECT_MODE == 'take_off'):
                current_sec = timer.elapsed_time() % self.PARAM.value['MS_TRACK_OP_FWD_PERIOD']
                current_dc = current_sec / self.PARAM.value['MS_TRACK_OP_FWD_PERIOD']
                if current_dc < self.PARAM.value['MS_TRACK_OP_FWD_DC']/100:
                    self.CMD.ATT['PITCH'] = -self.PARAM.value['MS_TRACK_OP_FWD_ANG']
                else:
                    self.CMD.ATT['PITCH'] = 0

            # 強制跳出(遙控器背後撥桿下撥讀值)
            if self.STATE.RC['7'] > self.PARAM.value['MS_RC_PASS_LIMIT']:
                return

            # 模擬超時
            if self.simulation and timer.elapsed_time() > self.sim_duration:
                break

            # 迴圈等待
            i += 1
            timer.wait_clock_arrive_ms(i * (1000 / self.PARAM.value['FREQ_MS']))

        # 回報抵達目標高度
        PrintLayout.fly_result('已抵達 %s 色塊\n' % color)

    # 等待顏色面積離開
    def wait_color_leave(self, color, area=0.0):
        """等待顏色面積抵達閥值(定位只接受同色)"""
        # 檢查偵測狀態
        if self.STATE.FLAG_DETECT_MODE == 'position' and color != self.STATE.FLAG_DETECT_COLOR:
            color = self.STATE.FLAG_DETECT_COLOR
            PrintLayout.warning('定位模式下只接受同色偵測')
        if color != self.STATE.FLAG_DETECT_COLOR:
            self.set_detect_mode(self.STATE.FLAG_DETECT_MODE, color)

        # 等待抵達指定顏色
        PrintLayout.fly_cmd('等待離開 %s 色塊\n' % color)

        # 無輸入時使用預設值
        if not area:
            area = self.PARAM.value['MS_AREA_LEAVE_LIMIT']

        # 計時器設定
        i = 0
        timer = Tool.Timer()

        # 等待抵達目標面積
        while self.STATE.AREA > area:
            # 開路控制作動模式(限循跡使用)
            if self.CMD.SW_TRACK_OP_FWD and self.STATE.FLAG_DETECT_MODE == 'tracking':
                current_sec = timer.elapsed_time() % self.PARAM.value['MS_TRACK_OP_FWD_PERIOD']
                current_dc = current_sec / self.PARAM.value['MS_TRACK_OP_FWD_PERIOD']
                if current_dc < self.PARAM.value['MS_TRACK_OP_FWD_DC'] / 100:
                    self.CMD.ATT['PITCH'] = -self.PARAM.value['MS_TRACK_OP_FWD_ANG']
                else:
                    self.CMD.ATT['PITCH'] = 0

            # 強制跳出(遙控器背後撥桿下撥讀值)
            if self.STATE.RC['7'] > self.PARAM.value['MS_RC_PASS_LIMIT']:
                return

            # 模擬超時
            if self.simulation and timer.elapsed_time() > self.sim_duration:
                break

            # 迴圈等待
            i += 1
            timer.wait_clock_arrive_ms(i * (1000 / self.PARAM.value['FREQ_MS']))

        # 回報抵達目標高度
        PrintLayout.fly_result('%s 色塊已離開\n' % color)

    # 等待鎖定位置(解析可選)
    def wait_lock(self, res=0.0):
        """等待鎖定位置"""
        # 模式檢查
        Tool.Timer.wait_ms(200)
        if self.STATE.FLAG_DETECT_MODE != 'position':
            return

        # 無輸入時使用預設值
        if not res:
            res = self.PARAM.value['MS_LOCK_RESOLUTION']

        # 等待抵達指定顏色
        PrintLayout.fly_cmd('等待鎖定色塊\n')

        # 計時器設定
        i = 0
        timer = Tool.Timer()

        # 等待抵達目標X位置
        while not self.in_range(self.STATE.POS_X, -res, res):
            # 強制跳出(遙控器背後撥桿下撥讀值)
            if self.STATE.RC['7'] > self.PARAM.value['MS_RC_PASS_LIMIT']:
                return

            # 模擬超時
            if self.simulation and timer.elapsed_time() > self.sim_duration:
                break

            # 迴圈等待
            i += 1
            timer.wait_clock_arrive_ms(i * (1000 / self.PARAM.value['FREQ_MS']))

        # 等待抵達目標Y位置
        while not self.in_range(self.STATE.POS_Y, -res, res):
            # 強制跳出(遙控器背後撥桿下撥讀值)
            if self.STATE.RC['7'] > self.PARAM.value['MS_RC_PASS_LIMIT']:
                return

            # 模擬超時
            if self.simulation and timer.elapsed_time() > self.sim_duration:
                break

            # 迴圈等待
            i += 1
            timer.wait_clock_arrive_ms(i * (1000 / self.PARAM.value['FREQ_MS']))

        # 回報抵達目標高度
        PrintLayout.fly_result('色塊已鎖定\n')

    @staticmethod
    def in_range(current, lower, upper):
        if upper > current > lower:
            return True
        else:
            return False


"""
PX4 Mode
0 : 'MANUAL',
1 : 'ALTCTL',
2 : 'POSCTL',
3 : 'AUTO_MISSION',
4 : 'AUTO_LOITER',
5 : 'AUTO_RTL',
6 : 'ACRO',
7 : 'OFFBOARD',
8 : 'STAB',
9 : 'RATTITUDE',
10 : 'AUTO_TAKEOFF',
11 : 'AUTO_LAND',
12 : 'AUTO_FOLLOW_TARGET',
13 : 'MAX',

Ardupilot Mode
0 : 'MANUAL',
1 : 'CIRCLE',
2 : 'STABILIZE',
3 : 'TRAINING',
4 : 'ACRO',
5 : 'FBWA',
6 : 'FBWB',
7 : 'CRUISE',
8 : 'AUTOTUNE',
10 : 'AUTO',
11 : 'RTL',
12 : 'LOITER',
13 : 'TAKEOFF',
14 : 'AVOID_ADSB',
15 : 'GUIDED',
16 : 'INITIALISING',
17 : 'QSTABILIZE',
18 : 'QHOVER',
19 : 'QLOITER',
20 : 'QLAND',
21 : 'QRTL',
22 : 'QAUTOTUNE',
23 : 'QACRO',
24 : 'THERMAL',
"""