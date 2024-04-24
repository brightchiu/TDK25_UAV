# TDK25 飛行組 競賽程式碼 - 工具模組
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞

# 版本資訊
# 程式序號 TDK25-TOOL
# 版號 01
# 最後編輯日 2021/05/15

# 程式說明

# 注意事項

# 模組匯入
from os import listdir
import matplotlib.pyplot as plt
import RPi.GPIO as GPIO_rpi
import asyncio
import numpy as np
import platform
import math
import time
import csv
try:
    import rpi_ws281x
except ModuleNotFoundError:
    pass
import argparse
import dronekit

# 副程式匯入
import TDK25.Variable as Variable

# TDK25 飛行組 競賽程式碼 - 工具模組 - 計時器程式
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞

# 版本資訊
# 程式序號 TDK25-TOOL-TIMER
# 版號 02
# 最後編輯日 2021/10/05

# 程式說明
# 更改時鐘成 time.perf_counter()以獲得高精度穩定時刻

# 注意事項


# 計時器模組
class Timer:
    """計時器模組"""
    def __init__(self):
        self.set_start()

    def set_start(self):
        """設定起始時刻"""
        self.start_ns = time.perf_counter_ns()
        self.previous_ns = self.start_ns

    def elapsed_time(self):
        """經過時間"""
        return (time.perf_counter_ns() - self.start_ns)/1e9

    def elapsed_time_ms(self):
        """經過時間(毫秒)"""
        return (time.perf_counter_ns() - self.start_ns)/1e6

    def elapsed_time_us(self):
        """經過時間(微秒)"""
        return (time.perf_counter_ns() - self.start_ns)/1e3

    # 回傳經過時間
    def period(self):
        """回傳間隔時間"""
        self.current_ns = time.perf_counter_ns()
        period = self.current_ns - self.previous_ns
        self.previous_ns = self.current_ns
        return period / 1e9

    # 等待時刻抵達(線程排程器使用)
    def wait_clock_arrive(self, clock):
        """等待時刻抵達"""
        while time.perf_counter_ns()-self.start_ns < clock*1e9:
            pass

    def wait_clock_arrive_ms(self, clock):
        """等待時刻抵達(毫秒)"""
        while time.perf_counter_ns()-self.start_ns < clock*1e6:
            pass

    def wait_clock_arrive_us(self, clock):
        """等待時刻抵達(微秒)"""
        while time.perf_counter_ns()-self.start_ns < clock*1e3:
            pass

    @staticmethod
    def wait(sec):
        t0 = time.perf_counter()
        while time.perf_counter()-t0 < sec:
            pass

    @staticmethod
    def wait_ms(ms):
        t0 = time.perf_counter()
        while time.perf_counter()-t0 < ms/1000:
            pass

    @staticmethod
    def wait_us(us):
        t0 = time.perf_counter_ns()
        while time.perf_counter_ns()-t0 < us*1000:
            pass

    @property
    def start_time(self):
        return self.start_ns / 1e9

    # 正時等待
    async def wait_until_next_tick(self, loop_count, freq):
        await asyncio.sleep(self.start_time + loop_count * (1 / freq) - time.perf_counter())


# TDK25 飛行組 競賽程式碼 - 工具模組 - 記錄器程式
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞

# 版本資訊
# 程式序號 TDK25-TOOL-DATARECODER
# 版號 01
# 最後編輯日 2021/05/15

# 程式說明

# 注意事項


# 數據記錄模組
class DataRecoder:
    def __init__(self, state, contents_list, name):
        """資料紀錄(輸入：狀態、欄名稱、檔名)"""
        self.STATE = state
        self.name = name
        self.contents_list = contents_list
        self.file_name = self.STATE.get_mission_time_string + '_' + self.name
        self.rows_to_write = []

        # 參數檔存放位置
        if platform.platform() == 'macOS-10.16-x86_64-i386-64bit':
            self.file_address = '/Users/bright/PycharmProjects/TDK25_Final/Log/' + self.file_name + '.csv'
        else:
            # self.file_address = 'Log/' + self.file_name + '.csv'
            self.file_address = '/home/pi/Desktop/TDK25_Final/Log/' + self.file_name + '.csv'

    # 更新檔案名稱
    def register(self):
        self.file_name = self.STATE.get_mission_time_string + '_' + self.name

        # 參數檔存放位置
        if platform.platform() == 'macOS-10.16-x86_64-i386-64bit':
            self.file_address = '/Users/bright/PycharmProjects/TDK25_Final/Log/' + self.file_name + '.csv'
        else:
            # self.file_address = 'Log/' + self.file_name + '.csv'
            self.file_address = '/home/pi/Desktop/TDK25_Final/Log/' + self.file_name + '.csv'

        self.write_data(self.contents_list, contents=True)

    # 行暫存區
    def rows_storage(self, row_data_list):
        """於記錄狀態暫存行資料並記錄時間"""
        row_data_list.insert(0, self.STATE.get_time_since_mission_start)
        self.rows_to_write.append(row_data_list)

    # 儲存暫存檔
    def write_log(self, clear_cache=True):
        """儲存暫存檔"""
        self.write_rows_data(self.rows_to_write)
        # 清除暫存區
        if clear_cache:
            self.rows_to_write = []

    # 寫入資料
    def write_data(self, row_data_list, contents=False):
        """寫入資料(輸入一行)"""
        with open(self.file_address, 'a', newline='') as csvfile:
            write = csv.writer(csvfile, delimiter=',')
            if contents:
                pass
            else:
                row_data_list.insert(0, self.STATE.get_time_since_mission_start)
            write.writerow(row_data_list)
            csvfile.close()

    # 寫入多行資料
    def write_rows_data(self, rows_data_list):
        """寫入資料(輸入多行)"""
        with open(self.file_address, 'a', newline='') as csvfile:
            write = csv.writer(csvfile, delimiter=',')
            write.writerows(rows_data_list)
            csvfile.close()


# TDK25 飛行組 競賽程式碼 - 數據展示程式
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞

# 版本資訊
# 程式序號 TDK25-TOOL-Plot Data
# 版號 01
# 最後編輯日 2021/08/20

# 程式說明
# 尋找記錄資料夾內的記錄檔，讀取並顯示

# 注意事項
#

class PlotData:
    def __init__(self):
        # 檔案夾存放位置
        if platform.platform() == 'macOS-10.16-x86_64-i386-64bit':
            self.directory_address = '/Users/bright/PycharmProjects/TDK25_Final/Log/'
        else:
            self.directory_address = 'Log/'

        # 時間戳列表
        self.time_stamp_list = []
        pass

    def collect_files_list(self):
        files_list = listdir(self.directory_address)
        last_time_stamp = ''
        for file in files_list:
            if not file == 'UAV_CSV_Log_Reader.m':
                file_time_stamp = file.split('_')[0]
                if file_time_stamp != last_time_stamp and file.split('_')[1] == 'Altitude Controller.csv':
                    self.time_stamp_list.append(file_time_stamp)
                    last_time_stamp = file_time_stamp

    # 顯示檔案列表與選擇代號
    def show_files_by_time(self):
        PrintLayout.info('目前可讀取記錄檔的時刻', head=False)
        print('{:^5}'.format('ID'), '{:^15}'.format('Time'))
        print('{:20}'.format('====================='))
        for index in range(len(self.time_stamp_list)):
            print('{:^5}'.format(index),  '{:^15}'.format(self.time_stamp_list[index]))

    # 產生高度控制器圖
    def alt_ctrl_plot(self, file_time_stamp):
        # 儲存區
        alt_time = []
        alt_sp = []
        alt_resp = []
        alt_out = []
        alt_pt = []
        alt_mt = []
        alt_vel_time = []
        alt_vel_sp = []
        alt_vel_resp = []
        alt_vel_out = []
        alt_vel_pt = []
        alt_vel_mt = []

        # 讀取檔案A
        try:
            with open(self.directory_address + file_time_stamp + '_Altitude Controller.csv',
                      newline='') as csvfile:
                reader = csv.reader(csvfile, delimiter=',')
                for row in reader:
                    # 去除標題列
                    if row[0] == 'Time':
                        pass
                    else:
                        alt_time.append(row[0])
                        alt_sp.append(row[1])
                        alt_resp.append(row[2])
                        alt_out.append(row[3])
                        alt_pt.append(row[4])
                        alt_mt.append(row[5])
        except FileNotFoundError:
            PrintLayout.warning('高度控制器記錄檔->沒有找到指定的檔案')
            return

        # 讀取檔案B
        try:
            with open(self.directory_address + file_time_stamp + '_Altitude Velocity Controller.csv',
                      newline='') as csvfile:
                reader = csv.reader(csvfile, delimiter=',')
                for row in reader:
                    # 去除標題列
                    if row[0] == 'Time':
                        pass
                    else:
                        alt_vel_time.append(row[0])
                        alt_vel_sp.append(row[1])
                        alt_vel_resp.append(row[2])
                        alt_vel_out.append(row[3])
                        alt_vel_pt.append(row[4])
                        alt_vel_mt.append(row[5])

        except FileNotFoundError:
            PrintLayout.warning('高度速度控制器記錄檔->沒有找到指定的檔案 ')
            return

        # 出圖
        plt.figure(figsize=[5, 5], dpi=300)
        plt.subplot(2, 3, 1)
        plt.step(alt_time, alt_sp, label="Set Point")
        plt.step(alt_time, alt_sp, label="Response")
        plt.step(alt_time, alt_sp, label="Control Output")
        plt.legend()
        plt.xlabel('Time (sec)')
        plt.ylabel('Amplitude (m, m/s)')
        plt.title('Altitude Controller')

        plt.subplot(2, 3, 2)
        plt.step(alt_time, alt_sp, label="Set Point")
        plt.step(alt_time, alt_sp, label="Response")
        plt.step(alt_time, alt_sp, label="Control Output")
        plt.legend()
        plt.xlabel('Time (sec)')
        plt.ylabel('Amplitude (m, m/s)')
        plt.title('Altitude Controller')


# TDK25 飛行組 競賽程式碼 - 工具模組 - 濾波器程式
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞

# 版本資訊
# 程式序號 TDK25-TOOL-MAF
# 版號 01
# 最後編輯日 2020/12/22

# 程式說明
# 延遲程度由取樣次數決定，一般來說為(取樣次數/2*取樣頻率)sec
# 本物件不控制取樣頻率，由呼叫式的頻率決定

# 注意事項

class MovingAverageFilter:
    """移動均值濾波器，初始設定取樣次數，執行運算輸入當筆資料可得移動平均"""

    # 初始化設定
    def __init__(self, sample_number=10):
        self.register = []
        self.sample_number = sample_number

    # 計算平均值
    def processing(self, value):
        """計算移動平均"""
        self.register.insert(0, value)
        if len(self.register) > self.sample_number:
            self.register.pop()
        maf = np.average(self.register)
        return maf

    # 重設暫存器
    def reset(self):
        """重設暫存器"""
        self.register = []


# TDK25 飛行組 競賽程式碼 - 感測器調整模組
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞

# 版本資訊
# 程式序號 TDK25-Tuning-Sensor Trim
# 版號 02
# 最後編輯日 2021/08/19

# 程式說明
# 以飛行情形調整調整 sensor trim value

# 注意事項
# 不得於飛行中調整

class SensorTrim:
    """感測器調整模組"""
    def __init__(self, obj_handler):
        # 取得外部統一變數
        self.OBJ_HANDLER = obj_handler
        self.STATE = obj_handler.STATE
        self.CMD = obj_handler.CMD
        self.PARAM = obj_handler.PARAM
        self.UAV = obj_handler.UAV

    @property
    def ahrs_trim_x(self):
        if self.UAV.version.autopilot_type == 3:
            return math.degrees(self.UAV.parameters['AHRS_TRIM_X'])
        elif self.UAV.version.autopilot_type == 12:
            return self.UAV.parameters['SENS_BOARD_X_OFF']
        else:
            return 0

    @ahrs_trim_x.setter
    def ahrs_trim_x(self, trim_x):
        if self.UAV.version.autopilot_type == 3:
            self.UAV.parameters['AHRS_TRIM_X'] = math.radians(trim_x)
        elif self.UAV.version.autopilot_type == 12:
            self.UAV.parameters['SENS_BOARD_X_OFF'] = trim_x
        else:
            PrintLayout.error('飛控板版本錯誤，請檢查')

    @property
    def ahrs_trim_y(self):
        if self.UAV.version.autopilot_type == 3:
            return math.degrees(self.UAV.parameters['AHRS_TRIM_Y'])
        elif self.UAV.version.autopilot_type == 12:
            return self.UAV.parameters['SENS_BOARD_Y_OFF']
        else:
            return 0

    @ahrs_trim_y.setter
    def ahrs_trim_y(self, trim_y):
        if self.UAV.version.autopilot_type == 3:
            self.UAV.parameters['AHRS_TRIM_Y'] = math.radians(trim_y)
        elif self.UAV.version.autopilot_type == 12:
            self.UAV.parameters['SENS_BOARD_Y_OFF'] = trim_y
        else:
            PrintLayout.error('飛控板版本錯誤，請檢查')

    # 參數修改
    def input_interface(self):
        """姿態感測器配平微調介面"""
        PrintLayout.info('\n========================================', False)
        PrintLayout.info('>>>>>>>>>> 姿態感測器配平微調介面 <<<<<<<<<<', False)
        PrintLayout.info('========================================', False)
        # print('\n>>>>>>>>>> 姿態感測器配平微調介面 <<<<<<<<<<')

        # 選單迴圈
        while True:
            try:
                PrintLayout.info('\n現在偏移值 X=%.2f deg, Y=%.2f deg' % (self.ahrs_trim_x, self.ahrs_trim_y), False)
                PrintLayout.info('請選擇修改軸向 X=>Roll Y=>Pitch Z=>關閉調整介面', False)
                axis = str(input('> '))
            except dronekit.TimeoutError:
                PrintLayout.warning('等待下載參數超時')
                return

            if axis == 'X' or axis == 'x':
                PrintLayout.info('現在Roll偏移值 X=%.2f deg' % self.ahrs_trim_x, False)
                try:
                    PrintLayout.info('請輸入Roll偏移數值 (Deg, Positive values make the vehicle r right)', False)
                    trim_x = float(input('> '))
                except ValueError:
                    PrintLayout.warning('輸入數值錯誤')
                else:
                    if -10 <= trim_x <= 10:
                        self.ahrs_trim_x = trim_x
                    else:
                        PrintLayout.warning('輸入數值超出範圍')

            elif axis == 'Y' or axis == 'y':
                PrintLayout.info('現在Pitch偏移值 Y=%.2f deg' % self.ahrs_trim_y, False)
                try:
                    PrintLayout.info('請輸入Pitch偏移數值 (Deg, Positive values make the vehicle pitch up/back)', False)
                    trim_y = float(input('> '))
                except ValueError:
                    PrintLayout.warning('輸入數值錯誤')
                else:
                    if -10 <= trim_y <= 10:
                        self.ahrs_trim_y = trim_y
                    else:
                        PrintLayout.warning('輸入數值超出範圍')

            elif axis == 'Z' or axis == 'z':
                PrintLayout.info('姿態感測器配平微調介面，關閉')
                break
            else:
                PrintLayout.warning('輸入錯誤')


# TDK25 飛行組 競賽程式碼 - 命令列顯示模組
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞

# 版本資訊
# 程式序號 TDK25-Tuning-Shell Print
# 版號 01
# 最後編輯日 2021/07/14

# 程式說明
# 根據不同級別顯示不同顏色之訊息

# 注意事項

class PrintLayout:
    def __init__(self):
        pass

    @staticmethod
    def error(msg, head=True):
        """將指定字串以指定顏色展示"""
        color_str = '\033[91m'
        if head:
            pre_str = '<系統錯誤> '
            print(color_str + pre_str + msg)
        else:
            print(color_str + msg)

    @staticmethod
    def success(msg, head=True):
        """將指定字串以指定顏色展示"""
        color_str = '\033[92m'
        if head:
            pre_str = '<設定成功> '
            print(color_str + pre_str + msg)
        else:
            print(color_str + msg)

    @staticmethod
    def warning(msg, head=True):
        """將指定字串以指定顏色展示"""
        color_str = '\033[93m'
        if head:
            pre_str = '<錯誤警告> '
            print(color_str + pre_str + msg)
        else:
            print(color_str + msg)

    @staticmethod
    def fly_result(msg, head=True):
        """將指定字串以指定顏色展示"""
        color_str = '\033[94m'
        if head:
            pre_str = '<飛行命令結果> '
            print(color_str + pre_str + msg)
        else:
            print(color_str + msg)

    @staticmethod
    def fly_cmd(msg, head=True):
        """將指定字串以指定顏色展示"""
        color_str = '\033[95m'
        if head:
            pre_str = '<飛行命令> '
            print(color_str + pre_str + msg)
        else:
            print(color_str + msg)

    @staticmethod
    def info(msg, head=True):
        """將指定字串以指定顏色展示"""
        color_str = '\033[0m'
        if head:
            pre_str = '<系統訊息> '
            print(color_str + pre_str + msg)
        else:
            print(color_str + msg)

    """
    顏色字串附註
    90~97 Word
    NONE = '\033[90m' #Black
    ERROR = '\033[91m' #RED
    SUCCESS = '\033[92m' #GREEN
    WARNING = '\033[93m' #YELLOW
    FLY_RESULT = '\033[94m' #BLUE
    FLY_CMD = '\033[95m' #PURPLE
    NONE = '\033[96m' #L-BLUE
    NONE = '\033[97m' #L-WHITE
    100~107 Background
    WARNING = '\033[100m' #GRAY_BG
    NORMAL = '\033[0m' #Normal
    """


# TDK25 飛行組 競賽程式碼 - 工具模組 - 拾物機構模組
# C04 台灣大艦隊
# 程式撰寫：羅宇彥、邱柏瑞

# 版本資訊
# 程式序號 TDK25-TOOL-EM
# 版號 01
# 最後編輯日 2021/11/18

# 程式說明
# 負責控制機構升降控制、電磁鐵控制、物件感測
# 本程式呼叫後皆會佔用主程序

# 注意事項 : 直流馬達版本、腳位確定


class Picker:
    """拾物模組"""

    def __init__(self):
        """設定腳位編號"""
        # 設定開關的腳位編號
        self._EM_button_pin = 36  # 鐵盒開關
        self._init_button_pin = 38  # 歸位開關
        self._limit_button_pin = 40  # 極限開關

        # 設定電磁鐵、直流馬達的腳位編號
        self._em_pin = 32
        self._in1_pin = 33
        self._in2_pin = 35
        self._PWM_pin = 37

        # 拾取貨物的旗標、啟用的旗標、歸位時間、極限時間
        self._pick_flag = False
        self._enable_flag = False
        self._init_time = 0.1  # 0.1s
        self._limit_time = 3.3  # 3.3s

    def enable(self):
        """啟用(建立腳位、建立電磁鐵與直流馬達的物件)"""
        try:
            GPIO_rpi.setmode(GPIO_rpi.BOARD)  # 設定腳位模式
            GPIO_rpi.setwarnings(False)

            # 開關腳位採用下拉電阻
            GPIO_rpi.setup(self._EM_button_pin, GPIO_rpi.IN, pull_up_down=GPIO_rpi.PUD_DOWN)
            GPIO_rpi.setup(self._init_button_pin, GPIO_rpi.IN, pull_up_down=GPIO_rpi.PUD_DOWN)
            GPIO_rpi.setup(self._limit_button_pin, GPIO_rpi.IN, pull_up_down=GPIO_rpi.PUD_DOWN)

            # 建立電磁鐵、直流馬達的物件
            self.EM = Electromagnet(self._em_pin)
            self.M = Motor(self._in1_pin, self._in2_pin, self._PWM_pin)

            # 檢查開關狀態
            self._enable_flag = False
            if self.get_detected_em_button() == 0:
                PrintLayout.error('拾物機構-吸取開關異常')
                # print("EM_button is failed")

            elif self.get_detected_init_button() == 0:
                PrintLayout.error('拾物機構-原點開關異常')
                # print("init_button is failed")

            elif self.get_detected_limit_button() == 0:
                PrintLayout.error('拾物機構-極限開關異常')
                # print("limit_button is failed")
                
            else:
                self._enable_flag = True

        except Exception:  # 如果有問題(例外處理)
            self._enable_flag = False
            PrintLayout.error('拾物機構異常')
            return self._enable_flag  # 回傳False

        else:
            if self._enable_flag:
                self.initialization()  # 如果沒問題，回到初始位置
                PrintLayout.success('拾物機構-初始化完成')
            else:
                pass

            return self._enable_flag  # 回傳是否成功啟用

    def get_detected_em_button(self):
        """讀取鐵盒開關的狀態"""
        return GPIO_rpi.input(self._EM_button_pin)  # 壓下為0

    def get_detected_init_button(self):
        """讀取歸位開關的狀態"""
        return GPIO_rpi.input(self._init_button_pin)  # 壓下為0

    def get_detected_limit_button(self):
        """讀取極限開關的狀態"""
        return GPIO_rpi.input(self._limit_button_pin)  # 壓下為0

    def fetch(self):
        """吸取貨品"""
        self.EM.on()  # 開啟電磁鐵
        PrintLayout.success('拾物機構-電磁鐵開啟')
        # print("Electromagnet ON")

    def release(self):
        """釋放貨品"""
        self.EM.off()  # 關閉電磁鐵
        PrintLayout.success('拾物機構-電磁鐵關閉')
        # print("Electromagnet OFF")

    def initialization(self):
        """回到初始位置"""
        # 須注意馬達旋轉方向
        self.M.set_direction(-1)
        self.M.set_rpm(1)

        # 持續上升，直到歸位開關被觸發
        while self.get_detected_init_button():
            time.sleep(0.05)

        self.M.set_rpm(0)  # 停止馬達

        # 須注意馬達旋轉方向
        self.M.set_direction(1)
        self.M.set_rpm(1)
        time.sleep(self._init_time)
        self.M.set_rpm(0)  # 停止馬達

        # print("initialization is finished")

    def down(self):
        """下降過程"""
        # 須注意馬達旋轉方向
        self.M.set_direction(1)
        self.M.set_rpm(1)
        limit_count = 0  # 紀錄極限被觸發
        time_count = 0  # 紀錄超過極限時間
        start_time = time.time()  # 下降過程的開始時間

        # 持續下降(當鐵盒開關未被觸發)
        while self.get_detected_em_button():
            record_time = time.time() - start_time  # 計算下降過程的時間
            # 超過極限位置或極限時間，觸發警告
            if self.get_detected_limit_button() == 0:
                if limit_count == 0:
                    limit_count = 1
                    self.M.set_rpm(0)  # 停止馬達
                    PrintLayout.info('拾物機構-抵達最大伸展長度')
                    # print("limit warning")
                else:
                    pass
            elif record_time > self._limit_time:
                if time_count == 0:
                    time_count = 1
                    self.M.set_rpm(0)  # 停止馬達
                    PrintLayout.warning('拾物機構-伸展超時')
                    # print("time warning")
                else:
                    pass
            else:
                limit_count = 0
                # time_count = 0
                self.M.set_rpm(1)
            time.sleep(0.05)  # while迴圈間隔時間
            
            if record_time > self._limit_time + 3:
                PrintLayout.warning('拾物機構-拾物超時')
                # print('Waiting timeout')
                break

        self.M.set_rpm(0)  # 停止馬達
        # print("Downing is finished")

    def up(self):
        """上升過程"""
        # 須注意馬達旋轉方向
        self.M.set_direction(-1)
        self.M.set_rpm(1)

        # 持續上升，直到歸位開關被觸發
        while self.get_detected_init_button():
            time.sleep(0.05)
        self.M.set_rpm(0)  # 停止馬達

        # 須注意馬達旋轉方向
        self.M.set_direction(1)
        self.M.set_rpm(1)
        time.sleep(self._init_time)
        self.M.set_rpm(0)  # 停止馬達

        # 檢查貨物是否依然吸住(鐵盒開關的狀態)
        if not (self.get_detected_em_button()):
            self._pick_flag = True  # 拾取成功，拾取貨物的旗標改為True
            PrintLayout.success('拾物機構-拾物成功')
            
        else:
            self._pick_flag = False  # 拾取失敗，拾取貨物的旗標改為False
            PrintLayout.warning('拾物機構-拾物失敗')
        
    def pick_up(self):
        """拾取貨品"""
        self.fetch()  # 吸取貨品
        self.down()  # 持續下降，直到鐵盒開關被觸發
        self.up()  # 持續上升直到歸位開關被觸發
        return self._pick_flag  # 回傳拾取貨物的旗標

    def disable(self):
        """清除腳位"""
        self.EM.cleanup()
        self.M.cleanup()
        GPIO_rpi.cleanup()
        PrintLayout.info('拾物機構，已關閉')


class Electromagnet:
    """電磁鐵模組"""

    def __init__(self, em_pin):
        # 設定電池鐵的控制腳位
        self.em_pin = em_pin  # 電磁鐵腳位
        GPIO_rpi.setmode(GPIO_rpi.BOARD)
        GPIO_rpi.setup(self.em_pin, GPIO_rpi.OUT)  # 電磁鐵腳位為輸出腳位

    def on(self):
        GPIO_rpi.output(self.em_pin, 1)  # 開啟電磁鐵

    def off(self):
        GPIO_rpi.output(self.em_pin, 0)  # 關閉電磁鐵

    def cleanup(self):
        """關閉電磁鐵模組"""
        GPIO_rpi.cleanup(self.em_pin)


class Motor:
    """直流馬達"""

    def __init__(self, in1_pin, in2_pin, pwm_pin):

        GPIO_rpi.setmode(GPIO_rpi.BOARD)  # 設定腳位
        GPIO_rpi.setwarnings(False)

        # 設定直流馬達的控制腳位
        self._in1_pin = in1_pin
        self._in2_pin = in2_pin
        self._PWM_pin = pwm_pin

        # 將腳位設為輸出
        GPIO_rpi.setup(self._in1_pin, GPIO_rpi.OUT)
        GPIO_rpi.setup(self._in2_pin, GPIO_rpi.OUT)
        GPIO_rpi.setup(self._PWM_pin, GPIO_rpi.OUT)

        # 將輸出設為0
        GPIO_rpi.output(self._in1_pin, 0)
        GPIO_rpi.output(self._in2_pin, 0)
        GPIO_rpi.output(self._PWM_pin, 0)

    def set_direction(self, direction):
        """設定直流馬達的方向"""

        if direction == 1:  # 正轉
            GPIO_rpi.output(self._in1_pin, 1)
            GPIO_rpi.output(self._in2_pin, 0)

        elif direction == -1:  # 反轉
            GPIO_rpi.output(self._in1_pin, 0)
            GPIO_rpi.output(self._in2_pin, 1)

        else:
            print("direction error")

    def set_rpm(self, pwm):
        """設定直流馬達的轉速"""

        if pwm == 1:  # 旋轉
            GPIO_rpi.output(self._PWM_pin, 1)

        elif pwm == 0:  # 停止
            GPIO_rpi.output(self._PWM_pin, 0)

        else:
            print("PWM error")

    def cleanup(self):
        """清除腳位"""

        GPIO_rpi.cleanup(self._in1_pin)  # 清除腳位
        GPIO_rpi.cleanup(self._in2_pin)  # 清除腳位
        GPIO_rpi.cleanup(self._PWM_pin)  # 清除腳位
        
        
class Led_Control:
    def __init__(self):
        LED_COUNT      = 16      # Number of LED pixels.
        LED_PIN        = 18      # GPIO pin connected to the pixels (18 uses PWM!).
        #LED_PIN        = 10      # GPIO pin connected to the pixels (10 uses SPI /dev/spidev0.0).
        LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
        LED_DMA        = 10      # DMA channel to use for generating signal (try 10)
        LED_BRIGHTNESS = 255     # Set to 0 for darkest and 255 for brightest
        LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
        LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53

        parser = argparse.ArgumentParser()
        parser.add_argument('-c', '--clear', action='store_true', help='clear the display on exit')
        args = parser.parse_args()

        strip = rpi_ws281x.Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
        # Intialize the library (must be called once before other functions).
        strip.begin()

        self.led = strip
        self.turn_off()



    def green(self):
        # print("Green light On")
        for i in range(self.led.numPixels()):
            self.led.setPixelColor(i, rpi_ws281x.Color(0,255,0))
            time.sleep(50/1000.0)
        self.led.show()
        PrintLayout.info('LED Green On')

    def red(self):
        # print("Red light On")
        for i in range(self.led.numPixels()):
            self.led.setPixelColor(i, rpi_ws281x.Color(255,0,0))
            time.sleep(50/1000.0)
        self.led.show()
        PrintLayout.info('LED Red On')

    def turn_off(self):
        # print("Led Off")
        for i in range(self.led.numPixels()):
            self.led.setPixelColor(i, rpi_ws281x.Color(0,0,0))
            time.sleep(50/1000.0)
        self.led.show()
        PrintLayout.info('LED已關閉')


if __name__ == '__main__':
    stat = Variable.StateVariable()
    t_0 = time.time()
    dr = DataRecoder(stat, ['Time', 'Value'], name='Controller')
    for i in range(1000):
        dr.write_data([i])
    print(time.time()-t_0)
