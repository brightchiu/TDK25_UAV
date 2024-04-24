# TDK25 飛行組 競賽程式碼 - 感測器模組
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞

# 版本資訊
# 程式序號 TDK25-SENSOR
# 版號 02
# 最後編輯日 2021/07/16

# 程式說明
# 彙整感測器模組

# 注意事項

# 模組匯入
from pymavlink import mavutil
import numpy as np
import math
import time

# 副程式匯入
from TDK25.Tool import PrintLayout
import TDK25.Driver as Driver
import TDK25.Tool as Tool


# TDK25 飛行組 競賽程式碼 - 感測器模組 - 光達測距儀
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞

# 版本資訊
# 程式序號 TDK25-SENSOR-ALT-LIDAR
# 版號 01
# 最後編輯日 2021/05/16

# 程式說明
# 接收TFMiniS感測器之原始值，並作處理，上傳到狀態伺服器供取用

# 注意事項
#


class SensorAltLidar:
    """SEN0259雷射測距儀，使用UART通訊協定"""

    def __init__(self, obj_handler):
        # 取得外部統一變數
        self.OBJ_HANDLER = obj_handler
        self.STATE = obj_handler.STATE
        self.CMD = obj_handler.CMD
        self.PARAM = obj_handler.PARAM
        self.UAV = obj_handler.UAV

        # 量測參數
        self._calibrate = False
        self.offset = 0
        self.previous_height = 0

        # 變數儲存
        self.height_raw = 0
        self.height_offset = 0
        self.height_maf = 0
        self.strength = 0
        self.rate = 0
        self.error = 0
        self.mt = 0

        # 記錄器
        self.recorder = Tool.DataRecoder(self.STATE, ['Time', 'Raw', 'Offset', 'Filtered',
                                                      'strength', 'rate', 'error', 'mt'],
                                         name='Altitude Sensor - Lidar')

        # 初始化濾波器
        self.maf = Tool.MovingAverageFilter(self.PARAM.value['SENS_ALT_LID_MAF_SN'])

    def enable(self):
        # 初始化感測器
        self.lidar = Driver.Lidar(self.OBJ_HANDLER)
        if self.lidar.enable(bg_thread=True):
            # 校正執行
            self.calibrate(sn=self.PARAM.value['SENS_ALT_LID_CAL_SN'])
            return True
        else:
            return False

    # 高度校正
    def calibrate(self, sn=10):
        """執行光達測距儀校正"""
        # 平均降噪設定
        height_register = []

        # 執行量測(10Hz校正)
        cal_timer = Tool.Timer()
        cal_timer.set_start()
        for x in range(sn):
            height_raw = self.measure()
            height_register.append(height_raw)
            cal_timer.wait_clock_arrive_ms(x*100)

        # 更新高度校正偏移量
        self.offset = -np.average(height_register)
        if self.offset == 0:
            PrintLayout.warning('光達測距儀未就緒，請執行獨立檢查')
        else:
            self._calibrate = True
            PrintLayout.success('光達測距儀校正完畢，偏移 %.3f m\n' % self.offset)

    # 量測高度
    def measure(self):
        """量測高度值"""
        # 計時開始
        mt_start = time.time()

        # 執行量測
        self.height_raw = self.lidar.distance
        self.strength = self.lidar.strength
        self.rate = self.lidar.update_rate
        self.error = self.lidar.error_count

        # 未校正則回傳原始值
        if not self._calibrate:
            return self.height_raw

        # 高度角度補正
        roll_angle = math.radians(self.STATE.ATT_ROLL)
        pitch_angle = math.radians(self.STATE.ATT_PITCH)
        self.height_offset = math.sqrt((self.height_raw**2)
                                       /(1+math.tan(roll_angle)**2+math.tan(pitch_angle)**2)) + self.offset
        # self.height_offset = (self.height_raw * math.cos(roll_angle)
        #                       * math.cos(pitch_angle)) + self.offset

        # 濾波器
        self.height_maf = self.maf.processing(self.height_offset)

        # 記錄量測時間
        self.mt = time.time()-mt_start

        # 記錄
        rec_row_data = [self.height_raw, self.height_offset, self.height_maf,
                        self.strength, self.rate, self.error, self.mt]
        # self.recorder.write_data(rec_row_data)
        self.recorder.rows_storage(rec_row_data)

        return self.height_maf

    # 參數更新與套用
    def update_and_apply_param(self):
        """參數更新與套用"""
        # 校正執行
        self._calibrate = False
        self.calibrate(sn=self.PARAM.value['SENS_ALT_LID_CAL_SN'])

        # 更新紀錄檔
        self.recorder.register()

    # 儲存記錄檔(任務結束時呼叫)
    def save_log(self):
        """儲存記錄檔"""
        self.recorder.write_log()

        # 訊息提示
        PrintLayout.info('高度感測器-光達，記錄檔已儲存')

    # 更改濾波器取樣數
    def change_filter_samples(self, samples=5, init=False):
        """更改濾波器取樣數"""
        self.maf.sample_number = samples
        if init:
            self.maf.reset()

    # 關閉連線
    def close(self):
        """關閉連線"""
        self.lidar.disable()
        self._calibrate = False
        PrintLayout.info('光達測距儀連線已關閉，錯誤次數共計%d次\n' % self.error)

