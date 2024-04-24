# TDK25 飛行組 競賽程式碼 - 驅動程式
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞

# 版本資訊
# 程式序號 TDK25-DRIVER
# 版號 02
# 最後編輯日 2021/09/30

# 程式說明

# 注意事項
# 執行時請用 if __name__ == '__main__'

# 模組匯入
import threading
import serial
import math
import time
import struct
import platform

if platform.platform() == 'macOS-10.16-x86_64-i386-64bit':
    import smbus2 as smbus
else:
    import smbus

# 副程式匯入
import TDK25.Tool as Tool
import TDK25.Parameter as Parameter
import TDK25.Variable as Variable
from TDK25.Tool import PrintLayout
import TDK25

# TDK25 飛行組 競賽程式碼 - 協程 - 光達
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞

# 版本資訊
# 程式序號 TDK25-Driver-LiDAR
# 版號 01
# 最後編輯日 2021/09/30

# 程式說明
# 高度感測器模組：光達 TFMiniS SEN0259
# 配合Asyncio調整模組結構
# 啟動後於背景持續執行，採用線程

# 注意事項
# 執行時請用 if __name__ == '__main__'


class Lidar:
    # 連線物件
    def __init__(self,  obj_handler):
        # 取得外部統一變數
        self.STATE = obj_handler.STATE
        self.CMD = obj_handler.CMD
        self.PARAM = obj_handler.PARAM
        self.UAV = obj_handler.UAV

        self.lock = threading.Lock()

        # 內部數值
        self._distance = 0
        self._strength = 0
        self.error_count = 0
        self.update_rate = 0
        self.opened = False
        self.exe_type = ''

    # 啟動背景線程
    def enable(self, bg_thread=False):
        # 建立序列通訊埠
        if self.STATE.FLAG_SIM:
            port = '/dev/cu.usbserial-0001'  # '/dev/cu.usbserial-0001'
        else:
            port = self.PARAM.value['SENS_ALT_LID_CONN_PORT']
        try:
            self.sensor = serial.Serial(port,
                                        self.PARAM.value['SENS_ALT_LID_CONN_BAUD'],
                                        self.PARAM.value['SENS_ALT_LID_BYTESIZE'],
                                        self.PARAM.value['SENS_ALT_LID_PARITY'],
                                        self.PARAM.value['SENS_ALT_LID_STOPBITS'],
                                        self.PARAM.value['SENS_ALT_LID_TIMEOUT'])
        except serial.serialutil.SerialException:
            PrintLayout.error('光達感測模組連線錯誤')
            return False
        else:
            if not self.sensor.is_open:
                self.sensor.open()
            self.opened = True
            PrintLayout.success('光達感測模組，就緒')
            # 線程啟動
            if bg_thread:
                self.exe_type = 'THR'
                self.sensor_thread = threading.Thread(target=self.run_thread)
                self.sensor_thread.start()
            return True

    # 訊息接收與轉譯
    def receive(self, single=False):
        # 清除暫存器
        if single:
            self.sensor.flushInput()

        # 執行迴圈
        while True:
            # 找出標頭
            while True:
                head_msg = self.sensor.read(1)
                if head_msg[0] == 89 and self.sensor.read(1)[0] == 89:
                    break
                else:
                    pass

            # 讀取剩餘
            flag_find_head = True
            raw_msg = self.sensor.read(7)
            if raw_msg[5] == 89 and raw_msg[6] == 89:
                flag_find_head = False
                self.error_count += 1
                PrintLayout.error('光達感測模組標頭解碼錯誤，錯誤累計 %d 次' % self.error_count)
            elif raw_msg[5] != 89 and raw_msg[6] == 89:
                if self.sensor.read(1) == 89:
                    flag_find_head = False
                    self.error_count += 1
                    PrintLayout.error('光達感測模組標頭解碼錯誤，錯誤累計 %d 次' % self.error_count)
            else:
                flag_find_head = True

            if flag_find_head:
                # 確認資料正確性
                check_sum = 89 + 89
                for seq in range(6):
                    check_sum += raw_msg[seq]
                check_sum_bytes = check_sum.to_bytes(8, 'big')

                # 計算距離、反射強度
                if check_sum_bytes[7] == raw_msg[6]:
                    self._distance = (raw_msg[0] + raw_msg[1] * 256) / 100
                    self._strength = (raw_msg[2] + raw_msg[3] * 256)
                    return
                else:
                    self.error_count += 1
                    PrintLayout.error('光達感測模組解碼錯誤，錯誤累計 %d 次' % self.error_count)
            else:
                flag_find_head = True

            if not single:
                return

    # 線程主迴圈
    def run_thread(self):
        """線程主迴圈"""
        # 建立計時器
        PrintLayout.info('光達感測模組-背景程序，啟動')
        flag_first_call = True
        timer = Tool.Timer()
        i = 0

        # 量測指令程序
        while self.opened:
            # 計數
            self.lock.acquire()
            i += 1

            # 訊息處理
            self.receive()

            if flag_first_call:
                self.update_rate = 0
                timer.period()
                flag_first_call = False
            else:
                self.update_rate = 1 / timer.period()

            # Debug Line
            # self.debug_test(i)

            # 正時等待
            self.lock.release()
            timer.wait_clock_arrive(i*(1/98))
        print('<SR-ALT> Time elapsed', timer.elapsed_time(), 'cycles:', i, 'avg rate', i/timer.elapsed_time())

    # 除錯測試用
    def debug_test(self, i):
        if not i % 10:
            PrintLayout.info('', False)
            print('{:10}'.format('Dis_raw=%.2f' % self.distance),
                  '{:10}'.format('Str=%d' % self.strength),
                  '{:13}'.format('Freq=%.2f' % self.update_rate),
                  '{:10}'.format('Err=%d' % self.error_count))

    # 關閉背景線程
    def disable(self):
        if self.opened:
            # 傳送關閉指令
            self.opened = False
            if self.exe_type == 'THR':
                self.sensor_thread.join()

            # 結束時關閉連線
            self.sensor.close()
            PrintLayout.info('光達感測模組-背景程序，關閉')
        else:
            pass

    @property
    def distance(self):
        return self._distance

    @property
    def strength(self):
        return self._strength


# TDK25 飛行組 競賽程式碼 - 相機模組
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞、徐慧哲

# 版本資訊
# 程式序號 TDK25-Driver-CAM
# 版號 01
# 最後編輯日 2021/10/01

# 程式說明
# 影像辨識

# 注意事項
# 參數採用外部引入方式進行


class Camera:
    """相機模組"""

    # 連線物件
    def __init__(self, obj_handler):
        # 取得外部統一變數
        self.STATE = obj_handler.STATE
        self.CMD = obj_handler.CMD
        self.PARAM = obj_handler.PARAM
        self.UAV = obj_handler.UAV

        self.lock = threading.Lock()

        # 內部數值
        self.opened = False

        # ->影像狀態控制
        self._img_state = ''
        self._detect_color = ''
        self._detect_mode = ''

        # ->影像位置、角度、面積
        self.pos_x_raw = 0
        self.pos_y_raw = 0
        self.angle_raw = 0
        self.area_raw = 0
        self.line_area_raw = 0
        self.update_rate_remote = 0
        self.update_rate_local = 0
        
        self.vertical_px = 354.893

        # 命令結果回傳區
        self.ack_mode_result = {
            'latest': False,
            'success': False,
            'mode': '',
            'color': '',
            'descrip': '',
        }

    # 啟動背景線程
    def enable(self):
        # 建立序列通訊埠
        if self.STATE.FLAG_SIM:
            port = '/dev/cu.usbserial-0001'
            # port = '/dev/cu.usbserial-A600HHJW'
        else:
            port = self.PARAM.value['SENS_CAM_CONN_PORT']
        try:
            self.camera = serial.Serial(port,
                                        self.PARAM.value['SENS_CAM_CONN_BAUD'],
                                        self.PARAM.value['SENS_CAM_BYTESIZE'],
                                        self.PARAM.value['SENS_CAM_PARITY'],
                                        self.PARAM.value['SENS_CAM_STOPBITS'],
                                        self.PARAM.value['SENS_CAM_TIMEOUT'])
        except serial.serialutil.SerialException:
            PrintLayout.error('影像模組，連線錯誤')
            return False
        else:
            if not self.camera.is_open:
                self.camera.open()
            self.opened = True
            PrintLayout.success('影像模組，就緒')
            # 線程啟動
            self.sensor_thread = threading.Thread(target=self.run)
            self.sensor_thread.start()
            return True

    # 接收程序主迴圈
    def run(self):
        """接收程序主迴圈"""
        # 建立計時器
        PrintLayout.info('影像模組-背景程序，啟動')
        flag_first_call = True
        timer = Tool.Timer()
        i = 0

        # 量測指令程序
        while self.opened:
            # 計數    不開Lock
            i += 1

            # 讀取狀態
            while not self.receive():
                pass

            # self.debug_test()

            if flag_first_call:
                self.update_rate_local = 0
                flag_first_call = False
            else:
                self.update_rate_local = 1 / timer.period()

            # 正時等待
            timer.wait_clock_arrive_ms(i * (1 / self.update_rate_remote))
        elapsed_time = timer.elapsed_time()
        print('Time elapsed', elapsed_time, 'cycles:', i, 'avg rate', i / elapsed_time)

    # 接收訊息
    def receive(self):
        # 讀取行訊息，構成：<{ID,msg} [check sum bytes]*2 \n>
        raw_msg = self.camera.readline()
        if len(raw_msg) < 10:
            return
        check_sum = raw_msg[-3] + 256 * raw_msg[-2]
        msg_sum = sum(raw_msg[0:-3])
        # 訊息檢查
        if msg_sum == check_sum:
            recv_msg = raw_msg[0:-3].decode().split(",")
            msg_head = recv_msg[0]
            # 根據收到的標頭執行對應的變數儲存
            if msg_head == 'stat':
                self._detect_mode = recv_msg[1]
                self._detect_color = recv_msg[2]
                self.pos_x_raw = float(recv_msg[3])
                self.pos_y_raw = float(recv_msg[4])
                self.area_raw = int(recv_msg[5])
                self.angle_raw = float(recv_msg[6])
                self.line_area_raw = int(recv_msg[7])
                self.update_rate_remote = float(recv_msg[8])
            elif msg_head == 'cmd':
                msg_ack_cmd = recv_msg[1]
                if msg_ack_cmd == 'mode':
                    self.ack_mode_result['success'] = self.str_to_bool(recv_msg[2])
                    self.ack_mode_result['mode'] = recv_msg[3]
                    self.ack_mode_result['color'] = recv_msg[4]
                    self.ack_mode_result['descrip'] = recv_msg[5]
                    self.ack_mode_result['latest'] = True
            return True
        else:
            return False

    # 傳送訊息
    def send(self, msg):
        # 檢查碼運算
        raw_msg = msg.encode()
        check_sum = sum(raw_msg)
        raw_msg += check_sum.to_bytes(2, 'little')
        raw_msg += '\n'.encode()
        # 寫入訊息
        self.camera.write(raw_msg)

    # 模式設定命令
    def set_mode(self, mode, color, wait=True, retry=True, timeout=1):
        msg = 'mode,%s,%s' % (mode, color)
        self.send(msg)
        result = False
        error_count = 0
        # 等待結果返回
        if wait:
            t0 = time.perf_counter()
            while True:
                time.sleep(0.5)
                # 回傳訊息
                if self.ack_mode_result['success'] and mode == self.ack_mode_result['mode'] and color == self.ack_mode_result['color'] :
                    PrintLayout.success('影像模組，模式設定成功，模式=%s，顏色=%s，原因=%s' % (
                        self.ack_mode_result['mode'], self.ack_mode_result['color'], self.ack_mode_result['descrip']
                    ))
                    self.ack_mode_result['latest'] = False
                    return True
                else:
                    PrintLayout.error('影像模組，模式設定失敗，模式=%s，顏色=%s，原因=%s' % (
                        self.ack_mode_result['mode'], self.ack_mode_result['color'], self.ack_mode_result['descrip']
                    ))
                    self.ack_mode_result['latest'] = False
                    if not retry:
                        return False
                    error_count += 1

                    msg = 'mode,%s,%s' % (mode, color)
                    self.send(msg)

                if time.perf_counter() - t0 > timeout and error_count == 3:
                    return False


    # 除錯測試用
    def debug_test(self):
        if self.detect_mode == 'position':
            PrintLayout.info('Position Mode, Color=%s, X=%.2f m, Y=%.2f m, area= %.3f m^2, fps=%.1f' %
                             (self.detect_color, self.pos_x, self.pos_y, self.area, self.update_rate_local))
        elif self.detect_mode == 'tracking':
            PrintLayout.info(
                'Tracking Mode, Color=%s, Y=%.2f m, Theta=%.2f deg, area= %.3f m^2, line_area= %.3f m^2, fps=%.1f' %
                (self.detect_color, self.pos_y, self.angle, self.area, self.line_area_raw, self.update_rate_local))
        elif self.detect_mode == 'take_off':
            PrintLayout.info(
                'Takeoff Mode, Color=%s, Y=%.2f m, Theta=%.2f deg, area= %.3f m^2, line_area= %.3f m^2, fps=%.1f' %
                (self.detect_color, self.pos_y, self.angle, self.area, self.line_area_raw, self.update_rate_local))
        else:
            pass

    # 將布林字串轉成布林格式
    @staticmethod
    def str_to_bool(string=''):
        if string == 'True' or string == 'TRUE' or string == 'true':
            return True
        else:
            return False

    @property
    def pos_x(self):
        """回傳目前色塊X位置"""
        # 計算畫素->公尺
        pos_x_meter = (self.pos_x_raw/self.vertical_px) * (self.STATE.ALT_REL + self.PARAM.value['CAM_SENSOR_TO_CAM'])
        #cam_height = round(math.tan(self.PARAM.value['CAM_ANGLE_X'] * math.pi / 180 / 2), 3) * (
        #        self.STATE.ALT_REL + self.PARAM.value['CAM_SENSOR_TO_CAM'])
        #pixel_to_meter = cam_height / self.PARAM.value['CAM_BODY_REF_X']
        #pos_x_meter = self.pos_x_raw * pixel_to_meter
        return pos_x_meter

    @property
    def pos_y(self):
        """回傳目前色塊Y位置"""
        # 計算畫素->公尺
        pos_y_meter = (self.pos_y_raw/self.vertical_px) * (self.STATE.ALT_REL + self.PARAM.value['CAM_SENSOR_TO_CAM'])
        #cam_width = round(math.tan(self.PARAM.value['CAM_ANGLE_Y'] * math.pi / 180 / 2), 3) * (
        #        self.STATE.ALT_REL + self.PARAM.value['CAM_SENSOR_TO_CAM'])
        #pixel_to_meter = cam_width / self.PARAM.value['CAM_BODY_REF_Y']
        #pos_y_meter = self.pos_y_raw * pixel_to_meter
        return pos_y_meter

    @property
    def area(self):
        """回傳目前色塊面積"""
        total_px = 307200
        actual_alt = self.STATE.ALT_REL + self.PARAM.value['CAM_SENSOR_TO_CAM']
        actual_height = 2*actual_alt*math.tan(math.radians(self.PARAM.value['CAM_ANGLE_X']/2))
        actual_width = 2*actual_alt*math.tan(math.radians(self.PARAM.value['CAM_ANGLE_Y']/2))
        actual_area = actual_height*actual_width
        area_meter = (self.area_raw/307200) * actual_area
        #cam_height = round(math.tan(self.PARAM.value['CAM_ANGLE_X'] * math.pi / 180 / 2), 3) * (
        #        self.STATE.ALT_REL + self.PARAM.value['CAM_SENSOR_TO_CAM'])
        #cam_width = round(math.tan(self.PARAM.value['CAM_ANGLE_Y'] * math.pi / 180 / 2), 3) * (
        #        self.STATE.ALT_REL + self.PARAM.value['CAM_SENSOR_TO_CAM'])
        #length_per_pixel = (cam_height * cam_width) / (self.PARAM.value['CAM_BODY_REF_Y'] *
        #                                               self.PARAM.value['CAM_BODY_REF_X'])
        #area_meter = length_per_pixel * self.area_raw
        return area_meter

    @property
    def detect_mode(self):
        return self._detect_mode

    @property
    def detect_color(self):
        return self._detect_color

    @property
    def angle(self):
        """回傳線夾角"""
        return self.angle_raw

    # 關閉背景線程
    def disable(self):
        if self.opened:
            # 傳送關閉指令
            self.opened = False
            self.sensor_thread.join()

            # 結束時關閉連線
            self.camera.close()
            PrintLayout.info('影像模組-背景程序，關閉')
        else:
            pass


# TDK25 飛行組 競賽程式碼 - 姿態模組
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞、徐慧哲

# 版本資訊
# 程式序號 TDK25-Driver-Attitude
# 版號 01
# 最後編輯日 2021/12/08

# 程式說明
# 取得Roll&Pitch姿態，使用互補濾波器與MPU6050

# 注意事項
# 參數採用外部引入方式進行


class AttitudeSensor:

    # Global Variables
    GRAVITY_MS2 = 9.80665
    address = None
    bus = None

    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0

    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    FILTER_BW_256 = 0x00
    FILTER_BW_188 = 0x01
    FILTER_BW_98 = 0x02
    FILTER_BW_42 = 0x03
    FILTER_BW_20 = 0x04
    FILTER_BW_10 = 0x05
    FILTER_BW_5 = 0x06

    # MPU-6050 Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F

    TEMP_OUT0 = 0x41

    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47

    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B
    MPU_CONFIG = 0x1A

    def __init__(self, cf=0.97, address=0x68, bus=1):
        self.address = address
        self.bus = smbus.SMBus(bus)
        # Wake up the MPU-6050 since it starts in sleep mode
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

        self.set_accel_range(self.ACCEL_RANGE_2G)
        self.set_gyro_range(self.GYRO_RANGE_500DEG)

        self.cf_weight = cf

        self.pre_roll = 0
        self.pre_pitch = 0

        self.flag_first_call = True
        self.timer = Tool.Timer()

    # I2C communication methods

    def read_i2c_word(self, register):
        """Read two i2c registers and combine them.
        register -- the first register to read from.
        Returns the combined read results.
        """
        # Read the data from the registers
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)

        value = (high << 8) + low

        if value >= 0x8000:
            return -((65535 - value) + 1)
        else:
            return value

    # MPU-6050 Methods

    def get_temp(self):
        """Reads the temperature from the onboard temperature sensor of the MPU-6050.
        Returns the temperature in degrees Celcius.
        """
        raw_temp = self.read_i2c_word(self.TEMP_OUT0)

        # Get the actual temperature using the formulae given in the
        # MPU-6050 Register Map and Descriptions revision 4.2, page 30
        actual_temp = (raw_temp / 340.0) + 36.53

        return actual_temp

    def set_accel_range(self, accel_range):
        """Sets the range of the accelerometer to range.
        accel_range -- the range to set the accelerometer to. Using a
        pre-defined range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)

    def read_accel_range(self, raw=False):
        """Reads the range the accelerometer is set to.
        If raw is True, it will return the raw value from the ACCEL_CONFIG
        register
        If raw is False, it will return an integer: -1, 2, 4, 8 or 16. When it
        returns -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.ACCEL_RANGE_2G:
                return 2
            elif raw_data == self.ACCEL_RANGE_4G:
                return 4
            elif raw_data == self.ACCEL_RANGE_8G:
                return 8
            elif raw_data == self.ACCEL_RANGE_16G:
                return 16
            else:
                return -1

    def get_accel_data(self, g=False):
        """Gets and returns the X, Y and Z values from the accelerometer.
        If g is True, it will return the data in g
        If g is False, it will return the data in m/s^2
        Returns a dictionary with the measurement results.
        """
        x = self.read_i2c_word(self.ACCEL_XOUT0)
        y = self.read_i2c_word(self.ACCEL_YOUT0)
        z = self.read_i2c_word(self.ACCEL_ZOUT0)

        accel_range = self.read_accel_range(True)

        if accel_range == self.ACCEL_RANGE_2G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
        elif accel_range == self.ACCEL_RANGE_4G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
        elif accel_range == self.ACCEL_RANGE_8G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
        elif accel_range == self.ACCEL_RANGE_16G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
        else:
            print("Unkown range - accel_scale_modifier set to self.ACCEL_SCALE_MODIFIER_2G")
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G

        x = x / accel_scale_modifier
        y = y / accel_scale_modifier
        z = z / accel_scale_modifier

        if g is True:
            return {'x': x, 'y': y, 'z': z}
        elif g is False:
            x = x * self.GRAVITY_MS2
            y = y * self.GRAVITY_MS2
            z = z * self.GRAVITY_MS2
            return {'x': x, 'y': y, 'z': z}

    def set_gyro_range(self, gyro_range):
        """Sets the range of the gyroscope to range.
        gyro_range -- the range to set the gyroscope to. Using a pre-defined
        range is advised.
        """
        # First change it to 0x00 to make sure we write the correct value later
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)

        # Write the new range to the ACCEL_CONFIG register
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)

    def set_filter_range(self, filter_range=FILTER_BW_256):
        """Sets the low-pass bandpass filter frequency"""
        # Keep the current EXT_SYNC_SET configuration in bits 3, 4, 5 in the MPU_CONFIG register
        EXT_SYNC_SET = self.bus.read_byte_data(self.address, self.MPU_CONFIG) & 0b00111000
        return self.bus.write_byte_data(self.address, self.MPU_CONFIG,  EXT_SYNC_SET | filter_range)

    def read_gyro_range(self, raw=False):
        """Reads the range the gyroscope is set to.
        If raw is True, it will return the raw value from the GYRO_CONFIG
        register.
        If raw is False, it will return 250, 500, 1000, 2000 or -1. If the
        returned value is equal to -1 something went wrong.
        """
        raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)

        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.GYRO_RANGE_250DEG:
                return 250
            elif raw_data == self.GYRO_RANGE_500DEG:
                return 500
            elif raw_data == self.GYRO_RANGE_1000DEG:
                return 1000
            elif raw_data == self.GYRO_RANGE_2000DEG:
                return 2000
            else:
                return -1

    def get_gyro_data(self):
        """Gets and returns the X, Y and Z values from the gyroscope.
        Returns the read values in a dictionary.
        """
        x = self.read_i2c_word(self.GYRO_XOUT0)
        y = self.read_i2c_word(self.GYRO_YOUT0)
        z = self.read_i2c_word(self.GYRO_ZOUT0)

        gyro_range = self.read_gyro_range(True)

        if gyro_range == self.GYRO_RANGE_250DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
        elif gyro_range == self.GYRO_RANGE_500DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
        elif gyro_range == self.GYRO_RANGE_1000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
        elif gyro_range == self.GYRO_RANGE_2000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
        else:
            print("Unknown range - gyro_scale_modifier set to self.GYRO_SCALE_MODIFIER_250DEG")
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG

        x = x / gyro_scale_modifier
        y = y / gyro_scale_modifier
        z = z / gyro_scale_modifier

        return {'x': x, 'y': y, 'z': z}

    def get_all_data(self):
        """Reads and returns all the available data."""
        temp = self.get_temp()
        accel = self.get_accel_data()
        gyro = self.get_gyro_data()

        return [accel, gyro, temp]

    def get_attitude(self):
        accel = self.get_accel_data()
        gyro = self.get_gyro_data()
        accel_x = accel['x']-0.1
        accel_y = accel['y']-0.125
        accel_z = accel['z']-3
        accel_roll = math.degrees(math.atan2(-accel_y, -accel_z))
        accel_pitch = math.degrees(math.atan2(accel_x, -accel_z))

        if self.flag_first_call:
            dt = 0
            self.flag_first_call = False
        else:
            dt = self.timer.period()
        roll = self.cf_weight * (self.pre_roll + gyro['x']*dt) + (self.cf_weight - 1) * accel_roll
        pitch = self.cf_weight * (self.pre_pitch + gyro['y']*dt) + (self.cf_weight - 1) * accel_pitch

        self.pre_roll = roll
        self.pre_pitch = pitch

        return roll, pitch


# TDK25 飛行組 競賽程式碼 - 姿態模組
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞

# 版本資訊
# 程式序號 TDK25-Driver-IMU
# 版號 01
# 最後編輯日 2021/12/08

# 程式說明
# 取得Roll&Pitch姿態，使用BNO055

# 注意事項
# 參數採用外部引入方式進行

class BNO055:
    BNO055_ADDRESS_A = 0x28
    BNO055_ADDRESS_B = 0x29
    BNO055_ID = 0xA0

    # Power mode settings
    POWER_MODE_NORMAL = 0X00
    POWER_MODE_LOWPOWER = 0X01
    POWER_MODE_SUSPEND = 0X02

    # Operation mode settings
    OPERATION_MODE_CONFIG = 0X00
    OPERATION_MODE_ACCONLY = 0X01
    OPERATION_MODE_MAGONLY = 0X02
    OPERATION_MODE_GYRONLY = 0X03
    OPERATION_MODE_ACCMAG = 0X04
    OPERATION_MODE_ACCGYRO = 0X05
    OPERATION_MODE_MAGGYRO = 0X06
    OPERATION_MODE_AMG = 0X07
    OPERATION_MODE_IMUPLUS = 0X08
    OPERATION_MODE_COMPASS = 0X09
    OPERATION_MODE_M4G = 0X0A
    OPERATION_MODE_NDOF_FMC_OFF = 0X0B
    OPERATION_MODE_NDOF = 0X0C

    # Output vector type
    VECTOR_ACCELEROMETER = 0x08
    VECTOR_MAGNETOMETER = 0x0E
    VECTOR_GYROSCOPE = 0x14
    VECTOR_EULER = 0x1A
    VECTOR_LINEARACCEL = 0x28
    VECTOR_GRAVITY = 0x2E

    # REGISTER DEFINITION START
    BNO055_PAGE_ID_ADDR = 0X07

    BNO055_CHIP_ID_ADDR = 0x00
    BNO055_ACCEL_REV_ID_ADDR = 0x01
    BNO055_MAG_REV_ID_ADDR = 0x02
    BNO055_GYRO_REV_ID_ADDR = 0x03
    BNO055_SW_REV_ID_LSB_ADDR = 0x04
    BNO055_SW_REV_ID_MSB_ADDR = 0x05
    BNO055_BL_REV_ID_ADDR = 0X06

    # Accel data register
    BNO055_ACCEL_DATA_X_LSB_ADDR = 0X08
    BNO055_ACCEL_DATA_X_MSB_ADDR = 0X09
    BNO055_ACCEL_DATA_Y_LSB_ADDR = 0X0A
    BNO055_ACCEL_DATA_Y_MSB_ADDR = 0X0B
    BNO055_ACCEL_DATA_Z_LSB_ADDR = 0X0C
    BNO055_ACCEL_DATA_Z_MSB_ADDR = 0X0D

    # Mag data register
    BNO055_MAG_DATA_X_LSB_ADDR = 0X0E
    BNO055_MAG_DATA_X_MSB_ADDR = 0X0F
    BNO055_MAG_DATA_Y_LSB_ADDR = 0X10
    BNO055_MAG_DATA_Y_MSB_ADDR = 0X11
    BNO055_MAG_DATA_Z_LSB_ADDR = 0X12
    BNO055_MAG_DATA_Z_MSB_ADDR = 0X13

    # Gyro data registers
    BNO055_GYRO_DATA_X_LSB_ADDR = 0X14
    BNO055_GYRO_DATA_X_MSB_ADDR = 0X15
    BNO055_GYRO_DATA_Y_LSB_ADDR = 0X16
    BNO055_GYRO_DATA_Y_MSB_ADDR = 0X17
    BNO055_GYRO_DATA_Z_LSB_ADDR = 0X18
    BNO055_GYRO_DATA_Z_MSB_ADDR = 0X19

    # Euler data registers
    BNO055_EULER_H_LSB_ADDR = 0X1A
    BNO055_EULER_H_MSB_ADDR = 0X1B
    BNO055_EULER_R_LSB_ADDR = 0X1C
    BNO055_EULER_R_MSB_ADDR = 0X1D
    BNO055_EULER_P_LSB_ADDR = 0X1E
    BNO055_EULER_P_MSB_ADDR = 0X1F

    # Quaternion data registers
    BNO055_QUATERNION_DATA_W_LSB_ADDR = 0X20
    BNO055_QUATERNION_DATA_W_MSB_ADDR = 0X21
    BNO055_QUATERNION_DATA_X_LSB_ADDR = 0X22
    BNO055_QUATERNION_DATA_X_MSB_ADDR = 0X23
    BNO055_QUATERNION_DATA_Y_LSB_ADDR = 0X24
    BNO055_QUATERNION_DATA_Y_MSB_ADDR = 0X25
    BNO055_QUATERNION_DATA_Z_LSB_ADDR = 0X26
    BNO055_QUATERNION_DATA_Z_MSB_ADDR = 0X27

    # Linear acceleration data registers
    BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR = 0X28
    BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR = 0X29
    BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR = 0X2A
    BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR = 0X2B
    BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR = 0X2C
    BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR = 0X2D

    # Gravity data registers
    BNO055_GRAVITY_DATA_X_LSB_ADDR = 0X2E
    BNO055_GRAVITY_DATA_X_MSB_ADDR = 0X2F
    BNO055_GRAVITY_DATA_Y_LSB_ADDR = 0X30
    BNO055_GRAVITY_DATA_Y_MSB_ADDR = 0X31
    BNO055_GRAVITY_DATA_Z_LSB_ADDR = 0X32
    BNO055_GRAVITY_DATA_Z_MSB_ADDR = 0X33

    # Temperature data register
    BNO055_TEMP_ADDR = 0X34

    # Status registers
    BNO055_CALIB_STAT_ADDR = 0X35
    BNO055_SELFTEST_RESULT_ADDR = 0X36
    BNO055_INTR_STAT_ADDR = 0X37

    BNO055_SYS_CLK_STAT_ADDR = 0X38
    BNO055_SYS_STAT_ADDR = 0X39
    BNO055_SYS_ERR_ADDR = 0X3A

    # Unit selection register
    BNO055_UNIT_SEL_ADDR = 0X3B
    BNO055_DATA_SELECT_ADDR = 0X3C

    # Mode registers
    BNO055_OPR_MODE_ADDR = 0X3D
    BNO055_PWR_MODE_ADDR = 0X3E

    BNO055_SYS_TRIGGER_ADDR = 0X3F
    BNO055_TEMP_SOURCE_ADDR = 0X40

    # Axis remap registers
    BNO055_AXIS_MAP_CONFIG_ADDR = 0X41
    BNO055_AXIS_MAP_SIGN_ADDR = 0X42

    # SIC registers
    BNO055_SIC_MATRIX_0_LSB_ADDR = 0X43
    BNO055_SIC_MATRIX_0_MSB_ADDR = 0X44
    BNO055_SIC_MATRIX_1_LSB_ADDR = 0X45
    BNO055_SIC_MATRIX_1_MSB_ADDR = 0X46
    BNO055_SIC_MATRIX_2_LSB_ADDR = 0X47
    BNO055_SIC_MATRIX_2_MSB_ADDR = 0X48
    BNO055_SIC_MATRIX_3_LSB_ADDR = 0X49
    BNO055_SIC_MATRIX_3_MSB_ADDR = 0X4A
    BNO055_SIC_MATRIX_4_LSB_ADDR = 0X4B
    BNO055_SIC_MATRIX_4_MSB_ADDR = 0X4C
    BNO055_SIC_MATRIX_5_LSB_ADDR = 0X4D
    BNO055_SIC_MATRIX_5_MSB_ADDR = 0X4E
    BNO055_SIC_MATRIX_6_LSB_ADDR = 0X4F
    BNO055_SIC_MATRIX_6_MSB_ADDR = 0X50
    BNO055_SIC_MATRIX_7_LSB_ADDR = 0X51
    BNO055_SIC_MATRIX_7_MSB_ADDR = 0X52
    BNO055_SIC_MATRIX_8_LSB_ADDR = 0X53
    BNO055_SIC_MATRIX_8_MSB_ADDR = 0X54

    # Accelerometer Offset registers
    ACCEL_OFFSET_X_LSB_ADDR = 0X55
    ACCEL_OFFSET_X_MSB_ADDR = 0X56
    ACCEL_OFFSET_Y_LSB_ADDR = 0X57
    ACCEL_OFFSET_Y_MSB_ADDR = 0X58
    ACCEL_OFFSET_Z_LSB_ADDR = 0X59
    ACCEL_OFFSET_Z_MSB_ADDR = 0X5A

    # Magnetometer Offset registers
    MAG_OFFSET_X_LSB_ADDR = 0X5B
    MAG_OFFSET_X_MSB_ADDR = 0X5C
    MAG_OFFSET_Y_LSB_ADDR = 0X5D
    MAG_OFFSET_Y_MSB_ADDR = 0X5E
    MAG_OFFSET_Z_LSB_ADDR = 0X5F
    MAG_OFFSET_Z_MSB_ADDR = 0X60

    # Gyroscope Offset registers
    GYRO_OFFSET_X_LSB_ADDR = 0X61
    GYRO_OFFSET_X_MSB_ADDR = 0X62
    GYRO_OFFSET_Y_LSB_ADDR = 0X63
    GYRO_OFFSET_Y_MSB_ADDR = 0X64
    GYRO_OFFSET_Z_LSB_ADDR = 0X65
    GYRO_OFFSET_Z_MSB_ADDR = 0X66

    # Radius registers
    ACCEL_RADIUS_LSB_ADDR = 0X67
    ACCEL_RADIUS_MSB_ADDR = 0X68
    MAG_RADIUS_LSB_ADDR = 0X69
    MAG_RADIUS_MSB_ADDR = 0X6A

    # REGISTER DEFINITION END

    def __init__(self, sensorId=-1, address=0x28):
        self._sensorId = sensorId
        self._address = address
        self._mode = BNO055.OPERATION_MODE_NDOF

    def begin(self, mode=None):
        if mode is None: mode = BNO055.OPERATION_MODE_NDOF
        # Open I2C bus
        self._bus = smbus.SMBus(1)

        # Make sure we have the right device
        if self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0] != BNO055.BNO055_ID:
            time.sleep(1)  # Wait for the device to boot up
            if self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0] != BNO055.BNO055_ID:
                return False

        # Switch to config mode
        self.setMode(BNO055.OPERATION_MODE_CONFIG)

        # Trigger a reset and wait for the device to boot up again
        self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0x20])
        time.sleep(1)
        while self.readBytes(BNO055.BNO055_CHIP_ID_ADDR)[0] != BNO055.BNO055_ID:
            time.sleep(0.01)
        time.sleep(0.05)

        # Set to normal power mode
        self.writeBytes(BNO055.BNO055_PWR_MODE_ADDR, [BNO055.POWER_MODE_NORMAL])
        time.sleep(0.01)

        self.writeBytes(BNO055.BNO055_PAGE_ID_ADDR, [0])
        self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0])
        time.sleep(0.01)

        # Set the requested mode
        self.setMode(mode)
        time.sleep(0.02)

        return True

    def setMode(self, mode):
        self._mode = mode
        self.writeBytes(BNO055.BNO055_OPR_MODE_ADDR, [self._mode])
        time.sleep(0.03)

    def setExternalCrystalUse(self, useExternalCrystal=True):
        prevMode = self._mode
        self.setMode(BNO055.OPERATION_MODE_CONFIG)
        time.sleep(0.025)
        self.writeBytes(BNO055.BNO055_PAGE_ID_ADDR, [0])
        self.writeBytes(BNO055.BNO055_SYS_TRIGGER_ADDR, [0x80] if useExternalCrystal else [0])
        time.sleep(0.01)
        self.setMode(prevMode)
        time.sleep(0.02)

    def getSystemStatus(self):
        self.writeBytes(BNO055.BNO055_PAGE_ID_ADDR, [0])
        (sys_stat, sys_err) = self.readBytes(BNO055.BNO055_SYS_STAT_ADDR, 2)
        self_test = self.readBytes(BNO055.BNO055_SELFTEST_RESULT_ADDR)[0]
        return (sys_stat, self_test, sys_err)

    def getRevInfo(self):
        (accel_rev, mag_rev, gyro_rev) = self.readBytes(BNO055.BNO055_ACCEL_REV_ID_ADDR, 3)
        sw_rev = self.readBytes(BNO055.BNO055_SW_REV_ID_LSB_ADDR, 2)
        sw_rev = sw_rev[0] | sw_rev[1] << 8
        bl_rev = self.readBytes(BNO055.BNO055_BL_REV_ID_ADDR)[0]
        return (accel_rev, mag_rev, gyro_rev, sw_rev, bl_rev)

    def getCalibration(self):
        calData = self.readBytes(BNO055.BNO055_CALIB_STAT_ADDR)[0]
        return (calData >> 6 & 0x03, calData >> 4 & 0x03, calData >> 2 & 0x03, calData & 0x03)

    def getTemp(self):
        return self.readBytes(BNO055.BNO055_TEMP_ADDR)[0]

    def getVector(self, vectorType):
        buf = self.readBytes(vectorType, 6)
        xyz = struct.unpack('hhh', struct.pack('BBBBBB', buf[0], buf[1], buf[2], buf[3], buf[4], buf[5]))
        if vectorType == BNO055.VECTOR_MAGNETOMETER:
            scalingFactor = 16.0
        elif vectorType == BNO055.VECTOR_GYROSCOPE:
            scalingFactor = 900.0
        elif vectorType == BNO055.VECTOR_EULER:
            scalingFactor = 16.0
        elif vectorType == BNO055.VECTOR_GRAVITY:
            scalingFactor = 100.0
        else:
            scalingFactor = 1.0
        return tuple([i / scalingFactor for i in xyz])

    def getQuat(self):
        buf = self.readBytes(BNO055.BNO055_QUATERNION_DATA_W_LSB_ADDR, 8)
        wxyz = struct.unpack('hhhh',
                             struct.pack('BBBBBBBB', buf[0], buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7]))
        return tuple([i * (1.0 / (1 << 14)) for i in wxyz])

    def readBytes(self, register, numBytes=1):
        return self._bus.read_i2c_block_data(self._address, register, numBytes)

    def writeBytes(self, register, byteVals):
        return self._bus.write_i2c_block_data(self._address, register, byteVals)


if __name__ == '__main__':
    bno = BNO055()
    if bno.begin() is not True:
        print("Error initializing device")
        exit()
    time.sleep(1)
    while True:
        print(bno.getVector(BNO055.VECTOR_EULER))
        time.sleep(0.1)


if __name__ == '__main__':
    attitude = AttitudeSensor(cf=0.97)
    while True:
        r, p = attitude.get_attitude()
        print('Roll= %.2f deg, Pitch= %.2f deg' % (r, p))
        time.sleep(0.1)


if __name__ == '__main__' and False:
    STATE = Variable.StateVariable()
    STATE.ALT_REL = 1
    CMD = Variable.Command()
    PARAM = Parameter.Param()
    PARAM_IO = Parameter.ParamIO()
    PARAM.update(PARAM_IO.export())
    obj = TDK25.ObjectHandler(STATE, CMD, PARAM, PARAM_IO, object)
    cam = Camera(obj)
    mode_list = ['position', 'tracking', 'take_off']
    color_list = ['red_light', 'red_dot_grab_on', 'red_dot_drop_off', 'blue', 'green',
                  'black_dot', 'black_line', 'silver']
    if cam.enable():
        for mode in mode_list:
            for color in color_list:
                cam.set_mode(mode, color)
                time.sleep(3)

    cam.disable()

"""
if __name__ == '__main__':
    STATE = Variable.StateVariable()
    CMD = Variable.Command()
    PARAM = Parameter.Param()
    PARAM_IO = Parameter.ParamIO()
    PARAM.update(PARAM_IO.export())
    obj = TDK25.ObjectHandler(STATE, CMD, PARAM, PARAM_IO, object)
    lidar = Lidar(obj)

    if lidar.enable(bg_thread=True):
        STATE.FLAG_STATE = 'EXECUTE'
        time.sleep(20)

    lidar.disable()



if __name__ == '__main__':
    import time
    import serial

    ser = serial.Serial('/dev/cu.usbserial-0001', 921600, 8, 'N', 1, 10)
    p_time = time.time()
    t0 = p_time
    i = 0
    while True:
        i += 1
        # cmd = 'm3c1'
        # ser.write(cmd.encode())
        msg_raw = ser.readline()
        # print(msg_raw)
        # print(msg_raw[0:-3].decode().split(","))

        if i == 100:
            msg_s = 'mode,position,red_light'.encode()
            msg_s += sum(msg_s).to_bytes(2,'little')
            msg_s += '\n'.encode()
            ser.write(msg_s)
            print('send')

        if i == 300:
            msg_s = 'mode,position,blue'.encode()
            msg_s += sum(msg_s).to_bytes(2,'little')
            msg_s += '\n'.encode()
            ser.write(msg_s)
            print('send')

        if i == 500:
            msg_s = 'mode,position,green'.encode()
            msg_s += sum(msg_s).to_bytes(2,'little')
            msg_s += '\n'.encode()
            ser.write(msg_s)
            print('send')

        if i == 700:
            msg_s = 'mode,position,silver'.encode()
            msg_s += sum(msg_s).to_bytes(2,'little')
            msg_s += '\n'.encode()
            ser.write(msg_s)
            print('send')

        if i == 900:
            msg_s = 'mode,position,red_dot_drop_off'.encode()
            msg_s += sum(msg_s).to_bytes(2, 'little')
            msg_s += '\n'.encode()
            ser.write(msg_s)
            print('send',msg_s)

        if i == 1100:
            msg_s = 'mode,position,red_dot_grab_on'.encode()
            msg_s += sum(msg_s).to_bytes(2, 'little')
            msg_s += '\n'.encode()
            ser.write(msg_s)
            print('send')

        if i == 1300:
            msg_s = 'mode,position,black_dot'.encode()
            msg_s += sum(msg_s).to_bytes(2, 'little')
            msg_s += '\n'.encode()
            ser.write(msg_s)
            print('send')

        if i == 1500:
            msg_s = 'mode,position,black_line'.encode()
            msg_s += sum(msg_s).to_bytes(2,'little')
            msg_s += '\n'.encode()
            ser.write(msg_s)

        if i == 1700:
            msg_s = 'mode,tracking,red_light'.encode()
            msg_s += sum(msg_s).to_bytes(2,'little')
            msg_s += '\n'.encode()
            ser.write(msg_s)

        if i == 1700:
            msg_s = 'mode,tracking,blue'.encode()
            msg_s += sum(msg_s).to_bytes(2,'little')
            msg_s += '\n'.encode()
            ser.write(msg_s)

        if i == 1900:
            msg_s = 'mode,tracking,green'.encode()
            msg_s += sum(msg_s).to_bytes(2,'little')
            msg_s += '\n'.encode()
            ser.write(msg_s)
            print('send')

        if i == 2100:
            msg_s = 'mode,tracking,silver'.encode()
            msg_s += sum(msg_s).to_bytes(2,'little')
            msg_s += '\n'.encode()
            ser.write(msg_s)
            print('send')

        if i == 2300:
            msg_s = 'mode,tracking,red_dot_drop_off'.encode()
            msg_s += sum(msg_s).to_bytes(2, 'little')
            msg_s += '\n'.encode()
            ser.write(msg_s)
            print('send')

        if i == 2500:
            msg_s = 'mode,tracking,red_dot_grab_on'.encode()
            msg_s += sum(msg_s).to_bytes(2, 'little')
            msg_s += '\n'.encode()
            ser.write(msg_s)
            print('send')

        if i == 2700:
            msg_s = 'mode,takeoff,red_light'.encode()
            msg_s += sum(msg_s).to_bytes(2,'little')
            msg_s += '\n'.encode()
            ser.write(msg_s)

        # print(msg_raw)
        # mode, color, pos_x, pos_y, area, angle, fps = msg_raw.decode().split(",")
        c_time = time.time()
        dt = c_time - p_time
        print(msg_raw[0:-3].decode().split(","), 1/dt)
        # print('receive:', mode, color, pos_x, pos_y, area, angle, '%.2f' % (1 / dt), fps)
        p_time = c_time
"""
