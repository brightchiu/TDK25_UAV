# TDK25 飛行組 競賽程式碼 - 線程模組
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞

# 版本資訊
# 程式序號 TDK25-ThreadScheduler
# 版號 02
# 最後編輯日 2021/11/29

# 程式說明
# 採用協程方式，獨立while迴圈
# 包含：高度感測器 位置-速度控制器 航向控制器 影像辨識模組 命令傳輸模組

# 注意事項

# 模組匯入
import dronekit_sitl
import dronekit
import threading
import platform
import sched
import time
import math

# 副程式匯入
import TDK25.Controller as Controller
import TDK25.Parameter as Parameter
import TDK25.Variable as Variable
import TDK25.Sensor as Sensor
import TDK25.Driver as Driver
import TDK25.Camera as Camera
import TDK25.Tool as Tool
from TDK25.Tool import PrintLayout


class ThreadScheduler:
    """線程管理器"""

    # 初始化線程
    def __init__(self, obj_handler):
        # 取得外部統一變數
        self.OBJ_HANDLER = obj_handler
        self.STATE = obj_handler.STATE
        self.CMD = obj_handler.CMD
        self.PARAM = obj_handler.PARAM
        self.PARAM_IO = obj_handler.PARAM_IO
        self.UAV = obj_handler.UAV

        # 線程狀態檢查標籤
        self.flag_thread_on = False

        # 線程排程器
        self.scheduler = sched.scheduler(time.perf_counter, time.sleep)

        # 建立計時器
        self.timer = Tool.Timer()

        # 初始化執行單元
        if self.CMD.SW_THR_SR_ALT:
            self.sr_alt = SensorAlt(obj_handler)
            self.sr_cam = SensorCam(obj_handler)
        self.ctrl_pos = ControllerPos(obj_handler)
        self.ctrl_hdg = ControllerHdg(obj_handler)
        self.cmd = CommandTransmit(obj_handler)

        # 線程狀態聯集檢查
        self.STATE.FLAG_THREAD_READY = (self.STATE.FLAG_SR_ALT and self.STATE.FLAG_SR_CAM and
                                        self.STATE.FLAG_CTRL_POS and self.STATE.FLAG_CTRL_HDG and self.STATE.FLAG_CT)
        # self.STATE.FLAG_THREAD_READY = True
        if self.STATE.FLAG_THREAD_READY:
            PrintLayout.success('線程模組就緒')
        elif not self.STATE.FLAG_THREAD_READY and self.STATE.FLAG_SIM:
            self.STATE.FLAG_THREAD_READY = True
            PrintLayout.success('線程模組就緒，模擬模式下感測器不啟用')
        else:
            PrintLayout.error('線程模組啟動錯誤，請檢查後再啟動')

    # 執行前檢查
    def entry_check(self):
        # 線程就緒檢查
        if not self.STATE.FLAG_THREAD_READY:
            PrintLayout.warning('線程未就緒，執行禁止')
            return False

        # 線程重置中檢查
        elif self.STATE.FLAG_THREAD_RESET:
            PrintLayout.warning('線程重置中，執行禁止')
            return False

        # 狀態機檢查
        elif self.STATE.FLAG_STATE != 'EXECUTE':
            PrintLayout.warning('線程執行狀態錯誤，現在狀態為：%s' % self.STATE.FLAG_STATE)
            return False

        else:
            return True

    # 背景循環線程
    def processing(self):
        """背景循環線程"""
        # 執行前檢查
        if not self.entry_check():
            return

        # 任務時間更新與參數備份
        self.STATE.set_mission_time()
        self.PARAM_IO.update(self.PARAM.export())
        self.PARAM_IO.write_param_into_csv(backup=True, mission_time_str=self.STATE.get_mission_time_string)
        self.reset()  # 更新所有子模組、記錄檔標頭

        # 更改旗標
        self.STATE.FLAG_THREAD = True

        # 線程位址初始化
        sr_alt_seq = self.get_sequence(self.PARAM.value['FREQ_SR_ALT'])
        ctrl_pos_seq = self.get_sequence(self.PARAM.value['FREQ_CTRL_POS'])
        ctrl_hdg_seq = self.get_sequence(self.PARAM.value['FREQ_CTRL_HDG'])
        cmd_seq = self.get_sequence(self.PARAM.value['FREQ_CT'])
        sr_cam_seq = self.get_sequence(self.PARAM.value['FREQ_CAM'])

        PrintLayout.success('本次任務執行起始時間為 ' + self.STATE.get_mission_time_string)
        PrintLayout.success('線程已啟動')

        # 迴圈計時與計數
        self.timer.set_start()
        loop_count = 0

        # 背景執行迴圈
        while self.STATE.FLAG_STATE == 'EXECUTE':
            # 執行中旗標
            self.flag_thread_on = True

            # 註冊執行時間
            # 位置控制器
            for tick in ctrl_pos_seq:
                self.scheduler.enterabs(self.timer.start_time + loop_count + tick, 1, self.ctrl_pos.run)

            # 航向控制器
            for tick in ctrl_hdg_seq:
                self.scheduler.enterabs(self.timer.start_time + loop_count + tick, 1, self.ctrl_hdg.run)

            # 高度感測器
            if self.CMD.SW_THR_SR_ALT:
                for tick in sr_alt_seq:
                    self.scheduler.enterabs(self.timer.start_time + loop_count + tick, 1, self.sr_alt.run)

            # 影像辨識
            if self.CMD.SW_THR_SR_ALT:
                for tick in sr_cam_seq:
                    self.scheduler.enterabs(self.timer.start_time + loop_count + tick, 1, self.sr_cam.run)

            # 命令傳輸
            for tick in cmd_seq:
                self.scheduler.enterabs(self.timer.start_time + loop_count + tick, 1, self.cmd.run)

            # 執行迴圈
            self.scheduler.run()

            # 結束跳出
            if self.STATE.FLAG_STATE != 'EXECUTE':
                # 線程執行統計
                loop_count += 1
                loop_time = self.timer.elapsed_time()
                loop_time_delta = (loop_time - loop_count) * 1e6
                loop_time_delta_ave = loop_time_delta / loop_count

                # 關閉訊息
                PrintLayout.info('線程迴圈已關閉，執行圈數：%.3f圈，耗時 %.6f 秒' % (loop_count, loop_time))
                PrintLayout.info('             偏差時間 %6.3f us，圈平均偏差 %6.3f us' %
                                 (loop_time_delta, loop_time_delta_ave), False)
                break

            # 圈序計數
            loop_count += 1

        # 更改旗標
        self.STATE.FLAG_THREAD = False

        # 輸出參數檔
        self.save_log()
        PrintLayout.success('記錄檔已儲存，任務時間為 ' + self.STATE.get_mission_time_string)

    # 儲存紀錄檔
    def save_log(self):
        """儲存紀錄檔"""
        if self.CMD.SW_THR_SR_ALT:
            self.sr_alt.save_log()
            self.sr_cam.save_log()
        self.ctrl_pos.save_log()
        self.ctrl_hdg.save_log()
        self.cmd.save_log()

    # 啟動線程
    def start(self):
        """啟動線程"""
        self.thread_process = threading.Thread(target=self.processing)
        self.thread_process.start()

    # 等待線程結束
    def join(self):
        """等待線程結束"""
        self.thread_process.join()

    # 重置
    def reset(self):
        # 線程執行中檢查
        if self.STATE.FLAG_THREAD:
            PrintLayout.warning('線程運作中，重置禁止')
            return

        # 更改旗標
        self.STATE.FLAG_THREAD_RESET = True

        # 重置各元件
        if self.CMD.SW_THR_SR_ALT:
            self.sr_cam.reset()
        if self.CMD.SW_THR_SR_ALT:
            self.sr_alt.reset()
        self.ctrl_pos.reset()
        self.ctrl_hdg.reset()
        self.cmd.reset()

        # 更改旗標
        self.STATE.FLAG_THREAD_RESET = False

    # 關閉線程(儲存狀態時呼叫)
    def close(self):
        """關閉線程並顯示執行記錄"""
        if self.STATE.FLAG_STATE == 'CLOSE':
            if self.CMD.SW_THR_SR_ALT:
                self.sr_alt.close()
            self.ctrl_pos.close()
            self.ctrl_hdg.close()
            self.cmd.close()
            if self.CMD.SW_THR_SR_ALT:
                self.sr_cam.close()
            PrintLayout.info('線程關閉')

    # 設定辨識模式
    def set_camera_mode(self, mode, color):
        """設定辨識模式"""
        self.sr_cam.set_camera_mode(mode, color)

    # 更新螢幕顯示
    def show_screen_image(self):
        """更新螢幕顯示"""
        self.sr_cam.show_screen_image()

    # 關閉螢幕顯示
    def close_screen_image(self):
        """關閉螢幕顯示"""
        self.sr_cam.close_screen_image()

    # 產生執行時刻表
    @staticmethod
    def get_sequence(freq):
        sequence = list(range(0, 1000000, int(1000000/freq)))
        if 1000000 % freq:
            sequence.pop()
        for i in range(len(sequence)):
            sequence[i] = sequence[i]/1000000
        return sequence


# TDK25 飛行組 競賽程式碼 - 線程模組 - 位置控制器
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞

# 版本資訊
# 程式序號 TDK25-THREAD-CTRL-POS
# 版號 03
# 最後編輯日 2021/11/29

# 程式說明
# 位置控制器，採相對座標系控制(地面目標座標系NEU座標,飛機為NED,僅差垂直方向)
# 增設Z軸向控制，不重置
# 調整為整合型回授控制(轉速回授控制整入外控制器）

# 注意事項
# 僅供PX4使用


class ControllerPos:
    """位置控制器"""
    def __init__(self, obj_handler):
        # 取得外部統一變數
        self.OBJ_HANDLER = obj_handler
        self.STATE = obj_handler.STATE
        self.CMD = obj_handler.CMD
        self.PARAM = obj_handler.PARAM
        self.UAV = obj_handler.UAV

        self.flag_first_call = True

        # 初始化控制器
        self.ctrl_pos_x = Controller.PIdashDController()
        self.ctrl_pos_y = Controller.PIdashDController()
        self.ctrl_pos_z = Controller.PIdashDController()

        # 參數設定
        self.set_param()

        # 記錄器
        self.recorder = Tool.DataRecoder(self.STATE,
                                         ['Time', 'Xsp', 'Ysp', 'X', 'Y',
                                          'Rollsp', 'Pitchsp', 'Roll uav', 'Pitch uav', 'Roll', 'Pitch',
                                          'PT', 'LT', 'X Kp', 'X Ki', 'X Kd', 'Y Kp', 'Y Ki', 'Y Kd'],
                                         name='Position Controller')

        self.recorder_alt = Tool.DataRecoder(self.STATE,
                                             ['Time', 'Alt Tar', 'ALT Resp', 'Alt Vel Output',
                                              'Climb rate sp', 'PT', 'LT', 'Kp', 'Ki', 'Kd'],
                                             name='Altitude Controller')
        self.timer = Tool.Timer()
        self.pt = 0
        self.lt = 0
        self.loop_count = 0
        
        # Bound Flag
        self.left_out = False
        self.left_in = False
        self.right_out = False
        self.right_in = False
        self.up_out = False
        self.up_in = False
        self.down_out = False
        self.down_in = False

        # 回報位置控制器啟動
        self.STATE.FLAG_CTRL_POS = True
        PrintLayout.success('位置控制器，就緒')

    # 參數設定
    def set_param(self):
        """參數設定"""
        # 更新控制器X參數
        self.ctrl_pos_x.set_param(freq=self.PARAM.value['FREQ_CTRL_POS'],
                                  kp=self.PARAM.value['CTRL_POS_X_KP'],
                                  ki=self.PARAM.value['CTRL_POS_X_KI'],
                                  kd=self.PARAM.value['CTRL_POS_X_KD'],)

        self.ctrl_pos_x.set_input_filter_sn(self.PARAM.value['CTRL_POS_X_INPUT_F_SN'])
        self.ctrl_pos_x.set_dt_filter_sn(self.PARAM.value['CTRL_POS_X_DT_F_SN'])
        self.ctrl_pos_x.set_output_filter_sn(self.PARAM.value['CTRL_POS_X_OUTPUT_F_SN'])

        self.ctrl_pos_x.set_output_limit(self.PARAM.value['CTRL_POS_X_OUTPUT_LIMIT'])
        self.ctrl_pos_x.set_integral_limit(self.PARAM.value['CTRL_POS_X_INT_LIMIT'])

        self.ctrl_pos_x.set_input_deadzone(self.PARAM.value['CTRL_POS_X_INPUT_DZ'])
        self.ctrl_pos_x.set_feedback_deadzone(self.PARAM.value['CTRL_POS_X_FEEDBACK_DZ'])
        self.ctrl_pos_x.set_output_deadzone(self.PARAM.value['CTRL_POS_X_OUTPUT_DZ'])

        # 更新控制器Y參數
        self.ctrl_pos_y.set_param(freq=self.PARAM.value['FREQ_CTRL_POS'],
                                  kp=self.PARAM.value['CTRL_POS_Y_KP'],
                                  ki=self.PARAM.value['CTRL_POS_Y_KI'],
                                  kd=self.PARAM.value['CTRL_POS_Y_KD'],)

        self.ctrl_pos_y.set_input_filter_sn(self.PARAM.value['CTRL_POS_Y_INPUT_F_SN'])
        self.ctrl_pos_y.set_dt_filter_sn(self.PARAM.value['CTRL_POS_Y_DT_F_SN'])
        self.ctrl_pos_y.set_output_filter_sn(self.PARAM.value['CTRL_POS_Y_OUTPUT_F_SN'])

        self.ctrl_pos_y.set_output_limit(self.PARAM.value['CTRL_POS_Y_OUTPUT_LIMIT'])
        self.ctrl_pos_y.set_integral_limit(self.PARAM.value['CTRL_POS_Y_INT_LIMIT'])

        self.ctrl_pos_y.set_input_deadzone(self.PARAM.value['CTRL_POS_Y_INPUT_DZ'])
        self.ctrl_pos_y.set_feedback_deadzone(self.PARAM.value['CTRL_POS_Y_FEEDBACK_DZ'])
        self.ctrl_pos_y.set_output_deadzone(self.PARAM.value['CTRL_POS_Y_OUTPUT_DZ'])
        
        # 更新控制器Z參數
        self.ctrl_pos_z.set_param(freq=self.PARAM.value['FREQ_CTRL_POS'],
                                  kp=self.PARAM.value['CTRL_POS_Z_KP'],
                                  ki=self.PARAM.value['CTRL_POS_Z_KI'],
                                  kd=self.PARAM.value['CTRL_POS_Z_KD'],)

        self.ctrl_pos_z.set_input_filter_sn(self.PARAM.value['CTRL_POS_Z_INPUT_F_SN'])
        self.ctrl_pos_z.set_dt_filter_sn(self.PARAM.value['CTRL_POS_Z_DT_F_SN'])
        self.ctrl_pos_z.set_output_filter_sn(self.PARAM.value['CTRL_POS_Z_OUTPUT_F_SN'])

        self.ctrl_pos_z.set_output_limit(self.PARAM.value['CTRL_POS_Z_OUTPUT_LIMIT'])
        self.ctrl_pos_z.set_integral_limit(self.PARAM.value['CTRL_POS_Z_INT_LIMIT'])

        self.ctrl_pos_z.set_input_deadzone(self.PARAM.value['CTRL_POS_Z_INPUT_DZ'])
        self.ctrl_pos_z.set_feedback_deadzone(self.PARAM.value['CTRL_POS_Z_FEEDBACK_DZ'])
        self.ctrl_pos_z.set_output_deadzone(self.PARAM.value['CTRL_POS_Z_OUTPUT_DZ'])

    # 執行控制器(線程方法)
    def run(self):
        # 運算計時器
        pt_start = time.perf_counter()

        # 執行控制器取得姿態值(限制後速度=>上下界之間值)
        acc_x = self.ctrl_pos_x.processing(self.CMD.POS_X, self.STATE.POS_X)
        acc_y = self.ctrl_pos_y.processing(self.CMD.POS_Y, self.STATE.POS_Y)
        vel_z = self.ctrl_pos_z.processing(self.CMD.ALT_REL, self.STATE.ALT_REL)
        
        self.left_out = False
        self.right_out = False
        self.up_out = False
        self.down_out = False
        
        # 判斷式作法
        if self.STATE.POS_X > 0.4:
            acc_x = -0.3
            self.up_out = True
        elif self.STATE.POS_X < -0.4:
            acc_x = 0.3
            self.down_out = True
        else:
            if self.up_out and 0.2 < self.STATE.POS_X <= 0.4:
                acc_x = 0.5
            elif self.down_out and -0.4 <= self.STATE.POS_X < -0.2:
                acc_x = -0.5
            else:
                self.up_out = False
                self.up_in = False
                self.down_out = False
                self.down_in = False

        if self.STATE.POS_Y > 0.4:
            acc_y = -0.3
            self.right_out = True
        elif self.STATE.POS_Y < -0.4:
            acc_y = 0.3
            self.left_out = True
        else:
            if self.right_out and 0.2 < self.STATE.POS_Y <= 0.4:
                acc_y = 0.5
            elif self.left_out and -0.4 <= self.STATE.POS_Y < -0.2:
                acc_y = -0.5
            else:
                self.left_out = False
                self.left_in = False
                self.right_out = False
                self.right_in = False

        # 加速度到角度命令轉換
        pitch = -math.degrees(math.atan2(acc_x, 9.80665))  # 非循跡模式X由控制器控制
        roll = math.degrees(math.atan2(acc_y, 9.80665))
        
        # 角度輸出限制
        if pitch > self.PARAM.value['CTRL_POS_MAX_ANGLE']:
            pitch = self.PARAM.value['CTRL_POS_MAX_ANGLE']
        elif pitch < -self.PARAM.value['CTRL_POS_MAX_ANGLE']:
            pitch = -self.PARAM.value['CTRL_POS_MAX_ANGLE']
        if roll > self.PARAM.value['CTRL_POS_MAX_ANGLE']:
            roll = self.PARAM.value['CTRL_POS_MAX_ANGLE']
        elif roll < -self.PARAM.value['CTRL_POS_MAX_ANGLE']:
            roll = -self.PARAM.value['CTRL_POS_MAX_ANGLE']
        
        # 輸出命令(高度低於門檻時回復為0)
        if self.STATE.ALT_REL < self.PARAM.value['CTRL_POS_MIN_ALT']:
            self.CMD.ATT['ROLL'] = 0
            self.CMD.ATT['PITCH'] = 0
        else:
            if self.CMD.SW_TRACK_OP_FWD and self.STATE.FLAG_DETECT_MODE == 'tracking':
                self.CMD.ATT['ROLL'] = roll
            else:
                self.CMD.ATT['ROLL'] = roll
                self.CMD.ATT['PITCH'] = pitch

        # 輸出命令(推力)
        self.CMD.ALT_VEL = vel_z
        self.CMD.ATT['VZ_RATE'] = self.vz_normalize(self.CMD.ALT_VEL)

        # 抓取現在姿態
        self.STATE.UAV_ROLL = math.degrees(self.UAV.attitude.roll)
        self.STATE.UAV_PITCH = math.degrees(self.UAV.attitude.pitch)
        self.STATE.UAV_YAW = math.degrees(self.UAV.attitude.yaw)

        # 記錄耗時
        self.pt = time.perf_counter() - pt_start
        if self.flag_first_call:
            self.lt = 1 / self.PARAM.value['FREQ_CTRL_POS']
            self.timer.period()
            self.flag_first_call = False
        else:
            self.lt = self.timer.period()

        # 記錄
        self.loop_count += 1
        rec_row_data = [self.CMD.POS_X, self.CMD.POS_Y, self.STATE.POS_X, self.STATE.POS_Y,
                        self.CMD.ATT['ROLL'], self.CMD.ATT['PITCH'],
                        self.STATE.UAV_ROLL, self.STATE.UAV_PITCH, self.STATE.ATT_ROLL, self.STATE.ATT_PITCH,
                        self.pt, self.lt,
                        self.ctrl_pos_x.kp_output, self.ctrl_pos_x.ki_output,
                        self.ctrl_pos_x.kd_output, self.ctrl_pos_y.kp_output,
                        self.ctrl_pos_y.ki_output, self.ctrl_pos_y.kd_output]
        
        rec_row_data_alt = [self.CMD.ALT_REL, self.STATE.ALT_REL, self.CMD.ALT_VEL, self.CMD.ATT['VZ_RATE'],
                            self.pt, self.lt,
                            self.ctrl_pos_z.kp_output, self.ctrl_pos_z.ki_output, self.ctrl_pos_z.kd_output]

        self.recorder.rows_storage(rec_row_data)
        self.recorder_alt.rows_storage(rec_row_data_alt)

    # 重置感測器(任務開始時呼叫)
    def reset(self):
        self.set_param()
        self.ctrl_pos_x.reset()
        self.ctrl_pos_y.reset()
        self.ctrl_pos_z.reset()
        self.recorder.register()
        self.recorder_alt.register()
        self.flag_first_call = True
        self.loop_count = 0

        # 訊息提示
        PrintLayout.info('位置控制器，已重置')

    # 儲存記錄檔(任務結束時呼叫)
    def save_log(self):
        """儲存記錄檔"""
        self.recorder.write_log()
        self.recorder_alt.write_log()

        # 訊息提示
        PrintLayout.info('位置控制器，記錄檔已儲存')

    # 關閉物件(儲存狀態時呼叫)
    def close(self):
        # 位置控制器關閉
        self.STATE.FLAG_CTRL_POS = False

        # 訊息提示
        PrintLayout.info('位置控制器，已關閉')

    # 垂直速度命令正規化 For Ardupilot
    def vz_normalize(self, vz):
        if self.CMD.SW_LAND and self.STATE.ALT_REL < self.PARAM.value['CTRL_POS_Z_DESCEND_ALT']:
            vz_rate = 0  # 怠速模式，降落+離地閥值內觸發
        elif vz >= self.PARAM.value['WPNAV_SPEED_UP']:
            vz_rate = 1
        elif 0 < vz < self.PARAM.value['WPNAV_SPEED_UP']:
            vz_rate = 0.5 + 0.5 * (vz / self.PARAM.value['WPNAV_SPEED_UP'])
        elif 0 > vz > -self.PARAM.value['WPNAV_SPEED_DN']:
            vz_rate = 0.5 + 0.5 * (vz / self.PARAM.value['WPNAV_SPEED_DN'])
        elif vz < -self.PARAM.value['WPNAV_SPEED_DN']:
            vz_rate = 0
        else:
            vz_rate = 0.5
        return vz_rate


# TDK25 飛行組 競賽程式碼 - 線程模組 - 航向控制器
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞

# 版本資訊
# 程式序號 TDK25-THREAD-CTRL-HDG
# 版號 01
# 最後編輯日 2021/07/15

# 程式說明
# 航向控制器，於循跡時作用控制旋轉速率

# 注意事項


class ControllerHdg:
    """航向控制器"""
    def __init__(self, obj_handler):
        # 取得外部統一變數
        self.OBJ_HANDLER = obj_handler
        self.STATE = obj_handler.STATE
        self.CMD = obj_handler.CMD
        self.PARAM = obj_handler.PARAM
        self.UAV = obj_handler.UAV

        self.flag_first_call = True
        self.lock = threading.Lock()

        # 初始化控制器
        self.ctrl_hdg = Controller.PIdashDController()

        # 參數設定
        self.set_param()

        # 記錄器
        self.recorder = Tool.DataRecoder(self.STATE,
                                         ['Time', 'ANG_Tar', 'ANG_Resp', 'HDG', 'Yaw_Rate_Output', 'PT', 'LT', 'Kp',
                                          'Ki', 'Kd'],
                                         name='Heading Controller')
        self.timer = Tool.Timer()
        self.pt = 0
        self.lt = 0
        self.loop_count = 0

        # 回報高度控制器啟動
        self.STATE.FLAG_CTRL_HDG = True
        PrintLayout.success('航向控制器，就緒')

    # 參數設定
    def set_param(self):
        """參數設定"""
        self.ctrl_hdg.set_param(freq=self.PARAM.value['FREQ_CTRL_HDG'],
                                kp=self.PARAM.value['CTRL_HDG_KP'],
                                ki=self.PARAM.value['CTRL_HDG_KI'],
                                kd=self.PARAM.value['CTRL_HDG_KD'],)

        self.ctrl_hdg.set_input_filter_sn(self.PARAM.value['CTRL_HDG_INPUT_F_SN'])
        self.ctrl_hdg.set_dt_filter_sn(self.PARAM.value['CTRL_HDG_DT_F_SN'])
        self.ctrl_hdg.set_output_filter_sn(self.PARAM.value['CTRL_HDG_OUTPUT_F_SN'])

        self.ctrl_hdg.set_output_limit(self.PARAM.value['CTRL_HDG_OUTPUT_LIMIT'])
        self.ctrl_hdg.set_integral_limit(self.PARAM.value['CTRL_HDG_INT_LIMIT'])

        self.ctrl_hdg.set_input_deadzone(self.PARAM.value['CTRL_HDG_INPUT_DZ'])
        self.ctrl_hdg.set_feedback_deadzone(self.PARAM.value['CTRL_HDG_FEEDBACK_DZ'])
        self.ctrl_hdg.set_output_deadzone(self.PARAM.value['CTRL_HDG_OUTPUT_DZ'])

    # 執行控制器(線程方法)
    def run(self):
        # 運算計時器
        pt_start = time.perf_counter()

        # 執行控制器取得角速度值(限制後速度=>上下界之間值)
        yaw_rate = self.ctrl_hdg.processing(self.CMD.ANG, self.STATE.ANG)

        # 輸出命令
        if self.STATE.FLAG_DETECT_MODE != 'position':
            self.CMD.ATT['YAW_RATE'] = yaw_rate
        else:
            self.CMD.ATT['YAW_RATE'] = 0

        # 更新狀態
        self.STATE.HDG = self.UAV.heading

        # 記錄耗時
        self.pt = time.perf_counter() - pt_start
        if self.flag_first_call:
            self.lt = 1 / self.PARAM.value['FREQ_CTRL_HDG']
            self.timer.period()
            self.flag_first_call = False
        else:
            self.lt = self.timer.period()

        # 記錄
        self.loop_count += 1
        rec_row_data = [self.CMD.ANG, self.STATE.ANG, self.STATE.HDG,
                        self.CMD.ATT['YAW_RATE'], self.pt, self.lt,
                        self.ctrl_hdg.kp_output, self.ctrl_hdg.ki_output, self.ctrl_hdg.kd_output]
        self.recorder.rows_storage(rec_row_data)

    # 重置控制器(任務開始時呼叫)
    def reset(self):
        self.set_param()

        self.ctrl_hdg.reset()
        self.recorder.register()
        self.flag_first_call = True
        self.loop_count = 0

        # 訊息提示
        PrintLayout.info('航向控制器，已重置')

    # 儲存記錄檔(任務結束時呼叫)
    def save_log(self):
        """儲存記錄檔"""
        self.recorder.write_log()

        # 訊息提示
        PrintLayout.info('航向控制器，記錄檔已儲存')

    # 關閉物件(儲存狀態時呼叫)
    def close(self):
        # 回報高度控制器關閉
        self.STATE.FLAG_CTRL_HDG = False

        # 訊息提示
        PrintLayout.info('航向控制器，已關閉')


# TDK25 飛行組 競賽程式碼 - 線程模組 - 命令傳送模組
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞

# 版本資訊
# 程式序號 TDK25-THREAD-CMD-TRANSMIT
# 版號 01
# 最後編輯日 2021/05/24

# 程式說明
#

# 注意事項


class CommandTransmit:
    def __init__(self, obj_handler):
        # 取得外部統一變數
        self.OBJ_HANDLER = obj_handler
        self.STATE = obj_handler.STATE
        self.CMD = obj_handler.CMD
        self.PARAM = obj_handler.PARAM
        self.UAV = obj_handler.UAV

        self.flag_first_call = True
        self.lock = threading.Lock()

        # 記錄器
        self.recorder = Tool.DataRecoder(self.STATE,
                                         ['Time', 'Roll', 'Pitch', 'Yaw_Rate', 'Vz_Rate', 'TT', 'LT'],
                                         name='Command Transmit')
        self.timer = Tool.Timer()
        self.tt = 0
        self.lt = 0
        self.loop_count = 0

        # 回報命令傳輸程式啟動
        self.STATE.FLAG_CT = True
        PrintLayout.success('命令傳送模組，就緒')

    # 命令編碼與傳送(線程方法)
    def run(self):
        # 計時器
        tt_start = time.perf_counter()

        # 維持YAW恆定(循線時測試效果)
        self.CMD.ATT['YAW'] = math.degrees(self.UAV.attitude.yaw)

        # 命令編碼，使用偏航角速度控制(餘採姿態控制)
        msg = self.UAV.message_factory.set_attitude_target_encode(
            0,  # 開機時間
            0,  # 目標系統
            0,  # 目標裝置
            0b00000011,  # 命令遮罩(1忽略，0啟用)
            self.to_quaternion(self.CMD.ATT['ROLL'], self.CMD.ATT['PITCH'],
                               self.CMD.ATT['YAW']),  # 姿態四元數
            0,  # Roll滾轉角速度(radian/s)
            0,  # Pitch滾轉角速度(radian/s)
            math.radians(self.CMD.ATT['YAW_RATE']),  # Yaw滾轉角速度(radian/s)
            self.CMD.ATT['VZ_RATE'])  # 集合推力(0~1)

        # 傳輸命令
        self.UAV.send_mavlink(msg)

        # 記錄耗時
        self.tt = time.perf_counter() - tt_start
        if self.flag_first_call:
            self.lt = 1 / self.PARAM.value['FREQ_CT']
            self.timer.period()
            self.flag_first_call = False
        else:
            self.lt = self.timer.period()

        # 記錄
        self.loop_count += 1
        rec_row_data = [self.CMD.ATT['ROLL'], self.CMD.ATT['PITCH'], self.CMD.ATT['YAW_RATE'],
                        self.CMD.ATT['VZ_RATE'], self.tt, self.lt]
        self.recorder.rows_storage(rec_row_data)

    # 轉換尤拉角到四元數
    # 因命令格式針對姿態需使用四元數格式，因此需先把尤拉角作轉換，此格式可避免萬向節鎖產生
    # 輸入格式為度(degree)，輸出為四元數(q,x,y,z)
    @staticmethod
    def to_quaternion(roll=0.0, pitch=0.0, yaw=0.0):
        """輸入尤拉角轉換成四元數"""
        t0 = math.cos(math.radians(yaw * 0.5))
        t1 = math.sin(math.radians(yaw * 0.5))
        t2 = math.cos(math.radians(roll * 0.5))
        t3 = math.sin(math.radians(roll * 0.5))
        t4 = math.cos(math.radians(pitch * 0.5))
        t5 = math.sin(math.radians(pitch * 0.5))

        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5

        return [w, x, y, z]

    # 重置傳輸記錄(任務開始時呼叫)
    def reset(self):
        self.recorder.register()
        self.flag_first_call = True
        self.loop_count = 0

        # 訊息提示
        PrintLayout.info('命令傳輸程式，已重置')

    # 儲存記錄檔(任務結束時呼叫)
    def save_log(self):
        """儲存記錄檔"""
        self.recorder.write_log()

        # 訊息提示
        PrintLayout.info('命令傳輸程式，記錄檔已儲存')

    # 關閉物件(儲存狀態時呼叫)
    def close(self):
        # 回報命令傳輸程式關閉
        self.STATE.FLAG_CT = False

        # 訊息提示
        PrintLayout.info('命令傳輸程式，已關閉')


# TDK25 飛行組 競賽程式碼 - 線程模組 - 高度感測器
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞

# 版本資訊
# 程式序號 TDK25-THREAD-SR-ALT
# 版號 01
# 最後編輯日 2021/05/23

# 程式說明
#

# 注意事項


class SensorAlt:
    """高度感測器"""
    def __init__(self, obj_handler):
        # 取得外部統一變數
        self.OBJ_HANDLER = obj_handler
        self.STATE = obj_handler.STATE
        self.CMD = obj_handler.CMD
        self.PARAM = obj_handler.PARAM
        self.UAV = obj_handler.UAV

        self.flag_first_call = True
        self.lock = threading.Lock()

        self.pre_alt_rel = 0

        # 光達測距儀初始化
        self.lidar = Sensor.SensorAltLidar(self.OBJ_HANDLER)

        # 姿態感測器初始化
        self.imu = Driver.BNO055()
        if self.imu.begin() is not True:
            print("BNO055 Error initializing device")
            imu_result = False
        else:
            imu_result = True
        
        # 濾波器初始化
        self.vel_z_maf = Tool.MovingAverageFilter(self.PARAM.value['SENS_ALT_VZ_F_SN'])
        self.roll_maf = Tool.MovingAverageFilter(self.PARAM.value['SENS_ATT_ROLL_MAF_SN'])
        self.pitch_maf = Tool.MovingAverageFilter(self.PARAM.value['SENS_ATT_PITCH_MAF_SN'])

        # 記錄器
        self.recorder = Tool.DataRecoder(self.STATE, ['Time', 'Alt Rel', 'Alt Abs', 'Alt Vel Rel', 'Alt Vel Abs',
                                                      'MT', 'LT'],
                                         name='Altitude Sensor')
        self.timer = Tool.Timer()
        self.timer2 = Tool.Timer()
        self.mt = 0
        self.lt = 0
        self.loop_count = 0

        # 回報高度感測器啟動
        if self.lidar.enable() and imu_result:
            self.STATE.FLAG_SR_ALT = True
            PrintLayout.success('高度感測器，就緒')
        else:
            self.STATE.FLAG_SR_ALT = False
            PrintLayout.error('高度感測器，初始化失敗')

    # 量測高度(線程方法)
    def run(self):
        # 量測計時器
        mt_start = time.perf_counter()

        # 姿態感測器讀值與處理
        yaw, roll, pitch = list(self.imu.getVector(self.imu.VECTOR_EULER))
        roll += self.PARAM.value['SENS_ATT_ROLL_OFFSET']
        pitch += self.PARAM.value['SENS_ATT_PITCH_OFFSET']
        if math.fabs(roll) > 90:
            roll = self.STATE.ATT_ROLL
        if math.fabs(pitch) > 90:
            pitch = self.STATE.ATT_PITCH
        self.STATE.ATT_ROLL = self.roll_maf.processing(roll)
        self.STATE.ATT_PITCH = self.pitch_maf.processing(pitch)

        # 讀取光達測距儀資料
        self.ALT_LIDAR = self.lidar.measure()

        # 更新相對高度值
        self.STATE.ALT_REL = self.ALT_LIDAR

        # 垂直速度計算
        delta_alt_rel = self.STATE.ALT_REL - self.pre_alt_rel
        if self.flag_first_call or self.PARAM.value['SENS_ALT_DT_FLAG']:
            delta_time = 1 / self.PARAM.value['FREQ_SR_ALT']
            self.timer2.period()
        else:
            delta_time = self.timer2.period()
        alt_vel_raw = delta_alt_rel / delta_time
        self.STATE.ALT_VEL_REL = self.vel_z_maf.processing(alt_vel_raw)

        # 數值記錄
        self.mt = time.perf_counter() - mt_start
        if self.flag_first_call:
            self.lt = 1 / self.PARAM.value['FREQ_SR_ALT']
            self.timer.period()
            self.flag_first_call = False
        else:
            self.lt = self.timer.period()

        # 記錄
        self.loop_count += 1
        rec_row_data = [self.STATE.ALT_REL, self.STATE.ALT_ABS,
                        self.STATE.ALT_VEL_REL, self.STATE.ALT_VEL_ABS, self.mt, self.lt]
        self.recorder.rows_storage(rec_row_data)

        # 傳遞值
        self.pre_alt_rel = self.STATE.ALT_REL

    # 重置感測器(任務開始時呼叫)
    def reset(self):
        self.lidar.update_and_apply_param()
        self.recorder.register()
        self.reset_filter()

        self.flag_first_call = True
        self.loop_count = 0

        # 訊息提示
        PrintLayout.info('高度感測器，已重置')

    # 重置濾波器
    def reset_filter(self):
        """重置濾波器"""
        self.vel_z_maf.sample_number = self.PARAM.value['SENS_ALT_VZ_F_SN']
        self.vel_z_maf.reset()
        self.roll_maf.sample_number = self.PARAM.value['SENS_ATT_ROLL_MAF_SN']
        self.roll_maf.reset()
        self.pitch_maf.sample_number = self.PARAM.value['SENS_ATT_PITCH_MAF_SN']
        self.pitch_maf.reset()

    # 儲存記錄檔(任務結束時呼叫)
    def save_log(self):
        """儲存記錄檔"""
        self.lidar.recorder.write_log()
        self.recorder.write_log()

        # 訊息提示
        PrintLayout.info('高度感測器，記錄檔已儲存')

    # 關閉物件(程式結束時呼叫)
    def close(self):
        # 關閉光達連線
        self.lidar.close()

        # 回報高度感測器關閉
        self.STATE.FLAG_SR_ALT = False

        # 訊息提示
        PrintLayout.info('高度感測器，已關閉')


# TDK25 飛行組 競賽程式碼 - 線程模組 - 影像辨識模組
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞

# 版本資訊
# 程式序號 TDK25-THREAD-CAMERA-Sensor
# 版號 01
# 最後編輯日 2021/07/12

# 程式說明
# 影像辨識線程介面，一般迴圈處理影像數據截取，模式播換、條件擷取

# 注意事項

class SensorCam:
    def __init__(self, obj_handler):
        # 取得外部統一變數
        self.OBJ_HANDLER = obj_handler
        self.STATE = obj_handler.STATE
        self.CMD = obj_handler.CMD
        self.PARAM = obj_handler.PARAM
        self.UAV = obj_handler.UAV

        self.flag_first_call = True
        self.flag_filter_reset = False
        self.lock = threading.Lock()

        # 初始化相機
        self.cam = Camera.Camera(self.STATE, self.PARAM)

        # 初始化濾波器
        self.pos_x_maf = Tool.MovingAverageFilter(self.PARAM.value['SENS_CAM_X_F_SN'])
        self.pos_y_maf = Tool.MovingAverageFilter(self.PARAM.value['SENS_CAM_Y_F_SN'])
        self.vel_x_maf = Tool.MovingAverageFilter(self.PARAM.value['SENS_CAM_VX_F_SN'])
        self.vel_y_maf = Tool.MovingAverageFilter(self.PARAM.value['SENS_CAM_VY_F_SN'])
        self.ang_maf = Tool.MovingAverageFilter(self.PARAM.value['SENS_CAM_ANG_F_SN'])
        self.area_maf = Tool.MovingAverageFilter(self.PARAM.value['SENS_CAM_AREA_F_SN'])
        self.dt_maf = Tool.MovingAverageFilter(self.PARAM.value['SENS_CAM_X_F_SN'])

        # 前一筆位置
        self.pre_pos_x = 0
        self.pre_pos_y = 0

        # 記錄器
        self.loop_count = 0
        self.recorder = Tool.DataRecoder(self.STATE,
                                         ['Time', 'Mode', 'Color', 'X', 'Y',
                                          'Vx', 'Vy', 'Angle', 'Area', 'MT', 'LT', 'Line Area',
                                          'Raw_X', 'Raw_Y', 'Pitch', 'Roll'],
                                         name='Camera Sensor')
        self.timer = Tool.Timer()
        self.timer2 = Tool.Timer()
        self.mt = 0
        self.lt = 0
        self.loop_count = 0

        # 回報影像感測程式啟動
        if self.STATE.FLAG_CAM:
            self.STATE.FLAG_SR_CAM = True
            PrintLayout.success('影像辨識模組，就緒')
        else:
            PrintLayout.error('影像辨識模組-相機啟動錯誤')

    # 影像量測計算(線程方法)
    def run(self):
        # 量測計時器
        mt_start = time.perf_counter()

        # 重置條件
        if self.flag_filter_reset:
            self.reset_filter()
            self.flag_filter_reset = False

        # 取得影像資訊(轉成global座標-色塊中心，加上角度補償)
        pitch = self.STATE.ATT_PITCH
        roll = self.STATE.ATT_ROLL
        pos_x_raw = -self.cam.pos_x
        pos_y_raw = -self.cam.pos_y
        angle_raw = self.cam.angle
        area_raw = self.cam.area
        current_mode = self.cam.detect_mode[0]
        current_color = self.cam.detect_mode[1]
        
        if not pos_x_raw and self.pre_pos_x:
            pos_x_raw = self.pre_pos_x
            outbound_x = True
        else:
            outbound_x = False
        if not pos_y_raw and self.pre_pos_y:
            pos_y_raw = self.pre_pos_y
            outbound_y = True
        else:
            outbound_y = False

        # 更新時鐘
        if self.flag_first_call:
            delta_time = 1 / self.PARAM.value['FREQ_CAM']
            self.timer2.period()
        else:
            delta_time = self.timer2.period()
            
        dt = self.dt_maf.processing(delta_time)

        # 濾波處理
        pos_x = self.pos_x_maf.processing(pos_x_raw)
        pos_y = self.pos_y_maf.processing(pos_y_raw)
        area = self.area_maf.processing(area_raw)
        angle = self.ang_maf.processing(angle_raw)

        # 計算變化率(速度)
        vel_x_raw = (pos_x - self.pre_pos_x) / dt
        vel_y_raw = (pos_y - self.pre_pos_y) / dt
        
        # 速度出界狀態值保留
        if outbound_x:
            vel_x_raw = 0
        if outbound_y:
            vel_y_raw = 0

        # 濾波處理
        vel_x = self.vel_x_maf.processing(vel_x_raw)
        vel_y = self.vel_y_maf.processing(vel_y_raw)

        # 上傳數值
        if current_mode == 'tracking' or current_mode == 'take_off':
            self.STATE.POS_Y = pos_y
            self.STATE.VEL_Y = vel_y
            self.STATE.AREA = area
            self.STATE.ANG = angle
        else:
            self.STATE.POS_X = pos_x
            self.STATE.VEL_X = vel_x
            self.STATE.POS_Y = pos_y
            self.STATE.VEL_Y = vel_y
            self.STATE.AREA = area
            self.STATE.ANG = 0
        self.STATE.FLAG_DETECT_MODE = current_mode
        self.STATE.FLAG_DETECT_COLOR = current_color

        # 傳遞值
        self.pre_pos_x = pos_x
        self.pre_pos_y = pos_y

        # 數值記錄
        self.mt = time.perf_counter() - mt_start
        if self.flag_first_call:
            self.lt = 1 / self.PARAM.value['FREQ_CAM']
            self.timer.period()
            self.flag_first_call = False
        else:
            self.lt = self.timer.period()

        # 記錄
        self.loop_count += 1
        rec_row_data = [current_mode, current_color, self.STATE.POS_X, self.STATE.POS_Y,
                        self.STATE.VEL_X, self.STATE.VEL_Y,
                        self.STATE.ANG, self.STATE.AREA, self.mt, self.lt, self.cam.line_area_raw,
                        -self.cam.pos_x, -self.cam.pos_y, pitch, roll]
        self.recorder.rows_storage(rec_row_data)

    # 設定辨識模式
    def set_camera_mode(self, mode, color):
        """設定辨識模式"""
        self.cam.detect_mode = (mode, color)
        self.flag_filter_reset = True       # 以旗標方式執行，防止執行中調撥
        PrintLayout.fly_result('相機偵測模式改為 %s 模式，目標顏色 %s' % (mode, color))

    # 顯示現在辨識畫面
    def show_screen_image(self):
        try:
            self.cam.show_screen_image()
        except AttributeError:
            pass

    # 關閉現在辨識畫面
    def close_screen_image(self):
        try:
            self.cam.close_screen_image()
        except AttributeError:
            pass

    # 重置濾波器
    def reset_filter(self):
        """重置濾波器"""
        self.update_filter_sn()
        self.pos_x_maf.reset()
        self.pos_y_maf.reset()
        self.vel_x_maf.reset()
        self.vel_y_maf.reset()
        self.ang_maf.reset()
        self.area_maf.reset()

    # 更新濾波器參數
    def update_filter_sn(self):
        """更新濾波器參數"""
        self.pos_x_maf.sample_number = self.PARAM.value['SENS_CAM_X_F_SN']
        self.pos_y_maf.sample_number = self.PARAM.value['SENS_CAM_Y_F_SN']
        self.vel_x_maf.sample_number = self.PARAM.value['SENS_CAM_VX_F_SN']
        self.vel_y_maf.sample_number = self.PARAM.value['SENS_CAM_VY_F_SN']
        self.ang_maf.sample_number = self.PARAM.value['SENS_CAM_ANG_F_SN']
        self.area_maf.sample_number = self.PARAM.value['SENS_CAM_AREA_F_SN']

    # 重置相機
    def reset(self):
        self.recorder.register()
        self.reset_filter()
        self.cam.close()
        del self.cam
        self.cam = Camera.Camera(self.STATE, self.PARAM)
        self.pre_pos_x = 0
        self.pre_pos_y = 0

        self.flag_first_call = True

        # 訊息提示
        PrintLayout.info('相機感測程式，已重置')

    # 儲存記錄檔(任務結束時呼叫)
    def save_log(self):
        """儲存記錄檔"""
        self.recorder.write_log()

        # 訊息提示
        PrintLayout.info('相機感測程式，記錄檔已儲存')

    # 關閉物件(儲存狀態時呼叫)
    def close(self):
        # 相機關閉程式
        if self.STATE.FLAG_CAM:
            self.cam.close()

        # 回報相機感測程式關閉
        self.STATE.FLAG_SR_CAM = False

        # 訊息提示
        PrintLayout.info('相機感測程式，已關閉')


# 物件位址傳遞模組
class ObjectHandler:
    # 物件位址連結
    def __init__(self, state, cmd, param, param_io, uav):
        self.STATE = state
        self.CMD = cmd
        self.PARAM = param
        self.PARAM_IO = param_io
        self.UAV = uav


# 測試程式碼
# 標準呼叫程序
# Step.1 建立物件
# Step.2 線程初始化
# Step.
if __name__ == '__main__':
    # 物件建立
    timer_tick = Tool.Timer()
    PARAM = Parameter.Param()
    PARAM_IO = Parameter.ParamIO()
    STATE = Variable.StateVariable()
    CMD = Variable.Command()

    # 參數接入->參數集
    PARAM_IO.read_param_from_csv(show_param=False)
    PARAM.update(PARAM_IO.export())

    # 模擬環境設定
    if platform.platform() == 'macOS-10.16-x86_64-i386-64bit':
        flag_real_fc = False
        flag_real_alt = False
    else:           # 樹莓派 'Linux-5.10.17-v7l+-armv7l-with-debian-10.10'
        flag_real_fc = True
        flag_real_alt = True

    # 高度感測器
    CMD.SW_THR_SR_ALT = flag_real_alt

    # 控制器連線
    if flag_real_fc:
        pass
    else:
        sitl = dronekit_sitl.start_default()
        PARAM.value['CONN_UAV_PORT'] = sitl.connection_string()
    UAV = dronekit.connect(PARAM.value['CONN_UAV_PORT'], baud=PARAM.value['CONN_UAV_BAUD'],
                           rate=int(PARAM.value['CONN_UAV_RATE']), wait_ready=True)

    print('UAV Connected')

    # 物件位址傳遞模組
    obj_hand = ObjectHandler(STATE, CMD, PARAM, PARAM_IO, UAV)

    # 線程初始化
    BG_THR = ThreadScheduler(obj_hand)

    # 切換旗標
    STATE.FLAG_STATE = 'EXECUTE'
    if platform.platform() == 'macOS-10.16-x86_64-i386-64bit':
        STATE.ALT_REL = 0.7

    # 執行背景線程迴圈
    BG_THR.start()

    # 更改影像設定
    # BG_THR.set_camera_mode('position', 'red_light')

    # 等待時長
    ms_time = 20
    ms_timer = Tool.Timer()
    for n in range(ms_time * 100):
        ms_timer.wait_clock_arrive_ms((n + 1) * 10)

        if not n % 100:
            print('經過', n / 100 + 1, '秒')

    # 切換旗標
    STATE.FLAG_STATE = 'CLOSE'
    BG_THR.join()

    BG_THR.close()
