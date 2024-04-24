# TDK25 飛行組 競賽程式碼 - 監視模組
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞

# 版本資訊
# 程式序號 TDK25-MONITOR
# 版號 01
# 最後編輯日 2021/07/05

# 程式說明
# 顯示目前飛行狀態
#

# 注意事項

# 模組匯入
import threading

# 副程式匯入
from TDK25.Tool import PrintLayout
import TDK25.Tool as Tool


class Monitor:
    def __init__(self, obj_handler, bg_thread):
        # 取得外部統一變數
        self.OBJ_HANDLER = obj_handler
        self.STATE = obj_handler.STATE
        self.CMD = obj_handler.CMD
        self.PARAM = obj_handler.PARAM
        self.UAV = obj_handler.UAV
        self.BG_THR = bg_thread

        self.lock = threading.Lock()

        # 建立計時器
        self.timer = Tool.Timer()

    def show(self):
        """背景循環線程"""
        # 線程就緒檢查
        if not self.STATE.FLAG_THREAD_READY:
            return

        # 等待線程就緒
        if not self.STATE.FLAG_THREAD:
            pass

        # 執行迴圈
        i = 0
        self.timer.set_start()

        while self.state == 'EXECUTE':
            # self.lock.acquire()
            
            if self.STATE.FLAG_MONITOR:
                self.show_alt_ctrl()
                self.show_camera_result()

                #self.show_pos_ctrl()
                #self.show_vel_ctrl()
                PrintLayout.info('\n', False)
            else:
                pass

            # self.lock.release()

            # 更新頻率控制
            i += 1
            self.timer.wait_clock_arrive_ms(i * (1000 / self.PARAM.value['FREQ_MONIT']))

        PrintLayout.info('監控介面關閉')

    def show_alt_ctrl(self):
        PrintLayout.info('高度控制器 -> R@ %.2f m  Y@ %.2f m  U@ %.2f m/s' %
                         (self.CMD.ALT_REL, self.STATE.ALT_REL, self.CMD.ALT_VEL))
        PrintLayout.info('高度速度控制器 -> R@ %.2f m/s  Y@ %.2f m/s  U@ %.2f %%' %
                         (self.CMD.ALT_VEL, self.STATE.ALT_VEL_REL, self.CMD.ATT['VZ_RATE'] * 100))

    def show_pos_ctrl(self):
        PrintLayout.info('位置控制器 -> R@ (%.2f, %.2f) m  Y@ (%.2f, %.2f) m  U@ (%.2f, %.2f) m/s' %
                         (self.CMD.POS_X, self.CMD.POS_Y, self.STATE.POS_X, self.STATE.POS_Y,
                          self.CMD.VEL_X, self.CMD.VEL_Y))

    def show_vel_ctrl(self):
        PrintLayout.info('速度控制器 -> R@ (%.2f, %.2f) m/s  Y@ (%.2f, %.2f) m/s\n'
                         '                     R_U@ (%.2f, %.2f) deg  Y_U@ (%.2f, %.2f) deg' %
                         (self.CMD.VEL_X, self.CMD.VEL_Y, self.STATE.VEL_X, self.STATE.VEL_Y,
                          self.CMD.ATT['ROLL'], self.CMD.ATT['PITCH'], self.STATE.ATT_ROLL, self.STATE.ATT_PITCH))

    def show_camera_result(self):
        if self.STATE.FLAG_DETECT_MODE == 'position':
            PrintLayout.info('影像辨識 -> (X,Y)位置：(%.2f,%.2f) m, (X,Y)速度：(%.2f,%.2f) m/s, 色塊面積 %.2f m^2' %
                             (self.STATE.POS_X, self.STATE.POS_Y, self.STATE.VEL_X, self.STATE.VEL_Y, self.STATE.AREA))

        elif self.STATE.FLAG_DETECT_MODE == 'tracking':
            PrintLayout.info('影像辨識 -> Y位置：%.2f m, Y速度：%.2f m/s, 循跡角度 %.2f deg, 色塊面積 %.2f m^2' %
                             (self.STATE.POS_Y, self.STATE.VEL_Y, self.STATE.ANG, self.STATE.AREA))
        else:
            pass

    # 狀態標籤-讀取
    @property
    def state(self):
        """回傳程式狀態"""
        return self.STATE.FLAG_STATE

    # 啟動線程
    def start(self):
        """啟動線程"""
        self.thread_process = threading.Thread(target=self.show)
        self.thread_process.start()
        PrintLayout.success('開始監控狀態，更新率%dHz' % self.PARAM.value['FREQ_MONIT'])

    # 等待線程結束
    def join(self):
        """等待線程結束"""
        self.thread_process.join()
