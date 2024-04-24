# TDK25 飛行組 競賽程式碼 - 模組主程式
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞、羅宇彥、徐慧哲

# 版本資訊
# 程式序號 TDK25-MAIN
# 版號 02
# 最後編輯日 2021/07/16

# 程式說明
# 整合所有程式碼，以類別撰寫狀態機架構，於外程式呼叫建立後，僅需執行run()即可

# 副程式匯入
import TDK25.ThreadScheduler as ThreadScheduler
import TDK25.MissionScheduler as MissionScheduler
import TDK25.Monitor as Monitor
import TDK25.Parameter as Parameter
import TDK25.Variable as Variable
import TDK25.Tool as Tool
from TDK25.Tool import PrintLayout


# 模組匯入
import threading
import dronekit
import dronekit_sitl


# 無人機狀態機模組
class TDK25UAV:
    """無人機執行API"""
    # 物件初始化
    def __init__(self, simulation=False):
        # 初始化物件
        self.STATE = Variable.StateVariable()
        self.CMD = Variable.Command()
        self.PARAM = Parameter.Param()
        self.PARAM_IO = Parameter.ParamIO()
        self.PARAM.update(self.PARAM_IO.export())

        self.simulation = simulation

        # 連線程序
        self.conn_result = self.connect(simulation=simulation)

        # 系統啟動
        if self.conn_result:
            # 建立背景狀態監聽器
            self.start_listener()

            # 送入物件位址傳遞模組
            obj_handler = ObjectHandler(self.STATE, self.CMD, self.PARAM, self.PARAM_IO, self.UAV)

            # 建立背景程序物件
            self.BG_THR = ThreadScheduler.ThreadScheduler(obj_handler)

            # 建立監控介面
            self.BG_MONIT = Monitor.Monitor(obj_handler, self.BG_THR)

            # 建立拾物機構
            self.PICKER = Tool.Picker()
            if not self.PICKER.enable():
                PrintLayout.error('Picker enable failed')
                self.conn_result = False

            # 開啟任務管理器(送入背景程序物件)
            self.MS = MissionScheduler.MissionScheduler(obj_handler, self.BG_THR, self.PICKER, simulation=simulation)

            # 建立感測參數調整物件
            self.SEN_TRIM = Tool.SensorTrim(obj_handler)

            # 前往選擇介面
            self.state = 'PANEL'

        # 連線失敗跳過
        else:
            pass

    # 無人機連線
    def connect(self, simulation=False):
        """無人機連線"""
        conn_count = 1
        while conn_count <= self.PARAM.value['CONN_UAV_ERR_LIMIT']:
            self.connect_success = None
            # 載入連線計時器
            uav_connect_count = threading.Thread(target=self.connect_countdown)
            uav_connect_count.start()

            # 連線執行程式
            if simulation:
                self.sitl = dronekit_sitl.start_default()
                self.PARAM.value['CONN_UAV_PORT'] = self.sitl.connection_string()
                self.CMD.SW_THR_SR_ALT = False

            self.UAV = dronekit.connect(self.PARAM.value['CONN_UAV_PORT'],
                                        baud=self.PARAM.value['CONN_UAV_BAUD'],
                                        rate=int(self.PARAM.value['CONN_UAV_RATE']))

            # 客製化wait_ready函式
            self.connect_success = self.UAV.wait_ready('armed', 'mode', 'attitude',
                                                       types=True, timeout=self.PARAM.value['CONN_UAV_TIMEOUT'],
                                                       raise_exception=False)

            # 等待計時器關閉
            uav_connect_count.join()
            # print(self.connect_success)

            # 連線成功
            if self.connect_success:
                return True
            else:
                # 多次連線失敗，關閉程式
                if conn_count > self.PARAM.value['CONN_UAV_ERR_LIMIT']:
                    PrintLayout.warning('因連線失敗次數過高，請檢查後再行連線')
                    return False
                # 連線失敗，再次執行
                else:
                    PrintLayout.warning('已累積%d次連線失敗' % conn_count)
                    conn_count += 1

    # 函數宣告-連線計時器
    def connect_countdown(self):
        """連線計時器"""
        # 初始化
        PrintLayout.info('與UAV連線執行中，請稍候')
        count_down_timer = Tool.Timer()
        time_count = 0

        # 計時迴圈
        count_down_timer.set_start()
        while self.state == 'CONNECT':
            time_count += 1
            count_down_timer.wait_clock_arrive(time_count)
            PrintLayout.info('連線執行中，經過%d秒，預計剩下%d秒完成' %
                             (time_count, self.PARAM.value['CONN_UAV_DURATION'] - time_count))

            # 主程式錯誤計數增加時跳出
            if self.connect_success is not None:
                break

        # 提示成功訊息
        if self.connect_success:
            PrintLayout.success('UAV已連線，花費%d秒\n' % time_count)

        # 連線超時訊息
        else:
            PrintLayout.error('連線超時，將重新連線\n')

    # 狀態標籤-讀取
    @property
    def state(self):
        """回傳程式狀態"""
        return self.STATE.FLAG_STATE

    # 狀態標籤-寫入
    @state.setter
    def state(self, target_state):
        """設定程式狀態"""
        self.STATE.FLAG_STATE = target_state

    # 無人機程式主迴圈
    def run(self):
        """無人機程式主迴圈"""
        while True:
            # 連線失敗時自動跳出
            if not self.conn_result:
                PrintLayout.error('Connection Failed')
                return

            # 主控台
            self.panel()

            # 執行
            self.brief()
            self.execute()

            # 參數調整
            self.param_modify()

            # 任務編輯
            self.mission_modify()

            # 結束程序
            if self.state == 'CLOSE':
                # 關閉線程物件
                self.BG_THR.close()
                self.PICKER.disable()
                break

    # 選擇清單
    def panel(self):
        """選擇清單"""
        # 狀態執行限制
        if self.state != 'PANEL':
            return

        # 狀態選擇
        while True:
            # 遠端控制輸入
            # elif remote_control == True:
            # 通知遠端輸入訊號

            # 本地輸入
            # else:
            try:
                PrintLayout.info('<選擇清單> 1:任務提示與檢查 2:任務排程 3:參數修改 4:關閉程式 5:電池資訊', False)
                next_state = int(input('> '))
            except ValueError:
                PrintLayout.warning('輸入數值錯誤')
            else:
                # 前往指定狀態
                if next_state == 1:
                    self.state = 'BRIEF'    # 任務簡報->任務執行
                elif next_state == 2:
                    self.state = 'MISSION'  # 任務規劃
                elif next_state == 3:
                    self.state = 'PARAM'    # 參數調整
                elif next_state == 4:
                    self.state = 'CLOSE'    # 關閉程式
                # 電池資訊
                elif next_state == 5:
                    self.battery_info()
                else:
                    PrintLayout.warning('清單輸入錯誤，請重選\n')

            # 脫離輸入迴圈，前往目標狀態
            if self.state != 'PANEL':
                break

    # 任務簡報
    def brief(self):
        """任務簡報"""
        # 狀態執行限制
        if self.state != 'BRIEF':
            return

        # 展示飛行任務列

        # 前往執行
        self.state = 'EXECUTE'

    # 任務執行
    def execute(self):
        """任務執行"""
        # 狀態執行限制
        if self.state != 'EXECUTE':
            return

        # 線程就緒檢查
        if not self.STATE.FLAG_THREAD_READY:
            PrintLayout.warning('線程未就緒，執行禁止')
            self.state = 'PANEL'
            return

        # 執行背景線程迴圈
        self.BG_THR.start()

        # 開始監控
        self.BG_MONIT.start()

        # 執行任務
        if self.simulation:
            self.MS.start()     # 背景執行，由Mission處理標籤轉換

            # 計時器設定
            i = 0
            timer = Tool.Timer()

            # 影像顯示
            while self.state == 'EXECUTE':
                # 顯示畫面
                self.show_screen_image()

                # 迴圈等待
                i += 1
                timer.wait_clock_arrive_ms(i * (1000 / self.PARAM.value['FREQ_IMG']))

            # 關閉顯示畫面
            self.close_screen_image()

        else:
            # 啟動影像
            self.start_screen_image_loop()

            # 任務執行
            self.MS.execute()

            # 前往選單
            self.state = 'PANEL'

            # 等待線程結束
            self.join_screen_image_loop()

        # 等待線程結束
        self.BG_MONIT.join()
        self.BG_THR.join()

        # 電池資訊
        self.battery_info()

    # 參數修改
    def param_modify(self):
        """參數修改"""
        # 狀態執行限制
        if self.state != 'PARAM':
            return

        # 參數修改模組選單
        while True:
            try:
                PrintLayout.info('<選擇清單> 選取參數修改模組 1:程式參數 2:飛機參數 3:姿態配平微調 4:關閉修改清單', False)
                next_state = int(input('> '))
            except ValueError:
                PrintLayout.warning('輸入數值錯誤')
            else:
                # 程式參數修改，介接參數雙模組
                if next_state == 1:
                    while True:
                        try:
                            PrintLayout.info('<選擇清單> 程式參數模組 11:重新載入 12:儲存參數檔案 2:顯示參數表'
                                             ' 3:無 4:關閉修改清單', False)
                            next_state = int(input('> '))
                        except ValueError:
                            PrintLayout.warning('輸入數值錯誤')
                        else:
                            # 重新載入
                            if next_state == 11:
                                self.PARAM_IO.read_param_from_csv()
                                self.PARAM.update(self.PARAM_IO.export())
                                PrintLayout.success('參數表重新載入完成')
                            # 儲存參數檔案
                            elif next_state == 12:
                                self.PARAM_IO.update(self.PARAM.export())
                                self.PARAM_IO.write_param_into_csv()
                                PrintLayout.success('參數儲存成功')
                            # 顯示參數表
                            elif next_state == 2:
                                PrintLayout.info('', False)
                                self.PARAM.param_list()
                            # 退出選單
                            elif next_state == 4:
                                break
                            else:
                                PrintLayout.warning('清單輸入錯誤，請重選\n')

                # 無人機參數修改，介接專用修改模組與無人機連線物件
                elif next_state == 2:
                    pass

                # 姿態配平微調
                elif next_state == 3:
                    self.SEN_TRIM.input_interface()

                # 退出選單
                elif next_state == 4:
                    break

                else:
                    PrintLayout.warning('清單輸入錯誤，請重選\n')

        # 前往選單
        self.state = 'PANEL'

    # 任務修改
    def mission_modify(self):
        # 狀態執行限制
        if self.state != 'MISSION':
            return

        # 任務修改

        # 前往選單
        self.state = 'PANEL'

    # 電池資訊
    def battery_info(self):
        battery_offset = 0.2
        battery_level = ((battery_offset+self.UAV.battery.voltage-14.8)/(16.8-14.8))*100
        PrintLayout.info('目前電量 %.1f %%, 電壓 %.3f V' % (battery_level, battery_offset+self.UAV.battery.voltage))
        if 30 < battery_level < 50:
            PrintLayout.warning('電池電量低於一半，請預備更換')
        elif battery_level < 30:
            PrintLayout.warning('電池電量低於30%，請更換電池以維持飛行效能表現')

    # 影像畫面顯示迴圈
    def screen_image_loop(self):
        """影像畫面顯示迴圈"""
        # 線程就緒檢查
        if not self.STATE.FLAG_THREAD_READY:
            return

        # 計時器設定
        i = 0
        timer = Tool.Timer()

        # 影像顯示
        while self.state == 'EXECUTE':
            # 顯示畫面
            self.show_screen_image()

            # 迴圈等待
            i += 1
            timer.wait_clock_arrive_ms(i * (1000 / self.PARAM.value['FREQ_IMG']))

        # 關閉顯示畫面
        self.close_screen_image()

    # 啟動影像顯示線程
    def start_screen_image_loop(self):
        """啟動影像顯示線程"""
        self.screen_thread_process = threading.Thread(target=self.screen_image_loop)
        self.screen_thread_process.start()

    # 等待線程結束
    def join_screen_image_loop(self):
        """等待影像顯示線程結束"""
        self.screen_thread_process.join()

    def show_screen_image(self):
        self.BG_THR.show_screen_image()

    def close_screen_image(self):
        self.BG_THR.close_screen_image()
        PrintLayout.info('監控螢幕關閉')

    # 啟動背景狀態監聽器
    def start_listener(self):
        self.UAV.add_message_listener('RC_CHANNELS', self.msg_to_rc_channels)
        self.UAV.add_message_listener('GLOBAL_POSITION_INT', self.msg_to_odometry)

    # 遙控器數值擷取
    def msg_to_rc_channels(self, x, name, msg):
        # 接收遙控器讀值 65 RC_CHANNELS
        self.STATE.RC['1'] = int(msg.chan1_raw)
        self.STATE.RC['2'] = int(msg.chan2_raw)
        self.STATE.RC['3'] = int(msg.chan3_raw)
        self.STATE.RC['4'] = int(msg.chan4_raw)
        self.STATE.RC['5'] = int(msg.chan5_raw)
        self.STATE.RC['6'] = int(msg.chan6_raw)
        self.STATE.RC['7'] = int(msg.chan7_raw)
        self.STATE.RC['8'] = int(msg.chan8_raw)

    def msg_to_odometry(self, x, name, msg):
        # 讀取里程計數值 33 GLOBAL_POSITION_INT
        self.STATE.ALT_ABS = float(msg.relative_alt)/1000
        self.STATE.ALT_VEL_ABS = -float(msg.vz)/100


# 物件位址傳遞模組
class ObjectHandler:
    # 物件位址連結
    def __init__(self, state, cmd, param, param_io, uav):
        self.STATE = state
        self.CMD = cmd
        self.PARAM = param
        self.PARAM_IO = param_io
        self.UAV = uav
