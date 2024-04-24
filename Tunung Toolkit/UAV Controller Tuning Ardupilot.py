# 無人機校正模式
# 搭配 Ardupilot 韌體
# Date: 2021/9/24

import asyncio
import dronekit
import struct
import math
import time
import threading


class State:
    def __init__(self):
        self.ROLL = 0
        self.PITCH = 0
        self.YAW = 0
        self.ROLL_RATE = 0
        self.PITCH_RATE = 0
        self.YAW_RATE = 0

        self.PGM_STATE = 0
        self.TEST_MODE = 0
        self.SIG_MODE = 0


class Command:
    def __init__(self):
        self.ROLL = 0
        self.PITCH = 0
        self.YAW = 0
        self.ROLL_RATE = 0
        self.PITCH_RATE = 0
        self.YAW_RATE = 0


class Parameter:
    def __init__(self):
        self.value = {
            # Angle Controller Parameter 0-6
            'ATC_INPUT_TC': 0.15,           # Attitude control input time constant 0-1 (Input LPF)
            'ATC_ANG_RLL_P': 2,             # Roll axis angle controller P gain = 3-12
            'ATC_RATE_R_MAX': 720,          # Angular Velocity Max for Roll
            'ATC_ANG_PIT_P': 2,             # Pitch axis angle controller P gain
            'ATC_RATE_P_MAX': 720,          # Angular Velocity Max for Pitch
            'ATC_ANG_YAW_P': 2,             # Yaw axis angle controller P gain
            'ATC_RATE_Y_MAX': 720,          # Angular Velocity Max for Yaw

            # Rate Controller Parameter 7-31
            # 7-14
            'ATC_RAT_RLL_P': 0.3,           # Roll axis rate controller gain
            'ATC_RAT_RLL_I': 0.5,
            'ATC_RAT_RLL_D': 0.004,
            'ATC_RAT_RLL_IMAX': 0.5,
            'ATC_ACCEL_R_MAX': 108000,      # Roll acc max
            'ATC_RAT_RLL_FLTT': 20,         # Roll axis rate controller filter (Hz)
            'ATC_RAT_RLL_FLTE': 20,
            'ATC_RAT_RLL_FLTD': 20,
            # 15-22
            'ATC_RAT_PIT_P': 1,             # Pitch axis rate controller gain
            'ATC_RAT_PIT_I': 0.4,
            'ATC_RAT_PIT_D': 0.3,
            'ATC_RAT_PIT_IMAX': 0.5,
            'ATC_ACCEL_P_MAX': 108000,      # Pitch acc max
            'ATC_RAT_PIT_FLTT': 20,         # Pitch axis rate controller filter (Hz)
            'ATC_RAT_PIT_FLTE': 20,
            'ATC_RAT_PIT_FLTD': 20,
            # 23-30
            'ATC_RAT_YAW_P': 2,
            'ATC_RAT_YAW_I': 0.3,
            'ATC_RAT_YAW_D': 0.05,
            'ATC_RAT_YAW_IMAX': 0.5,
            'ATC_ACCEL_Y_MAX': 36000,       # Yaw acc max
            'ATC_RAT_YAW_FLTT': 20,
            'ATC_RAT_YAW_FLTE': 20,
            'ATC_RAT_YAW_FLTD': 20,

            # Command and Signal Parameter 31-34
            'CMD_FREQ': 50,
            'SIG_FREQ_SAMPLE': 500,
            'SIG_FREQ': 1,
            'SIG_AMP': 5,

            # Others parameter 35-40  only 35 can be set
            'ATC_ANGLE_BOOST': 0,           # Increases throttle to reduce loss of altitude
            'CONN_UAV_ERR_LIMIT': 3,
            'CONN_UAV_BAUD': 921600,
            'CONN_UAV_RATE': 100,
            'CONN_UAV_DURATION': 24,
            'CONN_UAV_TIMEOUT': 30,
            'CONN_UAV_PORT': '/dev/cu.usbmodem14401',
        }


# 計時器模組
class Timer:
    """計時器模組"""
    def __init__(self):
        self.set_start()

    def set_start(self):
        """設定起始時刻"""
        self.start_ns = time.time_ns()
        self.previous_ns = self.start_ns

    def elapsed_time(self):
        """經過時間"""
        return (time.time_ns() - self.start_ns)/1e9

    def elapsed_time_ms(self):
        """經過時間(毫秒)"""
        return (time.time_ns() - self.start_ns)/1e6

    def elapsed_time_us(self):
        """經過時間(微秒)"""
        return (time.time_ns() - self.start_ns)/1e3

    # 回傳經過時間
    def period(self):
        """回傳間隔時間"""
        self.current_ns = time.time_ns()
        period = self.current_ns - self.previous_ns
        self.previous_ns = self.current_ns
        return period / 1e9

    # 等待時刻抵達(線程排程器使用)
    def wait_clock_arrive(self, clock):
        """等待時刻抵達"""
        while time.time_ns()-self.start_ns < clock*1e9:
            pass

    def wait_clock_arrive_ms(self, clock):
        """等待時刻抵達(毫秒)"""
        while time.time_ns()-self.start_ns < clock*1e6:
            pass

    def wait_clock_arrive_us(self, clock):
        """等待時刻抵達(微秒)"""
        while time.time_ns()-self.start_ns < clock*1e3:
            pass

    @staticmethod
    def wait(sec):
        t0 = time.time()
        while time.time()-t0 < sec:
            pass

    @staticmethod
    def wait_ms(ms):
        t0 = time.time()
        while time.time()-t0 < ms/1000:
            pass

    @staticmethod
    def wait_us(us):
        t0 = time.time_ns()
        while time.time_ns()-t0 < us*1000:
            pass


# 訊號產生物件
class SignalGenerator:
    def __init__(self, state, command, param, uav):
        self.UAV = uav
        self.CMD = command
        self.STATE = state
        self.PARAM = param

        self.param_values = []

        self.close = False

    # 訊號產生協程
    async def sig_gen(self):
        print('Signal Gen start')
        while True:
            self.sleep()
            await self.arm()
            await self.activate()
            await self.disarm()
            self.stop()

            # 狀態機迴圈間隔(防止資源佔用)
            await asyncio.sleep(0.1)

            if self.close:
                break

        print('Signal Gen closed')

    # 休眠
    def sleep(self):
        if self.STATE.PGM_STATE != 1:
            return

        self.CMD.ROLL = 0
        self.CMD.PITCH = 0
        self.CMD.YAW = 0
        self.CMD.ROLL_RATE = 0
        self.CMD.PITCH_RATE = 0
        self.CMD.YAW_RATE = 0

    # 解鎖
    async def arm(self):
        if self.STATE.PGM_STATE != 2:
            return

        # 解鎖無人機
        self.UAV.arm(wait=True)
        print('Drone is armed')

        # 切換至無衛星導航模式
        if self.UAV.version.autopilot_type == 3:
            self.UAV.mode = dronekit.VehicleMode('GUIDED_NOGPS')
        elif self.UAV.version.autopilot_type == 12:
            self.UAV.mode = dronekit.VehicleMode('OFFBOARD')
        print('Drone is in offboard mode, ready to go for launch.')

        await asyncio.sleep(0.5)
        self.CMD.VZ_RATE = 0.7
        self.send_cmd(0b00000111)

        await asyncio.sleep(2)
        self.CMD.VZ_RATE = 0.5
        self.send_cmd(0b00000111)
        print('Throttle set at trim value.')

        # 傳送到運作狀態
        self.STATE.PGM_STATE = 3

    # 運作中(訊號生成階段)
    async def activate(self):
        if self.STATE.PGM_STATE != 3:
            return

        # 訊號生成迴圈(含發送)
        t0 = time.time()
        i = 0
        while self.STATE.PGM_STATE == 3:
            i += 1
            t = time.time() - t0

            # == 訊號生成 ==
            # 手動模式
            if self.STATE.SIG_MODE == 0:
                sig = 0  # 暫不開放手動（後續增加）
            # 弦波模式
            elif self.STATE.SIG_MODE == 1:
                sig = math.sin(2 * math.pi * self.PARAM.value['SIG_FREQ'] * t) * self.PARAM.value['SIG_AMP']
            # 方波模式
            elif self.STATE.SIG_MODE == 2:
                sig = math.sin(2 * math.pi * self.PARAM.value['SIG_FREQ'] * t) * self.PARAM.value['SIG_AMP']
                if sig > 0:
                    sig = self.PARAM.value['SIG_AMP']
                elif sig < 0:
                    sig = -self.PARAM.value['SIG_AMP']
                else:
                    sig = 0
            # 三角波
            elif self.STATE.SIG_MODE == 3:
                sig = 0  # 不開放三角波

            else:
                sig = 0

            # == 輸出頻道切換 ==
            # 角度模式(角速度命令為擬和訊號)
            if self.STATE.TEST_MODE < 3:
                if self.STATE.TEST_MODE == 0:  # Roll
                    self.CMD.ROLL = sig
                    self.CMD.ROLL_RATE = self.normalize((self.CMD.ROLL - self.STATE.ROLL)
                                                        * self.PARAM.value['ATC_ANG_RLL_P'],
                                                        self.PARAM.value['ATC_RATE_R_MAX'])
                    self.CMD.PITCH = 0
                    self.CMD.YAW = 0
                    self.CMD.PITCH_RATE = 0
                    self.CMD.YAW_RATE = 0
                elif self.STATE.TEST_MODE == 1:  # Pitch
                    self.CMD.PITCH = sig
                    self.CMD.PITCH_RATE = self.normalize((self.CMD.PITCH - self.STATE.PITCH)
                                                         * self.PARAM.value['ATC_ANG_PIT_P'],
                                                         self.PARAM.value['ATC_RATE_P_MAX'])
                    self.CMD.ROLL = 0
                    self.CMD.YAW = 0
                    self.CMD.ROLL_RATE = 0
                    self.CMD.YAW_RATE = 0
                elif self.STATE.TEST_MODE == 2:  # Yaw
                    self.CMD.YAW = sig
                    self.CMD.YAW_RATE = self.normalize((self.CMD.YAW - self.STATE.YAW)
                                                       * self.PARAM.value['ATC_ANG_YAW_P'],
                                                       self.PARAM.value['ATC_RATE_Y_MAX'])
                    self.CMD.ROLL = 0
                    self.CMD.PITCH = 0
                    self.CMD.ROLL_RATE = 0
                    self.CMD.PITCH_RATE = 0
                else:
                    self.CMD.ROLL = 0
                    self.CMD.PITCH = 0
                    self.CMD.YAW = 0
                    self.CMD.ROLL_RATE = 0
                    self.CMD.PITCH_RATE = 0
                    self.CMD.YAW_RATE = 0

            # 角速度模式
            elif self.STATE.TEST_MODE > 2:
                if self.STATE.TEST_MODE == 3:  # Roll Rate
                    self.CMD.ROLL = 0
                    self.CMD.ROLL_RATE = sig
                    self.CMD.PITCH = 0
                    self.CMD.YAW = 0
                    self.CMD.PITCH_RATE = 0
                    self.CMD.YAW_RATE = 0
                elif self.STATE.TEST_MODE == 4:  # Pitch Rate
                    self.CMD.PITCH = 0
                    self.CMD.PITCH_RATE = sig
                    self.CMD.ROLL = 0
                    self.CMD.YAW = 0
                    self.CMD.ROLL_RATE = 0
                    self.CMD.YAW_RATE = 0
                elif self.STATE.TEST_MODE == 5:  # Yaw Rate
                    self.CMD.YAW = 0
                    self.CMD.YAW_RATE = sig
                    self.CMD.ROLL = 0
                    self.CMD.PITCH = 0
                    self.CMD.ROLL_RATE = 0
                    self.CMD.PITCH_RATE = 0
                else:
                    self.CMD.ROLL = 0
                    self.CMD.PITCH = 0
                    self.CMD.YAW = 0
                    self.CMD.ROLL_RATE = 0
                    self.CMD.PITCH_RATE = 0
                    self.CMD.YAW_RATE = 0

            # 送出訊號
            if self.STATE.TEST_MODE < 3:
                mask = 0b00000111
            elif self.STATE.TEST_MODE > 2:  # 角速度模式
                if self.STATE.TEST_MODE == 3:  # Roll Rate
                    mask = 0b00000110
                elif self.STATE.TEST_MODE == 4:  # Pitch Rate
                    mask = 0b00000101
                elif self.STATE.TEST_MODE == 5:  # Yaw Rate
                    mask = 0b00000011
                else:
                    mask = 0b00000111
            else:
                mask = 0b00000111
            self.send_cmd(mask)

            await asyncio.sleep(t0 + i * (1 / self.PARAM.value['SIG_FREQ_SAMPLE']) - time.time())

    # 上鎖
    async def disarm(self):
        if self.STATE.PGM_STATE != 4:
            return

        if self.UAV.armed:
            self.CMD.ROLL = 0
            self.CMD.PITCH = 0
            self.CMD.YAW = 0
            self.CMD.ROLL_RATE = 0
            self.CMD.PITCH_RATE = 0
            self.CMD.YAW_RATE = 0
            self.CMD.VZ_RATE = 0
            self.send_cmd(0b00000111)
            print('Drone is disarming.')
            self.UAV.disarm(wait=False)
            while self.UAV.armed:
                await asyncio.sleep(0.1)
            print('Drone is disarmed.')

        # 傳送到睡眠狀態
        self.STATE.PGM_STATE = 1

    # 停止
    def stop(self):
        if self.STATE.PGM_STATE != 5:
            return

        if self.UAV.armed:
            self.CMD.ROLL = 0
            self.CMD.PITCH = 0
            self.CMD.YAW = 0
            self.CMD.ROLL_RATE = 0
            self.CMD.PITCH_RATE = 0
            self.CMD.YAW_RATE = 0
            self.CMD.VZ_RATE = 0
            self.send_cmd(0b00000111)
            print('Drone is disarming.')
            self.UAV.disarm(wait=True)
            print('Drone is disarmed.')

        self.close = True

        # 回送成預設狀態
        self.STATE.PGM_STATE = 0

    def send_cmd(self, mask):
        # 命令編碼，使用偏航角速度控制(餘採姿態控制)
        msg = self.UAV.message_factory.set_attitude_target_encode(
            0,  # 開機時間
            0,  # 目標系統
            0,  # 目標裝置
            mask,  # 命令遮罩(1忽略，0啟用)
            self.to_quaternion(self.CMD.ROLL, self.CMD.PITCH, self.CMD.YAW),  # 姿態四元數
            math.radians(self.CMD.ROLL_RATE),  # Roll滾轉角速度(radian/s)
            math.radians(self.CMD.PITCH_RATE),  # Pitch滾轉角速度(radian/s)
            math.radians(self.CMD.YAW_RATE),  # Yaw滾轉角速度(radian/s)
            self.CMD.VZ_RATE)  # 集合推力(0~1)

        # 傳輸命令
        self.UAV.send_mavlink(msg)

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

    @staticmethod
    def normalize(value, limit):
        if value > limit:
            corrected = limit
        elif value < -limit:
            corrected = -limit
        else:
            corrected = value

        return corrected


# TCP/IP傳輸物件
class TCPTransmit:
    def __init__(self, state, command, param, uav):
        self.UAV = uav
        self.CMD = command
        self.STATE = state
        self.PARAM = param

        self.param_values = []

    # 伺服器執行迴圈
    async def sever_loop(self, reader, writer):
        while not self._connect:
            await asyncio.sleep(1)

        while self.STATE.PGM_STATE != 5:
            try:
                # Get message length
                data = await reader.read(2)
                msg_length = struct.unpack('H', data)[0]
                # print(msg_length)

                # Get message ID
                msg = await reader.read(msg_length)
                msg_id = msg[0]
                # print('ID', msg_id)

                # ID 1 Request State
                if msg_id == 1:
                    msg_send = [self.CMD.ROLL, self.CMD.PITCH, self.CMD.YAW,
                                self.CMD.ROLL_RATE, self.CMD.PITCH_RATE, self.CMD.YAW_RATE,
                                self.STATE.ROLL, self.STATE.PITCH, self.STATE.YAW,
                                self.STATE.ROLL_RATE, self.STATE.PITCH_RATE, self.STATE.YAW_RATE]
                    # print(msg_send)
                    # Encode the value
                    msg_encoded = b''
                    for num in msg_send:
                        msg_encoded += self.float_to_bytes(num)
                        # print(msg_encoded)

                    # Add state and mode
                    msg_encoded = self.STATE.SIG_MODE.to_bytes(1, 'little') + msg_encoded
                    msg_encoded = self.STATE.TEST_MODE.to_bytes(1, 'little') + msg_encoded
                    msg_encoded = self.STATE.PGM_STATE.to_bytes(1, 'little') + msg_encoded

                    # Add message ID
                    msg_id = 2
                    msg_encoded = msg_id.to_bytes(1, 'little') + msg_encoded

                # ID 3 Request Param
                elif msg_id == 3:
                    if not self.param_values:
                        self.param_values = list(self.PARAM.value.values())

                    param_id = msg[1]
                    param_value = self.param_values[param_id]

                    msg_encoded = self.float_to_bytes(param_value)
                    msg_encoded = param_id.to_bytes(1, 'little') + msg_encoded

                    # Add message ID
                    msg_id = 4
                    msg_encoded = msg_id.to_bytes(1, 'little') + msg_encoded

                # ID 5 Set Param
                elif msg_id == 5:
                    param_id = msg[1]
                    param_value = struct.unpack('f', msg[2:6])[0]
                    param_name = list(self.PARAM.values.keys())[param_id]
                    set_result = 0
                    if param_id < 18:
                        try:
                            self.UAV.parameters[param_name] = param_value
                        except dronekit.APIException:
                            print('Set %s Failed' % param_name)
                            result_msg = 'Set %s Failed' % param_name
                        else:
                            self.PARAM.values[param_name] = param_value
                            print('Successful %s = %.3f' % (param_name, self.UAV.parameters[param_name]))
                            result_msg = 'Successful %s = %.3f' % (param_name, self.UAV.parameters[param_name])
                            set_result = 1
                    else:
                        self.PARAM.values[param_name] = param_value
                        print('Successful %s = %.3f' % (param_name, param_value))
                        result_msg = 'Successful %s = %.3f' % (param_name, param_value)
                        set_result = 1

                    # Encode return message
                    # ID_VALUE_FLAG_RESULT
                    msg_encoded = result_msg.encode()
                    msg_encoded = set_result.to_bytes(1, 'little') + msg_encoded
                    msg_encoded = self.float_to_bytes(param_value) + msg_encoded
                    msg_encoded = param_id.to_bytes(1, 'little') + msg_encoded

                    # Add message ID
                    msg_id = 6
                    msg_encoded = msg_id.to_bytes(1, 'little') + msg_encoded

                elif msg_id == 7:
                    print('set state', msg[1])
                    self.STATE.PGM_STATE = msg[1]
                    self.STATE.TEST_MODE = msg[2]
                    self.STATE.SIG_MODE = msg[3]

                    # Add state and mode
                    msg_encoded = self.STATE.SIG_MODE.to_bytes(1, 'little')
                    msg_encoded = self.STATE.TEST_MODE.to_bytes(1, 'little') + msg_encoded
                    msg_encoded = self.STATE.PGM_STATE.to_bytes(1, 'little') + msg_encoded

                    # Add message ID
                    msg_id = 8
                    msg_encoded = msg_id.to_bytes(1, 'little') + msg_encoded

                else:
                    msg_encoded = b''

                # Add message length
                msg_length = len(msg_encoded)
                msg_encoded = msg_length.to_bytes(2, 'little') + msg_encoded
                # print(msg_encoded)
                writer.write(msg_encoded)
                await writer.drain()

                if self.STATE.PGM_STATE == 5:
                    break

            except Exception:
                print('Exception Occurred')
                break

        self._connect = False
        writer.close()
        self.server.close()
        print("Close the connection")
        return

    @staticmethod
    def float_to_bytes(num):
        bytes_num = struct.pack('f', num)
        return bytes_num

    async def enable_sever(self):
        print('Creat sever')
        self.server = await asyncio.start_server(self.sever_loop, '192.168.137.186', 5000)
        print('<Notify> Start sever on ', self.server.sockets[0].getsockname())
        self._connect = True
        async with self.server:
            await self.server.serve_forever()


class UAVTuning:
    def __init__(self):
        self.STATE = State()
        self.CMD = Command()
        self.PARAM = Parameter()

        # 連線程序
        self.conn_result = self.connect()

        # 系統啟動
        if self.conn_result:
            # 建立監聽
            self.UAV.add_message_listener('ATTITUDE', self.get_attitude)

            # 建立訊號產生器物件
            self.SIG_GEN = SignalGenerator(self.STATE, self.CMD, self.PARAM, self.UAV)

            # 建立伺服器物件
            self.TCP = TCPTransmit(self.STATE, self.CMD, self.PARAM, self.UAV)

            # 前往選擇介面
            self.STATE.PGM_STATE = 1

        # 連線失敗跳過
        else:
            pass

    # 無人機連線
    def connect(self):
        """無人機連線"""
        conn_count = 1
        while conn_count <= self.PARAM.value['CONN_UAV_ERR_LIMIT']:
            self.connect_success = None
            # 載入連線計時器
            uav_connect_count = threading.Thread(target=self.connect_countdown)
            uav_connect_count.start()

            self.UAV = dronekit.connect(self.PARAM.value['CONN_UAV_PORT'],
                                        baud=self.PARAM.value['CONN_UAV_BAUD'],
                                        rate=int(self.PARAM.value['CONN_UAV_RATE']))

            # wait_ready函式以後須另外撰寫
            self.connect_success = True
            #    self.UAV.wait_ready(types=True, timeout=self.PARAM.value['CONN_UAV_TIMEOUT'],
            #                                           raise_exception=False)

            # 等待計時器關閉
            uav_connect_count.join()

            # 連線成功
            if self.connect_success:
                return True
            else:
                # 多次連線失敗，關閉程式
                if conn_count > self.PARAM.value['CONN_UAV_ERR_LIMIT']:
                    print('因連線失敗次數過高，請檢查後再行連線')
                    return False
                # 連線失敗，再次執行
                else:
                    print('已累積%d次連線失敗' % conn_count)
                    conn_count += 1

    # 函數宣告-連線計時器
    def connect_countdown(self):
        """連線計時器"""
        # 初始化
        print('與UAV連線執行中，請稍候')
        count_down_timer = Timer()
        time_count = 0

        # 計時迴圈
        count_down_timer.set_start()
        while self.STATE.PGM_STATE == 0:
            time_count += 1
            count_down_timer.wait_clock_arrive(time_count)
            print('連線執行中，經過%d秒，預計剩下%d秒完成' %
                  (time_count, self.PARAM.value['CONN_UAV_DURATION'] - time_count))

            # 主程式錯誤計數增加時跳出
            if self.connect_success is not None:
                break

        # 提示成功訊息
        if self.connect_success:
            print('UAV已連線，花費%d秒\n' % time_count)

        # 連線超時訊息
        else:
            print('連線超時，將重新連線\n')

    async def main_loop(self):
        await asyncio.gather(self.TCP.enable_sever(), self.SIG_GEN.sig_gen())

    def run(self):
        asyncio.run(self.main_loop())

    def get_attitude(self, x, name, msg):
        self.STATE.ROLL = math.degrees(float(msg.roll))
        self.STATE.PITCH = math.degrees(float(msg.pitch))
        self.STATE.YAW = math.degrees(float(msg.yaw))
        self.STATE.ROLL_RATE = math.degrees(float(msg.rollspeed))
        self.STATE.PITCH_RATE = math.degrees(float(msg.pitchspeed))
        self.STATE.YAW_RATE = math.degrees(float(msg.yawspeed))


if __name__ == '__main__':
    UT = UAVTuning()
    UT.run()
