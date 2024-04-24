# TDK25 無人機飛行程式碼
# 程式序號：UAV-Ctrl-Tuning
# 無人機參數調整程式(樹莓派端)
# 版號：1104

from dronekit import APIException
import asyncio
import dronekit
import struct
import math
import time
import threading

# 參數儲存區


#
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

        # 監聽數據
        self.attitude = {'time': 0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                         'rollspeed': 0.0, 'pitchspeed': 0.0, 'yawspeed': 0.0,
                         'freq': 0.0}

        self.attitude_target = {'time': 0, 'mask': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                                'rollspeed': 0.0, 'pitchspeed': 0.0, 'yawspeed': 0.0, 'thrust': 0.0,
                                'freq': 0.0}

        self.esc_stat = {'rpm0': 0, 'rpm1': 0, 'rpm2': 0, 'rpm3': 0,
                         'vol0': 0.0, 'vol1': 0.0, 'vol2': 0.0, 'vol3': 0.0,
                         'amp0': 0.0, 'amp1': 0.0, 'amp2': 0.0, 'amp3': 0.0,
                         'freq': 0.0}

        self.actuator_target = {'time': 0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0, 'throttle': 0.0}

        self.rc_channels = {'time': 0, '1': 0, '2': 0, '3': 0, '4': 0, '5': 0, '6': 0, '7': 0, '8': 0,
                            'rssi': 0, 'freq': 0.0}

        self.z_state = {'alt': 0.0, 'vel': 0.0, 'acc': 0.0}


class Parameter:
    def __init__(self):
        self.value = {
            'CONN_UAV_ERR_LIMIT': 3,
            'CONN_UAV_BAUD': 921600,
            'CONN_UAV_RATE': 100,
            'CONN_UAV_DURATION': 24,
            'CONN_UAV_TIMEOUT': 30,
            'CONN_UAV_PORT': '/dev/cu.usbmodem01',
            }


# 時間控制函數
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


# TCP/IP傳輸物件
class TCPTransmit:
    def __init__(self, state, param, uav):
        self.UAV = uav
        self.STATE = state
        self.PARAM = param

        self.param_values = []

    # 伺服器執行迴圈
    async def sever_loop(self, reader, writer):
        while not self._connect:
            await asyncio.sleep(1)

        while 1:
            try:
                # 接收客戶端要求
                lengthMessage = str(await reader.read(3), encoding='utf-8')
                if not lengthMessage:
                    lengthMessage = 0
                else:
                    lengthMessage = int(lengthMessage)
                clientMessage = str(await reader.read(lengthMessage), encoding='utf-8')

                # 依據指定格式回傳指令
                # 統一封裝回送(不含參數)
                if clientMessage == 'get_all':
                    sub = True
                    msg = (self.att_str(sub) + self.att_tar_str(sub) + self.esc_rpm_str(sub) + self.esc_vol_str(sub) +
                           self.esc_amp_str(sub) + self.rc_str(sub) + self.act_tar_str(sub) + self.z_state_str(sub))
                # 姿態控制器監控
                elif clientMessage == 'att_att':
                    msg = self.att_str()
                elif clientMessage == 'att_tar':
                    msg = self.att_tar_str()

                # ESC狀態監控
                elif clientMessage == 'esc_rpm':
                    msg = self.esc_rpm_str()
                elif clientMessage == 'esc_vol':
                    msg = self.esc_vol_str()
                elif clientMessage == 'esc_amp':
                    msg = self.esc_amp_str()

                # 遙控器狀態
                elif clientMessage == 'rc':
                    msg = self.rc_str()

                # 控制器輸出
                elif clientMessage == 'act_tar':
                    msg = self.act_tar_str()

                # 參數回傳
                elif clientMessage == 'par_att':
                    msg = self.param_att_ctrl_str()
                elif clientMessage == 'par_att_rate':
                    msg = self.param_att_rate_ctrl_str()
                elif clientMessage == 'par_alt':
                    msg = self.param_alt_ctrl_str()

                # 參數設定
                elif clientMessage == 'set_param':
                    msg = 'ready'
                    msg = ('%3d' + msg) % len(msg)
                    writer.write(msg.encode())
                    await writer.drain()

                    lengthMessage = str(await reader.read(3), encoding='utf-8')
                    if not lengthMessage:
                        lengthMessage = 0
                    else:
                        lengthMessage = int(lengthMessage)
                    param_name = str(await reader.read(lengthMessage), encoding='utf-8')

                    msg = 'ready'
                    msg = ('%3d' + msg) % len(msg)
                    writer.write(msg.encode())
                    await writer.drain()

                    lengthMessage = str(await reader.read(3), encoding='utf-8')
                    if not lengthMessage:
                        lengthMessage = 0
                    else:
                        lengthMessage = int(lengthMessage)
                    param_value = float(str(await reader.read(lengthMessage), encoding='utf-8'))

                    if param_name and lengthMessage:
                        try:
                            self.UAV.parameters[param_name] = param_value
                        except APIException:
                            msg = 'Failed'
                        else:
                            msg = 'Successful %s = %.3f' % (param_name, self.UAV.parameters[param_name])
                            print(msg)
                    else:
                        msg = 'Failed'

                else:
                    msg = 'err'

                # 於檔頭附註接下來傳送的位元數
                msg = ('%3d' + msg) % len(msg)

                writer.write(msg.encode())
                await writer.drain()

            except asyncio.exceptions.CancelledError or ConnectionResetError('Connection lost'):
                print('Exception Occurred')
                break

            else:
                pass

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

    def att_str(self, sub=False):
        msg = 'r%.3fp%.3fy%.3frr%.3fpr%.3fyr%.3f' % (self.STATE.attitude['roll'], self.STATE.attitude['pitch'],
                                                     self.STATE.attitude['yaw'], self.STATE.attitude['rollspeed'],
                                                     self.STATE.attitude['pitchspeed'], self.STATE.attitude['yawspeed'])
        if sub:
            return '=att,' + msg
        else:
            return msg

    def att_tar_str(self, sub=False):
        msg = 'r%.3fp%.3fy%.3frr%.3fpr%.3fyr%.3f' % (self.STATE.attitude_target['roll'],
                                                     self.STATE.attitude_target['pitch'],
                                                     self.STATE.attitude_target['yaw'],
                                                     self.STATE.attitude_target['rollspeed'],
                                                     self.STATE.attitude_target['pitchspeed'],
                                                     self.STATE.attitude_target['yawspeed'])
        if sub:
            return '=att_tar,' + msg
        else:
            return msg

    def esc_rpm_str(self, sub=False):
        msg = '%5d,%5d,%5d,%5d' % (self.STATE.esc_stat['rpm0'], self.STATE.esc_stat['rpm1'],
                                   self.STATE.esc_stat['rpm2'], self.STATE.esc_stat['rpm3'])
        if sub:
            return '=esc_rpm,' + msg
        else:
            return msg

    def esc_vol_str(self, sub=False):
        msg = '%.2f,%.2f,%.2f,%.2f' % (self.STATE.esc_stat['vol0'], self.STATE.esc_stat['vol1'],
                                       self.STATE.esc_stat['vol2'], self.STATE.esc_stat['vol3'])
        if sub:
            return '=esc_vol,' + msg
        else:
            return msg

    def esc_amp_str(self, sub=False):
        msg = '%.2f,%.2f,%.2f,%.2f' % (self.STATE.esc_stat['amp0'], self.STATE.esc_stat['amp1'],
                                       self.STATE.esc_stat['amp2'], self.STATE.esc_stat['amp3'])
        if sub:
            return '=esc_amp,' + msg
        else:
            return msg

    def act_tar_str(self, sub=False):
        msg = '%.3f,%.3f,%.3f,%.3f' % (self.STATE.actuator_target['roll'], self.STATE.actuator_target['pitch'],
                                       self.STATE.actuator_target['yaw'], self.STATE.actuator_target['throttle'])
        if sub:
            return '=act_tar,' + msg
        else:
            return msg

    def rc_str(self, sub=False):
        msg = '%d,%d,%d,%d,%d,%d,%d,%d' % (self.STATE.rc_channels['1'], self.STATE.rc_channels['2'],
                                           self.STATE.rc_channels['4'], self.STATE.rc_channels['3'],
                                           self.STATE.rc_channels['5'], self.STATE.rc_channels['6'],
                                           self.STATE.rc_channels['7'], self.STATE.rc_channels['8'])

        if sub:
            return '=rc,' + msg
        else:
            return msg

    def z_state_str(self, sub=False):
        msg = '%.3f,%.3f,%.3f,%.3f' % (-self.STATE.z_state['alt'], -self.STATE.z_state['vel'],
                                       -(self.STATE.z_state['acc'] + 9.81), self.STATE.attitude_target['thrust'])

        if sub:
            return '=z_state,' + msg
        else:
            return msg

    def param_att_ctrl_str(self):
        msg = '%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f' % (
            self.UAV.parameters['MC_ROLL_P'],
            self.UAV.parameters['MC_PITCH_P'],
            self.UAV.parameters['MC_YAW_P'],
            self.UAV.parameters['MC_ROLLRATE_MAX'],
            self.UAV.parameters['MC_PITCHRATE_MAX'],
            self.UAV.parameters['MC_YAWRATE_MAX'],
            self.UAV.parameters['MC_YAW_WEIGHT'])
        return msg

    def param_att_rate_ctrl_str(self):
        msg = '%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f' % (
            self.UAV.parameters['MC_ROLLRATE_K'],
            self.UAV.parameters['MC_ROLLRATE_P'],
            self.UAV.parameters['MC_ROLLRATE_I'],
            self.UAV.parameters['MC_ROLLRATE_D'],
            self.UAV.parameters['MC_RR_INT_LIM'],
            self.UAV.parameters['MC_PITCHRATE_K'],
            self.UAV.parameters['MC_PITCHRATE_P'],
            self.UAV.parameters['MC_PITCHRATE_I'],
            self.UAV.parameters['MC_PITCHRATE_D'],
            self.UAV.parameters['MC_PR_INT_LIM'],
            self.UAV.parameters['MC_YAWRATE_K'],
            self.UAV.parameters['MC_YAWRATE_P'],
            self.UAV.parameters['MC_YAWRATE_I'],
            self.UAV.parameters['MC_YAWRATE_D'],
            self.UAV.parameters['MC_YR_INT_LIM'])
        return msg

    def param_alt_ctrl_str(self):
        msg = '%.3f,%.3f,%.3f,%.3f,%.3f,%.3f' % (
            self.UAV.parameters['MPC_Z_VEL_MAX_UP'],
            self.UAV.parameters['MPC_Z_VEL_MAX_DN'],
            self.UAV.parameters['MPC_Z_P'],
            self.UAV.parameters['MPC_Z_VEL_P_ACC'],
            self.UAV.parameters['MPC_Z_VEL_I_ACC'],
            self.UAV.parameters['MPC_Z_VEL_D_ACC'])
        return msg


# 主執行物件
class UAVTuning:
    def __init__(self):
        self.STATE = State()
        self.PARAM = Parameter()

        # 連線程序
        self.conn_result = self.connect()

        # 系統啟動
        if self.conn_result:
            # 建立監聽
            self.add_msg_listener()

            # 建立伺服器物件
            self.TCP = TCPTransmit(self.STATE, self.PARAM, self.UAV)

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
        await asyncio.gather(self.TCP.enable_sever())

    def run(self):
        asyncio.run(self.main_loop())

    def add_msg_listener(self):
        # 建立各組計時器
        self.attitude_timer = Timer()
        self.attitude_target_timer = Timer()
        self.esc_stat_timer = Timer()
        self.rc_timer = Timer()

        # 架設訊息接受器
        self.UAV.add_message_listener('ATTITUDE', self.msg_to_attitude)
        self.UAV.add_message_listener('ATTITUDE_TARGET', self.msg_to_attitude_target)
        self.UAV.add_message_listener('RC_CHANNELS', self.msg_to_rc_channels)
        self.UAV.add_message_listener('BAD_DATA', self.msg_to_esc_stat)
        self.UAV.add_message_listener('ACTUATOR_CONTROL_TARGET', self.msg_to_actuator_target)
        self.UAV.add_message_listener('HIGHRES_IMU', self.msg_to_highres_imu)
        self.UAV.add_message_listener('ODOMETRY', self.msg_to_odometry)

    def msg_to_attitude(self, x, name, msg):
        # 接收角度資訊 30 ATTITUDE
        self.STATE.attitude['time'] = int(msg.time_boot_ms)
        self.STATE.attitude['roll'] = math.degrees(float(msg.roll))
        self.STATE.attitude['pitch'] = math.degrees(float(msg.pitch))
        self.STATE.attitude['yaw'] = math.degrees(float(msg.yaw))
        self.STATE.attitude['rollspeed'] = math.degrees(float(msg.rollspeed))
        self.STATE.attitude['pitchspeed'] = math.degrees(float(msg.pitchspeed))
        self.STATE.attitude['yawspeed'] = math.degrees(float(msg.yawspeed))
        self.STATE.attitude['freq'] = 1/self.attitude_timer.period()
        # print(self.attitude['roll'], self.attitude['pitch'], self.attitude['yaw'])

    def msg_to_attitude_target(self, x, name, msg):
        # 接收角度命令 83 ATTITUDE_TARGET
        self.STATE.attitude_target['time'] = int(msg.time_boot_ms)
        self.STATE.attitude_target['mask'] = int(msg.type_mask)
        (self.STATE.attitude_target['roll'],
         self.STATE.attitude_target['pitch'],
         self.STATE.attitude_target['yaw']) = self.quaternion_to_euler(msg.q)
        self.STATE.attitude_target['rollspeed'] = math.degrees(float(msg.body_roll_rate))
        self.STATE.attitude_target['pitchspeed'] = math.degrees(float(msg.body_pitch_rate))
        self.STATE.attitude_target['yawspeed'] = math.degrees(float(msg.body_yaw_rate))
        self.STATE.attitude_target['thrust'] = float(msg.thrust)
        self.STATE.attitude_target['freq'] = 1 / self.attitude_target_timer.period()
        # print(self.attitude_target)

    def msg_to_esc_stat(self, x, name, msg):
        # API不支援，需手動解碼
        # 接收ESC狀態 291 ESC_STATUS
        if msg.reason == 'unknown MAVLink message ID 291':
            self.STATE.esc_stat['rpm0'] = struct.unpack('i', self.ints_to_bytes(msg.data[18], msg.data[19],
                                                                          msg.data[20], msg.data[21]))[0]
            self.STATE.esc_stat['rpm1'] = struct.unpack('i', self.ints_to_bytes(msg.data[22], msg.data[23],
                                                                          msg.data[24], msg.data[25]))[0]
            self.STATE.esc_stat['rpm2'] = struct.unpack('i', self.ints_to_bytes(msg.data[26], msg.data[27],
                                                                          msg.data[28], msg.data[29]))[0]
            self.STATE.esc_stat['rpm3'] = struct.unpack('i', self.ints_to_bytes(msg.data[30], msg.data[31],
                                                                          msg.data[32], msg.data[33]))[0]
            self.STATE.esc_stat['vol0'] = struct.unpack('f', self.ints_to_bytes(msg.data[34], msg.data[35],
                                                                          msg.data[36], msg.data[37]))[0]
            self.STATE.esc_stat['vol1'] = struct.unpack('f', self.ints_to_bytes(msg.data[38], msg.data[39],
                                                                          msg.data[40], msg.data[41]))[0]
            self.STATE.esc_stat['vol2'] = struct.unpack('f', self.ints_to_bytes(msg.data[42], msg.data[43],
                                                                          msg.data[44], msg.data[45]))[0]
            self.STATE.esc_stat['vol3'] = struct.unpack('f', self.ints_to_bytes(msg.data[46], msg.data[47],
                                                                          msg.data[48], msg.data[49]))[0]
            self.STATE.esc_stat['amp0'] = struct.unpack('f', self.ints_to_bytes(msg.data[50], msg.data[51],
                                                                          msg.data[52], msg.data[53]))[0]
            self.STATE.esc_stat['amp1'] = struct.unpack('f', self.ints_to_bytes(msg.data[54], msg.data[55],
                                                                          msg.data[56], msg.data[57]))[0]
            self.STATE.esc_stat['amp2'] = struct.unpack('f', self.ints_to_bytes(msg.data[58], msg.data[59],
                                                                          msg.data[60], msg.data[61]))[0]
            self.STATE.esc_stat['amp3'] = struct.unpack('f', self.ints_to_bytes(msg.data[62], msg.data[63],
                                                                          msg.data[64], msg.data[65]))[0]
            self.STATE.esc_stat['freq'] = 1 / self.esc_stat_timer.period()

    def msg_to_actuator_target(self, x, name, msg):
        # 接收控制器輸出值(正規化，混合輸出前) 140 ACTUATOR_CONTROL_TARGET
        self.STATE.actuator_target['time'] = int(msg.time_usec)
        self.STATE.actuator_target['roll'] = float(msg.controls[0])
        self.STATE.actuator_target['pitch'] = float(msg.controls[1])
        self.STATE.actuator_target['yaw'] = float(msg.controls[2])
        self.STATE.actuator_target['throttle'] = float(msg.controls[3])

    def msg_to_rc_channels(self, x, name, msg):
        # 接收遙控器讀值 65 RC_CHANNELS
        self.STATE.rc_channels['time'] = int(msg.time_boot_ms)
        self.STATE.rc_channels['1'] = int(msg.chan1_raw)
        self.STATE.rc_channels['2'] = int(msg.chan2_raw)
        self.STATE.rc_channels['3'] = int(msg.chan3_raw)
        self.STATE.rc_channels['4'] = int(msg.chan4_raw)
        self.STATE.rc_channels['5'] = int(msg.chan5_raw)
        self.STATE.rc_channels['6'] = int(msg.chan6_raw)
        self.STATE.rc_channels['7'] = int(msg.chan7_raw)
        self.STATE.rc_channels['8'] = int(msg.chan8_raw)
        self.STATE.rc_channels['rssi'] = int(msg.rssi)
        self.STATE.rc_channels['freq'] = 1 / self.rc_timer.period()

    def msg_to_highres_imu(self, x, name, msg):
        # 讀取高解析IMU數值 105 HIGHRES_IMU
        self.STATE.z_state['acc'] = float(msg.zacc)

    def msg_to_odometry(self, x, name, msg):
        # 讀取里程計數值 331 ODOMETRY
        self.STATE.z_state['alt'] = float(msg.z)
        self.STATE.z_state['vel'] = float(msg.vz)

    # 轉換到歐拉角
    @staticmethod
    def quaternion_to_euler(q):
        q1 = q[0]
        q2 = q[1]
        q3 = q[2]
        q4 = q[3]

        y = 2 * (q1 * q2 + q3 * q4)
        x = 1 - 2 * (q2 * q2 + q3 * q3)
        roll = math.degrees(math.atan2(y, x))

        x = 2 * (q1 * q3 - q4 * q2)
        if math.fabs(x) >= 1:
            if x > 0:
                pitch = 90
            else:
                pitch = -90
        else:
            pitch = math.degrees(math.asin(x))

        y = 2 * (q1 * q4 + q2 * q3)
        x = 1 - 2 * (q3 * q3 + q4 * q4)
        yaw = math.degrees(math.atan2(y, x))

        return roll, pitch, yaw

    @staticmethod
    def ints_to_bytes(int0, int1, int2, int3):
        byte = (int0.to_bytes(1, 'big') + int1.to_bytes(1, 'big') +
                int2.to_bytes(1, 'big') + int3.to_bytes(1, 'big'))
        return byte


if __name__ == '__main__':
    UT = UAVTuning()
    UT.run()

