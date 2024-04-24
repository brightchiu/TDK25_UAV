# TDK25X 無人機飛行程式碼
# 程式序號：UAV-Ctrl-Tuning
# 無人機參數調整程式(樹莓派端)
# 版號：0808 快速版本

from dronekit import connect, APIException
import threading
import socket
import struct
import math
import time


class UavInfoHandler:
    def __init__(self, param):
        self.PARAM = param
        self.ERROR = False

        # 接收變數
        self.CMD_ACK = {'CMD_ID': 0, 'Result': 0}

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

    def set_address(self, ip, port):
        self.address_ip = ip
        self.address_port = port

    # 啟動連線
    def enable(self):
        try:
            self.UAV = connect(self.PARAM.CONN['PORT'], baud=self.PARAM.CONN['BAUD'],
                               rate=self.PARAM.CONN['RATE'])
        except TimeoutError:
            print('Connect timeout error, check the info. of the connection.')
            return False
        except APIException:
            print('API Error Occurred.')
            return False
        else:

            print('UAV Connected')
            return True

    # 建立伺服器
    def creat_sever(self):
        try:
            self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server.bind((self.address_ip, self.address_port))
            self.server.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, True)
            self.server.listen(1)
            self.SEVER, addr = self.server.accept()     # 建立伺服端
        except socket.error:
            print('TCP/IP establish error.')
            return False
        else:
            print('Sever establish ready')
            print('Connect to Address: ', addr)
            return True

    # 指定連線速率
    def set_msg_interval(self, msg_id, freq):
        msg = self.UAV.message_factory.command_long_encode(
            0,  # Target_system
            0,  # Target_component
            511,  # Command ID(511:MAV_CMD_SET_MESSAGE_INTERVAL)
            0,  # confirmation
            msg_id,  # P1 Message ID
            int(1e6 / freq),  # P2 Interval(us)
            0,  # P3 Response Target
            0,  # P4
            0,  # P5
            0,  # P6
            0,  # P7
        )

        for c in range(3):
            self.UAV.send_mavlink(msg)
            if self.CMD_ACK['CMD_ID'] == msg_id and self.CMD_ACK['Result'] == 0:
                break
        if self.CMD_ACK['CMD_ID'] == msg_id and self.CMD_ACK['Result'] != 0:
            print('Set message interval failed')

    def add_msg_listener(self):
        # 建立各組計時器
        self.attitude_timer = Timer()
        self.attitude_target_timer = Timer()
        self.esc_stat_timer = Timer()
        self.rc_timer = Timer()

        # 架設訊息接受器
        self.UAV.add_message_listener('COMMAND_ACK', self.msg_to_cmd_ack)
        self.UAV.add_message_listener('ATTITUDE', self.msg_to_attitude)
        self.UAV.add_message_listener('ATTITUDE_TARGET', self.msg_to_attitude_target)
        self.UAV.add_message_listener('RC_CHANNELS', self.msg_to_rc_channels)
        self.UAV.add_message_listener('BAD_DATA', self.msg_to_esc_stat)
        self.UAV.add_message_listener('ACTUATOR_CONTROL_TARGET', self.msg_to_actuator_target)
        self.UAV.add_message_listener('HIGHRES_IMU', self.msg_to_highres_imu)
        self.UAV.add_message_listener('ODOMETRY', self.msg_to_odometry)

    def msg_to_cmd_ack(self, x, name, msg):
        """MAVLink 命令回傳值"""
        # print(msg)
        if name == 'COMMAND_ACK':
            self.CMD_ACK = {'CMD_ID': msg.command, 'Result': msg.result}
            print('Request CMD ID:', msg.command, ', Result:', self.mav_result(msg.result))

    def msg_to_attitude(self, x, name, msg):
        # 接收角度資訊 30 ATTITUDE
        self.attitude['time'] = int(msg.time_boot_ms)
        self.attitude['roll'] = math.degrees(float(msg.roll))
        self.attitude['pitch'] = math.degrees(float(msg.pitch))
        self.attitude['yaw'] = math.degrees(float(msg.yaw))
        self.attitude['rollspeed'] = math.degrees(float(msg.rollspeed))
        self.attitude['pitchspeed'] = math.degrees(float(msg.pitchspeed))
        self.attitude['yawspeed'] = math.degrees(float(msg.yawspeed))
        self.attitude['freq'] = 1/self.attitude_timer.period()
        # print(self.attitude['roll'], self.attitude['pitch'], self.attitude['yaw'])

    def msg_to_attitude_target(self, x, name, msg):
        # 接收角度命令 83 ATTITUDE_TARGET
        self.attitude_target['time'] = int(msg.time_boot_ms)
        self.attitude_target['mask'] = int(msg.type_mask)
        (self.attitude_target['roll'],
         self.attitude_target['pitch'],
         self.attitude_target['yaw']) = self.quaternion_to_euler(msg.q)
        self.attitude_target['rollspeed'] = math.degrees(float(msg.body_roll_rate))
        self.attitude_target['pitchspeed'] = math.degrees(float(msg.body_pitch_rate))
        self.attitude_target['yawspeed'] = math.degrees(float(msg.body_yaw_rate))
        self.attitude_target['thrust'] = float(msg.thrust)
        self.attitude_target['freq'] = 1 / self.attitude_target_timer.period()
        # print(self.attitude_target)

    def msg_to_esc_stat(self, x, name, msg):
        # API不支援，需手動解碼
        # 接收ESC狀態 291 ESC_STATUS
        if msg.reason == 'unknown MAVLink message ID 291':
            self.esc_stat['rpm0'] = struct.unpack('i', self.ints_to_bytes(msg.data[18], msg.data[19],
                                                                          msg.data[20], msg.data[21]))[0]
            self.esc_stat['rpm1'] = struct.unpack('i', self.ints_to_bytes(msg.data[22], msg.data[23],
                                                                          msg.data[24], msg.data[25]))[0]
            self.esc_stat['rpm2'] = struct.unpack('i', self.ints_to_bytes(msg.data[26], msg.data[27],
                                                                          msg.data[28], msg.data[29]))[0]
            self.esc_stat['rpm3'] = struct.unpack('i', self.ints_to_bytes(msg.data[30], msg.data[31],
                                                                          msg.data[32], msg.data[33]))[0]
            self.esc_stat['vol0'] = struct.unpack('f', self.ints_to_bytes(msg.data[34], msg.data[35],
                                                                          msg.data[36], msg.data[37]))[0]
            self.esc_stat['vol1'] = struct.unpack('f', self.ints_to_bytes(msg.data[38], msg.data[39],
                                                                          msg.data[40], msg.data[41]))[0]
            self.esc_stat['vol2'] = struct.unpack('f', self.ints_to_bytes(msg.data[42], msg.data[43],
                                                                          msg.data[44], msg.data[45]))[0]
            self.esc_stat['vol3'] = struct.unpack('f', self.ints_to_bytes(msg.data[46], msg.data[47],
                                                                          msg.data[48], msg.data[49]))[0]
            self.esc_stat['amp0'] = struct.unpack('f', self.ints_to_bytes(msg.data[50], msg.data[51],
                                                                          msg.data[52], msg.data[53]))[0]
            self.esc_stat['amp1'] = struct.unpack('f', self.ints_to_bytes(msg.data[54], msg.data[55],
                                                                          msg.data[56], msg.data[57]))[0]
            self.esc_stat['amp2'] = struct.unpack('f', self.ints_to_bytes(msg.data[58], msg.data[59],
                                                                          msg.data[60], msg.data[61]))[0]
            self.esc_stat['amp3'] = struct.unpack('f', self.ints_to_bytes(msg.data[62], msg.data[63],
                                                                          msg.data[64], msg.data[65]))[0]
            self.esc_stat['freq'] = 1 / self.esc_stat_timer.period()

    def msg_to_actuator_target(self, x, name, msg):
        # 接收控制器輸出值(正規化，混合輸出前) 140 ACTUATOR_CONTROL_TARGET
        self.actuator_target['time'] = int(msg.time_usec)
        self.actuator_target['roll'] = float(msg.controls[0])
        self.actuator_target['pitch'] = float(msg.controls[1])
        self.actuator_target['yaw'] = float(msg.controls[2])
        self.actuator_target['throttle'] = float(msg.controls[3])

    def msg_to_rc_channels(self, x, name, msg):
        # 接收遙控器讀值 65 RC_CHANNELS
        self.rc_channels['time'] = int(msg.time_boot_ms)
        self.rc_channels['1'] = int(msg.chan1_raw)
        self.rc_channels['2'] = int(msg.chan2_raw)
        self.rc_channels['3'] = int(msg.chan3_raw)
        self.rc_channels['4'] = int(msg.chan4_raw)
        self.rc_channels['5'] = int(msg.chan5_raw)
        self.rc_channels['6'] = int(msg.chan6_raw)
        self.rc_channels['7'] = int(msg.chan7_raw)
        self.rc_channels['8'] = int(msg.chan8_raw)
        self.rc_channels['rssi'] = int(msg.rssi)
        self.rc_channels['freq'] = 1 / self.rc_timer.period()

    def msg_to_highres_imu(self, x, name, msg):
        # 讀取高解析IMU數值 105 HIGHRES_IMU
        self.z_state['acc'] = float(msg.zacc)

    def msg_to_odometry(self, x, name, msg):
        # 讀取里程計數值 331 ODOMETRY
        self.z_state['alt'] = float(msg.z)
        self.z_state['vel'] = float(msg.vz)

    @staticmethod
    def mav_result(msg_result):
        msg_result = int(msg_result)
        if msg_result == 0:
            return 'Accepted'
        elif msg_result == 1:
            return 'Temporarily rejected'
        elif msg_result == 2:
            return 'Denied'
        elif msg_result == 3:
            return 'Unsupported'
        elif msg_result == 4:
            return 'Failed'
        elif msg_result == 5:
            return 'In progress'
        elif msg_result == 6:
            return 'Cancelled'

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

    # 啟動伺服器監聽
    def start_sever(self):
        self.SEVER_THR = threading.Thread(target=self.tcp_listener)
        self.SEVER_THR.start()
        print('Sever start to echo.')

    # 伺服端迴圈
    def tcp_listener(self):
        # TCP自動開啟迴圈
        self.SEVER.setblocking(True)
        while 1:
            try:
                # 接收客戶端要求
                lengthMessage = str(self.SEVER.recv(3), encoding='utf-8')
                if not lengthMessage:
                    lengthMessage = 0
                else:
                    lengthMessage = int(lengthMessage)
                clientMessage = str(self.SEVER.recv(lengthMessage), encoding='utf-8')

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
                    self.SEVER.send(msg.encode())

                    lengthMessage = str(self.SEVER.recv(3), encoding='utf-8')
                    if not lengthMessage:
                        lengthMessage = 0
                    else:
                        lengthMessage = int(lengthMessage)
                    param_name = str(self.SEVER.recv(lengthMessage), encoding='utf-8')

                    msg = 'ready'
                    msg = ('%3d' + msg) % len(msg)
                    self.SEVER.send(msg.encode())

                    lengthMessage = str(self.SEVER.recv(3), encoding='utf-8')
                    if not lengthMessage:
                        lengthMessage = 0
                    else:
                        lengthMessage = int(lengthMessage)
                    param_value = float(str(self.SEVER.recv(lengthMessage), encoding='utf-8'))

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

                self.SEVER.send(msg.encode())

            except socket.error:
                print('TCP transfer error. Reopen.')
                self.SEVER.close()
                self.creat_sever()
            else:
                pass

    def att_str(self, sub=False):
        msg = 'r%.3fp%.3fy%.3frr%.3fpr%.3fyr%.3f' % (self.attitude['roll'], self.attitude['pitch'],
                                                     self.attitude['yaw'], self.attitude['rollspeed'],
                                                     self.attitude['pitchspeed'], self.attitude['yawspeed'])
        if sub:
            return '=att,' + msg
        else:
            return msg

    def att_tar_str(self, sub=False):
        msg = 'r%.3fp%.3fy%.3frr%.3fpr%.3fyr%.3f' % (self.attitude_target['roll'],
                                                     self.attitude_target['pitch'],
                                                     self.attitude_target['yaw'],
                                                     self.attitude_target['rollspeed'],
                                                     self.attitude_target['pitchspeed'],
                                                     self.attitude_target['yawspeed'])
        if sub:
            return '=att_tar,' + msg
        else:
            return msg

    def esc_rpm_str(self, sub=False):
        msg = '%5d,%5d,%5d,%5d' % (self.esc_stat['rpm0'], self.esc_stat['rpm1'],
                                   self.esc_stat['rpm2'], self.esc_stat['rpm3'])
        if sub:
            return '=esc_rpm,' + msg
        else:
            return msg

    def esc_vol_str(self, sub=False):
        msg = '%.2f,%.2f,%.2f,%.2f' % (self.esc_stat['vol0'], self.esc_stat['vol1'],
                                       self.esc_stat['vol2'], self.esc_stat['vol3'])
        if sub:
            return '=esc_vol,' + msg
        else:
            return msg

    def esc_amp_str(self, sub=False):
        msg = '%.2f,%.2f,%.2f,%.2f' % (self.esc_stat['amp0'], self.esc_stat['amp1'],
                                       self.esc_stat['amp2'], self.esc_stat['amp3'])
        if sub:
            return '=esc_amp,' + msg
        else:
            return msg

    def act_tar_str(self, sub=False):
        msg = '%.3f,%.3f,%.3f,%.3f' % (self.actuator_target['roll'], self.actuator_target['pitch'],
                                       self.actuator_target['yaw'], self.actuator_target['throttle'])
        if sub:
            return '=act_tar,' + msg
        else:
            return msg

    def rc_str(self, sub=False):
        msg = '%d,%d,%d,%d,%d,%d,%d,%d' % (self.rc_channels['1'], self.rc_channels['2'],
                                           self.rc_channels['4'], self.rc_channels['3'],
                                           self.rc_channels['5'], self.rc_channels['6'],
                                           self.rc_channels['7'], self.rc_channels['8'])

        if sub:
            return '=rc,' + msg
        else:
            return msg

    def z_state_str(self, sub=False):
        msg = '%.3f,%.3f,%.3f,%.3f' % (-self.z_state['alt'], -self.z_state['vel'], -(self.z_state['acc'] + 9.81),
                                       self.attitude_target['thrust'])

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


class Param:
    def __init__(self):
        self.CONN = {
            'PORT': '/dev/cu.usbmodem01',   # 連接位址 樹莓派->/dev/ttyACM0 '/dev/cu.usbmodem01' /dev/cu.usbserial-0001
            'BAUD': 2000000,                # 鮑率
            'RATE': 50,                    # 資料串流更新率(Hz)
            }


# ==================================================================================================
# TDK25X飛行組 無人機程式
# 程式序號：UAV-Tool-Timer
# 計時器
# 版號：0310


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


if __name__ == '__main__':
    parameter = Param()
    uav = UavInfoHandler(parameter)
    uav.set_address('192.168.137.186', 5000)

    # 正常啟動
    if uav.enable() and uav.creat_sever():
        # 伺服器端連線建立
        uav.start_sever()

        # 無人機端連線建立
        uav.add_msg_listener()
        # uav.set_msg_interval(141, 1)
        # uav.set_msg_interval(290, 1)
        # uav.set_msg_interval(105, 1)
        # uav.set_msg_interval(331, 1)
        # uav.set_msg_interval(36, 1)
        # uav.set_msg_interval(116, 1)
        # uav.set_msg_interval(74, 1)
        # uav.set_msg_interval(30, 30)
        # uav.set_msg_interval(83, 30)

        while True:
            pass
    # 連線失敗
    else:
        pass

"""
For console test

from dronekit import connect
import math

CONNECT_PORT = '/dev/cu.usbmodem01'
CONNECT_BAUD = 2000000
UAV = connect(CONNECT_PORT, baud=CONNECT_BAUD, rate=100)

attitude_target = {'time': 0, 'mask': 0.0, 'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
                                'rollspeed': 0.0, 'pitchspeed': 0.0, 'yawspeed': 0.0, 'thrust': 0.0,
                                'freq': 0}
                                
def quaternion_to_euler(q):
    q1 = q[0]
    q2 = q[1]
    q3 = q[2]
    q4 = q[3]

    y = 2 * (q1 * q2 + q3 * q4)
    x = 1 - 2 * (q2 * q2 + q3 * q3)
    roll = math.degrees(math.atan2(y, x))

    x = 2 * (q1 * q3 - q4 * q1)
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

def msg_process(x, name, msg):
    if name == 'COMMAND_ACK':
        self.CMD_ACK = {'CMD_ID': msg.command, 'Result': msg.result}
        print('Request CMD ID:', msg.command, ', Result:', self.mav_result(msg.result))
    # 接收角度資訊 30 ATTITUDE
    if name == 'ATTITUDE':
        pass
    # 接收角度命令 83 ATTITUDE_TARGET
    if name == 'ATTITUDE_TARGET':
        print(quaternion_to_euler(msg.q))
        print(msg)
    if name == 'ATTITUDE_QUATERNION':
        pass

UAV.add_message_listener('*', msg_process)


from dronekit import connect
import struct
CONNECT_PORT = '/dev/cu.usbmodem01'
CONNECT_BAUD = 2000000
UAV = connect(CONNECT_PORT, baud=CONNECT_BAUD, rate=100)
def ints_to_bytes(int0, int1, int2, int3):
    byte = (int0.to_bytes(1, 'big') + int1.to_bytes(1, 'big') +
            int2.to_bytes(1, 'big') + int3.to_bytes(1, 'big'))
    return byte
def msg_process(x, name, msg):
    if msg.reason == 'unknown MAVLink message ID 291':
        print(struct.unpack('f', ints_to_bytes(msg.data[38], msg.data[39], msg.data[40], msg.data[41]))[0])
UAV.add_message_listener('BAD_DATA', msg_process)
"""
