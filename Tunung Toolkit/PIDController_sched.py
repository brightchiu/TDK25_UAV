# PI-D Controller

# 2021/11/15
import time
import sched
import numpy as np

import TDK25.Tool as Tool
import math
import matplotlib.pyplot as plt
import dronekit
import threading


class PIdashDController:
    def __init__(self, freq=20, kp=0.0, ki=0.0, kd=0.0, mode='parallel'):
        # 旗標
        self.flag_first_call = True
        # 參數作用模式(parallel:平行, standard:標準)
        if mode != 'parallel' or mode != 'standard':
            self.mode = 'parallel'
        else:
            self.mode = mode

        # 目標、回授、誤差
        self.reference = 0  # 輸入參考
        self.feedback = 0  # 回授
        self.feedback_previous = 0  # 前次回授
        self.error = 0  # 誤差值
        self.error_integral = 0  # 誤差積分

        # 增益值
        self.kp = kp  # 比例增益
        self.ki = ki  # 積分增益
        self.kd = kd  # 微分增益

        # 增益輸出
        self.output = 0
        self.kp_output = 0
        self.ki_output = 0
        self.kd_output = 0

        # 限制器(對稱限界)
        self.integral_limit = 1e10
        self.output_limit = 1e10

        # 中性區(對稱限界)
        self.input_deadzone = 0
        self.feedback_deadzone = 0
        self.output_deadzone = 0

        # 濾波器(輸入預設不開=1)
        self.input_filter_sn = 1
        self.input_maf = Tool.MovingAverageFilter(self.input_filter_sn)
        self.dt_filter_sn = 1
        self.dt_maf = Tool.MovingAverageFilter(self.dt_filter_sn)
        self.output_filter_sn = 1
        self.output_maf = Tool.MovingAverageFilter(self.output_filter_sn)

        # 計時器
        self.timer = Tool.Timer()

        # 控制器頻率
        self.freq = freq

    # 設定參數
    def set_param(self, freq=20, kp=0.0, ki=0.0, kd=0.0, dt_flag=False, mode='parallel'):
        """設定參數"""
        self.flag_dt = dt_flag              # 使用實際時間間隔
        self.mode = mode                    # 參數作用模式(parallel:平行, standard:標準)
        self.kp = kp                        # 比例增益
        self.ki = ki                        # 積分增益
        self.kd = kd                        # 微分增益
        self.freq = freq                    # 控制器頻率

    # 設定輸入均值濾波器取樣數
    def set_input_filter_sn(self, input_sn):
        """設定輸入均值濾波器取樣數"""
        if input_sn >= 1:
            self.input_maf.sample_number = int(input_sn)

    # 設定微分項濾波器頻率
    def set_dt_filter_sn(self, dt_sn):
        """設定微分項均值濾波器取樣數"""
        if dt_sn >= 1:
            self.dt_maf.sample_number = int(dt_sn)

    # 設定輸出均值濾波器取樣數
    def set_output_filter_sn(self, output_sn):
        """設定輸出均值濾波器取樣數"""
        if output_sn >= 1:
            self.output_maf.sample_number = int(output_sn)

    # 設定輸出限制
    def set_output_limit(self, limit):
        """設定輸出限制"""
        self.output_limit = limit

    # 設定積分限制
    def set_integral_limit(self, limit):
        """設定積分限制"""
        self.integral_limit = limit

    # 設定輸出中性區
    def set_output_deadzone(self, deadzone):
        """設定輸出中性區"""
        self.output_deadzone = deadzone

    # 設定回授中性區
    def set_feedback_deadzone(self, deadzone):
        """設定回授中性區"""
        self.feedback_deadzone = deadzone

    # 設定輸入中性區
    def set_input_deadzone(self, deadzone):
        """設定輸入中性區"""
        self.input_deadzone = deadzone

    # 更新控制器
    def processing(self, reference, feedback):
        """更新輸入與回授，計算輸出值"""
        # 輸入參考值(濾波處理)
        reference = self.input_maf.processing(reference)

        # 輸入截止
        if -self.input_deadzone < reference < self.input_deadzone:
            self.reference = 0
        else:
            self.reference = reference

        # 回授截止
        if -self.feedback_deadzone < feedback < self.feedback_deadzone:
            self.feedback = 0
        else:
            self.feedback = feedback

        # 更新時鐘
        delta_time = self.timer.period()

        # 誤差計算
        self.error = self.reference - self.feedback

        # 積分反揚升(前置)
        if self.ki_output > self.integral_limit:
            if self.error * delta_time < 0:
                self.error_integral += (self.error * delta_time)
        elif self.ki_output < -self.integral_limit:
            if self.error * delta_time > 0:
                self.error_integral += (self.error * delta_time)
        else:
            self.error_integral += (self.error * delta_time)

        # 回授值微分計算
        delta_feedback = self.feedback - self.feedback_previous
        self.feedback_previous = self.feedback
        if self.flag_first_call:
            derivative_raw = 0
            self.flag_first_call = False
        else:
            derivative_raw = delta_feedback / delta_time
        derivative = self.dt_maf.processing(derivative_raw)

        # 誤差增益處理
        if self.mode == 'parallel':
            self.kp_output = self.error * self.kp
            self.ki_output = self.error_integral * self.ki
            self.kd_output = derivative * self.kd
        elif self.mode == 'standard':
            self.kp_output = self.error * self.kp
            self.ki_output = self.error_integral * (self.kp/self.ki)
            self.kd_output = derivative * (self.kd/self.kp)
        else:
            self.kp_output = self.error * self.kp
            self.ki_output = self.error_integral * self.ki
            self.kd_output = derivative * self.kd

        # 積分反揚升限制器(後置)
        if self.ki_output > self.integral_limit:
            self.ki_output = self.integral_limit
        elif self.ki_output < -self.integral_limit:
            self.ki_output = -self.integral_limit

        # 加總輸出
        output = self.kp_output + self.ki_output - self.kd_output

        # 輸出值濾波器
        output = self.output_maf.processing(output)

        # 總輸出值限制器(飽和與截止)
        if output > self.output_limit:
            self.output = self.output_limit
        elif output < -self.output_limit:
            self.output = -self.output_limit
        elif -self.output_deadzone < output < self.output_deadzone:
            self.output = 0
        else:
            self.output = output

        return self.output

    # 重設控制器
    def reset(self):
        """重設控制器"""
        self.flag_first_call = True
        self.feedback_previous = 0
        self.error_integral = 0
        self.input_maf.reset()
        self.dt_maf.reset()
        self.output_maf.reset()


class PlantModel:
    def __init__(self, freq):
        self.m = 1.8
        self.acc = 0
        self.vel = 0
        self.pos = 1
        self.pre_acc = 0
        self.pre_vel = 0
        self.dt = 1/freq

    def system_model_block(self, control_u):
        self.acc = math.sin(math.radians(control_u))
        self.vel += self.acc * self.dt
        self.pos += self.vel * self.dt


class ActuatorModel:
    def __init__(self):
        pass

    @ staticmethod
    def actuator_model_block(control_u, t):
        force = 263 * math.exp(-13.54)*control_u
        return force


class RateFeedbackController:
    def __init__(self, scheduler, freq, duration):
        self.ctrl_out = PIdashDController(freq=freq, kp=0.5, ki=0, kd=0.5)
        self.ctrl_in = PIdashDController(freq=freq, kp=0, ki=0, kd=0)
        self.scheduler = scheduler
        self.freq = freq
        self.duration = duration
        self.p = PlantModel(20)
        self.p.pos = 0.5
        self.ref = 0
        self.r_refo = []
        self.r_stao = []
        self.r_refi = []
        self.r_stai = []
        self.r_u = []
        self.r_t = []
        self.r_lp = []
        self.ctrl_timer = Tool.Timer()
        self.counter = 1

    def looping(self):
        while self.ctrl_timer.elapsed_time() < self.duration:
            self.scheduler.enterabs(self.ctrl_timer.start_time + (1/self.freq) * self.counter, 1, self.process)
            self.scheduler.run()

    def process(self):
        if self.duration*0.4/(1/self.freq) < self.counter < self.duration*0.6/(1/self.freq) and self.ref < 1:
            self.ref += 1
        if self.counter > self.duration*0.8/(1/self.freq) and self.ref > 0:
            self.ref -= 1
        u = self.ctrl_out.processing(self.ref, self.p.pos)
        # u = self.ctrl_in.processing(ref_in, self.p.vel)
        self.p.system_model_block(u)
        self.r_refo.append(self.ref)
        self.r_stao.append(self.p.pos)
        self.r_refi.append(0)
        self.r_stai.append(self.p.vel)
        self.r_t.append(self.ctrl_timer.elapsed_time())
        self.r_lp.append(self.ctrl_timer.period())
        self.counter += 1


def controller_loop(scheduler, freq, duration, sn):
    rfc = RateFeedbackController(scheduler, freq, duration)
    rfc.looping()

    rfc.ctrl_out.reset()
    rfc.ctrl_in.reset()

    if sn == 0:
        global C1_R
        C1_R = [rfc.r_refo, rfc.r_stao, rfc.r_refi, rfc.r_stai, rfc.r_t, rfc.r_lp]
    if sn == 1:
        global C2_R
        C2_R = [rfc.r_refo, rfc.r_stao, rfc.r_refi, rfc.r_stai, rfc.r_t, rfc.r_lp]

    lock = threading.Lock()
    lock.acquire()
    print('Loop info %d' % sn, max(rfc.r_lp), min(rfc.r_lp), np.average(rfc.r_lp), np.std(rfc.r_lp))
    lock.release()


# 測試範例
# Rate Feedback with MAVLink Simulate
if __name__ == '__main__':
    # uav = dronekit.connect('/dev/cu.usbmodem01', baud=921600, rate=30)
    print('UAV Connected')
    C1_R = []
    C2_R = []
    s = sched.scheduler(time.perf_counter, time.sleep)
    print('Controller Start')
    # controller_loop(s, 50, 20, 0)
    C1 = threading.Thread(target=controller_loop, args=(s, 500, 10, 0))
    #C2 = threading.Thread(target=controller_loop, args=(s, 50, 60, 1))
    C1.start()
    #C2.start()

    C1.join()
    #C2.join()

    plt.figure(figsize=(5, 5), dpi=300)
    plt.step(C1_R[4], C1_R[0], label='ref')
    plt.step(C1_R[4], C1_R[1], label='alt')
    plt.xlabel('Time (sec)')
    plt.ylabel('Altitude (m)')
    plt.title('Altitude Controller %d' % 1)
    plt.show()

    plt.figure(figsize=(5, 5), dpi=300)
    plt.step(C1_R[4], C1_R[2])
    plt.step(C1_R[4], C1_R[3])
    plt.xlabel('Time (sec)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Altitude Rate Controller %d' % 1)
    plt.show()

    plt.figure(figsize=(5, 5), dpi=300)
    plt.step(C1_R[1], C1_R[3])
    plt.xlabel('Position (m)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Phase Plot %d' % 1)
    plt.show()

    plt.figure(figsize=(5, 5), dpi=300)
    plt.plot(C1_R[4], C1_R[5])
    plt.xlabel('Time (sec)')
    plt.ylabel('Period (ses)')
    plt.title('Time Loop Regularity %d' % 1)
    plt.show()
    """
    plt.figure(figsize=(5, 5), dpi=300)
    plt.step(C2_R[4], C2_R[0], label='ref')
    plt.step(C2_R[4], C2_R[1], label='alt')
    plt.xlabel('Time (sec)')
    plt.ylabel('Altitude (m)')
    plt.title('Altitude Controller %d' % 2)
    plt.show()

    plt.figure(figsize=(5, 5), dpi=300)
    plt.step(C2_R[4], C2_R[2])
    plt.step(C2_R[4], C2_R[3])
    plt.xlabel('Time (sec)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Altitude Rate Controller %d' % 2)
    plt.show()

    plt.figure(figsize=(5, 5), dpi=300)
    plt.step(C2_R[1], C2_R[3])
    plt.xlabel('Position (m)')
    plt.ylabel('Velocity (m/s)')
    plt.title('Phase Plot %d' % 2)
    plt.show()

    plt.figure(figsize=(5, 5), dpi=300)
    plt.plot(C2_R[4], C2_R[5])
    plt.xlabel('Time (sec)')
    plt.ylabel('Period (ses)')
    plt.title('Time Loop Regularity %d' % 2)
    plt.show()

    plt.figure(figsize=(5, 5), dpi=300)
    plt.plot(C2_R[4], C2_R[5])
    plt.xlabel('Time (sec)')
    plt.ylabel('Period (ses)')
    plt.title('Time Loop Regularity %d' % 2)
    plt.show()
    """
    print('complete')


if __name__ == '__main__' and False:
    s = sched.scheduler(time.perf_counter, time.sleep)

    def event(sn, tick, target):
        print('task%d' % sn, 'tick%d' % tick, 'Ttime=%.3f' % target, 'time=%.3f' % time.perf_counter())
        time.sleep(sn/5000)
    reg = []
    t0 = time.perf_counter()
    for x in range(600):
        t00 = time.perf_counter()
        for i in range(100):
            s.enterabs(t0+x+0.01*i, 1, event, (1, i, t0+x+0.01*i))
        for i in range(100):
            s.enterabs(t0+x+0.01*i, 2, event, (2, i, t0+x+0.01*i))
        for i in range(100):
            s.enterabs(t0+x + 0.01 * i, 3, event, (3, i, t0+x + 0.01 * i))
        for i in range(100):
            s.enterabs(t0+x + 0.01 * i, 4, event, (4, i, t0+x + 0.01 * i))
        for i in range(100):
            s.enterabs(t0+x + 0.01 * i, 4, event, (5, i, t0+x + 0.01 * i))
        t1 = time.perf_counter()
        reg.append(t1 - t00)
        s.run()
    print('execute time =', time.perf_counter()-t0)
    print('register time =', np.average(reg))
