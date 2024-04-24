# TDK25 飛行組 競賽程式碼 - 控制器模組
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞

# 版本資訊
# 程式序號 TDK25-CTRL
# 版號 02
# 最後編輯日 2021/10/11

# 程式說明
# 微分項作用於回授值避免參考值導致誤差突然變化
# 可選擇時間變化項使用實際值或理想值
# 可選擇標準型(增益同時作用於三個迴路)、並聯型(理論型，各別作用)
# 調整反揚升至係數乘完後做限制

# 注意事項

# 模組匯入
import numpy as np
import matplotlib.pyplot as plt
import sched
import time

# 副程式匯入
import TDK25.Tool as Tool


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
    def set_param(self, freq=20, kp=0.0, ki=0.0, kd=0.0, mode='parallel'):
        """設定參數"""
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


# 回授補償控制器
class RateFeedbackController:
    def __init__(self, freq=20, main_kp=0.0, rate_kp=0.0, rate_ki=0.0, rate_kd=0.0, rate_mode='parallel'):
        # 初始化控制器
        self.main_ctrl = PIdashDController(freq=freq, kp=main_kp)
        self.rate_ctrl = PIdashDController(freq=freq, kp=rate_kp, ki=rate_ki, kd=rate_kd, mode=rate_mode)

        # 微分回授輸入參考(供外部使用)
        self.rate_reference = 0

    # 設定參數
    def set_param(self, freq=20, main_kp=0.0, rate_kp=0.0, rate_ki=0.0, rate_kd=0.0, rate_mode='parallel'):
        """設定參數"""
        self.main_ctrl.set_param(freq=freq, kp=main_kp)
        self.rate_ctrl.set_param(freq=freq, kp=rate_kp, ki=rate_ki, kd=rate_kd, mode=rate_mode)

    # 設定均值濾波器取樣數
    def set_filter(self, main_in, main_out, rate_in, rate_out, rate_dt):
        """設定均值濾波器取樣數"""
        self.main_ctrl.set_input_filter_sn(main_in)
        self.main_ctrl.set_output_filter_sn(main_out)
        self.rate_ctrl.set_input_filter_sn(rate_in)
        self.rate_ctrl.set_output_filter_sn(rate_out)
        self.rate_ctrl.set_dt_filter_sn(rate_dt)

    # 設定輸出與積分限制
    def set_limit(self, main_out_limit, rate_out_limit, rate_int_limit):
        """設定輸出與積分限制"""
        self.main_ctrl.set_output_limit(main_out_limit)
        self.rate_ctrl.set_output_limit(rate_out_limit)
        self.rate_ctrl.set_integral_limit(rate_int_limit)

    # 設定輸出中性區
    def set_deadzone(self, rate_out_deadzone, main_feedback_deadzone):
        """設定輸出中性區"""
        self.rate_ctrl.set_output_deadzone(rate_out_deadzone)
        self.main_ctrl.set_feedback_deadzone(main_feedback_deadzone)

    # 執行控制器
    def run(self, reference, feedback, rate_feedback):
        """執行控制器"""
        self.rate_reference = self.main_ctrl.processing(reference, feedback)
        output = self.rate_ctrl.processing(self.rate_reference, rate_feedback)

        return output

    # 重設控制器
    def reset(self):
        """重設控制器"""
        self.main_ctrl.reset()
        self.rate_ctrl.reset()


class PlantModel:
    def __init__(self, freq):
        self.m = 1.8
        self.acc = 0
        self.vel = 0
        self.pos = 0
        self.pre_acc = 0
        self.pre_vel = 0
        self.dt = 1/freq

    def system_model_block(self, control_u):
        self.acc = control_u/self.m
        self.vel += self.acc * self.dt
        self.pos += self.vel * self.dt


# 測試範例 轉速回授
if __name__ == '__main__':
    loop_freq = 30
    duration = 10
    refer = 0
    ctrl = RateFeedbackController()
    ctrl.set_param(freq=loop_freq, main_kp=1, rate_kp=10, rate_ki=0, rate_kd=0.5, rate_mode='parallel')
    ctrl.set_filter(1, 3, 3, 5, 5)
    ctrl.set_limit(main_out_limit=2, rate_out_limit=50, rate_int_limit=50)
    ctrl.set_deadzone(0)

    r_refo = []
    r_stao = []
    r_refi = []
    r_stai = []
    r_t = []
    r_lp = []

    p = PlantModel(loop_freq)

    scheduler = sched.scheduler(time.perf_counter, time.sleep)

    def task():
        global duration, counter, loop_freq, refer
        # if duration*0.2/(1/loop_freq) < counter < duration*0.4/(1/loop_freq) and refer < 1:
        #     refer += 1
        # if counter > duration*0.6/(1/loop_freq) and refer > 0:
        #     refer -= 1
        if duration * 0.2  < counter < duration * 0.4 and refer < 1:
            refer += 1
        if counter > duration*0.6 and refer > 0:
            refer -= 1
        u = ctrl.run(refer, p.pos, p.vel)
        p.system_model_block(u)

        p.system_model_block(u)
        r_refo.append(refer)
        r_stao.append(p.pos)
        r_refi.append(ctrl.rate_reference)
        r_stai.append(p.vel)
        r_t.append(ctrl_timer.elapsed_time())
        r_lp.append(ctrl_timer.period())

    ctrl_timer = Tool.Timer()
    counter = 0
    while ctrl_timer.elapsed_time() < duration:
        for tick in range(loop_freq):
            cc = ctrl_timer.start_time + counter + (1 / loop_freq) * tick
            print(cc)
            scheduler.enterabs(cc, 1, task)
            scheduler.run(True)
            print(scheduler.queue)
            # scheduler.enterabs(ctrl_timer.start_time + counter + (1 / loop_freq) * tick+0.05, 1, time.sleep, argument=(0.005,))

        scheduler.run()
        counter += 1

    print('Loop info maxT=%.2f ms, minT=%.2f ms, aveT=%.2f ms stdT=%.2f ms'
          % (max(r_lp)*1000, min(r_lp)*1000, np.average(r_lp)*1000, np.std(r_lp)*1000))

    C1_R = [r_refo, r_stao, r_refi, r_stai, r_t, r_lp]

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

# 測試範例
if __name__ == '__main__' and False:
    ctrl = PIdashDController(freq=30, kp=1, ki=0.1, kd=0.5)
    ctrl.set_output_filter_sn(5)
    ctrl.flag_dt = True
    ctrl.mode = 'standard'
    i = 0
    timer = Tool.Timer()
    for x in range(100):
        ctrl.processing(x, x-2)
        timer.wait_clock_arrive_ms((x+1)*1000/30)
    ctrl.reset()
