# TDK25 飛行組 競賽程式碼 - 相機模組
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞、徐慧哲

# 版本資訊
# 程式序號 TDK25-CAM
# 版號 03
# 最後編輯日 2021/07/14

# 程式說明
# 影像辨識

# 注意事項
# 參數採用外部引入方式進行

# 模組匯入
from multiprocessing import Process, Value, Pipe
import numpy as np
import threading
import dronekit_sitl
import dronekit
import math
import cv2

# 副程式匯入
import TDK25.Parameter as Parameter
import TDK25.Variable as Variable
import TDK25.Sensor as Sensor
import TDK25.Tool as Tool
from TDK25.Tool import PrintLayout


class Camera:
    """相機模組"""
    def __init__(self, state, param):
        # 取得外部統一變數
        self.STATE = state
        self.PARAM = param

        # 建立影像共享變數
        self.IMG = []

        # ->影像狀態控制
        self.IMG_STATE = Value('B', 1)  # 迴圈控制
        self.DETECT_MODE = Value('B', 21)  # 偵測模式代碼

        # ->影像位置、角度、面積
        self.POS_X_RAW = Value('d', 0.0)  # X 上下 Y左右
        self.POS_Y_RAW = Value('d', 0.0)
        self.ANG_RAW = Value('d', 0.0)
        self.AREA_RAW = Value('d', 0.0)

        # 更新參數、啟動相機
        if self.update_param():
            self.enable_grab()

    # 更新參數
    def update_param(self):
        """更新參數"""
        # 參數列表
        # mode_list = ['position', 'tracking', 'take_off']
        # color_list = ['red_light', 'red_dot_grab', 'red_dot_drop', 'blue', 'green',
        #               'black_dot', 'black_line', 'silver']
        try:
            self.set_update_rate(self.PARAM.value['FREQ_CAM'], self.PARAM.value['FREQ_CAM_MONIT'])
            self.set_image_size(self.PARAM.value['CAM_IMG_WIDTH'], self.PARAM.value['CAM_IMG_HEIGHT'])
            self.set_coordinate_center(self.PARAM.value['CAM_BODY_REF_X'], self.PARAM.value['CAM_BODY_REF_Y'])

            self.set_image_morphology_level(self.PARAM.value['CAM_MORPH_KERNEL_CL'])
            self.set_area_threshold(self.PARAM.value['CAM_IMG_AREA_LIMIT'], self.PARAM.value['CAM_TAKE_OFF_AREA_LIMIT'],
                                    self.PARAM.value['CAM_SILVER_BOX_AREA_UPPER'],
                                    self.PARAM.value['CAM_SILVER_BOX_AREA_LOWER'])
            self.set_calculate_param(self.PARAM.value['CAM_ANGLE_X'], self.PARAM.value['CAM_ANGLE_Y'],
                                     self.PARAM.value['CAM_SENSOR_TO_CAM'])

            self.set_green_threshold([self.PARAM.value['CAM_GREEN_L_H'], self.PARAM.value['CAM_GREEN_L_S'],
                                      self.PARAM.value['CAM_GREEN_L_V']],
                                     [self.PARAM.value['CAM_GREEN_H_H'], self.PARAM.value['CAM_GREEN_H_S'],
                                      self.PARAM.value['CAM_GREEN_H_V']])
            self.set_red_light_threshold([self.PARAM.value['CAM_RED_LIGHT_L_H'], self.PARAM.value['CAM_RED_LIGHT_L_S'],
                                          self.PARAM.value['CAM_RED_LIGHT_L_V']],
                                         [self.PARAM.value['CAM_RED_LIGHT_H_H'], self.PARAM.value['CAM_RED_LIGHT_H_S'],
                                          self.PARAM.value['CAM_RED_LIGHT_H_V']])
            self.set_red_dot_grab_threshold([self.PARAM.value['CAM_RED_DOT_GRAB_L_H'],
                                             self.PARAM.value['CAM_RED_DOT_GRAB_L_S'],
                                             self.PARAM.value['CAM_RED_DOT_GRAB_L_V']],
                                            [self.PARAM.value['CAM_RED_DOT_GRAB_H_H'],
                                             self.PARAM.value['CAM_RED_DOT_GRAB_H_S'],
                                             self.PARAM.value['CAM_RED_DOT_GRAB_H_V']])
            self.set_red_dot_drop_threshold([self.PARAM.value['CAM_RED_DOT_DROP_L_H'],
                                             self.PARAM.value['CAM_RED_DOT_DROP_L_S'],
                                             self.PARAM.value['CAM_RED_DOT_DROP_L_V']],
                                            [self.PARAM.value['CAM_RED_DOT_DROP_H_H'],
                                             self.PARAM.value['CAM_RED_DOT_DROP_H_S'],
                                             self.PARAM.value['CAM_RED_DOT_DROP_H_V']])

            self.set_sliver_threshold([self.PARAM.value['CAM_SILVER_L_H'], self.PARAM.value['CAM_SILVER_L_S'],
                                       self.PARAM.value['CAM_SILVER_L_V']],
                                      [self.PARAM.value['CAM_SILVER_H_H'], self.PARAM.value['CAM_SILVER_H_S'],
                                       self.PARAM.value['CAM_SILVER_H_V']])
            self.set_black_line_threshold(
                [self.PARAM.value['CAM_BLACK_LINE_L_H'], self.PARAM.value['CAM_BLACK_LINE_L_S'],
                 self.PARAM.value['CAM_BLACK_LINE_L_V']],
                [self.PARAM.value['CAM_BLACK_LINE_H_H'], self.PARAM.value['CAM_BLACK_LINE_H_S'],
                 self.PARAM.value['CAM_BLACK_LINE_H_V']])
            self.set_black_dot_threshold(
                [self.PARAM.value['CAM_BLACK_DOT_L_H'], self.PARAM.value['CAM_BLACK_DOT_L_S'],
                 self.PARAM.value['CAM_BLACK_DOT_L_V']],
                [self.PARAM.value['CAM_BLACK_DOT_H_H'], self.PARAM.value['CAM_BLACK_DOT_H_S'],
                 self.PARAM.value['CAM_BLACK_DOT_H_V']])

            self.set_blue_threshold(
                [self.PARAM.value['CAM_BLUE_L_H'], self.PARAM.value['CAM_BLUE_L_S'], self.PARAM.value['CAM_BLUE_L_V']],
                [self.PARAM.value['CAM_BLUE_H_H'], self.PARAM.value['CAM_BLUE_H_S'], self.PARAM.value['CAM_BLUE_H_V']])
        except KeyError:
            PrintLayout.error('相機參數遺失，請檢查')
            return False
        else:
            PrintLayout.success('相機模組參數更新完成')
            return True

    # 背景影像擷取程式(背景第一線程)
    def img_processing(self, send_img, img_state, detect_mode, pos_x, pos_y, angle, area):
        """背景影像擷取程式(背景第一線程)"""
        # 建立背景用全域變數
        self._resized_image = []

        # 影像傳輸管道
        self._send_img = send_img
        self._img_state = img_state

        # 辨識模式
        self._detect_mode = detect_mode

        # 共享變數
        self._pos_x = pos_x
        self._pos_y = pos_y
        self._angle = angle
        self._area = area

        # 建立背景第二線程->影像計算
        thread_img_cal = threading.Thread(target=self.img_calculate, daemon=True)
        thread_img_cal.start()

        # 啟用鏡頭
        source = cv2.VideoCapture(0)
        PrintLayout.success('相機模組擷取程序就緒')

        # 建立計時器
        timer = Tool.Timer()
        i = 0

        # 影像擷取迴圈(第一線程)
        while img_state.value:
            # 成功擷取時回傳布林與影像
            if source.isOpened() and source.grab():
                state_retrieve, image = source.retrieve()

                # 就緒時影像處理
                if state_retrieve:
                    self._resized_image = cv2.resize(image, (self.IMG_WIDTH, self.IMG_HEIGHT))

            # 迴圈等待
            i += 1
            timer.wait_clock_arrive_ms(i * int(1000 / self.FREQ_CAM))

        # 等待背景第二線程結束
        thread_img_cal.join()
        PrintLayout.info('影像模組-背景程序-擷取程序關閉')

    # 背景影像計算程式(背景第二線程)
    def img_calculate(self):
        """背景影像計算程式(背景第二線程)"""
        # 等待影像就緒
        while not len(self._resized_image):
            pass
        PrintLayout.success('相機模組影像計算程序就緒')

        # 建立計時器
        timer = Tool.Timer()
        i = 0

        # 運作迴圈
        while self._img_state.value:
            # 取得現在指定偵測模式
            color_string, detect_mode = self.detect_mode_decode(self._detect_mode.value)

            # 純色塊定位
            if detect_mode == 1:
                # 如果detect_mode是什麼顏色，就坐甚麼二值化，統一二值化後再做計算
                # 以下放置區塊程式碼
                # 回傳色塊位置、面積(下設子case方式)
                image, pos_x, pos_y, block_area = self.block_positioning(self._resized_image, color_string)

                # 輸出數值
                self._pos_x.value = pos_x
                self._pos_y.value = pos_y
                self._area.value = block_area
                self._angle.value = 0

            # 循線與色塊偵測、起飛區循線
            else:
                # 以下放置區塊程式碼
                # 回傳線中心位置、夾角、色塊位置、面積(下設子case方式)

                # 一般循線
                if detect_mode == 2:
                    image, pos_y, angle, line_area = self.line_tracking(self._resized_image, 'black_line')
                # 起飛區循線
                else:
                    # 畫面上方計算角度
                    image, line_pos_y, angle, line_area = self.line_tracking(self._resized_image, 'black_line',
                                                                             takeoff=True)
                    # 型心計算位置
                    image_sub, pos_x, pos_y, area = self.block_positioning(self._resized_image, 'black_line')

                # 計算循線時目標顏色面積
                block_area = self.block_positioning(self._resized_image, color_string, area_only=True)

                # 輸出數值
                self._pos_x.value = 0
                self._area.value = block_area
                self._pos_y.value = pos_y
                self._angle.value = angle

            # 影像傳送至雲端共享中 限制傳輸次數
            if not i % int(self.FREQ_CAM / self.FREQ_CAM_MONIT):
                self._send_img.send(image)

            # 迴圈等待
            i += 1
            timer.wait_clock_arrive_ms(i * int(1000 / self.FREQ_CAM))

        PrintLayout.info('影像模組-背景程序-處理程序關閉')

    # 更新影像(前景第二線程)
    def image_receive(self):
        PrintLayout.success('相機模組影像接收程序就緒')
        while self.send_state_to_th:
            if self.get_img.poll():
                self.IMG = self.get_img.recv()  # 影像通道從背景接收至前景

        self.get_img.close()
        PrintLayout.info('影像模組-前景程序-影像接收程序關閉')

    # 啟動背景影像處理
    def enable_grab(self):
        """啟動背景影像處理(關閉後需先啟用)"""
        # 建立傳送管道(單工)
        self.get_img, self.send_img = Pipe(duplex=False)

        # 背景執行(開啟背景執行緒)
        self.mp = Process(target=self.img_processing,
                          args=(self.send_img, self.IMG_STATE, self.DETECT_MODE,
                                self.POS_X_RAW, self.POS_Y_RAW, self.ANG_RAW, self.AREA_RAW),
                          daemon=True)
        self.mp.start()

        # 前景執行(開啟背景線程)
        self.send_state_to_th = True
        self.thread_image_receive = threading.Thread(target=self.image_receive)
        self.thread_image_receive.start()

        # 等待影像擷取就緒
        while not len(self.IMG):
            pass

        self.STATE.FLAG_CAM = True
        PrintLayout.success('相機模組就緒')

    # 關閉背景模組
    def disable_grab(self):
        """關閉背景影像處理"""
        # 送出關閉訊號到背景程序
        self.IMG_STATE.value = 0

        # 主程序側關閉程序
        self.mp.join()
        self.send_state_to_th = False
        self.thread_image_receive.join()

        self.STATE.FLAG_CAM = False
        PrintLayout.info('相機模組關閉')

    # =========== 影像處理主函式 ==========
    # 色塊處理
    def block_positioning(self, resized_image, block_color, area_only=False):
        # 僅計算面積時(算完立即回傳)
        if area_only:
            block_area = 0
            hsv_image = self.trans_bgr_to_hsv(resized_image)
            binary_image = self.trans_hsv_to_binary(hsv_image, block_color)

            contours, hierarchy = cv2.findContours(binary_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                block_area = cv2.contourArea(contour)

            return block_area

        # 設定型心暫存列表
        moment_x_register = []
        moment_y_register = []
        area_register = []

        # 銀色鐵盒定位(面積)
        if block_color == 'silver':
            # 將紅色圓圈濾掉，只留周圍和銀色鐵盒
            hsv_image = self.trans_bgr_to_hsv(resized_image)
            binary_image = self.trans_hsv_to_binary(hsv_image, block_color)
            contours, hierarchy = cv2.findContours(binary_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)

            # 輪廓識別與處理
            for contour in contours:
                # 面積計算
                block_area = cv2.contourArea(contour)
                # 濾掉周圍太大的面積，只計算鐵盒位置
                # 面積閥值根據高度變化 先不動
                if self.SILVER_BOX_AREA_UPPER > block_area > self.SILVER_BOX_AREA_LOWER:
                    # 最小面積矩形法
                    rect = cv2.minAreaRect(contour)
                    moment_y_raw, moment_x_raw = rect[0]

                    # 寫入待算列表
                    moment_x_register.append(moment_x_raw)
                    moment_y_register.append(moment_y_raw)
                    area_register.append(block_area)

        # 一般色塊定位
        else:
            # 影像二值化流程
            hsv_image = self.trans_bgr_to_hsv(resized_image)
            binary_image = self.trans_hsv_to_binary(hsv_image, block_color)
            contours, hierarchy = cv2.findContours(binary_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

            # 計算型心與面積
            for contour in contours:
                # 計算色塊面積
                block_area = cv2.contourArea(contour)

                # 面積符合使用門檻方取用(降噪處理)
                if block_area >= self.AREA_LIMIT:
                    # 取得型心位置(左上為原點，加法增量)
                    moment = cv2.moments(contour)
                    if moment["m00"] != 0:
                        moment_x_raw = int(moment["m01"] / moment["m00"])
                        moment_y_raw = int(moment["m10"] / moment["m00"])
                    else:
                        moment_x_raw = 0
                        moment_y_raw = 0

                    # 寫入待算列表
                    moment_x_register.append(moment_x_raw)
                    moment_y_register.append(moment_y_raw)
                    area_register.append(block_area)

        # 設定型心運算暫存列表
        moment_area_x = 0
        moment_area_y = 0
        area_sum = 0

        # 計算面積一次矩
        for i in range(len(moment_x_register)):
            moment_area_x += moment_x_register[i] * area_register[i]
            moment_area_y += moment_y_register[i] * area_register[i]
            area_sum += area_register[i]

        # 求得組合型心位置
        if len(moment_x_register) != 0:
            block_x = moment_area_x / area_sum
            block_y = moment_area_y / area_sum
        else:
            block_x = self.BODY_REF_X
            block_y = self.BODY_REF_Y

        # 標記位置
        cv2.circle(resized_image, (int(block_y), int(block_x)), 9, (255, 0, 0), -1)
        cv2.line(resized_image, (320, 210), (320, 270), (0, 0, 0), 3)
        cv2.line(resized_image, (280, 240), (360, 240), (0, 0, 0), 3)

        # 計算色塊位置(畫面中心基準，座標位移採增量表示)
        block_x_offset = self.BODY_REF_X - block_x
        block_y_offset = block_y - self.BODY_REF_Y

        return resized_image, block_x_offset, block_y_offset, area_sum

    # 循跡處理
    def line_tracking(self, image, color, takeoff=False):
        # 初始值
        x_list = []
        y_list = []
        area_list = []
        sum_area = 0
        rect_angle = 0
        width = 0
        height = 0

        # 起飛區例外 (如果是take off 只擷取影像上方，一般循線不擷取)
        if takeoff:
            image = image[0:150, 100:540]

        # 影像轉換處理
        hsv_image = self.trans_bgr_to_hsv(image)
        binary_image = self.trans_hsv_to_binary(hsv_image, color)

        # 取得輪廓點與索引(OpenCV Ver.2/4)
        contours, hierarchy = cv2.findContours(binary_image, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            block_area = cv2.contourArea(cnt)
            if block_area >= self.AREA_LIMIT:
                rect = cv2.minAreaRect(cnt)
                rect_angle = rect[2]
                width = rect[1][0]
                height = rect[1][1]

                # 起飛區循線只計算角度
                if takeoff:
                    point_y, point_x = 0, 0
                # 一般循線定位
                else:
                    point_y, point_x = rect[0]

                box = np.int0(cv2.boxPoints(rect))
                cv2.drawContours(image, [box], 0, (0, 0, 0), 2)

                x_list.append(point_x)
                y_list.append(point_y)
                area_list.append(block_area)

        # 因為有可能除以0，如果是零的話位置就在中間
        if not x_list:
            out_x = self.BODY_REF_X
            out_y = self.BODY_REF_Y
        else:
            out_x = sum(x_list) / len(x_list)
            out_y = sum(y_list) / len(y_list)

        # 座標轉換
        pos_y = out_y - self.BODY_REF_Y

        # 畫面面積加總
        for i in range(len(area_list)):
            sum_area += area_list[i]

        # 標記
        cv2.circle(image, (int(out_y), int(out_x)), 10, (0, 0, 0), -1)
        cv2.line(image, (320, 210), (320, 270), (0, 0, 0), 3)
        cv2.line(image, (280, 240), (360, 240), (0, 0, 0), 3)

        # 角度順時針負，逆時針正(地面座標)
        if width > height:
            angle = 90 - rect_angle
        else:
            angle = -rect_angle

        # 起飛區交界處角度計算為零
        if takeoff:
            if sum_area > self.TAKE_OFF_AREA_LIMIT or angle == 90 or angle == -90:
                angle = 0
            else:
                pass

        return image, pos_y, angle, sum_area

    # =========== 影像處理副函式 ==========
    # 將BGR轉換成HSV影像
    @staticmethod
    def trans_bgr_to_hsv(bgr_image):
        """將BGR轉換成HSV影像"""
        hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)
        return hsv_image

    # 雜訊處理
    def image_smooth(self, raw_image):
        """雜訊處理"""
        # 閉運算
        image_morphology = cv2.morphologyEx(raw_image, cv2.MORPH_CLOSE,
                                            np.ones(self.MORPH_KERNEL_CL, np.uint8))
        return image_morphology

    # 將BGR轉換成二值化影像
    def trans_hsv_to_binary(self, hsv_image, color):
        """將BGR轉換成二值化影像"""
        # 二值化
        if color == 'red_light':
            binary_image = cv2.inRange(hsv_image, self.RED_L_LIGHT, self.RED_H_LIGHT)
        elif color == 'blue':
            binary_image = cv2.inRange(hsv_image, self.BLUE_L, self.BLUE_H)
        elif color == 'green':
            binary_image = cv2.inRange(hsv_image, self.GREEN_L, self.GREEN_H)
        elif color == 'silver':
            binary_image = cv2.inRange(hsv_image, self.SILVER_L, self.SILVER_H)
        elif color == 'black_line':
            binary_image = cv2.inRange(hsv_image, self.BLACK_LINE_L, self.BLACK_LINE_H)
        elif color == 'black_dot':
            binary_image = cv2.inRange(hsv_image, self.BLACK_DOT_L, self.BLACK_DOT_H)
        elif color == 'red_dot_grab':
            binary_image = cv2.inRange(hsv_image, self.RED_DOT_GRAB_L, self.RED_DOT_GRAB_H)
        elif color == 'red_dot_drop':
            binary_image = cv2.inRange(hsv_image, self.RED_DOT_DROP_L, self.RED_DOT_DROP_H)
        else:
            binary_image = hsv_image

        smooth_binary_image = self.image_smooth(binary_image)

        return smooth_binary_image

    # 偵測模式解碼
    @staticmethod
    def detect_mode_decode(detect_mode_code):
        # 取得顏色
        color = detect_mode_code % 10  # 數值的顏色
        if color == 1:
            color_string = 'red_light'
        elif color == 2:
            color_string = 'blue'
        elif color == 3:
            color_string = 'green'
        elif color == 4:
            color_string = 'silver'
        elif color == 5:
            color_string = 'black_line'
        elif color == 6:
            color_string = 'red_dot_grab'
        elif color == 7:
            color_string = 'red_dot_drop'
        elif color == 8:
            color_string = 'black_dot'
        else:
            color_string = 'black_line'

        # 取得偵測模式(1定位,2循線,3起飛)
        detect_mode = int(detect_mode_code / 10)  # 數值的模式

        return color_string, detect_mode

    # 參數設定集
    # ========== 顏色設定 ==========
    # 綠色燈
    def set_green_threshold(self, hsv_l, hsv_h):
        """綠色閥值"""
        self.GREEN_L = np.array(hsv_l)
        self.GREEN_H = np.array(hsv_h)

    # 紅色燈
    def set_red_light_threshold(self, hsv_l, hsv_h):
        """紅色燈閥值"""
        self.RED_L_LIGHT = np.array(hsv_l)
        self.RED_H_LIGHT = np.array(hsv_h)

    # 拾物區紅點
    def set_red_dot_grab_threshold(self, hsv_l, hsv_h):
        """拾物區紅點閥值"""
        self.RED_DOT_GRAB_L = np.array(hsv_l)
        self.RED_DOT_GRAB_H = np.array(hsv_h)

    # 投放區紅點
    def set_red_dot_drop_threshold(self, hsv_l, hsv_h):
        """投放區閥值"""
        self.RED_DOT_DROP_L = np.array(hsv_l)
        self.RED_DOT_DROP_H = np.array(hsv_h)

    # 黑色線
    def set_black_line_threshold(self, hsv_l, hsv_h):
        """黑色線閥值"""
        self.BLACK_LINE_L = np.array(hsv_l)
        self.BLACK_LINE_H = np.array(hsv_h)

    # 黑色點
    def set_black_dot_threshold(self, hsv_l, hsv_h):
        """黑色點閥值"""
        self.BLACK_DOT_L = np.array(hsv_l)
        self.BLACK_DOT_H = np.array(hsv_h)

    # 藍色
    def set_blue_threshold(self, hsv_l, hsv_h):
        """藍色閥值"""
        self.BLUE_L = np.array(hsv_l)
        self.BLUE_H = np.array(hsv_h)

    # 銀色
    def set_sliver_threshold(self, hsv_l, hsv_h):
        """銀色閥值"""
        self.SILVER_L = np.array(hsv_l)
        self.SILVER_H = np.array(hsv_h)

    # ========== 影像處理設定 ==========
    # 更新頻率設定
    def set_update_rate(self, camera, monitor):
        """更新頻率"""
        self.FREQ_CAM = camera
        self.FREQ_CAM_MONIT = monitor

    # 影像尺寸
    def set_image_size(self, width, height):
        """影像尺寸(寬,高)"""
        self.IMG_WIDTH = width
        self.IMG_HEIGHT = height

    # 坐標系中心位置
    def set_coordinate_center(self, x0, y0):
        """坐標系中心，相對畫面左上角距離"""
        self.BODY_REF_X = x0
        self.BODY_REF_Y = y0

    # 閉運算模糊等級
    def set_image_morphology_level(self, level):
        """閉運算模糊等級"""
        # 轉成整數並檢查是否為奇數
        level = int(level)
        if level % 2:
            pass
        else:
            level += 1
        self.MORPH_KERNEL_CL = (level, level)

    # 面積閥值
    def set_area_threshold(self, normal, take_off, box_up, bux_low):
        """面積閥值"""
        self.AREA_LIMIT = normal
        self.TAKE_OFF_AREA_LIMIT = take_off
        self.SILVER_BOX_AREA_UPPER = box_up
        self.SILVER_BOX_AREA_LOWER = bux_low

    # 影像尺寸運算
    def set_calculate_param(self, fov_x, fov_y, camera_z_offset):
        """影像尺寸運算"""
        self.CAM_ANGLE_X = fov_x  # 相機垂直方向角度
        self.CAM_ANGLE_Y = fov_y  # 相機水平方向角度
        self.BOTTOM_TO_CAMERA = camera_z_offset  # 公尺

    # ========== 以下為API供外部使用 =================================================

    # 顯示現在顏色
    @property
    def detect_mode(self):
        """回傳現在相機偵測模式"""
        if self.DETECT_MODE.value == 11:
            color_string = ['position', 'red_light']
        elif self.DETECT_MODE.value == 12:
            color_string = ['position', 'blue']
        elif self.DETECT_MODE.value == 13:
            color_string = ['position', 'green']
        elif self.DETECT_MODE.value == 14:
            color_string = ['position', 'silver']
        elif self.DETECT_MODE.value == 15:
            color_string = ['position', 'black_line']
        elif self.DETECT_MODE.value == 16:
            color_string = ['position', 'red_dot_grab']
        elif self.DETECT_MODE.value == 17:
            color_string = ['position', 'red_dot_drop']
        elif self.DETECT_MODE.value == 18:
            color_string = ['position', 'black_dot']

        elif self.DETECT_MODE.value == 21:
            color_string = ['tracking', 'red_light']
        elif self.DETECT_MODE.value == 22:
            color_string = ['tracking', 'blue']
        elif self.DETECT_MODE.value == 23:
            color_string = ['tracking', 'green']
        elif self.DETECT_MODE.value == 24:
            color_string = ['tracking', 'silver']
        elif self.DETECT_MODE.value == 25:
            color_string = ['tracking', 'black_line']
        elif self.DETECT_MODE.value == 26:
            color_string = ['tracking', 'red_dot_grab']
        elif self.DETECT_MODE.value == 31:
            color_string = ['take_off', 'red_light']
        else:
            color_string = ['NONE', 'NONE']
        return color_string

    # 先判斷模式，再判斷顏色為何
    @detect_mode.setter
    def detect_mode(self, code):
        mode = code[0]
        color = code[1]

        if mode == 'tracking':
            if color == 'red_light':  # 紅色+循跡
                self.DETECT_MODE.value = 21
            elif color == 'blue':
                self.DETECT_MODE.value = 22
            elif color == 'green':
                self.DETECT_MODE.value = 23
            elif color == 'silver':
                self.DETECT_MODE.value = 24
            elif color == 'black_line':
                self.DETECT_MODE.value = 25
            elif color == 'red_dot_grab':  # 鹽山上紅色圓圈
                self.DETECT_MODE.value = 26
            else:
                self.DETECT_MODE.value = 21

        elif mode == 'position':
            if color == 'red_light':  # 紅色+定位
                self.DETECT_MODE.value = 11
            elif color == 'blue':
                self.DETECT_MODE.value = 12
            elif color == 'green':
                self.DETECT_MODE.value = 13
            elif color == 'silver':
                self.DETECT_MODE.value = 14
            elif color == 'black_line':
                self.DETECT_MODE.value = 15
            elif color == 'red_dot_grab':  # 鹽山上紅色圓圈
                self.DETECT_MODE.value = 16
            elif color == 'red_dot_drop':  # 投放區紅色圓圈
                self.DETECT_MODE.value = 17
            elif color == 'black_dot':
                self.DETECT_MODE.value = 18
            else:
                self.DETECT_MODE.value = 21
        elif mode == 'take_off':
            # 起飛區循線
            if color == 'red_light':
                self.DETECT_MODE.value = 31
            else:
                self.DETECT_MODE.value = 21

    @property
    def pos_x(self):
        """回傳目前色塊X位置"""
        # 計算畫素->公尺
        cam_height = round(math.tan(self.CAM_ANGLE_X * math.pi / 180 / 2), 3) * (
                self.STATE.ALT_REL + self.BOTTOM_TO_CAMERA)
        pixel_to_meter = cam_height / self.BODY_REF_X
        pos_x_meter = self.POS_X_RAW.value * pixel_to_meter
        return pos_x_meter

    @property
    def pos_y(self):
        """回傳目前色塊Y位置"""
        # 計算畫素->公尺
        cam_width = round(math.tan(self.CAM_ANGLE_Y * math.pi / 180 / 2), 3) * (
                self.STATE.ALT_REL + self.BOTTOM_TO_CAMERA)
        pixel_to_meter = cam_width / self.BODY_REF_Y
        pos_y_meter = self.POS_Y_RAW.value * pixel_to_meter
        return pos_y_meter

    @property
    def area(self):
        """回傳目前色塊面積"""
        cam_height = round(math.tan(self.CAM_ANGLE_X * math.pi / 180 / 2), 3) * (
                self.STATE.ALT_REL + self.BOTTOM_TO_CAMERA)
        cam_width = round(math.tan(self.CAM_ANGLE_Y * math.pi / 180 / 2), 3) * (
                self.STATE.ALT_REL + self.BOTTOM_TO_CAMERA)
        length_per_pixel = (cam_height * cam_width) / (self.BODY_REF_Y * self.BODY_REF_X)
        area_meter = length_per_pixel * self.AREA_RAW.value
        return area_meter

    @property
    def angle(self):
        """回傳線夾角"""
        return self.ANG_RAW.value

    @property
    def line_area_raw(self):
        """回傳線夾角"""
        return self.ANG_RAW.value

    # 視窗顯示
    def show_screen_image(self):
        """顯示原始影像"""
        cv2.imshow('Camera Monitor', self.IMG)
        cv2.waitKey(1)

    @staticmethod
    def close_screen_image():
        cv2.destroyAllWindows()

    # 關閉視窗
    def close(self):
        self.disable_grab()
        cv2.destroyAllWindows()


# 物件位址傳遞模組
class ObjectHandler:
    # 物件位址連結
    def __init__(self, state, cmd, param, uav):
        self.STATE = state
        self.CMD = cmd
        self.PARAM = param
        self.UAV = uav


# 測試程式碼
if __name__ == '__main__':
    # 物件建立
    timer_tick = Tool.Timer()
    PARAM = Parameter.Param()
    PARAM_IO = Parameter.ParamIO()
    STATE = Variable.StateVariable()
    CMD = Variable.Command()

    # 參數接入
    PARAM_IO.read_param_from_csv(show_param=False)

    # 資料傳遞->參數集
    PARAM.update(PARAM_IO.export())

    # 建立相機物件
    CAM = Camera(STATE, PARAM)
    print('相機啟動時間:', timer_tick.elapsed_time(), 'sec')

    # 模擬環境設定
    flag_real_alt = False       # 真實高度計
    flag_real_fc = False        # 真實飛控
    # 採用實際模擬
    if flag_real_alt:
        # 模擬控制器
        if not flag_real_fc:
            sitl = dronekit_sitl.start_default()
            PARAM.value['CONN_UAV_PORT'] = sitl.connection_string()
        # 控制器連線
        UAV = dronekit.connect(PARAM.value['CONN_UAV_PORT'], baud=PARAM.value['CONN_UAV_BAUD'],
                               rate=int(PARAM.value['CONN_UAV_RATE']))
        obj_handler = ObjectHandler(STATE, CMD, PARAM, UAV)
        ALT_SEN = Sensor.SensorAltLidar(obj_handler)

    # 測試參數
    test_period = 10
    fps = 20
    period = 1/fps
    loops = int(test_period*fps)

    # 模式設定
    CAM.detect_mode = ('position', 'red_dot')  # setter只能放一個input，所以使用tuple

    # 執行迴圈A
    timer_tick.set_start()
    for loop in range(loops):
        # 高度模擬改變
        if not flag_real_alt:
            if loop < loops / 2 and STATE.ALT_REL < 1.5:
                STATE.ALT_REL = min(0.05 * loop, 1.5)
            elif loop > loops / 2 and STATE.ALT_REL > 0:
                STATE.ALT_REL = min(0.05 * (loops-loop), 1.5)

        # 更新相對高度值
        if flag_real_alt:
            pass
            # STATE.ALT_REL = ALT_SEN.measure()

        CAM.show_screen_image()
        if loop % fps == 0:
            print('Mode:%s, Color:%s, X:%.2f m, Y:%.2f m, Area:%.5f m^2, Angle:%.2f deg' %
                  (CAM.detect_mode[0], CAM.detect_mode[1], CAM.pos_x, CAM.pos_y, CAM.area, CAM.angle))
            print('rel_alt = %.2f m\n' % STATE.ALT_REL)

        # 迴圈正時
        timer_tick.wait_clock_arrive((loop+1)*period)

    # 模式設定
    print('執行模式切換')
    CAM.detect_mode = ('tracking', 'red_light')  # setter只能放一個input，所以使用tuple

    # 執行迴圈B
    for loop in range(loops):
        # 高度模擬改變
        if not flag_real_alt:
            if loop < loops / 2 and STATE.ALT_REL < 1.5:
                STATE.ALT_REL = min(0.05 * loop, 1.5)
            elif loop > loops / 2 and STATE.ALT_REL > 0:
                STATE.ALT_REL = min(0.05 * (loops - loop), 1.5)

        # 更新相對高度值
        if flag_real_alt:
            pass
            # TATE.ALT_REL = ALT_SEN.measure()

        CAM.show_screen_image()
        if loop % fps == 0:
            print('Mode:%s, Color:%s, X:%.2f m, Y:%.2f m, Area:%.5f m^2, Angle:%.2f deg' %
                  (CAM.detect_mode[0], CAM.detect_mode[1], CAM.pos_x, CAM.pos_y, CAM.area, CAM.angle))
            print('rel_alt = %.2f m\n' % STATE.ALT_REL)

        # 迴圈正時
        timer_tick.wait_clock_arrive((loops + loop + 1) * period)

    # 時間計算
    t_elapsed = timer_tick.elapsed_time()

    # 關閉程序
    timer_tick.set_start()
    CAM.close()
    t_close = timer_tick.elapsed_time()
    print('Total Time: %.2f sec' % t_elapsed)
    print('fps: %.2f frames' % (loops / t_elapsed))
    print('spf: %.2f ms' % ((t_elapsed / loops)*1000))
    print('相機關閉時間:', timer_tick.elapsed_time(), 'sec')

    del CAM

    # 建立相機物件
    timer_tick.set_start()
    CAM = Camera(STATE, PARAM)
    print('相機啟動時間:', timer_tick.elapsed_time(), 'sec')

    # 模式設定
    CAM.detect_mode = ('position', 'green')  # setter只能放一個input，所以使用tuple

    # 執行迴圈C
    timer_tick.set_start()
    for loop in range(loops):
        # 高度模擬改變
        if not flag_real_alt:
            if loop < loops / 2 and STATE.ALT_REL < 1.5:
                STATE.ALT_REL = min(0.05 * loop, 1.5)
            elif loop > loops / 2 and STATE.ALT_REL > 0:
                STATE.ALT_REL = min(0.05 * (loops - loop), 1.5)

        CAM.show_screen_image()
        if loop % fps == 0:
            print('Mode:%s, Color:%s, X:%.2f m, Y:%.2f m, Area:%.5f m^2, Angle:%.2f deg' %
                  (CAM.detect_mode[0], CAM.detect_mode[1], CAM.pos_x, CAM.pos_y, CAM.area, CAM.angle))
            print('rel_alt = %.2f m\n' % STATE.ALT_REL)

        # 迴圈正時
        timer_tick.wait_clock_arrive((loop + 1) * period)

    # 時間計算
    t_elapsed = timer_tick.elapsed_time()

    # 關閉程序
    timer_tick.set_start()
    CAM.close()
    t_close = timer_tick.elapsed_time()
    print('Total Time: %.2f sec' % t_elapsed)
    print('fps: %.2f frames' % (loops / t_elapsed))
    print('spf: %.2f ms' % ((t_elapsed / loops)*1000))
    print('相機關閉時間:', timer_tick.elapsed_time(), 'sec')
