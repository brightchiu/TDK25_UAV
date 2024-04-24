# HSV 色域參數調整程式

import cv2 as cv


class ColorAdjustment:
    def __init__(self, cam_num):
        self.max_value = 255
        self.max_value_H = 360//2
        self.low_H = 0
        self.low_S = 0
        self.low_V = 0
        self.high_H = self.max_value_H
        self.high_S = self.max_value
        self.high_V = self.max_value
        self.window_capture_name = 'Video Capture'
        self.window_detection_name = 'Object Detection'
        self.low_H_name = 'Low H'
        self.low_S_name = 'Low S'
        self.low_V_name = 'Low V'
        self.high_H_name = 'High H'
        self.high_S_name = 'High S'
        self.high_V_name = 'High V'

        self.camera_init(cam_num)

    def on_low_h_thresh_trackbar(self, val):
        self.low_H = val
        self.low_H = min(self.high_H-1, self.low_H)
        cv.setTrackbarPos(self.low_H_name, self.window_detection_name, self.low_H)

    def on_high_h_thresh_trackbar(self, val):
        self.high_H = val
        self.high_H = max(self.high_H, self.low_H+1)
        cv.setTrackbarPos(self.high_H_name, self.window_detection_name, self.high_H)

    def on_low_s_thresh_trackbar(self, val):
        self.low_S = val
        self.low_S = min(self.high_S-1, self.low_S)
        cv.setTrackbarPos(self.low_S_name, self.window_detection_name, self.low_S)

    def on_high_s_thresh_trackbar(self, val):
        self.high_S = val
        self.high_S = max(self.high_S, self.low_S+1)
        cv.setTrackbarPos(self.high_S_name, self.window_detection_name, self.high_S)

    def on_low_v_thresh_trackbar(self,val):
        self.low_V = val
        self.low_V = min(self.high_V-1, self.low_V)
        cv.setTrackbarPos(self.low_V_name, self.window_detection_name, self.low_V)

    def on_high_v_thresh_trackbar(self, val):
        self.high_V = val
        self.high_V = max(self.high_V, self.low_V+1)
        cv.setTrackbarPos(self.high_V_name, self.window_detection_name, self.high_V)

    def camera_init(self, cam_num):
        self.cam = cv.VideoCapture(cam_num)   #鏡頭
        cv.namedWindow(self.window_capture_name)
        cv.namedWindow(self.window_detection_name)
        cv.createTrackbar(self.low_H_name, self.window_detection_name, self.low_H, self.max_value_H,
                          self.on_low_h_thresh_trackbar)
        cv.createTrackbar(self.high_H_name, self.window_detection_name, self.high_H, self.max_value_H,
                          self.on_high_h_thresh_trackbar)
        cv.createTrackbar(self.low_S_name, self.window_detection_name, self.low_S, self.max_value,
                          self.on_low_s_thresh_trackbar)
        cv.createTrackbar(self.high_S_name, self.window_detection_name, self.high_S, self.max_value,
                          self.on_high_s_thresh_trackbar)
        cv.createTrackbar(self.low_V_name, self.window_detection_name, self.low_V, self.max_value,
                          self.on_low_v_thresh_trackbar)
        cv.createTrackbar(self.high_V_name, self.window_detection_name, self.high_V, self.max_value,
                          self.on_high_v_thresh_trackbar)

    def run(self):
        while self.cam.isOpened():
            state_retrieve, image = self.cam.read()
            if state_retrieve:
                image = cv.resize(image, (480, 360))
                # 480 360
                frame_HSV = cv.cvtColor(image, cv.COLOR_BGR2HSV)
                frame_threshold = cv.inRange(frame_HSV, (self.low_H, self.low_S, self.low_V),
                                             (self.high_H, self.high_S,self. high_V))

                cv.imshow(self.window_capture_name, image)
                cv.imshow(self.window_detection_name, frame_threshold)

                if cv.waitKey(100) & 0xff == ord("q"):
                    break
            else:
                break

    def close(self):
        self.cam.release()
        cv.destroyAllWindows()


if __name__ == '__main__':
    CA = ColorAdjustment(0)
    CA.run()
    CA.close()


        

























