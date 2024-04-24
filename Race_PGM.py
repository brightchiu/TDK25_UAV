# TDK25 飛行組 競賽程式碼
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞、羅宇彥、徐慧哲

# 版本資訊
# 位階 0
# 程式名稱 啟動程式碼
# 版號 02
# 最後編輯日 2021/07/16

# 程式說明
# 啟動與執行程式碼

# 模組匯入
import TDK25
import platform

if __name__ == '__main__':
    # 平台判定
    if platform.platform() == 'macOS-10.16-x86_64-i386-64bit':
        simulation = True
        print('<系統訊息> 以模擬環境測試')
    else:
        simulation = False
        print('<系統訊息> 以實機環境測試')

    # 實例化物件
    PGM = TDK25.TDK25UAV(simulation=simulation)

    # 主程式迴圈執行
    PGM.run()

    print('主程序已關閉，感謝使用')
    del PGM