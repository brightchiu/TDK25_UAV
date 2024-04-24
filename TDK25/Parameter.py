# TDK25 飛行組 競賽程式碼 - 參數模組
# C04 台灣大艦隊
# 程式撰寫：邱柏瑞

# 版本資訊
# 程式序號 TDK25-PARAM
# 版號 03
# 最後編輯日 2021/07/06

# 程式說明
# 存取參數檔，以CSV讀寫，包含預設值、值域、資料型態、說明
# 欄分類：名稱、數值、單位、上界、下界、預設值、資料型態、說明(0~7)
# 名稱類型：(1)_(2)_(3) 主類別_次類別_參數名
# 於初始化時讀取，飛行前儲存，副本於儲存區
# 取用方式：實例化物件後，呼叫 value['XXXXX'] 方法，XXXXX為參數名稱，便可使用
# 任何修改後，務必執行 write_param_into_csv 方法執行存檔作業，供下次使用

# 注意事項
# 字串格式參數毋需上下界，但以外之值皆需要有，否則會引發錯誤
# 寫入檔可分成標準寫入

# 模組匯入
import platform
import csv

from TDK25.Tool import PrintLayout


# 參數管理模組
class Param:
    """參數管理模組"""
    def __init__(self):
        # 參數儲存字典
        self.value = {}
        self.unit = {}
        self.boundary = {}
        self.default = {}
        self.data_type = {}
        self.description = {}

    # 更新參數儲存字典
    def update(self, dataset):
        """更新參數儲存字典"""
        self.value = dataset[0]
        self.unit = dataset[1]
        self.boundary = dataset[2]
        self.default = dataset[3]
        self.data_type = dataset[4]
        self.description = dataset[5]

    # 輸出參數儲存字典
    def export(self):
        """輸出參數儲存字典"""
        return self.value, self.unit, self.boundary, self.default, self.data_type, self.description

    # 展示所有參數資訊
    def param_list(self):
        """展示所有參數資訊"""
        print('\n{:39}'.format(''), '==============================================')
        print('{:39}'.format(''), '>>>>>>>>>> TDK25 UAV Parameter List <<<<<<<<<<')
        print('{:39}'.format(''), '==============================================\n')
        print('{:25}'.format('Name'),
              '{:15}'.format('Value'),
              '{:10}'.format('Unit'),
              '{:15}'.format('Bound L-H'),
              '{:15}'.format('Default'),
              '{:15}'.format('Data Type'),
              '{:15}'.format('Description'))
        print('{:25}'.format('========================='),
              '{:15}'.format('==============='),
              '{:10}'.format('=========='),
              '{:15}'.format('==============='),
              '{:15}'.format('==============='),
              '{:15}'.format('==============='),
              '{:15}'.format('=============================='))

        for param_name in self.value.keys():
            if self.data_type[param_name] == 'str':
                print('{:25}'.format(param_name),
                      '{:15}'.format(self.value[param_name]),
                      '{:10}'.format(self.unit[param_name]),
                      '{:15}'.format(''),
                      '{:15}'.format(str(self.default[param_name])),
                      '{:15}'.format(self.data_type[param_name]),
                      '{:15}'.format(self.description[param_name]))
            elif self.data_type[param_name] == 'bool':
                print('{:25}'.format(param_name),
                      '{:15}'.format(str(self.value[param_name])),
                      '{:10}'.format(self.unit[param_name]),
                      '{:15}'.format(''),
                      '{:15}'.format(str(self.default[param_name])),
                      '{:15}'.format(self.data_type[param_name]),
                      '{:15}'.format(self.description[param_name]))
            else:
                print('{:25}'.format(param_name),
                      '{:15}'.format(str(self.value[param_name])),
                      '{:10}'.format(self.unit[param_name]),
                      '{:15}'.format('[' + str(self.boundary[param_name][0]) + ',' +
                                     str(self.boundary[param_name][1]) + ']'),
                      '{:15}'.format(str(self.default[param_name])),
                      '{:15}'.format(self.data_type[param_name]),
                      '{:15}'.format(self.description[param_name]))


# 參數輸入輸出模組
class ParamIO:
    """參數輸入輸出模組"""
    def __init__(self):
        # 欄位序號
        self.SN_NAME = 0
        self.SN_VALUE = 1
        self.SN_UNIT = 2
        self.SN_L_BOUND = 3
        self.SN_H_BOUND = 4
        self.SN_DEFAULT = 5
        self.SN_D_TYPE = 6
        self.SN_DESCRIP = 7

        # 參數儲存字典
        self.value = {}
        self.unit = {}
        self.boundary = {}
        self.default = {}
        self.data_type = {}
        self.description = {}

        # 參數檔存放位置
        if platform.platform() == 'macOS-10.16-x86_64-i386-64bit':
            self.__file_address = '/Users/bright/PycharmProjects/TDK25_Final/Parameter.csv'
        else:
            # self.__file_address = 'Parameter.csv'
            self.__file_address = '/home/pi/Desktop/TDK25_Final/Parameter.csv'

        self.read_param_from_csv()

    # 匯入參數檔
    def read_param_from_csv(self, show_param=False):
        with open(self.__file_address, newline='') as csvfile:
            self.reader = csv.reader(csvfile, delimiter=',')
            # 資料抽取與部署
            for row in self.reader:
                # 去除標題列
                if row[self.SN_NAME] == 'Name':
                    pass
                # 逐列(參數)取值
                else:
                    # 依照參數資料格式進行格式化
                    if row[self.SN_D_TYPE] == 'str':
                        self.value[row[self.SN_NAME]] = row[self.SN_VALUE]
                        self.default[row[self.SN_NAME]] = row[self.SN_DEFAULT]

                    elif row[self.SN_D_TYPE] == 'int':
                        self.value[row[self.SN_NAME]] = int(row[self.SN_VALUE])
                        self.default[row[self.SN_NAME]] = int(row[self.SN_DEFAULT])
                        if row[self.SN_L_BOUND] or row[self.SN_H_BOUND]:
                            self.boundary[row[self.SN_NAME]] = (int(row[self.SN_L_BOUND]),
                                                                int(row[self.SN_H_BOUND]))
                    elif row[self.SN_D_TYPE] == 'float':
                        self.value[row[self.SN_NAME]] = float(row[self.SN_VALUE])
                        self.default[row[self.SN_NAME]] = float(row[self.SN_DEFAULT])
                        if row[self.SN_L_BOUND] or row[self.SN_H_BOUND]:
                            self.boundary[row[self.SN_NAME]] = (float(row[self.SN_L_BOUND]),
                                                                float(row[self.SN_H_BOUND]))
                    elif row[self.SN_D_TYPE] == 'bool':
                        self.value[row[self.SN_NAME]] = self.str_to_bool(row[self.SN_VALUE])
                        self.default[row[self.SN_NAME]] = self.str_to_bool(row[self.SN_DEFAULT])

                    self.unit[row[self.SN_NAME]] = row[self.SN_UNIT]
                    self.data_type[row[self.SN_NAME]] = row[self.SN_D_TYPE]
                    self.description[row[self.SN_NAME]] = row[self.SN_DESCRIP]

                # 除錯顯示
                if show_param:
                    print('{:25}'.format(row[self.SN_NAME]), '{:15}'.format(row[self.SN_VALUE]),
                          '{:10}'.format(row[self.SN_UNIT]), '{:40}'.format(row[self.SN_DESCRIP]))

    # 將參數寫入CSV檔
    def write_param_into_csv(self, show_list=False, backup=False, mission_time_str=''):
        # 以備份寫入
        if backup:
            # 參數檔存放位置
            if platform.platform() == 'macOS-10.16-x86_64-i386-64bit':
                backup_address = '/Users/bright/PycharmProjects/TDK25_Final/Log/'+mission_time_str+'_Parameter.csv'
            else:
                # backup_address = 'Log/' + mission_time_str + '_Parameter.csv'
                backup_address = '/home/pi/Desktop/TDK25_Final/Log/' + mission_time_str + '_Parameter.csv'
            with open(backup_address, 'w', newline='') as csvfile:
                self.writer = csv.writer(csvfile, delimiter=',')

                # 組織二維列表
                param_list = [['Name', 'Value', 'Unit', 'Lower Bound', 'Upper Bound',
                               'Default', 'Data Type', 'Description']]
                for param_name in self.value.keys():
                    if self.data_type[param_name] != 'str' and self.data_type[param_name] != 'bool':
                        param_list.append([param_name, self.value[param_name], self.unit[param_name],
                                           self.boundary[param_name][0], self.boundary[param_name][1],
                                           self.default[param_name], self.data_type[param_name],
                                           self.description[param_name]])
                    else:
                        param_list.append([param_name, self.value[param_name], self.unit[param_name],
                                           '', '', self.default[param_name], self.data_type[param_name],
                                           self.description[param_name]])

                # 寫入參數表
                self.writer.writerows(param_list)

                PrintLayout.success('參數表已經備份完成')
                # print('<系統訊息> 參數表已經備份完成')

                # 除錯顯示
                if show_list:
                    print(param_list)
        # 標準寫入
        else:
            with open(self.__file_address, 'w', newline='') as csvfile:
                self.writer = csv.writer(csvfile, delimiter=',')

                # 組織二維列表
                param_list = [['Name', 'Value', 'Unit', 'Lower Bound', 'Upper Bound',
                               'Default', 'Data Type', 'Description']]
                for param_name in self.value.keys():
                    if self.data_type[param_name] != 'str' and self.data_type[param_name] != 'bool':
                        param_list.append([param_name, self.value[param_name], self.unit[param_name],
                                           self.boundary[param_name][0], self.boundary[param_name][1],
                                           self.default[param_name], self.data_type[param_name],
                                           self.description[param_name]])
                    else:
                        param_list.append([param_name, self.value[param_name], self.unit[param_name],
                                           '', '', self.default[param_name], self.data_type[param_name],
                                           self.description[param_name]])

                # 寫入參數表
                self.writer.writerows(param_list)
                print('<系統訊息> 參數表已經寫入完成')

                # 除錯顯示
                if show_list:
                    print(param_list)

    # 更新參數儲存字典
    def update(self, dataset):
        """更新參數儲存字典"""
        self.value = dataset[0]
        self.unit = dataset[1]
        self.boundary = dataset[2]
        self.default = dataset[3]
        self.data_type = dataset[4]
        self.description = dataset[5]

    # 輸出參數儲存字典
    def export(self):
        """輸出參數儲存字典"""
        return self.value, self.unit, self.boundary, self.default, self.data_type, self.description

    # 展示所有參數資訊
    def param_list(self):
        """展示所有參數資訊"""
        print('\n{:39}'.format(''), '==============================================')
        print('{:39}'.format(''), '>>>>>>>>>> TDK25 UAV Parameter List <<<<<<<<<<')
        print('{:39}'.format(''), '==============================================\n')
        print('{:25}'.format('Name'),
              '{:15}'.format('Value'),
              '{:10}'.format('Unit'),
              '{:15}'.format('Bound L-H'),
              '{:15}'.format('Default'),
              '{:15}'.format('Data Type'),
              '{:15}'.format('Description'))
        print('{:25}'.format('========================='),
              '{:15}'.format('==============='),
              '{:10}'.format('=========='),
              '{:15}'.format('==============='),
              '{:15}'.format('==============='),
              '{:15}'.format('==============='),
              '{:15}'.format('=============================='))

        for param_name in self.value.keys():
            if self.data_type[param_name] == 'str':
                print('{:25}'.format(param_name),
                      '{:15}'.format(self.value[param_name]),
                      '{:10}'.format(self.unit[param_name]),
                      '{:15}'.format(''),
                      '{:15}'.format(str(self.default[param_name])),
                      '{:15}'.format(self.data_type[param_name]),
                      '{:15}'.format(self.description[param_name]))
            elif self.data_type[param_name] == 'bool':
                print('{:25}'.format(param_name),
                      '{:15}'.format(str(self.value[param_name])),
                      '{:10}'.format(self.unit[param_name]),
                      '{:15}'.format(''),
                      '{:15}'.format(str(self.default[param_name])),
                      '{:15}'.format(self.data_type[param_name]),
                      '{:15}'.format(self.description[param_name]))
            else:
                print('{:25}'.format(param_name),
                      '{:15}'.format(str(self.value[param_name])),
                      '{:10}'.format(self.unit[param_name]),
                      '{:15}'.format('[' + str(self.boundary[param_name][0]) + ',' +
                                     str(self.boundary[param_name][1]) + ']'),
                      '{:15}'.format(str(self.default[param_name])),
                      '{:15}'.format(self.data_type[param_name]),
                      '{:15}'.format(self.description[param_name]))

    # 將布林字串轉成布林格式
    @staticmethod
    def str_to_bool(string=''):
        if string == 'True' or string == 'TRUE' or string == 'true':
            return True
        else:
            return False


# 偵錯用，更新參數後先執行做測試
if __name__ == '__main__':
    # 建立參數集與存取集
    param = Param()
    param_io = ParamIO()
    param_io.read_param_from_csv(show_param=False)
    print('Param from Param IO Module')
    param_io.param_list()

    # 資料傳遞->參數集
    param.update(param_io.export())
    print('Param from Param Sever Module')
    param.param_list()

    # 資料傳遞->存取集
    param_io.update(param.export())
    param_io.write_param_into_csv(show_list=False)

    # 呼叫範例
    # print(param.value['SENS_ALT_LID_BYTESIZE'])
