from openpyxl import Workbook
from openpyxl.utils import get_column_letter
import os


class DataRecoder(object):
    def __init__(self, work_book_name, sheet_name="Data", data_folder_name="Data"):
        self.__initializeWorkBook(work_book_name, sheet_name, data_folder_name)
        self.__preconfigure()

    def __initializeWorkBook(self, work_book_name, sheet_name, data_folder_name):
        self.__name = work_book_name + ".xlsx"
        self.__sheet_name = sheet_name
        self.__data_folder_name = data_folder_name
        self.__work_book = Workbook()

    def __preconfigure(self):
        for i in range(1, 4):
            self.__work_book.active.column_dimensions[get_column_letter(i)].width = 25
        self.writeData(1, 1, "PWM (10 bits)")
        self.writeData(2, 1, "PWM frequency (Hz)")
        self.writeData(3, 1, "Sample time (s)")
        self.writeData(5, 1, "RPM Motor 1")
        self.writeData(5, 3, "RPM Motor 2")

    def saveWorkBook(self):
        self.__current_path = os.getcwd()
        try:
            os.makedirs(os.path.join(self.__current_path, self.__data_folder_name))
        except:
            pass
        self.__save_path = os.path.join(self.__data_folder_name, self.__name)
        self.__work_book.save(filename=self.__save_path)

    def writeData(self, row, col, data):
        self.__work_book.active.cell(row=row, column=col, value=data)

    def configure(self, pwm, pwm_frequency, sample_time):
        self.writeData(1, 2, pwm)
        self.writeData(2, 2, pwm_frequency)
        self.writeData(3, 2, sample_time)


def main():
    workbook = DataRecoder("Motor_Data")
    workbook.configure(1023, 1000, 0.05)
    workbook.saveWorkBook()


if __name__ == "__main__":
    main()