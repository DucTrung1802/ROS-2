from openpyxl import Workbook
from openpyxl.utils import get_column_letter
from pathlib import Path
import os


class DataRecoder(object):
    def __init__(self, pwm, pwm_frequency, sample_time):
        self.__initializeWorkBook()
        self.__preconfigure(pwm, pwm_frequency, sample_time)

    def __preconfigure(self, pwm, pwm_frequency, sample_time):
        self.__name = (
            "Motor_Data_"
            + str(pwm_frequency)
            + "Hz"
            + "_"
            + "Vary_PWM"
            + "_"
            + str(sample_time)
            + "s"
            + ".xlsx"
        )

    def __initializeWorkBook(self):
        self.__work_book = Workbook()
        for i in range(1, 7):
            self.__work_book.active.column_dimensions[get_column_letter(i)].width = 30
        self.writeData(1, 1, "MOTOR 1 NO KF")
        self.writeData(1, 2, "MOTOR 1 WITH KF")
        self.writeData(1, 4, "MOTOR 2 NO KF")
        self.writeData(1, 5, "MOTOR 2 WITH KF")
        self.writeData(1, 6, "Checksum")

    def saveWorkBook(self):
        # self.__folder_name = os.path.dirname(os.path.realpath(__file__))
        self.__save_path = os.path.join(Path.cwd(), self.__name)
        self.__work_book.save(filename=self.__save_path)
        print(self.__save_path)

    def writeData(self, row, col, data):
        self.__work_book.active.cell(row=row, column=col, value=data)


def main():
    i = 0
    workbook = DataRecoder("Motor_Data")
    workbook.configure(1023, 1000, 0.05)
    workbook.writeData(i + 5, 1, 10)
    workbook.saveWorkBook()


if __name__ == "__main__":
    main()
