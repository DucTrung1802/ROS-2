from openpyxl import Workbook
import os


class DataRecoder(object):
    def __init__(self, work_book_name, sheet_name="Data", data_folder_name="Data"):
        self.__initializeWorkBook(work_book_name, sheet_name, data_folder_name)

    def __initializeWorkBook(self, work_book_name, sheet_name, data_folder_name):
        self.__name = work_book_name + ".xlsx"
        self.__sheet_name = sheet_name
        self.__data_folder_name = data_folder_name
        self.__work_book = Workbook()

    def saveWorkBook(self):
        self.__current_path = os.getcwd()
        try:
            os.makedirs(os.path.join(self.__current_path, self.__data_folder_name))
        except:
            pass
        self.__save_path = os.path.join(self.__data_folder_name, self.__name)
        self.__work_book.save(filename=self.__save_path)



def main():
    # work_book = Workbook(work_book_name="hello")
    # os.makedirs("motor/Data")
    workbook = DataRecoder("hello")
    workbook.saveWorkBook()


if __name__ == "__main__":
    main()

# wb = Workbook()

# dest_filename = "empty_book.xlsx"

# ws1 = wb.active
# ws1.title = "range names"

# ws2 = wb.create_sheet(title="Pi")

# ws2["F5"] = 3.14

# ws3 = wb.create_sheet(title="Data")
# ws3.cell(column=1, row=1, value=1)
# wb.save(filename=dest_filename)
