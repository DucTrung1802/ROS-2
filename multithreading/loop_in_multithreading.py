import threading
import time

number = 0


def task_1(numbers):
    global number
    while True:
        start = time.time()
        time.sleep(0.005)
        number += 1
        # print("task 1: " + str(number))
        end = time.time()
        print("Time: " + str(end - start))


def task_2(numbers):
    global number
    while True:
        start = time.time()
        time.sleep(0.004)
        number += 1
        # print("task 1: " + str(number))
        end = time.time()
        print("Time: " + str(end - start))


def task_3(numbers):
    global number
    while True:
        start = time.time()
        time.sleep(0.003)
        number += 1
        # print("task 1: " + str(number))
        end = time.time()
        print("Time: " + str(end - start))


arr = [2, 3, 7, 9]
arg = 3


def main():
    try:
        t = time.time()
        t1 = threading.Thread(target=task_1, args=(arr,))
        t2 = threading.Thread(target=task_2, args=(arr,))
        t3 = threading.Thread(target=task_3, args=(arr,))
        t1.start()
        t2.start()
        t3.start()
        t1.join()
        t2.join()
        t3.join()
        print("done in ", time.time() - t)
    except:
        print("error")


if __name__ == "__main__":
    main()
