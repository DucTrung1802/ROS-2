from threading import Thread
import threading
import time

number = 0


def task_1(numbers):
    global number
    for n in numbers:
        time.sleep(0.3)
        number += 1
        print("task 1: " + str(number))


def task_2(numbers):
    global number
    for n in numbers:
        time.sleep(0.3)
        number += 1
        print("task 2: " + str(number))


def task_3(numbers):
    global number
    for n in numbers:
        time.sleep(0.3)
        number += 1
        print("task 3: " + str(number))


arr = [2, 3, 7, 9]
arg = 3

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
