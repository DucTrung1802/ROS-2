import math
import timeit

def is_prime(number):
    if number == 2:
        return True
    if number <= 1 or not number % 2:
        return False
    max_range = int(math.sqrt(number)) + 1
    for div in range(3, max_range, 2):
        if not number % div:
            return False
    return True


def run_program(max_number):
    for number in range(max_number):
        is_prime(number)


if __name__ == '__main__':
    max_number = 10000000
    start = timeit.default_timer()
    run_program(max_number)
    stop = timeit.default_timer()
    print(stop - start, " (seconds)")