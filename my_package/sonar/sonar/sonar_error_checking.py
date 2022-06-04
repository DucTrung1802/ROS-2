import time

start = time.time()

target_time = time.time() + 0.1

i = 0

while i <= 10000000 and time.time() <= target_time:
    i += 1
    end = time.time()

print(i)
print(end - start)
