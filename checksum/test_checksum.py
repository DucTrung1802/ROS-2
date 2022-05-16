# Python 3 code to demonstrate the
# working of MD5 (string - hexadecimal)

import hashlib
import json

motor_data = {"left_RPM":1.401298464e-44,"right_RPM":1.401298464e-44}
motor_data_string = json.dumps(motor_data)
motor_data_string = motor_data_string.replace(" ", "")
print(motor_data_string)
# initializing string
str2hash = "hello world"

# encoding GeeksforGeeks using encode()
# then sending to md5()
result = hashlib.md5(motor_data_string.encode())

# printing the equivalent hexadecimal value.
print("The hexadecimal equivalent of hash is : ", end ="")
print(result.hexdigest())

print(json.loads(motor_data_string)["left_RPM"])
