# Python program to demonstrate
# Conversion of JSON data to
# dictionary


# importing the module
import json

data = """{"people1": "robot 1", "people2":"robot 2"}"""

# Print the type of data variable
print("Type:", type(data))

res = json.loads(data)

# Print the data of dictionary
print("People1:", res['people1'])
print("People2:", res['people2'])
