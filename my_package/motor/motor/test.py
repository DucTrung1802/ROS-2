import copy
dict_A = {"key":"abcd1234"}
dict_B = copy.deepcopy(dict_A)
dict_B["hello"] = "A"
print(dict_A)