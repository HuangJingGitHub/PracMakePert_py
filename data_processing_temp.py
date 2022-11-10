from macpath import split
import numpy as np

file = open("single_pt_data_2022-10-26_15_43.txt", "r")

for i in range(3):
    line_str = file.readline()
    print(line_str)

line_str = file.readline()
line_list = line_str.split(" ")
target_S = np.array([])
for i in range(6, 10):
    target_S = np.append(target_S, [float(line_list[i])])

min_err = 1e4
while True:
    line_str = file.readline();

    if not line_str:
        break
    
    line_list = line_str.split(" ")
    current_S = np.array([])
    for i in range(2, 6):
        current_S = np.append(current_S, [float(line_list[i])])
    
    error_S = current_S - target_S
    error_S_norm = np.linalg.norm(error_S)
    if error_S_norm < min_err:
        min_err = error_S_norm
    print(min_err)
