import numpy as np
import scipy.io
from os import listdir
from os.path import isfile, join

data_path = 'D:/Files/Research/Manipulation/Nonfixed Contact/Experiments/data/0501/'
mat_save_path = 'D:/Files/Research/Manipulation/Nonfixed Contact/Experiments/data/0501/mat/'
data_file_list = [f for f in listdir(data_path) if isfile(join(data_path, f))]
print(data_file_list)

for data_file in data_file_list:
    data_file_path = data_path + data_file
    postfix = data_file[data_file.rfind('_') : data_file.rfind('.')]

    data_np = np.load(data_file_path)
    scipy.io.savemat(mat_save_path + 'pt' + postfix + '.mat', {'pt' + postfix: data_np})