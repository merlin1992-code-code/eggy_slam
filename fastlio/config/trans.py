'''
Description: Do not Edit
Author: hao.lin (voyah perception)
Date: 2025-06-26 15:24:23
LastEditors: Do not Edit
LastEditTime: 2025-06-26 18:13:16
'''
import numpy as np

T = np.array([
    [ 1.02622291e-03,  9.99813379e-01,  1.92912927e-02,  1.03000000e+00],
    [-9.99993653e-01,  1.09184778e-03, -3.39156230e-03,  0.00000000e+00],
    [-3.41199252e-03, -1.92876897e-02,  9.99808153e-01,  2.06680700e+00],
    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
])
R = T[:3, :3]
t = T[:3, 3]
print("r_il:", R.flatten())
print("t_il:", t)