import numpy as np 
from numpy import sin,cos

l0 = 0.143
l1,l2,l3=0.155,0.137,0.218
a=0.033
fkdict = {
    '01' : lambda theta1: np.array([[cos(theta1), -sin(theta1), 0, 0],
                                   [sin(theta1), cos(theta1), 0, 0 ],
                                   [0, 0, 1, l0],
                                   [0, 0, 0, 1]]) ,
    '12' : lambda theta2: np.array([[-sin(theta2), -cos(theta2), 0, a],
                                   [0, 0, -1, 0 ],
                                   [cos(theta2), -sin(theta2), 0, 0],
                                   [0, 0, 0, 1]]) ,
    '23' : lambda theta3: np.array([[cos(theta3), -sin(theta3), 0, l1],
                                   [sin(theta3), cos(theta3), 0, 0 ],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]]) ,
    '34' : lambda theta4: np.array([[-sin(theta4), -cos(theta4), 0, l2],
                                   [cos(theta4), -sin(theta4), 0, 0 ],
                                   [0, 0, 1, 0],
                                   [0, 0, 0, 1]]) ,
    '45' : lambda theta5: np.array([[cos(theta5), -sin(theta5), 0, 0],
                                   [0, 0, -1, 0 ],
                                   [ sin(theta5),cos(theta5), 0, 0],
                                   [0, 0, 0, 1]]) ,
    '5e' : lambda theta5: np.array([[1, 0, 0, 0],
                                   [0, 1, 0, 0 ],
                                   [ 0, 0, 1, l3],
                                   [0, 0, 0, 1]]) ,
}

print(fkdict['01'](0.1))