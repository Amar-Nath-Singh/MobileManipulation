# import dh_parameters
import numpy as np 
from numpy import sin,cos
from numpy import pi as PI

def hmat(theta,alpha,rn,dn): 
    h = np.array([[cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), rn*cos(theta)],
                  [sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), rn*sin(theta)],
                  [0, sin(alpha), cos(alpha), dn],
                  [0, 0, 0, 1]])
    return h 

def transform(theta1,theta2,theta3,theta4,theta5):
    alpha1,alpha2,alpha3,alpha4,alpha5 = PI/2, 0, 0, PI/2, 0
    rn1,rn2,rn3,rn4,rn5 = 0.033, 0.155, 0.135, 0, 0
    dn1,dn2,dn3,dn4,dn5 = 0.147, 0, 0, 0, 0.218
    h1 = hmat(theta1,alpha1,rn1,dn1)
    h2 = hmat(theta2,alpha2,rn2,dn2)
    h3 = hmat(theta3,alpha3,rn3,dn3)
    h4 = hmat(theta4,alpha4,rn4,dn4)
    h5 = hmat(theta5,alpha5,rn5,dn5)
    return h1@h2@h3@h4@h5


def transform2(theta1,theta2,theta3,theta4,theta5):
    a = 0
    l0 = 0.143
    l1,l2,l3=0.155,0.137,0.218
    a=0.033

    to = np.array([[cos(theta1),sin(theta1),0,0],
                   [-sin(theta1), cos(theta1), 0, 0],
                   [0, 0, 1, -l0],
                   [0, 0, 0, 1]])
    h = np.array([[-cos(theta2+theta3+theta4)*cos(theta5), cos(theta2+theta3+theta4)*sin(theta5), -sin(theta2+theta3+theta4), a-l1*sin(theta2)-l2*sin(theta2+theta3)-l3*sin(theta2+theta3+theta4)],
                  [-sin(theta5), -cos(theta5), 0 , 0],
                  [-cos(theta5)*sin(theta2+theta3+theta4), sin(theta5)*sin(theta2+theta3+theta4), cos(theta2+theta3+theta4), l1*cos(theta2)+l2*cos(theta2+theta3)+l3*cos(theta2+theta4+theta4)],
                  [0, 0, 0, 1]])
    return np.linalg.inv(to)@h+ +np.array([[0,0,0,0],[0,0,0,0],[0,0,0,0.728-0.653],[0,0,0,0]])


import numpy as np

def invkin(x, y, z):
    l0 = 0.143  # Check Link Lengths
    l1 = 0.155
    l2 = 0.137
    l3 = 0.218

    Te = np.array([[-1, 0, 0, x],
                   [0, 1, 0, y],
                   [0, 0, -1, z],
                   [0, 0, 0, 1]])
    print("Te:", Te)
    px = Te[0, 3]
    py = Te[1, 3]
    pz = Te[2, 3]
    ax = Te[0, 2]
    ay = Te[1, 2]
    az = Te[2, 2]
    ox = Te[0, 1]
    oy = Te[1, 1]
    oz = Te[2, 1]
    nx = Te[0, 0]
    ny = Te[1, 0]
    nz = Te[2, 0]

    q11 = np.arctan2(px, py)
    q12 = np.arctan2(-px, -py)
    q51 = np.arctan2(np.sin(q11)*nx - np.cos(q11)*ox, np.sin(q11)*ny - np.cos(q11)*oy)
    q52 = np.arctan2(np.sin(q12)*nx - np.cos(q12)*ox, np.sin(q12)*ny - np.cos(q12)*oy)

    sum234_1 = np.arctan2(az, np.sin(q11)*ay - np.cos(q11)*ax)
    sum234_2 = np.arctan2(az, np.sin(q12)*ay - np.cos(q12)*ax)
    solset = [(q11, q51, sum234_1), (q12, q52, sum234_2)]
    for i in solset:
        if int(-nx*np.cos(i[0])*np.sin(i[2])-ny*np.sin(i[0])*np.sin(i[2])+nz*np.cos(i[2]))==0 and int(-ox*np.cos(i[0])*np.sin(i[2])-ny*np.sin(i[0])*np.sin(i[2])+nz*np.cos(i[2]))==0:
            print('valid')
    

    dx_1 = px/np.cos(q11) - l3*np.sin(sum234_1)
    dz_1 = pz - l0 + l3*np.cos(sum234_1)
    dx_2 = px/np.cos(q12) - l3*np.sin(sum234_2)
    dz_2 = pz - l0 + l3*np.cos(sum234_2)
    print((l1-l2)**2,(dx_1**2+dz_1**2),(l1+l2)*2)
    q31 = (l1**2 + l2**2 - dx_1**2 - dz_1**2)/(2*l1*l2)
    q32 = (l1**2 + l2**2 - dx_2**2 - dz_2**2)/(2*l1*l2)

    sol1 = sol2 = False
    if dx_1**2 + dz_1**2 >= (l1-l2)**2 and dx_1**2 + dz_1**2 <= (l1+l2)**2:
        sol1 = True
        print("Valid IK for 1") 
    if dx_2**2 + dz_2**2 >= (l1-l2)**2 and dx_2**2 + dz_2**2 <= (l1+l2)**2:
        sol2 = True
        print("Valid IK for 2")

    phi_1 = np.arctan2(dx_1, dz_1)
    if (dx_1**2 + dz_1**2 + l1**2 - l2**2)/(2*l1*np.sqrt(dx_1**2 + dz_1**2))>1 or (dx_1**2 + dz_1**2 + l1**2 - l2**2)/(2*l1*np.sqrt(dx_1**2 + dz_1**2))<-1:
        print('cant use 1')
    else:
        beta_1 = np.arccos((dx_1**2 + dz_1**2 + l1**2 - l2**2)/(2*l1*np.sqrt(dx_1**2 + dz_1**2)))
    phi_2 = np.arctan2(dx_2, dz_2)
    if (dx_2**2 + dz_2**2 + l1**2 - l2**2)/(2*l1*np.sqrt(dx_2**2 + dz_2**2))>1 or (dx_2**2 + dz_2**2 + l1**2 - l2**2)/(2*l1*np.sqrt(dx_2**2 + dz_2**2))<-1:
        print('cant use 2')
    else:
        beta_2 = np.arccos((dx_2**2 + dz_2**2 + l1**2 - l2**2)/(2*l1*np.sqrt(dx_2**2 + dz_2**2)))

    if q31 > 0:
        q21 = phi_1 - beta_1 
    elif q31 < 0:
        q21 = phi_1 + beta_1

    if q32 > 0:
        q22 = phi_2 - beta_2 
    elif q32 < 0:
        q22 = phi_2 + beta_2

    q41 = (sum234_1) - q21 - q31
    q42 = (sum234_2) - q22 - q32

    v1 = np.array([-np.cos(q11)*np.cos(sum234_1), -np.sin(q11)*np.sin(sum234_1), np.cos(sum234_1)])
    v2 = np.array([-np.cos(q12)*np.cos(sum234_2), -np.sin(q12)*np.sin(sum234_2), np.cos(sum234_2)])
    vv = [v1[1]*v2[2] - v1[2]*v2[1], v1[2]*v2[0] - v1[0]*v2[2], v1[0]*v2[1] - v1[1]*v2[0]]

    print("vv:", vv)
    print("vv x [nx, ny, nz]:", [vv[1]*nx - vv[2]*ny, vv[2]*nx - vv[0]*nz, vv[0]*ny - vv[1]*nx])
    print("vv x [ox, oy, oz]:", [vv[1]*ox - vv[2]*oy, vv[2]*ox - vv[0]*oz, vv[0]*oy - vv[1]*ox])

    if np.all([vv[1]*nx - vv[2]*ny, vv[2]*nx - vv[0]*nz, vv[0]*ny - vv[1]*nx] == 0) or \
       np.all([vv[1]*ox - vv[2]*oy, vv[2]*ox - vv[0]*oz, vv[0]*oy - vv[1]*ox] == 0):
        print("Multiple solution group exists")
        if sol1 and sol2:
            return [[q11, q21, q31, q41], [q12, q22, q32, q42]]
        elif sol1:
            return [q11, q21, q31, q41]
        elif sol2:
            return [q12, q22, q32, q42]
    elif sol1:
        return [q11, q21, q31, q41]
    elif sol2:
        return [q12, q22, q32, q42]

# Example usage:
# invkin(x_val, y_val, z_val)


# print(invkin(5,2,3))



   


    
# print(transform2(0,0,0,0,0))