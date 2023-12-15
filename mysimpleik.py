import numpy as np
from math import cos, sin, atan2, sqrt, pi, acos, degrees

def fwdkin(angles=[0,0], L=[14.5, 14.8]):
    if angles == None:
        return 0
    
    print(angles)
    theta1 = np.deg2rad(angles[0])
    theta2 = np.deg2rad(angles[1])
    L1 = L[0]
    L2 = L[1]
    x_e = round( (L1*cos(theta1) + L2*cos(theta1+theta2)), 1)
    z_e = round( (L1*sin(theta1) + L2*sin(theta1+theta2)), 1)
    
    print(f'coords: ({x_e},{z_e})')
    return [x_e, z_e]

def invkin(coords=[12, 12], L=[14.5, 14.8], servo_1_lim=[10, 150], servo_2_lim=[32, 120]):
    try:

        servo_1_min = servo_1_lim[0]
        servo_1_max = servo_1_lim[1]
        servo_2_min = servo_2_lim[0]
        servo_2_max = servo_2_lim[1]

        x_e = coords[0]
        z_e = coords[1]

        R_end = sqrt((x_e) ** 2 + (z_e) ** 2)
        R_arm = L[0] + L[1]
        if R_end > R_arm:
            # print('location is outside range of arm')
            return None

        L1 = L[0]
        L2 = L[1]
        L3 = R_end

        I = (L1 ** 2 + L2 ** 2 - L3 ** 2) / (2 * L1 * L2)
        J = (L3 ** 2 + L1 ** 2 - L2 ** 2) / (2 * L1 * L3)
        
        theta_2 = np.rad2deg(pi - acos(I))
        theta_1 = np.rad2deg(atan2(z_e,x_e) - acos(J))

        # alt solution?
        theta_2_alt = np.rad2deg(-pi + (acos(I)))
        theta_1_alt = np.rad2deg(atan2(z_e,x_e) - (-acos(J)))
        
        theta_1 = round(theta_1, 1)
        theta_2 = round(theta_2, 1)
        
        servo_1 = round((180 - theta_1), 1)
        servo_2 = round((theta_1-theta_2+90), 1)

        servo_1_alt = round((180 - theta_1_alt), 1)
        servo_2_alt = round((theta_1_alt-theta_2_alt+90), 1)
        #print([theta_1_alt, servo_1_alt])
        #print([theta_2_alt, servo_2_alt])

        if servo_1 > servo_1_max:
            theta_1 = theta_1_alt
            theta_2 = theta_2_alt
            servo_1 = round((180 - theta_1 + 10), 1)
            servo_2 = round((theta_1+theta_2-270+360 + 7), 1)

        return [servo_1, servo_2]

    except Exception as e:
        print(f'value error: {e}')
        return None