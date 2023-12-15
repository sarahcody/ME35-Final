import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata
from matplotlib.colors import Normalize
from math import cos, sin, atan2, sqrt, pi, acos, degrees
from mysimpleik import invkin

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
            return 0

        L1 = L[0]
        L2 = L[1]
        L3 = R_end

        I = (L1 ** 2 + L2 ** 2 - L3 ** 2) / (2 * L1 * L2)
        theta_2 = np.rad2deg(pi - acos(I))

        J = (L3 ** 2 + L1 ** 2 - L2 ** 2) / (2 * L1 * L3)
        theta_1 = np.rad2deg(atan2(z_e , x_e) - acos(J))

        # alt solution?
        theta_2_alt = -np.rad2deg(pi - (acos(I)))
        theta_1_alt = np.rad2deg(atan2(z_e , x_e) - (-acos(J)))

        theta_1 = round(theta_1, 1)
        theta_2 = round(theta_2, 1)

        servo_1 = round((180 - theta_1), 1)
        servo_2 = round((180 + theta_2), 1)

        # error handling
        if (
            servo_1 > servo_1_max
            or servo_2 > servo_2_max
            or servo_1 < servo_1_min
            or servo_2 < servo_2_min
        ):
            theta_1 = theta_1_alt
            theta_2 = theta_2_alt
            servo_1 = round((180 - theta_1), 1)
            servo_2 = round((theta_2 + 180), 1)
            theta_1_alt = np.nan
            theta_2_alt = np.nan
            # print(f'Calculated servo angle is outside physical bounds... \n Trying servo angles ({servo_1}, {servo_2})')

        if (
            servo_1 > servo_1_max
            or servo_2 > servo_2_max
            or servo_1 < servo_1_min
            or servo_2 < servo_2_min
        ):
            # print(f'SECONDARY SOLUTION FAILED')
            return 1

        # print(f'RAW ANGLES: \n Theta_1 = {theta_1} \n Theta_2 = {theta_2}')
        
        return [theta_1, theta_2, theta_1_alt, theta_2_alt]

    except ValueError as e:
        # print(f'value error: {e}')
        return [-1,-1,-1,-1]


def plot_arm_heatmap(L, servo_1_lim, servo_2_lim, grid_resolution=0.25):
    arm_radius = sum(L)
    x_range = np.arange(-arm_radius, arm_radius + grid_resolution, grid_resolution)
    z_range = np.arange(-arm_radius, arm_radius + grid_resolution, grid_resolution)

    arm_grid = np.zeros((len(x_range), len(z_range)))

    for i, x in enumerate(x_range):
        for j, z in enumerate(z_range):
            coords = [x, z]
            angles = invkin(coords, L, servo_1_lim, servo_2_lim)

            if angles == np.nan:
                arm_grid[i, j] = 1  # Blue for weird points
            elif angles == 0:
                arm_grid[i, j] = 5  # No marker for can't reach
            elif angles == 1:
                arm_grid[i, j] = 7  # Red for servo limited
            else:
                # Check if secondary solution is used
                if np.isnan(angles[2]) or np.isnan(angles[3]):
                    arm_grid[i, j] = 3  # Purple for secondary solution
                else:
                    arm_grid[i, j] = 2  # Green for valid points

    cmap = plt.cm.get_cmap('Paired', 12)  # Use the viridis colormap with 4 colors
    norm = Normalize(vmin=0, vmax=12)
    img = plt.imshow(
        arm_grid, extent=[-arm_radius, arm_radius, -arm_radius, arm_radius], cmap=cmap, origin='lower', norm=norm
    )
    # Create a legend
    legend_labels = {
        1: 'Weird Points',
        5: 'Can\'t Reach',
        7: 'Servo Limited',
        3: 'Secondary Solution Valid',
        2: 'Initial Solution Valid',
    }
    legend_elements = [plt.Line2D([0], [0], marker='o', color='w', markerfacecolor=img.cmap(img.norm(key)), markersize=10, label=legend_labels[key]) for key in legend_labels]

    plt.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(1, 1))
    
    plt.xlabel('X-axis')
    plt.ylabel('Z-axis')
    plt.title('Arm Workspace Heatmap')
    plt.show()


# Example usage
L = [14.5, 14.8]
servo_1_lim=[10, 150]
servo_2_lim=[32, 120]
plot_arm_heatmap(L, servo_1_lim, servo_2_lim)

    