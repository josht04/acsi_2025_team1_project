"""
Crazyflie trajectory control for pendulum ball catching.
Uses MotionCommander for smooth relative movements.
"""
import logging
import time
#import pandas 

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

URI = 'radio://0/80/2M'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

# Original sequence in millimeters (x, y, z, yaw)

#df = pd.read_excel("Traj.xlsx")
#df = df.astype(float).round(3)
#sequence = [tuple(float(x) for x in row) for row in df.to_numpy()]

sequence_mm = [
    (0.0, 0.0, 1000.0, 0),
    (0.0, 0.0, 1000.0, 0),
    (0.0, 0.0, 999.847515, 0),
    (0.0, 0.0, 999.566358, 0),
    (0.0, -0.652312, 999.177929, 0),
    (0.0, -2.387549, 998.677766, 0),
    (0.0, -5.308278, 998.062847, 0),
    (0.0, -9.327619, 997.348279, 0),
    (0.0, -14.222069, 996.555229, 0),
    (0.0, -19.631471, 995.684362, 0),
    (0.0, -25.094016, 994.707433, 0),
    (0.0, -30.145084, 993.58986, 0),
    (0.0, -34.371026, 992.324935, 0),
    (0.0, -37.457961, 990.946063, 0),
    (0.0, -39.217269, 989.51581, 0),
    (0.0, -39.593275, 988.117636, 0),
    (0.0, -38.641669, 986.861693, 0),
    (0.0, -36.483225, 985.880885, 0),
    (0.0, -33.29182, 985.279427, 0),
    (0.0, -29.319238, 985.066969, 0),
    (0.0, -24.933891, 985.148292, 0),
    (0.0, -20.585384, 985.376961, 0),
    (0.0, -16.726927, 985.636216, 0),
    (0.0, -13.759369, 985.895291, 0),
    (0.0, -12.02395, 986.206904, 0),
    (0.0, -11.813541, 986.652145, 0),
    (0.0, -13.369242, 987.270486, 0),
    (0.0, -16.849216, 988.015467, 0),
    (0.0, -22.280173, 988.756867, 0),
    (0.0, -29.521029, 989.323162, 0),
    (0.0, -38.25987, 989.558745, 0),
    (0.0, -48.039429, 989.367661, 0),
    (0.0, -58.293163, 988.732196, 0),
    (0.0, -68.388572, 987.716597, 0),
    (0.0, -77.691154, 986.468634, 0),
    (0.0, -85.640133, 985.21432, 0),
    (0.0, -91.809822, 984.228191, 0),
    (0.0, -95.889574, 983.779913, 0),
    (0.0, -97.646645, 984.062591, 0),
    (0.0, -96.925155, 985.090768, 0),
    (0.0, -93.639186, 986.645001, 0),
    (0.0, -87.812325, 988.273189, 0),
    (0.0, -79.659197, 989.366584, 0),
    (0.0, -69.660333, 989.319304, 0),
    (0.0, -58.617181, 987.731315, 0),
    (0.0, -47.589493, 984.583974, 0),
    (0.0, -37.62653, 980.304988, 0),
    (0.0, -29.566565, 975.674349, 0),
    (0.0, -23.950239, 971.625243, 0),
    (0.0, -21.053072, 969.027169, 0),
    (0.0, -21.001791, 968.510777, 0),
    (0.0, -23.935557, 970.347875, 0),
    (0.0, -30.09739, 974.366976, 0),
    (0.0, -39.811736, 979.894538, 0),
    (0.0, -53.302388, 985.752547, 0),
    (0.0, -70.357163, 990.38444, 0),
    (0.0, -90.080323, 992.204442, 0),
    (0.0, -110.823322, 990.132196, 0),
    (0.0, -130.505879, 984.107644, 0),
    (0.0, -147.24701, 975.260703, 0),
    (0.0, -159.954273, 965.570841, 0),
    (0.0, -168.492227, 957.213716, 0),
    (0.0, -173.36115, 951.992852, 0),
    (0.0, -175.146172, 951.085908, 0),
    (0.0, -174.025511, 955.032459, 0),
    (0.0, -169.474888, 963.747178, 0),
    (0.0, -160.359171, 976.380314, 0),
    (0.0, -145.255554, 991.067998, 0),
    (0.0, -123.170551, 1004.779027, 0),
    (0.0, -94.548515, 1013.659719, 0),
    (0.0, -62.028341, 1014.243353, 0),
    (0.0, -30.107716, 1005.208122, 0),
    (0.0, -3.406498, 988.477193, 0),
    (0.0, 15.596338, 968.477983, 0),
    (0.0, 27.333996, 949.980354, 0),
    (0.0, 34.173714, 936.285225, 0),
    (0.0, 38.967572, 928.893569, 0),
    (0.0, 43.961718, 928.24827, 0),
    (0.0, 50.114063, 934.521178, 0),
    (0.0, 56.986995, 948.024412, 0),
    (0.0, 62.733632, 969.088921, 0),
    (0.0, 64.095645, 997.440297, 0),
    (0.0, 56.76443, 1031.064032, 0),
    (0.0, 36.742466, 1064.923473, 0),
    (0.0, 3.038758, 1090.795556, 0),
    (0.0, -39.173957, 1100.000536, 0),
    (0.0, -78.720402, 1089.021846, 0),
    (0.0, -104.460596, 1063.322968, 0),
    (0.0, -112.488787, 1033.858983, 0),
    (0.0, -106.677544, 1009.483589, 0),
    (0.0, -93.56851, 993.433297, 0),
    (0.0, -78.238548, 984.93909, 0),
    (0.0, -63.377748, 981.866881, 0),
    (0.0, -49.984543, 982.192876, 0),
    (0.0, -38.22067, 984.427631, 0),
    (0.0, -27.963718, 987.57593, 0),
    (0.0, -19.081461, 990.986751, 0),
    (0.0, -11.567362, 994.221867, 0),
    (0.0, -5.630514, 996.964014, 0),
    (0.0, -1.683866, 998.957617, 0),
    (0.0, -0.000424, 1000.000832, 0),
]

# Subsample every 5th point and convert to meters
def subsample_trajectory(sequence, step=5):
    subsampled = sequence[::step]
    # Convert from mm to meters
    return [(x/1000.0, y/1000.0, z/1000.0, yaw) for x, y, z, yaw in subsampled]


if __name__ == '__main__':
    # Initialize the low-level drivers
    cflib.crtp.init_drivers(enable_debug_driver=False)

    # Subsample the trajectory
    sequence = subsample_trajectory(sequence_mm, step=5)
    
    print(f'Trajectory has {len(sequence)} waypoints')

    with SyncCrazyflie(URI) as scf:
        # Arm the Crazyflie
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)

        with MotionCommander(scf) as mc:
            print('Taking off!')
            
            # Start at first waypoint position (should be around 1.0m)
            first_z = sequence[0][2]
            print(f'Moving to starting height: {first_z}m')
            mc.up(first_z - 0.3)  # Assuming default takeoff is ~0.3m
            time.sleep(1)

            # Execute trajectory
            prev_x, prev_y, prev_z, prev_yaw = sequence[0]
            
            for i, (x, y, z, yaw) in enumerate(sequence[1:], 1):
                # Calculate relative movements
                delta_x = x - prev_x
                delta_y = y - prev_y
                delta_z = z - prev_z
                
                # Execute movements
                if abs(delta_y) > 0.001:  # More than 1mm movement
                    if delta_y > 0:
                        print(f'Step {i}: Moving right {abs(delta_y):.3f}m')
                        mc.right(abs(delta_y))
                    else:
                        print(f'Step {i}: Moving left {abs(delta_y):.3f}m')
                        mc.left(abs(delta_y))
                
                if abs(delta_z) > 0.001:  # More than 1mm movement
                    if delta_z > 0:
                        mc.up(abs(delta_z))
                    else:
                        mc.down(abs(delta_z))
                
                # Small pause between movements
                time.sleep(0.1)
                
                # Update previous position
                prev_x, prev_y, prev_z, prev_yaw = x, y, z, yaw

            print('Trajectory complete! Landing...')