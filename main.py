import robotica
from controller import *
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser
import numpy as np
import matplotlib.pyplot as plt
import pybreezyslam

MAP_SIZE_M = 20
MAP_SIZE = int(MAP_SIZE_M / 0.01)
map_array = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.uint8)

mapbytes = bytearray(MAP_SIZE * MAP_SIZE)

laser = Laser(684, 10, 240, 4000, 60, 15)
slam = RMHC_SLAM(laser, MAP_SIZE, MAP_SIZE_M,
                 hole_width_mm=200,
                 max_search_iter=10000,
                 random_seed=42
                 )

init_x = (-3000 + 5000)
init_y = (-1475 + 5000)
slam.position = pybreezyslam.Position(init_x, init_y, 0)

plt.ion()
fig, ax = plt.subplots()
img = ax.imshow(map_array, cmap='gray', vmin=0, vmax=255)
plt.title("Mapa generado por BreezySLAM")
plt.axis('off')


def run_robot():
    coppelia = robotica.Coppelia()
    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX')
    controller = create_fuzzy_controller()

    try:
        coppelia.start_simulation()
        while coppelia.is_running():
            sonar_readings = robot.get_sonar()
            print([min(sonar_readings[0:3]), min(sonar_readings[3:5]),
                   min(sonar_readings[5:8])])

            left_distance = min(sonar_readings[0:3]) * 1000  # Convert to mm
            front_distance = min(sonar_readings[2:6]) * 1000  # Convert to mm
            right_distance = min(sonar_readings[5:8]) * 1000  # Convert to mm

            # Pass distances to the fuzzy controller
            controller.input['left_distance'] = left_distance
            controller.input['front_distance'] = front_distance
            controller.input['right_distance'] = right_distance

            position = robot.get_position()
            controller.compute()

            # Get the computed wheel speeds
            left_speed = controller.output['left_speed']
            right_speed = controller.output['right_speed']

            print(f'L: {left_speed}, R: {right_speed}\n')

            # Set robot speeds
            robot.set_speed(left_speed, right_speed)
            # END TEST

            lidar_readings = [x * 1000 for x in robot.get_lidar_data()]
            print(max(lidar_readings))

            slam.update(
                lidar_readings,
                pose_change=position,
                should_update_map=True
            )

            slam.getmap(mapbytes)
            map_array = np.array(mapbytes).reshape((MAP_SIZE, MAP_SIZE))
            img.set_data(np.flipud(map_array))
            plt.draw()
            plt.pause(0.1)
    except:
        print('ignoring exception')
    finally:
        coppelia.stop_simulation()


if __name__ == '__main__':
    run_robot()
    plt.ioff()
