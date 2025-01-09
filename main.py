import robotica
from controller import FuzzyController
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

MAP_SIZE = 4000

plt.ion()
fig, ax = plt.subplots()
map_array = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.uint8)
img = ax.imshow(map_array, cmap='gray', vmin=0, vmax=255)
plt.title("SLAM")
plt.axis('off')


class MyLaser(Laser):
    def __init__(self):
        super().__init__(
            scan_size=684,
            scan_rate_hz=10,
            detection_angle_degrees=240,
            distance_no_detection_mm=4000,
            detection_margin=5,
            offset_mm=1
        )


mapbytes = bytearray(MAP_SIZE*MAP_SIZE)
slam = RMHC_SLAM(MyLaser(), MAP_SIZE, 30)

df_lidar = pd.DataFrame()


def run_robot():
    coppelia = robotica.Coppelia()
    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX')
    controller = FuzzyController()

    try:
        coppelia.start_simulation()

        while coppelia.is_running():
            sonar_readings = robot.get_sonar()
            lidar_readings = robot.get_lidar_data()

            lidar_readings = [x * 1000 for x in lidar_readings]
            position = robot.get_position()

            slam.update(
                lidar_readings,
                pose_change=position,
                should_update_map=True
            )

            left_speed, right_speed = controller.compute_movement(
                sonar_readings
            )
            robot.set_speed(left_speed, right_speed)

            slam.getmap(mapbytes)

            map_built_array = np.array(mapbytes).reshape((MAP_SIZE, MAP_SIZE))
            img.set_data(map_built_array)
            plt.draw()
            plt.pause(0.1)
    finally:
        map_built_array = np.array(mapbytes).reshape((MAP_SIZE, MAP_SIZE))
        plt.imsave('map.png', map_built_array, cmap='gray')
        coppelia.stop_simulation()


if __name__ == '__main__':
    run_robot()
