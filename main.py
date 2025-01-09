import robotica
from controller import FuzzyController
from breezyslam.algorithms import RMHC_SLAM
from breezyslam.sensors import Laser
from PIL import Image
import numpy as np
import matplotlib.pyplot as plt

# Configurar la visualización
plt.ion()  # Activar modo interactivo
fig, ax = plt.subplots()
map_array = np.zeros((800, 800), dtype=np.uint8)
img = ax.imshow(map_array, cmap='gray', vmin=0, vmax=255)
plt.title("Mapa generado por BreezySLAM")
plt.axis('off')


class MyLaser(Laser):
    def __init__(self):
        super().__init__(
            scan_size=684, #542, #684
            scan_rate_hz=15,
            detection_angle_degrees=270,
            distance_no_detection_mm=100,
            detection_margin=0,
            offset_mm=0
        )


mapbytes = bytearray(800*800)
slam = RMHC_SLAM(MyLaser(), 800, 5)


def run_robot():
    coppelia = robotica.Coppelia()
    robot = robotica.P3DX(coppelia.sim, 'PioneerP3DX')
    controller = FuzzyController()

    try:
        coppelia.start_simulation()
        while coppelia.is_running():
            sonar_readings = robot.get_sonar()
            lidar_readings = robot.get_lidar_data()
            # position = robot.get_position()
            # print(position)
            print(len(lidar_readings))

            slam.update(
                lidar_readings,
                should_update_map=True
            )

            x, y, theta = slam.getpos()
            print(x, y, theta)

            left_speed, right_speed = controller.compute_movement(
                sonar_readings
            )
            robot.set_speed(left_speed, right_speed)

            slam.getmap(mapbytes)
            # Convertir mapbytes a una matriz 2D y actualizar la visualización
            map_array = np.array(mapbytes).reshape((800, 800))
            img.set_data(map_array)
            plt.draw()
            plt.pause(0.1)
    finally:
        coppelia.stop_simulation()


if __name__ == '__main__':
    run_robot()
