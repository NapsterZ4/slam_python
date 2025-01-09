"""
robotica.py

Provides the communication between CoppeliaSim robotics simulator and
external Python applications via the ZeroMQ remote API.

Copyright (C) 2024 Javier de Lope

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import time
import json
from coppeliasim_zmqremoteapi_client import RemoteAPIClient


class Coppelia:

    def __init__(self):
        self.default_idle_fps = None
        print('*** connecting to coppeliasim')
        client = RemoteAPIClient()
        self.sim = client.getObject('sim')

    def start_simulation(self):
        self.default_idle_fps = self.sim.getInt32Param(
            self.sim.intparam_idle_fps
        )
        self.sim.setInt32Param(self.sim.intparam_idle_fps, 0)
        self.sim.startSimulation()

    def stop_simulation(self):
        self.sim.stopSimulation()
        while self.sim.getSimulationState() != self.sim.simulation_stopped:
            time.sleep(0.1)
        self.sim.setInt32Param(self.sim.intparam_idle_fps,
                               self.default_idle_fps)
        print('*** done')

    def is_running(self):
        return self.sim.getSimulationState() != self.sim.simulation_stopped


class P3DX:
    num_sonar = 16
    sonar_max = 1.0

    def __init__(self, sim, robot_id):
        self.sim = sim
        print('*** getting handles', robot_id)
        self.left_motor = self.sim.getObject(f'/{robot_id}/leftMotor')
        self.right_motor = self.sim.getObject(f'/{robot_id}/rightMotor')
        self.sonar = []
        for i in range(self.num_sonar):
            self.sonar.append(self.sim.getObject(f'/{robot_id}/'
                                                 f'ultrasonicSensor[{i}]'))
        self.lidar = self.sim.getObject(f'/{robot_id}/lidar')
        # self.lidar_sensor_1 = self.sim.getObject(
        #     f'/{robot_id}/lidar/SICK_TiM310_sensor1'
        # )

    def get_sonar(self):
        readings = []
        for i in range(self.num_sonar):
            res, dist, _, _, _ = self.sim.readProximitySensor(self.sonar[i])
            readings.append(dist if res == 1 else self.sonar_max)
        return readings

    def get_lidar_data(self) -> list:
        data = self.sim.getStringSignal('PioneerP3dxLidarData')
        data_loaded = json.loads(data)
        return data_loaded

    def get_position(self):
        data = self.sim.getStringSignal('PioneerP3dxPositionData')
        data_loaded = json.loads(data)
        return data_loaded

    # def get_lidar_sensor_1_data(self):
    #     data = self.sim.readVisionSensor(self.lidar_sensor_1)
    #     return data[2]

    def set_speed(self, left_speed, right_speed):
        self.sim.setJointTargetVelocity(self.left_motor, left_speed)
        self.sim.setJointTargetVelocity(self.right_motor, right_speed)
