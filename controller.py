from dataclasses import dataclass
from typing import List, Tuple


@dataclass
class FuzzyRules:
    """Fuzzy rules for the robot navigation"""

    VERY_CLOSE = 0.2
    CLOSE = 0.3
    SAFE = 0.8

    NORMAL_SPEED = 1.0
    TURN_SPEED = 0.6
    SLOW_SPEED = 0.4


class SensorReader:
    """Class to interpret sensor data"""

    @staticmethod
    def get_danger_zones(readings: List[float]) -> Tuple[float, float, float]:
        """
        Get the danger level in three zones: left, center and right
        Returns: (left_danger, center_danger, right_danger)
        """
        left_danger = min(readings[2:4])
        center_danger = min(readings[4:6])
        right_danger = min(readings[6:8])

        return left_danger, center_danger, right_danger


class FuzzyController:
    """Fuzzy controller for the robot navigation"""

    def __init__(self):
        self.rules = FuzzyRules()
        self.sensor_reader = SensorReader()

    def evaluate_situation(self, left: float, center: float,
                           right: float) -> str:
        if center < self.rules.VERY_CLOSE:
            return "emergency_turn"
        elif left < self.rules.CLOSE and right < self.rules.CLOSE:
            return "emergency_turn"
        elif right < self.rules.CLOSE:
            return "turn_left"
        elif left < self.rules.CLOSE:
            return "turn_right"
        else:
            return "advance"

    def get_wheel_speeds(self, action: str) -> Tuple[float, float]:
        speeds = {
            "advance": (self.rules.NORMAL_SPEED, self.rules.NORMAL_SPEED),
            "turn_left": (-self.rules.SLOW_SPEED, self.rules.TURN_SPEED),
            "turn_right": (self.rules.TURN_SPEED, -self.rules.SLOW_SPEED),
            "emergency_turn": (-self.rules.TURN_SPEED, self.rules.TURN_SPEED)
        }
        return speeds[action]

    def compute_movement(self, readings: List[float]) -> Tuple[float, float]:
        left, center, right = self.sensor_reader.get_danger_zones(readings)
        action = self.evaluate_situation(left, center, right)
        return self.get_wheel_speeds(action)
