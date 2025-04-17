import skfuzzy as fuzz
import numpy as np
from skfuzzy.control import Antecedent, Consequent, Rule, ControlSystem, ControlSystemSimulation


def get_fuzzy_distances():

    left_distance = Antecedent(np.arange(0, 1001, 1), 'left_distance')
    right_distance = Antecedent(np.arange(0, 1001, 1), 'right_distance')
    front_distance = Antecedent(np.arange(0, 1001, 1), 'front_distance')

    for distance in [left_distance, right_distance, front_distance]:
        distance['very_near'] = fuzz.trapmf(distance.universe, [0, 0, 200, 200])
        distance['near'] = fuzz.trapmf(distance.universe, [200, 200, 400, 400])
        distance['far'] = fuzz.trimf(distance.universe, [200, 1000, 1000])

    return left_distance, front_distance, right_distance


def get_fuzzy_speeds():
    left_speed = Consequent(np.arange(-0.5, 1, 0.01), 'left_speed')
    right_speed = Consequent(np.arange(-0.5, 1, 0.01), 'right_speed')

    for speed in [left_speed, right_speed]:
        speed['reverse'] = fuzz.trimf(speed.universe, [-5, -5, 0])
        speed['stop'] = fuzz.trimf(speed.universe, [-0.5, 0, 0.5])
        speed['forward'] = fuzz.trimf(speed.universe, [0, 5, 5])

    return left_speed, right_speed


def generate_rules():
    left_distance, front_distance, right_distance = get_fuzzy_distances()
    left_speed, right_speed = get_fuzzy_speeds()

    return [
        Rule(front_distance['far'], (left_speed['forward'], right_speed['forward'])),

        Rule(
            front_distance['very_near'] & ~left_distance['very_near'] & right_distance['very_near'], 
            (left_speed['reverse'], right_speed['forward'])),
        Rule(
            front_distance['very_near'] & ~left_distance['very_near'] & ~right_distance['very_near'], 
            (left_speed['forward'], right_speed['reverse'])),
        Rule(
            front_distance['very_near'] & left_distance['very_near'], 
            (left_speed['forward'], right_speed['reverse'])),

        Rule(
            front_distance['near'] & left_distance['near'],
            (left_speed['forward'], right_speed['stop'])
        ),
        Rule(
            front_distance['near'] & right_distance['near'],
            (left_speed['stop'], right_speed['forward'])
        )
    ]


def create_fuzzy_controller():
    return ControlSystemSimulation(
        ControlSystem(generate_rules())
    )
