import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt

class FuzzyBrakingSystem:
    def __init__(self):
        # Define input variables
        self.stop_distance = ctrl.Antecedent(np.arange(0, 11, 1), 'stop_distance')
        self.vehicle_speed = ctrl.Antecedent(np.arange(0, 41, 1), 'vehicle_speed')

        # Define output variable
        self.braking_signal = ctrl.Consequent(np.arange(0, 1.01, 0.01), 'braking_signal')

        # Define membership functions for stop_distance
        self.stop_distance['very_close'] = fuzz.trapmf(self.stop_distance.universe, [0, 0, 2, 4])
        self.stop_distance['close'] = fuzz.trimf(self.stop_distance.universe, [2, 4, 6])
        self.stop_distance['medium'] = fuzz.trimf(self.stop_distance.universe, [4, 6, 8])
        self.stop_distance['far'] = fuzz.trapmf(self.stop_distance.universe, [6, 8, 10, 10])

        # Define membership functions for vehicle_speed
        self.vehicle_speed['very_slow'] = fuzz.trapmf(self.vehicle_speed.universe, [0, 0, 5, 10])
        self.vehicle_speed['slow'] = fuzz.trimf(self.vehicle_speed.universe, [5, 10, 20])
        self.vehicle_speed['medium'] = fuzz.trimf(self.vehicle_speed.universe, [15, 25, 35])
        self.vehicle_speed['fast'] = fuzz.trapmf(self.vehicle_speed.universe, [30, 35, 40, 40])

        # Define membership functions for braking_signal
        self.braking_signal['light'] = fuzz.trapmf(self.braking_signal.universe, [0, 0, 0.2, 0.4])
        self.braking_signal['moderate'] = fuzz.trimf(self.braking_signal.universe, [0.3, 0.5, 0.7])
        self.braking_signal['strong'] = fuzz.trimf(self.braking_signal.universe, [0.6, 0.8, 1])
        self.braking_signal['emergency'] = fuzz.trapmf(self.braking_signal.universe, [0.8, 0.9, 1, 1])

        # Define fuzzy rules
        rule1 = ctrl.Rule(self.stop_distance['far'] & self.vehicle_speed['slow'], self.braking_signal['light'])
        rule2 = ctrl.Rule(self.stop_distance['medium'] & self.vehicle_speed['medium'], self.braking_signal['moderate'])
        rule3 = ctrl.Rule(self.stop_distance['close'] & self.vehicle_speed['fast'], self.braking_signal['strong'])
        rule4 = ctrl.Rule(self.stop_distance['very_close'] & self.vehicle_speed['fast'], self.braking_signal['emergency'])

        # Create the fuzzy control system
        self.braking_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4])
        self.braking_simulation = ctrl.ControlSystemSimulation(self.braking_ctrl)

    def get_braking_value(self, stop_distance, vehicle_speed):
        self.braking_simulation.input['stop_distance'] = stop_distance
        self.braking_simulation.input['vehicle_speed'] = vehicle_speed
        self.braking_simulation.compute()
        return self.braking_simulation.output['braking_signal']

    def plot_membership_functions(self):
        fig, (ax0, ax1, ax2) = plt.subplots(nrows=3, figsize=(8, 9))

        self.stop_distance.view(ax=ax0)
        ax0.set_title('Stop Distance')
        ax0.legend()

        self.vehicle_speed.view(ax=ax1)
        ax1.set_title('Vehicle Speed')
        ax1.legend()

        self.braking_signal.view(ax=ax2)
        ax2.set_title('Braking Signal')
        ax2.legend()

        plt.tight_layout()
        plt.show()

    def plot_result(self, stop_distance, vehicle_speed):
        self.get_braking_value(stop_distance, vehicle_speed)
        self.braking_signal.view(sim=self.braking_simulation)
        plt.show()

# Example usage
if __name__ == "__main__":
    braking_system = FuzzyBrakingSystem()

    # Test the system
    example_distance = 5
    example_speed = 20

    braking_value = braking_system.get_braking_value(example_distance, example_speed)
    print(f"\nFor distance {example_distance}m and speed {example_speed}km/h:")
    print(f"Braking signal: {braking_value}")

    # Plot membership functions
    braking_system.plot_membership_functions()

    # Plot result for the example inputs
    braking_system.plot_result(example_distance, example_speed)