from sliders import Sliders
from movement import Movement
import time
import pybullet as p


#Class for simualtion
class Simulation(Sliders,Movement):
    def __init__(self, urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True):
        super().__init__(urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True)
    
    #! START SIMULATION
    def run_simulation(self):

        p.setRealTimeSimulation(self.useRealTimeSimulation)

        #Poses comandadas
        poses = [
            [[0.3368, 0.5086, 1.4509], p.getQuaternionFromEuler([-1.7339, 0.77, -1.8])],
            [[0.338, 0.5789, 1.391], p.getQuaternionFromEuler([-1.709, 0.575, -1.764])],
            [[0.4046, 0.54367, 1.3562], p.getQuaternionFromEuler([-1.842, 0.5423, 1.3562])],
            [[0.483963,0.5057,1.2756], p.getQuaternionFromEuler([-1.9825, 0.3428, 2.1356])]
        ]

        while True:

            #Inicializamos el tiempo de simulacion
            if (self.useSimulation and self.useRealTimeSimulation==0):
                p.stepSimulation()

            self.move_arm_to_multiple_poses("left",poses)
            # self.apply_slider_values()

            if not self.useRealTimeSimulation:
                time.sleep(self.t)


# Programa principal
robot_urdf_path = "/home/victor/TFM/Adam_sim/paquetes_simulacion/rb1_base_description/robots/robot.urdf"
robot_stl_path = "/home/victor/TFM/Adam_sim/paquetes_simulacion/rb1_base_description/meshes/others/torso_sin_hombros.stl"


adam_robot = Simulation(robot_urdf_path,robot_stl_path,0,1)
adam_robot.create_sliders()

adam_robot.run_simulation()