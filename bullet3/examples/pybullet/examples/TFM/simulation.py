from sliders import Sliders
from kinematics import Kinematics
import time
import pybullet as p
import scipy.io


#Class for simualtion
class Simulation(Sliders,Kinematics):
    def __init__(self, urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True):
        super().__init__(urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True)
    
    #! START SIMULATION
    def run_simulation(self):

        p.setRealTimeSimulation(self.useRealTimeSimulation)

        #Poses comandadas
        # poses = [
        #     [[0.3368, 0.5086, 1.4509], p.getQuaternionFromEuler([-1.7339, 0.77, -1.8])],
        #     [[0.338, 0.5789, 1.391], p.getQuaternionFromEuler([-1.709, 0.575, -1.764])],
        #     [[0.4046, 0.54367, 1.3562], p.getQuaternionFromEuler([-1.842, 0.5423, 1.3562])],
        #     [[0.483963,0.5057,1.2756], p.getQuaternionFromEuler([-1.9825, 0.3428, 2.1356])]
        # ]

        #Initial_arm_poses
        initial_left_pose = [0.11,-1.96,-0.79,0.67,-0.08,-0.01]
        initial_right_pose = [0.66,-2.26,-0.70,-0.14,2.55,-1.15]

        #Cargamos el archivo de matlab
        positions_data = scipy.io.loadmat('/home/vrosi/TFM/Adam_sim/bullet3/examples/pybullet/examples/Test_matlab/positions.mat')
        positions = positions_data['positionR']
        orientations_data = scipy.io.loadmat('/home/vrosi/TFM/Adam_sim/bullet3/examples/pybullet/examples/Test_matlab/orientations.mat')
        orientations = orientations_data['quaternionR']
        
        poses = [
        [(positions[i, 0], positions[i, 1], positions[i, 2]), (orientations[i, 1], orientations[i, 2], orientations[i, 3], orientations[i, 0])]
        for i in range(positions.shape[0])]
        print("len de poses: ",len(poses))



        for i in range(100):
            self.initial_arm_pose("right",initial_right_pose)
            self.initial_arm_pose("left",initial_left_pose)

        while True:

            #Inicializamos el tiempo de simulacion
            if (self.useSimulation and self.useRealTimeSimulation==0):
                p.stepSimulation()

            self.move_arm_to_multiple_poses("right", poses, poses2=None, dynamic_time=10, acc=None, threshold=None)
            # self.apply_slider_values()

            if not self.useRealTimeSimulation:
                time.sleep(self.t)


# Programa principal
robot_urdf_path = "/home/vrosi/TFM/Adam_sim/paquetes_simulacion/rb1_base_description/robots/robot.urdf"
robot_stl_path = "/home/vrosi/TFM/Adam_sim/paquetes_simulacion/rb1_base_description/meshes/others/adam_modelv3.stl"


adam_robot = Simulation(robot_urdf_path,robot_stl_path,1,0)
adam_robot.create_sliders()

adam_robot.run_simulation()