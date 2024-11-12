from collisions import Collisions
import pybullet as p
import numpy as np
import time


#Class for the sliders
class Sliders(Collisions):

    def __init__(self, urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True):
        super().__init__(urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True)
    
    def create_sliders(self):
            # Crear sliders para controlar las articulaciones
            self.slider_ids = []
            joint_range = np.pi
            for i in self.ur3_left_arm_joints + self.ur3_right_arm_joints:
                joint_info = p.getJointInfo(self.robot_id, i)
                joint_name = joint_info[1].decode("utf-8")
                if joint_info[2] == p.JOINT_REVOLUTE:
                    self.slider_ids.append(p.addUserDebugParameter(joint_name, -joint_range, joint_range, 0))

    def apply_slider_values(self):
        # Aplicar los valores de los sliders a las articulaciones
        for i, joint_id in enumerate(self.ur3_left_arm_joints + self.ur3_right_arm_joints):
            slider_value = p.readUserDebugParameter(self.slider_ids[i])
            # print(f"Valor del slider para la articulación {joint_id}: {slider_value}")

            # Controlar la articulación con el valor del slider
            if not self.detect_autocollisions():
                p.setJointMotorControl2(self.robot_id, joint_id, p.POSITION_CONTROL, targetPosition=slider_value, force=500)
            else:
                p.setJointMotorControl2(self.robot_id, joint_id, p.POSITION_CONTROL, targetPosition=slider_value, force=500)
                # print("Colisión detectada.")

            # Avanzar la simulación para que los movimientos se apliquen
            if not self.useRealTimeSimulation:
                p.stepSimulation()
                time.sleep(self.t)
