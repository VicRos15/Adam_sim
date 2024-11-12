from dynamics import Dynamics
from kinematics import Kinematics
import pybullet as p
import time
import math


#Class for movement
class Movement(Dynamics,Kinematics):
    def __init__(self, urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True):
        super().__init__(urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True)

    def move_joints_to_angles(self, arm, joint_angles):
        
        if arm == "left":
            joint_indices = self.ur3_left_arm_joints
        elif arm == "right":
            joint_indices = self.ur3_right_arm_joints
        else:
            raise ValueError("El brazo debe ser 'left' o 'right'.")

        #Asignar los ángulos a cada articulación del brazo
        for i, joint_angle in enumerate(joint_angles):
            p.resetJointState(self.robot_id, joint_indices[i], joint_angle)

        #Calculo de la  cinematica directa
        pos_ee, ori_ee = self.Calculate_forward_kinematics(arm)

    def move_arm_to_pose(self, arm, pose):
        #Descomponemos la pose
        target_position = pose[0]
        target_orientation = pose[1]
        
        #Numero de articulaciones
        all_joints = list(range(p.getNumJoints(self.robot_id)))

        # Obtener los índices del brazo seleccionado
        if arm == "left":
            joint_indices = self.ur3_left_arm_joints
            # Excluir las del brazo izquierdo
            avoid_joints = list(set(all_joints) - set(self.ur3_left_arm_joints))
            #offset del brazo izquierdo para la solucion cinematica inversa
            offset_iksol = 8

        elif arm == "right":
            joint_indices = self.ur3_right_arm_joints 
            # Excluir las del brazo izquierdo
            avoid_joints = list(set(all_joints) - set(self.ur3_right_arm_joints))
            #offset del brazo derecho para la solucion cinematica inversa
            offset_iksol = 2

        else:
            raise ValueError("El brazo debe ser 'left' o 'right'.")


        # Inverse kinematics for the UR3 robot
        ik_solution = self.Calculate_inverse_kinematics(self.robot_id, joint_indices[-1], target_position, target_orientation)
        
        # Comprobar si la solución es válida (verificar si hay posiciones NaN)
        if ik_solution is None or any([math.isnan(val) for val in ik_solution]):
            print(f"Posición no alcanzable por el brazo {arm}.")
            return False
        # Si la solución es válida, mover el brazo
        else:

            if self.InverseDynamics:
                #Calculo de la dinamica inversa
                torque = self.Calculate_inverse_dynamics(ik_solution,arm,offset_iksol)

                for i, joint_id in enumerate(joint_indices):
                    p.setJointMotorControl2(self.robot_id, joint_id, p.TORQUE_CONTROL, torque[i])
                    # print(f"Brazo {arm} movido a la posición: {target_position}.")

                #Calculamos la dinámica directa para obtener la aceleracion de las articulaciones al aplicar una fuerza sobre ellas
                acc = self.Calculate_forward_dynamics(torque,arm)
                for i in range(len(acc)):
                    print(f"Aceleracion de cada joint: {acc[i]}")

            else:
                for i, joint_id in enumerate(joint_indices):
                    p.setJointMotorControl2(self.robot_id, joint_id, p.POSITION_CONTROL, ik_solution[i+offset_iksol])
                    # print(f"Brazo {arm} movido a la posición: {target_position}.")
        return True

    
    def move_arm_to_multiple_poses(self, arm, poses):
    
        for pose in poses:
            self.move_arm_to_pose(arm, pose)

            # Avanzar la simulación para que los movimientos se apliquen
            if not self.useRealTimeSimulation:
                p.stepSimulation()
                time.sleep(self.t)
