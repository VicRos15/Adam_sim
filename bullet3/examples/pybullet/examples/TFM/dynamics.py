from Adam import ADAM 
import pybullet as p


#Class for dynamics
class Dynamics(ADAM):

    def __init__(self, urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True):
        super().__init__(urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True)

    
    def get_joints_pos_vel(self, arm):

        joint_positions = []
        joint_velocities = []

        # Obtener los índices del brazo seleccionado
        if arm == "left":
            joint_indices = self.ur3_left_arm_joints
        elif arm == "right":
            joint_indices = self.ur3_right_arm_joints
        else:
            raise ValueError("El brazo debe ser 'left' o 'right'.")

        # Leer las posiciones articulares
        for joint_id in joint_indices:
            joint_state = p.getJointState(self.robot_id, joint_id)
            joint_positions.append(joint_state[0])  # La posición de la articulación está en el índice 0
            joint_velocities.append(joint_state[1]) # La velocidad de la articulacion en el indice 1

        return joint_positions, joint_velocities

    
    # Forward dynamics
    def Calculate_forward_dynamics(self, torques, arm):
        if arm == "right":
            joints = self.ur3_right_arm_joints

        elif arm == "left":
            joints = self.ur3_left_arm_joints

        else:
            raise ValueError("El brazo debe ser 'left' o 'right'.")

        #Se obtienen las posiciones y velocidades actuales
        self.pos_act,self.vel_act = self.get_joints_pos_vel(arm)

        #Obtener los términos de la dinámica del sistema
        zero_acc = [0.0] * len(joints)
        zero_vel = zero_acc
        #Termino gravitacional
        tau_g = p.calculateInverseDynamics(self.robot_id,self.pos_act,zero_vel,zero_acc, flags=1)

        #Terminos de coriolis y centrifugo
        #Para calcular el torque de coriolis y centrifugas despreciamos la gravedad
        tau_c_c_g = p.calculateInverseDynamics(self.robot_id,self.pos_act, self.vel_act,zero_acc, flags=1)
    
        # Restar los términos gravitacionales de los combinados
        tau_c_c = [tau_c_c_g[i] - tau_g[i] for i in range(len(joints))]
        
        #Calculo de la aceleracion
        joints_acc=[]
        for j, i in enumerate(joints):
            #Matriz de inercia M
            dynamics_info = p.getDynamicsInfo(self.robot_id,i)
            # Obtener el valor de inercia para el eje z
            I_z = dynamics_info[2][2]

            
            if I_z>0:
                joints_acc.append((torques[j]-tau_c_c[j]-tau_g[j])/I_z)
            else: 
                joints_acc.append(0.0)

        return joints_acc
    
    # Inverse dynamics
    def Calculate_inverse_dynamics (self,target_pose, arm, offset):
        #Calculate current pos, vel 
        if arm == "right" or arm == "left":
            pos_act,vel_act = self.get_joints_pos_vel(arm)
        else:
            raise ValueError("El brazo debe ser 'left' o 'right'.")
        
        # for i in range(len(pos_act)):
        #     print(f"Velocidad es de tamaño {i} y vale {vel_act[i]}")
        #     print(f"Posicion es de tamaño {i} y vale {pos_act[i]}")
        # if len(target_pose) < len(self.ur3_left_arm_joints):
        #     raise ValueError("target_pose no tiene suficientes elementos para el número de articulaciones.")
        
        # Goal pose
        pos_des = []
        for i in range(len(self.ur3_left_arm_joints)):
            pos_des.append(target_pose[i])

        #Implement a PD controller to calculate desired acceleration
        #PD parameters
        kp = 100
        kd = 10
        #PD controller
        acc_des=[]
        for i in range(len(vel_act)):
            acc_des.append(kp*(pos_des[i]-pos_act[i])-kd*vel_act[i])
        
        for i in range(len(acc_des)):
            print(f"ACeleracion es de tamaño {i} y vale {acc_des[i]}")

        #Calculate inverse dynamics
        try:
            torque_IK = []
            torque_all = list(p.calculateInverseDynamics(self.robot_id, pos_act, vel_act, acc_des, flags=1))
            for i in range (len(torque_all)):
                print(f"torque en articulacion {i} es de {torque_all[i]}")
            for i in range(len(self.ur3_left_arm_joints)):
                torque_IK.append(torque_all[i+offset])
        except Exception as e:
            raise SystemError(f"Error en calculateInverseDynamics: {e}")

        # Compare torque calculated with the maximum torque 
        for i, torque in enumerate(torque_IK):
            joint_info = p.getJointInfo(self.robot_id,i)
            max_torque = joint_info[10] 

            if abs(torque) > max_torque:
                torque_IK[i] = max_torque if torque > 0 else -max_torque
            # print(f"Torque aplicado en la joint {i}: {torque_IK[i]}")

        return torque_IK
    

