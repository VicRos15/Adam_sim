from arms_dynamics import ArmsDynamics 
import pybullet as p
import time
import math
import rospy

#Class for kinematics
class ArmsKinematics(ArmsDynamics):
    def __init__(self, urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True):
        super().__init__(urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True)


    # Cinemática directa
    def calculate_arm_forward_kinematics(self, arm):
        pos, ori = self.get_arm_end_effector_pose(arm)

        return pos, ori
    
    # Cálculo de la cinématica inversa
    def calculate_arm_inverse_kinematics(self,robot_id, ee_index, target_position, target_orientation, null=True):
        
        if null:
            ik_solution = p.calculateInverseKinematics(robot_id, ee_index, target_position, target_orientation,lowerLimits=self.ll,
                                                    upperLimits=self.ul,
                                                    jointRanges=self.jr,
                                                    restPoses=self.rp)
        else:
            ik_solution = p.calculateInverseKinematics(robot_id, ee_index, target_position, target_orientation,jointDamping=self.jd,
                                                solver=0,
                                                maxNumIterations=100,
                                                residualThreshold=.01)
        return ik_solution
    
    def move_arm_to_pose(self, arm, pose_des, pos_act=None, vel_act=None, accurate=None, threshold=None):

        #Descomponemos la pose_des
        target_position = pose_des[0]
        target_orientation = pose_des[1]
        
        #Numero de articulaciones
        all_joints = list(range(p.getNumJoints(self.robot_id)))

        # Obtener los índices del brazo seleccionado
        if arm == "left":
            joint_indices = self.ur3_left_arm_joints
            #offset del brazo izquierdo para la solucion cinematica inversa
            offset_iksol = 8

        elif arm == "right":
            joint_indices = self.ur3_right_arm_joints 
            #offset del brazo derecho para la solucion cinematica inversa
            offset_iksol = 2

        else:
            raise ValueError("El brazo debe ser 'left' o 'right'.")

        # Inverse kinematics for the UR3 robot
        ik_solution = self.calculate_arm_inverse_kinematics(self.robot_id, joint_indices[-1], target_position, target_orientation)
    

        # Comprobar si la solución es válida (verificar si hay posiciones NaN)
        if ik_solution is None or any([math.isnan(val) for val in ik_solution]):
            print(f"Posición no alcanzable por el brazo {arm}.")
            return False
        
        pos_des = []
        for i in range(len(joint_indices)):
            pos_des.append(ik_solution[i+offset_iksol])

        # Si la solución es válida, mover el brazo
        else:

            # Calculando la dinamica
            if self.Dynamics:
                #Calculo de la dinamica inversa
                torque, vel_des, acc_des = self.calculate_arm_inverse_dynamics(pos_des, pos_act, vel_act, arm)

                for i, joint_id in enumerate(joint_indices):
                      #set the joint friction
                    p.setJointMotorControl2(self.robot_id, joint_id, p.VELOCITY_CONTROL, targetVelocity=0, force=20)
                    #apply a joint torque
                    p.setJointMotorControl2(self.robot_id, joint_id, p.TORQUE_CONTROL, force=torque[i])
                p.stepSimulation()

                #Calculamos la dinámica directa para obtener la aceleracion de las articulaciones al aplicar una fuerza sobre ellas
                acc = self.calculate_arm_forward_dynamics(torque,arm)

            # Sin calcular la dinámica
            else:

                # Calculo de la cinematica inversa precisa
                if accurate is not None:
                    threshold2=threshold*threshold
                    closeEnough=False
                    while not closeEnough:
                        closeEnough = self.close_enough_pose(joint_indices, ik_solution, offset_iksol, target_position, threshold2)

                # Calculo de la cinematica inversa sin precision
                else:
                    for i, joint_id in enumerate(joint_indices):
                        p.setJointMotorControl2(self.robot_id, joint_id, p.POSITION_CONTROL, ik_solution[i+offset_iksol])

                # Visualize real robot
                # right_position = rospy.get_param('position_right')
                # print("joints right",right_position)
                vel_des = None
        
        return ik_solution, pos_des, vel_des


    
    def move_arm_to_multiple_poses(self, arm, poses, poses2=None, dynamic_time=None, acc=None, threshold=None):
        
        cont = 0
        previous_pos = None
        previous_vel = None
        pos_act = None
        vel_act = None
        if arm == "left" or arm == "right":
            self.dt = (dynamic_time/(len(poses)) )+ 10e-30
            if arm =="left":
                # Activar publicador de left arm
                self.pub_left=True
                
            else:
                # Activar publicador de right arm
                self.pub_right=True

            for pose in poses:
                if cont==0 and self.Dynamics==True:
                    pos_act, vel_act = self.get_joints_pos_vel(arm)
                elif cont!=0 and self.Dynamics==True:
                    pos_act = previous_pos
                    vel_act = previous_vel


                self.detect_autocollisions()
                ik, pos_prev, vel_prev = self.move_arm_to_pose(arm, pose, pos_act, vel_act, acc, threshold)

                # Guardar en una lista las poses del brazo, para publicar más tarde en ROS
                if arm =="left":
                    self.left_joints.append(pos_prev)
                else:
                    self.right_joints.append(pos_prev)

                cont=cont+1
                previous_pos = pos_prev
                previous_vel = vel_prev

                # Avanzar la simulación para que los movimientos se apliquen
                if not self.useRealTimeSimulation:
                    p.stepSimulation()
                    time.sleep(self.t)
            

        if arm == "both":
            # Activamos ambos publicadores
            self.pub_right, self.pub_left = True, True 

            if poses2 is None:
                raise ValueError("Debes proporcionar poses2 para mover ambos brazos")

            for pose_left, pose_right in zip(poses, poses2):
                self.detect_autocollisions()
                ik, pos_left, vel_prev = self.move_arm_to_pose("left", pose_left, pos_act, vel_act, acc, threshold)
                self.left_joints.append(pos_left)
                ik, pos_right, vel_prev = self.move_arm_to_pose("right", pose_right, pos_act, vel_act, acc, threshold)
                self.right_joints.append(pos_right)

                # Avanzar la simulación para que los movimientos se apliquen
                if not self.useRealTimeSimulation:
                    p.stepSimulation()
                    time.sleep(self.t)
    

    def close_enough_pose(self, joint_indices, ik_solution, offset_iksol, targetPos, threshold):
        closeEnough = False
        dist2 = 1e30
        while (not closeEnough):
            for i, joint_id in enumerate(joint_indices):
                p.setJointMotorControl2(self.robot_id, joint_id, p.POSITION_CONTROL, ik_solution[i+offset_iksol])

            ls = p.getLinkState(self.robot_id, joint_indices[-1])
            newPos = ls[4] # End-Effector position
            diff = [targetPos[0] - newPos[0], targetPos[1] - newPos[1], targetPos[2] - newPos[2]]
            dist2 = (diff[0] * diff[0] + diff[1] * diff[1] + diff[2] * diff[2])
            closeEnough = (dist2 < threshold)
            if not self.useRealTimeSimulation:
                p.stepSimulation()
                time.sleep(self.t)

        return closeEnough

    def move_arm_joints_to_angles(self, arm, joint_angles):
        
        if arm == "left":
            joint_indices = self.ur3_left_arm_joints
        elif arm == "right":
            joint_indices = self.ur3_right_arm_joints
        else:
            raise ValueError("El brazo debe ser 'left' o 'right'.")

        #Asignar los ángulos a cada articulación del brazo
        for i, joint_angle in enumerate(joint_angles):
            p.resetJointState(self.robot_id, joint_indices[i], joint_angle)

        #Calculo de la cinematica directa
        pos_ee, ori_ee = self.calculate_arm_forward_kinematics(arm)

    
    def initial_arm_pose(self,arm,pose,type="joint"):
        # Obtener los índices del brazo seleccionado
        if arm == "left":
            joint_indices = self.ur3_left_arm_joints
            #offset del brazo izquierdo para la solucion cinematica inversa
            offset_iksol = 8

        elif arm == "right":
            joint_indices = self.ur3_right_arm_joints
            #offset del brazo derecho para la solucion cinematica inversa
            offset_iksol = 2
        else:
            raise ValueError("El brazo debe ser 'left' o 'right'.")
        
        if type == "joint":
            for i, joint_id in enumerate(joint_indices):
                    p.setJointMotorControl2(self.robot_id, joint_id, p.POSITION_CONTROL, pose[i])
        elif type == "pose":
            ik_solution = self.calculate_arm_inverse_kinematics(self.robot_id, joint_indices[-1], pose[0], pose[1])
            for i, joint_id in enumerate(joint_indices):
                    p.setJointMotorControl2(self.robot_id, joint_id, p.POSITION_CONTROL, ik_solution[i+offset_iksol])
        else:
            raise ValueError("El tipo de pose debe ser 'joint' o 'pose'.")
        
        if not self.useRealTimeSimulation:
            p.stepSimulation()
            time.sleep(self.t)

    def get_arm_end_effector_pose(self, arm):

        # Obtener la posición del efector final del brazo
        if arm == "left":
            end_effector_index = self.ur3_left_arm_joints[-1]
        elif arm == "right":
            end_effector_index = self.ur3_right_arm_joints[-1]
        else:
            raise ValueError("El brazo debe ser 'left' o 'right'.")

        link_state = p.getLinkState(self.robot_id, end_effector_index)
        return link_state[4],link_state[5]

