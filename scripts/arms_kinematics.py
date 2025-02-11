from arms_dynamics import ArmsDynamics 
import pybullet as p
import time
import math
import PyKDL
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
import numpy as np


# Class for kinematics
class ArmsKinematics(ArmsDynamics):
    def __init__(self, urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True):
        super().__init__(urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True)

        # Listas para registrar los datos de la cinemtica inversa
        self.ik_solution_data_left = []
        self.ik_solution_data_right = []

        # Load URDF model to create an object of the class URDF
        self.adam = URDF.from_xml_file(urdf_path)

        # Convert URDF to KDLtree
        success, self.KDL_tree = treeFromUrdfModel(self.adam)

        if not success:
            raise RuntimeError("Failed to parse URDF into a KDL tree")
        
        # Create both arms' chains
        #LEFT ARM
        left_base_link = "rb1_left_arm_base_link"
        left_end_effector = "L_hand_base_link"


        # Compute kinematic chain
        self.left_kdl_chain = self.KDL_tree.getChain(left_base_link, left_end_effector)

        # Compute the transform from world to the base link of the arm
        self.world_to_left_base_transform = self.get_transform_to_base(left_base_link)

        # Create a solver for forward kinematics
        self.left_fk_solver = PyKDL.ChainFkSolverPos_recursive(self.left_kdl_chain)



        #RIGHT ARM
        right_base_link = "rb1_right_arm_base_link"
        right_end_effector = "R_hand_base_link"

        # Compute kinematic chain
        self.right_kdl_chain = self.KDL_tree.getChain(right_base_link, right_end_effector)

        # Compute the transform from world to the base link of the arm
        self.world_to_right_base_transform = self.get_transform_to_base(right_base_link)

        # Create a solver for forward kinematics
        self.right_fk_solver = PyKDL.ChainFkSolverPos_recursive(self.right_kdl_chain)

        '''
        # Right arm transformation matrix
        right_rotation_matrix = [[0, -1, 0, 0.07],
                        [-0.7071, 0, -0.7071, -0.1655],
                        [0.7071, 0, -0.7071, 1.202],
                        [0, 0, 0, 1]]

        # Left arm transformation matrix
        left_rotation_matrix = [[0, 1, 0, 0.07],
                            [0.7071, 0, 0.7071, -0.1655],
                            [0.7071, 0, -0.7071, 1.202],
                            [0, 0, 0, 1]]
        
        # Create a PyKDL Rotation object from the matrix
        rotation = PyKDL.Rotation(
            rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2],
            rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2],
            rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2]
        )

        # Create a PyKDL Frame with the rotation and an optional translation
        translation = PyKDL.Vector(rotation_matrix[0][3], rotation_matrix[1][3], rotation_matrix[2][3])  # Example translation vector
        world_to_base_transform = PyKDL.Frame(rotation, translation)'''


    # Extract the transform from the world to the base link of the arm
    def get_transform_to_base(self, base_link):
        
        # Computes the transform from the world frame to the base link of the arm.
        
        world_to_base_transform = PyKDL.Frame()  # Initialize as identity

        current_link = base_link
        while current_link:
            for joint in self.adam.joints:
                if joint.child == current_link:
                    # Parse joint's origin (translation + rotation)
                    origin = joint.origin
                    translation = PyKDL.Vector(*origin.xyz)
                    rotation = PyKDL.Rotation.RPY(*origin.rpy)
                    joint_transform = PyKDL.Frame(rotation, translation)

                    # Pre-multiply to accumulate transform
                    world_to_base_transform = joint_transform * world_to_base_transform

                    # Move to the parent link
                    current_link = joint.parent
                    break
            else:
                # No more parents; reached the root
                break

        return world_to_base_transform



    # Cinemática directa
    def calculate_arm_forward_kinematics(self, arm):
        pos, ori = self.get_arm_end_effector_pose(arm)

        return pos, ori
    
    # Cinematica directa PyKDL
    def calculate_FK_PyKDL(self, arm, q_joints):
        if arm == 'right':
            world_to_base_transform = self.world_to_right_base_transform
            kdl_chain = self.right_kdl_chain
            fk_solver = self.right_fk_solver
        elif arm == 'left':
            world_to_base_transform = self.world_to_left_base_transform
            kdl_chain = self.left_kdl_chain
            fk_solver = self.left_fk_solver
        
        num_joints = kdl_chain.getNrOfJoints()

        # Create PyKDL joints
        q_joints_KDL = PyKDL.JntArray(num_joints)

        for i, q in enumerate(q_joints):
            q_joints_KDL[i] = q
        
        # Create a frame to store the result
        end_effector_frame = PyKDL.Frame()

        # Calculate FK if the result obtained is positive, there is a solution
        if fk_solver.JntToCart(q_joints_KDL, end_effector_frame) >= 0:
            
            # Convert FK result to the world reference
            end_effector_in_world = world_to_base_transform * end_effector_frame
            # Extract position and orientation
            position = end_effector_in_world.p #PyKDL.Vector
            orientation = end_effector_in_world.M #PyKDL.Rotation

            # Transform the orientation into quaternions
            quaternions = orientation.GetQuaternion()
        else:
            print("Error al calcular la cinematica directa")

        # Devuelve la pose del efector final
        return position, quaternions


    ## Cinemática inversa
    def ik_data(self, ik_solution, arm):
        if arm == "left":
            self.ik_solution_data_left.append(ik_solution)
        elif arm == "right":
            self.ik_solution_data_right.append(ik_solution)

    def print_ik_data(self, arm):
        if arm == "left":
            print(f"Valores generados por la cinematica inversa en el brazo izquierdo: {self.ik_solution_data_left}")
        elif arm == "right":
            print(f"Valores generados por la cinematica inversa en el brazo derecho: {self.ik_solution_data_right}")
        elif arm == "both":
            print(f"Valores generados por la cinematica inversa en el brazo izquierdo: {self.ik_solution_data_left}")
            print(f"Valores generados por la cinematica inversa en el brazo derecho: {self.ik_solution_data_right}")
        
    def reset_ik_data(self, arm):

        self.ik_solution_data_left = []
        self.ik_solution_data_right = []

    
    # Cinematica inversa con PyKDL
    def pose_is_achievable(self, arm, target_pose):
        target_position = target_pose[0]
        target_orientation = target_pose[1]
        
        if arm == "left":
            joint_indices = self.ur3_left_arm_joints
            offset_iksol = 20
            ee_joint = 52
        elif arm == "right":
            joint_indices = self.ur3_right_arm_joints 
            offset_iksol = 2
            ee_joint = 28
        
        isAchievable = False

        all_joints_sol = self.calculate_arm_inverse_kinematics(self.robot_id, joint_indices[-1], target_position, target_orientation)
        arm_joints_sol = all_joints_sol[offset_iksol:(offset_iksol+6)]

        #Calculate FK using PyKDL solver
        position, orientation = self.calculate_FK_PyKDL(arm, arm_joints_sol)

        PyKdl_pose = [position, orientation]

        error, angle = self.get_pose_error(PyKdl_pose, target_pose)

        # Calculo del error de posicion mediante la norma euclidiana
        pos_error = np.linalg.norm(error[0:3])
        print(f"error de posicion {pos_error} y de angulo {angle}")

        if pos_error < 0.1 and angle < 0.035: isAchievable=True
        else:isAchievable=False

        return isAchievable, arm_joints_sol



    def get_pose_error(self, current_pose, target_pose):

        # Convert target_pose to PyKDL
        target_position = PyKDL.Vector(*target_pose[0])  
        target_orientation = PyKDL.Rotation.Quaternion(*target_pose[1])  
        
        # Error de posicion
        # En PyKDL se pueden restar los vectores directamente
        position_error = list(current_pose[0] - target_position)


        # Compute the relative rotation from current to target
        current_rotation = PyKDL.Rotation.Quaternion(*current_pose[1])  # Convierte de tuple a PyKDL.Rotation
        relative_rotation = current_rotation.Inverse() * target_orientation

        # Convert the relative rotation to axis-angle representation
        angle, axis = relative_rotation.GetRotAngle()
        orientation_error = [angle * axis.x(), angle * axis.y(), angle * axis.z()]

        # Combine position and orientation errors
        error = np.zeros(6)  # 6D vector for 3D position and 3D orientation
        error[0:3] = position_error
        error[3:6] = orientation_error
        return error, angle
    

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
    
        pos_des = []
        for i in range(len(joint_indices)):
            pos_des.append(ik_solution[i+offset_iksol])

        # Si la solución es válida, mover el brazo
        else:

            # # Calculando la dinamica
            # if self.Dynamics:
            #     #Calculo de la dinamica inversa
            #     torque, vel_des, acc_des = self.calculate_arm_inverse_dynamics(pos_des, pos_act, vel_act, arm)

            #     for i, joint_id in enumerate(joint_indices):
            #           #set the joint friction
            #         p.setJointMotorControl2(self.robot_id, joint_id, p.VELOCITY_CONTROL, targetVelocity=0, force=20)
            #         #apply a joint torque
            #         p.setJointMotorControl2(self.robot_id, joint_id, p.TORQUE_CONTROL, force=torque[i])
            #     p.stepSimulation()

            #     #Calculamos la dinámica directa para obtener la aceleracion de las articulaciones al aplicar una fuerza sobre ellas
            #     acc = self.calculate_arm_forward_dynamics(torque,arm)

            # Sin calcular la dinámica
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
            if dynamic_time is not None:
                self.dt = (dynamic_time/(len(poses)) )+ 10e-30
            
            # ROS interface
            if arm =="left":
                # Activar publicador de left arm
                self.pub_left=True
                
            else:
                # Activar publicador de right arm
                self.pub_right=True
            

            # Comprobar toda la trayectoria antes de mover el brazo
            print('Comprobando que la trayectoria introducida es válida...............')

            for pose in poses:

                achievable, ik_sol = self.pose_is_achievable(arm, pose)

                if not achievable:
                    raise ValueError("Error: Se encontró una pose no alcanzable.")

            print(f'Trayectoria válida para el brazo {arm}')

            for pose in poses:
                if cont==0 and self.Dynamics==True:
                    pos_act, vel_act = self.get_joints_pos_vel(arm)
                elif cont!=0 and self.Dynamics==True:
                    pos_act = previous_pos
                    vel_act = previous_vel

                ik, pos_prev, vel_prev = self.move_arm_to_pose(arm, pose, pos_act, vel_act, acc, threshold)
                
                # Detectamos las colisiones del movimiento generado antes de avanzar en la simulacion
                self.detect_autocollisions()
                # Registramos los ik del brazo
                self.ik_data(ik, arm)

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

            # Print ik solution 
            # self.print_ik_data(arm) 
            

        if arm == "both":
            # ROS
            # Activamos ambos publicadores
            self.pub_right, self.pub_left = True, True 

            if poses2 is None:
                raise ValueError("Debes proporcionar poses2 para mover ambos brazos")


            # Comprobar toda la trayectoria antes de mover el brazo
            print('Comprobando que las trayectorias introducidas son válidas..............')

            for pose_left, pose_right in zip(poses, poses2):

                achievable_left, ik_sol = self.pose_is_achievable('left', pose_left)
                achievable_right, ik_sol = self.pose_is_achievable('right', pose_right)
                if not achievable_left:
                    raise ValueError("Error: pose no alcanzable por el brazo izquierdo")
                if not achievable_right:
                    raise ValueError("Error: pose no alcanzable por el brazo derecho")

            print(f'Trayectoria válida para ambos brazos')
            for pose_left, pose_right in zip(poses, poses2):
                self.detect_autocollisions()
                ik, pos_left, vel_prev = self.move_arm_to_pose("left", pose_left, pos_act, vel_act, acc, threshold)
                self.ik_data(ik, "left")
                self.left_joints.append(pos_left)
                ik, pos_right, vel_prev = self.move_arm_to_pose("right", pose_right, pos_act, vel_act, acc, threshold)
                self.ik_data(ik, "right")
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

