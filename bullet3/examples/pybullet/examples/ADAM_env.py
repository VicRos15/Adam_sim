import pybullet as p
import pybullet_data
import numpy as np
import math
import time



class ADAM:
    def __init__(self, urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True):
        # Inicializar PyBullet y cargar el robot
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        #Change simulation mode
        self.useSimulation = useSimulation
        self.useRealTimeSimulation = useRealTimeSimulation
        self.t = 0.1

        self.robot_id = p.loadURDF(urdf_path, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT) #flags=p.URDF_USE_SELF_COLLISION,# flags=p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT) # Cambiar la posición si es necesario
        
        #Creamos el objeto sin hombros
        self.robot_shape = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                            fileName=robot_stl_path,
                                            meshScale=[1, 1, 1])
        self.robot_visual_shape = p.createVisualShape(shapeType=p.GEOM_MESH,
                                                    fileName=robot_stl_path,
                                                    meshScale=[1, 1, 1])  # Ajusta el escalado 
        self.robot_stl_id = p.createMultiBody(baseMass=0,              
                                            baseCollisionShapeIndex=self.robot_shape,
                                            baseVisualShapeIndex=self.robot_visual_shape,
                                            basePosition=[-0.10, 0, 0.73])    # Cambia la posición 

        #Definir null space
        #lower limits for null space
        self.ll = [-3.14]*6
        #upper limits for null space
        self.ul = [3.14]*6
        #joint ranges for null space
        self.jr = [6.28]*6
        #restposes for null space
        self.rp = [0]*6
        #joint damping coefficents
        self.jd = [0.1]*6

        # Definir los índices de los brazos (esto depende de tu URDF)
        self.ur3_right_arm_joints = [20,21,22,23,24,25]  # Brazo derecho
        self.ur3_left_arm_joints = [31,32,33,34,35,36]  # Brazo izquierdo

        self.body_joints = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,26,27,28,29,30,37,38,39] #Cuerpo 
        self.joints=[0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31,32,33,34,35,36,37,38,39]

        self.arm_joints = 6


        #Current pos, vel
        self.pos_act = []
        self.vel_act = []
        self.acc_joints = []


        #Calculo de la dinamica inversa
        self.InverseDynamics = True



        # p.setCollisionFilterPair(bodyUniqueIdA=self.robot_id,bodyUniqueIdB=self.robot_id, linkIndexA=17, linkIndexB=20, enableCollision=0, physicsClientId=self.physicsClient)
        # p.setCollisionFilterPair(bodyUniqueIdA=self.robot_id, bodyUniqueIdB=self.robot_id, linkIndexA=17, linkIndexB=31, enableCollision=0, physicsClientId=self.physicsClient)


    # Inverse dynamics
    def Calculate_inverse_dynamics (self,target_pose, arm, offset):
        #Calculate current pos, vel 
        if arm == "right" or arm == "left":
            pos_act,vel_act = self.get_joints_pos_vel(arm)
        else:
            raise ValueError("El brazo debe ser 'left' o 'right'.")
        

        # if len(target_pose) < len(self.ur3_left_arm_joints):
        #     raise ValueError("target_pose no tiene suficientes elementos para el número de articulaciones.")
        
        # Goal pose
        pos_des = []
        for i in range(len(self.ur3_left_arm_joints)):
            pos_des.append(target_pose[i])

        #Implement a PD controller to calculate desired acceleration
        #PD parameters
        kp = 300
        kd = 10
        #PD controller
        acc_des=[]
        for i in range(len(vel_act)):
            acc_des.append(kp*(pos_des[i]-pos_act[i])-kd*vel_act[i])

        #Calculate inverse dynamics
        try:
            torque_IK = list(p.calculateInverseDynamics(self.robot_id, pos_act, vel_act, acc_des, flags=1))
        except Exception as e:
            raise SystemError(f"Error en calculateInverseDynamics: {e}")

        # Compare torque calculated with the maximum torque 
        for i, torque in enumerate(torque_IK):
            joint_info = p.getJointInfo(self.robot_id,i)
            max_torque = joint_info[10]

            if abs(torque) > max_torque:
                torque_IK[i] = max_torque if torque > 0 else -max_torque

        return torque_IK
    

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


    def detect_autocollisions(self):
        # Colisiones entre los brazos
        for left_joint in self.ur3_left_arm_joints:
            for right_joint in self.ur3_right_arm_joints:
                contact_points = p.getClosestPoints(self.robot_id, self.robot_id, distance=0.001, linkIndexA=left_joint, linkIndexB=right_joint)
                if len(contact_points) > 0:
                    print("Colisión entre brazos detectada")
                    return True
                

        #Colisiones cuerpo-brazo izquierdo
        for left_joint in self.ur3_left_arm_joints:
            contact_points = p.getClosestPoints(self.robot_id, self.robot_stl_id, distance=0, linkIndexA=left_joint)
            if len(contact_points) > 0:
                print("Colisión entre brazo izq-cuerpo")
                return True
        
        #Colisiones cuerpo-brazo derecho
        for right_joint in self.ur3_right_arm_joints:
            contact_points = p.getClosestPoints(self.robot_id, self.robot_stl_id, distance=0, linkIndexA=right_joint)
            if len(contact_points) > 0:
                print("Colisión entre brazo der-cuerpo")
                return True

        return False  # No hay colisiones


    
    def detect_collision_with_objects(self, object_id):
        #! TODO: Ver si se quiere dectectar la colision con el rest odel cuerpo
        # Detectar colisiones del brazo izquierdo o derecho con otros objetos en la escena
        left_arm_collision = False
        right_arm_collision = False

        # Comprobar colisiones del brazo izquierdo con el objeto
        for left_joint in self.ur3_left_arm_joints:
            contact_points = p.getClosestPoints(self.robot_id, object_id, distance=0, linkIndexA=left_joint)
            if len(contact_points) > 0:
                left_arm_collision = True

        # Comprobar colisiones del brazo derecho con el objeto
        for right_joint in self.ur3_right_arm_joints:
            contact_points = p.getClosestPoints(self.robot_id, object_id, distance=0, linkIndexA=right_joint)
            if len(contact_points) > 0:
                right_arm_collision = True


        #cOMPROBAR OBJETO CON CUERPO
        for right_joint in self.ur3_right_arm_joints:
            contact_points = p.getClosestPoints(self.robot_id, object_id, distance=0, linkIndexA=right_joint)
            if len(contact_points) > 0:
                right_arm_collision = True
 
        #Que nos devuelva puntos de contacto(articulaciones) y además un true o false
        return left_arm_collision, right_arm_collision
    
    
    def get_end_effector_pose(self, arm):
        # Obtener la posición del efector final del brazo
        if arm == "left":
            end_effector_index = self.ur3_left_arm_joints[-1]
        elif arm == "right":
            end_effector_index = self.ur3_right_arm_joints[-1]
        else:
            raise ValueError("El brazo debe ser 'left' o 'right'.")

        link_state = p.getLinkState(self.robot_id, end_effector_index)
        return link_state[4],link_state[5]
    
    
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


    #Cálculo de la cinématica inversa
    def Calculate_inverse_kinematics(self,robot_id, ee_index, target_position, target_orientation, null=True):
        
        if null:
            ik_solution= p.calculateInverseKinematics(robot_id, ee_index, target_position, target_orientation,lowerLimits=self.ll,
                                                    upperLimits=self.ul,
                                                    jointRanges=self.jr,
                                                    restPoses=self.rp)
        else:
            ik_solution= p.calculateInverseKinematics(robot_id, ee_index, target_position, target_orientation,lowerLimits=None,
                                                    upperLimits=None,
                                                    jointRanges=None,
                                                    restPoses=None)
        return ik_solution
    

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


    def print_robot_info(self):
        num_joints = p.getNumJoints(self.robot_id)
        print(f"Robot ID: {self.robot_id}")
        print("Elementos del robot:")
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_id = joint_info[0]
            joint_name = joint_info[1].decode("utf-8")
            print(f"ID: {joint_id}, Nombre: {joint_name}")

    
    
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

            if not self.useRealTimeSimulation:
                time.sleep(self.t)


# Programa principal
robot_urdf_path = "/home/victor/TFM/Adam_sim/paquetes_simulacion/rb1_base_description/robots/robot.urdf"
robot_stl_path = "/home/victor/TFM/Adam_sim/paquetes_simulacion/rb1_base_description/meshes/others/torso_sin_hombros.stl"


adam_robot = ADAM(robot_urdf_path,robot_stl_path,0,1)
adam_robot.create_sliders()

adam_robot.run_simulation()
