import pybullet as p
import pybullet_data
import numpy as np
import math
import time

class ADAM:
    def __init__(self, urdf_path,used_fixed_base=True):
        # Inicializar PyBullet y cargar el robot
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        self.robot_id = p.loadURDF(urdf_path, useFixedBase=True)  # Cambiar la posición si es necesario

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
        self.ur3_left_arm_joints = [31,32,33,34,35,36]  # Brazo izquierdo
        self.ur3_right_arm_joints = [20,21,22,23,24,25]  # Brazo derecho
        self.left_avoid_joints = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,37,38,39]  # Articulaciones a evitar para la detección de colisiones
        self.right_avoid_joints = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,37,38,39]  # Articulaciones a evitar para la detección de colisiones

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
                print("Colisión detectada.")

            # Avanzar la simulación para que los movimientos se apliquen
            p.stepSimulation()
            time.sleep(10.0 / 240.0)

    def detect_autocollisions(self):        
        #! TODO: Detectar colisiones con el cuerpo del robot
        #* SOLUTION: Hacer bloques de cuerpos para detectar collisiones, en lugar de i not in , hace i in
        # Detectar colisiones entre los brazos y el cuerpo del robot
        for left_joint in self.ur3_left_arm_joints:
            for right_joint in self.ur3_right_arm_joints:
                contact_points = p.getClosestPoints(self.robot_id, self.robot_id, distance=0, linkIndexA=left_joint, linkIndexB=right_joint)
                if len(contact_points) > 0:
                    print("Colision entre brazos")
                    return True  # Detecta colisión entre el brazo izquierdo y el derecho

        #* Detectar colisiones del brazo izquierdo con el cuerpo (excluyendo sus propias articulaciones)
        for left_joint in self.ur3_left_arm_joints:
            for i in range(p.getNumJoints(self.robot_id)):
                if i not in self.ur3_left_arm_joints and i not in self.left_avoid_joints:  # Evitar autocolisión con el mismo brazo y excluimos el valor de la articulación 9
                    print("Estoy petando aqui:",i)
                    contact_points = p.getClosestPoints(self.robot_id, self.robot_id, distance=0, linkIndexA=left_joint, linkIndexB=i)
                    if len(contact_points) > 0:
                        print("Colision izquierdo")
                        return True  # Detecta colisión entre el brazo izquierdo y el cuerpo

        #* Detectar colisiones del brazo derecho con el cuerpo (excluyendo sus propias articulaciones)
        for right_joint in self.ur3_right_arm_joints :
            for i in range(p.getNumJoints(self.robot_id)):
                if i not in self.ur3_right_arm_joints and i not in self.right_avoid_joints:  # Evitar autocolisión con el mismo brazo
                    contact_points = p.getClosestPoints(self.robot_id, self.robot_id, distance=0, linkIndexA=right_joint, linkIndexB=i)
                    if len(contact_points) > 0:
                        print("Colision derecho")
                        return True  # Detecta colisión entre el brazo derecho y el cuerpo
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
    
    
    def get_joint_pose(self, arm):
        joint_positions = {}

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
            joint_positions[joint_id] = joint_state[0]  # La posición de la articulación está en el índice 0

        return joint_positions




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

        # for joint_i in avoid_joints:
        #     p.resetJointState(self.robot_id, joint_i, 0)  # Establecer la posición fija deseada
        #     p.setJointMotorControl2(self.robot_id, joint_i, p.POSITION_CONTROL, targetPosition=0, force=500) #mantener las articulaciones restantes rígidas
        

        # Inverse kinematics for the UR3 robot
        ik_solution = self.Calculate_inverse_kinematics(self.robot_id, joint_indices[-1], target_position, target_orientation)
        
        # Comprobar si la solución es válida (verificar si hay posiciones NaN)
        if ik_solution is None or any([math.isnan(val) for val in ik_solution]):
            print(f"Posición no alcanzable por el brazo {arm}.")
            return False
        # Si la solución es válida, mover el brazo
        else:
            for i, joint_id in enumerate(joint_indices):
                p.setJointMotorControl2(self.robot_id, joint_id, p.POSITION_CONTROL, ik_solution[i+offset_iksol])
                print(f"Brazo {arm} movido a la posición: {target_position}.")
            return True

    
    def move_arm_to_multiple_poses(self, arm, poses):
    
        for pose in poses:
            self.move_arm_to_pose(arm, pose)

            #Pequeño delay para apreciar el movimiento entre cada una de las posiciones comandadas
            p.stepSimulation()
            # time.sleep(100.0 / 240.0)
    
    
    #! START SIMULATION
    def run_simulation(self, useSimulation, useRealTimeSimulation):
    
        p.setRealTimeSimulation(useRealTimeSimulation)

        #Inicializamos el tiempo de simulacion
        if useRealTimeSimulation:
            t = time.time() #Emplea como tiempo de simulacion el tiempo real, o tiempo del ordenador
        else:
            t = 0.01 #frecuencia de 100Hz


        # Función principal de simulación
        #box_id = p.loadURDF("cube.urdf", [0.5, 0.5, 0.5],useFixedBase=True)  # Cambiar la posición si es necesario

        #Poses comandadas
        poses = [
            [[0.4, 0.7, 0.36], p.getQuaternionFromEuler([0, -math.pi, 0])],
            [[0.45, 0.7, 0.7], p.getQuaternionFromEuler([0, -math.pi, 0])],
            [[0.5, 0.7, 0.7], p.getQuaternionFromEuler([0, -math.pi, 0])],
            [[0.4, 0.7, 0.36], p.getQuaternionFromEuler([0, -math.pi, 0])],
            [[0.45, 0.7, 0.7], p.getQuaternionFromEuler([0, -math.pi, 0])],
            [[0.5, 0.7, 0.7], p.getQuaternionFromEuler([0, -math.pi, 0])],
            [[0.4, 0.7, 0.36], p.getQuaternionFromEuler([0, -math.pi, 0])],
            [[0.45, 0.7, 0.7], p.getQuaternionFromEuler([0, -math.pi, 0])],
            [[0.5, 0.7, 0.7], p.getQuaternionFromEuler([0, -math.pi, 0])]
        ]


        while True:
            #Actualizar los valores del slide
            # self.apply_slider_values()

            joints_pos = self.get_joint_pose("left")

            self.move_arm_to_multiple_poses("left", poses)
            pos,ori = self.get_end_effector_pose("left")
            print("Posicion ee real:",pos)
            print("Orientación ee real:",ori)
            # p.stepSimulation()
            # time.sleep(10.0 / 240.0)



# #Clase KINEMATICS
# class ADAM_kinematics(ADAM):
#     def __init__(self, urdf_path, used_fixed_base=True):
#         #Constructor de la clase padre (ADAM)
#         super().__init__(urdf_path, used_fixed_base)

#     def Calculate_direct_kinematics(self, arm):
#         #Obtener la pose del efector final
#         pos, ori = self.get_end_effector_pose(arm)
#         print(f"Posición del efector final del brazo {arm}: {pos}")
#         print(f"Orientación del efector final del brazo {arm}: {ori}")
#         return pos, ori


# Ejemplo de uso:
robot_urdf_path = "/home/victor/TFM/Adam_sim/paquetes_simulacion/rb1_base_description/robots/robot.urdf"
adam_robot = ADAM(robot_urdf_path)
adam_robot.create_sliders()
# adam_kinematics = ADAM_kinematics(robot_urdf_path)

# pos,ori = adam_robot.get_end_effector_pose("left")
# joints_pos = adam_robot.get_joint_positions("left")
adam_robot.run_simulation(0,1)
