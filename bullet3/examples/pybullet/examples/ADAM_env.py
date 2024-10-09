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

        # Definir los índices de los brazos (esto depende de tu URDF)
        self.ur3_left_arm_joints = [31,32,33,34,35,36]  # Brazo izquierdo
        self.ur3_right_arm_joints = [20,21,22,23,24,25]  # Brazo derecho
        self.left_avoid_joints = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,37,38,39]  # Articulaciones a evitar para la detección de colisiones
        self.right_avoid_joints = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,37,38,39]  # Articulaciones a evitar para la detección de colisiones

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
            if not self.detect_autocollisions():
                p.setJointMotorControl2(self.robot_id, joint_id, p.POSITION_CONTROL, slider_value)
            else:
                p.setJointMotorControl2(self.robot_id, joint_id, p.POSITION_CONTROL, slider_value)
                print("Colisión detectada.")

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
    
    def get_end_effector_position(self, arm):
        # Obtener la posición del efector final del brazo
        if arm == "left":
            end_effector_index = self.ur3_left_arm_joints[-1]
        elif arm == "right":
            end_effector_index = self.ur3_right_arm_joints[-1]
        else:
            raise ValueError("El brazo debe ser 'left' o 'right'.")

        link_state = p.getLinkState(self.robot_id, end_effector_index)
        return link_state[4],link_state[5]
    
    def move_arm_to_position(self, arm, target_position, target_orientation=None):
        #! Inverse kinematics for the UR3 robot
        #lower limits for null space
        ll = [-3.14]*6
        #upper limits for null space
        ul = [3.14]*6
        #joint ranges for null space
        jr = [6.28]*6
        #restposes for null space
        rp = [0]*6
        #joint damping coefficents
        jd = [0.1]*6
        
        # Obtener los índices del brazo seleccionado
        if arm == "left":
            joint_indices = self.ur3_left_arm_joints
        elif arm == "right":
            joint_indices = self.ur3_right_arm_joints
        else:
            raise ValueError("El brazo debe ser 'left' o 'right'.")

        # Si no se pasa orientación, se asume una orientación por defecto (efector apuntando hacia abajo)
        if target_orientation is None:
            target_orientation = p.getQuaternionFromEuler([0, -math.pi, 0])

        for joint_i in self.left_avoid_joints:
            p.resetJointState(self.robot_id, joint_i, 0)  # Establecer la posición fija deseada
            p.setJointMotorControl2(self.robot_id, joint_i, p.POSITION_CONTROL, targetPosition=0, force=500)
        
        # Calcular la cinemática inversa
        ik_solution = p.calculateInverseKinematics(self.robot_id, joint_indices[-1], target_position, target_orientation,lowerLimits=ll,
                                                upperLimits=ul,
                                                jointRanges=jr,
                                                restPoses=rp)

        # Comprobar si la solución es válida (verificar si hay posiciones NaN)
        if ik_solution is None or any([math.isnan(val) for val in ik_solution]):
            print(f"Posición no alcanzable por el brazo {arm}.")
            return False

        # Si la solución es válida, mover el brazo
        for i, joint_id in enumerate(joint_indices):
            p.setJointMotorControl2(self.robot_id, joint_id, p.POSITION_CONTROL, ik_solution[i+8])

        print(f"Brazo {arm} movido a la posición: {target_position}.")
        return True
    
    def move_arm_to_multiple_positions(self, arm, target_positions, target_orientation=None):
    
        for target_position in target_positions:
            success = self.move_arm_to_position(arm, target_position, target_orientation)
            if not success:
                print(f"Brazo {arm} no pudo alcanzar la posición: {target_position}.")
    
    def get_joint_positions(self, arm):
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

    
    
    #! START SIMULATION
    def run_simulation(self):
        # Función principal de simulación
        #box_id = p.loadURDF("cube.urdf", [0.5, 0.5, 0.5],useFixedBase=True)  # Cambiar la posición si es necesario
        positions = [[0.17, 0.48, 1.36], [0.15, 0.48, 1.36], [0.13, 0.48, 1.36]]
        while True:
            """ self.apply_slider_values()
            left_collision, right_collision = self.detect_collision_with_objects(box_id)
    
            if left_collision:
                print("¡Colisión detectada en el brazo izquierdo!")
            if right_collision:
                print("¡Colisión detectada en el brazo derecho!") """
            joints_pos = self.get_joint_positions("left")
            print("Antes",joints_pos)
            
            self.move_arm_to_multiple_positions("left", positions)
            pos,ori = self.get_end_effector_position("left")
            print("Valores internos",pos)
            p.stepSimulation()
            time.sleep(1.0 / 240.0)
            print("Despues",joints_pos)
            


# Ejemplo de uso:
robot_urdf_path = "/home/nox/Escritorio/paquetes_simulacion/rb1_base_description/robots/robot.urdf"
adam_robot = ADAM(robot_urdf_path)
#adam_robot.create_sliders()
""" adam_robot.move_arm_to_position("left", [0.3, 0.45, 1.30])
pos,ori = adam_robot.get_end_effector_position("left")
print(pos) """
joints_pos = adam_robot.get_joint_positions("left")
print("Antes",joints_pos)
adam_robot.run_simulation()
