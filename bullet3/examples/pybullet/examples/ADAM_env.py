import pybullet as p
import pybullet_data
import numpy as np
import math
import time



class ADAM:
    def __init__(self, urdf_path, useSimulation, useRealTimeSimulation, used_fixed_base=True):
        # Inicializar PyBullet y cargar el robot
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        #Change simulation mode
        self.useSimulation = useSimulation
        self.useRealTimeSimulation = useRealTimeSimulation
        self.t = 0.01

        self.robot_id = p.loadURDF(urdf_path, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT) #flags=p.URDF_USE_SELF_COLLISION,# flags=p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT) # Cambiar la posición si es necesario

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
        self.left_body = [17,30,37]  # Hombro izquierdo
        self.right_body = [17,19,26] #Hombro derecho
        # self.right_avoid_joints = [0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,20,21,22,23,24,25,26,27,28,29,30,37,38,39]  # Articulaciones a evitar para la detección de colisiones

        # #Activar las colisiones de todo el robot
        # for i in range(p.getNumJoints(self.robot_id)):
        #     p.setCollisionFilterPair(self.robot_id, self.robot_id, -1, i, 1)

        # Desactivar colisiones para el joint 17 con otras articulaciones de los brazos
        # p.setCollisionFilterPair(self.robot_id, self.robot_id, linkIndexA=17, linkIndexB=20, enableCollision=0)
        # p.setCollisionFilterPair(self.robot_id, self.robot_id, linkIndexA=17, linkIndexB=31, enableCollision=0)
        # p.setCollisionFilterPair(self.robot_id, self.robot_id, linkIndexA=17, linkIndexB=21, enableCollision=0)
        # p.setCollisionFilterPair(self.robot_id, self.robot_id, linkIndexA=17, linkIndexB=32, enableCollision=0)


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

        # Colisiones entre brazo izquierdo y el cuerpo
        for left_joint in self.ur3_left_arm_joints:
            for body_joint in self.body_joints:
                if body_joint not in self.ur3_left_arm_joints and body_joint not in self.left_body:
                    contact_points = p.getClosestPoints(self.robot_id, self.robot_id, distance=0.001, linkIndexA=left_joint, linkIndexB=body_joint)
                    if len(contact_points) > 0:
                        print(f"Colisión detectada entre brazo izquierdo y el cuerpo en los joints: {left_joint} y {body_joint}")
                        return True

        # Colisiones entre brazo derecho y el cuerpo
        for right_joint in self.ur3_right_arm_joints:
            for body_joint in self.body_joints:
                if body_joint not in self.ur3_right_arm_joints and body_joint not in self.right_body:
                    contact_points = p.getClosestPoints(self.robot_id, self.robot_id, distance=0.001, linkIndexA=right_joint, linkIndexB=body_joint)
                    if len(contact_points) > 0:
                        print(f"Colisión detectada entre brazo derecho y el cuerpo en los joints: {right_joint} y {body_joint}")
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

            # Avanzar la simulación para que los movimientos se apliquen
            if not self.useRealTimeSimulation:
                p.stepSimulation()
                time.sleep(self.t)

    
    
    #! START SIMULATION
    def run_simulation(self):

        p.setRealTimeSimulation(self.useRealTimeSimulation)

        # Función principal de simulación
        box_id = p.loadURDF("cube.urdf", [1, -0.5, 0.5],useFixedBase=True)  # Cambiar la posición si es necesario

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

            #Actualizar los valores del slide
            self.apply_slider_values()
            # self.detect_autocollisions()
            left_collision, right_collision = self.detect_collision_with_objects(box_id)

            if left_collision:
                print("Colision del cubo con brazo izquierdo")
            if right_collision:
                print("Colision del cubo con brazo derecho")

            if not self.useRealTimeSimulation:
                time.sleep(self.t)


# Ejemplo de uso:
robot_urdf_path = "/home/victor/TFM/Adam_sim/paquetes_simulacion/rb1_base_description/robots/robot.urdf"
adam_robot = ADAM(robot_urdf_path,1,0)
adam_robot.create_sliders()

adam_robot.run_simulation()
