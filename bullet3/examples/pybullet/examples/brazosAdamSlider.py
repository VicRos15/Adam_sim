import pybullet as p
import pybullet_data
import time
import numpy as np

# Inicializar PyBullet
physicsClient = p.connect(p.GUI)  # GUI para ver la simulación
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # Ruta adicional para recursos de PyBullet
p.setGravity(0, 0, -9.81)

# Cargar el robot desde el archivo URDF
robot_urdf_path = "/home/nox/Escritorio/paquetes_simulacion/rb1_base_description/robots/robot.urdf"
robot_id = p.loadURDF(robot_urdf_path, useFixedBase=True)

# Definir el índice de los brazos (esto depende de tu URDF)
ur3_left_arm_joints = [22,23,24,25,26,27]  # Reemplazar por los índices correctos de tu brazo izquierdo
ur3_right_arm_joints = [12,13,14,15,16,17]  # Reemplazar por los índices correctos de tu brazo derecho

# Crear sliders para controlar las articulaciones
num_joints = p.getNumJoints(robot_id)
slider_ids = []
joint_range = np.pi

for i,ii in enumerate(ur3_left_arm_joints + ur3_right_arm_joints):
    joint_info = p.getJointInfo(robot_id, ii)
    joint_name = joint_info[1].decode("utf-8")
    if joint_info[2] == p.JOINT_REVOLUTE:
        slider_ids.append(p.addUserDebugParameter(joint_name, -joint_range, joint_range, 0))

# Función para aplicar los valores de los sliders a las articulaciones
def apply_slider_values():
    for i, joint_id in enumerate(ur3_left_arm_joints + ur3_right_arm_joints):
        slider_value = p.readUserDebugParameter(slider_ids[i])
        p.setJointMotorControl2(robot_id, joint_id, p.POSITION_CONTROL, slider_value)

# Cinemática inversa (IK) para el brazo izquierdo
def move_left_arm_to_position(target_position, target_orientation):
    end_effector_index = ur3_left_arm_joints[-1]  # Última articulación del brazo izquierdo
    ik_solution = p.calculateInverseKinematics(robot_id, end_effector_index, target_position, target_orientation)
    for i, joint_id in enumerate(ur3_left_arm_joints):
        p.setJointMotorControl2(robot_id, joint_id, p.POSITION_CONTROL, ik_solution[i])

# Cinemática inversa (IK) para el brazo derecho
def move_right_arm_to_position(target_position, target_orientation):
    end_effector_index = ur3_right_arm_joints[-1]  # Última articulación del brazo derecho
    ik_solution = p.calculateInverseKinematics(robot_id, end_effector_index, target_position, target_orientation)
    for i, joint_id in enumerate(ur3_right_arm_joints):
        p.setJointMotorControl2(robot_id, joint_id, p.POSITION_CONTROL, ik_solution[i])

# Ejecutar la simulación
while True:
    #apply_slider_values()
    
    """ # Ejemplo de uso de cinemática inversa para mover el brazo izquierdo
    target_pos = [0.4, 0.2, 0.3]  # Objetivo en coordenadas (x, y, z)
    target_ori = p.getQuaternionFromEuler([0, 0, 0])  # Orientación deseada
    move_left_arm_to_position(target_pos, target_ori) """
    
    for i in range(num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        print(f"Índice de articulación: {i}")
        print(f"  Nombre: {joint_info[1].decode('utf-8')}")
        print(f"  Tipo de articulación: {joint_info[2]}")
        print(f"  Índice del padre: {joint_info[16]}")
        print(f"  Rango de movimiento: {joint_info[8], joint_info[9]}")
        print("-" * 30)
    p.stepSimulation()
    time.sleep(1.0/240.0)  # Control de la velocidad de simulación

# Desconectar PyBullet al finalizar
p.disconnect()
