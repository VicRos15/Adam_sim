import pybullet as p
import time
import math
from datetime import datetime
import pybullet_data

# Conectar a PyBullet
clid = p.connect(p.SHARED_MEMORY)
if (clid < 0):
    p.connect(p.GUI)

# Cargar recursos de PyBullet
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Cargar el plano y el robot desde el URDF
p.loadURDF("plane.urdf")
robot_id = p.loadURDF("/home/nox/Escritorio/seguridad ficheros/rb1_base_description/urdf/bases/robot_pruebaV2.urdf", useFixedBase=True)
p.resetBasePositionAndOrientation(robot_id, [0, 0, 0], [0, 0, 0, 1])

# Definir los índices de las articulaciones de los brazos
ur3_left_arm_joints = [22, 23, 24, 25, 26, 27]  # Reemplazar con los índices correctos del brazo izquierdo
ur3_right_arm_joints = [12, 13, 14, 15, 16, 17]  # Reemplazar con los índices correctos del brazo derecho

# Configurar los límites de las articulaciones, rangos y posiciones de descanso
ll = [-2, -2, -2, -2, -2, -2, -2]  # Limites inferiores
ul = [2, 2, 2, 2, 2, 2, 2]         # Limites superiores
jr = [4, 4, 4, 4, 4, 4, 4]         # Rango de las articulaciones
rp = [0, 0, 0, 0, 0, 0, 0]         # Posiciones de descanso

# Coeficientes de amortiguación de las articulaciones
jd = [0.1] * len(ur3_left_arm_joints)

# Inicializar posiciones de las articulaciones de ambos brazos
for i in ur3_left_arm_joints:
    p.resetJointState(robot_id, i, rp[0])

for i in ur3_right_arm_joints:
    p.resetJointState(robot_id, i, rp[0])

p.setGravity(0, 0, -9.81)
t = 0.0
useNullSpace = 1
useOrientation = 1
useSimulation = 1
useRealTimeSimulation = 0
ikSolver = 0
p.setRealTimeSimulation(useRealTimeSimulation)
trailDuration = 15

while True:
    # Obtener el tiempo para la animación
    t += 0.01

    if (useSimulation and useRealTimeSimulation == 0):
        p.stepSimulation()

    # Definir posiciones deseadas para ambos brazos
    pos_left = [-0.4, 0.2 * math.cos(t), 0. + 0.2 * math.sin(t)]
    pos_right = [0.4, 0.2 * math.cos(t), 0. + 0.2 * math.sin(t)]
    orn = p.getQuaternionFromEuler([0, -math.pi, 0])  # End effector points down

    # Cinemática inversa para el brazo izquierdo
    if useNullSpace == 1:
        jointPoses_left = p.calculateInverseKinematics(robot_id, ur3_left_arm_joints[-1], pos_left, orn, ll, ul, jr, rp)
    else:
        jointPoses_left = p.calculateInverseKinematics(robot_id, ur3_left_arm_joints[-1], pos_left, orn)

    # Cinemática inversa para el brazo derecho
    if useNullSpace == 1:
        jointPoses_right = p.calculateInverseKinematics(robot_id, ur3_right_arm_joints[-1], pos_right, orn, ll, ul, jr, rp)
    else:
        jointPoses_right = p.calculateInverseKinematics(robot_id, ur3_right_arm_joints[-1], pos_right, orn)

    # Controlar las articulaciones del brazo izquierdo
    for i, joint_id in enumerate(ur3_left_arm_joints):
        p.setJointMotorControl2(robot_id, joint_id, p.POSITION_CONTROL, jointPoses_left[i], force=500)

    # Controlar las articulaciones del brazo derecho
    for i, joint_id in enumerate(ur3_right_arm_joints):
        p.setJointMotorControl2(robot_id, joint_id, p.POSITION_CONTROL, jointPoses_right[i], force=500)

    # Visualizar la trayectoria del efector final
    ls_left = p.getLinkState(robot_id, ur3_left_arm_joints[-1])
    ls_right = p.getLinkState(robot_id, ur3_right_arm_joints[-1])

    p.addUserDebugLine(pos_left, ls_left[4], [0, 0, 1], 1, trailDuration)
    p.addUserDebugLine(pos_right, ls_right[4], [1, 0, 0], 1, trailDuration)

    # Simulación en tiempo real
    if not useRealTimeSimulation:
        time.sleep(1.0 / 240.0)
