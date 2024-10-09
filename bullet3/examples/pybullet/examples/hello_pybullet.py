import pybullet as p
from time import sleep
import pybullet_data


physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

p.setGravity(0, 0, -9.81)
planeId = p.loadURDF("plane.urdf") #*Cargamos el plano
cubeStartPos = [0, 0, 0]
cubeStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
#robot = p.loadURDF("/home/nox/Escritorio/seguridad ficheros/rb1_base_description/robots/robot_prueba.urdf", cubeStartPos, cubeStartOrientation,flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)
robot = p.loadURDF("/home/nox/Escritorio/seguridad ficheros/rb1_base_description/urdf/bases/robot_pruebaV2.urdf", cubeStartPos, cubeStartOrientation)
#robot = p.loadURDF("/home/nox/Escritorio/seguridad ficheros/ur_description/urdf/brazitos_prueba.urdf", cubeStartPos, cubeStartOrientation,flags=p.URDF_USE_SELF_COLLISION_EXCLUDE_ALL_PARENTS)
robotPos, robotOr = p.getBasePositionAndOrientation(robot)
useRealTimeSimulation = 0

print(p.getNumJoints(robot))

if (useRealTimeSimulation):
  p.setRealTimeSimulation(1)

while 1:
  if (useRealTimeSimulation):
    p.setGravity(0, 0, -9.81)
    sleep(0.01)  # Time in seconds.
  else:
    p.stepSimulation()
    sleep(1./240.)
    
  #print("Posicion del robot:", robotPos)
  #print("Brazos:, ", p.getJointInfo(robot, 17))
