import pybullet as p
import pybullet_data


class Hands:
    def __init__(self, urdf_path, useSimulation, useRealTimeSimulation):
        
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        #Change simulation mode
        self.useSimulation = useSimulation
        self.useRealTimeSimulation = useRealTimeSimulation
        self.t = 0.05

        self.hands_id = p.loadURDF(urdf_path, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT)


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
        self.jd = [0.1]*21


    #Funciones para las manos (rellenar)
