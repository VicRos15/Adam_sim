import pybullet as p
import pybullet_data
import math


#Class for ADAM info
class ADAM:
    def __init__(self, urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True):
        # Inicializar PyBullet y cargar el robot
        self.physicsClient = p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)

        #Change simulation mode
        self.useSimulation = useSimulation
        self.useRealTimeSimulation = useRealTimeSimulation
        self.t = 0.05

        self.robot_id = p.loadURDF(urdf_path, useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT) #flags=p.URDF_USE_SELF_COLLISION,# flags=p.URDF_USE_SELF_COLLISION_INCLUDE_PARENT) # Cambiar la posición si es necesario
        
        #Creamos el objeto sin hombros
        #Orientacion del stl
        rotation_quaternion = p.getQuaternionFromEuler([math.pi/2, 0, math.pi])

        self.robot_shape = p.createCollisionShape(shapeType=p.GEOM_MESH,
                                            fileName=robot_stl_path,
                                            meshScale=[1, 1, 1])
        self.robot_visual_shape = p.createVisualShape(shapeType=p.GEOM_MESH,
                                                    fileName=robot_stl_path,
                                                    meshScale=[1, 1, 1])  # Ajusta el escalado 
        self.robot_stl_id = p.createMultiBody(baseMass=0,              
                                            baseCollisionShapeIndex=self.robot_shape,
                                            baseVisualShapeIndex=self.robot_visual_shape,
                                            basePosition=[-0.10, 0.0, 0.54],
                                            baseOrientation = rotation_quaternion)    # Cambia la posición 

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
        self.Dynamics = False

    #Collisions
    def detect_autocollisions(self):
        # Colisiones entre los brazos
        for left_joint in self.ur3_left_arm_joints:
            for right_joint in self.ur3_right_arm_joints:
                contact_points = p.getClosestPoints(self.robot_id, self.robot_id, distance=0, linkIndexA=left_joint, linkIndexB=right_joint)
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


    


    def print_robot_info(self):
        num_joints = p.getNumJoints(self.robot_id)
        print(f"Robot ID: {self.robot_id}")
        print("Elementos del robot:")
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_id = joint_info[0]
            joint_name = joint_info[1].decode("utf-8")
            print(f"ID: {joint_id}, Nombre: {joint_name}")


