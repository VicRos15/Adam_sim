import pybullet as p
import pybullet_data


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
        self.InverseDynamics = False



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

    def print_robot_info(self):
        num_joints = p.getNumJoints(self.robot_id)
        print(f"Robot ID: {self.robot_id}")
        print("Elementos del robot:")
        for i in range(num_joints):
            joint_info = p.getJointInfo(self.robot_id, i)
            joint_id = joint_info[0]
            joint_name = joint_info[1].decode("utf-8")
            print(f"ID: {joint_id}, Nombre: {joint_name}")