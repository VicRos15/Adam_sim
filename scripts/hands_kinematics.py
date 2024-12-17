import pybullet as p
import pybullet_data
from adam import ADAM 


class HandsKinematics(ADAM):
    def __init__(self, urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True):
        super().__init__(urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True)


    # Cinematica directa
    def calculate_hand_forward_kinematics(self, hand):
        pos, ori = self.get_hand_ee_pose(hand)

        return pos, ori


    # Cinematica inversa
    # def calculate_hand_inverse_kinematics(self, hand):

    # def get_hand_ee_pose(self, hand, finger):

    #     if hand == "right":
    #         end_effector_index = 
    #     elif hand == "left":
    #         end_effector_index = 
    #     else:
    #         raise ValueError("La mano debe ser 'left' o 'right'")
        
    #     link_state = p.getLinkState(self.robot_id, end_effector_index)
    #     return link_state[4], link_state[5]

        



        
