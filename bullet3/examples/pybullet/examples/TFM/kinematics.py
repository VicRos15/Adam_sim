from Adam import ADAM 
import pybullet as p

#Class for kinematics
class Kinematics(ADAM):
    def __init__(self, urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True):
        super().__init__(urdf_path, robot_stl_path, useSimulation, useRealTimeSimulation, used_fixed_base=True)


    # Cinemática directa
    def Calculate_forward_kinematics(self, arm):
        pos, ori = self.get_end_effector_pose(arm)

        return pos, ori
    
    # Cálculo de la cinématica inversa
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