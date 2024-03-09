#!/usr/bin/env python3
import rospy
from math import atan2, pi, sin, cos, pow, sqrt
from decimal import Decimal, getcontext


from dataclasses import dataclass
@dataclass
class EndEffectorWorkingSpace:
    # Pos of The P2, which is the one we resolved for
    Pee_x: float
    Pee_y: float
    Pee_z: float

class ComputeIk():

    def __init__(self, DH_parameters):

        # DH parameters
        self.DH_parameters_ = DH_parameters

    def get_dh_param(self, name):

        if name in self.DH_parameters_:
            return self.DH_parameters_[name]
        else:
            assert False, "Asked for Non existen param DH name ="+str(name)

    def compute_ik(self, end_effector_pose, theta_2_config = "plus", theta_3_config = "plus"):
        
        # Initialization
        Pee_x = end_effector_pose.Pee_x
        Pee_y = end_effector_pose.Pee_y
        Pee_z = end_effector_pose.Pee_z


        # We get all the DH parameters
        r1 = self.get_dh_param("r1")
        r2 = self.get_dh_param("r2")
        r3 = self.get_dh_param("r3")

        #print("Input Data===== theta_2_config CONFIG == "+str(theta_2_config))
        #print("Input Data===== theta_3_config CONFIG == "+str(theta_3_config))
        #print("Pee_x = "+str(Pee_x))
        #print("Pee_y = "+str(Pee_y))
        #print("Pee_z = "+str(Pee_z))
        #print("r1 = "+str(r1))
        #print("r2 = "+str(r2))
        #print("r3 = "+str(r3))

        # We declare all the equations for theta1, theta2, theta3 and auxiliary
        #########################################################################
        # theta_1

        #espacio de trabajo del robot
        D = sqrt(pow(Pee_x,2.0) + pow(Pee_y,2.0) + pow(Pee_z,2.0))
        r_work = r2 + r3

        possible_solution = False
        if (D <= r_work ):
            possible_solution = True
        else:
            print("Fuera de espacio de trabajo")
            pass
        

        theta_array = []

        if possible_solution:

            if theta_2_config == "plus":

                # theta 1
                theta_1 = atan2(Pee_y, Pee_x)

            elif theta_2_config == "minus":
                 # theta 1
                theta_1 = atan2(Pee_y, Pee_x) - pi
            #########################################################################

            # theta 3

            E = (pow(D,2.0)-pow(r2,2.0)-pow(r3,2.0))/(2.0*r2*r3)
            if theta_3_config == "plus":
                numerator_3 = sqrt(1.0- pow(E,2))
            
            elif theta_3_config == "minus":
                numerator_3 = -1.0 *  sqrt(1.0- pow(E,2))
            
            denominator_3 = E
            
            theta_3 = atan2(numerator_3, denominator_3)

            #limites
            lim_inf_theta_3 = -3.0*pi/4.0
            lim_sup_theta_3 = 3.0*pi/4.0

            if(theta_3<= lim_inf_theta_3 or theta_3>= lim_sup_theta_3):
                possible_solution = False



            #########################################################################

            # theta 2

            F =  pow(Pee_x,2.0)+pow(Pee_y,2.0)

            numerator_2 = Pee_z
            if theta_2_config == "plus":
                
                denomminadr_2 = sqrt(F)
                
       
            elif theta_2_config == "minus":
                denomminadr_2 = -sqrt(F)
                

            theta_2 = atan2(numerator_2, denomminadr_2)  - atan2(r3*sin(theta_3), r2 + r3*cos(theta_3))

            #normalizo el angulo
            if(theta_2 > pi):
                theta_2 = theta_2 -2*pi
            if(theta_2 < -pi):    
                theta_2 = theta_2 + 2*pi
            #limites
            lim_inf_theta_2 = -pi/4
            lim_sup_theta_2 = 5.0*pi/4
            
            if(theta_2<= lim_inf_theta_2 or theta_2>= lim_sup_theta_2):
                possible_solution = False

            #########################################################################

            theta_array = [theta_1, theta_2, theta_3]

        return theta_array, possible_solution
        

def calculate_ik(Pee_x, Pee_y, Pee_z, DH_parameters, elbow_config = "plus-minus"):

    ik = ComputeIk(DH_parameters = DH_parameters)
    
    elbow_config_string = elbow_config
    policies = elbow_config_string.split("-")

    # La variable 'result' ahora será una lista con dos strings
    # Puedes acceder a cada uno de ellos por índice
    theta_2_config = policies[0]
    theta_3_config = policies[1]


    end_effector_pose = EndEffectorWorkingSpace(Pee_x = Pee_x,
                                                Pee_y = Pee_y,
                                                Pee_z = Pee_z)

                                
    thetas, possible_solution = ik.compute_ik(end_effector_pose=end_effector_pose,theta_2_config = theta_2_config, theta_3_config = theta_3_config)
    print("Angles thetas solved ="+str(thetas))
    print("possible_solution = "+str(possible_solution))

    return thetas, possible_solution


if __name__ == '__main__':
    

    r1 = 0.0
    r2 = 1.0
    r3 = 1.0

    # Parametros DH
    DH_parameters={"r1":r1,
                    "r2":r2,
                    "r3":r3}
    # Punto de efector final
    Pee_x = 0.5
    Pee_y = 0.6
    Pee_z = 0.7

    rospy.init_node('ik_kinematics_node', anonymous=True)
    try:
        calculate_ik(Pee_x=Pee_x, Pee_y=Pee_y, Pee_z=Pee_z ,DH_parameters=DH_parameters, elbow_config = "plus-plus")
        calculate_ik(Pee_x=Pee_x, Pee_y=Pee_y, Pee_z=Pee_z ,DH_parameters=DH_parameters, elbow_config = "plus-minus")
        calculate_ik(Pee_x=Pee_x, Pee_y=Pee_y, Pee_z=Pee_z ,DH_parameters=DH_parameters, elbow_config = "minus-plus")
        calculate_ik(Pee_x=Pee_x, Pee_y=Pee_y, Pee_z=Pee_z ,DH_parameters=DH_parameters, elbow_config = "minus-minus")
    except rospy.ROSInterruptException:
        pass