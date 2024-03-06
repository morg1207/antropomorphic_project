

#!/usr/bin/env python3

#!/usr/bin/env python3

import rospy
from sympy import Matrix, cos, sin, Symbol, simplify, trigsimp
from sympy.interactive import printing
from sympy import preview
from generate_matrixes import MatrixGerarate
from math import pi
import os



def fk_antropomorphic():

    
    theta_1 = Symbol("theta_1")
    theta_2 = Symbol("theta_2")
    theta_3 = Symbol("theta_3")
    r1 = Symbol("r1")
    r2 = Symbol("r2")
    r3 = Symbol("r3")

    matrix_generate = MatrixGerarate()
    matrix = matrix_generate.matrix_generate()
    A03 = matrix['A03']
    #constantes r
    r_1_val = 0.0
    r_2_val = 1.0
    r_3_val = 1.0 
    #obtengo los angulos
    angles = ingresar_angulos()
    theta_1_val = angles[0]
    theta_2_val = angles[1]
    theta_3_val = angles[2]
    #sustituyo los angulos
    A03_evaluated = A03.subs(theta_1,theta_1_val).subs(theta_2,theta_2_val).subs(theta_3, theta_3_val).subs(r1, r_1_val).subs(r2, r_2_val).subs(r3, r_3_val)
    print("Position : ",A03_evaluated[0:3 ,3] )
    print("Orientation : ",A03_evaluated[0:3 ,0:3] )
    

def ingresar_angulos():
    try:
        # Solicitar al usuario que ingrese el valor de los tres ángulos
        theta1 = float(input("Please enter theta 1: "))
        while(theta1>pi or theta1<-pi ):
            print("Ingrese un valor entre 3.14 y -3.14")
            theta1 = float(input("Please enter theta 1: "))
        
        theta2 = float(input("Please enter theta 2: "))
        while(theta2>pi or theta2<-pi ):
            print("Ingrese un valor entre 3.14 y -3.14")
            theta2 = float(input("Please enter theta 2: "))

        theta3 = float(input("Please enter theta 3: "))
        while(theta3>pi or theta3<-pi ):
            print("Ingrese un valor entre 3.14 y -3.14")
            theta3 = float(input("Please enter theta 3: "))

    except ValueError:
        print("Por favor, ingrese valores numéricos.")

    angles = []
    angles.append(theta1)
    angles.append(theta2)
    angles.append(theta3)
    return angles


if __name__ == '__main__':
    rospy.init_node('forward_kinematics_node', anonymous=True)
    try:
       fk_antropomorphic()
    except rospy.ROSInterruptException:
        pass