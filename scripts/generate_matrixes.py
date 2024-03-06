#!/usr/bin/env python3

from sympy import Matrix, cos, sin, Symbol, simplify, trigsimp
from sympy.interactive import printing
from sympy import preview
from math import pi
import os


# To make display prety
printing.init_printing(use_latex = True)

class MatrixGerarate(object):


    theta_i = Symbol("theta_i")
    alpha_i = Symbol("alpha_i")
    r_i = Symbol("r_i")
    d_i = Symbol("d_i")
    # Variable thetas
    theta_1 = Symbol("theta_1")
    theta_2 = Symbol("theta_2")
    theta_3 = Symbol("theta_3")
    # Constans
    alpha_1 = pi/2
    alpha_2 = 0.0
    alpha_3 = 0.0

    r_1 = Symbol("r1")
    r_2 = Symbol("r2")
    r_3 = Symbol("r3")


    d_1 = 0.0
    d_2 = 0.0
    d_3 = 0.0

    def __init__(self) :


        self.DH_Matric_Generic = Matrix([[cos(self.theta_i), -sin(self.theta_i)*cos(self.alpha_i), sin(self.theta_i)*sin(self.alpha_i), self.r_i*cos(self.theta_i)],
                                    [sin(self.theta_i), cos(self.theta_i)*cos(self.alpha_i), -cos(self.theta_i)*sin(self.alpha_i), self.r_i*sin(self.theta_i)],
                                    [0, sin(self.alpha_i), cos(self.alpha_i), self.d_i],
                                    [0,0,0,1]])
        result_simpl = simplify(self.DH_Matric_Generic)
        

        
    def matrix_generate(self):

        A01 = self.DH_Matric_Generic.subs(self.r_i,self.r_1).subs(self.alpha_i,self.alpha_1).subs(self.d_i,self.d_1).subs(self.theta_i, self.theta_1)
        A12 = self.DH_Matric_Generic.subs(self.r_i,self.r_2).subs(self.alpha_i,self.alpha_2).subs(self.d_i,self.d_2).subs(self.theta_i, self.theta_2)
        A23 = self.DH_Matric_Generic.subs(self.r_i,self.r_3).subs(self.alpha_i,self.alpha_3).subs(self.d_i,self.d_3).subs(self.theta_i, self.theta_3)


    
        A03 = A01 * A12 * A23
        A13 = A12 * A23
        
        # Simplify matrix
        A01_simplify = trigsimp(A01)
        A03_simplify = trigsimp(A03)
        A12_simplify = trigsimp(A12)
        A13_simplify = trigsimp(A13)
        A23_simplify = trigsimp(A23)

        matrix = dict()
        matrix['A01'] = A01_simplify
        matrix['A03'] = A03_simplify
        matrix['A12'] = A12_simplify
        matrix['A13'] = A13_simplify
        matrix['A23'] = A23_simplify

        return matrix


    def print_matrix(self):

        matrix = self.matrix_generate()
        A01_simplify = matrix['A01']
        A03_simplify = matrix['A03']
        A12_simplify = matrix['A12']
        A13_simplify = matrix['A13']
        A23_simplify = matrix['A23']

        if os.path.exists("A01.png"):
            os.remove("A01.png")
        if os.path.exists("A03.png"):
            os.remove("A03.png")
        if os.path.exists("A12.png"):
            os.remove("A12.png")
        if os.path.exists("A13.png"):
            os.remove("A13.png")
        if os.path.exists("A23.png"):
            os.remove("A23.png")

        preview(A01_simplify, viewer='file', filename="A01.png", dvioptions=['-D','300'])
        preview(A03_simplify, viewer='file', filename="A03.png", dvioptions=['-D','300'])
        preview(A12_simplify, viewer='file', filename="A12.png", dvioptions=['-D','300'])
        preview(A13_simplify, viewer='file', filename="A13.png", dvioptions=['-D','300'])
        preview(A23_simplify, viewer='file', filename="A23.png", dvioptions=['-D','300'])



if __name__ == '__main__':
    matrix_generate = MatrixGerarate()
    try:
        matrix_generate.print_matrix()
    except Exception as e:
            print(f"Error: {e}")