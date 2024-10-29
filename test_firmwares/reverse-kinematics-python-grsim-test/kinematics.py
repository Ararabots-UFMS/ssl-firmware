from math import pi, sin, cos
import numpy as np

######## Velocidade no eixo x, y e angular
#velocities = np.matrix([[-1], [1], [0]])

def reverse_kinematics(velocities):
    
    velocities = np.matrix([velocities[0], velocities[1], velocities[2]]).T
    
    #Wheel radius
    r = 0.027
    
    #Radius of the robot (from center to wheel)
    R = 0.09

    ######## Angulos das rodas em relação ao eixo x
    ######## 0 -> 30 graus front right
    ######## 1 -> 150 graus front left
    ######## 2 -> 225 graus back left
    ######## 3 -> 315 graus back right
    wheel_alngles = [pi/6, (pi*5)/6, (pi*5)/4, (pi*7)/4]

    ######## Matriz Jacobiana
    jacobian = np.matrix([[cos(wheel_alngles[0]), sin(wheel_alngles[0]), R],
                        [cos(wheel_alngles[1]), sin(wheel_alngles[1]), R],
                        [cos(wheel_alngles[2]), sin(wheel_alngles[2]), R],
                        [cos(wheel_alngles[3]), sin(wheel_alngles[3]), R],])

    wheels_acc = ((1/r)*jacobian*velocities).T.tolist()[0]
    
    return [wheels_acc[1], wheels_acc[2], wheels_acc[3], wheels_acc[0]]