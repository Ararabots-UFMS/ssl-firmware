import simulation_commander
from grSim_Packet_pb2 import grSim_Packet
from grSim_Commands_pb2 import grSim_Robot_Command
from grSim_Commands_pb2 import grSim_Commands
from grSim_Replacement_pb2 import grSim_RobotReplacement
from grSim_Replacement_pb2 import grSim_BallReplacement
from grSim_Replacement_pb2 import grSim_Replacement

import kinematics

import time
from move import *
import numpy as np

def main(init_pos, goal_pos):
    client = simulation_commander.Client(ip='127.0.0.1', port=20011)
    packet = grSim_Packet()

    robot_replacement = grSim_RobotReplacement()
    robot_replacement.x = init_pos[0]
    robot_replacement.y = init_pos[1]
    robot_replacement.id = 0
    robot_replacement.dir = 0
    robot_replacement.yellowteam = 0

    packet.replacement.robots.append(robot_replacement)
    client.send(packet)

    control_path = time_optimal_steer_2d((init_pos[0], init_pos[1], 0.0, 0.0), (goal_pos[0], goal_pos[1], 0.0, 0.0), umin=(-1, -1), umax=(1, 1))

    time_unit = 0.01

    path_velocity_x = []
    path_velocity_y = []

    dt = 0
    control_time1 = control_time(control_path)
    print(control_path)
    while dt <= control_time1:
        sub_path = []
        total_time = 0
        for session in control_path:
                if session[1] + total_time < dt:
                    sub_path.append(session)
                    total_time += session[1]
                else:
                    sub_path.append([session[0], dt - total_time])
                    break
                
        temp_state = integrate_control_2d((init_pos[0], init_pos[1], 0.0, 0.0), sub_path)
        path_velocity_x.append(temp_state[2])
        path_velocity_y.append(temp_state[3])
        dt += time_unit
        
    for step in range(len(path_velocity_x)):
        start = time.time()
        
        packet = grSim_Packet()
        
        packet.commands.isteamyellow = False
        packet.commands.timestamp = 0

        wheel_speed = kinematics.reverse_kinematics([path_velocity_x[step], path_velocity_y[step], 0])
        robot_command = grSim_Robot_Command()
        robot_command.id = 0
        robot_command.wheelsspeed = 1
        robot_command.velnormal = 0
        robot_command.kickspeedx = 0
        robot_command.kickspeedz = 0
        robot_command.veltangent = 0
        robot_command.velangular = 0
        robot_command.spinner = 0
        robot_command.wheel1 = wheel_speed[0]
        robot_command.wheel2 = wheel_speed[1]
        robot_command.wheel3 = wheel_speed[2]
        robot_command.wheel4 = wheel_speed[3]
        
        # robot_command = grSim_Robot_Command()
        # robot_command.id = 0
        # robot_command.wheelsspeed = 0
        # robot_command.velnormal = path_velocity_y[step]
        # robot_command.kickspeedx = 0
        # robot_command.kickspeedz = 0
        # robot_command.veltangent = path_velocity_x[step]
        # robot_command.velangular = 0
        # robot_command.spinner = 0

        packet.commands.robot_commands.append(robot_command)
        client.send(packet)    

        # finish = time.time() - start
        # time.sleep(time_unit - finish)
        time.sleep(time_unit)

if __name__ == "__main__":
    init_x = -4500 #float(input("Inital position X (mm)"))
    init_y = -3000 #float(input("Inital position Y (mm)"))
    goal_x = 0 #float(input("Goal position X  (mm)"))
    goal_y = 0 #float(input("Goal position Y  (mm)"))
    main((init_x / 1000, init_y / 1000), (goal_x / 1000, goal_y / 1000))