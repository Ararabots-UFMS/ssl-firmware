import socket
import grSim_Commands_pb2
import grSim_Packet_pb2
import time

class Client:
    
    def __init__(self, ip:str, port:int):
        """Client that connects and receives messages from ssl-vision"""
        
        self.ip = ip
        self.port = port
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 32)

    def send(self, message):
        """Encode and send message."""
        msg = message.SerializeToString()
        self.sock.sendto(msg, (self.ip, self.port))
        
def main():
    client = Client(ip='127.0.0.1', port=20011)

    packet = grSim_Packet_pb2.grSim_Packet()
    packet.commands.isteamyellow = False
    packet.commands.timestamp = 0
    robot_command = grSim_Commands_pb2.grSim_Robot_Command()
    robot_command.id = 0
    robot_command.wheelsspeed = 0
    robot_command.velnormal = 15
    robot_command.kickspeedx = 0
    robot_command.kickspeedz = 0
    robot_command.veltangent = 15
    robot_command.velangular = 15
    robot_command.spinner = 0
    packet.commands.robot_commands.append(robot_command)

    # robot_replacement = grSim_Replacement_pb2.grSim_RobotReplacement()
    # robot_replacement.x = 0
    # robot_replacement.y = 0
    # robot_replacement.dir = 0
    # robot_replacement.id = 0
    # robot_replacement.yellowteam = 0
    # packet.replacement.robots.append(robot_replacement)

    print(packet)
    while(True):
        client.send(packet)
        time.sleep(0.01)

if __name__ == "__main__":
    main()