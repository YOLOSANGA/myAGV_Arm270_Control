from pymycobot.mycobot import MyCobot
import serial.tools.list_ports
import time
import socket
import json
import re
import sys
import random

def get_port(): # \ubaa8\ub4e0 \uc2dc\ub9ac\uc5bc \ud3ec\ud2b8 \ubc88\ud638 \uac00\uc838\uc624\uae30
    port_list = serial.tools.list_ports.comports()
    i = 1
    res = {}
    for port in port_list:
        print("{} - {}".format(i, port.device))
        res[str(i)] = port.device
        i += 1
    return res

def main():
    ml = MyCobot('/dev/ttyACM0')
    
    angles = [0,0,0,0,0,0] 
    
    while True:
    
    	for i in range(0,6):
    		angles[i] = random.randint(-45, 45)
    	
    	ml.send_angles(angles, 80)
    	time.sleep(3)
    	
main()