//Raspberry PI ( 주차공간 업데이트 + 적절한 주차공간으로의 경로 탐색 + 주차공간으로의 Drone의 안내 )
import numpy as np
import Queue
import threading
import time
from bluetooth import *

# drone --------------------------------------------------
import serial
import RPi.GPIO as GPIO

ser = serial.Serial('/dev/ttyS0', 9600, timeout = 0.001)
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)
startBit = 0xf0
commandBit = 0xa1
roll = 100
pitch = 103
yaw = 100
throttle = 0
operationBit = 0x01
checkSum = 0

def checkThrottle(): # down < 100 < up
    global throttle
    throttle = 80

def checkPitch(): # ( black, black ) back < 100 < forward ( white, white )
    global pitch

def checkRoll(): # left < 100 < right
    global roll

def checkYaw(): # left turn < 100 < right turn
    global yaw

def checkCRC():
    global commandBit
    global roll
    global pitch
    global yaw
    global throttle
    global operationBit
    global checkSum

    checkSum = commandBit + roll + pitch + yaw + throttle + operationBit
    checkSum = checkSum & 0x00ff


switch_list = [22, 27, 17, 23, 24, 25]

print("\nRaspberrypi Data Packet 02\n")

time.sleep(0.5)
ser.write("atd")
ser.write("083a5c1f0aee")
ser.write("\r")
time.sleep(0.5)

# drone --------------------------------------------------


map = np.loadtxt("Map.txt", delimiter=',')
rows = map.shape[0]
cols = map.shape[1]
path = np.zeros((rows, cols))
q = Queue.Queue()
checkpt, dpmt, start, vnode, nevi, turns = [], [], [], [], [], []
dx = [-1, 0, 1, 0]
dy = [0, -1, 0, 1]
dir_idx = ['E', 'N', 'W', 'S']
Ldir_idx = ['N', 'W', 'S', 'E']
Rdir_idx = ['S', 'E', 'N', 'W']
Exit = False

def promise(a,b):
    if 1 <= a and a < rows-1 and 1 <= b and b < cols-1:
        if path[a][b] == 0 and map[a][b] == 0:
            return True
    return False

def global_variables_init():
    for i in range(rows):
        for j in range(cols):
            if map[i][j] == 5:
                start.append((i,j))
            if map[i][j] > 0 and map[i][j]%10 == 0:
                dpmt.append(map[i][j])
                dpmt.append((i,j))
    q.put(start[0])
    while not q.empty():
        a, b = q.get()
        if map[a+1][b] == 0 and map[a][b+1] == 0:
            checkpt.append((a,b))
        elif map[a][b+1] == 0 and map[a-1][b] == 0:
            checkpt.append((a,b))
        elif map[a-1][b] == 0 and map[a][b-1] == 0:
            checkpt.append((a,b))
        elif map[a][b-1] == 0 and map[a+1][b] == 0:
            checkpt.append((a,b))
        for i in range(4):
            if promise(a + dx[i], b + dy[i]):
                q.put((a + dx[i], b + dy[i]))
                path[a + dx[i]][b + dy[i]] = path[a][b] + 1

def getSpot(a,b):
    spots = []
    absmin = rows**2 + cols**2
    for i in range(rows):
        for j in range(cols):
            if map[i][j] == 1:
                spots.append((i,j))
    if spots:
        for (i,j) in spots:
            if absmin > (a-i)**2 + (b-j)**2:
                absmin = (a-i)**2 + (b-j)**2
                dest_i, dest_j = i, j
        for i in range(4):
            if map[dest_i + dx[i]][dest_j + dy[i]] == 0:
                return dest_i, dest_j, int(dest_i + dx[i]), int(dest_j + dy[i])
    return 0, 0, 0, 0

def findPath(cur_a, cur_b, a, b):
    if cur_a == a and cur_b == b:
        vnode.append((a,b))
        nevi.append(vnode[:])
        del vnode[len(vnode)-1]
        return
    vnode.append((cur_a, cur_b))
    for i in range(4):
        if path[cur_a + dx[i]][cur_b + dy[i]] > path[cur_a][cur_b]:
            findPath(cur_a + dx[i], cur_b + dy[i], a, b)
    del vnode[len(vnode)-1]

def getDirection(dest_i, dest_j):
    minpath, mincost = [], rows * cols
    answer, direction, drive = [], [] ,[]
    for walk in nevi:
        if len(walk) < mincost:
            mincost = len(walk)
            minpath = walk
    for node in minpath:
        if node in checkpt or node==minpath[0] or node==minpath[-1]:
            answer.append(node)
    print(answer)
    for i in range(1, len(answer)):
        x = answer[i][0] - answer[i-1][0]
        y = answer[i][1] - answer[i-1][1]
        if x==0 and y > 0:
            direction.append(('E', answer[i]))
        elif x < 0 and y==0:
            direction.append(('N', answer[i]))
        elif x==0 and y < 0:
            direction.append(('W', answer[i]))
        elif x > 0 and y==0:
            direction.append(('S', answer[i]))
    print(direction)
    for i in range(1, len(direction)):
            cur_dir = dir_idx.index(direction[i-1][0])
            if Ldir_idx[cur_dir]==direction[i][0]:
                turns.append(('L', direction[i-1][1]))
            if Rdir_idx[cur_dir]==direction[i][0]:
                turns.append(('R', direction[i-1][1]))
    print(turns)

def start_nevigation():
    user_input = int(input("please choose a department(put - 1 to exit) : "))
    if user_input == -1:
        return True
    if user_input in dpmt:
        dest_i, dest_j, path_i, path_j = getSpot(dpmt[dpmt.index(user_input)+1][0], dpmt[dpmt.index(user_input)+1][1])
    if dest_i == 0 or dest_j == 0:
        print("Parking Station is Currently Full")
    else:
        map[dest_i][dest_j] = -1
        print(map)
        findPath(start[0][0], start[0][1], path_i, path_j)
        getDirection(dest_i, dest_j)

        # drone ----------------------------------------
        dronetime = 0
        while dronetime < 20:
            checkThrottle()
            checkPitch()
            checkCRC()

            ser.write("at+writeh0006")
            ser.write(hex(startBit)[2:4])
            ser.write(hex(commandBit)[2:4])


            ser.write(hex(roll)[2:4])
            ser.write(hex(pitch)[2:4])
            ser.write(hex(yaw)[2:4])

            if throttle < 0x10:
                ser.write('0' + hex(throttle)[2:4])
            else:
                ser.write(hex(throttle)[2:4])

            ser.write("01")

            if checkSum < 0x10:
                ser.write('0' + hex(checkSum)[2:4])
            else:
                ser.write(hex(checkSum)[2:4])

            ser.write("\r")
            dronetime = dronetime + 1
            time.sleep(0.1)
    # drone --------------------------------------------------
    

    while len(nevi) > 0: nevi.pop()
    while len(turns) > 0: turns.pop()
    return False


def sensor_update():
    client_socket = BluetoothSocket(RFCOMM)
    client_socket.connect(("98:D3:61:FD:5D:B5", 1))

    while not Exit:
        msg1 = int(client_socket.recv(1024))
        time.sleep(1)
        msg2 = int(client_socket.recv(1024))
        map[msg1][msg2] = 1
    
    print("bluetooth End!")
    client_socket.close()

if __name__ == "__main__":
    global_variables_init()
    print("Initial map")
    print(map)
    time.sleep(1)
    t = threading.Thread(target=sensor_update)
    t.start()
    while not Exit:
        Exit = start_nevigation()
