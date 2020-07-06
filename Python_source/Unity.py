# ======================================================
# Copyright (C) 2019 BME Automated Drive Lab
# This program and the accompanying materials
# are made available under the terms of the MIT license.
# ======================================================
# Author: Balazs Varga
# Date: 2019. 11. 10.
# ======================================================

import threading
import socket
import time

def StartUnity(Client, UnityQueue, UnityRunningFlag):

    #Start thread
    UnityClientErr = threading.Event()
    UnityThread = threading.Thread(target=SendMessage, args=(Client, UnityClientErr, UnityQueue, UnityRunningFlag))
    UnityThread.start()

    return UnityClientErr, UnityThread

def SendMessage(Client, UnityClientErr, UnityQueue, UnityRunningFlag):

    #Transmit message periodically to Unity
    while UnityRunningFlag[0]:
        #Try to get data from the queue
        if (UnityQueue.empty()):
            time.sleep(0.005)
        else:
            msg = UnityQueue.get()

            # Prefix each message with a 4-byte length (network byte order)
            try:
                #msg = str(struct.pack('>I', len(msg))) + msg
                Client.send(msg.encode())
            except socket.error as e: #if client connection is lost, display error
                print(e)
                #Set client state to error
                UnityClientErr.set()
                break


#Construct the message that will be sent to Unity
def ToUnity(Vehicles, UnityQueue):

    DataToUnity = "O1G"

    #Other vehicles in the simulation
    for veh in Vehicles:
        DataToUnity += veh.ID + ";" + "{0:.3f}".format(veh.PosX_Center) + ";" + "{0:.3f}".format(veh.PosY_Center) + ";" + "{0:.2f}".format(veh.Velocity) + ";"  + "{0:.2f}".format(veh.Heading) + ";" + str(int(veh.StBrakePedal)) + ";" + str(veh.SizeClass) + ";" + str(int(veh.action))+ ";" + veh.message + "@"

    #Add line break
    DataToUnity = DataToUnity + "&\n"

    with UnityQueue.mutex:
        UnityQueue.queue.clear()
    UnityQueue.put(DataToUnity)  # Enqueue the updated data.
