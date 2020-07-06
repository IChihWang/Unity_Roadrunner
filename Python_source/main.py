from __future__ import absolute_import
from __future__ import print_function


import os
import sys

sys.path.append('/usr/share/sumo/tools/')
# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import optparse
import random
import numpy
import threading
import time
import numpy as np

from ortools.linear_solver import pywraplp
from sumolib import checkBinary
import traci
import traceback

import config as cfg

from gen_route import generate_routefile

import time
from queue import Queue
import TrafficSimulator
import TCP_server
import Unity
import SUMO_vehicle



# For debug
#from playsound import playsound

from IntersectionManager import IntersectionManager
#from myGraphic import Gui
#import myGraphic


#myGraphic.gui = Gui()

###################


class SumoUnity(object):
    def __init__(self, IP, Port):
        # Define queues for communication
        self.UnityQueue = Queue(maxsize=1)
        #Launch SUMO
        self.TrafficSim = TrafficSimulator.TrafficSimulator()
        self.SumoObjects = []

        #Start TCP server
        self.ServerIP = IP
        self.ServerPort = Port
        self.Server = TCP_server.TCP_Server(self.ServerIP, self.ServerPort)
        self.Server.StartServer(self.UnityQueue)

    def run(self):
        """execute the TraCI control loop"""
        simu_step = 0

        intersection_manager = IntersectionManager()


        try:
            highlighted_car_id = None
            while traci.simulation.getMinExpectedNumber() > 0:
                #Get timestamp
                TiStamp1 = time.time()

                if (simu_step*10)//1/10.0 == 200:
                    break

                traci.simulationStep()
                all_c = traci.vehicle.getIDList()



                # Update the position of each car
                for car_id in all_c:
                    lane_id = traci.vehicle.getLaneID(car_id)
                    intersection_manager.update_car(car_id, lane_id, simu_step)

                intersection_manager.run(simu_step)
                simu_step += cfg.TIME_STEP


                #Monitor TCP connection
                self.Server.ReopenSocket(self.UnityQueue)

                # Print it to Unity
                SumoObjectNames = traci.vehicle.getIDList()  # Make it unique

                # Remove SUMO objects from the list if they left the network
                for Obj in self.SumoObjects:
                    if (not(Obj.ID in SumoObjectNames)):
                        self.SumoObjects.remove(Obj)
                        if Obj.ID == highlighted_car_id:
                            highlighted_car_id = None


                #Append new objects and update existing ones.
                for VehID in SumoObjectNames:
                    if(not(any(Obj.ID == VehID for Obj in self.SumoObjects))):
                        NewlyArrived = SUMO_vehicle.SumoObject(VehID)
                        self.SumoObjects.append(NewlyArrived)

                #Update Sumo vehicle objects
                for Obj in self.SumoObjects:
                    Obj.UpdateVehicle()
                    action, message = intersection_manager.get_action_message_unity(Obj.ID)
                    Obj.action = action
                    Obj.message = message


                # Find new highlighted_car_id
                if highlighted_car_id == None and len(self.SumoObjects) > 0:
                    highlighted_car_id = self.SumoObjects[-1].ID
                    traci.vehicle.setColor(highlighted_car_id, (255,233,96))

                #Update Unity
                Unity.ToUnity(self.SumoObjects, self.UnityQueue)

                #Synchronize time
                deltaT = cfg.TIME_STEP
                TiStamp2 = time.time() - TiStamp1
                if TiStamp2 > deltaT:
                    pass
                else:
                    time.sleep(deltaT-TiStamp2)

        except Exception as e:
            traceback.print_exc()


        #debug_t = threading.Thread(target=debug_ring)
        #debug_t.start()
        self.Server.CloseSocket()
        sys.stdout.flush()

        traci.close()





##########################
# Setup running options for sumo
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


###########################
# Main function
if __name__ == "__main__":
    print("Usage: python code.py")

    random_seed = 10
    arrival_rate_in = 0.3
    IP = 'localhost'
    port = 4042

    seed = random_seed
    random.seed(seed)  # make tests reproducible
    numpy.random.seed(seed)

    options = get_options()




    # 0. Generate the intersection information files
    #os.system("bash gen_intersection/gen_data.sh " + str(cfg.LANE_NUM_PER_DIRECTION))

    # 1. Generate the route file for this simulation
    arrival_rate = float(arrival_rate_in)
    generate_routefile(arrival_rate)


    sys.path.append('/usr/share/sumo/tools/')
    if 'SUMO_HOME' in os.environ:
        tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
        sys.path.append(tools)
    else:
        sys.exit("please declare environment variable 'SUMO_HOME'")
    sumoBinary = checkBinary('sumo-gui')

    try:
        # 3. This is the normal way of using traci. sumo is started as a subprocess and then the python script connects and runs
        traci.start([sumoBinary, "-c", "data/Roadrunner_net.sumocfg",
                                 "--step-length", str(cfg.TIME_STEP),
                                 "--collision.mingap-factor", "0",
                                 "--start"])

        # 4. Start running SUMO
        #INIT
        Simulation = SumoUnity(IP, port)
        #MAIN
        Simulation.run()
    except Exception as e:
        traceback.print_exc()
