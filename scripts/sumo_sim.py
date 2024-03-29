#!/usr/bin/env python

import os
import sys
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:   
    sys.exit("please declare environment variable 'SUMO_HOME'")
    
import traci

import rospy, rospkg
from mrpp_algos.msg import NextTask, TaskDone


class TraciSim:

    def __init__(self, num_vehicles = 4):
        HIGH_NUM = 5000
        self.num_vehicles = num_vehicles
        self.routes = {}
        self.updated = []
        self.update = False
        self.last_nodes = {}
        self.stopped_already = []
        for i in range(num_vehicles):
            rospy.Subscriber('/robot_{}/next_task'.format(i), NextTask, self.callback, i)
            self.routes[i] = []
            self.updated.append(False)
            self.last_nodes[i] = HIGH_NUM
            self.stopped_already.append(False)
        self.pub = rospy.Publisher('task_done', TaskDone, queue_size = 10)

    def callback(self, data, robot_id):
        route = []
        # print robot_id, data.task
        for i in range(len(data.task) - 1):
            route.append(str(data.task[i]) + "to" + str(data.task[i + 1]))
        if len(self.routes[robot_id]) != 0:
            route.insert(0, self.routes[robot_id][-1])
        self.routes[robot_id] = route
        self.updated[robot_id] = True 
        self.update = True

def main(argv):
    rospy.init_node('sumo_sim', anonymous = True)
    graph_name = argv[0]
    num_vehicles = int(argv[1])
    t = TraciSim(num_vehicles)
    dirname = rospkg.RosPack().get_path('sumo_ws')
    sumo_startup = ['sumo-gui', '-c', dirname +'/graph_sumo/{}.sumocfg'.format(graph_name)]
    print 'Click Play button only on ensuring that the algorithm is fully functional'
    traci.start(sumo_startup)
    #Click Play Button in the GUI, if GUI
    
    while True:
        vehicles = traci.vehicle.getIDList()
        # print vehicles
        for i in range(num_vehicles):
            if t.updated[i]:
                # print 'here'
                if str(i) not in vehicles:
                    traci.route.add(routeID = str(i + num_vehicles), edges = t.routes[i])
                    traci.vehicle.add(vehID = str(i),routeID = str(i + num_vehicles), typeID = "type1")
                else:
                    e = traci.vehicle.getRoute(str(i))
                    if traci.vehicle.isStopped(str(i)) and e[-1] == t.routes[i][0]:
                        traci.vehicle.resume(str(i))
                        traci.vehicle.setRoute(vehID = str(i), edgeList = t.routes[i])
                        t.stopped_already[i] = False
                
            t.updated[i] = False
        
        msg = TaskDone()
        msg.stamp = traci.simulation.getTime()
        msg.node_id = []
        msg.robot_id = []
        for i in range(num_vehicles):
            vehicles = traci.vehicle.getIDList()
            if str(i) in vehicles:
                last_edge = traci.vehicle.getRoadID(str(i))
                # print last_edge
                if last_edge.find(':') == -1:
                    last_node = int(last_edge[:last_edge.find('to')])
                    # if traci.vehicle.isStopped(str(i)) and not t.stopped_already[i]:
                    if traci.vehicle.isStopped(str(i)):
                        stop_node = int(last_edge[last_edge.find('to') + 2:])
                        msg.node_id.append(stop_node)
                        msg.robot_id.append(i)
                        # t.last_nodes[i] = stop_node
                        t.stopped_already[i] = True

                    elif not traci.vehicle.isStopped(str(i)) and t.last_nodes[i] != last_node:
                        total_len = traci.lane.getLength(last_edge + '_0')
                        cur_len = traci.vehicle.getLanePosition(str(i))
                        if cur_len < total_len/2:
                            stop_pos = traci.lane.getLength(t.routes[i][-1] + '_0')
                            traci.vehicle.setStop(vehID = str(i), edgeID = t.routes[i][-1], pos = stop_pos, duration = 2000.)
                            msg.node_id.append(last_node)
                            msg.robot_id.append(i)
                            t.last_nodes[i] = last_node
            
        if len(msg.node_id) > 0:
            t.pub.publish(msg)

        # print 'Num_vehicles', traci.vehicle.getIDCount()
        traci.simulationStep()

if __name__ == '__main__':
    if len(sys.argv[1:]) == 2:
        main(sys.argv[1:])
    else:
        print 'Please pass the appropriate arguments'
