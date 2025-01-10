import rclpy
from rclpy.executors import SingleThreadedExecutor
import sys
from agentcontrol_pkg.agentcontrol import AgentController
from agentcontrol_pkg.agentcontrol_sim import AgentController_Sim
from agentcontrol_pkg.LF_formation_ctl import LF_formation_ctl
from agentcontrol_pkg.Consensus_ctl import Consensus_ctl
from agentcontrol_pkg.relay import TFRelay

import argparse
import yaml
import numpy as np

def main():
    rclpy.init()
    ##
   
    
    executor = SingleThreadedExecutor()

    opt, A,formation_distances = get_opt()
    agents = opt.agents
    leaders = opt.leaders
    namespace = opt.namespace
    mode = opt.mode
   
    N= dict.fromkeys(agents)
    Fd = dict.fromkeys(agents)
    controllers = []

    for i in range(len(agents)):
        if i not in leaders:
            Ni = []
            Fdi = dict.fromkeys(agents)
            for j in range(len(A[i])):
                if A[i][j]  != 0 or i == j:
                    Ni.append(agents[j])
                    if opt.lformation:
                        Fdi[agents[j]] = .6*formation_distances[i][j]
            N[agents[i]] = Ni
            if opt.lformation:
                Fd[agents[i]] = Fdi
    
    tf_relays = []
    
    try:
        if mode == 3:
            for agent in agents:
                tf_relay = TFRelay(namespace = opt.namespace, agent = i)
                tf_relays.append(tf_relay)
            for relay in tf_relays:
                executor.add_node(relay)
                
        for agent in agents:
            if opt.consensus:
                controller = Consensus_ctl(agent,neighbors = N[agent], namespace = opt.namespace, mode = mode)
            elif opt.lformation:
            	if agent not in leaders:
                    controller = LF_formation_ctl(agent,neighbors = N[agent],namespace = opt.namespace,leaders = leaders,Fd = Fd[agent], mode = mode) 
            else:
                controller = AgentController(agent,neighbors = N[agent])
            controllers.append(controller)

        for controller in controllers:
            executor.add_node(controller)
        try:
            executor.spin()
        finally:
            executor.shutdown()

    except KeyboardInterrupt:
            executor.shutdown()

def get_opt():
    parser = argparse.ArgumentParser()
    parser.add_argument("-m","--mode" ,type = int, default =0, help ="Enter mode: 0 = optitrack, 1= sim, 2 = tf/odom")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("-cm","--consensus",action= "store_true",default = False, help = "Consensus Mode")
    group.add_argument("-lfm","--lformation",action = "store_true", default = True, help = " Leader/follower formation mode")
    parser.add_argument("-p", "--path",type = str, default = "src/agentcontrol_pkg/agent_setup/agent_setup (C).yaml",help = "/path/to/agent_setup.yaml")
    parser.add_argument("-a","--agents", nargs = '+',type = int)
    parser.add_argument("-l","--leaders", default = [],nargs = '+',type = int )
    parser.add_argument("-ns","--namespace",type = str, default = 'robot', help = "namespace to be used")
    opt = parser.parse_args()
    formation_distances = []
    num_agents = len(opt.agents)
    with open(opt.path, 'r') as f:
         data = yaml.safe_load(f)
    if opt.consensus:
        A = np.ones((num_agents,num_agents))
    else:
        A = np.array(data['adjacency_matrix'])
    if opt.lformation: 
        formation_distances = np.array(data['formation_distances'])
    
    
    return opt,A,formation_distances

if __name__ == '__main__':
    main()

 