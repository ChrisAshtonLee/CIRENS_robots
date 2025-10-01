#!/usr/bin/env python3

import numpy as np
import rclpy
from agent_control.agent import Agent
import argparse
import datetime
from TukeyMedian import TukeyContour
import pdb

class CPIH(Agent):
    def __init__(self, my_number, my_neighbors=[], *args, sim=False, sync_move=False,
        destination_tolerance=0.01,
        restricted_area = False, restricted_x_min = -2.9, restricted_x_max = 2.9, restricted_y_min = -5, restricted_y_max = 4,
        laser_avoid=True, laser_distance=0.5, laser_delay=5, laser_walk_around=2, laser_avoid_loop_max=1,
        neighbor_avoid=True, neighbor_delay=5):
        super().__init__(my_number, my_neighbors, sim=sim, sync_move=sync_move, 
                        destination_tolerance=destination_tolerance,
                        restricted_area=restricted_area, restricted_x_min=restricted_x_min, restricted_x_max=restricted_x_max, restricted_y_min=restricted_y_min, restricted_y_max=restricted_y_max,
                        laser_avoid=laser_avoid, laser_distance=laser_distance, laser_delay=laser_delay, laser_walk_around=laser_walk_around, laser_avoid_loop_max=laser_avoid_loop_max,
                        neighbor_avoid=neighbor_avoid, neighbor_delay=neighbor_delay)

    def controller(self):
        '''
        This function is called every time the robot position is updated. We will put our concensus logic here.

        Equation:
        new_position = sum of each neighbor [ weight * (ni_pos - my_pos) * (ni_pos - my_pos - 0.5)]

        Needed info from agent.
        self.position                   This agents position
        self.neighbor_position          Dictionary of neighbors position
        self.move_direction([x,y])      Function to move in a direction
        self.move_to_position([x,y])    Function to move to a position
        '''
        X = np.zeros((6,2))
        i = 0
        for neighbor in self.neighbor_poses:
            X[i] = np.array((neighbor.pose.position.x, neighbor.pose.position.y))
        tc = TukeyContour(X)
        if tc.median_contour.shape[0] > 0:
            # Target is the centroid of the median contour
            safepoint = np.mean(tc.median_contour, axis=0)
        else:
            safepoint = np.array((self.pose.position.x,self.pose.position.y))
        target = safepoint

        self.move_to_position(target)


def main(args=None):
    ## Start Simulation Script
    ## ros2 launch turtlebot_base launch_sim.launch.py 
    ## ros2 launch turtlebot_base launch_robots.launch.py yaml_load:=False robot_number:=4
    ## ros2 launch agent_control consensus_batch.launch.py yaml_load:=False number_robots=4
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--index", default="1", type=int, help="Index of this robot")
    parser.add_argument("-n", "--neighbor", default=[], nargs='+', type=int, help="Array of neighbors")
    parser.add_argument("-s", "--sim", default=False, action="store_true", help="Set Simmulation mode")
    parser.add_argument("-l", "--laser_avoid", default=True, action="store_false", help="Avoid using laser")
    parser.add_argument("-m", "--loop_max", default=1, type=int, help="Laser Loop Max Number")
    parser.add_argument("-b", "--neighbor_avoid", default=True, action="store_false", help="Avoid Using neighbor position")
    parser.add_argument("--ros-args", default=False, action="store_true")
    script_args = parser.parse_args()

    try:
        rclpy.init(args=args)
        my_robot = CPIH(int(script_args.index), np.array(script_args.neighbor), sim=script_args.sim, 
            restricted_area=True, laser_avoid=script_args.laser_avoid, neighbor_avoid=script_args.neighbor_avoid, laser_avoid_loop_max=script_args.loop_max)
        rclpy.spin(my_robot)
    except Exception as e:
        traceback.print_exc()
    finally:
        my_robot.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

'''
To Do:
Do sync move? - in agent file
Make a concenses example
'''