#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped


from rclpy.action import ActionClient

from nav2_msgs.action import ComputePathToPose

import time

from datetime import timedelta
class calculatePath(Node):

    def __init__(self):
        super().__init__('calc_path_ActClient')


        # self.declare_parameter("starts_goals", starts_goals_positions)
        # self.starts_goals = self.get_parameter("starts_goals").get_parameter_value().double_array_value

        self._action_client = ActionClient(self, ComputePathToPose, '/compute_path_to_pose')

    
    def calculate_path_to(self, planner, start, goal):

        self.planner=planner
        self.start=start
        self.goal=goal

        goal_msg = ComputePathToPose.Goal()

        goal_msg.planner_id = planner
        goal_msg.use_start = True

        goal_msg.start.header.frame_id = 'map'
        goal_msg.start.pose.position.x = start[0]
        goal_msg.start.pose.position.y = start[1]
        goal_msg.start.pose.position.z = 0.0
        goal_msg.start.pose.orientation.x = 0.0
        goal_msg.start.pose.orientation.y = 0.0
        goal_msg.start.pose.orientation.z = 0.0
        goal_msg.start.pose.orientation.w = 1.0



        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.pose.position.x = goal[0]
        goal_msg.goal.pose.position.y = goal[1]
        goal_msg.goal.pose.position.z = 0.0
        goal_msg.goal.pose.orientation.x = 0.0
        goal_msg.goal.pose.orientation.y = 0.0
        goal_msg.goal.pose.orientation.z = 0.0
        goal_msg.goal.pose.orientation.w = 1.0

        self._action_client.wait_for_server()
        self.accepted=False

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        return self._send_goal_future

    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

        self.accepted=True

    def get_result_callback(self, future):
        result = future.result().result
        if hasattr(result, 'error_code'):
            if result.error_code != 0:
                self.get_logger().error('Result: {0}'.format(result))
        else:
            try:
                if not hasattr(result, 'error_code') and result.planning_time.sec== 0 and result.planning_time.nanosec== 0:
                    self.get_logger().error('failed to generate path')
                    self.time_calculation = timedelta(seconds=10).seconds
                else:
                    self.get_logger().info(f'Obtained path in {result.planning_time.sec+result.planning_time.nanosec*1e-9 }')
                    self.time_calculation=result.planning_time.nanosec * 1e-9 + result.planning_time.sec
            except Exception as e:
                print(e)

        self.get_logger().debug('---------------------------')
        self.get_logger().debug('Result: {0}'.format(result))
        self.get_logger().debug('---------------------------')
        rclpy.shutdown()

def main(args=None):    
    import statistics

    position_elettra={
        "ufficio" : [6.00483, 23.9497],
        "ripostiglio" : [-4.87824, 12.0739],
        "ingresso" : [-13.1734, -37.918],
        "disegnatori" : [32.9481, 70.8242],
    }
    starts_goals_positions=[
        [
            position_elettra['ufficio'],
            position_elettra['ripostiglio'],
        ],
        [
            position_elettra['ufficio'],
            position_elettra['ripostiglio'],
        ],
        [
            position_elettra['ufficio'],
            position_elettra['ingresso'],
        ],
        [
            position_elettra['ufficio'],
            position_elettra['ingresso'],
        ],
        [
            position_elettra['ufficio'],
            position_elettra['ingresso'],
        ],
        # [
        #     position_elettra['ingresso'],
        #     position_elettra['ripostiglio'],
        # ],
        [
            position_elettra['ingresso'],
            position_elettra['disegnatori'],
        ],
        [
            position_elettra['ingresso'],
            position_elettra['disegnatori'],
        ],
    ]
    for planner in ["GridBased_Dstar"]:
    # for planner in ["GridBased_Dstar", "GridBased"]:
    # for planner in ["GridBased"]:
        for start, goal in starts_goals_positions:
            time_calculation=[]
            for i in range(10):
                rclpy.init(args=args)
                calculatePath_ActClient = calculatePath()
                calculatePath_ActClient.calculate_path_to(planner,start, goal)
                rclpy.spin(calculatePath_ActClient)
                time_calculation.append(calculatePath_ActClient.time_calculation)
            print(statistics.mean(time_calculation))

if __name__ == '__main__':
    main()
