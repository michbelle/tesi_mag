#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import numpy as np
import csv

from pathlib import Path

import math

from ament_index_python.packages import get_package_share_directory
import os


import tf2_ros
from tf2_ros import TransformException
from rclpy.time import Time
from tf_transformations import euler_from_quaternion


class OdometryRecorder(Node):
    def __init__(self):
        super().__init__("odometry_recorder")
        
        self.declare_parameter("odom_topic", "/odometry/wheels")
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        
        self.declare_parameter("file_name", "data")
        self.file_name = self.get_parameter("file_name").get_parameter_value().string_value
        
        self.declare_parameter("names_file_to_plot", ["odom_result_000"])
        self.names_file_to_plot = self.get_parameter("names_file_to_plot").get_parameter_value().string_array_value
        
        self.declare_parameter("enable_recording_odom", True)
        self.enable_recording_odom = self.get_parameter("enable_recording_odom").get_parameter_value().bool_value

        self.declare_parameter("enable_recording_tf", True)
        self.enable_recording_tf = self.get_parameter("enable_recording_tf").get_parameter_value().bool_value
        
        self.declare_parameter("enable_plotting", False)
        self.enable_plotting = self.get_parameter("enable_plotting").get_parameter_value().bool_value
        
        self.subscription_odom = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.listener_callback_odom,
            10)
        self.odometry_data = []


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.25, self.listener_callback_tf)

        # self.subscription_tf = self.create_subscription(
        #     Odometry,
        #     self.odom_topic,
        #     self.listener_callback_tf,
        #     10)
        
        self.tf_data = []

        self.filepath=os.path.join("/root","openRMF_ws","src","tesi_code", "result")

    def listener_callback_odom(self, msg):
        # Record odometry data
        position = msg.pose.pose.position
        heading = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.odometry_data.append((msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9, position.x, position.y, heading))

    def listener_callback_tf(self):
        now = Time()

        # Lookup transform: odom -> base_link
        try:
            tf_odom_base = self.tf_buffer.lookup_transform(
                'odom', 'base_link', now
            )
            pos = tf_odom_base.transform.translation
            rot = tf_odom_base.transform.rotation
            quat = [rot.x, rot.y, rot.z, rot.w]
            roll, pitch, yaw = euler_from_quaternion(quat)
            time = tf_odom_base.header.stamp.sec + tf_odom_base.header.stamp.nanosec * 1e-9

            data_odom_base_link=[time, pos.x , pos.y, yaw]
            
        except TransformException as ex:
            self.get_logger().warn(f'Could not get odom->base_link: {ex}')

        # Lookup transform: map -> odom
        try:
            tf_map_odom = self.tf_buffer.lookup_transform(
                'map', 'odom', now
            )
            pos = tf_map_odom.transform.translation
            rot = tf_map_odom.transform.rotation
            quat = [rot.x, rot.y, rot.z, rot.w]
            roll, pitch, yaw = euler_from_quaternion(quat)
            data_map_odom=[pos.x , pos.y, yaw]
        except TransformException as ex:
            self.get_logger().warn(f'Could not get map->odom: {ex}')
        data_odom_base_link.extend(data_map_odom)
        self.tf_data.append(data_odom_base_link)

    def quaternion_to_yaw(self, orientation):
        # Convert quaternion to yaw (heading)
        # Roll, Pitch, Yaw
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def save_data(self):
        # Save data to CSV
        i=0
        limit=100
        while (i<(limit+2)):
            if i==(limit+1):
                raise NotImplementedError()
            my_file = Path(f"{self.filepath}",f"{self.file_name}_odom_{i:03.0f}.csv")
            if my_file.is_file():
                i+=1
            else:
                with open(my_file, mode="w", newline="") as file:
                    writer = csv.writer(file)
                    writer.writerow(["Time (s)", "X (m)", "Y (m)", "heading"])
                    writer.writerows(self.odometry_data)
                    break
        i=0
        limit=100
        while (i<(limit+2)):
            if i==(limit+1):
                raise NotImplementedError()
            my_file = Path(f"{self.filepath}/{self.file_name}_tf_{i:03.0f}.csv")
            if my_file.is_file():
                i+=1
            else:
                with open(my_file, mode="w", newline="") as file:
                    writer = csv.writer(file)
                    writer.writerow(["Time (s)", "X (m) odom_base", "Y (m) odom_base", "heading odom_base", "X (m) map_odom", "Y (m) map_odom", "heading map_odom"])
                    writer.writerows(self.tf_data)
                    break

    def plot_data(self):
        # Plot the odometry data
        odom_data=[]
        if self.enable_recording_odom:
            # times, x_positions, y_positions, heading = zip(*self.odometry_data)
            # plt.plot(x_positions, y_positions, marker="o")
            odom_data.append(self.odometry_data)
        for filedata in self.names_file_to_plot:
            if filedata == self.file_name:
                continue
            else:
                if not Path(f"{self.filepath}{filedata}.csv").is_file():
                    raise NameError(f"no file {self.filepath}{filedata}.csv")
                odometry_data=[]
                n=0
                with open(self.filepath+filedata+".csv", mode="r", newline="") as file:
                    data = csv.reader(file)
                    for row_data in data:
                        if n==0:
                            n+=1
                        else:
                            odometry_data.append((row_data[0],row_data[1],row_data[2],row_data[3]))
                    odom_data.append(odometry_data)
        plt.figure()
        # odom_data=[[(0,1,1,0),(0,2,2,0)]]
        # odom_data.append([(0,1,2,0),(0,2,1,0)])
        for data in odom_data:
            times, x_positions, y_positions, heading = zip(*data)
            plt.plot(x_positions, y_positions, marker="o")
            break
        # plt.savefig("odometry_plot.png")
        plt.title("Odometry Data")
        plt.xlabel("X Position (m)")
        plt.ylabel("Y Position (m)")
        plt.grid()
        plt.axis("equal")
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    odometry_recorder = OdometryRecorder()

    try:
        if odometry_recorder.enable_recording_odom:
            rclpy.spin(odometry_recorder)
    except KeyboardInterrupt:
        pass
    if odometry_recorder.enable_recording_odom:
        odometry_recorder.save_data()
    if odometry_recorder.enable_plotting:
        odometry_recorder.plot_data()
    odometry_recorder.destroy_node()
        # rclpy.shutdown()

if __name__ == "__main__":
    main()
