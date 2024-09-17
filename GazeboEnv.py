import math
import os
import random
import subprocess
import time
from os import path
import numpy as np
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

TIME_DELTA = 0.1

class GazeboEnv(Node):
    """Superclass for all Gazebo environments."""

    def __init__(self, launchfile):
        super().__init__('gazebo_env')
        if launchfile.startswith("/"):
            fullpath = launchfile
        else:
            fullpath = os.path.join(os.path.dirname(__file__), launchfile)
        if not path.exists(fullpath):
            raise IOError("File " + fullpath + " does not exist")

        # Launch the Gazebo simulation
        self.launch_gazebo_simulation(fullpath)

        # Create clients for ROS2 services
        self.unpause_client = self.create_client(Empty, "/unpause_physics")
        self.pause_client = self.create_client(Empty, "/pause_physics")
        self.reset_client = self.create_client(Empty, "/reset_simulation")
        self.reset_world = self.create_client(Empty, "reset_world")

    def launch_gazebo_simulation(self, fullpath):
        self.gazebo_process = subprocess.Popen(["ros2", "launch", "neo_simulation2", "simulation.launch.py"])
        self.get_logger().info("Gazebo launched!")

    def step(self, action):
        """Perform an action and read a new state"""
        self.get_logger().info(f"Performing step with action: {action}")
        self.call_service_sync(self.unpause_client)
        time.sleep(TIME_DELTA)
        self.call_service_sync(self.pause_client)
        return 0  # Placeholder return

    def reset(self):
        """Reset the state of the environment and return an initial observation."""
        self.get_logger().info("Resetting the environment...")
        self.call_service_sync(self.reset_client)
        self.call_service_sync(self.unpause_client)
        time.sleep(TIME_DELTA)
        self.call_service_sync(self.pause_client)
        self.get_logger().info("Environment reset complete.")
        return 0  # Placeholder return

    def call_service_sync(self, client):
        """Helper method to call ROS2 services synchronously."""
        if not client.service_is_ready():
            self.get_logger().info(f"Waiting for service {client.srv_name} to become available...")
            client.wait_for_service()
        req = Empty.Request()
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error(f"Service call failed: {future.exception()}")

    def turn_off(self):
        """Turn off the simulation"""
        self.get_logger().info("Turning off the simulation...")
        self.call_service_sync(self.pause_client)
        self.shutdown()

    @staticmethod
    def get_reward(target, collision, action, min_laser):
        """Compute and return the reward"""
        reward = 0

        return reward

    def shutdown(self):
        """Shutdown the Gazebo process"""
        if hasattr(self, 'gazebo_process'):
            self.gazebo_process.terminate()
            self.gazebo_process.wait()
            self.get_logger().info("Gazebo process terminated.")