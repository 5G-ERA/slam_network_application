import os
import signal
import subprocess
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from cartographer_ros_msgs.srv import FinishTrajectory, WriteState
import argparse
from rclpy.parameter import Parameter  # Import the missing Parameter class
from era_5g_interfaces.srv import TriggerMap

class MapCreatorNode(Node):
    def __init__(self, use_sim_time: bool, output_folder: str, launch_file_mapping: str, launch_file_localization: str, launch_file_octomap_server: str):
        

        super().__init__('map_creator_node')

        self.cartographer_process = None  # To store the subprocess instance
        self.bag_recorder_process = None
        self.octomap_server_process = None
        self.map_saver_process = None

        use_sim_time_parameter = Parameter("use_sim_time", Parameter.Type.BOOL, use_sim_time)
        self.set_parameters([use_sim_time_parameter])
        
        
        # Create a service to finish mapping and stop bag recording
        self.finish_mapping_service = self.create_service(Trigger, 'finish_mapping', self.finish_map_callback)
        
        # Create a service to start mapping
        self.start_mapping_service = self.create_service(TriggerMap, 'start_mapping', self.start_mapping_callback)
        
        # Create a service to cancel mapping
        self.cancel_mapping_service = self.create_service(Trigger, 'cancel_mapping', self.cancel_mapping_callback)
        
        self.start_localization_service = self.create_service(TriggerMap, 'start_localization', self.start_localization_callback)
        
        self.stop_localization_service = self.create_service(Trigger, 'stop_localization', self.stop_localization_callback)

        # Create a client to call the "write_state" service of Cartographer
        self.write_state_client = self.create_client(WriteState, '/write_state')

        # Create a client to call the "finish_trajectory" service of Cartographer
        self.finish_trajectory_client = self.create_client(FinishTrajectory, '/finish_trajectory')
 
        self.output_folder = output_folder
        self.cartographer_executable_mapping = launch_file_mapping
        self.cartographer_executable_localization = launch_file_localization
        self.octomap_executable = launch_file_octomap_server
        self.finish_map_trigger = False
        self.cancel_map_trigger = False
        
        
        
    def start_mapping_callback(self, request, response):
        if self.cartographer_process is None:
            self.map_name = request.map_name
            self.bag_recorder_process = self.start_subprocess('ros2 bag record -o ' + os.path.join(self.output_folder, self.map_name) + ' /robot/odometry/filtered /robot/imu/data /robot/top_laser/point_cloud /tf /tf_static /clock')
            self.cartographer_process = self.start_subprocess('ros2 launch era_5g_cartographer ' + self.cartographer_executable_mapping)
            response.success = True
            response.message = 'Mapping started.'
        else:
            response.success = False
            response.message = 'Mapping already in progress.'
        return response
    
    def cancel_mapping_callback(self, request, response):
        if self.cartographer_process is not None:
            # Stop bag recording
            
            self.get_logger().info('Cancelling mapping...')
            self.cancel_map_trigger = True

            response.success = True
            response.message = 'Finishing map.'
        else:
            response.success = False
            response.message = 'No mapping in progress.'
        return response
        
    
    
    def start_octomap_server(self):
        if self.octomap_server_process is None:
            self.octomap_server_process = subprocess.Popen(['ros2', 'launch', "era_5g_cartographer", self.octomap_executable])
            print('Asset writer started.')
            
    def start_subprocess(self, command: str, env = None):
        if env:
            return subprocess.Popen(command.split(), env=env)
        else:
            return subprocess.Popen(command.split())
    
    def start_localization(self, env = None):
        localization_string = 'ros2 launch era_5g_cartographer ' + self.cartographer_executable_localization + ' map_file:=' + os.path.join(self.output_folder, self.map_name + '.pbstream')
        print(localization_string)
        return self.start_subprocess(localization_string, env)
            
        

    def finish_map_callback(self, request, response):
        if self.cartographer_process is not None:
            # Stop bag recording
            
            self.get_logger().info('Stopping mapping...')
            self.finish_map_trigger = True
            

            response.success = True
            response.message = 'Finishing map.'
        else:
            response.success = False
            response.message = 'No mapping in progress.'
        return response
    
    def start_localization_callback(self, request, response):
        if self.cartographer_process is None:
            self.map_name = request.map_name
            self.cartographer_process = self.start_localization()
            response.success = True
            response.message = 'Starting localization.'
        else:
            response.success = False
            response.message = 'Localization/Mapping already in progress.'
        return response
    
    def stop_localization_callback(self, request, response):
        if self.cartographer_process is not None:
            self.cartographer_process.send_signal(signal.SIGINT)
            self.cartographer_process = None
            response.success = True
            response.message = 'Localization stopped.'
        else:
            response.success = False
            response.message = 'No localization in progress.'
        return response

    def call_finish_trajectory_service(self):
        while not self.finish_trajectory_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for "finish_trajectory" service...')
        request = FinishTrajectory.Request()
        print("calling finish_trajectory_service")
        future = self.finish_trajectory_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        print(future.result())
        
            
    

    def call_write_state_service(self):
        while not self.write_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for "write_state" service...')
        request = WriteState.Request()
        request.filename = os.path.join(self.output_folder, self.map_name + '.pbstream')
        print("calling write_state_service")
        future = self.write_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        print(future.result())
         
    def run(self):
        while rclpy.ok():
            if self.finish_map_trigger:
                # Call "finish_trajectory" service of Cartographer
                self.call_finish_trajectory_service()
                #time.sleep(1.0)  # Wait for the map to be finished (optional, depending on the map size and the system performance
                self.bag_recorder_process.send_signal(signal.SIGINT)
                self.call_write_state_service()
                self.cartographer_process.send_signal(signal.SIGINT)
                
                self.cartographer_process = None
                self.bag_recorder_process = None
                time.sleep(2.0)
                env = os.environ.copy()
                env["ROS_DOMAIN_ID"] = "89"
                self.cartographer_process = self.start_localization(env)
                self.bag_recorder_process = self.start_subprocess("ros2 bag play " + os.path.join(self.output_folder, self.map_name), env)
                
                
                #TODO: check if localization is OK
                
                self.octomap_server_process = self.start_subprocess('ros2 launch era_5g_cartographer ' + self.octomap_executable, env)
                self.bag_recorder_process.wait()
                self.map_saver_process = self.start_subprocess('ros2 run nav2_map_server map_saver_cli -t /projected_map -f ' + os.path.join(self.output_folder, self.map_name), env)
                self.map_saver_process.wait()
                self.octomap_server_process.send_signal(signal.SIGINT)
                self.cartographer_process.send_signal(signal.SIGINT)
                self.octomap_server_process = None
                self.cartographer_process = None                
                self.map_saver_process = None
                self.finish_map_trigger = False
            elif self.cancel_map_trigger:
                self.call_finish_trajectory_service()
                self.cartographer_process.send_signal(signal.SIGINT)
                self.bag_recorder_process.send_signal(signal.SIGINT)
                self.cartographer_process = None
                self.bag_recorder_process = None
                self.cancel_map_trigger = False
                
            rclpy.spin_once(self)
            

def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='SLAM Network Application')
    parser.add_argument('-t', action='store_true', dest="use_sim_time", help='Flag to use simulated time')
    parser.add_argument('-m', action='store_true', dest="start_mapping", help='Flag to start mapping immediately (dont wait for the service call)')
    parser.add_argument('-o', type=str, dest="output_folder", help='Path to the output folder')
    parser.add_argument('--launch-mapping', type=str, dest="launch_file_mapping", help='Name of the cartographer launch file for mapping')
    parser.add_argument('--launch-localization', type=str, dest="launch_file_localization", help='Name of the cartographer launch file for localization')
    parser.add_argument('--launch-octomap-server', type=str, dest="launch_file_octomap_server", help='Name of the cartographer launch file for the asset writer')
    
    args = parser.parse_args(args)
    # Check if the output file and launch file are provided
    if args.output_folder is None or args.launch_file_mapping is None or args.launch_file_localization is None:
        print("Please provide the output folder path and cartographer launch files names!")
        exit(1)  

    
    node = MapCreatorNode(args.use_sim_time, args.output_folder, args.launch_file_mapping, args.launch_file_localization, args.launch_file_octomap_server)
    
    if args.start_mapping:
        node.start_mapping()
    node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()