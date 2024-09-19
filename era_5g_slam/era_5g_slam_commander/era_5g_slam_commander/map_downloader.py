import rclpy
from rclpy.node import Node
from era_5g_interfaces.srv import DownloadMap
import os
import argparse

class FileReaderNode(Node):
    def __init__(self, folder_path):
        super().__init__('file_reader_node')
        self.folder_path = folder_path
        self.srv = self.create_service(DownloadMap, 'download_map', self.read_file_callback)

    def read_file_callback(self, request, response):
        map_file = os.path.join(self.folder_path, request.map_id + ".pgm")
        yaml_file = os.path.join(self.folder_path, request.map_id + ".yaml")
        try:
            with open(map_file, 'rb') as file:  # Open file in binary mode ('rb')
                content = file.read()   # Read binary data
                response.map = content if content else bytes()
            with open(yaml_file, 'r') as file:
                response.yaml = file.read() 
                self.get_logger().info(f"Read file '{yaml_file}' successfully.")
            
        except FileNotFoundError:
            self.get_logger().error(f"File '{yaml_file}' not found.")
            response.map = bytes()  # Initialize empty bytes for response
            response.yaml = ""
        except Exception as e:
            self.get_logger().error(f"Error reading file '{yaml_file}': {str(e)}")
            response.map = bytes()  # Initialize empty bytes for response
            response.yaml = ""
        finally:
            return response

def main(args=None):
    rclpy.init(args=args)
    
    parser = argparse.ArgumentParser(description='ROS 2 File Reader Node')
    parser.add_argument('folder_path', type=str, help='Folder path containing the files')
    args = parser.parse_args(args)

    file_reader_node = FileReaderNode(args.folder_path)
    rclpy.spin(file_reader_node)
    file_reader_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()