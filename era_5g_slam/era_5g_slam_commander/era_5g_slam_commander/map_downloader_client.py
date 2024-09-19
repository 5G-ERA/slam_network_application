import argparse
import rclpy
from rclpy.node import Node
from era_5g_interfaces.srv import DownloadMap
import os

class FileReaderClient(Node):
    def __init__(self):
        super().__init__('map_downloader_client')

    def call_read_file_service(self, map_id):
        client = self.create_client(DownloadMap, 'download_map')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        request = DownloadMap.Request()
        request.map_id = map_id
        future = client.call_async(request)
        
        file_path_map = os.path.join(os.path.curdir, map_id + ".pgm")  
        file_path_yaml = os.path.join(os.path.curdir, map_id + ".yaml") 
        while rclpy.ok():
            rclpy.spin_once(self)
            if future.done():
                try:
                    response = future.result()
                    if response.map and response.yaml:
                        try:
                            with open(file_path_map, 'wb') as file:
                                file.write(response.map)
                            with open(file_path_yaml, 'w') as file:
                                file.write(response.yaml)
                        except Exception as e:
                            self.get_logger().error(f"Error saving file: {str(e)}")
                    else:
                        self.get_logger().error(f"Failed to obtain map files '{map_id}'")
                except Exception as e:
                    self.get_logger().error(f"Service call failed: {str(e)}")
                break

    
def main(args=None):
    rclpy.init(args=args)
    parser = argparse.ArgumentParser(description='ROS 2 File Reader Node')
    parser.add_argument('map_id', type=str, help='Folder path containing the files')
    args = parser.parse_args(args)
    
    file_reader_client = FileReaderClient()
    file_reader_client.call_read_file_service(args.map_id)
    rclpy.shutdown()

if __name__ == '__main__':
    main()