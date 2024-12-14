'''
Credits:
- Edouardo Renard
'''

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyCustomNode(Node): # modify name
    def __init__(self):
        super().init("node_name") # modift name

    def main(args=None):
        rclpy.init(args=None)
        node = MyCustomNode() # modify name
        rclpy.spin(node)
        rclpy.shutdown()

    if __name__ == "__main__":
        main()