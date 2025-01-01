import rclpy
from rclpy.node import Node

class Ventilator(Node):
    def __init__(self):
        super().__init__('Ventilator')

def main(args=None):
    rclpy.init(args=args)
    node = Ventilator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
