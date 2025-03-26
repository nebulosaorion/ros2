import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TextPublisher(Node):
    def __init__(self):
        super().__init__('text_publisher')
        self.publisher = self.create_publisher(String, '/ui_messages', 10)
        self.timer = self.create_timer(1.0, self.publish_text)
        self.messages = [
            "Assista ao movimento do braço robótico.",
            "Agora, levante o objeto devagar.",
            "Pronto! Movimento concluído."
        ]
        self.current_msg = 0

    def publish_text(self):
        msg = String()
        msg.data = self.messages[self.current_msg]
        self.publisher.publish(msg)
        self.get_logger().info(f'Publicando: "{msg.data}"')
        self.current_msg = (self.current_msg + 1) % len(self.messages)

def main(args=None):
    rclpy.init(args=args)
    node = TextPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()