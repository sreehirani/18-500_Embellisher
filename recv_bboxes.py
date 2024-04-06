import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
from path_planning import *

class Subscriber(Node):

    def __init__(self):
        super().__init__('bbox_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

        # convert str back into proper lists
        bboxes: list[list[int]] = []
        scores: list[float] = []
        labels: list[str] = []
        res = str(msg.data)
        if (res != '[]--[]--[]'):
            bboxes_str, scores_str, labels_str = res.split("--")
            bbox_parse = bboxes_str.strip('][').split("], [")
            bbox_parse[0] = bbox_parse[0].strip("[")
            bbox_parse[-1] = bbox_parse[-1].strip("]")

            for bbox in bbox_parse:
                bboxes.append([eval(i) for i in bbox.split(', ')])
            scores: list[float] = [eval(i) for i in scores_str.strip('][').split(', ')]
            labels: list[str] = labels_str.strip('][').split(', ')

        # send resultant data to path finding algo
        path_finding_algo(bboxes, scores, labels)

        # artificial delay
        time.sleep(0.5)

if __name__ == '__main__':
    rclpy.init()
    subscriber = Subscriber()
    rclpy.spin(subscriber)

    # Destroy the node explicitly
    subscriber.destroy_node()
    rclpy.shutdown()
