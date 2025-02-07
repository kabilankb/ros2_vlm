import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from PIL import Image as PILImage, ImageDraw, ImageFont
import numpy as np
from nano_llm import NanoLLM, ChatHistory

class Nano_LLM_Subscriber(Node):

    def __init__(self):
        super().__init__('nano_llm_subscriber')

        # Declare parameters
        self.declare_parameter('model', "Efficient-Large-Model/VILA1.5-3b")
        self.declare_parameter('api', "mlc")
        self.declare_parameter('quantization', "q4f16_ft")

        # Subscriber for input query
        self.query_subscription = self.create_subscription(
            String,
            'input_query',
            self.query_listener_callback,
            10)

        # Subscriber for input image
        self.image_subscription = self.create_subscription(
            Image,
            '/rgb',
            self.image_listener_callback,
            10)

        # Publisher for output (text)
        self.output_publisher = self.create_publisher(String, 'output', 10)

        # Publisher for output image with caption for RViz
        self.image_caption_publisher = self.create_publisher(Image, 'output_image', 10)

        # To convert ROS image message to OpenCV image
        self.cv_br = CvBridge()

        # Load the model
        self.model = NanoLLM.from_pretrained("Efficient-Large-Model/VILA1.5-3b")

        # Initialize chat history
        self.chat_history = ChatHistory(self.model)

        self.query = ""
        self.process_flag = False  # Flag to control processing

    def query_listener_callback(self, msg):
        command = msg.data.lower().strip()
        self.get_logger().info(f"Received query: '{command}'")

        # Enable processing for any command received
        self.process_flag = True
        self.query = command
        self.get_logger().info("Processing enabled.")

    def image_listener_callback(self, data):
        if not self.process_flag:
            # Removed log statement about ignoring image
            return

        input_query = self.query

        try:
            # Check the image encoding
            if data.encoding == '8UC3':
                cv_img = self.cv_br.imgmsg_to_cv2(data, desired_encoding='passthrough')
                cv_img_rgb = cv_img[:, :, ::-1]  # Convert BGR to RGB
            else:
                cv_img = self.cv_br.imgmsg_to_cv2(data, 'bgr8')
                cv_img_rgb = cv_img[:, :, ::-1]  # Convert BGR to RGB

            # Convert to PIL image
            PIL_img = PILImage.fromarray(cv_img_rgb)

            # Process the command
            self.get_logger().info('Processing image based on the command...')
            self.chat_history.append('user', image=PIL_img)
            self.chat_history.append('user', input_query, use_cache=True)
            embedding, _ = self.chat_history.embed_chat()

            output = self.model.generate(
                inputs=embedding,
                kv_cache=self.chat_history.kv_cache,
                min_new_tokens=10,
                streaming=False,
                do_sample=True,
            )

            # Publish output as text (for rqt)
            output_msg = String()
            output_msg.data = output
            self.output_publisher.publish(output_msg)
            self.get_logger().info(f"Published output: {output}")

            # Overlay text (output) onto the image for RViz
            draw = ImageDraw.Draw(PIL_img)
            font = ImageFont.load_default()
            green_color = (0, 255, 0)
            draw.text((10, 10), output, font=font, fill=green_color)

            # Convert the PIL image back to OpenCV image
            annotated_cv_img = np.array(PIL_img)

            # Convert OpenCV image back to ROS Image message
            annotated_ros_img = self.cv_br.cv2_to_imgmsg(annotated_cv_img, encoding="rgb8")

            # Publish annotated image for RViz
            self.image_caption_publisher.publish(annotated_ros_img)

            # Reset chat history after generating output
            self.chat_history.reset()

            # Disable further processing until a new command is received
            self.process_flag = False

        except CvBridgeError as e:
            self.get_logger().error(f"Failed to convert image: {e}")
        except Exception as e:
            self.get_logger().error(f"An error occurred: {e}")

def main(args=None):
    rclpy.init(args=args)

    nano_llm_subscriber = Nano_LLM_Subscriber()

    rclpy.spin(nano_llm_subscriber)

    # Cleanup
    nano_llm_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
