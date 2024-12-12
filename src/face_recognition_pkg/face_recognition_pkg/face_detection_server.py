import rclpy
from rclpy.node import Node
from zack_interfaces.srv import FaceDetection
import cv2
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

class FaceDetectionServer(Node):
    def __init__(self):
        super().__init__('face_detection_server')
        self.srv = self.create_service(FaceDetection, 'face_detection', self.face_detection_callback)
        self.get_logger().info('Face Detection Service is ready.')

        # 获取包的共享目录路径
        package_share = get_package_share_directory('face_recognition_pkg')
        cascade_path = os.path.join(package_share, 'models', 'haarcascade_frontalface_default.xml')
        self.get_logger().info(f'加载 Haar Cascade 分类器: {cascade_path}')
        self.face_cascade = cv2.CascadeClassifier(cascade_path)

        if self.face_cascade.empty():
            self.get_logger().error('无法加载 Haar Cascade 分类器。请检查文件路径是否正确。')
            raise FileNotFoundError('无法加载 Haar Cascade 分类器。请检查文件路径是否正确。')

        # 声明参数
        self.declare_parameter('scale_factor', 1.1)
        self.declare_parameter('min_neighbors', 20)
        self.declare_parameter('min_size', [30, 30])

    def face_detection_callback(self, request, response):
        self.get_logger().info('Received image for face detection.')

        # 将字节数据转换为OpenCV图像
        nparr = np.frombuffer(request.image_data, np.uint8)
        img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

        if img is None:
            self.get_logger().error('Invalid image data received.')
            response.face_count = 0
            response.annotated_image_data = bytes()
            response.error_message = '无效的图像数据。'
            return response

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 获取参数
        scale_factor = self.get_parameter('scale_factor').get_parameter_value().double_value
        min_neighbors = self.get_parameter('min_neighbors').get_parameter_value().integer_value
        min_size = self.get_parameter('min_size').get_parameter_value().double_array_value
        min_size = tuple(map(int, min_size)) if min_size else (30, 30)

        # 检测人脸
        faces = self.face_cascade.detectMultiScale(gray, scaleFactor=scale_factor, minNeighbors=min_neighbors, minSize=min_size)

        # 画蓝色矩形框
        for (x, y, w, h) in faces:
            cv2.rectangle(img, (x, y), (x+w, y+h), (255, 0, 0), 2)

        # 编码图像回字节
        ret, buffer = cv2.imencode('.jpg', img)
        if not ret:
            self.get_logger().error('图像编码失败。')
            response.face_count = len(faces)
            response.annotated_image_data = bytes()
            response.error_message = '图像编码失败。'
            return response
        annotated_image = buffer.tobytes()

        response.face_count = len(faces)
        response.annotated_image_data = annotated_image
        response.error_message = ''

        self.get_logger().info(f'Detected {len(faces)} face(s).')

        return response

def main(args=None):
    rclpy.init(args=args)
    server = FaceDetectionServer()
    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        server.get_logger().info('Shutting down Face Detection Service.')
    rclpy.shutdown()

if __name__ == '__main__':
    main()