import rclpy
from rclpy.node import Node
from zack_interfaces.srv import FaceDetection
import tkinter as tk
from tkinter import filedialog, messagebox
from tkinter import ttk
from PIL import Image, ImageTk
import cv2
import numpy as np
import os
import threading
import tempfile

class FaceRecognitionClient(Node):
    def __init__(self):
        super().__init__('face_recognition_client')
        self.cli = self.create_client(FaceDetection, 'face_detection')
        while not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().info('Service not available, waiting...')
        
        # 初始化GUI
        self.root = tk.Tk()
        self.root.title("人脸识别客户端")
        self.root.geometry("800x600")  # 设置初始窗口大小
        self.root.minsize(400, 300)    # 设置最小窗口尺寸
        
        # 处理窗口关闭事件
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        
        # 创建上传按钮
        self.upload_button = tk.Button(self.root, text="上传图片", command=self.upload_image, width=20, height=2)
        self.upload_button.pack(pady=10)
        
        # 创建一个可滚动的Canvas框架
        self.canvas_frame = ttk.Frame(self.root)
        self.canvas_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # 创建垂直滚动条
        self.v_scroll = ttk.Scrollbar(self.canvas_frame, orient=tk.VERTICAL)
        self.v_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        # 创建水平滚动条
        self.h_scroll = ttk.Scrollbar(self.canvas_frame, orient=tk.HORIZONTAL)
        self.h_scroll.pack(side=tk.BOTTOM, fill=tk.X)

        # 创建Canvas
        self.canvas = tk.Canvas(self.canvas_frame, 
                                yscrollcommand=self.v_scroll.set,
                                xscrollcommand=self.h_scroll.set,
                                background='grey')
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # 配置滚动条
        self.v_scroll.config(command=self.canvas.yview)
        self.h_scroll.config(command=self.canvas.xview)
        
        # 创建结果标签
        self.result_label = tk.Label(self.root, text="人脸数量: ", font=("Helvetica", 16))
        self.result_label.pack(pady=10)
        
        self.future = None  # 初始化future

    def upload_image(self):
        file_path = filedialog.askopenfilename(filetypes=[("Image Files", "*.jpg *.jpeg *.png")])
        if file_path:
            self.get_logger().info(f'您选择的文件路径: {file_path}')
            
            # 检查文件是否存在
            if not os.path.exists(file_path):
                self.get_logger().error('文件不存在，请检查路径是否正确。')
                messagebox.showerror("错误", "文件不存在，请检查路径是否正确。")
                return

            # 尝试读取图像
            try:
                img = cv2.imread(file_path)
                if img is None:
                    self.get_logger().error('无法读取选定的图像文件。请确保图像格式正确且文件未损坏。')
                    messagebox.showerror("错误", "无法读取选定的图像文件。请确保图像格式正确且文件未损坏。")
                    return
                self.get_logger().info(f'图像读取成功，图像尺寸: {img.shape}')
            except Exception as e:
                self.get_logger().error(f'读取图像时发生异常: {e}')
                messagebox.showerror("错误", f'读取图像时发生异常: {e}')
                return

            # 将图像编码为JPEG格式的字节
            try:
                success, buffer = cv2.imencode('.jpg', img)
                if not success:
                    self.get_logger().error('图像编码失败。')
                    messagebox.showerror("错误", "图像编码失败。")
                    return
                img_bytes = buffer.tobytes()
                self.get_logger().info(f'图像编码成功，字节长度: {len(img_bytes)}')
            except Exception as e:
                self.get_logger().error(f'图像编码时发生异常: {e}')
                messagebox.showerror("错误", f'图像编码时发生异常: {e}')
                return

            # 创建服务请求
            request = FaceDetection.Request()
            request.image_data = img_bytes

            # 发送请求
            self.future = self.cli.call_async(request)
            self.get_logger().info('发送服务请求，等待响应...')
            self.root.after(100, self.check_response)

    def check_response(self):
        if self.future and self.future.done():
            try:
                response = self.future.result()
                self.get_logger().info(f'服务响应接收成功，识别到的人脸数量: {response.face_count}')
            except Exception as e:
                self.get_logger().error(f'Service call failed: {e}')
                messagebox.showerror("错误", f'Service call failed: {e}')
                self.future = None
                return

            # 更新结果标签
            self.result_label.config(text=f"人脸数量: {response.face_count}")

            # 检查响应中是否包含标注图像数据
            if not response.annotated_image_data:
                self.get_logger().error('服务响应中没有包含标注的图像数据。')
                messagebox.showerror("错误", "服务响应中没有包含标注的图像数据。")
                self.future = None
                return

            self.get_logger().info(f'标注图像数据的字节长度: {len(response.annotated_image_data)}')

            # 将标注后的图像保存到本地
            try:
                save_path = os.path.join(tempfile.gettempdir(), "annotated_image.jpg")
                with open(save_path, "wb") as f:
                    f.write(response.annotated_image_data)
                self.get_logger().info(f'标注后的图像已保存至: {save_path}')
            except Exception as e:
                self.get_logger().error(f'保存标注图像时发生异常: {e}')
                messagebox.showerror("错误", f'保存标注图像时发生异常: {e}')

            # 显示标注后的图像
            try:
                annotated_img = cv2.imdecode(np.frombuffer(response.annotated_image_data, np.uint8), cv2.IMREAD_COLOR)
                if annotated_img is not None:
                    # 转换颜色空间 BGR 到 RGB
                    annotated_img = cv2.cvtColor(annotated_img, cv2.COLOR_BGR2RGB)
                    img_pil = Image.fromarray(annotated_img)
                    self.display_image(img_pil)
                    self.get_logger().info('标注后的图像显示成功。')
                else:
                    self.get_logger().error('无法解码返回的图像数据。')
                    messagebox.showerror("错误", "无法解码返回的图像数据。")
            except Exception as e:
                self.get_logger().error(f'处理标注后的图像时发生异常: {e}')
                messagebox.showerror("错误", f'处理标注后的图像时发生异常: {e}')

            self.future = None  # 重置future以便下次请求
        else:
            self.root.after(100, self.check_response)

    def display_image(self, img_pil):
        # 设置最大显示尺寸为原图的大小，防止缩小
        # 如果需要缩放图像，可以在此处添加缩放逻辑

        # 获取图像的原始尺寸
        img_width, img_height = img_pil.size

        # 设置Canvas的大小与图像相同
        self.canvas.config(width=img_width, height=img_height)

        # 将PIL图像转换为ImageTk
        img_tk = ImageTk.PhotoImage(img_pil)

        # 清除之前的图像（如果有）
        self.canvas.delete("all")

        # 在Canvas上创建图像
        self.canvas.create_image(0, 0, anchor='nw', image=img_tk)

        # 配置Canvas的滚动区域
        self.canvas.config(scrollregion=self.canvas.bbox(tk.ALL))

        # 保持对图像的引用
        self.canvas.image = img_tk

    def run(self):
        self.root.after(100, self.check_response)
        self.root.mainloop()

    def on_close(self):
        self.get_logger().info('正在关闭ROS2并退出GUI。')
        rclpy.shutdown()
        self.root.destroy()

def main(args=None):
    rclpy.init(args=args)
    client = FaceRecognitionClient()

    # 在单独的守护线程中运行ROS2 spin
    spin_thread = threading.Thread(target=rclpy.spin, args=(client,), daemon=True)
    spin_thread.start()

    # 运行Tkinter的主循环
    client.run()

    # 确保ROS2关闭
    if rclpy.ok():
        rclpy.shutdown()

if __name__ == '__main__':
    main()