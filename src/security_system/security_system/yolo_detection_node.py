
import os
import time
from threading import Thread
import queue
import cv2 as cv
import numpy as np
import math
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, String
import tf2_ros
from .yolo_detect_human_range import *


class YoloDetector(Node):
    """Creates a YOLO object detection model"""

    def __init__(self):
        super().__init__("yolo_detection_node")

        # ROS attributes
        self.publisher_output = self.create_publisher(Image, "camera/yolo", 1)

        #self.publisher_3dbbox = self.create_publisher(MarkerArray, '/person_marker_array', 10)
        # self.publisher_array = self.create_publisher(YoloMsgArray, "yolo/array", 1)

        self.subscriber_camera = self.create_subscription(Image, "/camera/camera/color/image_raw", self.get_frame, 1)

        self.subscriber_depth = self.create_subscription(Image, "/camera/camera/depth/image_rect_raw", self.get_depth, 1)

        self.bridge = CvBridge()

        self.framesQueue_ = self.QueueFPS() 
        self.predictionsQueue_ = self.QueueFPS()
        # self.depthQueue_ = self.QueueFPS() 
        
        self.focal_length = load_camera_calibration("/home/ubuntu24/ros2_ws/src/security_system/security_system/camera_calibration.txt")

        self.rknn = setup_rknn_model('/home/ubuntu24/ros2_ws/src/security_system/security_system/yolov8n-seg-rk3588.rknn')
        self.co_helper = COCO_test_helper(enable_letter_box=True)
        self.new_detections_msg = MarkerArray()

        self.publisher_text = self.create_publisher(String, "string/yolo", 1)

        self.fx =  612.2227172851562 
        self.fy =  612.4107666015625 
        self.cx =  319.5724182128906 
        self.cy =  236.4240264892578 

        self.depth_ = None
        self.depth_scale = 0.0010000000474974513
        self.rate = self.create_rate(2)


    class QueueFPS(queue.Queue):
        def __init__(self):
            queue.Queue.__init__(self, maxsize=1)
            self.startTime = 0
            self.counter = 0
        def put(self, v):
            queue.Queue.put(self, v)
            self.counter += 1
            if self.counter == 1:
                self.startTime = time.time()

        def getFPS(self):
            return self.counter / (time.time() - self.startTime)

    def get_frame(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        self.framesQueue_.put(frame)
    
    def get_depth(self, msg):
        self.depth_ = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        


    def start_detection(self):

        process = True

        processed_frames_queue = queue.Queue()

        def processing_thread_body():

            future_outputs = []
            while process:
                # Get a next frame
                frame = None
                depth = None
                try:
                    frame = self.framesQueue_.get_nowait()
                    self.framesQueue_.queue.clear()  # Skip the rest of frames
                    self.depth_map = self.depth_
                except queue.Empty:
                    pass
                # self.get_logger().info(str(type(frame)))

                if not frame is None and not self.depth_map is None:
                     # Preprocess frame for YOLO
                    img_src = frame.copy()
                    
                    # Letterbox resize
                    pad_color = (0, 0, 0)
                    img = self.co_helper.letter_box(im=img_src.copy(), new_shape=(IMG_SIZE[1], IMG_SIZE[0]), pad_color=pad_color)
                    img = cv.cvtColor(img, cv.COLOR_BGR2RGB)

                    # Run inference
                    outputs = self.rknn.inference(inputs=[img])
                    
                    boxes, classes, scores = post_process(outputs)
                    self.id_bbox = 1

                    self.new_detections_msg.markers = []
                    marker_array_msg = MarkerArray()
                    marker = Marker()
                    marker.id = 0
                    marker.ns = "points"
                    marker.action = Marker.DELETEALL
                    self.new_detections_msg.markers.append(marker)

                    # self.publisher_3dbbox.publish(marker_array_msg)
                    # self.rate.sleep()

                    msg_string_yolo = String()
                    list_msg_string_yolo = []
                    
                    # Draw results on original frame
                    if boxes is not None:
                        list_person = draw(img_src, self.co_helper.get_real_box(boxes), scores, classes, self.focal_length)
                        if len(list_person) > 0:
                            for person_info in list_person:
                                # if person_info[1] is not None:
                                #     self.new_detections_msg.markers.append(self.convert_bb_to_3d(person_info[0], person_info[1]))
                                list_msg_string_yolo.append('\n' + self.convert_bb_to_text(person_info[0], person_info[2]))
                                cube = self.convert_bb_to_cube(person_info[0],  person_info[2])
                                if not cube is None: 
                                    self.new_detections_msg.markers.append(cube)
                                
                                self.id_bbox += 1
                           
                            self.publisher_3dbbox.publish(self.new_detections_msg)
                            
                            msg_string_yolo.data = ''.join(list_msg_string_yolo)
                            self.publisher_text.publish(msg_string_yolo)
                            self.get_logger().info(f'Publishing: "{msg_string_yolo.data}"')

                    msg = self.bridge.cv2_to_imgmsg(img_src, "bgr8")
                    msg.header.frame_id = "camera"
                    self.publisher_output.publish(msg)

        processing_thread = Thread(target=processing_thread_body)
        processing_thread.start()


    def convert_bb_to_3d(
        self,
        bbox, # (top, left, right, bottom)
        distance, # m 
        is_remove=False,
    ):
        left, top, right, bottom = bbox
        
        # Предполагаем, что камера в центре изображения
        principal_point_x = 480 / 2
        principal_point_y = 480 / 2
        
        # Центр bounding box
        bbox_center_x = (left + right) / 2
        bbox_center_y = (top + bottom) / 2
        
        # Размеры куба
        bbox_width_px = right - left
        bbox_height_px = bottom - top
        
        # Угловые отклонения
        dx = bbox_center_x - principal_point_x
        dy = bbox_center_y - principal_point_y
        
        azimuth = math.atan2(dx, self.focal_length)
        elevation = math.atan2(dy, self.focal_length)
        
        # 3D координаты центра
        center_x = distance 
        center_z = 1.7/2
        center_y = -(dx / self.focal_length) * distance 
        

        
        # Масштабируем размеры на основе расстояния
        width_3d = 0.3 # (bbox_width_px / self.focal_length) * distance
        # Корректируем высоту до 1.7м
        size_x = width_3d
        size_y = width_3d
        size_z = 1.7
        

        frame_id = 'map'
        
        # Создаем большую заметную точку
        marker = Marker()
        
        # Заголовок
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        
        # Basic properties
        marker.ns = "debug_points"
        marker.id = self.id_bbox
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Позиция - пробуем разные
        positions = [
            (0.0, 0.0, 1.0),   # прямо вверх от начала координат
            (1.0, 0.0, 0.0),   # вперед по X
            (0.0, 1.0, 0.0),   # влево по Y
            (2.0, 2.0, 2.0)    # далеко во всех направлениях
        ]
        pos = positions[1]
        marker.pose.position.x = center_x
        marker.pose.position.y = center_y
        marker.pose.position.z = center_z 
        
        # Ориентация
        marker.pose.orientation.w = 1.0
        
        # Размер - делаем БОЛЬШИМ
        marker.scale.x = size_x  
        marker.scale.y = size_y
        marker.scale.z = size_z

        # Цвет - яркий и непрозрачный
        colors = [
            (1.0, 0.0, 0.0, 1.0),  # красный
            (0.0, 1.0, 0.0, 1.0),  # зеленый
            (0.0, 0.0, 1.0, 1.0),  # синий
            (1.0, 1.0, 0.0, 1.0),  # желтый
        ]
        color = colors[0]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        
        marker.color.a = 0.3  # полностью непрозрачный
        if is_remove: 
            marker.color.a = 0.0
        # Время жизни
        marker.lifetime.sec = 1  # исчезнет через 2 секунды
        
        return marker

    def get_depth_at_point(self, x, y):
        """
        Получает значение глубины в точке (x, y) с фильтрацией
        """
        x, y = int(x), int(y)
        
        if (x < 0 or x >= self.depth_map.shape[1] or 
            y < 0 or y >= self.depth_map.shape[0]):
            return 0
        
        # Берем небольшую область вокруг точки для устойчивости
        roi_size = 10
        x1 = max(0, x - roi_size // 2)
        x2 = min(self.depth_map.shape[1], x + roi_size // 2 + 1)
        y1 = max(0, y - roi_size // 2)
        y2 = min(self.depth_map.shape[0], y + roi_size // 2 + 1)
        
        roi = self.depth_map[y1:y2, x1:x2]
        
        # Фильтруем нулевые значения
        valid_depths = roi[roi > 0]
        
        if len(valid_depths) == 0:
            return 0
        
        # Возвращаем медиану для устойчивости к выбросам
        return float(np.median(valid_depths))


    def convert_bb_to_cube(
        self,
        bbox,  # (center_x, center_y, width, height) в пикселях
        class_name,  # str
    ):
        """
        Преобразует bounding box YOLO в 3D куб в пространстве камеры
        с использованием карты глубины RealSense
        """

        left, top, right, bottom = bbox
        # Центр bounding box
        center_x = (left + right) / 2
        center_y = (top + bottom) / 2
        
        # Размеры куба
        bbox_width = right - left
        bbox_height = bottom - top
        
        # Получаем глубину в центре bounding box
        depth_value = self.get_depth_at_point(center_x, center_y) * self.depth_scale
        
        if depth_value <= 0: # or depth_value > 5.0:
            return None  # Невалидная глубина
        
        # Параметры камеры (должны быть заданы в классе)
        fx = self.fx  # фокусное расстояние по X
        fy = self.fy  # фокусное расстояние по Y
        cx = 320  # главная точка X
        cy = 240 # главная точка Y
        
        # Преобразуем 2D координаты в 3D (система координат камеры)
        # X - вправо, Y - вниз, Z - вперед
        z = depth_value 
        x = (center_x - cx) * z / fx
        y = (center_y - cy) * z / fy
        
        # Вычисляем размеры объекта в 3D на основе bounding box и глубины
        width_3d = (bbox_width / fx) * z
        height_3d = (bbox_height / fy) * z
        # self.get_logger().info(f'{class_name}, {bbox_width}, {bbox_height}')
        # Глубина объекта (предполагаем пропорционально ширине)
        depth_3d = width_3d  * 0.5 # можно настроить коэффициент
        # self.get_logger().info(f'{class_name}, {str(x)}, {str(y)}, {str(z)}, {str(width_3d)}, {str(height_3d)}, {str(depth_3d)}')
        
        frame_id = 'map'
        
        # Создаем большую заметную точку
        marker = Marker()
        
        # Заголовок
        marker.header = Header()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        
        # Basic properties
        marker.ns = "points"
        marker.id = self.id_bbox
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Позиция центра объекта
        marker.pose.position.x = z  # Z - вперед от камеры
        marker.pose.position.y = -x  
        marker.pose.position.z = -y + 1.0 
        
        # Ориентация (смотрим на камеру)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        # Размеры куба
        marker.scale.x = depth_3d   # глубина
        marker.scale.y = width_3d   # ширина  
        marker.scale.z = height_3d  # высота
        
        # Цвет в зависимости от класса
        color = get_color_for_class(class_name)
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.5  # полупрозрачный
        
        # Время жизни
        # marker.lifetime.sec = rclpy.duration.Duration(seconds=1.0)
        
        return marker


    def convert_bb_to_text(
        self,
        bbox,  # (center_x, center_y, width, height) в пикселях
        class_name,  # str
        img_size=(640, 480),  # (width, height)
    ):
        """
        Преобразует bounding box в пикселях в текстовое описание положения объекта
        
        Args:
            bbox: (center_x, center_y, width, height) в пикселях
            class_name: название класса объекта
            img_size: (width, height) изображения в пикселях
        
        Returns:
            str: текстовое описание положения объекта
        """
        left, top, right, bottom = bbox
        # Центр bounding box
        center_x = (left + right) / 2
        center_y = (top + bottom) / 2
        
        # Размеры куба
        bbox_width = right - left
        bbox_height = bottom - top
        img_width, img_height = img_size
        
        # Определяем горизонтальную зону
        horizontal_zone = ""
        if center_x < img_width * 0.33:
            horizontal_zone = "Слева"
        elif center_x < img_width * 0.66:
            horizontal_zone = "Прямо"
        else:
            horizontal_zone = "Справа"
        
        # Определяем вертикальную зону (опционально)
        vertical_zone = ""
        # if center_y < img_height * 0.33:
        #     vertical_zone = " вверху"
        # elif center_y < img_height * 0.66:
        #     vertical_zone = " в центре"
        # else:
        #     vertical_zone = " внизу"
        
        # Определяем расстояние по размеру bounding box
        bbox_area = bbox_width * bbox_height
        img_area = img_width * img_height
        relative_size = bbox_area / img_area
        
        distance = ""
        if relative_size > 0.3:
            distance = " близко"
        elif relative_size > 0.1:
            distance = ""
        else:
            distance = " далеко"
        
        # Формируем итоговый текст
        text = f"{horizontal_zone}{vertical_zone}{distance} {class_name}"
        
        return text.strip()

def main():
    rclpy.init()
    obj_det = YoloDetector()

    obj_det.start_detection()
    
    try:
        rclpy.spin(obj_det)
    except KeyboardInterrupt:
        pass

    obj_det.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    




