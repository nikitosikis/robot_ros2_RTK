import rclpy
import math
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA


class CameraVisualizer:
    def __init__(self, node):
        self.node = node
        self.marker_pub = self.node.create_publisher(MarkerArray, '/camera_visualization', 10)
        

    def create_camera_fov_marker(self, camera_pos, camera_orientation, fov_horizontal, fov_vertical, 
                               max_range=5.0, frame_id="base_link", camera_id=0, color=(0.0, 1.0, 0.0, 0.5)):
        """
        Создает визуализацию камеры с полем зрения
        
        Args:
            camera_pos: (x, y, z) позиция камеры
            camera_orientation: (roll, pitch, yaw) ориентация камеры в радианах
            fov_horizontal: горизонтальный угол обзора в градусах
            fov_vertical: вертикальный угол обзора в градусах
            max_range: максимальная дальность видимости поля зрения
            frame_id: система координат
            camera_id: идентификатор камеры
            color: (r, g, b, a) цвет поля зрения
        """
        marker_array = MarkerArray()
        
        pos_x, pos_y, pos_z = camera_pos
        roll, pitch, yaw = camera_orientation
        
        # 1. ТЕЛО КАМЕРЫ (маленький куб)
        camera_body = self._create_camera_body(pos_x, pos_y, pos_z, roll, pitch, yaw, frame_id, camera_id)
        marker_array.markers.append(camera_body)
        
        # 2. ЛИНЗА КАМЕРЫ (сфера)
        camera_lens = self._create_camera_lens(pos_x, pos_y, pos_z, roll, pitch, yaw, frame_id, camera_id)
        marker_array.markers.append(camera_lens)
        
        # 3. ПОЛЕ ЗРЕНИЯ (пирамида)
        fov_markers = self._create_fov_pyramid(pos_x, pos_y, pos_z, roll, pitch, yaw, 
                                             fov_horizontal, fov_vertical, max_range, 
                                             frame_id, camera_id, color)
        marker_array.markers.extend(fov_markers)
        
        return marker_array

    def _create_camera_body(self, pos_x, pos_y, pos_z, roll, pitch, yaw, frame_id, camera_id):
        """Создает тело камеры"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "camera_body"
        marker.id = camera_id * 10 + 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        marker.pose.position.x = pos_x
        marker.pose.position.y = pos_y
        marker.pose.position.z = pos_z
        
        marker.scale.x = 0.08
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 0.2
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 1.0
        
        return marker

    def _create_camera_lens(self, pos_x, pos_y, pos_z, roll, pitch, yaw, frame_id, camera_id):
        """Создает линзу камеры"""
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.node.get_clock().now().to_msg()
        marker.ns = "camera_lens"
        marker.id = camera_id * 10 + 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Смещаем линзу немного вперед по направлению камеры
        lens_offset = 0.04
        forward_x = math.cos(pitch) * math.cos(yaw)
        forward_y = math.cos(pitch) * math.sin(yaw) 
        forward_z = math.sin(pitch)
        
        marker.pose.position.x = pos_x + forward_x * lens_offset
        marker.pose.position.y = pos_y + forward_y * lens_offset
        marker.pose.position.z = pos_z + forward_z * lens_offset
        
        
        marker.scale.x = 0.04
        marker.scale.y = 0.04
        marker.scale.z = 0.04
        marker.color.r = 0.9
        marker.color.g = 0.9
        marker.color.b = 0.1
        marker.color.a = 1.0
        
        return marker

    def _create_fov_pyramid(self, pos_x, pos_y, pos_z, roll, pitch, yaw, 
                          fov_h, fov_v, max_range, frame_id, camera_id, color):
        """Создает пирамиду поля зрения"""
        markers = []
        
        fov_h_rad = math.radians(fov_h)
        fov_v_rad = math.radians(fov_v)
        
        half_fov_h = fov_h_rad / 2
        half_fov_v = fov_v_rad / 2
        
        # Углы пирамиды в локальной системе камеры
        corners_local = [
            (max_range, max_range * math.tan(half_fov_h), max_range * math.tan(half_fov_v)),   # правый верхний
            (max_range, max_range * math.tan(half_fov_h), -max_range * math.tan(half_fov_v)),  # правый нижний
            (max_range, -max_range * math.tan(half_fov_h), -max_range * math.tan(half_fov_v)), # левый нижний
            (max_range, -max_range * math.tan(half_fov_h), max_range * math.tan(half_fov_v))   # левый верхний
        ]
        
        # Преобразуем в глобальные координаты
        camera_center = Point(x=pos_x, y=pos_y, z=pos_z)
        corners_global = []
        
        for corner in corners_local:
            x_local, y_local, z_local = corner
            
            # Поворот согласно ориентации камеры
            x_rot = (x_local * math.cos(yaw) * math.cos(pitch) -
                    y_local * math.sin(yaw) +
                    z_local * math.cos(yaw) * math.sin(pitch))
                    
            y_rot = (x_local * math.sin(yaw) * math.cos(pitch) +
                    y_local * math.cos(yaw) +
                    z_local * math.sin(yaw) * math.sin(pitch))
                    
            z_rot = (-x_local * math.sin(pitch) +
                    z_local * math.cos(pitch))
            
            corners_global.append(Point(
                x=pos_x + x_rot,
                y=pos_y + y_rot,
                z=pos_z + z_rot
            ))
        
        # 3A. ЛИНИИ ПОЛЯ ЗРЕНИЯ
        fov_lines = Marker()
        fov_lines.header.frame_id = frame_id
        fov_lines.header.stamp = self.node.get_clock().now().to_msg()
        fov_lines.ns = "camera_fov_lines"
        fov_lines.id = camera_id * 10 + 2
        fov_lines.type = Marker.LINE_LIST
        fov_lines.action = Marker.ADD
        fov_lines.pose.orientation.w = 1.0
        fov_lines.scale.x = 0.01
        fov_lines.color.r = color[0]
        fov_lines.color.g = color[1]
        fov_lines.color.b = color[2]
        fov_lines.color.a = color[3]
        
        # Линии от камеры к углам пирамиды
        for corner in corners_global:
            fov_lines.points.append(camera_center)
            fov_lines.points.append(corner)
        
        # Линии дальнего прямоугольника
        for i in range(4):
            fov_lines.points.append(corners_global[i])
            fov_lines.points.append(corners_global[(i + 1) % 4])
        
        markers.append(fov_lines)
        
        # 3B. ЗАПОЛНЕННАЯ ОБЛАСТЬ ПОЛЯ ЗРЕНИЯ
        fov_filled = Marker()
        fov_filled.header.frame_id = frame_id
        fov_filled.header.stamp = self.node.get_clock().now().to_msg()
        fov_filled.ns = "camera_fov_filled"
        fov_filled.id = camera_id * 10 + 3
        fov_filled.type = Marker.TRIANGLE_LIST
        fov_filled.action = Marker.ADD
        fov_filled.pose.orientation.w = 1.0
        fov_filled.scale.x = 1.0
        fov_filled.scale.y = 1.0
        fov_filled.scale.z = 1.0
        fov_filled.color.r = color[0]
        fov_filled.color.g = color[1]
        fov_filled.color.b = color[2]
        fov_filled.color.a = color[3] * 0.3  # более прозрачный
        
        # Боковые грани пирамиды
        for i in range(4):
            fov_filled.points.append(camera_center)
            fov_filled.points.append(corners_global[i])
            fov_filled.points.append(corners_global[(i + 1) % 4])
        
        # Дальняя грань (два треугольника)
        fov_filled.points.append(corners_global[0])
        fov_filled.points.append(corners_global[1])
        fov_filled.points.append(corners_global[2])
        
        fov_filled.points.append(corners_global[0])
        fov_filled.points.append(corners_global[2])
        fov_filled.points.append(corners_global[3])
        
        markers.append(fov_filled)
        
        return markers

    def publish_camera(self, camera_pos, camera_orientation, fov_horizontal, fov_vertical, **kwargs):
        """Публикует визуализацию камеры"""
        marker_array = self.create_camera_fov_marker(
            camera_pos, camera_orientation, fov_horizontal, fov_vertical, **kwargs
        )
        self.marker_pub.publish(marker_array)


def main():
    rclpy.init()
    node = rclpy.create_node('camera_visualizer')
    
    visualizer = CameraVisualizer(node)
    
    # Параметры камеры
    camera_pos = (0.0, 0.0, 1.0)  # (x, y, z)
    camera_orientation = (0.0, 0.0, 0.0)  # (roll, pitch, yaw) в радианах
    fov_horizontal = 60.0  # градусы
    fov_vertical = 60.0   # градусы
    
    # Таймер для публикации
    def timer_callback():
        visualizer.publish_camera(
            camera_pos=camera_pos,
            camera_orientation=camera_orientation,
            fov_horizontal=fov_horizontal,
            fov_vertical=fov_vertical,
            max_range=3.0,
            frame_id="map",
            color=(0.0, 1.0, 0.0, 0.5)  # зеленый
        )
        node.get_logger().info('Camera visualization published')
    
    timer = node.create_timer(1.0, timer_callback)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
