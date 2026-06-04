import os
import sys
import shutil
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist

import threading
import time
from typing import List, Tuple, Optional

import copy

class TaskProcessor(Node):
    def __init__(self):
        super().__init__('kursant1_task_processor')

        self.is_running = False
        self.processing_thread: Optional[threading.Thread] = None
        
        self.task_queue_lock = threading.Lock()
        self.task_queue = []
        
        self.task_state_lock = threading.Lock()
        self.task_state = 'wait'
        self.task_id = -1
        self.task_vx = 0.0
        self.task_vrz = 0.0
        self.task_duration = 0.0
        
        self.task_subscriber = self.create_subscription(String, 'kursant1_task', self.task_callback, 10)
        
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.task_state_publisher = self.create_publisher(String, 'task_state', 10)
        
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.start_processing()
        
    
    def timer_callback(self):
      with self.task_state_lock:
          tid = self.task_id
          tstate = self.task_state
      
      msg_state = String()
      msg_state.data = f"{tid} {tstate}"
      self.task_state_publisher.publish(msg_state)
    
    def task_callback(self, msg):
        cmd = msg.data
        with self.task_queue_lock:
            q = copy.deepcopy(self.task_queue)  
        
        cmd_id = int(cmd.split()[0])
        q_ids = [int(x.split()[0]) for x in q]
        
        if not cmd_id in q_ids:
            with self.task_queue_lock:
                self.task_queue.append(cmd)
        
    def start_processing(self):
        """Запуск потока для обработки заданий"""
        self.is_running = True
        self.processing_thread = threading.Thread(target=self._processing_loop)
        self.processing_thread.daemon = True
        self.processing_thread.start()
        self.get_logger().info("Started processing thread")
    
    def _processing_loop(self):
        """Основной цикл публикации сообщений"""
        while self.is_running and rclpy.ok():
            if not self.task_queue_lock:
              break
            
            with self.task_queue_lock:
              if len(self.task_queue)==0:
                with self.task_state_lock:
                  self.task_state = 'wait'
                  self.task_id = -1
              else:
                task_string = self.task_queue[0]
                task_spl = task_string.strip().split()
                with self.task_state_lock:
                  self.task_state = 'acquired'
                  self.task_id = int(task_spl[0])
                  self.task_vx = float(task_spl[1])
                  self.task_vrz = float(task_spl[2])
                  self.task_duration = float(task_spl[3])
                  
            
            
            
            with self.task_state_lock:
              state = self.task_state
              #print(self.task_id)
            
            if state=='wait':
              time.sleep(0.1)
              continue
            
            
            with self.task_state_lock:
              tid = self.task_id
              vx = self.task_vx
              vrz = self.task_vrz
              duration = self.task_duration
              
            
            with self.task_state_lock:
              self.task_state = 'reset'
            
            msg_twist_reset = Twist()
            msg_twist_reset.linear.x = 0.0
            msg_twist_reset.angular.z = 0.0
            self.cmd_vel_publisher.publish(msg_twist_reset)
            self.get_logger().info(f"RESET: [id:{tid}, {vx}, {vrz}]: (waiting 0.1s)")
            time.sleep(0.1)
            
            with self.task_state_lock:
              self.task_state = 'processing'
            
            msg_twist = Twist()
            msg_twist.linear.x = vx
            msg_twist.angular.z = vrz
            self.cmd_vel_publisher.publish(msg_twist)
            self.get_logger().info(f"PROCESSING: [id:{tid}, {vx}, {vrz}]: (waiting {duration}s)")
            time.sleep(duration)
            
            with self.task_state_lock:
              self.task_state = 'finishing'
            
            #msg_twist_reset = Twist()
            #msg_twist_reset.linear.x = 0.0
            #msg_twist_reset.angular.z = 0.0
            self.cmd_vel_publisher.publish(msg_twist_reset)
            self.get_logger().info(f"FINISHING: [id:{tid}, {vx}, {vrz}]: (waiting 0.1s)")
            time.sleep(0.1)
            
            with self.task_queue_lock:
                self.task_queue.pop(0)
            
            self.get_logger().info(f"Removed task id:{tid} [{vx}, {vrz}] from queue")
            
            with self.task_state_lock:
              self.task_state = 'wait'
            
            
        
        #msg_comment = String()
        #msg_comment.data = "Finalization"
        #self.publisher_cmd_comment.publish(msg_comment)
        
        msg_twist = Twist()
        msg_twist.linear.x = 0.0
        msg_twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(msg_twist)
        
        self.is_running = False
        self.get_logger().info("Task processing thread finished")
    
    def cleanup(self):
        """Очистка ресурсов"""
        self.get_logger().info("Cleaning up...")
        
        self.is_running = False
        if self.processing_thread and self.processing_thread.is_alive():
            self.processing_thread.join(timeout=2.0)   




def main(args=None):
    # Парсинг аргументов командной строки
                       
    
    rclpy.init(args=args)
    
    # Получаем аргументы командной строки
    #raw_args = rclpy.utilities.remove_ros_args(args)
    #parsed_args = parser.parse_args(raw_args[1:])
    
    # Создаем узел
    node = TaskProcessor()
    
    try:
        # Запускаем spinning в отдельном потоке
        import threading
        
        def spin_node():
            rclpy.spin(node)
        
        spin_thread = threading.Thread(target=spin_node)
        spin_thread.start()
        
        # Ждем завершения потока публикации
        if node.processing_thread:
            node.processing_thread.join()
        
        # Останавливаем spinning
        rclpy.shutdown()
        spin_thread.join()
        
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        node.cleanup()
        node.destroy_node()
        
    #rclpy.init(args=args)

    #twist_subscriber = TwistSubscriber()

    #rclpy.spin(twist_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    #twist_subscriber.destroy_node()
    #rclpy.shutdown()



if __name__ == '__main__':
    main()
