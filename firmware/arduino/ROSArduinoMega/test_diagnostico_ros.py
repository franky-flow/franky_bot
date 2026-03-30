#!/usr/bin/env python3
"""
Script de diagnóstico para verificar comunicación ROS → Arduino
Monitorea los comandos que ROS envía al Arduino
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped
import time

class DiagnosticoROS(Node):
    def __init__(self):
        super().__init__('diagnostico_ros')
        
        # Suscribirse a cmd_vel
        self.subscription = self.create_subscription(
            TwistStamped,
            '/diff_cont/cmd_vel',  # Ajustar según tu topic
            self.cmd_vel_callback,
            10)
        
        self.last_time = time.time()
        self.count = 0
        
        self.get_logger().info('=== DIAGNÓSTICO ROS → ARDUINO ===')
        self.get_logger().info('Monitoreando /diff_cont/cmd_vel')
        self.get_logger().info('Mueve el joystick y observa la frecuencia...\n')
    
    def cmd_vel_callback(self, msg):
        self.count += 1
        current_time = time.time()
        dt = current_time - self.last_time
        
        # Mostrar cada 10 mensajes
        if self.count % 10 == 0:
            freq = 10.0 / (current_time - self.start_time) if hasattr(self, 'start_time') else 0
            self.get_logger().info(
                f'Msg #{self.count} | '
                f'Linear: {msg.twist.linear.x:.3f} | '
                f'Angular: {msg.twist.angular.z:.3f} | '
                f'dt: {dt*1000:.1f}ms | '
                f'Freq: {1/dt if dt>0 else 0:.1f}Hz'
            )
            self.start_time = current_time
        
        self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = DiagnosticoROS()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n=== FIN DIAGNÓSTICO ===')
        node.get_logger().info(f'Total mensajes recibidos: {node.count}')
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()