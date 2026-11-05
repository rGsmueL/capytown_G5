
#!/usr/bin/env python3

import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class PruebaRecta(Node):

    def __init__(self):
        super().__init__('prueba_recta')

        self.pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.sub = self.create_subscription(
            Odometry, '/odom', self.recibir_odom, 10
        )

        self.x = None
        self.y = None
        self.datos_recibidos = threading.Event()

    def recibir_odom(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.datos_recibidos.set()

    def velocidad(self, linear_x):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = 0.0
        self.pub.publish(msg)


def main():
    rclpy.init()
    robot = PruebaRecta()
    hilo = threading.Thread(target=rclpy.spin, args=(robot,), daemon=True)
    hilo.start()

    try:
        print('Esperando datos de /odom...')
        if not robot.datos_recibidos.wait(timeout=10):
            raise RuntimeError('No se reciben mensajes de /odom.')

        # Posición inicial.
        x0, y0 = robot.x, robot.y
        print(f'Inicio: x={x0:.4f}, y={y0:.4f}')

        print('Avanzando durante 10 segundos...')
        robot.velocidad(0.10)
        time.sleep(10.5)

    finally:
        # Enviar la parada aunque ocurra un error o Ctrl+C.
        if rclpy.ok():
            robot.velocidad(0.0)
            time.sleep(0.2)

    # Esperar a recibir una lectura actualizada después de frenar.
    robot.datos_recibidos.clear()
    recibido = robot.datos_recibidos.wait(timeout=3)

    if recibido:
        x1, y1 = robot.x, robot.y
        distancia = ((x1 - x0)**2 + (y1 - y0)**2)**0.5

        print(f'Final: x={x1:.4f}, y={y1:.4f}')
        print(f'Distancia según /odom: {distancia:.4f} m')
        print('Mide con cinta la distancia física recorrida.')
    else:
        print('No se recibió una lectura final de /odom.')

    robot.destroy_node()
    rclpy.shutdown()
    hilo.join(timeout=1)


if __name__ == '__main__':
    main()
