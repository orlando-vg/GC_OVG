#!/usr/bin/env python3
"""
ir_a_puntos.py — Llevar el robot Mecanum por una secuencia de poses (x, y, θ) (Parte 5)

    ros2 run mi_mecanum_control ir_a_puntos --ros-args -p use_sim_time:=true

Ley de la clase, ec. (21), con η̇d = 0 (set-point):

    ξ = [u, v, r]ᵀ = J(φ)⁺ Λ (ηd − η),   Λ = diag(λ1, λ2, λ3)

¿Cuánto vale J(φ) para un robot holonómico? ¿Necesita la fase de
"girar en el sitio" que usaba ir_a_pose en el diferencial?
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

# ===== TODO: puntos de SU variante: (x [m], y [m], θ [rad]) =====
PUNTOS = [
    # (x, y, theta),
]

L1, L2, L3 = 1.0, 1.0, 1.5     # Λ = diag(λ1, λ2, λ3)
V_MAX = 0.4                    # [m/s]  saturación de la NORMA de (u, v)
R_MAX = 1.0                    # [rad/s]
TOL_POS = 0.05                 # [m]
TOL_ANG = 0.05                 # [rad]


def envolver(a):
    """Ángulo en [-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))


def ley_control(pose, meta):
    """
    Ec. (21) con η̇d = 0. Función pura (sin ROS) para poder probarla fuera de Gazebo.

    pose = (x, y, φ)      meta = (xd, yd, θd)
    Devuelve (u, v, r, alcanzado)
    """
    # TODO 1: errores de posición (en odom) y de orientación (envuelto)
    # TODO 2: ¿alcanzado? (TOL_POS y TOL_ANG) -> devolver 0, 0, 0, True
    # TODO 3: ξ = J(φ)⁺ Λ e
    # TODO 4: saturar la NORMA de (u, v) a V_MAX conservando la dirección, y |r| a R_MAX
    raise NotImplementedError


class IrAPuntos(Node):
    def __init__(self):
        super().__init__('ir_a_puntos')
        self.puntos = PUNTOS
        self.i = 0
        self.pose = None

        self.create_subscription(Odometry, '/odom', self.cb_odom, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(0.02, self.lazo)    # 50 Hz
        self.get_logger().info(f'Puntos: {self.puntos}')

    def cb_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        phi = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        self.pose = (p.x, p.y, phi)

    def lazo(self):
        if self.pose is None:
            return
        cmd = Twist()
        if self.i < len(self.puntos):
            u, v, r, alcanzado = ley_control(self.pose, self.puntos[self.i])
            if alcanzado:
                x, y, phi = self.pose
                self.get_logger().info(
                    f'Punto {self.i + 1}/{len(self.puntos)} alcanzado: '
                    f'x={x:.3f}, y={y:.3f}, θ={math.degrees(phi):.1f}°')
                self.i += 1
            cmd.linear.x, cmd.linear.y, cmd.angular.z = u, v, r
        self.pub.publish(cmd)


def main():
    rclpy.init()
    nodo = IrAPuntos()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.pub.publish(Twist())   # detener el robot
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
