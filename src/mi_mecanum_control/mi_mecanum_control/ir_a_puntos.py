#!/usr/bin/env python3
"""
ir_a_puntos.py — Llevar el robot Mecanum por una secuencia de poses (x, y, θ) (Parte 5)

    ros2 run mi_mecanum_control ir_a_puntos --ros-args -p use_sim_time:=true

Ley de la clase, ec. (21), con η̇d = 0 (set-point):

    ξ = [u, v, r]ᵀ = J(φ)⁺ Λ (ηd − η),   Λ = diag(λ1, λ2, λ3)

Robot holonómico: J(φ) es la matriz de rotación 3x3 (cuadrada e invertible),
así que J(φ)⁺ = J(φ)⁻¹ = J(φ)ᵀ. No necesita la fase de girar en el sitio:
se traslada y gira al mismo tiempo.
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

# ===== Puntos de SU variante: (x [m], y [m], θ [rad]) =====

PUNTOS = [
    (1.0, 1.0, math.pi / 4),
    (2.0, 0.0, 0.0),
    (1.0, -1.0, -math.pi / 4),
    (0.0, 0.0, 0.0),
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
    x, y, phi = pose
    xd, yd, thd = meta

    # 1: errores en odom y error angular envuelto a [-π, π]
    ex = xd - x
    ey = yd - y
    eth = envolver(thd - phi)

    # 2: ¿alcanzado?
    if math.hypot(ex, ey) < TOL_POS and abs(eth) < TOL_ANG:
        return 0.0, 0.0, 0.0, True

    # 3: ξ = J(φ)⁺ Λ e, con J(φ)⁺ = J(φ)ᵀ
    c, s = math.cos(phi), math.sin(phi)
    ax, ay, ath = L1 * ex, L2 * ey, L3 * eth
    u = c * ax + s * ay
    v = -s * ax + c * ay
    r = ath

    # 4: saturar la norma de (u, v) conservando la dirección, y |r|
    norma = math.hypot(u, v)
    if norma > V_MAX:
        u *= V_MAX / norma
        v *= V_MAX / norma
    r = max(-R_MAX, min(R_MAX, r))

    return u, v, r, False


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
                if self.i < len(self.puntos):
                    self.get_logger().info(f'Siguiente punto: {self.puntos[self.i]}')
                else:
                    self.get_logger().info('Recorrido completo.')
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