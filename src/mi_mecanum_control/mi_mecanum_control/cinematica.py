#!/usr/bin/env python3
"""
cinematica.py — Cinemática del robot Mecanum (Parte 4)

    ξ = [u, v, r]ᵀ                       velocidades en el marco del robot
    ω = [ω_di, ω_dd, ω_ti, ω_td]ᵀ        ruedas: delantera izq/der, trasera izq/der

    Directa : ξ = W ω     (W de 3x4, ver enunciado)
    Inversa : ω = W⁺ ξ

Lazo:
    /cmd_vel --W⁺--> /vel_rueda_* --> Gazebo (JointController) --> ruedas
    /joint_states --W--> ξ --η̇ = J(φ)ξ, Euler--> /odom + TF odom -> base_link

    ros2 run mi_mecanum_control cinematica --ros-args -p use_sim_time:=true
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster

# ===== TODO: parámetros físicos de SU variante (deben coincidir con el xacro) =====
A_RUEDA = None     # a  : rueda_radio
LX = None          # lx : distancia_ejes / 2
LY = None          # ly : separacion_ruedas / 2

RUEDAS = ['del_izq', 'del_der', 'tra_izq', 'tra_der']
JOINTS = [f'joint_rueda_{p}' for p in RUEDAS]
TOPICOS = [f'/vel_rueda_{p}' for p in RUEDAS]


def matriz_W(a, lx, ly):
    """Cinemática directa del Mecanum: ξ = W ω  (3x4)."""
    # TODO
    raise NotImplementedError


def envolver(a):
    """Envuelve un ángulo a [-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))


def integrar(x, y, phi, u, v, r, dt):
    """η̇ = J(φ) ξ e integración de Euler. Devuelve (x, y, φ) con φ en [-π, π]."""
    # TODO: ojo, u y v están en el marco del robot, no en odom
    raise NotImplementedError


class Cinematica(Node):
    def __init__(self):
        super().__init__('cinematica')
        self.W = matriz_W(A_RUEDA, LX, LY)
        self.W_pinv = None   # TODO: ω = W⁺ ξ

        # Inversa
        self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 10)
        self.pubs = [self.create_publisher(Float64, t, 10) for t in TOPICOS]

        # Directa + odometría
        self.create_subscription(JointState, '/joint_states', self.cb_joints, 10)
        self.pub_odom = self.create_publisher(Odometry, '/odom', 10)
        self.tf = TransformBroadcaster(self)

        self.x = self.y = self.phi = 0.0
        self.t_ant = None

        np.set_printoptions(precision=4, suppress=True)
        self.get_logger().info(f'\nW =\n{self.W}\nW⁺ =\n{self.W_pinv}')

    # ---------- Cinemática inversa: ω = W⁺ ξ ----------
    def cb_cmd_vel(self, msg):
        # TODO: armar ξ desde msg, calcular ω y publicar cada ω en su publisher (Float64)
        pass

    # ---------- Cinemática directa: ξ = W ω, y odometría ----------
    def cb_joints(self, msg):
        # ω en el orden de JOINTS
        try:
            omega = np.array([msg.velocity[msg.name.index(j)] for j in JOINTS])
        except (ValueError, IndexError):
            return

        ahora = self.get_clock().now()
        if self.t_ant is None:
            self.t_ant = ahora
            return
        dt = (ahora - self.t_ant).nanoseconds * 1e-9
        self.t_ant = ahora
        if dt <= 0.0:
            return

        # TODO: calcular u, v, r con W y actualizar self.x, self.y, self.phi
        u = v = r = 0.0

        qz, qw = math.sin(self.phi / 2.0), math.cos(self.phi / 2.0)
        stamp = ahora.to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        # TODO: llenar odom.pose.pose (posición y orientación) y odom.twist.twist (u, v, r)
        self.pub_odom.publish(odom)

        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    nodo = Cinematica()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
