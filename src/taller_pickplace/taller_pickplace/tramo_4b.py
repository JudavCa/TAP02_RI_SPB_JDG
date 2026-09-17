#!/usr/bin/env python3
"""
Tramo 4B unicamente: pre-pick -> pick
"""

import rclpy
from rclpy.node import Node
from .utils_movimiento import TramoCartesiano
import numpy as np


def main():
    rclpy.init()
    node = Node('nodo_tramo_4b')

    tramo = TramoCartesiano(node, link_name='tool0', group_name='ur_manipulator')
    tramo.esperar()

    # ---------------- DATOS REALES ----------------
    pick_pos = np.array([-0.112, 0.506, 0.155])
    prepick_pos = np.array([-0.112, 0.506, 0.205])
    quat_pick = [1.000, 0.000, 0.000, 0.000]  # x, y, z, w

    v_max_ida, a_max_ida = 0.200, 0.300  # tramo rojo
    # ------------------------------------------------

    tramo.ejecutar_tramo(
        nombre='4B',
        base_alta_pos=prepick_pos,
        base_baja_pos=pick_pos,
        quat=quat_pick,
        v_max=v_max_ida,
        a_max=a_max_ida,
        perfil='quintico',   # cambia a 'cubico' si quieres probar el otro
    )

    rclpy.shutdown()


if __name__ == '__main__':
    main()
