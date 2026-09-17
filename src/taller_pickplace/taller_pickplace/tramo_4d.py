#!/usr/bin/env python3
"""
Tramo 4D unicamente: pre-place -> place
"""

import rclpy
from rclpy.node import Node
from .utils_movimiento import TramoCartesiano
import numpy as np


def main():
    rclpy.init()
    node = Node('nodo_tramo_4d')

    tramo = TramoCartesiano(node, link_name='tool0', group_name='ur_manipulator')
    tramo.esperar()

    # ---------------- DATOS REALES ----------------
    place_pos = np.array([-0.535, -0.112, 0.075])
    preplace_pos = np.array([-0.535, -0.112, 0.125])
    quat_place = [0.707, 0.707, 0.000, 0.000]  # x, y, z, w

    v_max_retorno, a_max_retorno = 0.100, 0.020  # tramo azul
    # ------------------------------------------------

    tramo.ejecutar_tramo(
        nombre='4D',
        base_alta_pos=preplace_pos,
        base_baja_pos=place_pos,
        quat=quat_place,
        v_max=v_max_retorno,
        a_max=a_max_retorno,
        perfil='cubico',   # debe ser el mismo perfil ganador que usaste en 4B
    )

    rclpy.shutdown()


if __name__ == '__main__':
    main()
