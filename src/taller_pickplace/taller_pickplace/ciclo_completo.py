#!/usr/bin/env python3
"""
Ciclo completo de pick-and-place:
  4A: HOME -> pre-pick        (movimiento libre, evade obstaculo, OMPL)
  4B: pre-pick -> pick        (acercamiento fino, interpolacion)
  4C: pick -> pre-place       (movimiento libre, evade obstaculo, OMPL)
  4D: pre-place -> place      (acercamiento fino, interpolacion)

IMPORTANTE: antes de correr este nodo, el robot debe estar en HOME
(o el nodo debe arrancar desde donde este el robot en ese momento,
ya que MoveIt siempre planea desde el estado actual).
"""

import rclpy
from rclpy.node import Node
from .utils_movimiento import TramoCartesiano, MovimientoLibre
import numpy as np


# Nombres de las articulaciones del UR3, en el orden que usa MoveIt/ur_manipulator
JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint',
]

# ---------------- DATOS REALES (joint values, en radianes) ----------------

# EDITA ESTO: valores articulares de tu pose HOME (los defines en el
# Setup Assistant como "home"; consultalos en tu SRDF o en RViz -> Joints)
HOME_TH = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # <-- reemplaza por los reales

PREPICK_TH = [1.570105, -0.552148, 0.518372, -1.537021, -1.570796, -0.000692]
PICK_TH    = [1.570800, -0.523599, 0.698132, -1.745300, -1.570800, 0.000000]

PREPLACE_TH = [3.140938, -0.245195, 0.264636, -1.590237, -1.570796, -0.000805]
PLACE_TH    = [3.141590, -0.174533, 0.349066, -1.745300, -1.570800, 0.000000]

# Poses cartesianas para los tramos finos (4B / 4D)
PICK_POS     = np.array([-0.112, 0.506, 0.155])
PREPICK_POS  = np.array([-0.112, 0.506, 0.205])
QUAT_PICK    = [1.000, 0.000, 0.000, 0.000]

PLACE_POS    = np.array([-0.535, -0.112, 0.075])
PREPLACE_POS = np.array([-0.535, -0.112, 0.125])
QUAT_PLACE   = [0.707, 0.707, 0.000, 0.000]

# Restricciones de velocidad/aceleracion de la consultora
V_MAX_IDA, A_MAX_IDA = 0.200, 0.300         # tramo rojo (4B)
V_MAX_RETORNO, A_MAX_RETORNO = 0.100, 0.020  # tramo azul (4D)

# Planeador OMPL ganador de tu comparacion 4A/4C (ajusta segun tu resultado)
PLANNER_ID = 'RRTConnectkConfigDefault'

# Perfil de interpolacion ganador (ajusta segun tu comparacion 4B/4D)
PERFIL_GANADOR = 'quintico'

# ----------------------------------------------------------------------


def main():
    rclpy.init()
    node = Node('ciclo_completo_pick_place')

    libre = MovimientoLibre(node, group_name='ur_manipulator')
    libre.esperar()

    fino = TramoCartesiano(node, link_name='tool0', group_name='ur_manipulator')
    fino.esperar()

    # --- 4A: HOME -> pre-pick (movimiento libre, evade obstaculo) ---
    node.get_logger().info('=== 4A: HOME -> pre-pick ===')
    libre.mover_a_joint_goal(JOINT_NAMES, PREPICK_TH, planner_id=PLANNER_ID)

    # --- 4B: pre-pick -> pick (acercamiento fino) ---
    node.get_logger().info('=== 4B: pre-pick -> pick ===')
    fino.ejecutar_tramo(
        nombre='4B',
        base_alta_pos=PREPICK_POS,
        base_baja_pos=PICK_POS,
        quat=QUAT_PICK,
        v_max=V_MAX_IDA,
        a_max=A_MAX_IDA,
        perfil=PERFIL_GANADOR,
    )

    # Aqui, en un ciclo real, cerrarias el gripper para tomar la pieza.
    # Si tu robot/gripper tiene su propio grupo MoveIt, agregarias esa
    # llamada aqui (fuera del alcance de este taller si no lo pide el enunciado).

    # --- 4C: pick -> pre-place (movimiento libre, evade obstaculo) ---
    node.get_logger().info('=== 4C: pick -> pre-place ===')
    libre.mover_a_joint_goal(JOINT_NAMES, PREPLACE_TH, planner_id=PLANNER_ID)

    # --- 4D: pre-place -> place (acercamiento fino) ---
    node.get_logger().info('=== 4D: pre-place -> place ===')
    fino.ejecutar_tramo(
        nombre='4D',
        base_alta_pos=PREPLACE_POS,
        base_baja_pos=PLACE_POS,
        quat=QUAT_PLACE,
        v_max=V_MAX_RETORNO,
        a_max=A_MAX_RETORNO,
        perfil=PERFIL_GANADOR,
    )

    node.get_logger().info('=== Ciclo completo finalizado ===')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
