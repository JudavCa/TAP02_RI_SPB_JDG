#!/usr/bin/env python3
"""
Corre el tramo 4B y 4D con tu perfil quintico, y SOLO imprime los valores
de q (posicion articular) y qdot (velocidad articular) que MoveIt calculo
en cada punto -- listos para copiar a MATLAB. No hace ninguna
multiplicacion aqui; eso lo haces tu con tu propio Jacobiano en MATLAB.

El Jacobiano usado internamente para calcular qdot viene de /compute_fk
(cinemática directa real de MoveIt), NO de ninguna tabla DH -- ver
jacobiano_moveit.py para el detalle.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from builtin_interfaces.msg import Duration as DurationMsg

from taller_pickplace.utils_movimiento import (
    calcular_T, construir_waypoints, MovimientoLibre,
)
from taller_pickplace.jacobiano_moveit import jacobiano_moveit_numerico

JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]


def interp_quintica_completa(p0, p1, v0, v1, a0, a1, T, n=4):
    t = np.linspace(0, T, n)
    M = np.array([
        [1, 0, 0, 0, 0, 0], [0, 1, 0, 0, 0, 0], [0, 0, 2, 0, 0, 0],
        [1, T, T**2, T**3, T**4, T**5], [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],
        [0, 0, 2, 6*T, 12*T**2, 20*T**3],
    ])
    c = np.linalg.solve(M, [p0, v0, a0, p1, v1, a1])
    pos = sum(c[i] * t**i for i in range(6))
    vel = sum(i * c[i] * t**(i - 1) for i in range(1, 6))
    return t, pos, vel


def reparametrizar(node, cli_fk, joint_trajectory, t_array, vel_escalar_array, direccion):
    """
    Sobreescribe tiempo y velocidad de cada punto para que respete el
    perfil quintico, resolviendo el sistema completo J*qdot = [v;0;0;0]
    (para que la orientacion se mantenga fija), usando el Jacobiano REAL
    obtenido de /compute_fk (no de ninguna tabla DH).
    """
    puntos = joint_trajectory.points
    n = min(len(puntos), len(t_array))
    for i in range(n):
        q = list(puntos[i].positions)
        J = jacobiano_moveit_numerico(node, cli_fk, q)

        v_deseada = vel_escalar_array[i] * direccion
        V_deseada = np.concatenate([v_deseada, [0.0, 0.0, 0.0]])

        try:
            qdot = np.linalg.solve(J, V_deseada)
        except np.linalg.LinAlgError:
            qdot = np.linalg.pinv(J) @ V_deseada

        puntos[i].velocities = [float(x) for x in qdot]
        t_i = float(t_array[i])
        puntos[i].time_from_start = DurationMsg(sec=int(t_i), nanosec=int((t_i - int(t_i)) * 1e9))
    return joint_trajectory


def imprimir_tabla_matlab(nombre, joint_trajectory):
    print(f'\n========== {nombre}: puntos de MoveIt (q y qdot) ==========')
    for i, punto in enumerate(joint_trajectory.points):
        t = punto.time_from_start.sec + punto.time_from_start.nanosec * 1e-9
        q = np.round(list(punto.positions), 6).tolist()
        qdot = np.round(list(punto.velocities), 6).tolist() if punto.velocities else None
        print(f'\n%% Punto {i+1}  (t = {t:.4f} s)')
        print(f'q_{nombre}_{i+1}    = {q};')
        print(f'qdot_{nombre}_{i+1} = {qdot};')


def planear_tramo(node, cli_fk, base_alta_pos, base_baja_pos, direccion, quat,
                   v_max, a_max, joint_th_inicio,
                   link_name='tool0', group_name='ur_manipulator'):
    libre = MovimientoLibre(node, group_name=group_name)
    libre.esperar()
    node.get_logger().info('Moviendo el robot a la posicion de arranque...')
    libre.mover_a_joint_goal(JOINT_NAMES, joint_th_inicio)

    offset = float(np.linalg.norm(base_baja_pos - base_alta_pos))
    T = calcular_T(offset, v_max, a_max)
    t_array, pos_q, vel_q = interp_quintica_completa(0, offset, 0, 0, 0, 0, T, n=4)
    waypoints = construir_waypoints(pos_q, base_alta_pos, direccion, quat)

    cli_cart = node.create_client(GetCartesianPath, '/compute_cartesian_path')
    while not cli_cart.wait_for_service(timeout_sec=2.0):
        node.get_logger().info('Esperando /compute_cartesian_path ...')

    req = GetCartesianPath.Request()
    req.header.frame_id = 'base_link'
    req.group_name = group_name
    req.link_name = link_name
    req.waypoints = waypoints
    req.max_step = 0.005
    req.jump_threshold = 0.0
    req.avoid_collisions = True

    future = cli_cart.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    resultado = future.result()
    node.get_logger().info(f'Fraccion completada: {resultado.fraction}  (T calculado = {T:.3f} s)')

    joint_traj = reparametrizar(node, cli_fk, resultado.solution.joint_trajectory,
                                 t_array, vel_q, direccion)
    return joint_traj


def main():
    rclpy.init()
    nodo = Node('velocidades_moveit')

    cli_fk = nodo.create_client(GetPositionFK, '/compute_fk')
    while not cli_fk.wait_for_service(timeout_sec=2.0):
        nodo.get_logger().info('Esperando /compute_fk ...')

    PICK_POS = np.array([-0.112, 0.506, 0.155])
    PREPICK_POS = np.array([-0.112, 0.506, 0.205])
    QUAT_PICK = [1.0, 0.0, 0.0, 0.0]
    PREPICK_TH = [1.570105, -0.552148, 0.518372, -1.537021, -1.570796, -0.000692]

    PLACE_POS = np.array([-0.535, -0.112, 0.075])
    PREPLACE_POS = np.array([-0.535, -0.112, 0.125])
    QUAT_PLACE = [0.707, 0.707, 0.0, 0.0]
    PREPLACE_TH = [3.140938, -0.245195, 0.264636, -1.590237, -1.570796, -0.000805]

    dir_4b = (PICK_POS - PREPICK_POS) / np.linalg.norm(PICK_POS - PREPICK_POS)
    traj_4b = planear_tramo(nodo, cli_fk, PREPICK_POS, PICK_POS, dir_4b, QUAT_PICK,
                             0.200, 0.300, PREPICK_TH)
    imprimir_tabla_matlab('4B', traj_4b)

    dir_4d = (PLACE_POS - PREPLACE_POS) / np.linalg.norm(PLACE_POS - PREPLACE_POS)
    traj_4d = planear_tramo(nodo, cli_fk, PREPLACE_POS, PLACE_POS, dir_4d, QUAT_PLACE,
                             0.100, 0.020, PREPLACE_TH)
    imprimir_tabla_matlab('4D', traj_4d)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
