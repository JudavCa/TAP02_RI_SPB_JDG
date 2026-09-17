#!/usr/bin/env python3
"""
Calcula el Jacobiano usando UNICAMENTE el servicio /compute_fk de MoveIt2
(cinemática directa real, con el URDF cargado) -- NO usa tu tabla DH para
nada. Perturba cada articulacion un poco, le pregunta a MoveIt donde queda
el efector, y arma el Jacobiano por diferencias finitas con esas respuestas.

Esto es una fuente verdaderamente independiente para comparar contra tu
Jacobiano de MATLAB.
"""

import numpy as np
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionFK
from sensor_msgs.msg import JointState


JOINT_NAMES = [
    'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
    'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint',
]

LINK_NAME = 'tool0'
FRAME_ID = 'base_link'


def obtener_pose_fk(node, cli, q):
    """Le pregunta a MoveIt (via /compute_fk) donde queda el efector para el q dado."""
    req = GetPositionFK.Request()
    req.header.frame_id = FRAME_ID
    req.fk_link_names = [LINK_NAME]
    req.robot_state.joint_state = JointState(name=JOINT_NAMES, position=list(q))

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    res = future.result()

    if res.error_code.val != 1:  # 1 = SUCCESS
        raise RuntimeError(f'compute_fk fallo, error_code={res.error_code.val}. '
                            f'Revisa que LINK_NAME="{LINK_NAME}" exista en tu robot.')
    return res.pose_stamped[0].pose


def quaternion_a_matriz(qx, qy, qz, qw):
    return np.array([
        [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [    2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz),     2*(qy*qz - qx*qw)],
        [    2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy)],
    ])


def pose_a_posicion_rotacion(pose):
    p = np.array([pose.position.x, pose.position.y, pose.position.z])
    R = quaternion_a_matriz(pose.orientation.x, pose.orientation.y,
                             pose.orientation.z, pose.orientation.w)
    return p, R


def jacobiano_moveit_numerico(node, cli, q, eps=1e-6):
    """
    Jacobiano 6xN por diferencias finitas, usando SOLO respuestas de
    /compute_fk (MoveIt real, no tu DH).
    """
    n = len(q)
    p0, R0 = pose_a_posicion_rotacion(obtener_pose_fk(node, cli, q))

    J = np.zeros((6, n))
    for i in range(n):
        q_pert = list(q)
        q_pert[i] += eps
        p1, R1 = pose_a_posicion_rotacion(obtener_pose_fk(node, cli, q_pert))

        J[0:3, i] = (p1 - p0) / eps

        dR = R1 @ R0.T
        w_skew = (dR - np.eye(3)) / eps
        J[3, i] = w_skew[2, 1]
        J[4, i] = w_skew[0, 2]
        J[5, i] = w_skew[1, 0]

    return J


def imprimir_como_matlab(J, nombre='J_moveit'):
    print(f'\n{nombre} = [')
    for fila in J:
        print('    ' + ' '.join(f'{v: .6f}' for v in fila) + ';')
    print('];')


CONFIGURACIONES = {
    'pre-pick':  [1.570105, -0.552148, 0.518372, -1.537021, -1.570796, -0.000692],
    'pick':      [1.570800, -0.523599, 0.698132, -1.745300, -1.570800, 0.000000],
    'pre-place': [3.140938, -0.245195, 0.264636, -1.590237, -1.570796, -0.000805],
    'place':     [3.141590, -0.174533, 0.349066, -1.745300, -1.570800, 0.000000],
}


def main():
    rclpy.init()
    nodo = Node('jacobiano_moveit_real')

    cli = nodo.create_client(GetPositionFK, '/compute_fk')
    while not cli.wait_for_service(timeout_sec=2.0):
        nodo.get_logger().info('Esperando /compute_fk ...')

    for nombre, q in CONFIGURACIONES.items():
        J = jacobiano_moveit_numerico(nodo, cli, q)
        print(f'\n========== {nombre} (desde /compute_fk de MoveIt) ==========')
        print('q =', np.round(q, 6).tolist())
        imprimir_como_matlab(J, nombre=f'J_moveit_{nombre.replace("-", "_")}')

    rclpy.shutdown()


if __name__ == '__main__':
    main()
