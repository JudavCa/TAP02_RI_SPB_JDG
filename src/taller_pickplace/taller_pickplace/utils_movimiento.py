#!/usr/bin/env python3
"""
Modulo compartido para el ciclo pick-and-place.
Contiene:
  - Funciones de interpolacion cubica y quintica
  - Clase TramoCartesiano: para los acercamientos finos 4B y 4D
  - Clase MovimientoLibre: para los traslados con evasion de obstaculo 4A y 4C
"""

import rclpy
from rclpy.action import ActionClient
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions
from geometry_msgs.msg import Pose
import numpy as np


# ---------------------------------------------------------------------
# Interpolacion
# ---------------------------------------------------------------------

def interp_cubica(p0, p1, v0, v1, T, n=4):
    t = np.linspace(0, T, n)
    a2 = (3 * (p1 - p0) - (2 * v0 + v1) * T) / T**2
    a3 = (2 * (p0 - p1) + (v0 + v1) * T) / T**3
    pos = p0 + v0 * t + a2 * t**2 + a3 * t**3
    acc = 2 * a2 + 6 * a3 * t
    return pos, acc


def interp_quintica(p0, p1, v0, v1, a0, a1, T, n=4):
    t = np.linspace(0, T, n)
    M = np.array([
        [1, 0, 0, 0, 0, 0],
        [0, 1, 0, 0, 0, 0],
        [0, 0, 2, 0, 0, 0],
        [1, T, T**2, T**3, T**4, T**5],
        [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],
        [0, 0, 2, 6*T, 12*T**2, 20*T**3],
    ])
    c = np.linalg.solve(M, [p0, v0, a0, p1, v1, a1])
    pos = sum(c[i] * t**i for i in range(6))
    acc = sum(i * (i - 1) * c[i] * t**(i - 2) for i in range(2, 6))
    return pos, acc


def calcular_T(offset, v_max, a_max):
    """Duracion del tramo respetando v_max y a_max (perfil trapezoidal/triangular)."""
    t_acc = v_max / a_max
    d_acc = 0.5 * a_max * t_acc**2
    if 2 * d_acc >= offset:
        t_acc = np.sqrt(offset / a_max)
        T = 2 * t_acc
    else:
        d_const = offset - 2 * d_acc
        t_const = d_const / v_max
        T = 2 * t_acc + t_const
    return T


def construir_waypoints(pos_escalares, base_pos, direccion, quat):
    waypoints = []
    for s in pos_escalares:
        punto = base_pos + s * direccion
        pose = Pose()
        pose.position.x = float(punto[0])
        pose.position.y = float(punto[1])
        pose.position.z = float(punto[2])
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        waypoints.append(pose)
    return waypoints


# ---------------------------------------------------------------------
# Acercamiento fino (4B / 4D): interpolacion + compute_cartesian_path
# ---------------------------------------------------------------------

class TramoCartesiano:
    def __init__(self, node, link_name='tool0', group_name='ur_manipulator'):
        self.node = node
        self.link_name = link_name
        self.group_name = group_name
        self.cli_cartesian = node.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.exec_client = ActionClient(node, ExecuteTrajectory, '/execute_trajectory')

    def esperar(self):
        while not self.cli_cartesian.wait_for_service(timeout_sec=2.0):
            self.node.get_logger().info('Esperando /compute_cartesian_path ...')
        self.exec_client.wait_for_server()

    def _calcular_camino(self, waypoints):
        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.group_name = self.group_name
        req.link_name = self.link_name
        req.waypoints = waypoints
        req.max_step = 0.005
        req.jump_threshold = 0.0
        req.avoid_collisions = True
        future = self.cli_cartesian.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()

    def _ejecutar(self, trayectoria):
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trayectoria
        future = self.exec_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error('Ejecucion rechazada.')
            return None
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        return result_future.result()

    def ejecutar_tramo(self, nombre, base_alta_pos, base_baja_pos, quat, v_max, a_max, perfil='quintico'):
        """
        base_alta_pos -> punto de inicio (pre-pick o pre-place)
        base_baja_pos -> punto final (pick o place)
        Devuelve True si se ejecuto correctamente.
        """
        offset = float(np.linalg.norm(base_baja_pos - base_alta_pos))
        direccion = (base_baja_pos - base_alta_pos) / offset
        T = calcular_T(offset, v_max, a_max)
        self.node.get_logger().info(f'--- {nombre}: offset={offset:.3f} m, T={T:.3f} s, perfil={perfil} ---')

        if perfil == 'cubico':
            pos, acc = interp_cubica(0, offset, 0, 0, T, n=4)
        else:
            pos, acc = interp_quintica(0, offset, 0, 0, 0, 0, T, n=4)

        print(f'[{nombre}] posiciones escalares:', pos)
        print(f'[{nombre}] aceleraciones:        ', acc)

        waypoints = construir_waypoints(pos, base_alta_pos, direccion, quat)
        resultado = self._calcular_camino(waypoints)
        print(f'[{nombre}] fraccion completada:', resultado.fraction)

        if resultado.fraction > 0.95:
            self._ejecutar(resultado.solution)
            return True
        else:
            self.node.get_logger().warn(f'[{nombre}] No se pudo planear el 100% del camino.')
            return False


# ---------------------------------------------------------------------
# Movimiento libre con evasion de obstaculo (4A / 4C): MoveGroup + OMPL
# Equivale a presionar "Plan and Execute" en RViz2, pero por codigo.
# ---------------------------------------------------------------------

class MovimientoLibre:
    def __init__(self, node, group_name='ur_manipulator'):
        self.node = node
        self.group_name = group_name
        self.client = ActionClient(node, MoveGroup, '/move_action')

    def esperar(self):
        self.client.wait_for_server()

    def mover_a_joint_goal(self, joint_names, joint_values, planner_id='RRTConnectkConfigDefault',
                            planning_time=5.0, tolerance=0.001):
        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.group_name = self.group_name
        req.planner_id = planner_id
        req.allowed_planning_time = planning_time
        req.num_planning_attempts = 5
        req.max_velocity_scaling_factor = 1.0
        req.max_acceleration_scaling_factor = 1.0

        constraints = Constraints()
        for name, value in zip(joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(value)
            jc.tolerance_above = tolerance
            jc.tolerance_below = tolerance
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)

        goal_msg.request = req
        goal_msg.planning_options = PlanningOptions()
        goal_msg.planning_options.plan_only = False  # plan + ejecuta en un solo paso

        future = self.client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, future)
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().error('Meta de movimiento libre rechazada por MoveIt.')
            return None
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        resultado = result_future.result()
        self.node.get_logger().info(f'Movimiento libre finalizado, codigo error: {resultado.result.error_code.val}')
        return resultado
