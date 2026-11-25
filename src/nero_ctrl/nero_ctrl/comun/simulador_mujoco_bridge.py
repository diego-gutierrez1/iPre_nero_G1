#!/usr/bin/env python3
"""
simulador_mujoco_bridge.py — Puente ROS 2 <-> MuJoCo empaquetado para la Entrega Final
======================================================================================
Basado directamente en `codigo_oficial_grupo/mujoco_viewer.py`, con extensiones limpias
para soportar las nuevas escenas de la Entrega Final:
  - `--scene-name <archivo.xml>`: busca automáticamente en `nero_ctrl/models/escena_cinta_grupo/`.
  - `--init-ctrl NAME=VAL` (repetible): fija actuadores iniciales (ej. velocidad de cinta/cubos).
  - `--loop-cubes`: recicla automáticamente cubos deslizantes `cube*_joint` al llegar al final
    de la cinta para generar flujo continuo de obstáculos (Tareas 2.1 y 2.2).
  - `--headless`: corre sin ventana gráfica para pruebas automatizadas en CI/WSL.
"""

import argparse
import math
import os
import sys
import threading
import time

import mujoco
import mujoco.viewer
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, PoseStamped
from sensor_msgs.msg import JointState

from nero_ctrl.comun.cinematica_se3 import ARM_JOINTS, resolve_scene_path


FLANGE_BODY = "link7"
BASE_BODY = "base_link"


def mat_to_quat(R):
    t = np.trace(R)
    if t > 0.0:
        s = math.sqrt(t + 1.0) * 2.0
        q = [0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        q = [(R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s]
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s]
    q = np.array(q, dtype=float)
    return q / np.linalg.norm(q)


class MujocoBridgeFinal(Node):
    def __init__(
        self,
        scene_path: str,
        rate_hz: float = 200.0,
        joint_speed: float = 1.2,
        init_ctrls=None,
        loop_cubes: bool = False,
    ):
        super().__init__("mujoco_bridge")
        self.joint_speed = float(joint_speed)
        self.loop_cubes = bool(loop_cubes)

        self.get_logger().info(f"Cargando escena MuJoCo: {scene_path}")
        self.model = mujoco.MjModel.from_xml_path(scene_path)
        self.data = mujoco.MjData(self.model)

        scene_key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "scene_start")
        if scene_key_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, scene_key_id)
        else:
            # Reset limpio preservando qpos0 de freejoints/cubos; solo ponemos el brazo en home
            mujoco.mj_resetData(self.model, self.data)
            home_q = [0.0, -0.4, 0.0, 2.0, 0.0, 0.0, 0.0]
            for jname, qval in zip(ARM_JOINTS, home_q):
                jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, jname)
                if jid >= 0:
                    self.data.qpos[self.model.jnt_qposadr[jid]] = qval

        self._actuator_ids = {}
        for i in range(self.model.nu):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if name:
                self._actuator_ids[name] = i

        self._joint_info = {}
        self._loop_slide_joints = []
        for i in range(self.model.njnt):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if not name:
                continue
            jtype = int(self.model.jnt_type[i])
            # Solo juntas 1D (hinge/slide) se publican en JointState
            if jtype in (mujoco.mjtJoint.mjJNT_HINGE, mujoco.mjtJoint.mjJNT_SLIDE):
                info = {
                    "id": i,
                    "qpos_adr": int(self.model.jnt_qposadr[i]),
                    "qvel_adr": int(self.model.jnt_dofadr[i]),
                }
                self._joint_info[name] = info
                if (
                    self.loop_cubes
                    and jtype == mujoco.mjtJoint.mjJNT_SLIDE
                    and name.startswith("cube")
                    and name.endswith("_joint")
                    and "_x_" not in name
                    and "_z_" not in name
                    and self.model.jnt_limited[i]
                ):
                    lo, hi = float(self.model.jnt_range[i, 0]), float(self.model.jnt_range[i, 1])
                    self._loop_slide_joints.append((name, info["qpos_adr"], info["qvel_adr"], lo, hi))

        self._gripper_max = None
        g_id = self._actuator_ids.get("gripper")
        if g_id is not None:
            g_joint = int(self.model.actuator_trnid[g_id, 0])
            self._gripper_max = float(self.model.jnt_range[g_joint, 1])

        self._arm_aid = np.array([self._actuator_ids[n] for n in ARM_JOINTS], dtype=int)
        self._arm_qadr = np.array([self._joint_info[n]["qpos_adr"] for n in ARM_JOINTS], dtype=int)
        lo, hi = [], []
        for n in ARM_JOINTS:
            jid = self._joint_info[n]["id"]
            if self.model.jnt_limited[jid]:
                lo.append(float(self.model.jnt_range[jid, 0]))
                hi.append(float(self.model.jnt_range[jid, 1]))
            else:
                lo.append(-np.inf)
                hi.append(np.inf)
        self._q_lo, self._q_hi = np.array(lo), np.array(hi)

        self._flange_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, FLANGE_BODY)
        mujoco.mj_forward(self.model, self.data)
        base_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, BASE_BODY)
        if base_id >= 0:
            self._base_p = self.data.xpos[base_id].copy()
            self._base_R = self.data.xmat[base_id].reshape(3, 3).copy()
        else:
            self._base_p = np.zeros(3)
            self._base_R = np.eye(3)

        # Mantener pose inicial del brazo
        self.data.ctrl[self._arm_aid] = self.data.qpos[self._arm_qadr]

        # Aplicar comandos iniciales de actuadores (ej. cube_motor=0.03)
        if init_ctrls:
            for item in init_ctrls:
                if "=" not in item:
                    continue
                cname, cval = item.split("=", 1)
                cname = cname.strip()
                val = float(cval.strip())
                if cname in self._actuator_ids:
                    self.data.ctrl[self._actuator_ids[cname]] = val
                    self.get_logger().info(f"Actuador inicial: {cname} = {val}")

        self._jgoal = None
        self._data_lock = threading.Lock()

        self.create_subscription(JointState, "control/joint_states", self._on_control_msg, 10)
        self.create_subscription(JointState, "control/move_j", self._on_move_j, 10)

        self.feedback_pub = self.create_publisher(JointState, "feedback/joint_states", 10)
        self.tcp_pub = self.create_publisher(PoseStamped, "feedback/tcp_pose", 10)
        self.create_timer(1.0 / rate_hz, self._publish_feedback)

        self.get_logger().info(
            f"Bridge listo (actuadores={list(self._actuator_ids.keys())}, "
            f"loop_cubes={self.loop_cubes}, rate={rate_hz} Hz)."
        )

    def _set_gripper_locked(self, width: float):
        g_id = self._actuator_ids.get("gripper")
        if g_id is None or self._gripper_max is None:
            return
        self.data.ctrl[g_id] = min(max(width / 2.0, 0.0), self._gripper_max)

    def _on_control_msg(self, msg: JointState):
        with self._data_lock:
            for name, pos in zip(msg.name, msg.position):
                if name in ARM_JOINTS:
                    self._jgoal = None
                if name == "gripper":
                    self._set_gripper_locked(float(pos))
                    continue
                aid = self._actuator_ids.get(name)
                if aid is not None:
                    self.data.ctrl[aid] = float(pos)

    def _on_move_j(self, msg: JointState):
        with self._data_lock:
            base = self._jgoal if self._jgoal is not None else self.data.ctrl[self._arm_aid]
            q_goal = np.array(base, dtype=float)
            moved = False
            for name, pos in zip(msg.name, msg.position):
                if name in ARM_JOINTS:
                    k = ARM_JOINTS.index(name)
                    q_goal[k] = min(max(float(pos), self._q_lo[k]), self._q_hi[k])
                    moved = True
                elif name == "gripper":
                    self._set_gripper_locked(float(pos))
            if moved:
                self._jgoal = q_goal

    def _follow_joint_goal(self):
        if self._jgoal is None:
            return
        dt = float(self.model.opt.timestep)
        cur = self.data.ctrl[self._arm_aid].copy()
        err = self._jgoal - cur
        n = float(np.linalg.norm(err))
        if n < 1e-6:
            self.data.ctrl[self._arm_aid] = self._jgoal
            self._jgoal = None
            return
        step = min(n * (1.0 - math.exp(-dt / 0.08)), self.joint_speed * dt)
        self.data.ctrl[self._arm_aid] = cur + err / n * step

    def _wrap_loop_cubes(self):
        if not self._loop_slide_joints:
            return
        for _, qadr, vadr, lo, hi in self._loop_slide_joints:
            val = float(self.data.qpos[qadr])
            if val >= hi - 0.015:
                self.data.qpos[qadr] = lo + 0.015
                self.data.qvel[vadr] = 0.0
            elif val <= lo + 0.005 and float(self.data.qvel[vadr]) < -1e-4:
                self.data.qpos[qadr] = hi - 0.015
                self.data.qvel[vadr] = 0.0

    def _publish_feedback(self):
        stamp = self.get_clock().now().to_msg()
        js = JointState()
        js.header.stamp = stamp
        with self._data_lock:
            def add(name, info):
                js.name.append(name)
                js.position.append(float(self.data.qpos[info["qpos_adr"]]))
                js.velocity.append(float(self.data.qvel[info["qvel_adr"]]))

            for name in ARM_JOINTS:
                add(name, self._joint_info[name])
            g = self._joint_info.get("gripper_joint1")
            if g is not None:
                js.name.append("gripper")
                js.position.append(2.0 * float(self.data.qpos[g["qpos_adr"]]))
                js.velocity.append(0.0)
            for name, info in self._joint_info.items():
                if name not in ARM_JOINTS:
                    add(name, info)

            p_w = self.data.xpos[self._flange_id].copy()
            R_w = self.data.xmat[self._flange_id].reshape(3, 3).copy()

        self.feedback_pub.publish(js)

        p_b = self._base_R.T @ (p_w - self._base_p)
        q_b = mat_to_quat(self._base_R.T @ R_w)
        ps = PoseStamped()
        ps.header.stamp = stamp
        ps.header.frame_id = BASE_BODY
        ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, p_b)
        ps.pose.orientation.w = float(q_b[0])
        ps.pose.orientation.x = float(q_b[1])
        ps.pose.orientation.y = float(q_b[2])
        ps.pose.orientation.z = float(q_b[3])
        self.tcp_pub.publish(ps)

    def step(self):
        with self._data_lock:
            self._follow_joint_goal()
            mujoco.mj_step(self.model, self.data)
            self._wrap_loop_cubes()


def main(args=None):
    # Separar argumentos ROS (--ros-args ...) antes de argparse
    raw_argv = sys.argv[1:] if args is None else list(args)
    clean_argv = []
    skip = False
    for tok in raw_argv:
        if tok == "--ros-args":
            break
        if skip:
            skip = False
            continue
        clean_argv.append(tok)

    parser = argparse.ArgumentParser(description="Puente ROS 2 <-> MuJoCo para Entrega Final Nero")
    parser.add_argument("--scene-name", default="scene_cube.xml", help="Nombre del XML en escena_cinta_grupo/")
    parser.add_argument("--scene-path", default="", help="Ruta absoluta opcional a la escena XML")
    parser.add_argument("--rate", type=float, default=200.0)
    parser.add_argument("--joint-speed", type=float, default=1.2)
    parser.add_argument("--init-ctrl", action="append", default=[], help="Actuador inicial NAME=VAL (repetible)")
    parser.add_argument("--loop-cubes", action="store_true", help="Recicla cubos al final de la cinta")
    parser.add_argument("--headless", action="store_true", help="Corre sin ventana gráfica")
    parsed = parser.parse_args(clean_argv)

    scene_file = resolve_scene_path(parsed.scene_name, parsed.scene_path)

    rclpy.init(args=sys.argv if args is None else args)
    bridge = MujocoBridgeFinal(
        scene_path=scene_file,
        rate_hz=parsed.rate,
        joint_speed=parsed.joint_speed,
        init_ctrls=parsed.init_ctrl,
        loop_cubes=parsed.loop_cubes,
    )

    spin_thread = threading.Thread(target=rclpy.spin, args=(bridge,), daemon=True)
    spin_thread.start()

    try:
        if parsed.headless:
            sim_ref = bridge.data.time
            wall_ref = time.perf_counter()
            while rclpy.ok():
                bridge.step()
                ahead = (bridge.data.time - sim_ref) - (time.perf_counter() - wall_ref)
                if ahead > 0:
                    time.sleep(ahead)
                elif ahead < -0.5:
                    sim_ref = bridge.data.time
                    wall_ref = time.perf_counter()
        else:
            with mujoco.viewer.launch_passive(bridge.model, bridge.data) as viewer:
                sim_ref = bridge.data.time
                wall_ref = time.perf_counter()
                last_sync = wall_ref
                while viewer.is_running() and rclpy.ok():
                    bridge.step()
                    now = time.perf_counter()
                    if now - last_sync >= 1.0 / 60.0:
                        viewer.sync()
                        last_sync = now
                    ahead = (bridge.data.time - sim_ref) - (time.perf_counter() - wall_ref)
                    if ahead > 0:
                        time.sleep(ahead)
                    elif ahead < -0.5:
                        sim_ref = bridge.data.time
                        wall_ref = time.perf_counter()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
