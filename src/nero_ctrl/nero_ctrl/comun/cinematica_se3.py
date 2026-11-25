#!/usr/bin/env python3
"""
cinematica_se3.py — Núcleo compartido de cinemática inversa SE(3) para AgileX Nero
==================================================================================
Evolución modular del controlador en Álgebra de Lie de `codigo_oficial_grupo/tarea_Nero_new.py`:
  - Resuelve pasos incrementales en SE(3) sobre `pinch_site` usando logaritmo matricial (`logm`)
    y Jacobiano del cuerpo `Jb`.
  - Usa mínimos cuadrados amortiguados (DLS) en lugar de pseudoinversa pura para evitar
    picos de velocidad cerca de singularidades (fundamental para el brazo físico).
  - Incorpora sesgo suave en el espacio nulo (7 GDL -> 1 GDL redundante) hacia una postura
    nominal cómoda (`q_nominal`) para mantener buen condicionamiento en toda la mesa/cinta.
  - Incluye piso de seguridad en Z (`z_min_seguridad`) para pruebas seguras en laboratorio.
"""

import os
from pathlib import Path
from typing import Optional, Tuple

import mujoco
import numpy as np
import scipy.linalg
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory


ARM_JOINTS = [f"joint{i}" for i in range(1, 8)]

# Orientación vertical hacia abajo con dedos abriendo a lo largo del eje Y (mejor alcance en -X y Z alto)
# Columnas = ejes X, Y, Z de pinch_site en el marco mundo:
#   X_site = [0, 0, -1] (apunta hacia abajo)
#   Y_site = [1, 0,  0] (palas paralelas a X)
#   Z_site = [0, -1, 0] (eje de apertura de dedos a lo largo de Y)
R_PINZA_ABAJO_Y = np.column_stack((
    [0.0, 0.0, -1.0],
    [1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
))

# Orientación alternativa original de tarea_Nero_new.py (dedos abriendo a lo largo del eje X)
R_PINZA_ABAJO_X = np.column_stack((
    [0.0, 0.0, -1.0],
    [0.0, -1.0, 0.0],
    [-1.0, 0.0, 0.0],
))

# Postura nominal cómoda (codo arriba, zona de trabajo frontal -X)
Q_NOMINAL_NERO = np.array([0.0, -0.35, 0.0, 1.85, 0.0, 0.15, 0.0], dtype=float)


def smoothstep(p: float) -> float:
    """Interpolación suave C1 en [0, 1]: 3p^2 - 2p^3."""
    p = float(np.clip(p, 0.0, 1.0))
    return p * p * (3.0 - 2.0 * p)


def smootherstep(p: float) -> float:
    """Interpolación suave C2 (minimum-jerk) en [0, 1]: 6p^5 - 15p^4 + 10p^3."""
    p = float(np.clip(p, 0.0, 1.0))
    return p * p * p * (10.0 - 15.0 * p + 6.0 * p * p)


def transform_se3(R: np.ndarray, p: np.ndarray) -> np.ndarray:
    """Construye matriz homogénea 4x4 en SE(3)."""
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def vee_se3(V_mat: np.ndarray) -> np.ndarray:
    """Extrae twist 6D [wx, wy, wz, vx, vy, vz] desde una matriz en se(3)."""
    return np.array([
        V_mat[2, 1],
        V_mat[0, 2],
        V_mat[1, 0],
        V_mat[0, 3],
        V_mat[1, 3],
        V_mat[2, 3],
    ], dtype=float)


def resolve_scene_path(scene_name: str = "scene_cube.xml", explicit_path: str = "") -> str:
    """Localiza una escena XML dentro de nero_ctrl/models/escena_cinta_grupo/ o ruta explícita."""
    if explicit_path:
        p = Path(os.path.expanduser(explicit_path)).resolve()
        if p.is_file():
            return str(p)
        raise FileNotFoundError(f"No se encontró la escena explícita: {p}")

    env_path = os.environ.get("NERO_SCENE_XML", "")
    if env_path and not scene_name:
        p = Path(os.path.expanduser(env_path)).resolve()
        if p.is_file():
            return str(p)

    try:
        share_dir = Path(get_package_share_directory("nero_ctrl"))
        cand = share_dir / "models" / "escena_cinta_grupo" / scene_name
        if cand.is_file():
            return str(cand)
    except PackageNotFoundError:
        pass

    for parent in Path(__file__).resolve().parents:
        for rel in (
            Path("models/escena_cinta_grupo") / scene_name,
            Path("src/nero_ctrl/models/escena_cinta_grupo") / scene_name,
        ):
            cand = parent / rel
            if cand.is_file():
                return str(cand)

    raise FileNotFoundError(
        f"No se encontró la escena '{scene_name}' en nero_ctrl/models/escena_cinta_grupo/."
    )


class ControladorSE3:
    """Solucionador cinemático diferencial en SE(3) con DLS y sesgo en espacio nulo."""

    def __init__(
        self,
        model_path: str,
        site_name: str = "pinch_site",
        damping: float = 1.5e-2,
        alpha: float = 0.35,
        max_step_norm: float = 0.12,
        nullspace_gain: float = 0.04,
        z_min_seguridad: float = 0.072,
    ):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)

        self.ee_site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        if self.ee_site_id < 0:
            raise RuntimeError(f"No se encontró el site '{site_name}' en {model_path}")

        self.arm_joint_ids = np.array(
            [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n) for n in ARM_JOINTS],
            dtype=int,
        )
        if np.any(self.arm_joint_ids < 0):
            raise RuntimeError("Faltan articulaciones joint1..joint7 en el modelo MuJoCo.")

        self.arm_qpos_adr = self.model.jnt_qposadr[self.arm_joint_ids].copy()
        self.arm_dof_adr = self.model.jnt_dofadr[self.arm_joint_ids].copy()
        self.arm_ranges = self.model.jnt_range[self.arm_joint_ids].copy()

        self.jacp = np.zeros((3, self.model.nv), dtype=float)
        self.jacr = np.zeros((3, self.model.nv), dtype=float)

        self.damping = float(damping)
        self.alpha = float(alpha)
        self.max_step_norm = float(max_step_norm)
        self.nullspace_gain = float(nullspace_gain)
        self.z_min_seguridad = float(z_min_seguridad)
        self.q_nominal = Q_NOMINAL_NERO.copy()

    def fk(self, q: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Retorna (posicion_3d, matriz_rotacion_3x3) de pinch_site para los ángulos q."""
        self.data.qpos[:] = self.model.qpos0
        self.data.qpos[self.arm_qpos_adr] = q
        mujoco.mj_forward(self.model, self.data)
        pos = self.data.site_xpos[self.ee_site_id].copy()
        rot = self.data.site_xmat[self.ee_site_id].reshape(3, 3).copy()
        return pos, rot

    def step_ik(
        self,
        q_current: np.ndarray,
        theta_cmd: np.ndarray,
        pos_des: np.ndarray,
        rot_des: np.ndarray,
        alpha_override: Optional[float] = None,
        max_step_override: Optional[float] = None,
    ) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        Resuelve la cinemática inversa SE(3) de forma convergente a partir del último
        comando suave `theta_cmd` (evitando el lazo integrador sobre el estado atrasado
        `q_current` que genera rebotes y oscilaciones) y avanza suavemente hacia la
        solución exacta con límite de velocidad por ciclo.
        """
        pos_safe = np.asarray(pos_des, dtype=float).copy()
        pos_safe[2] = max(pos_safe[2], self.z_min_seguridad)

        # Posición real actual del efector (solo para telemetría y lógica de estados)
        pos_cur, _ = self.fk(q_current)

        # Semilla en el último comando suave (sin retroalimentar retardo del actuador)
        q_sol = np.asarray(theta_cmd, dtype=float).copy()
        T_sd = transform_se3(rot_des, pos_safe)
        lam2 = max(self.damping, 2.0e-2) ** 2

        for _ in range(18):
            p_k, R_k = self.fk(q_sol)
            T_sb = transform_se3(R_k, p_k)
            T_bd = np.linalg.solve(T_sb, T_sd)
            Vb = vee_se3(scipy.linalg.logm(T_bd).real)

            err_p_k = float(np.linalg.norm(pos_safe - p_k))
            err_r_k = float(np.linalg.norm(Vb[:3]))
            if err_p_k < 5e-4 and err_r_k < 5e-3:
                break

            mujoco.mj_jacSite(self.model, self.data, self.jacp, self.jacr, self.ee_site_id)
            J_wb = R_k.T @ self.jacr[:, self.arm_dof_adr]
            J_vb = R_k.T @ self.jacp[:, self.arm_dof_adr]
            Jb = np.vstack((J_wb, J_vb))

            Jb_pinv = Jb.T @ np.linalg.inv(Jb @ Jb.T + lam2 * np.eye(6))
            dq = Jb_pinv @ Vb

            if self.nullspace_gain > 0.0:
                null_proj = np.eye(7) - Jb_pinv @ Jb
                dq = dq + null_proj @ (0.5 * self.nullspace_gain * (self.q_nominal - q_sol))

            dq_norm = float(np.linalg.norm(dq))
            if dq_norm > 0.20:
                dq *= 0.20 / dq_norm

            q_sol = np.clip(q_sol + 0.65 * dq, self.arm_ranges[:, 0], self.arm_ranges[:, 1])

        # Paso suave en espacio articular hacia la solución exacta q_sol (cero sobreimpulso)
        delta_cmd = q_sol - theta_cmd
        max_step = self.max_step_norm if max_step_override is None else float(max_step_override)
        step_norm = float(np.linalg.norm(delta_cmd))
        if step_norm > max_step:
            delta_cmd *= max_step / step_norm

        alpha = 1.0 if alpha_override is None else float(np.clip(alpha_override, 0.05, 1.0))
        new_theta = np.clip(theta_cmd + alpha * delta_cmd, self.arm_ranges[:, 0], self.arm_ranges[:, 1])

        # Seguridad anti-deriva extrema si el robot físico fue detenido por E-stop o choque
        lead = new_theta - q_current
        lead_norm = float(np.linalg.norm(lead))
        if lead_norm > 0.45:
            new_theta = q_current + lead * (0.45 / lead_norm)

        err_pos = float(np.linalg.norm(pos_safe - pos_cur))
        return new_theta, pos_cur, err_pos
