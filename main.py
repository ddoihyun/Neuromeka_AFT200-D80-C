
# ***********************************************************************
#
# 6-axis F/T damping-only admittance controller for Nuri Robot teleoperation.
#
# Control idea:
#   1. Bias-compensated force/torque is converted directly to
#      command velocity through damping:
#
#          D * x_dot = F
#          x_dot = F / D
#
#   2. The command velocity is integrated into a 6D relative task pose.
#   3. MoveTeleL receives the accumulated 6D relative task pose:
#      [x, y, z, Rx, Ry, Rz].
#
# Notes:
#   - Translation units are mm.
#   - Rotation units are assumed to be deg for Nuri Robot task poses.
#   - The real TCP pose is approximated by the last commanded relative pose.
#     If robot feedback pose is needed later, compare command_pose with the
#     measured TCP-relative pose for safety monitoring.
#
# ***********************************************************************

from __future__ import annotations

import json
import logging
import math
import sys
import threading
import time
from datetime import datetime
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Sequence

import can
import numpy as np
from neuromeka import IndyDCP3

try:
    from neuromeka.proto_step import control_msgs_pb2 as control_msgs
except ModuleNotFoundError:
    from neuromeka.proto import control_msgs_pb2 as control_msgs


log = logging.getLogger('DampingAdmittance')

# 특이점 지표 계산에 사용하는 MDH 파라미터와 임시 TCP offset이다.
MDH_A_M = np.array([0.0, 0.0, 0.4031, -0.05, 0.0, 0.0], dtype=float)
MDH_ALPHA_RAD = np.array([0.0, math.pi / 2.0, 0.0, math.pi / 2.0, math.pi / 2.0, math.pi / 2.0], dtype=float)
MDH_D_M = np.array([0.3280, 0.0, 0.0, 0.40, 0.136, 0.1035], dtype=float)
MDH_THETA0_RAD = np.array([math.pi, 1.6952, 1.4464, math.pi, math.pi, 0.0], dtype=float)
TCP_OFFSET_M = np.array([0.0, 0.0, 0.0], dtype=float)
SINGULARITY_EPS = 1e-9

# Jacobian 지표 기반 damping 보호의 기본 기준값이다.
DEFAULT_COND_SLOW = 1.0e4
DEFAULT_COND_STRONG = 1.0e5
DEFAULT_COND_STOP = 1.0e6
DEFAULT_SIGMA_SLOW = 1.0e-3
DEFAULT_SIGMA_STRONG = 1.0e-4
DEFAULT_SIGMA_STOP = 1.0e-5
DEFAULT_DAMPING_FORCE_MIN = 0.4
DEFAULT_DAMPING_FORCE_MAX = 0.6
DEFAULT_DAMPING_TORQUE_MIN = 0.7
DEFAULT_DAMPING_TORQUE_MAX = 1.0
PROTECTION_UPDATE_INTERVAL_LOOPS = 10
# Jacobian/SVD 기반 특이점 로그는 CPU 부담을 줄이기 위해 더 낮은 주기로 계산한다.
SINGULARITY_LOG_INTERVAL_LOOPS = 50


def axis_names():
    return ('X(tool)', 'Y(tool)', 'Z(tool)', 'Rx(tool)', 'Ry(tool)', 'Rz(tool)')


def axis_units():
    return ('mm', 'mm', 'mm', 'deg', 'deg', 'deg')


def default_config_path():
    return Path(__file__).resolve().parent / 'config.json'


# ===================================================================
# Configuration
# ===================================================================

@dataclass(frozen=True)
class CanConfig:
    interface: str
    channel: str
    bitrate: int
    force_id: int
    torque_id: int


@dataclass(frozen=True)
class RobotConfig:
    ip: str
    index: int
    apply_robot_commands: bool
    teleop_start_mode: int
    vel_ratio: float
    acc_ratio: float


@dataclass(frozen=True)
class OpStateConfig:
    allowed_states: frozenset
    state_names: dict   # {int: str}

    def name(self, code: int) -> str:
        return self.state_names.get(code, f'UNDEFINED({code})')

    def is_allowed(self, code: int) -> bool:
        return code in self.allowed_states


@dataclass(frozen=True)
class LogConfig:
    log_level: str
    log_to_file: bool
    log_output_dir: str


@dataclass(frozen=True)
class CommonControlConfig:
    period_sec: float
    bias_sample_count: int
    stale_sensor_timeout_sec: float
    force_threshold: float
    force_release_threshold: float
    release_hold_sec: float


@dataclass(frozen=True)
class ProtectionConfig:
    enabled: bool
    update_interval_loops: int
    singularity_log_interval_loops: int
    damping_force_min: float
    damping_force_max: float
    damping_torque_min: float
    damping_torque_max: float
    cond_slow: float
    cond_strong: float
    cond_stop: float
    sigma_slow: float
    sigma_strong: float
    sigma_stop: float


@dataclass(frozen=True)
class ControlConfig:
    common: CommonControlConfig
    damping_force: float
    damping_torque: float
    # config.json에서 Jacobian 기반 damping 보호 판단을 제어한다.
    protection: ProtectionConfig


@dataclass(frozen=True)
class AppConfig:
    can: CanConfig
    robot: RobotConfig
    log: LogConfig
    control: ControlConfig
    op_state: OpStateConfig


def load_config(config_path: Path) -> AppConfig:
    with config_path.open('r', encoding='utf-8') as f:
        data = json.load(f)

    can_cfg = data['can']
    robot_cfg = data['robot']
    log_cfg = data['logging']
    control_cfg = data['control']
    common_cfg = control_cfg['common']
    damping_cfg = control_cfg['damping']
    # protection 항목이 없으면 기존처럼 보호 판단을 켠 상태로 동작한다.
    protection_cfg = control_cfg.get('protection', {})
    # enabled 값은 JSON boolean true/false만 허용해 문자열 오입력을 막는다.
    protection_enabled = protection_cfg.get('enabled', True)
    if not isinstance(protection_enabled, bool):
        raise ValueError('control.protection.enabled must be boolean')
    # damping 보호 범위와 Jacobian 임계값은 config.json에서 조정 가능하게 읽는다.
    protection_config = ProtectionConfig(
        enabled=protection_enabled,
        update_interval_loops=int(protection_cfg.get('update_interval_loops', PROTECTION_UPDATE_INTERVAL_LOOPS)),
        singularity_log_interval_loops=int(protection_cfg.get('singularity_log_interval_loops', SINGULARITY_LOG_INTERVAL_LOOPS)),
        damping_force_min=float(protection_cfg.get('damping_force_min', DEFAULT_DAMPING_FORCE_MIN)),
        damping_force_max=float(protection_cfg.get('damping_force_max', DEFAULT_DAMPING_FORCE_MAX)),
        damping_torque_min=float(protection_cfg.get('damping_torque_min', DEFAULT_DAMPING_TORQUE_MIN)),
        damping_torque_max=float(protection_cfg.get('damping_torque_max', DEFAULT_DAMPING_TORQUE_MAX)),
        cond_slow=float(protection_cfg.get('cond_slow', DEFAULT_COND_SLOW)),
        cond_strong=float(protection_cfg.get('cond_strong', DEFAULT_COND_STRONG)),
        cond_stop=float(protection_cfg.get('cond_stop', DEFAULT_COND_STOP)),
        sigma_slow=float(protection_cfg.get('sigma_slow', DEFAULT_SIGMA_SLOW)),
        sigma_strong=float(protection_cfg.get('sigma_strong', DEFAULT_SIGMA_STRONG)),
        sigma_stop=float(protection_cfg.get('sigma_stop', DEFAULT_SIGMA_STOP)),
    )
    op_state_cfg = data['op_state']
    op_state_config = OpStateConfig(
        allowed_states=frozenset(int(s) for s in op_state_cfg['allowed_states']),
        state_names={int(k): v for k, v in op_state_cfg['state_names'].items()},
    )

    config = AppConfig(
        can=CanConfig(
            interface=can_cfg['interface'],
            channel=can_cfg['channel'],
            bitrate=int(can_cfg['bitrate']),
            force_id=int(can_cfg['force_id']),
            torque_id=int(can_cfg['torque_id']),
        ),
        robot=RobotConfig(
            ip=robot_cfg['ip'],
            index=int(robot_cfg['index']),
            apply_robot_commands=bool(robot_cfg['apply_robot_commands']),
            teleop_start_mode=int(robot_cfg['teleop_start_mode']),
            vel_ratio=float(robot_cfg['vel_ratio']),
            acc_ratio=float(robot_cfg['acc_ratio']),
        ),
        log=LogConfig(
            log_level=log_cfg['log_level'],
            log_to_file=log_cfg['log_to_file'],
            log_output_dir=log_cfg['log_output_dir'],
        ),
        control=ControlConfig(
            common=CommonControlConfig(
                period_sec=float(common_cfg['period_sec']),
                bias_sample_count=int(common_cfg['bias_sample_count']),
                stale_sensor_timeout_sec=float(common_cfg['stale_sensor_timeout_sec']),
                force_threshold=float(common_cfg['force_threshold']),
                force_release_threshold=float(common_cfg['force_release_threshold']),
                release_hold_sec=float(common_cfg['release_hold_sec']),
            ),
            damping_force=float(damping_cfg['damping_force']),
            damping_torque=float(damping_cfg['damping_torque']),
            protection=protection_config,
        ),
        op_state=op_state_config,
    )

    if config.control.common.bias_sample_count <= 0:
        raise ValueError('control.common.bias_sample_count must be positive')
    if config.control.common.period_sec <= 0.0:
        raise ValueError('control.common.period_sec must be positive')
    if config.control.common.stale_sensor_timeout_sec <= 0.0:
        raise ValueError('control.common.stale_sensor_timeout_sec must be positive')
    if config.control.common.force_threshold < 0.0:
        raise ValueError('control.common.force_threshold must be non-negative')
    if config.control.common.release_hold_sec < 0.0:
        raise ValueError('control.common.release_hold_sec must be non-negative')
    if config.control.damping_force <= 0.0:
        raise ValueError('control.damping.damping_force must be positive')
    if config.control.damping_torque <= 0.0:
        raise ValueError('control.damping.damping_torque must be positive')
    if config.control.protection.damping_force_min <= 0.0:
        raise ValueError('control.protection.damping_force_min must be positive')
    if config.control.protection.update_interval_loops <= 0:
        raise ValueError('control.protection.update_interval_loops must be positive')
    if config.control.protection.singularity_log_interval_loops <= 0:
        raise ValueError('control.protection.singularity_log_interval_loops must be positive')
    if config.control.protection.damping_force_max < config.control.protection.damping_force_min:
        raise ValueError('control.protection.damping_force_max must be >= damping_force_min')
    if config.control.protection.damping_torque_min <= 0.0:
        raise ValueError('control.protection.damping_torque_min must be positive')
    if config.control.protection.damping_torque_max < config.control.protection.damping_torque_min:
        raise ValueError('control.protection.damping_torque_max must be >= damping_torque_min')
    if not (0.0 < config.control.protection.cond_slow < config.control.protection.cond_strong < config.control.protection.cond_stop):
        raise ValueError('control.protection cond thresholds must satisfy cond_slow < cond_strong < cond_stop')
    if not (config.control.protection.sigma_slow > config.control.protection.sigma_strong > config.control.protection.sigma_stop > 0.0):
        raise ValueError('control.protection sigma thresholds must satisfy sigma_slow > sigma_strong > sigma_stop > 0')
    return config


def resolve_config_path(argv: Sequence[str]) -> Path:
    if not argv:
        return default_config_path()
    if argv[0] in ('-c', '--config'):
        if len(argv) < 2:
            raise ValueError('missing config path after -c/--config')
        return Path(argv[1]).expanduser()
    return Path(argv[0]).expanduser()


def setup_logging(script_name: str = '', config: 'LogConfig | None' = None) -> None:
    level = logging.INFO
    handlers: list[logging.Handler] = [logging.StreamHandler(sys.stdout)]

    if config is not None:
        level = getattr(logging, config.log_level.upper(), logging.INFO)

        if config.log_to_file:
            log_dir = Path(config.log_output_dir)
            log_dir.mkdir(parents=True, exist_ok=True)
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            log_filename = log_dir / '{}_{}.log'.format(script_name, timestamp)
            handlers.append(
                logging.FileHandler(log_filename, encoding='utf-8')
            )
            print('Log file: {}'.format(log_filename))

    logging.basicConfig(
        level=level,
        format='%(asctime)s %(levelname)s %(message)s',
        datefmt='%H:%M:%S',
        handlers=handlers,
    )


# ===================================================================
# F/T sensor reader
# ===================================================================

class FTSensorReader(object):
    def __init__(self, config: CanConfig):
        self._interface = config.interface
        self._channel = config.channel
        self._bitrate = config.bitrate
        self._force_id = config.force_id
        self._torque_id = config.torque_id
        self._lock = threading.Lock()
        self._Fx = self._Fy = self._Fz = self._Tx = self._Ty = self._Tz = 0.0
        self._last_sample_time: Optional[float] = None
        self._running = False
        self._thread = None
        self._bus = None
        self._new_data = threading.Event()

    def start(self):
        log.info('Initializing CAN bus: interface=%s, channel=%s, bitrate=%d',
                 self._interface, self._channel, self._bitrate)
        self._bus = can.Bus(
            interface=self._interface,
            channel=self._channel,
            bitrate=self._bitrate,
        )
        self._send_start_command()
        self._running = True
        self._thread = threading.Thread(target=self._recv_loop, name='FTReader', daemon=True)
        self._thread.start()
        log.info('F/T sensor receiver started')

    def stop(self):
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        if self._bus is not None:
            self._bus.shutdown()
        log.info('F/T sensor receiver stopped')

    def _send_start_command(self):
        data = [0x04, 0x02, 0x06, 0x01, 0x03, 0x01]
        cmd = can.Message(arbitration_id=0x000, data=data, is_extended_id=False)
        try:
            self._bus.send(cmd)
            log.debug('Sensor start command sent')
        except can.CanError as e:
            log.warning('Sensor start command failed, ignored if streaming already: %s', e)

    def _recv_loop(self):
        while self._running:
            try:
                msg = self._bus.recv(timeout=1.0)
                if msg is None:
                    log.warning('[CAN] receive timeout - check sensor connection')
                    continue
                if len(msg.data) < 6:
                    log.warning('[CAN] ignored short frame: id=0x%03X len=%d',
                                msg.arbitration_id, len(msg.data))
                    continue

                now = time.monotonic()
                if msg.arbitration_id == self._force_id:
                    d = msg.data
                    with self._lock:
                        self._Fx = (d[0] * 256 + d[1]) / 100.0 - 300.0
                        self._Fy = (d[2] * 256 + d[3]) / 100.0 - 300.0
                        self._Fz = (d[4] * 256 + d[5]) / 100.0 - 300.0
                        self._last_sample_time = now
                    self._new_data.set()
                elif msg.arbitration_id == self._torque_id:
                    d = msg.data
                    with self._lock:
                        self._Tx = (d[0] * 256 + d[1]) / 500.0 - 50.0
                        self._Ty = (d[2] * 256 + d[3]) / 500.0 - 50.0
                        self._Tz = (d[4] * 256 + d[5]) / 500.0 - 50.0
                        self._last_sample_time = now
                    self._new_data.set()
            except Exception as e:
                if self._running:
                    log.error('[CAN] receive error: %s', e)

    def get_ft(self):
        with self._lock:
            return [self._Fx, self._Fy, self._Fz,
                    self._Tx, self._Ty, self._Tz]

    def sample_age_sec(self) -> Optional[float]:
        with self._lock:
            if self._last_sample_time is None:
                return None
            last_sample_time = self._last_sample_time
        return time.monotonic() - last_sample_time

    def is_fresh(self, timeout_sec: float) -> bool:
        age = self.sample_age_sec()
        return age is not None and age <= timeout_sec

    def wait_for_data(self, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                if self._last_sample_time is not None:
                    return True
            remaining = max(0.0, deadline - time.monotonic())
            self._new_data.wait(timeout=min(0.05, remaining))
            self._new_data.clear()
        return False


# ===================================================================
# Control helpers
# ===================================================================

def measure_bias(sensor, sample_count: int, period_sec: float, stale_timeout_sec: float):
    log.info('Bias measurement: collecting %d samples. Keep robot and sensor still.',
             sample_count)
    accum = [0.0] * 6
    for _ in range(sample_count):
        sample_age = sensor.sample_age_sec()
        if sample_age is None or sample_age > stale_timeout_sec:
            age_msg = 'unknown' if sample_age is None else '{:.1f}ms'.format(sample_age * 1000)
            raise RuntimeError('F/T sensor stale during bias measurement: {}'.format(age_msg))

        ft = sensor.get_ft()
        for j in range(6):
            accum[j] += ft[j]
        time.sleep(period_sec)

    bias = [accum[j] / sample_count for j in range(6)]
    log.info('Bias: F=[%.3f, %.3f, %.3f]N, T=[%.3f, %.3f, %.3f]Nm',
             bias[0], bias[1], bias[2], bias[3], bias[4], bias[5])
    return bias


def compensate_bias(ft_raw, bias):
    return [ft_raw[i] - bias[i] for i in range(6)]


# MDH 변환 행렬을 구성해 각 관절축과 TCP 위치를 계산한다.
def make_tx(distance_m: float):
    transform = np.eye(4)
    transform[0, 3] = distance_m
    return transform


def make_tz(distance_m: float):
    transform = np.eye(4)
    transform[2, 3] = distance_m
    return transform


def make_rx(angle_rad: float):
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, c, -s, 0.0],
        [0.0, s, c, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=float)


def make_rz(angle_rad: float):
    c = math.cos(angle_rad)
    s = math.sin(angle_rad)
    return np.array([
        [c, -s, 0.0, 0.0],
        [s, c, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ], dtype=float)


def make_translation(offset_m):
    transform = np.eye(4)
    transform[:3, 3] = np.asarray(offset_m, dtype=float)
    return transform


def compute_mdh_jacobian(q_deg, tcp_offset_m=TCP_OFFSET_M):
    q_rad = np.radians(np.asarray(q_deg, dtype=float))
    if q_rad.shape[0] != 6:
        raise ValueError('q must contain 6 joint values')

    transform = np.eye(4)
    joint_origins = []
    joint_axes = []

    for i in range(6):
        pre_joint = transform @ make_tx(MDH_A_M[i]) @ make_rx(MDH_ALPHA_RAD[i])
        joint_origins.append(pre_joint[:3, 3].copy())
        joint_axes.append(pre_joint[:3, 2].copy())
        transform = pre_joint @ make_rz(MDH_THETA0_RAD[i] + q_rad[i]) @ make_tz(MDH_D_M[i])

    tcp_transform = transform @ make_translation(tcp_offset_m)
    tcp_origin = tcp_transform[:3, 3]

    jacobian = np.zeros((6, 6), dtype=float)
    for i in range(6):
        axis = joint_axes[i]
        origin = joint_origins[i]
        jacobian[:3, i] = np.cross(axis, tcp_origin - origin)
        jacobian[3:, i] = axis

    return jacobian


# SVD 기반 condition number와 최소 singular value를 로그용 지표로 만든다.
def compute_condition_metrics(jacobian):
    singular_values = np.linalg.svd(jacobian, compute_uv=False)
    sigma_max = float(singular_values[0])
    sigma_min = float(singular_values[-1])
    condition_number = math.inf if sigma_min <= SINGULARITY_EPS else sigma_max / sigma_min
    return condition_number, sigma_min, singular_values


def compute_singularity_metrics(q_deg):
    jacobian = compute_mdh_jacobian(q_deg)
    full_condition, full_sigma_min, full_singular_values = compute_condition_metrics(jacobian)
    linear_condition, linear_sigma_min, _ = compute_condition_metrics(jacobian[:3, :])
    angular_condition, angular_sigma_min, _ = compute_condition_metrics(jacobian[3:, :])

    return {
        'condition_full': full_condition,
        'sigma_min_full': full_sigma_min,
        'manipulability': float(np.prod(full_singular_values)),
        'condition_linear': linear_condition,
        'sigma_min_linear': linear_sigma_min,
        'condition_angular': angular_condition,
        'sigma_min_angular': angular_sigma_min,
    }


# 특이점 로그 계산에 실패했을 때 로그 형식을 유지하기 위한 빈 지표를 만든다.
def empty_singularity_metrics():
    return {
        'condition_full': math.nan,
        'sigma_min_full': math.nan,
        'manipulability': math.nan,
        'condition_linear': math.nan,
        'sigma_min_linear': math.nan,
        'condition_angular': math.nan,
        'sigma_min_angular': math.nan,
    }


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


# Jacobian 지표를 0~1 위험도로 변환해 damping 보간값을 만든다.
def compute_protection_state(q_deg, singularity_metrics, protection_config: ProtectionConfig):
    q5_abs_deg = abs(float(q_deg[4]))
    cond_full = singularity_metrics['condition_full']
    sigma_min_full = singularity_metrics['sigma_min_full']

    if math.isfinite(cond_full) and cond_full > protection_config.cond_slow:
        cond_span = math.log10(protection_config.cond_stop) - math.log10(protection_config.cond_slow)
        cond_ratio = (math.log10(cond_full) - math.log10(protection_config.cond_slow)) / cond_span
        cond_severity = clamp(cond_ratio, 0.0, 1.0)
    else:
        cond_severity = 0.0

    if math.isfinite(sigma_min_full) and sigma_min_full < protection_config.sigma_slow:
        sigma_span = math.log10(protection_config.sigma_slow) - math.log10(protection_config.sigma_stop)
        sigma_ratio = (math.log10(protection_config.sigma_slow) - math.log10(max(sigma_min_full, SINGULARITY_EPS))) / sigma_span
        sigma_severity = clamp(sigma_ratio, 0.0, 1.0)
    else:
        sigma_severity = 0.0

    severity = max(cond_severity, sigma_severity)
    damping_force = protection_config.damping_force_min + severity * (
        protection_config.damping_force_max - protection_config.damping_force_min
    )
    damping_torque = protection_config.damping_torque_min + severity * (
        protection_config.damping_torque_max - protection_config.damping_torque_min
    )

    reasons = []
    if cond_full >= protection_config.cond_stop:
        reasons.append('cond_stop')
    elif cond_full >= protection_config.cond_strong:
        reasons.append('cond_strong')
    elif cond_full >= protection_config.cond_slow:
        reasons.append('cond_slow')

    if sigma_min_full <= protection_config.sigma_stop:
        reasons.append('sigma_stop')
    elif sigma_min_full <= protection_config.sigma_strong:
        reasons.append('sigma_strong')
    elif sigma_min_full <= protection_config.sigma_slow:
        reasons.append('sigma_slow')

    force_scale = protection_config.damping_force_min / damping_force
    torque_scale = protection_config.damping_torque_min / damping_torque

    return {
        'scale': min(force_scale, torque_scale),
        'force_scale': force_scale,
        'torque_scale': torque_scale,
        'reason': ','.join(reasons) if reasons else 'normal',
        'q5_abs_deg': q5_abs_deg,
        'condition_full': cond_full,
        'sigma_min_full': sigma_min_full,
        'severity': severity,
        'damping_force': damping_force,
        'damping_torque': damping_torque,
    }


# 보호 판단이 비활성화되었거나 아직 갱신 전일 때 사용할 기본 damping 상태를 만든다.
def make_default_protection_state(config: AppConfig, reason='normal'):
    if config.control.protection.enabled:
        damping_force = config.control.protection.damping_force_min
        damping_torque = config.control.protection.damping_torque_min
    else:
        damping_force = config.control.damping_force
        damping_torque = config.control.damping_torque

    return {
        'scale': 1.0,
        'force_scale': 1.0,
        'torque_scale': 1.0,
        'reason': reason,
        'q5_abs_deg': math.nan,
        'condition_full': math.nan,
        'sigma_min_full': math.nan,
        'severity': 0.0,
        'damping_force': damping_force,
        'damping_torque': damping_torque,
    }


def has_force_input(ft_comp, force_threshold, force_release_threshold, is_moving: bool) -> bool:
    max_force = max(abs(ft_comp[i]) for i in range(3))

    if is_moving:
        return max_force > force_release_threshold
    else:
        return max_force > force_threshold


def compute_damping_step(ft_comp, dt, damping_force: float, damping_torque: float):
    command_velocity = [
        ft_comp[0] / damping_force,   # Fx
        ft_comp[1] / damping_force,   # Fy
        ft_comp[2] / damping_force,   # Fz
        ft_comp[3] / damping_torque,  # Tx
        ft_comp[4] / damping_torque,  # Ty
        ft_comp[5] / damping_torque,  # Tz
    ]
    command_step = [command_velocity[i] * dt for i in range(6)]
    return command_step, command_velocity


def monitor_op_state(indy, op_state_config: OpStateConfig) -> tuple[bool, int]:
    try:
        robot_data = indy.get_robot_data()
        op_state = robot_data.get('op_state', -1)
    except Exception as e:
        log.error('[Monitor] get_robot_data() failed: %s', e)
        return False, -1

    if not op_state_config.is_allowed(op_state):
        log.error('[Monitor] Abnormal op_state: %d (%s) → stopping immediately',
                  op_state, op_state_config.name(op_state))
        return False, op_state

    return True, op_state


def check_robot_connection(indy, config: AppConfig) -> bool:
    try:
        robot_data = indy.get_robot_data()
        op_state = robot_data.get('op_state', -1)
        sim_mode = robot_data.get('sim_mode', False)
        log.info('Robot state: op_state=%d (%s), sim_mode=%s',
                 op_state, config.op_state.name(op_state), sim_mode)

        if not config.op_state.is_allowed(op_state) and op_state not in {1, 5}:
            log.error('Robot is in abnormal state: op_state=%d (%s)',
                      op_state, config.op_state.name(op_state))
            return False
        if sim_mode:
            log.warning('Simulation mode is active; real robot may not move.')
        return True
    except Exception as e:
        log.error('Robot connection check failed: %s', e)
        return False


def log_config_summary(config: AppConfig, config_path: Path) -> None:
    log.info('=' * 70)
    log.info('6-axis damping-only admittance controller started')
    log.info('Config: %s', config_path)
    log.info('Robot command mode: %s',
             'APPLY' if config.robot.apply_robot_commands else 'DEBUG_ONLY')
    log.info('Admittance mode: damping_only')
    log.info('Damping force=%.3f, Damping torque=%.3f', config.control.damping_force, config.control.damping_torque)
    log.info('Force threshold=%.3fN', config.control.common.force_threshold)
    log.info('Sensor stale timeout=%.3fs', config.control.common.stale_sensor_timeout_sec)
    log.info('Protection enabled=%s', config.control.protection.enabled)
    log.info('Singularity MDH convention: Tx(a) -> Rx(alpha) -> Rz(theta) -> Tz(d)')
    log.info('Singularity TCP offset estimate: [%.3f, %.3f, %.3f]m',
             TCP_OFFSET_M[0], TCP_OFFSET_M[1], TCP_OFFSET_M[2])
    log.info('Protection damping force range=%.3f..%.3f, torque range=%.3f..%.3f',
             config.control.protection.damping_force_min,
             config.control.protection.damping_force_max,
             config.control.protection.damping_torque_min,
             config.control.protection.damping_torque_max)
    log.info('Protection update interval=%d loops, singularity log interval=%d loops',
             config.control.protection.update_interval_loops,
             config.control.protection.singularity_log_interval_loops)
    log.info('Protection cond thresholds: slow>%.1e, strong>%.1e, stop>%.1e',
             config.control.protection.cond_slow,
             config.control.protection.cond_strong,
             config.control.protection.cond_stop)
    log.info('Protection sigma thresholds: slow<%.1e, strong<%.1e, stop<%.1e',
             config.control.protection.sigma_slow,
             config.control.protection.sigma_strong,
             config.control.protection.sigma_stop)
    log.info('Allowed op_states: %s',
             {config.op_state.name(s) for s in config.op_state.allowed_states})
    log.info('=' * 70)


def log_status(ft_raw, ft_comp, command_velocity, command_step,
               command_pose, loop_count, indy, ctrl_state=None,
               singularity=None, protection=None,
               protection_update_interval=PROTECTION_UPDATE_INTERVAL_LOOPS,
               singularity_log_interval=SINGULARITY_LOG_INTERVAL_LOOPS):
    if loop_count % protection_update_interval != 0:
        return

    moving_axes = []
    units = axis_units()
    for i, name in enumerate(axis_names()):
        if abs(command_step[i]) > 1e-4:
            direction = '+' if command_step[i] > 0 else '-'
            moving_axes.append('{}{} {:.3f}{}'.format(
                direction, name, abs(command_step[i]), units[i]
            ))
    move_str = ', '.join(moving_axes) if moving_axes else 'stop'

    log.debug(
        '[Loop %4d] RAW F=[%+6.2f,%+6.2f,%+6.2f]N RAW T=[%+6.3f,%+6.3f,%+6.3f]Nm',
        loop_count,
        ft_raw[0], ft_raw[1], ft_raw[2],
        ft_raw[3], ft_raw[4], ft_raw[5],
    )
    log.debug(
        '[Loop %4d] COMP F=[%+6.2f,%+6.2f,%+6.2f]N COMP T=[%+6.3f,%+6.3f,%+6.3f]Nm',
        loop_count,
        ft_comp[0], ft_comp[1], ft_comp[2],
        ft_comp[3], ft_comp[4], ft_comp[5],
    )
    log.debug(
        '[Loop %4d] cmd_vel=[%+.3f,%+.3f,%+.3f]mm/s/[%+.3f,%+.3f,%+.3f]deg/s',
        loop_count,
        command_velocity[0], command_velocity[1], command_velocity[2],
        command_velocity[3], command_velocity[4], command_velocity[5],
    )
    log.debug(
        '[Loop %4d] cmd_step=[%+.3f,%+.3f,%+.3f]mm/[%+.3f,%+.3f,%+.3f]deg '
        'cmd_pose=[%+.2f,%+.2f,%+.2f]mm/[%+.2f,%+.2f,%+.2f]deg axes=%s',
        loop_count,
        command_step[0], command_step[1], command_step[2],
        command_step[3], command_step[4], command_step[5],
        command_pose[0], command_pose[1], command_pose[2],
        command_pose[3], command_pose[4], command_pose[5],
        move_str,
    )

    if indy is not None:
        try:
            # 보호 계산에서 읽은 로봇 상태를 로그에서도 재사용한다.
            if ctrl_state is None:
                ctrl_state = indy.get_control_state()
            q = ctrl_state['q']
            qdot = ctrl_state.get('qdot')
            p = ctrl_state['p']
            pdot = ctrl_state.get('pdot')
            log.debug(
                '[Loop %4d] Joint q=[%s]', loop_count,
                ', '.join(f'{v:.3f}' for v in q),
            )
            if qdot is not None:
                log.debug(
                    '[Loop %4d] Joint qdot=[%s]', loop_count,
                    ', '.join(f'{v:.3f}' for v in qdot),
                )
            log.debug(
                '[Loop %4d] Pose  p=[%s]', loop_count,
                ', '.join(f'{v:.3f}' for v in p),
            )
            if pdot is not None:
                log.debug(
                    '[Loop %4d] Pose  pdot=[%s]', loop_count,
                    ', '.join(f'{v:.3f}' for v in pdot),
                )
            if loop_count % singularity_log_interval == 0:
                # 특이점 지표 로그 출력은 CPU와 파일 I/O 부담을 줄이기 위해 낮은 주기로만 수행한다.
                if singularity is None:
                    singularity = compute_singularity_metrics(q)
                log.debug(
                    '[Loop %4d] Singularity cond_full=%.3e sigma_min_full=%.3e '
                    'manip=%.3e cond_linear=%.3e sigma_min_linear=%.3e '
                    'cond_angular=%.3e sigma_min_angular=%.3e',
                    loop_count,
                    singularity['condition_full'],
                    singularity['sigma_min_full'],
                    singularity['manipulability'],
                    singularity['condition_linear'],
                    singularity['sigma_min_linear'],
                    singularity['condition_angular'],
                    singularity['sigma_min_angular'],
                )
            if protection is not None:
                log.debug(
                    '[Loop %4d] Protection severity=%.2f scale=%.2f reason=%s '
                    'q5_abs=%.3fdeg cond_full=%.3e sigma_min_full=%.3e '
                    'damping_force=%.3f damping_torque=%.3f',
                    loop_count,
                    protection['severity'],
                    protection['scale'],
                    protection['reason'],
                    protection['q5_abs_deg'],
                    protection['condition_full'],
                    protection['sigma_min_full'],
                    protection['damping_force'],
                    protection['damping_torque'],
                )
        except Exception as e:
            log.warning('get_control_state failed: %s', e)


# ===================================================================
# Main control loop
# ===================================================================

def main(argv: Optional[Sequence[str]] = None):
    argv = sys.argv[1:] if argv is None else argv
    try:
        config_path = resolve_config_path(argv)
        config = load_config(config_path)
    except Exception as e:
        setup_logging()
        log.error('Configuration load failed: %s', e)
        return

    setup_logging('main_protect', config.log)
    log_config_summary(config, config_path)

    sensor = FTSensorReader(config.can)
    try:
        sensor.start()
    except Exception as e:
        log.error('F/T sensor initialization failed: %s', e)
        return

    log.info('Waiting for first complete F/T sample...')
    if not sensor.wait_for_data(timeout=5.0):
        log.error('F/T sensor data timeout')
        sensor.stop()
        return
    log.info('F/T sensor data confirmed')

    indy = None
    if config.robot.apply_robot_commands:
        log.info('Connecting robot: IP=%s, index=%d', config.robot.ip, config.robot.index)
        try:
            indy = IndyDCP3(robot_ip=config.robot.ip, index=config.robot.index)
        except Exception as e:
            log.error('Robot connection failed: %s', e)
            sensor.stop()
            return

        if not check_robot_connection(indy, config):
            sensor.stop()
            return
    else:
        log.warning('DEBUG_ONLY mode: robot connection, teleop, and MoveTeleL commands are disabled.')

    input('\nKeep robot and F/T sensor still, then press Enter to measure bias... ')
    try:
        bias = measure_bias(
            sensor,
            config.control.common.bias_sample_count,
            config.control.common.period_sec,
            config.control.common.stale_sensor_timeout_sec,
        )
    except RuntimeError as e:
        log.error('%s', e)
        sensor.stop()
        return

    if config.robot.apply_robot_commands:
        log.info('Starting teleoperation mode')
        try:
            indy.start_teleop(config.robot.teleop_start_mode)
            time.sleep(0.5)
        except Exception as e:
            log.error('start_teleop failed: %s', e)
            sensor.stop()
            return

        teleop_state = indy.get_teleop_state()
        log.info('Teleop state: %s', teleop_state)

        robot_data = indy.get_robot_data()
        op_state = robot_data.get('op_state', -1)
        if op_state != 17:
            log.error('Teleop transition failed: op_state=%d (%s), expected 17=TELE_OP',
                      op_state, config.op_state.name(op_state))
            indy.stop_teleop()
            sensor.stop()
            return

    loop_count = 0
    command_pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    is_moving = False
    prev_time = time.monotonic()
    last_protection = make_default_protection_state(config)

    log.info('Control loop started. Press Ctrl+C to stop.')

    try:
        while True:
            t_start = time.monotonic()
            dt = t_start - prev_time
            prev_time = t_start
            ctrl_state_for_log = None
            singularity_for_log = None

            if (
                config.robot.apply_robot_commands
                and loop_count % config.control.protection.update_interval_loops == 0
            ):
                ok, _ = monitor_op_state(indy, config.op_state)
                if not ok:
                    break

            sample_age = sensor.sample_age_sec()
            if sample_age is None or sample_age > config.control.common.stale_sensor_timeout_sec:
                age_msg = 'unknown' if sample_age is None else '{:.1f}ms'.format(sample_age * 1000)
                log.error('F/T sensor stale for %s; stopping control loop', age_msg)
                break

            ft_raw = sensor.get_ft()
            ft_comp = compensate_bias(ft_raw, bias)

            # Jacobian 보호 판단은 주기적으로 갱신하고 다음 갱신 전까지 마지막 damping 값을 재사용한다.
            if (
                config.robot.apply_robot_commands
                and indy is not None
                and loop_count % config.control.protection.update_interval_loops == 0
            ):
                try:
                    ctrl_state_for_log = indy.get_control_state()
                    q = ctrl_state_for_log['q']
                    if config.control.protection.enabled:
                        singularity_for_log = compute_singularity_metrics(q)
                        last_protection = compute_protection_state(
                            q, singularity_for_log, config.control.protection
                        )
                    else:
                        last_protection = make_default_protection_state(config, 'disabled')
                        last_protection['q5_abs_deg'] = abs(float(q[4]))
                        if loop_count % config.control.protection.singularity_log_interval_loops == 0:
                            singularity_for_log = compute_singularity_metrics(q)
                except Exception as e:
                    if config.control.protection.enabled:
                        last_protection = make_default_protection_state(config, 'state_read_failed')
                        last_protection['damping_force'] = config.control.protection.damping_force_max
                        last_protection['damping_torque'] = config.control.protection.damping_torque_max
                        last_protection['force_scale'] = (
                            config.control.protection.damping_force_min / config.control.protection.damping_force_max
                        )
                        last_protection['torque_scale'] = (
                            config.control.protection.damping_torque_min / config.control.protection.damping_torque_max
                        )
                        last_protection['scale'] = min(
                            last_protection['force_scale'], last_protection['torque_scale']
                        )
                        last_protection['severity'] = 1.0
                        log.warning('[Protection] get_control_state failed; max damping applied: %s', e)
                    else:
                        last_protection = make_default_protection_state(config, 'disabled_state_read_failed')
                        log.warning('[Protection] get_control_state failed; protection disabled: %s', e)

            force_detected = has_force_input(
                ft_comp,
                config.control.common.force_threshold,
                config.control.common.force_release_threshold,
                is_moving,
            )

            if force_detected:
                command_step, command_velocity = compute_damping_step(
                    ft_comp, dt,
                    last_protection['damping_force'],
                    last_protection['damping_torque'],
                )
                is_moving = True
            else:
                command_step     = [0.0] * 6
                command_velocity = [0.0] * 6
                is_moving = False

            if (
                config.control.protection.enabled
                and force_detected
                and last_protection['severity'] > 0.0
                and loop_count % config.control.protection.update_interval_loops == 0
            ):
                log.warning(
                    '[Protection] severity=%.2f reason=%s cond_full=%.3e sigma_min_full=%.3e '
                    'damping_force=%.3f damping_torque=%.3f',
                    last_protection['severity'],
                    last_protection['reason'],
                    last_protection['condition_full'],
                    last_protection['sigma_min_full'],
                    last_protection['damping_force'],
                    last_protection['damping_torque'],
                )

            for i in range(6):
                command_pose[i] += command_step[i]

            log_status(
                ft_raw, ft_comp, command_velocity, command_step,
                command_pose, loop_count, indy, ctrl_state_for_log,
                singularity_for_log, last_protection,
                config.control.protection.update_interval_loops,
                config.control.protection.singularity_log_interval_loops,
            )

            if config.robot.apply_robot_commands:
                try:
                    indy.control.MoveTeleL(
                        control_msgs.MoveTeleLReq(
                            tpos=command_pose,
                            vel_ratio=config.robot.vel_ratio,
                            acc_ratio=config.robot.acc_ratio,
                            method=control_msgs.TELE_TASK_TCP,
                        )
                    )
                except Exception as e:
                    log.error('MoveTeleL(TCP) error: %s', e)
                    if not check_robot_connection(indy, config):
                        log.error('Robot connection abnormal; stopping control loop')
                        break

            loop_count += 1

            elapsed = time.monotonic() - t_start
            sleep_time = config.control.common.period_sec - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                log.warning('[Loop %d] control period overrun: %.1fms > %.0fms',
                            loop_count, elapsed * 1000, config.control.common.period_sec * 1000)

    except KeyboardInterrupt:
        log.info('Interrupted by user')
    except Exception as e:
        log.error('Control loop exception: %s', e, exc_info=True)
    finally:
        if config.robot.apply_robot_commands and indy is not None:
            log.info('Stopping teleoperation...')
            try:
                indy.stop_teleop()
                time.sleep(0.3)
                log.info('Teleoperation stopped')
            except Exception as e:
                log.error('stop_teleop error: %s', e)
        else:
            log.info('DEBUG_ONLY mode: no teleoperation session to stop.')

        sensor.stop()
        log.info('System stopped. Total loops: %d', loop_count)
        log.info('Final command pose: X=%.2f Y=%.2f Z=%.2f mm, Rx=%.2f Ry=%.2f Rz=%.2f deg',
                 command_pose[0], command_pose[1], command_pose[2],
                 command_pose[3], command_pose[4], command_pose[5])


if __name__ == '__main__':
    main()
