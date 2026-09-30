"""
Standardized Benchmark Scenario Suite (S00 - S15).
Defines deterministic 3D trajectories, distractor kinematics, and disturbance environments.
"""
from dataclasses import dataclass, field
from typing import Dict, Any, List, Tuple, Callable
import numpy as np


@dataclass
class BenchmarkScenario:
    id: str
    name: str
    description: str
    duration: float = 15.0
    designated_target_id: str = "BEACON-01"
    initial_pan_deg: float = 0.0
    initial_tilt_deg: float = 0.0
    disturbances: Dict[str, int] = field(default_factory=lambda: {
        "platform_vibration": 0,
        "camera_jitter": 0,
        "image_noise": 0,
        "blur": 0,
        "atmospheric_turbulence": 0,
        "illumination_variation": 0
    })
    generator: Callable[[float], List[Dict[str, Any]]] = None

    def get_beacons(self, t: float) -> List[Dict[str, Any]]:
        if self.generator:
            return self.generator(t)
        return []


def _s00_generator(t: float) -> List[Dict[str, Any]]:
    # Stationary target at (0, 5, -100)m
    return [{
        "id": "BEACON-01",
        "position": [0.0, 5.0, -100.0],
        "velocity": [0.0, 0.0, 0.0],
        "size": 1.5,
        "intensity": 2.0,
        "color": "#10B981",
        "active": True,
        "designated": True
    }]


def _s01_generator(t: float) -> List[Dict[str, Any]]:
    # Constant transverse linear velocity v = 5.0 m/s
    x = 10.0 - 2.0 * t
    y = 5.0 + 0.2 * t
    z = -100.0 + 0.5 * t
    return [{
        "id": "BEACON-01",
        "position": [x, y, z],
        "velocity": [-2.0, 0.2, 0.5],
        "size": 1.5,
        "intensity": 2.0,
        "color": "#10B981",
        "active": True,
        "designated": True
    }]


def _s02_generator(t: float) -> List[Dict[str, Any]]:
    # Harmonic sinusoidal oscillation
    omega = 2.0 * np.pi * 0.15  # 0.15 Hz
    x = 12.0 * np.sin(omega * t)
    y = 5.0 + 6.0 * np.cos(omega * t)
    z = -100.0
    vx = 12.0 * omega * np.cos(omega * t)
    vy = -6.0 * omega * np.sin(omega * t)
    return [{
        "id": "BEACON-01",
        "position": [x, y, z],
        "velocity": [vx, vy, 0.0],
        "size": 1.5,
        "intensity": 2.0,
        "color": "#10B981",
        "active": True,
        "designated": True
    }]


def _s03_generator(t: float) -> List[Dict[str, Any]]:
    # High-speed flyby (v = 22.0 m/s, high angular rate)
    x = 25.0 - 5.0 * t
    y = 5.0 + 0.4 * t
    z = -80.0
    return [{
        "id": "BEACON-01",
        "position": [x, y, z],
        "velocity": [-5.0, 0.4, 0.0],
        "size": 1.5,
        "intensity": 2.0,
        "color": "#10B981",
        "active": True,
        "designated": True
    }]


def _s04_generator(t: float) -> List[Dict[str, Any]]:
    # Constant transverse acceleration a = 2.5 m/s^2
    a = 2.0
    v0 = 0.5
    x0 = -14.0
    x = x0 + v0 * t + 0.5 * a * (t ** 2)
    vx = v0 + a * t
    return [{
        "id": "BEACON-01",
        "position": [x, 5.0, -100.0],
        "velocity": [vx, 0.0, 0.0],
        "size": 1.5,
        "intensity": 2.0,
        "color": "#10B981",
        "active": True,
        "designated": True
    }]


def _s05_generator(t: float) -> List[Dict[str, Any]]:
    # Sudden direction reversal at t = 5.0s
    if t < 5.0:
        x = -12.0 + 3.0 * t
        y = 5.0
        vx = 3.0
        vy = 0.0
    else:
        dt_after = t - 5.0
        x = 3.0 - 3.5 * dt_after
        y = 5.0 + 2.0 * dt_after
        vx = -3.5
        vy = 2.0
    return [{
        "id": "BEACON-01",
        "position": [x, y, -100.0],
        "velocity": [vx, vy, 0.0],
        "size": 1.5,
        "intensity": 2.0,
        "color": "#10B981",
        "active": True,
        "designated": True
    }]


def _s06_generator(t: float) -> List[Dict[str, Any]]:
    # Stationary target under severe platform vibration
    return [{
        "id": "BEACON-01",
        "position": [0.0, 5.0, -100.0],
        "velocity": [0.0, 0.0, 0.0],
        "size": 1.5,
        "intensity": 2.0,
        "color": "#10B981",
        "active": True,
        "designated": True
    }]


def _s07_generator(t: float) -> List[Dict[str, Any]]:
    # Slow target under sensor noise & blur
    x = 4.0 - 0.8 * t
    return [{
        "id": "BEACON-01",
        "position": [x, 5.0, -100.0],
        "velocity": [-0.8, 0.0, 0.0],
        "size": 1.5,
        "intensity": 2.0,
        "color": "#10B981",
        "active": True,
        "designated": True
    }]


def _s08_generator(t: float) -> List[Dict[str, Any]]:
    # Severe atmospheric fading & optical turbulence
    x = 6.0 - 1.0 * t
    return [{
        "id": "BEACON-01",
        "position": [x, 5.0, -100.0],
        "velocity": [-1.0, 0.0, 0.0],
        "size": 1.5,
        "intensity": 2.0,
        "color": "#10B981",
        "active": True,
        "designated": True
    }]


def _s09_generator(t: float) -> List[Dict[str, Any]]:
    # Temporary dropout (blackout between t=4.0 and t=5.5s)
    x = 10.0 - 2.0 * t
    active = not (4.0 <= t <= 5.5)
    return [{
        "id": "BEACON-01",
        "position": [x, 5.0, -100.0],
        "velocity": [-2.0, 0.0, 0.0],
        "size": 1.5,
        "intensity": 2.0,
        "color": "#10B981",
        "active": active,
        "designated": True
    }]


def _s10_generator(t: float) -> List[Dict[str, Any]]:
    # Target genuinely exits camera FOV (>15 deg at 100m is >26m), holds outside, re-enters at t=7.0s
    if t < 2.0:
        x = 0.0
    elif t < 7.0:
        x = 28.0  # Holds outside sensor FOV
    else:
        dt = t - 7.0
        x = 2.0 - 0.5 * dt  # Re-enters FOV
    return [{
        "id": "BEACON-01",
        "position": [x, 5.0, -100.0],
        "velocity": [0.0, 0.0, 0.0],
        "size": 1.5,
        "intensity": 2.0,
        "color": "#10B981",
        "active": True,
        "designated": True
    }]


def _s11_generator(t: float) -> List[Dict[str, Any]]:
    # Stationary distractor beacon at fixed offset
    # Target moves at 2.5 m/s
    x_target = 8.0 - 2.0 * t
    return [
        {
            "id": "BEACON-01",
            "position": [x_target, 5.0, -100.0],
            "velocity": [-2.0, 0.0, 0.0],
            "size": 1.5,
            "intensity": 2.0,
            "color": "#10B981",
            "active": True,
            "designated": True
        },
        {
            "id": "DISTRACTOR-01",
            "position": [3.0, 7.5, -100.0],  # Fixed distractor
            "velocity": [0.0, 0.0, 0.0],
            "size": 1.5,
            "intensity": 2.0,
            "color": "#10B981",  # Same visual appearance
            "active": True,
            "designated": False
        }
    ]


def _s12_generator(t: float) -> List[Dict[str, Any]]:
    # TEMPORALLY COHERENT MOVING DISTRACTOR
    # Designated target moves steadily along path
    x_target = -8.0 + 1.8 * t
    y_target = 5.0
    z_target = -100.0

    # Distractor enters neighborhood, moves with plausible continuous motion,
    # passes near designated beacon, crosses its trajectory, and remains visible
    # for multiple frames.
    # Distractor trajectory: converges from top-right towards bottom-left
    x_dist = 10.0 - 2.2 * t
    y_dist = 9.0 - 0.7 * t
    z_dist = -98.0 - 0.3 * t
    
    return [
        {
            "id": "BEACON-01",
            "position": [x_target, y_target, z_target],
            "velocity": [1.8, 0.0, 0.0],
            "size": 1.5,
            "intensity": 2.0,
            "color": "#10B981",
            "active": True,
            "designated": True
        },
        {
            "id": "DISTRACTOR-01",
            "position": [x_dist, y_dist, z_dist],
            "velocity": [-2.2, -0.7, -0.3],
            "size": 1.5,
            "intensity": 2.0,
            "color": "#10B981",  # Identical color
            "active": True,
            "designated": False
        }
    ]


def _s13_generator(t: float) -> List[Dict[str, Any]]:
    # Direct collision/crossing trajectory
    # Target moving left-to-right, distractor moving right-to-left
    # They intersect at t = 5.0s, x = 0.0
    x_target = -10.0 + 2.0 * t
    x_dist = 10.0 - 2.0 * t
    return [
        {
            "id": "BEACON-01",
            "position": [x_target, 5.0, -100.0],
            "velocity": [2.0, 0.0, 0.0],
            "size": 1.5,
            "intensity": 2.0,
            "color": "#10B981",
            "active": True,
            "designated": True
        },
        {
            "id": "DISTRACTOR-01",
            "position": [x_dist, 5.05, -100.0],  # Crossing within 0.05m (< 2px)
            "velocity": [-2.0, 0.0, 0.0],
            "size": 1.5,
            "intensity": 2.0,
            "color": "#10B981",
            "active": True,
            "designated": False
        }
    ]


def _s14_generator(t: float) -> List[Dict[str, Any]]:
    # Brighter distractor (3x brightness / larger contour area)
    # Target moves at 1.5 m/s
    x_target = 6.0 - 1.5 * t
    # Distractor parallel at 2.0m offset, 3x intensity
    x_dist = 4.0 - 1.5 * t
    return [
        {
            "id": "BEACON-01",
            "position": [x_target, 5.0, -100.0],
            "velocity": [-1.5, 0.0, 0.0],
            "size": 1.2,
            "intensity": 1.5,
            "color": "#10B981",
            "active": True,
            "designated": True
        },
        {
            "id": "DISTRACTOR-01",
            "position": [x_dist, 8.0, -100.0],
            "velocity": [-1.5, 0.0, 0.0],
            "size": 2.5,
            "intensity": 5.0,  # 3x brighter, larger contour
            "color": "#10B981",
            "active": True,
            "designated": False
        }
    ]


def _s15_generator(t: float) -> List[Dict[str, Any]]:
    # Full combined stress: multi-axis motion + dropout + distractor
    omega = 2.0 * np.pi * 0.12
    x_target = 8.0 * np.sin(omega * t)
    y_target = 5.0 + 4.0 * np.cos(omega * t)
    active = not (4.5 <= t <= 5.5)  # 1.0s dropout

    # Crossing distractor
    x_dist = 8.0 - 1.8 * t
    y_dist = 4.0 + 0.5 * t

    return [
        {
            "id": "BEACON-01",
            "position": [x_target, y_target, -100.0],
            "velocity": [0.0, 0.0, 0.0],
            "size": 1.5,
            "intensity": 2.0,
            "color": "#10B981",
            "active": active,
            "designated": True
        },
        {
            "id": "DISTRACTOR-01",
            "position": [x_dist, y_dist, -100.0],
            "velocity": [-1.8, 0.5, 0.0],
            "size": 1.5,
            "intensity": 2.0,
            "color": "#10B981",
            "active": True,
            "designated": False
        }
    ]


SCENARIO_REGISTRY: Dict[str, BenchmarkScenario] = {
    "S00_nominal": BenchmarkScenario(
        id="S00_nominal",
        name="S00 Nominal Static",
        description="Stationary target under zero disturbances; establishes sensor noise floor and baseline pointing precision.",
        duration=12.0,
        initial_pan_deg=0.5,
        initial_tilt_deg=0.3,
        disturbances={
            "platform_vibration": 0,
            "camera_jitter": 0,
            "image_noise": 0,
            "blur": 0,
            "atmospheric_turbulence": 0,
            "illumination_variation": 0
        },
        generator=_s00_generator
    ),
    "S01_linear": BenchmarkScenario(
        id="S01_linear",
        name="S01 Linear Motion",
        description="Constant transverse velocity tracking at v=5.0 m/s under nominal conditions.",
        duration=12.0,
        disturbances={
            "platform_vibration": 0,
            "camera_jitter": 0,
            "image_noise": 0,
            "blur": 0,
            "atmospheric_turbulence": 0,
            "illumination_variation": 0
        },
        generator=_s01_generator
    ),
    "S02_sinusoidal": BenchmarkScenario(
        id="S02_sinusoidal",
        name="S02 Sinusoidal Motion",
        description="Harmonic orbital motion testing phase lag and bandwidth limits.",
        duration=15.0,
        disturbances={
            "platform_vibration": 0,
            "camera_jitter": 0,
            "image_noise": 0,
            "blur": 0,
            "atmospheric_turbulence": 0,
            "illumination_variation": 0
        },
        generator=_s02_generator
    ),
    "S03_high_speed": BenchmarkScenario(
        id="S03_high_speed",
        name="S03 High-Speed Flyby",
        description="Rapid angular velocity transit testing actuator rate limits and slewing capability.",
        duration=10.0,
        disturbances={
            "platform_vibration": 0,
            "camera_jitter": 0,
            "image_noise": 0,
            "blur": 0,
            "atmospheric_turbulence": 0,
            "illumination_variation": 0
        },
        generator=_s03_generator
    ),
    "S04_acceleration": BenchmarkScenario(
        id="S04_acceleration",
        name="S04 Constant Acceleration",
        description="Second-derivative kinematic stress evaluating constant-velocity Kalman estimator lag.",
        duration=12.0,
        disturbances={
            "platform_vibration": 0,
            "camera_jitter": 0,
            "image_noise": 0,
            "blur": 0,
            "atmospheric_turbulence": 0,
            "illumination_variation": 0
        },
        generator=_s04_generator
    ),
    "S05_direction_change": BenchmarkScenario(
        id="S05_direction_change",
        name="S05 Sudden Direction Change",
        description="Kinematic step maneuver with instantaneous velocity reversal at t=5.0s.",
        duration=12.0,
        disturbances={
            "platform_vibration": 0,
            "camera_jitter": 0,
            "image_noise": 0,
            "blur": 0,
            "atmospheric_turbulence": 0,
            "illumination_variation": 0
        },
        generator=_s05_generator
    ),
    "S06_vibration": BenchmarkScenario(
        id="S06_vibration",
        name="S06 Platform Vibration",
        description="High-frequency mechanical base vibration (intensity 65, 15Hz harmonic jitter).",
        duration=12.0,
        disturbances={
            "platform_vibration": 65,
            "camera_jitter": 0,
            "image_noise": 0,
            "blur": 0,
            "atmospheric_turbulence": 0,
            "illumination_variation": 0
        },
        generator=_s06_generator
    ),
    "S07_sensor_noise": BenchmarkScenario(
        id="S07_sensor_noise",
        name="S07 Sensor Noise & Blur",
        description="Degraded sensor SNR with heavy Gaussian noise (65) and optical blur (15).",
        duration=12.0,
        disturbances={
            "platform_vibration": 0,
            "camera_jitter": 0,
            "image_noise": 65,
            "blur": 15,
            "atmospheric_turbulence": 0,
            "illumination_variation": 0
        },
        generator=_s07_generator
    ),
    "S08_fading": BenchmarkScenario(
        id="S08_fading",
        name="S08 Scintillation Fading",
        description="Atmospheric optical turbulence (65) and deep illumination fluctuations (30).",
        duration=12.0,
        disturbances={
            "platform_vibration": 0,
            "camera_jitter": 0,
            "image_noise": 0,
            "blur": 0,
            "atmospheric_turbulence": 65,
            "illumination_variation": 30
        },
        generator=_s08_generator
    ),
    "S09_temporary_dropout": BenchmarkScenario(
        id="S09_temporary_dropout",
        name="S09 Temporary Dropout",
        description="Complete optical signal loss for 1.5 seconds (t=4.0s to 5.5s) testing Kalman coasting.",
        duration=12.0,
        disturbances={
            "platform_vibration": 0,
            "camera_jitter": 0,
            "image_noise": 0,
            "blur": 0,
            "atmospheric_turbulence": 0,
            "illumination_variation": 0
        },
        generator=_s09_generator
    ),
    "S10_fov_exit_reentry": BenchmarkScenario(
        id="S10_fov_exit_reentry",
        name="S10 FOV Exit & Reentry",
        description="Target maneuvers completely out of camera FOV (>4.5 deg), testing recovery when re-entering.",
        duration=14.0,
        disturbances={
            "platform_vibration": 0,
            "camera_jitter": 0,
            "image_noise": 0,
            "blur": 0,
            "atmospheric_turbulence": 0,
            "illumination_variation": 0
        },
        generator=_s10_generator
    ),
    "S11_stationary_distractor": BenchmarkScenario(
        id="S11_stationary_distractor",
        name="S11 Stationary Distractor",
        description="Moving target passing a stationary false beacon of identical color and intensity at 1.5 deg offset.",
        duration=12.0,
        disturbances={
            "platform_vibration": 0,
            "camera_jitter": 0,
            "image_noise": 0,
            "blur": 0,
            "atmospheric_turbulence": 0,
            "illumination_variation": 0
        },
        generator=_s11_generator
    ),
    "S12_moving_distractor": BenchmarkScenario(
        id="S12_moving_distractor",
        name="S12 Temporally Coherent Moving Distractor",
        description="Temporally coherent distractor beacon enters neighborhood, moves continuously, passes near target, and crosses path.",
        duration=12.0,
        disturbances={
            "platform_vibration": 0,
            "camera_jitter": 0,
            "image_noise": 0,
            "blur": 0,
            "atmospheric_turbulence": 0,
            "illumination_variation": 0
        },
        generator=_s12_generator
    ),
    "S13_crossing_distractor": BenchmarkScenario(
        id="S13_crossing_distractor",
        name="S13 Crossing Distractor",
        description="Two identical optical beacons on collision course crossing within <4px separation at t=5.0s.",
        duration=12.0,
        disturbances={
            "platform_vibration": 0,
            "camera_jitter": 0,
            "image_noise": 0,
            "blur": 0,
            "atmospheric_turbulence": 0,
            "illumination_variation": 0
        },
        generator=_s13_generator
    ),
    "S14_brighter_distractor": BenchmarkScenario(
        id="S14_brighter_distractor",
        name="S14 Brighter Distractor",
        description="Target beacon present alongside a 3x brighter distractor nearby; tests area-based selection bias.",
        duration=12.0,
        disturbances={
            "platform_vibration": 0,
            "camera_jitter": 0,
            "image_noise": 0,
            "blur": 0,
            "atmospheric_turbulence": 0,
            "illumination_variation": 0
        },
        generator=_s14_generator
    ),
    "S15_combined_stress": BenchmarkScenario(
        id="S15_combined_stress",
        name="S15 Combined Stress Mission",
        description="Simultaneous harmonic motion, platform vibration (40), turbulence (45), image noise (35), optical dropout, and crossing distractor.",
        duration=14.0,
        disturbances={
            "platform_vibration": 40,
            "camera_jitter": 20,
            "image_noise": 35,
            "blur": 20,
            "atmospheric_turbulence": 45,
            "illumination_variation": 25
        },
        generator=_s15_generator
    )
}
