"""一维垂直恒速卡尔曼滤波器。

公司设备协议只提供离散高度量测，不直接提供垂直速度。本模块使用连续高度
估计状态 ``[z, vz]``，并在有效高度更新不足时明确把速度标记为不可用。
"""

import numpy as np


def _finite(value):
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if np.isfinite(number) else None


class VerticalKalmanFilter:
    """由高度量测估计高度和垂直速度，正速度表示上升。"""

    def __init__(
        self,
        altitude=None,
        dt=0.5,
        *,
        velocity=None,
        altitude_sigma=15.0,
        acceleration_sigma=2.0,
        initial_velocity_sigma=20.0,
        min_velocity_updates=3,
    ):
        self.dt = max(float(dt), 1e-3)
        base_altitude_sigma = _finite(altitude_sigma)
        base_acceleration_sigma = _finite(acceleration_sigma)
        base_velocity_sigma = _finite(initial_velocity_sigma)
        self.altitude_sigma = max(
            abs(15.0 if base_altitude_sigma is None else base_altitude_sigma),
            1e-3,
        )
        self.acceleration_sigma = max(
            abs(2.0 if base_acceleration_sigma is None else base_acceleration_sigma),
            1e-3,
        )
        self.initial_velocity_sigma = max(
            abs(20.0 if base_velocity_sigma is None else base_velocity_sigma),
            1e-3,
        )
        self.min_velocity_updates = max(int(min_velocity_updates), 2)
        self.x = None
        self.P = None
        self.measurement_updates = 0

        z0 = _finite(altitude)
        if z0 is not None:
            vz0 = _finite(velocity)
            self._initialize(z0, 0.0 if vz0 is None else vz0)
            self.measurement_updates = (
                self.min_velocity_updates if vz0 is not None else 1
            )

    def _initialize(self, altitude, velocity=0.0):
        self.x = np.array([float(altitude), float(velocity)], float)
        self.P = np.diag([
            self.altitude_sigma ** 2,
            self.initial_velocity_sigma ** 2,
        ]).astype(float)

    @property
    def altitude(self):
        return None if self.x is None else float(self.x[0])

    @property
    def velocity(self):
        return None if self.x is None else float(self.x[1])

    @property
    def altitude_sigma_estimate(self):
        return None if self.P is None else float(np.sqrt(max(self.P[0, 0], 0.0)))

    @property
    def velocity_sigma_estimate(self):
        return None if self.P is None else float(np.sqrt(max(self.P[1, 1], 0.0)))

    @property
    def velocity_valid(self):
        return (
            self.x is not None
            and self.measurement_updates >= self.min_velocity_updates
            and np.isfinite(self.x[1])
        )

    def predict(self, dt=None):
        if self.x is None:
            return
        step = self.dt if dt is None else max(float(dt), 1e-3)
        F = np.array([[1.0, step], [0.0, 1.0]], float)
        G = np.array([[0.5 * step ** 2], [step]], float)
        Q = G @ G.T * self.acceleration_sigma ** 2
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q

    def update(self, altitude, altitude_sigma=None):
        measurement = _finite(altitude)
        if measurement is None:
            return False
        sigma = self.altitude_sigma
        candidate_sigma = _finite(altitude_sigma)
        if candidate_sigma is not None:
            sigma = max(abs(candidate_sigma), 1e-3)

        if self.x is None:
            self.altitude_sigma = sigma
            self._initialize(measurement)
            self.measurement_updates = 1
            return True

        H = np.array([[1.0, 0.0]], float)
        R = np.array([[sigma ** 2]], float)
        innovation = np.array([measurement]) - H @ self.x
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + (K @ innovation).ravel()
        # Joseph 形式在长期运行时比简化协方差更新更不易失去半正定性。
        I_KH = np.eye(2) - K @ H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        self.measurement_updates += 1
        return True
