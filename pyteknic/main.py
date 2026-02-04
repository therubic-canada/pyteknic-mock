# Copyright 2026 The Rubic. All Rights Reserved.
#
# This computer program and its associated files, documentation,
# and materials (collectively referred to as the "Software") are
# proprietary and confidential information of The Rubic ("Company").
# The Software contains trade secrets and proprietary information
# protected by intellectual property laws and international treaties.
# Unauthorized reproduction, distribution, or disclosure of
# the Software or any portion thereof is strictly prohibited.
#
# No part of the Software may be used, reproduced, stored in a
# retrieval system, or transmitted in any form or by any means,
# electronic, mechanical, photocopying, recording, or otherwise,
# without the prior written consent of The Rubic. Any such unauthorized
# use, reproduction, or distribution may result in severe civil and
# criminal penalties and will be prosecuted to the maximum extent
# possible under the law.
# =============================================================================
from __future__ import annotations

from collections import defaultdict
import threading
import time
from typing import TypeAlias
from dataclasses import dataclass

MapIntMotor: TypeAlias = dict[int, 'Motor']


class Hub:
    def __init__(self, port: str) -> None:
        self.port = port
        self._motors: MapIntMotor = defaultdict(Motor)

    def close(self) -> None:
        pass

    def trigger_moves_in_group(self, group_number: int) -> None:
        for motor in self._motors.values():
            if motor._trigger_group == group_number:  # noqa: SLF001
                motor.trigger_move()

    @property
    def motors(self) -> MapIntMotor:
        return self._motors


class Motor:
    def __init__(self) -> None:
        self._position: int = 0
        self._velocity: int = 0
        self._torque: int = 0
        self._homed: bool = False
        self._moving: bool = False
        self._ready: bool = True
        self._trigger_group: int | None = None
        self._status = MotorStatus()

        self._lock = threading.Lock()
        self._motion_thread: threading.Thread | None = None
        self._thread_started = False
        self._stop_requested = False

    def alerts_clear(self) -> None:
        pass

    def assign_trigger_group(self, group_number: int) -> None:
        self._trigger_group = group_number

    def config(self, arg0: str) -> None:
        pass

    def home(self) -> None:
        self._homed = True
        self._status.WasHomed = 1

    def is_ready(self) -> bool:
        return self._ready

    def move(
        self,
        distance: int,
        speed: int = 2000,
        acceleration: int = 3000,
        *,
        absolute: bool = False,
        is_triggered: bool = False,
    ) -> float:
        if self._motion_thread and self._motion_thread.is_alive():
            raise RuntimeError('Motor already moving')

        with self._lock:
            start_pos = self._position
            target_pos = distance if absolute else start_pos + distance

            self._moving = True
            self._stop_requested = False
            self._status.InMotion = 1
            self._status.MoveDone = 0

        def _run_motion() -> None:
            dt = 0.05  # 50 ms timestep
            direction = 1.0 if target_pos > start_pos else -1.0

            v = 0.0
            pos = start_pos

            while True:
                with self._lock:
                    if self._stop_requested:
                        break

                    remaining = abs(target_pos - pos)
                    stopping_distance = (
                        (v * v) / (2 * acceleration) if acceleration > 0 else 0
                    )

                    # Decelerate if needed
                    if remaining <= stopping_distance:
                        v = max(0.0, v - acceleration * dt)
                    else:
                        v = min(speed, v + acceleration * dt)

                    step = direction * v * dt

                    if abs(step) >= remaining:
                        pos = target_pos
                    else:
                        pos += step

                    self._position = int(pos)
                    self._velocity = int(v * direction)

                    if pos == target_pos:
                        break

                time.sleep(dt)

            with self._lock:
                self._position = target_pos
                self._velocity = 0
                self._moving = False
                self._status.InMotion = 0
                self._status.MoveDone = 1

        self._motion_thread = threading.Thread(target=_run_motion, daemon=True)
        self._thread_started = False

        if not is_triggered:
            self._motion_thread.start()
            self._thread_started = True

        return 0.0

    def move_is_done(self) -> bool:
        return not self._moving

    def position(self) -> float:
        return self._position

    def signal_homing_complete(self) -> None:
        self._homed = True

    def status(self) -> MotorStatus:
        return self._status

    def stop(self, *, abrupt: bool = True) -> None:  # noqa: ARG002
        self._moving = False

    def stop_clear(self) -> None:
        pass

    def tail_move(  # noqa: PLR0913
        self,
        distance: int,
        speed: int = 2000,
        acceleration: int = 3000,
        *,
        absolute: bool = False,
        tail_distance: int = 0,  # noqa: ARG002
        tail_speed: int = 2000,  # noqa: ARG002
        is_triggered: bool = False,
    ) -> float:
        self.move(
            distance,
            speed,
            acceleration,
            absolute=absolute,
            is_triggered=is_triggered,
        )
        return 0.0

    def torque(self) -> float:
        return self._torque

    def trigger_move(self) -> None:
        if self._motion_thread is not None and not self._thread_started:
            self._motion_thread.start()
            self._thread_started = True

    def velocity(self) -> float:
        return self._velocity

    def was_homed(self) -> bool:
        return self._homed

    def zero_position(self) -> None:
        self._position = 0

@dataclass
class MotorStatus:
    AFromStart: int = 0
    AbovePosn: int = 0
    AlertPresent: int = 0
    AtTargetVel: int = 0
    BFromEnd: int = 0
    Disabled: int = 0
    Enabled: int = 0
    GoingDisabled: int = 0
    Homing: int = 0
    HwFailure: int = 0
    InA: int = 0
    InB: int = 0
    InDisableStop: int = 0
    InHardStop: int = 0
    InMotion: int = 0
    InNegLimit: int = 0
    InPosLimit: int = 0
    InvInA: int = 0
    InvInB: int = 0
    MotionBlocked: int = 0
    MoveBufAvail: int = 0
    MoveCanceled: int = 0
    MoveCmdComplete: int = 0
    MoveCmdNeg: int = 0
    MoveDone: int = 1
    NotReady: int = 0
    OutOfRange: int = 0
    Ready: int = 1
    ShutdownState: int = 0
    SoftwareInputs: int = 0
    StatusEvent: int = 0
    StepsActive: int = 0
    TimerExpired: int = 0
    UserAlert: int = 0
    VectorSearch: int = 0
    WasHomed: int = 1


class TimeoutException(Exception):  # noqa: N818
    pass
