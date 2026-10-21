"""Causal, learner-independent event interfaces. No training is performed here."""
from dataclasses import dataclass
from typing import Protocol
import numpy as np
from analysis_tools.metrics import evaluate_window


@dataclass(frozen=True)
class EventFeaturePacket:
    schema: str
    event: str
    time_s: float
    attempt_id: int
    values: tuple


def event_feature_extractor(event):
    """Only event.pre is read. No force oracle, future trajectory or outcome."""
    pre=event['pre']
    values=tuple(pre['qpos']+pre['qvel']+pre['motor_actual_N']+pre['motor_target_N'])
    if len(values)!=42 or not np.isfinite(values).all():
        raise ValueError('Expected 14 qpos, 12 qvel, 8 actual and 8 target motor values')
    return EventFeaturePacket('event_pre_state_v1',event['event'],float(event['time']),
                              int(event.get('attempt_id',0)),values)


def event_response_recorder(rows,events,horizon_s=1.2):
    """Each occurrence retained; failed/short windows explicitly right censored."""
    if horizon_s<=0:
        raise ValueError('Positive response horizon required')
    return [dict(event=e['event'],time_s=e['time'],attempt_id=e.get('attempt_id',0),
                 complete=bool(rows and rows[-1]['time']>=e['time']+horizon_s-1e-10),
                 rows=[r for r in rows if e['time']-1e-10 <= r['time'] <= e['time']+horizon_s+1e-10])
            for e in events]


def event_metrics(rows,event,horizon_s=1.2):
    """Reuse audited active-reference SO(3), recovery/censoring and control metrics."""
    return evaluate_window(rows,event['time'],duration_s=horizon_s)


class LearningControllerInterface(Protocol):
    """ILC, fixed feedforward and future ECLR share the same causal boundary."""
    def reset(self): ...
    def on_event(self, packet: EventFeaturePacket): ...
    def motor_delta(self,time_s: float) -> np.ndarray: ...


class CausalMotorCompensation:
    """Safe event gate wrapping any future implementation; actuator limits remain downstream."""
    def __init__(self, controller: LearningControllerInterface):
        self.controller=controller
        self.reset()

    def reset(self):
        self.event_time=None
        self.controller.reset()

    def on_event(self, packet):
        self.event_time=packet.time_s
        self.controller.on_event(packet)

    def motor_delta(self,time_s):
        if self.event_time is None or time_s < self.event_time:
            return np.zeros(8)
        result=np.asarray(self.controller.motor_delta(time_s),dtype=float)
        if result.shape!=(8,) or not np.isfinite(result).all():
            raise ValueError('Compensation must return eight finite motor deltas')
        return result
