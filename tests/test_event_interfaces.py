from copy import deepcopy
import numpy as np
from mujoco_sim.python.event_interfaces import (event_feature_extractor,event_response_recorder,
    event_metrics,CausalMotorCompensation)
from mujoco_sim.python.sim_runner import SimulationRunner


def test_event_features_are_pre_event_causal_and_motor_gate_never_backfills():
    runner=SimulationRunner()
    rows=runner.run_scenario(duration=.2,dock_cmd_time=0,undock_cmd_time=None,
                             q1_init_y=-.2199,q2_init_y=.2199)
    event=next(e for e in runner.events if e['event']=='DOCK')
    packet=event_feature_extractor(event)
    altered=deepcopy(event);altered['post']['qvel']=[999]*12
    assert event_feature_extractor(altered)==packet
    class Fixed:
        def reset(self): pass
        def on_event(self,packet): pass
        def motor_delta(self,t): return np.ones(8)*.1
    gate=CausalMotorCompensation(Fixed())
    assert not np.any(gate.motor_delta(0.))
    gate.on_event(packet)
    assert not np.any(gate.motor_delta(packet.time_s-1e-9))
    np.testing.assert_array_equal(gate.motor_delta(packet.time_s),np.full(8,.1))
    gate.reset();assert not np.any(gate.motor_delta(99.))
    responses=event_response_recorder(rows,runner.events,horizon_s=.5)
    assert len(responses)==len(runner.events)
    assert all(not r['complete'] for r in responses)
    assert event_metrics(rows,event,.5)['window']['right_truncated']


def test_research_compensation_prepares_features_before_switch_and_activates_after():
    from mujoco_sim.python.research_policy import ResearchPolicy
    class AuditedPolicy(ResearchPolicy):
        def prepare_event(self,runner,event,t,motorcopy):
            if event=='DOCK':
                assert not runner.world.is_weld_active()
            return super().prepare_event(runner,event,t,motorcopy)
        def on_event(self,runner,event,t,motorcopy,pre_packet=None):
            if event=='DOCK':
                assert runner.world.is_weld_active()
                assert pre_packet is not None
            return super().on_event(runner,event,t,motorcopy,pre_packet=pre_packet)
    runner=SimulationRunner();policy=AuditedPolicy('B1')
    runner.run_scenario(duration=.2,dock_cmd_time=0,undock_cmd_time=None,q1_init_y=-.2199,q2_init_y=.2199,
                        controller_policy=policy)
    assert policy.pre_event_packets['DOCK']['time']==runner.dock_event_time


def test_counterfactual_weld_kept_is_an_intervention_not_an_undock_event():
    from mujoco_sim.python.research_policy import ResearchPolicy
    runner=SimulationRunner();policy=ResearchPolicy('B1')
    rows=runner.run_scenario(duration=1.3,preconnected=True,undock_cmd_time=0.,
        research_options=dict(release_constraint=False,switch_controller=False),controller_policy=policy)
    assert runner.release_transition_time is not None
    assert runner.release_event_time is None
    assert any(e['event']=='INTERVENTION' for e in runner.events)
    assert not any(e['event']=='UNDOCK' for e in runner.events)
    assert 'UNDOCK' not in policy.pre_event_packets
    assert all(r['release_event_time_s'] is None for r in rows)
    assert all(r['weld_active'] for r in rows)
