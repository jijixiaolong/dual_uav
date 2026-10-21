import json
import numpy as np
from analysis_tools.capture_trials import generate_trials
from mujoco_sim.python.sim_runner import SimulationRunner


def test_trial_generator_seed_reproducibility_and_different_seed_diversity():
    a=generate_trials(1042,2)
    assert json.dumps(a,sort_keys=True)==json.dumps(generate_trials(1042,2),sort_keys=True)
    assert a!=generate_trials(1043,2)
    assert {r['category'] for r in a}=={'safe','boundary','reject'}


def test_reused_runner_restores_plant_capture_controller_motors_and_collision():
    runner=SimulationRunner()
    contaminated=generate_trials(7301,1)[1]['scenario']
    contaminated['duration']=.7
    runner.run_scenario(**contaminated)
    target=generate_trials(1042,1)[0]['scenario'];target['duration']=.7
    reused=runner.run_scenario(**target)
    events=runner.events
    fresh_runner=SimulationRunner();fresh=fresh_runner.run_scenario(**target)
    assert events==fresh_runner.events
    for key in ('q1_y','q2_y','q1_quat_w','q2_quat_w','motor0_command_N',
                'motor7_actual_N','capture_resultant_force_N','rel_dist','weld_active'):
        np.testing.assert_array_equal([r[key] for r in reused],[r[key] for r in fresh])
    nominal=dict(duration=.2,dock_cmd_time=99,undock_cmd_time=None)
    reused=runner.run_scenario(**nominal)
    fresh=SimulationRunner().run_scenario(**nominal)
    assert reused==fresh


def test_preconnected_fixture_applies_pose_and_preload_before_capturing_reference():
    runner=SimulationRunner(zero_preload=False)
    runner.run_scenario(duration=0.,preconnected=True,research_options=dict(
        q2_initial_state=dict(rpy_rad=[.01,-.01,.02]),preload_deflection_m=.004))
    rows=runner.world.data.efc_type==0  # native equality rows
    np.testing.assert_allclose(runner.world.data.efc_pos[rows],0.,atol=1e-12)
    assert runner.latch.events[0]['spring_charging_work_J']==.5*60*.004**2
