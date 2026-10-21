"""Independent integration experiments for capture passivity, rupture and contact isolation."""
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from mujoco_sim.python.mujoco_world import MuJoCoWorld
from mujoco_sim.python.latch_manager import LatchManager
from mujoco_sim.python.sim_runner import SimulationRunner
from analysis_tools.capture_experiments import fixture, capture_experiment
import docking_core_py as dcp

XML = ROOT/'mujoco_sim/assets/dual_quads.xml'



def test_zero_site_distance_has_continuous_finite_central_wrench():
    w,l = fixture()
    w.data.qpos[w.q2_qpos_adr+1] = .2201
    w.forward();l.update_capture_state()
    forces = []
    for d in (-1e-9,0.,1e-9):
        w.data.qpos[w.q2_qpos_adr+1] = .2201+d
        w.forward()
        f = l.capture_force_generalized()
        assert np.isfinite(f).all()
        forces.append(f)
        np.testing.assert_allclose(f[:3]+f[6:9],0.,atol=1e-14)
        np.testing.assert_allclose(f[3:6],0.,atol=1e-14)
        assert f[1] > 0.8  # A finite tensile axis, even at coincident faces.
    assert np.max(np.abs(forces[0]-forces[2])) < 1e-5



@pytest.mark.parametrize('damping_only',[True,False])
def test_unpowered_capture_conserves_momenta_and_has_finite_energy_budget(damping_only):
    w,l,initial,rows = capture_experiment(damping_only=damping_only)
    assert max(r['linear_error'] for r in rows) < 1e-10
    assert max(r['angular_error'] for r in rows) < 2e-6
    assert max(r['power_W'] for r in rows) <= 1e-12
    # Integration of the real native body states, no reproduction of force formula.
    accounted = [r['kinetic_J']+r['capture_potential_J']+r['dissipated_J']+
                 r['discarded_J']+r['reservoir_J'] for r in rows]
    assert max(accounted) < initial[2]+initial[3]+initial[4]+2e-5
    assert rows[-1]['dissipated_J'] > 1e-5
    if damping_only:
        assert rows[-1]['kinetic_J'] < initial[2]


def test_capture_momentum_error_converges_under_timestep_refinement():
    coarse = capture_experiment(.001)[-1]
    fine = capture_experiment(.00025)[-1]
    assert max(r['angular_error'] for r in fine) < max(r['angular_error'] for r in coarse)


def test_failed_capture_cannot_rearm_from_historical_contact():
    w,l = fixture()
    assert l.is_captured
    # Setup-only geometry sweep: test a return near the face without NEW contact.
    w.data.qpos[w.q2_qpos_adr+1] += .03
    w.forward();l.update_capture_state()
    assert not l.is_captured and not l.has_contacted and not l.attempt_open
    w.data.qpos[w.q2_qpos_adr+1] = .224
    w.forward();l.update_capture_state()
    assert not l.contact_details()
    np.testing.assert_array_equal(l.capture_force_generalized(),np.zeros(w.model.nv))
    assert not l.capture_eligible()
    assert [e['event'] for e in l.events].count('CAPTURE_FAILED') == 1


def test_two_physical_attempts_after_a_fast_first_collision():
    w,l = fixture(dt=.001)
    w.data.qvel[w.q1_qvel_adr+1] = -.3
    w.data.qvel[w.q2_qvel_adr+1] = .3
    w.forward()
    fsm = dcp.HybridFSM(dcp.DockingParams());fsm.command_approach()
    locked = False
    for _ in range(3000):
        # A laboratory position servo reverses the approach using real forces.
        # No position/velocity is changed after initialization.
        for i,target in ((1,-.2199),(2,.2199)):
            v,a = getattr(w,f'q{i}_qvel_adr'),getattr(w,f'q{i}_qpos_adr')
            w.data.xfrc_applied[getattr(w,f'q{i}_body_id'),1] = (25*(target-w.data.qpos[a+1])-8*w.data.qvel[v+1]) if w.time>.15 else 0.
        fsm.set_capture_contact(l.is_contact_confirmed())
        if not l.capture_eligible():
            fsm.invalidate_capture_dwell()
        mode = fsm.update(w.get_quad_state(1),w.get_quad_state(2),w.time,w.dt)
        if mode == dcp.FlightMode.CONNECTED and not locked:
            assert w.time > .15
            assert l.attempt_id >= 2
            assert l.engage_weld()
            locked = True
            break
        w.step(np.zeros(8))
    assert locked
    names = [e['event'] for e in l.events]
    assert names.index('CAPTURE_FAILED') < len(names)-1
    assert sum(e['event']=='CONTACT' for e in l.events) >= 2
    assert sum(e['event']=='CAPTURE_START' for e in l.events) >= 2


@pytest.mark.parametrize('kind',['linear','angular','edge','exit'])
def test_first_collision_invalid_conditions_never_immediately_lock(kind):
    w,l = fixture(dt=.001,offset=(.038,0,0) if kind=='edge' else (0,0,0))
    if kind in ('linear','exit'):
        w.data.qvel[w.q2_qvel_adr+1] = .8
    elif kind == 'angular':
        w.data.qvel[w.q2_qvel_adr+5] = 4.
    w.forward()
    assert l.contact_details()
    fsm = dcp.HybridFSM(dcp.DockingParams());fsm.command_approach()
    for _ in range(20):
        fsm.set_capture_contact(l.is_contact_confirmed())
        mode = fsm.update(w.get_quad_state(1),w.get_quad_state(2),w.time,w.dt)
        assert mode != dcp.FlightMode.CONNECTED
        assert not l.engage_weld()  # Strict instantaneous kinematic/contact guard.
        w.step(np.zeros(8))
    if kind in ('linear','exit','edge'):
        assert not l.is_captured


def obstacle_xml(tmp_path):
    root = ET.parse(XML)
    ET.SubElement(root.getroot().find('worldbody'),'geom',name='obstacle',type='sphere',
                  size='.012',pos='0 .01 2.025',contype='1',conaffinity='0')
    p=tmp_path/'obstacle.xml';root.write(p)
    return p


def active_pairs(w):
    return {frozenset((int(c.geom1),int(c.geom2))) for c in w.data.contact if c.efc_address>=0}


def test_only_connector_pair_is_filtered_third_party_contact_survives_and_restores(tmp_path):
    w=MuJoCoWorld(obstacle_xml(tmp_path))
    w.reset(q1_init_pos=(0,-.2199,2),q2_init_pos=(0,.2199,2))
    l=LatchManager(w);w.forward()
    padpair=frozenset(w.connector_geoms)
    obs=mujoco.mj_name2id(w.model,mujoco.mjtObj.mjOBJ_GEOM,'obstacle')
    third=frozenset((l.q2_conn_geom,obs))
    assert padpair in active_pairs(w) and third in active_pairs(w)
    l.initialize_connected();w.forward()
    assert padpair not in active_pairs(w)
    assert third in active_pairs(w)
    force=np.zeros(6)
    j=next(i for i,c in enumerate(w.data.contact) if frozenset((int(c.geom1),int(c.geom2)))==third)
    mujoco.mj_contactForce(w.model,w.data,j,force)
    assert force[0] > 0.
    l.disengage_weld();w.forward()
    assert padpair in active_pairs(w) and third in active_pairs(w)
    l.initialize_connected()  # t=0 fixture; reset/abort must restore compatibility.
    l.abort();w.forward()
    assert padpair in active_pairs(w) and third in active_pairs(w)
    l.reset();w.forward()
    assert padpair in active_pairs(w) and third in active_pairs(w)


def test_collision_filter_preserves_entire_compatibility_matrix():
    w,l=fixture();f=l.collision_filter
    for locked in (False,True,False):
        l.set_connector_collision_active(not locked)
        for a in range(w.model.ngeom):
            for b in range(a):
                original=bool((int(f.original_type[a])&int(f.original_affinity[b])) |
                              (int(f.original_type[b])&int(f.original_affinity[a])))
                actual=bool((int(w.model.geom_contype[a])&int(w.model.geom_conaffinity[b])) |
                            (int(w.model.geom_contype[b])&int(w.model.geom_conaffinity[a])))
                expected=original and not (locked and {a,b}==w.connector_geoms)
                assert actual==expected


def test_fsm_dwell_resets_on_lost_contact_and_speed_excursion():
    params=dcp.DockingParams();params.lin_vel_tol=.1;params.dwell_time=.03
    fsm=dcp.HybridFSM(params);fsm.command_approach()
    a,b=dcp.RigidBodyState(),dcp.RigidBodyState()
    a.pos=dcp.Vec3(0,-.22,1);b.pos=dcp.Vec3(0,.22,1)
    t=0.
    def tick(contact=True):
        nonlocal t
        t+=.005;fsm.set_capture_contact(contact)
        return fsm.update(a,b,t,.005)
    tick();tick();tick()
    for _ in range(4): tick()
    assert tick(False)==dcp.FlightMode.ALIGN
    tick()
    for _ in range(4): tick()
    b.vel=dcp.Vec3(0,.11,0)
    assert tick()==dcp.FlightMode.ALIGN
    b.vel=dcp.Vec3()
    tick()
    for _ in range(5): assert tick()!=dcp.FlightMode.CONNECTED
    assert tick()==dcp.FlightMode.CONNECTED


def test_high_energy_impact_has_real_rebound_and_capture_failure():
    runner=SimulationRunner()
    rows=runner.run_scenario(duration=1.,dock_cmd_time=0.,undock_cmd_time=None,
        q1_init_y=-.225,q2_init_y=.225,research_options=dict(
            q1_initial_state=dict(velocity_world_mps=[0,2,0]),
            q2_initial_state=dict(velocity_world_mps=[0,-2,0])))
    assert runner.first_contact_time is not None
    assert runner.dock_event_time is None
    assert any(e['event']=='CAPTURE_FAILED' for e in runner.events)
    assert all(not r['weld_active'] for r in rows)
    assert max(r['rel_dist'] for r in rows)>.02


def test_explicit_connector_contact_pair_is_rejected(tmp_path):
    root=ET.parse(XML)
    contact=ET.SubElement(root.getroot(),'contact')
    ET.SubElement(contact,'pair',geom1='q1_conn_funnel',geom2='q2_conn_probe')
    path=tmp_path/'explicit.xml';root.write(path)
    world=MuJoCoWorld(path)
    with pytest.raises(ValueError,match='Explicit connector'):
        LatchManager(world)


def test_active_release_cannot_immediately_relock_from_restored_face_contact():
    world,latch=fixture()
    latch.initialize_connected();latch.disengage_weld();world.forward()
    assert latch.contact_details()
    assert not latch.capture_eligible()
    assert not latch.engage_weld()
    assert not world.is_weld_active()


def test_contact_storage_is_transferred_from_a_finite_internal_reservoir():
    world,latch=fixture()
    initial=latch.capture_reservoir_initial_J
    assert initial==pytest.approx(.00924)
    assert latch.capture_reservoir_energy_J+latch.capture_potential()==pytest.approx(initial)
    before=latch.capture_reservoir_energy_J+latch.capture_potential()
    latch.fail_capture('test_clutch_open')
    assert latch.capture_reservoir_energy_J==pytest.approx(before)
    assert latch.capture_potential()==0.
    # At the same touching pose a new engagement transfers storage, not free work.
    latch.update_capture_state()
    assert latch.is_captured
    assert latch.capture_reservoir_energy_J+latch.capture_potential()==pytest.approx(before)


def test_real_face_contact_inside_20mm_tolerance_needs_6mm_mechanical_engagement():
    world,latch=fixture(offset=(.010,0,0))
    assert latch.contact_details()
    assert .006<np.linalg.norm(latch.relative_state()['position'])<.020
    assert not latch.is_captured
    assert not latch.capture_eligible()
    assert not latch.engage_weld()
    runner=SimulationRunner()
    rows=runner.run_scenario(duration=.3,dock_cmd_time=0.,undock_cmd_time=None,
        q1_init_y=-.2199,q2_init_y=.2199,research_options=dict(q2_initial_offset=[.010,0,0]))
    assert runner.first_contact_time is not None
    assert runner.dock_event_time is None
    assert not any(r['weld_active'] for r in rows)
