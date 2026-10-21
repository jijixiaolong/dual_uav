"""Physics-rate, time-aligned telemetry; weld and collision forces stay separate."""
import csv
import json
from pathlib import Path
import mujoco
import numpy as np


def vec(v):
    return np.array([v.x,v.y,v.z])


def add_xyz(row, name, value):
    row.update({f'{name}_{axis}':float(x) for axis,x in zip('xyz',value)})


def record_state(runner, motor_cmd, nominal, compensation, control_updated, control_dt):
    world, latch = runner.world, runner.latch
    row = dict(time=world.time,mode=int(runner.fsm.current_mode()),
               mode_name=str(runner.fsm.current_mode()).split('.')[-1],
               weld_active=int(world.is_weld_active()),control_updated=int(control_updated),
               control_dt_s=float(control_dt if control_updated else 0.),
               controller_state_time_s=runner.last_control_sample_time,
               first_contact_time_s=runner.first_contact_time,
               dock_event_time_s=runner.dock_event_time,release_event_time_s=runner.release_event_time,
               u_comp_active=int(runner.compensator.is_active(world.time)))
    phase = {'FREE':'FREE','APPROACH':'APPROACH','ALIGN':'APPROACH','CAPTURE_PENDING':'CAPTURE',
             'CONNECTED':'CONNECTED','RELEASE_PREP':'PRE_RELEASE','SEPARATING':'RELEASE','RECOVERY':'SEPARATED','ABORT':'ABORT'}
    row['phase'] = phase[row['mode_name']]
    if row['mode_name'] == 'FREE' and runner.release_event_time is not None:
        row['phase'] = 'SEPARATED'
    row['release_transition_time_s'] = runner.release_transition_time
    row['physical_release_observed'] = int(runner.release_event_time is not None)
    row['aggregate_controller_active'] = int(runner.aggregate_control_active)
    row['control_delay_s'] = float(getattr(runner,'research_options',{}).get('control_delay_s',0.))
    row['motor_tau_s'] = float(world.motor_tau)
    for i in (1,2):
        s, sp = world.get_quad_state(i), getattr(runner,f'sp_q{i}')
        add_xyz(row,f'q{i}',vec(s.pos))
        add_xyz(row,f'q{i}_velocity',vec(s.vel))
        add_xyz(row,f'q{i}_omega_body',vec(s.omega))
        for c in 'wxyz':
            row[f'q{i}_quat_{c}'] = float(getattr(s.quat,c))
        w,x,y,z = s.quat.w,s.quat.x,s.quat.y,s.quat.z
        roll = np.degrees(np.arctan2(2*(w*x+y*z),1-2*(x*x+y*y)))
        pitch = np.degrees(np.arcsin(np.clip(2*(w*y-z*x),-1,1)))
        row[f'q{i}_roll_deg'], row[f'q{i}_pitch_deg'] = float(roll),float(pitch)
        row[f'q{i}_pitch_rate'] = s.omega.y
        if runner.aggregate_control_active:
            Rd = np.array(runner.ctrl_conn.get_desired_rotation().m).reshape(3,3)
            r = -vec(runner.alloc_conn.com_offset())
            if i == 2:
                r += latch.captured_separation
            reference_position = vec(runner.sp_conn.pos)+Rd@r
            if i == 2:
                Rrel = np.zeros(9)
                mujoco.mju_quat2Mat(Rrel,latch.captured_quaternion)
                Rd = Rd@Rrel.reshape(3,3)
        else:
            Rd = np.array(getattr(runner,f'ctrl_q{i}').get_desired_rotation().m).reshape(3,3)
            reference_position = vec(sp.pos)
        error_rotation = Rd.T@world.rotation(i)
        row[f'q{i}_attitude_error_rad'] = float(np.arccos(np.clip((np.trace(error_rotation)-1)/2,-1,1)))
        row[f'q{i}_roll_error_deg'] = float(np.degrees(np.arctan2(error_rotation[2,1],error_rotation[2,2])))
        row[f'q{i}_pitch_error_deg'] = float(np.degrees(np.arcsin(np.clip(-error_rotation[2,0],-1,1))))
        add_xyz(row,f'q{i}_reference',reference_position)
        row[f'q{i}_reference_kinematics_valid'] = int(not runner.aggregate_control_active)
        if runner.aggregate_control_active:
            # COM reference derivatives are logged separately below. Individual
            # point derivatives also require desired frame angular acceleration.
            # Do not present an inactive independent controller's values as valid.
            for kind in ('velocity','acceleration'):
                row.update({f'q{i}_reference_{kind}_{a}':None for a in 'xyz'})
        else:
            add_xyz(row,f'q{i}_reference_velocity',vec(sp.vel))
            add_xyz(row,f'q{i}_reference_acceleration',vec(sp.acc))
        row[f'q{i}_pos_error_m'] = float(np.linalg.norm(vec(s.pos)-reference_position))
        desired_quat = np.zeros(4)
        mujoco.mju_mat2Quat(desired_quat,Rd.ravel())
        for c,x in zip('wxyz',desired_quat):
            row[f'q{i}_desired_quat_{c}'] = float(x)
        adr = getattr(world,f'q{i}_qvel_adr')
        # qfrc_actuator translation is WORLD, freejoint rotation is BODY.
        add_xyz(row,f'q{i}_rotor_force_world',world.data.qfrc_actuator[adr:adr+3])
        add_xyz(row,f'q{i}_rotor_torque_body',world.data.qfrc_actuator[adr+3:adr+6])
    rel = latch.relative_state()
    for key in ('position','velocity','angular_velocity_world'):
        add_xyz(row,f'connector_relative_{key}',rel[key])
    row['relative_normal_velocity_mps'] = float(np.dot(rel['velocity'],world.rotation(1)[:,1]))
    row['relative_angular_speed_radps'] = float(np.linalg.norm(rel['angular_velocity_world']))
    row['relative_attitude_rad'] = rel['angle_rad']
    row['rel_dist'] = float(np.linalg.norm(rel['position']))
    for c,x in zip('wxyz',rel['quaternion']):
        row[f'relative_quat_{c}'] = float(x)
    quantities = world.system_quantities()
    for k in ('com','com_velocity','aggregate_omega_world','linear_momentum','angular_momentum'):
        add_xyz(row,k,quantities[k])
    for k in ('kinetic_energy_J','potential_energy_J','mechanical_energy_J'):
        row[k] = quantities[k]
    for c in 'wxyz':
        row[f'aggregate_quat_{c}'] = getattr(world.get_quad_state(1).quat,c)
    add_xyz(row,'aggregate_reference',vec(runner.sp_conn.pos))
    add_xyz(row,'aggregate_reference_velocity',vec(runner.sp_conn.vel))
    add_xyz(row,'aggregate_reference_acceleration',vec(runner.sp_conn.acc))
    row['aggregate_pos_error_m'] = float(np.linalg.norm(quantities['com']-vec(runner.sp_conn.pos)))
    contacts = latch.contact_details()
    row['connector_contact_count'] = len(contacts)
    row['contact_normal_force_N'] = latch.check_contacts()
    row['contact_impulse_est_Ns'] = world.contact_impulse_est_Ns
    row['step_contact_normal_N'] = world.last_step_contact_normal_N
    row['contacts_json'] = json.dumps([{k:(v.tolist() if isinstance(v,np.ndarray) else v) for k,v in c.items()} for c in contacts])
    qforce = latch.weld_generalized_force()
    for i in (1,2):
        a = getattr(world,f'q{i}_qvel_adr')
        add_xyz(row,f'weld_q{i}_force_world_N',qforce[a:a+3])
        torque = world.rotation(i)@qforce[a+3:a+6]
        add_xyz(row,f'weld_q{i}_torque_at_com_world_Nm',torque)
        # Equivalent wrench about the anchor; a weld transmits a free couple too.
        anchor_local = world.model.eq_data[world.weld_eq_id,3:6] if i == 1 else world.model.eq_data[world.weld_eq_id,:3]
        b = getattr(world,f'q{i}_body_id')
        anchor = world.data.xpos[b]+world.rotation(i)@anchor_local
        add_xyz(row,f'weld_q{i}_anchor_world_m',anchor)
        add_xyz(row,f'weld_q{i}_couple_at_anchor_world_Nm',torque-np.cross(anchor-world.data.xipos[b],qforce[a:a+3]))
    eq = world.model.eq_data[world.weld_eq_id]
    a1 = world.data.xpos[world.q1_body_id]+world.rotation(1)@eq[3:6]
    a2 = world.data.xpos[world.q2_body_id]+world.rotation(2)@eq[:3]
    row['weld_anchor_error_m'] = float(np.linalg.norm(a2-a1)) if world.is_weld_active() else None
    Rref = np.zeros(9)
    mujoco.mju_quat2Mat(Rref,eq[6:10])
    Rerr = Rref.reshape(3,3).T@world.rotation(1).T@world.rotation(2)
    row['weld_attitude_error_rad'] = float(np.arccos(np.clip((np.trace(Rerr)-1)/2,-1,1))) if world.is_weld_active() else None
    row['spring_force_N'] = latch.preload_force
    row['spring_energy_J'] = latch.spring_energy()
    # Current state evaluation and previous integration-step load are distinct.
    latch.capture_force_generalized()
    row['capture_force_N'] = latch.last_capture_force
    for key in ('static_force_N','damping_force_N','resultant_force_N','axial_velocity_mps','damping_power_W','potential_J'):
        row['capture_'+key] = latch.capture_diagnostics[key]
        row['capture_step_'+key] = latch.last_capture_diagnostics[key]
    row['capture_transferred_energy_J'] = latch.capture_transferred_energy_J
    row['capture_reservoir_energy_J'] = latch.capture_reservoir_energy_J
    row['capture_reservoir_initial_J'] = latch.capture_reservoir_initial_J
    row['capture_returned_energy_J'] = latch.capture_returned_energy_J
    row['capture_discarded_energy_J'] = latch.capture_discarded_energy_J
    row['capture_dissipated_energy_J'] = latch.capture_dissipated_energy_J
    row['contact_actual'] = int(latch.current_contact)
    row['contact_state'] = latch.contact_state()
    row['contact_held'] = int(latch.contact_state() == 'CONTACT_HOLD')
    row['capture_attempt_id'] = latch.attempt_id
    row['collision_index'] = latch.collision_index
    row['connector_pair_collision_active'] = int(latch.collision_filter.active)
    row['capture_active'] = int(getattr(latch, 'is_captured', False))
    for k in range(8):
        row[f'motor{k}_nominal_N'] = float(nominal[k])
        row[f'motor{k}_compensation_N'] = float(compensation[k])
        row[f'motor{k}_command_N'] = float(motor_cmd[k])
        row[f'motor{k}_delayed_command_N'] = float(runner.last_delayed_motor_command[k])
        row[f'motor{k}_target_N'] = float(world.target_thrusts[k])
        row[f'motor{k}_actual_N'] = float(world.applied_thrusts[k])
    row['u_mean'] = float(np.mean(motor_cmd))
    row['u_q1'],row['u_q2'] = float(np.mean(motor_cmd[:4])),float(np.mean(motor_cmd[4:]))
    for name,w in runner.last_wrenches.items():
        row[f'{name}_desired_thrust_N'] = w.thrust
        add_xyz(row,f'{name}_desired_torque_Nm',vec(w.torque))
    return row


def save_csv(path, rows):
    path = Path(path)
    path.parent.mkdir(parents=True,exist_ok=True)
    with path.open('w',newline='') as f:
        writer = csv.DictWriter(f,fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
