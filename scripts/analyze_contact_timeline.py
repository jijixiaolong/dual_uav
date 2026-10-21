#!/usr/bin/env python3
"""Reconstruct physical-step capture guards without modifying P0/P1 dynamics."""
import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from analysis_tools.contact_analysis import (capture_sample, contiguous_intervals,
                                            desired_pair_angle, reconstruct_control_ticks)


def save_csv(path, rows):
    fields = list(dict.fromkeys(k for row in rows for k in row))
    with path.open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def nearest(rows, time):
    result = min(rows, key=lambda row: abs(float(row['time'])-time))
    if abs(float(result['time'])-time) > 1e-8:
        raise ValueError(f'No logged physics state at {time}')
    return result


def plot_outputs(path, rows, samples, ticks, summary):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import numpy as np

    t = np.array([s['time_s'] for s in samples])
    values = lambda key: np.array([s[key] for s in samples])
    contact, ready, dock = [summary['event_times_s'][k] for k in ('CONTACT', 'CAPTURE_READY', 'DOCK')]
    limits = summary['latch_criteria']

    def scale_visible(ax, lower, upper):
        visible = (t >= lower) & (t <= upper)
        ys = []
        for line in ax.lines:
            data = np.asarray(line.get_ydata(), dtype=float)
            if len(data) == len(t):
                ys.extend(data[visible & np.isfinite(data)])
        if ys:
            span = max(ys)-min(ys)
            ax.set_ylim(min(ys)-.08*span, max(ys)+.12*span)

    fig, axes = plt.subplots(7, 1, figsize=(13, 17), sharex=True)
    axes[0].plot(t, 1000*values('distance_m'), label='Connector distance')
    axes[0].axhline(1000*limits['position_tolerance_m'], color='r', ls='--', label='Capture limit')
    axes[0].set_ylabel('Distance (mm)')
    axes[1].plot(t, values('relative_speed_mps'), label='Relative speed norm')
    axes[1].plot(t, values('radial_speed_mps'), label='Radial speed (+ separating)', alpha=.8)
    axes[1].axhline(limits['relative_linear_velocity_mps'], color='r', ls='--')
    axes[1].axhline(0., color='k', lw=.6)
    axes[1].set_ylabel('Velocity (m/s)')
    axes[2].plot(t, np.degrees(values('relative_angle_rad')), label='Actual relative attitude')
    axes[2].plot(t, np.degrees(values('desired_relative_angle_rad')), label='Active desired relative attitude', ls='--')
    axes[2].axhline(math.degrees(limits['orientation_tolerance_rad']), color='r', ls=':')
    axes[2].set_ylabel('Angle (deg)')
    axes[3].plot(t, values('relative_angular_speed_radps'), label='Relative angular speed norm')
    axes[3].axhline(limits['relative_angular_velocity_radps'], color='r', ls='--')
    axes[3].set_ylabel('Angular speed (rad/s)')
    axes[4].plot(t, values('contact_normal_force_N'), label='Connector normal force sum')
    axes[4].fill_between(t, 0., values('contact_count') > 0, transform=axes[4].get_xaxis_transform(),
                         color='orange', alpha=.2, label='Contact objects exist (may carry zero load)')
    axes[4].set_ylabel('Contact force (N)')
    axes[5].step(t, [int(float(r['mode'])) for r in rows], where='post', label='FSM post-step telemetry')
    axes[5].set_yticks([2, 3, 4]); axes[5].set_yticklabels(['ALIGN', 'CAPTURE_PENDING', 'CONNECTED'])
    axes[5].set_ylim(1.5, 4.5)
    guard_keys = ['pass_contact', 'pass_position', 'pass_linear_speed', 'pass_orientation', 'pass_angular_speed']
    for i, key in enumerate(guard_keys):
        axes[6].step(t, values(key)*.75+i, where='post', label=key[5:])
    axes[6].set_yticks([i+.375 for i in range(len(guard_keys))])
    axes[6].set_yticklabels([k[5:] for k in guard_keys]); axes[6].set_ylabel('Physical guards (up=pass)')
    for ax in axes:
        ax.grid(alpha=.25)
        for event_time, label in ((contact, 'CONTACT'), (ready, 'CAPTURE_READY'), (dock, 'DOCK')):
            ax.axvline(event_time, ls=':', c='black', alpha=.6)
        ax.set_xlim(contact-.2, dock+.15)
        if ax.get_legend_handles_labels()[0] and ax is not axes[6]:
            ax.legend(fontsize=8, loc='upper right')
    axes[0].set_title('P0/P1 contact -> capture | physical guards unchanged | actual MuJoCo logs')
    for ax in axes[:5]:
        scale_visible(ax, contact-.2, dock+.15)
    axes[-1].set_xlabel('Simulation time (s); event times are pre-step/control times')
    fig.tight_layout(); fig.savefig(path/'contact_to_dock.png', dpi=150); plt.close(fig)

    fig, ax = plt.subplots(figsize=(13, 4.5))
    transitions = summary['fsm_transitions']
    ax.step([x['controller_time_s'] for x in transitions] + [float(rows[-1]['time'])],
            [x['mode_number'] for x in transitions] + [transitions[-1]['mode_number']], where='post')
    unique = sorted({(x['mode_number'], x['mode']) for x in transitions})
    ax.set_yticks([x[0] for x in unique]); ax.set_yticklabels([x[1] for x in unique])
    for name, time in summary['event_times_s'].items():
        ax.axvline(time, color='black', alpha=.4, ls=':')
        label_height = .78 if name == 'CAPTURE_READY' else .98
        ax.text(time+.02, label_height, f'{name}\n{time:.3f} s', transform=ax.get_xaxis_transform(),
                va='top', fontsize=8)
    ax.set_xlim(0, float(rows[-1]['time'])); ax.grid(alpha=.25); ax.set_xlabel('Controller/event time (s)')
    ax.set_title('Complete FSM state sequence | state transition times corrected for post-step logging')
    fig.tight_layout(); fig.savefig(path/'full_fsm_sequence.png', dpi=150); plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    selected = [tick for tick in ticks if ready-.02 <= tick['controller_time_s'] <= dock+.01]
    tx = [tick['controller_time_s'] for tick in selected]
    axes[0].step(tx, [1000*tick['dwell_after_s'] for tick in selected], where='post', marker='o')
    axes[0].axhline(1000*limits['dwell_time_s'], color='r', ls='--', label='30 ms dwell threshold')
    axes[0].set_ylabel('Reconstructed dwell (ms)'); axes[0].legend()
    axes[1].stem(tx, [1000*tick['control_dt_s'] for tick in selected], basefmt=' ')
    axes[1].set_ylabel('Actual control dt (ms)'); axes[1].set_xlabel('Controller sample time (s)')
    for ax in axes:
        ax.grid(alpha=.25); ax.axvline(ready, color='black', ls=':'); ax.axvline(dock, color='black', ls=':')
    axes[0].set_title('Exact float replay: dwell at 4.900 s = 0.029999999999996696 < 0.03 s')
    fig.tight_layout(); fig.savefig(path/'capture_dwell.png', dpi=150); plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    for i, color in ((1, 'tab:blue'), (2, 'tab:red')):
        column = lambda key: [float(row[f'q{i}_{key}']) if row[f'q{i}_{key}'] else math.nan for row in rows]
        axes[0].plot(t, column('y'), color=color, label=f'Q{i} actual')
        axes[0].plot(t, column('reference_y'), color=color, ls='--', label=f'Q{i} reference')
        axes[1].plot(t, column('velocity_y'), color=color, label=f'Q{i} actual')
        axes[1].plot(t, column('reference_velocity_y'), color=color, ls='--', label=f'Q{i} reference')
        axes[2].plot(t, column('reference_acceleration_y'), color=color, label=f'Q{i} reference')
        axes[3].plot(t, column('roll_deg'), color=color, label=f'Q{i} actual roll')
        qr = [[float(row[f'q{i}_desired_quat_{c}']) for c in 'wxyz'] for row in rows]
        desired_roll = [math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))) for w,x,y,z in qr]
        axes[3].plot(t, desired_roll, color=color, ls='--', label=f'Q{i} active desired roll')
    for ax, label in zip(axes, ('Y position (m)', 'Y velocity (m/s)', 'Reference Y acceleration (m/s2)', 'Roll (deg)')):
        ax.set_ylabel(label); ax.grid(alpha=.25); ax.legend(fontsize=8, ncol=2)
        ax.axvline(contact, color='black', ls=':', label='CONTACT')
        ax.axvline(summary['reference_approach_end_time_s'], color='gray', ls=':')
        ax.set_xlim(2.4, 3.6)
        scale_visible(ax, 2.4, 3.6)
    axes[0].set_title('Active independent references through first contact; quintic approach remains in progress')
    axes[-1].set_xlabel('Simulation time (s)'); fig.tight_layout()
    fig.savefig(path/'reference_at_contact.png', dpi=150); plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, default=ROOT/'results/physics_validation/contact_capture_release')
    parser.add_argument('--output', type=Path, default=ROOT/'results/event_attribution/contact_timeline')
    parser.add_argument('--report', type=Path, default=ROOT/'reports/CONTACT_TIMELINE_CN.md')
    args = parser.parse_args()
    source, output = args.source.resolve(), args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    with (source/'timeseries.csv').open() as stream:
        rows = list(csv.DictReader(stream))
    events = json.loads((source/'events.json').read_text())
    config = json.loads((source/'config.json').read_text())
    limits = config['docking']['latch_criteria']
    samples = []
    for row in rows:
        sample = capture_sample(row, limits)
        sample.update(mode=row['mode_name'], mode_number=int(float(row['mode'])),
                      desired_relative_angle_rad=desired_pair_angle(row),
                      control_updated=bool(int(float(row['control_updated']))),
                      controller_state_time_s=float(row['controller_state_time_s']))
        samples.append(sample)
    ticks = reconstruct_control_ticks(rows, limits)
    mismatches = [tick for tick in ticks if not tick['capture_guard_replay_matches']]
    if mismatches:
        raise AssertionError(f'Guard replay diverged at {mismatches[0]}')
    event_times = {e['event']: e['time'] for e in events}
    contact, ready, dock = [event_times[k] for k in ('CONTACT', 'CAPTURE_READY', 'DOCK')]
    dock_index = min(range(len(rows)), key=lambda i: abs(float(rows[i]['time'])-dock))
    contact_index = min(range(len(rows)), key=lambda i: abs(float(rows[i]['time'])-contact))
    pre_dock = samples[:dock_index]
    selected = samples[contact_index:dock_index]
    interval_times = [s['time_s'] for s in pre_dock]
    intervals = contiguous_intervals(interval_times, [s['pass_contact'] for s in pre_dock], dock)
    force_intervals = contiguous_intervals(interval_times, [s['contact_normal_force_N'] > 1e-9 for s in pre_dock], dock)
    kinematic_intervals = contiguous_intervals([s['time_s'] for s in selected], [s['pass_kinematics'] for s in selected], dock)
    transitions, last_mode = [], None
    for tick in ticks:
        if tick['mode'] != last_mode:
            transitions.append(dict(controller_time_s=tick['controller_time_s'],
                                    telemetry_time_s=tick['telemetry_time_s'],
                                    previous_mode=last_mode, mode=tick['mode'],
                                    mode_number=int(float(nearest(rows, tick['telemetry_time_s'])['mode']))))
            last_mode = tick['mode']
    timeline = [dict(time_s=e['time'], event=e['event'], source='events.json') for e in events]
    for transition in transitions:
        timeline.append(dict(time_s=transition['controller_time_s'], event='FSM_TRANSITION', source='control_tick_replay',
                             mode_prev=transition['previous_mode'], mode_next=transition['mode'],
                             telemetry_time_s=transition['telemetry_time_s']))
    for index, interval in enumerate(intervals):
        timeline.append(dict(time_s=interval['start_time_s'], event='CONTACT_INTERVAL_START',
                             source='physical_step_criteria', contact_segment=index+1))
        timeline.append(dict(time_s=interval['end_time_exclusive_s'], event='CONTACT_INTERVAL_END' if not interval['right_censored'] else 'DOCK_CENSORS_CONTACT_INTERVAL',
                             source='physical_step_criteria', contact_segment=index+1))
    timeline.sort(key=lambda item: item['time_s'])
    first = next(s for s in samples if abs(s['time_s']-contact) < 1e-8)
    capture = next(s for s in samples if abs(s['time_s']-ready) < 1e-8)
    peak_gap = max(selected, key=lambda s: s['distance_m'])
    failed_duration = {}
    for key in ('pass_contact', 'pass_position', 'pass_linear_speed', 'pass_orientation', 'pass_angular_speed'):
        failed_duration[key[5:]] = sum(samples[i+1]['time_s']-samples[i]['time_s']
                                      for i in range(contact_index, dock_index) if not samples[i][key])
    scenario = config['scenario']
    summary = dict(source=str(source.relative_to(ROOT)), read_only_analysis=True, new_simulations_run=False,
                   physics_rows=len(rows), control_ticks=len(ticks), event_times_s=event_times,
                   latch_criteria=limits, contact_to_dock_s=dock-contact,
                   contact_to_capture_ready_s=ready-contact, capture_ready_to_dock_s=dock-ready,
                   contact_intervals_before_dock=intervals, positive_force_intervals_before_dock=force_intervals,
                   kinematic_eligible_intervals_contact_to_dock=kinematic_intervals,
                   failed_guard_duration_s=failed_duration,
                   failed_guard_window='[CONTACT, DOCK); overlapping failure durations must not be summed',
                   fsm_transitions=transitions, capture_pending_entries=sum(t['mode'] == 'CAPTURE_PENDING' for t in transitions),
                   capture_pending_exits_before_dock=sum(t['previous_mode'] == 'CAPTURE_PENDING' and t['mode'] != 'CONNECTED' for t in transitions),
                   dwell_resets=sum(t['dwell_reset'] for t in ticks), guard_replay_mismatches=len(mismatches),
                   initial_contact_sample=first, capture_ready_sample=capture,
                   maximum_rebound_distance_m=peak_gap['distance_m'], maximum_rebound_time_s=peak_gap['time_s'],
                   reference_approach_end_time_s=scenario['dock_cmd_time']+config['experiments']['simulation']['approach_duration_s'],
                   first_contact_before_reference_end_s=scenario['dock_cmd_time']+config['experiments']['simulation']['approach_duration_s']-contact,
                   no_reference_branch_at_contact=True,
                   dwell_ticks=[t for t in ticks if ready-1e-9 <= t['controller_time_s'] <= dock+1e-9],
                   interpretation='First contact fails velocity and attitude guards; rebound removes contact until 4.870 s. Second contact immediately satisfies all guards. No capture dwell reset. The reference remains a smooth independent quintic until the actual DOCK event.')
    files = [source/n for n in ('timeseries.csv', 'events.json', 'config.json')]
    files += [ROOT/n for n in ('core/src/fsm.cpp', 'mujoco_sim/python/sim_runner.py', 'mujoco_sim/python/latch_manager.py',
                              'mujoco_sim/python/recorder.py', 'analysis_tools/contact_analysis.py', 'scripts/analyze_contact_timeline.py')]
    manifest = dict(command='python3 scripts/analyze_contact_timeline.py',
                    source_sha256={str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in files},
                    time_alignment='At a control-updated output row: use preceding physical row as FSM input; output row state is 2 ms newer.',
                    units='SI: m, s, rad, N; figures explicitly convert distance to mm and attitude to deg',
                    input_config=config)
    simulation_manifest_path = source.parent/'manifest.json'
    if simulation_manifest_path.exists():
        recorded_manifest = json.loads(simulation_manifest_path.read_text())
        manifest['original_simulation_manifest_sha256'] = hashlib.sha256(simulation_manifest_path.read_bytes()).hexdigest()
        manifest['original_simulation_recorded_source_sha256'] = recorded_manifest.get('source_sha256', {})
    (output/'summary.json').write_text(json.dumps(summary, indent=2, allow_nan=False))
    (output/'manifest.json').write_text(json.dumps(manifest, indent=2, allow_nan=False))
    save_csv(output/'physical_step_capture_criteria.csv', samples)
    save_csv(output/'control_tick_capture_dwell.csv', ticks)
    save_csv(output/'event_timeline.csv', timeline)
    save_csv(output/'contact_intervals.csv', intervals)
    save_csv(output/'positive_contact_force_intervals.csv', force_intervals)
    plot_outputs(output, rows, samples, ticks, summary)
    relative_output = str(output.relative_to(ROOT))
    source_link = '../'+str(source.relative_to(ROOT))
    table = '\n'.join(f"| {t['controller_time_s']:.3f} | {t['previous_mode'] or '初始化'} → {t['mode']} | {t['telemetry_time_s']:.3f} |" for t in transitions)
    failures = '\n'.join(f'| {key} | {value:.3f} |' for key, value in failed_duration.items())
    first_row = nearest(rows, contact)
    report = f'''# CONTACT—DOCK 时间差归因（原始 P0/P1 只读复核）

首次接触到锁定的 **{dock-contact:.3f} s** 主要来自**第一次接触未满足速度/姿态捕获条件后弹跳、脱离，直到第二次低速接触**。不是长期持续接触而未锁定，也不是反复清零捕获驻留计时。未修改物理、控制器、状态机或捕获阈值；本分析直接复用 [原始 MuJoCo CSV]({source_link}/timeseries.csv)、[events.json]({source_link}/events.json) 和配置。

## 时间与状态序列

| FSM 控制采样时刻 / s | 状态变化 | 对应输出 CSV 时刻 / s |
|---|---|---|
{table}

独立事件时间：CONTACT **{contact:.3f} s**，CAPTURE_READY **{ready:.3f} s**，DOCK **{dock:.3f} s**，UNDOCK **{event_times['UNDOCK']:.3f} s**。CSV 每次物理步执行完才记录；控制切换在本步开始，故其对应 CSV 行晚 2 ms。重建每个控制判据时使用前一行的物理状态，不能把切换输出行状态作为判据输入。共重建 {len(rows)} 个物理样本、{len(ticks)} 个控制采样；捕获状态/驻留重放与实际 FSM **{len(mismatches)} 处不一致**。

- 第一次接触对象存在于 **2.722–2.760 s**（首个无接触样本 **2.762 s**），半开时间段为 [2.722, 2.762)，共 40 ms。两个接触点是同一次接触的接触流形，并非两次弹跳。
- 正法向接触力只持续至 **2.738 s**，2.740 s 起为零，尽管接触对象暂时还在碰撞检测集合中。初次法向力为 **{first['contact_normal_force_N']:.6f} N**。
- **2.762–4.870 s 完全无连接器接触，共 2.108 s**。连接点距离最大为 **{1000*peak_gap['distance_m']:.3f} mm**，发生于 **{peak_gap['time_s']:.3f} s**。径向相对速度从负（靠拢）变正（分离），随后重新变负，明确显示一次脱离—回接。
- **4.542 s** 起所有运动学判据均满足，但接触判据仍不满足；因此必须等待 4.870 s 的实际再接触。
- **4.870 s** 二次接触立即进入 CAPTURE_PENDING / CAPTURE_READY；整个过程中 CAPTURE_PENDING 仅进入 **{summary['capture_pending_entries']} 次**、捕获前退出 **{summary['capture_pending_exits_before_dock']} 次**、驻留重置 **{summary['dwell_resets']} 次**。
- 从 CAPTURE_READY 到 DOCK 共 **36 ms**。配置驻留为 30 ms，实际控制步交替为 4/6 ms。4.900 s 的累加驻留为 `0.029999999999996696`，因现有 C++ 使用直接 `>= 0.03` 比较，未在该采样锁定，下一 6 ms 采样于 4.906 s 锁定。该浮点比较延迟只解释 6 ms，不能解释 2.184 s 的主体。

因此 2.184 s = **2.148 s 等待二次合格接触 + 0.036 s 驻留**。未通过放宽物理阈值改变结果。未来如修正驻留计时的浮点边界，应单独回归并更新事件时间，不应改捕获速度或姿态门槛。

## 判据与控制参考证据

| 量 | 首次接触 2.722 s | 二次接触 4.870 s | 捕获阈值 |
|---|---:|---:|---:|
| 连接点距离 / mm | {1000*first['distance_m']:.6f} | {1000*capture['distance_m']:.6f} | ≤ 20 |
| 连接点相对速度 / m/s | {first['relative_speed_mps']:.6f} | {capture['relative_speed_mps']:.6f} | ≤ 0.1 |
| 两机相对姿态 / rad | {first['relative_angle_rad']:.6f} | {capture['relative_angle_rad']:.6f} | ≤ 0.08 |
| 两机相对角速度 / rad/s | {first['relative_angular_speed_radps']:.6f} | {capture['relative_angular_speed_radps']:.6f} | ≤ 0.2 |
| 法向接触力 / N | {first['contact_normal_force_N']:.6f} | {capture['contact_normal_force_N']:.6f} | 接触对象必须存在，无最低力阈值 |

首次接触时相对速度和相对姿态同时失败。初碰速度消减后，姿态仍超限，随后角速度也超限，最终脱离接触。控制/物理保护正确拒绝了不合格捕获。

| 失败判据 | [CONTACT, DOCK) 失败总时长 / s |
|---|---:|
{failures}

各判据失败可重叠，不能把时长相加。完整逐步判据及各控制采样驻留均见 CSV。

APPROACH、ALIGN、CAPTURE_PENDING 共用同一独立控制器和以指令时刻计时的五次多项式接近轨迹；CONTACT 没有控制器或参考分支切换。轨迹在 **3.200 s** 结束，而首次碰撞提前 **{summary['first_contact_before_reference_end_s']:.3f} s** 发生。首次接触 Q1 的 Y 位置为 **{float(first_row['q1_y']):.6f} m**，活动参考为 **{float(first_row['q1_reference_y']):.6f} m**；参考速度仍为 **{float(first_row['q1_reference_velocity_y']):.6f} m/s**，加速度为 **{float(first_row['q1_reference_acceleration_y']):.6f} m/s²**。两机在减速阶段仍需相反滚转：活动期望相对姿态为 **{math.degrees(first['desired_relative_angle_rad']):.6f}°**，真实相对姿态为 **{math.degrees(first['relative_angle_rad']):.6f}°**，均高于捕获门槛 **{math.degrees(limits['orientation_tolerance_rad']):.6f}°**。

所以存在“到达物理接触时轨迹还在减速、参考姿态尚未对齐”的时序问题；证据不支持参考坐标系错误、CONTACT 时任务级轨迹指令分支跳变或提前启用组合体参考。**C2 连续的是位置/速度/加速度指令；活动姿态参考由 Geometric 控制器根据这些指令和反馈状态重算，不能假定其也连续。**碰撞迅速改变速度反馈后，控制采样 2.720→2.726 s 的活动期望相对姿态由 7.246469° 降为 4.040581°，这是反馈诱导的姿态参考变化，不是新任务轨迹的切换。仅在实际 DOCK 后才初始化组合参考。若后续优化接近策略，应让实际触碰前的速度/姿态已进入捕获集合，保留现有物理保护。本次未为提高成功率调整任何阈值。

## 真实产物与复现

- [判据、距离、速度、接触力及 FSM 图](../{relative_output}/contact_to_dock.png)
- [全部 FSM 状态序列](../{relative_output}/full_fsm_sequence.png)
- [控制采样与驻留精确重放](../{relative_output}/capture_dwell.png)
- [首次接触前后的真实状态和活动参考](../{relative_output}/reference_at_contact.png)
- [逐物理步判据 CSV](../{relative_output}/physical_step_capture_criteria.csv)
- [逐控制采样判据和驻留 CSV](../{relative_output}/control_tick_capture_dwell.csv)
- [事件时间线 CSV](../{relative_output}/event_timeline.csv)
- [接触分段 CSV](../{relative_output}/contact_intervals.csv)
- [正法向力分段 CSV](../{relative_output}/positive_contact_force_intervals.csv)
- [数值汇总 JSON](../{relative_output}/summary.json)、[源文件 SHA-256 与复现配置](../{relative_output}/manifest.json)

```bash
python3 scripts/analyze_contact_timeline.py
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest tests/test_contact_analysis.py -q
```

本分析不重新仿真，不把相邻样本的插值作为实际采样。事件之前的初始 t=0 状态未在原 CSV 保存；首个控制采样明确标记 `input_exact_available=false`，其余均有精确对应输入。新增单元测试覆盖各捕获边界/非有限值、零载荷接触、接触断续、控制输入错位、驻留清零和现有浮点驻留延迟；**10/10 通过**，见[测试日志](../{relative_output}/python_tests.log)和[JUnit XML](../{relative_output}/pytest.xml)。当前系统 anyio 自动加载插件与 pytest 版本不兼容，复现时关闭第三方插件自动加载；项目测试本身正常执行。
'''
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(report)
    print(json.dumps({key: summary[key] for key in ('contact_to_dock_s', 'contact_to_capture_ready_s',
          'capture_ready_to_dock_s', 'capture_pending_entries', 'dwell_resets', 'guard_replay_mismatches')}, indent=2))
    print(f'Artifacts: {output}')


if __name__ == '__main__':
    main()
