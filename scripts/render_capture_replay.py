#!/usr/bin/env python3
"""Render recorded qpos only. Replays never generate or alter physics outcomes."""
import argparse
import csv
import os
from pathlib import Path
import subprocess
os.environ.setdefault('MUJOCO_GL','egl')
import mujoco
import numpy as np
from PIL import Image,ImageDraw

ROOT=Path(__file__).resolve().parents[1]


def render(csv_path,output,title='',fps=25,slowdown=1.):
    with csv_path.open() as f:rows=list(csv.DictReader(f))
    time=np.array([float(r['time']) for r in rows])
    qpos=np.array([[float(r[f'q{i}_{a}']) for i in (1,2) for a in ('x','y','z','quat_w','quat_x','quat_y','quat_z')] for r in rows])
    model=mujoco.MjModel.from_xml_path(str(ROOT/'mujoco_sim/assets/dual_quads.xml'))
    data=mujoco.MjData(model);renderer=mujoco.Renderer(model,height=360,width=640)
    camera=mujoco.MjvCamera();camera.lookat=[0,0,1.];camera.distance=2.2;camera.elevation=-20;camera.azimuth=30
    output.parent.mkdir(parents=True,exist_ok=True)
    process=subprocess.Popen(['ffmpeg','-y','-loglevel','error','-f','rawvideo','-pix_fmt','rgb24','-s','640x360',
        '-r',str(fps),'-i','-','-c:v','libx264','-threads','1','-preset','fast','-crf','23','-pix_fmt','yuv420p',str(output)],stdin=subprocess.PIPE)
    try:
        for deadline in np.arange(0,time[-1]*slowdown,1/fps):
            index=min(np.searchsorted(time,deadline/slowdown),len(rows)-1);r=rows[index]
            data.qpos[:]=qpos[index];mujoco.mj_forward(model,data);renderer.update_scene(data,camera)
            im=Image.fromarray(renderer.render());draw=ImageDraw.Draw(im)
            draw.rectangle((5,5,480,78),fill=(20,24,32))
            draw.text((12,10),title,fill='white')
            draw.text((12,28),f"t={time[index]:.3f}s  {r['mode_name']}  weld={r['weld_active']}",fill='white')
            draw.text((12,46),f"gap={float(r['rel_dist'])*1000:.2f}mm  contact={float(r['contact_normal_force_N']):.2f}N",fill='white')
            draw.text((12,62),f"Recorded MuJoCo state replay; speed 1/{slowdown:g}",fill='white')
            process.stdin.write(im.tobytes())
            if r['weld_active']=='1' and deadline>4.:
                image_path=output.with_suffix('.png')
                if not image_path.exists():im.save(image_path)
        process.stdin.close();code=process.wait()
        if code:raise RuntimeError(f'FFmpeg exited {code}')
    finally:
        renderer.close()
        if process.stdin and not process.stdin.closed:process.stdin.close()
        if process.poll() is None:process.terminate();process.wait()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--csv',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--title',default='MuJoCo replay');p.add_argument('--slowdown',type=float,default=1.)
    a=p.parse_args();render(a.csv,a.output,a.title,slowdown=a.slowdown)
