#!/usr/bin/env python3
"""Static native-MuJoCo collision experiment: joined pair plus third-party obstacle."""
import argparse
import json
from pathlib import Path
import sys
import tempfile
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from mujoco_sim.python.mujoco_world import MuJoCoWorld
from mujoco_sim.python.latch_manager import LatchManager


def run(output):
    tree=ET.parse(ROOT/'mujoco_sim/assets/dual_quads.xml')
    ET.SubElement(tree.getroot().find('worldbody'),'geom',name='obstacle',type='sphere',
                  size='.012',pos='0 .01 2.025',contype='1',conaffinity='0')
    with tempfile.TemporaryDirectory() as d:
        xml=Path(d)/'obstacle.xml';tree.write(xml)
        world=MuJoCoWorld(xml)
        world.model.opt.gravity[:]=0.
        world.reset(q1_init_pos=(0,-.2199,2),q2_init_pos=(0,.2199,2))
        latch=LatchManager(world);records=[]
        def sample(name):
            world.forward();contacts=[]
            for i,c in enumerate(world.data.contact):
                f=np.zeros(6);mujoco.mj_contactForce(world.model,world.data,i,f)
                contacts.append(dict(geom1=world.model.geom(int(c.geom1)).name,
                    geom2=world.model.geom(int(c.geom2)).name,force_local=f.tolist(),efc_address=int(c.efc_address)))
            records.append(dict(phase=name,weld=world.is_weld_active(),contacts=contacts,snapshot=latch.snapshot()))
        sample('unlocked')
        latch.initialize_connected();sample('locked')
        latch.disengage_weld();sample('released')
        latch.reset();sample('reset')
    result=dict(note='Independent static solver fixture, zero gravity and motors. No integration or state correction at switches.',phases=records)
    output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True);a=p.parse_args();run(a.output)
