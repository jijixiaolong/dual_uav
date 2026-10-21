#!/usr/bin/env python3
"""Read persisted FCStd XML without executing FreeCAD, macros or modeling scripts.

This is a coordinate/parameter audit, not a BREP validity/volume certification.
"""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import zipfile
import numpy as np
from scipy.spatial.transform import Rotation

ROOT=Path(__file__).resolve().parents[1]


def placement(obj):
    node=obj.find("Properties/Property[@name='Placement']/PropertyPlacement")
    if node is None:
        return np.eye(3),np.zeros(3)
    d=node.attrib
    return Rotation.from_quat([float(d['Q'+str(i)]) for i in range(4)]).as_matrix(),np.array([float(d['P'+a]) for a in 'xyz'])


def cad_parameters(path):
    with zipfile.ZipFile(path) as z:
        root=ET.fromstring(z.read('Document.xml'))
    types={n.attrib['name']:n.attrib['type'] for n in root.findall('Objects/Object')}
    objects={n.attrib['name']:n for n in root.findall('ObjectData/Object')}
    sketches={}
    for name,obj in objects.items():
        if types.get(name)!='Sketcher::SketchObject':
            continue
        R,t=placement(obj)
        points=[];circles=[];approx=False
        geometry=obj.findall("Properties/Property[@name='Geometry']/GeometryList/Geometry")
        for g in geometry:
            if g.find('Construction') is not None and g.find('Construction').get('value')=='1':
                continue
            line=g.find('LineSegment');circle=g.find('Circle')
            if line is not None:
                for end in ('Start','End'):
                    points.append(R@np.array([float(line.get(end+a)) for a in 'XYZ'])+t)
            elif circle is not None:
                c=R@np.array([float(circle.get('Center'+a)) for a in 'XYZ'])+t
                radius=float(circle.get('Radius'))
                extent=radius*np.sqrt(R[:,0]**2+R[:,1]**2)
                points.extend([c-extent,c+extent]);circles.append(dict(center_mm=c.tolist(),radius_mm=radius,
                    normal=(R@np.array([0,0,1])).tolist()))
            else:
                approx=True
        constraint=obj.find("Properties/Property[@name='Constraints']/ConstraintList")
        sketches[name]=dict(placement_mm=t.tolist(),normal=R[:,2].tolist(),circles=circles,
            geometry_count=len(geometry),constraint_count=int(constraint.get('count',0)) if constraint is not None else None,
            bounds_mm=[np.min(points,axis=0).tolist(),np.max(points,axis=0).tolist()] if points else None,
            unparsed_geometry=approx)
    pads=[]
    for name,obj in objects.items():
        if types.get(name)!='PartDesign::Pad':
            continue
        profile=obj.find("Properties/Property[@name='Profile']/LinkSub")
        if profile is None:
            continue
        sketch=sketches.get(profile.get('value'))
        length=obj.find("Properties/Property[@name='Length']/Float")
        if sketch is None or sketch['bounds_mm'] is None or length is None:
            continue
        L=float(length.get('value'))
        rev=obj.find("Properties/Property[@name='Reversed']/Bool")
        if rev is not None and rev.get('value')=='true': L=-L
        lower,upper=map(np.array,sketch['bounds_mm'])
        delta=L*np.array(sketch['normal'])
        pads.append(dict(name=name,profile=profile.get('value'),length_mm=L,
            sweep_bounds_mm=[np.minimum(lower,lower+delta).tolist(),np.maximum(upper,upper+delta).tolist()]))
    allbounds=np.array([p['sweep_bounds_mm'] for p in pads])
    magnets={k:v for k,v in sketches.items() if k.startswith('Magnets_')}
    props=sketches.get('Props_Hubs_Sketch',{}).get('circles',[])
    motors=sorted({tuple(round(x,6) for x in c['center_mm'][:2]) for c in props})
    return dict(path=str(path.resolve()),sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        body_count=sum(t=='PartDesign::Body' for t in types.values()),sketch_count=len(sketches),
        zero_constraint_sketches=[k for k,v in sketches.items() if v['constraint_count']==0],
        additive_profile_envelope_mm=[allbounds[:,0,:].min(axis=0).tolist(),allbounds[:,1,:].max(axis=0).tolist()],
        motor_centers_xy_mm=motors,magnet_profiles=magnets,
        docking_profiles={k:v for k,v in sketches.items() if 'Dock' in k},
        pads=pads,all_sketch_parameters=sketches,
        limitations=['Read-only persisted XML, no CAD operations, no source generation script executed.',
                    'Profile sweep envelope is not a measured BREP bbox; unparsed arcs/splines explicitly flagged.',
                    'Native solid validity/volumes, material densities, mass, COM and inertias unverified.',
                    'No persistent assembly mate or CAD-to-FLU origin/COM transform provided.'])


def audit(paths,output):
    import mujoco
    model=mujoco.MjModel.from_xml_path(str(ROOT/'mujoco_sim/assets/dual_quads.xml'))
    rotor=np.array([model.site(f'q1_rotor{i}').pos for i in range(4)])
    separation=.44
    other=rotor+np.array([0,separation,0])
    sim_clearance=min(np.linalg.norm(a-b) for a in rotor for b in other)-.12
    cad=[cad_parameters(Path(p)) for p in paths]
    for record in cad:
        centers=np.array(record['motor_centers_xy_mm'])
        if len(centers):
            # 5.1 inch swept disk specified in the CAD README, not blade mesh extent.
            record['disk_radius_from_readme_mm']=5.1*25.4/2
            record['cross_vehicle_disk_gap_at_sim_440mm_mm']=float(min(np.linalg.norm(a-b-[0,440]) for a in centers for b in centers)-5.1*25.4)
            record['disk_gap_at_candidate_372mm_mm']=float(min(np.linalg.norm(a-b-[0,372]) for a in centers for b in centers)-5.1*25.4)
    result=dict(schema='cad_mujoco_parameter_audit_v1',cad=cad,
        mujoco=dict(units='m, kg, s; body FLU; origin assumed COM',body_box_mm=[160,160,60],
            rotor_centers_mm=(rotor*1000).tolist(),rotor_disk_radius_mm=60,
            q1_contact_site_mm=(model.site('q1_conn_site').pos*1000).tolist(),
            q2_contact_site_mm=(model.site('q2_conn_site').pos*1000).tolist(),
            q1_face_normal=[0,1,0],q2_face_normal=[0,-1,0],
            pad_dimensions_mm=[40,20,40],locked_center_spacing_mm=440,
            min_cross_vehicle_rotor_disk_clearance_mm=float(sim_clearance*1000),
            body_mass_kg=1.,body_inertia_kg_m2=[.0082,.0082,.0145]),
        conclusion='Geometry is NOT equivalent; CAD contact mates, COM registration and mass properties remain unverified. Do not replace collision pads with a visual mesh.')
    output.parent.mkdir(parents=True,exist_ok=True)
    output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--cad',action='append',type=Path)
    p.add_argument('--output',type=Path,required=True);args=p.parse_args()
    audit(args.cad or [ROOT/'models/Machinable_Docking_UAV_300.FCStd'],args.output)
