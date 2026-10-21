#!/usr/bin/env python3
"""Build the complete authentic machinable 300mm UAV with:
1. 32mm wide carbon fiber arms (>3mm margin around 16x16 motor holes)
2. Decoupled Dual-Tier Mounting:
   - Tier A: PDB (Mateksys FCHUB-12S V2):
     * 4x M3 Brass Standoffs (H=3mm, D=5mm, at +-15.25, +-15.25mm), Z=3.0 to 6.0mm
     * 50 x 50 mm body with R4 corners + 5mm top DC-IN lug tab (Total 55mm)
     * Royal blue PCB (T=1.6mm, Z=6.0 to 7.6mm)
     * Low-profile M3 fastener screws (Z=7.6 to 9.0mm)
     * NO COPPER STANDOFFS ON TOP OF THE PDB per user requirement
     * High-detail onboard components: 2x power inductors ("100"), 2x shunt resistors,
       JST ribbon connector socket, gold DC-IN pads, annular grounding rings, ESC pads
   - Tier B: Flight Controller (CUAV Pixhawk V6X + Carbon FC Deck Plate):
     * 4x M3 Brass Standoffs (H=10mm, D=5mm, at +-36.0, +-12.0mm), Z=3.0 to 13.0mm
       Directly connecting motor chassis plate (电机板子) and flight controller deck!
     * 2mm Carbon fiber FC plate (96 x 54 mm, Z=13.0 to 15.0mm)
     * 44 x 44 mm central window clearing PDB wiring and components
     * CUAV Pixhawk V6X flight controller (85.1 x 45 x 29.2 mm, Z=15.0 to 44.2mm)
     * 8.8mm safe clearance under top protective rim (Z=53.0mm)
3. Landing gear legs REMOVED per user request
4. Lower battery cradle system (Z=-32 to 0mm):
   - 4x M3 Brass standoffs (H=30mm, D=5mm, at +-48, +-26mm)
   - 2mm Carbon fiber battery plate (146 x 56 mm, Z=-32 to -30mm) with strap slots
   - 4S LiPo battery (136 x 43.5 x 29 mm, Z=-29.5 to -0.5mm)
5. T-Motor F60 PRO V - LV motors (5-spoke bell, copper stator, M5 shaft)
6. Gemfan 51477 (5.1-inch) 3-blade aerodynamic propellers
7. Octagonal outer perimeter frame (370 x 370 mm) with hollow top rim & 8 red standoffs
8. Integrated perimeter docking mechanism
"""

import xmlrpc.client
import math
import sys
import time
import subprocess
import glob

def main():
    proxy = xmlrpc.client.ServerProxy('http://127.0.0.1:9875', allow_none=True)
    DOC = "Machinable_Docking_UAV_300"
    
    docs = proxy.list_documents().get('documents', [])
    if DOC in docs:
        print(f"Document {DOC} already exists in FreeCAD, closing it first...")
        proxy.execute_code(f"# close doc\nimport FreeCAD\nif '{DOC}' in FreeCAD.listDocuments(): FreeCAD.closeDocument('{DOC}')")
    
    print(f"Creating fresh document {DOC}...")
    res_doc = proxy.create_document(DOC)
    DOC = res_doc.get('document_name', DOC)
    print("Active doc:", DOC)

    def run_ops(ops, desc):
        for attempt in range(3):
            time.sleep(0.15)
            try:
                res = proxy.execute_operations(DOC, ops)
                status = res.get('success', False) if isinstance(res, dict) else False
                if status:
                    print(f"[{desc}] success=True")
                    return res
                err = res.get('error', '')
                if ('GUI dispatch timed out' in str(err) or 'timed out' in str(err)) and attempt < 2:
                    print(f"[{desc}] GUI timed out, retrying attempt {attempt+2}...")
                    time.sleep(1.0)
                    continue
                print(f"[{desc}] failed: {res}")
                raise RuntimeError(f"Operation failed: {desc} -> {res}")
            except Exception as e:
                if 'timed out' in str(e) and attempt < 2:
                    print(f"[{desc}] Exception timed out, retrying attempt {attempt+2}...")
                    time.sleep(1.0)
                    continue
                raise

    motor_xy = 150.0 / math.sqrt(2) # 106.066 mm
    motor_positions = [
        (motor_xy, motor_xy),
        (-motor_xy, motor_xy),
        (-motor_xy, -motor_xy),
        (motor_xy, -motor_xy)
    ]
    plate_t_bot = 3.0 # 3mm carbon
    plate_t_top = 2.0 # 2mm carbon
    standoff_h = 50.0 # 50mm aluminum
    
    outer_pts = [
        (185.0, 130.0), (130.0, 185.0),
        (-130.0, 185.0), (-185.0, 130.0),
        (-185.0, -130.0), (-130.0, -185.0),
        (130.0, -185.0), (185.0, -130.0)
    ]
    n = len(outer_pts)

    # ==========================================================================
    # BODY 1: Bottom Chassis Plate with 32mm Arms & Enlarged Deck (106 x 68 mm)
    # ==========================================================================
    print("\n=== 1. Building Bottom Chassis Plate (32mm Arms) ===")
    bottom_geo = []
    for i in range(n):
        bottom_geo.append({'type': 'line', 'from': outer_pts[i], 'to': outer_pts[(i+1)%n]})

    # 4 Cutouts for 32mm diagonal arms (Half-width 16mm, normal offset 22.627mm)
    north_pts = [
        (11.37, 34.0),
        (133.46, 156.08),
        (122.54, 167.0),
        (-122.54, 167.0),
        (-133.46, 156.08),
        (-11.37, 34.0)
    ]
    south_pts = [
        (-11.37, -34.0),
        (-133.46, -156.08),
        (-122.54, -167.0),
        (122.54, -167.0),
        (133.46, -156.08),
        (11.37, -34.0)
    ]
    east_pts = [
        (53.0, 11.37),
        (156.08, 133.46),
        (167.0, 122.54),
        (167.0, -122.54),
        (156.08, -133.46),
        (53.0, -11.37)
    ]
    west_pts = [
        (-53.0, -11.37),
        (-156.08, -133.46),
        (-167.0, -122.54),
        (-167.0, 122.54),
        (-156.08, 133.46),
        (-53.0, 11.37)
    ]

    for pts in [north_pts, south_pts, east_pts, west_pts]:
        for i in range(len(pts)):
            bottom_geo.append({'type': 'line', 'from': pts[i], 'to': pts[(i+1)%len(pts)]})

    # 4 Motors 16x16 mounting holes
    for mx, my in motor_positions:
        bottom_geo.append({'type': 'circle', 'center': [round(mx, 3), round(my, 3)], 'radius': 4.5})
        for dx in [-8.0, 8.0]:
            for dy in [-8.0, 8.0]:
                bottom_geo.append({'type': 'circle', 'center': [round(mx + dx, 3), round(my + dy, 3)], 'radius': 1.6})

    # 4 PDB mounting holes (Universal 30.5x30.5mm pattern for 3mm brass standoffs)
    for sx in [-15.25, 15.25]:
        for sy in [-15.25, 15.25]:
            bottom_geo.append({'type': 'circle', 'center': [round(sx, 3), round(sy, 3)], 'radius': 1.6})

    # 4 FC 5mm standoff mounting holes (72x24mm pattern directly connecting chassis to FC plate)
    for sx in [-36.0, 36.0]:
        for sy in [-12.0, 12.0]:
            bottom_geo.append({'type': 'circle', 'center': [round(sx, 3), round(sy, 3)], 'radius': 1.6})

    # 4 Battery standoff mounting holes (96x52mm pattern)
    for sx in [-48.0, 48.0]:
        for sy in [-26.0, 26.0]:
            bottom_geo.append({'type': 'circle', 'center': [round(sx, 3), round(sy, 3)], 'radius': 1.6})

    # 8 Perimeter standoff holes
    for px, py in outer_pts:
        norm = math.hypot(px, py)
        sx = px * (1.0 - 8.0 / norm)
        sy = py * (1.0 - 8.0 / norm)
        bottom_geo.append({'type': 'circle', 'center': [round(sx, 3), round(sy, 3)], 'radius': 1.6})

    run_ops([
        {'action': 'create_object', 'obj_type': 'PartDesign::Body', 'obj_name': 'Body_Bottom_Perimeter_Frame', 'description': 'Create Body_Bottom_Perimeter_Frame'},
        {'action': 'sketch', 'obj_name': 'Bottom_Chassis_Sketch', 'obj_properties': {'plane': 'XY', 'offset': 0.0, 'body': 'Body_Bottom_Perimeter_Frame', 'geometry': bottom_geo}, 'description': 'Sketch bottom chassis 32mm'},
        {'action': 'pad', 'obj_name': 'Bottom_Chassis_Sketch', 'obj_properties': {'length': plate_t_bot, 'body': 'Body_Bottom_Perimeter_Frame'}, 'description': 'Pad bottom chassis 3mm'},
        {'action': 'color', 'obj_name': 'Body_Bottom_Perimeter_Frame', 'obj_properties': {'color': [0.15, 0.15, 0.16]}, 'description': 'Carbon black color'}
    ], "Body_Bottom_Perimeter_Frame")

    # ==========================================================================
    # BODY 2: Top Aerodynamic Protection Rim (2mm Carbon Fiber, Hollow Interior)
    # ==========================================================================
    print("\n=== 2. Building Top Protective Rim ===")
    top_geo = []
    for i in range(n):
        top_geo.append({'type': 'line', 'from': outer_pts[i], 'to': outer_pts[(i+1)%n]})

    inner_pts = []
    for px, py in outer_pts:
        norm = math.hypot(px, py)
        scale = (norm - 18.0) / norm
        inner_pts.append((round(px * scale, 3), round(py * scale, 3)))

    for i in range(n):
        top_geo.append({'type': 'line', 'from': inner_pts[i], 'to': inner_pts[(i+1)%n]})

    run_ops([
        {'action': 'create_object', 'obj_type': 'PartDesign::Body', 'obj_name': 'Body_Top_Perimeter_Frame', 'description': 'Create Body_Top_Perimeter_Frame'},
        {'action': 'sketch', 'obj_name': 'Top_Rim_Sketch', 'obj_properties': {'plane': 'XY', 'offset': plate_t_bot + standoff_h, 'body': 'Body_Top_Perimeter_Frame', 'geometry': top_geo}, 'description': 'Sketch top rim'},
        {'action': 'pad', 'obj_name': 'Top_Rim_Sketch', 'obj_properties': {'length': plate_t_top, 'body': 'Body_Top_Perimeter_Frame'}, 'description': 'Pad top rim 2mm'},
        {'action': 'color', 'obj_name': 'Body_Top_Perimeter_Frame', 'obj_properties': {'color': [0.18, 0.18, 0.20]}, 'description': 'Matte carbon color'}
    ], "Body_Top_Perimeter_Frame")

    # ==========================================================================
    # BODY 3: 8x Perimeter Aluminum Standoffs (D=6mm, H=50mm)
    # ==========================================================================
    print("\n=== 3. Building 8x Perimeter Standoffs ===")
    standoff_geo = []
    for px, py in outer_pts:
        norm = math.hypot(px, py)
        sx = px * (1.0 - 8.0 / norm)
        sy = py * (1.0 - 8.0 / norm)
        standoff_geo.append({'type': 'circle', 'center': [round(sx, 3), round(sy, 3)], 'radius': 3.0})

    run_ops([
        {'action': 'create_object', 'obj_type': 'PartDesign::Body', 'obj_name': 'Body_Standoffs', 'description': 'Create Body_Standoffs'},
        {'action': 'sketch', 'obj_name': 'Standoffs_Sketch', 'obj_properties': {'plane': 'XY', 'offset': plate_t_bot, 'body': 'Body_Standoffs', 'geometry': standoff_geo}, 'description': 'Sketch 8 standoffs'},
        {'action': 'pad', 'obj_name': 'Standoffs_Sketch', 'obj_properties': {'length': standoff_h, 'body': 'Body_Standoffs'}, 'description': 'Pad standoffs 50mm'},
        {'action': 'color', 'obj_name': 'Body_Standoffs', 'obj_properties': {'color': [0.85, 0.20, 0.20]}, 'description': 'Anodized red aluminum'}
    ], "Body_Standoffs")

    # ==========================================================================
    # BODY 4A: 4x F60PROV Stator Bases & Copper Windings Core
    # ==========================================================================
    print("\n=== 4A. Building 4x F60PROV Stators & Copper Coils ===")
    stator_base_geo = []
    for mx, my in motor_positions:
        stator_base_geo.append({'type': 'circle', 'center': [round(mx, 3), round(my, 3)], 'radius': 8.5})
        for dx in [-8.0, 8.0]:
            for dy in [-8.0, 8.0]:
                stator_base_geo.append({'type': 'circle', 'center': [round(mx + dx, 3), round(my + dy, 3)], 'radius': 3.8})

    coils_geo = []
    for mx, my in motor_positions:
        coils_geo.append({'type': 'circle', 'center': [round(mx, 3), round(my, 3)], 'radius': 11.0})

    run_ops([
        {'action': 'create_object', 'obj_type': 'PartDesign::Body', 'obj_name': 'Body_F60_Stators', 'description': 'Create Body_F60_Stators'},
        {'action': 'sketch', 'obj_name': 'F60_Bases_Sketch', 'obj_properties': {'plane': 'XY', 'offset': plate_t_bot, 'body': 'Body_F60_Stators', 'geometry': stator_base_geo}, 'description': 'Sketch stator bases'},
        {'action': 'pad', 'obj_name': 'F60_Bases_Sketch', 'obj_properties': {'length': 2.5, 'body': 'Body_F60_Stators'}, 'description': 'Pad stator bases 2.5mm'},
        {'action': 'sketch', 'obj_name': 'F60_Copper_Coils_Sketch', 'obj_properties': {'plane': 'XY', 'offset': plate_t_bot + 2.5, 'body': 'Body_F60_Stators', 'geometry': coils_geo}, 'description': 'Sketch copper coils'},
        {'action': 'pad', 'obj_name': 'F60_Copper_Coils_Sketch', 'obj_properties': {'length': 11.0, 'body': 'Body_F60_Stators'}, 'description': 'Pad copper coils 11mm'},
        {'action': 'color', 'obj_name': 'Body_F60_Stators', 'obj_properties': {'color': [0.85, 0.45, 0.15]}, 'description': 'Copper color'}
    ], "Body_F60_Stators")

    # ==========================================================================
    # BODY 4B: 4x F60PROV 5-Spoke Rotor Bells & Shafts
    # ==========================================================================
    print("\n=== 4B. Building 4x F60PROV 5-Spoke Rotor Bells & Shafts ===")
    shells_geo = []
    for mx, my in motor_positions:
        shells_geo.append({'type': 'circle', 'center': [round(mx, 3), round(my, 3)], 'radius': 13.4})
        shells_geo.append({'type': 'circle', 'center': [round(mx, 3), round(my, 3)], 'radius': 11.8})

    spokes_geo = []
    r_in = 5.2
    r_out = 11.8
    half_w = math.radians(7.5)
    for mx, my in motor_positions:
        spokes_geo.append({'type': 'circle', 'center': [round(mx, 3), round(my, 3)], 'radius': 13.4})
        for k in range(5):
            ang_k = k * (2.0 * math.pi / 5.0)
            ang_next = (k + 1) * (2.0 * math.pi / 5.0)

            a1 = ang_k + half_w
            a2 = ang_next - half_w

            p1 = (round(mx + r_in * math.cos(a1), 3), round(my + r_in * math.sin(a1), 3))
            p2 = (round(mx + r_out * math.cos(a1), 3), round(my + r_out * math.sin(a1), 3))
            p3 = (round(mx + r_out * math.cos(a2), 3), round(my + r_out * math.sin(a2), 3))
            p4 = (round(mx + r_in * math.cos(a2), 3), round(my + r_in * math.sin(a2), 3))

            spokes_geo.append({'type': 'line', 'from': p1, 'to': p2})
            spokes_geo.append({'type': 'line', 'from': p2, 'to': p3})
            spokes_geo.append({'type': 'line', 'from': p3, 'to': p4})
            spokes_geo.append({'type': 'line', 'from': p4, 'to': p1})

    shafts_geo = []
    for mx, my in motor_positions:
        shafts_geo.append({'type': 'circle', 'center': [round(mx, 3), round(my, 3)], 'radius': 2.5})

    run_ops([
        {'action': 'create_object', 'obj_type': 'PartDesign::Body', 'obj_name': 'Body_F60_Rotors', 'description': 'Create Body_F60_Rotors'},
        {'action': 'sketch', 'obj_name': 'F60_Shells_Sketch', 'obj_properties': {'plane': 'XY', 'offset': plate_t_bot + 1.0, 'body': 'Body_F60_Rotors', 'geometry': shells_geo}, 'description': 'Sketch rotor shells'},
        {'action': 'pad', 'obj_name': 'F60_Shells_Sketch', 'obj_properties': {'length': 14.5, 'body': 'Body_F60_Rotors'}, 'description': 'Pad rotor shells 14.5mm'},
        {'action': 'sketch', 'obj_name': 'F60_Spokes_Sketch', 'obj_properties': {'plane': 'XY', 'offset': plate_t_bot + 15.5, 'body': 'Body_F60_Rotors', 'geometry': spokes_geo}, 'description': 'Sketch 5 spokes'},
        {'action': 'pad', 'obj_name': 'F60_Spokes_Sketch', 'obj_properties': {'length': 2.5, 'body': 'Body_F60_Rotors'}, 'description': 'Pad 5 spokes 2.5mm'},
        {'action': 'sketch', 'obj_name': 'F60_Shafts_Sketch', 'obj_properties': {'plane': 'XY', 'offset': plate_t_bot + 1.0, 'body': 'Body_F60_Rotors', 'geometry': shafts_geo}, 'description': 'Sketch shafts'},
        {'action': 'pad', 'obj_name': 'F60_Shafts_Sketch', 'obj_properties': {'length': 31.0, 'body': 'Body_F60_Rotors'}, 'description': 'Pad shafts 31mm'},
        {'action': 'color', 'obj_name': 'Body_F60_Rotors', 'obj_properties': {'color': [0.15, 0.40, 0.85]}, 'description': 'T-Motor metallic blue color'}
    ], "Body_F60_Rotors")

    # ==========================================================================
    # BODY 5: 4x Gemfan 51477 3-Blade Propellers (R=64.77mm, Cyan)
    # ==========================================================================
    print("\n=== 5. Building 4x Gemfan 51477 Propellers ===")
    prop_seat_z = plate_t_bot + 18.0
    hubs_geo = []
    for mx, my in motor_positions:
        hubs_geo.append({'type': 'circle', 'center': [round(mx, 3), round(my, 3)], 'radius': 6.5})
        hubs_geo.append({'type': 'circle', 'center': [round(mx, 3), round(my, 3)], 'radius': 2.55})

    blades_geo = []
    motor_coords = [
        (motor_xy, motor_xy, 0.0),
        (-motor_xy, motor_xy, math.pi / 2.0),
        (-motor_xy, -motor_xy, math.pi),
        (motor_xy, -motor_xy, -math.pi / 2.0)
    ]
    pts_local = [
        (6.0, -6.0),
        (6.0, 6.5),
        (35.0, 8.0),
        (62.0, 4.5),
        (64.77, 0.0),
        (62.0, -3.5),
        (35.0, -5.0)
    ]

    for mx, my, base_ang in motor_coords:
        for k in range(3):
            ang = base_ang + k * (2.0 * math.pi / 3.0)
            cos_a = math.cos(ang)
            sin_a = math.sin(ang)

            pts_world = []
            for u, v in pts_local:
                x = mx + (u * cos_a - v * sin_a)
                y = my + (u * sin_a + v * cos_a)
                pts_world.append((round(x, 3), round(y, 3)))

            for i in range(len(pts_world)):
                p_curr = pts_world[i]
                p_next = pts_world[(i + 1) % len(pts_world)]
                blades_geo.append({'type': 'line', 'from': p_curr, 'to': p_next})

    nuts_geo = []
    for mx, my, _ in motor_coords:
        nuts_geo.append({'type': 'circle', 'center': [round(mx, 3), round(my, 3)], 'radius': 4.0})

    run_ops([
        {'action': 'create_object', 'obj_type': 'PartDesign::Body', 'obj_name': 'Body_Props_51477', 'description': 'Create Body_Props_51477'},
        {'action': 'sketch', 'obj_name': 'Props_Hubs_Sketch', 'obj_properties': {'plane': 'XY', 'offset': prop_seat_z, 'body': 'Body_Props_51477', 'geometry': hubs_geo}, 'description': 'Sketch hubs'},
        {'action': 'pad', 'obj_name': 'Props_Hubs_Sketch', 'obj_properties': {'length': 7.0, 'body': 'Body_Props_51477'}, 'description': 'Pad hubs 7mm'},
        {'action': 'sketch', 'obj_name': 'Props_Blades_Sketch', 'obj_properties': {'plane': 'XY', 'offset': prop_seat_z + 1.5, 'body': 'Body_Props_51477', 'geometry': blades_geo}, 'description': 'Sketch blades'},
        {'action': 'pad', 'obj_name': 'Props_Blades_Sketch', 'obj_properties': {'length': 2.5, 'body': 'Body_Props_51477'}, 'description': 'Pad blades 2.5mm'},
        {'action': 'sketch', 'obj_name': 'Props_Nuts_Sketch', 'obj_properties': {'plane': 'XY', 'offset': prop_seat_z + 7.0, 'body': 'Body_Props_51477', 'geometry': nuts_geo}, 'description': 'Sketch nuts'},
        {'action': 'pad', 'obj_name': 'Props_Nuts_Sketch', 'obj_properties': {'length': 3.0, 'body': 'Body_Props_51477'}, 'description': 'Pad nuts 3mm'},
        {'action': 'color', 'obj_name': 'Body_Props_51477', 'obj_properties': {'color': [0.10, 0.80, 0.95], 'transparency': 15}, 'description': 'Clear cyan color'}
    ], "Body_Props_51477")

    # ==========================================================================
    # BODY 6: 4x M3 Brass Standoffs (H=3mm, D=5mm, Z=3.0 to 6.0mm) Propping Up PDB
    # ==========================================================================
    print("\n=== 6. Building 4x M3 Brass Standoffs (H=3mm) for PDB ===")
    standoffs_3mm = []
    for sx in [-15.25, 15.25]:
        for sy in [-15.25, 15.25]:
            standoffs_3mm.append({'type': 'circle', 'center': [sx, sy], 'radius': 2.5})

    run_ops([
        {'action': 'create_object', 'obj_type': 'PartDesign::Body', 'obj_name': 'Body_PDB_Standoffs_3mm', 'description': 'Create Body_PDB_Standoffs_3mm'},
        {'action': 'sketch', 'obj_name': 'PDB_Standoffs_3mm_Sketch', 'obj_properties': {'plane': 'XY', 'offset': 3.0, 'body': 'Body_PDB_Standoffs_3mm', 'geometry': standoffs_3mm}, 'description': 'Sketch 3mm standoffs'},
        {'action': 'pad', 'obj_name': 'PDB_Standoffs_3mm_Sketch', 'obj_properties': {'length': 3.0, 'body': 'Body_PDB_Standoffs_3mm'}, 'description': 'Pad 3mm standoffs'},
        {'action': 'color', 'obj_name': 'Body_PDB_Standoffs_3mm', 'obj_properties': {'color': [0.85, 0.68, 0.18]}, 'description': 'Polished brass gold'}
    ], "Body_PDB_Standoffs_3mm")

    # ==========================================================================
    # BODY 7: MATEKSYS FCHUB-12S V2 Power Distribution Board (PDB, Z=6.0 to 7.6mm)
    # 50x50mm Main Body with R4 Rounded Corners + 5mm Top Power Tab (Total 55mm)
    # ==========================================================================
    print("\n=== 7. Building MATEKSYS FCHUB-12S V2 PDB Board (Z=6.0 to 7.6mm) ===")
    pdb_pts = [
        # Top edge with DC-IN tab (width 18mm, height +5mm with center notch):
        (-9.0, 25.0), (-9.0, 30.0), (-3.5, 30.0), (-1.5, 27.5),
        (1.5, 27.5), (3.5, 30.0), (9.0, 30.0), (9.0, 25.0),
        # Top-Right rounded corner:
        (21.0, 25.0), (23.8, 23.8), (25.0, 21.0),
        # Right edge:
        (25.0, -21.0),
        # Bottom-Right rounded corner:
        (23.8, -23.8), (21.0, -25.0),
        # Bottom edge:
        (-21.0, -25.0),
        # Bottom-Left rounded corner:
        (-23.8, -23.8), (-25.0, -21.0),
        # Left edge:
        (-25.0, 21.0),
        # Top-Left rounded corner:
        (-23.8, 23.8), (-21.0, 25.0)
    ]
    pdb_geo = []
    for i in range(len(pdb_pts)):
        pdb_geo.append({'type': 'line', 'from': pdb_pts[i], 'to': pdb_pts[(i+1)%len(pdb_pts)]})

    # 4 Standard 30.5x30.5mm M3 mounting holes
    for sx in [-15.25, 15.25]:
        for sy in [-15.25, 15.25]:
            pdb_geo.append({'type': 'circle', 'center': [sx, sy], 'radius': 1.6})

    run_ops([
        {'action': 'create_object', 'obj_type': 'PartDesign::Body', 'obj_name': 'Body_Mateksys_PDB', 'description': 'Create Body_Mateksys_PDB'},
        {'action': 'sketch', 'obj_name': 'Mateksys_PDB_Sketch', 'obj_properties': {'plane': 'XY', 'offset': 6.0, 'body': 'Body_Mateksys_PDB', 'geometry': pdb_geo}, 'description': 'Sketch Mateksys PDB'},
        {'action': 'pad', 'obj_name': 'Mateksys_PDB_Sketch', 'obj_properties': {'length': 1.6, 'body': 'Body_Mateksys_PDB'}, 'description': 'Pad Mateksys PDB 1.6mm'},
        {'action': 'color', 'obj_name': 'Body_Mateksys_PDB', 'obj_properties': {'color': [0.08, 0.25, 0.65]}, 'description': 'Mateksys Royal Blue PCB'}
    ], "Body_Mateksys_PDB")

    # ==========================================================================
    # BODY 8: Mateksys FCHUB-12S V2 Detailed Solder Pads & DC-IN Tabs (Gold)
    # ==========================================================================
    print("\n=== 8. Building Mateksys PDB Gold Solder Pads & Terminals ===")
    pads_geo = []
    # 1. Dual DC-IN Power Lugs (Left: Negative, Right: Positive)
    dc_pads = [
        ((-8.0, 25.5), (-2.0, 25.5), (-2.0, 29.5), (-8.0, 29.5)),
        ((2.0, 25.5), (8.0, 25.5), (8.0, 29.5), (2.0, 29.5))
    ]
    for quad in dc_pads:
        for i in range(4):
            pads_geo.append({'type': 'line', 'from': quad[i], 'to': quad[(i+1)%4]})

    # 2. Main Copper Current Bus (center-top bar)
    bus_quad = [(-12.0, 11.5), (12.0, 11.5), (12.0, 14.5), (-12.0, 14.5)]
    for i in range(4):
        pads_geo.append({'type': 'line', 'from': bus_quad[i], 'to': bus_quad[(i+1)%4]})

    # 3. 4x M3 Annular Ground Rings around mounting holes
    for sx in [-15.25, 15.25]:
        for sy in [-15.25, 15.25]:
            pads_geo.append({'type': 'circle', 'center': [sx, sy], 'radius': 2.9})
            pads_geo.append({'type': 'circle', 'center': [sx, sy], 'radius': 1.65})

    # 4. 4 ESC Corner Power & Signal Pads
    esc_pads = [
        (-22.0, 18.0), (-18.0, 22.0), # ESC1
        (-22.0, -18.0), (-18.0, -22.0), # ESC2
        (22.0, 18.0), (18.0, 22.0), # ESC3
        (22.0, -18.0), (18.0, -22.0)  # ESC4
    ]
    for ex, ey in esc_pads:
        pads_geo.append({'type': 'circle', 'center': [ex, ey], 'radius': 1.2})

    run_ops([
        {'action': 'create_object', 'obj_type': 'PartDesign::Body', 'obj_name': 'Body_PDB_Copper_Pads', 'description': 'Create Body_PDB_Copper_Pads'},
        {'action': 'sketch', 'obj_name': 'PDB_Pads_Sketch', 'obj_properties': {'plane': 'XY', 'offset': 7.6, 'body': 'Body_PDB_Copper_Pads', 'geometry': pads_geo}, 'description': 'Sketch gold pads'},
        {'action': 'pad', 'obj_name': 'PDB_Pads_Sketch', 'obj_properties': {'length': 0.25, 'body': 'Body_PDB_Copper_Pads'}, 'description': 'Pad gold pads 0.25mm'},
        {'action': 'color', 'obj_name': 'Body_PDB_Copper_Pads', 'obj_properties': {'color': [0.90, 0.72, 0.20]}, 'description': 'ENIG Immersion Gold color'}
    ], "Body_PDB_Copper_Pads")

    # ==========================================================================
    # BODY 9: Mateksys FCHUB-12S V2 SMD Components (Inductors, Shunts, Connector)
    # ==========================================================================
    print("\n=== 9. Building Mateksys PDB SMD Components ===")
    components_geo = []
    # 2x Molded Power Inductors ("100")
    ind1 = [(-18.5, 1.0), (-9.5, 1.0), (-9.5, 10.0), (-18.5, 10.0)]
    for i in range(4):
        components_geo.append({'type': 'line', 'from': ind1[i], 'to': ind1[(i+1)%4]})
    ind2 = [(7.5, -9.5), (16.5, -9.5), (16.5, -0.5), (7.5, -0.5)]
    for i in range(4):
        components_geo.append({'type': 'line', 'from': ind2[i], 'to': ind2[(i+1)%4]})

    # 2x Current Shunt Resistors ("0m30", "0m50")
    sh1 = [(-6.5, 16.0), (-0.8, 16.0), (-0.8, 19.2), (-6.5, 19.2)]
    for i in range(4):
        components_geo.append({'type': 'line', 'from': sh1[i], 'to': sh1[(i+1)%4]})
    sh2 = [(0.8, 16.0), (6.5, 16.0), (6.5, 19.2), (0.8, 19.2)]
    for i in range(4):
        components_geo.append({'type': 'line', 'from': sh2[i], 'to': sh2[(i+1)%4]})

    # 1x JST-GH / Ribbon Cable Interconnect Socket (bottom center)
    conn = [(-6.5, -16.5), (6.5, -16.5), (6.5, -12.5), (-6.5, -12.5)]
    for i in range(4):
        components_geo.append({'type': 'line', 'from': conn[i], 'to': conn[(i+1)%4]})

    run_ops([
        {'action': 'create_object', 'obj_type': 'PartDesign::Body', 'obj_name': 'Body_PDB_Components', 'description': 'Create Body_PDB_Components'},
        {'action': 'sketch', 'obj_name': 'PDB_Components_Sketch', 'obj_properties': {'plane': 'XY', 'offset': 7.6, 'body': 'Body_PDB_Components', 'geometry': components_geo}, 'description': 'Sketch PDB components'},
        {'action': 'pad', 'obj_name': 'PDB_Components_Sketch', 'obj_properties': {'length': 2.0, 'body': 'Body_PDB_Components'}, 'description': 'Pad components 2.0mm'},
        {'action': 'color', 'obj_name': 'Body_PDB_Components', 'obj_properties': {'color': [0.28, 0.30, 0.32]}, 'description': 'Component metallic gray'}
    ], "Body_PDB_Components")

    # ==========================================================================
    # BODY 10: 4x M3 Low-Profile Fasteners for PDB (NO COPPER STANDOFFS ON TOP)
    # ==========================================================================
    print("\n=== 10. Building 4x M3 PDB Fasteners (No standoffs on PDB) ===")
    pdb_fasteners_geo = []
    for sx in [-15.25, 15.25]:
        for sy in [-15.25, 15.25]:
            pdb_fasteners_geo.append({'type': 'circle', 'center': [sx, sy], 'radius': 2.75})

    run_ops([
        {'action': 'create_object', 'obj_type': 'PartDesign::Body', 'obj_name': 'Body_PDB_Fasteners', 'description': 'Create Body_PDB_Fasteners'},
        {'action': 'sketch', 'obj_name': 'PDB_Fasteners_Sketch', 'obj_properties': {'plane': 'XY', 'offset': 7.6, 'body': 'Body_PDB_Fasteners', 'geometry': pdb_fasteners_geo}, 'description': 'Sketch PDB fastener nuts'},
        {'action': 'pad', 'obj_name': 'PDB_Fasteners_Sketch', 'obj_properties': {'length': 1.6, 'body': 'Body_PDB_Fasteners'}, 'description': 'Pad PDB fastener nuts 1.6mm'},
        {'action': 'color', 'obj_name': 'Body_PDB_Fasteners', 'obj_properties': {'color': [0.75, 0.78, 0.82]}, 'description': 'Stainless steel nickel color'}
    ], "Body_PDB_Fasteners")

    # ==========================================================================
    # BODY 11: 4x M3 Brass Standoffs (H=10mm, D=5mm, Z=3.0 to 13.0mm)
    # CONNECTING MOTOR PLATE (电机板子) DIRECTLY TO FLIGHT CONTROLLER (飞控底板)
    # ==========================================================================
    print("\n=== 11. Building 4x M3 Brass Standoffs (H=10mm) Connecting Motor Plate to FC ===")
    standoffs_10mm = []
    for sx in [-36.0, 36.0]:
        for sy in [-12.0, 12.0]:
            standoffs_10mm.append({'type': 'circle', 'center': [sx, sy], 'radius': 2.5})

    run_ops([
        {'action': 'create_object', 'obj_type': 'PartDesign::Body', 'obj_name': 'Body_FC_Standoffs_10mm', 'description': 'Create Body_FC_Standoffs_10mm'},
        {'action': 'sketch', 'obj_name': 'FC_Standoffs_10mm_Sketch', 'obj_properties': {'plane': 'XY', 'offset': 3.0, 'body': 'Body_FC_Standoffs_10mm', 'geometry': standoffs_10mm}, 'description': 'Sketch 10mm standoffs'},
        {'action': 'pad', 'obj_name': 'FC_Standoffs_10mm_Sketch', 'obj_properties': {'length': 10.0, 'body': 'Body_FC_Standoffs_10mm'}, 'description': 'Pad 10mm standoffs'},
        {'action': 'color', 'obj_name': 'Body_FC_Standoffs_10mm', 'obj_properties': {'color': [0.85, 0.68, 0.18]}, 'description': 'Polished brass gold'}
    ], "Body_FC_Standoffs_10mm")

    # ==========================================================================
    # BODY 12: Flight Controller Deck Plate (96 x 54 mm, T=2mm, Z=13.0 to 15.0mm)
    # Connected Directly to 10mm Standoffs, with 44x44mm Central Window for PDB
    # ==========================================================================
    print("\n=== 12. Building Flight Controller Base Plate (Connected via 10mm Standoffs) ===")
    fc_outer = [
        (44.0, 27.0), (48.0, 23.0),
        (48.0, -23.0), (44.0, -27.0),
        (-44.0, -27.0), (-48.0, -23.0),
        (-48.0, 23.0), (-44.0, 27.0)
    ]
    # Central clearance window over PDB (44x44mm)
    fc_window = [
        (22.0, 22.0), (-22.0, 22.0),
        (-22.0, -22.0), (22.0, -22.0)
    ]
    fc_geo = []
    for i in range(len(fc_outer)):
        fc_geo.append({'type': 'line', 'from': fc_outer[i], 'to': fc_outer[(i+1)%len(fc_outer)]})
    for i in range(len(fc_window)):
        fc_geo.append({'type': 'line', 'from': fc_window[i], 'to': fc_window[(i+1)%len(fc_window)]})

    # 4 Mounting holes mating directly with the 10mm standoffs from the motor plate
    for sx in [-36.0, 36.0]:
        for sy in [-12.0, 12.0]:
            fc_geo.append({'type': 'circle', 'center': [sx, sy], 'radius': 1.6})

    run_ops([
        {'action': 'create_object', 'obj_type': 'PartDesign::Body', 'obj_name': 'Body_FC_Plate', 'description': 'Create Body_FC_Plate'},
        {'action': 'sketch', 'obj_name': 'FC_Plate_Sketch', 'obj_properties': {'plane': 'XY', 'offset': 13.0, 'body': 'Body_FC_Plate', 'geometry': fc_geo}, 'description': 'Sketch FC deck plate'},
        {'action': 'pad', 'obj_name': 'FC_Plate_Sketch', 'obj_properties': {'length': 2.0, 'body': 'Body_FC_Plate'}, 'description': 'Pad FC deck plate 2mm'},
        {'action': 'color', 'obj_name': 'Body_FC_Plate', 'obj_properties': {'color': [0.15, 0.15, 0.16]}, 'description': 'Carbon black color'}
    ], "Body_FC_Plate")

    # ==========================================================================
    # BODY 13: CUAV Pixhawk V6X Flight Controller (Z=15.0 to 44.2mm)
    # ==========================================================================
    print("\n=== 13. Building CUAV Pixhawk V6X Flight Controller ===")
    v6x_lx = 85.1 / 2.0
    v6x_wy = 45.0 / 2.0
    v6x_base_geo = [
        {'type': 'line', 'from': [-v6x_lx, -v6x_wy], 'to': [v6x_lx, -v6x_wy]},
        {'type': 'line', 'from': [v6x_lx, -v6x_wy], 'to': [v6x_lx, v6x_wy]},
        {'type': 'line', 'from': [v6x_lx, v6x_wy], 'to': [-v6x_lx, v6x_wy]},
        {'type': 'line', 'from': [-v6x_lx, v6x_wy], 'to': [-v6x_lx, -v6x_wy]}
    ]
    imu_x1 = 22.25 - 38.65
    imu_x2 = 22.25
    imu_y = 31.65 / 2.0
    v6x_imu_geo = [
        {'type': 'line', 'from': [imu_x1, -imu_y], 'to': [imu_x2, -imu_y]},
        {'type': 'line', 'from': [imu_x2, -imu_y], 'to': [imu_x2, imu_y]},
        {'type': 'line', 'from': [imu_x2, imu_y], 'to': [imu_x1, imu_y]},
        {'type': 'line', 'from': [imu_x1, imu_y], 'to': [imu_x1, -imu_y]}
    ]
    run_ops([
        {'action': 'create_object', 'obj_type': 'PartDesign::Body', 'obj_name': 'Body_Pixhawk_V6X', 'description': 'Create Body_Pixhawk_V6X'},
        {'action': 'sketch', 'obj_name': 'V6X_Baseboard_Sketch', 'obj_properties': {'plane': 'XY', 'offset': 15.0, 'body': 'Body_Pixhawk_V6X', 'geometry': v6x_base_geo}, 'description': 'Sketch V6X base'},
        {'action': 'pad', 'obj_name': 'V6X_Baseboard_Sketch', 'obj_properties': {'length': 12.5, 'body': 'Body_Pixhawk_V6X'}, 'description': 'Pad V6X base 12.5mm'},
        {'action': 'sketch', 'obj_name': 'V6X_IMU_Core_Sketch', 'obj_properties': {'plane': 'XY', 'offset': 15.0 + 12.5, 'body': 'Body_Pixhawk_V6X', 'geometry': v6x_imu_geo}, 'description': 'Sketch IMU cube'},
        {'action': 'pad', 'obj_name': 'V6X_IMU_Core_Sketch', 'obj_properties': {'length': 16.7, 'body': 'Body_Pixhawk_V6X'}, 'description': 'Pad IMU cube 16.7mm'},
        {'action': 'color', 'obj_name': 'Body_Pixhawk_V6X', 'obj_properties': {'color': [0.20, 0.22, 0.25]}, 'description': 'CUAV Space Gray color'}
    ], "Body_Pixhawk_V6X")

    # ==========================================================================
    # BODY 14: 4x Brass Battery Standoffs (H=30mm, D=5mm, linking chassis to lower plate)
    # ==========================================================================
    print("\n=== 14. Building 4x Brass Battery Standoffs (H=30mm) ===")
    bat_standoffs_geo = []
    for sx in [-48.0, 48.0]:
        for sy in [-26.0, 26.0]:
            bat_standoffs_geo.append({'type': 'circle', 'center': [sx, sy], 'radius': 2.5})

    run_ops([
        {'action': 'create_object', 'obj_type': 'PartDesign::Body', 'obj_name': 'Body_Battery_Standoffs', 'description': 'Create Body_Battery_Standoffs'},
        {'action': 'sketch', 'obj_name': 'Bat_Standoffs_Sketch', 'obj_properties': {'plane': 'XY', 'offset': -30.0, 'body': 'Body_Battery_Standoffs', 'geometry': bat_standoffs_geo}, 'description': 'Sketch battery brass standoffs'},
        {'action': 'pad', 'obj_name': 'Bat_Standoffs_Sketch', 'obj_properties': {'length': 30.0, 'body': 'Body_Battery_Standoffs'}, 'description': 'Pad brass standoffs 30mm'},
        {'action': 'color', 'obj_name': 'Body_Battery_Standoffs', 'obj_properties': {'color': [0.85, 0.65, 0.15]}, 'description': 'Polished brass gold color'}
    ], "Body_Battery_Standoffs")

    # ==========================================================================
    # BODY 15: Lower Battery Tray Plate (2mm Carbon, 146 x 56 mm at Z = -32 to -30 mm)
    # ==========================================================================
    print("\n=== 15. Building Lower Battery Tray Plate ===")
    bat_plate_geo = [
        {'type': 'line', 'from': [-73.0, -28.0], 'to': [73.0, -28.0]},
        {'type': 'line', 'from': [73.0, -28.0], 'to': [73.0, 28.0]},
        {'type': 'line', 'from': [73.0, 28.0], 'to': [-73.0, 28.0]},
        {'type': 'line', 'from': [-73.0, 28.0], 'to': [-73.0, -28.0]}
    ]
    for sx in [-48.0, 48.0]:
        for sy in [-26.0, 26.0]:
            bat_plate_geo.append({'type': 'circle', 'center': [sx, sy], 'radius': 1.6})

    for y_slot in [-18.0, 18.0]:
        for x_slot in [-30.0, 30.0]:
            bat_plate_geo.append({'type': 'circle', 'center': [x_slot, y_slot], 'radius': 2.0})

    run_ops([
        {'action': 'create_object', 'obj_type': 'PartDesign::Body', 'obj_name': 'Body_Battery_Plate', 'description': 'Create Body_Battery_Plate'},
        {'action': 'sketch', 'obj_name': 'Bat_Plate_Sketch', 'obj_properties': {'plane': 'XY', 'offset': -32.0, 'body': 'Body_Battery_Plate', 'geometry': bat_plate_geo}, 'description': 'Sketch lower battery plate'},
        {'action': 'pad', 'obj_name': 'Bat_Plate_Sketch', 'obj_properties': {'length': 2.0, 'body': 'Body_Battery_Plate'}, 'description': 'Pad lower battery plate 2mm'},
        {'action': 'color', 'obj_name': 'Body_Battery_Plate', 'obj_properties': {'color': [0.15, 0.15, 0.16]}, 'description': 'Carbon black color'}
    ], "Body_Battery_Plate")

    # ==========================================================================
    # BODY 16: 4S LiPo Battery (136 x 43.5 x 29 mm, Cradled between Plates)
    # ==========================================================================
    print("\n=== 16. Building 4S LiPo Battery ===")
    bx_half = 136.0 / 2.0
    by_half = 43.5 / 2.0
    bat_geo = [
        {'type': 'line', 'from': [-bx_half, -by_half], 'to': [bx_half, -by_half]},
        {'type': 'line', 'from': [bx_half, -by_half], 'to': [bx_half, by_half]},
        {'type': 'line', 'from': [bx_half, by_half], 'to': [-bx_half, by_half]},
        {'type': 'line', 'from': [-bx_half, by_half], 'to': [-bx_half, -by_half]}
    ]

    run_ops([
        {'action': 'create_object', 'obj_type': 'PartDesign::Body', 'obj_name': 'Body_Battery_4S', 'description': 'Create Body_Battery_4S'},
        {'action': 'sketch', 'obj_name': 'Battery_Sketch', 'obj_properties': {'plane': 'XY', 'offset': -29.5, 'body': 'Body_Battery_4S', 'geometry': bat_geo}, 'description': 'Sketch 4S battery outline'},
        {'action': 'pad', 'obj_name': 'Battery_Sketch', 'obj_properties': {'length': 29.0, 'body': 'Body_Battery_4S'}, 'description': 'Pad 4S battery 29mm'},
        {'action': 'color', 'obj_name': 'Body_Battery_4S', 'obj_properties': {'color': [0.95, 0.75, 0.10]}, 'description': 'LiPo yellow gold color'}
    ], "Body_Battery_4S")

    # ==========================================================================
    # BODY 17: Integrated Perimeter Docking Mechanism
    # ==========================================================================
    print("\n=== 17. Building Integrated Perimeter Docking Mechanism ===")
    bracket_geo = [
        {'type': 'circle', 'center': [0.0, 180.0], 'radius': 6.0},
        {'type': 'circle', 'center': [0.0, 180.0], 'radius': 2.5},
        {'type': 'circle', 'center': [0.0, -180.0], 'radius': 6.0},
        {'type': 'circle', 'center': [0.0, -180.0], 'radius': 2.5},
        {'type': 'circle', 'center': [180.0, 0.0], 'radius': 6.0},
        {'type': 'circle', 'center': [180.0, 0.0], 'radius': 2.5},
        {'type': 'circle', 'center': [-180.0, 0.0], 'radius': 6.0},
        {'type': 'circle', 'center': [-180.0, 0.0], 'radius': 2.5}
    ]

    run_ops([
        {'action': 'create_object', 'obj_type': 'PartDesign::Body', 'obj_name': 'Body_Docking_Mechanism', 'description': 'Create Body_Docking_Mechanism'},
        {'action': 'sketch', 'obj_name': 'Docking_Bracket_Sketch', 'obj_properties': {'plane': 'XY', 'offset': 20.0, 'body': 'Body_Docking_Mechanism', 'geometry': bracket_geo}, 'description': 'Sketch docking brackets'},
        {'action': 'pad', 'obj_name': 'Docking_Bracket_Sketch', 'obj_properties': {'length': 15.0, 'body': 'Body_Docking_Mechanism'}, 'description': 'Pad docking brackets 15mm'},
        {'action': 'color', 'obj_name': 'Body_Docking_Mechanism', 'obj_properties': {'color': [0.85, 0.85, 0.90]}, 'description': 'Metallic silver brackets'}
    ], "Body_Docking_Mechanism")

    print("\nAll 17 bodies constructed successfully!")

    # ==========================================================================
    # SAVE AND EXPORT
    # ==========================================================================
    fcstd_path = "/home/ubuntu22/Downloads/dual_quad_codex_sim/models/Machinable_Docking_UAV_300.FCStd"
    step_path = "/home/ubuntu22/Downloads/dual_quad_codex_sim/models/Machinable_Docking_UAV_300.step"

    print(f"\nSaving native project to {fcstd_path}...")
    save_res = proxy.save_document(DOC, fcstd_path)
    print("Save result:", save_res)

    print("\nExporting standard STEP exchange file...")
    freecadcmd_bins = glob.glob('/tmp/.mount_FreeCA*/usr/bin/freecadcmd')
    if freecadcmd_bins:
        cmd = [
            freecadcmd_bins[0],
            '-c',
            f"""
import FreeCAD, Import
doc = FreeCAD.openDocument('{fcstd_path}')
bodies = [obj for obj in doc.Objects if obj.isDerivedFrom('PartDesign::Body')]
Import.export(bodies, '{step_path}')
print('Exported', len(bodies), 'bodies to STEP successfully')
"""
        ]
        res = subprocess.run(cmd, capture_output=True, text=True)
        print("STEP Export log:", res.stdout)
        if res.stderr:
            print("STEP Export err:", res.stderr)

    print("\n=== BUILD PROCESS COMPLETED SUCCESSFULLY ===")

if __name__ == '__main__':
    main()
