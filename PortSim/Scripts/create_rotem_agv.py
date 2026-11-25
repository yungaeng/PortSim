"""Blender --background --python create_rotem_agv.py. Dimensions are PortSim cm.

Reference-inspired exterior, not a manufacturer CAD model. The simulation keeps
its original deck collision, front/rear wheel pivots and handover coordinates.
"""
import bpy
import bmesh
import math
from pathlib import Path
from mathutils import Matrix

OUT = Path(__file__).resolve().parents[1] / 'Content/PortSim/Assets/SourceFiles/RotemAGV'
OUT.mkdir(parents=True, exist_ok=True)
bpy.ops.object.select_all(action='SELECT')
bpy.ops.object.delete(use_global=False)
bpy.context.preferences.filepaths.save_version = 0

def material(name, rgb, metal=0, rough=.4):
    m = bpy.data.materials.new('AGV_' + name)
    m.diffuse_color = (*rgb, 1)
    m.use_nodes = True
    p = m.node_tree.nodes.get('Principled BSDF')
    p.inputs['Base Color'].default_value = (*rgb, 1)
    p.inputs['Metallic'].default_value = metal
    p.inputs['Roughness'].default_value = rough
    return m

blue = material('RotemBlue', (.012,.19,.62), .35, .32)
dark = material('Underbody', (.023,.032,.042), .45, .55)
rubber = material('TireRubber', (.018,.021,.025), 0, .82)
silver = material('MachinedAlloy', (.48,.54,.60), .8, .28)
white = material('Lettering', (.92,.95,.98), .1, .38)
yellow = material('SafetyYellow', (.98,.69,.015), .15, .4)
black = material('VentBlack', (.006,.009,.013), .1, .62)
red = material('RedLamp', (.65,.012,.009), .15, .25)
lamp = material('Headlamp', (.85,.92,1), .45, .2)

def finish(obj, mat, bevel=0):
    obj.data.materials.append(mat)
    if bevel:
        mod = obj.modifiers.new('Manufactured edge radii', 'BEVEL')
        mod.width = bevel / 100
        mod.segments = 3
        bpy.context.view_layer.objects.active = obj
        bpy.ops.object.modifier_apply(modifier=mod.name)
    return obj

def box(name, pos, size, mat, bevel=1.5):
    bpy.ops.mesh.primitive_cube_add(size=1, location=[v/100 for v in pos])
    obj = bpy.context.object
    obj.name = name
    obj.dimensions = [v/100 for v in size]
    bpy.ops.object.transform_apply(location=False, rotation=False, scale=True)
    return finish(obj, mat, bevel)

def cylinder(name, pos, radius, depth, mat, axis='X', vertices=48):
    bpy.ops.mesh.primitive_cylinder_add(vertices=vertices, radius=radius/100,
        depth=depth/100, location=[v/100 for v in pos])
    obj = bpy.context.object
    obj.name = name
    if axis == 'X': obj.rotation_euler[1] = math.pi/2
    elif axis == 'Y': obj.rotation_euler[0] = math.pi/2
    return finish(obj, mat, .6)

def text(name, value, pos, height, right, normal):
    curve = bpy.data.curves.new(name, 'FONT')
    curve.body = value
    curve.align_x = 'CENTER'
    curve.align_y = 'CENTER'
    curve.size = height/100
    curve.extrude = .0004
    obj = bpy.data.objects.new(name, curve)
    bpy.context.collection.objects.link(obj)
    obj.location = [v/100 for v in pos]
    obj.rotation_euler = Matrix((right,(0,0,1),normal)).transposed().to_euler()
    curve.materials.append(white)
    bpy.context.view_layer.objects.active = obj
    obj.select_set(True)
    bpy.ops.object.convert(target='MESH')
    obj.select_set(False)

def export(name):
    bpy.ops.object.select_all(action='SELECT')
    bpy.context.view_layer.objects.active = bpy.context.selected_objects[0]
    bpy.ops.object.convert(target='MESH')
    bpy.ops.object.join()
    obj = bpy.context.object
    obj.name = name
    bpy.context.scene.cursor.location = (0,0,0)
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR')
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    if name.endswith('_Tire') or name.endswith('_Hub'):
        # Match the original cylinder's axial envelope exactly, including tread
        # and hub bolts; the existing component scales remain authoritative.
        axial_extent = max(abs(v.co.z) for v in obj.data.vertices)
        for v in obj.data.vertices: v.co.z *= .5/axial_extent
    bm = bmesh.new()
    bm.from_mesh(obj.data)
    bmesh.ops.recalc_face_normals(bm, faces=list(bm.faces))
    bm.to_mesh(obj.data)
    bm.free()
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.mesh.select_all(action='SELECT')
    bpy.ops.uv.cube_project(cube_size=1)
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.ops.wm.save_as_mainfile(filepath=str(OUT / (name+'.blend')))
    bpy.ops.export_scene.fbx(filepath=str(OUT / (name+'.fbx')), use_selection=True,
        object_types={'MESH'}, add_leaf_bones=False, bake_anim=False,
        axis_forward='-Y', axis_up='Z', use_mesh_modifiers=True, mesh_smooth_type='FACE')
    bpy.ops.object.delete(use_global=False)

# One combined static body mesh; no extra ticking or per-panel components.
box('Load bearing deck', (0,0,198), (300,1380,44), dark, 3)
for side in (-1,1):
    x = side*173
    box('Continuous blue upper rail', (x,0,201), (44,1380,38), blue, 3)
    box('Rail top wear strip', (x,0,219), (30,1320,2), silver, .5)
    # Openings follow the front/rear axle centres, including steering clearance.
    for axle in (-480,480):
        # A curved annular fender, extruded across the side of the chassis.
        verts, faces = [], []
        for px in (side*163, side*203):
            for radius in (104,119):
                for i in range(17):
                    a = math.radians(18+144*i/16)
                    verts.append((px/100,(axle+radius*math.cos(a))/100,(75+radius*math.sin(a))/100))
        for i in range(16):
            for a,b in ((0,17),(34,51),(0,34),(17,51)):
                faces.append((a+i,a+i+1,b+i+1,b+i))
        faces.extend([(0,17,51,34),(16,50,67,33)])
        mesh=bpy.data.meshes.new('Arched fender')
        mesh.from_pydata(verts,[],faces)
        mesh.update()
        obj=bpy.data.objects.new('Blue wheel arch',mesh)
        bpy.context.collection.objects.link(obj)
        finish(obj,blue,1)
    for y in (-180,180):
        box('Battery cabinet', (side*163,y,112), (60,344,95), dark, 3)
        box('Blue service door', (side*195,y,115), (5,317,85), blue, 2)
        for edge in (-1,1):
            box('Door edge trim', (side*198,y+edge*158,115), (2,3,85), silver, .3)
            for z in (88,142):
                cylinder('Recessed latch', (side*200,y+edge*144,z), 3.5, 1.2, silver)
        box('Door handle', (side*200,y+75,117), (2,3,12), black, .4)
        box('Warning plate', (side*200,y-80,128), (1,7,10), yellow, .2)
    for end in (-1,1):
        y=end*648
        box('End corner tower', (side*171,y,141), (57,106,156), blue, 5)
        for z in (87,109,131,153):
            box('Recessed cooling slot', (side*200,y,z), (1.8,67,10), black, .8)
            box('Vent lower lip', (side*201,y,z-5), (2,67,2), silver, .2)
        text('Manufacturer', 'HYUNDAI', (side*202,y,191), 7,
            (0,side,0),(side,0,0))
        text('Rotem wordmark', 'Rotem', (side*202,y,177), 14,
            (0,side,0),(side,0,0))
    text('Operator lettering','Dongwon Global Terminal Busan', (side*197,0,201), 9,
        (0,side,0),(side,0,0))
    for y in (-347,347):
        box('Amber side reflector',(side*205,y,173),(2,12,6),yellow,.8)

for end in (-1,1):
    box('End fascia', (0,end*692,157), (294,30,122), blue, 5)
    box('Raised end shoulder', (0,end*678,232), (255,55,24), blue, 3)
    box('Rubber bumper backing', (0,end*709,74), (360,24,35), dark, 2)
    box('Yellow safety bumper', (0,end*722,81), (366,10,29), yellow, 1)
    for x in range(-162,180,45):
        obj=box('Diagonal hazard stripe',(x,end*728,81),(19,1,28),black,0)
        # Slant in the end face, staying within the bumper height.
        for v in obj.data.vertices: v.co.x += v.co.z*.4
    box('LiDAR recess',(0,end*709,180),(76,4,30),black,2)
    cylinder('LiDAR scanner',(0,end*714,180),11,8,dark,'Y')
    for x in (-118,118):
        cylinder('Headlamp bezel',(x,end*710,144),9,4,dark,'Y')
        cylinder('Headlamp lens',(x,end*713,144),6.6,2,lamp,'Y')
        cylinder('Marker lamp',(x,end*712,116),3.7,2,red,'Y')
    text('End operator','Dongwon',(0,end*711,210),16,(-end,0,0),(0,end,0))
    for x in (-130,130):
        box('Container landing pad',(x,end*590,219),(34,44,2),silver,.6)
box('Underbody keel',(0,0,85),(180,1180,38),dark,4)
export('SM_RotemAGV_Body')

# Replacement meshes use the same 100 cm cylinder envelope and local Z axle as
# the original engine cylinder. Component transforms and animation stay intact.
bpy.ops.mesh.primitive_cylinder_add(vertices=64, radius=.5, depth=1)
tire=bpy.context.object
tire.name='Rounded industrial tire'
finish(tire,rubber,7)
for end in (-1,1):
    bpy.ops.mesh.primitive_torus_add(major_radius=.385,minor_radius=.018,
        major_segments=64,minor_segments=8,location=(0,0,end*.493))
    finish(bpy.context.object,rubber)
for i in range(48):
    a=2*math.pi*i/48
    tread=box('Tread block',(49*math.cos(a),49*math.sin(a),0),(1.8,3.8,65),rubber,.4)
    tread.rotation_euler.z=a
export('SM_RotemAGV_Tire')

cylinder('Alloy rim',(0,0,0),50,100,silver,'Z',64)
for end in (-1,1):
    cylinder('Dark rim dish',(0,0,end*50),39,2,dark,'Z',64)
    cylinder('Hub cap',(0,0,end*52),24,4,silver,'Z',48)
    for i in range(10):
        a=2*math.pi*i/10
        cylinder('Wheel nut',(30*math.cos(a),30*math.sin(a),end*52),3,4,silver,'Z',6)
        cylinder('Cooling hole',(43*math.cos(a),43*math.sin(a),end*51.5),3,1,black,'Z',12)
export('SM_RotemAGV_Hub')
print('PORTSIM_ROTEM_SOURCE_PASS')
