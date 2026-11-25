"""UE 5.6 commandlet: -EnablePlugins=PythonScriptPlugin,EditorScriptingUtilities
-run=pythonscript -script=<this file>. Rebuilds only dedicated AGV assets.
"""
import unreal
import json
from pathlib import Path

root = Path(unreal.Paths.project_dir())
folder = '/Game/PortSim/Assets/Models/RotemAGV'
tools = unreal.AssetToolsHelpers.get_asset_tools()
palette = {
    'RotemBlue': ((.012,.19,.62),.35,.32),
    'Underbody': ((.023,.032,.042),.45,.55),
    'TireRubber': ((.018,.021,.025),0,.82),
    'MachinedAlloy': ((.48,.54,.60),.8,.28),
    'Lettering': ((.92,.95,.98),.1,.38),
    'SafetyYellow': ((.98,.69,.015),.15,.4),
    'VentBlack': ((.006,.009,.013),.1,.62),
    'RedLamp': ((.65,.012,.009),.15,.25),
    'Headlamp': ((.85,.92,1),.45,.2),
}
materials = {}
for name,(color,metal,rough) in palette.items():
    asset_name = 'M_AGV_' + name
    path = folder + '/' + asset_name
    if unreal.EditorAssetLibrary.does_asset_exist(path):
        materials['AGV_'+name] = unreal.load_asset(path)
        continue
    mat = tools.create_asset(asset_name,folder,unreal.Material,unreal.MaterialFactoryNew())
    node = unreal.MaterialEditingLibrary.create_material_expression(mat,unreal.MaterialExpressionConstant3Vector)
    node.set_editor_property('constant',unreal.LinearColor(*color,1))
    unreal.MaterialEditingLibrary.connect_material_property(node,'',unreal.MaterialProperty.MP_BASE_COLOR)
    for value,prop in ((metal,unreal.MaterialProperty.MP_METALLIC),(rough,unreal.MaterialProperty.MP_ROUGHNESS)):
        scalar = unreal.MaterialEditingLibrary.create_material_expression(mat,unreal.MaterialExpressionConstant)
        scalar.set_editor_property('r',value)
        unreal.MaterialEditingLibrary.connect_material_property(scalar,'',prop)
    unreal.MaterialEditingLibrary.recompile_material(mat)
    unreal.EditorAssetLibrary.save_loaded_asset(mat)
    materials['AGV_'+name] = mat

report = []
for part in ('Body','Tire','Hub'):
    name = 'SM_RotemAGV_' + part
    options = unreal.FbxImportUI()
    options.import_mesh = True
    options.import_as_skeletal = False
    options.import_materials = False
    options.import_textures = False
    options.automated_import_should_detect_type = False
    options.mesh_type_to_import = unreal.FBXImportType.FBXIT_STATIC_MESH
    options.static_mesh_import_data.combine_meshes = True
    options.static_mesh_import_data.auto_generate_collision = False
    options.static_mesh_import_data.generate_lightmap_u_vs = False
    task = unreal.AssetImportTask()
    task.filename = str(root/'Content/PortSim/Assets/SourceFiles/RotemAGV'/(name+'.fbx'))
    task.destination_path = folder
    task.destination_name = name
    task.automated = True
    task.replace_existing = True
    task.save = True
    task.options = options
    # Use the legacy FBX importer for repeatable names and material slots.
    task.factory = unreal.FbxFactory()
    tools.import_asset_tasks([task])
    mesh = unreal.load_asset(folder+'/'+name)
    if not isinstance(mesh,unreal.StaticMesh):
        raise RuntimeError('Missing imported mesh '+name)
    slots = []
    for i,slot in enumerate(mesh.static_materials):
        key = str(slot.material_slot_name)
        if key not in materials:
            raise RuntimeError('Unknown material slot '+key)
        mesh.set_material(i,materials[key])
        slots.append(key)
    unreal.EditorAssetLibrary.save_loaded_asset(mesh)
    bounds = mesh.get_bounds()
    extent = bounds.box_extent
    if part == 'Body' and not (680 < extent.y <= 730 and 190 < extent.x <= 210):
        raise RuntimeError('Unexpected body scale/orientation: '+str(extent))
    if part == 'Tire' and not (49 < extent.x < 52 and 49 < extent.z < 52):
        raise RuntimeError('Unexpected tire envelope: '+str(extent))
    report.append(dict(part=part,extent=[extent.x,extent.y,extent.z],materials=slots))
report_path = root/'Saved/Reports/RotemAGV_Import.json'
report_path.parent.mkdir(parents=True,exist_ok=True)
report_path.write_text(json.dumps(report,indent=2),encoding='utf-8')
unreal.log('PORTSIM_ROTEM_IMPORT_PASS '+json.dumps(report))
