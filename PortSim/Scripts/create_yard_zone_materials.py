"""Save only the six display-zone materials; does not run the simulation or tests."""
import unreal

palette = {
    'YardReefer': (0.009, 0.153, 0.381),
    'YardDangerous': (0.418, 0.072, 0.002),
    'YardChemical': (0.216, 0.041, 0.386),
    'YardReeferFill': (0.10, 0.22, 0.30),
    'YardDangerousFill': (0.30, 0.19, 0.09),
    'YardChemicalFill': (0.24, 0.15, 0.30),
}
tools = unreal.AssetToolsHelpers.get_asset_tools()
folder = '/Game/PortSim/Assets/Materials'
for name, color in palette.items():
    path = folder + '/M_' + name
    material = unreal.load_asset(path) if unreal.EditorAssetLibrary.does_asset_exist(path) else tools.create_asset('M_' + name, folder, unreal.Material, unreal.MaterialFactoryNew())
    material.set_editor_property('used_with_instanced_static_meshes', True)
    unreal.MaterialEditingLibrary.delete_all_material_expressions(material)
    node = unreal.MaterialEditingLibrary.create_material_expression(material, unreal.MaterialExpressionConstant3Vector)
    node.set_editor_property('constant', unreal.LinearColor(*color, 1))
    unreal.MaterialEditingLibrary.connect_material_property(node, '', unreal.MaterialProperty.MP_BASE_COLOR)
    unreal.MaterialEditingLibrary.recompile_material(material)
    if not unreal.EditorAssetLibrary.save_loaded_asset(material):
        raise RuntimeError('Could not save ' + path)
unreal.log('Yard display materials saved.')
