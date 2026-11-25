"""Temporary unsaved UE editor studio; -ExecCmds="py <this file>".
Validates exterior isolation and captures the actual imported actor.
"""
import unreal
from pathlib import Path

unreal.EditorLevelLibrary.new_level('/Temp/RotemAGVReview')
actors = unreal.get_editor_subsystem(unreal.EditorActorSubsystem)
vehicle = actors.spawn_actor_from_class(unreal.load_class(None,'/Script/PortSim.PortAGVActor'),unreal.Vector())
meshes = vehicle.get_components_by_class(unreal.StaticMeshComponent)
deck = next(m for m in meshes if m.get_name()=='AGV_Deck')
assert not deck.is_visible()
assert deck.get_editor_property('relative_location') == unreal.Vector(0,0,170)
scale = deck.get_editor_property('relative_scale3d')
assert abs(scale.x-3)<.001 and abs(scale.y-13.8)<.001 and abs(scale.z-1)<.001
assert deck.get_collision_enabled() != unreal.CollisionEnabled.NO_COLLISION
wheel_count = 0
for mesh in meshes:
    if 'PortSim.AuthoredExterior' in [str(t) for t in mesh.component_tags]:
        assert mesh.get_collision_enabled() == unreal.CollisionEnabled.NO_COLLISION
        assert mesh.static_mesh is not None
    if mesh.get_name().startswith(('Bogie_AGV','WheelHub_AGV')):
        wheel_count += 1
assert wheel_count == 8
for label in vehicle.get_components_by_class(unreal.TextRenderComponent):
    if 'PortSim.FleetNumber' in [str(t) for t in label.component_tags]: label.set_text('103')
    else: label.set_visibility(False)
unreal.log('PORTSIM_ROTEM_VISUAL_INVARIANTS_PASS: 4 wheel pivots, original collision deck, visual-only exterior')

floor = actors.spawn_actor_from_class(unreal.StaticMeshActor,unreal.Vector(0,0,-10))
floor.static_mesh_component.set_static_mesh(unreal.load_asset('/Engine/BasicShapes/Cube'))
floor.set_actor_scale3d(unreal.Vector(100,100,.1))
floor.static_mesh_component.set_material(0,unreal.load_asset('/Game/PortSim/Assets/Materials/M_SiteBuilding'))
sun = actors.spawn_actor_from_class(unreal.DirectionalLight,unreal.Vector(0,0,1000),unreal.Rotator(-40,-30,0))
sun.light_component.set_intensity(6)
fill = actors.spawn_actor_from_class(unreal.DirectionalLight,unreal.Vector(0,0,1000),unreal.Rotator(-25,150,0))
fill.light_component.set_intensity(3)
fill.light_component.set_cast_shadows(False)
camera = actors.spawn_actor_from_class(unreal.CameraActor,unreal.Vector(1650,1850,650))
target = unreal.Vector(0,0,130)
camera.set_actor_rotation(unreal.MathLibrary.find_look_at_rotation(camera.get_actor_location(),target),False)
camera.camera_component.set_field_of_view(47)
unreal.EditorLevelLibrary.set_level_viewport_camera_info(camera.get_actor_location(),camera.get_actor_rotation())
out = Path(unreal.Paths.project_saved_dir())/'Screenshots/RotemAGV_Studio.png'
out.parent.mkdir(parents=True,exist_ok=True)
elapsed = 0
shot = False
def tick(dt):
    global elapsed,shot
    elapsed += dt
    if elapsed > 20 and not shot:
        unreal.AutomationLibrary.take_high_res_screenshot(1600,1000,str(out),camera=camera)
        shot = True
    if elapsed > 27:
        unreal.unregister_slate_post_tick_callback(handle)
        unreal.SystemLibrary.quit_editor()
handle = unreal.register_slate_post_tick_callback(tick)
