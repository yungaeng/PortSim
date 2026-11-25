"""Build the GPU Gerstner water material with Unreal's Python commandlet."""
import unreal

path = '/Game/PortSim/Assets/Materials/M_PortGerstnerWater'
material = unreal.load_asset(path)
if material is None:
    material = unreal.AssetToolsHelpers.get_asset_tools().create_asset(
        'M_PortGerstnerWater', '/Game/PortSim/Assets/Materials',
        unreal.Material, unreal.MaterialFactoryNew())
unreal.MaterialEditingLibrary.delete_all_material_expressions(material)
material.set_editor_property('tangent_space_normal', False)
material.set_editor_property('two_sided', True)
edit = unreal.MaterialEditingLibrary


def node(kind):
    return edit.create_material_expression(material, kind)


def scalar(name, default):
    result = node(unreal.MaterialExpressionScalarParameter)
    result.set_editor_property('parameter_name', name)
    result.set_editor_property('default_value', default)
    return result


def custom(code, inputs):
    result = node(unreal.MaterialExpressionCustom)
    result.set_editor_property('code', code)
    result.set_editor_property('output_type', unreal.CustomMaterialOutputType.CMOT_FLOAT3)
    descriptors = []
    for name in inputs:
        descriptor = unreal.CustomInput()
        descriptor.set_editor_property('input_name', name)
        descriptors.append(descriptor)
    result.set_editor_property('inputs', descriptors)
    for name, (source, output) in inputs.items():
        if not edit.connect_material_expressions(source, output, result, name):
            raise RuntimeError('Could not connect custom input ' + name)
    return result


position = node(unreal.MaterialExpressionWorldPosition)
position.set_editor_property('world_position_shader_offset',
                             unreal.WorldPositionIncludedOffsets.WPT_EXCLUDE_ALL_SHADER_OFFSETS)
enabled = scalar('WaveEnabled', 1)
phases = node(unreal.MaterialExpressionVectorParameter)
phases.set_editor_property('parameter_name', 'WavePhases')


def rgba(parameter):
    result = node(unreal.MaterialExpressionAppendVector)
    if not edit.connect_material_expressions(parameter, '', result, 'A'):
        raise RuntimeError('Could not connect vector RGB')
    if not edit.connect_material_expressions(parameter, 'A', result, 'B'):
        raise RuntimeError('Could not connect vector alpha')
    return result


inputs = {'P': (position, ''), 'Enabled': (enabled, ''), 'Phases': (rgba(phases), '')}
for index in range(4):
    wave = node(unreal.MaterialExpressionVectorParameter)
    wave.set_editor_property('parameter_name', 'Wave' + str(index))
    wave.set_editor_property('default_value', unreal.LinearColor(1, 0, 8, .001))
    inputs['W' + str(index)] = (rgba(wave), '')

# All four Gerstner terms and their analytic tangents are evaluated per vertex.
# Fade the displacement at the water's perimeter, including the quay edge.
shared = '''
float4 waves[4] = {W0, W1, W2, W3};
float fade = saturate(min(min(P.x + 60700., -700. - P.x), 72500. - abs(P.y)) / 2000.);
fade = fade * fade * (3. - 2. * fade) * Enabled;
float3 offset = 0.;
float3 tx = float3(1,0,0), ty = float3(0,1,0);
for (int i=0; i<4; ++i) {
    float2 d = waves[i].xy;
    float a = waves[i].z * fade;
    float k = waves[i].w;
    float s, c;
    sincos(k * dot(d, P.xy) - Phases[i], s, c);
    // Low steepness for sheltered port water, with a bounded horizontal Jacobian.
    float q = min(.35, .15 / max(k * a, .0001));
    offset += float3(q*a*d*c, a*s);
    tx += float3(-q*a*k*d*d.x*s, a*k*d.x*c);
    ty += float3(-q*a*k*d*d.y*s, a*k*d.y*c);
}
'''
displacement = custom(shared + 'return offset;', inputs)
edit.connect_material_property(displacement, '', unreal.MaterialProperty.MP_WORLD_POSITION_OFFSET)
normal = custom(shared + 'return normalize(cross(tx,ty));', inputs)
interpolator = node(unreal.MaterialExpressionVertexInterpolator)
edit.connect_material_expressions(normal, '', interpolator, '')
ripples = custom('''
// Two short directional ripples, shaded without extra mesh subdivisions.
float r0 = dot(P.xy, float2(.012,.006)) - T
    + 1.6*sin(dot(P.xy,float2(.0013,.0021)) - T);
float r1 = dot(P.xy, float2(-.008,.019)) - T*2.
    + 1.9*sin(dot(P.xy,float2(-.0017,.0009)) + T);
float footprint = max(length(ddx(P.xy)), length(ddy(P.xy)));
float detail = 1.-smoothstep(30.,200.,footprint);
float3 n = N + Enabled*detail*float3(.025*cos(r0)-.018*cos(r1),
                                    .0125*cos(r0)+.035*cos(r1),0);
return normalize(n);
''', {'P': (position, ''), 'N': (interpolator, ''),
      'T': (scalar('RipplePhase', 0), ''), 'Enabled': (enabled, '')})
edit.connect_material_property(ripples, '', unreal.MaterialProperty.MP_NORMAL)
color = node(unreal.MaterialExpressionConstant3Vector)
color.set_editor_property('constant', unreal.LinearColor(.008, .045, .065))
edit.connect_material_property(color, '', unreal.MaterialProperty.MP_BASE_COLOR)
edit.connect_material_property(scalar('Roughness', .22), '', unreal.MaterialProperty.MP_ROUGHNESS)
edit.connect_material_property(scalar('Specular', .5), '', unreal.MaterialProperty.MP_SPECULAR)
edit.recompile_material(material)
if not unreal.EditorAssetLibrary.save_loaded_asset(material):
    raise RuntimeError('Could not save water material')
unreal.log('PORTSIM_WATER_MATERIAL_PASS')
