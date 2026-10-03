"""Render-only laboratory staging. Never adds or changes collision geometry."""
import numpy as np


def material(stage, name, color, roughness=.6, metallic=0., vertex_color=False):
    from pxr import UsdShade, Sdf, Gf
    mat=UsdShade.Material.Define(stage, '/World/Studio/Materials/'+name)
    shader=UsdShade.Shader.Define(stage, str(mat.GetPath())+'/Surface')
    shader.CreateIdAttr('UsdPreviewSurface')
    diffuse=shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f)
    diffuse.Set(Gf.Vec3f(*color))
    shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(roughness)
    shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(metallic)
    if vertex_color:
        reader=UsdShade.Shader.Define(stage,str(mat.GetPath())+'/Color')
        reader.CreateIdAttr('UsdPrimvarReader_float3')
        reader.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('displayColor')
        diffuse.ConnectToSource(reader.ConnectableAPI(),'result')
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
    return mat


def bind(prim, mat):
    from pxr import UsdShade
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat,UsdShade.Tokens.strongerThanDescendants)


def setup(stage):
    import carb
    carb.settings.get_settings().set_int("/persistent/app/viewport/displayOptions",0)
    carb.settings.get_settings().set_bool("/app/viewport/grid/enabled",False)
    from pxr import Usd, UsdGeom, UsdLux, Gf
    UsdGeom.Xform.Define(stage,'/World/Studio')
    # Rebind the existing floor: its physical plane remains exactly unchanged.
    floor=material(stage,'Floor',(.115,.12,.125),.83)
    ground=stage.GetPrimAtPath('/World/defaultGroundPlane')
    if ground:
        bind(ground,floor)
        # The bundled grid floor includes a 100,000-intensity point light.
        # Disable that inherited light so this rig controls exposure.
        for prim in Usd.PrimRange(ground):
            if prim.GetTypeName().endswith('Light'):
                for attr in ('inputs:intensity','intensity'):
                    value=prim.GetAttribute(attr)
                    if value: value.Set(0.)
    for path in ('/World/Light',):
        if stage.GetPrimAtPath(path): stage.RemovePrim(path)
    dome=UsdLux.DomeLight.Define(stage,'/World/Studio/Ambient')
    dome.CreateIntensityAttr(300.)
    dome.CreateColorAttr(Gf.Vec3f(.86,.91,1.))
    wall=material(stage,'Wall',(.30,.31,.30),.9)
    for name,position,scale in [
        ('BackWall',(0,3.5,1.8),(5,.035,1.8)),
        ('SideWall',(3.5,0,1.8),(.035,5,1.8)),
        ('FrontWall',(0,-3.5,1.8),(5,.035,1.8)),
        ('LeftWall',(-3.5,0,1.8),(.035,5,1.8))]:
        cube=UsdGeom.Cube.Define(stage,'/World/Studio/'+name)
        cube.CreateSizeAttr(2.)
        cube.AddTranslateOp().Set(Gf.Vec3d(*position))
        cube.AddScaleOp().Set(Gf.Vec3f(*scale))
        bind(cube.GetPrim(),wall)
    lights=[('Key',(-2.,-2.,3.2),700.,5600.),
            ('Fill',(1.5,-1.8,2.5),420.,6500.),
            ('Rim',(-.4,2.,2.7),600.,6200.),
            ('Overhead',(-.3,0,3.4),350.,6000.),
            ('ReceiverFill',(-2.5,.6,1.8),280.,5800.)]
    for name,position,intensity,temperature in lights:
        light=UsdLux.RectLight.Define(stage,'/World/Studio/'+name)
        light.CreateWidthAttr(1.6);light.CreateHeightAttr(1.2)
        light.CreateIntensityAttr(intensity)
        light.CreateExposureAttr(4.)
        light.CreateEnableColorTemperatureAttr(True)
        light.CreateColorTemperatureAttr(temperature)
        matrix=Gf.Matrix4d().SetLookAt(Gf.Vec3d(*position),Gf.Vec3d(-.4,-.2,.95),Gf.Vec3d(0,0,1)).GetInverse()
        UsdGeom.Xformable(light).AddTransformOp().Set(matrix)
    return {'preset':'lab','area_lights':[x[0] for x in lights],
            'ambient_intensity':300.,'decorative_geometry_collision':False,
            'reference':'Neutral grey laboratory and dark workbench inspired by paper Figures 1 and 6'}


def smooth(mesh):
    """Shade original triangles smoothly without moving vertices or collider faces."""
    from pxr import Gf
    p=np.asarray(mesh.GetPointsAttr().Get(),dtype=float)
    counts=np.asarray(mesh.GetFaceVertexCountsAttr().Get())
    if not np.all(counts==3): return
    f=np.asarray(mesh.GetFaceVertexIndicesAttr().Get()).reshape(-1,3)
    face=np.cross(p[f[:,1]]-p[f[:,0]],p[f[:,2]]-p[f[:,0]])
    normals=np.zeros_like(p)
    for i in range(3): np.add.at(normals,f[:,i],face)
    normals/=np.maximum(np.linalg.norm(normals,axis=1,keepdims=True),1e-15)
    mesh.CreateNormalsAttr([Gf.Vec3f(*n) for n in normals])
    mesh.SetNormalsInterpolation('vertex')


def dress(stage, asset_robot=None):
    from pxr import Usd, UsdGeom
    table=material(stage,'Table',(.035,.04,.045),.48)
    bind(stage.GetPrimAtPath('/World/Table'),table)
    if asset_robot:
        # The source assembly's static workbench lives beneath /world/.
        for prim in Usd.PrimRange(asset_robot.root):
            if '/world/' in str(prim.GetPath()) and prim.IsA(UsdGeom.Gprim): bind(prim,table)
    hand=stage.GetPrimAtPath('/World/Trial/Hand/mesh')
    if hand:
        bind(hand,material(stage,'Hand',(.58,.36,.24),.58))
        smooth(UsdGeom.Mesh(hand))
    obj=stage.GetPrimAtPath('/World/Trial/ObjectMesh')
    if obj:
        bind(obj,material(stage,'Object',(.5,.5,.5),.48,vertex_color=True))
        smooth(UsdGeom.Mesh(obj))
    # Only presentation guides are hidden; all metric volumes stay in the trial.
    for path in ('/World/Trial/ReachRegion','/World/Trial/ReceiverSkeleton'):
        prim=stage.GetPrimAtPath(path)
        if prim:
            for child in Usd.PrimRange(prim):
                imageable=UsdGeom.Imageable(child)
                if imageable: imageable.MakeInvisible()
