"""Shared URDF scene for read-only joint display and offline pose verification."""
import xml.etree.ElementTree as ET
from pathlib import Path


def build_scene(source, output, frames, ground_z):
    from pxr import Gf, PhysicsSchemaTools, Usd, UsdGeom, UsdPhysics
    from isaacsim.core.api import World
    from isaacsim.core.prims import SingleArticulation, SingleXFormPrim
    from isaacsim.core.utils.extensions import enable_extension
    source = Path(source)
    tree = ET.parse(source)
    package = source.parent.parent
    for mesh in tree.findall('.//mesh'):
        uri = mesh.get('filename')
        prefix = f'package://{package.name}/'
        if uri.startswith(prefix):
            mesh.set('filename', str(package/uri.removeprefix(prefix)))
        elif not Path(uri).is_absolute():
            raise ValueError('UNRESOLVED_ISAAC_MESH:'+uri)
    roots = {link.get('name') for link in tree.findall('link')} - {
        joint.find('child').get('link') for joint in tree.findall('joint')}
    if len(roots) != 1:
        raise ValueError('URDF_REQUIRES_ONE_ROOT')
    absolute = output/'model.urdf'
    tree.write(absolute, encoding='utf-8', xml_declaration=True)
    enable_extension('isaacsim.asset.importer.urdf')
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    usd = URDFImporter(URDFImporterConfig(urdf_path=str(absolute), usd_path=str(output/'usd'),
        fix_base=True, merge_fixed_joints=False, allow_self_collision=False)).import_urdf()
    world = World(stage_units_in_meters=1., physics_dt=.01, rendering_dt=.05)
    model = world.stage.DefinePrim('/World/Robot', 'Xform')
    model.GetReferences().AddReference(str(usd))
    variant = model.GetVariantSets().GetVariantSet('Physics')
    if variant.IsValid() and not variant.SetVariantSelection('physx'):
        raise ValueError('PHYSX_ASSET_VARIANT_REQUIRED')
    links = {prim.GetName(): prim for prim in Usd.PrimRange(model) if prim.IsA(UsdGeom.Xform)}
    if not {roots.pop(), *frames}.issubset(links):
        raise ValueError('ISAAC_LINK_LAYOUT')
    art_roots = [p for p in Usd.PrimRange(model) if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if len(art_roots) != 1:
        raise ValueError('ISAAC_ARTICULATION_LAYOUT')
    body = world.scene.add(SingleArticulation(str(art_roots[0].GetPath()), name='robot'))
    flanges = {name: world.scene.add(SingleXFormPrim(str(links[name].GetPath()), name=name)) for name in frames}
    PhysicsSchemaTools.addGroundPlane(world.stage, '/World/Ground', 'Z', 5.,
        Gf.Vec3f(0., 0., ground_z), Gf.Vec3f(.5, .5, .5))
    world.get_physics_context().set_gravity(0.)
    world.reset()
    return world, body, flanges
