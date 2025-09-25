# World building (city/ground), lighting, dome, tonemapper, friction

from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.stage import add_reference_to_stage
from pxr import UsdLux, Sdf, UsdPhysics, PhysxSchema

def set_world(stage, world, cfg):
    if cfg["World"]["using_simple_ground_plane"]:
        world.scene.add_default_ground_plane()
    else:
        add_reference_to_stage(usd_path=cfg["World"]["city_usd_path"].replace("{PACKAGE}", cfg.get("PACKAGE","")),
                               prim_path=cfg["World"]["city_prim_path"])
        city = XFormPrim(cfg["World"]["city_prim_path"], name=cfg["World"]["city_name"])
        world.scene.add(city)

        light = UsdLux.DistantLight.Define(world.stage, Sdf.Path(cfg["World"]["light_prim_path"]))
        light.CreateIntensityAttr(cfg["World"]["light_intensity"])
        light.CreateColorTemperatureAttr(cfg["World"]["light_colortemperature"])
        light.AddTranslateOp()
        world.scene.add(XFormPrim(cfg["World"]["light_prim_path"], name=cfg["World"]["light_name"]))

def setup_sunlight(stage):
    sun_path = Sdf.Path("/World/SunLight")
    sun = UsdLux.DistantLight.Get(stage, sun_path) or UsdLux.DistantLight.Define(stage, sun_path)
    sun.CreateAngleAttr(0.53); sun.CreateIntensityAttr(3000.0)
    UsdLux.LightAPI(sun).CreateExposureAttr(2.0)
    sun.CreateColorTemperatureAttr(6000.0); sun.CreateEnableColorTemperatureAttr(True)
    UsdLux.ShadowAPI.Apply(sun.GetPrim()).CreateShadowEnableAttr(True)
    x = UsdLux.DistantLight(stage.GetPrimAtPath(sun_path))
    # orientation set via Xformable ops:
    from pxr import UsdGeom, Gf
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(sun_path))
    for op in xf.GetOrderedXformOps(): xf.RemoveXformOp(op)
    xf.AddRotateXYZOp().Set((45.0, 20.0, 0.0))

def setup_dome(stage):
    dome_path = Sdf.Path("/World/SkyDome")
    dome = UsdLux.DomeLight.Get(stage, dome_path) or UsdLux.DomeLight.Define(stage, dome_path)
    dome.CreateIntensityAttr(500.0)
    UsdLux.LightAPI(dome).CreateExposureAttr(1.0)
    from pxr import UsdGeom
    xf = UsdGeom.Xformable(stage.GetPrimAtPath(dome_path))
    for op in xf.GetOrderedXformOps(): xf.RemoveXformOp(op)
    xf.AddRotateYOp().Set(90.0)

def setup_tonemapper(exposure_stops: float = 1.5):
    import carb
    s = carb.settings.get_settings()
    s.set("/rtx/post/tonemapper/enable", True)
    s.set("/rtx/post/tonemapper/aces", True)
    s.set("/rtx/post/tonemapper/exposure", float(exposure_stops))

def bind_material(stage, prim_path: str, mat_path: Sdf.Path):
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        print(f"[Friction][WARN] invalid prim: {prim_path}")
        return
    UsdPhysics.CollisionAPI.Apply(prim)
    rel = prim.GetRelationship("physics:material:binding") or prim.CreateRelationship("physics:material:binding", False)
    rel.ClearTargets(False); rel.SetTargets([mat_path])

def setup_friction(stage, cfg=None):
    friction_cfg = (cfg or {}).get("Friction", {})
    apply_wheel = friction_cfg.get("apply_wheel_friction", True)
    apply_ground = friction_cfg.get("apply_ground_friction", True)
    wheel_paths = []
    if cfg and "Robot" in cfg:
        base_link = cfg["Robot"].get("robot_base_link_prim")
        container_root = cfg["Robot"].get("robot_container_prim")
        if base_link:
            from pxr import Sdf
            root_path = Sdf.Path(base_link).GetParentPath().pathString
        else:
            root_path = container_root
        if root_path:
            for name in (
                "fl_wheel_link",
                "fr_wheel_link",
                "rl_wheel_link",
                "rr_wheel_link",
            ):
                wheel_paths.append(f"{root_path}/{name}")

    if not wheel_paths:
        wheel_paths = [
            "/World/robot/r_front_wheel",
            "/World/robot/l_front_wheel",
            "/World/robot/r_rear_wheel",
            "/World/robot/l_rear_wheel",
        ]
    use_city = True
    if cfg and cfg.get("World", {}).get("using_simple_ground_plane"):
        use_city = False

    ground_path = friction_cfg.get("ground_prim_path") if apply_ground else None
    if not ground_path and use_city:
        ground_path = "/World/City/Terrian"
    elif not apply_ground:
        ground_path = None

    tire_mat = Sdf.Path(friction_cfg.get("wheel_material_path", "/World/Materials/Tire"))
    ground_mat = Sdf.Path(friction_cfg.get("ground_material_path", "/World/Materials/Ground"))
    stage.DefinePrim(tire_mat, "PhysicsMaterial")
    stage.DefinePrim(ground_mat, "PhysicsMaterial")

    tm = UsdPhysics.MaterialAPI.Get(stage, tire_mat)
    gm = UsdPhysics.MaterialAPI.Get(stage, ground_mat)
    tm.CreateStaticFrictionAttr(1.10); tm.CreateDynamicFrictionAttr(1.00); tm.CreateRestitutionAttr(0.00)
    gm.CreateStaticFrictionAttr(0.90); gm.CreateDynamicFrictionAttr(0.80); gm.CreateRestitutionAttr(0.00)

    tpx = PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath(tire_mat))
    gpx = PhysxSchema.PhysxMaterialAPI.Apply(stage.GetPrimAtPath(ground_mat))
    for api in (tpx, gpx):
        api.CreateFrictionCombineModeAttr(PhysxSchema.Tokens.average)
        api.CreateRestitutionCombineModeAttr(PhysxSchema.Tokens.average)

    if apply_wheel:
        for wp in wheel_paths:
            bind_material(stage, wp, tire_mat)
    if apply_ground and ground_path:
        bind_material(stage, ground_path, ground_mat)
