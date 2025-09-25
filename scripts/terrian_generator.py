import omni
import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics, PhysxSchema, UsdShade, Sdf, Gf
import yaml
from ament_index_python.packages import get_package_share_directory

# -----------------------
# 공용 유틸
# -----------------------
def _to_raw(h_float, vertical_scale):
    return np.clip(np.round(h_float / vertical_scale), -32768, 32767).astype(np.int16)

def _smooth2d(arr, iters=1):
    out = arr.copy()
    for _ in range(iters):
        out = (out + np.roll(out, 1, 0) + np.roll(out, -1, 0)
                    + np.roll(out, 1, 1) + np.roll(out, -1, 1)) / 5.0
    return out

def convert_heightfield_to_trimesh(heightfield_raw, horizontal_scale=0.25, vertical_scale=0.005):
    H, W = heightfield_raw.shape
    zz = (heightfield_raw.astype(np.float32) * vertical_scale)
    xv, yv = np.meshgrid(np.arange(W), np.arange(H))
    px = xv * horizontal_scale
    py = yv * horizontal_scale
    pz = zz
    vertices = np.stack([px, py, pz], axis=-1).reshape(-1, 3)

    tris = []
    def vid(r, c): return r * W + c
    for r in range(H - 1):
        for c in range(W - 1):
            v0 = vid(r, c); v1 = vid(r, c + 1); v2 = vid(r + 1, c); v3 = vid(r + 1, c + 1)
            tris.append([v0, v1, v2]); tris.append([v2, v1, v3])
    triangles = np.array(tris, dtype=np.int32)
    return vertices, triangles

def bind_color_material(stage, prim, color=(0.25, 0.55, 0.25), roughness=0.9, metallic=0.0):
    mat_path    = prim.GetPath().AppendPath("Looks/terrain_mat")
    shader_path = mat_path.AppendPath("PBRShader")

    material = UsdShade.Material.Define(stage, mat_path)
    shader   = UsdShade.Shader.Define(stage, shader_path)
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f)\
          .Set(Gf.Vec3f(float(color[0]), float(color[1]), float(color[2])))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(float(roughness))
    shader.CreateInput("metallic",  Sdf.ValueTypeNames.Float).Set(float(metallic))

    shader_out = shader.CreateOutput("surface", Sdf.ValueTypeNames.Token)
    mat_out = material.CreateSurfaceOutput()
    try:
        mat_out.ConnectToSource(shader_out)
    except Exception:
        mat_out.ConnectToSource(shader, "surface")

    try:
        UsdShade.MaterialBindingAPI(prim).Bind(material)
    except Exception:
        rel = prim.CreateRelationship("material:binding", True)
        rel.SetTargets([material.GetPath()])

def _ensure_physx_material(stage, path="/World/PhysicsMaterials/TerrainPhysXMat",
                           static_friction=0.9, dynamic_friction=0.9, restitution=0.0):
    p = Sdf.Path(path)
    prim = stage.GetPrimAtPath(p)
    if not prim:
        prim = stage.DefinePrim(p, "Material")
    prim.CreateAttribute("physxMaterial:staticFriction",  Sdf.ValueTypeNames.Float).Set(float(static_friction))
    prim.CreateAttribute("physxMaterial:dynamicFriction", Sdf.ValueTypeNames.Float).Set(float(dynamic_friction))
    prim.CreateAttribute("physxMaterial:restitution",     Sdf.ValueTypeNames.Float).Set(float(restitution))
    return prim

def add_or_replace_terrain_mesh(stage, vertices, triangles, position, orientation_xyzw, prim_path="/World/Terrain"):
    p = Sdf.Path(prim_path)
    if stage.GetPrimAtPath(p):
        import omni.kit.commands
        omni.kit.commands.execute("DeletePrims", paths=[str(p)])

    mesh = UsdGeom.Mesh.Define(stage, p)

    verts_list = [Gf.Vec3f(float(x), float(y), float(z)) for x, y, z in vertices.reshape(-1, 3).tolist()]
    mesh.CreatePointsAttr(verts_list)
    mesh.CreateFaceVertexCountsAttr([3] * len(triangles))
    mesh.CreateFaceVertexIndicesAttr([int(i) for i in triangles.flatten().tolist()])

    xf = UsdGeom.Xformable(mesh.GetPrim())
    xf.AddTranslateOp().Set(Gf.Vec3d(float(position[0]), float(position[1]), float(position[2])))
    q = np.array(orientation_xyzw, dtype=float)
    n = np.linalg.norm(q)
    if n > 0 and abs(n - 1.0) > 1e-6:
        q /= n
    xf.AddOrientOp().Set(Gf.Quatf(float(q[3]), float(q[0]), float(q[1]), float(q[2])))

    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    PhysxSchema.PhysxCollisionAPI.Apply(mesh.GetPrim())
    try:
        mapi = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
        if hasattr(mapi, "CreateApproximationAttr"):
            mapi.CreateApproximationAttr().Set("triangleMesh")
        else:
            mesh.GetPrim().CreateAttribute("physics:meshCollision:approximation",
                                           Sdf.ValueTypeNames.Token).Set("triangleMesh")
    except Exception:
        mesh.GetPrim().CreateAttribute("physics:meshCollision:approximation",
                                       Sdf.ValueTypeNames.Token).Set("triangleMesh")

    mat_prim = _ensure_physx_material(stage)
    if hasattr(UsdPhysics, "MaterialBindingAPI"):
        try:
            bind_api = UsdPhysics.MaterialBindingAPI.Apply(mesh.GetPrim())
            if hasattr(bind_api, "Bind"):
                bind_api.Bind(mat_prim)
            else:
                bind_api.CreateMaterialRel().SetTargets([mat_prim.GetPath()])
        except Exception:
            mesh.GetPrim().CreateRelationship("physics:material:binding", True)\
                          .SetTargets([mat_prim.GetPath()])
    else:
        mesh.GetPrim().CreateRelationship("physics:material:binding", True)\
                      .SetTargets([mat_prim.GetPath()])
    return mesh

# -----------------------
# 지형 생성기 (YAML 파라미터 방식 + 사면 벽)
# -----------------------
class UnstructuredTerrainWorld:
    def __init__(self, stage):
        self.stage = stage
        pkg = get_package_share_directory("isaac_ml")
        yaml_path = pkg + "/config/unstructured_orchard.yaml"
        with open(yaml_path, "r") as f:
            cfg_all = yaml.safe_load(f)
        self.cfg = cfg_all["Terrain"]

        # 공통 파라미터
        self.prim_path = self.cfg.get("prim_path", "/World/Terrain")
        self.width  = float(self.cfg.get("width_m", 20.0))
        self.length = float(self.cfg.get("length_m", 20.0))
        self.hs = float(self.cfg.get("horizontal_scale_m", 0.25))
        self.vs = float(self.cfg.get("vertical_scale_m", 0.005))
        self.position = tuple(self.cfg.get("position_xyz", [0.0, 0.0, 0.0]))
        self.orient_xyzw = tuple(self.cfg.get("orientation_xyzw", [0.0, 0.0, 0.0, 1.0]))
        self.color = tuple(self.cfg.get("color_rgb", [0.25, 0.55, 0.25]))
        self.roughness = float(self.cfg.get("roughness", 0.9))
        self.metallic  = float(self.cfg.get("metallic", 0.0))
        self.method = self.cfg.get("method", "fbm").lower()
        self.seed = int(self.cfg.get("seed", 42))

        # 벽 설정
        wcfg = self.cfg.get("walls", {})
        self.walls_enabled  = bool(wcfg.get("enabled", True))
        self.wall_height    = float(wcfg.get("height_m", 3.0))
        self.wall_thickness = float(wcfg.get("thickness_m", 0.1))
        self.wall_margin    = float(wcfg.get("margin_m", 0.0))  # +면 바깥으로 살짝 여유
        self.wall_color     = tuple(wcfg.get("color_rgb", [0.6, 0.6, 0.6]))
        self.wall_base_z    = float(wcfg.get("base_z", 0.0))    # 벽 바닥 z (보통 0)

        # 해상도(셀 수)
        self.rows = int(self.length / self.hs)
        self.cols = int(self.width  / self.hs)

    # ---- 방법 A: bumps ----
    def _build_bumps_height(self):
        p = self.cfg["bumps"]
        n_bumps = int(p.get("n_bumps", 120))
        amp_lo, amp_hi = map(float, p.get("amp_range_m", [0.05, 0.25]))
        sig_lo, sig_hi = map(float, p.get("sigma_range_m", [0.4, 1.6]))
        plateau = bool(p.get("plateau", True))

        rng = np.random.default_rng(self.seed)
        x = np.arange(self.cols) * self.hs
        y = np.arange(self.rows) * self.hs
        X, Y = np.meshgrid(x, y)
        H = np.zeros((self.rows, self.cols), dtype=np.float32)

        for _ in range(n_bumps):
            cx = rng.uniform(0.0, x[-1] if self.cols > 1 else 1.0)
            cy = rng.uniform(0.0, y[-1] if self.rows > 1 else 1.0)
            amp = rng.uniform(amp_lo, amp_hi)
            sig = rng.uniform(sig_lo, sig_hi)
            R2 = (X - cx) ** 2 + (Y - cy) ** 2
            bump = amp * np.exp(-0.5 * R2 / (sig ** 2))
            if plateau:
                bump = amp * np.tanh(bump / max(1e-6, 0.5 * amp))
            H += bump

        H = H - np.median(H)
        H = np.clip(H, 0.0, None)
        return H

    # ---- 방법 B: fBM ----
    def _build_fbm_height(self):
        p = self.cfg["fbm"]
        octaves     = int(p.get("octaves", 4))
        base_res    = int(p.get("base_res", 8))
        lacunarity  = float(p.get("lacunarity", 2.0))
        gain        = float(p.get("gain", 0.55))
        amplitude   = float(p.get("amplitude_m", 0.18))
        smooth_iter = int(p.get("smooth_iters", 2))

        rng = np.random.default_rng(self.seed)
        H = np.zeros((self.rows, self.cols), dtype=np.float32)
        amp = 1.0

        for o in range(octaves):
            gh = max(2, int(base_res * (lacunarity ** o)))
            gw = max(2, int(base_res * (lacunarity ** o)))
            grid = rng.random((gh, gw)).astype(np.float32)
            up = np.kron(grid, np.ones((int(np.ceil(self.rows/gh)),
                                        int(np.ceil(self.cols/gw))), dtype=np.float32))
            up = up[:self.rows, :self.cols]
            up = _smooth2d(up, iters=smooth_iter)
            H += amp * up
            amp *= gain

        hmin, hmax = float(H.min()), float(H.max())
        if hmax > hmin:
            H = (H - hmin) / (hmax - hmin)
        H = (H - 0.5) * 2.0 * amplitude
        H = np.clip(H, -0.1, None)
        return H

    # ---- 벽 생성 ----
    def _create_walls(self):
        if not self.walls_enabled:
            return

        stage = self.stage
        group_path = self.prim_path + "_Walls"
        gp = Sdf.Path(group_path)

        # 기존 그룹 삭제(재생성)
        if stage.GetPrimAtPath(gp):
            import omni.kit.commands
            omni.kit.commands.execute("DeletePrims", paths=[str(gp)])

        # 그룹 Xform (지형과 같은 위치/자세)
        group = UsdGeom.Xform.Define(stage, gp)
        gxf = UsdGeom.Xformable(group.GetPrim())

        # translate
        gxf.AddTranslateOp().Set(Gf.Vec3d(float(self.position[0]), float(self.position[1]), float(self.position[2])))
        # orient (xyzw -> wxyz)
        q = np.array(self.orient_xyzw, dtype=float)
        n = np.linalg.norm(q)
        if n > 0 and abs(n - 1.0) > 1e-6:
            q /= n
        gxf.AddOrientOp().Set(Gf.Quatf(float(q[3]), float(q[0]), float(q[1]), float(q[2])))

        # 치수/위치(로컬)
        W, L = self.width, self.length
        T, H = self.wall_thickness, self.wall_height
        M = self.wall_margin
        base_z = self.wall_base_z

        # 각 벽의 (로컬) 중심과 스케일
        walls = [
            # name,             center(x,y,z),                        size(x,y,z)
            ("WestWall",  (-T/2 - M,    L/2, base_z + H/2),          (T,        L + 2*M, H)),
            ("EastWall",  (W + T/2 + M, L/2, base_z + H/2),          (T,        L + 2*M, H)),
            ("SouthWall", (W/2, -T/2 - M,  base_z + H/2),            (W + 2*M,  T,       H)),
            ("NorthWall", (W/2, L + T/2 + M, base_z + H/2),          (W + 2*M,  T,       H)),
        ]

        phys_mat = _ensure_physx_material(stage)

        for name, center, size in walls:
            cube_path = f"{group_path}/{name}"
            cube = UsdGeom.Cube.Define(stage, Sdf.Path(cube_path))
            cube.CreateSizeAttr(1.0)  # 기준 1로 두고 스케일로 치수 적용

            # 로컬 변환(그룹 기준): 위치 + 스케일
            cxf = UsdGeom.Xformable(cube.GetPrim())
            cxf.AddTranslateOp().Set(Gf.Vec3d(float(center[0]), float(center[1]), float(center[2])))
            cxf.AddScaleOp().Set(Gf.Vec3f(float(size[0]), float(size[1]), float(size[2])))

            # 충돌(정적 벽)
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
            PhysxSchema.PhysxCollisionAPI.Apply(cube.GetPrim())
            # 물리 머티리얼 바인딩(버전 호환)
            if hasattr(UsdPhysics, "MaterialBindingAPI"):
                try:
                    bind_api = UsdPhysics.MaterialBindingAPI.Apply(cube.GetPrim())
                    if hasattr(bind_api, "Bind"):
                        bind_api.Bind(phys_mat)
                    else:
                        bind_api.CreateMaterialRel().SetTargets([phys_mat.GetPath()])
                except Exception:
                    cube.GetPrim().CreateRelationship("physics:material:binding", True)\
                                  .SetTargets([phys_mat.GetPath()])
            else:
                cube.GetPrim().CreateRelationship("physics:material:binding", True)\
                              .SetTargets([phys_mat.GetPath()])

            # 회색 머티리얼
            bind_color_material(stage, cube.GetPrim(), color=self.wall_color, roughness=0.95, metallic=0.0)

    def build_and_place(self):
        # 높이장 생성
        if self.method == "bumps":
            h_m = self._build_bumps_height()
        elif self.method == "fbm":
            h_m = self._build_fbm_height()
        else:
            raise ValueError("Terrain.method must be 'bumps' or 'fbm'")

        height_raw = _to_raw(h_m, self.vs)
        vertices, triangles = convert_heightfield_to_trimesh(height_raw,
                                                             horizontal_scale=self.hs,
                                                             vertical_scale=self.vs)
        mesh = add_or_replace_terrain_mesh(self.stage, vertices, triangles,
                                           self.position, self.orient_xyzw, self.prim_path)
        bind_color_material(self.stage, mesh.GetPrim(),
                            color=self.color, roughness=self.roughness, metallic=self.metallic)

        # 사면 벽 생성
        self._create_walls()

        print(f"[Terrain] method={self.method} rows={self.rows} cols={self.cols} hs={self.hs} vs={self.vs}")

# -----------------------
# 실행
# -----------------------
def main():
    stage = omni.usd.get_context().get_stage()
    t = UnstructuredTerrainWorld(stage)
    t.build_and_place()

if __name__ == "__main__":
    main()

