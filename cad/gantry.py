"""
Hypercube 300 V2 -> linear-rail XY conversion (parametric).

Coordinate system (installed, "gantry frame"), millimetres:
  X  : along the X beam, + = toward the bed centre (inboard) for the LEFT joiner
  Y  : along the Y rails, - = front of printer, + = back
  Z  : up; Z = 0 is the TOP face of the 2020 that carries the MGN12 Y rail
Left Y rail centre is X = 0, right Y rail centre is X = P.Y_RAIL_SPACING.

All belt-path geometry is copied from the original XY_Joiner_10mm-X_LM8LUU:
  idler A : 21 mm inboard of the old Y-rod axis, 7 mm toward the front
  idler B : 34 mm inboard, 7 mm toward the back
  the two belt levels sit +-7.25 mm around the old Y-rod axis height
  one belt run passes straight through each joiner 14.63 mm inboard.
Keeping those points fixed relative to the frame means the motors and corner
pulleys do not move.
"""
from dataclasses import dataclass
import numpy as np
import manifold3d as m3
import trimesh

SEG = 96  # circle resolution


# --------------------------------------------------------------------------- params
@dataclass
class Params:
    # --- measured / chosen by user (edit these, re-run build.py) ---
    ROD_DZ: float = 40.0          # old Y-rod centre height above the 2020 top face
    ROD_DX: float = 0.0           # old Y-rod centre offset from 2020 centreline (+ = inboard)
    LEFT_A_LEVEL: str = "upper"   # belt level of the LEFT joiner's front/outer idler (A)
    Y_RAIL_SPACING: float = 440.0 # left-to-right Y rail centre distance (HC300 estimate)
    FRAME_Y_INNER: float = 440.0  # inside length of the frame along Y (sim only)
    Y_RAIL_LEN: float = 400.0     # MGN12 rail length (sim only)

    # --- original belt geometry (from the original joiner, do not change) ---
    LEVEL_HALF_GAP: float = 7.25
    IDLER_A: tuple = (21.0, -7.0)
    IDLER_B: tuple = (34.0, 7.0)
    IDLER_PITCH_R: float = 6.37   # 20T GT2 idler pitch radius
    POCKET_D: float = 20.5        # original pocket 20
    POCKET_H: float = 8.0         # original pocket 8
    BELT_W: float = 6.0
    BELT_T: float = 1.38

    # --- hardware (HIWIN standard) ---
    MGN12H_W: float = 27.0; MGN12H_L: float = 45.4; MGN12_H: float = 13.0; MGN12_H1: float = 3.0
    MGN12_RAIL_W: float = 12.0; MGN12_RAIL_H: float = 8.0; MGN12H_B: float = 20.0; MGN12H_C: float = 20.0
    MGN9H_W: float = 20.0; MGN9H_L: float = 39.9; MGN9_H: float = 10.0; MGN9_H1: float = 2.0
    MGN9_RAIL_W: float = 9.0; MGN9_RAIL_H: float = 6.5; MGN9H_B: float = 15.0; MGN9H_C: float = 16.0

    # --- X beam (2020) ---
    BEAM_Y: float = 15.0          # beam centre Y: sits ON TOP of the joiners, offset back so the
                                  # belts pass under its front edge into the carrier
    BEAM_END_X: float = -5.0      # beam end X (relative to Y-rail centre); length = spacing - 2*this
    SEAT_T: float = 5.0           # joiner roof / beam seat thickness above the upper idler pocket
    CHEEK_T: float = 5.0          # joiner cheeks clamping the beam front/back faces
    X_RAIL_LEN: float = 350.0     # MGN9 rail length, centred on the beam

    # --- carrier ---
    CARRIER_HALF_W: float = 27.0
    CLAMP_LEN: float = 16.0

    @property
    def zL(self): return self.ROD_DZ - self.LEVEL_HALF_GAP
    @property
    def zU(self): return self.ROD_DZ + self.LEVEL_HALF_GAP
    @property
    def beam_bot(self): return self.zU + self.POCKET_H / 2 + self.SEAT_T
    @property
    def BEAM_TOP(self): return self.beam_bot + 20.0
    @property
    def x_rail_zc(self): return self.beam_bot + 10.0          # MGN9 rail centred on the beam's front face
    @property
    def x_carriage_face(self): return self.BEAM_Y - 10.0 - self.MGN9_H   # Y of the MGN9H front (mounting) face
    @property
    def pass_x(self): return self.IDLER_A[0] + self.ROD_DX - self.IDLER_PITCH_R


# --------------------------------------------------------------------------- helpers
def box(x0, x1, y0, y1, z0, z1):
    return m3.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def zcyl(x, y, z0, z1, d, seg=SEG):
    return m3.Manifold.cylinder(z1 - z0, d / 2, d / 2, seg).translate([x, y, z0])


def ycyl(x, z, y0, y1, d, seg=SEG):
    c = m3.Manifold.cylinder(y1 - y0, d / 2, d / 2, seg)          # along +Z
    return c.rotate([-90, 0, 0]).translate([x, y0, z])           # now along +Y


def xcyl(y, z, x0, x1, d, seg=SEG):
    c = m3.Manifold.cylinder(x1 - x0, d / 2, d / 2, seg)
    return c.rotate([0, 90, 0]).translate([x0, y, z])


def union(parts):
    return m3.Manifold.batch_boolean(parts, m3.OpType.Add)


def to_trimesh(M):
    mesh = M.to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts), process=False)


def from_trimesh(t):
    return m3.Manifold(m3.Mesh(vert_properties=np.asarray(t.vertices, np.float32),
                               tri_verts=np.asarray(t.faces, np.uint32)))


def mirror_x(M, about=0.0):
    return M.mirror([1, 0, 0]).translate([2 * about, 0, 0])


# --------------------------------------------------------------------------- idler layout
def idlers(P, side):
    """Return list of dicts for the joiner idlers in LEFT-joiner local coords.
    side = 'left' or 'right' (right part is built in left coords then mirrored)."""
    a_up = (P.LEFT_A_LEVEL == "upper")
    if side == "right":
        a_up = not a_up
    zA = P.zU if a_up else P.zL
    zB = P.zL if a_up else P.zU
    r = P.IDLER_PITCH_R
    ax, ay = P.IDLER_A[0] + P.ROD_DX, P.IDLER_A[1]
    bx, by = P.IDLER_B[0] + P.ROD_DX, P.IDLER_B[1]
    return [
        dict(name="A", x=ax, y=ay, z=zA, approach="front", strand_x=ax - r, depart_y=ay + r),
        dict(name="B", x=bx, y=by, z=zB, approach="back", strand_x=bx - r, depart_y=by - r),
    ]


# --------------------------------------------------------------------------- joiner
def build_joiner(P, side="left"):
    """XY joiner in LEFT-local installed coords (rail centre X=0, joiner centre Y=0).
    For side='right' the idler levels are swapped; caller mirrors it."""
    I = idlers(P, side)
    A, B = I
    up = A if A["z"] > B["z"] else B
    lo = B if up is A else A
    hb = P.POCKET_H / 2
    car_top = P.MGN12_H
    base_top = car_top + 5.0
    zl0, zl1 = lo["z"] - hb, lo["z"] + hb
    zu0, zu1 = up["z"] - hb, up["z"] + hb
    yF, yR = -P.MGN12H_L / 2, P.MGN12H_L / 2
    xo = -P.MGN12H_W / 2
    by0, by1 = P.BEAM_Y - 10, P.BEAM_Y + 10
    xmax = max(A["x"], B["x"]) + 8.0   # like the original: outer pocket breaks through the inboard face
    ymid = 19.5

    seat_top = P.beam_bot
    yback = by1 + P.CHEEK_T                                   # back face of the rear cheek
    ftop = P.BEAM_TOP - 4.0                                   # cheek top (below beam top edge radius)

    def pillar_free(i):
        # solid beside the idler where no belt runs on that level
        if i["name"] == "A":   # wraps outboard->back, so the back side is free
            return box(i["x"] - 10, i["x"] + 7, i["y"] + 11, ymid, 0, 1)
        else:                  # wraps outboard->front, so the front side (below depart line) is free
            return box(i["x"] - 9, xmax, -ymid, i["y"] - 11, 0, 1)

    def band(i, z0, z1):
        pf = pillar_free(i)
        bb = pf.bounding_box()
        pf = box(bb[0], bb[3], bb[1], bb[4], z0, z1)
        pout = box(xo, 10.5, -ymid, ymid, z0, z1)
        return pout + pf

    solid = union([
        box(xo, -xo, yF, yR, car_top, base_top),                       # carriage pad
        box(xo, 14.0, yF, yR, base_top, zl0),                          # deck over carriage
        box(14.0, xmax, -ymid, ymid, zl0 - 8.0, zl0),                  # floor of the lower idler
        band(lo, zl0, zl1),                                            # lower belt level pillars
        box(xo, xmax, -ymid, ymid, zl1, zu0),                          # middle plate
        box(xo, xmax, -ymid, ymid, zu0, zu1),                          # upper level: full, belts cut out below
        box(xo, xmax, -ymid, yback, zu1, seat_top),                    # roof = beam seat
        box(xo, xmax, by0 - P.CHEEK_T, by0, seat_top - 0.01, ftop),    # front cheek
        box(xo, xmax, by1, yback, seat_top - 0.01, ftop),              # rear cheek
    ])

    cut = []
    for i in I:
        z0, z1 = i["z"] - hb, i["z"] + hb
        R = P.POCKET_D / 2
        cut.append(zcyl(i["x"], i["y"], z0, z1, P.POCKET_D))
        # idler insertion window, also clears the incoming belt run
        if i["approach"] == "front":
            cut.append(box(i["x"] - R, i["x"] + R, -200, i["y"], z0, z1))
        else:
            cut.append(box(i["x"] - R, i["x"] + R, i["y"], 200, z0, z1))
        # outgoing X run toward the carrier
        cut.append(box(i["x"], 400, i["depart_y"] - 1.5, i["depart_y"] + 1.5, i["z"] - 3.75, i["z"] + 3.75))
    # straight-through belt run on idler B's level (this side's other strand)
    zb = B["z"]
    cut.append(box(P.pass_x - 1.6, P.pass_x + 1.6, -200, 200, zb - 3.75, zb + 3.75))

    # idler bolts M3 (heads recessed so the beam sits flat on the seat)
    cut.append(zcyl(up["x"], up["y"], zu0 - 0.01, seat_top + 1, 3.3))
    cut.append(zcyl(up["x"], up["y"], seat_top - 3.5, seat_top + 1, 6.2))
    cut.append(zcyl(up["x"], up["y"], zl1 - 0.01, zu0 + 0.01, 2.8))
    cut.append(zcyl(lo["x"], lo["y"], zl1 - 0.01, zu0 + 0.01, 3.3))
    cut.append(zcyl(lo["x"], lo["y"], zl0 - 8, zl0 + 0.01, 2.8))
    cut.append(zcyl(lo["x"], lo["y"], zu0, seat_top + 1, 6.5))       # driver / head access

    # MGN12H screws: M3, 20x20, counterbored to the seat (fit before the beam)
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = sx * P.MGN12H_C / 2, sy * P.MGN12H_B / 2
            cut.append(zcyl(x, y, car_top - 1, base_top + 0.01, 3.4))
            cut.append(zcyl(x, y, base_top, seat_top + 1, 6.5))

    # beam cheek bolts: M5 into T-nuts in the beam's front & back slots
    zc = P.beam_bot + 10
    for x in (xo + 8.0, xmax - 8.0):
        cut.append(ycyl(x, zc, by0 - 10, yback + 10, 5.5))
    # lightening: hollow the outboard pillar between the screw shafts
    cut.append(box(-8.5, 6.0, -6.0, 6.0, base_top + 3, zl1 - 1.5))

    J = solid - union(cut)
    return J, dict(idlers=I, roof_top=seat_top, base_top=base_top, xmax=xmax)


# --------------------------------------------------------------------------- carrier
def load_original_carrier(path):
    t = trimesh.load(path)
    t.merge_vertices()
    return from_trimesh(t)


def orig_to_installed(P):
    """4x4 transform mapping the original carrier STL frame to carrier-local installed coords.
    Original: X along rods, Y = up (rod bores at Y=-30.5 / +21.0, Z=-11.05), +Z = front.
    Hot-end axis (X=-2.5, Z=12.1) -> local x=0; rod plane Z=-11.05 -> belt plane y=0;
    rod mid-height Y=-4.75 -> old Y-rod height ROD_DZ (belts stay at the same height)."""
    T = np.array([[1, 0, 0, 2.5],
                  [0, 0, -1, -11.05],
                  [0, 1, 0, P.ROD_DZ + 4.75],
                  [0, 0, 0, 1.0]])
    return T


def apply_T(M, T):
    return M.transform(T[:3, :].tolist())


def build_carrier(P, orig):
    """Carrier in carrier-local installed coords (x=0 at hot-end axis / MGN9H centre,
    y and z as gantry frame)."""
    T = orig_to_installed(P)
    mod = apply_T(orig, T)
    zL, zU = P.zL, P.zU
    hw = P.CARRIER_HALF_W
    top = zU + 3.5 + 4.25
    cf = P.x_carriage_face
    rzc = P.x_rail_zc
    # keep the whole original below the new clamp-block top (incl. the rear blower bracket),
    # drop the upper rod tube, and fill the lower rod bore solid ("flatten the rod holes")
    mod = mod ^ box(-200, 200, -200, 200, -200, top - 0.02)
    zrod = P.ROD_DZ - 25.75
    bore_fill = xcyl(0.0, zrod, -18.63, 18.63, 12.4)

    y_front = -8.0
    plate_t = 5.0
    body = union([
        mod,
        bore_fill,
        box(-hw, hw, y_front, 2.63, zL - 4.0, top),                      # belt-clamp block
        box(-17.47, 19.47, -7.47, y_front + 0.53, zL - 3.97, top - 0.03),  # tie module to block
        box(-hw, hw, cf - plate_t, cf, top - 0.03, P.BEAM_TOP),          # plate bolted to the MGN9H face
    ])
    cut = []
    belts = []
    for s in (-1, 1):
        x0, x1 = (s * hw, s * (hw - P.CLAMP_LEN))
        xa, xb = min(x0, x1), max(x0, x1)
        for lvl, z in (("upper", zU), ("lower", zL)):
            # which joiner feeds this end
            side = "left" if s < 0 else "right"
            I = idlers(P, side)
            i = [k for k in I if abs(k["z"] - z) < 1e-6][0]
            yb = i["depart_y"]
            if lvl == "upper":
                zc0, zc1 = z - 3.5, top + 1     # open on top: drop the belt in
            else:
                zc0, zc1 = zL - 5.0, z + 3.5
            cut.append(box(xa - 1, xb, yb - 0.8, yb + 0.8, zc0, zc1))
            belts.append(dict(side=side, level=lvl, y=yb, z=z, x_entry=s * hw))
    # MGN9H screws
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, z = sx * P.MGN9H_C / 2, rzc + sy * P.MGN9H_B / 2
            cut.append(ycyl(x, z, cf - plate_t - 1, cf + 1, 3.4))     # M3x8 from the front
    C = body - union(cut)
    # comb ridges (GT2 2 mm pitch) on the tooth-side wall of each channel
    ridges = []
    for b in belts:
        s = -1 if b["side"] == "left" else 1
        tooth_dir = -1 if b["y"] < 0 else 1
        yw = b["y"] + tooth_dir * 1.0          # 0.2 mm embedded into the wall
        yi = b["y"] + tooth_dir * 0.05
        ya, yb2 = min(yw, yi), max(yw, yi)
        z0, z1 = b["z"] - 3.3, b["z"] + 3.3
        for k in range(7):
            xc = s * (hw - 2.0 - 2.0 * k)
            ridges.append(box(xc - 0.5, xc + 0.5, ya, yb2, z0, z1))
    C = C + union(ridges)
    return C, dict(belts=belts, top=top, hotend_axis=(0.0, -23.15))


def extrusion_2020_x(x0, x1, yc, ztop):
    """2020 along X with 6 mm slots (visual + collision)."""
    b = box(x0, x1, yc - 10, yc + 10, ztop - 20, ztop)
    slots = []
    for (dy, dz) in ((0, 0), (0, -20), (-10, -10), (10, -10)):
        slots.append(box(x0 - 1, x1 + 1, yc + dy - 3, yc + dy + 3, ztop + dz - 3, ztop + dz + 3)
                     if dy == 0 else
                     box(x0 - 1, x1 + 1, yc + dy - 3, yc + dy + 3, ztop - 13, ztop - 7))
    return b - union(slots[:1] + slots[1:2] + slots[2:])
