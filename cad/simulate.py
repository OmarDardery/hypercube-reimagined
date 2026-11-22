"""Virtual gantry: assemble everything, route both CoreXY belts with true tangent
geometry, sweep the XY workspace and check collisions / belt alignment.
Usage:  python3 cad/simulate.py   -> sim/report.json + sim/*.stl scenes"""
import json, math, os, sys
import numpy as np
import manifold3d as m3
sys.path.insert(0, os.path.dirname(__file__))
from gantry import (Params, box, zcyl, union, to_trimesh, mirror_x, build_joiner,
                    build_carrier, load_original_carrier, idlers)

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SIM = os.path.join(ROOT, "sim")
SRC = os.path.join(ROOT, "cad", "source", "Hypercube300V2_M12_Sensorless_Carrier.stl")
CONTACT_TOL = 1.0   # mm^3: bolted faces that merely touch


# ---------------------------------------------------------------- static frame + hardware
def frame(P):
    S, Fy = P.Y_RAIL_SPACING, P.FRAME_Y_INNER
    y0 = (Fy - P.Y_RAIL_LEN) / 2
    parts = {
        "frame_Y_left": box(-10, 10, 0, Fy, -20, 0),
        "frame_Y_right": box(S - 10, S + 10, 0, Fy, -20, 0),
        "frame_X_front": box(-10, S + 10, -20, 0, -20, 0),
        "frame_X_back": box(-10, S + 10, Fy, Fy + 20, -20, 0),
        "rail_Y_left": box(-6, 6, y0, y0 + P.Y_RAIL_LEN, 0, P.MGN12_RAIL_H),
        "rail_Y_right": box(S - 6, S + 6, y0, y0 + P.Y_RAIL_LEN, 0, P.MGN12_RAIL_H),
    }
    return parts, (y0, y0 + P.Y_RAIL_LEN)


def corner_devices(P):
    """Stationary pulleys, placed so their strands line up with the original joiner."""
    S, Fy = P.Y_RAIL_SPACING, P.FRAME_Y_INNER
    xc = P.pass_x + P.IDLER_PITCH_R            # 21.0 (+ROD_DX)
    yF, yB = 16.0, Fy - 16.0
    I = idlers(P, "left")
    lvA = I[0]["z"]; lvB = I[1]["z"]             # belt1 runs on left-A level, belt2 on left-B level
    return dict(xc=xc, yF=yF, yB=yB, lv1=lvA, lv2=lvB, r=P.IDLER_PITCH_R)


# ---------------------------------------------------------------- belt routing
def tangent(c1, r1, s1, c2, r2, s2):
    """Belt leaves circle1 (centre c1, radius r1, winding s1=+1 ccw/-1 cw) and
    arrives on circle2.  Returns (p_leave, p_arrive)."""
    c1, c2 = np.asarray(c1, float), np.asarray(c2, float)
    d = c2 - c1; L = np.linalg.norm(d); u = d / L
    # signed radii: for travel direction t, contact point = c + s*r*perp(t) rotated
    a, b = s1 * r1, s2 * r2
    k = (b - a) / L
    h = math.sqrt(max(0.0, 1 - k * k))
    # unit normal n with n.(c2-c1) = (a-b)... choose n so that point = c - n*sr
    perp = np.array([-u[1], u[0]])
    n = k * u + h * perp
    # select branch giving travel direction consistent with winding
    for nn in (n, k * u - h * perp):
        p1 = c1 - nn * a; p2 = c2 - nn * b
        t = p2 - p1
        if np.linalg.norm(t) < 1e-9:
            continue
        t /= np.linalg.norm(t)
        # ccw winding: travel direction = rotate(radial,+90)
        ok1 = r1 == 0 or np.dot(np.array([-(p1 - c1)[1], (p1 - c1)[0]]) * s1, t) > 0
        ok2 = r2 == 0 or np.dot(np.array([-(p2 - c2)[1], (p2 - c2)[0]]) * s2, t) > 0
        if ok1 and ok2:
            return p1, p2
    raise RuntimeError("no tangent")


def route(nodes):
    """nodes: list of (centre(x,y), r, winding).  First/last are clamp points (r=0).
    Returns straight segments and arcs + total length."""
    segs, arcs, L = [], [], 0.0
    pts_in = [None] * len(nodes)
    pts_out = [None] * len(nodes)
    for i in range(len(nodes) - 1):
        (c1, r1, s1), (c2, r2, s2) = nodes[i], nodes[i + 1]
        p1, p2 = tangent(c1, r1, s1, c2, r2, s2)
        pts_out[i] = p1; pts_in[i + 1] = p2
        segs.append((p1, p2)); L += np.linalg.norm(p2 - p1)
    for i in range(1, len(nodes) - 1):
        c, r, s = nodes[i]
        a0 = math.atan2(*(pts_in[i] - c)[::-1]); a1 = math.atan2(*(pts_out[i] - c)[::-1])
        da = (a1 - a0) * s
        while da < 0: da += 2 * math.pi
        while da >= 2 * math.pi - 1e-9: da -= 2 * math.pi
        arcs.append((np.asarray(c), r, a0, da * s)); L += r * da
    return segs, arcs, L


def belt_nodes(P, Xc, Yg, cd, ci):
    S = P.Y_RAIL_SPACING
    IL, IR = idlers(P, "left"), idlers(P, "right")
    hw = P.CARRIER_HALF_W
    r = P.IDLER_PITCH_R
    A_L, B_L = IL; A_R, B_R = IR
    # belt 1 : carrier(left end) -> left A -> FL -> FR -> (through right joiner) -> BR -> right B -> carrier(right end)
    b1 = [((Xc - hw, Yg + A_L["depart_y"]), 0, 0),
          ((A_L["x"], Yg + A_L["y"]), r, +1),
          ((cd["xc"], cd["yF"]), r, +1),
          ((S - cd["xc"], cd["yF"]), r, +1),
          ((S - cd["xc"], cd["yB"]), r, +1),
          ((S - B_R["x"], Yg + B_R["y"]), r, -1),
          ((Xc + hw, Yg + B_R["depart_y"]), 0, 0)]
    # belt 2 : mirror image
    b2 = [((Xc + hw, Yg + A_R["depart_y"]), 0, 0),
          ((S - A_R["x"], Yg + A_R["y"]), r, -1),
          ((S - cd["xc"], cd["yF"]), r, -1),
          ((cd["xc"], cd["yF"]), r, -1),
          ((cd["xc"], cd["yB"]), r, -1),
          ((B_L["x"], Yg + B_L["y"]), r, +1),
          ((Xc - hw, Yg + B_L["depart_y"]), 0, 0)]
    return [(b1, A_L["z"], "belt1 (left-A level)"), (b2, B_L["z"], "belt2 (left-B level)")]


def seg_box(p1, p2, z, w=6.0, t=1.38, trim=0.3):
    p1, p2 = np.asarray(p1, float), np.asarray(p2, float)
    d = p2 - p1; L = np.linalg.norm(d)
    if L < 2 * trim + 0.1:
        return None
    u = d / L; n = np.array([-u[1], u[0]])
    q1, q2 = p1 + u * trim, p2 - u * trim
    pts = []
    for q in (q1, q2):
        for sn in (-t / 2, t / 2):
            for sz in (-w / 2, w / 2):
                pts.append([*(q + n * sn), z + sz])
    return m3.Manifold.hull_points(pts)


def arc_boxes(arc, z, n=8):
    c, r, a0, da = arc
    out = []
    for k in range(n):
        t0, t1 = a0 + da * k / n, a0 + da * (k + 1) / n
        p1 = c + r * np.array([math.cos(t0), math.sin(t0)])
        p2 = c + r * np.array([math.cos(t1), math.sin(t1)])
        b = seg_box(p1, p2, z, trim=0.0)
        if b: out.append(b)
    return out


# ---------------------------------------------------------------- hotend / probe models
def hotend(P):
    top = P.ROD_DZ + 4.75 + 8.2            # top of E3D-V6 style hot end (from groove mount)
    x, y = 0.0, -23.15
    parts = [zcyl(x, y, top - 3.7, top, 16), zcyl(x, y, top - 9.7, top - 3.7, 12),
             zcyl(x, y, top - 12.7, top - 9.7, 16), zcyl(x, y, top - 42.7, top - 12.7, 22.3),
             zcyl(x, y, top - 46.7, top - 42.7, 4)]
    hb = box(x - 8, x + 15, y - 8, y + 8, top - 58.2, top - 46.7)   # heater block
    nz = zcyl(x, y, top - 62.3, top - 58.2, 6, 24)
    nozzle_z = top - 62.3
    return union(parts + [hb, nz]), nozzle_z


def probe(P, nozzle_z):
    x, y = -23.87, -24.75
    return zcyl(x, y, nozzle_z + 1.0, nozzle_z + 61.0, 12), (x, y)


# ---------------------------------------------------------------- assembly
class Gantry:
    def __init__(self, P=None):
        self.P = P = P or Params()
        self.JL, _ = build_joiner(P, "left")
        JRl, _ = build_joiner(P, "right")
        self.JR = mirror_x(JRl)
        self.C, self.ci = build_carrier(P, load_original_carrier(SRC))
        self.H, self.nozzle_z = hotend(P)
        self.PR, self.probe_xy = probe(P, self.nozzle_z)
        self.static, self.rail_y = frame(P)
        self.cd = corner_devices(P)
        S = P.Y_RAIL_SPACING
        self.mgn12h = box(-P.MGN12H_W / 2, P.MGN12H_W / 2, -P.MGN12H_L / 2, P.MGN12H_L / 2, P.MGN12_H1, P.MGN12_H)
        self.beam = box(P.BEAM_END_X, S - P.BEAM_END_X, P.BEAM_Y - 10, P.BEAM_Y + 10, P.beam_bot, P.BEAM_TOP)
        self.rail9_x = ((S - P.X_RAIL_LEN) / 2, (S + P.X_RAIL_LEN) / 2)
        bf = P.BEAM_Y - 10.0                      # beam front face; MGN9 rail lies on it sideways
        self.rail9 = box(self.rail9_x[0], self.rail9_x[1], bf - P.MGN9_RAIL_H, bf,
                         P.x_rail_zc - P.MGN9_RAIL_W / 2, P.x_rail_zc + P.MGN9_RAIL_W / 2)
        self.mgn9h = box(-P.MGN9H_L / 2, P.MGN9H_L / 2, P.x_carriage_face, bf - P.MGN9_H1,
                         P.x_rail_zc - P.MGN9H_W / 2, P.x_rail_zc + P.MGN9H_W / 2)
        # stationary pulleys as 18 mm discs, 8 mm tall, at their belt level
        cd = self.cd; xs = [cd["xc"], S - cd["xc"]]
        devs = []
        for x in xs:
            for z in (cd["lv1"], cd["lv2"]):
                devs.append(zcyl(x, cd["yF"], z - 4, z + 4, 18))
        devs.append(zcyl(S - cd["xc"], cd["yB"], cd["lv1"] - 4, cd["lv1"] + 4, 18))
        devs.append(zcyl(cd["xc"], cd["yB"], cd["lv2"] - 4, cd["lv2"] + 4, 18))
        self.static["corner_pulleys"] = union(devs)

    def pose(self, Xc, Yg):
        """Xc = carrier (hot-end axis) X, Yg = gantry Y (joiner centre)."""
        S = self.P.Y_RAIL_SPACING
        P_ = self.P
        T = lambda M, x, y: M.translate([x, y, 0])
        moving = {
            "joiner_left": T(self.JL, 0, Yg),
            "joiner_right": T(self.JR, S, Yg),
            "mgn12h_left": T(self.mgn12h, 0, Yg),
            "mgn12h_right": T(self.mgn12h, S, Yg),
            "beam_2020": T(self.beam, 0, Yg),
            "rail_X_mgn9": T(self.rail9, 0, Yg),
            "mgn9h": T(self.mgn9h, Xc, Yg),
            "carrier": T(self.C, Xc, Yg),
            "hotend": T(self.H, Xc, Yg),
            "probe": T(self.PR, Xc, Yg),
            # space claim for the part-cooling blower on the rear bracket (5015 class)
            "blower_envelope": T(box(-25, 25, 4.5, 50, 0, P_.beam_bot - 1.5), Xc, Yg),
        }
        for side in ("left", "right"):
            discs = []
            for i in idlers(self.P, side):
                d = zcyl(i["x"], i["y"], i["z"] - 3.8, i["z"] + 3.8, 18.0)
                discs.append(d)
            D = union(discs)
            moving[f"idlers_{side}"] = T(D, 0, Yg) if side == "left" else T(mirror_x(D), S, Yg)
        return moving

    def belts(self, Xc, Yg):
        out = []
        for nodes, z, name in belt_nodes(self.P, Xc, Yg, self.cd, self.ci):
            segs, arcs, L = route(nodes)
            out.append(dict(name=name, z=z, segs=segs, arcs=arcs, length=L))
        return out


# pairs that are SUPPOSED to touch / interpenetrate in the simplified models
ALLOWED = {
    frozenset(["mgn12h_left", "rail_Y_left"]), frozenset(["mgn12h_right", "rail_Y_right"]),
    frozenset(["mgn9h", "rail_X_mgn9"]),
    frozenset(["probe", "carrier"]),          # probe passes through its ring
    frozenset(["hotend", "carrier"]),         # groove mount
    frozenset(["blower_envelope", "carrier"]),  # blower bolts to the rear bracket
}


def collisions(parts, names=None, only_with=None):
    keys = list(parts)
    res = []
    for i in range(len(keys)):
        for j in range(i + 1, len(keys)):
            a, b = keys[i], keys[j]
            if frozenset([a, b]) in ALLOWED:
                continue
            if only_with and not (a in only_with or b in only_with):
                continue
            A, B = parts[a], parts[b]
            ba, bb = A.bounding_box(), B.bounding_box()
            if any(ba[k] > bb[k + 3] or bb[k] > ba[k + 3] for k in range(3)):
                continue
            v = (A ^ B).volume()
            if v > CONTACT_TOL:
                res.append((a, b, round(v, 2)))
    return res


def belt_clearance(g, Xc, Yg, parts):
    """Belt straight runs + idler wraps vs every rigid part (pocket voids must contain them)."""
    hits = []
    for b in g.belts(Xc, Yg):
        boxes = [seg_box(p1, p2, b["z"]) for p1, p2 in b["segs"]]
        for a in b["arcs"]:
            boxes += arc_boxes(a, b["z"])
        B = union([x for x in boxes if x is not None])
        for k, M in parts.items():
            if k in ("corner_pulleys", "idlers_left", "idlers_right"):
                continue
            v = (B ^ M).volume()
            if v > 0.05:
                hits.append((b["name"], k, round(v, 3)))
    return hits


def clamp_check(g, Xc, Yg):
    """Model the belt backing inside each carrier clamp channel and make sure it fits."""
    hits = []
    C = g.C.translate([Xc, Yg, 0])
    hw = g.P.CARRIER_HALF_W
    for bb in g.ci["belts"]:
        s = -1 if bb["side"] == "left" else 1
        td = -1 if bb["y"] < 0 else 1
        ya, yb = sorted([bb["y"] - 0.75 * td, bb["y"] - 0.2 * td])
        x0, x1 = sorted([s * hw, s * (hw - g.P.CLAMP_LEN + 1)])
        M = box(Xc + x0, Xc + x1, Yg + ya, Yg + yb, bb["z"] - 3, bb["z"] + 3)
        v = (M ^ C).volume()
        hits.append((f"{bb['side']}-{bb['level']}", round(v, 3)))
    return hits


def travel_limits(g):
    """Binary-search X and Y extremes where no collision occurs."""
    P = g.P; S = P.Y_RAIL_SPACING
    Ymid = P.FRAME_Y_INNER / 2

    def x_ok(Xc):
        parts = dict(g.pose(Xc, Ymid)); parts.update(g.static)
        r0, r1 = g.rail9_x
        if Xc - P.MGN9H_L / 2 < r0 or Xc + P.MGN9H_L / 2 > r1:
            return False
        return not collisions(parts, only_with={"carrier", "hotend", "probe", "mgn9h", "blower_envelope"})

    def y_ok(Yg, Xc):
        parts = dict(g.pose(Xc, Yg)); parts.update(g.static)
        y0, y1 = g.rail_y
        on_rail = (Yg - P.MGN12H_L / 2 >= y0) and (Yg + P.MGN12H_L / 2 <= y1)
        return on_rail and not collisions(parts) and not belt_clearance(g, Xc, Yg, parts)

    def search(ok, lo, hi):   # ok(lo)=True, ok(hi)=False
        for _ in range(22):
            mid = (lo + hi) / 2
            if ok(mid): lo = mid
            else: hi = mid
        return lo

    xmin = search(x_ok, S / 2, 0.0)
    xmax = search(x_ok, S / 2, S)
    xs = [xmin, S / 2, xmax]
    ymin = max(search(lambda y: y_ok(y, x), Ymid, -60.0) for x in xs)
    ymax = min(search(lambda y: y_ok(y, x), Ymid, P.FRAME_Y_INNER + 60) for x in xs)
    return xmin, xmax, ymin, ymax


def run():
    os.makedirs(SIM, exist_ok=True)
    g = Gantry(); P = g.P; S = P.Y_RAIL_SPACING
    rep = {"params": {k: getattr(P, k) for k in P.__dataclass_fields__}}
    xmin, xmax, ymin, ymax = travel_limits(g)
    rep["carrier_x_range"] = [xmin, xmax]; rep["gantry_y_range"] = [ymin, ymax]
    rep["x_travel"] = xmax - xmin; rep["y_travel"] = ymax - ymin
    rep["nozzle_z"] = g.nozzle_z
    rep["nozzle_y_from_mgn9h_face"] = -23.15 - P.x_carriage_face
    rep["probe_offset_from_nozzle"] = [g.probe_xy[0], g.probe_xy[1] + 23.15]

    # sweep a grid (margin 0.5 mm inside limits)
    xs = np.linspace(xmin + 0.5, xmax - 0.5, 5)
    ys = np.linspace(ymin + 0.5, ymax - 0.5, 5)
    poses, lens = [], {}
    for X in xs:
        for Y in ys:
            parts = dict(g.pose(X, Y)); parts.update(g.static)
            col = collisions(parts)
            bh = belt_clearance(g, X, Y, parts)
            cc = clamp_check(g, X, Y)
            bl = g.belts(X, Y)
            for b in bl:
                lens.setdefault(b["name"], []).append(b["length"])
            # strand angles from the X / Y axes
            ang = []
            for b in bl:
                for p1, p2 in b["segs"]:
                    d = np.asarray(p2) - p1
                    a = math.degrees(math.atan2(abs(d[1]), abs(d[0])))
                    ang.append(min(a, 90 - a))
            poses.append(dict(X=float(X), Y=float(Y), part_collisions=col, belt_hits=bh,
                              clamp_fit=cc, max_strand_angle_deg=max(ang)))
    rep["poses"] = poses
    rep["belt_length_variation_mm"] = {k: float(max(v) - min(v)) for k, v in lens.items()}
    rep["belt_length_mm"] = {k: float(np.mean(v)) for k, v in lens.items()}
    rep["all_clear"] = all(not p["part_collisions"] and not p["belt_hits"] and
                           all(v < 0.05 for _, v in p["clamp_fit"]) for p in poses)
    # CoreXY kinematics check: motor-side length change vs dX, dY
    def seg_len_to_motor(belt, X, Y):
        b = [bb for bb in g.belts(X, Y) if bb["name"] == belt][0]
        segs, arcs = b["segs"], b["arcs"]
        return sum(np.linalg.norm(np.asarray(q) - p) for p, q in segs[:2]) + arcs[0][1] * abs(arcs[0][3])
    X0, Y0 = (xmin + xmax) / 2, (ymin + ymax) / 2
    kin = {}
    for name in ("belt1 (left-A level)", "belt2 (left-B level)"):
        l0 = seg_len_to_motor(name, X0, Y0)
        kin[name] = dict(dL_per_dX=seg_len_to_motor(name, X0 + 10, Y0) - l0,
                         dL_per_dY=seg_len_to_motor(name, X0, Y0 + 10) - l0)
    rep["corexy_kinematics_per_10mm"] = kin
    with open(os.path.join(SIM, "report.json"), "w") as f:
        json.dump(rep, f, indent=1, default=float)
    return g, rep


if __name__ == "__main__":
    g, rep = run()
    print(json.dumps({k: v for k, v in rep.items() if k not in ("poses", "params")}, indent=1, default=float))
    bad = [p for p in rep["poses"] if p["part_collisions"] or p["belt_hits"]]
    print("poses with issues:", len(bad))
    for p in bad[:6]:
        print(p)
    print("clamp fit sample:", rep["poses"][0]["clamp_fit"])


def export_scene(g, Xc, Yg, out_dir):
    """Write one STL per component group (for rendering)."""
    os.makedirs(out_dir, exist_ok=True)
    parts = dict(g.pose(Xc, Yg)); parts.update(g.static)
    for k, M in parts.items():
        to_trimesh(M).export(os.path.join(out_dir, f"{k}.stl"))
    for b in g.belts(Xc, Yg):
        boxes = [seg_box(p1, p2, b["z"], trim=0.0) for p1, p2 in b["segs"]]
        for a in b["arcs"]:
            boxes += arc_boxes(a, b["z"], 12)
        tag = "belt1" if b["name"].startswith("belt1") else "belt2"
        to_trimesh(union([x for x in boxes if x is not None])).export(os.path.join(out_dir, f"{tag}.stl"))
    return list(parts) + ["belt1", "belt2"]
